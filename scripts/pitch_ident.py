"""上 pitch 静态辨识：PC 通过 var 下发目标角，log 采集角度与力矩，反推重力矩与摩擦。

python scripts/pitch_ident.py --port COM8 --low -50 --high -10 --points 9 --cycles 2
python scripts/pitch_ident.py --file pitch_ident_capture.csv

单位：CSV 时间 ms，角度 mdeg，力矩 mN*m。
"""
import argparse
import csv
import re
import time
from pathlib import Path

FIELDS = ["t_ms", "point", "phase", "target_mdeg", "angle_mdeg", "torque_mnm", "kp", "ki", "kd"]
ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
DATA = re.compile(r"(-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+)\s*$")

BAUD = 921600


def clean(raw):
    return ANSI.sub("", raw.decode("utf-8", errors="replace")).strip()


def capture(args):
    import serial

    with serial.Serial(args.port, args.baud, timeout=0.05, write_timeout=1) as ser, \
            open(args.save, "w", newline="", encoding="utf-8") as output:
        writer = csv.writer(output)
        writer.writerow(FIELDS)

        def send(command):
            ser.write((command + "\r\n").encode("ascii"))
            ser.flush()

        def receive():
            line = clean(ser.readline())
            match = DATA.search(line)
            if match:
                return line, [float(x) for x in match.groups()]
            return line, None

        def configure(command):
            send(command)
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline:
                line, _ = receive()
                if line.startswith(("ok", "log off: ok", "log on: ok", "log mode:")):
                    return
            raise RuntimeError(f"命令未确认: {command}")

        ser.reset_input_buffer()
        configure("log off")
        configure(f"var set pitch_up_kp {args.kp}")
        configure(f"var set pitch_up_ki {args.ki}")
        configure(f"var set pitch_up_kd {args.kd}")
        configure("log mode pitch_track on")
        configure("log on pitch_track 1")

        points = [args.low + (args.high - args.low) * i / max(1, args.points - 1) for i in range(args.points)]

        start = beat = time.monotonic()
        print(f"[i] 目标点 {points}", flush=True)
        try:
            for cycle in range(args.cycles):
                for point in points:
                    target = round(point, 2)
                    send(f"var set pitch_up_target {target}")
                    settle_until = time.monotonic() + args.settle_ms / 1000.0
                    while time.monotonic() < settle_until:
                        line, data = receive()
                        if data:
                            target_deg, angle_deg, torque_nm = data
                            t_ms = int((time.monotonic() - start) * 1000)
                            writer.writerow([t_ms, f"{point:.2f}", cycle, int(target_deg * 1000),
                                             int(angle_deg * 1000), int(torque_nm * 1000),
                                             args.kp, args.ki, args.kd])
                    output.flush()
                    now = time.monotonic()
                    if now - beat >= 1:
                        print(f"[hb] {now-start:.1f}s cycle={cycle} point={point:.1f}", flush=True)
                        beat = now
                points = points[::-1]
        finally:
            send("log off")
            output.flush()
            print(f"[i] 已保存 {args.save}", flush=True)


def analyze(path):
    import numpy as np

    with open(path, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise ValueError("CSV 为空")

    groups = {}
    for row in rows:
        key = (row["point"], row["phase"])
        groups.setdefault(key, []).append((float(row["angle_mdeg"]) / 1000.0,
                                           float(row["torque_mnm"]) / 1000.0))

    print(f"[i] 共 {len(rows)} 帧，{len(groups)} 组")
    print(f"{'point':>8} {'cycle':>5} {'angle_mean':>11} {'angle_std':>10} {'torque_mean':>12} {'n':>5}")
    for key in sorted(groups, key=lambda k: (float(k[0]), k[1])):
        values = np.asarray(groups[key])
        print(f"{float(key[0]):8.2f} {key[1]:>5} {values[:, 0].mean():11.3f} {values[:, 0].std():10.4f} "
              f"{values[:, 1].mean():12.4f} {len(values):5d}")

    # 往复两轮按实际角度插值配对：同一 θ 上比较正反向力矩，避免把重力矩差算进摩擦
    by_phase = {}
    for (point, phase), values in groups.items():
        by_phase.setdefault(int(phase), []).append(np.asarray(values))

    if len(by_phase) < 2:
        print("[!] 只有单个方向，无法分离摩擦；跳过配对")
        return

    # 每个方向按目标点排序，得到 (角度中位数, 力矩中位数) 序列
    profiles = {}
    for phase, chunks in by_phase.items():
        chunks.sort(key=lambda c: c[:, 0].mean())
        angles = np.asarray([c[:, 0].mean() for c in chunks])
        torques = np.asarray([c[:, 1].mean() for c in chunks])
        profiles[phase] = (angles, torques)

    phases = sorted(profiles)
    forward, backward = profiles[phases[0]], profiles[phases[1]]

    # 公共角度区间：两方向都覆盖到的部分
    lo = max(forward[0][0], backward[0][0])
    hi = min(forward[0][-1], backward[0][-1])
    if hi - lo < 1.0:
        print(f"[!] 正反向公共角度区间只有 {hi-lo:.2f}°，无法配对")
        return

    common = np.linspace(lo, hi, max(3, len(forward[0])))
    tau_f = np.interp(common, forward[0], forward[1])
    tau_b = np.interp(common, backward[0], backward[1])
    gravity = (tau_f + tau_b) / 2.0
    friction = np.abs(tau_f - tau_b) / 2.0

    theta = np.deg2rad(common)
    design = np.column_stack([np.sin(theta), np.cos(theta)])
    coeff, _, _, _ = np.linalg.lstsq(design, gravity, rcond=None)
    rms = float(np.sqrt(np.mean((design @ coeff - gravity) ** 2)))

    print(f"\n[i] 公共区间 {lo:.2f}° ~ {hi:.2f}°，配对 {len(common)} 点")
    print(f"{'angle':>8} {'tau_fwd':>10} {'tau_bwd':>10} {'gravity':>10} {'friction':>10}")
    for i in range(len(common)):
        print(f"{common[i]:8.2f} {tau_f[i]:10.4f} {tau_b[i]:10.4f} {gravity[i]:10.4f} {friction[i]:10.4f}")
    print(f"\n[fit] gravity = {coeff[0]:+.6f}*sin(theta) {coeff[1]:+.6f}*cos(theta) N*m (theta 为 rad)")
    print(f"[fit] RMS = {rms:.5f} N*m, 条件数 = {np.linalg.cond(design):.1f}")
    print(f"[fit] 摩擦 平均={friction.mean():.4f} 中位={np.median(friction):.4f} "
          f"最小={friction.min():.4f} 最大={friction.max():.4f} N*m")


def main():
    parser = argparse.ArgumentParser(description="上 pitch 静态辨识采集")
    parser.add_argument("--port", default="COM8")
    parser.add_argument("--baud", type=int, default=BAUD)
    parser.add_argument("--low", type=float, default=-50.0, help="最低目标角（度）")
    parser.add_argument("--high", type=float, default=-10.0, help="最高目标角（度）")
    parser.add_argument("--points", type=int, default=9, help="均匀分布的测量角数量")
    parser.add_argument("--cycles", type=int, default=2, help="往复轮数")
    parser.add_argument("--settle-ms", type=int, default=2000, help="每个角度停留时间")
    parser.add_argument("--kp", type=float, default=6.0)
    parser.add_argument("--ki", type=float, default=0.0)
    parser.add_argument("--kd", type=float, default=0.085)
    parser.add_argument("--save", default="pitch_ident_capture.csv")
    parser.add_argument("--file", help="只分析已有 CSV")
    args = parser.parse_args()

    if args.file:
        analyze(args.file)
        return
    capture(args)
    analyze(args.save)


if __name__ == "__main__":
    main()
