"""上 pitch 重力补偿 A/B 验证：固定角度，依次 0/50/100/0%，输出对照表。

python scripts/pitch_gravity_check.py --port COM8 --angles -40 -30 -25
python scripts/pitch_gravity_check.py --file pitch_gravity_check.csv
"""
import argparse
import csv
import math
import re
import statistics as st
import time
from collections import defaultdict, deque
from pathlib import Path

ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
DATA = re.compile(r"PGRAV,((?:-?\d+,){7}-?\d+)\s*$")
FIELDS = ["angle_deg", "stage", "scale", "t_ms", "ref_mdeg", "actual_mdeg",
          "pid_mnm", "gravity_mnm", "total_mnm", "feedback_mnm", "scale_permille"]


class Link:
    def __init__(self, ser):
        self.ser = ser
        self.pending = 0
        self.feed = 1
        self.last_feed = time.monotonic()
        self.active = False
        self.end = None
        self.ack_commands = deque()
        self.feed_ack = 0
        self.last_feed_ack = time.monotonic()
        self.mcu_feed = None
        self.mcu_feed_age = None

    def send(self, command, ack=True):
        self.ser.write((command + "\r\n").encode("ascii"))
        self.ser.flush()
        if ack:
            self.pending += 1
            self.ack_commands.append(command)

    def receive(self):
        now = time.monotonic()
        if self.active and now-self.last_feed >= 0.5:
            self.feed += 1
            self.send(f"var set ptest_feed {self.feed}")
            self.last_feed = now
        line = ANSI.sub("", self.ser.readline().decode("utf-8", errors="replace")).strip()
        if line in ("ok", "log off: ok"):
            self.pending = max(0, self.pending-1)
            command = self.ack_commands.popleft() if self.ack_commands else ""
            match = re.fullmatch(r"var set ptest_feed (\d+)", command)
            if match:
                self.feed_ack = int(match.group(1))
                self.last_feed_ack = time.monotonic()
        heart = re.search(r"PHEART,(\d+),(-?\d+),(\d+),(\d+)\s*$", line)
        if heart:
            self.mcu_feed = int(heart.group(2))
            self.mcu_feed_age = int(heart.group(3))
            return line, None
        if "PTEST_END," in line:
            self.end = line.split("PTEST_END,", 1)[1]
        match = DATA.search(line)
        if match:
            return line, list(map(int, match.group(1).split(",")))
        if line and line not in ("ok", "log off: ok") and not re.fullmatch(r"[-+\d.eE]+", line):
            print("[mcu]", line, flush=True)
        return line, None

    def wait_acks(self):
        deadline = time.monotonic()+3
        while self.pending and time.monotonic() < deadline:
            self.receive()
        if self.pending:
            raise RuntimeError("shell 命令未确认；不会继续下一测试阶段")

    def set(self, name, value):
        self.send(f"var set {name} {value}")
        self.wait_acks()

    def get_float(self, name):
        self.wait_acks()
        self.send(f"var get {name}", ack=False)
        deadline = time.monotonic()+3
        while time.monotonic() < deadline:
            line, _ = self.receive()
            number = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
            match = re.fullmatch(rf"(?:{re.escape(name)}\s*=\s*)?({number})", line)
            if match:
                value = float(match.group(1))
                if not math.isfinite(value):
                    raise RuntimeError(f"{name} 回复了非有限值")
                return value
        raise RuntimeError(f"读取 {name} 失败")


def capture(args):
    import serial
    rows = []
    with serial.Serial(args.port, args.baud, timeout=0.05, write_timeout=1) as ser, \
            open(args.save, "w", newline="", encoding="utf-8") as f:
        ser.reset_input_buffer()
        link = Link(ser)
        writer = csv.writer(f)
        writer.writerow(FIELDS)
        original_scale = None
        try:
            link.send("log off")
            link.wait_acks()
            link.set("ptest_on", 0)
            original_scale = link.get_float("pitch_up_gscale")
            # 初始角取最近的 IMU 角，由 MCU 从当前姿态平滑进入第一测点。
            link.set("ptest_mode", 2)
            link.set("ptest_speed", args.speed)
            link.set("ptest_hold", args.angles[0])
            link.set("ptest_feed", link.feed)
            link.end = None
            link.active = True
            link.set("ptest_on", 1)
            for angle in args.angles:
                link.set("ptest_hold", angle)
                # 回到 0% 再开始该角度的对照；末尾再测 0% 检查漂移/滞回。
                for stage, scale in enumerate([0.0, 0.5, 1.0, 0.0]):
                    link.set("pitch_up_gscale", scale)
                    print(f"[stage] angle={angle:g}deg scale={scale:g}，等待目标到位及稳定时间", flush=True)
                    start = beat = last_data = time.monotonic()
                    ready_at = collect_at = None
                    count = 0
                    while True:
                        _, data = link.receive()
                        now = time.monotonic()
                        if link.end:
                            raise RuntimeError(f"MCU 结束测试: {link.end}")
                        if data is not None:
                            last_data = now
                            # 使用 MCU 实际目标和实际强度确认阶段，不能把串口排队旧数据混入。
                            matching = abs(data[1]/1000-angle) < 0.05 and abs(data[7]/1000-scale) < 0.002
                            if not matching:
                                ready_at = collect_at = None
                            else:
                                if ready_at is None:
                                    ready_at = now
                                if now-ready_at >= args.settle:
                                    if collect_at is None:
                                        collect_at = now
                                    row = [angle, stage, scale] + data
                                    rows.append(row)
                                    writer.writerow(row)
                                    count += 1
                            if collect_at is not None and now-collect_at >= args.measure:
                                if count < 50:
                                    raise RuntimeError("测量段数据不足")
                                break
                        if now-beat >= 1:
                            print(f"[hb] angle={angle:g} scale={scale:g} elapsed={now-start:.1f}s samples={count} "
                                  f"feed_sent={link.feed} ack={link.feed_ack} "
                                  f"ack_age={now-link.last_feed_ack:.2f}s "
                                  f"mcu_feed={link.mcu_feed} mcu_age_ms={link.mcu_feed_age}", flush=True)
                            f.flush()
                            beat = now
                        if now-last_data > 3:
                            raise RuntimeError("3 秒无 PGRAV 遥测；需烧录支持模式 2 的固件")
                        if now-start > args.stage_timeout:
                            raise RuntimeError("阶段超时，实际目标或补偿强度未匹配")
                    f.flush()
        finally:
            # 请求结束固定目标；MCU 保持当前位置。恢复测试前的补偿强度。
            link.active = False
            try:
                link.set("ptest_on", 0)
            finally:
                if original_scale is not None:
                    link.set("pitch_up_gscale", original_scale)
            print(f"[i] 已保存 {len(rows)} 点: {args.save}", flush=True)
    return rows


def summarize(rows, args):
    grouped = defaultdict(list)
    for row in rows:
        grouped[(float(row[0]), int(row[1]), float(row[2]))].append([float(v) for v in row[3:]])
    summaries = []
    for (angle, stage, scale), data in sorted(grouped.items()):
        if len(data) < 50:
            print(f"[quality] angle={angle} stage={stage} 数据不足，排除")
            continue
        actual = [r[2]/1000 for r in data]
        errors = [x-angle for x in actual]
        pid = st.mean(r[3]/1000 for r in data)
        gravity = st.mean(r[4]/1000 for r in data)
        total = st.mean(r[5]/1000 for r in data)
        dt = (data[-1][0]-data[0][0])/1000
        if dt <= 0:
            raise ValueError("MCU 时间戳异常")
        n = max(1, len(actual)//5)
        drift = (st.mean(actual[-n:])-st.mean(actual[:n])) / (dt*0.8)
        summaries.append(dict(angle_deg=angle, stage=stage, scale=scale, points=len(data),
                              error_deg=st.mean(errors), rms_deg=math.sqrt(st.mean(e*e for e in errors)),
                              jitter_std_deg=st.pstdev(actual), peak_to_peak_deg=max(actual)-min(actual),
                              drift_deg_s=drift, pid_nm=pid, gravity_nm=gravity, total_nm=total))
    if not summaries:
        raise ValueError("没有足够的有效测量段")
    path = Path(args.save).with_name(Path(args.save).stem+"_summary.csv")
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(summaries[0]))
        writer.writeheader()
        writer.writerows(summaries)
    for angle in sorted({r["angle_deg"] for r in summaries}):
        stage = {r["stage"]: r for r in summaries if r["angle_deg"] == angle}
        print(f"\n[angle] {angle:g}deg")
        for index, r in sorted(stage.items()):
            print(f"  stage={index} scale={r['scale']:g} error={r['error_deg']:+.3f}deg "
                  f"jitter_std={r['jitter_std_deg']:.3f}deg PID={r['pid_nm']:+.3f} "
                  f"gravity={r['gravity_nm']:+.3f} total={r['total_nm']:+.3f}N*m")
        if not all(i in stage for i in range(4)):
            print("  [结论] 阶段不完整，不能判定")
            continue
        first, last = stage[0], stage[3]
        if any(abs(r['drift_deg_s']) >= 0.1 or r['jitter_std_deg'] >= args.stable_std for r in (first, last)):
            print("  [结论] 0% 基线仍有明显运动/抖动，本角度不判定优劣")
            continue
        if abs(first['pid_nm']-last['pid_nm']) > max(0.05, abs(first['pid_nm'])*0.25) or \
                abs(first['error_deg']-last['error_deg']) > args.error_tolerance:
            print("  [结论] 两次 0% 基线不同，存在漂移/滞回，本角度不判定优劣")
            continue
        baseline_pid = (abs(first['pid_nm'])+abs(last['pid_nm']))/2
        baseline_error = (abs(first['error_deg'])+abs(last['error_deg']))/2
        baseline_jitter = max(first['jitter_std_deg'], last['jitter_std_deg'])
        for index in [1, 2]:
            r = stage[index]
            reduction = baseline_pid-abs(r['pid_nm'])
            stable = abs(r['drift_deg_s']) < 0.1 and r['jitter_std_deg'] < args.stable_std
            better = reduction > max(0.02, baseline_pid*0.1) and \
                abs(r['error_deg']) <= baseline_error+args.error_tolerance and \
                r['jitter_std_deg'] <= baseline_jitter+0.1
            verdict = "PID 负担减小且未明显恶化" if stable and better else "未满足改善判据，需查看数据"
            print(f"  [结论] {r['scale']*100:.0f}%: PID绝对均值减少 {reduction:+.3f}N*m；{verdict}")
    print(f"\n[i] 统计表: {path}；总力矩不要求减小，主要比较 PID 负担和姿态质量。")
    return summaries


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default="COM8")
    p.add_argument("--baud", type=int, default=921600)
    p.add_argument("--angles", nargs="+", type=float, default=[-40, -30, -25])
    p.add_argument("--speed", type=float, default=2)
    p.add_argument("--settle", type=float, default=8, help="MCU 目标到位后等待秒数")
    p.add_argument("--measure", type=float, default=6, help="每阶段采样秒数")
    p.add_argument("--stage-timeout", type=float, default=90)
    p.add_argument("--error-tolerance", type=float, default=0.5)
    p.add_argument("--stable-std", type=float, default=0.2)
    p.add_argument("--save", default="pitch_gravity_check.csv")
    p.add_argument("--file")
    args = p.parse_args()
    values = args.angles+[args.speed, args.settle, args.measure, args.stage_timeout,
                         args.error_tolerance, args.stable_std]
    if not all(math.isfinite(v) for v in values) or not (0 < args.speed <= 10 and args.settle > 0 and
            args.measure >= 1 and args.stage_timeout > args.settle+args.measure and
            args.error_tolerance > 0 and args.stable_std > 0):
        p.error("参数无效")
    try:
        if args.file:
            with open(args.file, encoding="utf-8") as f:
                rows = [[float(v) for v in r] for r in csv.reader(f) if r and r[0] != FIELDS[0]]
        else:
            rows = capture(args)
        summarize(rows, args)
    except (RuntimeError, ValueError, OSError, KeyboardInterrupt) as e:
        raise SystemExit(f"[!] {e}；已有采样已保留")


if __name__ == "__main__":
    main()
