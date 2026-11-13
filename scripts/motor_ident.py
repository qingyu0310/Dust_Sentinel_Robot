"""
C620 电流阶跃辨识：串口抓点 → 拟合 K / tau → 给 IMC 整定

模型（一阶惯性，输入为电流 A，输出为减速后轮角速度 rad/s）：

    omega(s)       K
    --------- = -------
     i(s)      1 + tau*s

用法：
    python motor_ident.py                    # 抓 COM21 @921600，抓完自动拟合
    python motor_ident.py --port COM5
    python motor_ident.py --file cap.csv     # 不抓串口，直接拟合已有 CSV

流程：
    1. 板子烧 TRD_TEST=y 固件（chassis 线程需关掉，否则两路都在发 0x200）
    2. 本脚本连串口 → 发 `var set test_on 1` → 等板子回 `ok`
    3. test 线程下一拍跑一轮电流阶跃，脚本同步抓数据
    4. 收到数据后拟合 K / tau / 死区

固件每 1ms 输出一行：
    <uptime_ms>,<current_mA>,<omega_mrad_s>
"""

import argparse
import math
import re
import time

import numpy as np

HEARTBEAT_S = 1.0       # 每 1s 打一次心跳，绝不静默
IDLE_ABORT_S = 3.0      # 连续 3s 没数据就停

ANSI_RE = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
DATA_RE = re.compile(r"m(\d+),(\d+),(-?\d+),(-?\d+)\s*$")


def _text(raw: bytes) -> str:
    return raw.decode("utf-8", errors="replace").strip()


def _parse_point(line: str):
    """从一行日志抠出 (motor, t_ms, current_mA, omega_mrad_s)，抠不到返回 None。"""
    plain = ANSI_RE.sub("", line).strip()
    for prefix in ("[inf]", "[dbg]", "[wrn]", "[err]", "[ok]"):
        if plain.startswith(prefix):
            plain = plain[len(prefix):].strip()
            break
    m = DATA_RE.search(plain)
    if not m:
        return None
    return int(m.group(1)), int(m.group(2)), int(m.group(3)), int(m.group(4))


def _send_cmd(ser, cmd: str) -> bool:
    """发一条 shell 命令并等 ok。"""
    ser.write((cmd + "\r\n").encode())
    ser.flush()
    print(f"→ {cmd}", flush=True)

    deadline = time.time() + 3.0
    while time.time() < deadline:
        raw = ser.readline()
        if not raw:
            continue
        line = _text(raw)
        print(f"← {line}", flush=True)
        if line == "ok":
            return True
    return False


def read_serial(port, baud, steps):
    """逐台电机：var set step_i / test_motor → var set test_on 1 → 抓数据。

    steps: {电机索引: 阶跃电流 A}
    返回 (motor, t_ms, current_mA, omega_mrad_s) 列表。
    """
    try:
        import serial
    except ImportError:
        raise SystemExit("需要 pyserial：pip install pyserial")

    rows = []
    n_bytes = 0
    n_lines = 0

    with serial.Serial(port, baud, timeout=0.05) as ser:
        ser.reset_input_buffer()

        for motor, step in steps.items():
            if not _send_cmd(ser, f"var set step_i {step}"):
                raise SystemExit(f"设置 step_i={step} 失败")
            if not _send_cmd(ser, f"var set test_motor {motor}"):
                raise SystemExit(f"设置 test_motor={motor} 失败")

            n_before = len(rows)

            if not _send_cmd(ser, "var set test_on 1"):
                raise SystemExit(f"触发 m{motor} 失败")

            print(f"[i] m{motor} 已触发（step_i={step}A），开始抓数据", flush=True)

            t_start = last_data = last_beat = time.time()

            try:
                while True:
                    raw = ser.readline()
                    now = time.time()

                    if raw:
                        n_bytes += len(raw)
                        n_lines += 1
                        line = _text(raw)
                        if not line:
                            print(f"[raw-bin] {raw!r}", flush=True)
                        else:
                            parsed = _parse_point(line)

                            if parsed is None:
                                print(f"[raw] {line}", flush=True)
                            else:
                                rows.append(parsed)
                                last_data = now
                                if len(rows) <= 5 or len(rows) % 100 == 0:
                                    mot, t, i, w = parsed
                                    print(f"[i] #{len(rows)}  m{mot} t={t} i={i}mA omega={w}mrad/s", flush=True)

                    if now - last_beat >= HEARTBEAT_S:
                        last_beat = now
                        print(
                            f"[hb] {now - t_start:5.1f}s  字节 {n_bytes}  行 {n_lines}  "
                            f"有效点 {len(rows)}  距上次数据 {now - last_data:4.1f}s",
                            flush=True,
                        )

                    if now - last_data >= IDLE_ABORT_S:
                        print(f"[!] 连续 {IDLE_ABORT_S:.0f}s 没有数据，m{motor} 结束（收 {len(rows) - n_before} 点）", flush=True)
                        break

            except KeyboardInterrupt:
                print(f"\n[i] 手动停止（m{motor} 收 {len(rows) - n_before} 点）", flush=True)
                break

        print(f"[i] 收尾：共收 {n_bytes} 字节 / {n_lines} 行，有效点 {len(rows)} 个", flush=True)

    return rows


def split_steps(rows):
    """把一轮数据按电机、按阶跃切段：返回 [(motor, i_step, t_ms, y), ...]。

    固件逐台跑：每台先正阶跃、停 500ms、再反阶跃；这里先按电机分组，
    再在组内按电流跳变切。组内数据不连续是因为中间插了别台的段，按 t 顺序取即可。
    """
    data = np.array(rows, dtype=float)   # (motor, t_ms, cur_mA, omega_mrad)
    segs = []

    for motor in sorted(set(data[:, 0].astype(int))):
        g = data[data[:, 0] == motor]
        t_ms = g[:, 1]
        cur = g[:, 2] / 1000.0      # mA → A
        omega = g[:, 3] / 1000.0    # mrad/s → rad/s

        idx = np.where(np.abs(np.diff(cur)) > 0.3)[0] + 1

        for n, i0 in enumerate(idx):
            i_step = cur[i0]
            if abs(i_step) <= 0.01:
                continue

            i_end = idx[n + 1] if n + 1 < len(idx) else len(cur)
            segs.append((int(motor), float(i_step), t_ms[i0:i_end], omega[i0:i_end]))

    if not segs:
        raise SystemExit("没找到阶跃，检查固件是否跑起来了")
    return segs


def fit_first_order(t_ms, y, i_step):
    """一阶：omega(t) = w0 + K*i*(1 - exp(-t/tau))。"""
    t = (t_ms - t_ms[0]) / 1000.0

    n_tail = min(len(y), 200)
    w_ss = float(np.mean(y[-n_tail:]))

    if abs(w_ss) < 1e-3:
        raise SystemExit("稳态角速度为 0，电机没转起来？检查供电/接线/step_i")

    t_fit = t
    y_fit_in = y

    def model(tt, K, tau, w0):
        return w0 + K * i_step * (1.0 - np.exp(-tt / tau))

    from scipy.optimize import curve_fit

    p0 = [w_ss / i_step, 0.05, float(y[0])]
    bounds = ([-1000.0, 1e-4, -100.0], [1000.0, 2.0, 100.0])
    popt, _ = curve_fit(model, t_fit, y_fit_in, p0=p0, bounds=bounds, maxfev=20000)

    K, tau, _ = popt
    rms = float(np.sqrt(np.mean((model(t_fit, *popt) - y_fit_in) ** 2)))

    return {"K": K, "tau": tau, "w_ss": w_ss, "i_step": i_step, "rms": rms, "t": t, "y": y, "y_fit": model(t, *popt)}


def fit_second_order(t_ms, y, i_step):
    """二阶：两个一阶串联 omega(t) = w0 + K*i*(1 - (t1*e^(-t/t1) - t2*e^(-t/t2))/(t1-t2))。

    电气极点 L/R 与机械极点 J/b 同时拟合；t1 == t2 时退化成临界阻尼 (1 - (1+t/tau)e^(-t/tau))。
    """
    t = (t_ms - t_ms[0]) / 1000.0

    n_tail = min(len(y), 200)
    w_ss = float(np.mean(y[-n_tail:]))

    if abs(w_ss) < 1e-3:
        raise SystemExit("稳态角速度为 0，电机没转起来？")

    t_fit = t
    y_fit_in = y

    def model(tt, K, t1, t2, w0):
        # t1 == t2 时用临界阻尼极限，避免 0/0
        denom = t1 - t2
        if abs(denom) < 1e-6:
            return w0 + K * i_step * (1.0 - (1.0 + tt / t1) * np.exp(-tt / t1))
        return w0 + K * i_step * (1.0 - (t1 * np.exp(-tt / t1) - t2 * np.exp(-tt / t2)) / denom)

    from scipy.optimize import curve_fit

    p0 = [w_ss / i_step, 0.002, 0.05, float(y[0])]
    bounds = ([-1000.0, 1e-6, 1e-6, -100.0], [1000.0, 1.0, 1.0, 100.0])
    popt, _ = curve_fit(model, t_fit, y_fit_in, p0=p0, bounds=bounds, maxfev=40000)

    K, t1, t2, _ = popt
    rms = float(np.sqrt(np.mean((model(t_fit, *popt) - y_fit_in) ** 2)))

    return {"K": K, "t1": t1, "t2": t2, "w_ss": w_ss, "i_step": i_step, "rms": rms, "t": t, "y": y, "y_fit": model(t, *popt)}


def print_first_order(name, r):
    print(f"\n---------- {name}（一阶） ----------")
    print(f"阶跃电流   i_step = {r['i_step']:+.3f} A")
    print(f"稳态角速度 w_ss   = {r['w_ss']:+.4f} rad/s  ({r['w_ss'] * 60 / (2 * math.pi):.1f} rpm 轮)")
    print(f"稳态增益   K      = {r['K']:+.4f} rad/s / A")
    print(f"时间常数   tau    = {r['tau'] * 1000:.1f} ms")
    print(f"拟合 RMS          = {r['rms']:.5f} rad/s  (占稳态 {r['rms'] / abs(r['w_ss']) * 100:.2f}%)")


def print_second_order(name, r):
    print(f"\n---------- {name}（二阶） ----------")
    print(f"阶跃电流   i_step = {r['i_step']:+.3f} A")
    print(f"稳态增益   K      = {r['K']:+.4f} rad/s / A")
    t1_ms = r["t1"] * 1000
    t2_ms = r["t2"] * 1000
    tag = "电气极点可忽略（数据里看不到 L/R）" if t1_ms < 1.0 else "电气 L/R"
    print(f"极点时间常数 t1   = {t1_ms:.2f} ms  ({tag})")
    print(f"极点时间常数 t2   = {t2_ms:.2f} ms  (机械 J/b)")
    print(f"拟合 RMS          = {r['rms']:.5f} rad/s  (占稳态 {r['rms'] / abs(r['w_ss']) * 100:.2f}%)")


def main():
    ap = argparse.ArgumentParser(description="C620 电流阶跃辨识")
    ap.add_argument("--port", default="COM8")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--file", help="已有 CSV（不抓串口）")
    ap.add_argument("--save", default="ident_capture.csv", help="抓到的数据存这里")
    ap.add_argument("--step", default="1.0,1.0,2.0,1.5",
                    help="四台电机各自的阶跃电流 A，逗号分隔（默认 1.0,1.0,2.0,1.5）")
    args = ap.parse_args()

    steps = [float(x) for x in args.step.split(",")]
    if len(steps) != 4:
        raise SystemExit(f"--step 需要 4 个值，收到 {len(steps)} 个")

    step_map = {i: s for i, s in enumerate(steps)}

    if args.file:
        rows = []
        with open(args.file) as f:
            for line in f:
                p = line.strip().split(",")
                if len(p) == 4:
                    rows.append((int(p[0]), int(p[1]), int(p[2]), int(p[3])))
        if not rows:
            raise SystemExit(f"{args.file} 里没有数据")
    else:
        rows = read_serial(args.port, args.baud, step_map)
        if not rows:
            raise SystemExit("没抓到数据")

        with open(args.save, "w") as f:
            for m, t, i, w in rows:
                f.write(f"{m},{t},{i},{w}\n")
        print(f"[i] 已保存 {len(rows)} 行 → {args.save}")

    print(f"\n[i] 采样点 {len(rows)}，电机数 {len(set(r[0] for r in rows))}")

    segs = split_steps(rows)
    print(f"[i] 切出 {len(segs)} 段阶跃")

    # 按电机分组结果，最后统一出表
    result = {}

    for motor in sorted(set(s[0] for s in segs)):
        motor_segs = [s for s in segs if s[0] == motor]
        print(f"\n{'=' * 60}\n电机 m{motor}（共 {len(motor_segs)} 段）\n{'=' * 60}")

        first = []
        second = []

        for _, i_step, t_ms, y in motor_segs:
            name = "正向" if i_step > 0 else "反向"

            r1 = fit_first_order(t_ms, y, i_step)
            first.append((name, r1))
            print_first_order(f"m{motor} {name}", r1)

            r2 = fit_second_order(t_ms, y, i_step)
            second.append((name, r2))
            print_second_order(f"m{motor} {name}", r2)

        # 正反对称性
        pos = next((r for n, r in first if n == "正向"), None)
        neg = next((r for n, r in first if n == "反向"), None)

        if pos and neg:
            dK = abs(abs(pos["K"]) - abs(neg["K"])) / max(abs(pos["K"]), abs(neg["K"])) * 100
            dTau = abs(pos["tau"] - neg["tau"]) / max(pos["tau"], neg["tau"]) * 100
            print(f"\n-- 正反对称性 m{motor} --")
            print(f"|K| 正向 {abs(pos['K']):.4f}  反向 {abs(neg['K']):.4f}  偏差 {dK:.1f}%")
            print(f"tau 正向 {pos['tau'] * 1000:.1f}ms  反向 {neg['tau'] * 1000:.1f}ms  偏差 {dTau:.1f}%")
            if dK > 15 or dTau > 15:
                print("[!] 正反偏差 > 15%：摩擦/齿隙非对称明显，一阶模型不成立")
            else:
                print("[ok] 正反基本对称，一阶模型可用")

        # 一阶 vs 二阶
        if first and second:
            r1 = first[0][1]
            r2 = second[0][1]
            improve = (1.0 - r2["rms"] / r1["rms"]) * 100 if r1["rms"] > 0 else 0.0
            print(f"\n-- 一阶 vs 二阶 m{motor} --")
            print(f"一阶 RMS {r1['rms']:.5f}  二阶 RMS {r2['rms']:.5f}  改善 {improve:.1f}%")

            if improve < 0:
                print("[!] 二阶比一阶更差：两极点无法分离（数据时长不足以区分），二阶结果作废，用一阶")
            elif improve < 10:
                print("[ok] 二阶改善 < 10%，一阶够用")
            else:
                print("[!] 二阶改善 > 10%，电气极点不可忽略")

        if first:
            result[motor] = first

    # 汇总表
    print(f"\n{'=' * 70}")
    print("汇总（一阶）")
    print(f"{'电机':<6}{'K (rad/s/A)':>16}{'tau (ms)':>12}{'RMS 占比':>12}{'kp':>10}{'ki':>10}")
    print("-" * 70)

    for motor, entries in result.items():
        r = entries[0][1]                       # 用正向段出 IMC
        K_abs = abs(r["K"])
        lam = r["tau"] / 3.0
        kp = r["tau"] / (K_abs * lam)
        ki = 1.0 / (K_abs * lam)
        print(f"m{motor:<5}{r['K']:>16.4f}{r['tau'] * 1000:>12.1f}{r['rms'] / abs(r['w_ss']) * 100:>11.2f}%{kp:>10.4f}{ki:>10.4f}")

    print("-" * 70)
    print("IMC：lambda = tau/3，kp = tau/(|K|*lambda)，ki = 1/(|K|*lambda)")

    # 扫档分析：稳态转速 vs 电流。线性 = 惯性主导，亚线性 = 摩擦主导
    print(f"\n{'=' * 70}")
    print("扫档分析：稳态转速 vs 电流")
    print(f"{'电机':<6}{'电流 A':>10}{'稳态 rad/s':>14}{'K 等效':>12}")
    print("-" * 70)

    for motor in sorted(set(s[0] for s in segs)):
        motor_segs = [s for s in segs if s[0] == motor]
        pts = []

        for _, i_step, _, y in motor_segs:
            n_tail = min(len(y), 200)
            w_ss = float(np.mean(y[-n_tail:]))
            pts.append((i_step, w_ss))
            print(f"m{motor:<5}{i_step:>10.2f}{w_ss:>14.2f}{w_ss / i_step:>12.2f}")

        # 判据：最大电流档的等效 K 与最小电流档比较
        if len(pts) >= 2:
            pts_sorted = sorted(pts, key=lambda p: abs(p[0]))
            lo_i, lo_w = pts_sorted[0]
            hi_i, hi_w = pts_sorted[-1]
            k_lo = abs(lo_w / lo_i)
            k_hi = abs(hi_w / hi_i)
            ratio = k_hi / k_lo if k_lo > 0 else 0.0

            print(f"  等效 K：{abs(lo_i):.1f}A 时 {k_lo:.2f}，{abs(hi_i):.1f}A 时 {k_hi:.2f}，比值 {ratio:.2f}")

            if ratio < 0.7:
                print("  [!] 大电流下等效 K 明显降低 = 摩擦主导（转速亚线性）")
            elif ratio > 1.3:
                print("  [!] 大电流下等效 K 明显升高 = 异常，检查饱和/温漂")
            else:
                print("  [ok] 等效 K 基本恒定 = 惯性/反电动势主导（线性）")
        print()

    try:
        import matplotlib.pyplot as plt
        n = len(result) * 2
        fig, axes = plt.subplots(n, 1, figsize=(10, 3.0 * n), squeeze=False)
        idx = 0

        for motor, entries in result.items():
            for name, r1 in entries:
                ax = axes[idx, 0]
                ax.plot(r1["t"] * 1000, r1["y"], ".", ms=1, label="measured")
                ax.plot(r1["t"] * 1000, r1["y_fit"], "-", lw=1, label=f"1st K={r1['K']:.3f} tau={r1['tau'] * 1000:.1f}ms")
                ax.set_title(f"m{motor} {name}")
                ax.set_xlabel("t (ms)")
                ax.set_ylabel("omega (rad/s)")
                ax.legend(fontsize=8)
                ax.grid(True, alpha=0.3)
                idx += 1

        fig.tight_layout()
        fig.savefig("ident_curve.png", dpi=120)
        print("\n[i] 曲线已存 ident_curve.png")
    except ImportError:
        pass


if __name__ == "__main__":
    main()
