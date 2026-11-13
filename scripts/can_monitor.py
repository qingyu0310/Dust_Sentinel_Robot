"""
CAN 总线监听：接收并打印所有帧，用于验证发送链路

用法：
    python can_monitor.py                    # COM87 @1Mbps
    python can_monitor.py --port COM87 --bitrate 1000000
    python can_monitor.py --filter 0x13      # 只显示指定 ID
    python can_monitor.py --csv out.csv      # 存 CSV

帧格式：时间戳 | ID | DLC | 数据
"""

import argparse
import csv
import time

import can

HEARTBEAT_S = 2.0       # 每 2s 打一次心跳
IDLE_ABORT_S = 10.0     # 连续 10s 没数据就停


def main():
    ap = argparse.ArgumentParser(description="CAN 总线监听")
    ap.add_argument("--port", default="COM87")
    ap.add_argument("--bitrate", type=int, default=1000000)
    ap.add_argument("--filter", type=lambda x: int(x, 0), default=None,
                    help="只显示指定 ID（十六进制，如 0x13）")
    ap.add_argument("--csv", help="保存到 CSV")
    args = ap.parse_args()

    bus = can.Bus(interface="slcan", channel=args.port, bitrate=args.bitrate)
    print(f"[i] 已打开 {args.port} @ {args.bitrate} bps", flush=True)

    rows = []
    counts = {}
    t_start = last_data = last_beat = time.time()

    try:
        while True:
            msg = bus.recv(timeout=0.1)
            now = time.time()

            if msg is not None:
                last_data = now
                counts[msg.arbitration_id] = counts.get(msg.arbitration_id, 0) + 1

                if args.filter is None or msg.arbitration_id == args.filter:
                    data = " ".join(f"{b:02X}" for b in msg.data)
                    line = f"{now - t_start:8.3f}s  ID=0x{msg.arbitration_id:03X}  DLC={msg.dlc}  [{data}]"
                    print(line, flush=True)
                    rows.append((f"{now - t_start:.3f}", f"0x{msg.arbitration_id:03X}", msg.dlc, data))

            if now - last_beat >= HEARTBEAT_S:
                last_beat = now
                summary = "  ".join(f"0x{k:03X}:{v}" for k, v in sorted(counts.items()))
                print(f"[hb] {now - t_start:6.1f}s  帧数 {sum(counts.values())}  {summary or '(无)'}", flush=True)

            if now - last_data >= IDLE_ABORT_S:
                print(f"[!] 连续 {IDLE_ABORT_S:.0f}s 没有帧，停止", flush=True)
                break

    except KeyboardInterrupt:
        print("\n[i] 手动停止", flush=True)

    bus.shutdown()

    print(f"\n[i] 收尾：总帧数 {sum(counts.values())}")
    for k, v in sorted(counts.items()):
        print(f"  0x{k:03X}: {v} 帧")

    if args.csv and rows:
        with open(args.csv, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["t", "id", "dlc", "data"])
            w.writerows(rows)
        print(f"[i] 已保存 {len(rows)} 行 → {args.csv}")


if __name__ == "__main__":
    main()
