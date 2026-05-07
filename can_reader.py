#!/usr/bin/env python3
"""
CAN 总线接收程序 — TK-mid 反馈帧监控
======================================
实时监听 CAN 总线, 解析并展示 TK-mid 反馈帧。

用法:
  # 监听所有 TK-mid 反馈帧
  python can_reader.py

  # 只监听运动控制反馈
  python can_reader.py --id ctrl_fb

  # 只监听轮速反馈
  python can_reader.py --id wheel_fb

  # 显示原始报文 (不解析)
  python can_reader.py --raw

  # 输出到日志文件
  python can_reader.py --log can_data.log

  # 指定接口和波特率
  python can_reader.py --channel can0 --bitrate 500000
"""

import sys
import time
import argparse
import signal
from datetime import datetime
from tkmid_can import TKMidCAN
from tkmid_can.config import (
    CTRL_FB_ID, L_WHEEL_FB_ID, R_WHEEL_FB_ID,
    IO_FB_ID, BMS_FB_ID, BMS_FLAG_FB_ID,
)
from tkmid_can.frames import (
    parse_ctrl_fb, parse_wheel_fb, parse_io_fb,
)

# ============================================================
# CAN ID → 名称映射
# ============================================================
ID_MAP = {
    CTRL_FB_ID:      'CTRL_FB',
    L_WHEEL_FB_ID:   'L_WHEEL',
    R_WHEEL_FB_ID:   'R_WHEEL',
    IO_FB_ID:        'IO_FB',
    BMS_FB_ID:       'BMS',
    BMS_FLAG_FB_ID:  'BMS_FLAG',
}

# ============================================================
# 解析器分发
# ============================================================
class FrameParser:
    """根据 CAN ID 解析不同的反馈帧"""

    @staticmethod
    def parse(arb_id: int, data: bytes) -> dict | None:
        if arb_id == CTRL_FB_ID:
            return parse_ctrl_fb(data)
        elif arb_id in (L_WHEEL_FB_ID, R_WHEEL_FB_ID):
            return parse_wheel_fb(data)
        elif arb_id == IO_FB_ID:
            return parse_io_fb(data)
        # BMS 帧暂不解析, 只显示原始数据
        return None

    @staticmethod
    def format(arb_id: int, data: bytes, raw_mode: bool = False) -> str:
        """格式化一帧报文为可读字符串"""
        name = ID_MAP.get(arb_id, f'0x{arb_id:08X}')
        hex_str = data.hex()

        if raw_mode:
            return f"[{name}] {hex_str}"

        parsed = FrameParser.parse(arb_id, data)
        if parsed is None:
            return f"[{name}] {hex_str}"

        if arb_id == CTRL_FB_ID:
            return (f"[{name}] "
                    f"档位={parsed['gear']} "
                    f"速度={parsed['speed']:+.3f}m/s "
                    f"角速度={parsed['angular_vel']:+.2f}°/s "
                    f"心跳={parsed['alive']} "
                    f"BCC=0x{parsed['bcc']:02X}")
        elif arb_id in (L_WHEEL_FB_ID, R_WHEEL_FB_ID):
            wheel = 'L' if arb_id == L_WHEEL_FB_ID else 'R'
            return (f"[{name}] "
                    f"{wheel}轮速={parsed['speed']:+.3f}m/s "
                    f"脉冲={parsed['pulse_count']} "
                    f"心跳={parsed['alive']} "
                    f"BCC=0x{parsed['bcc']:02X}")
        elif arb_id == IO_FB_ID:
            return (f"[{name}] {parsed['raw']} "
                    f"心跳={parsed['alive']} "
                    f"BCC=0x{parsed['bcc']:02X}")

        return f"[{name}] {hex_str}"


# ============================================================
# 可选的帧计数器
# ============================================================
class FrameCounter:
    def __init__(self):
        self.counts = {}
        self.start_time = time.time()

    def tick(self, arb_id: int):
        self.counts[arb_id] = self.counts.get(arb_id, 0) + 1

    def summary(self) -> str:
        elapsed = time.time() - self.start_time
        lines = [f"--- 统计 (运行 {elapsed:.0f}s) ---"]
        total = 0
        for aid, cnt in sorted(self.counts.items()):
            name = ID_MAP.get(aid, f'0x{aid:08X}')
            rate = cnt / elapsed if elapsed > 0 else 0
            lines.append(f"  {name}: {cnt} 帧 ({rate:.1f} fps)")
            total += cnt
        lines.append(f"  总计: {total} 帧")
        return "\n".join(lines)


# ============================================================
# 主逻辑
# ============================================================
def main():
    parser = argparse.ArgumentParser(
        description='TK-mid CAN 反馈帧监控程序',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python can_reader.py
  python can_reader.py --id ctrl_fb
  python can_reader.py --raw
  python can_reader.py --log can_data.log
        """
    )
    parser.add_argument('--channel', default='can0', help='CAN 接口名 (默认: can0)')
    parser.add_argument('--bitrate', type=int, default=500000, help='波特率 (默认: 500000)')
    parser.add_argument('--id', dest='filter_id', default=None,
                        choices=['ctrl_fb', 'l_wheel', 'r_wheel', 'io_fb', 'bms', 'bms_flag'],
                        help='只监听指定 ID 的帧')
    parser.add_argument('--raw', action='store_true', help='原始模式 (不解析, 只显示十六进制)')
    parser.add_argument('--log', dest='logfile', default=None, help='日志文件路径')
    parser.add_argument('--stats', action='store_true', help='退出时显示统计信息')

    args = parser.parse_args()

    # ID 过滤映射
    filter_id_map = {
        'ctrl_fb':  CTRL_FB_ID,
        'l_wheel':  L_WHEEL_FB_ID,
        'r_wheel':  R_WHEEL_FB_ID,
        'io_fb':    IO_FB_ID,
        'bms':      BMS_FB_ID,
        'bms_flag': BMS_FLAG_FB_ID,
    }
    filter_aid = filter_id_map.get(args.filter_id)

    # 打开日志文件
    log_fh = None
    if args.logfile:
        log_fh = open(args.logfile, 'a', encoding='utf-8')
        print(f"[LOG] 日志输出到: {args.logfile}")

    counter = FrameCounter() if args.stats else None

    # 使用 TKMidCAN 的底层接收能力
    tkmid = TKMidCAN(channel=args.channel, bitrate=args.bitrate)
    if not tkmid.connect():
        print("[FAIL] 无法连接 CAN 总线")
        sys.exit(1)

    running = True

    def graceful_exit(sig, frame):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT, graceful_exit)
    signal.signal(signal.SIGTERM, graceful_exit)

    # 不使用 start_recv, 直接在主线程循环接收
    print(f"[监听] {args.channel} 上等待 TK-mid 反馈帧 ... (Ctrl+C 退出)")
    if filter_aid:
        name = ID_MAP.get(filter_aid, hex(filter_aid))
        print(f"[过滤] 只显示: {name}")
    print()

    last_summary = time.time()

    while running:
        try:
            msg = tkmid.bus.recv(timeout=0.5)
            if msg is None:
                continue
        except Exception as e:
            print(f"[ERROR] 接收失败: {e}")
            continue

        arb_id = msg.arbitration_id
        data = bytes(msg.data)

        # 过滤: 只处理 TK-mid 相关 ID (或用户指定的 ID)
        if filter_aid is not None:
            if arb_id != filter_aid:
                continue
        elif arb_id not in ID_MAP:
            continue  # 只显示已知反馈帧, 忽略无关报文

        # 格式化输出
        line = FrameParser.format(arb_id, data, args.raw)
        print(line)

        # 写日志
        if log_fh:
            ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
            log_fh.write(f"{ts} {line}\n")
            log_fh.flush()

        # 统计
        if counter:
            counter.tick(arb_id)

        # 定期显示统计 (每 10 秒)
        if counter and (time.time() - last_summary) > 10:
            print(counter.summary())
            print()
            last_summary = time.time()

    # 退出
    tkmid.disconnect()
    if counter:
        print()
        print(counter.summary())
    if log_fh:
        log_fh.close()
    print("[OK] 已退出")


if __name__ == '__main__':
    main()
