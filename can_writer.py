#!/usr/bin/env python3
"""
CAN 总线发送程序 — TK-mid 控制帧下发
======================================
以 10ms 周期持续下发运动控制/自由控制/IO 指令。
支持命令行参数和交互模式。

用法:
  # 运动控制 — 前进 0.5 m/s
  python can_writer.py motion --speed 0.5

  # 运动控制 — 前进 + 左转
  python can_writer.py motion --speed 0.3 --angular 25

  # 自由控制 — 差速原地左转
  python can_writer.py free --left -0.3 --right 0.3

  # IO 解锁
  python can_writer.py io --unlock

  # 停车
  python can_writer.py stop

  # 交互模式
  python can_writer.py
"""

import sys
import time
import argparse
import signal
from tkmid_can import TKMidCAN, Gear


def create_parser():
    parser = argparse.ArgumentParser(
        description='TK-mid CAN 控制帧发送程序',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python can_writer.py motion --speed 0.5
  python can_writer.py motion --speed 0.3 --angular 25
  python can_writer.py free --left -0.3 --right 0.3
  python can_writer.py io --unlock
  python can_writer.py stop
        """
    )
    parser.add_argument('--channel', default='can0', help='CAN 接口名 (默认: can0)')
    parser.add_argument('--bitrate', type=int, default=500000, help='波特率 (默认: 500000)')

    sub = parser.add_subparsers(dest='command', help='控制模式')

    # motion
    p_motion = sub.add_parser('motion', help='运动学控制模式')
    p_motion.add_argument('--speed', type=float, default=0.0, help='目标线速度 m/s (默认: 0)')
    p_motion.add_argument('--angular', type=float, default=0.0, help='目标角速度 °/s, 左正右负 (默认: 0)')

    # free
    p_free = sub.add_parser('free', help='自由控制模式')
    p_free.add_argument('--left', type=float, default=0.0, help='左轮目标速度 m/s (默认: 0)')
    p_free.add_argument('--right', type=float, default=0.0, help='右轮目标速度 m/s (默认: 0)')

    # io
    p_io = sub.add_parser('io', help='IO 控制')
    p_io.add_argument('--unlock', action='store_true', help='安全停车解锁')

    # stop
    sub.add_parser('stop', help='停止 (发送零速停车帧)')

    return parser


def interactive_mode(tkmid: TKMidCAN):
    """交互式控制台"""
    print("\n" + "=" * 55)
    print(" CAN Writer — 交互模式 (控制帧 10ms 周期下发)")
    print("=" * 55)
    print("  w <speed> [angular]  — 运动控制 (例: w 0.5 / w 0.3 25)")
    print("  f <left> <right>     — 自由控制 (例: f -0.3 0.3)")
    print("  u                    — 安全停车解锁")
    print("  s                    — 停止")
    print("  q                    — 退出")
    print("=" * 55 + "\n")

    ctrl_active = False
    ctrl_mode = None

    try:
        while True:
            raw = input("CAN-WRITE> ").strip()
            if not raw:
                continue
            parts = raw.split()
            cmd = parts[0].lower()

            if cmd == 'q':
                break
            elif cmd == 'w':
                speed = float(parts[1]) if len(parts) > 1 else 0.0
                angular = float(parts[2]) if len(parts) > 2 else 0.0
                if not ctrl_active:
                    tkmid.start_ctrl(speed, angular)
                    ctrl_active = True
                    ctrl_mode = 'motion'
                elif ctrl_mode == 'motion':
                    tkmid.update_ctrl(speed, angular)
                else:
                    tkmid.stop_control()
                    tkmid.start_ctrl(speed, angular)
                    ctrl_mode = 'motion'
                print(f"  → 运动控制: speed={speed} m/s, angular={angular} °/s")
            elif cmd == 'f':
                if len(parts) < 3:
                    print("  用法: f <left> <right>")
                    continue
                left = float(parts[1])
                right = float(parts[2])
                if not ctrl_active:
                    tkmid.start_free_ctrl(left, right)
                    ctrl_active = True
                    ctrl_mode = 'free'
                elif ctrl_mode == 'free':
                    tkmid.update_free_ctrl(left, right)
                else:
                    tkmid.stop_control()
                    tkmid.start_free_ctrl(left, right)
                    ctrl_mode = 'free'
                print(f"  → 自由控制: left={left} m/s, right={right} m/s")
            elif cmd == 'u':
                tkmid.send_io_cmd(unlock=True)
                print("  → 安全停车解锁已发送")
            elif cmd == 's':
                tkmid.stop_control()
                ctrl_active = False
                ctrl_mode = None
                print("  → 已停止 (零速帧已发送)")
            else:
                print(f"  未知命令: {cmd}")

    except KeyboardInterrupt:
        print("\n中断")
    finally:
        if ctrl_active:
            tkmid.stop_control()
        tkmid.disconnect()


def main():
    parser = create_parser()
    args = parser.parse_args()

    tkmid = TKMidCAN(channel=args.channel, bitrate=args.bitrate)

    if not tkmid.connect():
        print("[FAIL] 无法连接 CAN 总线, 请检查:")
        print("  sudo ip link set can0 type can bitrate 500000")
        print("  sudo ip link set up can0")
        sys.exit(1)

    # 注册 SIGINT 优雅退出
    def graceful_exit(sig, frame):
        print("\n[INFO] 正在停止...")
        tkmid.stop_control()
        tkmid.disconnect()
        sys.exit(0)
    signal.signal(signal.SIGINT, graceful_exit)
    signal.signal(signal.SIGTERM, graceful_exit)

    if args.command is None:
        # 无子命令 → 交互模式
        interactive_mode(tkmid)
    elif args.command == 'motion':
        print(f"[启动] 运动控制 speed={args.speed} angular={args.angular} (Ctrl+C 停止)")
        tkmid.start_ctrl(args.speed, args.angular)
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass
        finally:
            tkmid.stop_control()
    elif args.command == 'free':
        print(f"[启动] 自由控制 left={args.left} right={args.right} (Ctrl+C 停止)")
        tkmid.start_free_ctrl(args.left, args.right)
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass
        finally:
            tkmid.stop_control()
    elif args.command == 'io':
        print(f"[发送] IO 控制 unlock={args.unlock}")
        tkmid.send_io_cmd(args.unlock)
    elif args.command == 'stop':
        print("[发送] 停车指令")
        tkmid.stop_control()

    tkmid.disconnect()
    print("[OK] 已退出")


if __name__ == '__main__':
    main()
