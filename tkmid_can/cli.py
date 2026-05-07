#!/usr/bin/env python3
"""
TK-mid CAN 总线 - 交互式控制台
================================
可直接运行: python -m tkmid_can.cli
"""

from .bus import TKMidCAN


def setup_can_interface(interface: str = 'can0', bitrate: int = 500000):
    """返回配置 CAN0 接口的 shell 命令 (需要 sudo 执行)。"""
    return f"""
# 配置 CAN0 接口 (在终端中执行):
sudo ip link set {interface} down
sudo ip link set {interface} type can bitrate {bitrate}
sudo ip link set up {interface}

# 验证接口状态:
ip -details link show {interface}
"""


def interactive_demo():
    """交互式演示模式"""
    tkmid = TKMidCAN(channel='can0')

    if not tkmid.connect():
        print(setup_can_interface())
        return

    tkmid.start_recv()

    print("\n" + "=" * 50)
    print("TK-mid CAN 通讯交互式控制台")
    print("=" * 50)
    print("命令 (控制帧自动以 10ms 周期循环下发):")
    print("  w <speed> <angular>  - 运动控制, 在线修改 (例: w 0.5 0)")
    print("  f <left> <right>     - 自由控制, 在线修改 (例: f 0.3 0.5)")
    print("  s                    - 停止 (自动发零速停车帧)")
    print("  p                    - 驻车")
    print("  i                    - 查看反馈数据")
    print("  u                    - IO 安全停车解锁")
    print("  q                    - 退出")
    print("=" * 50 + "\n")

    ctrl_active = False   # 追踪控制循环是否已启动
    ctrl_mode = None      # 'motion' or 'free'

    try:
        while True:
            cmd = input("> ").strip().split()
            if not cmd:
                continue

            if cmd[0] == 'q':
                break
            elif cmd[0] == 'w' and len(cmd) >= 3:
                speed = float(cmd[1])
                angular = float(cmd[2])
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
                print(f"  -> 运动控制 (10ms周期): speed={speed} m/s, angular={angular} °/s")
            elif cmd[0] == 'f' and len(cmd) >= 3:
                left = float(cmd[1])
                right = float(cmd[2])
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
                print(f"  -> 自由控制 (10ms周期): left={left} m/s, right={right} m/s")
            elif cmd[0] == 's':
                tkmid.stop_control()
                ctrl_active = False
                ctrl_mode = None
                print("  -> 已停止 (零速停车帧已发送)")
            elif cmd[0] == 'p':
                tkmid.send_park()
                print("  -> 已发送驻车指令")
            elif cmd[0] == 'i':
                fb = tkmid.get_feedback()
                print("  --- 反馈数据 ---")
                for k, v in fb.items():
                    print(f"  {k}: {v}")
            elif cmd[0] == 'u':
                tkmid.send_io_cmd(unlock=True)
                print("  -> 已发送 IO 安全停车解锁")
            else:
                print("  未知命令, 请输入 h 查看帮助 (或 q 退出)")

    except KeyboardInterrupt:
        print("\n[INFO] 收到中断信号")
    finally:
        tkmid.disconnect()
        print("已退出")


if __name__ == '__main__':
    interactive_demo()
