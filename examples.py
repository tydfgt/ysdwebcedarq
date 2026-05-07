#!/usr/bin/env python3
"""
TK-mid CAN 总线 - 使用示例
===========================
演示不同场景下如何调用 tkmid_can 包的各个模块。

运行前请确保:
  1. 已安装 python-can: pip install python-can
  2. 已配置 can0 接口:
     sudo ip link set can0 type can bitrate 500000
     sudo ip link set up can0
"""

import time

# 方式 1: 从包顶层导入 (最常用)
from tkmid_can import TKMidCAN, Gear

# 方式 2: 按需从子模块导入 (低耦合, 适合在其他项目中单独使用)
from tkmid_can.frames import build_ctrl_cmd, build_free_ctrl_cmd, parse_ctrl_fb
from tkmid_can.signal_utils import pack_signal, unpack_signal, calc_bcc
from tkmid_can.cli import setup_can_interface


def example_motion_control():
    """示例 1: 运动控制 — 10ms 周期自动下发, 在线调参"""
    print("\n=== 示例 1: 运动控制 (10ms 自动循环) ===\n")

    tkmid = TKMidCAN(channel='can0')
    if not tkmid.connect():
        return

    # 启动运动控制: 10ms 周期, 心跳自增, BCC 自动计算
    tkmid.start_ctrl(speed=0.5, angular_vel=0.0)
    print("前进 0.5 m/s, 运行 3 秒...")
    time.sleep(3)

    # 在线修改: 加速并左转
    tkmid.update_ctrl(speed=0.8, angular_vel=25.0)
    print("加速 + 左转, 运行 3 秒...")
    time.sleep(3)

    # 在线修改: 减速
    tkmid.update_ctrl(speed=0.2, angular_vel=0.0)
    print("减速, 运行 2 秒...")
    time.sleep(2)

    # 停止 (自动发送零速停车帧)
    tkmid.stop_control()
    print("已停止")

    tkmid.disconnect()


def example_with_feedback():
    """示例 2: 运动控制 + 实时读取反馈"""
    print("\n=== 示例 2: 控制 + 反馈 ===\n")

    tkmid = TKMidCAN(channel='can0')
    if not tkmid.connect():
        return

    tkmid.start_recv()
    tkmid.start_ctrl(speed=0.3, angular_vel=0.0)

    for i in range(5):
        time.sleep(0.5)
        fb = tkmid.get_feedback()
        if fb['ctrl']:
            print(f"反馈 #{i+1}: "
                  f"档位={fb['ctrl']['gear']}, "
                  f"实际速度={fb['ctrl']['speed']:.3f} m/s, "
                  f"实际角速度={fb['ctrl']['angular_vel']:.2f} °/s")

    tkmid.stop_control()
    tkmid.disconnect()


def example_free_control():
    """示例 3: 自由控制模式 (差速, 10ms 自动循环)"""
    print("\n=== 示例 3: 自由控制模式 ===\n")

    tkmid = TKMidCAN(channel='can0')
    if not tkmid.connect():
        return

    # 原地左转: 左轮反转, 右轮正转 (10ms 周期)
    tkmid.start_free_ctrl(left_speed=-0.3, right_speed=0.3)
    print("原地左转 3 秒...")
    time.sleep(3)

    # 在线修改: 前进
    tkmid.update_free_ctrl(left_speed=0.5, right_speed=0.5)
    print("前进 2 秒...")
    time.sleep(2)

    tkmid.stop_control()
    tkmid.disconnect()


def example_ramp_up():
    """示例 4: 平滑加速 (在线逐帧修改速度)"""
    print("\n=== 示例 4: 平滑加速 ===\n")

    tkmid = TKMidCAN(channel='can0')
    if not tkmid.connect():
        return

    tkmid.start_ctrl(speed=0.0, angular_vel=0.0)

    # 从 0 加速到 1.0 m/s, 步长 0.05, 每 50ms 更新一次
    for s in [x * 0.05 for x in range(1, 21)]:
        tkmid.update_ctrl(speed=s)
        print(f"  速度 {s:.2f} m/s")
        time.sleep(0.05)

    print("匀速 2 秒...")
    time.sleep(2)

    # 减速到 0
    for s in [x * 0.05 for x in range(19, -1, -1)]:
        tkmid.update_ctrl(speed=s)
        time.sleep(0.05)

    tkmid.stop_control()
    tkmid.disconnect()


def example_low_coupling():
    """
    示例 5: 低耦合用法 — 不依赖 TKMidCAN 类, 直接使用帧构建/解析函数。

    适用场景:
      - 在其他项目中只需构造 CAN 帧, 用自己的 CAN 驱动发送
      - 只需解析收到的 CAN 帧, 不需要整个通讯类
    """
    print("\n=== 示例 5: 低耦合用法 (纯函数, 可用在任意项目中) ===\n")

    # ---- 5a. 构建帧 ----
    data = build_ctrl_cmd(Gear.MOTION_CTRL, speed=0.5, angular_vel=0.0, alive_counter=3)
    print(f"[构建] 运动控制帧: {data.hex()}")
    print(f"        BCC 校验: 0x{calc_bcc(data):02X}")

    data2 = build_free_ctrl_cmd(Gear.FREE_CTRL, left_speed=0.3, right_speed=0.5, alive_counter=7)
    print(f"[构建] 自由控制帧: {data2.hex()}")
    print(f"        BCC 校验: 0x{calc_bcc(data2):02X}")

    # ---- 5b. 解析反馈帧 (模拟收到的数据) ----
    # 模拟一段收到的运动控制反馈数据
    fake_feedback = bytes([0x13, 0x02, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00])
    # byte0=0x13: gear=3 (bits 0-3), speed low nibble=1 (bit4-7)
    # byte1=0x02: speed high = 2
    # 所以 speed_raw = 0x021? 让我们重新计算...
    # 实际上我们直接用解析函数即可, 它会正确计算

    fb = parse_ctrl_fb(fake_feedback)
    print(f"[解析] 运动反馈: gear={fb['gear']}, "
          f"speed={fb['speed']:.3f} m/s, "
          f"angular={fb['angular_vel']:.2f} °/s")

    # ---- 5c. 手动打包/解包信号 (最底层) ----
    # 如果你想完全控制信号打包
    raw = bytearray(8)
    pack_signal(raw, 0, 0, 4, 3)          # byte0 bit0-3: gear=3
    pack_signal(raw, 0, 4, 16, 150, True) # byte0 bit4 + byte1: speed=0.150 m/s
    pack_signal(raw, 6, 52, 4, 5)         # byte6 bit4-7: alive=5
    raw[7] = calc_bcc(raw)                # byte7: BCC
    print(f"[底层] 手动打包: {bytes(raw).hex()}")
    print(f"       解包 gear={unpack_signal(bytes(raw), 0, 0, 4)}")


if __name__ == '__main__':
    print("TK-mid CAN 总线 - 使用示例")
    print("=" * 50)

    # 先显示配置命令
    print(setup_can_interface())

    # 运行示例 (逐个选择运行)
    try:
        example_motion_control()
        # example_with_feedback()
        # example_free_control()
        # example_ramp_up()
        # example_low_coupling()   # 无需 CAN 硬件, 纯函数演示
    except KeyboardInterrupt:
        print("\n已中断")
