"""
CAN 帧构建与解析
=================
控制帧构建 (Ctrl / Free / IO) & 反馈帧解析。
依赖: config, signal_utils
"""

from .config import Gear
from .signal_utils import pack_signal, unpack_signal, calc_bcc


# ============================================================
# 控制帧构建
# ============================================================

def build_ctrl_cmd(gear: Gear, speed: float, angular_vel: float,
                   alive_counter: int) -> bytes:
    """
    构建运动控制指令帧 (ID: 0x18C4D1D0)。

    协议参考: can.md §3.1

    参数:
        gear:         目标档位 (Gear.MOTION_CTRL=3)
        speed:        目标线速度 (m/s), 精度 0.001
        angular_vel:  目标角速度 (°/s), 精度 0.01, 左转为正
        alive_counter: 心跳计数值 (0~15, 外部管理以保持连续)
    返回:
        8 字节 CAN 数据
    """
    data = bytearray(8)

    # 档位: byte0, bit0, length 4, unsigned
    pack_signal(data, 0, 0, 4, int(gear))
    # 目标线速度: byte0, bit4, length 16, signed (0.001 m/s/bit)
    speed_raw = int(speed / 0.001)
    pack_signal(data, 0, 4, 16, speed_raw, signed=True)
    # 目标角速度: byte2, bit20, length 16, signed (0.01 °/s/bit)
    angular_raw = int(angular_vel / 0.01)
    pack_signal(data, 2, 20, 16, angular_raw, signed=True)
    # 心跳: byte6, bit52, length 4
    pack_signal(data, 6, 52, 4, alive_counter & 0x0F)
    # BCC: byte7
    data[7] = calc_bcc(data)

    return bytes(data)


def build_free_ctrl_cmd(gear: Gear, left_speed: float,
                        right_speed: float, alive_counter: int) -> bytes:
    """
    构建自由控制指令帧 (ID: 0x18C4D2D0)。

    协议参考: can.md §3.2

    参数:
        gear:         目标档位 (应为 Gear.FREE_CTRL=4)
        left_speed:   左轮目标速度 (m/s), 精度 0.001
        right_speed:  右轮目标速度 (m/s), 精度 0.001
        alive_counter: 心跳计数值
    返回:
        8 字节 CAN 数据
    """
    data = bytearray(8)

    # 档位: byte0, bit0, length 4
    pack_signal(data, 0, 0, 4, int(gear))
    # 左轮目标速度: byte0, bit4, length 16, signed
    left_raw = int(left_speed / 0.001)
    pack_signal(data, 0, 4, 16, left_raw, signed=True)
    # 右轮目标速度: byte2, bit20, length 16, signed
    right_raw = int(right_speed / 0.001)
    pack_signal(data, 2, 20, 16, right_raw, signed=True)
    # 心跳: byte6, bit52
    pack_signal(data, 6, 52, 4, alive_counter & 0x0F)
    # BCC: byte7
    data[7] = calc_bcc(data)

    return bytes(data)


def build_io_cmd(unlock: bool, alive_counter: int) -> bytes:
    """
    构建 IO 控制指令帧 (ID: 0x18C4D7D0)。

    协议参考: can.md §3.3

    参数:
        unlock:        安全停车解锁: True=解锁使能, False=无效
        alive_counter: 心跳计数值
    返回:
        8 字节 CAN 数据
    """
    data = bytearray(8)

    # 安全停车解锁开关: byte0, bit1, length 1
    pack_signal(data, 0, 1, 1, 1 if unlock else 0)
    # 心跳: byte6, bit52
    pack_signal(data, 6, 52, 4, alive_counter & 0x0F)
    # BCC: byte7
    data[7] = calc_bcc(data)

    return bytes(data)


def build_park_cmd(alive_counter: int) -> bytes:
    """
    构建驻车指令帧 (档位=1, 其余为 0)。

    参数:
        alive_counter: 心跳计数值
    返回:
        8 字节 CAN 数据
    """
    data = bytearray(8)
    pack_signal(data, 0, 0, 4, int(Gear.PARK))
    pack_signal(data, 6, 52, 4, alive_counter & 0x0F)
    data[7] = calc_bcc(data)
    return bytes(data)


def build_stop_cmd(gear: Gear, alive_counter: int) -> bytes:
    """
    构建零速控制帧 (档位不变, 速度归零)。

    参数:
        gear:         档位 (MOTION_CTRL 或 FREE_CTRL)
        alive_counter: 心跳计数值
    返回:
        8 字节 CAN 数据
    """
    data = bytearray(8)
    pack_signal(data, 0, 0, 4, int(gear))
    pack_signal(data, 6, 52, 4, alive_counter & 0x0F)
    data[7] = calc_bcc(data)
    return bytes(data)


# ============================================================
# 反馈帧解析
# ============================================================

def parse_ctrl_fb(data: bytes) -> dict:
    """
    解析运动控制状态-反馈帧 (ID: 0x18C4D1EF)。

    协议参考: can.md §3.4

    返回:
        dict: {
            'gear':        当前档位反馈 (0~4),
            'speed':       当前车体线速度 (m/s),
            'angular_vel': 当前车体角速度 (°/s),
            'alive':       心跳计数值,
            'bcc':         校验值,
        }
    """
    return {
        'gear':        unpack_signal(data, 0, 0, 4),
        'speed':       unpack_signal(data, 0, 4, 16, signed=True) * 0.001,
        'angular_vel': unpack_signal(data, 2, 20, 16, signed=True) * 0.01,
        'alive':       unpack_signal(data, 6, 52, 4),
        'bcc':         data[7],
    }


def parse_wheel_fb(data: bytes) -> dict:
    """
    解析轮系控制状态-反馈帧 (ID: 0x18C4D7EF 左 / 0x18C4D8EF 右)。

    协议参考: can.md §3.5

    返回:
        dict: {
            'speed':       轮速 (m/s),
            'pulse_count': 脉冲数,
            'alive':       心跳计数值,
            'bcc':         校验值,
        }
    """
    return {
        'speed':       unpack_signal(data, 0, 0, 16, signed=True) * 0.001,
        'pulse_count': unpack_signal(data, 2, 16, 32, signed=True),
        'alive':       unpack_signal(data, 6, 52, 4),
        'bcc':         data[7],
    }


def parse_io_fb(data: bytes) -> dict:
    """
    解析 IO 控制状态-反馈帧 (ID: 0x18C4DAEF)。

    协议参考: can.md §3.6

    返回:
        dict: {
            'raw':   原始数据 hex 字符串,
            'alive': 心跳计数值,
            'bcc':   校验值,
        }
    """
    return {
        'raw':   data.hex(),
        'alive': unpack_signal(data, 6, 52, 4),
        'bcc':   data[7],
    }
