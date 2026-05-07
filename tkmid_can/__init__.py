"""
tkmid_can — TK-mid CAN 总线通讯包
===================================

模块化 CAN 总线通讯库, 适用于树莓派 + can0 接口。
各子模块可独立导入, 方便在其他项目中按需耦合。

快速上手:
    from tkmid_can import TKMidCAN

    tkmid = TKMidCAN(channel='can0')
    tkmid.connect()
    tkmid.start_recv()

    # 启动 10ms 周期控制 (心跳自增 + BCC 自动计算)
    tkmid.start_ctrl(speed=0.5, angular_vel=0)
    time.sleep(3)

    # 在线修改参数 (不中断心跳)
    tkmid.update_ctrl(speed=0.8, angular_vel=25.0)
    time.sleep(3)

    # 停止 (自动发送零速停车帧)
    tkmid.stop_control()

    fb = tkmid.get_feedback()

子模块:
    config        — CAN ID、档位枚举、默认参数
    signal_utils  — Intel 格式位打包/解包、BCC 校验
    frames        — 控制帧构建、反馈帧解析
    bus           — CAN 总线通讯层 (TKMidCAN 类)
    cli           — 交互式命令行控制台
"""

from .config import (
    Gear,
    CTRL_CMD_ID, FREE_CTRL_CMD_ID, IO_CMD_ID,
    CTRL_FB_ID, L_WHEEL_FB_ID, R_WHEEL_FB_ID, IO_FB_ID,
    BMS_FB_ID, BMS_FLAG_FB_ID,
    MAX_SPEED, MAX_ANGULAR_VEL,
    DEFAULT_CHANNEL, DEFAULT_BUSTYPE, DEFAULT_BITRATE,
    SEND_PERIOD, IO_FB_PERIOD,
)
from .signal_utils import (
    abs_bit,
    pack_signal,
    unpack_signal,
    calc_bcc,
)
from .frames import (
    build_ctrl_cmd,
    build_free_ctrl_cmd,
    build_io_cmd,
    build_park_cmd,
    build_stop_cmd,
    parse_ctrl_fb,
    parse_wheel_fb,
    parse_io_fb,
    clamp_speed,
    clamp_angular,
)
from .bus import TKMidCAN

__all__ = [
    # 核心类
    'TKMidCAN',
    # 配置
    'Gear',
    'CTRL_CMD_ID', 'FREE_CTRL_CMD_ID', 'IO_CMD_ID',
    'CTRL_FB_ID', 'L_WHEEL_FB_ID', 'R_WHEEL_FB_ID', 'IO_FB_ID',
    'BMS_FB_ID', 'BMS_FLAG_FB_ID',
    'MAX_SPEED', 'MAX_ANGULAR_VEL',
    'DEFAULT_CHANNEL', 'DEFAULT_BUSTYPE', 'DEFAULT_BITRATE',
    'SEND_PERIOD', 'IO_FB_PERIOD',
    # 信号工具
    'abs_bit', 'pack_signal', 'unpack_signal', 'calc_bcc',
    # 限幅工具
    'clamp_speed', 'clamp_angular',
    # 帧构建
    'build_ctrl_cmd', 'build_free_ctrl_cmd', 'build_io_cmd',
    'build_park_cmd', 'build_stop_cmd',
    # 帧解析
    'parse_ctrl_fb', 'parse_wheel_fb', 'parse_io_fb',
]
