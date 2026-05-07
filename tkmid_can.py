#!/usr/bin/env python3
"""
TK-mid CAN 总线通讯 — 兼容包装器
==================================
此文件为向后兼容的薄封装, 实际逻辑已拆分到 tkmid_can/ 包中。

新项目建议直接使用:
    from tkmid_can import TKMidCAN, Gear, ...
    from tkmid_can.config import CTRL_CMD_ID, ...
    from tkmid_can.frames import build_ctrl_cmd, parse_ctrl_fb, ...
    from tkmid_can.signal_utils import pack_signal, unpack_signal, calc_bcc
"""

# 从包中重新导出, 保持旧导入方式兼容
from tkmid_can import *

# 交互式控制台入口 (向后兼容: python tkmid_can.py)
if __name__ == '__main__':
    from tkmid_can.cli import interactive_demo
    interactive_demo()
