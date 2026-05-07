"""
CAN 信号位操作工具
==================
Intel 格式 (Little-Endian) 的信号打包/解包、BCC 校验等底层工具函数。
纯函数，无外部依赖 (除 Python 标准库外)。
"""


def abs_bit(start_byte: int, start_bit: int) -> int:
    """
    返回 Intel 格式下信号在 CAN 报文中的绝对起始位。

    CAN DBC Intel 格式位编号: byte0 bit0=0, byte0 bit7=7, byte1 bit0=8, ...
    协议中的「起始位」即此绝对位号,「起始字节」= 起始位 // 8。

    参数:
        start_byte: 信号起始字节 (仅做校验用, 不参与计算)
        start_bit:  信号起始位 (绝对位号, 如 0,4,20,52)
    返回:
        绝对位位置 (等价于 start_bit)
    """
    return start_bit


def pack_signal(data: bytearray, start_byte: int, start_bit: int,
                length: int, value: int, signed: bool = False) -> bytearray:
    """
    将一个信号值按 Intel 格式打包到 CAN 数据字节中。

    参数:
        data:       目标 8 字节 bytearray (原地修改)
        start_byte: 起始字节 (0~7)
        start_bit:  起始位 (绝对位号, 协议表中的"起始位")
        length:     信号长度 (bit)
        value:      要写入的值
        signed:     是否为有符号数
    返回:
        原地修改后的 data (便于链式调用)
    """
    pos = abs_bit(start_byte, start_bit)
    mask = (1 << length) - 1
    value = value & mask

    for i in range(length):
        byte_idx = (pos + i) // 8
        bit_idx = (pos + i) % 8
        if value & (1 << i):
            data[byte_idx] |= (1 << bit_idx)
        else:
            data[byte_idx] &= ~(1 << bit_idx)

    return data


def unpack_signal(data: bytes, start_byte: int, start_bit: int,
                  length: int, signed: bool = False) -> int:
    """
    从 CAN 数据字节中按 Intel 格式解包一个信号值。

    参数:
        data:       8 字节数据
        start_byte: 起始字节
        start_bit:  起始位 (绝对位号)
        length:     信号长度 (bit)
        signed:     是否为有符号数 (自动进行符号扩展)
    返回:
        解包后的整数值
    """
    pos = abs_bit(start_byte, start_bit)

    value = 0
    for i in range(length):
        byte_idx = (pos + i) // 8
        bit_idx = (pos + i) % 8
        if data[byte_idx] & (1 << bit_idx):
            value |= (1 << i)

    # 有符号数符号扩展
    if signed and (value & (1 << (length - 1))):
        value -= (1 << length)

    return value


def calc_bcc(data: bytearray | bytes) -> int:
    """
    计算 BCC 异或校验和。

    协议定义: Checksum = Byte0 XOR Byte1 XOR ... XOR Byte6

    参数:
        data: 8 字节数据 (只计算前 7 字节)
    返回:
        8-bit 校验值 (0~255)
    """
    result = 0
    for i in range(7):
        result ^= data[i]
    return result & 0xFF
