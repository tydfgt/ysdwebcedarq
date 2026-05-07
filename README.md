# TK-mid CAN 总线通讯程序

基于树莓派 + SocketCAN 的 TK-mid 底盘 CAN 总线控制程序，支持运动学控制、自由控制（差速）、IO 控制，实时解析反馈帧。

## 硬件要求

- 树莓派 (Raspberry Pi)，已启用 CAN 接口
- TK-mid 底盘 CAN 线已正确连接

## 快速开始

### 1. 安装依赖

```bash
pip install -r requirements.txt
```

### 2. 配置 CAN0 接口

```bash
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0

# 验证
ip -details link show can0
```

### 3. 使用

**一个终端接收反馈，一个终端发送指令：**

```bash
# 终端 1 — 监控反馈帧
python can_reader.py

# 终端 2 — 发送控制指令
python can_writer.py motion --speed 0.5
```

---

## 程序说明

### `can_writer.py` — 发送程序

以 10ms 周期持续下发控制帧（心跳自动递增、BCC 每帧重算）。

```bash
# 命令行模式
python can_writer.py motion --speed 0.5                 # 前进 0.5 m/s
python can_writer.py motion --speed 0.3 --angular 25     # 前进 + 左转
python can_writer.py free --left -0.3 --right 0.3        # 原地左转
python can_writer.py io --unlock                         # 安全停车解锁
python can_writer.py stop                                # 停止

# 交互模式
python can_writer.py
```

交互模式命令：

| 命令 | 说明 | 示例 |
|------|------|------|
| `w <速度> [角速度]` | 运动控制 | `w 0.5` / `w 0.3 25` |
| `f <左轮> <右轮>` | 自由控制 | `f -0.3 0.3` |
| `u` | 安全停车解锁 | `u` |
| `s` | 停止 | `s` |
| `q` | 退出 | `q` |

### `can_reader.py` — 接收程序

实时监听并解析反馈帧。

```bash
# 监听所有反馈帧
python can_reader.py

# 只看运动控制反馈
python can_reader.py --id ctrl_fb

# 只看轮速
python can_reader.py --id l_wheel

# 原始十六进制输出
python can_reader.py --raw

# 记录到日志文件
python can_reader.py --log can_data.log

# 退出时显示统计
python can_reader.py --stats
```

输出示例：
```
[CTRL_FB] 档位=3 速度=+1.000m/s 角速度=+0.00°/s 心跳=1 BCC=0xAD
[L_WHEEL] L轮速=+0.520m/s 脉冲=12345 心跳=3 BCC=0xA1
[R_WHEEL] R轮速=+0.510m/s 脉冲=12340 心跳=3 BCC=0x9F
```

---

## 协议概要

| 参数 | 值 |
|------|-----|
| 标准 | CAN 2.0B |
| 波特率 | 500K |
| 报文格式 | Intel (小端) |
| 控制帧周期 | 10ms |

### CAN ID 速查

| ID | 方向 | 说明 |
|----|------|------|
| `0x18C4D1D0` | 发送 | 运动控制指令 |
| `0x18C4D2D0` | 发送 | 自由控制指令 |
| `0x18C4D7D0` | 发送 | IO 控制指令 |
| `0x18C4D1EF` | 接收 | 运动控制反馈 |
| `0x18C4D7EF` | 接收 | 左轮反馈 |
| `0x18C4D8EF` | 接收 | 右轮反馈 |
| `0x18C4DAEF` | 接收 | IO 控制反馈 |
| `0x18C4E1EF` | 接收 | 电池状态反馈 |
| `0x18C4E2EF` | 接收 | 电池标志反馈 |

### 校验规则

```
BCC = Byte0 XOR Byte1 XOR Byte2 XOR Byte3 XOR Byte4 XOR Byte5 XOR Byte6
```

心跳计数器 (AliveCounter) 占 4 位，每帧 +1（0~15 循环），用于检测丢包。

---

## API 参考（编程调用）

```python
from tkmid_can import TKMidCAN, Gear
import time

tkmid = TKMidCAN(channel='can0')
tkmid.connect()
tkmid.start_recv()

# 启动 10ms 周期控制
tkmid.start_ctrl(speed=0.5, angular_vel=0)

time.sleep(3)

# 在线修改参数（心跳不中断）
tkmid.update_ctrl(speed=0.8, angular_vel=25.0)

time.sleep(3)

# 停止（自动发送零速停车帧）
tkmid.stop_control()

# 获取反馈
fb = tkmid.get_feedback()
print(fb['ctrl']['speed'])       # 当前实际速度 m/s
print(fb['ctrl']['angular_vel']) # 当前实际角速度 °/s

tkmid.disconnect()
```

### 低耦合用法（只用帧构建/解析）

```python
from tkmid_can.frames import build_ctrl_cmd, parse_ctrl_fb
from tkmid_can.config import Gear

# 构造一帧
data = build_ctrl_cmd(Gear.MOTION_CTRL, speed=1.0, angular_vel=0, alive_counter=1)
print(data.hex())  # 833e0000000010ad

# 解析反馈
fb = parse_ctrl_fb(data)
print(fb['speed'])  # 1.0
```

### TKMidCAN 主要方法

| 方法 | 说明 |
|------|------|
| `connect()` | 连接 CAN 总线 |
| `disconnect()` | 断开连接 |
| `start_recv()` | 启动接收线程 |
| `start_ctrl(speed, angular)` | 启动运动控制 (10ms 循环) |
| `update_ctrl(speed, angular)` | 在线修改运动参数 |
| `start_free_ctrl(left, right)` | 启动自由控制 (10ms 循环) |
| `update_free_ctrl(left, right)` | 在线修改自由控制参数 |
| `stop_control()` | 停止控制循环（自动发零速帧） |
| `send_io_cmd(unlock)` | 发送 IO 控制指令 |
| `get_feedback()` | 获取所有反馈数据 |

---

## 项目结构

```
midcantest/
├── README.md
├── requirements.txt
├── can.md                  # 完整协议文档
├── can_writer.py           # 发送程序（独立运行）
├── can_reader.py           # 接收程序（独立运行）
├── examples.py             # 编程示例
├── tkmid_can.py            # 向后兼容包装器
└── tkmid_can/              # 底层通讯库
    ├── __init__.py         # 包入口，统一导出
    ├── config.py           # CAN ID、Gear 枚举、默认参数
    ├── signal_utils.py     # Intel 格式位打包/解包、BCC 校验
    ├── frames.py           # 控制帧构建 & 反馈帧解析
    ├── bus.py              # CAN 总线通讯层 (TKMidCAN 类)
    └── cli.py              # 交互式控制台
```
