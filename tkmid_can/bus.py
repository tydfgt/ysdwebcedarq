"""
CAN 总线通讯层
==============
封装 python-can 的发送/接收、周期性发送线程、反馈接收线程。
依赖: config, frames, signal_utils
"""

import time
import threading
import can

from .config import (
    Gear,
    CTRL_CMD_ID, FREE_CTRL_CMD_ID, IO_CMD_ID,
    CTRL_FB_ID, L_WHEEL_FB_ID, R_WHEEL_FB_ID, IO_FB_ID,
    DEFAULT_CHANNEL, DEFAULT_BUSTYPE, DEFAULT_BITRATE, SEND_PERIOD,
)
from .frames import (
    build_ctrl_cmd, build_free_ctrl_cmd, build_io_cmd,
    build_park_cmd, build_stop_cmd,
    parse_ctrl_fb, parse_wheel_fb, parse_io_fb,
)
from .signal_utils import calc_bcc


class TKMidCAN:
    """
    TK-mid CAN 总线通讯控制器。

    使用方式:
        tkmid = TKMidCAN(channel='can0')
        tkmid.connect()
        tkmid.start_recv()           # 启动接收线程
        tkmid.send_ctrl_cmd(0.5, 0)  # 发送运动控制指令
        fb = tkmid.get_feedback()    # 获取反馈数据
        tkmid.disconnect()
    """

    def __init__(self, channel: str = DEFAULT_CHANNEL,
                 bustype: str = DEFAULT_BUSTYPE,
                 bitrate: int = DEFAULT_BITRATE):
        self.channel = channel
        self.bustype = bustype
        self.bitrate = bitrate
        self.bus: can.Bus | None = None
        self.alive_counter = 0          # 心跳计数器 (4-bit, 0~15)
        self._recv_running = False
        self._ctrl_running = False      # 控制循环独立标志
        self._recv_thread: threading.Thread | None = None
        self._ctrl_thread: threading.Thread | None = None
        self._send_period = SEND_PERIOD
        self._ctrl_lock = threading.Lock()

        # 当前控制参数 (由控制循环线程读取)
        self._ctrl_mode: str | None = None   # 'motion' | 'free' | None
        self._ctrl_speed = 0.0
        self._ctrl_angular = 0.0
        self._ctrl_left = 0.0
        self._ctrl_right = 0.0

        # 反馈数据缓存 (线程安全: 原子替换 dict)
        self.ctrl_fb: dict = {}
        self.l_wheel_fb: dict = {}
        self.r_wheel_fb: dict = {}
        self.io_fb: dict = {}

    # ================================================================
    # 连接 / 断开
    # ================================================================

    def connect(self) -> bool:
        """
        连接 CAN 总线。

        树莓派上需要先配置 can0 接口:
            sudo ip link set can0 type can bitrate 500000
            sudo ip link set up can0
        """
        try:
            self.bus = can.Bus(
                channel=self.channel,
                bustype=self.bustype,
                bitrate=self.bitrate,
            )
            print(f"[OK] 已连接到 CAN 总线: {self.channel} @ {self.bitrate // 1000}Kbps")
            return True
        except OSError as e:
            print(f"[ERROR] 无法连接 CAN 总线: {e}")
            print("  请确认已执行以下命令:")
            print("    sudo ip link set can0 type can bitrate 500000")
            print("    sudo ip link set up can0")
            return False
        except Exception as e:
            print(f"[ERROR] 连接失败: {e}")
            return False

    def disconnect(self):
        """断开 CAN 总线, 停止所有线程。"""
        self.stop_control()
        self._recv_running = False
        if self._recv_thread and self._recv_thread.is_alive():
            self._recv_thread.join(timeout=2)
        if self.bus:
            self.bus.shutdown()
            print("[OK] CAN 总线已断开")

    @property
    def is_connected(self) -> bool:
        """是否已连接"""
        return self.bus is not None

    # ================================================================
    # 心跳计数器
    # ================================================================

    def _next_alive(self) -> int:
        """获取下一个心跳计数值 (4-bit, 0~15 循环)。"""
        self.alive_counter = (self.alive_counter + 1) & 0x0F
        return self.alive_counter

    def reset_alive(self):
        """重置心跳计数器 (通常在停车/模式切换时调用)。"""
        self.alive_counter = 0

    # ================================================================
    # 发送（单帧）
    # ================================================================

    def send_frame(self, arb_id: int, data: bytes):
        """发送单帧 CAN 报文 (底层方法)。"""
        if self.bus is None:
            print("[WARN] 总线未连接")
            return False
        msg = can.Message(
            arbitration_id=arb_id,
            data=data,
            is_extended_id=True,
        )
        try:
            self.bus.send(msg)
            return True
        except can.CanError as e:
            print(f"[ERROR] 发送失败: {e}")
            return False

    def send_ctrl_cmd(self, speed: float, angular_vel: float):
        """发送运动控制指令 (档位=MOTION_CTRL, 自动心跳)。"""
        data = build_ctrl_cmd(Gear.MOTION_CTRL, speed, angular_vel, self._next_alive())
        return self.send_frame(CTRL_CMD_ID, data)

    def send_free_ctrl_cmd(self, left_speed: float, right_speed: float):
        """发送自由控制指令 (档位=FREE_CTRL, 自动心跳)。"""
        data = build_free_ctrl_cmd(Gear.FREE_CTRL, left_speed, right_speed, self._next_alive())
        return self.send_frame(FREE_CTRL_CMD_ID, data)

    def send_io_cmd(self, unlock: bool = False):
        """发送 IO 控制指令 (安全停车解锁)。"""
        data = build_io_cmd(unlock, self._next_alive())
        return self.send_frame(IO_CMD_ID, data)

    def send_stop(self):
        """发送停车指令 (发送两种控制帧的零速版本 + 驻车)。"""
        self.alive_counter = 0
        data1 = build_stop_cmd(Gear.MOTION_CTRL, self._next_alive())
        data2 = build_stop_cmd(Gear.FREE_CTRL, self._next_alive())
        self.send_frame(CTRL_CMD_ID, data1)
        self.send_frame(FREE_CTRL_CMD_ID, data2)

    def send_park(self):
        """发送驻车指令。"""
        data = build_park_cmd(self._next_alive())
        return self.send_frame(CTRL_CMD_ID, data)

    # ================================================================
    # 接收线程
    # ================================================================

    def start_recv(self):
        """启动后台接收线程。"""
        if self._recv_thread and self._recv_thread.is_alive():
            print("[WARN] 接收线程已在运行")
            return
        self._recv_thread = threading.Thread(
            target=self._recv_loop, daemon=True, name="CAN-Recv"
        )
        self._recv_thread.start()
        print("[OK] 接收线程已启动")

    def _recv_loop(self):
        """接收线程主循环。"""
        self._recv_running = True
        while self._recv_running:
            if self.bus is None:
                time.sleep(0.1)
                continue
            try:
                msg = self.bus.recv(timeout=0.1)
                if msg is None:
                    continue
                self._handle_message(msg)
            except can.CanError as e:
                print(f"[ERROR] 接收错误: {e}")
            except Exception as e:
                print(f"[ERROR] 未知错误: {e}")

    def _handle_message(self, msg: can.Message):
        """根据 CAN ID 分发处理收到的报文。"""
        data = bytes(msg.data)
        arb_id = msg.arbitration_id

        if arb_id == CTRL_FB_ID:
            self.ctrl_fb = parse_ctrl_fb(data)
        elif arb_id == L_WHEEL_FB_ID:
            self.l_wheel_fb = parse_wheel_fb(data)
        elif arb_id == R_WHEEL_FB_ID:
            self.r_wheel_fb = parse_wheel_fb(data)
        elif arb_id == IO_FB_ID:
            self.io_fb = parse_io_fb(data)

    def get_feedback(self) -> dict:
        """
        获取所有最新的反馈数据。

        返回:
            dict: {
                'ctrl':     {...},   # 运动控制反馈
                'l_wheel':  {...},   # 左轮反馈
                'r_wheel':  {...},   # 右轮反馈
                'io':       {...},   # IO 反馈
            }
        """
        return {
            'ctrl':    dict(self.ctrl_fb),
            'l_wheel': dict(self.l_wheel_fb),
            'r_wheel': dict(self.r_wheel_fb),
            'io':      dict(self.io_fb),
        }

    def get_speed(self) -> float | None:
        """快捷方法: 获取当前实际线速度 (m/s), 无数据返回 None。"""
        return self.ctrl_fb.get('speed')

    def get_angular_vel(self) -> float | None:
        """快捷方法: 获取当前实际角速度 (°/s), 无数据返回 None。"""
        return self.ctrl_fb.get('angular_vel')

    # ================================================================
    # 控制循环 (10ms 周期, 心跳自动递增 + BCC 每帧重算)
    # ================================================================

    def start_ctrl(self, speed: float = 0.0, angular_vel: float = 0.0):
        """
        启动运动学控制模式 — 以 10ms 周期持续下发 ctrl_cmd 帧。

        每次循环自动: 递增 AliveCounter、重算 BCC 校验。
        调用 update_ctrl() 可在线修改速度/角速度。
        调用 stop_control() 停止并自动下发零速停车帧。

        参数:
            speed:       目标线速度 (m/s)
            angular_vel: 目标角速度 (°/s), 左正右负
        """
        if self._ctrl_running:
            self.update_ctrl(speed, angular_vel)
            return

        with self._ctrl_lock:
            self._ctrl_mode = 'motion'
            self._ctrl_speed = speed
            self._ctrl_angular = angular_vel
            self.reset_alive()

        self._ctrl_running = True
        self._ctrl_thread = threading.Thread(
            target=self._ctrl_loop, daemon=True, name="CAN-Ctrl"
        )
        self._ctrl_thread.start()
        print(f"[OK] 运动控制已启动 (speed={speed} m/s, angular={angular_vel} °/s, 周期=10ms)")

    def start_free_ctrl(self, left_speed: float = 0.0, right_speed: float = 0.0):
        """
        启动自由控制模式 — 以 10ms 周期持续下发 free_ctrl_cmd 帧。
        """
        if self._ctrl_running:
            self.update_free_ctrl(left_speed, right_speed)
            return

        with self._ctrl_lock:
            self._ctrl_mode = 'free'
            self._ctrl_left = left_speed
            self._ctrl_right = right_speed
            self.reset_alive()

        self._ctrl_running = True
        self._ctrl_thread = threading.Thread(
            target=self._ctrl_loop, daemon=True, name="CAN-Ctrl"
        )
        self._ctrl_thread.start()
        print(f"[OK] 自由控制已启动 (left={left_speed} m/s, right={right_speed} m/s, 周期=10ms)")

    def update_ctrl(self, speed: float, angular_vel: float = 0.0):
        """
        在线更新运动控制参数 (速度/角速度)。
        不影响心跳连续性。
        """
        if self._ctrl_mode != 'motion':
            print("[WARN] 当前非运动控制模式, 请先调用 start_ctrl()")
            return
        with self._ctrl_lock:
            self._ctrl_speed = speed
            self._ctrl_angular = angular_vel

    def update_free_ctrl(self, left_speed: float, right_speed: float):
        """
        在线更新自由控制参数 (左右轮速)。
        """
        if self._ctrl_mode != 'free':
            print("[WARN] 当前非自由控制模式, 请先调用 start_free_ctrl()")
            return
        with self._ctrl_lock:
            self._ctrl_left = left_speed
            self._ctrl_right = right_speed

    def stop_control(self):
        """
        停止控制循环。
        自动下发零速停车帧 (两种模式各一帧) 后退出线程。
        """
        if not self._ctrl_running:
            return
        self._ctrl_running = False
        if self._ctrl_thread and self._ctrl_thread.is_alive():
            self._ctrl_thread.join(timeout=2)
        with self._ctrl_lock:
            self._ctrl_mode = None

    def _ctrl_loop(self):
        """
        控制循环主线程。
        每 10ms 构建新帧 (含心跳递增、BCC 重算) 并发送。
        """
        while self._ctrl_running:
            with self._ctrl_lock:
                mode = self._ctrl_mode
                if mode == 'motion':
                    speed = self._ctrl_speed
                    angular = self._ctrl_angular
                    data = build_ctrl_cmd(Gear.MOTION_CTRL, speed, angular, self._next_alive())
                    arb_id = CTRL_CMD_ID
                elif mode == 'free':
                    left = self._ctrl_left
                    right = self._ctrl_right
                    data = build_free_ctrl_cmd(Gear.FREE_CTRL, left, right, self._next_alive())
                    arb_id = FREE_CTRL_CMD_ID
                else:
                    data = None
                    arb_id = None

            if data is not None and arb_id is not None:
                self.send_frame(arb_id, data)

            time.sleep(self._send_period)

        # 退出循环: 自动发送零速停车帧
        with self._ctrl_lock:
            self.reset_alive()
            stop_data1 = build_stop_cmd(Gear.MOTION_CTRL, self._next_alive())
            stop_data2 = build_stop_cmd(Gear.FREE_CTRL, self._next_alive())
        self.send_frame(CTRL_CMD_ID, stop_data1)
        self.send_frame(FREE_CTRL_CMD_ID, stop_data2)
        self._ctrl_mode = None
        print("[OK] 控制循环已停止 (已发送零速停车帧)")

    @property
    def is_controlling(self) -> bool:
        """是否正在下发控制帧"""
        return self._ctrl_running
