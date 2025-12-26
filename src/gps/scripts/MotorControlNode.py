#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import serial
import struct
import time
import os
import math
from typing import Optional, List, Dict, Tuple, Generator, Union
import rospy
from std_msgs.msg import String, Float32
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3
from gps.msg import WTRTK
import threading

# -------------------------- 1. 全局配置 --------------------------
# 串口配置（CAN转USB工具 + SBUS遥控器串口）
# SERIAL_PORT_CAN = "/dev/CAN-pyserial"    # CAN转USB串口（电机控制）
SERIAL_PORT_CAN = "/dev/ttyv0"    # CAN转USB串口（电机控制）
SERIAL_BAUDRATE_CAN = 921600             # CAN串口波特率
SERIAL_TIMEOUT_CAN = 0.1                 # CAN串口超时时间

# SERIAL_PORT_SBUS = "/dev/ttyUSB0"        # SBUS转USB串口（遥控器）
SERIAL_PORT_SBUS = "/dev/ttyv1"        # SBUS转USB串口（遥控器）
SERIAL_BAUDRATE_SBUS = 115200            # SBUS串口波特率（115200,8N1）
SERIAL_TIMEOUT_SBUS = 0.1                # SBUS串口超时时间

# 遥控器配置（SBUS协议）
FRAME_HEADER_SBUS = 0x0F                 # SBUS帧头
FRAME_TAIL1_SBUS = 0x00                  # SBUS帧尾1
FRAME_TAIL2_SBUS = 0x48                  # SBUS校验位
FRAME_LENGTH_SBUS = 33                   # SBUS帧长度（1+30+2）
CHANNEL_COUNT_SBUS = 15                  # 15通道
# 遥控器通道阈值（三挡开关专用）
RC_CH_MIN_VALUE = 282    # 低挡位
RC_CH_MID_VALUE = 1002   # 中挡位
RC_CH_MAX_VALUE = 1722   # 高挡位
RC_CH_HALF_RANGE = 720   # 半量程
DEAD_ZONE_SBUS = 20      # 摇杆死区
MAX_SPEED_REMOTE = 2.0   # 遥控器控制最大角速度（rad/s，可调整）

# RTK导航配置
# RTK_PATH_FILE = rospy.get_param('~rtk_path_file', "/home/ubuntu/rtk_nav/src/gps/cleaning_path/cleaning_path_20251121_173149.txt")
RTK_PATH_FILE = rospy.get_param('~rtk_path_file', "/home/ubuntu/rtk_nav/src/gps/cleaning_path/cleaning_path_20251226_160957.txt")
# RTK_PATH_FILE = rospy.get_param('~rtk_path_file', "/home/ubuntu/rtk_nav/src/gps/cleaning_path/cleaning_path_20251226_155005.txt")
# RTK_PATH_FILE = "/home/ubuntu/rtk_nav/src/gps/cleaning_path/cleaning_path_20251121_173149.txt"  # RTK路径文件
RTK_WAYPOINT_TOLERANCE = 0.5       # 到达目标点的距离 tolerance（米）
RTK_HEADING_TOLERANCE = 0.5         # 到达目标点的航向角容忍度（度）
LINEAR_SPEED_BASE = 0.0124           # 基础线速度（m/s，需根据电机减速比/轮径转换）0.0124m/s>2rad/s
ANGULAR_SPEED_BASE = 0.1             # 调整：航向校准角速度（rad/s）,提高校准效率
INITIAL_MOVE_TOLERANCE = 0.5         # 初始点到第一个航点的到达阈值（米）
IMU_CALIBRATION_TIMEOUT = 3.0        # IMU初始校准超时时间（秒）
HEADING_CALIBRATION_TIMEOUT = 5.0    # 新增：航向校准超时时间（秒），避免卡在转向步骤

# 电机配置（支持多电机差速）
GLOBAL_MOTOR_CONFIG: List[Dict] = [
    {
        "id": 1,                      # 左电机ID（1-127，手册限制）新电机默认127，底层使用1/2/3
        "velocity": 0.0,              # 初始速度（rad/s）
        "current_limit": 20.0,        # 电流限制（A，手册0x7018参数）
        "run_mode": 2                  # 2=速度模式（手册0x7005参数）
    },
    {
        "id": 2,                      # 右电机ID
        "velocity": 0.0,
        "current_limit": 20.0,
        "run_mode": 2
    },
    {
        "id": 3,                      # 滚刷电机ID
        "velocity": 0.0,
        "current_limit": 20.0,
        "run_mode": 2
    }
]

# 协议固定字段（手册定义）
FRAME_HEADER = b'\x41\x54'  # 帧头 "AT"
FRAME_TAIL = b'\x0D\x0A'    # 帧尾 "\r\n"
DATA_LEN = b'\x08'          # 数据帧长度（固定8字节，手册要求）

# 电机参数索引（手册可读写参数列表）
RUN_MODE_INDEX = 0x7005    # 运行模式索引（）
SPEED_REF_INDEX = 0x700A   # 速度指令索引（）
LIMIT_CUR_INDEX = 0x7018   # 电流限制索引（）

# 主机ID与通信类型（手册私有协议）
MOTOR_MASTER_ID = 99       # 主机ID（0-300） 新电机默认253，底层使用99
COMM_TYPE_WRITE = 0x12     # 参数写入（十进制18）
COMM_TYPE_ENABLE = 0x03    # 电机使能（）
COMM_TYPE_RESET = 0x04     # 电机停止（）

# 状态机定义
STATE_DICT = {
    's': "STOP",       # 停止状态（电机失能）
    'f': "FORWARD",    # 前进（直线）
    'b': "BACKWARD",   # 后退（直线）
    'a': "START"       # 启动状态（仅使能电机，不运动）
}
CURRENT_STATE = "STOP"      # 初始状态

# 控制模式标志（通过CH6开关控制，无需全局变量）
class ControlMode:
    REMOTE = "REMOTE"       # 遥控器控制模式（CH6低挡位）
    NORMAL = "NORMAL"       # 正常控制模式（CH6中/高挡位）

# 导航状态枚举（新增：标记当前导航阶段）
class NavState:
    IDLE = "IDLE"                   # 导航未启动
    INITIAL_MOVE = "INITIAL_MOVE"   # 初始点→第一个航点
    WAYPOINT_MOVE = "WAYPOINT_MOVE" # 移动到目标航点
    WAYPOINT_CALIB = "WAYPOINT_CALIB" # 航点航向校准


# -------------------------- 2. SBUS遥控器解析类 --------------------------
class SBUSRemoteParser:
    def __init__(self):
        self.ser_sbus: Optional[serial.Serial] = None
        self.channels = [RC_CH_MID_VALUE] * CHANNEL_COUNT_SBUS  # 通道数据缓存
        self.running = False
        self.read_thread: Optional[threading.Thread] = None
        self.lock = threading.Lock()
        self.current_mode = ControlMode.NORMAL  # 初始模式
        # self.current_mode = ControlMode.REMOTE  # 遥控器模式
        
        # 重连配置  
        self.init_retry_count = 10  # 初始化时最大重连次数，默认5次，设为-1表示无限重试
        self.init_retry_interval = 1  # 初始化重连间隔（秒）
        self.runtime_retry_interval = 2  # 运行中重连间隔（秒）
        self.is_connected = False  # 标记当前串口连接状态

    def init_sbus_serial(self) -> bool:
        """初始化SBUS遥控器串口（115200,8N1）"""
        try:
            if self.ser_sbus and self.ser_sbus.is_open:
                self.ser_sbus.close()
            
            self.ser_sbus = serial.Serial(
                port=SERIAL_PORT_SBUS,
                baudrate=SERIAL_BAUDRATE_SBUS,
                timeout=SERIAL_TIMEOUT_SBUS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS
            )
            
            if self.ser_sbus.is_open:
                rospy.loginfo(f"[SBUSRemote] 串口初始化成功（端口：{SERIAL_PORT_SBUS}，波特率：{SERIAL_BAUDRATE_SBUS}）")
                return True
            rospy.logwarn(f"[SBUSRemote] 串口已创建但未打开")
            return False
        
        except Exception as e:
            rospy.logerr(f"[SBUSRemote] 串口初始化失败：{str(e)}")
            return False
    def _reconnect_serial(self, is_initial: bool = False) -> bool:
        """通用重连逻辑（初始化重连/运行中重连）"""
        retry_count = self.init_retry_count if is_initial else -1  # 运行中无限重连（-1表示无限制）
        retry_interval = self.init_retry_interval if is_initial else self.runtime_retry_interval
        attempt = 0
        
        rospy.loginfo(f"[SBUSRemote] 开始{'初始化' if is_initial else '运行中'}重连，间隔{retry_interval}秒{'，最多尝试' + str(retry_count) + '次' if is_initial else '（无限重试）'}")
        
        while not rospy.is_shutdown():
            attempt += 1
            # 初始化重连：超过次数则退出
            if is_initial and attempt > retry_count:
                rospy.logerr(f"[SBUSRemote] 初始化重连失败（已尝试{retry_count}次），无法启用遥控器控制")
                self.is_connected = False
                return False
            
            # 尝试重连
            rospy.loginfo(f"[SBUSRemote] 第{attempt}次重连...")
            if self.init_sbus_serial():
                rospy.loginfo(f"[SBUSRemote] {'初始化' if is_initial else '运行中'}重连成功！")
                self.is_connected = True
                return True
            
            # 等待重连间隔
            time.sleep(retry_interval)
    def _parse_sbus_frame(self, frame: bytes) -> Optional[list]:
        """解析SBUS帧数据"""
        # 校验帧长度、帧头和尾标
        if len(frame) != FRAME_LENGTH_SBUS:
            return None
        if frame[0] != FRAME_HEADER_SBUS or frame[-2] != FRAME_TAIL1_SBUS:
            return None

        # 解析32字节数据为16个16bit通道值（小端模式）
        channels = []
        for i in range(CHANNEL_COUNT_SBUS):
            byte_start = 1 + 2 * i
            byte_end = byte_start + 2
            value = struct.unpack("<H", frame[byte_start:byte_end])[0]
            # 限制通道值在有效范围
            value = max(RC_CH_MIN_VALUE, min(RC_CH_MAX_VALUE, value))
            channels.append(value)
        return channels

    def _read_sbus_loop(self):
        """SBUS串口读取循环（后台线程）"""
        buffer = b""
        while self.running and not rospy.is_shutdown():
            # 检测串口连接状态：断联则自动重连
            if not self.is_connected or not (self.ser_sbus and self.ser_sbus.is_open):
                rospy.logwarn("[SBUSRemote] 串口断联，启动自动重连...")
                # 运行中重连（无限重试）
                if not self._reconnect_serial(is_initial=False):
                    rospy.logerr("[SBUSRemote] 运行中重连失败，停止读取循环")
                    break
            try:
                if not self.ser_sbus or not self.ser_sbus.is_open:
                    time.sleep(0.5)
                    continue

                data = self.ser_sbus.read(1024)
                if not data:
                    time.sleep(0.001)
                    continue
                buffer += data

                # 查找完整帧
                while len(buffer) >= FRAME_LENGTH_SBUS:
                    header_idx = buffer.find(bytes([FRAME_HEADER_SBUS]))
                    if header_idx == -1:
                        buffer = b""
                        break
                    if header_idx + FRAME_LENGTH_SBUS > len(buffer):
                        buffer = buffer[header_idx:]
                        break
                    
                    # 提取并解析帧
                    frame = buffer[header_idx:header_idx+FRAME_LENGTH_SBUS]
                    buffer = buffer[header_idx+FRAME_LENGTH_SBUS:]
                    channels = self._parse_sbus_frame(frame)
                    if channels:
                        with self.lock:
                            self.channels = channels
                            # 解析CH6状态，切换控制模式
                            self._update_control_mode(self.channels[5])  # CH6是第6个通道（索引5）

            except serial.SerialException as e:
                # 串口通信异常（断联、读写失败等）
                rospy.logerr(f"[SBUSRemote] 串口通信异常：{str(e)}")
                self.is_connected = False
                time.sleep(0.5)  # 等待后进入重连逻辑
            except Exception as e:
                rospy.logerr(f"[SBUSRemote] 读取循环异常：{str(e)}")
                time.sleep(0.1)

    def _update_control_mode(self, ch6_value: int):
        """根据CH6三挡开关状态更新控制模式"""
        if abs(ch6_value - RC_CH_MIN_VALUE) < DEAD_ZONE_SBUS:
            new_mode = ControlMode.REMOTE
        else:
            new_mode = ControlMode.NORMAL
        
        if new_mode != self.current_mode:
            rospy.loginfo(f"[SBUSRemote] 控制模式切换：{self.current_mode} → {new_mode}")
            self.current_mode = new_mode
            # 模式切换时停止电机，避免冲突
            if 'node' in locals():
                node.set_motors_speed(0.0, 0.0)

    def get_control_mode(self) -> str:
        """获取当前控制模式"""
        with self.lock:
            return self.current_mode

    def _map_channel_to_speed(self, channel_value: int) -> float:
        """将通道值映射为电机速度"""
        if abs(channel_value - RC_CH_MID_VALUE) < DEAD_ZONE_SBUS:
            return 0.0
        
        # 线性映射到速度范围
        if channel_value > RC_CH_MID_VALUE:
            ratio = (channel_value - RC_CH_MID_VALUE - DEAD_ZONE_SBUS) / (RC_CH_HALF_RANGE - DEAD_ZONE_SBUS)
            speed = ratio * MAX_SPEED_REMOTE
        else:
            ratio = (channel_value - RC_CH_MID_VALUE + DEAD_ZONE_SBUS) / (DEAD_ZONE_SBUS - RC_CH_HALF_RANGE)
            speed = -ratio * MAX_SPEED_REMOTE
        
        return max(-MAX_SPEED_REMOTE, min(MAX_SPEED_REMOTE, speed))

    def get_remote_speed(self) -> Tuple[float, float]:
        """获取遥控器控制的电机速度（差速控制）"""
        with self.lock:
            ch3 = self.channels[0]  # CH1: 左摇杆前后（前进/后退）
            ch4 = self.channels[2]  # CH3: 右摇杆左右（转向） 
            # 右摇杆左右：CH1,LEFT<1002
            # 左摇杆前后：CH2,up<1002
            # 右摇杆前后：CH3,UP>1002
            # 左摇杆左右：CH4,UP<1002  

        # 映射速度
        forward_speed = self._map_channel_to_speed(ch3)
        turn_speed = self._map_channel_to_speed(ch4)

        # 差速计算
        left_speed = forward_speed - turn_speed
        right_speed = forward_speed + turn_speed

        # 速度限幅
        left_speed = max(-MAX_SPEED_REMOTE, min(MAX_SPEED_REMOTE, left_speed))
        right_speed = max(-MAX_SPEED_REMOTE, min(MAX_SPEED_REMOTE, right_speed))

        return left_speed, right_speed

    def start(self):
        """启动SBUS解析"""
        rospy.loginfo("[SBUSRemote] 启动遥控器解析...")
        # 初始化串口：失败则进入初始化重连（最多尝试init_retry_count次）
        if not self.init_sbus_serial():
            rospy.logwarn("[SBUSRemote] 串口初始化失败，启动初始化重连...")
            if not self._reconnect_serial(is_initial=True):
                return False
        
        self.running = True
        self.read_thread = threading.Thread(target=self._read_sbus_loop)
        self.read_thread.daemon = True
        self.read_thread.start()
        rospy.loginfo("[SBUSRemote] 遥控器解析启动成功")
        return True

    def stop(self):
        """停止SBUS解析"""
        self.running = False
        if self.ser_sbus and self.ser_sbus.is_open:
            self.ser_sbus.close()
        if self.read_thread:
            self.read_thread.join(timeout=2)
        rospy.loginfo("[SBUSRemote] 遥控器解析已停止")


# -------------------------- 3. 核心工具类（串口通信+电机控制） --------------------------
class SerialMotorController:
    def __init__(self, port: str, baudrate: int):
        self.ser: Optional[serial.Serial] = None
        self.serial_port = port
        self.serial_baudrate = baudrate
        self.motors = [m for m in GLOBAL_MOTOR_CONFIG if "id" in m]
        rospy.loginfo(f"[MotorCtrl] 加载{len(self.motors)}个电机配置（ID：{[m['id'] for m in self.motors]}）")
        self.reconnect_interval = 2
        self.max_reconnect_attempts = 0

    # 新增：上下文管理器 - 进入时初始化串口
    def __enter__(self):
        self.init_serial()
        return self

    # 新增：上下文管理器 - 退出时自动关闭串口
    def __exit__(self, exc_type, exc_val, exc_tb):
        if self.ser and self.ser.is_open:
            rospy.loginfo("[MotorCtrl] 上下文管理器自动关闭串口")
            try:
                self.ser.close()
            except Exception as e:
                rospy.logerr(f"[MotorCtrl] 上下文管理器关闭串口失败：{str(e)}")
        # 停止所有电机
        for motor in self.motors:
            try:
                self.motor_set_speed(motor["id"], 0.0)
                self.motor_disable(motor["id"])
            except Exception as e:
                rospy.logwarn(f"[MotorCtrl] 上下文管理器停止电机{motor['id']}失败：{str(e)}")
        return False

    def init_serial(self) -> bool:
        """初始化串口（连接CAN转USB工具）"""
        try:
            # 若已打开，先关闭再重连
            if self.ser and self.ser.is_open:
                self.ser.close()
            
            self.ser = serial.Serial(
                port=self.serial_port,
                baudrate=self.serial_baudrate,
                timeout=SERIAL_TIMEOUT_CAN,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS
            )
            
            if self.ser.is_open:
                rospy.loginfo(f"[MotorCtrl] 串口初始化成功（端口：{self.serial_port}，波特率：{self.serial_baudrate}）")
                return True
            rospy.logwarn(f"[MotorCtrl] 串口已创建但未打开")
            return False
        
        except Exception as e:
            rospy.logerr(f"[MotorCtrl] 串口初始化失败：{str(e)}")
            return False

    def reconnect_serial(self) -> bool:
        """串口重连逻辑（带重试次数控制）"""
        attempts = 0
        while not rospy.is_shutdown():
            attempts += 1
            rospy.loginfo(f"[MotorCtrl] 第{attempts}次尝试重连串口...")
            
            if self.init_serial():
                return True  # 重连成功
            
            # 检查是否达到最大重试次数（0表示无限重试）
            if self.max_reconnect_attempts > 0 and attempts >= self.max_reconnect_attempts:
                rospy.logerr(f"[MotorCtrl] 已达到最大重试次数（{self.max_reconnect_attempts}次），重连失败")
                return False
            
            # 等待重连间隔
            time.sleep(self.reconnect_interval)

    def _convert_can_id_to_serial(self, can_id: int) -> bytes:
        """CAN ID转串口扩展帧（手册规则：29位ID补3个0→大端4字节）（）"""
        can_id_32bit = (can_id << 3)  # 末尾补3个0（手册规定）
        return struct.pack(">I", can_id_32bit)  # 大端存储

    def _build_serial_packet(self, can_id: int, can_data: bytes) -> bytes:
        """构造串口数据包（帧头+扩展帧+数据长度+数据帧+帧尾）"""
        serial_ext_id = self._convert_can_id_to_serial(can_id)
        packet = (
            FRAME_HEADER + serial_ext_id + DATA_LEN + can_data + FRAME_TAIL
        )
        return packet

    def _send_serial_packet(self, packet: bytes, motor_id: int) -> bool:
        """发送串口数据包（带ROS日志）"""
        if not self.ser or not self.ser.is_open:
            rospy.logerr(f"[MotorCtrl] 电机{motor_id}：串口未打开")
            return False
        
        try:
            self.ser.write(packet)
            # 打印数据包（调试用，可注释）
            packet_hex = " ".join([f"{byte:02X}" for byte in packet])
            rospy.logdebug(f"[MotorCtrl] 电机{motor_id}发送：{packet_hex}")
            return True
        except Exception as e:
            rospy.logerr(f"[MotorCtrl] 电机{motor_id}发送失败：{str(e)}")
            return False

    # -------------------------- 电机控制核心接口 --------------------------
    def motor_set_mode(self, motor_id: int, mode: int) -> bool:
        """设置电机模式（手册0x7005参数）（）"""
        can_data = bytearray(8)
        struct.pack_into("<H", can_data, 0, RUN_MODE_INDEX)  # 0x7005（小端）
        struct.pack_into("<B", can_data, 4, mode)            # 模式值
        can_id = (COMM_TYPE_WRITE << 24) | (MOTOR_MASTER_ID << 8) | motor_id
        packet = self._build_serial_packet(can_id, can_data)
        return self._send_serial_packet(packet, motor_id)

    def motor_set_current_limit(self, motor_id: int, current_limit: float) -> bool:
        """设置电流限制（手册0x7018参数）（）"""
        can_data = bytearray(8)
        struct.pack_into("<H", can_data, 0, LIMIT_CUR_INDEX)  # 0x7018（小端）
        struct.pack_into("<f", can_data, 4, current_limit)    # 电流值（float）
        can_id = (COMM_TYPE_WRITE << 24) | (MOTOR_MASTER_ID << 8) | motor_id
        packet = self._build_serial_packet(can_id, can_data)
        return self._send_serial_packet(packet, motor_id)

    def motor_set_speed(self, motor_id: int, speed_rad: float) -> bool:
        """设置电机速度（手册0x700A参数，单位：rad/s）（）"""
        can_data = bytearray(8)
        struct.pack_into("<H", can_data, 0, SPEED_REF_INDEX)  # 0x700A（小端）
        struct.pack_into("<f", can_data, 4, speed_rad)        # 速度值（float）
        can_id = (COMM_TYPE_WRITE << 24) | (MOTOR_MASTER_ID << 8) | motor_id
        packet = self._build_serial_packet(can_id, can_data)
        return self._send_serial_packet(packet, motor_id)

    def motor_enable(self, motor_id: int) -> bool:
        """使能电机（手册通信类型3）（）"""
        can_data = b'\x00' * 8
        can_id = (COMM_TYPE_ENABLE << 24) | (MOTOR_MASTER_ID << 8) | motor_id
        packet = self._build_serial_packet(can_id, can_data)
        return self._send_serial_packet(packet, motor_id)

    def motor_reset(self, motor_id: int) -> bool:
        """停止电机（手册通信类型4，清故障）（）"""
        can_data = b'\x80' + b'\x00' * 7  # Byte0=0x80表示清故障（手册规则）
        can_id = (COMM_TYPE_RESET << 24) | (MOTOR_MASTER_ID << 8) | motor_id
        packet = self._build_serial_packet(can_id, can_data)
        return self._send_serial_packet(packet, motor_id)

    def motor_disable(self, motor_id: int) -> bool:
        """失能电机（停止+复位）"""
        return self.motor_reset(motor_id)


# -------------------------- 4. RTK路径解析与导航类 --------------------------
class RTKNavigator:
    def __init__(self):
        self.waypoints: List[Tuple[float, float, float]] = []  # RTK航点列表（经度，纬度，航向角(度)）
        self.current_waypoint_idx = 0                           # 当前目标航点索引
        self.current_gps: Tuple[float, float] = None  # 当前GPS位置
        self.imu_yaw = 0.0                                      # 初始IMU偏航角（rad），RTK惯导提供
        self.imu_initialized = False                             # IMU是否完成初始校准
        self.imu_calibration_offset = 0.0                        # 自动校准的偏移量（rad）
        self.load_rtk_path()

        # 导航上下文（新增：保存导航状态，支持中断恢复）
        self.nav_context = {
            "nav_state": NavState.IDLE,  # 当前导航阶段
            "target_waypoint": None,     # 当前目标航点（中断时保存）
            "calib_generator": None      # 航向校准生成器（中断时保存）
        }

    def load_rtk_path(self) -> bool:
        """加载RTK路径文件（序号,经度,纬度,航向角(度)）"""
        if not os.path.exists(RTK_PATH_FILE):
            rospy.logerr(f"[RTKNav] RTK路径文件不存在：{RTK_PATH_FILE}")
            return False
        
        try:
            with open(RTK_PATH_FILE, 'r', encoding='utf-8') as f:
                lines = f.readlines()[1:]  # 跳过表头
                for line in lines:
                    line = line.strip()
                    if not line:
                        continue
                    seq, lon, lat, heading_deg = line.split(',')  # 新增航向角字段
                    self.waypoints.append((float(lon), float(lat), float(heading_deg)))
            rospy.loginfo(f"[RTKNav] 加载RTK航点{len(self.waypoints)}个，首个航点：{self.waypoints[0]}")
            return True
        except Exception as e:
            rospy.logerr(f"[RTKNav] 解析RTK文件失败：{str(e)}")
            return False

    def gps_callback(self, msg: NavSatFix) -> None:
        """GPS位置回调（需订阅GPS节点的位置消息）"""
        if msg.status.status < 0:
            rospy.logwarn("[RTKNav] GPS信号无效，忽略当前数据")
            return
        elif msg.status.status ==1:
            rospy.loginfo("[RTKNav] GPS信号弱，单点定位")
        elif msg.status.status ==2:
            rospy.loginfo("[RTKNav] 差分定位")
        elif msg.status.status ==3:
            rospy.loginfo("[RTKNav] RTK定位")
        elif msg.status.status ==4:
            pass
            # rospy.loginfo("[RTKNav] RTK固定解")

        self.current_gps = (msg.longitude, msg.latitude)  # x=经度，y=纬度

    def heading_callback(self, msg: WTRTK) -> None:
        """IMU数据回调（自动校准初始偏移，后续数据基于初始基准修正）"""
        # 1. 提取INS航向角（度，真北0°顺时针）
        ins_heading_deg = msg.ins_heading
        # 2. 转换为rad并归一化到[-π, π]
        ins_heading_rad = math.radians(ins_heading_deg)
        ins_heading_rad = math.fmod(ins_heading_rad + math.pi, 2 * math.pi) - math.pi
        
        # 3. 首次收到IMU数据时，自动校准偏移（以第一帧为基准）
        if not self.imu_initialized:
            # 校准偏移量 = 0 - 第一帧IMU航向角（让初始IMU偏航角为0，与路径基准对齐）
            self.imu_calibration_offset = -ins_heading_rad
            self.imu_initialized = True
            rospy.loginfo(f"[RTKNav] IMU校准完成！初始偏移：{math.degrees(self.imu_calibration_offset):.2f}°")
        
        # 4. 应用自动校准偏移，得到最终IMU偏航角（与初始基准一致）
        self.imu_yaw = ins_heading_rad + self.imu_calibration_offset
        # 再次归一化，避免累积误差
        self.imu_yaw = math.fmod(self.imu_yaw + math.pi, 2 * math.pi) - math.pi
        
        rospy.logdebug(
            f"[RTKNav] INS原始航向：{ins_heading_deg:.2f}° → "
            f"校准偏移：{math.degrees(self.imu_calibration_offset):.2f}° → "
            f"最终偏航角：{math.degrees(self.imu_yaw):.2f}°"
        )

    def get_target_waypoint(self) -> Optional[Tuple[float, float, float]]:
        """获取当前目标航点（含航向角）"""
        if self.current_waypoint_idx >= len(self.waypoints):
            rospy.loginfo("[RTKNav] 已到达最后一个航点")
            return None
        return self.waypoints[self.current_waypoint_idx]

    def calc_distance_to_waypoint(self, waypoint: Tuple[float, float, float]) -> float:
        """使用Haversine公式计算两点经纬度的地表距离（米）"""
        # 地球平均半径（米）
        R = 6371000.0
        
        # 提取当前位置和目标航点的经纬度（度）
        lon1, lat1 = self.current_gps
        lon2, lat2, _ = waypoint
        
        # 转换为弧度
        lon1_rad = math.radians(lon1)
        lat1_rad = math.radians(lat1)
        lon2_rad = math.radians(lon2)
        lat2_rad = math.radians(lat2)
        
        # 计算经纬度差（弧度）
        delta_lon = lon2_rad - lon1_rad
        delta_lat = lat2_rad - lat1_rad
        
        # Haversine公式
        a = math.sin(delta_lat / 2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(delta_lon / 2)** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        
        # 计算距离（米）
        distance = R * c
        return distance

    def get_path_heading(self, waypoint: Tuple[float, float, float]) -> float:
        """获取目标航点的路径航向角（转换为rad并归一化，与IMU基准一致）"""
        # 修正：航点航向角是绝对角度，需叠加IMU校准偏移（让路径航向与IMU基准对齐）
        heading_deg = waypoint[2] + math.degrees(self.imu_calibration_offset)
        heading_rad = math.radians(heading_deg)
        # 归一化到[-π, π]，与IMU偏航角基准一致
        heading_rad = math.fmod(heading_rad + math.pi, 2 * math.pi) - math.pi
        return heading_rad

    def get_heading_error(self, target_heading: float) -> float:
        """新增：计算当前航向与目标航向的误差（归一化到[-π, π]，单位：rad）"""
        heading_error = target_heading - self.imu_yaw
        # 归一化误差，避免跨π/-π的大角度跳转
        heading_error = math.fmod(heading_error + math.pi, 2 * math.pi) - math.pi
        return heading_error

    def get_speed_correction(self, target_heading: float) -> float:
        yaw_error = self.get_heading_error(target_heading)
        yaw_error_deg = math.degrees(abs(yaw_error))  # 误差角度（度）
        # 待测试，需要调整参数
        # 动态kp：大误差用大kp（快速修正），小误差用小kp（避免震荡）
        if yaw_error_deg > 10:  # 误差>10°：快速修正
            kp = 0.8
        elif yaw_error_deg > 3:  # 误差3°~10°：平衡修正
            kp = 0.5
        else:  # 误差<3°：缓慢收敛（避免超调）
            kp = 0.2
        
        correction = kp * yaw_error
        
        # 限制修正量最大值（避免电机速度突变）
        max_correction = 0.3  # 最大修正量（rad/s），可根据电机性能调整
        correction = max(min(correction, max_correction), -max_correction)
        
        rospy.logdebug(
            f"[RTKNav] 路径航向：{math.degrees(target_heading):.2f}° → "
            f"IMU航向：{math.degrees(self.imu_yaw):.2f}° → "
            f"误差：{yaw_error_deg:.2f}° → "
            f"kp：{kp} → 修正量：{correction:.3f}rad/s"
        )
        return correction

    def calibrate_heading_at_waypoint(self, target_heading: float) -> Generator[Tuple[float, float], None, bool]:
        """新增：航点航向校准（到达距离阈值后，原地转向校准航向）"""
        rospy.loginfo(
            f"[RTKNav] 开始航向校准：目标航向{math.degrees(target_heading):.2f}°，当前航向{math.degrees(self.imu_yaw):.2f}°，"
            f"容忍度±{RTK_HEADING_TOLERANCE}°，超时{HEADING_CALIBRATION_TIMEOUT}秒"
        )
        start_time = rospy.Time.now()
        
        while not rospy.is_shutdown():
            # 1. 计算航向误差（rad）和误差角度（deg）
            heading_error_rad = self.get_heading_error(target_heading)
            heading_error_deg = math.degrees(abs(heading_error_rad))
            
            # 2. 航向达标：退出校准
            if heading_error_deg <= RTK_HEADING_TOLERANCE:
                rospy.loginfo(f"[RTKNav] 航向校准完成！最终误差：{heading_error_deg:.2f}°（≤{RTK_HEADING_TOLERANCE}°）")
                return True
            
            # 3. 超时处理：强制退出（避免卡死）
            elapsed_time = (rospy.Time.now() - start_time).to_sec()
            if elapsed_time > HEADING_CALIBRATION_TIMEOUT:
                rospy.logwarn(f"[RTKNav] 航向校准超时！当前误差：{heading_error_deg:.2f}°（＞{RTK_HEADING_TOLERANCE}°），继续下一段导航")
                return True  # 超时仍返回True，不阻断导航
            
            # 4. 原地转向：根据误差方向控制电机（差速转向，不移动,待测试正负）
            # 误差为正：需要顺时针转（右电机反转，左电机正转）
            # 误差为负：需要逆时针转（左电机反转，右电机正转）
            angular_speed = ANGULAR_SPEED_BASE
            if heading_error_rad > 0:
                left_speed = angular_speed
                right_speed = -angular_speed
            else:
                left_speed = -angular_speed
                right_speed = angular_speed
            
            # 5. 返回转向速度指令（由上层调用设置电机）
            yield (left_speed, right_speed)
        
        return False

    def move_to_first_waypoint(self) -> Generator[Tuple[float, float], None, bool]:
        """从当前初始位置移动到第一个航点（含距离+航向校验）"""
        if not self.waypoints:
            rospy.logerr("[RTKNav] 无航点数据，无法执行初始移动")
            return False
        
        # 等待IMU校准完成
        if not self.imu_initialized:
            rospy.loginfo(f"[RTKNav] 等待IMU初始校准（超时{IMU_CALIBRATION_TIMEOUT}秒）...")
            start_time = rospy.Time.now()
            while not self.imu_initialized and not rospy.is_shutdown():
                rospy.sleep(0.05)
                if (rospy.Time.now() - start_time).to_sec() > IMU_CALIBRATION_TIMEOUT:
                    rospy.logwarn("[RTKNav] IMU校准超时，使用默认偏航角0rad")
                    break
                
        first_waypoint = self.waypoints[0]
        self.nav_context["target_waypoint"] = first_waypoint  # 保存目标航点
        rospy.loginfo(f"[RTKNav] 开始从初始位置移动到第一个航点：{first_waypoint[:2]}，目标航向：{first_waypoint[2]}°")
        
        last_distance = 0
        while not rospy.is_shutdown():
            # 1. 计算到第一个航点的距离
            distance = self.calc_distance_to_waypoint(first_waypoint)
            if abs(last_distance - distance) > 0.1:  # 距离变化超过0.1m才打印日志
                rospy.loginfo(f"[RTKNav] 到第一个航点距离：{distance:.2f}m")
                last_distance = distance
            
            # 2. 距离达标：开始航向校准
            if distance < INITIAL_MOVE_TOLERANCE:
                rospy.loginfo(f"[RTKNav] 已到达第一个航点距离阈值（{distance:.2f}m ≤ {INITIAL_MOVE_TOLERANCE}m）")
                target_heading = self.get_path_heading(first_waypoint)
                # 调用航向校准生成器并保存到上下文
                self.nav_context["calib_generator"] = self.calibrate_heading_at_waypoint(target_heading)
                self.nav_context["nav_state"] = NavState.WAYPOINT_CALIB  # 更新导航状态
                while not rospy.is_shutdown():
                    try:
                        left_speed, right_speed = next(self.nav_context["calib_generator"])
                        yield (left_speed, right_speed)
                    except StopIteration:
                        # 校准完成，重置上下文
                        self.nav_context["calib_generator"] = None
                        self.nav_context["target_waypoint"] = None
                        return True
                return True
            
            # 3. 距离未达标：直线行驶+实时纠偏
            target_heading = self.get_path_heading(first_waypoint)
            correction = self.get_speed_correction(target_heading)
            
            # 4. 差速控制：左电机=基础速度-修正量，右电机=基础速度+修正量
            base_speed = LINEAR_SPEED_BASE
            left_speed_rad = self.rad_from_linear(base_speed) - correction
            right_speed_rad = self.rad_from_linear(base_speed) + correction
            
            # 5. 返回速度指令（由上层调用设置电机）
            yield (left_speed_rad, right_speed_rad)
        
        return False

    def rad_from_linear(self, linear_speed: float) -> float:
        """线速度（m/s）转电机角速度（rad/s）：需根据电机减速比/轮径校准"""
        reduction_ratio = 7.75  # 电机减速比（手册0x201f参数）
        wheel_diameter = 0.04874 # 车轮直径（m，需根据实际硬件调整）48.74mm
        return (linear_speed * reduction_ratio) / wheel_diameter

    # 新增：重置导航上下文（重新开始导航时调用）
    def reset_nav_context(self):
        self.current_waypoint_idx = 0
        self.nav_context = {
            "nav_state": NavState.IDLE,
            "target_waypoint": None,
            "calib_generator": None
        }


# -------------------------- 5. ROS节点核心逻辑 --------------------------
class MotorControlNode:
    def __init__(self):
        rospy.init_node('motor_control_node', anonymous=True)
        self.rate = rospy.Rate(50)  # 提高到50Hz，适配遥控器控制频率

        # 初始化核心模块
        self.motor_ctrl = SerialMotorController(SERIAL_PORT_CAN, SERIAL_BAUDRATE_CAN)
        rospy.loginfo("[ROSNode] 开始初始化CAN串口...")
        if not self.motor_ctrl.init_serial():
            rospy.logwarn("[ROSNode] CAN串口首次初始化失败，进入重连模式")
            if not self.motor_ctrl.reconnect_serial():
                rospy.logfatal("[ROSNode] CAN串口重连失败，无法继续运行，退出节点")
                rospy.signal_shutdown("CAN串口重连失败")
                return
            
        # 初始化遥控器解析模块
        self.sbus_remote = SBUSRemoteParser()
        if not self.sbus_remote.start():
            rospy.logwarn("[ROSNode] 遥控器串口初始化失败，仅支持原有控制逻辑")
        
        self.rtk_nav = RTKNavigator()

        # ROS订阅器
        rospy.Subscriber("/keyboard/control", String, self.keyboard_callback)  # 键盘控制
        rospy.Subscriber("/fix", NavSatFix, self.rtk_nav.gps_callback)  # GPS位置
        rospy.Subscriber("/wtrtk_data", WTRTK, self.rtk_nav.heading_callback)  # 惯导数据
        rospy.Subscriber("/rtk_nav/start", String, self.rtk_nav_start_callback)  # RTK导航启动
        rospy.Subscriber("/rtk_nav/resume", String, self.rtk_nav_resume_callback)  # 新增：恢复导航

        # ROS发布器
        self.state_pub = rospy.Publisher("/motor/state", String, queue_size=10)  # 电机状态
        self.speed_pub = rospy.Publisher("/motor/current_speed", Vector3, queue_size=10)  # 电机速度
        self.mode_pub = rospy.Publisher("/control/mode", String, queue_size=10)  # 控制模式发布
        self.nav_state_pub = rospy.Publisher("/rtk_nav/state", String, queue_size=10)  # 新增：导航状态发布

        # 初始化电机（进入START状态）
        self.switch_state('a')

        # 导航控制标志（新增）
        self.nav_running = False  # 标记导航是否正在运行

    def keyboard_callback(self, msg: String) -> None:
        """键盘控制回调（仅在非遥控器模式下有效）"""
        if self.sbus_remote.get_control_mode() == ControlMode.REMOTE:
            rospy.logwarn("[ROSNode] 当前为遥控器控制模式，忽略键盘指令")
            return
        
        key = msg.data.strip().lower()
        if key in STATE_DICT:
            self.switch_state(key)
        else:
            rospy.logwarn(f"[ROSNode] 无效键盘指令：{key}，支持指令：{list(STATE_DICT.keys())}")

    def rtk_nav_start_callback(self, msg: String) -> None:
        """RTK导航启动回调（仅在非遥控器模式下有效，重新开始导航）"""
        if self.sbus_remote.get_control_mode() == ControlMode.REMOTE:
            rospy.logwarn("[ROSNode] 当前为遥控器控制模式，忽略RTK导航启动指令")
            return
        
        if msg.data.strip().lower() == "start" and CURRENT_STATE != "STOP":
            rospy.loginfo("[ROSNode] 重新开始RTK导航（含初始点到第一个航点）")
            self.rtk_nav.reset_nav_context()  # 重置导航上下文
            self.nav_running = True
            self.run_rtk_navigation()

    def rtk_nav_resume_callback(self, msg: String) -> None:
        """新增：恢复中断的RTK导航（仅在非遥控器模式下有效）"""
        if self.sbus_remote.get_control_mode() == ControlMode.REMOTE:
            rospy.logwarn("[ROSNode] 当前为遥控器控制模式，忽略RTK导航恢复指令")
            return
        
        if msg.data.strip().lower() == "resume" and CURRENT_STATE != "STOP" and not self.nav_running:
            # 检查是否有可恢复的导航上下文
            if self.rtk_nav.nav_context["nav_state"] in [NavState.INITIAL_MOVE, NavState.WAYPOINT_MOVE, NavState.WAYPOINT_CALIB]:
                rospy.loginfo("[ROSNode] 恢复中断的RTK导航")
                self.nav_running = True
                self.run_rtk_navigation(resume=True)
            else:
                rospy.logwarn("[ROSNode] 无中断的导航任务可恢复，建议发送start指令重新开始")

    def switch_state(self, key: str) -> None:
        """状态机切换逻辑"""
        global CURRENT_STATE
        new_state = STATE_DICT[key]
        if new_state == CURRENT_STATE:
            rospy.loginfo(f"[ROSNode] 已处于{new_state}状态，无需切换")
            return

        rospy.loginfo(f"[ROSNode] 状态切换：{CURRENT_STATE} → {new_state}")
        CURRENT_STATE = new_state

        # 状态执行逻辑
        if new_state == "STOP":
            # 停止：失能所有电机，停止导航
            self.nav_running = False
            self.rtk_nav.nav_context["nav_state"] = NavState.IDLE  # 重置导航状态
            for motor in self.motor_ctrl.motors:
                self.motor_ctrl.motor_set_speed(motor["id"], 0.0)  # 初始速度0
                self.motor_ctrl.motor_disable(motor["id"])
                time.sleep(0.001)

        elif new_state == "START":
            # 启动：仅使能电机，不运动
            for motor in self.motor_ctrl.motors:
                self.motor_ctrl.motor_set_mode(motor["id"], motor["run_mode"])
                time.sleep(0.001)
                self.motor_ctrl.motor_set_current_limit(motor["id"], motor["current_limit"])
                time.sleep(0.001)
                self.motor_ctrl.motor_enable(motor["id"])
                time.sleep(0.001)
                self.motor_ctrl.motor_set_speed(motor["id"], 0.0)  # 初始速度0
                time.sleep(0.001)

        elif new_state == "FORWARD":
            # 前进：双电机正转（需先确认电机转向，调整符号）
            left_speed = self.rtk_nav.rad_from_linear(-LINEAR_SPEED_BASE)
            right_speed = self.rtk_nav.rad_from_linear(LINEAR_SPEED_BASE)
            self.set_motors_speed(left_speed, right_speed)

        elif new_state == "BACKWARD":
            # 后退：双电机反转
            left_speed = -self.rtk_nav.rad_from_linear(LINEAR_SPEED_BASE)
            right_speed = -self.rtk_nav.rad_from_linear(-LINEAR_SPEED_BASE)
            self.set_motors_speed(left_speed, right_speed)

        # 发布当前状态
        self.state_pub.publish(CURRENT_STATE)

    def set_motors_speed(self, left_speed: float, right_speed: float) -> None:
        """设置双电机速度（差速控制）"""
        # 左电机（ID=1）
        self.motor_ctrl.motor_set_speed(GLOBAL_MOTOR_CONFIG[0]["id"], left_speed)
        # 右电机（ID=2）
        self.motor_ctrl.motor_set_speed(GLOBAL_MOTOR_CONFIG[1]["id"], right_speed)
        wheel_speed_msg = Vector3()
        wheel_speed_msg.x = left_speed    # 左轮角速度（rad/s）
        wheel_speed_msg.y = right_speed   # 右轮角速度（rad/s）
        wheel_speed_msg.z = 0.0           # 预留字段，无意义设为0
        self.speed_pub.publish(wheel_speed_msg)  # 发布左右轮速度

    def run_rtk_navigation(self, resume: bool = False) -> None:
        """执行RTK导航（支持中断恢复）"""
        # 导航过程中若切换到遥控器模式，立即暂停导航（保存上下文）
        def check_control_mode() -> bool:
            if self.sbus_remote.get_control_mode() == ControlMode.REMOTE:
                rospy.loginfo("[ROSNode] 切换到遥控器控制模式，暂停RTK导航（保存上下文）")
                self.set_motors_speed(0.0, 0.0)
                self.nav_running = False
                return False
            return True

        # 恢复导航：从之前的状态继续
        if resume:
            current_nav_state = self.rtk_nav.nav_context["nav_state"]
            rospy.loginfo(f"[ROSNode] 从状态{current_nav_state}恢复导航")
        else:
            # 新启动导航：从初始状态开始
            current_nav_state = NavState.INITIAL_MOVE
            self.rtk_nav.nav_context["nav_state"] = current_nav_state

        # 1. 初始移动阶段（初始点→第一个航点）
        if current_nav_state == NavState.INITIAL_MOVE and not resume:
            try:
                initial_move_generator = self.rtk_nav.move_to_first_waypoint()
                while not rospy.is_shutdown() and check_control_mode():
                    try:
                        left_speed, right_speed = next(initial_move_generator)
                        self.set_motors_speed(left_speed, right_speed)
                    except StopIteration:
                        # 初始移动完成，切换到第一个航点
                        self.rtk_nav.current_waypoint_idx = 1
                        current_nav_state = NavState.WAYPOINT_MOVE
                        self.rtk_nav.nav_context["nav_state"] = current_nav_state
                        break
                    self.rate.sleep()
                if not check_control_mode():
                    return
            except Exception as e:
                rospy.logerr(f"[ROSNav] 初始点到第一个航点移动失败：{str(e)}")
                self.nav_running = False
                self.switch_state('s')
                return

        # 2. 航点导航阶段（从当前航点继续）
        last_distance = 0.0
        last_target_heading = 0.0
        while not rospy.is_shutdown() and self.rtk_nav.current_waypoint_idx < len(self.rtk_nav.waypoints) and check_control_mode():
            # 恢复导航时，从保存的目标航点获取
            if self.rtk_nav.nav_context["target_waypoint"]:
                target_waypoint = self.rtk_nav.nav_context["target_waypoint"]
            else:
                target_waypoint = self.rtk_nav.get_target_waypoint()
            
            if not target_waypoint:
                break

            # 发布当前导航状态
            self.nav_state_pub.publish(current_nav_state)

            # 2.1 航向校准阶段（恢复之前的校准过程）
            if current_nav_state == NavState.WAYPOINT_CALIB:
                calib_generator = self.rtk_nav.nav_context["calib_generator"]
                if not calib_generator:
                    rospy.logwarn("[ROSNav] 校准生成器不存在，重新开始校准")
                    target_heading = self.rtk_nav.get_path_heading(target_waypoint)
                    calib_generator = self.rtk_nav.calibrate_heading_at_waypoint(target_heading)
                    self.rtk_nav.nav_context["calib_generator"] = calib_generator
                
                while not rospy.is_shutdown() and check_control_mode():
                    try:
                        left_speed, right_speed = next(calib_generator)
                        self.set_motors_speed(left_speed, right_speed)
                    except StopIteration as e:
                        # 校准完成，切换到下一个航点
                        calib_result = e.value if hasattr(e, 'value') else False
                        rospy.loginfo(f"[ROSNav] 航点{self.rtk_nav.current_waypoint_idx+1}航向校准完成，结果：{calib_result}")
                        self.rtk_nav.current_waypoint_idx += 1
                        self.rtk_nav.nav_context["calib_generator"] = None
                        self.rtk_nav.nav_context["target_waypoint"] = None
                        current_nav_state = NavState.WAYPOINT_MOVE
                        self.rtk_nav.nav_context["nav_state"] = current_nav_state
                        time.sleep(1.0)
                        break
                    self.rate.sleep()
                if not check_control_mode():
                    return

            # 2.2 移动到航点阶段
            if current_nav_state == NavState.WAYPOINT_MOVE:
                # 保存当前目标航点到上下文
                self.rtk_nav.nav_context["target_waypoint"] = target_waypoint
                # 计算到目标航点的距离和目标航向角
                distance = self.rtk_nav.calc_distance_to_waypoint(target_waypoint)
                target_heading = self.rtk_nav.get_path_heading(target_waypoint)
                
                if abs(last_distance - distance) > 0.1 or abs(last_target_heading - target_heading) > 0.1:
                    last_distance = distance
                    last_target_heading = target_heading
                    rospy.loginfo(f"[ROSNav] 目标航点{self.rtk_nav.current_waypoint_idx+1}：距离{distance:.2f}m，目标航向{math.degrees(target_heading):.2f}°")

                # 距离未达标：直线行驶+实时纠偏
                if distance >= RTK_WAYPOINT_TOLERANCE:
                    correction = self.rtk_nav.get_speed_correction(target_heading)
                    base_speed_rad = self.rtk_nav.rad_from_linear(LINEAR_SPEED_BASE)
                    left_speed = base_speed_rad - correction
                    right_speed = base_speed_rad + correction
                    self.set_motors_speed(left_speed, right_speed)

                # 距离达标：进入航向校准阶段
                else:
                    rospy.loginfo(f"[ROSNav] 已到达航点{self.rtk_nav.current_waypoint_idx+1}距离阈值（{distance:.2f}m ≤ {RTK_WAYPOINT_TOLERANCE}m）")
                    target_heading = self.rtk_nav.get_path_heading(target_waypoint)
                    calib_generator = self.rtk_nav.calibrate_heading_at_waypoint(target_heading)
                    self.rtk_nav.nav_context["calib_generator"] = calib_generator
                    current_nav_state = NavState.WAYPOINT_CALIB
                    self.rtk_nav.nav_context["nav_state"] = current_nav_state

            self.rate.sleep()

        # 导航完成：重置上下文
        if check_control_mode():
            rospy.loginfo("[ROSNav] RTK导航全部完成")
            self.rtk_nav.reset_nav_context()
            self.nav_running = False
            self.switch_state('s')

    def run(self) -> None:
        """节点主循环（运行中检测串口状态和控制模式）"""
        while not rospy.is_shutdown():
            # 检查CAN串口是否正常打开
            if not (self.motor_ctrl.ser and self.motor_ctrl.ser.is_open):
                rospy.logwarn("[ROSNode] CAN串口连接断开，尝试重连...")
                if not self.motor_ctrl.reconnect_serial():
                    rospy.logfatal("[ROSNode] CAN串口重连失败，退出节点")
                    rospy.signal_shutdown("CAN串口连接永久断开")
                    break
            
            # 处理控制模式
            current_mode = self.sbus_remote.get_control_mode()
            self.mode_pub.publish(current_mode)  # 发布当前模式
            self.nav_state_pub.publish(self.rtk_nav.nav_context["nav_state"])  # 发布导航状态
            
            if current_mode == ControlMode.REMOTE:
                # 遥控器模式：优先执行遥控器指令
                if CURRENT_STATE != "START":
                    self.switch_state('a')  # 确保电机已使能
                left_speed, right_speed = self.sbus_remote.get_remote_speed()
                self.set_motors_speed(left_speed, right_speed)
                rospy.logdebug(f"[RemoteControl] 左轮：{left_speed:.2f}，右轮：{right_speed:.2f}")
            else:
                # 正常模式：保持原有逻辑，若导航已暂停且有上下文，提示可恢复
                self.state_pub.publish(CURRENT_STATE)
                if not self.nav_running and self.rtk_nav.nav_context["nav_state"] != NavState.IDLE:
                    rospy.loginfo("[ROSNode] 存在暂停的导航任务，发送/resume指令可恢复，发送/start指令可重新开始")
                    # 自动恢复导航
                    self.run_rtk_navigation(resume=True)
            
            self.rate.sleep()


# -------------------------- 6. 节点启动入口 --------------------------
if __name__ == "__main__":
    node = None
    try:
        node = MotorControlNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("[ROSNode] 节点被中断")
    except Exception as e:
        rospy.logerr(f"[ROSNode] 节点运行异常：{str(e)}")
    finally:
        # 三重保障：停止电机 → 关闭串口 → 释放资源
        if node:
            # 1. 先停止所有电机（避免电机持续运行）
            rospy.loginfo("[ROSNode] 退出时停止所有电机...")
            for motor in node.motor_ctrl.motors:
                try:
                    node.motor_ctrl.motor_set_speed(motor["id"], 0.0)
                    node.motor_ctrl.motor_disable(motor["id"])
                except Exception as e:
                    rospy.logwarn(f"[ROSNode] 停止电机{motor['id']}失败：{str(e)}")
                time.sleep(0.001)
            
            # 2. 关闭CAN串口
            if node.motor_ctrl.ser and node.motor_ctrl.ser.is_open:
                rospy.loginfo("[ROSNode] 关闭CAN串口连接...")
                try:
                    node.motor_ctrl.ser.close()
                except Exception as e:
                    rospy.logerr(f"[ROSNode] 关闭CAN串口失败：{str(e)}")
            
            # 3. 关闭SBUS遥控器串口
            node.sbus_remote.stop()
        
        rospy.loginfo("[ROSNode] 节点退出完成")