#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import serial
import struct
import time
import os
import math
from typing import Optional, List, Dict, Tuple
import rospy
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point,Vector3Stamped


# -------------------------- 1. 全局配置（与手册/硬件/ROS匹配） --------------------------
# 串口配置（CAN转USB工具）
SERIAL_PORT = "/dev/CAN-pyserial"    # 实际串口设备（需根据硬件调整）
SERIAL_BAUDRATE = 921600             # 与CAN转USB工具波特率一致
SERIAL_TIMEOUT = 0.1                 # 串口超时时间

# RTK导航配置
RTK_PATH_FILE = "/home/ubuntu/rtk_nav/src/gps/cleaning_path/cleaning_path_20251117_123454.txt"  # RTK路径文件
RTK_WAYPOINT_TOLERANCE = 0.1         # 到达目标点的距离 tolerance（米）
IMU_YAW_OFFSET = 0.0                 # IMU偏航角校准值（根据实际安装调整）
LINEAR_SPEED_BASE = 0.0124              # 基础线速度（m/s，需根据电机减速比/轮径转换）0.0124m/s>2rad/s
ANGULAR_SPEED_BASE = 0.1             # 基础角速度（rad/s，差速转向用）

# 电机配置（支持多电机差速）
GLOBAL_MOTOR_CONFIG: List[Dict] = [
    {
        "id": 127,                      # 左电机ID（1-127，手册限制）
        "velocity": 0.0,                # 初始速度（rad/s）
        "current_limit": 20.0,          # 电流限制（A，手册0x7018参数）
        "run_mode": 2                   # 2=速度模式（手册0x7005参数）
    },
    {
        "id": 2,                      # 右电机ID
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
MOTOR_MASTER_ID = 253      # 主机ID（0-300）
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


# -------------------------- 2. 核心工具类（串口通信+电机控制） --------------------------
class SerialMotorController:
    def __init__(self, port: str, baudrate: int):
        self.ser: Optional[serial.Serial] = None
        self.serial_port = port
        self.serial_baudrate = baudrate
        self.motors = [m for m in GLOBAL_MOTOR_CONFIG if "id" in m]
        rospy.loginfo(f"[MotorCtrl] 加载{len(self.motors)}个电机配置（ID：{[m['id'] for m in self.motors]}）")
        # 新增：重连参数
        self.reconnect_interval = 2  # 重连间隔（秒）
        self.max_reconnect_attempts = 0  # 0表示无限重试

    def init_serial(self) -> bool:
        """初始化串口（连接CAN转USB工具）"""
        try:
            # 若已打开，先关闭再重连
            if self.ser and self.ser.is_open:
                self.ser.close()
            
            self.ser = serial.Serial(
                port=self.serial_port,
                baudrate=self.serial_baudrate,
                timeout=SERIAL_TIMEOUT,
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


# -------------------------- 3. RTK路径解析与导航类 --------------------------
class RTKNavigator:
    def __init__(self):
        self.waypoints: List[Tuple[float, float]] = []  # RTK航点列表（经度，纬度）
        self.current_waypoint_idx = 0                   # 当前目标航点索引
        self.current_gps: Tuple[float, float] = (120.06717, 30.32088)  # 当前GPS位置
        self.imu_yaw = 0.0                              # 当前IMU偏航角（rad）
        self.load_rtk_path()

    def load_rtk_path(self) -> bool:
        """加载RTK路径文件（序号,经度,纬度）"""
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
                    seq, lon, lat = line.split(',')
                    self.waypoints.append((float(lon), float(lat)))
            rospy.loginfo(f"[RTKNav] 加载RTK航点{len(self.waypoints)}个：{self.waypoints}")
            return True
        except Exception as e:
            rospy.logerr(f"[RTKNav] 解析RTK文件失败：{str(e)}")
            return False

    def gps_callback(self, msg: Point) -> None:
        """GPS位置回调（需订阅GPS节点的位置消息）"""
        self.current_gps = (msg.x, msg.y)  # x=经度，y=纬度

    def heading_callback(self, msg: Vector3Stamped) -> None:
        """IMU数据回调（提取偏航角，用于直线纠偏）"""
        # 填充时间戳（使用当前时间或传感器原始时间戳）
        self.imu_yaw = msg.vector.z    # 通常用z轴表示yaw（航向角）

    def get_target_waypoint(self) -> Optional[Tuple[float, float]]:
        """获取当前目标航点"""
        if self.current_waypoint_idx >= len(self.waypoints):
            rospy.loginfo("[RTKNav] 已到达最后一个航点")
            return None
        return self.waypoints[self.current_waypoint_idx]

    def calc_distance_to_waypoint(self, waypoint: Tuple[float, float]) -> float:
        """计算当前位置到目标航点的距离（简化：度→米，1度≈111319.9米）"""
        lon_diff = (waypoint[0] - self.current_gps[0]) * 111319.9
        lat_diff = (waypoint[1] - self.current_gps[1]) * 111319.9
        return math.hypot(lon_diff, lat_diff)

    def calc_target_yaw(self, waypoint: Tuple[float, float]) -> float:
        """计算当前位置到目标航点的目标偏航角"""
        lon_diff = waypoint[0] - self.current_gps[0]
        lat_diff = waypoint[1] - self.current_gps[1]
        target_yaw = math.atan2(lon_diff, lat_diff)  # 经度差→x，纬度差→y
        target_yaw = math.fmod(target_yaw + math.pi, 2 * math.pi) - math.pi
        return target_yaw

    def get_speed_correction(self, target_yaw: float) -> float:
        """直线纠偏：根据IMU偏航角与目标偏航角的差值，计算速度修正量"""
        yaw_error = target_yaw - self.imu_yaw
        yaw_error = math.fmod(yaw_error + math.pi, 2 * math.pi) - math.pi  # 归一化误差
        # 比例控制（Kp=0.5，可调整）
        correction = 0.5 * yaw_error
        rospy.logdebug(f"[RTKNav] 偏航误差：{yaw_error:.3f}rad，修正量：{correction:.3f}rad/s")
        return correction


# -------------------------- 4. ROS节点核心逻辑 --------------------------
class MotorControlNode:
    def __init__(self):
        rospy.init_node('motor_control_node', anonymous=True)
        self.rate = rospy.Rate(10)  # 10Hz控制频率

        # 初始化核心模块
        self.motor_ctrl = SerialMotorController(SERIAL_PORT, SERIAL_BAUDRATE)
        rospy.loginfo("[ROSNode] 开始初始化串口...")
        if not self.motor_ctrl.init_serial():
            rospy.logwarn("[ROSNode] 首次初始化失败，进入重连模式")
            if not self.motor_ctrl.reconnect_serial():
                rospy.logfatal("[ROSNode] 串口重连失败，无法继续运行，退出节点")
                rospy.signal_shutdown("串口重连失败")
                return  # 退出初始化
        self.rtk_nav = RTKNavigator()

        # ROS订阅器
        rospy.Subscriber("/keyboard/control", String, self.keyboard_callback)  # 键盘控制
        rospy.Subscriber("/fix", Point, self.rtk_nav.gps_callback)  # GPS位置
        rospy.Subscriber("/imu_data", Vector3Stamped, self.rtk_nav.heading_callback)  # RTK的惯导数据
        rospy.Subscriber("/rtk_nav/start", String, self.rtk_nav_start_callback)  # RTK导航启动

        # ROS发布器
        self.state_pub = rospy.Publisher("/motor/state", String, queue_size=10)  # 电机状态
        self.speed_pub = rospy.Publisher("/motor/speed", Twist, queue_size=10)  # 电机速度

        # 初始化电机（进入START状态）
        self.switch_state('a')

    def keyboard_callback(self, msg: String) -> None:
        """键盘控制回调（接收's'/'f'/'b'/'a'指令）"""
        key = msg.data.strip().lower()
        if key in STATE_DICT:
            self.switch_state(key)
        else:
            rospy.logwarn(f"[ROSNode] 无效键盘指令：{key}，支持指令：{list(STATE_DICT.keys())}")

    def rtk_nav_start_callback(self, msg: String) -> None:
        """RTK导航启动回调（接收"start"指令开始导航）"""
        if msg.data.strip().lower() == "start" and CURRENT_STATE != "STOP":
            rospy.loginfo("[ROSNode] 开始RTK导航")
            self.run_rtk_navigation()

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
            # 停止：失能所有电机
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
            left_speed = self.rad_from_linear(LINEAR_SPEED_BASE)
            right_speed = self.rad_from_linear(LINEAR_SPEED_BASE)
            self.set_motors_speed(left_speed, right_speed)

        elif new_state == "BACKWARD":
            # 后退：双电机反转
            left_speed = -self.rad_from_linear(LINEAR_SPEED_BASE)
            right_speed = -self.rad_from_linear(LINEAR_SPEED_BASE)
            self.set_motors_speed(left_speed, right_speed)

        # 发布当前状态
        self.state_pub.publish(CURRENT_STATE)

    def rad_from_linear(self, linear_speed: float) -> float:
        """线速度（m/s）转电机角速度（rad/s）：需根据电机减速比/轮径校准"""
        # 公式：rad/s = (linear_speed * 减速比) / 轮径（单位：m）1-2rad/s
        # 示例：减速比7.75，轮径0.04874m → rad/s = (v *7.75)/0.04874 = 77.5*v
        reduction_ratio = 7.75  # 电机减速比（手册0x201f参数）
        wheel_diameter = 0.04874     # 车轮直径（m，需根据实际硬件调整）48.74mm
        return (linear_speed * reduction_ratio) / wheel_diameter

    def set_motors_speed(self, left_speed: float, right_speed: float) -> None:
        """设置双电机速度（差速控制）"""
        # 左电机（ID127）
        self.motor_ctrl.motor_set_speed(127, left_speed)
        # 右电机（ID=）
        self.motor_ctrl.motor_set_speed(2, right_speed)
        # 发布速度消息
        speed_msg = Twist()
        speed_msg.linear.x = (left_speed + right_speed) / 2  # 平均线速度
        speed_msg.angular.z = (right_speed - left_speed) / 0.5  # 角速度（轮距0.5m，需调整）
        self.speed_pub.publish(speed_msg)

    def run_rtk_navigation(self) -> None:
        """执行RTK导航（按航点依次移动，直线纠偏）"""
        while not rospy.is_shutdown() and self.rtk_nav.current_waypoint_idx < len(self.rtk_nav.waypoints):
            target_waypoint = self.rtk_nav.get_target_waypoint()
            if not target_waypoint:
                break

            # 1. 计算到目标航点的距离和目标偏航角
            distance = self.rtk_nav.calc_distance_to_waypoint(target_waypoint)
            target_yaw = self.rtk_nav.calc_target_yaw(target_waypoint)
            rospy.loginfo(f"[ROSNav] 目标航点{self.rtk_nav.current_waypoint_idx+1}：距离{distance:.2f}m")

            # 2. 到达目标航点：切换到下一个
            if distance < RTK_WAYPOINT_TOLERANCE:
                rospy.loginfo(f"[ROSNav] 到达航点{self.rtk_nav.current_waypoint_idx+1}")
                self.rtk_nav.current_waypoint_idx += 1
                self.switch_state('a')  # 短暂停止，准备下一段
                time.sleep(1.0)
                continue

            # 3. 直线纠偏：计算速度修正量
            correction = self.rtk_nav.get_speed_correction(target_yaw)
            # 4. 差速控制：左电机=基础速度-修正量，右电机=基础速度+修正量
            base_speed = self.rad_from_linear(LINEAR_SPEED_BASE)
            left_speed = base_speed - correction
            right_speed = base_speed + correction
            self.set_motors_speed(left_speed, right_speed)

            self.rate.sleep()

        # 导航结束：停止电机
        self.switch_state('s')
        rospy.loginfo("[ROSNav] RTK导航完成")

    def run(self) -> None:
        """节点主循环（运行中检测串口状态，断开后自动重连）"""
        while not rospy.is_shutdown():
            # 检查串口是否正常打开
            if not (self.motor_ctrl.ser and self.motor_ctrl.ser.is_open):
                rospy.logwarn("[ROSNode] 串口连接断开，尝试重连...")
                if not self.motor_ctrl.reconnect_serial():
                    rospy.logfatal("[ROSNode] 重连失败，退出节点")
                    rospy.signal_shutdown("串口连接永久断开")
                    break
            
            # 发布当前状态（保持心跳）
            self.state_pub.publish(CURRENT_STATE)
            self.rate.sleep()


# -------------------------- 5. 节点启动入口 --------------------------
if __name__ == "__main__":
    try:
        node = MotorControlNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("[ROSNode] 节点被中断")
    finally:
        # 退出时停止电机并关闭串口
        if 'node' in locals() and node.motor_ctrl.ser:
            for motor in node.motor_ctrl.motors:
                node.motor_ctrl.motor_disable(motor["id"])
            node.motor_ctrl.ser.close()
            rospy.loginfo("[ROSNode] 电机已停止，串口已关闭")