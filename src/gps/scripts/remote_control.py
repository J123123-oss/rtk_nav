import serial
import struct
import threading
import time
from typing import Optional, Tuple, List, Dict
import rospy
from geometry_msgs.msg import Vector3

# 全局配置 - 精确阈值定义（与C宏定义一致）
SERIAL_PORT = "/dev/ttyUSB0"  # 根据实际设备修改
BAUD_RATE = 115200
FRAME_HEADER = 0x0F
FRAME_TAIL1 = 0x00
FRAME_TAIL2 = 0x48
FRAME_LENGTH = 35  # 帧头(1) + 数据(32) + 尾标(2) = 35字节
CHANNEL_COUNT = 16  # 16通道

# 遥控器通道阈值（参考C宏定义com_rc_slave.h）
RC_CH_MIN_VALUE = 282    # 通道最小值
RC_CH_MID_VALUE = 1002   # 通道中间值
RC_CH_MAX_VALUE = 1722   # 通道最大值
RC_CH_HALF_RANGE = 720   # 半量程（1002-282=720，1722-1002=720）

MAX_SPEED = 5.0  # 电机最大角速度(rad/s)，根据实际设备调整
DEAD_ZONE = 50   # 摇杆死区范围（在中间值附近，避免微小偏移误触发）

# 电机配置，需要与MotorControllerNode配置相同
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

# 全局控制标志
remote_flag = True  # True:遥控器控制，False:其他命令控制
lock = threading.Lock()  # 线程安全锁


class MotorController:
    """电机控制类（模拟原有motor_ctrl接口）"""
    def __init__(self):
        pass

    def motor_set_speed(self, motor_id: int, speed: float):
        """设置单个电机速度"""
        rospy.logdebug(f"电机{motor_id}速度设置为: {speed:.2f} rad/s")


class SBUSRemoteControl:
    def __init__(self):
        # 初始化串口
        self.ser = self._init_serial()
        # 初始化电机控制器
        self.motor_ctrl = MotorController()
        # 初始化ROS节点和发布者
        rospy.init_node("sbus_remote_control", anonymous=True)
        self.speed_pub = rospy.Publisher("/wheel_speed", Vector3, queue_size=10)
        # 遥控器通道数据缓存（初始化为中间值）
        self.channels = [RC_CH_MID_VALUE] * CHANNEL_COUNT
        # 线程控制标志
        self.running = True
        # 启动串口读取线程
        self.read_thread = threading.Thread(target=self._read_serial_loop)
        self.read_thread.daemon = True
        self.read_thread.start()

    def _init_serial(self) -> serial.Serial:
        """初始化串口连接（115200,8N1）"""
        try:
            ser = serial.Serial(
                port=SERIAL_PORT,
                baudrate=BAUD_RATE,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=0.1
            )
            rospy.loginfo(f"串口初始化成功: {SERIAL_PORT} (115200,8N1)")
            return ser
        except Exception as e:
            rospy.logerr(f"串口初始化失败: {e}")
            raise

    def _parse_frame(self, frame: bytes) -> Optional[list]:
        """解析串口帧数据，严格校验格式"""
        # 校验帧长度、帧头和尾标
        if len(frame) != FRAME_LENGTH:
            rospy.logdebug(f"帧长度错误: 实际{len(frame)}字节，期望{FRAME_LENGTH}字节")
            return None
        if frame[0] != FRAME_HEADER:
            rospy.logdebug(f"帧头错误: 实际0x{frame[0]:02X}，期望0x{FRAME_HEADER:02X}")
            return None
        if frame[-2] != FRAME_TAIL1 or frame[-1] != FRAME_TAIL2:
            rospy.logdebug(f"帧尾错误: 实际0x{frame[-2]:02X}{frame[-1]:02X}，期望0x{FRAME_TAIL1:02X}{FRAME_TAIL2:02X}")
            return None

        # 解析32字节数据为16个16bit通道值（小端模式）
        channels = []
        for i in range(CHANNEL_COUNT):
            byte_start = 1 + 2 * i
            byte_end = byte_start + 2
            # 解析16bit小端数据
            value = struct.unpack("<H", frame[byte_start:byte_end])[0]
            # 校验值是否在有效范围（0-2047）
            if not (0 <= value <= 2047):
                rospy.logdebug(f"通道{i+1}值超出范围: {value}")
                return None
            # 限制通道值在标准阈值范围内（避免异常值）
            value = max(RC_CH_MIN_VALUE, min(RC_CH_MAX_VALUE, value))
            channels.append(value)
        return channels

    def _read_serial_loop(self):
        """串口数据读取循环（后台线程）"""
        buffer = b""
        while self.running and not rospy.is_shutdown():
            try:
                # 读取串口数据
                data = self.ser.read(1024)
                if not data:
                    time.sleep(0.001)
                    continue
                buffer += data

                # 查找完整帧（帧头+35字节）
                while len(buffer) >= FRAME_LENGTH:
                    header_idx = buffer.find(bytes([FRAME_HEADER]))
                    if header_idx == -1:
                        # 无有效帧头，清空缓冲区
                        buffer = b""
                        break
                    # 检查帧是否完整
                    if header_idx + FRAME_LENGTH > len(buffer):
                        # 帧不完整，保留从帧头开始的剩余数据
                        buffer = buffer[header_idx:]
                        break
                    # 提取完整帧并解析
                    frame = buffer[header_idx:header_idx+FRAME_LENGTH]
                    buffer = buffer[header_idx+FRAME_LENGTH:]  # 移除已解析帧
                    channels = self._parse_frame(frame)
                    if channels:
                        with lock:
                            self.channels = channels
                        # 打印常用通道数据（CH1-CH12）
                        rospy.logdebug(f"通道数据: CH1-{self.channels[0]}, CH2-{self.channels[1]}, CH3-{self.channels[2]}, CH4-{self.channels[3]}, "
                                       f"CH5-{self.channels[4]}, CH6-{self.channels[5]}, CH7-{self.channels[6]}, CH8-{self.channels[7]}")

            except Exception as e:
                rospy.logerr(f"串口读取异常: {e}")
                time.sleep(0.1)

    def _map_channel_to_speed(self, channel_value: int, mid: int) -> float:
        """
        将通道值映射为速度（基于精确阈值）
        映射逻辑：
        - 低于中间值-死区：反向速度（0 ~ -MAX_SPEED）
        - 中间值±死区：零速度（死区）
        - 高于中间值+死区：正向速度（0 ~ MAX_SPEED）
        """
        # 死区处理
        if abs(channel_value - mid) < DEAD_ZONE:
            return 0.0
        
        # 基于半量程（RC_CH_HALF_RANGE）计算归一化比例
        if channel_value > mid:
            # 正向区间（mid+DEAD_ZONE ~ RC_CH_MAX_VALUE）
            ratio = (channel_value - mid - DEAD_ZONE) / (RC_CH_HALF_RANGE - DEAD_ZONE)
            speed = ratio * MAX_SPEED
        else:
            # 反向区间（RC_CH_MIN_VALUE ~ mid-DEAD_ZONE）
            ratio = (channel_value - mid + DEAD_ZONE) / (DEAD_ZONE - RC_CH_HALF_RANGE)
            speed = -ratio * MAX_SPEED
        
        # 速度限幅（确保不超出最大范围）
        return max(-MAX_SPEED, min(MAX_SPEED, speed))

    def get_remote_control(self) -> Tuple[float, float]:
        """
        根据遥控器通道计算电机速度（差速控制）
        通道映射：
        - CH3（右摇杆前后）：前进/后退控制
        - CH4（左摇杆左右）：转向控制
        """
        with lock:
            ch3 = self.channels[2]  # CH3: 右摇杆前后（UP>1002=前进）
            ch4 = self.channels[3]  # CH4: 左摇杆左右（UP<1002=左转）
            # 右摇杆左右：CH1,LEFT<1002
            # 左摇杆前后：CH2,up<1002
            # 右摇杆前后：CH3,UP>1002
            # 左摇杆左右：CH4,UP<1002

        # 映射前进/后退速度（CH3）
        forward_speed = self._map_channel_to_speed(ch3, RC_CH_MID_VALUE)
        # 映射转向速度（CH4）- 左转时左轮减速、右轮加速，右转相反
        turn_speed = self._map_channel_to_speed(ch4, RC_CH_MID_VALUE)

        # 差速控制公式
        left_speed = forward_speed - turn_speed   # 左转时turn_speed为正，左轮减速
        right_speed = forward_speed + turn_speed  # 左转时turn_speed为正，右轮加速

        # 最终速度限幅
        left_speed = max(-MAX_SPEED, min(MAX_SPEED, left_speed))
        right_speed = max(-MAX_SPEED, min(MAX_SPEED, right_speed))

        return left_speed, right_speed

    def set_motors_speed(self, left_speed: float, right_speed: float) -> None:
        """设置双电机速度（与原有接口完全兼容）"""
        # 左电机（ID=1）
        self.motor_ctrl.motor_set_speed(GLOBAL_MOTOR_CONFIG[0]["id"], left_speed)
        # 右电机（ID=2）
        self.motor_ctrl.motor_set_speed(GLOBAL_MOTOR_CONFIG[1]["id"], right_speed)
        # 发布速度消息
        wheel_speed_msg = Vector3()
        wheel_speed_msg.x = left_speed    # 左轮角速度（rad/s）
        wheel_speed_msg.y = right_speed   # 右轮角速度（rad/s）
        wheel_speed_msg.z = 0.0           # 预留字段
        self.speed_pub.publish(wheel_speed_msg)
        rospy.loginfo(f"电机速度: 左轮={left_speed:.2f} rad/s, 右轮={right_speed:.2f} rad/s")

    def control_loop(self):
        """主控制循环（50Hz）"""
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            global remote_flag
            with lock:
                current_mode = remote_flag

            if current_mode:
                # 遥控器控制模式
                left_speed, right_speed = self.get_remote_control()
                self.set_motors_speed(left_speed, right_speed)
            else:
                # 外部命令控制模式（需通过外部接口设置速度）
                pass

            rate.sleep()

    def stop(self):
        """停止控制，关闭资源"""
        self.running = False
        if self.ser.is_open:
            self.ser.close()
        # 紧急停止电机
        self.set_motors_speed(0.0, 0.0)
        rospy.loginfo("控制停止，电机已急停")


# 外部控制接口（保持原有逻辑）
def switch_control_mode(to_remote: bool):
    """切换控制模式：True-遥控器控制，False-外部命令控制"""
    global remote_flag
    with lock:
        remote_flag = to_remote
        rospy.loginfo(f"控制模式切换: {'遥控器控制' if to_remote else '外部命令控制'}")


def set_external_speed(controller: SBUSRemoteControl, left_speed: float, right_speed: float):
    """外部命令设置电机速度（需先切换到外部控制模式）"""
    global remote_flag
    with lock:
        if not remote_flag:
            # 外部速度同样需要限幅
            left_speed = max(-MAX_SPEED, min(MAX_SPEED, left_speed))
            right_speed = max(-MAX_SPEED, min(MAX_SPEED, right_speed))
            controller.set_motors_speed(left_speed, right_speed)
        else:
            rospy.logwarn("当前为遥控器控制模式，无法执行外部速度命令")


# 三挡开关状态解析（扩展功能）remote_state
def remote_state(channel_value: int) -> str:
    """解析CH5/CH6三挡开关状态"""
    if abs(channel_value - RC_CH_MIN_VALUE) < DEAD_ZONE:
        return "LOW"    # 低挡位
    elif abs(channel_value - RC_CH_MID_VALUE) < DEAD_ZONE:
        return "MID"    # 中挡位
    elif abs(channel_value - RC_CH_MAX_VALUE) < DEAD_ZONE:
        return "HIGH"   # 高挡位
    else:
        return "UNKNOWN"


if __name__ == "__main__":
    try:
        # 创建控制器实例
        remote_controller = SBUSRemoteControl()
        # 启动主控制循环
        remote_controller.control_loop()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS节点中断")
    finally:
        # 确保资源释放和电机停止
        remote_controller.stop()