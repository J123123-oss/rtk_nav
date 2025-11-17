import can
import struct
import time
from typing import Optional, List, Dict

# -------------------------- 1. 全局电机配置（多电机参数，支持启用/注释） --------------------------
GLOBAL_MOTOR_CONFIG: List[Dict] = [
    {
        "id": 127,                      # 靠近工控机电机ID（1-127）
        "velocity": 5.0,              # 目标速度（单位：rad/s，参考手册0x700A参数）
        # "acceleration": 10.0,        # 加速度（单位：rad/s²，位置模式PP专用）
        "current_limit": 20.0         # 电流限制（单位：A，参考手册0x7018参数）
    },
    # {
    #     "id": 2,                      # 远离工控机电机ID（取消注释启用）
    #     "velocity": 5.0,
    #     # "acceleration": 10.0,
    #     "current_limit": 20.0
    # },
    # {
    #     "id": 3,                      # 滚刷电机ID（注释则不启用）
    #     "velocity": 8.38,             # 示例：1600 RPM转换为rad/s（需根据减速比调整）
    #   # "acceleration": 1500.0,
    #     "current_limit": 25.0
    # }
]

# -------------------------- 2. 固定配置参数（与手册/原程序对应） --------------------------
CAN_CHANNEL = "can0"  # 实际通道：USB-CAN为"COM3"/"slcan0"，嵌入式设备为"can0"
CAN_BAUDRATE = 1000000  # 1Mbps，手册私有协议波特率（）
MOTOR_MASTER_ID = 253  # 主机ID，与原32程序一致

# 电机参数索引（参考手册可读写参数列表）（）
RUN_MODE_INDEX = 0x7005    # 运行模式索引（run_mode）
SPEED_REF_INDEX = 0x700A   # 速度指令索引（spd_ref）
LIMIT_CUR_INDEX = 0x7018   # 电流限制索引（limit_cur）
ACC_SET_INDEX = 0x7026     # 加速度索引（acc_set，位置模式PP专用）（）

# 通信类型（参考手册私有协议）（）
COMM_TYPE_WRITE = 0x12     # 单个参数写入（十进制18）
COMM_TYPE_ENABLE = 0x03    # 电机使能
COMM_TYPE_RESET = 0x04     # 电机停止/复位

# 运行模式定义（参考手册0x7005参数说明）（）
MODE_SPEED = 2        # 速度模式（当前配置用）
MODE_POSITION_PP = 1  # 位置模式PP（需配合加速度参数）


class MotorController:
    def __init__(self, can_channel: str = CAN_CHANNEL, baudrate: int = CAN_BAUDRATE):
        self.can_bus: Optional[can.interface.Bus] = None
        self.can_channel = can_channel
        self.baudrate = baudrate
        # 加载全局电机配置（过滤有效电机）
        self.motors = [motor for motor in GLOBAL_MOTOR_CONFIG if "id" in motor]
        print(f"[INFO] 加载电机配置：共{len(self.motors)}个电机（ID：{[m['id'] for m in self.motors]}）")

    def init_can(self) -> bool:
        """初始化CAN总线"""
        try:
            self.can_bus = can.interface.Bus(
                channel=self.can_channel,
                interface="socketcan",
                bitrate=self.baudrate
            )
            print(f"[INFO] CAN总线初始化成功（通道：{self.can_channel}，波特率：{self.baudrate}）")
            return True
        except Exception as e:
            print(f"[ERROR] CAN总线初始化失败：{str(e)}")
            return False

    def _create_can_msg(self, comm_type: int, master_id: int, motor_id: int, data: bytes) -> can.Message:
        """创建CAN消息（扩展帧，符合手册格式）（）"""
        can_id = (comm_type << 24) | (master_id << 8) | motor_id  # 帧ID：通信类型+主机ID+电机ID
        return can.Message(
            arbitration_id=can_id,
            is_extended_id=True,
            data=data.ljust(8, b'\x00'),  # 补零至8字节（原32程序逻辑）
            dlc=8
        )

    def _send_can_msg(self, motor_id: int, msg: can.Message) -> bool:
        """发送CAN消息并打印指令（含电机ID标识）"""
        if not self.can_bus:
            print(f"[ERROR] 电机{motor_id}：CAN总线未初始化，无法发送")
            return False
        
        try:
            self.can_bus.send(msg)
            # 打印格式：电机ID + 帧ID（十六进制） + 数据段（十六进制）
            id_hex = hex(msg.arbitration_id)
            data_hex = " ".join([f"{byte:02X}" for byte in msg.data])
            print(f"[SEND] 电机{motor_id:2d} | ID: {id_hex:>10} | Data: {data_hex}")
            return True
        except Exception as e:
            print(f"[ERROR] 电机{motor_id}：发送失败：{str(e)}")
            return False

    # -------------------------- 核心控制函数（单个电机操作） --------------------------
    def motor_set_mode(self, motor_id: int, mode: int) -> bool:
        """设置单个电机运行模式（0x7005参数）（）"""
        data = bytearray(8)
        struct.pack_into("<H", data, 0, RUN_MODE_INDEX)  # 0x7005索引（小端存储）
        struct.pack_into("<B", data, 4, mode)            # 模式值存于Byte4
        msg = self._create_can_msg(COMM_TYPE_WRITE, MOTOR_MASTER_ID, motor_id, data)
        return self._send_can_msg(motor_id, msg)

    def motor_set_current_limit(self, motor_id: int, current_limit: float) -> bool:
        """设置单个电机电流限制（0x7018参数）（）"""
        data = bytearray(8)
        struct.pack_into("<H", data, 0, LIMIT_CUR_INDEX)  # 0x7018索引
        struct.pack_into("<f", data, 4, current_limit)    # 电流值（float类型）
        msg = self._create_can_msg(COMM_TYPE_WRITE, MOTOR_MASTER_ID, motor_id, data)
        return self._send_can_msg(motor_id, msg)

    def motor_set_acceleration(self, motor_id: int, acceleration: float) -> bool:
        """设置单个电机加速度（0x7026参数，位置模式PP专用）（）"""
        data = bytearray(8)
        struct.pack_into("<H", data, 0, ACC_SET_INDEX)    # 0x7026索引
        struct.pack_into("<f", data, 4, acceleration)     # 加速度值（float类型）
        msg = self._create_can_msg(COMM_TYPE_WRITE, MOTOR_MASTER_ID, motor_id, data)
        return self._send_can_msg(motor_id, msg)

    def motor_enable(self, motor_id: int) -> bool:
        """使能单个电机（通信类型3）（）"""
        data = b'\x00' * 8
        msg = self._create_can_msg(COMM_TYPE_ENABLE, MOTOR_MASTER_ID, motor_id, data)
        return self._send_can_msg(motor_id, msg)

    def motor_set_speed(self, motor_id: int, speed: float) -> bool:
        """设置单个电机速度（0x700A参数）（）"""
        data = bytearray(8)
        struct.pack_into("<H", data, 0, SPEED_REF_INDEX)  # 0x700A索引
        struct.pack_into("<f", data, 4, speed)            # 速度值（float，rad/s）
        msg = self._create_can_msg(COMM_TYPE_WRITE, MOTOR_MASTER_ID, motor_id, data)
        return self._send_can_msg(motor_id, msg)

    def motor_reset(self, motor_id: int) -> bool:
        """停止单个电机（通信类型4）（）"""
        data = b'\x80'+b'\x00' * 7
        msg = self._create_can_msg(COMM_TYPE_RESET, MOTOR_MASTER_ID, motor_id, data)
        return self._send_can_msg(motor_id, msg)


# -------------------------- main函数（直接循环控制多电机） --------------------------
if __name__ == "__main__":
    # 1. 初始化电机控制器（加载全局配置）
    mc = MotorController(can_channel=CAN_CHANNEL, baudrate=CAN_BAUDRATE)
    if not mc.init_can():
        exit(1)

    try:
        # -------------------------- 2. 循环初始化所有配置的电机 --------------------------
        print("\n[INFO] 开始初始化所有电机...")
        time.sleep(3)  # 模拟原程序3秒延时等待（确保硬件就绪）
        
        for motor in mc.motors:
            motor_id = motor["id"]
            print(f"\n[INFO] 初始化电机{motor_id}...")
            
            # ① 设置速度模式（可改为MODE_POSITION_PP切换位置模式）
            mc.motor_set_mode(motor_id, mode=MODE_SPEED)
            time.sleep(0.001)  # 短延时避免CAN总线拥堵
            
            # ② 设置电流限制（从全局配置读取参数）
            mc.motor_set_current_limit(motor_id, motor["current_limit"])
            time.sleep(0.001)
            
            # ③ 若为位置模式，需设置加速度（当前为速度模式，注释禁用）
            # mc.motor_set_acceleration(motor_id, motor["acceleration"])
            # time.sleep(0.001)
            
            # ④ 使能电机（必须在模式/参数设置后执行）
            mc.motor_enable(motor_id)
            time.sleep(0.001)

        # -------------------------- 3. 循环发送速度指令（持续运行） --------------------------
        print("\n[INFO] 开始循环发送速度指令（按Ctrl+C停止）...")
        while True:
            for motor in mc.motors:
                motor_id = motor["id"]
                target_speed = motor["velocity"]
                # 发送当前电机的速度指令（从全局配置读取目标速度）
                mc.motor_set_speed(motor_id, target_speed)
                time.sleep(0.001)  # 避免多电机指令冲突
            time.sleep(0.1)  # 每100ms循环一次（可调整发送频率）

    except KeyboardInterrupt:
        # -------------------------- 4. 中断后循环停止所有电机 --------------------------
        print("\n[INFO] 检测到停止指令，正在停止所有电机...")
        for motor in mc.motors:
            motor_id = motor["id"]
            mc.motor_reset(motor_id)
            time.sleep(0.001)
        print("[INFO] 所有电机已停止")

    finally:
        # -------------------------- 5. 关闭CAN总线 --------------------------
        if mc.can_bus:
            mc.can_bus.shutdown()
            print("[INFO] CAN总线已关闭")