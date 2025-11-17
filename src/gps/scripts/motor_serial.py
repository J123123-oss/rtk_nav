import serial
import struct
import time
from typing import Optional, List, Dict

# -------------------------- 1. 全局配置（与手册/硬件匹配） --------------------------
# 串口配置（CAN转USB工具参数，参考手册）（）
SERIAL_PORT = "/dev/CAN-pyserial"    # 串口
SERIAL_BAUDRATE = 921600 # 串口波特率（需与CAN转USB工具一致，手册默认115200）
SERIAL_TIMEOUT = 0.1    # 串口超时时间（秒）

# 电机配置（与原逻辑一致）
GLOBAL_MOTOR_CONFIG: List[Dict] = [
    {
        "id": 127,                      # 电机ID
        "velocity": 2.0,              # 目标速度（rad/s）
        "current_limit": 20.0,        # 电流限制（A）
        "run_mode": 2                 # 运行模式（0运控模式 1位置模式（PP）2速度模式3电流模式 5位置模式（CSP））
    },
    {
        "id": 126,                      # 第二个电机
        "velocity": 0.0,
        "current_limit": 20.0,
        "run_mode": 2
    }
]

# 协议固定字段（手册定义）（）
FRAME_HEADER = b'\x41\x54'  # 帧头 "AT"
FRAME_TAIL = b'\x0D\x0A'    # 帧尾 "\r\n"
DATA_LEN = b'\x08'          # 数据帧长度（固定8字节）

# 电机参数索引（手册可读写参数列表）（）
RUN_MODE_INDEX = 0x7005    # 运行模式索引
SPEED_REF_INDEX = 0x700A   # 速度指令索引
LIMIT_CUR_INDEX = 0x7018   # 电流限制索引

# 主机ID（与原32程序一致）
MOTOR_MASTER_ID = 253    # 示例主机ID（需根据实际配置修改）


class SerialMotorController:
    def __init__(self, port: str = SERIAL_PORT, baudrate: int = SERIAL_BAUDRATE):
        self.ser: Optional[serial.Serial] = None
        self.serial_port = port
        self.serial_baudrate = baudrate
        self.motors = [m for m in GLOBAL_MOTOR_CONFIG if "id" in m]
        print(f"[INFO] 加载{len(self.motors)}个电机配置（ID：{[m['id'] for m in self.motors]}）")

    def init_serial(self) -> bool:
        """初始化串口（连接CAN转USB工具）"""
        try:
            self.ser = serial.Serial(
                port=self.serial_port,
                baudrate=self.serial_baudrate,
                timeout=SERIAL_TIMEOUT,
                parity=serial.PARITY_NONE,    # 无校验（手册默认）
                stopbits=serial.STOPBITS_ONE, # 1个停止位（手册默认）
                bytesize=serial.EIGHTBITS     # 8位数据位（手册默认）
            )
            if self.ser.is_open:
                print(f"[INFO] 串口初始化成功（端口：{self.serial_port}，波特率：{self.serial_baudrate}）")
                return True
            return False
        except Exception as e:
            print(f"[ERROR] 串口初始化失败：{str(e)}")
            return False

    def _convert_can_id_to_serial(self, can_id: int) -> bytes:
        """将29位CAN ID转换为串口协议中的4字节“扩展帧”字段（手册规则）（）"""
        # 步骤1：29位CAN ID补3个0（末尾），转为32位数据（因串口扩展帧是4字节=32位）
        can_id_32bit = (can_id << 3)  # 末尾补3个0（手册规定）
        # 步骤2：按大端字节序转为4字节（手册规定大端存储）
        return struct.pack(">I", can_id_32bit)  # ">"表示大端，"I"表示无符号32位整数

    def _build_serial_packet(self, can_id: int, can_data: bytes) -> bytes:
        """构造完整串口数据包（帧头+扩展帧+数据长度+数据帧+帧尾）"""
        # 1. 转换CAN ID为串口扩展帧字段（4字节）
        serial_ext_id = self._convert_can_id_to_serial(can_id)
        # 2. 拼接完整数据包
        packet = (
            FRAME_HEADER       # 2字节：帧头
            + serial_ext_id    # 4字节：扩展帧（转换后的CAN ID）
            + DATA_LEN         # 1字节：数据帧长度（固定8）
            + can_data         # 8字节：CAN数据段
            + FRAME_TAIL       # 2字节：帧尾
        )
        return packet

    def _send_serial_packet(self, packet: bytes) -> bool:
        """发送串口数据包并打印详情"""
        if not self.ser or not self.ser.is_open:
            print("[ERROR] 串口未打开，无法发送数据")
            return False
        
        try:
            # 发送数据包
            self.ser.write(packet)
            # 打印发送的数据包（十六进制格式，便于调试）
            # packet_hex = " ".join([f"{byte:02X}" for byte in packet])
            # print(f"[SEND] 串口数据包：{packet_hex}")
            return True
        except Exception as e:
            print(f"[ERROR] 串口发送失败：{str(e)}")
            return False

    # -------------------------- 核心控制函数（构造CAN数据段+串口发送） --------------------------
    def motor_set_mode(self, motor_id: int, mode: int) -> bool:
        """设置电机模式（对应原32程序motor_set_mode）（）"""
        # 1. 构造CAN数据段（8字节）：0x7005索引 + 模式值
        can_data = bytearray(8)
        struct.pack_into("<H", can_data, 0, RUN_MODE_INDEX)  # 0x7005（小端）
        struct.pack_into("<B", can_data, 4, mode)            # 模式值存Byte4
        # 2. 构造29位CAN ID（通信类型=0x12=参数写入，主机ID=0x00FD，电机ID=motor_id）
        can_id = (0x12 << 24) | (MOTOR_MASTER_ID << 8) | motor_id
        # 3. 构造串口数据包并发送
        packet = self._build_serial_packet(can_id, can_data)
        return self._send_serial_packet(packet)

    def motor_set_current_limit(self, motor_id: int, current_limit: float) -> bool:
        """设置电机电流限制（0x7018参数）（）"""
        can_data = bytearray(8)
        struct.pack_into("<H", can_data, 0, LIMIT_CUR_INDEX)  # 0x7018（小端）
        struct.pack_into("<f", can_data, 4, current_limit)    # 电流值（float）
        can_id = (0x12 << 24) | (MOTOR_MASTER_ID << 8) | motor_id
        packet = self._build_serial_packet(can_id, can_data)
        return self._send_serial_packet(packet)

    def motor_set_speed(self, motor_id: int, speed: float) -> bool:
        """设置电机速度（0x700A参数）（）"""
        can_data = bytearray(8)
        struct.pack_into("<H", can_data, 0, SPEED_REF_INDEX)  # 0x700A（小端）
        struct.pack_into("<f", can_data, 4, speed)            # 速度值（float）
        can_id = (0x12 << 24) | (MOTOR_MASTER_ID << 8) | motor_id
        packet = self._build_serial_packet(can_id, can_data)
        return self._send_serial_packet(packet)

    def motor_enable(self, motor_id: int) -> bool:
        """使能电机（通信类型=0x03）（）"""
        can_data = b'\x00' * 8  # 使能指令数据段全零
        can_id = (0x03 << 24) | (MOTOR_MASTER_ID << 8) | motor_id
        packet = self._build_serial_packet(can_id, can_data)
        return self._send_serial_packet(packet)
    def motor_reset(self, motor_id: int) -> bool:
        """停止单个电机（通信类型4）（）"""
        can_data = b'\x80'+b'\x00' * 7
        can_id = (0x04 << 24) | (MOTOR_MASTER_ID << 8) | motor_id
        packet = self._build_serial_packet(can_id, can_data)
        return self._send_serial_packet(packet)


# -------------------------- main函数（串口发送自定义协议） --------------------------
if __name__ == "__main__":
    # 1. 初始化串口控制器
    mc = SerialMotorController(port=SERIAL_PORT, baudrate=SERIAL_BAUDRATE)
    if not mc.init_serial():
        exit(1)

    try:
        # 2. 循环初始化电机（模式→电流限制→使能）
        print("\n[INFO] 开始初始化电机...")
        time.sleep(0.5)  # 等待CAN转USB工具就绪
        
        for motor in mc.motors:
            motor_id = motor["id"]
            run_mode = motor["run_mode"]
            current_limit = motor["current_limit"]
            print(f"\n[INFO] 初始化电机{motor_id}...")
            
            # ① 设置模式
            mc.motor_set_mode(motor_id, run_mode)
            time.sleep(0.001)
            # ② 设置电流限制
            mc.motor_set_current_limit(motor_id, current_limit)
            time.sleep(0.001)
            # ③ 使能电机
            mc.motor_enable(motor_id)
            time.sleep(0.001)

        # 3. 循环发送速度指令
        print("\n[INFO] 循环发送速度指令（按Ctrl+C停止）...")
        while True:
            for motor in mc.motors:
                mc.motor_set_speed(motor["id"], motor["velocity"])
                time.sleep(0.001)
            time.sleep(0.1)  # 100ms循环一次

    except KeyboardInterrupt:
        print("\n[INFO] 停止发送指令")
        for motor in mc.motors:
            mc.motor_set_speed(motor["id"], 0)
            motor_reset = mc.motor_reset(motor["id"])


    finally:
        # 关闭串口
        if mc.ser and mc.ser.is_open:
            mc.ser.close()
            print("[INFO] 串口已关闭")