#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import serial
import threading
import time
from std_msgs.msg import Header
from sensor_msgs.msg import NavSatFix  #GNGGA解析结果
from serial_comms.msg import WTRTK  # 替换为你的功能包名

class WTRTKSerialDriver:
    def __init__(self):
        # 初始化节点
        rospy.init_node('wtrtk_serial_driver', anonymous=True)
        
        # 读取参数（默认端口和波特率）
        self.port = rospy.get_param('~port', '/dev/WTRTK')
        self.baud_rate = rospy.get_param('~baud', 460800)
        
        # 初始化串口
        self.ser = None
        self.connect_serial()
        
        # 新增：GNGGA消息发布者（话题名：/fix）
        # self.fix_pub = rospy.Publisher('/fix', NavSatFix, queue_size=10)
        # WTRTK消息发布者（保持不变）
        self.wtrtk_pub = rospy.Publisher('/wtrtk_data', WTRTK, queue_size=10)
        
        self.buffer = ""  # 缓存串口数据，同时用于两种帧的解析
        
        # 线程与事件（保持不变，用于控制发布频率）
        self.publish_event = threading.Event()
        self.publish_thread = threading.Thread(target=self.publish_loop, daemon=True)
        self.publish_thread.start()
        self.read_thread = threading.Thread(target=self.read_serial, daemon=True)
        self.read_thread.start()
        
        rospy.loginfo("GNGGA + WTRTK serial driver started")

    def connect_serial(self):
        """连接串口设备"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                timeout=0.05,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS
            )
            if self.ser.is_open:
                rospy.loginfo(f"Connected to {self.port} at {self.baud_rate} baud")
            return True
        except Exception as e:
            rospy.logerr(f"Failed to open serial port {self.port}: {str(e)}")
            return False
    def dms_to_decimal(self, dms_str, is_latitude=True):
        """度分格式转十进制（强化空字符串和异常处理）"""
        try:
            # 先去除字符串首尾空白，处理纯空格情况
            dms_str = dms_str.strip()
            if not dms_str:  # 空字符串直接返回0.0
                return 0.0
            
            dms = float(dms_str)
            degrees = int(dms // 100)
            minutes = dms % 100
            decimal = degrees + minutes / 60.0
            
            # 校验范围（超出范围返回0.0，避免无效值）
            if is_latitude:
                if not (-90 <= decimal <= 90):
                    rospy.logwarn(f"纬度超出范围: {decimal}，重置为0.0")
                    return 0.0
            else:
                if not (-180 <= decimal <= 180):
                    rospy.logwarn(f"经度超出范围: {decimal}，重置为0.0")
                    return 0.0
            return decimal
        except (ValueError, TypeError) as e:
            # 仅在非空且无法转换时报警（空字符串已提前处理）
            if dms_str:
                rospy.logwarn(f"经纬度转换失败: '{dms_str}', 错误: {e}，重置为0.0")
            return 0.0
    def parse_gngga(self, frame):
        """解析$GNGGA帧，返回sensor_msgs/NavSatFix消息（优化空字段处理）"""
        if not frame.startswith("$GNGGA"):
            return None
        
        star_pos = frame.find('*')
        if star_pos == -1:
            rospy.logwarn("Invalid GNGGA frame (no checksum)")
            return None
        
        content = frame[7:star_pos]
        fields = content.split(',')
        
        if len(fields) < 14:
            rospy.logwarn(f"Invalid GNGGA fields count: {len(fields)} (expected >=14)")
            return None
        
        fix_msg = NavSatFix()
        fix_msg.header = Header()
        fix_msg.header.stamp = rospy.Time.now()
        fix_msg.header.frame_id = "gps"
        
        try:
            # 解析经纬度（度分格式→十进制）- 优化空字段处理
            # 纬度：字段2（可能为空）+ 字段3（N/S，可能为空）
            lat_dms = fields[2] if fields[2].strip() else "0.0"  # 空字段默认0.0
            lat_flag = fields[3].strip() if len(fields) > 3 else "N"  # 默认N
            latitude = self.dms_to_decimal(lat_dms, is_latitude=True)
            if latitude is not None and lat_flag == 'S':
                latitude = -latitude
            
            # 经度：字段4（可能为空）+ 字段5（E/W，可能为空）
            lon_dms = fields[4] if fields[4].strip() else "0.0"  # 空字段默认0.0
            lon_flag = fields[5].strip() if len(fields) > 5 else "E"  # 默认E
            longitude = self.dms_to_decimal(lon_dms, is_latitude=False)
            if longitude is not None and lon_flag == 'W':
                longitude = -longitude
            
            # 解析海拔（字段9：可能为空）
            altitude = float(fields[9]) if fields[9].strip() else 0.0
            
            # 定位状态（字段6：可能为空，默认0=未定位）
            fix_status = int(fields[6]) if fields[6].strip() else 0
            
            # 填充消息
            fix_msg.latitude = latitude if latitude is not None else 0.0
            fix_msg.longitude = longitude if longitude is not None else 0.0
            fix_msg.altitude = altitude
            fix_msg.status.status = fix_status
            fix_msg.status.service = 1
            
            # covariance
            fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            fix_msg.position_covariance = [0.1, 0, 0, 0, 0.1, 0, 0, 0, 1.0]
            
        except (ValueError, IndexError) as e:
            rospy.logwarn(f"Failed to parse GNGGA fields: {str(e)}")
            return None
        
        return fix_msg
    def parse_wtrtk(self, frame):
        """解析$WTRTK帧（严格按照WTRTK.msg结构，优化空字段处理）"""
        if not frame.startswith("$WTRTK"):
            return None
        
        star_pos = frame.find('*')
        if star_pos == -1:
            rospy.logwarn("Invalid WTRTK frame (no checksum)")
            return None
        
        content = frame[7:star_pos]
        fields = content.split(',')
        
        if len(fields) != 25:
            # rospy.logwarn(f"Invalid WTRTK fields count: {len(fields)} (expected 25)")
            return None
        
        msg = WTRTK()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "wtrtk_link"
        
        try:
            # 1. 差分相关字段（0-3）
            msg.diff_x = float(fields[0]) if fields[0].strip() else 0.0
            msg.diff_y = float(fields[1]) if fields[1].strip() else 0.0
            msg.diff_z = float(fields[2]) if fields[2].strip() else 0.0
            msg.diff_r = float(fields[3]) if fields[3].strip() else 0.0
            
            # 2. 角度相关字段（4-6）
            msg.angle_x = float(fields[4]) if fields[4].strip() else 0.0
            msg.angle_y = float(fields[5]) if fields[5].strip() else 0.0
            msg.angle_z = float(fields[6]) if fields[6].strip() else 0.0
            
            # 3. 状态相关字段（7-11）
            msg.fix_status = int(fields[7]) if fields[7].strip() else 0
            msg.wireless_status = int(fields[8]) if fields[8].strip() else 0
            msg.ntrip_status = int(fields[9]) if fields[9].strip() else 0
            msg.signal_quality = int(fields[10]) if fields[10].strip() else 0
            msg.data_rate = int(fields[11]) if fields[11].strip() else 0
            
            # 4. GPS航向角（12）
            msg.gps_heading = fields[12].strip() if fields[12].strip() else "--"
            
            # 5. 校准标志（13）
            msg.calib_flag = int(fields[13]) if fields[13].strip() else 0
            
            # 6. 电池电压和温度（14-15）
            msg.battery_voltage = float(fields[14]) if fields[14].strip() else 0.0
            msg.temperature = float(fields[15]) if fields[15].strip() else 0.0
            
            # 7. 基站距离和惯导标志（16-17）
            msg.base_distance = int(fields[16]) if fields[16].strip() else 0
            msg.ins_flag = int(fields[17]) if fields[17].strip() else 0
            
            # 8. 惯导经纬度（18-21）- 核心修复：处理空字段
            ins_lat_dms = fields[18].strip() if fields[18].strip() else "0.0"  # 空字段默认0.0
            msg.ins_latitude = self.dms_to_decimal(ins_lat_dms, is_latitude=True)
            msg.lat_flag = fields[19].strip() if fields[19].strip() else "N"  # 空字段默认N
            
            ins_lon_dms = fields[20].strip() if fields[20].strip() else "0.0"  # 空字段默认0.0
            msg.ins_longitude = self.dms_to_decimal(ins_lon_dms, is_latitude=False)
            msg.lon_flag = fields[21].strip() if fields[21].strip() else "E"  # 空字段默认E
            
            # 9. 惯导速度、航向角、高度（22-24）
            msg.ins_speed = float(fields[22]) if fields[22].strip() else 0.0
            msg.ins_heading = float(fields[23]) if fields[23].strip() else 0.0
            msg.ins_altitude = float(fields[24]) if fields[24].strip() else 0.0
            
        except (ValueError, IndexError) as e:
            rospy.logwarn(f"Failed to parse WTRTK fields: {str(e)}")
            return None
        
        return msg
    def publish_loop(self):
        """1Hz频率发布GNGGA和WTRTK数据"""
        last_fix = None
        last_wtrtk = None
        rate = rospy.Rate(1)  # 1Hz
        while not rospy.is_shutdown():
            self.publish_event.wait(timeout=1.0)
            self.publish_event.clear()
            
            # 更新最新消息缓存
            if hasattr(self, 'latest_fix'):
                last_fix = self.latest_fix
            if hasattr(self, 'latest_wtrtk'):
                last_wtrtk = self.latest_wtrtk
            
            # 发布GNGGA解析结果
            if last_fix:
                last_fix.header.stamp = rospy.Time.now()
                # self.fix_pub.publish(last_fix)
                rospy.logdebug(f"Published GNGGA (fix status: {last_fix.status.status})")
            
            # 发布WTRTK解析结果
            if last_wtrtk:
                last_wtrtk.header.stamp = rospy.Time.now()
                self.wtrtk_pub.publish(last_wtrtk)
                rospy.logdebug(f"Published WTRTK (fix status: {last_wtrtk.fix_status})")
            
            rate.sleep()
    def read_serial(self):
        """持续读取串口数据，同时解析GNGGA和WTRTK帧"""
        while not rospy.is_shutdown():
            if not self.ser or not self.ser.is_open:
                rospy.logwarn("Serial port closed, reconnecting...")
                if not self.connect_serial():
                    time.sleep(1)
                    continue
            
            try:
                data = self.ser.read(1024)
                if data:
                    # 解码并缓存数据（保留无效字符替换，避免分割错误）
                    self.buffer += data.decode('utf-8', errors='replace')
                    
                    # 循环处理缓存中的所有完整帧（同时支持GNGGA和WTRTK）
                    while True:
                        # 查找两种帧的起始位置
                        gngga_start = self.buffer.find('$GNGGA')
                        wtrtk_start = self.buffer.find('$WTRTK')
                        
                        # 没有任何帧起始，退出循环
                        if gngga_start == -1 and wtrtk_start == -1:
                            break
                        
                        # 选择最早出现的帧进行处理
                        if gngga_start != -1 and (wtrtk_start == -1 or gngga_start < wtrtk_start):
                            # 处理GNGGA帧
                            start_idx = gngga_start
                            end_idx = self.buffer.find('\r\n', start_idx)
                            if end_idx == -1:
                                break  # 未找到帧尾，等待下一次数据
                            frame = self.buffer[start_idx:end_idx]
                            self.buffer = self.buffer[end_idx+2:]  # 移除已处理部分
                            parsed_fix = self.parse_gngga(frame)
                            if parsed_fix:
                                self.latest_fix = parsed_fix  # 缓存最新GNGGA消息
                                self.publish_event.set()
                        else:
                            # 处理WTRTK帧（复用原有逻辑）
                            start_idx = wtrtk_start
                            end_idx = self.buffer.find('\r\n', start_idx)
                            if end_idx == -1:
                                break
                            frame = self.buffer[start_idx:end_idx]
                            self.buffer = self.buffer[end_idx+2:]
                            parsed_wtrtk = self.parse_wtrtk(frame)
                            if parsed_wtrtk:
                                self.latest_wtrtk = parsed_wtrtk  # 缓存最新WTRTK消息
                                self.publish_event.set()
            
            except Exception as e:
                rospy.logerr(f"Serial read error: {str(e)}")
                self.ser.close()
                time.sleep(1)

    def run(self):
        """保持节点运行"""
        rospy.spin()

if __name__ == '__main__':
    try:
        driver = WTRTKSerialDriver()
        driver.run()
    except rospy.ROSInterruptException:
        pass