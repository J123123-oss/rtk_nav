#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import threading
import time
from std_msgs.msg import Header
from sensor_msgs.msg import NavSatFix  # GNGGA解析结果
from serial_comms.msg import WTRTK  # 替换为你的功能包名

class WTRTKFileParser:
    def __init__(self):
        # 初始化节点
        rclpy.init()
        node = rclpy.create_node('wtrtk_file_parser', anonymous=True)
        
        # 读取文件路径参数（默认：./返回.txt）
        self.file_path = rospy.get_param('~file_path', '/home/ubuntu/rtk_nav/返回.txt')
        # 读取播放速率参数（单位：秒/行，默认0.1秒）
        self.play_rate = rospy.get_param('~play_rate', 0.1)
        
        # 消息发布者
        self.fix_pub = node.create_publisher(NavSatFix, queue_size=10, '/fix')
        self.wtrtk_pub = node.create_publisher(WTRTK, queue_size=10, '/wtrtk_data')
        
        self.buffer = ""  # 缓存文件读取的数据
        self.latest_fix = None  # 最新GNGGA解析结果
        self.latest_wtrtk = None  # 最新WTRTK解析结果
        
        # 线程与事件
        self.publish_event = threading.Event()
        self.publish_thread = threading.Thread(target=self.publish_loop, daemon=True)
        self.publish_thread.start()
        self.read_thread = threading.Thread(target=self.read_file, daemon=True)
        self.read_thread.start()
        
        rospy.loginfo(f"GNGGA + WTRTK file parser started. Reading from: {self.file_path}, play rate: {self.play_rate}s/line")

    def dms_to_decimal(self, dms_str, is_latitude=True):
        """度分格式转十进制（强化空字符串和异常处理）"""
        try:
            dms_str = dms_str.strip()
            if not dms_str:
                return 0.0
            
            dms = float(dms_str)
            degrees = int(dms // 100)
            minutes = dms % 100
            decimal = degrees + minutes / 60.0
            
            # 校验范围
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
            if dms_str:
                rospy.logwarn(f"经纬度转换失败: '{dms_str}', 错误: {e}，重置为0.0")
            return 0.0

    def parse_gngga(self, frame):
        """解析$GNGGA帧（修正字段索引，适配标准GNGGA格式）"""
        if not frame.startswith("$GNGGA"):
            return None
        
        star_pos = frame.find('*')
        if star_pos == -1:
            rospy.logwarn("Invalid GNGGA frame (no checksum)")
            return None
        
        content = frame[7:star_pos]
        fields = content.split(',')
        
        # 标准GNGGA字段数为14（不含帧头和校验位），允许字段数>=14（兼容扩展字段）
        if len(fields) < 14:
            rospy.logwarn(f"Invalid GNGGA fields count: {len(fields)} (expected >=14)")
            return None
        
        fix_msg = NavSatFix()
        fix_msg.header = Header()
        fix_msg.header.stamp = rospy.Time.now()
        fix_msg.header.frame_id = "gps"
        
        try:
            # 修正GNGGA字段索引（关键修复！）
            # 标准GNGGA格式（字段索引对应）：
            # 0: 时间, 1: 纬度(度分), 2: 纬度标志(N/S), 3: 经度(度分), 4: 经度标志(E/W)
            # 5: 定位状态, 6: 卫星数, 7: HDOP, 8: 海拔(数值), 9: 海拔单位(M)
            # 10: 大地水准面高度, 11: 大地水准面高度单位, 12: 差分年龄, 13: 差分站ID
            
            # 纬度解析（字段1 + 字段2）
            lat_dms = fields[1] if fields[1].strip() else "0.0"
            lat_flag = fields[2].strip() if len(fields) > 2 and fields[2].strip() else "N"
            latitude = self.dms_to_decimal(lat_dms, is_latitude=True)
            if latitude is not None and lat_flag == 'S':
                latitude = -latitude
            
            # 经度解析（字段3 + 字段4）
            lon_dms = fields[3] if fields[3].strip() else "0.0"
            lon_flag = fields[4].strip() if len(fields) > 4 and fields[4].strip() else "E"
            longitude = self.dms_to_decimal(lon_dms, is_latitude=False)
            if longitude is not None and lon_flag == 'W':
                longitude = -longitude
            
            # 海拔解析（字段8：数值，字段9：单位）
            altitude = float(fields[8]) if len(fields) > 8 and fields[8].strip() else 0.0
            
            # 定位状态（字段5）
            fix_status = int(fields[5]) if len(fields) > 5 and fields[5].strip() else 0
            
            # 填充消息
            fix_msg.latitude = latitude if latitude is not None else 0.0
            fix_msg.longitude = longitude if longitude is not None else 0.0
            fix_msg.altitude = altitude
            fix_msg.status.status = fix_status
            fix_msg.status.service = 1  # GPS服务
            
            #  covariance
            fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            fix_msg.position_covariance = [0.1, 0, 0, 0, 0.1, 0, 0, 0, 1.0]
            
        except (ValueError, IndexError) as e:
            rospy.logwarn(f"Failed to parse GNGGA fields: {str(e)}")
            return None
        
        return fix_msg

    def parse_wtrtk(self, frame):
        """解析$WTRTK帧（修正经纬度字段索引，适配实际消息格式）"""
        if not frame.startswith("$WTRTK"):
            return None
        
        star_pos = frame.find('*')
        if star_pos == -1:
            rospy.logwarn("Invalid WTRTK frame (no checksum)")
            return None
        
        content = frame[7:star_pos]
        fields = content.split(',')
        
        # 从实际消息看，WTRTK字段数为25（保持校验）
        if len(fields) != 25:
            rospy.logwarn(f"Invalid WTRTK fields count: {len(fields)} (expected 25)")
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
            
            # 关键修复：WTRTK经纬度字段索引（根据实际消息格式修正）
            # 实际消息中WTRTK经纬度字段位置：18=纬度(度分), 19=纬度标志, 20=经度(度分), 21=经度标志
            # 与原代码一致，但需要确保字段不为空时正确处理
            ins_lat_dms = fields[18].strip() if fields[18].strip() else "0.0"
            msg.ins_latitude = self.dms_to_decimal(ins_lat_dms, is_latitude=True)
            msg.lat_flag = fields[19].strip() if fields[19].strip() else "N"
            
            ins_lon_dms = fields[20].strip() if fields[20].strip() else "0.0"
            msg.ins_longitude = self.dms_to_decimal(ins_lon_dms, is_latitude=False)
            msg.lon_flag = fields[21].strip() if fields[21].strip() else "E"
            
            # 9. 惯导速度、航向角、高度（22-24）
            msg.ins_speed = float(fields[22]) if fields[22].strip() else 0.0
            msg.ins_heading = float(fields[23]) if fields[23].strip() else 0.0
            msg.ins_altitude = float(fields[24]) if fields[24].strip() else 0.0
            
        except (ValueError, IndexError) as e:
            rospy.logwarn(f"Failed to parse WTRTK fields: {str(e)}")
            return None
        
        return msg

    def publish_loop(self):
        """1Hz频率发布数据"""
        last_fix = None
        last_wtrtk = None
        rate = rospy.Rate(1)  # 1Hz
        while not rospy.is_shutdown():
            self.publish_event.wait(timeout=1.0)
            self.publish_event.clear()
            
            if hasattr(self, 'latest_fix'):
                last_fix = self.latest_fix
            if hasattr(self, 'latest_wtrtk'):
                last_wtrtk = self.latest_wtrtk
            
            # 发布GNGGA解析结果（取消注释即可启用）
            if last_fix:
                last_fix.header.stamp = rospy.Time.now()
                self.fix_pub.publish(last_fix)
                rospy.logdebug(f"Published GNGGA: lat={last_fix.latitude:.6f}, lon={last_fix.longitude:.6f}, alt={last_fix.altitude:.2f}")
            
            # 发布WTRTK解析结果
            if last_wtrtk:
                last_wtrtk.header.stamp = rospy.Time.now()
                self.wtrtk_pub.publish(last_wtrtk)
                rospy.logdebug(f"Published WTRTK: ins_lat={last_wtrtk.ins_latitude:.6f}, ins_lon={last_wtrtk.ins_longitude:.6f}")
            
            rate.sleep()

    def read_file(self):
        """从文件读取串口消息并处理"""
        while not rospy.is_shutdown():
            try:
                with open(self.file_path, 'r', encoding='utf-8') as f:
                    rospy.loginfo(f"Successfully opened file: {self.file_path}")
                    
                    for line in f:
                        if rospy.is_shutdown():
                            break
                            
                        line = line.strip()
                        if not line:
                            continue
                        
                        self.buffer += line + "\r\n"
                        self.parse_buffer()
                        time.sleep(self.play_rate)
                
                rospy.loginfo(f"File read completed. Re-reading after 1 second...")
                time.sleep(1)
                
            except FileNotFoundError:
                rospy.logerr(f"File not found: {self.file_path}")
                time.sleep(2)
            except Exception as e:
                rospy.logerr(f"File read error: {str(e)}")
                time.sleep(1)

    def parse_buffer(self):
        """解析缓存中的完整帧"""
        while True:
            gngga_start = self.buffer.find('$GNGGA')
            wtrtk_start = self.buffer.find('$WTRTK')
            
            if gngga_start == -1 and wtrtk_start == -1:
                break
            
            if gngga_start != -1 and (wtrtk_start == -1 or gngga_start < wtrtk_start):
                # 处理GNGGA帧
                start_idx = gngga_start
                end_idx = self.buffer.find('\r\n', start_idx)
                if end_idx == -1:
                    break
                frame = self.buffer[start_idx:end_idx]
                self.buffer = self.buffer[end_idx+2:]
                parsed_fix = self.parse_gngga(frame)
                if parsed_fix:
                    self.latest_fix = parsed_fix
                    self.publish_event.set()
            else:
                # 处理WTRTK帧
                start_idx = wtrtk_start
                end_idx = self.buffer.find('\r\n', start_idx)
                if end_idx == -1:
                    break
                frame = self.buffer[start_idx:end_idx]
                self.buffer = self.buffer[end_idx+2:]
                parsed_wtrtk = self.parse_wtrtk(frame)
                if parsed_wtrtk:
                    self.latest_wtrtk = parsed_wtrtk
                    self.publish_event.set()

    def run(self):
        """保持节点运行"""
        rospy.spin()

if __name__ == '__main__':
    try:
        parser = WTRTKFileParser()
        parser.run()
    except rospy.ROSInterruptException:
        pass