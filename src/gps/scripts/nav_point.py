#!/usr/bin/env python3
import rospy
import json
import math
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class BowtieNavigator:
    def __init__(self):
        rclpy.init()
        node = rclpy.create_node('point_navigator', anonymous=True)
        
        # 1. 弓字型路径点（含目标朝向）：(经度, 纬度, 目标朝向角°)
        self.waypoints = [
            (120.06918300336037, 30.320237305773368, 90),   # A点：目标朝向90°
            (120.06895813800729, 30.32080027255977, 180),   # B点：目标朝向180°
            (120.06886808025975, 30.320800223712464, 270),  # C点：目标朝向270°
            (120.06911989030114, 30.32010573049472, 0),     # D点：目标朝向0°
            (120.0690091410155, 30.320084695329985, 90)     # E点：目标朝向90°
        ]
        self.current_idx = 0  # 当前目标点索引
        self.state = "MOVING"  # 导航状态：MOVING（行驶）/ TURNING（转向）
        
        # 2. 状态与参数
        self.current_lat = None
        self.current_lon = None
        self.current_heading = 0.0  # 当前航向（°）
        self.rtk_qual = 0
        self.initialized = False
        
        # 控制参数
        self.arrival_threshold = 0.3  # 到达坐标阈值（米）
        self.turn_threshold = 1.0     # 转向精度阈值（°）
        self.max_linear = 0.5         # 最大线速度（m/s）
        self.max_angular = 0.3        # 最大角速度（rad/s）
        self.decel_distance = 2.0     # 减速距离（米）
        
        # 3. ROS通信
        self.gps_sub = node.create_subscription(String, '/gps/raw', self.gps_callback)
        self.cmd_pub = node.create_publisher(Twist, queue_size=10, '/cmd_vel')
        rospy.loginfo("Bowtie navigator initialized.")

    def gps_callback(self, msg):
        """解析GPS数据"""
        try:
            gps_data = json.loads(msg.data)
            self.current_lat = float(gps_data['latitude'])
            self.current_lon = float(gps_data['longitude'])
            self.current_heading = float(gps_data.get('heading', 0)) if gps_data.get('heading') != 'null' else 0.0
            self.rtk_qual = gps_data.get('qual', 0)
            self.initialized = True
        except Exception as e:
            rospy.logwarn(f"GPS error: {e}")

    def haversine(self, lat1, lon1, lat2, lon2):
        """计算两点距离（米）"""
        R = 6378137.0
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        d_phi = math.radians(lat2 - lat1)
        d_lon = math.radians(lon2 - lon1)
        a = math.sin(d_phi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(d_lon/2)** 2
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

    def target_bearing(self, lat1, lon1, lat2, lon2):
        """计算目标方位角（°）"""
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        lon1_rad, lon2_rad = math.radians(lon1), math.radians(lon2)
        d_lon = lon2_rad - lon1_rad
        y = math.sin(d_lon) * math.cos(phi2)
        x = math.cos(phi1)*math.sin(phi2) - math.sin(phi1)*math.cos(phi2)*math.cos(d_lon)
        bearing = math.degrees(math.atan2(y, x))
        return (bearing + 360) % 360

    def normalize_angle(self, angle):
        """归一化角度至[-180, 180]°"""
        return angle - 360 if angle > 180 else angle + 360 if angle < -180 else angle

    def navigate(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if not self.initialized or self.rtk_qual < 4:
                rospy.loginfo("Waiting for RTK fixed solution...")
                rate.sleep()
                continue
            
            # 所有点完成
            if self.current_idx >= len(self.waypoints):
                rospy.loginfo("All waypoints completed!")
                self.cmd_pub.publish(Twist())
                break
            
            # 当前目标点信息
            target_lon, target_lat, target_heading = self.waypoints[self.current_idx]
            rospy.loginfo(f"Target {self.current_idx+1}: ({target_lon:.6f}, {target_lat:.6f}) → Heading {target_heading}°")

            if self.state == "MOVING":
                # 阶段1：行驶到目标点坐标
                distance = self.haversine(self.current_lat, self.current_lon, target_lat, target_lon)
                
                if distance < self.arrival_threshold:
                    # 到达坐标，切换到转向阶段
                    rospy.loginfo(f"Reached position {self.current_idx+1}, starting to turn...")
                    self.state = "TURNING"
                    self.cmd_pub.publish(Twist())  # 停止行驶
                    rate.sleep()
                    continue
                
                # 未到达：直线行驶控制
                bearing = self.target_bearing(self.current_lat, self.current_lon, target_lat, target_lon)
                heading_err = self.normalize_angle(bearing - self.current_heading)
                
                # 线速度（减速控制）
                linear = min(self.max_linear, distance * 0.5) if distance < self.decel_distance else self.max_linear
                # 角速度（航向纠偏）
                angular = max(-self.max_angular, min(self.max_angular, heading_err * 0.01 * math.pi / 180))
                
                cmd = Twist()
                cmd.linear.x = linear
                cmd.angular.z = angular
                self.cmd_pub.publish(cmd)

            elif self.state == "TURNING":
                # 阶段2：转向至目标朝向
                heading_err = self.normalize_angle(target_heading - self.current_heading)
                
                if abs(heading_err) < self.turn_threshold:
                    # 转向完成，切换到下一个点
                    rospy.loginfo(f"Turn completed. Heading: {self.current_heading:.1f}°")
                    self.current_idx += 1
                    self.state = "MOVING"
                    self.cmd_pub.publish(Twist())  # 停止转向
                    rate.sleep()
                    continue
                
                # 未完成：转向控制
                angular = max(-self.max_angular, min(self.max_angular, heading_err * 0.02 * math.pi / 180))
                cmd = Twist()
                cmd.angular.z = angular
                self.cmd_pub.publish(cmd)

            rate.sleep()

if __name__ == '__main__':
    try:
        nav = BowtieNavigator()
        nav.navigate()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation stopped.")