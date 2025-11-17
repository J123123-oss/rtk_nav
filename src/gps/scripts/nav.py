#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from gps.msg import LatLonList, LatLonPoint  # 自定义消息

class LatLonNavController:
    def __init__(self):
        # ROS节点初始化
        rospy.init_node('latlon_nav_controller', anonymous=True)
        
        # 订阅器：经纬度列表、惯导/里程计数据
        self.waypoint_sub = rospy.Subscriber(
            '/latlon_waypoints', LatLonList, self.waypoint_callback
        )

        self.imu_sub = rospy.Subscriber(
            '/imu/data', Imu, self.imu_callback
        )
        # self.odom_sub = rospy.Subscriber(
        #     '/odom', Odometry, self.odom_callback  # 优先用里程计（比IMU更准）
        # )
        # 发布器：控制指令
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # 目标点参数
        self.utm_waypoints = []  # 转换后的UTM坐标 [(x1,y1), (x2,y2), ...]
        self.current_waypoint_idx = 0
        self.waypoints_received = False
        
        # 机器人状态（来自里程计）
        self.robot_x = 0.0  # UTM x坐标
        self.robot_y = 0.0  # UTM y坐标
        self.robot_yaw = 0.0  # 航向角（弧度）
        self.robot_linear_vel = 0.0  # 实际线速度（m/s）
        self.robot_angular_vel = 0.0  # 实际角速度（rad/s）
        
        # 控制参数（可通过参数服务器配置）
        self.max_linear_vel = rospy.get_param('~max_linear_vel', 0.5)  # m/s
        self.max_angular_vel = rospy.get_param('~max_angular_vel', 1.0)  # rad/s
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 0.1)  # m（到达阈值）
        
        # PID参数
        self.linear_kp = 0.5
        self.linear_ki = 0.0
        self.linear_kd = 0.1
        
        self.angular_kp = 2.0
        self.angular_ki = 0.0
        self.angular_kd = 0.5
        
        # PID变量
        self.linear_error = 0.0
        self.linear_error_sum = 0.0
        self.linear_error_prev = 0.0
        
        self.angular_error = 0.0
        self.angular_error_sum = 0.0
        self.angular_error_prev = 0.0
        
        # 时间记录（用于PID微分/积分计算）
        self.prev_time = rospy.Time.now()

    def latlon_to_utm(self, lat, lon):
        """将经纬度（WGS84）转换为UTM坐标（简化实现，实际可调用utm库）"""
        # 注意：实际应用中建议使用utm库（pip install utm）
        # 示例：import utm; x, y, zone_num, zone_let = utm.from_latlon(lat, lon)
        # 此处为简化计算，假设已通过utm库转换，返回(x,y)
        import utm
        x, y, _, _ = utm.from_latlon(lat, lon)
        return (x, y)

    def waypoint_callback(self, msg):
        """处理订阅的经纬度列表，转换为UTM坐标"""
        self.utm_waypoints = []
        for pt in msg.points:
            utm_x, utm_y = self.latlon_to_utm(pt.latitude, pt.longitude)
            self.utm_waypoints.append((utm_x, utm_y))
            rospy.loginfo(f"添加目标点（UTM）：x={utm_x:.2f}, y={utm_y:.2f}")
        
        self.current_waypoint_idx = 0
        self.waypoints_received = True
        rospy.loginfo(f"收到{len(self.utm_waypoints)}个目标点")

    def odom_callback(self, msg):
        """从里程计更新机器人状态（位置、速度、航向）"""
        # 位置（假设里程计已转换为UTM坐标系）
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        # 航向角（四元数转欧拉角）
        quat = msg.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        self.robot_yaw = yaw
        
        # 速度
        self.robot_linear_vel = msg.twist.twist.linear.x
        self.robot_angular_vel = msg.twist.twist.angular.z

    #
    def imu_callback(self, msg):
        pass

    def calculate_errors(self):
        """计算当前位置与目标点的偏差（距离、航向角）"""
        if self.current_waypoint_idx >= len(self.utm_waypoints):
            return (0.0, 0.0, 0.0, 0.0)  # 无目标点
        
        # 当前目标点UTM坐标
        goal_x, goal_y = self.utm_waypoints[self.current_waypoint_idx]
        
        # 位置偏差
        dx = goal_x - self.robot_x
        dy = goal_y - self.robot_y
        distance = math.hypot(dx, dy)
        
        # 航向偏差（目标方向与当前航向的差）
        goal_yaw = math.atan2(dy, dx)  # 目标点相对机器人的方向角
        heading_error = goal_yaw - self.robot_yaw
        
        # 归一化到[-π, π]
        heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi
        return (dx, dy, distance, heading_error)

    def compute_control(self):
        """PID控制计算期望速度，并结合惯导数据纠偏"""
        dx, dy, distance, heading_error = self.calculate_errors()
        
        # 检查是否到达当前目标点
        if distance < self.goal_tolerance:
            self.current_waypoint_idx += 1
            if self.current_waypoint_idx >= len(self.utm_waypoints):
                rospy.loginfo("所有目标点已到达！")
                return (0.0, 0.0)  # 停止
            rospy.loginfo(f"到达目标点{self.current_waypoint_idx}，切换到下一个")
            return (0.0, 0.0)  # 短暂停止
        
        # 计算时间差（用于PID积分/微分）
        current_time = rospy.Time.now()
        dt = (current_time - self.prev_time).to_sec()
        self.prev_time = current_time
        if dt == 0:
            return (0.0, 0.0)
        
        # 线速度PID（基于距离偏差）
        self.linear_error = distance
        self.linear_error_sum += self.linear_error * dt
        linear_error_diff = (self.linear_error - self.linear_error_prev) / dt
        
        linear_vel = (self.linear_kp * self.linear_error 
                    + self.linear_ki * self.linear_error_sum 
                    + self.linear_kd * linear_error_diff)
        
        # 限幅（不超过最大速度，且不为负）
        linear_vel = max(0.0, min(linear_vel, self.max_linear_vel))
        self.linear_error_prev = self.linear_error
        
        # 角速度PID（基于航向偏差）
        self.angular_error = heading_error
        self.angular_error_sum += self.angular_error * dt
        angular_error_diff = (self.angular_error - self.angular_error_prev) / dt
        
        angular_vel = (self.angular_kp * self.angular_error 
                     + self.angular_ki * self.angular_error_sum 
                     + self.angular_kd * angular_error_diff)
        
        # 限幅
        angular_vel = max(-self.max_angular_vel, min(angular_vel, self.max_angular_vel))
        self.angular_error_prev = self.angular_error
        
        # 惯导纠偏：根据实际速度与期望速度的偏差调整输出
        # 例：若实际速度比期望小，增加输出（系数需调参）
        vel_error = linear_vel - self.robot_linear_vel
        linear_vel += 0.1 * vel_error  # 比例纠偏
        linear_vel = max(0.0, min(linear_vel, self.max_linear_vel))  # 再次限幅
        
        return (linear_vel, angular_vel)

    def run(self):
        """主循环：发布控制指令"""
        rate = rospy.Rate(10)  # 10Hz控制频率
        while not rospy.is_shutdown():
            if self.waypoints_received and self.current_waypoint_idx < len(self.utm_waypoints):
                linear_vel, angular_vel = self.compute_control()
                
                # 发布控制指令
                cmd = Twist()
                cmd.linear.x = linear_vel
                cmd.angular.z = angular_vel
                self.cmd_vel_pub.publish(cmd)
            
            rate.sleep()

if __name__ == '__main__':
    try:
        controller = LatLonNavController()
        controller.run()
    except rospy.ROSInterruptException:
        pass