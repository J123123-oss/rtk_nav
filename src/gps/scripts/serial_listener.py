#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import serial

def talker():
    rclpy.init()
    node = rclpy.create_node('serial_publisher', anonymous=True)
    pub = node.create_publisher(String, queue_size=10, 'serial_topic')
    rate = rospy.Rate(10)  # 10 Hz

    # Open the serial port
    ser = serial.Serial('/dev/ttyUSB0', 119200)
    
    while not rospy.is_shutdown():
        if ser.in_waiting > 0:
            data = ser.readline().decode('utf-8').strip()
            # Ensure data ends with a newline
            if not data.endswith('\n'):
                data += '\n'
            # rospy.loginfo("Publishing: %s", data)
            pub.publish(data)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

# rostopic echo /serial_topic 
# data: "$GNGGA,122420.00,2140.83628676,N,11055.10256280,E,1,26,0.6,53.3950,M,-13.4261,M,,*6D"
# ---
# data: "$KSXT,20240830122420.00,110.91837605,21.68060478,53.3950,0.00,0.00,152.04,0.407,,1,0,0,26,,,,0.191,-0.359,0.606,,*0F"
# ---
# data: "$GNGGA,122421.00,2140.83607200,N,11055.10262943,E,1,26,0.6,54.5514,M,-13.4261,M,,*6A"
# ---
# data: "$KSXT,20240830122421.00,110.91837716,21.68060120,54.5514,0.00,0.00,179.68,0.421,,1,0,0,26,,,,0.002,-0.421,0.369,,*00"
# ---
# data: "$GNGGA,122422.00,2140.83603309,N,11055.10272687,E,1,25,0.6,55.3768,M,-13.4261,M,,*6E"
# ---
# data: "$KSXT,20240830122422.00,110.91837878,21.68060055,55.3768,0.00,0.00,185.33,0.315,,1,0,0,25,,,,-0.029,-0.313,0.240,,*2F"
# ---

# 打印串口：roslaunch gps serial_listener.launch 