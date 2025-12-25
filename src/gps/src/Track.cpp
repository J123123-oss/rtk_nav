#include "rclcpp/rclcpp.hpp" //包含ROS的头文件，用于ROS的初始化和节点的创建。
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp> //Twist消息类型用于控制机器人的线速度和角速度。
#include <stdio.h>
#include <termios.h> //包含终端I/O的头文件。
#include "yhs_can_control.h" //包含自定义的CAN控制头文件。
#include <queue>
#include <iostream>
#include <serial/serial.h>



class RosBridge {
 
    private:
        bool shouldSendStopCmd = false;
       

    public:
        rclcpp::Node nh_;
        
        
      发布到/ctrl_cmd话题 小车控制指令
        ros::Publisher pub_state_;       //发布小车在线状态

        serial::Serial ser_;  //定义串口对象

    
        RosBridge(rclcpp::Node &nh) : nh_(nh)  {
            // 订阅ctrl_fb话题，并执行回调函数，消息队列容量为10
            subBmsFb_ = nh_.subscribe("/bms_fb",10, &RosBridge::bmsFbCallback, this);
            // 发布到话题
            pub_ctrl_data_ = nh_.advertise<std_msgs::msg::String>("/xun/100001/fb", 5);
            initBaseCmd();
            // 创建定时器 5秒触发一次
           ms_fb_timer_ = nh_.createTimer(rclcpp::Duration(5.0), &RosBridge::processBmsFb, this);
             // 每隔10秒发布一次
            online_state_timer_ = nh_.createTimer(rclcpp::Duration(10.0), &RosBridge::updateState, this); 
        }
        
        
        }
        //线速度、角速度消息处理
        void processCtrlFb(const rclcpp::TimerEvent&) {
            if (new_ctrl_fb_received_) {
                std::string data_fb = std::to_string(latest_ctrl_fb_.ctrl_fb_target_gear) + "|" +
                                    std::to_string(latest_ctrl_fb_.ctrl_fb_linear) + "|" +
                                    std::to_string(latest_ctrl_fb_.ctrl_fb_angular);
                std_msgs::msg::String ms;
                ms.data = data_fb;
                pub_ctrl_data_.publish(ms);
                new_ctrl_fb_received_ = false; // 重置标志
            }
        }
        void ctrlFbCallback(const yhs_can_msgs::ctrl_fb::ConstPtr& msg) {
            latest_ctrl_fb_ = *msg;
            new_ctrl_fb_received_ = true;
        }

        void bmsFbCallback(const yhs_can_msgs::bms_fb::ConstPtr& msg) {
            latest_bms_fb_ = *msg;
            new_bms_fb_received_ = true; 
        }

        void cmdCallback(const std_msgs::msg::String::ConstSharedPtr& msg) {
            std::string data = msg->data;
            RCLCPP_INFO(rclcpp::get_logger("Gps"), "Received from topic xunjian/100001/cmd: %s", msg->data.c_str());   
            updateBaseCmd(data);//小车运动控制
            publishBaseCmd(); //发布小车控制底层指令 
        }
    
        void initBaseCmd() {
            base_cmd.ctrl_cmd_gear = 03;//档位3
            base_cmd.ctrl_cmd_linear = 0;
            base_cmd.ctrl_cmd_angular = 0;
        }
    
        void updateBaseCmd(const std::string& data) {  //有字符串，所以别用switch-case
            if (data == "w") {
                base_cmd.ctrl_cmd_linear = 0.08;  // 向前移动
                base_cmd.ctrl_cmd_angular = 0;  // 停止旋转
                RCLCPP_INFO(rclcpp::get_logger("Gps"), "-前进-");
            }else if(data == "s"){
                base_cmd.ctrl_cmd_linear = -0.08;  // 向后移动
                base_cmd.ctrl_cmd_angular = 0;  // 停止旋转
                RCLCPP_INFO(rclcpp::get_logger("Gps"), "-后退-");
          
            else {
                base_cmd.ctrl_cmd_linear = 0;  // 停止线性移动
                base_cmd.ctrl_cmd_angular = 0;  // 停止旋转
                RCLCPP_ERROR(rclcpp::get_logger("Gps"), "-未定义指令-");
            }
        }

        void publishBaseCmd() {
            pub_cmd_vel_.publish(base_cmd);
        }
};



int main(int argc, char **argv) {//  形参数量    数组
    setlocale(LC_ALL,"");
    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("ros_mqtt_control");
    RosBridge bridge(nh);
    std::string port_name = "/dev/ttyUSB1";
    if (!bridge.initSerial(port_name, 2400)) {
        ROS_FATAL_STREAM("-初始化串口失败-");
        return -1;
    }
    rclcpp::spin(node);//循环监听并处理所有订阅的话题
    // 在程序结束前关闭串口
    bridge.ser_.close();
    return 0;
}