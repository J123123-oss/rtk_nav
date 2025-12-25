#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>

void send_message(ros::Publisher& pub, int& counter, const std::vector<std::string>& messages, int group_size)
{
    std_msgs::msg::String msg;
    
    // 计算当前组的起始位置
    int start_index = (counter * group_size) % messages.size();
    
    // 将接下来的group_size行组合成一个字符串
    std::stringstream combined_msg;
    for (int i = 0; i < group_size && (start_index + i) < messages.size(); ++i)
    {
        combined_msg << messages[start_index + i] << "\n";
    }
    
    msg.data = combined_msg.str();
    pub.publish(msg);
    
    // 更新计数器，表示已发送的组数
    counter++;
}

std::vector<std::string> load_messages_from_file(const std::string& file_path)
{
    std::vector<std::string> messages;
    std::ifstream file(file_path);

    if (!file.is_open())
    {
        RCLCPP_ERROR(rclcpp::get_logger("Gps"), "Failed to open file: %s", file_path.c_str());
        return messages;
    }

    std::string line;
    while (std::getline(file, line))
    {
        // 添加读取的每一行到消息列表
        messages.push_back(line);
    }

    file.close();
    return messages;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("serial_publisher");
    auto pub = nh.advertise<std_msgs::msg::String>("rosmsg", 1000);
    rclcpp::Rate loop_rate(10); // 10 Hz, 发布频率可以根据需要调整

    // 从文件中加载消息
    std::vector<std::string> messages = load_messages_from_file("/home/ubuntu/robot_solar/src/gps/rtkmsgs/返回.txt");
    if (messages.empty())
    {
        RCLCPP_ERROR(rclcpp::get_logger("Gps"), "No messages loaded from the file.");
        return 1;
    }

    int counter = 0;
    int group_size = 10; // 每组包含10行消息
    while (rclcpp::ok())
    {
        send_message(pub, counter, messages, group_size);
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    return 0;
}






// #include "rclcpp/rclcpp.hpp"
// #include <std_msgs/msg/string.hpp>
// #include <fstream>
// #include <vector>
// #include <string>

// void send_message(ros::Publisher& pub, int& counter, const std::vector<std::string>& messages)
// {
//     std_msgs::msg::String msg;
    
//     // 如果计数器超出了信息列表的范围，重新从头开始发布
//     int index = counter % messages.size();
//     msg.data = messages[index];

//     pub.publish(msg);
//     counter++;
// }

// std::vector<std::string> load_messages_from_file(const std::string& file_path)
// {
//     std::vector<std::string> messages;
//     std::ifstream file(file_path);

//     if (!file.is_open())
//     {
//         RCLCPP_ERROR(rclcpp::get_logger("Gps"), "Failed to open file: %s", file_path.c_str());
//         return messages;
//     }

//     std::string line;
//     while (std::getline(file, line))
//     {
//         // 添加读取的每一行到消息列表
//         messages.push_back(line);
//     }

//     file.close();
//     return messages;
// }

// int main(int argc, char** argv)
// {
//     rclcpp::init(argc, argv);
     auto node = rclcpp::Node::make_shared("serial_publisher");
//     rclcpp::Node nh;
//     auto pub = nh.advertise<std_msgs::msg::String>("rosmsg", 1000);
//     rclcpp::Rate loop_rate(100); // 10 Hz

//     // 从文件中加载消息
//     // std::vector<std::string> messages = load_messages_from_file("/home/orangepi/ros_ws/src/gps/rtkmsgs/bd1.txt");
//     std::vector<std::string> messages = load_messages_from_file("/home/rosubuntu/robot_solar/src/gps/rtkmsgs/bd1.txt");
//     if (messages.empty())
//     {
//         RCLCPP_ERROR(rclcpp::get_logger("Gps"), "No messages loaded from the file.");
//         return 1;
//     }

//     int counter = 0;
//     while (rclcpp::ok())
//     {
//         send_message(pub, counter, messages);
//         rclcpp::spin_some(node);
//         loop_rate.sleep();
//     }

//     return 0;
// }

// BD1:gps->latitude:30.32101833, gps->longitude:120.07105000, gps->qual:4, gps->direction:0.00000000"

// BD2:gps->latitude:30.32098833, gps->longitude:120.07123333, gps->qual:4, gps->direction:0.00000000"

// BD3:gps->latitude:30.32086333, gps->longitude:120.07125000, gps->qual:4, gps->direction:0.00000000

























// void send_message(ros::Publisher& pub, int& counter)
// {
//     std_msgs::msg::String msg;

//     if (counter % 2 == 0) {
//         msg.data = "$GNGGA,101620.00,2140.83682949,N,11055.09963460,E,1,20,0.7,59.7145,M,-13.4263,M,,*67\n";
//     } else {
//         msg.data = "$GPRMC,094403.00,A,4004.73794422,N,11614.18999462,E,0.037,5.5,260815,6.5,W,A*35\n";
//     }

//     pub.publish(msg);
//     counter++;
// }

// int main(int argc, char** argv)
// {
//     rclcpp::init(argc, argv);
     auto node = rclcpp::Node::make_shared("serial_publisher");
//     rclcpp::Node nh;
//     auto pub = nh.advertise<std_msgs::msg::String>("serial_topic", 1000);
//     rclcpp::Rate loop_rate(1); // 1 Hz
//     int counter = 0;
//     while (rclcpp::ok())
//     {
//         send_message(pub, counter);
//         rclcpp::spin_some(node);
//         loop_rate.sleep();
//     }

//     return 0;
// }
 // 1
 // gps->latitude:21.680427, gps->longitude:110.918648, gps->speed:0.003700, gps->direction:19.200001
 // gps->time: 17:51:05
 // 2
 // gps->latitude:21.680228, gps->longitude:110.918682, gps->speed:0.590150, gps->direction:183.800003
 // gps->time: 17:56:09
//  距离: 0.022314 公里
//  方位角: 170.926928 度


//  第二处参考点
//  gps1.latitude = 21.68007020091791;//随机选取：主教左下角
//  gps1.longitude = 110.91824768309247;// 
// 
//  posi_now.latitude = 21.680078262699684;
//  posi_now.longitude = 110.91905925768252;//地图选点：主教右下角
// 距离: 83.996350M
// 方位角: 89.390943°
