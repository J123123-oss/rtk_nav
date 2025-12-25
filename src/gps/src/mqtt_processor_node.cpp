#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>
#include <cstring>// For std::strlen
#include <stdio.h>
#include <math.h>
#include <iostream>
#include "/home/ubuntu/robot_solar/src/gps/include/gps.h"
// #include <stdint.h>  // 包含标准整数类型定义
#include <cstdint>
#include <iomanip>  // For std::setw and std::setfill
#include <nlohmann/json.hpp> // 包含 JSON 库头文件
#include <sstream>

// 使用 nlohmann::json 命名空间
using json = nlohmann::json;

uint8_t uart2_get_buffer;
gps_info_struct gps_tau1201;
char gps_tau1201_buffer1[256];  // 语句获取后将数据转移到此数组，然后开始解析语句内容
char gps_tau1201_buffer2[256];  // 串口接收缓冲区
uint8_t gps_tau1201_num = 0;           // 当前接收字符数量
int gps_tau1201_flag = 0; // Define and initialize the variable


class SerialProcessor {
public:
    SerialProcessor() {
        // 初始化ROS节点和订阅者
        rclcpp::Node nh;
        sub_ = nh.subscribe("/rosmsg", 1000, &SerialProcessor::serialCallback, this);//调试文本消息
        // sub_ = nh.subscribe("/mqttmsg", 1000, &SerialProcessor::serialCallback, this);
        // 初始化数据缓冲区和标志
        // gps_tau1201_num = 0;
        // gps_tau1201_flag = 0;
        printf("Processor node initialized.\n" );
    }

void serialCallback(const std_msgs::msg::String::ConstSharedPtr& msg) {
    // RCLCPP_INFO(rclcpp::get_logger("Gps"), "Callback called with message: %s", msg->data.c_str());
    
    const std::string& data = msg->data;
    // printf("Received data: %s\n", data.c_str());

for (char dat : data) {
    // std::cout << "Current char: " << dat << ", gps_tau1201_buffer2[0]: " << gps_tau1201_buffer2[0] << std::endl;
    if (dat == '$' || dat == '#' || (gps_tau1201_buffer2[0] != '$' && gps_tau1201_buffer2[0] != '#')) {
        gps_tau1201_num = 0;
    }
    



        // 检查缓冲区是否溢出
        if (gps_tau1201_num >= sizeof(gps_tau1201_buffer2) - 1) {
            RCLCPP_WARN(rclcpp::get_logger("Gps"), "Buffer overflow, discarding data.");
            continue;
        }

        // 将字符存入缓冲区
        gps_tau1201_buffer2[gps_tau1201_num++] = dat;

        // 检查是否到达行尾
        if (dat == '\n') {
        // if ((dat == '\r' || dat == '\n') && gps_tau1201_num > 1) {
            gps_tau1201_buffer2[gps_tau1201_num] = '\0';  // 在末尾添加 null 终止符
            gps_tau1201_num++;

            // 拷贝数据到 gps_tau1201_buffer1
            std::memcpy(gps_tau1201_buffer1, gps_tau1201_buffer2, gps_tau1201_num);
            gps_tau1201_flag = 1;

            // 清空缓冲区
            std::fill(std::begin(gps_tau1201_buffer2), std::end(gps_tau1201_buffer2), 0);

            // 打印接收到的完整消息
            std::string message(gps_tau1201_buffer1);
            // std::cout << "serialCallback: " << gps_tau1201_buffer1 << std::endl;

                        // 在 serialCallback 中
            // std::cout << "gps_data_parse - : " << gps_tau1201_buffer1 << std::endl;
                // RCLCPP_INFO(rclcpp::get_logger("Gps"), "Received complete message: %s", message.c_str());
        }
    }
}
private:
    ros::Subscriber sub_;
    // char gps_tau1201_buffer1[256];
    // char gps_tau1201_buffer2[256];
    // int gps_tau1201_num;
    // int gps_tau1201_flag;
};



//-----------------------------------------------------------------------------
// void gps_data_parse (void)
// {
// 	// printf("gps_tau1201_flag:%d\n",gps_tau1201_flag);   //flag=0
//     // printf("gps_data_parse中IF之前的标志位:%d\n",gps_tau1201_flag);   //flag=最初0后续1
//     if(gps_tau1201_flag)
//     {
// 		gps_tau1201_flag = 0;
 
// 		if(0 == std::strncmp((char *)&gps_tau1201_buffer1[3], "RMC", 3))//"RMC"
// 		{
            
//         std::cout << "Buffer长度: " << std::string((char *)&gps_tau1201_buffer1[3]).length() << std::endl;

// 			gps_gnrmc_parse((char *)gps_tau1201_buffer1, &gps_tau1201);
// 		}
		
// 		else if(0 == std::strncmp((char *)&gps_tau1201_buffer1[3], "GGA", 3))
// 		{
			
//             std::cout << "GGA循环内容: " << std::string((char *)&gps_tau1201_buffer1[3], 3) << std::endl;//--------------------
//             // printf("gps_tau1201_buffer1:%s\n",gps_tau1201_buffer1);
//             gps_gngga_parse((char *)gps_tau1201_buffer1, &gps_tau1201);
// 		}
// 	}
// }
static uint8_t gps_heading_parse (char *line, gps_info_struct *gps);
void gps_data_parse(void) {
    if (gps_tau1201_flag) {
        gps_tau1201_flag = 0;
        
        // std::cout << "gps_data_parse: " << gps_tau1201_buffer1 << std::endl;
        // 确保 gps_tau1201_buffer1 以 null 终止符结尾
        std::string buffer_str(reinterpret_cast<char*>(gps_tau1201_buffer1));
        // std::cout << "Received buffer: " << buffer_str << std::endl;
        
    // 确保 buffer_str 至少包含 6 个字符（GGA 的最小长度）
if (buffer_str.length() >= 6) {
    // 检查前缀是否为 "#UNIHEADINGA"
    if (std::strncmp(buffer_str.c_str(), "#UNIHEADINGA", 12) == 0) {
        // std::cout << "UNIHEADINGA的Buffer长度: " << buffer_str.length() << std::endl;
        // 确保缓冲区大小足够
        size_t buffer_length = std::strlen(reinterpret_cast<char*>(gps_tau1201_buffer1));
        if (buffer_length >= 70) {
            // 调用解析函数
            gps_heading_parse(reinterpret_cast<char*>(gps_tau1201_buffer1), &gps_tau1201);
        }
    } 
    
    // 检查后三位是否为 "GGA"
    if (buffer_str.substr(3, 3) == "GGA") {
        // std::cout << "GNGGA的Buffer长度: " << buffer_str.length() << std::endl;
        // 如果是 GGA 数据，调用 GGA 解析函数
        gps_gngga_parse(reinterpret_cast<char*>(gps_tau1201_buffer1), &gps_tau1201);
    }
}
    
    // 确保 buffer_str 至少包含 6 个字符（GGA 的最小长度）
    // if (buffer_str.length() >= 6) {
    //     // 检查前缀是否为 "#UNIHEADINGA"
    //     if (std::strncmp(buffer_str.c_str(), "#UNIHEADINGA", 12) == 0) {
    //         std::cout << "UNIHEADINGA的Buffer长度: " << buffer_str.length() << std::endl;
    //         // 确保缓冲区大小足够
    //         size_t buffer_length = std::strlen(reinterpret_cast<char*>(gps_tau1201_buffer1));
    //         if (buffer_length >= 70) {
    //             // 调用解析函数
    //             gps_heading_parse(reinterpret_cast<char*>(gps_tau1201_buffer1), &gps_tau1201);
    //         }
    //         //  else {
    //         //     std::cerr << "Error: Buffer length is too short for parsing." << std::endl;
    //         // }
    //     } 
    //     // 检查后三位是否为 "GGA"
    //     if (buffer_str.substr(3, 3) == "GGA") {
    //         std::cout << "GNGGA的Buffer长度: " << buffer_str.length() << std::endl;
    //         // 如果是 GGA 数据，调用 GGA 解析函数
    //         gps_gngga_parse(reinterpret_cast<char*>(gps_tau1201_buffer1), &gps_tau1201);
    //     } 
    //     // else {
    //     //     std::cerr << "Error: Unsupported prefix. Received: " << buffer_str.substr(0, 12) << std::endl;
    //     // }
    // // } else {
    // //     std::cerr << "Error: Input string is too short. Length: " << buffer_str.length() << std::endl;
    //     }


    }
}


//-------------------------------------------------------------------------------------------------------------------
// @brief		获取指定,后面的索引
// @param		num             第几个逗号
// @param		*str            字符串           
// @return		uint8_t           返回索引
// Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
// static uint8_t get_parameter_index (uint8_t num, char *str)
// {
// 	uint8_t i, j = 0;
//     char *temp;
//     size_t len = 0, len1;
    
//     // temp = strchr(str, '\n');
//     // if(NULL != temp)
//     // {
//     //     // len = (uint32_t)temp - (uint32_t)str + 1;//最初的写法
//     //     // size_t len = (size_t)temp - (size_t)str + 1;//修改1
//     //     len = (size_t)temp - (size_t)str + 1;//修改2

//     // }
//     std::string str_str = str;  // 将 char* 转换为 std::string
//     size_t pos = str_str.find('\n');

//     if (pos != std::string::npos) {
//         len = pos + 1;
//     }
//     printf("len = %ld\n", len);
// 	for(i = 0; i < len; i ++)
// 	{
// 		if(str[i] == ',')
//         {
//             j ++;
//         }
// 		if(j == num)
//         {
//             len1 =  i + 1;	
//             break;
//         }
// 	}

// 	return len1;
// }

// static uint8_t get_parameter_index(uint8_t num, char *str) {
//     std::string str_str = str;
//     size_t pos = str_str.find('\n');
//     size_t len = (pos != std::string::npos) ? pos + 1 : str_str.length();
    
//     uint8_t j = 0;
//     size_t len1 = 0;
//     for (size_t i = 0; i < len; ++i) {
//         if (str[i] == ',') {
//             ++j;
//         }
//         if (j == num) {
//             len1 = static_cast<uint8_t>(i + 1);
//             break;
//         }
//     }
//     return len1;
// }

static uint8_t get_parameter_index(uint8_t num, char *str) {
    std::string str_str = str;
    size_t pos = str_str.find('\n');
    size_t len = (pos != std::string::npos) ? pos + 1 : str_str.length();

    uint8_t j = 0;
    size_t len1 = 0;
    for (size_t i = 0; i < len; ++i) {
        if (str[i] == ',') {
            ++j;
        }
        if (j == num) {
            len1 = static_cast<uint8_t>(i + 1);
            break;
        }
    }

    // Check if index is valid
    if (num > j) {
        len1 = 0;
    }

    return len1;
}

//-------------------------------------------------------------------------------------------------------------------
// @brief		给定字符串第一个,之前的数据转换为double	
// @param		*s              字符串
// @return		double          返回数值
// Sample usage:			
// static double get_double_number (char *s)
// {
//     uint8_t i;
// 	char buf[10];
// 	double return_value;
    
// 	i = get_parameter_index(1, s);
// 	i = i - 1;
// 	strncpy(buf, s, i);
// 	buf[i] = 0;
// 	return_value = str_to_double(buf);
// 	return return_value;
// }

// @brief        给定字符串第一个,之前的数据转换为double  
// @param        *s              字符串
// @return       double          返回数值
// Sample usage:            
// static double get_double_number (char *s)
// {
//     uint8_t i;
//     char buf[10];
//     double return_value;
//     printf("get_double_number: %s\n", s);
//     // 获取参数的索引位置
//     i = get_parameter_index(1, s);
    
//     // 检查索引是否超出buf的长度
//     if (i > sizeof(buf) - 1) {
//         // 如果超出范围，限制i为buf的大小 - 1，并输出错误信息
//         std::cerr << "Warning: Index out of bounds. double Limiting to " << (sizeof(buf) - 1) << "." << std::endl;
//         i = sizeof(buf) - 1;
//     }

//     // 复制数据到buf中
//     strncpy(buf, s, i);
    
//     // 确保buf以null结尾
//     buf[i] = '\0';
    
//     // 将buf转换为double
//     return_value = str_to_double(buf);
    
//     return return_value;
// }

//第3版  报错
// get_double_number: 11055.10021596,E,0.025,232.7,020924,2.6,W,A,V*45

// Warning: Index out of bounds. Double Limiting to 9.

static double get_double_number(char *s) {
    uint8_t i;
    char buf[10];
    double return_value;

    // printf("get_double_number: %s\n", s);
    
    // 获取参数的索引位置
    i = get_parameter_index(1, s);

    // Check if index is within bounds
    if (i >= sizeof(buf)) {
        // std::cerr << "Warning: Index out of bounds. Double Limiting to " << (sizeof(buf) - 1) << "." << std::endl;
        i = sizeof(buf) - 1;
    }

    // Copy data to buf and ensure null termination
    strncpy(buf, s, i);
    buf[i] = '\0';

    // Convert buf to double
    return_value = str_to_double(buf);
    //打印return_value
    // printf("return_value: %f\n", return_value);
    return return_value;
}
// Dummy implementation of str_to_double for completeness
static double str_to_double(const char* str) {
    return std::strtod(str, nullptr);
}

//-------------------------------------------------------------------------------------------------------------------
// @brief		给定字符串第一个,之前的数据转换为int
// @param		*s              字符串
// @return		float           返回数值
// Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
static int get_int_number (char *s)
{
	char buf[10];
	uint8_t i;
	int return_value;
	i = get_parameter_index(1, s);
	i = i - 1;
	strncpy(buf, s, i);
	buf[i] = 0;
	return_value = str_to_int(buf);
	return return_value;
}

//-------------------------------------------------------------------------------------------------------------------
// @brief		给定字符串第一个,之前的数据转换为float
// @param		*s              字符串
// @return		float           返回数值
// Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
// static float get_float_number (char *s)
// {
//     uint8_t i;
// 	char buf[10];
// 	float return_value;
    
// 	i=get_parameter_index(1, s);
// 	i = i - 1;
// 	strncpy(buf, s, i);
// 	buf[i] = 0;
// 	return_value = (float)str_to_double(buf);
// 	return return_value;	
// }

static float get_float_number (char *s)
{
    uint8_t i;
    char buf[10];
    float return_value;

    // 获取参数的索引位置
    i = get_parameter_index(1, s);
    
    // 检查索引是否超出buf的长度
    if (i > sizeof(buf) - 1) {
        // 如果超出范围，限制i为buf的大小 - 1，并输出警告信息
        std::cerr << "Warning: Index out of bounds. float Limiting to " << (sizeof(buf) - 1) << "." << std::endl;
        i = sizeof(buf) - 1;
    }

    // 复制数据到buf中
    strncpy(buf, s, i);
    
    // 确保buf以null结尾
    buf[i] = '\0';
    
    // 将buf转换为double后再转换为float
    return_value = (float)str_to_double(buf);
    
    return return_value;    
}
							

//-------------------------------------------------------------------------------------------------------------------
// @brief       字符串转浮点数 有效累计精度为小数点后九位
// @param       str             传入字符串 可带符号
// @return      double          转换后的数据          
// Sample usage:                double dat = str_to_double("-100.2");
//-------------------------------------------------------------------------------------------------------------------
double str_to_double (char *str)
{
    int sign = 0;                                                             // 标记符号 0-正数 1-负数
    double temp = 0.0;                                                          // 临时计算变量 整数部分
    double temp_point = 0.0;                                                    // 临时计算变量 小数部分
    double point_bit = 1;                                                       // 小数累计除数

    if('-' == *str)                                                             // 负数
    {
        sign = 1;                                                               // 标记负数
        str ++;
    }
    else if('+' == *str)                                                        // 如果第一个字符是正号
    {
        str ++;
    }

    // 提取整数部分
    while(('0' <= *str) && ('9' >= *str))                                       // 确定这是个数字
    {
        temp = temp * 10 + ((uint8_t)(*str) - 0x30);                              // 将数值提取出来
        str ++;
    }
    if('.' == *str)
    {
        str ++;
        while(('0' <= *str) && ('9' >= *str) && point_bit < 1000000000.0)       // 确认这是个数字 并且精度控制还没到九位
        {
            temp_point = temp_point * 10 + ((uint8_t)(*str) - 0x30);              // 提取小数部分数值
            point_bit *= 10;                                                    // 计算这部分小数的除数
            str ++;
        }
        temp_point /= point_bit;                                                // 计算小数
    }
    temp += temp_point;                                                         // 将数值拼合

    if(sign)
        return -temp;
    return temp;

}


//-------------------------------------------------------------------------------------------------------------------
// @brief       字符串转整形数字 数据范围是 [-32768,32767]
// @param       str             传入字符串 可带符号
// @return      int             转换后的数据          
// Sample usage:                int32_t dat = str_to_int("-100");
//-------------------------------------------------------------------------------------------------------------------
int32_t str_to_int (char *str)
{
    uint8_t sign = 0;                                                             // 标记符号 0-正数 1-负数
    int32_t temp = 0;                                                             // 临时计算变量

    if('-' == *str)                                                             // 如果第一个字符是负号
    {
        sign = 1;                                                               // 标记负数
        str ++;
    }
    else if('+' == *str)                                                        // 如果第一个字符是正号
    {
        str ++;
    }

    while(('0' <= *str) && ('9' >= *str))                                       // 确定这是个数字
    {
        temp = temp * 10 + ((uint8_t)(*str) - 0x30);                              // 计算数值
        str ++;
    }

    if(sign)
        return -temp;
    return temp;
}

//-------------------------------------------------------------------------------------------------------------------
// @brief		世界时间转换为北京时间	
// @param		*time           保存的时间
// @return		void           
// Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
static void utc_to_btc (gps_time_struct *time)
{
    uint8_t day_num;
    
    time->hour = time->hour + 8;
    // printf("hour:%d\n",time->hour);
	if(time->hour > 23)
	{
		time->hour -= 24;
		time->day += 1;

        if(2 == time->month)
        {
            day_num = 28;
            if((time->year % 4 == 0 && time->year % 100 != 0) || time->year % 400 == 0) // 判断是否为闰年 
            {
                day_num ++;                                                     // 闰月 2月为29天
            }
        }
        else
        {
            day_num = 31;                                                       // 1 3 5 7 8 10 12这些月份为31天
            if(4  == time->month || 6  == time->month || 9  == time->month || 11 == time->month )
            {
                day_num = 30;
            }
        }
        
        if(time->day > day_num)
        {
            time->day = 1;
            time->month ++;
            if(time->month > 12)
            {
                time->month -= 12;
                time->year ++;
            }
        }
	}
}

//-------------------------------------------------------------------------------------------------------------------
// @brief		RMC语句解析
// @param		*line	        接收到的语句信息		
// @param		*gps            保存解析后的数据
// @return		uint8_t           1：解析成功 0：数据有问题不能解析
// Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
// 
    // msg.data = "$GNRMC,055322.20,A,4004.73976661,N,11614.19695591,E,0.003,316.8,181017,6.7,W,A*39\n"; 
                 //    1    2       3     4         5     6          7     8    9     10   11 12  13
                
                //  #UNIHEADINGA,92,GPS,FINE,2364,198401000,0,0,18,12;SOL_COMPUTED,NARROW_INT,0.7713,351.2625,-1.2604,0.0000,0.8986,1.9005,"999",16,14,14,14,3,01,0,c0*62f42d91
                //    1       2   3    4    5     6      7 8  9 10   11                12  13  13 pitch  14heading
    static uint8_t gps_heading_parse (char *line, gps_info_struct *gps)
{
    
    // printf("-----------------heading!-------------------");
    uint8_t state;
    char *buf = line;

    uint8_t return_value = 0;


    state = buf[get_parameter_index(3, buf)];
    // gps->state = 0;
    // std::cout << "state: " << state << std::endl;

    // 解析 pitch 和 heading
    uint8_t pitch_index = get_parameter_index(11, buf); // pitch 的索引
    uint8_t heading_index = get_parameter_index(12, buf); // heading 的索引
    // std::cout << "gps_heading_parse, state: " << state << std::endl;
    if (state == 'F') {
        // RCLCPP_INFO(rclcpp::get_logger("Gps"), "State: FINE");
        if (pitch_index > 0) {
            gps->pitch = get_float_number(&buf[pitch_index]); // 解析 pitch
            // printf("gps->pitch: %lf\n", gps->pitch);
        } else {
            gps->pitch = 0.0;
            // printf("gps->pitch: Not available\n");
        }

        if (heading_index > 0) {
            gps->heading = get_float_number(&buf[heading_index]); // 解析 heading
            printf("gps->heading: %lf\n", gps->heading);
        } else {
            gps->heading = 0.0;
            printf("gps->heading: Not available\n");
        }
        return_value = 1;
    }

    return return_value;
}


//-------------------------------------------------------------------------------------------------------------------
// @brief		GGA语句解析
// @param		*line	        接收到的语句信息		
// @param		*gps            保存解析后的数据
// @return		uint8_t           1：解析成功 0：数据有问题不能解析
// Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
static uint8_t gps_gngga_parse (char *line, gps_info_struct *gps)
{
	uint8_t state;
	char *buf = line;
    uint8_t return_value = 0;

	state = buf[get_parameter_index(2, buf)];
    // printf("gps_gngga_parse_state:%c\n", state);
    if (state != ',')
    {
        // 解析经纬度
        double latitude = get_double_number(&buf[get_parameter_index(2, buf)]);
        double longitude = get_double_number(&buf[get_parameter_index(4, buf)]);

        gps->latitude_degree = (int)(latitude / 100);
        float lati_cent_tmp = latitude - gps->latitude_degree * 100;
        gps->latitude_cent = (int)lati_cent_tmp;
        float lati_second_tmp = (lati_cent_tmp - gps->latitude_cent) * 10000;
        gps->latitude_second = (int)lati_second_tmp;

        gps->longitude_degree = (int)(longitude / 100);
        float long_cent_tmp = longitude - gps->longitude_degree * 100;
        gps->longitude_cent = (int)long_cent_tmp;
        float long_second_tmp = (long_cent_tmp - gps->longitude_cent) * 10000;
        gps->longitude_second = (int)long_second_tmp;

        gps->latitude = gps->latitude_degree + (double)gps->latitude_cent / 60 + (double)gps->latitude_second / 600000;
        gps->longitude = gps->longitude_degree + (double)gps->longitude_cent / 60 + (double)gps->longitude_second / 600000;

        gps->qual = get_int_number(&buf[get_parameter_index(6, buf)]);
        // gps->height         = get_float_number(&buf[get_parameter_index(9, buf)]) + get_float_number(&buf[get_parameter_index(11, buf)]);  // 高度 = 海拔高度 + 地球椭球面相对大地水准面的高度 
        return_value = 1;
    }
    // qual:  0-无效,1-定位有效,2-差分定位有效,3-PPS模式,定位有效,4-RTK模式,5-浮动RTK,6-估算模式,7-手动输入模式,8-模拟器模式
    printf("gps->latitude:%lf, gps->longitude:%lf, gps->qual:%d, \n", gps->latitude, gps->longitude, gps->qual);
	// printf("gps->satellite_used:%d, gps->height:%lf\n", gps->satellite_used, gps->height);
    std::fill(std::begin(gps_tau1201_buffer1), std::end(gps_tau1201_buffer1), 0); // 清空缓冲区
	return return_value;
}


//--------------------------------------主函数----------------------------------------------------
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("serial_processor_node");
    rclcpp::Time::init();
    
    // 设定两个不同的频率：一个是10Hz用来解析GPS数据，另一个是1Hz用来发布数据
    rclcpp::Rate gps_parse_rate(10); // GPS数据解析频率为10Hz
    rclcpp::Rate publish_rate(1);    // 发布频率为1Hz
    double last_publish_time = 0.0; // 上次发布的时间
    SerialProcessor processor;

    rclcpp::Node nh;
    auto gps_pub = nh.advertise<std_msgs::msg::String>("gps/raw", 1000);
    bool has_valid_data = false; // 添加标志位，表示是否接收到有效数据

    while (rclcpp::ok()) {
        gps_data_parse(); // GPS 数据解析

        std_msgs::msg::String msg;

        // 检查GPS数据的有效性
        // if (gps_tau1201.latitude != 0 && gps_tau1201.longitude != 0 && gps_tau1201.heading != 0 ) {
        if (gps_tau1201.latitude != 0 && gps_tau1201.longitude != 0 ) {
            has_valid_data = true; // 设置标志位为 true

            std::ostringstream latitude_stream;
            latitude_stream << std::fixed << std::setprecision(8) << gps_tau1201.latitude;
        
            std::ostringstream longitude_stream;
            longitude_stream << std::fixed << std::setprecision(8) << gps_tau1201.longitude;
        
            std::ostringstream heading_stream;
            heading_stream << std::fixed << std::setprecision(8) << gps_tau1201.heading;
        
            std::ostringstream pitch_stream;
            pitch_stream << std::fixed << std::setprecision(8) << gps_tau1201.pitch;
            // qual:  0-无效,1-定位有效,2-差分定位有效,3-PPS模式,定位有效,4-RTK模式,5-浮动RTK,6-估算模式,7-手动输入模式,8-模拟器模式
            // heading: 航向 ( 0 到 360.0 ) 航向是主天线至从天线方向间基线向量逆时针方向与真北的夹角
            // pitch: 俯仰( ± 90 ) 
            json gps_data = {
                {"timestamp", rclcpp::Time::now().toSec()},
                {"latitude", latitude_stream.str()},
                {"longitude", longitude_stream.str()},
                {"qual", gps_tau1201.qual},
                {"heading", heading_stream.str()},
                {"pitch", pitch_stream.str()}
            };                 
            msg.data = gps_data.dump(4); // 格式化输出，缩进 4 空格
        } else if (has_valid_data) {
            // 如果之前接收到过有效数据，但当前数据无效，输出无效数据提示
            json error_data = {{"error", "Invalid GPS data: latitude or longitude is 0."}};
            msg.data = error_data.dump();
        } else {
            // 如果从未接收到有效数据，则不输出任何内容
            gps_parse_rate.sleep(); // 等待下次GPS数据解析周期
            rclcpp::spin_some(node);
            continue;
        }

        // 发布频率控制在1Hz
        if (rclcpp::Time::now().toSec() - last_publish_time >= 1.0) {
            // RCLCPP_INFO(rclcpp::get_logger("Gps"), "%s", msg.data.c_str());
            gps_pub.publish(msg);
            last_publish_time = rclcpp::Time::now().toSec();
        }

        gps_parse_rate.sleep();  // 持续解析GPS数据
        rclcpp::spin_some(node);
    }
}


// [INFO] [1725170394.934803]: Publishing: $GNGGA,055955.00,2140.83906234,N,11055.09795115,E,1,25,0.6,63.8438,M,-13.4265,M,,*63

// [ INFO] [1725170394.937920135]: Callback called with message: $GNGGA,055955.00,2140.83906234,N,11055.09795115,E,1,25,0.6,63.8438,M,-13.4265,M,,*63

// [ INFO] [1725170394.938032328]: Buffer content: $GNGGA,055955.00,2140.83906234,N,11055.09795115,E,1,25,0.6,63.8438,M,-13.4265,M,,*63
// ,2,5,26,,,,-0.289,-0.078,-0.025,,*3A

// [ INFO] [1725170394.938236305]: Received complete message: $GNGGA,055955.00,2140.83906234,N,11055.09795115,E,1,25,0.6,63.8438,M,-13.4265,M,,*63

// [INFO] [1725170395.035115]: Publishing: $KSXT,20240901055955.00,110.91829919,21.68065104,63.8438,243.17,41.67,263.04,0.317,,1,2,7,25,,,,-0.315,-0.038,0.031,,*1C

// [ INFO] [1725170395.038550078]: Callback called with message: $KSXT,20240901055955.00,110.91829919,21.68065104,63.8438,243.17,41.67,263.04,0.317,,1,2,7,25,,,,-0.315,-0.038,0.031,,*1C


// [ INFO] [1725170395.038728527]: Received complete message: $KSXT,20240901055955.00,110.91829919,21.68065104,63.8438,243.17,41.67,263.04,0.317,,1,2,7,25,,,,-0.315,-0.038,0.031,,*1C

// [ INFO] [1725280594.431053303]: Received complete message: $GNRMC,060221.00,A,2140.83989160,N,11055.10021596,E,0.025,232.7,020924,2.6,W,A,V*45

// gps->latitude:21.680663, gps->longitude:110.918332, gps->speed:0.046250, gps->direction:232.699997
// gps->time: 14:02:21
