// #include "GPS.h"
// #include "headfile.h"
// #include <stdio.h>
// #include <math.h>
// uint8 uart2_get_buffer;
// gps_info_struct gps_tau1201;
// uint8 gps_tau1201_buffer1[100];                                                 // 语句获取后将数据转移到此数组，然后开始解析语句内容
// uint8 gps_tau1201_buffer2[100];                                                 // 串口接收缓冲区
// uint8 gps_tau1201_num;                                                          // 当前接收字符数量
// uint8 gps_tau1201_flag;


// //-------------------------------------------------------------------------------------------------------------------
// // @brief       字符串转浮点数 有效累计精度为小数点后九位
// // @param       str             传入字符串 可带符号
// // @return      double          转换后的数据          
// // Sample usage:                double dat = str_to_double("-100.2");
// //-------------------------------------------------------------------------------------------------------------------
// double str_to_double (char *str)
// {
//     uint8 sign = 0;                                                             // 标记符号 0-正数 1-负数
//     double temp = 0.0;                                                          // 临时计算变量 整数部分
//     double temp_point = 0.0;                                                    // 临时计算变量 小数部分
//     double point_bit = 1;                                                       // 小数累计除数

//     if('-' == *str)                                                             // 负数
//     {
//         sign = 1;                                                               // 标记负数
//         str ++;
//     }
//     else if('+' == *str)                                                        // 如果第一个字符是正号
//     {
//         str ++;
//     }

//     // 提取整数部分
//     while(('0' <= *str) && ('9' >= *str))                                       // 确定这是个数字
//     {
//         temp = temp * 10 + ((uint8)(*str) - 0x30);                              // 将数值提取出来
//         str ++;
//     }
//     if('.' == *str)
//     {
//         str ++;
//         while(('0' <= *str) && ('9' >= *str) && point_bit < 1000000000.0)       // 确认这是个数字 并且精度控制还没到九位
//         {
//             temp_point = temp_point * 10 + ((uint8)(*str) - 0x30);              // 提取小数部分数值
//             point_bit *= 10;                                                    // 计算这部分小数的除数
//             str ++;
//         }
//         temp_point /= point_bit;                                                // 计算小数
//     }
//     temp += temp_point;                                                         // 将数值拼合

//     if(sign)
//         return -temp;
//     return temp;

// }

// //-------------------------------------------------------------------------------------------------------------------
// // @brief       字符串转整形数字 数据范围是 [-32768,32767]
// // @param       str             传入字符串 可带符号
// // @return      int             转换后的数据          
// // Sample usage:                int32 dat = str_to_int("-100");
// //-------------------------------------------------------------------------------------------------------------------
// int32 str_to_int (char *str)
// {
//     uint8 sign = 0;                                                             // 标记符号 0-正数 1-负数
//     int32 temp = 0;                                                             // 临时计算变量

//     if('-' == *str)                                                             // 如果第一个字符是负号
//     {
//         sign = 1;                                                               // 标记负数
//         str ++;
//     }
//     else if('+' == *str)                                                        // 如果第一个字符是正号
//     {
//         str ++;
//     }

//     while(('0' <= *str) && ('9' >= *str))                                       // 确定这是个数字
//     {
//         temp = temp * 10 + ((uint8)(*str) - 0x30);                              // 计算数值
//         str ++;
//     }

//     if(sign)
//         return -temp;
//     return temp;
// }
// //-------------------------------------------------------------------------------------------------------------------
// // @brief		GPS串口回调函数
// // @param		void			
// // @return		void            
// // Sample usage:				此函数需要在串口接收中断内进行调用
// //-------------------------------------------------------------------------------------------------------------------

// void uart_interrupt_handler (void)													// 这个函数在 isr.c 的 UART2_IRQHandler 中调用
// {
	
// 	uint8 dat;
    
//     uart_query(GPS_TAU1201_UART, &dat);
//     if('$' == dat || ('$' != gps_tau1201_buffer2[0]))                           // 帧头校验
//     {
//         gps_tau1201_num = 0;
//     }
//     gps_tau1201_buffer2[gps_tau1201_num ++] = dat;
    
//     if('\n' == dat)
//     {
//         // 收到一个语句
//         gps_tau1201_buffer2[gps_tau1201_num] = 0;                               // 在末尾添加\0
//         gps_tau1201_num ++;
//         // 拷贝数据到 gps_tau1201_buffer1
//         memcpy(gps_tau1201_buffer1, gps_tau1201_buffer2, gps_tau1201_num);
//         gps_tau1201_flag = 1;
//     }

// }

// //-------------------------------------------------------------------------------------------------------------------
// // @brief		获取指定,后面的索引
// // @param		num             第几个逗号
// // @param		*str            字符串           
// // @return		uint8           返回索引
// // Sample usage:				
// //-------------------------------------------------------------------------------------------------------------------
// static uint8 get_parameter_index (uint8 num, char *str)
// {
// 	uint8 i, j = 0;
//     char *temp;
//     uint8 len = 0, len1;
    
//     temp = strchr(str, '\n');
//     if(NULL != temp)
//     {
//         len = (uint32)temp - (uint32)str + 1;
//     }

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

// //-------------------------------------------------------------------------------------------------------------------
// // @brief		给定字符串第一个,之前的数据转换为int
// // @param		*s              字符串
// // @return		float           返回数值
// // Sample usage:				
// //-------------------------------------------------------------------------------------------------------------------
// static int get_int_number (char *s)
// {
// 	char buf[10];
// 	uint8 i;
// 	int return_value;
// 	i = get_parameter_index(1, s);
// 	i = i - 1;
// 	strncpy(buf, s, i);
// 	buf[i] = 0;
// 	return_value = str_to_int(buf);
// 	return return_value;
// }
												
// //-------------------------------------------------------------------------------------------------------------------
// // @brief		给定字符串第一个,之前的数据转换为float
// // @param		*s              字符串
// // @return		float           返回数值
// // Sample usage:				
// //-------------------------------------------------------------------------------------------------------------------
// static float get_float_number (char *s)
// {
//     uint8 i;
// 	char buf[10];
// 	float return_value;
    
// 	i=get_parameter_index(1, s);
// 	i = i - 1;
// 	strncpy(buf, s, i);
// 	buf[i] = 0;
// 	return_value = (float)str_to_double(buf);
// 	return return_value;	
// }
									
// //-------------------------------------------------------------------------------------------------------------------
// // @brief		给定字符串第一个,之前的数据转换为double	
// // @param		*s              字符串
// // @return		double          返回数值
// // Sample usage:				
// //-------------------------------------------------------------------------------------------------------------------
// static double get_double_number (char *s)
// {
//     uint8 i;
// 	char buf[10];
// 	double return_value;
    
// 	i = get_parameter_index(1, s);
// 	i = i - 1;
// 	strncpy(buf, s, i);
// 	buf[i] = 0;
// 	return_value = str_to_double(buf);
// 	return return_value;
// }

// //-------------------------------------------------------------------------------------------------------------------
// // @brief		世界时间转换为北京时间	
// // @param		*time           保存的时间
// // @return		void           
// // Sample usage:				
// //-------------------------------------------------------------------------------------------------------------------
// static void utc_to_btc (gps_time_struct *time)
// {
//     uint8 day_num;
    
//     time->hour = time->hour + 8;
// 	if(time->hour > 23)
// 	{
// 		time->hour -= 24;
// 		time->day += 1;

//         if(2 == time->month)
//         {
//             day_num = 28;
//             if((time->year % 4 == 0 && time->year % 100 != 0) || time->year % 400 == 0) // 判断是否为闰年 
//             {
//                 day_num ++;                                                     // 闰月 2月为29天
//             }
//         }
//         else
//         {
//             day_num = 31;                                                       // 1 3 5 7 8 10 12这些月份为31天
//             if(4  == time->month || 6  == time->month || 9  == time->month || 11 == time->month )
//             {
//                 day_num = 30;
//             }
//         }
        
//         if(time->day > day_num)
//         {
//             time->day = 1;
//             time->month ++;
//             if(time->month > 12)
//             {
//                 time->month -= 12;
//                 time->year ++;
//             }
//         }
// 	}
// }
// //-------------------------------------------------------------------------------------------------------------------
// // @brief		RMC语句解析
// // @param		*line	        接收到的语句信息		
// // @param		*gps            保存解析后的数据
// // @return		uint8           1：解析成功 0：数据有问题不能解析
// // Sample usage:				
// //-------------------------------------------------------------------------------------------------------------------
// static uint8 gps_gnrmc_parse (char *line, gps_info_struct *gps)
// {
// 	uint8 state, temp;
    
//     double  latitude;                                                           // 经度
//     double  longitude;                                                          // 纬度
    
// 	float lati_cent_tmp, lati_second_tmp;
// 	float long_cent_tmp, long_second_tmp;
// 	float speed_tmp;
// 	char *buf = line;
//     uint8 return_value = 0;

// 	state = buf[get_parameter_index(2, buf)];

//     gps->state = 0;
//     if (state == 'A')                                                           // 如果数据有效 则解析数据
//     {
//         return_value = 1;
//         gps->state = 1;
//         gps -> ns       = buf[get_parameter_index(4, buf)];
//         gps -> ew       = buf[get_parameter_index(6, buf)];

//         latitude   = get_double_number(&buf[get_parameter_index(3, buf)]);
//         longitude  = get_double_number(&buf[get_parameter_index(5, buf)]);

//         gps->latitude_degree  = (int)latitude / 100;                            // 纬度转换为度分秒
//         lati_cent_tmp         = (latitude - gps->latitude_degree * 100);
//         gps->latitude_cent    = (int)lati_cent_tmp;
//         lati_second_tmp       = (lati_cent_tmp - gps->latitude_cent) * 10000;
//         gps->latitude_second  = (int)lati_second_tmp;

//         gps->longitude_degree = (int)longitude / 100;	                        // 经度转换为度分秒
//         long_cent_tmp         = (longitude - gps->longitude_degree * 100);
//         gps->longitude_cent   = (int)long_cent_tmp;
//         long_second_tmp       = (long_cent_tmp - gps->longitude_cent) * 10000;
//         gps->longitude_second = (int)long_second_tmp;
        
//         gps->latitude = gps->latitude_degree + (double)gps->latitude_cent / 60 + (double)gps->latitude_second / 600000;
//         gps->longitude = gps->longitude_degree + (double)gps->longitude_cent / 60 + (double)gps->longitude_second / 600000;

//         speed_tmp      = get_float_number(&buf[get_parameter_index(7, buf)]);   // 速度(海里/小时)
//         gps->speed     = speed_tmp * 1.85f;                                     // 转换为公里/小时
//         gps->direction = get_float_number(&buf[get_parameter_index(8, buf)]);   // 角度			
//     }

//     // 在定位没有生效前也是有时间数据的，可以直接解析
//     gps->time.hour    = (buf[7] - '0') * 10 + (buf[8] - '0');		            // 时间
//     gps->time.minute  = (buf[9] - '0') * 10 + (buf[10] - '0');
//     gps->time.second  = (buf[11] - '0') * 10 + (buf[12] - '0');
//     temp = get_parameter_index(9, buf);
//     gps->time.day     = (buf[temp + 0] - '0') * 10 + (buf[temp + 1] - '0');     // 日期
//     gps->time.month   = (buf[temp + 2] - '0') * 10 + (buf[temp + 3] - '0');
//     gps->time.year    = (buf[temp + 4] - '0') * 10 + (buf[temp + 5] - '0') + 2000;

//     utc_to_btc(&gps->time);

// 	return return_value;
// }

// //-------------------------------------------------------------------------------------------------------------------
// // @brief		GGA语句解析
// // @param		*line	        接收到的语句信息		
// // @param		*gps            保存解析后的数据
// // @return		uint8           1：解析成功 0：数据有问题不能解析
// // Sample usage:				
// //-------------------------------------------------------------------------------------------------------------------
// static uint8 gps_gngga_parse (char *line, gps_info_struct *gps)
// {
// 	uint8 state;
// 	char *buf = line;
//     uint8 return_value = 0;

// 	state = buf[get_parameter_index(2, buf)];

//     if (state != ',')
//     {
//         gps->satellite_used = get_int_number(&buf[get_parameter_index(7, buf)]);
//         gps->height         = get_float_number(&buf[get_parameter_index(9, buf)]) + get_float_number(&buf[get_parameter_index(11, buf)]);  // 高度 = 海拔高度 + 地球椭球面相对大地水准面的高度 
//         return_value = 1;
//     }
	
// 	return return_value;
// }


// //-------------------------------------------------------------------------------------------------------------------
// // @brief		解析GPS数据
// // @param		void
// // @return		void
// // Sample usage:				gps_data_parse();
// //-------------------------------------------------------------------------------------------------------------------
// void gps_data_parse (void)
// {
// 	if(gps_tau1201_flag)
//     {
// 		gps_tau1201_flag = 0;

// 		if(0 == strncmp((char *)&gps_tau1201_buffer1[3], "RMC", 3))
// 		{
// 			gps_gnrmc_parse((char *)gps_tau1201_buffer1, &gps_tau1201);
// 		}
		
// 		else if(0 == strncmp((char *)&gps_tau1201_buffer1[3], "GGA", 3))
// 		{
// 			gps_gngga_parse((char *)gps_tau1201_buffer1, &gps_tau1201);
// 		}
// 	}
// }
// void send_string(char* ch,int num){
// 	int i = 0;
// 	while(i < num){
// 		uart_putchar(UART_1, ch[i]);
// 		i++;
// 	}
// }

// //根据经纬度解算笛卡尔坐标，存放于temp数组
// //高斯-克吕格投影
// void GeodeticToCartesian(double longitude,double latitude,double temp[])
// {
// 	double b;//纬度度数
// 	double L;//经度度数
// 	double L0;//中央经线度数
// 	double L1;//L - L0
// 	double t;//tanB
// 	double m;//ltanB
// 	double N;//卯酉圈曲率半径
// 	double q2;
// 	double X;// 高斯平面纵坐标
// 	double Y;// 高斯平面横坐标
// 	double s;// 赤道至纬度B的经线弧长
// 	double f; // 参考椭球体扁率
// 	double e2;// 椭球第一偏心率
// 	double a; // 参考椭球体长半轴
// 	//
// 	double a1;
// 	double a2;
// 	double a3;
// 	double a4;
// 	double 	 b1;
// 	double	 b2;
// 	double	 b3;
// 	double	 b4;
// 	double	c0;
// 	double	c1;
// 	double	c2;
// 	double	c3;
// 	//
// 	int Datum, prjno, zonewide;
// 	double IPI;
// 	//
// 	Datum = 84;// 投影基准面类型：北京54基准面为54，西安80基准面为80，WGS84基准面为84
// 	prjno = 0;// 投影带号
// 	zonewide = 3;
// 	IPI = 0.0174532925199433333333; // 3.1415926535898/180.0
// 	b = latitude; //纬度
// 	L = longitude;//经度
// 	if (zonewide == 6)
// 	{
// 		prjno = trunc(L / zonewide) + 1;
// 		L0 = prjno * zonewide - 3;
// 	}
// 	else
// 	{
// 		prjno = trunc((L - 1.5) / 3) + 1;
// 		L0 = prjno * 3;
// 	}
// 	if (Datum == 54)
// 	{
// 		a = 6378245;
// 		f = 1 / 298.3;
// 	}
// 	else if (Datum == 84)
// 	{
// 		a = 6378137;
// 		f = 1 / 298.257223563;
// 	}
// 	//
// 	L0 = L0 * IPI;
// 	L = L * IPI;
// 	b = b * IPI;
 
// 	e2 = 2 * f - f * f; // (a*a-b*b)/(a*a);
// 	L1 = L - L0;
// 	t = tan(b);
// 	m = L1 * cos(b);
// 	N = a / sqrt(1 - e2 * sin(b) * sin(b));
// 	q2 = e2 / (1 - e2) * cos(b) * cos(b);
// 	a1 = 1 + 3 / 4 * e2 + 45 / 64 * e2 * e2 + 175 / 256 * e2 * e2 * e2 + 11025 /
// 		16384 * e2 * e2 * e2 * e2 + 43659 / 65536 * e2 * e2 * e2 * e2 * e2;
// 	a2 = 3 / 4 * e2 + 15 / 16 * e2 * e2 + 525 / 512 * e2 * e2 * e2 + 2205 /
// 		2048 * e2 * e2 * e2 * e2 + 72765 / 65536 * e2 * e2 * e2 * e2 * e2;
// 	a3 = 15 / 64 * e2 * e2 + 105 / 256 * e2 * e2 * e2 + 2205 / 4096 * e2 * e2 *
// 		e2 * e2 + 10359 / 16384 * e2 * e2 * e2 * e2 * e2;
// 	a4 = 35 / 512 * e2 * e2 * e2 + 315 / 2048 * e2 * e2 * e2 * e2 + 31185 /
// 		13072 * e2 * e2 * e2 * e2 * e2;
// 	b1 = a1 * a * (1 - e2);
// 	b2 = -1 / 2 * a2 * a * (1 - e2);
// 	b3 = 1 / 4 * a3 * a * (1 - e2);
// 	b4 = -1 / 6 * a4 * a * (1 - e2);
// 	c0 = b1;
// 	c1 = 2 * b2 + 4 * b3 + 6 * b4;
// 	c2 = -(8 * b3 + 32 * b4);
// 	c3 = 32 * b4;
// 	s = c0 * b + cos(b) * (c1 * sin(b) + c2 * sin(b) * sin(b) * sin(b) + c3 * sin
// 		(b) * sin(b) * sin(b) * sin(b) * sin(b));
// 	X = s + 1 / 2 * N * t * m * m + 1 / 24 * (5 - t * t + 9 * q2 + 4 * q2 * q2)
// 		* N * t * m * m * m * m + 1 / 720 * (61 - 58 * t * t + t * t * t * t)
// 		* N * t * m * m * m * m * m * m;
// 	Y = N * m + 1 / 6 * (1 - t * t + q2) * N * m * m * m + 1 / 120 *
// 		(5 - 18 * t * t + t * t * t * t - 14 * q2 - 58 * q2 * t * t)
// 		* N * m * m * m * m * m;
// 	Y = Y + 1000000 * prjno + 500000;
// 	temp[0] = X;
// 	temp[1] = Y;
// }

// double home_x,home_y;
// double my_x,my_y;

// //根据经纬度获取以设定的home点为原点的笛卡尔坐标
// void GET_POSI(double longitude,double latitude,POSI_ST* posi){
// 	double temp[2];
// 	GeodeticToCartesian(longitude,latitude,temp);//解算
// 	posi->x = temp[1] - home_x;//计算相对home点的坐标
// 	posi->y = temp[0] - home_y;//计算相对home点的坐标
// }

// //获取车身笛卡尔坐标
// void GET_MY_POSI(POSI_ST* posi){
// 	GET_POSI(gps_tau1201.longitude,gps_tau1201.latitude,posi);
// }

// void show_gps(void){
// 	char x[20],y[20];
// 	char JIN[20],WEI[20];
// 	double longitude,latitude;
// 	POSI_ST posi;
// 	longitude = gps_tau1201.longitude;
// 	latitude = gps_tau1201.latitude;
// 	GET_POSI(longitude,latitude,&posi);
// 	sprintf(x,"%.2lf",posi.x);
// 	sprintf(y,"%.2lf",posi.y);
// 	sprintf(JIN,"%lf",longitude);
// 	sprintf(WEI,"%lf",latitude);
// 	lcd_showstr(5,0,(char*)x);
// 	lcd_showstr(5,1,(char*)y);
// 	lcd_showstr(10,3,(char*)JIN);
// 	lcd_showstr(10,5,(char*)WEI);
// }

// void GPS_INIT(void){
// 	const uint8 set_rate[]      = {0xF1, 0xD9, 0x06, 0x42, 0x14, 0x00, 0x00, 0x0A, 0x05, 0x00, 0x64, 0x00, 0x00, 0x00, 0x60, 0xEA, 0x00, 0x00, 0xD0, 0x07, 0x00, 0x00, 0xC8, 0x00, 0x00, 0x00, 0xB8, 0xED};
//     const uint8 open_gga[]      = {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x00, 0x01, 0xFB, 0x10};
//     const uint8 open_rmc[]      = {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x05, 0x01, 0x00, 0x1A};
    
//     const uint8 close_gll[]     = {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x01, 0x00, 0xFB, 0x11};
//     const uint8 close_gsa[]     = {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x02, 0x00, 0xFC, 0x13};
//     const uint8 close_grs[]     = {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15};
//     const uint8 close_gsv[]     = {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x04, 0x00, 0xFE, 0x17};
//     const uint8 close_vtg[]     = {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x06, 0x00, 0x00, 0x1B};
//     const uint8 close_zda[]     = {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x07, 0x00, 0x01, 0x1D};
//     const uint8 close_gst[]     = {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x08, 0x00, 0x02, 0x1F};
//     const uint8 close_txt[]     = {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x40, 0x00, 0x3A, 0x8F};
//     const uint8 close_txt_ant[] = {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x20, 0x00, 0x1A, 0x4F};
    
//     systick_delay_ms(500);                                                       // 等待GPS启动后开始初始化
//     uart_init(GPS_TAU1201_UART, 115200, GPS_TAU1201_RX, GPS_TAU1201_TX);

//     uart_putbuff(GPS_TAU1201_UART, (uint8 *)set_rate, sizeof(set_rate));        // 设置GPS更新速率为10hz 如果不调用此语句则默认为1hz
//     systick_delay_ms(200);
    
//     uart_putbuff(GPS_TAU1201_UART, (uint8 *)open_rmc, sizeof(open_rmc));        // 开启rmc语句
//     systick_delay_ms(50);
//     uart_putbuff(GPS_TAU1201_UART, (uint8 *)open_gga, sizeof(open_gga));        // 开启gga语句
//     systick_delay_ms(50);
//     uart_putbuff(GPS_TAU1201_UART, (uint8 *)close_gll, sizeof(close_gll));
//     systick_delay_ms(50);
//     uart_putbuff(GPS_TAU1201_UART, (uint8 *)close_gsa, sizeof(close_gsa));
//     systick_delay_ms(50);
//     uart_putbuff(GPS_TAU1201_UART, (uint8 *)close_grs, sizeof(close_grs));
//     systick_delay_ms(50);
//     uart_putbuff(GPS_TAU1201_UART, (uint8 *)close_gsv, sizeof(close_gsv));
//     systick_delay_ms(50);
//     uart_putbuff(GPS_TAU1201_UART, (uint8 *)close_vtg, sizeof(close_vtg));
//     systick_delay_ms(50);
//     uart_putbuff(GPS_TAU1201_UART, (uint8 *)close_zda, sizeof(close_zda));
//     systick_delay_ms(50);
//     uart_putbuff(GPS_TAU1201_UART, (uint8 *)close_gst, sizeof(close_gst));
//     systick_delay_ms(50);
//     uart_putbuff(GPS_TAU1201_UART, (uint8 *)close_txt, sizeof(close_txt));
//     systick_delay_ms(50);
//     uart_putbuff(GPS_TAU1201_UART, (uint8 *)close_txt_ant, sizeof(close_txt_ant));
//     systick_delay_ms(50);

//     uart_rx_irq(GPS_TAU1201_UART, 1);
	
// 	double temp[2];
// 	GeodeticToCartesian(home_lo,home_la,temp);
// 	home_x = temp[1];
// 	home_y = temp[0];
// }
