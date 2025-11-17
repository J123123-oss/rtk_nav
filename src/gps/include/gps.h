#include <cstdint>

// #ifndef _GPS_
// #define _GPS_
// // #include "mm32_device.h"
// // #include "hal_conf.h"
// // #include "headfile.h"
// typedef unsigned char  u8;
// void uart_interrupt_handler (void);

// extern char location[100];
// extern double my_x,my_y;
// extern double home_x,home_y;
// void send_string(char* ch,int num);
// double GPS_Convert(u8* a);

// typedef struct{
// 	double x;// m
// 	double y;// m
// }POSI_ST;
// #define USART2_MAX_RECV_LEN		1024	//最大接收缓存字节数
// u8 SkyTra_Cfg_Rate(u8 Frep);
// void show_gps(void);
// #define home_lo 118.7115405 //最好设定为场地最中央
// #define home_la 30.9083666  
// void GPS_INIT(void);
// void GET_MY_POSI(POSI_ST* posi);
// void GET_POSI(double longitude,double latitude,POSI_ST* posi);
//--------------------------------------------------------------------------------------------------


typedef struct{
	uint16_t year;  
	uint8_t  month; 
	uint8_t  day;
	uint8_t  hour;
	uint8_t  minute;
	uint8_t  second;
}gps_time_struct;

typedef struct{
    gps_time_struct    time;                                                    // 时间    
    uint8_t       state;                                                          // 有效状态  1：定位有效  0：定位无效    
    uint16_t      latitude_degree;	                                            // 度
	uint16_t      latitude_cent;		                                            // 分
	uint16_t      latitude_second;                                                // 秒
	uint16_t      longitude_degree;	                                            // 度
	uint16_t      longitude_cent;		                                            // 分
	uint16_t      longitude_second;                                               // 秒
    
    double      latitude;                                                       // 经度
    double      longitude;                                                      // 纬度
    
    int 	    ns;                                                             // 纬度半球 N（北半球）或 S（南半球）
    int 	    ew;                                                             // 经度半球 E（东经）或 W（西经）
    
	float 	    speed;                                                          // 速度（公里/每小时）
    float 	    direction;                                                      // 地面航向（000.0~359.9 度，以真北方为参考基准）
    
    // 下面两个个信息从GNGGA语句中获取
    int     qual;                         // 用于定位的GPS状态
	float 	height;                                                         // 高度   
	float   pitch;
	float   heading;
}gps_info_struct;

extern gps_info_struct  gps_tau1201;
// extern uint8_t            gps_tau1201_flag;


static uint8_t gps_gnrmc_parse(char *line, gps_info_struct *gps);
static uint8_t gps_gngga_parse(char *line, gps_info_struct *gps);
double str_to_double (char *str);
int32_t str_to_int (char *str);


// void        gps_uart_callback           (void);

// double      get_two_points_distance     (double lat1, double lng1, double lat2, double lng2);
// double      get_two_points_azimuth      (double lat1, double lon1, double lat2, double lon2);

// void        gps_data_parse              (void);

// void        gps_init                    (void);

// #endif