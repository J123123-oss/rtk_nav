#include <ros/ros.h>
#include <geographic_msgs/GeoPoint.h>
#include <geodesy/utm.h>

class DistanceCalculator
{
public:
    DistanceCalculator()
    {
        // 示例坐标

        // BD1:gps->latitude:30.32101833, gps->longitude:120.07105000, gps->qual:4, gps->direction:0.00000000"

        // BD2:gps->latitude:30.32098833, gps->longitude:120.07123333, gps->qual:4, gps->direction:0.00000000"

        // BD3:gps->latitude:30.32086333, gps->longitude:120.07125000, gps->qual:4, gps->direction:0.00000000

        geographic_msgs::GeoPoint point1, point2;
        // point1.latitude = 30.32101833; // BD1
        // point1.longitude = 120.07105000;
        point1.latitude = 30.32098833;  // BD2
        point1.longitude = 120.07123333;
        point2.latitude = 30.32086333;  // BD2
        point2.longitude = 120.07125000;
        // 计算距离
        double utm_dist = calculateUTMDistance(point1, point2);
        double haversine_dist = calculateHaversineDistance(point1, point2);
        
        ROS_INFO("UTM Distance: %.2f meters", utm_dist);
        ROS_INFO("Haversine Distance: %.2f meters", haversine_dist);
    }
    
    double calculateUTMDistance(const geographic_msgs::GeoPoint& p1, 
                              const geographic_msgs::GeoPoint& p2)
    {
        geodesy::UTMPoint utm1(p1);
        geodesy::UTMPoint utm2(p2);
        
        // 确保在同一UTM区域
        if(utm1.zone != utm2.zone || utm1.band != utm2.band)
        {
            ROS_WARN("Points are in different UTM zones!");
        }
        
        double dx = utm1.easting - utm2.easting;
        double dy = utm1.northing - utm2.northing;
        return sqrt(dx*dx + dy*dy);
    }
    
    double calculateHaversineDistance(const geographic_msgs::GeoPoint& p1,
                                    const geographic_msgs::GeoPoint& p2)
    {
        const double R = 6371000.0; // 地球半径(米)
        
        double lat1 = p1.latitude * M_PI/180.0;
        double lon1 = p1.longitude * M_PI/180.0;
        double lat2 = p2.latitude * M_PI/180.0;
        double lon2 = p2.longitude * M_PI/180.0;
        
        double dlat = lat2 - lat1;
        double dlon = lon2 - lon1;
        
        double a = sin(dlat/2) * sin(dlat/2) +
                  cos(lat1) * cos(lat2) *
                  sin(dlon/2) * sin(dlon/2);
        double c = 2 * atan2(sqrt(a), sqrt(1-a));
        
        return R * c;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "distance_calculator");
    DistanceCalculator dc;
    ros::spin();
    return 0;
}

// rosrun gps dis_cal 测量18.5m
// [ INFO] [1745918435.109672349]: UTM Distance: 17.95 meters
// [ INFO] [1745918435.109993663]: Haversine Distance: 17.91 meters
// rosrun gps dis_cal 测量13.9m
// [ INFO] [1745919000.732980700]: UTM Distance: 13.96 meters
// [ INFO] [1745919000.733050346]: Haversine Distance: 13.99 meters