from geographiclib.geodesic import Geodesic

def calculate_distance_and_bearing(origin, destination):
    # 原点和目标点的经纬度
    geod = Geodesic.WGS84
    result = geod.Inverse(origin[0], origin[1], destination[0], destination[1])

    # 获取距离和方位角
    distance = result['s12']   # 转换为M
    bearing = result['azi1']  # 初始方位角、方位角是从北方向顺时针测量的角度。

    return distance, bearing


# 示例经纬度
# origin = (21.68007020091791, 110.91824768309247)  # 原点（纬度lat, 经度lon）
# destination = (21.680078262699684, 110.91905925768252)  # 目标点（纬度, 经度）

origin = 30.32101833, 120.07105000  #BD1
destination =30.32098833, 120.07123333 #BD2

# destination = 30.32086333, 120.07125000 #BD3
# 计算距离与方位角
distance, bearing = calculate_distance_and_bearing(origin, destination)

# 输出结果
print(f"距离: {distance:.6f}M")
print(f"方位角: {bearing:.6f}°")


# // 1
# // gps->latitude:21.680427, gps->longitude:110.918648, gps->speed:0.003700, gps->direction:19.200001
# // gps->time: 17:51:05
# // 2
# // gps->latitude:21.680228, gps->longitude:110.918682, gps->speed:0.590150, gps->direction:183.800003
# // gps->time: 17:56:09
# 距离: 0.022314 公里
# 方位角: 170.926928 度


# 第二处参考点
# gps1.latitude = 21.68007020091791;//随机选取：主教左下角
#     gps1.longitude = 110.91824768309247;
#
#
# posi_now.longitude = 110.91905925768252;//地图选点：主教右下角
# 	posi_now.latitude = 21.680078262699684;
# 距离: 83.996350M
# 方位角: 89.390943°



# origin = 30.32101833, 120.07105000  #BD1
# destination =30.32098833, 120.07123333 #BD2

# destination = 30.32086333, 120.07125000 #BD3
# BD1>BD2距离: 17.942538M

# BD2>BD3距离: 13.949669M