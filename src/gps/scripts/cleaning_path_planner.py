import math
import matplotlib
import matplotlib.pyplot as plt
import utm
import rospy
import datetime
from matplotlib.patches import FancyArrowPatch
from std_msgs.msg import String
import os

# 设置中文字体
matplotlib.rcParams["font.family"] = ["WenQuanYi Micro Hei", "Heiti TC", "SimHei"]
matplotlib.rcParams['axes.unicode_minus'] = False

def degrees_to_radians(degrees):
    return degrees * math.pi / 180.0

def radians_to_degrees(radians):
    return radians * 180.0 / math.pi

def get_utm_coords(lat, lon):
    """将经纬度转换为UTM坐标（东向、北向、区号、字母）"""
    return utm.from_latlon(lat, lon)

def get_latlon_from_utm(easting, northing, zone_number, zone_letter):
    """将UTM坐标转换为经纬度"""
    return utm.to_latlon(easting, northing, zone_number, zone_letter)

def calculate_heading_angles(path_latlon):
    """
    计算轨迹中每个点的航向角（单位：度，0°为北，顺时针递增）
    
    参数:
        path_latlon: 轨迹点列表，格式为[(lon1, lat1), (lon2, lat2), ...]
    
    返回:
        headings: 航向角列表，长度与path_latlon相同
    """
    headings = []
    if len(path_latlon) <= 1:
        # 若轨迹点不足2个，航向角默认为0
        return [0.0] * len(path_latlon)
    
    for i in range(len(path_latlon)):
        if i == len(path_latlon) - 1:
            # 最后一个点沿用前一个点的航向角
            headings.append(headings[-1])
        else:
            # 提取当前点和下一个点的经纬度（弧度）
            lon1, lat1 = path_latlon[i]
            lon2, lat2 = path_latlon[i+1]
            
            lat1_rad = math.radians(lat1)
            lat2_rad = math.radians(lat2)
            delta_lon_rad = math.radians(lon2 - lon1)
            
            # 计算方位角（基于球面三角公式）
            y = math.sin(delta_lon_rad) * math.cos(lat2_rad)
            x = math.cos(lat1_rad) * math.sin(lat2_rad) - \
                math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon_rad)
            heading_rad = math.atan2(y, x)  # 弧度，范围(-π, π)
            
            # 转换为度并归一化到0°~360°
            heading_deg = math.degrees(heading_rad)
            heading_deg = (heading_deg + 360) % 360  # 确保为正角度
            headings.append(heading_deg)
    
    return headings

def generate_cleaning_path(corner1, corner2, param):
    """生成矩形区域的清扫路径（支持经度/纬度方向分别设置边缘距离）"""
    lon1, lat1 = corner1
    lon2, lat2 = corner2
    
    # 从参数中提取配置
    interval = param['interval']
    edge_lon = param['edge_distance_lon']  # 经度方向（东西）边缘距离
    edge_lat = param['edge_distance_lat']  # 纬度方向（南北）边缘距离
    
    # 转换原始角点到UTM坐标
    e1, n1, zone_num, zone_letter = get_utm_coords(lat1, lon1)
    e2, n2, _, _ = get_utm_coords(lat2, lon2)
    
    # 原始UTM边界（平面坐标）
    min_e = min(e1, e2)
    max_e = max(e1, e2)
    min_n = min(n1, n2)
    max_n = max(n1, n2)
    
    # 计算内部后的UTM边界
    inner_min_e = min_e + edge_lon
    inner_max_e = max_e - edge_lon
    inner_min_n = min_n + edge_lat
    inner_max_n = max_n - edge_lat
    
    # 安全检查
    if inner_min_e >= inner_max_e - 0.1 or inner_min_n >= inner_max_n - 0.1:
        raise ValueError(
            f"边缘距离过大，有效区域为空！\n"
            f"经度方向内部后宽度: {inner_max_e - inner_min_e:.2f}m，"
            f"纬度方向内部后高度: {inner_max_n - inner_min_n:.2f}m"
        )
    
    # 原始和内部矩形角点（UTM坐标）
    original_corners_utm = [
        (min_e, max_n), (max_e, max_n), (max_e, min_n), (min_e, min_n)
    ]
    inner_corners_utm = [
        (inner_min_e, inner_max_n), (inner_max_e, inner_max_n), 
        (inner_max_e, inner_min_n), (inner_min_e, inner_min_n)
    ]
    
    # 计算内部矩形尺寸
    width_m = inner_max_e - inner_min_e
    height_m = inner_max_n - inner_min_n
    
    # 生成清扫路径
    path_latlon = []  # 经纬度路径点
    path_utm = []     # UTM平面坐标路径点
    
    if width_m >= height_m:
        # 沿高度方向来回清扫
        num_strips = max(1, int(height_m / interval) + 1)
        for i in range(num_strips):
            current_n = inner_max_n - (inner_max_n - inner_min_n) * (i / (num_strips - 1) if num_strips > 1 else 0)
            
            if i % 2 == 0:
                start_e, start_n = inner_min_e, current_n
                end_e, end_n = inner_max_e, current_n
            else:
                start_e, start_n = inner_max_e, current_n
                end_e, end_n = inner_min_e, current_n
            
            # 记录当前行的起点
            lat_start, lon_start = get_latlon_from_utm(start_e, start_n, zone_num, zone_letter)
            path_latlon.append((lon_start, lat_start))
            path_utm.append((start_e, start_n))
            
            # 记录当前行的终点（最后一行也需要记录终点）
            lat_end, lon_end = get_latlon_from_utm(end_e, end_n, zone_num, zone_letter)
            path_latlon.append((lon_end, lat_end))
            path_utm.append((end_e, end_n))
    else:
        # 沿宽度方向来回清扫
        num_strips = max(1, int(width_m / interval) + 1)
        for i in range(num_strips):
            current_e = inner_min_e + (inner_max_e - inner_min_e) * (i / (num_strips - 1) if num_strips > 1 else 0)
            
            if i % 2 == 0:
                start_e, start_n = current_e, inner_max_n
                end_e, end_n = current_e, inner_min_n
            else:
                start_e, start_n = current_e, inner_min_n
                end_e, end_n = current_e, inner_max_n

            # 记录当前行的起点
            lat_start, lon_start = get_latlon_from_utm(start_e, start_n, zone_num, zone_letter)
            path_latlon.append((lon_start, lat_start))
            path_utm.append((start_e, start_n))
            
            # 记录当前行的终点（最后一行也需要记录终点）
            lat_end, lon_end = get_latlon_from_utm(end_e, end_n, zone_num, zone_letter)
            path_latlon.append((lon_end, lat_end))
            path_utm.append((end_e, end_n))
    
    return path_latlon, path_utm, original_corners_utm, inner_corners_utm, (zone_num, zone_letter)

def add_direction_arrows(ax, path_utm, arrow_interval=5):
    """在路径上添加方向箭头"""
    # 箭头间隔：每隔arrow_interval个点添加一个箭头
    for i in range(0, len(path_utm) - arrow_interval, arrow_interval):
        # 起点
        x1, y1 = path_utm[i]
        # 终点（箭头指向的位置）
        x2, y2 = path_utm[i + arrow_interval]
        
        # 计算箭头长度（根据实际距离调整箭头大小）
        dx = x2 - x1
        dy = y2 - y1
        length = math.sqrt(dx**2 + dy**2)
        
        # 添加箭头
        arrow = FancyArrowPatch(
            (x1, y1), (x2, y2),
            arrowstyle='->',  # 箭头样式
            mutation_scale=15,  # 箭头大小
            color='blue',       # 箭头颜色
            linewidth=1.5,      # 箭杆粗细
            alpha=0.7           # 透明度
        )
        ax.add_patch(arrow)

def get_ros_param(name, default):
    """获取ROS参数，若不存在则使用默认值"""
    if rospy.has_param(name):
        return rospy.get_param(name)
    else:
        rospy.logwarn(f"未找到参数 {name}，使用默认值: {default}")
        return default

def main():
    # 初始化ROS节点
    rclpy.init()
    node = rclpy.create_node('cleaning_path_planner', anonymous=True)
    
    # 从rosparam获取参数（封装rospy.get_param,支持通过launch文件或命令行设置）
    param = {
        'corner1': (
            get_ros_param('~corner1/lon', 120.07108186), #120.0709835,120.07065199383332
            get_ros_param('~corner1/lat', 30.32157528) #30.32157528,30.321056776833334
        ),
        'corner2': (
            get_ros_param('~corner2/lon', 120.07104303), #120.07104303,120.070640642
            get_ros_param('~corner2/lat', 30.32149357)#30.32158572,30.321051885333336
        ),
        'interval': get_ros_param('~interval', 0.5),
        'edge_distance_lon': get_ros_param('~edge_distance_lon', 0.1),
        'edge_distance_lat': get_ros_param('~edge_distance_lat', 0.1)
    }
    
    # 获取当前时间戳（用于文件名）
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    
    try:
        # 生成清扫路径
        path_latlon, path_utm, original_corners_utm, inner_corners_utm, utm_zone = generate_cleaning_path(
            param['corner1'], param['corner2'], param)
        zone_num, zone_letter = utm_zone
        
        # 打印参数信息
        rospy.loginfo("参数配置:")
        rospy.loginfo(f"  矩形对角点1: {param['corner1']}")
        rospy.loginfo(f"  矩形对角点2: {param['corner2']}")
        rospy.loginfo(f"  路径间隔: {param['interval']}米")
        rospy.loginfo(f"  经度方向边缘距离: {param['edge_distance_lon']}米")
        rospy.loginfo(f"  纬度方向边缘距离: {param['edge_distance_lat']}米")
        rospy.loginfo(f"\n生成的路径点数量: {len(path_latlon)}")
        save_dir = os.path.expanduser("/home/ubuntu/rtk_nav/src/gps/cleaning_path/")
        os.makedirs(save_dir, exist_ok=True)  # 确保目录存在
        points_filename = os.path.join(save_dir, f"cleaning_path_{timestamp}.txt")
        headings = calculate_heading_angles(path_latlon)

        # 保存路径点时同时写入航向角
        with open(points_filename, "w", encoding="utf-8") as f:
            f.write("序号,经度,纬度,航向角(度)\n")  # 新增航向角列
            for i in range(len(path_latlon)):
                lon, lat = path_latlon[i]
                heading = headings[i]
                f.write(f"{i+1},{lon:.8f},{lat:.8f},{heading:.2f}\n")  # 保留2位小数
        rospy.loginfo(f"所有路径点及航向角已保存到 {points_filename} 文件")
        
        # 绘制清扫路径
        fig, ax = plt.subplots(figsize=(10, 8))
        
        # 绘制原始矩形边界
        orig_e = [corner[0] for corner in original_corners_utm] + [original_corners_utm[0][0]]
        orig_n = [corner[1] for corner in original_corners_utm] + [original_corners_utm[0][1]]
        ax.plot(orig_e, orig_n, 'b-', label='原始区域边界')
        
        # 绘制内部矩形边界
        inner_e = [corner[0] for corner in inner_corners_utm] + [inner_corners_utm[0][0]]
        inner_n = [corner[1] for corner in inner_corners_utm] + [inner_corners_utm[0][1]]
        ax.plot(inner_e, inner_n, 'g--', label='内部清扫边界')
        
        # 绘制清扫路径
        path_e = [p[0] for p in path_utm]
        path_n = [p[1] for p in path_utm]
        ax.plot(path_e, path_n, 'r-', linewidth=1, label='清扫路径')
        
        # 添加方向箭头
        add_direction_arrows(ax, path_utm, arrow_interval=1)
        
        # 标记起点和终点
        ax.scatter(path_e[0], path_n[0], c='green', s=100, marker='o', label='起点')
        ax.scatter(path_e[-1], path_n[-1], c='purple', s=100, marker='x', label='终点')
        
        ax.set_xlabel(f'UTM东向 (米) - 区域 {zone_num}{zone_letter}')
        ax.set_ylabel(f'UTM北向 (米) - 区域 {zone_num}{zone_letter}')
        ax.set_title(
            f'机器人清扫路径规划\n'
            f'经度边缘距离: {param["edge_distance_lon"]}m, 纬度边缘距离: {param["edge_distance_lat"]}m'
        )
        ax.legend()
        ax.grid(True)
        ax.axis('equal')
        plt.tight_layout()
        
        # 保存带时间戳的图片
        img_filename = os.path.join(save_dir, f'cleaning_path_{timestamp}.png')
        plt.savefig(img_filename, dpi=300)
        rospy.loginfo(f"路径图已保存到 {img_filename}")
        
        # 若不是在headless模式，显示图像
        if not rospy.get_param('~headless', False):
            plt.show()
        
        # 保持节点运行
        rospy.spin()
        
    except ValueError as e:
        rospy.logerr(f"错误: {e}")
    except Exception as e:
        rospy.logerr(f"发生异常: {str(e)}")

if __name__ == "__main__":
    main()