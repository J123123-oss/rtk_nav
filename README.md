1
先单独启动launch运行cleaning_path_planner，生成轨迹，并保存。
2
launch运行MotorControlNode修改"~rtk_path_file"为cleaning_path目录下刚生成的文件
RTK可通过launch中录制的RTK消息解析调试，实地测试使用RTK实时消息解析


通过之前RTK采样数据转换为导航路线测试导航运行情况，流程可以跑通，偏角部分还有问题。
解析遥控器解码的数据并转换为运动控制,待测试。
电机控制订阅/current_speed,并执行
步骤：
启动launch文件后，发布start消息：
rostopic pub /rtk_nav/start std_msgs/String "data: 'start'"
订阅motor/state
motor/current_speed