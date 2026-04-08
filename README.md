### 启动仿真环境(带相机本机/uav0+目标机/uav1)
```bash
roslaunch ros_drone two_uav_depth_escape.launch target_offset_east_m:=5
```

### 目标机控制（W前 S后 A左 D右，Q下降 E上升）
```bash
roslaunch ros_drone uav_control.launch
```

下载yolo环境
```bash
pip3 install ultralytics torch torchvision --user
```


### 启动识别
```bash
roslaunch detector detector.launch
```

清理环境
```bash
pkill -x gzserver
pkill -x gzclient
pkill -x px4
```

### 先启动ROS master
```bash
source /opt/ros/noetic/setup.bash
source /home/yy/ros_drone/devel/setup.bash
roscore
```

### 启动带云台相机
```bash
cd /home/yy/PX4-Autopilot
make px4_sitl gazebo_iris_cgo3
```

### 启动仿真环境(带云台相机本机/uav0+目标机/uav1)
```bash
roslaunch ros_drone two_uav_cgo3.launch target_offset_east_m:=5
```

### 启动云台偏航（yaw）追踪
```bash
roslaunch ros_drone cgo3_yaw_track.launch
```

### 启动云台追踪（yaw + pitch，一起扫描）
```bash
roslaunch ros_drone cgo3_track.launch
```

### TargetSearcher 测试（`config/target_searcher.yaml` 由 launch 内 `rosparam load` 注入）
```bash
# 默认 session_mode=true：起飞后在初始点悬停，在同一终端输入命令（需有 stdin）：
#   WIDE 1 1        — 执行一段搜索（最长 scenario_duration_s 秒）
#   return 或 r     — 回到本次记录的初始悬停点
#   quit            — 退出节点
roslaunch ros_drone target_searcher_tester.launch uav_prefix:=/uav0 takeoff_altitude_m:=2.0

# 单次自动测试（无终端交互）：session_mode:=false，并用 lost_* 指定像素
roslaunch ros_drone target_searcher_tester.launch session_mode:=false lost_pixel_x:=1.0 lost_pixel_y:=1.0
```

### 可视化查看话题数值
```bash
rqt_plot /uav0/mavros/local_position/velocity_local/twist/linear/x \
         /uav0/mavros/local_position/velocity_local/twist/linear/y \
         /uav0/mavros/local_position/velocity_local/twist/linear/z
```