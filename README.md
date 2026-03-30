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

### 目标丢失反馈测试
只让云台动，不让机体动
```bash
rosrun ros_drone target_searcher_tester \
  _uav_prefix:=/uav0 _interactive:=true _scenario_duration_s:=8 \
  _arm_and_offboard:=false _send_velocity:=false
```
云台 + 机体都执行
```bash
rosrun ros_drone target_searcher_tester \
  _uav_prefix:=/uav0 _interactive:=true _scenario_duration_s:=8 \
  _arm_and_offboard:=true _send_velocity:=true
```