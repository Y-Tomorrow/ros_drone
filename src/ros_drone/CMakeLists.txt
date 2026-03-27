cmake_minimum_required(VERSION 3.0.2)
project(ros_drone)
find_package(catkin REQUIRED)
catkin_package()
catkin_install_python(PROGRAMS scripts/uav_control.py scripts/export_yolov8_onnx.py scripts/gimbal_yaw_track_target.py scripts/gimbal_track_target.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(PROGRAMS scripts/read_mission_distance.py scripts/two_uav_depth_escape.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
