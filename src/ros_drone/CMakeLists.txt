cmake_minimum_required(VERSION 3.0.2)
project(ros_drone)

find_package(catkin REQUIRED COMPONENTS
  roslib
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  mavros_msgs
)

find_package(Eigen3 QUIET)
if(NOT Eigen3_FOUND)
  find_path(EIGEN3_INCLUDE_DIR NAMES Eigen/Core
    PATHS /usr/include/eigen3 /usr/local/include/eigen3 ${CMAKE_INSTALL_PREFIX}/include/eigen3)
  if(EIGEN3_INCLUDE_DIR)
    set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
    set(Eigen3_FOUND TRUE)
  endif()
endif()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

if(EIGEN3_INCLUDE_DIRS)
  include_directories(${EIGEN3_INCLUDE_DIRS})
endif()

#
# 搜索逻辑（TargetSearcher）已移除，后续将从头实现。
#

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs nav_msgs mavros_msgs
)

add_executable(capturer_node
  src/main.cpp
  src/DataManager.cpp
)
target_link_libraries(capturer_node ${catkin_LIBRARIES})
add_dependencies(capturer_node ${catkin_EXPORTED_TARGETS})

add_executable(target_searcher_tester_bin
  src/target_searcher_tester_bin.cpp
  src/DataManager.cpp
  src/TargetSearcher.cpp
)
target_link_libraries(target_searcher_tester_bin ${catkin_LIBRARIES})
add_dependencies(target_searcher_tester_bin ${catkin_EXPORTED_TARGETS})

install(TARGETS
  capturer_node
  target_searcher_tester_bin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/uav_control.py scripts/export_yolov8_onnx.py scripts/gimbal_yaw_track_target.py scripts/gimbal_track_target.py scripts/lost_target_demo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(PROGRAMS scripts/read_mission_distance.py scripts/two_uav_depth_escape.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
