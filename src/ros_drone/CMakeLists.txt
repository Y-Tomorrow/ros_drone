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

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

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

add_library(ros_drone_state STATIC src/DataManager.cpp)
add_dependencies(ros_drone_state ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ros_drone_state ${catkin_LIBRARIES})
if(TARGET Eigen3::Eigen)
  target_link_libraries(ros_drone_state Eigen3::Eigen)
endif()
target_compile_features(ros_drone_state PRIVATE cxx_std_14)

add_library(ros_drone_pursuit STATIC src/target_pursuit.cpp)
add_dependencies(ros_drone_pursuit ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ros_drone_pursuit ros_drone_state ${catkin_LIBRARIES})
if(TARGET Eigen3::Eigen)
  target_link_libraries(ros_drone_pursuit Eigen3::Eigen)
endif()
target_compile_features(ros_drone_pursuit PRIVATE cxx_std_14)

add_library(ros_drone_search STATIC src/target_searcher.cpp)
add_dependencies(ros_drone_search ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_include_directories(ros_drone_search PRIVATE ${YAML_CPP_INCLUDE_DIRS})
target_link_libraries(ros_drone_search ros_drone_state ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
if(TARGET Eigen3::Eigen)
  target_link_libraries(ros_drone_search Eigen3::Eigen)
endif()
target_compile_options(ros_drone_search PRIVATE ${YAML_CPP_CFLAGS_OTHER})
target_compile_features(ros_drone_search PRIVATE cxx_std_14)

add_executable(capturer_node src/main.cpp)
add_dependencies(capturer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(capturer_node ros_drone_state ${catkin_LIBRARIES})
if(TARGET Eigen3::Eigen)
  target_link_libraries(capturer_node Eigen3::Eigen)
endif()
target_compile_features(capturer_node PRIVATE cxx_std_14)

add_executable(target_searcher_tester_bin src/target_searcher_tester.cpp)
add_dependencies(target_searcher_tester_bin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(target_searcher_tester_bin ros_drone_search ros_drone_state ${catkin_LIBRARIES})
if(TARGET Eigen3::Eigen)
  target_link_libraries(target_searcher_tester_bin Eigen3::Eigen)
endif()
target_compile_features(target_searcher_tester_bin PRIVATE cxx_std_14)
set_target_properties(target_searcher_tester_bin PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ros_drone_state ros_drone_pursuit ros_drone_search
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs nav_msgs mavros_msgs
)

install(TARGETS capturer_node target_searcher_tester_bin ros_drone_state ros_drone_pursuit ros_drone_search
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/uav_control.py scripts/export_yolov8_onnx.py scripts/gimbal_yaw_track_target.py scripts/gimbal_track_target.py scripts/lost_target_demo.py scripts/target_searcher_tester
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(PROGRAMS scripts/read_mission_distance.py scripts/two_uav_depth_escape.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
