cmake_minimum_required(VERSION 3.0.2)
project(target_searcher)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  mavros_msgs
  ros_drone
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

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp roslib std_msgs geometry_msgs sensor_msgs nav_msgs mavros_msgs ros_drone
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

if(EIGEN3_INCLUDE_DIRS)
  include_directories(${EIGEN3_INCLUDE_DIRS})
endif()

add_library(${PROJECT_NAME}
  src/TargetSearcher.cpp
)
target_link_libraries(${PROJECT_NAME} ros_drone_datamanager ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

add_executable(target_searcher_pkg_tester_bin
  src/target_searcher_tester_bin.cpp
)
target_link_libraries(target_searcher_pkg_tester_bin ${PROJECT_NAME} ros_drone_datamanager ${catkin_LIBRARIES})
add_dependencies(target_searcher_pkg_tester_bin ${catkin_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME} target_searcher_pkg_tester_bin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
