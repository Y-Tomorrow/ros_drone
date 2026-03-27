cmake_minimum_required(VERSION 3.0.2)
project(detector)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  image_transport
  std_msgs
)

find_package(OpenCV REQUIRED COMPONENTS core imgproc dnn imgcodecs)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs cv_bridge image_transport std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(yolov8_detector_node src/yolov8_detector_node.cpp)
target_link_libraries(yolov8_detector_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
target_compile_features(yolov8_detector_node PRIVATE cxx_std_14)

install(TARGETS yolov8_detector_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
catkin_install_python(PROGRAMS scripts/yolov8_detector_pt.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
