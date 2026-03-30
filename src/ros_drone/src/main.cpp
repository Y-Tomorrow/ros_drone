#include <boost/bind.hpp>
#include <clocale>
#include <ros/ros.h>

#include <ros_drone/DataManager.h>

int main(int argc, char** argv) {
  setlocale(LC_ALL, "");
  ros::init(argc, argv, "capturer_node");
  ros::NodeHandle nh("~");
  ros::Duration(1.0).sleep();

  constexpr double kDefaultLoopRate = 50.0;
  double loop_rate = kDefaultLoopRate;
  nh.param("loop_rate", loop_rate, kDefaultLoopRate);
  ros::Rate rate(loop_rate);

  std::string ros_namespace{"none"};
  nh.param("ros_namespace", ros_namespace, std::string("none"));
  if (ros_namespace == "none") ros_namespace = "";
  const std::string& ns = ros_namespace;

  DataManager& data_manager = DataManager::getInstance();
  ros::AsyncSpinner spinner(3);
  spinner.start();

  nh.subscribe<mavros_msgs::State>(ns + "/mavros/state", 10,
                                     boost::bind(&DataManager::feed_state, &data_manager, _1));

  nh.subscribe<mavros_msgs::HomePosition>(ns + "/mavros/home_position/home", 10,
                                          boost::bind(&DataManager::feed_home_pos, &data_manager, _1));

  nh.subscribe<geometry_msgs::PoseStamped>(ns + "/mavros/local_position/pose", 10,
                                           boost::bind(&DataManager::feed_local_pos, &data_manager, _1));

  nh.subscribe<geometry_msgs::TwistStamped>(ns + "/mavros/local_position/velocity_body", 10,
                                            boost::bind(&DataManager::feed_vel_body, &data_manager, _1));

  nh.subscribe<geometry_msgs::TwistStamped>(ns + "/mavros/global_position/velocity_local", 10,
                                            boost::bind(&DataManager::feed_vel_enu, &data_manager, _1));

  nh.subscribe<sensor_msgs::Imu>(ns + "/mavros/imu/data", 10,
                                 boost::bind(&DataManager::feed_imu, &data_manager, _1));

  nh.subscribe<nav_msgs::Odometry>(ns + "/mavros/gimbal_pose", 10,
                                     boost::bind(&DataManager::feed_gimbal_pose, &data_manager, _1));

  while (!data_manager.is_connected()) {
    ROS_INFO_THROTTLE(5.0, "[%s] 等待飞控连接...", ns.c_str());
    ros::Duration(1.0).sleep();
  }
  ROS_INFO("DataManager connected");

  while (ros::ok()) {
    (void)DataManager::getInstance().get_flight_mode();
    rate.sleep();
  }

  ROS_INFO("capturer_node finished");
  return 0;
}
