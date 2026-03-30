/**
 * @file DataManager.h
 * @brief 无人机状态数据单例：订阅回调写入、线程安全读取
 */
#pragma once

#include <ros/ros.h>
#include <deque>
#include <mutex>
#include <string>

#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

class GimbalPoseData_t {
 public:
  nav_msgs::Odometry msg;
  Eigen::Quaternionf q;
  ros::Time rcv_stamp;

  GimbalPoseData_t();
  void feed(const nav_msgs::Odometry::ConstPtr& pMsg);
};

class StateData_t {
 public:
  mavros_msgs::State msg;
  bool connected;
  bool armed;
  std::string mode;
  ros::Time rcv_stamp;

  StateData_t();
  void feed(const mavros_msgs::State::ConstPtr& pMsg);
};

class HomePosData_t {
 public:
  mavros_msgs::HomePosition msg;
  Eigen::Vector3f p;
  Eigen::Quaternionf q;
  float yaw;
  ros::Time rcv_stamp;

  HomePosData_t();
  void feed(const mavros_msgs::HomePosition::ConstPtr& pMsg);
};

class LocalPosData_t {
 public:
  geometry_msgs::PoseStamped msg;
  Eigen::Vector3f p;
  Eigen::Quaternionf q;
  float yaw;
  ros::Time rcv_stamp;

  LocalPosData_t();
  void feed(const geometry_msgs::PoseStamped::ConstPtr& pMsg);
};

class VelData_t {
 public:
  geometry_msgs::TwistStamped msg;
  Eigen::Vector3f v;
  float yaw_rate;
  ros::Time rcv_stamp;

  VelData_t();
  void feed(const geometry_msgs::TwistStamped::ConstPtr& pMsg);
};

class ImuData_t {
 public:
  sensor_msgs::Imu msg;
  Eigen::Vector3f w;
  Eigen::Vector3f a;
  Eigen::Quaternionf q;
  float roll, pitch, yaw;
  ros::Time rcv_stamp;

  ImuData_t();
  void feed(const sensor_msgs::Imu::ConstPtr& pMsg);
};

class DataManager {
 public:
  static DataManager& getInstance();

  DataManager(const DataManager&) = delete;
  DataManager& operator=(const DataManager&) = delete;

  GimbalPoseData_t getGimbalPose() const;
  StateData_t getState() const;
  HomePosData_t getHomePos() const;
  LocalPosData_t getLocalPos() const;
  VelData_t getVelBody() const;
  VelData_t getVelEnu() const;
  ImuData_t getImu() const;

  bool is_connected() const;
  bool is_armed() const;
  bool is_ready_for_fly() const;
  std::string get_flight_mode() const;
  Eigen::Quaternionf get_synced_orientation() const;

  void feed_gimbal_pose(const nav_msgs::Odometry::ConstPtr& msg);
  void feed_state(const mavros_msgs::State::ConstPtr& msg);
  void feed_home_pos(const mavros_msgs::HomePosition::ConstPtr& msg);
  void feed_local_pos(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void feed_vel_body(const geometry_msgs::TwistStamped::ConstPtr& msg);
  void feed_vel_enu(const geometry_msgs::TwistStamped::ConstPtr& msg);
  void feed_imu(const sensor_msgs::Imu::ConstPtr& msg);

 private:
  DataManager() = default;
  ~DataManager() = default;

  mutable std::mutex _mutex;
  GimbalPoseData_t _gimbal_data;
  StateData_t _state_data;
  HomePosData_t _home_data;
  LocalPosData_t _local_pos_data;
  VelData_t _vel_body_data;
  VelData_t _vel_enu_data;
  ImuData_t _imu_data;

  struct OmegaBufItem {
    double timestamp;
    Eigen::Vector3f w;
  };
  std::deque<OmegaBufItem> _omega_buffer;
  struct OriBufItem {
    double timestamp;
    Eigen::Quaternionf q;
  };
  std::deque<OriBufItem> _orientation_buffer;
  const double MAX_BUFFER_AGE = 0.5;
};
