/**
 * @file util.h
 * @brief 数学工具函数头文件（角度、四元数、ROS 消息转换等）
 */
#pragma once

#include <cmath>
#include <algorithm>
#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <Eigen/Dense>

inline float wrap_pi(float angle) {
  while (angle > static_cast<float>(M_PI)) angle -= 2.0f * static_cast<float>(M_PI);
  while (angle < -static_cast<float>(M_PI)) angle += 2.0f * static_cast<float>(M_PI);
  return angle;
}

inline float deg2rad_normalized(float degrees) {
  float deg_norm = std::fmod(degrees + 180.0f, 360.0f);
  if (deg_norm < 0) {
    deg_norm += 360.0f;
  }
  deg_norm -= 180.0f;
  return deg_norm * static_cast<float>(M_PI) / 180.0f;
}

inline float rad2deg_normalized(float radians) {
  float rad_norm = std::atan2(std::sin(radians), std::cos(radians));
  return rad_norm * 180.0f / static_cast<float>(M_PI);
}

template <typename T>
bool check_timeout(const T& data, double timeout_sec) {
  ros::Duration time_diff = ros::Time::now() - data.timestamp;
  return time_diff.toSec() > timeout_sec;
}

inline Eigen::Quaternionf eulerToQuaternion(float roll, float pitch, float yaw) {
  Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());
  Eigen::Quaternionf q = yawAngle * pitchAngle * rollAngle;
  q.normalize();
  return q;
}

inline Eigen::Vector3f quaternionToEuler(const Eigen::Quaternionf& q) {
  Eigen::Vector3f rpy;
  Eigen::Quaternionf qn = q.normalized();
  float x = qn.x();
  float y = qn.y();
  float z = qn.z();
  float w = qn.w();

  float sinr_cosp = 2.0f * (w * x + y * z);
  float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
  rpy(0) = std::atan2(sinr_cosp, cosr_cosp);

  float sinp = 2.0f * (w * y - z * x);
  if (std::abs(sinp) >= 1.0f)
    rpy(1) = std::copysign(1.5707963f, sinp);
  else
    rpy(1) = std::asin(sinp);

  float siny_cosp = 2.0f * (w * z + x * y);
  float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
  rpy(2) = std::atan2(siny_cosp, cosy_cosp);

  return rpy;
}

inline Eigen::Quaternionf rosToEigen(const geometry_msgs::Quaternion& ros_q) {
  return Eigen::Quaternionf(
      static_cast<float>(ros_q.w),
      static_cast<float>(ros_q.x),
      static_cast<float>(ros_q.y),
      static_cast<float>(ros_q.z));
}

inline geometry_msgs::Quaternion eigenToRos(const Eigen::Quaternionf& eigen_q) {
  geometry_msgs::Quaternion ros_q;
  ros_q.x = static_cast<double>(eigen_q.x());
  ros_q.y = static_cast<double>(eigen_q.y());
  ros_q.z = static_cast<double>(eigen_q.z());
  ros_q.w = static_cast<double>(eigen_q.w());
  return ros_q;
}

template <typename T>
T clamp(T value, T min_val, T max_val) {
  return std::max(min_val, std::min(value, max_val));
}
