#pragma once

#include <limits>
#include <Eigen/Dense>

#include <ros/ros.h>

#include <ros_drone/DataManager.h>


struct TaskResult {
  Eigen::Vector3f gimbal_rpy = Eigen::Vector3f::Zero();  // (roll, pitch, yaw)
  Eigen::Vector4f vel_vxyzy = Eigen::Vector4f::Zero();   // (vx, vy, vz, yaw_rate)
};

class TargetSearcher {
public:
  struct WideParams {
    bool enable_task1_decel{true};
    bool enable_task2_gimbal_deflect{true};
    bool enable_task3_translate{true};   // 低速：按 pixel 给 vx/vy 增量
    bool enable_task3_yaw_rate{true};    // 低速：按 pixel 给 yaw_rate 增量

    float timeout_s{60.0f};

    // --- task1: 减速 ---
    float decel_mps2{0.8f};    // vx 每秒减少多少 (m/s^2)
    float min_vx_mps{-1.0f};   // vx 下限（允许为负）
    // 相对丢失瞬间机体系 vx，指令 vx 最多再降低多少 (m/s)；<=0 表示不限制
    float max_vx_drop_from_lost_mps{0.f};

    // --- task2: 云台偏转（相对丢失瞬间的云台角度） ---
    float gimbal_roll_rad{0.0f};
    float gimbal_pitch_gain_rad{0.25f};   // pixel.y -> pitch 偏转幅度
    float gimbal_yaw_gain_rad{0.35f};     // pixel.x -> yaw 偏转幅度
    float gimbal_pitch_limit_rad{0.8f};
    float gimbal_yaw_limit_rad{1.2f};

    // --- task3：平移 / yaw_rate（各有一开关；各通道独立“低速/保持”判断）---
    // vxyzy_keep_threshold: (vx, vy, vz, yaw_rate) in body frame
    Eigen::Vector4f vxyzy_keep_threshold{4.0f, 4.0f, 1.0f, 0.2f};
    float translate_gain_mps{1.0f};               // pixel -> (vx, vy) 增量
    float cmd_vx_limit_mps{4.0f};                 // 指令限幅（绝对值）
    float cmd_vy_limit_mps{4.0f};
    float yaw_rate_gain_rps{0.8f};                // pixel.x -> yaw_rate
    float yaw_rate_limit_rps{1.2f};
    // 机体航向相对「切入搜索瞬间」的最大偏角；<=0 表示不限制
    float body_yaw_limit_rad{0.0f};
  };

  TargetSearcher() = default;

  explicit TargetSearcher(const ros::NodeHandle& private_nh);
  void loadParams(const ros::NodeHandle& private_nh);

  // 【接口1】总管切入搜索状态瞬间调用（pixel 为归一化像素坐标）
  void set_search_strategy(CameraID lost_from_cam, const Eigen::Vector2f& pixel);

  // 【接口2】50Hz 调用，输出搜索阶段的控制指令
  TaskResult update(float dt);

  void reset();

private:
  CameraID _lost_cam{CameraID::NONE};
  float _search_time{0.0f};
  Eigen::Vector2f _lost_pixel{Eigen::Vector2f::Zero()};
  Eigen::Vector3f _gimbal_rpy_at_lost{Eigen::Vector3f::Zero()};
  float _body_yaw_at_lost{0.0f};
  float _vx_body_at_lost{0.0f};
  // Task1：低速分支下每帧若从 0 起算再只减一次 decel*dt，指令永远只有一步大小；用状态按 decel 连续积分
  float _task1_vx_cmd_integrator{0.0f};
  WideParams _wide{};
};

