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
  enum class Phase {
    PRIMARY,      // 执行指定相机对应任务（WIDE/TELE/BOTH/NONE）
    NONE_PATROL,  // 其他任务超时后进入 NONE 巡逻
    FINISHED      // 输出 INF（上层应结束/切换）
  };

  struct NoneParams {
    float timeout_s{15.0f};             // NONE 巡逻持续时间；<=0 表示不进入 NONE 巡逻
    float yaw_rate_rps{0.6f};            // 机体持续旋转 yaw_rate (rad/s)
    float gimbal_pitch_center_rad{0.0f}; // 云台 pitch 摆动中心 (rad)
    float gimbal_pitch_amp_rad{0.35f};   // 云台 pitch 摆动幅度 (rad)
    float gimbal_pitch_omega_rps{0.8f};  // 云台 pitch 摆动角频率 (rad/s)，越大摆得越快
  };

  struct TeleParams {
    bool enable_task1_accel{true};        // 追击速度基础上加速（vx）
    bool enable_task2_gimbal_drift{true}; // 云台朝丢失方向缓慢漂移

    float timeout_s{60.0f};

    // 保持/置零策略：|当前值| < threshold 认为是噪声 -> 置 0；否则保持当前（避免“真实速度被抹掉”）
    float vy_keep_threshold_mps{0.15f};
    float vz_keep_threshold_mps{0.15f};
    float yaw_rate_keep_threshold_rps{0.10f};

    // --- task1: 加速（vx）---
    float accel_mps2{0.6f};     // vx 每秒增加多少 (m/s^2)
    float max_vx_mps{8.0f};     // vx 上限
    // 相对丢失瞬间机体系 vx，指令 vx 最多再增加多少 (m/s)；<=0 表示不限制
    float max_vx_rise_from_lost_mps{0.f};

    // --- task2: 云台漂移（速率限制 + 偏移限幅；以丢失瞬间云台角为基准）---
    float gimbal_pitch_rate_rps{0.10f};      // |pixel_y|=1 时的 pitch 漂移速率 (rad/s)
    float gimbal_yaw_rate_rps{0.15f};        // |pixel_x|=1 时的 yaw 漂移速率 (rad/s)
    float gimbal_pitch_offset_limit_rad{0.3f};
    float gimbal_yaw_offset_limit_rad{0.5f};
  };

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
  CameraID _lost_cam{CameraID::NONE};        // 当前正在执行的“外部相机模式”
  CameraID _primary_cam{CameraID::NONE};     // set_search_strategy 时指定的主任务相机
  Phase _phase{Phase::PRIMARY};
  float _phase_time{0.0f};                   // 当前 phase 已运行时间
  Eigen::Vector2f _lost_pixel{Eigen::Vector2f::Zero()};
  Eigen::Vector3f _gimbal_rpy_at_lost{Eigen::Vector3f::Zero()};
  float _body_yaw_at_lost{0.0f};
  float _vx_body_at_lost{0.0f};
  // 搜索阶段上一拍下发的速度指令状态（body frame v + yaw_rate）
  Eigen::Vector4f _last_cmd_vel{Eigen::Vector4f::Zero()};
  // Task1：低速分支下用状态按 decel 连续积分
  float _task1_vx_cmd_integrator{0.0f};

  // TELE: 云台相对丢失瞬间的偏移积分 (pitch, yaw)
  Eigen::Vector2f _tele_gimbal_offset{Eigen::Vector2f::Zero()};

  NoneParams _none{};
  TeleParams _tele{};
  WideParams _wide{};
};

