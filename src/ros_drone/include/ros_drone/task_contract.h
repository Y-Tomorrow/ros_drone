/**
 * @file task_contract.h
 * @brief 任务模块与总管之间的数据合同（双方唯一约定）
 *
 * 双相机：短焦 WIDE（广角）+ 长焦 TELE（长焦）。
 * 总管仅在「短焦与长焦两路跟踪均已丢失」时切入搜索；lost_from_cam 描述丢失过程，
 * pixel 为丢失前最后一帧在「对应相机」上的归一化坐标（见 TargetSearcher::set_search_strategy）：
 * 中心为原点，x 右正，y 上正，分量约 [-1,1]。
 */
#pragma once

#include <Eigen/Dense>

/**
 * 丢失过程分类（双路都已丢之后由总管传入）：
 * - NONE：两路自始至终都未检出过目标（无有效末帧像素时可传 (0,0)）；
 * - BOTH：两路同时丢失；
 * - TELE：短焦已先丢，最后时刻在长焦画面丢失（pixel 为长焦图像归一化坐标）；
 * - WIDE：长焦已先丢，最后时刻在短焦画面丢失（pixel 为短焦图像归一化坐标）。
 */
enum class CameraID {
  NONE,
  BOTH,
  TELE,
  WIDE,
};

/** 模块执行状态 */
enum class TaskStatus { RUNNING, SUCCESS, FAILED };

/** 每帧交给总管的输出 */
struct TaskResult {
  TaskStatus status{TaskStatus::RUNNING};
  /** 本机线速度 (vx, vy, vz)，坐标系由总管与飞控约定（常见 ENU 或机体系） */
  Eigen::Vector3f vel_cmd{Eigen::Vector3f::Zero()};
  /** 本机偏航角速率 (rad/s) */
  float yaw_rate{0.f};
  /** 云台期望姿态 (roll, pitch, yaw)，单位 rad；顺序 roll → pitch → yaw */
  Eigen::Vector3f att_cmd{Eigen::Vector3f::Zero()};
  /** 失败等场景下回传：与本次搜索策略一致的丢失类型 */
  CameraID lost_cam_id{CameraID::NONE};
};
