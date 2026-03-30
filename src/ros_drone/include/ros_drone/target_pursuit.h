/**
 * @file target_pursuit.h
 * @brief 追击控制（不继承任何任务接口，仅产出 TaskResult 供总管使用）
 *
 * 依赖 DataManager 单例（如 getLocalPos()）；须由 capturer_node 或总管订阅 MAVROS 并 feed_* 写入。
 */
#pragma once

#include <ros_drone/task_contract.h>

#include <Eigen/Dense>

class TargetPursuit {
 public:
  TargetPursuit() = default;

  void reset();

  /**
   * 总管以固定周期调用；在此根据单例状态与你的跟踪量计算 res。
   * 以下为占位示例：若有 ENU 目标点则做简单比例逼近，你可整体替换为视觉/制导逻辑。
   */
  TaskResult update(float dt);

  /** 清除 ENU 追击点（无目标时 update 返回零速度、RUNNING） */
  void clear_goal();
  /** 设置 ENU 下期望逼近的三维点（可选，不用可不调） */
  void set_goal_enu(const Eigen::Vector3f& p_enu);
  bool has_goal() const { return _has_goal; }

 private:
  bool _has_goal{false};
  Eigen::Vector3f _goal_enu{0.f, 0.f, 0.f};
};
