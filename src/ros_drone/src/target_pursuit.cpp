#include <ros_drone/target_pursuit.h>

#include <ros_drone/DataManager.h>

#include <cmath>

namespace {
constexpr float kVelGain = 0.5f;
constexpr float kVelMax = 2.0f;
constexpr float kArriveHoriz = 0.5f;
constexpr float kArriveZ = 0.3f;
}  // namespace

void TargetPursuit::reset() {
  clear_goal();
}

void TargetPursuit::clear_goal() {
  _has_goal = false;
}

void TargetPursuit::set_goal_enu(const Eigen::Vector3f& p_enu) {
  _goal_enu = p_enu;
  _has_goal = true;
}

TaskResult TargetPursuit::update(float dt) {
  (void)dt;
  TaskResult res;
  res.status = TaskStatus::RUNNING;
  res.lost_cam_id = CameraID::NONE;

  if (!_has_goal) {
    return res;
  }

  const Eigen::Vector3f pos = DataManager::getInstance().getLocalPos().p;
  const Eigen::Vector3f err = _goal_enu - pos;
  Eigen::Vector3f v = err * kVelGain;
  const float n = v.norm();
  if (n > kVelMax && n > 1e-6f) {
    v *= kVelMax / n;
  }
  res.vel_cmd = v;
  res.yaw_rate = 0.f;
  res.att_cmd.setZero();

  const float eh = err.head<2>().norm();
  if (eh < kArriveHoriz && std::abs(err.z()) < kArriveZ) {
    res.status = TaskStatus::SUCCESS;
    res.vel_cmd.setZero();
  }

  return res;
}
