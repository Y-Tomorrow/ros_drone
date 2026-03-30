#include <ros_drone/target_searcher.h>

#include <ros_drone/DataManager.h>
#include <ros_drone/util.h>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>

namespace {

void yaml_read_float(const YAML::Node& n, const char* key, float& out) {
  if (n[key]) {
    out = n[key].as<float>();
  }
}

constexpr float kSearchFailTimeoutSec = 60.0f;

constexpr float kMinHorizontalSpeed = 0.5f;

constexpr float kTeleBodyYawRate = 0.2f;
constexpr float kBothBodyYawRate = 0.65f;
constexpr float kNoneBodyYawRate = 0.18f;
constexpr float kPixBodyYawTele = 0.35f;
constexpr float kPixBodyYawBoth = 0.28f;

constexpr float kBothRetreatSpeed = 1.35f;
constexpr float kBothClimbSpeed = 0.15f;
constexpr float kTeleCoastSpeed = 0.30f;
constexpr float kPixelRetreatSteerMax = 0.52f;
constexpr float kSteerStrengthBoth = 0.75f;
constexpr float kSteerStrengthTele = 1.0f;

constexpr float kStrafeFromPixTele = 0.35f;

constexpr float kPixGimbalYaw = 0.40f;
constexpr float kPixGimbalPitch = 0.30f;
constexpr float kPitchLim = 0.65f;

Eigen::Vector3f gimbal_scan_with_pixel(float search_time, float pitch_amp, float yaw_amp, float w_p, float w_y,
                                         const Eigen::Vector2f& pixel, float pix_yaw_scale, float pix_pitch_scale) {
  const GimbalPoseData_t gimbal = DataManager::getInstance().getGimbalPose();
  const Eigen::Vector3f rpy = quaternionToEuler(gimbal.q);
  const float d_pitch = pitch_amp * std::sin(w_p * search_time);
  const float d_yaw = yaw_amp * std::sin(w_y * search_time);
  const float px = clamp(pixel.x(), -1.f, 1.f);
  const float py = clamp(pixel.y(), -1.f, 1.f);
  const float yaw_bias = pix_yaw_scale * kPixGimbalYaw * px;
  const float pitch_bias = pix_pitch_scale * kPixGimbalPitch * py;
  float pitch = rpy(1) + d_pitch + pitch_bias;
  pitch = clamp(pitch, -kPitchLim, kPitchLim);
  const float yaw = wrap_pi(rpy(2) + d_yaw + yaw_bias);
  return Eigen::Vector3f(rpy(0), pitch, yaw);
}

void set_retreat_from_velocity_enu(TaskResult& res, const Eigen::Vector3f& vel_enu, float speed, float pixel_x,
                                     float steer_strength) {
  const float vx = vel_enu.x();
  const float vy = vel_enu.y();
  const float vh = std::sqrt(vx * vx + vy * vy);
  Eigen::Vector2f dir;
  if (vh > kMinHorizontalSpeed) {
    dir.x() = -vx / vh;
    dir.y() = -vy / vh;
  } else {
    dir.x() = -1.f;
    dir.y() = 0.f;
  }
  const float px = clamp(pixel_x, -1.f, 1.f);
  const float steer = steer_strength * kPixelRetreatSteerMax * px;
  const float c = std::cos(steer);
  const float s = std::sin(steer);
  const float dx = dir.x() * c - dir.y() * s;
  const float dy = dir.x() * s + dir.y() * c;
  res.vel_cmd.x() = dx * speed;
  res.vel_cmd.y() = dy * speed;
  res.vel_cmd.z() = 0.f;
}

void add_enu_strafe_from_pixel_x(TaskResult& res, float pixel_x, float yaw_deg, float gain) {
  const float psi = yaw_deg * static_cast<float>(M_PI) / 180.f;
  const float px = clamp(pixel_x, -1.f, 1.f);
  const float v_by = -gain * px;
  const float vn_x = -std::sin(psi) * v_by;
  const float vn_y = std::cos(psi) * v_by;
  res.vel_cmd.x() += vn_x;
  res.vel_cmd.y() += vn_y;
}

void apply_wide_search(TaskResult& res, const Eigen::Vector3f& pos, const LocalPosData_t& lp,
                       const Eigen::Vector2f& pixel, const TargetSearchWideParams& wp, const Eigen::Vector3f& origin,
                       float horiz_speed_mps) {
  const float px = clamp(pixel.x(), -1.f, 1.f);
  const float py = clamp(pixel.y(), -1.f, 1.f);

  const GimbalPoseData_t gimbal = DataManager::getInstance().getGimbalPose();
  const Eigen::Vector3f rpy = quaternionToEuler(gimbal.q);
  res.att_cmd(0) = rpy(0);
  res.att_cmd(1) =
      clamp(rpy(1) + wp.gimbal_pitch_gain_rad * py, -wp.gimbal_pitch_limit_rad, wp.gimbal_pitch_limit_rad);
  res.att_cmd(2) = wrap_pi(rpy(2) + wp.gimbal_yaw_gain_rad * px);

  // 机体偏航：朝目标丢失方向旋转，并随 |px| 平滑增强（px 归一化到 [-1,1]）。
  // wp.body_yaw_rate 作为“最大偏航角速度”使用（|px|=1 时达到上限）。
  float px_eff = px;
  if (std::abs(px_eff) < 0.08f) px_eff = 0.f;  // 小死区：减少中心附近抖动
  const float yaw_scale = 0.35f + 0.65f * std::abs(px_eff);  // 平滑增强，避免过激
  res.yaw_rate = -wp.body_yaw_rate * px_eff * yaw_scale;

  // 若丢失前/当前水平速度过小，没有可靠的平移方向，则不做平移，只动 yaw 与云台。
  if (horiz_speed_mps <= kMinHorizontalSpeed) {
    res.vel_cmd.setZero();
    return;
  }

  const float psi = lp.yaw * static_cast<float>(M_PI) / 180.f;
  float v_by = -wp.max_horizontal_speed * px;
  float vn_x = -std::sin(psi) * v_by;
  float vn_y = std::cos(psi) * v_by;
  float vz = -wp.max_vertical_speed * py;

  const Eigen::Vector3f d = pos - origin;
  const float horiz = d.head<2>().norm();
  if (horiz >= wp.max_horizontal_travel_m) {
    vn_x = 0.f;
    vn_y = 0.f;
  }
  if (vz > 0.f && d.z() >= wp.max_vertical_travel_m) {
    vz = 0.f;
  }
  if (vz < 0.f && d.z() <= -wp.max_vertical_travel_m) {
    vz = 0.f;
  }

  res.vel_cmd.x() = vn_x;
  res.vel_cmd.y() = vn_y;
  res.vel_cmd.z() = vz;
}
}  // namespace

bool TargetSearcher::load_search_config(const std::string& yaml_path) {
  try {
    YAML::Node root = YAML::LoadFile(yaml_path);
    if (!root["wide_search"]) {
      return false;
    }
    const YAML::Node w = root["wide_search"];
    yaml_read_float(w, "max_horizontal_speed", _wide_params.max_horizontal_speed);
    yaml_read_float(w, "max_vertical_speed", _wide_params.max_vertical_speed);
    yaml_read_float(w, "max_horizontal_travel_m", _wide_params.max_horizontal_travel_m);
    yaml_read_float(w, "max_vertical_travel_m", _wide_params.max_vertical_travel_m);
    yaml_read_float(w, "gimbal_yaw_gain_rad", _wide_params.gimbal_yaw_gain_rad);
    yaml_read_float(w, "gimbal_pitch_gain_rad", _wide_params.gimbal_pitch_gain_rad);
    yaml_read_float(w, "gimbal_pitch_limit_rad", _wide_params.gimbal_pitch_limit_rad);
    yaml_read_float(w, "body_yaw_rate", _wide_params.body_yaw_rate);
    _wide_params.yaml_loaded = true;
    return true;
  } catch (...) {
    return false;
  }
}

void TargetSearcher::set_search_strategy(CameraID lost_from_cam, Eigen::Vector2f pixel) {
  _lost_cam = lost_from_cam;
  _last_pixel.x() = clamp(pixel.x(), -1.f, 1.f);
  _last_pixel.y() = clamp(pixel.y(), -1.f, 1.f);
  _search_time = 0.f;
  _search_origin = DataManager::getInstance().getLocalPos().p;
}

void TargetSearcher::reset() {
  _lost_cam = CameraID::NONE;
  _last_pixel.setZero();
  _search_time = 0.f;
  _search_origin.setZero();
}

TaskResult TargetSearcher::update(float dt) {
  (void)dt;
  TaskResult res;
  res.status = TaskStatus::RUNNING;
  res.lost_cam_id = CameraID::NONE;
  _search_time += dt;

  const Eigen::Vector3f current_pos = DataManager::getInstance().getLocalPos().p;
  const Eigen::Vector3f current_vel = DataManager::getInstance().getVelEnu().v;
  const LocalPosData_t local_pose = DataManager::getInstance().getLocalPos();
  const float drift_xy = (current_pos.head<2>() - _search_origin.head<2>()).norm();
  const float vh = current_vel.head<2>().norm();

  if (_lost_cam == CameraID::NONE) {
    res.yaw_rate = kNoneBodyYawRate;
    res.vel_cmd.setZero();
    const Eigen::Vector2f no_pix(0.f, 0.f);
    res.att_cmd = gimbal_scan_with_pixel(_search_time, 0.12f, 0.25f, 0.95f, 0.7f, no_pix, 0.f, 0.f);
  } 
  else if (_lost_cam == CameraID::BOTH) {
    set_retreat_from_velocity_enu(res, current_vel, kBothRetreatSpeed, _last_pixel.x(), kSteerStrengthBoth);
    res.vel_cmd.z() = kBothClimbSpeed;
    res.yaw_rate = kBothBodyYawRate + kPixBodyYawBoth * _last_pixel.x();
    res.att_cmd = gimbal_scan_with_pixel(_search_time, 0.22f, 0.45f, 1.1f, 0.85f, _last_pixel, 0.6f, 0.6f);
  } 
  else if (_lost_cam == CameraID::TELE) {
    const float yaw_gain = 1.f + 0.05f * std::min(drift_xy / 10.f, 1.f);
    res.yaw_rate = kTeleBodyYawRate * yaw_gain + kPixBodyYawTele * _last_pixel.x();
    if (vh > kMinHorizontalSpeed) {
      set_retreat_from_velocity_enu(res, current_vel, kTeleCoastSpeed, _last_pixel.x(), kSteerStrengthTele);
    } 
    else {
      res.vel_cmd.setZero();
      add_enu_strafe_from_pixel_x(res, _last_pixel.x(), local_pose.yaw, kStrafeFromPixTele);
    }
    res.att_cmd = gimbal_scan_with_pixel(_search_time, 0.10f, 0.15f, 1.3f, 1.0f, _last_pixel, 1.0f, 1.0f);
  } 
  else if (_lost_cam == CameraID::WIDE) {
    apply_wide_search(res, current_pos, local_pose, _last_pixel, _wide_params, _search_origin, vh);
  }

  if (_search_time > kSearchFailTimeoutSec) {
    res.status = TaskStatus::FAILED;
    res.lost_cam_id = _lost_cam;
  }

  return res;
}
