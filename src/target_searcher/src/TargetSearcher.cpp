#include <target_searcher/TargetSearcher.h>

#include <algorithm>
#include <cmath>

#include <ros_drone/util.h>

namespace {

// 水平面：机体系前向 (x,y) -> 世界系（与 local pose yaw 一致：psi 为绕 +Z 从世界 x 到机头在水平面投影的角）
inline Eigen::Vector2f body_xy_to_world_xy(const Eigen::Vector2f& vb, float yaw_world_rad) {
  const float c = std::cos(yaw_world_rad);
  const float s = std::sin(yaw_world_rad);
  return Eigen::Vector2f(c * vb.x() - s * vb.y(), s * vb.x() + c * vb.y());
}

inline Eigen::Vector2f world_xy_to_body_xy(const Eigen::Vector2f& vw, float yaw_world_rad) {
  const float c = std::cos(yaw_world_rad);
  const float s = std::sin(yaw_world_rad);
  return Eigen::Vector2f(c * vw.x() + s * vw.y(), -s * vw.x() + c * vw.y());
}

// 纵向标量沿 yaw_ref 对应的世界水平轴；保留上一拍「世界系水平」指令在垂直于该轴的分量
inline void compose_world_horizontal_from_world_last(float yaw_ref_world_rad,
                                                     const Eigen::Vector2f& vw_last_xy,
                                                     float s_cmd_along_ref,
                                                     Eigen::Vector2f* out_vw_xy) {
  const float cr = std::cos(yaw_ref_world_rad);
  const float sr = std::sin(yaw_ref_world_rad);
  const Eigen::Vector2f e_ref(cr, sr);
  const Eigen::Vector2f e_perp(-sr, cr);
  const float t = vw_last_xy.dot(e_perp);
  *out_vw_xy = e_ref * s_cmd_along_ref + e_perp * t;
}

}  // namespace

TargetSearcher::TargetSearcher(const ros::NodeHandle& private_nh) { loadParams(private_nh); }

void TargetSearcher::loadParams(const ros::NodeHandle& private_nh) {
  // =============== NONE ===============
  {
    ros::NodeHandle nh(private_nh, "target_searcher/none");
    nh.param("timeout_s", _none.timeout_s, _none.timeout_s);
    nh.param("yaw_rate_rps", _none.yaw_rate_rps, _none.yaw_rate_rps);
    nh.param("gimbal_pitch_center_rad", _none.gimbal_pitch_center_rad, _none.gimbal_pitch_center_rad);
    nh.param("gimbal_pitch_amp_rad", _none.gimbal_pitch_amp_rad, _none.gimbal_pitch_amp_rad);
    nh.param("gimbal_pitch_omega_rps", _none.gimbal_pitch_omega_rps, _none.gimbal_pitch_omega_rps);
  }

  // =============== 单相机：边界距离阈值（选 WIDE / TELE） ===============
  {
    ros::NodeHandle nh(private_nh, "target_searcher/search");
    nh.param("edge_distance_threshold", _search.edge_distance_threshold, _search.edge_distance_threshold);
  }

  // =============== TELE ===============
  {
    ros::NodeHandle nh(private_nh, "target_searcher/tele");
    nh.param("enable_task1_accel", _tele.enable_task1_accel, _tele.enable_task1_accel);
    nh.param("enable_task2_gimbal_drift", _tele.enable_task2_gimbal_drift, _tele.enable_task2_gimbal_drift);
    nh.param("longitudinal_cmd_in_body_frame", _tele.longitudinal_cmd_in_body_frame, _tele.longitudinal_cmd_in_body_frame);

    nh.param("timeout_s", _tele.timeout_s, _tele.timeout_s);

    nh.param("vy_keep_threshold_mps", _tele.vy_keep_threshold_mps, _tele.vy_keep_threshold_mps);
    nh.param("vz_keep_threshold_mps", _tele.vz_keep_threshold_mps, _tele.vz_keep_threshold_mps);
    nh.param("yaw_rate_keep_threshold_rps", _tele.yaw_rate_keep_threshold_rps, _tele.yaw_rate_keep_threshold_rps);

    nh.param("accel_mps2", _tele.accel_mps2, _tele.accel_mps2);
    nh.param("max_vx_mps", _tele.max_vx_mps, _tele.max_vx_mps);
    nh.param("max_vx_rise_from_lost_mps", _tele.max_vx_rise_from_lost_mps, _tele.max_vx_rise_from_lost_mps);

    nh.param("gimbal_pitch_rate_rps", _tele.gimbal_pitch_rate_rps, _tele.gimbal_pitch_rate_rps);
    nh.param("gimbal_yaw_rate_rps", _tele.gimbal_yaw_rate_rps, _tele.gimbal_yaw_rate_rps);
    nh.param("gimbal_pitch_offset_limit_rad", _tele.gimbal_pitch_offset_limit_rad, _tele.gimbal_pitch_offset_limit_rad);
    nh.param("gimbal_yaw_offset_limit_rad", _tele.gimbal_yaw_offset_limit_rad, _tele.gimbal_yaw_offset_limit_rad);
  }

  // =============== WIDE ===============
  ros::NodeHandle nh(private_nh, "target_searcher/wide");
  nh.param("enable_task1_decel", _wide.enable_task1_decel, _wide.enable_task1_decel);
  nh.param("enable_task2_gimbal_deflect", _wide.enable_task2_gimbal_deflect, _wide.enable_task2_gimbal_deflect);
  nh.param("enable_task3_translate", _wide.enable_task3_translate, _wide.enable_task3_translate);
  nh.param("enable_task3_yaw_rate", _wide.enable_task3_yaw_rate, _wide.enable_task3_yaw_rate);
  nh.param("longitudinal_cmd_in_body_frame", _wide.longitudinal_cmd_in_body_frame, _wide.longitudinal_cmd_in_body_frame);

  nh.param("timeout_s", _wide.timeout_s, _wide.timeout_s);

  nh.param("decel_mps2", _wide.decel_mps2, _wide.decel_mps2);
  nh.param("min_vx_mps", _wide.min_vx_mps, _wide.min_vx_mps);
  nh.param("max_vx_drop_from_lost_mps", _wide.max_vx_drop_from_lost_mps, _wide.max_vx_drop_from_lost_mps);

  nh.param("gimbal_roll_rad", _wide.gimbal_roll_rad, _wide.gimbal_roll_rad);
  nh.param("gimbal_pitch_gain_rad", _wide.gimbal_pitch_gain_rad, _wide.gimbal_pitch_gain_rad);
  nh.param("gimbal_yaw_gain_rad", _wide.gimbal_yaw_gain_rad, _wide.gimbal_yaw_gain_rad);
  nh.param("gimbal_pitch_limit_rad", _wide.gimbal_pitch_limit_rad, _wide.gimbal_pitch_limit_rad);
  nh.param("gimbal_yaw_limit_rad", _wide.gimbal_yaw_limit_rad, _wide.gimbal_yaw_limit_rad);

  {
    XmlRpc::XmlRpcValue v;
    if (nh.getParam("vxyzy_keep_threshold", v) && v.getType() == XmlRpc::XmlRpcValue::TypeArray && v.size() == 4) {
      _wide.vxyzy_keep_threshold[0] = static_cast<float>(static_cast<double>(v[0]));
      _wide.vxyzy_keep_threshold[1] = static_cast<float>(static_cast<double>(v[1]));
      _wide.vxyzy_keep_threshold[2] = static_cast<float>(static_cast<double>(v[2]));
      _wide.vxyzy_keep_threshold[3] = static_cast<float>(static_cast<double>(v[3]));
    } else if (nh.getParam("vxyzy_keep_threshold", v) && v.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
      if (v.hasMember("vx") && v.hasMember("vy") && v.hasMember("vz") && v.hasMember("yaw_rate")) {
        _wide.vxyzy_keep_threshold[0] = static_cast<float>(static_cast<double>(v["vx"]));
        _wide.vxyzy_keep_threshold[1] = static_cast<float>(static_cast<double>(v["vy"]));
        _wide.vxyzy_keep_threshold[2] = static_cast<float>(static_cast<double>(v["vz"]));
        _wide.vxyzy_keep_threshold[3] = static_cast<float>(static_cast<double>(v["yaw_rate"]));
      } else {
        ROS_WARN("Param ~target_searcher/wide/vxyzy_keep_threshold struct missing keys; using defaults.");
      }
    } else {
      ROS_WARN("Param ~target_searcher/wide/vxyzy_keep_threshold not set or invalid; using defaults.");
    }
  }
  nh.param("translate_gain_mps", _wide.translate_gain_mps, _wide.translate_gain_mps);
  nh.param("cmd_vx_limit_mps", _wide.cmd_vx_limit_mps, _wide.cmd_vx_limit_mps);
  nh.param("cmd_vy_limit_mps", _wide.cmd_vy_limit_mps, _wide.cmd_vy_limit_mps);
  nh.param("yaw_rate_gain_rps", _wide.yaw_rate_gain_rps, _wide.yaw_rate_gain_rps);
  nh.param("yaw_rate_limit_rps", _wide.yaw_rate_limit_rps, _wide.yaw_rate_limit_rps);
  nh.param("body_yaw_limit_rad", _wide.body_yaw_limit_rad, _wide.body_yaw_limit_rad);
}

void TargetSearcher::set_search_strategy() {
  _lost_pixel = DataManager::getInstance().get_pixel_norm();

  // pixel_norm 到画面边界的最小距离；<= 阈值 -> WIDE（wide 参数），否则 -> TELE（tele 参数）
  const float dist_x_to_edge = 1.0f - std::abs(_lost_pixel.x());
  const float dist_y_to_edge = 1.0f - std::abs(_lost_pixel.y());
  const float min_dist_to_edge = std::max(0.0f, std::min(dist_x_to_edge, dist_y_to_edge));
  _search_profile = (min_dist_to_edge <= std::abs(_search.edge_distance_threshold)) ? SearchProfile::WIDE
                                                                                     : SearchProfile::TELE;

  _phase = Phase::PRIMARY;
  _phase_time = 0.0f;

  const Eigen::Quaternionf q_gimbal = DataManager::getInstance().get_gimbal_orientation();
  _gimbal_rpy_at_lost = quaternionToEuler(q_gimbal);

  const auto lp = DataManager::getInstance().getLocalPos();
  if (lp.rcv_stamp != ros::Time(0))
    _body_yaw_at_lost = quaternionToEuler(lp.q).z();
  else
    _body_yaw_at_lost = 0.f;

  _vx_body_at_lost = DataManager::getInstance().getVelBody().v.x();
  _last_cmd_vel = Eigen::Vector4f(_vx_body_at_lost,
                                  DataManager::getInstance().getVelBody().v.y(),
                                  DataManager::getInstance().getVelBody().v.z(),
                                  DataManager::getInstance().getVelBody().yaw_rate);
  {
    float psi_now = _body_yaw_at_lost;
    if (lp.rcv_stamp != ros::Time(0))
      psi_now = quaternionToEuler(lp.q).z();
    const float cr = std::cos(_body_yaw_at_lost);
    const float sr = std::sin(_body_yaw_at_lost);
    const Eigen::Vector2f e_ref(cr, sr);
    const auto venu = DataManager::getInstance().getVelEnu();
    if (venu.rcv_stamp != ros::Time(0)) {
      _last_cmd_world_xy = Eigen::Vector2f(venu.v.x(), venu.v.y());
      _long_speed_world_at_lost = _last_cmd_world_xy.dot(e_ref);
    } else {
      _last_cmd_world_xy = body_xy_to_world_xy(_last_cmd_vel.head<2>(), psi_now);
      _long_speed_world_at_lost = _last_cmd_world_xy.dot(e_ref);
    }
  }
  _task1_vx_cmd_integrator = 0.f;
  _tele_gimbal_offset.setZero();
}

TaskResult TargetSearcher::update(float dt) {
  TaskResult res;
  res.velocity_frame = TaskResult::VelocityFrame::BODY;
  _phase_time += dt;

  auto emitInfAndFinish = [&]() -> TaskResult {
    const float inf = std::numeric_limits<float>::infinity();
    res.gimbal_rpy = Eigen::Vector3f(inf, inf, inf);
    res.vel_vxyzy = Eigen::Vector4f(inf, inf, inf, inf);
    res.velocity_frame = TaskResult::VelocityFrame::BODY;
    _phase = Phase::FINISHED;
    return res;
  };

  auto enterNonePatrolOrFinish = [&]() -> TaskResult {
    if (!(_none.timeout_s > 0.0f)) {
      return emitInfAndFinish();
    }
    _phase = Phase::NONE_PATROL;
    _search_profile = SearchProfile::IDLE;
    _phase_time = 0.0f;
    return res;
  };

  if (_phase == Phase::NONE_PATROL) {
    res.velocity_frame = TaskResult::VelocityFrame::BODY;
    res.vel_vxyzy = Eigen::Vector4f(0.f, 0.f, 0.f, _none.yaw_rate_rps);
    const float t = _phase_time;
    const float pitch = _none.gimbal_pitch_center_rad +
                        _none.gimbal_pitch_amp_rad * std::sin(_none.gimbal_pitch_omega_rps * t);
    res.gimbal_rpy = Eigen::Vector3f(0.f, -pitch, 0.f);

    if (_phase_time > _none.timeout_s) {
      return emitInfAndFinish();
    }
    _last_cmd_vel = res.vel_vxyzy;
    _last_cmd_world_xy.setZero();
    return res;
  }

  if (_search_profile == SearchProfile::TELE) {
    float vy_cmd = (std::abs(_last_cmd_vel(1)) < std::abs(_tele.vy_keep_threshold_mps)) ? 0.0f : _last_cmd_vel(1);
    float vz_cmd = (std::abs(_last_cmd_vel(2)) < std::abs(_tele.vz_keep_threshold_mps)) ? 0.0f : _last_cmd_vel(2);
    float yaw_rate_cmd =
        (std::abs(_last_cmd_vel(3)) < std::abs(_tele.yaw_rate_keep_threshold_rps)) ? 0.0f : _last_cmd_vel(3);

    float vx_cmd = 0.f;
    const auto lp_tele = DataManager::getInstance().getLocalPos();
    const float psi_now_tele =
        (lp_tele.rcv_stamp != ros::Time(0)) ? quaternionToEuler(lp_tele.q).z() : _body_yaw_at_lost;

    if (_tele.longitudinal_cmd_in_body_frame) {
      vx_cmd = _last_cmd_vel(0);
      if (_tele.enable_task1_accel) {
        vx_cmd = vx_cmd + _tele.accel_mps2 * dt;
        vx_cmd = std::min(vx_cmd, _tele.max_vx_mps);
        if (_tele.max_vx_rise_from_lost_mps > 0.f) {
          const float vx_ceiling = _vx_body_at_lost + _tele.max_vx_rise_from_lost_mps;
          vx_cmd = std::min(vx_cmd, vx_ceiling);
        }
      }
      res.velocity_frame = TaskResult::VelocityFrame::BODY;
      res.vel_vxyzy = Eigen::Vector4f(vx_cmd, vy_cmd, vz_cmd, yaw_rate_cmd);
      _last_cmd_world_xy = body_xy_to_world_xy(Eigen::Vector2f(vx_cmd, vy_cmd), psi_now_tele);
      _last_cmd_vel = res.vel_vxyzy;
    } else {
      const float cr = std::cos(_body_yaw_at_lost);
      const float sr = std::sin(_body_yaw_at_lost);
      float s_cmd = _last_cmd_world_xy.dot(Eigen::Vector2f(cr, sr));
      if (_tele.enable_task1_accel) {
        s_cmd = s_cmd + _tele.accel_mps2 * dt;
        s_cmd = std::min(s_cmd, _tele.max_vx_mps);
        if (_tele.max_vx_rise_from_lost_mps > 0.f) {
          const float s_ceiling = _long_speed_world_at_lost + _tele.max_vx_rise_from_lost_mps;
          s_cmd = std::min(s_cmd, s_ceiling);
        }
      }
      Eigen::Vector2f vw_new;
      compose_world_horizontal_from_world_last(_body_yaw_at_lost, _last_cmd_world_xy, s_cmd, &vw_new);
      _last_cmd_world_xy = vw_new;
      const Eigen::Vector2f vb_fb = world_xy_to_body_xy(vw_new, psi_now_tele);
      _last_cmd_vel = Eigen::Vector4f(vb_fb.x(), vb_fb.y(), vz_cmd, yaw_rate_cmd);
      res.velocity_frame = TaskResult::VelocityFrame::ENU;
      res.vel_vxyzy = Eigen::Vector4f(vw_new.x(), vw_new.y(), vz_cmd, yaw_rate_cmd);
    }

    if (_tele.enable_task2_gimbal_drift) {
      _tele_gimbal_offset.y() += clamp(_lost_pixel.x(), -1.0f, 1.0f) * _tele.gimbal_yaw_rate_rps * dt;
      _tele_gimbal_offset.x() += clamp(-_lost_pixel.y(), -1.0f, 1.0f) * _tele.gimbal_pitch_rate_rps * dt;

      _tele_gimbal_offset.x() =
          clamp(_tele_gimbal_offset.x(), -std::abs(_tele.gimbal_pitch_offset_limit_rad), std::abs(_tele.gimbal_pitch_offset_limit_rad));
      _tele_gimbal_offset.y() =
          clamp(_tele_gimbal_offset.y(), -std::abs(_tele.gimbal_yaw_offset_limit_rad), std::abs(_tele.gimbal_yaw_offset_limit_rad));

      res.gimbal_rpy = Eigen::Vector3f(
          _gimbal_rpy_at_lost.x(),
          -(_gimbal_rpy_at_lost.y() + _tele_gimbal_offset.x()),
          _gimbal_rpy_at_lost.z() + _tele_gimbal_offset.y());
    } else {
      res.gimbal_rpy = Eigen::Vector3f(_gimbal_rpy_at_lost.x(), -_gimbal_rpy_at_lost.y(), _gimbal_rpy_at_lost.z());
    }

    if (_phase_time > _tele.timeout_s) {
      enterNonePatrolOrFinish();
      if (_phase == Phase::NONE_PATROL) {
        res.velocity_frame = TaskResult::VelocityFrame::BODY;
        res.vel_vxyzy = Eigen::Vector4f(0.f, 0.f, 0.f, _none.yaw_rate_rps);
        const float t = _phase_time;
        const float pitch = _none.gimbal_pitch_center_rad +
                            _none.gimbal_pitch_amp_rad * std::sin(_none.gimbal_pitch_omega_rps * t);
        res.gimbal_rpy = Eigen::Vector3f(0.f, -pitch, 0.f);
      }
      _last_cmd_vel = res.vel_vxyzy;
      _last_cmd_world_xy.setZero();
      return res;
    }
  } else if (_search_profile == SearchProfile::WIDE) {
    const float vx_now = _last_cmd_vel(0);
    const float vy_now = _last_cmd_vel(1);
    const float vz_now = _last_cmd_vel(2);
    const float yaw_rate_now = _last_cmd_vel(3);

    const float th_vx = std::abs(_wide.vxyzy_keep_threshold[0]);
    const float th_vy = std::abs(_wide.vxyzy_keep_threshold[1]);
    const float th_vz = std::abs(_wide.vxyzy_keep_threshold[2]);
    const float th_yaw = std::abs(_wide.vxyzy_keep_threshold[3]);

    const bool low_vx = std::abs(vx_now) < th_vx;
    const bool low_vy = std::abs(vy_now) < th_vy;
    const bool low_vz = std::abs(vz_now) < th_vz;
    const bool low_yaw = std::abs(yaw_rate_now) < th_yaw;

    float vx_cmd = low_vx ? 0.f : vx_now;
    float vy_cmd = low_vy ? 0.f : vy_now;
    float vz_cmd = low_vz ? 0.f : vz_now;
    float yaw_rate_cmd = low_yaw ? 0.f : yaw_rate_now;

    if (_wide.enable_task1_decel) {
      if (low_vx) {
        _task1_vx_cmd_integrator =
            std::max(_task1_vx_cmd_integrator - _wide.decel_mps2 * dt, _wide.min_vx_mps);
        vx_cmd = _task1_vx_cmd_integrator;
      } else {
        vx_cmd = std::max(vx_now - _wide.decel_mps2 * dt, _wide.min_vx_mps);
        _task1_vx_cmd_integrator = vx_cmd;
      }
      if (_wide.max_vx_drop_from_lost_mps > 0.f) {
        const float vx_floor = _vx_body_at_lost - _wide.max_vx_drop_from_lost_mps;
        const float vx_before_floor = vx_cmd;
        vx_cmd = std::max(vx_cmd, vx_floor);
        if (vx_cmd != vx_before_floor)
          _task1_vx_cmd_integrator = vx_cmd;
      }
    } else {
      if (low_vx)
        _task1_vx_cmd_integrator = 0.f;
      else
        _task1_vx_cmd_integrator = vx_cmd;
    }

    if (_wide.enable_task3_translate) {
      if (low_vy) vy_cmd += (_lost_pixel.x()) * (-_wide.translate_gain_mps);
      if (low_vz) vz_cmd += (_lost_pixel.y()) * (_wide.translate_gain_mps);
    }
    if (_wide.enable_task3_yaw_rate) {
      if (low_yaw) yaw_rate_cmd += (_lost_pixel.x()) * _wide.yaw_rate_gain_rps;
    }

    vx_cmd = clamp(vx_cmd, -std::abs(_wide.cmd_vx_limit_mps), std::abs(_wide.cmd_vx_limit_mps));
    vy_cmd = clamp(vy_cmd, -std::abs(_wide.cmd_vy_limit_mps), std::abs(_wide.cmd_vy_limit_mps));
    yaw_rate_cmd = clamp(yaw_rate_cmd, -std::abs(_wide.yaw_rate_limit_rps), std::abs(_wide.yaw_rate_limit_rps));

    if (_wide.body_yaw_limit_rad > 0.f) {
      const auto lp = DataManager::getInstance().getLocalPos();
      if (lp.rcv_stamp != ros::Time(0)) {
        const float yaw_now = quaternionToEuler(lp.q).z();
        const float dyaw = wrap_pi(yaw_now - _body_yaw_at_lost);
        const float lim = std::abs(_wide.body_yaw_limit_rad);
        if (dyaw > lim)
          yaw_rate_cmd = std::max(yaw_rate_cmd, 0.f);
        else if (dyaw < -lim)
          yaw_rate_cmd = std::min(yaw_rate_cmd, 0.f);
      }
    }

    yaw_rate_cmd = -yaw_rate_cmd;

    {
      const auto lp_wide = DataManager::getInstance().getLocalPos();
      const float psi_now_wide =
          (lp_wide.rcv_stamp != ros::Time(0)) ? quaternionToEuler(lp_wide.q).z() : _body_yaw_at_lost;
      if (!_wide.longitudinal_cmd_in_body_frame) {
        Eigen::Vector2f vw_new;
        compose_world_horizontal_from_world_last(_body_yaw_at_lost, _last_cmd_world_xy, vx_cmd, &vw_new);
        _last_cmd_world_xy = vw_new;
        const Eigen::Vector2f vb_fb = world_xy_to_body_xy(vw_new, psi_now_wide);
        _last_cmd_vel = Eigen::Vector4f(vb_fb.x(), vb_fb.y(), vz_cmd, yaw_rate_cmd);
        res.velocity_frame = TaskResult::VelocityFrame::ENU;
        res.vel_vxyzy = Eigen::Vector4f(vw_new.x(), vw_new.y(), vz_cmd, yaw_rate_cmd);
      } else {
        res.velocity_frame = TaskResult::VelocityFrame::BODY;
        res.vel_vxyzy = Eigen::Vector4f(vx_cmd, vy_cmd, vz_cmd, yaw_rate_cmd);
        _last_cmd_world_xy = body_xy_to_world_xy(Eigen::Vector2f(vx_cmd, vy_cmd), psi_now_wide);
        _last_cmd_vel = res.vel_vxyzy;
      }
    }

    if (_wide.enable_task2_gimbal_deflect) {
      const float d_pitch = clamp((-_lost_pixel.y()) * _wide.gimbal_pitch_gain_rad,
                                  -std::abs(_wide.gimbal_pitch_limit_rad),
                                  std::abs(_wide.gimbal_pitch_limit_rad));
      const float d_yaw = clamp((_lost_pixel.x()) * _wide.gimbal_yaw_gain_rad,
                                -std::abs(_wide.gimbal_yaw_limit_rad),
                                std::abs(_wide.gimbal_yaw_limit_rad));

      res.gimbal_rpy = Eigen::Vector3f(
          _wide.gimbal_roll_rad,
          -(_gimbal_rpy_at_lost.y() + d_pitch),
          _gimbal_rpy_at_lost.z() + d_yaw);
    } else {
      res.gimbal_rpy =
          Eigen::Vector3f(_gimbal_rpy_at_lost.x(), -_gimbal_rpy_at_lost.y(), _gimbal_rpy_at_lost.z());
    }
  } else {
    if (!(_none.timeout_s > 0.0f)) {
      return emitInfAndFinish();
    }
    res.velocity_frame = TaskResult::VelocityFrame::BODY;
    res.vel_vxyzy = Eigen::Vector4f(0.f, 0.f, 0.f, _none.yaw_rate_rps);
    const float t = _phase_time;
    const float pitch = _none.gimbal_pitch_center_rad +
                        _none.gimbal_pitch_amp_rad * std::sin(_none.gimbal_pitch_omega_rps * t);
    res.gimbal_rpy = Eigen::Vector3f(0.f, -pitch, 0.f);
    if (_phase_time > _none.timeout_s) {
      return emitInfAndFinish();
    }
    _last_cmd_vel = res.vel_vxyzy;
    _last_cmd_world_xy.setZero();
  }

  if (_search_profile == SearchProfile::WIDE && _phase_time > _wide.timeout_s) {
    enterNonePatrolOrFinish();
    if (_phase == Phase::NONE_PATROL) {
      res.velocity_frame = TaskResult::VelocityFrame::BODY;
      res.vel_vxyzy = Eigen::Vector4f(0.f, 0.f, 0.f, _none.yaw_rate_rps);
      const float t = _phase_time;
      const float pitch = _none.gimbal_pitch_center_rad +
                          _none.gimbal_pitch_amp_rad * std::sin(_none.gimbal_pitch_omega_rps * t);
      res.gimbal_rpy = Eigen::Vector3f(0.f, -pitch, 0.f);
      _last_cmd_vel = res.vel_vxyzy;
      _last_cmd_world_xy.setZero();
    } else {
      return res;
    }
  }

  if (res.velocity_frame == TaskResult::VelocityFrame::BODY) {
    _last_cmd_vel = res.vel_vxyzy;
  }
  return res;
}

void TargetSearcher::reset() {
  _search_profile = SearchProfile::IDLE;
  _phase = Phase::PRIMARY;
  _phase_time = 0.0f;
  _lost_pixel.setZero();
  _gimbal_rpy_at_lost.setZero();
  _body_yaw_at_lost = 0.f;
  _vx_body_at_lost = 0.f;
  _long_speed_world_at_lost = 0.f;
  _last_cmd_vel.setZero();
  _last_cmd_world_xy.setZero();
  _task1_vx_cmd_integrator = 0.f;
  _tele_gimbal_offset.setZero();
}
