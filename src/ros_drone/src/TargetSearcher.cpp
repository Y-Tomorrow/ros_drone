#include <ros_drone/TargetSearcher.h>

#include <algorithm>
#include <cmath>

#include <ros_drone/util.h>

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

  // =============== TELE ===============
  {
    ros::NodeHandle nh(private_nh, "target_searcher/tele");
    nh.param("enable_task1_accel", _tele.enable_task1_accel, _tele.enable_task1_accel);
    nh.param("enable_task2_gimbal_drift", _tele.enable_task2_gimbal_drift, _tele.enable_task2_gimbal_drift);

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

  // 约定参数命名：~target_searcher/wide/xxx
  ros::NodeHandle nh(private_nh, "target_searcher/wide");
  nh.param("enable_task1_decel", _wide.enable_task1_decel, _wide.enable_task1_decel);
  nh.param("enable_task2_gimbal_deflect", _wide.enable_task2_gimbal_deflect, _wide.enable_task2_gimbal_deflect);
  nh.param("enable_task3_translate", _wide.enable_task3_translate, _wide.enable_task3_translate);
  nh.param("enable_task3_yaw_rate", _wide.enable_task3_yaw_rate, _wide.enable_task3_yaw_rate);

  nh.param("timeout_s", _wide.timeout_s, _wide.timeout_s);

  nh.param("decel_mps2", _wide.decel_mps2, _wide.decel_mps2);
  nh.param("min_vx_mps", _wide.min_vx_mps, _wide.min_vx_mps);
  nh.param("max_vx_drop_from_lost_mps", _wide.max_vx_drop_from_lost_mps, _wide.max_vx_drop_from_lost_mps);

  nh.param("gimbal_roll_rad", _wide.gimbal_roll_rad, _wide.gimbal_roll_rad);
  nh.param("gimbal_pitch_gain_rad", _wide.gimbal_pitch_gain_rad, _wide.gimbal_pitch_gain_rad);
  nh.param("gimbal_yaw_gain_rad", _wide.gimbal_yaw_gain_rad, _wide.gimbal_yaw_gain_rad);
  nh.param("gimbal_pitch_limit_rad", _wide.gimbal_pitch_limit_rad, _wide.gimbal_pitch_limit_rad);
  nh.param("gimbal_yaw_limit_rad", _wide.gimbal_yaw_limit_rad, _wide.gimbal_yaw_limit_rad);

  // 新参数：vxyzy_keep_threshold: [vx, vy, vz, yaw_rate] (body frame)
  {
    XmlRpc::XmlRpcValue v;
    if (nh.getParam("vxyzy_keep_threshold", v) && v.getType() == XmlRpc::XmlRpcValue::TypeArray && v.size() == 4) {
      _wide.vxyzy_keep_threshold[0] = static_cast<float>(static_cast<double>(v[0]));
      _wide.vxyzy_keep_threshold[1] = static_cast<float>(static_cast<double>(v[1]));
      _wide.vxyzy_keep_threshold[2] = static_cast<float>(static_cast<double>(v[2]));
      _wide.vxyzy_keep_threshold[3] = static_cast<float>(static_cast<double>(v[3]));
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

void TargetSearcher::set_search_strategy(CameraID lost_from_cam, const Eigen::Vector2f& pixel) {
  _primary_cam = lost_from_cam;
  _lost_cam = lost_from_cam;
  _phase = Phase::PRIMARY;
  _phase_time = 0.0f;
  _lost_pixel = pixel;

  // 记录“丢失瞬间”的云台姿态作为基准（后续 task2 在此基础上偏转）
  const Eigen::Quaternionf q_gimbal = DataManager::getInstance().get_gimbal_orientation();
  _gimbal_rpy_at_lost = quaternionToEuler(q_gimbal);

  const auto lp = DataManager::getInstance().getLocalPos();
  if (lp.rcv_stamp != ros::Time(0))
    _body_yaw_at_lost = quaternionToEuler(lp.q).z();
  else
    _body_yaw_at_lost = 0.f;

  _vx_body_at_lost = DataManager::getInstance().getVelBody().v.x();
  _task1_vx_cmd_integrator = 0.f;
  _tele_gimbal_offset.setZero();
}

TaskResult TargetSearcher::update(float dt) {
  TaskResult res;
  _phase_time += dt;

  // 当前状态
  const auto vel_body = DataManager::getInstance().getVelBody();  // body frame v + yaw_rate
  const auto vel_enu = DataManager::getInstance().getVelEnu();    // ENU v + yaw_rate

  auto emitInfAndFinish = [&]() -> TaskResult {
    const float inf = std::numeric_limits<float>::infinity();
    res.gimbal_rpy = Eigen::Vector3f(inf, inf, inf);
    res.vel_vxyzy = Eigen::Vector4f(inf, inf, inf, inf);
    _phase = Phase::FINISHED;
    return res;
  };

  auto enterNonePatrolOrFinish = [&]() -> TaskResult {
    // none.timeout_s <= 0：不进入 NONE，直接 INF
    if (!(_none.timeout_s > 0.0f)) {
      return emitInfAndFinish();
    }
    _phase = Phase::NONE_PATROL;
    _lost_cam = CameraID::NONE;
    _phase_time = 0.0f;
    // 进入 NONE 后立即输出一次 NONE 指令
    //（继续往下走会落入 NONE 分支）
    return res;
  };

  // phase = NONE_PATROL：只跑 NONE 逻辑，直到 timeout_s 后 INF
  if (_phase == Phase::NONE_PATROL) {
    // 执行 NONE 巡逻
    res.vel_vxyzy = Eigen::Vector4f(0.f, 0.f, 0.f, _none.yaw_rate_rps);
    const float t = _phase_time;
    const float pitch = _none.gimbal_pitch_center_rad +
                        _none.gimbal_pitch_amp_rad * std::sin(_none.gimbal_pitch_omega_rps * t);
    res.gimbal_rpy = Eigen::Vector3f(0.f, -pitch, 0.f);

    if (_phase_time > _none.timeout_s) {
      return emitInfAndFinish();
    }
    return res;
  }

  if (_lost_cam == CameraID::TELE) {
    // =============== TELE 搜索策略（参数在 yaml，可开关各子任务） ===============

    // 速度：vx 以当前追击速度为基准（可加速）；vy/vz/yaw_rate 使用 deadband：
    // 小于阈值视为噪声置 0，大于阈值认为是真实运动则保持当前值。
    float vx_cmd = vel_body.v.x();
    float vy_cmd = (std::abs(vel_body.v.y()) < std::abs(_tele.vy_keep_threshold_mps)) ? 0.0f : vel_body.v.y();
    float vz_cmd = (std::abs(vel_body.v.z()) < std::abs(_tele.vz_keep_threshold_mps)) ? 0.0f : vel_body.v.z();
    float yaw_rate_cmd =
        (std::abs(vel_body.yaw_rate) < std::abs(_tele.yaw_rate_keep_threshold_rps)) ? 0.0f : vel_body.yaw_rate;

    // --- task1: 在追击速度基础上加速（vx） ---
    if (_tele.enable_task1_accel) {
      vx_cmd = vx_cmd + _tele.accel_mps2 * dt;
      vx_cmd = std::min(vx_cmd, _tele.max_vx_mps);
      if (_tele.max_vx_rise_from_lost_mps > 0.f) {
        const float vx_ceiling = _vx_body_at_lost + _tele.max_vx_rise_from_lost_mps;
        vx_cmd = std::min(vx_cmd, vx_ceiling);
      }
    }

    res.vel_vxyzy = Eigen::Vector4f(vx_cmd, vy_cmd, vz_cmd, yaw_rate_cmd);

    // --- task2: 云台低速朝丢失方向漂移（以丢失瞬间云台角为基准） ---
    if (_tele.enable_task2_gimbal_drift) {
      // pixel 的符号直接决定漂移方向；幅值决定漂移速率（轻微、低速）
      _tele_gimbal_offset.y() += clamp(_lost_pixel.x(), -1.0f, 1.0f) * _tele.gimbal_yaw_rate_rps * dt;     // yaw offset
      _tele_gimbal_offset.x() += clamp(-_lost_pixel.y(), -1.0f, 1.0f) * _tele.gimbal_pitch_rate_rps * dt;  // pitch offset

      _tele_gimbal_offset.x() =
          clamp(_tele_gimbal_offset.x(), -std::abs(_tele.gimbal_pitch_offset_limit_rad), std::abs(_tele.gimbal_pitch_offset_limit_rad));
      _tele_gimbal_offset.y() =
          clamp(_tele_gimbal_offset.y(), -std::abs(_tele.gimbal_yaw_offset_limit_rad), std::abs(_tele.gimbal_yaw_offset_limit_rad));

      // TELE 下沿用 WIDE 的“pitch 取反”现场约定
      res.gimbal_rpy = Eigen::Vector3f(
          _gimbal_rpy_at_lost.x(),
          -(_gimbal_rpy_at_lost.y() + _tele_gimbal_offset.x()),
          _gimbal_rpy_at_lost.z() + _tele_gimbal_offset.y());
    } else {
      res.gimbal_rpy = Eigen::Vector3f(_gimbal_rpy_at_lost.x(), -_gimbal_rpy_at_lost.y(), _gimbal_rpy_at_lost.z());
    }

    // TELE 超时：先进入 NONE 巡逻（若 none.timeout_s>0），再由 NONE 超时输出 INF
    if (_phase_time > _tele.timeout_s) {
      enterNonePatrolOrFinish();
      // 若直接 finish，上面已返回 INF；否则继续执行 NONE（下一帧也行，这里直接给一次输出更顺滑）
      if (_phase == Phase::NONE_PATROL) {
        res.vel_vxyzy = Eigen::Vector4f(0.f, 0.f, 0.f, _none.yaw_rate_rps);
        const float t = _phase_time;  // 此刻 phase_time 已被重置为 0
        const float pitch = _none.gimbal_pitch_center_rad +
                            _none.gimbal_pitch_amp_rad * std::sin(_none.gimbal_pitch_omega_rps * t);
        res.gimbal_rpy = Eigen::Vector3f(0.f, -pitch, 0.f);
      }
      return res;
    }
  } 
  else if (_lost_cam == CameraID::WIDE) {
    // =============== WIDE 搜索策略（参数在 yaml，可开关各子任务） ===============

    // 以 body 速度各通道独立判断“低速/保持”
    const float vx_now = vel_body.v.x();
    const float vy_now = vel_body.v.y();
    const float vz_now = vel_body.v.z();
    const float yaw_rate_now = vel_body.yaw_rate;

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

    // --- task1: 减速（vx 逐步减小，可到负数，且有下限） ---
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

    // --- task3：低速时平移 / yaw_rate（分别开关）---
    if (_wide.enable_task3_translate) {
      if (low_vy) vy_cmd += (_lost_pixel.x()) * (-_wide.translate_gain_mps);    
      if (low_vz) vz_cmd += (_lost_pixel.y()) * (_wide.translate_gain_mps);     
    }
    if (_wide.enable_task3_yaw_rate) {
      if (low_yaw) yaw_rate_cmd += (_lost_pixel.x()) * _wide.yaw_rate_gain_rps;
    }

    // 限幅（避免指令爆掉）
    vx_cmd = clamp(vx_cmd, -std::abs(_wide.cmd_vx_limit_mps), std::abs(_wide.cmd_vx_limit_mps));
    vy_cmd = clamp(vy_cmd, -std::abs(_wide.cmd_vy_limit_mps), std::abs(_wide.cmd_vy_limit_mps));
    yaw_rate_cmd = clamp(yaw_rate_cmd, -std::abs(_wide.yaw_rate_limit_rps), std::abs(_wide.yaw_rate_limit_rps));

    // 机体航向角限幅：相对切入搜索时的 yaw，只允许在 ±body_yaw_limit_rad 内（超出则禁止继续往外侧转）
    if (_wide.body_yaw_limit_rad > 0.f) {
      const auto lp = DataManager::getInstance().getLocalPos();
      if (lp.rcv_stamp != ros::Time(0)) {
        const float yaw_now = quaternionToEuler(lp.q).z();
        const float dyaw = wrap_pi(yaw_now - _body_yaw_at_lost);
        const float lim = std::abs(_wide.body_yaw_limit_rad);
        // 下面 yaw_rate_cmd 会整体取反输出，饱和方向与未取反时相反
        if (dyaw > lim)
          yaw_rate_cmd = std::max(yaw_rate_cmd, 0.f);
        else if (dyaw < -lim)
          yaw_rate_cmd = std::min(yaw_rate_cmd, 0.f);
      }
    }

    yaw_rate_cmd = -yaw_rate_cmd;  // 机体 yaw_rate 指令符号与云台 pitch 一并按现场约定取反

    res.vel_vxyzy = Eigen::Vector4f(vx_cmd, vy_cmd, vz_cmd, yaw_rate_cmd);

    // --- task2: 云台在原角度上向丢失方向偏转 ---
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
  } 
  else if (_lost_cam == CameraID::BOTH) {
    // TODO: BOTH 策略留空
  } 
  else {  // CameraID::NONE
    // 直接 NONE：若 timeout_s<=0，按“不进入搜索模式”处理，直接 INF
    if (!(_none.timeout_s > 0.0f)) {
      return emitInfAndFinish();
    }
    // 直接 NONE：按 NONE_PATROL 计时规则
    res.vel_vxyzy = Eigen::Vector4f(0.f, 0.f, 0.f, _none.yaw_rate_rps);
    const float t = _phase_time;
    const float pitch = _none.gimbal_pitch_center_rad +
                        _none.gimbal_pitch_amp_rad * std::sin(_none.gimbal_pitch_omega_rps * t);
    res.gimbal_rpy = Eigen::Vector3f(0.f, -pitch, 0.f);
    if (_phase_time > _none.timeout_s) {
      return emitInfAndFinish();
    }
  }

  // WIDE 超时：先进入 NONE 巡逻（若 none.timeout_s>0），再由 NONE 超时输出 INF
  if (_lost_cam == CameraID::WIDE && _phase_time > _wide.timeout_s) {
    enterNonePatrolOrFinish();
    if (_phase == Phase::NONE_PATROL) {
      res.vel_vxyzy = Eigen::Vector4f(0.f, 0.f, 0.f, _none.yaw_rate_rps);
      const float t = _phase_time;  // 此刻 phase_time 已被重置为 0
      const float pitch = _none.gimbal_pitch_center_rad +
                          _none.gimbal_pitch_amp_rad * std::sin(_none.gimbal_pitch_omega_rps * t);
      res.gimbal_rpy = Eigen::Vector3f(0.f, -pitch, 0.f);
    } else {
      return res; // emitInfAndFinish 已返回
    }
  }

  return res;
}

void TargetSearcher::reset() {
  _lost_cam = CameraID::NONE;
  _primary_cam = CameraID::NONE;
  _phase = Phase::PRIMARY;
  _phase_time = 0.0f;
  _lost_pixel.setZero();
  _gimbal_rpy_at_lost.setZero();
  _body_yaw_at_lost = 0.f;
  _vx_body_at_lost = 0.f;
  _task1_vx_cmd_integrator = 0.f;
  _tele_gimbal_offset.setZero();
}

