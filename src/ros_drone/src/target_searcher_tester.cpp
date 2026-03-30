#include <ros/ros.h>

#include <boost/bind.hpp>

#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <algorithm>
#include <atomic>
#include <thread>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/MountControl.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/HomePosition.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include <ros/package.h>

#include <Eigen/Dense>

#include <ros_drone/DataManager.h>
#include <ros_drone/target_searcher.h>
#include <ros_drone/util.h>

namespace {

constexpr float kRad2Deg = 180.0f / 3.1415926535f;

Eigen::Vector3f enu_vel_to_ned_vel(const Eigen::Vector3f& v_enu) {
  // ENU: (x=east, y=north, z=up) -> NED: (x=north, y=east, z=down)
  return Eigen::Vector3f(v_enu.y(), v_enu.x(), -v_enu.z());
}

std::vector<CameraID> parse_run_case(const std::string& s) {
  auto up = s;
  for (char& c : up) c = static_cast<char>(::toupper(static_cast<unsigned char>(c)));

  if (up == "ALL") {
    return {CameraID::NONE, CameraID::BOTH, CameraID::TELE, CameraID::WIDE};
  }
  if (up == "NONE") return {CameraID::NONE};
  if (up == "BOTH") return {CameraID::BOTH};
  if (up == "TELE") return {CameraID::TELE};
  if (up == "WIDE") return {CameraID::WIDE};
  ROS_WARN("Unknown run_case=%s, fallback to ALL", s.c_str());
  return {CameraID::NONE, CameraID::BOTH, CameraID::TELE, CameraID::WIDE};
}

bool parse_case_token(const std::string& token, CameraID& out) {
  std::string up = token;
  std::transform(up.begin(), up.end(), up.begin(),
                 [](unsigned char ch) { return static_cast<char>(std::toupper(ch)); });
  if (up == "NONE") {
    out = CameraID::NONE;
    return true;
  }
  if (up == "BOTH") {
    out = CameraID::BOTH;
    return true;
  }
  if (up == "TELE") {
    out = CameraID::TELE;
    return true;
  }
  if (up == "WIDE") {
    out = CameraID::WIDE;
    return true;
  }
  return false;
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "target_searcher_tester");
  ros::NodeHandle nh("~");

  std::string uav_prefix = nh.param<std::string>("uav_prefix", std::string("/uav0"));
  // 兼容“只测算法不真的上电”：仅发布 mount 控制，不发布机体速度/yaw_rate
  bool send_velocity = nh.param<bool>("send_velocity", true);

  bool arm_and_offboard = nh.param<bool>("arm_and_offboard", true);
  double wait_timeout_s = nh.param<double>("wait_timeout_s", 120.0);

  // 起飞到固定高度（ENU z），便于观察机体运动
  bool takeoff_enable = nh.param<bool>("takeoff_enable", true);
  double takeoff_altitude_m = nh.param<double>("takeoff_altitude_m", 1.0);
  double takeoff_tol_m = nh.param<double>("takeoff_tol_m", 0.12);
  double takeoff_timeout_s = nh.param<double>("takeoff_timeout_s", 45.0);
  double takeoff_setpoint_rate_hz = nh.param<double>("takeoff_setpoint_rate_hz", 20.0);

  double loop_rate_hz = nh.param<double>("loop_rate_hz", 50.0);
  double scenario_duration_s = nh.param<double>("scenario_duration_s", 8.0);
  std::string run_case = nh.param<std::string>("run_case", std::string("ALL"));

  std::vector<CameraID> cases = parse_run_case(run_case);
  bool interactive = nh.param<bool>("interactive", true);

  // 各情况输入 pixel（归一化坐标，[-1,1]）
  float tele_px = nh.param<float>("tele_px", 0.3f);
  float tele_py = nh.param<float>("tele_py", -0.1f);
  float wide_px = nh.param<float>("wide_px", 0.2f);
  float wide_py = nh.param<float>("wide_py", 0.25f);
  float both_px = nh.param<float>("both_px", -0.15f);
  float both_py = nh.param<float>("both_py", 0.05f);
  float none_px = nh.param<float>("none_px", 0.0f);
  float none_py = nh.param<float>("none_py", 0.0f);

  bool vel_enu_to_ned = nh.param<bool>("vel_enu_to_ned", true);

  // 搜索参数 YAML
  std::string search_yaml_path = nh.param<std::string>("search_yaml_path", std::string(""));
  if (search_yaml_path.empty()) {
    search_yaml_path = ros::package::getPath("ros_drone") + "/config/target_search.yaml";
  }

  // DataManager：通过订阅飞控/MAVROS topic 来持续更新内部状态
  DataManager& dm = DataManager::getInstance();
  ros::AsyncSpinner spinner(3);
  spinner.start();

  // ---- 订阅：喂给 DataManager ----
  ros::Subscriber sub_state = nh.subscribe<mavros_msgs::State>(
      uav_prefix + "/mavros/state", 10, boost::bind(&DataManager::feed_state, &dm, _1));
  ros::Subscriber sub_home = nh.subscribe<mavros_msgs::HomePosition>(
      uav_prefix + "/mavros/home_position/home", 10,
      boost::bind(&DataManager::feed_home_pos, &dm, _1));
  ros::Subscriber sub_pose = nh.subscribe<geometry_msgs::PoseStamped>(
      uav_prefix + "/mavros/local_position/pose", 10,
      boost::bind(&DataManager::feed_local_pos, &dm, _1));
  ros::Subscriber sub_vel_body = nh.subscribe<geometry_msgs::TwistStamped>(
      uav_prefix + "/mavros/local_position/velocity_body", 10,
      boost::bind(&DataManager::feed_vel_body, &dm, _1));
  ros::Subscriber sub_vel_enu = nh.subscribe<geometry_msgs::TwistStamped>(
      uav_prefix + "/mavros/global_position/velocity_local", 10,
      boost::bind(&DataManager::feed_vel_enu, &dm, _1));
  ros::Subscriber sub_imu =
      nh.subscribe<sensor_msgs::Imu>(uav_prefix + "/mavros/imu/data", 10,
                                       boost::bind(&DataManager::feed_imu, &dm, _1));
  ros::Subscriber sub_gimbal_pose =
      nh.subscribe<nav_msgs::Odometry>(uav_prefix + "/mavros/gimbal_pose", 10,
                                         boost::bind(&DataManager::feed_gimbal_pose, &dm, _1));

  // ---- 发布：mount + body setpoint ----
  ros::Publisher pub_mount = nh.advertise<mavros_msgs::MountControl>(uav_prefix + "/mavros/mount_control/command", 10);
  ros::Publisher pub_setpoint_raw =
      nh.advertise<mavros_msgs::PositionTarget>(uav_prefix + "/mavros/setpoint_raw/local", 10);
  ros::Publisher pub_setpoint_pos =
      nh.advertise<geometry_msgs::PoseStamped>(uav_prefix + "/mavros/setpoint_position/local", 10);

  // ---- 等待飞控连接 ----
  double t0 = ros::Time::now().toSec();
  ROS_INFO("Waiting DataManager connected for %s ...", uav_prefix.c_str());
  while (ros::ok() && !dm.is_connected()) {
    if (ros::Time::now().toSec() - t0 > wait_timeout_s) {
      ROS_ERROR("Timeout waiting mavros connection for %s", uav_prefix.c_str());
      return 1;
    }
    ros::Duration(0.5).sleep();
  }
  ROS_INFO("DataManager connected (%s), mode=%s armed=%d", uav_prefix.c_str(),
           dm.get_flight_mode().c_str(), dm.is_armed() ? 1 : 0);

  // ---- 加载搜索 YAML ----
  TargetSearcher searcher;
  if (!searcher.load_search_config(search_yaml_path)) {
    ROS_WARN("load_search_config failed: %s (will use default params)", search_yaml_path.c_str());
  } else {
    ROS_INFO("Loaded search config: %s", search_yaml_path.c_str());
  }

  // ---- 可选：切 OFFBOARD + 解锁 ----
  if (arm_and_offboard) {
    ros::ServiceClient cli_set_mode = nh.serviceClient<mavros_msgs::SetMode>(uav_prefix + "/mavros/set_mode");
    ros::ServiceClient cli_arming = nh.serviceClient<mavros_msgs::CommandBool>(uav_prefix + "/mavros/cmd/arming");

    // OFFBOARD 通常需要先连续发送 setpoint
    mavros_msgs::PositionTarget pt0;
    pt0.header.frame_id = "map";
    pt0.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    pt0.type_mask = mavros_msgs::PositionTarget::IGNORE_PX | mavros_msgs::PositionTarget::IGNORE_PY |
                     mavros_msgs::PositionTarget::IGNORE_PZ | mavros_msgs::PositionTarget::IGNORE_AFX |
                     mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ |
                     mavros_msgs::PositionTarget::IGNORE_YAW;  // 不忽略 yaw_rate：后续会设置
    pt0.velocity.x = 0.0;
    pt0.velocity.y = 0.0;
    pt0.velocity.z = 0.0;
    pt0.yaw_rate = 0.0;

    ROS_INFO("Pre-publish setpoints for OFFBOARD...");
    ros::Rate pre_r(20.0);
    for (int i = 0; ros::ok() && i < 40; ++i) {
      pt0.header.stamp = ros::Time::now();
      pub_setpoint_raw.publish(pt0);
      ros::spinOnce();
      pre_r.sleep();
    }

    // OFFBOARD
    ROS_INFO("Setting mode to OFFBOARD...");
    mavros_msgs::SetMode srv_mode;
    srv_mode.request.custom_mode = "OFFBOARD";
    if (!cli_set_mode.call(srv_mode) || !srv_mode.response.mode_sent) {
      ROS_ERROR("Failed to set OFFBOARD");
      return 2;
    }
    ROS_INFO("OFFBOARD mode sent");

    ROS_INFO("Arming...");
    mavros_msgs::CommandBool srv_arm;
    srv_arm.request.value = true;
    if (!cli_arming.call(srv_arm) || !srv_arm.response.success) {
      ROS_ERROR("Failed to arm");
      return 3;
    }
    ROS_INFO("Armed");

    // 给飞控一点时间进入稳定状态
    ros::Duration(1.0).sleep();
  }

  // ---- 起飞到 1m（或自定义高度） ----
  std::atomic<bool> hold_active{false};
  std::atomic<bool> scenario_active{false};
  std::atomic<bool> stop_hold_thread{false};
  geometry_msgs::PoseStamped hold_sp;

  if (takeoff_enable) {
    if (!arm_and_offboard) {
      ROS_WARN("takeoff_enable=true 但 arm_and_offboard=false：当前模式下位置设定点可能不生效，将跳过起飞。");
    } else {
      // 等待 local_position 有效
      const double t_lp0 = ros::Time::now().toSec();
      while (ros::ok() && dm.getLocalPos().rcv_stamp == ros::Time(0)) {
        if (ros::Time::now().toSec() - t_lp0 > wait_timeout_s) {
          ROS_ERROR("Timeout waiting local_position for takeoff.");
          return 4;
        }
        ros::Duration(0.1).sleep();
      }

      const LocalPosData_t lp0 = dm.getLocalPos();
      geometry_msgs::PoseStamped sp;
      sp.header.frame_id = "map";
      sp.pose.position.x = lp0.p.x();
      sp.pose.position.y = lp0.p.y();
      sp.pose.position.z = takeoff_altitude_m;
      // 姿态保持当前（更稳）
      sp.pose.orientation = lp0.msg.pose.orientation;
      hold_sp = sp;

      ROS_INFO("Takeoff: publishing position setpoint z=%.2f m (tol=%.2f) ...", takeoff_altitude_m, takeoff_tol_m);
      ros::Rate r_to(std::max(5.0, takeoff_setpoint_rate_hz));
      const double t_to0 = ros::Time::now().toSec();
      while (ros::ok()) {
        sp.header.stamp = ros::Time::now();
        pub_setpoint_pos.publish(sp);
        ros::spinOnce();

        const double z = dm.getLocalPos().p.z();
        if (z >= takeoff_altitude_m - takeoff_tol_m) {
          ROS_INFO("Takeoff reached: z=%.3f", z);
          break;
        }
        if (ros::Time::now().toSec() - t_to0 > takeoff_timeout_s) {
          ROS_WARN("Takeoff timeout: current z=%.3f (continue anyway)", z);
          break;
        }
        r_to.sleep();
      }
      ros::Duration(0.5).sleep();

      // 起飞完成后，默认开启“非场景期间的悬停保持”，避免 OFFBOARD 断流掉控
      hold_active.store(true);
    }
  }

  // ---- OFFBOARD 保持线程：等待用户输入时也不断流 ----
  std::thread hold_thread([&]() {
    ros::Rate r_hold(std::max(5.0, takeoff_setpoint_rate_hz));
    while (ros::ok() && !stop_hold_thread.load()) {
      if (hold_active.load() && !scenario_active.load()) {
        hold_sp.header.stamp = ros::Time::now();
        pub_setpoint_pos.publish(hold_sp);
      }
      r_hold.sleep();
    }
  });

  auto publish_mount_from_att = [&](const Eigen::Vector3f& att_cmd_rad) {
    mavros_msgs::MountControl m;
    m.header.stamp = ros::Time::now();
    m.mode = mavros_msgs::MountControl::MAV_MOUNT_MODE_MAVLINK_TARGETING;
    // gimbal 控制只关心 pitch/yaw（与脚本保持一致）
    m.roll = 0.0;
    m.pitch = att_cmd_rad(1) * kRad2Deg;
    m.yaw = att_cmd_rad(2) * kRad2Deg;
    m.altitude = 0.0;
    m.latitude = 0.0;
    m.longitude = 0.0;
    pub_mount.publish(m);
  };

  auto publish_body_from_result = [&](const TaskResult& res) {
    if (!send_velocity) return;
    Eigen::Vector3f vel = res.vel_cmd;
    if (vel_enu_to_ned) vel = enu_vel_to_ned_vel(vel);

    mavros_msgs::PositionTarget pt;
    pt.header.stamp = ros::Time::now();
    pt.header.frame_id = "map";
    pt.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

    // 忽略位置与加速度、忽略 yaw，仅使用 velocity + yaw_rate
    pt.type_mask = mavros_msgs::PositionTarget::IGNORE_PX | mavros_msgs::PositionTarget::IGNORE_PY |
                    mavros_msgs::PositionTarget::IGNORE_PZ | mavros_msgs::PositionTarget::IGNORE_AFX |
                    mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ |
                    mavros_msgs::PositionTarget::IGNORE_YAW;

    pt.velocity.x = vel.x();
    pt.velocity.y = vel.y();
    pt.velocity.z = vel.z();

    pt.yaw_rate = res.yaw_rate;  // rad/s
    pub_setpoint_raw.publish(pt);
  };

  Eigen::Vector2f pixel_none(none_px, none_py);
  Eigen::Vector2f pixel_tele(tele_px, tele_py);
  Eigen::Vector2f pixel_wide(wide_px, wide_py);
  Eigen::Vector2f pixel_both(both_px, both_py);

  ros::Rate r(loop_rate_hz);
  double dt = 1.0 / loop_rate_hz;

  for (CameraID c : cases) {
    CameraID cur_case = c;
    Eigen::Vector2f cur_px;
    switch (c) {
      case CameraID::NONE: cur_px = pixel_none; break;
      case CameraID::BOTH: cur_px = pixel_both; break;
      case CameraID::TELE: cur_px = pixel_tele; break;
      case CameraID::WIDE: cur_px = pixel_wide; break;
      default: cur_px = pixel_none; break;
    }

    if (interactive) {
      auto case_name = [&](CameraID cc) -> const char* {
        switch (cc) {
          case CameraID::NONE: return "NONE";
          case CameraID::BOTH: return "BOTH";
          case CameraID::TELE: return "TELE";
          case CameraID::WIDE: return "WIDE";
          default: return "UNKNOWN";
        }
      };

      std::cout << std::endl
                << "[target_searcher_tester] 输入: <case> [px py] 以覆盖本次执行；或直接回车使用预设。"
                << std::endl;
      std::cout << "[target_searcher_tester] 例: TELE 0.30 -0.10, 或输入 quit 退出。" << std::endl;
      std::cout << "[target_searcher_tester] 预设: " << case_name(c) << " px=" << cur_px.x()
                << " py=" << cur_px.y() << "  (duration=" << scenario_duration_s
                << "s, uav_prefix=" << uav_prefix << ", send_velocity="
                << (send_velocity ? "true" : "false") << ")"
                << std::endl;
      std::cout << "> " << std::flush;

      // 等待输入期间持续悬停保持，防止 OFFBOARD 断流掉控
      hold_active.store(true);
      std::string line;
      std::getline(std::cin, line);
      // 输入结束后仍保持开启（只要不在执行场景，就持续悬停）
      hold_active.store(true);
      if (!ros::ok()) break;

      // 空行：使用预设
      if (!line.empty()) {
        std::string trimmed = line;
        // 同时去掉前后空白，避免“quit ”这种输入无法识别。
        trimmed.erase(trimmed.begin(),
                       std::find_if(trimmed.begin(), trimmed.end(),
                                    [](unsigned char ch) { return !std::isspace(ch); }));
        trimmed.erase(
            trimmed.find_last_not_of(" \t\n\r\f\v") + 1);
        if (trimmed == "quit" || trimmed == "QUIT" || trimmed == "exit" || trimmed == "EXIT") {
          break;
        }

        std::stringstream ss(trimmed);
        std::string case_tok;
        if (ss >> case_tok) {
          CameraID user_case;
          if (parse_case_token(case_tok, user_case)) {
            cur_case = user_case;
            float user_px = cur_px.x();
            float user_py = cur_px.y();
            bool has_px = false;
            bool has_py = false;
            ss >> user_px;
            if (ss) has_px = true;
            if (has_px) {
              ss >> user_py;
              if (ss) has_py = true;
            }

            if (has_px && has_py) {
              cur_px = Eigen::Vector2f(user_px, user_py);
            } else {
              // 若只给了一个数或都没给，则保持预设 px/py 不动
              switch (cur_case) {
                case CameraID::NONE: cur_px = pixel_none; break;
                case CameraID::BOTH: cur_px = pixel_both; break;
                case CameraID::TELE: cur_px = pixel_tele; break;
                case CameraID::WIDE: cur_px = pixel_wide; break;
                default: cur_px = pixel_none; break;
              }
            }
          } else {
            ROS_WARN("无法解析 case_token=%s，使用预设。", case_tok.c_str());
          }
        }
      }

    }

    searcher.reset();
    searcher.set_search_strategy(cur_case, cur_px);
    ROS_INFO("Start scenario lost_cam_id=%d px=%.3f py=%.3f for %.2f s", static_cast<int>(cur_case),
             cur_px.x(), cur_px.y(), scenario_duration_s);

    TaskResult last_res;
    bool failed = false;
    float yaw_rate_min = 1e9f, yaw_rate_max = -1e9f;

    double sc_start = ros::Time::now().toSec();
    scenario_active.store(true);
    while (ros::ok() && ros::Time::now().toSec() - sc_start < scenario_duration_s) {
      TaskResult res = searcher.update(dt);
      last_res = res;

      yaw_rate_min = std::min(yaw_rate_min, res.yaw_rate);
      yaw_rate_max = std::max(yaw_rate_max, res.yaw_rate);

      publish_mount_from_att(res.att_cmd);
      publish_body_from_result(res);

      if (res.status == TaskStatus::FAILED) failed = true;
      ros::spinOnce();
      r.sleep();
    }
    scenario_active.store(false);

    ROS_INFO("Scenario lost_cam_id=%d done: status=%d failed=%d yaw_rate[min,max]=[%.3f,%.3f]",
             static_cast<int>(cur_case), static_cast<int>(last_res.status), failed, yaw_rate_min,
             yaw_rate_max);
  }

  // 归零：停止速度/Yaw_rate，同时保持 mount 不动一小段
  ROS_INFO("Scenario finished: publish zero velocity...");
  for (int i = 0; ros::ok() && i < 20; ++i) {
    // 保持当前云台姿态（用 gimbal_pose 四元数换算 RPY）
    const Eigen::Vector3f att_hold = quaternionToEuler(dm.getGimbalPose().q);
    publish_mount_from_att(att_hold);
    mavros_msgs::PositionTarget pt;
    pt.header.stamp = ros::Time::now();
    pt.header.frame_id = "map";
    pt.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    pt.type_mask = mavros_msgs::PositionTarget::IGNORE_PX | mavros_msgs::PositionTarget::IGNORE_PY |
                    mavros_msgs::PositionTarget::IGNORE_PZ | mavros_msgs::PositionTarget::IGNORE_AFX |
                    mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ |
                    mavros_msgs::PositionTarget::IGNORE_YAW;
    pt.velocity.x = 0.0;
    pt.velocity.y = 0.0;
    pt.velocity.z = 0.0;
    pt.yaw_rate = 0.0;
    pub_setpoint_raw.publish(pt);
    ros::spinOnce();
    ros::Duration(0.05).sleep();
  }

  ROS_INFO("target_searcher_tester exiting.");
  stop_hold_thread.store(true);
  if (hold_thread.joinable()) hold_thread.join();
  return 0;
}

