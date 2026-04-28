#include <boost/bind.hpp>
#include <clocale>
#include <cmath>
#include <cctype>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <thread>
#include <mutex>
#include <queue>
#include <unistd.h>

#include <ros/ros.h>

#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/MountControl.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/HomePosition.h>

#include <ros_drone/DataManager.h>
#include <target_searcher/TargetSearcher.h>
#include <ros_drone/util.h>

static bool isFiniteVec3(const Eigen::Vector3f& v) {
  return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}
static bool isFiniteVec4(const Eigen::Vector4f& v) {
  return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z()) && std::isfinite(v.w());
}

static mavros_msgs::PositionTarget makeVelocitySetpoint(const TaskResult& out) {
  mavros_msgs::PositionTarget pt;
  pt.header.stamp = ros::Time::now();
  pt.header.frame_id = "map";
  pt.type_mask =
      mavros_msgs::PositionTarget::IGNORE_PX |
      mavros_msgs::PositionTarget::IGNORE_PY |
      mavros_msgs::PositionTarget::IGNORE_PZ |
      mavros_msgs::PositionTarget::IGNORE_AFX |
      mavros_msgs::PositionTarget::IGNORE_AFY |
      mavros_msgs::PositionTarget::IGNORE_AFZ |
      mavros_msgs::PositionTarget::IGNORE_YAW;

  if (out.velocity_frame == TaskResult::VelocityFrame::ENU) {
    // 这里保持 ROS 侧 ENU 轴语义直接发布，避免与 mavros 内部坐标变换叠加导致轴错位
    pt.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    pt.velocity.x = out.vel_vxyzy.x();
    pt.velocity.y = out.vel_vxyzy.y();
    pt.velocity.z = out.vel_vxyzy.z();
    pt.yaw_rate = out.vel_vxyzy.w();
    return pt;
  }

  pt.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
  // FRAME_BODY_NED：z 向下为正；与原先机体系一致
  pt.velocity.x = out.vel_vxyzy.x();
  pt.velocity.y = out.vel_vxyzy.y();
  pt.velocity.z = out.vel_vxyzy.z();
  pt.yaw_rate = out.vel_vxyzy.w();
  return pt;
}

static mavros_msgs::MountControl makeGimbalCmdDeg(const Eigen::Vector3f& rpy_rad) {
  mavros_msgs::MountControl mc;
  mc.header.stamp = ros::Time::now();
  mc.mode = mavros_msgs::MountControl::MAV_MOUNT_MODE_MAVLINK_TARGETING;
  mc.pitch = rad2deg_normalized(rpy_rad.y());
  mc.roll = rad2deg_normalized(rpy_rad.x());
  mc.yaw = rad2deg_normalized(rpy_rad.z());
  return mc;
}

static bool waitForLocalPose(const std::string& ns, double timeout_s) {
  const ros::Time t0 = ros::Time::now();
  while (ros::ok()) {
    const auto lp = DataManager::getInstance().getLocalPos();
    if (lp.rcv_stamp != ros::Time(0)) {
      return true;
    }
    if ((ros::Time::now() - t0).toSec() > timeout_s) {
      ROS_ERROR("[%s] 超时未收到 local_position/pose", ns.c_str());
      return false;
    }
    ROS_INFO_THROTTLE(2.0, "[%s] 等待 local_position/pose ...", ns.c_str());
    ros::Duration(0.1).sleep();
  }
  return false;
}

static bool waitConnected(const std::string& ns, double timeout_s) {
  const double t0 = ros::Time::now().toSec();
  while (ros::ok()) {
    if (DataManager::getInstance().is_connected()) return true;
    if (ros::Time::now().toSec() - t0 > timeout_s) {
      ROS_ERROR("[%s] MAVROS 超时未连接", ns.c_str());
      return false;
    }
    ROS_INFO_THROTTLE(3.0, "[%s] 等待 MAVROS 连接...", ns.c_str());
    ros::Duration(0.2).sleep();
  }
  return false;
}

static bool callSetMode(const std::string& ns_mavros, const std::string& mode, double timeout_s) {
  ros::NodeHandle nh;
  const std::string srv_name = ns_mavros + "/set_mode";
  if (!ros::service::waitForService(srv_name, ros::Duration(timeout_s))) {
    ROS_ERROR("等待服务失败: %s", srv_name.c_str());
    return false;
  }
  ros::ServiceClient client = nh.serviceClient<mavros_msgs::SetMode>(srv_name);
  mavros_msgs::SetMode req;
  req.request.custom_mode = mode;
  if (!client.call(req) || !req.response.mode_sent) {
    ROS_ERROR("切换模式失败: %s", mode.c_str());
    return false;
  }
  ROS_INFO("模式已请求切换: %s", mode.c_str());
  return true;
}

static bool callArm(const std::string& ns_mavros, bool arm, double timeout_s) {
  ros::NodeHandle nh;
  const std::string srv_name = ns_mavros + "/cmd/arming";
  if (!ros::service::waitForService(srv_name, ros::Duration(timeout_s))) {
    ROS_ERROR("等待服务失败: %s", srv_name.c_str());
    return false;
  }
  ros::ServiceClient client = nh.serviceClient<mavros_msgs::CommandBool>(srv_name);
  mavros_msgs::CommandBool req;
  req.request.value = arm;
  if (!client.call(req) || !req.response.success) {
    ROS_ERROR("解锁/上锁失败: %d", arm ? 1 : 0);
    return false;
  }
  ROS_INFO("解锁状态已请求: %d", arm ? 1 : 0);
  return true;
}

// 单相机版：只解析归一化像素 (x y)。真实飞控链路里像素由视觉节点写入 DataManager::set_pixel_norm；
// 本测试程序在调用 set_search_strategy() 前主动写入，否则 DataManager 内无像素数据。
static bool parsePixelLine(const std::string& line, double* px_out, double* py_out, std::string* err_out) {
  std::istringstream iss(line);
  double px = 0.0;
  double py = 0.0;
  if (!(iss >> px >> py)) {
    if (err_out) {
      *err_out = "格式应为: <pixel_x> <pixel_y>  例: 1 0（靠边离开，多为 WIDE）  0.2 0（离边远，多为 TELE）";
    }
    return false;
  }
  std::string extra;
  if (iss >> extra) {
    if (err_out) *err_out = "多余字段；只需两个数字: pixel_x pixel_y";
    return false;
  }
  *px_out = px;
  *py_out = py;
  return true;
}

static std::string trimCopy(const std::string& s) {
  const auto a = s.find_first_not_of(" \t\r\n");
  if (a == std::string::npos) {
    return "";
  }
  const auto b = s.find_last_not_of(" \t\r\n");
  return s.substr(a, b - a + 1);
}

namespace {
std::queue<std::string> g_cmd_q;
std::mutex g_cmd_mtx;
}  // namespace

static void stdinToQueueThread() {
  std::string line;
  while (std::getline(std::cin, line)) {
    if (!ros::ok()) {
      break;
    }
    std::lock_guard<std::mutex> lk(g_cmd_mtx);
    g_cmd_q.push(std::move(line));
  }
}

static bool readInteractivePixel(double* lost_pixel_x, double* lost_pixel_y) {
  std::cerr << "\n[target_searcher_tester] 请输入归一化像素 (pixel_x pixel_y)（将写入 DataManager::set_pixel_norm）\n"
               "  例: 1 0    或    0.2 0\n"
               "> " << std::flush;
  std::string line;
  while (std::getline(std::cin, line)) {
    const auto start = line.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) {
      std::cerr << "> " << std::flush;
      continue;
    }
    const auto end = line.find_last_not_of(" \t\r\n");
    line = line.substr(start, end - start + 1);
    if (line.empty()) {
      std::cerr << "> " << std::flush;
      continue;
    }
    std::string err;
    if (parsePixelLine(line, lost_pixel_x, lost_pixel_y, &err)) {
      return true;
    }
    std::cerr << "[错误] " << err << "\n> " << std::flush;
  }
  return false;
}

int main(int argc, char** argv) {
  setlocale(LC_ALL, "");
  ros::init(argc, argv, "target_searcher_pkg_tester");
  ros::NodeHandle pnh("~");
  {
    ros::NodeHandle nh_wide(pnh, "target_searcher/wide");
    if (!nh_wide.hasParam("enable_task1_decel")) {
      ROS_WARN(
          "未检测到 ~/target_searcher/wide/*（例如 enable_task1_decel）。"
          "请使用: roslaunch target_searcher target_searcher_tester.launch（或自行 rosparam load 本包 config/target_searcher.yaml）");
    }
  }

  // =============== 参数 ===============
  std::string uav_prefix{"/uav0"};
  pnh.param("uav_prefix", uav_prefix, uav_prefix);

  bool arm_and_offboard{true};
  bool send_velocity{true};
  bool send_gimbal{true};
  pnh.param("arm_and_offboard", arm_and_offboard, arm_and_offboard);
  pnh.param("send_velocity", send_velocity, send_velocity);
  pnh.param("send_gimbal", send_gimbal, send_gimbal);

  double loop_hz{50.0};
  double scenario_duration_s{10.0};
  pnh.param("loop_hz", loop_hz, loop_hz);
  pnh.param("scenario_duration_s", scenario_duration_s, scenario_duration_s);

  // session_mode：悬停 → 终端下任务 → return 回初始点 → 可重复；否则为单次测试 + 可选 hold_after_test
  bool session_mode{true};
  pnh.param("session_mode", session_mode, session_mode);
  double return_pos_tol_m{0.25};
  double return_z_tol_m{0.15};
  pnh.param("return_pos_tol_m", return_pos_tol_m, return_pos_tol_m);
  pnh.param("return_z_tol_m", return_z_tol_m, return_z_tol_m);

  // 测试结束后是否持续发位置保持（仅 !session_mode 时有效）
  bool hold_after_test{true};
  pnh.param("hold_after_test", hold_after_test, hold_after_test);

  // 起飞：先到指定 ENU 高度再进入 TargetSearcher（默认 1 m）
  bool enable_takeoff{true};
  double takeoff_altitude_m{1.0};
  double takeoff_alt_tolerance_m{0.15};
  double takeoff_timeout_s{60.0};
  int takeoff_settle_cycles{25};
  pnh.param("enable_takeoff", enable_takeoff, enable_takeoff);
  pnh.param("takeoff_altitude_m", takeoff_altitude_m, takeoff_altitude_m);
  pnh.param("takeoff_alt_tolerance_m", takeoff_alt_tolerance_m, takeoff_alt_tolerance_m);
  pnh.param("takeoff_timeout_s", takeoff_timeout_s, takeoff_timeout_s);
  pnh.param("takeoff_settle_cycles", takeoff_settle_cycles, takeoff_settle_cycles);

  // 丢失方向（归一化像素坐标，[-1,1] 大致范围）；单次非交互模式从参数读入并写入 DataManager
  double lost_pixel_x{1.0};
  double lost_pixel_y{0.0};
  pnh.param("lost_pixel_x", lost_pixel_x, lost_pixel_x);
  pnh.param("lost_pixel_y", lost_pixel_y, lost_pixel_y);

  // 单次模式：启动时读 pixel；session_mode 下由悬停后终端命令给出 pixel
  bool interactive_default = (isatty(STDIN_FILENO) != 0);
  bool interactive = interactive_default;
  pnh.param("interactive", interactive, interactive);

  if (!session_mode) {
    if (interactive) {
      if (!readInteractivePixel(&lost_pixel_x, &lost_pixel_y)) {
        ROS_ERROR("未读到有效输入，退出。");
        return 1;
      }
    }
  } else {
    ROS_INFO("session_mode=true：起飞后在初始点悬停；终端输入 pixel_x pixel_y，return 回原点，可多次执行。");
  }

  const std::string ns = uav_prefix;
  const std::string ns_mavros = uav_prefix + "/mavros";

  ROS_INFO("target_searcher 包测试节点（单相机版，WIDE/TELE 由 yaml edge_distance_threshold 自动判定）:");
  ROS_INFO("  uav_prefix=%s", uav_prefix.c_str());
  ROS_INFO("  session_mode=%d scenario_duration_s=%.1f", session_mode ? 1 : 0, scenario_duration_s);
  ROS_INFO("  arm_and_offboard=%d send_velocity=%d send_gimbal=%d", arm_and_offboard ? 1 : 0, send_velocity ? 1 : 0, send_gimbal ? 1 : 0);
  if (!session_mode) {
    ROS_INFO("  单次模式 lost_pixel=(%.2f, %.2f)（将 set_pixel_norm 后 set_search_strategy）", lost_pixel_x, lost_pixel_y);
    ROS_INFO("  hold_after_test=%d", hold_after_test ? 1 : 0);
  }

  // =============== 数据订阅喂入 DataManager（与 capturer_node 类似） ===============
  DataManager& dm = DataManager::getInstance();
  ros::AsyncSpinner spinner(3);
  spinner.start();

  // state / home / local / vel / imu / gimbal_pose
  auto sub_state = pnh.subscribe<mavros_msgs::State>(ns_mavros + "/state", 10,
                                                    boost::bind(&DataManager::feed_state, &dm, _1));
  auto sub_home = pnh.subscribe<mavros_msgs::HomePosition>(ns_mavros + "/home_position/home", 10,
                                                           boost::bind(&DataManager::feed_home_pos, &dm, _1));
  auto sub_local = pnh.subscribe<geometry_msgs::PoseStamped>(ns_mavros + "/local_position/pose", 10,
                                                             boost::bind(&DataManager::feed_local_pos, &dm, _1));
  auto sub_vel_body = pnh.subscribe<geometry_msgs::TwistStamped>(ns_mavros + "/local_position/velocity_body", 10,
                                                                 boost::bind(&DataManager::feed_vel_body, &dm, _1));
  auto sub_vel_enu = pnh.subscribe<geometry_msgs::TwistStamped>(ns_mavros + "/local_position/velocity_local", 10,
                                                                boost::bind(&DataManager::feed_vel_enu, &dm, _1));
  auto sub_imu = pnh.subscribe<sensor_msgs::Imu>(ns_mavros + "/imu/data", 10,
                                                 boost::bind(&DataManager::feed_imu, &dm, _1));
  auto sub_gimbal = pnh.subscribe<nav_msgs::Odometry>(ns_mavros + "/gimbal_pose", 10,
                                                      boost::bind(&DataManager::feed_gimbal_pose, &dm, _1));
  (void)sub_state;
  (void)sub_home;
  (void)sub_local;
  (void)sub_vel_body;
  (void)sub_vel_enu;
  (void)sub_imu;
  (void)sub_gimbal;

  if (!waitConnected(ns, 120.0)) return 1;
  if (!waitForLocalPose(ns, 60.0)) return 1;

  // =============== 发布器 ===============
  ros::Publisher pub_raw = pnh.advertise<mavros_msgs::PositionTarget>(ns_mavros + "/setpoint_raw/local", 20);
  ros::Publisher pub_pos = pnh.advertise<geometry_msgs::PoseStamped>(ns_mavros + "/setpoint_position/local", 20);
  ros::Publisher pub_mount = pnh.advertise<mavros_msgs::MountControl>(ns_mavros + "/mount_control/command", 20);

  TargetSearcher searcher(pnh);

  ros::Rate rate(std::max(5.0, loop_hz));

  const auto lp_start = DataManager::getInstance().getLocalPos();

  auto make_hold_pose_xy_z = [&](double z_enu) -> geometry_msgs::PoseStamped {
    geometry_msgs::PoseStamped sp;
    sp.header.stamp = ros::Time::now();
    sp.header.frame_id = "map";
    sp.pose.position.x = static_cast<double>(lp_start.p.x());
    sp.pose.position.y = static_cast<double>(lp_start.p.y());
    sp.pose.position.z = z_enu;
    sp.pose.orientation = lp_start.msg.pose.orientation;
    return sp;
  };

  // OFFBOARD 前预热：发位置设定值（与后续起飞同一话题，避免 PX4 指令类型跳变）
  const int warmup_n = 60;
  for (int i = 0; ros::ok() && i < warmup_n; ++i) {
    pub_pos.publish(make_hold_pose_xy_z(static_cast<double>(lp_start.p.z())));
    rate.sleep();
  }

  if (arm_and_offboard) {
    if (!callSetMode(ns_mavros, "OFFBOARD", 10.0)) return 2;
    if (!callArm(ns_mavros, true, 10.0)) return 3;
  } else {
    ROS_WARN("arm_and_offboard=false：不会自动切 OFFBOARD/解锁，仅发布指令（若飞控未接管，机体不会动）。");
  }

  // 爬升到 takeoff_altitude_m 后再切入搜索（云台“丢失瞬间”基准也在空中记录）
  if (arm_and_offboard && enable_takeoff) {
    ROS_INFO("起飞至 %.2f m（ENU z，容差 %.2f m）...", takeoff_altitude_m, takeoff_alt_tolerance_m);
    const double t_takeoff0 = ros::Time::now().toSec();
    geometry_msgs::PoseStamped sp_climb = make_hold_pose_xy_z(takeoff_altitude_m);
    while (ros::ok()) {
      sp_climb.header.stamp = ros::Time::now();
      pub_pos.publish(sp_climb);
      const auto lp = DataManager::getInstance().getLocalPos();
      if (static_cast<double>(lp.p.z()) >= takeoff_altitude_m - takeoff_alt_tolerance_m) {
        ROS_INFO("已到达目标高度附近: z=%.2f m", static_cast<double>(lp.p.z()));
        break;
      }
      if (ros::Time::now().toSec() - t_takeoff0 > takeoff_timeout_s) {
        ROS_ERROR("起飞超时（> %.1f s），当前 z=%.2f m", takeoff_timeout_s, static_cast<double>(lp.p.z()));
        return 4;
      }
      rate.sleep();
    }
    for (int i = 0; ros::ok() && i < takeoff_settle_cycles; ++i) {
      sp_climb.header.stamp = ros::Time::now();
      pub_pos.publish(sp_climb);
      rate.sleep();
    }
  } else if (enable_takeoff && !arm_and_offboard) {
    ROS_WARN("enable_takeoff=true 但 arm_and_offboard=false，跳过起飞。");
  }

  geometry_msgs::PoseStamped pose_initial;
  pose_initial.header.frame_id = "map";
  {
    const auto lp0 = DataManager::getInstance().getLocalPos();
    pose_initial.pose.position.x = static_cast<double>(lp0.p.x());
    pose_initial.pose.position.y = static_cast<double>(lp0.p.y());
    pose_initial.pose.position.z = static_cast<double>(lp0.p.z());
    pose_initial.pose.orientation = lp0.msg.pose.orientation;
  }
  ROS_INFO(
      "初始悬停点已记录: x=%.2f y=%.2f z=%.2f m（return 将回到此处）",
      pose_initial.pose.position.x,
      pose_initial.pose.position.y,
      pose_initial.pose.position.z);

  if (session_mode) {
    enum class SessionState { IDLE_HOVER, RUN_SEARCH, RETURN_HOME };
    std::thread(stdinToQueueThread).detach();

    geometry_msgs::PoseStamped pose_hold = pose_initial;
    SessionState state = SessionState::IDLE_HOVER;
    double t_search0 = 0.0;
    double t_last = ros::Time::now().toSec();

    ROS_INFO(
        ">>> 悬停就绪。命令:  <pixel_x> <pixel_y>（写入 DataManager 后自动选 WIDE/TELE）| return / home / r | quit / q | help / h");

    while (ros::ok()) {
      std::string cmd_in;
      {
        std::lock_guard<std::mutex> lk(g_cmd_mtx);
        if (!g_cmd_q.empty()) {
          cmd_in = std::move(g_cmd_q.front());
          g_cmd_q.pop();
        }
      }

      if (!cmd_in.empty() && state == SessionState::IDLE_HOVER) {
        const std::string t = trimCopy(cmd_in);
        std::string tl = t;
        for (auto& c : tl) {
          c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        }
        if (!tl.empty()) {
          if (tl == "quit" || tl == "exit" || tl == "q") {
            ros::shutdown();
          } else if (tl == "help" || tl == "h") {
            ROS_INFO(
                "<pixel_x> <pixel_y> — 执行搜索（先 set_pixel_norm），最长 %.1f s；return/home/r — 回初始悬停点；quit — 退出",
                scenario_duration_s);
          } else if (tl == "return" || tl == "home" || tl == "r") {
            const auto lp_now = DataManager::getInstance().getLocalPos();
            const double ex = static_cast<double>(lp_now.p.x()) - pose_initial.pose.position.x;
            const double ey = static_cast<double>(lp_now.p.y()) - pose_initial.pose.position.y;
            const double ez = static_cast<double>(lp_now.p.z()) - pose_initial.pose.position.z;
            if (std::hypot(ex, ey) < return_pos_tol_m && std::abs(ez) < return_z_tol_m) {
              ROS_INFO("已在初始点附近，无需移动。");
            } else {
              state = SessionState::RETURN_HOME;
              ROS_INFO("正在回到初始悬停点 ...");
            }
          } else {
            double px = 0.0;
            double py = 0.0;
            std::string err;
            if (parsePixelLine(t, &px, &py, &err)) {
              DataManager::getInstance().set_pixel_norm(Eigen::Vector2f(static_cast<float>(px), static_cast<float>(py)));
              searcher.reset();
              searcher.set_search_strategy();
              state = SessionState::RUN_SEARCH;
              t_search0 = ros::Time::now().toSec();
              t_last = t_search0;
              ROS_INFO("已 set_pixel_norm(%.2f, %.2f) 并切入搜索，最长 %.1f s（WIDE/TELE 由参数阈值决定）", px, py,
                       scenario_duration_s);
            } else {
              ROS_WARN("无效命令 \"%s\"：%s", t.c_str(), err.c_str());
            }
          }
        }
      }

      const double t_now = ros::Time::now().toSec();
      const double dt = std::max(1e-3, t_now - t_last);
      t_last = t_now;

      switch (state) {
        case SessionState::IDLE_HOVER:
          pose_hold.header.stamp = ros::Time::now();
          pub_pos.publish(pose_hold);
          break;
        case SessionState::RETURN_HOME: {
          geometry_msgs::PoseStamped sp_ret = pose_initial;
          sp_ret.header.stamp = ros::Time::now();
          pub_pos.publish(sp_ret);
          const auto lp = DataManager::getInstance().getLocalPos();
          const double ex = static_cast<double>(lp.p.x()) - pose_initial.pose.position.x;
          const double ey = static_cast<double>(lp.p.y()) - pose_initial.pose.position.y;
          const double ez = static_cast<double>(lp.p.z()) - pose_initial.pose.position.z;
          if (std::hypot(ex, ey) < return_pos_tol_m && std::abs(ez) < return_z_tol_m) {
            state = SessionState::IDLE_HOVER;
            pose_hold = pose_initial;
            ROS_INFO("已回到初始悬停点。可再次输入任务。");
          }
          break;
        }
        case SessionState::RUN_SEARCH: {
          const TaskResult out = searcher.update(static_cast<float>(dt));
          const bool vel_ok = isFiniteVec4(out.vel_vxyzy);
          const bool gim_ok = isFiniteVec3(out.gimbal_rpy);
          if (!vel_ok || !gim_ok || (t_now - t_search0) > scenario_duration_s) {
            if (!vel_ok || !gim_ok) {
              ROS_WARN("TargetSearcher 输出无效/超时(INF)，直接执行 return 回初始悬停点。");
            } else {
              ROS_INFO("本段任务已达时长上限（%.1f s）。", scenario_duration_s);
            }
            searcher.reset();
            if (!vel_ok || !gim_ok) {
              state = SessionState::RETURN_HOME;
              ROS_INFO("正在回到初始悬停点 ...");
            } else {
              state = SessionState::IDLE_HOVER;
              const auto lp = DataManager::getInstance().getLocalPos();
              pose_hold.header.frame_id = "map";
              pose_hold.pose.position.x = static_cast<double>(lp.p.x());
              pose_hold.pose.position.y = static_cast<double>(lp.p.y());
              pose_hold.pose.position.z = static_cast<double>(lp.p.z());
              pose_hold.pose.orientation = lp.msg.pose.orientation;
              ROS_INFO("已在当前位置悬停。输入 return 回初始点，或直接输入下一段任务。");
            }
            break;
          }
          if (send_velocity) {
            pub_raw.publish(makeVelocitySetpoint(out));
          }
          if (send_gimbal) {
            pub_mount.publish(makeGimbalCmdDeg(out.gimbal_rpy));
          }
          break;
        }
      }

      rate.sleep();
    }

    ROS_INFO("target_searcher_pkg_tester finished.");
    return 0;
  }

  // =============== 单次模式：一条任务后退出或 hold ===============
  searcher.reset();
  DataManager::getInstance().set_pixel_norm(
      Eigen::Vector2f(static_cast<float>(lost_pixel_x), static_cast<float>(lost_pixel_y)));
  searcher.set_search_strategy();
  ROS_INFO("已进入 TargetSearcher 单次阶段（scenario_duration_s=%.1f）。", scenario_duration_s);

  const double t_start = ros::Time::now().toSec();
  double t_last = t_start;
  bool ended_by_inf = false;

  while (ros::ok()) {
    const double t_now = ros::Time::now().toSec();
    const double dt = std::max(1e-3, t_now - t_last);
    t_last = t_now;

    const TaskResult out = searcher.update(static_cast<float>(dt));

    const bool vel_ok = isFiniteVec4(out.vel_vxyzy);
    const bool gim_ok = isFiniteVec3(out.gimbal_rpy);

    if (!vel_ok || !gim_ok) {
      ROS_WARN_THROTTLE(1.0, "TargetSearcher 超时输出 INF（认为搜索结束/放弃）。");
      ended_by_inf = true;
      break;
    }

    if (send_velocity) {
      pub_raw.publish(makeVelocitySetpoint(out));
    }
    if (send_gimbal) {
      pub_mount.publish(makeGimbalCmdDeg(out.gimbal_rpy));
    }

    if (t_now - t_start > scenario_duration_s) {
      ROS_INFO("测试到达 scenario_duration_s=%.2f，退出。", scenario_duration_s);
      break;
    }

    rate.sleep();
  }

  // 单次模式：若因 INF 退出，则直接执行 return 回初始悬停点
  if (arm_and_offboard && ended_by_inf) {
    ROS_WARN("单次模式：检测到 INF，开始 return 回初始悬停点 ...");
    while (ros::ok()) {
      geometry_msgs::PoseStamped sp_ret = pose_initial;
      sp_ret.header.stamp = ros::Time::now();
      pub_pos.publish(sp_ret);
      const auto lp = DataManager::getInstance().getLocalPos();
      const double ex = static_cast<double>(lp.p.x()) - pose_initial.pose.position.x;
      const double ey = static_cast<double>(lp.p.y()) - pose_initial.pose.position.y;
      const double ez = static_cast<double>(lp.p.z()) - pose_initial.pose.position.z;
      if (std::hypot(ex, ey) < return_pos_tol_m && std::abs(ez) < return_z_tol_m) {
        ROS_INFO("单次模式：已回到初始悬停点。");
        break;
      }
      rate.sleep();
    }
  }

  if (arm_and_offboard && hold_after_test) {
    const auto lp_h = DataManager::getInstance().getLocalPos();
    geometry_msgs::PoseStamped sp_h;
    sp_h.header.frame_id = "map";
    sp_h.pose.position.x = static_cast<double>(lp_h.p.x());
    sp_h.pose.position.y = static_cast<double>(lp_h.p.y());
    sp_h.pose.position.z = static_cast<double>(lp_h.p.z());
    sp_h.pose.orientation = lp_h.msg.pose.orientation;
    ROS_INFO(
        "单次模式：持续发布当前位姿 position setpoint 直到 Ctrl+C（避免 OFFBOARD 断流降落）。");
    while (ros::ok()) {
      sp_h.header.stamp = ros::Time::now();
      pub_pos.publish(sp_h);
      rate.sleep();
    }
  }

  ROS_INFO("target_searcher_pkg_tester finished.");
  return 0;
}

