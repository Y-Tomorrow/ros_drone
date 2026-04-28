/**
 * @file StatusMonitor.h
 * @brief 无人机状态监控器，纯数据大管家（无 ROS 节点依赖，单例模式）
 */
#pragma once

#include <ros/ros.h>
#include <mutex>
#include <string>
#include <deque>
#include <array>
#include <Eigen/Dense>

// 仅保留必需的 ROS Msgs（本仓库内缺失 capturer_node/TargetInfo，自适应剥离相机像素链路）
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/HomePosition.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

// ==========================================
// 数据结构体定义区 (纯粹的数据容器封装)
// ==========================================
struct ControlCommand {
    Eigen::Vector4f vel=Eigen::Vector4f::Zero(); // (v_x, v_y, v_z, yaw_rate)
    Eigen::Vector3f gimbal_attitude=Eigen::Vector3f::Zero(); // (roll, pitch, yaw)
};
enum class CameraID {
    NONE,
    TELE,
    WIDE,
    BOTH
};

class GimbalPoseData_t {
public:
    nav_msgs::Odometry msg;
    Eigen::Quaternionf q;
    ros::Time rcv_stamp;

    GimbalPoseData_t();
    void feed(const nav_msgs::Odometry::ConstPtr& pMsg);
};

class StateData_t {
public:
    mavros_msgs::State msg;
    bool connected;
    bool armed;
    std::string mode;
    ros::Time rcv_stamp;

    StateData_t();
    void feed(const mavros_msgs::State::ConstPtr& pMsg);
};

class HomePosData_t {
public:
    mavros_msgs::HomePosition msg;
    Eigen::Vector3f p;
    Eigen::Quaternionf q;
    float yaw;
    ros::Time rcv_stamp;

    HomePosData_t();
    void feed(const mavros_msgs::HomePosition::ConstPtr& pMsg);
};

class LocalPosData_t {
public:
    geometry_msgs::PoseStamped msg;
    Eigen::Vector3f p;
    Eigen::Quaternionf q;
    float yaw;
    ros::Time rcv_stamp;

    LocalPosData_t();
    void feed(const geometry_msgs::PoseStamped::ConstPtr& pMsg);
};

class VelData_t { 
public:
    geometry_msgs::TwistStamped msg;
    Eigen::Vector3f v;
    float yaw_rate;
    ros::Time rcv_stamp;

    VelData_t();
    void feed(const geometry_msgs::TwistStamped::ConstPtr& pMsg);
};

class ImuData_t {
public:
    sensor_msgs::Imu msg;
    Eigen::Vector3f w; // Angular velocity
    Eigen::Vector3f a; // Linear acceleration
    Eigen::Quaternionf q;
    float roll, pitch, yaw; // Euler angles in ENU frame
    ros::Time rcv_stamp;

    ImuData_t();
    void feed(const sensor_msgs::Imu::ConstPtr& pMsg);
};

// ==========================================
// 状态监控核心类 (单例模式)
// ==========================================

class DataManager {
public:
    // 获取唯一的单例实例
    static DataManager& getInstance();
    
    // 禁用拷贝与赋值，保证纯正单例
    DataManager(const DataManager&) = delete;
    DataManager& operator=(const DataManager&) = delete;

    // --- Getter 接口 (按值返回拷贝，脱离锁后绝对内存安全) ---
    GimbalPoseData_t getGimbalPose() const;
    StateData_t      getState() const;
    HomePosData_t    getHomePos() const;
    LocalPosData_t   getLocalPos() const;
    VelData_t        getVelBody() const;
    VelData_t        getVelEnu() const;
    ImuData_t        getImu() const;

    // --- 业务状态快速查询 ---
    bool is_connected() const;
    bool is_armed() const;
    bool is_ready_for_fly() const;
    std::string get_flight_mode() const;
    // 获取同步后的飞控姿态四元数
    Eigen::Quaternionf get_imu_orientation() const;
    Eigen::Quaternionf get_gimbal_orientation() const;
    // 单相机跟踪：归一化像素误差 (x,y)，由视觉/跟踪模块写入
    Eigen::Vector2f get_pixel_norm() const;
    void set_pixel_norm(const Eigen::Vector2f& p);
    // --- 公开的数据喂入接口 (供节点使用 boost::bind 绑定) ---
    void feed_gimbal_pose(const nav_msgs::Odometry::ConstPtr& msg);
    void feed_state(const mavros_msgs::State::ConstPtr& msg);
    void feed_home_pos(const mavros_msgs::HomePosition::ConstPtr& msg);
    void feed_local_pos(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void feed_vel_body(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void feed_vel_enu(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void feed_imu(const sensor_msgs::Imu::ConstPtr& msg);

    // --- 缓冲与同步查询接口 (保留原有的时间戳同步逻辑) ---
    // OmegaStatus        get_synced_omega() const;


private:
    DataManager() = default;
    ~DataManager() = default;

    // 线程安全互斥锁
    mutable std::mutex _mutex;
    // --- 数据实体 (核心数据源) ---
    GimbalPoseData_t _gimbal_data;
    StateData_t      _state_data;
    HomePosData_t    _home_data;
    LocalPosData_t   _local_pos_data;
    VelData_t        _vel_body_data;
    VelData_t        _vel_enu_data;
    ImuData_t        _imu_data;
    Eigen::Vector2f  _pixel_norm{Eigen::Vector2f::Zero()};
    // --- 缓冲区结构与数据 (保留用于插值) ---
    struct OmegaBufItem { double timestamp; Eigen::Vector3f w; };
    std::deque<OmegaBufItem> _omega_buffer;
    struct OriBufItem { double timestamp; Eigen::Quaternionf q; };
    // 飞控姿态四元数缓冲区
    std::deque<OriBufItem> _imu_orientation_buffer;
    // 云台姿态四元数缓冲区
    std::deque<OriBufItem> _gimbal_orientation_buffer;
    const double MAX_BUFFER_AGE = 0.5;
private:
    // --- 缓冲区时间同步与插值计算 ---
    Eigen::Quaternionf get_synced_orientation(const std::deque<OriBufItem>& buffer) const;
};