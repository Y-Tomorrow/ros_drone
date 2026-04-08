/**
 * @file DataManager.cpp
 */
#include "DataManager.h"
#include "utils.h"
#include <cmath>

// ==========================================
// _Data_t 结构体 Feed() 具体解析实现区
// ==========================================

GimbalPoseData_t::GimbalPoseData_t() : rcv_stamp(ros::Time(0)) { q.setIdentity(); }
void GimbalPoseData_t::feed(const nav_msgs::Odometry::ConstPtr& pMsg) {
    msg = *pMsg;
    rcv_stamp = ros::Time::now();
    q = Eigen::Quaternionf(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
                           msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);
}

StateData_t::StateData_t() : rcv_stamp(ros::Time(0)), connected(false), armed(false), mode("UNKNOWN") {}
void StateData_t::feed(const mavros_msgs::State::ConstPtr& pMsg) {
    msg = *pMsg;
    rcv_stamp = ros::Time::now();
    connected = msg.connected;
    armed = msg.armed;
    mode = msg.mode;
}

HomePosData_t::HomePosData_t() : rcv_stamp(ros::Time(0)), p(Eigen::Vector3f::Zero()), yaw(0.0f) { q.setIdentity(); }
void HomePosData_t::feed(const mavros_msgs::HomePosition::ConstPtr& pMsg) {
    msg = *pMsg;
    rcv_stamp = ros::Time::now();
    p << msg.position.x, msg.position.y, msg.position.z;
    q = Eigen::Quaternionf(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
    yaw = std::atan2(2.0f * (q.w() * q.z() + q.x() * q.y()), 1.0f - 2.0f * (q.y() * q.y() + q.z() * q.z())) * 180.0f / M_PI;
}

LocalPosData_t::LocalPosData_t() : rcv_stamp(ros::Time(0)), p(Eigen::Vector3f::Zero()), yaw(0.0f) { q.setIdentity(); }
void LocalPosData_t::feed(const geometry_msgs::PoseStamped::ConstPtr& pMsg) {
    msg = *pMsg;
    rcv_stamp = ros::Time::now();
    p << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
    q = Eigen::Quaternionf(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z);
    yaw = std::atan2(2.0f * (q.w() * q.z() + q.x() * q.y()), 1.0f - 2.0f * (q.y() * q.y() + q.z() * q.z())) * 180.0f / M_PI;
}

VelData_t::VelData_t() : rcv_stamp(ros::Time(0)), v(Eigen::Vector3f::Zero()), yaw_rate(0.0f) {}
void VelData_t::feed(const geometry_msgs::TwistStamped::ConstPtr& pMsg) {
    msg = *pMsg;
    rcv_stamp = ros::Time::now();
    v << msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z;
    yaw_rate = msg.twist.angular.z;
}

ImuData_t::ImuData_t() : rcv_stamp(ros::Time(0)), w(Eigen::Vector3f::Zero()), a(Eigen::Vector3f::Zero()), roll(0), pitch(0), yaw(0) { q.setIdentity(); }
void ImuData_t::feed(const sensor_msgs::Imu::ConstPtr& pMsg) {
    msg = *pMsg;
    rcv_stamp = ros::Time::now();
    w << msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z;
    a << msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z;
    q = Eigen::Quaternionf(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
    
    // 复用原来的 utils 转换
    Eigen::Vector3f att = quaternionToEuler(q); 
    roll = att(0); pitch = att(1); yaw = att(2);
}

// ==========================================
// DataManager 单例与公开喂入/获取接口
// ==========================================

DataManager& DataManager::getInstance() {
    static DataManager instance;
    return instance;
}

// --- 数据喂入接口 (供外部 boost::bind 使用，带锁保护) ---
void DataManager::feed_gimbal_pose(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(_mutex);
    _gimbal_data.feed(msg);
    // 更新 云台 数据时，同时维护时间同步所需的历史缓存队列
    double ts = _gimbal_data.rcv_stamp.toSec();
    _gimbal_orientation_buffer.push_back({ts, _gimbal_data.q});

    while (!_gimbal_orientation_buffer.empty() && (ts - _gimbal_orientation_buffer.front().timestamp > MAX_BUFFER_AGE)) {
        _gimbal_orientation_buffer.pop_front();
    }
}

void DataManager::feed_state(const mavros_msgs::State::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(_mutex);
    std::string last_mode = _state_data.mode;
    _state_data.feed(msg);

    // 如果状态发生改变，可以保留日志打印
    if (_state_data.mode != last_mode) {
        ROS_INFO("Mode %s changed to: %s", last_mode.c_str(), _state_data.mode.c_str());
    }
}

void DataManager::feed_home_pos(const mavros_msgs::HomePosition::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(_mutex);
    _home_data.feed(msg);
}

void DataManager::feed_local_pos(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(_mutex);
    _local_pos_data.feed(msg);
}

void DataManager::feed_vel_body(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(_mutex);
    _vel_body_data.feed(msg);
}

void DataManager::feed_vel_enu(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(_mutex);
    _vel_enu_data.feed(msg);
}

void DataManager::feed_imu(const sensor_msgs::Imu::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(_mutex);
    _imu_data.feed(msg);

    // 更新 IMU 数据时，同时维护时间同步所需的历史缓存队列
    double ts = _imu_data.rcv_stamp.toSec();
    _omega_buffer.push_back({ts, _imu_data.w});
    _imu_orientation_buffer.push_back({ts, _imu_data.q});

    while (!_omega_buffer.empty() && (ts - _omega_buffer.front().timestamp > MAX_BUFFER_AGE)) {
        _omega_buffer.pop_front();
    }
    while (!_imu_orientation_buffer.empty() && (ts - _imu_orientation_buffer.front().timestamp > MAX_BUFFER_AGE)) {
        _imu_orientation_buffer.pop_front();
    }
}

// --- Getter 数据获取接口 (拷贝返回，脱离锁后绝对内存安全) ---
GimbalPoseData_t DataManager::getGimbalPose() const { std::lock_guard<std::mutex> lock(_mutex); return _gimbal_data; }
StateData_t      DataManager::getState() const      { std::lock_guard<std::mutex> lock(_mutex); return _state_data; }
HomePosData_t    DataManager::getHomePos() const    { std::lock_guard<std::mutex> lock(_mutex); return _home_data; }
LocalPosData_t   DataManager::getLocalPos() const   { std::lock_guard<std::mutex> lock(_mutex); return _local_pos_data; }
VelData_t        DataManager::getVelBody() const    { std::lock_guard<std::mutex> lock(_mutex); return _vel_body_data; }
VelData_t        DataManager::getVelEnu() const     { std::lock_guard<std::mutex> lock(_mutex); return _vel_enu_data; }
ImuData_t        DataManager::getImu() const        { std::lock_guard<std::mutex> lock(_mutex); return _imu_data; }

// --- 业务状态快速查询 ---
bool DataManager::is_connected() const {
    std::lock_guard<std::mutex> lock(_mutex);
    return _state_data.connected;
}

bool DataManager::is_armed() const {
    std::lock_guard<std::mutex> lock(_mutex);
    return _state_data.armed;
}

bool DataManager::is_ready_for_fly() const {
    std::lock_guard<std::mutex> lock(_mutex);
    return _state_data.connected && _state_data.armed;
}

std::string DataManager::get_flight_mode() const {
    std::lock_guard<std::mutex> lock(_mutex);
    return _state_data.mode;
}

Eigen::Quaternionf DataManager::get_imu_orientation() const {
    return get_synced_orientation(_imu_orientation_buffer);
}

Eigen::Quaternionf DataManager::get_gimbal_orientation() const {
    return get_synced_orientation(_gimbal_orientation_buffer);
}

// --- 缓冲区时间同步与插值计算 ---
Eigen::Quaternionf DataManager::get_synced_orientation(const std::deque<OriBufItem>& buffer) const {
    double timestamp=ros::Time::now().toSec();
    std::lock_guard<std::mutex> lock(_mutex);
    if (buffer.empty()) return Eigen::Quaternionf::Identity();
    if (timestamp >= buffer.back().timestamp) return buffer.back().q;
    if (timestamp <= buffer.front().timestamp) return buffer.front().q;

    auto it = std::lower_bound(buffer.begin(), buffer.end(), timestamp,[](const OriBufItem& data, double ts) { return data.timestamp < ts; });

    const auto& next = *it;
    const auto& curr = *std::prev(it);
    double dt = next.timestamp - curr.timestamp;
    
    // 时间极小时，防除零保护
    if (dt < 1e-9) return curr.q; 

    float ratio = static_cast<float>((timestamp - curr.timestamp) / dt);
    return curr.q.slerp(ratio, next.q);
}



// OmegaStatus DataManager::get_synced_omega(double image_timestamp) const {
//     std::lock_guard<std::mutex> lock(_mutex);
//     OmegaStatus invalid_result = {NAN_VAL, NAN_VAL, NAN_VAL};
//     if (_omega_buffer.empty()) return invalid_result;

//     auto to_omega =[](const Eigen::Vector3f& v) -> OmegaStatus { return {v.x(), v.y(), v.z()}; };

//     if (image_timestamp >= _omega_buffer.back().timestamp) return to_omega(_omega_buffer.back().omega);
//     if (image_timestamp <= _omega_buffer.front().timestamp) return to_omega(_omega_buffer.front().omega);

//     auto it = std::lower_bound(_omega_buffer.begin(), _omega_buffer.end(), image_timestamp,[](const ImuBufItem& data, double ts) { return data.timestamp < ts; });

//     const auto& next = *it;
//     const auto& curr = *std::prev(it);
//     double dt = next.timestamp - curr.timestamp;
    
//     // 时间极小时，防除零保护
//     if (dt < 1e-9) return to_omega(curr.omega); 

//     float ratio = static_cast<float>((image_timestamp - curr.timestamp) / dt);
//     Eigen::Vector3f interpolated_omega = curr.omega + ratio * (next.omega - curr.omega);
//     return to_omega(interpolated_omega);
// }