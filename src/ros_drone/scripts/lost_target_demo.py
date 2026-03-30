#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
目标丢失测试 Demo（纯发布/纯观测）。

用途：
1) 向 gimbal 跟踪节点订阅的 /target_in_image（Float64MultiArray）发布“有效目标”
2) 再发布 valid=0 的“丢失目标”（gimbal 节点会因为 lost_timeout 而进入扫描）
3) 订阅 /uav0/mavros/mount_control/command 与 /uav0/mavros/gimbal_pose 观测云台是否开始/结束扫描

说明：
- 该脚本不依赖检测器（detector/yolov8_detector_pt.py）。
- 你需要先启动 mavros 仿真与云台跟踪节点（gimbal_track_target.py 或 gimbal_yaw_track_target.py）。
"""

import threading
import time

import rospy
from mavros_msgs.msg import MountControl
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from tf.transformations import euler_from_quaternion


def _now_s() -> float:
    return rospy.Time.now().to_sec()


class _LogBuffer:
    def __init__(self):
        self._lock = threading.Lock()
        self.cmd_samples = []  # (t, yaw_deg, pitch_deg)
        self.pose_samples = []  # (t, yaw_deg, pitch_deg)

    def on_cmd(self, msg: MountControl):
        # gimbal_* 脚本里直接用“度”发布 pitch/yaw
        t = msg.header.stamp.to_sec() if msg.header.stamp else _now_s()
        with self._lock:
            self.cmd_samples.append((t, float(msg.yaw), float(msg.pitch)))

    def on_pose(self, msg: Odometry):
        q = msg.pose.pose.orientation
        # tf 返回 (roll, pitch, yaw) in radians
        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        t = msg.header.stamp.to_sec() if msg.header.stamp else _now_s()
        with self._lock:
            self.pose_samples.append((t, float(yaw * 180.0 / 3.1415926535), float(pitch * 180.0 / 3.1415926535)))

    def snapshot(self):
        with self._lock:
            return list(self.cmd_samples), list(self.pose_samples)


def main():
    rospy.init_node("lost_target_demo", anonymous=False)

    target_track_topic = rospy.get_param("~target_track_topic", "/target_in_image")
    mount_command_topic = rospy.get_param("~mount_command_topic", "/uav0/mavros/mount_control/command")
    gimbal_pose_topic = rospy.get_param("~gimbal_pose_topic", "/uav0/mavros/gimbal_pose")

    pub_rate_hz = float(rospy.get_param("~pub_rate_hz", 20.0))
    pre_lost_s = float(rospy.get_param("~pre_lost_s", 2.0))
    lost_s = float(rospy.get_param("~lost_s", 4.0))
    restore_s = float(rospy.get_param("~restore_s", 2.0))
    expected_lost_timeout_s = float(rospy.get_param("~expected_lost_timeout_s", 1.5))

    # 目标框像素参数（gimbal 脚本用它们计算 ex/ey）
    img_w = float(rospy.get_param("~img_w", 640.0))
    img_h = float(rospy.get_param("~img_h", 480.0))
    cx_ratio = float(rospy.get_param("~cx_ratio", 0.5))
    cy_ratio = float(rospy.get_param("~cy_ratio", 0.5))
    iw = float(rospy.get_param("~iw", 200.0))
    ih = float(rospy.get_param("~ih", 200.0))

    conf = float(rospy.get_param("~conf", 0.8))
    valid_flag = float(rospy.get_param("~valid_flag", 1.0))
    invalid_valid_flag = float(rospy.get_param("~invalid_valid_flag", 0.0))

    scan_detect_yaw_change_deg = float(rospy.get_param("~scan_detect_yaw_change_deg", 5.0))
    scan_detect_pitch_change_deg = float(rospy.get_param("~scan_detect_pitch_change_deg", 5.0))
    use_pitch_feedback = bool(rospy.get_param("~use_pitch_feedback", True))

    buf = _LogBuffer()

    pub = rospy.Publisher(target_track_topic, Float64MultiArray, queue_size=10)
    rospy.Subscriber(mount_command_topic, MountControl, buf.on_cmd, queue_size=50)
    do_pose = bool(rospy.get_param("~use_pose_feedback", True))
    if do_pose:
        rospy.Subscriber(gimbal_pose_topic, Odometry, buf.on_pose, queue_size=50)

    cx = img_w * cx_ratio
    cy = img_h * cy_ratio

    def publish_target(vflag: float):
        # Float64MultiArray: [cx, cy, iw, ih, conf, valid]
        msg = Float64MultiArray()
        msg.data = [float(cx), float(cy), float(iw), float(ih), float(conf), float(vflag)]
        pub.publish(msg)

    # 给 gimbal 节点一点时间订阅/初始化
    rospy.sleep(1.0)

    rospy.loginfo(
        "lost_target_demo: publish to %s; pre_lost=%.2fs lost=%.2fs restore=%.2fs expected_timeout=%.2fs",
        target_track_topic,
        pre_lost_s,
        lost_s,
        restore_s,
        expected_lost_timeout_s,
    )

    t0 = _now_s()
    lost_start = t0 + pre_lost_s
    lost_end = lost_start + lost_s
    restore_end = lost_end + restore_s

    # 丢失检测窗口：大概在 lost_timeout 后开始统计
    scan_window_start = lost_start + 0.8 * expected_lost_timeout_s

    r = rospy.Rate(pub_rate_hz)
    while not rospy.is_shutdown() and _now_s() < lost_start:
        publish_target(valid_flag)
        r.sleep()

    # 进入丢失：发布 valid=0（gimbal 节点会 return，不刷新 last_track_time）
    while not rospy.is_shutdown() and _now_s() < lost_end:
        publish_target(invalid_valid_flag)
        r.sleep()

    # 恢复：再发布有效目标
    while not rospy.is_shutdown() and _now_s() < restore_end:
        publish_target(valid_flag)
        r.sleep()

    # 结束后再等一小段，让恢复过程至少发出几条 mount 指令
    rospy.sleep(0.5)

    cmd_samples, pose_samples = buf.snapshot()
    if not cmd_samples:
        rospy.logwarn("lost_target_demo: 没有收到 %s 的 MountControl 回传，无法判断扫描结果。", mount_command_topic)
        return

    def stats_in_range(samples, t_start, t_end):
        values = [s for s in samples if t_start <= s[0] <= t_end]
        if not values:
            return None
        return values

    pre_cmd = stats_in_range(cmd_samples, t0, lost_start)
    lost_cmd = stats_in_range(cmd_samples, scan_window_start, lost_end)
    restore_cmd = stats_in_range(cmd_samples, lost_end, restore_end)

    def compute_range(arr, idx):
        if not arr:
            return None
        vmin = min(x[idx] for x in arr)
        vmax = max(x[idx] for x in arr)
        return vmax - vmin, vmin, vmax

    yaw_range_lost = compute_range(lost_cmd, 1)
    pitch_range_lost = compute_range(lost_cmd, 2)

    yaw_baseline = None
    pitch_baseline = None
    if pre_cmd:
        yaw_baseline = sum(x[1] for x in pre_cmd) / float(len(pre_cmd))
        pitch_baseline = sum(x[2] for x in pre_cmd) / float(len(pre_cmd))

    scanning_detected = False
    yaw_range_deg = None
    pitch_range_deg = None

    if yaw_range_lost is not None:
        yaw_range_deg = yaw_range_lost[0]
        if yaw_range_deg >= scan_detect_yaw_change_deg:
            scanning_detected = True

    if use_pitch_feedback and pitch_range_lost is not None:
        pitch_range_deg = pitch_range_lost[0]
        if pitch_range_deg < scan_detect_pitch_change_deg:
            # 允许 yaw 正常扫描但 pitch 未明显变化（例如 yaw-only 节点）
            pass

    # 恢复是否停止扫描：看恢复窗口内 yaw_range 是否变小
    restore_yaw_range = compute_range(restore_cmd, 1)
    restore_scan_stopped = False
    if restore_yaw_range is not None:
        restore_yaw_range_deg = restore_yaw_range[0]
        # 恢复窗口里 yaw 摆动显著降低（经验阈值）
        restore_scan_stopped = restore_yaw_range_deg < max(2.0, 0.35 * (yaw_range_deg or 0.0))

    # 计算一个“首次显著偏离”时间点（用于定位）
    first_significant_t = None
    if yaw_baseline is not None and lost_cmd:
        for t, yaw_deg, _pitch_deg in lost_cmd:
            if abs(yaw_deg - yaw_baseline) >= scan_detect_yaw_change_deg:
                first_significant_t = t
                break

    rospy.loginfo("lost_target_demo result:")
    rospy.loginfo("  scanning_detected=%s first_scan_time=%.3f s", scanning_detected, (first_significant_t or -1.0) )
    rospy.loginfo(
        "  yaw_range_lost=%.3f deg pitch_range_lost=%.3f deg (lost window)",
        float(yaw_range_deg or 0.0),
        float(pitch_range_deg or 0.0),
    )
    if yaw_baseline is not None:
        rospy.loginfo("  yaw_baseline(pre)=%.3f deg pitch_baseline(pre)=%.3f deg", yaw_baseline, pitch_baseline or 0.0)
    rospy.loginfo("  restore_scan_stopped=%s", restore_scan_stopped)

    if pose_samples:
        pre_pose = stats_in_range(pose_samples, t0, lost_start)
        lost_pose = stats_in_range(pose_samples, scan_window_start, lost_end)
        restore_pose = stats_in_range(pose_samples, lost_end, restore_end)
        yaw_pose_range = compute_range(lost_pose, 1)
        pitch_pose_range = compute_range(lost_pose, 2)
        rospy.loginfo(
            "  gimbal_pose(yaw/pitch) range lost: yaw=%.3f deg pitch=%.3f deg",
            float(yaw_pose_range[0]) if yaw_pose_range else 0.0,
            float(pitch_pose_range[0]) if pitch_pose_range else 0.0,
        )

    rospy.loginfo("lost_target_demo: finished.")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

