#!/usr/bin/env python3
"""
简洁云台：俯仰固定，仅偏航（yaw）跟踪。

跟随模式：收到「有效目标」时（见下方），只根据水平像素偏差调偏航，并刷新 last_track；
         此时定时器不再发扫描角，等价于停止搜索。

搜索模式：连续超过 lost_target_timeout 未收到有效目标时，偏航在 [yaw_min,yaw_max] 内正弦扫描。

订阅 Float64MultiArray：[cx, cy, w, h, conf] 或 6 元 [..., valid]。
  - 若有第 6 元 valid：须 valid>=0.5 才算有效。
  - 若 conf 存在：须 conf >= min_detection_conf 才算有效（与 detector 阈值一致即可）。
"""

import math

import rospy
from mavros_msgs.msg import MountControl
from std_msgs.msg import Float64MultiArray


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def main():
    rospy.init_node("gimbal_yaw_track_target")

    mount_topic = rospy.get_param(
        "~mount_command_topic", "/uav0/mavros/mount_control/command"
    )
    # 默认与 yolov8_detector_pt 相对名 target_in_image 解析结果一致：/target_in_image
    track_topic = rospy.get_param("~target_track_topic", "/target_in_image")

    pitch_fixed = float(rospy.get_param("~fixed_pitch_deg", 0.0))
    yaw_gain_deg = float(rospy.get_param("~yaw_gain_deg", 2.5))
    dead_norm = float(rospy.get_param("~dead_zone_normalized", 0.02))
    yaw_min = float(rospy.get_param("~yaw_min_deg", -80.0))
    yaw_max = float(rospy.get_param("~yaw_max_deg", 80.0))
    invert_yaw = bool(rospy.get_param("~invert_yaw", False))

    lost_timeout = float(rospy.get_param("~lost_target_timeout", 1.5))
    min_detection_conf = float(rospy.get_param("~min_detection_conf", 0.25))
    scan_enable = bool(rospy.get_param("~scan_enable", True))
    scan_yaw_period_s = float(rospy.get_param("~scan_yaw_period_s", 14.0))
    scan_yaw_span_ratio = float(rospy.get_param("~scan_yaw_span_ratio", 0.92))
    scan_rate_hz = float(rospy.get_param("~scan_rate_hz", 20.0))

    yaw_cmd = float(rospy.get_param("~initial_yaw_deg", 0.0))
    yaw_cen = 0.5 * (yaw_min + yaw_max)
    yaw_amp = 0.5 * (yaw_max - yaw_min) * clamp(scan_yaw_span_ratio, 0.05, 1.0)

    last_track_time = rospy.Time(0)
    scanning = False
    last_pub_yaw = yaw_cmd
    t_scan0 = rospy.Time.now().to_sec()

    cmd_pub = rospy.Publisher(mount_topic, MountControl, queue_size=1)

    def publish_mount(yaw_deg):
        nonlocal last_pub_yaw
        m = MountControl()
        m.header.stamp = rospy.Time.now()
        m.mode = MountControl.MAV_MOUNT_MODE_MAVLINK_TARGETING
        m.pitch = pitch_fixed
        m.roll = 0.0
        m.yaw = yaw_deg
        m.altitude = 0.0
        m.latitude = 0.0
        m.longitude = 0.0
        cmd_pub.publish(m)
        last_pub_yaw = yaw_deg

    def on_track(msg: Float64MultiArray):
        nonlocal yaw_cmd, last_track_time, scanning
        if len(msg.data) < 5:
            return
        cx, _, iw, ih = msg.data[0], msg.data[1], msg.data[2], msg.data[3]
        if iw <= 1 or ih <= 1:
            return
        if len(msg.data) >= 6 and msg.data[5] < 0.5:
            return
        conf = float(msg.data[4])
        if conf < min_detection_conf:
            return

        if scanning:
            yaw_cmd = last_pub_yaw
            scanning = False
            rospy.loginfo("检测到目标 (conf=%.2f)，停止搜索，进入跟随模式", conf)

        last_track_time = rospy.Time.now()

        ex = (cx - 0.5 * iw) / (0.5 * iw)
        if abs(ex) < dead_norm:
            ex = 0.0
        dy = yaw_gain_deg * ex
        if invert_yaw:
            dy = -dy
        yaw_cmd = clamp(yaw_cmd + dy, yaw_min, yaw_max)
        publish_mount(yaw_cmd)

    def on_timer(_event):
        nonlocal scanning
        if not scan_enable:
            return
        now = rospy.Time.now()
        if (now - last_track_time).to_sec() <= lost_timeout:
            return
        scanning = True
        t = rospy.Time.now().to_sec() - t_scan0
        w = 2.0 * math.pi / max(scan_yaw_period_s, 0.5)
        y = yaw_cen + yaw_amp * math.sin(w * t)
        publish_mount(clamp(y, yaw_min, yaw_max))

    rospy.Subscriber(track_topic, Float64MultiArray, on_track, queue_size=4, buff_size=65536)
    rospy.Timer(rospy.Duration(1.0 / max(scan_rate_hz, 1.0)), on_timer)

    rospy.loginfo(
        "gimbal_yaw_track_target: pitch=%.1f° 固定；订阅 track(解析后)=%s；conf≥%.2f 且未丢失>%.1fs 时为跟随，否则 yaw 正弦 T=%.1fs",
        pitch_fixed,
        rospy.resolve_name(track_topic),
        min_detection_conf,
        lost_timeout,
        scan_yaw_period_s,
    )
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
