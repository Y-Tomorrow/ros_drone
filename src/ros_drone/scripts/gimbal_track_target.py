#!/usr/bin/env python3

import math

import rospy
from mavros_msgs.msg import MountControl
from std_msgs.msg import Float64MultiArray


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def smoothstep(x):
    x = clamp(x, 0.0, 1.0)
    return x * x * (3.0 - 2.0 * x)


def lerp(a, b, t):
    return a + (b - a) * t


def main():
    rospy.init_node("gimbal_track_target")

    mount_topic = rospy.get_param(
        "~mount_command_topic", "/uav0/mavros/mount_control/command"
    )
    track_topic = rospy.get_param("~target_track_topic", "/target_in_image")

    yaw_gain_deg = float(rospy.get_param("~yaw_gain_deg", 2.5))
    pitch_gain_deg = float(rospy.get_param("~pitch_gain_deg", 2.0))
    dead_norm = float(rospy.get_param("~dead_zone_normalized", 0.02))

    yaw_min = float(rospy.get_param("~yaw_min_deg", -80.0))
    yaw_max = float(rospy.get_param("~yaw_max_deg", 80.0))
    pitch_min = float(rospy.get_param("~pitch_min_deg", -45.0))
    pitch_max = float(rospy.get_param("~pitch_max_deg", 45.0))

    invert_yaw = bool(rospy.get_param("~invert_yaw", False))
    invert_pitch = bool(rospy.get_param("~invert_pitch", False))

    lost_timeout = float(rospy.get_param("~lost_target_timeout", 1.5))
    min_detection_conf = float(rospy.get_param("~min_detection_conf", 0.25))

    scan_enable = bool(rospy.get_param("~scan_enable", True))
    scan_rate_hz = float(rospy.get_param("~scan_rate_hz", 20.0))

    scan_yaw_period_s = float(rospy.get_param("~scan_yaw_period_s", 14.0))
    scan_yaw_span_ratio = float(rospy.get_param("~scan_yaw_span_ratio", 0.92))

    # 跟随稳定性参数
    error_lpf_alpha = float(rospy.get_param("~error_lpf_alpha", 0.35))
    max_step_yaw_deg = float(rospy.get_param("~max_step_yaw_deg", 1.2))
    max_step_pitch_deg = float(rospy.get_param("~max_step_pitch_deg", 1.0))

    yaw_cmd = float(rospy.get_param("~initial_yaw_deg", 0.0))
    pitch_cmd = float(rospy.get_param("~initial_pitch_deg", 0.0))

    yaw_cen = 0.5 * (yaw_min + yaw_max)
    yaw_span = (yaw_max - yaw_min) * clamp(scan_yaw_span_ratio, 0.05, 1.0)
    yaw_lo = yaw_cen - 0.5 * yaw_span
    yaw_hi = yaw_cen + 0.5 * yaw_span

    pitch_cen = 0.5 * (pitch_min + pitch_max)

    last_track_time = rospy.Time(0)
    scanning = False
    last_pub_yaw = yaw_cmd
    last_pub_pitch = pitch_cmd
    t_scan0 = rospy.Time.now().to_sec()
    ex_f = 0.0
    ey_f = 0.0
    have_err = False

    pitch_level = 0  # 1,3: 水平，0: 向上，2: 向下
    prev_scan_u_phase = -1.0  # 搜索：检测越过 u=0.5（min→max 完成）
    last_scan_t_mod = -1.0  # 搜索：检测 t_mod 回绕（max→min 完成）
    last_have_target = False  # 从跟随切回搜索时重置相位，避免误判

    cmd_pub = rospy.Publisher(mount_topic, MountControl, queue_size=1)

    def publish_mount(yaw_deg, pitch_deg):
        nonlocal last_pub_yaw, last_pub_pitch
        m = MountControl()
        m.header.stamp = rospy.Time.now()
        m.mode = MountControl.MAV_MOUNT_MODE_MAVLINK_TARGETING
        m.pitch = pitch_deg
        m.roll = 0.0
        m.yaw = yaw_deg
        m.altitude = 0.0
        m.latitude = 0.0
        m.longitude = 0.0
        cmd_pub.publish(m)
        last_pub_yaw = yaw_deg
        last_pub_pitch = pitch_deg

    def on_track(msg: Float64MultiArray):
        nonlocal last_track_time, scanning, ex_f, ey_f, have_err
        if len(msg.data) < 5:
            return
        cx, cy, iw, ih = msg.data[0], msg.data[1], msg.data[2], msg.data[3]
        if iw <= 1 or ih <= 1:
            return
        if len(msg.data) >= 6 and msg.data[5] < 0.5:
            return
        conf = float(msg.data[4])
        if conf < min_detection_conf:
            return

        if scanning:
            scanning = False
            rospy.loginfo("检测到目标 (conf=%.2f)，停止搜索，进入跟随模式", conf)

        last_track_time = rospy.Time.now()

        ex = (cx - 0.5 * iw) / (0.5 * iw)
        ey = (cy - 0.5 * ih) / (0.5 * ih)
        if abs(ex) < dead_norm:
            ex = 0.0
        if abs(ey) < dead_norm:
            ey = 0.0

        if invert_yaw:
            ex = -ex
        if invert_pitch:
            ey = -ey

        if not have_err:
            ex_f, ey_f = ex, ey
            have_err = True
        else:
            a = clamp(error_lpf_alpha, 0.0, 1.0)
            ex_f = (1.0 - a) * ex_f + a * ex
            ey_f = (1.0 - a) * ey_f + a * ey
 
    def on_timer(_event):
        nonlocal scanning, yaw_cmd, pitch_cmd, pitch_level
        nonlocal prev_scan_u_phase, last_scan_t_mod, last_have_target
        now = rospy.Time.now()
        have_target = (now - last_track_time).to_sec() <= lost_timeout

        # 跟随模式：固定频率输出（更稳）
        if have_target:
            scanning = False
            last_have_target = True
            if have_err:
                dyaw = clamp(yaw_gain_deg * ex_f, -abs(max_step_yaw_deg), abs(max_step_yaw_deg))
                dpitch = clamp(
                    pitch_gain_deg * ey_f, -abs(max_step_pitch_deg), abs(max_step_pitch_deg)
                )
                yaw_cmd = clamp(yaw_cmd + dyaw, yaw_min, yaw_max)
                pitch_cmd = clamp(pitch_cmd + dpitch, pitch_min, pitch_max)
                publish_mount(yaw_cmd, pitch_cmd)
            return

        # 搜索模式
        if not scan_enable:
            return
        if last_have_target:
            prev_scan_u_phase = -1.0
            last_scan_t_mod = -1.0
            last_have_target = False

        scanning = True
        t = rospy.Time.now().to_sec() - t_scan0
        T = max(scan_yaw_period_s, 0.5)
        t_mod = t % T
        u_phase = t_mod / T
        if u_phase < 0.5:
            uu = smoothstep(u_phase / 0.5)
            y = yaw_lo + (yaw_hi - yaw_lo) * uu
        else:
            uu = smoothstep((u_phase - 0.5) / 0.5)
            y = yaw_hi - (yaw_hi - yaw_lo) * uu

        if prev_scan_u_phase >= 0.0 and prev_scan_u_phase < 0.5 <= u_phase:
            pitch_level = (pitch_level + 1) % 4
        if last_scan_t_mod >= 0.0 and t_mod + 1e-6 < last_scan_t_mod:
            pitch_level = (pitch_level + 1) % 4
        prev_scan_u_phase = u_phase
        last_scan_t_mod = t_mod

        yaw_cmd = clamp(y, yaw_min, yaw_max)
        if pitch_level == 0:
            pitch_cmd = clamp(pitch_max - 10, pitch_min, pitch_max)
        elif pitch_level == 2:
            pitch_cmd = clamp(pitch_min + 10, pitch_min, pitch_max)
        else:
            pitch_cmd = clamp(0, pitch_min, pitch_max)
        publish_mount(yaw_cmd, pitch_cmd)

    rospy.Subscriber(
        track_topic, Float64MultiArray, on_track, queue_size=4, buff_size=65536
    )
    rospy.Timer(rospy.Duration(1.0 / max(scan_rate_hz, 1.0)), on_timer)

    rospy.loginfo(
        "gimbal_track_target: 订阅 track=%s；conf≥%.2f 且未丢失>%.1fs 时跟随(yaw/pitch)，否则 yaw 往返扫 (周期=%.1fs)",
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

