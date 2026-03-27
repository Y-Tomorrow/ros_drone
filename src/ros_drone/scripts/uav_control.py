#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
目标机 (uav1) 任务：先悬停 mission.hover_altitude_m，再用键盘遥控 OFFBOARD 速度。

水平控制（按键语义）：
  W = 前 S = 后 A = 左 D = 右

垂直控制：
  E = 上升 Q = 下降
"""
from __future__ import annotations

import os
import select
import sys
import threading
import time

import rospy
import yaml
from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, ParamSet, SetMode
from mavros_msgs.msg import ParamValue
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler

try:
    import termios
    import tty
except ImportError:
    termios = None
    tty = None

try:
    from pynput import keyboard
except ImportError:
    keyboard = None


def _load_yaml_defaults():
    """从包内 config/target_escape_teleop.yaml 读取默认参数。"""
    candidates = []
    try:
        import rospkg

        rp = rospkg.RosPack()
        candidates.append(
            os.path.join(rp.get_path("ros_drone"), "config", "target_escape_teleop.yaml")
        )
    except Exception:
        pass
    here = os.path.dirname(os.path.abspath(__file__))
    candidates.append(os.path.join(here, "..", "config", "target_escape_teleop.yaml"))
    for path in candidates:
        path = os.path.normpath(path)
        if os.path.isfile(path):
            try:
                with open(path, "r", encoding="utf-8") as f:
                    return yaml.safe_load(f) or {}
            except Exception as e:
                rospy.logwarn("读取 %s 失败: %s", path, e)
    return {}


class TargetEscapeSequence(object):
    def __init__(self):
        rospy.init_node("uav_control", anonymous=False)
        self.ns = "/uav1/mavros"
        self.state = State()
        self.local_pos = PoseStamped()
        self._phase = "hover"  # hover -> teleop
        self._lock = threading.Lock()

        defaults = _load_yaml_defaults()
        td = defaults.get("teleop", {})
        md = defaults.get("mission", {})

        self.h_speed = float(
            rospy.get_param(
                "~teleop/horizontal_speed_mps", td.get("horizontal_speed_mps", 8.0)
            )
        )
        self.v_speed = float(
            rospy.get_param(
                "~teleop/vertical_speed_mps", td.get("vertical_speed_mps", 2.0)
            )
        )
        self.control_rate_hz = float(
            rospy.get_param("~teleop/control_rate_hz", td.get("control_rate_hz", 20.0))
        )
        self.hover_z = float(
            rospy.get_param("~mission/hover_altitude_m", md.get("hover_altitude_m", 10.0))
        )
        # stdin 模式：无新字符超过该时间则认为松开（需略大于系统键重复间隔）
        self._key_idle_timeout = float(rospy.get_param("~key_idle_timeout", 0.55))

        rospy.Subscriber(self.ns + "/state", State, self._cb_state, queue_size=1)
        rospy.Subscriber(
            self.ns + "/local_position/pose",
            PoseStamped,
            self._cb_local,
            queue_size=1,
        )

        self.pub_pos = rospy.Publisher(
            self.ns + "/setpoint_position/local", PoseStamped, queue_size=10
        )
        self.pub_raw = rospy.Publisher(
            self.ns + "/setpoint_raw/local", PositionTarget, queue_size=10
        )

        self._pos_sp = PoseStamped()
        self._ctl_stop = threading.Event()
        self._ctl_thread = None

        # 各键最后一次收到字符的时间；0 表示未按下（pynput release）
        self._last_seen = {k: 0.0 for k in "wasdeq"}
        self._stop_r_until_right_up = False
        self._stop_l_until_left_up = False
        self._stop_n_until_up_up = False
        self._stop_s_until_down_up = False

        self._kb_listener = None
        self._stdin_stop = threading.Event()
        self._kbd_old_termios = None
        self._stdin_thread = None
        self._stdin_stream = None  # sys.stdin 或 /dev/tty
        self._stdin_stream_owned = False  # 若为 open('/dev/tty') 需在退出时关闭

        rospy.on_shutdown(self._shutdown)

    def _shutdown(self):
        self._stdin_stop.set()
        st = self._stdin_stream if self._stdin_stream is not None else sys.stdin
        if self._kbd_old_termios is not None and termios is not None:
            try:
                termios.tcsetattr(st, termios.TCSADRAIN, self._kbd_old_termios)
            except Exception:
                pass
        if self._stdin_stream_owned and self._stdin_stream is not None:
            try:
                self._stdin_stream.close()
            except Exception:
                pass
            self._stdin_stream = None
            self._stdin_stream_owned = False
        if self._kb_listener is not None:
            try:
                self._kb_listener.stop()
            except Exception:
                pass

    def _key_active(self, k: str) -> bool:
        t = self._last_seen.get(k, 0.0)
        if t <= 0.0:
            return False
        return time.time() - t < self._key_idle_timeout

    def _sync_latches(self):
        """松开某键后清除对应闩锁（stdin 模式无 key-up，用超时判定）。"""
        if not self._key_active("d"):
            self._stop_r_until_right_up = False
        if not self._key_active("a"):
            self._stop_l_until_left_up = False
        if not self._key_active("w"):
            self._stop_n_until_up_up = False
        if not self._key_active("s"):
            self._stop_s_until_down_up = False

    def _cb_state(self, msg: State):
        self.state = msg

    def _cb_local(self, msg: PoseStamped):
        self.local_pos = msg

    def wait_connected(self, timeout_s: float = 120.0):
        t0 = time.time()
        rospy.loginfo("等待 MAVROS 连接 /uav1 ...")
        while not rospy.is_shutdown():
            if self.state.connected:
                break
            if time.time() - t0 > timeout_s:
                rospy.logerr("MAVROS 超时未连接")
                sys.exit(1)
            rospy.sleep(0.2)

    def _wait_local_position(self, timeout_s: float = 60.0):
        t0 = time.time()
        while not rospy.is_shutdown():
            if self.local_pos.header.stamp != rospy.Time(0):
                break
            if time.time() - t0 > timeout_s:
                rospy.logerr("未收到 local_position")
                sys.exit(1)
            rospy.sleep(0.1)

    def set_param(self, param_id: str, integer: int, timeout_s: float = 5.0):
        rospy.wait_for_service(self.ns + "/param/set", timeout=timeout_s)
        ps = rospy.ServiceProxy(self.ns + "/param/set", ParamSet)
        pv = ParamValue()
        pv.integer = integer
        pv.real = 0.0
        r = ps(param_id, pv)
        if not r.success:
            rospy.logwarn("设置参数失败 %s (可忽略)", param_id)

    def set_mode(self, mode: str, timeout_s: float = 10.0):
        rospy.wait_for_service(self.ns + "/set_mode", timeout=timeout_s)
        sm = rospy.ServiceProxy(self.ns + "/set_mode", SetMode)
        t0 = time.time()
        while not rospy.is_shutdown():
            if sm(custom_mode=mode).mode_sent:
                rospy.loginfo("模式已切换: %s", mode)
                return
            if time.time() - t0 > timeout_s:
                rospy.logerr("切换模式失败: %s", mode)
                sys.exit(1)
            rospy.sleep(0.2)

    def set_arm(self, arm: bool, timeout_s: float = 10.0):
        rospy.wait_for_service(self.ns + "/cmd/arming", timeout=timeout_s)
        ar = rospy.ServiceProxy(self.ns + "/cmd/arming", CommandBool)
        t0 = time.time()
        while not rospy.is_shutdown():
            if ar(arm).success:
                rospy.loginfo("解锁: %s", arm)
                return
            if time.time() - t0 > timeout_s:
                rospy.logerr("解锁失败")
                sys.exit(1)
            rospy.sleep(0.2)

    @staticmethod
    def _pynput_key_char(key):
        ch = getattr(key, "char", None)
        if ch is None:
            return None
        return ch.lower()

    def _handle_char_event(self, ch: str, is_press: bool):
        """is_press: stdin 与按下为 True；pynput 松开为 False。"""
        if ch not in "wasdeq":
            return
        with self._lock:
            if is_press:
                if ch == "w":
                    if self._key_active("s"):
                        self._stop_s_until_down_up = True
                elif ch == "s":
                    if self._key_active("w"):
                        self._stop_n_until_up_up = True
                elif ch == "a":
                    if self._key_active("d"):
                        self._stop_r_until_right_up = True
                elif ch == "d":
                    if self._key_active("a"):
                        self._stop_l_until_left_up = True
                self._last_seen[ch] = time.time()
            else:
                self._last_seen[ch] = 0.0
                if ch == "w":
                    self._stop_n_until_up_up = False
                elif ch == "s":
                    self._stop_s_until_down_up = False
                elif ch == "a":
                    self._stop_l_until_left_up = False
                elif ch == "d":
                    self._stop_r_until_right_up = False

    def _compute_vel_ned(self):
        """返回 (vn, ve, vd) NED 速度。"""
        with self._lock:
            self._sync_latches()
            kw, ks, ka, kd = (
                self._key_active("w"),
                self._key_active("s"),
                self._key_active("a"),
                self._key_active("d"),
            )
            ke, kq = self._key_active("e"), self._key_active("q")
            sr, sl, sn, sd = (
                self._stop_r_until_right_up,
                self._stop_l_until_left_up,
                self._stop_n_until_up_up,
                self._stop_s_until_down_up,
            )
            hs, vs = self.h_speed, self.v_speed

        # 左右：修复“左右反了”的情况后，交换左右速度分量的符号映射。
        # 约定：A=左，对应 ve 取正；D=右，对应 ve 取负（在你的仿真世界坐标下校准）。
        if sr and kd:
            ve = 0.0
        elif sl and ka:
            ve = 0.0
        elif kd and not ka:
            ve = -hs
        elif ka and not kd:
            ve = hs
        elif kd and ka:
            ve = 0.0
        else:
            ve = 0.0

        if sn and kw:
            vn = 0.0
        elif sd and ks:
            vn = 0.0
        elif kw and not ks:
            vn = hs
        elif ks and not kw:
            vn = -hs
        elif kw and ks:
            vn = 0.0
        else:
            vn = 0.0

        if ke and kq:
            vd = 0.0
        elif ke:
            vd = -vs
        elif kq:
            vd = vs
        else:
            vd = 0.0

        return (vn, ve, vd)

    def _pynput_on_press(self, key):
        try:
            ch = self._pynput_key_char(key)
            if ch in "wasdeq":
                self._handle_char_event(ch, True)
        except Exception:
            pass

    def _pynput_on_release(self, key):
        try:
            ch = self._pynput_key_char(key)
            if ch in "wasdeq":
                self._handle_char_event(ch, False)
        except Exception:
            pass

    def _stdin_loop(self):
        if termios is None or tty is None:
            return
        st = self._stdin_stream
        if st is None:
            return
        fd = st.fileno()
        try:
            self._kbd_old_termios = termios.tcgetattr(fd)
            tty.setcbreak(fd)
        except Exception as e:
            rospy.logerr("无法设置终端 raw 模式: %s", e)
            return
        try:
            while not rospy.is_shutdown() and not self._stdin_stop.is_set():
                r, _, _ = select.select([st], [], [], 0.12)
                if not r:
                    continue
                ch = st.read(1)
                if not ch:
                    continue
                o = ord(ch)
                if o in (3, 4):  # Ctrl+C / Ctrl+D
                    rospy.signal_shutdown("stdin eof")
                    break
                c = ch.lower()
                if c in "wasdeq":
                    self._handle_char_event(c, True)
        finally:
            if self._kbd_old_termios is not None:
                try:
                    termios.tcsetattr(fd, termios.TCSADRAIN, self._kbd_old_termios)
                except Exception:
                    pass
            self._kbd_old_termios = None

    def _resolve_stdin_stream(self):
        """rosrun 时多为 TTY；roslaunch 子进程常无 TTY，可改用 /dev/tty。"""
        if sys.stdin.isatty():
            self._stdin_stream = sys.stdin
            self._stdin_stream_owned = False
            return True
        try:
            self._stdin_stream = open("/dev/tty", "r")
            self._stdin_stream_owned = True
            rospy.loginfo("节点 stdin 非 TTY，已改用 /dev/tty（请在运行 roslaunch 的真实终端按键）")
            return True
        except OSError as e:
            rospy.logwarn("无法打开 /dev/tty: %s", e)
            self._stdin_stream = None
            return False

    def _start_keyboard(self):
        use_stdin = rospy.get_param("~use_stdin_keyboard", True)
        force_pynput = rospy.get_param("~use_pynput_keyboard", False)

        if use_stdin and not force_pynput and termios is not None:
            if self._resolve_stdin_stream():
                rospy.loginfo("终端键盘模式（英文输入法；cbreak 下无回显）")
                self._stdin_thread = threading.Thread(target=self._stdin_loop)
                self._stdin_thread.daemon = True
                self._stdin_thread.start()
                return

        if keyboard is None:
            rospy.logfatal(
                "stdin 非 TTY 或不可用，且未安装 pynput。请：在本机终端运行 rosrun，或 pip install pynput 并设置 _use_pynput_keyboard:=true"
            )
            sys.exit(1)

        rospy.loginfo("使用 pynput 全局键盘")
        self._kb_listener = keyboard.Listener(
            on_press=self._pynput_on_press, on_release=self._pynput_on_release
        )
        self._kb_listener.daemon = True
        self._kb_listener.start()

    def _thread_offboard_stream(self):
        rate_hz = max(5.0, min(50.0, self.control_rate_hz))
        r = rospy.Rate(rate_hz)
        while not rospy.is_shutdown() and not self._ctl_stop.is_set():
            with self._lock:
                ph = self._phase
            if ph == "hover":
                self._pos_sp.header.stamp = rospy.Time.now()
                self.pub_pos.publish(self._pos_sp)
            else:
                vn, ve, vd = self._compute_vel_ned()
                pt = PositionTarget()
                pt.header = Header()
                pt.header.stamp = rospy.Time.now()
                pt.header.frame_id = "map"
                pt.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
                pt.type_mask = (
                    PositionTarget.IGNORE_PX
                    | PositionTarget.IGNORE_PY
                    | PositionTarget.IGNORE_PZ
                    | PositionTarget.IGNORE_AFX
                    | PositionTarget.IGNORE_AFY
                    | PositionTarget.IGNORE_AFZ
                    | PositionTarget.IGNORE_YAW
                    | PositionTarget.IGNORE_YAW_RATE
                )
                pt.velocity.x = vn
                pt.velocity.y = ve
                pt.velocity.z = vd
                self.pub_raw.publish(pt)
            try:
                r.sleep()
            except rospy.ROSInterruptException:
                break

    def _wait_altitude(self, z_target: float, tol: float = 0.5, timeout_s: float = 60.0):
        t0 = time.time()
        rospy.loginfo("等待到达高度 %.1f m (ENU z) ...", z_target)
        while not rospy.is_shutdown():
            z = self.local_pos.pose.position.z
            if z > z_target - tol:
                rospy.loginfo("到达悬停高度: z=%.2f", z)
                return
            if time.time() - t0 > timeout_s:
                rospy.logerr("爬升超时")
                sys.exit(1)
            rospy.sleep(0.2)

    def run(self):
        self.wait_connected()
        self._wait_local_position()

        self.set_param("COM_RCL_EXCEPT", 4)

        rospy.sleep(2.0)

        cx = self.local_pos.pose.position.x
        cy = self.local_pos.pose.position.y
        self._pos_sp = PoseStamped()
        self._pos_sp.header.stamp = rospy.Time.now()
        self._pos_sp.header.frame_id = "map"
        self._pos_sp.pose.position.x = cx
        self._pos_sp.pose.position.y = cy
        self._pos_sp.pose.position.z = self.hover_z
        q = quaternion_from_euler(0.0, 0.0, 0.0)
        self._pos_sp.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

        self._phase = "hover"
        self._ctl_stop.clear()
        self._ctl_thread = threading.Thread(target=self._thread_offboard_stream)
        self._ctl_thread.daemon = True
        self._ctl_thread.start()
        rospy.sleep(2.0)

        self.set_mode("OFFBOARD")
        self.set_arm(True)

        self._wait_altitude(self.hover_z)

        rospy.loginfo("目标已悬停；本机地面待机。启动键盘遥控。")
        rospy.loginfo("WASD：W前 S后 A左 D右 | 对向键刹停该轴 | E升 Q降 | Ctrl+C 退出")

        self._start_keyboard()
        with self._lock:
            self._phase = "teleop"

        rospy.spin()


def main():
    try:
        TargetEscapeSequence().run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
