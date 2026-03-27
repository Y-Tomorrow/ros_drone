#!/usr/bin/env python3
"""直接使用 Ultralytics 加载 .pt 权重推理（无需 ONNX / OpenCV DNN）。"""

import numpy as np
import rospy

try:
    import torch
except ImportError:
    torch = None
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge, CvBridgeError


def main():
    rospy.init_node("yolov8_detector_pt")
    bridge = CvBridge()

    model_path = rospy.get_param("~model_path", "")
    image_topic = rospy.get_param("~image_topic", "/uav0/camera/rgb/image_raw")
    annotated_topic = rospy.get_param("~annotated_image_topic", "annotated_image")
    conf = float(rospy.get_param("~conf_threshold", 0.25))
    iou = float(rospy.get_param("~nms_threshold", 0.45))
    imgsz = int(rospy.get_param("~input_size", 640))
    publish_annotated = bool(rospy.get_param("~publish_annotated", True))
    publish_target_track = bool(rospy.get_param("~publish_target_track", True))
    target_track_topic = rospy.get_param("~target_track_topic", "target_in_image")
    target_class_id = int(rospy.get_param("~target_class_id", -1))  # -1 表示任意类，取置信度最高
    # device: auto（默认，有 CUDA 用 GPU）| cpu | cuda | 0 | cuda:0 等，与 ultralytics 一致
    device_param = rospy.get_param("~device", "auto")

    if not model_path:
        rospy.logfatal("请设置私有参数 ~model_path，例如 best_v8.pt 的绝对路径。")
        return

    try:
        from ultralytics import YOLO
    except ImportError as e:
        rospy.logfatal("需要 ultralytics：pip3 install ultralytics  （并安装 PyTorch）\n%s", e)
        return

    def resolve_device(spec):
        """
        返回 ultralytics predict / PyTorch 可用的 device。
        注意：PyTorch 的 module.to() 不接受字符串 '0'，必须用 int 0 或 'cuda:0'。
        """
        s = (spec or "auto").strip().lower()
        if s in ("auto", "", "default"):
            if torch is not None and getattr(torch, "cuda", None) is not None and torch.cuda.is_available():
                return 0
            return "cpu"
        if s in ("cuda", "gpu"):
            return 0
        if s in ("0", "cuda:0"):
            return 0
        if s == "cpu":
            return "cpu"
        # 其它透传：如 'cuda:1' 字符串给 ultralytics
        return spec.strip() or "cpu"

    device = resolve_device(device_param)

    if torch is not None:
        rospy.loginfo(
            "PyTorch %s | cuda.is_available=%s | cuda=%s",
            torch.__version__,
            torch.cuda.is_available(),
            getattr(torch.version, "cuda", None),
        )
        if torch.cuda.is_available():
            rospy.loginfo("CUDA 设备: %s", torch.cuda.get_device_name(0))

    rospy.loginfo("加载 YOLO 权重: %s", model_path)
    model = YOLO(model_path)
    try:
        model.to(device)
    except Exception as e:
        rospy.logwarn("model.to(%s) 失败，改用 cpu: %s", device, e)
        device = "cpu"
        model.to("cpu")
    rospy.loginfo("YOLO 使用设备: %r (~device=%s)", device, device_param)

    pub = rospy.Publisher(annotated_topic, Image, queue_size=1)
    track_pub = (
        rospy.Publisher(target_track_topic, Float64MultiArray, queue_size=1)
        if publish_target_track
        else None
    )
    if publish_target_track and track_pub is not None:
        rospy.loginfo(
            "target track 发布话题(解析后): %s",
            rospy.resolve_name(target_track_topic),
        )

    def cb(msg):
        try:
            cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logwarn_throttle(2.0, "cv_bridge: %s", e)
            return
        results = model.predict(
            cv_image, imgsz=imgsz, conf=conf, iou=iou, device=device, verbose=False
        )
        if not results:
            return
        r0 = results[0]
        h, w = cv_image.shape[:2]
        if publish_target_track and track_pub is not None:
            tmsg = Float64MultiArray()
            boxes = r0.boxes
            if boxes is not None and len(boxes) > 0:
                confs = boxes.conf.cpu().numpy()
                clss = boxes.cls.cpu().numpy().astype(int)
                xyxy = boxes.xyxy.cpu().numpy()
                mask = np.ones(len(confs), dtype=bool)
                if target_class_id >= 0:
                    mask &= clss == target_class_id
                if np.any(mask):
                    idxs = np.where(mask)[0]
                    bi = idxs[np.argmax(confs[idxs])]
                    x1, y1, x2, y2 = xyxy[bi]
                    cx = 0.5 * (x1 + x2)
                    cy = 0.5 * (y1 + y2)
                    # [cx, cy, w, h, conf, valid] 每帧都发，无检测时 valid=0，避免 YOLO 帧间隔误判「丢失」而一直扫描
                    tmsg.data = [float(cx), float(cy), float(w), float(h), float(confs[bi]), 1.0]
                else:
                    tmsg.data = [0.0, 0.0, float(w), float(h), 0.0, 0.0]
            else:
                tmsg.data = [0.0, 0.0, float(w), float(h), 0.0, 0.0]
            track_pub.publish(tmsg)
        annotated = r0.plot()
        if publish_annotated and pub.get_num_connections() > 0:
            try:
                out = bridge.cv2_to_imgmsg(annotated, "bgr8")
            except CvBridgeError as e:
                rospy.logwarn_throttle(2.0, "cv2_to_imgmsg: %s", e)
                return
            out.header = msg.header
            pub.publish(out)

    rospy.Subscriber(image_topic, Image, cb, queue_size=1, buff_size=2**24)
    rospy.loginfo("detector(pt) 就绪。订阅: %s", image_topic)
    rospy.spin()


if __name__ == "__main__":
    main()
