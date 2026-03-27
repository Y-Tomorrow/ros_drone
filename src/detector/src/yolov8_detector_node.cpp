/**
 * YOLOv8 检测节点：使用 OpenCV DNN 加载 Ultralytics 导出的 ONNX。
 * 原始 best_v8.pt 需先导出：见 ros_drone 包内 scripts/export_yolov8_onnx.py
 */

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

#include <algorithm>
#include <cstdio>
#include <string>
#include <vector>

#include <std_msgs/Header.h>

namespace {

struct LetterboxMeta {
  float gain{1.f};
  float pad_x{0.f};
  float pad_y{0.f};
};

void letterboxUltralytics(const cv::Mat& src, cv::Mat& dst, int new_size, LetterboxMeta& meta) {
  const int h = src.rows;
  const int w = src.cols;
  meta.gain = std::min(static_cast<float>(new_size) / h, static_cast<float>(new_size) / w);
  const int new_w = static_cast<int>(std::round(w * meta.gain));
  const int new_h = static_cast<int>(std::round(h * meta.gain));
  cv::Mat resized;
  cv::resize(src, resized, cv::Size(new_w, new_h), 0, 0, cv::INTER_LINEAR);

  const float dw = (new_size - new_w) / 2.f;
  const float dh = (new_size - new_h) / 2.f;
  const int top = static_cast<int>(std::round(dh - 0.1f));
  const int bottom = static_cast<int>(std::round(dh + 0.1f));
  const int left = static_cast<int>(std::round(dw - 0.1f));
  const int right = static_cast<int>(std::round(dw + 0.1f));

  cv::copyMakeBorder(resized, dst, top, bottom, left, right, cv::BORDER_CONSTANT,
                     cv::Scalar(114, 114, 114));
  meta.pad_x = static_cast<float>(left);
  meta.pad_y = static_cast<float>(top);
}

void scaleCoordsBack(float cx, float cy, float bw, float bh, const LetterboxMeta& meta,
                     int orig_w, int orig_h, cv::Rect2f& box_xyxy) {
  const float cx_o = (cx - meta.pad_x) / meta.gain;
  const float cy_o = (cy - meta.pad_y) / meta.gain;
  const float w_o = bw / meta.gain;
  const float h_o = bh / meta.gain;
  box_xyxy.x = cx_o - w_o * 0.5f;
  box_xyxy.y = cy_o - h_o * 0.5f;
  box_xyxy.width = w_o;
  box_xyxy.height = h_o;
  box_xyxy &= cv::Rect2f(0, 0, static_cast<float>(orig_w), static_cast<float>(orig_h));
}

}  // namespace

class Yolov8DetectorNode {
 public:
  Yolov8DetectorNode() : nh_(), pnh_("~") {
    std::string onnx_path;
    pnh_.param<std::string>("onnx_model_path", onnx_path, "");
    pnh_.param<std::string>("image_topic", image_topic_, "/camera/image_raw");
    pnh_.param<std::string>("annotated_image_topic", annotated_topic_, "annotated_image");
    pnh_.param<double>("conf_threshold", conf_th_, 0.25);
    pnh_.param<double>("nms_threshold", nms_th_, 0.45);
    pnh_.param<int>("input_size", input_size_, 640);
    pnh_.param<bool>("publish_annotated", publish_annotated_, true);

    if (onnx_path.empty()) {
      ROS_FATAL("~onnx_model_path 未设置。请先用 scripts/export_yolov8_onnx.py 从 best_v8.pt 导出 ONNX，"
                "并在 launch 中设置 onnx_model_path。");
      ros::shutdown();
      return;
    }

    try {
      net_ = cv::dnn::readNetFromONNX(onnx_path);
    } catch (const cv::Exception& e) {
      ROS_FATAL("加载 ONNX 失败: %s — 路径: %s", e.what(), onnx_path.c_str());
      ros::shutdown();
      return;
    }

    net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

    sub_ = nh_.subscribe(image_topic_, 1, &Yolov8DetectorNode::imageCb, this);
    if (publish_annotated_) {
      pub_ = nh_.advertise<sensor_msgs::Image>(annotated_topic_, 1);
    }

    ROS_INFO("YOLOv8 节点就绪。订阅: %s, ONNX: %s", image_topic_.c_str(), onnx_path.c_str());
  }

 private:
  void imageCb(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      ROS_WARN_THROTTLE(2.0, "cv_bridge 转换失败: %s", e.what());
      return;
    }

    const cv::Mat& orig = cv_ptr->image;
    if (orig.empty()) {
      return;
    }

    LetterboxMeta lb;
    cv::Mat lb_img;
    letterboxUltralytics(orig, lb_img, input_size_, lb);

    cv::Mat blob = cv::dnn::blobFromImage(lb_img, 1.0 / 255.0, cv::Size(), cv::Scalar(), true, false);
    net_.setInput(blob);

    cv::Mat out;
    try {
      out = net_.forward();
    } catch (const cv::Exception& e) {
      ROS_WARN_THROTTLE(1.0, "推理失败: %s", e.what());
      return;
    }

    // Ultralytics YOLOv8 ONNX: [1, 4+nc, A] 或 [1, A, 4+nc]
    if (out.dims != 3) {
      ROS_WARN_THROTTLE(2.0, "意外输出维度: %d", out.dims);
      return;
    }
    const int d1 = out.size[1];
    const int d2 = out.size[2];
    bool chw = d1 < d2;  // [1, 84, 8400]
    int num_anchors = 0;
    int nc = 0;
    if (chw) {
      nc = d1 - 4;
      num_anchors = d2;
    } else {
      nc = d2 - 4;
      num_anchors = d1;
    }
    if (nc <= 0) {
      ROS_WARN_THROTTLE(2.0, "类别数无效: nc=%d", nc);
      return;
    }

    const float* p = out.ptr<float>();

    auto read_pred = [&](int i, int row) -> float {
      if (chw) {
        return p[row * num_anchors + i];
      }
      return p[i * (4 + nc) + row];
    };

    std::vector<cv::Rect> boxes;
    std::vector<float> scores;
    std::vector<int> class_ids;

    for (int i = 0; i < num_anchors; ++i) {
      float cx = read_pred(i, 0);
      float cy = read_pred(i, 1);
      float w = read_pred(i, 2);
      float h = read_pred(i, 3);

      float best_score = 0.f;
      int best_cls = -1;
      for (int c = 0; c < nc; ++c) {
        const float s = read_pred(i, 4 + c);
        if (s > best_score) {
          best_score = s;
          best_cls = c;
        }
      }
      if (best_score < static_cast<float>(conf_th_)) {
        continue;
      }

      cv::Rect2f xyxy;
      scaleCoordsBack(cx, cy, w, h, lb, orig.cols, orig.rows, xyxy);
      boxes.push_back(cv::Rect(
          static_cast<int>(std::round(xyxy.x)),
          static_cast<int>(std::round(xyxy.y)),
          static_cast<int>(std::round(xyxy.width)),
          static_cast<int>(std::round(xyxy.height))));
      scores.push_back(best_score);
      class_ids.push_back(best_cls);
    }

    std::vector<int> nms_idx;
    cv::dnn::NMSBoxes(boxes, scores, static_cast<float>(conf_th_), static_cast<float>(nms_th_), nms_idx);

    cv::Mat vis = orig.clone();
    for (int idx : nms_idx) {
      const cv::Rect& r = boxes[idx];
      cv::rectangle(vis, r, cv::Scalar(0, 255, 0), 2);
      char buf[128];
      std::snprintf(buf, sizeof(buf), "cls%d %.2f", class_ids[idx], scores[idx]);
      cv::putText(vis, buf, cv::Point(r.x, std::max(0, r.y - 5)), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                  cv::Scalar(0, 255, 0), 1);
    }

    ROS_INFO_THROTTLE(1.0, "检测 %zu 目标（NMS 后 %zu）", scores.size(), nms_idx.size());

    if (publish_annotated_ && pub_.getNumSubscribers() > 0) {
      std_msgs::Header hdr;
      hdr.stamp = msg->header.stamp;
      hdr.frame_id = msg->header.frame_id;
      cv_bridge::CvImage out_msg(hdr, sensor_msgs::image_encodings::BGR8, vis);
      pub_.publish(out_msg.toImageMsg());
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  cv::dnn::Net net_;
  std::string image_topic_;
  std::string annotated_topic_;
  double conf_th_{0.25};
  double nms_th_{0.45};
  int input_size_{640};
  bool publish_annotated_{true};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "yolov8_detector_node");
  Yolov8DetectorNode node;
  if (!ros::ok()) {
    return 1;
  }
  ros::spin();
  return 0;
}
