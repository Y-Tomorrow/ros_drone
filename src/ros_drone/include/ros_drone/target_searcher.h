/**
 * @file target_searcher.h
 * @brief 目标丢失后的搜索策略（总管在短焦+长焦均已丢失后切入）
 */
#pragma once

#include <ros_drone/task_contract.h>

#include <Eigen/Dense>
#include <string>

/** WIDE 策略数值，默认与 config/target_search.yaml 中 wide_search 一致；可由 load_search_config 覆盖 */
struct TargetSearchWideParams {
  float max_horizontal_speed{0.35f};
  float max_vertical_speed{0.22f};
  float max_horizontal_travel_m{4.f};
  float max_vertical_travel_m{2.5f};
  float gimbal_yaw_gain_rad{0.55f};
  float gimbal_pitch_gain_rad{0.45f};
  float gimbal_pitch_limit_rad{0.65f};
  float body_yaw_rate{0.08f};
  bool yaml_loaded{false};
};

class TargetSearcher {
 public:
  TargetSearcher() = default;

  /**
   * 从共用 YAML 加载参数（当前解析 wide_search；后续可在同一文件加 tele_search 等并在本函数内扩展）。
   * @return 是否成功读到文件且含 wide_search；失败则保留内存中已有参数。
   */
  bool load_search_config(const std::string& yaml_path);

  /**
   * 切入搜索时调用（仅应在 WIDE 与 TELE 两路跟踪均丢失后调用）。
   *
   * @param lost_from_cam 丢失过程：
   *        NONE=两路从未检出；BOTH=同时丢；TELE=短焦先丢、最后在长焦丢；WIDE=长焦先丢、最后在短焦丢。
   * @param pixel 丢失前最后一帧，目标在「末次成像相机」上的归一化坐标：
   *              中心为原点，x∈[-1,1] 向右为正，y∈[-1,1] 向上为正。
   *              NONE 或无测量时传 (0,0)；TELE 对应长焦图，WIDE 对应短焦图，BOTH 可传任一路最后一帧或合成。
   */
  void set_search_strategy(CameraID lost_from_cam, Eigen::Vector2f pixel);

  TaskResult update(float dt);

  void reset();

 private:
  CameraID _lost_cam{CameraID::NONE};
  Eigen::Vector2f _last_pixel{0.f, 0.f};
  float _search_time{0.f};
  Eigen::Vector3f _search_origin{0.f, 0.f, 0.f};
  TargetSearchWideParams _wide_params{};
};
