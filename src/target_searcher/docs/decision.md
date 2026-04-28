# 决策说明（按任务顺序）


## 1. 开始任务：`set_search_strategy()`

- 读取丢失像素 `pixel_norm`。
- 根据像素离边界的距离分流：
  - 靠边 -> `WIDE`
  - 不靠边 -> `TELE`
- 记录丢失瞬间基准（机体 yaw、云台姿态、速度）。
- 初始化速度缓存：
  - 机体系缓存：`_last_cmd_vel`
  - 世界水平缓存：`_last_cmd_world_xy`（优先用 `getVelEnu()`）

## 2. 周期更新：`update(dt)`

每帧输出三项：
- `vel_vxyzy`
- `velocity_frame`（`BODY` 或 `ENU`）
- `gimbal_rpy`

流程顺序是：先执行主分支（`TELE` 或 `WIDE`），超时后降级到 `NONE`，再超时就输出 `INF` 结束。

## 3. TELE（远离边界）

- 先处理 `vy/vz/yaw_rate`：小于阈值就置 0，否则保持。
- `enable_task1_accel` 开启时做纵向加速。
- `longitudinal_cmd_in_body_frame` 决定纵向坐标系：
  - `true`：机体系纵向，输出 `BODY`
  - `false`：世界参考前向纵向，输出 `ENU`
- 世界模式下基于 `_last_cmd_world_xy` 继续更新，不是每帧从零开始。
- 云台按参数漂移，或保持丢失姿态。

## 4. WIDE（靠近边界）

- 先做低速门控（`wide.vxyzy_keep_threshold`）。
- 按开关执行：
  - task1：纵向减速
  - task2：云台偏转
  - task3：低速时平移和/或 yaw_rate 修正
- `longitudinal_cmd_in_body_frame` 决定输出：
  - `true`：`BODY`
  - `false`：`ENU`

## 5. 超时与降级

- `TELE` 或 `WIDE` 超时后：
  - `none.timeout_s > 0` -> 进入 `NONE_PATROL`
  - 否则 -> 直接 `INF`

## 6. NONE_PATROL

- 固定输出巡逻速度：`vx=0, vy=0, vz=0, yaw_rate=none.yaw_rate_rps`
- 云台按正弦规律摆动
- `velocity_frame` 固定 `BODY`
- `NONE` 超时后输出 `INF`

## 7. INF 含义

- `vel_vxyzy` 和 `gimbal_rpy` 全 `+inf` 表示任务结束。
- 上层应据此切换流程（返航、退出、下一任务等）。


