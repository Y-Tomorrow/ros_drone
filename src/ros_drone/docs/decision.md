# 决策说明

本文件汇总追击目标丢失相关的**决策与行为约定**，按场景分节编写。

实现与参数路径以代码为准；当前与 `TargetSearcher` 相关的 YAML 在 `config/target_searcher.yaml`（ROS 私有命名空间 `~target_searcher/wide/*`）。

对应实现：`src/TargetSearcher.cpp`。

---
## 角色与接口

- **触发**：上层在判定 **WIDE（广角）** 丢失目标时调用  
  `set_search_strategy(CameraID::WIDE, pixel)`，其中 `pixel` 为二维归一化图像误差（标量范围由上层约定）。
- **运行**：以固定频率调用 `update(dt)`，输出 `TaskResult`：
  - `vel_vxyzy = (vx, vy, vz, yaw_rate)`：机体速度类指令（下游由 `target_searcher_tester_bin` 等映射到 MAVROS，单位 m/s 与 rad/s）。
  - `gimbal_rpy = (roll, pitch, yaw)`：云台姿态指令（弧度，具体与 `MountControl` 的映射见下游节点）。


## WIDE：广角丢失目标搜索（`TargetSearcher`，`CameraID::WIDE`）

### 低速门控（决定“从 0 起算”还是“沿用当前速度”）

对机体系 **vx / vy / vz / yaw_rate** 四个通道分别比较 `vxyzy_keep_threshold`：  
- **低速**（\(|当前|\) < 阈值）：该通道**从 0 起算**  
- **非低速**：该通道**沿用当前实测值**

注意：**vx 在开启 Task1 时会继续被 Task1 修改**（因此低速时 vx 不一定为 0）。

### 各任务在做什么（每个任务单独开关）

- **Task1（前向减速，`enable_task1_decel`）**  
  开启后：每次 `update(dt)` 都会让 **vx_cmd 变小**，每次减小量约为 `decel_mps2 * dt`，并且不低于 `min_vx_mps`（可为负，表示允许“后退”指令）。    
  最终 vx 还会被 `cmd_vx_limit_mps` 做绝对值限幅。

- **Task2（云台偏转，`enable_task2_gimbal_deflect`）**  
  开启后：以 **丢失目标那一瞬间的云台姿态** 为基准，根据 **像素误差** 让云台 **俯仰、偏航** 往「目标可能在的方向」偏一点（幅度有参数限幅）。  
  关闭后：云台指令基本维持在丢失瞬间的姿态。

- **Task3（机体平移，`enable_task3_translate`）**  
  开启后：在 **左右（vy）、上下（vz；NED 机体系里 z 向下为正）** 上按像素给小的速度修正；**只有 vy / vz 各自算低速时才改对应轴**。

- **Task4（机体偏航角速度，`enable_task3_yaw_rate`）**  
  开启后：按像素 **横向** 误差给 **机体绕竖直轴转动的角速度**；**只有偏航角速度算低速时才加这项**。若配置了 `body_yaw_limit_rad`，会限制相对切入搜索时的最大偏航角。

## 超时退出

若 `_search_time > timeout_s`：

- `gimbal_rpy` 与 `vel_vxyzy` 各分量均置为 **`+inf`**  



