# PX4 SITL：iris_cgo3（云台 + ROS 相机）启动与修改说明

本文档汇总为在 **Gazebo Classic + PX4 v1.13.x** 下启动 **带 CGO3 云台的 Iris** 并打通 **ROS Noetic 相机话题** 所做的有效修改与推荐流程。

---

## 1. 目标

- 使用自定义机模 `iris_cgo3`（Iris + `model://cgo3` 云台）。
- PX4 侧通过 airframe 启用云台挂载与仿真输出。
- ROS 侧能收到图像话题（如 `/iris_cgo3/camera/image_raw`），并可在 `rqt_image_view` / RViz 中查看。

---

## 2. 关键文件与作用

| 路径 | 作用 |
|------|------|
| `platforms/posix/cmake/sitl_target.cmake` | 在 `models` 列表中注册 `iris_cgo3`，使 `make px4_sitl gazebo_iris_cgo3` 成为合法目标。 |
| `Tools/sitl_run.sh` | 启动 Gazebo 时：加载 ROS1 的 `gazebo_ros_api_plugin`；为 `libgazebo_ros_camera.so` 补充 Gazebo 核心插件库路径；默认关闭在线模型库以减少网络报错。 |
| `Tools/sitl_gazebo/models/iris_cgo3/iris_cgo3.sdf.jinja` | 机模主 SDF 模板；内含顶层 **ROS 相机桥接**（见下节）。 |
| `Tools/sitl_gazebo/models/iris_cgo3/iris_cgo3.sdf` | 由 jinja **生成**的最终 SDF；`sitl_run.sh` 只加载 `*.sdf`，修改 jinja 后需重新生成。 |
| `Tools/sitl_gazebo/models/cgo3/cgo3.sdf` | 云台子模型；内含原 CGO3 相机与 `libgazebo_ros_camera` 配置（曾清理无效 XML、修正插件参数）。 |
| `ROMFS/px4fmu_common/init.d-posix/airframes/1071_iris_cgo3` | 自定义 airframe（若与构建 ROMFS 中编号不一致，以实际 `*_iris_cgo3` 为准）。 |

---

## 3. `sitl_run.sh` 中的三项实用修改

### 3.1 ROS1 下加载 Gazebo ROS API 插件

当环境中设置了 `ROS_VERSION=1`（Noetic）时，`gzserver` 增加：

`-s libgazebo_ros_api_plugin.so`

否则 `gazebo_ros` 相关插件无法正确初始化 ROS 节点。

### 3.2 修复 `libgazebo_ros_camera.so` 加载失败

错误表现为日志类似：

`Failed to load plugin ... libgazebo_ros_camera.so: libCameraPlugin.so: cannot open shared object file`

**原因**：ROS 相机插件依赖 Gazebo 自带的传感器核心库，需将系统目录加入 `LD_LIBRARY_PATH`，例如：

`/usr/lib/x86_64-linux-gnu/gazebo-11/plugins`

### 3.3 默认关闭在线模型库（可选但推荐）

若未事先设置 `GAZEBO_MODEL_DATABASE_URI`，则设为**空字符串**，避免在无外网或证书问题时终端刷屏：

`Unable to get model name[http://models.gazebosim.org/...]`、`SSL_ERROR_SYSCALL` 等。

需要在线拉模型时，启动前自行 `export GAZEBO_MODEL_DATABASE_URI=<官方 URI>` 覆盖即可。

---

## 4. 机模与 ROS 相机

### 4.1 必须存在 `iris_cgo3/iris_cgo3.sdf`

`sitl_run.sh` 按固定路径查找：

`Tools/sitl_gazebo/models/<model>/<model>.sdf`

若仅有 `iris_cgo3.sdf.jinja`，需生成：

```bash
cd /home/yy/PX4-Autopilot
python3 Tools/sitl_gazebo/scripts/jinja_gen.py \
  Tools/sitl_gazebo/models/iris_cgo3/iris_cgo3.sdf.jinja \
  Tools/sitl_gazebo
```

### 4.2 顶层 ROS 相机桥接（解决 Gazebo11 下嵌套模型 ROS 话题不注册）

在 `iris_cgo3` **顶层**增加固定于云台相机连杆的 `ros_camera_bridge`（`libgazebo_ros_camera.so`），并发布例如：

- `/iris_cgo3/camera/image_raw`
- `/iris_cgo3/camera/camera_info`

（具体话题名以 `iris_cgo3.sdf` / jinja 中 `imageTopicName`、`cameraInfoTopicName` 为准。）

### 4.3 CGO3 子模型 `cgo3.sdf`

对云台自带相机插件做了清理与参数收敛，避免 XML 注释块内混入非注释文本导致解析异常。

---

## 5. 推荐启动顺序（ROS 相机）

1. **终端 A**：ROS Master  

   ```bash
   source /opt/ros/noetic/setup.bash
   source /home/yy/ros_drone/devel/setup.bash   # 若有工作空间
   roscore
   ```

2. **终端 B**：PX4 + Gazebo  

   ```bash
   cd /home/yy/PX4-Autopilot
   make px4_sitl gazebo_iris_cgo3
   ```

3. **终端 C**：验证话题  

   ```bash
   source /opt/ros/noetic/setup.bash
   source /home/yy/ros_drone/devel/setup.bash
   rostopic list | grep -E "iris_cgo3|camera|image"
   rqt_image_view
   ```

---

## 6. 常见问题

| 现象 | 处理 |
|------|------|
| `unknown target 'gazebo-classic_iris_cgo3'` | 本版本使用 `gazebo_iris_cgo3`，且需已在 `sitl_target.cmake` 注册 `iris_cgo3`。 |
| `Model iris_cgo3 not found` | 确认已生成 `iris_cgo3.sdf` 且路径正确。 |
| `param` 在普通 shell 中不存在 | `param` 仅在 PX4 的 `pxh>` 或通过 MAVLink 使用；ROS 侧用 `rosparam` / `rostopic`。 |
| `SYS_AUTOSTART=10021` 与文件名 `1071` 不一致 | ROMFS 构建时会按仓库内 airframe 编号生成；以启动日志为准，或统一维护一份 `NNNN_iris_cgo3`。 |

---

## 7. 变更清单（便于代码审查）

- `platforms/posix/cmake/sitl_target.cmake`：增加 `iris_cgo3` 模型目标。
- `Tools/sitl_run.sh`：ROS1 API 插件、`LD_LIBRARY_PATH`（Gazebo-11 插件目录）、默认 `GAZEBO_MODEL_DATABASE_URI`。
- `Tools/sitl_gazebo/models/iris_cgo3/iris_cgo3.sdf.jinja`：顶层 ROS 相机桥接链路。
- `Tools/sitl_gazebo/models/cgo3/cgo3.sdf`：相机插件与 XML 清理。
- 生成物：`iris_cgo3.sdf` / `iris_cgo3.sdf.last_generated`（由 `jinja_gen.py` 维护）。

---

*文档随本机 PX4 树内实际修改整理；升级 PX4 大版本时请重新核对 `sitl_target.cmake` 与 `sitl_run.sh` 是否仍一致。*
