# rc26_nmpc_controller

`rc26_nmpc_controller` 是面向 R2 四驱麦克纳姆底盘的定位感知控制器插件，运行于 Nav2 `controller_server`。
该模块以 `rc26_omni_controller` 作为安全回退内核，在定位质量正常时执行 NMPC 约束求解，在定位风险或求解异常时自动回退到 `FollowPath`，保证控制链路连续和可恢复。

## 模块背景与目标平台

- 目标平台：R2 自动机器人（Qualcomm QCS8550，Ubuntu 22.04 + ROS 2 Humble）
- 目标问题：在定位质量波动场景中，控制器不仅要跟踪路径，还要根据定位健康度主动收敛到保守控制策略
- 对接方式：作为 `nav2_core::Controller` 插件加载，不改动 Nav2 核心框架

## 核心特性

### 1. 定位感知控制闭环

控制器直接订阅以下输入并参与控制决策：

- `/control_state`：底盘状态反馈（速度测量）
- `/localization/health`：定位健康分级（GREEN/YELLOW/ORANGE/RED）
- `/localization/backend_status`：图后端状态（optimizer_ready / graph_health / imu_spike 等）

控制输出会根据上述健康度自动缩放，定位质量下降时优先保守而不是仅依赖外层 profile 限速。

### 2. 固定求解器路线（OSQP）

模块构建链固定使用：

- `find_package(osqp_vendor REQUIRED)`
- `find_package(osqp REQUIRED CONFIG)`
- 链接 `osqp::osqp`

当前实现采用首版 1-step RTI QP 内层求解，便于在 30Hz 控制周期下稳定运行并可复现。

### 3. 回退条件与回退目标固定

满足以下任一条件进入回退模式：

- `LHI == RED`
- 求解超时连续超过阈值
- QP 不可行连续超过阈值

回退后控制输出由 `rc26_omni_controller::OmniPidPursuitController` 负责，同时触发定位 profile：

- `loc_red_hold`（RED 场景）
- `loc_orange`（求解异常场景）

### 4. 限斜率交接防抖

NMPC 与 FollowPath 之间采用限斜率交接（`handover_a_lin`, `handover_a_ang`），防止模式切换时速度尖峰和轨迹抖动。

### 5. 可观测运行模式

插件发布运行模式话题：

- `/NMPCFollowPath/mode`

常见值包括：

- `nmpc`
- `fallback:loc_red`
- `fallback:solver_timeout`
- `fallback:solver_infeasible`

用于快速定位当前控制链是否已进入回退态。

## 模块结构与使用

- 插件类：`rc26_nmpc_controller::NmpcController`
- 插件导出：`rc26_nmpc_controller.xml`
- 主配置：`src/rc26_bringup/config/nav2_params.yaml`
- 测试配置：`src/rc26_bringup/config/nav2_test_omni_controller.yaml`

典型运行方式：

```bash
cd "${RC26_WS:-$HOME/RC_2026}"
colcon build --symlink-install --parallel-workers 3 --packages-select rc26_nmpc_controller rc26_bringup --cmake-args -DCMAKE_BUILD_TYPE=Release
source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"
ros2 launch rc26_bringup test_omni_controller.launch.py
```

更细的调试步骤（参数注入、回退场景复现、日志判读）见：

- `docs/debug_guide.md`
