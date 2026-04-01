# rc26_omni_controller

## 模块定位

`rc26_omni_controller` 是 R2 麦克纳姆底盘控制宿主包，当前同时提供 Nav2 局部控制器插件与 `xhu_direct` 独立跟踪节点实现。

## 当前实现

- 构建产物：
  - Nav2 控制器插件库 `rc26_omni_controller`
  - 独立可执行节点 `xhu_motion_follower_node`
- 插件描述：`src/rc26_omni_controller/rc26_omni_controller.xml`
- 核心源码：
  - `src/rc26_omni_controller/src/omni_pid_pursuit_controller.cpp`
  - `src/rc26_omni_controller/src/pid.cpp`
  - `src/rc26_omni_controller/src/xhu_motion_follower.cpp`

`xhu_motion_follower_node` 在 `xhu_direct` 模式下承担持续速度输出，主要接口如下：

- 订阅：
  - `/xhu_nav/corridor_cmd` (`XhuSemanticCorridor`)
  - `/xhu_nav/motion_mode_state` (`XhuMotionModeState`)
  - `/localization/health`、`terrain_features`、`odom_topic`（默认 `control_state`）
- 发布：
  - `cmd_vel`
  - `/xhu_nav/lookahead_path`
  - `/xhu_nav/tracking_state`
  - `/xhu_nav/semantic_gate`

当前实现强调以下控制特性：

- 麦克纳姆底盘的全向跟踪
- 切线跟踪与横向误差修正
- 曲率前馈角速度
- 渐进制动和安全速度约束

## 2026-04 `xhu_motion_follower` 收口

- `xhu_motion_follower` 已拆成显式 `hpp/cpp`，运行时状态改为共享 corridor 指针快照，避免控制循环每帧复制整条 `Path`。
- direct 跟踪现在把 `XhuMotionModeState` 视为单一约束权威；模式状态缺失或过期时会先进入 `HOLD`，而不是继续沿用旧限速盲跑。
- TF / odom 暂时不可用不再立即硬失败，而是先 `HOLD` 等待恢复；只有超过 `hold_to_abort_sec` 才转 `ABORT`。
- 地形门控已从“只查机器人点和 lookahead 点”升级为沿当前走廊前视段做 stop-envelope 采样，并在横向误差过大时直接请求 `REPLAN`。

## 源码入口与阅读顺序
- 先看 `rc26_omni_controller.xml` 和 `README.md`，确认插件与独立节点两条运行路径。
- 再看 `src/omni_pid_pursuit_controller.cpp`，理解 Nav2 插件路径。
- 然后看 `src/xhu_motion_follower.cpp`，理解 `xhu_direct` 下的 corridor 跟踪与语义门控。
- 最后回到 `src/rc26_bringup/config/nav2_params.yaml`、`src/rc26_bringup/config/xhu_motion_follower.yaml` 和 `docs/debug_guide.md`。

## 目录解剖
- `omni_pid_pursuit_controller.cpp`：配置、地形缩放、路径变换、碰撞检测、曲率前馈、速度限幅和动态参数。
- `pid.cpp`：PID 计算辅助。
- `xhu_motion_follower.cpp`：独立跟踪循环、模式约束、地形门控、tracking state 反馈。
- `rc26_omni_controller.xml`：Nav2 插件导出。
- `README.md` / `docs/debug_guide.md`：控制语义、调试话题和实车标定路径。

## 关键文件体量
- `src/omni_pid_pursuit_controller.cpp`：2135 行，是当前控制器实现最厚的单文件之一。
- `src/pid.cpp`：94 行。
- `src/xhu_motion_follower.cpp`：604 行。
- `README.md`：35 行。

## 关键源码行段速览
- `src/rc26_omni_controller/src/omni_pid_pursuit_controller.cpp:43-313`：`configure()`，参数声明和接口初始化。
- `src/rc26_omni_controller/src/omni_pid_pursuit_controller.cpp:436-693`：costmap snapshot、pose covariance 订阅和参数清洗。
- `src/rc26_omni_controller/src/omni_pid_pursuit_controller.cpp:694-913`：动态参数校验与运行态重置。
- `src/rc26_omni_controller/src/omni_pid_pursuit_controller.cpp:928-1148`：terrain grid 订阅与 terrain scale 评估。
- `src/rc26_omni_controller/src/omni_pid_pursuit_controller.cpp:1149-1455`：`computeVelocityCommands()` 主控制回路。
- `src/rc26_omni_controller/src/omni_pid_pursuit_controller.cpp:1488-1975`：计划变换、碰撞检测、曲率与动态参数回调。
- `src/rc26_omni_controller/src/xhu_motion_follower.cpp:50-141`：节点参数、topic 订阅/发布初始化；`345-548`：主控制回路（lookahead、LHI 缩放、速度/加速度限幅）；`549-604`：运行态缓存与主函数。

## 模块边界

- 它既包含 Nav2 控制器插件，也包含 `xhu_direct` 的独立执行节点
- 它只负责局部路径跟踪，不负责建图、定位和决策
- Nav2 插件路径依赖 costmap；`xhu_motion_follower` 路径依赖 corridor + odom/tf + 地形/定位语义输入
