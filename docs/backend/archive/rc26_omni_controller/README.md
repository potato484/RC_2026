# rc26_omni_controller

## 模块定位

`rc26_omni_controller` 是 R2 麦克纳姆底盘的 Nav2 局部控制器插件，采用 PID + Pure Pursuit 的全向路径跟踪方案。

## 当前实现

- 构建产物：Nav2 控制器插件库 `rc26_omni_controller`
- 插件描述：`src/rc26_omni_controller/rc26_omni_controller.xml`
- 核心源码：
  - `src/rc26_omni_controller/src/omni_pid_pursuit_controller.cpp`
  - `src/rc26_omni_controller/src/pid.cpp`

当前实现强调以下控制特性：

- 麦克纳姆底盘的全向跟踪
- 切线跟踪与横向误差修正
- 曲率前馈角速度
- 渐进制动和安全速度约束

## 源码入口与阅读顺序
- 先看 `rc26_omni_controller.xml` 和 `README.md`，确认插件身份和控制目标。
- 再看 `src/omni_pid_pursuit_controller.cpp`，主体控制都在这里。
- 然后看 `src/pid.cpp`，理解误差控制子模块。
- 最后回到 `src/rc26_bringup/config/nav2_params.yaml` 和 `docs/debug_guide.md`。

## 目录解剖
- `omni_pid_pursuit_controller.cpp`：配置、地形缩放、路径变换、碰撞检测、曲率前馈、速度限幅和动态参数。
- `pid.cpp`：PID 计算辅助。
- `rc26_omni_controller.xml`：Nav2 插件导出。
- `README.md` / `docs/debug_guide.md`：控制语义、调试话题和实车标定路径。

## 关键文件体量
- `src/omni_pid_pursuit_controller.cpp`：2135 行，是当前控制器实现最厚的单文件之一。
- `src/pid.cpp`：94 行。
- `README.md`：35 行。

## 关键源码行段速览
- `src/rc26_omni_controller/src/omni_pid_pursuit_controller.cpp:43-313`：`configure()`，参数声明和接口初始化。
- `src/rc26_omni_controller/src/omni_pid_pursuit_controller.cpp:436-693`：costmap snapshot、pose covariance 订阅和参数清洗。
- `src/rc26_omni_controller/src/omni_pid_pursuit_controller.cpp:694-913`：动态参数校验与运行态重置。
- `src/rc26_omni_controller/src/omni_pid_pursuit_controller.cpp:928-1148`：terrain grid 订阅与 terrain scale 评估。
- `src/rc26_omni_controller/src/omni_pid_pursuit_controller.cpp:1149-1455`：`computeVelocityCommands()` 主控制回路。
- `src/rc26_omni_controller/src/omni_pid_pursuit_controller.cpp:1488-1975`：计划变换、碰撞检测、曲率与动态参数回调。

## 模块边界

- 它是控制器插件，不是独立业务节点
- 它只负责局部路径跟踪，不负责建图、定位和决策
- 它需要依赖上游路径、定位和代价地图输入才能工作
