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

## 模块边界

- 它是控制器插件，不是独立业务节点
- 它只负责局部路径跟踪，不负责建图、定位和决策
- 它需要依赖上游路径、定位和代价地图输入才能工作
