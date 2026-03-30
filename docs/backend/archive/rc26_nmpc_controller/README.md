# rc26_nmpc_controller

## 模块定位

`rc26_nmpc_controller` 是挂载到 Nav2 `controller_server` 上的定位感知型 NMPC 控制器插件。

## 当前实现

- 构建产物：Nav2 控制器插件库 `rc26_nmpc_controller`
- 插件描述：`src/rc26_nmpc_controller/rc26_nmpc_controller.xml`
- 核心源码：`src/rc26_nmpc_controller/src/nmpc_controller.cpp`
- 构建特征：当前 CMake 已启用 OSQP 后端

当前插件实现的核心点包括：

- 在定位健康度正常时走 NMPC 求解
- 订阅定位健康度与图后端状态参与控制降级判断
- 求解失败或风险升高时回退到 `rc26_omni_controller`
- 以 Nav2 插件方式集成，不修改 Nav2 核心框架

## 源码入口与阅读顺序
- 先看 `rc26_nmpc_controller.xml` 和 `README.md`，确认它以 Nav2 插件身份出现。
- 再看 `src/nmpc_controller.cpp`，整个控制器都在单文件里。
- 最后回到 `src/rc26_bringup/config/nav2_params.yaml` 和 `docs/debug_guide.md`，确认插件如何被 controller_server 装进来。

## 目录解剖
- `nmpc_controller.cpp`：生命周期、计划接收、求解、fallback、terrain scale 和 slew-rate handover 全在这里。
- `rc26_nmpc_controller.xml`：Nav2 插件导出描述。
- `README.md`：控制模式、fallback 条件和调试 topic 入口。
- `docs/debug_guide.md`：最小复现步骤。

## 关键文件体量
- `src/nmpc_controller.cpp`：1053 行，单文件插件。
- `README.md`：84 行。
- `rc26_nmpc_controller.xml`：9 行。

## 关键源码行段速览
- `src/rc26_nmpc_controller/src/nmpc_controller.cpp:70-392`：`configure/cleanup/activate/deactivate/setPlan/setSpeedLimit`，生命周期与基础接口。
- `src/rc26_nmpc_controller/src/nmpc_controller.cpp:393-610`：`computeVelocityCommands()` 主路径、fallback 进入/退出和定位质量缩放。
- `src/rc26_nmpc_controller/src/nmpc_controller.cpp:611-879`：测量速度读取、slew-rate 限斜率、terrain layer 读取与 terrain scale。
- `src/rc26_nmpc_controller/src/nmpc_controller.cpp:880-1053`：受约束求解和尾部运行状态收敛。

## 模块边界

- 它不是独立节点，而是被 `controller_server` 动态加载的插件
- 它只负责局部路径跟踪控制，不负责全局路径生成
- 它依赖定位和里程计质量，本身不生产这些输入
