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

## 模块边界

- 它不是独立节点，而是被 `controller_server` 动态加载的插件
- 它只负责局部路径跟踪控制，不负责全局路径生成
- 它依赖定位和里程计质量，本身不生产这些输入
