# rc26_nav_mode_manager

`rc26_nav_mode_manager` 现在只保留 `xhu_motion_mode_manager_node`，负责自研导航链里的运动模式切换、停稳预检查和 watchdog 回退。

## 当前职责

- 提供服务: `set_xhu_motion_mode`
- 发布状态: `/xhu_nav/motion_mode_state`
- 订阅里程计: 默认 `control_state`
- 根据 `config/nav_profiles.yaml` 装载运动模式约束
- 在需要停稳的模式切换前做速度窗口检查
- 在复杂模式驻留超时后回退到配置中的 `fallback_profile`

## 当前有效配置

- [config/nav_profiles.yaml](/home/aidlux/RC_2026/src/rc26_nav_mode_manager/config/nav_profiles.yaml)
- [launch/xhu_motion_mode_manager.launch.py](/home/aidlux/RC_2026/src/rc26_nav_mode_manager/launch/xhu_motion_mode_manager.launch.py)

当前仓库已经清理旧模式桥接和旧配置资产，当前只保留 `xhu_motion_mode_manager_node` 及 `nav_profiles.yaml`。

## 源码入口

- [src/xhu_motion_mode_manager.cpp](/home/aidlux/RC_2026/src/rc26_nav_mode_manager/src/xhu_motion_mode_manager.cpp): 节点主实现
- [include/rc26_nav_mode_manager/xhu_motion_mode_manager.hpp](/home/aidlux/RC_2026/src/rc26_nav_mode_manager/include/rc26_nav_mode_manager/xhu_motion_mode_manager.hpp): 运行时状态与接口
- [src/profile_loader.cpp](/home/aidlux/RC_2026/src/rc26_nav_mode_manager/src/profile_loader.cpp): 配置解析与兼容字段校验

## 当前约束

- 模式权威只来自 `set_xhu_motion_mode` 和 `/xhu_nav/motion_mode_state`。
- 不再向其他执行器写运行时参数，也不再维护额外兼容清理逻辑。
- 任何新模式都应先在 `nav_profiles.yaml` 中定义，再由 topo edge 或决策逻辑引用。
