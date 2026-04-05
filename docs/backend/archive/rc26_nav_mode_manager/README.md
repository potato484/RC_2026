# rc26_nav_mode_manager

## 模块定位

`rc26_nav_mode_manager` 现在是 xhu 自研导航链的运动模式管理宿主包。

## 当前实现

- 构建产物:
  - `xhu_motion_mode_manager_node`
- 关键配置:
  - `config/nav_profiles.yaml`
- 启动文件:
  - `launch/xhu_motion_mode_manager.launch.py`

## 对外接口

- 服务:
  - `set_xhu_motion_mode`
- 发布:
  - `/xhu_nav/motion_mode_state`
- 订阅:
  - `control_state` 或配置中的 odom 话题

## 源码入口

- [src/xhu_motion_mode_manager.cpp](/home/aidlux/RC_2026/src/rc26_nav_mode_manager/src/xhu_motion_mode_manager.cpp)
- [src/profile_loader.cpp](/home/aidlux/RC_2026/src/rc26_nav_mode_manager/src/profile_loader.cpp)
- [src/profile_db.cpp](/home/aidlux/RC_2026/src/rc26_nav_mode_manager/src/profile_db.cpp)
- [src/watchdog_timer.cpp](/home/aidlux/RC_2026/src/rc26_nav_mode_manager/src/watchdog_timer.cpp)

## 当前边界

- 只负责模式切换、停稳预检查和 watchdog 回退
- 对外主链是 `set_xhu_motion_mode` 与 `/xhu_nav/motion_mode_state`
- 不负责 topo 图搜索和 corridor 执行
