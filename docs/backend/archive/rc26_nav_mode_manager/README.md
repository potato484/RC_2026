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

## 当前清理状态

旧模式桥接节点、旧兼容消息与旧配置资产已移除，当前只保留 xhu 运动模式管理主线。

## 源码入口

- [src/xhu_motion_mode_manager.cpp](/home/potato/RC_2026/src/rc26_nav_mode_manager/src/xhu_motion_mode_manager.cpp)
- [src/profile_loader.cpp](/home/potato/RC_2026/src/rc26_nav_mode_manager/src/profile_loader.cpp)
- [src/profile_db.cpp](/home/potato/RC_2026/src/rc26_nav_mode_manager/src/profile_db.cpp)
- [src/watchdog_timer.cpp](/home/potato/RC_2026/src/rc26_nav_mode_manager/src/watchdog_timer.cpp)

## 当前边界

- 只负责模式切换、停稳预检查和 watchdog 回退
- 不再向其他执行器写参数
- 不再桥接 terrain profile
