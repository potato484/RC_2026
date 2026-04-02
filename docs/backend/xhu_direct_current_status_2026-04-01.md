# 2026-04-02 当前导航架构状态

本文档沿用 2026-04-01 的文件名，继续记录 2026-04-02 完成旧兼容导航清理后的真实状态。

## 结论

- 当前仓库已经不再保留旧兼容导航运行时，也不再保留 dual-backend 兼容执行路径。
- 当前唯一有效的导航主链是：
  - `rc26_topo_nav`
  - `xhu_motion_mode_manager_node`
  - `xhu_motion_follower_node`
- `rc26_decision` 现统一使用 `main_tree.xml`，其中 MF 导航已收口到 topo/xhu 节点。
- `rc26_visualization`、`rc26_bringup`、`docs/middle` 等配套入口已同步切到 xhu 自研话题与服务。

## 本轮完成的清理

- 删除旧 waypoint / 安全 / 模式切换接口
- 删除旧二维执行器包与旧地形兼容桥接包
- 删除旧决策侧 waypoint 导航桥接和独立 waypoint 配置
- 删除旧插件化控制器实现与配套测试入口
- 删除 bringup 内旧兼容参数与测试入口
- 删除旧分模式主树，统一收口到 `main_tree.xml`

## 当前运行时口径

- `rc26_bringup` 在 `slam:=false` 时固定装配 topo/xhu 自研导航链
- `rc26_topo_nav` 不再暴露 `execution_backend`
- `rc26_nav_mode_manager` 只提供 `set_xhu_motion_mode`
- `rc26_omni_controller` 只保留 `xhu_motion_follower_node`
- `rc26_visualization` 只消费：
  - `/xhu_nav/semantic_gate`
  - `/xhu_nav/motion_mode_state`
  - `/xhu_nav/tracking_state`
  - `/mf_block_overlay`

## 仍需注意

- 包名仍然维持 `rc26_*`，这是宿主包历史名称，不代表仍有旧兼容导航运行时。
- `build/` 与 `install/` 中可能残留历史构建产物；判断当前真实实现时以 `src/` 与 `docs/` 为准。

## 判断依据

- [bringup.launch.py](/home/potato/RC_2026/src/rc26_bringup/launch/bringup.launch.py)
- [topo_nav.yaml](/home/potato/RC_2026/src/rc26_topo_nav/config/topo_nav.yaml)
- [nav_profiles.yaml](/home/potato/RC_2026/src/rc26_nav_mode_manager/config/nav_profiles.yaml)
- [main_tree.xml](/home/potato/RC_2026/src/rc26_decision/behavior_trees/main_tree.xml)
- [navigation.yaml](/home/potato/RC_2026/docs/middle/modules/navigation.yaml)
