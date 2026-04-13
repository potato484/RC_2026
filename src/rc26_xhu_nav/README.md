# rc26_xhu_nav

`rc26_xhu_nav` 是 R2 当前唯一的 3D 导航实现宿主包。

## 当前职责

- 统一承载 topo graph / surface graph / corridor 生成
- 统一承载 body-aware surface planner、local 3D planner 和 motion mode manager
- 统一承载 `topo_nav_node`、`xhu_motion_mode_manager_node`、`xhu_motion_runtime_node`
- 对外继续通过 `navigate_topo_target`、`navigate_surface_route`、`set_xhu_motion_mode` 和 `/xhu_nav/*` 契约工作

## 当前边界

- 不拥有 `rc26_interfaces` 的接口定义权
- 不拥有 `rc26_bringup` 的装配权
- 不拥有 `rc26_terrain`、`rc26_base_ground`、`rc26_kfs_keepout`、`rc26_localization` 的状态真源
- 运行时唯一 `cmd_vel` 权威是 `xhu_motion_runtime_node`

## 当前运行时口径

- `topo_nav_node` 负责 action、graph、overlay、route 和 corridor 调度
- `xhu_motion_mode_manager_node` 负责模式切换、停稳预检查和 watchdog 回退
- `xhu_motion_runtime_node` 负责 local planner 评分、recovery 状态和 `cmd_vel` 输出
- legacy `xhu_motion_follower_node` 与 observe-only `local_3d_planner_node` 已退出主链

## 关键资产

- `config/`: topo graph、surface graph、runtime、nav profile 配置
- `scripts/`: graph 生成与渲染脚本
- `sim_assets/`: 导航图对应的 Gazebo 资产
- `scenarios/`: local planner 快照场景

## 当前说明

- 这是一次“多包实现收口到单包宿主”的架构变更；旧导航实现包已被新包取代
- 外部 ROS topic / service / action 名保持不变，变化只发生在实现归属、源码路径和文档口径

## 调试入口

- 运行时与 mode manager 的当前调试说明统一收口到仓库根目录 `调试/rc26_xhu_nav调试.md`
