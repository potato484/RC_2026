# 2026-04-01 当前导航架构状态

## 结论

- 当前项目还不是“已经完成的全场通用 3D 导航框架”。
- 当前真实落地的是 `xhu_direct` 首轮链路：以 `rc26_topo_nav + xhu_motion_mode_manager + xhu_motion_follower` 为核心的比赛特化 3D/2.5D 语义直连执行路径。
- 默认启动模式仍是 `navigation_stack_mode:=legacy`，`topo/topo_nav2` 与 Nav2 兼容执行路径仍然保留。
- `xhu_direct` 首轮只覆盖 MF topo 子树；MC、对抗区和全场统一目标表达仍未迁移完成。

## 已落地范围

- `rc26_bringup` 已支持 `legacy / topo(topo_nav2) / xhu_direct` 三态装配。
- `rc26_decision` 已新增 `main_tree_topo.xml` 与 `main_tree_xhu_direct.xml`，其中 `xhu_direct` 当前只挂载 `mf_tree_topo.xml`。
- `rc26_topo_nav` 已具备双执行后端：
  - `nav2_follow_path`
  - `xhu_direct`
- `rc26_nav_mode_manager` 已新增 `xhu_motion_mode_manager_node`，不再依赖 `controller_server` 参数写入和 costmap 清理。
- `rc26_omni_controller` 已新增 `xhu_motion_follower_node`，在 direct 模式下直接输出 `cmd_vel`。
- `rc26_interfaces` 已补齐 `xhu_direct` 首轮运行时所需的 corridor、mode state、tracking state 与 mode service 契约。

## 当前仍存在的断层

- 比赛主树还没有完成“全场统一走 topo/xhu 意图”的迁移；当前 `xhu_direct` 只覆盖 MF。
- Nav2 运行时仍是有效兼容路径，仓库还没有进入“正式比赛配置只保留 `xhu_direct`”阶段。
- `src/rc26_topo_nav/config/topo_nav.yaml` 的默认 `execution_backend` 仍是 `nav2_follow_path`，说明 direct 路径目前仍依赖 bringup 显式切模装配。
- `rc26_terrain_nav2`、`terrain_speed_limit_bridge` 等 Nav2 适配链仍为 `legacy/topo` 路径服务，尚未从整仓彻底退出。
- 当前多数实现宿主包仍保持 `rc26_*` 包名，`xhu_*` 主要体现在新增节点、接口和话题层面。

## 判断依据

- 核心方案准则：`MVP技术方案/3D导航重构版本/改进方案/改进方案2.md`
- 架构约束与落地口径：`docs/fitness/architecture_fitness_ros2_workspace/README.md`
- 装配入口：`src/rc26_bringup/launch/bringup.launch.py`
- 首轮 direct 决策树：`src/rc26_decision/behavior_trees/main_tree_xhu_direct.xml`
- topo 执行默认配置：`src/rc26_topo_nav/config/topo_nav.yaml`

## 本轮仓库清理

以下根目录外部参考仓库快照已删除：

- `ExplorationRRT-master/`
- `FUEL-main/`
- `Fast-Planner-master/`
- `PCT_planner-main/`
- `dddmr_navigation-main/`
- `mesh_navigation-main/`
- `move_base_flex-ros2/`
- `rtabmap-master/`
- `vox_nav-humble/`

清理原因：

- 这些目录不属于当前 R2 主运行时工作区，不在 `docs/` 约束定义的正式后端边界内。
- 它们在根目录中表现为未纳管的外部参考源码快照，容易污染 `git status` 和后续协作判断。
- 方案口径已经明确：吸收外部方案思想，不把整套外部导航仓库直接作为当前比赛运行时依赖引入。

## 未动项

- `src/`、`docs/`、`merlin-bt-visualizer/`、`MVP技术方案/` 保留。
- `build/`、`install/`、`log/`、`rosbags/` 未在本轮清理范围内。
