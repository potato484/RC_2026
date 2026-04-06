# rc26_bringup

## 模块定位

`rc26_bringup` 是 R2 整车链路的统一装配入口。

## 当前导航装配口径

- `slam:=false` 时装配 topo/xhu 自研导航链，并通过 `local_execution_backend` 选择局部执行后端：
  - `rc26_topo_nav`
  - `xhu_motion_mode_manager_node`
  - `xhu_motion_follower_node` 或 `xhu_motion_runtime_node`
  - `rc26_robot_geometry` 提供的共享几何配置文件
  - `rc26_decision`
  - `rc26_visualization`
- `local_execution_backend:=follower` 时，可通过 `enable_local_3d_planner_observe:=true` 额外拉起 observe-only `local_3d_planner_node`
- `slam:=true` 时不启动导航执行链
- 决策统一加载 `main_tree.xml`

## 当前关键文件

- 主启动:
  - [launch/bringup.launch.py](/home/potato/RC_2026/src/rc26_bringup/launch/bringup.launch.py)
- 子链路:
  - `launch/localization.launch.py`
  - `launch/odometry.launch.py`
  - `launch/realsense_d455.launch.py`
- 配置:
  - `config/localization.yaml`
  - `config/xhu_motion_follower.yaml`
  - `config/xhu_motion_runtime.yaml`
  - `config/odom_interface.yaml`
  - `config/sensor_scan_generation.yaml`
  - `config/realsense_d455.yaml`
- 脚本:
  - `scripts/r2_acceptance_probe.py`
  - `scripts/render_foxglove_layouts.py`

## 当前边界

- 负责装配，不承载算法本体
- 当前整车导航模式下只会启动自研 topo/xhu 链

## 近期实现说明

- 当前 bringup 的默认 `chassis_model` 已切到 `tracked_diff`，下游 `xhu_motion_follower` 会默认按履带差速模式装配。
- 当前新增 `robot_geometry_file` 与 `robot_geometry_profile` 两个 launch 参数，默认把 `rc26_robot_geometry/config/r2_body_geometry.yaml` 的 `compact` profile 同时装配给 `rc26_topo_nav`、`xhu_motion_follower`、`xhu_motion_runtime` 和 observe-only `local_3d_planner_node`。
- 当前新增 `local_execution_backend / enable_local_3d_planner_observe / local_3d_planner_config_file / xhu_motion_runtime_config_file` 四组导航执行相关参数。
- bringup 当前通过单选装配保证同一时刻只有一个局部执行器拥有 `cmd_vel` 权威；observe-only `local_3d_planner_node` 只发布状态与 preview，不接管运动输出。
