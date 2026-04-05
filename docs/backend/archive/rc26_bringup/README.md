# rc26_bringup

## 模块定位

`rc26_bringup` 是 R2 整车链路的统一装配入口。

## 当前导航装配口径

- `slam:=false` 时固定装配：
  - `rc26_topo_nav`
  - `xhu_motion_mode_manager_node`
  - `xhu_motion_follower_node`
  - `rc26_robot_geometry` 提供的共享几何配置文件
  - `rc26_decision`
  - `rc26_visualization`
- `slam:=true` 时不启动导航执行链
- 决策统一加载 `main_tree.xml`

## 当前关键文件

- 主启动:
  - [launch/bringup.launch.py](/home/aidlux/RC_2026/src/rc26_bringup/launch/bringup.launch.py)
- 子链路:
  - `launch/localization.launch.py`
  - `launch/odometry.launch.py`
  - `launch/realsense_d455.launch.py`
- 配置:
  - `config/localization.yaml`
  - `config/xhu_motion_follower.yaml`
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
- 当前新增 `robot_geometry_file` 与 `robot_geometry_profile` 两个 launch 参数，默认把 `rc26_robot_geometry/config/r2_body_geometry.yaml` 的 `compact` profile 同时装配给 `rc26_topo_nav` 和 `xhu_motion_follower`。
