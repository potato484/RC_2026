# rc26_bringup

## 模块定位

`rc26_bringup` 是 R2 整车链路的统一装配入口。

## 当前导航装配口径

- `slam:=false` 时固定装配：
  - `rc26_topo_nav`
  - `xhu_motion_mode_manager_node`
  - `xhu_motion_follower_node`
  - `rc26_decision`
  - `rc26_visualization`
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
  - `config/odom_interface.yaml`
  - `config/sensor_scan_generation.yaml`
  - `config/realsense_d455.yaml`
- 脚本:
  - `scripts/r2_acceptance_probe.py`
  - `scripts/render_foxglove_layouts.py`

## 当前边界

- 负责装配，不承载算法本体
- 当前整车导航模式下只会启动自研 topo/xhu 链
