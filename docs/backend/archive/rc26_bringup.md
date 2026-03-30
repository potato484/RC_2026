# rc26_bringup

## 模块定位

`rc26_bringup` 是 R2 自动机器人整车链路的统一启动与装配包，负责把各个算法包、驱动包、可视化包和测试入口组织起来。

## 当前实现

- 主启动文件：`src/rc26_bringup/launch/bringup.launch.py`
- 分链路启动：`localization.launch.py`、`odometry.launch.py`、`odometry_mock.launch.py`、`realsense_d455.launch.py`
- 单模块测试：`test_localization.launch.py`、`test_odom_interface.launch.py`、`test_odometry_chain.launch.py`、`test_omni_controller.launch.py`、`test_sensor_scan.launch.py`
- 关键配置：`config/localization.yaml`、`config/nav2_params.yaml`、`config/odom_interface.yaml`、`config/sensor_scan_generation.yaml`、`config/realsense_d455.yaml`
- 辅助脚本：`scripts/mock_point_lio.py`、`scripts/r2_acceptance_probe.py`、`scripts/render_foxglove_layouts.py`

`bringup.launch.py` 当前会把以下链路装配到一起：

- 里程计链：`rc26_mid360_driver`、`rc26_point_lio`、`rc26_odom_interface`、`rc26_sensor_scan`
- 定位链：`rc26_localization`
- 基础地形链：`rc26_base_ground`、`rc26_terrain`
- 导航链：Nav2、自定义控制器、`rc26_nav_mode_manager`
- 安全/规则链：`rc26_kfs_keepout`、`rc26_terrain_nav2`
- 决策链：`rc26_decision`
- 状态聚合：`rc26_visualization`
- 可选可视化：RViz、Foxglove

## 模块边界

- 这个包本身不承载核心算法，主要职责是参数装配、进程拉起和链路编排
- 它不是前端工程，`foxglove/` 里放的是布局模板
- 当某个算法异常时，通常真正要修的是下游具体功能包，而不是 `bringup` 本身
