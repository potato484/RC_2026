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

## 源码入口与阅读顺序
- 先看 `launch/bringup.launch.py`，它是整车运行时的 composition root。
- 再按子链路看 `launch/odometry.launch.py`、`launch/localization.launch.py`、`launch/realsense_d455.launch.py`，确认每条运行链是怎么被 include 进来的。
- 然后回到 `config/nav2_params.yaml`、`config/localization.yaml`、`config/odom_interface.yaml` 看包归属参数。
- 最后看 `scripts/r2_acceptance_probe.py`、`foxglove/README.md` 和 `test/README.md`，理解验收和可视化是如何被挂进 bringup 的。

## 目录解剖
- `launch/`：真正的装配层，负责决定哪些包被拉起、是否启用可视化、如何切换 mock/real 路径。
- `config/`：对下游包参数文件的选择入口，不是把算法写进 YAML。
- `scripts/r2_acceptance_probe.py`：整车验收探针，把多条 topic 汇总成可读报告。
- `foxglove/`：布局模板和说明，属于可视化资产，不是诊断逻辑本体。
- `test/`：控制器和链路评估脚本。

## 关键文件体量
- `launch/bringup.launch.py`：643 行，整车总装逻辑集中。
- `launch/odometry.launch.py`：462 行，里程计子链装配很重。
- `config/nav2_params.yaml`：336 行，控制器/规划器参数量大。
- `config/localization.yaml`：333 行，定位参数入口。
- `scripts/r2_acceptance_probe.py`：660 行，验收逻辑接近一个独立工具。
- `test/eval_controller.py`：284 行，离线评估脚本。

## 关键源码行段速览
- `src/rc26_bringup/launch/bringup.launch.py:27-643`：整车 `generate_launch_description()`，从参数声明、路径解析、子链 include、可视化后端切换一路组到最终返回。
- `src/rc26_bringup/launch/odometry.launch.py:22-63`：布尔参数与 Point-LIO profile 解析；`64-123` 创建 Point-LIO/里程计动作；`124-462` 生成完整里程计链 LaunchDescription。
- `src/rc26_bringup/launch/localization.launch.py:15-113`：定位链参数声明与节点装配。
- `src/rc26_bringup/scripts/r2_acceptance_probe.py:86-574`：`AcceptanceProbe` 订阅与检查逻辑；`575-660`：CLI、报表输出和 `main()`。

## 模块边界

- 这个包本身不承载核心算法，主要职责是参数装配、进程拉起和链路编排
- 它不是前端工程，`foxglove/` 里放的是布局模板
- 当某个算法异常时，通常真正要修的是下游具体功能包，而不是 `bringup` 本身
