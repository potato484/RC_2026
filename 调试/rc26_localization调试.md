# rc26_localization 调试

## 模块定位

`rc26_localization` 是 R2 当前的激光重定位主模块，基于 `rc26_small_gicp` 和 GTSAM 做局部配准与可选图后端。

## 适用场景

- 排查 `map -> odom` 为什么不稳定
- 调定位健康、鲁棒核和退化检测
- 用已有 PCD 做单独重定位联调

## 前置条件

- 已准备先验点云，推荐路径：
  - `${RC26_WS:-$HOME/RC_2026}/src/rc26_bringup/pcd/default.pcd`
  - `${RC26_WS:-$HOME/RC_2026}/src/rc26_point_lio/PCD/scans.pcd`
- 上游 `/odom`、`/registered_scan` 正常
- `odom -> base_link` TF 已存在

## 标准编译

```bash
cd "${RC26_WS:-$HOME/RC_2026}"
MAKEFLAGS='-j2 -l2' colcon build --executor sequential --parallel-workers 1 \
  --packages-select rc26_localization rc26_bringup
source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"
```

## 推荐启动

基础定位链：

```bash
ros2 launch rc26_bringup test_localization_chain.launch.py \
  prior_pcd_file:=${RC26_WS:-$HOME/RC_2026}/src/rc26_point_lio/PCD/scans.pcd
```

重定位联调：

```bash
ros2 launch rc26_bringup test_relocalization.launch.py \
  prior_pcd_file:=${RC26_WS:-$HOME/RC_2026}/src/rc26_point_lio/PCD/scans.pcd
```

回环联调：

```bash
ros2 launch rc26_bringup test_loop_closure.launch.py \
  prior_pcd_file:=${RC26_WS:-$HOME/RC_2026}/src/rc26_point_lio/PCD/scans.pcd
```

只调定位本体：

```bash
ros2 launch rc26_bringup localization.launch.py
```

## 最小验收

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 topic echo /localization/pose_with_cov --once
ros2 topic echo /localization/diagnostics --once
ros2 topic echo /localization/health --once
ros2 topic echo /localization/backend_status --once
ros2 topic echo /localization/route_observability --once
```

## 常用在线调参

```bash
ros2 param set /localization gicp_optimizer_mode "gn_auto"
ros2 param set /localization robust_enable true
ros2 param set /localization huber_c 1.0
ros2 param set /localization hessian_degen_enable true
ros2 param set /localization hessian_lambda_hard 10.0
```

## 优先排查

- `map -> odom` 不发布：先确认 `slam:=false`，再确认 `prior_pcd_file` 路径真实存在。
- 诊断长期退化：先看 `h_min_eig`、`sigma_xy`、`sigma_yaw`，再决定是否打开鲁棒核或调阈值。
- 单包定位起得来但整车链不正常：优先确认上游 `/odom`、`/registered_scan` 和 prior PCD 是否与当前现场一致。

## 相关入口

- [定位启动](./定位启动.md)
- [重定位启动](./重定位启动.md)
- [回环启动](./回环启动.md)
- [导航启动](./导航启动.md)
- [rc26_small_gicp调试](./rc26_small_gicp调试.md)
