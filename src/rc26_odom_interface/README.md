# rc26_odom_interface

`rc26_odom_interface` 是 R2 自动导航链里的底盘系规范化接口层。它消费 Point-LIO 的内部 body frame 里程计，并把自动链统一成：

`map -> odom -> base_footprint -> base_link -> livox_frame`

## 当前职责

- 订阅 Point-LIO 的 `/state_estimation`，要求 `child_frame_id` 为内部 `point_lio.body_frame`
- 接收 `rc26_bringup/launch/odometry.launch.py` 注入的 `base_link -> input_body_frame` 外参
- 继续作为自动链动态 TF 的唯一权威，发布：
  - `odom -> base_footprint`
  - `base_footprint -> base_link`
- 发布标准化 `/odom`，其中 `child_frame_id=base_footprint`
- 继续输出 `registered_scan`，保持在 `odom` 坐标系表达
- 启动阶段可先发布动态 bootstrap `/odom` 与 TF，给 Nav2 和下游节点提供稳定零速基座位姿；真实 Point-LIO odom 完成静止位姿归零并接管后停止 bootstrap
- `registered_scan` 与 odom 的短时缓存由 `cloud_queue_size`、`pending_cloud_queue_size` 和 `odom_stamp_history_size` 调节
- 低算力完整导航默认关闭 `odom_path` 与 `odom_pose_markers` 调试发布；需要 RViz 观察时再通过 `odom_interface.yaml` 打开

## 当前坐标语义

- `base_footprint`：导航使用的 2D 地面投影基座
- `base_link`：底盘最下层金属刚性主板中心
- `base_link` 相对 `base_footprint` 的高度由 `rc26_bringup/config/odom_interface.yaml` 提供，当前为 `0.065m`
- `base_link` 保留 Point-LIO 解算出来的 roll/pitch；`odom -> base_footprint` 只保留 `x/y/yaw`
- `base_link -> input_body_frame` 外参按 TF 语义注入；节点在位姿链中使用其 inverse 恢复 `odom -> base_link`，并把 Point-LIO twist 从 `input_body_frame` 转换到底盘 `base_link` 后再输出给导航链。
- `zero_origin_to_first_frame=true` 时，启动静止窗口会同时归零首帧平移和 yaw；例如 Mid-360 的 `livox_frame +X` 指向车体 `+Y` 时，传感器外参 yaw 仍应保持 `+90deg`，但输出 `/odom` 的初始 `base_footprint` yaw 会被归到 `0`，避免决策层误认为车体初始朝向差 90 度。

## 当前边界

- 只负责自动导航链的里程计与动态 TF 归一化，不承担遥控或目标 MCU 底盘执行
- 不负责里程计估计本体
- 不再查询或发布 `base_link -> point_lio.body_frame` 对外 TF；这段内部外参只在 bringup 装配期推导并注入
- 不直接做控制求解
- 不发布静态 `base_footprint -> base_link`；该边仍由本节点作为唯一权威动态发布

## 本轮同步

2026-07-03 同步：启动静止归零从只平移归零扩展为平移 + yaw 同时归零。`rc26_sensor_extrinsics` 继续表达真实雷达安装方向，不为了修正导航初始朝向去改传感器 yaw；当雷达 `livox_frame +X` 物理指向车体 `+Y` 时，外参 profile 中 `base_link -> livox_frame` 的 `yaw=+90deg` 仍然正确，`rc26_odom_interface` 会把输出 `/odom` 的首帧 `base_footprint` yaw 归到 0。
