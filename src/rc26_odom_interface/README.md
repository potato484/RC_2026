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

## 当前坐标语义

- `base_footprint`：导航使用的 2D 地面投影基座
- `base_link`：底盘最下层金属刚性主板中心
- `base_link` 相对 `base_footprint` 的高度由 `rc26_bringup/config/odom_interface.yaml` 提供，当前为 `0.2m`
- `base_link` 保留 Point-LIO 解算出来的 roll/pitch；`odom -> base_footprint` 只保留 `x/y/yaw`

## 当前边界

- 只负责自动导航链，不改遥控 / `rc26_merge_odom` / minimal-mcu 链路
- 不负责里程计估计本体
- 不再查询或发布 `base_link -> point_lio.body_frame` 对外 TF；这段内部外参只在 bringup 装配期推导并注入
- 不直接做控制求解
