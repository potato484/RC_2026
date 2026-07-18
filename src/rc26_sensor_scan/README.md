# rc26_sensor_scan

`rc26_sensor_scan` 是自动导航链里的点云与底盘里程计时空对齐模块。它把 `odom` 坐标系下的 `registered_scan` 按同步后的底盘位姿重新投影回 `livox_frame` 视角，保留给导航链调试或后续恢复 Nav2 obstacle layer 使用。

## 当前职责

- 同步 `/odom` 与 `/registered_scan`
- 要求输入 `/odom.child_frame_id = base_footprint`
- 发布：
  - `/sensor_scan`，`frame_id=livox_frame`
  - `/odometry`，`child_frame_id=base_footprint`

## 当前实现口径

- 自动导航链固定使用 `base_footprint` 作为 2D 导航基座
- `base_link` 保留 roll/pitch，因此 `base_footprint -> livox_frame` 对导航链来说不再是纯静态量
- 本模块现在按每帧时间戳实时查询 `base_footprint -> livox_frame` 组合 TF，而不是缓存“底盘到雷达”的静态变换
- 这样可以在不改变 `rc26_sensor_extrinsics` 数值真源的前提下，继续把点云准确投影回传感器局部视角
- 当前默认 Nav2 配置已开启 local/global obstacle layer，`/sensor_scan` 会进入 costmap；障碍高度过滤由 `rc26_bringup/config/nav2_params.yaml` 维护，当前过滤 `0.05m` 以下低矮点后参与 inflation layer 合成
- `input_qos_depth` 与 `sync_queue_size` 默认均为 20，用于吸收 `/odom` 与 `/registered_scan` 的短时回调拥塞；同步仍使用 ExactTime，不接受时间戳不一致的数据

## 当前边界

- 不负责点云配准，不替代 `rc26_point_lio`
- 不负责 `/scan` LaserScan 兼容话题
- 不负责决定 Nav2 是否启用 obstacle layer
- 不负责遥控链或目标 MCU 底盘执行的 TF 语义
