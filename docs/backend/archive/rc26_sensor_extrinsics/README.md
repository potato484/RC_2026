# rc26_sensor_extrinsics

## 模块定位

`rc26_sensor_extrinsics` 是 R2 当前的静态传感器安装外参配置真源，负责用 YAML 描述车身坐标系到雷达坐标系的安装位置与朝向。

## 当前实现

- 当前是 config-only 包，不导出运行节点
- 配置入口：`src/rc26_sensor_extrinsics/config/r2_sensor_extrinsics.yaml`
- 默认 profile：`r2_mid360_left_90`
  - `base_link -> livox_frame`
  - 平移：`[-0.3, 0.0, 0.15] m`
  - 姿态：`[0.0, 0.0, 1.57079632679] rad`
- `rc26_bringup/launch/odometry.launch.py` 在启动时读取该 YAML，并继续通过 `tf2_ros/static_transform_publisher` 发布静态 TF
- `base_link -> point_lio.body_frame` 不在 YAML 中手写，而是由 `base_link -> livox_frame` 和 `rc26_point_lio/config/mid360.yaml` 中的 Point-LIO 内部 LiDAR/IMU 外参推导；该结果只注入 `rc26_odom_interface`，不再对外发布 TF

## 当前边界

- 本包只描述机器人级安装外参，不负责发布 TF
- `rc26_bringup` 负责装配和发布静态 TF，但不再硬编码传感器安装姿态
- `rc26_point_lio/config/mid360.yaml` 继续只维护 Point-LIO 内部 LiDAR/IMU 外参，不能把整机安装 yaw 写进那里
- `point_lio.body_frame` 只表示 Point-LIO 内部输出 frame 名；它应与 `rc26_odom_interface` 的 `input_body_frame` 口径一致，但不要求对外暴露为 TF 边

## 维护口径

- 现场更换雷达安装方向时，优先新增或切换 `sensor_extrinsics.profiles.*`
- 使用 `sensor_extrinsics_profile:=<profile>` 选择 profile
- 若只调整车身到雷达的安装 yaw、平移或高度，不应修改 `rc26_point_lio/config/mid360.yaml`
- 若只调整 Point-LIO 内部 body frame 名或其装配契约，应同步检查 `odometry.launch.py` 注入值与 `rc26_odom_interface` 参数口径
