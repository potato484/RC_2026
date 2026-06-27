# rc26_odom_interface

## 模块定位

`rc26_odom_interface` 是 Point-LIO 与底盘统一坐标系之间的接口层，负责把上游里程计结果转换成下游统一消费的 odom 与 TF。

## 当前实现

- 导出节点: `rc26_odom_interface_node`
- 关键职责:
  - 消费 Point-LIO 内部 body frame 语义的 `/state_estimation`
  - 使用 `odometry.launch.py` 注入的 `base_link -> input_body_frame` 外参做底盘系映射
  - 权威输出自动导航链动态 TF：`odom -> base_footprint` 与 `base_footprint -> base_link`
  - 发布标准化 `/odom`，其中 `child_frame_id=base_footprint`
  - `base_link` 相对 `base_footprint` 的高度由 `rc26_bringup/config/odom_interface.yaml` 提供，当前为 `0.2m`
  - 注入外参按 TF 语义表示 `base_link -> input_body_frame`；节点在恢复 `odom -> base_link` 位姿时会使用其 inverse，在转换 Point-LIO twist 时会把速度从 `input_body_frame` 旋到 `base_link` 并扣除传感器安装杆臂项。现场若看到 RViz 中 `base_footprint` 箭头与实体车头相反，应优先检查这条外参语义和传感器 profile，而不是改 `/cmd_vel` 符号。

## 当前边界

- 不负责里程计估计本体
- 不再查询 `base_link -> point_lio.body_frame` TF；内部 body 外参只接受 launch 注入，不再作为对外 TF 边存在
- 不直接做控制求解
- 只负责自动导航链，不改遥控发布侧或外部底盘执行链
- 供定位、Nav2 和可视化统一消费
