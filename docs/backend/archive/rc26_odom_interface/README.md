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
  - 启动后在真实 Point-LIO 里程计完成静止位姿归零并接管前，可按 `publish_bootstrap_pose=true` 发布动态 bootstrap `/odom` 与 TF，保持 `odom -> base_footprint` 和 `base_footprint -> base_link` 可查询；该位姿只用于启动阶段稳定下游 TF/odom 消费，不表示定位成功，不能作为会发布 `/cmd_vel` 的导航/运动入口启动依据
  - `base_link` 相对 `base_footprint` 的高度由 `rc26_bringup/config/odom_interface.yaml` 提供，当前为 `0.065m`
  - 注入外参按 TF 语义表示 `base_link -> input_body_frame`；节点在恢复 `odom -> base_link` 位姿时会使用其 inverse，在转换 Point-LIO twist 时会把速度从 `input_body_frame` 旋到 `base_link` 并扣除传感器安装杆臂项。`zero_origin_to_first_frame=true` 时，启动静止窗口会同时归零首帧平移和 yaw；例如 Mid-360 的 `livox_frame +X` 物理指向车体 `+Y` 时，传感器外参 yaw 仍应保持 `+90deg`，但输出 `/odom` 的初始 `base_footprint` yaw 会被归到 `0`。现场若看到 RViz 中 `base_footprint` 箭头与实体车头持续不一致，应优先检查这条外参语义、传感器 profile 和启动静止归零状态，而不是改 `/cmd_vel` 符号。
  - `registered_scan` 与 odom 时间匹配缓存由 `cloud_queue_size`、`pending_cloud_queue_size` 和 `odom_stamp_history_size` 控制，用于吸收点云先到、回调调度抖动和短时队列拥塞；时间差门控仍由 `max_time_diff_sec` 负责，缓存扩大不代表接受严重错时数据
  - 低算力完整导航默认关闭 `odom_path` 与 `odom_pose_markers` 调试发布，避免无 RViz 运行时继续序列化路径和 marker；需要现场观察时可在 `odom_interface.yaml` 中显式打开

## 当前边界

- 不负责里程计估计本体
- 不再查询 `base_link -> point_lio.body_frame` TF；内部 body 外参只接受 launch 注入，不再作为对外 TF 边存在
- 不直接做控制求解
- 只负责自动导航链，不改遥控发布侧或外部底盘执行链
- 供决策侧 odom 闭环导航、定位联调和可视化统一消费
- 不发布静态 `base_footprint -> base_link`；该边仍由本节点作为唯一权威动态发布

## 本轮同步

2026-07-03 同步：启动静止归零从只平移归零扩展为平移 + yaw 同时归零。`rc26_sensor_extrinsics` 继续表达真实雷达安装方向，不为了修正导航初始朝向去改传感器 yaw；当雷达 `livox_frame +X` 物理指向车体 `+Y` 时，外参 profile 中 `base_link -> livox_frame` 的 `yaw=+90deg` 仍然正确，`rc26_odom_interface` 会把输出 `/odom` 的首帧 `base_footprint` yaw 归到 0，避免决策层台阶 yaw gate 把初始安装偏置误当成车体朝向误差。

2026-06-30 同步：完整导航 `bringup.launch.py run_mode:=navigation` 和独立 `grid_heading.launch.py` 启动 odometry 时显式关闭 `publish_bootstrap_pose`。bootstrap `/odom` 继续作为 odom_interface 的可选启动占位能力保留给非运动观察/联调场景，但实车会发布 `/cmd_vel` 的导航入口必须等待真实 Point-LIO odom 经本节点接管后再放行动作。
