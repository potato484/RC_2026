# rc26_merge_odom

## 模块定位

`rc26_merge_odom` 是 R2 当前的多源里程计融合与位姿下发包，负责把轮速、CAN 里程计、达妙 IMU 和控制下发整合到同一条运行链路里。

## 当前实现

这个包不是单节点实现，而是一个多库、多可执行拼装的子系统：

- `can_odom` + `can_odom_node`
- `wheel_odom` + `wheel_odom_node`
- `dm_imu_driver` + `dm_imu_node`
- `wheel_odom_fuser` + `wheel_odom_fuser_node`
- `pose_sender` + `pose_sender_node`
- `merge_odom_node`
- 调试节点：`single_wheel_test_node`

源码目录已经按功能拆开：

- `src/can/`：CAN 里程计解析
- `src/wheel/`：轮式里程计
- `src/imu/`：达妙 IMU 驱动与预处理
- `src/fuser/`：多源里程计软融合
- `src/pose/`：位姿/速度下发
- `src/merge_odom_node.cpp`：总装与统一启动入口

关键配置与启动文件：

- `config/merge_odom_params.yaml`
- `config/ekf_params.yaml`
- `config/pose_sender_cmd_vel_teleop.yaml`
- `launch/merge_odom.launch.py`
- `launch/merge_odom_fused.launch.py`
- `launch/can_odom_only.launch.py`
- `launch/wheel_odom_only.launch.py`
- `launch/dm_imu_only.launch.py`

## 模块边界

- 这个包输出的是局部融合里程计和下发保护，不是全局定位
- 它不替代 `rc26_localization` 的地图配准职责
- 它也不做上层路径规划，只为控制和定位提供更稳的底层状态与执行接口
