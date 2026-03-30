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

## 源码入口与阅读顺序
- 先看 `launch/merge_odom_fused.launch.py` 和 `launch/merge_odom.launch.py`，理解这个子系统是如何把多个节点拼起来的。
- 再看 `src/merge_odom_node.cpp`，它是总装入口。
- 然后分模块看 `src/can/`、`src/wheel/`、`src/imu/`、`src/fuser/`、`src/pose/`。
- 最后看参数 YAML 和调试脚本，确认串口/CAN/速度保护的部署值。

## 目录解剖
- `src/merge_odom_node.cpp`：统一创建和装配 CAN、轮速、IMU、融合器、位姿下发器。
- `src/can/`：CAN 里程计采集与解码。
- `src/wheel/`：轮里程计串口接入和速度解算。
- `src/imu/`：达妙 IMU 驱动与解析。
- `src/fuser/`：CAN/轮里程计软融合。
- `src/pose/`：结合反馈、地形限速和保护器向下位机发速度。

## 关键文件体量
- `src/pose/pose_sender.cpp`：794 行，下发保护逻辑最重。
- `src/imu/dm_imu_driver.cpp`：454 行。
- `src/can/can_odom.cpp`：396 行。
- `src/fuser/wheel_odom_fuser.cpp`：353 行。
- `src/wheel/wheel_odom.cpp`：304 行。
- `src/merge_odom_node.cpp`：234 行，总装入口。

## 关键源码行段速览
- `src/rc26_merge_odom/src/merge_odom_node.cpp:1-221`：节点组合、参数分发和各子模块 wiring；`222-234`：`main()`。
- `src/rc26_merge_odom/src/can/can_odom.cpp:24-99`：构造和接口初始化；`100-225`：CAN 打开、接收线程和帧解析；`226-388`：里程计发布与状态访问。
- `src/rc26_merge_odom/src/wheel/wheel_odom.cpp:21-134`：串口轮速解包与机体系速度换算；`135-304`：里程计发布、状态获取与复位。
- `src/rc26_merge_odom/src/imu/dm_imu_driver.cpp:107-216`：串口打开与初始化；`247-374`：接收线程和缓冲解析；`375-454`：帧级解析。
- `src/rc26_merge_odom/src/fuser/wheel_odom_fuser.cpp:42-163`：输入缓存和源状态构造；`164-315`：定时融合主路径；`316-353`：健康度发布。
- `src/rc26_merge_odom/src/pose/pose_sender.cpp:56-319`：输入订阅和基础状态缓存；`320-586`：地形限速、IMU spike、fallback/governor 保护；`587-794`：反馈和目标发送定时器。

## 模块边界

- 这个包输出的是局部融合里程计和下发保护，不是全局定位
- 它不替代 `rc26_localization` 的地图配准职责
- 它也不做上层路径规划，只为控制和定位提供更稳的底层状态与执行接口
