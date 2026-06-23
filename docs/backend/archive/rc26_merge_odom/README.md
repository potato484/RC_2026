# rc26_merge_odom

## 模块状态

`rc26_merge_odom` 源码当前保留在仓库中，但已经退出 R2 默认运行装配。

当前默认口径是：

- `rc26_bringup` 不再依赖或启动 `rc26_merge_odom`
- `test_navigation.launch.py` 不再启动 `merge_odom.launch.py`
- `start_r2_teleop.sh` 不再启动 `merge_odom_node` 或 `pose_sender_node`
- `/merge_odom` 不再是当前默认运行时契约
- `/cmd_vel` 的默认硬件消费方由 `rc26_mcu_transport` 提供
- `/mechanism/send_command` 与 `/mechanism/command_feedback` 的默认 provider 由 `rc26_mcu_transport` 提供

也就是说，本包现在不是默认整车链路中的底盘执行、局部反馈或目标 MCU 串口权威。

## 保留内容

本包仍保留历史底盘局部链路源码与配置，供后续手工调试、对照或重新设计时参考：

- `can_odom` + `can_odom_node`
- `wheel_odom` + `wheel_odom_node`
- `dm_imu_driver` + `dm_imu_node`
- `pose_sender` + `pose_sender_node`
- `merge_odom_node`
- `config/merge_odom_params.yaml`
- `config/ekf_params.yaml`
- `config/pose_sender_cmd_vel_teleop.yaml`
- `launch/merge_odom.launch.py`
- `launch/can_odom_only.launch.py`
- `launch/wheel_odom_only.launch.py`
- `launch/dm_imu_only.launch.py`

## 当前边界

- 这个包不再被默认 bringup、导航联调或遥控脚本装配。
- 这个包不再定义当前 `/cmd_vel` 到 MCU 的默认执行路径。
- 这个包不再提供当前 mechanism transport provider 的默认实现。
- 这个包不替代 `rc26_odom_interface` 的 `/odom` 与动态 TF 权威。
- 如果未来要重新启用本包或迁移其中能力，需要先重新定义运行时权威、接口归属、launch 参数和文档契约。

## 手工调试注意

源码和 launch 文件仍可被维护者手动调用，但这属于显式调试行为，不代表当前整车默认启动口径。

手工启用时需要额外确认：

- 同一时刻只有一个节点拥有目标 MCU 串口
- 同一时刻只有一个节点消费或下发底盘运动命令
- `/mechanism/send_command` 与 `/mechanism/command_feedback` 不与 `rc26_mcu_transport` 重复提供
- 不发布与 `rc26_odom_interface` 冲突的动态 TF

## 本轮同步

2026-06-22 同步：`rc26_merge_odom` 从默认运行链停用但保留源码。`rc26_bringup`、`test_navigation.launch.py` 和 `start_r2_teleop.sh` 已不再启动本包；机构 transport 改由 `rc26_mcu_transport` 承担。

2026-06-23 同步：默认底盘 `/cmd_vel` consumer 改由 `rc26_mcu_transport` 提供，并通过目标 MCU 串口下发 `POSE_TARGET(0x1F)`；本包的 `pose_sender_node` 仍仅作为历史调试入口保留，不回到默认链路。
