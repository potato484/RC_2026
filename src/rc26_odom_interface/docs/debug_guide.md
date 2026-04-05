# rc26_odom_interface 调试指南

本文档提供针对 `rc26_odom_interface` 模块的具体调试步骤和命令行指令，用于验证其功能是否正常运行（主要验证 TF 发布和里程计数据的正确性）。

## 1. 编译模块

首先需要编译对应的模块，建议直接使用当前 AidLux 环境下实测更快的默认构建参数：

```bash
cd "${RC26_WS:-$HOME/RC_2026}"
MAKEFLAGS='-j4 -l4' colcon build --parallel-workers 2 --packages-select rc26_odom_interface rc26_bringup --cmake-args -DCMAKE_BUILD_TYPE=Release
source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"
```

## 2. 启动节点

目前该节点已集成在系统的里程计启动脚本中。请使用以下指令启动：

```bash
ros2 launch rc26_bringup odometry.launch.py
```

*注：确保上游的 MID-360 驱动和 Point-LIO 节点已正常运行并发布 `/state_estimation` 话题。*

## 3. 验证 TF 树 (odom -> base_link)

`rc26_odom_interface` 的核心职责是发布从 `odom` 到 `base_link` 的 TF 变换。

**查看 TF 变换是否持续发布：**
```bash
ros2 run tf2_ros tf2_echo odom base_link
```
*预期结果：终端应持续输出平滑的平移（Translation）和旋转（Rotation）数据，不应出现间歇性中断或跳变。*

**检查 TF 树中是否存在重复发布告警：**
```bash
ros2 run tf2_ros tf2_monitor odom base_link
```
*预期结果：在 `Broadcaster` 列表中，只应存在 `rc26_odom_interface` 作为 `odom` 到 `base_link` 的唯一发布者，不应有其他节点冲突发布。*

## 4. 验证话题输出

该模块将上游位姿转换为底盘坐标系下的里程计，并输出 `/odom` 话题。

**检查 `/odom` 话题数据：**
```bash
ros2 topic echo /odom --once
```
*预期结果：*
- `header.frame_id` 应为 `odom`
- `child_frame_id` 应为 `base_link`
- 当机器人移动时，`pose.pose` 和 `twist.twist` 应有平滑的非零数值输出。

**检查话题发布频率：**
```bash
ros2 topic hz /odom
```
*预期结果：频率应与上游 `/state_estimation` 话题（通常为 10Hz 或 20Hz 视雷达配置而定）保持一致。*

## 5. 常见问题排查

**问题：没有 `/odom` 话题输出或 TF 未发布**
1. 检查上游输入话题是否存在：
   ```bash
   ros2 topic hz /state_estimation
   ```
2. 检查静态 TF 是否提供（特别是 `base_link` 到 `livox_frame`）：
   ```bash
   ros2 run tf2_ros tf2_echo base_link livox_frame
   ```
3. 查看节点日志，检查是否有因时间戳乱序或 `dt` 过大导致跳过计算的警告信息：
   ```bash
   ros2 run rqt_console rqt_console
   ```
