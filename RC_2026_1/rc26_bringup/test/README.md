# RC26 模块测试指南

本目录包含各模块的独立测试 launch 文件，用于单独验证每个模块的功能。

## 前置条件

```bash
# 确保已编译并 source 环境
cd ~/RC_2026
colcon build --symlink-install
source install/setup.bash
```

---

## 测试指令汇总

### 1. 里程计接口测试 (rc26_odom_interface)

**功能**: 验证 Point-LIO → Nav2 坐标转换是否正常

```bash
# 启动测试 (需要 point_lio 数据源)
ros2 launch rc26_bringup test_odom_interface.launch.py

# 验证输出话题
ros2 topic echo /odom --once
ros2 topic echo /registered_scan --once

# 检查 TF 树
ros2 run tf2_ros tf2_echo odom base_link
```

---

### 2. 传感器扫描测试 (rc26_sensor_scan)

**功能**: 验证点云坐标转换和里程计速度发布

```bash
# 启动测试 (需要 odom_interface 数据源)
ros2 launch rc26_bringup test_sensor_scan.launch.py

# 验证输出话题
ros2 topic echo /sensor_scan --once
ros2 topic echo /odometry --once

# 检查 TF 树
ros2 run tf2_ros tf2_echo base_link laser_link
```

---

### 3. 定位模块测试 (rc26_localization)

**功能**: 验证基于 small_gicp 的点云配准定位

```bash
# 启动测试 (指定先验点云)
ros2 launch rc26_bringup test_localization.launch.py \
    prior_pcd_file:=/path/to/prior.pcd

# 验证 TF 发布 (map -> odom)
ros2 run tf2_ros tf2_echo map odom

# 检查定位状态
ros2 topic echo /localization/status --once
```

---

### 4. 完整里程计链测试 (point_lio + odom_interface + sensor_scan)

**功能**: 验证完整的里程计数据流

```bash
# 启动完整里程计链
ros2 launch rc26_bringup test_odometry_chain.launch.py

# 验证数据流
ros2 topic list | grep -E "(odom|scan|odometry)"
ros2 topic hz /odometry

# 检查完整 TF 树
ros2 run tf2_tools view_frames
```

---

### 5. 决策系统测试 (rc26_decision)

**功能**: 验证行为树决策逻辑

```bash
# 启动测试 (独立模式，无 Nav2)
ros2 launch rc26_bringup test_decision.launch.py

# 验证节点状态
ros2 node info /waypoint_patrol

# 检查黑板状态
ros2 topic echo /decision/blackboard_state --once
```

---

### 5.1 串口通信测试 (MCU 通信)

**功能**: 验证与 MCU 的串口通信 (位姿发送、指令交互)

```bash
# 连接真实 MCU
ros2 launch rc26_bringup test_serial_comm.launch.py \
    serial_port:=/dev/ttyUSB0

# 使用虚拟串口对 (调试用)
# 终端1: 创建虚拟串口对
socat -d -d pty,raw,echo=0 pty,raw,echo=0
# 输出类似: N PTY is /dev/pts/3, N PTY is /dev/pts/4

# 终端2: 启动测试
ros2 launch rc26_bringup test_serial_comm.launch.py \
    serial_port:=/dev/pts/3

# 终端3: 模拟 MCU 回复 (监听另一端)
cat /dev/pts/4

# 验证位姿发送
ros2 topic pub /odometry nav_msgs/msg/Odometry \
    "{pose: {pose: {position: {x: 1.0, y: 2.0}}}}" --once
```

**协议说明** (参见 `serial/protocol.hpp`):
- `0x01` POSE_DATA - 位姿数据
- `0x02` NAVIGATE_CMD - 导航指令
- `0x03` ACTION_CMD - 动作指令
- `0x05` STOP_CMD - 急停命令

---

### 6. 感知模块测试 (rc26_perception)

**功能**: 验证 D455 相机 + YOLO 检测

```bash
# 启动测试 (pass-through 模式，无需模型)
ros2 launch rc26_bringup test_perception.launch.py

# 启动测试 (带模型)
ros2 launch rc26_bringup test_perception.launch.py \
    model_path:=/path/to/yolo.tflite

# 验证输出话题
ros2 topic echo /rc26/block_detections --once
```

---

### 7. 控制器插件测试 (rc26_omni_controller)

**功能**: 验证全向运动控制器 (需要 Nav2 环境)

```bash
# 启动最小化 Nav2 + 控制器测试
ros2 launch rc26_bringup test_omni_controller.launch.py

# 手动发送导航目标
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0}}}}"

# 检查速度输出
ros2 topic echo /cmd_vel --once
```

---

## 验证清单

| 模块 | 话题/TF | 预期结果 |
|------|---------|----------|
| odom_interface | `/odom` | odom→base_link 变换 |
| sensor_scan | `/sensor_scan` | laser_link 坐标系点云 |
| localization | TF `map→odom` | 有效变换 |
| decision | `/decision/*` | 行为树运行 |
| serial_comm | 串口设备 | MCU 收到位姿帧 |
| perception | `/rc26/block_detections` | 检测消息 |
| omni_controller | `/cmd_vel` | 速度指令 |

---

## 调试技巧

```bash
# 查看所有节点
ros2 node list

# 查看所有话题
ros2 topic list

# 检查节点日志
ros2 run rqt_console rqt_console

# TF 树可视化
ros2 run tf2_tools view_frames && evince frames.pdf
```
