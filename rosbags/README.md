# 项目内 ROS bag 目录

本目录用于存放当前仓库相关的调试数据包，方便围绕 R2 自动机器人问题做复现、回放和离线分析。

## 使用约定

- 建议在仓库根目录执行录包命令。
- bag 数据默认录到本目录下。
- 真实 bag 文件通常较大，因此 `.gitignore` 已默认忽略本目录中的录包产物，仅保留本说明文件。

## 推荐命令

```bash
cd ~/RC_2026
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 bag record -o rosbags/teleop_left_stick_$(date +%F_%H-%M-%S) \
  /joy \
  /cmd_vel \
  /terrain_speed_limit \
  /pose_sender/target_protected \
  /pose_sender/feedback_protected \
  /pose_sender/imu_spike_active \
  /merge_odom \
  /tf \
  /tf_static
```

## 录包动作模板

建议每次录包按下面顺序操作，便于后续对齐分析：

1. 摇杆回中静止 `3` 秒。
2. 左摇杆前后推满 `3` 秒。
3. 左摇杆左右推满 `3` 秒。
4. 右摇杆旋转推满 `3` 秒。
5. 全部回中静止 `3` 秒。
6. `Ctrl+C` 停止录包。

## 快速检查

```bash
ros2 bag info rosbags/teleop_left_stick_时间戳
```

如需针对“左摇杆无法控制平移、右摇杆可以旋转”的问题做专项排查，请参考：

- `src/rc26_telecontrol/docs/left_stick_no_translation_debug.md`
