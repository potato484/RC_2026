# rc26_localization 调试指南

## 1. 编译模块
在进行任何调试前，请确保模块已成功编译（由于 R2 算力平台限制，推荐限制编译核心数以防内存溢出）：
```bash
cd "${RC26_WS:-$HOME/RC_2026}"
colcon build --symlink-install --parallel-workers 3 --packages-select rc26_localization --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## 2. 启动定位节点
通过 Launch 文件启动定位节点及其依赖的参数文件：
```bash
# 刷新工作空间环境变量
source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"

# 启动定位节点（包含参数加载）
ros2 launch rc26_bringup localization.launch.py
```
*注：如果需要在控制台查看详细的 DEBUG 日志，可以在 launch 命令后增加 `--ros-args --log-level debug`。*

## 3. 实时参数动态调节 (Dynamic Reconfigure)
定位模块支持运行时动态调参，可通过命令行快速验证不同参数组合的效果，无需重新编译。

### 3.1 切换优化器模式
在 GN（Gauss-Newton）和 LM（Levenberg-Marquardt）之间切换：
```bash
# 设置为自动切换模式（推荐，初值好时用 GN 省算力，初值差时用 LM 保精度）
ros2 param set /rc26_localization gicp_optimizer_mode "gn_auto"

# 强制使用纯 GN 模式
ros2 param set /rc26_localization gicp_optimizer_mode "gn"

# 强制使用纯 LM 模式
ros2 param set /rc26_localization gicp_optimizer_mode "lm"
```

### 3.2 调节 Huber 鲁棒核
当场上出现大量动态障碍物（如敌方机器人）导致定位漂移时，可以调节 Huber 核参数：
```bash
# 开启/关闭 Huber 核（开启后能有效过滤动态噪点）
ros2 param set /rc26_localization robust_enable true

# 调节 Huber 核的阈值 c（单位：标准差倍数。越小对异常值越敏感）
ros2 param set /rc26_localization huber_c 1.0
```

### 3.3 调整协方差与退化检测
调整用于评估定位置信度（协方差）和检测走廊退化的相关参数：
```bash
# 启用基于 Hessian 矩阵的协方差计算
ros2 param set /rc26_localization cov_from_hessian_enable true

# 启用基于特征值分析的硬退化检测
ros2 param set /rc26_localization hessian_degen_enable true

# 设置硬退化阈值（当 Hessian 最小特征值小于此值时，拒绝更新当前帧，沿用 Odom/IMU 预测）
ros2 param set /rc26_localization hessian_lambda_hard 10.0
```

## 4. 话题监控与数据分析
通过订阅 ROS 2 话题实时监控定位模块的运行状态。

### 4.1 检查定位质量与退化诊断信息
```bash
# 持续打印诊断信息（包含协方差特征值、当前是否退化等）
ros2 topic echo /localization/diagnostics
```
**关键字段说明**：
- `h_min_eig` / `h_max_eig`: Hessian 矩阵的最小/最大特征值。在长走廊等退化场景下，`h_min_eig` 会显著变小。
- `sigma_xy` / `sigma_yaw`: 评估出的 X/Y 位置与航向角的不确定性（标准差）。
- `hard_degen_consec`: 连续触发硬退化（拒绝更新）的帧数。

### 4.2 查看位姿输出
```bash
# 查看包含协方差的位姿估计
ros2 topic echo /localization/pose_with_cov
```

### 4.3 查看 P0 健康度与后端状态
```bash
# LHI: 给控制器/决策层的语义化定位健康度
ros2 topic echo /localization/health

# 后端状态: P0 阶段为占位字段，P1 后切换为真实图后端状态
ros2 topic echo /localization/backend_status
```
**关键字段说明**：
- `health.level`: `GREEN/YELLOW/ORANGE/RED`，用于控制器二层速度钳位。
- `health.reason`: 当前触发主因，例如 `sigma_xy_warn`、`control_degraded`、`global_recovery_running`。
- `backend_status.optimizer_ready`: P0 阶段固定 `false`（占位），P1 启用图后端后为真实状态。
- `backend_status.last_local_reg_age_sec`: 距离最近一次局部配准输出的时间，超阈值会触发 `RED`。

### 4.4 P0/P1 开关参数
```bash
# 是否启用图后端（P0 默认 false，P1 开始逐步开启）
ros2 param get /localization enable_graph_backend

# 是否允许旧版硬切重定位（仅紧急回退使用，默认 false）
ros2 param get /localization legacy_hard_reloc_enable
```

## 5. 结合 Bag 包离线调试 (推荐)
为了可重复地复现问题，强烈推荐录制比赛/测试时的数据包（Bag），并在 PC 端离线回放调试：

**录制 Bag 包（在 R2 机器人上）**：
```bash
ros2 bag record -o loc_test_bag /livox/lidar /imu/data /odom /tf /tf_static
```

**回放与调试（在 PC 或 R2 上）**：
1. 终端 1：启动定位节点（如第 2 步）
2. 终端 2：回放数据包（建议降低播放速率，如 0.5 倍速，以便观察）
```bash
ros2 bag play loc_test_bag --rate 0.5
```
3. 终端 3：使用 RViz2 监控点云匹配情况，或使用 `ros2 topic echo` 监控输出状态。

## 6. 常见问题排查

- **定位节点已启动但 `/localization/pose_with_cov` 无输出**：先核对 `/livox/lidar`、`/imu/data`、`/odom` 是否都有数据，再检查 launch 中地图、参数文件和命名空间是否加载正确。
- **频繁进入硬退化或位姿卡住**：优先观察 `/localization/diagnostics` 中的 `h_min_eig` 与 `hard_degen_consec`，并结合场景特征决定是否降低 `hessian_lambda_hard`、启用 `robust_enable` 或切换 `gicp_optimizer_mode`。
- **协方差过大或位姿抖动明显**：检查初值来源、IMU/里程计时间同步，以及点云输入是否存在动态障碍大面积遮挡；离线回放时建议降低 bag 播放速率便于定位问题。
