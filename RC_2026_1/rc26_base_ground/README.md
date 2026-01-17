# rc26_base_ground

基于 TF 树的台阶参考系估计器，用于稳定量化车辆垂直位移（台阶层级）并判定上/下台阶动作是否完成。

## 原理

### 核心思想

`base_ground` 是一个"当前车辆所在平面的参考地表"坐标系，其特点：

- **X/Y/Yaw**：实时跟随 `base_link`
- **Z (ground_z)**：离散值（0, 0.2, 0.4...），仅在确认台阶跃迁后突变
- **Roll/Pitch**：锁定为 0，消除车辆俯仰/横滚干扰

### 算法流程

```
┌─────────────────┐
│  /odom 订阅     │
│ (odom->base_link)│
└────────┬────────┘
         ▼
┌─────────────────┐
│  稳定性检测     │  滑窗统计 Z 方差、dz/dt、droll/dt、dpitch/dt
│  (Stability)    │  全部低于阈值 → Stable
└────────┬────────┘
         ▼
┌─────────────────┐
│  h0 自标定      │  首个稳定段记录 h0 = z_base_link
│  (Calibration)  │  作为地面基准高度
└────────┬────────┘
         ▼
┌─────────────────┐
│  阶高量化       │  ground_z_candidate = z - h0
│  (Quantization) │  k = round(candidate / H)
└────────┬────────┘
         ▼
┌─────────────────┐
│  滞回确认       │  |candidate - k*H| < tol 持续 T_confirm
│  (Hysteresis)   │  → 确认层级切换
└────────┬────────┘
         ▼
┌─────────────────┐
│  发布 TF/话题   │  base_ground/level, stair_delta, stable
└─────────────────┘
```

### 状态机

```
CALIBRATING ──stable──▶ STABLE ──candidate──▶ TRANSITIONING ──confirm──▶ STABLE
     ▲                    │                        │
     │                    └──unstable──────────────┘
     └──unstable───────────────────────────────────┘
```

## 前提条件

1. **里程计输入**：需要 `/odom` 话题（`nav_msgs/Odometry`），提供 `odom -> base_link` 位姿
2. **TF 树**：
   - SLAM 模式：`odom -> base_link` 可用
   - 导航模式：`map -> odom -> base_link` 可用
3. **依赖包**：
   - `rclcpp`
   - `nav_msgs`
   - `geometry_msgs`
   - `std_msgs`
   - `tf2`、`tf2_ros`、`tf2_geometry_msgs`

## 接口

### 订阅话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/odom` | `nav_msgs/Odometry` | 里程计输入（可配置） |

### 发布话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `base_ground/level` | `std_msgs/Int32` | 当前绝对层级（0, 1, 2, ...） |
| `base_ground/stair_delta` | `std_msgs/Int8` | 层级变化脉冲（+1/-1），仅在变化时发布 |
| `base_ground/stable` | `std_msgs/Bool` | 当前稳定性状态 |

### TF 发布

| 变换 | 说明 |
|------|------|
| `parent_frame -> base_ground` | 父坐标系由参数决定（`odom` 或 `map`） |

## 参数配置

配置文件：`config/base_ground_estimator.yaml`

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `odom_topic` | string | `/odom` | 里程计话题名 |
| `parent_frame` | string | `odom` | 父坐标系（SLAM: `odom`, 导航: `map`） |
| `base_ground_frame` | string | `base_ground` | 子坐标系名称 |
| `step_height_m` | double | 0.20 | 台阶单级高度 (m) |
| `tol` | double | 0.06 | 高度误差容忍范围 (m) |
| `T_stable` | double | 0.5 | 稳定窗口时长 (s) |
| `T_confirm` | double | 0.3 | 状态确认消抖时长 (s) |
| `tf_timeout_sec` | double | 0.05 | TF 查询超时 (s) |
| `enable_tf_publish` | bool | true | 是否发布 TF |

## 使用方法

### 编译

```bash
cd ~/RC_2026
colcon build --packages-select rc26_base_ground
source install/setup.bash
```

### 启动

**SLAM 模式**（挂载于 `odom`）：
```bash
ros2 launch rc26_base_ground base_ground_estimator.launch.py parent_frame:=odom
```

**导航模式**（挂载于 `map`）：
```bash
ros2 launch rc26_base_ground base_ground_estimator.launch.py parent_frame:=map
```

**自定义配置**：
```bash
ros2 launch rc26_base_ground base_ground_estimator.launch.py \
    config_file:=/path/to/custom_config.yaml \
    parent_frame:=odom \
    use_sim_time:=false
```

### 验证

```bash
# 查看 TF
ros2 run tf2_ros tf2_echo odom base_ground

# 查看层级
ros2 topic echo /base_ground/level

# 查看稳定性
ros2 topic echo /base_ground/stable

# RViz 可视化
# 添加 TF 显示，观察 base_ground 坐标系
```

## 与决策层集成

`rc26_decision` 已集成本模块：

1. **黑板变量**：
   - `current_level` (int32)：当前层级
   - `stair_delta` (int8)：层级变化
   - `base_ground_stable` (bool)：稳定性状态
   - `level_start` (int32)：动作开始时的层级

2. **行为树节点**：
   - `StairClimbAction`：当 `current_level >= level_start + 3` 时返回 SUCCESS
   - `StairDescendAction`：当 `current_level <= level_start - 3` 时返回 SUCCESS

## 注意事项

1. **h0 标定**：节点启动后需等待首个稳定段完成标定，标定前不发布 TF
2. **抗干扰**：平地快速加减速或小幅震荡不会触发层级跳变
3. **时间戳**：自动处理时间戳回退，避免滑窗计算异常
4. **延迟**：稳定检测 + 确认消抖约 0.8s 延迟，属于正常行为

## 数据流

```
Point-LIO
    │
    ▼
rc26_odom_interface ──▶ /odom (odom->base_link)
                 │
                            ▼
                   rc26_base_ground
                            │
              ┌─────────────┼─────────────┐
              ▼             ▼             ▼
        TF 发布      /base_ground/*   rc26_decision
   (odom->base_ground)   话题          (黑板集成)
```
