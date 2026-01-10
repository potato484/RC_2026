# rc26_omni_controller

RC2026 全向轮 PID + Pure Pursuit 路径跟踪控制器，Nav2 兼容插件。

## 功能

- 全向轮运动学控制
- PID 平移控制
- PID 航向控制 (可选)
- 自适应前视距离
- 曲率限速
- 接近目标减速
- 动态参数调整

## 算法原理

```
┌─────────────────────────────────────────────────────────────────┐
│                    Pure Pursuit + PID 控制                       │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│   全局路径 ──▶ 局部路径变换 ──▶ 前视点计算 ──▶ 目标方向          │
│                                      │                           │
│                                      ▼                           │
│                              ┌───────────────┐                   │
│                              │  PID 控制器   │                   │
│                              ├───────────────┤                   │
│                              │ 平移 PID      │──▶ vx, vy         │
│                              │ 航向 PID      │──▶ omega          │
│                              └───────────────┘                   │
│                                      │                           │
│                                      ▼                           │
│                              ┌───────────────┐                   │
│                              │  速度限制     │                   │
│                              ├───────────────┤                   │
│                              │ 曲率限速      │                   │
│                              │ 接近减速      │                   │
│                              │ 最大速度限制  │                   │
│                              └───────────────┘                   │
│                                      │                           │
│                                      ▼                           │
│                                 cmd_vel                          │
└─────────────────────────────────────────────────────────────────┘
```

## 目录结构

```
rc26_omni_controller/
├── include/rc26_omni_controller/
│   ├── omni_pid_pursuit_controller.hpp  # 控制器头文件
│   └── pid.hpp                          # PID 控制器头文件
├── src/
│   ├── omni_pid_pursuit_controller.cpp  # 控制器实现
│   └── pid.cpp                          # PID 控制器实现
├── rc26_omni_controller.xml             # Nav2 ── CMakeLists.txt
└── package.xml
```

## 参数

### PID 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `translation_kp` | 3.0 | 平移比例增益 |
| `translation_ki` | 0.1 | 平移积分增益 |
| `translation_kd` | 0.3 | 平移微分增益 |
| `enable_rotation` | true | 启用航向控制 |
| `rotation_kp` | 3.0 | 航向比例增益 |
| `rotation_ki` | 0.1 | 航向积分增益 |
| `rotation_kd` | 0.3 | 航向微分增益 |
| `min_max_sum_error` | 1.0 | 积分限幅 |

### 前视距离参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `lookahead_dist` | 0.8 | 基础前视距离 (m) |
| `use_velocity_scaled_lookahead_dist` | true | 速度自适应前视距离 |
| `lookahead_time` | 1.0 | 前视时间 (s) |
| `min_lookahead_dist` | 0.5 | 最小前视距离 (m) |
| `max_lookahead_dist` | 1.5 | 最大前视距离 (m) |

### 速度限制参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `v_linear_min` | -2.5 | 最小线速度 (m/s) |
| `v_linear_max` | 2.5 | 最大线速度 (m/s) |
| `v_angular_min` | -3.0 | 最小角速度 (rad/s) |
| `v_angular_max` | 3.0 | 最大角速度 (rad/s) |

### 接近减速参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `min_approach_linear_velocity` | 0.5 | 接近最小速度 (m/s) |
| `approach_velocity_scaling_dist` | 1.0 | 减速开始距离 (m) |

### 曲率限速参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `curvature_min` | 0.4 | 曲率下限 |
| `curvature_max` | 0.7 | 曲率上限 |
| `reduction_ratio_at_high_curvature` | 0.5 | 高曲率速度衰减比 |
| `curvature_forward_dist` | 0.7 | 曲率前向采样距离 (m) |
| `curvature_backward_dist` | 0.3 | 曲率后向采样距离 (m) |

### 其他参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `transform_tolerance` | 0.1 | TF 变换容差 (s) |
| `use_interpolation` | false | 路径插值 |
| `use_rotate_to_heading` | true | 原地旋转对准 |
| `use_rotate_to_heading_threshold` | 0.1 | 旋转对准阈值 (rad) |
| `max_velocity_scaling_factor_rate` | 0.9 | 速度缩放因子变化率 |

## Nav2 配置

在 `nav2_params.yaml` 中配置：

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    controller_plugins: ["FollowPath"]

    FollowPath:
      plugin: "rc26_omni_controller::OmniPidPursuitController"
      translation_kp: 3.0
      translation_ki: 0.1
      translation_kd: 0.3
      enable_rotation: true
      rotation_kp: 3.0
      rotation_ki: 0.1
      rotation_kd: 0.3
      lookahead_dist: 0.8
      use_velocity_scaled_lookahead_dist: true
      min_lookahead_dist: 0.5
      max_lookahead_dist: 1.5
      v_linear_max: 2.5
      v_angular_max: 3.0
```

## 发布话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `local_plan` | nav_msgs/Path | 局部路径 |
| `lookahead_point` | geometry_msgs/PointStamped | 前视点 |
| `curvature_points` | visualization_msgs/MarkerArray | 曲率采样点 (调试) |

## 使用方式

### 作为 Nav2 插件

控制器作为 Nav2 controller_server 的插件自动加载，无需单独启动。

### 测试控制器

```bash
ros2 launch rc26_bringup test_omni_controller.launch.py
```

## PID 控制器

内置 PID 控制器类，支持：
- 比例、积分、微分控制
- 积分限幅
- 输出限幅
- 参数动态调整

```cpp
#include "rc26_omni_controller/pid.hpp"

rc26_omni_controller::PID pid(kp, ki, kd, min_output, max_output, min_integral, max_integral);
double output = pid.compute(error, dt);
pid.reset();
```

## 依赖

- rclcpp
- nav2_core
- nav2_costmap_2d
- nav_msgs
- geometry_msgs
- visualization_msgs
- tf2_ros

## 编译

```bash
colcon build --packages-select rc26_omni_controller
```
