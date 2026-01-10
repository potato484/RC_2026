# rc26_odom_interface

RC2026 里程计接口模块，将 Point-LIO 输出转换为导航栈统一坐标系。

## 功能

- 坐标系转换 (lidar_odom -> odom)
- 点云坐标变换
- 里程计速度估计
- TF 变换管理

## 工作原理

```
┌─────────────────────────────────────────────────────────────────┐
│                      rc26_odom_interface                         │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│   Point-LIO 输出                        导航栈输入               │
│   ┌───────────────┐                    ┌───────────────┐        │
│   │ lidar_odom    │                    │ odom          │        │
│   │ 坐标系        │ ──── 变换 ────▶   │ 坐标系        │        │
│   └───────────────┘                    └───────────────┘        │
│                                                                  │
│   输入:                                输出:                     │
│   - state_estimation (Odometry)        - /odom (Odometry)       │
│   - registered_scan (PointCloud2)      - /registered_scan       │
│                                          (PointCloud2)          │
└─────────────────────────────────────────────────────────────────┘
```

## 目录结构

```
rc26_odom_interface/
├── include/rc26_odom_interface/
│   └── odom_interface.hpp     # 节点头文件
├── src/
│   └── odom_interface.cpp     # 节点实现
├── launch/
│   └── loam_interface_launch.py
├── CMakeLists.txt
└── package.xml
```

## 话题

### 订阅

| 话题 | 类型 | 说明 |
|------|------|------|
| `state_estimation_topic` | nav_msgs/Odometry | Point-LIO 里程计输出 |
| `registered_scan_topic` | sensor_msgs/PointCloud2 | Point-LIO 配准点云 |

### 发布

| 话题 | 类型 | 说明 |
|------|------|------|
| `/odom` | nav_msgs/Odometry | 转换后里程计 (odom -> base_link) |
| `/registered_scan` | sensor_msgs/PointCloud2 | 转换后点云 (odom 坐标系) |

## 坐标系转换

```
Point-LIO 输出:
  lidar_odom -> laser_link

转换后:
  odom -> base_link -> laser_link

变换关系:
  T_odom_base = T_lidar_odom_laser * T_laser_base^(-1)
```

## 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `state_estimation_topic` | - | 输入里程计话题 |
| `registered_scan_topic` | - | 输入点云话题 |
| `odom_frame` | `odom` | 输出里程计坐标系 |
| `base_frame` | `base_link` | 底盘坐标系 |
| `lidar_frame` | `laser_link` | 激光雷达坐标系 |
| `tf_timeout_sec` | 0.5 | TF 查询超时 (s) |
| `max_time_diffc` | 0.2 | 点云与里程计最大时间差 (s) |
| `tf_refresh_interval_sec` | 1.0 | TF 断连重新拉取周期 (s) |

## 速度估计

节点通过相邻帧位姿差分估计线速度和角速度：

```cpp
struct OdomState {
    tf2::Transform previous_transform;
    rclcpp::Time previous_stamp;
    bool initialized;
};

// 速度计算
v = (current_position - previous_position) / dt
omega = (current_orientation - previous_orientation) / dt
```

## 时间同步

- 使用 `max_time_diff_sec` 限制点云与里程计的时间差
- 防止严重时间对不齐导致的坐标变换错误

## 使用方式

### 通过 bringup 启动 (推荐)

```bash
ros2 launch rc26_bringup odometry.launch.py
```

### 单独启动

```bash
ros2 launch rc26_odom_interface loam_interface_launch.py
```

### 测试

```bash
ros2 launch rc26_bringup test_odom_interface.launch.py
```

## 配置示例

```yaml
odom_interface:
  ros__parameters:
    state_estimation_topic: "/point_lio/odometry"
    registered_scan_topic: "/point_lio/cloud_registered"
    odom_frame: "odom"
    base_frame: "base_link"
    lidar_frame: "laser_link"
    tf_timeout_sec: 0.5
    max_time_diff_sec: 0.2
```

## 坐标系约定

```
map (全局地图坐标系)
 │
 └── odom (里程计坐标系) ← 本节点输出
      │
      └── base_link (底盘中心)
           │
           └── laser_link (激光雷达)
```

## 依赖

- rclcpp
- nav_msgs
- sensor_msgs
- tf2_ros
- tf2_geometry_msgs

## 编译

```bash
colcon build --packages-select rc26_odom_interface
```
