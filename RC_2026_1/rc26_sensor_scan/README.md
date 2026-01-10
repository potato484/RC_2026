# rc26_sensor_scan

RC2026 传感器扫描同步模块，同步里程计与点云数据并发布统一时间戳的传感器数据。

## 功能

- 里程计与点云时间同步
- 底盘速度计算
- TF 变换发布
- 统一坐标系输出

## 工作原理

```
┌─────────────────────────────────────────────────────────────────┐
│                      rc26_sensor_scan                            │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│   ┌───────────────┐     ┌───────────────┐                       │
│   │   Odometry    │     │  PointCloud2  │                       │
│   │   (输入)      │     │   (输入)      │                       │
│   └───────┬───────┘     └───────┬───────┘                       │
│           │                     │                                │
│           └──────────┬──────────┘                                │
│                      ▼                                           │
│           ┌─────────────────────┐                                │
│           │  ApproximateTime    │                                │
│           │  Synchronizer       │                                │
│           │  (时间同步)         │                                │
│           └──────────┬──────────┘                                │
│                      │                                           │
│           ┌──────────┴──────────┐                                │
│           ▼                     ▼                                │
│   ┌───────────────┐     ┌───────────────┐                       │
│   │  Odometry     │     │  PointCloud2  │                       │
│   │  (输出)       │     │  (输出)       │                       │
│   │  + 速度估计   │     │  + 坐标变换   │                       │
│   └───────────────┘     └───────────────┘                       │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

## 目录结构

```
rc26_sensor_scan/
├── include/rc26_sensor_scan/
│   └── sensor_scan.hpp        # 节点头文件
├── src/
│   └── sensor_scan.cpp        # 节点实现
├── launch/
│   └── sensor_scan_generation.launch.py
├── CMakeLists.txt
└── package.xml
```

## 话题

### 订阅

| 话题 | 类型 | 说明 |
|------|------|------|
| `lidar_odometry_topic` | nav_msgs/Odometry | 激光里程计 |
| `registered_scan_topic` | sensor_msgs/PointCloud2 | 配准点云 |

### 发布

| 话题 | 类型 | 说明 |
|------|------|------|
| `scan_topic` | sensor_msgs/PointCloud2 | 同步后点云 |
| `odometry_topic` | nav_msgs/Odometry | 同步后里程计 (含速度) |

## 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `lidar_frame` | - | 激光雷达坐标系 |
| `base_frame` | - | 底盘坐标系 |
| `robot_base_frame` | - | 机器人基坐标系 |
| `lidar_odometry_topic` | - | 输入里程计话题 |
| `registered_scan_topic` | - | 输入点云话题 |
| `scan_topic` | - | 输出点云话题 |
| `odometry_topic` | - | 输出里程计话题 |
| `max_time_diff_sec` | 0.1 | 最大时间差 (s) |
| `tf_timeout_sec` | 0.5 | TF 查询超时 (s) |

## 时间同步

使用 `message_filters` 的 `ApproximateTime` 策略进行近似时间同步：

```cpp
using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    nav_msgs::msg::Odometry, sensor_msgs::msg::PointCloud2>;

sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(10), odometry_sub_, laser_cloud_sub_);

sync_->registerCallback(&SensorScanNode::laserCloudAndOdometryHandler, this);
```

## 速度估计

通过相邻帧位姿差分计算底盘速度：

```cpp
struct OdometryState {
    tf2::Transform previous_transform;
    rclcpp::Time previous_stamp;
    bool initialized = false;
};

// 线速度
linear.x = (current_x - previous_x) / dt;
linear.y = (current_y - previous_y) / dt;
linear.z = (current_z - previous_z) / dt;

// 角速度
angular.z = (current_yaw - previous_yaw) / dt;
```

## TF 发布

节点发布以下 变换：
- `odom -> base_link` (里程计位姿)

## 使用方式

### 通过 bringup 启动 (推荐)

```bash
ros2 launch rc26_bringup odometry.launch.py
```

### 单独启动

```bash
ros2 launch rc26_sensor_scan sensor_scan_generation.launch.py
```

### 测试

```bash
ros2 launch rc26_bringup test_sensor_scan.launch.py
```

## 配置示例

```yaml
sensor_scan:
  ros__parameters:
    lidar_frame: "laser_link"
    base_frame: "base_link"
    robot_base_frame: "base_link"
    lidar_odometry_topic: "/odom"
    registered_scan_topic: "/registered_scan"
    scan_topic: "/scan"
    odometry_topic: "/odometry"
    max_time_diff_sec: 0.1
    tf_timeout_sec: 0.5
```

## 数据流

```
Point-LIO
    │
    ├── /point_lio/odometry ──▶ rc26_odom_interface ──▶ /odom
    │                                                      │
    └── /point_lio/cloud_registered ──▶ rc26_odom_interface ──▶ /registered_scan
                                                               │
                                                               ▼
                                                    rc26_sensor_scan
                                                               │
                                                    ┌──────────┴──────────┐
                                                    ▼                     ▼
                                               /odometry              /scan
                                            (含速度估计)          (同步点云)
                                                    │
                                                    ▼
                                               Nav2 / 定位
```

## 坐标系约定

```
map (全局地图坐标系)
 │
 └── odom (里程计坐标系)
      │
      └── base_link (底盘中心) ← 本节点发布 TF
           │
           └── laser_link (激光雷达)
```

## 依赖

- rclcpp
- nav_msgs
- sensor_msgs
- message_filters
- tf2_ros

## 编译

```bash
colcon build --packages-select rc26_sensor_scan
```
