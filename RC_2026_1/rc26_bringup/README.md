# rc26_bringup

RC2026 R2 自动机器人系统启动与配置管理模块。

## 功能

统一管理整个导航系统的启动配置，包括：
- 里程计链路 (Point-LIO + rc26_odom_interface + rc26_sensor_scan)
- 定位模块 (rc26_localization)
- Nav2 导航栈 (含自定义控制器)
- 决策系统 (rc26_decision)
- 感知模块 (rc26_perception) [可选]

## 目录结构

```
rc26_bringup/
├── launch/
│   ├── bringup.launch.py              # 主启动文件
│   ├── odometry.launch.py             # 里程计链路启动
│   ├── localization.launch.py         # 定位模块启动
│   ├── test_odometry_chain.launch.py  # 里程计链路测试
│   ├── test_localization.launch.py    # 定位模块测试
│   ├── test_decision.launch.py        # 决策系统测试
│   ├── test_perception.launch.py      # 感知模块测试
│   ├── test_omni_controller.launch.py # 控制器测试
│   ├── test_odom_interface.launch.py  # 里程计接口测试
│   ├── test_sensor_scan.launch.py     # 传感器扫描测试
│   └── test_serial_comm.launch.py     # 串口通信测试
├── config/
│   ├── nav2_params.yaml               # Nav2 导航参数
│   ├── localization.yaml              # 定位参数
│   ├── odom_interface.yaml            # 里程计接口参数
│   ├── sensor_scan_generation.yaml    # 传感器扫描参数
│   ├── decision.yaml                  # 决策参数 (通用)
│   ├── decision_red.yaml              # 红方决策配置
│   └── decision_blue.yaml             # 蓝方决策配置
├── map/
│   ├── default.yaml                   # 地图配置
│   └── default.pgm                    # 2D 栅格地图
├── pcd/
│   └── *.pcd                          # 先验点云地图
├── rviz/
│   ├── nav2_default.rviz              # 导航 RViz 配置
│   └── slam.rviz                      # 建图 RViz 配置
└── behavior_trees/
    └── sentry_mission.xml             # 行为树定义
```

## 启动参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `namespace` | `''` | 顶级命名空间 |
| `use_sim_time` | `false` | 使用仿真时间 |
| `slam` | `false` | 建图模式 (true) 或导航模式 (false) |
| `world` | `default` | 地图名称 |
| `map` | `map/default.yaml` | 地图文件路径 |
| `prior_pcd_file` | `pcd/default.pcd` | 先验点云文件路径 |
| `params_file` | `config/nav2_params.yaml` | Nav2 参数文件 |
| `use_rviz` | `true` | 启动 RViz |
| `use_decision` | `true` | 启动决策系统 |
| `use_perception` | `false` | 启动感知模块 |
| `model_path` | `''` | YOLO 模型路径 |
| `team` | `red` | 队伍颜色 (red/blue) |

## 使用方式

### 完整系统启动

```bash
# 红方 (默认)
ros2 launch rc26_bringup bringup.launch.py

# 蓝方
ros2 launch rc26_bringup bringup.launch.py team:=blue

# 启用感知模块
ros2 launch rc26_bringup bringup.launch.py use_perception:=true model_path:=/path/to/model.bin

# 建图模式
ros2 launch rc26_bringup bringup.launch.py slam:=true

# 不启动 RViz
ros2 launch rc26_bringup bringup.launch.py use_rviz:=false
```

### 单独测试各模块

```bash
# 测试里程计链路
ros2 launch rc26_bringup test_odometry_chain.launch.py

# 测试定位模块
ros2 launch rc26_bringup test_localization.launch.py

# 测试决策系统
ros2 launch rc26_bringup test_decision.launch.py

# 测试感知模块
ros2 launch rc26_bringup test_perception.launch.py

# 测试控制器
ros2 launch rc26_bringup test_omni_controller.launch.py

# 测试串口通信
ros2 launch rc26_bringup test_serial_comm.launch.py
```

## 配置说明

### Nav2 参数 (nav2_params.yaml)

主要配置项：
- `bt_navigator`: 行为树导航器配置
- `controller_server`: 控制器服务配置 (使用 rc26_omni_controller)
- `local_costmap`: 局部代价地图配置
- `global_costmap`: 全局代价地图配置
- `planner_server`: 路径规划器配置

### 定位参数 (localization.yaml)

主要配置项：
- 坐标系配置 (map/odom/base_link/laser_link)
- small_gicp 配准参数
- 绑架检测参数
- 全局重定位参数 (SAC-IA + NDT + ICP)

## 坐标系约定

```
map (全局地图坐标系)
 │
 └── odom (里程计坐标系)
      │
      └── base_link (底盘中心)
           │
           ├── laser_link (激光雷达)
           └── camera_link (相机)
```

## 依赖

- nav2_bringup
- rc26_localization
- rc26_odom_interface
- rc26_sensor_scan
- rc26_decision
- rc26_perception
- point_lio
- mid360_driver

## 编译

```bash
colcon build --packages-select rc26_bringup
```
