# rc26_nav_mode_manager

## 1. 模块简介
`rc26_nav_mode_manager` 是导航系统的**安全模式管理器**，统一对外提供：
- 安全模式切换服务：`nav_safety/set_mode`（`rc26_interfaces/srv/SetNavMode`）
- 安全模式状态发布：`nav_safety/state`（`rc26_interfaces/msg/NavSafetyMode`）
- 对 Nav2 local costmap 的 `drop_layer.enabled` 进行动态控制，并在切换关键模式时清图/等待重建
- 对关键模式启用超时保护：超时会自动回退并发出“需要停止”的信号，供上层（决策/导航执行器）中断导航

## 2. 支持的模式（与接口定义一致）
模式定义在 `rc26_interfaces/msg/NavSafetyMode.msg`：
- `NORMAL = 0`
- `MF_SAFE = 1`
- `MF_TRAVERSE = 2`
- `MF_EXIT = 3`

其中 `MF_TRAVERSE/MF_EXIT` 是“临时放开/穿越”的模式：本节点会**禁用** costmap 的 drop layer，并启动一个超时定时器；超时则回退到 `MF_SAFE`。

## 3. 核心行为

### 3.1 里程计速度监测（停稳判定）
订阅 `odom`，根据速度阈值判断机器人是否“停稳”：
- `sqrt(vx^2 + vy^2) < stop_linear_eps_mps`
- `abs(wz) < stop_angular_eps_rps`

当外部请求进入 `MF_TRAVERSE/MF_EXIT` 时，若未停稳会拒绝切换（服务返回失败）。

### 3.2 Drop Layer 与清图/重建确认
切换到 `MF_TRAVERSE/MF_EXIT` 时：
1. 通过参数服务设置 `drop_layer.enabled=false`
2. 调用 `ClearEntireCostmap` 清除 local costmap
3. 通过订阅 `terrain_obstacles` 的时间戳是否更新，粗略确认清图后 costmap 已重新接收障碍信息（重建开始/恢复）

切回 `NORMAL/MF_SAFE` 时：
- 设 `drop_layer.enabled=true`
- 取消超时定时器

### 3.3 超时回退（仅对 MF_TRAVERSE/MF_EXIT）
进入 `MF_TRAVERSE/MF_EXIT` 后会启动一个 `timeout` 定时器：
- 定时器触发时：强制回退到 `MF_SAFE`
- 立刻发布 `nav_safety/state`，并置 `stop_required=true`、`timed_out=true`

上层模块（如导航执行器）可订阅该话题以中断当前导航并执行停机策略。

## 4. 接口说明

### 4.1 服务（Services）
| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `nav_safety/set_mode` | `rc26_interfaces/srv/SetNavMode` | 请求切换安全模式（可带超时与原因） |

`SetNavMode` 请求字段：
- `mode`：目标模式（0..3）
- `timeout`：当目标为 `MF_TRAVERSE/MF_EXIT` 时的超时时间（秒）；`<=0` 使用默认值
- `reason`：切换原因（会写入状态消息的 `reason`）

### 4.2 发布（Published Topics）
| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `nav_safety/state` | `rc26_interfaces/msg/NavSafetyMode` | 当前模式与安全状态（含 `stop_required/timed_out`） |

### 4.3 订阅（Subscribed Topics）
| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `odom` | `nav_msgs/msg/Odometry` | 用于停稳判定 |
| `terrain_obstacles` | `sensor_msgs/msg/PointCloud2` | 用于清图后重建确认（时间戳更新） |

## 5. 参数（config/nav_mode_manager.yaml）
| 参数名 | 类型 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `costmap_node_name` | string | `local_costmap/local_costmap` | local costmap 参数节点名 |
| `odom_topic` | string | `odom` | odom 话题 |
| `obstacles_topic` | string | `terrain_obstacles` | 障碍点云话题 |
| `default_timeout_sec` | double | 5.0 | `timeout<=0` 时使用的默认超时 |
| `stop_linear_eps_mps` | double | 0.05 | 停稳线速度阈值 |
| `stop_angular_eps_rps` | double | 0.05 | 停稳角速度阈值 |
| `param_timeout_sec` | double | 2.0 | 设置参数等待超时 |
| `clear_timeout_sec` | double | 2.0 | 清图服务等待超时 |
| `rebuild_timeout_sec` | double | 0.5 | 等待重建确认超时（障碍点云刷新） |

关于清图服务名：
- 代码会用 `costmap_node_name + "/clear_entirely_local_costmap"` 拼接
- 若 `costmap_node_name` 包含 `local_costmap/local_costmap`，会替换成 `local_costmap/clear_entirely_local_costmap` 以兼容常见 Nav2 命名

## 6. 启动与使用示例

### 6.1 Launch 启动
```bash
ros2 launch rc26_nav_mode_manager nav_mode_manager.launch.py

ros2 launch rc26_nav_mode_manager nav_mode_manager.launch.py \
  namespace:=robot1 \
  use_sim_time:=true \
  params_file:=/path/to/custom_params.yaml
```

### 6.2 命令行切换模式
```bash
# 切到 MF_TRAVERSE，10 秒超时（要求机器人已停稳）
ros2 service call /nav_safety/set_mode rc26_interfaces/srv/SetNavMode "{mode: 2, timeout: 10.0, reason: 'stairs'}"

# 切回 MF_SAFE（会启用 drop_layer，并取消定时器）
ros2 service call /nav_safety/set_mode rc26_interfaces/srv/SetNavMode "{mode: 1, timeout: 0.0, reason: 'post_nav'}"
```

### 6.3 查看状态
```bash
ros2 topic echo /nav_safety/state
```

## 7. 与上层模块的典型配合方式
- 上层（例如 `rc26_decision` 的导航执行器）在“开始导航前”调用 `nav_safety/set_mode` 进入目标模式
- 导航过程中订阅 `nav_safety/state`，若看到 `stop_required/timed_out` 则取消 Nav2 goal 并停机
- 导航结束后（若进入过 `MF_TRAVERSE/MF_EXIT`）再调用 `nav_safety/set_mode` 切回 `MF_SAFE` 或 `NORMAL`

## 6. 目录结构 (Directory Structure)
```
rc26_nav_mode_manager/
├── include/rc26_nav_mode_manager/
│   └── nav_mode_manager.hpp         # 节点类定义
├── src/
│   └── nav_mode_manager.cpp         # 节点实现
├── config/
│   └── nav_mode_manager.yaml        # 默认参数配置
├── launch/
│   └── nav_mode_manager.launch.py   # 启动文件
├── package.xml                      # ROS 2 包描述
├── CMakeLists.txt                   # 构建配置
└── README.md                        # 本文档
```

## 7. 依赖项 (Dependencies)
*   `rclcpp`: ROS 2 C++ 客户端库
*   `nav_msgs`: 里程计消息类型
*   `sensor_msgs`: 点云消息类型
*   `nav2_msgs`: Nav2 服务类型 (ClearEntireCostmap)
*   `rc26_interfaces`: 自定义消息和服务类型

## 8. 与 Nav2 集成
本模块需要与 Nav2 导航栈配合使用。确保以下服务可用：
*   `/{costmap_node_name}/clear_entirely_local_costmap`
*   `/{costmap_node_name}/clear_entirely_global_costmap`

**Nav2 参数配置示例**:
```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      # 确保 Costmap 节点名称与 nav_mode_manager 配置一致
      # ...
```

## 9. 典型应用场景
*   **导航超时恢复**: 机器人长时间无法到达目标时，自动清理 Costmap 并重试。
*   **紧急停止**: 检测到危险情况时，通过服务调用立即停止机器人。
*   **地形适应**: 在不同地形区域动态启用/禁用特定 Costmap 层。
