# rc26_nav_mode_manager

## 1. 模块简介 (Introduction)
`rc26_nav_mode_manager` 是导航系统的**安全模式管理器**。它负责监控机器人状态，根据外部指令或异常情况（如超时、堵转）动态切换导航模式，并管理 Costmap 的清理与层级控制。

本模块是机器人导航安全的核心保障，能够在检测到异常时自动触发保护机制，防止机器人陷入危险状态。

## 2. 核心功能 (Core Features)
*   **导航模式切换**：
    *   `NORMAL`: 正常导航模式，机器人按照规划路径行驶。
    *   `SLOW`: 低速/精密模式（可选），用于精细操作场景。
    *   `STOP`: 强制急停模式，立即停止所有运动。
*   **Costmap 管理**：提供服务接口以清除全局或局部代价地图，常用于解决机器人陷入"假障碍物"包围的困境。
*   **自动回退/恢复**：检测机器人是否长时间未移动（超时），自动触发 Costmap 清理或重置导航状态。
*   **速度监控**：监听里程计速度，判断机器人是否物理静止。
*   **Drop Layer 控制**：动态启用/禁用 Costmap 的 Drop Layer，适应不同地形场景。

## 3. 底层原理 (Underlying Principles)

### 3.1 速度监测与静止判定
通过订阅 `odom` 话题，计算线速度和角速度模长：
$$ v = \sqrt{v_x^2 + v_y^2}, \quad \omega = |\omega_z| $$
若 $v < \epsilon_{linear}$ 且 $\omega < \epsilon_{angular}$ 持续一定时间，判定为 `RobotStopped`。

**判定参数**:
*   `stop_linear_eps_mps`: 线速度静止阈值 (默认 0.05 m/s)
*   `stop_angular_eps_rps`: 角速度静止阈值 (默认 0.05 rad/s)

### 3.2 Costmap 层级控制
通过 ROS 2 Parameter Client 动态修改 Navigation Stack（如 Nav2）的参数。
*   例如：在特定区域（如平坦地面）禁用 `voxel_layer` 或 `obstacle_layer`，仅保留静态地图，以减少噪点干扰。
*   通过 `setDropLayerEnabled(bool)` 方法控制 Drop Layer 的启用状态。

### 3.3 超时机制
内置定时器 `timeout_timer_`。若在 `current_timeout_sec_` 时间内未收到特定刷新信号或达成目标，触发超时处理：
1.  发布 `STOP` 状态。
2.  尝试清理 Costmap。
3.  等待 Costmap 重建完成。

### 3.4 模式切换状态机
```
┌────────┐  set_nav_mode(STOP)  ┌──────┐
│ NORMAL │ ──────────────────→ │ STOP │
└────┬───┘                     └──┬───┘
     │                            │
     │  set_nav_mode(SLOW)        │ set_nav_mode(NORMAL)
     ↓                            ↓
┌────────┐                    ┌────────┐
│  SLOW  │ ←───────────────── │ NORMAL │
└────────┘  set_nav_mode(SLOW) └────────┘
```

## 4. 接口说明 (Interface Description)

### 4.1 服务 (Services)
| 服务名 (Service) | 类型 (Type) | 说明 |
| :--- | :--- | :--- |
| `set_nav_mode` | `rc26_interfaces/srv/SetNavMode` | 外部请求切换导航模式 |

**SetNavMode 服务定义**:
```
# Request
uint8 mode    # 0: NORMAL, 1: SLOW, 2: STOP

# Response
bool success
string message
```

### 4.2 话题订阅 (Subscribed Topics)
| 话题 (Topic) | 类型 (Type) | 说明 |
| :--- | :--- | :--- |
| `odom` | `nav_msgs/msg/Odometry` | 用于速度监控，判断机器人是否静止 |
| `terrain_obstacles` | `sensor_msgs/msg/PointCloud2` | (可选) 监控障碍物密度 |

### 4.3 话题发布 (Published Topics)
| 话题 (Topic) | 类型 (Type) | 说明 |
| :--- | :--- | :--- |
| `nav_safety_mode` | `rc26_interfaces/msg/NavSafetyMode` | 广播当前生效的安全模式 |

**NavSafetyMode 消息定义**:
```
uint8 NORMAL = 0
uint8 SLOW = 1
uint8 STOP = 2

uint8 mode
builtin_interfaces/Time stamp
string reason
```

### 4.4 客户端 (Clients)
| 客户端 (Client) | 类型 (Type) | 说明 |
| :--- | :--- | :--- |
| `clear_entire_costmap` | `nav2_msgs/srv/ClearEntireCostmap` | 调用 Nav2 的清除地图服务 |

### 4.5 参数配置 (Parameters)

通过 `config/nav_mode_manager.yaml` 配置：

| 参数名 | 类型 | 默认值 | 说明 |
| :--- | :--- | :--- | :--- |
| `costmap_node_name` | string | `local_costmap/local_costmap` | Costmap 节点名称 |
| `odom_topic` | string | `odom` | 里程计话题 |
| `obstacles_topic` | string | `terrain_obstacles` | 障碍物点云话题 |
| `default_timeout_sec` | double | 5.0 | 默认超时时间 (秒) |
| `stop_linear_eps_mps` | double | 0.05 | 线速度静止阈值 (m/s) |
| `stop_angular_eps_rps` | double | 0.05 | 角速度静止阈值 (rad/s) |
| `param_timeout_sec` | double | 2.0 | 参数服务超时时间 (秒) |
| `clear_timeout_sec` | double | 2.0 | 清除 Costmap 超时时间 (秒) |
| `rebuild_timeout_sec` | double | 0.5 | Costmap 重建等待时间 (秒) |

## 5. 启动示例 (Usage)

### 5.1 命令行启动
```bash
# 使用默认参数启动
ros2 launch rc26_nav_mode_manager nav_mode_manager.launch.py

# 指定命名空间和参数
ros2 launch rc26_nav_mode_manager nav_mode_manager.launch.py \
    namespace:=robot1 \
    use_sim_time:=true \
    params_file:=/path/to/custom_params.yaml
```

### 5.2 Launch 文件参数
| 参数 | 默认值 | 说明 |
| :--- | :--- | :--- |
| `namespace` | `""` | 顶级命名空间 |
| `use_sim_time` | `false` | 是否使用仿真时间 |
| `params_file` | `config/nav_mode_manager.yaml` | 参数文件路径 |

### 5.3 在其他 Launch 文件中包含
```python
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        PathJoinSubstitution([
            FindPackageShare('rc26_nav_mode_manager'), 'launch', 'nav_mode_manager.launch.py'
        ])
    ]),
    launch_arguments={
        'namespace': 'robot1',
        'use_sim_time': 'false'
    }.items()
)
```

### 5.4 调用服务切换模式示例

**命令行调用**:
```bash
# 切换到急停模式
ros2 service call /set_nav_mode rc26_interfaces/srv/SetNavMode "{mode: 2}"

# 切换到正常模式
ros2 service call /set_nav_mode rc26_interfaces/srv/SetNavMode "{mode: 0}"
```

**C++ 调用**:
```cpp
#include "rc26_interfaces/srv/set_nav_mode.hpp"

class MyNode : public rclcpp::Node {
public:
    MyNode() : Node("my_node") {
        client_ = create_client<rc26_interfaces::srv::SetNavMode>("set_nav_mode");
    }

    void emergencyStop() {
        auto request = std::make_shared<rc26_interfaces::srv::SetNavMode::Request>();
        request->mode = 2;  // STOP

        auto future = client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(shared_from_this(), future) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(get_logger(), "Emergency stop activated");
            }
        }
    }

private:
    rclcpp::Client<rc26_interfaces::srv::SetNavMode>::SharedPtr client_;
};
```

### 5.5 订阅安全模式状态
```cpp
#include "rc26_interfaces/msg/nav_safety_mode.hpp"

class MyNode : public rclcpp::Node {
public:
    MyNode() : Node("my_node") {
        mode_sub_ = create_subscription<rc26_interfaces::msg::NavSafetyMode>(
            "nav_safety_mode", 10,
            [this](const rc26_interfaces::msg::NavSafetyMode::SharedPtr msg) {
                switch (msg->mode) {
                    case rc26_interfaces::msg::NavSafetyMode::NORMAL:
                        RCLCPP_INFO(get_logger(), "Mode: NORMAL");
                        break;
                    case rc26_interfaces::msg::NavSafetyMode::SLOW:
                        RCLCPP_INFO(get_logger(), "Mode: SLOW");
                        break;
                    case rc26_interfaces::msg::NavSafetyMode::STOP:
                        RCLCPP_WARN(get_logger(), "Mode: STOP - Reason: %s", msg->reason.c_str());
                        break;
                }
            });
    }

private:
    rclcpp::Subscription<rc26_interfaces::msg::NavSafetyMode>::SharedPtr mode_sub_;
};
```

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
