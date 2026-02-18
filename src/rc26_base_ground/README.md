# rc26_base_ground

## 1. 模块简介 (Introduction)
`rc26_base_ground` 模块用于估计机器人的**基准地面高度**及**自身稳定性状态**。在多层结构或阶梯地形中，准确获知当前所处的"楼层"高度对于导航规划和底盘控制至关重要。

本模块通过分析机器人的里程计数据（位置与姿态），实时判断机器人是否处于稳定平面、是否正在进行上下台阶动作，并发布相应的状态信息供其他模块使用。

## 2. 核心功能 (Core Features)
*   **地面高度自适应**：利用滑动窗口算法，实时分析机器人的 Z 轴高度和姿态（Roll/Pitch），判断机器人是否处于稳定平面。
*   **楼层/阶梯识别**：检测高度突变，识别机器人是否完成了上/下台阶动作，并更新基准高度 `current_level`。
*   **稳定性检测**：输出 `stable` 信号，指示机器人当前是否处于静止或平稳行驶状态，用于触发高精度操作（如机械臂动作）。
*   **TF 发布**：发布 `base_ground` 坐标系，作为局部导航的参考基准平面。

## 3. 底层原理 (Underlying Principles)

### 3.1 状态机 (State Machine)
```
┌─────────────┐     高度稳定      ┌─────────┐
│ Calibrating │ ───────────────→ │ Stable  │
└─────────────┘                  └────┬────┘
                                      │
                              检测到高度变化
                                      ↓
                               ┌──────────────┐
                               │ Transitioning │
                               └──────┬───────┘
                                      │
                              高度再次稳定
                                      ↓
                               ┌─────────┐
                               │ Stable  │ (更新 level)
                               └─────────┘
```

*   **Calibrating (校准中)**: 初始化阶段，收集数据以确定初始地面高度 `h0`。
*   **Stable (稳定)**: 机器人在平面上运行，高度波动在 `tol` 范围内，且姿态平稳。
*   **Transitioning (过渡中)**: 检测到高度显著变化（如正在爬坡或上下台阶），暂停基准高度更新，直到再次进入稳定状态。

### 3.2 滑动窗口稳定性判据
维护一个时间窗口（如 0.5s）的历史样本 `std::deque<Sample>`。

**Sample 结构**:
```cpp
struct Sample {
    rclcpp::Time stamp;
    double z;      // Z 轴高度
    double roll;   // 横滚角
    double pitch;  // 俯仰角
};
```

**判定条件**:
1.  **高度一致性**：窗口内所有样本的 Z 值极差 $\max(Z) - \min(Z) < \text{tol}$。
2.  **姿态水平**：窗口内所有样本的 Roll 和 Pitch 均小于设定阈值。

### 3.3 阶梯判定
当从 `Transitioning` 恢复到 `Stable` 时，比较新旧稳定高度：
$$ \Delta H = H_{new} - H_{old} $$
*   若 $\Delta H \approx +\text{step\_height\_m}$，判定为**上台阶**，`stair_delta = +1`。
*   若 $\Delta H \approx -\text{step\_height\_m}$，判定为**下台阶**，`stair_delta = -1`。
*   更新 `current_level` 索引。

### 3.4 候选确认机制
为防止误判，引入候选确认时间 `T_confirm`：
*   当检测到可能的楼层变化时，进入候选状态。
*   只有候选状态持续超过 `T_confirm` 时间后，才正式确认楼层变化。

## 4. 接口说明 (Interface Description)

### 4.1 话题订阅 (Subscribed Topics)
| 话题 (Topic) | 类型 (Type) | 说明 |
| :--- | :--- | :--- |
| `odom` | `nav_msgs/msg/Odometry` | 里程计数据，提供 Pose (Z, Roll, Pitch)。 |

### 4.2 话题发布 (Published Topics)
| 话题 (Topic) | 类型 (Type) | 说明 |
| :--- | :--- | :--- |
| `level` | `std_msgs/msg/Int32` | 当前楼层索引 (0, 1, -1, 2, ...)。 |
| `stair_delta` | `std_msgs/msg/Int8` | 最近一次阶梯变化 (+1: 上台阶, -1: 下台阶, 0: 无变化)。 |
| `stable` | `std_msgs/msg/Bool` | 机器人是否处于稳定状态。 |

### 4.3 TF 发布
| 父坐标系 | 子坐标系 | 说明 |
| :--- | :--- | :--- |
| `parent_frame` (默认 `odom`) | `base_ground` | 基准地面坐标系，Z 轴对齐当前楼层高度。 |

### 4.4 参数配置 (Parameters)

通过 `config/base_ground_estimator.yaml` 配置：

| 参数名 | 类型 | 默认值 | 说明 |
| :--- | :--- | :--- | :--- |
| `odom_topic` | string | `odom` | 里程计话题名称 |
| `step_height_m` | double | 0.20 | 阶梯标准高度 (米)，用于容差判断 |
| `tol` | double | 0.06 | 高度波动容差 (米) |
| `T_stable` | double | 0.5 | 判定稳定的最小持续时间 (秒) |
| `T_confirm` | double | 0.3 | 楼层变化确认时间 (秒) |
| `parent_frame` | string | `odom` | TF 父坐标系 |
| `base_ground_frame` | string | `base_ground` | TF 子坐标系 |
| `enable_tf_publish` | bool | true | 是否发布 TF 变换 |
| `tf_timeout_sec` | double | 0.05 | TF 查询超时时间 (秒) |

## 5. 启动示例 (Usage)

### 5.1 命令行启动
```bash
# 使用默认参数启动
ros2 launch rc26_base_ground base_ground_estimator.launch.py

# 指定参数
ros2 launch rc26_base_ground base_ground_estimator.launch.py \
    use_sim_time:=true \
    parent_frame:=odom \
    config_file:=/path/to/custom_config.yaml
```

### 5.2 Launch 文件参数
| 参数 | 默认值 | 说明 |
| :--- | :--- | :--- |
| `use_sim_time` | `false` | 是否使用仿真时间 |
| `parent_frame` | `odom` | TF 父坐标系 |
| `config_file` | `config/base_ground_estimator.yaml` | 参数文件路径 |

### 5.3 在其他 Launch 文件中包含
```python
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        PathJoinSubstitution([
            FindPackageShare('rc26_base_ground'), 'launch', 'base_ground_estimator.launch.py'
        ])
    ]),
    launch_arguments={
        'use_sim_time': 'false',
        'parent_frame': 'odom'
    }.items()
)
```

### 5.4 订阅楼层变化示例
```cpp
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/bool.hpp"

class MyNode : public rclcpp::Node {
public:
    MyNode() : Node("my_node") {
        // 订阅楼层索引
        level_sub_ = create_subscription<std_msgs::msg::Int32>(
            "level", 10,
            [this](const std_msgs::msg::Int32::SharedPtr msg) {
                RCLCPP_INFO(get_logger(), "Current level: %d", msg->data);
            });

        // 订阅阶梯变化事件
        stair_sub_ = create_subscription<std_msgs::msg::Int8>(
            "stair_delta", 10,
            [this](const std_msgs::msg::Int8::SharedPtr msg) {
                if (msg->data > 0) {
                    RCLCPP_INFO(get_logger(), "Climbed up a stair!");
                } else if (msg->data < 0) {
                    RCLCPP_INFO(get_logger(), "Descended a stair!");
                }
            });

        // 订阅稳定性状态
        stable_sub_ = create_subscription<std_msgs::msg::Bool>(
            "stable", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                if (msg->data) {
                    // 机器人稳定，可以执行精密操作
                }
            });
    }

private:
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr level_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr stair_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stable_sub_;
};
```

## 6. 目录结构 (Directory Structure)
```
rc26_base_ground/
├── include/rc26_base_ground/
│   └── base_ground_estimator.hpp    # 节点类定义
├── src/
│   └── base_ground_estimator_node.cpp  # 节点实现
├── config/
│   └── base_ground_estimator.yaml   # 默认参数配置
├── launch/
│   └── base_ground_estimator.launch.py  # 启动文件
├── package.xml                      # ROS 2 包描述
├── CMakeLists.txt                   # 构建配置
└── README.md                        # 本文档
```

## 7. 依赖项 (Dependencies)
*   `rclcpp`: ROS 2 C++ 客户端库
*   `nav_msgs`: 里程计消息类型
*   `std_msgs`: 标准消息类型 (Int32, Int8, Bool)
*   `geometry_msgs`: 几何消息类型
*   `tf2_ros`: TF2 变换库

## 8. 典型应用场景
*   **梅林区阶梯导航**: 机器人在上下阶梯时，通过 `level` 和 `stair_delta` 信息调整导航策略。
*   **机械臂精密操作**: 在 `stable = true` 时才执行抓取动作，避免因机器人晃动导致抓取失败。
*   **多层地图切换**: 根据 `level` 值切换对应楼层的静态地图。
