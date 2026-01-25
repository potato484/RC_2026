# rc26_vision

`rc26_vision` 是 RC2026 机器人的视觉感知功能包。它基于 YOLO 算法对彩色图像进行目标检测，并结合对齐后的深度图像，解算出目标的类别、属性及距离信息，为上层决策与控制模块提供关键的感知数据。

## 核心功能

*   **YOLO 目标检测**：基于 ONNX Runtime C++ 推理引擎，支持 YOLOv5/YOLOv8 模型，兼容多种输出布局。
*   **深度测距**：通过对检测框中心区域的深度数据进行采样、过滤和中值计算，提供稳健的目标距离估计。
*   **ROS 2 集成**：封装为标准的 ROS 2 组件，通过 `VisionInferenceManager` 管理图像订阅、推理循环及结果分发。
*   **异步推理架构**：图像接收与模型推理线程解耦，确保高频图像流下的系统响应性能。

## 依赖环境

*   **ROS 2** (Humble 或更高版本)
*   **OpenCV** (需支持 C++ 接口)
*   **ONNX Runtime** (C++ API)
*   **cv_bridge**

## 架构与原理

### 1. 数据处理流程

1.  **数据采集**：订阅相机的 RGB 图像、深度图像及相机内参。
2.  **异步推理**：RGB 图像到达触发推理线程，线程获取最新的 RGB 帧及对应的深度帧缓存。
3.  **目标检测**：YOLO 引擎处理 RGB 图像，输出候选检测框，并根据置信度筛选出最优目标。
4.  **距离解算**：
    *   在深度图中截取目标中心周围的感兴趣区域 (ROI)。
    *   过滤无效值（如 0、NaN、Inf）及超出预设范围的噪点。
    *   计算区域内有效深度的**中位数**作为最终距离 `distance_m`，以消除离群点影响。
5.  **结果输出**：将目标类别映射为业务属性（如红/蓝方、真/假目标），并通过回调函数或查询接口对外发布。

### 2. 模块化设计

为了保证代码的复用性与清晰度，功能被解耦为两层：

*   **`YoloEngine`**：纯粹的推理引擎。负责调用 ONNX Runtime 执行推理和后处理（NMS），不依赖 ROS，便于独立测试或移植。
*   **`VisionInferenceManager`**：ROS 适配层。负责节点通信、线程管理、参数配置以及将 2D 检测结果与深度信息融合。

### 3. 就绪状态判定

`VisionInferenceManager::isReady()` 用于指示感知系统是否正常工作，需同时满足：
*   配置参数加载成功且模型初始化完成。
*   最近 1 秒内接收到有效的图像数据（防止使用陈旧数据）。

## 接口说明

### ROS 话题订阅

默认订阅以下话题（可通过参数重映射）：

| 默认话题名 | 消息类型 | 说明 |
| :--- | :--- | :--- |
| `/camera/color/image_raw` | `sensor_msgs/msg/Image` | RGB 彩色图像 |
| `/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/msg/Image` | 与 RGB 严格对齐的深度图像 |
| `/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | 相机内参 |

> **注意**：本模块默认**不发布**检测结果话题，而是通过 `VisionInferenceManager` 提供 C++ 接口供其他模块直接调用，以减少通信延迟。

### C++ API

`rc26_vision::VisionInferenceManager` 是外部调用的主入口。

#### 初始化与控制

*   **构造**：`VisionInferenceManager(rclcpp::Node& node)`
    *   初始化参数并建立话题订阅。
*   **配置**：`bool configure(const std::string& model_path, const std::vector<std::string>& class_names, float conf_thresh)`
    *   加载 ONNX 模型，设置类别名称和置信度阈值。
*   **运行控制**：
    *   `bool start()`: 启动推理线程。
    *   `void stop()`: 停止推理线程。
*   **状态查询**：
    *   `bool isRunning()`: 线程是否运行中。
    *   `bool isReady()`: 系统是否就绪（模型已加载且有实时数据）。

#### 结果获取

*   **获取最新结果**：`TargetResult getLatestResult() const`
    *   线程安全地获取最近一次的推理结果。
*   **设置回调**：`void setResultCallback(ResultCallback cb)`
    *   注册回调函数，每次推理更新后自动触发。

#### 数据结构

**`struct TargetResult`**

| 字段 | 类型 | 说明 |
| :--- | :--- | :--- |
| `has_target` | bool | 是否检测到有效目标（检测框存在且距离有效） |
| `attr_kind` | AttributeKind | 目标属性 (R_R1, B_R1, Truth, False, Unknown) |
| `distance_m` | double | 目标距离（米），无效时为 0 |
| `score` | float | 目标置信度 |
| `bbox_cx`, `bbox_cy` | float | 目标框中心像素坐标 |
| `timestamp_ns` | int64_t | 图像采集时间戳 |

### 参数配置

#### 节点参数 (ROS Parameters)

可直接在 launch 文件中配置：

| 参数名 | 类型 | 默认值 | 说明 |
| :--- | :--- | :--- | :--- |
| `vision_color_topic` | string | `/camera/color/image_raw` | RGB 图像话题 |
| `vision_depth_topic` | string | `/camera/aligned_depth_to_color/image_raw` | 深度图像话题 |
| `vision_depth_roi` | int | 7 | 测距 ROI 边长（像素），以目标中心为原点 |
| `vision_depth_min_valid_count` | int | 10 | ROI 内最小有效像素数，不足则认为距离无效 |
| `vision_depth_min_m` | double | 0.2 | 有效距离下限（米） |
| `vision_depth_max_m` | double | 5.0 | 最大有效距离上限（米） |

#### 运行时参数 (Configure API)

*   `model_path`: ONNX 模型绝对路径。
*   `class_names`: 模型输出对应的类别名称列表（需与训练时一致）。
*   `conf_thresh`: 置信度过滤阈值 (0.0 - 1.0)。

## 类别映射规则

为了适应业务逻辑，模型输出的类别名 (`class_name`) 会被映射为 `AttributeKind` 枚举：

*   **红方机器人**: `R_R1` → `AttributeKind::R_R1`
*   **蓝方机器人**: `B_R1` → `AttributeKind::B_R1`
*   **真目标**: 以 `T_` 开头 (如 `T_1`, `T_guard`) → `AttributeKind::Truth`
*   **假目标**: 以 `F_` 开头 (如 `F_1`) → `AttributeKind::False`
*   **其他**: → `AttributeKind::Unknown`

## 测试工具

`vision_test_node` 用于独立验证模型加载与图像订阅是否正常。

### 启动命令

```bash
ros2 run rc26_vision vision_test_node --ros-args -p vision_model_path:=/path/to/model.onnx
```

### 专用参数

*   `vision_model_path` (必须): 模型路径。
*   `vision_conf_thresh`: 阈值，默认 0.5。
*   `print_rate_ms`: 终端打印频率，默认 500ms。

## 注意事项

1.  **对齐要求**：必须使用与 RGB 严格对齐的深度图 (`aligned_depth_to_color`)，否则测距将出现严重偏差。
2.  **数据格式**：深度图支持 `16UC1` (mm) 或 `32FC1` (m) 编码。
3.  **多目标策略**：当前逻辑仅输出**置信度最高**的一个目标。如需多目标跟踪或特定优先级策略，需修改 `VisionInferenceManager` 中的筛选逻辑。
4.  **硬件加速**：请确保 ONNX Runtime 已正确安装并链接了对应的硬件加速库（如 CUDA/TensorRT），否则推理速度可能无法满足实时性要求。