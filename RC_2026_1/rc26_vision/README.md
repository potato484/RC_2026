# rc26_vision

## 1. 模块简介 (Introduction)
`rc26_vision` 是 RC2026 机器人系统的核心视觉感知模块，专为复杂动态环境下的实时目标检测与三维精确定位设计。该模块集成了深度学习推理引擎与 RGB-D 深度融合算法，支持多种推理后端（ONNX Runtime, AidLite），能够高效处理来自 RealSense 或类似 RGB-D 相机的数据流。

本模块的核心任务是识别视野中的关键目标（如敌方机器人、特定的比赛道具等），解算其在相机坐标系下的精确三维坐标 $(X, Y, Z)$，并将感知结果实时反馈给决策与控制层，为机器人的自动瞄准、抓取和避障提供关键数据支撑。

## 2. 核心功能 (Core Features)

*   **高性能多后端推理架构**：
    *   **YoloEngine (ONNX Runtime)**: 基于 Microsoft ONNX Runtime 的通用推理实现，具备极高的兼容性，适用于 PC 端开发、调试及验证。
    *   **AidLiteEngine**: 专为嵌入式边缘计算设备（如 AidLux 平台）优化的推理引擎，支持 INT8/FP16 量化模型，显著降低计算延迟与功耗。
*   **RGB-D 深度融合定位**：
    *   创新性地结合 2D 目标检测框 (Bounding Box) 与对齐后的深度图 (Aligned Depth Map)。
    *   采用 **ROI (Region of Interest) 深度滤波算法**，有效剔除物体边缘的背景噪声与无效深度点，实现亚厘米级的测距精度。
*   **动态模型管理**：
    *   支持**配置文件驱动**的模型加载机制（通过 `ProfileLoader`）。
    *   支持运行时**热切换 (Hot-swapping)** 检测模型，允许机器人根据比赛阶段（如巡逻模式 vs. 攻坚模式）灵活调整感知策略。
*   **全异步并发设计**：
    *   图像采集、预处理、推理计算与后处理均在独立线程中执行，最大化利用多核 CPU 资源，确保高帧率（FPS）输出。

## 3. 底层原理 (Underlying Principles)

### 3.1 推理引擎架构 (Inference Engine Architecture)
模块采用策略模式设计，定义了抽象基类 `InferenceEngine`，屏蔽了底层推理框架的差异。

```
InferenceEngine (抽象基类)
    ├── YoloEngine      (ONNX Runtime 实现)
    └── AidLiteEngine   (AidLux 平台加速实现)
```

1.  **预处理 (Pre-processing)**:
    *   **Letterbox 自适应缩放**: 保持图像宽高比缩放至模型输入尺寸（如 640x640），填充灰色边缘，避免图像失真。
    *   **归一化**: 将像素值映射至 $[0, 1]$ 区间。
    *   **格式转换**: 将图像数据布局从 HWC (Height-Width-Channel) 转换为 CHW，符合深度学习模型标准。
2.  **后处理 (Post-processing)**:
    *   **解码**: 将模型输出的 Tensor 解码为 $(cx, cy, w, h, obj\_conf, class\_conf)$。
    *   **NMS (Non-Maximum Suppression)**: 非极大值抑制算法，去除重叠度高且置信度较低的冗余检测框，保留最优结果。

### 3.2 三维坐标解算 (3D Localization)
仅靠 2D 图像无法获得空间深度，模块利用 RGB-D 相机对齐后的深度图进行二次解算：

1.  **目标 ROI 提取**:
    根据 2D 检测框中心 $(cx, cy)$，在深度图中截取一个 $N \times N$ 的邻域窗口（默认 $7 \times 7$ 像素）。
2.  **鲁棒深度滤波**:
    *   **无效值剔除**: 过滤掉深度值为 0 或 NaN 的无效点（通常由高光或遮挡引起）。
    *   **范围门控**: 仅保留在有效探测距离 [`depth_min_m_`, `depth_max_m_`] 内的数据点。
    *   **统计估算**: 计算剩余有效点的中值 (Median) 或均值 (Mean)，作为目标的最终深度 $Z_{target}$，具有极强的抗噪能力。
3.  **针孔相机模型反投影**:
    利用相机内参矩阵 $(f_x, f_y, c_x, c_y)$ 将图像像素坐标 $(u, v)$ 转换为相机坐标系下的空间坐标 $(X, Y, Z)$：
    $$
    \begin{cases}
    Z = Z_{target} \\
    X = \frac{(u - c_x) \cdot Z}{f_x} \\
    Y = \frac{(v - c_y) \cdot Z}{f_y}
    \end{cases}
    $$

### 3.3 目标属性分类 (Attribute Classification)
```cpp
enum class AttributeKind : int {
    Unknown = 0,  // 未知目标
    R_R1 = 1,     // 红方 R1 机器人
    B_R1 = 2,     // 蓝方 R1 机器人
    Truth = 3,    // 真目标 (如真 KFS)
    False = 4     // 假目标 (如假 KFS)
};
```

## 4. 接口说明 (Interface Description)

### 4.1 话题订阅 (Subscribed Topics)
| 话题名称 (Topic) | 消息类型 (Type) | 说明 (Description) |
| :--- | :--- | :--- |
| `color_image` | `sensor_msgs/msg/Image` | RGB 彩色图像流，通常为 `bgr8` 或 `rgb8` 编码。 |
| `depth_image` | `sensor_msgs/msg/Image` | 与 RGB 严格对齐的深度图像流，编码通常为 `16UC1` (单位: 毫米)。 |
| `camera_info` | `sensor_msgs/msg/CameraInfo` | 相机内参信息 (D, K, R, P 矩阵)，用于坐标反投影。 |

### 4.2 结果回调与发布 (Result Callback)
本模块主要通过 C++ 高效回调接口 `VisionInferenceManager::setResultCallback` 向下游提供数据，同时也支持扩展 ROS 2 Topic 发布。

**核心数据结构 `TargetResult`**:
```cpp
struct TargetResult {
    bool has_target;          // 是否检测到有效目标
    AttributeKind attr_kind;  // 目标属性类别 (如: R_R1, Truth, False)
    double distance_m;        // 目标直线距离 (单位: 米)
    double score;             // 检测置信度 (0.0 ~ 1.0)
    int bbox_cx;              // 检测框中心 X 像素坐标
    int bbox_cy;              // 检测框中心 Y 像素坐标
    int64_t timestamp_ns;     // 图像采集时间戳 (纳秒)
};
```

**核心数据结构 `Detection`**:
```cpp
struct Detection {
    float x1, y1, x2, y2;     // 检测框左上角和右下角坐标
    float score;              // 检测置信度
    int class_id;             // 类别 ID
    std::string class_name;   // 类别名称
};
```

### 4.3 C++ API 调用示例
```cpp
#include "rc26_vision/vision_inference_manager.hpp"

// 1. 初始化管理器
auto vision_manager = std::make_shared<rc26_vision::VisionInferenceManager>(*node);

// 2. 方式一：直接配置模型
vision_manager->configure(
    "/path/to/model.onnx",           // 模型路径
    {"R_R1", "B_R1", "Truth", "False"}, // 类别名称
    0.5f                              // 置信度阈值
);

// 2. 方式二：通过配置文件加载
rc26_vision::VisionConfig config;
config.model_path = "/path/to/model.onnx";
config.conf_thresh = 0.5;
vision_manager->loadConfig(config);

// 3. 注册结果回调
vision_manager->setResultCallback([](const rc26_vision::TargetResult& result) {
    if (result.has_target) {
        RCLCPP_INFO(logger, "Target: %d, Dist: %.2fm, Score: %.2f",
                    static_cast<int>(result.attr_kind),
                    result.distance_m,
                    result.score);
    }
});

// 4. 启动推理线程
vision_manager->start();

// 5. 运行时切换模型 (热切换)
vision_manager->switchModel("combat_model");

// 6. 获取最新结果 (轮询方式)
auto result = vision_manager->getLatestResult();

// 7. 停止推理
vision_manager->stop();
```

### 4.4 模型热切换 API
```cpp
// 选择模型 (配置阶段)
void selectModel(const std::string& model_id);

// 切换模型 (运行时热切换)
void switchModel(const std::string& model_id);

// 获取当前激活的模型 ID
std::string getActiveModel() const;
```

## 5. 参数配置 (Configuration)

推荐通过 `config/vision_config.yaml` 或 ROS 2 参数服务器进行配置：

| 参数名 | 类型 | 默认值 | 说明 |
| :--- | :--- | :--- | :--- |
| `model_path` | string | "" | ONNX 或 AidLite 模型文件的绝对路径 |
| `class_names` | list | [] | 类别名称列表，顺序需与模型输出一致 |
| `conf_thresh` | float | 0.45 | 置信度阈值，低于此值的检测框将被过滤 |
| `iou_thresh` | float | 0.45 | NMS 交并比阈值 |
| `depth_roi_size` | int | 7 | 深度采样的矩形区域边长 (像素) |
| `depth_min_valid_count`| int | 10 | 区域内最小有效深度点数，过少则视为测距失败 |
| `depth_min_m` | float | 0.2 | 有效探测最小距离 (米) |
| `depth_max_m` | float | 5.0 | 有效探测最大距离 (米) |

### 5.1 模型配置文件示例 (vision_models.yaml)
```yaml
models:
  default:
    path: "/opt/models/yolov8n.onnx"
    class_names: ["R_R1", "B_R1", "Truth", "False"]
    conf_thresh: 0.45
    iou_thresh: 0.45
    engine: "onnx"  # onnx 或 aidlite

  combat_model:
    path: "/opt/models/yolov8s_combat.onnx"
    class_names: ["enemy", "ally", "target"]
    conf_thresh: 0.5
    iou_thresh: 0.4
    engine: "onnx"

  merlin_model:
    path: "/opt/models/kfs_detector.dlc"
    class_names: ["auto_kfs", "manual_kfs", "fake_kfs"]
    conf_thresh: 0.6
    iou_thresh: 0.5
    engine: "aidlite"

active_model: "default"
```

## 6. 启动示例 (Usage)

### 6.1 作为库集成到其他节点
```cpp
#include "rc26_vision/vision_inference_manager.hpp"

class MyDecisionNode : public rclcpp::Node {
public:
    MyDecisionNode() : Node("my_decision_node") {
        vision_manager_ = std::make_shared<rc26_vision::VisionInferenceManager>(*this);

        // 配置并启动
        vision_manager_->configure("/opt/models/yolov8n.onnx",
                                   {"R_R1", "B_R1", "Truth", "False"},
                                   0.5f);
        vision_manager_->setResultCallback(
            std::bind(&MyDecisionNode::onVisionResult, this, std::placeholders::_1));
        vision_manager_->start();
    }

private:
    void onVisionResult(const rc26_vision::TargetResult& result) {
        if (result.has_target && result.distance_m < 2.0) {
            // 执行决策逻辑
        }
    }

    std::shared_ptr<rc26_vision::VisionInferenceManager> vision_manager_;
};
```

### 6.2 模型转换脚本
```bash
# PyTorch 模型转 ONNX
python3 src/scripts/pytorch_to_onnx.py \
    --weights /path/to/yolov8n.pt \
    --output /opt/models/yolov8n.onnx \
    --imgsz 640
```

## 7. 目录结构 (Directory Structure)
```
rc26_vision/
├── include/rc26_vision/
│   ├── vision_inference_manager.hpp  # 核心管理类，负责任务调度与数据流
│   ├── inference_engine.hpp          # 推理引擎抽象基类
│   ├── yolo_engine.hpp               # YoloV5/V8 ONNX Runtime 实现
│   ├── aidlite_engine.hpp            # AidLux 平台加速实现
│   ├── model_profile.hpp             # 模型配置结构体
│   ├── profile_loader.hpp            # 配置文件加载器
│   └── types.hpp                     # 数据结构定义 (Detection, TargetResult)
├── src/
│   ├── vision_inference_manager.cpp  # 管理器实现
│   ├── yolo_engine.cpp               # ONNX Runtime 引擎实现
│   ├── aidlite_engine.cpp            # AidLite 引擎实现
│   ├── profile_loader.cpp            # 配置加载器实现
│   └── scripts/
│       ├── pytorch_to_onnx.py        # 模型转换脚本
│       ├── vision_test_node.cpp      # 测试节点
│       └── yolo_inference_test.cpp   # 推理测试
├── models/                           # 预训练模型文件 (.onnx, .dlc)
├── config/                           # 配置文件 (.yaml)
├── launch/                           # Launch 文件（相机/联调入口）
├── package.xml                       # ROS 2 包描述
├── CMakeLists.txt                    # 构建配置
└── README.md                         # 本文档
```

## 7.1 快速启动（ROS）

```bash
# 启动 RealSense D455（仅相机）
ros2 launch rc26_vision realsense_d455.launch.py

# 启动相机 + 视觉测试节点（推荐用于独立联调）
ros2 launch rc26_vision vision_test_with_camera.launch.py
```

## 8. 依赖项 (Dependencies)
*   `rclcpp`: ROS 2 C++ 客户端库
*   `sensor_msgs`: 图像消息类型
*   `cv_bridge`: OpenCV 与 ROS 消息转换
*   `OpenCV`: 图像处理
*   `ONNX Runtime`: 深度学习推理框架 (YoloEngine)
*   `AidLite SDK`: 边缘计算推理框架 (AidLiteEngine, 可选)

## 9. 性能优化建议
*   **模型量化**: 使用 INT8/FP16 量化模型可显著提升推理速度。
*   **输入分辨率**: 根据实际需求调整模型输入尺寸（如 640x640 → 416x416）。
*   **深度 ROI 大小**: 较小的 `depth_roi_size` 可减少计算量，但可能降低测距稳定性。
*   **置信度阈值**: 适当提高 `conf_thresh` 可减少后处理开销。
