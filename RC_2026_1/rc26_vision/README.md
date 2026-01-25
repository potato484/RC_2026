# rc26_vision

`rc26_vision` 是 RC2026 机器人的视觉感知功能包。它集成了 YOLO (You Only Look Once) 目标检测模型与深度相机数据处理，用于实时识别场上的目标物体（如 R1、兑换站标志等）并计算其三维空间距离。

## 功能特性

*   **YOLO 目标检测**: 基于 ONNX Runtime 推理引擎，支持 YOLOv5/v8 模型。
*   **深度感知**: 结合 RGB 图像与对齐后的深度图像，计算检测目标的实际距离。
*   **ROS 2 集成**: 提供标准的 ROS 2 接口，通过 `VisionInferenceManager` 类轻松集成到其他节点中。
*   **多线程处理**: 推理与图像接收分离，保证实时性。

## 依赖项

*   **ROS 2** (Humble 或更高版本)
*   **OpenCV** (需支持 C++ 接口)
*   **ONNX Runtime** (C++ API)
*   **cv_bridge**

## 话题订阅

本模块订阅以下话题（话题名称可通过参数配置）：

| 默认话题名 | 类型 | 描述 |
| :--- | :--- | :--- |
| `/camera/color/image_raw` | `sensor_msgs/msg/Image` | RGB 彩色图像流 |
| `/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/msg/Image` | 与 RGB 对齐的深度图像流 |
| `/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | 相机内参信息 |

## 参数配置

### 节点参数

| 参数名 | 类型 | 默认值 | 描述 |
| :--- | :--- | :--- | :--- |
| `vision_color_topic` | string | `/camera/color/image_raw` | RGB 图像话题名 |
| `vision_depth_topic` | string | `/camera/aligned_depth_to_color/image_raw` | 深度图像话题名 |
| `vision_info_topic` | string | `/camera/color/camera_info` | 相机信息话题名 |
| `vision_depth_roi` | int | 7 | 深度采样区域大小 (像素)，以目标中心为原点的正方形边长 |
| `vision_depth_min_valid_count` | int | 10 | ROI 区域内计算深度所需的最小有效像素数量 |
| `vision_depth_min_m` | double | 0.2 | 最小有效距离 (米) |
| `vision_depth_max_m` | double | 5.0 | 最大有效距离 (米) |

### 运行时配置 (通过 C++ API)

*   **model_path**: ONNX 模型文件的绝对路径。
*   **class_names**: 模型输出类别对应的名称列表 (如 `["R_R1", "B_R1", ...]`)。
*   **conf_thresh**: 置信度阈值 (0.0 - 1.0)。

## C++ API 使用示例

`rc26_vision` 主要通过 `VisionInferenceManager` 类对外提供服务。

```cpp
#include "rc26_vision/vision_inference_manager.hpp"

// 1. 在你的 Node 中初始化 Manager
auto manager_ = std::make_shared<rc26_vision::VisionInferenceManager>(*this);

// 2. 准备类别名称列表 (需与模型训练时的类别一致)
std::vector<std::string> class_names = {
    "R_R1", "B_R1", "T_03", "T_04", // ... 更多类别
};

// 3. 配置模型路径和阈值
std::string model_path = "/path/to/your/model.onnx";
if (!manager_->configure(model_path, class_names, 0.5f)) {
    // 处理配置失败的情况
}

// 4. 设置结果回调函数
manager_->setResultCallback([](const rc26_vision::TargetResult& result) {
    if (result.has_target) {
        // 处理检测结果
        // result.attr_kind: 目标属性 (枚举)
        // result.distance_m: 目标距离
        // result.bbox_cx, result.bbox_cy: 目标中心像素坐标
    }
});

// 5. 启动推理
manager_->start();

// ... 在析构时停止
manager_->stop();
```

## 测试节点

本功能包提供了一个测试节点 `vision_test_node`，用于验证模型和相机输入。

### 运行命令

```bash
ros2 run rc26_vision vision_test_node --ros-args -p vision_model_path:=/绝对路径/到/你的模型.onnx
```

### 常用参数

*   `vision_model_path`: (必须) ONNX 模型文件的路径。
*   `vision_conf_thresh`: (可选) 置信度阈值，默认 0.5。
*   `print_rate_ms`: (可选) 状态打印频率 (毫秒)，默认 500。

## 模型类别映射

默认的 `vision_test_node` 使用以下类别映射 (请根据实际模型进行调整):

*   **R_R1**: 红方机器人
*   **B_R1**: 蓝方机器人
*   **T_xx**: 真目标 (Truth)
*   **F_xx**: 假目标 (False)

## 注意事项

1.  请确保深度图与 RGB 图已通过硬件或软件实现了严格对齐，否则距离计算会有偏差。
2.  ONNX Runtime 需要根据目标硬件 (CPU/GPU) 正确安装依赖库。
