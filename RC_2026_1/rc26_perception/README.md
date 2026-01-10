# rc26_perception

RC26 感知模块：D455 深度相机 + YOLO 目标检测

## 模块架构

```
rc26_perception/
├── include/rc26_perception/
│   ├── yolo_engine.hpp        # YOLO 推理引擎 (AidLite)
│   ├── preprocess.hpp         # 图像预处理
│   └── postprocess.hpp        # 检测后处理
├── src/
│   ├── yolo_engine.cpp
│   ├── preprocess.cpp
│   ├── postprocess.cpp
│   └── perception_node.cpp    # ROS2 感知节点
├── scripts/
│   ├── visualization_node.py  # 可视化调试节点 (ONNX Runtime)
│   └── export_onnx.py         # 模型导出工具
├── msg/
│   ├── BlockDetection.msg     # 单个检测结果
│   └── BlockDetections.msg    # 多目标检测结果
├── launch/
│   ├── perception.launch.py   # 生产模式 (仅发布话题)
│   └── visualization.launch.py # 调试模式 (可视化界面)
└── config/
    └── perception.yaml
```

## 话题

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/camera/color/image_raw` | sensor_msgs/Image | 订阅 | 彩色图像 |
| `/camera/aligned_depth_to_color/image_raw` | sensor_msgs/Image | 订阅 | 对齐深度图 |
| `/rc26/block_detections` | rc26_perception/BlockDetections | 发布 | 检测结果 |

## 消息格式

### BlockDetection.msg

```
string class_name           # 类别名
float32 confidence          # 置信度
int32 pixel_x, pixel_y      # 像素坐标
int32 box_x, box_y          # 边界框位置
int32 box_width, box_height # 边界框尺寸
float32 distance_m          # 深度距离
float32 camera_x/y/z        # 相机坐标系位置
```

### 自定义类别 (训练后启用)

| ID | 类别名 | 说明 |
|----|--------|------|
| 0 | R1 | 手动机器人KFS (比赛LOGO) |
| 1-15 | T_03~T_17 | 自动机器人KFS (甲骨文字，共15种) |
| 16-30 | F_18~F_32 | 假KFS (小篆体文字，共15种) |

**类别分类规则**:
- `R1`: 手动机器人专用KFS
- `T_` 前缀: 自动机器人KFS (T_03, T_04, ..., T_17)
- `F_` 前缀: 假KFS (F_18, F_19, ..., F_32)

## 启动模式

### 1. 生产模式 (perception.launch.py)

**用途**: 正式运行，仅发布检测结果到 `/rc26/block_detections`，供决策/黑板调用

```bash
# 基本启动 (需要外部相机驱动)
ros2 launch rc26_perception perception.launch.py enable_driver:=false

# 启动相机驱动 + 感知
ros2 launch rc26_perception perception.launch.py \
    model_path:=/home/potato/RC_2026/RC_2026_1/rc26_perception/models/best.onnx

# 自定义类别模式 (AidLux 平台)
ros2 launch rc26_perception perception.launch.py \
    model_path:=/path/to/model.bin \
    use_custom_classes:=true \
    num_classes:=31
```

### 2. 可视化调试模式 (visualization.launch.py)

**用途**: 验证模型识别效果，带有可缩放的可视化界面

```bash
# 仅显示相机画面 (无推理)
ros2 launch rc26_perception visualization.launch.py

# 使用 ONNX 模型验证识别
ros2 launch rc26_perception visualization.launch.py \
    model_path:=/path/to/yolov8s.onnx

# 自定义模型 + 调整阈值
ros2 launch rc26_perception visualization.launch.py \
    model_path:=/home/potato/RC_2026/RC_2026_1/rc26_perception/models/best.onnx \
    conf_thres:=0.5 \
    num_classes:=31

# 调整窗口初始大小
ros2 launch rc26_perception visualization.launch.py \
    model_path:=/path/to/model.onnx \
    window_width:=1280 \
    window_height:=960
```

**可视化界面功能**:
- **窗口可缩放**: 拖拽窗口边缘自由调整大小
- **信息面板**: 显示 FPS、推理时间、分辨率、模型信息、阈值、深度状态
- **检测统计**: 当前检测数量、平均检测数、各类别统计
- **检测框信息**: 类别、置信度、深度距离、边界框尺寸、中心点坐标
- **深度图窗口**: 独立的深度可视化窗口（可选）
- **快捷键**: Q / ESC 退出

### 3. 整合启动

```bash
# 通过 bringup 启动（需要 D455 相机）
ros2 launch rc26_bringup bringup.launch.py \
    use_perception:=true \
    model_path:=/path/to/model.bin
```

## 可视化参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `window_width` | 960 | 初始窗口宽度 |
| `window_height` | 720 | 初始窗口高度 |
| `show_info_panel` | true | 显示信息面板 |
| `show_depth` | true | 检测标签显示深度 |
| `show_depth_window` | true | 显示深度图窗口 |
| `depth_colormap` | 2 | 深度色图 (0=灰度, 2=JET, 4=RAINBOW, 11=TURBO) |
| `depth_vis_min` | 0.1 | 深度可视化最小值 (米) |
| `depth_vis_max` | 5.0 | 深度可视化最大值 (米) |

## 编译

### 普通平台 (调试用)

```bash
cd ~/RC_2026/RC_2026_1
colcon build --packages-select rc26_perception
```

### AidLux 平台 (完整推理)

```bash
colcon build --packages-select rc26_perception \
    --cmake-args -DAIDLUX_PLATFORM=ON
```

## 与决策系统对接

`rc26_decision` 中的 `DetectionInterface` 已自动订阅 `/rc26/block_detections` 话题。

使用示例:

```cpp
// 在行为树节点中
auto result = detection_interface_->waitForDetection(5.0);
if (result && result->detected) {
    for (const auto& det : result->detections) {
        // T_开头为自动机器人KFS (T_03~T_17)
        if (det.class_name.size() >= 2 && det.class_name.substr(0, 2) == "T_") {
            // 处理自动机器人KFS
        }
        // R1为手动机器人KFS
        else if (det.class_name == "R1") {
            // 处理手动机器人KFS
        }
        // F_开头为假KFS (F_18~F_32)
        else if (det.class_name.size() >= 2 && det.class_name.substr(0, 2) == "F_") {
            // 处理假KFS
        }
    }
}
```

## 依赖

- ROS2 Humble/Iron
- OpenCV 4.x
- cv_bridge
- message_filters
- tf2_ros
- ONNX Runtime (可视化节点)
- AidLite SDK (仅 AidLux 平台生产节点)

## 训练自定义模型

1. 采集比赛方块图像
2. 标注 31 个类别 (R1, T_03~T_17, F_18~F_32)
3. 训练 YOLOv8 模型
4. 导出 ONNX (调试用) 或 QNN (AidLux 生产用)
5. 使用可视化模式验证识别效果
6. 更新 `model_path` 参数部署生产环境
