# rc26_vision

## 模块定位

`rc26_vision` 是 R2 当前的视觉推理与弹头定位包，负责模型装载、图像推理、深度融合和目标三维定位。

## 当前实现

- 构建产物：
  - 共享库 `rc26_vision`
  - `vision_test_node`
  - `yolo_inference_test`
  - `tip_localizer_node`
- 配置文件：
  - `config/realsense_d455.yaml`
  - `config/vision_models.yaml`
- 启动文件：
  - `launch/realsense_d455.launch.py`
  - `launch/vision_test_with_camera.launch.py`

当前代码结构已经比较清晰：

- `profile_loader.cpp`
  - 读取模型和后端配置
- `vision_inference_manager.cpp`
  - 管理彩色图、深度图、相机内参订阅
  - 维护推理线程
  - 支持按 `model_id` 切换模型
  - 对外提供 `start/stop/isReady/getLatestResult`
- `yolo_engine.cpp`
  - 推理引擎封装
- `aidlite_engine.cpp`
  - AidLite 后端实现
- `aidlite_engine_stub.cpp`
  - 当系统没有 AidLite 依赖时抛出不可用异常
- `tip_localizer.cpp`
  - 结合深度图和 TF，把检测结果转换成 `map` 坐标系下的 `TipDetectionArray`

从当前实现看，这个包已经同时服务两类调用方：

- `rc26_decision`
  - 通过 `VisionInferenceManager` 把识别结果写进 BT 黑板
- 其他订阅者
  - 通过 `tip_localizer_node` 直接消费三维定位后的目标检测消息

## 模块边界

- 它只负责视觉推理和目标定位，不负责比赛级流程决策
- 它不是前端显示层，也不提供 Web 可视化
- AidLite 后端依赖系统环境，缺少相关库时当前代码会退到 stub 报错路径
