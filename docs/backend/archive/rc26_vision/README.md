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

## 源码入口与阅读顺序
- 先看 `launch/realsense_d455.launch.py` 和 `launch/vision_test_with_camera.launch.py`，理解相机和测试链如何拉起。
- 再看 `src/vision_inference_manager.cpp`，它是运行时调度中心。
- 然后看 `profile_loader.cpp`、`yolo_engine.cpp`、`aidlite_engine.cpp`/`aidlite_engine_stub.cpp`、`tip_localizer.cpp`。
- 最后看 `config/vision_models.yaml` 和 `config/realsense_d455.yaml`。

## 目录解剖
- `vision_inference_manager.cpp`：配置载入、模型切换、推理线程和图像/深度/相机信息缓存。
- `profile_loader.cpp`：模型 profile 解析。
- `yolo_engine.cpp`：推理引擎封装。
- `aidlite_engine.cpp` / `aidlite_engine_stub.cpp`：真实后端与缺依赖时的 stub。
- `tip_localizer.cpp`：深度 + TF 融合，把识别结果投到 `map`。
- `src/scripts/vision_test_node.cpp`、`yolo_inference_test.cpp`：测试入口。

## 关键文件体量
- `src/vision_inference_manager.cpp`：438 行。
- `src/aidlite_engine.cpp`：315 行。
- `src/tip_localizer.cpp`：279 行。
- `src/profile_loader.cpp`：204 行。

## 关键源码行段速览
- `src/rc26_vision/src/vision_inference_manager.cpp:28-182`：构造、配置读取、模型选择与切换。
- `src/rc26_vision/src/vision_inference_manager.cpp:183-259`：运行状态查询和 result callback 接口。
- `src/rc26_vision/src/vision_inference_manager.cpp:260-389`：推理线程和图像/深度/相机信息回调。
- `src/rc26_vision/src/tip_localizer.cpp:18-94`：节点构造和数据缓存。
- `src/rc26_vision/src/tip_localizer.cpp:95-189`：深度取中值、投影到 map、间距校验。
- `src/rc26_vision/src/tip_localizer.cpp:190-273`：彩色图回调与 `main()`。

## 模块边界

- 它只负责视觉推理和目标定位，不负责比赛级流程决策
- 它不是前端显示层，也不提供 Web 可视化
- AidLite 后端依赖系统环境，缺少相关库时当前代码会退到 stub 报错路径

## 配置注释口径

- `config/vision_models.yaml` 与 `config/realsense_d455.yaml` 已保留常用/高影响参数的中文注释，说明模型 profile、推理后端、相机流、深度对齐和 RealSense 设备参数；本次只改变注释，不改变视觉运行配置。
