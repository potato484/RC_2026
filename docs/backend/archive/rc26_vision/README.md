# rc26_vision

## 模块定位

`rc26_vision` 是 R2 当前的视觉推理与端头定位包，负责模型装载、图像推理、深度融合和目标三维定位。

## 当前实现

- 构建产物：
  - 共享库 `rc26_vision`
  - `vision_test_node`
  - `yolo_inference_test`
  - `tip_localizer_node`
  - `tip_vision_test_node`
- 配置文件：
  - `config/realsense_d455.yaml`
  - `config/vision_models.yaml`
  - `config/tip_vision_params.yaml`
- 启动文件：
  - `launch/realsense_d455.launch.py`
  - `launch/vision_test_with_camera.launch.py`
  - `launch/test_tip_vision.launch.py`

当前实现已经按职责重排为 5 个子层：

- `include/rc26_vision/runtime`、`src/runtime`
  - `types`、`model_profile`、`profile_loader`、`inference_backend_resolver`、`vision_inference_manager`
- `include/rc26_vision/engines`、`src/engines`
  - `InferenceEngine`、`YoloEngine`、`AidLiteEngine`、`本地 ONNX Runtime backend`、`YOLO backend utils`、`AidLite stub`
- `include/rc26_vision/pipelines`、`src/pipelines`
  - `TipLocalizer` 的 RGB-D + TF 定位流水线
- `src/nodes`
  - `vision_test_node` 和 `tip_localizer_node` 的薄入口
- `test`
  - `tip_vision_test_node` 的单文件 test 节点实现，包含 main 入口、私有类声明、USB 相机、串口、目标选择和 overlay 逻辑
- `src/tools`、`tools`
  - 离线 C++ 工具、模型转换脚本和数据统计脚本

旧的顶层兼容头文件已经删除；仓库内调用方统一使用 `include/rc26_vision/runtime`、`include/rc26_vision/engines` 和 `include/rc26_vision/pipelines` 下的运行时公开 include 路径。`tip_vision_test_node` 的私有声明已经合并进包根 `test/tip_vision_test_node.cpp`，不再单独保留头文件，也不随公开 include 安装。`src/` 下不再保留头文件。

## 源码入口与阅读顺序
- 先看 `launch/realsense_d455.launch.py` 和 `launch/vision_test_with_camera.launch.py`，理解相机和测试链如何拉起。
- 再看 `src/runtime/vision_inference_manager.cpp`，它是运行时调度中心。
- 然后看 `src/runtime/profile_loader.cpp`、`src/runtime/inference_backend_resolver.cpp`、`src/runtime/inference_engine_factory.cpp`、`src/engines/aidlite_engine.cpp`/`src/engines/opencv_onnx_engine.cpp`/`src/engines/aidlite_engine_stub.cpp`、`src/pipelines/tip_localizer.cpp`。
- 最后看 `test/tip_vision_test_node.cpp`、`config/vision_models.yaml`、`config/tip_vision_params.yaml`、`launch/test_tip_vision.launch.py` 和 `config/realsense_d455.yaml`。

## 目录解剖
- `src/runtime/vision_inference_manager.cpp`：配置载入、模型切换、推理线程和图像/深度/相机信息缓存。
- `src/runtime/profile_loader.cpp`：模型 profile 解析。
- `src/runtime/inference_backend_resolver.cpp` / `src/runtime/inference_engine_factory.cpp`：启动时的后端探测、自动选链、中文日志与引擎创建逻辑。
- `src/engines/aidlite_engine.cpp` / `src/engines/opencv_onnx_engine.cpp` / `src/engines/aidlite_engine_stub.cpp`：AidLite 实机链、本地 ONNX Runtime 链与缺依赖时的 stub；AidLite 与本地 ONNX 共用 `yolo_backend_utils` 的 stretch/letterbox 预处理、坐标回映和 NMS。
- `src/pipelines/tip_localizer.cpp`：深度 + TF 融合，把识别结果投到 `map`。
- `src/nodes/vision_test_node.cpp`、`src/nodes/tip_localizer_node.cpp`：主链节点入口。
- `test/tip_vision_test_node.cpp`：USB 相机 + 单目 tip test 节点的入口、私有声明、参数、串口、相机、目标选择和 overlay 单文件实现；推理直接复用主链 `InferenceEngine`。
- `src/tools/yolo_inference_test.cpp`、`tools/*.py`：离线工具和实验脚本。

## 默认链与 test 链

- 当前模型 profile 支持 `engine: auto`；启动时如果检测到 `/usr/local` 下 AidLux/AidLite 头文件与库，并且当前 `rc26_vision` 构建已启用 AidLite，就优先走 `AidLiteEngine`。
- 如果当前系统不是 AidLux 环境，或者路径存在但这份二进制没有编进 AidLite，运行时会自动回退到本地 `ONNX Runtime` 推理链，不再走旧的 AidLite stub 报错退出。
- 自动选链和回退决策统一收口在共享工厂层；`tip_vision_test_node`、`vision_test_node`、`tip_localizer_node` 和 `yolo_inference_test` 会在启动时打印中文日志，说明 profile、AidLux 路径探测结果、AidLite 编译状态与最终选中的后端。
- `config/vision_models.yaml` 是唯一模型 profile 配置入口，当前包含 `kfs_default` 与 `tip_test`。
- `config/vision_models.yaml` 当前默认 profile 已切到 `engine: auto`；显式写 `onnxruntime` / `opencv_onnx` 仍会落到本地 ONNX Runtime 链，显式写 `aidlite` 则保持强制 AidLite、不参与自动回退。
- `tip` test 链已经并入 `rc26_vision`，当前入口是 `tip_vision_test_node` 与 `launch/test_tip_vision.launch.py`。
- 默认 KFS 模型资产命名为 `models/kfs.pt` / `models/kfs.onnx`，标签文件命名为 `models/kfs_labels.txt`；tip test 模型资产命名为 `models/tip.pt` / `models/tip.onnx`，标签文件命名为 `models/tip_labels.txt`。
- `config/tip_vision_params.yaml` 只保留 USB 相机、串口、窗口和目标选择等节点业务参数；模型路径、输入输出名、量化和后处理参数统一写在 `config/vision_models.yaml` 的 `tip_test` profile。
- `tip_vision_test_node` 现在通过 `vision_config_file + model_id` 选择主链模型 profile，不再自己维护 AidLite interpreter、输入 tensor buffer 或私有 YOLO 后处理。
- `AidLiteEngine` 根据输入 tensor shape 自动区分 `NCHW / NHWC`，对 `float32 ONNX` 按真实布局喂输入，不再把 `NCHW` 模型误喂成 HWC 平铺。
- 本地 ONNX Runtime 链同样会读取模型真实 input/output tensor shape，并复用与 AidLite 相同的 YOLO 预处理、坐标回映和 NMS 逻辑，避免两条链的框语义继续漂移。
- `tip_vision_test_node` 已移除旧的距离估计和距离文字叠加；`show_center_distance` 与旧距离参数名只为兼容旧配置而保留，不再参与运行时判定。
- 当前犀牛派 X1 板上实测 `models/tip.onnx` 为固定 `640x640` 的 CPU ONNX 链，`infer_ms` 大约 `80~95ms`、`infer_fps` 大约 `10~12`；这套 AidLite ONNX 后端对该模型不支持 `GPU/DSP`，若要逼近 `30 infer_fps`，需要换更小输入的 ONNX，或改用可落到 QNN/AMF 的量化资产。
- `tip_vision_test_node` 的串口发送已不再自己维护 `termios` 裸写；当前改为复用 `rc26_serial::SerialDriver`，并通过 `TIP_VISION(0x12)` 发送 5 字节业务 payload：`grab_ready | dir_code | amp_code | ts16_lo | ts16_hi`。
- 当前 `tip` test 参数默认串口已对齐仓库目标串口口径为 `/dev/ttyUSB1`，串口格式固定沿用 `rc26_serial` 的 `8N1`。
- 当前 `tip` test 参数仍默认优先 `camera_index=2` 这路外接 USB 摄像头，但 `auto_scan_camera` 已默认打开；如果首选设备能枚举却读不出第一帧，节点会打印中文告警并自动扫描其他 `/dev/video*` 作为兜底。
- 默认联调入口仍然是 RealSense + `vision_test_node`；tip test 节点不参与默认 launch，需要单独显式启动。

## 模块边界

- 它只负责视觉推理和目标定位，不负责比赛级流程决策
- 它不是前端显示层，也不提供 Web 可视化
- AidLite 后端依赖系统环境；当前只有显式要求 `aidlite` 且构建未启用 AidLite 时才会直接报错，`engine: auto` 会优先回退到本地 ONNX Runtime
- `tip` test 链继续留在包内，但与默认 RealSense 主链隔离；它面向 USB 相机/单目 test，不是当前决策运行时权威入口
- `tip` 当前虽然已经复用仓库 `rc26_serial` 协议栈，但仍是独立直连串口的 test 链；如果同一时刻 `rc26_merge_odom` 已持有同一个目标串口，不应再并发启动这个 test 节点

## 本轮收口

- 目录按 `runtime / engines / pipelines / nodes / tools` 重排，便于继续拆子模块而不破坏对外接口。
- 删除了默认 build 对不存在源码的依赖，`MAKEFLAGS='-j2 -l2' colcon build --executor sequential --parallel-workers 1 --packages-select rc26_vision` 已可通过。
- 旧顶层兼容头文件已删除，`rc26_decision` 已改为直接 include `rc26_vision/runtime/*` 分层头文件。
- 原根目录旧端头视觉 test 包已按 tip test 子链并入 `src/rc26_vision`；节点源码纳入包根 `test/` 单文件实现，参数和模型资产当前统一收口到包内 `config/` 与 `models/` 根目录。
- tip 默认模型已改为 `models/tip.onnx`，不再指向旧的缺失 `.amf` 文件名。
- 修正了 tip test 节点的模型框架识别逻辑：当模型路径显式是 `.onnx` 时，不再依赖文件名是否带 `fp32` 来决定 `ONNX / QNN231`。
- tip test 节点已删除距离估计和距离 overlay；异步推理线程也去掉了一次多余的 pending frame `clone()`，仅保留提交队列时的必要复制，稍微降低了显示侧额外开销。
- tip 启动时现在会以模型真实 input tensor shape 为准校正 `input_width / input_height`；如果参数误配成与 ONNX 不一致的尺寸，会给出告警并自动回到模型尺寸，避免固定 `640x640` 模型因误改参数直接跑崩。
- tip 串口链已切到仓库 `rc26_serial`，不再直接发送 `AA FF ... FE` 短帧；当前对 MCU 暴露的是 `TIP_VISION(0x12)` + 5 字节 payload 的 RC26 封帧。
- tip test 节点当前按测试链口径保留为 `test/tip_vision_test_node.cpp` 单文件实现，便于联调时直接查看参数、串口、相机、推理和 overlay 全链路而不改变外部行为。
- 本次进一步把 tip 推理收口到主链 `InferenceEngine / AidLiteEngine`：`tip_vision_test_node` 只保留 USB 相机、目标选择、串口和 overlay 业务；私有后处理头文件已删除，tip test 节点私有声明合并进 `test/tip_vision_test_node.cpp`，不再放入公开 `include/`。
- 本次按其他模块口径把 tip test 链路源码收口到包根 `test/`，不再放入 `src/` 或公开 `include/`；tip test 参数和模型资产直接放在包内 `config/` 与 `models/` 根目录，避免测试链继续维护额外嵌套目录。
- 模型 profile 已统一到 `config/vision_models.yaml`，`config/test/vision_models_tip.yaml` 删除；模型文件按用途重命名为 `kfs.pt/onnx` 与 `tip.pt/onnx`。
- 本次把 `engine: auto`、启动中文日志和共享后端解析收口到 `inference_backend_resolver + inference_engine_factory`；非 AidLux 系统启动时会自动切到本地 ONNX Runtime，而不是继续撞 AidLite stub。
- 本次新增本地 ONNX Runtime backend，并把 AidLite / 本地 ONNX 共用的 YOLO 预处理、坐标回映和 NMS 抽到 `yolo_backend_utils`；`tip_vision_test_node`、`vision_test_node`、`tip_localizer_node` 与 `yolo_inference_test` 统一走这套共享入口。
- 本次补强了 tip 相机初始化日志，并把默认 tip 参数改成“优先外接摄像头、失败时自动扫描回退”；像 `/dev/video2` 这种能枚举但首帧超时的 UVC 设备，不会再让节点静默卡死在无窗口状态。
- 本次进一步把 tip test 资源目录扁平化：旧的嵌套 test 参数/模型目录已提升并收口到包内 `config/` 与 `models/` 根目录，同时把标签文件显式命名为 `kfs_labels.txt` 与 `tip_labels.txt`，减少同名资源和相对路径歧义。
