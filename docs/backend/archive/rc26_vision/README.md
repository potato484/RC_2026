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

当前实现已经按“共享 / 预处理 / 推理 / 后处理 / 入口”重排为 7 个子层，并在这四个库层下继续细分二级功能目录：

- `include/rc26_vision/shared`、`src/shared`
  - `contracts/vision_types`
  - `sensors/depth_roi_sampler`
  - `transforms/yolo_transform`
- `include/rc26_vision/preprocess`、`src/preprocess`
  - `yolo/yolo_image_preprocessor`
- `include/rc26_vision/inference`、`src/inference`
  - `contracts/inference_engine`
  - `config/model_profile`、`config/model_profile_loader`
  - `runtime/backend_resolver`、`runtime/engine_factory`、`runtime/vision_inference_manager`
  - `yolo/yolo_engine`
  - `aidlite/aidlite_engine`、`aidlite/aidlite_engine_stub`
  - `onnx/onnx_runtime_engine`、`onnx/onnx_runtime_engine_stub`
- `include/rc26_vision/postprocess`、`src/postprocess`
  - `yolo/yolo_detection_postprocessor`
  - `localization/tip_localizer`
- `src/nodes`
  - `vision_test_node` 和 `tip_localizer_node` 的薄入口
- `test`
  - `tip_vision_test_node` 的单文件 test 节点实现，包含 main 入口、私有类声明、USB 相机、串口、目标选择和 overlay 逻辑
- `src/tools`、`tools`
  - 离线 C++ 工具、模型转换脚本和数据统计脚本

旧的 `runtime / engines / pipelines` 公开 include 路径已经删除；仓库内调用方统一切到二级语义更明确的公开路径，例如 `include/rc26_vision/shared/contracts`、`include/rc26_vision/inference/runtime`、`include/rc26_vision/inference/aidlite`、`include/rc26_vision/postprocess/localization`。`tip_vision_test_node` 的私有声明已经合并进包根 `test/tip_vision_test_node.cpp`，不再单独保留头文件，也不随公开 include 安装。`src/` 下不再保留头文件。

## 源码入口与阅读顺序
- 先看 `launch/realsense_d455.launch.py` 和 `launch/vision_test_with_camera.launch.py`，理解相机和测试链如何拉起。
- 再看 `src/inference/runtime/vision_inference_manager.cpp`，它是运行时调度中心。
- 然后看 `src/inference/config/model_profile_loader.cpp`、`src/inference/runtime/backend_resolver.cpp`、`src/inference/runtime/engine_factory.cpp`、`src/inference/aidlite/aidlite_engine.cpp`/`src/inference/aidlite/aidlite_engine_stub.cpp`、`src/inference/onnx/onnx_runtime_engine.cpp`/`src/inference/onnx/onnx_runtime_engine_stub.cpp`、`src/preprocess/yolo/yolo_image_preprocessor.cpp`、`src/postprocess/yolo/yolo_detection_postprocessor.cpp`、`src/shared/sensors/depth_roi_sampler.cpp`、`src/postprocess/localization/tip_localizer.cpp`。
- 最后看 `test/tip_vision_test_node.cpp`、`config/vision_models.yaml`、`config/tip_vision_params.yaml`、`launch/test_tip_vision.launch.py` 和 `config/realsense_d455.yaml`。

## 目录解剖
- `src/shared/sensors/depth_roi_sampler.cpp`：共享深度 ROI 中值采样，供默认推理链和 tip localizer 共用。
- `src/preprocess/yolo/yolo_image_preprocessor.cpp`：YOLO 输入图像预处理与 letterbox/stretch 变换信息。
- `src/inference/runtime/vision_inference_manager.cpp`：配置载入、模型切换、推理线程和图像/深度/相机信息缓存。
- `src/inference/config/model_profile_loader.cpp`：模型 profile 解析。
- `src/inference/runtime/backend_resolver.cpp` / `src/inference/runtime/engine_factory.cpp`：启动时的后端探测、自动选链、中文日志与引擎创建逻辑。
- `src/inference/aidlite/aidlite_engine.cpp` / `src/inference/aidlite/aidlite_engine_stub.cpp` / `src/inference/onnx/onnx_runtime_engine.cpp` / `src/inference/onnx/onnx_runtime_engine_stub.cpp`：AidLite 实机链、本地 ONNX Runtime 链与缺依赖时的 stub。
- `src/postprocess/yolo/yolo_detection_postprocessor.cpp`：YOLO 输出解码、坐标回映和 NMS。
- `src/postprocess/localization/tip_localizer.cpp`：深度 + TF 融合，把识别结果投到 `map`。
- `src/nodes/vision_test_node.cpp`、`src/nodes/tip_localizer_node.cpp`：主链节点入口。
- `test/tip_vision_test_node.cpp`：USB 相机 + 单目 tip test 节点的入口、私有声明、参数、串口、相机、目标选择和 overlay 单文件实现；推理直接复用主链 `InferenceEngine`。
- `src/tools/yolo_inference_test.cpp`、`tools/*.py`：离线工具和实验脚本。

## 默认链与 test 链

- 当前模型 profile 支持 `engine: auto`；启动时如果检测到 `/usr/local` 下 AidLux/AidLite 头文件与库，并且当前 `rc26_vision` 构建已启用 AidLite，就优先走 `AidLiteEngine`；否则如果当前二进制已编进 ONNX Runtime C++ 后端，就回退到本地 `ONNX Runtime` 推理链。
- AidLite 和本地 ONNX Runtime 都是可选编译后端。缺 ONNX Runtime C++ 头文件或库时，构建不会失败，而是编译 `onnx_runtime_engine_stub`；缺 AidLite 时同样编译 `aidlite_engine_stub`。stub 只提供链接符号，实际启动到对应推理后端时会快速报错，不产生假推理结果。
- 自动选链和回退决策统一收口在共享工厂层；`tip_vision_test_node`、`vision_test_node`、`tip_localizer_node` 和 `yolo_inference_test` 会在启动时打印中文日志，说明 profile、AidLux 路径探测结果、AidLite 编译状态、ONNX Runtime 编译状态与最终选中的后端。
- 当前兼容矩阵：AidLite 有且 ONNX Runtime C++ 缺失时可构建并由 `engine: auto` 选择 AidLite；AidLite 缺失且 ONNX Runtime C++ 存在时可构建并回退 ONNX Runtime；两者都有时优先 AidLite；两者都没有时仍允许构建，但启动推理会报“无可用推理后端”。
- `config/vision_models.yaml` 是唯一模型 profile 配置入口，当前包含 `kfs_default` 与 `tip_test`。
- `config/vision_models.yaml` 当前默认 profile 已切到 `engine: auto`；显式写 `onnxruntime` / `opencv_onnx` 仍会落到本地 ONNX Runtime 链，显式写 `aidlite` 则保持强制 AidLite、不参与自动回退。
- 默认视觉主链当前通过 `VisionInferenceManager` 使用 `0.6m ~ 1.2m` 的深度 ROI 有效距离窗口；落在窗口外或有效深度样本不足的检测不会被上游决策当作 `has_target=true`。
- `tip` test 链已经并入 `rc26_vision`，当前入口是 `tip_vision_test_node` 与 `launch/test_tip_vision.launch.py`。
- 默认 KFS 模型资产命名为 `models/kfs.pt` / `models/kfs.onnx`，标签文件命名为 `models/kfs_labels.txt`；tip test 模型资产命名为 `models/tip.pt` / `models/tip.onnx`，标签文件命名为 `models/tip_labels.txt`。
- `config/tip_vision_params.yaml` 只保留 USB 相机、串口、窗口和目标选择等节点业务参数；模型路径和后处理参数统一写在 `config/vision_models.yaml` 的 `tip_test` profile。
- `tip_vision_test_node` 现在通过 `vision_config_file + model_id` 选择主链模型 profile，不再自己维护 AidLite interpreter、输入 tensor buffer 或私有 YOLO 后处理。
- `AidLiteEngine` 根据输入 tensor shape 自动区分 `NCHW / NHWC`，对 `float32 ONNX` 按真实布局喂输入，不再把 `NCHW` 模型误喂成 HWC 平铺。
- 本地 ONNX Runtime 链在编译启用时同样会读取模型真实 input/output tensor shape，并复用与 AidLite 相同的 YOLO 预处理、坐标回映和 NMS 逻辑，避免两条链的框语义继续漂移。
- 当前 `models/kfs.onnx` 与 `models/tip.onnx` 都按 float ONNX 默认口径运行；`tip_test` profile 已移除旧的显式 input/output 名和量化参数，输入输出 tensor 名改由运行时自动探测。
- `tip_test` 当前仍刻意保留 `resize_mode: letterbox` 与 `num_classes: 1`；前者直接影响 tip test 的预处理和框坐标回映，不属于可与 `kfs_default` 一起删除的冗余配置。
- `tip_vision_test_node` 已移除旧的距离估计和距离文字叠加；`show_center_distance` 与旧距离参数名只为兼容旧配置而保留，不再参与运行时判定。
- 当前犀牛派 X1 板上实测 `models/tip.onnx` 为固定 `640x640` 的 CPU ONNX 链，`infer_ms` 大约 `80~95ms`、`infer_fps` 大约 `10~12`；这套 AidLite ONNX 后端对该模型不支持 `GPU/DSP`，若要逼近 `30 infer_fps`，需要换更小输入的 ONNX，或改用可落到 QNN/AMF 的量化资产。
- `tip_vision_test_node` 的串口发送已不再自己维护 `termios` 裸写；当前改为复用 `rc26_serial::SerialDriver`，并通过 `TIP_VISION(0x12)` 发送 5 字节业务 payload：`grab_ready | dir_code | amp_code | ts16_lo | ts16_hi`。
- 当前 `tip` test 参数默认串口已对齐仓库目标串口口径为 `/dev/ttyUSB1`，串口格式固定沿用 `rc26_serial` 的 `8N1`。
- 当前 `tip` test 参数仍默认优先 `camera_index=2` 这路外接 USB 摄像头，但 `auto_scan_camera` 已默认打开；如果首选设备能枚举却读不出第一帧，节点会打印中文告警并自动扫描其他 `/dev/video*` 作为兜底。
- 默认联调入口仍然是 RealSense + `vision_test_node`；tip test 节点不参与默认 launch，需要单独显式启动。

## 模块边界

- 它只负责视觉推理和目标定位，不负责比赛级流程决策
- 它不是显示层，也不提供 Web 可视化
- AidLite 和 ONNX Runtime 后端都依赖部署环境；显式指定未编译进当前二进制的后端会直接报错，`engine: auto` 只在对应后端已编译启用时自动选择或回退
- `tip` test 链继续留在包内，但与默认 RealSense 主链隔离；它面向 USB 相机/单目 test，不是当前决策运行时权威入口
- `tip` 当前虽然已经复用仓库 `rc26_serial` 协议栈，但仍是独立直连串口的 test 链；如果同一时刻 `rc26_merge_odom` 已持有同一个目标串口，不应再并发启动这个 test 节点

## 本轮收口

- 本次把 ONNX Runtime C++ 改为可选编译后端：缺 `onnxruntime_cxx_api.h` 或 `libonnxruntime.so` 时不再阻塞 `rc26_vision` 构建，而是编译本地 ONNX stub；当前 AidLux 板卡可在 AidLite 已安装、ONNX Runtime C++ 开发头缺失的状态下构建并通过 `engine: auto` 选择 AidLite。
- 本次补充了后端能力日志和 resolver 测试，`engine: auto` 的顺序固定为 AidLite 优先、ONNX Runtime 次之、两者都不可用时明确报错。
- 目录已进一步按 `shared / preprocess / inference / postprocess / nodes / tools` 重排，并继续在四个库层下细分二级功能目录；公开 include 也同步硬切到二级子目录，不再保留旧的一层路径。
- 删除了默认 build 对不存在源码的依赖，`MAKEFLAGS='-j2 -l2' colcon build --executor sequential --parallel-workers 1 --packages-select rc26_vision` 已可通过。
- `rc26_decision` 已同步切到新的 `rc26_vision/shared/contracts/*` 与 `rc26_vision/inference/runtime/*`、`rc26_vision/inference/config/*` 公开 include 路径。
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
- 本次把 `engine: auto`、启动中文日志和共享后端解析收口到 `backend_resolver + engine_factory`；非 AidLux 系统启动时会自动切到本地 ONNX Runtime，而不是继续撞 AidLite stub。
- 本次把 AidLite / 本地 ONNX 共用的 YOLO 交集继续拆到明确阶段：`preprocess/yolo/yolo_image_preprocessor` 负责输入预处理，`postprocess/yolo/yolo_detection_postprocessor` 负责解码与 NMS，`shared/transforms/yolo_transform` 负责跨阶段变换元数据；`tip_vision_test_node`、`vision_test_node`、`tip_localizer_node` 与 `yolo_inference_test` 统一走这套共享入口。
- 本次新增 `shared/sensors/depth_roi_sampler`，把 `vision_inference_manager` 和 `tip_localizer` 里原本各自维护的深度 ROI 中值采样逻辑收口到共享层，避免两条链继续各改各的。
- 本次进一步把 `inference` 拆成 `contracts / config / runtime / yolo / aidlite / onnx` 六组，减少“一层目录内同时混放接口、配置、运行时和后端实现”的平铺耦合。
- 本次补强了 tip 相机初始化日志，并把默认 tip 参数改成“优先外接摄像头、失败时自动扫描回退”；像 `/dev/video2` 这种能枚举但首帧超时的 UVC 设备，不会再让节点静默卡死在无窗口状态。
- 本次进一步把 tip test 资源目录扁平化：旧的嵌套 test 参数/模型目录已提升并收口到包内 `config/` 与 `models/` 根目录，同时把标签文件显式命名为 `kfs_labels.txt` 与 `tip_labels.txt`，减少同名资源和相对路径歧义。
- 本次进一步把 `tip_test` 的 AidLite profile 收口到 float ONNX 默认口径，删除了旧的显式 tensor 名和量化/反量化参数；当前 tip 仍保留 `letterbox` 作为与默认 `kfs_default` 不同的预处理行为。
