# rc26_vision

## 模块定位

`rc26_vision` 是 R2 当前的视觉推理与端头定位包，负责模型装载、图像推理、深度融合和目标三维定位。

## 当前实现

- 构建产物：
  - 共享库 `rc26_vision`
  - `kfs_vision_test_node`
  - `yolo_inference_test`
  - `tip_localizer_node`
  - `tip_vision_test_node`
- 配置文件：
  - `config/realsense_d455.yaml`
  - `config/vision_models.yaml`
  - `config/kfs_vision_params.yaml`
  - `config/tip_vision_params.yaml`
- 启动文件：
  - `launch/realsense_d455.launch.py`
  - `launch/test_kfs_vision.launch.py`
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
  - `kfs_vision_test_node` 和 `tip_localizer_node` 的薄入口
- `test`
  - `tip_vision_test_node` 的单文件 test 节点实现，包含 main 入口、私有类声明、USB 相机、目标选择、可选视觉横移对线、抓取 service 触发和 overlay 逻辑
- `src/tools`、`tools`
  - 离线 C++ 工具、模型转换脚本和数据统计脚本

旧的 `runtime / engines / pipelines` 公开 include 路径已经删除；仓库内调用方统一切到二级语义更明确的公开路径，例如 `include/rc26_vision/shared/contracts`、`include/rc26_vision/inference/runtime`、`include/rc26_vision/inference/aidlite`、`include/rc26_vision/postprocess/localization`。`tip_vision_test_node` 的私有声明已经合并进包根 `test/tip_vision_test_node.cpp`，不再单独保留头文件，也不随公开 include 安装。`src/` 下不再保留头文件。

## 源码入口与阅读顺序
- 先看 `launch/realsense_d455.launch.py` 和 `launch/test_kfs_vision.launch.py`，理解相机和测试链如何拉起；当前 R2 D455 默认固定选择 RealSense 序列号 `239222303644`。
- 再看 `src/inference/runtime/vision_inference_manager.cpp`，它是运行时调度中心。
- 然后看 `src/inference/config/model_profile_loader.cpp`、`src/inference/runtime/backend_resolver.cpp`、`src/inference/runtime/engine_factory.cpp`、`src/inference/aidlite/aidlite_engine.cpp`/`src/inference/aidlite/aidlite_engine_stub.cpp`、`src/inference/onnx/onnx_runtime_engine.cpp`/`src/inference/onnx/onnx_runtime_engine_stub.cpp`、`src/preprocess/yolo/yolo_image_preprocessor.cpp`、`src/postprocess/yolo/yolo_detection_postprocessor.cpp`、`src/shared/sensors/depth_roi_sampler.cpp`、`src/postprocess/localization/tip_localizer.cpp`。
- 最后看 `test/tip_vision_test_node.cpp`、`config/vision_models.yaml`、`config/kfs_vision_params.yaml`、`config/tip_vision_params.yaml`、`launch/test_tip_vision.launch.py` 和 `config/realsense_d455.yaml`。

## 目录解剖
- `src/shared/sensors/depth_roi_sampler.cpp`：共享深度 ROI 中值采样，供默认推理链和 tip localizer 共用。
- `src/preprocess/yolo/yolo_image_preprocessor.cpp`：YOLO 输入图像预处理与 letterbox/stretch 变换信息。
- `src/inference/runtime/vision_inference_manager.cpp`：配置载入、模型切换、推理线程和图像/深度/相机信息缓存。
- `VisionInferenceManager::getLatestFrameSnapshot()`：只读深拷贝快照 API，暴露最近彩色帧、深度帧、完整 detections、目标结果和推理帧序号，供决策层 KFS 阶梯等待链判断占用目标是否仍在有效深度范围内。
- `src/inference/config/model_profile_loader.cpp`：模型 profile 解析。
- `src/inference/runtime/backend_resolver.cpp` / `src/inference/runtime/engine_factory.cpp`：启动时的后端探测、自动选链、中文日志与引擎创建逻辑。
- `src/inference/aidlite/aidlite_engine.cpp` / `src/inference/aidlite/aidlite_engine_stub.cpp` / `src/inference/onnx/onnx_runtime_engine.cpp` / `src/inference/onnx/onnx_runtime_engine_stub.cpp`：AidLite 实机链、本地 ONNX Runtime 链与缺依赖时的 stub。
- `src/postprocess/yolo/yolo_detection_postprocessor.cpp`：YOLO 输出解码、坐标回映和 NMS。
- `src/postprocess/localization/tip_localizer.cpp`：深度 + TF 融合，把识别结果投到 `map`。
- `src/nodes/kfs_vision_test_node.cpp`：kfs 主链(D455 + kfs.onnx)测试节点入口，在主线程通过 OpenCV overlay 窗口画出全部检测框、标签特征色、类别、置信度、中心 ROI 深度有效性、角落帧率与 best target 的 D455 深度距离；`model_id` 为空时使用 `vision_models.yaml` 的 `default_model`，非空时显式选择对应 profile；`show_window=false` 时降级无头。高频 `[检测]` 日志默认由 `log_detections=false` 关闭，周期 `[状态]` 日志默认由 `log_status=false` 关闭，需要排查识别或动作状态细节时可临时打开。节点还保留默认关闭的 `kfs_action_enable` 实机动作测试链，开启后只把 `T_*` 当作本车 KFS 夹取目标，先按 `direction` 发送 `ARM_RAISE(0x04)` 或 `ARM_LOWER(0x05)` 并等待同 seq 完成反馈，再发布 `/cmd_vel` 做横移对齐；横移稳定后只锁定一次有效深度。`direction=down` 会在开环前先发送 `ARM_SECOND_LOWER(0x0E)` 并等待同 seq 的 `ARM_SECOND_LOWER_DONE(0x0A)`，确认后才按 `max(0, locked_depth - kfs_action_grab_distance_m) / kfs_action_approach_speed_mps` 做 x 正向纯开环趋近；`direction=up` 保持直接开环趋近。到时停车并通过 `/mechanism/send_command` 发送 `GRAB_KFS_UP(0x03)` 或 `GRAB_KFS_DOWN(0x02)`，ACK 后用原目标视觉消失确认物理夹取成功。KFS action test 的 service ACK 等待默认按 `6000ms` 覆盖 `rc26_serial` 底层可靠发送重试闭包；机械臂预调与第二节放下完成反馈等待默认 `10s`，且从 ACK 返回并拿到 `seq` 后才开始计时。`src/nodes/tip_localizer_node.cpp`：tip 定位节点入口。
- `test/tip_vision_test_node.cpp`：USB 相机 + 单目 tip test 节点的入口、私有声明、参数、相机、目标选择、可选视觉横移对线、对齐后 x 负向前探等待 0x06 限位、抓取 service 触发和 overlay 单文件实现；推理直接复用主链 `InferenceEngine`。
- `src/tools/yolo_inference_test.cpp`、`tools/*.py`：离线工具和实验脚本。

## 默认链与 test 链

- 当前模型 profile 支持 `engine: auto`；启动时如果检测到 `/usr/local` 下 AidLux/AidLite 头文件与库，并且当前 `rc26_vision` 构建已启用 AidLite，就优先走 `AidLiteEngine`；否则如果当前二进制已编进 ONNX Runtime C++ 后端，就回退到本地 `ONNX Runtime` 推理链。
- AidLite 和本地 ONNX Runtime 都是可选编译后端。缺 ONNX Runtime C++ 头文件或库时，构建不会失败，而是编译 `onnx_runtime_engine_stub`；缺 AidLite 时同样编译 `aidlite_engine_stub`。stub 只提供链接符号，实际启动到对应推理后端时会快速报错，不产生假推理结果。
- 自动选链和回退决策统一收口在共享工厂层；`tip_vision_test_node`、`kfs_vision_test_node`、`tip_localizer_node` 和 `yolo_inference_test` 会在启动时打印中文日志，说明 profile、AidLux 路径探测结果、AidLite 编译状态、ONNX Runtime 编译状态与最终选中的后端。
- 当前兼容矩阵：AidLite 有且 ONNX Runtime C++ 缺失时可构建并由 `engine: auto` 选择 AidLite；AidLite 缺失且 ONNX Runtime C++ 存在时可构建并回退 ONNX Runtime；两者都有时优先 AidLite；两者都没有时仍允许构建，但启动推理会报“无可用推理后端”。
- `config/vision_models.yaml` 是唯一模型 profile 配置入口，当前包含 `kfs_default` 与 `tip_default`。
- `config/vision_models.yaml` 当前默认 profile 已切到 `engine: auto`；显式写 `onnxruntime` / `opencv_onnx` 仍会落到本地 ONNX Runtime 链，显式写 `aidlite` 则保持强制 AidLite、不参与自动回退。
- 默认视觉主链当前通过 `VisionInferenceManager` 使用 `vision_depth_min_m / vision_depth_max_m` 作为深度 ROI 有效距离窗口；未覆盖时库默认值为 `0.6m ~ 1.2m`。`config/kfs_vision_params.yaml` 已为独立 KFS action test 显式覆盖该窗口，当前实机调试值为 `0.50m ~ 0.55m`；正式 MF 预选赛仍读取 `r2_runtime.yaml` 中的 `mf_preselect_depth_min_m / mf_preselect_depth_max_m`。落在窗口外或有效深度样本不足的检测不会被上游决策当作 `has_target=true`。
- `tip` test 链已经并入 `rc26_vision`，当前入口是 `tip_vision_test_node` 与 `launch/test_tip_vision.launch.py`。
- 默认 KFS 模型资产命名为 `models/kfs.pt` / `models/kfs.onnx`，标签文件命名为 `models/kfs_labels.txt`；tip test 模型资产命名为 `models/tip.pt` / `models/tip.onnx`，标签文件命名为 `models/tip_labels.txt`。
- 当前 `models/kfs_labels.txt` 中的 `R_R1` / `B_R1` 表示其它机器人需要拾取的 KFS，占用本车当前阶梯格时用于决策层停车等待；它们不是本车可夹取标签。`rc26_vision` 独立 KFS action test 默认只把 `T_*` 当作本车可夹取真目标，`F_*` 和其它未知标签默认忽略。
- `config/tip_vision_params.yaml` 只保留 USB 相机、窗口、目标选择、可选对线控制、对齐后限位前探和抓取下发等节点业务参数；模型路径和后处理参数统一写在 `config/vision_models.yaml` 的 `tip_default` profile。
- `tip_vision_test_node` 现在通过 `vision_config_file + model_id` 选择主链模型 profile，不再自己维护 AidLite interpreter、输入 tensor buffer 或私有 YOLO 后处理。
- `AidLiteEngine` 根据输入 tensor shape 自动区分 `NCHW / NHWC`，对 `float32 ONNX` 按真实布局喂输入，不再把 `NCHW` 模型误喂成 HWC 平铺。
- 本地 ONNX Runtime 链在编译启用时同样会读取模型真实 input/output tensor shape，并复用与 AidLite 相同的 YOLO 预处理、坐标回映和 NMS 逻辑，避免两条链的框语义继续漂移。
- 当前 `models/kfs.onnx` 与 `models/tip.onnx` 都按 float ONNX 默认口径运行；`tip_default` profile 已移除旧的显式 input/output 名和量化参数，输入输出 tensor 名改由运行时自动探测。
- `tip_default` 当前仍刻意保留 `resize_mode: letterbox` 与 `num_classes: 1`；前者直接影响 tip test 的预处理和框坐标回映，不属于可与 `kfs_default` 一起删除的冗余配置。
- `tip_vision_test_node` 已移除旧的距离估计和距离文字叠加；`show_center_distance` 与旧距离参数名只为兼容旧配置而保留，不再参与运行时判定。
- 当前犀牛派 X1 板上实测 `models/tip.onnx` 为固定 `640x640` 的 CPU ONNX 链，`infer_ms` 大约 `80~95ms`、`infer_fps` 大约 `10~12`；这套 AidLite ONNX 后端对该模型不支持 `GPU/DSP`，若要逼近 `30 infer_fps`，需要换更小输入的 ONNX，或改用可落到 QNN/AMF 的量化资产。
- `tip_vision_test_node` 已删除旧的视觉直连串口状态下发链路，不再发送原下行 `0x12` 状态命令，也不再维护 `serial_*` 参数。
- `tip` test 链的自动横移对线以画面中心竖线作为目标线，primary target 识别框中心竖线作为检测线；多框同时出现时，节点会先按“识别框中心距离画面中心竖线最近”获取锁定目标，随后在锁定窗口内持续跟踪同一个物理端头，不再因为另一侧框短暂更近就来回切换。启用 `alignment_control_enable=true` 后，对齐阶段发布 `cmd_vel.linear.y` 执行左右横移；当前默认同时启用 `alignment_heading_hold_enable=true`，订阅 `alignment_odom_topic=odom` 并按 `alignment_target_yaw_rad` 发布 `cmd_vel.angular.z` 保持车身朝向。yaw 偏差超过 `alignment_heading_gate_deg` 时暂停横移/前探，只先转向；像素误差和 yaw 误差都进入容差后才允许稳定计数。对齐稳定后发布 `cmd_vel.linear.x<0` 前探，等待 `/mechanism/command_feedback` 中的 `FRONT_LIMIT_SWITCH_TRIGGERED(0x06)` 后立即停车并进入抓取。当前 tip test 默认按相机朝机器人后方的安装口径反转横移方向，可通过 `alignment_invert_direction` 现场一键改回；纯相机桌面调试或未启动 odom 链路时，应显式设置 `alignment_heading_hold_enable=false`。`postprocess/alignment/tip_alignment` 的目标选择结果现在保留原始 detection 下标，供 `rc26_decision` 在复用同一中心线锁定口径时回查 KFS 深度、标签和视觉消失验证信息。
- `tip` test 链的 `single_target_mode` 默认保持关闭；如果手动开启，推理结果会先按置信度截断到单个框，这会绕过多框中心优先选择，主要用于旧式单目标调试。
- 对齐误差进入 `alignment_tolerance_px` 并稳定达到 `alignment_stable_frames` 后，tip test 节点必须先 x 负向前探并等待 0x06 前方限位反馈；收到限位后才会通过 `/mechanism/send_command` 共享 transport 下发一次 `GRAB_TIP(0x01)` 空 payload。它不直接打开目标 MCU 串口，`/cmd_vel` 消费和 mechanism transport 都由 `rc26_mcu_transport` 提供。
- 启用自动对线时，同一时刻不要启动遥控、决策导航或其它 `/cmd_vel` 发布权威；必须启动 `rc26_mcu_transport` 消费 `/cmd_vel` 并提供 `/mechanism/send_command` 与 `/mechanism/command_feedback`。
- `realsense_d455.launch.py` 默认固定选择 R2 当前 D455 序列号 `239222303644`，避免多 RealSense 或 USB 枚举顺序变化时选错设备；如现场更换相机，可临时用 `serial_no:=<new_serial>` 覆盖。该 wrapper 会兼容并清理手工传入的外层引号或 `_` 前缀，但传给 `realsense2_camera_node` 的实际值必须是纯序列号字符串。
- `test_kfs_vision.launch.py` 默认仍是 RealSense + `kfs_vision_test_node` 的纯视觉 overlay；传 `action_enable:=true direction:=up|down` 后会按 `start_mcu_transport:=auto` 自动带起 `rc26_mcu_transport`，并启动 KFS 底盘/机构测试链。KFS action test 会先等待 `/mechanism/send_command` 被本节点发现，等待上限由 `kfs_action_service_wait_timeout_s` 控制，避免 launch 并发启动时 ROS graph 尚未发现 service 就直接失败；随后发送方向对应的机械臂预调命令并等待同 seq 完成反馈：`up -> ARM_RAISE(0x04)/ARM_RAISE_DONE(0x02)`，`down -> ARM_LOWER(0x05)/ARM_LOWER_DONE(0x03)`。预调 service ACK 等待由 `kfs_action_arm_prep_service_timeout_ms` 控制，默认 `6000ms`，用于覆盖底层 `0x00` 到 `0x09` 的可靠发送重试；预调 done 等待由 `kfs_action_arm_prep_done_timeout_s` 控制，默认 `10s`，从 service ACK 返回并拿到 `seq` 后开始计时。预调完成后用 `T_*` 目标做画面中心横移对齐，进入 x 趋近前只锁定一次 `vision_depth_min_m ~ vision_depth_max_m` 内的有效深度；`direction=down` 会先发送 `ARM_SECOND_LOWER(0x0E)` 并等待同 seq 的 `ARM_SECOND_LOWER_DONE(0x0A)`，等待成功后才开始开环计时并发布固定 x 速度，`direction=up` 不触发该命令。开环前进不再因趋近途中的框消失、框跳变或深度波动停车/重搜。计划时长到达后停车并发送空 payload 的 `GRAB_KFS_UP(0x03)` 或 `GRAB_KFS_DOWN(0x02)`；夹取 service ACK 后会保存触发夹取的 `label + bbox + sequence`，只有同 label 且 bbox IoU 达到 `kfs_action_grab_verify_iou_threshold` 的原目标连续 `kfs_action_grab_verify_lost_stable_frames` 个新推理帧不可见，才显示 `SUCCESS`；验证超时、原目标仍可见或无新帧都会停车进入 `FAILED`。
- 当前 `tip` test 参数仍默认优先 `camera_index=2` 这路外接 USB 摄像头，但 `auto_scan_camera` 已默认打开；如果首选设备能枚举却读不出第一帧，节点会打印中文告警并自动扫描其他 `/dev/video*` 作为兜底。
- 默认联调入口仍然是 RealSense + `kfs_vision_test_node`（`test_kfs_vision.launch.py`）；KFS action test 需要显式 `action_enable:=true` 才会发布速度和机构指令，tip test 节点不参与默认 launch，需要单独显式启动。

## 模块边界

- 它只负责视觉推理和目标定位，不负责比赛级流程决策
- 它不是显示层，也不提供 Web 可视化
- AidLite 和 ONNX Runtime 后端都依赖部署环境；显式指定未编译进当前二进制的后端会直接报错，`engine: auto` 只在对应后端已编译启用时自动选择或回退
- KFS action test 继续是包内受限实机测试入口，不是比赛级流程决策权威；开启前必须停用遥控、决策导航、台阶树或其它 `/cmd_vel` 发布节点。
- `tip` test 链继续留在包内，但与默认 RealSense 主链隔离；它面向 USB 相机/单目 test，不是当前决策运行时权威入口
- `tip` test 链的自动横移和限位前探只通过标准 `/cmd_vel` 接入底盘，限位反馈只订阅 `/mechanism/command_feedback`，抓取只通过 `/mechanism/send_command` 共享 transport 接入机构；它不拥有目标 MCU 串口，也不是默认导航/决策运行时权威入口
- KFS action test 同样只通过标准 `/cmd_vel` 接入底盘，通过 `/mechanism/send_command` 共享 transport 发送机械臂预调、down 方向第二节放下和 KFS 夹取 raw command，并订阅 `/mechanism/command_feedback` 等待对应完成反馈；它不直接打开目标 MCU 串口，也不改变 `rc26_bringup + rc26_decision` 的正式 KFS 阶梯等待链。
- `VisionInferenceManager` 当前除 `getLatestDisplay()` 外，还提供 `getLatestFrameSnapshot()` 供 headless 决策节点消费；getter 端深拷贝图像与检测结果，不新增视觉 ROS topic，也不改变 `/vision/tip_detections` 等既有外部契约。

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
- tip test 链已删除旧视觉状态下发；当前只在自动对线启用、稳定对齐并收到 0x06 前方限位反馈后，通过 `/mechanism/send_command` 共享 transport 发送 `GRAB_TIP(0x01)` 空 payload。
- tip test 节点当前按测试链口径保留为 `test/tip_vision_test_node.cpp` 单文件实现，便于联调时直接查看参数、串口、相机、推理和 overlay 全链路而不改变外部行为；对线目标选择与速度计算已收口到共享 alignment helper，当前 MC 端头夹取与 MF 预选赛 KFS 横移都复用这套 helper，避免 test 与决策链分叉。
- 本次进一步把 tip 推理收口到主链 `InferenceEngine / AidLiteEngine`：`tip_vision_test_node` 只保留 USB 相机、目标选择、可选对线控制、抓取 service 触发和 overlay 业务；私有后处理头文件已删除，tip test 节点私有声明合并进 `test/tip_vision_test_node.cpp`，不再放入公开 `include/`。
- 本次按其他模块口径把 tip test 链路源码收口到包根 `test/`，不再放入 `src/` 或公开 `include/`；tip test 参数和模型资产直接放在包内 `config/` 与 `models/` 根目录，避免测试链继续维护额外嵌套目录。
- 模型 profile 已统一到 `config/vision_models.yaml`，`config/test/vision_models_tip.yaml` 删除；模型文件按用途重命名为 `kfs.pt/onnx` 与 `tip.pt/onnx`。
- 本次把 `engine: auto`、启动中文日志和共享后端解析收口到 `backend_resolver + engine_factory`；非 AidLux 系统启动时会自动切到本地 ONNX Runtime，而不是继续撞 AidLite stub。
- 本次把 AidLite / 本地 ONNX 共用的 YOLO 交集继续拆到明确阶段：`preprocess/yolo/yolo_image_preprocessor` 负责输入预处理，`postprocess/yolo/yolo_detection_postprocessor` 负责解码与 NMS，`shared/transforms/yolo_transform` 负责跨阶段变换元数据；`tip_vision_test_node`、`kfs_vision_test_node`、`tip_localizer_node` 与 `yolo_inference_test` 统一走这套共享入口。
- 本次新增 `shared/sensors/depth_roi_sampler`，把 `vision_inference_manager` 和 `tip_localizer` 里原本各自维护的深度 ROI 中值采样逻辑收口到共享层，避免两条链继续各改各的。
- 本次进一步把 `inference` 拆成 `contracts / config / runtime / yolo / aidlite / onnx` 六组，减少“一层目录内同时混放接口、配置、运行时和后端实现”的平铺耦合。
- 本次补强了 tip 相机初始化日志，并把默认 tip 参数改成“优先外接摄像头、失败时自动扫描回退”；像 `/dev/video2` 这种能枚举但首帧超时的 UVC 设备，不会再让节点静默卡死在无窗口状态。
- 本次进一步把 tip test 资源目录扁平化：旧的嵌套 test 参数/模型目录已提升并收口到包内 `config/` 与 `models/` 根目录，同时把标签文件显式命名为 `kfs_labels.txt` 与 `tip_labels.txt`，减少同名资源和相对路径歧义。
- 本次进一步把 `tip_default` 的 AidLite profile 收口到 float ONNX 默认口径，删除了旧的显式 tensor 名和量化/反量化参数；当前 tip 仍保留 `letterbox` 作为与默认 `kfs_default` 不同的预处理行为。
- 端头模型 profile ID 已从历史测试命名 `tip_test` 改为更通用的 `tip_default`；模型文件仍是 `models/tip.onnx`，标签文件仍是 `models/tip_labels.txt`。

## 最近修改

- **KFS 向下夹取开环前等待第二节机械臂放下**：`kfs_vision_test_node` 的 `direction=down` 动作链在横移对齐稳定、锁定一次有效深度并计算开环距离后，会先发送 `ARM_SECOND_LOWER(0x0E)`，等待同 `seq` 的 `ARM_SECOND_LOWER_DONE(0x0A)`，再开始 x 方向开环趋近；若计划距离为 0，也仍先等待 `0x0A` 后直接发送 `GRAB_KFS_DOWN(0x02)`。`direction=up` 不触发该命令，继续保持原来的 `ARM_RAISE -> 视觉对齐 -> 开环趋近 -> GRAB_KFS_UP` 流程。

- **KFS action test 改为对齐后纯开环趋近**：`kfs_vision_test_node` 现在只在 `Search/Align` 阶段用 YOLO 框中心做横移闭环；横移稳定后锁定一次有效深度，并用 `max(0, locked_depth - kfs_action_grab_distance_m) / kfs_action_approach_speed_mps` 计算 x 方向前进时间。`kfs_action_grab_distance_m` 当前表示机械臂可触达距离/开环停止距离；`vision_depth_min_m / vision_depth_max_m` 只约束进入开环前的锁定深度。进入 `APPROACH` 后不再依据持续识别框或实时深度闭环控制距离，框消失、框跳变或深度波动不会让节点停车重搜；计划时长到达后停车并发送 `GRAB_KFS_UP/DOWN`，后续仍用原目标视觉消失确认物理夹取成功。

- **R2 D455 启动入口固定相机序列号**：`launch/realsense_d455.launch.py` 的 `serial_no` 默认值已固定为纯序列号 `239222303644`，与当前 R2 D455 实机枚举结果一致；直接启动相机或经 `test_kfs_vision.launch.py` 间接启动相机时都会默认选择这台设备，避免多相机或 USB 枚举顺序变化导致误选。该 wrapper 会在传给 `realsense2_camera_node` 前清理手工覆盖值的外层引号和 `_` 前缀，避免把字面量引号作为序列号的一部分。该改动只属于相机装配选择，不改变视觉 topic、模型推理或 KFS action test 的运动/机构接口。

- **KFS 独立测试显式补齐深度窗口**：`config/kfs_vision_params.yaml` 现在显式设置 `vision_depth_min_m / vision_depth_max_m`，当前实机调试值为 `0.50m ~ 0.55m`。注意两个链路参数名前缀不同：独立视觉节点读取 `vision_depth_*`，决策 MF 预选赛读取 `mf_preselect_depth_*`，两者可以按现场任务分别调参。

- **KFS action test 放宽预调 ACK/完成反馈等待**：`kfs_vision_test_node` 的机械臂预调 service ACK 等待默认从 `200ms` 调整到 `6000ms`，夹取 service ACK 等待同步调整到 `6000ms`，避免上层在 `rc26_serial` 仍按 `0x00` 到 `0x09` 做可靠发送重试时提前判定失败。机械臂预调完成反馈等待默认从 `3s` 调整到 `10s`，且从 service ACK 返回、拿到用于匹配反馈的 `seq` 后才开始计时。`test_kfs_vision.launch.py` 透传这些超时参数，实机联调可按机械臂实际动作时间临时覆盖。

- **KFS 测试节点刷屏日志默认降噪**：`kfs_vision_test_node` 的 `log_detections` 与 `log_status` 默认均为 `false`，不再每个有效检测帧打印 `[检测]`，也不再按 `print_rate_ms` 周期打印 `[状态]`。启动、动作阶段、service 请求、失败原因和 overlay 显示保持不变。`test_kfs_vision.launch.py` 已透传同名参数，需要看逐帧检测或周期状态时可临时设置 `log_detections:=true` / `log_status:=true`，或在参数文件中打开。

- **KFS action test 修正启动时 service 发现竞态**：`kfs_vision_test_node` 开启动作链后不再在首次 `service_is_ready()` 为 false 时立刻进入 `FAILED`，而是按 `kfs_action_service_wait_timeout_s` 等待 `/mechanism/send_command` 被本节点发现。该改动只处理 `test_kfs_vision.launch.py` 同时启动 `rc26_mcu_transport` 和视觉节点时的 ROS graph 发现时序，不改变 `/cmd_vel` 或 raw mechanism transport 的权威边界。

- **KFS 独立视觉测试链同步机械臂预调与物理夹取验证**：`kfs_vision_test_node` 的 `action_enable:=true` 链路现在先按 `direction` 发送 `ARM_RAISE(0x04)` / `ARM_LOWER(0x05)` 并等待同 seq 完成反馈，随后对 `T_*` 目标横移对齐、x 正向趋近、发送 `GRAB_KFS_UP(0x03)` / `GRAB_KFS_DOWN(0x02)`，最后用同 label + bbox IoU 的原目标连续新帧消失确认物理夹取成功。独立测试中视觉验证失败会停车进入 `FAILED`，便于单次实机验收；同目标匹配 helper 已抽到 `shared/target/visual_target_match`，供 vision test 与 decision MF 预选赛共用。

- **KFS 独立视觉测试新增默认关闭的底盘/机构动作链**：`launch/test_kfs_vision.launch.py` 新增 `action_enable`、`direction`、`start_mcu_transport`、`cmd_vel_topic`、`target_serial_port`、`target_baudrate` 和 `params_file` 参数；默认仍只启动 RealSense + overlay。开启 `action_enable:=true` 时，launch 按 `start_mcu_transport:=auto` 自动带起 `rc26_mcu_transport`，`kfs_vision_test_node` 只选 `T_*` 目标，先做方向对应机械臂预调，再横移对齐、x 正向低速趋近并发送 `GRAB_KFS_UP(0x03)` 或 `GRAB_KFS_DOWN(0x02)`；overlay 会叠加 phase、目标、偏差、深度、cmd、seq 和视觉验证状态。该能力仅用于独立联调，运行时必须确保没有其它 `/cmd_vel` 权威。

- **kfs 主链测试链整体 kfs 化 + 新增实时 overlay 窗口**：`vision_test_node` 改名为 `kfs_vision_test_node`，`launch/vision_test_with_camera.launch.py` 改名为 `launch/test_kfs_vision.launch.py`，源文件 `git mv` 为 `src/nodes/kfs_vision_test_node.cpp`（保留历史）。节点新增主线程 OpenCV overlay：画全部检测框 + `类别[id] 置信度` + 角落帧率/检测数，并叠加 best target 的 D455 深度距离与中心十字；`show_window`（默认 true）/`window_name` 可经 `test_kfs_vision.launch.py` 参数透传，`show_window=false` 走无头（无 DISPLAY 时也自动降级）。配套 `VisionInferenceManager` 新增 `getLatestDisplay()`，在独立 `display_mutex_` 下暴露“最近彩色帧 + 完整 detections + 最新 TargetResult”，getter 端 `clone()` 深拷贝，不阻塞推理线程；CMake 给 `kfs_vision_test_node` 显式补了 OpenCV include/库（共享库的 OpenCV 是 PRIVATE 不传递）。tip test 链零改动。

- **tip 多框对线目标选择修正**：`test/tip_vision_test_node.cpp` 的 primary target 选择从“最大面积优先”改为“初次按框中心距离画面中心竖线最近获锁，随后持续跟踪同一物理端头”；自动横移对线继续只使用 primary target 的 `offset_px`，多框场景下不再跟随边侧大框反复切换。当前默认按后置相机口径反转横移方向，可由 `alignment_invert_direction` 配置修正。
- **tip_vision_test_node 日志中文化**：将 test/tip_vision_test_node.cpp 中所有 RCLCPP 日志和 std::fprintf 错误信息从英文转换为中文，与仓库其他模块日志口径保持一致。覆盖范围包括相机初始化、推理引擎状态、端头对准控制、GRAB_TIP 指令发送和运行时帧率统计等全部日志出口。
