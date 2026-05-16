# rc26_vision 调试

## 模块定位

`rc26_vision` 是 R2 当前的视觉推理与端头定位包，负责模型装载、图像推理、深度融合和目标三维定位。

## 适用场景

- 单独验证相机和推理链
- 验证 `tip_localizer_node` 是否能输出 `/vision/tip_detections`
- 给 `rc26_decision` 的带视觉模式做前置排障

## 前置条件

- 相机设备可用
- 模型文件和 `vision_models.yaml` 路径有效
- 已 `source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"`
- `config/vision_models.yaml` 是当前唯一模型 profile 配置入口，默认 profile 为 `kfs_default`，tip 单目 test profile 为 `tip_test`
- tip 单目 test 链当前入口为 `launch/test_tip_vision.launch.py`，其默认模型资产位于 `models/test/tip/`
- tip 若打开 `serial_enable:=true`，当前会通过 `rc26_serial` 直接占用 `serial_device`，默认是 `/dev/ttyUSB1`；如果同一台车上 `rc26_merge_odom` 已经持有这条目标串口，不要并发启动两者

## 标准编译

```bash
cd "${RC26_WS:-$HOME/RC_2026}"
MAKEFLAGS='-j2 -l2' colcon build --executor sequential --parallel-workers 1 \
  --packages-select rc26_vision rc26_bringup
source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"
```

## 推荐启动

独立视觉链：

```bash
ros2 launch rc26_vision vision_test_with_camera.launch.py
```

tip 单目 test 链：

```bash
ros2 launch rc26_vision test_tip_vision.launch.py
```

通过整车链路带起视觉：

```bash
ros2 launch rc26_bringup bringup.launch.py \
  slam:=false \
  use_realsense:=true \
  use_decision:=false
```

## 最小验收

```bash
ros2 topic hz /camera/color/image_raw
ros2 topic hz /vision/tip_detections
ros2 topic echo /vision/tip_detections --once
```

## 优先排查

- 图像有、检测没有：先确认 `vision_models.yaml` 的模型路径和阈值。
- 决策带视觉起不来：先单独跑 `vision_test_with_camera.launch.py` 确认基础链通。
- Realsense 带起了但没有视觉结果：先看是否只是相机起来了而没有同时启动 `rc26_vision`。
- 如果要通过 manager 或 tip test 节点切到 tip profile，直接使用 `config/vision_models.yaml` 里的 `tip_test`，不要再使用已删除的 `config/test/vision_models_tip.yaml`。
- tip test 节点默认通过 `vision_config_file + model_id` 读取 `models/test/tip/tip.onnx` 和同目录 `labels.txt`；如果你替换模型，优先更新 `config/vision_models.yaml` 对应 profile。
- 当前主链 `AidLiteEngine` 会按输入 tensor shape 自动识别 `NCHW / NHWC`；如果你替换为新的 ONNX，优先看启动日志或引擎配置是否符合模型实际输入布局。
- 当前 tip 已不再做距离估计和距离 overlay；`show_center_distance` 与旧距离参数名保留只是为了兼容旧 YAML，不再影响目标判定。
- `models/test/tip/tip.onnx` 当前是固定 `640x640` 输入；单改 profile 的 `input_w / input_h` 不能把这份模型降到 `320`，实际输入尺寸以模型 tensor shape 为准。
- 当前犀牛派 X1 板上实测这份 `tip.onnx` 在 CPU 下约 `80~95ms / frame`，`infer_fps` 约 `10~12`；并且这套 AidLite ONNX 后端对它不支持 `GPU/DSP`。如果目标是 `30 infer_fps`，需要在有 `torch/ultralytics` 的机器上重新导出更小输入的 ONNX，或改走 QNN/AMF 量化资产。
- tip 串口发送当前已切到 `rc26_serial`，业务 payload 为 `grab_ready | dir_code | amp_code | ts16_lo | ts16_hi`，并以 `TIP_VISION(0x12)` 的 RC26 帧下发；若 MCU 端仍按旧的 `AA FF ... FE` 短帧解析，需要同步更新下位机。

## 当前目录口径

- `src/runtime`：配置读取、结果缓存、引擎切换
- `src/engines`：AidLite/YOLO 推理后端
- `src/pipelines`：`TipLocalizer` RGB-D + TF 定位流水线
- `src/nodes`：可执行入口
- `test`：tip 单目 test 链单文件源码；`config/test`、`models/test`：tip 单目 test 链参数与模型资产
- `src/tools`、`tools`：离线工具和实验脚本

## 相关入口

- [感知启动](./感知启动.md)
- [决策启动](./决策启动.md)
- [rc26_decision调试](./rc26_decision调试.md)
