# rc26_vision 调试

## 模块定位

`rc26_vision` 是 R2 当前的视觉推理与弹头定位包，负责模型装载、图像推理、深度融合和目标三维定位。

## 适用场景

- 单独验证相机和推理链
- 验证 `tip_localizer_node` 是否能输出 `/vision/tip_detections`
- 给 `rc26_decision` 的带视觉模式做前置排障

## 前置条件

- 相机设备可用
- 模型文件和 `vision_models.yaml` 路径有效
- 已 `source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"`

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

## 相关入口

- [感知启动](./感知启动.md)
- [决策启动](./决策启动.md)
- [rc26_decision调试](./rc26_decision调试.md)
