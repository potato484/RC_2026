# rc26_small_gicp 调试

## 模块定位

`rc26_small_gicp` 是 `rc26_localization` 使用的点云配准基础库，本身不是独立运行时节点。

## 适用场景

- 排查配准性能、退化和鲁棒核行为
- 给 `rc26_localization` 做算法层排障
- 需要做性能剖析或离线实验时快速定位库层入口

## 前置条件

- 已 `source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"`
- 更常见的真实验证入口仍然是 `rc26_localization`

## 标准编译

```bash
cd "${RC26_WS:-$HOME/RC_2026}"
MAKEFLAGS='-j2 -l2' colcon build --executor sequential --parallel-workers 1 \
  --packages-select rc26_small_gicp rc26_localization
source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"
```

## 推荐验证

优先通过重定位链验证：

```bash
ros2 launch rc26_bringup localization.launch.py
```

如果只是做性能观察，可在运行定位链时配合：

```bash
htop
```

## 最小验收

```bash
ros2 topic echo /localization/diagnostics --once
ros2 topic echo /localization/pose_with_cov --once
```

## 优先排查

- 想单独看 `small_gicp`：当前仓库没有独立长期维护的运行节点，真实排障入口是 `rc26_localization`。
- 配准性能异常：先看 CPU 负载和 `localization/diagnostics`，再决定是否做更深的 profiler 分析。

## 相关入口

- [重定位启动](./重定位启动.md)
- [rc26_localization调试](./rc26_localization调试.md)
