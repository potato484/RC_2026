# rc26_kfs_keepout 调试

## 模块定位

`rc26_kfs_keepout` 已归档为 source-only 历史源码包。当前主链不编译它的运行时目标，不通过 bringup 启动它，`rc26_decision` 不调用 `/kfs_keepout/set_runtime`，也不订阅 keepout 输出。

本页仅用于显式恢复历史 keepout 节点时的本地调试资料，不属于当前 R2 默认联调顺序。

## 适用场景

- 显式恢复归档目标后，单独验证历史 KFS 占据掩码、overlay 和心跳
- 复现历史 keepout runtime 行为
- 确认历史参数与接口，不得把 `/mf_block_overlay` 或 `/kfs_filter_mask` 接回当前主链

## 前置条件

- 已 `source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"`
- 若单包调试，需要手工提供 `/mf_kfs_state`
- 当前整车联调不会通过 `rc26_bringup` 带起本包

## 标准编译

```bash
cd "${RC26_WS:-$HOME/RC_2026}"
MAKEFLAGS='-j2 -l2' colcon build --executor sequential --parallel-workers 1 \
  --packages-select rc26_kfs_keepout \
  --cmake-args -DRC26_ENABLE_ARCHIVED_RUNTIME_TARGETS=ON
source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"
```

## 推荐启动

当前整车链路不再启动 keepout。归档恢复调试时可直接运行历史节点：

```bash
ros2 run rc26_kfs_keepout kfs_block_fuser_node --ros-args \
  -p grid_layout_file:=${RC26_WS:-$HOME/RC_2026}/src/rc26_kfs_keepout/config/r2_mf_world.yaml \
  -p mask_topic:=/kfs_filter_mask \
  -p heartbeat_topic:=/kfs_keepout_heartbeat
```

## 最小验收

```bash
ros2 topic hz /kfs_keepout_heartbeat
ros2 topic echo /kfs_filter_mask --once
ros2 topic echo /mf_block_overlay --once
ros2 topic echo /diagnostics --once
```

单包调试时可以手工打入状态：

```bash
ros2 topic pub --once /mf_kfs_state rc26_interfaces/msg/MfKfsState \
  "{cells: [{grid_id: 5, kfs_type: 1, confidence: 1.0}]}"

ros2 topic pub --once /kfs_force_release_grid std_msgs/msg/UInt8 "{data: 5}"
```

## 优先排查

- 心跳有、掩码没变化：先确认 `/mf_kfs_state` 是否真的在变，再看 `min_confidence` 和 `dwell_cycles`。
- 当前决策侧不再等待 keepout；如果恢复历史链路，需要另行恢复对应 BT/决策调用。
- 单包调试时起不来：当前包没有独立 launch 文件，直接用 `ros2 run ... kfs_block_fuser_node` 才是当前真实入口。

## 相关入口

- [rc26_decision调试](./rc26_decision调试.md)
