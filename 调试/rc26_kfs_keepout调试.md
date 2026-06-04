# rc26_kfs_keepout 调试

## 模块定位

`rc26_kfs_keepout` 负责把梅林区 KFS 状态融合成稳定的禁入约束输入，向导航和决策输出 `/kfs_filter_mask`、`/mf_block_overlay` 与 `/kfs_keepout_heartbeat`。

## 适用场景

- 单独验证 KFS 占据掩码、心跳和强制释放服务
- 排查导航为什么被 keepout 阻塞
- 排查决策 `keepout_gate` 为什么一直等待或降级

## 前置条件

- 已 `source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"`
- 若单包调试，需要手工提供 `/mf_kfs_state`
- 若整车联调，建议直接通过 `rc26_bringup` 带起

## 标准编译

```bash
cd "${RC26_WS:-$HOME/RC_2026}"
MAKEFLAGS='-j2 -l2' colcon build --executor sequential --parallel-workers 1 \
  --packages-select rc26_kfs_keepout rc26_decision rc26_bringup
source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"
```

## 推荐启动

整车链路推荐直接通过 bringup：

```bash
ros2 launch rc26_bringup bringup.launch.py slam:=false use_decision:=false
```

单包调试可直接运行节点：

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
- 决策侧一直等 keepout：先确认 `heartbeat_topic` 名称和决策使用的 topic 一致。
- 单包调试时起不来：当前包没有独立 launch 文件，直接用 `ros2 run ... kfs_block_fuser_node` 才是当前真实入口。

## 相关入口

- [决策启动](./决策启动.md)
- [rc26_decision调试](./rc26_decision调试.md)
- [Nav2导航调试](./Nav2导航调试.md)
