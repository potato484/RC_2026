# rc26_serial 调试

## 模块定位

`rc26_serial` 是底层串口通信基础库，本身不是独立运行时主链，通常通过 `rc26_merge_odom` 或 `rc26_mechanism` 间接验证。

## 适用场景

- 跑单元测试验证 `RingParser`、`AdaptiveTimeout` 等基础能力
- 排查 ACK 超时、断链恢复、串口重连
- 给 `rc26_merge_odom`、`rc26_mechanism` 的通信问题做底层定位

## 前置条件

- 已 `source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"`
- 如做真机联调，需要目标串口真实存在

## 标准编译

```bash
cd "${RC26_WS:-$HOME/RC_2026}"
MAKEFLAGS='-j2 -l2' colcon build --executor sequential --parallel-workers 1 \
  --packages-select rc26_serial rc26_mechanism rc26_merge_odom
source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"
```

## 推荐验证

先跑单元测试：

```bash
colcon test --packages-select rc26_serial
colcon test-result --verbose
```

再通过真实消费者验证：

```bash
ros2 launch rc26_mechanism mechanism.launch.py hal_type:=serial
```

或：

```bash
ros2 launch rc26_merge_odom merge_odom.launch.py
```

## 最小验收

```bash
ros2 topic hz /odom
ros2 topic echo /odom --once
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.0}}"
```

## 优先排查

- 重插串口后不恢复：先确认设备名有没有漂移，再看上层是否还指向旧端口。
- ACK 长时间失败：先排查物理串口、波特率，再看上层是否在并发争抢同一设备。
- 想“单独启动 rc26_serial”：当前没有单独 launch，实际应通过 `rc26_mechanism` 或 `rc26_merge_odom` 验证。

## 相关入口

- [遥控启动](./遥控启动.md)
- [rc26_merge_odom调试](./rc26_merge_odom调试.md)
- [rc26_mechanism调试](./rc26_mechanism调试.md)
