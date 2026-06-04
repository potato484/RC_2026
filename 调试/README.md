# 调试入口

`调试/` 是当前仓库唯一的调试文档入口，面向“先把运动主链拉通，再按模块排障”的现场使用场景。

当前主联调顺序固定为：

1. 遥控
2. 建图
3. 定位
4. 重定位
5. 回环
6. 导航

已确认事实：

- 当前遥控器已经可以发送指令并驱动车体移动
- 建图、定位、重定位、回环、导航仍需要按阶段逐步验证
- `rc26_terrain`、`rc26_base_ground`、`rc26_kfs_keepout` 已归档为 source-only 包，不参与当前默认联调顺序

## 主入口

- [联调顺序](./联调顺序.md)
- [遥控启动](./遥控启动.md)
- [建图启动](./建图启动.md)
- [定位启动](./定位启动.md)
- [重定位启动](./重定位启动.md)
- [回环启动](./回环启动.md)
- [导航启动](./导航启动.md)

## 附加场景

- [感知启动](./感知启动.md)
- [决策启动](./决策启动.md)
- [联调顺序兼容页](./调试清单.md)

## 模块入口

- [rc26_base_ground调试](./rc26_base_ground调试.md)（归档恢复资料）
- [rc26_decision调试](./rc26_decision调试.md)
- [rc26_kfs_keepout调试](./rc26_kfs_keepout调试.md)（归档恢复资料）
- [rc26_localization调试](./rc26_localization调试.md)
- [rc26_mechanism调试](./rc26_mechanism调试.md)
- [rc26_merge_odom调试](./rc26_merge_odom调试.md)
- [rc26_mid360_driver调试](./rc26_mid360_driver调试.md)
- [rc26_odom_interface调试](./rc26_odom_interface调试.md)
- [rc26_point_lio调试](./rc26_point_lio调试.md)
- [rc26_sensor_scan调试](./rc26_sensor_scan调试.md)
- [rc26_serial调试](./rc26_serial调试.md)
- [rc26_small_gicp调试](./rc26_small_gicp调试.md)
- [rc26_telecontrol调试](./rc26_telecontrol调试.md)
- [rc26_terrain调试](./rc26_terrain调试.md)（归档恢复资料）
- [rc26_vision调试](./rc26_vision调试.md)
- [Nav2导航调试](./Nav2导航调试.md)
