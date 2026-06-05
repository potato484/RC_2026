# rc26_odom_interface

## 模块定位

`rc26_odom_interface` 是 Point-LIO 与底盘统一坐标系之间的接口层，负责把上游里程计结果转换成下游统一消费的 odom 与 TF。

## 当前实现

- 导出节点: `rc26_odom_interface_node`
- 关键职责:
  - 消费 Point-LIO 内部 body frame 语义的 `/state_estimation`
  - 使用 `odometry.launch.py` 注入的 `base_link -> input_body_frame` 外参做底盘系映射
  - 权威 `odom -> base_link` TF 输出
  - 标准化 odometry 输出

## 当前边界

- 不负责里程计估计本体
- 不再查询 `base_link -> point_lio.body_frame` TF；内部 body 外参只接受 launch 注入，不再作为对外 TF 边存在
- 不直接做控制求解
- 供定位、Nav2 和可视化统一消费
