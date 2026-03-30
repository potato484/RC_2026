# rc26_localization

## 模块定位

`rc26_localization` 是 R2 当前的激光重定位主模块，基于仓库内置的 `rc26_small_gicp` 和 GTSAM 进行局部配准与可选图后端优化。

## 当前实现

- 构建方式：共享库组件 + 可执行节点
- 导出节点：`rc26_localization_node`
- 启动文件：`launch/sentry_localization.launch.py`
- 运维脚本：
  - `scripts/setup_realtime_aidlux.sh`
  - `scripts/profile_localization_perf.sh`
  - `scripts/run_localization_acceptance.sh`
  - `scripts/publish_synthetic_loc_inputs.py`

源码已经按职责拆得比较细：

- `localization.cpp`：主节点入口和核心流程
- `localization_callbacks.cpp`：订阅回调和状态更新
- `localization_params.cpp`：参数装载
- `localization_map.cpp`：地图装载与管理
- `localization_registration.cpp`：点云配准主逻辑
- `localization_reloc.cpp`：重定位/重新锚定流程
- `localization_graph.cpp`、`pose_graph_backend.cpp`：在线图后端
- `keyframe_manager.cpp`、`online_scan_context_db.cpp`、`constraint_validator.cpp`：关键帧、候选约束与有效性检查
- `map_to_odom_smoother.cpp`：`map -> odom` 平滑输出
- `route_observability_evaluator.cpp`：路线可观测性评估

当前实现已经包含：

- 定位健康度输出
- 图后端状态输出
- 可选图优化闭环
- 面向控制和决策的风险语义输出

## 模块边界

- 它是定位模块，不是建图驱动，Mid-360 与 Point-LIO 仍在其他包
- 它不负责底盘控制，只给控制和决策提供位姿与健康度
- 它不负责前端可视化，只提供被前端和可视化消费的数据源
