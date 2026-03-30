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

## 源码入口与阅读顺序
- 先看 `launch/sentry_localization.launch.py` 和 `README.md`，确认这个包怎样被单独拉起。
- 再看 `src/localization.cpp`，这里是节点状态机和运行时骨架。
- 然后按职责继续看 `localization_registration.cpp`、`localization_reloc.cpp`、`localization_graph.cpp`、`localization_params.cpp`。
- 最后看 `scripts/` 和 `docs/`，理解性能剖析、验收和合成输入脚本。

## 目录解剖
- `localization.cpp`：节点骨架、状态机、健康度、后端状态和线程绑定。
- `localization_callbacks.cpp`：订阅回调和缓存更新。
- `localization_registration.cpp`：局部配准、协方差、诊断与 TF 发布。
- `localization_reloc.cpp`：全局重定位、scan context、候选通道和重锚定。
- `localization_graph.cpp` + `pose_graph_backend.cpp`：图后端、关键帧和外部锚点。
- `localization_params.cpp`：动态参数回调。

## 关键文件体量
- `src/localization.cpp`：1756 行，节点骨架非常重。
- `src/localization_reloc.cpp`：1210 行，重定位链本身就是一个子系统。
- `src/localization_registration.cpp`：811 行，局部配准和健康度输出主链。
- `src/localization_graph.cpp`：579 行，图后端接入。
- `src/localization_params.cpp`：536 行，参数面不小。

## 关键源码行段速览
- `src/rc26_localization/src/localization.cpp:134-769`：构造函数和节点初始化，创建 pub/sub、定时器、内部线程和缓存。
- `src/rc26_localization/src/localization.cpp:770-1215`：定位状态机、置信度管理、健康度/后端状态/路线可观测性输出。
- `src/rc26_localization/src/localization.cpp:1356-1720`：重定位 worker 和参数规范化；`1721-1756`：QCS8550 线程亲和力配置。
- `src/rc26_localization/src/localization_registration.cpp:34-445`：局部配准主流程；`446-811`：可观测性分析、协方差、诊断、TF 发布和被绑架检测。
- `src/rc26_localization/src/localization_reloc.cpp:34-819`：全局候选通道与 scan context 检索；`820-1210`：真正的全局重定位执行与 map->odom 恢复。
- `src/rc26_localization/src/localization_graph.cpp:23-385`：图后端初始化、外部 anchor 消费和局部配准后的图更新。

## 模块边界

- 它是定位模块，不是建图驱动，Mid-360 与 Point-LIO 仍在其他包
- 它不负责底盘控制，只给控制和决策提供位姿与健康度
- 它不负责前端可视化，只提供被前端和可视化消费的数据源
