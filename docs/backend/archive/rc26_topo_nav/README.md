# rc26_topo_nav

## 模块定位

`rc26_topo_nav` 是 R2 当前唯一的导航表达层，负责把 node/task/route 目标转换成 topo route 与语义 corridor，并驱动自研执行链完成单边执行。

## 当前实现

- Action Server: `navigate_topo_target`
- 发布:
  - `/topo_nav/route`
  - `/topo_nav/corridor`
  - `/xhu_nav/corridor_cmd`
  - `/xhu_nav/active_edge`
  - `/xhu_nav/semantic_gate`
  - `/xhu_nav/diagnostics`
- 订阅:
  - `/localization/health`
  - `/localization/backend_status`
  - `/localization/route_observability`
  - `/mf_block_overlay`
  - `/xhu_nav/tracking_state`
  - 地形与 base_ground 相关输入
- 服务客户端:
  - `set_xhu_motion_mode`

## 关键变化

- 已删除旧兼容执行后端
- 已删除双后端选择参数
- `edge_executor` 现在只做两件事：
  - 切换 xhu 运动模式
  - 发布 `XhuSemanticCorridor` 并等待 `XhuTrackingState`

## 源码入口

- [src/topo_nav_node.cpp](/home/potato/RC_2026/src/rc26_topo_nav/src/topo_nav_node.cpp)
- [src/edge_executor.cpp](/home/potato/RC_2026/src/rc26_topo_nav/src/edge_executor.cpp)
- [src/planner.cpp](/home/potato/RC_2026/src/rc26_topo_nav/src/planner.cpp)

## 当前边界

- 负责 topo 图搜索与单边执行调度
- 不负责底层速度控制求解
- 只对接 topo/xhu 自研执行接口
