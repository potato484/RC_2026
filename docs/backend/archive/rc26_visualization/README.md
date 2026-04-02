# rc26_visualization

## 模块定位

`rc26_visualization` 负责把定位、控制、keepout、地形、机构和导航运行时状态压成统一的诊断语义。

## 当前实现

- 构建产物:
  - `visualization_status_core`
  - `rc26_visualization_status_node`
- 关键配置:
  - `config/visualization_status.yaml`
- 关键输入:
  - `/xhu_nav/semantic_gate`
  - `/xhu_nav/motion_mode_state`
  - `/xhu_nav/tracking_state`
  - `/mf_block_overlay`
  - `/kfs_filter_mask`
  - `/kfs_keepout_heartbeat`

## 关键变化

- 诊断输入已经完全收口到 xhu 主链与 keepout 约束输入
- 当前不再维护任何旧兼容导航状态映射

## 当前边界

- 只做状态聚合与事件生成
- 不参与导航控制和决策
