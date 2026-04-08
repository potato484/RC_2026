# 前端边界

## 当前允许的前端能力

- 读取 `rc26_xhu_viewer_server.py` 输出的 scene manifest、规划回放和 live 状态
- 在浏览器里做场地渲染、布局切换、路线回放和只读诊断聚合展示
- 通过受控 API 下发 `navigate_surface_route`

## 当前明确不允许的能力

- 浏览器直接成为 ROS2 控制面
- 前端自己定义 topo graph、surface graph 或 planner 真源
- 把 `rc26_xhu_viewer/viewer` 说成在线后端

## 当前 Web 可视化链路边界

- `src/rc26_xhu_viewer/rc26_xhu_viewer/viewer` 只连接本地 `rc26_xhu_viewer_server.py`
- `rc26_xhu_viewer_server.py` 只是 adapter，不替代 `rc26_topo_nav` action / planner 权威
- `src/rc26_bringup/foxglove/*.json` 只是历史模板资产，不再是当前前端主链

## 维护时必须保持的事实

- viewer 是工具，不是权威后端
- live 运行态默认只读
- 新增在线能力时，必须先设计新的 adapter boundary，而不是把 React 页面直接绑到底层 ROS2 细节
