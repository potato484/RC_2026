# 前端边界

## 当前允许的前端能力

- 读取行为树 XML、相关文档和本地模拟输入
- 在浏览器里做行为树查看、编辑和本地模拟执行展示

## 当前明确不允许的能力

- 浏览器直接成为 ROS2 控制面
- 前端自己定义 ROS2 运行时真源
- 把 `merlin-bt-visualizer` 说成机器人后端

## 当前边界事实

- `src/rc26_xhu_viewer` 已删除，不再存在仓库内 Web 可视化主链
- `src/rc26_bringup/foxglove/*.json` 只是历史模板资产，不是当前前端主链
- 当前前端边界只覆盖 `merlin-bt-visualizer`

## 维护时必须保持的事实

- `merlin-bt-visualizer` 是工具，不是权威后端
- 如果以后重新引入在线可视化，必须先设计新的 adapter boundary
- 不能把 React 页面直接绑到底层 ROS2 细节
