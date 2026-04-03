# 当前前端的明确边界

## 1. 不能误判成什么

当前前端的边界必须说清楚，否则很容易高估它的成熟度。

- 这些前端都不是完整的机器人在线驾驶舱。
- `merlin-bt-visualizer` 仍然没有真实 ROS2 / WebSocket / HTTP API 接入；它的“实机模式”只是本地状态切换和提示，不会接入真实运行时数据。
- `src/rc26_topo_nav/sim_viewer` 虽然有 HTTP / WebSocket，但它只连接本地 `topo_sim_server.py` adapter；实时模式只读消费 ROS 话题，不是浏览器直连 ROS2，也不是控制面。
- 它们都没有鉴权、没有 SSR、没有多页面站点级路由。
- 它们都没有通用后端持久化。
- `merlin-bt-visualizer` 开发态现在通过 Vite 本地适配层支持把当前区域 XML 写回仓库文件，但这不是浏览器天然文件写权限，也不是通用部署协议。
- `merlin-bt-visualizer` 虽然现在有独立 CD 打包 workflow，但它当前只产出静态站点 artifact，不等于已经具备在线部署后的写回后端。
- `merlin-bt-visualizer` 仍然没有撤销/重做、schema 校验、冲突处理、多人协作、拖线重连后的完整编辑语义。
- `src/rc26_bringup/foxglove/*.json` 只是 Foxglove 布局模板，不是这些前端工程本身。

## 2. 已经做到什么

截至当前代码状态，这个前端已经形成的是：

- `merlin-bt-visualizer`
  - 一个读取 `rc26_decision` 行为树 XML 的本地可视化工具。
  - 一套浏览器内基础属性编辑与 XML 导出的初步编辑器；开发态可通过本地适配层把当前区域 XML 写回源文件。
  - 一套仅用于演示和 UI 联调的本地模拟执行器。
  - 一套覆盖查看态、编辑态与开发态写回的独立自动化验证和静态打包链路。
- `src/rc26_topo_nav/sim_viewer`
  - 一个读取 topo 图、Gazebo world 与 KFS 对齐配置的本地三维路径规划 viewer。
  - 一套通过 `topo_sim_server.py` 输出场景 manifest、离线 run 帧和只读 live 事件的 adapter 链。
  - 一个支持完整 3D mesh、路径 tube、关键节点、搜索树和多视角相机的 WebGL 观察面。

## 3. 准确定位

截至当前代码状态，这些前端更准确的定义是：

- `merlin-bt-visualizer`: 读取 `rc26_decision` 行为树 XML 的本地可视化/演示工具，加上一套浏览器内基础属性编辑与 XML 导出的初步编辑器。
- `src/rc26_topo_nav/sim_viewer`: 围绕 `rc26_topo_nav` 的本地三维仿真与观测工具，离线模式可回放算法帧，实时模式只读观察运行时状态。

## 4. 需求评估时的边界提醒

如果后续有人提出下面这类需求，不能直接往当前前端里硬塞，而要先判断是不是架构变更：

- 浏览器直接接入 ROS2 运行时状态。
- 浏览器直接成为机器人控制入口。
- 浏览器直接写回仓库文件或在线持久化 XML。
  当前开发态的本地保存适配层只覆盖 `merlin-bt-visualizer` 三份固定行为树 XML，不能把它误判成通用在线持久化能力。
- 用 `merlin-bt-visualizer` 的“实机模式”承载真正的联机数据流。
- 让 `src/rc26_topo_nav/sim_viewer` 直接下发控制命令、修改规划真源或替代 `rc26_topo_nav` / `rc26_visualization` 的运行时职责。

这类需求一旦落地，就不再是简单的前端小补丁，而是需要新增单独 adapter、权限、持久化或在线契约设计。
