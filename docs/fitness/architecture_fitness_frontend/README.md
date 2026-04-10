# RC_2026 前端架构 Fitness 准则

## 1. 目的

本文档约束当前仓库已经存在的前端工具，主要包括：

- `merlin-bt-visualizer`
- `src/rc26_xhu_viewer/rviz2/viewer`

目标是保证这些页面继续保持工具属性，而不是在实现上偷偷长成运行时后端。

## 2. 基本准则

### 2.1 前端必须保持消费者身份

- 前端只消费 `docs/`、代码真源和 adapter 输出
- 前端不能定义 planner、control 或 behavior tree 的运行时真相

### 2.2 在线化必须经过 adapter boundary

- React 页面不能直接耦合 ROS2 细节
- 当前允许存在的 adapter 例子就是 `src/rc26_xhu_viewer/rviz2/scripts/rviz2_rc26_server.py`
- adapter 负责把 topo 图、world、planner trace 和只读 live 话题转换成浏览器模型，但不拥有运行时导航权威

### 2.3 viewer 不能反向吞掉后端职责

- `src/rc26_xhu_viewer/rviz2/viewer` 可以展示路线、阶段区、keepout、定位健康和 BT 快照
- 但它不能替代 `rc26_topo_nav`、`rc26_xhu_viewer_status` 或 `rc26_bringup` 的运行时职责

### 2.4 页面文案必须反映真实能力

- 如果只是本地工具，就必须写成本地工具
- 不允许把 `local_web` 页面说成在线驾驶舱或机器人主后端

## 3. 当前项目规则

### 3.1 `merlin-bt-visualizer` 和 `src/rc26_xhu_viewer/rviz2/viewer` 都不是自研“后端”

- 一个是行为树工作台
- 一个是比赛场地可视化平台前端
- 两者都不拥有机器人控制权

### 3.2 规划回放必须继续尊重后端真源

- A* / surface-route / local planner 回放都以 CLI、`render_graph_sim_html.py` 或 ROS2 输出为准
- 不允许在前端里再复制一套“更真实”的规划逻辑

### 3.3 live 状态默认只读

- 当前 viewer 允许执行 `navigate_surface_route`
- 但整体仍然应当默认按只读状态理解 live bridge，不把它扩成浏览器控制面

## 4. Fitness Function

- 这次改动是否让前端开始承担运行时权威职责
- 这次改动是否让页面直接耦合 ROS2 细节而没有 adapter
- 文档和 UI 是否准确描述了当前工具边界

## 5. 当前立场

- `merlin-bt-visualizer` 是本地行为树工具
- `src/rc26_xhu_viewer/rviz2/viewer` 是本地 Web 可视化平台前端
- `rviz2` 只是在同一包内承载 GUI / Web 工具源码，不因此获得规划或状态权威
- 如果以后要进一步在线化，必须单独设计新的 adapter 架构
