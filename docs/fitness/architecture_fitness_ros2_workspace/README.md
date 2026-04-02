# RC_2026 ROS2 工作区架构 Fitness 准则

## 1. 目的

本文档只讨论 `src/` 下的 ROS2 工作区架构，不讨论 `merlin-bt-visualizer` 的前端内部实现。

目标是给当前 R2 自动机器人主运行时建立一套长期稳定的架构与维护基线。

## 2. ROS2 工作区代码维护的基本准则

### 2.1 先稳住边界，再谈局部优化

- **推论**：这类项目后期最贵的成本不是某个算法写得不够强，而是边界混乱后谁都能改、谁都得懂、谁也不敢动。
- **规则**：每个包必须只有一个主要职责和一个主要变化原因。
- **规则**：如果一个改动同时在“协议适配、比赛策略、可视化聚合”三个维度变化，那就不是一个包该承受的事，应该拆层。

### 2.2 依赖方向必须单向向上

- **推论**：机器人系统要可维护，依赖关系必须从底层往上流，不能反向把高层语义灌回底层模块。
- **规则**：允许的依赖方向是：
  - 驱动与底层 IO
  - 状态估计与感知
  - 控制与执行
  - 决策与流程编排
  - 聚合诊断与操作员可视化
- **规则**：高层可以消费低层输出，低层不能依赖高层比赛策略或操作员语义。

### 2.3 跨包交互必须走契约，不走暗门

- **推论**：真正可维护的跨包耦合，必须通过消息、服务、Action、插件接口、参数契约表达。
- **规则**：跨包 API 一律使用 ROS message、service、action、plugin interface 或明确文档化的参数契约。
- **规则**：禁止一个包依赖另一个包的内部 `.cpp` 实现细节或未文档化副作用。

### 2.4 launch 负责装配，不负责业务

- **规则**：launch 负责进程拉起、参数选择、remap、namespace、生命周期顺序、可选后端切换。
- **规则**：核心算法、协议解析、安全策略、比赛逻辑，不能主要堆在 launch 里。

### 2.5 参数是部署输入，不是隐藏代码路径

- **规则**：参数可以调阈值、topic 名、路径、feature flag。
- **规则**：参数不能变成“看 YAML 才知道系统怎么工作”的隐形代码。
- **规则**：一个参数如果看名字和 README 都无法理解用途，那就是设计有问题。

### 2.6 硬件和安全相关模块必须有明确启停语义

- **规则**：所有碰硬件、影响安全、可能导致运动输出的模块，都必须有明确的 inactive、active、degraded、safe-stop 语义。
- **规则**：deactivate 不是“什么都不做”，而是必须进入可预期安全状态。

### 2.7 行为树负责编排，不负责取代领域模块

- **规则**：BT XML 负责阶段切换、顺序、fallback、retry、guard、任务编排。
- **规则**：重感知、控制求解、硬件协议、长耗时轮询，不应该直接塞进 BT 胶水层冒充“一个节点”。

### 2.8 可观测性是架构能力，不是调试附属品

- **规则**：任何会影响安全、运动或操作员判断的模块，都必须输出结构化诊断、健康度或其他机器可消费状态。
- **规则**：仅靠日志打印，不足以构成运行时契约。

### 2.9 关键资源只能有一个权威者

- **推论**：ROS 系统里最难排查的问题之一，就是多个节点都在“看起来合理地”发布同一类权威状态。
- **规则**：动态 TF、执行器命令权、整车健康汇总，必须各自只有一个文档化权威。

### 2.10 改动必须可按包验证

- **规则**：代码改动完成的标准，不只是“逻辑说得通”，而是被影响包至少能独立编译，必要时还能独立测试。
- **规则**：本仓库统一使用下列命令做包级编译验证：

```bash
MAKEFLAGS='-j2 -l2' colcon build --executor sequential --parallel-workers 1 --packages-select <pkg...>
```

## 3. 为当前 ROS2 工作区树立的架构准则

### 3.1 `rc26_bringup` 必须保持轻薄

- **规则**：`rc26_bringup` 可以拥有 launch 结构、参数拼装、后端切换、验收脚本。
- **规则**：`rc26_bringup` 不得承载算法主体、协议解析、比赛策略。
- **规则**：如果 bug 根因在下游算法包，就修算法包，不要把补丁长期堆在 `bringup`。

### 3.2 `rc26_decision` 只拥有策略，不拥有设备细节

- **规则**：`rc26_decision` 可以决定阶段、目标、条件、守护、重试、回退、流程顺序。
- **规则**：`rc26_decision` 不得直接承载串口协议、CAN 解析、相机驱动内部控制、控制器求解细节。
- **规则**：新增 BT 节点必须调用清晰的下层接口，而不是绕过下层边界直接碰设备逻辑。

### 3.3 `rc26_mechanism` 是机构执行唯一边界

- **规则**：所有机构动作语义、超时、取消、底层硬件抽象，必须收敛在这里或它的 HAL 中。
- **规则**：决策层可以请求动作、订阅状态，但不得在别处复制一套机制状态机。
- **规则**：inactive 状态下拒绝 goal 不是“优化项”，而是强制语义。

### 3.4 控制器必须保持 plugin 形态

- **规则**：控制器包可以依赖导航状态、定位健康、地形输入、控制参数。
- **规则**：控制器包不得吸纳比赛阶段语义、前端需求或临时策略分支。
- **规则**：任何带有“梅林阶段时这样做”“武馆阶段时那样做”的逻辑，如果不是纯控制保护，大概率应放在决策层而不是控制器层。

### 3.5 状态估计与感知包负责产出状态，不负责操作员策略

- **规则**：localization、terrain、vision、odom normalization、keepout 负责产出技术状态与语义健康度。
- **规则**：它们不得直接编码操作员界面逻辑或布局假设。
- **规则**：它们应输出结构化结果，让更高层按需消费。

### 3.6 `rc26_visualization` 是操作员语义聚合边界

- **规则**：操作员看见的整车健康语义应收敛在 `rc26_visualization`，而不是散落在 Foxglove 布局和 RViz 面板里各写一份。
- **规则**：Foxglove JSON 只是布局资产，不是诊断逻辑载体。

### 3.7 `rc26_odom_interface` 继续保持 TF 权威

- **规则**：每条动态 TF 边只能有一个文档化权威者。
- **规则**：除非做过明确架构变更并更新文档，否则不得新增第二个发布同一动态边的节点。

### 3.8 launch 参数必须声明明确、归属明确

- **规则**：新增 launch 参数必须显式声明、命名清晰、传递路径清楚。
- **规则**：参数文件归包所有，不归 `bringup` 统一托管其内部细节。
- **规则**：`bringup` 只负责选择加载哪个参数文件，不应长期成为所有内部调参逻辑的宿主。

### 3.9 BT blackboard 必须契约化

- **推论**：BT 系统里最容易失控的隐式耦合之一，就是 blackboard key 漫灌。
- **规则**：跨 subtree 共享的 blackboard key 必须文档化。
- **规则**：subtree 必须有明确输入/输出约定。
- **规则**：如果一个 BT 节点依赖远处节点的未文档化 blackboard 副作用，这就是架构缺陷，不是“灵活实现”。

### 3.10 长耗时 BT 工作必须异步化

- **规则**：BT 节点不得通过长时间 sleep、硬件轮询、重计算直接堵塞树执行线程，除非这个节点本身是明确设计过的异步可取消节点。
- **规则**：长耗时工作应落在 Action、后台 worker、专用模块中。

### 3.11 文档是接口的一部分

- **规则**：每个包必须维护 README，至少写清：
  - 职责
  - 输入
  - 输出
  - 运行时权威边界
  - 失效/降级语义
- **规则**：跨层边界变化时，必须同步更新对应 README 和本文档。

## 4. ROS2 工作区 Fitness Function

### 4.1 `bringup` 纯度检查

- 问题：这次改动是不是把领域逻辑塞进了 `rc26_bringup`？
- 通过标准：`rc26_bringup` 只改了 launch、参数装配、后端切换或验收脚本。

### 4.2 动态 TF 单一权威检查

- 问题：这次改动是否给已有动态 TF 边新增了第二个发布者？
- 通过标准：每条动态 TF 边仍只有一个文档化权威。

### 4.3 控制器无策略泄漏检查

- 问题：控制器代码里是否新增了比赛阶段、得分规则、区域名称等高层策略逻辑？
- 通过标准：控制器仍然只是控制器。

### 4.4 决策层无设备泄漏检查

- 问题：决策层是否开始自己处理串口帧、CAN 细节、底层设备重试？
- 通过标准：决策层继续只调用类型化下层接口。

### 4.5 硬件安全启停检查

- 问题：新增硬件或安全相关模块时，是否定义了 inactive、active、degraded、safe-stop 语义？
- 通过标准：启停语义明确且可验证。

### 4.6 跨包契约清晰度检查

- 问题：新增跨包依赖是否走了 msg / srv / action / plugin / 文档化参数契约？
- 通过标准：不存在“靠内部实现碰巧配合”的耦合。

### 4.7 参数卫生检查

- 问题：新增参数是否归属明确、名称清晰、文档可读？
- 通过标准：操作者和维护者能知道参数归谁、改什么、为什么改。

### 4.8 BT 契约纪律检查

- 问题：BT 改动是否引入了未文档化 blackboard 共享，或者阻塞式长耗时节点？
- 通过标准：blackboard key 可追踪，长耗时逻辑异步化。

### 4.9 可视化单向依赖检查

- 问题：聚合诊断和布局是否仍然只是消费状态，而不是反向成为状态和策略生产者？
- 通过标准：`rc26_visualization`、Foxglove、RViz 仍处于下游消费侧。

### 4.10 包级验证检查

- 问题：被影响的包有没有做编译验证，必要时有没有做测试验证？
- 通过标准：至少有包级 build 证据，缺失时必须明确记录原因。

### 4.11 README 与边界同步检查

- 问题：这次改动是否改变了包职责、输入输出、权威边界？
- 通过标准：相关 README 和架构文档同步更新。

## 5. 今后评审一个 ROS2 改动时，必须先回答的 7 个问题

1. 这个行为应该由哪一层拥有？
2. 这个包现在是不是该行为的唯一权威者？
3. 依赖方向有没有逆流？
4. 跨包交互是不是走了明确契约？
5. 启停、降级、安全语义是否明确？
6. 操作员可视化是否仍处于状态下游？
7. 被影响的包是否能用标准命令做编译验证？

## 6. ROS2 工作区最终立场

对 `src/` 这套工作区，正式架构立场应当是：

- `src/` 是机器人运行时工作区。
- `rc26_bringup` 是整车 composition root。
- `rc26_decision` 是比赛流程大脑，不是设备细节宿主。
- `rc26_mechanism` 和各控制器插件负责安全执行意图。
- localization、terrain、vision、odom 相关包负责产出规范化机器状态。
- `rc26_visualization` 负责把这些技术状态聚合成操作员语义。

任何后续需求如果要打破这些边界，都应视为一次明确的架构变更，而不是普通功能补丁。

## 7. 架构变更记录

### 7.1 rc26_topo_nav 引入（2026-04-01）

**变更类型**：架构级 — 新增导航表达层

**变更原因**：MF 和坡道链路从“旧二维全局规划 + 多层三维桥接”迁移到“拓扑图搜索 + 单边执行”，解决原有主链问题表达不匹配（二维 planner 很难直接表达离散合法站位）的根因。

**变更范围**：
- 新增 `rc26_topo_nav` 包（graph_loader / overlay_reducer / planner / edge_executor / diagnostics）
- 新增 `NavigateTopoTarget.action`、`MfBlockOverlay.msg`、`MfBlockOverlayCell.msg`
- `rc26_kfs_keepout` 新增 MfBlockOverlay 离散输出
- `rc26_decision` 新增 NavToTopoNode / NavToTaskPose / ExecuteTopoRoute BT 节点，新增 `mf_tree_topo.xml`
- `rc26_bringup` 新增分模式导航装配入口，topo 模式装配 topo 图、profile 和对应主树

**当前落地口径**：
- `rc26_bringup` 在 topo 模式下会把 `rc26_decision` 切到独立 topo 主树，并把 `team` / topo graph / topo nav profiles 一起装配给运行链
- `rc26_decision` 仍然拥有 MF 目标格选择；`NavToTaskPose(grid_id)` 只是把已选格映射为 topo node，不让 `rc26_topo_nav` 反向接管任务选格
- `rc26_kfs_keepout` 的 `r2_mf_world.yaml` 作为 shared 几何底座使用，真正发布到 `MfBlockOverlay.team` 的阵营来自运行态 KFS 输入
- `rc26_topo_nav` 图文件当前支持 `routes` 和 `edges.control_points`，分别服务 `ExecuteTopoRoute` 和坡道 / 转折 corridor 细化

**兼容策略**：`navigation_stack_mode` 默认 `legacy`，原有链路完全不受影响；topo 模式需显式启用。

**不变的边界**：
- TF 权威归属不变（rc26_odom_interface）
- 控制器权威暂不变化
- 任务策略仍在 rc26_decision（MerlinRuleWorldModel / SelectNextGrid）

### 7.2 xhu_direct 首轮落地（2026-04-01）

**变更类型**：架构级 — 导航执行后端新增 `xhu_direct`

**变更原因**：在保留 `legacy/topo` 兼容路径的前提下，为方案 2 提供“去旧兼容导航运行时”的可运行首轮链路，验证 corridor 直连执行闭环。

**变更范围**：
- `rc26_interfaces` 新增 `XhuSemanticCorridor.msg`、`XhuMotionModeState.msg`、`XhuTrackingState.msg`、`SetXhuMotionMode.srv`
- `rc26_nav_mode_manager` 新增 `xhu_motion_mode_manager_node`（独立模式管理、停稳预检查、watchdog 回退）
- `rc26_omni_controller` 新增 `xhu_motion_follower_node`（订阅 corridor/语义状态，直接输出 `cmd_vel`）
- `rc26_topo_nav` `edge_executor` 新增双后端执行路径：旧兼容执行后端 / `xhu_direct`
- `rc26_topo_nav` diagnostics 新增 `/xhu_nav/route`、`/xhu_nav/corridor`、`/xhu_nav/diagnostics`、`/xhu_nav/active_edge`、`/xhu_nav/semantic_gate`、`/xhu_nav/risk_markers`
- `rc26_bringup` 新增 `navigation_stack_mode:=xhu_direct` 装配路径，挂载 `xhu_motion_mode_manager` + `xhu_motion_follower` + `rc26_topo_nav(execution_backend=xhu_direct)`
- `rc26_decision` 新增独立 `xhu_direct` 主树（首轮仅启用 MF topo 子树）

**当前落地口径**：
- `xhu_direct` 模式下不启动旧规划执行进程
- `xhu_direct` 模式下旧地形限速桥接不启动，主链也不再向旧速度限制入口输出值
- `rc26_topo_nav` 在 `xhu_direct` 下通过 `set_xhu_motion_mode` + `/xhu_nav/corridor_cmd` 驱动执行，并根据 `/xhu_nav/tracking_state` 判定 `PASS/HOLD/REPLAN/ABORT`

**兼容策略**：
- `navigation_stack_mode` 默认仍为 `legacy`
- `topo` 模式继续保留旧兼容执行路径作为过渡与回归对照

**当前未完成项（显式记录）**：
- `xhu_direct` 首轮仅覆盖 MF topo 子树；MC / 对抗区仍未迁移到统一 topo 目标表达
- 旧兼容运行时仍在 `legacy/topo` 模式可用，尚未进入“比赛正式配置只保留 `xhu_direct`”阶段

### 7.3 旧导航链彻底退场（2026-04-02）

**变更类型**：架构级 — 删除旧导航运行时与兼容执行后端

**变更原因**：用户已明确要求仓库只保留自研导航实现，不再维护旧兼容导航路径、旧 waypoint 导航桥接或相关兼容接口。

**变更范围**：
- 删除旧二维执行器包
- 删除旧地形兼容桥接包
- 删除 `rc26_decision` 中旧 waypoint 导航桥接实现
- 删除 `rc26_interfaces` 中旧 waypoint / 安全 / 模式切换接口
- 删除 `rc26_bringup` 中所有旧兼容参数与测试入口
- `rc26_topo_nav` 收口为单一 xhu 执行器
- `rc26_nav_mode_manager` 收口为单一 `xhu_motion_mode_manager_node`
- `rc26_omni_controller` 收口为单一 `xhu_motion_follower_node`

**当前落地口径**：
- `slam:=false` 时整车固定装配 topo/xhu 自研导航链
- `rc26_decision` 固定使用 `main_tree.xml`
- 运动模式权威为 `set_xhu_motion_mode + /xhu_nav/motion_mode_state`
- 执行反馈权威为 `/xhu_nav/tracking_state`

**兼容策略**：无。旧兼容导航链和相关接口已从 `src/` 中删除。
