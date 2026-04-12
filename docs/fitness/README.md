# RC_2026 共同遵守的准则

这份文档是 `RC_2026` 的总入口约束页，用来明确前端和 ROS2 工作区在协作时共同遵守的底线规则。

具体实现细则已经拆成两份独立文档，避免前端工具约束和 ROS2 工作区约束继续混写在一起。

## 文档拆分

- [`architecture_fitness_frontend`](architecture_fitness_frontend/README.md): 前端工具边界、查看态/编辑态分离、round-trip 和在线化约束。`(file: architecture_fitness_frontend/README.md)`
- [`architecture_fitness_ros2_workspace`](architecture_fitness_ros2_workspace/README.md): ROS2 工作区的职责边界、依赖方向、契约纪律和包级验证基线。`(file: architecture_fitness_ros2_workspace/README.md)`

## 共同遵守的准则

- 先守边界，再做功能。前端、ROS2 工作区、可视化配置各自有明确职责，不能为了省事跨层写逻辑。
- 任何跨模块交互都应走清晰契约，而不是依赖隐式约定、内部实现细节或临时补丁。
- 文档描述必须和真实能力一致，不能把本地模拟能力写成实机在线能力，也不能把装配层描述成算法层。
- 新需求如果会改变模块权威边界、依赖方向或运行时职责，应视为架构变更，而不是普通功能补丁。
- 已经落地并成为当前事实的架构变更，必须归档到对应实现入口文档，不再长期停留在独立变更流水账里。
- `merlin-bt-visualizer` 仍然只是本地工程工具，不是机器人运行时权威后端。
- `src/` 仍然是 R2 自动机器人的主运行时工作区。
