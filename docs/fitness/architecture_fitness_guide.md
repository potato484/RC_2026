# RC_2026 架构 Fitness 准则

原先这份总文档已经按关注对象拆成两份独立文档，避免前端工具约束和 ROS2 工作区约束继续混写在一起。

## 文档拆分

- 前端版：[architecture_fitness_frontend.md](/home/potato/RC_2026/docs/fitness/architecture_fitness_frontend.md)
- ROS2 工作区版：[architecture_fitness_ros2_workspace.md](/home/potato/RC_2026/docs/fitness/architecture_fitness_ros2_workspace.md)

## 当前总原则

- `merlin-bt-visualizer` 仍然只是本地工程工具，不是机器人运行时权威后端。
- `src/` 仍然是 R2 自动机器人的主运行时工作区。
- 任何后续需求如果要打破这两个边界，都应视为一次明确的架构变更，而不是普通功能补丁。
