**核心准则**
- 检索项目代码或实现方案时，优先使用 `ace-tool`。
- 对项目做任何修改前，如果需要了解模块职责、系统边界、接口约定或现有实现背景，先阅读 `docs/` 中与任务相关的文档，再回到代码核对细节。
- 对具体代码进行修改时，先遵循 `docs/` 中已经明确的实现说明、接口契约和架构约束；如果文档与代码或真实接口冲突，以代码和实际接口为准，并明确说明冲突点，不得无说明地忽略。
- 涉及职责边界、依赖方向、运行时权威归属或接口语义的改动，先看 `docs/fitness/` 与 `docs/middle/`，确认这是不是一次普通补丁，还是一次需要显式说明的架构变更。
- 如果改动改变了包职责、输入输出、接口契约或模块边界，必须同步更新相关 README 和 `docs/` 文档。
- 如果 `ace-tool` / `grok` 在当前会话不可用，明确说明不确定性后，使用本地代码与可用工具继续推进，不阻塞任务。
- 当前项目仅为 R2 这个自动机器人设计，R1 只是手动机器人。

**修改前优先阅读的文档**
- 涉及整体边界、职责拆分、依赖方向、文档同步规则时，先读 `docs/fitness/shared_rules.md`，再按任务类型继续读对应专题文档。
- 涉及 `src/` 下 ROS2 工作区、包职责、运行时分层、bringup、控制、决策、感知、可视化边界时，优先读 `docs/fitness/architecture_fitness_ros2_workspace.md`，并补读 `docs/backend/archive/` 下对应包文档。
- 涉及 `merlin-bt-visualizer` 或前端行为边界时，优先读 `docs/frontend/achieve.md` 和 `docs/fitness/architecture_fitness_frontend.md`。
- 涉及 ROS2 topic、service、前后端字段、桥接层接口语义时，优先读 `docs/middle/openapi.yaml` 和 `docs/middle/modules/*.yaml`。
- 需要快速建立某个 `rc26_*` 包的上下文时，优先把 `docs/backend/archive/<pkg>.md` 作为入口；真正落地改代码前，必须再回到对应源码和实际接口确认。

**判断依据**
- 建立上下文时，以 `docs/`、项目代码和可获取的搜索结果为主要依据，避免无依据猜测。
- 最终判断以项目代码、实际接口和可验证结果为准；`docs/` 的作用是帮助快速建立正确上下文，并为实现提供约束。
- 在调用编程语言的非内置库时，优先查阅官方文档或权威资料（`grok`、`context7` 等）；若无法联网检索，先标注风险再编码。
- 优先处理当前仓库内容，不把仓库外假设当作已知事实。
- 工作环境为 Linux Ubuntu 22.04；涉及 Python 命令统一使用 `python3`。
- 编译验证统一使用 `MAKEFLAGS='-j2 -l2' colcon build --executor sequential --parallel-workers 1 --packages-select <pkg...>`；需要提速时优先小幅调高 `MAKEFLAGS`，不要直接提高 `--parallel-workers`。

**R2 项目背景**
- R2 算力平台基于 Qualcomm QCS8550，采用 4nm 工艺，CPU 算力 300k DMIPS，集成 Adreno 740 GPU（3000 GFLOPS），提供 48 TOPS INT8 AI 推理能力，运行环境支持 AidLux（Android 13 + Ubuntu 22.04）深度融合，硬件配置为 16GB LPDDR5x + 256GB UFS 4.0。
- R2 是四驱麦克纳姆轮底盘；高精度陀螺仪位于底盘中心，用于位姿融合下发。达妙陀螺仪在比赛时间内没有明显漂移，处于可控范围。相关实现可参考 `rc26_merge_odom`，`rc26_telecontrol` 用于人为遥控测试 R2 机器人。

**`docs/` 目录说明**
- `docs/backend/archive/`：按 ROS2 包拆分的后端模块归档说明，用于快速理解各 `rc26_*` 包的职责、输入输出、边界和当前实现状态，适合作为建立模块上下文的入口。
- `docs/frontend/`：当前前端工程的实现说明与边界文档。现阶段核心文件是 `docs/frontend/achieve.md`，用来说明 `merlin-bt-visualizer` 已实现什么、没实现什么，以及它的准确定位，避免把它误判成在线驾驶舱。
- `docs/fitness/`：团队共同遵守的架构与维护准则目录。`shared_rules.md` 是总入口，`architecture_fitness_frontend.md` 面向前端约束，`architecture_fitness_ros2_workspace.md` 面向 ROS2 工作区约束；涉及边界、职责、依赖方向、文档同步规则时优先看这里。
- `docs/middle/`：中间层接口目录，用 OpenAPI 风格 YAML 去索引和描述“ROS2 topic/service 本身就是接口”这件事。`docs/middle/openapi.yaml` 是模块总索引，`docs/middle/modules/*.yaml` 按 `behavior_tree`、`diagnostics`、`localization`、`mechanism`、`navigation`、`stream`、`vision` 等主题拆分接口契约，适合给前端、桥接层、文档整理时统一字段和接口认知。
