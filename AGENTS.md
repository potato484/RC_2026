**核心原则**
- - **目标** 当前项目仅为R2这个自动机器人设计，R1只是手动机器人
**判断依据**
- 以项目代码和可获取的搜索结果作为主要判断依据，避免无依据猜测。
- **Project Scope**: 优先处理当前仓库内容；
- **工作 Environment**: Linux Ubuntu 22.04。涉及 Python 命令统一使用 `python3`。
- **编译验证** 统一用colcon build --symlink-install --parallel-workers 3 --cmake-args -DCMAKE_BUILD_TYPE=Release来进行编译验证，以限制编译核心
- **R2算力平台** 基于 Qualcomm® QCS8550 平台，采用领先的 4nm 工艺，CPU 算力达 300k DMIPS，并集成 Adreno 740 GPU（3000 GFLOPS）。系统提供高达 48 TOPS 的 INT8 AI 推理能力，支持 AidLux (Android 13 + Ubuntu 22.04) 深度融合环境。硬件配置 16GB LPDDR5x + 256GB UFS 4.0 顶级存储组合，具备卓越的 8K 视频编解码 性能，是高性能边缘计算的理想选择。
- **R2基本信息** 四驱麦克纳姆轮底盘，高精度陀螺仪放在底盘中心用来进行位姿融合下发（陀螺仪在比赛时间内没有明显漂移，在可控范围内），参照rc26_merge_odom，rc26_telecontrol用来人为遥控测试R2机器人
