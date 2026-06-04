# rc26_base_ground

## 模块定位

`rc26_base_ground` 是已归档的 R2 基础标高与离散层级估计源码包。当前主运行时不再编译、启动或消费它；默认 CMake 只完成包配置，不生成节点、库、测试或安装目标。

## 当前实现

- 归档历史构建产物：`base_ground_estimator_node`
- 核心源码：`src/rc26_base_ground/src/base_ground_estimator_node.cpp`
- 配置文件：`src/rc26_base_ground/config/base_ground_estimator.yaml`
- 启动文件：`src/rc26_base_ground/launch/base_ground_estimator.launch.py`

历史实现重点在于：

- 根据高频位姿/速度输入估计机器人当前的地面参考高度
- 将 Z 轴变化离散化为台阶层级，而不是只保留连续高度
- 维护地形稳定、操作稳定、被举起等安全语义
- 曾为下游决策、地形门控和机构动作提供“当前层级/是否稳定”信号；这些输出现在不再进入当前主链

## 源码入口与阅读顺序
- 先看 `launch/base_ground_estimator.launch.py`，确认节点名、参数文件和发布链路。
- 再看 `src/base_ground_estimator_node.cpp`，这个包的核心状态机、TF 发布和层级估计都集中在这里。
- 最后看 `config/base_ground_estimator.yaml` 和仓库根目录 `调试/rc26_base_ground调试.md`，确认阈值、稳定窗口和验收话题。

## 目录解剖
- `src/base_ground_estimator_node.cpp`：单文件主实现，负责样本窗口、抬起保护、层级状态和 TF。
- `config/base_ground_estimator.yaml`：台阶高度、姿态门槛、稳定窗口等部署参数。
- `launch/base_ground_estimator.launch.py`：单节点装配入口。
- 仓库根目录 `调试/rc26_base_ground调试.md`：运行期排查和 topic 验证手册。

## 关键文件体量
- `src/base_ground_estimator_node.cpp`：537 行，几乎所有运行逻辑都在这里。
- `launch/base_ground_estimator.launch.py`：38 行，启动入口很薄。
- `config/base_ground_estimator.yaml`：23 行，参数量不大但决定层级判定灵敏度。

## 关键源码行段速览
- `src/base_ground_estimator_node.cpp:25-109`：构造函数，声明参数、创建 pub/sub、初始化 TF 广播器和状态窗口。
- `src/base_ground_estimator_node.cpp:110-201`：参数清洗、近水平面判定和垂向速度计算。
- `src/base_ground_estimator_node.cpp:202-344`：`onOdom()`、`resolveBasePose()`、`updateStabilityWindow()`，把里程计样本转成可判定的稳定状态。
- `src/base_ground_estimator_node.cpp:345-474`：抬起保护和层级状态机更新，是“是否换层”的核心。
- `src/base_ground_estimator_node.cpp:475-537`：层级发布、层级增量事件、TF 输出和 `main()`。

## 模块边界

- 不参与 `rc26_bringup`、`rc26_decision`、Nav2、smoke CI 的默认运行时链路
- 默认不发布 `base_ground/*` 话题，也不广播 `base_ground` TF
- 当前主链没有任何模块订阅 `base_ground/*` 数据；`rc26_decision` 只保留楼梯相关黑板默认值，不再从本包更新这些键
- 如未来恢复，必须先重新定义接口契约、启动入口、验证范围和文档边界

## 配置注释口径

- 本轮在 CMake 中加入默认关闭的归档构建开关，并从 bringup、decision 和 CI 默认闭包中移除 base-ground 链路。`config/base_ground_estimator.yaml` 仅作为历史调试资料保留。
