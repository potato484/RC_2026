# rc26_base_ground

## 模块定位

`rc26_base_ground` 是 R2 的基础标高与离散层级估计模块，用于把连续高度变化压成导航和机构更容易消费的地形层级语义。

## 当前实现

- 构建产物：`base_ground_estimator_node`
- 核心源码：`src/rc26_base_ground/src/base_ground_estimator_node.cpp`
- 配置文件：`src/rc26_base_ground/config/base_ground_estimator.yaml`
- 启动文件：`src/rc26_base_ground/launch/base_ground_estimator.launch.py`

当前实现重点在于：

- 根据高频位姿/速度输入估计机器人当前的地面参考高度
- 将 Z 轴变化离散化为台阶层级，而不是只保留连续高度
- 维护地形稳定、操作稳定、被举起等安全语义
- 为下游决策、地形门控和机构动作提供更稳的“当前层级/是否稳定”信号

## 源码入口与阅读顺序
- 先看 `launch/base_ground_estimator.launch.py`，确认节点名、参数文件和发布链路。
- 再看 `src/base_ground_estimator_node.cpp`，这个包的核心状态机、TF 发布和层级估计都集中在这里。
- 最后看 `config/base_ground_estimator.yaml` 和 `src/rc26_base_ground/docs/debug_guide.md`，确认阈值、稳定窗口和验收话题。

## 目录解剖
- `src/base_ground_estimator_node.cpp`：单文件主实现，负责样本窗口、抬起保护、层级状态和 TF。
- `config/base_ground_estimator.yaml`：台阶高度、姿态门槛、稳定窗口等部署参数。
- `launch/base_ground_estimator.launch.py`：单节点装配入口。
- `src/rc26_base_ground/docs/debug_guide.md`：运行期排查和 topic 验证手册。

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

- 这个包不做全局定位，不代替 `rc26_localization`
- 它不做三维地形语义分割，不代替 `rc26_terrain`
- 它输出的是“基础地面高度与层级语义”，不是完整路径规划结果
