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

## 模块边界

- 这个包不做全局定位，不代替 `rc26_localization`
- 它不做三维地形语义分割，不代替 `rc26_terrain`
- 它输出的是“基础地面高度与层级语义”，不是完整路径规划结果
