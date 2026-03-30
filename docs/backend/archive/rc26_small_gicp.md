# rc26_small_gicp

## 模块定位

`rc26_small_gicp` 是仓库内置的点云配准基础库，被 `rc26_localization` 直接以内嵌源码方式依赖。

## 当前实现

- 构建产物：基础库 `small_gicp`
- 主要目录：
  - `include/small_gicp/`
  - `src/small_gicp/`
- CMake 当前把它作为仓库内源码直接编译，而不是构建期联网拉取依赖

从当前 README 和构建方式看，这个包承担的是算法底座角色：

- 多线程点云配准
- 鲁棒核与优化策略
- 协方差和退化评估
- 作为 `rc26_localization` 的底层配准实现

## 模块边界

- 它不是 R2 的业务包，而是算法基础件
- 它不负责 ROS 2 节点装配、launch 或比赛逻辑
- 上层定位策略、健康度语义和图后端都不在这里实现
