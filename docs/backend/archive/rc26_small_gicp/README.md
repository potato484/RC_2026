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

当前 Plane ICP 因子保持上游 `small_gicp` 的实现口径：使用目标法向量对 4D residual 做分量加权，并据此构造 4x6 Jacobian，而不是把残差收敛为单个 normal dot residual 标量。后续对齐上游或排查 Plane ICP 行为时，应以这一点作为当前真实实现边界。

## 源码入口与阅读顺序
- 先看 `README.md`，确认它只是算法底座而不是 ROS 2 业务包。
- 再看 `include/small_gicp/registration/`、`factors/`、`ann/`、`util/`，理解模板和算法拼装点。
- 最后看 `src/small_gicp/registration/registration_helper.cpp`，这里是少数真正落到 `.cpp` 的实现。

## 目录解剖
- `include/small_gicp/ann/`：近邻搜索和体素地图。
- `include/small_gicp/factors/`：GICP/ICP/Plane ICP 因子与鲁棒核。
- `include/small_gicp/registration/`：优化器、reduction、终止条件和配准模板。
- `include/small_gicp/util/`：下采样、法向估计、Lie 工具。
- `src/small_gicp/registration/registration.cpp` + `registration_helper.cpp`：少量非头文件实现。

## 关键文件体量
- `src/small_gicp/registration/registration_helper.cpp`：155 行。
- `src/small_gicp/registration/registration.cpp`：5 行，主要是实例化胶水。
- `README.md`：10 行，入口极简。

## 关键源码行段速览
- `src/rc26_small_gicp/CMakeLists.txt`：定义它以仓库内源码方式参与构建，而不是在构建期联网拉取。
- `include/small_gicp/registration/registration.hpp`：理解配准器模板如何由 factor/reduction/optimizer 组合。
- `include/small_gicp/factors/robust_kernel.hpp`：鲁棒核策略入口。
- `src/small_gicp/registration/registration_helper.cpp`：真正的 helper 实现，适合追具体配准步骤。

## 模块边界

- 它不是 R2 的业务包，而是算法基础件
- 它不负责 ROS 2 节点装配、launch 或比赛逻辑
- 上层定位策略、健康度语义和图后端都不在这里实现
