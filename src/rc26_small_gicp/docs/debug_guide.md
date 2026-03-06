# rc26_small_gicp 调试指南

## 1. 编译库文件
`rc26_small_gicp` 作为一个独立的点云配准 C++ 库，提供了基础算法。在调试之前，需要先编译整个工程（R2 环境下推荐限制编译核心数）：
```bash
cd ~/RC_2026
colcon build --parallel-workers 1 --packages-select rc26_small_gicp
```

## 2. 编写测试程序
为了调试和验证 `small_gicp` 的性能和精度，可以编写一个独立的 C++ 测试程序（例如 `test_gicp.cpp`），或者使用 `rc26_localization` 中的测试用例。下面是一个使用 `small_gicp` 库的基础代码框架，用于调试核心算法：

```cpp
#include <small_gicp/registration/registration.hpp>
#include <small_gicp/factors/gicp_factor.hpp>
#include <small_gicp/factors/robust_kernel.hpp>

// 示例：基于 GN 和 Huber 核的点云配准测试
int main() {
    // 1. 准备源点云和目标点云 (例如 pcl::PointCloud<pcl::PointXYZ> 转换而来)
    // 2. 构建目标点云的 KdTree (small_gicp 内部数据结构)
    
    // 3. 实例化配准器 (选择优化器和鲁棒核)
    using RegGN = small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP, small_gicp::NullFactor, small_gicp::DistanceRejector, small_gicp::GaussNewtonOptimizer>;
    
    RegGN register_gn;
    register_gn.reduction.num_threads = 4; // 根据 R2 算力设置
    register_gn.rejector.max_dist_sq = 1.0;
    register_gn.optimizer.max_iterations = 30;
    
    // 4. 执行配准
    Eigen::Isometry3d initial_guess = Eigen::Isometry3d::Identity();
    auto result = register_gn.align(target_cloud, source_cloud, target_tree, initial_guess);
    
    // 5. 打印调试结果
    std::cout << "Converged: " << result.converged << std::endl;
    std::cout << "Iterations: " << result.iterations << std::endl;
    std::cout << "Error: " << result.error << std::endl;
    std::cout << "Transformation: \n" << result.T_target_source.matrix() << std::endl;
    
    // 重点分析 Hessian 矩阵 (用于评估退化)
    std::cout << "Hessian (H): \n" << result.H << std::endl;
    
    return 0;
}
```

## 3. 性能剖析与资源监控
由于 `small_gicp` 深度依赖 CPU 计算（特别是 OpenMP 多线程），在 R2 的 Qualcomm QCS8550 平台上运行时，监控 CPU 使用情况非常关键。

### 3.1 监控系统负载
在运行依赖 `small_gicp` 的节点时，通过 `htop` 观察各个 CPU 核心的负载均衡情况：
```bash
htop
```
如果发现某些核心负载极高，而其他核心空闲，说明 OpenMP 的线程分配可能存在问题，可以尝试显式设置环境变量限制线程数：
```bash
export OMP_NUM_THREADS=4
# 然后再运行 ROS 2 节点或测试程序
```

### 3.2 使用 Valgrind (Callgrind) 进行性能分析
（仅限深度调试时使用，这会显著降低运行速度）
```bash
valgrind --tool=callgrind ./test_gicp
# 运行结束后，使用 kcachegrind 查看性能瓶颈
kcachegrind callgrind.out.xxxx
```

## 4. 关键指标调试
在配准算法中，有几个关键参数直接影响结果：

- **Hessian 矩阵 (result.H)**：如果配准成功，Hessian 应该是一个正定矩阵。如果在某个方向（如长走廊的 X 轴方向）Hessian 的对应特征值非常小（接近 0），说明在这个方向上存在退化，位姿估计不可靠。可以通过打印特征值来调试环境的退化情况。
- **目标点拒绝距离 (max_dist_sq)**：设置过大容易引入噪点，设置过小可能导致初始偏差稍大时无法收敛。调试时可通过打印 `result.num_inliers` 观察有效点数的变化。
- **迭代次数 (iterations)**：正常情况下配准应该在 10-20 次内收敛。如果持续达到 `max_iterations`，通常意味着陷入了局部最优或初始位姿偏差过大。

## 5. 常见问题排查

- **测试程序编译时报找不到头文件或库**：确认已完成 `colcon build --parallel-workers 1 --packages-select rc26_small_gicp`，并在当前终端执行 `source install/setup.bash` 后再编译依赖示例。
- **`result.converged` 持续为 `false`**：优先减小初始位姿误差，并检查 `max_dist_sq` 是否过小、点云重叠区域是否足够，同时关注 `result.num_inliers` 是否明显偏低。
- **CPU 占用过高或线程打满**：先限制 `OMP_NUM_THREADS`，再观察 `htop` 中各核心是否均衡；在 R2 平台联调时不建议默认放开过多 OpenMP 线程。
