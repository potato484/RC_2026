# rc26_nav_mode_manager

该模块是导航安全模式管理器（Nav Mode Manager），主要用于处理不同地形（如平地、楼梯、门槛等）下导航参数的动态安全切换，保证机器人在进入特定危险地形前，能够以正确的参数（速度、加速度、地形评估策略等）进行导航，并在超时或异常时安全降级。

## 模块功能说明

本模块基于 ROS 2 设计，核心功能包含以下两部分：

1. **导航策略与控制管理（NavModeManager）**
   - 监听和处理来自上层状态机（如 `SmartWaypointNavigator`）的模式切换请求。
   - 读取并解析 `nav_profiles.yaml` 配置文件，根据不同的模式名称（如 `normal`, `safe`, `stair_up`, `mf_traverse` 等）下发对应的导航参数。
   - 执行切换前安全检查（如要求机器人完全停稳后才能切换模式）。
   - 在关键地形切换时，自动清除局部代价地图（Local Costmap），避免历史点云残留导致误判。
   - 动态修改底层控制器（Controller Server）的限速和加速度配置（如最大线速度、最大角速度）。
   - 维持 Watchdog 超时监控机制，如果机器人在某复杂地形模式下驻留时间过长，则自动触发安全回退（Fallback）到降级模式，甚至触发紧急停止。

2. **地形感知参数适配（TerrainModeAdapter）**
   - 监听 `NavModeManager` 发布的当前导航安全状态。
   - 读取并解析 `terrain_profiles.yaml` 配置文件。
   - 动态向 `terrain_semantic` 节点下发地形感知识别相关的参数（如未知区域处理策略、前向盲区角度、最小障碍物面积等）。
   - 实现参数回读校验机制，确保地形节点已经成功应用了下发的参数，防止下发失败导致的安全隐患。
   - 采用后台工作线程异步下发参数，避免阻塞主线程的执行效率。

## 配置文件说明

所有的行为和参数都是通过 YAML 配置文件进行数据驱动：

- **`nav_mode_manager.yaml`**：模块的系统级配置，如各类超时时间、判断停止的速度阈值、代价地图节点名等。
- **`nav_profiles.yaml`**：定义所有的导航安全模式。每种模式包含：
  - `fallback_profile`：超时回退的降级模式。
  - `watchdog`：超时时间及超时后是否要求强制停止。
  - `precheck`：切换至该模式前是否要求机器人停稳。
  - `costmap`：切换时是否清空局部代价地图。
  - `controller`：该模式下的最大速度、加速度等限制。
- **`terrain_profiles.yaml`**：定义对应模式下地形节点 `terrain_semantic` 需要应用的分析参数，如 `jump_thresh_m`、`drop_forward_sector_deg` 等。

## 核心接口说明

- **服务提供 (Services Provided)**
  - `/set_nav_mode`：接收外部模式切换请求（名称 + 切换原因）。

- **发布的话题 (Published Topics)**
  - `/nav_safety_state`：以 1Hz 频率广播当前模块的安全状态，包含当前所处模式、切换原因、是否发生超时、是否要求紧急停止等信息。
  - `/diagnostics`：发布标准的 ROS 2 诊断信息，便于监控节点健康状态和切换异常。

- **订阅的话题 (Subscribed Topics)**
  - `/odom`：订阅里程计信息，用于判断机器人是否停稳以及平滑的速度过渡。

- **依赖的外部接口 (External Interfaces Used)**
  - 参数客户端：动态配置 `controller_server` 的限速参数，以及 `terrain_semantic` 的地形参数。
  - 服务客户端：调用 `local_costmap/clear_entirely_local_costmap` 清理局部代价地图。

## 适用场景

本模块适用于机器人在多层复合场景下的安全行驶。例如：
- 平整路面：使用 `normal` 模式，放宽限速，快速通行。
- 靠近楼梯/悬崖：下发 `safe` 模式，减速并切换地形节点的感知策略为保守模式。
- 爬楼梯过程：必须先停稳，清空代价地图，然后以极低速度和极高的地形灵敏度参数开启 `stair_up` 模式，并附带 30 秒的 Watchdog。若 30 秒未完成，则自动降级到 `safe_low` 进行停障等待或求援。
