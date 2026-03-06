# rc26_interfaces

RC2026 机器人自定义 ROS2 接口包，包含消息 (msg)、服务 (srv) 和动作 (action) 定义。

## 1. 消息 (Messages)

### 机构与硬件状态
* **`MechanismState.msg`**
  * 描述下位机/执行机构的实时状态。
  * 关键字段：
    * `tip_state`: 弹头状态
    * `hal_open`: 硬件层是否就绪
    * `locked_tip_slot`: 当前锁定的弹槽 ID
    * `assembled_count`: 已组装完成的数量
    * `comm_health_level`: 通信健康度等级

### 感知与环境 (KFS/Tip/Terrain)
* **`MfKfsState.msg`**
  * 描述矿石/兑换站 (KFS) 的整体状态。
  * 包含 `MfKfsCell[]` 数组。
* **`MfKfsCell.msg`**
  * 单个矿石/格子单元信息。
  * 字段: `grid_id`, `kfs_type`, `confidence`。
* **`TipDetection.msg`**
  * 视觉/传感器检测到的弹头信息。
  * 字段: `tip_index`, `position_map` (地图系坐标), `depth_m`, `spacing_valid`。
* **`TipDetectionArray.msg`**
  * 包含多个 `TipDetection` 的数组。
* **`TerrainFeatureGrid.msg`**
  * 地形特征网格数据，用于路径规划和可通行性分析。
  * 包含高度 (`h_ground`, `h_top`)、坡度 (`slope_x`, `slope_y`)、粗糙度 (`roughness`)、障碍物概率 (`p_obstacle`) 等层级数据。

### 导航与决策
* **`SmartWaypoint.msg`**
  * 智能导航目标点，包含行为决策信息。
  * 字段：
    * `pose`: 目标位姿
    * `strategy_tag`: 策略标签 (如 "attack", "retreat")
    * `tolerance`: 容差设置 (`NavTolerance`)
    * `nav_safety_mode`: 导航模式 (0=NORMAL, 1=MF_SAFE, 2=MF_TRAVERSE, 3=MF_EXIT)
    * `speed_profile`: 速度配置
* **`NavTolerance.msg`**
  * 导航到达判定容差。
  * 字段: `xy_tolerance`, `yaw_tolerance`。
* **`NavSafetyState.msg`**
  * 导航模块的安全状态反馈。
  * 字段: `current_profile`, `stop_required`, `reason`。

## 2. 服务 (Services)

* **`SetNavMode.srv`**
  * 动态切换导航模式或配置。
  * **Request**: `profile` (配置名), `timeout`, `reason`
  * **Response**: `success`, `message`

## 3. 动作 (Actions)

用于长时间运行的任务，支持反馈和取消。

* **`AssembleWeapon.action`**
  * 触发武器组装流程。
  * **Result**: `success`, `error_code`
* **`ExecuteMechanism.action`**
  * 执行通用机构指令。
  * **Goal**: `command_id`, `payload`, `timeout_sec`
* **`GrabTip.action`**
  * 执行抓取弹头任务。
  * **Goal**: `tip_index` (目标弹头索引)
* **`PlaceKFSGrid.action`**
  * 执行放置 KFS 网格任务。
  * **Goal**: `grid_position`, `layer`

## 构建依赖
* `builtin_interfaces`
* `geometry_msgs`
* `std_msgs`
* `action_msgs`
