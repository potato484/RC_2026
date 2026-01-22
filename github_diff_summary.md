# 当前项目实现 vs GitHub(`origin/main`) 差异总结

## 对比基线
- 本地分支：`main`
- 远端分支：`origin/main`
- 当前 `HEAD` 与 `origin/main` 同一提交；差异来自 **工作区未提交改动（含新增/删除/修改）**。

## 核心差异（按模块）

### 1) `rc26_decision`：航点导航机制重构为 “Smart Waypoint”
- 替换旧实现：移除 `WaypointNavigator` / `WaypointCatalog` / `bt_navigate_waypoint`（对应头文件与 cpp 均删除）。
- 新增能力：
  - `WaypointManager`：从 YAML 加载语义航点（`config/waypoints/waypoints_{red,blue}.yaml`），并支持生成/注入“Merlin”拓扑点与邻接图。
  - `SmartWaypointNavigator`：围绕单次导航任务封装状态机，支持
    - 导航安全模式切换（`rc26_interfaces/srv/SetNavMode`）
    - 动态调整 controller 参数（通过参数客户端读取默认值/回滚）
    - 基于里程计的“停车判定”和超时控制
    - 向 MCU 下发导航模式/速度等指令（通过 `rc26_serial` 协议命令）
  - 行为树节点 `NavToSmartPoint`：黑板读取 `target_name`→查表→发起导航→tick 等待结果。
- `decision_node` 参数扩展：
  - `team`/`waypoints_file` 用于选择 YAML 航点配置
  - `controller_server_node`/`odom_topic`/停车阈值等用于导航执行与判定
  - 心跳从固定 1Hz 改为由 `heartbeat_rate_hz` 决定（<=0 时禁用）

### 2) `rc26_bringup`：启动链路与参数简化
- `bringup.launch.py`
  - 增加 `rc26_base_ground`（非建图模式）
  - 地图服务改为仅启动 `nav2_map_server` + lifecycle manager（避免与 `rc26_localization` 的 `map->odom` 冲突）
  - 决策系统从 include launch 改为直接启动 `rc26_decision/decision_node`（使用 `decision_params.yaml`）
  - 移除 `world`、`team` 等启动参数
- `odometry.launch.py`/`localization.launch.py`：去除 `world`/`slam` 等冗余参数与依赖逻辑，测试 launch 同步调整。
- `CMakeLists.txt`：不再安装 `behavior_trees`（目录与相关文件被删除）。

### 3) `rc26_interfaces`：新增消息契约
- 新增：
  - `msg/NavTolerance.msg`（xy/yaw 容差）
  - `msg/SmartWaypoint.msg`（Pose + strategy_tag + tolerance + safety_mode + speed_profile + timeout + 扩展 payload）
- `rosidl_generate_interfaces` 增加 `geometry_msgs` 依赖，并在 `package.xml` 补齐依赖。

### 4) 其他调整
- `rc26_base_ground/config/base_ground_estimator.yaml`：`odom_topic` 由绝对 `/odom` 改为相对 `odom`（兼容 namespace，如 `/r2/odom`）。
- 文档/测试清理：
  - `rc26_md/` 下多份旧文档删除（保留/更新了部分中文文档）
  - `rc26_bringup/test` README 更新为指向新的决策测试方式
  - 根目录 `CHANGELOG.md` 被删除

## 影响与注意事项
- 这次变更属于“功能重构 + 大量清理”，合并后会影响：
  - 决策行为树节点名称/端口（从旧 waypoint 节点到 `NavToSmartPoint`）
  - bringup 启动参数（`world/team/slam` 等不再可用或被收敛）
  - 航点配置来源（需提供 `waypoints_{team}.yaml` 或显式 `waypoints_file`）
- 建议在 push 前本地构建一次 `rc26_decision` 与 `rc26_interfaces` 以确认依赖（`yaml-cpp`、`geometry_msgs`）齐全。

