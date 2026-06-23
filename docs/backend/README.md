# 后端归档文档索引

`docs/backend/archive/` 采用“按当前 ROS2 包保留入口文档，必要时补充少量历史资料”的结构。

仓库根目录的集中式调试文档已经删除。后续排查和验证入口以各包 README、launch 文件、包内脚本和实机操作记录为准，不再维护第二套根目录调试手册。

## 当前导航口径

R2 运行时导航权威已经迁移到 Nav2。`rc26_bringup` 在 `slam=false` 时装配 `map_server + nav2_bringup/navigation_launch.py`，`/cmd_vel` 由 Nav2 controller/velocity_smoother 输出。

`rc26_decision` 不再发送自定义导航 action，而是在 BT XML 中显式写 Nav2 pose 目标，通过 `NavToPose` 节点调用 `/navigate_to_pose`。

`rc26_merge_odom` 已从当前 R2 默认运行装配中停用但保留源码：`rc26_bringup`、导航联调入口和遥控脚本不再启动它，也不再把 `/merge_odom` 作为当前运行时契约。`/cmd_vel` 的默认硬件消费方由 `rc26_mcu_transport` 提供，它复用目标 MCU 串口下发 `POSE_TARGET(0x1F)`；机构指令共享串口 provider 同样由 `rc26_mcu_transport` 提供。

`rc26_terrain`、`rc26_base_ground` 与 `rc26_kfs_keepout` 已从当前 R2 主运行时链路中归档退出：默认 CMake 不生成运行时目标，`rc26_bringup` 不启动它们，`rc26_decision` 不订阅或发布它们的数据，默认运行和手动验证闭包也不再纳入这些归档链路。

## 当前验证口径

- 仓库级 CI/CD 已删除，当前不再维护 GitHub Actions workflow 或本地 CI smoke 脚本。
- ROS2 包级验证继续按仓库统一命令手动执行：

```bash
MAKEFLAGS='-j2 -l2' colcon build --symlink-install --executor sequential --parallel-workers 1 --packages-select <pkg...>
```

- 机器人运行时仍以犀牛派 X1 / AidLux 实机环境和 `rc26_bringup` 装配入口为准。

## 当前 IDE 索引入口

- `scripts/dev/refresh-compile-commands.sh`：当前工作区统一的 C/C++ 编译数据库刷新入口。
- 仓库内可追踪的 VS Code C/C++ 配置当前以 `.vscode/c_cpp_properties.json` 为入口，统一指向仓库根目录 `compile_commands.json`。
- 如果清理过 `build/ install/`，或新增 / 重命名了 C++ 源文件与包，需重新执行一次该脚本，避免跳转仍落在过期声明或直接跳转失败。

## 包目录索引

### 装配与决策

- [`rc26_bringup`](archive/rc26_bringup/README.md): R2 整车链路统一装配入口；导航模式下装配定位、Nav2 基础导航栈和决策。`(file: archive/rc26_bringup/README.md)`
- [`rc26_decision`](archive/rc26_decision/README.md): R2 主决策包；内部行为树通过 `NavToPose` 调用 Nav2 `/navigate_to_pose`，不再对外暴露 BT 调试 topic/service。`(file: archive/rc26_decision/README.md)`
- [`rc26_interfaces`](archive/rc26_interfaces/README.md): R2 自定义 ROS 2 接口契约包；当前保留机构、视觉和少量归档接口定义，定位主链改用标准 ROS 消息与 TF。`(file: archive/rc26_interfaces/README.md)`

### 里程计、定位与点云主链

- [`rc26_mid360_driver`](archive/rc26_mid360_driver/README.md): Livox Mid-360 驱动。`(file: archive/rc26_mid360_driver/README.md)`
- [`rc26_sensor_extrinsics`](archive/rc26_sensor_extrinsics/README.md): R2 静态传感器安装外参 YAML 真源。`(file: archive/rc26_sensor_extrinsics/README.md)`
- [`rc26_point_lio`](archive/rc26_point_lio/README.md): LiDAR-Inertial Odometry 主链。`(file: archive/rc26_point_lio/README.md)`
- [`rc26_localization`](archive/rc26_localization/README.md): 激光重定位主模块，继续作为 `map -> odom` 权威。`(file: archive/rc26_localization/README.md)`
- [`rc26_merge_odom`](archive/rc26_merge_odom/README.md): 已停用的底盘局部反馈、位姿下发和目标 MCU 串口桥接源码，保留给历史参考和手工调试，不再属于默认运行链。`(file: archive/rc26_merge_odom/README.md)`
- [`rc26_odom_interface`](archive/rc26_odom_interface/README.md): 上游里程计到下游统一底盘坐标系的接口层。`(file: archive/rc26_odom_interface/README.md)`
- [`rc26_sensor_scan`](archive/rc26_sensor_scan/README.md): 点云与里程计时空对齐模块。`(file: archive/rc26_sensor_scan/README.md)`
- [`rc26_small_gicp`](archive/rc26_small_gicp/README.md): 点云配准基础库。`(file: archive/rc26_small_gicp/README.md)`

### 控制与执行

- [`rc26_mechanism`](archive/rc26_mechanism/README.md): 机构执行与生命周期管理。`(file: archive/rc26_mechanism/README.md)`
- [`rc26_mcu_transport`](archive/rc26_mcu_transport/README.md): 目标 MCU 共享串口 owner，提供 `/mechanism/send_command`、`/mechanism/command_feedback` 与默认 `/cmd_vel` 到 `POSE_TARGET` 的底盘执行。`(file: archive/rc26_mcu_transport/README.md)`
- [`rc26_telecontrol`](archive/rc26_telecontrol/README.md): 人工遥控测试包。`(file: archive/rc26_telecontrol/README.md)`
- [`rc26_serial`](archive/rc26_serial/README.md): 串口通信基础库。`(file: archive/rc26_serial/README.md)`

### 归档地形与规则安全

- [`rc26_base_ground`](archive/rc26_base_ground/README.md): 已归档的基础标高与离散层级估计源码；默认不编译运行时目标。`(file: archive/rc26_base_ground/README.md)`
- [`rc26_kfs_keepout`](archive/rc26_kfs_keepout/README.md): 已归档的 KFS keepout 融合源码；默认不编译运行时目标，主链不再调用其服务或订阅其输出。`(file: archive/rc26_kfs_keepout/README.md)`
- [`rc26_terrain`](archive/rc26_terrain/README.md): 已归档的地形感知与语义栅格源码；默认不编译运行时目标，主链不再消费其话题。`(file: archive/rc26_terrain/README.md)`

### 感知与可视化

- [`rc26_vision`](archive/rc26_vision/README.md): 视觉推理与端头定位。`(file: archive/rc26_vision/README.md)`
