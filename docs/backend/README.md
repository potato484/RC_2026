# 后端归档文档索引

`docs/backend/archive/` 采用“按当前 ROS2 包保留入口文档，必要时补充少量历史资料”的结构。

仓库根目录的集中式调试文档已经删除。后续排查和验证入口以各包 README、launch 文件、包内脚本和实机操作记录为准，不再维护第二套根目录调试手册。

## 当前导航口径

R2 运行时导航权威已经迁移到 `rc26_decision` 内部 odom 单轴分段闭环导航。默认导航链不再启动旧外部地图定位、地图服务、路径规划、底盘控制、速度平滑或传感器扫描链路。

`rc26_decision` 注册三个可复用导航 BT 动作：`OdomDriveX`、`OdomDriveY`、`OdomTurnToYaw`。X/Y 动作在进入动作时捕获当前 `/odom` 位姿和 yaw，分别只沿车体系 X 或 Y 单轴闭环发布 `/cmd_vel.linear.x/y`，并用 `angular.z` 保持进入该段时的 yaw；转向动作只按绝对 odom yaw 闭环发布 `angular.z`。当前不再提供或调用旧外部位姿导航 action，也不保留兼容 BT 节点。

`rc26_bringup run_mode:=navigation` 只装配 `rc26_mcu_transport`、`odometry.launch.py start_sensor_scan:=false`、`rc26_decision`，以及按需 RealSense。`/cmd_vel` 由决策侧导航/动作节点串行发布，默认硬件消费方由 `rc26_mcu_transport` 提供并下发 `POSE_TARGET(0x0C)`；同一时刻不得启动遥控、测试动作或其它 `/cmd_vel` 发布者。

`rc26_merge_odom` 已从当前 R2 默认运行装配中停用但保留源码：`rc26_bringup`、导航联调入口和遥控脚本不再启动它，也不再把 `/merge_odom` 作为当前运行时契约。`/cmd_vel` 的默认硬件消费方由 `rc26_mcu_transport` 提供，它复用目标 MCU 串口下发 `POSE_TARGET(0x0C)`；机构指令共享串口 provider 同样由 `rc26_mcu_transport` 提供。

旧地形、base-ground 与 MF keepout 包已经从工作区删除，相关 keepout / terrain / MF KFS 兼容接口也不再由 `rc26_interfaces` 生成。当前 `rc26_bringup` 不启动这些链路，`rc26_decision` 不订阅、发布或调用它们，默认运行和手动验证闭包也不再包含这些历史包。

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

- [`rc26_bringup`](archive/rc26_bringup/README.md): R2 整车链路统一装配入口；导航模式下只装配 odom、MCU transport、决策和按需 RealSense。`(file: archive/rc26_bringup/README.md)`
- [`rc26_decision`](archive/rc26_decision/README.md): R2 主决策包；内部行为树通过 `OdomDriveX`、`OdomDriveY`、`OdomTurnToYaw` 执行 odom 单轴分段导航，不再对外暴露 BT 调试 topic/service。`(file: archive/rc26_decision/README.md)`
- [`rc26_interfaces`](archive/rc26_interfaces/README.md): R2 自定义 ROS 2 接口契约包；当前保留机构、视觉和动态预测接口，定位主链改用标准 ROS 消息与 TF。`(file: archive/rc26_interfaces/README.md)`

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

### 感知与可视化

- [`rc26_vision`](archive/rc26_vision/README.md): 视觉推理与端头定位。`(file: archive/rc26_vision/README.md)`
