# rc26_mechanism

## 模块定位

`rc26_mechanism` 是 R2 的最小机构执行边界，负责把上层动作语义可靠地下发给下位机，并回传最小执行状态。

## 当前实现

- 构建方式：组件库 + 独立可执行
- 导出节点：`mechanism_server_node`
- 启动文件：`launch/mechanism.launch.py`

当前实现已经收口为“一个生命周期服务端 + 一份集中命令目录 + 一个真实共享串口 HAL”：

- `include/rc26_mechanism/nodes` + `src/nodes`
  - `mechanism_lifecycle_server.hpp/.cpp`
  - 生命周期管理、action 服务、取消/超时/结果收敛
- `include/rc26_mechanism/catalog` + `src/catalog`
  - `mechanism_command_catalog.hpp/.cpp`
  - `rc26_mechanism` 唯一的业务命令目录真源
  - 统一描述命令是否允许走 `/mechanism/run_command`
  - 统一描述 terminal success feedback 与默认 timeout
- `include/rc26_mechanism/runtime`
  - `command_context.hpp`
- `include/rc26_mechanism/hal/shared_serial` + `src/hal/shared_serial`
  - `shared_serial_mechanism_hal.hpp/.cpp`
  - 通过 ROS 2 service/topic 复用 `rc26_merge_odom` 或 `pose_sender_node` 已打开的目标 MCU 串口
- `include/rc26_mechanism/hal/contracts`
  - `i_mechanism_hal.hpp`
- `test/catalog` 与 `test/transport`
  - 命令目录回归与共享 transport 回归测试

## 当前对外接口

当前只保留 3 个动作入口：

- `/mechanism/grab_tip`
- `/mechanism/assemble_weapon`
- `/mechanism/run_command`

当前只保留 1 个状态 topic：

- `/mechanism/status`
  - `hal_open`
  - `last_error_code`
  - `current_cmd_id`

也就是说，机构节点不再发布端头状态机、锁定槽位、装配计数或通信健康统计。

## 当前业务命令口径

当前 mechanism 业务目录只保留 4 条命令：

- `GRAB_TIP`
- `ASSEMBLE_WEAPON`
- `GRAB_KFS`
- `PLACE_KFS_GRID`

其中：

- `GRAB_TIP` 与 `ASSEMBLE_WEAPON` 继续保留专用 action 入口
- `GRAB_KFS` 与 `PLACE_KFS_GRID` 继续走 `/mechanism/run_command`

`PlaceKFSGrid.action` 已经移除；如果上层要放置 KFS，当前应通过 `/mechanism/run_command` 下发 `PLACE_KFS_GRID + payload{grid_position, layer}`。

## 真实部署口径

当前真实部署已经不是“`rc26_mechanism` 自己独占默认目标口 `/dev/ttyUSB0`”，而是：

- 真机上由 `rc26_merge_odom` 作为目标 MCU 串口的唯一 owner
- `rc26_mechanism` 通过 `hal_type:=shared_serial` 复用这条链路
- 下行发送经由 `/mechanism/send_command`
- 上行反馈经由 `/mechanism/command_feedback`
- 当前只支持 `hal_type:=shared_serial`；其它 `hal_type` 会在 `configure` 阶段直接失败

串口职责速记：

- `feedback_serial_port` 是保留中的底盘反馈链入口，主要对应 `ODOM_DATA` 接收和 `POSE_FEEDBACK` 发送；当前默认值是 `__disabled__`，`rc26_mechanism` 不直接使用它
- `target_serial_port` 才是 mechanism 共享 transport 复用的真实物理链路，同时承载 `POSE_TARGET` 与机构/遥控 sidecar 命令；当前默认主口是 `/dev/ttyUSB0`
- 因此只要 `/mechanism/send_command` 或 `/mechanism/command_feedback` 异常，优先排查 `rc26_merge_odom` 当前是否由 `merge_odom_node` 或 `pose_sender_node` 成功持有 `target_serial_port`

## 当前运行时语义

- 只有节点处于 `active` 且 HAL 已打开时才接收 goal
- 同一时刻只允许一个机构动作执行
- 取消、退活或错误时会发送 `STOP` 并清退 pending context
- 继续保留 `pending_contexts_ + buffered_feedbacks_` 这套反馈收敛逻辑，处理：
  - 正常完成反馈
  - 早到成功反馈
  - 超时
- 当前 MCU 口径是不再返回 `ACTION_FAIL/ERROR`；因此 mechanism 侧不再把它们当作运行时判定分支，未收到命令专属完成反馈时统一按超时失败收敛

## 新增命令维护规则

如果想让“在 `rc26_serial` 增加协议定义后，`rc26_mechanism` 这边补一处就能把整条链路打通”，当前固定按下面的顺序维护：

1. 在 `rc26_serial/protocol.hpp` 增加新的 `CommandID` / `FeedbackID`
2. 在 `rc26_mechanism/catalog/mechanism_command_catalog.*` 增加命令目录项
3. 如果只需要通用执行，直接调用 action `/mechanism/run_command`
4. 只有在确实需要更强业务语义时，才新增专用 action 包装

命令目录项当前至少维护三件事：

- `execute_supported`
- `terminal_success_feedback_ids`
- `default_timeout`

## 源码入口与阅读顺序

- 先看 `launch/mechanism.launch.py`，确认当前只保留 `shared_serial` 装配。
- 再看 `include/rc26_mechanism/catalog` 与 `src/catalog/mechanism_command_catalog.cpp`，确认当前业务命令目录。
- 然后看 `include/rc26_mechanism/nodes` 与 `src/nodes/mechanism_lifecycle_server.cpp`，生命周期节点、action server 和反馈收敛都在这里。
- 最后看 `src/hal/shared_serial/shared_serial_mechanism_hal.cpp`，确认真实部署如何桥接 `rc26_merge_odom` 的共享串口。


## 模块边界

- 它不做比赛级决策，只执行机构动作
- 它不负责底盘导航控制
- 它不直接做视觉识别或地图处理，只消费上层语义并驱动硬件
