# rc26_nav_mode_manager

## 模块定位

`rc26_nav_mode_manager` 是 R2 的导航/运动模式管理宿主包，当前同时承载 legacy Nav2 模式管理链路和 `xhu_direct` 的独立运动模式管理链路。

## 当前实现

- 构建方式：共享库组件
- 导出节点：
  - `nav_mode_manager_node`
  - `terrain_mode_adapter_node`
  - `xhu_motion_mode_manager_node`
- 关键配置：
  - `config/nav_mode_manager.yaml`
  - `config/nav_profiles.yaml`
  - `config/terrain_profiles.yaml`
- 启动文件：`launch/nav_mode_manager.launch.py`、`launch/xhu_motion_mode_manager.launch.py`

当前实现分成两条运行时：

- legacy Nav2 模式管理：
  - `nav_mode_manager_node` + `terrain_mode_adapter_node`
  - 保持 `SetNavMode`、controller 参数下发、terrain profile 同步逻辑
- `xhu_direct` 运动模式管理：
  - `xhu_motion_mode_manager_node`
  - 提供 `set_xhu_motion_mode` 服务，发布 `/xhu_nav/motion_mode_state`
  - 使用 odom 停稳预检查和 watchdog 回退，不依赖 Nav2 参数服务与 local_costmap 清理

## 2026-04 `xhu_motion_mode_manager` 收口

- `xhu_motion_mode_manager` 已拆成独立 `hpp/cpp`，便于 direct 运行时继续扩展而不把实现细节埋在单文件 `main` 里。
- 停稳预检查已从固定 5 帧计数窗口改成时间窗语义，减少不同 odom 频率下“同样速度、不同判定”的漂移。
- 若直接模式误用了 legacy `nav_profiles.yaml`，管理器会自动补出 `hold / plane_move / ramp_up / ramp_down / mf_exit` 等 xhu 基础 profile 别名，避免 topo 图请求模式时因配置缺项直接失败。
- watchdog deadline 与模式状态更新现在统一走受控状态路径，保持 direct 模式下“动作模式/限速/watchdog”仍是单一权威出口。

源码职责划分比较清晰：

- `nav_mode_manager.cpp`
  - 模式切换主逻辑、服务处理、安全检查
- `profile_loader.cpp`
  - 读取配置文件中的模式定义
- `profile_db.cpp`
  - 维护已装载的 profile 数据
- `profile_executor.cpp`
  - 真正把参数下发到控制器/相关节点
- `terrain_mode_adapter.cpp`
  - 根据当前导航模式调整地形感知参数
- `watchdog_timer.cpp`
  - 复杂模式驻留超时与回退逻辑
- `xhu_motion_mode_manager.cpp`
  - `xhu_direct` 模式管理服务、停稳预检查、watchdog 回退与约束发布

## 源码入口与阅读顺序
- 先看 `launch/nav_mode_manager.launch.py` 与 `launch/xhu_motion_mode_manager.launch.py`，确认 legacy 与 `xhu_direct` 两条模式管理链如何启动。
- 再看 `src/nav_mode_manager.cpp` 与 `src/xhu_motion_mode_manager.cpp`，理解两套服务接口、watchdog 和 fallback 主路径。
- 然后看 `profile_loader.cpp`、`profile_executor.cpp`、`terrain_mode_adapter.cpp`，区分“读配置”“执行切换”“同步 terrain 参数”三层。
- 最后回到三个 YAML 和测试文件，确认 profile 数据和约束。

## 目录解剖
- `nav_mode_manager.cpp`：服务入口、watchdog、fallback 和状态广播。
- `profile_loader.cpp`：把 `nav_profiles.yaml` 读成内存结构，并做合法性检查。
- `profile_db.cpp`：保管已加载 profile。
- `profile_executor.cpp`：真正执行停稳预检查、costmap 清理、参数下发与回滚。
- `terrain_mode_adapter.cpp`：监听导航模式并异步把 terrain profile 打给 `rc26_terrain`。
- `watchdog_timer.cpp`：watchdog 辅助。
- `xhu_motion_mode_manager.cpp`：`xhu_direct` 模式服务入口、模式状态发布和 watchdog 回退。

## 关键文件体量
- `src/profile_executor.cpp`：437 行，切换执行器是核心复杂点。
- `src/terrain_mode_adapter.cpp`：350 行。
- `src/nav_mode_manager.cpp`：291 行。
- `src/xhu_motion_mode_manager.cpp`：373 行，`xhu_direct` 模式管理主实现。
- `config/nav_profiles.yaml`：157 行，模式数据真源。
- `config/terrain_profiles.yaml`：64 行。

## 关键源码行段速览
- `src/rc26_nav_mode_manager/src/nav_mode_manager.cpp:19-188`：管理器构造、watchdog、状态发布和 fallback 辅助。
- `src/rc26_nav_mode_manager/src/nav_mode_manager.cpp:189-291`：`/set_nav_mode` 服务处理。
- `src/rc26_nav_mode_manager/src/profile_loader.cpp:42-92`：profile 校验和环依赖检测；`93-226`：文件装载。
- `src/rc26_nav_mode_manager/src/profile_executor.cpp:52-121`：运行上下文和里程计停稳判断；`122-345`：预检查、costmap、参数下发与回滚；`363-437`：切换执行入口。
- `src/rc26_nav_mode_manager/src/terrain_mode_adapter.cpp:18-111`：构造与订阅；`112-291`：后台 worker、apply/retry/verify；`292-350`：safe mode 请求和 diagnostics。
- `src/rc26_nav_mode_manager/src/xhu_motion_mode_manager.cpp:112-173`：参数装载、`set_xhu_motion_mode` 服务和模式状态发布器初始化；`174-279`：停稳预检查与模式切换；`280-373`：watchdog fallback 与状态发布。

## 模块边界

- 它不做路径规划，只做“模式”和“参数”层的切换
- 它不代替 Nav2 控制器实现本身
- 它也不直接产出地形感知结果，只负责协调感知参数与导航策略一致
- `xhu_motion_mode_manager_node` 不访问 `controller_server` 参数服务，也不调用 local/global costmap 清理服务
