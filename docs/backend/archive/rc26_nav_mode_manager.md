# rc26_nav_mode_manager

## 模块定位

`rc26_nav_mode_manager` 是 R2 的导航安全模式管理器，负责在不同风险场景下切换导航参数档位，并协调地形感知参数随之同步变化。

## 当前实现

- 构建方式：共享库组件
- 导出节点：
  - `nav_mode_manager_node`
  - `terrain_mode_adapter_node`
- 关键配置：
  - `config/nav_mode_manager.yaml`
  - `config/nav_profiles.yaml`
  - `config/terrain_profiles.yaml`
- 启动文件：`launch/nav_mode_manager.launch.py`

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

## 模块边界

- 它不做路径规划，只做“模式”和“参数”层的切换
- 它不代替 Nav2 控制器实现本身
- 它也不直接产出地形感知结果，只负责协调感知参数与导航策略一致
