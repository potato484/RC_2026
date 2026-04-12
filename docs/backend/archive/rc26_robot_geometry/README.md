# rc26_robot_geometry

## 模块定位

`rc26_robot_geometry` 是 R2 当前统一的机器人几何配置真源，负责提供车体轮廓与基础安全包络 profile，不负责规划算法、控制求解或机构状态机。

## 当前实现

- 当前是 config-only 包：
  - [config/r2_body_geometry.yaml](/home/potato/RC_2026/src/rc26_robot_geometry/config/r2_body_geometry.yaml)
  - YAML 已补充中文内联注释，直接说明 profile 字段的单位、运行时用途与约束关系，便于后续在不翻源码的前提下维护几何口径
- 当前提供 `compact` profile，包含：
  - `body.half_length_m`
  - `body.half_width_m`
  - `body.height_m`
  - `safety.stop_envelope_half_width_m`
  - `planning.surface_projection_radius_m`

## 当前接口

- 不直接发布 topic / service / action
- 通过参数契约接入其他包：
  - `robot_geometry_file`
  - `robot_geometry_profile`

## 当前消费方

- `rc26_xhu_nav`
  - 读取 geometry profile，并同时用于：
    - `planning.surface_projection_radius_m` 的 surface route 投影锚定半径
    - body-aware surface planning 的最小 lateral clearance 约束
    - `surface_planner_backend=body_planner` 时独立 body planner 的 `half_length_m / half_width_m` 输入
    - surface route 失败分诊里对 `*_POINT_BLOCKED_BY_BODY_CONSTRAINT / BODY_CONSTRAINT_UNSATISFIED` 的几何约束判断
  - 读取 geometry profile，并用 `safety.stop_envelope_half_width_m` 约束 runtime 局部地形风险采样宽度

## 当前边界

- 负责共享几何真源
- 不负责底盘运动学模式选择
- 不负责机构展开态实时发布
- 不负责 full-body 3D collision checking

## 当前口径

- 这个包的引入是一次架构级补强：把“机器人轮廓常量”从散落在导航/控制参数里的隐式事实，收敛成一个可被多个包消费的共享配置真源
- 当前静态 `compact` profile 已经被 `rc26_xhu_nav` 用于离线 body-aware graph 注解、运行时 body-aware overlay，以及 body-aware 全车体全局搜索；它不再只是投影半径脚手架。
- 当前仍只有静态 profile，未来如果机构展开态会改变整车轮廓，应由机构状态提供方额外发布“当前 profile / 当前轮廓状态”，再由规划与控制消费；不应把机构状态机直接塞进本包
