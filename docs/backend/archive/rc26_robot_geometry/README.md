# rc26_robot_geometry

## 模块定位

`rc26_robot_geometry` 是 R2 当前统一的机器人几何配置真源，负责提供车体轮廓与基础安全包络 profile，不负责规划算法、控制求解或机构状态机。

## 当前实现

- 当前是 config-only 包：
  - [config/r2_body_geometry.yaml](/home/potato/RC_2026/src/rc26_robot_geometry/config/r2_body_geometry.yaml)
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

## 当前边界

- 负责共享几何真源
- 不负责底盘运动学模式选择
- 不负责机构展开态实时发布
- 不负责 full-body 3D collision checking

## 本轮说明

基础 Nav2 迁移后，本包不再被旧导航运行时消费。当前仍保留 config-only 几何真源，供后续需要车体包络的规划、控制或安全模块复用。
