# 先验点云目录

此目录用于存放定位模块所需的先验点云文件（`.pcd`）。

## 使用说明

1. 将建图阶段生成的点云地图放置于此目录；
2. 启动时指定参数：`prior_pcd_file:=<path_to_your.pcd>`。

## 默认行为

仓库内提供了一个最小化烟测地图：`default.pcd`。

- 该文件仅用于 launch 冒烟和接口自检，不代表比赛地图质量；
- 真机联调和比赛必须替换为现场标定后的真实地图。

## 推荐建图与保存流程

推荐使用 `mapping_dense` profile 建图，它会：

- 保留较高点云密度；
- 持续发布累计地图 `/laser_map_full`，方便可视化确认“之前建好的内容仍在”；
- 默认开启 PCD 保存。

```bash
# 完整建图链路（保留 terrain 等附加模块）
ros2 launch rc26_bringup bringup.launch.py \
  slam:=true \
  point_lio_profile:=mapping_dense \
  use_decision:=false
```

如果当前目标只是纯建图、录制 PCD、查看累计地图，不需要地形语义与决策，可启用更轻量的纯建图模式：

```bash
# 纯建图最小链路（跳过 rc26_terrain / rc26_decision）
ros2 launch rc26_bringup bringup.launch.py \
  slam:=true \
  pure_mapping_mode:=true \
  point_lio_profile:=mapping_dense
```

说明：

- `pure_mapping_mode` 仅在 `slam:=true` 时生效；
- 该模式仍会保留建图必需的 Point-LIO、`odom_interface` 与 `map -> odom` 静态变换；
- 该模式会额外跳过 `lio_state_predictor`，避免在高密建图时因上游延迟产生持续 stale 告警；
- `odometry.launch.py` 默认还会强制 `odometry.publish_odometry_without_downsample:=false`，避免 `state_estimation` 比 `cloud_registered` 超前过多而触发丢云；
- 当前 bringup / odometry 链路默认就是 headless；如需检查累计地图，可手工运行 `rviz2 -d /home/potato/RC_2026/src/rc26_bringup/rviz/slam.rviz` 只读观察现有 topic。

建图完成后：

1. 使用 `Ctrl+C` 正常退出；
2. Point-LIO 会将累计地图写入 `src/rc26_point_lio/PCD/scans.pcd`；
3. 若 `pcd_save.interval > 0`，则会分段保存为 `scans_1.pcd`、`scans_2.pcd` 等。

补充说明：

- 建图得到的 PCD 默认可能包含地面点，这通常是正常现象，不表示建图失败；
- MID-360 本身具备向下观测能力，而 Point-LIO 默认不会主动删除地面；
- 如果只是希望导出的 PCD 更干净，可在建图时启用 `output_filter.world_z_filter_en`，并调节 `output_filter.world_z_min` 只裁剪输出地图，不影响内部里程计地图。

## 作为定位先验地图使用

```bash
ros2 launch rc26_bringup bringup.launch.py \
  slam:=false \
  prior_pcd_file:=${RC26_WS:-$HOME/RC_2026}/src/rc26_point_lio/PCD/scans.pcd \
  use_decision:=false
```

也可以将生成的 PCD 复制到本目录，例如：

```bash
cp ${RC26_WS:-$HOME/RC_2026}/src/rc26_point_lio/PCD/scans.pcd \
   ${RC26_WS:-$HOME/RC_2026}/src/rc26_bringup/pcd/site_a_2026_03_09.pcd
```

然后在启动时引用：

```bash
ros2 launch rc26_bringup bringup.launch.py \
  slam:=false \
  prior_pcd_file:=${RC26_WS:-$HOME/RC_2026}/src/rc26_bringup/pcd/site_a_2026_03_09.pcd \
  use_decision:=false
```

## 生成点云地图的其他来源

可使用以下方式生成先验点云：

1. Point-LIO 建图后保存；
2. FAST-LIO 等其他 SLAM 建图后导出；
3. 第三方工具转换（如 CloudCompare）。

推荐格式：Binary 或 ASCII PCD，至少包含 `XYZ` 字段。
