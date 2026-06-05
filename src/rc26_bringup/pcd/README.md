# 先验点云目录

此目录用于存放定位模块所需的先验点云文件（`.pcd`）。

## 使用说明

1. 将建图阶段生成的点云地图放置于此目录；
2. 启动定位或导航链路时指定 `prior_pcd_file:=<path_to_your.pcd>`。

## 默认行为

仓库内提供了一个最小化烟测地图：`default.pcd`。

- 该文件仅用于 launch 冒烟和接口自检，不代表比赛地图质量；
- 真机联调和比赛必须替换为现场标定后的真实地图。

## 建图与保存流程

Point-LIO 当前只有单配置启动口径。默认 `src/rc26_point_lio/config/mid360.yaml` 已开启 PCD 保存：

```yaml
pcd_save:
  pcd_save_en: true
  interval: -1
```

直接启动建图链路即可在正常退出后导出单个 PCD：

```bash
ros2 launch rc26_bringup test_mapping.launch.py
```

如需使用自定义完整 Point-LIO YAML，则通过上层参数传入：

```bash
ros2 launch rc26_bringup test_mapping.launch.py \
  point_lio_config_file:=/abs/path/to/point_lio_mapping.yaml
```

说明：

- `test_mapping.launch.py` 固定启用 `slam:=true` 和 `pure_mapping_mode:=true`，默认打开 RViz2；
- Point-LIO 不再通过 profile 覆盖配置，也不会自动读取定位先验地图；
- `odometry.launch.py` 默认强制 `odometry.publish_odometry_without_downsample:=false`，保持 `/state_estimation` 与 `/cloud_registered` 时间戳同源；
- 当前 RViz 预设观察 `/point_lio/map_cloud` 完整累计地图、`/registered_scan` 实时点云与 `/Laser_map` 初始地图。

建图完成后：

1. 使用 `Ctrl+C` 正常退出；
2. 默认 `pcd_save.pcd_save_en=true` 且 `pcd_save.interval=-1`，Point-LIO 会将累计点云写入 `src/rc26_point_lio/PCD/scans.pcd`；
3. 若 `pcd_save.interval > 0`，则会分段保存为 `scans_1.pcd`、`scans_2.pcd` 等。

补充说明：

- 建图得到的 PCD 默认可能包含地面点，这通常是正常现象，不表示建图失败；
- MID-360 本身具备向下观测能力，而 Point-LIO 默认不会主动删除地面；
- 若需要清理地图，请在 PCD 后处理或定位地图制作环节处理，不要把车身 ROI 当作大范围地面过滤器。

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
3. 第三方工具转换，如 CloudCompare。

推荐格式：Binary 或 ASCII PCD，至少包含 `XYZ` 字段。
