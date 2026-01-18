# rc26_terrain Bag 录制/回放

本文件用于按规格书交付“Bag 录制/回放操作文档”，用于复现与调参。

## 录制（建议）

至少包含点云、里程计与 TF：

```bash
ros2 bag record \
  /registered_scan \
  /odom \
  /tf \
  /tf_static \
  /terrain_obstacles \
  /terrain_drop \
  /terrain_climbable
```

如你的系统话题不同，按 `terrain_semantic.yaml` 的参数/Remap 进行替换即可。

## 回放

```bash
ros2 bag play /path/to/bag --clock
```

回放时请在启动文件中设置 `use_sim_time:=true`：

```bash
ros2 launch rc26_terrain terrain_semantic.launch.py use_sim_time:=true
```

