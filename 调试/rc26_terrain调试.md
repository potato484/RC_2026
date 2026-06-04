# rc26_terrain 调试

## 模块定位

`rc26_terrain` 已归档为 source-only 历史源码包。当前主链不编译它的运行时目标，不通过 bringup/odometry 启动它，也没有模块订阅它的输出。

本页仅用于显式恢复历史地形节点时的本地调试资料，不属于当前 R2 默认联调顺序。

## 适用场景

- 显式恢复归档目标后，排查历史 `/terrain_obstacles`、`/terrain_drop` 和 `/terrain_grid_map_local`
- 单独验证历史地形语义节点
- 复现历史 terrain 调参结论；不得把这些输出接回当前主链

## 前置条件

- 已 `source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"`
- 上游点云与 TF 正常
- 单独调试时建议先有 Bag 或真实雷达输入

## 标准编译

```bash
cd "${RC26_WS:-$HOME/RC_2026}"
MAKEFLAGS='-j2 -l2' colcon build --executor sequential --parallel-workers 1 \
  --packages-select rc26_interfaces rc26_terrain \
  --cmake-args -DRC26_ENABLE_ARCHIVED_RUNTIME_TARGETS=ON
source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"
```

## 推荐启动

单独启动：

```bash
ros2 launch rc26_terrain terrain_semantic.launch.py
```

当前整车链路不再提供 terrain 启动入口，也没有 `enable_terrain_grid_map` 参数。

## 最小验收

```bash
ros2 topic hz /terrain_obstacles
ros2 topic echo /terrain_obstacles --once
ros2 topic hz /terrain_drop
ros2 topic echo /terrain_drop --once
ros2 topic echo /terrain_features --once
```

如需外部观察：

```bash
rviz2 -d "${RC26_WS:-$HOME/RC_2026}/src/rc26_terrain/rviz/terrain_semantic.rviz"
```

## 常用在线调参

```bash
ros2 param set /terrain_semantic ground_ema_alpha 0.5
ros2 param set /terrain_semantic h_obstacle_m 0.35
ros2 param set /terrain_semantic h_drop_m 0.18
```

## 去天花板、保地形和台阶

这部分是我们现在最关心的调法，目标很明确：

- 把雷达打到天花板上的高点尽量去掉
- 地面、台阶顶面、台阶立面这些对机器人走路有用的结构要保住
- 先别乱动 Point-LIO 内部，避免把定位链路搞坏

说人话就是：

- 我们现在不是为了“让点云看起来更漂亮”
- 我们是为了“别让系统把天花板当成有用地形”
- 所以第一刀要砍在 `rc26_terrain`

为什么先改 `rc26_terrain`？

- 因为它本来就是做地形语义理解的
- 它关心的是“这个点对走路、跨台阶、导航有没有意义”
- 在这一层改，影响最小
- 在归档恢复调试中，改动只影响历史地形理解节点，不会直接去动 Point-LIO 内部匹配

### 先去哪里改

工作空间根目录是：

`/home/robocon/ros2_workspace`

我们第一步先改这个文件：

`2026_416/RC_2026/src/rc26_terrain/config/terrain_semantic.yaml`

你重点看这几行：

- 第 `40` 行：`min_rel_z_m: -1.5`
- 第 `41` 行：`max_rel_z_m: 0.5`
- 第 `42` 行：`dis_ratio_z: 0.1`
- 第 `136` 行：`top_z_max_delta_m: 0.7`

### 我们先从 `max_rel_z_m` 下手

建议先从 `max_rel_z_m` 下手，默认是 `0.5`，可以往 `0.25 ~ 0.35` 试。

最大可跨越台阶是 `200mm`，这个范围通常够保留台阶顶面，又能把天花板打掉。

你可以这么理解：

- 如果设 `0.50`，太宽松，很多高点会混进来
- 如果设 `0.20`，太狠，台阶顶面和有用立面也可能被误删
- `0.30` 往往是一个比较像“工程值”的中间点

所以第一轮最推荐你先把第 `41` 行改成：

```yaml
max_rel_z_m: 0.30
```

这个参数是什么意思？

说人话就是：

**一个点最多允许比机器人当前底盘高多少，还能算“有用地形”。**

超过这个高度，系统就更倾向于把它当成没必要关心的高点，比如：

- 天花板
- 顶部梁
- 抬头打到的高处结构

### 然后再看 `dis_ratio_z`

如果你把 `max_rel_z_m` 调到 `0.30` 以后，发现：

- 近处已经干净了
- 但远处高点还是会漏进来

那下一步就去改同一个文件里的第 `42` 行：

```yaml
dis_ratio_z: 0.1
```

建议第二步先试：

```yaml
dis_ratio_z: 0.05
```

这个参数是什么意思？

说人话就是：

**点离机器人越远，系统会稍微“放点水”，允许它高一点。**

代码逻辑在：

`2026_416/RC_2026/src/rc26_terrain/src/terrain_semantic_node.cpp`

大约第 `1891 ~ 1893` 行：

```cpp
const double r = std::sqrt(d2);
if (rel_z < (min_rel_z_m_ - dis_ratio_z_ * r) ||
    rel_z > (max_rel_z_m_ + dis_ratio_z_ * r))
    continue;
```

你不用死记代码，记一句话就行：

```text
允许上限 = max_rel_z_m + dis_ratio_z * 距离
```

举个例子，假设你设：

- `max_rel_z_m = 0.30`
- `dis_ratio_z = 0.10`

那么：

- 距离 `1m` 的点，允许高度上限 = `0.40m`
- 距离 `2m` 的点，允许高度上限 = `0.50m`
- 距离 `3m` 的点，允许高度上限 = `0.60m`

这就说明一个关键问题：

- 你虽然把近处上限压到 `0.30` 了
- 但远处其实还是被放宽了

所以：

- 如果近处已经干净了，但远处高点还漏，就优先减 `dis_ratio_z`
- 如果近处和远处都太高，就先动 `max_rel_z_m`

你可以这么记：

- `0.10`：偏宽松
- `0.05`：比较适合现在这个去天花板需求
- `0.03`：更狠，但要小心远处有用结构也可能少掉

### `top_z_max_delta_m` 是第二道保险

如果你把前两个参数调完了，还是发现有这种情况：

- 偶尔还是有少量天花板点漏进来
- 虽然漏得不多，但会把某个局部栅格“撑得很高”
- 看起来像某个格子被高点污染了

那就去改同一个文件里的第 `136` 行：

```yaml
top_z_max_delta_m: 0.7
```

建议先试：

```yaml
top_z_max_delta_m: 0.50
```

如果还觉得高，再试：

```yaml
top_z_max_delta_m: 0.40
```

这个参数是什么意思？

这个不是第一道删点过滤，而是第二道保险。

你可以把它理解成：

- 系统先估计这个格子的地面有多高
- 再估计这个格子的顶部有多高
- 如果顶部比地面高得太离谱，就直接按住，不让它再往上飞

对应代码在：

`2026_416/RC_2026/src/rc26_terrain/src/terrain_semantic_node.cpp`

大约第 `1088 ~ 1089` 行：

```cpp
if (top_z > ground_z + static_cast<float>(top_z_max_delta_m_)) {
    top_z = ground_z + static_cast<float>(top_z_max_delta_m_);
}
```

说人话就是：

**就算有少量脏点漏进来了，我也不让它把这一格的高度理解带飞。**

### 第一轮推荐你直接改成这样

直接去改文件：

`2026_416/RC_2026/src/rc26_terrain/config/terrain_semantic.yaml`

把这几个值改成：

```yaml
min_rel_z_m: -1.5
max_rel_z_m: 0.30
dis_ratio_z: 0.05
top_z_max_delta_m: 0.50
```

这组参数的意思很简单：

- `min_rel_z_m` 先不动，避免误伤低处边缘和跌落区域
- `max_rel_z_m = 0.30`，先把高处点往下压
- `dis_ratio_z = 0.05`，不要让远处高点放得太宽
- `top_z_max_delta_m = 0.50`，防止少量漏点把局部格子撑爆

### 改完以后怎么看效果

先看这些话题：

```bash
ros2 topic hz /terrain_obstacles
ros2 topic echo /terrain_obstacles --once
ros2 topic echo /terrain_climbable --once
ros2 topic echo /terrain_features --once
```

你主要盯这几件事：

- 天花板点不要再被当成有效障碍
- 地面附近结构还在
- 台阶顶面还在
- 不要出现“场地基本空了”或者“台阶也没了”

如果出现这些现象，按下面调：

- 近处和远处高点都很多：先继续减 `max_rel_z_m`
- 近处还行，远处高点还漏：先继续减 `dis_ratio_z`
- 偶尔局部格子高度被拉得很夸张：先继续减 `top_z_max_delta_m`
- 台阶顶面也被误删了：把 `max_rel_z_m` 从 `0.30` 回调到 `0.35`

### 如果还想让 RViz 里的点云也更干净

如果你调完 `rc26_terrain` 以后，发现：

- 地形理解已经正常了
- 但是 `/cloud_registered`
- 或者 RViz 里的累计点云
- 看起来还是很多高处点

那这时候再去改第二层：

`rc26_point_lio`

去改这个文件：

`2026_416/RC_2026/src/rc26_point_lio/config/mid360.yaml`

重点看这几行：

- 第 `81` 行：`world_z_filter_en: False`
- 第 `82` 行：`world_z_min: -10.0`
- 第 `83` 行：`world_z_max: 10.0`

可以先改成：

```yaml
output_filter:
    world_z_filter_en: True
    world_z_min: -10.0
    world_z_max: 1.5
```

这一层你可以理解成：

**拿一把固定高度的水平刀，直接把世界里太高的点切掉。**

所以它适合干：

- 清理 `/cloud_registered`
- 清理 RViz 显示
- 让输出点云更干净

但它不适合单独承担“只保留地面和台阶”这个任务。

因为它看的是**世界系绝对高度**，不是“相对机器人当前高度”。

说白了就是：

- `rc26_terrain`：负责“别让系统把天花板当成有用地形”
- `rc26_point_lio`：负责“顺手把输出点云里的高点切掉，让显示更干净”

## 优先排查

- 平地上障碍物误报太多：先调 `h_obstacle_m`。
- 一直没有跌落输出：先确认输入点云覆盖到了落差区域，再看 `h_drop_m`。
- 只想看历史 grid map：必须显式恢复归档构建目标后单独启动 `rc26_terrain`；当前总装链不再提供 grid map 开关。

## 相关入口

- [感知启动](./感知启动.md)
- [rc26_sensor_scan调试](./rc26_sensor_scan调试.md)
