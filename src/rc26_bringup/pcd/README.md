# 先验点云目录

此目录用于存放定位模块所需的先验点云文件 (.pcd)。

## 使用说明

1. 将建图阶段生成的点云地图放置于此目录
2. 启动时指定参数: `prior_pcd_file:=<path_to_your.pcd>`

## 默认行为

仓库内提供了一个最小化烟测地图：`default.pcd`。
- 该文件仅用于 launch 冒烟和接口自检，不代表比赛地图质量。
- 真机联调和比赛必须替换为现场标定后的真实地图。

## 生成点云地图

可使用以下方式生成先验点云:
1. SLAM 建图后保存 (Point-LIO / FAST-LIO 等)
2. 第三方工具转换 (如 CloudCompare)

推荐格式: ASCII 或 Binary PCD, 含 XYZ 字段
