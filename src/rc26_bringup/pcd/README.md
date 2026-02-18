# 先验点云目录

此目录用于存放定位模块所需的先验点云文件 (.pcd)。

## 使用说明

1. 将建图阶段生成的点云地图放置于此目录
2. 启动时指定参数: `prior_pcd_file:=<path_to_your.pcd>`

## 默认行为

若未指定 `prior_pcd_file` 参数或文件不存在：
- `sentry_localization` 将输出警告并跳过定位功能
- 系统仍可运行，但 `map -> odom` TF 不会发布

## 生成点云地图

可使用以下方式生成先验点云:
1. SLAM 建图后保存 (Point-LIO / FAST-LIO 等)
2. 第三方工具转换 (如 CloudCompare)

推荐格式: ASCII 或 Binary PCD, 含 XYZ 字段
