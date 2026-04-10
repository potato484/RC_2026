# rc26_xhu_viewer_status

## 当前状态

`src/rc26_xhu_viewer/rc26_xhu_viewer_status/` 已于 2026-04-10 随整棵 `src/rc26_xhu_viewer/` 删除，不再是当前工作区源码。

## 历史职责

- `r2/xhu_viewer/summary`
- `r2/xhu_viewer/operator_status`
- `r2/xhu_viewer/events`

它曾负责把 localization、controller、keepout、terrain、navigation runtime 和 mechanism 的技术状态聚合成统一的操作员语义。

## 当前口径

- `rc26_bringup` 已不再装配 `rc26_xhu_viewer_status_node`
- 当前工作区默认不再发布 `r2/xhu_viewer/*`
- 如果后续需要重新引入操作员语义聚合，应按新的边界重新设计和文档化，而不是默认恢复这个已删除包
