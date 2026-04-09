# Foxglove Legacy Assets

`rc26_bringup` 已不再通过 `visualization_backend:=foxglove` 启动可视化后端。当前 bringup 的主入口已经切到：

- `visualization_backend:=rviz2`
- `visualization_backend:=none`

本目录下的 `operator.json`、`engineering.json`、`diagnostic.json` 仅作为历史布局模板保留，方便离线对照 topic 组合，不再参与默认安装、launch 参数生成或主链路值守。

如果需要现场继续手工使用 Foxglove，应当把这些 JSON 当作独立参考资产，而不是认为它们仍然是 `rc26_bringup` 当前维护的可视化真入口。
