# 测试脚本入口

本目录当前只收口 `merlin-bt-visualizer` 的测试和打包脚本。

当前推荐入口是：

- `docs/test/merlin_bt_visualizer/local-preflight.sh`
- `docs/test/merlin_bt_visualizer/run-e2e-local.sh`
- `docs/test/merlin_bt_visualizer/package-release.sh`

## 当前范围

- `merlin_bt_visualizer/`：行为树工作台的本地预演、E2E 与 release 打包
- ROS2 运行时包的验收说明继续写在各包 README、launch 文档和 `src/` 子目录里，不再由 `docs/test/` 维护第二套 viewer 历史入口

## 文档索引

- [merlin_bt_visualizer/README.md](merlin_bt_visualizer/README.md)
