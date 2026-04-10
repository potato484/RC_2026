# 测试脚本入口

本目录当前主要收口 `merlin-bt-visualizer` 的测试和打包脚本。

`src/rc26_xhu_viewer` 删除后，旧的 viewer preflight / E2E / release 入口已移除。

当前推荐入口是：

- `docs/test/merlin_bt_visualizer/local-preflight.sh`
- `docs/test/merlin_bt_visualizer/run-e2e-local.sh`
- `docs/test/merlin_bt_visualizer/package-release.sh`

## 当前范围

- `merlin_bt_visualizer/`：行为树工作台的本地预演、E2E 与 release 打包
- `e2e/`、`preflight/`、`release/` 下只保留被删除 viewer 链路的历史说明

## 文档索引

- [merlin_bt_visualizer/README.md](merlin_bt_visualizer/README.md)
