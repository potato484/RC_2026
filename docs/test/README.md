# 测试脚本入口

本目录收口根仓库的前端预演、浏览器 E2E 和 release 打包脚本。

当前与 `rc26_xhu_viewer` Web 工具链直接相关的入口是：

- `docs/test/local-preflight.sh`
- `docs/test/e2e/run-e2e-local.sh`
- `docs/test/release/package-release.sh`
- `docs/test/release/deploy-via-ssh.sh`

## 当前范围

- `local-preflight.sh`：执行 `rc26_xhu_viewer` Web 前端的 `npm test / build`、Python adapter 语法检查、E2E 和 release 打包
- `e2e/`：使用契约 stub 启动浏览器链路，验证 `src/rc26_xhu_viewer/rc26_xhu_viewer/viewer`
- `release/`：打包 `rc26_xhu_viewer` Web 前端与 adapter 发布目录

## 文档索引

- [preflight/README.md](preflight/README.md)
- [e2e/README.md](e2e/README.md)
- [release/README.md](release/README.md)
