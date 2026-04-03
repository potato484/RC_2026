# 本地测试与预演入口

`docs/test/README.md` 现在作为 `docs/test/` 的入口页，用来快速回答两件事：

- 提交到 GitHub 前，当前仓库本地可以先做哪些 CI/CD/E2E 预测。
- 这些本地预测脚本和真实 GitHub Actions、浏览器 E2E、release 打包之间各自是什么关系。

更细的说明已经按模块拆到目录内的独立 `README.md`，避免把入口说明、脚本定位和执行细节继续混写在一个文件里。

## 1. 当前文档范围

`docs/test/` 当前只讨论三类内容：

- 提交前的本地预演与预测链路
- 浏览器 E2E 的真实执行入口
- release 打包与远端 deploy 的执行入口

它不直接替代：

- `.github/workflows/*.yml` 里的真实 GitHub Actions 定义
- `docs/frontend/`、`docs/backend/`、`docs/middle/` 里的实现、边界和契约说明

## 2. 模块文档索引

后续维护 `docs/test/` 时，优先从这里进入，再按改动范围进入对应模块文档：

- [`local-preflight.sh`](./local-preflight.sh): 当前根仓库本地预演脚本入口，供 `npm run preflight` 与 `npm run preflight:strict` 直接调用。`(file: local-preflight.sh)`
- [`preflight`](./preflight/README.md): 本地预演链路说明，包含执行顺序、输出产物、严格模式和与 CI/CD/E2E 的对应关系。`(file: preflight/README.md)`
- [`e2e`](./e2e/README.md): 浏览器 E2E 入口与 sim_viewer stub 链路说明，覆盖 `ensure-playwright-ready.sh`、`run-e2e-local.sh`、`topo_sim_stub_server.py` 和 `sim_viewer_flow.py`。`(file: e2e/README.md)`
- [`release`](./release/README.md): release 打包与远端部署说明，覆盖 `package-release.sh` 和 `deploy-via-ssh.sh`。`(file: release/README.md)`

## 3. 推荐阅读顺序

如果是第一次接触当前仓库的测试预演链路，建议按下面顺序阅读：

1. 先看本文，确认 `docs/test/` 只负责“提交前本地预测”，不替代 GitHub Actions 真正的执行定义。
2. 再看 [preflight/README.md](./preflight/README.md)，建立本地预演的执行顺序、输出产物和退出语义认知。
3. 如果要核对真实浏览器链路，再看 [e2e/README.md](./e2e/README.md) 以及 `docs/test/e2e/run-e2e-local.sh`、`docs/test/e2e/topo_sim_stub_server.py`。
4. 如果要核对真实交付链路，再看 [release/README.md](./release/README.md) 以及 `docs/test/release/package-release.sh`、`docs/test/release/deploy-via-ssh.sh` 和 `.github/workflows/cd.yml`。

## 4. 按改动范围同步文档

- 改 `docs/test/local-preflight.sh` 的执行顺序、环境探测、退出码或报告格式时，优先更新 [preflight/README.md](./preflight/README.md)。
- 改根仓库 `package.json` 里 `preflight`、`preflight:strict`、`test:e2e`、`cd:package` 的入口绑定时，必须同步更新本文和 [preflight/README.md](./preflight/README.md)。
- 如果改动影响 `docs/test/e2e/ensure-playwright-ready.sh`、`docs/test/e2e/run-e2e-local.sh`、`docs/test/e2e/topo_sim_stub_server.py` 或 `docs/test/e2e/sim_viewer_flow.py` 的真实行为，也要同步更新对应模块文档和 [preflight/README.md](./preflight/README.md)。
- 如果改动影响 `docs/test/release/package-release.sh`、`docs/test/release/deploy-via-ssh.sh` 或 `.github/workflows/cd.yml` 的真实行为，也要同步更新 [release/README.md](./release/README.md)。

## 5. 当前正式定位

截至当前仓库状态，`docs/test/` 的正式定位应当是：

一组围绕“提交前本地预演、浏览器 E2E 与 release 交付”组织起来的测试文档与脚本入口。它负责把 `sim_viewer` 这条前端链路的预测入口、真实 E2E 执行器和交付脚本统一收口到 `docs/test/` 下，降低根级杂散脚本继续膨胀的风险。
