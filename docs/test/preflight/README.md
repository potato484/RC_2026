# 本地预演链路

## 1. 模块范围

这份文档只描述提交前本地预演的那条链路，包括：

- 本机环境与 GitHub Actions 的版本差异探测
- `sim_viewer` 前端 `test / build` 与 topo adapter Python 语法检查的本地收口
- 浏览器 E2E 是否真正跑到，还是被本机环境阻塞
- release 打包与 deploy gate 的本地预测结果
- 预演报告、日志和严格模式的输出语义

## 2. 入口与执行顺序

当前这条链路的正式入口与装配顺序如下：

```text
package.json
  -> npm run preflight
  -> npm run preflight:strict
     -> docs/test/local-preflight.sh
        -> environment parity probe
        -> optional npm ci for src/rc26_topo_nav/sim_viewer
        -> sim_viewer vitest
        -> sim_viewer vite build
        -> topo adapter py_compile
        -> docs/test/e2e/run-e2e-local.sh
        -> docs/test/release/package-release.sh
        -> deploy gate dry-run or docs/test/release/deploy-via-ssh.sh
        -> artifacts/preflight/<timestamp>/summary.md
```

## 3. 关键文件导读

| 文件 | 当前实现 |
| --- | --- |
| `docs/test/local-preflight.sh` | 当前本地预演主入口。负责探测 Node 22 / Python 3.10 对齐情况、是否需要补装 `sim_viewer` 的 `node_modules`，并按顺序执行单测、构建、E2E 和 release 打包。 |
| `package.json` | 对外暴露 `npm run preflight`、`npm run preflight:strict`、`npm run test:e2e` 与 `npm run cd:package` 四个统一入口，不要求协作者记住脚本真实路径。 |
| `.github/workflows/ci.yml` | 当前 GitHub CI 编排入口。它复用这条链路的依赖前提，并在 Ubuntu 22.04 上跑严格模式 preflight。 |
| `.github/workflows/cd.yml` | 当前 GitHub CD 编排入口。它先打包 `release/`，再根据 deploy secrets 是否齐全决定是否执行远端 SSH 部署。 |
| `docs/test/e2e/run-e2e-local.sh` | 当前浏览器 E2E 真实执行器。本地预演不会重写它，只会在环境允许时复用它。 |
| `docs/test/release/package-release.sh` | 当前 release 打包入口。本地预演会在 CI 检查通过后直接复用它，确认 `release/` 产物完整。 |
| `docs/test/release/deploy-via-ssh.sh` | 当前远端部署执行脚本。本地预演默认只做 gate 判断，只有显式传 `--run-deploy` 才会真的执行 SSH 路径。 |

## 4. 当前输出与退出语义

- 默认模式：只有真实失败才返回非零；`SKIP` 不会阻断预演。
- 严格模式：`npm run preflight:strict` 会把 `WARN` 和 `FAIL` 都视为不满足提交门槛，返回非零。
- 报告目录：`artifacts/preflight/<timestamp>/summary.md`
- 最新报告快捷入口：`artifacts/preflight/latest.md`
- 分步日志目录：`artifacts/preflight/<timestamp>/logs/*.log`

## 5. 当前注意点

- 本地预演的目标是“预测 GitHub CI/CD 大概率会怎么走”，不是完全替代 GitHub runner 本身。
- 浏览器 E2E 当前依赖 Python Playwright、Chromium 主浏览器、`chromium_headless_shell`、`fastapi` 和 `uvicorn`；这些条件缺失时，E2E 会直接失败。
- 这条链路当前使用的是 `docs/test/e2e/topo_sim_stub_server.py`，而不是依赖真实 ROS2 / `planner_trace_cli`；它的目标是稳定覆盖浏览器真实交互，不是替代真实 planner 联调。
- deploy 预测默认只走到 gate 判断；如果本机已经具备完整 `DEPLOY_*` 变量并且确实要演练远端 SSH，再显式传 `--run-deploy`。
