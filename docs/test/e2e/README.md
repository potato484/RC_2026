# 浏览器 E2E 链路

## 1. 模块范围

这份文档只描述浏览器 E2E 的真实执行链路，包括：

- 根仓库如何启动 sim_viewer 的 E2E stub backend 与前端 preview
- 浏览器脚本当前覆盖了哪些页面与交互
- E2E 产物、日志和失败定位入口

## 2. 当前入口与执行顺序

当前这条链路的正式入口与装配顺序如下：

```text
package.json
  -> npm run test:e2e
     -> docs/test/e2e/run-e2e-local.sh
        -> docs/test/e2e/ensure-playwright-ready.sh
        -> docs/test/e2e/topo_sim_stub_server.py
        -> src/rc26_topo_nav/sim_viewer VITE_API_BASE_URL=... VITE_WS_BASE_URL=... npm run build -- --outDir dist-e2e
        -> python3 -m http.server --directory dist-e2e
        -> docs/test/e2e/sim_viewer_flow.py
        -> artifacts/e2e/*
```

## 3. 关键文件导读

| 文件 | 当前实现 |
| --- | --- |
| `docs/test/e2e/ensure-playwright-ready.sh` | 当前浏览器 E2E 的 Playwright 运行时守门脚本。负责检查 Python Playwright 包、Chromium 主浏览器和 `chromium_headless_shell` 是否齐全，并在缺失时补执行安装。 |
| `docs/test/e2e/run-e2e-local.sh` | 当前浏览器 E2E 主入口。负责找可用端口、启动 `topo_sim_stub_server.py`、以 `dist-e2e/` 重建 `sim_viewer` 静态预览，并在失败时打印对应日志尾部。 |
| `docs/test/e2e/topo_sim_stub_server.py` | 当前契约型 stub backend。它不跑真实 ROS2 或 `planner_trace_cli`，但会补齐浏览器 E2E 需要的 `scene-manifest`、`surface-route preview / trace-from-nodes / trace / execute` 以及旧 run / live 兼容接口面，保证 `sim_viewer` 的静态预览在本地和 CI 上都能走通当前主链。 |
| `docs/test/e2e/sim_viewer_flow.py` | 当前 sim_viewer E2E 用例本体。覆盖首屏加载、切到顶部正交视角、在 Babylon 画布里实际设起点/终点、点击“生成三维路线”、等待 preview-first 主链补齐搜索回放、切换“已探查”图层，以及拖动“回放帧”滑块回看首帧这条真实用户链路。脚本会按多组坐标重试画布点击，降低 CI 里因视口细微差异导致单次选点失效的概率。 |
| `src/rc26_topo_nav/sim_viewer/src/api.ts` | 当前前端 API 入口。现在支持通过 `VITE_API_BASE_URL` 与 `VITE_WS_BASE_URL` 切换后端来源，让浏览器 E2E 不再依赖 dev proxy。 |
| `src/rc26_topo_nav/sim_viewer/src/App.tsx` / `src/rc26_topo_nav/sim_viewer/src/store.ts` | 当前页面已经收口成“单用途三维路线观察台”。主界面只保留画布、图层按钮、视角切换、场景选点、`生成三维路线`、回放滑块与可选执行；不再暴露旧的 `高级 / 调试`、离线 run WebSocket 控制或 live 只读桥接。 |

## 4. 当前输出与注意点

- 默认产物目录：`artifacts/e2e/`
- 常见日志文件：`backend-e2e.log`、`frontend-e2e.log`
- 失败时截图与页面快照：`sim-viewer-e2e-failure.png`、`sim-viewer-e2e-failure.html`
- 成功时截图：`sim-viewer-e2e-success.png`
- Playwright 浏览器运行时默认复用用户缓存目录 `~/.cache/ms-playwright/`；`artifacts/` 只保存执行产物，不保存可复用的 Chromium 二进制。
- 这条链路依赖 Python Playwright、Chromium 主浏览器、`chromium_headless_shell`、`fastapi` 和 `uvicorn`；`run-e2e-local.sh` 与 `npm run preflight` 都会先检查浏览器运行时是否齐全。
- 当前 E2E 的目标是稳定覆盖“浏览器能否真实走通主要交互”，而不是验证真实 planner 算法输出；算法真链路仍应通过 `topo_sim_server.py` 和人工联调确认。
