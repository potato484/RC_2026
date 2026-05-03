# Merlin 行为树前端测试链路

## 1. 模块范围

这份文档只描述 `merlin-bt-visualizer` 这条前端链路的真实测试与交付入口，包括：

- 本地如何做提交前预演
- 浏览器 E2E 当前覆盖哪些查看态、编辑态和开发态写回交互
- release 打包当前会收口哪些静态产物
- GitHub CI/CD 如何复用这套本地脚本

## 2. 当前入口与执行顺序

当前这条链路的正式入口与装配顺序如下：

```text
package.json
  -> npm run merlin:preflight
  -> npm run merlin:preflight:strict
     -> docs/test/merlin_bt_visualizer/local-preflight.sh
        -> optional npm ci for merlin-bt-visualizer
        -> merlin vitest
        -> merlin vite build
        -> docs/test/merlin_bt_visualizer/run-e2e-local.sh
        -> docs/test/merlin_bt_visualizer/package-release.sh
        -> artifacts/merlin-bt-visualizer/preflight/<timestamp>/summary.md

package.json
  -> npm run merlin:test:e2e
     -> docs/test/merlin_bt_visualizer/run-e2e-local.sh
        -> docs/test/merlin_bt_visualizer/ensure-playwright-ready.sh
        -> merlin-bt-visualizer npm run build
        -> merlin-bt-visualizer npm run preview
        -> merlin-bt-visualizer/e2e/viewer-editor.spec.ts
        -> merlin-bt-visualizer npm run dev (MERLIN_BT_SAVE_DIR=...)
        -> merlin-bt-visualizer/e2e/save-to-source.spec.ts
        -> artifacts/merlin-bt-visualizer/e2e/*

package.json
  -> npm run merlin:cd:package
     -> docs/test/merlin_bt_visualizer/package-release.sh
        -> release/merlin_bt_visualizer/

.github/workflows/merlin-bt-visualizer-ci.yml
  -> npm run merlin:preflight:strict

.github/workflows/merlin-bt-visualizer-cd.yml
  -> npm run merlin:cd:package
```

## 3. 关键文件导读

| 文件 | 当前实现 |
| --- | --- |
| `docs/test/merlin_bt_visualizer/local-preflight.sh` | `merlin-bt-visualizer` 的本地预演入口。负责环境版本提示、依赖安装、单测、构建、E2E 与 release 打包的统一收口。 |
| `docs/test/merlin_bt_visualizer/ensure-playwright-ready.sh` | `merlin-bt-visualizer` 的 Playwright 运行时守门脚本。负责在本地或 CI 上准备 Chromium 运行时。 |
| `docs/test/merlin_bt_visualizer/run-e2e-local.sh` | `merlin-bt-visualizer` 的浏览器 E2E 主入口。它会先跑 `preview` 验证查看态/编辑态中文联动，再跑 `dev` 验证“保存到源文件”的开发态本地适配层。 |
| `docs/test/merlin_bt_visualizer/package-release.sh` | `merlin-bt-visualizer` 的 release 打包入口。当前只收口静态 `dist/`、前端包元数据和说明文档摘录。 |
| `merlin-bt-visualizer/playwright.config.ts` | `merlin-bt-visualizer` 的 Playwright 全局配置，统一管理 `baseURL`、trace、截图、录像和报告输出目录。 |
| `merlin-bt-visualizer/e2e/viewer-editor.spec.ts` | 覆盖查看态区域切换、编辑态区域联动和默认中文展示这条主链路。 |
| `merlin-bt-visualizer/e2e/save-to-source.spec.ts` | 覆盖开发态“保存到源文件”能力。测试不会改真实仓库 XML，而是把写回目标改到临时副本目录。 |
| `.github/workflows/merlin-bt-visualizer-ci.yml` | `merlin-bt-visualizer` 的独立 GitHub CI workflow，负责跑严格 preflight 并上传构建与 E2E 产物。 |
| `.github/workflows/merlin-bt-visualizer-cd.yml` | `merlin-bt-visualizer` 的独立 GitHub CD workflow，负责按 tag 或手动触发打包 release artifact。 |

## 4. 当前输出与注意点

- 默认 E2E 产物目录：`artifacts/merlin-bt-visualizer/e2e/`
- 默认 preflight 报告目录：`artifacts/merlin-bt-visualizer/preflight/<timestamp>/summary.md`
- 默认 release 目录：`release/merlin_bt_visualizer/`
- `run-e2e-local.sh` 默认优先尝试 `4173/4174`，如果本机已有别的服务占用，会自动顺延到下一个可用端口，避免预检时误连到错误站点。
- E2E 当前拆成两段：
  - `preview` 段验证“查看态 / 编辑态是否跟随区域切换、默认展示是否继续收口中文”
  - `dev` 段验证“保存到源文件”是否真的写回目标文件
- 开发态写回 E2E 当前不会直接改 `src/rc26_decision/behavior_trees/*.xml` 真源，而是通过 `MERLIN_BT_SAVE_DIR` 把写回路径重定向到临时目录，避免测试污染工作区。
- 当前 CD 只打包静态站点 artifact，不做远端部署，也不意味着浏览器版本拥有通用持久化后端。
- 当前仓库只保留 `merlin-bt-visualizer` 这套前端 GitHub CI/CD。旧的 `rc26-visualization-viewer-*` workflow 已删除，因为对应 viewer 模块和文档入口已不再是当前仓库真源。

## 5. 当前正式定位

截至当前仓库状态，这条链路更准确的定义是：

一组围绕 `merlin-bt-visualizer` 组织起来的本地测试、浏览器 E2E 与静态 release 打包入口。它的目标是稳定验证“查看 XML、编辑 XML、开发态本地写回 XML”这三类前端能力，而不是把这个前端伪装成在线后端或部署系统。
