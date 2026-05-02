# 前端总览

## 当前工程

- `merlin-bt-visualizer`

## 当前主链路

当前仍在仓库内维护的前端主链路是 `merlin-bt-visualizer` 的行为树查看、编辑和本地模拟执行。

## 当前入口文件

- `merlin-bt-visualizer/src/main.tsx`
- `merlin-bt-visualizer/src/App.tsx`
- `merlin-bt-visualizer/src/components/*`

## 本轮结构更新

- 前端文档继续只服务 `merlin-bt-visualizer`
- 任何机器人运行时可视化都不在本目录内维护
- `merlin-bt-visualizer` 当前只保留 Vite 本地保存等项目特有配置说明，Playwright、Tailwind、PostCSS 和 tsconfig 这类标准工具选项不再逐项注释；本次不新增在线后端能力。
