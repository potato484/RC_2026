# 本地预演

`docs/test/local-preflight.sh` 是当前 `rc26_xhu_viewer` Web 工具链的本地预演主入口。

## 当前执行内容

- 可选 `npm ci`
- `src/rc26_xhu_viewer/rc26_xhu_viewer/viewer` 的 `npm test`
- `src/rc26_xhu_viewer/rc26_xhu_viewer/viewer` 的 `npm run build`
- `src/rc26_xhu_viewer/rc26_xhu_viewer/scripts/*.py` 的 `py_compile`
- 浏览器 E2E
- release 打包

## 当前注意点

- 浏览器 E2E 仍然使用契约 stub，而不是依赖真实 ROS2 / planner CLI
- 真实 `rc26_xhu_viewer_server.py` 仍建议通过人工联调验证
