# 浏览器 E2E

`docs/test/e2e/run-e2e-local.sh` 会为 `src/rc26_xhu_viewer/rc26_xhu_viewer/viewer` 启动一条本地浏览器 E2E 链路。

## 当前链路

- 契约 stub backend：`docs/test/e2e/xhu_viewer_stub_server.py`
- 前端构建目录：`src/rc26_xhu_viewer/rc26_xhu_viewer/viewer/dist-e2e`
- 浏览器脚本：`docs/test/e2e/xhu_viewer_flow.py`

## 当前定位

- 目标是稳定验证浏览器主交互，而不是替代真实 planner 联调
- stub 会镜像当前 viewer 依赖的 manifest、local planner 案例接口和 live snapshot 契约
- 真实 `rc26_xhu_viewer_server.py` 仍建议通过人工联调验证，E2E 重点是保证浏览器主链路持续可回归

## 当前验证重点

- 首屏加载
- manifest 驱动的标题、布局预设和阶段区
- 场景选起点/终点
- 生成表面路线
- 等待回放补齐
- 加载局部规划案例
- 图层切换和回放滑块
