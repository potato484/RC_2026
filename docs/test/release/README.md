# Release 与部署链路

## 1. 模块范围

这份文档只描述 release 打包与远端部署的真实执行链路，包括：

- 如何从 `sim_viewer` 构建产物生成 `release/` 目录
- 远端 SSH 部署当前需要的环境变量与目录结构
- 本地预演和 GitHub CD 分别如何复用这些脚本

## 2. 当前入口与执行顺序

当前这条链路的正式入口与装配顺序如下：

```text
package.json
  -> npm run cd:package
     -> docs/test/release/package-release.sh
        -> release/

.github/workflows/cd.yml
  -> npm run cd:package
  -> docs/test/release/deploy-via-ssh.sh
```

## 3. 关键文件导读

| 文件 | 当前实现 |
| --- | --- |
| `docs/test/release/package-release.sh` | 当前 release 打包入口。它会把 `sim_viewer/dist`、viewer package 元数据，以及 topo adapter 的 Python 脚本、`config/`、`sim_assets/` 收口到 `release/rc26_topo_sim_viewer/`。 |
| `docs/test/release/deploy-via-ssh.sh` | 当前远端部署执行器。它会用 `rsync + ssh` 把 `release/rc26_topo_sim_viewer/` 推到远端目录。 |
| `.github/workflows/cd.yml` | 当前 GitHub CD 编排入口。它先打包，再根据 deploy secrets 是否齐全决定执行部署还是只保留 artifact。 |

## 4. 当前注意点

- `docs/test/release/deploy-via-ssh.sh` 至少依赖 `DEPLOY_HOST`、`DEPLOY_USER`、`DEPLOY_PATH`。
- 当前 release 打包只保证前端静态资产和 adapter 资源被收口，不负责把完整 ROS 工作区或已编译二进制一起打进 artifact。
- 如果目标机器要跑真实 `topo_sim_server.py` 的 live / A* 真链路，仍然需要目标环境本身具备 `rc26_topo_nav` 工作区依赖与 `install/setup.bash`。
- 如果改了 release 目录结构、远端部署目录或需要同步的文件集合，必须同时更新本文、`docs/test/preflight/README.md` 和 `.github/workflows/cd.yml`。
