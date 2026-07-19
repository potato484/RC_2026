# 第三方与包级许可说明

本仓库由西华大学创界 RC 战队视觉算法组开源，但它不是单一来源、单一许可证的软件包。根目录 [MIT License](LICENSE) 只适用于项目组有权授权且没有更具体许可声明的内容。

使用、修改或再分发本仓库时，按以下优先顺序判断许可：

1. 文件自身的版权头、SPDX 标识或许可证说明。
2. 所在 ROS 2 包的 `package.xml` 和随包许可证。
3. 本文件列出的上游项目许可证与版权声明。
4. 以上均未单独说明、且项目组有权授权的内容，适用根目录 MIT License。

根许可证不会撤销、替换或重新授权现有上游代码。外部依赖通常由系统包、ROS 2 包或板端 SDK 单独安装，它们继续遵守各自许可证，不因被本项目调用而改用根 MIT。

## ROS 2 包许可汇总

下表按当前各包 `package.xml` 记录。`BSD` 的具体文本依据 Point-LIO 当前使用的上游 `point-lio-with-grid-map` 分支确认为 BSD 3-Clause。

| 包 | `package.xml` 声明 | 说明 |
| --- | --- | --- |
| `rc26_bringup` | Apache-2.0 | 整车 launch 与装配 |
| `rc26_decision` | MIT | 行为树决策与动作 |
| `rc26_interfaces` | MIT | 自定义 ROS 2 接口 |
| `rc26_localization` | Apache-2.0 | 点云地图定位 |
| `rc26_mcu_transport` | MIT | 目标 MCU 串口 transport |
| `rc26_mid360_driver` | MIT | Mid-360 驱动；保留 Yingjie Huang 等文件级版权声明 |
| `rc26_odom_interface` | Apache-2.0 | 保留 Lihan Chen 等文件级版权声明 |
| `rc26_point_lio` | BSD | 上游具体为 BSD-3-Clause，详见下文 |
| `rc26_sensor_extrinsics` | MIT | 传感器安装外参 |
| `rc26_sensor_scan` | Apache-2.0 | 保留 Lihan Chen 等文件级版权声明 |
| `rc26_serial` | MIT | 串口协议基础库 |
| `rc26_small_gicp` | MIT | 上游 small_gicp，详见下文 |
| `rc26_telecontrol` | Apache-2.0 | 人工遥控测试 |
| `rc26_vision` | MIT | 视觉推理与定位 |

Apache-2.0 的完整文本见 [LICENSES/Apache-2.0.txt](LICENSES/Apache-2.0.txt)。各文件和包内已有版权声明必须继续保留。

## Point-LIO 与 LOAM 来源

仓库中的 `src/rc26_point_lio` 基于 HKU-MARS Point-LIO 的 `point-lio-with-grid-map` 分支演进，并保留其 `package.xml` 中的 BSD 声明。

- 上游仓库：https://github.com/hku-mars/Point-LIO
- 核验分支：`point-lio-with-grid-map`
- 上游许可证：https://github.com/hku-mars/Point-LIO/blob/point-lio-with-grid-map/LICENSE
- 本仓库许可证副本：[LICENSES/BSD-3-Clause-Point-LIO.txt](LICENSES/BSD-3-Clause-Point-LIO.txt)

应保留的上游版权信息包括：

```text
Modifier: livox               dev@livoxtech.com

Copyright 2013, Ji Zhang, Carnegie Mellon University
Further contributions copyright (c) 2016, Southwest Research Institute
All rights reserved.
```

Point-LIO 源码树还包含 IKFoM、MTK、iVox/Hilbert 等来源明确的代码片段。它们的文件级版权和许可注释继续有效，不能因根目录新增 MIT License 而删除。

## small_gicp

仓库中的 `src/rc26_small_gicp` 基于 Kenji Koide 的 small_gicp，并在项目内进行适配。

- 上游仓库：https://github.com/koide3/small_gicp
- 上游许可证：https://github.com/koide3/small_gicp/blob/master/LICENSE
- 本仓库许可证副本：[LICENSES/MIT-small-gicp.txt](LICENSES/MIT-small-gicp.txt)

应保留的上游版权信息：

```text
Copyright (c) 2024 Kenji Koide
```

`rc26_small_gicp` 内的文件已经使用 `SPDX-License-Identifier: MIT` 标识主要代码。部分辅助实现包含其它来源的版权与许可，例如：

- `include/small_gicp/ann/kdtree.hpp` 包含来自 nanoflann 的 BSD 许可说明与原作者版权。
- `include/small_gicp/util/lie.hpp` 包含来自 Sophus 的 MIT 许可片段与原作者版权。

这些文件内声明必须原样保留。

## 外部库与运行环境

本项目构建或运行时会链接、调用或加载 ROS 2、BehaviorTree.CPP、PCL、Eigen、OpenCV、GTSAM、ONNX Runtime、AidLite、librealsense、yaml-cpp 等外部组件。它们通常不以完整源码形式包含在本仓库中，其许可证、专利和再分发要求由各自发布方决定。

AidLite 和相关板端运行库属于 AidLux 部署环境。使用者需要自行确认所获得 SDK、运行库和硬件镜像的授权范围。训练数据、第三方预训练基础模型和未包含在本仓库中的运行资产也不因本项目的 MIT License 自动获得授权。

## 再分发要求

再分发本仓库源码或二进制产物时，至少应：

1. 保留根目录 `LICENSE`。
2. 保留本文件 `THIRD_PARTY_NOTICES.md`。
3. 保留 `LICENSES/` 中适用的许可证文本。
4. 保留源文件中的版权、SPDX、上游来源和免责声明。
5. 对二进制分发满足 BSD-3-Clause 和 Apache-2.0 关于许可证、版权与 NOTICE（如适用）的要求。

本说明用于整理仓库内已识别的许可关系，不构成法律意见。如果未来新增第三方源码、模型或二进制资产，提交者必须同时补充来源、版本、许可证和必要的版权文本。
