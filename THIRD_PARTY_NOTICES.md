# Third-Party Notices

This file centralizes third-party attribution and license notes for the RC_2026 repository. It is a navigation aid, not a replacement for the exact license text in source files, package manifests, vendored packages, installed system packages, or upstream SDKs.

If a file, package, model, dataset, SDK, or binary artifact carries a more specific notice, that specific notice takes precedence over this summary.

## Project Default License

Content authored by 西华大学创界 RC 战队视觉算法组 and not otherwise marked is provided under the repository root [MIT License](LICENSE).

Some first-party ROS 2 packages use Apache-2.0 in their `package.xml`; those package-level declarations remain authoritative for those packages.

| Package | Declared license | Notes |
| --- | --- | --- |
| `rc26_interfaces` | MIT | Custom ROS 2 messages and services |
| `rc26_sensor_extrinsics` | MIT | R2 sensor extrinsics profiles |
| `rc26_serial` | MIT | Serial protocol library |
| `rc26_mcu_transport` | MIT | Target MCU ROS 2 transport |
| `rc26_vision` | MIT | Vision runtime and backend integration |
| `rc26_mid360_driver` | MIT | Mid-360 driver integration |
| `rc26_decision` | MIT | Behavior-tree decision logic |
| `rc26_bringup` | Apache-2.0 | Launch, bringup, runtime configuration |
| `rc26_localization` | Apache-2.0 | Prior-map localization integration |
| `rc26_odom_interface` | Apache-2.0 | Unified odom / TF interface |
| `rc26_sensor_scan` | Apache-2.0 | Sensor scan transforms |
| `rc26_telecontrol` | Apache-2.0 | Teleoperation controller |

## Vendored or In-Tree Third-Party Code

| Component | Location | License / attribution | Repository usage |
| --- | --- | --- | --- |
| Point-LIO / LOAM-derived code | `src/rc26_point_lio/` | BSD-style license in `src/rc26_point_lio/LICENSE`; package manifest declares BSD | LiDAR-inertial odometry and mapping base, adapted for R2 runtime |
| LOAM | `src/rc26_point_lio/LICENSE` and inherited code comments | Copyright Ji Zhang, Carnegie Mellon University; BSD-style notice | Upstream algorithm lineage for LIO / mapping code |
| IKFoM / MTK utilities | `src/rc26_point_lio/include/IKFoM/` | File headers credit The University of Hong Kong and Universitaet Bremen contributors | Manifold and ESKF utilities used by Point-LIO code |
| iVox / Point-LIO utilities | `src/rc26_point_lio/include/ivox/` | File-level notices where present | Incremental voxel structures and point-cloud support |
| hilbert_hpp snippet | `src/rc26_point_lio/include/ivox/hilbert.hpp` | MIT notice in file header; copyright David Beynon | Hilbert curve helper used by voxel indexing code |
| small_gicp | `src/rc26_small_gicp/` | MIT, copyright Kenji Koide, see `src/rc26_small_gicp/LICENSE` | In-tree point-cloud registration library |
| nanoflann-inspired KD-tree code | `src/rc26_small_gicp/include/small_gicp/ann/kdtree.hpp` | BSD notice retained in file header | KD-tree implementation inspiration / attribution |
| Sophus-derived SO3 expmap code | `src/rc26_small_gicp/include/small_gicp/util/lie.hpp` | MIT-style Sophus notice retained in file header | Lie algebra helper implementation |

## External Build and Runtime Dependencies

These components are consumed through the operating system, ROS 2 environment, board SDK, or normal package installation. They are not owned by this repository.

| Component | Typical license family | Repository usage |
| --- | --- | --- |
| ROS 2 Humble and standard ROS messages | Apache-2.0 and package-specific ROS licenses | Node runtime, messages, launch, TF, build tooling |
| BehaviorTree.CPP | MIT | Behavior-tree execution in `rc26_decision` |
| PCL | BSD | Point-cloud containers, filtering, registration support |
| Eigen | MPL2 and bundled component notices | Linear algebra |
| OpenCV | Apache-2.0 | Image processing and vision utilities |
| GTSAM | BSD-style license | Point-LIO experimental loop-closure build dependency |
| yaml-cpp | MIT | YAML configuration parsing |
| ONNX Runtime | MIT | Optional local inference backend |
| librealsense / realsense2_camera | Apache-2.0 and package-specific notices | Optional RealSense D455 integration |
| AidLite / AidLux SDK | Vendor SDK terms | Optional board-side inference backend on AidLux / X1 |
| Livox SDK / Mid-360 ecosystem packages | Vendor and package-specific terms | Mid-360 hardware access and integration |

Always verify the exact license text from the installed package or upstream project before redistributing binaries, container images, SDK files, model files, or documentation derived from those projects.

## Model and Dataset Notice

The repository contains vision model artifacts under `src/rc26_vision/models/`, including `.pt`, `.onnx`, and label files. Before public redistribution or downstream reuse, maintainers should record for each model:

- model architecture and source project;
- training dataset source and permission status;
- checkpoint author or exporting maintainer;
- license or internal-use restriction;
- conversion command and exported inference backend target.

Until those records are complete, do not assume model weights or datasets share the repository root MIT license.

## Redistribution Checklist

Before publishing a release, archive, container, or fork, verify that:

1. Root `LICENSE`, `LICENSE-APACHE`, this notice file, and in-tree package license files are included.
2. `src/rc26_point_lio/LICENSE` and `src/rc26_small_gicp/LICENSE` remain with their source trees.
3. File-level SPDX or copyright headers are preserved.
4. External SDKs, camera libraries, inference runtimes, and model weights are redistributed only when their upstream terms allow it.
5. Any newly added third-party code, model, dataset, or binary has a source, version, license, and modification note added to this file.
