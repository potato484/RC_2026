# Changelog

This repository follows semantic versioning at the repository level via Git tags (e.g. `v2.0.0`).

## v2.0.0

- Breaking: Adjusted `point_lio` MID360 default topics and timestamp unit assumptions in `RC_2026_1/point_lio/config/mid360.yaml`.
- Updated `mid360_driver` publishers to use `rclcpp::SensorDataQoS()` to match typical downstream consumers.
- Updated bringup/odometry launch and RViz configs for the new pipeline.
- Improved decision and odometry/serial integration (new/updated modules under `RC_2026_1/rc26_*`).

