# rc26_sensor_extrinsics

`rc26_sensor_extrinsics` is the config-only source of truth for static sensor mounting
extrinsics on R2.

It does not publish TF by itself. `rc26_bringup` reads this package's YAML profile
and publishes the runtime static TF edges.

## Current Profile

- `r2_mid360_left_90`: Mid-360 mounted at `z=0.13 m` with `livox_frame` yawed
  `+90 deg` relative to `base_link`.

## Boundary

- This package describes the robot-level mounting pose from `base_link` to the
  external sensor frame.
- Point-LIO's internal LiDAR/IMU calibration remains in
  `rc26_point_lio/config/mid360.yaml`.
