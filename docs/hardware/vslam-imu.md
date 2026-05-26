# VSLAM + IMU (indoor, GPS-denied)

**Target platform:** any ground robot or aerial platform using visual SLAM for localization. No GPS, no wheel encoders required.

**Confirmed setup (issue #46):** ORB-SLAM3 stereo-only + VectorNav 9-axis IMU, indoor.

---

## Quick start

```bash
ros2 launch fusioncore_ros fusioncore.launch.py \
  fusioncore_config:=$(ros2 pkg prefix fusioncore_ros)/share/fusioncore_ros/config/vslam_imu.yaml
```

Set `vslam.topic` to wherever your VSLAM publishes `nav_msgs/Odometry`. The covariance field must be filled (see [Covariance requirement](#covariance-requirement) below).

---

## How it works

FusionCore fuses IMU gyro/accel measurements at full rate (typically 100-200 Hz) and corrects the drifting UKF state with 6-DOF pose updates from VSLAM at a lower rate (5-30 Hz). The twist component of the Odometry message is ignored; only `pose.pose` and `pose.covariance` are used.

The update path:

1. IMU predict steps run at full rate, integrating linear acceleration and gyro into the 23-state UKF.
2. Each VSLAM pose arrives as a 6D measurement `[x, y, z, roll, pitch, yaw]`.
3. Mahalanobis outlier gate (chi2(6, 0.999) = 22.46) rejects reinitializations and tracking jumps.
4. If accepted: UKF update step corrects position and orientation; covariance shrinks.

---

## Covariance requirement

The `pose.covariance` field of `nav_msgs/Odometry` is a 6x6 matrix in `[x, y, z, roll, pitch, yaw]` order. FusionCore reads the diagonal of this matrix to build the measurement noise covariance R.

If covariance is all zeros, FusionCore falls back to:

```yaml
vslam.position_noise: 0.1    # 10 cm 1-sigma
vslam.orientation_noise: 0.02  # ~1.1 deg 1-sigma
```

**ORB-SLAM3:** covariance is filled when tracking quality is good. A zero covariance means ORB-SLAM3 is reporting unknown uncertainty (typically during reinitialization). FusionCore's outlier gate will reject reinitializations automatically, but calibrated covariance makes the gate tighter and the fusion more accurate.

To verify covariance is being published:

```bash
ros2 topic echo /vslam/odometry --field pose.covariance
```

---

## IMU configuration

### 9-axis IMU (VectorNav, Xsens, BNO08x)

These IMUs include a magnetometer and publish full orientation with a magnetometer-referenced yaw:

```yaml
imu.has_magnetometer: true
imu.gyro_noise: 0.005   # rad/s (VectorNav VN-100 typical)
imu.accel_noise: 0.1    # m/s²
```

With `imu.has_magnetometer: true`, FusionCore validates heading from the IMU orientation message (if your driver publishes it). This enables lever arm correction for GPS (unused here) and marks `heading_source = IMU_ORIENTATION`.

### 6-axis IMU (MPU-6050, ICM-42688, etc.)

```yaml
imu.has_magnetometer: false
```

Yaw from a 6-axis IMU drifts over time. VSLAM corrects this drift on every update.

---

## VectorNav setup

VectorNav drivers publish two topics:

| Topic | Message | FusionCore input |
|---|---|---|
| `/vectornav/imu` | `sensor_msgs/Imu` | IMU (gyro + accel) |
| `/vectornav/imu` | `sensor_msgs/Imu` (orientation field) | `update_imu_orientation` |

The FusionCore ROS node subscribes to the IMU topic and automatically calls both `update_imu` and `update_imu_orientation` from the same message when `imu.has_magnetometer: true`.

---

## ORB-SLAM3 setup

ORB-SLAM3 in stereo-only mode publishes pose on a configurable topic. Point FusionCore at it:

```yaml
vslam.topic: "/orbslam3/camera/odometry"  # adjust to your node's topic
```

If your ORB-SLAM3 node publishes `geometry_msgs/PoseStamped` instead of `nav_msgs/Odometry`, wrap it with a converter node or use a fork that publishes Odometry (most modern forks do).

### Reinitialization handling

ORB-SLAM3 can jump by meters on tracking loss and recovery. Two mechanisms work together to handle this:

**Chi-squared gate** (immediate): a large jump produces a high Mahalanobis distance and is rejected. The filter state is unaffected. `outlier_threshold_vslam` (default 22.46 = chi2(6, 0.999)) controls this threshold.

**Map re-anchoring** (recovery): after `vslam.reinit_n` consecutive gate rejections (default 10, roughly 2 seconds at 5 Hz), FusionCore concludes that ORB-SLAM3 has reinitialized to a new map origin. It computes a new offset between the VSLAM map frame and the filter's odom frame using the filter's current position estimate, and resumes fusion from there. This is logged as a WARN:

```
[WARN] VSLAM: 10 consecutive rejections: reinitialization detected. Re-anchoring map origin.
```

Tune `outlier_threshold_vslam` if:

- **Too many rejections during normal motion:** increase threshold (e.g. 30.0)
- **Reinitializations are accepted as valid updates:** decrease threshold (e.g. 18.0)

Tune `vslam.reinit_n` if:
- **Re-anchor fires too eagerly during fast motion:** increase (e.g. 20)
- **Recovery after tracking loss is too slow:** decrease (e.g. 5)

---

## Complete YAML

```yaml
fusioncore:
  ros__parameters:
    base_frame: base_link
    odom_frame: odom
    publish_rate: 100.0
    publish.force_2d: true        # set false for aerial platforms

    imu.has_magnetometer: true    # VectorNav, Xsens, BNO08x
    imu.gyro_noise: 0.005         # rad/s
    imu.accel_noise: 0.1          # m/s²
    imu.remove_gravitational_acceleration: false

    vslam.topic: "/vslam/odometry"
    vslam.position_noise: 0.1
    vslam.orientation_noise: 0.02

    outlier_rejection: true
    outlier_threshold_vslam: 22.46

    adaptive.imu: true
    adaptive.encoder: false
    adaptive.gnss: false
    adaptive.window: 50
    adaptive.alpha: 0.01

    ukf.q_position: 0.01
    ukf.q_orientation: 1.0e-9
    ukf.q_velocity: 0.1
    ukf.q_angular_vel: 0.1
    ukf.q_acceleration: 1.0
    ukf.q_gyro_bias: 1.0e-5
    ukf.q_accel_bias: 1.0e-5

    init.stationary_window: 2.0
    init.wait_for_all_sensors: true
    init.sensor_wait_timeout: 10.0

    motion_model: "ConstantVelocityAcceleration"

    reference.use_first_fix: false
    reference.x: 0.0
    reference.y: 0.0
    reference.z: 0.0
    input.gnss_crs: "EPSG:4326"
    output.crs: "EPSG:4978"
    output.convert_to_enu_at_reference: true
```

---

## Diagnostics

When VSLAM is active, `/fusioncore/diagnostics` reports:

```
VSLAM: OK   (or STALE if no message in >1s)
```

To check outlier rejection rate:

```bash
ros2 topic echo /fusioncore/status | grep vslam
```

A healthy system should show `vslam_outliers` near 0 during steady tracking.

---

## Troubleshooting

**Filter drifts badly between VSLAM updates**
: Lower `ukf.q_position` or increase IMU rate. The IMU integrates forward between pose corrections; high process noise causes drift.

**VSLAM updates rejected (vslam_outliers climbing)**
: Check covariance is filled. If ORB-SLAM3 reinitializes frequently, increase `outlier_threshold_vslam` to allow gradual recovery.

**Position jumps on ORB-SLAM3 reinitialization**
: Expected behavior. The outlier gate should catch these. If jumps pass through, calibrate ORB-SLAM3's covariance output or lower `outlier_threshold_vslam`.

**VSLAM health shows STALE**
: Check `vslam.topic` matches what ORB-SLAM3 publishes: `ros2 topic list | grep odom`.
