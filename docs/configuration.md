# Configuration

FusionCore is configured with a single YAML file passed to the launch file:

```bash
ros2 launch fusioncore_ros fusioncore.launch.py \
  fusioncore_config:=/path/to/your_robot.yaml
```

---

## Full parameter reference

```yaml
fusioncore:
  ros__parameters:
    base_frame: base_link    # must match your robot's base TF frame
    odom_frame: odom
    publish_rate: 100.0
    publish.force_2d: true   # zeroes Z position and Z velocity in published output. Use for ground robots.
    publish.tf: true         # set false to suppress the odom->base_link TF broadcast.
                             # /fusion/odom keeps publishing. Use when another node owns
                             # the odom->base_link transform, or when running two
                             # FusionCore instances where only one should broadcast TF.

    # ── IMU ──────────────────────────────────────────────────────────────────
    imu.gyro_noise: 0.005       # rad/s: from your IMU datasheet (ARW spec)
    imu.accel_noise: 0.1        # m/s² : from your IMU datasheet (VRW spec)
    imu.has_magnetometer: false # true for 9-axis (BNO08x, VectorNav, Xsens)
                                # false for 6-axis: yaw comes from gyro integration
    imu.remove_gravitational_acceleration: false
    # Set true if your IMU driver already subtracted gravity.
    # Check: echo linear_acceleration.z at rest.
    #   ~9.8 m/s² → leave false (driver publishes raw)
    #   ~0.0 m/s² → set true (driver already removed it)
    # NOTE: opposite of robot_localization's imu0_remove_gravitational_acceleration.

    imu.frame_id: ""  # override IMU TF frame. Leave empty to use msg header.frame_id.
                      # Set when your simulator or IMU driver stamps messages with a
                      # non-standard frame name (e.g. Gazebo Harmonic TurtleBot3 publishes
                      # "waffle/imu_link/tb3_imu"). Set to your URDF frame name instead.

    # Optional second IMU. When non-empty, FusionCore subscribes to this topic
    # and fuses each message as an independent measurement of the same state.
    # Both IMUs must be in (or TF-transformable to) base_link frame.
    # The second IMU uses the same noise model as the primary.
    # Useful when your platform has two IMUs and you want redundancy without
    # pre-merging them with imu_filter_madgwick or similar. Leave empty to disable.
    imu2.topic: ""
    imu2.frame_id: ""
    imu2.remove_gravitational_acceleration: false

    # ── Wheel encoders ────────────────────────────────────────────────────────
    encoder.vel_noise: 0.05     # m/s
    encoder.yaw_noise: 0.02     # rad/s

    # Optional second velocity source (lidar odometry, visual odometry, etc.)
    # Must publish nav_msgs/Odometry with velocity in twist field.
    # FusionCore does NOT accept sensor_msgs/LaserScan or PointCloud2 directly.
    # A scan-matching node must sit between your LiDAR and FusionCore:
    #
    #   LaserScan / PointCloud2
    #       → KISS-ICP             → /kiss/odometry      (nav_msgs/Odometry)
    #       → rtabmap icp_odometry → /icp_odom           (nav_msgs/Odometry)
    #       → rf2o_laser_odometry  → /odom               (nav_msgs/Odometry)
    #
    # Note: slam_toolbox publishes PoseWithCovarianceStamped, not Odometry.
    # It cannot be used directly as encoder2; wrap it or use KISS-ICP instead.
    encoder2.topic: ""          # e.g. "/kiss/odometry" or "/icp_odom"
    encoder2.vel_noise: 0.05    # m/s  fallback when message covariance is zero
    encoder2.yaw_noise: 0.02    # rad/s fallback when message covariance is zero

    # ── GPS ───────────────────────────────────────────────────────────────────
    gnss.base_noise_xy: 1.0     # m: baseline sigma at HDOP=1
                                # scaled automatically by HDOP from the message
                                # standard autonomous GPS: 1.0–2.5
                                # RTK float: 0.5,  RTK fixed: 0.015
    gnss.base_noise_z: 2.0      # m
    gnss.heading_noise: 0.02    # rad: for dual antenna heading

    gnss.max_hdop: 4.0          # reject fixes with HDOP worse than this
    gnss.min_satellites: 4
    gnss.min_fix_type: 1        # 1=GPS, 2=DGPS, 3=RTK_FLOAT, 4=RTK_FIXED
                                # NavSatFix: status=2 maps to RTK_FIXED. RTK_FLOAT (3)
                                # is unreachable via NavSatFix — use gnss.use_gps_fix
                                # below if your receiver publishes gps_msgs/GPSFix.

    gnss.use_gps_fix: false     # Set true when your driver publishes gps_msgs/GPSFix
                                # on /gnss/fix instead of sensor_msgs/NavSatFix.
                                # GPSFix unlocks RTK_FLOAT status, uses receiver-native
                                # hdop/vdop values, satellites_used, and err_horz/err_vert
                                # as a fallback covariance. Default false: NavSatFix works
                                # with all receivers. See GPS Receiver Setup below.

    # Antenna lever arm: offset from base_link to GPS antenna in body frame
    # x=forward, y=left, z=up (meters). Leave 0.0 if antenna is above base_link.
    # Correction only activates after heading is independently validated.
    gnss.lever_arm_x: 0.0
    gnss.lever_arm_y: 0.0
    gnss.lever_arm_z: 0.0

    # Second GPS receiver (optional)
    gnss.fix2_topic: ""
    gnss.lever_arm2_x: 0.0
    gnss.lever_arm2_y: 0.0
    gnss.lever_arm2_z: 0.0

    # GPS velocity (optional): fuses horizontal GPS speed as an independent
    # measurement update, separate from wheel odometry.
    # Accepts nav_msgs/Odometry with velocity in ENU frame:
    #   twist.linear.x = east velocity (m/s)
    #   twist.linear.y = north velocity (m/s)
    #   twist.covariance[0,7] used when positive; falls back to encoder noise otherwise
    # FusionCore rotates ENU -> body frame internally using the current quaternion.
    # Enables slip detection: innovation between GPS velocity and wheel odometry
    # reveals wheel slip. Works with F9P, Septentrio, or any receiver that
    # publishes velocity. Leave empty to disable.
    gnss.velocity_topic: ""

    # Radar Doppler velocity (optional): fuses ego-velocity from a 4D imaging radar
    # (Continental ARS548, Oculii Eagle, Aptiv ESR, etc.) as an independent measurement.
    # Accepts nav_msgs/Odometry with velocity in robot body frame:
    #   linear.x = forward speed (m/s), linear.y = lateral speed (m/s)
    # A bridge node extracts ego-velocity from raw Doppler point cloud and publishes here.
    # Works indoors and in all weather: rain, fog, dust, darkness.
    # radar.vel_noise is used when the message covariance is zero or negative.
    radar.velocity_topic: ""
    radar.vel_noise: 0.1        # m/s fallback noise when message has no covariance

    # Heading input: pick one or both
    gnss.heading_topic: ""      # sensor_msgs/Imu (dual antenna heading)
    gnss.azimuth_topic: ""      # compass_msgs/Azimuth (preferred REP-145 standard)

    # ── GPS coast mode ────────────────────────────────────────────────────────
    # During GPS blackouts, inflates process noise so P grows and the chi2 gate
    # relaxes by the time GPS resumes. Prevents the filter from rejecting its
    # own recovery fixes because dead-reckoning drift made innovations look like
    # outliers. See How It Works for the full explanation.

    gnss.coast_n: 3
    # Consecutive chi2 GPS rejections before entering coast mode. 0 = disabled.

    gnss.coast_q_factor: 10.0
    # Q_position multiplier in coast mode. Controls how fast position uncertainty
    # grows during the blackout. 10.0: after 228s, sigma_xy=48m (rejects 840m
    # outliers, accepts 193m drift). After 461s, sigma_xy=68m (accepts 274m drift).

    gnss.coast_timeout_s: 30.0
    # Also enter coast if GPS is silent this long (seconds). Handles outages
    # where the receiver stops publishing entirely. 0.0 = timeout trigger disabled.

    gnss.coast_q_bias_factor: 100.0
    # Q_gyro_bias multiplier in coast mode. Loosens bias confidence so encoder
    # WZ can drive fast heading bias correction during the blackout. 100.0 typical.

    gnss.coast_imu_wz_scale: 500.0
    # R_imu[WZ,WZ] multiplier in coast mode. Makes IMU heading rate less trusted
    # so encoder WZ dominates. 1.0 = disabled. 500.0 typical for long blackouts.

    gnss.recovery_rejection_n: 0
    # After this many consecutive rejections, inflate P[x,x] and P[y,y] directly.
    # Fires once per cascade. Must be > gnss.coast_n. 0 = disabled. Typical: 15.

    gnss.p_inflate_sigma: 50.0
    # XY sigma used for the P inflation above (meters). Only used when
    # gnss.recovery_rejection_n > 0.

    gnss.recovery_timeout_s: 0.0
    # GPS absence (seconds) before entering position-injection recovery mode, which
    # bypasses chi2 for the first returning fix. Useful only when blackouts are very
    # long (>200s) AND GPS outliers are not a concern at that location.
    # 0.0 = disabled (chi2 always active, recommended). Must be >= coast_timeout_s.

    # ── GPS track heading fusion ──────────────────────────────────────────────
    gnss.track_heading_enabled: true
    # Fuses GPS displacement bearing as a yaw pseudo-measurement whenever the
    # robot has moved gnss.track_heading_min_dist meters since the last fusion.
    # This is the primary mechanism for estimating encoder WZ bias without a
    # dual-antenna GPS. Disable only with an independent heading source.

    gnss.track_heading_min_dist: 5.0
    # Minimum GPS displacement (m) between heading fusions.

    gnss.track_heading_max_sigma: 0.4
    # Maximum heading uncertainty to allow a fusion (radians). Computed as
    # gps_noise / displacement. 0.4 rad = 23 degrees.

    # ── Outlier rejection ─────────────────────────────────────────────────────
    outlier_rejection: true
    outlier_threshold_gnss: 16.27   # chi2(3, 0.999): 3D GPS position
    outlier_threshold_imu:  15.09   # chi2(6, 0.999): 6D IMU (gyro + accel)
                                   # If imu.has_magnetometer: false, IMU only fuses roll/pitch (DOF=2).
                                   # Lower to 13.82 (chi2(2, 0.999)) to maintain 99.9% confidence.
    outlier_threshold_enc:  11.34   # chi2(3, 0.999): 3D encoder
    outlier_threshold_hdg:  10.83   # chi2(1, 0.999): 1D heading
    outlier_threshold_vslam: 22.46  # chi2(6, 0.999): 6D VSLAM pose
    # Keep vslam gate at chi2(6, 0.999). ORB-SLAM3 can jump on reinitialization:
    # this gate rejects those jumps automatically when covariance is calibrated.
    # Do NOT lower these below chi2 critical values. At 7.0 normal GPS noise
    # trips the gate and every fix gets rejected.

    # ── VSLAM (visual SLAM pose input) ───────────────────────────────────────
    # FusionCore accepts 6-DOF pose from any VSLAM system that publishes
    # nav_msgs/Odometry (ORB-SLAM3, RTAB-Map, Kimera, OpenVINS, etc.).
    # Only pose.pose and pose.covariance are used; twist is ignored.
    # See docs/hardware/vslam-imu.md for setup details.
    vslam.topic: ""              # e.g. "/vslam/odometry" or "/orbslam3/camera/odometry"
                                 # Leave empty to disable VSLAM input.
    vslam.position_noise: 0.1   # m: fallback when message covariance is zero
    vslam.orientation_noise: 0.02  # rad: fallback when message covariance is zero
    vslam.frame_id: ""           # override VSLAM TF frame. Leave empty to use msg header.
    vslam.reinit_n: 10           # consecutive gate rejections before re-anchoring map origin
    # FusionCore tracks the offset between the VSLAM map frame and the filter's odom
    # frame. When VSLAM reinitializes after tracking loss, its pose jumps to a new
    # map origin. The chi-squared gate rejects these jumps. After vslam.reinit_n
    # consecutive rejections, FusionCore assumes reinitialization occurred and
    # re-anchors the map origin to the filter's current position, restoring fusion.

    # ── GPS coast mode (cascade rejection recovery) ───────────────────────────
    gnss.coast_n: 5              # consecutive rejections before entering coast mode
    gnss.coast_q_factor: 20.0   # process noise multiplier while coasting (inflates P)
    gnss.degraded_noise_multiplier: 3.0
    # After gnss.coast_n consecutive GPS outliers, the filter enters coast mode.
    # In coast mode: (1) P is inflated by coast_q_factor each step so the filter
    # stays open to correction, (2) the next fix is tested against a gate inflated
    # by degraded_noise_multiplier, giving it a wider acceptance window.
    # This breaks the cascade rejection loop where a stationary or slowly-drifting
    # filter keeps rejecting valid fixes because its covariance is too tight.
    # Increase coast_n if you want more patience before relaxing; increase
    # degraded_noise_multiplier if large GPS jumps should still be accepted.

    # ── Adaptive noise ────────────────────────────────────────────────────────
    adaptive.imu: true
    adaptive.encoder: true
    adaptive.gnss: true
    adaptive.window: 50       # sliding window size (updates, not seconds)
    adaptive.alpha: 0.01      # EMA learning rate. 0.01 = slow, stable.

    # ── UKF process noise ─────────────────────────────────────────────────────
    # These are per-predict-step noise values (not spectral densities).
    # At 100Hz IMU, the filter predicts 100 times per second. Each step adds
    # Q to P, so effective noise rate = q_* * 100 per second.

    ukf.q_position: 0.01
    ukf.q_orientation: 1.0e-9   # quaternion regularization ONLY: do not increase.
                                 # Orientation uncertainty propagates from q_angular_vel
                                 # through the kinematics. Large values here corrupt
                                 # quaternion norm and cause yaw/Z drift.
    ukf.q_velocity: 0.1
    ukf.q_angular_vel: 0.1
    ukf.q_acceleration: 1.0
    ukf.q_gyro_bias: 1.0e-5     # biases change slowly (MEMS thermal drift)
    ukf.q_accel_bias: 1.0e-5
    ukf.q_encoder_wz_bias: 1.0e-7  # encoder WZ bias is mechanical: very stable.
                                    # Smaller than gyro bias because it changes only
                                    # with physical wear, not thermal effects.

    # ── Startup ───────────────────────────────────────────────────────────────
    init.stationary_window: 0.0
    # Seconds of IMU data to collect before starting (robot must be stationary).
    # Estimates accelerometer bias at startup → reduces 60s warmup transient
    # from ~10cm to under 1cm. Set 2.0 if startup drift is a problem.
    # Falls back to zero-bias automatically if robot moves during window.

    init.wait_for_all_sensors: false
    # When true: hold filter initialization until every configured sensor has
    # published at least one message. Prevents the filter from drifting on IMU
    # alone while GPS and wheel odometry are still coming online at startup.
    # Replaces the sleep() workaround in launch files.
    init.sensor_wait_timeout: 10.0
    # Seconds to wait before starting anyway if a sensor never arrives.
    # A WARN lists which sensors were missing. Set 0.0 to disable the timeout.

    # ── Motion model ──────────────────────────────────────────────────────────
    motion_model: "ConstantVelocityAcceleration"
    # Controls how sigma points are propagated in the UKF predict step.
    #
    # "ConstantVelocityAcceleration" (default): no platform constraints.
    #   VY and AY grow freely between measurements. Correct for aerial vehicles.
    #   Good baseline for any platform.
    #
    # "DifferentialDrive": zeros VY and AY each predict step.
    #   The filter knows a diff-drive robot cannot slide sideways.
    #   Tighter lateral covariance, less position smear on straight runs.
    #   Use for: differential drive, skid-steer, tracked vehicles.
    #
    # "Ackermann": same lateral constraint as DifferentialDrive.
    #   wheelbase stored for future minimum-turning-radius extensions.
    #   Use for: car-like robots, forklifts, front-steered outdoor platforms.
    motion_model_params.wheelbase: 0.55    # metres (only used by Ackermann)

    # ── Deterministic replay ──────────────────────────────────────────────────
    replay.checkpoint_path: "/tmp/fusioncore_checkpoint.txt"
    # File used by ~/save_checkpoint and ~/load_checkpoint services.
    # save_checkpoint: writes the full 23-state + 23x23 covariance to this file.
    # load_checkpoint: restores that state (restarts filter from that point).
    # Workflow: replay a bag to a known-good point → save → tweak params →
    #   load (instant, no re-replay) → observe the problem window.

    # ── Ground constraint ─────────────────────────────────────────────────────
    # FusionCore always fuses VZ=0 and AZ=0 on every encoder callback.
    # These are unconditional for wheeled ground robots and cannot be disabled.
    #
    # The optional Z position constraint targets a different problem: GPS
    # altitude noise (typically 3-5m std dev) causing the filter's Z position
    # to oscillate on flat terrain. When enabled, it fuses Z=0 as a position
    # pseudo-measurement tighter than GPS altitude noise, keeping the filter
    # anchored to the ground.
    #
    # 0.0 = disabled (default, correct for 3D outdoor operation or unknown terrain)
    # 0.3 = flat terrain mode: campus paths, parking lots, warehouse floors
    ground_constraint.z_position_sigma: 0.0

    # ── ZUPT ──────────────────────────────────────────────────────────────────
    zupt.enabled: true
    zupt.velocity_threshold: 0.05   # m/s: encoder speed below this → stationary
    zupt.angular_threshold: 0.05    # rad/s: angular rate below this → not rotating
    zupt.noise_sigma: 0.01          # m/s: tighter = stronger zero-velocity correction

    # ── GPS coordinate system ─────────────────────────────────────────────────
    input.gnss_crs: "EPSG:4326"              # WGS84 lat/lon (standard GPS)
    output.crs: "EPSG:4978"                  # ECEF XYZ (globally valid default)
    output.convert_to_enu_at_reference: true # required when output.crs is ECEF
    reference.use_first_fix: true            # map origin = first GPS fix
    reference.x: 0.0                         # fixed origin (when use_first_fix: false)
    reference.y: 0.0
    reference.z: 0.0
```

---

## Choosing a motion model

Start with the default (`ConstantVelocityAcceleration`) unless you have a specific reason to change it. It works well for all platforms and matches what robot_localization users are used to.

Switch to `DifferentialDrive` if:
- Your robot is a differential drive, skid-steer, or tracked vehicle
- You see small lateral position drift on straight runs
- Your `VY` state doesn't stay near zero between encoder updates

Switch to `Ackermann` if:
- Your robot has front-wheel steering (car-like, forklift, outdoor field robot)
- The lateral constraint is the same as `DifferentialDrive`; `wheelbase` is stored for future minimum-turning-radius enforcement

Do not use `DifferentialDrive` or `Ackermann` for:
- Aerial vehicles (no lateral constraint applies)
- Holonomic (mecanum) platforms (those can move sideways intentionally)

---

## Wait for all sensors: replacing sleep() in launch files

A common pattern in ROS launch files is `sleep(3)` before starting the navigation stack to give sensors time to come online. This is fragile: on a slow machine the sensors might need 5 seconds, and on a fast one you waste 3 seconds on every launch.

`init.wait_for_all_sensors: true` replaces this entirely. FusionCore holds initialization until it has seen at least one message from every sensor you configured. Then it starts. No sleep needed.

```yaml
init.wait_for_all_sensors: true
init.sensor_wait_timeout: 10.0
```

The timeout is a safety net: if a sensor fails to start, the filter does not hang forever. It logs which sensors were missing and starts anyway:

```
[WARN] Sensor wait timed out after 10.0s. Missing: [GNSS]. Starting anyway.
```

This is especially useful at competition startup, on-site robot power-on, or any deployment where sensor initialization order is not guaranteed.

---

## Deterministic replay: debugging without hardware

See [How It Works: Deterministic replay](how-it-works.md#deterministic-replay-and-state-checkpoints) for the full workflow. Quick reference:

```bash
# Save filter state at any point during bag replay
ros2 service call /fusioncore/save_checkpoint std_srvs/srv/Trigger

# Restore that state instantly (no need to replay from the beginning)
ros2 service call /fusioncore/load_checkpoint std_srvs/srv/Trigger
```

The checkpoint file path is set by `replay.checkpoint_path` (default `/tmp/fusioncore_checkpoint.txt`).

---

## GPS without a fix (indoor / no GPS)

Set `reference.use_first_fix: false` and leave reference coords at 0.0. The filter starts at the origin and runs on IMU + wheel odometry alone. No GPS topics needed.

---

## Agricultural RTK (UTM output)

Some RTK receivers output easting/northing directly:

```yaml
input.gnss_crs: "EPSG:32617"        # UTM zone 17N (adjust for your zone)
output.crs: "EPSG:32617"
output.convert_to_enu_at_reference: false
reference.use_first_fix: true
```

---

## GPS receiver setup: NavSatFix vs GPSFix

FusionCore supports two GPS message types on `/gnss/fix`. The default is `sensor_msgs/NavSatFix` because every ROS GPS driver publishes it. Set `gnss.use_gps_fix: true` to switch to `gps_msgs/GPSFix` if your driver supports it.

| | `sensor_msgs/NavSatFix` | `gps_msgs/GPSFix` |
|---|---|---|
| Driver support | Universal | nmea_navsat_driver, ublox_dgnss, others |
| RTK_FLOAT status | Not expressible | Yes (status 20) |
| Separate HDOP / VDOP | No | Yes |
| Satellites used | No | Yes |
| 95% CI error bounds | No | err_horz / err_vert |
| Covariance matrix | Yes | Yes |

### When to use NavSatFix (default)

NavSatFix works with all receivers. For most setups, leave `gnss.use_gps_fix: false`.

The only thing you cannot get via NavSatFix is RTK_FLOAT. If you are using autonomous GPS (CEP 1-3m) or RTK fixed, NavSatFix is all you need.

### When to use GPSFix

Switch to `gnss.use_gps_fix: true` when:

- Your receiver can output RTK_FLOAT and you want to fuse those fixes (better than autonomous, worse than RTK fixed). Set `gnss.min_fix_type: 3` to require it or allow it.
- Your driver publishes receiver-native HDOP/VDOP rather than a covariance matrix, and you want those values used directly in the noise model.
- Your driver sets `err_horz`/`err_vert` (95% CI bounds) and you prefer that over a synthetic covariance.

```yaml
fusioncore:
  ros__parameters:
    gnss.use_gps_fix: true
    gnss.min_fix_type: 3      # require RTK_FLOAT or better (3=FLOAT, 4=FIXED)
    gnss.base_noise_xy: 0.5   # metres: baseline at HDOP=1 for RTK_FLOAT
    gnss.base_noise_z: 1.0
```

### Covariance priority (GPSFix)

When `gnss.use_gps_fix: true`, FusionCore picks the best available covariance source in this order:

1. `position_covariance_type == 3` (full 3x3): used directly, including off-diagonal terms.
2. `position_covariance_type >= 1` (diagonal): diagonal elements used, hdop/vdop derived from them.
3. `err_horz > 0` and `err_vert > 0`: 95% CI bounds converted to 1-sigma variance (divide by 1.96), used as a diagonal covariance.
4. `hdop > 0` and `vdop > 0`: receiver-native DOP values used directly in the noise model (`sigma_xy = base_noise_xy * hdop`).
5. Defaults (`hdop=1.5, vdop=2.0`).

---

!!! warning "Upgrading from an older config"
    If your YAML has `ukf.q_orientation: 0.01`, change it to `1.0e-9` or delete the line. The old value corrupts quaternion math at typical IMU rates and causes yaw drift and Z-axis rise in simulation.
