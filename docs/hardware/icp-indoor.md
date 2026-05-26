# Indoor / GPS-denied robots (LiDAR ICP odometry)

For robots operating indoors, underground, or anywhere GPS isn't available. FusionCore runs fine without GPS: it starts at the origin and builds a local odometry frame from IMU + LiDAR ICP alone.

---

## Common setups this covers

- Indoor mobile robots (warehouses, offices, labs)
- Robots with a LiDAR running KISS-ICP or rtabmap `icp_odometry`
- Robots where wheel odometry is unreliable or unavailable
- OAK-D camera + Velodyne/RPLiDAR + any ground robot

---

## Quick start

```bash
ros2 launch fusioncore_ros fusioncore.launch.py \
  fusioncore_config:=$(ros2 pkg prefix fusioncore_ros)/share/fusioncore_ros/config/icp_indoor.yaml \
  --ros-args \
  -r /odom/wheels:=/kiss/odometry
```

Replace `/kiss/odometry` with whatever your ICP pipeline publishes:

| ICP pipeline | Default topic | Type |
|---|---|---|
| KISS-ICP | `/kiss/odometry` | `nav_msgs/Odometry` |
| rtabmap `icp_odometry` | `/icp_odom` | `nav_msgs/Odometry` |
| rf2o_laser_odometry | `/odom` | `nav_msgs/Odometry` |
| cartographer | `/tracked_pose` | `PoseStamped` (needs twist wrapper) |
| slam_toolbox | `/pose` | `PoseWithCovarianceStamped` (needs twist wrapper) |

> **FusionCore does not accept `sensor_msgs/LaserScan` or `PointCloud2` directly.**
> A scan-matching node must convert your LiDAR data to `nav_msgs/Odometry` first.
> Nodes that publish pose-only (cartographer, slam_toolbox) need a wrapper that
> differentiates the pose to produce a velocity in the twist field.

FusionCore treats LiDAR ICP odometry exactly like wheel odometry: same outlier gate, same adaptive noise, same ZUPT logic. The only difference is the noise values: ICP is more accurate than wheels so `icp_indoor.yaml` uses tighter defaults (`encoder.vel_noise: 0.02` instead of `0.05`).

---

## If you also have wheel odometry

If your wheel odom is reliable, use it as primary and add ICP as a second cross-check via `encoder2.topic`:

```yaml
# in icp_indoor.yaml
encoder2.topic: "/kiss/odometry"
```

Then launch without the remap: FusionCore subscribes to `/odom/wheels` natively and picks up ICP as a second velocity source.

---

## Using RTABMAP alongside FusionCore (Madgwick separation)

RTABMAP's `icp_odometry` and the `rtabmap_slam` node both expect IMU data on `/imu/data`. FusionCore also subscribes to `/imu/data`: but it needs **raw** gyro and accelerometer data, not a Madgwick-filtered orientation.

If you run `imu_filter_madgwick` in your stack, do **not** remap FusionCore's IMU input to the Madgwick output. The Madgwick filter bakes in a coordinate frame rotation based on its world-frame orientation estimate. When FusionCore receives that, it tries to fuse an already-rotated orientation as if it were raw rates: the result is the robot appearing to spin or roll when it isn't.

**Correct setup:**

```
/imu  (raw from driver)
  ├── → FusionCore    (via -r /imu/data:=/imu)
  └── → imu_filter_madgwick  → /imu/data  → RTABMAP + icp_odometry
```

In your launch file:

```python
# FusionCore reads raw IMU directly
fc = LifecycleNode(
    ...
    remappings=[
        ("/imu/data", "/imu"),     # raw, not Madgwick output
    ]
)

# Madgwick runs separately for RTABMAP and ICP
Node(
    package='imu_filter_madgwick', executable='imu_filter_madgwick_node',
    parameters=[{'use_mag': False, 'world_frame': 'enu', 'publish_tf': False}],
    remappings=[('imu/data_raw', '/imu')],   # reads same raw /imu
),
```

FusionCore handles orientation estimation internally from raw gyro + accel. Madgwick is only needed to give RTABMAP and ICP odometry the filtered `/imu/data` they expect.

---

## SLAM integration (slam_toolbox, RTABMAP)

FusionCore and your SLAM system divide the TF tree cleanly:

```
map → odom        ← your SLAM system (slam_toolbox, RTABMAP)
odom → base_link  ← FusionCore
```

FusionCore handles the fast, smooth odometry estimate. Your SLAM system handles global map corrections. They don't conflict and don't need to know about each other.

Nav2 reads from `/fusion/odom` as usual.

### slam_toolbox

```bash
# Terminal 1: FusionCore
ros2 launch fusioncore_ros fusioncore.launch.py \
  fusioncore_config:=icp_indoor.yaml \
  --ros-args -r /odom/wheels:=/kiss/odometry

# Terminal 2: slam_toolbox
ros2 launch slam_toolbox online_async_launch.py
```

### RTABMAP with ICP odometry

When using `rtabmap_odom icp_odometry` alongside FusionCore, the stack has three components: ICP odometry, FusionCore, and the RTABMAP SLAM node. Each has a distinct job.

**Wire RTABMAP SLAM to `icp_odom`, not FusionCore's output.**

FusionCore fuses ICP odometry with IMU to produce a smooth 100 Hz estimate. That's exactly what you want for the `odom → base_link` TF and for Nav2. But for building the map, you want scan-consistent odometry: each robot pose derived directly from comparing two consecutive scans. When the map is built from scan-consistent odometry, walls come out straight and geometric inconsistencies don't accumulate.

If RTABMAP is wired to FusionCore's fused output instead, two failure modes appear. First, any IMU bias that hasn't fully converged adds a slow lateral drift to the map even when ICP is working correctly. Second, if ICP throws a registration failure mid-run (common during fast rotation), FusionCore briefly runs on IMU alone. The IMU's uncorrected acceleration causes rapid divergence. Scans placed during that window land in the wrong location permanently, producing a starburst or explosion pattern in the map. See [SLAM map looks like a starburst](../troubleshooting.md#slam-map-looks-like-a-starburst-or-explosion-pattern) in troubleshooting.

With `icp_odom` wired directly to RTABMAP, ICP failures cause RTABMAP to pause rather than place scans at bad positions. The map accumulates only from frames where the position was geometrically reliable.

```python
Node(
    package='rtabmap_slam', executable='rtabmap', output='screen',
    parameters=[{
        'frame_id': 'base_link',   # the robot body frame, not a camera frame
        'subscribe_scan': True,
        'approx_sync': True,
    }],
    remappings=[
        ('odom', '/icp_odom'),     # SLAM uses ICP odometry for map building
        ('imu', '/imu/data'),
    ]
),
```

FusionCore continues owning the `odom → base_link` TF. Nav2 still reads `/fusion/odom`. The only change is what RTABMAP uses when deciding where to place each incoming scan.

**Set `frame_id` to `base_link`, not a camera frame name.**

The depthai launch files default to `frame_id: 'oak-d-base-frame'`. This builds the map relative to the camera origin rather than the robot center. During rotation, the map rotates around the camera instead of the robot, and loop closures produce geometric inconsistencies that accumulate over time. Use `base_link` regardless of which sensors are feeding RTABMAP.

**Startup delay when part of the TF tree comes from a separate machine.**

If your robot runs on two computers (one for sensors, one for compute), the TF frames published by the sensor machine need time to arrive over the network before RTABMAP starts looking for them. Add a 10-second startup delay for the RTABMAP SLAM node:

```python
TimerAction(
    period=10.0,
    actions=[Node(
        package='rtabmap_slam', executable='rtabmap', output='screen',
        parameters=[...],
        remappings=[...],
    )]
),
```

Without this delay, RTABMAP logs `Tf has two or more unconnected trees` at startup. The error is a race condition, not a real configuration problem, but it can prevent the map display from initializing correctly in RViz.

---

## IMU topic remaps

Most drivers publish at `/imu/data` already and need no remap. Exceptions:

| Driver | Topic | Remap needed |
|---|---|---|
| OAK-D (depthai-ros) | `/imu` | `-r /imu/data:=/imu` |
| Realsense D435i | `/camera/imu` | `-r /imu/data:=/camera/imu` |
| Most others | `/imu/data` | none |

Also set `imu.frame_id` in your config if your driver doesn't fill in `header.frame_id`:

```yaml
imu.frame_id: "oak_imu_frame"      # OAK-D
imu.frame_id: "camera_imu_optical_frame"  # Realsense
```

---

## OAK-D: correcting a physically tilted IMU

The OAK-D's BNO085 IMU is mounted inside the camera housing at roughly 85 degrees pitch relative to the camera body. The Luxonis URDF accounts for this in the `oak_imu_frame` definition. But if that transform doesn't reach FusionCore correctly (clock skew between machines, network delay, or a URDF not matching the actual mount angle), FusionCore applies the wrong rotation and gravity lands in the wrong axis.

**How to tell if this is your problem:** with the robot completely still, run:

```bash
ros2 topic echo /imu --field linear_acceleration --once
```

The `linear_acceleration` values are the raw sensor output before FusionCore applies any transform. If `x` is near 9.81 (positive or negative) and `z` is near zero, the IMU's X axis is pointed roughly up or down and the rotation hasn't been applied yet.

**The fix:** create a new TF frame that describes the IMU's actual orientation relative to `base_link`, and run it on the same machine as FusionCore:

```python
Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=[
        '--x', '0.101', '--y', '-0.015', '--z', '0.066',
        '--roll', '0.0', '--pitch', '1.483', '--yaw', '0.0',
        '--frame-id', 'base_link',
        '--child-frame-id', 'oak_imu_correct'
    ]
),
```

Then in your YAML:

```yaml
imu.frame_id: "oak_imu_correct"
```

The pitch value of 1.483 radians (approximately 85 degrees) rotates the IMU's reporting axes so that gravity resolves into body-frame Z after the transform. Use a positive value. A negative value doubles the error in the opposite direction rather than correcting it.

**Verify the correction worked:** with the robot still, check the orientation output:

```bash
ros2 topic echo /fusion/odom --field pose.pose.orientation --once
```

When the robot is sitting flat on the ground, the quaternion should be close to `(x=0, y=0, z=0, w=1)`. Large roll or pitch values at rest mean the transform is still wrong.

**Why run this publisher on the same machine as FusionCore:** TF lookups depend on both machines agreeing on the current time. If the static transform publisher and FusionCore are on different devices whose clocks drift apart even briefly, FusionCore's TF lookup will time out or return stale data. Running the publisher on the same machine eliminates the cross-machine timing dependency entirely.

---

## One thing to get right: startup bias init

Without GPS corrections, IMU bias matters more indoors. Add this to your config:

```yaml
init.stationary_window: 2.0
```

This collects 2 seconds of stationary IMU data at startup to estimate bias before the filter starts. Reduces startup drift from ~10 cm to under 1 cm. Robot must be stationary for the first 2 seconds after launch: it falls back automatically if it detects movement.

The `icp_indoor.yaml` config has this enabled by default.
