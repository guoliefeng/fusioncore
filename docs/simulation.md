# Simulation

FusionCore ships with a Gazebo Harmonic simulation world so you can test the full fusion pipeline without physical hardware. It includes a differential drive robot with a 100 Hz IMU and GPS, in an outdoor environment with the GPS origin set to Hamilton, Ontario.

!!! note "Gazebo NavSat bug"
    Gazebo Harmonic's built-in NavSat sensor has a known bug (gz-sim issue #2163) where it periodically outputs GPS fixes at completely wrong coordinates. The simulation works around this by deriving GPS from Gazebo's ground truth world pose and adding realistic Gaussian noise (0.5 m horizontal, 0.3 m vertical 1-sigma).

---

## Prerequisites

Gazebo Harmonic and the ROS-Gazebo bridge are not installed by `rosdep` automatically. Install them first:

```bash
sudo apt install ros-jazzy-ros-gz
```

---

## Running the simulation

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
ros2 launch fusioncore_gazebo fusioncore_gazebo.launch.py
```

Drive the robot and watch the fused position:

```bash
# Terminal 2: drive in a circle
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.3}}" --rate 10

# Terminal 3: watch position
ros2 topic echo /fusion/odom --field pose.pose.position
```

---

## Automated integration tests

```bash
python3 ~/ros2_ws/src/fusioncore/fusioncore_gazebo/launch/integration_test.py
```

Four tests run automatically against the live simulation. Each has a hard pass/fail threshold:

| Test | What it does | Pass threshold |
|---|---|---|
| **IMU dead reckoning** | Robot stationary for 10 s with GPS active. Measures position drift from IMU noise alone. | Drift < 2.0 m |
| **GPS spike rejection** | Publishes one corrupted GPS fix at +500 m north. Measures how far the filter moves in response. | Position jump < 5.0 m |
| **GPS correction** | Robot drives forward 3 s then stops. Measures position stability 3 s after stopping with GPS active. | Drift < 2.0 m |
| **Circle return** | Robot drives one full circle (radius ~0.5 m). Measures distance between start and end position. | Return error < 3.0 m |

Expected output on a clean session:

```
══════════════════════════════════════════════════
  FUSIONCORE INTEGRATION TEST SCORECARD
══════════════════════════════════════════════════
  [PASS] ✓ IMU dead reckoning
           0.041m drift in 10s stationary
  [PASS] ✓ Outlier rejection
           0.312m jump on 500m GPS outlier
  [PASS] ✓ GPS correction
           0.018m drift after stop with GPS active
  [PASS] ✓ Circle return
           0.247m from start after full circle
══════════════════════════════════════════════════
  Overall: ALL TESTS PASSED ✓
══════════════════════════════════════════════════
```

The spike rejection test (0.3 m movement on a 500 m corrupted fix) directly verifies the chi-squared gate described in [How It Works](../how-it-works.md#mahalanobis-outlier-rejection). The same scenario is shown visually in the [zero-dependency demo](../index.md#see-it-before-you-install).
