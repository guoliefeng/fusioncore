# Comparison configs

This directory contains the robot_localization configuration used in the FusionCore hardware video comparison and the NCLT benchmark.

## Files

| File | Purpose |
|---|---|
| `rl_ekf_outdoor.yaml` | EKF node config for outdoor robot comparison (ZED-F9P + BNO085 + wheel odometry) |
| `navsat_transform_outdoor.yaml` | navsat_transform config paired with the above |
| `../../fusioncore_datasets/config/rl_ekf.yaml` | NCLT benchmark EKF config |
| `../../fusioncore_datasets/config/navsat_transform.yaml` | NCLT navsat_transform config |

## Improving the robot_localization config

This is our best-faith attempt at a well-configured rl setup for the same task. If you know a better way to configure it for outdoor GPS fusion, open a PR.

Specifically, contributions welcome for:
- Better process noise covariance values for a typical differential-drive outdoor robot
- IMU config flags if the BNO085 NDOF mode setup is suboptimal
- navsat_transform parameters that reduce startup sensitivity

We will update the config and re-run the comparison with any merged improvements.

## Fairness approach

Both filters receive the same rosbag data, same sensor topics, same ground truth.

What is different by design (these are architecture differences, not tuning choices):
- FusionCore has gyro and accel bias states in the filter; robot_localization does not
- FusionCore adapts GPS noise from the innovation sequence; robot_localization uses the sensor-reported covariance as-is
- FusionCore uses chi-squared gates calibrated per sensor DOF; robot_localization uses unsquared Mahalanobis scalars

The Mahalanobis rejection thresholds in rl_ekf_outdoor.yaml are set to chi-squared critical values at 99.9% confidence, matching FusionCore's defaults, so both filters reject at the same statistical confidence level.

Full fairness statement: [benchmarks/comparison/DISCLAIMER.md](DISCLAIMER.md)
