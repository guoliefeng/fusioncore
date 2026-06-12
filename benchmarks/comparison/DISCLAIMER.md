# Fairness statement for FusionCore vs robot_localization comparisons

This statement accompanies every published FusionCore vs robot_localization comparison: the hardware video, the NCLT benchmark, and any future dataset.

## What we did to make this fair

**Same data.** Both filters run on identical rosbag files. Same IMU topic, same wheel odometry topic, same GPS topic, same timestamps.

**Same ground truth.** Trajectory error is computed against PPP post-processed GPS ground truth (NRCan CSRS-PPP, kinematic mode, centimeter-accurate). The same ground truth file is used for both filters.

**Same metric.** ATE RMSE computed with `evo_ape` using SE3 alignment. The evo command and alignment mode are identical for both filters.

**Same rejection confidence.** robot_localization's Mahalanobis rejection thresholds are set to chi-squared critical values at 99.9% confidence, matching FusionCore's defaults. Both filters reject outliers at the same statistical level.

**Documented rl config.** The robot_localization config is committed to the repo at `benchmarks/comparison/`. It is open to PRs. If there is a better way to configure rl for this task, we will update it and re-run.

## What is different by design

These are architectural differences, not tuning choices. We cannot remove them without changing what the software is.

**IMU bias estimation.** FusionCore tracks gyro bias and accelerometer bias as states in the 23D filter, estimating them continuously. robot_localization does not have bias states. Gyro drift accumulates silently into heading error in rl; FusionCore corrects it online. This is structural.

**Adaptive GPS noise.** FusionCore maintains a 50-sample innovation window per sensor and updates the noise model in real time: `R = (1-alpha)*R + alpha*C_hat`. robot_localization uses the GPS-reported covariance from the sensor message as-is and does not adapt from the innovation sequence. This is structural.

**chi-squared gate calibration.** FusionCore's gate thresholds are calibrated to the sensor's measurement DOF by default: chi2(3, 0.999)=16.27 for a 3-DOF GPS measurement. robot_localization exposes unsquared Mahalanobis scalars with no DOF guidance; users must set them manually. We matched FusionCore's statistical confidence level in the rl config, which is the most generous interpretation.

## What we would do differently if we were the robot_localization team

We are not affiliated with the robot_localization project and this is not a criticism. If we were maintaining rl and wanted to close the gap on these specific scenarios, the highest-value changes would be:

1. Increase the GPS covariance used in the filter when the GPS driver reports optimistic covariance (manual, but possible).
2. Use `smooth_lagged_data: true` with a longer `history_length` to handle delayed GPS fixes.
3. Add a pre-filter on GPS fix quality using HDOP from NavSatFix.status.

None of these changes require modifying robot_localization itself; they are all achievable through configuration. The NCLT benchmark includes a note on what covariance inflation would have helped rl.

## Reproducing the comparison

The rosbags, configs, and replay scripts will be published at [Zenodo](https://zenodo.org) alongside the hardware video. One command runs both filters on the same bag and produces the comparison metrics.

robot_localization is a solid, well-maintained package with a large user base. This comparison is an honest accounting of where different architectural choices lead on GPS-heavy outdoor navigation. It is not a claim that FusionCore is better in every scenario; the NCLT benchmark documents two sequences where robot_localization wins and explains why.
