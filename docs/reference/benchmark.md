# Benchmark Results

FusionCore vs robot_localization EKF on the [NCLT dataset](http://robots.engin.umich.edu/nclt/) (University of Michigan North Campus Long-Term). Nine sequences, same sensor pipeline, no per-sequence tuning.

---

## Summary: 9 sequences, 7 FC wins

| Sequence | Duration | FC ATE (3D) | RL-EKF ATE (3D) | Winner |
|---|---|---|---|---|
| 2012-01-08 | 92 min | **18.6 m** | 41.2 m | FC +55% |
| 2012-02-04 | 77 min | **49.7 m** | 265.5 m | FC +81% |
| 2012-03-31 | 87 min | **22.0 m** | 156.5 m | FC +86% |
| 2012-05-11 | 84 min | **9.7 m** | 11.5 m | FC +16% |
| 2012-06-15 | 55 min | 49.2 m | **18.2 m** | RL +63% |
| 2012-08-20 | 83 min | 98.3 m | **10.6 m** | RL +89% |
| 2012-09-28 | 77 min | **10.8 m** | 55.7 m | FC +81% |
| 2012-10-28 | 85 min | **29.9 m** | 60.0 m | FC +50% |
| 2012-11-04 | 79 min | **60.1 m** | 122.0 m | FC +51% |

**ATE = absolute trajectory error, SE3-aligned to RTK GPS ground truth.**

---

## Full metrics table

| Sequence | Filter | ATE 3D | ATE XY | Within 5m | Within 10m | Drift (m/km) | RPE@10m |
|---|---|---|---|---|---|---|---|
| 2012-01-08 | FusionCore | **18.6 m** | **16.9 m** | 26.7% | 74.5% | **2.55** | **22.5 m** |
|            | RL-EKF     | 41.2 m | 41.0 m | 22.8% | 76.5% | 5.64 | 25.3 m |
| 2012-02-04 | FusionCore | **49.7 m** | **31.5 m** | 6.4% | 33.4% | **5.96** | **30.0 m** |
|            | RL-EKF     | 265.5 m | 265.4 m | 0.0% | 0.1% | 31.84 | 44.3 m |
| 2012-03-31 | FusionCore | **22.0 m** | **20.2 m** | 19.9% | 67.3% | **2.27** | **21.4 m** |
|            | RL-EKF     | 156.5 m | 156.3 m | 0.2% | 0.6% | 16.16 | 42.7 m |
| 2012-05-11 | FusionCore | **9.7 m** | **4.9 m** | 45.9% | 82.6% | **1.05** | 19.0 m |
|            | RL-EKF     | 11.5 m | 9.0 m | 56.2% | 90.1% | 1.25 | **20.2 m** |
| 2012-06-15 | FusionCore | 49.2 m | 48.4 m | 2.4% | 20.0% | 8.40 | 22.4 m |
|            | RL-EKF     | **18.2 m** | **17.1 m** | 42.8% | 78.4% | **3.11** | **22.3 m** |
| 2012-08-20 | FusionCore | 98.3 m | 97.9 m | 0.1% | 13.8% | 13.08 | 53.7 m |
|            | RL-EKF     | **10.6 m** | **9.9 m** | 59.4% | 89.3% | **1.40** | **19.1 m** |
| 2012-09-28 | FusionCore | **10.8 m** | **7.5 m** | 31.4% | 76.9% | **1.50** | **23.7 m** |
|            | RL-EKF     | 55.7 m | 55.5 m | 1.7% | 25.1% | 7.73 | 28.0 m |
| 2012-10-28 | FusionCore | **29.9 m** | **21.1 m** | 19.9% | 59.7% | **3.69** | 40.6 m |
|            | RL-EKF     | 60.0 m | 59.6 m | 0.1% | 3.6% | 7.40 | **27.8 m** |
| 2012-11-04 | FusionCore | **60.1 m** | **59.2 m** | 3.8% | 29.5% | **9.86** | **32.3 m** |
|            | RL-EKF     | 122.0 m | 121.9 m | 0.0% | 0.0% | 20.02 | 37.0 m |

---

## Methodology

**Dataset:** NCLT (University of Michigan, 2012-2013). Wheeled robot (Segway RMP) driving on a large campus over multiple seasons. Raw CSV sensor files replayed at 3x real time via `nclt_player`.

**Sensors used (identical inputs to both filters):**
- IMU: Microstrain 3DM-GX3-45 at 100 Hz (raw specific force, no factory gravity removal)
- Wheel odometry: Segway RMP encoders at 100 Hz
- GPS: Novatel SPAN-CPT, ~3m CEP, 5 Hz

**Ground truth:** RTK GPS (`gps_rtk.csv`), projected to local ENU via PROJ/WGS84. Evaluation: [evo](https://github.com/MichaelGrupp/evo), SE(3)-aligned ATE.

**FusionCore config:** Single YAML file (`fusioncore_datasets/config/nclt_fusioncore.yaml`), same across all 9 sequences. No per-sequence tuning.

**RL-EKF config:** `two_d_mode: true` (flat-terrain Segway assumption), GPS fused via `navsat_transform` with a fixed datum from the first valid RTK fix. Matching chi-squared gating thresholds to FusionCore (`odom0_twist_rejection_threshold: 4.03`, `odom1_pose_rejection_threshold: 3.72`).

**RL-UKF:** Diverged with NaN on all sequences during sim-time playback (rapid timer catchup causes near-zero dt, Cholesky failure). Excluded from results.

---

## Why FusionCore wins on most sequences

RL-EKF fails badly on 2012-02-04, 2012-03-31, and 2012-09-28 (265m, 156m, 55m ATE vs FC's 49m, 22m, 10m). The common cause: RL-EKF takes GPS covariance directly from the `NavSatFix` message, which NCLT reports as `var_xy=9` (3m sigma). This is tighter than the actual noise, so valid GPS innovations fail RL's Mahalanobis gate and GPS gets rejected for long stretches. FusionCore's `gnss.base_noise_xy` parameter floors the measurement noise to match real sensor behavior, keeping chi2 statistics calibrated.

FusionCore's adaptive noise estimation (window=50, alpha=0.01) further adjusts GPS and encoder noise in real time when actual innovation variance diverges from the noise model. This helps on sequences with variable GPS quality (urban campus, tree cover, buildings).

---

## The two losses: honest analysis

### 2012-06-15 (FC 49.2m, RL 18.2m)

This 55-minute sequence has the lowest GPS density of the nine (15,594 fixes vs 30,000-46,000 on others) and contains a **461-second GPS blackout** (7.7 minutes). During that window the filter dead-reckons on encoder and IMU alone.

FC's coast mode inflates process noise during GPS absence so that the chi2 gate relaxes and the first valid returning fix is accepted. The coast/recovery logic was tuned for the majority of sequences, which have shorter or no blackouts. On this sequence specifically, the 461s blackout pushes the limit of what the current dead-reckoning maintains: heading error accumulates during the blackout to the point that returning GPS triggers a correction that briefly inflates ATE before the filter re-converges. RL-EKF handles this better on this specific sequence because its simpler state (2D mode, no bias states) accumulates less uncertainty over the blackout period.

This is an active area of work. The gyro bias estimation during coast mode (coast_q_bias_factor=100, encoder WZ correction) is intended to limit heading drift, and the parameters are being tuned for long-blackout robustness.

### 2012-08-20 (FC 98.3m, RL 10.6m)

This is the hardest sequence in the dataset for GPS-based localization. The raw GPS stream contains **105 mode-3 fixes that are 720-840m off the RTK ground truth** (visible in `gps.csv`, excluded from `gps_rtk.csv`). These outliers cluster in a 24-second window at the end of the second GPS blackout at t=66 min. The sequence has two blackouts: 228s at t=42.2 min and 211s at t=62.5 min.

Per-minute trajectory analysis of the current run shows:

| Time | FC error | Note |
|---|---|---|
| 0-42 min | 1-10 m | Normal GPS, both filters tracking correctly |
| 43-46 min | spike to ~100m, recovers | First blackout (228s): GPS errors up to ~70m at boundary |
| 47-62 min | 3-10 m | Full recovery, GPS tracking correctly |
| 63-67 min | spike to ~788m, recovers | Second blackout (211s): 105 fixes 720-840m off RTK at t=66 min |
| 68-82 min | 5-10 m | Full recovery for remaining 15 minutes |

The 98m ATE RMSE comes almost entirely from those two 2-3 minute transients. Outside those windows FC tracks at 5-10m, comparable to RL-EKF.

The root cause: FusionCore's coast mode inflates `Q_position` during GPS absence to keep the chi2 gate permissive for genuine drift correction on return. With the current `coast_q_factor=10`, position uncertainty after a 211s blackout is sigma_xy~46m. An 840m outlier produces chi2=333, which is strongly rejected. However, the outlier GPS cluster arrives precisely at the end of the blackout before a short ramp-back to correct GPS, and the filter's position prediction after 211s of dead-reckoning may not be accurate enough to reject intermediate outliers under all configurations tested.

RL-EKF avoids this because it does not inflate process noise during GPS absence, keeping P small and chi2 tighter. It also operates with navsat_transform which may filter some outliers before they reach the EKF.

**This is an active investigation.** The specific challenge is that `coast_q_factor` controls a tradeoff: too high and outliers slip through chi2; too low and valid corrective fixes are rejected after genuine drift. The 2012-08-20 GPS outlier pattern (outliers clustered exactly at a blackout boundary) is particularly adversarial for any chi2-based gating scheme.

---

## Reproduce

```bash
# Build
colcon build --packages-select fusioncore_core fusioncore_ros

# Run one sequence (auto-stops on playback complete, ~15-50 min at 3x)
bash benchmarks/run_one.sh 2012-01-08

# Results in:
# benchmarks/nclt/2012-01-08/results_full/BENCHMARK.md
```

Full tooling and configs in [`benchmarks/`](https://github.com/manankharwar/fusioncore/tree/benchmark/benchmarks).
