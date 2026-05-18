# Benchmark Results: NCLT Sequence 2012-06-15

## Metrics (SE3-aligned to RTK ground truth)

| Filter | ATE RMSE 3D (m) | ATE RMSE XY (m) | Within 5 m | Within 10 m | Path Length Ratio | Drift (m/km) | RPE@10m RMSE (m) |
|--------|----------------|----------------|------------|-------------|-------------------|--------------|------------------|
| FusionCore | 49.232 | 48.384 | 2.4% | 20.0% | 0.7941 | 8.40 | 22.362 |
| RL-EKF | 18.235 | 17.120 | 42.8% | 78.4% | 0.7788 | 3.11 | 22.331 |

## Methodology

- Dataset: NCLT (University of Michigan)
- Sequence: 2012-06-15
- Ground truth: RTK GPS (gps_rtk.csv) projected to local ENU
- Evaluation: [evo](https://github.com/MichaelGrupp/evo), SE(3) alignment
- All filters consume identical sensor streams: same IMU, wheel odometry, and GPS topics
- FusionCore: full 3D UKF, adaptive noise, ZUPT, IMU bias estimation
- RL-EKF: two_d_mode=true (flat-terrain Segway RMP), GPS via navsat_transform
- RL-UKF excluded: robot_localization UKF diverges under high-rate sim time playback
  (rapid timer catchup causes near-zero dt between predictions, Cholesky failure, immediate NaN)
