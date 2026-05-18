# Benchmark Results: NCLT Sequence 2012-08-20

## Metrics (SE3-aligned to RTK ground truth)

| Filter | ATE RMSE 3D (m) | ATE RMSE XY (m) | Within 5 m | Within 10 m | Path Length Ratio | Drift (m/km) | RPE@10m RMSE (m) |
|--------|----------------|----------------|------------|-------------|-------------------|--------------|------------------|
| FusionCore | 98.314 | 97.886 | 0.1% | 13.8% | 1.0937 | 13.08 | 53.730 |
| RL-EKF | 10.553 | 9.889 | 59.4% | 89.3% | 0.8672 | 1.40 | 19.090 |

## Methodology

- Dataset: NCLT (University of Michigan)
- Sequence: 2012-08-20
- Ground truth: RTK GPS (gps_rtk.csv) projected to local ENU
- Evaluation: [evo](https://github.com/MichaelGrupp/evo), SE(3) alignment
- All filters consume identical sensor streams: same IMU, wheel odometry, and GPS topics
- FusionCore: full 3D UKF, adaptive noise, ZUPT, IMU bias estimation
- RL-EKF: two_d_mode=true (flat-terrain Segway RMP), GPS via navsat_transform
- RL-UKF excluded: robot_localization UKF diverges under high-rate sim time playback
  (rapid timer catchup causes near-zero dt between predictions, Cholesky failure, immediate NaN)
