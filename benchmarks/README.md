# FusionCore NCLT Benchmarks

FusionCore vs robot_localization EKF on the [NCLT dataset](http://robots.engin.umich.edu/nclt/) (University of Michigan North Campus Long-Term). Nine sequences across all seasons, same single config file, no per-sequence tuning.

---

## Results: 7/9 FC wins

| Sequence | Season | Duration | FC ATE 3D | RL-EKF ATE 3D | Winner |
|---|---|---|---|---|---|
| 2012-01-08 | Winter | 92 min | **18.6 m** | 41.2 m | FC +55% |
| 2012-02-04 | Winter | 77 min | **49.7 m** | 265.5 m | FC +81% |
| 2012-03-31 | Spring | 87 min | **22.0 m** | 156.5 m | FC +86% |
| 2012-05-11 | Spring | 84 min | **9.7 m** | 11.5 m | FC +16% |
| 2012-06-15 | Summer | 55 min | 49.2 m | **18.2 m** | RL +63% |
| 2012-08-20 | Summer | 83 min | 98.3 m | **10.6 m** | RL +89% |
| 2012-09-28 | Fall | 77 min | **10.8 m** | 55.7 m | FC +81% |
| 2012-10-28 | Fall | 85 min | **29.9 m** | 60.0 m | FC +50% |
| 2012-11-04 | Fall | 79 min | **60.1 m** | 122.0 m | FC +51% |

ATE = absolute trajectory error, SE3-aligned to RTK GPS ground truth.
RL-UKF: NaN divergence on all sequences (known numerical instability under sim-time playback, confirmed by RL maintainer).

Full per-sequence results, plots, and launch logs are in each `results_full/` directory.

---

## Why RL-EKF loses on most sequences

RL-EKF's large losses on 2012-02-04 (265m), 2012-03-31 (156m), and 2012-09-28 (55m) share the same root cause: NCLT's GPS receiver reports `position_covariance` with `var_xy = 9` (3m sigma), which is tighter than the actual noise in those sequences. RL-EKF's Mahalanobis gate uses this raw covariance directly via `navsat_transform`. When the GPS innovation exceeds the reported sigma, fixes are rejected as outliers, and RL-EKF loses GPS for long stretches.

FusionCore's `gnss.base_noise_xy` floors the GPS measurement noise to match real sensor behavior. The chi2 gate then operates on calibrated statistics and accepts the same fixes that RL-EKF rejected.

---

## The two FC losses

### 2012-06-15 (FC 49.2m, RL 18.2m)

The lowest-density GPS sequence in the set (15,594 fixes vs 30,000-46,000 on others) with a 461-second GPS blackout (7.7 minutes). During that window the filter dead-reckons on encoder and IMU. FusionCore's coast mode inflates process noise to let chi2 accept the returning GPS fix after drift. The heading error accumulated in the 2012-06-15 blackout exceeds what the current bias-estimation pipeline fully corrects during that duration. Active area of work.

### 2012-08-20 (FC 98.3m, RL 10.6m)

The raw GPS stream for this sequence contains 105 mode-3 GPS fixes that are 720-840m off the RTK ground truth. These are excluded from `gps_rtk.csv` (the GT preprocessor rejects them) but are present in `gps.csv` as valid mode-3 fixes. They cluster in a 24-second window at the end of the second GPS blackout (t=66 min). The sequence has two GPS blackouts: 228s at t=42.2 min and 211s at t=62.5 min.

Per-minute trajectory analysis:

| Time window | FC error | Status |
|---|---|---|
| 0 - 42 min | 1 - 10 m | Nominal GPS coverage |
| 43 - 46 min | spike to ~100m, recovers in 2-3 min | First blackout (228s): GPS errors up to ~70m at boundary |
| 47 - 62 min | 3 - 10 m | Full recovery |
| 63 - 67 min | spike to ~788m, recovers in 2 min | Second blackout (211s): 105 fixes 720-840m off RTK at boundary |
| 68 - 82 min | 5 - 10 m | Full recovery for remaining 15 minutes |

The 98m ATE RMSE is driven almost entirely by those two transients. Outside those windows FC achieves 5-10m, on par with RL-EKF. The tradeoff in `coast_q_factor` that determines chi2 relaxation during blackout recovery is the active area of investigation: this GPS outlier clustering pattern (outliers at exactly the blackout boundary) is adversarial for any chi2-based gating scheme.

---

## Methodology

**Filters compared:**
- **FusionCore**: 23-state UKF, full 3D, adaptive noise, GPS chi2 gating, gyro + accel + encoder WZ bias estimation, inertial coast mode
- **RL-EKF**: robot_localization EKF with `two_d_mode: true`, GPS via `navsat_transform` with fixed RTK datum. Matching chi2 thresholds: `odom0_twist_rejection_threshold: 4.03` (chi2(3, 0.999) = sqrt(16.27)), `odom1_pose_rejection_threshold: 3.72` (chi2(2, 0.999) = sqrt(13.82))

**Sensor inputs (identical to both filters):**
- IMU: Microstrain 3DM-GX3-45, 100 Hz, raw specific force (gravity not removed by driver)
- Wheel odometry: Segway RMP encoders, 100 Hz, from `odometry_mu_100hz.csv`
- GPS: Novatel SPAN-CPT, 5 Hz, ~3m CEP in urban Michigan campus

**Ground truth:** RTK GPS (`gps_rtk.csv`), projected to local ENU. Only fixes with RTK mode >= 3 are used.

**Evaluation:** [evo](https://github.com/MichaelGrupp/evo) `evo_ape` with `--align` (SE3 alignment). ATE is computed after finding the best rigid-body transform between the filter trajectory and the RTK ground truth.

**Config:** Single YAML for all sequences: `fusioncore_datasets/config/nclt_fusioncore.yaml`. No per-sequence modifications.

---

## Reproduce

### Prerequisites

- NCLT data: download from http://robots.engin.umich.edu/nclt/
  Place each sequence under `benchmarks/nclt/<date>/raw files/`
- ROS 2 Jazzy sourced
- `evo` installed: `pip install evo --break-system-packages`
- FusionCore built: `colcon build --packages-select fusioncore_core fusioncore_ros fusioncore_datasets`

### Run one sequence (full length, auto-stops)

```bash
bash benchmarks/run_one.sh 2012-01-08
```

This script handles everything: kills leftover processes, runs the launch, waits for "Playback complete", extracts trajectories from the bag, generates ground truth TUM, evaluates with evo, and writes results to `benchmarks/nclt/2012-01-08/results_full/`. Takes 15-50 minutes depending on sequence length (running at 3x real time).

### Run all sequences sequentially

```bash
bash benchmarks/run_all.sh
```

Runs all sequences in chronological order. Plan for 4-6 hours total.

### Manual step-by-step (for understanding the pipeline)

```bash
SEQ=2012-01-08
DATA_DIR="benchmarks/nclt/$SEQ"
RESULTS_DIR="$DATA_DIR/results_full"

source /opt/ros/jazzy/setup.bash
source install/setup.bash

# 1. Kill leftover processes
pkill -9 -f "fusioncore_node|nclt_player|ekf_node|navsat_transform_node|ros2 bag record" 2>/dev/null; sleep 5

# 2. Generate navsat datum from first valid RTK fix
python3 tools/nclt_rtk_to_datum.py \
  --rtk "$DATA_DIR/raw files/gps_rtk.csv" \
  --out /tmp/navsat_datum_${SEQ}.yaml

# 3. Run benchmark launch (auto-stops on playback complete)
mkdir -p "$RESULTS_DIR"
ros2 launch fusioncore_datasets nclt_benchmark.launch.py \
  data_dir:="$DATA_DIR/raw files" \
  output_bag:="$DATA_DIR/bag_full" \
  playback_rate:=3.0 \
  duration_s:=0.0 \
  navsat_datum_yaml:=/tmp/navsat_datum_${SEQ}.yaml \
  2>&1 | tee "$RESULTS_DIR/launch.log"

# 4. Extract trajectories from the bag
python3 tools/odom_to_tum.py \
  --bag "$DATA_DIR/bag_full" --topic /fusion/odom \
  --out "$DATA_DIR/fusioncore.tum"

python3 tools/odom_to_tum.py \
  --bag "$DATA_DIR/bag_full" --topic /rl/odometry \
  --out "$DATA_DIR/rl_ekf.tum"

# 5. Generate ground truth
python3 tools/nclt_rtk_to_tum.py \
  --rtk "$DATA_DIR/raw files/gps_rtk.csv" \
  --out "$DATA_DIR/ground_truth.tum"

# 6. Sort (timestamps can be slightly out of order)
sort -n "$DATA_DIR/fusioncore.tum" > /tmp/fc.tum && mv /tmp/fc.tum "$DATA_DIR/fusioncore.tum"
sort -n "$DATA_DIR/rl_ekf.tum"    > /tmp/rl.tum && mv /tmp/rl.tum "$DATA_DIR/rl_ekf.tum"

# 7. Evaluate
python3 tools/evaluate.py \
  --gt         "$DATA_DIR/ground_truth.tum" \
  --fusioncore "$DATA_DIR/fusioncore.tum" \
  --rl         "$DATA_DIR/rl_ekf.tum" \
  --sequence   "$SEQ" \
  --out_dir    "$RESULTS_DIR"
```

---

## Directory structure

```
benchmarks/
  README.md               ← this file
  run_one.sh              ← run one sequence end-to-end
  run_all.sh              ← run all sequences sequentially
  nclt/
    2012-01-08/
      raw files/          ← NCLT CSV data (not committed, download separately)
        ms25.csv          ← IMU (100 Hz)
        ms25_euler.csv    ← IMU Euler angles
        odometry_mu_100hz.csv  ← wheel encoder (100 Hz)
        gps.csv           ← GPS fixes (5 Hz, raw including outliers)
        gps_rtk.csv       ← RTK GPS (5 Hz, clean ground truth)
        kvh.csv           ← heading (not used)
        wheels.csv        ← raw wheel counts (not used)
      bag_full/           ← ROS 2 bag from the run (not committed, large)
      fusioncore.tum      ← FC trajectory (not committed, regenerate)
      rl_ekf.tum          ← RL-EKF trajectory (not committed, regenerate)
      ground_truth.tum    ← RTK ground truth (not committed, regenerate)
      results_full/
        BENCHMARK.md      ← metrics table (committed)
        trajectories.png  ← trajectory overlay plot (committed)
        ate_over_time.png ← ATE vs time plot (committed)
        error_distribution.png ← error histogram (committed)
        launch.log        ← full launch output (committed)
```

`.tum` files, `.mcap` bags, and `bag_full/` directories are in `.gitignore`. Only `results_full/` contents are committed.
