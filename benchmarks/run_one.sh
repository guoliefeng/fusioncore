#!/bin/bash
# Run a single NCLT benchmark sequence (full length).
# Usage: bash benchmarks/run_one.sh 2012-01-08
#
# Stops automatically when nclt_player prints "Playback complete."
# No Ctrl+C needed.
#
# Prerequisites: see benchmarks/check_prereqs.sh

set -euo pipefail

SEQ=${1:-2012-01-08}

# Derive repo root from script location — works from any directory.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$SCRIPT_DIR/.." && pwd)"

# Add pip user bin so evo is found regardless of how it was installed.
export PATH="$HOME/.local/bin:$PATH"

# Source ROS if not already active.
if [ -z "${ROS_DISTRO:-}" ]; then
    ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
    if [ -f "$ROS_SETUP" ]; then
        # shellcheck disable=SC1090
        source "$ROS_SETUP"
    else
        echo "ERROR: ROS is not sourced and $ROS_SETUP was not found."
        echo "  Fix: source /opt/ros/<distro>/setup.bash   or set ROS_SETUP=/path/to/setup.bash"
        exit 1
    fi
fi

# Source the FusionCore workspace if fusioncore_datasets is not already available.
if ! ros2 pkg list 2>/dev/null | grep -q "^fusioncore_datasets$"; then
    INSTALL_SETUP="$REPO/install/setup.bash"
    if [ -f "$INSTALL_SETUP" ]; then
        # shellcheck disable=SC1090
        # colcon's setup.bash references COLCON_TRACE without a default; guard it
        set +u
        source "$INSTALL_SETUP"
        set -u
    else
        echo "ERROR: fusioncore_datasets package not found."
        echo "  Fix: cd $REPO && colcon build --packages-select fusioncore_core fusioncore_ros fusioncore_datasets"
        exit 1
    fi
fi
RATE=1.0
WALL_TIME=9000   # hard cap: 150 min covers longest sequence at 1x

DATA_DIR="$REPO/benchmarks/nclt/$SEQ"
BAG_DIR="$DATA_DIR/bag_full"
RESULTS_DIR="$DATA_DIR/results_full"
DATUM_YAML="/tmp/navsat_datum_${SEQ}.yaml"

echo "========================================"
echo "SEQ: $SEQ  ($(date))"
echo "========================================"

mkdir -p "$RESULTS_DIR"

# Generate fixed navsat datum from first valid RTK fix.
python3 << PYEOF
import csv, math, sys
rtk_path = "$DATA_DIR/raw files/gps_rtk.csv"
datum_yaml = "$DATUM_YAML"
lat_deg = lon_deg = None
with open(rtk_path) as f:
    for row in csv.reader(f):
        try:
            if int(row[1]) < 3:
                continue
            lr, lonr = float(row[3]), float(row[4])
        except (ValueError, IndexError):
            continue
        if not (math.isnan(lr) or math.isnan(lonr)):
            lat_deg, lon_deg = math.degrees(lr), math.degrees(lonr)
            break
if lat_deg is None:
    sys.exit("ERROR: no valid RTK fix in " + rtk_path)
with open(datum_yaml, 'w') as f:
    f.write(f"navsat_transform:\n  ros__parameters:\n    datum: [{lat_deg:.8f}, {lon_deg:.8f}, 0.0]\n    wait_for_datum: true\n")
print(f"navsat datum: {lat_deg:.6f}N {lon_deg:.6f}E -> {datum_yaml}")
PYEOF

pkill -9 -f "fusioncore_node|nclt_player|ekf_node|navsat_transform_node|ros2 bag record|ros2 launch|component_container" 2>/dev/null || true
sleep 8

rm -rf "$BAG_DIR"
rm -f "$DATA_DIR/fusioncore.tum" "$DATA_DIR/rl_ekf.tum" "$DATA_DIR/ground_truth.tum"
> "$RESULTS_DIR/launch.log"

echo "Launching full-length benchmark (auto-stops on playback complete)..."

# Run launch in background, log to file and show on screen simultaneously.
ros2 launch fusioncore_datasets nclt_benchmark.launch.py \
    data_dir:="$DATA_DIR/raw files" \
    output_bag:="$BAG_DIR" \
    playback_rate:=$RATE \
    duration_s:=0.0 \
    navsat_datum_yaml:="$DATUM_YAML" \
    2>&1 >> "$RESULTS_DIR/launch.log" &
LAUNCH_PID=$!

# Show live output while monitoring for completion.
tail -f "$RESULTS_DIR/launch.log" &
TAIL_PID=$!

ELAPSED=0
while [ $ELAPSED -lt $WALL_TIME ]; do
    if ! kill -0 $LAUNCH_PID 2>/dev/null; then
        echo ""
        echo "Launch exited on its own."
        break
    fi
    if grep -q "Playback complete" "$RESULTS_DIR/launch.log" 2>/dev/null; then
        echo ""
        echo "Playback complete detected. Waiting 8s for bag to flush..."
        sleep 8
        kill -SIGINT $LAUNCH_PID 2>/dev/null || true
        sleep 5
        break
    fi
    sleep 5
    ELAPSED=$((ELAPSED + 5))
done

if [ $ELAPSED -ge $WALL_TIME ]; then
    echo "Wall-time cap hit ($WALL_TIME s). Stopping..."
    kill -SIGINT $LAUNCH_PID 2>/dev/null || true
    sleep 5
fi

kill $TAIL_PID 2>/dev/null || true
# Don't wait indefinitely — give it 10s then force-kill.
for i in $(seq 1 10); do
    kill -0 $LAUNCH_PID 2>/dev/null || break
    sleep 1
done
kill -9 $LAUNCH_PID 2>/dev/null || true
pkill -9 -f "fusioncore_node|nclt_player|ekf_node|navsat_transform_node|ros2 bag record|component_container" 2>/dev/null || true
sleep 5

if [ ! -d "$BAG_DIR" ]; then
    echo "ERROR: bag not created for $SEQ"
    exit 1
fi

echo "Bag ready. Generating ground truth and evaluating..."

python3 "$REPO/tools/nclt_rtk_to_tum.py" \
    --rtk "$DATA_DIR/raw files/gps_rtk.csv" \
    --out "$DATA_DIR/ground_truth.tum"

python3 "$REPO/tools/odom_to_tum.py" \
    --bag "$BAG_DIR" --topic /fusion/odom \
    --out "$DATA_DIR/fusioncore.tum"

python3 "$REPO/tools/odom_to_tum.py" \
    --bag "$BAG_DIR" --topic /rl/odometry \
    --out "$DATA_DIR/rl_ekf.tum"

sort -n "$DATA_DIR/fusioncore.tum" > /tmp/fc_s.tum && cp /tmp/fc_s.tum "$DATA_DIR/fusioncore.tum"
sort -n "$DATA_DIR/rl_ekf.tum"    > /tmp/rl_s.tum && cp /tmp/rl_s.tum "$DATA_DIR/rl_ekf.tum"

python3 "$REPO/tools/evaluate.py" \
    --gt         "$DATA_DIR/ground_truth.tum" \
    --fusioncore "$DATA_DIR/fusioncore.tum" \
    --rl         "$DATA_DIR/rl_ekf.tum" \
    --sequence   "$SEQ" \
    --out_dir    "$RESULTS_DIR"

echo "DONE: $SEQ  ($(date))"
