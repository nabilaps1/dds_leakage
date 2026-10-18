#!/usr/bin/env bash
# Runs N repetitions of the experiment with several robots at the same time.
# The simulation (sim/sim_multi.launch.py) must already be running.
# Usage: run_experiment.sh <condition> <n_runs> [first_run_number]
#   e.g.: run_experiment.sh shapes 5     -> data/shapes/run_01 .. run_05
#         run_experiment.sh shapes 3 6   -> data/shapes/run_06 .. run_08

CONDITION=${1:?give the condition (e.g. shapes)}
N=${2:?give the number of runs}
FIRST=${3:-1}
PKG_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
DATA_DIR=$PKG_DIR/data/$CONDITION
CONTROLLER_SRC=$PKG_DIR/dds_leakage/controller.py

# One robot per line: "name path laps". The name is the namespace and the model name in Gazebo.
# Each robot drives a different shape; the names do not reveal which (only the commands do).
ROBOTS=(
    "tb3_1 square 1"
    "tb3_2 triangle 1"
    "tb3_3 circle 1"
    "tb3_4 up_down 1"
    "tb3_5 hexagon 1"
)

source "$PKG_DIR/env.sh"

PIDS=()

# Stops background processes: SIGINT to the process group (ros2 run + Python node)
stop_groups() {
    for pid in "${PIDS[@]}"; do
        kill -INT -- "-$pid" 2>/dev/null
        wait "$pid" 2>/dev/null
    done
    PIDS=()
}
trap 'echo; echo "Interrupted: delete the incomplete run in $OUT"; stop_groups; exit 130' INT
trap stop_groups EXIT

NAMES=()
for entry in "${ROBOTS[@]}"; do
    read -r name _ <<< "$entry"
    NAMES+=("$name")
done
MODEL_NAMES="[$(IFS=,; echo "${NAMES[*]}")]"

TOPICS=$(ros2 topic list)
for topic in /gazebo/model_states "${NAMES[@]/#//}"; do
    [ "$topic" = /gazebo/model_states ] || topic="$topic/cmd_vel"
    if ! grep -qx "$topic" <<< "$TOPICS"; then
        echo "ERROR: $topic not found. Start the simulation first:"
        echo "  ros2 launch $PKG_DIR/sim/sim_multi.launch.py"
        exit 1
    fi
done

for ((i = FIRST; i < FIRST + N; i++)); do
    RUN=$(printf 'run_%02d' "$i")
    OUT=$DATA_DIR/$RUN
    if [ -e "$OUT" ]; then
        echo "ERROR: $OUT already exists. Choose another first run number or delete the folder."
        exit 1
    fi
    for name in "${NAMES[@]}"; do mkdir -p "$OUT/$name"; done
    echo "=== $CONDITION/$RUN ==="

    # 1. Robots back to their initial poses (without resetting the clock)
    ros2 service call /reset_world std_srvs/srv/Empty > /dev/null
    sleep 1

    # 2. Recorders in the background, each in its own process group (set -m)
    set -m
    ros2 run dds_leakage ground_truth_recorder --ros-args \
        -p output_dir:="$OUT" -p model_names:="$MODEL_NAMES" \
        > "$OUT/ground_truth_recorder.log" 2>&1 &
    PIDS+=($!)
    ros2 run dds_leakage observer --ros-args -p output_dir:="$OUT" \
        > "$OUT/observer.log" 2>&1 &
    PIDS+=($!)
    set +m
    sleep 3  # DDS discovery

    # 3. Controllers: the first robot in the foreground (log on screen), the others in the background
    CONTROLLER_PIDS=()
    set -m
    for entry in "${ROBOTS[@]:1}"; do
        read -r name path laps <<< "$entry"
        ros2 run dds_leakage controller --ros-args -r __ns:=/"$name" \
            -p path:="$path" -p laps:="$laps" > "$OUT/$name/controller.log" 2>&1 &
        PIDS+=($!)
        CONTROLLER_PIDS+=($!)
    done
    set +m
    read -r name path laps <<< "${ROBOTS[0]}"
    ros2 run dds_leakage controller --ros-args -r __ns:=/"$name" \
        -p path:="$path" -p laps:="$laps" 2>&1 | tee "$OUT/$name/controller.log"
    for pid in "${CONTROLLER_PIDS[@]}"; do wait "$pid"; done
    sleep 2

    # 4. Stop the recorders
    stop_groups

    # 5. Run metadata and per-robot check
    {
        echo "condition: $CONDITION"
        echo "run: $RUN"
        echo "date: $(date -Iseconds)"
        echo "ros_domain_id: $ROS_DOMAIN_ID"
        echo "controller_sha256: $(sha256sum "$CONTROLLER_SRC" | cut -d' ' -f1)"
    } > "$OUT/notes.txt"
    WARNINGS=()
    for entry in "${ROBOTS[@]}"; do
        read -r name path laps <<< "$entry"
        LOG=$OUT/$name/controller.log
        OBS=$OUT/$name/observed_data.csv
        GT=$OUT/$name/ground_truth.csv
        SCAN=$OUT/$name/observed_scan.csv
        PLANNED=$(grep -oP 'Plan with \K[0-9]+' "$LOG")
        PLANNED_MOVING=$(grep -oP '[0-9]+(?= moving)' "$LOG")
        OBSERVED=0; OBSERVED_MOVING=0
        if [ -f "$OBS" ]; then
            OBSERVED=$(($(wc -l < "$OBS") - 1))
            OBSERVED_MOVING=$(awk -F, 'NR > 1 && ($2 != 0 || $3 != 0)' "$OBS" | wc -l)
        fi
        {
            echo "$name:"
            echo "  path: $path x$laps"
            echo "  planned_commands: $PLANNED ($PLANNED_MOVING moving)"
            echo "  observed_rows: $OBSERVED ($OBSERVED_MOVING moving)"
            echo "  ground_truth_rows: $(($(wc -l < "$GT") - 1))"
            if [ -s "$SCAN" ]; then
                echo "  scan_rows: $(($(wc -l < "$SCAN") - 1))"
            else
                echo "  scan_rows: 0"
            fi
        } >> "$OUT/notes.txt"
        if [ "$OBSERVED_MOVING" != "$PLANNED_MOVING" ]; then
            WARNINGS+=("WARNING: $RUN/$name recorded $OBSERVED_MOVING moving commands (planned: $PLANNED_MOVING)")
        fi
    done
    cat "$OUT/notes.txt"
    printf '%s\n' "${WARNINGS[@]}"
done

echo "Done: $N run(s) in $DATA_DIR"
