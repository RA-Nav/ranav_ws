#!/usr/bin/env bash
# Usage: ./rosbag.sh [extra topics or ros2 bag record options]
# The recording starts immediately; node parameters are dumped in parallel (5 s timeout each).
# Ctrl+C stops the recording cleanly.

TOPICS=(
    # robot description and transforms
    /robot_description
    /tf
    /tf_static
    # joints and IMU
    /joint_states
    /imu_sensor_broadcaster/imu
    # clean lidar map (after SC-PGO) and corrected odometry
    /aft_pgo_map
    /aft_pgo_odom
    /aft_pgo_path
    # OptiPessi goal and obstacles (bridges of ranav_mission)
    /opti_pessi/goal
    /opti_pessi/obstacles
    # Acados MPC: predicted CoM/feet/footholds, obstacles seen by the MPC, solver monitor and statistics
    /mpc_predictions
    /mpc_obstacles
    /monitor
    /mpc_stats
    /odom
    # RealSense camera
    /camera/camera/color/image_raw
    /camera/camera/color/camera_info
)

set -m  # job control: the recorder gets its own process group and is stopped only by our trap

# 1. Name the session directory with a timestamp
SESSION_DIR="rosbag_session_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$SESSION_DIR"

echo "Saving environment variables..."
printenv | grep -E '^(ROS_|RMW_|FASTRTPS_|CYCLONEDDS_)' > "$SESSION_DIR/ros_env.txt" || true

# 2. Start recording first (/parameter_events also captures parameter changes during the recording)
echo "Starting ros2 bag recording (Press Ctrl+C to stop)..."
# --disable-keyboard-controls and </dev/null: in the background the recorder would be stopped by SIGTTOU/SIGTTIN
# when it touches the terminal, and Ctrl+C would hang
ros2 bag record --disable-keyboard-controls -o "$SESSION_DIR/bag_data" \
    --topics /parameter_events "${TOPICS[@]}" "$@" </dev/null &
BAG_PID=$!

stop_bag() {
    trap '' INT TERM
    echo
    echo "Stopping recording..."
    kill -INT "$BAG_PID" 2>/dev/null
    wait "$BAG_PID" 2>/dev/null
    # Drop an unfinished parameter dump (own process group, see set -m) instead of waiting for it
    [ -n "$DUMP_PID" ] && kill -TERM -- "-$DUMP_PID" 2>/dev/null
    echo "Done: $SESSION_DIR"
    exit 0
}
trap stop_bag INT TERM

# 3. Dump node parameters in parallel, in the background of the recording
echo "Dumping active node parameters (in parallel)..."
mkdir -p "$SESSION_DIR/node_params"
(
    for node in $(timeout -s KILL 10 ros2 node list 2>/dev/null | grep -v '^/rosbag2_recorder'); do
        # Convert '/namespace/node_name' to 'namespace_node_name.yaml'
        clean_name=$(echo "$node" | sed 's/^\///; s/\//_/g')
        (
            if timeout -s KILL 5 ros2 param dump "$node" > "$SESSION_DIR/node_params/${clean_name}.yaml.tmp" 2>/dev/null; then
                mv "$SESSION_DIR/node_params/${clean_name}.yaml.tmp" "$SESSION_DIR/node_params/${clean_name}.yaml" 2>/dev/null
            else
                rm -f "$SESSION_DIR/node_params/${clean_name}.yaml.tmp"
                echo "param dump failed/timeout: $node" >> "$SESSION_DIR/node_params/_failed.txt"
            fi
        ) &
    done
    wait
    echo "Parameter dump finished."
) &
DUMP_PID=$!

# 4. Wait until the recorder ends (Ctrl+C goes through the trap)
wait "$BAG_PID"
