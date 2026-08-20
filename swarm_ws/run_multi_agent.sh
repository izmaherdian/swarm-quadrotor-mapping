#!/usr/bin/env bash
set -e

WS_DIR="$(cd "$(dirname "$0")" && pwd)"
source /opt/ros/lyrical/setup.bash
source "$WS_DIR/../.venv/bin/activate"

export AMENT_PREFIX_PATH="$WS_DIR/install/swarm_msgs:$WS_DIR/install/swarm_sim:$WS_DIR/install/swarm_high_level:$WS_DIR/install/swarm_low_level:$WS_DIR/install/swarm_mid_level:$AMENT_PREFIX_PATH"
export PYTHONPATH="$WS_DIR/install/swarm_msgs/local/lib/python3.14/dist-packages:$WS_DIR/install/swarm_sim/lib/python3.14/site-packages:$WS_DIR/install/swarm_high_level/lib/python3.14/site-packages:$WS_DIR/install/swarm_low_level/lib/python3.14/site-packages:$WS_DIR/install/swarm_mid_level/lib/python3.14/site-packages:$PYTHONPATH"
export LD_LIBRARY_PATH="$WS_DIR/install/swarm_sim/lib:$LD_LIBRARY_PATH"
export GZ_SIM_RESOURCE_PATH="$WS_DIR/src/swarm_sim/models"

echo "=== Kill leftover gz/ros2 ==="
for pid in $(ps aux | grep -E "gz.sim|ros2 launch|ros_gz|spawn|controller|ai_iris|bridge|tf_prefix|static_transform" | grep -v grep | awk '{print $2}'); do
    kill -9 "$pid" 2>/dev/null || true
done
sleep 2
rm -f /tmp/sim_multi.log

echo "=== Build swarm_mid_level, swarm_low_level & swarm_sim ==="
colcon build --packages-select swarm_mid_level swarm_low_level 2>&1 | tail -3
colcon build --packages-select swarm_sim 2>&1 | tail -3
for node in collision_avoidance_node pid_lqr_node tf_prefix_node; do
    chmod +x "$WS_DIR/install/swarm_mid_level/lib/swarm_mid_level/$node" 2>/dev/null || true
    chmod +x "$WS_DIR/install/swarm_low_level/lib/swarm_low_level/$node" 2>/dev/null || true
    sed -i '1s|^#!".*|#!/usr/bin/env python3|' "$WS_DIR/install/swarm_mid_level/lib/swarm_mid_level/$node" 2>/dev/null || true
    sed -i '1s|^#!".*|#!/usr/bin/env python3|' "$WS_DIR/install/swarm_low_level/lib/swarm_low_level/$node" 2>/dev/null || true
done

echo "=== Clean old CSVs ==="
rm -f "$WS_DIR/src/swarm_sim/results/multi_agent/pid_lqr/"*.csv
rm -f "$WS_DIR/src/swarm_sim/results/multi_agent/pid_hinf/"*.csv

cleanup() {
    trap - EXIT INT TERM
    echo ""
    echo "=== Cleanup ==="
    if [ -n "$SIM_PID" ]; then
        kill "$SIM_PID" 2>/dev/null || true
    fi
    for pid in $(ps aux | grep -E "gz.sim|ros2 launch|parameter_bridge|ai_iris|collision|pid_" | grep -v grep | awk '{print $2}'); do
        kill -9 "$pid" 2>/dev/null || true
    done
    echo ""
    echo "=== CSV hasil 7 Drone tersimpan di: ==="
    echo "  $WS_DIR/src/swarm_sim/results/multi_agent/pid_lqr/"
    echo ""
    ls -lh "$WS_DIR/src/swarm_sim/results/multi_agent/pid_lqr/"*.csv 2>/dev/null || echo "  (tidak ada CSV)"
    echo ""
    echo "Done (Multi-Agent Swarm Selesai)."
}
trap cleanup EXIT INT TERM

echo "=== Launch Gazebo sim (background) ==="
nohup ros2 launch swarm_sim sim_launch.py \
    num_drones:=7 controller:=pid_lqr_node \
    headless:=false rviz:=false \
    results_base:=multi_agent "$@" > /tmp/sim_multi.log 2>&1 &
SIM_PID=$!
echo "  sim PID=$SIM_PID"

echo "=== Wait for ORCA initialized ==="
for i in $(seq 1 60); do
    if grep -q "initialized" /tmp/sim_multi.log 2>/dev/null; then
        echo "  ORCA ready after ${i}s"
        break
    fi
    sleep 1
done
if ! grep -q "initialized" /tmp/sim_multi.log 2>/dev/null; then
    echo "  ERROR: ORCA tidak pernah initialized. Cek /tmp/sim_multi.log"
    exit 1
fi

echo "=== Run test_waypoints.py (Monitoring 7 Drone Swarm secara Real-Time) ==="
python3 "$WS_DIR/test_waypoints.py"