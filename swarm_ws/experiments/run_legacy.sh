#!/bin/bash
set -e

echo "Memulai proses Build & Run Swarm Drone..."

cd "$(dirname "$0")"
WS_DIR="$(cd "$(dirname "$0")/.." && pwd)"

pkill -9 -f "install/swarm_mid_level|install/swarm_low_level|install/swarm_sim|ros_gz_sim|parameter_bridge" 2>/dev/null || true

unset AMENT_PREFIX_PATH
unset PYTHONPATH
unset LD_LIBRARY_PATH
source /opt/ros/lyrical/setup.bash
source "$WS_DIR/../.venv/bin/activate"

echo "Membangun workspace..."
colcon build --event-handlers console_cohesion+

source "$WS_DIR/install/setup.bash"

ARGS="$@"
if [ -z "$ARGS" ]; then
    ARGS="num_drones:=7 headless:=false rviz:=true controller:=pid_lqr_node"
fi

echo ""
echo "Menjalankan simulasi dengan argumen: $ARGS"
echo "--------------------------------------------------------"
ros2 launch swarm_sim sim_launch.py $ARGS
