#!/usr/bin/env bash
set -e

WS_DIR="$(cd "$(dirname "$0")" && pwd)"
source /opt/ros/lyrical/setup.bash
if [ -f "$WS_DIR/../.venv/bin/activate" ]; then
    source "$WS_DIR/../.venv/bin/activate"
fi

export AMENT_PREFIX_PATH="$WS_DIR/install/swarm_msgs:$WS_DIR/install/swarm_sim:$WS_DIR/install/swarm_high_level:$WS_DIR/install/swarm_low_level:$WS_DIR/install/swarm_mid_level:$AMENT_PREFIX_PATH"
export PYTHONPATH="$WS_DIR/install/swarm_msgs/local/lib/python3.14/dist-packages:$WS_DIR/install/swarm_sim/lib/python3.14/site-packages:$WS_DIR/install/swarm_high_level/lib/python3.14/site-packages:$WS_DIR/install/swarm_low_level/lib/python3.14/site-packages:$WS_DIR/install/swarm_mid_level/lib/python3.14/site-packages:$PYTHONPATH"

echo "=== [TERMINAL 3] Memeriksa koneksi drone (Terminal 2) ==="
if ! ros2 topic list 2>/dev/null | grep -q "/iris_1/odometry"; then
    echo ""
    echo "⚠️  PERINGATAN: Drone (/iris_1/odometry) belum terdeteksi!"
    echo "   Pastikan TERMINAL 1 (./run_world.sh) dan TERMINAL 2 (./run_drones.sh) sudah berjalan."
    echo ""
    exit 1
fi

echo ""
echo "========================================================================="
echo "  🗺️  [TERMINAL 3] MEMULAI MISI PEMETAAN BOUSTROPHEDON ($*)"
echo "  Area: [-6, 6] x [-6, 6] m | 10 Baris | Coverage Footprint di RViz2"
echo "========================================================================="
echo ""

python3 "$WS_DIR/test_mapping.py" "$@"
