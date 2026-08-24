#!/usr/bin/env bash
set -e

WS_DIR="$(cd "$(dirname "$0")" && pwd)"
source /opt/ros/lyrical/setup.bash
if [ -f "$WS_DIR/../.venv/bin/activate" ]; then
    source "$WS_DIR/../.venv/bin/activate"
fi

export AMENT_PREFIX_PATH="$WS_DIR/install/swarm_msgs:$WS_DIR/install/swarm_sim:$WS_DIR/install/swarm_high_level:$WS_DIR/install/swarm_low_level:$WS_DIR/install/swarm_mid_level:$AMENT_PREFIX_PATH"
export PYTHONPATH="$WS_DIR/install/swarm_msgs/local/lib/python3.14/dist-packages:$WS_DIR/install/swarm_sim/lib/python3.14/site-packages:$WS_DIR/install/swarm_high_level/lib/python3.14/site-packages:$WS_DIR/install/swarm_low_level/lib/python3.14/site-packages:$WS_DIR/install/swarm_mid_level/lib/python3.14/site-packages:$PYTHONPATH"
export LD_LIBRARY_PATH="$WS_DIR/install/swarm_sim/lib:$LD_LIBRARY_PATH"
export GZ_SIM_RESOURCE_PATH="$WS_DIR/src/swarm_sim/models"

# Opsi RViz dan Headless
RVIZ_FLAG=true
HEADLESS_FLAG=false
for arg in "$@"; do
    case $arg in
        --no-rviz)
            RVIZ_FLAG=false
            ;;
        --headless)
            HEADLESS_FLAG=true
            RVIZ_FLAG=false
            ;;
    esac
done

echo "=== [TERMINAL 1] Bersihkan sisa proses lama ==="
for pid in $(ps aux | grep -E "gz.sim|ros2 launch.*world_launch|rviz2|global_clock_bridge|static_tf_world_publisher" | grep -v grep | awk '{print $2}'); do
    kill -9 "$pid" 2>/dev/null || true
done
sleep 1

echo "=== [TERMINAL 1] Build paket swarm_sim ==="
colcon build --packages-select swarm_sim 2>&1 | tail -3
source "$WS_DIR/install/setup.bash"

cleanup() {
    trap - EXIT INT TERM
    echo ""
    echo "=== [TERMINAL 1] Menutup Simulasi Gazebo & RViz2 ==="
    if [ -n "$WORLD_PID" ]; then
        kill "$WORLD_PID" 2>/dev/null || true
    fi
    for pid in $(ps aux | grep -E "gz.sim|ros2 launch.*world_launch|rviz2|global_clock_bridge" | grep -v grep | awk '{print $2}'); do
        kill -9 "$pid" 2>/dev/null || true
    done
    echo "Done (Terminal 1 Ditutup)."
}
trap cleanup EXIT INT TERM

echo ""
echo "========================================================================="
echo "  🌍 [TERMINAL 1] MEMULAI GAZEBO SIMULATION WORLD & RVIZ2 (PERSISTENT)"
echo "  Dunia: empty.world (Arena 12x12m) | RViz2: $RVIZ_FLAG | Headless: $HEADLESS_FLAG"
echo "========================================================================="
echo ""

ros2 launch swarm_sim world_launch.py \
    headless:=$HEADLESS_FLAG rviz:=$RVIZ_FLAG &
WORLD_PID=$!

echo "Menunggu Gazebo Sim siap..."
for i in $(seq 1 30); do
    if gz topic -l 2>/dev/null | grep -q "/world/swarm_world"; then
        echo "✅ Gazebo Sim World (/world/swarm_world) aktif & siap setelah ${i}s!"
        break
    fi
    sleep 1
done

echo ""
echo "========================================================================="
echo "  ✅ TERMINAL 1 BERJALAN & SIAP DIGUNAKAN!"
echo "  Biarkan terminal ini tetap berjalan (TIDAK PERLU DI-RESTART)."
echo ""
echo "  👉 Selanjutnya buka TERMINAL 2 dan jalankan:"
echo "     cd $WS_DIR"
echo "     ./run_drones.sh"
echo "========================================================================="
echo ""

wait "$WORLD_PID"
