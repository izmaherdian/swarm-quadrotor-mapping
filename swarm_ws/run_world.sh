#!/usr/bin/env bash
set -e

WS_DIR="$(cd "$(dirname "$0")" && pwd)"
source /opt/ros/lyrical/setup.bash
if [ -f "$WS_DIR/../.venv/bin/activate" ]; then
    source "$WS_DIR/../.venv/bin/activate"
fi

export AMENT_PREFIX_PATH="$WS_DIR/install/swarm_msgs:$WS_DIR/install/swarm_sim:$WS_DIR/install/swarm_high_level:$WS_DIR/install/swarm_low_level:$WS_DIR/install/swarm_mid_level:$AMENT_PREFIX_PATH"
export PYTHONPATH="$WS_DIR/install/swarm_msgs/local/lib/python3.14/dist-packages:$WS_DIR/install/swarm_sim/lib/python3.14/site-packages:$WS_DIR/install/swarm_high_level/lib/python3.14/site-packages:$WS_DIR/install/swarm_low_level/lib/python3.14/site-packages:$WS_DIR/install/swarm_mid_level/lib/python3.14/site-packages:$PYTHONPATH"
export GZ_SIM_SYSTEM_PLUGIN_PATH="/opt/ros/lyrical/opt/gz_sim_vendor/lib/gz-sim-10/plugins:$GZ_SIM_SYSTEM_PLUGIN_PATH"
export LD_LIBRARY_PATH="/opt/ros/lyrical/opt/gz_sim_vendor/lib:/opt/ros/lyrical/opt/gz_sim_vendor/lib/gz-sim-10/plugins:$WS_DIR/install/swarm_sim/lib:$LD_LIBRARY_PATH"
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
    if [ -n "$GZ_PID" ]; then
        kill "$GZ_PID" 2>/dev/null || true
    fi
    if [ -n "$RVIZ_PID" ]; then
        kill "$RVIZ_PID" 2>/dev/null || true
    fi
    for pid in $(ps aux | grep -E "gz sim|gz-sim|rviz2|parameter_bridge.*clock|static_tf_world_publisher" | grep -v grep | awk '{print $2}'); do
        kill -9 "$pid" 2>/dev/null || true
    done
    echo "✅ Gazebo Sim dan RViz2 telah ditutup bersih."
    echo "Done (Terminal 1 Ditutup)."
}
trap cleanup EXIT INT TERM

echo ""
echo "========================================================================="
echo "  🌍 [TERMINAL 1] MEMULAI GAZEBO SIMULATION WORLD & RVIZ2 (PERSISTENT)"
echo "  Dunia: empty.world (Arena 12x12m) | RViz2: $RVIZ_FLAG | Headless: $HEADLESS_FLAG"
echo "========================================================================="
echo ""

# 1. Global Clock Bridge
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock \
    --ros-args -r __node:=global_clock_bridge >/dev/null 2>&1 &
BRIDGE_PID=$!

# 2. Static Transform Publisher
ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 --yaw 0 --pitch 0 --roll 0 \
    --frame-id world --child-frame-id swarm_world \
    --ros-args -r __node:=static_tf_world_publisher >/dev/null 2>&1 &
TF_PID=$!

# 3. Gazebo Simulation World
WORLD_FILE="$WS_DIR/install/swarm_sim/share/swarm_sim/worlds/empty.world"
if [ "$HEADLESS_FLAG" = "true" ]; then
    gz sim -r -s "$WORLD_FILE" &
    GZ_PID=$!
else
    gz sim -r "$WORLD_FILE" &
    GZ_PID=$!
fi

# 4. RViz2 GUI
if [ "$RVIZ_FLAG" = "true" ]; then
    RVIZ_CONFIG="$WS_DIR/install/swarm_sim/share/swarm_sim/rviz/swarm.rviz"
    rviz2 -d "$RVIZ_CONFIG" &
    RVIZ_PID=$!
fi

echo "Menunggu Gazebo Sim dan RViz2 siap dimuat..."
for i in $(seq 1 30); do
    if gz topic -l 2>/dev/null | grep -q "/world/swarm_world"; then
        echo "✅ Gazebo Sim World (/world/swarm_world) aktif & siap setelah ${i}s!"
        break
    fi
    sleep 1
done

if [ "$RVIZ_FLAG" = "true" ]; then
    echo "✅ Jendela RViz2 aktif!"
fi

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

# Tunggu sampai pengguna menekan Ctrl+C
if [ -n "$GZ_PID" ]; then
    wait "$GZ_PID"
else
    wait
fi
