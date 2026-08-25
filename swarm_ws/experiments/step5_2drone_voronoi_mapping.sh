#!/usr/bin/env bash
# ==============================================================================
#   STEP 5: 2-DRONE VORONOI BOUSTROPHEDON MAPPING (ALL-IN-ONE EXPERIMENT)
# ==============================================================================
#  Alur Eksekusi:
#    1. Generate koordinat spawn acak untuk 2 drone (iris_1 & iris_2)
#    2. Jalankan Gazebo Sim World & RViz2
#    3. Spawn 2 Drone dengan Auto-Takeoff ke Z = 2.0m (PID-LQR / PID-Hinf)
#    4. Jalankan Node Mid-Level ORCA 2D & Koordinator Pemetaan Voronoi
#    5. Monitor visualisasi jejak cakupan gabungan (100% Coverage)
# ==============================================================================

set -e

WS_DIR="$(cd "$(dirname "$0")/.." && pwd)"
source /opt/ros/lyrical/setup.bash
if [ -f "$WS_DIR/../.venv/bin/activate" ]; then
    source "$WS_DIR/../.venv/bin/activate"
fi

export AMENT_PREFIX_PATH="$WS_DIR/install/swarm_msgs:$WS_DIR/install/swarm_sim:$WS_DIR/install/swarm_high_level:$WS_DIR/install/swarm_low_level:$WS_DIR/install/swarm_mid_level:$AMENT_PREFIX_PATH"
export PYTHONPATH="$WS_DIR/install/swarm_msgs/local/lib/python3.14/dist-packages:$WS_DIR/install/swarm_sim/lib/python3.14/site-packages:$WS_DIR/install/swarm_high_level/lib/python3.14/site-packages:$WS_DIR/install/swarm_low_level/lib/python3.14/site-packages:$WS_DIR/install/swarm_mid_level/lib/python3.14/site-packages:$PYTHONPATH"
export GZ_SIM_SYSTEM_PLUGIN_PATH="/opt/ros/lyrical/opt/gz_sim_vendor/lib/gz-sim-10/plugins:$GZ_SIM_SYSTEM_PLUGIN_PATH"
export LD_LIBRARY_PATH="/opt/ros/lyrical/opt/gz_sim_vendor/lib:/opt/ros/lyrical/opt/gz_sim_vendor/lib/gz-sim-10/plugins:$WS_DIR/install/swarm_sim/lib:$LD_LIBRARY_PATH"
export GZ_SIM_RESOURCE_PATH="$WS_DIR/src/swarm_sim/models"

# ── Parameter Default ────────────────────────────────────────────────────────
CONTROLLER="pid_lqr_node"
CONTROLLER_NAME="PID-LQR"
RANDOM_SPAWN=false
HEADLESS=false
RVIZ=true

SPAWN_X1="-4.0"
SPAWN_Y1="-3.0"
SPAWN_X2="4.0"
SPAWN_Y2="3.0"

# ── Parsing Argumen ──────────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case "$1" in
        --lqr)
            CONTROLLER="pid_lqr_node"
            CONTROLLER_NAME="PID-LQR"
            shift
            ;;
        --hinf)
            CONTROLLER="pid_hinf_node"
            CONTROLLER_NAME="PID-Hinf"
            shift
            ;;
        --random-spawn)
            RANDOM_SPAWN=true
            shift
            ;;
        --headless)
            HEADLESS=true
            RVIZ=false
            shift
            ;;
        --no-rviz)
            RVIZ=false
            shift
            ;;
        --spawn-x1)
            SPAWN_X1="$2"
            shift 2
            ;;
        --spawn-y1)
            SPAWN_Y1="$2"
            shift 2
            ;;
        --spawn-x2)
            SPAWN_X2="$2"
            shift 2
            ;;
        --spawn-y2)
            SPAWN_Y2="$2"
            shift 2
            ;;
        -h|--help)
            echo "Usage: ./experiments/step5_2drone_voronoi_mapping.sh [OPTIONS]"
            echo "Options:"
            echo "  --random-spawn       Generate posisi acak untuk 2 drone di arena [-4.5, 4.5]m"
            echo "  --lqr                Gunakan kontroler PID-LQR (default)"
            echo "  --hinf               Gunakan kontroler PID-Hinf"
            echo "  --headless           Jalankan Gazebo tanpa GUI"
            echo "  --no-rviz            Jangan buka RViz2"
            echo "  --spawn-x1/y1 <val>  Posisi spawn iris_1"
            echo "  --spawn-x2/y2 <val>  Posisi spawn iris_2"
            exit 0
            ;;
        *)
            shift
            ;;
    esac
done

# ── Random Spawn Generator ───────────────────────────────────────────────────
if [ "$RANDOM_SPAWN" = true ]; then
    RAND_COORDS=$(python3 -c '
import random, math
while True:
    x1 = round(random.uniform(-4.5, 4.5), 2)
    y1 = round(random.uniform(-4.5, 4.5), 2)
    x2 = round(random.uniform(-4.5, 4.5), 2)
    y2 = round(random.uniform(-4.5, 4.5), 2)
    dist = math.hypot(x1 - x2, y1 - y2)
    if dist >= 1.50:  # Jarak aman awal antar drone minimal 1.5m
        print(f"{x1} {y1} {x2} {y2}")
        break
')
    SPAWN_X1=$(echo "$RAND_COORDS" | awk '{print $1}')
    SPAWN_Y1=$(echo "$RAND_COORDS" | awk '{print $2}')
    SPAWN_X2=$(echo "$RAND_COORDS" | awk '{print $3}')
    SPAWN_Y2=$(echo "$RAND_COORDS" | awk '{print $4}')
fi

echo "========================================================================="
echo "  🛸 [STEP 5] 2-DRONE VORONOI BOUSTROPHEDON MAPPING"
echo "  Kontroler: $CONTROLLER_NAME | Ketinggian: Z = 2.0m (Sama, 2D ORCA Active)"
echo "  iris_1 Spawn: (${SPAWN_X1}, ${SPAWN_Y1}, 0.01m)"
echo "  iris_2 Spawn: (${SPAWN_X2}, ${SPAWN_Y2}, 0.01m)"
echo "========================================================================="
echo ""

echo "=== [1] Membersihkan sisa proses lama ==="
for pid in $(ps aux | grep -E "gz.sim|ros2 launch|test_2drone|parameter_bridge|pid_lqr_node|pid_hinf_node|collision_avoidance_node" | grep -v grep | awk '{print $2}'); do
    kill -9 "$pid" 2>/dev/null || true
done
sleep 1

despawn_active_drones() {
    local models
    models=$(gz model --list 2>/dev/null || true)
    for i in $(seq 1 7); do
        if echo "$models" | grep -q "iris_${i}"; then
            gz service -s /world/swarm_world/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 500 --req "name: \"iris_${i}\", type: 2" >/dev/null 2>&1 || true
        fi
    done
}
despawn_active_drones

cleanup() {
    trap - EXIT INT TERM
    echo ""
    echo "=== Menutup Seluruh Proses & Membersihkan Gazebo World ==="
    if [ -n "$MAPPING_PID" ]; then kill -9 "$MAPPING_PID" 2>/dev/null || true; fi
    if [ -n "$RVIZ_PID" ];    then kill -9 "$RVIZ_PID" 2>/dev/null || true; fi
    if [ -n "$DRONES_PID" ];  then kill -9 "$DRONES_PID" 2>/dev/null || true; fi
    if [ -n "$WORLD_PID" ];   then kill -9 "$WORLD_PID" 2>/dev/null || true; fi
    for pid in $(ps aux | grep -E "gz.sim|ros2 launch|test_2drone|parameter_bridge|pid_lqr_node|pid_hinf_node|collision_avoidance_node|rviz2" | grep -v grep | awk '{print $2}'); do
        kill -9 "$pid" 2>/dev/null || true
    done
    despawn_active_drones
    echo "✅ Selesai (Cleanup Berhasil)."
}
trap cleanup EXIT INT TERM

echo "=== [2] Menjalankan Gazebo Sim World ==="
WORLD_FILE="$WS_DIR/src/swarm_sim/worlds/empty.world"
if [ "$HEADLESS" = true ]; then
    gz sim -s -r "$WORLD_FILE" > /tmp/step5_world.log 2>&1 &
    WORLD_PID=$!
else
    gz sim -r "$WORLD_FILE" > /tmp/step5_world.log 2>&1 &
    WORLD_PID=$!
    if [ "$RVIZ" = true ]; then
        RVIZ_CFG="$WS_DIR/experiments/rviz/step5_2drone.rviz"
        if [ ! -f "$RVIZ_CFG" ]; then
            RVIZ_CFG="$WS_DIR/src/swarm_sim/rviz/swarm.rviz"
        fi
        echo "   -> Membuka RViz2 dengan konfigurasi: $RVIZ_CFG"
        rviz2 -d "$RVIZ_CFG" > /tmp/step5_rviz.log 2>&1 &
        RVIZ_PID=$!
    fi
fi

echo "Menunggu Gazebo World siap..."
for i in $(seq 1 20); do
    if gz topic -l 2>/dev/null | grep -q "/world/swarm_world"; then
        echo "✅ Gazebo World siap setelah ${i}s!"
        break
    fi
    sleep 1
done

echo "=== [3] Spawning 2 Drone & Auto-Takeoff ke Z = 2.0m ==="
ros2 launch swarm_sim spawn_drones_launch.py \
    num_drones:=2 controller:=$CONTROLLER \
    spawn_x1:=$SPAWN_X1 spawn_y1:=$SPAWN_Y1 \
    spawn_x2:=$SPAWN_X2 spawn_y2:=$SPAWN_Y2 \
    results_base:=multi_agent > /tmp/step5_drones.log 2>&1 &
DRONES_PID=$!

echo "Menunggu kedua drone terdeteksi odometrinya..."
for i in $(seq 1 25); do
    TOPICS=$(ros2 topic list 2>/dev/null || true)
    if echo "$TOPICS" | grep -q "/iris_1/odometry" && echo "$TOPICS" | grep -q "/iris_2/odometry"; then
        echo "✅ Kedua drone aktif dan mengudara setelah ${i}s!"
        break
    fi
    sleep 1
done

echo ""
echo "=== [4] Menjalankan Node Koordinator Pemetaan Voronoi (2 Drone) ==="
python3 "$WS_DIR/experiments/test_2drone_voronoi_mapping.py" &
MAPPING_PID=$!

wait "$MAPPING_PID"
