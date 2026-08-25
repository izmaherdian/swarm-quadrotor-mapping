#!/usr/bin/env bash
# ==============================================================================
#   STEP 6: SWARM 7-DRONE 2D VORONOI BOUSTROPHEDON MAPPING (ARENA 30x30m)
# ==============================================================================
#  Alur Eksekusi:
#    1. Generate koordinat spawn acak untuk 7 drone (iris_1 s/d iris_7) di arena [-12, 12]m
#    2. Jalankan Gazebo Sim World (Arena 30x30m) & RViz2 (step6_7drone.rviz)
#    3. Spawn 7 Drone dengan Auto-Takeoff ke Z = 2.0m (PID-LQR)
#    4. Jalankan Node Koordinator Centroidal Voronoi (12x Lloyd) + Boustrophedon
#    5. Monitor visualisasi pemetaan serentak (100% Coverage pada 900 m^2)
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
RANDOM_SPAWN=true
HEADLESS=false
RVIZ=true

# Posisi spawn default berbaris rapi jika tidak random
SPAWN_X1="-10.0"; SPAWN_Y1="-12.0"
SPAWN_X2="-10.0"; SPAWN_Y2="-8.0"
SPAWN_X3="-10.0"; SPAWN_Y3="-4.0"
SPAWN_X4="-10.0"; SPAWN_Y4="0.0"
SPAWN_X5="-10.0"; SPAWN_Y5="4.0"
SPAWN_X6="-10.0"; SPAWN_Y6="8.0"
SPAWN_X7="-10.0"; SPAWN_Y7="12.0"

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
        --fixed-spawn)
            RANDOM_SPAWN=false
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
        -h|--help)
            echo "Usage: ./experiments/step6_7drone_voronoi_mapping.sh [OPTIONS]"
            echo "Options:"
            echo "  --random-spawn       Generate posisi acak untuk 7 drone di arena [-12, 12]m (default)"
            echo "  --fixed-spawn        Gunakan posisi baris teratur default"
            echo "  --lqr                Gunakan kontroler PID-LQR (default)"
            echo "  --hinf               Gunakan kontroler PID-Hinf"
            echo "  --headless           Jalankan Gazebo tanpa GUI"
            echo "  --no-rviz            Jangan buka RViz2"
            exit 0
            ;;
        *)
            shift
            ;;
    esac
done

# ── Random Spawn Generator untuk 7 Drone ─────────────────────────────────────
if [ "$RANDOM_SPAWN" = true ]; then
    RAND_COORDS=$(python3 -c '
import random, math
pts = []
while len(pts) < 7:
    x = round(random.uniform(-11.0, 11.0), 2)
    y = round(random.uniform(-11.0, 11.0), 2)
    ok = True
    for px, py in pts:
        if math.hypot(x - px, y - py) < 1.50:
            ok = False
            break
    if ok:
        pts.append((x, y))

out = " ".join(f"{x} {y}" for x, y in pts)
print(out)
')
    read -r SPAWN_X1 SPAWN_Y1 SPAWN_X2 SPAWN_Y2 SPAWN_X3 SPAWN_Y3 SPAWN_X4 SPAWN_Y4 SPAWN_X5 SPAWN_Y5 SPAWN_X6 SPAWN_Y6 SPAWN_X7 SPAWN_Y7 <<< "$RAND_COORDS"
fi

echo "========================================================================="
echo "  🛸 [STEP 6] SWARM 7-DRONE 2D VORONOI BOUSTROPHEDON MAPPING"
echo "  Arena: 30 x 30 Meter (900 m^2) | Kontroler: $CONTROLLER_NAME | Ketinggian: Z = 2.0m"
echo "  iris_1: (${SPAWN_X1}, ${SPAWN_Y1}) | iris_2: (${SPAWN_X2}, ${SPAWN_Y2}) | iris_3: (${SPAWN_X3}, ${SPAWN_Y3})"
echo "  iris_4: (${SPAWN_X4}, ${SPAWN_Y4}) | iris_5: (${SPAWN_X5}, ${SPAWN_Y5}) | iris_6: (${SPAWN_X6}, ${SPAWN_Y6})"
echo "  iris_7: (${SPAWN_X7}, ${SPAWN_Y7})"
echo "========================================================================="
echo ""

echo "=== [1] Membersihkan sisa proses lama ==="
for pid in $(ps aux | grep -E "gz.sim|ros2 launch|test_7drone|test_2drone|parameter_bridge|pid_lqr_node|pid_hinf_node|collision_avoidance_node" | grep -v grep | awk '{print $2}'); do
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
sleep 1

WORLD_FILE="$WS_DIR/src/swarm_sim/worlds/empty.world"
RVIZ_CFG="$WS_DIR/experiments/rviz/step6_7drone.rviz"

# Trap exit untuk cleanup mulus
cleanup() {
    echo ""
    echo "=== Menutup Seluruh Proses & Membersihkan Gazebo World ==="
    pkill -9 -f "test_7drone_voronoi_mapping" 2>/dev/null || true
    pkill -9 -f "spawn_drones_launch" 2>/dev/null || true
    pkill -9 -f "parameter_bridge" 2>/dev/null || true
    pkill -9 -f "pid_lqr_node" 2>/dev/null || true
    pkill -9 -f "pid_hinf_node" 2>/dev/null || true
    pkill -9 -f "collision_avoidance_node" 2>/dev/null || true
    pkill -9 -f "rviz2" 2>/dev/null || true
    pkill -9 -f "gz sim" 2>/dev/null || true
    pkill -9 -f "gz-sim" 2>/dev/null || true
    despawn_active_drones
    echo "✅ Selesai (Cleanup Berhasil)."
}
trap cleanup EXIT INT TERM

echo "=== [2] Meluncurkan Gazebo Simulator (Arena 30x30m) ==="
if [ "$HEADLESS" = true ]; then
    gz sim -r -s "$WORLD_FILE" > /tmp/step6_world.log 2>&1 &
else
    gz sim -r "$WORLD_FILE" > /tmp/step6_world.log 2>&1 &
fi
GZ_PID=$!

echo "Menunggu Gazebo World siap..."
for i in $(seq 1 30); do
    if gz topic -l 2>/dev/null | grep -q "/world/swarm_world/clock"; then
        echo "Gazebo Clock terdeteksi ($i detik)."
        break
    fi
    sleep 1
done

if [ "$RVIZ" = true ]; then
    echo "=== [3] Membuka RViz2 (step6_7drone.rviz) ==="
    rviz2 -d "$RVIZ_CFG" > /tmp/step6_rviz.log 2>&1 &
    RVIZ_PID=$!
    sleep 2
fi

echo "=== [4] Spawning 7 Drones & Kontroler Low-Level ==="
ros2 launch swarm_sim spawn_drones_launch.py \
    num_drones:=7 \
    controller:="$CONTROLLER" \
    spawn_x1:="$SPAWN_X1" spawn_y1:="$SPAWN_Y1" \
    spawn_x2:="$SPAWN_X2" spawn_y2:="$SPAWN_Y2" \
    spawn_x3:="$SPAWN_X3" spawn_y3:="$SPAWN_Y3" \
    spawn_x4:="$SPAWN_X4" spawn_y4:="$SPAWN_Y4" \
    spawn_x5:="$SPAWN_X5" spawn_y5:="$SPAWN_Y5" \
    spawn_x6:="$SPAWN_X6" spawn_y6:="$SPAWN_Y6" \
    spawn_x7:="$SPAWN_X7" spawn_y7:="$SPAWN_Y7" \
    results_base:=multi_agent > /tmp/step6_drones.log 2>&1 &

echo "Menunggu 7 drone ter-spawn & odometri aktif..."
for i in $(seq 1 35); do
    TOPIC_COUNT=$(ros2 topic list 2>/dev/null | grep -E "/iris_[1-7]/odometry" | wc -l || true)
    if [ "$TOPIC_COUNT" -ge 7 ]; then
        echo "Seluruh 7 Odometri Drone Terdeteksi ($TOPIC_COUNT/7) pada detik ke-$i!"
        break
    fi
    sleep 1
done

sleep 3

echo "=== [5] Meluncurkan Swarm 7-Drone Voronoi Mapping Coordinator ==="
python3 "$WS_DIR/experiments/test_7drone_voronoi_mapping.py"
