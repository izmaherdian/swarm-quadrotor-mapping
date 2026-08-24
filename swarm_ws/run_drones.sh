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

CONTROLLER="pid_lqr_node"
CONTROLLER_NAME="PID-LQR"
NUM_DRONES=1
SPAWN_X="-5.5"
SPAWN_Y="-5.5"

while [[ $# -gt 0 ]]; do
    case $1 in
        --hinf)
            CONTROLLER="pid_hinf_node"
            CONTROLLER_NAME="PID-Hinf"
            shift
            ;;
        --lqr)
            CONTROLLER="pid_lqr_node"
            CONTROLLER_NAME="PID-LQR"
            shift
            ;;
        --drones)
            NUM_DRONES="$2"
            shift 2
            ;;
        --spawn-x)
            SPAWN_X="$2"
            shift 2
            ;;
        --spawn-y)
            SPAWN_Y="$2"
            shift 2
            ;;
        --random-spawn|--random)
            # Generate random X dan Y antara -4.50 sampai +4.50
            SPAWN_X=$(python3 -c "import random; print(f'{random.uniform(-4.5, 4.5):.2f}')")
            SPAWN_Y=$(python3 -c "import random; print(f'{random.uniform(-4.5, 4.5):.2f}')")
            echo "🎲 [RANDOM SPAWN] Posisi Acak Terpilih: (${SPAWN_X}, ${SPAWN_Y})"
            shift
            ;;
        -h|--help)
            echo "Usage: ./run_drones.sh [OPTIONS]"
            echo "Options:"
            echo "  --lqr                Gunakan kontroler PID-LQR (default)"
            echo "  --hinf               Gunakan kontroler PID-Hinf"
            echo "  --spawn-x <X>        Posisi X awal spawn (default: -5.5)"
            echo "  --spawn-y <Y>        Posisi Y awal spawn (default: -5.5)"
            echo "  --random-spawn       Pilih posisi spawn secara acak dalam arena [-4.5, 4.5]"
            echo "  --drones <N>         Jumlah drone (default: 1)"
            exit 0
            ;;
        *)
            shift
            ;;
    esac
done

echo "=== [TERMINAL 2] Memeriksa status Gazebo Sim World (Terminal 1) ==="
if ! gz topic -l 2>/dev/null | grep -q "/world/swarm_world"; then
    echo ""
    echo "⚠️  PERINGATAN: Gazebo World (/world/swarm_world) belum aktif!"
    echo "   Silakan jalankan TERMINAL 1 terlebih dahulu:"
    echo "   $ ./run_world.sh"
    echo ""
    exit 1
fi

despawn_active_drones() {
    local models
    models=$(gz model --list 2>/dev/null || true)
    for i in $(seq 1 7); do
        if echo "$models" | grep -q "iris_${i}"; then
            gz service -s /world/swarm_world/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 500 --req "name: \"iris_${i}\", type: 2" >/dev/null 2>&1 || true
        fi
    done
}

echo "=== [TERMINAL 2] Bersihkan sisa proses kontroler & bridge lama ==="
for pid in $(ps aux | grep -E "spawn_drones_launch|pid_lqr_node|pid_hinf_node|collision_avoidance_node|tf_prefix_node|bridge_iris" | grep -v grep | awk '{print $2}'); do
    kill -9 "$pid" 2>/dev/null || true
done
sleep 1

# Hapus model iris lama hanya jika memang ada di Gazebo
despawn_active_drones

echo "=== [TERMINAL 2] Build paket kontroler & simulasi ==="
colcon build --packages-select swarm_low_level swarm_mid_level swarm_sim 2>&1 | tail -4
source "$WS_DIR/install/setup.bash"

cleanup() {
    trap - EXIT INT TERM
    echo ""
    echo "=== [TERMINAL 2] Menutup Kontroler & Menghapus Drone dari Gazebo ==="
    if [ -n "$DRONES_PID" ]; then
        kill "$DRONES_PID" 2>/dev/null || true
    fi
    for pid in $(ps aux | grep -E "spawn_drones_launch|pid_lqr_node|pid_hinf_node|collision_avoidance_node|tf_prefix_node|bridge_iris" | grep -v grep | awk '{print $2}'); do
        kill -9 "$pid" 2>/dev/null || true
    done
    
    # Despawn model dari Gazebo agar dunia kembali bersih
    despawn_active_drones
    echo "✅ Model drone berhasil di-despawn bersih dari Gazebo."
    echo "Done (Terminal 2 Ditutup)."
}
trap cleanup EXIT INT TERM

echo ""
echo "========================================================================="
echo "  🚀 [TERMINAL 2] SPAWN DRONE & AUTO-TAKEOFF KE Z = 2.0 METER"
echo "  Drone: iris_1 ($NUM_DRONES drone) | Kontroler: $CONTROLLER_NAME ($CONTROLLER)"
echo "  Posisi Spawn: ($SPAWN_X, $SPAWN_Y, 0.01m) -> Target Hover: ($SPAWN_X, $SPAWN_Y, 2.00m)"
echo "========================================================================="
echo ""

ros2 launch swarm_sim spawn_drones_launch.py \
    num_drones:=$NUM_DRONES controller:=$CONTROLLER results_base:=single_agent \
    spawn_x:=$SPAWN_X spawn_y:=$SPAWN_Y &
DRONES_PID=$!

echo "Menunggu drone lepas landas & hover stabil di Z = 2.0m..."
for i in $(seq 1 20); do
    if ros2 topic list 2>/dev/null | grep -q "/iris_1/odometry"; then
        echo "✅ Sensor Odometry & Kontroler aktif setelah ${i}s!"
        break
    fi
    sleep 1
done

echo ""
echo "========================================================================="
echo "  ✅ DRONE TELAH MENGUDARA & HOVER STABIL DI Z = 2.0m!"
echo ""
echo "  👉 Sekarang buka TERMINAL 3 untuk menjalankan misi pemetaan:"
echo "     cd $WS_DIR"
echo "     ./run_mapping.sh --yaw-follow    # (Hidung aktif hadap arah terbang)"
echo "     # atau"
echo "     ./run_mapping.sh --fixed-yaw     # (Orientasi tetap 0°)"
echo ""
echo "  ℹ️  Terminal 2 ini dapat dibiarkan terus berjalan."
echo "     Jika Anda tekan Ctrl+C di terminal ini, drone akan di-despawn dari Gazebo."
echo "========================================================================="
echo ""

wait "$DRONES_PID"
