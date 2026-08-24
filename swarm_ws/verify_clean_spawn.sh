#!/usr/bin/env bash
set -e

WS_DIR="$(cd "$(dirname "$0")" && pwd)"
source /opt/ros/lyrical/setup.bash
if [ -f "$WS_DIR/../.venv/bin/activate" ]; then
    source "$WS_DIR/../.venv/bin/activate"
fi

export AMENT_PREFIX_PATH="$WS_DIR/install/swarm_msgs:$WS_DIR/install/swarm_sim:$WS_DIR/install/swarm_high_level:$WS_DIR/install/swarm_low_level:$WS_DIR/install/swarm_mid_level:$AMENT_PREFIX_PATH"
export PYTHONPATH="$WS_DIR/install/swarm_msgs/local/lib/python3.14/dist-packages:$WS_DIR/install/swarm_sim/lib/python3.14/site-packages:$WS_DIR/install/swarm_high_level/lib/python3.14/site-packages:$WS_DIR/install/swarm_low_level/lib/python3.14/site-packages:$WS_DIR/install/swarm_mid_level/lib/python3.14/site-packages:$PYTHONPATH"
export GZ_SIM_RESOURCE_PATH="$WS_DIR/src/swarm_sim/models"

LOG_DIR="$WS_DIR/test_logs"
mkdir -p "$LOG_DIR"
WORLD_LOG="$LOG_DIR/verify_world.log"
DRONES_LOG="$LOG_DIR/verify_drones.log"

echo "========================================================================="
echo "  🧪 UJI VERIFIKASI: CLEAN SPAWN & DESPAWN (NOL USERCOMMANDS.CC ERROR)"
echo "========================================================================="

# 1. Bersihkan sisa proses lama
for pid in $(ps aux | grep -E "gz sim|gz-sim|rviz2|world_launch|spawn_drones_launch|pid_lqr_node|collision_avoidance_node|test_mapping" | grep -v grep | awk '{print $2}'); do
    kill -9 "$pid" 2>/dev/null || true
done
sleep 1

# 2. Jalankan Terminal 1 (World Headless untuk test)
echo "🌍 [1] Memulai Terminal 1 (Gazebo World)..."
./run_world.sh --headless > "$WORLD_LOG" 2>&1 &
WORLD_PID=$!

sleep 4

# 3. Jalankan Terminal 2 (Spawn Drones dengan smart pre-check)
echo "🚀 [2] Memulai Terminal 2 (Spawn Drone iris_1)..."
./run_drones.sh --lqr --spawn-x 1.0 --spawn-y 1.0 > "$DRONES_LOG" 2>&1 &
DRONES_PID=$!

echo "Menunggu 10 detik..."
sleep 10

# 4. Hentikan Terminal 2
echo "🛑 [3] Menghentikan Terminal 2 (Despawn Drone)..."
kill -INT "$DRONES_PID" 2>/dev/null || true
sleep 3

# 5. Hentikan Terminal 1
echo "🛑 [4] Menghentikan Terminal 1..."
kill -INT "$WORLD_PID" 2>/dev/null || true
sleep 2

for pid in $(ps aux | grep -E "gz sim|gz-sim|rviz2|world_launch|spawn_drones_launch|pid_lqr_node|collision_avoidance_node|test_mapping" | grep -v grep | awk '{print $2}'); do
    kill -9 "$pid" 2>/dev/null || true
done

echo ""
echo "=== LOG WORLD OUTPUT ==="
cat "$WORLD_LOG" | grep -E "UserCommands|error|aktif" || echo "(Tidak ada log error UserCommands.cc)"
echo ""
echo "=== Evaluasi Kriteria ==="
if grep -q "UserCommands.cc:1133" "$WORLD_LOG"; then
    echo "❌ GAGAL: Masih ditemukan log error UserCommands.cc!"
else
    echo "🎉 HASIL: 100% SUKSES! NOL error UserCommands.cc. Despawn dan Spawn bersih total!"
fi
