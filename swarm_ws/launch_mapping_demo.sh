#!/usr/bin/env bash
# ==============================================================================
# Script 1-Klik Launch Simulasi Visual 7-Drone Voronoi Mapping
# Gazebo GUI + RViz2 + 7-Drone Spawner + Voronoi Mapping Node
# ==============================================================================

set -e

WS_DIR="/home/izmaherdian/Documents/swarm-quadrotor-mapping/swarm_ws"
WORLD_FILE="$WS_DIR/src/swarm_sim/worlds/empty.world"
RVIZ_CONFIG="$WS_DIR/src/swarm_sim/rviz/multi_agent.rviz"

# Setup ROS 2 Environment
source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null || true
source "$WS_DIR/install/setup.bash"

# Export Gazebo Resource Path
export GZ_SIM_RESOURCE_PATH="$WS_DIR/src/swarm_sim/models:$GZ_SIM_RESOURCE_PATH"

echo "========================================================================="
echo "  🚀 MEMULAI SIMULASI VISUAL SWARM 7-DRONE VORONOI MAPPING"
echo "========================================================================="

# Fungsi Cleanup saat user menekan Ctrl+C
cleanup() {
    echo ""
    echo "🛑 Menghentikan seluruh proses Gazebo, RViz2, dan ROS 2 nodes..."
    pkill -9 -f "gz sim" 2>/dev/null || true
    pkill -9 -f "spawn_drones" 2>/dev/null || true
    pkill -9 -f "pid_lqr_node" 2>/dev/null || true
    pkill -9 -f "ros_gz_bridge" 2>/dev/null || true
    pkill -9 -f "rviz2" 2>/dev/null || true
    pkill -9 -f "test_7drone_voronoi_mapping" 2>/dev/null || true
    echo "✅ Semua proses berhasil dibersihkan."
    exit 0
}

trap cleanup SIGINT SIGTERM EXIT

# 1. Bersihkan proses lama
echo "🧹 Membersihkan proses lama..."
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f "spawn_drones" 2>/dev/null || true
pkill -9 -f "pid_lqr_node" 2>/dev/null || true
pkill -9 -f "ros_gz_bridge" 2>/dev/null || true
pkill -9 -f "rviz2" 2>/dev/null || true
pkill -9 -f "test_7drone_voronoi_mapping" 2>/dev/null || true
sleep 1

# 2. Jalankan Gazebo Simulator GUI
echo "1️⃣  Menjalankan Gazebo Simulator GUI..."
gz sim -r "$WORLD_FILE" &
GZ_PID=$!

echo "   ⏳ Menunggu Gazebo Engine siap..."
for t in $(seq 1 30); do
    if gz topic -l 2>/dev/null | grep -q "/world/swarm_world/clock"; then
        echo "   ✅ Gazebo Engine siap!"
        break
    fi
    sleep 0.5
done

# 3. Spawn 7 Drone Quadrotor & Controller PID-LQR
echo "2️⃣  Melakukan Spawn 7 Drone Quadrotor & Low-Level Flight Controller..."
ros2 launch swarm_sim spawn_drones_launch.py \
    num_drones:=7 \
    controller:=pid_lqr_node \
    use_mid_level:=true \
    spawn_x1:="-9.0" spawn_y1:="-9.0" \
    spawn_x2:="0.0"  spawn_y2:="-9.0" \
    spawn_x3:="9.0"  spawn_y3:="-9.0" \
    spawn_x4:="-9.0" spawn_y4:="0.0" \
    spawn_x5:="0.0"  spawn_y5:="0.0" \
    spawn_x6:="9.0"  spawn_y6:="0.0" \
    spawn_x7:="0.0"  spawn_y7:="9.0" \
    results_base:=multi_agent &

echo "   ⏳ Menunggu 7 drone terdeteksi aktif..."
for t in $(seq 1 35); do
    TOPIC_COUNT=$(ros2 topic list 2>/dev/null | grep -E "/iris_[1-7]/odometry" | wc -l || true)
    if [ "$TOPIC_COUNT" -ge 7 ]; then
        echo "   ✅ 7 Drone Quadrotor siap di udara (7/7 Odometry Online)!"
        break
    fi
    sleep 1
done

# 4. Jalankan RViz2
if [ -f "$RVIZ_CONFIG" ]; then
    echo "3️⃣  Membuka RViz2 Visualizer..."
    rviz2 -d "$RVIZ_CONFIG" --ros-args -p use_sim_time:=true &
fi
sleep 2

# 5. Jalankan Node Pemetaan Voronoi & Boustrophedon
echo "4️⃣  Menjalankan Algoritma Pemetaan Voronoi Swarm..."
echo "========================================================================="
python3 "$WS_DIR/experiments/test_7drone_voronoi_mapping.py" --ros-args -p use_sim_time:=true
