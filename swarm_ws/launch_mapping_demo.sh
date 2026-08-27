#!/usr/bin/env bash
# ==============================================================================
# Script 1-Klik Launch Simulasi Visual 7-Drone Voronoi Mapping
# Gazebo GUI + RViz2 + 7-Drone Spawner + Voronoi Mapping Node
#
# Pilihan Mode Kontroler Low-Level:
#   ./launch_mapping_demo.sh            # Default: Mode PID-LQR
#   ./launch_mapping_demo.sh --pid-lqr  # Mode PID-LQR (Optimal Control)
#   ./launch_mapping_demo.sh --pid-hinf # Mode PID-H-Infinity (Robust Control)
# ==============================================================================

set -e

WS_DIR="/home/izmaherdian/Documents/swarm-quadrotor-mapping/swarm_ws"
WORLD_FILE="$WS_DIR/src/swarm_sim/worlds/empty.world"
RVIZ_CONFIG="$WS_DIR/src/swarm_sim/rviz/multi_agent.rviz"

# Setup ROS 2 Environment
source /opt/ros/lyrical/setup.bash 2>/dev/null || source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null || true
source "$WS_DIR/install/setup.bash"

# Export Gazebo Resource Path
export GZ_SIM_RESOURCE_PATH="$WS_DIR/src/swarm_sim/models:$GZ_SIM_RESOURCE_PATH"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
MAGENTA='\033[0;35m'
BOLD='\033[1m'
NC='\033[0m'

# Default Mode: PID-LQR
CONTROLLER="pid_lqr_node"
CONTROLLER_TITLE="PID-LQR (Optimal Linear Quadratic Regulator)"
CONTROLLER_COLOR="$CYAN"

# Parsing Argumen CLI
while [[ $# -gt 0 ]]; do
  case $1 in
    --pid-lqr|-lqr)
      CONTROLLER="pid_lqr_node"
      CONTROLLER_TITLE="PID-LQR (Optimal Linear Quadratic Regulator)"
      CONTROLLER_COLOR="$CYAN"
      shift
      ;;
    --pid-hinf|-hinf|--pid-h-infinity)
      CONTROLLER="pid_hinf_node"
      CONTROLLER_TITLE="PID-H-Infinity (Robust Disturbance Attenuation)"
      CONTROLLER_COLOR="$MAGENTA"
      shift
      ;;
    -h|--help)
      echo -e "${BOLD}Penggunaan:${NC}"
      echo "  ./launch_mapping_demo.sh            # Default: Mode PID-LQR"
      echo "  ./launch_mapping_demo.sh --pid-lqr  # Jalankan dengan Kontroler PID-LQR"
      echo "  ./launch_mapping_demo.sh --pid-hinf # Jalankan dengan Kontroler PID-H-Infinity"
      echo ""
      echo -e "${BOLD}Opsi:${NC}"
      echo "  --pid-lqr, -lqr     Menggunakan kontroler low-level PID-LQR"
      echo "  --pid-hinf, -hinf   Menggunakan kontroler low-level PID-H-Infinity (Robust)"
      echo "  -h, --help          Menampilkan panduan ini"
      exit 0
      ;;
    *)
      echo -e "${YELLOW}⚠️  Opsi '$1' tidak dikenal, menggunakan default PID-LQR.${NC}"
      shift
      ;;
  esac
done

echo "========================================================================="
echo -e "  🚀 MEMULAI SIMULASI VISUAL SWARM 7-DRONE VORONOI MAPPING"
echo -e "  🎮 MODE KONTROLER LOW-LEVEL: ${CONTROLLER_COLOR}${BOLD}[$CONTROLLER_TITLE]${NC}"
echo "  💡 Terminal 2 Fault Injection: ./kill_drone.sh <id...> (Contoh: ./kill_drone.sh 4)"
echo "========================================================================="

# Fungsi Cleanup saat user menekan Ctrl+C
cleanup() {
    echo ""
    echo "🛑 Menghentikan seluruh proses Gazebo, RViz2, dan ROS 2 nodes..."
    pkill -9 -f "gz sim" 2>/dev/null || true
    pkill -9 -f "spawn_drones" 2>/dev/null || true
    pkill -9 -f "pid_lqr_node" 2>/dev/null || true
    pkill -9 -f "pid_hinf_node" 2>/dev/null || true
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
pkill -9 -f "pid_hinf_node" 2>/dev/null || true
pkill -9 -f "ros_gz_bridge" 2>/dev/null || true
pkill -9 -f "rviz2" 2>/dev/null || true
pkill -9 -f "test_7drone_voronoi_mapping" 2>/dev/null || true
sleep 1

# 2. Jalankan Gazebo Simulator GUI dengan custom gui config
GUI_CONFIG="$(ros2 pkg prefix swarm_sim 2>/dev/null)/share/swarm_sim/config/gazebo_gui.config"
if [ ! -f "$GUI_CONFIG" ]; then
    GUI_CONFIG="/home/izmaherdian/Documents/swarm-quadrotor-mapping/swarm_ws/src/swarm_sim/config/gazebo_gui.config"
fi

echo "1️⃣  Menjalankan Gazebo Simulator GUI..."
gz sim -r --gui-config "$GUI_CONFIG" "$WORLD_FILE" &
GZ_PID=$!

echo "   ⏳ Menunggu Gazebo Engine siap..."
for t in $(seq 1 30); do
    if gz topic -l 2>/dev/null | grep -q "/world/swarm_world/clock"; then
        echo "   ✅ Gazebo Engine siap!"
        break
    fi
    sleep 0.5
done

# 3. Spawn 7 Drone Quadrotor & Controller Terpilih
echo -e "2️⃣  Melakukan Spawn 7 Drone & Flight Controller ${CONTROLLER_COLOR}${BOLD}[$CONTROLLER]${NC}..."
ros2 launch swarm_sim spawn_drones_launch.py \
    num_drones:=7 \
    controller:="$CONTROLLER" \
    use_mid_level:=false \
    spawn_x1:="-3.0" spawn_y1:="-18.0" \
    spawn_x2:="-1.0" spawn_y2:="-18.0" \
    spawn_x3:="1.0"  spawn_y3:="-18.0" \
    spawn_x4:="3.0"  spawn_y4:="-18.0" \
    spawn_x5:="-2.0" spawn_y5:="-16.5" \
    spawn_x6:="0.0"  spawn_y6:="-16.5" \
    spawn_x7:="2.0"  spawn_y7:="-16.5" \
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
