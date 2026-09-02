#!/usr/bin/env bash
# ==============================================================================
# Script 1-Klik Launch Simulasi Visual Swarm 7-Drone Voronoi Mapping
# Gazebo GUI + RViz2 + 7-Drone Spawner + Dryden Wind + Obstacles + Voronoi Node
#
# Pilihan 4 Skema Pemetaan:
#   --scheme 1 | -s 1   : Skema 1 - Nominal Mapping (Baseline, Zero Disturbance)
#   --scheme 2 | -s 2   : Skema 2 - Dryden Wind Turbulence Mapping (σ=2.5N, τ=0.5s)
#   --scheme 3 | -s 3   : Skema 3 - Obstacle Avoidance (9 rintangan statis)
#   --scheme 4 | -s 4   : Skema 4 - Combined Wind & Obstacles Disturbance Mapping
#
# Pilihan Mode Kontroler Low-Level:
#   --pid-lqr  | -lqr   : Mode PID-LQR (Optimal Linear Quadratic Regulator)
#   --pid-hinf | -hinf  : Mode PID-H-Infinity (Robust Disturbance Attenuation)
# ==============================================================================

set -e

WS_DIR="/home/izmaherdian/Documents/swarm-quadrotor-mapping/swarm_ws"
RVIZ_CONFIG="$WS_DIR/src/swarm_sim/rviz/multi_agent.rviz"

# Setup ROS 2 Environment
source /opt/ros/lyrical/setup.bash 2>/dev/null || source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null || true
source "$WS_DIR/install/setup.bash"

# Export Gazebo Resource Path & Python Path
export GZ_SIM_RESOURCE_PATH="$WS_DIR/src/swarm_sim/models:$GZ_SIM_RESOURCE_PATH"
export PYTHONPATH="$WS_DIR/src/swarm_high_level:$WS_DIR/src/swarm_mid_level:$WS_DIR/src/swarm_low_level:${PYTHONPATH:-}"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
MAGENTA='\033[0;35m'
BOLD='\033[1m'
NC='\033[0m'

# Default: Skema 1 & PID-LQR
SCHEME=1
HEADLESS=false         # --headless untuk tanpa GUI Gazebo & RViz
RESULTS_DIR=""         # --results <dir> untuk memisahkan hasil tiap run
REGION="rect"          # --region <preset|path.yaml> bentuk wilayah pemetaan
SWEEP_SPEED=""         # --sweep-speed <m/s> override kecepatan sapuan
EXIT_AFTER="3.0"        # --exit-after <detik-sim> berhenti otomatis setelah misi tuntas
CONTROLLER="pid_lqr_node"
CONTROLLER_TITLE="PID-LQR (Optimal Linear Quadratic Regulator)"
CONTROLLER_COLOR="$CYAN"

# Parsing Argumen CLI
while [[ $# -gt 0 ]]; do
  case $1 in
    --scheme|-s)
      SCHEME="$2"
      shift 2
      ;;
    --scheme=*)
      SCHEME="${1#*=}"
      shift
      ;;
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
    --results)
      RESULTS_DIR="$2"
      shift 2
      ;;
    --region)
      REGION="$2"
      shift 2
      ;;
    --region=*)
      REGION="${1#*=}"
      shift
      ;;
    --sweep-speed)
      SWEEP_SPEED="$2"
      shift 2
      ;;
    --exit-after)
      EXIT_AFTER="$2"
      shift 2
      ;;
    --headless)
      HEADLESS=true
      shift
      ;;
    -h|--help)
      echo -e "${BOLD}Penggunaan:${NC}"
      echo "  ./launch_mapping_demo.sh [OPSI]"
      echo ""
      echo -e "${BOLD}Pilihan Skema Pengujian (--scheme / -s):${NC}"
      echo "  -s 1  : Skema 1 (Nominal / Baseline - Zero Disturbance)"
      echo "  -s 2  : Skema 2 (Dryden Wind Turbulence Disturbances)"
      echo "  -s 3  : Skema 3 (Obstacle Avoidance: 9 rintangan statis per wilayah)"
      echo "  -s 4  : Skema 4 (Combined: Dryden Wind + Obstacles Statis & Dinamis)"
      echo ""
      echo -e "${BOLD}Pilihan Kontroler Low-Level:${NC}"
      echo "  --pid-lqr, -lqr     Menggunakan kontroler low-level PID-LQR"
      echo "  --pid-hinf, -hinf   Menggunakan kontroler low-level PID-H-Infinity (Robust)"
      echo ""
      echo -e "${BOLD}Lainnya:${NC}"
      echo "  --headless            Tanpa GUI Gazebo & RViz (jauh lebih ringan)"
      echo "  --results <dir>       Simpan CSV telemetri ke direktori tersebut"
      echo "  --region <preset|yaml> Bentuk wilayah pemetaan: rect (default),"
      echo "                        l_shape, u_shape, plus, atau path berkas YAML"
      echo "  --sweep-speed <m/s>   Override kecepatan sapuan baris (default 1.6)"
      echo ""
      echo -e "${BOLD}Contoh:${NC}"
      echo "  ./launch_mapping_demo.sh -s 1 --pid-lqr"
      echo "  ./launch_mapping_demo.sh -s 2 --pid-hinf"
      echo "  ./launch_mapping_demo.sh -s 1 --pid-lqr --region u_shape"
      echo "  ./launch_mapping_demo.sh -s 4 --pid-hinf"
      exit 0
      ;;
    *)
      echo -e "${YELLOW}⚠️  Opsi '$1' tidak dikenal, diabaikan.${NC}"
      shift
      ;;
  esac
done

# Konfigurasi Berdasarkan Skema
case $SCHEME in
  1)
    SCHEME_NAME="Skema 1: Nominal Mapping (Baseline, Zero Disturbance)"
    WORLD_FILE="$WS_DIR/src/swarm_sim/worlds/empty.world"
    ENABLE_WIND="false"
    ENABLE_OBSTACLES="false"
    ENABLE_DYN_OBSTACLES="false"
    ;;
  2)
    SCHEME_NAME="Skema 2: Dryden Wind Turbulence Mapping (σ=2.5N, τ=0.5s + Gust)"
    WORLD_FILE="$WS_DIR/src/swarm_sim/worlds/empty.world"
    ENABLE_WIND="true"
    ENABLE_OBSTACLES="false"
    ENABLE_DYN_OBSTACLES="false"
    ;;
  3)
    # Skema 3 = rintangan STATIS saja. Tiap wilayah punya berkas world
    # sendiri berisi 9 silinder DI DALAM wilayah itu, dibangkitkan oleh
    # tools/gen_obstacle_worlds.py, tanpa silinder dinamis sama sekali.
    SCHEME_NAME="Skema 3: Obstacle Avoidance Mapping (9 rintangan statis)"
    WORLD_FILE="$WS_DIR/src/swarm_sim/worlds/obstacles_${REGION}.world"
    if [ ! -f "$WORLD_FILE" ]; then
      echo -e "${RED}❌ World Skema 3 untuk wilayah '$REGION' tidak ada:${NC}"
      echo -e "   $WORLD_FILE"
      echo -e "   Wilayah yang punya world Skema 3: rect, l_shape, u_shape, plus."
      echo -e "   Bila baru menambah wilayah, jalankan: ${BOLD}python3 tools/gen_obstacle_worlds.py${NC}"
      exit 1
    fi
    ENABLE_WIND="false"
    ENABLE_OBSTACLES="true"
    ENABLE_DYN_OBSTACLES="false"
    ;;
  4)
    SCHEME_NAME="Skema 4: Static & Dynamic Obstacle Avoidance (9 Statis + 2 Dinamis Pola X)"
    WORLD_FILE="$WS_DIR/src/swarm_sim/worlds/obstacles.world"
    ENABLE_WIND="false"
    ENABLE_OBSTACLES="true"
    ENABLE_DYN_OBSTACLES="true"
    ;;
  5)
    SCHEME_NAME="Skema 5: Combined Multi-Hazard Disturbance Mapping (Dryden Wind + 9 Statis + 2 Dinamis Pola X)"
    WORLD_FILE="$WS_DIR/src/swarm_sim/worlds/obstacles.world"
    ENABLE_WIND="true"
    ENABLE_OBSTACLES="true"
    ENABLE_DYN_OBSTACLES="true"
    ;;
  *)
    echo -e "${RED}❌ Skema '$SCHEME' tidak valid! Pilih antara 1, 2, 3, 4, atau 5.${NC}"
    exit 1
    ;;
esac

echo "========================================================================="
echo -e "  🚀 MEMULAI SIMULASI VISUAL SWARM 7-DRONE VORONOI MAPPING"
echo -e "  📋 SKEMA PENGUJIAN        : ${YELLOW}${BOLD}[$SCHEME_NAME]${NC}"
echo -e "  🎮 KONTROLER LOW-LEVEL     : ${CONTROLLER_COLOR}${BOLD}[$CONTROLLER_TITLE]${NC}"
echo -e "  🌍 GAZEBO WORLD            : ${BOLD}$(basename $WORLD_FILE)${NC}"
echo -e "  🌪️  DRYDEN WIND TURBULENCE : ${BOLD}$ENABLE_WIND${NC}"
echo -e "  🚧 RINTANGAN (OBSTACLES)  : ${BOLD}$ENABLE_OBSTACLES${NC} (statis)${ENABLE_DYN_OBSTACLES:+ + dinamis=$ENABLE_DYN_OBSTACLES}"
echo -e "  🗺️  WILAYAH PEMETAAN      : ${BOLD}$REGION${NC}"
echo -e "  🛡️  PENGHINDARAN          : ${BOLD}CBF-QP${NC}"
echo "  💡 Terminal 2 Fault Injection: ./kill_drone.sh <id...> (Contoh: ./kill_drone.sh 4)"
echo "========================================================================="

# Fungsi Cleanup saat user menekan Ctrl+C
cleanup() {
    echo ""
    echo "🛑 Menghentikan seluruh proses Gazebo, RViz2, dan ROS 2 nodes..."
    killall -9 gz-sim-main parameter_bridge 2>/dev/null || true
    pkill -9 -f "gz sim" 2>/dev/null || true
    pkill -9 -f "spawn_drones" 2>/dev/null || true
    pkill -9 -f "pid_lqr_node" 2>/dev/null || true
    pkill -9 -f "pid_hinf_node" 2>/dev/null || true
    pkill -9 -f "dryden_wind_node" 2>/dev/null || true
    pkill -9 -f "ros_gz_bridge" 2>/dev/null || true
    pkill -9 -f "rviz2" 2>/dev/null || true
    pkill -9 -f "test_7drone_voronoi_mapping" 2>/dev/null || true
    echo "✅ Semua proses berhasil dibersihkan."
    exit 0
}

trap cleanup SIGINT SIGTERM EXIT

# 1. Bersihkan proses lama
echo "🧹 Membersihkan proses lama..."
killall -9 gz-sim-main parameter_bridge 2>/dev/null || true
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f "spawn_drones" 2>/dev/null || true
pkill -9 -f "pid_lqr_node" 2>/dev/null || true
pkill -9 -f "pid_hinf_node" 2>/dev/null || true
pkill -9 -f "dryden_wind_node" 2>/dev/null || true
pkill -9 -f "ros_gz_bridge" 2>/dev/null || true
pkill -9 -f "rviz2" 2>/dev/null || true
pkill -9 -f "test_7drone_voronoi_mapping" 2>/dev/null || true
sleep 1

# 2. Jalankan Gazebo Simulator GUI dengan custom gui config
GUI_CONFIG="$(ros2 pkg prefix swarm_sim 2>/dev/null)/share/swarm_sim/config/gazebo_gui.config"
if [ ! -f "$GUI_CONFIG" ]; then
    GUI_CONFIG="/home/izmaherdian/Documents/swarm-quadrotor-mapping/swarm_ws/src/swarm_sim/config/gazebo_gui.config"
fi

if [ "$HEADLESS" = true ]; then
    echo "1️⃣  Menjalankan Gazebo Simulator (headless)..."
    gz sim -s -r "$WORLD_FILE" > /dev/null 2>&1 &
else
    echo "1️⃣  Menjalankan Gazebo Simulator GUI..."
    gz sim -r --gui-config "$GUI_CONFIG" "$WORLD_FILE" &
fi
GZ_PID=$!

echo "   ⏳ Menunggu Gazebo Engine siap..."
for t in $(seq 1 30); do
    if gz topic -l 2>/dev/null | grep -q "/world/swarm_world/clock"; then
        echo "   ✅ Gazebo Engine siap!"
        break
    fi
    sleep 0.5
done

# 3. Spawn 7 Drone Quadrotor & Controller Terpilih (+ Dryden Wind jika aktif)
echo -e "2️⃣  Melakukan Spawn 7 Drone & Flight Controller ${CONTROLLER_COLOR}${BOLD}[$CONTROLLER]${NC}..."
ros2 launch swarm_sim spawn_drones_launch.py \
    num_drones:=7 \
    controller:="$CONTROLLER" \
    use_mid_level:=false \
    enable_wind:="$ENABLE_WIND" \
    spawn_x1:="-3.0" spawn_y1:="-18.0" \
    spawn_x2:="-1.0" spawn_y2:="-18.0" \
    spawn_x3:="1.0"  spawn_y3:="-18.0" \
    spawn_x4:="3.0"  spawn_y4:="-18.0" \
    spawn_x5:="-2.0" spawn_y5:="-16.5" \
    spawn_x6:="0.0"  spawn_y6:="-16.5" \
    spawn_x7:="2.0"  spawn_y7:="-16.5" \
    results_base:="${RESULTS_DIR:-multi_agent}" &

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
if [ -f "$RVIZ_CONFIG" ] && [ "$HEADLESS" != true ]; then
    echo "3️⃣  Membuka RViz2 Visualizer..."
    rviz2 -d "$RVIZ_CONFIG" --ros-args -p use_sim_time:=true &
fi
sleep 2

# 5. Jalankan Node Pemetaan Voronoi & Boustrophedon dengan Parameter Skema
echo "4️⃣  Menjalankan Algoritma Pemetaan Voronoi Swarm..."
echo "========================================================================="
# Parameter DOUBLE harus dikirim dengan titik desimal: rclpy menolak "12"
# sebagai INTEGER saat deklarasinya DOUBLE.
SWEEP_ARG=()
if [ -n "$SWEEP_SPEED" ]; then
    SWEEP_ARG+=(-p sweep_speed:="$(printf '%.4f' "$SWEEP_SPEED")")
fi
if [ -n "$EXIT_AFTER" ]; then
    SWEEP_ARG+=(-p exit_after_success:="$(printf '%.1f' "$EXIT_AFTER")")
fi

python3 "$WS_DIR/experiments/test_7drone_voronoi_mapping.py" \
    --ros-args \
    -p use_sim_time:=true \
    -p scheme:="$SCHEME" \
    -p enable_wind:="$ENABLE_WIND" \
    -p enable_obstacles:="$ENABLE_OBSTACLES" \
    -p enable_dynamic_obstacles:="$ENABLE_DYN_OBSTACLES" \
    -p region:="$REGION" \
    ${SWEEP_ARG[@]+"${SWEEP_ARG[@]}"}
