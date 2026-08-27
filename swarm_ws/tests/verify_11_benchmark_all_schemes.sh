#!/usr/bin/env bash
# ==============================================================================
#   TEST 11: AUTOMATED BENCHMARK — 4 MAPPING SCHEMES (PID-LQR vs PID-H-INFINITY)
# ==============================================================================
#   Skema 1: Nominal Voronoi Mapping (Zero Disturbance)
#   Skema 2: Dryden Wind Turbulence (σ=2.5N, τ=0.5s + Gust Step)
#   Skema 3: Obstacle Avoidance (9 Static Cylinders + 2 Dynamic 'X' Crossing)
#   Skema 4: Combined Dryden Wind & Obstacles Disturbance Mapping
# ==============================================================================
set +m



RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
MAGENTA='\033[0;35m'
BOLD='\033[1m'
NC='\033[0m'

TESTS_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$(cd "$TESTS_DIR/.." && pwd)"

# Setup ROS 2 Environment
source /opt/ros/lyrical/setup.bash 2>/dev/null || source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null || true
source "$WS_DIR/install/setup.bash"

export GZ_SIM_SYSTEM_PLUGIN_PATH="/opt/ros/lyrical/opt/gz_sim_vendor/lib/gz-sim-10/plugins:$GZ_SIM_SYSTEM_PLUGIN_PATH"
export LD_LIBRARY_PATH="/opt/ros/lyrical/opt/gz_sim_vendor/lib:/opt/ros/lyrical/opt/gz_sim_vendor/lib/gz-sim-10/plugins:$WS_DIR/install/swarm_sim/lib:$LD_LIBRARY_PATH"
export GZ_SIM_RESOURCE_PATH="$WS_DIR/src/swarm_sim/models"

RESULTS_BASE="$WS_DIR/results/benchmark"
mkdir -p "$RESULTS_BASE"

# Opsi Default
RUN_SCHEME="all"
DURATION_SEC=45

# Parsing Argumen
while [[ $# -gt 0 ]]; do
  case $1 in
    --scheme|-s)
      RUN_SCHEME="$2"
      shift 2
      ;;
    --duration|-d)
      DURATION_SEC="$2"
      shift 2
      ;;
    --quick)
      DURATION_SEC=30
      shift
      ;;
    -h|--help)
      echo -e "${BOLD}Penggunaan:${NC}"
      echo "  ./verify_11_benchmark_all_schemes.sh [OPSI]"
      echo ""
      echo "  -s, --scheme <1|2|3|4|all>   Pilih skema pengujian (default: all)"
      echo "  -d, --duration <detik>       Durasi per sesi benchmark (default: 45s)"
      echo "  --quick                      Jalankan benchmark cepat (30s per sesi)"
      exit 0
      ;;
    *)
      shift
      ;;
  esac
done

cleanup() {
    killall -9 gz-sim-main parameter_bridge dryden_wind_node pid_lqr_node pid_hinf_node 2>/dev/null || true
    pkill -9 -f "gz sim" 2>/dev/null || true
    pkill -9 -f "spawn_drones_launch" 2>/dev/null || true
    pkill -9 -f "test_7drone_voronoi_mapping.py" 2>/dev/null || true
    wait 2>/dev/null || true
    sleep 1
}

run_single_benchmark() {
    local SCHEME_ID="$1"
    local CTRL_TYPE="$2"  # "lqr" atau "hinf"
    local SUBDIR="scheme${SCHEME_ID}_${CTRL_TYPE}"
    local LOG_DIR="$RESULTS_BASE/$SUBDIR"
    mkdir -p "$LOG_DIR"

    local CTRL_NODE="pid_lqr_node"
    local CTRL_LABEL="PID-LQR (Baseline)"
    if [ "$CTRL_TYPE" == "hinf" ]; then
        CTRL_NODE="pid_hinf_node"
        CTRL_LABEL="PID-H-Infinity (Robust)"
    fi

    local WORLD_FILE="$WS_DIR/src/swarm_sim/worlds/empty.world"
    local ENABLE_WIND="false"
    local ENABLE_OBS="false"

    if [ "$SCHEME_ID" -eq 2 ]; then
        ENABLE_WIND="true"
    elif [ "$SCHEME_ID" -eq 3 ]; then
        WORLD_FILE="$WS_DIR/src/swarm_sim/worlds/obstacles.world"
        ENABLE_OBS="true"
    elif [ "$SCHEME_ID" -eq 4 ]; then
        WORLD_FILE="$WS_DIR/src/swarm_sim/worlds/obstacles.world"
        ENABLE_WIND="true"
        ENABLE_OBS="true"
    fi

    echo -e "  ▶️  Menjalankan [Skema $SCHEME_ID | $CTRL_LABEL] (Durasi: ${DURATION_SEC}s)..."
    cleanup
    sleep 1

    # 1. Start Headless Gazebo
    gz sim -s -r "$WORLD_FILE" > /dev/null 2>&1 &
    local GZ_PID=$!
    sleep 3

    # 2. Spawn 7 Drones with Target Controller and Log Directory
    ros2 launch swarm_sim spawn_drones_launch.py \
        num_drones:=7 \
        controller:="$CTRL_NODE" \
        use_mid_level:=false \
        enable_wind:="$ENABLE_WIND" \
        results_base:="$LOG_DIR" \
        spawn_x1:="-3.0" spawn_y1:="-18.0" \
        spawn_x2:="-1.0" spawn_y2:="-18.0" \
        spawn_x3:="1.0"  spawn_y3:="-18.0" \
        spawn_x4:="3.0"  spawn_y4:="-18.0" \
        spawn_x5:="-2.0" spawn_y5:="-16.5" \
        spawn_x6:="0.0"  spawn_y6:="-16.5" \
        spawn_x7:="2.0"  spawn_y7:="-16.5" > /dev/null 2>&1 &
    
    # Wait for drones online
    for t in $(seq 1 25); do
        TOPIC_COUNT=$(ros2 topic list 2>/dev/null | grep -E "/iris_[1-7]/odometry" | wc -l || true)
        if [ "$TOPIC_COUNT" -ge 7 ]; then
            break
        fi
        sleep 0.8
    done

    # 3. Run Voronoi Mapping Node in Background
    python3 "$WS_DIR/experiments/test_7drone_voronoi_mapping.py" \
        --ros-args \
        -p use_sim_time:=true \
        -p scheme:="$SCHEME_ID" \
        -p enable_wind:="$ENABLE_WIND" \
        -p enable_obstacles:="$ENABLE_OBS" > "$LOG_DIR/coordinator.log" 2>&1 &
    local NODE_PID=$!

    # Wait for target duration
    sleep "$DURATION_SEC"

    # Cleanup this session
    cleanup
    echo -e "     ${GREEN}✅ Selesai: Log tersimpan di $SUBDIR/${NC}"
    sleep 2
}

echo "===================================================================================================="
echo -e "  🧪 ${BOLD}[TEST 11] BENCHMARK OTOMATIS: 4 SKEMA PEMETAAN SWARM 7-DRONE${NC}"
echo "  Komparasi Head-to-Head: PID-LQR vs PID-H-Infinity (Tracking, Robustness, Clearance, & Energy)"
echo "===================================================================================================="
echo ""

SCHEMES_TO_RUN=()
if [ "$RUN_SCHEME" == "all" ]; then
    SCHEMES_TO_RUN=(1 2 3 4)
else
    SCHEMES_TO_RUN=("$RUN_SCHEME")
fi

for s in "${SCHEMES_TO_RUN[@]}"; do
    echo "────────────────────────────────────────────────────────────────────────────────────────────────────"
    echo -e "  📋 ${YELLOW}${BOLD}SKEMA $s${NC}"
    echo "────────────────────────────────────────────────────────────────────────────────────────────────────"
    run_single_benchmark "$s" "lqr"
    run_single_benchmark "$s" "hinf"
    echo ""
done

# Generate Visual Plots & Analysis
echo "===================================================================================================="
echo "  🎨 Menghasilkan Visualisasi Resolusi Tinggi & Analisis Metrik Komparasi..."
echo "===================================================================================================="
python3 "$WS_DIR/experiments/plot_benchmark_schemes.py" "$RESULTS_BASE" "$RESULTS_BASE/figures"

echo ""
echo -e "${GREEN}${BOLD}🏆 SELURUH PENGUJIAN BENCHMARK 4 SKEMA SELESAI DENGAN SUKSES!${NC}"
echo -e "📁 Output Visualisasi: $RESULTS_BASE/figures/"
echo "===================================================================================================="
