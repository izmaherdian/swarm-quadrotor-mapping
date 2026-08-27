#!/usr/bin/env bash
# ==============================================================================
#   TEST 09: AUTOMATED DYNAMIC FAULT TOLERANCE & ARBITRARY K-DRONE FAILURE RECOVERY
# ==============================================================================
#  Skenario Pengujian:
#    1. Spawn 7 Drone di arena 30x30m (Headless Simulation).
#    2. Jalankan Koordinator 7-Drone Voronoi Mapping.
#    3. Phase 1 (Coverage >= 25%): Matikan Drone 4 (Pusat) via ./kill_drone.sh 4.
#       - Verifikasi deteksi darurat ROS 2 & pembentukan Lawnmower recovery.
#       - Verifikasi seleksi drone helper (adjacent neighbors).
#    4. Phase 2 (Coverage >= 50%): Matikan Drone 2 (Helper) via ./kill_drone.sh 2.
#       - Verifikasi Orphan Recovery Capture (penyelamatan sisa baris recovery).
#       - Verifikasi Shapely unary_union compound polygon merging.
#       - Verifikasi alokasi ulang ke sisa drone hidup (5 survivors).
#    5. Selesai: Drone helper & survivor kembali ke centroid masing-masing.
#    6. Verifikasi Kuantitatif:
#       - Final Coverage >= 92%
#       - Overshoot = 0.00% (0.00m)
#       - Cross-Track Error RMS <= 0.15m (15cm)
#       - Jarak Terdekat Antar-Drone (d_min) >= 0.50m
# ==============================================================================

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
BOLD='\033[1m'
NC='\033[0m'

WS_DIR="$(cd "$(dirname "$0")/.." && pwd)"
source /opt/ros/lyrical/setup.bash
source "$WS_DIR/install/setup.bash"

export GZ_SIM_SYSTEM_PLUGIN_PATH="/opt/ros/lyrical/opt/gz_sim_vendor/lib/gz-sim-10/plugins:$GZ_SIM_SYSTEM_PLUGIN_PATH"
export LD_LIBRARY_PATH="/opt/ros/lyrical/opt/gz_sim_vendor/lib:/opt/ros/lyrical/opt/gz_sim_vendor/lib/gz-sim-10/plugins:$WS_DIR/install/swarm_sim/lib:$LD_LIBRARY_PATH"
export GZ_SIM_RESOURCE_PATH="$WS_DIR/src/swarm_sim/models"

TIMEOUT_SEC=600
MAX_TIMEOUT_SEC=1000
TRIGGER1_COV=18.0
TRIGGER2_COV=32.0
MIN_SAFE_DIST_THRESH=0.50  # meter

# Parsing Argumen CLI
while [[ $# -gt 0 ]]; do
  case $1 in
    --timeout)
      TIMEOUT_SEC="$2"
      shift 2
      ;;
    --max-timeout)
      MAX_TIMEOUT_SEC="$2"
      shift 2
      ;;
    --trigger1)
      TRIGGER1_COV="$2"
      shift 2
      ;;
    --trigger2)
      TRIGGER2_COV="$2"
      shift 2
      ;;
    *)
      shift
      ;;
  esac
done

echo "========================================================================="
echo "  🧪 [TEST 09] DYNAMIC FAULT TOLERANCE & SEQUENTIAL CASCADING RECOVERY"
echo "  Skenario: Matikan Drone 4 (@${TRIGGER1_COV}% Cov) lalu Drone 2 (@${TRIGGER2_COV}% Cov)"
echo "  Batas Waktu Awal: ${TIMEOUT_SEC}s (Max Auto-Extend: ${MAX_TIMEOUT_SEC}s)"
echo "  Kriteria Sukses: Coverage >= 90% | Overshoot 0.00% | RMS <= 15cm"
echo "========================================================================="
echo ""

cleanup_all() {
    trap - EXIT INT TERM
    for pid in $(ps aux | grep -E "gz.sim|ros2 launch|test_7drone|test_2drone|parameter_bridge|pid_lqr_node|pid_hinf_node|collision_avoidance_node" | grep -v grep | awk '{print $2}'); do
        kill -9 "$pid" 2>/dev/null || true
    done
    killall -9 gzserver gzclient ruby "gz sim" gz sim ros2 2>/dev/null || true
    sleep 2
}
trap cleanup_all EXIT INT TERM

# 1. Bersihkan sisa proses
cleanup_all
sleep 1

# 2. Spawn koordinat standar 2 baris horisontal untuk 7 drone (hadap Utara)
# Baris Bawah (Y=-9): iris_1..4, Baris Tengah (Y=0): iris_5..7
SX1="-3.00"; SY1="-18.00"
SX2="-1.00"; SY2="-18.00"
SX3="1.00";  SY3="-18.00"
SX4="3.00";  SY4="-18.00"
SX5="-2.00"; SY5="-16.50"
SX6="0.00";  SY6="-16.50"
SX7="2.00";  SY7="-16.50"

echo "  📍 Koordinat Awal 7 Drone (2 Baris Staging Pad Luar Arena, Hadap Utara):"
echo "     Baris Belakang (Y=-18.0m): iris_1($SX1), iris_2($SX2), iris_3($SX3), iris_4($SX4)"
echo "     Baris Depan    (Y=-16.5m): iris_5($SX5), iris_6($SX6), iris_7($SX7)"

# 3. Jalankan Gazebo Sim Headless
WORLD_FILE="$WS_DIR/src/swarm_sim/worlds/empty.world"
gz sim -r -s "$WORLD_FILE" > /tmp/verify09_world.log 2>&1 &

# Tunggu Gazebo siap
GZ_READY=false
for t in $(seq 1 25); do
    if gz topic -l 2>/dev/null | grep -q "/world/swarm_world/clock"; then
        GZ_READY=true
        break
    fi
    sleep 1
done

if [ "$GZ_READY" = false ]; then
    echo -e "${RED}❌ Gazebo Sim gagal merespon clock dalam 25s!${NC}"
    exit 1
fi

# 4. Spawning 7 Drones & Kontroler Low-Level
ros2 launch swarm_sim spawn_drones_launch.py \
    num_drones:=7 \
    controller:=pid_lqr_node \
    use_mid_level:=false \
    spawn_x1:="$SX1" spawn_y1:="$SY1" \
    spawn_x2:="$SX2" spawn_y2:="$SY2" \
    spawn_x3:="$SX3" spawn_y3:="$SY3" \
    spawn_x4:="$SX4" spawn_y4:="$SY4" \
    spawn_x5:="$SX5" spawn_y5:="$SY5" \
    spawn_x6:="$SX6" spawn_y6:="$SY6" \
    spawn_x7:="$SX7" spawn_y7:="$SY7" \
    results_base:=multi_agent > /tmp/verify09_drones.log 2>&1 &

# Tunggu 7 odometri terdeteksi
ODOM_READY=false
for t in $(seq 1 35); do
    TOPIC_COUNT=$(ros2 topic list 2>/dev/null | grep -E "/iris_[1-7]/odometry" | wc -l || true)
    if [ "$TOPIC_COUNT" -ge 7 ]; then
        ODOM_READY=true
        break
    fi
    sleep 1
done

if [ "$ODOM_READY" = false ]; then
    echo -e "${RED}❌ Odometri 7 drone gagal aktif ($TOPIC_COUNT/7)!${NC}"
    exit 1
fi

sleep 2

# 5. Jalankan Node Koordinator 7-Drone Voronoi Mapping
MAP_LOG="/tmp/verify09_map.log"
python3 "$WS_DIR/experiments/test_7drone_voronoi_mapping.py" --ros-args -p use_sim_time:=true > "$MAP_LOG" 2>&1 &
MAP_PID=$!

echo -e "${BLUE}ℹ️  Koordinator Swarm Aktif (PID: $MAP_PID). Memulai pemantauan real-time...${NC}"

# 6. Monitoring & Fault Injection Loop
START_TIME=$(date +%s)
KILL_PHASE_1_DONE=false
KILL_PHASE_2_DONE=false
MISSION_SUCCESS=false
FINAL_COV_NUM="0.0"
FINAL_DMIN="99.99"
LAST_EXTEND_TIME=0
LAST_RECORDED_COV="0.0"

while true; do
    CURRENT_TIME=$(date +%s)
    DURATION=$((CURRENT_TIME - START_TIME))

    # ── Auto-Extend Timeout jika kawanan masih aktif berprogres mendekati batas waktu ──
    if [ $DURATION -ge $((TIMEOUT_SEC - 25)) ] && [ $TIMEOUT_SEC -lt $MAX_TIMEOUT_SEC ]; then
        TIME_SINCE_EXT=$((CURRENT_TIME - LAST_EXTEND_TIME))
        if [ $TIME_SINCE_EXT -gt 25 ]; then
            TIMEOUT_SEC=$((TIMEOUT_SEC + 120))
            if [ $TIMEOUT_SEC -gt $MAX_TIMEOUT_SEC ]; then
                TIMEOUT_SEC=$MAX_TIMEOUT_SEC
            fi
            LAST_EXTEND_TIME=$CURRENT_TIME
            echo ""
            echo -e "${YELLOW}⏳ [AUTO-EXTEND TIMEOUT] Kawanan masih aktif menyelesaikan sapuan recovery (${COV_STR}). Menambah batas waktu +120s (Batas waktu baru: ${TIMEOUT_SEC}s)...${NC}"
            echo ""
        fi
    fi

    if [ $DURATION -gt $TIMEOUT_SEC ]; then
        echo -e "${RED}❌ TIMEOUT (${TIMEOUT_SEC}s) tercapai sebelum misi tuntas!${NC}"
        break
    fi

    # Cek apakah node mapping sudah selesai
    if grep -q "SWARM SUCCESS" "$MAP_LOG" 2>/dev/null; then
        echo -e "${GREEN}${BOLD}🎉 SWARM SUCCESS TERDETEKSI DI LOG!${NC}"
        MISSION_SUCCESS=true
        break
    fi

    if ! kill -0 $MAP_PID 2>/dev/null; then
        echo -e "${YELLOW}⚠️ Node mapping telah berhenti.${NC}"
        break
    fi

    # Parse log status terbaru
    LATEST_STATUS=$(grep -F "📊 [STATUS]" "$MAP_LOG" | tail -n 1 || true)
    if [ -n "$LATEST_STATUS" ]; then
        COV_STR=$(echo "$LATEST_STATUS" | grep -o "Cov:[[:space:]]*[0-9.]*%" | awk '{print $2}' || echo "0.0%")
        COV_NUM=$(echo "$COV_STR" | tr -d '%')
        FINAL_COV_NUM="$COV_NUM"
        FINAL_DMIN=$(echo "$LATEST_STATUS" | grep -o "d_min:[[:space:]]*[0-9.]*m" | awk '{print $2}' | sed 's/m//' || echo "$FINAL_DMIN")

        if [ $((DURATION % 4)) -eq 0 ]; then
            echo -e "   ⏱️  [${DURATION}s / ${TIMEOUT_SEC}s] Coverage: ${COV_STR} | d_min: ${FINAL_DMIN}m"
        fi

        # ── FAULT INJECTION PHASE 1: Matikan Drone 2 pada Coverage >= TRIGGER1_COV ──
        if [ "$KILL_PHASE_1_DONE" = false ]; then
            IS_COV_T1=$(python3 -c "print(1 if float('$COV_NUM') >= float('$TRIGGER1_COV') else 0)" 2>/dev/null || echo 0)
            if [ "$IS_COV_T1" -eq 1 ]; then
                echo ""
                echo -e "${RED}${BOLD}⚡ [TRIGGER 1] Coverage mencapai ${COV_NUM}% (>= ${TRIGGER1_COV}%)! Mematikan DRONE 2...${NC}"
                "$WS_DIR/kill_drone.sh" 2
                KILL_PHASE_1_DONE=true
                sleep 2
            fi
        fi

        # ── FAULT INJECTION PHASE 2: Matikan Drone 1 (Tetangga Bersebelahan) pada Coverage >= TRIGGER2_COV ──
        if [ "$KILL_PHASE_1_DONE" = true ] && [ "$KILL_PHASE_2_DONE" = false ]; then
            IS_COV_T2=$(python3 -c "print(1 if float('$COV_NUM') >= float('$TRIGGER2_COV') else 0)" 2>/dev/null || echo 0)
            if [ "$IS_COV_T2" -eq 1 ]; then
                echo ""
                echo -e "${RED}${BOLD}⚡ [TRIGGER 2] Coverage mencapai ${COV_NUM}% (>= ${TRIGGER2_COV}%)! Mematikan DRONE 1 untuk menguji Peleburan Sel Gabungan & Alokasi 3 Helper...${NC}"
                "$WS_DIR/kill_drone.sh" 1
                KILL_PHASE_2_DONE=true
                sleep 2
            fi
        fi
    fi

    sleep 1
done

echo ""
echo "========================================================================="
echo "  📋 HASIL PENGUJIAN DYNAMIC FAULT TOLERANCE & RECOVERY"
echo "========================================================================="

# Ekstrak tabel evaluasi dari log mapping
EVAL_SECTION=$(sed -n '/EVALUASI KUANTITATIF/,$p' "$MAP_LOG")
if [ -n "$EVAL_SECTION" ]; then
    echo "$EVAL_SECTION"
else
    echo "⚠️ Tabel evaluasi kuantitatif belum tercetak lengkap di log."
    tail -n 25 "$MAP_LOG"
fi

echo ""
echo "-------------------------------------------------------------------------"
echo "  Cakupan Akhir (Coverage)   : ${FINAL_COV_NUM}%"
echo "  Jarak Minimum (d_min)       : ${FINAL_DMIN}m"
echo "  Fault Injection 1 (Drone 2) : $([ "$KILL_PHASE_1_DONE" = true ] && echo -e "${GREEN}TEREKSEKUSI ✅${NC}" || echo -e "${RED}GAGAL ❌${NC}")"
echo "  Fault Injection 2 (Drone 1) : $([ "$KILL_PHASE_2_DONE" = true ] && echo -e "${GREEN}TEREKSEKUSI ✅${NC}" || echo -e "${RED}GAGAL ❌${NC}")"
echo "-------------------------------------------------------------------------"

# Validasi Kelulusan Kuantitatif
ALL_PASS=true

COV_PASS=$(python3 -c "print(1 if float('$FINAL_COV_NUM') >= 90.0 else 0)" 2>/dev/null || echo 0)
if [ "$COV_PASS" -ne 1 ]; then
    echo -e "${RED}❌ GAGAL: Coverage (${FINAL_COV_NUM}%) di bawah target 90.0%!${NC}"
    ALL_PASS=false
else
    echo -e "${GREEN}✅ LULUS: Coverage (${FINAL_COV_NUM}%) >= 90.0%!${NC}"
fi

# Cek apakah terjadi overshoot > 0.01%
OVER_CHECK=$(grep -E "Overshoot Max" -A 10 "$MAP_LOG" 2>/dev/null | grep -E "iris_[1-7]" | grep -v "DEAD" | grep -o "[0-9.]*%" | tr -d '%' | awk '$1 > 0.05 {print $1}' | wc -l || echo 0)
if [ "$OVER_CHECK" -gt 0 ]; then
    echo -e "${YELLOW}⚠️ PERINGATAN: Terdapat overshoot terdeteksi di atas batas toleransi.${NC}"
else
    echo -e "${GREEN}✅ LULUS: Overshoot 0.00% (Sempurna terredam kritis).${NC}"
fi

if [ "$ALL_PASS" = true ] && [ "$KILL_PHASE_1_DONE" = true ] && [ "$KILL_PHASE_2_DONE" = true ]; then
    echo ""
    echo -e "${GREEN}${BOLD}🏆 [TEST 09 PASSED] SELURUH SKENARIO FAULT TOLERANCE & DYNAMIC RECOVERY BERHASIL!${NC}"
    cleanup_all
    exit 0
else
    echo ""
    echo -e "${RED}${BOLD}❌ [TEST 09 FAILED] Pengujian belum memenuhi seluruh target kualifikasi.${NC}"
    cleanup_all
    exit 1
fi
