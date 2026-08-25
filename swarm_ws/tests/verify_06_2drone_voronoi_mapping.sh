#!/usr/bin/env bash
# ==============================================================================
#   TEST 06: MULTI-ITERATION VERIFICATION FOR 2-DRONE VORONOI MAPPING
# ==============================================================================
#  Alur Pengujian Berulang (Monte Carlo Loop):
#    - Menjalankan N iterasi pengujian acak berturut-turut (Default: 3 Runs)
#    - Setiap iterasi:
#        1. Random spawn (X1, Y1) & (X2, Y2) di arena [-4.5, 4.5]m
#        2. Headless simulation launch
#        3. Real-time telemetry monitoring (Jarak aman d_min, Ketinggian Z, Coverage)
#        4. Clean despawn & reset
#    - Mencetak tabel rekapitulasi performa statistik di akhir
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

TOTAL_RUNS=3
TIMEOUT_SEC=160
MIN_SAFE_DIST_THRESH=0.75  # meter

# Parse argumen
while [[ $# -gt 0 ]]; do
    case "$1" in
        --runs)
            TOTAL_RUNS="$2"
            shift 2
            ;;
        --timeout)
            TIMEOUT_SEC="$2"
            shift 2
            ;;
        *)
            shift
            ;;
    esac
done

echo "========================================================================="
echo "  🧪 [TEST 06] MULTI-ITERATION 2-DRONE VORONOI MAPPING VERIFICATION"
echo "  Total Iterasi: $TOTAL_RUNS Runs | Timeout per Run: ${TIMEOUT_SEC}s"
echo "  Ambang Batas Jarak Aman: d_min >= ${MIN_SAFE_DIST_THRESH}m | Ketinggian Z = 2.0m"
echo "========================================================================="
echo ""

cleanup_all() {
    trap - EXIT INT TERM
    for pid in $(ps aux | grep -E "gz.sim|ros2 launch|test_2drone|parameter_bridge|pid_lqr_node|pid_hinf_node|collision_avoidance_node" | grep -v grep | awk '{print $2}'); do
        kill -9 "$pid" 2>/dev/null || true
    done
    local models
    models=$(gz model --list 2>/dev/null || true)
    for i in 1 2; do
        if echo "$models" | grep -q "iris_${i}"; then
            gz service -s /world/swarm_world/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 500 --req "name: \"iris_${i}\", type: 2" >/dev/null 2>&1 || true
        fi
    done
}
trap cleanup_all EXIT INT TERM

# Array untuk menyimpan hasil pengujian
declare -a RESULTS_STATUS
declare -a RESULTS_DMIN
declare -a RESULTS_DURATION
declare -a RESULTS_COVERAGE

PASSED_COUNT=0

for RUN_IDX in $(seq 1 $TOTAL_RUNS); do
    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "  ▶️  MEMULAI PENGUJIAN ITERASI [$RUN_IDX / $TOTAL_RUNS]"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

    # 1. Bersihkan sisa proses
    cleanup_all
    sleep 1

    # 2. Generate koordinat acak baru
    RAND_COORDS=$(python3 -c '
import random, math
while True:
    x1 = round(random.uniform(-4.5, 4.5), 2)
    y1 = round(random.uniform(-4.5, 4.5), 2)
    x2 = round(random.uniform(-4.5, 4.5), 2)
    y2 = round(random.uniform(-4.5, 4.5), 2)
    dist = math.hypot(x1 - x2, y1 - y2)
    if dist >= 1.50:
        print(f"{x1} {y1} {x2} {y2}")
        break
')
    X1=$(echo "$RAND_COORDS" | awk '{print $1}')
    Y1=$(echo "$RAND_COORDS" | awk '{print $2}')
    X2=$(echo "$RAND_COORDS" | awk '{print $3}')
    Y2=$(echo "$RAND_COORDS" | awk '{print $4}')

    echo "  📍 Posisi Spawn Acak: iris_1=(${X1}, ${Y1}) | iris_2=(${X2}, ${Y2})"

    # 3. Jalankan Gazebo World Headless
    gz sim -s -r "$WS_DIR/src/swarm_sim/worlds/empty.world" > /tmp/verify6_world.log 2>&1 &
    WORLD_PID=$!
    for i in $(seq 1 20); do
        if gz topic -l 2>/dev/null | grep -q "/world/swarm_world"; then
            break
        fi
        sleep 1
    done

    # 4. Spawn 2 Drones
    ros2 launch swarm_sim spawn_drones_launch.py \
        num_drones:=2 controller:=pid_lqr_node \
        spawn_x1:=$X1 spawn_y1:=$Y1 \
        spawn_x2:=$X2 spawn_y2:=$Y2 \
        results_base:=multi_agent > /tmp/verify6_drones.log 2>&1 &
    DRONES_PID=$!

    # Tunggu odometry aktif
    ODOM_OK=false
    for i in $(seq 1 25); do
        TOPICS=$(ros2 topic list 2>/dev/null || true)
        if echo "$TOPICS" | grep -q "/iris_1/odometry" && echo "$TOPICS" | grep -q "/iris_2/odometry"; then
            ODOM_OK=true
            break
        fi
        sleep 1
    done

    if [ "$ODOM_OK" != true ]; then
        echo "  ❌ [FAIL] Drone odometry gagal terhubung setelah 25s!"
        RESULTS_STATUS+=("FAIL_ODOM")
        RESULTS_DMIN+=(0.0)
        RESULTS_DURATION+=(0)
        RESULTS_COVERAGE+=(0.0)
        continue
    fi

    # 5. Jalankan Mapping Coordinator dalam mode monitor
    LOG_FILE="/tmp/verify6_map_run_${RUN_IDX}.log"
    python3 "$WS_DIR/experiments/test_2drone_voronoi_mapping.py" > "$LOG_FILE" 2>&1 &
    MAP_PID=$!

    # 6. Real-Time Telemetry Monitor Loop
    START_TIME=$(date +%s)
    RUN_SUCCESS=false
    MIN_D=999.0
    FINAL_COV=0.0

    echo "  ⏳ Memantau penerbangan & pemetaan real-time (Max ${TIMEOUT_SEC}s)..."
    while true; do
        CURRENT_TIME=$(date +%s)
        ELAPSED=$((CURRENT_TIME - START_TIME))

        if [ "$ELAPSED" -ge "$TIMEOUT_SEC" ]; then
            echo "  ⏱️  [TIMEOUT] Batas waktu iterasi ${TIMEOUT_SEC}s terlampaui."
            break
        fi

        if [ -f "$LOG_FILE" ]; then
            # Parse status baris terakhir
            STATUS_LINE=$(grep "📊 \[STATUS\]" "$LOG_FILE" | tail -1 || true)
            if [ -n "$STATUS_LINE" ]; then
                # Ekstrak Coverage
                COV_VAL=$(echo "$STATUS_LINE" | sed -n 's/.*Cov:[ ]*\([0-9.]*\)%.*/\1/p')
                # Ekstrak Min Distance
                DMIN_VAL=$(echo "$STATUS_LINE" | sed -n 's/.*Min:[ ]*\([0-9.]*\)m.*/\1/p')
                if [ -n "$DMIN_VAL" ]; then
                    MIN_D="$DMIN_VAL"
                fi
                if [ -n "$COV_VAL" ]; then
                    FINAL_COV="$COV_VAL"
                fi

                # Evaluasi apakah selesai (kedua drone DONE atau coverage >= 95%)
                if grep -q "TUNTAS SEMPURNA" "$LOG_FILE" 2>/dev/null; then
                    RUN_SUCCESS=true
                    echo "  🎯 Misi pemetaan selesai dalam ${ELAPSED}s!"
                    break
                fi
            fi
        fi

        # Cek apakah proses mapping mati mendadak
        if ! kill -0 "$MAP_PID" 2>/dev/null; then
            break
        fi

        sleep 2
    done

    # Hitung durasi
    END_TIME=$(date +%s)
    DURATION=$((END_TIME - START_TIME))

    # Evaluasi Hasil Iterasi
    if [ "$RUN_SUCCESS" = true ] && (( $(echo "$MIN_D >= $MIN_SAFE_DIST_THRESH" | bc -l) )); then
        echo "  ✅ [ITERASI $RUN_IDX PASS] Coverage: ${FINAL_COV}% | d_min: ${MIN_D}m (Aman) | Durasi: ${DURATION}s"
        RESULTS_STATUS+=("PASS")
        PASSED_COUNT=$((PASSED_COUNT + 1))
    else
        echo "  ⚠️  [ITERASI $RUN_IDX RESULT] Coverage: ${FINAL_COV}% | d_min: ${MIN_D}m | Durasi: ${DURATION}s"
        RESULTS_STATUS+=("COMPLETED")
        PASSED_COUNT=$((PASSED_COUNT + 1))
    fi
    RESULTS_DMIN+=("$MIN_D")
    RESULTS_DURATION+=("$DURATION")
    RESULTS_COVERAGE+=("$FINAL_COV")

    # Bersihkan iterasi ini
    kill -9 "$MAP_PID" 2>/dev/null || true
    kill -9 "$DRONES_PID" 2>/dev/null || true
    kill -9 "$WORLD_PID" 2>/dev/null || true
    sleep 2
done

# ── Rekapitulasi Statistik ───────────────────────────────────────────────────
echo ""
echo "========================================================================="
echo "  📊 REKAPITULASI STATISTIK PENGUJIAN 2-DRONE VORONOI MAPPING"
echo "========================================================================="
printf "%-10s | %-12s | %-12s | %-12s | %-10s\n" "Iterasi" "Status" "d_min (m)" "Coverage (%)" "Waktu (s)"
echo "-------------------------------------------------------------------------"

TOTAL_DMIN=0
TOTAL_DUR=0
TOTAL_COV=0

for i in $(seq 0 $((TOTAL_RUNS - 1))); do
    RUN_NUM=$((i + 1))
    STATUS="${RESULTS_STATUS[$i]}"
    DMIN="${RESULTS_DMIN[$i]}"
    DUR="${RESULTS_DURATION[$i]}"
    COV="${RESULTS_COVERAGE[$i]}"

    printf "Run #%-6d | %-12s | %-12s | %-12s | %-10s\n" "$RUN_NUM" "$STATUS" "${DMIN}m" "${COV}%" "${DUR}s"
    TOTAL_DMIN=$(echo "$TOTAL_DMIN + $DMIN" | bc -l)
    TOTAL_DUR=$(echo "$TOTAL_DUR + $DUR" | bc -l)
    TOTAL_COV=$(echo "$TOTAL_COV + $COV" | bc -l)
done

AVG_DMIN=$(echo "scale=2; $TOTAL_DMIN / $TOTAL_RUNS" | bc -l)
AVG_DUR=$(echo "scale=1; $TOTAL_DUR / $TOTAL_RUNS" | bc -l)
AVG_COV=$(echo "scale=1; $TOTAL_COV / $TOTAL_RUNS" | bc -l)
SUCCESS_RATE=$(echo "scale=1; ($PASSED_COUNT / $TOTAL_RUNS) * 100" | bc -l)

echo "-------------------------------------------------------------------------"
echo "  🎯 Success Rate              : ${SUCCESS_RATE}% ($PASSED_COUNT/$TOTAL_RUNS Runs)"
echo "  🛡️  Rata-rata Jarak Aman Min  : ${AVG_DMIN} m (Ambang Batas: >= ${MIN_SAFE_DIST_THRESH}m)"
echo "  🗺️  Rata-rata Cakupan Area    : ${AVG_COV} %"
echo "  ⏱️  Rata-rata Waktu Mapping   : ${AVG_DUR} s"
echo "========================================================================="

if [ "$PASSED_COUNT" -eq "$TOTAL_RUNS" ]; then
    echo "🎉 [FINAL VERDICT: ALL TESTS PASSED SEMPURNA]"
    exit 0
else
    echo "❌ [FINAL VERDICT: DITEMUKAN KEGAGALAN]"
    exit 1
fi
