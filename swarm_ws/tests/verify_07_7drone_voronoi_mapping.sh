#!/usr/bin/env bash
# ==============================================================================
#   TEST 07: MULTI-ITERATION VERIFICATION FOR 7-DRONE 2D VORONOI MAPPING
# ==============================================================================
#  Alur Pengujian Berulang (Monte Carlo Loop):
#    - Menjalankan N iterasi pengujian acak berturut-turut (Default: 3 Runs)
#    - Setiap iterasi:
#        1. Random spawn 7 drone (iris_1 s/d iris_7) di arena [-11, 11]m
#        2. Headless simulation launch (Gazebo 30x30m World)
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
TIMEOUT_SEC=320
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
echo "  🧪 [TEST 07] MULTI-ITERATION 7-DRONE 2D VORONOI MAPPING VERIFICATION"
echo "  Total Iterasi: $TOTAL_RUNS Runs | Timeout per Run: ${TIMEOUT_SEC}s"
echo "  Ambang Batas Jarak Aman: d_min >= ${MIN_SAFE_DIST_THRESH}m | Arena 30x30m (900 m^2)"
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

    # 2. Generate koordinat acak 7 drone
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
    read -r SX1 SY1 SX2 SY2 SX3 SY3 SX4 SY4 SX5 SY5 SX6 SY6 SX7 SY7 <<< "$RAND_COORDS"

    echo "  📍 Koordinat Spawn Acak 7 Drone (d_min awal >= 1.50m):"
    echo "     iris_1: ($SX1, $SY1) | iris_2: ($SX2, $SY2) | iris_3: ($SX3, $SY3)"
    echo "     iris_4: ($SX4, $SY4) | iris_5: ($SX5, $SY5) | iris_6: ($SX6, $SY6)"
    echo "     iris_7: ($SX7, $SY7)"

    # 3. Jalankan Gazebo Sim Headless
    WORLD_FILE="$WS_DIR/src/swarm_sim/worlds/empty.world"
    gz sim -r -s "$WORLD_FILE" > /tmp/verify07_world.log 2>&1 &

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
        echo "❌ [Run $RUN_IDX] Gazebo Sim gagal merespon clock dalam 25s!"
        RESULTS_STATUS+=("FAIL (Gazebo Timeout)")
        RESULTS_DMIN+=("N/A")
        RESULTS_DURATION+=("0s")
        RESULTS_COVERAGE+=("0.0%")
        cleanup_all
        continue
    fi

    # 4. Spawning 7 Drones & Kontroler Low-Level
    ros2 launch swarm_sim spawn_drones_launch.py \
        num_drones:=7 \
        controller:=pid_lqr_node \
        spawn_x1:="$SX1" spawn_y1:="$SY1" \
        spawn_x2:="$SX2" spawn_y2:="$SY2" \
        spawn_x3:="$SX3" spawn_y3:="$SY3" \
        spawn_x4:="$SX4" spawn_y4:="$SY4" \
        spawn_x5:="$SX5" spawn_y5:="$SY5" \
        spawn_x6:="$SX6" spawn_y6:="$SY6" \
        spawn_x7:="$SX7" spawn_y7:="$SY7" \
        results_base:=multi_agent > /tmp/verify07_drones.log 2>&1 &

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
        echo "❌ [Run $RUN_IDX] Odometri 7 drone gagal aktif ($TOPIC_COUNT/7)!"
        RESULTS_STATUS+=("FAIL (Odom Missing)")
        RESULTS_DMIN+=("N/A")
        RESULTS_DURATION+=("0s")
        RESULTS_COVERAGE+=("0.0%")
        cleanup_all
        continue
    fi

    sleep 2

    # 5. Jalankan Node Koordinator 7-Drone Voronoi Mapping
    MAP_LOG="/tmp/verify07_map_run${RUN_IDX}.log"
    python3 "$WS_DIR/experiments/test_7drone_voronoi_mapping.py" > "$MAP_LOG" 2>&1 &
    MAP_PID=$!

    # 6. Monitoring Loop Telemetri Real-Time
    START_TIME=$(date +%s)
    RUN_SUCCESS=false
    FINAL_DMIN="99.99"
    FINAL_COV="0.0"
    DURATION=0

    while true; do
        CURRENT_TIME=$(date +%s)
        DURATION=$((CURRENT_TIME - START_TIME))

        # Cek apakah node mapping sudah selesai atau mati
        if ! kill -0 $MAP_PID 2>/dev/null; then
            break
        fi

        # Parse log telemetri terbaru
        LATEST_STATUS=$(grep -F "📊 [STATUS]" "$MAP_LOG" | tail -n 1 || true)
        if [ -n "$LATEST_STATUS" ]; then
            FINAL_COV=$(echo "$LATEST_STATUS" | grep -o "Cov:[[:space:]]*[0-9.]*%" | awk '{print $2}' || echo "$FINAL_COV")
            FINAL_DMIN=$(echo "$LATEST_STATUS" | grep -o "d_min:[[:space:]]*[0-9.]*m" | awk '{print $2}' | sed 's/m//' || echo "$FINAL_DMIN")

            # Cetak telemetri setiap 5 detik
            if [ $((DURATION % 5)) -eq 0 ]; then
                echo "   ⏱️  [${DURATION}s] Coverage: ${FINAL_COV} | Jarak Terdekat (d_min): ${FINAL_DMIN}m"
            fi

            # Evaluasi ketercapaian cakupan (>= 97.0% atau SWARM SUCCESS)
            COV_NUM=$(echo "$FINAL_COV" | sed 's/%//')
            if grep -q "SWARM SUCCESS" "$MAP_LOG" || (( $(echo "$COV_NUM >= 97.0" | bc -l) )); then
                echo "   🎉 [Run $RUN_IDX] Target Pemetaan Tuntas (Coverage: ${FINAL_COV}) pada detik ke-${DURATION}!"
                RUN_SUCCESS=true
                break
            fi
        fi

        # Cek Timeout
        if [ "$DURATION" -ge "$TIMEOUT_SEC" ]; then
            echo "   ⚠️  [Run $RUN_IDX] Mencapai batas timeout (${TIMEOUT_SEC}s)!"
            break
        fi

        sleep 0.5
    done

    # Hentikan node mapping
    kill -9 $MAP_PID 2>/dev/null || true

    # Evaluasi hasil run
    COV_NUM=$(echo "$FINAL_COV" | sed 's/%//')
    DMIN_NUM=$(echo "$FINAL_DMIN" | sed 's/m//')

    if [ -z "$DMIN_NUM" ] || [ "$DMIN_NUM" = "99.99" ]; then
        DMIN_NUM="0.00"
    fi

    IS_SAFE=$(echo "$DMIN_NUM >= $MIN_SAFE_DIST_THRESH" | bc -l 2>/dev/null || echo 0)
    IS_COV_OK=$(echo "$COV_NUM >= 97.0" | bc -l 2>/dev/null || echo 0)

    if [ "$IS_SAFE" -eq 1 ] && [ "$IS_COV_OK" -eq 1 ]; then
        RESULTS_STATUS+=("PASS ✅")
        PASSED_COUNT=$((PASSED_COUNT + 1))
        echo "  🟢 [HASIL RUN $RUN_IDX]: LULUS (PASS) | Cov: ${FINAL_COV} | d_min: ${DMIN_NUM}m | Waktu: ${DURATION}s"
    else
        RESULTS_STATUS+=("FAIL ❌")
        echo "  🔴 [HASIL RUN $RUN_IDX]: GAGAL (FAIL) | Cov: ${FINAL_COV} | d_min: ${DMIN_NUM}m | Waktu: ${DURATION}s"
    fi

    RESULTS_DMIN+=("${DMIN_NUM}m")
    RESULTS_DURATION+=("${DURATION}s")
    RESULTS_COVERAGE+=("${FINAL_COV}")

    # Bersihkan Gazebo world untuk iterasi berikutnya
    cleanup_all
    sleep 2
done

# ═════════════════════════════════════════════════════════════════════════════
#  REKAPITULASI STATISTIK AKHIR PENGUJIAN
# ═════════════════════════════════════════════════════════════════════════════
echo ""
echo "========================================================================="
echo "  📊 TABEL REKAPITULASI PENGUJIAN 7-DRONE VORONOI MAPPING ($TOTAL_RUNS RUNS)"
echo "========================================================================="
printf "| %-6s | %-12s | %-12s | %-12s | %-10s |\n" "Run #" "Status" "d_min (m)" "Coverage" "Waktu"
echo "|--------|--------------|--------------|--------------|------------|"

for idx in $(seq 0 $((TOTAL_RUNS - 1))); do
    RUN_NUM=$((idx + 1))
    printf "| Run %-2d | %-12s | %-12s | %-12s | %-10s |\n" \
        "$RUN_NUM" "${RESULTS_STATUS[$idx]}" "${RESULTS_DMIN[$idx]}" "${RESULTS_COVERAGE[$idx]}" "${RESULTS_DURATION[$idx]}"
done
echo "========================================================================="

SUCCESS_RATE=$(python3 -c "print(f'{$PASSED_COUNT / $TOTAL_RUNS * 100.0:.1f}')")
echo "  🎯 Tingkat Keberhasilan Swarm (Success Rate): ${SUCCESS_RATE}% ($PASSED_COUNT / $TOTAL_RUNS Runs)"

if [ "$PASSED_COUNT" -eq "$TOTAL_RUNS" ]; then
    echo "  🏆 KESIMPULAN: SELURUH STANDAR KUALITAS TERPENUHI SEMPURNA (100% PASS)!"
    echo "========================================================================="
    exit 0
else
    echo "  ⚠️ KESIMPULAN: Terdapat iterasi yang belum memenuhi standar kelulusan."
    echo "========================================================================="
    exit 1
fi
