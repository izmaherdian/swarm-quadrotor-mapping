#!/usr/bin/env bash
# ==============================================================================
#   PERBANDINGAN PID-LQR vs PID-H-INFINITY
# ==============================================================================
# Menjalankan skema yang sama dengan kedua kontroler low-level, memisahkan
# hasilnya ke direktori masing-masing, lalu mencetak tabel kuantitatif.
#
#   ./compare_lqr_hinf.sh                 # Skema 4 (default), 600s tiap kontroler
#   ./compare_lqr_hinf.sh -s 3 -d 900     # skema & durasi lain
#
# Keluar dengan status bukan-nol bila ada tabrakan pada run mana pun.
# ==============================================================================
set -u

WS_DIR="$(cd "$(dirname "$0")" && pwd)"
SCHEME=4
DURATION=600
OUT_ROOT="$WS_DIR/results/compare"
REGION="rect"
SWEEP_SPEED=""

while [[ $# -gt 0 ]]; do
    case $1 in
        -s|--scheme)   SCHEME="$2";   shift 2 ;;
        -d|--duration) DURATION="$2"; shift 2 ;;
        -o|--out)      OUT_ROOT="$2"; shift 2 ;;
        --region)      REGION="$2";   shift 2 ;;
        --sweep-speed) SWEEP_SPEED="$2"; shift 2 ;;
        -h|--help)     sed -n '2,14p' "$0"; exit 0 ;;
        *) echo "Opsi tidak dikenal: $1"; exit 1 ;;
    esac
done

EXTRA_ARGS=(--region "$REGION")
[ -n "$SWEEP_SPEED" ] && EXTRA_ARGS+=(--sweep-speed "$SWEEP_SPEED")

STAMP="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="$OUT_ROOT/scheme${SCHEME}_$STAMP"
mkdir -p "$RUN_DIR"

echo "========================================================================="
echo "  PERBANDINGAN KONTROLER — Skema $SCHEME, ${DURATION}s tiap kontroler"
echo "  Wilayah: $REGION${SWEEP_SPEED:+  | sweep_speed=$SWEEP_SPEED}"
echo "  Hasil: $RUN_DIR"
echo "========================================================================="

for ctrl in lqr hinf; do
    echo
    echo "### PID-${ctrl^^} mulai $(date +%T)"
    mkdir -p "$RUN_DIR/$ctrl"
    # Berurutan, bukan paralel: dua instans Gazebo akan berebut topik gz
    # yang sama dan hasilnya tidak sahih.
    timeout -s INT "$DURATION" "$WS_DIR/launch_mapping_demo.sh" \
        -s "$SCHEME" --headless "--pid-$ctrl" \
        --results "$RUN_DIR/$ctrl" \
        "${EXTRA_ARGS[@]}" \
        > "$RUN_DIR/$ctrl/coordinator.log" 2>&1
    echo "### PID-${ctrl^^} selesai $(date +%T)"
    sleep 5
done

echo
export PYTHONPATH="$WS_DIR/src/swarm_high_level:${PYTHONPATH:-}"
python3 -m swarm_high_level.metrics.run_report \
    "PID-LQR:$RUN_DIR/lqr/coordinator.log:$RUN_DIR/lqr" \
    "PID-Hinf:$RUN_DIR/hinf/coordinator.log:$RUN_DIR/hinf" \
    --json "$RUN_DIR/summary.json"
STATUS=$?

echo
echo "Artefak mentah tersimpan di: $RUN_DIR"
exit $STATUS
