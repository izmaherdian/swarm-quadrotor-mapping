#!/usr/bin/env bash
# =============================================================================
# Sweep Skema 1, 2, 3 — {rect} x {Skema 1, Skema 2, Skema 3} x {hinf, lqr}
# =============================================================================
set -uo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS_DIR"

SCHEMES="1 2 3"
REGIONS="rect"
CTRLS="hinf"
TIMEOUT=2400
EXIT_AFTER="12.0"
OUT="$WS_DIR/results/benchmark_s123_$(date +%Y%m%d_%H%M%S)"

while [[ $# -gt 0 ]]; do
  case $1 in
    -s|--schemes) SCHEMES="$2"; shift 2 ;;
    -r|--regions) REGIONS="$2"; shift 2 ;;
    -c|--controllers) CTRLS="$2"; shift 2 ;;
    -t|--timeout) TIMEOUT="$2"; shift 2 ;;
    -o|--out) OUT="$2"; shift 2 ;;
    -h|--help) echo "Usage: $0 [-s '1 2 3'] [-r 'rect'] [-c 'hinf lqr'] [-o <out_dir>]"; exit 0 ;;
    *) echo "Opsi tidak dikenal: $1"; exit 1 ;;
  esac
done

mkdir -p "$OUT"
LOG="$OUT/sweep.log"

COOL_MAX_C=${COOL_MAX_C:-70}
COOL_MAX_LOAD=${COOL_MAX_LOAD:-2}
COOL_WAIT_MAX=${COOL_WAIT_MAX:-1800}

cpu_temp() {
  sensors 2>/dev/null | grep -oE "Tctl: *\+[0-9.]+" | grep -oE "[0-9.]+" | head -1
}

wait_cool() {
  local t l waited=0
  t=$(cpu_temp); [ -n "$t" ] || { sleep 30; return; }
  l=$(cut -d. -f1 /proc/loadavg)
  while { [ "${t%%.*}" -gt "$COOL_MAX_C" ] || [ "$l" -gt "$COOL_MAX_LOAD" ]; } \
        && [ "$waited" -lt "$COOL_WAIT_MAX" ]; do
    echo "    ...mendinginkan: ${t} C / load ${l}  (target <=${COOL_MAX_C} C & <=${COOL_MAX_LOAD}, tunggu ${waited}s)" | tee -a "$LOG"
    sleep 30; waited=$((waited + 30)); t=$(cpu_temp); l=$(cut -d. -f1 /proc/loadavg)
  done
  if [ "${t%%.*}" -gt "$COOL_MAX_C" ]; then
    echo "    ⚠️  BATAS TUNGGU HABIS pada ${t} C" | tee -a "$LOG"
  else
    echo "    siap: ${t} C, load ${l}" | tee -a "$LOG"
  fi
}

cleanup_env() {
  pkill -9 -f "[g]z sim"          2>/dev/null || true
  pkill -9 -f "[g]z-sim"          2>/dev/null || true
  pkill -9 -f "[p]id_lqr_node"    2>/dev/null || true
  pkill -9 -f "[p]id_hinf_node"   2>/dev/null || true
  pkill -9 -f "[t]est_7drone"     2>/dev/null || true
  pkill -9 -f "[p]arameter_bridge" 2>/dev/null || true
  pkill -9 -f "[d]ryden_wind_node" 2>/dev/null || true
  sleep 3
  find /dev/shm -maxdepth 1 -type f -user "$(id -un)" -delete 2>/dev/null || true
}

total=0
for _s in $SCHEMES; do
  for _r in $REGIONS; do
    for _c in $CTRLS; do
      total=$((total + 1))
    done
  done
done

echo "=== SWEEP SKEMA $SCHEMES mulai $(date) ===" | tee "$LOG"
echo "    skema: $SCHEMES | wilayah: $REGIONS | kontroler: $CTRLS" | tee -a "$LOG"
echo "    hasil: $OUT" | tee -a "$LOG"

i=0
for scheme in $SCHEMES; do
  for region in $REGIONS; do
    for ctrl in $CTRLS; do
      i=$((i + 1))
      run_dir="$OUT/s${scheme}_${region}_${ctrl}"
      mkdir -p "$run_dir"
      echo "" | tee -a "$LOG"
      echo ">>> [$i/$total] Skema $scheme | $region | $ctrl | mulai $(date +%H:%M:%S)" | tee -a "$LOG"

      cleanup_env
      wait_cool
      t0=$(date +%s)
      timeout "$TIMEOUT" ./launch_mapping_demo.sh \
          -s "$scheme" --headless --region "$region" "--pid-$ctrl" \
          --results "$run_dir" --exit-after "$EXIT_AFTER" \
          > "$run_dir/coordinator.log" 2>&1
      rc=$?
      t1=$(date +%s)

      csv=$(ls "$run_dir"/pid_"$ctrl"/*.csv 2>/dev/null | wc -l)
      cov=$(grep -oE "Cov: *[0-9.]+" "$run_dir/coordinator.log" 2>/dev/null | tail -1)
      dmin=$(grep -oE "d_min: *[0-9.]+" "$run_dir/coordinator.log" 2>/dev/null \
             | grep -oE "[0-9.]+" | sort -n | head -1)
      crash=$(grep -c "MENABRAK" "$run_dir/coordinator.log" 2>/dev/null || echo 0)
      sim=$(tail -1 "$run_dir"/pid_"$ctrl"/*iris_2*.csv 2>/dev/null | cut -d, -f1)
      rtf=$(awk -v s="${sim:-0}" -v w="$((t1 - t0))" 'BEGIN{if(w>0)printf "%.3f", s/w; else print "n/a"}')
      echo "<<< [$i/$total] selesai $(date +%H:%M:%S) | wall $((t1 - t0))s | exit=$rc" \
           " | csv=$csv | ${cov:-Cov n/a} | d_min=${dmin:-n/a} | tabrakan=$crash" \
           " | RTF=$rtf | suhu=$(cpu_temp)C" | tee -a "$LOG"
    done
  done
done

cleanup_env
echo "" | tee -a "$LOG"
echo "=== SWEEP SKEMA 1, 2, 3 SELESAI $(date) ===" | tee -a "$LOG"
echo "Laporan Evaluasi:" | tee -a "$LOG"
PYTHONPATH=src/swarm_high_level:src/swarm_mid_level:src/swarm_low_level python3 -m swarm_high_level.metrics.region_report "$OUT" | tee -a "$LOG"
