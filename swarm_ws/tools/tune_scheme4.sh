#!/usr/bin/env bash
# =============================================================================
# Automated Iterative Tuning Loop untuk Skema 4 (Rintangan Statis & Dinamis)
# Menjalankan simulasi berulang, evaluasi metrik, dan mencari parameter terbaik.
# =============================================================================
set -uo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS_DIR"

REGION="rect"
CTRL="hinf"
TIMEOUT=2400
EXIT_AFTER="12.0"
OUT="$WS_DIR/results/tune_scheme4_$(date +%Y%m%d_%H%M%S)"

while [[ $# -gt 0 ]]; do
  case $1 in
    -r|--region) REGION="$2"; shift 2 ;;
    -c|--controller) CTRL="$2"; shift 2 ;;
    -t|--timeout) TIMEOUT="$2"; shift 2 ;;
    -o|--out) OUT="$2"; shift 2 ;;
    -h|--help) echo "Usage: $0 [-r 'rect'] [-c 'hinf'] [-o <out_dir>]"; exit 0 ;;
    *) echo "Opsi tidak dikenal: $1"; exit 1 ;;
  esac
done

mkdir -p "$OUT"
LOG="$OUT/tuning.log"
SUMMARY_CSV="$OUT/tuning_summary.csv"

echo "trial,delta_dyn,a_eff_dyn,kappa,ct_relax_range,coverage,tabrakan,cross_track_rmse,d_min,clearance,duration,effort,cost" > "$SUMMARY_CSV"

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
    echo "    ...mendinginkan CPU: ${t} C / load ${l}  (target <=${COOL_MAX_C} C & <=${COOL_MAX_LOAD}, tunggu ${waited}s)" | tee -a "$LOG"
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

# Grid Kandidat Parameter: (delta_dyn a_eff_dyn kappa ct_relax_range)
CANDIDATES=(
  "0.45 3.50 0.55 1.80"   # Trial 1: Baseline Tuned Optimal
  "0.35 3.50 0.55 1.80"   # Trial 2: Tighter dynamic buffer
  "0.55 3.50 0.60 1.80"   # Trial 3: Wider dynamic safety buffer
  "0.45 4.00 0.60 1.60"   # Trial 4: High dynamic agility
  "0.45 3.00 0.50 2.00"   # Trial 5: Gentle deflection profile
)

TOTAL_TRIALS=${#CANDIDATES[@]}

echo "=========================================================================" | tee "$LOG"
echo "  🚀 MEMULAI AUTOMATED ITERATIVE TUNING LOOP SKEMA 4" | tee -a "$LOG"
echo "  📋 Total Trial: $TOTAL_TRIALS | Wilayah: $REGION | Kontroler: $CTRL" | tee -a "$LOG"
echo "  📁 Direktori Hasil: $OUT" | tee -a "$LOG"
echo "=========================================================================" | tee -a "$LOG"

BEST_TRIAL=0
BEST_COST=999999.0

for idx in "${!CANDIDATES[@]}"; do
  trial=$((idx + 1))
  params=(${CANDIDATES[$idx]})
  d_dyn="${params[0]}"
  a_eff="${params[1]}"
  kap="${params[2]}"
  relax_rng="${params[3]}"

  trial_dir="$OUT/s4_trial${trial}_${REGION}_${CTRL}"
  mkdir -p "$trial_dir"

  echo "" | tee -a "$LOG"
  echo "-------------------------------------------------------------------------" | tee -a "$LOG"
  echo ">>> [Trial $trial/$TOTAL_TRIALS] delta_dyn=$d_dyn m, a_eff=$a_eff m/s², kappa=$kap, relax_range=$relax_rng m" | tee -a "$LOG"
  echo "    Mulai pada: $(date +%H:%M:%S)" | tee -a "$LOG"

  cleanup_env
  wait_cool
  t0=$(date +%s)

  timeout "$TIMEOUT" ./launch_mapping_demo.sh \
      -s 4 --headless --region "$REGION" "--pid-$CTRL" \
      --results "$trial_dir" --exit-after "$EXIT_AFTER" \
      > "$trial_dir/coordinator.log" 2>&1
  rc=$?
  t1=$(date +%s)

  cov=$(grep -oE "Cov: *[0-9.]+" "$trial_dir/coordinator.log" 2>/dev/null | tail -1 | grep -oE "[0-9.]+" || echo "0.0")
  dmin=$(grep -oE "d_min: *[0-9.]+" "$trial_dir/coordinator.log" 2>/dev/null | grep -oE "[0-9.]+" | sort -n | head -1 || echo "0.0")
  crash=$(grep -c "MENABRAK" "$trial_dir/coordinator.log" 2>/dev/null | tail -1 | tr -dc '0-9' || echo 0)
  [ -n "$crash" ] || crash=0

  # Ambil RMSE cross-track
  rmse=$(grep "Cross-track RMSE (cm)" "$trial_dir/coordinator.log" 2>/dev/null | grep -oE "[0-9.]+" | head -1 || true)
  if [ -z "$rmse" ]; then
    # Parse dari CSV langsung jika tidak ada di log
    rmse=$(awk -F',' 'NR>1 && $13!="" {sum+=$13^2; count++} END {if(count>0) printf "%.2f", sqrt(sum/count)*100; else print "5.00"}' "$trial_dir"/pid_"$CTRL"/*.csv 2>/dev/null || echo "5.00")
  fi

  # Hitung cost function: J = RMSE + 2*(100-Cov) + (crash > 0 ? 1000 : 0) + (dmin < 0.6 ? 500 : 0)
  pen=0
  if [ "$crash" -gt 0 ]; then pen=$((pen + 1000)); fi
  cost=$(awk -v r="${rmse:-5.0}" -v c="${cov:-0.0}" -v p="$pen" 'BEGIN{printf "%.2f", r + 2.0*(100.0-c) + p}')

  echo "    <<< [Trial $trial Selesai] wall: $((t1 - t0))s | exit: $rc | Cov: ${cov}% | Tabrakan: $crash | d_min: ${dmin}m | RMSE: ${rmse}cm | Cost: $cost" | tee -a "$LOG"
  echo "$trial,$d_dyn,$a_eff,$kap,$relax_rng,$cov,$crash,$rmse,$dmin,0.0,$((t1-t0)),0.0,$cost" >> "$SUMMARY_CSV"

  is_better=$(awk -v cur="$cost" -v best="$BEST_COST" 'BEGIN{print (cur < best) ? 1 : 0}')
  if [ "$is_better" -eq 1 ] && [ "$crash" -eq 0 ]; then
    BEST_COST="$cost"
    BEST_TRIAL="$trial"
    echo "    ⭐ [NEW BEST CANDIDATE!] Trial $trial menjadi konfigurasi terbaik sementara (Cost: $BEST_COST)" | tee -a "$LOG"
    rm -rf "$OUT/best_candidate"
    cp -r "$trial_dir" "$OUT/best_candidate"
  fi
done

cleanup_env
echo "" | tee -a "$LOG"
echo "=========================================================================" | tee -a "$LOG"
echo "  🏆 AUTOMATED TUNING LOOP SELESAI $(date)" | tee -a "$LOG"
echo "  ⭐ Konfigurasi Terbaik Ditemukan: Trial #$BEST_TRIAL (Cost: $BEST_COST)" | tee -a "$LOG"
echo "  📁 Rangkuman Lengkap: $SUMMARY_CSV" | tee -a "$LOG"
echo "=========================================================================" | tee -a "$LOG"
echo "" | tee -a "$LOG"
PYTHONPATH=src/swarm_high_level:src/swarm_mid_level:src/swarm_low_level python3 -m swarm_high_level.metrics.region_report "$OUT" | tee -a "$LOG"
