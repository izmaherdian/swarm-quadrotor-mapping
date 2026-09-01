#!/usr/bin/env bash
# ==============================================================================
# tune_scheme5.sh — Automated Iterative Tuning Workflow untuk Skema 5
# Skenario: Dryden Wind Turbulence + 9 Static Obstacles + 2 Dynamic Obstacles (Pola 'X')
# ==============================================================================

set -euo pipefail

WS_DIR="/home/izmaherdian/Documents/swarm-quadrotor-mapping/swarm_ws"
cd "$WS_DIR"

REGION="rect"
CTRL="hinf"
OUT="$WS_DIR/results/tune_scheme5_$(date +%Y%m%d_%H%M%S)"
EXIT_AFTER="12.0"
TIMEOUT=800
TEMP_LIMIT=85.0

usage() {
  cat <<EOF
Penggunaan: $0 [OPSI]
  -r, --region <rect|l_shape|u_shape|plus>   (default: rect)
  -c, --controller <hinf|lqr>                (default: hinf)
  -o, --output <dir>                         (default: results/tune_scheme5_TIMESTAMP)
  -h, --help                                 Tampilkan bantuan ini
EOF
  exit 1
}

while [ $# -gt 0 ]; do
  case "$1" in
    -r|--region) REGION="$2"; shift 2 ;;
    -c|--controller) CTRL="$2"; shift 2 ;;
    -o|--output) OUT="$2"; shift 2 ;;
    -h|--help) usage ;;
    *) echo "Opsi tidak dikenal: $1"; usage ;;
  esac
done

mkdir -p "$OUT"
LOG="$OUT/tune_scheme5.log"
SUMMARY_CSV="$OUT/tuning_summary.csv"

echo "trial,cbf_wind_accel,delta_dynamic,a_eff,kappa,relax_range,coverage,tabrakan,rmse_cm,d_min,alt_rms_cm,duration_s,effort,cost" > "$SUMMARY_CSV"

echo "=========================================================================" | tee "$LOG"
echo "  🚀 MEMULAI AUTOMATED ITERATIVE TUNING LOOP SKEMA 5" | tee -a "$LOG"
echo "  🌪️  GANGGUAN   : Dryden Wind Turbulence (sigma=2.5N, tau=0.5s + Gust)" | tee -a "$LOG"
echo "  🛑 RINTANGAN  : 9 Silinder Statis (r=0.40m) + 2 Silinder Dinamis (Pola 'X')" | tee -a "$LOG"
echo "  🎮 KONTROLER  : PID-${CTRL^^}" | tee -a "$LOG"
echo "  🗺️  WILAYAH    : $REGION" | tee -a "$LOG"
echo "  📂 OUTPUT     : $OUT" | tee -a "$LOG"
echo "  ⏱️  TIMEOUT    : ${TIMEOUT}s per trial" | tee -a "$LOG"
echo "=========================================================================" | tee -a "$LOG"

cleanup_env() {
  pkill -9 -f "gz sim|gzserver|gzclient|ros_gz_bridge|rviz2|parameter_bridge|test_7drone|pid_hinf|pid_lqr|dryden_wind" 2>/dev/null || true
  sleep 2
}

get_cpu_temp() {
  local max_t=0
  for f in /sys/class/thermal/thermal_zone*/temp; do
    if [ -r "$f" ]; then
      local val
      val=$(cat "$f" 2>/dev/null || echo 0)
      local deg
      deg=$(awk "BEGIN {print $val / 1000.0}")
      max_t=$(awk "BEGIN {print ($deg > $max_t) ? $deg : $max_t}")
    fi
  done
  echo "$max_t"
}

wait_cool() {
  local attempts=0
  local t
  t=$(get_cpu_temp)
  while awk "BEGIN {exit !($t > $TEMP_LIMIT)}" && [ "$attempts" -lt 3 ]; do
    echo "    [Thermal Guard] CPU temp ${t}°C > ${TEMP_LIMIT}°C. Menunggu pendinginan 10 detik..." | tee -a "$LOG"
    sleep 10
    attempts=$((attempts + 1))
    t=$(get_cpu_temp)
  done
}

# ── GRID PARAMETER TUNING SKEMA 5 (5 TRIALS) ─────────────────────────────────
# Format: "cbf_wind_accel delta_dynamic a_eff kappa relax_range"
TRIALS=(
  "0.50 0.45 3.80 0.60 1.60"  # Trial 1: Baseline Robust
  "0.80 0.50 3.50 0.65 1.80"  # Trial 2: High Wind Resilience & Wide Clearance
  "0.50 0.50 4.00 0.70 1.60"  # Trial 3: Agile Dynamic Evasion in Wind
  "0.60 0.45 4.00 0.60 1.60"  # Trial 4: Tight Tracking & Moderate Cushion
  "0.80 0.50 3.50 0.60 2.00"  # Trial 5: Conservative Wind Safety Cushion
)

TOTAL_TRIALS=${#TRIALS[@]}
BEST_COST=999999.0
BEST_TRIAL=0

COORD_SCRIPT="$WS_DIR/experiments/test_7drone_voronoi_mapping.py"
cp "$COORD_SCRIPT" "$COORD_SCRIPT.bak_scheme5"

restore_coord() {
  if [ -f "$COORD_SCRIPT.bak_scheme5" ]; then
    cp "$COORD_SCRIPT.bak_scheme5" "$COORD_SCRIPT"
    rm -f "$COORD_SCRIPT.bak_scheme5"
  fi
}
trap restore_coord EXIT

apply_params() {
  local c_wind="$1"
  local d_dyn="$2"
  local a_eff="$3"
  local kap="$4"
  local relax_rng="$5"

  python3 - <<PYEOF
import re

with open("$COORD_SCRIPT", "r") as f:
    code = f.read()

# Update delta_dynamic
code = re.sub(r"self\.cbf_cfg\.delta_dynamic\s*=\s*[0-9.]+", "self.cbf_cfg.delta_dynamic = $d_dyn", code)
# Update kappa
code = re.sub(r"self\.cbf_cfg\.kappa\s*=\s*[0-9.]+", "self.cbf_cfg.kappa = $kap", code)
# Update CT_RELAX_RANGE
code = re.sub(r"self\.CT_RELAX_RANGE\s*=\s*[0-9.]+", "self.CT_RELAX_RANGE = $relax_rng", code)
# Update cbf_wind_accel default parameter
code = re.sub(r"self\.declare_parameter\('cbf_wind_accel',\s*[0-9.]+\)", "self.declare_parameter('cbf_wind_accel', $c_wind)", code)

with open("$COORD_SCRIPT", "w") as f:
    f.write(code)
PYEOF
}

for i in "${!TRIALS[@]}"; do
  trial=$((i + 1))
  IFS=' ' read -r -a params <<< "${TRIALS[$i]}"
  c_wind="${params[0]}"
  d_dyn="${params[1]}"
  a_eff="${params[2]}"
  kap="${params[3]}"
  relax_rng="${params[4]}"

  trial_dir="$OUT/s5_trial${trial}_${REGION}_${CTRL}"
  mkdir -p "$trial_dir"

  echo "" | tee -a "$LOG"
  echo "-------------------------------------------------------------------------" | tee -a "$LOG"
  echo ">>> [Trial $trial/$TOTAL_TRIALS] cbf_wind=$c_wind m/s², delta_dyn=$d_dyn m, a_eff=$a_eff m/s², kappa=$kap, relax_range=$relax_rng m" | tee -a "$LOG"
  echo "    Mulai pada: $(date +%H:%M:%S)" | tee -a "$LOG"

  apply_params "$c_wind" "$d_dyn" "$a_eff" "$kap" "$relax_rng"
  cleanup_env
  wait_cool
  t0=$(date +%s)

  timeout "$TIMEOUT" ./launch_mapping_demo.sh \
      -s 5 --headless --region "$REGION" "--pid-$CTRL" \
      --results "$trial_dir" --exit-after "$EXIT_AFTER" \
      > "$trial_dir/coordinator.log" 2>&1 || true
  rc=$?
  t1=$(date +%s)

  cov=$(grep -oE "Cov: *[0-9.]+" "$trial_dir/coordinator.log" 2>/dev/null | tail -1 | grep -oE "[0-9.]+" || echo "0.0")
  dmin=$(grep -oE "d_min: *[0-9.]+" "$trial_dir/coordinator.log" 2>/dev/null | grep -oE "[0-9.]+" | sort -n | head -1 || echo "0.0")
  crash=$(grep -c "MENABRAK" "$trial_dir/coordinator.log" 2>/dev/null | tail -1 | tr -dc '0-9' || echo 0)
  [ -n "$crash" ] || crash=0

  # Ambil RMSE cross-track
  rmse=$(grep "Cross-track RMSE (cm)" "$trial_dir/coordinator.log" 2>/dev/null | grep -oE "[0-9.]+" | head -1 || true)
  if [ -z "$rmse" ]; then
    rmse=$(awk -F',' 'NR>1 && $13!="" {sum+=$13^2; count++} END {if(count>0) printf "%.2f", sqrt(sum/count)*100; else print "5.00"}' "$trial_dir"/pid_"$CTRL"/*.csv 2>/dev/null || echo "5.00")
  fi

  # Hitung cost function: J = RMSE + 2*(100-Cov) + (crash > 0 ? 1000 : 0) + (dmin < 0.6 ? 500 : 0)
  pen=0
  if [ "$crash" -gt 0 ]; then pen=$((pen + 1000)); fi
  cost=$(awk -v r="${rmse:-5.0}" -v c="${cov:-0.0}" -v p="$pen" 'BEGIN{printf "%.2f", r + 2.0*(100.0-c) + p}')

  echo "    <<< [Trial $trial Selesai] wall: $((t1 - t0))s | exit: $rc | Cov: ${cov}% | Tabrakan: $crash | d_min: ${dmin}m | RMSE: ${rmse}cm | Cost: $cost" | tee -a "$LOG"
  echo "$trial,$c_wind,$d_dyn,$a_eff,$kap,$relax_rng,$cov,$crash,$rmse,$dmin,0.0,$((t1-t0)),0.0,$cost" >> "$SUMMARY_CSV"

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
echo "  🏁 TUNING SKEMA 5 SELESAI!" | tee -a "$LOG"
echo "  🏆 KANDIDAT TERBAIK: Trial $BEST_TRIAL (Cost: $BEST_COST)" | tee -a "$LOG"
echo "=========================================================================" | tee -a "$LOG"

# Tampilkan tabel perbandingan
echo "" | tee -a "$LOG"
echo "📊 TABEL REKAP HASIL TUNING SKEMA 5:" | tee -a "$LOG"
column -s, -t < "$SUMMARY_CSV" | tee -a "$LOG"
