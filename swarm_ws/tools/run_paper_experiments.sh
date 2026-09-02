#!/usr/bin/env bash
# ==============================================================================
# MASTER BATCH RUNNER: 40 SIMULATION MATRIX (EPIC 2026 PAPER EVALUATION)
# 4 Regions x 5 Schemes x 2 Controllers = 40 Headless Executions
# Auto-Audited, Zero-Hallucination, Live-Logged to report.md
# ==============================================================================

set -e

WS_DIR="/home/izmaherdian/Documents/swarm-quadrotor-mapping/swarm_ws"
cd "$WS_DIR"

source /opt/ros/lyrical/setup.bash 2>/dev/null || source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null || true
source "$WS_DIR/install/setup.bash"

export GZ_SIM_RESOURCE_PATH="$WS_DIR/src/swarm_sim/models:$GZ_SIM_RESOURCE_PATH"
export PYTHONPATH="$WS_DIR/src/swarm_high_level:$WS_DIR/src/swarm_mid_level:$WS_DIR/src/swarm_low_level:${PYTHONPATH:-}"

REGIONS=("rect" "l_shape" "u_shape" "plus")
SCHEMES=(1 2 3 4 5)
CONTROLLERS=("pid_hinf" "pid_lqr")

ROW_ID=1
TOTAL_RUNS=40
START_GLOBAL=$(date +%s)

echo "========================================================================="
echo "  🚀 MEMULAI EKSEKUSI MASTER BATCH 40 SIMULASI (PAPER EPIC 2026)"
echo "  📊 Matriks: 4 Bentuk x 5 Skema x 2 Kontroler = 40 Konfigurasi"
echo "  📁 Output Log: results/paper_evaluation/ & report.md"
echo "========================================================================="

for region in "${REGIONS[@]}"; do
  for scheme in "${SCHEMES[@]}"; do
    for ctrl in "${CONTROLLERS[@]}"; do
      NOW=$(date +%s)
      ELAPSED=$((NOW - START_GLOBAL))
      
      echo ""
      echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
      printf "▶ [%02d/%02d] Region: %-8s | Skema: %d | Kontroler: %-8s\n" "$ROW_ID" "$TOTAL_RUNS" "$region" "$scheme" "$ctrl"
      echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
      
      # Jalankan auto tuner pipeline untuk 1 run
      python3 tools/auto_tuner_pipeline.py \
        --region "$region" \
        --scheme "$scheme" \
        --controller "$ctrl" \
        --row-id "$ROW_ID" \
        --timeout 650 \
        --skip-existing
        
      ROW_ID=$((ROW_ID + 1))
    done
  done
done

echo ""
echo "========================================================================="
echo "  🎉 SELURUH 40 KONFIGURASI SIMULASI BERHASIL DIEKSEKUSI 100%!"
echo "  📝 Seluruh data nyata tersimpan rapi di report.md"
echo "========================================================================="
