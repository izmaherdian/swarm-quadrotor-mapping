#!/usr/bin/env bash
# =============================================================================
# Sweep Skema 3 (rintangan statis) — wilayah non-convex x {PID-LQR, PID-H-inf}
#
#   ./tools/sweep_scheme3.sh [-r "l_shape u_shape plus"] [-t 2400] [-o DIR]
#
# Menghasilkan  <out>/s3_<region>_<ctrl>/  berisi coordinator.log + pid_<ctrl>/*.csv,
# siap dibaca:  python3 -m swarm_high_level.metrics.region_report <out>
#
# Dua jebakan yang sudah terbukti memakan waktu dan sudah ditangani di sini:
#
#  1. Coordinator TIDAK PERNAH exit sendiri. Tanpa `--exit-after`, `timeout`
#     selalu habis penuh walau misi sudah tuntas — sweep 29 Agu memakai 241
#     menit untuk 6 misi yang masing-masing selesai ~11 menit.
#  2. Fast-DDS meninggalkan segmen shared-memory setelah `kill -9`. Ratusan
#     segmen basi membuat misi berikutnya menggantung tanpa satu pun baris
#     STATUS. Karena itu /dev/shm dibersihkan sebelum tiap misi.
#
# Catatan pola pkill: `pkill -f "gz sim"` mencocokkan baris perintah shell yang
# menjalankannya sendiri, sehingga membunuh proses induk. Kelas karakter
# `[g]z sim` mencegahnya.
# =============================================================================
set -uo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS_DIR"

REGIONS="l_shape u_shape plus"
CTRLS="lqr hinf"
TIMEOUT=2400
EXIT_AFTER="12.0"
OUT=""

while [[ $# -gt 0 ]]; do
  case $1 in
    -r|--regions) REGIONS="$2"; shift 2 ;;
    -c|--controllers) CTRLS="$2"; shift 2 ;;
    -t|--timeout) TIMEOUT="$2"; shift 2 ;;
    -o|--out) OUT="$2"; shift 2 ;;
    -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
    *) echo "Opsi tidak dikenal: $1"; exit 1 ;;
  esac
done

[ -n "$OUT" ] || OUT="$WS_DIR/results/s3_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT"
LOG="$OUT/sweep.log"

# Mesin ini adalah laptop. Enam misi 7-drone berturut-turut membuatnya
# thermal-throttle: pada sweep 31 Agu 07:02 suhu mencapai 93.8 C, RTF ambruk
# 0.31 -> 0.09, loop kendali 20 Hz tidak terkejar dan TIGA drone jatuh
# (WATCHDOG CRASH, Z=0.05 m). Data seperti itu tidak sahih tapi terlihat
# seperti kegagalan algoritma — persis jebakan yang harus dihindari.
COOL_MAX_C=${COOL_MAX_C:-70}
COOL_MAX_LOAD=${COOL_MAX_LOAD:-2}
COOL_WAIT_MAX=${COOL_WAIT_MAX:-1800}

cpu_temp() {
  sensors 2>/dev/null | grep -oE "Tctl: *\+[0-9.]+" | grep -oE "[0-9.]+" | head -1
}

# Suhu SAJA tidak cukup. Setelah `pkill`, Gazebo dan tujuh node kontroler butuh
# beberapa menit untuk benar-benar mati; selama itu load masih ~9 dan suhu
# justru NAIK walau gerbang mengira sedang mendinginkan (terukur 31 Agu: 85 C
# -> 97 C saat gerbang "menunggu"). Karena itu load average ikut dijaga —
# syarat yang sama yang dipakai manual pada run `rect` yang berhasil
# (65 C, load 1.06 -> RTF 0.29).
wait_cool() {
  local t l waited=0
  t=$(cpu_temp); [ -n "$t" ] || { sleep 120; return; }   # tanpa sensor: jeda tetap
  l=$(cut -d. -f1 /proc/loadavg)
  while { [ "${t%%.*}" -gt "$COOL_MAX_C" ] || [ "$l" -gt "$COOL_MAX_LOAD" ]; } \
        && [ "$waited" -lt "$COOL_WAIT_MAX" ]; do
    echo "    ...mendinginkan: ${t} C / load ${l}  (target <=${COOL_MAX_C} C & <=${COOL_MAX_LOAD}, tunggu ${waited}s)" | tee -a "$LOG"
    sleep 30; waited=$((waited + 30)); t=$(cpu_temp); l=$(cut -d. -f1 /proc/loadavg)
  done
  if [ "${t%%.*}" -gt "$COOL_MAX_C" ]; then
    echo "    ⚠️  BATAS TUNGGU HABIS pada ${t} C — misi ini berisiko throttling, periksa RTF-nya." | tee -a "$LOG"
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
  sleep 3
  find /dev/shm -maxdepth 1 -type f -user "$(id -un)" -delete 2>/dev/null || true
}

total=0
for _r in $REGIONS; do for _c in $CTRLS; do total=$((total + 1)); done; done

echo "=== SWEEP SKEMA 3 (rintangan statis) mulai $(date) ===" | tee "$LOG"
echo "    wilayah: $REGIONS | kontroler: $CTRLS | timeout: ${TIMEOUT}s" | tee -a "$LOG"
echo "    hasil  : $OUT" | tee -a "$LOG"

i=0
for region in $REGIONS; do
  for ctrl in $CTRLS; do
    i=$((i + 1))
    run_dir="$OUT/s3_${region}_${ctrl}"
    mkdir -p "$run_dir"
    echo "" | tee -a "$LOG"
    echo ">>> [$i/$total] $region | $ctrl | mulai $(date +%H:%M:%S)" | tee -a "$LOG"

    cleanup_env
    wait_cool
    t0=$(date +%s)
    timeout "$TIMEOUT" ./launch_mapping_demo.sh \
        -s 3 --headless --region "$region" "--pid-$ctrl" \
        --results "$run_dir" --exit-after "$EXIT_AFTER" \
        > "$run_dir/coordinator.log" 2>&1
    rc=$?
    t1=$(date +%s)

    csv=$(ls "$run_dir"/pid_"$ctrl"/*.csv 2>/dev/null | wc -l)
    cov=$(grep -oE "Cov: *[0-9.]+" "$run_dir/coordinator.log" 2>/dev/null | tail -1)
    dmin=$(grep -oE "d_min: *[0-9.]+" "$run_dir/coordinator.log" 2>/dev/null \
           | grep -oE "[0-9.]+" | sort -n | head -1)
    crash=$(grep -c "MENABRAK" "$run_dir/coordinator.log" 2>/dev/null || echo 0)
    # RTF adalah penjaga kesahihan. Di bawah ~0.20 loop kendali 20 Hz tidak
    # terkejar dan drone mulai berjatuhan; angka dari misi seperti itu TIDAK
    # boleh masuk laporan sebagai hasil algoritma.
    sim=$(tail -1 "$run_dir"/pid_"$ctrl"/*iris_2*.csv 2>/dev/null | cut -d, -f1)
    rtf=$(awk -v s="${sim:-0}" -v w="$((t1 - t0))" 'BEGIN{if(w>0)printf "%.3f", s/w; else print "n/a"}')
    warn=""
    awk -v r="$rtf" 'BEGIN{exit !(r+0 < 0.20)}' && warn="  ⚠️ RTF RENDAH — DATA TIDAK SAHIH"
    # rc=124 berarti timeout habis (auto-exit tidak sempat kena) — bukan sukses.
    echo "<<< [$i/$total] selesai $(date +%H:%M:%S) | wall $((t1 - t0))s | exit=$rc" \
         " | csv=$csv | ${cov:-Cov n/a} | d_min=${dmin:-n/a} | tabrakan=$crash" \
         " | RTF=$rtf | suhu=$(cpu_temp)C$warn" | tee -a "$LOG"
  done
done

cleanup_env
echo "" | tee -a "$LOG"
echo "=== SWEEP selesai $(date) ===" | tee -a "$LOG"
echo "Laporan: PYTHONPATH=src/swarm_high_level python3 -m swarm_high_level.metrics.region_report $OUT" \
     | tee -a "$LOG"
echo "$OUT"
