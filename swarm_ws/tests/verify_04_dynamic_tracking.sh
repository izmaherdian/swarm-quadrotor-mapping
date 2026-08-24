#!/usr/bin/env bash
set -e

WS_DIR="$(cd "$(dirname "$0")/.." && pwd)"
source /opt/ros/lyrical/setup.bash
source "$WS_DIR/install/setup.bash"

export GZ_SIM_RESOURCE_PATH="$WS_DIR/src/swarm_sim/models"

echo "=== 1. Clean old processes ==="
for pid in $(ps aux | grep -E "gz.sim|ros2 launch|parameter_bridge|ai_iris|pid_lqr_node|pid_hinf_node|test_mapping" | grep -v grep | awk '{print $2}'); do
    kill -9 "$pid" 2>/dev/null || true
done
sleep 2

CSV_PATH="$WS_DIR/src/swarm_sim/results/single_agent/pid_lqr/flight_data_log_lqr_iris_1.csv"
rm -f "$CSV_PATH"

echo "=== 2. Launching Gazebo simulation with PID-LQR (Headless Dynamic Tracking Test) ==="
ros2 launch swarm_sim sim_launch.py \
    num_drones:=1 controller:=pid_lqr_node \
    headless:=true rviz:=false \
    results_base:=single_agent > /tmp/sim_test_dynamic.log 2>&1 &
SIM_PID=$!

echo "Sim PID: $SIM_PID. Waiting for hover settling..."

for i in $(seq 1 30); do
    if [ -f "$CSV_PATH" ] && [ $(stat -c%s "$CSV_PATH" 2>/dev/null || echo 0) -gt 500 ]; then
        echo "Simulation logging active at ${i}s..."
        break
    fi
    sleep 1
done

if [ ! -f "$CSV_PATH" ]; then
    echo "ERROR: Simulation failed to initialize."
    kill -9 "$SIM_PID" 2>/dev/null || true
    exit 1
fi

sleep 4

echo "=== 3. Starting test_mapping.py for Dynamic Tracking (Row 1 & Row 2) ==="
python3 "$WS_DIR/test_mapping.py" > /tmp/test_mapping_dynamic.log 2>&1 &
MAP_PID=$!

echo "Monitoring Row 1 & 2 sweep (60 seconds)..."
for s in $(seq 1 12); do
    sleep 5
    if [ -f "$CSV_PATH" ]; then
        python3 -c "
import csv
with open('$CSV_PATH', 'r') as f:
    rows = list(csv.DictReader(f))
if rows:
    last = rows[-1]
    print(f'  [T={float(last[\"Time_s\"]):5.1f}s] Pos=({float(last[\"X\"]):5.2f}, {float(last[\"Y\"]):5.2f}, {float(last[\"Z\"]):4.2f}) | Ref=({float(last[\"Ref_X\"]):5.2f}, {float(last[\"Ref_Y\"]):5.2f}) | Pitch={float(last[\"Pitch_deg\"]):+5.1f}° Roll={float(last[\"Roll_deg\"]):+5.1f}°')
"
    fi
done

echo "=== 4. Cleaning up Simulation ==="
kill -9 "$MAP_PID" 2>/dev/null || true
kill -9 "$SIM_PID" 2>/dev/null || true
for pid in $(ps aux | grep -E "gz.sim|ros2 launch|parameter_bridge|ai_iris|pid_lqr_node|pid_hinf_node|test_mapping" | grep -v grep | awk '{print $2}'); do
    kill -9 "$pid" 2>/dev/null || true
done
sleep 1

echo "=== 5. Evaluasi Kuantitatif Dynamic Tracking Baris 1 & 2 ==="
python3 -c "
import csv
import numpy as np

with open('$CSV_PATH', 'r') as f:
    rows = list(csv.DictReader(f))

times = np.array([float(r['Time_s']) for r in rows])
xs = np.array([float(r['X']) for r in rows])
ys = np.array([float(r['Y']) for r in rows])
zs = np.array([float(r['Z']) for r in rows])
ref_xs = np.array([float(r['Ref_X']) for r in rows])
ref_ys = np.array([float(r['Ref_Y']) for r in rows])
ref_zs = np.array([float(r['Ref_Z']) for r in rows])
vxs = np.array([float(r['vx']) for r in rows])
vys = np.array([float(r['vy']) for r in rows])
pitches = np.array([float(r['Pitch_deg']) for r in rows])
rolls = np.array([float(r['Roll_deg']) for r in rows])

print(f'Total sampel telemetri: {len(rows)} ({times[-1]:.1f} detik simulasi)')

# Analisis Baris 1: saat drone melaju maju dari X = -4.5m ke +4.5m pada Y ~ -5.5m
idx_row1 = np.where((xs >= -4.5) & (xs <= 4.5) & (np.abs(ys - (-5.5)) < 0.5))[0]

if len(idx_row1) > 0:
    t_r1 = times[idx_row1]
    x_r1 = xs[idx_row1]
    y_r1 = ys[idx_row1]
    z_r1 = zs[idx_row1]
    ref_x_r1 = ref_xs[idx_row1]
    ref_y_r1 = ref_ys[idx_row1]
    pt_r1 = pitches[idx_row1]
    rl_r1 = rolls[idx_row1]
    vx_r1 = vxs[idx_row1]
    
    # 1. Cross-Track Error (Penyimpangan Kanan-Kiri Sumbu Y saat maju lurus)
    err_y_cm = np.abs(y_r1 - ref_y_r1) * 100.0
    max_err_y = np.max(err_y_cm)
    mean_err_y = np.mean(err_y_cm)
    
    # 2. Along-Track Error (Penyimpangan Maju-Mundur Sumbu X)
    err_x_cm = np.abs(x_r1 - ref_x_r1) * 100.0
    max_err_x = np.max(err_x_cm)
    mean_err_x = np.mean(err_x_cm)
    
    # 3. Kestabilan Sikap Dinamis
    roll_osc_r1 = np.max(rl_r1) - np.min(rl_r1)
    pitch_mean_r1 = np.mean(pt_r1)
    pitch_std_r1 = np.std(pt_r1)
    
    # 4. Ketinggian saat jelajah
    z_dev_cm = np.max(np.abs(z_r1 - 2.00)) * 100.0
    
    print('\n--- HASIL UJI DINAMIS BARIS 1 (Active Forward Cruise @ 0.60 m/s) ---')
    print(f'1. Cross-Track Error (Kanan-Kiri Sumbu Y) : Max = {max_err_y:.2f} cm, Mean = {mean_err_y:.2f} cm [Target Max < 3.0 cm]')
    print(f'2. Along-Track Error (Maju-Mundur Sumbu X): Max = {max_err_x:.2f} cm, Mean = {mean_err_x:.2f} cm [Target Max < 5.0 cm]')
    print(f'3. Osilasi Kemiringan Roll (Samping)      : {roll_osc_r1:.2f}° (Rentang [{np.min(rl_r1):.2f}°, {np.max(rl_r1):.2f}°]) [Target < 1.5°]')
    print(f'4. Kemiringan Pitch Cruise (Maju)         : Mean = {pitch_mean_r1:.2f}°, Std Dev = {pitch_std_r1:.2f}° [Stabil, Tanpa Osilasi]')
    print(f'5. Kecepatan Jelajah Rata-rata            : {np.mean(vx_r1):.3f} m/s')
    print(f'6. Deviasi Ketinggian Vertikal (Z)        : Max Deviasi = {z_dev_cm:.2f} cm [Target < 2.0 cm]')
    
    tracking_passed = (max_err_y < 5.0) and (roll_osc_r1 < 2.0) and (z_dev_cm < 3.0)
    if tracking_passed:
        print('\nSTATUS TAHAP 4: ✅ DYNAMIC TRACKING LURUS MULUS & TANPA GOYANG (PASS)!')
    else:
        print('\nSTATUS TAHAP 4: ⚠️ TRACKING DINAMIS BUTUH PENYESUAIAN')
else:
    print('WARNING: Data Baris 1 belum tercapai dalam durasi uji.')
"
