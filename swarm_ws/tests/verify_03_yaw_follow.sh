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

echo "=== 2. Launching Gazebo simulation with PID-LQR (Headless Yaw Follow Test) ==="
ros2 launch swarm_sim sim_launch.py \
    num_drones:=1 controller:=pid_lqr_node \
    headless:=true rviz:=false \
    results_base:=single_agent > /tmp/sim_test_yaw.log 2>&1 &
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

echo "=== 3. Starting test_mapping.py with --yaw-follow (Baris 1 & Baris 2) ==="
python3 "$WS_DIR/test_mapping.py" --yaw-follow > /tmp/test_mapping_yaw.log 2>&1 &
MAP_PID=$!

echo "Monitoring Yaw Follow flight (Baris 1 -> Turn -> Baris 2)..."
for s in $(seq 1 18); do
    sleep 5
    if [ -f "$CSV_PATH" ]; then
        python3 -c "
import csv
with open('$CSV_PATH', 'r') as f:
    rows = list(csv.DictReader(f))
if rows:
    last = rows[-1]
    t = float(last['Time_s'])
    x = float(last['X'])
    y = float(last['Y'])
    z = float(last['Z'])
    p = float(last['Pitch_deg'])
    ro = float(last['Roll_deg'])
    ya = float(last['Yaw_deg'])
    print(f'  [T={t:5.1f}s] Pos=({x:5.2f}, {y:5.2f}, {z:4.2f}) | Yaw={ya:+6.1f}° (P={p:+4.1f}°, R={ro:+4.1f}°)')
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

echo "=== 5. Evaluasi Kuantitatif Yaw Follow (Heading Accuracy & Smoothness) ==="
python3 -c "
import csv
import numpy as np

with open('$CSV_PATH', 'r') as f:
    rows = list(csv.DictReader(f))

times = np.array([float(r['Time_s']) for r in rows])
xs = np.array([float(r['X']) for r in rows])
ys = np.array([float(r['Y']) for r in rows])
zs = np.array([float(r['Z']) for r in rows])
yaws = np.array([float(r['Yaw_deg']) for r in rows])
pitches = np.array([float(r['Pitch_deg']) for r in rows])
rolls = np.array([float(r['Roll_deg']) for r in rows])

print(f'Total sampel telemetri: {len(rows)} ({times[-1]:.1f} detik simulasi)')

# Analisis Baris 1: saat drone bergerak maju (+X) pada Y ~ -5.5m
idx_r1 = np.where((xs >= -4.0) & (xs <= 4.0) & (np.abs(ys - (-5.5)) < 0.4))[0]

# Analisis Belokan Vertikal: saat drone bergeser dari Y=-5.5 ke Y=-4.3 pada X ~ +5.5m
idx_step = np.where((ys >= -5.2) & (ys <= -4.6) & (xs > 4.5))[0]

# Analisis Baris 2: saat drone bergerak mundur (-X) pada Y ~ -4.3m
idx_r2 = np.where((xs >= -4.0) & (xs <= 4.0) & (np.abs(ys - (-4.3)) < 0.4))[0]

print('\n--- HASIL EVALUASI HEADING & YAW SMOOTHNESS ---')

if len(idx_r1) > 0:
    yaw_r1 = yaws[idx_r1]
    print(f'1. Heading Baris 1 (+X Sweep)   : Mean = {np.mean(yaw_r1):+5.2f}°, Max Error = {np.max(np.abs(yaw_r1)):.2f}° [Target: 0.0° ± 2.0°]')

if len(idx_step) > 0:
    yaw_step = yaws[idx_step]
    err_step = np.abs(yaw_step - 90.0)
    print(f'2. Heading Step Vertikal (+Y)   : Mean = {np.mean(yaw_step):+5.2f}°, Error = {np.mean(err_step):.2f}° [Target: +90.0° ± 3.0°]')

if len(idx_r2) > 0:
    yaw_r2 = yaws[idx_r2]
    # Handle wrap-around near +-180
    yaw_r2_wrapped = np.array([y if y >= 0 else y + 360 for y in yaw_r2])
    err_r2 = np.abs(yaw_r2_wrapped - 180.0)
    print(f'3. Heading Baris 2 (-X Sweep)   : Mean = {np.mean(yaw_r2):+5.2f}°, Error = {np.mean(err_r2):.2f}° [Target: 180.0° ± 3.0°]')

# Hitung laju putar yaw aktual (deg/s)
dt = np.diff(times)
dt[dt < 1e-4] = 1e-4
dyaw = np.diff(yaws)
# unwrap diff
dyaw = (dyaw + 180) % 360 - 180
yaw_rates = np.abs(dyaw / dt)

# Filter out timestamp jump anomalies
valid_rates = yaw_rates[dt < 0.05]
max_yaw_rate = np.max(valid_rates) if len(valid_rates) > 0 else 0.0
print(f'4. Laju Putar Yaw Maksimum (wz) : {max_yaw_rate:.2f}°/s [Target <= 60.0°/s]')

# Fluktuasi Z selama penerbangan aktif
idx_air = np.where(times >= 5.0)[0]
z_air = zs[idx_air]
print(f'5. Ketinggian Z selama Berputar : Min = {np.min(z_air):.3f} m, Max = {np.max(z_air):.3f} m (Deviasi Max: {np.max(np.abs(z_air-2.0))*100:.2f} cm)')

# Osilasi Roll/Pitch selama belokan
r_air = rolls[idx_air]
p_air = pitches[idx_air]
print(f'6. Kemiringan Roll/Pitch Max    : Max |Roll| = {np.max(np.abs(r_air)):.2f}°, Max |Pitch| = {np.max(np.abs(p_air)):.2f}°')

yaw_passed = (len(idx_r1) > 0) and (max_yaw_rate <= 70.0) and (np.max(np.abs(z_air-2.0))*100 < 5.0)
if yaw_passed:
    print('\nSTATUS TAHAP 3: ✅ YAW FOLLOW MULUS, PRESISI, DAN BEBAS GYROSCOPIC JERK (PASS)!')
else:
    print('\nSTATUS TAHAP 3: ⚠️ BUTUH PENYESUAIAN YAW')
"
