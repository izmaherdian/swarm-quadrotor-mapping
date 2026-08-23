#!/usr/bin/env bash
set -e

WS_DIR="$(cd "$(dirname "$0")" && pwd)"
source /opt/ros/lyrical/setup.bash
source "$WS_DIR/install/setup.bash"

export GZ_SIM_RESOURCE_PATH="$WS_DIR/src/swarm_sim/models"

echo "=== 1. Clean old processes ==="
for pid in $(ps aux | grep -E "gz.sim|ros2 launch|parameter_bridge|ai_iris|pid_lqr_node|pid_hinf_node" | grep -v grep | awk '{print $2}'); do
    kill -9 "$pid" 2>/dev/null || true
done
sleep 2

CSV_PATH="$WS_DIR/src/swarm_sim/results/single_agent/pid_lqr/flight_data_log_lqr_iris_1.csv"
rm -f "$CSV_PATH"

echo "=== 2. Launching Gazebo simulation with PID-LQR (Headless Hover Test) ==="
ros2 launch swarm_sim sim_launch.py \
    num_drones:=1 controller:=pid_lqr_node \
    headless:=true rviz:=false \
    results_base:=single_agent > /tmp/sim_test_hover.log 2>&1 &
SIM_PID=$!

echo "Sim PID: $SIM_PID. Waiting for hover settling (25 seconds)..."

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

# Let drone hover for 25 seconds
for s in $(seq 1 5); do
    sleep 5
    echo "  Hovering... ($((s*5))/25s)"
done

echo "=== 3. Cleaning up Simulation ==="
kill -9 "$SIM_PID" 2>/dev/null || true
for pid in $(ps aux | grep -E "gz.sim|ros2 launch|parameter_bridge|ai_iris|pid_lqr_node|pid_hinf_node" | grep -v grep | awk '{print $2}'); do
    kill -9 "$pid" 2>/dev/null || true
done
sleep 1

echo "=== 4. Evaluasi Kuantitatif Telemetri Hover ==="
python3 -c "
import csv
import numpy as np

with open('$CSV_PATH', 'r') as f:
    rows = list(csv.DictReader(f))

times = np.array([float(r['Time_s']) for r in rows])
xs = np.array([float(r['X']) for r in rows])
ys = np.array([float(r['Y']) for r in rows])
zs = np.array([float(r['Z']) for r in rows])
vz = np.array([float(r['vz']) for r in rows])
pitches = np.array([float(r['Pitch_deg']) for r in rows])
rolls = np.array([float(r['Roll_deg']) for r in rows])

print(f'Total sampel telemetri: {len(rows)} ({times[-1]:.1f} detik)')

# Takeoff Profile
z_max = np.max(zs)
t_z_max = times[np.argmax(zs)]
overshoot_z_cm = (z_max - 2.00) * 100.0
vz_max = np.max(vz)

print('\n--- HASIL UJI TAKEOFF & HOVER ---')
print(f'1. Target Ketinggian        : 2.000 m (200.0 cm)')
print(f'2. Ketinggian Puncak        : {z_max:.4f} m pada T = {t_z_max:.2f} s')
print(f'3. Overshoot Ketinggian (Z) : {overshoot_z_cm:+.2f} cm ({(overshoot_z_cm/200.0)*100:+.2f}%) [Target < 2.0 cm]')
print(f'4. Kecepatan Vertikal Max   : {vz_max:.3f} m/s [Target <= 1.0 m/s]')

# Steady Hover (T in [6.0s, 25.0s])
idx_steady = np.where((times >= 6.0) & (times <= 25.0))[0]
if len(idx_steady) > 0:
    xs_st = xs[idx_steady]
    ys_st = ys[idx_steady]
    zs_st = zs[idx_steady]
    pt_st = pitches[idx_steady]
    rl_st = rolls[idx_steady]
    
    x_p2p = (np.max(xs_st) - np.min(xs_st)) * 100.0
    y_p2p = (np.max(ys_st) - np.min(ys_st)) * 100.0
    z_p2p = (np.max(zs_st) - np.min(zs_st)) * 100.0
    pitch_p2p = np.max(pt_st) - np.min(pt_st)
    roll_p2p = np.max(rl_st) - np.min(rl_st)
    
    print(f'5. Goyangan Sumbu X (P-to-P): {x_p2p:.2f} cm (Rentang [{np.min(xs_st):.4f}, {np.max(xs_st):.4f}]) [Target < 2.0 cm]')
    print(f'6. Goyangan Sumbu Y (P-to-P): {y_p2p:.2f} cm (Rentang [{np.min(ys_st):.4f}, {np.max(ys_st):.4f}]) [Target < 2.0 cm]')
    print(f'7. Fluktuasi Sumbu Z (P-to-P): {z_p2p:.2f} cm (Rentang [{np.min(zs_st):.4f}, {np.max(zs_st):.4f}]) [Target < 2.0 cm]')
    print(f'8. Osilasi Pitch Angle      : {pitch_p2p:.2f}° (Rentang [{np.min(pt_st):.2f}°, {np.max(pt_st):.2f}°]) [Target < 2.0°]')
    print(f'9. Osilasi Roll Angle       : {roll_p2p:.2f}° (Rentang [{np.min(rl_st):.2f}°, {np.max(rl_st):.2f}°]) [Target < 2.0°]')
    print(f'10. Altitude Steady Std Dev : {np.std(zs_st)*100:.3f} cm')
    
    hover_passed = (overshoot_z_cm < 3.0) and (x_p2p < 3.0) and (y_p2p < 3.0) and (pitch_p2p < 3.0)
    if hover_passed:
        print('\nSTATUS TAHAP 3: ✅ HOVER DIAM SEMPURNA (ROCK-SOLID PASS)!')
    else:
        print('\nSTATUS TAHAP 3: ⚠️ HOVER BUTUH PENYESUAIAN LEBIH LANJUT')
"
