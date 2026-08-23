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

CSV_PATH="$WS_DIR/src/swarm_sim/results/single_agent/pid_hinf/flight_data_log_hinf_iris_1.csv"
rm -f "$CSV_PATH"

echo "=== 2. Launching Gazebo simulation with PID-H_inf (Headless) ==="
ros2 launch swarm_sim sim_launch.py \
    num_drones:=1 controller:=pid_hinf_node \
    headless:=true rviz:=false \
    results_base:=single_agent > /tmp/sim_test_flight_hinf.log 2>&1 &
SIM_PID=$!
echo "Sim PID: $SIM_PID. Waiting for hover initialization..."

for i in $(seq 1 30); do
    if [ -f "$CSV_PATH" ] && [ $(stat -c%s "$CSV_PATH" 2>/dev/null || echo 0) -gt 500 ]; then
        echo "Simulation initialized and logging after ${i}s!"
        break
    fi
    sleep 1
done

if [ ! -f "$CSV_PATH" ]; then
    echo "ERROR: Simulation failed to initialize. Log output:"
    cat /tmp/sim_test_flight_hinf.log | tail -30
    kill -9 "$SIM_PID" 2>/dev/null || true
    exit 1
fi

sleep 4

echo "=== 3. Starting Pure Continuous Boustrophedon Mapping ==="
python3 "$WS_DIR/test_mapping.py" &
MAP_PID=$!

echo "Monitoring flight..."
for i in $(seq 1 35); do
    sleep 5
    if ! kill -0 "$MAP_PID" 2>/dev/null; then
        echo "Mapping process finished."
        break
    fi
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
    tau_y = float(last['tau_y'])
    print(f'  [T={t:6.1f}s] Pos=({x:5.2f}, {y:5.2f}, {z:4.2f}) | P={p:5.1f}° R={ro:5.1f}° Y={ya:5.1f}° | tau_y={tau_y:5.2f}')
"
done

echo "=== 4. Cleaning up Simulation ==="
kill -9 "$MAP_PID" 2>/dev/null || true
kill -9 "$SIM_PID" 2>/dev/null || true
for pid in $(ps aux | grep -E "gz.sim|ros2 launch|parameter_bridge|ai_iris|pid_lqr_node|pid_hinf_node" | grep -v grep | awk '{print $2}'); do
    kill -9 "$pid" 2>/dev/null || true
done
sleep 1

echo "=== 5. Final Flight Data Analysis (PID-H_inf) ==="
python3 -c "
import csv
with open('$CSV_PATH', 'r') as f:
    rows = list(csv.DictReader(f))

print(f'Total telemetry samples recorded: {len(rows)}')
if rows:
    airborne_rows = [r for r in rows if float(r['Time_s']) >= 5.0]
    if not airborne_rows:
        airborne_rows = rows
    
    pitches = [float(r['Pitch_deg']) for r in airborne_rows]
    rolls = [float(r['Roll_deg']) for r in airborne_rows]
    yaws = [float(r['Yaw_deg']) for r in airborne_rows]
    alts = [float(r['Z']) for r in airborne_rows]
    tau_ys = [float(r['tau_y']) for r in airborne_rows]
    
    t_start = float(airborne_rows[0]['Time_s'])
    t_end = float(airborne_rows[-1]['Time_s'])
    duration = t_end - t_start
    
    print(f'Active Flight Duration: {duration:.1f}s (T={t_start:.1f}s to {t_end:.1f}s)')
    print(f'Max |Pitch|: {max(abs(p) for p in pitches):.2f}°')
    print(f'Max |Roll|:  {max(abs(r) for r in rolls):.2f}°')
    print(f'Max |Yaw|:   {max(abs(y) for y in yaws):.2f}°')
    print(f'Altitude: min={min(alts):.2f}m, max={max(alts):.2f}m, target=2.00m')
    print(f'Max |tau_y|: {max(abs(ty) for ty in tau_ys):.2f} Nm (Limit: 0.80 Nm)')
    
    crashed = any(z < 1.0 for z in alts) or max(abs(p) for p in pitches) > 45.0 or max(abs(r) for r in rolls) > 45.0
    if crashed:
        print('STATUS: ❌ FLIGHT CRASHED')
    elif duration > 300.0:
        print('STATUS: ✅ FULL 10-ROW MISSION COMPLETED CRASH-FREE & HIGHLY STABLE!')
    else:
        print('STATUS: ⚠️ FLIGHT STABLE BUT SHORT DURATION')
"
