#!/usr/bin/env bash
set -e

WS_DIR="$(cd "$(dirname "$0")/.." && pwd)"
source /opt/ros/lyrical/setup.bash
if [ -f "$WS_DIR/../.venv/bin/activate" ]; then
    source "$WS_DIR/../.venv/bin/activate"
fi

export AMENT_PREFIX_PATH="$WS_DIR/install/swarm_msgs:$WS_DIR/install/swarm_sim:$WS_DIR/install/swarm_high_level:$WS_DIR/install/swarm_low_level:$WS_DIR/install/swarm_mid_level:$AMENT_PREFIX_PATH"
export PYTHONPATH="$WS_DIR/install/swarm_msgs/local/lib/python3.14/dist-packages:$WS_DIR/install/swarm_sim/lib/python3.14/site-packages:$WS_DIR/install/swarm_high_level/lib/python3.14/site-packages:$WS_DIR/install/swarm_low_level/lib/python3.14/site-packages:$WS_DIR/install/swarm_mid_level/lib/python3.14/site-packages:$PYTHONPATH"
export LD_LIBRARY_PATH="$WS_DIR/install/swarm_sim/lib:$LD_LIBRARY_PATH"
export GZ_SIM_RESOURCE_PATH="$WS_DIR/src/swarm_sim/models"

# Opsi RViz dan Headless
RVIZ_FLAG=true
HEADLESS_FLAG=false
for arg in "$@"; do
    case $arg in
        --no-rviz)
            RVIZ_FLAG=false
            ;;
        --headless)
            HEADLESS_FLAG=true
            RVIZ_FLAG=false
            ;;
    esac
done

echo "=== Kill leftover gz/ros2 ==="
for pid in $(ps aux | grep -E "gz.sim|ros2 launch|ros_gz|spawn|controller|ai_iris|bridge|tf_prefix|static_transform|collision|pid_" | grep -v grep | awk '{print $2}'); do
    kill -9 "$pid" 2>/dev/null || true
done
sleep 2
rm -f /tmp/sim_single_hinf.log

echo "=== Build swarm_high_level, swarm_mid_level, swarm_low_level & swarm_sim ==="
colcon build --packages-select swarm_high_level swarm_mid_level swarm_low_level 2>&1 | tail -4
colcon build --packages-select swarm_sim 2>&1 | tail -3

echo "=== Clean old CSVs ==="
rm -f "$WS_DIR/src/swarm_sim/results/single_agent/pid_hinf/"*.csv

cleanup() {
    trap - EXIT INT TERM
    echo ""
    echo "=== Cleanup ==="
    if [ -n "$SIM_PID" ]; then
        kill "$SIM_PID" 2>/dev/null || true
    fi
    for pid in $(ps aux | grep -E "gz.sim|ros2 launch|parameter_bridge|ai_iris|collision|pid_" | grep -v grep | awk '{print $2}'); do
        kill -9 "$pid" 2>/dev/null || true
    done
    echo ""
    echo "=== CSV hasil Single Agent (PID-H_inf) tersimpan di: ==="
    echo "  $WS_DIR/src/swarm_sim/results/single_agent/pid_hinf/"
    echo ""
    ls -lh "$WS_DIR/src/swarm_sim/results/single_agent/pid_hinf/"*.csv 2>/dev/null || echo "  (tidak ada CSV)"
    echo ""
    echo "Done (Single-Agent PID-H_inf Selesai)."
}
trap cleanup EXIT INT TERM

echo "=== Launch Gazebo sim + PID-H_inf Controller (RViz: $RVIZ_FLAG, Headless: $HEADLESS_FLAG) ==="
nohup ros2 launch swarm_sim sim_launch.py \
    num_drones:=1 controller:=pid_hinf_node \
    headless:=$HEADLESS_FLAG rviz:=$RVIZ_FLAG \
    results_base:=single_agent > /tmp/sim_single_hinf.log 2>&1 &
SIM_PID=$!
echo "  sim PID=$SIM_PID"

echo "=== Wait for ORCA & Simulation initialized ==="
for i in $(seq 1 30); do
    if grep -q "initialized" /tmp/sim_single_hinf.log 2>/dev/null; then
        echo "  Simulation & ORCA ready after ${i}s"
        break
    fi
    sleep 1
done
if ! grep -q "initialized" /tmp/sim_single_hinf.log 2>/dev/null; then
    echo "  ERROR: Simulasi tidak pernah initialized. Cek /tmp/sim_single_hinf.log"
    exit 1
fi

echo "========================================================================="
echo "  🚀 SIMULASI SIAP — Single Drone (iris_1) [Pengendali: PID-H_inf]"
echo "  Spawn di (-5.5, -5.5, 2.0) | RViz2: $RVIZ_FLAG"
echo ""
echo "  Buka TERMINAL BARU, lalu jalankan:"
echo ""
echo "    cd $WS_DIR
    source /opt/ros/lyrical/setup.bash
    source install/setup.bash

    # Opsi 1: Mode Yaw Follow (Hidung Drone Aktif Menghadap Arah Terbang)
    python3 test_mapping.py --yaw-follow

    # Opsi 2: Mode Fixed Yaw (Orientasi Yaw Tetap 0° untuk Sensor Nadir)
    python3 test_mapping.py --fixed-yaw"
echo ""
echo "  Area Pemetaan: [-6, 6] x [-6, 6] meter (Total 144 m^2)"
echo "  Visualisasi: RViz2 topik /mapping/markers (Real-Time Coverage Footprint)"
echo "========================================================================="
echo ""
echo "SIM PID=$SIM_PID — Tekan Ctrl+C di terminal ini untuk berhenti & simpan log..."
wait "$SIM_PID"
