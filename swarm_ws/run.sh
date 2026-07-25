#!/bin/bash
set -e

echo "🚀 Memulai proses Build & Run Swarm Drone..."

# Pindah ke direktori tempat script ini berada (selalu tepat walau dipanggil dari mana saja)
cd "$(dirname "$0")"
WS_DIR="$PWD"

# 0. Matikan sisa proses node / gazebo dari sesi sebelumnya secara aman
pkill -9 -f "install/swarm_mid_level|install/swarm_low_level|install/swarm_sim|ros_gz_sim|parameter_bridge" 2>/dev/null || true

# 1. Bersihkan riwayat environment & Source dependensi dasar
unset AMENT_PREFIX_PATH
unset PYTHONPATH
unset LD_LIBRARY_PATH
source /opt/ros/lyrical/setup.bash
source "$WS_DIR/../.venv/bin/activate"

# 2. Setup symlink python yang pendek agar shebang colcon tidak terpotong (error path too long)
ln -sf "$WS_DIR/../.venv/bin/python" /tmp/venv_py

# 3. Jalankan colcon build dengan Python yang benar
echo "🔨 Membangun (Build) workspace..."
# Pastikan CMake memakai python dari venv (bukan python sistem)
export PYTHON_EXECUTABLE="/tmp/venv_py"
colcon build \
    --cmake-args \
        -DPYTHON_EXECUTABLE="/tmp/venv_py" \
        -DPython3_EXECUTABLE="/tmp/venv_py" \
    --event-handlers console_cohesion+

# 4. Patching shebang secara aman di sistem partisi NTFS (tanpa mengubah permission bawaan)
echo "🔧 Memperbaiki kompatibilitas eksekusi file..."
for pkg in swarm_high_level swarm_low_level swarm_mid_level; do 
    for f in install/$pkg/lib/$pkg/*; do 
        if [ -f "$f" ] && head -n 1 "$f" | grep -q "python"; then
            sed '1s|^#!.*|#!/tmp/venv_py|' "$f" > "$f.tmp"
            cat "$f.tmp" > "$f"
            rm "$f.tmp"
        fi
    done
done

# 5. Set environment variables (Bypass bug spasi di folder Windows)
echo "🔗 Mengaitkan Path ROS 2 (Workaround untuk bug spasi)..."
export AMENT_PREFIX_PATH="$WS_DIR/install/swarm_msgs:$WS_DIR/install/swarm_sim:$WS_DIR/install/swarm_high_level:$WS_DIR/install/swarm_low_level:$WS_DIR/install/swarm_mid_level:$AMENT_PREFIX_PATH"
export PYTHONPATH="$WS_DIR/install/swarm_msgs/local/lib/python3.14/dist-packages:$WS_DIR/install/swarm_sim/lib/python3.14/site-packages:$WS_DIR/install/swarm_high_level/lib/python3.14/site-packages:$WS_DIR/install/swarm_low_level/lib/python3.14/site-packages:$WS_DIR/install/swarm_mid_level/lib/python3.14/site-packages:$PYTHONPATH"
export LD_LIBRARY_PATH="$WS_DIR/install/swarm_sim/lib:$LD_LIBRARY_PATH"
export PATH="$WS_DIR/install/swarm_sim/lib/swarm_sim:$WS_DIR/install/swarm_high_level/lib/swarm_high_level:$WS_DIR/install/swarm_low_level/lib/swarm_low_level:$WS_DIR/install/swarm_mid_level/lib/swarm_mid_level:$PATH"

# 6. Tangkap parameter/argumen tambahan yang diinput user (default jika kosong)
ARGS="$@"
if [ -z "$ARGS" ]; then
    ARGS="num_drones:=7 headless:=false rviz:=false controller:=pid_lqr_node"
fi

# 7. Jalankan simulasi
echo ""
echo "🎮 Menjalankan simulasi dengan argumen: $ARGS"
echo "   (Tekan Ctrl+C untuk mematikan simulasi)"
echo "--------------------------------------------------------"
ros2 launch swarm_sim sim_launch.py $ARGS
