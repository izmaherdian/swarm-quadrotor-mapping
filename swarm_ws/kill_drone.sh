#!/bin/bash
# ==============================================================================
#   KILL DRONE SCRIPT — Dynamic Swarm Failure Trigger (Terminal 2)
# ==============================================================================
# Penggunaan:
#   ./kill_drone.sh 4              -> Mematikan Drone 4 (Pusat)
#   ./kill_drone.sh 2 3            -> Mematikan Drone 2 & 3 sekaligus
#   ./kill_drone.sh 1 3 5 7        -> Mematikan 4 drone sekaligus
#   ./kill_drone.sh 2 3 4 5 6 7    -> Mematikan 6 drone (tersisa hanya Drone 1)
# ==============================================================================

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
BOLD='\033[1m'
NC='\033[0m'

if [ $# -eq 0 ]; then
    echo -e "${RED}${BOLD}❌ ERROR: Harap tentukan ID drone yang ingin dimatikan!${NC}"
    echo -e "${YELLOW}Contoh Penggunaan:${NC}"
    echo "  ./kill_drone.sh 4            # Matikan Drone 4"
    echo "  ./kill_drone.sh 2 3          # Matikan Drone 2 & 3"
    echo "  ./kill_drone.sh 1 3 5 7      # Matikan 4 Drone"
    exit 1
fi

DRONE_IDS=("$@")
VALID_IDS=()

for id in "${DRONE_IDS[@]}"; do
    if [[ "$id" =~ ^[1-7]$ ]]; then
        VALID_IDS+=("$id")
    else
        echo -e "${YELLOW}⚠️  Peringatan: ID drone '$id' tidak valid (harus 1-7), dilewati.${NC}"
    fi
done

if [ ${#VALID_IDS[@]} -eq 0 ]; then
    echo -e "${RED}❌ Tidak ada ID drone valid yang dapat dimatikan.${NC}"
    exit 1
fi

echo -e "${RED}${BOLD}⚡ [EMERGENCY TRIGGER] Menghentikan ${#VALID_IDS[@]} Drone: ${VALID_IDS[*]}...${NC}"

# 1. Hentikan kontroler low-level PID/LQR masing-masing drone di Gazebo
for id in "${VALID_IDS[@]}"; do
    echo -e "   💥 Mematikan flight controller & actuator ${BOLD}iris_${id}${NC}..."
    pkill -9 -f "controller_iris_${id}" 2>/dev/null || true
    pkill -9 -f "ai_iris_${id}" 2>/dev/null || true
    pkill -9 -f "tf_prefix_iris_${id}" 2>/dev/null || true
done

# 2. Kirim sinyal event darurat ke node koordinator via Python ROS 2 publisher (Reliable DDS Handshake)
echo -e "   📡 Mempublikasikan sinyal kegagalan ke ${BOLD}/swarm/kill_drone${NC}..."
python3 -c "
import sys
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

rclpy.init()
node = Node('kill_drone_cli_node')
pub = node.create_publisher(Int32MultiArray, '/swarm/kill_drone', 10)
msg = Int32MultiArray()
msg.data = [int(x) for x in sys.argv[1:]]

# Tunggu discovery DDS subscriber maksimal 1 detik
for _ in range(20):
    if pub.get_subscription_count() > 0:
        break
    rclpy.spin_once(node, timeout_sec=0.05)

for _ in range(10):
    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.05)

time.sleep(0.1)
node.destroy_node()
rclpy.shutdown()
" "${VALID_IDS[@]}"

echo -e "${GREEN}${BOLD}✅ Sukses! Drone [${VALID_IDS[*]}] telah dimatikan.${NC}"
echo -e "${BLUE}ℹ️  Koordinator swarm di Terminal 1 otomatis menjalankan re-planning Shapely recovery & mengalokasikan tugas ke drone helper yang masih hidup.${NC}"
