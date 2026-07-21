#!/usr/bin/env python3
"""
===============================================================================
  NODE ROS 2 HIGH-LEVEL: TAKEOFF MANAGER (MANAJER TAKEOFF TERPUSAT)
===============================================================================

Deskripsi:
  Node ini bertanggung jawab untuk memerintahkan seluruh drone dalam swarm
  untuk lepas landas (takeoff) secara serentak ke ketinggian target yang
  ditentukan (default: Z = 2.0 meter).

  Node ini berjalan sekali, menunggu 3 detik agar Gazebo, bridge, dan
  kontroler PID-LQR siap, kemudian mulai mempublikasikan target pose ke
  semua topic /iris_{i}/target_pose pada frekuensi 10 Hz.

Alur Kerja:
  1. Startup → Timer 3 detik (menunggu sistem siap)
  2. Setelah delay → mulai publish PoseStamped target Z=2.0m ke semua drone
  3. Terus publish di frekuensi 10 Hz agar kontroler PID-LQR selalu menerima
     sinyal referensi terbaru

ROS 2 Topics Published:
  - /iris_{i}/target_pose  (geometry_msgs/PoseStamped) : Target posisi hovering

Parameter ROS 2:
  - num_drones     (int, default=7)   : Jumlah drone aktif
  - takeoff_alt    (float, default=2.0): Ketinggian target hovering (meter)
  - startup_delay  (float, default=3.0): Delay sebelum mulai kirim command (detik)
===============================================================================
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math


class TakeoffManager(Node):
    """
    Node terpusat yang mempublikasikan perintah takeoff ke semua drone
    dalam swarm secara serentak.
    """

    def __init__(self):
        super().__init__('takeoff_manager')

        # ── Parameter ROS 2 ──────────────────────────────────────────────────
        self.declare_parameter('num_drones',    7)
        self.declare_parameter('takeoff_alt',   2.0)
        self.declare_parameter('startup_delay', 0.5)

        self.num_drones    = self.get_parameter('num_drones').value
        self.takeoff_alt   = self.get_parameter('takeoff_alt').value
        self.startup_delay = self.get_parameter('startup_delay').value

        # ── Publisher per Drone ───────────────────────────────────────────────
        # Buat satu publisher untuk tiap drone dengan topic /iris_{i}/target_pose
        # PENTING: nama 'drone_pubs' (bukan '_publishers') agar tidak konflik
        # dengan internal attribute rclpy.Node yang juga bernama '_publishers' (list)
        self.drone_pubs = {}
        self.target_pubs = {}
        for i in range(1, self.num_drones + 1):
            topic_wp = f'/iris_{i}/waypoint_pose'
            topic_tp = f'/iris_{i}/target_pose'
            self.drone_pubs[i] = self.create_publisher(PoseStamped, topic_wp, 10)
            self.target_pubs[i] = self.create_publisher(PoseStamped, topic_tp, 10)

        # ── State Flag ───────────────────────────────────────────────────────
        self._ready = False  # Akan True setelah startup_delay habis

        # ── Startup Delay Timer (one-shot, berjalan sekali) ──────────────────
        self.create_timer(self.startup_delay, self._on_startup_ready)

        self.get_logger().info(
            f"╔══════════════════════════════════════════╗\n"
            f"║     TAKEOFF MANAGER AKTIF                ║\n"
            f"║  Jumlah drone  : {self.num_drones} unit              ║\n"
            f"║  Target Z      : {self.takeoff_alt:.1f} m               ║\n"
            f"║  Delay startup : {self.startup_delay:.1f} detik            ║\n"
            f"╚══════════════════════════════════════════╝"
        )

    def _on_startup_ready(self):
        """
        Callback setelah startup_delay selesai.
        Mengaktifkan flag siap dan memulai timer publish berkala.
        """
        self._ready = True
        self.get_logger().info(
            f"[TakeoffManager] Sistem siap! Memulai perintah takeoff ke "
            f"{self.num_drones} drone → Z = {self.takeoff_alt:.1f} m"
        )

        # Timer publish 10 Hz (0.1 detik sekali) — terus kirim referensi hover
        self.create_timer(0.1, self._publish_takeoff_commands)

    def _publish_takeoff_commands(self):
        """
        Publish PoseStamped target hovering ke semua drone.
        Dipanggil setiap 0.1 detik (10 Hz).

        Target:
          - x = 0.0 (tetap di posisi spawn)
          - y = posisi spawn awal masing-masing drone
          - z = takeoff_alt (default 2.0 meter)
          - yaw = 0.0 (heading lurus ke depan)
        """
        if not self._ready:
            return

        now = self.get_clock().now().to_msg()

        for i, pub in self.drone_pubs.items():
            msg = PoseStamped()
            msg.header.stamp    = now
            msg.header.frame_id = 'world'

            # Posisi target hovering: drone tetap di x=0.0, y=spawn_y, z=alt
            # y_spawn dihitung sama seperti swarm_launch.py: y = (i - 2.0) * spacing
            spacing = 2.0
            y_spawn = (i - 2.0) * spacing

            msg.pose.position.x = 0.0
            msg.pose.position.y = y_spawn
            msg.pose.position.z = self.takeoff_alt

            # Orientasi: yaw=0.0 → quaternion (x=0, y=0, z=0, w=1)
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 1.0

            pub.publish(msg)
            if i in self.target_pubs:
                self.target_pubs[i].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TakeoffManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
