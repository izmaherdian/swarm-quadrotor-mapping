#!/usr/bin/env python3
"""
test_trajectory_2.py — 2-Drone Symmetrical Opposite Circular Trajectory (R=6.0m)

Misi 2 Drone Bergerak Melingkar Saling Berlawanan Arah pada Lingkaran Bersama:
  - Pusat Lingkaran (Xc, Yc) : (0.0, 0.0) [Origin]
  - Radius (R)                : 6.0 meter
  - Ketinggian (Z)            : 2.0 meter
  - Kecepatan Tangensial (v)  : 0.60 m/s (omega = 0.10 rad/s, periode ~62.8s per lap)
  - Drone 1 (iris_1)          : Spawn di Barat (-6.0, 0.0) -> Gerak Melingkar CCW (Counter-Clockwise)
  - Drone 2 (iris_2)          : Spawn di Timur (+6.0, 0.0) -> Gerak Melingkar CW (Clockwise)
  - Titik Berpapasan (Head-on):
      1. Puncak Utara (X=0.0, Y=+6.0) pada progres 25%
      2. Puncak Selatan (X=0.0, Y=-6.0) pada progres 75%
  - Total Putaran             : 2 Lap Penuh (4 * pi radian)
  - Pasca Selesai             : Hovering diam di posisi akhir masing-masing
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point as GeometryPoint
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
import math
import numpy as np
import sys

class DualSymmetricTrajectoryNode(Node):
    def __init__(self):
        super().__init__('dual_symmetric_trajectory_node')

        # Publishers cmd_vel untuk iris_1 dan iris_2
        self.pub_vel_1 = self.create_publisher(Twist, '/iris_1/cmd_vel', 10)
        self.pub_vel_2 = self.create_publisher(Twist, '/iris_2/cmd_vel', 10)

        # Publisher visualisasi lintasan khayal & waypoints di RViz2
        self.pub_ref_markers = self.create_publisher(MarkerArray, '/swarm_reference_track', 10)

        # Subscribers odometry
        self.sub_odom_1 = self.create_subscription(Odometry, '/iris_1/odometry', self.odom_1_callback, 10)
        self.sub_odom_2 = self.create_subscription(Odometry, '/iris_2/odometry', self.odom_2_callback, 10)

        # Loop timer pada 10 Hz (dt = 0.1 detik)
        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.control_loop)

        # Parameter Geometri Trajectory Lingkaran Bersama
        self.center_x = 0.0
        self.center_y = 0.0
        self.radius = 6.0                                  # meter
        self.nominal_speed = 0.60                          # m/s
        self.omega = self.nominal_speed / self.radius      # 0.10 rad/s
        self.total_laps = 2                                # jumlah putaran
        self.target_z = 2.0                                # meter

        # Drone 1 State (CCW dari Barat (-6.0, 0.0))
        self.drone_1_pos = np.array([-6.0, 0.0, 0.0], dtype=np.float32)
        self.drone_1_yaw = 0.0
        self.drone_1_odom_ready = False
        self.theta_1 = 0.0

        # Drone 2 State (CW dari Timur (+6.0, 0.0))
        self.drone_2_pos = np.array([6.0, 0.0, 0.0], dtype=np.float32)
        self.drone_2_yaw = 0.0
        self.drone_2_odom_ready = False
        self.theta_2 = 0.0

        # State Control
        self.state = 'wait_odom'
        self.takeoff_timer = 0
        self.step_count = 0

        self.get_logger().info('DualSymmetricTrajectoryNode siap — menunggu odometry iris_1 dan iris_2...')

    def euler_from_quaternion(self, x, y, z, w):
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        return math.atan2(t3, t4)

    def odom_1_callback(self, msg):
        self.drone_1_pos[0] = msg.pose.pose.position.x
        self.drone_1_pos[1] = msg.pose.pose.position.y
        self.drone_1_pos[2] = msg.pose.pose.position.z
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.drone_1_yaw = self.euler_from_quaternion(qx, qy, qz, qw)
        self.drone_1_odom_ready = True
        self.check_all_odom()

    def odom_2_callback(self, msg):
        self.drone_2_pos[0] = msg.pose.pose.position.x
        self.drone_2_pos[1] = msg.pose.pose.position.y
        self.drone_2_pos[2] = msg.pose.pose.position.z
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.drone_2_yaw = self.euler_from_quaternion(qx, qy, qz, qw)
        self.drone_2_odom_ready = True
        self.check_all_odom()

    def check_all_odom(self):
        if self.state == 'wait_odom' and self.drone_1_odom_ready and self.drone_2_odom_ready:
            self.state = 'wait_takeoff'
            self.takeoff_timer = 30  # 3 detik takeoff stabilization (30 tick @ 10Hz)
            self.get_logger().info('Odometry kedua drone diterima!')
            self.get_logger().info(f'  iris_1 Posisi Awal : ({self.drone_1_pos[0]:.2f}, {self.drone_1_pos[1]:.2f}, {self.drone_1_pos[2]:.2f})')
            self.get_logger().info(f'  iris_2 Posisi Awal : ({self.drone_2_pos[0]:.2f}, {self.drone_2_pos[1]:.2f}, {self.drone_2_pos[2]:.2f})')
            self.get_logger().info(f'Pusat Lingkaran: ({self.center_x:.1f}, {self.center_y:.1f}) | Radius: {self.radius:.1f}m | Speed: {self.nominal_speed:.2f}m/s')
            self.get_logger().info('Menunggu takeoff naik ke ketinggian Z=2.0m (3 detik)...')

    def publish_twist_1(self, vx_body, vy_body, wz=0.0):
        msg = Twist()
        msg.linear.x = float(vx_body)
        msg.linear.y = float(vy_body)
        msg.angular.z = float(wz)
        self.pub_vel_1.publish(msg)

    def publish_twist_2(self, vx_body, vy_body, wz=0.0):
        msg = Twist()
        msg.linear.x = float(vx_body)
        msg.linear.y = float(vy_body)
        msg.angular.z = float(wz)
        self.pub_vel_2.publish(msg)

    def control_loop(self):
        # Selalu update visualisasi track dan waypoint di RViz2
        self.publish_reference_markers()

        # 1. Menunggu Takeoff
        if self.state == 'wait_takeoff':
            self.publish_twist_1(0.0, 0.0, 0.0)
            self.publish_twist_2(0.0, 0.0, 0.0)
            self.takeoff_timer -= 1
            if self.takeoff_timer <= 0:
                self.state = 'tracking'
                self.theta_1 = 0.0
                self.theta_2 = 0.0
                self.get_logger().info('========================================================================')
                self.get_logger().info('  MULAI TRAJECTORY 2 DRONE (R=6.0m BERLAWANAN ARAH):')
                self.get_logger().info('    - iris_1 : Titik Barat (-6.0, 0.0) -> Arah CCW (Counter-Clockwise)')
                self.get_logger().info('    - iris_2 : Titik Timur (+6.0, 0.0) -> Arah CW (Clockwise)')
                self.get_logger().info('    - Pertemuan Head-on di Puncak Utara (0, +6) dan Puncak Selatan (0, -6)')
                self.get_logger().info(f'    - Total Putaran: {self.total_laps} Lap Penuh | Kecepatan: {self.nominal_speed:.2f} m/s')
                self.get_logger().info('========================================================================')
            return

        # 2. Tracking Trajectory Melingkar 2 Drone
        if self.state == 'tracking':
            self.step_count += 1

            total_target = self.total_laps * 2.0 * math.pi
            lead_rad = 0.45 / self.radius  # Lead target 0.45m di depan busur drone

            # -------------------------------------------------------------
            # A. DRONE 1 (iris_1): Mulai di Barat (-6, 0), Gerak CCW -> ke Timur (+6, 0)
            # -------------------------------------------------------------
            raw_angle_1 = math.atan2(float(self.drone_1_pos[1]), float(-self.drone_1_pos[0]))
            if raw_angle_1 < 0:
                raw_angle_1 += 2.0 * math.pi

            if not hasattr(self, 'theta_1_prog'):
                self.theta_1_prog = 0.0
                self.last_raw_1 = raw_angle_1

            d_theta_1 = (raw_angle_1 - self.last_raw_1)
            if d_theta_1 < -math.pi:
                d_theta_1 += 2.0 * math.pi
            elif d_theta_1 > math.pi:
                d_theta_1 -= 2.0 * math.pi
            self.last_raw_1 = raw_angle_1

            if d_theta_1 > 0:
                self.theta_1_prog += d_theta_1

            # Target di lingkaran menunggu posisi nyata drone dan hanya memimpin sejauh 0.45m
            self.theta_1 = min(total_target, self.theta_1_prog + lead_rad)

            ref_1_x = self.center_x - self.radius * math.cos(self.theta_1)
            ref_1_y = self.center_y + self.radius * math.sin(self.theta_1)

            e1_x = ref_1_x - self.drone_1_pos[0]
            e1_y = ref_1_y - self.drone_1_pos[1]
            dist_err_1 = math.sqrt(e1_x**2 + e1_y**2)

            if self.theta_1_prog < total_target:
                v1_ff_x = self.nominal_speed * math.sin(self.theta_1)
                v1_ff_y = self.nominal_speed * math.cos(self.theta_1)
            else:
                v1_ff_x = 0.0
                v1_ff_y = 0.0

            if self.theta_1_prog >= total_target and dist_err_1 < 0.15:
                self.publish_twist_1(0.0, 0.0, 0.0)
            else:
                # Gain proporsional 0.90 menarik drone seketika kembali ke keliling R=6.0m pasca menghindar
                v1_cmd_x = v1_ff_x + 0.90 * e1_x
                v1_cmd_y = v1_ff_y + 0.90 * e1_y
                spd_1 = math.sqrt(v1_cmd_x**2 + v1_cmd_y**2)
                if spd_1 > 0.95:
                    v1_cmd_x = (v1_cmd_x / spd_1) * 0.95
                    v1_cmd_y = (v1_cmd_y / spd_1) * 0.95

                cos_y1 = math.cos(self.drone_1_yaw)
                sin_y1 = math.sin(self.drone_1_yaw)
                v1_body_x = v1_cmd_x * cos_y1 + v1_cmd_y * sin_y1
                v1_body_y = -v1_cmd_x * sin_y1 + v1_cmd_y * cos_y1
                self.publish_twist_1(v1_body_x, v1_body_y, 0.0)

            # -------------------------------------------------------------
            # B. DRONE 2 (iris_2): Mulai di Timur (+6, 0), Gerak Berlawanan Arah -> ke Barat (-6, 0)
            # -------------------------------------------------------------
            raw_angle_2 = math.atan2(float(self.drone_2_pos[1]), float(self.drone_2_pos[0]))
            if raw_angle_2 < 0:
                raw_angle_2 += 2.0 * math.pi

            if not hasattr(self, 'theta_2_prog'):
                self.theta_2_prog = 0.0
                self.last_raw_2 = raw_angle_2

            d_theta_2 = (raw_angle_2 - self.last_raw_2)
            if d_theta_2 < -math.pi:
                d_theta_2 += 2.0 * math.pi
            elif d_theta_2 > math.pi:
                d_theta_2 -= 2.0 * math.pi
            self.last_raw_2 = raw_angle_2

            if d_theta_2 > 0:
                self.theta_2_prog += d_theta_2

            self.theta_2 = min(total_target, self.theta_2_prog + lead_rad)

            ref_2_x = self.center_x + self.radius * math.cos(self.theta_2)
            ref_2_y = self.center_y + self.radius * math.sin(self.theta_2)

            e2_x = ref_2_x - self.drone_2_pos[0]
            e2_y = ref_2_y - self.drone_2_pos[1]
            dist_err_2 = math.sqrt(e2_x**2 + e2_y**2)

            if self.theta_2_prog < total_target:
                v2_ff_x = -self.nominal_speed * math.sin(self.theta_2)
                v2_ff_y = self.nominal_speed * math.cos(self.theta_2)
            else:
                v2_ff_x = 0.0
                v2_ff_y = 0.0

            if self.theta_2_prog >= total_target and dist_err_2 < 0.15:
                self.publish_twist_2(0.0, 0.0, 0.0)
            else:
                v2_cmd_x = v2_ff_x + 0.90 * e2_x
                v2_cmd_y = v2_ff_y + 0.90 * e2_y
                spd_2 = math.sqrt(v2_cmd_x**2 + v2_cmd_y**2)
                if spd_2 > 0.95:
                    v2_cmd_x = (v2_cmd_x / spd_2) * 0.95
                    v2_cmd_y = (v2_cmd_y / spd_2) * 0.95

                cos_y2 = math.cos(self.drone_2_yaw)
                sin_y2 = math.sin(self.drone_2_yaw)
                v2_body_x = v2_cmd_x * cos_y2 + v2_cmd_y * sin_y2
                v2_body_y = -v2_cmd_x * sin_y2 + v2_cmd_y * cos_y2
                self.publish_twist_2(v2_body_x, v2_body_y, 0.0)

            # Jarak antar kedua drone secara real-time
            inter_drone_dist = float(np.linalg.norm(self.drone_1_pos[:2] - self.drone_2_pos[:2]))

            # Tampilkan telemetri setiap 1 detik
            if self.step_count % 10 == 0:
                lap1 = min(100.0, ((self.theta_1_prog % (2 * math.pi)) / (2 * math.pi)) * 100.0)
                lap2 = min(100.0, ((self.theta_2_prog % (2 * math.pi)) / (2 * math.pi)) * 100.0)
                if self.theta_1_prog >= total_target:
                    lap1 = 100.0
                if self.theta_2_prog >= total_target:
                    lap2 = 100.0
                self.get_logger().info(
                    f'[iris_1: {lap1:4.1f}% | iris_2: {lap2:4.1f}%] '
                    f'D1=({self.drone_1_pos[0]:4.1f},{self.drone_1_pos[1]:4.1f}) '
                    f'D2=({self.drone_2_pos[0]:4.1f},{self.drone_2_pos[1]:4.1f}) | '
                    f'Jarak Antar-Drone={inter_drone_dist:4.2f}m'
                )

            # Selesai setelah kedua drone menyelesaikan 2 putaran penuh (4 * pi rad)
            if self.theta_1_prog >= total_target and self.theta_2_prog >= total_target and dist_err_1 < 0.20 and dist_err_2 < 0.20:
                self.state = 'hovering'
                self.get_logger().info('========================================================================')
                self.get_logger().info('  MISI 2 DRONE SELESAI: Tepat 2 Putaran Selesai!')
                self.get_logger().info(f'  iris_1 HOVER Presisi di: ({self.drone_1_pos[0]:.2f}, {self.drone_1_pos[1]:.2f}, 2.00)m (Target: -6.00, 0.00)')
                self.get_logger().info(f'  iris_2 HOVER Presisi di: ({self.drone_2_pos[0]:.2f}, {self.drone_2_pos[1]:.2f}, 2.00)m (Target: +6.00, 0.00)')
                self.get_logger().info('  Tekan Ctrl+C untuk mengakhiri.')
                self.get_logger().info('========================================================================')
            return

        # 3. Hovering Pasca Selesai
        if self.state == 'hovering':
            self.publish_twist_1(0.0, 0.0, 0.0)
            self.publish_twist_2(0.0, 0.0, 0.0)

    def publish_reference_markers(self):
        """Visualisasi garis khayal lingkaran referensi, waypoint, dan setpoint real-time di RViz2."""
        ma = MarkerArray()
        now_msg = self.get_clock().now().to_msg()

        # 1. Ring Lingkaran Global Ideal R=6.0m di Z=2.0m (LINE_STRIP)
        m_ring = Marker()
        m_ring.header.frame_id = 'world'
        m_ring.header.stamp = now_msg
        m_ring.ns = 'global_trajectory_ring'
        m_ring.id = 0
        m_ring.type = Marker.LINE_STRIP
        m_ring.action = Marker.ADD
        m_ring.scale.x = 0.04  # Ketebalan garis (m)
        m_ring.color.r = 0.0
        m_ring.color.g = 0.90
        m_ring.color.b = 1.0
        m_ring.color.a = 0.85
        num_segments = 120
        for i in range(num_segments + 1):
            phi = (2.0 * math.pi * i) / num_segments
            px = self.center_x + self.radius * math.cos(phi)
            py = self.center_y + self.radius * math.sin(phi)
            m_ring.points.append(GeometryPoint(x=float(px), y=float(py), z=float(self.target_z)))
        ma.markers.append(m_ring)

        # 2. Titik Start/Finish Drone 1 (-6, 0, 2) [Bola Merah]
        m_d1_start = Marker()
        m_d1_start.header.frame_id = 'world'
        m_d1_start.header.stamp = now_msg
        m_d1_start.ns = 'd1_start'
        m_d1_start.id = 1
        m_d1_start.type = Marker.SPHERE
        m_d1_start.action = Marker.ADD
        m_d1_start.pose.position.x = -6.0
        m_d1_start.pose.position.y = 0.0
        m_d1_start.pose.position.z = float(self.target_z)
        m_d1_start.scale.x = 0.25
        m_d1_start.scale.y = 0.25
        m_d1_start.scale.z = 0.25
        m_d1_start.color.r = 1.0
        m_d1_start.color.g = 0.15
        m_d1_start.color.b = 0.15
        m_d1_start.color.a = 0.90
        ma.markers.append(m_d1_start)

        # 3. Titik Start/Finish Drone 2 (+6, 0, 2) [Bola Oranye]
        m_d2_start = Marker()
        m_d2_start.header.frame_id = 'world'
        m_d2_start.header.stamp = now_msg
        m_d2_start.ns = 'd2_start'
        m_d2_start.id = 2
        m_d2_start.type = Marker.SPHERE
        m_d2_start.action = Marker.ADD
        m_d2_start.pose.position.x = 6.0
        m_d2_start.pose.position.y = 0.0
        m_d2_start.pose.position.z = float(self.target_z)
        m_d2_start.scale.x = 0.25
        m_d2_start.scale.y = 0.25
        m_d2_start.scale.z = 0.25
        m_d2_start.color.r = 1.0
        m_d2_start.color.g = 0.60
        m_d2_start.color.b = 0.0
        m_d2_start.color.a = 0.90
        ma.markers.append(m_d2_start)

        # 4. Zona Berpapasan Utara (0, +6, 2) [Silinder Kuning]
        m_north = Marker()
        m_north.header.frame_id = 'world'
        m_north.header.stamp = now_msg
        m_north.ns = 'encounter_zones'
        m_north.id = 3
        m_north.type = Marker.CYLINDER
        m_north.action = Marker.ADD
        m_north.pose.position.x = 0.0
        m_north.pose.position.y = 6.0
        m_north.pose.position.z = float(self.target_z)
        m_north.scale.x = 0.35
        m_north.scale.y = 0.35
        m_north.scale.z = 0.06
        m_north.color.r = 1.0
        m_north.color.g = 0.95
        m_north.color.b = 0.1
        m_north.color.a = 0.75
        ma.markers.append(m_north)

        # 5. Zona Berpapasan Selatan (0, -6, 2)
        m_south = Marker()
        m_south.header.frame_id = 'world'
        m_south.header.stamp = now_msg
        m_south.ns = 'encounter_zones'
        m_south.id = 4
        m_south.type = Marker.CYLINDER
        m_south.action = Marker.ADD
        m_south.pose.position.x = 0.0
        m_south.pose.position.y = -6.0
        m_south.pose.position.z = float(self.target_z)
        m_south.scale.x = 0.35
        m_south.scale.y = 0.35
        m_south.scale.z = 0.06
        m_south.color.r = 1.0
        m_south.color.g = 0.95
        m_south.color.b = 0.1
        m_south.color.a = 0.75
        ma.markers.append(m_south)

        # 6. Real-time Setpoint Target Marker untuk Iris 1 & Iris 2
        ref_1_x = self.center_x - self.radius * math.cos(self.theta_1)
        ref_1_y = self.center_y + self.radius * math.sin(self.theta_1)
        m_sp1 = Marker()
        m_sp1.header.frame_id = 'world'
        m_sp1.header.stamp = now_msg
        m_sp1.ns = 'setpoints'
        m_sp1.id = 5
        m_sp1.type = Marker.SPHERE
        m_sp1.action = Marker.ADD
        m_sp1.pose.position.x = float(ref_1_x)
        m_sp1.pose.position.y = float(ref_1_y)
        m_sp1.pose.position.z = float(self.target_z)
        m_sp1.scale.x = 0.12
        m_sp1.scale.y = 0.12
        m_sp1.scale.z = 0.12
        m_sp1.color.r = 1.0
        m_sp1.color.g = 0.2
        m_sp1.color.b = 0.2
        m_sp1.color.a = 0.95
        ma.markers.append(m_sp1)

        ref_2_x = self.center_x + self.radius * math.cos(self.theta_2)
        ref_2_y = self.center_y + self.radius * math.sin(self.theta_2)
        m_sp2 = Marker()
        m_sp2.header.frame_id = 'world'
        m_sp2.header.stamp = now_msg
        m_sp2.ns = 'setpoints'
        m_sp2.id = 6
        m_sp2.type = Marker.SPHERE
        m_sp2.action = Marker.ADD
        m_sp2.pose.position.x = float(ref_2_x)
        m_sp2.pose.position.y = float(ref_2_y)
        m_sp2.pose.position.z = float(self.target_z)
        m_sp2.scale.x = 0.12
        m_sp2.scale.y = 0.12
        m_sp2.scale.z = 0.12
        m_sp2.color.r = 1.0
        m_sp2.color.g = 0.6
        m_sp2.color.b = 0.0
        m_sp2.color.a = 0.95
        ma.markers.append(m_sp2)

        self.pub_ref_markers.publish(ma)

def main(args=None):
    rclpy.init(args=args)
    node = DualSymmetricTrajectoryNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
