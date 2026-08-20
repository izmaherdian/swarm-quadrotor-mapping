#!/usr/bin/env python3
"""
test_trajectory.py — Adaptive Arc-Length Circular Trajectory Tracking for Mapping

Dirancang khusus untuk misi pemetaan (Mapping):
  - Kecepatan pemetaan tenang & stabil: v = 0.50 m/s
  - Adaptive Carrot Progress: Ketika drone menghindar rintangan (deviasi jalur),
    laju target maya melambat/menunggu (tidak lari ke depan).
  - Menjamin drone mengitari rintangan secara lokal dan melanjutkan pemindaian
    seluruh keliling lingkaran tanpa memotong jalur (No Shortcut).
  - Total: 2 Putaran Penuh (2 Lap) -> Kunci Hovering di akhir.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import numpy as np
import sys

class AdaptiveMappingTrajectoryNode(Node):
    def __init__(self):
        super().__init__('adaptive_mapping_trajectory_node')

        # Publisher cmd_vel & Subscriber odometry
        self.pub_cmd_vel = self.create_publisher(Twist, '/iris_1/cmd_vel', 10)
        self.sub_odom = self.create_subscription(Odometry, '/iris_1/odometry', self.odom_callback, 10)

        # Loop timer pada 10 Hz (dt = 0.1 detik)
        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.control_loop)

        # Parameter Geometri Trajectory Lingkaran untuk Mapping
        self.radius = 3.0                                  # meter
        self.nominal_speed = 0.50                          # m/s (kecepatan stabil untuk mapping LiDAR)
        self.omega = self.nominal_speed / self.radius      # ~0.1667 rad/s (periode ~37.7s per lap)
        self.total_laps = 2                                # jumlah putaran
        self.target_z = 2.0                                # meter

        # State Variables
        self.drone_pos = np.array([0.0, -6.0, 0.0], dtype=np.float32)
        self.drone_yaw = 0.0
        self.center_x = 3.0
        self.center_y = -6.0
        self.progress_theta = 0.0                          # Akumulasi sudut progres lingkaran (radian)
        self.state = 'wait_odom'
        self.takeoff_timer = 0
        self.step_count = 0

        self.get_logger().info('AdaptiveMappingTrajectoryNode siap — menunggu data odometry /iris_1/odometry...')

    def euler_from_quaternion(self, x, y, z, w):
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        return math.atan2(t3, t4)

    def odom_callback(self, msg):
        self.drone_pos[0] = msg.pose.pose.position.x
        self.drone_pos[1] = msg.pose.pose.position.y
        self.drone_pos[2] = msg.pose.pose.position.z

        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.drone_yaw = self.euler_from_quaternion(qx, qy, qz, qw)

        if self.state == 'wait_odom':
            self.center_y = self.drone_pos[1]
            self.center_x = self.drone_pos[0] + self.radius
            self.state = 'wait_takeoff'
            self.takeoff_timer = 30  # 3 detik (30 tick @ 10Hz)
            self.get_logger().info(f'Odometry diterima: Posisi Awal=({self.drone_pos[0]:.2f}, {self.drone_pos[1]:.2f}, {self.drone_pos[2]:.2f})')
            self.get_logger().info(f'Pusat Lingkaran: Xc={self.center_x:.2f}, Yc={self.center_y:.2f} | Radius={self.radius:.1f}m | Speed={self.nominal_speed:.2f}m/s')
            self.get_logger().info('Menunggu takeoff naik ke ketinggian Z=2.0m (3 detik)...')

    def publish_twist(self, vx_body, vy_body, wz):
        msg = Twist()
        msg.linear.x = float(vx_body)
        msg.linear.y = float(vy_body)
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(wz)
        self.pub_cmd_vel.publish(msg)

    def control_loop(self):
        # 1. Menunggu Takeoff
        if self.state == 'wait_takeoff':
            self.publish_twist(0.0, 0.0, 0.0)
            self.takeoff_timer -= 1
            if self.takeoff_timer <= 0:
                self.state = 'tracking'
                self.progress_theta = 0.0
                self.get_logger().info('========================================================================')
                self.get_logger().info(f'  MULAI MAPPING TRAJECTORY (ADAPTIVE PROGRESS): Radius={self.radius}m, Total={self.total_laps} Lap')
                self.get_logger().info('========================================================================')
            return

        # 2. Adaptive Arc-Length Trajectory Tracking
        if self.state == 'tracking':
            self.step_count += 1

            # Titik target referensi pada lingkaran saat ini
            ref_x = self.center_x - self.radius * math.cos(self.progress_theta)
            ref_y = self.center_y + self.radius * math.sin(self.progress_theta)

            # Hitung error jarak drone terhadap target referensi
            e_x = ref_x - self.drone_pos[0]
            e_y = ref_y - self.drone_pos[1]
            tracking_err = math.sqrt(e_x**2 + e_y**2)

            # Adaptive Progress Rate:
            # Jika drone sedang meliuk menghindar rintangan (tracking_err membesar),
            # laju target maya diperlambat agar drone tidak tertinggal dan tidak memotong jalur.
            if tracking_err > 0.4:
                advance_scale = max(0.15, 0.4 / tracking_err)
            else:
                advance_scale = 1.0

            # Majukan sudut progres lingkaran
            d_theta = (self.omega * advance_scale) * self.dt
            self.progress_theta += d_theta

            # Kecepatan feedforward tangensial lingkaran
            v_ff_x = (self.nominal_speed * advance_scale) * math.sin(self.progress_theta)
            v_ff_y = (self.nominal_speed * advance_scale) * math.cos(self.progress_theta)

            # Proportional Feedback lembut untuk menarik drone ke lingkaran
            K_pos = 0.75
            v_cmd_world_x = v_ff_x + K_pos * e_x
            v_cmd_world_y = v_ff_y + K_pos * e_y

            # Batasi kecepatan maksimum dunia agar data pemetaan tajam dan halus
            cmd_speed = math.sqrt(v_cmd_world_x**2 + v_cmd_world_y**2)
            MAX_CMD_SPEED = 0.85
            if cmd_speed > MAX_CMD_SPEED:
                v_cmd_world_x = (v_cmd_world_x / cmd_speed) * MAX_CMD_SPEED
                v_cmd_world_y = (v_cmd_world_y / cmd_speed) * MAX_CMD_SPEED

            # Konversi perintah kecepatan dunia (World) ke koordinat tubuh drone (Body)
            cos_yaw = math.cos(self.drone_yaw)
            sin_yaw = math.sin(self.drone_yaw)
            v_body_x = v_cmd_world_x * cos_yaw + v_cmd_world_y * sin_yaw
            v_body_y = -v_cmd_world_x * sin_yaw + v_cmd_world_y * cos_yaw

            # Publikasikan ke /iris_1/cmd_vel (Yaw heading otomatis diselaraskan mulus oleh mid-level)
            self.publish_twist(v_body_x, v_body_y, 0.0)

            # Hitung progres putaran aktual
            total_target_rad = self.total_laps * 2.0 * math.pi
            current_lap = int(self.progress_theta // (2.0 * math.pi)) + 1
            lap_progress = ((self.progress_theta % (2.0 * math.pi)) / (2.0 * math.pi)) * 100.0

            # Tampilkan telemetri setiap 1 detik
            if self.step_count % 10 == 0:
                self.get_logger().info(
                    f'[Lap {min(current_lap, self.total_laps)}/{self.total_laps} - {lap_progress:4.1f}%] '
                    f'Pos: ({self.drone_pos[0]:4.1f}m, {self.drone_pos[1]:4.1f}m) | '
                    f'Target: ({ref_x:4.1f}m, {ref_y:4.1f}m) | '
                    f'Deviasi: {tracking_err:4.2f}m | Scale: {advance_scale:.2f}'
                )

            # Selesai setelah 2 putaran penuh (4 * pi radian)
            if self.progress_theta >= total_target_rad:
                self.state = 'hovering'
                self.get_logger().info('========================================================================')
                self.get_logger().info(f'  MISI TRAJECTORY SELESAI: {self.total_laps} Putaran Penuh Selesai Dipetakan!')
                self.get_logger().info(f'  Drone tetap HOVER diam di titik awal ({self.center_x - self.radius:.2f}, {self.center_y:.2f}, 2.00)m.')
                self.get_logger().info('  Tekan Ctrl+C untuk mengakhiri.')
                self.get_logger().info('========================================================================')
            return

        # 3. Hovering Pasca Selesai — Mengunci Posisi dan Sudut Heading Terakhir
        if self.state == 'hovering':
            self.publish_twist(0.0, 0.0, 0.0)

def main(args=None):
    rclpy.init(args=args)
    node = AdaptiveMappingTrajectoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
