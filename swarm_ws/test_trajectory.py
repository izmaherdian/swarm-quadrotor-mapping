#!/usr/bin/env python3
"""
test_trajectory.py — Single-Agent Velocity-Based Circular Trajectory Node

Menguji pelacakan lintasan melingkar (Circular Trajectory Tracking) berbasis
kecepatan linier (v) dan kecepatan putar haluan (omega) pada quadrotor iris_1:
  - Radius Lingkaran (R)      : 3.0 meter
  - Periode per Putaran (T)   : 26.0 detik
  - Kecepatan Maju (v_x)      : 0.725 m/s (2 * pi * R / T)
  - Kecepatan Putar (yaw_rate): 0.2417 rad/s (2 * pi / T)
  - Total Putaran             : 2 Lap (52.0 detik)
  - Pasca Selesai             : Hovering diam di posisi akhir
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import sys

class VelocityTrajectoryTrackingNode(Node):
    def __init__(self):
        super().__init__('velocity_trajectory_tracking_node')

        # Publisher cmd_vel & Subscriber odometry
        self.pub_cmd_vel = self.create_publisher(Twist, '/iris_1/cmd_vel', 10)
        self.sub_odom = self.create_subscription(Odometry, '/iris_1/odometry', self.odom_callback, 10)

        # Loop timer pada 10 Hz (dt = 0.1 detik)
        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.control_loop)

        # Parameter Geometri Trajectory Lingkaran
        self.radius = 3.0                                  # meter
        self.period = 26.0                                 # detik per 1 putaran (T)
        self.total_laps = 2                                # jumlah putaran
        self.target_speed = (2.0 * math.pi * self.radius) / self.period  # ~0.725 m/s
        self.target_yaw_rate = (2.0 * math.pi) / self.period             # ~0.2417 rad/s (~13.85 deg/s)

        # State Variables
        self.drone_pos = [0.0, 0.0, 0.0]
        self.drone_yaw = 0.0
        self.state = 'wait_odom'
        self.takeoff_timer = 0
        self.trajectory_time = 0.0
        self.step_count = 0

        self.get_logger().info('VelocityTrajectoryTrackingNode siap — menunggu data odometry /iris_1/odometry...')

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
            self.state = 'wait_takeoff'
            self.takeoff_timer = 30  # 3 detik (30 tick @ 10Hz)
            self.get_logger().info(f'Odometry diterima: Posisi Awal=({self.drone_pos[0]:.2f}, {self.drone_pos[1]:.2f}, {self.drone_pos[2]:.2f})')
            self.get_logger().info(f'Parameter Lingkaran: Radius={self.radius:.1f}m, Periode={self.period:.1f}s')
            self.get_logger().info(f'Perintah Kecepatan: Maju={self.target_speed:.3f} m/s, Yaw Rate={math.degrees(self.target_yaw_rate):.2f}°/s')
            self.get_logger().info('Menunggu takeoff naik ke ketinggian Z=2.0m (3 detik)...')

    def publish_twist(self, vx, vy, wz):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
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
                self.trajectory_time = 0.0
                self.get_logger().info('========================================================================')
                self.get_logger().info(f'  MULAI TRACKING LINGKARAN (VELOCITY MODE): Total={self.total_laps} Putaran ({self.total_laps * self.period:.1f}s)')
                self.get_logger().info('========================================================================')
            return

        # 2. Tracking Melingkar Mulus Berbasis Kecepatan
        if self.state == 'tracking':
            self.trajectory_time += self.dt
            self.step_count += 1

            # Publikasikan kecepatan maju konstan dan kecepatan belok konstan
            self.publish_twist(self.target_speed, 0.0, self.target_yaw_rate)

            # Hitung progres putaran
            current_lap = int(self.trajectory_time // self.period) + 1
            lap_progress = ((self.trajectory_time % self.period) / self.period) * 100.0

            # Tampilkan telemetri setiap 1 detik
            if self.step_count % 10 == 0:
                self.get_logger().info(
                    f'[Lap {min(current_lap, self.total_laps)}/{self.total_laps} - {lap_progress:4.1f}%] '
                    f'Pos: (X:{self.drone_pos[0]:4.1f}m, Y:{self.drone_pos[1]:4.1f}m, Z:{self.drone_pos[2]:4.1f}m) | '
                    f'Heading: {math.degrees(self.drone_yaw):5.1f}° | '
                    f'Cmd: (v={self.target_speed:.2f}m/s, w={math.degrees(self.target_yaw_rate):.1f}°/s)'
                )

            # Selesai setelah 2 putaran
            total_duration = self.total_laps * self.period
            if self.trajectory_time >= total_duration:
                self.state = 'hovering'
                self.get_logger().info('========================================================================')
                self.get_logger().info(f'  MISI TRAJECTORY SELESAI: {self.total_laps} Putaran Penuh Berhasil Diselesaikan!')
                self.get_logger().info(f'  Drone tetap HOVER diam di posisi akhir ({self.drone_pos[0]:.2f}, {self.drone_pos[1]:.2f}, {self.drone_pos[2]:.2f})m.')
                self.get_logger().info('  Tekan Ctrl+C untuk mengakhiri.')
                self.get_logger().info('========================================================================')
            return

        # 3. Hovering Pasca Selesai
        if self.state == 'hovering':
            self.step_count += 1
            if self.step_count % 10 == 0:
                self.publish_twist(0.0, 0.0, 0.0)

def main(args=None):
    rclpy.init(args=args)
    node = VelocityTrajectoryTrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
