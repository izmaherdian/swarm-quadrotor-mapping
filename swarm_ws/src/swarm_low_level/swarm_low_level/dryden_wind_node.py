#!/usr/bin/env python3
# ==============================================================================
#   DRYDEN WIND TURBULENCE GENERATOR NODE (ROS 2 & GAZEBO HARMONIC)
# ==============================================================================
#   Formulasi Matematis (MIL-F-8785C Discrete Stochastic Filter):
#     w_k = alpha_w * w_{k-1} + beta_w * eta_k
#     alpha_w = exp(-dt / tau_w)
#     beta_w  = sigma_w * sqrt(1 - alpha_w^2)
#     eta_k   ~ N(0, I)
# ==============================================================================

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped, Point as GeometryPoint
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import numpy as np
import math

try:
    from gz.transport import Node as GzNode
    from gz.msgs.wind_pb2 import Wind as GzWind
    GZ_TRANSPORT_AVAILABLE = True
except ImportError:
    GZ_TRANSPORT_AVAILABLE = False


class DrydenWindNode(Node):
    def __init__(self):
        super().__init__('dryden_wind_node')

        # Parameter Konfigurasi Dryden Turbulence
        self.declare_parameter('sigma_wind', 2.5)       # Standar Deviasi Intensitas Angin (m/s atau N)
        self.declare_parameter('tau_wind', 0.5)          # Time constant korelasi temporal (s)
        self.declare_parameter('seed', 42)               # Random seed identik untuk pengujian adil
        self.declare_parameter('enable_gust', True)      # Aktifkan hembusan kejut (gust step)
        self.declare_parameter('gust_time', 5.0)         # Waktu mulai hembusan kejut (s)
        self.declare_parameter('gust_magnitude', -2.0)   # Besaran hembusan kejut (m/s)

        self.sigma_wind = float(self.get_parameter('sigma_wind').value)
        self.tau_wind = float(self.get_parameter('tau_wind').value)
        self.seed = int(self.get_parameter('seed').value)
        self.enable_gust = bool(self.get_parameter('enable_gust').value)
        self.gust_time = float(self.get_parameter('gust_time').value)
        self.gust_magnitude = float(self.get_parameter('gust_magnitude').value)

        # Inisialisasi Pseudo-random Generator dengan fixed seed
        np.random.seed(self.seed)

        self.dt = 0.02  # 50 Hz update rate
        self.alpha_w = float(np.exp(-self.dt / self.tau_wind))
        self.beta_w = float(self.sigma_wind * np.sqrt(1.0 - self.alpha_w**2))
        self.wind_state = np.zeros(3, dtype=np.float64)

        # Publisher ROS 2 untuk Telemetri & Visualisasi
        self.pub_wind_vector = self.create_publisher(Vector3Stamped, '/swarm/wind_disturbance', 10)
        self.pub_wind_marker = self.create_publisher(MarkerArray, '/swarm/wind_marker', 10)

        # Publisher Gazebo Native Transport
        self.gz_node = None
        self.gz_pub = None
        if GZ_TRANSPORT_AVAILABLE:
            try:
                self.gz_node = GzNode()
                self.gz_pub = self.gz_node.advertise('/world/swarm_world/wind', GzWind)
                self.get_logger().info('🌪️ [DRYDEN WIND] Gazebo Transport Publisher `/world/swarm_world/wind` Siap!')
            except Exception as e:
                self.get_logger().warning(f'Gagal inisialisasi Gazebo Transport: {e}')

        self.start_time = None
        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.get_logger().info(
            f'🌪️ [DRYDEN WIND GENERATOR ACTIVE] σ={self.sigma_wind:.2f}, τ={self.tau_wind:.2f}s, '
            f'Seed={self.seed}, Gust={self.enable_gust} (t>={self.gust_time:.1f}s, mag={self.gust_magnitude:.1f}m/s)'
        )

    def timer_callback(self):
        now = self.get_clock().now()
        now_sec = now.nanoseconds * 1e-9

        if self.start_time is None:
            self.start_time = now_sec

        t_sim = now_sec - self.start_time

        # 1. Update Dryden Stochastic Filter
        eta = np.random.randn(3)
        self.wind_state = self.alpha_w * self.wind_state + self.beta_w * eta

        wx = float(self.wind_state[0])
        wy = float(self.wind_state[1])
        wz = float(self.wind_state[2] * 0.4)  # Vertikal biasanya lebih teredam

        # 2. Injeksi Gust Step
        if self.enable_gust and t_sim >= self.gust_time:
            wx += self.gust_magnitude
            wy += self.gust_magnitude
            wz += self.gust_magnitude * 0.3

        # 3. Publish ke Gazebo Harmonic Physics
        if self.gz_pub is not None:
            try:
                msg_gz = GzWind()
                msg_gz.enable_wind = True
                msg_gz.linear_velocity.x = float(wx)
                msg_gz.linear_velocity.y = float(wy)
                msg_gz.linear_velocity.z = float(wz)
                self.gz_pub.publish(msg_gz)
            except Exception as e:
                pass

        # 4. Publish ke ROS 2 Topic
        msg_ros = Vector3Stamped()
        msg_ros.header.stamp = now.to_msg()
        msg_ros.header.frame_id = 'world'
        msg_ros.vector.x = float(wx)
        msg_ros.vector.y = float(wy)
        msg_ros.vector.z = float(wz)
        self.pub_wind_vector.publish(msg_ros)

        # 5. Visualisasi Panah Angin 3D di RViz2 (Floating di atas arena Z=6.0m)
        ma = MarkerArray()
        m_wind = Marker()
        m_wind.header.frame_id = 'world'
        m_wind.header.stamp = now.to_msg()
        m_wind.ns = 'dryden_wind_vector'
        m_wind.id = 999
        m_wind.type = Marker.ARROW
        m_wind.action = Marker.ADD

        # Panah dimulai di (0, 0, 6.0) mengarah sesuai vektor angin
        p_start = GeometryPoint(x=0.0, y=0.0, z=6.0)
        p_end = GeometryPoint(x=float(wx * 0.8), y=float(wy * 0.8), z=float(6.0 + wz * 0.4))
        m_wind.points = [p_start, p_end]
        m_wind.scale.x = 0.12  # Diameter batang panah
        m_wind.scale.y = 0.28  # Diameter kepala panah
        m_wind.scale.z = 0.35  # Panjang kepala panah
        m_wind.color = ColorRGBA(r=0.2, g=0.85, b=1.0, a=0.85)  # Cyan bercahaya
        ma.markers.append(m_wind)

        self.pub_wind_marker.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = DrydenWindNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
