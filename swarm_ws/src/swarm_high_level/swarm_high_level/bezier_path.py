import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped, Point
import numpy as np
import math

class BezierPathNode(Node):
    def __init__(self):
        super().__init__('bezier_path')
        
        # 1. Deklarasi Parameter ID
        self.declare_parameter('drone_id', 1)
        self.drone_id = self.get_parameter('drone_id').value
        
        self.get_logger().info(f"Node Bezier Path aktif untuk Drone_{self.drone_id}.")
        
        # 2. State Lokal
        self.current_pos = np.array([0.0, 0.0]) # [x, y]
        self.current_yaw = 0.0                  # Radian
        self.target_centroid = np.array([0.0, 0.0])
        self.has_centroid = False
        
        # 3. Subscriber
        # Subscribe ke odometry drone untuk posisi & heading aktual
        self.odom_sub = self.create_subscription(
            Odometry,
            f'/model/iris_{self.drone_id}/odometry',
            self.odom_callback,
            10
        )
        
        # Subscribe ke target Centroid Voronoi
        self.centroid_sub = self.create_subscription(
            PointStamped,
            f'/iris_{self.drone_id}/voronoi_centroid',
            self.centroid_callback,
            10
        )
        
        # 4. Publisher Waypoint Halus
        self.waypoint_pub = self.create_publisher(
            PointStamped,
            f'/iris_{self.drone_id}/waypoint',
            10
        )
        
        # 5. Timer Interpolasi Lintasan Bézier: 10Hz (0.1s)
        self.path_timer = self.create_timer(0.1, self.generate_bezier_waypoint)

    def odom_callback(self, msg):
        # Ambil posisi aktual (X, Y)
        self.current_pos[0] = msg.pose.pose.position.x
        self.current_pos[1] = msg.pose.pose.position.y
        
        # Ambil orientasi Quaternion dan konversi ke sudut Yaw
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        # Rumus konversi Yaw
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def centroid_callback(self, msg):
        # Ambil target Centroid Voronoi terbaru
        self.target_centroid[0] = msg.point.x
        self.target_centroid[1] = msg.point.y
        self.has_centroid = True

    def generate_bezier_waypoint(self):
        if not self.has_centroid:
            return
            
        # 1. P0: Titik Awal (posisi drone saat ini)
        P0 = self.current_pos
        
        # 2. P2: Titik Akhir (target Centroid Voronoi)
        P2 = self.target_centroid
        
        # 3. P1: Control Point untuk menjamin kehalusan kemiringan (G1 Continuity)
        # Ekspansi P0 searah dengan heading (Yaw) saat ini sejauh d meter
        d = 0.5 # Jarak ekspansi control point
        heading_dir = np.array([math.cos(self.current_yaw), math.sin(self.current_yaw)])
        P1 = P0 + d * heading_dir
        
        # 4. Persamaan Quadratic Bezier:
        # B(t) = (1-t)^2 * P0 + 2*(1-t)*t * P1 + t^2 * P2
        # Ambil t kecil untuk interpolasi waypoint langkah berikutnya
        t = 0.2 # langkah interpolasi (0.0 = di P0, 1.0 = di P2)
        
        B_t = (1 - t)**2 * P0 + 2 * (1 - t) * t * P1 + t**2 * P2
        
        # 5. Publikasikan target waypoint ter-filter
        waypoint_msg = PointStamped()
        waypoint_msg.header.stamp = self.get_clock().now().to_msg()
        waypoint_msg.header.frame_id = 'world'
        waypoint_msg.point.x = float(B_t[0])
        waypoint_msg.point.y = float(B_t[1])
        waypoint_msg.point.z = 2.0 # Target ketinggian jelajah (Z=2.0m)
        self.waypoint_pub.publish(waypoint_msg)
        
        # Log debug sesekali
        self._debug_count = getattr(self, '_debug_count', 0) + 1
        if self._debug_count % 30 == 0:
            dist = np.linalg.norm(P2 - P0)
            self.get_logger().info(
                f"Drone_{self.drone_id} heading: {math.degrees(self.current_yaw):.1f}° | "
                f"Centroid Dist: {dist:.2f}m | Smoothed Waypoint X: {B_t[0]:.2f}, Y: {B_t[1]:.2f}"
            )

def main(args=None):
    rclpy.init(args=args)
    node = BezierPathNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
