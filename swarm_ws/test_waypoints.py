import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

class WaypointTester(Node):
    def __init__(self):
        super().__init__('waypoint_tester')
        
        self.pubs = {}
        # Posisi Y awal masing-masing drone
        self.y_starts = {
            1: -6.0,
            2: -4.0,
            3: -2.0,
            4: 0.0,
            5: 2.0,
            6: 4.0,
            7: 6.0
        }
        
        # Buat publisher untuk tiap drone
        for i in range(1, 8):
            topic = f'/iris_{i}/waypoint'
            self.pubs[i] = self.create_publisher(PointStamped, topic, 10)
            
        self.get_logger().info("Menunggu 2 detik agar koneksi ROS 2 stabil...")
        self.timer = self.create_timer(2.0, self.send_waypoints)
        self.sent = False

    def send_waypoints(self):
        if self.sent:
            return
            
        self.get_logger().info("Mengirim target waypoint X=10 ke semua drone serentak...")
        
        for i in range(1, 8):
            msg = PointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'world'
            
            # Targetkan sejauh 10 meter ke depan (X = 10)
            msg.point.x = 10.0
            msg.point.y = self.y_starts[i]
            msg.point.z = 2.0
            
            self.pubs[i].publish(msg)
            self.get_logger().info(f"-> Drone {i} ditugaskan ke (X: 10.0, Y: {self.y_starts[i]})")
            
        self.sent = True
        self.get_logger().info("Semua waypoint telah dikirim! Tekan Ctrl+C untuk keluar.")
        self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = WaypointTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
