import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import time

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
            
        self.get_logger().info("Menunggu sebentar agar koneksi ROS 2 stabil...")
        self.attempt = 0
        self.timer = self.create_timer(1.0, self.send_waypoints)

    def send_waypoints(self):
        self.attempt += 1
        self.get_logger().info(f'Mengirim target waypoint X=10 ke semua drone serentak... (Percobaan {self.attempt})')
        
        for i in range(1, 8):
            msg = PointStamped()
            msg.header.frame_id = 'world'
            msg.header.stamp = self.get_clock().now().to_msg()
            
            msg.point.x = 10.0
            msg.point.y = self.y_starts[i]
            msg.point.z = 2.0
            
            self.pubs[i].publish(msg)
            if self.attempt == 1:
                self.get_logger().info(f'-> Drone {i} ditugaskan ke (X: {msg.point.x}, Y: {msg.point.y})')
        
        if self.attempt >= 3:
            self.get_logger().info('Semua waypoint telah dikirim! Tekan Ctrl+C untuk keluar.')
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
