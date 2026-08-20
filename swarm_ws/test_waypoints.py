import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
import math
import sys

class SwarmWaypointTester(Node):
    def __init__(self):
        super().__init__('swarm_waypoint_tester')
        
        self.target_x = 10.0
        self.target_z = 2.0
        self.num_drones = 7
        
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
        
        self.pubs = {}
        self.subs = {}
        self.drone_pos = {}
        self.reached = {i: False for i in range(1, self.num_drones + 1)}
        
        for i in range(1, self.num_drones + 1):
            topic_wp = f'/iris_{i}/waypoint'
            self.pubs[i] = self.create_publisher(PointStamped, topic_wp, 10)
            
            topic_odom = f'/iris_{i}/odometry'
            self.subs[i] = self.create_subscription(
                Odometry,
                topic_odom,
                lambda msg, did=i: self.odom_callback(msg, did),
                10
            )
            
        self.state = 'wait_odom'
        self.takeoff_ticks = 30  # 3 detik delay takeoff
        self.step_count = 0
        
        self.timer = self.create_timer(0.1, self.loop)
        self.get_logger().info("SwarmWaypointTester siap — menunggu odometry ke-7 drone...")

    def odom_callback(self, msg, did):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        self.drone_pos[did] = (x, y, z)

    def send_all_waypoints(self):
        for i in range(1, self.num_drones + 1):
            msg = PointStamped()
            msg.header.frame_id = 'world'
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.point.x = self.target_x
            msg.point.y = self.y_starts[i]
            msg.point.z = self.target_z
            self.pubs[i].publish(msg)

    def loop(self):
        if len(self.drone_pos) < self.num_drones:
            return

        if self.state == 'wait_odom':
            self.get_logger().info(f"Semua {self.num_drones} drone terdeteksi! Takeoff delay 3s...")
            self.state = 'takeoff'
            return

        if self.state == 'takeoff':
            self.takeoff_ticks -= 1
            if self.takeoff_ticks <= 0:
                self.state = 'flying'
                self.get_logger().info(f"Target waypoint X={self.target_x:.1f}m dikirim ke seluruh formasi 7 drone!")
                self.send_all_waypoints()
            return

        if self.state == 'flying':
            # Publish waypoint secara berkala (tiap 5 tick = 0.5s)
            if self.step_count % 5 == 0:
                self.send_all_waypoints()

            self.step_count += 1

            # Hitung jarak tiap drone ke target
            all_done = True
            progress_str = []
            for i in range(1, self.num_drones + 1):
                px, py, pz = self.drone_pos[i]
                tx, ty, tz = self.target_x, self.y_starts[i], self.target_z
                dist = math.sqrt((px - tx)**2 + (py - ty)**2 + (pz - tz)**2)
                
                if dist < 0.3:
                    self.reached[i] = True
                else:
                    all_done = False
                    
                status_icon = "V" if self.reached[i] else f"{px:4.1f}m"
                progress_str.append(f"D{i}:{status_icon}")

            # Print progres setiap 1 detik
            if self.step_count % 10 == 0:
                num_reached = sum(self.reached.values())
                summary = " | ".join(progress_str)
                self.get_logger().info(f"[{num_reached}/{self.num_drones} Tercapai] Pos X: {summary}")

            if all_done:
                self.get_logger().info("========================================================================")
                self.get_logger().info("  MISI SELESAI: Seluruh 7 drone telah tiba di target X=10.0m!")
                self.get_logger().info("  Drone tetap HOVER di titik akhir. Tekan Ctrl+C untuk mengakhiri.")
                self.get_logger().info("========================================================================")
                self.state = 'hovering'
                return

        if self.state == 'hovering':
            # Terus kirim target posisi agar drone mempertahankan posisi hover di titik akhir
            self.step_count += 1
            if self.step_count % 10 == 0:
                self.send_all_waypoints()

def main(args=None):
    rclpy.init(args=args)
    node = SwarmWaypointTester()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
