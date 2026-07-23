import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
import math

class SquareWaypointNode(Node):
    def __init__(self):
        super().__init__('square_waypoint_node')

        self.pub = self.create_publisher(PointStamped, '/iris_1/waypoint', 10)
        self.sub = self.create_subscription(Odometry, '/iris_1/odometry', self.odom_callback, 10)
        self.timer = self.create_timer(0.1, self.check_progress)

        self.z = 2.0
        self.drone_pos = (0.0, 0.0)
        self.start_y = None
        self.waypoints = []
        self.current_wp = 0
        self.state = 'wait_odom'
        self.takeoff_timer = 0
        self.delay_remaining = 0

        self.get_logger().info('SquareWaypointNode siap — menunggu odometry...')

    def build_waypoints(self, start_y):
        self.waypoints = [
            (5.0, start_y),
            (5.0, start_y + 5.0),
            (0.0, start_y + 5.0),
            (0.0, start_y),
        ]
        self.get_logger().info(f'Start Y={start_y:.1f}, waypoints: {self.waypoints}')

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.drone_pos = (x, y)

        if self.state == 'wait_odom':
            self.start_y = y
            self.build_waypoints(y)
            self.state = 'wait_takeoff'
            self.takeoff_timer = 30
            self.get_logger().info(f'Odometry diterima. Takeoff delay 30 tick (3s)...')

    def send_current_waypoint(self):
        if self.current_wp >= len(self.waypoints):
            return
        x, y = self.waypoints[self.current_wp]
        msg = PointStamped()
        msg.header.frame_id = 'world'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x = x
        msg.point.y = y
        msg.point.z = self.z
        self.pub.publish(msg)
        self.get_logger().info(f'Waypoint {self.current_wp+1}/4: ({x:.1f}, {y:.1f}, z={self.z:.1f})')

    def check_progress(self):
        if self.state == 'wait_takeoff':
            self.takeoff_timer -= 1
            if self.takeoff_timer <= 0:
                self.state = 'flying'
                self.send_current_waypoint()
            return

        if self.state == 'delay':
            self.delay_remaining -= 1
            if self.delay_remaining <= 0:
                self.state = 'flying'
                self.current_wp += 1
                if self.current_wp < len(self.waypoints):
                    self.send_current_waypoint()
                else:
                    self.state = 'done'
                    self.get_logger().info('=== Square selesai! Semua 4 waypoint tercapai ===')
            return

        if self.state != 'flying':
            return

        tx, ty = self.waypoints[self.current_wp]
        dx = self.drone_pos[0] - tx
        dy = self.drone_pos[1] - ty
        dist = math.sqrt(dx*dx + dy*dy)

        if dist < 0.2:
            self.get_logger().info(f'  -> Waypoint {self.current_wp+1}/4 tercapai! (dist={dist:.3f}m)')
            self.state = 'delay'
            self.delay_remaining = 10

def main():
    rclpy.init()
    node = SquareWaypointNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
