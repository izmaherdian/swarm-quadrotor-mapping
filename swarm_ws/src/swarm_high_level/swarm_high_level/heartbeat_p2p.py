import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from swarm_msgs.msg import Heartbeat
import sys

class HeartbeatP2P(Node):
    def __init__(self):
        super().__init__('heartbeat_p2p')
        
        # 1. Deklarasi Parameters
        self.declare_parameter('drone_id', 1)
        self.declare_parameter('num_drones', 3)
        
        self.drone_id = self.get_parameter('drone_id').value
        self.num_drones = self.get_parameter('num_drones').value
        
        self.get_logger().info(f"Mengaktifkan Heartbeat P2P untuk Drone_{self.drone_id} dari total {self.num_drones} drone.")
        
        # 2. State Lokal
        self.current_position = Point(x=0.0, y=0.0, z=0.0)
        self.neighbor_states = {} # format: {neighbor_id: {'position': Point, 'timestamp': Time, 'is_active': bool}}
        
        # Inisialisasi status tetangga
        for i in range(1, self.num_drones + 1):
            if i != self.drone_id:
                self.neighbor_states[i] = {
                    'position': Point(x=0.0, y=0.0, z=0.0),
                    'timestamp': self.get_clock().now(),
                    'is_active': False
                }

        # 3. Publisher & Subscriber Lokal
        # Subscribe ke odometry drone itu sendiri
        self.odom_sub = self.create_subscription(
            Odometry,
            f'/model/iris_{self.drone_id}/odometry',
            self.odom_callback,
            10
        )
        
        # Publish heartbeat drone itu sendiri
        self.heartbeat_pub = self.create_publisher(
            Heartbeat,
            f'/iris_{self.drone_id}/heartbeat',
            10
        )
        
        # 4. Subscriber P2P untuk Tetangga
        self.heartbeat_subs = {}
        for neighbor_id in self.neighbor_states.keys():
            self.create_p2p_subscriber(neighbor_id)
            
        # 5. Timers
        # Timer kirim heartbeat: 5Hz (0.2s)
        self.pub_timer = self.create_timer(0.2, self.publish_heartbeat)
        
        # Timer check timeout tetangga: 1Hz (1.0s)
        self.timeout_timer = self.create_timer(1.0, self.check_timeouts)

    def create_p2p_subscriber(self, neighbor_id):
        topic_name = f'/iris_{neighbor_id}/heartbeat'
        self.get_logger().info(f"Drone_{self.drone_id} mendengarkan {topic_name}")
        
        # Callback wrapper dengan parameter neighbor_id
        def cb(msg):
            self.heartbeat_callback(msg, neighbor_id)
            
        self.heartbeat_subs[neighbor_id] = self.create_subscription(
            Heartbeat,
            topic_name,
            cb,
            10
        )

    def odom_callback(self, msg):
        # Update posisi aktual dari sensor odometri
        self.current_position = msg.pose.pose.position

    def publish_heartbeat(self):
        msg = Heartbeat()
        msg.drone_id = self.drone_id
        msg.position = self.current_position
        msg.is_active = True
        self.heartbeat_pub.publish(msg)

    def heartbeat_callback(self, msg, neighbor_id):
        # Update data tetangga jika menerima sinyal heartbeat
        now = self.get_clock().now()
        was_active = self.neighbor_states[neighbor_id]['is_active']
        
        self.neighbor_states[neighbor_id] = {
            'position': msg.position,
            'timestamp': now,
            'is_active': msg.is_active
        }
        
        if not was_active and msg.is_active:
            self.get_logger().info(f"[P2P] Drone_{neighbor_id} terhubung ke Drone_{self.drone_id} (ONLINE)!")

    def check_timeouts(self):
        now = self.get_clock().now()
        for neighbor_id, state in self.neighbor_states.items():
            if state['is_active']:
                # Hitung waktu sejak heartbeat terakhir diterima
                delay = (now - state['timestamp']).nanoseconds * 1e-9
                if delay > 2.5: # Timeout 2.5 detik
                    state['is_active'] = False
                    self.get_logger().warn(
                        f"[P2P ALERT] Kehilangan koneksi ke Drone_{neighbor_id} "
                        f"(Timeout {delay:.1f}s)! Memicu FT-CC."
                    )

def main(args=None):
    rclpy.init(args=args)
    node = HeartbeatP2P()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
