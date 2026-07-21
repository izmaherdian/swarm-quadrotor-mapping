import os
import rclpy
from rclpy.node import Node
import numpy as np
import onnxruntime as ort

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped, PoseStamped

class CollisionAvoidanceNode(Node):
    """
    ROS 2 Node for Real-time Obstacle Avoidance using a trained DRL (PPO) ONNX policy.
    Subscribes to Lidar scans and odometry, and outputs target coordinates to the low-level controller.
    """
    def __init__(self):
        super().__init__('collision_avoidance_node')

        # Parameters
        self.declare_parameter('model_path', '')
        self.declare_parameter('max_speed', 1.5)
        self.declare_parameter('target_z_height', 2.0)
        self.declare_parameter('dt', 0.1)
        self.declare_parameter('drone_id', 1)

        model_path = self.get_parameter('model_path').value
        self.max_speed = self.get_parameter('max_speed').value
        self.target_z_height = self.get_parameter('target_z_height').value
        self.dt = self.get_parameter('dt').value
        
        node_name = self.get_name()
        if '_' in node_name and node_name.split('_')[-1].isdigit():
            did = int(node_name.split('_')[-1])
        else:
            did = int(self.get_parameter('drone_id').value)
        self.drone_id = did

        # If model_path is empty, find the default ONNX model path
        if not model_path:
            from ament_index_python.packages import get_package_share_directory
            pkg_share = get_package_share_directory('swarm_mid_level')
            model_path = os.path.join(pkg_share, 'models', 'ppo_lidar_avoidance.onnx')

        self.get_logger().info(f"Loading ONNX Model for iris_{did} from: {model_path}")
        try:
            self.ort_session = ort.InferenceSession(model_path)
            self.get_logger().info("Successfully loaded ONNX policy model.")
        except Exception as e:
            self.get_logger().error(f"Failed to load ONNX model: {str(e)}")
            self.ort_session = None

        # State Variables
        self.current_pos = np.array([0.0, 0.0, 0.0])
        self.current_vel = np.array([0.0, 0.0])
        self.target_waypoint = np.array([0.0, 0.0])  # default starting target (X,Y)
        self.target_z_height = self.get_parameter('target_z_height').value
        self.lidar_ranges = np.ones(72, dtype=np.float32) * 10.0

        # Subscriptions
        self.lidar_sub = self.create_subscription(
            LaserScan,
            f'/iris_{did}/lidar_scan',
            self.lidar_callback,
            10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            f'/model/iris_{did}/odometry',
            self.odom_callback,
            10
        )
        self.waypoint_sub = self.create_subscription(
            PointStamped,
            f'/iris_{did}/waypoint',
            self.waypoint_callback,
            10
        )
        # Also accept PoseStamped on /iris_{did}/waypoint_pose for convenience
        self.waypoint_pose_sub = self.create_subscription(
            PoseStamped,
            f'/iris_{did}/waypoint_pose',
            self.waypoint_pose_callback,
            10
        )

        # Publisher to low-level controller
        self.pose_pub = self.create_publisher(
            PoseStamped,
            f'/iris_{did}/target_pose',
            10
        )

        # Timer to run inference at 10Hz (dt = 0.1)
        self.timer = self.create_timer(self.dt, self.control_loop)

    def lidar_callback(self, msg):
        # Read the 72 ranges and handle inf/nan values
        ranges = np.array(msg.ranges, dtype=np.float32)
        # Handle invalid values (replace inf and nan with max range)
        ranges = np.nan_to_num(ranges, nan=10.0, posinf=10.0, neginf=0.1)
        self.lidar_ranges = np.clip(ranges, 0.1, 10.0)

    def odom_callback(self, msg):
        # Update current position
        self.current_pos[0] = msg.pose.pose.position.x
        self.current_pos[1] = msg.pose.pose.position.y
        self.current_pos[2] = msg.pose.pose.position.z

        # Update current velocity (in world or body frame depending on odom config)
        self.current_vel[0] = msg.twist.twist.linear.x
        self.current_vel[1] = msg.twist.twist.linear.y

    def waypoint_callback(self, msg):
        # Update high-level target waypoint from PointStamped (X, Y, Z)
        self.target_waypoint[0] = msg.point.x
        self.target_waypoint[1] = msg.point.y
        self.target_z_height = msg.point.z
        self.get_logger().info(
            f"[AI] Waypoint baru: X={msg.point.x:.2f}, Y={msg.point.y:.2f}, Z={msg.point.z:.2f}"
        )

    def waypoint_pose_callback(self, msg):
        # Update high-level target waypoint from PoseStamped (X, Y, Z)
        self.target_waypoint[0] = msg.pose.position.x
        self.target_waypoint[1] = msg.pose.position.y
        self.target_z_height = msg.pose.position.z
        self.get_logger().info(
            f"[AI] Waypoint baru (PoseStamped): X={msg.pose.position.x:.2f}, Y={msg.pose.position.y:.2f}, Z={msg.pose.position.z:.2f}"
        )

    def control_loop(self):
        if self.ort_session is None:
            return

        # 1. Calculate relative target coordinates
        rel_target_raw = self.target_waypoint - self.current_pos[:2]

        # Normalize relative target to [-1, 1] range (model trained with max 10m)
        max_range = 10.0
        rel_target_norm = np.clip(rel_target_raw / max_range, -1.0, 1.0)

        # Normalize velocity (assume max vel ~5 m/s during training)
        vel_norm = np.clip(self.current_vel / 5.0, -1.0, 1.0)

        # 2. Build observation vector (shape: 1, 76)
        # Structure: 72 (Lidar, 0.1-10m) + 2 (rel_target normalized) + 2 (vel normalized)
        obs = np.concatenate([
            self.lidar_ranges / max_range,  # normalize lidar to [0, 1]
            rel_target_norm,
            vel_norm
        ]).astype(np.float32)
        obs = np.expand_dims(obs, axis=0) # Add batch dimension

        # 3. ONNX Model Inference
        try:
            ort_inputs = {self.ort_session.get_inputs()[0].name: obs}
            ort_outs = self.ort_session.run(None, ort_inputs)
            action = ort_outs[0][0] # Get first batch output [action_x, action_y]
        except Exception as e:
            self.get_logger().error(f"Inference error: {str(e)}")
            return

        # 4. Scale action output to velocity command
        action = np.clip(action, -1.0, 1.0)
        ref_vx = action[0] * self.max_speed
        ref_vy = action[1] * self.max_speed

        # Debug: log action dan posisi setiap 20 loop (~2 detik)
        self._debug_counter = getattr(self, '_debug_counter', 0) + 1
        if self._debug_counter % 20 == 0:
            dist = np.linalg.norm(rel_target_raw)
            self.get_logger().info(
                f"[AI] Pos=({self.current_pos[0]:.2f},{self.current_pos[1]:.2f}) "
                f"Target=({self.target_waypoint[0]:.1f},{self.target_waypoint[1]:.1f}) "
                f"Dist={dist:.2f}m | Action=({action[0]:.3f},{action[1]:.3f}) "
                f"→ Vx={ref_vx:.2f} Vy={ref_vy:.2f}"
            )

        # Fallback: jika AI output hampir nol tapi masih jauh dari target,
        # tambahkan dorongan langsung menuju target (blend AI + direct)
        dist_to_target = np.linalg.norm(rel_target_raw)
        if dist_to_target > 0.5:
            direction = rel_target_raw / dist_to_target
            ai_strength = abs(action[0]) + abs(action[1])
            blend = max(0.0, 0.8 - ai_strength)  # lebih agresif push ke target
            ref_vx += direction[0] * blend * self.max_speed
            ref_vy += direction[1] * blend * self.max_speed

        # Koreksi Overshoot: Jika target terlewati ke belakang (rel_target_raw[0] < -0.1)
        if rel_target_raw[0] < -0.1:
            ref_vx = 0.8 * (rel_target_raw[0] / dist_to_target * self.max_speed) + 0.2 * ref_vx
            ref_vy = 0.8 * (rel_target_raw[1] / dist_to_target * self.max_speed) + 0.2 * ref_vy

        # Stabilisasi Target Terdekat (Goal Reacher Mode):
        # Jika jarak ke target < 1.0m, transisi mulus dari AI ke kontrol proporsional murni agar berhenti tepat di target
        if dist_to_target < 1.0:
            alpha = dist_to_target  # linear blend factor
            v_direct_x = (rel_target_raw[0] / max(dist_to_target, 0.1)) * self.max_speed * dist_to_target
            v_direct_y = (rel_target_raw[1] / max(dist_to_target, 0.1)) * self.max_speed * dist_to_target
            ref_vx = alpha * ref_vx + (1.0 - alpha) * v_direct_x
            ref_vy = alpha * ref_vy + (1.0 - alpha) * v_direct_y

        # 5. Integrate velocity to output position commands
        target_pose = PoseStamped()
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.header.frame_id = 'world'
        
        # Calculate dynamic target position
        target_pose.pose.position.x = self.current_pos[0] + ref_vx * self.dt
        target_pose.pose.position.y = self.current_pos[1] + ref_vy * self.dt
        target_pose.pose.position.z = self.target_z_height # Hold target altitude
        
        target_pose.pose.orientation.w = 1.0 # default orientation

        # 6. Publish to low-level controller
        self.pose_pub.publish(target_pose)


def main(args=None):
    rclpy.init(args=args)
    node = CollisionAvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
