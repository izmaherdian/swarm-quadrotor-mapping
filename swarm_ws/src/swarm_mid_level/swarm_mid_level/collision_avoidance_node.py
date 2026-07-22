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
        self.declare_parameter('max_speed', 2.5)
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

        # AI State & Config
        self.current_pos = np.array([0.0, 0.0, 0.0])
        self.current_vel = np.array([0.0, 0.0])
        self.current_orientation = None # to store quaternion
        self.target_waypoint = None  # Will set to initial spawn position on first odom
        self.target_z_height = self.get_parameter('target_z_height').value
        self.lidar_ranges = np.ones(360, dtype=np.float32) * 10.0
        self.waypoint_received = False

        # Subscriptions
        self.lidar_sub = self.create_subscription(
            LaserScan,
            f'/iris_{did}/lidar_scan',
            self.lidar_callback,
            10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            f'/iris_{did}/odometry',
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
        # Read the ranges and handle inf/nan values
        ranges = np.array(msg.ranges, dtype=np.float32)
        
        # Ground filtering: if drone is pitched, the 2D lidar hits the ground.
        # We calculate the distance to ground for each ray and ignore it if it matches.
        if self.current_orientation is not None and self.current_pos[2] > 0.5:
            # Simple quaternion to pitch approximation (since we only care about forward pitch mostly)
            # Roll and pitch from quaternion
            q = self.current_orientation
            sinp = 2.0 * (q.w * q.y - q.z * q.x)
            pitch = np.arcsin(np.clip(sinp, -1.0, 1.0))
            sinr = 2.0 * (q.w * q.x + q.y * q.z)
            cosr = 1.0 - 2.0 * (q.x**2 + q.y**2)
            roll = np.arctan2(sinr, cosr)
            
            # The lidar rays are 360 degrees from -pi to pi
            angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
            
            # Vector in body frame: [cos(theta), sin(theta), 0]
            # When rotated by pitch (y-axis) and roll (x-axis), the Z component in world is:
            # z_world = -sin(pitch)*cos(theta) + sin(roll)*cos(pitch)*sin(theta)
            ray_z_world = -np.sin(pitch) * np.cos(angles) + np.sin(roll) * np.cos(pitch) * np.sin(angles)
            
            # If the ray is pointing downwards
            down_mask = ray_z_world < -0.05
            
            # Expected distance to hit the ground
            expected_ground_dist = np.zeros_like(ranges)
            expected_ground_dist[down_mask] = self.current_pos[2] / (-ray_z_world[down_mask])
            
            # If the lidar reading is close to the expected ground hit (within 0.8 meters)
            ground_hit_mask = down_mask & (ranges > (expected_ground_dist - 0.8)) & (ranges < (expected_ground_dist + 0.8))
            
            # Replace ground hits with max range
            ranges[ground_hit_mask] = 10.0

        # INFLATION RADIUS: Kurangi 25cm (0.25m) dari pembacaan sensor
        # agar drone menganggap halangan lebih dekat, menghindari benturan baling-baling.
        ranges = ranges - 0.25
        
        # Handle invalid values (replace inf and nan with max range)
        ranges = np.nan_to_num(ranges, nan=10.0, posinf=10.0, neginf=0.1)
        self.lidar_ranges = np.clip(ranges, 0.1, 10.0)

    def odom_callback(self, msg):
        # Update current position
        self.current_pos[0] = msg.pose.pose.position.x
        self.current_pos[1] = msg.pose.pose.position.y
        self.current_pos[2] = msg.pose.pose.position.z
        
        # Store orientation for ground filtering
        self.current_orientation = msg.pose.pose.orientation

        # Set default initial target waypoint to initial spawn location if no external waypoint received
        if self.target_waypoint is None:
            self.target_waypoint = np.array([self.current_pos[0], self.current_pos[1]])
            self.get_logger().info(f"[AI] Target awal di-set ke lokasi spawn: X={self.target_waypoint[0]:.2f}, Y={self.target_waypoint[1]:.2f}")

        # Update current velocity (in world or body frame depending on odom config)
        self.current_vel[0] = msg.twist.twist.linear.x
        self.current_vel[1] = msg.twist.twist.linear.y

    def waypoint_callback(self, msg):
        # Update high-level target waypoint from PointStamped (X, Y, Z)
        self.target_waypoint = np.array([msg.point.x, msg.point.y])
        self.target_z_height = msg.point.z
        self.waypoint_received = True
        self.get_logger().info(
            f"[AI] Waypoint baru: X={msg.point.x:.2f}, Y={msg.point.y:.2f}, Z={msg.point.z:.2f}"
        )

    def waypoint_pose_callback(self, msg):
        # Update high-level target waypoint from PoseStamped (X, Y, Z)
        self.target_waypoint = np.array([msg.pose.position.x, msg.pose.position.y])
        self.target_z_height = msg.pose.position.z
        self.waypoint_received = True
        self.get_logger().info(
            f"[AI] Waypoint baru (PoseStamped): X={msg.pose.position.x:.2f}, Y={msg.pose.position.y:.2f}, Z={msg.pose.position.z:.2f}"
        )

    def control_loop(self):
        if self.ort_session is None or self.target_waypoint is None:
            return

        # 0. Takeoff phase check: hold position until Z >= 1.5m
        if self.current_pos[2] < 1.5 and not self.waypoint_received:
            # Publish straight takeoff pose
            target_pose = PoseStamped()
            target_pose.header.stamp = self.get_clock().now().to_msg()
            target_pose.header.frame_id = 'world'
            target_pose.pose.position.x = float(self.target_waypoint[0])
            target_pose.pose.position.y = float(self.target_waypoint[1])
            target_pose.pose.position.z = float(self.target_z_height)
            target_pose.pose.orientation.w = 1.0
            self.pose_pub.publish(target_pose)
            return

        # 1. Calculate relative target coordinates
        rel_target_raw = self.target_waypoint - self.current_pos[:2]
        rel_target = rel_target_raw.copy()
        dist_to_target = float(np.linalg.norm(rel_target))
        
        # OOD PREVENTION: The AI was trained with targets 2.0 to 4.0 meters away.
        # If we feed a target 10m away, the neural network saturates and outputs near-zero action.
        # We must cap the relative target magnitude to 4.0 meters ("Carrot on a stick" approach).
        if dist_to_target > 4.0:
            rel_target = (rel_target / dist_to_target) * 4.0

        # 2. Build observation vector (shape: 1, 364)
        # Structure: 360 (Lidar, 0.1-10m) + 2 (rel_target raw) + 2 (vel raw)
        obs = np.concatenate([
            self.lidar_ranges,
            rel_target,
            self.current_vel
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

        # Clear Path Blending: Jika jalur di depan aman (lidar > 1.5m),
        # padukan kontrol AI dengan vektor langsung ke target untuk mencegah miring/drift
        dist_to_target = np.linalg.norm(rel_target)
        direction = rel_target_raw / max(dist_to_target, 0.1)
        min_lidar_dist = float(np.min(self.lidar_ranges))

        if min_lidar_dist > 1.5:
            clear_factor = np.clip((min_lidar_dist - 1.5) / 1.0, 0.0, 1.0)
            target_vx = direction[0] * self.max_speed
            target_vy = direction[1] * self.max_speed
            ref_vx = (1.0 - clear_factor) * ref_vx + clear_factor * target_vx
            ref_vy = (1.0 - clear_factor) * ref_vy + clear_factor * target_vy

        # Debug: log action dan posisi setiap 20 loop (~2 detik)
        self._debug_counter = getattr(self, '_debug_counter', 0) + 1
        if self._debug_counter % 20 == 0:
            dist = np.linalg.norm(rel_target)
            self.get_logger().info(
                f"[AI] Pos=({self.current_pos[0]:.2f},{self.current_pos[1]:.2f}) "
                f"Target=({self.target_waypoint[0]:.1f},{self.target_waypoint[1]:.1f}) "
                f"Dist={dist:.2f}m | Action=({action[0]:.3f},{action[1]:.3f}) "
                f"→ Vx={ref_vx:.2f} Vy={ref_vy:.2f}"
            )

        # Fallback: jika AI output hampir nol tapi masih jauh dari target
        if dist_to_target > 0.5:
            ai_strength = abs(action[0]) + abs(action[1])
            blend = max(0.0, 0.8 - ai_strength)
            ref_vx += direction[0] * blend * self.max_speed
            ref_vy += direction[1] * blend * self.max_speed

        # Koreksi Overshoot: Jika target terlewati ke belakang (rel_target_raw[0] < -0.1)
        if rel_target_raw[0] < -0.1:
            ref_vx = 0.8 * (rel_target_raw[0] / dist_to_target * self.max_speed) + 0.2 * ref_vx
            ref_vy = 0.8 * (rel_target_raw[1] / dist_to_target * self.max_speed) + 0.2 * ref_vy

        # Stabilisasi Target Terdekat (Goal Reacher Mode):
        if dist_to_target < 1.0:
            alpha = dist_to_target
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
    import sys
    rclpy.init(args=args)
    node = CollisionAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down gracefully...')
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
