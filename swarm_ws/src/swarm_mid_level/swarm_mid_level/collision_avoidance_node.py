#!/usr/bin/env python3
import os
import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

class ORCASolver2D:
    """
    Pure Python 2D ORCA (Optimal Reciprocal Collision Avoidance) Solver.
    Computes reciprocal velocity half-planes for multi-agent swarm collision avoidance
    and solves the 2D linear programming problem to find the optimal safe velocity.
    """
    def __init__(self, time_horizon=5.0, safety_radius=0.5, max_speed=4.0):
        self.tau = time_horizon
        self.radius = safety_radius
        self.max_speed = max_speed

    def compute_orca_velocity(self, pos_self, vel_self, pref_vel, neighbors, lidar_lines=None):
        """
        pos_self: np.array([x, y])
        vel_self: np.array([vx, vy])
        pref_vel: np.array([vx_pref, vy_pref])
        neighbors: list of dicts [{'pos': np.array([x,y]), 'vel': np.array([vx,vy])}]
        lidar_lines: optional list of static line obstacles [(point, direction)]
        """
        orca_lines = []
        inv_tau = 1.0 / self.tau

        # 1. Build ORCA half-planes for each neighbor (dynamic drone or static obstacle)
        for neighbor in neighbors:
            is_static = neighbor.get('is_static', False)
            weight = 1.0 if is_static else 0.5
            rad_obs = 0.8 if is_static else self.radius # 0.8m obstacle safety radius
            combined_radius = self.radius + rad_obs
            combined_radius_sq = combined_radius ** 2

            pos_rel = neighbor['pos'] - pos_self
            vel_rel = vel_self - neighbor['vel']
            dist_sq = np.dot(pos_rel, pos_rel)

            if dist_sq > (self.max_speed * self.tau + combined_radius) ** 2:
                continue

            dist = np.sqrt(max(dist_sq, 1e-6))
            w = vel_rel - inv_tau * pos_rel
            w_len_sq = np.dot(w, w)

            if dist < combined_radius:
                # Collision imminent: project velocity out of collision cone instantly
                dist_inv = 1.0 / max(dist, 1e-4)
                unit_pos = pos_rel * dist_inv
                direction = np.array([-unit_pos[1], unit_pos[0]])
                u = (combined_radius - dist) * inv_tau * unit_pos
                line_point = vel_self + weight * u
                line_dir = direction
            else:
                # No current collision, check reciprocal velocity obstacle cone
                leg_len = np.sqrt(max(0.0, dist_sq - combined_radius_sq))
                if np.dot(w, pos_rel) < 0 and (np.dot(w, pos_rel) ** 2) > combined_radius_sq * w_len_sq:
                    # Cutoff circle projection
                    w_len = np.sqrt(max(w_len_sq, 1e-6))
                    unit_w = w / w_len
                    direction = np.array([unit_w[1], -unit_w[0]])
                    u = (combined_radius * inv_tau - w_len) * unit_w
                    line_point = vel_self + weight * u
                    line_dir = direction
                else:
                    # Legs projection
                    leg_unit_x = (pos_rel[0] * leg_len - pos_rel[1] * combined_radius) / dist_sq
                    leg_unit_y = (pos_rel[1] * leg_len + pos_rel[0] * combined_radius) / dist_sq
                    if np.cross(pos_rel, w) > 0:
                        direction = np.array([leg_unit_x, leg_unit_y])
                    else:
                        direction = np.array([-leg_unit_x, -leg_unit_y])
                    u = np.dot(vel_rel, direction) * direction - vel_rel
                    line_point = vel_self + weight * u
                    line_dir = direction

            orca_lines.append({'point': line_point, 'dir': line_dir})

        # 2. Add static Lidar obstacle lines if available
        if lidar_lines:
            for obs in lidar_lines:
                orca_lines.append(obs)

        # 3. Solve 2D Linear Program to get optimal velocity closest to pref_vel
        result_vel = self._linear_program_2d(orca_lines, self.max_speed, pref_vel)
        return result_vel

    def _linear_program_1d(self, lines, line_no, radius, opt_vel, direction_opt):
        dot_product = np.dot(lines[line_no]['point'], lines[line_no]['dir'])
        discriminant = dot_product ** 2 + radius ** 2 - np.dot(lines[line_no]['point'], lines[line_no]['point'])

        if discriminant < 0:
            return False, opt_vel

        sqrt_disc = np.sqrt(discriminant)
        t_left = -dot_product - sqrt_disc
        t_right = -dot_product + sqrt_disc

        for i in range(line_no):
            denominator = np.cross(lines[line_no]['dir'], lines[i]['dir'])
            numerator = np.cross(lines[i]['dir'], lines[line_no]['point'] - lines[i]['point'])

            if abs(denominator) < 1e-7:
                if numerator < 0:
                    return False, opt_vel
                continue

            t = numerator / denominator
            if denominator > 0:
                t_right = min(t_right, t)
            else:
                t_left = max(t_left, t)

            if t_left > t_right:
                return False, opt_vel

        if direction_opt:
            if np.dot(opt_vel, lines[line_no]['dir']) > 0:
                result_t = t_right
            else:
                result_t = t_left
        else:
            result_t = np.dot(lines[line_no]['dir'], opt_vel - lines[line_no]['point'])
            result_t = np.clip(result_t, t_left, t_right)

        result_vel = lines[line_no]['point'] + result_t * lines[line_no]['dir']
        return True, result_vel

    def _linear_program_2d(self, lines, radius, opt_vel):
        if np.dot(opt_vel, opt_vel) > radius ** 2:
            result_vel = (opt_vel / np.linalg.norm(opt_vel)) * radius
        else:
            result_vel = opt_vel.copy()

        for i in range(len(lines)):
            if np.cross(lines[i]['dir'], lines[i]['point'] - result_vel) > 0:
                success, new_vel = self._linear_program_1d(lines, i, radius, opt_vel, False)
                if success:
                    result_vel = new_vel
                else:
                    # Fallback if constraints overlap tightly
                    result_vel = lines[i]['point'] + np.dot(opt_vel - lines[i]['point'], lines[i]['dir']) * lines[i]['dir']
                    if np.dot(result_vel, result_vel) > radius ** 2:
                        result_vel = (result_vel / np.linalg.norm(result_vel)) * radius

        return result_vel


class CollisionAvoidanceNode(Node):
    """
    ROS 2 Swarm Collision Avoidance Node using 2D ORCA (Optimal Reciprocal Collision Avoidance).
    Subscribes to odometry of all neighbor drones and Lidar 2D scans to compute
    reciprocal collision-free target trajectories.
    """
    def __init__(self):
        super().__init__('collision_avoidance_node')

        # Parameters
        self.declare_parameter('max_speed', 2.5)
        self.declare_parameter('target_z_height', 2.0)
        self.declare_parameter('dt', 0.1)
        self.declare_parameter('drone_id', 1)
        self.declare_parameter('num_drones', 7)
        self.declare_parameter('safety_radius', 0.8) # 0.8m radius -> 1.6m center-to-center = 80cm prop clearance
        self.declare_parameter('time_horizon', 5.0)

        self.max_speed = self.get_parameter('max_speed').value
        self.target_z_height = self.get_parameter('target_z_height').value
        self.dt = self.get_parameter('dt').value
        self.num_drones = self.get_parameter('num_drones').value
        self.safety_radius = self.get_parameter('safety_radius').value
        self.time_horizon = self.get_parameter('time_horizon').value

        node_name = self.get_name()
        if '_' in node_name and node_name.split('_')[-1].isdigit():
            did = int(node_name.split('_')[-1])
        else:
            did = int(self.get_parameter('drone_id').value)
        self.drone_id = did

        # Initialize ORCA Solver
        self.orca_solver = ORCASolver2D(
            time_horizon=self.time_horizon,
            safety_radius=self.safety_radius,
            max_speed=self.max_speed
        )

        # State Variables
        self.current_pos = np.zeros(3, dtype=np.float32) # [x, y, z]
        self.current_vel = np.zeros(2, dtype=np.float32) # [vx, vy]
        
        # Target awal waypoint = lokasi formation spawn persis
        spacing = 2.0
        spawn_y = float((did - 4.0) * spacing)
        self.target_waypoint = np.array([0.0, spawn_y], dtype=np.float32)
        self.waypoint_received = False
        self.lidar_ranges = np.ones(360, dtype=np.float32) * 10.0
        self.steps = 0

        # Swarm Neighbors Telemetry Dictionary {id: {'pos': [x,y], 'vel': [vx,vy], 'stamp': time}}
        self.neighbors_state = {}

        # 1. Own Drone Subscribers
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
        self.waypoint_pose_sub = self.create_subscription(
            PoseStamped,
            f'/iris_{did}/waypoint_pose',
            self.waypoint_pose_callback,
            10
        )

        # 2. Subscribe to all neighbor drones' odometry for ORCA reciprocal state
        for i in range(1, self.num_drones + 1):
            if i == did:
                continue
            self.create_subscription(
                Odometry,
                f'/iris_{i}/odometry',
                self.make_neighbor_odom_callback(i),
                10
            )

        # 3. Publisher to low-level PID-LQR / PID-Hinf controller
        self.pose_pub = self.create_publisher(
            PoseStamped,
            f'/iris_{did}/target_pose',
            10
        )

        # 4. Timer to run ORCA calculation at 10Hz
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info(f"🚀 [ORCA] Swarm Node initialized for iris_{did} (Total Drones: {self.num_drones})")

    def make_neighbor_odom_callback(self, nid):
        def callback(msg):
            px = msg.pose.pose.position.x
            py = msg.pose.pose.position.y
            vx = msg.twist.twist.linear.x
            vy = msg.twist.twist.linear.y
            self.neighbors_state[nid] = {
                'pos': np.array([px, py], dtype=np.float32),
                'vel': np.array([vx, vy], dtype=np.float32)
            }
        return callback

    def lidar_callback(self, msg):
        ranges = np.array(msg.ranges, dtype=np.float32)
        ranges = np.nan_to_num(ranges, nan=10.0, posinf=10.0, neginf=0.1)
        self.lidar_ranges = np.clip(ranges, 0.1, 10.0)

    def euler_from_quaternion(self, x, y, z, w):
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        return math.atan2(t3, t4)

    def odom_callback(self, msg):
        self.current_pos[0] = msg.pose.pose.position.x
        self.current_pos[1] = msg.pose.pose.position.y
        self.current_pos[2] = msg.pose.pose.position.z

        self.current_vel[0] = msg.twist.twist.linear.x
        self.current_vel[1] = msg.twist.twist.linear.y

        if not hasattr(self, 'spawn_yaw'):
            qx = msg.pose.pose.orientation.x
            qy = msg.pose.pose.orientation.y
            qz = msg.pose.pose.orientation.z
            qw = msg.pose.pose.orientation.w
            yaw0 = self.euler_from_quaternion(qx, qy, qz, qw)
            self.spawn_yaw = yaw0
            self.yaw_smooth = yaw0

    def waypoint_callback(self, msg):
        self.target_waypoint = np.array([msg.point.x, msg.point.y], dtype=np.float32)
        self.target_z_height = msg.point.z
        self.waypoint_received = True
        self.get_logger().info(
            f"[ORCA] Waypoint baru diterima: X={msg.point.x:.2f}, Y={msg.point.y:.2f}, Z={msg.point.z:.2f}"
        )

    def waypoint_pose_callback(self, msg):
        self.target_waypoint = np.array([msg.pose.position.x, msg.pose.position.y], dtype=np.float32)
        self.target_z_height = msg.pose.position.z
        self.waypoint_received = True

    def control_loop(self):
        self.steps += 1
        if self.target_waypoint is None:
            return

        # 0. Hover at spawn position until Waypoint is received
        if not self.waypoint_received:
            target_pose = PoseStamped()
            target_pose.header.stamp = self.get_clock().now().to_msg()
            target_pose.header.frame_id = 'world'
            target_pose.pose.position.x = float(self.target_waypoint[0])
            target_pose.pose.position.y = float(self.target_waypoint[1])
            target_pose.pose.position.z = float(self.target_z_height)
            init_yaw = getattr(self, 'spawn_yaw', 0.0)
            half_yaw = init_yaw * 0.5
            target_pose.pose.orientation.x = 0.0
            target_pose.pose.orientation.y = 0.0
            target_pose.pose.orientation.z = float(np.sin(half_yaw))
            target_pose.pose.orientation.w = float(np.cos(half_yaw))
            self.pose_pub.publish(target_pose)
            return

        # 1. Calculate Preferred Velocity towards Target Waypoint
        rel_target = self.target_waypoint - self.current_pos[:2]
        dist_to_target = float(np.linalg.norm(rel_target))

        if dist_to_target < 0.1:
            pref_vel = np.zeros(2, dtype=np.float32)
        else:
            # Deselerasi halus saat mendekati titik tujuan agar tidak overshoot
            speed = min(self.max_speed, dist_to_target * 0.8)
            pref_vel = (rel_target / dist_to_target) * speed

        # 2. Extract neighbor drone states and apply Non-Linear Repulsion (Inverse-Square Law)
        neighbor_list = list(self.neighbors_state.values())
        repulsion_vec = np.zeros(2, dtype=np.float32)

        # 2b. Repulsion from neighbor drones (Zone = 2.0m)
        for nbr in neighbor_list:
            rel_nbr = self.current_pos[:2] - nbr['pos'] # Pointing AWAY from neighbor
            dist_nbr = float(np.linalg.norm(rel_nbr))
            if 1e-3 < dist_nbr < 2.0:
                # Inverse-Square Law: semakin dekat (< 1.0m), gaya tolak melonjak sangat kuat
                rep_gain = ((2.0 / max(dist_nbr, 0.4)) ** 2) * 0.4
                repulsion_vec += (rel_nbr / dist_nbr) * rep_gain

        # 3. Extract static Lidar obstacles as Bounded ORCA Obstacle Cones (Center-based)
        current_yaw = getattr(self, 'yaw_smooth', 0.0)
        angles_body = np.linspace(-np.pi, np.pi, len(self.lidar_ranges))
        angles_world = current_yaw + angles_body # Transform Lidar body frame to World frame
        obs_mask = self.lidar_ranges < 3.0

        if np.any(obs_mask):
            min_idx = np.argmin(self.lidar_ranges)
            dist_min = float(self.lidar_ranges[min_idx])
            angle_min_world = float(angles_world[min_idx])

            # Vektor relatif ke permukaan rintangan terdekat
            obs_rel_min = np.array([dist_min * np.cos(angle_min_world), dist_min * np.sin(angle_min_world)], dtype=np.float32)
            obs_dir = obs_rel_min / max(dist_min, 0.05)

            # Estimasi titik pusat rintangan (0.3m di belakang permukaan terdepan)
            obs_center_pos = self.current_pos[:2] + obs_rel_min + obs_dir * 0.3
            neighbor_list.append({
                'pos': obs_center_pos,
                'vel': np.zeros(2, dtype=np.float32),
                'is_static': True
            })

            # Non-Linear Repulsion & Tangential Dodge untuk titik Lidar dekat (< 3.0m)
            close_indices = np.where(obs_mask)[0]
            for idx in close_indices[::4]:
                d_i = float(self.lidar_ranges[idx])
                ang_i_world = float(angles_world[idx])
                obs_rel_i = np.array([d_i * np.cos(ang_i_world), d_i * np.sin(ang_i_world)], dtype=np.float32)
                push_dir = -obs_rel_i / max(d_i, 0.05)
                rep_gain_i = ((3.0 / max(d_i, 0.4)) ** 2) * 0.3
                repulsion_vec += push_dir * rep_gain_i

            # Tangential Steering: Tambahkan belokan memutar jika rintangan tepat di depan
            dot_front = np.dot(pref_vel / max(np.linalg.norm(pref_vel), 0.1), obs_dir)
            if dot_front > 0.3:
                tangent_dir = np.array([-obs_dir[1], obs_dir[0]], dtype=np.float32)
                if np.cross(pref_vel, obs_dir) > 0:
                    tangent_dir = -tangent_dir
                repulsion_vec += tangent_dir * (self.max_speed * 0.6)

        # Cap total repulsion vector magnitude to prevent extreme force spikes
        rep_len = float(np.linalg.norm(repulsion_vec))
        max_rep = self.max_speed * 0.4
        if rep_len > max_rep:
            repulsion_vec = (repulsion_vec / rep_len) * max_rep

        # Anti-Chattering Filter: Smooth repulsion_vec across time
        if not hasattr(self, 'repulsion_smooth'):
            self.repulsion_smooth = repulsion_vec
        else:
            self.repulsion_smooth = 0.7 * self.repulsion_smooth + 0.3 * repulsion_vec

        # Gabungkan gaya tolak non-linear ke pref_vel
        pref_vel = pref_vel + self.repulsion_smooth

        # 4. Compute ORCA Reciprocal Safe Velocity with Static Wall Constraints
        safe_vel = self.orca_solver.compute_orca_velocity(
            pos_self=self.current_pos[:2],
            vel_self=self.current_vel,
            pref_vel=pref_vel,
            neighbors=neighbor_list,
            lidar_lines=lidar_lines
        )

        # 5. Low-Pass Velocity Filter & Slew Rate Limiter (mencegah RPM saturation & drone terbalik)
        ref_vx = np.clip(safe_vel[0], -self.max_speed, self.max_speed)
        ref_vy = np.clip(safe_vel[1], -0.5, 0.5) # Cap lateral speed to +-0.5m/s for zero roll instability

        if not hasattr(self, 'cmd_vel_smooth'):
            self.cmd_vel_smooth = np.array([ref_vx, ref_vy], dtype=np.float32)
        else:
            self.cmd_vel_smooth = 0.75 * self.cmd_vel_smooth + 0.25 * np.array([ref_vx, ref_vy], dtype=np.float32)

        out_vx, out_vy = self.cmd_vel_smooth[0], self.cmd_vel_smooth[1]

        # 5b. Responsive Heading-Tracking Yaw Control (Tanpa Double Phase Lag)
        if not hasattr(self, 'yaw_smooth'):
            self.yaw_smooth = getattr(self, 'spawn_yaw', 0.0)

        # Hitung yaw_target langsung dari safe_vel ORCA (bukan dari cmd_vel_smooth) untuk menghilangkan lag drift
        safe_speed = float(np.sqrt(safe_vel[0]**2 + safe_vel[1]**2))
        YAW_DEADBAND = 0.15  # m/s — freeze yaw jika kecepatan sangat kecil / hover

        if self.waypoint_received and safe_speed > YAW_DEADBAND and dist_to_target > 0.3:
            yaw_target = float(np.arctan2(safe_vel[1], safe_vel[0]))
            # Normalisasi selisih sudut ke range [-pi, pi]
            delta_yaw = (yaw_target - self.yaw_smooth + np.pi) % (2 * np.pi) - np.pi
            
            # Responsif tanpa lag drift: ikuti arah safe_vel secara langsung dengan smoothing ringan
            alpha_yaw = min(0.35 * (safe_speed / self.max_speed) + 0.1, 0.5)
            self.yaw_smooth += alpha_yaw * delta_yaw
            self.yaw_smooth = (self.yaw_smooth + np.pi) % (2 * np.pi) - np.pi

        # Encode yaw_smooth ke quaternion orientation (roll=0, pitch=0, yaw=yaw_smooth)
        half_yaw = self.yaw_smooth * 0.5
        qw = float(np.cos(half_yaw))
        qz = float(np.sin(half_yaw))

        # Debug log every ~2s
        if self.steps % 20 == 0:
            self.get_logger().info(
                f"[ORCA] Pos=({self.current_pos[0]:.2f},{self.current_pos[1]:.2f}) "
                f"Target=({self.target_waypoint[0]:.1f},{self.target_waypoint[1]:.1f}) "
                f"Dist={dist_to_target:.2f}m | Vel=({out_vx:.2f},{out_vy:.2f}) | Yaw={np.degrees(self.yaw_smooth):.1f}°"
            )

        # 6. Integrate ORCA velocity to target position with lookahead for full PID-LQR velocity tracking
        target_pose = PoseStamped()
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.header.frame_id = 'world'

        if dist_to_target < 0.6:
            # Tiba di tujuan: Kunci target_pose tepat pada target_waypoint (cegah overshoot / oscilasi)
            target_pose.pose.position.x = float(self.target_waypoint[0])
            target_pose.pose.position.y = float(self.target_waypoint[1])
            self.cmd_vel_smooth = np.zeros(2, dtype=np.float32)
        else:
            lookahead_sec = 0.75
            proj_x = self.current_pos[0] + out_vx * lookahead_sec
            proj_y = self.current_pos[1] + out_vy * lookahead_sec
            # Clamp agar proyeksi target_pose tidak melebihi target_waypoint
            if self.target_waypoint[0] >= 0:
                proj_x = min(proj_x, float(self.target_waypoint[0]))
            target_pose.pose.position.x = float(proj_x)
            target_pose.pose.position.y = float(proj_y)

        target_pose.pose.position.z = float(self.target_z_height)
        target_pose.pose.orientation.x = 0.0
        target_pose.pose.orientation.y = 0.0
        target_pose.pose.orientation.z = qz
        target_pose.pose.orientation.w = qw

        self.pose_pub.publish(target_pose)


def main(args=None):
    rclpy.init(args=args)
    node = CollisionAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down ORCA node gracefully...')
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
