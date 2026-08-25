# FIXXXXX yaw control
#!/usr/bin/env python3
import os
import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped, TwistStamped, Twist
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
            rad_obs = neighbor.get('radius', 0.8 if is_static else self.radius)
            combined_radius = self.radius + rad_obs
            combined_radius_sq = combined_radius ** 2

            # Gunakan time horizon yang jauh lebih kecil (0.8s) untuk rintangan statis agar respon tolakan instan
            curr_tau = 0.8 if is_static else self.tau
            curr_inv_tau = 1.0 / curr_tau

            pos_rel = neighbor['pos'] - pos_self
            vel_rel = vel_self - neighbor['vel']
            dist_sq = np.dot(pos_rel, pos_rel)

            if dist_sq > (self.max_speed * curr_tau + combined_radius) ** 2:
                continue

            dist = np.sqrt(max(dist_sq, 1e-6))
            w = vel_rel - curr_inv_tau * pos_rel
            w_len_sq = np.dot(w, w)

            if dist < combined_radius:
                # Collision imminent: project velocity out of collision cone instantly
                dist_inv = 1.0 / max(dist, 1e-4)
                unit_pos = pos_rel * dist_inv
                direction = np.array([-unit_pos[1], unit_pos[0]])
                u = (combined_radius - dist) * curr_inv_tau * unit_pos
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
                    u = (combined_radius * curr_inv_tau - w_len) * unit_w
                    line_point = vel_self + weight * u
                    line_dir = direction
                else:
                    # Legs projection
                    leg_unit_x = (pos_rel[0] * leg_len - pos_rel[1] * combined_radius) / dist_sq
                    leg_unit_y = (pos_rel[1] * leg_len + pos_rel[0] * combined_radius) / dist_sq
                    if (pos_rel[0] * w[1] - pos_rel[1] * w[0]) > 0:
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
            denominator = lines[line_no]['dir'][0] * lines[i]['dir'][1] - lines[line_no]['dir'][1] * lines[i]['dir'][0]
            diff_pt = lines[line_no]['point'] - lines[i]['point']
            numerator = lines[i]['dir'][0] * diff_pt[1] - lines[i]['dir'][1] * diff_pt[0]

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
            diff_vel = lines[i]['point'] - result_vel
            if (lines[i]['dir'][0] * diff_vel[1] - lines[i]['dir'][1] * diff_vel[0]) > 0:
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
        self.declare_parameter('max_speed', 3.0)
        self.declare_parameter('target_z_height', 2.0)
        self.declare_parameter('dt', 0.1)
        self.declare_parameter('drone_id', 1)
        self.declare_parameter('num_drones', 7)
        self.declare_parameter('safety_radius', 0.75) # 0.75m radius for wider safe clearance bubble
        self.declare_parameter('time_horizon', 5.0)

        self.max_speed = self.get_parameter('max_speed').value
        self.target_z_height = self.get_parameter('target_z_height').value
        self.dt = self.get_parameter('dt').value
        self.num_drones = int(self.get_parameter('num_drones').value)
        self.safety_radius = self.get_parameter('safety_radius').value
        self.time_horizon = self.get_parameter('time_horizon').value
        self.lookahead_damping = 0.0

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
        self.current_yaw = 0.0
        self.yaw_smooth = 0.0
        
        # Target awal waypoint: None sampai odometry pertama diterima
        self.spawn_x = 0.0
        self.spawn_y = 0.0
        self.target_waypoint = None
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
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            f'/iris_{did}/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.cmd_vel_input = None
        self.cmd_vel_timeout = 0

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
        # Publisher kecepatan ORCA untuk velocity feedforward di low-level
        self.vel_pub = self.create_publisher(
            TwistStamped,
            f'/iris_{did}/target_velocity',
            10
        )

        # 4. Timer to run ORCA calculation at 10Hz
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info(f"🚀 [ORCA] Swarm Node initialized for iris_{did} (Total Drones: {self.num_drones})")

    def make_neighbor_odom_callback(self, nid):
        def callback(msg):
            px = float(msg.pose.pose.position.x)
            py = float(msg.pose.pose.position.y)
            vx_b = float(msg.twist.twist.linear.x)
            vy_b = float(msg.twist.twist.linear.y)
            qx = msg.pose.pose.orientation.x
            qy = msg.pose.pose.orientation.y
            qz = msg.pose.pose.orientation.z
            qw = msg.pose.pose.orientation.w
            _, _, yaw_nbr = self.euler_from_quaternion(qx, qy, qz, qw)

            # Konversi kecepatan V2V tetangga dari Body Frame ke World Frame
            cos_y = math.cos(yaw_nbr)
            sin_y = math.sin(yaw_nbr)
            vx_w = vx_b * cos_y - vy_b * sin_y
            vy_w = vx_b * sin_y + vy_b * cos_y

            self.neighbors_state[nid] = {
                'pos': np.array([px, py], dtype=np.float32),
                'vel': np.array([vx_w, vy_w], dtype=np.float32)
            }
        return callback

    def lidar_callback(self, msg):
        ranges = np.array(msg.ranges, dtype=np.float32)
        ranges = np.nan_to_num(ranges, nan=10.0, posinf=10.0, neginf=0.1)
        self.lidar_ranges = np.clip(ranges, 0.1, 10.0)

    def euler_from_quaternion(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return roll, pitch, yaw

    def odom_callback(self, msg):
        self.current_pos[0] = msg.pose.pose.position.x
        self.current_pos[1] = msg.pose.pose.position.y
        self.current_pos[2] = msg.pose.pose.position.z

        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        roll, pitch, yaw = self.euler_from_quaternion(qx, qy, qz, qw)
        self.current_roll = roll
        self.current_pitch = pitch
        self.current_yaw = yaw

        # Konversi kecepatan sendiri dari Body Frame ke World Frame
        vx_b = float(msg.twist.twist.linear.x)
        vy_b = float(msg.twist.twist.linear.y)
        cos_y = math.cos(self.current_yaw)
        sin_y = math.sin(self.current_yaw)
        self.current_vel[0] = vx_b * cos_y - vy_b * sin_y
        self.current_vel[1] = vx_b * sin_y + vy_b * cos_y

        if not hasattr(self, 'spawn_yaw'):
            self.spawn_yaw = self.current_yaw
            self.yaw_smooth = self.current_yaw
            self.spawn_x = float(self.current_pos[0])
            self.spawn_y = float(self.current_pos[1])
            if not self.waypoint_received:
                self.target_waypoint = np.array([self.spawn_x, self.spawn_y], dtype=np.float32)

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

    def cmd_vel_callback(self, msg: Twist):
        self.cmd_vel_input = msg
        self.cmd_vel_timeout = 25  # active for 0.50s (25 ticks @ 50Hz)
        self.waypoint_received = True

    def control_loop(self):
        self.steps += 1
        if self.target_waypoint is None:
            return

        # 0. Hover at spawn position until Waypoint or cmd_vel is received
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

        # 1. Calculate Preferred Velocity
        is_vel_mode = (self.cmd_vel_input is not None and self.cmd_vel_timeout > 0)
        if is_vel_mode:
            self.cmd_vel_timeout -= 1
            self.target_waypoint = np.array([self.current_pos[0], self.current_pos[1]], dtype=np.float32)
            body_vx = float(self.cmd_vel_input.linear.x)
            body_vy = float(self.cmd_vel_input.linear.y)

            # Rotate body velocity to world frame using current yaw
            cos_y = math.cos(self.current_yaw)
            sin_y = math.sin(self.current_yaw)
            world_vx = body_vx * cos_y - body_vy * sin_y
            world_vy = body_vx * sin_y + body_vy * cos_y

            # Smooth Yaw Follow: align heading to actual world motion vector
            speed_xy = math.hypot(world_vx, world_vy)
            if speed_xy > 0.15:
                des_yaw = math.atan2(world_vy, world_vx)
                yaw_diff = (des_yaw - self.yaw_smooth + np.pi) % (2 * np.pi) - np.pi
                max_turn_step = 2.5 * self.dt  # smooth turning rate limit (~140 deg/s)
                self.yaw_smooth += np.clip(yaw_diff * 4.5 * self.dt, -max_turn_step, max_turn_step)
                self.yaw_smooth = (self.yaw_smooth + np.pi) % (2 * np.pi) - np.pi

            pref_vel = np.array([world_vx, world_vy], dtype=np.float32)
            dist_to_target = 5.0  # nominal distance for obstacle scaling
        else:
            rel_target = self.target_waypoint - self.current_pos[:2]
            dist_to_target = float(np.linalg.norm(rel_target))

            if dist_to_target < 0.1:
                pref_vel = np.zeros(2, dtype=np.float32)
            else:
                # Deselerasi halus saat mendekati titik tujuan agar tidak overshoot (tuning 1.5)
                speed = min(self.max_speed, dist_to_target * 1.5)
                pref_vel = (rel_target / dist_to_target) * speed
                
                # Lane-keeping restoring force to pull the drone back to Y_spawn lane
                # Only apply if the mission waypoint is along the original spawn lane (e.g. multi-agent straight-line mission)
                if dist_to_target > 0.5 and abs(self.target_waypoint[1] - self.spawn_y) < 0.2:
                    y_err = self.current_pos[1] - self.spawn_y
                    y_restore = -0.35 * y_err  # restoring proportional gain
                    y_restore = np.clip(y_restore, -0.6, 0.6) # clip to prevent excessive lateral commands
                    pref_vel[1] += y_restore

        # 1b. Break head-on symmetry (COLREGs Turn-Right Rule)
        # Hanya aktif saat jarak sudah dekat (< 2.2m) agar drone tetap berada di jalur mapping sepanjang mungkin
        for nbr in self.neighbors_state.values():
            rel_nbr = nbr['pos'] - self.current_pos[:2]
            dist_nbr = float(np.linalg.norm(rel_nbr))
            if dist_nbr < 2.2:
                pref_speed = np.linalg.norm(pref_vel)
                if pref_speed > 0.1:
                    unit_pref = pref_vel / pref_speed
                    unit_nbr = rel_nbr / max(dist_nbr, 0.05)
                    dot_front = np.dot(unit_pref, unit_nbr)
                    if dot_front > 0.45:
                        # Vektor tegak lurus ke kanan arah terbang
                        right_vec = np.array([unit_pref[1], -unit_pref[0]], dtype=np.float32)
                        bias_gain = 0.20 * (1.0 - (dist_nbr / 2.2))
                        pref_vel += right_vec * (self.max_speed * bias_gain)

        # 2. Extract neighbor drone states and apply Non-Linear Repulsion (Inverse-Square Law)
        neighbor_list = list(self.neighbors_state.values())
        repulsion_vec = np.zeros(2, dtype=np.float32)

        # 2b. Repulsion from neighbor drones (hanya aktif pada jarak ekstra dekat < 0.45m sebagai buffer darurat)
        for nbr in neighbor_list:
            rel_nbr = self.current_pos[:2] - nbr['pos'] # Pointing AWAY from neighbor
            dist_nbr = float(np.linalg.norm(rel_nbr))
            if 1e-3 < dist_nbr < 0.45:
                rep_gain = ((0.45 / max(dist_nbr, 0.2)) ** 2) * 0.2
                repulsion_vec += (rel_nbr / dist_nbr) * rep_gain

        # 3. Extract static Lidar obstacles as Point-Cloud Obstacles in ORCA
        # LiDAR only enabled above 1.5m altitude. Below that (takeoff/landing),
        # the 2D LiDAR beam hits the ground and creates 360° phantom obstacles,
        # corrupting ORCA velocity and yaw computation.
        current_yaw = getattr(self, 'current_yaw', 0.0)
        current_pitch = getattr(self, 'current_pitch', 0.0)
        current_roll = getattr(self, 'current_roll', 0.0)
        angles_body = np.linspace(-np.pi, np.pi, len(self.lidar_ranges))
        angles_world = current_yaw + angles_body # Transform Lidar body frame to World frame

        # Hitung estimasi ketinggian Z titik kontak laser di koordinat dunia berdasarkan Roll & Pitch
        # z_hit = z_drone - d_i * (sin(pitch)*cos(body_angle) - sin(roll)*sin(body_angle))
        z_drone = float(self.current_pos[2])
        z_ray_offsets = np.sin(current_pitch) * np.cos(angles_body) - np.sin(current_roll) * np.sin(angles_body)
        z_hits = z_drone - self.lidar_ranges * z_ray_offsets

        # Sinar laser hanya diakui sebagai rintangan nyata jika berada di atas lantai (z_hit > 0.35m)
        # dan berada dalam radius deteksi aktif (< 4.5m)
        if z_drone > 1.0:
            obs_mask = (self.lidar_ranges < 4.5) & (z_hits > 0.35)
        else:
            obs_mask = np.zeros(len(self.lidar_ranges), dtype=bool)

        if np.any(obs_mask):
            close_indices = np.where(obs_mask)[0]
            if self.steps % 20 == 0:
                self.get_logger().info(
                    f"[LIDAR] Real Obstacle points: {len(close_indices)} Z={z_drone:.2f}m"
                )
            
            # 3a. Represent Lidar points directly as static ORCA obstacle spheres
            # Downsample to every 6th ray to prevent solver lag
            for idx in close_indices[::6]:
                d_i = float(self.lidar_ranges[idx])
                if d_i < 0.5:
                    continue  # skip self-detection / LiDAR noise
                ang_i_world = float(angles_world[idx])
                obs_pos_i = self.current_pos[:2] + np.array([d_i * np.cos(ang_i_world), d_i * np.sin(ang_i_world)], dtype=np.float32)
                
                # Filter out teammate drones (exclude all LiDAR rays within 1.5m of teammate)
                is_neighbor = False
                for nbr in self.neighbors_state.values():
                    if np.linalg.norm(obs_pos_i - nbr['pos']) < 1.50:
                        is_neighbor = True
                        break
                if is_neighbor:
                    continue

                # Each point is a small static circle to form a clean boundary buffer
                neighbor_list.append({
                    'pos': obs_pos_i,
                    'vel': np.zeros(2, dtype=np.float32),
                    'is_static': True,
                    'radius': 0.15  # combined with safety_radius (0.75m) = 0.90m comfortable clearance bubble
                })

            # 3b. Non-Linear Repulsion for smooth steering
            for idx in close_indices[::4]:
                d_i = float(self.lidar_ranges[idx])
                if d_i < 0.35 or d_i > 1.6:
                    continue
                ang_i_world = float(angles_world[idx])
                obs_pos_i = self.current_pos[:2] + np.array([d_i * np.cos(ang_i_world), d_i * np.sin(ang_i_world)], dtype=np.float32)

                # Filter out teammate drones
                is_neighbor = False
                for nbr in self.neighbors_state.values():
                    if np.linalg.norm(obs_pos_i - nbr['pos']) < 1.50:
                        is_neighbor = True
                        break
                if is_neighbor:
                    continue

                obs_rel_i = np.array([d_i * np.cos(ang_i_world), d_i * np.sin(ang_i_world)], dtype=np.float32)
                
                # Hanya terapkan gaya tolak jika titik rintangan berada di hemisfer depan pergerakan drone
                is_front = True
                pref_speed = np.linalg.norm(pref_vel)
                if pref_speed > 0.1:
                    is_front = np.dot(obs_rel_i, pref_vel) > 0
                
                if is_front:
                    push_dir = -obs_rel_i / max(d_i, 0.05)
                    rep_gain_i = ((1.5 / max(d_i, 0.35)) ** 2) * 0.18
                    repulsion_vec += push_dir * rep_gain_i

            # 3c. Tangential Steering: Smooth curve around closest obstacle
            valid_ranges = np.where(obs_mask, self.lidar_ranges, 10.0)
            min_idx = np.argmin(valid_ranges)
            dist_min = float(valid_ranges[min_idx])
            angle_min_world = float(angles_world[min_idx])
            obs_rel_min = np.array([dist_min * np.cos(angle_min_world), dist_min * np.sin(angle_min_world)], dtype=np.float32)
            obs_dir = obs_rel_min / max(dist_min, 0.05)
            
            # Check if closest lidar point is a teammate drone
            obs_pos_min = self.current_pos[:2] + obs_rel_min
            is_neighbor_min = False
            for nbr in self.neighbors_state.values():
                if np.linalg.norm(obs_pos_min - nbr['pos']) < 1.50:
                    is_neighbor_min = True
                    break

            if not is_neighbor_min and dist_min < 1.3:
                dot_front = np.dot(pref_vel / max(np.linalg.norm(pref_vel), 0.1), obs_dir)
                if dot_front > 0.2:
                    tangent_dir = np.array([-obs_dir[1], obs_dir[0]], dtype=np.float32)
                    if (pref_vel[0] * obs_dir[1] - pref_vel[1] * obs_dir[0]) > 0:
                        tangent_dir = -tangent_dir
                    repulsion_vec += tangent_dir * (self.max_speed * 0.22)

        # Cap total repulsion vector magnitude to prevent extreme force spikes
        rep_len = float(np.linalg.norm(repulsion_vec))
        max_rep = self.max_speed * 0.3
        if rep_len > max_rep:
            repulsion_vec = (repulsion_vec / rep_len) * max_rep

        # Skala gaya tolak mengecil saat mendekati target untuk mencegah deadlock hover di akhir
        repulsion_scale = min(1.0, dist_to_target / 1.5)
        repulsion_vec *= repulsion_scale

        # Anti-Chattering Filter: Smooth repulsion_vec across time
        if not hasattr(self, 'repulsion_smooth'):
            self.repulsion_smooth = repulsion_vec
        else:
            self.repulsion_smooth = 0.85 * self.repulsion_smooth + 0.15 * repulsion_vec

        # Gabungkan gaya tolak hanya jika belum dekat target untuk menghindari drifting saat melayang diam
        if dist_to_target > 0.3:
            pref_vel = pref_vel + self.repulsion_smooth
        else:
            self.repulsion_smooth = np.zeros(2, dtype=np.float32)

        # 4. Compute ORCA Reciprocal Safe Velocity with Static Wall Constraints
        safe_vel = self.orca_solver.compute_orca_velocity(
            pos_self=self.current_pos[:2],
            vel_self=self.current_vel,
            pref_vel=pref_vel,
            neighbors=neighbor_list,
            lidar_lines=None
        )

        # 5. Low-Pass Velocity Filter & Slew Rate Limiter (mencegah RPM saturation & drone terbalik)
        ref_vx = np.clip(safe_vel[0], -self.max_speed, self.max_speed)
        ref_vy = np.clip(safe_vel[1], -2.0, 2.0) # Cap lateral speed to +-2.0m/s for fast stable avoidance

        # 5. Smooth Acceleration / Slew-Rate Limiter (Max 3.0 m/s^2 acceleration untuk gerak gesit tanpa kaget)
        MAX_ACCEL = 3.0  # m/s^2
        dt_mid = 0.1     # 10 Hz control loop
        max_dv = MAX_ACCEL * dt_mid  # max 0.30 m/s per step

        target_vel_raw = np.array([ref_vx, ref_vy], dtype=np.float32)
        if not hasattr(self, 'cmd_vel_smooth'):
            self.cmd_vel_smooth = np.array([0.0, 0.0], dtype=np.float32)
        
        dv = target_vel_raw - self.cmd_vel_smooth
        dv_mag = float(np.linalg.norm(dv))
        if dv_mag > max_dv:
            dv = (dv / dv_mag) * max_dv
        
        self.cmd_vel_smooth += dv
        out_vx, out_vy = float(self.cmd_vel_smooth[0]), float(self.cmd_vel_smooth[1])

        # 5b. Yaw Control & 6. Position Integration
        if is_vel_mode:
            # Mode Pemetaan / Holonomik: pertahankan orientasi yaw tetap (spawn_yaw = 0)
            # Quadrotor bergerak bebas di bidang (vx, vy) tanpa perlu memutar hidung.
            # Ini menghilangkan 100% gangguan gyroskopik dan cross-coupling roll-pitch saat sweep.
            yaw_rate = float(self.cmd_vel_input.angular.z) if (hasattr(self, 'cmd_vel_input') and self.cmd_vel_input is not None) else 0.0

            half_yaw = self.yaw_smooth * 0.5
            qw = float(np.cos(half_yaw))
            qz = float(np.sin(half_yaw))

            if not hasattr(self, 'pos_ref'):
                self.pos_ref = np.array([self.current_pos[0], self.current_pos[1]], dtype=np.float32)

            self.pos_ref[0] += out_vx * self.dt
            self.pos_ref[1] += out_vy * self.dt

            # Tether pos_ref to current_pos to avoid runaway reference (1.00m tuned lead window for fast, zero-overshoot flight)
            tracking_err = float(np.linalg.norm(self.pos_ref - self.current_pos[:2]))
            if tracking_err > 1.00:
                self.pos_ref = self.current_pos[:2] + (self.pos_ref - self.current_pos[:2]) * (1.00 / tracking_err)

            target_pose = PoseStamped()
            target_pose.header.stamp = self.get_clock().now().to_msg()
            target_pose.header.frame_id = 'world'
            target_pose.pose.position.x = float(self.pos_ref[0])
            target_pose.pose.position.y = float(self.pos_ref[1])
            target_pose.pose.position.z = float(self.target_z_height)
            target_pose.pose.orientation.x = 0.0
            target_pose.pose.orientation.y = 0.0
            target_pose.pose.orientation.z = qz
            target_pose.pose.orientation.w = qw
            self.pose_pub.publish(target_pose)

            vel_msg = TwistStamped()
            vel_msg.header.stamp = target_pose.header.stamp
            vel_msg.header.frame_id = 'world'
            vel_msg.twist.linear.x = float(out_vx)
            vel_msg.twist.linear.y = float(out_vy)
            vel_msg.twist.angular.z = float(yaw_rate)
            self.vel_pub.publish(vel_msg)
            return
        else:
            # Hybrid Waypoint + Velocity Blend for Waypoint Navigation
            if not hasattr(self, 'yaw_smooth'):
                self.yaw_smooth = getattr(self, 'spawn_yaw', 0.0)

            YAW_SPEED_DEADBAND = 0.15
            YAW_DIST_DEADBAND = 0.8
            YAW_FILTER_ALPHA = 0.25

            cmd_speed = float(np.sqrt(out_vx**2 + out_vy**2))
            dx_target = self.target_waypoint[0] - self.current_pos[0]
            dy_target = self.target_waypoint[1] - self.current_pos[1]

            if dist_to_target > 0.3:
                wp_angle = float(np.arctan2(dy_target, dx_target))
                if self.waypoint_received and cmd_speed > YAW_SPEED_DEADBAND and dist_to_target > YAW_DIST_DEADBAND:
                    vel_angle = float(np.arctan2(out_vy, out_vx))
                    diff = vel_angle - wp_angle
                    diff = (diff + np.pi) % (2 * np.pi) - np.pi
                    diff_abs = float(np.abs(diff))
                    if diff_abs > np.pi / 2:
                        blend = min(1.0, (diff_abs - np.pi / 2) / (np.pi / 2))
                        yaw_target = (1.0 - blend) * wp_angle + blend * vel_angle
                    else:
                        yaw_target = wp_angle
                else:
                    yaw_target = wp_angle
            else:
                yaw_target = self.yaw_smooth

            delta_yaw = (yaw_target - self.yaw_smooth + np.pi) % (2 * np.pi) - np.pi
            yaw_step = YAW_FILTER_ALPHA * delta_yaw
            MAX_YAW_STEP = 0.262
            yaw_step = np.clip(yaw_step, -MAX_YAW_STEP, MAX_YAW_STEP)
            self.yaw_smooth += yaw_step
            self.yaw_smooth = (self.yaw_smooth + np.pi) % (2 * np.pi) - np.pi

            yaw_rate = yaw_step / self.dt

            half_yaw = self.yaw_smooth * 0.5
            qw = float(np.cos(half_yaw))
            qz = float(np.sin(half_yaw))

            # Integrate ORCA velocity to target position with smooth blending lookahead
            target_pose = PoseStamped()
            target_pose.header.stamp = self.get_clock().now().to_msg()
            target_pose.header.frame_id = 'world'

            if not hasattr(self, 'pos_ref'):
                self.pos_ref = np.array([self.current_pos[0], self.current_pos[1]], dtype=np.float32)

            self.pos_ref[0] += out_vx * self.dt
            self.pos_ref[1] += out_vy * self.dt

            tracking_err = np.linalg.norm(self.pos_ref - self.current_pos[:2])
            if tracking_err > 0.2:
                self.pos_ref = self.current_pos[:2] + (self.pos_ref - self.current_pos[:2]) * (0.2 / tracking_err)

            # Clamp: jangan melampaui waypoint (anti-overshoot)
            if self.target_waypoint[0] <= self.current_pos[0]:
                self.pos_ref[0] = max(self.pos_ref[0], float(self.target_waypoint[0]))
            if self.target_waypoint[0] > self.current_pos[0]:
                self.pos_ref[0] = min(self.pos_ref[0], float(self.target_waypoint[0]))
            if self.target_waypoint[1] <= self.current_pos[1]:
                self.pos_ref[1] = max(self.pos_ref[1], float(self.target_waypoint[1]))
            if self.target_waypoint[1] > self.current_pos[1]:
                self.pos_ref[1] = min(self.pos_ref[1], float(self.target_waypoint[1]))

            target_pose.pose.position.x = float(self.pos_ref[0] + out_vx * self.lookahead_damping)
            target_pose.pose.position.y = float(self.pos_ref[1] + out_vy * self.lookahead_damping)

            # Kunci presisi mutlak saat sangat dekat (< 0.15m)
            if dist_to_target < 0.15:
                target_pose.pose.position.x = float(self.target_waypoint[0])
                target_pose.pose.position.y = float(self.target_waypoint[1])
                self.cmd_vel_smooth = np.zeros(2, dtype=np.float32)
                self.pos_ref = np.array([float(self.target_waypoint[0]),
                                          float(self.target_waypoint[1])], dtype=np.float32)
                out_vx = 0.0
                out_vy = 0.0
                yaw_rate = 0.0

            target_pose.pose.position.z = float(self.target_z_height)
            target_pose.pose.orientation.x = 0.0
            target_pose.pose.orientation.y = 0.0
            target_pose.pose.orientation.z = qz
            target_pose.pose.orientation.w = qw
            self.pose_pub.publish(target_pose)

            # Publish ORCA velocity sebagai feedforward untuk low-level
            vel_msg = TwistStamped()
            vel_msg.header.stamp = target_pose.header.stamp
            vel_msg.header.frame_id = 'world'
            vel_msg.twist.linear.x = float(out_vx)
            vel_msg.twist.linear.y = float(out_vy)
            vel_msg.twist.linear.z = 0.0
            vel_msg.twist.angular.z = float(yaw_rate)
            self.vel_pub.publish(vel_msg)


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
