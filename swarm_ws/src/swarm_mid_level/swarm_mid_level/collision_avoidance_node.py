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
            rad_obs = 0.4 if is_static else self.radius
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
        self.declare_parameter('safety_radius', 0.5)
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
        self.target_waypoint = None                      # [x, y]
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

    def odom_callback(self, msg):
        self.current_pos[0] = msg.pose.pose.position.x
        self.current_pos[1] = msg.pose.pose.position.y
        self.current_pos[2] = msg.pose.pose.position.z

        self.current_vel[0] = msg.twist.twist.linear.x
        self.current_vel[1] = msg.twist.twist.linear.y

        if self.target_waypoint is None:
            self.target_waypoint = np.array([self.current_pos[0], self.current_pos[1]], dtype=np.float32)
            self.get_logger().info(
                f"[ORCA] Target awal di-set ke lokasi spawn: X={self.target_waypoint[0]:.2f}, Y={self.target_waypoint[1]:.2f}"
            )

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

        # 0. Takeoff phase check
        if self.current_pos[2] < 1.5 and not self.waypoint_received:
            target_pose = PoseStamped()
            target_pose.header.stamp = self.get_clock().now().to_msg()
            target_pose.header.frame_id = 'world'
            target_pose.pose.position.x = float(self.target_waypoint[0])
            target_pose.pose.position.y = float(self.target_waypoint[1])
            target_pose.pose.position.z = float(self.target_z_height)
            target_pose.pose.orientation.w = 1.0
            self.pose_pub.publish(target_pose)
            return

        # 1. Calculate Preferred Velocity towards Target Waypoint
        rel_target = self.target_waypoint - self.current_pos[:2]
        dist_to_target = float(np.linalg.norm(rel_target))

        if dist_to_target < 0.1:
            pref_vel = np.zeros(2, dtype=np.float32)
        else:
            # Proportional velocity targeting max_speed
            speed = min(self.max_speed, dist_to_target * 1.5)
            pref_vel = (rel_target / dist_to_target) * speed

        # 2. Extract neighbor drone states
        neighbor_list = list(self.neighbors_state.values())

        # 3. Extract static Lidar obstacles as a single convex ORCA agent with tangential bias
        angles = np.linspace(-np.pi, np.pi, len(self.lidar_ranges))
        obs_mask = self.lidar_ranges < 2.5

        if np.any(obs_mask):
            min_idx = np.argmin(self.lidar_ranges)
            dist_min = float(self.lidar_ranges[min_idx])
            angle_min = float(angles[min_idx])

            # Position of static obstacle center
            obs_rel = np.array([dist_min * np.cos(angle_min), dist_min * np.sin(angle_min)], dtype=np.float32)
            neighbor_list.append({
                'pos': self.current_pos[:2] + obs_rel,
                'vel': np.zeros(2, dtype=np.float32),
                'is_static': True
            })

            # Tangential bias: jika rintangan berada tepat di jalur depan, beri geseran bias kecil pada pref_vel
            # agar ORCA tidak terjebak di tengah (deadlock/local minima)
            obs_dir = obs_rel / max(dist_min, 0.05)
            dot_front = np.dot(pref_vel / max(np.linalg.norm(pref_vel), 0.1), obs_dir)
            if dot_front > 0.5:
                # Pilih arah belok memutar (tangensial): jika rintangan agak di kanan, bias ke kiri; vice versa
                tangent = np.array([-obs_dir[1], obs_dir[0]], dtype=np.float32)
                if np.cross(pref_vel, obs_dir) > 0:
                    tangent = -tangent
                pref_vel += tangent * (self.max_speed * 0.4)

        # 4. Compute ORCA Reciprocal Safe Velocity
        safe_vel = self.orca_solver.compute_orca_velocity(
            pos_self=self.current_pos[:2],
            vel_self=self.current_vel,
            pref_vel=pref_vel,
            neighbors=neighbor_list,
            lidar_lines=None
        )

        # 5. Low-Pass Velocity Filter & Slew Rate Limiter (mencegah RPM saturation & drone terbalik)
        ref_vx = np.clip(safe_vel[0], -self.max_speed, self.max_speed)
        ref_vy = np.clip(safe_vel[1], -1.0, 1.0) # Cap lateral speed for pitch/roll stability

        if not hasattr(self, 'cmd_vel_smooth'):
            self.cmd_vel_smooth = np.array([ref_vx, ref_vy], dtype=np.float32)
        else:
            self.cmd_vel_smooth = 0.75 * self.cmd_vel_smooth + 0.25 * np.array([ref_vx, ref_vy], dtype=np.float32)

        out_vx, out_vy = self.cmd_vel_smooth[0], self.cmd_vel_smooth[1]

        # Debug log every ~2s
        if self.steps % 20 == 0:
            self.get_logger().info(
                f"[ORCA] Pos=({self.current_pos[0]:.2f},{self.current_pos[1]:.2f}) "
                f"Target=({self.target_waypoint[0]:.1f},{self.target_waypoint[1]:.1f}) "
                f"Dist={dist_to_target:.2f}m | PrefVel=({pref_vel[0]:.2f},{pref_vel[1]:.2f}) "
                f"→ ORCA Vel=({out_vx:.2f},{out_vy:.2f})"
            )

        # 6. Integrate ORCA velocity to position command for low-level controller
        target_pose = PoseStamped()
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.header.frame_id = 'world'

        target_pose.pose.position.x = float(self.current_pos[0] + out_vx * self.dt)
        target_pose.pose.position.y = float(self.current_pos[1] + out_vy * self.dt)
        target_pose.pose.position.z = float(self.target_z_height)
        target_pose.pose.orientation.w = 1.0

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
