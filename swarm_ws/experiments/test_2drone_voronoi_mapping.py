#!/usr/bin/env python3
"""
===============================================================================
  TEST 2-DRONE VORONOI MAPPING — Multi-Agent Boustrophedon Coverage Coordinator
===============================================================================
Fitur Utama:
  1. PARTISI VORONOI HORIZONTAL:
     - Area total: [-6.0, 6.0] x [-6.0, 6.0] m (144 m^2).
     - Sel Bawah: Y in [-5.50, -0.70] m (Baris 1 s/d 5, Y_rows: -5.50, -4.30, -3.10, -1.90, -0.70).
     - Sel Atas:  Y in [+0.50, +5.30] m (Baris 6 s/d 10, Y_rows: +0.50, +1.70, +2.90, +4.10, +5.30).

  2. V2V SMART TASK ASSIGNMENT:
     - Menghitung jarak Euclidean posisi spawn acak kedua drone ke titik awal Partisi Bawah (-5.5, -5.5)
       dan Partisi Atas (-5.5, +0.5).
     - Mengalokasikan partisi secara optimal untuk meminimalkan jarak transit dan mencegah persilangan jalur.

  3. V2V STATE SHARING & 2D COLLISION AVOIDANCE:
     - Cross-monitoring posisi real-time kedua drone (d_12 = ||p1 - p2||).
     - Repulsive separation field aktif jika jarak antar-drone < 1.20m saat transit.

  4. DYNAMIC YAW FOLLOW & CLOSED-LOOP PROGRESS GATING:
     - Hidung kedua drone aktif menghadap arah vektor terbang sepanjang misi.
     - Progress gating independen pada setiap drone.

  5. COMBINED RVIZ2 VISUALIZATION (/mapping/markers):
     - Jejak cakupan gabungan 100x100 grid.
     - Marker visualisasi posisi & sensor bubble masing-masing drone.
===============================================================================
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA


class DroneAgentState:
    """Menyimpan state machine dan telemetri untuk satu drone."""
    def __init__(self, drone_id, name, color_rgba):
        self.id = drone_id
        self.name = name
        self.color = color_rgba
        self.pos = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        self.yaw = 0.0
        self.odom_received = False
        
        # State machine: wait_odom -> wait_takeoff -> transit_to_start -> sweeping_row -> delay_corner_end -> stepping_vertical -> delay_new_row -> done
        self.state = 'wait_odom'
        self.takeoff_timer = 30  # 3 detik stabilisasi
        self.assigned_partition = None  # 'bottom' (Baris 1-5) atau 'top' (Baris 6-10)
        self.y_rows = []
        self.num_rows = 0
        self.row_idx = 0
        
        self.start_pos = np.array([-5.5, -5.5], dtype=np.float32)
        self.ref_pos = np.array([-5.5, -5.5], dtype=np.float32)
        self.delay_remaining = 0
        self.target_yaw = 0.0


class TwoDroneVoronoiMappingNode(Node):
    def __init__(self):
        super().__init__('two_drone_voronoi_mapping_node')
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)

        # ── Parameter Arena & Grid ───────────────────────────────────
        self.x_min, self.x_max = -6.0, 6.0
        self.y_min, self.y_max = -6.0, 6.0
        self.sweep_x_min, self.sweep_x_max = -5.50, 5.50
        self.line_len = self.sweep_x_max - self.sweep_x_min  # 11.0m
        self.row_spacing = 1.20

        # Seluruh 10 baris pada area 12x12m
        self.all_y_rows = np.array([
            -5.50, -4.30, -3.10, -1.90, -0.70,  # Partisi Bawah (Baris 1-5)
            +0.50, +1.70, +2.90, +4.10, +5.30   # Partisi Atas (Baris 6-10)
        ], dtype=np.float32)

        # Partisi Bawah & Atas
        self.bottom_y_rows = self.all_y_rows[0:5]
        self.top_y_rows    = self.all_y_rows[5:10]

        # Parameter Kontrol Kecepatan
        self.nominal_speed   = 0.60   # m/s pada baris lurus
        self.vertical_speed  = 0.35   # m/s pada langkah vertikal
        self.transit_speed   = 0.65   # m/s pada transit awal
        self.lead_dist_x     = 0.45   # m lookahead
        self.kp_pos_x        = 0.85   # gain koreksi posisi X
        self.kp_pos_y        = 1.10   # gain koreksi lateral Y
        self.kp_yaw          = 1.20   # gain orientasi yaw mulus
        self.max_wz          = math.radians(40.0)   # rad/s (~40 deg/s)
        self.corner_delay_ticks = 25  # 2.5 detik settling di sudut (10Hz)

        # Sensor Coverage
        self.sensor_radius = 0.85     # meter
        self.grid_n = 100
        self.cov_grid = np.zeros((self.grid_n, self.grid_n), dtype=bool)
        self.dx = (self.x_max - self.x_min) / self.grid_n
        self.dy = (self.y_max - self.y_min) / self.grid_n

        # ── Inisialisasi 2 Agent ─────────────────────────────────────
        self.agents = {
            1: DroneAgentState(1, 'iris_1', ColorRGBA(r=0.0, g=0.8, b=1.0, a=0.9)),   # Cyan
            2: DroneAgentState(2, 'iris_2', ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.9))    # Orange
        }
        self.assignment_done = False
        self.step_count = 0
        self.min_separation_recorded = 999.0

        # ── ROS 2 Publishers & Subscribers ───────────────────────────
        self.pub_vel = {
            1: self.create_publisher(Twist, '/iris_1/cmd_vel', 10),
            2: self.create_publisher(Twist, '/iris_2/cmd_vel', 10)
        }
        self.pub_markers = self.create_publisher(MarkerArray, '/mapping/markers', 10)

        self.sub_odom_1 = self.create_subscription(
            Odometry, '/iris_1/odometry',
            self.odom_callback_1,
            10
        )
        self.sub_odom_2 = self.create_subscription(
            Odometry, '/iris_2/odometry',
            self.odom_callback_2,
            10
        )

        # Main Loop pada 10 Hz (100 ms)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.marker_timer = self.create_timer(0.2, self.publish_markers)

        self.get_logger().info(
            '========================================================================\n'
            '  🚀 2-DRONE VORONOI BOUSTROPHEDON MAPPING COORDINATOR AKTIF\n'
            '  Area: [-6, 6] x [-6, 6] m | 10 Baris Total (5 Baris/Drone)\n'
            '  Partisi Bawah: Y in [-5.50, -0.70]m | Partisi Atas: Y in [+0.50, +5.30]m\n'
            '  Ketinggian: Z = 2.0m | V2V Real-Time Collision Avoidance Active\n'
            '========================================================================'
        )

    # ── Utilitas Matematika ──────────────────────────────────────────

    @staticmethod
    def euler_from_quaternion(x, y, z, w):
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

    def compute_wz(self, current_yaw, target_yaw):
        diff = target_yaw - current_yaw
        diff = math.atan2(math.sin(diff), math.cos(diff))
        wz = self.kp_yaw * diff
        return float(np.clip(wz, -self.max_wz, self.max_wz))

    def publish_twist(self, drone_id, vx_body, vy_body, wz=0.0):
        msg = Twist()
        msg.linear.x = float(vx_body)
        msg.linear.y = float(vy_body)
        msg.angular.z = float(wz)
        self.pub_vel[drone_id].publish(msg)

    # ── Odometry Callbacks & V2V State Update ───────────────────────
    def odom_callback_1(self, msg):
        self.odom_callback(1, msg)

    def odom_callback_2(self, msg):
        self.odom_callback(2, msg)

    def odom_callback(self, drone_id, msg):
        agent = self.agents[drone_id]
        p = msg.pose.pose.position
        agent.pos = np.array([p.x, p.y, p.z], dtype=np.float32)
        q = msg.pose.pose.orientation
        _, _, agent.yaw = self.euler_from_quaternion(q.x, q.y, q.z, q.w)

        if not agent.odom_received:
            agent.odom_received = True
            agent.state = 'wait_takeoff'
            self.get_logger().info(
                f'[{agent.name}] Odometry terdeteksi di ({p.x:.2f}, {p.y:.2f}, {p.z:.2f}). Menunggu takeoff 3s...'
            )

        # Update coverage grid
        self.update_coverage(agent.pos[0], agent.pos[1])

    def update_coverage(self, x, y):
        col = int((x - self.x_min) / self.dx)
        row = int((y - self.y_min) / self.dy)
        r_cells_x = int(self.sensor_radius / self.dx) + 1
        r_cells_y = int(self.sensor_radius / self.dy) + 1
        r2 = self.sensor_radius ** 2
        for r in range(max(0, row - r_cells_y), min(self.grid_n, row + r_cells_y + 1)):
            for c in range(max(0, col - r_cells_x), min(self.grid_n, col + r_cells_x + 1)):
                cell_x = self.x_min + (c + 0.5) * self.dx
                cell_y = self.y_min + (r + 0.5) * self.dy
                if (cell_x - x) ** 2 + (cell_y - y) ** 2 <= r2:
                    self.cov_grid[r, c] = True

    def get_coverage_percentage(self):
        return (np.count_nonzero(self.cov_grid) / (self.grid_n * self.grid_n)) * 100.0

    # ── V2V Task Assignment ──────────────────────────────────────────

    def perform_voronoi_task_assignment(self):
        """
        Hitung alokasi partisi Voronoi secara optimal berdasarkan posisi awal kedua drone
        untuk meminimalkan total jarak tempuh transit dan mencegah tabrakan persilangan.
        """
        p1 = self.agents[1].pos[:2]
        p2 = self.agents[2].pos[:2]

        start_bottom = np.array([self.sweep_x_min, self.bottom_y_rows[0]], dtype=np.float32)  # (-5.5, -5.5)
        start_top    = np.array([self.sweep_x_min, self.top_y_rows[0]], dtype=np.float32)     # (-5.5, +0.5)

        # Biaya jarak skenario 1: iris_1 -> Bottom, iris_2 -> Top
        cost_1 = np.linalg.norm(p1 - start_bottom) + np.linalg.norm(p2 - start_top)
        # Biaya jarak skenario 2: iris_1 -> Top, iris_2 -> Bottom
        cost_2 = np.linalg.norm(p1 - start_top) + np.linalg.norm(p2 - start_bottom)

        if cost_1 <= cost_2:
            self.assign_partition(1, 'bottom', self.bottom_y_rows, start_bottom)
            self.assign_partition(2, 'top',    self.top_y_rows,    start_top)
            self.get_logger().info('📋 [V2V ASSIGNMENT] iris_1 -> Partisi Bawah (Baris 1-5) | iris_2 -> Partisi Atas (Baris 6-10)')
        else:
            self.assign_partition(1, 'top',    self.top_y_rows,    start_top)
            self.assign_partition(2, 'bottom', self.bottom_y_rows, start_bottom)
            self.get_logger().info('📋 [V2V ASSIGNMENT] iris_1 -> Partisi Atas (Baris 6-10) | iris_2 -> Partisi Bawah (Baris 1-5)')

        self.assignment_done = True

    def assign_partition(self, drone_id, partition_name, y_rows, start_pos):
        agent = self.agents[drone_id]
        agent.assigned_partition = partition_name
        agent.y_rows = y_rows
        agent.num_rows = len(y_rows)
        agent.start_pos = start_pos
        agent.row_idx = 0

    # ── Control Loop Utama (10 Hz) ───────────────────────────────────

    def control_loop(self):
        self.step_count += 1

        # Tunggu sampai kedua drone menerima odometri
        if not (self.agents[1].odom_received and self.agents[2].odom_received):
            return

        # Lakukan Voronoi Task Assignment satu kali setelah kedua drone online
        if not self.assignment_done:
            self.perform_voronoi_task_assignment()

        # Monitor V2V Separation Distance
        p1 = self.agents[1].pos[:2]
        p2 = self.agents[2].pos[:2]
        d_12 = float(np.linalg.norm(p1 - p2))
        if d_12 < self.min_separation_recorded:
            self.min_separation_recorded = d_12

        # Eksekusi state machine untuk masing-masing drone
        for did in (1, 2):
            self.process_agent_step(did, d_12)

        # Logging berkala setiap 1.0 detik (10 ticks)
        if self.step_count % 10 == 0:
            cov = self.get_coverage_percentage()
            a1 = self.agents[1]
            a2 = self.agents[2]
            self.get_logger().info(
                f'📊 [STATUS] Cov: {cov:5.1f}% | Dist_12: {d_12:4.2f}m (Min: {self.min_separation_recorded:4.2f}m) | '
                f'iris_1: {a1.state:16s} ({a1.pos[0]:5.2f}, {a1.pos[1]:5.2f}) | '
                f'iris_2: {a2.state:16s} ({a2.pos[0]:5.2f}, {a2.pos[1]:5.2f})'
            )

    def process_agent_step(self, drone_id, d_12):
        agent = self.agents[drone_id]
        other_id = 2 if drone_id == 1 else 1
        other_agent = self.agents[other_id]

        # 0. Menunggu Takeoff Selesai
        if agent.state == 'wait_takeoff':
            self.publish_twist(drone_id, 0.0, 0.0, 0.0)
            agent.takeoff_timer -= 1
            if agent.takeoff_timer <= 0:
                agent.state = 'transit_to_start'
                self.get_logger().info(
                    f'🛫 [{agent.name}] Takeoff selesai! Memulai Smart Transit ke titik awal ({agent.start_pos[0]:.2f}, {agent.start_pos[1]:.2f})...'
                )
            return

        # 1. SMART TRANSIT — Menuju Titik Awal Partisi Masing-masing dengan V2V Repulsion
        if agent.state == 'transit_to_start':
            target_x = agent.start_pos[0]
            target_y = agent.start_pos[1]
            dx = target_x - float(agent.pos[0])
            dy = target_y - float(agent.pos[1])
            dist_to_start = math.hypot(dx, dy)

            # Arah target transit
            angle_to_start = math.atan2(dy, dx)
            wz_cmd = self.compute_wz(agent.yaw, angle_to_start)

            # Cek apakah sudah tiba di titik awal
            if dist_to_start < 0.25:
                # Selaraskan yaw ke 0.0 derajat sebelum mulai sapuan baris 1
                yaw_err = math.atan2(math.sin(0.0 - agent.yaw), math.cos(0.0 - agent.yaw))
                if abs(yaw_err) < math.radians(10.0):
                    self.publish_twist(drone_id, 0.0, 0.0, 0.0)
                    agent.state = 'sweeping_row'
                    agent.row_idx = 0
                    self.get_logger().info(
                        f'✅ [{agent.name}] Tiba di Titik Awal Partisi ({agent.pos[0]:.2f}, {agent.pos[1]:.2f})! '
                        f'Memulai Baris 1/{agent.num_rows}: Sweep ({self.sweep_x_min:.2f} -> {self.sweep_x_max:.2f}, Y={agent.y_rows[0]:.2f})'
                    )
                    return
                else:
                    wz_align = self.compute_wz(agent.yaw, 0.0)
                    self.publish_twist(drone_id, 0.0, 0.0, wz_align)
                    return

            # Kecepatan transit dasar
            speed = min(self.transit_speed, max(0.20, 0.85 * dist_to_start))
            v_world_x = (dx / dist_to_start) * speed
            v_world_y = (dy / dist_to_start) * speed

            # V2V Repulsive Field jika kedua drone terlalu dekat (< 1.20m) saat transit
            if d_12 < 1.20:
                p_self = agent.pos[:2]
                p_other = other_agent.pos[:2]
                diff_vec = p_self - p_other
                dist_norm = np.linalg.norm(diff_vec)
                if dist_norm > 0.01:
                    rep_strength = (1.20 - dist_norm) * 0.50
                    v_world_x += (diff_vec[0] / dist_norm) * rep_strength
                    v_world_y += (diff_vec[1] / dist_norm) * rep_strength

            # Transformasi World ke Body Frame
            cos_y = math.cos(agent.yaw)
            sin_y = math.sin(agent.yaw)
            v_body_x =  v_world_x * cos_y + v_world_y * sin_y
            v_body_y = -v_world_x * sin_y + v_world_y * cos_y
            self.publish_twist(drone_id, v_body_x, v_body_y, wz_cmd)

            agent.ref_pos = np.array([target_x, target_y], dtype=np.float32)
            return

        # 2. SWEEPING ROW — Closed-Loop Tracking dengan Progress Gating
        if agent.state == 'sweeping_row':
            go_right = (agent.row_idx % 2 == 0)
            x_start = self.sweep_x_min if go_right else self.sweep_x_max
            x_end   = self.sweep_x_max if go_right else self.sweep_x_min
            y_curr  = float(agent.y_rows[agent.row_idx])
            dir_x   = 1.0 if go_right else -1.0

            target_yaw = 0.0 if go_right else math.pi
            wz_cmd = self.compute_wz(agent.yaw, target_yaw)

            # Hitung progress aktual sepanjang garis
            if go_right:
                actual_prog = float(agent.pos[0]) - self.sweep_x_min
            else:
                actual_prog = self.sweep_x_max - float(agent.pos[0])
            actual_prog = max(0.0, min(self.line_len, actual_prog))

            e_y_current = y_curr - float(agent.pos[1])

            # Progress Gating: Jika deviasi lateral > 0.25m, jangan dorong titik referensi ke depan
            if abs(e_y_current) > 0.25:
                s_target = actual_prog
                ff_scale = 0.30
            else:
                s_target = min(self.line_len, actual_prog + self.lead_dist_x)
                dist_to_end_line = self.line_len - actual_prog
                if dist_to_end_line > 1.5:
                    ff_scale = 1.0
                elif dist_to_end_line > 0.1:
                    ff_scale = dist_to_end_line / 1.5
                else:
                    ff_scale = 0.0

            ref_x = x_start + s_target * dir_x
            ref_y = y_curr
            agent.ref_pos = np.array([ref_x, ref_y], dtype=np.float32)

            e_x = ref_x - float(agent.pos[0])
            e_y = ref_y - float(agent.pos[1])

            v_ff_x = self.nominal_speed * dir_x * ff_scale
            v_ff_y = 0.0

            v_cmd_x = v_ff_x + self.kp_pos_x * e_x
            v_cmd_y = v_ff_y + self.kp_pos_y * e_y

            # Batasi kecepatan maksimum
            v_mag = math.hypot(v_cmd_x, v_cmd_y)
            if v_mag > self.nominal_speed * 1.3:
                v_cmd_x = (v_cmd_x / v_mag) * (self.nominal_speed * 1.3)
                v_cmd_y = (v_cmd_y / v_mag) * (self.nominal_speed * 1.3)

            cos_y = math.cos(agent.yaw)
            sin_y = math.sin(agent.yaw)
            v_body_x =  v_cmd_x * cos_y + v_cmd_y * sin_y
            v_body_y = -v_cmd_x * sin_y + v_cmd_y * cos_y
            self.publish_twist(drone_id, v_body_x, v_body_y, wz_cmd)

            # Cek ketercapaian ujung baris
            dist_to_end = abs(float(agent.pos[0]) - x_end)
            dist_to_end_line = self.line_len - actual_prog
            if (dist_to_end < 0.22 or dist_to_end_line < 0.15) and abs(e_y_current) < 0.20:
                self.get_logger().info(
                    f'  -> [{agent.name}] Ujung Baris {agent.row_idx + 1}/{agent.num_rows} tercapai ({agent.pos[0]:.2f}, {agent.pos[1]:.2f}). Jeda sudut 1.5s'
                )
                self.publish_twist(drone_id, 0.0, 0.0, 0.0)
                agent.state = 'delay_corner_end'
                agent.delay_remaining = self.corner_delay_ticks
            return

        # 3. DELAY AT CORNER END — Jeda & rotasi in-place ke +90 deg (+Y)
        if agent.state == 'delay_corner_end':
            target_yaw = math.pi / 2.0
            wz_cmd = self.compute_wz(agent.yaw, target_yaw)
            self.publish_twist(drone_id, 0.0, 0.0, wz_cmd)
            agent.delay_remaining -= 1

            yaw_diff = math.atan2(math.sin(target_yaw - agent.yaw), math.cos(target_yaw - agent.yaw))
            if agent.delay_remaining <= 0 and abs(yaw_diff) < math.radians(8.0):
                if agent.row_idx >= agent.num_rows - 1:
                    agent.state = 'done'
                    self.get_logger().info(
                        f'🎉 [{agent.name}] PEMETAAN PARTISI TUNTAS SEMPURNA ({agent.num_rows} Baris)!'
                    )
                else:
                    agent.state = 'stepping_vertical'
                    y_next = float(agent.y_rows[agent.row_idx + 1])
                    self.get_logger().info(
                        f'  -> [{agent.name}] Melangkah vertikal ke Baris {agent.row_idx + 2} (Y={y_next:.2f})'
                    )
            return

        # 4. STEPPING VERTICAL — Continuous smooth cmd_vel ke baris berikutnya
        if agent.state == 'stepping_vertical':
            go_right = (agent.row_idx % 2 == 0)
            x_corner = self.sweep_x_max if go_right else self.sweep_x_min
            y_from   = float(agent.y_rows[agent.row_idx])
            y_target = float(agent.y_rows[agent.row_idx + 1])

            target_yaw = math.pi / 2.0
            wz_cmd = self.compute_wz(agent.yaw, target_yaw)

            prog_y = float(agent.pos[1]) - y_from
            prog_y = max(0.0, min(self.row_spacing, prog_y))

            lead_dist_y = 0.35
            s_target_y = min(self.row_spacing, prog_y + lead_dist_y)
            ref_y = y_from + s_target_y
            ref_x = x_corner
            agent.ref_pos = np.array([ref_x, ref_y], dtype=np.float32)

            e_x = ref_x - float(agent.pos[0])
            e_y = ref_y - float(agent.pos[1])

            dist_to_end_y = self.row_spacing - prog_y
            ff_scale_y = min(1.0, dist_to_end_y / 0.40) if dist_to_end_y > 0.05 else 0.0

            v_cmd_x = self.kp_pos_x * e_x
            v_cmd_y = self.vertical_speed * ff_scale_y + self.kp_pos_y * e_y

            cos_y = math.cos(agent.yaw)
            sin_y = math.sin(agent.yaw)
            v_body_x =  v_cmd_x * cos_y + v_cmd_y * sin_y
            v_body_y = -v_cmd_x * sin_y + v_cmd_y * cos_y
            self.publish_twist(drone_id, v_body_x, v_body_y, wz_cmd)

            dist_to_target_y = abs(float(agent.pos[1]) - y_target)
            if (dist_to_target_y < 0.15 or dist_to_end_y < 0.10) and abs(e_x) < 0.20:
                self.publish_twist(drone_id, 0.0, 0.0, 0.0)
                agent.state = 'delay_new_row'
                agent.delay_remaining = self.corner_delay_ticks
                self.get_logger().info(
                    f'  -> [{agent.name}] Titik awal Baris {agent.row_idx + 2} tercapai! Jeda settling 1.5s'
                )
            return

        # 5. DELAY AT NEW ROW — Rotasi in-place ke arah sapuan baris baru
        if agent.state == 'delay_new_row':
            next_go_right = ((agent.row_idx + 1) % 2 == 0)
            target_yaw = 0.0 if next_go_right else math.pi
            wz_cmd = self.compute_wz(agent.yaw, target_yaw)
            self.publish_twist(drone_id, 0.0, 0.0, wz_cmd)
            agent.delay_remaining -= 1

            yaw_diff = math.atan2(math.sin(target_yaw - agent.yaw), math.cos(target_yaw - agent.yaw))
            if agent.delay_remaining <= 0 and abs(yaw_diff) < math.radians(8.0):
                agent.row_idx += 1
                agent.state = 'sweeping_row'
                go_right = (agent.row_idx % 2 == 0)
                x_start = self.sweep_x_min if go_right else self.sweep_x_max
                x_end   = self.sweep_x_max if go_right else self.sweep_x_min
                y_curr  = float(agent.y_rows[agent.row_idx])
                self.get_logger().info(
                    f'[{agent.name}] Memulai Baris {agent.row_idx + 1}/{agent.num_rows}: '
                    f'Sweep ({x_start:.2f} -> {x_end:.2f}, Y={y_curr:.2f}) [Continuous cmd_vel]'
                )
            return

        # 6. Selesai
        if agent.state == 'done':
            self.publish_twist(drone_id, 0.0, 0.0, 0.0)

    # ── RViz2 Markers Visualization ──────────────────────────────────

    def publish_markers(self):
        ma = MarkerArray()
        now_msg = self.get_clock().now().to_msg()

        # 1. Boundary Grid Map (Arena 12x12m)
        m_bound = Marker()
        m_bound.header.frame_id = 'world'
        m_bound.header.stamp = now_msg
        m_bound.ns = 'mapping_boundary'
        m_bound.id = 0
        m_bound.type = Marker.LINE_STRIP
        m_bound.action = Marker.ADD
        m_bound.scale.x = 0.08
        m_bound.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.8)
        corners = [
            (self.x_min, self.y_min), (self.x_max, self.y_min),
            (self.x_max, self.y_max), (self.x_min, self.y_max),
            (self.x_min, self.y_min)
        ]
        for cx, cy in corners:
            m_bound.points.append(Point(x=float(cx), y=float(cy), z=0.0))
        ma.markers.append(m_bound)

        # 2. Garis Batas Voronoi (Y = 0.0m)
        m_voro = Marker()
        m_voro.header.frame_id = 'world'
        m_voro.header.stamp = now_msg
        m_voro.ns = 'voronoi_boundary'
        m_voro.id = 1
        m_voro.type = Marker.LINE_STRIP
        m_voro.action = Marker.ADD
        m_voro.scale.x = 0.06
        m_voro.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.9)  # Kuning
        m_voro.points.append(Point(x=float(self.x_min), y=0.0, z=0.05))
        m_voro.points.append(Point(x=float(self.x_max), y=0.0, z=0.05))
        ma.markers.append(m_voro)

        # 3. Sensor Footprint & Minimalist Nameplate per drone
        for did in (1, 2):
            agent = self.agents[did]
            ax, ay, az = float(agent.pos[0]), float(agent.pos[1]), float(agent.pos[2])

            # 3a. Sensor Ground Footprint
            m_foot = Marker()
            m_foot.header.frame_id = 'world'
            m_foot.header.stamp = now_msg
            m_foot.ns = f'drone_{did}_footprint'
            m_foot.id = 10 + did
            m_foot.type = Marker.CYLINDER
            m_foot.action = Marker.ADD
            m_foot.pose.position.x = ax
            m_foot.pose.position.y = ay
            m_foot.pose.position.z = 0.02
            m_foot.scale.x = float(self.sensor_radius * 2.0)
            m_foot.scale.y = float(self.sensor_radius * 2.0)
            m_foot.scale.z = 0.03
            m_foot.color = agent.color
            ma.markers.append(m_foot)

            # 3b. Simple Name Tag
            m_name = Marker()
            m_name.header.frame_id = 'world'
            m_name.header.stamp = now_msg
            m_name.ns = f'drone_{did}_name'
            m_name.id = 20 + did
            m_name.type = Marker.TEXT_VIEW_FACING
            m_name.action = Marker.ADD
            m_name.pose.position.x = ax
            m_name.pose.position.y = ay
            m_name.pose.position.z = az + 0.35
            m_name.scale.z = 0.25
            m_name.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
            m_name.text = f'iris_{did}'
            ma.markers.append(m_name)

        # 4. Total Combined Occupancy Coverage Map (Points)
        m_cov = Marker()
        m_cov.header.frame_id = 'world'
        m_cov.header.stamp = now_msg
        m_cov.ns = 'combined_coverage_cells'
        m_cov.id = 100
        m_cov.type = Marker.POINTS
        m_cov.action = Marker.ADD
        m_cov.scale.x = float(self.dx * 1.05)
        m_cov.scale.y = float(self.dy * 1.05)
        m_cov.color = ColorRGBA(r=0.1, g=0.95, b=0.2, a=0.45)  # Hijau Terang

        covered_indices = np.argwhere(self.cov_grid)
        for r, c in covered_indices:
            cx = self.x_min + (c + 0.5) * self.dx
            cy = self.y_min + (r + 0.5) * self.dy
            m_cov.points.append(Point(x=float(cx), y=float(cy), z=0.01))
        ma.markers.append(m_cov)

        # 5. Minimalist Header Text
        pct = self.get_coverage_percentage()
        m_text = Marker()
        m_text.header.frame_id = 'world'
        m_text.header.stamp = now_msg
        m_text.ns = 'hud_text'
        m_text.id = 200
        m_text.type = Marker.TEXT_VIEW_FACING
        m_text.action = Marker.ADD
        m_text.pose.position.x = 0.0
        m_text.pose.position.y = 6.4
        m_text.pose.position.z = 0.5
        m_text.scale.z = 0.40
        m_text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
        m_text.text = f'Coverage: {pct:5.1f}%'
        ma.markers.append(m_text)

        self.pub_markers.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = TwoDroneVoronoiMappingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Pemetaan dihentikan manual oleh pengguna (Ctrl+C).')
    finally:
        for did in (1, 2):
            node.publish_twist(did, 0.0, 0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
