#!/usr/bin/env python3
"""
===============================================================================
  TEST MAPPING — Pure Continuous Boustrophedon Mapping Coordinator (v5)
===============================================================================
Fitur Utama v5:
  1. ARBITRARY / RANDOM SPAWN SUPPORT & SMART TRANSIT:
     - Drone dapat di-spawn di koordinat mana pun dalam arena.
     - Jika jarak ke titik awal Baris 1 (-5.50, -5.50) > 0.25m, node otomatis masuk
       ke state TRANSIT_TO_START.
     - Hidung drone aktif berputar menghadap arah target transit:
       psi = atan2(y_start - y_drone, x_start - x_drone).
     - Setelah sampai di titik start, hidung berputar selaras ke 0.0° sebelum
       memulai sapuan Baris 1.

  2. ALWAYS DYNAMIC YAW FOLLOW:
     - Sepanjang seluruh tahapan misi (transit, sweep lurus, belokan sudut, dan
       langkah vertikal), hidung drone selalu aktif menghadap arah vektor terbang.
     - Rate limit wz = 60°/s untuk stabilitas aerodinamika & bebas jerk.

  3. HIGH-LEVEL CLOSED-LOOP WAITING (ANTI-TINGGAL):
     - Kemajuan waypoint high-level selalu terikat secara closed-loop dengan
       posisi nyata drone (Cross-Track & Along-Track progress gating).
     - Jika drone melambat atau terhalang, high-level MENUNGGU posisi riil drone
       dan tidak akan pernah meninggalkan drone.

  4. CONTINUOUS SMOOTH 100% CMD_VEL TRACKING:
     - Kecepatan kontinu nominal v=0.60 m/s pada baris lurus, v=0.35 m/s pada langkah vertikal.
     - Deselerasi linear mulus di ujung baris tanpa memicu lonjakan kontroler.

Grid: 10 baris, spacing 1.20m, Y: -5.50 -> 5.30, X: -5.50 -> 5.50
Visualisasi: RViz2 /mapping/markers (boundary, path, active ref, tracking vector,
             coverage footprint, HUD text overlay)
===============================================================================
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA


class PureContinuousMappingNode(Node):
    def __init__(self):
        super().__init__('pure_continuous_mapping_node')

        # ── Parameter Wilayah Pemetaan ───────────────────────────────
        self.x_min, self.x_max = -6.0, 6.0
        self.y_min, self.y_max = -6.0, 6.0
        self.cruise_alt = 2.0
        self.sensor_radius = 0.80

        self.boundary_poly = np.array([
            [self.x_min, self.y_min],
            [self.x_max, self.y_min],
            [self.x_max, self.y_max],
            [self.x_min, self.y_max]
        ], dtype=float)

        # ── Grid Boustrophedon ───────────────────────────────────────
        self.sweep_x_min = -5.50
        self.sweep_x_max =  5.50
        self.sweep_spacing = 1.20
        self.y_rows = np.arange(-5.50, 5.50 + 1e-4, self.sweep_spacing)
        self.num_rows = len(self.y_rows)
        self.line_len = abs(self.sweep_x_max - self.sweep_x_min)  # 11.0m

        # ── Tuning Kecepatan & Kontrol ────────────────────────────────
        self.nominal_speed = 0.60       # m/s sweep lurus mulus
        self.step_speed = 0.35          # m/s langkah vertikal halus
        self.transit_speed = 0.60       # m/s transit ke titik start
        self.kp_track = 0.90            # gain proporsional tracking
        self.max_cmd_speed = 0.95       # saturasi kecepatan sweep
        self.max_step_speed = 0.55      # saturasi kecepatan vertikal
        self.lead_dist_x = 0.45         # lead point sweep (m)
        self.lead_dist_y = 0.35         # lead point step (m)

        # ── Coverage Grid (60 x 60 cells) ────────────────────────────
        self.grid_n = 60
        self.cov_grid = np.zeros((self.grid_n, self.grid_n), dtype=bool)
        self.dx = (self.x_max - self.x_min) / self.grid_n
        self.dy = (self.y_max - self.y_min) / self.grid_n

        # ── Yaw Mode Configuration (Default: follow_path) ────────────
        self.declare_parameter('yaw_mode', 'follow_path')
        self.yaw_mode = self.get_parameter('yaw_mode').get_parameter_value().string_value
        import sys
        for arg in sys.argv:
            if arg in ['--yaw-follow', '--follow', 'yaw_mode:=follow_path']:
                self.yaw_mode = 'follow_path'
            elif arg in ['--fixed-yaw', '--fixed', 'yaw_mode:=fixed']:
                self.yaw_mode = 'fixed'

        # ── State Machine ────────────────────────────────────────────
        #   wait_odom -> wait_takeoff -> [transit_to_start -> align_start_yaw]
        #   -> sweeping_row -> delay_at_corner_end -> stepping_vertical
        #   -> delay_at_new_row -> sweeping_row -> ... -> done
        self.drone_pos = np.array([-5.50, -5.50, 2.00], dtype=np.float32)
        self.drone_yaw = 0.0
        self.odom_received = False
        self.state = 'wait_odom'
        self.takeoff_timer = 0
        self.delay_remaining = 0
        # 1.5s jeda di sudut jika follow_path (untuk rotasi yaw mulus), 0.5s jika fixed yaw
        self.corner_delay_ticks = 15 if self.yaw_mode == 'follow_path' else 5
        self.row_idx = 0
        self.ref_pos = np.array([-5.50, -5.50], dtype=np.float32)
        self.step_count = 0

        # ── Publishers & Subscribers ─────────────────────────────────
        self.pub_vel = self.create_publisher(Twist, '/iris_1/cmd_vel', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/mapping/markers', 10)
        self.sub_odom = self.create_subscription(Odometry, '/iris_1/odometry', self.odom_callback, 10)

        # Loop Timer 10 Hz
        self.timer_control = self.create_timer(0.10, self.control_loop)
        self.timer_viz = self.create_timer(0.10, self.publish_rviz_markers)

        self.get_logger().info(
            f'Pure Continuous Mapping v5 Siap! [Yaw Mode: {self.yaw_mode.upper()}] '
            f'({self.num_rows} Baris, v_sweep={self.nominal_speed:.2f}m/s, '
            f'Y=[{float(self.y_rows[0]):.2f} -> {float(self.y_rows[-1]):.2f}])'
        )

    # ── Utilitas ─────────────────────────────────────────────────────

    def compute_wz(self, target_yaw):
        """Hitung laju yaw (rad/s) mulus dengan shortest-path circular diff."""
        if self.yaw_mode != 'follow_path':
            return 0.0
        diff = math.atan2(math.sin(target_yaw - self.drone_yaw), math.cos(target_yaw - self.drone_yaw))
        MAX_WZ = math.radians(60.0)  # Max 60 deg/s untuk mencegah gyroscopic jerk
        wz = np.clip(1.5 * diff, -MAX_WZ, MAX_WZ)
        return float(wz)

    def euler_from_quaternion(self, qx, qy, qz, qw):
        t3 = 2.0 * (qw * qz + qx * qy)
        t4 = 1.0 - 2.0 * (qy * qy + qz * qz)
        return math.atan2(t3, t4)

    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        self.drone_pos = np.array([p.x, p.y, p.z], dtype=np.float32)
        q = msg.pose.pose.orientation
        self.drone_yaw = self.euler_from_quaternion(q.x, q.y, q.z, q.w)

        if not self.odom_received:
            self.odom_received = True
            self.state = 'wait_takeoff'
            self.takeoff_timer = 30  # 3 detik stabilisasi takeoff
            self.get_logger().info(
                f'Odometry diterima di ({p.x:.2f}, {p.y:.2f}, {p.z:.2f}). Menunggu takeoff 3s...'
            )

        self.update_coverage(self.drone_pos[0], self.drone_pos[1])

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

    def get_actual_progress_x(self):
        """Hitung progress AKTUAL drone sepanjang garis sweep X saat ini."""
        go_right = (self.row_idx % 2 == 0)
        if go_right:
            prog = float(self.drone_pos[0]) - self.sweep_x_min
        else:
            prog = self.sweep_x_max - float(self.drone_pos[0])
        return max(0.0, min(self.line_len, prog))

    def publish_twist(self, vx_body, vy_body, wz=0.0):
        """Kirim cmd_vel body-frame ke mid-level."""
        msg = Twist()
        msg.linear.x = float(vx_body)
        msg.linear.y = float(vy_body)
        msg.angular.z = float(wz)
        self.pub_vel.publish(msg)

    # ── Control Loop Utama ───────────────────────────────────────────

    def control_loop(self):
        self.step_count += 1

        if self.state == 'wait_odom':
            return

        # 0. Menunggu Takeoff & Evaluasi Titik Awal
        if self.state == 'wait_takeoff':
            self.publish_twist(0.0, 0.0, 0.0)
            self.takeoff_timer -= 1
            if self.takeoff_timer <= 0:
                x_start = self.sweep_x_min
                y_start = float(self.y_rows[0])
                dist_to_start = math.sqrt((float(self.drone_pos[0]) - x_start)**2 + (float(self.drone_pos[1]) - y_start)**2)
                
                if dist_to_start > 0.25:
                    self.state = 'transit_to_start'
                    self.get_logger().info(
                        f'Takeoff selesai di ({self.drone_pos[0]:.2f}, {self.drone_pos[1]:.2f})! '
                        f'Jarak ke titik awal Baris 1 ({x_start:.2f}, {y_start:.2f}): {dist_to_start:.2f}m. '
                        f'Memulai Smart Transit...'
                    )
                else:
                    self.state = 'align_start_yaw'
                    self.delay_remaining = self.corner_delay_ticks
                    self.get_logger().info(
                        f'Takeoff selesai di titik awal ({self.drone_pos[0]:.2f}, {self.drone_pos[1]:.2f}). '
                        f'Menyelaraskan heading ke 0.0°...'
                    )
            return

        # ─────────────────────────────────────────────────────────────
        # 1. TRANSIT TO START — Terbang langsung ke (-5.5, -5.5) dengan Yaw Hadap Transit
        # ─────────────────────────────────────────────────────────────
        if self.state == 'transit_to_start':
            x_start = self.sweep_x_min
            y_start = float(self.y_rows[0])
            dx = x_start - float(self.drone_pos[0])
            dy = y_start - float(self.drone_pos[1])
            dist = math.sqrt(dx**2 + dy**2)

            # Target yaw aktif menghadap langsung ke arah vektor titik start
            target_yaw = math.atan2(dy, dx)
            wz_cmd = self.compute_wz(target_yaw)

            # Profil kecepatan halus dengan deselerasi pada 1.5m terakhir
            if dist > 1.5:
                v_nom = self.transit_speed
            elif dist > 0.15:
                v_nom = max(0.20, self.transit_speed * (dist / 1.5))
            else:
                v_nom = 0.0

            # World-frame velocity command (Unit vector * nominal speed + Kp proportional)
            if dist > 1e-3:
                v_cmd_x = v_nom * (dx / dist) + self.kp_track * dx
                v_cmd_y = v_nom * (dy / dist) + self.kp_track * dy
            else:
                v_cmd_x = self.kp_track * dx
                v_cmd_y = self.kp_track * dy

            spd = math.sqrt(v_cmd_x**2 + v_cmd_y**2)
            if spd > self.max_cmd_speed:
                v_cmd_x = (v_cmd_x / spd) * self.max_cmd_speed
                v_cmd_y = (v_cmd_y / spd) * self.max_cmd_speed

            # Transformasi ke Body Frame
            cos_y = math.cos(self.drone_yaw)
            sin_y = math.sin(self.drone_yaw)
            v_body_x =  v_cmd_x * cos_y + v_cmd_y * sin_y
            v_body_y = -v_cmd_x * sin_y + v_cmd_y * cos_y
            self.publish_twist(v_body_x, v_body_y, wz_cmd)

            self.ref_pos = np.array([x_start, y_start], dtype=np.float32)

            # Telemetri setiap 1 detik
            if self.step_count % 10 == 0:
                self.get_logger().info(
                    f'  [TRANSIT] Pos: ({self.drone_pos[0]:5.2f}, {self.drone_pos[1]:5.2f}) | '
                    f'Target: ({x_start:5.2f}, {y_start:5.2f}) | Sisa: {dist:4.2f}m | '
                    f'Yaw: {math.degrees(self.drone_yaw):+5.1f}° -> Target: {math.degrees(target_yaw):+5.1f}°'
                )

            # Cek ketercapaian titik awal
            if dist < 0.18:
                self.get_logger().info(
                    f'  -> Tiba di Titik Start Baris 1 ({self.drone_pos[0]:.2f}, {self.drone_pos[1]:.2f})! '
                    f'Menyelaraskan heading ke arah sapuan Baris 1 (0.0°)...'
                )
                self.publish_twist(0.0, 0.0, 0.0)
                self.state = 'align_start_yaw'
                self.delay_remaining = self.corner_delay_ticks
            return

        # ─────────────────────────────────────────────────────────────
        # 2. ALIGN START YAW — Penyelarasan orientasi ke 0.0° sebelum Baris 1
        # ─────────────────────────────────────────────────────────────
        if self.state == 'align_start_yaw':
            target_yaw = 0.0  # Baris 1 selalu bergerak ke +X (East, 0.0 rad)
            wz_cmd = self.compute_wz(target_yaw)
            self.publish_twist(0.0, 0.0, wz_cmd)
            self.delay_remaining -= 1

            yaw_diff = math.atan2(math.sin(target_yaw - self.drone_yaw), math.cos(target_yaw - self.drone_yaw))
            if self.delay_remaining <= 0 and abs(yaw_diff) < math.radians(8.0):
                self.state = 'sweeping_row'
                self.row_idx = 0
                x_start = self.sweep_x_min
                x_end   = self.sweep_x_max
                y_curr  = float(self.y_rows[0])
                self.get_logger().info(
                    f'Heading selaras ({math.degrees(self.drone_yaw):.1f}°)! Memulai Baris 1/{self.num_rows}: '
                    f'Sweep ({x_start:.2f} -> {x_end:.2f}, Y={y_curr:.2f}) [Continuous cmd_vel]'
                )
            return

        # ─────────────────────────────────────────────────────────────
        # 3. SWEEPING ROW — Closed-Loop Tracking dengan Anti-Tinggal
        # ─────────────────────────────────────────────────────────────
        if self.state == 'sweeping_row':
            go_right = (self.row_idx % 2 == 0)
            x_start = self.sweep_x_min if go_right else self.sweep_x_max
            x_end   = self.sweep_x_max if go_right else self.sweep_x_min
            y_curr  = float(self.y_rows[self.row_idx])
            dir_x   = 1.0 if go_right else -1.0

            # Target yaw: 0.0 jika ke kanan (+X), pi jika ke kiri (-X)
            target_yaw = 0.0 if go_right else math.pi
            wz_cmd = self.compute_wz(target_yaw)

            actual_prog = self.get_actual_progress_x()
            e_y_current = y_curr - float(self.drone_pos[1])

            # Progress Gating: Jika deviasi lateral > 0.25m, jangan dorong X ke depan (tunggu drone merapat)
            if abs(e_y_current) > 0.25:
                s_target = actual_prog
                ff_scale = 0.30  # Reduksi feedforward X agar drone fokus koreksi lateral
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
            self.ref_pos = np.array([ref_x, ref_y], dtype=np.float32)

            e_x = ref_x - float(self.drone_pos[0])
            e_y = ref_y - float(self.drone_pos[1])

            v_ff_x = self.nominal_speed * dir_x * ff_scale
            v_ff_y = 0.0

            # Combined velocity command (Feedforward + Kp tracking)
            v_cmd_x = v_ff_x + self.kp_track * e_x
            v_cmd_y = v_ff_y + self.kp_track * e_y

            spd = math.sqrt(v_cmd_x ** 2 + v_cmd_y ** 2)
            if spd > self.max_cmd_speed:
                v_cmd_x = (v_cmd_x / spd) * self.max_cmd_speed
                v_cmd_y = (v_cmd_y / spd) * self.max_cmd_speed

            # Transformasi ke Body Frame
            cos_y = math.cos(self.drone_yaw)
            sin_y = math.sin(self.drone_yaw)
            v_body_x =  v_cmd_x * cos_y + v_cmd_y * sin_y
            v_body_y = -v_cmd_x * sin_y + v_cmd_y * cos_y
            self.publish_twist(v_body_x, v_body_y, wz_cmd)

            # Telemetri setiap 1 detik
            if self.step_count % 10 == 0:
                pct = self.get_coverage_percentage()
                self.get_logger().info(
                    f'  Baris {self.row_idx+1:2d}/{self.num_rows} | '
                    f'Prog: {actual_prog:4.1f}/{self.line_len:4.1f}m | '
                    f'Pos: ({self.drone_pos[0]:5.2f}, {self.drone_pos[1]:5.2f}) | '
                    f'Err: ({e_x:+5.3f}, {e_y:+5.3f}) | Yaw: {math.degrees(self.drone_yaw):+5.1f}° | Cov: {pct:4.1f}%'
                )

            # Cek ketercapaian ujung baris (Gated dengan toleransi posisi nyata)
            dist_to_end = abs(float(self.drone_pos[0]) - x_end)
            dist_to_end_line = self.line_len - actual_prog
            if (dist_to_end < 0.22 or dist_to_end_line < 0.15) and abs(e_y_current) < 0.20:
                pct = self.get_coverage_percentage()
                self.get_logger().info(
                    f'  -> Ujung Baris {self.row_idx + 1}/{self.num_rows} tercapai '
                    f'({self.drone_pos[0]:.2f}, {self.drone_pos[1]:.2f}). '
                    f'Coverage: {pct:.1f}%. Jeda sudut {self.corner_delay_ticks * 0.1:.1f}s'
                )
                self.publish_twist(0.0, 0.0, 0.0)
                self.state = 'delay_at_corner_end'
                self.delay_remaining = self.corner_delay_ticks
            return

        # ─────────────────────────────────────────────────────────────
        # 4. DELAY AT CORNER END — Jeda & rotasi in-place ke +90 deg (+Y)
        # ─────────────────────────────────────────────────────────────
        if self.state == 'delay_at_corner_end':
            target_yaw = math.pi / 2.0  # +90° menghadap arah langkah vertikal (+Y)
            wz_cmd = self.compute_wz(target_yaw)
            self.publish_twist(0.0, 0.0, wz_cmd)
            self.delay_remaining -= 1

            yaw_diff = math.atan2(math.sin(target_yaw - self.drone_yaw), math.cos(target_yaw - self.drone_yaw))
            if self.delay_remaining <= 0 and abs(yaw_diff) < math.radians(8.0):
                if self.row_idx >= self.num_rows - 1:
                    self.state = 'done'
                    pct = self.get_coverage_percentage()
                    self.get_logger().info(
                        f'========================================================================\n'
                        f'  🎉 PEMETAAN SELESAI! Seluruh {self.num_rows} Baris Tuntas Sempurna.\n'
                        f'  Total Luas Ter-Cover: {pct:.1f}%\n'
                        f'========================================================================'
                    )
                else:
                    self.state = 'stepping_vertical'
                    y_next = float(self.y_rows[self.row_idx + 1])
                    self.get_logger().info(
                        f'  -> Melangkah vertikal ke Y={y_next:.2f} [cmd_vel v={self.step_speed:.2f} m/s]'
                    )
            return

        # ─────────────────────────────────────────────────────────────
        # 5. STEPPING VERTICAL — Continuous smooth cmd_vel ke baris berikutnya
        # ─────────────────────────────────────────────────────────────
        if self.state == 'stepping_vertical':
            go_right = (self.row_idx % 2 == 0)
            x_corner = self.sweep_x_max if go_right else self.sweep_x_min
            y_start  = float(self.y_rows[self.row_idx])
            y_target = float(self.y_rows[self.row_idx + 1])
            step_len = abs(y_target - y_start)  # 1.20m

            target_yaw = math.pi / 2.0  # +90°
            wz_cmd = self.compute_wz(target_yaw)

            actual_prog_y = float(self.drone_pos[1]) - y_start
            actual_prog_y = max(0.0, min(step_len, actual_prog_y))

            # Lead point vertikal 0.35m di depan drone aktual
            s_target_y = min(step_len, actual_prog_y + self.lead_dist_y)
            ref_x = x_corner
            ref_y = y_start + s_target_y
            self.ref_pos = np.array([ref_x, ref_y], dtype=np.float32)

            e_x = ref_x - float(self.drone_pos[0])
            e_y = ref_y - float(self.drone_pos[1])

            # Deselerasi linear mendekati titik Y target
            dist_to_end_y = step_len - actual_prog_y
            if dist_to_end_y > 0.40:
                ff_scale_y = 1.0
            elif dist_to_end_y > 0.05:
                ff_scale_y = dist_to_end_y / 0.40
            else:
                ff_scale_y = 0.0

            v_ff_x = 0.0
            v_ff_y = self.step_speed * ff_scale_y

            # Koreksi posisi X dan Y
            v_cmd_x = v_ff_x + self.kp_track * e_x
            v_cmd_y = v_ff_y + self.kp_track * e_y

            spd_step = math.sqrt(v_cmd_x ** 2 + v_cmd_y ** 2)
            if spd_step > self.max_step_speed:
                v_cmd_x = (v_cmd_x / spd_step) * self.max_step_speed
                v_cmd_y = (v_cmd_y / spd_step) * self.max_step_speed

            # Transformasi ke Body Frame
            cos_y = math.cos(self.drone_yaw)
            sin_y = math.sin(self.drone_yaw)
            v_body_x =  v_cmd_x * cos_y + v_cmd_y * sin_y
            v_body_y = -v_cmd_x * sin_y + v_cmd_y * cos_y
            self.publish_twist(v_body_x, v_body_y, wz_cmd)

            # Cek ketercapaian titik awal baris baru
            dist_to_target_y = abs(float(self.drone_pos[1]) - y_target)
            if (dist_to_target_y < 0.15 or dist_to_end_y < 0.10) and abs(e_x) < 0.20:
                self.get_logger().info(
                    f'  -> Titik awal Baris {self.row_idx + 2} tercapai ({self.drone_pos[0]:.2f}, {self.drone_pos[1]:.2f})! '
                    f'Jeda settling {self.corner_delay_ticks * 0.1:.1f}s'
                )
                self.publish_twist(0.0, 0.0, 0.0)
                self.state = 'delay_at_new_row'
                self.delay_remaining = self.corner_delay_ticks
            return

        # ─────────────────────────────────────────────────────────────
        # 6. DELAY AT NEW ROW — Settling & rotasi in-place ke arah baris baru
        # ─────────────────────────────────────────────────────────────
        if self.state == 'delay_at_new_row':
            next_go_right = ((self.row_idx + 1) % 2 == 0)
            target_yaw = 0.0 if next_go_right else math.pi
            wz_cmd = self.compute_wz(target_yaw)
            self.publish_twist(0.0, 0.0, wz_cmd)
            self.delay_remaining -= 1

            yaw_diff = math.atan2(math.sin(target_yaw - self.drone_yaw), math.cos(target_yaw - self.drone_yaw))
            if self.delay_remaining <= 0 and abs(yaw_diff) < math.radians(8.0):
                self.row_idx += 1
                self.state = 'sweeping_row'
                go_right = (self.row_idx % 2 == 0)
                x_start = self.sweep_x_min if go_right else self.sweep_x_max
                x_end   = self.sweep_x_max if go_right else self.sweep_x_min
                y_curr = float(self.y_rows[self.row_idx])
                self.get_logger().info(
                    f'Memulai Baris {self.row_idx + 1}/{self.num_rows}: '
                    f'Sweep ({x_start:.2f} -> {x_end:.2f}, Y={y_curr:.2f}) [Continuous cmd_vel]'
                )
            return

        # 7. Selesai Misi
        if self.state == 'done':
            self.publish_twist(0.0, 0.0, 0.0)
            return

    # ── Visualisasi RViz2 ────────────────────────────────────────────

    def publish_rviz_markers(self):
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        # 1. Boundary Polygon (Garis Kuning Tebal)
        m_bdry = Marker()
        m_bdry.header.frame_id = 'world'
        m_bdry.header.stamp = now
        m_bdry.ns = 'boundary'
        m_bdry.id = 0
        m_bdry.type = Marker.LINE_STRIP
        m_bdry.action = Marker.ADD
        m_bdry.scale.x = 0.08
        m_bdry.color = ColorRGBA(r=1.0, g=0.84, b=0.0, a=0.95)
        for pt in np.vstack([self.boundary_poly, self.boundary_poly[0]]):
            p = Point()
            p.x, p.y, p.z = float(pt[0]), float(pt[1]), 0.05
            m_bdry.points.append(p)
        ma.markers.append(m_bdry)

        # 2. Planned Boustrophedon Path (Garis Cyan)
        m_path = Marker()
        m_path.header.frame_id = 'world'
        m_path.header.stamp = now
        m_path.ns = 'boustrophedon_plan'
        m_path.id = 1
        m_path.type = Marker.LINE_STRIP
        m_path.action = Marker.ADD
        m_path.scale.x = 0.05
        m_path.color = ColorRGBA(r=0.0, g=0.80, b=1.0, a=0.85)

        go_right = True
        for k in range(self.num_rows):
            y_curr = float(self.y_rows[k])
            if go_right:
                x_s, x_e = self.sweep_x_min, self.sweep_x_max
            else:
                x_s, x_e = self.sweep_x_max, self.sweep_x_min
            p_s = Point()
            p_s.x, p_s.y, p_s.z = float(x_s), y_curr, self.cruise_alt
            p_e = Point()
            p_e.x, p_e.y, p_e.z = float(x_e), y_curr, self.cruise_alt
            m_path.points.append(p_s)
            m_path.points.append(p_e)
            if k < self.num_rows - 1:
                y_next = float(self.y_rows[k + 1])
                p_step = Point()
                p_step.x, p_step.y, p_step.z = float(x_e), y_next, self.cruise_alt
                m_path.points.append(p_step)
            go_right = not go_right
        ma.markers.append(m_path)

        # 3. Active Reference Point (Bola Merah)
        m_ref = Marker()
        m_ref.header.frame_id = 'world'
        m_ref.header.stamp = now
        m_ref.ns = 'active_ref'
        m_ref.id = 2
        m_ref.type = Marker.SPHERE
        m_ref.action = Marker.ADD
        m_ref.pose.position.x = float(self.ref_pos[0])
        m_ref.pose.position.y = float(self.ref_pos[1])
        m_ref.pose.position.z = float(self.cruise_alt)
        m_ref.scale.x = 0.30
        m_ref.scale.y = 0.30
        m_ref.scale.z = 0.30
        m_ref.color = ColorRGBA(r=1.0, g=0.15, b=0.15, a=0.95)
        ma.markers.append(m_ref)

        # 4. Tracking Vector (Garis Kuning: Drone -> Reference)
        m_err = Marker()
        m_err.header.frame_id = 'world'
        m_err.header.stamp = now
        m_err.ns = 'tracking_vector'
        m_err.id = 3
        m_err.type = Marker.LINE_STRIP
        m_err.action = Marker.ADD
        m_err.scale.x = 0.03
        m_err.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.90)
        p_drone = Point()
        p_drone.x = float(self.drone_pos[0])
        p_drone.y = float(self.drone_pos[1])
        p_drone.z = float(self.drone_pos[2])
        p_target = Point()
        p_target.x = float(self.ref_pos[0])
        p_target.y = float(self.ref_pos[1])
        p_target.z = self.cruise_alt
        m_err.points.append(p_drone)
        m_err.points.append(p_target)
        ma.markers.append(m_err)

        # 5. Coverage Footprint (Hijau di Lantai)
        m_cov = Marker()
        m_cov.header.frame_id = 'world'
        m_cov.header.stamp = now
        m_cov.ns = 'coverage_footprint'
        m_cov.id = 4
        m_cov.type = Marker.POINTS
        m_cov.action = Marker.ADD
        m_cov.scale.x = float(self.dx * 1.05)
        m_cov.scale.y = float(self.dy * 1.05)
        m_cov.color = ColorRGBA(r=0.0, g=0.80, b=0.42, a=0.38)
        covered_indices = np.argwhere(self.cov_grid)
        for r, c in covered_indices:
            p = Point()
            p.x = float(self.x_min + (c + 0.5) * self.dx)
            p.y = float(self.y_min + (r + 0.5) * self.dy)
            p.z = 0.02
            m_cov.points.append(p)
        ma.markers.append(m_cov)

        # 6. HUD Text Overlay
        m_text = Marker()
        m_text.header.frame_id = 'world'
        m_text.header.stamp = now
        m_text.ns = 'hud_info'
        m_text.id = 5
        m_text.type = Marker.TEXT_VIEW_FACING
        m_text.action = Marker.ADD
        m_text.pose.position.x = float(self.drone_pos[0])
        m_text.pose.position.y = float(self.drone_pos[1])
        m_text.pose.position.z = float(self.drone_pos[2] + 0.50)
        m_text.scale.z = 0.28
        m_text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.95)

        pct = self.get_coverage_percentage()
        m_text.text = (
            f"Cov: {pct:.1f}% | Baris {self.row_idx + 1}/{self.num_rows}\n"
            f"[{self.state}] (Continuous cmd_vel)"
        )
        ma.markers.append(m_text)

        self.pub_markers.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = PureContinuousMappingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Pemetaan dihentikan manual oleh pengguna (Ctrl+C).')
    except Exception as e:
        node.get_logger().info(f'Sesi pemetaan selesai: {e}')
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
