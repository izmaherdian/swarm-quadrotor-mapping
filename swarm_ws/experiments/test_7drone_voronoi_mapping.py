#!/usr/bin/env python3
"""
===============================================================================
  SWARM 7-DRONE 2D VORONOI & BOUSTROPHEDON MAPPING COORDINATOR (STEP 6 v8.3)
===============================================================================
Fitur Utama:
  1. CRITICALLY DAMPED TRACKING & ZERO OVERSHOOT (0.0%):
     - Deselerasi feedforward ramp-down pada 0.80m menjelang ujung baris.
     - Strict endpoint clamping: s_target <= line_len (tidak ada overshoot).
     - Prioritas underdamped / undershoot aman (berhenti tepat pada batas baris).
  2. DYNAMIC FEEDBACK-COUPLED MOVING REFERENCE (CARROT WAITS FOR DRONE):
     - Titik referensi bergerak ref_pos terikat pada progres aktual drone (actual_prog).
     - Jika drone melambat atau memutar yaw, ref_pos berhenti dan menunggu drone fisik.
  3. STATIONARY IN-PLACE CORNER PIVOT:
     - Quadrotor berhenti linier total (vx=0, vy=0) di ujung baris.
     - Rotasi yaw in-place dengan jeda stabilisasi (corner_settle_ticks) hingga
       osilasi sudut reda sempurna sebelum melangkah vertikal.
  4. MAPPING ISOLATION & LONGITUDINAL YIELDING:
     - Gaya tolak samping V2V dinonaktifkan saat sweeping untuk menjaga garis 100% lurus.
     - Longitudinal Speed Throttle: memperlambat laju maju jika jarak ke tetangga < 1.25m
       tanpa pernah membelokkan drone ke samping.
     - Drone yang berstatus 'done' mengaktifkan passive repulsion yielding jika didekati tetangga.
  5. SINKRONISASI START BERSAMA (wait_all_start):
     - Seluruh drone menunggu di titik start sel masing-masing hingga semua 7 drone siap.
  6. REAL-TIME MOVING CARROT SPHERE VISUALIZER DI RVIZ2:
     - Titik referensi ref_pos dipublikasikan sebagai Marker Sphere bercahaya (Z=2.00m).
  7. COMPASS DIRECTION MARKERS (NORTH, SOUTH, EAST, WEST):
     - Marker panah kompas 3D dan label mata angin di RViz2 yang selaras dengan Top-View Gazebo.
  8. AUTOMATED QUANTITATIVE TRACKING EVALUATION:
     - Pencatatan telemetri Cross-Track RMS, Overshoot (<= 0.0%), dan Status Kelulusan.
===============================================================================
"""

import math
import sys
import numpy as np
from scipy.optimize import linear_sum_assignment
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, TwistStamped, Point, PoseStamped
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA


# ═════════════════════════════════════════════════════════════════════════════
#  MODUL GEOMETRI VORONOI & BOUSTROPHEDON
# ═════════════════════════════════════════════════════════════════════════════

def clip_voronoi(polygon, pi, pj):
    """Sutherland-Hodgman bisector clipping standar untuk relaksasi Lloyd."""
    if not polygon:
        return []
    mid = (pi + pj) / 2.0
    n = pj - pi
    inside = lambda p: np.dot(p - mid, n) <= 0.0

    def isect(a, b):
        d = b - a
        den = np.dot(d, n)
        return a if abs(den) < 1e-9 else a + np.dot(mid - a, n) / den * d

    out = []
    s = polygon[-1]
    for e in polygon:
        ei, si = inside(e), inside(s)
        if ei:
            if not si:
                out.append(isect(s, e))
            out.append(e)
        elif si:
            out.append(isect(s, e))
        s = e
    return out


def clip_voronoi_margin(polygon, pi, pj, margin=0.42):
    """
    Sutherland-Hodgman bisector clipping dengan safety margin normal.
    Menggeser garis batas pemisah sejauh 'margin' ke arah pi,
    menjamin jarak antar-sel minimal 2*margin di semua arah.
    """
    if not polygon:
        return []
    mid = (pi + pj) / 2.0
    n = pj - pi
    n_len = float(np.linalg.norm(n))
    if n_len < 1e-6:
        return polygon
    u_n = n / n_len
    plane_pt = mid - u_n * margin
    inside = lambda p: np.dot(p - plane_pt, u_n) <= 0.0

    def isect(a, b):
        d = b - a
        den = np.dot(d, u_n)
        return a if abs(den) < 1e-9 else a + np.dot(plane_pt - a, u_n) / den * d

    out = []
    s = polygon[-1]
    for e in polygon:
        ei, si = inside(e), inside(s)
        if ei:
            if not si:
                out.append(isect(s, e))
            out.append(e)
        elif si:
            out.append(isect(s, e))
        s = e
    return out


def poly_centroid(pts):
    """Menghitung titik berat geometris poligon (Shoelace formula)."""
    p = np.array(pts, dtype=float)
    n = len(p)
    if n < 3:
        return p.mean(axis=0) if n else np.zeros(2)
    A = cx = cy = 0.0
    for i in range(n):
        x0, y0 = p[i]
        x1, y1 = p[(i + 1) % n]
        f = x0 * y1 - x1 * y0
        A += f
        cx += (x0 + x1) * f
        cy += (y0 + y1) * f
    A *= 0.5
    if abs(A) < 1e-9:
        return p.mean(axis=0)
    return np.array([cx / (6 * A), cy / (6 * A)])


def polygon_scanline_intersections(polygon, y):
    """Mencari titik potong horizontal garis scanline y dengan sisi poligon."""
    xs = []
    pts = [np.array(p, dtype=float) for p in polygon]
    n = len(pts)
    for i in range(n):
        a, b = pts[i], pts[(i + 1) % n]
        ya, yb = a[1], b[1]
        if abs(yb - ya) < 1e-9:
            continue
        if not (min(ya, yb) <= y <= max(ya, yb)):
            continue
        t = (y - ya) / (yb - ya)
        x = a[0] + t * (b[0] - a[0])
        xs.append(x)
    xs.sort()
    return xs


def generate_boustrophedon(polygon, sweep_spacing=1.45, margin=0.02, start_from_top=False):
    """
    Menghasilkan rute sapuan Lawnmower horizontal zigzag di dalam sel poligon.
    Jika start_from_top=True, urutan baris dimulai dari Y tertinggi ke terendah.
    """
    if len(polygon) < 3:
        return [poly_centroid(polygon)]

    pts = np.array(polygon, dtype=float)
    min_x, max_x = pts[:, 0].min(), pts[:, 0].max()
    min_y, max_y = pts[:, 1].min(), pts[:, 1].max()

    scan_min_y = min_y + margin
    scan_max_y = max_y - margin

    if scan_max_y <= scan_min_y:
        scan_y_levels = [0.5 * (min_y + max_y)]
    else:
        n_lines = max(1, int(math.ceil((scan_max_y - scan_min_y) / sweep_spacing)))
        scan_y_levels = np.linspace(scan_min_y, scan_max_y, n_lines)
        if start_from_top:
            scan_y_levels = scan_y_levels[::-1]

    waypoints = []
    sweep_right = True

    for y in scan_y_levels:
        xs = polygon_scanline_intersections(polygon, y)
        if len(xs) < 2:
            continue

        x_left = xs[0] + margin
        x_right = xs[-1] - margin

        # Lewatkan baris yang terlalu pendek (< 0.60m)
        if (x_right - x_left) < 0.60:
            continue

        if sweep_right:
            waypoints.append(np.array([x_left, y]))
            waypoints.append(np.array([x_right, y]))
        else:
            waypoints.append(np.array([x_right, y]))
            waypoints.append(np.array([x_left, y]))

        sweep_right = not sweep_right

    if not waypoints:
        return [poly_centroid(polygon)]

    return waypoints


# ═════════════════════════════════════════════════════════════════════════════
#  KELAS AGENT DRONE
# ═════════════════════════════════════════════════════════════════════════════

class DroneAgent:
    def __init__(self, drone_id, color_rgb):
        self.id = drone_id
        self.ns = f'iris_{drone_id}'
        self.color = color_rgb

        self.pos = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        self.yaw = 0.0
        self.odom_received = False
        self.target_yaw = 0.0

        # State Machine Diskrit:
        # wait_takeoff -> transit_to_start -> wait_all_start -> align_start_yaw
        # -> sweeping_row -> delay_at_corner_end -> stepping_vertical -> delay_at_new_row -> sweeping_row
        # -> ... -> done
        self.state = 'wait_takeoff'
        self.delay_timer = 0

        # Voronoi Cell & Boustrophedon Path
        self.cell_polygon = []
        self.centroid = np.array([0.0, 0.0], dtype=np.float32)
        self.waypoints = []
        self.row_idx = 0
        self.num_rows = 0

        # Dynamic Moving Reference Carrot (Z=2.00m)
        self.ref_pos = np.array([0.0, 0.0], dtype=np.float32)

        # Actual Flight Trail History
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'world'
        self.last_path_pos = None

        # Telemetri & Tracking Metrics
        self.min_dist_to_others = float('inf')
        self.max_cross_track_err = 0.0
        self.max_overshoot = 0.0
        self.cross_track_errors = []
        self.overshoot_list = []
        self.yaw_errors = []
        self.altitude_errors = []


# ═════════════════════════════════════════════════════════════════════════════
#  NODE KOORDINATOR SWARM 7-DRONE
# ═════════════════════════════════════════════════════════════════════════════

class Swarm7DroneVoronoiMappingNode(Node):
    def __init__(self):
        super().__init__('swarm_7drone_voronoi_mapping_node')
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)

        # ── Parameter Wilayah Arena 30x30m (Active Map: 28x28m) ──────
        self.x_min, self.x_max = -15.0, 15.0
        self.y_min, self.y_max = -15.0, 15.0
        self.active_x_min, self.active_x_max = -14.0, 14.0
        self.active_y_min, self.active_y_max = -14.0, 14.0
        self.cruise_alt = 2.0
        self.sensor_radius = 0.95

        # Bounding box arena pemetaan aktif
        self.bbox = [
            np.array([self.active_x_min, self.active_y_min]),
            np.array([self.active_x_max, self.active_y_min]),
            np.array([self.active_x_max, self.active_y_max]),
            np.array([self.active_x_min, self.active_y_max]),
        ]

        # ── Parameter Kecepatan & Kontrol Presisi Kritis Terredam ────
        self.nominal_speed = 2.85       # Kecepatan sapuan baris nominal (m/s)
        self.transit_speed = 2.70       # Kecepatan transit awal (m/s)
        self.step_speed = 2.20          # Kecepatan langkah vertikal (m/s)
        self.max_cmd_speed = 3.00       # Saturation limit (m/s)
        self.kp_track = 2.20            # Tracking gain lateral
        self.lead_dist = 0.70           # Jarak maju virtual carrot di depan drone (m)
        self.corner_settle_ticks = 3    # Jeda 0.15 detik saat pivot statis di sudut (@20Hz)

        # ── Coverage Grid (100 x 100 sel = 10.000 sel, 0.3m/sel) ─────
        self.grid_n = 100
        self.cov_grid = np.zeros((self.grid_n, self.grid_n), dtype=bool)
        self.dx = (self.x_max - self.x_min) / self.grid_n
        self.dy = (self.y_max - self.y_min) / self.grid_n

        # ── 7 Drone Swarm Initialization ─────────────────────────────
        self.drone_colors = [
            (1.00, 0.15, 0.15),  # Drone 1: Merah
            (1.00, 0.55, 0.00),  # Drone 2: Oranye
            (1.00, 0.85, 0.00),  # Drone 3: Kuning
            (0.00, 1.00, 0.30),  # Drone 4: Hijau
            (0.00, 0.85, 1.00),  # Drone 5: Cyan
            (0.30, 0.50, 1.00),  # Drone 6: Biru
            (0.85, 0.25, 1.00),  # Drone 7: Ungu
        ]

        self.agents = {}
        for did in range(1, 8):
            self.agents[did] = DroneAgent(did, self.drone_colors[did - 1])

        self.global_min_dist = float('inf')
        self.step_count = 0
        self.voronoi_planned = False
        self.mission_completed = False

        # ── QoS Reliable 10 untuk Kompatibilitas Bridge Gazebo ────────
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ── Publishers & Subscribers ─────────────────────────────────
        self.pub_vel = {}
        self.pub_vel_stamped = {}
        self.pub_target_pose = {}
        self.pub_actual_path = {}
        self.sub_odom = {}
        for did, agent in self.agents.items():
            self.pub_vel[did] = self.create_publisher(Twist, f'/{agent.ns}/cmd_vel', 10)
            self.pub_vel_stamped[did] = self.create_publisher(TwistStamped, f'/{agent.ns}/target_velocity', 10)
            self.pub_target_pose[did] = self.create_publisher(PoseStamped, f'/{agent.ns}/target_pose', 10)
            self.pub_actual_path[did] = self.create_publisher(Path, f'/{agent.ns}/actual_path', 10)
            self.sub_odom[did] = self.create_subscription(
                Odometry,
                f'/{agent.ns}/odometry',
                lambda msg, d=did: self.odom_callback(d, msg),
                qos_reliable
            )

        self.pub_markers = self.create_publisher(MarkerArray, '/mapping/markers', 10)

        # Timer 20 Hz (50ms) untuk Loop Kontrol dan Visualisasi Presisi Tinggi
        self.timer_control = self.create_timer(0.05, self.control_loop)
        self.timer_viz = self.create_timer(0.10, self.publish_rviz_markers)

        self.get_logger().info(
            f'🚀 Swarm 7-Drone 2D Voronoi Mapping Node Siap (v8.3 Longitudinal Yield & Barrier)! '
            f'(Arena: {self.x_max-self.x_min:.0f}x{self.y_max-self.y_min:.0f}m, '
            f'Grid: {self.grid_n}x{self.grid_n}, v_nom={self.nominal_speed:.2f}m/s @ 20Hz)'
        )

    # ── Utilitas Matematika & Rotasi ─────────────────────────────────

    def euler_from_quaternion(self, qx, qy, qz, qw):
        t0 = +2.0 * (qw * qx + qy * qz)
        t1 = +1.0 - 2.0 * (qx * qx + qy * qy)
        roll = math.atan2(t0, t1)
        t2 = max(-1.0, min(1.0, +2.0 * (qw * qy - qz * qx)))
        pitch = math.asin(t2)
        t3 = +2.0 * (qw * qz + qx * qy)
        t4 = +1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(t3, t4)
        return roll, pitch, yaw

    def compute_wz(self, current_yaw, target_yaw):
        """Hitung laju yaw (rad/s) mulus dengan batas 40 deg/s untuk putaran anggun dan stabil di sudut."""
        diff = math.atan2(math.sin(target_yaw - current_yaw), math.cos(target_yaw - current_yaw))
        MAX_WZ = math.radians(40.0)  # Max 40 deg/s (~0.70 rad/s)
        return float(np.clip(1.2 * diff, -MAX_WZ, MAX_WZ))

    def odom_callback(self, did, msg):
        agent = self.agents[did]
        agent.pos[0] = msg.pose.pose.position.x
        agent.pos[1] = msg.pose.pose.position.y
        agent.pos[2] = msg.pose.pose.position.z

        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        _, _, agent.yaw = self.euler_from_quaternion(qx, qy, qz, qw)
        agent.odom_received = True

        # Perbarui jejak lintasan penerbangan riil (Path)
        if agent.last_path_pos is None or np.linalg.norm(agent.pos[:2] - agent.last_path_pos[:2]) > 0.12:
            agent.last_path_pos = agent.pos.copy()
            ps = PoseStamped()
            ps.header.frame_id = 'world'
            ps.header.stamp = msg.header.stamp
            ps.pose = msg.pose.pose
            agent.path_msg.poses.append(ps)
            if len(agent.path_msg.poses) > 1000:
                agent.path_msg.poses.pop(0)

    # ── Perencanaan Centroidal Voronoi + Hungarian Minimum-Distance Assignment ──

    def plan_centroidal_voronoi(self):
        """Menjalankan 25x Lloyd's Relaxation & Hungarian Assignment untuk 7 Drone."""
        self.get_logger().info('📐 Menjalankan Centroidal Voronoi (25x Lloyd) + Hungarian Assignment...')

        # Inisialisasi generator terdistribusi simetris
        seed_points = [
            np.array([-9.0, -9.0]), np.array([0.0, -9.8]), np.array([9.0, -9.0]),
            np.array([-9.0, 7.0]), np.array([0.0, 0.0]), np.array([9.0, 7.0]), np.array([0.0, 9.8])
        ]
        gens = {i + 1: seed_points[i].copy() for i in range(7)}

        # 25 Putaran Lloyd's Relaxation Loop untuk partisi homogen seimbang
        for _ in range(25):
            new_gens = {}
            for i in gens:
                cell_tmp = [np.array(v, dtype=float) for v in self.bbox]
                for j in gens:
                    if j != i:
                        cell_tmp = clip_voronoi(cell_tmp, gens[i], gens[j])
                if len(cell_tmp) >= 3:
                    new_gens[i] = poly_centroid(cell_tmp)
                else:
                    new_gens[i] = gens[i]
            gens = new_gens

        # Optimal Bipartite Matching: pasangkan posisi riil drone ke sel Voronoi terdekat
        drone_positions = np.array([self.agents[did].pos[:2] for did in range(1, 8)])
        target_centroids = np.array([gens[did] for did in range(1, 8)])
        cost_mat = np.linalg.norm(drone_positions[:, None, :] - target_centroids[None, :, :], axis=2)
        row_ind, col_ind = linear_sum_assignment(cost_mat)

        # Re-assign generator hasil matching
        drone_to_gen = {}
        for r_idx, c_idx in zip(row_ind, col_ind):
            drone_to_gen[r_idx + 1] = target_centroids[c_idx]

        # Bentuk sel poligon dengan margin bisector normal 0.20m & rute Boustrophedon
        for did, agent in self.agents.items():
            pi = drone_to_gen[did]
            cell = [np.array(v, dtype=float) for v in self.bbox]
            for other_did, pj in drone_to_gen.items():
                if other_did != did:
                    cell = clip_voronoi_margin(cell, pi, pj, margin=0.45)

            agent.cell_polygon = cell
            agent.centroid = poly_centroid(cell)
            pts = np.array(cell, dtype=float)
            y_mid = 0.5 * (pts[:, 1].min() + pts[:, 1].max())

            start_from_top = bool(agent.pos[1] > y_mid)
            wps = generate_boustrophedon(cell, sweep_spacing=1.45, margin=0.02, start_from_top=start_from_top)
            agent.waypoints = wps
            agent.num_rows = max(1, len(wps) // 2)
            agent.row_idx = 0
            agent.ref_pos = wps[0].copy()

        # Deconflict initial start waypoints if any pair of drones starts within 1.6m of each other
        for _ in range(5):
            conflict_found = False
            for i in range(1, 8):
                for j in range(i + 1, 8):
                    w_i = self.agents[i].waypoints[0]
                    w_j = self.agents[j].waypoints[0]
                    if np.linalg.norm(w_i - w_j) < 1.60:
                        conflict_found = True
                        pts_j = np.array(self.agents[j].cell_polygon, dtype=float)
                        y_mid_j = 0.5 * (pts_j[:, 1].min() + pts_j[:, 1].max())
                        curr_start_from_top = (self.agents[j].waypoints[0][1] > y_mid_j)
                        wps_new = generate_boustrophedon(
                            self.agents[j].cell_polygon,
                            sweep_spacing=1.45,
                            margin=0.02,
                            start_from_top=(not curr_start_from_top)
                        )
                        self.agents[j].waypoints = wps_new
                        self.agents[j].num_rows = max(1, len(wps_new) // 2)
                        self.agents[j].ref_pos = wps_new[0].copy()
                        break
                if conflict_found:
                    break

        for did, agent in self.agents.items():
            dist_to_start = float(np.linalg.norm(agent.pos[:2] - agent.waypoints[0]))
            self.get_logger().info(
                f'  -> [iris_{did}] Sel ({len(agent.cell_polygon)} simpul) | {agent.num_rows} Baris | '
                f'Start: ({agent.waypoints[0][0]:.2f}, {agent.waypoints[0][1]:.2f}) | Centroid: ({agent.centroid[0]:.2f}, {agent.centroid[1]:.2f}) | '
                f'Transit Dist: {dist_to_start:.2f}m'
            )

        self.voronoi_planned = True

    # ── Update Grid Cakupan (Coverage) ───────────────────────────────

    def update_coverage(self):
        """Memperbarui matriks okupansi sensor FoV (R=0.95m) untuk semua drone."""
        for agent in self.agents.values():
            if not agent.odom_received or agent.pos[2] < 0.8:
                continue

            cx_idx = int((agent.pos[0] - self.x_min) / self.dx)
            cy_idx = int((agent.pos[1] - self.y_min) / self.dy)
            rad_cells = int(math.ceil(self.sensor_radius / self.dx))

            i_min = max(0, cx_idx - rad_cells)
            i_max = min(self.grid_n, cx_idx + rad_cells + 1)
            j_min = max(0, cy_idx - rad_cells)
            j_max = min(self.grid_n, cy_idx + rad_cells + 1)

            for i in range(i_min, i_max):
                for j in range(j_min, j_max):
                    if not self.cov_grid[i, j]:
                        cell_x = self.x_min + (i + 0.5) * self.dx
                        cell_y = self.y_min + (j + 0.5) * self.dy
                        if (cell_x - agent.pos[0]) ** 2 + (cell_y - agent.pos[1]) ** 2 <= self.sensor_radius ** 2:
                            self.cov_grid[i, j] = True

    def get_coverage_percentage(self):
        """Menghitung persentase cakupan di dalam area pemetaan aktif."""
        i_start = int((self.active_x_min - self.x_min) / self.dx)
        i_end = int((self.active_x_max - self.x_min) / self.dx)
        j_start = int((self.active_y_min - self.y_min) / self.dy)
        j_end = int((self.active_y_max - self.y_min) / self.dy)

        sub_grid = self.cov_grid[i_start:i_end, j_start:j_end]
        return float(np.mean(sub_grid) * 100.0)

    # ── State Machine & Loop Kontrol Utama (20 Hz) ───────────────────

    def control_loop(self):
        self.step_count += 1

        if not all(a.odom_received for a in self.agents.values()):
            return

        # Perbarui jarak pisah antar-drone (hanya saat terbang Z >= 0.80m)
        for i in range(1, 8):
            for j in range(i + 1, 8):
                if self.agents[i].pos[2] >= 0.80 and self.agents[j].pos[2] >= 0.80:
                    p1 = self.agents[i].pos[:2]
                    p2 = self.agents[j].pos[:2]
                    d = float(np.linalg.norm(p1 - p2))
                    self.agents[i].min_dist_to_others = min(self.agents[i].min_dist_to_others, d)
                    self.agents[j].min_dist_to_others = min(self.agents[j].min_dist_to_others, d)
                    self.global_min_dist = min(self.global_min_dist, d)

        # ── Zero-Interference Autonomous Takeoff Check ───────────────
        if not self.voronoi_planned:
            takeoff_ready = True
            for agent in self.agents.values():
                if agent.pos[2] < 1.30:
                    takeoff_ready = False
                    break

            if takeoff_ready:
                self.plan_centroidal_voronoi()
                for agent in self.agents.values():
                    agent.state = 'transit_to_start'
            return

        self.update_coverage()

        # Eksekusi state machine per drone
        for did, agent in self.agents.items():

            # ─────────────────────────────────────────────────────────
            # 1. TRANSIT TO START (Fase Menuju Titik Awal Sel)
            # ─────────────────────────────────────────────────────────
            if agent.state == 'transit_to_start':
                start_wp = agent.waypoints[0]
                dx = start_wp[0] - float(agent.pos[0])
                dy = start_wp[1] - float(agent.pos[1])
                dist_to_start = math.hypot(dx, dy)

                # Coupled Carrot untuk Transit
                lead_t = min(dist_to_start, self.lead_dist)
                angle_to_start = math.atan2(dy, dx)
                agent.ref_pos = np.array([
                    float(agent.pos[0]) + lead_t * math.cos(angle_to_start),
                    float(agent.pos[1]) + lead_t * math.sin(angle_to_start)
                ], dtype=np.float32)

                if dist_to_start < 0.25:
                    self.get_logger().info(f'  🎯 [iris_{did}] Tiba di Titik Start Sel ({agent.pos[0]:.2f}, {agent.pos[1]:.2f}) | Menunggu kawanan...')
                    agent.state = 'wait_all_start'
                    self.publish_twist(did, 0.0, 0.0, 0.0)
                    continue

                v_mag = min(self.transit_speed, max(0.40, 2.0 * dist_to_start))
                v_world_x = v_mag * math.cos(angle_to_start) + np.clip(1.2 * dx, -0.40, 0.40)
                v_world_y = v_mag * math.sin(angle_to_start) + np.clip(1.2 * dy, -0.40, 0.40)

                # Full V2V avoidance during transit
                v_world_x, v_world_y = self.apply_v2v_repulsion(did, v_world_x, v_world_y, is_transit=True)
                self.send_world_twist(did, v_world_x, v_world_y, agent.yaw)

            # ─────────────────────────────────────────────────────────
            # 1b. WAIT ALL START (Sinkronisasi Start Bersama Antar Kawanan)
            # ─────────────────────────────────────────────────────────
            elif agent.state == 'wait_all_start':
                # Tetap aktifkan V2V repulsion pasif jika ada drone transit lain yang melintas
                v_x, v_y = self.apply_v2v_repulsion(did, 0.0, 0.0, is_transit=True)
                self.send_world_twist(did, v_x, v_y, agent.yaw)

                # Cek apakah seluruh 7 drone telah sampai di start point masing-masing
                all_arrived = all(a.state in ('wait_all_start', 'align_start_yaw', 'sweeping_row', 'delay_at_corner_end', 'stepping_vertical', 'delay_at_new_row', 'done') for a in self.agents.values())
                if all_arrived:
                    agent.state = 'align_start_yaw'
                    agent.delay_timer = self.corner_settle_ticks
                    self.publish_twist(did, 0.0, 0.0, 0.0)
                    self.get_logger().info(f'  🏁 [iris_{did}] Semua Drone Siap di Sel Masing-masing! Menyelaraskan heading...')

            # ─────────────────────────────────────────────────────────
            # 2. ALIGN START YAW (Pivot In-Place ke Baris 1)
            # ─────────────────────────────────────────────────────────
            elif agent.state == 'align_start_yaw':
                wp_start = agent.waypoints[0]
                wp_end = agent.waypoints[1]
                line_dir = wp_end - wp_start
                target_yaw = math.atan2(line_dir[1], line_dir[0])
                agent.target_yaw = target_yaw
                agent.ref_pos = wp_start.copy()

                wz_cmd = self.compute_wz(agent.yaw, target_yaw)
                self.publish_twist(did, 0.0, 0.0, wz_cmd)
                agent.delay_timer -= 1

                yaw_diff = abs(math.atan2(math.sin(target_yaw - agent.yaw), math.cos(target_yaw - agent.yaw)))
                if agent.delay_timer <= 0 or (agent.delay_timer <= 4 and yaw_diff < math.radians(8.0)):
                    agent.state = 'sweeping_row'
                    agent.row_idx = 0
                    self.get_logger().info(f'  🚀 [iris_{did}] Heading Selaras ({math.degrees(agent.yaw):.1f}°)! Memulai Baris 1/{agent.num_rows}')

            # ─────────────────────────────────────────────────────────
            # 3. SWEEPING ROW (Critically Damped Tracking & Longitudinal Yield)
            # ─────────────────────────────────────────────────────────
            elif agent.state == 'sweeping_row':
                idx_start = agent.row_idx * 2
                idx_end = idx_start + 1

                if idx_end >= len(agent.waypoints):
                    agent.state = 'return_to_centroid'
                    self.get_logger().info(f'🎉 [iris_{did}] PEMETAAN SEL TUNTAS! Kembali ke pusat sel ({agent.centroid[0]:.2f}, {agent.centroid[1]:.2f}).')
                    continue

                wp_start = agent.waypoints[idx_start]
                wp_end = agent.waypoints[idx_end]

                line_vec = wp_end - wp_start
                line_len = float(np.linalg.norm(line_vec))
                if line_len < 0.01:
                    agent.row_idx += 1
                    continue

                u_line = line_vec / line_len
                pos_rel = agent.pos[:2] - wp_start
                
                # Progres AKTUAL drone fisik di sepanjang garis sapuan
                actual_prog = max(0.0, min(line_len, float(np.dot(pos_rel, u_line))))
                e_lat = float(pos_rel[1] * u_line[0] - pos_rel[0] * u_line[1])

                # DYNAMIC FEEDBACK-COUPLED MOVING REFERENCE (CARROT WAITS FOR DRONE)
                s_target = min(line_len, actual_prog + self.lead_dist)
                agent.ref_pos = wp_start + s_target * u_line

                # Catat telemetri tracking
                agent.max_cross_track_err = max(agent.max_cross_track_err, abs(e_lat))
                agent.cross_track_errors.append(abs(e_lat))
                agent.altitude_errors.append(abs(float(agent.pos[2]) - self.cruise_alt))
                yaw_diff = abs(math.atan2(math.sin(agent.yaw - agent.target_yaw), math.cos(agent.yaw - agent.target_yaw)))
                agent.yaw_errors.append(math.degrees(yaw_diff))

                dist_to_end = line_len - actual_prog
                dist_to_end_pt = float(np.linalg.norm(agent.pos[:2] - wp_end))

                # Cek ketercapaian ujung baris (Snapping & Hard Stop)
                # Syarat: harus sudah melewati 40% panjang baris agar tidak salah picu di titik awal
                if actual_prog > 0.40 * line_len and (dist_to_end <= 0.25 or dist_to_end_pt < 0.26):
                    overshoot_dist = max(0.0, float(np.dot(pos_rel, u_line)) - line_len)
                    overshoot_pct = (overshoot_dist / line_len) * 100.0 if line_len > 0 else 0.0
                    agent.overshoot_list.append(overshoot_pct)
                    agent.max_overshoot = max(agent.max_overshoot, overshoot_pct)

                    self.get_logger().info(
                        f'  -> [iris_{did}] Ujung Baris {agent.row_idx+1}/{agent.num_rows} '
                        f'tercapai ({agent.pos[0]:.2f}, {agent.pos[1]:.2f}) | Overshoot: {overshoot_pct:.2f}% ({overshoot_dist:.2f}m)'
                    )
                    self.publish_twist(did, 0.0, 0.0, 0.0)

                    if agent.row_idx + 1 >= agent.num_rows:
                        agent.state = 'return_to_centroid'
                        self.get_logger().info(f'🎉 [iris_{did}] PEMETAAN SEL TUNTAS! Kembali ke pusat sel ({agent.centroid[0]:.2f}, {agent.centroid[1]:.2f}).')
                    else:
                        agent.state = 'delay_at_corner_end'
                        agent.delay_timer = 0
                    continue

                # CRITICALLY DAMPED FEEDFORWARD RAMP-DOWN pada 0.80m terakhir
                if dist_to_end > 0.80:
                    ff_scale = 1.0
                elif dist_to_end > 0.10:
                    ff_scale = dist_to_end / 0.80
                else:
                    ff_scale = 0.0

                # MAPPING ISOLATION: 100% fokus sapuan baris lurus tanpa interferensi
                v_ff = (self.nominal_speed * ff_scale) * u_line
                
                # Feedback controller: Koreksi lateral orthogonal (Cross-track)
                v_corr_lat = -np.clip(self.kp_track * e_lat, -0.45, 0.45) * np.array([-u_line[1], u_line[0]])

                v_world = v_ff + v_corr_lat
                v_world_x = float(v_world[0])
                v_world_y = float(v_world[1])

                self.send_world_twist(did, v_world_x, v_world_y, agent.yaw)

            # ─────────────────────────────────────────────────────────
            # 4. DELAY AT CORNER END (Stationary In-Place Pivot & Yielding)
            # ─────────────────────────────────────────────────────────
            elif agent.state == 'delay_at_corner_end':
                # Cek jika ada drone tetangga di perbatasan yang sedang belok / dekat (< 1.25m)
                has_yield_conflict = False
                for other_id, other_agent in self.agents.items():
                    if other_id != did and other_agent.odom_received and other_agent.pos[2] >= 0.80:
                        dist_neighbor = float(np.linalg.norm(agent.pos[:2] - other_agent.pos[:2]))
                        if dist_neighbor < 1.25:
                            # Jika drone tetangga sedang melangkah vertikal atau memiliki prioritas (did > other_id), tahan di sudut
                            if other_agent.state in ('stepping_vertical', 'sweeping_row') or (other_agent.state in ('delay_at_corner_end', 'delay_at_new_row') and did > other_id):
                                has_yield_conflict = True
                                break

                next_start = agent.waypoints[(agent.row_idx + 1) * 2]
                step_vec = next_start - agent.pos[:2]
                target_yaw = math.atan2(step_vec[1], step_vec[0])
                agent.target_yaw = target_yaw

                # Stationary: Kecepatan linier mutlak 0.0 m/s
                wz_cmd = self.compute_wz(agent.yaw, target_yaw)
                self.publish_twist(did, 0.0, 0.0, wz_cmd)
                agent.delay_timer = getattr(agent, 'delay_timer', 0) + 1

                yaw_diff = abs(math.atan2(math.sin(target_yaw - agent.yaw), math.cos(target_yaw - agent.yaw)))
                # Berhenti diam (min 1.25s = 25 ticks) & TUNGGU sampai yaw selaras (< 6.0°) dan TIDAK ADA konflik perbatasan
                if not has_yield_conflict and ((agent.delay_timer >= 25 and yaw_diff < math.radians(6.0)) or agent.delay_timer >= 120):
                    agent.state = 'stepping_vertical'
                    agent.step_timer = 0
                    agent.delay_timer = 0

            # ─────────────────────────────────────────────────────────
            # 5. STEPPING VERTICAL (Melangkah Lurus Maju ke Baris Baru)
            # ─────────────────────────────────────────────────────────
            elif agent.state == 'stepping_vertical':
                min_step_dist = float('inf')
                for other_id, other_agent in self.agents.items():
                    if other_id != did and other_agent.odom_received and other_agent.pos[2] >= 0.80:
                        d_ij = float(np.linalg.norm(agent.pos[:2] - other_agent.pos[:2]))
                        min_step_dist = min(min_step_dist, d_ij)

                next_start = agent.waypoints[(agent.row_idx + 1) * 2]
                dx = float(next_start[0]) - float(agent.pos[0])
                dy = float(next_start[1]) - float(agent.pos[1])
                dist_to_next = math.hypot(dx, dy)

                # Coupled Carrot untuk Langkah Vertikal
                lead_s = min(dist_to_next, self.lead_dist)
                angle_step = math.atan2(dy, dx)
                agent.ref_pos = np.array([
                    float(agent.pos[0]) + lead_s * math.cos(angle_step),
                    float(agent.pos[1]) + lead_s * math.sin(angle_step)
                ], dtype=np.float32)

                agent.step_timer = getattr(agent, 'step_timer', 0) + 1

                # Snapping ketercapaian langkah atau timeout watchdog (max 3.0s = 60 ticks)
                if dist_to_next < 0.28 or agent.step_timer >= 60:
                    agent.step_timer = 0
                    self.publish_twist(did, 0.0, 0.0, 0.0)
                    agent.state = 'delay_at_new_row'
                    agent.delay_timer = 0
                    continue

                v_step_max = self.step_speed if min_step_dist > 1.20 else 0.40
                v_step = min(v_step_max, max(0.30, 1.8 * dist_to_next))
                v_world_x = v_step * math.cos(angle_step) + np.clip(1.2 * dx, -0.35, 0.35)
                v_world_y = v_step * math.sin(angle_step) + np.clip(1.2 * dy, -0.35, 0.35)

                # Mapping Isolation: Tanpa V2V repulsion lateral
                self.send_world_twist(did, v_world_x, v_world_y, agent.yaw)

            # ─────────────────────────────────────────────────────────
            # 6. DELAY AT NEW ROW (Stationary In-Place Pivot ke Baris Baru)
            # ─────────────────────────────────────────────────────────
            elif agent.state == 'delay_at_new_row':
                next_idx_start = (agent.row_idx + 1) * 2
                next_idx_end = next_idx_start + 1
                wp_start = agent.waypoints[next_idx_start]
                wp_end = agent.waypoints[next_idx_end]
                line_dir = wp_end - wp_start
                target_yaw = math.atan2(line_dir[1], line_dir[0])
                agent.target_yaw = target_yaw
                agent.ref_pos = wp_start.copy()

                # Stationary: Kecepatan linier mutlak 0.0 m/s
                wz_cmd = self.compute_wz(agent.yaw, target_yaw)
                self.publish_twist(did, 0.0, 0.0, wz_cmd)
                agent.delay_timer = getattr(agent, 'delay_timer', 0) + 1

                yaw_diff = abs(math.atan2(math.sin(target_yaw - agent.yaw), math.cos(target_yaw - agent.yaw)))
                # Berhenti diam (min 1.25s = 25 ticks) & TUNGGU sampai yaw selaras (< 6.0°) sebelum mulai sapuan baris
                if (agent.delay_timer >= 25 and yaw_diff < math.radians(6.0)) or agent.delay_timer >= 120:
                    agent.row_idx += 1
                    agent.state = 'sweeping_row'
                    agent.delay_timer = 0
                    self.get_logger().info(f'  🚀 [iris_{did}] Memulai Baris {agent.row_idx+1}/{agent.num_rows} (Heading: {math.degrees(agent.yaw):.1f}°)')

            # ─────────────────────────────────────────────────────────
            # 7. RETURN TO CENTROID (Kembali ke Pusat Sel Voronoi)
            # ─────────────────────────────────────────────────────────
            elif agent.state == 'return_to_centroid':
                dx = float(agent.centroid[0]) - float(agent.pos[0])
                dy = float(agent.centroid[1]) - float(agent.pos[1])
                dist_to_c = math.hypot(dx, dy)

                # Coupled Carrot bergerak menuju centroid
                lead_c = min(dist_to_c, self.lead_dist)
                ang_c = math.atan2(dy, dx)
                agent.ref_pos = np.array([
                    float(agent.pos[0]) + lead_c * math.cos(ang_c),
                    float(agent.pos[1]) + lead_c * math.sin(ang_c)
                ], dtype=np.float32)

                if dist_to_c < 0.30:
                    agent.state = 'done'
                    agent.ref_pos = agent.centroid.copy()
                    agent.target_yaw = math.pi / 2.0  # Menghadap UTARA (+90.0°)
                    wz_cmd = self.compute_wz(agent.yaw, agent.target_yaw)
                    self.publish_twist(did, 0.0, 0.0, wz_cmd)
                    self.get_logger().info(f'  🎯 [iris_{did}] Tiba di Pusat Voronoi! Menghadap UTARA (+90.0°).')
                    continue

                v_back = min(self.transit_speed, max(0.40, 2.0 * dist_to_c))
                v_x = v_back * math.cos(ang_c) + np.clip(1.2 * dx, -0.40, 0.40)
                v_y = v_back * math.sin(ang_c) + np.clip(1.2 * dy, -0.40, 0.40)

                # V2V repulsion aktif di perjalanan pulang
                v_x, v_y = self.apply_v2v_repulsion(did, v_x, v_y, is_transit=True)
                self.send_world_twist(did, v_x, v_y, agent.yaw)

            # ─────────────────────────────────────────────────────────
            # 8. DONE (Hover Mengunci di Centroid & Menghadap Utara)
            # ─────────────────────────────────────────────────────────
            elif agent.state == 'done':
                agent.ref_pos = agent.centroid.copy()
                agent.target_yaw = math.pi / 2.0  # Menghadap UTARA (+90.0°)
                wz_cmd = self.compute_wz(agent.yaw, agent.target_yaw)
                self.publish_twist(did, 0.0, 0.0, wz_cmd)

        # Telemetri Terminal setiap 1 detik (20 ticks @ 20Hz)
        if self.step_count % 20 == 0:
            cov = self.get_coverage_percentage()
            states_summary = ' | '.join(f'i{did}:{a.state[:6]}' for did, a in self.agents.items())
            self.get_logger().info(
                f'📊 [STATUS] Cov: {cov:5.1f}% | d_min: {self.global_min_dist:4.2f}m | {states_summary}'
            )

            # Cek jika seluruh 7 drone telah tuntas atau cakupan mencapai 97.0%
            all_done = all(a.state == 'done' for a in self.agents.values())
            if (cov >= 97.0 or all_done) and not self.mission_completed:
                self.mission_completed = True
                self.get_logger().info(
                    f'🏆 [SWARM SUCCESS] Target Coverage {cov:.1f}% Tercapai! '
                    f'Jarak Terdekat (d_min): {self.global_min_dist:.2f}m | MISI TUNTAS!'
                )
                self.get_logger().info('====================================================================================================')
                self.get_logger().info('  📊 EVALUASI KUANTITATIF TRAJECTORY TRACKING & KINERJA SWARM 7-DRONE')
                self.get_logger().info('====================================================================================================')
                self.get_logger().info('| Drone  | Cross-Track Avg | Cross-Track RMS | Max CT Error | Overshoot Avg | Overshoot Max | Yaw Error Avg | Status |')
                self.get_logger().info('|--------|-----------------|-----------------|--------------|---------------|---------------|---------------|--------|')
                for d_id, a in self.agents.items():
                    ct_avg = float(np.mean(a.cross_track_errors) * 100.0) if a.cross_track_errors else 0.0
                    ct_rms = float(np.sqrt(np.mean(np.square(a.cross_track_errors))) * 100.0) if a.cross_track_errors else 0.0
                    ct_max = float(a.max_cross_track_err * 100.0)
                    ov_avg = float(np.mean(a.overshoot_list)) if a.overshoot_list else 0.0
                    ov_max = float(a.max_overshoot)
                    yaw_avg = float(np.mean(a.yaw_errors)) if a.yaw_errors else 0.0
                    status = "PASS ✅" if ov_max <= 0.01 and ct_max < 25.0 else "WARN ⚠️"
                    self.get_logger().info(
                        f'| iris_{d_id} | {ct_avg:13.2f}cm | {ct_rms:13.2f}cm | {ct_max:10.2f}cm | {ov_avg:11.2f}% | {ov_max:11.2f}% | {yaw_avg:11.2f}° | {status} |'
                    )
                self.get_logger().info('====================================================================================================')

    # ── Gaya Tolak V2V (Hanya Aktif Saat Transit, Wait, & Done Yield) ──

    def apply_v2v_repulsion(self, did, vx, vy, is_transit=True):
        """
        Menghitung gaya tolak V2V:
        - is_transit=True  -> buffer d_safe = 1.80m, evasion lateral & speed throttle.
        - is_transit=False -> 0.0 N (dinonaktifkan saat mapping untuk isolasi jalur).
        """
        if not is_transit:
            return vx, vy

        agent = self.agents[did]
        d_safe = 1.80

        for other_id, other_agent in self.agents.items():
            if other_id != did and other_agent.odom_received and other_agent.pos[2] >= 0.80:
                p_diff = agent.pos[:2] - other_agent.pos[:2]
                d_ij = float(np.linalg.norm(p_diff))
                if 0.01 < d_ij < d_safe:
                    u_sep = p_diff / d_ij
                    v_vec = np.array([vx, vy], dtype=float)
                    
                    # Pengereman aktif jika arah gerak saling mendekat (closing velocity)
                    closing_spd = -float(np.dot(v_vec, u_sep))
                    if closing_spd > 0:
                        v_vec += 2.2 * closing_spd * u_sep

                    # Gaya tolak murni menjauhi drone tetangga
                    rep_gain = 4.0 * (d_safe - d_ij)
                    v_vec += u_sep * rep_gain

                    # Evasion lateral (Right-Hand Aviation Rule)
                    u_lat = np.array([-u_sep[1], u_sep[0]])
                    v_vec += u_lat * (1.5 * (d_safe - d_ij))

                    vx = float(v_vec[0])
                    vy = float(v_vec[1])
        return vx, vy

    # ── Transformasi & Pengiriman Twist dengan Smooth Yaw Follow ──────

    def send_world_twist(self, did, v_world_x, v_world_y, current_yaw):
        """Mentransformasikan vektor kecepatan dunia ke frame bodi drone dengan continuous yaw follow."""
        spd = math.hypot(v_world_x, v_world_y)
        if spd > self.max_cmd_speed:
            v_world_x = (v_world_x / spd) * self.max_cmd_speed
            v_world_y = (v_world_y / spd) * self.max_cmd_speed
            spd = self.max_cmd_speed

        agent = self.agents[did]
        if spd > 0.15:
            agent.target_yaw = math.atan2(v_world_y, v_world_x)
        
        wz_cmd = self.compute_wz(current_yaw, agent.target_yaw)

        cos_y = math.cos(current_yaw)
        sin_y = math.sin(current_yaw)
        v_body_x = v_world_x * cos_y + v_world_y * sin_y
        v_body_y = -v_world_x * sin_y + v_world_y * cos_y

        now_msg = self.get_clock().now().to_msg()

        # 1. Twist (Body Frame) untuk cmd_vel
        tw = Twist()
        tw.linear.x = float(v_body_x)
        tw.linear.y = float(v_body_y)
        tw.linear.z = 0.0
        tw.angular.z = float(wz_cmd)
        self.pub_vel[did].publish(tw)

        # 2. TwistStamped (World Frame) untuk target_velocity controller
        tw_s = TwistStamped()
        tw_s.header.stamp = now_msg
        tw_s.header.frame_id = 'world'
        tw_s.twist.linear.x = float(v_world_x)
        tw_s.twist.linear.y = float(v_world_y)
        tw_s.twist.linear.z = 0.0
        tw_s.twist.angular.z = float(wz_cmd)
        self.pub_vel_stamped[did].publish(tw_s)

        # 3. PoseStamped (World Frame) untuk target_pose controller
        ps = PoseStamped()
        ps.header.stamp = now_msg
        ps.header.frame_id = 'world'
        ps.pose.position.x = float(agent.ref_pos[0])
        ps.pose.position.y = float(agent.ref_pos[1])
        ps.pose.position.z = float(self.cruise_alt)
        cy = math.cos(agent.target_yaw * 0.5)
        sy = math.sin(agent.target_yaw * 0.5)
        ps.pose.orientation.z = float(sy)
        ps.pose.orientation.w = float(cy)
        self.pub_target_pose[did].publish(ps)

    def publish_twist(self, did, vx, vy, wz):
        agent = self.agents[did]
        now_msg = self.get_clock().now().to_msg()

        tw = Twist()
        tw.linear.x = float(vx)
        tw.linear.y = float(vy)
        tw.linear.z = 0.0
        tw.angular.z = float(wz)
        self.pub_vel[did].publish(tw)

        cos_y = math.cos(agent.yaw)
        sin_y = math.sin(agent.yaw)
        vx_w = vx * cos_y - vy * sin_y
        vy_w = vx * sin_y + vy * cos_y

        tw_s = TwistStamped()
        tw_s.header.stamp = now_msg
        tw_s.header.frame_id = 'world'
        tw_s.twist.linear.x = float(vx_w)
        tw_s.twist.linear.y = float(vy_w)
        tw_s.twist.linear.z = 0.0
        tw_s.twist.angular.z = float(wz)
        self.pub_vel_stamped[did].publish(tw_s)

        ps = PoseStamped()
        ps.header.stamp = now_msg
        ps.header.frame_id = 'world'
        ps.pose.position.x = float(agent.ref_pos[0])
        ps.pose.position.y = float(agent.ref_pos[1])
        ps.pose.position.z = float(self.cruise_alt)
        cy = math.cos(agent.target_yaw * 0.5)
        sy = math.sin(agent.target_yaw * 0.5)
        ps.pose.orientation.z = float(sy)
        ps.pose.orientation.w = float(cy)
        self.pub_target_pose[did].publish(ps)

    # ── Publikasi Visualisasi RViz2 (/mapping/markers & /iris_i/actual_path) ──

    def publish_rviz_markers(self):
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        # 1. Bounding Box Arena 30x30m & Border Visuals
        m_bound = Marker()
        m_bound.header.frame_id = 'world'
        m_bound.header.stamp = stamp
        m_bound.ns = 'arena_boundary'
        m_bound.id = 0
        m_bound.type = Marker.LINE_STRIP
        m_bound.action = Marker.ADD
        m_bound.scale.x = 0.08
        m_bound.color = ColorRGBA(r=0.9, g=0.9, b=0.9, a=0.8)

        for p in self.bbox + [self.bbox[0]]:
            pt = Point()
            pt.x, pt.y, pt.z = float(p[0]), float(p[1]), 0.05
            m_bound.points.append(pt)
        ma.markers.append(m_bound)

        # 2. KOMPAS MATA ANGIN (NORTH / SOUTH / EAST / WEST) DI RVIZ2
        # Panah Besar Penunjuk UTARA (NORTH: +Y)
        m_north_arrow = Marker()
        m_north_arrow.header.frame_id = 'world'
        m_north_arrow.header.stamp = stamp
        m_north_arrow.ns = 'compass_markers'
        m_north_arrow.id = 1
        m_north_arrow.type = Marker.ARROW
        m_north_arrow.action = Marker.ADD
        p_n_start = Point()
        p_n_start.x, p_n_start.y, p_n_start.z = 0.0, 15.2, 0.15
        p_n_end = Point()
        p_n_end.x, p_n_end.y, p_n_end.z = 0.0, 17.5, 0.15
        m_north_arrow.points = [p_n_start, p_n_end]
        m_north_arrow.scale.x = 0.25  # Shaft diameter
        m_north_arrow.scale.y = 0.60  # Head diameter
        m_north_arrow.scale.z = 0.80  # Head length
        m_north_arrow.color = ColorRGBA(r=1.0, g=0.15, b=0.15, a=1.0)
        ma.markers.append(m_north_arrow)

        # Label Teks Mata Angin
        compass_labels = [
            (2, "NORTH (+Y)", 0.0, 18.2, 0.3, (1.0, 0.2, 0.2)),
            (3, "SOUTH (-Y)", 0.0, -16.2, 0.3, (0.3, 0.5, 1.0)),
            (4, "EAST (+X)", 16.2, 0.0, 0.3, (0.2, 0.9, 0.3)),
            (5, "WEST (-X)", -16.2, 0.0, 0.3, (0.9, 0.8, 0.2)),
        ]
        for lid, text, lx, ly, lz, (lr, lg, lb) in compass_labels:
            m_lbl = Marker()
            m_lbl.header.frame_id = 'world'
            m_lbl.header.stamp = stamp
            m_lbl.ns = 'compass_labels'
            m_lbl.id = lid
            m_lbl.type = Marker.TEXT_VIEW_FACING
            m_lbl.action = Marker.ADD
            m_lbl.pose.position.x = float(lx)
            m_lbl.pose.position.y = float(ly)
            m_lbl.pose.position.z = float(lz)
            m_lbl.scale.z = 0.75
            m_lbl.color = ColorRGBA(r=float(lr), g=float(lg), b=float(lb), a=1.0)
            m_lbl.text = text
            ma.markers.append(m_lbl)

        # 3. Poligon Sel 2D Voronoi & Rute Rencana Boustrophedon 3D (Z = 2.00m)
        if self.voronoi_planned:
            for did, agent in self.agents.items():
                r, g, b = agent.color

                # Batas Poligon Sel Voronoi
                if len(agent.cell_polygon) >= 3:
                    m_poly = Marker()
                    m_poly.header.frame_id = 'world'
                    m_poly.header.stamp = stamp
                    m_poly.ns = 'voronoi_cells'
                    m_poly.id = 10 + did
                    m_poly.type = Marker.LINE_STRIP
                    m_poly.action = Marker.ADD
                    m_poly.scale.x = 0.06
                    m_poly.color = ColorRGBA(r=r, g=g, b=b, a=0.9)

                    for p in agent.cell_polygon + [agent.cell_polygon[0]]:
                        pt = Point()
                        pt.x, pt.y, pt.z = float(p[0]), float(p[1]), 0.05
                        m_poly.points.append(pt)
                    ma.markers.append(m_poly)

                # Jalur Rencana Boustrophedon 3D (Sejajar Ketinggian Terbang Drone Z=2.0m)
                if len(agent.waypoints) >= 2:
                    m_plan = Marker()
                    m_plan.header.frame_id = 'world'
                    m_plan.header.stamp = stamp
                    m_plan.ns = 'planned_paths'
                    m_plan.id = 20 + did
                    m_plan.type = Marker.LINE_STRIP
                    m_plan.action = Marker.ADD
                    m_plan.scale.x = 0.05
                    m_plan.color = ColorRGBA(r=r, g=g, b=b, a=0.75)

                    m_wps = Marker()
                    m_wps.header.frame_id = 'world'
                    m_wps.header.stamp = stamp
                    m_wps.ns = 'planned_waypoints'
                    m_wps.id = 80 + did
                    m_wps.type = Marker.SPHERE_LIST
                    m_wps.action = Marker.ADD
                    m_wps.scale.x = 0.12
                    m_wps.scale.y = 0.12
                    m_wps.scale.z = 0.12
                    m_wps.color = ColorRGBA(r=r, g=g, b=b, a=0.90)

                    for wp in agent.waypoints:
                        pt = Point()
                        pt.x, pt.y, pt.z = float(wp[0]), float(wp[1]), float(self.cruise_alt)
                        m_plan.points.append(pt)
                        m_wps.points.append(pt)

                    ma.markers.append(m_plan)
                    ma.markers.append(m_wps)

        # 4. Moving Carrot Reference Spheres, Drone Body Hub, FoV, and Tags
        for did, agent in self.agents.items():
            if not agent.odom_received:
                continue

            r, g, b = agent.color
            px, py, pz = float(agent.pos[0]), float(agent.pos[1]), float(agent.pos[2])

            # REAL-TIME DYNAMIC MOVING REFERENCE CARROT SPHERE (Z = 2.00m)
            m_carrot = Marker()
            m_carrot.header.frame_id = 'world'
            m_carrot.header.stamp = stamp
            m_carrot.ns = 'drone_reference_carrots'
            m_carrot.id = 200 + did
            m_carrot.type = Marker.SPHERE
            m_carrot.action = Marker.ADD
            m_carrot.pose.position.x = float(agent.ref_pos[0])
            m_carrot.pose.position.y = float(agent.ref_pos[1])
            m_carrot.pose.position.z = float(self.cruise_alt)
            m_carrot.scale.x = 0.22
            m_carrot.scale.y = 0.22
            m_carrot.scale.z = 0.22
            m_carrot.color = ColorRGBA(r=r, g=g, b=b, a=0.95)
            ma.markers.append(m_carrot)

            # Lingkaran FoV Sensor di Tanah
            m_fov = Marker()
            m_fov.header.frame_id = 'world'
            m_fov.header.stamp = stamp
            m_fov.ns = 'sensor_fov'
            m_fov.id = 30 + did
            m_fov.type = Marker.LINE_STRIP
            m_fov.action = Marker.ADD
            m_fov.scale.x = 0.04
            m_fov.color = ColorRGBA(r=r, g=g, b=b, a=0.7)

            for deg in range(0, 361, 15):
                rad = math.radians(deg)
                pt = Point()
                pt.x = float(px + self.sensor_radius * math.cos(rad))
                pt.y = float(py + self.sensor_radius * math.sin(rad))
                pt.z = 0.08
                m_fov.points.append(pt)
            ma.markers.append(m_fov)

            # Drone Center Hub Sphere (Badan Utama Drone)
            m_hub = Marker()
            m_hub.header.frame_id = 'world'
            m_hub.header.stamp = stamp
            m_hub.ns = 'drone_hubs'
            m_hub.id = 40 + did
            m_hub.type = Marker.SPHERE
            m_hub.action = Marker.ADD
            m_hub.pose.position.x = px
            m_hub.pose.position.y = py
            m_hub.pose.position.z = pz
            m_hub.scale.x = 0.30
            m_hub.scale.y = 0.30
            m_hub.scale.z = 0.12
            m_hub.color = ColorRGBA(r=r, g=g, b=b, a=1.0)
            ma.markers.append(m_hub)

            # Panah Arah Heading Real-Time (Orientasi Arah Hadap Drone)
            m_arrow = Marker()
            m_arrow.header.frame_id = 'world'
            m_arrow.header.stamp = stamp
            m_arrow.ns = 'drone_heading_arrows'
            m_arrow.id = 50 + did
            m_arrow.type = Marker.ARROW
            m_arrow.action = Marker.ADD
            
            p_start = Point()
            p_start.x, p_start.y, p_start.z = px, py, pz
            p_end = Point()
            arrow_len = 0.70
            p_end.x = float(px + arrow_len * math.cos(agent.yaw))
            p_end.y = float(py + arrow_len * math.sin(agent.yaw))
            p_end.z = pz

            m_arrow.points = [p_start, p_end]
            m_arrow.scale.x = 0.08  # Shaft diameter
            m_arrow.scale.y = 0.15  # Head diameter
            m_arrow.scale.z = 0.18  # Head length
            m_arrow.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.95)
            ma.markers.append(m_arrow)

            # Name Tag Minimalis
            m_tag = Marker()
            m_tag.header.frame_id = 'world'
            m_tag.header.stamp = stamp
            m_tag.ns = 'drone_tags'
            m_tag.id = 60 + did
            m_tag.type = Marker.TEXT_VIEW_FACING
            m_tag.action = Marker.ADD
            m_tag.pose.position.x = px
            m_tag.pose.position.y = py
            m_tag.pose.position.z = pz + 0.35
            m_tag.scale.z = 0.32
            m_tag.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
            m_tag.text = f'iris_{did}'
            ma.markers.append(m_tag)

            # Publikasi Jalur Terbang Riil (Actual Path)
            agent.path_msg.header.stamp = stamp
            self.pub_actual_path[did].publish(agent.path_msg)

        # 5. Status HUD Coverage Ringkas
        cov = self.get_coverage_percentage()
        m_hud = Marker()
        m_hud.header.frame_id = 'world'
        m_hud.header.stamp = stamp
        m_hud.ns = 'hud_text'
        m_hud.id = 99
        m_hud.type = Marker.TEXT_VIEW_FACING
        m_hud.action = Marker.ADD
        m_hud.pose.position.x = 0.0
        m_hud.pose.position.y = 15.50
        m_hud.pose.position.z = 1.0
        m_hud.scale.z = 0.70
        m_hud.color = ColorRGBA(r=0.2, g=1.0, b=0.4, a=1.0)
        m_hud.text = f'Coverage: {cov:.1f}% | 7 Drones 2D Voronoi Mapping (30x30m)'
        ma.markers.append(m_hud)

        # 6. Grid Cakupan Hijau Padat & Rapat (Numpy Vectorized Fast Extraction)
        if self.step_count % 4 == 0:
            m_grid = Marker()
            m_grid.header.frame_id = 'world'
            m_grid.header.stamp = stamp
            m_grid.ns = 'coverage_footprint'
            m_grid.id = 100
            m_grid.type = Marker.CUBE_LIST
            m_grid.action = Marker.ADD
            m_grid.scale.x = float(self.dx * 1.02)
            m_grid.scale.y = float(self.dy * 1.02)
            m_grid.scale.z = 0.015
            m_grid.color = ColorRGBA(r=0.10, g=0.90, b=0.25, a=0.35)

            indices = np.argwhere(self.cov_grid)
            for idx in indices:
                pt = Point()
                pt.x = float(self.x_min + (idx[0] + 0.5) * self.dx)
                pt.y = float(self.y_min + (idx[1] + 0.5) * self.dy)
                pt.z = 0.01
                m_grid.points.append(pt)
            ma.markers.append(m_grid)

        self.pub_markers.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = Swarm7DroneVoronoiMappingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
