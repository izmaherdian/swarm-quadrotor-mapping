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
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, TwistStamped, Point, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Int32MultiArray
from matplotlib.path import Path as MplPath
from shapely.geometry import Polygon as SpPolygon, MultiPolygon as SpMultiPolygon
from shapely.ops import unary_union


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


def generate_boustrophedon(polygon, sweep_spacing=1.45, margin=0.02, start_from_top=False, obstacles=None):
    """
    Menghasilkan rute sapuan Lawnmower horizontal zigzag di dalam sel poligon.
    Jika start_from_top=True, urutan baris dimulai dari Y tertinggi ke terendah.
    Jika obstacles diberikan, memangkas ujung-ujung baris agar berjarak aman (>= 1.15m) dari pusat rintangan.
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

        # Pemangkasan batas ujung baris jika dekat dengan rintangan statis (Buffer clearance >= 1.35m)
        if obstacles:
            for obs in obstacles:
                ox, oy = obs[2], obs[3]
                if abs(y - oy) < 1.35:
                    d_crit = math.sqrt(max(0.01, 1.35**2 - (y - oy)**2))
                    if abs(x_left - ox) < d_crit:
                        if ox > x_left:
                            x_left = min(x_left, ox - d_crit - 0.10)
                        else:
                            x_left = max(x_left, ox + d_crit + 0.10)
                    if abs(x_right - ox) < d_crit:
                        if ox < x_right:
                            x_right = max(x_right, ox + d_crit + 0.10)
                        else:
                            x_right = min(x_right, ox - d_crit - 0.10)

        # Lewatkan baris yang terlalu pendek (< 0.60m) atau terbalik
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

        # Fault Tolerance & Dynamic Failure Recovery State
        self.is_alive = True
        self.wp_flags = []  # True = rute sel sendiri, False = rute recovery
        self.own_num_rows = 0
        self.own_waypoints = []
        self.last_odom_time = None

        # LiDAR Sensing & Geodesic Arc Obstacle Avoidance State
        self.lidar_ranges = None
        self.min_dist_to_obs = float('inf')
        self.bypass_state = 'none'       # 'none', 'arc_contour'
        self.bypass_side = 0             # +1 (kiri) atau -1 (kanan) - Hysteresis Locked
        self.bypass_obs_id = None
        self.last_bypassed_obs_id = None # Mencegah chattering re-trigger pada rintangan yang sama di baris yang sama
        self.my_static_obstacles = []    # Rintangan statis yang berlokasi eksklusif di dalam sel Voronoi drone ini
        self.transit_waypoints = []      # Koridor transit aman menuju sel
        self.transit_wp_idx = 0
        self.dyn_obs_yielding = False
        self.dyn_evade_side = 0          # +1 (kiri) atau -1 (kanan) - Arah Proactive Sidestep Dinamis


class DynamicObstacleKalmanFilter:
    """
    4-State Constant Velocity (CV) Kalman Filter dengan Ekstrapolasi Kinematika Harmonik Eksak:
    State: x = [pos_x, pos_y, vel_x, vel_y]^T
    Mengestimasi posisi dan kecepatan rintangan dinamis secara kontinu,
    serta memprediksi posisi masa depan secara eksak sinusoidal untuk horizon hingga 5.0 detik.
    """
    def __init__(self, init_pos=np.array([0.0, 0.0]), init_vel=np.array([0.0, 0.0])):
        self.x = np.array([init_pos[0], init_pos[1], init_vel[0], init_vel[1]], dtype=float)
        self.P = np.diag([0.1, 0.1, 1.0, 1.0])
        self.Q = np.diag([0.005, 0.005, 0.10, 0.10])
        self.R = np.diag([0.02, 0.02])
        self.H = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0]
        ], dtype=float)

    def predict(self, dt=0.05):
        dt_val = max(0.001, float(dt))
        F = np.array([
            [1.0, 0.0, dt_val, 0.0],
            [0.0, 1.0, 0.0, dt_val],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ], dtype=float)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q

    def update(self, z_meas):
        y = z_meas - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        I = np.eye(4)
        self.P = (I - K @ self.H) @ self.P

    @property
    def pos(self):
        return self.x[:2].copy()

    @property
    def vel(self):
        return self.x[2:4].copy()

    def predict_pos(self, tau):
        """Memprediksi posisi masa depan linier pada t + tau."""
        return self.x[:2] + self.x[2:4] * float(tau)

    def predict_harmonic_pos(self, tau, omega=0.15):
        """Memprediksi posisi masa depan pada t + tau menggunakan kinematika harmonik analitik eksak."""
        tau_val = float(tau)
        p = self.x[:2]
        v = self.x[2:4]
        if abs(omega) > 1e-4:
            cos_wt = math.cos(omega * tau_val)
            sin_wt = math.sin(omega * tau_val)
            return p * cos_wt + (v / omega) * sin_wt
        else:
            return p + v * tau_val


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
            (0.00, 0.75, 0.65),  # Drone 1: Teal / Hijau Toska
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

        # ── Parameter 4 Skema Pemetaan & Konfigurasi Lingkungan ──────
        self.declare_parameter('scheme', 1)
        self.declare_parameter('obstacles', 'preset')
        self.declare_parameter('enable_wind', False)
        self.declare_parameter('enable_obstacles', False)

        self.scheme = int(self.get_parameter('scheme').value)
        obs_mode = str(self.get_parameter('obstacles').value)
        self.enable_wind = bool(self.get_parameter('enable_wind').value) or (self.scheme in [2, 4])
        self.enable_obstacles = bool(self.get_parameter('enable_obstacles').value) or (self.scheme in [3, 4])

        scheme_names = {
            1: "Skema 1: Nominal Mapping (Zero Disturbance)",
            2: "Skema 2: Dryden Wind Turbulence Mapping",
            3: "Skema 3: Obstacle Avoidance Mapping (9 Static + 2 Dynamic 'X')",
            4: "Skema 4: Combined Disturbance & Obstacles Mapping"
        }
        self.get_logger().info("=========================================================================")
        self.get_logger().info(f"🚁 SWARM KOORDINATOR AKTIF: [{scheme_names.get(self.scheme, 'Skema Custom')}]")
        self.get_logger().info(f"   🌪️  Wind Disturbance: {'AKTIF' if self.enable_wind else 'NONAKTIF'}")
        self.get_logger().info(f"   🚧 Obstacles Engine: {'AKTIF (9 Statis + 2 Dinamis Pola X)' if self.enable_obstacles else 'NONAKTIF'}")
        self.get_logger().info("=========================================================================")

        # ── Definisi Rintangan Statis (9 Silinder di Sel Voronoi) ─────
        self.static_obstacles = [
            # (id, cell_did, x, y, radius, height, color_rgb)
            (101, 2, -1.5,  9.5, 0.40, 4.0, (1.0, 0.60, 0.0)),
            (102, 3,  4.0,  6.0, 0.40, 4.0, (1.0, 0.95, 0.1)),
            (103, 3,  6.5,  9.5, 0.40, 4.0, (1.0, 0.95, 0.1)),
            (104, 4, -8.0, -2.0, 0.40, 4.0, (0.1, 0.95, 0.2)),
            (105, 4, -5.0,  -7.5, 0.40, 4.0, (0.1, 0.95, 0.2)),
            (106, 4, -10.5, -12.5, 0.40, 4.0, (0.1, 0.95, 0.2)),
            (107, 5,  6.0,  -4.0, 0.40, 4.0, (0.1, 0.85, 1.0)),
            (108, 7,  0.0,  2.5, 0.40, 4.0, (0.9, 0.20, 1.0)),
            (109, 7,  2.5, -9.0, 0.40, 4.0, (0.9, 0.20, 1.0)),
        ]

        # ── Definisi Rintangan Dinamis (2 Silinder Pola 'X') ──────────
        self.dynamic_obstacles = [
            {'id': 201, 'pos': np.array([-10.0, 10.0], dtype=float), 'vel': np.zeros(2), 'color': (1.0, 0.1, 0.1), 'name': 'dynamic_obs_1'},
            {'id': 202, 'pos': np.array([ 10.0, 10.0], dtype=float), 'vel': np.zeros(2), 'color': (1.0, 0.5, 0.0), 'name': 'dynamic_obs_2'},
        ]
        self.kf_dyn_obs = [
            DynamicObstacleKalmanFilter(init_pos=np.array([-10.0, 10.0])),
            DynamicObstacleKalmanFilter(init_pos=np.array([ 10.0, 10.0]))
        ]
        self.last_dyn_obs_t = None
        self.pub_dyn_obs_vel_1 = self.create_publisher(Twist, '/model/dynamic_obs_1/cmd_vel', 10)
        self.pub_dyn_obs_vel_2 = self.create_publisher(Twist, '/model/dynamic_obs_2/cmd_vel', 10)

        # ── Masking Grid Okupansi untuk Rintangan Statis ───────────────
        self.obstacle_mask = np.zeros((self.grid_n, self.grid_n), dtype=bool)
        if self.enable_obstacles:
            for _, _, ox, oy, rad, _, _ in self.static_obstacles:
                cx_idx = int((ox - self.x_min) / self.dx)
                cy_idx = int((oy - self.y_min) / self.dy)
                r_cells = int(math.ceil((rad + 0.05) / self.dx))
                for i in range(max(0, cx_idx - r_cells), min(self.grid_n, cx_idx + r_cells + 1)):
                    for j in range(max(0, cy_idx - r_cells), min(self.grid_n, cy_idx + r_cells + 1)):
                        cell_x = self.x_min + (i + 0.5) * self.dx
                        cell_y = self.y_min + (j + 0.5) * self.dy
                        if (cell_x - ox)**2 + (cell_y - oy)**2 <= (rad + 0.05)**2:
                            self.obstacle_mask[i, j] = True

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
        self.sub_lidar = {}
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
            self.sub_lidar[did] = self.create_subscription(
                LaserScan,
                f'/{agent.ns}/lidar_scan',
                lambda msg, d=did: self.lidar_callback(d, msg),
                10
            )

        # ── Dynamic Obstacle Odometry Subscriptions (Ground-Truth dari Gazebo) ──
        self.sub_dyn_odom_1 = self.create_subscription(
            Odometry,
            '/model/dynamic_obs_1/odometry',
            lambda msg: self.dyn_obs_odom_callback(0, msg),
            qos_reliable
        )
        self.sub_dyn_odom_2 = self.create_subscription(
            Odometry,
            '/model/dynamic_obs_2/odometry',
            lambda msg: self.dyn_obs_odom_callback(1, msg),
            qos_reliable
        )

        self.pub_markers = self.create_publisher(MarkerArray, '/mapping/markers', 10)
        self.pub_dead_cells = self.create_publisher(MarkerArray, '/mapping/dead_cells', 10)

        # ── Fault Tolerance & Dynamic Failure Recovery State ─────────
        self.dead_drones = set()
        self.merged_dead_comp_polys = []
        self.pending_recovery_pts = []
        self.recovery_active = False

        # Topic Subscriber untuk Emergency Trigger dari Terminal 2
        self.sub_kill = self.create_subscription(
            Int32MultiArray,
            '/swarm/kill_drone',
            self.kill_drone_callback,
            10
        )

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
        agent.last_odom_time = self.get_clock().now()

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

        # Inisialisasi generator terdistribusi simetris untuk partisi 3 Bawah, 1 Tengah, 3 Atas
        seed_points = [
            np.array([-9.0, -9.0]), np.array([0.0, -9.5]), np.array([9.0, -9.0]),  # 3 Wilayah Bawah
            np.array([0.0, 0.0]),                                                  # 1 Wilayah Tengah
            np.array([-9.0, 9.0]),  np.array([0.0, 9.5]),  np.array([9.0, 9.0])    # 3 Wilayah Atas
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
            raw_cell = [np.array(v, dtype=float) for v in self.bbox]
            for other_did, pj in drone_to_gen.items():
                if other_did != did:
                    cell = clip_voronoi_margin(cell, pi, pj, margin=0.45)
                    raw_cell = clip_voronoi(raw_cell, pi, pj)

            agent.cell_polygon = cell
            agent.raw_cell_polygon = raw_cell
            agent.centroid = poly_centroid(cell)
            pts = np.array(cell, dtype=float)
            y_mid = 0.5 * (pts[:, 1].min() + pts[:, 1].max())

            poly_raw = MplPath(np.array(raw_cell))
            agent.my_static_obstacles = [obs for obs in self.static_obstacles if poly_raw.contains_point(np.array([obs[2], obs[3]], dtype=float))]

            start_from_top = bool(agent.pos[1] > y_mid)
            wps = generate_boustrophedon(cell, sweep_spacing=1.45, margin=0.02, start_from_top=start_from_top, obstacles=agent.my_static_obstacles)
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
                            start_from_top=(not curr_start_from_top),
                            obstacles=self.agents[j].my_static_obstacles
                        )
                        self.agents[j].waypoints = wps_new
                        self.agents[j].num_rows = max(1, len(wps_new) // 2)
                        self.agents[j].ref_pos = wps_new[0].copy()
                        break
                if conflict_found:
                    break

        for did, agent in self.agents.items():
            agent.wp_flags = [True] * agent.num_rows
            agent.own_num_rows = agent.num_rows
            agent.own_waypoints = list(agent.waypoints)
            dist_to_start = float(np.linalg.norm(agent.pos[:2] - agent.waypoints[0]))

            # Alokasikan rintangan statis secara eksklusif ke sel drone yang memuat rintangan tersebut
            poly_raw = MplPath(np.array(agent.raw_cell_polygon))
            agent.my_static_obstacles = []
            for obs in self.static_obstacles:
                obs_id, _, ox, oy, rad, height, color = obs
                obs_center = np.array([ox, oy], dtype=float)
                if poly_raw.contains_point(obs_center):
                    agent.my_static_obstacles.append(obs)

            obs_ids = [o[0] for o in agent.my_static_obstacles]

            # Rancang koridor transit aman bebas dari seluruh rintangan statis di arena
            p_start = agent.waypoints[0].copy()
            p_stage = agent.pos[:2].copy()
            u_tr = p_start - p_stage
            dist_tr = float(np.linalg.norm(u_tr))
            u_tr_hat = u_tr / max(1e-3, dist_tr)

            needs_intermediate = False
            for obs in self.static_obstacles:
                obs_center = np.array([obs[2], obs[3]], dtype=float)
                r_obs = obs_center - p_stage
                s_proj = float(np.dot(r_obs, u_tr_hat))
                if 0.5 < s_proj < (dist_tr - 0.5):
                    p_proj = p_stage + s_proj * u_tr_hat
                    d_perp = float(np.linalg.norm(obs_center - p_proj))
                    if d_perp < (obs[4] + 0.65):
                        needs_intermediate = True
                        break

            if p_start[1] > -5.0:  # Sel bagian atas / tengah
                if p_start[0] < -4.0:  # Sel Kiri Atas (e.g. iris_5)
                    agent.transit_waypoints = [
                        np.array([-14.00, -8.00], dtype=np.float32),
                        p_start.copy()
                    ]
                elif p_start[0] > 4.0:  # Sel Kanan Atas (e.g. iris_7)
                    agent.transit_waypoints = [
                        np.array([10.00, -8.00], dtype=np.float32),
                        p_start.copy()
                    ]
                else:  # Sel Tengah (e.g. iris_2, iris_3)
                    agent.transit_waypoints = [
                        np.array([p_stage[0], -14.00], dtype=np.float32),
                        p_start.copy()
                    ]
            else:
                agent.transit_waypoints = [p_start.copy()]
            agent.transit_wp_idx = 0

            self.get_logger().info(
                f'  -> [iris_{did}] Sel ({len(agent.cell_polygon)} simpul) | {agent.num_rows} Baris | '
                f'Start: ({agent.waypoints[0][0]:.2f}, {agent.waypoints[0][1]:.2f}) | Centroid: ({agent.centroid[0]:.2f}, {agent.centroid[1]:.2f}) | '
                f'Transit: {dist_tr:.2f}m ({len(agent.transit_waypoints)} WPs) | Obs Sel: {obs_ids if obs_ids else "Nihil"}'
            )

        self.voronoi_planned = True

    # ── Update Grid Cakupan (Coverage) ───────────────────────────────

    def update_coverage(self):
        """Memperbarui matriks okupansi sensor FoV (R=0.95m) hanya saat drone aktif melakukan pemetaan."""
        for did, agent in self.agents.items():
            if not agent.is_alive or agent.state == 'dead':
                continue
            if not agent.odom_received or agent.pos[2] < 0.8:
                continue

            # Hanya catat coverage saat drone sedang aktif menyapu baris pemetaan
            # (BUKAN saat takeoff, pivot transit, transit ke titik awal, atau align start yaw)
            if agent.state not in ('sweeping_row', 'delay_at_corner_end', 'stepping_vertical', 'delay_at_new_row', 'sweeping_recovery'):
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

    def lidar_callback(self, did, msg):
        """Menerima dan menyimpan scan LiDAR 2D/3D dari masing-masing drone."""
        if did in self.agents:
            self.agents[did].lidar_ranges = np.array(msg.ranges, dtype=np.float32)

    def dyn_obs_odom_callback(self, obs_idx, msg):
        """Menerima odometri fisik riil rintangan dinamis dari Gazebo Harmonic (100% Sinkron dengan Gazebo)."""
        pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y], dtype=float)
        vel = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y], dtype=float)
        if obs_idx < len(self.dynamic_obstacles):
            self.dynamic_obstacles[obs_idx]['pos'] = pos
            self.dynamic_obstacles[obs_idx]['vel'] = vel
            self.kf_dyn_obs[obs_idx].update(pos)

    def update_dynamic_obstacles(self, t_sim):
        """Menggerakkan 2 rintangan silinder dinamis membentuk pola 'X' diagonal secara harmonik tanpa tabrakan di (0,0) & update Kalman Filter."""
        if not self.enable_obstacles:
            return
        omega1 = 0.15
        omega2 = 0.11
        # Dynamic Obstacle 1: Diagonal NW <-> SE (-10, 10) <-> (10, -10)
        x1 = -10.0 * math.cos(omega1 * t_sim)
        y1 =  10.0 * math.cos(omega1 * t_sim)
        vx1 =  10.0 * omega1 * math.sin(omega1 * t_sim)
        vy1 = -10.0 * omega1 * math.sin(omega1 * t_sim)

        tw1 = Twist()
        tw1.linear.x = float(vx1)
        tw1.linear.y = float(vy1)
        self.pub_dyn_obs_vel_1.publish(tw1)

        # Dynamic Obstacle 2: Diagonal NE <-> SW (10, 10) <-> (-10, -10)
        x2 =  10.0 * math.cos(omega2 * t_sim)
        y2 =  10.0 * math.cos(omega2 * t_sim)
        vx2 = -10.0 * omega2 * math.sin(omega2 * t_sim)
        vy2 = -10.0 * omega2 * math.sin(omega2 * t_sim)

        tw2 = Twist()
        tw2.linear.x = float(vx2)
        tw2.linear.y = float(vy2)
        self.pub_dyn_obs_vel_2.publish(tw2)

        # Update Kalman Filters untuk Estimasi Posisi & Kecepatan
        dt = 0.05
        if self.last_dyn_obs_t is not None and t_sim > self.last_dyn_obs_t:
            dt = max(0.001, t_sim - self.last_dyn_obs_t)
        self.last_dyn_obs_t = t_sim

        self.kf_dyn_obs[0].predict(dt)
        self.kf_dyn_obs[1].predict(dt)

    def compute_obstacle_avoidance_offset(self, did, agent, p_nom_dir):
        """
        Menghitung pergeseran lateral & modulasi kecepatan berbasis Perpendicular Tangent Orbit, 
        Fast Corridor Dynamic Evasion, & Respon Low-Level:
        1. Rintangan Statis (9 silinder): Perpendicular Tangent Orbit (R = 0.90m, v = 0.80 m/s) & Direct Line Merge:
           - Vektor tangensial MURNI TEGAK LURUS terhadap jari-jari silinder (dot product == 0.00).
           - Arc Carrot Reference (phi_carrot = phi + dphi) menjaga low-level position error & velocity feedforward bekerja seirama ke depan.
           - Yaw continuous align dengan arah pergerakan orbit.
           - Direct Line Merge: Begitu melintasi meridian (prog_past >= 0.35m) dan arah busur kembali selaras dengan garis (dot >= 0.55),
             state bypass SEGERA berakhir ('none'), menyatu langsung ke garis sapuan tanpa bergelombang/menjauh.
        2. Rintangan Dinamis (2 silinder 'X'): Fast Corridor Evasion (Koridor 1.80m, v = 1.20 m/s)
           - Menghitung jarak lateral d_perp dan TTC longitudinal terhadap sumbu lintasan diagonal rintangan.
           - Jika berada di dalam koridor 1.80m dan rintangan mendekat (TTC <= 5.0s): Geser seketika ke samping luar koridor pada 1.20 m/s.
           - Menggeser ref_pos secara instan ke area aman di luar koridor.
        Mengembalikan tuple: (v_avoid, speed_scale)
        """
        if not self.enable_obstacles:
            return np.zeros(2, dtype=np.float32), 1.0

        p_drone = agent.pos[:2]
        v_drone = p_nom_dir.copy() if np.linalg.norm(p_nom_dir) > 0.05 else np.zeros(2, dtype=float)
        speed_nom = float(np.linalg.norm(v_drone))
        unit_vel = v_drone / speed_nom if speed_nom > 0.05 else np.array([math.cos(agent.yaw), math.sin(agent.yaw)])

        min_obs_dist = float('inf')
        v_avoid_static = np.zeros(2, dtype=np.float32)
        speed_scale_static = 1.0

        # ── 1. EVALUASI RINTANGAN STATIS (PURE CIRCULAR TANGENT ORBIT & STRICT CELL ISOLATION) ──
        R_ORBIT = 1.15          # Radius orbit konstan (m) -> clearance bersih 0.75m dari silinder 0.40m
        V_NOM = 0.80            # Kecepatan jelajah bypass (m/s)

        # Rintangan statis dievaluasi SECARA EKSKLUSIF hanya yang berada di dalam sel Voronoi drone ini
        relevant_obs_list = getattr(agent, 'my_static_obstacles', [])

        u_line = unit_vel.copy()
        n_line = np.array([-u_line[1], u_line[0]], dtype=float)

        active_bypass_obs = None

        for obs in relevant_obs_list:
            obs_id, _, ox, oy, rad, _, _ = obs
            # Mencegah re-triggering rintangan yang sama yang baru saja selesai di baris ini
            if obs_id == getattr(agent, 'last_bypassed_obs_id', None) and agent.bypass_state != 'arc_contour':
                continue

            obs_center = np.array([ox, oy], dtype=float)
            r_vec = obs_center - p_drone
            d_center = float(np.linalg.norm(r_vec))
            d_surf = d_center - rad
            min_obs_dist = min(min_obs_dist, d_surf)

            # Jarak lateral pusat rintangan dari garis lintasan
            w_0 = float(np.dot(r_vec, n_line))

            # Jika rintangan memotong koridor bahaya tabrakan (|w_0| < 0.90m)
            if abs(w_0) < 0.90:
                d_0 = math.sqrt(max(0.01, R_ORBIT**2 - w_0**2))
                # Titik potong garis lintasan dengan lingkaran orbit
                p_proj = obs_center - w_0 * n_line
                p_entry = p_proj - d_0 * u_line
                p_exit = p_proj + d_0 * u_line

                s_to_entry = float(np.dot(p_entry - p_drone, u_line))
                s_to_exit = float(np.dot(p_exit - p_drone, u_line))

                # Deteksi jika drone mendekati titik entry atau sedang berada di dalam zona orbit sebelum titik exit
                if s_to_entry <= 0.45 and s_to_exit > 0.10 and d_surf < 1.25:
                    active_bypass_obs = (obs, w_0, d_0, p_entry, p_exit)
                    break

        if active_bypass_obs is not None:
            obs, w_0, d_0, p_entry, p_exit = active_bypass_obs
            obs_id, _, ox, oy, rad, _, _ = obs
            obs_center = np.array([ox, oy], dtype=float)

            theta_entry = math.atan2(p_entry[1] - obs_center[1], p_entry[0] - obs_center[0])
            theta_exit = math.atan2(p_exit[1] - obs_center[1], p_exit[0] - obs_center[0])

            # Inisialisasi arah rotasi orbit (menjauhi rintangan ke sisi terbuka)
            if agent.bypass_obs_id != obs_id or agent.bypass_side == 0:
                open_sign = -1.0 if w_0 >= 0.0 else +1.0
                if abs(w_0) < 0.05 and hasattr(agent, 'cell_polygon') and len(agent.cell_polygon) >= 3:
                    poly_path = MplPath(np.array(agent.cell_polygon))
                    p_apex_cand_l = obs_center + n_line * R_ORBIT
                    p_apex_cand_r = obs_center - n_line * R_ORBIT
                    if poly_path.contains_point(p_apex_cand_l) and not poly_path.contains_point(p_apex_cand_r):
                        open_sign = +1.0
                    elif poly_path.contains_point(p_apex_cand_r) and not poly_path.contains_point(p_apex_cand_l):
                        open_sign = -1.0

                theta_apex = math.atan2(open_sign * n_line[1], open_sign * n_line[0])
                d_entry_to_apex = math.atan2(math.sin(theta_apex - theta_entry), math.cos(theta_apex - theta_entry))
                sigma_rot = +1.0 if d_entry_to_apex > 0 else -1.0

                agent.bypass_obs_id = obs_id
                agent.bypass_side = int(sigma_rot)
                agent.bypass_state = 'arc_contour'
                self.get_logger().info(
                    f'  🔄 [iris_{did}] Rintangan Statis #{obs_id} (w={w_0:.2f}m)! '
                    f'Pure Circular Orbit (R={R_ORBIT}m, {"CCW" if sigma_rot > 0 else "CW"})...'
                )

            sigma_rot = float(agent.bypass_side)

            # Posisi sudut drone saat ini relatif terhadap pusat rintangan
            r_drone = p_drone - obs_center
            d_curr = float(np.linalg.norm(r_drone))
            theta_now = math.atan2(r_drone[1], r_drone[0])

            # Total panjang busur orbit dan busur yang telah diselesaikan (robust tanpa modulo wrap bug)
            d_exit_angle = math.atan2(math.sin(theta_exit - theta_entry), math.cos(theta_exit - theta_entry)) * sigma_rot
            total_arc = d_exit_angle if d_exit_angle > 0.0 else (d_exit_angle + 2.0 * math.pi)

            d_now_angle = math.atan2(math.sin(theta_now - theta_entry), math.cos(theta_now - theta_entry)) * sigma_rot
            if d_now_angle < 0.0:
                arc_done = 0.0
            else:
                arc_done = min(d_now_angle, total_arc)

            # Kondisi Keluar: Harus sudah melewati garis entry DAN (melewati garis exit ATAU arc selesai)
            dist_to_entry_line = float(np.dot(p_drone - p_entry, u_line))
            dist_to_exit_line = float(np.dot(p_drone - p_exit, u_line))
            is_past_entry = dist_to_entry_line >= 0.0

            if is_past_entry and (dist_to_exit_line >= 0.0 or arc_done >= (total_arc - 0.08)):
                agent.last_bypassed_obs_id = obs_id
                agent.bypass_state = 'none'
                agent.bypass_obs_id = None
                agent.bypass_side = 0
                speed_scale_static = 1.0
                v_avoid_static = np.zeros(2, dtype=np.float32)
                agent.ref_pos = (p_exit + self.lead_dist * u_line).astype(np.float32)
                self.get_logger().info(
                    f'  ✅ [iris_{did}] Rintangan #{obs_id} selesai dilingkari! Melanjutkan lurus di jalur sapuan.'
                )
            else:
                # 1. Carrot Position pada busur lingkaran (maju lead_dist sepanjang busur)
                d_theta_lead = min(total_arc - arc_done, self.lead_dist / R_ORBIT)
                theta_carrot = theta_now + sigma_rot * d_theta_lead
                p_carrot = obs_center + max(R_ORBIT, d_curr) * np.array([math.cos(theta_carrot), math.sin(theta_carrot)], dtype=float)
                agent.ref_pos = p_carrot.astype(np.float32)

                # 2. Kecepatan Tangensial (100% Murni Tegak Lurus terhadap Vektor Radial)
                u_tangent = sigma_rot * np.array([-math.sin(theta_now), math.cos(theta_now)], dtype=float)
                v_ff = V_NOM * u_tangent

                # 3. Sawar Radial Satu Arah (Hanya Mendorong Keluar jika < R_ORBIT, Tidak Pernah Menarik ke Dalam)
                u_radial = r_drone / max(0.01, d_curr)
                v_radial = max(0.0, 3.0 * (R_ORBIT - d_curr)) * u_radial

                v_cmd = v_ff + v_radial
                v_avoid_static = v_cmd.astype(np.float32)
                speed_scale_static = 0.0

                # 4. Target Yaw tegak lurus mengitari rintangan
                agent.target_yaw = math.atan2(u_tangent[1], u_tangent[0])
        else:
            if agent.bypass_state == 'arc_contour':
                agent.bypass_state = 'none'
                agent.bypass_obs_id = None
                agent.bypass_side = 0

        # Hard Universal Safety Repulsion Layer darurat (< 0.65m dari silinder)
        for obs in self.static_obstacles:
            obs_id, _, ox, oy, rad, _, _ = obs
            obs_center = np.array([ox, oy], dtype=float)
            d_c = float(np.linalg.norm(obs_center - p_drone))
            d_s = d_c - rad
            if d_s < 0.65:
                u_away = (p_drone - obs_center) / max(0.05, d_c)
                push_str = min(2.5, float((0.65 - d_s) * 4.5))
                v_avoid_static += (u_away * push_str).astype(np.float32)
                agent.ref_pos = (obs_center + u_away * 1.15).astype(np.float32)
                speed_scale_static = 0.0

        # ── 2. EVALUASI RINTANGAN DINAMIS (PREDIKSI HARMONIK & PROACTIVE CORRIDOR EVASION) ──
        v_avoid_dyn = np.zeros(2, dtype=np.float32)
        speed_scale_dyn = 1.0

        for k_obs, dyn_obs in enumerate(self.dynamic_obstacles):
            p_obs_ground = dyn_obs['pos']
            v_obs_ground = dyn_obs['vel']
            kf = self.kf_dyn_obs[k_obs]
            rad_dyn = 0.45

            # Gunakan posisi & kecepatan gabungan
            p_obs_est = p_obs_ground if np.linalg.norm(p_obs_ground) > 0.01 else kf.pos
            v_obs_est = v_obs_ground if np.linalg.norm(v_obs_ground) > 0.01 else kf.vel
            v_obs_speed = float(np.linalg.norm(v_obs_est))

            rel_p = p_obs_est - p_drone
            dist_curr = float(np.linalg.norm(rel_p))
            dist_surf_curr = dist_curr - rad_dyn
            min_obs_dist = min(min_obs_dist, dist_surf_curr)

            if dist_curr > 8.0:
                continue

            # Unit track arah rintangan
            if v_obs_speed > 0.10:
                u_obs_track = v_obs_est / v_obs_speed
            else:
                u_obs_track = np.array([1.0, -1.0]) / math.sqrt(2.0) if k_obs == 0 else np.array([1.0, 1.0]) / math.sqrt(2.0)

            # Vektor normal terhadap garis lintasan diagonal rintangan
            n_obs_track = np.array([-u_obs_track[1], u_obs_track[0]], dtype=float)

            # Jarak lateral dan longitudinal drone terhadap rintangan
            r_obs_drone = p_drone - p_obs_est
            dist_lateral = float(np.dot(r_obs_drone, n_obs_track))
            dist_longitudinal = float(np.dot(r_obs_drone, u_obs_track))

            ttc_pass = dist_longitudinal / v_obs_speed if v_obs_speed > 0.10 else -1.0

            # Cek apakah rintangan sedang mendekati titik drone (TTC 0..5s atau jarak dekat dan mendekat)
            is_incoming = (0.0 <= ttc_pass <= 5.0) or (dist_curr < 4.0 and float(np.dot(rel_p, v_obs_est)) < 0.0)

            CORRIDOR_SAFE = 1.80

            if is_incoming and abs(dist_lateral) < CORRIDOR_SAFE:
                # Drone berada di dalam koridor bahaya!
                speed_scale_dyn = 0.0
                agent.dyn_obs_yielding = True

                # Tentukan arah geser lateral
                sign_evade = 1.0 if dist_lateral >= 0.0 else -1.0
                u_evade = sign_evade * n_obs_track

                v_evade_mag = 1.20
                v_avoid_dyn += u_evade * v_evade_mag

                # Geser ref_pos keluar koridor
                lateral_shift = (CORRIDOR_SAFE - abs(dist_lateral) + 0.35)
                agent.ref_pos = agent.pos[:2].copy() + u_evade * lateral_shift

                self.get_logger().info(
                    f'  ⚡ [iris_{did}] DYNAMIC OBS #{k_obs+1} INCOMING (d_lat={dist_lateral:.2f}m, TTC={ttc_pass:.1f}s)! '
                    f'Fast Sidestep ({v_evade_mag} m/s)...',
                    throttle_duration_sec=0.5
                )
            elif is_incoming and abs(dist_lateral) >= CORRIDOR_SAFE:
                speed_scale_dyn = 0.0
                agent.dyn_obs_yielding = True
                agent.ref_pos = agent.pos[:2].copy()

            # Emergency Hard Repulsion jika jarak mepet < 1.20m
            if dist_surf_curr < 1.20:
                u_away = -rel_p / max(0.05, dist_curr)
                push_str = float((1.20 - dist_surf_curr) * 3.5)
                v_avoid_dyn += u_away * push_str
                agent.ref_pos = agent.pos[:2].copy() + u_away * 0.50

        # Periksa juga pembacaan sensor LiDAR fisik aktual
        if hasattr(agent, 'lidar_ranges') and len(agent.lidar_ranges) > 0:
            valid_lidar = agent.lidar_ranges[np.isfinite(agent.lidar_ranges) & (agent.lidar_ranges > 0.15)]
            if len(valid_lidar) > 0:
                min_lidar_dist = float(np.min(valid_lidar))
                if min_lidar_dist < min_obs_dist:
                    min_obs_dist = min_lidar_dist

        agent.min_dist_to_obs = min(agent.min_dist_to_obs, max(0.0, min_obs_dist))

        # Gabungkan Modulasi Kecepatan & Vektor Menghindar
        speed_scale_total = min(speed_scale_static, speed_scale_dyn)
        avoid_offset_total = v_avoid_static + v_avoid_dyn

        return avoid_offset_total, speed_scale_total

    def get_coverage_percentage(self):
        """Menghitung persentase cakupan di dalam area pemetaan aktif."""
        i_start = int((self.active_x_min - self.x_min) / self.dx)
        i_end = int((self.active_x_max - self.x_min) / self.dx)
        j_start = int((self.active_y_min - self.y_min) / self.dy)
        j_end = int((self.active_y_max - self.y_min) / self.dy)

        sub_grid = self.cov_grid[i_start:i_end, j_start:j_end]
        if self.enable_obstacles and hasattr(self, 'obstacle_mask'):
            sub_mask = self.obstacle_mask[i_start:i_end, j_start:j_end]
            valid_cells = ~sub_mask
            if np.any(valid_cells):
                return float(np.sum(sub_grid & valid_cells) / np.sum(valid_cells) * 100.0)
        return float(np.mean(sub_grid) * 100.0)

    # ── Fault Tolerance & Dynamic Failure Recovery Functions ─────────

    def kill_drone_callback(self, msg):
        """Menerima array ID drone yang dimatikan dari Terminal 2."""
        killed_ids = list(msg.data)
        newly_failed = []
        for did in killed_ids:
            if did in self.agents and self.agents[did].is_alive:
                self.agents[did].is_alive = False
                self.agents[did].state = 'dead'
                self.dead_drones.add(did)
                newly_failed.append(did)
                self.publish_twist(did, 0.0, 0.0, 0.0)
                self.get_logger().warning(f'💥 [EMERGENCY KILL] iris_{did} dimatikan! Memicu Dynamic Fault Recovery...')

        if newly_failed:
            self._handle_failure_recovery(newly_failed)

    def _capture_pending_recovery(self, did):
        """Menyelamatkan waypoint recovery yang belum sempat diselesaikan oleh drone did yang mati."""
        agent = self.agents[did]
        pending_lines = []
        if agent.row_idx < agent.num_rows:
            for r in range(agent.row_idx, agent.num_rows):
                if r < len(agent.wp_flags) and not agent.wp_flags[r]:
                    idx_s = r * 2
                    idx_e = idx_s + 1
                    if idx_e < len(agent.waypoints):
                        pending_lines.append((agent.waypoints[idx_s], agent.waypoints[idx_e]))
        return pending_lines

    def are_polygons_adjacent(self, poly1, poly2, tol=1.10):
        """Mengecek apakah dua poligon saling bersebelahan / menempel."""
        if len(poly1) < 3 or len(poly2) < 3:
            return False
        pts1 = np.array(poly1, dtype=float)
        pts2 = np.array(poly2, dtype=float)
        min_d = np.min([np.linalg.norm(p1 - p2) for p1 in pts1 for p2 in pts2])
        if min_d < tol:
            return True
        for i in range(len(pts1)):
            a = pts1[i]
            b = pts1[(i + 1) % len(pts1)]
            ab = b - a
            ab2 = float(np.dot(ab, ab))
            if ab2 < 1e-6:
                continue
            for p in pts2:
                t = max(0.0, min(1.0, float(np.dot(p - a, ab)) / ab2))
                proj = a + t * ab
                if np.linalg.norm(p - proj) < tol:
                    return True
        return False

    def _handle_failure_recovery(self, newly_dead_ids=None):
        """
        KONSOLIDASI GLOBAL & DYNAMIC FAILURE RECOVERY (NON-BACKTRACKING):
        1. Amankan orphan recovery waypoints dari drone yang baru mati.
        2. Hapus riwayat coverage HANYA di dalam sel milik drone yang BARU mati.
           (Area yang sudah disapu drone helper tetap tersimpan & tetap hijau).
        3. Klasifikasikan status drone hidup:
           - Jika drone SUDAH menyelesaikan sel aslinya (row_idx >= own_num_rows atau state done/return_to_centroid):
             Pertahankan posisinya dan JANGAN pernah suruh balik ke sel aslinya.
             Alokasikan rute baru langsung dari posisi fisik real-time (x, y) saat ini.
           - Jika drone MASIH menyelesaikan sel aslinya:
             Pertahankan sisa baris sel aslinya, rute recovery baru akan disambung di belakang.
        4. Leburkan seluruh poligon sel mati via Shapely unary_union.
        5. Generate sapuan Lawnmower horizontal murni pada area sel mati gabungan.
        6. Filter baris Lawnmower: Hanya masukkan baris yang BELUM disapu (hindari duplikasi kerja).
        7. Alokasikan blok baris utuh secara optimal ke helper terdekat dari posisi mulai efektifnya.
        """
        alive_ids = [did for did, a in self.agents.items() if a.is_alive and a.state != 'dead']
        if not alive_ids:
            self.get_logger().error('❌ [RECOVERY] Tidak ada drone hidup tersisa di kawanan!')
            return

        # 1. Kumpulkan sisa waypoint yatim dari drone yang baru mati
        rescued_lines = []
        if newly_dead_ids:
            for d in newly_dead_ids:
                rescued = self._capture_pending_recovery(d)
                if rescued:
                    rescued_lines.extend(rescued)
                    self.get_logger().info(f'  📦 [ORPHAN RESCUE] {len(rescued)} baris recovery diselamatkan dari iris_{d}.')

        # 2. Hapus riwayat coverage HANYA di dalam sel milik drone yang BARU mati
        # (JANGAN hapus area yang sudah berhasil disapu oleh drone helper sebelumnya!)
        if newly_dead_ids:
            for d in newly_dead_ids:
                poly = getattr(self.agents[d], 'raw_cell_polygon', self.agents[d].cell_polygon)
                poly_arr = np.array(poly, dtype=float)
                if len(poly_arr) < 3:
                    continue
                if np.linalg.norm(poly_arr[0] - poly_arr[-1]) > 1e-4:
                    poly_arr = np.vstack([poly_arr, poly_arr[0]])
                poly_path = MplPath(poly_arr)
                min_x_p, min_y_p = np.min(poly_arr, axis=0) - 0.2
                max_x_p, max_y_p = np.max(poly_arr, axis=0) + 0.2
                i_min_p = max(0, int((min_x_p - self.x_min) / self.dx))
                i_max_p = min(self.grid_n, int((max_x_p - self.x_min) / self.dx) + 1)
                j_min_p = max(0, int((min_y_p - self.y_min) / self.dy))
                j_max_p = min(self.grid_n, int((max_y_p - self.y_min) / self.dy) + 1)
                for i in range(i_min_p, i_max_p):
                    for j in range(j_min_p, j_max_p):
                        cell_x = self.x_min + (i + 0.5) * self.dx
                        cell_y = self.y_min + (j + 0.5) * self.dy
                        if poly_path.contains_point([cell_x, cell_y]):
                            self.cov_grid[i, j] = False

        # 3. Klasifikasikan status masing-masing drone hidup
        for h in alive_ids:
            agent = self.agents[h]
            own_num = getattr(agent, 'own_num_rows', len(agent.waypoints) // 2)
            has_finished_own = (agent.row_idx >= own_num) or (agent.state in ('done', 'return_to_centroid'))

            if has_finished_own:
                # Drone sudah menyelesaikan sel aslinya: kosongkan antrean lama, siap terima rute baru
                agent.waypoints = []
                agent.wp_flags = []
                agent.num_rows = 0
                agent.row_idx = -1
            else:
                # Drone masih menyapu sel aslinya: pertahankan sisa baris aslinya
                own_wps = []
                own_flags = []
                for r in range(min(own_num, agent.num_rows)):
                    if r < len(agent.wp_flags) and agent.wp_flags[r]:
                        idx_s = r * 2
                        idx_e = idx_s + 1
                        if idx_e < len(agent.waypoints):
                            own_wps.extend([agent.waypoints[idx_s], agent.waypoints[idx_e]])
                            own_flags.append(True)
                agent.waypoints = own_wps
                agent.wp_flags = own_flags
                agent.num_rows = len(own_wps) // 2
                if agent.row_idx >= agent.num_rows:
                    agent.row_idx = max(0, agent.num_rows - 1)

        # 4. Kumpulkan poligon sel Voronoi seluruh drone mati dan kosongkan waypoints-nya
        dead_drones = [did for did, a in self.agents.items() if not a.is_alive or a.state == 'dead']
        all_dead_polys = [getattr(self.agents[d], 'raw_cell_polygon', self.agents[d].cell_polygon) for d in dead_drones if len(self.agents[d].cell_polygon) >= 3]
        for d in dead_drones:
            self.agents[d].waypoints = []
            self.agents[d].wp_flags = []
            self.agents[d].num_rows = 0
            self.agents[d].row_idx = 0

        if not all_dead_polys:
            return

        # 5. Peleburan poligon via Shapely unary_union (Mendukung Polygon dan MultiPolygon)
        try:
            sp_polys = [SpPolygon(p).buffer(0.08) for p in all_dead_polys]
            merged = unary_union(sp_polys).buffer(-0.08)
            if merged.geom_type == 'Polygon':
                comp_polys = [np.array(merged.exterior.coords, dtype=float)]
            elif merged.geom_type == 'MultiPolygon':
                comp_polys = [np.array(p.exterior.coords, dtype=float) for p in merged.geoms]
            else:
                comp_polys = all_dead_polys
        except Exception as e:
            self.get_logger().warning(f'Shapely merge fallback: {e}')
            comp_polys = all_dead_polys

        self.merged_dead_comp_polys = comp_polys

        # 6. Generate Lawnmower lines horizontal murni untuk setiap komponen sel mati
        raw_recovery_lines = []
        for comp in comp_polys:
            boust_wps = generate_boustrophedon(comp, sweep_spacing=1.45, margin=0.20, start_from_top=False, obstacles=self.static_obstacles)
            for k in range(0, len(boust_wps) - 1, 2):
                raw_recovery_lines.append((boust_wps[k], boust_wps[k + 1]))

        # Filter baris: Buang baris yang sudah terpetakan di cov_grid (>= 80% coverage)
        all_recovery_lines = []
        for p1, p2 in raw_recovery_lines:
            n_samples = 10
            covered_samples = 0
            for s in range(n_samples):
                frac = s / (n_samples - 1) if n_samples > 1 else 0.5
                p_samp = p1 + frac * (p2 - p1)
                gx = int((p_samp[0] - self.x_min) / self.dx)
                gy = int((p_samp[1] - self.y_min) / self.dy)
                if 0 <= gx < self.grid_n and 0 <= gy < self.grid_n:
                    if self.cov_grid[gx, gy]:
                        covered_samples += 1
            if covered_samples / n_samples < 0.80:
                all_recovery_lines.append((p1, p2))

        if not all_recovery_lines:
            self.get_logger().info('✅ [RECOVERY] Seluruh baris sel mati telah tercover sebelumnya! Tidak perlu replanning.')
            self.recovery_active = False
            return

        self.get_logger().info(
            f'📐 [DYNAMIC RECOVERY] Terbentuk {len(all_recovery_lines)} baris Lawnmower recovery murni '
            f'dari {len(dead_drones)} drone gugur untuk {len(alive_ids)} drone aktif tersisa.'
        )

        # 7. Pemilihan Drone Helper Paling Efisien (Tetangga Bersebelahan + Jarak Terdekat)
        def get_helper_start_pos(hid):
            ag = self.agents[hid]
            own_num = getattr(ag, 'own_num_rows', 0)
            has_finished_own = (ag.row_idx >= own_num) or (ag.state in ('done', 'return_to_centroid')) or (ag.num_rows == 0)
            if has_finished_own:
                return ag.pos[:2].copy()
            else:
                last_idx = len(ag.waypoints) - 1
                return ag.waypoints[last_idx] if last_idx >= 0 else ag.pos[:2].copy()

        centroid_global = np.mean([pt for line in all_recovery_lines for pt in line], axis=0)

        num_dead = len(dead_drones)
        num_alive = len(alive_ids)
        if num_dead == 1:
            n_needed = 2 if len(all_recovery_lines) > 5 else 1
            n_h = min(n_needed, num_alive)
        elif num_dead == 2:
            n_h = min(3, num_alive)
        else:
            n_h = num_alive

        # Cari tetangga bersebelahan
        adj_helpers = set()
        for d_poly in all_dead_polys:
            for h in alive_ids:
                h_poly = getattr(self.agents[h], 'raw_cell_polygon', self.agents[h].cell_polygon)
                if self.are_polygons_adjacent(d_poly, h_poly):
                    adj_helpers.add(h)

        candidates = list(adj_helpers) if adj_helpers else alive_ids
        dists = [(h, float(np.linalg.norm(get_helper_start_pos(h) - centroid_global))) for h in candidates]
        dists.sort(key=lambda x: x[1])
        selected_helpers = [h for h, _ in dists[:n_h]]

        if len(selected_helpers) < n_h:
            rem = [h for h in alive_ids if h not in selected_helpers]
            rem.sort(key=lambda h: float(np.linalg.norm(get_helper_start_pos(h) - centroid_global)))
            selected_helpers.extend(rem[:n_h - len(selected_helpers)])

        self.get_logger().info(
            f'  👥 [HELPER ALLOCATION] Terpilih {len(selected_helpers)} drone helper: '
            f'{[f"iris_{h}" for h in selected_helpers]} untuk menangani {len(all_recovery_lines)} baris recovery.'
        )

        # 8. Alokasi Blok Baris Utuh & Penyelarasan Paralel Spasial (Zero Path Crossing)
        block_sz = math.ceil(len(all_recovery_lines) / len(selected_helpers))
        blocks = [all_recovery_lines[k * block_sz : (k + 1) * block_sz] for k in range(len(selected_helpers)) if all_recovery_lines[k * block_sz : (k + 1) * block_sz]]

        block_centers = [np.mean([pt for line in b for pt in line], axis=0) for b in blocks]
        variances = np.var(block_centers, axis=0) if len(block_centers) > 1 else [0, 1]
        axis_idx = 1 if len(variances) > 1 and variances[1] >= variances[0] else 0

        sorted_blocks = [b for _, b in sorted(zip([c[axis_idx] for c in block_centers], blocks))]
        sorted_helpers = sorted(selected_helpers, key=lambda h: get_helper_start_pos(h)[axis_idx])

        # 9. Perekatan Jalur Recovery ke Masing-Masing Helper
        assigned_helpers = set()
        for h, b in zip(sorted_helpers, sorted_blocks):
            if not b:
                continue
            assigned_helpers.add(h)
            h_start = get_helper_start_pos(h)
            first_line = b[0]
            last_line = b[-1]
            d_first = min(np.linalg.norm(h_start - first_line[0]), np.linalg.norm(h_start - first_line[1]))
            d_last = min(np.linalg.norm(h_start - last_line[0]), np.linalg.norm(h_start - last_line[1]))

            lines_in_order = list(reversed(b)) if d_last < d_first else list(b)
            entry_line = lines_in_order[0]
            p_left = entry_line[0] if entry_line[0][0] <= entry_line[1][0] else entry_line[1]
            p_right = entry_line[1] if entry_line[0][0] <= entry_line[1][0] else entry_line[0]
            start_from_left = np.linalg.norm(h_start - p_left) <= np.linalg.norm(h_start - p_right)

            rec_wps_h = []
            go_left_to_right = start_from_left
            for l_start, l_end in lines_in_order:
                pa = l_start if l_start[0] <= l_end[0] else l_end
                pb = l_end if l_start[0] <= l_end[0] else l_start
                if go_left_to_right:
                    rec_wps_h.extend([pa, pb])
                else:
                    rec_wps_h.extend([pb, pa])
                go_left_to_right = not go_left_to_right

            ag = self.agents[h]
            num_new_rows = len(rec_wps_h) // 2

            own_num = getattr(ag, 'own_num_rows', 0)
            has_finished_own = (ag.row_idx >= own_num) or (ag.state in ('done', 'return_to_centroid')) or (ag.num_rows == 0)

            if has_finished_own:
                # Drone SUDAH selesai sel aslinya -> LANGSUNG ganti ke waypoints recovery murni
                ag.waypoints = rec_wps_h
                ag.wp_flags = [False] * num_new_rows
                ag.num_rows = num_new_rows
                ag.row_idx = -1  # Target baris berikutnya adalah baris 0
                ag.state = 'transit_to_recovery'
                self.publish_twist(h, 0.0, 0.0, 0.0)
                self.get_logger().info(
                    f'  🚀 [iris_{h}] LANGSUNG MENUJU BLOK RECOVERY BARU dari posisi real-time ({ag.pos[0]:.2f}, {ag.pos[1]:.2f}) '
                    f'-> ({rec_wps_h[0][0]:.2f}, {rec_wps_h[0][1]:.2f})! (Total: {num_new_rows} Baris)'
                )
            else:
                # Drone MASIH menyapu sel aslinya -> sambung di belakang antrean
                ag.waypoints.extend(rec_wps_h)
                ag.wp_flags.extend([False] * num_new_rows)
                ag.num_rows = len(ag.waypoints) // 2
                self.get_logger().info(
                    f'  🎯 [iris_{h}] Mendapat tambahan {num_new_rows} baris recovery di antrean '
                    f'(Total Baris: {ag.row_idx+1}/{ag.num_rows})'
                )

        # Untuk drone yang sudah selesai sel aslinya tetapi TIDAK dapat alokasi recovery baru
        for h in alive_ids:
            if h not in assigned_helpers:
                ag = self.agents[h]
                if ag.num_rows == 0 and ag.state not in ('done', 'return_to_centroid'):
                    ag.state = 'return_to_centroid'
                    self.get_logger().info(f'  🏁 [iris_{h}] Tidak ada sisa tugas recovery, kembali ke centroid.')

        # Perbarui rintangan statis yang relevan untuk setiap drone hidup (mencakup rintangan di sel recovery)
        for did in alive_ids:
            ag = self.agents[did]
            relevant_obs = []
            for obs in self.static_obstacles:
                obs_c = np.array([obs[2], obs[3]], dtype=float)
                for wp in ag.waypoints:
                    if np.linalg.norm(obs_c - wp) < 3.5:
                        if obs not in relevant_obs:
                            relevant_obs.append(obs)
                        break
            ag.my_static_obstacles = relevant_obs

        self.recovery_active = True
        self.mission_completed = False

    # ── State Machine & Loop Kontrol Utama (20 Hz) ───────────────────

    def control_loop(self):
        self.step_count += 1

        if not all(a.odom_received for a in self.agents.values()):
            return

        now_time = self.get_clock().now()
        t_sim = now_time.nanoseconds * 1e-9
        self.update_dynamic_obstacles(t_sim)

        # ── Watchdog Odom Timeout (> 5.0s) & Crash Detector (Z < 0.35m selama >= 10 ticks) ────
        auto_failed = []
        if self.voronoi_planned:
            for did, agent in self.agents.items():
                if agent.is_alive and agent.state != 'dead':
                    # Deteksi crash/jatuh ke tanah (Z < 0.35m terkonfirmasi selama 10 ticks = 0.5s)
                    if agent.odom_received and agent.state != 'wait_takeoff':
                        if agent.pos[2] < 0.35:
                            agent.crash_ticks = getattr(agent, 'crash_ticks', 0) + 1
                        else:
                            agent.crash_ticks = 0

                        if agent.crash_ticks >= 10:
                            agent.is_alive = False
                            agent.state = 'dead'
                            self.dead_drones.add(did)
                            auto_failed.append(did)
                            self.get_logger().error(f'🚨 [WATCHDOG CRASH] iris_{did} terdeteksi jatuh di tanah (Z={agent.pos[2]:.2f}m)!')

                    # Deteksi kehilangan heartbeat odom (> 5.0s)
                    if agent.is_alive and agent.last_odom_time is not None:
                        dt_odom = (now_time - agent.last_odom_time).nanoseconds * 1e-9
                        if dt_odom > 5.0 and agent.state != 'wait_takeoff':
                            agent.is_alive = False
                            agent.state = 'dead'
                            self.dead_drones.add(did)
                            auto_failed.append(did)
                            self.get_logger().error(f'🚨 [WATCHDOG TIMEOUT] iris_{did} kehilangan heartbeat odom ({dt_odom:.1f}s)!')

                    # Deteksi tabrakan fisik dengan obstacle (Z >= 0.50m)
                    if agent.is_alive and self.enable_obstacles and agent.pos[2] >= 0.50:
                        p_d = agent.pos[:2]
                        # 1. Cek rintangan statis (rad=0.40m + body=0.22m = 0.62m)
                        for obs in self.static_obstacles:
                            obs_id, cell_did, ox, oy, rad, _, _ = obs
                            d_center = float(np.linalg.norm(np.array([ox, oy]) - p_d))
                            if d_center < (rad + 0.22):
                                agent.is_alive = False
                                agent.state = 'dead'
                                self.dead_drones.add(did)
                                auto_failed.append(did)
                                self.get_logger().error(
                                    f'🚨 [OBSTACLE CRASH] iris_{did} MENABRAK Rintangan Statis #{obs_id}! '
                                    f'(d_center={d_center:.2f}m < {rad+0.22:.2f}m, Posisi: {p_d[0]:.2f}, {p_d[1]:.2f})'
                                )
                                self.publish_twist(did, 0.0, 0.0, 0.0)
                                break
                        # 2. Cek rintangan dinamis (rad=0.45m + body=0.22m = 0.67m)
                        if agent.is_alive:
                            for k_obs, dyn_obs in enumerate(self.dynamic_obstacles):
                                p_o = dyn_obs['pos']
                                d_center = float(np.linalg.norm(p_o - p_d))
                                if d_center < (0.45 + 0.22):
                                    agent.is_alive = False
                                    agent.state = 'dead'
                                    self.dead_drones.add(did)
                                    auto_failed.append(did)
                                    self.get_logger().error(
                                        f'🚨 [OBSTACLE CRASH] iris_{did} MENABRAK Rintangan Dinamis #{k_obs+1}! '
                                        f'(d_center={d_center:.2f}m < {0.45+0.22:.2f}m, Posisi: {p_d[0]:.2f}, {p_d[1]:.2f})'
                                    )
                                    self.publish_twist(did, 0.0, 0.0, 0.0)
                                    break

        if auto_failed:
            self._handle_failure_recovery(auto_failed)

        # Perbarui jarak pisah antar-drone (hanya saat terbang Z >= 0.80m dan masih hidup)
        for i in range(1, 8):
            for j in range(i + 1, 8):
                if self.agents[i].is_alive and self.agents[j].is_alive:
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
                    agent.state = 'pivot_to_transit'
                    agent.delay_timer = 0
            return

        self.update_coverage()

        # Eksekusi state machine per drone aktif
        for did, agent in self.agents.items():
            if not agent.is_alive or agent.state == 'dead':
                continue

            # ─────────────────────────────────────────────────────────
            # 1a. PIVOT TO TRANSIT (Pivot In-Place Menghadap Titik Awal Sel)
            # ─────────────────────────────────────────────────────────
            if agent.state == 'pivot_to_transit':
                start_wp = agent.waypoints[0]
                dx = float(start_wp[0]) - float(agent.pos[0])
                dy = float(start_wp[1]) - float(agent.pos[1])
                target_yaw = math.atan2(dy, dx)
                agent.target_yaw = target_yaw

                v_obs_avoid, speed_scale = self.compute_obstacle_avoidance_offset(did, agent, np.array([0.0, 0.0]))
                if np.linalg.norm(v_obs_avoid) > 0.05:
                    self.send_world_twist(did, float(v_obs_avoid[0]), float(v_obs_avoid[1]), target_yaw)
                else:
                    agent.ref_pos = agent.pos[:2].copy()
                    wz_cmd = self.compute_wz(agent.yaw, target_yaw)
                    self.publish_twist(did, 0.0, 0.0, wz_cmd)
                    agent.delay_timer = getattr(agent, 'delay_timer', 0) + 1

                    yaw_diff = abs(math.atan2(math.sin(target_yaw - agent.yaw), math.cos(target_yaw - agent.yaw)))
                    if (agent.delay_timer >= 20 and yaw_diff < math.radians(4.0)) or agent.delay_timer >= 80:
                        agent.state = 'transit_to_start'
                        agent.delay_timer = 0
                        self.publish_twist(did, 0.0, 0.0, 0.0)
                        self.get_logger().info(f'  🚀 [iris_{did}] Hadap titik start ({math.degrees(target_yaw):.1f}°)! Terbang ke sel...')

            # ─────────────────────────────────────────────────────────
            # 1b. TRANSIT TO START (Fase Menuju Titik Awal Sel Melalui Koridor Aman)
            # ─────────────────────────────────────────────────────────
            elif agent.state == 'transit_to_start':
                if not hasattr(agent, 'transit_wp_idx') or not getattr(agent, 'transit_waypoints', []):
                    agent.transit_waypoints = [agent.waypoints[0].copy()]
                    agent.transit_wp_idx = 0

                curr_target_wp = agent.transit_waypoints[agent.transit_wp_idx]
                dx = curr_target_wp[0] - float(agent.pos[0])
                dy = curr_target_wp[1] - float(agent.pos[1])
                dist_to_wp = math.hypot(dx, dy)
                angle_to_wp = math.atan2(dy, dx)

                is_final_transit_wp = (agent.transit_wp_idx >= len(agent.transit_waypoints) - 1)
                thresh = 0.35 if is_final_transit_wp else 0.70

                if dist_to_wp < thresh:
                    if is_final_transit_wp:
                        self.get_logger().info(f'  🎯 [iris_{did}] Tiba di Titik Start Sel ({agent.pos[0]:.2f}, {agent.pos[1]:.2f}) | Menunggu kawanan...')
                        agent.state = 'wait_all_start'
                        agent.delay_timer = 0
                        self.publish_twist(did, 0.0, 0.0, 0.0)
                        continue
                    else:
                        agent.transit_wp_idx += 1
                        curr_target_wp = agent.transit_waypoints[agent.transit_wp_idx]
                        dx = curr_target_wp[0] - float(agent.pos[0])
                        dy = curr_target_wp[1] - float(agent.pos[1])
                        dist_to_wp = math.hypot(dx, dy)
                        angle_to_wp = math.atan2(dy, dx)

                lead_t = min(dist_to_wp, self.lead_dist)
                agent.ref_pos = np.array([
                    float(agent.pos[0]) + lead_t * math.cos(angle_to_wp),
                    float(agent.pos[1]) + lead_t * math.sin(angle_to_wp)
                ], dtype=np.float32)

                v_mag = min(self.transit_speed, max(0.40, 2.0 * dist_to_wp))
                v_world_x = v_mag * math.cos(angle_to_wp) + np.clip(1.2 * dx, -0.40, 0.40)
                v_world_y = v_mag * math.sin(angle_to_wp) + np.clip(1.2 * dy, -0.40, 0.40)

                # Full V2V avoidance during transit
                v_world_x, v_world_y = self.apply_v2v_repulsion(did, v_world_x, v_world_y, is_transit=True)

                # Dynamic Harmonic Obstacle Avoidance (Hanya jika berpapasan dengan rintangan dinamis)
                v_obs_avoid, speed_scale = self.compute_obstacle_avoidance_offset(did, agent, np.array([v_world_x, v_world_y]))
                if speed_scale <= 0.01:
                    v_world_x = float(v_obs_avoid[0])
                    v_world_y = float(v_obs_avoid[1])
                else:
                    v_world_x = v_world_x * speed_scale + float(v_obs_avoid[0])
                    v_world_y = v_world_y * speed_scale + float(v_obs_avoid[1])

                self.send_world_twist(did, v_world_x, v_world_y, angle_to_wp)

            # ─────────────────────────────────────────────────────────
            # 1c. WAIT ALL START (Sinkronisasi Start Bersama Antar Kawanan)
            # ─────────────────────────────────────────────────────────
            elif agent.state == 'wait_all_start':
                start_wp = np.array(agent.waypoints[0], dtype=np.float32)
                v_x, v_y = self.apply_v2v_repulsion(did, 0.0, 0.0, is_transit=True)
                v_obs_avoid, speed_scale = self.compute_obstacle_avoidance_offset(did, agent, np.array([v_x, v_y]))
                if speed_scale <= 0.01:
                    v_x = float(v_obs_avoid[0])
                    v_y = float(v_obs_avoid[1])
                else:
                    agent.ref_pos = start_wp.copy()
                    if np.linalg.norm(v_obs_avoid) > 0.01:
                        v_x += float(v_obs_avoid[0])
                        v_y += float(v_obs_avoid[1])
                self.send_world_twist(did, v_x, v_y, agent.yaw)

                alive_agents = [a for a in self.agents.values() if a.is_alive and a.state != 'dead']
                all_arrived = len(alive_agents) > 0 and all(a.state in ('wait_all_start', 'align_start_yaw', 'sweeping_row', 'delay_at_corner_end', 'stepping_vertical', 'delay_at_new_row', 'transit_to_recovery', 'done') for a in alive_agents)
                if all_arrived:
                    agent.state = 'align_start_yaw'
                    agent.delay_timer = 0
                    self.publish_twist(did, 0.0, 0.0, 0.0)
                    self.get_logger().info(f'  🏁 [iris_{did}] Semua Drone Siap di Sel Masing-masing! Menyelaraskan heading baris 1...')

            # ─────────────────────────────────────────────────────────
            # 1d. TRANSIT TO RECOVERY (Fase Menuju Titik Awal Blok Recovery)
            # ─────────────────────────────────────────────────────────
            elif agent.state == 'transit_to_recovery':
                next_start = agent.waypoints[(agent.row_idx + 1) * 2]
                dx = float(next_start[0]) - float(agent.pos[0])
                dy = float(next_start[1]) - float(agent.pos[1])
                dist_to_start = math.hypot(dx, dy)
                angle_to_start = math.atan2(dy, dx)

                lead_t = min(dist_to_start, self.lead_dist)
                agent.ref_pos = np.array([
                    float(agent.pos[0]) + lead_t * math.cos(angle_to_start),
                    float(agent.pos[1]) + lead_t * math.sin(angle_to_start)
                ], dtype=np.float32)

                if dist_to_start < 0.28:
                    agent.state = 'delay_at_new_row'
                    agent.delay_timer = 0
                    self.publish_twist(did, 0.0, 0.0, 0.0)
                    self.get_logger().info(f'  🎯 [iris_{did}] Tiba di Titik Start Blok Recovery ({next_start[0]:.2f}, {next_start[1]:.2f})!')
                    continue

                v_mag = min(self.transit_speed, max(0.40, 2.0 * dist_to_start))
                v_world_x = v_mag * math.cos(angle_to_start) + np.clip(1.2 * dx, -0.40, 0.40)
                v_world_y = v_mag * math.sin(angle_to_start) + np.clip(1.2 * dy, -0.40, 0.40)

                # V2V avoidance aktif selama perjalanan melintasi arena
                v_world_x, v_world_y = self.apply_v2v_repulsion(did, v_world_x, v_world_y, is_transit=True)

                # Obstacle Avoidance Geodesic Arc & Dynamic Harmonic Avoidance
                v_obs_avoid, speed_scale = self.compute_obstacle_avoidance_offset(did, agent, np.array([v_world_x, v_world_y]))
                if speed_scale <= 0.01:
                    v_world_x = float(v_obs_avoid[0])
                    v_world_y = float(v_obs_avoid[1])
                else:
                    v_world_x = v_world_x * speed_scale + float(v_obs_avoid[0])
                    v_world_y = v_world_y * speed_scale + float(v_obs_avoid[1])

                self.send_world_twist(did, v_world_x, v_world_y, angle_to_start)

            # ─────────────────────────────────────────────────────────
            # 2. ALIGN START YAW (Pivot In-Place ke Baris 1 - Sama Seperti Corner)
            # ─────────────────────────────────────────────────────────
            elif agent.state == 'align_start_yaw':
                wp_start = agent.waypoints[0]
                wp_end = agent.waypoints[1]
                line_dir = wp_end - wp_start
                target_yaw = math.atan2(line_dir[1], line_dir[0])
                agent.target_yaw = target_yaw

                v_obs_avoid, speed_scale = self.compute_obstacle_avoidance_offset(did, agent, np.array([0.0, 0.0]))
                if np.linalg.norm(v_obs_avoid) > 0.05:
                    self.send_world_twist(did, float(v_obs_avoid[0]), float(v_obs_avoid[1]), target_yaw)
                else:
                    agent.ref_pos = wp_start.copy()
                    wz_cmd = self.compute_wz(agent.yaw, target_yaw)
                    self.publish_twist(did, 0.0, 0.0, wz_cmd)
                    agent.delay_timer = getattr(agent, 'delay_timer', 0) + 1

                    yaw_diff = abs(math.atan2(math.sin(target_yaw - agent.yaw), math.cos(target_yaw - agent.yaw)))
                    if (agent.delay_timer >= 25 and yaw_diff < math.radians(4.0)) or agent.delay_timer >= 100:
                        agent.state = 'sweeping_row'
                        agent.row_idx = 0
                        agent.delay_timer = 0
                        self.publish_twist(did, 0.0, 0.0, 0.0)
                        self.get_logger().info(f'  🚀 [iris_{did}] Heading Baris 1 Terkunci ({math.degrees(target_yaw):.1f}°)! Memulai Baris 1/{agent.num_rows}')

            # ─────────────────────────────────────────────────────────
            # 3. SWEEPING ROW (Critically Damped Tracking & Zero Overshoot)
            # ─────────────────────────────────────────────────────────
            elif agent.state == 'sweeping_row':
                idx_start = agent.row_idx * 2
                idx_end = idx_start + 1

                if idx_end >= len(agent.waypoints):
                    agent.state = 'return_to_centroid'
                    self.get_logger().info(f'🎉 [iris_{did}] SELURUH TUGAS TUNTAS! Kembali ke pusat sel ({agent.centroid[0]:.2f}, {agent.centroid[1]:.2f}).')
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

                # Obstacle Avoidance Geodesic Arc & Dynamic Harmonic Avoidance
                v_obs_avoid, obs_speed_scale = self.compute_obstacle_avoidance_offset(did, agent, u_line)

                # DYNAMIC FEEDBACK-COUPLED MOVING REFERENCE (Hanya diperbarui sepanjang garis jika tidak terhalang / tidak sedang orbit)
                if obs_speed_scale > 0.01 and agent.bypass_state != 'arc_contour':
                    s_target = min(line_len, actual_prog + self.lead_dist * obs_speed_scale)
                    agent.ref_pos = (wp_start + s_target * u_line)

                # Catat telemetri tracking
                agent.max_cross_track_err = max(agent.max_cross_track_err, abs(e_lat))
                agent.cross_track_errors.append(abs(e_lat))
                agent.altitude_errors.append(abs(float(agent.pos[2]) - self.cruise_alt))
                yaw_diff = abs(math.atan2(math.sin(agent.yaw - agent.target_yaw), math.cos(agent.yaw - agent.target_yaw)))
                agent.yaw_errors.append(math.degrees(yaw_diff))

                dist_to_end = line_len - actual_prog
                dist_to_end_pt = float(np.linalg.norm(agent.pos[:2] - wp_end))

                # Cek ketercapaian ujung baris (Snapping & Hard Stop 0.0% Overshoot)
                # Syarat: harus sudah melewati 40% panjang baris agar tidak salah picu di titik awal
                if actual_prog > 0.40 * line_len and (dist_to_end <= 0.25 or dist_to_end_pt < 0.26):
                    overshoot_dist = max(0.0, float(np.dot(pos_rel, u_line)) - line_len)
                    overshoot_pct = (overshoot_dist / line_len) * 100.0 if line_len > 0 else 0.0
                    agent.overshoot_list.append(overshoot_pct)
                    agent.max_overshoot = max(agent.max_overshoot, overshoot_pct)

                    is_rec_row = (agent.row_idx < len(agent.wp_flags)) and (not agent.wp_flags[agent.row_idx])
                    tag_type = "RECOVERY" if is_rec_row else "SEL ASLI"

                    self.get_logger().info(
                        f'  -> [iris_{did}] Ujung Baris {agent.row_idx+1}/{agent.num_rows} [{tag_type}] '
                        f'tercapai ({agent.pos[0]:.2f}, {agent.pos[1]:.2f}) | Overshoot: {overshoot_pct:.2f}% ({overshoot_dist:.2f}m)'
                    )
                    self.publish_twist(did, 0.0, 0.0, 0.0)
                    agent.bypass_state = 'none'
                    agent.bypass_obs_id = None
                    agent.bypass_side = 0

                    if agent.row_idx + 1 >= agent.num_rows:
                        agent.state = 'return_to_centroid'
                        self.get_logger().info(f'🎉 [iris_{did}] SELURUH TUGAS TUNTAS! Kembali ke pusat sel ({agent.centroid[0]:.2f}, {agent.centroid[1]:.2f}).')
                    else:
                        next_is_rec = (agent.row_idx + 1 < len(agent.wp_flags)) and (not agent.wp_flags[agent.row_idx + 1])
                        curr_is_own = (agent.row_idx < len(agent.wp_flags)) and agent.wp_flags[agent.row_idx]

                        if curr_is_own and next_is_rec:
                            agent.state = 'transit_to_recovery'
                            self.get_logger().info(f'🔄 [iris_{did}] Sel sendiri tuntas! Memulai transit ke Blok Recovery...')
                        else:
                            agent.state = 'delay_at_corner_end'
                            agent.delay_timer = 0
                    continue

                # CRITICALLY DAMPED FEEDFORWARD RAMP-DOWN pada 0.80m terakhir & Obstacle Braking
                if dist_to_end > 0.80:
                    ff_scale = 1.0
                elif dist_to_end > 0.10:
                    ff_scale = dist_to_end / 0.80
                else:
                    ff_scale = 0.0

                # MAPPING ISOLATION: Maju sepanjang garis sapuan dikalikan obs_speed_scale (0.0 jika terhalang)
                v_ff = (self.nominal_speed * ff_scale * obs_speed_scale) * u_line

                # Feedback controller: Koreksi lateral orthogonal (Cross-track) yang disesuaikan saat menghindar
                v_corr_lat = -np.clip(self.kp_track * e_lat, -0.45, 0.45) * np.array([-u_line[1], u_line[0]]) * obs_speed_scale

                v_world = v_ff + v_corr_lat + v_obs_avoid
                v_world_x = float(v_world[0])
                v_world_y = float(v_world[1])

                self.send_world_twist(did, v_world_x, v_world_y, agent.yaw)

            elif agent.state == 'delay_at_corner_end':
                v_obs_avoid, speed_scale = self.compute_obstacle_avoidance_offset(did, agent, np.array([0.0, 0.0]))
                if np.linalg.norm(v_obs_avoid) > 0.05:
                    self.send_world_twist(did, float(v_obs_avoid[0]), float(v_obs_avoid[1]), agent.target_yaw)
                else:
                    # Cek jika ada drone tetangga di perbatasan yang sedang belok / dekat (< 1.25m)
                    has_yield_conflict = False
                    for other_id, other_agent in self.agents.items():
                        if other_id != did and other_agent.is_alive and other_agent.odom_received and other_agent.pos[2] >= 0.80:
                            dist_neighbor = float(np.linalg.norm(agent.pos[:2] - other_agent.pos[:2]))
                            if dist_neighbor < 1.25:
                                if other_agent.state in ('stepping_vertical', 'sweeping_row') or (other_agent.state in ('delay_at_corner_end', 'delay_at_new_row') and did > other_id):
                                    has_yield_conflict = True
                                    break

                    next_start = agent.waypoints[(agent.row_idx + 1) * 2]
                    step_vec = next_start - agent.pos[:2]
                    target_yaw = math.atan2(step_vec[1], step_vec[0])
                    agent.target_yaw = target_yaw
                    agent.ref_pos = agent.pos[:2].copy()

                    wz_cmd = self.compute_wz(agent.yaw, target_yaw)
                    self.publish_twist(did, 0.0, 0.0, wz_cmd)
                    agent.delay_timer = getattr(agent, 'delay_timer', 0) + 1

                    yaw_diff = abs(math.atan2(math.sin(target_yaw - agent.yaw), math.cos(target_yaw - agent.yaw)))
                    if not has_yield_conflict and ((agent.delay_timer >= 25 and yaw_diff < math.radians(6.0)) or agent.delay_timer >= 120):
                        agent.state = 'stepping_vertical'
                        agent.step_timer = 0
                        agent.delay_timer = 0
                        agent.last_bypassed_obs_id = None

            # ─────────────────────────────────────────────────────────
            # 5. STEPPING VERTICAL (Melangkah Lurus Maju ke Baris Baru)
            # ─────────────────────────────────────────────────────────
            elif agent.state == 'stepping_vertical':
                min_step_dist = float('inf')
                for other_id, other_agent in self.agents.items():
                    if other_id != did and other_agent.is_alive and other_agent.odom_received and other_agent.pos[2] >= 0.80:
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

                # Dynamic step timeout based on distance (min 6s for short steps, proportional for long transit steps)
                max_step_ticks = max(120, int((dist_to_next / 0.50) * 20) + 60)

                # Snapping ketercapaian langkah atau timeout watchdog
                if dist_to_next < 0.32 or agent.step_timer >= max_step_ticks:
                    agent.step_timer = 0
                    self.publish_twist(did, 0.0, 0.0, 0.0)
                    agent.state = 'delay_at_new_row'
                    agent.delay_timer = 0
                    continue

                is_long_transit = dist_to_next > 2.0
                v_step_max = self.transit_speed if is_long_transit else (self.step_speed if min_step_dist > 1.20 else 0.40)
                v_step = min(v_step_max, max(0.30, 1.8 * dist_to_next))
                v_world_x = v_step * math.cos(angle_step) + np.clip(1.2 * dx, -0.35, 0.35)
                v_world_y = v_step * math.sin(angle_step) + np.clip(1.2 * dy, -0.35, 0.35)

                # V2V aktif saat transit jarak jauh lintas sel
                if is_long_transit:
                    v_world_x, v_world_y = self.apply_v2v_repulsion(did, v_world_x, v_world_y, is_transit=True)

                # Obstacle Avoidance Geodesic Arc & Dynamic Harmonic Avoidance
                v_obs_avoid, speed_scale = self.compute_obstacle_avoidance_offset(did, agent, np.array([v_world_x, v_world_y]))
                if speed_scale <= 0.01:
                    v_world_x = float(v_obs_avoid[0])
                    v_world_y = float(v_obs_avoid[1])
                else:
                    v_world_x = v_world_x * speed_scale + float(v_obs_avoid[0])
                    v_world_y = v_world_y * speed_scale + float(v_obs_avoid[1])

                self.send_world_twist(did, v_world_x, v_world_y, agent.yaw)

            # ─────────────────────────────────────────────────────────
            # 6. DELAY AT NEW ROW (Stationary In-Place Pivot ke Baris Baru)
            # ─────────────────────────────────────────────────────────
            elif agent.state == 'delay_at_new_row':
                v_obs_avoid, speed_scale = self.compute_obstacle_avoidance_offset(did, agent, np.array([0.0, 0.0]))
                if np.linalg.norm(v_obs_avoid) > 0.05:
                    self.send_world_twist(did, float(v_obs_avoid[0]), float(v_obs_avoid[1]), agent.target_yaw)
                else:
                    next_idx_start = (agent.row_idx + 1) * 2
                    next_idx_end = next_idx_start + 1
                    wp_start = agent.waypoints[next_idx_start]
                    wp_end = agent.waypoints[next_idx_end]
                    line_dir = wp_end - wp_start
                    target_yaw = math.atan2(line_dir[1], line_dir[0])
                    agent.target_yaw = target_yaw
                    agent.ref_pos = wp_start.copy()

                    wz_cmd = self.compute_wz(agent.yaw, target_yaw)
                    self.publish_twist(did, 0.0, 0.0, wz_cmd)
                    agent.delay_timer = getattr(agent, 'delay_timer', 0) + 1

                    yaw_diff = abs(math.atan2(math.sin(target_yaw - agent.yaw), math.cos(target_yaw - agent.yaw)))
                    if (agent.delay_timer >= 25 and yaw_diff < math.radians(6.0)) or agent.delay_timer >= 120:
                        agent.row_idx += 1
                        agent.state = 'sweeping_row'
                        agent.delay_timer = 0
                        agent.last_bypassed_obs_id = None
                        is_rec = (agent.row_idx < len(agent.wp_flags)) and (not agent.wp_flags[agent.row_idx])
                        tag_type = "RECOVERY" if is_rec else "SEL ASLI"
                        self.get_logger().info(f'  🚀 [iris_{did}] Memulai Baris {agent.row_idx+1}/{agent.num_rows} [{tag_type}] (Heading: {math.degrees(agent.yaw):.1f}°)')

            # ─────────────────────────────────────────────────────────
            # 7. RETURN TO CENTROID (Kembali ke Pusat Sel Voronoi Asli)
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

                # Obstacle Avoidance Geodesic Arc & Dynamic Harmonic Avoidance
                v_obs_avoid, speed_scale = self.compute_obstacle_avoidance_offset(did, agent, np.array([v_x, v_y]))
                if speed_scale <= 0.01:
                    v_x = float(v_obs_avoid[0])
                    v_y = float(v_obs_avoid[1])
                else:
                    v_x = v_x * speed_scale + float(v_obs_avoid[0])
                    v_y = v_y * speed_scale + float(v_obs_avoid[1])

                self.send_world_twist(did, v_x, v_y, agent.yaw)

            # ─────────────────────────────────────────────────────────
            # 8. DONE (Hover Mengunci di Centroid & Menghadap Utara)
            # ─────────────────────────────────────────────────────────
            elif agent.state == 'done':
                agent.target_yaw = math.pi / 2.0  # Menghadap UTARA (+90.0°)
                wz_cmd = self.compute_wz(agent.yaw, agent.target_yaw)
                v_x, v_y = 0.0, 0.0
                v_obs_avoid, speed_scale = self.compute_obstacle_avoidance_offset(did, agent, np.array([0.0, 0.0]))
                if speed_scale <= 0.01:
                    v_x = float(v_obs_avoid[0])
                    v_y = float(v_obs_avoid[1])
                else:
                    agent.ref_pos = agent.centroid.copy()
                    if np.linalg.norm(v_obs_avoid) > 0.01:
                        v_x += float(v_obs_avoid[0])
                        v_y += float(v_obs_avoid[1])
                self.send_world_twist(did, v_x, v_y, agent.yaw)

        # Telemetri Terminal setiap 1 detik (20 ticks @ 20Hz)
        if self.step_count % 20 == 0:
            cov = self.get_coverage_percentage()
            states_summary = ' | '.join(f'i{did}:{a.state[:6]}' for did, a in self.agents.items())
            self.get_logger().info(
                f'📊 [STATUS] Cov: {cov:5.1f}% | d_min: {self.global_min_dist:4.2f}m | {states_summary}'
            )

            alive_agents = [a for a in self.agents.values() if a.is_alive and a.state != 'dead']
            all_alive_done = len(alive_agents) > 0 and all(a.state == 'done' for a in alive_agents)

            # Cek jika seluruh drone aktif telah tuntas atau cakupan mencapai 97.0%
            if (cov >= 97.0 or all_alive_done) and not self.mission_completed:
                self.mission_completed = True
                self.get_logger().info(
                    f'🏆 [SWARM SUCCESS] Target Coverage {cov:.1f}% Tercapai! '
                    f'Jarak Terdekat (d_min): {self.global_min_dist:.2f}m | MISI TUNTAS!'
                )
                self.get_logger().info('====================================================================================================')
                self.get_logger().info('  📊 EVALUASI KUANTITATIF TRAJECTORY TRACKING & KINERJA SWARM 7-DRONE (FAULT-TOLERANT)')
                self.get_logger().info('====================================================================================================')
                self.get_logger().info('| Drone  | Role / State   | Cross-Track RMS | Max CT Error | Overshoot Max | Yaw Error Avg | Status |')
                self.get_logger().info('|--------|----------------|-----------------|--------------|---------------|---------------|--------|')
                for d_id, a in self.agents.items():
                    if not a.is_alive or a.state == 'dead':
                        role_str = "DEAD 💥"
                        status_str = "FAIL ❌ (Killed)"
                        self.get_logger().info(
                            f'| iris_{d_id} | {role_str:14s} |        N/A      |      N/A     |      N/A      |      N/A      | {status_str} |'
                        )
                    else:
                        is_helper = any(not f for f in a.wp_flags)
                        role_str = "HELPER 🛡️" if is_helper else "SURVIVOR ✈️"
                        ct_rms = float(np.sqrt(np.mean(np.square(a.cross_track_errors))) * 100.0) if a.cross_track_errors else 0.0
                        ct_max = float(a.max_cross_track_err * 100.0)
                        ov_max = float(a.max_overshoot)
                        yaw_avg = float(np.mean(a.yaw_errors)) if a.yaw_errors else 0.0
                        status_str = "PASS ✅" if ov_max <= 0.01 and ct_max < 25.0 else "WARN ⚠️"
                        self.get_logger().info(
                            f'| iris_{d_id} | {role_str:14s} | {ct_rms:13.2f}cm | {ct_max:10.2f}cm | {ov_max:11.2f}% | {yaw_avg:11.2f}° | {status_str} |'
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
        m_north_arrow.color = ColorRGBA(r=0.85, g=0.85, b=0.85, a=0.90)
        ma.markers.append(m_north_arrow)

        # Label Teks Mata Angin & Staging Base Pad
        compass_labels = [
            (2, "NORTH (+Y)", 0.0, 18.2, 0.3, (0.85, 0.85, 0.85)),
            (3, "SOUTH (-Y)", 0.0, -19.8, 0.3, (0.3, 0.5, 1.0)),
            (4, "EAST (+X)", 17.5, -10.5, 0.3, (0.2, 0.9, 0.3)),
            (5, "WEST (-X)", -16.2, 0.0, 0.3, (0.9, 0.8, 0.2)),
            (6, "📍 BASE / LAUNCH PAD", 0.0, -15.5, 0.2, (0.8, 0.8, 0.8)),
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
            m_lbl.scale.z = 0.75 if lid != 6 else 0.55
            m_lbl.color = ColorRGBA(r=float(lr), g=float(lg), b=float(lb), a=1.0)
            m_lbl.text = text
            ma.markers.append(m_lbl)

        # 3. Poligon Sel 2D Voronoi & Rute Rencana Boustrophedon 3D (Z = 2.00m)
        if self.voronoi_planned:
            # Batas Poligon Sel Voronoi (Drone Hidup) & Hapus Sel Individual Drone Mati
            for did, agent in self.agents.items():
                is_dead = (not agent.is_alive or agent.state == 'dead')

                if is_dead:
                    # Hapus sel individual drone mati agar tidak muncul sekat di area gabungan
                    m_del = Marker()
                    m_del.header.frame_id = 'world'
                    m_del.header.stamp = stamp
                    m_del.ns = 'voronoi_cells'
                    m_del.id = 10 + did
                    m_del.action = Marker.DELETE
                    ma.markers.append(m_del)
                elif len(agent.cell_polygon) >= 3:
                    m_poly = Marker()
                    m_poly.header.frame_id = 'world'
                    m_poly.header.stamp = stamp
                    m_poly.ns = 'voronoi_cells'
                    m_poly.id = 10 + did
                    m_poly.type = Marker.LINE_STRIP
                    m_poly.action = Marker.ADD
                    m_poly.scale.x = 0.06
                    r, g, b = agent.color
                    m_poly.color = ColorRGBA(r=r, g=g, b=b, a=0.9)

                    for p in agent.cell_polygon + [agent.cell_polygon[0]]:
                        pt = Point()
                        pt.x, pt.y, pt.z = float(p[0]), float(p[1]), 0.05
                        m_poly.points.append(pt)
                    ma.markers.append(m_poly)

            # Poligon Gabungan Sel Mati (Merged Dead Cells - Tanpa Garis Sekat di Dalam)
            if hasattr(self, 'merged_dead_comp_polys') and self.merged_dead_comp_polys:
                for c_idx, comp in enumerate(self.merged_dead_comp_polys):
                    if len(comp) >= 3:
                        m_comp = Marker()
                        m_comp.header.frame_id = 'world'
                        m_comp.header.stamp = stamp
                        m_comp.ns = 'merged_dead_cells'
                        m_comp.id = 500 + c_idx
                        m_comp.type = Marker.LINE_STRIP
                        m_comp.action = Marker.ADD
                        m_comp.scale.x = 0.14  # Garis tebal merah untuk batas luar area gugur gabungan
                        m_comp.color = ColorRGBA(r=1.0, g=0.15, b=0.15, a=0.95)
                        pts_list = list(comp)
                        if np.linalg.norm(pts_list[0] - pts_list[-1]) > 0.01:
                            pts_list.append(pts_list[0])
                        for p in pts_list:
                            pt = Point()
                            pt.x, pt.y, pt.z = float(p[0]), float(p[1]), 0.05
                            m_comp.points.append(pt)
                        ma.markers.append(m_comp)

                # Hapus sisa slot ID jika ada komponen berkurang
                for c_idx in range(len(self.merged_dead_comp_polys), 10):
                    m_del = Marker()
                    m_del.header.frame_id = 'world'
                    m_del.header.stamp = stamp
                    m_del.ns = 'merged_dead_cells'
                    m_del.id = 500 + c_idx
                    m_del.action = Marker.DELETE
                    ma.markers.append(m_del)
            else:
                for c_idx in range(10):
                    m_del = Marker()
                    m_del.header.frame_id = 'world'
                    m_del.header.stamp = stamp
                    m_del.ns = 'merged_dead_cells'
                    m_del.id = 500 + c_idx
                    m_del.action = Marker.DELETE
                    ma.markers.append(m_del)

            # Jalur Rencana Boustrophedon 3D (Waypoints Asli vs Recovery)
            for did, agent in self.agents.items():
                is_dead = (not agent.is_alive or agent.state == 'dead')

                # Jalur Rencana Boustrophedon 3D (Waypoints Asli vs Recovery)
                if is_dead or len(agent.waypoints) < 2:
                    # Hapus secara eksplisit seluruh garis lintasan Boustrophedon & waypoints drone mati dari RViz
                    for ns, mid in [('planned_paths', 20 + did), ('recovery_paths', 300 + did), ('planned_waypoints', 80 + did)]:
                        m_del = Marker()
                        m_del.header.frame_id = 'world'
                        m_del.header.stamp = stamp
                        m_del.ns = ns
                        m_del.id = mid
                        m_del.action = Marker.DELETE
                        ma.markers.append(m_del)
                else:
                    r, g, b = agent.color

                    m_plan_own = Marker()
                    m_plan_own.header.frame_id = 'world'
                    m_plan_own.header.stamp = stamp
                    m_plan_own.ns = 'planned_paths'
                    m_plan_own.id = 20 + did
                    m_plan_own.type = Marker.LINE_STRIP
                    m_plan_own.action = Marker.ADD
                    m_plan_own.scale.x = 0.05
                    m_plan_own.color = ColorRGBA(r=r, g=g, b=b, a=0.75)

                    m_plan_rec = Marker()
                    m_plan_rec.header.frame_id = 'world'
                    m_plan_rec.header.stamp = stamp
                    m_plan_rec.ns = 'recovery_paths'
                    m_plan_rec.id = 300 + did
                    m_plan_rec.type = Marker.LINE_STRIP
                    m_plan_rec.action = Marker.ADD
                    m_plan_rec.scale.x = 0.09  # Tebal bercahaya untuk rute recovery
                    m_plan_rec.color = ColorRGBA(r=r, g=g, b=b, a=1.0)

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

                    for r_i in range(agent.num_rows):
                        idx_s = r_i * 2
                        idx_e = idx_s + 1
                        if idx_e < len(agent.waypoints):
                            is_own_row = (r_i < len(agent.wp_flags)) and agent.wp_flags[r_i]
                            wp_a = agent.waypoints[idx_s]
                            wp_b = agent.waypoints[idx_e]

                            pt_a = Point()
                            pt_a.x, pt_a.y, pt_a.z = float(wp_a[0]), float(wp_a[1]), float(self.cruise_alt)
                            pt_b = Point()
                            pt_b.x, pt_b.y, pt_b.z = float(wp_b[0]), float(wp_b[1]), float(self.cruise_alt)

                            if is_own_row:
                                m_plan_own.points.extend([pt_a, pt_b])
                            else:
                                m_plan_rec.points.extend([pt_a, pt_b])
                            m_wps.points.extend([pt_a, pt_b])

                    if len(m_plan_own.points) > 0:
                        ma.markers.append(m_plan_own)
                    else:
                        m_del_own = Marker()
                        m_del_own.header.frame_id = 'world'
                        m_del_own.header.stamp = stamp
                        m_del_own.ns = 'planned_paths'
                        m_del_own.id = 20 + did
                        m_del_own.action = Marker.DELETE
                        ma.markers.append(m_del_own)

                    if len(m_plan_rec.points) > 0:
                        ma.markers.append(m_plan_rec)
                    else:
                        m_del_rec = Marker()
                        m_del_rec.header.frame_id = 'world'
                        m_del_rec.header.stamp = stamp
                        m_del_rec.ns = 'recovery_paths'
                        m_del_rec.id = 300 + did
                        m_del_rec.action = Marker.DELETE
                        ma.markers.append(m_del_rec)

                    if len(m_wps.points) > 0:
                        ma.markers.append(m_wps)

        # 4. Moving Carrot Reference Spheres, Drone Body Hub, FoV, and Tags
        for did, agent in self.agents.items():
            if not agent.odom_received:
                continue

            is_dead = (not agent.is_alive or agent.state == 'dead')
            r, g, b = agent.color
            px, py, pz = float(agent.pos[0]), float(agent.pos[1]), float(agent.pos[2])

            if is_dead:
                # Hapus carrot, FoV, dan arrow heading dari RViz untuk drone mati
                for ns, mid in [('drone_reference_carrots', 200 + did), ('sensor_fov', 30 + did), ('drone_heading_arrows', 50 + did)]:
                    m_del = Marker()
                    m_del.header.frame_id = 'world'
                    m_del.header.stamp = stamp
                    m_del.ns = ns
                    m_del.id = mid
                    m_del.action = Marker.DELETE
                    ma.markers.append(m_del)
            else:
                # REAL-TIME DYNAMIC MOVING REFERENCE CARROT SPHERE (Hanya untuk drone hidup)
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
                m_carrot.pose.orientation.w = 1.0
                m_carrot.scale.x = 0.22
                m_carrot.scale.y = 0.22
                m_carrot.scale.z = 0.22
                m_carrot.color = ColorRGBA(r=r, g=g, b=b, a=0.95)
                ma.markers.append(m_carrot)

                # Lingkaran FoV Sensor di Tanah (Hanya untuk drone aktif melayang Z >= 0.80m)
                if pz >= 0.80:
                    m_fov = Marker()
                    m_fov.header.frame_id = 'world'
                    m_fov.header.stamp = stamp
                    m_fov.ns = 'sensor_fov'
                    m_fov.id = 30 + did
                    m_fov.type = Marker.LINE_STRIP
                    m_fov.action = Marker.ADD
                    m_fov.pose.orientation.w = 1.0
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

                # Panah Arah Heading Real-Time (Orientasi Arah Hadap Drone - Real-Time Lifetime)
                m_arrow = Marker()
                m_arrow.header.frame_id = 'world'
                m_arrow.header.stamp = stamp
                m_arrow.ns = 'drone_heading_arrows'
                m_arrow.id = 50 + did
                m_arrow.type = Marker.ARROW
                m_arrow.action = Marker.ADD
                m_arrow.lifetime = Duration(nanoseconds=int(0.15 * 1e9)).to_msg()
                
                p_start = Point()
                p_start.x, p_start.y, p_start.z = px, py, pz
                p_end = Point()
                arrow_len = 0.55
                p_end.x = float(px + arrow_len * math.cos(agent.yaw))
                p_end.y = float(py + arrow_len * math.sin(agent.yaw))
                p_end.z = pz

                m_arrow.points = [p_start, p_end]
                m_arrow.scale.x = 0.06  # Shaft diameter
                m_arrow.scale.y = 0.14  # Head diameter
                m_arrow.scale.z = 0.16  # Head length
                m_arrow.color = ColorRGBA(r=float(r), g=float(g), b=float(b), a=1.0)
                ma.markers.append(m_arrow)

            # Drone Center Hub Sphere (Badan Utama Drone di tanah)
            m_hub = Marker()
            m_hub.header.frame_id = 'world'
            m_hub.header.stamp = stamp
            m_hub.ns = 'drone_hubs'
            m_hub.id = 40 + did
            m_hub.type = Marker.SPHERE
            m_hub.action = Marker.ADD
            m_hub.pose.position.x = px
            m_hub.pose.position.y = py
            m_hub.pose.position.z = max(0.05, pz)
            m_hub.pose.orientation.w = 1.0
            m_hub.scale.x = 0.35
            m_hub.scale.y = 0.35
            m_hub.scale.z = 0.15
            if is_dead:
                m_hub.color = ColorRGBA(r=0.9, g=0.1, b=0.1, a=0.95)
            else:
                m_hub.color = ColorRGBA(r=r, g=g, b=b, a=1.0)
            ma.markers.append(m_hub)

            # Name Tag Minimalis & Status Indikator
            m_tag = Marker()
            m_tag.header.frame_id = 'world'
            m_tag.header.stamp = stamp
            m_tag.ns = 'drone_tags'
            m_tag.id = 60 + did
            m_tag.type = Marker.TEXT_VIEW_FACING
            m_tag.action = Marker.ADD
            m_tag.pose.position.x = px
            m_tag.pose.position.y = py
            m_tag.pose.position.z = max(0.05, pz) + 0.40
            m_tag.pose.orientation.w = 1.0
            m_tag.scale.z = 0.38
            if is_dead:
                m_tag.color = ColorRGBA(r=1.0, g=0.2, b=0.2, a=1.0)
                m_tag.text = f'iris_{did} [DEAD]'
            else:
                is_helper = any(not f for f in agent.wp_flags)
                tag_label = " [HELPER]" if is_helper else ""
                m_tag.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
                m_tag.text = f'iris_{did}{tag_label}'
            ma.markers.append(m_tag)

            # Publikasi Jalur Terbang Riil (Actual Path)
            agent.path_msg.header.stamp = stamp
            self.pub_actual_path[did].publish(agent.path_msg)

        # 5. Status HUD Sidebar — Panel Samping Kanan (East / X = 22.0m)
        cov = self.get_coverage_percentage()
        t_elapsed = self.step_count / 20.0  # Timer @20Hz
        minutes = int(t_elapsed) // 60
        seconds = int(t_elapsed) % 60

        # Mapped cells vs total valid active cells
        i_s = int((self.active_x_min - self.x_min) / self.dx)
        i_e = int((self.active_x_max - self.x_min) / self.dx)
        j_s = int((self.active_y_min - self.y_min) / self.dy)
        j_e = int((self.active_y_max - self.y_min) / self.dy)
        sub = self.cov_grid[i_s:i_e, j_s:j_e]
        if self.enable_obstacles and hasattr(self, 'obstacle_mask'):
            valid_mask = ~self.obstacle_mask[i_s:i_e, j_s:j_e]
            mapped_cells = int(np.sum(sub & valid_mask))
            total_cells = int(np.sum(valid_mask))
        else:
            mapped_cells = int(np.sum(sub))
            total_cells = sub.size

        alive_count = sum(1 for a in self.agents.values() if a.is_alive and a.state != 'dead')

        scheme_labels = {
            1: "Scheme 1 (Nominal)",
            2: "Scheme 2 (Dryden Wind)",
            3: "Scheme 3 (Obstacles)",
            4: "Scheme 4 (Combined)"
        }
        sch_str = scheme_labels.get(self.scheme, "Custom Scheme")

        # ── 5a. Global Summary Header (X = 22.0m, Y = 13.5m) ──
        m_dash_title = Marker()
        m_dash_title.header.frame_id = 'world'
        m_dash_title.header.stamp = stamp
        m_dash_title.ns = 'hud_sidebar'
        m_dash_title.id = 90
        m_dash_title.type = Marker.TEXT_VIEW_FACING
        m_dash_title.action = Marker.ADD
        m_dash_title.pose.position.x = 22.0
        m_dash_title.pose.position.y = 13.5
        m_dash_title.pose.position.z = 1.0
        m_dash_title.pose.orientation.w = 1.0
        m_dash_title.scale.z = 0.80
        m_dash_title.color = ColorRGBA(r=0.95, g=0.95, b=0.95, a=0.95)
        m_dash_title.text = f'SWARM DASHBOARD  |  {sch_str}'
        ma.markers.append(m_dash_title)

        # ── 5b. Overall Coverage (X = 22.0m, Y = 11.8m) ──
        m_dash_cov = Marker()
        m_dash_cov.header.frame_id = 'world'
        m_dash_cov.header.stamp = stamp
        m_dash_cov.ns = 'hud_sidebar'
        m_dash_cov.id = 91
        m_dash_cov.type = Marker.TEXT_VIEW_FACING
        m_dash_cov.action = Marker.ADD
        m_dash_cov.pose.position.x = 22.0
        m_dash_cov.pose.position.y = 11.8
        m_dash_cov.pose.position.z = 1.0
        m_dash_cov.pose.orientation.w = 1.0
        m_dash_cov.scale.z = 0.95
        if cov >= 95.0:
            m_dash_cov.color = ColorRGBA(r=0.1, g=1.0, b=0.3, a=0.98)
        elif cov >= 50.0:
            m_dash_cov.color = ColorRGBA(r=1.0, g=0.88, b=0.1, a=0.98)
        else:
            m_dash_cov.color = ColorRGBA(r=0.2, g=0.85, b=1.0, a=0.95)

        m_dash_cov.text = (
            f'TOTAL COVERAGE: {cov:5.1f}%  |  TIME: {minutes:02d}:{seconds:02d}\n'
            f'Cells: {mapped_cells}/{total_cells}  |  Active: {alive_count}/7 Drones'
        )
        ma.markers.append(m_dash_cov)

        # ── 5c. Per-Drone Individual Cards (X = 22.0m, Y = 9.5 to -4.5m) ──
        for did in range(1, 8):
            agent = self.agents[did]
            is_dead = (not agent.is_alive or agent.state == 'dead')
            is_helper = any(not f for f in agent.wp_flags)
            cr, cg, cb = self.drone_colors[did - 1]

            m_dcard = Marker()
            m_dcard.header.frame_id = 'world'
            m_dcard.header.stamp = stamp
            m_dcard.ns = 'hud_sidebar'
            m_dcard.id = 100 + did
            m_dcard.type = Marker.TEXT_VIEW_FACING
            m_dcard.action = Marker.ADD
            m_dcard.pose.position.x = 22.0
            m_dcard.pose.position.y = 9.5 - (did - 1) * 2.3
            m_dcard.pose.position.z = 1.0
            m_dcard.pose.orientation.w = 1.0
            m_dcard.scale.z = 0.62

            if is_dead:
                m_dcard.color = ColorRGBA(r=1.0, g=0.25, b=0.25, a=0.90)
                m_dcard.text = (
                    f'iris_{did} [DEAD 💥]  •  INACTIVE\n'
                    f'Coverage: Reassigned to Helper'
                )
            else:
                m_dcard.color = ColorRGBA(r=float(cr), g=float(cg), b=float(cb), a=1.0)
                role_tag = "[HELPER 🛡️]" if is_helper else "[PRIMARY]"
                
                # Format state & progress
                if agent.state == 'done':
                    st_str = "COMPLETED ✅"
                    prog = 100.0
                elif agent.state == 'sweeping_row':
                    st_str = f"SWEEPING (Row {agent.row_idx + 1}/{agent.num_rows})"
                    prog = min(100.0, ((agent.row_idx + 1) / max(1, agent.num_rows)) * 100.0)
                elif agent.state in ['stepping_vertical', 'delay_at_corner_end', 'delay_at_new_row']:
                    st_str = f"NEXT ROW ({agent.row_idx + 1}/{agent.num_rows})"
                    prog = min(100.0, (agent.row_idx / max(1, agent.num_rows)) * 100.0)
                elif agent.state in ['transit_to_start', 'wait_takeoff', 'wait_all_start', 'align_start_yaw']:
                    st_str = "TRANSIT 🚀"
                    prog = 0.0
                else:
                    st_str = agent.state.upper()
                    prog = min(100.0, (agent.row_idx / max(1, agent.num_rows)) * 100.0) if agent.num_rows > 0 else 0.0

                m_dcard.text = (
                    f'iris_{did} {role_tag}  •  {st_str}\n'
                    f'Coverage: {prog:5.1f}%  |  Pos: ({agent.pos[0]:4.1f}, {agent.pos[1]:4.1f})'
                )

            ma.markers.append(m_dcard)

        # 6. Grid Cakupan Hijau Padat, Kontras & Bebas Z-Fighting (Numpy Vectorized Fast Extraction)
        m_grid = Marker()
        m_grid.header.frame_id = 'world'
        m_grid.header.stamp = stamp
        m_grid.ns = 'coverage_footprint'
        m_grid.id = 100
        m_grid.type = Marker.CUBE_LIST
        m_grid.pose.position.x = 0.0
        m_grid.pose.position.y = 0.0
        m_grid.pose.position.z = 0.0
        m_grid.pose.orientation.x = 0.0
        m_grid.pose.orientation.y = 0.0
        m_grid.pose.orientation.z = 0.0
        m_grid.pose.orientation.w = 1.0
        m_grid.scale.x = float(self.dx * 1.02)
        m_grid.scale.y = float(self.dy * 1.02)
        m_grid.scale.z = 0.04  # Tebal 4cm agar solid & tidak terpotong garis grid lantai
        m_grid.color = ColorRGBA(r=0.08, g=0.98, b=0.28, a=0.70)  # Hijau neon cerah & kontras

        indices = np.argwhere(self.cov_grid)
        if len(indices) > 0:
            m_grid.action = Marker.ADD
            for idx in indices:
                pt = Point()
                pt.x = float(self.x_min + (idx[0] + 0.5) * self.dx)
                pt.y = float(self.y_min + (idx[1] + 0.5) * self.dy)
                pt.z = 0.035  # Terangkat di atas lantai Z=0 agar sangat jelas terlihat dari segala view
                m_grid.points.append(pt)
        else:
            m_grid.action = Marker.DELETE
        ma.markers.append(m_grid)

        # 7. Visualisasi Rintangan 3D Statis & Dinamis di RViz2
        if self.enable_obstacles:
            # Rintangan Statis (9 Silinder)
            for obs_id, cell_did, ox, oy, rad, height, (cr, cg, cb) in self.static_obstacles:
                m_obs = Marker()
                m_obs.header.frame_id = 'world'
                m_obs.header.stamp = stamp
                m_obs.ns = 'static_obstacles'
                m_obs.id = obs_id
                m_obs.type = Marker.CYLINDER
                m_obs.action = Marker.ADD
                m_obs.pose.position.x = float(ox)
                m_obs.pose.position.y = float(oy)
                m_obs.pose.position.z = float(height * 0.5)
                m_obs.pose.orientation.w = 1.0
                m_obs.scale.x = float(rad * 2.0)
                m_obs.scale.y = float(rad * 2.0)
                m_obs.scale.z = float(height)
                m_obs.color = ColorRGBA(r=float(cr), g=float(cg), b=float(cb), a=0.90)
                ma.markers.append(m_obs)

                # Translucent Safety Bubble (Radius = rad + 0.45m)
                m_bub = Marker()
                m_bub.header.frame_id = 'world'
                m_bub.header.stamp = stamp
                m_bub.ns = 'obstacle_safety_bubbles'
                m_bub.id = obs_id + 500
                m_bub.type = Marker.CYLINDER
                m_bub.action = Marker.ADD
                m_bub.pose.position.x = float(ox)
                m_bub.pose.position.y = float(oy)
                m_bub.pose.position.z = 0.04
                m_bub.pose.orientation.w = 1.0
                m_bub.scale.x = float((rad + 0.45) * 2.0)
                m_bub.scale.y = float((rad + 0.45) * 2.0)
                m_bub.scale.z = 0.03
                m_bub.color = ColorRGBA(r=1.0, g=0.2, b=0.2, a=0.30)
                ma.markers.append(m_bub)

            # Rintangan Dinamis (2 Silinder Bergerak Pola 'X')
            for dyn_obs in self.dynamic_obstacles:
                d_id = dyn_obs['id']
                d_pos = dyn_obs['pos']
                d_col = dyn_obs['color']

                m_dyn = Marker()
                m_dyn.header.frame_id = 'world'
                m_dyn.header.stamp = stamp
                m_dyn.ns = 'dynamic_obstacles'
                m_dyn.id = d_id
                m_dyn.type = Marker.CYLINDER
                m_dyn.action = Marker.ADD
                m_dyn.pose.position.x = float(d_pos[0])
                m_dyn.pose.position.y = float(d_pos[1])
                m_dyn.pose.position.z = 2.05
                m_dyn.pose.orientation.w = 1.0
                m_dyn.scale.x = 0.90
                m_dyn.scale.y = 0.90
                m_dyn.scale.z = 3.6
                m_dyn.color = ColorRGBA(r=float(d_col[0]), g=float(d_col[1]), b=float(d_col[2]), a=0.95)
                ma.markers.append(m_dyn)

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
