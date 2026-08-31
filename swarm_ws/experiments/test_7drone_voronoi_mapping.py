#!/usr/bin/env python3
"""
===============================================================================
  SWARM 7-DRONE 2D VORONOI & BOUSTROPHEDON MAPPING COORDINATOR (STEP 6 v8.3)
===============================================================================
Fitur Utama:
  0. WILAYAH PEMETAAN POLIGON (BOLEH NON-CONVEX):
     - Bentuk wilayah dari preset ('rect', 'l_shape', 'u_shape', 'plus') atau
       berkas YAML lewat parameter ROS 'region'.
     - Lloyd/Voronoi, boustrophedon, dan grid coverage semua sadar-poligon.
  1. CRITICALLY DAMPED TRACKING & ZERO OVERSHOOT:
     - RAMP-DOWN feedforward pada brake_dist (default 1.0 m) menjelang ujung baris.
     - Clamp ujung baris DIKEMBALIKAN: _cbf_filter mengunci komponen ref_pos
       sepanjang garis sapuan ke [0, line_len] setelah QP — komponen lateral
       (koreksi cross-track + V2V) tetap lolos. Overshoot longitudinal = 0
       secara desain, bukan konstanta yang di-tune.
     - sweep_speed default 1.6 m/s (bukan 2.85 lama yang tak pernah tercapai &
       memaksa tilt feedforward melewati batas 15°).
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

import itertools
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
from shapely.geometry import (Polygon as SpPolygon, MultiPolygon as SpMultiPolygon,
                              Point as ShapelyPoint)
from shapely.ops import unary_union
from swarm_high_level.world.coverage_path import (
    OBSTACLE_KEEP_OUT, clip_poly_to_region, generate_boustrophedon,
    poly_centroid)
from swarm_high_level.world.obstacles import obstacles_for_region


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
        # Array kosong (bukan None) agar len() aman saat scan belum/tidak pernah tiba.
        self.lidar_ranges = np.zeros(0, dtype=np.float32)
        self.min_dist_to_obs = float('inf')
        self.my_static_obstacles = []    # Rintangan statis yang berlokasi eksklusif di dalam sel Voronoi drone ini
        self.transit_waypoints = []      # Koridor transit aman menuju sel
        self.transit_wp_idx = 0


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

        # ── Wilayah Pemetaan (poligon, boleh non-convex) ────────────
        # Preset atau path YAML lewat parameter 'region'. Arena Gazebo tetap
        # ~30x30 m; wilayah pemetaan adalah poligon di dalamnya.
        self.declare_parameter('region', 'rect')
        region_arg = str(self.get_parameter('region').value)
        try:
            from swarm_high_level.world.region import (
                grid_region_mask, load_region)
        except ImportError as exc:
            raise RuntimeError(
                f'Gagal impor swarm_high_level.world.region: {exc}\n'
                '   Jalankan: colcon build --packages-select swarm_high_level') from exc
        self.region_ring, self.region_poly = load_region(region_arg)
        self.region_name = region_arg
        self._grid_region_mask = grid_region_mask

        rx0, ry0, rx1, ry1 = self.region_poly.bounds
        self.cruise_alt = 2.0
        self.sensor_radius = 0.95

        # Arena grid = bbox wilayah + padding 1 m di tiap sisi.
        self.x_min, self.x_max = rx0 - 1.0, rx1 + 1.0
        self.y_min, self.y_max = ry0 - 1.0, ry1 + 1.0

        # bbox = cincin-luar wilayah (marker RViz).
        self.bbox = [np.asarray(v, dtype=float) for v in self.region_ring]
        # bbox_rect = persegi pembatas wilayah. Klip bisector Voronoi
        # (Sutherland-Hodgman) HANYA sahih pada poligon convex, jadi Lloyd
        # dimulai dari persegi ini lalu diiris ke region_poly via Shapely.
        self.bbox_rect = [
            np.array([rx0, ry0]), np.array([rx1, ry0]),
            np.array([rx1, ry1]), np.array([rx0, ry1]),
        ]

        # ── Parameter Kecepatan (dapat di-override per skema di launcher) ──
        # nominal_speed lama 2.85 m/s tidak pernah tercapai (maks terukur
        # 1.86) dan feedforward k_ff*v melampaui batas tilt 15°. Default baru
        # 1.6 m/s: benar-benar tercapai, tilt ~13.7°, waktu misi ~sama.
        self.declare_parameter('sweep_speed', 1.6)
        self.declare_parameter('transit_speed', 2.2)
        self.declare_parameter('step_speed', 1.8)
        self.declare_parameter('brake_dist', 1.0)
        # Auto-exit: berhenti N detik-sim SETELAH seluruh drone hidup
        # berstatus 'done'. 0 = jangan pernah keluar (perilaku lama).
        # Tanpa ini setiap run terkunci sampai timeout skrip pemanggil,
        # padahal misi sudah tuntas -> sweep 6 misi buang ~3 jam.
        self.declare_parameter('exit_after_success', 0.0)
        # Batas koreksi cross-track. Dinaikkan dari 0.45 supaya lintasan tetap
        # lurus saat berangin; efeknya roll lebih besar (usaha kendali naik),
        # yang memang itulah cara angin seharusnya terlihat.
        self.declare_parameter('ct_corr_max', 0.90)
        self.nominal_speed = float(self.get_parameter('sweep_speed').value)
        self.transit_speed = float(self.get_parameter('transit_speed').value)
        self.step_speed = float(self.get_parameter('step_speed').value)
        self.brake_dist = float(self.get_parameter('brake_dist').value)
        self.exit_after_success = float(self.get_parameter('exit_after_success').value)
        self.ct_corr_max = float(self.get_parameter('ct_corr_max').value)
        self.max_cmd_speed = 3.00       # Saturation limit (m/s)
        self.kp_track = 2.20            # Tracking gain lateral
        self.lead_dist = 0.70           # Jarak maju virtual carrot di depan drone (m)

        # Pengaman anti-deadlock (dipakai di control_loop). 1200 tik x 0.05 s
        # = 60 s. Radius terimanya BERBEDA untuk dua state, dan bedanya
        # disengaja:
        #
        #  * titik start (1.20 m) — di sinilah sapuan dimulai, jadi menerima
        #    terlalu jauh berarti baris pertama dimulai di luar jalur. Terukur
        #    memadai: pengaman ini menyala pada sisa 0.64 m.
        #  * centroid (2.50 m) — ini murni tempat PARKIR setelah pemetaan
        #    selesai; cakupan sudah terkunci sebelum drone pulang, jadi parkir
        #    melenceng 2 m tidak berbiaya apa pun. Radius 1.20 m terbukti
        #    terlalu ketat: iris_2 di u_shape mengorbit stabil pada 1.21-1.48 m
        #    dan TIDAK PERNAH masuk (0% sampel), sehingga misi tak pernah
        #    berstatus selesai dan habis di timeout 2400 s.
        self.TRANSIT_STUCK_TICKS = 1200
        self.TRANSIT_STUCK_ACCEPT = 1.20
        self.RETURN_STUCK_ACCEPT = 2.50
        self.corner_settle_ticks = 3    # Jeda 0.15 detik saat pivot statis di sudut (@20Hz)

        # ── Coverage Grid (100 x 100 sel) ───────────────────────────
        self.grid_n = 100
        self.cov_grid = np.zeros((self.grid_n, self.grid_n), dtype=bool)
        self.dx = (self.x_max - self.x_min) / self.grid_n
        self.dy = (self.y_max - self.y_min) / self.grid_n
        # Sel grid yang pusatnya di dalam wilayah — dasar perhitungan coverage.
        self.region_mask = self._grid_region_mask(
            self.region_poly, self.x_min, self.y_min,
            self.dx, self.dy, self.grid_n)

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
        self._done_step = None

        # ── Parameter 4 Skema Pemetaan & Konfigurasi Lingkungan ──────
        self.declare_parameter('scheme', 1)
        self.declare_parameter('enable_wind', False)
        self.declare_parameter('enable_obstacles', False)
        # Rintangan DINAMIS terpisah dari statis. Skema 3 = statis saja: kedua
        # silinder bergerak tidak di-spawn di world-nya, jadi mengaktifkan
        # jalur dinamis di sini hanya akan mengejar odometri yang tak pernah
        # datang dan memberi QP rintangan phantom.
        self.declare_parameter('enable_dynamic_obstacles', False)
        # PERSEPSI. Bila true, rintangan DITEMUKAN dari LiDAR dan tabel
        # koordinat tidak dipakai sama sekali — tidak ada peta a-priori.
        # Tabel tetap ada hanya untuk men-spawn silinder di Gazebo dan untuk
        # menilai hasil (ground truth penilaian, bukan masukan kendali).
        self.declare_parameter('use_lidar_obstacles', True)

        self.scheme = int(self.get_parameter('scheme').value)
        self.enable_wind = bool(self.get_parameter('enable_wind').value) or (self.scheme in [2, 4])
        self.enable_obstacles = bool(self.get_parameter('enable_obstacles').value) or (self.scheme in [3, 4])
        self.enable_dynamic_obstacles = (
            bool(self.get_parameter('enable_dynamic_obstacles').value)
            or self.scheme == 4)

        # ── Rintangan Statis: set khusus per wilayah ───────────────────
        # Sembilan silinder lama hanya seluruhnya berada di dalam `rect`; di
        # wilayah non-convex hanya 6 yang di dalam. `obstacles_for_region`
        # memberi tiap wilayah 9 rintangan di dalam wilayahnya sendiri, dan
        # berkas world Skema 3 dibangkitkan dari tabel yang sama.
        # (id, x, y, radius, height, color_rgb)
        self.truth_obstacles = (obstacles_for_region(self.region_name)
                                if self.enable_obstacles else [])
        self.use_lidar_obstacles = (
            bool(self.get_parameter('use_lidar_obstacles').value)
            and self.enable_obstacles)

        # `static_obstacles` adalah apa yang DIKETAHUI sistem. Dengan persepsi
        # aktif ia mulai KOSONG dan diisi oleh LiDAR; tanpa persepsi ia berisi
        # tabel koordinat seperti sebelumnya.
        self.static_obstacles = [] if self.use_lidar_obstacles else list(self.truth_obstacles)

        self.obs_map = None
        self._detect_obstacles = None
        if self.use_lidar_obstacles:
            try:
                from swarm_mid_level.perception.obstacle_map import (
                    ObstacleMap, detect)
            except ImportError as exc:
                raise RuntimeError(
                    f'Gagal impor swarm_mid_level.perception: {exc}\n'
                    '   Jalankan: colcon build --packages-select swarm_mid_level') from exc
            self.obs_map = ObstacleMap()
            self._detect_obstacles = detect

        scheme_names = {
            1: "Skema 1: Nominal Mapping (Zero Disturbance)",
            2: "Skema 2: Dryden Wind Turbulence Mapping",
            3: "Skema 3: Obstacle Avoidance Mapping (rintangan statis)",
            4: "Skema 4: Combined Disturbance & Obstacles Mapping"
        }
        if not self.enable_obstacles:
            obs_desc = 'NONAKTIF'
        else:
            src = ('LiDAR — tanpa peta a-priori' if self.use_lidar_obstacles
                   else 'tabel koordinat a-priori')
            obs_desc = (f'AKTIF ({len(self.truth_obstacles)} statis di Gazebo, '
                        f'sumber: {src}'
                        + (' + 2 dinamis pola X' if self.enable_dynamic_obstacles
                           else ', tanpa rintangan dinamis')
                        + f', wilayah {self.region_name})')
        self.get_logger().info("=========================================================================")
        self.get_logger().info(f"🚁 SWARM KOORDINATOR AKTIF: [{scheme_names.get(self.scheme, 'Skema Custom')}]")
        self.get_logger().info(f"   🌪️  Wind Disturbance: {'AKTIF' if self.enable_wind else 'NONAKTIF'}")
        self.get_logger().info(f"   🚧 Obstacles Engine: {obs_desc}")
        self.get_logger().info("=========================================================================")

        # ── Definisi Rintangan Dinamis (2 Silinder Pola 'X') ──────────
        self.dynamic_obstacles = []
        self.kf_dyn_obs = []
        self.last_dyn_obs_t = None
        self.pub_dyn_obs_vel_1 = self.pub_dyn_obs_vel_2 = None
        if self.enable_dynamic_obstacles:
            self.dynamic_obstacles = [
                {'id': 201, 'pos': np.array([-10.0, 10.0], dtype=float), 'vel': np.zeros(2), 'color': (1.0, 0.1, 0.1), 'name': 'dynamic_obs_1'},
                {'id': 202, 'pos': np.array([ 10.0, 10.0], dtype=float), 'vel': np.zeros(2), 'color': (1.0, 0.5, 0.0), 'name': 'dynamic_obs_2'},
            ]
            self.kf_dyn_obs = [
                DynamicObstacleKalmanFilter(init_pos=np.array([-10.0, 10.0])),
                DynamicObstacleKalmanFilter(init_pos=np.array([ 10.0, 10.0]))
            ]
            self.pub_dyn_obs_vel_1 = self.create_publisher(Twist, '/model/dynamic_obs_1/cmd_vel', 10)
            self.pub_dyn_obs_vel_2 = self.create_publisher(Twist, '/model/dynamic_obs_2/cmd_vel', 10)

        # ── Masking Grid Okupansi untuk Rintangan Statis ───────────────
        self.obstacle_mask = np.zeros((self.grid_n, self.grid_n), dtype=bool)
        if self.enable_obstacles:
            # PENILAIAN, bukan kendali: sel di bawah silinder memang tidak bisa
            # dipetakan siapa pun, jadi penyebutnya memakai kebenaran lapangan.
            # Ini juga menjaga angka coverage tetap sebanding dengan 18 misi lama.
            for _, ox, oy, rad, _, _ in self.truth_obstacles:
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
        if self.enable_dynamic_obstacles:
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

        # ── Lapisan Penghindaran CBF-QP (opsional) ───────────────────
        self._setup_cbf()

        # Timer 20 Hz (50ms) untuk Loop Kontrol dan Visualisasi Presisi Tinggi
        self.timer_control = self.create_timer(0.05, self.control_loop)
        self.timer_viz = self.create_timer(0.10, self.publish_rviz_markers)

        self.get_logger().info(
            f'🚀 Swarm 7-Drone 2D Voronoi Mapping Node Siap (v8.3 Longitudinal Yield & Barrier)! '
            f'(Arena: {self.x_max-self.x_min:.0f}x{self.y_max-self.y_min:.0f}m, '
            f'Grid: {self.grid_n}x{self.grid_n}, v_nom={self.nominal_speed:.2f}m/s @ 20Hz)'
        )

    # ── Lapisan Penghindaran CBF-QP ──────────────────────────────────
    #
    # Dipasang sebagai FILTER di depan send_world_twist, bukan dengan
    # membongkar state machine. State machine tetap menghitung kecepatan
    # yang DIINGINKAN; QP yang memutuskan kecepatan yang AMAN. Dengan
    # QP selalu aktif; tidak ada lagi jalur penghindaran alternatif.


    # State yang yaw-nya harus dikunci: v_safe yang terotasi kuat akan
    # mengayunkan target yaw, dan compute_wz membatasi 40 deg/s sehingga
    # putaran 90 derajat memakan 2.25 detik sambil merusak metrik yaw.
    # State yang MENAHAN POSISI di satu titik yang ditentukan state machine.
    # Di sini ref_pos TIDAK boleh diganti pos + T_lead*v_safe: dengan v_safe=0
    # itu membuat referensi mengikuti drone, jadi loop posisi low-level kehilangan
    # jangkarnya dan drone hanyut bebas tertiup angin. Terukur: hanyut 3.1 m
    # selama 112 s wait_all_start di Skema 2, keluar wilayah, dan Baris 1 langsung
    # dianggap tuntas tanpa pernah disapu.
    POS_HOLD_STATES = frozenset((
        'wait_all_start', 'align_start_yaw', 'delay_at_corner_end',
        'delay_at_new_row', 'done', 'pivot_to_transit',
    ))

    YAW_HOLD_STATES = frozenset((
        'sweeping_row', 'sweeping_recovery', 'align_start_yaw',
        'delay_at_corner_end', 'delay_at_new_row', 'wait_all_start', 'done',
    ))

    def _setup_cbf(self):
        self._cbf_cache_step = -1
        self._cbf_last_t = None
        self._cbf_dt = 0.05
        self._cbf_now = 0.0
        self._cbf_agents = {}
        self._cbf_stats = {'ticks': 0, 'tier': [0, 0, 0, 0], 'max_slack': 0.0}

        try:
            from swarm_mid_level.cbf import (
                Bounds, CBFAvoidance, CBFConfig, Obstacle, PlantModel)
            from swarm_mid_level.cbf import types as CT
            from swarm_mid_level.cbf.avoidance import priority_for_state
        except ImportError as exc:
            self.get_logger().error(
                f'❌ use_cbf=true tetapi swarm_mid_level.cbf tidak dapat diimpor: {exc}\n'
                '   Jalankan: colcon build --packages-select swarm_mid_level')
            raise

        self._CT = CT
        self._Obstacle = Obstacle
        self._priority_for_state = priority_for_state

        solver = 'hinf' if 'hinf' in self.get_namespace().lower() else 'lqr'
        self.cbf_plant = PlantModel.from_config(solver='lqr')
        self.cbf_cfg = CBFConfig()
        self.cbf_cfg.v_max = self.max_cmd_speed
        # Anggaran percepatan yang "dicuri" angin dari otoritas menghindar.
        # BELUM DIIDENTIFIKASI — ini perkiraan, bukan hasil ukur.
        #
        # Angin Dryden dipublikasikan ke Gazebo sebagai KECEPATAN (m/s), bukan
        # gaya, sehingga kolom Wind_* di CSV tidak bisa langsung dikonversi ke
        # percepatan. Yang benar-benar memakan otoritas hanyalah komponen yang
        # TIDAK ditolak loop dalam; aksi integral low-level menolak sebagian
        # besar gangguan lambat (Skema 2 mencapai 97.2% coverage tanpa
        # kesulitan), jadi sisanya kecil.
        #
        # Nilai awal 1.25 memangkas a_eff dinamis menjadi 0.60 m/s^2 dan justru
        # membuat constraint MUSTAHIL dipenuhi -> QP jatuh ke Tier 2 ->
        # pelanggaran diterima. Terlalu konservatif menghasilkan tabrakan,
        # bukan mencegahnya.
        self.declare_parameter('cbf_wind_accel', 0.5)
        self.cbf_wind_accel = float(self.get_parameter('cbf_wind_accel').value)

        self.cbf = CBFAvoidance(self.cbf_cfg, self.cbf_plant)
        self.cbf.set_world([], Bounds(self.x_min, self.x_max,
                                      self.y_min, self.y_max))

        self.get_logger().info(
            f'   🛡️  Avoidance: CBF-QP aktif | {self.cbf_plant}')
        self.get_logger().info(
            f'   📐 T_lead={self.cbf_plant.T_lead:.4f}s dipakai untuk ref_pos '
            f'(menggantikan lead_dist={self.lead_dist:.2f}m tetap)')

    def _cbf_obstacles(self):
        """Rakit daftar rintangan untuk QP.

        SELURUH rintangan disertakan, bukan hanya yang ada di sel Voronoi
        drone ini. Pembatasan per-sel di kode lama membuat rintangan #108 di
        (0.0, 2.5) — dekat pusat arena tempat jalur transit bersilangan —
        tidak terlihat oleh setiap drone yang tidak memiliki sel 7.
        """
        if not self.enable_obstacles:
            return []
        obs = [
            self._Obstacle(oid, np.array([ox, oy]), radius=rad,
                           kind=self._CT.CLASS_STATIC)
            for oid, ox, oy, rad, _h, _c in self.static_obstacles
        ]
        for k, dyn in enumerate(self.dynamic_obstacles):
            omega = 0.15 if k == 0 else 0.11
            obs.append(self._Obstacle(
                dyn['id'], np.asarray(dyn['pos'], dtype=float),
                np.asarray(dyn['vel'], dtype=float), 0.45,
                accel_bound=10.0 * omega * omega,
                kind=self._CT.CLASS_DYNAMIC))
        return obs

    def _cbf_snapshot(self):
        """Bangun snapshot seluruh drone sekali per tick.

        Satu snapshot itulah yang membuat split resiprokal lambda_ij +
        lambda_ji = 1 eksak — keunggulan nyata atas ORCA terdistribusi.
        """
        if self._cbf_cache_step == self.step_count:
            return self._cbf_agents

        # dt diukur SEKALI PER TICK, bukan per drone. Ketujuh drone
        # diselesaikan dalam tick yang sama beberapa mikrodetik terpisah;
        # mengukur per drone membuat enam drone terakhir menerima dt~0 yang
        # lalu dijepit ke dt_min, memangkas separuh percepatan yang mereka
        # boleh pakai — kawanan nyaris tidak bergerak.
        self._cbf_now = self.get_clock().now().nanoseconds * 1e-9
        self._cbf_dt = (0.05 if self._cbf_last_t is None
                        else self._cbf_now - self._cbf_last_t)
        self._cbf_last_t = self._cbf_now

        self.cbf.set_world(self._cbf_obstacles(),
                           self._CT.Bounds(self.x_min, self.x_max,
                                           self.y_min, self.y_max),
                           wind_accel=(self.cbf_wind_accel
                                       if self.enable_wind else 0.0))

        agents = {}
        for did, ag in self.agents.items():
            agents[did] = self._CT.AgentState(
                aid=did,
                pos=ag.pos[:2].copy(),
                vel=getattr(ag, 'vel_world', np.zeros(2)),
                v_prev_cmd=getattr(ag, 'cbf_v_prev', np.zeros(2)),
                radius=self.cbf_cfg.drone_radius,
                priority_w=self._priority_for_state(ag.state),
                airborne=bool(ag.pos[2] >= 0.80),
                alive=bool(ag.is_alive and ag.state != 'dead'),
            )
            if getattr(ag, 'cell_polygon', None) is not None:
                self.cbf.set_cell_polygon(did, ag.cell_polygon)

        self._cbf_agents = agents
        self._cbf_cache_step = self.step_count
        return agents

    def _row_clamp_ref(self, agent, v_world_x, v_world_y):
        """Titik acuan saat menyapu — DIPROYEKSIKAN PENUH ke garis baris.

        Komponen lateral sengaja DIBUANG. Kalau tidak, ref_pos = pos +
        T_lead*v ikut tertiup angin (posisi drone melenceng, kecepatan punya
        komponen koreksi lateral), sehingga bola acuan di RViz ikut membelok
        dan loop posisi low-level mengejar garis yang bengkok — angin jadi
        merusak petanya, bukan sekadar menambah usaha kendali.

        Dengan acuan terkunci di garis, angin hanya muncul sebagai roll/pitch
        (usaha kendali) sementara lintasan tetap lurus.
        """
        pos = agent.pos[:2].astype(float)
        ref = pos + self.cbf_plant.T_lead * np.array([v_world_x, v_world_y])
        seg = getattr(agent, '_row_seg', None)
        if seg is not None:
            wp0, u_line, line_len = seg
            s_long = min(max(float(np.dot(ref - wp0, u_line)), 0.0), line_len)
            ref = wp0 + s_long * u_line          # murni di garis baris
        agent.ref_pos = ref.astype(np.float32)

    def _cbf_filter(self, did, v_world_x, v_world_y):
        """Saring kecepatan yang diinginkan lewat QP; kembalikan yang aman."""
        agent = self.agents[did]

        # SAAT MENYAPU DI SKEMA 1/2 (tanpa rintangan): mapping tidak boleh
        # diganggu kawanan. QP di-bypass total selama tak ada tetangga dalam
        # ~0.85 m — margin sel 0.35 m menjamin dua drone bertetangga terpisah
        # >= 0.70 m secara geometris. Bila ada yang menerobos masuk radius itu
        # (mis. saat transisi berdekatan), QP penuh (batas keras V2V 0.70 m)
        # kembali aktif otomatis.
        if (agent.state == 'sweeping_row' and not self.enable_obstacles):
            near = False
            for oid, o in self.agents.items():
                if oid == did or not o.is_alive or o.state == 'dead':
                    continue
                if o.pos[2] > 0.8 and float(np.linalg.norm(
                        agent.pos[:2] - o.pos[:2])) < 0.85:
                    near = True
                    break
            if not near:
                self._row_clamp_ref(agent, v_world_x, v_world_y)
                agent.cbf_v_prev = np.array([v_world_x, v_world_y], dtype=float)
                self._cbf_stats['ticks'] += 1
                self._cbf_stats['tier'][0] += 1
                return v_world_x, v_world_y

        agents = self._cbf_snapshot()
        if did not in agents:
            return v_world_x, v_world_y

        # Titik tahan yang DIMAKSUD state machine, direkam sebelum QP menimpanya.
        hold_anchor = (np.array(agent.ref_pos, dtype=float)
                       if agent.state in self.POS_HOLD_STATES else None)

        task = self._CT.Task(v_nom=np.array([v_world_x, v_world_y], dtype=float))
        res = self.cbf.solve(did, agents, task, self._cbf_dt, t_now=self._cbf_now)

        # Aturan ref_pos TUNGGAL. Di kode lama enam tempat berbeda
        # me-teleport ref_pos, masing-masing bertarung dengan perintah
        # kecepatan lewat position loop low-level.
        ref = res.ref_pos.astype(float)

        # State penahan: jangkarkan ke titik yang dimaksud, bukan ke posisi drone.
        # v_safe tetap ditambahkan supaya manuver menghindar dari QP tetap lolos.
        if hold_anchor is not None:
            ref = hold_anchor + self.cbf_plant.T_lead * res.v_safe

        # CLAMP UJUNG BARIS. Saat menyapu, komponen ref_pos SEPANJANG garis
        # sapuan dikunci ke [0, line_len] — inilah yang mengembalikan jaminan
        # "tidak ada overshoot" (klaim di docstring). Komponen LATERAL (koreksi
        # cross-track + manuver V2V dari QP) dibiarkan utuh: CBF tetap bekerja
        # untuk V2V, hanya overshoot longitudinal yang dimatikan.
        seg = getattr(agent, '_row_seg', None)
        if seg is not None and agent.state == 'sweeping_row':
            wp0, u_line, line_len = seg
            d = ref - wp0
            s_long = float(np.dot(d, u_line))
            perp = d - s_long * u_line
            s_long = min(max(s_long, 0.0), line_len)
            ref = wp0 + s_long * u_line + perp

        agent.ref_pos = ref.astype(np.float32)
        agent.cbf_v_prev = res.v_safe
        agents[did].v_prev_cmd = res.v_safe

        st = self._cbf_stats
        st['ticks'] += 1
        st['tier'][res.tier] += 1
        if np.isfinite(res.slack):
            st['max_slack'] = max(st['max_slack'], res.slack)
        if res.h_min < agent.min_dist_to_obs:
            agent.min_dist_to_obs = max(0.0, res.h_min)

        if res.tier >= 2:
            self.get_logger().warning(
                f'  ⚠️  [iris_{did}] QP Tier {res.tier} (slack={res.slack:.3f}, '
                f'pembatas={res.limiting}, h_min={res.h_min:.2f}m)',
                throttle_duration_sec=1.0)

        return float(res.v_safe[0]), float(res.v_safe[1])

    def cbf_summary(self):
        st = self._cbf_stats
        n = max(1, st['ticks'])
        return (f"CBF-QP: {st['ticks']} solve | "
                f"T0 {st['tier'][0]} T1 {st['tier'][1]} "
                f"T2 {st['tier'][2]} T3 {st['tier'][3]} | "
                f"P(tier>0)={100.0*(n-st['tier'][0])/n:.3f}% | "
                f"slack_maks={st['max_slack']:.4f}")

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

        # Kecepatan dunia; QP memerlukannya untuk deteksi macet. Gazebo
        # OdometryPublisher melaporkan twist di frame bodi, jadi diputar.
        vb_x = msg.twist.twist.linear.x
        vb_y = msg.twist.twist.linear.y
        cy, sy = math.cos(agent.yaw), math.sin(agent.yaw)
        agent.vel_world = np.array([vb_x * cy - vb_y * sy,
                                    vb_x * sy + vb_y * cy], dtype=float)

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

    def _push_centroid_clear(self, ctr):
        """Geser titik parkir keluar dari zona aman rintangan mana pun.

        Dipakai hanya untuk centroid (tempat hinggap setelah pemetaan tuntas),
        bukan untuk jalur sapuan — cakupan sudah selesai saat drone pulang,
        jadi parkir bergeser satu-dua meter tidak berbiaya apa pun.
        """
        if not self.enable_obstacles:
            return ctr
        p = np.asarray(ctr, dtype=float)[:2].copy()
        for _oid, ox, oy, rad, _h, _c in self.static_obstacles:
            c = np.array([ox, oy], dtype=float)
            d = p - c
            r = float(np.linalg.norm(d))
            if r < OBSTACLE_KEEP_OUT:
                u = (np.array([1.0, 0.0]) if r < 1e-6 else d / r)
                p = c + OBSTACLE_KEEP_OUT * u
        return p.astype(np.float32)

    def _log_crash_forensics(self, did, agent, obs_id, obs_xy, d_center):
        """Keadaan lengkap saat benturan — memisahkan dua hipotesis.

        H1  silinder BELUM terkonfirmasi saat benturan -> peta terlalu lambat.
        H2  silinder SUDAH di peta tapi QP tetap meloloskan -> masalah di
            perakitan constraint, bukan di persepsi.

        Tanpa ini keduanya terlihat sama dari luar, dan dua tebakan sebelumnya
        sudah terbukti meleset. Jangan memperbaiki apa pun tanpa membaca ini.
        """
        if not self.use_lidar_obstacles:
            return
        # Jalur peta terdekat ke silinder yang tertabrak
        best, bd = None, float('inf')
        for oid, mx, my, mr, _h, _c in self.obs_map.confirmed():
            d = float(np.linalg.norm(np.array([mx, my]) - obs_xy))
            if d < bd:
                best, bd = (oid, mx, my, mr), d
        tot, conf, moving = self.obs_map.n_tracks()

        if best is not None and bd < 1.0:
            hyp = (f'H2 — SUDAH DIPETAKAN sebagai #{best[0]} '
                   f'({best[1]:.2f}, {best[2]:.2f}) r={best[3]:.2f}, '
                   f'meleset {bd:.2f} m dari posisi asli')
        else:
            hyp = (f'H1 — TIDAK ADA di peta (jalur terdekat {bd:.2f} m); '
                   'QP tidak pernah tahu silinder ini ada')

        v = np.asarray(agent.vel[:2] if hasattr(agent, 'vel') else [0.0, 0.0],
                       dtype=float)
        n_hat = (obs_xy - agent.pos[:2])
        nn = float(np.linalg.norm(n_hat))
        closing = float(n_hat @ v / nn) if nn > 1e-6 else 0.0
        st = self._cbf_stats
        self.get_logger().error(
            f'   🔬 [FORENSIK iris_{did}] {hyp}\n'
            f'      peta saat benturan : {conf}/{tot} terkonfirmasi '
            f'({moving} ditolak karena bergerak)\n'
            f'      state / laju       : {agent.state} | |v|={float(np.linalg.norm(v)):.2f} m/s '
            f'| mendekat {closing:+.2f} m/s\n'
            f'      h ke silinder ini  : {d_center - 0.62:+.3f} m\n'
            f'      tier QP kumulatif  : T0={st["tier"][0]} T1={st["tier"][1]} '
            f'T2={st["tier"][2]} T3={st["tier"][3]}')

    def _obstacles_near_cell(self, cell_poly):
        """Rintangan yang zona amannya menyentuh sel tersapu ``cell_poly``.

        BUKAN kepemilikan eksklusif. Kode lama memakai `contains_point` atas
        sel mentah, sehingga satu rintangan hanya dimiliki satu drone —
        padahal zona aman 1.30 m kerap menyeberangi batas sel, dan drone
        tetangga menyapu masuk ke sana TANPA merencanakan jalan memutar. Itu
        terjadi di keempat preset wilayah (mis. #102 di `rect`: pemilik sel 6,
        disapu sel 1 dan 6). Sebuah rintangan boleh dimiliki lebih dari satu
        drone — memang itu yang benar.

        Pada Skema 1/2 daftar ini WAJIB kosong: kalau tidak, boustrophedon
        memecah baris di koordinat rintangan yang tidak ada di Gazebo dan
        meninggalkan celah cakupan palsu.
        """
        if not self.enable_obstacles or len(cell_poly) < 3:
            return []
        try:
            poly = SpPolygon([(float(p[0]), float(p[1])) for p in cell_poly])
            if not poly.is_valid:
                poly = poly.buffer(0)
        except Exception:
            return list(self.static_obstacles)
        return [obs for obs in self.static_obstacles
                if poly.intersects(ShapelyPoint(obs[1], obs[2]).buffer(OBSTACLE_KEEP_OUT))]

    def plan_centroidal_voronoi(self):
        """Menjalankan 25x Lloyd's Relaxation & Hungarian Assignment untuk 7 Drone."""
        self.get_logger().info('📐 Menjalankan Centroidal Voronoi (25x Lloyd) + Hungarian Assignment...')

        # Generator awal: 7 titik DI DALAM wilayah, tersebar via k-means++
        # deterministik. Menggantikan 7 titik hardcoded yang bisa jatuh di
        # luar wilayah non-convex.
        from swarm_high_level.world.region import region_seed_points
        seed_points = region_seed_points(self.region_poly, 7, seed=7)
        gens = {i + 1: seed_points[i].copy() for i in range(7)}

        # 25 Putaran Lloyd's Relaxation. Tiap iterasi: sel = (Voronoi ∩ wilayah),
        # centroid = titik berat sel itu (bukan sel Voronoi tak-terpotong).
        for _ in range(25):
            new_gens = {}
            for i in gens:
                cell_tmp = [np.array(v, dtype=float) for v in self.bbox_rect]
                for j in gens:
                    if j != i:
                        cell_tmp = clip_voronoi(cell_tmp, gens[i], gens[j])
                ring, ctr = clip_poly_to_region(cell_tmp, self.region_poly)
                new_gens[i] = ctr if len(ring) >= 3 else gens[i]
            gens = new_gens

        # ── Penugasan sel: Hungarian DUA TAHAP ───────────────────────────
        # Tahap 1 memakai centroid sel (perkiraan kasar). Tahap 2 memakai TITIK
        # MULAI SAPUAN yang sebenarnya — itulah yang benar-benar diterbangi
        # drone saat transit, dan bisa jauh dari centroid pada sel memanjang.
        drone_positions = np.array([self.agents[did].pos[:2] for did in range(1, 8)])
        gen_list = [gens[k] for k in range(1, 8)]

        def _assign(targets):
            tgt = np.asarray(targets, dtype=float)
            cost = np.linalg.norm(drone_positions[:, None, :] - tgt[None, :, :], axis=2)
            r, c = linear_sum_assignment(cost)
            return {int(ri) + 1: int(ci) for ri, ci in zip(r, c)}

        def _cell_of(gi, owners):
            """Sel bermargin untuk generator ke-gi, diberi peta pemilik saat ini."""
            cell = [np.array(v, dtype=float) for v in self.bbox_rect]
            for gj in range(7):
                if gj != gi:
                    cell = clip_voronoi_margin(
                        cell, gen_list[gi], gen_list[gj], margin=0.35)
            cell, _ctr = clip_poly_to_region(cell, self.region_poly)
            return cell

        # Tahap 1 — dekat centroid.
        owner = _assign(gen_list)

        # Titik mulai sapuan tiap sel, dilihat dari drone yang ditugaskan tahap 1.
        starts = []
        for gi in range(7):
            cell_i = _cell_of(gi, owner)
            did_i = next((d for d, g in owner.items() if g == gi), 1)
            wp = generate_boustrophedon(
                cell_i, sweep_spacing=1.45, margin=0.02,
                entry_point=self.agents[did_i].pos[:2])
            starts.append(np.asarray(wp[0], dtype=float))

        # Tahap 2 — dekat titik MULAI yang sebenarnya.
        owner = _assign(starts)

        drone_to_gen = {did: gen_list[gi] for did, gi in owner.items()}

        # Bentuk sel poligon dengan margin bisector normal 0.45m & rute Boustrophedon
        for did, agent in self.agents.items():
            pi = drone_to_gen[did]
            cell = [np.array(v, dtype=float) for v in self.bbox_rect]
            raw_cell = [np.array(v, dtype=float) for v in self.bbox_rect]
            for other_did, pj in drone_to_gen.items():
                if other_did != did:
                    cell = clip_voronoi_margin(cell, pi, pj, margin=0.35)
                    raw_cell = clip_voronoi(raw_cell, pi, pj)

            # Potong ke batas wilayah non-convex (no-op bila wilayah = persegi).
            cell, cell_ctr = clip_poly_to_region(cell, self.region_poly)
            raw_cell, _ = clip_poly_to_region(raw_cell, self.region_poly)

            agent.cell_polygon = cell
            agent.raw_cell_polygon = raw_cell
            # Titik parkir TIDAK BOLEH berada di dalam rintangan. `return_to_centroid`
            # menembak lurus ke centroid tanpa perencanaan rintangan — hanya QP
            # yang menahannya — jadi centroid yang jatuh di atas silinder membuat
            # drone mendorong terus melawan CBF sampai merayap masuk. Terukur
            # 31 Agu di u_shape: centroid iris_2 hanya 0.24 m dan iris_7 0.40 m
            # dari pusat silinder (radius tabrakan fisik 0.62 m) — keduanya
            # MENABRAK dan mati. Geser radial keluar sampai zona aman.
            agent.centroid = self._push_centroid_clear(cell_ctr)

            # Rintangan hanya relevan bila skema mengaktifkannya. Pada Skema
            # 1/2 daftar ini HARUS kosong — kalau tidak, boustrophedon memangkas
            # baris di sekitar koordinat rintangan yang tidak ada di Gazebo dan
            # meninggalkan celah cakupan palsu.
            agent.my_static_obstacles = self._obstacles_near_cell(cell)

            # Sapuan SELALU bawah→atas; entry_point membuat drone masuk lewat
            # ujung rantai tepi bawah yang TERDEKAT dengan posisinya, jadi tidak
            # memutar ke sisi jauh sel.
            agent.waypoints = generate_boustrophedon(
                cell, sweep_spacing=1.45, margin=0.02,
                entry_point=agent.pos[:2], obstacles=agent.my_static_obstacles)
            agent.num_rows = max(1, len(agent.waypoints) // 2)
            agent.row_idx = 0
            agent.ref_pos = agent.waypoints[0].copy()

        # ── Deconflict titik mulai (pencarian kombinatorial) ─────────────
        # Sel yang bersebelahan bisa punya pojok bawah yang berdekatan, jadi
        # "ujung terdekat" untuk dua drone bisa jatuh nyaris di titik yang sama
        # (terukur 0.70 m — persis di batas keras V2V). Tiap drone punya DUA
        # kandidat masuk: ujung dekat / ujung jauh rantai tepi bawah. 2^7 = 128
        # kombinasi, cukup dievaluasi seluruhnya.
        #
        # Skor: maksimalkan jarak minimum antar titik-mulai (dibatasi di
        # SAFE_START — lebih dari itu tidak berguna), lalu minimalkan total
        # transit. Loop flip lama gagal karena prefer_far bukan toggle:
        # membaliknya dua kali menghasilkan rute yang sama.
        SAFE_START = 2.5
        cand = {}
        for did, agent in self.agents.items():
            cand[did] = [
                generate_boustrophedon(
                    agent.cell_polygon, sweep_spacing=1.45, margin=0.02,
                    entry_point=agent.pos[:2], prefer_far=far,
                    obstacles=agent.my_static_obstacles)
                for far in (False, True)
            ]

        def _score(combo):
            starts = [np.asarray(cand[d][combo[d - 1]][0], dtype=float)
                      for d in range(1, 8)]
            d_min = min(float(np.linalg.norm(starts[a] - starts[b]))
                        for a in range(7) for b in range(a + 1, 7))
            transit = sum(float(np.linalg.norm(
                self.agents[d].pos[:2] - starts[d - 1])) for d in range(1, 8))
            return (min(d_min, SAFE_START), -transit), d_min

        best_combo, best_key, best_dmin = None, None, 0.0
        for combo in itertools.product((0, 1), repeat=7):
            key, d_min = _score(combo)
            if best_key is None or key > best_key:
                best_combo, best_key, best_dmin = combo, key, d_min

        for did, agent in self.agents.items():
            agent.waypoints = cand[did][best_combo[did - 1]]
            agent.num_rows = max(1, len(agent.waypoints) // 2)
            agent.row_idx = 0
            agent.ref_pos = agent.waypoints[0].copy()

        n_flip = sum(best_combo)
        self.get_logger().info(
            f'  🔀 Deconflict titik mulai: jarak minimum {best_dmin:.2f} m '
            f'({n_flip} drone masuk lewat ujung jauh)')
        if best_dmin < 1.20:
            self.get_logger().warning(
                f'  ⚠️  Titik mulai terdekat hanya {best_dmin:.2f} m — '
                'CBF V2V akan menanganinya saat wait_all_start.')

        for did, agent in self.agents.items():
            agent.wp_flags = [True] * agent.num_rows
            agent.own_num_rows = agent.num_rows
            agent.own_waypoints = list(agent.waypoints)
            dist_to_start = float(np.linalg.norm(agent.pos[:2] - agent.waypoints[0]))

            agent.my_static_obstacles = self._obstacles_near_cell(agent.cell_polygon)
            obs_ids = [o[0] for o in agent.my_static_obstacles]

            # Rancang koridor transit aman bebas dari seluruh rintangan statis di arena
            p_start = agent.waypoints[0].copy()
            p_stage = agent.pos[:2].copy()
            u_tr = p_start - p_stage
            dist_tr = float(np.linalg.norm(u_tr))
            u_tr_hat = u_tr / max(1e-3, dist_tr)

            needs_intermediate = False
            if self.enable_obstacles:
                for obs in self.static_obstacles:
                    obs_center = np.array([obs[1], obs[2]], dtype=float)
                    r_obs = obs_center - p_stage
                    s_proj = float(np.dot(r_obs, u_tr_hat))
                    if 0.5 < s_proj < (dist_tr - 0.5):
                        p_proj = p_stage + s_proj * u_tr_hat
                        if np.linalg.norm(obs_center - p_proj) < (obs[3] + 0.65):
                            needs_intermediate = True
                            break

            # Koridor transit dengan titik antara HANYA bila ada rintangan yang
            # harus dihindari (Skema 3/4). Pada Skema 1/2 titik antara hardcoded
            # itu tak perlu dan bisa jatuh di luar wilayah non-convex.
            if needs_intermediate and p_start[1] > -5.0:
                if p_start[0] < -4.0:
                    mid = np.array([-14.00, -8.00], dtype=np.float32)
                elif p_start[0] > 4.0:
                    mid = np.array([10.00, -8.00], dtype=np.float32)
                else:
                    mid = np.array([p_stage[0], -14.00], dtype=np.float32)
                agent.transit_waypoints = [mid, p_start.copy()]
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
        """Scan LiDAR -> deteksi rintangan -> peta bersama kawanan.

        Sebelumnya callback ini hanya MENYIMPAN `lidar_ranges`, dan tidak ada
        satu pun kode yang membacanya: LiDAR-nya mati total sementara rintangan
        diambil dari tabel koordinat yang sudah diketahui. Sekarang rintangan
        DITEMUKAN di sini, dan perencana maupun QP memakai hasil temuan itu.

        Peta sengaja BERSAMA untuk seluruh kawanan: itu asumsi yang wajar pada
        swarm yang saling berkomunikasi, dan membuat peta terbentuk jauh lebih
        cepat daripada tiap drone memetakan sendiri-sendiri.
        """
        if did not in self.agents:
            return
        agent = self.agents[did]
        agent.lidar_ranges = np.array(msg.ranges, dtype=np.float32)
        if not self.use_lidar_obstacles or agent.pos[2] < 1.0:
            return
        # TUTUPI SETIAP drone lain, tanpa syarat ketinggian. Versi pertama
        # hanya menutupi yang di atas 0.8 m, sehingga drone yang sedang lepas
        # landas atau transit rendah terdeteksi sebagai silinder — puluhan
        # rintangan hantu menumpuk di tepi bawah dekat landasan. Menutupi drone
        # yang kebetulan tidak terlihat tidak merugikan apa pun; melewatkannya
        # merusak peta.
        others = [self.agents[o].pos[:2] for o in self.agents if o != did]
        det = self._detect_obstacles(
            agent.lidar_ranges, float(msg.angle_min), float(msg.angle_increment),
            float(agent.pos[0]), float(agent.pos[1]), float(agent.yaw),
            others=others,
            arena=(self.x_min, self.y_min, self.x_max, self.y_max))
        self.obs_map.update(det)

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
        if not self.enable_dynamic_obstacles:
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


    def get_coverage_percentage(self):
        """Persentase cakupan di dalam wilayah pemetaan (poligon, bisa non-convex)."""
        valid = self.region_mask.copy()
        if self.enable_obstacles and hasattr(self, 'obstacle_mask'):
            valid &= ~self.obstacle_mask
        n_valid = int(np.sum(valid))
        if n_valid == 0:
            return 0.0
        return float(np.sum(self.cov_grid & valid) / n_valid * 100.0)

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
        rec_obs = self.static_obstacles if self.enable_obstacles else None
        for comp in comp_polys:
            boust_wps = generate_boustrophedon(comp, sweep_spacing=1.45, margin=0.20, start_from_top=False, obstacles=rec_obs)
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

        # Perbarui rintangan statis yang relevan untuk setiap drone hidup
        # (hanya bila skema mengaktifkan rintangan).
        for did in alive_ids:
            ag = self.agents[did]
            relevant_obs = []
            if self.enable_obstacles:
                for obs in self.static_obstacles:
                    obs_c = np.array([obs[1], obs[2]], dtype=float)
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

        # Peta LiDAR menyuapi QP saja — TIDAK menyentuh perencana.
        #
        # Perencanaan adalah urusan high level dan hanya berjalan sekali; ia
        # menghasilkan boustrophedon polos, persis seperti Skema 1/2, karena
        # peta masih kosong saat lepas landas. Penghindaran sepenuhnya urusan
        # mid level: "state machine mengusulkan, QP memutuskan".
        #
        # Versi sebelumnya melanggar pembagian itu dengan merencanakan ulang
        # baris setiap kali peta berubah. Akibatnya perencana dan QP mengejar
        # peta yang sama pada laju berbeda: 222 lalu 161 kali rencana ulang
        # dalam satu misi, cakupan runtuh ke 57.8% lalu 40.4%, dan drone tak
        # pernah maju. Jangan kembalikan.
        #
        # Penyegaran tiap tik tetap WAJIB: sebuah jalur menjadi terkonfirmasi
        # pada hit ke-4 tanpa jalur baru muncul, jadi menyegarkan hanya saat
        # ada jalur baru membuat QP tidak pernah diberi tahu.
        if self.use_lidar_obstacles:
            self.static_obstacles = self.obs_map.confirmed()

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
                        # PENILAIAN: tabrakan diukur terhadap silinder yang
                        # BENAR-BENAR ada di Gazebo, bukan terhadap apa yang
                        # kebetulan sudah ditemukan LiDAR — kalau tidak, drone
                        # yang menabrak rintangan tak-terdeteksi akan lolos.
                        for obs in self.truth_obstacles:
                            obs_id, ox, oy, rad, _, _ = obs
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
                                self._log_crash_forensics(did, agent, obs_id,
                                                          np.array([ox, oy]), d_center)
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

                agent.ref_pos = agent.pos[:2].copy()
                wz_cmd = self.compute_wz(agent.yaw, target_yaw)
                self.publish_twist(did, 0.0, 0.0, wz_cmd)
                agent.delay_timer = getattr(agent, 'delay_timer', 0) + 1

                yaw_diff = abs(math.atan2(math.sin(target_yaw - agent.yaw), math.cos(target_yaw - agent.yaw)))
                if (agent.delay_timer >= 20 and yaw_diff < math.radians(4.0)) or agent.delay_timer >= 80:
                    agent.state = 'transit_to_start'
                    agent.delay_timer = 0
                    agent.transit_ticks = 0
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

                # PENGAMAN ANTI-DEADLOCK. `wait_all_start` menunggu SELURUH
                # drone, jadi satu drone yang tak pernah menyentuh bola 0.35 m
                # menggantung seluruh kawanan selamanya. Terukur pada sweep
                # Skema 3 (31 Agu): iris_5 di l_shape berhenti di 0.452 m dan
                # iris_3 di plus di 0.745 m — keduanya mengorbit target selama
                # 750 s sim, cakupan 0.0%, sementara enam drone lain tiba di
                # 0.000-0.003 m. Setelah TRANSIT_STUCK_TICKS (60 s pada 20 Hz)
                # jarak "cukup dekat" diterima, dengan peringatan, supaya
                # kegagalan satu drone tidak lagi membatalkan misi.
                agent.transit_ticks = getattr(agent, 'transit_ticks', 0) + 1
                stuck = (is_final_transit_wp
                         and agent.transit_ticks > self.TRANSIT_STUCK_TICKS
                         and dist_to_wp < self.TRANSIT_STUCK_ACCEPT)
                if stuck:
                    self.get_logger().warning(
                        f'  ⚠️  [iris_{did}] tidak konvergen ke titik start dalam '
                        f'{agent.transit_ticks * 0.05:.0f}s (sisa {dist_to_wp:.2f}m > '
                        f'{thresh:.2f}m) — diterima agar kawanan tidak menggantung.')

                if dist_to_wp < thresh or stuck:
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

                # Lantai kecepatan 0.40 m/s membuat drone TIDAK BISA berhenti
                # di target: `_cbf_filter` menulis ref_pos = pos + T_lead*v_safe,
                # jadi acuan selalu >= 0.33*0.40 = 0.13 m di depan drone dan ia
                # mengorbit alih-alih mendarat di titiknya. Lantai itu berguna
                # agar tidak merayap di transit jauh, jadi hanya diturunkan pada
                # pendekatan akhir ke waypoint terakhir.
                v_floor = 0.15 if (is_final_transit_wp and dist_to_wp < 1.5) else 0.40
                v_mag = min(self.transit_speed, max(v_floor, 2.0 * dist_to_wp))
                v_world_x = v_mag * math.cos(angle_to_wp) + np.clip(1.2 * dx, -0.40, 0.40)
                v_world_y = v_mag * math.sin(angle_to_wp) + np.clip(1.2 * dy, -0.40, 0.40)

                # Full V2V avoidance during transit


                self.send_world_twist(did, v_world_x, v_world_y, angle_to_wp)

            # ─────────────────────────────────────────────────────────
            # 1c. WAIT ALL START (Sinkronisasi Start Bersama Antar Kawanan)
            # ─────────────────────────────────────────────────────────
            elif agent.state == 'wait_all_start':
                start_wp = np.array(agent.waypoints[0], dtype=np.float32)
                # Menahan posisi di titik start; QP di send_world_twist yang
                # menambahkan manuver menghindar bila ada yang mendekat.
                v_x, v_y = 0.0, 0.0
                agent.ref_pos = start_wp.copy()
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
                    agent.return_ticks = 0
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

                # Segmen baris untuk clamp ujung di _cbf_filter (anti-overshoot).
                agent._row_seg = (np.array(wp_start, dtype=float), u_line, line_len)

                # Progres AKTUAL drone fisik di sepanjang garis sapuan
                actual_prog = max(0.0, min(line_len, float(np.dot(pos_rel, u_line))))
                e_lat = float(pos_rel[1] * u_line[0] - pos_rel[0] * u_line[1])

                # Referensi bergerak terikat progres nyata drone. _cbf_filter
                # menimpanya dengan pos + T_lead*v_safe LALU meng-clamp komponen
                # longitudinal ke [0, line_len] memakai agent._row_seg.
                s_target = min(line_len, actual_prog + self.lead_dist)
                agent.ref_pos = (wp_start + s_target * u_line)

                # Catat telemetri tracking
                agent.max_cross_track_err = max(agent.max_cross_track_err, abs(e_lat))
                agent.cross_track_errors.append(abs(e_lat))
                agent.altitude_errors.append(abs(float(agent.pos[2]) - self.cruise_alt))
                yaw_diff = abs(math.atan2(math.sin(agent.yaw - agent.target_yaw), math.cos(agent.yaw - agent.target_yaw)))
                agent.yaw_errors.append(math.degrees(yaw_diff))

                dist_to_end = line_len - actual_prog
                dist_to_end_pt = float(np.linalg.norm(agent.pos[:2] - wp_end))

                # Ketercapaian ujung baris. Toleransi RELATIF: baris pendek
                # (segmen rantai tepi) tidak boleh dipotong separuh oleh
                # ambang absolut — dulu 0.25 m memotong baris 0.5 m di tengah.
                end_tol = max(0.10, min(0.22, 0.28 * line_len))
                if actual_prog > 0.50 * line_len and (dist_to_end <= end_tol or dist_to_end_pt < end_tol + 0.02):
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

                    if agent.row_idx + 1 >= agent.num_rows:
                        agent.state = 'return_to_centroid'
                        agent.return_ticks = 0
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

                # RAMP-DOWN feedforward pada brake_dist terakhir. Jarak henti
                # pada 1.6 m/s ~0.60 m; default brake_dist 1.0 m memberi margin.
                bd = self.brake_dist
                if dist_to_end > bd:
                    ff_scale = 1.0
                elif dist_to_end > 0.10:
                    ff_scale = dist_to_end / bd
                else:
                    ff_scale = 0.0

                # Kecepatan yang DIINGINKAN sepanjang garis sapuan. Perlambatan
                # karena rintangan tidak lagi dihitung di sini: QP yang
                # memodulasinya lewat constraint, secara kontinu — bukan
                # pengali biner 0/1 seperti pendekatan lama.
                v_ff = (self.nominal_speed * ff_scale) * u_line

                # Koreksi lateral cross-track.
                # Koreksi cross-track. Batas dinaikkan 0.45 -> ct_corr_max
                # (default 0.90 m/s) supaya drone benar-benar menahan garis saat
                # berangin: angin muncul sebagai roll yang lebih besar (usaha
                # kendali), bukan sebagai lintasan yang melengkung.
                v_corr_lat = -np.clip(self.kp_track * e_lat,
                                      -self.ct_corr_max, self.ct_corr_max) \
                    * np.array([-u_line[1], u_line[0]])

                v_world = v_ff + v_corr_lat
                v_world_x = float(v_world[0])
                v_world_y = float(v_world[1])

                self.send_world_twist(did, v_world_x, v_world_y, agent.yaw)

            elif agent.state == 'delay_at_corner_end':
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
                next_end = agent.waypoints[(agent.row_idx + 1) * 2 + 1]
                step_vec = next_start - agent.pos[:2]
                target_yaw = math.atan2(step_vec[1], step_vec[0])
                agent.target_yaw = target_yaw
                agent.ref_pos = agent.pos[:2].copy()

                wz_cmd = self.compute_wz(agent.yaw, target_yaw)
                self.publish_twist(did, 0.0, 0.0, wz_cmd)
                agent.delay_timer = getattr(agent, 'delay_timer', 0) + 1

                # Segmen pendek (cap/connector pelacak-tepi) → pivot cepat, tak
                # perlu settle penuh: geometri toh sudah menyusuri tepi sel.
                seg_short = (float(np.linalg.norm(next_end - next_start)) < 2.5
                             and float(np.linalg.norm(step_vec)) < 2.5)
                settle_ticks = 6 if seg_short else 25
                yaw_tol = math.radians(15.0 if seg_short else 6.0)
                yaw_diff = abs(math.atan2(math.sin(target_yaw - agent.yaw), math.cos(target_yaw - agent.yaw)))
                if not has_yield_conflict and ((agent.delay_timer >= settle_ticks and yaw_diff < yaw_tol) or agent.delay_timer >= 120):
                    agent.state = 'stepping_vertical'
                    agent.step_timer = 0
                    agent.delay_timer = 0

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

                wz_cmd = self.compute_wz(agent.yaw, target_yaw)
                self.publish_twist(did, 0.0, 0.0, wz_cmd)
                agent.delay_timer = getattr(agent, 'delay_timer', 0) + 1

                seg_short = float(np.linalg.norm(line_dir)) < 2.5
                settle_ticks = 6 if seg_short else 25
                yaw_tol = math.radians(15.0 if seg_short else 6.0)
                yaw_diff = abs(math.atan2(math.sin(target_yaw - agent.yaw), math.cos(target_yaw - agent.yaw)))
                if (agent.delay_timer >= settle_ticks and yaw_diff < yaw_tol) or agent.delay_timer >= 120:
                    agent.row_idx += 1
                    agent.state = 'sweeping_row'
                    agent.delay_timer = 0
                    is_rec = (agent.row_idx < len(agent.wp_flags)) and (not agent.wp_flags[agent.row_idx])
                    tag_type = "RECOVERY" if is_rec else "SEL ASLI"
                    self.get_logger().info(f'  🚀 [iris_{did}] Memulai Baris {agent.row_idx+1}/{agent.num_rows} [{tag_type}] (Heading: {math.degrees(agent.yaw):.1f}°)')

            # ─────────────────────────────────────────────────────────
            # 7. RETURN TO CENTROID (Kembali ke Titik Pusat Sel Voronoi)
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

                # Pengaman yang sama seperti `transit_to_start`: acuan adalah
                # carrot di depan drone dan `v_back` berlantai 0.40 m/s, jadi
                # drone bisa mengorbit centroid tanpa pernah menyentuh bola
                # 0.30 m. Karena `exit_after_success` menunggu SELURUH drone
                # berstatus `done`, satu drone yang mengorbit membuat misi tak
                # pernah selesai: 4 dari 6 misi sweep 31 Agu habis di timeout
                # 2400 s meski cakupan sudah 99.5-99.6% (i2/i7 tetap `return`).
                agent.return_ticks = getattr(agent, 'return_ticks', 0) + 1
                stuck_c = (agent.return_ticks > self.TRANSIT_STUCK_TICKS
                           and dist_to_c < self.RETURN_STUCK_ACCEPT)
                if stuck_c:
                    self.get_logger().warning(
                        f'  ⚠️  [iris_{did}] tidak konvergen ke centroid dalam '
                        f'{agent.return_ticks * 0.05:.0f}s (sisa {dist_to_c:.2f}m) — '
                        'diterima sebagai selesai. Parkir, bukan pemetaan.')

                if dist_to_c < 0.30 or stuck_c:
                    agent.state = 'done'
                    agent.ref_pos = agent.centroid.copy()
                    agent.target_yaw = math.pi / 2.0  # Menghadap UTARA (+90.0°)
                    wz_cmd = self.compute_wz(agent.yaw, agent.target_yaw)
                    self.publish_twist(did, 0.0, 0.0, wz_cmd)
                    self.get_logger().info(f'  🎯 [iris_{did}] Tiba di Pusat Sel Voronoi ({agent.centroid[0]:.2f}, {agent.centroid[1]:.2f})! Menghadap UTARA (+90.0°).')
                    continue

                v_back_floor = 0.15 if dist_to_c < 1.5 else 0.40
                v_back = min(self.transit_speed, max(v_back_floor, 2.0 * dist_to_c))
                v_x = v_back * math.cos(ang_c) + np.clip(1.2 * dx, -0.40, 0.40)
                v_y = v_back * math.sin(ang_c) + np.clip(1.2 * dy, -0.40, 0.40)

                # V2V repulsion aktif di perjalanan pulang


                self.send_world_twist(did, v_x, v_y, agent.yaw)

            # ─────────────────────────────────────────────────────────
            # 8. DONE (Hover di Centroid dengan Penghindaran Aktif)
            # ─────────────────────────────────────────────────────────
            elif agent.state == 'done':
                agent.target_yaw = math.pi / 2.0  # Menghadap UTARA (+90.0°)
                wz_cmd = self.compute_wz(agent.yaw, agent.target_yaw)
                v_x, v_y = 0.0, 0.0
                # Mengunci lembut di pusat sel. Bila ada rintangan melintas,
                # QP yang menambahkan manuver menghindar di send_world_twist.
                if True:
                    agent.ref_pos = agent.centroid.copy()
                    dx_c = float(agent.centroid[0] - agent.pos[0])
                    dy_c = float(agent.centroid[1] - agent.pos[1])
                    v_x = float(np.clip(1.5 * dx_c, -0.60, 0.60))
                    v_y = float(np.clip(1.5 * dy_c, -0.60, 0.60))
                self.send_world_twist(did, v_x, v_y, agent.yaw)

        # Telemetri Terminal setiap 1 detik (20 ticks @ 20Hz)
        if self.step_count % 20 == 0:
            cov = self.get_coverage_percentage()
            states_summary = ' | '.join(f'i{did}:{a.state[:6]}' for did, a in self.agents.items())
            map_txt = ''
            if self.use_lidar_obstacles:
                tot, conf, moving = self.obs_map.n_tracks()
                map_txt = f' | peta {conf}/{tot} (gerak {moving})'
                # Tiap 15 detik: DI MANA jalur-jalur itu. Jumlah saja tidak
                # cukup untuk tahu benda apa yang sebenarnya terdeteksi.
                if self.step_count % 300 == 0 and tot:
                    rows = ' '.join(
                        f'({x:+.1f},{y:+.1f})r{r:.2f}n{n}d{dr:.1f}{"M" if mv else ""}'
                        for x, y, r, n, dr, mv in self.obs_map.dump())
                    self.get_logger().info(f'  🗺️  [PETA] {rows}')
            self.get_logger().info(
                f'📊 [STATUS] Cov: {cov:5.1f}% | d_min: {self.global_min_dist:4.2f}m'
                f'{map_txt} | {states_summary}'
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

                # Clearance rintangan terukur — dihitung sejak dulu di
                # agent.min_dist_to_obs tetapi tidak pernah dilaporkan.
                if self.enable_obstacles:
                    d_obs = min((a.min_dist_to_obs for a in self.agents.values()
                                 if a.is_alive), default=float('inf'))
                    self.get_logger().info(
                        f'  🚧 Clearance rintangan minimum (semua drone): {d_obs:.3f} m')
                # Ringkasan CBF SELALU dicetak (juga Skema 1/2) supaya
                # P(tier>0) bisa dilaporkan apa adanya — Tier>0 di Skema 1/2
                # murni dari relaksasi V2V-soft.
                self.get_logger().info(f'  🛡️  {self.cbf_summary()}')
                self.get_logger().info('====================================================================================================')

            # ── Auto-exit setelah misi benar-benar tuntas ────────────────
            # Syarat: SELURUH drone hidup berstatus 'done' (bukan sekadar
            # coverage >= 97%), lalu diam exit_after_success detik-sim supaya
            # CSV low-level sempat ter-flush. Tanpa ini proses hidup selamanya
            # dan skrip pemanggil harus menunggu timeout penuh.
            if self.exit_after_success > 0.0:
                alive_now = [a for a in self.agents.values()
                             if a.is_alive and a.state != 'dead']
                if alive_now and all(a.state == 'done' for a in alive_now):
                    if self._done_step is None:
                        self._done_step = self.step_count
                        self.get_logger().info(
                            f'  ⏳ Semua drone di centroid — keluar dalam '
                            f'{self.exit_after_success:.0f}s.')
                elif self._done_step is not None:
                    self._done_step = None      # ada yang kembali bertugas

                if (self._done_step is not None
                        and (self.step_count - self._done_step)
                        >= self.exit_after_success * 20.0):
                    self.get_logger().info(
                        f'🛑 [AUTO-EXIT] Misi tuntas & mengendap. '
                        f'Cov akhir {cov:.1f}%. Node berhenti.')
                    raise SystemExit(0)

    # ── Gaya Tolak V2V (Hanya Aktif Saat Transit, Wait, & Done Yield) ──


    # ── Transformasi & Pengiriman Twist dengan Smooth Yaw Follow ──────

    def send_world_twist(self, did, v_world_x, v_world_y, current_yaw):
        """Mentransformasikan vektor kecepatan dunia ke frame bodi drone dengan continuous yaw follow."""
        agent = self.agents[did]

        # Titik cegat CBF: state machine mengusulkan, QP yang memutuskan.
        # Batas laju sudah menjadi constraint di dalam QP, jadi tidak boleh
        # dipotong lagi setelahnya — pemotongan pasca-QP bisa melanggar baris
        # dengan sisi kanan negatif (mis. rintangan menyusul dari belakang,
        # yang justru MEWAJIBKAN drone bergerak).
        v_world_x, v_world_y = self._cbf_filter(did, v_world_x, v_world_y)
        spd = math.hypot(v_world_x, v_world_y)

        # Saat menyapu, yaw dikunci ke arah baris: v_safe yang terotasi kuat
        # akan mengayunkan target yaw dan merusak kelurusan sapuan.
        hold_yaw = agent.state in self.YAW_HOLD_STATES
        if spd > 0.15 and not hold_yaw:
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

        # State penahan (delay_*, pivot, done) memanggil jalur ini, bukan
        # send_world_twist. Tanpa filter di sini penghindaran MATI TOTAL
        # selama drone menahan posisi — dan rintangan dinamis yang menyapu
        # persis saat itu tetap menabrak. Ini penyebab tabrakan yang tersisa
        # setelah perbaikan urgensi.
        cy, sy = math.cos(agent.yaw), math.sin(agent.yaw)
        vx_w = vx * cy - vy * sy
        vy_w = vx * sy + vy * cy
        vx_w, vy_w = self._cbf_filter(did, vx_w, vy_w)
        vx = vx_w * cy + vy_w * sy
        vy = -vx_w * sy + vy_w * cy

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

        # Sel terpetakan vs total sel valid di dalam wilayah.
        valid_mask = self.region_mask.copy()
        if self.enable_obstacles and hasattr(self, 'obstacle_mask'):
            valid_mask &= ~self.obstacle_mask
        mapped_cells = int(np.sum(self.cov_grid & valid_mask))
        total_cells = max(1, int(np.sum(valid_mask)))

        alive_count = sum(1 for a in self.agents.values() if a.is_alive and a.state != 'dead')

        scheme_labels = {
            1: "Scheme 1 (Nominal)",
            2: "Scheme 2 (Dryden Wind)",
            3: "Scheme 3 (Obstacles)",
            4: "Scheme 4 (Combined)"
        }
        sch_str = scheme_labels.get(self.scheme, "Custom Scheme")

        # ── 5a. Global Summary Header (X = 25.5m, Y = 13.5m) ──
        m_dash_title = Marker()
        m_dash_title.header.frame_id = 'world'
        m_dash_title.header.stamp = stamp
        m_dash_title.ns = 'hud_sidebar'
        m_dash_title.id = 90
        m_dash_title.type = Marker.TEXT_VIEW_FACING
        m_dash_title.action = Marker.ADD
        m_dash_title.pose.position.x = 25.5
        m_dash_title.pose.position.y = 13.5
        m_dash_title.pose.position.z = 1.0
        m_dash_title.pose.orientation.w = 1.0
        m_dash_title.scale.z = 0.80
        m_dash_title.color = ColorRGBA(r=0.95, g=0.95, b=0.95, a=0.95)
        m_dash_title.text = f'SWARM DASHBOARD  |  {sch_str}'
        ma.markers.append(m_dash_title)

        # ── 5b. Overall Coverage (X = 25.5m, Y = 11.8m) ──
        m_dash_cov = Marker()
        m_dash_cov.header.frame_id = 'world'
        m_dash_cov.header.stamp = stamp
        m_dash_cov.ns = 'hud_sidebar'
        m_dash_cov.id = 91
        m_dash_cov.type = Marker.TEXT_VIEW_FACING
        m_dash_cov.action = Marker.ADD
        m_dash_cov.pose.position.x = 25.5
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

        # ── 5c. Per-Drone Individual Cards (X = 25.5m, Y = 9.5 to -4.5m) ──
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
            m_dcard.pose.position.x = 25.5
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
            for obs_id, ox, oy, rad, height, (cr, cg, cb) in self.static_obstacles:
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
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
