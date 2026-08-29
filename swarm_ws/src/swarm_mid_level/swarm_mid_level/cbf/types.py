"""Tipe data untuk lapisan penghindaran CBF-QP.

Semua vektor 2D dalam frame dunia (world), satuan SI.
"""
from dataclasses import dataclass, field

import numpy as np

# Kelas constraint. Urutan menentukan prioritas pelepasan saat infeasible:
# yang lunak dilepas lebih dulu, yang keras tidak pernah dilepas.
CLASS_STATIC = 'static'      # rintangan statis  — keras
CLASS_DYNAMIC = 'dynamic'    # rintangan dinamis — keras
CLASS_V2V_HARD = 'v2v_hard'  # antar-drone, radius minimum — keras
CLASS_V2V_SOFT = 'v2v_soft'  # antar-drone, radius nyaman — lunak
CLASS_WALL = 'wall'          # batas arena — lunak
CLASS_STOP = 'stop'          # dinding henti ujung baris — lunak
CLASS_SPEED = 'speed'        # ‖u‖ <= v_max — selalu dipertahankan
CLASS_RATE = 'rate'          # ‖u - u_prev‖ <= a·dt — selalu dipertahankan

HARD_CLASSES = (CLASS_STATIC, CLASS_DYNAMIC, CLASS_V2V_HARD)
# Dilepas berurutan pada Tier 1.
SOFT_DROP_ORDER = (CLASS_WALL, CLASS_V2V_SOFT, CLASS_STOP)
# Hanya dua kelas ini yang bertahan di Tier 2.
KINEMATIC_CLASSES = (CLASS_SPEED, CLASS_RATE)


def _vec2(v=None):
    return np.zeros(2, dtype=float) if v is None else np.asarray(v, dtype=float)


@dataclass
class AgentState:
    """Kondisi satu drone pada satu tick."""
    aid: int
    pos: np.ndarray                    # (2,) posisi dunia
    vel: np.ndarray                    # (2,) kecepatan terukur
    v_prev_cmd: np.ndarray             # (2,) perintah kecepatan tick sebelumnya
    radius: float = 0.22
    priority_w: float = 1.0            # kesediaan bermanuver; makin besar = makin mengalah
    airborne: bool = True
    alive: bool = True

    def __post_init__(self):
        self.pos = _vec2(self.pos)
        self.vel = _vec2(self.vel)
        self.v_prev_cmd = _vec2(self.v_prev_cmd)


@dataclass
class Obstacle:
    """Rintangan silinder, statis maupun bergerak."""
    oid: int
    pos: np.ndarray
    vel: np.ndarray = field(default_factory=lambda: np.zeros(2))
    radius: float = 0.40
    accel_bound: float = 0.0           # percepatan maks rintangan itu sendiri
    kind: str = CLASS_STATIC

    def __post_init__(self):
        self.pos = _vec2(self.pos)
        self.vel = _vec2(self.vel)


@dataclass
class Task:
    """Keluaran state machine misi untuk satu drone: apa yang DIINGINKAN."""
    v_nom: np.ndarray
    yaw_mode: str = 'follow_velocity'   # 'follow_velocity' | 'hold'
    yaw_hold: float = 0.0
    stop_walls: tuple = ()              # ((titik, normal_masuk), ...)

    def __post_init__(self):
        self.v_nom = _vec2(self.v_nom)


@dataclass
class Bounds:
    """Kotak arena."""
    x_min: float = -15.0
    x_max: float = 15.0
    y_min: float = -15.0
    y_max: float = 15.0


@dataclass
class AvoidanceResult:
    """Keluaran QP untuk satu drone."""
    v_safe: np.ndarray
    ref_pos: np.ndarray                # = pos + T_lead * v_safe (satu-satunya sumber kebenaran)
    active: bool = False               # ada baris keamanan yang mengikat
    h_min: float = float('inf')        # clearance terkecil di antara semua constraint
    slack: float = 0.0                 # > 0 hanya bila tier >= 2
    tier: int = 0
    limiting: str = 'none'
    n_rows: int = 0
    solve_us: float = 0.0


@dataclass
class CBFConfig:
    """Parameter yang dapat disetel. Nilai fisik diturunkan di PlantModel."""
    dt_nominal: float = 0.05
    dt_min: float = 0.04
    dt_max: float = 0.10

    T_d: float = 0.40          # DIIDENTIFIKASI dari telemetri (persentil ke-75
                               # dari 7 drone; median 0.375s). Tebakan awal
                               # 0.30 membuat phi terlalu permisif.
    a_margin_static: float = 0.75      # fraksi a_max yang dipakai di phi
    a_margin_wall: float = 0.85

    drone_radius: float = 0.22
    delta_static: float = 0.25         # bantalan tambahan ke rintangan statis
    delta_dynamic: float = 0.30
    v2v_hard: float = 0.70             # jarak pusat-ke-pusat minimum mutlak
    v2v_soft: float = 1.20

    include_radius: float = 8.0        # jangkauan pertimbangan RINTANGAN
    # Jangkauan antar-drone dipisah dari jangkauan rintangan. JANGAN
    # dikecilkan: diuji pada pertukaran 7-agen antipodal, nilai 3.0 dan 4.0
    # membuat kawanan menumpuk lalu buntu, sementara 6.0 lolos. Constraint
    # butuh cukup jarak pandang — pada 3 m dengan laju saling mendekat 5 m/s
    # hanya tersisa 0.6 detik untuk bernegosiasi.
    v2v_include_radius: float = 6.0
    max_obstacle_rows: int = 4         # kuota rintangan STATIS
    max_dynamic_rows: int = 2          # kuota terpisah: yang bergerak tidak
                                       # boleh tersingkir oleh yang diam
    max_neighbour_rows: int = 4
    polygon_sides: int = 8             # aproksimasi lingkaran untuk batas laju/rate

    w_smooth: float = 0.25             # bobot kehalusan terhadap u_prev

    # Pemecah kebuntuan (hanya menyentuh objektif, tidak pernah constraint)
    kappa: float = 0.6
    cone_deg: float = 20.0
    stall_speed: float = 0.15
    stall_time: float = 2.0
    k_separate: float = 2.0   # penguatan pemulihan saat di dalam radius nyaman

    v_max: float = 3.0        # batas laju perintah (m/s)
