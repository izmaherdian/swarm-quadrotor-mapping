"""Perakitan baris constraint CBF untuk satu drone.

Semua barrier berbentuk h = jarak - radius_gabungan, dengan turunan

    h_dot = n_hat^T (u - v_lawan),      n_hat = (p_drone - p_lawan)/||.||

Syarat CBF  h_dot >= -phi(h)  menjadi baris linear

    -n_hat^T u  <=  phi(h) - n_hat^T v_lawan

Karena a = -n_hat, solusi QP saat baris ini aktif berbentuk
u* = v_nom + (beta - n_hat^T v_nom) n_hat: KOREKSI NORMAL MURNI. Komponen
tangensial v_nom lolos utuh, jadi drone MENYUSUR mengitari rintangan sambil
tetap maju — perilaku yang di kode lama butuh ~140 baris orbit tangen.
"""
import numpy as np

from . import types as T
from .barrier import a_eff_for, phi
from .qp2d import polygon_rows


class RowSet:
    """Baris constraint terkumpul beserta kelas dan labelnya."""

    def __init__(self):
        self.A = []
        self.b = []
        self.cls = []
        self.label = []
        self.h_min = float('inf')

    def add(self, a, b, cls, label, h=None):
        self.A.append(np.asarray(a, dtype=float).reshape(2))
        self.b.append(float(b))
        self.cls.append(cls)
        self.label.append(label)
        if h is not None:
            self.h_min = min(self.h_min, float(h))

    def add_many(self, A, b, cls, label):
        for row, rhs in zip(np.asarray(A, dtype=float), np.asarray(b, dtype=float)):
            self.add(row, rhs, cls, label)

    def arrays(self):
        if not self.A:
            return np.zeros((0, 2)), np.zeros(0)
        return np.vstack(self.A), np.array(self.b)

    def subset(self, keep_classes):
        """Baris yang kelasnya termasuk keep_classes."""
        idx = [i for i, c in enumerate(self.cls) if c in keep_classes]
        if not idx:
            return np.zeros((0, 2)), np.zeros(0)
        A, b = self.arrays()
        return A[idx], b[idx]

    def without(self, drop_classes):
        idx = [i for i, c in enumerate(self.cls) if c not in drop_classes]
        if not idx:
            return np.zeros((0, 2)), np.zeros(0)
        A, b = self.arrays()
        return A[idx], b[idx]

    def __len__(self):
        return len(self.A)


def _unit(v):
    n = float(np.linalg.norm(v))
    if n < 1e-9:
        return np.array([1.0, 0.0]), 0.0
    return v / n, n


def build_rows(agent, task, obstacles, neighbours, bounds, cfg, plant, dt,
               wind_accel=0.0):
    """Rakit seluruh baris constraint untuk satu drone.

    obstacles  : list[Obstacle]  — SEMUA rintangan, bukan hanya milik sel ini.
                 Membatasi ke sel Voronoi sendiri adalah cacat kode lama:
                 rintangan di dekat pusat arena jadi tak terlihat saat transit.
    neighbours : list[(AgentState, lambda_ij)]
    """
    rs = RowSet()
    p = agent.pos
    v_c = plant.v_c
    T_d = cfg.T_d

    # ── 1. Rintangan statis & dinamis ────────────────────────────────────
    #
    # Pemilihan berdasarkan URGENSI, bukan jarak semata. Rintangan dinamis
    # yang mendekat 1.1 m/s dari 3 m jauh lebih mendesak daripada silinder
    # diam di 1.5 m; pengurutan murni jarak menjatuhkan yang salah dan itu
    # persis penyebab dua tabrakan rintangan dinamis di uji Gazebo.
    #
    # Rintangan dinamis juga diberi kuota sendiri sehingga tidak pernah
    # tersingkir oleh kerumunan rintangan statis yang tidak berbahaya.
    horizon = cfg.T_d + 1.0
    dyn_scored, stat_scored = [], []
    for obs in obstacles:
        d_vec = p - obs.pos
        d = float(np.linalg.norm(d_vec))
        if d > cfg.include_radius + obs.radius:
            continue
        delta = (cfg.delta_dynamic if obs.kind == T.CLASS_DYNAMIC
                 else cfg.delta_static)
        R = obs.radius + cfg.drone_radius + delta
        h = d - R

        n_hat_s = d_vec / d if d > 1e-9 else np.array([1.0, 0.0])
        closing = max(0.0, -float(n_hat_s @ (agent.vel - obs.vel)))
        urgency = h - closing * horizon

        entry = (urgency, h, obs, d_vec, d, R)
        (dyn_scored if obs.kind == T.CLASS_DYNAMIC else stat_scored).append(entry)

    dyn_scored.sort(key=lambda s: s[0])
    stat_scored.sort(key=lambda s: s[0])
    selected = (dyn_scored[:cfg.max_dynamic_rows]
                + stat_scored[:cfg.max_obstacle_rows])

    for _urg, h, obs, d_vec, d, R in selected:
        n_hat, _ = _unit(d_vec)
        a_eff = a_eff_for(obs.kind, cfg, plant,
                          obstacle_accel=obs.accel_bound, wind_accel=wind_accel)
        rhs = float(phi(h, a_eff, T_d, v_c)) - float(n_hat @ obs.vel)
        rs.add(-n_hat, rhs, obs.kind, f'{obs.kind}:{obs.oid}', h=h)

    # ── 2. Antar-drone (resiprokal, dua tingkat radius) ──────────────────
    # Dua tingkat itu WAJIB: clip_voronoi_margin membuat drone di sel
    # bersebelahan bisa sah berjarak ~0.84 m, di bawah radius nyaman 1.20 m.
    # Tanpa tingkat lunak yang boleh dilepas, QP infeasible sejak awal misi.
    nb = []
    for other, lam in neighbours:
        if not (other.alive and other.airborne):
            continue
        d_vec = p - other.pos
        d = float(np.linalg.norm(d_vec))
        if d > cfg.v2v_include_radius:
            continue
        nb.append((d, d_vec, other, lam))

    nb.sort(key=lambda s: s[0])
    a_eff_v2v = a_eff_for(T.CLASS_V2V_HARD, cfg, plant, wind_accel=wind_accel)
    for d, d_vec, other, lam in nb[:cfg.max_neighbour_rows]:
        n_hat, _ = _unit(d_vec)
        for radius, cls in ((cfg.v2v_hard, T.CLASS_V2V_HARD),
                            (cfg.v2v_soft, T.CLASS_V2V_SOFT)):
            h = d - radius
            rs.add(-n_hat, lam * float(phi(h, a_eff_v2v, T_d, v_c)),
                   cls, f'{cls}:{other.aid}',
                   h=h if cls == T.CLASS_V2V_HARD else None)

    # ── 3. Dinding arena ─────────────────────────────────────────────────
    a_eff_wall = a_eff_for(T.CLASS_WALL, cfg, plant, wind_accel=wind_accel)
    r = cfg.drone_radius
    walls = (
        (np.array([1.0, 0.0]), bounds.x_max - p[0] - r, '+x'),
        (np.array([-1.0, 0.0]), p[0] - bounds.x_min - r, '-x'),
        (np.array([0.0, 1.0]), bounds.y_max - p[1] - r, '+y'),
        (np.array([0.0, -1.0]), p[1] - bounds.y_min - r, '-y'),
    )
    for a_row, h, name in walls:
        if h < cfg.include_radius:
            rs.add(a_row, float(phi(h, a_eff_wall, T_d, v_c)),
                   T.CLASS_WALL, f'wall:{name}', h=h)

    # ── 4. Dinding henti (ujung baris sapuan) ────────────────────────────
    # Menggantikan ramp feedforward hand-tuned. Nol overshoot jadi konsekuensi
    # teorema yang sama, bukan konstanta yang disetel manual.
    for q, u_dir in task.stop_walls:
        u_hat, _ = _unit(np.asarray(u_dir, dtype=float))
        h = float((np.asarray(q, dtype=float) - p) @ u_hat)
        rs.add(u_hat, float(phi(h, a_eff_wall, T_d, v_c)),
               T.CLASS_STOP, 'stop', h=h)

    # ── 5. Batas laju & batas percepatan ─────────────────────────────────
    # Baris rate inilah kunci perbaikannya: QP tidak bisa lagi melompat
    # 2.85 -> 0.80 m/s dalam satu tick seperti kode lama.
    v_max = getattr(cfg, 'v_max', 3.0)
    A_s, b_s = polygon_rows(np.zeros(2), v_max, cfg.polygon_sides)
    rs.add_many(A_s, b_s, T.CLASS_SPEED, 'speed')

    A_r, b_r = polygon_rows(agent.v_prev_cmd, plant.a_max * dt, cfg.polygon_sides)
    rs.add_many(A_r, b_r, T.CLASS_RATE, 'rate')

    return rs
