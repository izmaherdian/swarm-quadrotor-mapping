"""CBFAvoidance — satu QP per drone per tick, menggantikan seluruh lapisan lama.

Kode lama menumpuk empat lapisan (orbit tangen, repulsion darurat, evasion
koridor, repulsion V2V) yang masing-masing menulis v_avoid dan ref_pos sendiri
lalu dijumlahkan tanpa arbitrasi. Di sini semuanya menjadi baris constraint
dalam SATU QP, jadi mereka dinegosiasikan bersama dan menghasilkan satu
perintah kecepatan yang konsisten.

Pemakaian:

    avoid = CBFAvoidance(cfg, plant)
    avoid.set_world(obstacles, bounds)
    results = avoid.solve_all(agents, tasks, dt, t_now)

Selalu pakai solve_all, bukan solve() tujuh kali: split tanggung jawab
resiprokal lambda_ij + lambda_ji = 1 hanya eksak bila dihitung dari SATU
snapshot. Ini keunggulan nyata atas ORCA terdistribusi.
"""
import time

import numpy as np

from . import types as T
from .constraints import build_rows
from .deadlock import DeadlockBreaker
from .qp2d import max_violation, solve_least_violating, solve_projection

# Kesediaan bermanuver per state misi. Makin besar = makin bersedia mengalah.
# Drone yang sedang menyapu baris dilindungi supaya garis petanya tetap lurus.
DEFAULT_PRIORITY_W = {
    'sweeping_row': 0.25,
    'sweeping_recovery': 0.25,
    'align_start_yaw': 0.25,
    'delay_at_corner_end': 0.50,
    'delay_at_new_row': 0.50,
    'stepping_vertical': 1.00,
    'transit_to_start': 1.00,
    'transit_to_recovery': 1.00,
    'pivot_to_transit': 1.00,
    'return_to_centroid': 1.00,
    'wait_all_start': 1.50,
    'done': 1.50,
}


class CBFAvoidance:
    def __init__(self, cfg, plant):
        self.cfg = cfg
        self.plant = plant
        self.obstacles = []
        self.bounds = T.Bounds()
        self.wind_accel = 0.0
        self.breaker = DeadlockBreaker(cfg)
        self.cell_polygons = {}

    def set_world(self, obstacles, bounds, wind_accel=0.0):
        self.obstacles = list(obstacles)
        self.bounds = bounds
        self.wind_accel = float(wind_accel)

    def set_cell_polygon(self, aid, polygon):
        """Opsional: dipakai pemecah simetri untuk memilih sisi memutar."""
        self.cell_polygons[aid] = polygon

    # ── Inti: satu QP ────────────────────────────────────────────────────

    def _solve_one(self, agent, task, neighbours, dt, t_now):
        cfg, plant = self.cfg, self.plant
        t0 = time.perf_counter()

        # Target: campuran v_nom dengan perintah sebelumnya (kehalusan), lalu
        # diberi bias tangensial bila berhadapan langsung. Semua di objektif.
        v_biased = self.breaker.bias(
            agent, task.v_nom, self.obstacles, neighbours, t_now,
            cell_polygon=self.cell_polygons.get(agent.aid))
        z = (v_biased + cfg.w_smooth * agent.v_prev_cmd) / (1.0 + cfg.w_smooth)

        rs = build_rows(agent, task, self.obstacles, neighbours, self.bounds,
                        cfg, plant, dt, wind_accel=self.wind_accel)

        A, b = rs.arrays()
        tier, slack = 0, 0.0

        u, feasible = solve_projection(A, b, z)

        # Tier 1: lepas kelas lunak berurutan, yang keras tidak pernah dilepas.
        if not feasible:
            dropped = []
            for cls in T.SOFT_DROP_ORDER:
                dropped.append(cls)
                A_r, b_r = rs.without(dropped)
                u, feasible = solve_projection(A_r, b_r, z)
                if feasible:
                    tier = 1
                    break

        # Tier 2: pelanggaran seminimal mungkin, tetap di dalam himpunan kinematik.
        if not feasible:
            A_keep, b_keep = rs.subset(T.KINEMATIC_CLASSES)
            A_hard, b_hard = rs.subset(T.HARD_CLASSES)
            if len(b_hard) and len(b_keep):
                u, slack = solve_least_violating(A_hard, b_hard, A_keep, b_keep, z)
                tier, feasible = 2, True

        # Tier 3: tidak pernah mengembalikan sampah.
        if not feasible or not np.all(np.isfinite(u)):
            u = 0.9 * agent.v_prev_cmd
            tier, slack = 3, float('inf')

        # Baris mana yang mengikat (untuk diagnosa & laporan paper).
        limiting, active = 'none', False
        if len(b):
            resid = b - A @ u
            k = int(np.argmin(resid))
            if resid[k] < 1e-3:
                active = True
                limiting = rs.label[k]

        return T.AvoidanceResult(
            v_safe=u,
            ref_pos=agent.pos + plant.T_lead * u,
            active=active,
            h_min=rs.h_min,
            slack=slack,
            tier=tier,
            limiting=limiting,
            n_rows=len(rs),
            solve_us=(time.perf_counter() - t0) * 1e6,
        )

    # ── API publik ───────────────────────────────────────────────────────

    def solve_all(self, agents, tasks, dt, t_now=0.0):
        """Selesaikan seluruh drone dari satu snapshot.

        agents : dict[aid, AgentState]
        tasks  : dict[aid, Task]
        """
        dt = float(np.clip(dt, self.cfg.dt_min, self.cfg.dt_max))
        results = {}
        for aid, agent in agents.items():
            if not agent.alive or aid not in tasks:
                continue
            neighbours = [
                (other, _lambda(agent, other))
                for oid, other in agents.items() if oid != aid
            ]
            results[aid] = self._solve_one(agent, tasks[aid], neighbours, dt, t_now)
        return results

    def solve(self, aid, agents, task, dt, t_now=0.0):
        """Satu drone saja (mode node terdistribusi). Resiprositas jadi
        aproksimasi karena snapshot tetangga bisa berbeda umur."""
        dt = float(np.clip(dt, self.cfg.dt_min, self.cfg.dt_max))
        agent = agents[aid]
        neighbours = [(o, _lambda(agent, o)) for k, o in agents.items() if k != aid]
        return self._solve_one(agent, task, neighbours, dt, t_now)


def _lambda(agent_i, agent_j):
    """Bagi tanggung jawab menghindar berdasarkan kesediaan bermanuver.

    lambda_ij + lambda_ji = 1 secara eksak, sehingga penjumlahan kedua
    constraint memulihkan syarat CBF bersama h_dot >= -phi(h) tanpa perlu
    komunikasi maupun negosiasi.
    """
    wi = max(1e-6, float(agent_i.priority_w))
    wj = max(1e-6, float(agent_j.priority_w))
    return wi / (wi + wj)


def priority_for_state(state_name):
    """Petakan nama state misi ke bobot kesediaan bermanuver."""
    return DEFAULT_PRIORITY_W.get(state_name, 1.0)
