#!/usr/bin/env python3
"""Fast-sim Skema 3: medan rintangan asli, plant asli, tanpa Gazebo.

Memakai 9 rintangan statis dan 2 rintangan dinamis persis seperti
obstacles.world, plus model kecepatan loop-tertutup yang teridentifikasi.
Berjalan ~1 detik untuk 200 detik simulasi — dibanding Gazebo yang RTF-nya
0.26 (200 detik simulasi = 13 menit menunggu).

Pakai:
    PYTHONPATH=src/swarm_mid_level:src/swarm_low_level \
        python3 src/swarm_mid_level/test/scenario_scheme3.py
"""
import math
import sys

import numpy as np

from swarm_mid_level.cbf import (
    AgentState, Bounds, CBFAvoidance, CBFConfig, Obstacle, PlantModel, Task,
)
from swarm_mid_level.cbf import types as T
from swarm_mid_level.cbf.avoidance import priority_for_state

# Salinan persis dari obstacles.world / test_7drone_voronoi_mapping.py:442-453
STATIC_OBSTACLES = [
    (101, -1.5, 9.5), (102, 4.0, 6.0), (103, 6.5, 9.5),
    (104, -8.0, -2.0), (105, -5.0, -7.5), (106, -10.5, -12.5),
    (107, 6.0, -4.0), (108, 0.0, 2.5), (109, 2.5, -9.0),
]
OBS_RADIUS = 0.40
DYN_RADIUS = 0.45
OMEGA = (0.15, 0.11)


def dynamic_positions(t):
    """Pola 'X' diagonal, sama seperti update_dynamic_obstacles()."""
    w1, w2 = OMEGA
    p1 = np.array([-10.0 * math.cos(w1 * t), 10.0 * math.cos(w1 * t)])
    v1 = np.array([10.0 * w1 * math.sin(w1 * t), -10.0 * w1 * math.sin(w1 * t)])
    p2 = np.array([10.0 * math.cos(w2 * t), 10.0 * math.cos(w2 * t)])
    v2 = np.array([-10.0 * w2 * math.sin(w2 * t), -10.0 * w2 * math.sin(w2 * t)])
    return (p1, v1), (p2, v2)


def boustrophedon(x0, x1, y0, y1, spacing=1.45, trim=True):
    """Jalur sapuan bolak-balik dalam kotak.

    Waypoint yang jatuh di dalam zona eksklusi rintangan dibuang — cerminan
    pemangkasan ujung baris di generate_boustrophedon() pada sistem asli.
    Tanpa ini, planner bisa meminta titik yang secara fisik mustahil dicapai
    (rintangan #102 persis di (4.0, 6.0), salah satu sudut sapuan).
    """
    wps, y, flip = [], y0, False
    while y <= y1 + 1e-9:
        wps.append(np.array([x1 if flip else x0, y]))
        wps.append(np.array([x0 if flip else x1, y]))
        y += spacing
        flip = not flip

    if not trim:
        return wps

    keep_out = OBS_RADIUS + 0.22 + 0.45
    out = []
    for p in wps:
        if all(np.linalg.norm(p - np.array([ox, oy])) > keep_out
               for _, ox, oy in STATIC_OBSTACLES):
            out.append(p)
    return out if len(out) >= 2 else wps


# Tujuh koridor sapuan yang sengaja melewati rintangan statis.
CELLS = [
    (1, -14.0, -6.0, -14.0, -8.0),
    (2, -4.0, 4.0, 6.0, 12.0),
    (3, 3.0, 11.0, 4.0, 11.0),
    (4, -12.0, -4.0, -9.0, -1.0),
    (5, 2.0, 10.0, -6.0, -2.0),
    (6, -4.0, 4.0, -13.0, -9.0),
    (7, -2.0, 5.0, -10.0, 3.0),
]

NOMINAL_SPEED = 2.85
ARRIVE_TOL = 0.35


def run(seconds=200.0, dt=0.05, speed=NOMINAL_SPEED, verbose=True):
    plant = PlantModel.from_config()
    cfg = CBFConfig()
    avoid = CBFAvoidance(cfg, plant)

    agents, routes, idx = {}, {}, {}
    for aid, x0, x1, y0, y1 in CELLS:
        wps = boustrophedon(x0, x1, y0, y1)
        routes[aid] = wps
        idx[aid] = 0
        agents[aid] = AgentState(
            aid=aid, pos=wps[0].copy(), vel=np.zeros(2), v_prev_cmd=np.zeros(2),
            priority_w=priority_for_state('sweeping_row'))

    bounds = Bounds(-15, 15, -15, 15)
    n_steps = int(seconds / dt)

    h_static_min = float('inf')
    h_dyn_min = float('inf')
    d_v2v_min = float('inf')
    crashes = 0
    tiers = np.zeros(4, dtype=int)
    max_slack = 0.0

    for step in range(n_steps):
        t = step * dt

        (p1, v1), (p2, v2) = dynamic_positions(t)
        obstacles = [
            Obstacle(oid, np.array([x, y]), radius=OBS_RADIUS, kind=T.CLASS_STATIC)
            for oid, x, y in STATIC_OBSTACLES
        ] + [
            Obstacle(201, p1, v1, DYN_RADIUS, accel_bound=10.0 * OMEGA[0] ** 2,
                     kind=T.CLASS_DYNAMIC),
            Obstacle(202, p2, v2, DYN_RADIUS, accel_bound=10.0 * OMEGA[1] ** 2,
                     kind=T.CLASS_DYNAMIC),
        ]
        avoid.set_world(obstacles, bounds)

        tasks = {}
        for aid, ag in agents.items():
            wps = routes[aid]
            k = min(idx[aid], len(wps) - 1)
            target = wps[k]
            d = target - ag.pos
            dist = float(np.linalg.norm(d))
            if dist < ARRIVE_TOL and idx[aid] < len(wps) - 1:
                idx[aid] += 1
                target = wps[idx[aid]]
                d = target - ag.pos
                dist = float(np.linalg.norm(d))
            v_nom = (d / dist) * min(speed, 2.0 * dist) if dist > 1e-6 else np.zeros(2)
            # Dinding henti di ujung baris: nol overshoot sebagai konsekuensi
            # barrier yang sama, bukan konstanta ramp yang disetel tangan.
            stop = ((target, d / dist),) if dist > 1e-6 else ()
            tasks[aid] = Task(v_nom=v_nom, stop_walls=stop)

        results = avoid.solve_all(agents, tasks, dt, t_now=t)

        for aid, ag in agents.items():
            r = results[aid]
            tiers[r.tier] += 1
            if np.isfinite(r.slack):
                max_slack = max(max_slack, r.slack)
            ag.vel = plant.step(ag.vel, r.v_safe, dt)
            ag.pos = ag.pos + ag.vel * dt
            ag.v_prev_cmd = r.v_safe

        # Pengukuran clearance nyata (radius fisik saja, tanpa bantalan)
        for aid, ag in agents.items():
            for oid, ox, oy in STATIC_OBSTACLES:
                h = float(np.linalg.norm(ag.pos - np.array([ox, oy]))) \
                    - (OBS_RADIUS + cfg.drone_radius)
                h_static_min = min(h_static_min, h)
                if h < 0:
                    crashes += 1
            for p in (p1, p2):
                h = float(np.linalg.norm(ag.pos - p)) - (DYN_RADIUS + cfg.drone_radius)
                h_dyn_min = min(h_dyn_min, h)
                if h < 0:
                    crashes += 1

        P = np.array([a.pos for a in agents.values()])
        D = np.linalg.norm(P[:, None, :] - P[None, :, :], axis=2)
        np.fill_diagonal(D, np.inf)
        d_v2v_min = min(d_v2v_min, float(D.min()))

    total = int(tiers.sum())
    tier_counts = tiers.copy()
    progress = {aid: (idx[aid], len(routes[aid])) for aid in agents}
    done = sum(1 for aid in agents if idx[aid] >= len(routes[aid]) - 1)

    if verbose:
        print('=' * 68)
        print(f'  FAST-SIM SKEMA 3 — {seconds:.0f}s simulasi, v_nom={speed:.2f} m/s')
        print('=' * 68)
        print(f'  Tabrakan (h < 0)            : {crashes}')
        print(f'  Clearance min rintangan stat: {h_static_min:+.3f} m')
        print(f'  Clearance min rintangan dyn : {h_dyn_min:+.3f} m')
        print(f'  Jarak antar-drone minimum   : {d_v2v_min:.3f} m  '
              f'(batas keras {cfg.v2v_hard})')
        print(f'  Drone menuntaskan rute      : {done}/{len(agents)}')
        print(f'  Distribusi tier ({total} tick) : '
              f'T0 {tier_counts[0]}  T1 {tier_counts[1]}  '
              f'T2 {tier_counts[2]}  T3 {tier_counts[3]}   '
              f'-> P(tier>0) = {(total-tier_counts[0])/total*100:.3f}%')
        print(f'  Slack maksimum              : {max_slack:.4f}')
        print('  Kemajuan per drone          : '
              + ', '.join(f'i{a}:{p[0]}/{p[1]}' for a, p in progress.items()))
        print('=' * 68)

    return {
        'crashes': crashes,
        'h_static_min': h_static_min,
        'h_dyn_min': h_dyn_min,
        'd_v2v_min': d_v2v_min,
        'tier_frac': tiers / max(1, total),
        'max_slack': max_slack,
        'done': done,
    }


if __name__ == '__main__':
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 200.0
    spd = float(sys.argv[2]) if len(sys.argv) > 2 else NOMINAL_SPEED
    res = run(secs, speed=spd)
    sys.exit(0 if res['crashes'] == 0 else 1)
