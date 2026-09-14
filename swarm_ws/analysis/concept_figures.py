#!/usr/bin/env python3
"""
Figure konsep untuk full paper EPIC 2026 — digambar dengan KODE ASLI, bukan skema karangan.

  fig_concept_high.pdf   partisi Voronoi -> lajur boustrophedon -> FT-CC (U-shape, run PID-H∞)
      * generator Lloyd dihitung ulang dengan fungsi coordinator (clip_voronoi,
        clip_voronoi_margin diambil verbatim dari nodes/swarm_mapping_coordinator.py via AST)
        dan region_seed_points/clip_poly_to_region/generate_boustrophedon dari swarm_high_level;
      * diverifikasi terhadap centroid & jumlah baris per drone yang tercatat di coordinator.log;
      * helper yang benar-benar menyapu tiap lajur recovery ditentukan dari CSV sesudah kill.
  fig_concept_cbf.pdf    φ(h) dengan parameter nyata + geometri QP di ruang kecepatan
      * QP diselesaikan oleh swarm_mid_level.cbf (CBFAvoidance, build_rows) pada satu
        konfigurasi contoh (bukan momen dari misi — dinyatakan di caption).
  generated/concept_numbers.tex   angka rantai satu-tick (u_des -> u* -> p_ref -> θ_ref -> τ -> Ω)
      untuk gain LQR dan H∞ asli.

Pakai:  cd swarm_ws && python3 analysis/concept_figures.py
"""
import ast
import math
import re
import sys
from pathlib import Path

import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt                                          # noqa: E402
from matplotlib.patches import Polygon as MplPolygon, Circle            # noqa: E402
from matplotlib.lines import Line2D                                      # noqa: E402
from shapely.geometry import Polygon as SpPolygon                        # noqa: E402
from shapely.ops import unary_union                                      # noqa: E402

REPO = Path(__file__).resolve().parents[2]
WS = REPO / 'swarm_ws'
for sub in ('swarm_high_level', 'swarm_mid_level', 'swarm_low_level'):
    sys.path.insert(0, str(WS / 'src' / sub))
sys.path.insert(0, str(WS / 'analysis'))

from swarm_high_level.world.region import REGION_PRESETS, region_seed_points          # noqa: E402
from swarm_high_level.world.coverage_path import clip_poly_to_region, generate_boustrophedon  # noqa: E402
from swarm_high_level.world.obstacles import OBSTACLES_BY_REGION, OBSTACLE_RADIUS      # noqa: E402
from swarm_mid_level.cbf import CBFAvoidance, CBFConfig, Obstacle, Bounds, PlantModel  # noqa: E402
from swarm_mid_level.cbf import types as CT                                            # noqa: E402
from swarm_mid_level.cbf.barrier import phi, a_eff_for, phi_zero_h                     # noqa: E402
from swarm_mid_level.cbf.constraints import build_rows                                 # noqa: E402
from swarm_low_level.solver_pid_lqr import PIDLQRSolver                                # noqa: E402
from swarm_low_level.solver_pid_hinf import PIDHinfSolver                              # noqa: E402
import skema5_paper_analysis as S5                                                     # noqa: E402

PAPER = REPO / 'docs' / 'Full Paper - EPIC'
FIG, GEN = PAPER / 'figures', PAPER / 'generated'
COORD = WS / 'nodes' / 'swarm_mapping_coordinator.py'
REGION, FOLDER, CTRL = 'u_shape', '03_u-shape', 'hinf'
C_H, C_L = S5.C_HINF, S5.C_LQR
DC = S5.DRONE_COLORS


def coordinator_functions(*names):
    """Ambil definisi fungsi level-modul dari coordinator TANPA mengimpor ROS."""
    tree = ast.parse(COORD.read_text())
    mod = ast.Module(body=[n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name in names],
                     type_ignores=[])
    ns = {'np': np}
    exec(compile(mod, str(COORD), 'exec'), ns)
    return [ns[n] for n in names]


clip_voronoi, clip_voronoi_margin = coordinator_functions('clip_voronoi', 'clip_voronoi_margin')


# ═════════════════════════════════════════════════════════════════════════
# 1. HIGH LEVEL
# ═════════════════════════════════════════════════════════════════════════
def reproduce_partition(region):
    ring = REGION_PRESETS[region]
    poly = SpPolygon(ring)
    rx0, ry0, rx1, ry1 = poly.bounds
    bbox = [np.array([rx0, ry0]), np.array([rx1, ry0]), np.array([rx1, ry1]), np.array([rx0, ry1])]
    seeds = region_seed_points(poly, 7, seed=7)
    gens = [seeds[i].copy() for i in range(7)]
    for _ in range(25):                                   # coordinator.plan_centroidal_voronoi
        new = []
        for i in range(7):
            cell = [v.copy() for v in bbox]
            for j in range(7):
                if j != i:
                    cell = clip_voronoi(cell, gens[i], gens[j])
            r, ctr = clip_poly_to_region(cell, poly)
            new.append(ctr if len(r) >= 3 else gens[i])
        gens = new
    cells, raws, ctrs = [], [], []
    for i in range(7):
        c = [v.copy() for v in bbox]
        rw = [v.copy() for v in bbox]
        for j in range(7):
            if j != i:
                c = clip_voronoi_margin(c, gens[i], gens[j], margin=0.35)
                rw = clip_voronoi(rw, gens[i], gens[j])
        c, ctr = clip_poly_to_region(c, poly)
        rw, _ = clip_poly_to_region(rw, poly)
        cells.append(np.asarray(c)); raws.append(np.asarray(rw)); ctrs.append(np.asarray(ctr))
    return gens, cells, raws, ctrs


def lanes_of(flat):
    return [(np.asarray(flat[k]), np.asarray(flat[k + 1])) for k in range(0, len(flat) - 1, 2)]


def helper_of_lane(lane, drones, t_from, t_to=None):
    """Drone hidup yang paling banyak melintasi lajur ini sesudah t_from (dari CSV)."""
    pts = [lane[0] + f * (lane[1] - lane[0]) for f in np.linspace(0.05, 0.95, 10)]
    best, best_frac = None, 0.0
    for did, d in drones.items():
        m = d['alive'] & (d['Z'] > 0.8) & (d['Time_s'] >= t_from)
        if t_to is not None:
            m &= d['Time_s'] <= t_to
        if m.sum() < 5:
            continue
        P = np.c_[d['X'][m], d['Y'][m]]
        frac = np.mean([np.min(np.hypot(*(P - p).T)) < 0.5 for p in pts])
        if frac > best_frac:
            best, best_frac = did, frac
    return best, best_frac


def fig_high(out_tex):
    drones, log = S5.load_run(REGION, FOLDER, CTRL, 'pid_' + CTRL)
    gens, cells, raws, ctrs = reproduce_partition(REGION)
    # drone -> sel lewat centroid tercatat di log
    owner, errs = {}, []
    for did, info in log['cells'].items():
        k = int(np.argmin([np.hypot(*(c - np.asarray(info['centroid']))) for c in ctrs]))
        owner[did] = k
        errs.append(float(np.hypot(*(ctrs[k] - np.asarray(info['centroid'])))))
    lanes, row_ok = {}, []
    for did, k in owner.items():
        wp = generate_boustrophedon(cells[k], sweep_spacing=1.45, margin=0.02,
                                    entry_point=np.asarray(log['cells'][did]['start']))
        lanes[did] = lanes_of(wp)
        row_ok.append(len(lanes[did]) == log['cells'][did]['rows'])
    print(f'[high] centroid err maks {max(errs):.3f} m; jumlah baris cocok {sum(row_ok)}/7')
    if max(errs) > 0.05 or not all(row_ok):
        S5.die('partisi hasil reproduksi tidak cocok dengan log — figure tidak dibuat')

    kills = log['kills']
    off = S5.run_metrics(REGION, CTRL, drones, log)['t_offset_csv_minus_log']
    stages = []
    dead = []
    for ki, k in enumerate(kills):
        dead.append(k['drone'])
        polys = [SpPolygon(raws[owner[d]]).buffer(0.08) for d in dead]
        merged = unary_union(polys).buffer(-0.08)
        comps = [merged] if merged.geom_type == 'Polygon' else list(merged.geoms)
        rec = []
        for comp in comps:
            wp = generate_boustrophedon(np.asarray(comp.exterior.coords), sweep_spacing=1.45,
                                        margin=0.20, start_from_top=False, obstacles=None)
            rec += lanes_of(wp)
        t_k = drones[k['drone']]['t_kill']
        t_next = drones[kills[ki + 1]['drone']]['t_kill'] if ki + 1 < len(kills) else None
        who = [helper_of_lane(l, {d: v for d, v in drones.items() if d not in dead}, t_k, t_next) for l in rec]
        stages.append({'dead': list(dead), 'merged': comps, 'lanes': rec, 'who': who, 'logged_rows': k.get('rows'),
                       'helpers': k.get('helpers', []), 't_kill': t_k})
        print(f"[high] kill {ki + 1} iris_{k['drone']}: lajur recovery tanpa filter={len(rec)}, log={k.get('rows')}, "
              f"helper log={k.get('helpers')}, helper dari CSV={sorted({w for w, f in who if w and f >= 0.5})}")

    S5.style()
    fig, axes = plt.subplots(1, 3, figsize=(6.7, 2.55))
    ring = np.asarray(REGION_PRESETS[REGION])

    def base(ax, title):
        ax.add_patch(MplPolygon(ring, closed=True, fc='white', ec='k', lw=0.8, zorder=0))
        for _, ox, oy in OBSTACLES_BY_REGION[REGION]:
            ax.add_patch(Circle((ox, oy), OBSTACLE_RADIUS, fc='#444444', ec='none', zorder=4))
        ax.set_aspect('equal'); ax.set_xlim(-14.8, 14.8); ax.set_ylim(-14.8, 14.8)
        ax.set_xticks([]); ax.set_yticks([]); ax.set_title(title)
        for s in ax.spines.values():
            s.set_visible(False)

    # (a) sel + lajur
    ax = axes[0]
    base(ax, '(a) Voronoi cells and lanes')
    for did, k in owner.items():
        ax.add_patch(MplPolygon(raws[k], closed=True, fc=DC[did - 1], alpha=0.10, ec='none', zorder=1))
        ax.add_patch(MplPolygon(cells[k], closed=True, fc='none', ec=DC[did - 1], lw=0.7, zorder=2))
        for a, b in lanes[did]:
            ax.plot([a[0], b[0]], [a[1], b[1]], c=DC[did - 1], lw=0.55, zorder=3)
        g = gens[k]
        ax.plot(*g, marker='o', ms=3.5, mfc='white', mec=DC[did - 1], mew=1.0, zorder=5)
        ax.text(g[0], g[1] + 1.0, str(did), ha='center', va='bottom', fontsize=7, color=DC[did - 1],
                fontweight='bold', zorder=6)
    # (b),(c) FT-CC
    for si, st in enumerate(stages):
        ax = axes[1 + si]
        base(ax, f'({"bc"[si]}) after loss of agent' + ('s ' if len(st['dead']) > 1 else ' ')
             + ' and '.join(str(d) for d in st['dead']))
        for did, k in owner.items():
            if did in st['dead']:
                continue
            ax.add_patch(MplPolygon(raws[k], closed=True, fc=DC[did - 1], alpha=0.07, ec='none', zorder=1))
        for comp in st['merged']:
            ax.add_patch(MplPolygon(np.asarray(comp.exterior.coords), closed=True, fc='none', ec='k',
                                    lw=0.9, ls=(0, (3, 1.5)), hatch='////', alpha=0.35, zorder=2))
        flown = [w for w, f in st['who'] if w and f >= 0.5]
        if flown:          # warna = helper yang benar-benar menyapu lajur itu (CSV)
            for (a, b), (w, f) in zip(st['lanes'], st['who']):
                col = DC[w - 1] if (w and f >= 0.5) else '#bbbbbb'
                ax.plot([a[0], b[0]], [a[1], b[1]], c=col, lw=1.1 if w and f >= 0.5 else 0.6, zorder=3)
            note = 'flown by ' + ', '.join(f'iris_{h}' for h in sorted(set(flown)))
        else:              # belum diterbangi sebelum kill berikutnya: tampilkan blok alokasi (urutan kode)
            n_h = len(st['helpers'])
            bs = math.ceil(len(st['lanes']) / max(1, n_h))
            tones = ['#6a6a6a', '#b0b0b0', '#8c8c8c']
            for li, (a, b) in enumerate(st['lanes']):
                ax.plot([a[0], b[0]], [a[1], b[1]], c=tones[(li // bs) % 3], lw=1.1, zorder=3)
            note = f"{len(st['lanes'])} lanes in {n_h} blocks for " + ', '.join(f'iris_{h}' for h in st['helpers'])
        for d in st['dead']:
            dd = drones[d]
            kk = np.where(dd['alive'])[0][-1]
            ax.plot(dd['X'][kk], dd['Y'][kk], marker='x', ms=6, mew=1.6, c='k', zorder=7)
            ax.plot(dd['X'][kk], dd['Y'][kk], marker='x', ms=5, mew=0.9, c=DC[d - 1], zorder=8)
        ax.text(0, -14.6, note, ha='center', va='bottom', fontsize=6.3, zorder=9)
    h = [Line2D([], [], c='k', lw=0.7, label='cell after 0.35 m margin'),
         Line2D([], [], marker='o', ls='', mfc='white', mec='k', ms=4, label='Lloyd generator'),
         Line2D([], [], c='k', lw=0.9, ls=(0, (3, 1.5)), label='merged failed cells'),
         Line2D([], [], c='#8c8c8c', lw=1.1, label='recovery lanes (grey: allocated block)'),
         Line2D([], [], marker='x', c='k', ls='', label='agent killed'),
         Line2D([], [], marker='o', c='#444444', ls='', ms=4, label='static cylinder')]
    fig.legend(handles=h, loc='lower center', ncol=6, frameon=False, bbox_to_anchor=(0.5, -0.04),
               fontsize=6.3, handlelength=1.6, columnspacing=0.9)
    fig.subplots_adjust(wspace=0.04)
    fig.savefig(FIG / 'fig_concept_high.pdf')
    plt.close(fig)
    out_tex += [f"\\newcommand{{\\ConceptCentroidErr}}{{{max(errs) * 100:.1f}}}",
                f"\\newcommand{{\\ConceptRecRowsA}}{{{len(stages[0]['lanes'])}}}",
                f"\\newcommand{{\\ConceptRecRowsB}}{{{len(stages[1]['lanes'])}}}"]


# ═════════════════════════════════════════════════════════════════════════
# 2. MID LEVEL + 3. RANTAI SATU TICK
# ═════════════════════════════════════════════════════════════════════════
def cbf_setup():
    plant = PlantModel.from_config(solver='lqr')           # sama dengan coordinator
    cfg = CBFConfig()
    cfg.v_max = 3.0
    cfg.delta_static, cfg.delta_dynamic = 0.35, 0.75
    cfg.include_radius, cfg.cone_deg, cfg.kappa = 10.0, 60.0, 1.0
    return plant, cfg


def fig_cbf(out_tex):
    plant, cfg = cbf_setup()
    wind = 0.50
    a_st = a_eff_for(CT.CLASS_STATIC, cfg, plant, wind_accel=wind)
    T_d, v_c = cfg.T_d, plant.v_c

    # konfigurasi contoh: menyapu ke timur 1.6 m/s, silinder terpeta di depan-kiri
    p = np.array([0.0, 0.0]); v = np.array([1.6, 0.0]); u_des = np.array([1.6, 0.0])
    obs = Obstacle(900, np.array([2.45, 0.55]), radius=0.45, kind=CT.CLASS_STATIC)
    agent = CT.AgentState(aid=1, pos=p, vel=v, v_prev_cmd=v.copy(), priority_w=0.25)
    task = CT.Task(v_nom=u_des)
    avoid = CBFAvoidance(cfg, plant)
    avoid.set_world([obs], Bounds(-50, 50, -50, 50), wind_accel=wind)
    res = avoid.solve_all({1: agent}, {1: task}, 0.05)[1]
    rs = build_rows(agent, task, [obs], [], Bounds(-50, 50, -50, 50), cfg, plant, 0.05, wind_accel=wind)
    A, b = rs.arrays()
    d = float(np.hypot(*(p - obs.pos)))
    h = d - (obs.radius + cfg.drone_radius + cfg.delta_static)
    u = res.v_safe
    if res.tier != 0:
        S5.die(f'konfigurasi contoh tidak feasible (tier {res.tier}) — pilih jarak lain')
    z = (avoid.breaker.bias(agent, u_des, [obs], [], 0.0) + cfg.w_smooth * agent.v_prev_cmd) / (1.0 + cfg.w_smooth)
    print(f'[cbf] a_eff={a_st:.3f} T_d={T_d} v_c={v_c:.3f} h={h:.3f} phi={phi(h, a_st, T_d, v_c):.3f} '
          f'u*={u} tier={res.tier} limiting={res.limiting}')

    S5.style()
    fig, axes = plt.subplots(1, 2, figsize=(6.7, 2.35), gridspec_kw={'width_ratios': [1.15, 1.0]})
    # (a) φ(h)
    ax = axes[0]
    hh = np.linspace(-0.3, 3.0, 800)
    ph = phi(hh, a_st, T_d, v_c)
    h0 = phi_zero_h(a_st, T_d, v_c)
    gam = float(phi(1.0, a_st, T_d, v_c)) / 1.0
    ax.axhline(0, c='0.6', lw=0.5)
    ax.axvspan(0, h0, color='red', alpha=0.08, lw=0)
    ax.plot(hh, ph, c=C_H, lw=1.3, label=r'actuator-consistent $\varphi(h)$')
    ax.plot(hh[hh >= 0], gam * hh[hh >= 0], c=C_L, lw=1.0, ls='--', label=r'linear $\gamma h$ (equal at $h=1$ m)')
    for s_ in (0.5, 1.0, 1.6):
        hreq = (s_ ** 2 + 2 * a_st * T_d * s_ + v_c ** 2) / (2 * a_st)
        ax.plot([hreq], [s_], marker='o', ms=3, c='k')
        ax.annotate(f'{s_:.1f} m/s needs {hreq:.2f} m', (hreq, s_), xytext=(4, -9), textcoords='offset points', fontsize=6.3)
    ax.text(h0 / 2, 1.9, 'must\nstop', ha='center', fontsize=6.3, color='red')
    ax.set_xlim(-0.3, 3.0); ax.set_ylim(-0.8, 2.6)
    ax.set_xlabel('clearance $h$ (m)'); ax.set_ylabel('admissible approach speed (m/s)')
    ax.set_title(r'(a) class-$\mathcal{K}$ function, static rows')
    ax.legend(frameon=False, loc='upper left', fontsize=6.5)
    ax.tick_params(length=2)
    # (b) QP di ruang kecepatan
    ax = axes[1]
    g = np.linspace(-0.5, 3.2, 500)
    UX, UY = np.meshgrid(g, np.linspace(-1.9, 1.9, 400))
    feas = np.ones_like(UX, bool)
    for Ai, bi in zip(A, b):
        feas &= (Ai[0] * UX + Ai[1] * UY) <= bi + 1e-9
    ax.contourf(UX, UY, feas.astype(float), levels=[0.5, 1.5], colors=['#d9ecd9'], zorder=0)
    lab_done = set()
    for Ai, bi, cls in zip(A, b, rs.cls if hasattr(rs, 'cls') else [None] * len(b)):
        if abs(Ai[1]) > 1e-9:
            yy = (bi - Ai[0] * g) / Ai[1]
            xs, ys = g, yy
        else:
            xs, ys = np.full(50, bi / Ai[0]), np.linspace(-1.9, 1.9, 50)
        style = dict(c='#a23b72', lw=1.2) if cls == CT.CLASS_STATIC else dict(c='0.65', lw=0.5)
        ax.plot(xs, ys, **style, zorder=1)
    ax.annotate('', xy=u_des, xytext=(0, 0), arrowprops=dict(arrowstyle='-|>', color='k', lw=1.0))
    ax.annotate('', xy=u, xytext=(0, 0), arrowprops=dict(arrowstyle='-|>', color=C_H, lw=1.4))
    ax.plot([z[0], u[0]], [z[1], u[1]], c='0.3', lw=0.6, ls=':')
    if np.hypot(*(z - u_des)) > 0.02:
        ax.plot(*z, marker='+', ms=6, c='0.3')
        ax.text(z[0] + 0.05, z[1] + 0.12, r'$\mathbf{z}$', fontsize=8, color='0.3')
    ax.text(u_des[0] + 0.05, u_des[1] + 0.12, r'$\mathbf{u}_{\mathrm{des}}$', fontsize=8)
    ax.text(u[0] + 0.05, u[1] - 0.32, r'$\mathbf{u}^\star$', fontsize=8, color=C_H)
    ax.add_patch(Circle((0, 0), 0.06, fc='k', zorder=5))
    ax.set_xlim(-0.5, 3.2); ax.set_ylim(-1.9, 1.9); ax.set_aspect('equal')
    ax.set_xlabel('$u_x$ (m/s)'); ax.set_ylabel('$u_y$ (m/s)')
    ax.set_title('(b) projection in velocity space')
    ax.legend(handles=[Line2D([], [], c='#a23b72', lw=1.2, label='obstacle row, Eq. (4)'),
                       Line2D([], [], c='0.65', lw=0.5, label='speed/rate polygon rows'),
                       plt.Rectangle((0, 0), 1, 1, fc='#d9ecd9', label='feasible set')],
              frameon=False, loc='lower left', fontsize=6.3)
    ax.tick_params(length=2)
    fig.subplots_adjust(wspace=0.32)
    fig.savefig(FIG / 'fig_concept_cbf.pdf')
    plt.close(fig)

    # ── rantai satu tick (angka untuk TikZ) ──
    params = yaml.safe_load((WS / 'src' / 'swarm_low_level' / 'config' / 'quadrotor_params.yaml').read_text())
    ph_, lim, act = params['physics'], params['actuator_limits'], params['actuator_physics']
    p_ref = p + plant.T_lead * u
    e = p_ref - p
    m, gacc = ph_['mass'], ph_['g']
    kf, km = act['kf'], act['km']
    dd = ph_['arm_length'] * 0.707106781
    M = np.array([[kf, kf, kf, kf], [-kf * dd, kf * dd, kf * dd, -kf * dd],
                  [-kf * dd, kf * dd, -kf * dd, kf * dd], [-km, -km, km, km]])
    Minv = np.linalg.inv(M)
    L = [f'\\newcommand{{\\TickUdesX}}{{{u_des[0]:.2f}}}',
         f'\\newcommand{{\\TickUstarX}}{{{u[0]:.2f}}}', f'\\newcommand{{\\TickUstarY}}{{{u[1]:.2f}}}',
         f'\\newcommand{{\\TickH}}{{{h:.2f}}}', f'\\newcommand{{\\TickPhi}}{{{float(phi(h, a_st, T_d, v_c)):.2f}}}',
         f'\\newcommand{{\\TickEx}}{{{e[0]:.2f}}}', f'\\newcommand{{\\TickEy}}{{{e[1]:.2f}}}',
         f'\\newcommand{{\\TickAeff}}{{{a_st:.2f}}}', f'\\newcommand{{\\TickTd}}{{{T_d:.2f}}}',
         f'\\newcommand{{\\TickObsX}}{{{obs.pos[0]:.2f}}}', f'\\newcommand{{\\TickObsY}}{{{obs.pos[1]:.2f}}}']
    for name, gains in (('Hinf', PIDHinfSolver(ph_).get_all_gains()), ('Lqr', PIDLQRSolver(ph_).compute_all_gains())):
        gx, gy = gains['x_outer'], gains['y_outer']
        th = gx['Kp'] * e[0] - gx['Kd'] * v[0] + 0.15 * u[0]              # pid_*_node.py: theta_ref_raw (I = 0)
        ph_r = gy['Kp'] * e[1] - gy['Kd'] * v[1] - 0.15 * u[1]            # phi_ref_raw
        th_c = float(np.clip(th, -lim['angle_max'], lim['angle_max']))
        ph_c = float(np.clip(ph_r, -lim['angle_max'], lim['angle_max']))
        tau_y = float(np.clip(gains['x_inner']['Kp'] * th_c, -lim['tau_rp_max'], lim['tau_rp_max']))
        tau_x = float(np.clip(gains['y_inner']['Kp'] * ph_c, -lim['tau_rp_max'], lim['tau_rp_max']))
        w2 = Minv @ np.array([m * gacc, tau_x, tau_y, 0.0])
        w = np.sqrt(np.clip(w2, 0, None))
        L += [f'\\newcommand{{\\TickTheta{name}}}{{{math.degrees(th):.1f}}}',
              f'\\newcommand{{\\TickThetaC{name}}}{{{math.degrees(th_c):.1f}}}',
              f'\\newcommand{{\\TickPhiRef{name}}}{{{math.degrees(ph_r):.1f}}}',
              f'\\newcommand{{\\TickPhiC{name}}}{{{math.degrees(ph_c):.1f}}}',
              f'\\newcommand{{\\TickTauY{name}}}{{{tau_y:.2f}}}', f'\\newcommand{{\\TickTauX{name}}}{{{tau_x:.2f}}}',
              f'\\newcommand{{\\TickWmin{name}}}{{{w.min():.0f}}}', f'\\newcommand{{\\TickWmax{name}}}{{{w.max():.0f}}}']
        print(f'[tick] {name}: theta_ref {math.degrees(th):.1f}->{math.degrees(th_c):.1f} deg, '
              f'phi_ref {math.degrees(ph_r):.1f}->{math.degrees(ph_c):.1f} deg, tau=({tau_x:.2f},{tau_y:.2f}), '
              f'omega {w.min():.0f}-{w.max():.0f} rad/s')
    out_tex += L


def main():
    out = ['% DIHASILKAN OTOMATIS oleh swarm_ws/analysis/concept_figures.py — JANGAN DIEDIT.']
    fig_high(out)
    fig_cbf(out)
    (GEN / 'concept_numbers.tex').write_text('\n'.join(out) + '\n')
    print('ok')


if __name__ == '__main__':
    main()
