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
  generated/concept_numbers.tex   angka yang dikutip caption/teks (jumlah lajur, h0, contoh QP).

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
from swarm_mid_level.cbf.barrier import phi, a_eff_for                     # noqa: E402
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
    fig, axes = plt.subplots(1, 3, figsize=(6.7, 2.3))
    ring = np.asarray(REGION_PRESETS[REGION])

    def base(ax, title):
        ax.add_patch(MplPolygon(ring, closed=True, fc='white', ec='k', lw=0.8, zorder=0))
        for _, ox, oy in OBSTACLES_BY_REGION[REGION]:
            ax.add_patch(Circle((ox, oy), OBSTACLE_RADIUS, fc='#444444', ec='none', zorder=4))
        ax.set_aspect('equal'); ax.set_xlim(-14.6, 14.6); ax.set_ylim(-14.6, 14.6)
        ax.set_xticks([]); ax.set_yticks([]); ax.set_title(title, pad=3)
        for s in ax.spines.values():
            s.set_visible(False)

    # (a) sel + lajur
    ax = axes[0]
    base(ax, '(a) cells and lanes at take-off')
    ax.set_xlabel('7 agents, ' + str(sum(len(v) for v in lanes.values())) + ' lanes', fontsize=7, labelpad=2)
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
        base(ax, f'({"bc"[si]}) agent {st["dead"][-1]} lost at {st["t_kill"]:.0f} s')
        for did, k in owner.items():
            if did in st['dead']:
                continue
            ax.add_patch(MplPolygon(raws[k], closed=True, fc=DC[did - 1], alpha=0.07, ec='none', zorder=1))
        for comp in st['merged']:
            ax.add_patch(MplPolygon(np.asarray(comp.exterior.coords), closed=True, fc='none', ec='k',
                                    lw=0.9, ls=(0, (3, 1.5)), hatch='//', alpha=0.30, zorder=2))
        flown = [w for w, f in st['who'] if w and f >= 0.5]
        if flown:          # warna = helper yang benar-benar menyapu lajur itu (CSV)
            for (a, b), (w, f) in zip(st['lanes'], st['who']):
                col = DC[w - 1] if (w and f >= 0.5) else '#bbbbbb'
                ax.plot([a[0], b[0]], [a[1], b[1]], c=col, lw=1.1 if w and f >= 0.5 else 0.6, zorder=3)
            n_fl = sum(1 for w, f in st['who'] if w and f >= 0.5)
            note = (f"{len(st['lanes'])} lanes; {n_fl} flown by iris_"
                    + ', '.join(str(h) for h in sorted(set(flown))))
            st['n_flown'] = n_fl
        else:              # belum diterbangi sebelum kill berikutnya: tampilkan blok alokasi (urutan kode)
            n_h = len(st['helpers'])
            bs = math.ceil(len(st['lanes']) / max(1, n_h))
            tones = ['#555555', '#aaaaaa', '#808080']
            for li, (a, b) in enumerate(st['lanes']):
                ax.plot([a[0], b[0]], [a[1], b[1]], c=tones[(li // bs) % 3], lw=1.1, zorder=3)
            note = (f"{len(st['lanes'])} lanes in {n_h} blocks for iris_"
                    + ', '.join(str(h) for h in st['helpers']))
            st['n_flown'] = 0
        for d in st['dead']:
            dd = drones[d]
            kk = np.where(dd['alive'])[0][-1]
            ax.plot(dd['X'][kk], dd['Y'][kk], marker='x', ms=6, mew=1.6, c='k', zorder=7)
            ax.plot(dd['X'][kk], dd['Y'][kk], marker='x', ms=5, mew=0.9, c=DC[d - 1], zorder=8)
        ax.set_xlabel(note, fontsize=7, labelpad=2)
    h = [Line2D([], [], c='k', lw=0.7, label='cell (0.35 m margin)'),
         Line2D([], [], marker='o', ls='', mfc='white', mec='k', ms=4, label='Lloyd generator'),
         Line2D([], [], c='k', lw=0.9, ls=(0, (3, 1.5)), label='merged failed cells'),
         Line2D([], [], c='#808080', lw=1.1, label='recovery lane'),
         Line2D([], [], marker='x', c='k', ls='', label='agent killed'),
         Line2D([], [], marker='o', c='#444444', ls='', ms=4, label='static cylinder')]
    fig.legend(handles=h, loc='lower center', ncol=6, frameon=False, bbox_to_anchor=(0.5, -0.035),
               fontsize=7, handlelength=1.6, columnspacing=1.0)
    fig.subplots_adjust(wspace=0.04, bottom=0.14)
    fig.savefig(FIG / 'fig_concept_high.pdf')
    plt.close(fig)
    out_tex += [f"\\newcommand{{\\ConceptCentroidErr}}{{{max(errs) * 100:.1f}}}",
                f"\\newcommand{{\\ConceptRowsTotal}}{{{sum(len(v) for v in lanes.values())}}}",
                f"\\newcommand{{\\ConceptRecRowsA}}{{{len(stages[0]['lanes'])}}}",
                f"\\newcommand{{\\ConceptRecRowsB}}{{{len(stages[1]['lanes'])}}}",
                f"\\newcommand{{\\ConceptFlownB}}{{{stages[1]['n_flown']}}}",
                f"\\newcommand{{\\ConceptHelpersA}}{{{', '.join(str(x) for x in stages[0]['helpers'])}}}",
                f"\\newcommand{{\\ConceptHelpersB}}{{{', '.join(str(x) for x in stages[1]['helpers'])}}}",
                f"\\newcommand{{\\ConceptDeadA}}{{{stages[0]['dead'][-1]}}}",
                f"\\newcommand{{\\ConceptDeadB}}{{{stages[1]['dead'][-1]}}}"]


# ═════════════════════════════════════════════════════════════════════════
# 2. MID LEVEL
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
    wind = 0.50                                             # coordinator: cbf_wind_accel
    a_st = a_eff_for(CT.CLASS_STATIC, cfg, plant, wind_accel=wind)
    T_d, v_c = cfg.T_d, plant.v_c
    h0 = v_c ** 2 / (2.0 * a_st)          # φ(h)=0 tepat untuk 0 <= h <= v_c^2/2a (lihat catatan phi_zero_h)
    assert float(phi(h0 - 1e-6, a_st, T_d, v_c)) == 0.0 and float(phi(h0 + 1e-2, a_st, T_d, v_c)) > 0.0

    # Konfigurasi CONTOH (bukan momen dari misi): menyapu ke timur 1.6 m/s, silinder terpeta di depan.
    dt = 0.05
    p = np.array([0.0, 0.0]); v = np.array([1.6, 0.0]); u_des = np.array([1.6, 0.0])
    obs = Obstacle(900, np.array([2.45, 0.55]), radius=0.45, kind=CT.CLASS_STATIC)
    agent = CT.AgentState(aid=1, pos=p, vel=v, v_prev_cmd=v.copy(), priority_w=0.25)
    task = CT.Task(v_nom=u_des)
    bounds = Bounds(-50, 50, -50, 50)
    avoid = CBFAvoidance(cfg, plant)
    avoid.set_world([obs], bounds, wind_accel=wind)
    res = avoid.solve_all({1: agent}, {1: task}, dt)[1]
    if res.tier != 0:
        S5.die(f'konfigurasi contoh tidak feasible (tier {res.tier})')
    rs = build_rows(agent, task, [obs], [], bounds, cfg, plant, dt, wind_accel=wind)
    A, b = rs.arrays()
    cls = list(rs.cls)
    d = float(np.hypot(*(p - obs.pos)))
    h = d - (obs.radius + cfg.drone_radius + cfg.delta_static)
    ph_h = float(phi(h, a_st, T_d, v_c))
    u = res.v_safe
    z = (avoid.breaker.bias(agent, u_des, [obs], [], 0.0) + cfg.w_smooth * agent.v_prev_cmd) / (1.0 + cfg.w_smooth)
    n_hat = (p - obs.pos) / d
    print(f'[cbf] a_eff={a_st:.3f} T_d={T_d} v_c={v_c:.3f} h0={h0:.3f} h={h:.3f} phi={ph_h:.3f} '
          f'closing(u_des)={-n_hat @ u_des:.3f} closing(u*)={-n_hat @ u:.3f} u*={u} z={z} limiting={res.limiting}')

    S5.style()
    fig, axes = plt.subplots(1, 2, figsize=(6.7, 2.15), gridspec_kw={'width_ratios': [1.25, 1.0]})
    # ── (a) φ(h) ──
    ax = axes[0]
    hh = np.linspace(-0.25, 2.6, 1200)
    ph = phi(hh, a_st, T_d, v_c)
    s_sweep = 1.6
    h_sweep = (s_sweep ** 2 + 2 * a_st * T_d * s_sweep + v_c ** 2) / (2 * a_st)
    gam = s_sweep / h_sweep                                   # linear αh = γh yang sama di laju sapu
    ax.axhline(0, c='0.6', lw=0.5)
    ax.axvspan(0, h0, color='red', alpha=0.12, lw=0)
    ax.plot(hh, ph, c=C_H, lw=1.4, label=r'$\varphi(h)$, Eq. (3)')
    hp = hh[hh >= 0]
    ax.plot(hp, gam * hp, c=C_L, lw=1.0, ls='--', label=r'linear $\gamma h$, same speed at 1.6 m/s')
    for s_ in (0.5, 1.0, 1.6):
        hreq = (s_ ** 2 + 2 * a_st * T_d * s_ + v_c ** 2) / (2 * a_st)
        ax.plot([hreq], [s_], marker='o', ms=3, c='k', zorder=5)
        ax.annotate(f'{s_:.1f} m/s needs $h\\geq${hreq:.2f} m', (hreq, s_), xytext=(5, -8),
                    textcoords='offset points', fontsize=6.5)
    ax.annotate(f'$\\varphi=0$ for $h\\leq v_c^2/2a$ = {h0:.2f} m', xy=(h0 * 0.5, 0.0), xytext=(0.35, -0.55),
                fontsize=6.5, color='red', arrowprops=dict(arrowstyle='-', color='red', lw=0.5))
    ax.annotate(r'$h<0$: $\gamma_r h$', xy=(-0.13, -0.46), xytext=(-0.23, 0.42), fontsize=6.5, color=C_H,
                arrowprops=dict(arrowstyle='-', color=C_H, lw=0.5))
    ax.set_xlim(-0.25, 2.6); ax.set_ylim(-0.85, 2.3)
    ax.set_xlabel('clearance $h$ (m)'); ax.set_ylabel('approach speed limit (m/s)')
    ax.set_title(f'(a) barrier for static rows ($a$ = {a_st:.2f} m/s$^2$, $T_d$ = {T_d:.2f} s)', pad=3)
    ax.legend(frameon=False, loc='upper left', fontsize=6.8)
    ax.tick_params(length=2)
    # ── (b) QP di ruang kecepatan, di-zoom sekitar u_prev ──
    ax = axes[1]
    cx, cy, R = u_des[0], u_des[1], 0.32
    gx = np.linspace(cx - R, cx + R, 600); gy = np.linspace(cy - R, cy + R, 600)
    UX, UY = np.meshgrid(gx, gy)
    feas = np.ones_like(UX, bool)
    for Ai, bi in zip(A, b):
        feas &= (Ai[0] * UX + Ai[1] * UY) <= bi + 1e-9
    ax.contourf(UX, UY, feas.astype(float), levels=[0.5, 1.5], colors=['#cfe8cf'], zorder=0)
    rate = [(Ai, bi) for Ai, bi, c in zip(A, b, cls) if c == CT.CLASS_RATE]
    # poligon laju: titik sudut dari perpotongan baris berurutan
    verts = []
    for k in range(len(rate)):
        (A1, b1), (A2, b2) = rate[k], rate[(k + 1) % len(rate)]
        verts.append(np.linalg.solve(np.vstack([A1, A2]), np.array([b1, b2])))
    ax.add_patch(MplPolygon(np.asarray(verts), closed=True, fc='none', ec='0.45', lw=0.8, ls=(0, (2, 1.5)), zorder=1))
    for Ai, bi, c in zip(A, b, cls):
        if c != CT.CLASS_STATIC:
            continue
        ys = gy
        xs = (bi - Ai[1] * ys) / Ai[0]
        ax.plot(xs, ys, c='#a23b72', lw=1.4, zorder=2)
        k = len(ys) // 5
        ax.annotate('', xy=(xs[k] + 0.07 * Ai[0], ys[k] + 0.07 * Ai[1]), xytext=(xs[k], ys[k]),
                    arrowprops=dict(arrowstyle='-|>', color='#a23b72', lw=0.8))
        ax.text(xs[k] + 0.08, ys[k] - 0.035, 'infeasible\n(closes too fast)', fontsize=6.3, color='#a23b72', va='top')
    ax.plot([z[0], u[0]], [z[1], u[1]], c='0.25', lw=0.7, ls=':', zorder=3)
    ax.plot(*u_des, marker='o', ms=4.5, mfc='white', mec='k', mew=1.0, zorder=4)
    ax.plot(*u, marker='o', ms=4.5, c=C_H, zorder=5)
    ax.text(u_des[0] + 0.02, u_des[1] + 0.03, r'$\mathbf{u}_{\mathrm{des}}=\mathbf{u}_{\mathrm{prev}}$', fontsize=7.5)
    ax.text(u[0] - 0.03, u[1] - 0.05, r'$\mathbf{u}^\star$', fontsize=8, color=C_H, ha='right', va='top')
    ax.text(cx + 0.19, cy + 0.15, 'rate limit\n' + r'$\|\mathbf{u}-\mathbf{u}_{\mathrm{prev}}\|\leq a_{\max}\Delta t$',
            fontsize=6.3, color='0.35', va='bottom', ha='center')
    ax.set_xlim(cx - R, cx + R); ax.set_ylim(cy - R, cy + R); ax.set_aspect('equal')
    ax.set_xlabel('$u_x$ (m/s)'); ax.set_ylabel('$u_y$ (m/s)', labelpad=1)
    ax.set_title('(b) one QP solve in velocity space', pad=3)
    ax.tick_params(length=2)
    fig.subplots_adjust(wspace=0.30)
    fig.savefig(FIG / 'fig_concept_cbf.pdf')
    plt.close(fig)

    out_tex += [f'\\newcommand{{\\CbfAeff}}{{{a_st:.2f}}}', f'\\newcommand{{\\CbfHzero}}{{{h0:.2f}}}',
                f'\\newcommand{{\\CbfHsweep}}{{{h_sweep:.2f}}}',
                f'\\newcommand{{\\CbfExDist}}{{{d:.2f}}}', f'\\newcommand{{\\CbfExH}}{{{h:.2f}}}',
                f'\\newcommand{{\\CbfExPhi}}{{{ph_h:.2f}}}',
                f'\\newcommand{{\\CbfExCloseDes}}{{{-n_hat @ u_des:.2f}}}',
                f'\\newcommand{{\\CbfExCloseStar}}{{{-n_hat @ u:.2f}}}',
                f'\\newcommand{{\\CbfExUx}}{{{u[0]:.2f}}}', f'\\newcommand{{\\CbfExUy}}{{{u[1]:.2f}}}',
                f'\\newcommand{{\\CbfRate}}{{{plant.a_max * dt:.2f}}}']


def main():
    out = ['% DIHASILKAN OTOMATIS oleh swarm_ws/analysis/concept_figures.py — JANGAN DIEDIT.']
    fig_high(out)
    fig_cbf(out)
    (GEN / 'concept_numbers.tex').write_text('\n'.join(out) + '\n')
    print('ok')


if __name__ == '__main__':
    main()
