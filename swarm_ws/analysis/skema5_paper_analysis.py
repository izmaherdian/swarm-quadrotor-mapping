#!/usr/bin/env python3
"""
Analisis Skema 5 (8 run video) untuk full paper EPIC 2026.

SATU-SATUNYA sumber angka dan grafik hasil di `docs/Full Paper - EPIC/`.
Setiap nilai diturunkan dari:
  * 56 CSV  `results/skema5_video_runs/<region>/<ctrl>/flight_data_log_*.csv`
    (ditulis node low-level, 20 Hz; baris pasca-kill 250 Hz dengan RPM=0)
  * 8 log   `.../<ctrl>/coordinator.log` (salinan ~/.ros/log, lihat MANIFEST.json)
  * solver gain `swarm_low_level/solver_pid_{lqr,hinf}.py` (analisis linear)

Tidak ada nilai pengganti: data hilang -> `n/a` + exit(1). Tidak ada angka
literal di judul figure.

Pakai:
    cd swarm_ws
    python3 analysis/skema5_paper_analysis.py

Keluaran:
    docs/Full Paper - EPIC/generated/{metrics.json,numbers.tex,table_results.tex,audit.md}
    docs/Full Paper - EPIC/figures/fig_{traj,coverage,tracking,events}.pdf
"""
import json
import math
import re
import sys
from pathlib import Path

import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt                                  # noqa: E402
from matplotlib.patches import Circle, Polygon as MplPolygon     # noqa: E402
from matplotlib.lines import Line2D                              # noqa: E402
from scipy.linalg import solve_continuous_lyapunov               # noqa: E402

REPO = Path(__file__).resolve().parents[2]
WS = REPO / 'swarm_ws'
sys.path.insert(0, str(WS / 'src' / 'swarm_high_level'))
sys.path.insert(0, str(WS / 'src' / 'swarm_low_level'))

from swarm_high_level.world.region import REGION_PRESETS, grid_region_mask   # noqa: E402
from swarm_high_level.world.obstacles import OBSTACLES_BY_REGION, OBSTACLE_RADIUS  # noqa: E402
from swarm_high_level.metrics.region_report import sweeping_mask             # noqa: E402
from swarm_low_level.solver_pid_lqr import PIDLQRSolver                      # noqa: E402
from swarm_low_level.solver_pid_hinf import PIDHinfSolver                    # noqa: E402

DATA = WS / 'src' / 'swarm_sim' / 'results' / 'skema5_video_runs'
PAPER = REPO / 'docs' / 'Full Paper - EPIC'
GEN = PAPER / 'generated'
FIG = PAPER / 'figures'

REGIONS = [('rect', '01_rect'), ('l_shape', '02_l-shape'),
           ('u_shape', '03_u-shape'), ('plus', '04_plus')]
CTRLS = [('hinf', 'pid_hinf'), ('lqr', 'pid_lqr')]
REGION_LABEL = {'rect': 'rect', 'l_shape': 'L-shape', 'u_shape': 'U-shape', 'plus': 'plus'}
CTRL_LABEL = {'hinf': r'PID-$H_\infty$', 'lqr': 'PID-LQR'}
MACRO_REGION = {'rect': 'Rect', 'l_shape': 'Lshape', 'u_shape': 'Ushape', 'plus': 'Plus'}
MACRO_CTRL = {'hinf': 'Hinf', 'lqr': 'Lqr'}

# ── Konstanta yang dibaca dari kode (bukan dipilih di sini) ─────────────────
T_SETTLE = 20.0            # region_report.TAKEOFF_SETTLE_S
Z_AIR = 0.5                # baris "di udara" untuk statistik
Z_COV = 0.8                # coordinator.update_coverage: pos[2] >= 0.8
SENSOR_R = 0.95            # coordinator.sensor_radius
GRID_N = 100               # coordinator.grid_n
OBS_MASK_PAD = 0.05        # coordinator obstacle_mask: rad + 0.05
# Geometri tabrakan iris_base/model.sdf: body box 0.47x0.47 m, rotor r=0.128 m
# pada lengan 0.22 m (sin45 -> 0.1556 m). Selubung kontak di bidang xy:
BODY_HALF = 0.47 / 2.0                         # 0.235 m (sisi box)
ROTOR_REACH = 0.22 + 0.128                     # 0.348 m (ujung rotor)
CBF_DRONE_R = 0.22                             # cbf types.drone_radius
KILL_TILT_EVENT = 30.0     # deg: ambang "upset" untuk analisis kejadian

# Lintasan nominal rintangan dinamis (coordinator.update_dynamic_obstacles).
DYN_PATHS = {
    'default': [((-10.0, 10.0), (10.0, -10.0)), ((-10.0, -10.0), (10.0, 10.0))],
    'plus': [((-11.0, 13.0), (13.0, -11.0)), ((-11.0, -13.0), (13.0, 11.0))],
}


def die(msg):
    print(f'[FATAL] {msg}', file=sys.stderr)
    sys.exit(1)


# ═════════════════════════════════════════════════════════════════════════
# 1. LOADER
# ═════════════════════════════════════════════════════════════════════════
def load_csv(path):
    a = np.genfromtxt(path, delimiter=',', names=True)
    if a.size < 100:
        die(f'CSV terlalu pendek: {path}')
    d = {k: np.asarray(a[k], float) for k in a.dtype.names}
    dead = (d['RPM_0'] == 0.0) & (d['RPM_1'] == 0.0) & (d['Time_s'] > 5.0)
    d['alive'] = ~dead
    d['t_kill'] = float(d['Time_s'][dead][0]) if dead.any() else None
    return d


def subset(d, m):
    return {k: (v[m] if isinstance(v, np.ndarray) else v) for k, v in d.items()}


def load_run(region, folder, ctrl, sub):
    run_dir = DATA / folder / sub
    drones = {}
    for p in sorted(run_dir.glob(f'flight_data_log_{ctrl}_iris_*.csv')):
        did = int(p.stem.split('_')[-1])
        drones[did] = load_csv(p)
    if len(drones) != 7:
        die(f'{run_dir}: {len(drones)} CSV, harus 7')
    log = run_dir / 'coordinator.log'
    if not log.is_file():
        die(f'{log} tidak ada (jalankan Langkah 0: salin log coordinator)')
    return drones, parse_log(log)


# ═════════════════════════════════════════════════════════════════════════
# 2. PARSER LOG COORDINATOR
# ═════════════════════════════════════════════════════════════════════════
RE_STATUS = re.compile(r'\[STATUS\] Cov:\s*([\d.]+)% \| d_min:\s*([\d.]+)m(?: \| peta (\d+)/(\d+))?')
RE_FAULT = re.compile(r'\[AUTO FAULT\] Coverage ([\d.]+)% >= ([\d.]+)%: Mematikan iris_(\d+)')
RE_HELP = re.compile(r'\[HELPER ALLOCATION\] Terpilih (\d+) drone helper: \[([^\]]*)\] untuk menangani (\d+) baris')
RE_SUCC = re.compile(r'\[SWARM SUCCESS\] Target Coverage ([\d.]+)% Tercapai! .*Durasi Misi: ([\d.]+)s .*d_min\): ([\d.]+)m')
RE_CBF = re.compile(r'CBF-QP: (\d+) solve \| T0 (\d+) T1 (\d+) T2 (\d+) T3 (\d+) \| P\(tier>0\)=([\d.]+)% \| slack_maks=([\d.]+)')
RE_TOTAL = re.compile(r'Total Waktu Misi: ([\d.]+)s .*Coverage final: ([\d.]+)%')
RE_ABORT = re.compile(r'\[AUTO-EXIT: FAILURE\] Drone iris_(\d+) jatuh tak terkendali ke Z=([\d.]+)m')
RE_TIER2 = re.compile(r'\[iris_(\d+)\] QP Tier 2 \(slack=([\d.]+), pembatas=(\w+):?(\d*), h_min=(-?[\d.]+)m\)')
RE_PLANT = re.compile(r'PlantModel\(k_v=([\d.]+)/s, a_max=([\d.]+) m/s\^2, T_lead=([\d.]+) s, v_c=([\d.]+) m/s\)')
RE_CELL = re.compile(r'-> \[iris_(\d+)\] Sel \((\d+) simpul\) \| (\d+) Baris \| Start: \(([-\d.]+), ([-\d.]+)\) \| Centroid: \(([-\d.]+), ([-\d.]+)\)')
RE_PETA = re.compile(r'\(([+-][\d.]+),([+-][\d.]+)\)r([\d.]+)n(\d+)d([\d.]+)(M?)')
RE_RECROWS = re.compile(r'\[DYNAMIC RECOVERY\] Terbentuk (\d+) baris')


def parse_log(path):
    out = {'status': [], 'kills': [], 'tier2': [], 'cells': {}, 'peta': [],
           'success': None, 'cbf': None, 'total': None, 'abort': None, 'plant': None}
    for line in Path(path).read_text(errors='replace').splitlines():
        t = float(len(out['status']))          # 1 baris STATUS = 1 s sim (divalidasi di audit)
        if m := RE_STATUS.search(line):
            out['status'].append((float(m.group(1)), float(m.group(2))))
        elif m := RE_FAULT.search(line):
            out['kills'].append({'drone': int(m.group(3)), 'cov': float(m.group(1)),
                                 'trigger': float(m.group(2)), 't_log': t})
        elif (m := RE_HELP.search(line)) and out['kills']:
            out['kills'][-1]['helpers'] = [int(x) for x in re.findall(r'iris_(\d+)', m.group(2))]
            out['kills'][-1]['rows'] = int(m.group(3))
        elif m := RE_SUCC.search(line):
            out['success'] = {'cov': float(m.group(1)), 't': float(m.group(2)), 'dmin': float(m.group(3))}
        elif m := RE_CBF.search(line):
            g = [float(x) for x in m.groups()]
            out['cbf'] = {'solves': int(g[0]), 'T0': int(g[1]), 'T1': int(g[2]), 'T2': int(g[3]),
                          'T3': int(g[4]), 'p_tier': g[5], 'slack_max': g[6]}
        elif m := RE_TOTAL.search(line):
            out['total'] = {'t': float(m.group(1)), 'cov': float(m.group(2))}
        elif m := RE_ABORT.search(line):
            out['abort'] = {'drone': int(m.group(1)), 'z': float(m.group(2)), 't_log': t}
        elif m := RE_TIER2.search(line):
            out['tier2'].append({'t_log': t, 'drone': int(m.group(1)), 'slack': float(m.group(2)),
                                 'limiter': m.group(3), 'id': m.group(4), 'h': float(m.group(5))})
        elif m := RE_PLANT.search(line):
            out['plant'] = dict(zip(['k_v', 'a_max', 'T_lead', 'v_c'], map(float, m.groups())))
        elif m := RE_CELL.search(line):
            out['cells'][int(m.group(1))] = {'vertices': int(m.group(2)), 'rows': int(m.group(3)),
                                             'start': (float(m.group(4)), float(m.group(5))),
                                             'centroid': (float(m.group(6)), float(m.group(7)))}
        elif '[PETA]' in line:
            out['peta'] = [(float(a), float(b), float(r), int(n), float(dr), mv == 'M')
                           for a, b, r, n, dr, mv in RE_PETA.findall(line)]
    if not out['status']:
        die(f'{path}: tidak ada baris STATUS')
    return out


# ═════════════════════════════════════════════════════════════════════════
# 3. METRIK
# ═════════════════════════════════════════════════════════════════════════
def dt_of(t):
    dt = np.diff(t, append=t[-1])
    return np.clip(dt, 0.0, 0.2)


def coverage_offline(region, drones):
    """Batas ATAS coverage dari CSV (tanpa kolom state & tanpa penghapusan sel korban)."""
    poly_pts = REGION_PRESETS[region]
    from shapely.geometry import Polygon
    poly = Polygon(poly_pts)
    rx0, ry0, rx1, ry1 = poly.bounds
    x_min, y_min = rx0 - 1.0, ry0 - 1.0
    dx = (rx1 + 1.0 - x_min) / GRID_N
    dy = (ry1 + 1.0 - y_min) / GRID_N
    region_mask = grid_region_mask(poly, x_min, y_min, dx, dy, GRID_N)
    obs_mask = np.zeros_like(region_mask)
    ii, jj = np.meshgrid(np.arange(GRID_N), np.arange(GRID_N), indexing='ij')
    cx = x_min + (ii + 0.5) * dx
    cy = y_min + (jj + 0.5) * dy
    for _, ox, oy in OBSTACLES_BY_REGION[region]:
        obs_mask |= (cx - ox) ** 2 + (cy - oy) ** 2 <= (OBSTACLE_RADIUS + OBS_MASK_PAD) ** 2
    cov = np.zeros_like(region_mask)
    rc = int(math.ceil(SENSOR_R / dx))
    for d in drones.values():
        m = d['alive'] & (d['Z'] >= Z_COV)
        pts = np.unique(np.round(np.c_[d['X'][m], d['Y'][m]] / 0.05) * 0.05, axis=0)
        for px, py in pts:
            ci, cj = int((px - x_min) / dx), int((py - y_min) / dy)
            i0, i1 = max(0, ci - rc), min(GRID_N, ci + rc + 1)
            j0, j1 = max(0, cj - rc), min(GRID_N, cj + rc + 1)
            if i0 >= i1 or j0 >= j1:
                continue
            sub = (cx[i0:i1, j0:j1] - px) ** 2 + (cy[i0:i1, j0:j1] - py) ** 2 <= SENSOR_R ** 2
            cov[i0:i1, j0:j1] |= sub
    valid = region_mask & ~obs_mask
    return 100.0 * float((cov & valid).sum()) / float(valid.sum()), int(valid.sum())


def v2v_min(drones, t_end):
    """Definisi coordinator (baris 1803-1813): kedua drone hidup, Z >= 1.50 m,
    setelah Voronoi direncanakan (semua drone > 1.30 m)."""
    t_plan = max(float(d['Time_s'][np.argmax(d['Z'] > 1.30)]) for d in drones.values())
    tg = np.arange(t_plan, t_end, 0.05)
    P = {}
    for did, d in drones.items():
        m = d['alive'] & (d['Z'] >= 1.50)
        if m.sum() < 10:
            continue
        t = d['Time_s'][m]
        xs = np.interp(tg, t, d['X'][m], left=np.nan, right=np.nan)
        ys = np.interp(tg, t, d['Y'][m], left=np.nan, right=np.nan)
        # Jangan interpolasi melewati celah (mis. setelah kill atau saat Z<Z_COV)
        gap = np.interp(tg, t[1:], np.diff(t), left=np.inf, right=np.inf) > 0.5
        xs[gap] = np.nan
        ys[gap] = np.nan
        P[did] = (xs, ys)
    best = (np.inf, None, None, None)
    ids = sorted(P)
    for a in range(len(ids)):
        for b in range(a + 1, len(ids)):
            xa, ya = P[ids[a]]
            xb, yb = P[ids[b]]
            dist = np.hypot(xa - xb, ya - yb)
            if np.all(np.isnan(dist)):
                continue
            k = int(np.nanargmin(dist))
            if dist[k] < best[0]:
                best = (float(dist[k]), ids[a], ids[b], float(tg[k]))
    return best


def static_clearance(region, drones):
    """Jarak pusat drone ke PERMUKAAN silinder statis (d - 0.40), dari tabel truth."""
    best = (np.inf, None, None, None)
    for did, d in drones.items():
        m = d['alive'] & (d['Z'] > Z_AIR)
        for oid, ox, oy in OBSTACLES_BY_REGION[region]:
            dist = np.hypot(d['X'][m] - ox, d['Y'][m] - oy) - OBSTACLE_RADIUS
            k = int(np.argmin(dist))
            if dist[k] < best[0]:
                best = (float(dist[k]), did, oid, float(d['Time_s'][m][k]))
    return best


def nearest_static_surface(region, x, y):
    obs = np.array([(ox, oy) for _, ox, oy in OBSTACLES_BY_REGION[region]])
    dd = np.hypot(x[:, None] - obs[None, :, 0], y[:, None] - obs[None, :, 1]) - OBSTACLE_RADIUS
    return dd.min(axis=1)


def run_metrics(region, ctrl, drones, log):
    r = {'region': region, 'ctrl': ctrl}
    # ── misi (log) ──
    r['aborted'] = log['abort'] is not None
    r['t_total'] = log['total']['t'] if log['total'] else None
    r['cov_final'] = log['total']['cov'] if log['total'] else None
    r['t97'] = log['success']['t'] if log['success'] else None
    r['cov_at_success'] = log['success']['cov'] if log['success'] else None
    r['dmin_log'] = log['success']['dmin'] if log['success'] else min(s[1] for s in log['status'])
    r['n_status'] = len(log['status'])
    r['cov_last_status'] = log['status'][-1][0]
    r['cbf'] = log['cbf']
    r['plant'] = log['plant']
    r['kills_log'] = log['kills']
    r['abort'] = log['abort']
    r['cells'] = {str(k): v for k, v in log['cells'].items()}
    tier2 = log['tier2']
    r['tier2_count'] = len(tier2)
    r['tier2_by_limiter'] = {}
    for e in tier2:
        r['tier2_by_limiter'][e['limiter']] = r['tier2_by_limiter'].get(e['limiter'], 0) + 1
    dyn = [e['h'] for e in tier2 if e['limiter'] == 'dynamic']
    # PERHATIAN: h_min di log = minimum atas SEMUA baris ber-h (rintangan, v2v_hard, dinding,
    # dinding henti) pada solve itu, BUKAN khusus baris rintangan bergerak (constraints.RowSet.add).
    # Hanya boleh dibaca sebagai 'suatu margin dimasuki saat baris bergerak mengikat'.
    r['h_dyn_min_logged'] = min(dyn) if dyn else None
    # PETA terakhir: kecocokan dengan truth untuk track terkonfirmasi & tidak bergerak
    truth = OBSTACLES_BY_REGION[region]
    errs = []
    for x, y, rad, n, _dr, moving in log['peta']:
        if moving or n < 12:
            continue
        dd = [math.hypot(x - ox, y - oy) for _, ox, oy in truth]
        k = int(np.argmin(dd))
        errs.append({'truth_id': truth[k][0], 'pos_err': dd[k], 'r_err': rad - OBSTACLE_RADIUS, 'hits': n})
    r['peta_tracks'] = errs
    matched = [e for e in errs if e['pos_err'] < 0.5]
    r['peta_matched_ids'] = sorted({e['truth_id'] for e in matched})
    r['peta_pos_err_max'] = max((e['pos_err'] for e in matched), default=None)
    r['peta_r_err_absmax'] = max((abs(e['r_err']) for e in matched), default=None)
    r['peta_spurious'] = len(errs) - len(matched)

    # ── kill dari CSV ──
    r['kills_csv'] = {str(did): d['t_kill'] for did, d in drones.items() if d['t_kill'] is not None}
    offs = []
    for k in log['kills']:
        tk = drones[k['drone']]['t_kill']
        if tk is not None:
            offs.append(tk - k['t_log'])
    r['t_offset_csv_minus_log'] = float(np.median(offs)) if offs else None
    r['t_end_csv'] = float(max(d['Time_s'][-1] for d in drones.values()))

    # ── tracking/attitude/effort (CSV, drone hidup) ──
    lat, lat_all, vs, alt, tilt_sw, eff, dur, sat, gate = [], [], [], [], [], [], [], [], []
    tilt_max = (0.0, None, None)
    for did, d0 in drones.items():
        d = subset(d0, d0['alive'])
        m = (d['Time_s'] >= T_SETTLE) & (d['Z'] > Z_AIR)
        if m.sum() < 50:
            continue
        alt.append(d['Ref_Z'][m] - d['Z'][m])
        tilt = np.maximum(np.abs(d['Roll_deg']), np.abs(d['Pitch_deg']))
        k = int(np.argmax(np.where(m, tilt, -1)))
        if tilt[k] > tilt_max[0]:
            tilt_max = (float(tilt[k]), did, float(d['Time_s'][k]))
        dt = dt_of(d['Time_s'])
        eff.append(float(np.sum((d['tau_x'][m] ** 2 + d['tau_y'][m] ** 2) * dt[m])))
        dur.append(float(np.sum(dt[m])))
        R = np.stack([d[f'RPM_{i}'][m] for i in range(4)])
        sat.append(((R >= 1049.0) | (R <= 251.0)).any(axis=0))
        sw = sweeping_mask(d) & (d['Z'] > Z_AIR)
        if sw.sum() < 20:
            continue
        th = np.radians(d['Ref_Yaw'][sw])
        ex = d['X'][sw] - d['Ref_X'][sw]
        ey = d['Y'][sw] - d['Ref_Y'][sw]
        lat_all.append(-ex * np.sin(th) + ey * np.cos(th))
        vs.append(np.hypot(d['vx'][sw], d['vy'][sw]))
        tilt_sw.append(tilt[sw])
        ew = sw & (np.abs(np.sin(np.radians(d['Ref_Yaw']))) < 0.05)
        lat.append(d['Y'][ew] - d['Ref_Y'][ew])
        # jendela integrator posisi LQR (pid_lqr_node.py:449): |e_x|,|e_y| < 0.25 m
        gate.append((np.abs(d['X'][ew] - d['Ref_X'][ew]) < 0.25) & (np.abs(d['Y'][ew] - d['Ref_Y'][ew]) < 0.25))
    L = np.concatenate(lat)
    LA = np.concatenate(lat_all)
    A = np.concatenate(alt)
    rms = lambda x: float(np.sqrt(np.mean(np.square(x))))   # noqa: E731
    r['lat_rms_cm'] = 100 * rms(L)
    r['lat_p95_cm'] = 100 * float(np.percentile(np.abs(L), 95))
    r['lat_p50_cm'] = 100 * float(np.percentile(np.abs(L), 50))
    r['lat_n'] = int(L.size)
    r['lat_mean_cm'] = 100 * float(np.mean(L))          # bertanda: negatif = ke -y (searah angin rata-rata)
    r['lat_std_cm'] = 100 * float(np.std(L))
    r['integ_window_pct'] = 100 * float(np.mean(np.concatenate(gate)))
    r['lat_samples_cm'] = (100 * np.abs(L)).tolist()
    r['ct_all_rms_cm'] = 100 * rms(LA)
    r['v_sweep'] = float(np.mean(np.concatenate(vs)))
    r['alt_rms_cm'] = 100 * rms(A)
    r['tilt_sweep_p95'] = float(np.percentile(np.concatenate(tilt_sw), 95))
    r['tilt_max'] = tilt_max[0]
    r['tilt_max_drone'] = tilt_max[1]
    r['tilt_max_t'] = tilt_max[2]
    r['effort_rate'] = float(np.sum(eff) / np.sum(dur))
    r['sat_pct'] = 100 * float(np.mean(np.concatenate(sat)))

    # ── keselamatan (CSV + truth) ──
    c = static_clearance(region, drones)
    r['obs_surface_min'] = c[0]
    r['obs_surface_min_drone'], r['obs_surface_min_id'], r['obs_surface_min_t'] = c[1], c[2], c[3]
    r['obs_contact'] = ('certain' if c[0] < BODY_HALF else
                        'possible' if c[0] < ROTOR_REACH else 'none')
    v = v2v_min(drones, r['t_end_csv'])
    r['v2v_min_csv'], r['v2v_pair'], r['v2v_t'] = v[0], [v[1], v[2]], v[3]

    # ── kejadian upset (drone hidup, tilt > ambang) ──
    events = []
    for did, d0 in drones.items():
        d = subset(d0, d0['alive'])
        tilt = np.maximum(np.abs(d['Roll_deg']), np.abs(d['Pitch_deg']))
        m = (d['Time_s'] >= T_SETTLE) & (tilt > KILL_TILT_EVENT)
        if not m.any():
            continue
        k = int(np.argmax(np.where(m, tilt, -1)))
        tk = d['Time_s'][k]
        w = (d['Time_s'] >= tk - 5) & (d['Time_s'] <= tk + 5)
        after = d['Time_s'] >= tk + 5
        surf = nearest_static_surface(region, d['X'][w], d['Y'][w])
        events.append({'drone': did, 't': float(tk), 'tilt': float(tilt[k]),
                       'x': float(d['X'][k]), 'y': float(d['Y'][k]),
                       'z_min_window': float(d['Z'][w].min()),
                       'surface_min_window': float(surf.min()),
                       'z_after5s': float(d['Z'][after][0]) if after.any() else None,
                       'recovered': bool(after.any() and d['Z'][after][0] > 1.5)})
    # tier-2 terikat silinder statis sebelum puncak tilt (waktu log = waktu CSV - offset)
    off = r['t_offset_csv_minus_log'] or 0.0
    for ev in events:
        tl = ev['t'] - off
        ts = [e['t_log'] for e in log['tier2']
              if e['drone'] == ev['drone'] and e['limiter'] == 'static' and tl - 8 <= e['t_log'] <= tl + 1]
        ev['tier2_static_span_s'] = (max(ts) - min(ts) + 1.0) if ts else 0.0
        ev['tier2_static_hmin'] = min((e['h'] for e in log['tier2'] if e['drone'] == ev['drone']
                                       and e['limiter'] == 'static' and tl - 8 <= e['t_log'] <= tl + 1), default=None)
    r['events'] = events

    # ── angin (drone hidup pertama, t >= 10 s) ──
    d0 = drones[min(did for did, d in drones.items() if d['t_kill'] is None)]
    m = d0['Time_s'] >= 10.0
    r['wind'] = {'mean': [float(d0[f'Wind_{a}'][m].mean()) for a in 'XYZ'],
                 'std': [float(d0[f'Wind_{a}'][m].std()) for a in 'XYZ'],
                 'max_h': float(np.hypot(d0['Wind_X'][m], d0['Wind_Y'][m]).max())}

    # ── coverage offline (batas atas) ──
    r['cov_offline_upper'], r['n_valid_cells'] = coverage_offline(region, drones)
    return r


# ═════════════════════════════════════════════════════════════════════════
# 4. ANALISIS DESAIN LINEAR (tanpa simulasi): gangguan percepatan -> posisi
# ═════════════════════════════════════════════════════════════════════════
def design_analysis():
    params = yaml.safe_load((WS / 'src' / 'swarm_low_level' / 'config' / 'quadrotor_params.yaml').read_text())
    phys = params['physics']
    g, Iy = float(phys['g']), float(phys['iy'])
    designs = {'lqr': PIDLQRSolver(phys).compute_all_gains(),
               'hinf': PIDHinfSolver(phys).get_all_gains()}
    tau_w = 0.5      # dryden_wind_node: tau
    out = {}
    w = np.logspace(-2, 3, 6000)
    for name, G in designs.items():
        Kp, Ki, Kd = (float(G['x_outer'][k]) for k in ('Kp', 'Ki', 'Kd'))
        Kpi, Kdi = float(G['x_inner']['Kp']), float(G['x_inner']['Kd'])  # Ki dalam = 0 di node
        # state [x, v, th, q, xi]; theta_ref = -Kp x + Ki xi - Kd v ; tau = Kpi(th_ref - th) - Kdi q
        A = np.zeros((5, 5))
        A[0, 1] = 1.0
        A[1, 2] = g
        A[2, 3] = 1.0
        A[3, :] = np.array([-Kpi * Kp, -Kpi * Kd, -Kpi, -Kdi, Kpi * Ki]) / Iy
        A[4, 0] = -1.0
        B = np.zeros((5, 1)); B[1, 0] = 1.0
        C = np.zeros((1, 5)); C[0, 0] = 1.0
        poles = np.linalg.eigvals(A)
        H = np.array([abs((C @ np.linalg.solve(1j * wi * np.eye(5) - A, B))[0, 0]) for wi in w])
        k = int(np.argmax(H))
        # respons kovarians terhadap angin Gauss-Markov berunit (percepatan, var=1)
        Aa = np.zeros((6, 6)); Aa[:5, :5] = A; Aa[:5, 5] = B[:, 0]; Aa[5, 5] = -1.0 / tau_w
        Ba = np.zeros((6, 1)); Ba[5, 0] = math.sqrt(2.0 / tau_w)
        Pcov = solve_continuous_lyapunov(Aa, -Ba @ Ba.T)
        # margin loop luar (loop dalam tertutup): L = C(s) * T_in(s) * g / s^2
        s = 1j * w
        Cs = Kp + Ki / s + Kd * s
        Tin = Kpi / (Iy * s ** 2 + Kdi * s + Kpi)
        Ls = Cs * Tin * g / s ** 2
        mag = np.abs(Ls)
        ph = np.degrees(np.unwrap(np.angle(Ls)))
        ph = ph - 360.0 * np.round((ph[0] + 270.0) / 360.0)     # tiga integrator: fase DC = -270 deg
        ic = np.where(np.diff(np.sign(mag - 1.0)) != 0)[0]
        wc = float(w[ic[-1]]) if len(ic) else None
        pm = float(180.0 + ph[ic[-1]]) if len(ic) else None
        ip = np.where(np.diff(np.sign(ph + 180.0)) != 0)[0]
        # loop kondisional-stabil: margin atas = persilangan -180 frekuensi tinggi
        gm = float(-20 * np.log10(mag[ip[-1]])) if len(ip) else None
        gm_low = float(-20 * np.log10(mag[ip[0]])) if len(ip) > 1 else None
        out[name] = {'gains_outer': {'Kp': Kp, 'Ki': Ki, 'Kd': Kd},
                     'gains_inner': {'Kp': Kpi, 'Kd': Kdi},
                     'stable': bool(np.all(poles.real < 0)),
                     'hinf_norm_w_to_x': float(H[k]), 'hinf_peak_rad_s': float(w[k]),
                     'gain_at_wind_corner': float(H[np.argmin(np.abs(w - 1.0 / tau_w))]),
                     'dc_gain': float(H[0]),
                     'sigma_x_per_unit_accel_gm': float(math.sqrt(Pcov[0, 0])),
                     'crossover_rad_s': wc, 'phase_margin_deg': pm, 'gain_margin_db': gm, 'gain_margin_low_db': gm_low,
                     'freq': w.tolist(), 'mag': H.tolist()}
    return out


# ═════════════════════════════════════════════════════════════════════════
# 5. FIGURE
# ═════════════════════════════════════════════════════════════════════════
def style():
    plt.rcParams.update({
        'font.family': 'serif', 'font.serif': ['Times New Roman', 'TeX Gyre Termes', 'Nimbus Roman'],
        'mathtext.fontset': 'stix', 'font.size': 8, 'axes.titlesize': 8, 'axes.labelsize': 8,
        'xtick.labelsize': 7, 'ytick.labelsize': 7, 'legend.fontsize': 7,
        'axes.linewidth': 0.6, 'lines.linewidth': 0.9, 'pdf.fonttype': 42,
        'savefig.bbox': 'tight', 'savefig.pad_inches': 0.02})


C_HINF, C_LQR = '#1f5fa8', '#d9731a'
DRONE_COLORS = ['#1b9e77', '#d95f02', '#7570b3', '#e7298a', '#66a61e', '#a6761d', '#377eb8']


def fig_traj(runs):
    fig, axes = plt.subplots(2, 4, figsize=(6.7, 2.8), sharex=True, sharey=True)
    for ci, (ctrl, _) in enumerate(CTRLS):
        for ri, (region, _) in enumerate(REGIONS):
            ax = axes[ci, ri]
            R = runs[(region, ctrl)]
            ax.add_patch(MplPolygon(REGION_PRESETS[region], closed=True, fc='#f1f1f1', ec='k', lw=0.7, zorder=0))
            for (p0, p1) in DYN_PATHS.get(region, DYN_PATHS['default']):
                ax.plot([p0[0], p1[0]], [p0[1], p1[1]], ls=(0, (3, 2)), c='#888888', lw=0.6, zorder=1)
            for _, ox, oy in OBSTACLES_BY_REGION[region]:
                ax.add_patch(Circle((ox, oy), OBSTACLE_RADIUS, fc='#333333', ec='none', zorder=3))
            for did, d in R['drones'].items():
                m = d['alive'] & (d['Z'] > Z_AIR)
                ax.plot(d['X'][m], d['Y'][m], c=DRONE_COLORS[did - 1], lw=0.45, zorder=2)
                if d['t_kill'] is not None:
                    k = np.where(d['alive'])[0][-1]
                    ax.plot(d['X'][k], d['Y'][k], marker='x', ms=5, mew=1.3, c='k', zorder=5)
                    ax.plot(d['X'][k], d['Y'][k], marker='x', ms=4, mew=0.8, c=DRONE_COLORS[did - 1], zorder=6)
            M = R['metrics']
            for ev in M['events']:
                mk = 'v' if ev['recovered'] else '*'
                ax.plot(ev['x'], ev['y'], marker=mk, ms=6 if mk == '*' else 4.5, mfc='red', mec='k', mew=0.4, zorder=7)
            ax.set_aspect('equal')
            ax.set_xlim(-15.5, 15.5)
            ax.set_ylim(-19.0, 15.5)
            if ci == 0:
                ax.set_title(REGION_LABEL[region])
            if ri == 0:
                ax.set_ylabel(f"{CTRL_LABEL[ctrl]}\n$y$ (m)")
            if ci == 1:
                ax.set_xlabel('$x$ (m)')
            ax.tick_params(length=2)
            ax.set_xticks([-10, 0, 10])
            ax.set_yticks([-10, 0, 10])
    handles = [Line2D([], [], c=DRONE_COLORS[i], lw=1.2, label=f'iris_{i + 1}') for i in range(7)]
    handles += [Line2D([], [], marker='x', c='k', ls='', label='agent killed'),
                Line2D([], [], marker='o', c='#333333', ls='', ms=4, label='static obstacle'),
                Line2D([], [], c='#888888', ls=(0, (3, 2)), label='dynamic-obstacle path'),
                Line2D([], [], marker='*', mfc='red', mec='k', ls='', ms=6, label='upset, crashed'),
                Line2D([], [], marker='v', mfc='red', mec='k', ls='', ms=4.5, label='upset, recovered')]
    fig.legend(handles=handles, loc='center left', ncol=1, frameon=False, bbox_to_anchor=(0.80, 0.52),
               handlelength=1.6, labelspacing=0.45, borderaxespad=0.0)
    fig.subplots_adjust(left=0.07, right=0.80, wspace=0.05, hspace=0.06)
    fig.savefig(FIG / 'fig_traj.pdf')
    plt.close(fig)


def fig_coverage(runs):
    fig, axes = plt.subplots(1, 4, figsize=(6.7, 1.45), sharey=True)
    for ri, (region, _) in enumerate(REGIONS):
        ax = axes[ri]
        for ctrl, col, ls in (('hinf', C_HINF, '-'), ('lqr', C_LQR, '--')):
            log = runs[(region, ctrl)]['log']
            cov = np.array([s[0] for s in log['status']])
            t = np.arange(len(cov), dtype=float)
            ax.plot(t, cov, c=col, ls=ls, lw=1.0, label=CTRL_LABEL[ctrl])
            for k in log['kills']:
                i = int(k['t_log'])
                ax.plot(t[min(i, len(t) - 1)], cov[min(i, len(t) - 1)], marker='v', ms=4, c=col, mec='k', mew=0.3)
            if log['abort']:
                ax.plot(t[-1], cov[-1], marker='X', ms=6, c='red', mec='k', mew=0.4)
        ax.axhline(97.0, c='0.5', lw=0.5, ls=':')
        ax.set_title(REGION_LABEL[region])
        ax.set_xlabel('mission time (s)')
        ax.set_xlim(0, None)
        ax.set_ylim(0, 102)
        ax.tick_params(length=2)
    axes[0].set_ylabel('coverage (%)')
    h = [Line2D([], [], c=C_HINF, lw=1.0, label=CTRL_LABEL['hinf']),
         Line2D([], [], c=C_LQR, lw=1.0, ls='--', label=CTRL_LABEL['lqr']),
         Line2D([], [], marker='v', c='0.4', mec='k', mew=0.3, ls='', label='agent killed'),
         Line2D([], [], marker='X', c='red', mec='k', mew=0.4, ls='', label='mission aborted'),
         Line2D([], [], c='0.5', lw=0.5, ls=':', label='97% target')]
    fig.legend(handles=h, loc='lower center', ncol=5, frameon=False, bbox_to_anchor=(0.5, -0.30))
    fig.subplots_adjust(wspace=0.08)
    fig.savefig(FIG / 'fig_coverage.pdf')
    plt.close(fig)


def fig_tracking(runs, design):
    fig, axes = plt.subplots(1, 3, figsize=(6.7, 1.75), gridspec_kw={'width_ratios': [1.25, 1.0, 1.0]})
    # (a) error lateral saat sapuan timur-barat
    ax = axes[0]
    data, pos, cols = [], [], []
    for ri, (region, _) in enumerate(REGIONS):
        for j, ctrl in enumerate(('hinf', 'lqr')):
            data.append(runs[(region, ctrl)]['metrics']['lat_samples_cm'])
            pos.append(ri * 3 + j)
            cols.append(C_HINF if ctrl == 'hinf' else C_LQR)
    bp = ax.boxplot(data, positions=pos, widths=0.75, whis=(5, 95), showfliers=False, patch_artist=True,
                    medianprops={'color': 'k', 'lw': 0.8}, boxprops={'lw': 0.5},
                    whiskerprops={'lw': 0.5}, capprops={'lw': 0.5})
    for patch, c in zip(bp['boxes'], cols):
        patch.set_facecolor(c)
        patch.set_alpha(0.75)
    ax.set_xticks([ri * 3 + 0.5 for ri in range(4)])
    ax.set_xticklabels([REGION_LABEL[r] for r, _ in REGIONS])
    ax.set_ylabel('|lateral error| (cm)')
    ax.set_title('(a) sweep lateral error')
    ax.tick_params(length=2)
    ax.set_ylim(0, 31)
    ax.legend(handles=[plt.Rectangle((0, 0), 1, 1, fc=C_HINF, alpha=0.75, label=CTRL_LABEL['hinf']),
                       plt.Rectangle((0, 0), 1, 1, fc=C_LQR, alpha=0.75, label=CTRL_LABEL['lqr'])],
              loc='upper center', ncol=2, frameon=False, handlelength=1.0, columnspacing=0.8)
    # (b) trade-off effort vs error lateral
    ax = axes[1]
    mk = {'rect': 's', 'l_shape': '^', 'u_shape': 'D', 'plus': 'P'}
    for region, _ in REGIONS:
        for ctrl in ('hinf', 'lqr'):
            M = runs[(region, ctrl)]['metrics']
            ax.plot(M['effort_rate'], M['lat_rms_cm'], marker=mk[region], ls='',
                    mfc=C_HINF if ctrl == 'hinf' else C_LQR, mec='k', mew=0.4, ms=5)
    ax.set_xlabel(r'torque effort $\overline{\tau_x^2+\tau_y^2}$ (N$^2$m$^2$)')
    ax.set_ylabel('lateral RMS (cm)')
    ax.set_title('(b) accuracy vs. effort')
    ax.legend(handles=[Line2D([], [], marker=mk[r], ls='', mfc='w', mec='k', label=REGION_LABEL[r]) for r, _ in REGIONS],
              frameon=False, loc='upper right', handletextpad=0.2, borderpad=0.2)
    ax.tick_params(length=2)
    # (c) |G_{a->x}(jw)|
    ax = axes[2]
    for name, col, ls in (('hinf', C_HINF, '-'), ('lqr', C_LQR, '--')):
        D = design[name]
        ax.loglog(D['freq'], D['mag'], c=col, ls=ls, lw=1.0, label=CTRL_LABEL[name])
    ax.axvline(1.0 / 0.5, c='0.5', lw=0.5, ls=':')
    ax.set_xlabel(r'$\omega$ (rad/s)')
    ax.set_ylabel(r'$|G_{a_w\to x}(j\omega)|$ (s$^2$)')
    ax.set_title('(c) linear disturbance gain')
    ax.set_xlim(1e-2, 1e2)
    ax.legend(frameon=False, loc='lower left')
    ax.tick_params(length=2)
    fig.subplots_adjust(wspace=0.42)
    fig.savefig(FIG / 'fig_tracking.pdf')
    plt.close(fig)


def fig_events(runs):
    """Kronologi upset: rect-LQR iris_2 (jatuh) vs plus H-inf/LQR iris_2 (pulih)."""
    panels = []
    for region, ctrl in (('rect', 'lqr'), ('plus', 'hinf'), ('plus', 'lqr')):
        ev = sorted(runs[(region, ctrl)]['metrics']['events'], key=lambda e: -e['tilt'])
        if ev:
            panels.append((region, ctrl, ev[0]))
    if not panels:
        return
    fig, axes = plt.subplots(3, 2, figsize=(6.7, 2.05), sharex='col')
    groups = [[p for p in panels if p[0] == 'rect'], [p for p in panels if p[0] == 'plus']]
    handles_ev = []
    for gi, grp in enumerate(groups):
        for region, ctrl, ev in grp:
            d = runs[(region, ctrl)]['drones'][ev['drone']]
            m = d['alive'] & (d['Time_s'] >= ev['t'] - 8) & (d['Time_s'] <= ev['t'] + 6)
            t = d['Time_s'][m] - ev['t']
            col = C_HINF if ctrl == 'hinf' else C_LQR
            ls = '-' if ctrl == 'hinf' else '--'
            lab = f"{REGION_LABEL[region]}, {CTRL_LABEL[ctrl]}, iris_{ev['drone']}"
            axes[0, gi].plot(t, nearest_static_surface(region, d['X'][m], d['Y'][m]), c=col, ls=ls, label=lab)
            handles_ev.append(Line2D([], [], c=col, ls=ls, label=lab))
            axes[1, gi].plot(t, np.maximum(np.abs(d['Roll_deg'][m]), np.abs(d['Pitch_deg'][m])), c=col, ls=ls)
            axes[2, gi].plot(t, d['Z'][m], c=col, ls=ls)
        axes[0, gi].axhspan(0, BODY_HALF, color='red', alpha=0.18, lw=0)
        axes[0, gi].axhspan(BODY_HALF, ROTOR_REACH, color='red', alpha=0.08, lw=0)
        axes[0, gi].set_ylim(0, 2.6)
        axes[1, gi].axhline(25.0, c='0.5', lw=0.5, ls=':')
        axes[2, gi].set_xlabel('time relative to peak tilt (s)')
        for a in axes[:, gi]:
            a.tick_params(length=2)
    axes[0, 0].set_ylabel('surface\ndist. (m)')
    axes[1, 0].set_ylabel('tilt (deg)')
    axes[2, 0].set_ylabel('$z$ (m)')
    axes[0, 0].set_title('(a) contact leading to loss')
    axes[0, 1].set_title('(b) near-contact with recovery')
    handles_ev += [plt.Rectangle((0, 0), 1, 1, fc='red', alpha=0.18, label='body contact'),
                   plt.Rectangle((0, 0), 1, 1, fc='red', alpha=0.08, label='rotor contact possible')]
    fig.legend(handles=handles_ev, loc='lower center', ncol=5, frameon=False, bbox_to_anchor=(0.5, -0.15),
               fontsize=6.5, handlelength=1.8, columnspacing=1.0)
    fig.subplots_adjust(hspace=0.10, wspace=0.18)
    fig.savefig(FIG / 'fig_events.pdf')
    plt.close(fig)


# ═════════════════════════════════════════════════════════════════════════
# 6. EKSPOR
# ═════════════════════════════════════════════════════════════════════════
def fmt(v, nd=1):
    if v is None or (isinstance(v, float) and not np.isfinite(v)):
        return 'n/a'
    return f'{v:.{nd}f}'


def export(runs, design):
    GEN.mkdir(parents=True, exist_ok=True)
    metrics = {f'{r}_{c}': runs[(r, c)]['metrics'] for r, _ in REGIONS for c, _ in CTRLS}
    slim = {k: {kk: vv for kk, vv in v.items() if kk != 'lat_samples_cm'} for k, v in metrics.items()}
    design_slim = {k: {kk: vv for kk, vv in v.items() if kk not in ('freq', 'mag')} for k, v in design.items()}
    (GEN / 'metrics.json').write_text(json.dumps({'runs': slim, 'design': design_slim}, indent=2))

    # numbers.tex — macro \<Metrik><Region><Ctrl>
    L = ['% DIHASILKAN OTOMATIS oleh swarm_ws/analysis/skema5_paper_analysis.py — JANGAN DIEDIT.']
    def mac(name, val):
        L.append(f'\\newcommand{{\\{name}}}{{{val}}}')
    for region, _ in REGIONS:
        for ctrl, _ in CTRLS:
            M = metrics[f'{region}_{ctrl}']
            s = MACRO_REGION[region] + MACRO_CTRL[ctrl]
            mac('Cov' + s, fmt(M['cov_final']))
            mac('Tmis' + s, fmt(M['t_total'], 0) if M['t_total'] else fmt(M['t_end_csv'], 1))
            mac('Tninetyseven' + s, fmt(M['t97'], 0))
            mac('Latrms' + s, fmt(M['lat_rms_cm']))
            mac('Latpfive' + s, fmt(M['lat_p95_cm']))
            mac('Latmed' + s, fmt(M['lat_p50_cm']))
            mac('Vsweep' + s, fmt(M['v_sweep'], 2))
            mac('Altrms' + s, fmt(M['alt_rms_cm']))
            mac('Tiltpfive' + s, fmt(M['tilt_sweep_p95']))
            mac('Tiltmax' + s, fmt(M['tilt_max']))
            mac('Effort' + s, fmt(M['effort_rate'], 3))
            mac('Sat' + s, fmt(M['sat_pct'], 2))
            mac('Obssurf' + s, fmt(M['obs_surface_min'], 2))
            mac('Vtwov' + s, fmt(M['v2v_min_csv'], 2))
            mac('Dminlog' + s, fmt(M['dmin_log'], 2))
            mac('Ptier' + s, fmt(M['cbf']['p_tier'], 2) if M['cbf'] else 'n/a')
            mac('Tiertwo' + s, str(M['cbf']['T2']) if M['cbf'] else 'n/a')
            mac('Covoffline' + s, fmt(M['cov_offline_upper']))
    for name in ('hinf', 'lqr'):
        D = design[name]
        s = MACRO_CTRL[name]
        mac('Hnorm' + s, fmt(D['hinf_norm_w_to_x'], 3))
        mac('Hpeak' + s, fmt(D['hinf_peak_rad_s'], 2))
        mac('Gcorner' + s, fmt(D['gain_at_wind_corner'], 3))
        mac('Sigx' + s, fmt(D['sigma_x_per_unit_accel_gm'], 3))
        mac('Wc' + s, fmt(D['crossover_rad_s'], 2))
        mac('Pm' + s, fmt(D['phase_margin_deg'], 1))
        mac('Gm' + s, fmt(D['gain_margin_db'], 1))
        for k, v in D['gains_outer'].items():
            mac(f'Kout{k.lower()}' + s, fmt(v, 3))
        for k, v in D['gains_inner'].items():
            mac(f'Kin{k.lower()}' + s, fmt(v, 3))
    # ── turunan lintas-run (untuk kalimat abstrak/diskusi) ──
    pairs = [r for r, _ in REGIONS]
    done = [r for r in pairs if not metrics[f'{r}_lqr']['aborted'] and not metrics[f'{r}_hinf']['aborted']]
    tred = [100 * (1 - metrics[f'{r}_hinf']['t_total'] / metrics[f'{r}_lqr']['t_total']) for r in done]
    effr = [metrics[f'{r}_hinf']['effort_rate'] / metrics[f'{r}_lqr']['effort_rate'] for r in done]
    latred = [100 * (1 - metrics[f'{r}_hinf']['lat_rms_cm'] / metrics[f'{r}_lqr']['lat_rms_cm']) for r in pairs]
    covs = [metrics[f'{r}_{c}']['cov_final'] for r in pairs for c in ('hinf', 'lqr')
            if metrics[f'{r}_{c}']['cov_final'] is not None]
    covh = [metrics[f'{r}_hinf']['cov_final'] for r in pairs]
    medh = [metrics[f'{r}_hinf']['lat_p50_cm'] for r in pairs]
    medl = [metrics[f'{r}_lqr']['lat_p50_cm'] for r in pairs]
    rmsh = [metrics[f'{r}_hinf']['lat_rms_cm'] for r in pairs]
    rmsl = [metrics[f'{r}_lqr']['lat_rms_cm'] for r in pairs]
    vh = [metrics[f'{r}_hinf']['v_sweep'] for r in pairs]
    for key, nd in (('lat_mean_cm', 1), ('lat_std_cm', 1), ('integ_window_pct', 0)):
        for c in ('hinf', 'lqr'):
            vals = [metrics[f'{r}_{c}'][key] for r in pairs]
            nm = {'lat_mean_cm': 'LatMean', 'lat_std_cm': 'LatStd', 'integ_window_pct': 'IntegWin'}[key]
            mac(nm + MACRO_CTRL[c] + 'Min', fmt(min(vals), nd)); mac(nm + MACRO_CTRL[c] + 'Max', fmt(max(vals), nd))
    vl = [metrics[f'{r}_lqr']['v_sweep'] for r in pairs]
    mac('NDone', str(len(covs)))
    mac('CovMin', fmt(min(covs))); mac('CovMax', fmt(max(covs)))
    mac('CovHinfMin', fmt(min(covh))); mac('CovHinfMax', fmt(max(covh)))
    mac('TimeRedMin', fmt(min(tred), 0)); mac('TimeRedMax', fmt(max(tred), 0))
    mac('EffRatioMin', fmt(min(effr), 1)); mac('EffRatioMax', fmt(max(effr), 1))
    mac('LatRedMin', fmt(min(latred), 0)); mac('LatRedMax', fmt(max(latred), 0))
    mac('LatMedHinfMin', fmt(min(medh))); mac('LatMedHinfMax', fmt(max(medh)))
    mac('LatMedLqrMin', fmt(min(medl))); mac('LatMedLqrMax', fmt(max(medl)))
    mac('LatRmsHinfMin', fmt(min(rmsh))); mac('LatRmsHinfMax', fmt(max(rmsh)))
    mac('LatRmsLqrMin', fmt(min(rmsl))); mac('LatRmsLqrMax', fmt(max(rmsl)))
    mac('VHinfMin', fmt(min(vh), 2)); mac('VHinfMax', fmt(max(vh), 2))
    mac('VLqrMin', fmt(min(vl), 2)); mac('VLqrMax', fmt(max(vl), 2))
    mac('HnormRatio', fmt(design['lqr']['hinf_norm_w_to_x'] / design['hinf']['hinf_norm_w_to_x'], 1))
    mac('SigxRatio', fmt(design['lqr']['sigma_x_per_unit_accel_gm'] / design['hinf']['sigma_x_per_unit_accel_gm'], 1))
    ev_plus = sorted(metrics['plus_hinf']['events'], key=lambda e: -e['tilt'])
    ev_plusl = sorted(metrics['plus_lqr']['events'], key=lambda e: -e['tilt'])
    ev_rect = sorted(metrics['rect_lqr']['events'], key=lambda e: -e['tilt'])
    if ev_plus:
        mac('UpPlusHinfTilt', fmt(ev_plus[0]['tilt'])); mac('UpPlusHinfZmin', fmt(ev_plus[0]['z_min_window'], 2))
        mac('UpPlusHinfSurf', fmt(ev_plus[0]['surface_min_window'], 2))
    if ev_plusl:
        mac('UpPlusLqrTilt', fmt(ev_plusl[0]['tilt'])); mac('UpPlusLqrZmin', fmt(ev_plusl[0]['z_min_window'], 2))
        mac('UpPlusLqrSurf', fmt(ev_plusl[0]['surface_min_window'], 2))
    if ev_rect:
        mac('UpRectLqrTilt', fmt(ev_rect[0]['tilt'])); mac('UpRectLqrZmin', fmt(ev_rect[0]['z_min_window'], 2))
        mac('UpRectLqrSurf', fmt(ev_rect[0]['surface_min_window'], 2)); mac('UpRectLqrT', fmt(ev_rect[0]['t'], 1))
        if len(ev_rect) > 1:
            mac('UpRectLqrSecondTilt', fmt(ev_rect[1]['tilt'])); mac('UpRectLqrSecondT', fmt(ev_rect[1]['t'], 1))
            mac('UpRectLqrSecondSurf', fmt(ev_rect[1]['surface_min_window'], 2))
    spans = [e['tier2_static_span_s'] for e in (ev_rect[:1] + ev_plus[:1] + ev_plusl[:1])]
    mac('TierSpanMin', fmt(min(spans), 0)); mac('TierSpanMax', fmt(max(spans), 0))
    offl = [100 - metrics[f'{r}_lqr']['integ_window_pct'] for r in pairs]
    mac('IntegOffLqrMin', fmt(min(offl), 0)); mac('IntegOffLqrMax', fmt(max(offl), 0))
    mac('KiRatio', fmt(design['hinf']['gains_outer']['Ki'] / design['lqr']['gains_outer']['Ki'], 1))
    # barrier dinamis: h = d - (0.50 model + 0.22 + 0.75); jarak pusat-ke-permukaan truth = h + 1.47 - 0.45
    hd = [metrics[f'{r}_{c}']['h_dyn_min_logged'] for r in pairs for c in ('hinf', 'lqr')
          if metrics[f'{r}_{c}']['h_dyn_min_logged'] is not None]
    mac('HdynMinLogged', fmt(min(hd), 2))   # min atas semua baris; lihat catatan di run_metrics
    mac('NRunsHdynNeg', str(sum(1 for v in hd if v < 0)))
    mac('BodyHalf', fmt(BODY_HALF, 3)); mac('RotorReach', fmt(ROTOR_REACH, 3))
    kills = metrics['rect_hinf']['kills_log']
    for r in pairs:
        for c in ('hinf', 'lqr'):
            ks = metrics[f'{r}_{c}']['kills_log']
            s = MACRO_REGION[r] + MACRO_CTRL[c]
            mac('Victims' + s, ', '.join(str(k['drone']) for k in ks))
            mac('Helpers' + s, '; '.join(','.join(str(h) for h in k.get('helpers', [])) for k in ks))
    del kills
    w = metrics['rect_hinf']['wind']
    mac('WindMeanX', fmt(w['mean'][0], 2)); mac('WindMeanY', fmt(w['mean'][1], 2))
    mac('WindStdX', fmt(w['std'][0], 2)); mac('WindStdY', fmt(w['std'][1], 2))
    mac('WindStdZ', fmt(w['std'][2], 2)); mac('WindMaxH', fmt(w['max_h'], 1))
    mac('WindMeanMag', fmt(math.hypot(w['mean'][0], w['mean'][1]), 1))
    pl = metrics['rect_hinf']['plant']
    for k, v in pl.items():
        mac('Plant' + k.replace('_', '').capitalize(), fmt(v, 3))
    (GEN / 'numbers.tex').write_text('\n'.join(L) + '\n')

    # table_results.tex — isi tabel utama
    T = ['% DIHASILKAN OTOMATIS oleh skema5_paper_analysis.py']
    for region, _ in REGIONS:
        for ci, (ctrl, _) in enumerate(CTRLS):
            M = metrics[f'{region}_{ctrl}']
            reg = f'\\multirow{{2}}{{*}}{{{REGION_LABEL[region]}}}' if ci == 0 else ''
            if M['aborted']:
                status = f"abort@{fmt(M['t_end_csv'], 1)}"
                cov = 'n/a'
                tm = 'n/a'
            else:
                status = 'done'
                cov = fmt(M['cov_final'])
                tm = fmt(M['t_total'], 0)
            dv2v = fmt(M['v2v_min_csv'], 2)
            T.append(' & '.join([reg, CTRL_LABEL[ctrl], status, cov, tm,
                                 fmt(M['lat_rms_cm']), fmt(M['v_sweep'], 2), fmt(M['alt_rms_cm']),
                                 fmt(M['tilt_sweep_p95']), fmt(M['tilt_max']),
                                 fmt(M['effort_rate'], 3), fmt(M['obs_surface_min'], 2), dv2v,
                                 fmt(M['cbf']['p_tier'], 1) if M['cbf'] else 'n/a']) + r' \\')
        T.append(r'\midrule' if region != 'plus' else '')
    (GEN / 'table_results.tex').write_text('\n'.join(T) + '\n')


def audit(runs, design):
    A = ['# Audit data Skema 5 (dihasilkan otomatis)', '']
    ok = True
    for region, _ in REGIONS:
        for ctrl, _ in CTRLS:
            M = runs[(region, ctrl)]['metrics']
            log = runs[(region, ctrl)]['log']
            A.append(f'## {region} / {ctrl}')
            n = M['n_status']
            tt = M['t_total']
            A.append(f"- baris STATUS = {n}; Total Waktu Misi log = {tt}; akhir CSV = {M['t_end_csv']:.2f} s")
            if tt is not None and abs(n - tt) > 2:
                A.append('  - **TIDAK COCOK**: asumsi 1 STATUS = 1 s'); ok = False
            if tt is not None and abs(M['t_end_csv'] - tt) > 2.0:
                A.append('  - **TIDAK COCOK**: durasi CSV vs log > 2 s'); ok = False
            for k in log['kills']:
                tk = M['kills_csv'].get(str(k['drone']))
                A.append(f"- kill iris_{k['drone']}: log t≈{k['t_log']:.0f} s (cov {k['cov']}%), CSV onset {tk}")
                if tk is None:
                    A.append('  - **TIDAK COCOK**: korban log tanpa RPM=0 di CSV'); ok = False
            extra = set(M['kills_csv']) - {str(k['drone']) for k in log['kills']}
            if extra:
                A.append(f'  - **TIDAK COCOK**: RPM=0 di CSV tanpa kill di log: {sorted(extra)}'); ok = False
            A.append(f"- offset CSV−log (median kill) = {M['t_offset_csv_minus_log']}")
            A.append(f"- coverage final log = {M['cov_final']} | STATUS terakhir = {M['cov_last_status']} "
                     f"| offline batas atas CSV = {M['cov_offline_upper']:.1f}")
            if M['cov_final'] is not None and M['cov_offline_upper'] + 0.5 < M['cov_final']:
                A.append('  - **TIDAK COCOK**: offline (batas atas) < log'); ok = False
            A.append(f"- lateral: mean {M['lat_mean_cm']:.1f} cm, std {M['lat_std_cm']:.1f} cm, jendela integrator LQR {M['integ_window_pct']:.0f}%")
            A.append(f"- d_min V2V: log = {M['dmin_log']} | CSV = {M['v2v_min_csv']:.2f} m "
                     f"(pasangan {M['v2v_pair']}, t={M['v2v_t']})")
            A.append(f"- clearance permukaan statis min = {M['obs_surface_min']:.3f} m (iris_{M['obs_surface_min_drone']}, "
                     f"obs {M['obs_surface_min_id']}, t={M['obs_surface_min_t']:.1f}) → kontak: {M['obs_contact']}")
            A.append(f"- PETA: {len(M['peta_tracks'])} track terkonfirmasi-diam, id truth tercocok {M['peta_matched_ids']}; "
                     f"err posisi maks (cocok) {M['peta_pos_err_max']} m, |err r| maks {M['peta_r_err_absmax']}, track spurious {M['peta_spurious']}")
            A.append(f"- Tier-2 per pembatas: {M['tier2_by_limiter']}; h_min log (semua baris) saat baris bergerak mengikat = {M['h_dyn_min_logged']}")
            for ev in M['events']:
                A.append(f"- upset iris_{ev['drone']} t={ev['t']:.1f} tilt={ev['tilt']:.1f}° pos=({ev['x']:.2f},{ev['y']:.2f}) "
                         f"zmin={ev['z_min_window']:.2f} surf_min={ev['surface_min_window']:.2f} recovered={ev['recovered']}")
            if M['abort']:
                A.append(f"- ABORT: iris_{M['abort']['drone']} Z={M['abort']['z']} pada log t≈{M['abort']['t_log']:.0f} s")
            A.append('')
    A.append('## Analisis desain linear (sumbu x)')
    for name, D in design.items():
        A.append(f"- {name}: stabil={D['stable']} ‖G‖∞={D['hinf_norm_w_to_x']:.4f} @ {D['hinf_peak_rad_s']:.2f} rad/s, "
                 f"PM={D['phase_margin_deg']}, GM={D['gain_margin_db']} dB, wc={D['crossover_rad_s']}, "
                 f"σx/σa={D['sigma_x_per_unit_accel_gm']:.4f}, gains out={D['gains_outer']} in={D['gains_inner']}")
    A.append('')
    A.append(f'**STATUS AUDIT: {"LOLOS" if ok else "ADA KETIDAKCOCOKAN — lihat di atas"}**')
    (GEN / 'audit.md').write_text('\n'.join(A) + '\n')
    return ok


def main():
    style()
    runs = {}
    for region, folder in REGIONS:
        for ctrl, sub in CTRLS:
            drones, log = load_run(region, folder, ctrl, sub)
            runs[(region, ctrl)] = {'drones': drones, 'log': log}
            runs[(region, ctrl)]['metrics'] = run_metrics(region, ctrl, drones, log)
            print(f'[ok] {region}/{ctrl}')
    design = design_analysis()
    FIG.mkdir(parents=True, exist_ok=True)
    fig_traj(runs)
    fig_coverage(runs)
    fig_tracking(runs, design)
    fig_events(runs)
    export(runs, design)
    ok = audit(runs, design)
    print(f'audit: {"LOLOS" if ok else "ADA KETIDAKCOCOKAN"} → {GEN / "audit.md"}')


if __name__ == '__main__':
    main()
