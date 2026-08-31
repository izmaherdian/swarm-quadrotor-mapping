#!/usr/bin/env python3
"""Gambar untuk laporan Skema 1 & 2 x {LQR, Hinf} pada wilayah non-convex.

SELURUH data berasal dari artefak run yang nyata (CSV low-level + stdout
coordinator). Tidak ada nilai sintetis: bila sebuah run tidak punya berkas,
panelnya dikosongkan dengan tulisan 'data tidak ada', bukan diisi karangan.

Pakai:
    python3 -m swarm_high_level.metrics.region_figures <root> --out DIR
"""
import glob
import json
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D
from matplotlib.patches import Circle

from .region_report import load_csvs
from ..world.obstacles import OBSTACLES_BY_REGION, OBSTACLE_RADIUS
from ..world.region import load_region

# Palet kategorikal tervalidasi (slot 1 & 2), mode terang.
C_LQR, C_HINF = '#2a78d6', '#eb6834'
INK, INK2, GRID = '#0b0b0b', '#52514e', '#dcdcd8'
SURFACE = '#fcfcfb'
CTRL_COLOR = {'lqr': C_LQR, 'hinf': C_HINF}
CTRL_NAME = {'lqr': 'PID-LQR', 'hinf': 'PID-H∞'}


def _style(ax):
    ax.set_facecolor(SURFACE)
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)
    for s in ('left', 'bottom'):
        ax.spines[s].set_color(GRID)
    ax.tick_params(colors=INK2, labelsize=8, length=3)
    ax.grid(True, color=GRID, lw=0.6, alpha=0.9)
    ax.set_axisbelow(True)


SCHEME_NAME = {'s1': 'Skema 1 (nominal)', 's2': 'Skema 2 (angin Dryden)',
               's3': 'Skema 3 (rintangan statis)'}

# Skema mana yang benar-benar men-spawn silinder. Menggambar rintangan pada
# Skema 1/2 akan menampilkan halangan yang TIDAK ADA di Gazebo.
OBSTACLE_SCHEMES = ('s3', 's4')

# Diisi ulang dari data yang benar-benar ada di `main`.
SCHEMES = ('s1', 's2')


def fig_trajectories(runs, regions, out, sch, summ=None):
    """Lintasan terbang NYATA per wilayah, dua kontroler bersebelahan."""
    n = len(regions)
    fig, axes = plt.subplots(2, n, figsize=(4.1 * n, 8.6), facecolor=SURFACE)
    axes = np.atleast_2d(axes)
    for col, reg in enumerate(regions):
        ring, _poly = load_region(reg)
        for row, ctrl in enumerate(('lqr', 'hinf')):
            ax = axes[row][col]
            _style(ax)
            rr = np.vstack([ring, ring[0]])
            ax.plot(rr[:, 0], rr[:, 1], color=INK, lw=1.6, zorder=3)
            if sch in OBSTACLE_SCHEMES:
                for _oid, ox, oy in OBSTACLES_BY_REGION.get(reg, ()):
                    ax.add_patch(Circle((ox, oy), OBSTACLE_RADIUS, facecolor=INK,
                                        edgecolor='none', zorder=4))
                    ax.add_patch(Circle((ox, oy), 1.30, facecolor='none',
                                        edgecolor=INK2, lw=0.7, ls=':', zorder=4))
            d = runs.get((sch, reg, ctrl))
            if not d:
                ax.text(0.5, 0.5, 'data tidak ada', ha='center', va='center',
                        transform=ax.transAxes, color=INK2, fontsize=9)
            else:
                for df in d.values():
                    m = df['Time_s'] >= 20.0
                    ax.plot(df['X'][m], df['Y'][m], color=CTRL_COLOR[ctrl],
                            lw=0.8, alpha=0.75, zorder=2)
                cov = (summ or {}).get(f'{sch}|{reg}|{ctrl}', {}).get('coverage_final_pct')
                if cov is not None:
                    ax.text(0.03, 0.03, f'coverage {cov:.1f}%  ·  {len(d)} drone',
                            transform=ax.transAxes, fontsize=8, color=INK,
                            ha='left', va='bottom', zorder=5,
                            bbox=dict(facecolor=SURFACE, edgecolor=GRID,
                                      boxstyle='round,pad=0.3', linewidth=0.6))
            ax.set_aspect('equal')
            ax.set_xlim(-15.5, 15.5); ax.set_ylim(-15.5, 15.5)
            if row == 0:
                ax.set_title(reg, color=INK, fontsize=11, pad=8)
            if col == 0:
                ax.set_ylabel(f'{CTRL_NAME[ctrl]}\nY (m)', color=INK, fontsize=9)
            ax.set_xlabel('X (m)', color=INK2, fontsize=8)
    fig.suptitle(f'Lintasan terbang aktual — {SCHEME_NAME.get(sch, sch)}',
                 color=INK, fontsize=13, y=0.98)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    p = os.path.join(out, f'fig1_trajectories_{sch}.png')
    fig.savefig(p, dpi=150, facecolor=SURFACE); plt.close(fig)
    return p


def fig_metric_bars(summ, regions, out):
    """Batang berdampingan LQR vs Hinf untuk metrik kunci, per skema."""
    metrics = [('ct_rms_cm', 'Cross-track RMS (cm)'),
               ('alt_rms_cm', 'Altitude RMS (cm)'),
               ('roll_sweep_p95_deg', 'Roll p95 saat sapu (°)'),
               ('effort_total', 'Integral torsi')]
    fig, axes = plt.subplots(len(SCHEMES), len(metrics),
                             figsize=(3.5 * len(metrics), 3.6 * len(SCHEMES)),
                             facecolor=SURFACE)
    axes = np.atleast_2d(axes)
    x = np.arange(len(regions)); w = 0.36
    for r, sch in enumerate(SCHEMES):
        for c, (key, label) in enumerate(metrics):
            ax = axes[r][c]; _style(ax)
            for i, ctrl in enumerate(('lqr', 'hinf')):
                vals = [summ.get(f'{sch}|{reg}|{ctrl}', {}).get(key) for reg in regions]
                vals = [np.nan if v is None else v for v in vals]
                ax.bar(x + (i - 0.5) * w, vals, w * 0.82,
                       color=CTRL_COLOR[ctrl], zorder=2,
                       label=CTRL_NAME[ctrl] if (r == 0 and c == 0) else None)
                for xi, v in zip(x + (i - 0.5) * w, vals):
                    if np.isfinite(v):
                        ax.text(xi, v, f'{v:.2f}', ha='center', va='bottom',
                                fontsize=6.5, color=INK2)
            ax.set_xticks(x); ax.set_xticklabels(regions, fontsize=8)
            ax.set_title(label, color=INK, fontsize=9.5, pad=6)
            if c == 0:
                ax.set_ylabel(f'Skema {sch[1:]}', color=INK, fontsize=10)
    handles = [Line2D([0], [0], color=CTRL_COLOR[c], lw=7, label=CTRL_NAME[c])
               for c in ('lqr', 'hinf')]
    fig.suptitle('PID-LQR vs PID-H∞ pada wilayah non-convex',
                 color=INK, fontsize=13, y=0.985)
    fig.legend(handles=handles, loc='upper center', ncol=2, frameon=False,
               fontsize=10, bbox_to_anchor=(0.5, 0.948))
    fig.tight_layout(rect=[0, 0, 1, 0.915])
    p = os.path.join(out, 'fig2_metrics.png')
    fig.savefig(p, dpi=150, facecolor=SURFACE); plt.close(fig)
    return p


def fig_coverage(logs, regions, out):
    """Kurva coverage vs waktu, dibaca dari baris STATUS coordinator."""
    fig, axes = plt.subplots(1, len(regions),
                             figsize=(4.2 * len(regions), 3.6), facecolor=SURFACE)
    axes = np.atleast_1d(axes)
    for i, reg in enumerate(regions):
        ax = axes[i]; _style(ax)
        for sch, ls in zip(SCHEMES, ('-', '--', ':')):
            for ctrl in ('lqr', 'hinf'):
                s = logs.get(f'{sch}|{reg}|{ctrl}')
                if not s:
                    continue
                t, cov = s
                ax.plot(t, cov, ls, color=CTRL_COLOR[ctrl], lw=1.8, alpha=0.9)
        ax.set_title(reg, color=INK, fontsize=10.5)
        ax.set_xlabel('waktu sim (s)', color=INK2, fontsize=8)
        if i == 0:
            ax.set_ylabel('coverage (%)', color=INK, fontsize=9)
        ax.set_ylim(0, 105)
    handles = [Line2D([0], [0], color=CTRL_COLOR[c], lw=2.4, label=CTRL_NAME[c])
               for c in ('lqr', 'hinf')]
    handles += [Line2D([0], [0], color=INK2, lw=1.8, ls=s, label=f'Skema {sc[1:]}')
                for sc, s in zip(SCHEMES, ('-', '--', ':'))]
    fig.legend(handles=handles, loc='upper center', ncol=4, frameon=False, fontsize=9)
    fig.tight_layout(rect=[0, 0, 1, 0.87])
    p = os.path.join(out, 'fig3_coverage.png')
    fig.savefig(p, dpi=150, facecolor=SURFACE); plt.close(fig)
    return p


def read_cov(log_path):
    import re
    rx = re.compile(r'\[(\d+)\.\d+\].*Cov:\s*([0-9.]+)%')
    t, c = [], []
    try:
        with open(log_path, errors='replace') as f:
            for line in f:
                m = rx.search(line)
                if m:
                    t.append(int(m.group(1))); c.append(float(m.group(2)))
    except OSError:
        return None
    if not t:
        return None
    t = np.array(t, float); return t - t[0], np.array(c)


def main(argv):
    root = argv[0]
    out = argv[argv.index('--out') + 1] if '--out' in argv else root
    os.makedirs(out, exist_ok=True)

    sfile = os.path.join(out, 'region_summary.json')
    summ = json.load(open(sfile)) if os.path.isfile(sfile) else {}

    runs, logs, regions, schemes = {}, {}, [], []
    for d in sorted(glob.glob(os.path.join(root, 's*_*_*'))):
        parts = os.path.basename(d).split('_')
        sch, ctrl, reg = parts[0], parts[-1], '_'.join(parts[1:-1])
        if reg not in regions:
            regions.append(reg)
        if sch not in schemes:
            schemes.append(sch)
        runs[(sch, reg, ctrl)] = load_csvs(d, ctrl)
        cv = read_cov(os.path.join(d, 'coordinator.log'))
        if cv:
            logs[f'{sch}|{reg}|{ctrl}'] = cv
    regions.sort(); schemes.sort()
    global SCHEMES
    SCHEMES = tuple(schemes)
    if not regions:
        print(f'Tidak ada run di {root}'); return 1

    made = [fig_trajectories(runs, regions, out, sch, summ) for sch in SCHEMES]
    if summ:
        made.append(fig_metric_bars(summ, regions, out))
    if logs:
        made.append(fig_coverage(logs, regions, out))
    for p in made:
        print('gambar:', p)
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
