#!/usr/bin/env python3
"""Laporan Skema 1/2/3 x {PID-LQR, PID-Hinf} pada beberapa wilayah non-convex.

Membaca HANYA artefak nyata:
  * stdout coordinator  -> coverage, d_min, tabrakan, overshoot, diagnostik CBF
  * CSV low-level 28 kolom -> tracking, sikap, usaha kendali, lintasan

Tidak ada nilai yang disintesis. Kombinasi yang berkasnya tidak ada dilaporkan
'n/a' dan dihitung sebagai run gagal, bukan diisi tebakan.

Pakai:
    python3 -m swarm_high_level.metrics.region_report <root> [--out DIR]

<root> berisi subfolder bernama  s<skema>_<region>_<ctrl>/  masing-masing dengan
coordinator.log dan pid_<ctrl>/*.csv
"""
import csv
import glob
import json
import os
import sys

import numpy as np

from ..world.obstacles import OBSTACLE_RADIUS, OBSTACLES_BY_REGION
from .telemetry import parse_coordinator_log

CTRL = {'lqr': 'PID-LQR', 'hinf': 'PID-H∞'}
TAKEOFF_SETTLE_S = 20.0
DRONE_RADIUS = 0.22


def load_csvs(run_dir, prefix):
    out = {}
    for path in sorted(glob.glob(os.path.join(
            run_dir, f'pid_{prefix}', f'flight_data_log_{prefix}_iris_*.csv'))):
        try:
            did = int(os.path.basename(path).split('_')[-1].replace('.csv', ''))
        except ValueError:
            continue
        cols = {}
        with open(path) as f:
            for row in csv.DictReader(f):
                for k, v in row.items():
                    try:
                        cols.setdefault(k, []).append(float(v))
                    except (TypeError, ValueError):
                        cols.setdefault(k, []).append(np.nan)
        if cols and len(next(iter(cols.values()))) > 100:
            out[did] = {k: np.asarray(v, float) for k, v in cols.items()}
    return out


def sweeping_mask(df):
    """Sampel saat MENYAPU mantap: Ref_Yaw stabil >=1.5 s, drone bergerak.

    Memisahkan fase sapuan dari pivot 180 deg di ujung baris; tanpa ini metrik
    yaw didominasi transien belokan dan tidak mencerminkan kualitas sapuan.
    """
    t, ry = df['Time_s'], df['Ref_Yaw']
    spd = np.hypot(df['vx'], df['vy'])
    w = 30
    stable = np.zeros(len(t), bool)
    for i in range(w, len(t)):
        seg = ry[i - w:i + 1]
        if np.max(np.abs((seg - seg[0] + 180.0) % 360.0 - 180.0)) < 1.0:
            stable[i] = True
    return stable & (spd > 0.6) & (t >= TAKEOFF_SETTLE_S)


def obstacle_clearance_m(data, region):
    """Clearance FISIK terkecil ke silinder statis, dari lintasan CSV.

    Hanya bermakna untuk Skema 3/4. Untuk Skema 1/2 pemanggil harus melewatkan
    ``region=None``: di sana tidak ada silinder yang di-spawn, jadi jarak ke
    tabel rintangan adalah angka phantom, bukan pengukuran.

    Rintangan DINAMIS sengaja tidak dihitung — posisinya bergantung waktu
    absolut simulasi yang tidak sinkron dengan `Time_s` pada CSV; angka untuk
    silinder bergerak harus diambil dari coordinator.
    """
    if not region:
        return None
    table = OBSTACLES_BY_REGION.get(region)
    if not table:
        return None
    best = float('inf')
    for df in data.values():
        m = df['Z'] > 0.5
        if not m.sum():
            continue
        xs, ys = df['X'][m], df['Y'][m]
        for _oid, ox, oy in table:
            d = np.hypot(xs - ox, ys - oy) - (OBSTACLE_RADIUS + DRONE_RADIUS)
            best = min(best, float(d.min()))
    return None if not np.isfinite(best) else best


def per_run(run_dir, prefix, region=None):
    log = os.path.join(run_dir, 'coordinator.log')
    out = {'dir': run_dir, 'ok': False}
    if os.path.isfile(log):
        out.update(parse_coordinator_log(log))
    data = load_csvs(run_dir, prefix)
    if not data:
        out['error'] = f'tidak ada CSV pid_{prefix}'
        return out

    ct, alt, yaw, roll, pitch, eff, rpm, plen, dur = ([] for _ in range(9))
    lag = []
    for df in data.values():
        m = df['Time_s'] >= TAKEOFF_SETTLE_S
        if m.sum() < 50:
            continue
        alt.append(np.abs(df['Ref_Z'][m] - df['Z'][m]))
        sw = sweeping_mask(df)
        if sw.sum() > 50:
            yaw.append(np.abs((df['Ref_Yaw'][sw] - df['Yaw_deg'][sw] + 180.0) % 360.0 - 180.0))
            roll.append(np.abs(df['Roll_deg'][sw]))
            pitch.append(np.abs(df['Pitch_deg'][sw]))

            # CROSS-TRACK sejati = komponen (pos - ref) TEGAK LURUS arah baris.
            # |ref - pos| mentah BUKAN cross-track: ref adalah titik pandu di
            # depan drone (pos + T_lead*v), jadi selisihnya didominasi jarak
            # pandu longitudinal ~T_lead*|v| ~= 0.5 m dan akan melaporkan
            # "cross-track 20 cm" padahal drone tepat di garis.
            th = np.radians(df['Ref_Yaw'][sw])
            ex = df['X'][sw] - df['Ref_X'][sw]
            ey = df['Y'][sw] - df['Ref_Y'][sw]
            ct.append(np.abs(-ex * np.sin(th) + ey * np.cos(th)))
            lag.append(np.abs(ex * np.cos(th) + ey * np.sin(th)))
        dt = np.diff(df['Time_s'])
        dt = np.append(dt, dt[-1] if len(dt) else 0.05)
        eff.append(float(np.sum((df['tau_x']**2 + df['tau_y']**2 + df['tau_z']**2) * dt)))
        rpm.append(np.sqrt(np.mean(((df['RPM_0'] + df['RPM_1'] + df['RPM_2'] + df['RPM_3']) / 4.0)**2)))
        plen.append(float(np.sum(np.hypot(np.diff(df['X']), np.diff(df['Y'])))))
        dur.append(float(df['Time_s'][-1] - df['Time_s'][0]))

    if not alt:
        out['error'] = 'CSV terlalu pendek'
        return out
    A = np.concatenate(alt)
    out.update({
        'n_drones': len(data),
        'alt_rms_cm': float(np.sqrt((A**2).mean()) * 100),
        'effort_total': float(np.sum(eff)),
        'rpm_rms': float(np.mean(rpm)),
        'path_len_m': float(np.sum(plen)),
        'duration_s': float(np.max(dur)),
        'ok': True,
    })
    # Hanya diisi bila skema memang men-spawn rintangan (region diberikan).
    d_obs = obstacle_clearance_m(data, region)
    if d_obs is not None:
        out['obs_clearance_m'] = d_obs
        out['n_obstacles'] = len(OBSTACLES_BY_REGION[region])
    if yaw:
        Y = np.concatenate(yaw); R = np.concatenate(roll); P = np.concatenate(pitch)
        C = np.concatenate(ct); L = np.concatenate(lag)
        out.update({'yaw_sweep_rms_deg': float(np.sqrt((Y**2).mean())),
                    'roll_sweep_p95_deg': float(np.percentile(R, 95)),
                    'pitch_sweep_p95_deg': float(np.percentile(P, 95)),
                    'ct_rms_cm': float(np.sqrt((C**2).mean()) * 100),
                    'ct_p95_cm': float(np.percentile(C, 95) * 100),
                    'ct_max_cm': float(C.max() * 100),
                    'lag_rms_cm': float(np.sqrt((L**2).mean()) * 100)})
    return out


def main(argv):
    root = argv[0] if argv else '.'
    out_dir = None
    if '--out' in argv:
        out_dir = argv[argv.index('--out') + 1]

    res = {}
    for d in sorted(glob.glob(os.path.join(root, 's*_*_*'))):
        name = os.path.basename(d)
        parts = name.split('_')
        scheme, ctrl, region = parts[0], parts[-1], '_'.join(parts[1:-1])
        # Clearance rintangan hanya diukur bila skema memang men-spawn
        # silinder; pada Skema 1/2 angkanya akan phantom, jadi tetap n/a.
        obs_region = region if scheme in ('s3', 's4') else None
        res[(scheme, region, ctrl)] = per_run(d, ctrl, region=obs_region)

    if not res:
        print(f'Tidak ada run di {root}')
        return 1

    schemes = sorted({k[0] for k in res})
    regions = sorted({k[1] for k in res})
    rows = [
        ('Coverage akhir (%)', 'coverage_final_pct', '.1f'),
        ('TABRAKAN', 'collisions', 'd'),
        ('Watchdog', 'watchdogs', 'd'),
        ('Overshoot maks (cm)', 'overshoot_max_cm', '.2f'),
        ('Cross-track RMS (cm)', 'ct_rms_cm', '.2f'),
        ('Cross-track p95 (cm)', 'ct_p95_cm', '.2f'),
        ('Cross-track maks (cm)', 'ct_max_cm', '.2f'),
        ('Lag longitudinal (cm)', 'lag_rms_cm', '.1f'),
        ('Altitude RMS (cm)', 'alt_rms_cm', '.2f'),
        ('Yaw RMS saat sapu (°)', 'yaw_sweep_rms_deg', '.2f'),
        ('Roll p95 saat sapu (°)', 'roll_sweep_p95_deg', '.2f'),
        ('Pitch p95 saat sapu (°)', 'pitch_sweep_p95_deg', '.2f'),
        ('d_min antar-drone (m)', 'd_min_inter_drone_m', '.2f'),
        ('Clearance rintangan (m)', 'obs_clearance_m', '.3f'),
        ('Jumlah rintangan', 'n_obstacles', 'd'),
        ('Durasi misi (s-sim)', 'duration_s', '.0f'),
        ('Panjang lintasan (m)', 'path_len_m', '.0f'),
        ('Integral torsi', 'effort_total', '.2f'),
        ('RPM RMS', 'rpm_rms', '.1f'),
        ('P(tier>0) (%)', 'cbf_p_tier_gt0_pct', '.3f'),
    ]

    def fmt(v, spec):
        if v is None:
            return 'n/a'
        try:
            return format(v, spec)
        except (TypeError, ValueError):
            return str(v)

    for sch in schemes:
        for reg in regions:
            hdr = [CTRL[c] for c in ('lqr', 'hinf') if (sch, reg, c) in res]
            if not hdr:
                continue
            title = f'  SKEMA {sch[1:]} — wilayah {reg}'
            print('\n' + '=' * 62)
            print(title)
            print('=' * 62)
            print(f'{"Metrik":<26}' + ''.join(f'{h:>17}' for h in hdr))
            print('-' * 62)
            for label, key, spec in rows:
                cells = []
                for c in ('lqr', 'hinf'):
                    r = res.get((sch, reg, c))
                    if r is None:
                        continue
                    cells.append(f'{fmt(r.get(key), spec):>17}')
                print(f'{label:<26}' + ''.join(cells))
            for c in ('lqr', 'hinf'):
                r = res.get((sch, reg, c))
                if r and r.get('error'):
                    print(f'  ! {CTRL[c]}: {r["error"]}')

    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
        js = {f'{k[0]}|{k[1]}|{k[2]}': v for k, v in res.items()}
        with open(os.path.join(out_dir, 'region_summary.json'), 'w') as f:
            json.dump(js, f, indent=2, default=float)
        print(f'\nJSON: {os.path.join(out_dir, "region_summary.json")}')

    bad = [k for k, v in res.items() if not v.get('ok')]
    coll = sum((v.get('collisions') or 0) for v in res.values())
    print(f'\nRun sukses {len(res)-len(bad)}/{len(res)} | total tabrakan {coll}')
    return 1 if (bad or coll) else 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
