#!/usr/bin/env python3
"""Identifikasi dead time efektif T_d dari telemetri penerbangan nyata.

Model loop tertutup (lihat plant_model.py) menyatakan percepatan lateral

    a(t) = g*Kp * e(t - T_d)  -  g*Kd * v(t - T_d)

dengan e = referensi - posisi. T_d yang benar TIDAK dapat ditebak: ia
mencakup lag sensor, periode loop kendali, prefilter referensi orde-2 di
low-level, dan lag motor. Menebak terlalu kecil membuat phi(h) terlalu
permisif — drone bereaksi terlambat dan menembus rintangan.

Metode: sapu kandidat T_d, untuk tiap kandidat lakukan regresi linear
a(t) terhadap [e(t-T_d), v(t-T_d)], ambil T_d dengan R^2 terbaik.

Pakai:
    python3 -m swarm_mid_level.cbf.identify_plant <dir_csv_atau_file...>
"""
import glob
import os
import sys

import numpy as np


def load_csv(path):
    import csv
    cols = {}
    with open(path) as f:
        for row in csv.DictReader(f):
            for k, v in row.items():
                try:
                    cols.setdefault(k, []).append(float(v))
                except (TypeError, ValueError):
                    cols.setdefault(k, []).append(np.nan)
    return {k: np.asarray(v, dtype=float) for k, v in cols.items()}


def identify(df, t_min=8.0, td_grid=None):
    """Kembalikan (T_d terbaik, R^2, k_p_eff, k_v_eff) untuk satu drone."""
    if td_grid is None:
        td_grid = np.arange(0.0, 0.85, 0.025)

    t = df['Time_s']
    keep = t >= t_min                      # buang fase takeoff
    if keep.sum() < 200:
        return None

    t = t[keep]
    dt = np.diff(t)
    ok = dt > 1e-4
    if ok.sum() < 100:
        return None

    best = None
    for axis, ref in (('X', 'Ref_X'), ('Y', 'Ref_Y')):
        pos = df[axis][keep]
        rf = df[ref][keep]
        vel = df['vx' if axis == 'X' else 'vy'][keep]

        acc = np.diff(vel) / np.where(ok, dt, 1.0)
        acc = np.where(ok, acc, 0.0)
        err = (rf - pos)[:-1]
        v = vel[:-1]
        tt = t[:-1]

        for td in td_grid:
            # Geser regresor mundur sebesar td (interpolasi pada grid waktu).
            e_d = np.interp(tt - td, tt, err)
            v_d = np.interp(tt - td, tt, v)
            A = np.stack([e_d, v_d], axis=1)
            m = np.isfinite(A).all(axis=1) & np.isfinite(acc)
            if m.sum() < 100:
                continue
            coef, *_ = np.linalg.lstsq(A[m], acc[m], rcond=None)
            pred = A[m] @ coef
            resid = acc[m] - pred
            ss_res = float(resid @ resid)
            ss_tot = float(((acc[m] - acc[m].mean()) ** 2).sum())
            r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else -np.inf
            if best is None or r2 > best[1]:
                best = (float(td), r2, float(coef[0]), float(-coef[1]))
    return best


def main(argv):
    paths = []
    for a in argv:
        paths.extend(sorted(glob.glob(os.path.join(a, '*.csv')))
                     if os.path.isdir(a) else [a])
    if not paths:
        print('Tidak ada CSV. Berikan direktori atau berkas.')
        return 1

    rows = []
    for p in paths:
        try:
            r = identify(load_csv(p))
        except Exception as exc:                     # noqa: BLE001
            print(f'  ! {os.path.basename(p)}: {exc}')
            continue
        if r is None:
            continue
        rows.append(r)
        print(f'  {os.path.basename(p):38s} T_d={r[0]:.3f}s  R2={r[1]:.3f}  '
              f'k_p={r[2]:7.3f}  k_v={r[3]:6.3f}')

    if not rows:
        print('Tidak ada data yang dapat diidentifikasi.')
        return 1

    tds = np.array([r[0] for r in rows])
    kvs = np.array([r[3] for r in rows])
    print()
    print(f'  T_d : median {np.median(tds):.3f}s  '
          f'rentang [{tds.min():.3f}, {tds.max():.3f}]  (n={len(tds)})')
    print(f'  k_v : median {np.median(kvs):.3f}/s')
    print()
    print(f'  -> Setel cbf.T_d = {np.percentile(tds, 75):.2f} '
          f'(persentil ke-75; barrier harus konservatif, bukan rata-rata)')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
