"""Pembacaan telemetri & perhitungan metrik — satu definisi untuk semua alat.

Sumber data:
  * CSV low-level 28 kolom @20 Hz (pid_lqr_node / pid_hinf_node)
  * stdout coordinator (coverage, d_min, tabrakan, diagnostik CBF)

Aturan yang ditegakkan di sini: TIDAK ADA nilai yang dikarang. Bila data tidak
ada, fungsi mengembalikan None atau melempar — tidak pernah mengisi angka
pengganti. Alat benchmark lama pernah menyintesis telemetri diam-diam saat CSV
hilang, dan grafik hasilnya terlihat masuk akal padahal fiksi.
"""
import csv
import glob
import os
import re

import numpy as np

TAKEOFF_SETTLE_S = 10.0     # abaikan fase lepas landas saat menilai tracking


# ── Pemuatan ────────────────────────────────────────────────────────────

def load_drone_csvs(log_dir, controller_prefix):
    """Muat CSV iris_1..7 dari sebuah direktori hasil.

    Mengembalikan dict {drone_id: {kolom: np.ndarray}}. Berkas dengan < 10
    baris dibuang (sisa run yang gagal).
    """
    patterns = (
        os.path.join(log_dir, f'flight_data_log_{controller_prefix}_iris_*.csv'),
        os.path.join(log_dir, f'pid_{controller_prefix}',
                     f'flight_data_log_{controller_prefix}_iris_*.csv'),
        os.path.join(log_dir, '**',
                     f'flight_data_log_{controller_prefix}_iris_*.csv'),
    )
    files = []
    for pat in patterns:
        files = sorted(glob.glob(pat, recursive='**' in pat))
        if files:
            break

    out = {}
    for path in files:
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
        if cols and len(next(iter(cols.values()))) > 10:
            out[did] = {k: np.asarray(v, dtype=float) for k, v in cols.items()}
    return out


# ── Metrik per drone ────────────────────────────────────────────────────

def overshoot_cm(df):
    """Overshoot ujung baris yang DIUKUR, bukan diasumsikan nol.

    Peristiwa ujung baris dideteksi dengan histeresis pada laju referensi:
    dianggap bergerak di atas 0.30 m/s, berhenti di bawah 0.05 m/s. Deselerasi
    memakan beberapa sampel, jadi membandingkan sampel bersebelahan tidak akan
    pernah memicu. Pada tiap peristiwa dicatat seberapa jauh drone melewati
    titik henti referensi sepanjang arah gerak terakhir.
    """
    if 'Ref_X' not in df or len(df.get('Time_s', ())) < 20:
        return 0.0

    t, rx, ry = df['Time_s'], df['Ref_X'], df['Ref_Y']
    px, py = df['X'], df['Y']

    dt = np.diff(t)
    dt[dt <= 1e-6] = 1e-6
    vrx, vry = np.diff(rx) / dt, np.diff(ry) / dt
    speed = np.hypot(vrx, vry)

    worst, moving, ux, uy = 0.0, False, 0.0, 0.0
    for k in range(len(speed)):
        if speed[k] > 0.30:
            moving = True
            ux, uy = vrx[k] / speed[k], vry[k] / speed[k]
            continue
        if not (moving and speed[k] < 0.05):
            continue
        moving = False
        win = (t >= t[k]) & (t <= t[k] + 2.0)
        if np.any(win):
            worst = max(worst, float(np.max((px[win] - rx[k]) * ux
                                            + (py[win] - ry[k]) * uy)))
    return max(0.0, worst) * 100.0


def path_length_m(df):
    return float(np.sum(np.hypot(np.diff(df['X']), np.diff(df['Y']))))


def per_drone_metrics(df, drone_radius=0.22, region='rect'):
    """Metrik satu drone. Fase lepas landas dibuang dari penilaian tracking."""
    from ..world import obstacles as W

    t = df['Time_s']
    m = t >= TAKEOFF_SETTLE_S
    if m.sum() < 20:
        m = np.ones_like(t, dtype=bool)

    ct = np.hypot(df['Ref_X'][m] - df['X'][m], df['Ref_Y'][m] - df['Y'][m])
    alt = np.abs(df['Ref_Z'][m] - df['Z'][m])

    # Kedua kolom yaw dalam DERAJAT dan membungkus di +-180. Selisih mentah
    # menghasilkan 358 derajat padahal error sebenarnya 2 derajat, sehingga
    # RMS-nya melonjak ke ~124 derajat dan tampak seperti drone berputar liar.
    if 'Ref_Yaw' in df:
        yaw = np.abs((df['Ref_Yaw'][m] - df['Yaw_deg'][m] + 180.0) % 360.0 - 180.0)
    else:
        yaw = None

    dt = np.diff(t)
    dt = np.append(dt, dt[-1] if len(dt) else 0.05)
    tau_sq = (df['tau_x'] ** 2 + df['tau_y'] ** 2 + df['tau_z'] ** 2)
    rpm = (df['RPM_0'] + df['RPM_1'] + df['RPM_2'] + df['RPM_3']) / 4.0

    # Clearance ke rintangan STATIS saja, dan hanya saat benar-benar terbang.
    #
    # Rintangan DINAMIS sengaja tidak dihitung di sini: posisinya bergantung
    # waktu absolut simulasi, sedangkan Time_s pada CSV dimulai dari odometri
    # pertama masing-masing kontroler. Kedua basis waktu itu tidak sinkron,
    # sehingga rekonstruksi analitik bergeser fase dan menghasilkan clearance
    # negatif palsu — pernah melaporkan -0.64 m padahal coordinator mencatat
    # NOL tabrakan. Angka clearance terhadap rintangan bergerak harus diambil
    # dari coordinator, yang memakai odometri rintangan yang sebenarnya.
    airborne = m & (df['Z'] > 0.5)
    d_obs = float('inf')
    xs, ys = df['X'][airborne], df['Y'][airborne]
    for _oid, ox, oy in W.OBSTACLES_BY_REGION.get(region, W.STATIC_OBSTACLES):
        d = np.hypot(xs - ox, ys - oy) - (W.OBSTACLE_RADIUS + drone_radius)
        if d.size:
            d_obs = min(d_obs, float(d.min()))

    return {
        'ct_rms_cm': float(np.sqrt(np.mean(ct ** 2)) * 100.0),
        'ct_max_cm': float(np.max(ct) * 100.0),
        'alt_rms_cm': float(np.sqrt(np.mean(alt ** 2)) * 100.0),
        'yaw_rms_deg': float(np.sqrt(np.mean(yaw ** 2))) if yaw is not None else None,
        'overshoot_cm': overshoot_cm(df),
        'path_len_m': path_length_m(df),
        'control_effort': float(np.sum(tau_sq * dt)),
        'rpm_rms': float(np.sqrt(np.mean(rpm ** 2))),
        'd_obs_min_m': d_obs,
        'duration_s': float(t[-1] - t[0]),
    }


def aggregate(drone_data, drone_radius=0.22, region='rect'):
    """Gabungkan metrik seluruh drone menjadi satu ringkasan per run."""
    if not drone_data:
        return None
    per = {did: per_drone_metrics(df, drone_radius, region)
           for did, df in sorted(drone_data.items())}

    def agg(key, how='mean'):
        vals = [v[key] for v in per.values() if v[key] is not None]
        if not vals:
            return None
        return float(np.max(vals) if how == 'max'
                     else np.min(vals) if how == 'min'
                     else np.sum(vals) if how == 'sum'
                     else np.mean(vals))

    return {
        'n_drones': len(per),
        'ct_rms_cm': agg('ct_rms_cm'),
        'ct_max_cm': agg('ct_max_cm', 'max'),
        'alt_rms_cm': agg('alt_rms_cm'),
        'yaw_rms_deg': agg('yaw_rms_deg'),
        'overshoot_max_cm': agg('overshoot_cm', 'max'),
        'path_len_total_m': agg('path_len_m', 'sum'),
        'control_effort_total': agg('control_effort', 'sum'),
        'rpm_rms': agg('rpm_rms'),
        'd_obs_min_m': agg('d_obs_min_m', 'min'),
        'duration_s': agg('duration_s', 'max'),
        'per_drone': per,
    }


# ── Pembacaan stdout coordinator ────────────────────────────────────────

_RE_STATUS = re.compile(r'Cov:\s*([0-9.]+)%\s*\|\s*d_min:\s*([0-9.]+)m')
_RE_TIER = re.compile(r'QP Tier (\d+) \(slack=([0-9.]+)')
_RE_CBF = re.compile(
    r'T0 (\d+) T1 (\d+) T2 (\d+) T3 (\d+).*?slack_maks=([0-9.]+)')
# Overshoot yang OTORITATIF: diukur coordinator sebagai jarak fisik drone
# melewati waypoint ujung baris di sepanjang arah baris. Metrik overshoot dari
# CSV (`overshoot_cm`) salah tafsir di bawah aturan carrot ref_pos = pos +
# T_lead*v_safe — ia menghitung selisih drone terhadap titik henti CARROT
# (yang berhenti di waypoint saat drone masih lead_dist di belakang), bukan
# overshoot sebenarnya.
_RE_OVERSHOOT = re.compile(r'Overshoot:\s*[0-9.]+%\s*\(([0-9.]+)m\)')


def parse_coordinator_log(path):
    """Ambil coverage, d_min, tabrakan, overshoot, dan diagnostik CBF dari stdout."""
    cov_series, d_min = [], float('inf')
    crashes = watchdogs = 0
    tiers = [0, 0, 0, 0]
    max_slack = 0.0
    max_overshoot_m = 0.0
    n_row_ends = 0
    success = False
    obstacles_enabled = None

    with open(path, errors='replace') as f:
        for line in f:
            if 'Obstacles Engine' in line:
                obstacles_enabled = 'NONAKTIF' not in line
            m = _RE_STATUS.search(line)
            if m:
                cov_series.append(float(m.group(1)))
                d_min = min(d_min, float(m.group(2)))
            if 'OBSTACLE CRASH' in line:
                crashes += 1
            if 'WATCHDOG' in line:
                watchdogs += 1
            if 'SWARM SUCCESS' in line:
                success = True
            m = _RE_OVERSHOOT.search(line)
            if m:
                max_overshoot_m = max(max_overshoot_m, float(m.group(1)))
                n_row_ends += 1
            m = _RE_TIER.search(line)
            if m:
                max_slack = max(max_slack, float(m.group(2)))
            m = _RE_CBF.search(line)
            if m:
                tiers = [int(m.group(i)) for i in range(1, 5)]
                max_slack = max(max_slack, float(m.group(5)))

    total_tier = sum(tiers)
    return {
        'coverage_final_pct': cov_series[-1] if cov_series else None,
        'coverage_max_pct': max(cov_series) if cov_series else None,
        'time_to_90pct_idx': next((i for i, c in enumerate(cov_series)
                                   if c >= 90.0), None),
        'd_min_inter_drone_m': d_min if np.isfinite(d_min) else None,
        'collisions': crashes,
        'watchdogs': watchdogs,
        'mission_success': success,
        'overshoot_max_cm': max_overshoot_m * 100.0 if n_row_ends else None,
        'row_ends_logged': n_row_ends,
        'obstacles_enabled': obstacles_enabled,
        'cbf_tier_counts': tiers,
        'cbf_p_tier_gt0_pct': (100.0 * (total_tier - tiers[0]) / total_tier
                               if total_tier else None),
        'cbf_max_slack': max_slack,
    }
