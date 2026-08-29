#!/usr/bin/env python3
"""Laporan kuantitatif satu atau beberapa run — untuk tabel paper.

Menggabungkan CSV telemetri low-level dengan stdout coordinator, lalu
melaporkan empat kelompok metrik:

  1. Tracking & keselamatan  — cross-track, altitude, overshoot, clearance,
                               jarak antar-drone, jumlah tabrakan
  2. Waktu & cakupan misi    — coverage akhir, durasi, panjang lintasan
  3. Usaha kendali           — integral torsi, RMS RPM
  4. Diagnostik CBF-QP       — P(tier>0), slack maksimum

Pakai:
    python3 -m swarm_high_level.metrics.run_report \\
        lqr:/path/coordinator.log:/path/results/pid_lqr \\
        hinf:/path/coordinator.log:/path/results/pid_hinf \\
        [--json keluaran.json]

Bila data sebuah run tidak lengkap, kolomnya diisi 'n/a' — TIDAK PERNAH
ditambal angka karangan.
"""
import json
import sys

from .telemetry import aggregate, load_drone_csvs, parse_coordinator_log


def analyse(label, log_path, csv_dir, prefix=None):
    if prefix is None:
        prefix = 'hinf' if 'hinf' in label.lower() else 'lqr'
    out = {'label': label, 'log': log_path, 'csv_dir': csv_dir}
    try:
        out.update(parse_coordinator_log(log_path))
    except OSError as exc:
        out['log_error'] = str(exc)
    data = load_drone_csvs(csv_dir, prefix)
    out['telemetry'] = aggregate(data)
    if out['telemetry'] is None:
        out['telemetry_error'] = f'tidak ada CSV pid_{prefix} di {csv_dir}'
    elif out.get('obstacles_enabled') is False:
        # Tanpa rintangan, jarak ke STATIC_OBSTACLES adalah angka phantom.
        out['telemetry']['d_obs_min_m'] = None
    return out


def _fmt(v, spec='.2f'):
    return 'n/a' if v is None else format(v, spec)


ROWS = (
    ('1. TRACKING & KESELAMATAN', None, None),
    ('   Cross-Track RMS (cm)', 't', 'ct_rms_cm'),
    ('   Cross-Track maks (cm)', 't', 'ct_max_cm'),
    ('   Altitude RMS (cm)', 't', 'alt_rms_cm'),
    ('   Yaw RMS (deg)', 't', 'yaw_rms_deg'),
    ('   Overshoot maks (cm)', 'c', 'overshoot_max_cm'),
    ('   Clearance stat. min (m)', 't', 'd_obs_min_m'),
    ('   Jarak antar-drone min (m)', 'c', 'd_min_inter_drone_m'),
    ('   TABRAKAN', 'c', 'collisions'),
    ('   Watchdog', 'c', 'watchdogs'),
    ('2. WAKTU & CAKUPAN MISI', None, None),
    ('   Coverage akhir (%)', 'c', 'coverage_final_pct'),
    ('   Coverage maks (%)', 'c', 'coverage_max_pct'),
    ('   Durasi simulasi (s)', 't', 'duration_s'),
    ('   Panjang lintasan total (m)', 't', 'path_len_total_m'),
    ('3. USAHA KENDALI', None, None),
    ('   Integral torsi total', 't', 'control_effort_total'),
    ('   RPM RMS', 't', 'rpm_rms'),
    ('4. DIAGNOSTIK CBF-QP', None, None),
    ('   P(tier>0) (%)', 'c', 'cbf_p_tier_gt0_pct'),
    ('   Slack maksimum', 'c', 'cbf_max_slack'),
)


def print_table(runs):
    w = 32
    head = f'{"Metrik":<{w}}' + ''.join(f'{r["label"]:>16}' for r in runs)
    print('=' * len(head))
    print(head)
    print('-' * len(head))
    for name, src, key in ROWS:
        if src is None:
            print(f'\n{name}')
            continue
        cells = []
        for r in runs:
            if src == 't':
                d = r.get('telemetry') or {}
            else:
                d = r
            v = d.get(key)
            spec = 'd' if isinstance(v, int) and key in (
                'collisions', 'watchdogs') else '.2f'
            cells.append(f'{_fmt(v, spec):>16}')
        print(f'{name:<{w}}' + ''.join(cells))
    print('=' * len(head))


def main(argv):
    specs, json_out, i = [], None, 0
    while i < len(argv):
        if argv[i] == '--json':
            if i + 1 >= len(argv):
                print('--json butuh nama berkas')
                return 1
            json_out = argv[i + 1]
            i += 2                       # lewati NILAInya, bukan hanya flagnya
            continue
        specs.append(argv[i])
        i += 1

    if not specs:
        print(__doc__)
        return 1

    runs = []
    for s in specs:
        parts = s.split(':')
        if len(parts) < 3:
            print(f'Format salah: {s}  (harap label:log:csv_dir)')
            return 1
        runs.append(analyse(parts[0], parts[1], ':'.join(parts[2:])))

    print_table(runs)

    problems = [r for r in runs if r.get('telemetry_error') or r.get('log_error')]
    for r in problems:
        print(f'  ! {r["label"]}: '
              f'{r.get("telemetry_error") or r.get("log_error")}')

    if json_out:
        with open(json_out, 'w') as f:
            json.dump(runs, f, indent=2, default=float)
        print(f'\nJSON: {json_out}')

    # Gagal bila ada tabrakan pada run mana pun.
    return 1 if any((r.get('collisions') or 0) > 0 for r in runs) else 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
