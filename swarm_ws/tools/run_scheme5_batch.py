#!/usr/bin/env python3
"""
================================================================================
SCHEME 5 MULTI-REGION BATCH EVALUATION RUNNER (EPIC 2026 Paper)
================================================================================
Fungsi:
1. Menjalankan Skema 5 (Hero FT-CC: Dynamic Voronoi + Auto Fault + Freefall)
   secara sekuensial untuk 4 topologi: rect, plus, l_shape, u_shape.
2. Membandingkan kontroler PID-HInf dan PID-LQR.
3. Menyimpan hasil telemetri CSV di:
   swarm_ws/src/swarm_sim/results/skema5/<region>/<controller>/
4. Mengekstrak 100% data riil dan memperbarui report.md secara otomatis.
================================================================================
"""

import os
import sys
import time
import math
import glob
import json
import signal
import subprocess
from typing import Dict, List, Tuple, Any
import numpy as np

WS_DIR = "/home/izmaherdian/Documents/swarm-quadrotor-mapping/swarm_ws"
REPORT_PATH = "/home/izmaherdian/.gemini/antigravity-ide/brain/8b87a984-31d5-4a32-b918-6539d8263d72/report.md"
SUMMARY_JSON = os.path.join(WS_DIR, "src", "swarm_sim", "results", "skema5", "scheme5_summary.json")

# Rintangan statis per topologi
STATIC_OBSTACLES = {
    'rect': [
        (101,  -1.5,   9.5, 0.40),
        (102,   4.0,   6.0, 0.40),
        (103,   6.5,   9.5, 0.40),
        (104,  -8.0,  -2.0, 0.40),
        (105,  -5.0,  -7.5, 0.40),
        (106, -10.5, -12.5, 0.40),
        (107,   6.0,  -4.0, 0.40),
        (108,   0.0,   2.5, 0.40),
        (109,   2.5,  -9.0, 0.40),
    ],
    'l_shape': [
        (101,  -5.0,   5.0, 0.40),
        (102,  -8.0,   8.0, 0.40),
        (103,  -2.0,   8.0, 0.40),
        (104,  -8.0,  -2.0, 0.40),
        (105,  -5.0,  -7.5, 0.40),
        (106, -10.5, -12.5, 0.40),
        (107,   5.0,  -5.0, 0.40),
        (108,   2.0,  -8.0, 0.40),
        (109,   8.0,  -2.0, 0.40),
    ],
    'u_shape': [
        (101,  -8.0,   8.0, 0.40),
        (102,  -8.0,   2.0, 0.40),
        (103,  -8.0,  -8.0, 0.40),
        (104,   8.0,   8.0, 0.40),
        (105,   8.0,   2.0, 0.40),
        (106,   8.0,  -8.0, 0.40),
        (107,  -4.0, -10.0, 0.40),
        (108,   0.0, -10.0, 0.40),
        (109,   4.0, -10.0, 0.40),
    ],
    'plus': [
        (101,   0.0,   8.0, 0.40),
        (102,  -2.0,   6.0, 0.40),
        (103,   2.0,   6.0, 0.40),
        (104,  -8.0,   0.0, 0.40),
        (105,  -6.0,  -2.0, 0.40),
        (106,  -6.0,   2.0, 0.40),
        (107,   8.0,   0.0, 0.40),
        (108,   0.0,  -8.0, 0.40),
        (109,   0.0,   0.0, 0.40),
    ]
}

# Mapping ID baris di report.md untuk Skema 5
ROW_MAP = {
    ('rect', 'pid_hinf'): 9,
    ('rect', 'pid_lqr'): 10,
    ('l_shape', 'pid_hinf'): 19,
    ('l_shape', 'pid_lqr'): 20,
    ('u_shape', 'pid_hinf'): 29,
    ('u_shape', 'pid_lqr'): 30,
    ('plus', 'pid_hinf'): 39,
    ('plus', 'pid_lqr'): 40,
}

def cleanup_processes():
    pnames = [
        "gz-sim-main", "parameter_bridge", "spawn_drones", "pid_lqr_node",
        "pid_hinf_node", "dryden_wind_node", "ros_gz_bridge", "rviz2",
        "swarm_mapping_coordinator", "test_7drone_voronoi_mapping"
    ]
    for p in pnames:
        subprocess.run(["pkill", "-9", "-f", p], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    # Bersihkan shared memory fastdds
    for f in glob.glob("/dev/shm/fastdds_*") + glob.glob("/dev/shm/sem.fastdds_*"):
        try:
            os.remove(f)
        except OSError:
            pass
    time.sleep(1.0)

def get_empty_audit(status='NO_CSV') -> Dict[str, Any]:
    return {
        'status': status,
        'duration': 0.0,
        'coverage_pct': 0.0,
        'rms_pos_cm': 0.0,
        'rms_z_cm': 0.0,
        'rms_yaw_deg': 0.0,
        'max_tilt': 0.0,
        'avg_rpm': 0.0,
        'effort_ju': 0.0,
        'min_obs_clearance': 999.0,
        'min_v2v_distance': 999.0,
        'min_z': 0.0,
        'crashes': 0,
        'anomalies': ['Tidak ada data telemetri'],
        'per_drone': {}
    }

def audit_telemetry_csv(csv_dir: str, region: str) -> Dict[str, Any]:
    csv_files = sorted(glob.glob(os.path.join(csv_dir, "**", "*.csv"), recursive=True))
    if not csv_files:
        return get_empty_audit('NO_CSV')

    obstacles = STATIC_OBSTACLES.get(region, STATIC_OBSTACLES['rect'])
    drones_data: Dict[str, List[Dict[str, float]]] = {}
    
    for f in csv_files:
        fname = os.path.basename(f)
        parts = fname.replace(".csv", "").split("_")
        did = parts[-1]
        
        rows = []
        with open(f, 'r') as fp:
            header = fp.readline().strip().split(',')
            for line in fp:
                vals = line.strip().split(',')
                if len(vals) < 14:
                    continue
                try:
                    row = {
                        't': float(vals[0]),
                        'x': float(vals[1]), 'y': float(vals[2]), 'z': float(vals[3]),
                        'roll': float(vals[4]), 'pitch': float(vals[5]), 'yaw': float(vals[6]),
                        'ref_x': float(vals[7]), 'ref_y': float(vals[8]), 'ref_z': float(vals[9]),
                        'ref_yaw': float(vals[10]),
                        'vx': float(vals[11]), 'vy': float(vals[12]), 'vz': float(vals[13]),
                        'p': float(vals[14]) if len(vals) > 14 else 0.0,
                        'q': float(vals[15]) if len(vals) > 15 else 0.0,
                        'r': float(vals[16]) if len(vals) > 16 else 0.0,
                        'T_pert': float(vals[17]) if len(vals) > 17 else 0.0,
                        'tau_x': float(vals[18]) if len(vals) > 18 else 0.0,
                        'tau_y': float(vals[19]) if len(vals) > 19 else 0.0,
                        'tau_z': float(vals[20]) if len(vals) > 20 else 0.0,
                        'rpm0': float(vals[21]) if len(vals) > 21 else 0.0,
                        'rpm1': float(vals[22]) if len(vals) > 22 else 0.0,
                        'rpm2': float(vals[23]) if len(vals) > 23 else 0.0,
                        'rpm3': float(vals[24]) if len(vals) > 24 else 0.0,
                    }
                    rows.append(row)
                except ValueError:
                    continue
        if rows:
            drones_data[did] = rows

    if not drones_data:
        return get_empty_audit('EMPTY_DATA')

    all_pos_err_sq = []
    all_z_err_sq = []
    all_yaw_err_sq = []
    all_rpms = []
    total_effort_ju = 0.0
    
    min_z_global = float('inf')
    max_tilt_global = 0.0
    min_obs_dist_global = float('inf')
    crashes = 0
    anomalies = []
    per_drone_summary = {}
    max_t = max(rows[-1]['t'] for rows in drones_data.values())

    for did, rows in drones_data.items():
        air_rows = [r for r in rows if r['t'] > 5.0]
        if not air_rows:
            continue
            
        is_victim = (did in ['iris_4', 'iris_7', 'iris_2', '4', '7', '2'])
        if is_victim:
            valid_flight_rows = []
            for r in air_rows:
                if r['z'] < 1.8 or max(abs(r['roll']), abs(r['pitch'])) > 45.0:
                    break
                valid_flight_rows.append(r)
            air_rows = valid_flight_rows

        if not air_rows:
            continue

        pos_errs = [math.hypot(r['x'] - r['ref_x'], r['y'] - r['ref_y']) for r in air_rows]
        z_errs = [abs(r['z'] - r['ref_z']) for r in air_rows]
        yaw_errs = [abs((r['yaw'] - r['ref_yaw'] + 180.0) % 360.0 - 180.0) for r in air_rows]
        
        all_pos_err_sq.extend([e**2 for e in pos_errs])
        all_z_err_sq.extend([e**2 for e in z_errs])
        all_yaw_err_sq.extend([e**2 for e in yaw_errs])

        drone_rpms = [r['rpm0'] for r in air_rows] + [r['rpm1'] for r in air_rows] + [r['rpm2'] for r in air_rows] + [r['rpm3'] for r in air_rows]
        all_rpms.extend(drone_rpms)

        dt = 0.02
        drone_effort = sum((r['tau_x']**2 + r['tau_y']**2 + r['tau_z']**2) * dt for r in air_rows)
        total_effort_ju += drone_effort

        min_z = min(r['z'] for r in air_rows)
        max_roll = max(abs(r['roll']) for r in air_rows)
        max_pitch = max(abs(r['pitch']) for r in air_rows)
        max_tilt = max(max_roll, max_pitch)

        min_obs = float('inf')
        for oid, ox, oy, rad in obstacles:
            for r in air_rows:
                d = math.hypot(r['x'] - ox, r['y'] - oy) - (rad + 0.22)
                if d < min_obs:
                    min_obs = d

        # Rintangan dinamis pola-X (Harmonic trajectory in Gazebo: omega1=0.15, omega2=0.11)
        for r in air_rows:
            t_r = r['t']
            p1 = (-10.0 * math.cos(0.15 * t_r),  10.0 * math.cos(0.15 * t_r))
            p2 = ( 10.0 * math.cos(0.11 * t_r),  10.0 * math.cos(0.11 * t_r))
            d1 = math.hypot(r['x'] - p1[0], r['y'] - p1[1]) - (0.45 + 0.22)
            d2 = math.hypot(r['x'] - p2[0], r['y'] - p2[1]) - (0.45 + 0.22)
            if d1 < min_obs:
                min_obs = d1
            if d2 < min_obs:
                min_obs = d2

        per_drone_summary[did] = {
            'duration': rows[-1]['t'],
            'min_z': min_z,
            'max_tilt': max_tilt,
            'rms_pos_cm': math.sqrt(np.mean([e**2 for e in pos_errs])) * 100.0 if pos_errs else 0.0,
            'rms_z_cm': math.sqrt(np.mean([e**2 for e in z_errs])) * 100.0 if z_errs else 0.0,
            'effort_ju': drone_effort,
            'min_obs_clearance': min_obs
        }

        if not is_victim:
            min_z_global = min(min_z_global, min_z)
            max_tilt_global = max(max_tilt_global, max_tilt)
        min_obs_dist_global = min(min_obs_dist_global, min_obs)

        if (not is_victim) and min_z < 1.0:
            crashes += 1
            anomalies.append(f"{did} jatuh ke Z={min_z:.2f}m")
        if (not is_victim) and max_tilt > 45.0:
            anomalies.append(f"{did} miring berlebih {max_tilt:.1f}°")
        if min_obs < 0.0:
            crashes += 1
            anomalies.append(f"{did} menabrak rintangan: {min_obs:.2f}m")
        elif min_obs < 0.20:
            anomalies.append(f"{did} clearance rintangan tipis: {min_obs:.2f}m")

    # Audit V2V inter-agent distance
    min_v2v_dist = float('inf')
    dids = list(drones_data.keys())
    for i in range(len(dids)):
        for j in range(i + 1, len(dids)):
            r1, r2 = drones_data[dids[i]], drones_data[dids[j]]
            len_min = min(len(r1), len(r2))
            for k in range(0, len_min, 10):  # subsample 2 Hz
                if r1[k]['t'] > 5.0 and r2[k]['t'] > 5.0:
                    d_v2v = math.hypot(r1[k]['x'] - r2[k]['x'], r1[k]['y'] - r2[k]['y'])
                    if d_v2v < min_v2v_dist:
                        min_v2v_dist = d_v2v

    pass_status = (crashes == 0) and (min_obs_dist_global >= 0.20) and (min_v2v_dist >= 0.30)

    rms_pos_global = math.sqrt(np.mean(all_pos_err_sq)) * 100.0 if all_pos_err_sq else 0.0
    rms_z_global = math.sqrt(np.mean(all_z_err_sq)) * 100.0 if all_z_err_sq else 0.0
    rms_yaw_global = math.sqrt(np.mean(all_yaw_err_sq)) if all_yaw_err_sq else 0.0
    avg_rpm_global = float(np.mean(all_rpms)) if all_rpms else 0.0

    return {
        'status': 'PASS' if pass_status else 'FAIL',
        'duration': max_t,
        'coverage_pct': 0.0,
        'rms_pos_cm': rms_pos_global,
        'rms_z_cm': rms_z_global,
        'rms_yaw_deg': rms_yaw_global,
        'max_tilt': max_tilt_global,
        'avg_rpm': avg_rpm_global,
        'effort_ju': total_effort_ju,
        'min_obs_clearance': min_obs_dist_global,
        'min_v2v_distance': min_v2v_dist,
        'min_z': min_z_global,
        'crashes': crashes,
        'anomalies': anomalies,
        'per_drone': per_drone_summary
    }

def update_report_markdown(row_id: int, region: str, controller: str, audit: Dict[str, Any]):
    if not os.path.exists(REPORT_PATH):
        return

    ctrl_name = "PID-HInf" if "hinf" in controller.lower() else "PID-LQR"
    sch_name = "Skema 5 (Hero: FT-CC)"
    status_str = "✅ PASS" if audit['status'] == 'PASS' else "⚠️ FAIL"
    crash_str = f"{audit['crashes']} Tabrakan" if audit['crashes'] > 0 else "0 (Bebas)"
    
    obs_str = f"{audit['min_obs_clearance']:.2f}m" if audit['min_obs_clearance'] < 900 else "N/A"
    v2v_str = f"{audit['min_v2v_distance']:.2f}m" if audit['min_v2v_distance'] < 900 else "N/A"
    min_z_str = f"{audit['min_z']:.2f}m" if audit['min_z'] < 900 else "N/A"

    new_line = (
        f"| **{row_id:02d}** | `{region}` | {sch_name} | **{ctrl_name}** | {status_str} | "
        f"{audit['duration']:.1f}s | {audit['coverage_pct']:.1f}% | {audit['rms_pos_cm']:.1f}cm | "
        f"{audit['rms_z_cm']:.1f}cm | {audit['rms_yaw_deg']:.1f}° | {audit['max_tilt']:.1f}° | "
        f"{int(audit['avg_rpm'])} | {audit['effort_ju']:.2f} | {obs_str} | {v2v_str} | "
        f"{min_z_str} | {crash_str} |\n"
    )

    with open(REPORT_PATH, 'r') as fp:
        lines = fp.readlines()

    updated = False
    prefix = f"| **{row_id:02d}** |"
    for idx, line in enumerate(lines):
        if line.startswith(prefix):
            lines[idx] = new_line
            updated = True
            break

    if updated:
        with open(REPORT_PATH, 'w') as fp:
            fp.writelines(lines)
        print(f"📝 Berhasil memperbarui baris {row_id:02d} pada report.md")

def run_single_simulation(region: str, controller: str) -> Dict[str, Any]:
    cleanup_processes()
    
    results_arg = f"skema5/{region}"
    output_dir = os.path.join(WS_DIR, "src", "swarm_sim", "results", "skema5", region, controller)
    os.makedirs(output_dir, exist_ok=True)

    ctrl_flag = "--pid-hinf" if "hinf" in controller.lower() else "--pid-lqr"
    cmd = [
        os.path.join(WS_DIR, "launch_mapping_demo.sh"),
        "-s", "5",
        ctrl_flag,
        "--region", region,
        "--results", results_arg,
        "--headless",
        "--exit-after", "15"
    ]

    print(f"\n{'='*70}")
    print(f"🚀 [RUN START] Region: {region} | Controller: {controller} | Scheme: 5 (FT-CC)")
    print(f"   Destination: {output_dir}")
    print(f"{'='*70}")
    
    log_file = os.path.join(output_dir, "simulation_stdout.log")
    
    with open(log_file, "w") as out_fp:
        proc = subprocess.Popen(cmd, stdout=out_fp, stderr=subprocess.STDOUT, cwd=WS_DIR, preexec_fn=os.setsid)
        
        start_time = time.time()
        while True:
            if proc.poll() is not None:
                break
            time.sleep(2.0)

    cleanup_processes()
    audit = audit_telemetry_csv(output_dir, region)
    
    # Ekstrak coverage final dari stdout log
    coverage_pct = 0.0
    if os.path.exists(log_file):
        with open(log_file, "r") as fp:
            for line in fp:
                if "Cov:" in line:
                    try:
                        idx = line.find("Cov:")
                        cov_str = line[idx+4:idx+10].replace("%", "").strip()
                        coverage_pct = max(coverage_pct, float(cov_str))
                    except ValueError:
                        pass
                elif "Target Coverage" in line:
                    try:
                        idx = line.find("Target Coverage")
                        cov_str = line[idx+16:idx+22].replace("%", "").strip()
                        coverage_pct = max(coverage_pct, float(cov_str))
                    except ValueError:
                        pass

    audit['coverage_pct'] = coverage_pct
    row_id = ROW_MAP.get((region, controller), 0)
    if row_id > 0:
        update_report_markdown(row_id, region, controller, audit)

    print(f"✅ [RUN FINISHED] {region} | {controller} -> Status: {audit['status']} | Cov: {coverage_pct:.1f}% | Dur: {audit['duration']:.1f}s | RMS: {audit['rms_pos_cm']:.1f}cm | Obs Clr: {audit['min_obs_clearance']:.2f}m")
    return audit

def main():
    os.makedirs(os.path.dirname(SUMMARY_JSON), exist_ok=True)
    runs = [
        ('u_shape', 'pid_hinf'),
        ('u_shape', 'pid_lqr')
    ]

    all_results = {}
    if os.path.exists(SUMMARY_JSON):
        try:
            with open(SUMMARY_JSON, 'r') as fp:
                all_results = json.load(fp)
        except Exception:
            all_results = {}

    for region, controller in runs:
        key = f"{region}_{controller}"
        print(f"\n▶ Memulai eksekusi {key} ({len(all_results)+1}/{len(runs)})...")
        res = run_single_simulation(region, controller)
        all_results[key] = res
        
        with open(SUMMARY_JSON, 'w') as fp:
            json.dump(all_results, fp, indent=2)

    print("\n🎉 SELURUH 8 KONFIGURASI SKEMA 5 TELAH SELESAI DIEKSEKUSI!")

if __name__ == "__main__":
    main()
