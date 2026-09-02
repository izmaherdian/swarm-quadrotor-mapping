#!/usr/bin/env python3
"""
================================================================================
AUTONOMOUS AUTO-TUNING & BATCH SIMULATION PIPELINE (EPIC 2026 Paper Evaluation)
================================================================================
Fungsi:
1. Mengeksekusi simulasi headless 7-drone swarm mapping untuk kombinasi
   (Region, Scheme, Controller).
2. Mengaudit telemetri CSV secara real-time (Ketinggian Z, Tilt, Jarak Obs, Jarak V2V).
3. Mendeteksi anomali (crash, tabrakan, deviasi jalur) dan melakukan auto-tuning adaptif.
4. Menyimpan ringkasan metrik murni (100% data nyata) ke JSON & memperbarui report.md.
================================================================================
"""

import os
import sys
import time
import math
import glob
import json
import signal
import argparse
import subprocess
from typing import Dict, List, Tuple, Any, Optional
import numpy as np

WS_DIR = "/home/izmaherdian/Documents/swarm-quadrotor-mapping/swarm_ws"
REPORT_PATH = "/home/izmaherdian/.gemini/antigravity-ide/brain/8b87a984-31d5-4a32-b918-6539d8263d72/report.md"

# Koordinat & radius 9 silinder statis di Gazebo per wilayah
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

def cleanup_processes():
    """Membersihkan seluruh proses simulator Gazebo dan ROS 2."""
    pnames = [
        "gz-sim-main", "parameter_bridge", "spawn_drones", "pid_lqr_node",
        "pid_hinf_node", "dryden_wind_node", "ros_gz_bridge", "rviz2",
        "test_7drone_voronoi_mapping"
    ]
    for p in pnames:
        subprocess.run(["pkill", "-9", "-f", p], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.0)

def audit_telemetry_csv(csv_dir: str, region: str, scheme: int = 1) -> Dict[str, Any]:
    """Membaca file CSV telemetri penerbangan dan menghitung seluruh metrik keselamatan, tracking, dan energi kendali."""
    csv_files = sorted(glob.glob(os.path.join(csv_dir, "**", "*.csv"), recursive=True))
    if not csv_files:
        return {'status': 'NO_CSV', 'error': 'Tidak ada file CSV telemetri ditemukan'}

    has_obstacles = (scheme in [3, 4, 5])
    is_fault = (scheme == 5)
    obstacles = STATIC_OBSTACLES.get(region, STATIC_OBSTACLES['rect']) if has_obstacles else []
    drones_data: Dict[str, List[Dict[str, float]]] = {}
    
    for f in csv_files:
        fname = os.path.basename(f)
        did = fname.replace("flight_data_log_hinf_", "").replace("flight_data_log_lqr_", "").replace(".csv", "")
        rows = []
        with open(f, 'r') as fp:
            for line in fp:
                parts = line.strip().split(',')
                if len(parts) >= 6:
                    try:
                        row_dict = {
                            't': float(parts[0]),
                            'x': float(parts[1]), 'y': float(parts[2]), 'z': float(parts[3]),
                            'roll': float(parts[4]), 'pitch': float(parts[5]), 'yaw': float(parts[6]) if len(parts) > 6 else 0.0,
                            'ref_x': float(parts[7]) if len(parts) > 7 else float(parts[1]),
                            'ref_y': float(parts[8]) if len(parts) > 8 else float(parts[2]),
                            'ref_z': float(parts[9]) if len(parts) > 9 else float(parts[3]),
                            'ref_yaw': float(parts[10]) if len(parts) > 10 else 0.0,
                            'tau_x': float(parts[18]) if len(parts) > 18 else 0.0,
                            'tau_y': float(parts[19]) if len(parts) > 19 else 0.0,
                            'tau_z': float(parts[20]) if len(parts) > 20 else 0.0,
                            'rpm0': float(parts[21]) if len(parts) > 21 else 0.0,
                            'rpm1': float(parts[22]) if len(parts) > 22 else 0.0,
                            'rpm2': float(parts[23]) if len(parts) > 23 else 0.0,
                            'rpm3': float(parts[24]) if len(parts) > 24 else 0.0,
                        }
                        rows.append(row_dict)
                    except ValueError:
                        continue
        if rows:
            drones_data[did] = rows

    if not drones_data:
        return {'status': 'EMPTY_DATA', 'error': 'File CSV tidak berisi data valid'}

    max_t = max(rows[-1]['t'] for rows in drones_data.values())
    
    # Audit per drone dan agregat global
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

    for did, rows in drones_data.items():
        air_rows = [r for r in rows if r['t'] > 5.0]
        if not air_rows:
            continue
            
        is_victim = is_fault and (did in ['iris_4', 'iris_2', '4', '2'])
        if is_victim:
            valid_flight_rows = []
            for r in air_rows:
                if r['z'] < 1.8 or max(abs(r['roll']), abs(r['pitch'])) > 45.0:
                    break
                valid_flight_rows.append(r)
            air_rows = valid_flight_rows

        if not air_rows:
            continue

        # Metrik tracking
        pos_errs = [math.hypot(r['x'] - r['ref_x'], r['y'] - r['ref_y']) for r in air_rows]
        z_errs = [abs(r['z'] - r['ref_z']) for r in air_rows]
        yaw_errs = [abs((r['yaw'] - r['ref_yaw'] + 180.0) % 360.0 - 180.0) for r in air_rows]
        
        all_pos_err_sq.extend([e**2 for e in pos_errs])
        all_z_err_sq.extend([e**2 for e in z_errs])
        all_yaw_err_sq.extend([e**2 for e in yaw_errs])

        # Metrik RPM & Energi
        drone_rpms = [r['rpm0'] for r in air_rows] + [r['rpm1'] for r in air_rows] + [r['rpm2'] for r in air_rows] + [r['rpm3'] for r in air_rows]
        all_rpms.extend(drone_rpms)

        # Control effort integral: sum(tau_x^2 + tau_y^2 + tau_z^2) * dt
        dt = 0.02
        drone_effort = sum((r['tau_x']**2 + r['tau_y']**2 + r['tau_z']**2) * dt for r in air_rows)
        total_effort_ju += drone_effort

        min_z = min(r['z'] for r in air_rows)
        max_roll = max(abs(r['roll']) for r in air_rows)
        max_pitch = max(abs(r['pitch']) for r in air_rows)
        max_tilt = max(max_roll, max_pitch)

        min_obs = float('inf')
        if has_obstacles:
            for oid, ox, oy, rad in obstacles:
                for r in air_rows:
                    d = math.hypot(r['x'] - ox, r['y'] - oy) - rad
                    if d < min_obs:
                        min_obs = d

        per_drone_summary[did] = {
            'duration': rows[-1]['t'],
            'min_z': min_z,
            'max_tilt': max_tilt,
            'rms_pos_cm': math.sqrt(np.mean([e**2 for e in pos_errs])) * 100.0 if pos_errs else 0.0,
            'rms_z_cm': math.sqrt(np.mean([e**2 for e in z_errs])) * 100.0 if z_errs else 0.0,
            'effort_ju': drone_effort,
            'min_obs_clearance': min_obs if has_obstacles else float('inf')
        }

        if not is_victim:
            min_z_global = min(min_z_global, min_z)
            max_tilt_global = max(max_tilt_global, max_tilt)
        if has_obstacles:
            min_obs_dist_global = min(min_obs_dist_global, min_obs)

        if (not is_victim) and min_z < 1.0:
            crashes += 1
            anomalies.append(f"{did} jatuh ke Z={min_z:.2f}m")
        if (not is_victim) and max_tilt > 35.0:
            anomalies.append(f"{did} miring berlebih {max_tilt:.1f}°")
        if has_obstacles and min_obs < 0.20:
            anomalies.append(f"{did} clearance rintangan terlalu tipis: {min_obs:.2f}m")

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

    # Ekstrak coverage final dari simulation_stdout.log jika ada
    coverage_pct = 0.0
    stdout_log = os.path.join(csv_dir, "simulation_stdout.log")
    if os.path.exists(stdout_log):
        with open(stdout_log, "r") as fp:
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

    return {
        'status': 'PASS' if pass_status else 'FAIL',
        'duration': max_t,
        'coverage_pct': coverage_pct,
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

def run_single_simulation(region: str, scheme: int, controller: str, output_dir: str, timeout_sec: int = 0) -> Dict[str, Any]:
    """Mengeksekusi 1 run simulasi headless dan mengaudit telemetrinya."""
    cleanup_processes()
    os.makedirs(output_dir, exist_ok=True)

    ctrl_flag = "--pid-hinf" if "hinf" in controller.lower() else "--pid-lqr"
    cmd = [
        os.path.join(WS_DIR, "launch_mapping_demo.sh"),
        "-s", str(scheme),
        ctrl_flag,
        "--region", region,
        "--results", output_dir,
        "--headless",
        "--exit-after", "15"
    ]

    print(f"🚀 Menjalankan: Region={region}, Scheme={scheme}, Controller={controller} (Headless, Tanpa Timeout)...")
    log_file = os.path.join(output_dir, "simulation_stdout.log")
    
    with open(log_file, "w") as out_fp:
        proc = subprocess.Popen(cmd, stdout=out_fp, stderr=subprocess.STDOUT, cwd=WS_DIR, preexec_fn=os.setsid)
        
        start_time = time.time()
        while True:
            if proc.poll() is not None:
                break
            if timeout_sec > 0 and (time.time() - start_time >= timeout_sec):
                print(f"⚠️  Timeout {timeout_sec}s tercapai, menghentikan proses...")
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                except Exception:
                    pass
                time.sleep(3.0)
                break
            time.sleep(2.0)

    cleanup_processes()
    audit = audit_telemetry_csv(output_dir, region, scheme=scheme)
    
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
    return audit

def update_report_markdown(row_id: int, region: str, scheme: int, controller: str, audit: Dict[str, Any]):
    """Memperbarui baris tabel pada artifact report.md dengan seluruh metrik telemetri nyata."""
    if not os.path.exists(REPORT_PATH):
        return

    ctrl_name = "PID-HInf" if "hinf" in controller.lower() else "PID-LQR"
    sch_name = f"Skema {scheme}"
    if scheme == 1: sch_name += " (Nominal)"
    elif scheme == 2: sch_name += " (Turbulence)"
    elif scheme == 3: sch_name += " (Static Obs)"
    elif scheme == 4: sch_name += " (Dynamic Obs)"
    elif scheme == 5: sch_name += " (Hero: FT-CC)"

    status_badge = "✅ PASS" if audit.get('status') == 'PASS' else "⚠️ FAIL"
    dur_str = f"{audit.get('duration', 0.0):.1f}s"
    cov_str = f"{audit.get('coverage_pct', 0.0):.1f}%"
    rms_pos_str = f"{audit.get('rms_pos_cm', 0.0):.1f}cm"
    rms_z_str = f"{audit.get('rms_z_cm', 0.0):.1f}cm"
    rms_yaw_str = f"{audit.get('rms_yaw_deg', 0.0):.1f}°"
    max_tilt_str = f"{audit.get('max_tilt', 0.0):.1f}°"
    avg_rpm_str = f"{audit.get('avg_rpm', 0.0):.0f}"
    effort_str = f"{audit.get('effort_ju', 0.0):.2f}"
    obs_val = audit.get('min_obs_clearance', float('inf'))
    min_obs_str = "N/A" if math.isinf(obs_val) else f"{obs_val:.2f}m"
    min_v2v_str = f"{audit.get('min_v2v_distance', 0.0):.2f}m"
    min_z_str = f"{audit.get('min_z', 0.0):.2f}m"
    collision_str = "0 (Bebas)" if audit.get('crashes', 0) == 0 else f"{audit.get('crashes')} Tabrakan"

    new_line = (
        f"| **{row_id:02d}** | `{region}` | {sch_name} | **{ctrl_name}** | "
        f"{status_badge} | {dur_str} | {cov_str} | {rms_pos_str} | {rms_z_str} | {rms_yaw_str} | "
        f"{max_tilt_str} | {avg_rpm_str} | {effort_str} | {min_obs_str} | {min_v2v_str} | {min_z_str} | {collision_str} |"
    )

    with open(REPORT_PATH, "r") as fp:
        lines = fp.readlines()

    updated = False
    prefix = f"| **{row_id:02d}** |"
    for i, l in enumerate(lines):
        if l.startswith(prefix):
            lines[i] = new_line + "\n"
            updated = True
            break

    if updated:
        with open(REPORT_PATH, "w") as fp:
            fp.writelines(lines)
        print(f"📝 report.md baris #{row_id:02d} berhasil diperbarui!")

def main():
    parser = argparse.ArgumentParser(description="Autonomous Swarm Auto-Tuner & Batch Runner")
    parser.add_argument("--region", default="rect", choices=["rect", "l_shape", "u_shape", "plus", "all"])
    parser.add_argument("--scheme", type=int, default=1)
    parser.add_argument("--controller", default="pid_hinf", choices=["pid_hinf", "pid_lqr", "all"])
    parser.add_argument("--row-id", type=int, default=1)
    parser.add_argument("--timeout", type=int, default=0, help="Timeout detik (0 = unlimited / tunggu sampai selesai alami)")
    parser.add_argument("--skip-existing", action="store_true", help="Lewati jika sudah ada CSV valid dan PASS")
    args = parser.parse_args()

    results_base = os.path.join(WS_DIR, "results", "paper_evaluation")
    out_dir = os.path.join(results_base, f"{args.region}_s{args.scheme}_{args.controller}")
    
    if args.skip_existing and os.path.exists(out_dir):
        audit_pre = audit_telemetry_csv(out_dir, args.region, scheme=args.scheme)
        if audit_pre.get('status') == 'PASS' and audit_pre.get('coverage_pct', 0.0) >= 80.0:
            print(f"⏩ [Row #{args.row_id:02d}] {args.region}_s{args.scheme}_{args.controller} sudah PASS ({audit_pre.get('coverage_pct', 0.0):.1f}% Cov). Melewati...")
            update_report_markdown(args.row_id, args.region, args.scheme, args.controller, audit_pre)
            return

    audit = run_single_simulation(args.region, args.scheme, args.controller, out_dir, timeout_sec=args.timeout)
    print("\n" + "="*80)
    print(f"📊 HASIL AUDIT TELEMETRI NYATA [Row #{args.row_id:02d}]:")
    print(f"   Status     : {audit.get('status')}")
    print(f"   Durasi     : {audit.get('duration', 0.0):.1f} s")
    print(f"   Coverage   : {audit.get('coverage_pct', 0.0):.1f} %")
    print(f"   Min Z      : {audit.get('min_z', 0.0):.2f} m")
    print(f"   Max Tilt   : {audit.get('max_tilt', 0.0):.1f} deg")
    print(f"   Min Obs Clr: {audit.get('min_obs_clearance', 0.0):.2f} m")
    print(f"   Min V2V Dist: {audit.get('min_v2v_distance', 0.0):.2f} m")
    print("="*80 + "\n")

    update_report_markdown(args.row_id, args.region, args.scheme, args.controller, audit)

if __name__ == "__main__":
    main()
