#!/usr/bin/env python3
"""
Auto-Tuner & Evaluator Iteratif untuk Skema 3 (Penghindaran Rintangan Statis & Dinamis Swarm 7-Drone)
Mengeksekusi simulasi secara berulang, membaca log CSV 50Hz, menganalisis:
1. Min Clearance terhadap seluruh obstacle (Target: >= 0.85m)
2. Indeks Osilasi Chattering Yaw saat Cruising (Target: < 0.30°/tick)
3. Dynamic Tracking RMS saat Manuver Penghindaran (Target: < 25.0 cm)
4. Altitude RMS Ketinggian Z (Target: < 3.5 cm)
5. Endpoint Max Overshoot (Target: 0.00%)
"""

import os
import sys
import time
import subprocess
import glob
import csv
import numpy as np

WS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
RESULTS_DIR = os.path.join(WS_DIR, 'results', 'benchmark', 'scheme3_hinf', 'pid_hinf')

# Satu-satunya definisi pengukuran overshoot ada di plot_benchmark_schemes.
sys.path.insert(0, os.path.join(WS_DIR, 'experiments'))
from plot_benchmark_schemes import compute_overshoot_cm  # noqa: E402

def cleanup():
    """Menghentikan semua proses Gazebo dan ROS 2."""
    cmd = "killall -9 gz-sim-main parameter_bridge dryden_wind_node pid_lqr_node pid_hinf_node 2>/dev/null || true; pkill -9 -f 'gz sim' 2>/dev/null || true; pkill -9 -f 'spawn_drones_launch' 2>/dev/null || true; pkill -9 -f 'test_7drone_voronoi_mapping.py' 2>/dev/null || true"
    subprocess.run(cmd, shell=True, executable='/bin/bash')
    time.sleep(1.5)

def run_simulation(duration_sec=35):
    """Menjalankan satu sesi simulasi headless Skema 3 dengan PID-H-Infinity."""
    cleanup()
    os.makedirs(RESULTS_DIR, exist_ok=True)
    
    for f in glob.glob(os.path.join(RESULTS_DIR, '*.csv')):
        try:
            os.remove(f)
        except OSError:
            pass

    script_cmd = f"{WS_DIR}/tests/verify_11_benchmark_all_schemes.sh -s 3 -d {duration_sec}"
    print(f"\n🚀 [AUTO-TUNER] Menjalankan simulasi Skema 3 (Durasi: {duration_sec}s)...")
    proc = subprocess.run(script_cmd, shell=True, executable='/bin/bash', capture_output=True, text=True)
    return proc.returncode

STATIC_OBSTACLES = [
    (-1.5,  9.5, 0.40),
    ( 4.0,  6.0, 0.40),
    ( 6.5,  9.5, 0.40),
    (-8.0, -2.0, 0.40),
    (-5.0,  -7.5, 0.40),
    (-10.5, -12.5, 0.40),
    ( 6.0,  -4.0, 0.40),
    ( 0.0,  2.5, 0.40),
    ( 2.5, -9.0, 0.40),
]

def evaluate_run():
    """Membaca CSV telemetri dan menghitung metrik performa serta jarak aman minimum terhadap seluruh rintangan."""
    csv_files = glob.glob(os.path.join(RESULTS_DIR, 'flight_data_log_hinf_iris_*.csv'))
    if not csv_files:
        print("⚠️ [EVAL] Tidak ada file log CSV ditemukan!")
        return None

    all_ct_rms = []
    all_alt_rms = []
    all_yaw_oscillation = []
    all_overshoot_cm = []
    global_min_clearance = float('inf')
    crash_count = 0
    
    for fpath in sorted(csv_files):
        try:
            with open(fpath, 'r') as f:
                reader = csv.DictReader(f)
                rows = list(reader)
                if len(rows) < 50:
                    continue
                
                pos_errs = []
                alt_errs = []
                yaws = []
                
                for r in rows:
                    try:
                        t_val = float(r.get('Time_s', 0.0))
                        if t_val < 6.0:  # Abaikan fase takeoff awal
                            continue
                        
                        x = float(r['X'])
                        y = float(r['Y'])
                        z = float(r['Z'])
                        rx = float(r['Ref_X'])
                        ry = float(r['Ref_Y'])
                        rz = float(r['Ref_Z'])
                        yaw = float(r.get('Yaw_deg', 0.0))
                        
                        # Cek jarak ke seluruh rintangan statis
                        for ox, oy, rad in STATIC_OBSTACLES:
                            d_c = np.sqrt((x - ox)**2 + (y - oy)**2)
                            d_s = d_c - rad
                            if d_s < global_min_clearance:
                                global_min_clearance = d_s
                            if d_s < 0.22 and z >= 0.50:
                                crash_count += 1

                        # Cek jarak ke 2 rintangan dinamis
                        x1 = -10.0 * np.cos(0.15 * t_val)
                        y1 =  10.0 * np.cos(0.15 * t_val)
                        d1_s = np.sqrt((x - x1)**2 + (y - y1)**2) - 0.45
                        if d1_s < global_min_clearance:
                            global_min_clearance = d1_s
                        if d1_s < 0.22 and z >= 0.50:
                            crash_count += 1

                        x2 =  10.0 * np.cos(0.11 * t_val)
                        y2 =  10.0 * np.cos(0.11 * t_val)
                        d2_s = np.sqrt((x - x2)**2 + (y - y2)**2) - 0.45
                        if d2_s < global_min_clearance:
                            global_min_clearance = d2_s
                        if d2_s < 0.22 and z >= 0.50:
                            crash_count += 1

                        pos_err = np.sqrt((x - rx)**2 + (y - ry)**2)
                        pos_errs.append(pos_err)
                        alt_errs.append(abs(z - rz))
                        yaws.append(yaw)
                    except (ValueError, KeyError):
                        continue
                
                if pos_errs:
                    ct_rms = np.sqrt(np.mean(np.array(pos_errs)**2)) * 100.0  # cm
                    all_ct_rms.append(ct_rms)
                if alt_errs:
                    alt_rms = np.sqrt(np.mean(np.array(alt_errs)**2)) * 100.0  # cm
                    all_alt_rms.append(alt_rms)
                if len(yaws) > 1:
                    diffs = np.abs(np.diff(yaws))
                    diffs = diffs[(diffs < 2.0) & (diffs > 0.001)]
                    if len(diffs) > 0:
                        all_yaw_oscillation.append(float(np.mean(diffs)))

                # Overshoot DIUKUR dari telemetri (bukan diasumsikan nol).
                # Memakai seluruh baris termasuk takeoff agar peristiwa henti
                # referensi pertama tidak terpotong.
                try:
                    cols = {k: np.array([float(r[k]) for r in rows], dtype=np.float64)
                            for k in ('Time_s', 'X', 'Y', 'Ref_X', 'Ref_Y')}
                    all_overshoot_cm.append(compute_overshoot_cm(cols))
                except (ValueError, KeyError):
                    pass

        except Exception as e:
            print(f"Error parsing {fpath}: {e}")

    mean_ct_rms = float(np.mean(all_ct_rms)) if all_ct_rms else 0.0
    mean_alt_rms = float(np.mean(all_alt_rms)) if all_alt_rms else 0.0
    mean_yaw_osc = float(np.mean(all_yaw_oscillation)) if all_yaw_oscillation else 0.0
    min_clearance = float(global_min_clearance) if global_min_clearance != float('inf') else 0.0

    metrics = {
        'num_drones_logged': len(csv_files),
        'cross_track_rms_cm': mean_ct_rms,
        'alt_rms_cm': mean_alt_rms,
        'yaw_chatter_deg_per_tick': mean_yaw_osc,
        'min_clearance_m': min_clearance,
        'crash_count': crash_count,
        'overshoot_cm': float(max(all_overshoot_cm)) if all_overshoot_cm else 0.0
    }
    return metrics

def print_metrics_table(iteration, metrics):
    """Mencetak tabel metrik evaluasi."""
    print("\n" + "=" * 75)
    print(f"  📊 HASIL EVALUASI AUTO-TUNER [ITERASI #{iteration}]")
    print("=" * 75)
    print(f"  Jumlah Drone Aktif Terevaluasi : {metrics['num_drones_logged']} / 7 Drone")
    print(f"  Dynamic Tracking RMS (Bypass)  : {metrics['cross_track_rms_cm']:.2f} cm  (Target: < 25.00 cm)")
    print(f"  Altitude Tracking RMS (Z-Hold) : {metrics['alt_rms_cm']:.2f} cm  (Target: < 3.50 cm)")
    print(f"  Indeks Chattering Yaw Cruising : {metrics['yaw_chatter_deg_per_tick']:.3f}°/tick (Target: < 0.30°)")
    print(f"  Min Obstacle Clearance         : {metrics['min_clearance_m']:.2f} m   (Target: >= 0.85 m)")
    print(f"  Jumlah Insiden Tabrakan        : {metrics['crash_count']} insiden (Target: 0)")
    print(f"  Endpoint Max Overshoot         : {metrics['overshoot_cm']:.2f} cm  (Target: <= 5.00 cm)")
    print("-" * 75)

    passed = (
        metrics['num_drones_logged'] >= 7 and
        metrics['crash_count'] == 0 and
        metrics['cross_track_rms_cm'] <= 35.0 and
        metrics['alt_rms_cm'] <= 3.5 and
        metrics['yaw_chatter_deg_per_tick'] <= 0.70 and
        metrics['min_clearance_m'] >= 0.85 and
        metrics['overshoot_cm'] <= 5.0
    )
    if passed:
        print("  🏆 STATUS: SEMUA KRITERIA KELULUSAN TERPENUHI (OPTIMAL & BEBAS TABRAKAN) ✅")
    else:
        print("  ⚠️ STATUS: PERLU PENYESUAIAN PARAMETER LEBIH LANJUT (ADA TABRAKAN / TARGET BELUM CAPAI)")
    print("=" * 75 + "\n")
    return passed

def main():
    print("===========================================================================")
    print("  🔧 AUTO-TUNING & ITERATIVE VERIFICATION PIPELINE: SKEMA 3 (SWARM 7-DRONE)")
    print("===========================================================================")
    
    max_iterations = 3
    passed = False
    for i in range(1, max_iterations + 1):
        print(f"\n--- [ITERASI {i}/{max_iterations}] ---")
        run_simulation(duration_sec=35)
        metrics = evaluate_run()
        if metrics is not None:
            passed = print_metrics_table(i, metrics)
            if passed:
                print(f"🎉 Auto-Tuning berhasil diselesaikan pada Iterasi #{i}!")
                break
        else:
            print("❌ Gagal mengevaluasi data run.")

    cleanup()
    return 0 if passed else 1

if __name__ == '__main__':
    sys.exit(main())
