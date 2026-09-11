#!/usr/bin/env python3
"""
================================================================================
SCHEME 5 MULTI-REGION BATCH RUNNER WITH NATIVE GNOME SCREENCAST RECORDING
================================================================================
Fungsi:
1. Menjalankan ulang seluruh 8 skenario Skema 5 (Hero Multi-Hazard FT-CC).
2. Merekam layar Side-by-Side (Gazebo 3D di kiri + RViz2 di kanan) menjadi
   8 video MP4 resolusi tinggi via D-Bus GNOME Screencast.
3. Menggunakan Adaptive Stopping (berhenti otomatis saat misi selesai atau
   jika terjadi crash/deadlock timeout).
4. Menyimpan data di folder terpisah:
   swarm_ws/results/skema5_video_runs/
     ├── videos/ (8 file MP4)
     ├── telemetry/ (CSV per-drone per-skenario)
     ├── summary.json (Rekap metrik numerik)
     └── REPORT_VIDEO_RUNS.md (Laporan komparatif lengkap)
================================================================================
"""

import os
import sys
import time
import math
import glob
import json
import shutil
import signal
import subprocess
from typing import Dict, List, Tuple, Any
import numpy as np

try:
    import dbus
except ImportError:
    dbus = None

WS_DIR = "/home/izmaherdian/Documents/swarm-quadrotor-mapping/swarm_ws"
SRC_DIR = os.path.join(WS_DIR, "src", "swarm_high_level")
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

from swarm_high_level.world.obstacles import OBSTACLES_BY_REGION, OBSTACLE_RADIUS

BASE_OUTPUT = os.path.join(WS_DIR, "results", "skema5_video_runs")
VIDEOS_DIR = os.path.join(BASE_OUTPUT, "videos")
TELEMETRY_DIR = os.path.join(BASE_OUTPUT, "telemetry")
SUMMARY_JSON = os.path.join(BASE_OUTPUT, "summary.json")
REPORT_MD = os.path.join(BASE_OUTPUT, "REPORT_VIDEO_RUNS.md")

os.makedirs(VIDEOS_DIR, exist_ok=True)
os.makedirs(TELEMETRY_DIR, exist_ok=True)

# 8 Skenario Skema 5
RUN_CONFIGS = [
    ("rect",    "pid_hinf", "01_rect_pid_hinf"),
    ("rect",    "pid_lqr",  "02_rect_pid_lqr"),
    ("l_shape", "pid_hinf", "03_l_shape_pid_hinf"),
    ("l_shape", "pid_lqr",  "04_l_shape_pid_lqr"),
    ("u_shape", "pid_hinf", "05_u_shape_pid_hinf"),
    ("u_shape", "pid_lqr",  "06_u_shape_pid_lqr"),
    ("plus",    "pid_hinf", "07_plus_pid_hinf"),
    ("plus",    "pid_lqr",  "08_plus_pid_lqr"),
]

def cleanup_processes():
    """Hentikan seluruh proses simulator dan node ROS 2."""
    pnames = [
        "gz-sim-main", "parameter_bridge", "spawn_drones", "pid_lqr_node",
        "pid_hinf_node", "dryden_wind_node", "ros_gz_bridge", "rviz2",
        "swarm_mapping_coordinator", "test_7drone_voronoi_mapping"
    ]
    for p in pnames:
        subprocess.run(["pkill", "-9", "-f", p], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    for f in glob.glob("/dev/shm/fastdds_*") + glob.glob("/dev/shm/sem.fastdds_*"):
        try:
            os.remove(f)
        except OSError:
            pass
    time.sleep(1.0)

class GnomeScreencaster:
    """Pengendali Perekaman Layar Native GNOME Shell melalui D-Bus."""
    def __init__(self):
        self.iface = None
        self.recording_file = None
        if dbus is not None:
            try:
                bus = dbus.SessionBus()
                screencast = bus.get_object('org.gnome.Shell.Screencast', '/org/gnome/Shell/Screencast')
                self.iface = dbus.Interface(screencast, 'org.gnome.Shell.Screencast')
            except Exception as e:
                print(f"⚠️ [SCREENCAST] Gagal terhubung ke GNOME D-Bus Screencast: {e}")

    def start(self, template_path: str) -> bool:
        if self.iface is None:
            return False
        try:
            success, target_path = self.iface.Screencast(template_path, {'draw-cursor': dbus.Boolean(True)})
            if success:
                self.recording_file = str(target_path)
                print(f"🎥 [SCREENCAST START] Merekam ke: {self.recording_file}")
                return True
        except Exception as e:
            print(f"⚠️ [SCREENCAST] Error saat start: {e}")
        return False

    def stop(self, final_dest: str):
        if self.iface is None:
            return
        try:
            self.iface.StopScreencast()
            print("⏹️  [SCREENCAST STOP] Menghentikan perekaman...")
            time.sleep(2.5) # Tunggu finalisasi kontainer MP4/WebM
            if self.recording_file and os.path.exists(self.recording_file):
                shutil.move(self.recording_file, final_dest)
                size_mb = os.path.getsize(final_dest) / (1024 * 1024)
                print(f"🎬 [VIDEO DISIMPAN] {final_dest} ({size_mb:.2f} MB)")
            else:
                print(f"⚠️ [SCREENCAST] File rekaman tidak ditemukan di {self.recording_file}")
        except Exception as e:
            print(f"⚠️ [SCREENCAST] Error saat stop: {e}")
        self.recording_file = None

def get_harmonic_obstacles(t: float) -> List[Tuple[float, float, float]]:
    """Posisi rintangan dinamis pola-X harmonik sinusoidal Gazebo."""
    # dynamic_obs_1: (-10*cos(0.15 t), 10*cos(0.15 t))
    x1 = -10.0 * math.cos(0.15 * t)
    y1 = 10.0 * math.cos(0.15 * t)
    # dynamic_obs_2: (10*cos(0.11 t), 10*cos(0.11 t))
    x2 = 10.0 * math.cos(0.11 * t)
    y2 = 10.0 * math.cos(0.11 * t)
    return [(x1, y1, 0.45), (x2, y2, 0.45)]

def audit_telemetry(csv_dir: str, region: str) -> Dict[str, Any]:
    """Audit numerik 100% telemetri CSV nyata per drone."""
    static_table = OBSTACLES_BY_REGION.get(region, OBSTACLES_BY_REGION['rect'])
    static_obs = [(ox, oy, OBSTACLE_RADIUS) for _, ox, oy in static_table]

    csv_files = glob.glob(os.path.join(csv_dir, "flight_data_log_iris_*.csv"))
    if not csv_files:
        return {
            'status': 'FAIL', 'duration': 0.0, 'coverage_pct': 0.0,
            'rms_pos_cm': 0.0, 'rms_z_cm': 0.0, 'rms_yaw_deg': 0.0,
            'max_tilt': 0.0, 'avg_rpm': 0, 'effort_ju': 0.0,
            'min_obs_clearance': -999.0, 'min_v2v_distance': 0.0,
            'min_z': 0.0, 'crashes': 0, 'anomalies': ['File CSV tidak ditemukan']
        }

    all_pos_err_sq, all_z_err_sq, all_yaw_err_sq = [], [], []
    all_rpms = []
    total_effort_ju = 0.0
    max_tilt_global = 0.0
    min_z_global = float('inf')
    min_obs_dist_global = float('inf')
    max_t = 0.0
    crashes = 0
    anomalies = []

    for fpath in csv_files:
        did = os.path.basename(fpath).replace("flight_data_log_iris_", "").replace(".csv", "")
        is_victim = (did in ('4', '7', '2', 'iris_4', 'iris_7', 'iris_2'))

        rows = []
        try:
            with open(fpath, 'r') as fp:
                header = fp.readline()
                for line in fp:
                    p = line.strip().split(',')
                    if len(p) >= 16:
                        rows.append({
                            't': float(p[0]), 'x': float(p[1]), 'y': float(p[2]), 'z': float(p[3]),
                            'ref_x': float(p[4]), 'ref_y': float(p[5]), 'ref_z': float(p[6]),
                            'roll': float(p[7]), 'pitch': float(p[8]), 'yaw': float(p[9]),
                            'rpm_m1': float(p[11]), 'rpm_m2': float(p[12]), 'rpm_m3': float(p[13]), 'rpm_m4': float(p[14]),
                            'effort': float(p[15])
                        })
        except Exception:
            continue

        if not rows:
            continue

        max_t = max(max_t, rows[-1]['t'])
        total_effort_ju += rows[-1]['effort']

        # Evaluasi kondisi terbang di atas 0.8m
        air_rows = [r for r in rows if r['z'] > 0.8]
        if air_rows:
            min_z = min(r['z'] for r in air_rows)
            max_tilt = max(max(abs(r['roll']), abs(r['pitch'])) for r in air_rows)
            min_z_global = min(min_z_global, min_z)
            max_tilt_global = max(max_tilt_global, max_tilt)

            for r in air_rows:
                pos_err = math.hypot(r['x'] - r['ref_x'], r['y'] - r['ref_y'])
                all_pos_err_sq.append(pos_err**2)
                all_z_err_sq.append((r['z'] - r['ref_z'])**2)
                yaw_err = abs((r['yaw'] - 0.0 + 180) % 360 - 180)
                all_yaw_err_sq.append(yaw_err**2)
                all_rpms.extend([r['rpm_m1'], r['rpm_m2'], r['rpm_m3'], r['rpm_m4']])

                # Clearance rintangan statis & dinamis
                for ox, oy, orad in static_obs:
                    d = math.hypot(r['x'] - ox, r['y'] - oy) - (orad + 0.22)
                    min_obs_dist_global = min(min_obs_dist_global, d)

                for dx, dy, drad in get_harmonic_obstacles(r['t']):
                    d = math.hypot(r['x'] - dx, r['y'] - dy) - (drad + 0.22)
                    min_obs_dist_global = min(min_obs_dist_global, d)

            if (not is_victim) and min_z < 1.0:
                crashes += 1
                anomalies.append(f"Drone {did} jatuh ke Z={min_z:.2f}m")
            if (not is_victim) and max_tilt > 45.0:
                anomalies.append(f"Drone {did} miring berlebih {max_tilt:.1f}°")

    rms_pos = math.sqrt(np.mean(all_pos_err_sq)) * 100.0 if all_pos_err_sq else 0.0
    rms_z = math.sqrt(np.mean(all_z_err_sq)) * 100.0 if all_z_err_sq else 0.0
    rms_yaw = math.sqrt(np.mean(all_yaw_err_sq)) if all_yaw_err_sq else 0.0
    avg_rpm = float(np.mean(all_rpms)) if all_rpms else 0.0

    return {
        'status': 'PASS' if crashes == 0 and min_z_global >= 1.5 else 'FAIL',
        'duration': max_t,
        'coverage_pct': 0.0,
        'rms_pos_cm': rms_pos,
        'rms_z_cm': rms_z,
        'rms_yaw_deg': rms_yaw,
        'max_tilt': max_tilt_global,
        'avg_rpm': avg_rpm,
        'effort_ju': total_effort_ju,
        'min_obs_clearance': min_obs_dist_global,
        'min_z': min_z_global if min_z_global < 100 else 0.0,
        'crashes': crashes,
        'anomalies': anomalies
    }

def update_markdown_report(all_results: Dict[str, Any]):
    """Buat dan perbarui berkas laporan markdown tabel lengkap."""
    lines = [
        "# Laporan Hasil Uji Ulang Skema 5 & Rekaman Video (Side-by-Side)",
        "",
        "Laporan ini mencatat hasil pengujian ulang 8 skenario Skema 5 beserta file rekaman video MP4 lengkap.",
        "",
        "| ID | Wilayah | Tipe | Kontroler | Status | Durasi | Coverage | RMS Pos | RMS Z | Max Tilt | Avg RPM | Min Z | Video MP4 |",
        "| :---: | :--- | :---: | :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :--- |",
    ]
    
    for idx, (reg, ctrl, tag) in enumerate(RUN_CONFIGS, 1):
        res = all_results.get(tag)
        cname = "PID-HInf" if "hinf" in ctrl else "PID-LQR"
        geotype = "Konveks" if reg == "rect" else "Non-Konveks"
        vfile = f"{tag}.mp4"
        vpath = f"videos/{vfile}"
        
        if res:
            st = "✅ PASS" if res['status'] == 'PASS' else "⚠️ FAIL/CRASH"
            lines.append(
                f"| **{idx:02d}** | `{reg}` | {geotype} | **{cname}** | {st} | "
                f"{res['duration']:.1f}s | **{res['coverage_pct']:.1f}%** | {res['rms_pos_cm']:.1f}cm | "
                f"{res['rms_z_cm']:.1f}cm | {res['max_tilt']:.1f}° | {int(res['avg_rpm'])} | "
                f"{res['min_z']:.2f}m | [{vfile}]({vpath}) |"
            )
        else:
            lines.append(
                f"| **{idx:02d}** | `{reg}` | {geotype} | **{cname}** | `[SEDANG/BELUM]` | "
                f"--- | --- | --- | --- | --- | --- | --- | --- |"
            )
            
    lines.append("")
    with open(REPORT_MD, 'w') as fp:
        fp.write('\n'.join(lines))

def run_scenario(region: str, controller: str, tag: str, screencaster: GnomeScreencaster) -> Dict[str, Any]:
    """Jalankan 1 skenario simulasi lengkap dengan perekaman video."""
    cleanup_processes()
    
    temp_results_tag = f"run_{tag}"
    raw_sim_results = os.path.join(WS_DIR, "src", "swarm_sim", "results", temp_results_tag)
    dest_telemetry = os.path.join(TELEMETRY_DIR, region, controller)
    os.makedirs(dest_telemetry, exist_ok=True)
    
    final_video_path = os.path.join(VIDEOS_DIR, f"{tag}.mp4")
    temp_video_template = f"/tmp/screencast_{tag}_%d.mp4"

    ctrl_flag = "--pid-hinf" if "hinf" in controller else "--pid-lqr"
    cmd = [
        os.path.join(WS_DIR, "launch_mapping_demo.sh"),
        "-s", "5",
        ctrl_flag,
        "--region", region,
        "--results", temp_results_tag,
        "--exit-after", "12"  # Berhenti 12 detik setelah coverage selesai/stabil
    ]

    print(f"\n{'='*75}")
    print(f"🚀 [SKENARIO {tag.upper()}] Wilayah: {region} | Kontroler: {controller}")
    print(f"   Video Output: {final_video_path}")
    print(f"{'='*75}")

    # 1. Mulai Rekam Layar Native
    screencaster.start(temp_video_template)

    # 2. Jalankan Launch Script GUI Side-by-Side
    log_file = os.path.join(dest_telemetry, "simulation_stdout.log")
    with open(log_file, "w") as out_fp:
        proc = subprocess.Popen(cmd, stdout=out_fp, stderr=subprocess.STDOUT, cwd=WS_DIR, preexec_fn=os.setsid)
        
        start_t = time.time()
        cov_history = []
        
        while True:
            elapsed = time.time() - start_t
            if proc.poll() is not None:
                print("   ℹ️ Proses simulasi selesai normal.")
                break
                
            # Baca coverage terbaru dari log
            current_cov = 0.0
            if os.path.exists(log_file):
                try:
                    with open(log_file, "r") as fp:
                        lines = fp.readlines()[-20:]
                        for l in reversed(lines):
                            if "Cov:" in l:
                                idx = l.find("Cov:")
                                current_cov = float(l[idx+4:idx+10].replace("%", "").strip())
                                break
                except Exception:
                    pass

            cov_history.append((elapsed, current_cov))
            
            # Adaptive Stopping:
            # 1. Jika coverage sudah tinggi >= 90% dan stabil selama 30 detik
            if current_cov >= 90.0 and elapsed > 240.0:
                last_30s = [c for t, c in cov_history if t >= elapsed - 30.0]
                if last_30s and max(last_30s) - min(last_30s) < 0.2:
                    print(f"   🎯 [ADAPTIVE STOP] Coverage tercapai {current_cov:.1f}% dan stabil!")
                    break

            # 2. Jika drone mengalami crash fatal (LQR) dan stagnan di coverage rendah selama > 60 detik
            if elapsed > 120.0 and current_cov < 35.0:
                last_60s = [c for t, c in cov_history if t >= elapsed - 60.0]
                if last_60s and max(last_60s) - min(last_60s) < 0.1:
                    print(f"   🛑 [ADAPTIVE STOP] Crash/stagnasi terdeteksi pada cov {current_cov:.1f}% selama >60s!")
                    break

            # 3. Maximum Safety Timeout (420 detik)
            if elapsed > 420.0:
                print(f"   ⏰ [TIMEOUT] Batas waktu maksimal 420s tercapai.")
                break

            time.sleep(3.0)

    # 3. Hentikan Perekaman Layar
    screencaster.stop(final_video_path)

    # 4. Bersihkan Proses
    cleanup_processes()

    # 5. Salin Data CSV Telemetri
    if os.path.exists(raw_sim_results):
        for f in glob.glob(os.path.join(raw_sim_results, "*.csv")):
            shutil.copy(f, dest_telemetry)

    # 6. Audit Data Telemetri
    audit = audit_telemetry(dest_telemetry, region)
    
    # Ekstrak coverage final tertinggi
    max_cov = 0.0
    if os.path.exists(log_file):
        with open(log_file, "r") as fp:
            for line in fp:
                if "Cov:" in line:
                    try:
                        idx = line.find("Cov:")
                        cov_val = float(line[idx+4:idx+10].replace("%", "").strip())
                        max_cov = max(max_cov, cov_val)
                    except ValueError:
                        pass
    audit['coverage_pct'] = max_cov
    audit['video_file'] = f"{tag}.mp4"

    print(f"📊 [HASIL] Cov: {max_cov:.1f}% | RMS Pos: {audit['rms_pos_cm']:.1f}cm | Status: {audit['status']}")
    return audit

def main():
    print("""
    ========================================================================
     SWARM QUADROTOR MAPPING: SCHEME 5 VIDEO & BENCHMARK SUITE
     (Auto-Execution: 8 Scenarios + Native Side-by-Side Screen Recording)
    ========================================================================
    """)
    screencaster = GnomeScreencaster()
    all_results = {}
    
    if os.path.exists(SUMMARY_JSON):
        try:
            with open(SUMMARY_JSON, 'r') as fp:
                all_results = json.load(fp)
        except Exception:
            all_results = {}

    update_markdown_report(all_results)

    for idx, (reg, ctrl, tag) in enumerate(RUN_CONFIGS, 1):
        print(f"\n▶ [{idx}/8] Memulai {tag}...")
        res = run_scenario(reg, ctrl, tag, screencaster)
        all_results[tag] = res
        
        with open(SUMMARY_JSON, 'w') as fp:
            json.dump(all_results, fp, indent=2)
            
        update_markdown_report(all_results)
        print(f"✨ Skenario {tag} selesai! Jeda 3 detik...")
        time.sleep(3.0)

    print(f"\n🎉 SELURUH 8 SKENARIO & 8 REKAMAN VIDEO TELAH SELESAI!")
    print(f"📂 Video Tersimpan di: {VIDEOS_DIR}")
    print(f"📄 Laporan Markdown: {REPORT_MD}")

if __name__ == "__main__":
    main()
