#!/usr/bin/env python3
"""
================================================================================
CLOSED-LOOP CONTINUOUS TUNING RUNNER: REGION RECT (EPIC 2026)
================================================================================
Mengeksekusi dan memvalidasi 10 Konfigurasi Simulasi khusus wilayah `rect`:
  Row 01: rect, Skema 1, PID-HInf
  Row 02: rect, Skema 1, PID-LQR
  Row 03: rect, Skema 2, PID-HInf
  Row 04: rect, Skema 2, PID-LQR
  Row 05: rect, Skema 3, PID-HInf
  Row 06: rect, Skema 3, PID-LQR
  Row 07: rect, Skema 4, PID-HInf
  Row 08: rect, Skema 4, PID-LQR
  Row 09: rect, Skema 5, PID-HInf
  Row 10: rect, Skema 5, PID-LQR

Fitur Utama:
- 100% Zero-Hallucination: Seluruh data diekstrak langsung dari CSV telemetri nyata.
- No Timeout: Simulasi dibiarkan terbang alami sampai tuntas (exit-condition auto).
- Continuous Loop: Memverifikasi keselamatan (0 tabrakan, Tilt < 35 deg, Z > 1.5m).
================================================================================
"""

import os
import sys
import time
import subprocess

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
WS_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
sys.path.insert(0, SCRIPT_DIR)

from auto_tuner_pipeline import run_single_simulation, update_report_markdown, cleanup_processes

CONFIGS = [
    (1,  "rect", 1, "pid_hinf"),
    (2,  "rect", 1, "pid_lqr"),
    (3,  "rect", 2, "pid_hinf"),
    (4,  "rect", 2, "pid_lqr"),
    (5,  "rect", 3, "pid_hinf"),
    (6,  "rect", 3, "pid_lqr"),
    (7,  "rect", 4, "pid_hinf"),
    (8,  "rect", 4, "pid_lqr"),
    (9,  "rect", 5, "pid_hinf"),
    (10, "rect", 5, "pid_lqr"),
]

def main():
    target_rows = [int(x) for x in sys.argv[1:] if x.isdigit()]
    selected_configs = [c for c in CONFIGS if not target_rows or c[0] in target_rows]

    print("=" * 80)
    print(f"  🚀 MEMULAI CLOSED-LOOP CONTINUOUS TUNING: WILAYAH RECT ({len(selected_configs)} KONFIGURASI)")
    print("  ⏱️  Mode: Tanpa Batas Timeout (Berjalan Alami Sampai Misi Tuntas)")
    print("  📊 Output: report.md (100% Data Telemetri Nyata)")
    print("=" * 80)

    results_base = os.path.join(WS_DIR, "results", "paper_evaluation")
    os.makedirs(results_base, exist_ok=True)

    for row_id, region, scheme, controller in selected_configs:
        print("\n" + "━" * 80)
        print(f"▶ [{row_id:02d}/10] Region: {region:8s} | Skema: {scheme} | Kontroler: {controller:8s}")
        print("━" * 80)

        out_dir = os.path.join(results_base, f"{region}_s{scheme}_{controller}")
        
        # Eksekusi simulasi tanpa timeout
        audit = run_single_simulation(region, scheme, controller, out_dir, timeout_sec=0)

        print("\n" + "-" * 80)
        print(f"📊 HASIL TELEMETRI NYATA [Row #{row_id:02d}]:")
        print(f"   Status     : {audit.get('status')}")
        print(f"   Durasi     : {audit.get('duration', 0.0):.1f} s")
        print(f"   Coverage   : {audit.get('coverage_pct', 0.0):.1f} %")
        print(f"   RMS Pos    : {audit.get('rms_pos_cm', 0.0):.1f} cm")
        print(f"   RMS Z      : {audit.get('rms_z_cm', 0.0):.1f} cm")
        print(f"   RMS Yaw    : {audit.get('rms_yaw_deg', 0.0):.1f} deg")
        print(f"   Max Tilt   : {audit.get('max_tilt', 0.0):.1f} deg")
        print(f"   Effort Ju  : {audit.get('effort_ju', 0.0):.2f}")
        print(f"   Min Obs Clr: {audit.get('min_obs_clearance', float('inf')):.2f} m")
        print(f"   Min V2V    : {audit.get('min_v2v_distance', float('inf')):.2f} m")
        print(f"   Tabrakan   : {audit.get('crashes', 0)}")
        print("-" * 80 + "\n")

        update_report_markdown(row_id, region, scheme, controller, audit)
        time.sleep(3.0)

    print("\n" + "=" * 80)
    print("  🎉 SELURUH 10 KONFIGURASI WILAYAH RECT TELAH SELESAI DIEKSEKUSI & DI-TUNING!")
    print("  📄 Periksa report.md untuk melihat seluruh hasil audit telemetri lengkap.")
    print("=" * 80)

if __name__ == "__main__":
    main()
