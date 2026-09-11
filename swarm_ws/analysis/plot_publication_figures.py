#!/usr/bin/env python3
"""
================================================================================
PUBLICATION FIGURE GENERATOR (EPIC 2026 AIP CONFERENCE)
================================================================================
Menghasilkan seluruh gambar publikasi standar AIP (300 DPI, font Times New Roman,
palet warna ilmiah harmonis) langsung dari data telemetri nyata di results/paper_evaluation/.
File output disimpan ke docs/Progress/figures/
================================================================================
"""

import os
import glob
import math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from typing import Dict, List, Tuple

WS_DIR = "/home/izmaherdian/Documents/swarm-quadrotor-mapping/swarm_ws"
RESULTS_BASE = os.path.join(WS_DIR, "results", "paper_evaluation")
FIG_DIR = "/home/izmaherdian/Documents/swarm-quadrotor-mapping/docs/Progress/figures"

# Style AIP / IEEE Publication Quality
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.size'] = 9
plt.rcParams['axes.labelsize'] = 9
plt.rcParams['legend.fontsize'] = 8
plt.rcParams['xtick.labelsize'] = 8
plt.rcParams['ytick.labelsize'] = 8
plt.rcParams['figure.dpi'] = 300

DRONE_COLORS = [
    '#1f77b4', '#ff7f0e', '#2ca02c', '#d62728',
    '#9467bd', '#8c564b', '#e377c2'
]

def load_flight_csv(csv_path: str) -> np.ndarray:
    """Membaca CSV dan mengembalikan numpy array (t, x, y, z, roll, pitch)."""
    rows = []
    if not os.path.exists(csv_path):
        return np.empty((0, 6))
    with open(csv_path, 'r') as fp:
        for line in fp:
            parts = line.strip().split(',')
            if len(parts) >= 6:
                try:
                    rows.append([float(p) for p in parts[:6]])
                except ValueError:
                    continue
    return np.array(rows) if rows else np.empty((0, 6))

def plot_fig1_trajectories(run_dir: str, title: str, out_filename: str):
    """Plot Fig 1: Lintasan 2D 7 drone di atas partisi wilayah."""
    os.makedirs(FIG_DIR, exist_ok=True)
    csvs = sorted(glob.glob(os.path.join(run_dir, "*.csv")))
    if not csvs:
        return

    fig, ax = plt.subplots(figsize=(4.0, 3.8))
    
    for idx, f in enumerate(csvs):
        data = load_flight_csv(f)
        if len(data) == 0:
            continue
        c = DRONE_COLORS[idx % len(DRONE_COLORS)]
        ax.plot(data[:, 1], data[:, 2], color=c, lw=1.2, label=f'iris_{idx+1}', alpha=0.85)
        # Tandai titik awal dan akhir
        ax.plot(data[0, 1], data[0, 2], 'o', color=c, markersize=3.5)
        ax.plot(data[-1, 1], data[-1, 2], 's', color=c, markersize=3.5)

    ax.set_xlabel('X (meters)')
    ax.set_ylabel('Y (meters)')
    ax.set_title(title, fontsize=9.5, fontweight='bold')
    ax.grid(True, linestyle='--', alpha=0.5)
    ax.axis('equal')
    
    out_path = os.path.join(FIG_DIR, out_filename)
    fig.tight_layout()
    fig.savefig(out_path, dpi=300)
    plt.close(fig)
    print(f"  🎨 Gambar berhasil disimpan: {out_path}")

def plot_fig5_crosstrack_comparison(lqr_runs: List[str], hinf_runs: List[str], out_filename: str = "fig5_crosstrack_spread.png"):
    """Plot Fig 5: Perbandingan Boxplot Cross-Track RMS LQR vs H-infinity."""
    os.makedirs(FIG_DIR, exist_ok=True)
    fig, ax = plt.subplots(figsize=(4.5, 3.2))
    
    lqr_errors = []
    hinf_errors = []
    
    for r in lqr_runs:
        for f in glob.glob(os.path.join(r, "*.csv")):
            d = load_flight_csv(f)
            if len(d) > 100:
                # Estimasi error lateral dari simpangan posisi
                lqr_errors.append(float(np.std(d[100:, 1])) * 100.0)
                
    for r in hinf_runs:
        for f in glob.glob(os.path.join(r, "*.csv")):
            d = load_flight_csv(f)
            if len(d) > 100:
                hinf_errors.append(float(np.std(d[100:, 1])) * 100.0)

    if lqr_errors and hinf_errors:
        bp = ax.boxplot([lqr_errors, hinf_errors], tick_labels=['PID-LQR', 'PID-$H_\\infty$'], patch_artist=True)
        bp['boxes'][0].set_facecolor('#2E86AB')
        bp['boxes'][1].set_facecolor('#A23B72')
        for box in bp['boxes']:
            box.set_alpha(0.7)
            
        ax.set_ylabel('Cross-Track Error (cm)')
        ax.set_title('Cross-Track Tracking Error under Dryden Turbulence', fontsize=9.0, fontweight='bold')
        ax.grid(True, linestyle='--', alpha=0.5)
        
        out_path = os.path.join(FIG_DIR, out_filename)
        fig.tight_layout()
        fig.savefig(out_path, dpi=300)
        plt.close(fig)
        print(f"  🎨 Gambar berhasil disimpan: {out_path}")

def main():
    print("="*80)
    print("🎨 MEMPROSES GAMBAR PUBLIKASI PAPER EPIC 2026")
    print("="*80)
    
    # Cari run yang sudah selesai
    s1_runs = glob.glob(os.path.join(RESULTS_BASE, "*_s1_*"))
    if s1_runs:
        plot_fig1_trajectories(s1_runs[0], "Flown Trajectories (Nominal Condition)", "fig1_trajectories_s1.png")
        
    s2_runs = glob.glob(os.path.join(RESULTS_BASE, "*_s2_*"))
    if s2_runs:
        plot_fig1_trajectories(s2_runs[0], "Flown Trajectories under Turbulence", "fig1_trajectories_s2.png")
        
    lqr_s2 = glob.glob(os.path.join(RESULTS_BASE, "*_s2_pid_lqr*"))
    hinf_s2 = glob.glob(os.path.join(RESULTS_BASE, "*_s2_pid_hinf*"))
    if lqr_s2 and hinf_s2:
        plot_fig5_crosstrack_comparison(lqr_s2, hinf_s2)

if __name__ == "__main__":
    main()
