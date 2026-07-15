import csv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import sys
import os

# Set up paths relative to this script
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
RESULTS_DIR = os.path.join(SCRIPT_DIR, '..', 'results', 'single_agent')
os.makedirs(RESULTS_DIR, exist_ok=True)

# Parse arguments: [script.py] [optional_csv_filename] [optional_output_png]
csv_filename = sys.argv[1] if len(sys.argv) > 1 else 'flight_data_log_lqr.csv'
csv_path = os.path.join(RESULTS_DIR, csv_filename)

# Load data
df = {
    'Time_s': [], 'X': [], 'Y': [], 'Z': [], 'Roll_deg': [], 'Pitch_deg': [], 'Yaw_deg': [],
    'Ref_X': [], 'Ref_Y': [], 'Ref_Z': [], 'Ref_Yaw': [],
    'vx': [], 'vy': [], 'vz': [], 'p': [], 'q': [], 'r': [],
    'T_pert': [], 'tau_x': [], 'tau_y': [], 'tau_z': []
}
with open(csv_path, mode='r') as f:
    reader = csv.DictReader(f)
    for row in reader:
        for k in df.keys():
            df[k].append(float(row[k]))

for k in df.keys():
    df[k] = np.array(df[k])

# Determine controller name for plot titles and legends
if 'hinf' in csv_filename.lower():
    ctrl_name = 'PID H-Infinity'
else:
    ctrl_name = 'PID-LQR'

# Setup limits for plots
thrust_max = 20.0
tau_rp_max = 0.5
tau_y_max = 0.1

fig = plt.figure(figsize=(15, 14))
fig.canvas.manager.set_window_title(f'Pelacakan Komparatif Berfilter {ctrl_name} (4x3 Grid)')

# [1] X
plt.subplot(4, 3, 1)
plt.plot(df['Time_s'], df['X'], 'b-', linewidth=2, label=f'Aktual ({ctrl_name})')
plt.plot(df['Time_s'], df['Ref_X'], 'r--', linewidth=1.5, label='Referensi Filter')
plt.xlabel('t [s]'); plt.ylabel('x [m]'); plt.title('Pelacakan Posisi X'); plt.grid(True); plt.legend()

# [2] Y
plt.subplot(4, 3, 2)
plt.plot(df['Time_s'], df['Y'], 'b-', linewidth=2, label=f'Aktual ({ctrl_name})')
plt.plot(df['Time_s'], df['Ref_Y'], 'r--', linewidth=1.5, label='Referensi Filter')
plt.xlabel('t [s]'); plt.ylabel('y [m]'); plt.title('Pelacakan Posisi Y'); plt.grid(True); plt.legend()

# [3] Z
plt.subplot(4, 3, 3)
plt.plot(df['Time_s'], df['Z'], 'b-', linewidth=2, label=f'Aktual ({ctrl_name})')
plt.plot(df['Time_s'], df['Ref_Z'], 'r--', linewidth=1.5, label='Referensi Filter')
plt.xlabel('t [s]'); plt.ylabel('z [m]'); plt.title('Pelacakan Ketinggian Z'); plt.grid(True); plt.legend()

# [4] Roll (phi)
plt.subplot(4, 3, 4)
plt.plot(df['Time_s'], df['Roll_deg'], 'b-', linewidth=2, label=f'Aktual ({ctrl_name})')
plt.plot(df['Time_s'], np.zeros_like(df['Time_s']), 'r--', linewidth=1.5, label='Referensi')
plt.xlabel('t [s]'); plt.ylabel('Roll [deg]'); plt.title('Sudut Roll (phi)'); plt.grid(True); plt.legend()

# [5] Pitch (theta)
plt.subplot(4, 3, 5)
plt.plot(df['Time_s'], df['Pitch_deg'], 'b-', linewidth=2, label=f'Aktual ({ctrl_name})')
plt.plot(df['Time_s'], np.zeros_like(df['Time_s']), 'r--', linewidth=1.5, label='Referensi')
plt.xlabel('t [s]'); plt.ylabel('Pitch [deg]'); plt.title('Sudut Pitch (theta)'); plt.grid(True); plt.legend()

# [6] Yaw (psi)
plt.subplot(4, 3, 6)
plt.plot(df['Time_s'], df['Yaw_deg'], 'b-', linewidth=2, label=f'Aktual ({ctrl_name})')
plt.plot(df['Time_s'], np.rad2deg(df['Ref_Yaw']), 'r--', linewidth=1.5, label='Referensi Filter')
plt.xlabel('t [s]'); plt.ylabel('Yaw [deg]'); plt.title('Sudut Yaw (psi)'); plt.grid(True); plt.legend()

# [7] Thrust
plt.subplot(4, 3, 7)
plt.plot(df['Time_s'], df['T_pert'], 'k-', linewidth=1.5)
plt.axhline(thrust_max, color='r', linestyle=':', alpha=0.7, label='Limit')
plt.axhline(-thrust_max, color='r', linestyle=':', alpha=0.7)
plt.xlabel('t [s]'); plt.ylabel('T_pert [N]'); plt.title('Thrust Perturbation'); plt.grid(True); plt.legend()

# [8] Torsi Roll & Pitch
plt.subplot(4, 3, 8)
plt.plot(df['Time_s'], df['tau_x'], 'r-', linewidth=1.5, label='tau_x (Roll)')
plt.plot(df['Time_s'], df['tau_y'], 'g-', linewidth=1.5, label='tau_y (Pitch)')
plt.axhline(tau_rp_max, color='k', linestyle=':', alpha=0.7, label='Limit')
plt.axhline(-tau_rp_max, color='k', linestyle=':', alpha=0.7)
plt.xlabel('t [s]'); plt.ylabel('Torsi [Nm]'); plt.title('Torsi Roll & Pitch'); plt.grid(True); plt.legend()

# [9] Torsi Yaw
plt.subplot(4, 3, 9)
plt.plot(df['Time_s'], df['tau_z'], 'm-', linewidth=1.5)
plt.axhline(tau_y_max, color='r', linestyle=':', alpha=0.7, label='Limit')
plt.axhline(-tau_y_max, color='r', linestyle=':', alpha=0.7)
plt.xlabel('t [s]'); plt.ylabel('tau_z [Nm]'); plt.title('Torsi Yaw'); plt.grid(True); plt.legend()

# [10] Kecepatan Linear (vx, vy, vz)
plt.subplot(4, 3, 10)
plt.plot(df['Time_s'], df['vx'], 'r-', linewidth=1.5, label='vx')
plt.plot(df['Time_s'], df['vy'], 'g-', linewidth=1.5, label='vy')
plt.plot(df['Time_s'], df['vz'], 'b-', linewidth=1.5, label='vz')
plt.xlabel('t [s]'); plt.ylabel('Kecepatan [m/s]'); plt.title('Profil Kecepatan Linear'); plt.grid(True); plt.legend()

# [11] Kecepatan Angular (p, q, r)
plt.subplot(4, 3, 11)
plt.plot(df['Time_s'], df['p'], 'r-', linewidth=1.5, label='p (Roll rate)')
plt.plot(df['Time_s'], df['q'], 'g-', linewidth=1.5, label='q (Pitch rate)')
plt.plot(df['Time_s'], df['r'], 'b-', linewidth=1.5, label='r (Yaw rate)')
plt.xlabel('t [s]'); plt.ylabel('Laju [rad/s]'); plt.title('Profil Kecepatan Angular'); plt.grid(True); plt.legend()

# [12] Lintasan 3D
ax = fig.add_subplot(4, 3, 12, projection='3d')
ax.plot(df['X'], df['Y'], df['Z'], 'b-', linewidth=2)
ax.plot([1.0], [1.0], [1.0], 'go', markersize=8, label='Start (1,1,1)')
ax.plot([2.0], [2.0], [2.0], 'rs', markersize=8, label='Target (2,2,2)')
ax.set_xlabel('X [m]'); ax.set_ylabel('Y [m]'); ax.set_zlabel('Z [m]')
ax.set_title('Lintasan Spasial 3D')
ax.legend()
ax.grid(True)

plt.tight_layout()
plt.suptitle(f'Evaluasi Kinerja {ctrl_name} Quadrotor ROS 2', fontsize=16, fontweight='bold')
plt.tight_layout(rect=[0, 0.03, 1, 0.95])
out_path = sys.argv[2] if len(sys.argv) > 2 else os.path.join(RESULTS_DIR, csv_filename.replace('.csv', '.png'))
plt.savefig(out_path, dpi=300)
print(f"Plot saved to {out_path}")
