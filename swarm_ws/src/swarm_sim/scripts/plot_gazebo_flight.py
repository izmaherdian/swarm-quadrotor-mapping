import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import sys

def plot_gazebo_results():
    results_dir = os.path.abspath('src/swarm_sim/results/single_agent')
    csv_file = os.path.join(results_dir, 'flight_data_log_hinf.csv')
    
    if not os.path.exists(csv_file):
        print(f"Error: {csv_file} tidak ditemukan.")
        return
        
    df = pd.read_csv(csv_file)
    t = df['Time_s']
    
    # Buat figure
    fig = plt.figure(figsize=(15, 10))
    fig.suptitle('H-Infinity Flight Performance in Gazebo 3D (Wind Turbulence)', fontsize=16, fontweight='bold')
    
    # 1. 3D Trajectory
    ax1 = fig.add_subplot(2, 2, 1, projection='3d')
    ax1.plot(df['X'], df['Y'], df['Z'], 'b-', label='Actual Flight Path')
    ax1.plot(df['Ref_X'], df['Ref_Y'], df['Ref_Z'], 'r--', label='Target Waypoint')
    ax1.scatter(2.0, 2.0, 2.0, color='red', marker='*', s=200)
    ax1.scatter(0, 0, 0, color='green', marker='o', s=100)
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    ax1.set_title('3D Flight Trajectory')
    ax1.legend()
    
    # 2. X, Y, Z vs Time
    ax2 = fig.add_subplot(2, 2, 2)
    ax2.plot(t, df['X'], label='X Actual')
    ax2.plot(t, df['Ref_X'], 'r--', alpha=0.5)
    ax2.plot(t, df['Y'], label='Y Actual')
    ax2.plot(t, df['Ref_Y'], 'g--', alpha=0.5)
    ax2.plot(t, df['Z'], label='Z Actual')
    ax2.plot(t, df['Ref_Z'], 'b--', alpha=0.5)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Position (m)')
    ax2.set_title('Position vs Time')
    ax2.legend()
    ax2.grid(True)
    
    # 3. Attitude (Roll, Pitch, Yaw)
    ax3 = fig.add_subplot(2, 2, 3)
    ax3.plot(t, np.degrees(df['Roll_deg']), label='Roll')
    ax3.plot(t, np.degrees(df['Pitch_deg']), label='Pitch')
    ax3.plot(t, np.degrees(df['Yaw_deg']), label='Yaw')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Angle (deg)')
    ax3.set_title('Attitude (Euler Angles)')
    ax3.legend()
    ax3.grid(True)
    
    # 4. Motor RPMs
    ax4 = fig.add_subplot(2, 2, 4)
    ax4.plot(t, df['RPM_0'], label='M1 (Front Right)')
    ax4.plot(t, df['RPM_1'], label='M2 (Rear Left)')
    ax4.plot(t, df['RPM_2'], label='M3 (Front Left)')
    ax4.plot(t, df['RPM_3'], label='M4 (Rear Right)')
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('RPM (rad/s)')
    ax4.set_title('Motor Velocities')
    ax4.legend()
    ax4.grid(True)
    
    plt.tight_layout()
    output_png = os.path.join(results_dir, 'gazebo_hinf_plot.png')
    plt.savefig(output_png, dpi=200)
    print(f"Grafik Gazebo berhasil disimpan ke: {output_png}")

if __name__ == '__main__':
    plot_gazebo_results()
