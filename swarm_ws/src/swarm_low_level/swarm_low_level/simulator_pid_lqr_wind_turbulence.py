import numpy as np
import matplotlib.pyplot as plt
from solver_pid_lqr import PIDLQRSolver
import yaml
import os

class PID:
    def __init__(self, Kp, Ki, Kd, dt, out_min=-np.inf, out_max=np.inf):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self.out_min = out_min
        self.out_max = out_max
        self.integral = 0
        self.prev_error = 0
        
    def compute(self, error):
        proportional = self.Kp * error
        derivative = self.Kd * (error - self.prev_error) / self.dt
        self.prev_error = error
        
        output_no_i = proportional + derivative
        if not ((output_no_i > self.out_max and error > 0) or (output_no_i < self.out_min and error < 0)):
            self.integral += error * self.dt
            
        output = proportional + self.Ki * self.integral + derivative
        return np.clip(output, self.out_min, self.out_max)

def calc_transient(time, response, target, start):
    step_size = target - start
    if abs(step_size) < 1e-6:
        return 0.0, 0.0, 0.0
    y = response - start
    y_target = target - start
    
    # Overshoot
    max_val = np.max(y)
    overshoot = max(0.0, (max_val - y_target) / y_target * 100.0)
    
    # Rise time (10% to 90%)
    try:
        t_10 = time[np.where(y >= 0.1 * y_target)[0][0]]
        t_90 = time[np.where(y >= 0.9 * y_target)[0][0]]
        tr = t_90 - t_10
    except IndexError:
        tr = 0.0
        
    # Settling time (2% band)
    band = 0.02 * y_target
    outside = np.where(np.abs(y - y_target) > band)[0]
    if len(outside) > 0:
        ts = time[outside[-1]]
    else:
        ts = 0.0
        
    return overshoot, tr, ts

def simulate():
    # Load parameters dari file YAML terpisah
    config_path = os.path.join(os.path.dirname(__file__), '../config/quadrotor_params.yaml')
    with open(config_path, 'r') as f:
        yaml_data = yaml.safe_load(f)
        params = yaml_data['physics']
        limits = yaml_data['actuator_limits']
        
    # Set seed for perfectly identical wind turbulence evaluation
    np.random.seed(42)
        
    # 1. Panggil nilai Gain dari Solver dengan parameter spesifik
    solver = PIDLQRSolver(params)
    gains = solver.compute_all_gains()
    
    # 2. Setup Parameter Simulasi Waktu Kontinu
    dt = 0.01  # Waktu sampling 10ms
    t_end = 20.0 # Simulasi selama 10 detik
    time = np.arange(0, t_end, dt)
    n_steps = len(time)
    
    # 3. Target Referensi (Titik akhir 2, 2, 2)
    x_cmd, y_cmd, z_cmd = 2.0, 2.0, 2.0
    yaw_cmd = np.deg2rad(10) # 10 derajat
    
    # Array State: [x, vx, theta, q, y, vy, phi, p, z, vz, yaw, r]
    states = np.zeros((12, n_steps))
    # Mulai dari titik awal 1, 1, 1
    states[0, 0] = 1.0
    states[4, 0] = 1.0
    states[8, 0] = 1.0
    
    # Array untuk menyimpan referensi ber-filter (seperti x_ref_f di Matlab)
    refs_f = np.zeros((4, n_steps))
    refs_f[0, 0], refs_f[1, 0], refs_f[2, 0] = 1.0, 1.0, 1.0
    
    # Array untuk menyimpan sinyal kontrol [U1, U2, U3, U4] -> [T_pert, tau_x, tau_y, tau_z]
    u_opt = np.zeros((4, n_steps))
    
    # 3. Minta Solver menghitung parameter PID
    gains = solver.compute_all_gains()
    
    # State Filter [posisi_f, kecepatan_f]
    filt_x = [1.0, 0.0]
    filt_y = [1.0, 0.0]
    filt_z = [1.0, 0.0]
    filt_yaw = [0.0, 0.0]
    
    # Filter Referensi diperlambat (wn = 1.5 rad/s) untuk ts ~ 3.5 detik
    w_n_sq = 2.25       # 1.5^2
    two_zeta_wn = 3.0   # 2 * 1.0 * 1.5
    
    # 4. Inisialisasi Blok PID (Sama persis dengan diagram Simulink)
    pid_x_out = PID(gains['x_outer']['Kp'], gains['x_outer']['Ki'], gains['x_outer']['Kd'], dt, -limits['angle_max'], limits['angle_max'])
    pid_x_in  = PID(gains['x_inner']['Kp'], gains['x_inner']['Ki'], gains['x_inner']['Kd'], dt, -limits['tau_rp_max'], limits['tau_rp_max'])
    
    pid_y_out = PID(gains['y_outer']['Kp'], gains['y_outer']['Ki'], gains['y_outer']['Kd'], dt, -limits['angle_max'], limits['angle_max'])
    pid_y_in  = PID(gains['y_inner']['Kp'], gains['y_inner']['Ki'], gains['y_inner']['Kd'], dt, -limits['tau_rp_max'], limits['tau_rp_max'])
    
    pid_z   = PID(gains['z']['Kp'], gains['z']['Ki'], gains['z']['Kd'], dt, -limits['thrust_max'], limits['thrust_max'])
    pid_yaw = PID(gains['yaw']['Kp'], gains['yaw']['Ki'], gains['yaw']['Kd'], dt, -limits['tau_y_max'], limits['tau_y_max'])
    
    g, m = params['g'], params['mass']
    Ix, Iy, Iz = params['ix'], params['iy'], params['iz']
    
    act_phys = yaml_data['actuator_physics']
    kf, km, tau_m = act_phys['kf'], act_phys['km'], act_phys['tau_m']
    w_max, w_min = act_phys['omega_max'], act_phys['omega_min']
    d = params['arm_length'] * 0.707106781  # sin(45 deg)
    
    # Inisialisasi State Kecepatan Baling-Baling pada kondisi Hover
    w_hover = np.sqrt((m * g / 4.0) / kf)
    w_actual = np.array([w_hover, w_hover, w_hover, w_hover])
    
    # Matriks Control Allocation (Mixer)
    M = np.array([
        [kf, kf, kf, kf],
        [-kf*d, kf*d, kf*d, -kf*d],
        [-kf*d, kf*d, -kf*d, kf*d],
        [-km, -km, km, km]
    ])
    M_inv = np.linalg.inv(M)
    
    # 5. Looping Integrasi Numerik (Mesin Simulasi)
    # === KONFIGURASI DRYDEN TURBULENCE ===
    tau_wind = 0.5  # Time constant (korelasi spasial/temporal)
    sigma_wind = 3.0 # Intensitas (Standard Deviation)
    alpha_w = np.exp(-dt / tau_wind)
    beta_w = sigma_wind * np.sqrt(1 - alpha_w**2)
    wind_state = np.zeros(3)
    wind_history = np.zeros((3, n_steps))
    
    for k in range(1, n_steps):
        x, vx, theta, q = states[0, k-1], states[1, k-1], states[2, k-1], states[3, k-1]
        y, vy, phi, p   = states[4, k-1], states[5, k-1], states[6, k-1], states[7, k-1]
        z, vz           = states[8, k-1], states[9, k-1]
        yaw, r          = states[10, k-1], states[11, k-1]
        
        # === LOW PASS FILTER REFERENSI (33.99 / (s^2 + 11.66s + 33.99)) ===
        filt_x[1] += (w_n_sq * (x_cmd - filt_x[0]) - two_zeta_wn * filt_x[1]) * dt
        filt_x[0] += filt_x[1] * dt
        
        filt_y[1] += (w_n_sq * (y_cmd - filt_y[0]) - two_zeta_wn * filt_y[1]) * dt
        filt_y[0] += filt_y[1] * dt
        
        filt_z[1] += (w_n_sq * (z_cmd - filt_z[0]) - two_zeta_wn * filt_z[1]) * dt
        filt_z[0] += filt_z[1] * dt
        
        filt_yaw[1] += (w_n_sq * (yaw_cmd - filt_yaw[0]) - two_zeta_wn * filt_yaw[1]) * dt
        filt_yaw[0] += filt_yaw[1] * dt
        
        refs_f[:, k] = [filt_x[0], filt_y[0], filt_z[0], filt_yaw[0]]
        
        # === ARSITEKTUR KONTROL (Dari diagram .pdf) ===
        # Subsistem Longitudinal (X -> Pitch)
        err_x = filt_x[0] - x
        theta_ref = pid_x_out.compute(err_x)
        err_theta = theta_ref - theta
        uy_pid = pid_x_in.compute(err_theta)
        
        # Subsistem Lateral (Y -> Roll)
        err_y = filt_y[0] - y
        phi_ref = pid_y_out.compute(err_y)
        err_phi = phi_ref - phi
        ux_pid = pid_y_in.compute(err_phi)
        
        # Subsistem Altitude (Z -> Thrust)
        err_z = filt_z[0] - z
        uz_pid = pid_z.compute(err_z) 
        
        # Subsistem Yaw
        err_yaw = filt_yaw[0] - yaw
        uyaw_pid = pid_yaw.compute(err_yaw)
        
        # === MIXER & DINAMIKA AKTUATOR ORDE 1 ===
        U_cmd = np.array([uz_pid + (m * g), ux_pid, uy_pid, uyaw_pid])
        w_sq_cmd = M_inv @ U_cmd
        w_cmd = np.sqrt(np.maximum(w_sq_cmd, 0))
        w_cmd = np.clip(w_cmd, w_min, w_max)
        w_actual = w_actual + ((w_cmd - w_actual) / tau_m) * dt
        U_actual = M @ (w_actual**2)
        T_pert = U_actual[0] - (m * g)
        tau_x = U_actual[1]
        tau_y = U_actual[2]
        tau_z = U_actual[3]
        
        # Simpan sinyal kontrol
        u_opt[:, k] = [T_pert, tau_x, tau_y, tau_z]
        
        # === PERSAMAAN FISIKA KINEMATIKA (Quadrotor) ===
        # GANGGUAN ANGIN (Dryden Turbulence stokastik) mulai detik ke-5
        if (k * dt) >= 5.0:
            wind_state = alpha_w * wind_state + beta_w * np.random.randn(3)
        F_wind_x, F_wind_y, F_wind_z = wind_state
        wind_history[:, k] = wind_state
        
        # Sumbu X (dipengaruhi Pitch)
        x_new = x + vx * dt
        vx_new = vx + (g * theta + (F_wind_x / m)) * dt
        theta_new = theta + q * dt
        q_new = q + (tau_y / Iy) * dt
        
        # Sumbu Y (dipengaruhi Roll)
        y_new = y + vy * dt
        vy_new = vy + (-g * phi + (F_wind_y / m)) * dt
        phi_new = phi + p * dt
        p_new = p + (tau_x / Ix) * dt
        
        # Sumbu Z (Ketinggian)
        z_new = z + vz * dt
        vz_new = vz + ((T_pert + F_wind_z) / m) * dt
        
        # Sumbu Yaw (Rotasi)
        yaw_new = yaw + r * dt
        r_new = r + (tau_z / Iz) * dt
        
        states[:, k] = [x_new, vx_new, theta_new, q_new, y_new, vy_new, phi_new, p_new, z_new, vz_new, yaw_new, r_new]

    target_pos = [x_cmd, y_cmd, z_cmd]
    e_x = states[0, 1:] - refs_f[0, 1:]
    e_y = states[4, 1:] - refs_f[1, 1:]
    e_z = states[8, 1:] - refs_f[2, 1:]
    e_yaw = np.rad2deg(states[10, 1:]) - np.rad2deg(refs_f[3, 1:])
    
    rmse_x = np.sqrt(np.mean(e_x**2))
    rmse_y = np.sqrt(np.mean(e_y**2))
    rmse_z = np.sqrt(np.mean(e_z**2))
    rmse_3d = np.sqrt(np.mean(e_x**2 + e_y**2 + e_z**2))
    rmse_yaw = np.sqrt(np.mean(e_yaw**2))
    
    # Energi Kontrol Total (Eu) menggunakan Trapezoidal Integration
    E_u_val = np.trapezoid(np.sum(u_opt**2, axis=0), time)
    
    print("\n=========================================")
    print("     HASIL EVALUASI METRIK KINERJA")
    print("=========================================")
    print(f"RMSE Sumbu X               = {rmse_x:.6f} m")
    print(f"RMSE Sumbu Y               = {rmse_y:.6f} m")
    print(f"RMSE Ketinggian Z          = {rmse_z:.6f} m")
    print("-----------------------------------------")
    print(f"RMSE Total Posisi 3D (XYZ) = {rmse_3d:.6f} m")
    print(f"RMSE Total Sudut Yaw (psi) = {rmse_yaw:.6f} deg")
    print("-----------------------------------------")
    print(f"Energi Kontrol Total (Eu)  = {E_u_val:.4f} N^2.s")
    print("=========================================")
    
    # Menghitung Metrik Transien (Overshoot, Rise Time, Settling Time)
    os_x, tr_x, ts_x = calc_transient(time, states[0, :], x_cmd, states[0, 0])
    os_y, tr_y, ts_y = calc_transient(time, states[4, :], y_cmd, states[4, 0])
    os_z, tr_z, ts_z = calc_transient(time, states[8, :], z_cmd, states[8, 0])
    os_yaw, tr_yaw, ts_yaw = calc_transient(time, states[10, :], yaw_cmd, states[10, 0])
    
    print("\n=========================================")
    print("     METRIK TRANSIEN (TIME-DOMAIN)")
    print("=========================================")
    print(f"X   -> Overshoot: {os_x:6.2f}%, tr: {tr_x:.3f}s, ts: {ts_x:.3f}s")
    print(f"Y   -> Overshoot: {os_y:6.2f}%, tr: {tr_y:.3f}s, ts: {ts_y:.3f}s")
    print(f"Z   -> Overshoot: {os_z:6.2f}%, tr: {tr_z:.3f}s, ts: {ts_z:.3f}s")
    print(f"Yaw -> Overshoot: {os_yaw:6.2f}%, tr: {tr_yaw:.3f}s, ts: {ts_yaw:.3f}s")
    print("=========================================\n")

    # 6. Render Grafik 3x3 (Sesuai Gambar 2 Matlab)
    fig = plt.figure(figsize=(15, 17))
    fig.canvas.manager.set_window_title('Pelacakan Komparatif Berfilter PID-LQR (4x3 Grid)')
    
    # [1] X
    plt.subplot(5, 3, 1)
    plt.plot(time, states[0, :], 'b-', linewidth=2, label='Aktual (PID-LQR)')
    plt.plot(time, refs_f[0, :], 'r--', linewidth=1.5, label='Referensi Filter')
    plt.xlabel('t [s]'); plt.ylabel('x [m]'); plt.title('Pelacakan Posisi X'); plt.grid(True); plt.legend()
    
    # [2] Y
    plt.subplot(5, 3, 2)
    plt.plot(time, states[4, :], 'b-', linewidth=2, label='Aktual (PID-LQR)')
    plt.plot(time, refs_f[1, :], 'r--', linewidth=1.5, label='Referensi Filter')
    plt.xlabel('t [s]'); plt.ylabel('y [m]'); plt.title('Pelacakan Posisi Y'); plt.grid(True); plt.legend()
    
    # [3] Z
    plt.subplot(5, 3, 3)
    plt.plot(time, states[8, :], 'b-', linewidth=2, label='Aktual (PID-LQR)')
    plt.plot(time, refs_f[2, :], 'r--', linewidth=1.5, label='Referensi Filter')
    plt.xlabel('t [s]'); plt.ylabel('z [m]'); plt.title('Pelacakan Ketinggian Z'); plt.grid(True); plt.legend()
    
    # [4] Roll (phi)
    plt.subplot(5, 3, 4)
    plt.plot(time, np.rad2deg(states[6, :]), 'b-', linewidth=2, label='Aktual (PID-LQR)')
    plt.plot(time, np.zeros_like(time), 'r--', linewidth=1.5, label='Referensi')
    plt.xlabel('t [s]'); plt.ylabel('Roll [deg]'); plt.title('Sudut Roll (phi)'); plt.grid(True); plt.legend()
    
    # [5] Pitch (theta)
    plt.subplot(5, 3, 5)
    plt.plot(time, np.rad2deg(states[2, :]), 'b-', linewidth=2, label='Aktual (PID-LQR)')
    plt.plot(time, np.zeros_like(time), 'r--', linewidth=1.5, label='Referensi')
    plt.xlabel('t [s]'); plt.ylabel('Pitch [deg]'); plt.title('Sudut Pitch (theta)'); plt.grid(True); plt.legend()
    
    # [6] Yaw (psi)
    plt.subplot(5, 3, 6)
    plt.plot(time, np.rad2deg(states[10, :]), 'b-', linewidth=2, label='Aktual (PID-LQR)')
    plt.plot(time, np.rad2deg(refs_f[3, :]), 'r--', linewidth=1.5, label='Referensi Filter')
    plt.xlabel('t [s]'); plt.ylabel('Yaw [deg]'); plt.title('Sudut Yaw (psi)'); plt.grid(True); plt.legend()
    
    # [10] Kecepatan Linear (vx, vy, vz)
    plt.subplot(5, 3, 10)
    plt.plot(time, states[1, :], 'r-', linewidth=1.5, label='vx')
    plt.plot(time, states[5, :], 'g-', linewidth=1.5, label='vy')
    plt.plot(time, states[9, :], 'b-', linewidth=1.5, label='vz')
    plt.xlabel('t [s]'); plt.ylabel('Kecepatan [m/s]'); plt.title('Profil Kecepatan Linear'); plt.grid(True); plt.legend()
    
    # [11] Kecepatan Angular (p, q, r)
    plt.subplot(5, 3, 11)
    plt.plot(time, states[7, :], 'r-', linewidth=1.5, label='p (Roll rate)')
    plt.plot(time, states[3, :], 'g-', linewidth=1.5, label='q (Pitch rate)')
    plt.plot(time, states[11, :], 'b-', linewidth=1.5, label='r (Yaw rate)')
    plt.xlabel('t [s]'); plt.ylabel('Laju [rad/s]'); plt.title('Profil Kecepatan Angular'); plt.grid(True); plt.legend()
    
    # [12] Lintasan 3D
    ax = fig.add_subplot(5, 3, 12, projection='3d')
    ax.plot(states[0, :], states[4, :], states[8, :], 'b-', linewidth=2)
    ax.plot([1.0], [1.0], [1.0], 'go', markersize=8, label='Start (1,1,1)') # Start point
    ax.plot([x_cmd], [y_cmd], [z_cmd], 'rs', markersize=8, label='Target (2,2,2)') # End point
    ax.set_xlabel('X [m]'); ax.set_ylabel('Y [m]'); ax.set_zlabel('Z [m]')
    ax.set_title('Lintasan Spasial 3D')
    ax.legend()
    ax.grid(True)
    
    
    # [7] Thrust
    plt.subplot(5, 3, 7)
    plt.plot(time, u_opt[0, :], 'k-', linewidth=1.5)
    plt.axhline(limits['thrust_max'], color='r', linestyle=':', alpha=0.7, label='Limit')
    plt.axhline(limits['thrust_min'], color='r', linestyle=':', alpha=0.7)
    plt.xlabel('t [s]'); plt.ylabel('T_pert [N]'); plt.title('Thrust Perturbation'); plt.grid(True); plt.legend()
    
    # [8] Torsi Roll & Pitch
    plt.subplot(5, 3, 8)
    plt.plot(time, u_opt[1, :], 'r-', linewidth=1.5, label='tau_x (Roll)')
    plt.plot(time, u_opt[2, :], 'g-', linewidth=1.5, label='tau_y (Pitch)')
    plt.axhline(limits['tau_rp_max'], color='k', linestyle=':', alpha=0.7, label='Limit')
    plt.axhline(-limits['tau_rp_max'], color='k', linestyle=':', alpha=0.7)
    plt.xlabel('t [s]'); plt.ylabel('Torsi [Nm]'); plt.title('Torsi Roll & Pitch'); plt.grid(True); plt.legend()
    
    # [9] Torsi Yaw
    plt.subplot(5, 3, 9)
    plt.plot(time, u_opt[3, :], 'm-', linewidth=1.5)
    plt.axhline(limits['tau_y_max'], color='r', linestyle=':', alpha=0.7, label='Limit')
    plt.axhline(-limits['tau_y_max'], color='r', linestyle=':', alpha=0.7)
    plt.xlabel('t [s]'); plt.ylabel('tau_z [Nm]'); plt.title('Torsi Yaw'); plt.grid(True); plt.legend()
    
    # [13-15] Profil Gangguan Angin (Dryden)
    ax_wind = plt.subplot(5, 3, (13, 15))
    ax_wind.plot(time, wind_history[0, :], 'r-', alpha=0.7, label='F_wind_x')
    ax_wind.plot(time, wind_history[1, :], 'g-', alpha=0.7, label='F_wind_y')
    ax_wind.plot(time, wind_history[2, :], 'b-', alpha=0.7, label='F_wind_z')
    ax_wind.set_xlabel('t [s]')
    ax_wind.set_ylabel('Force [N]')
    ax_wind.set_title('Profil Gangguan Angin Stokastik (Dryden Model)')
    ax_wind.grid(True)
    ax_wind.legend()
    
    plt.tight_layout()
    plt.suptitle('Evaluasi Kinerja PID-LQR Quadrotor (Dryden Turbulence Stochastic)', fontsize=16, fontweight='bold')
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.savefig('grafik_pid_lqr_wind_turbulence.png', dpi=300)
    print("Sukses! Grafik 4x3 (dengan angin multi-axis) disimpan ke grafik_pid_lqr_wind_turbulence.png")

if __name__ == '__main__':
    simulate()
