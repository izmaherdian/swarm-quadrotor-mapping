import numpy as np
import yaml
import os
from solver_pid_hinf import PIDHinfSolver

class PID:
    def __init__(self, Kp, Ki, Kd, dt, out_min=-np.inf, out_max=np.inf):
        self.Kp = Kp; self.Ki = Ki; self.Kd = Kd; self.dt = dt
        self.out_min = out_min; self.out_max = out_max
        self.integral = 0.0; self.prev_error = 0.0

    def compute(self, error):
        proportional = self.Kp * error
        derivative = self.Kd * (error - self.prev_error) / self.dt
        output_no_i = proportional + derivative
        if not ((output_no_i > self.out_max and error > 0) or (output_no_i < self.out_min and error < 0)):
            self.integral += error * self.dt
        output = proportional + self.Ki * self.integral + derivative
        self.prev_error = error
        return np.clip(output, self.out_min, self.out_max)

def calc_transient(time, response, target, start_val=0.0):
    steady_state = target
    peak_val = np.max(np.abs(response)) if target > 0 else np.min(response)
    if steady_state == 0:
        overshoot = 0
    else:
        overshoot = max(0, (np.abs(peak_val) - np.abs(steady_state)) / np.abs(steady_state)) * 100

    rise_time = 0.0
    settling_time = time[-1]
    
    if steady_state != 0:
        for i, t in enumerate(time):
            if np.abs(response[i]) >= 0.9 * np.abs(steady_state):
                rise_time = t
                break
        for i in range(len(time)-1, -1, -1):
            if np.abs(response[i] - steady_state) > 0.05 * np.abs(steady_state):
                settling_time = time[min(i+1, len(time)-1)]
                break
    else:
        for i in range(len(time)-1, -1, -1):
            if np.abs(response[i]) > 0.05:
                settling_time = time[min(i+1, len(time)-1)]
                break
                
    return overshoot, rise_time, settling_time

def simulate_tuning(params_full, Q_xy, R_xy, Q_z, R_z, gamma_val):
    solver = PIDHinfSolver(params_full['physics'])
    dt = 0.01; time = np.arange(0, 15.0, dt)
    n_steps = len(time)
    
    g, m = solver.g, solver.m
    Iy = solver.Iy
    limits = params_full['actuator_limits']
    
    act_phys = params_full['actuator_physics']
    kf, km, tau_m = act_phys['kf'], act_phys['km'], act_phys['tau_m']
    w_max, w_min = act_phys['omega_max'], act_phys['omega_min']
    d = params_full['physics']['arm_length'] * 0.707106781
    
    w_hover = np.sqrt((m * g / 4.0) / kf)
    w_actual = np.array([w_hover, w_hover, w_hover, w_hover])
    
    M = np.array([
        [kf, kf, kf, kf],
        [-kf*d, kf*d, kf*d, -kf*d],
        [-kf*d, kf*d, -kf*d, kf*d],
        [-km, -km, km, km]
    ])
    M_inv = np.linalg.inv(M)
    
    # Setup H-Infinity
    A_x_out = np.array([[0, 1], [0, 0]]); B_x_out = np.array([[0], [g]]); C_x_out = np.array([[1, 0]])
    A_x_in = np.array([[0, 1], [0, 0]]); B_x_in = np.array([[0], [1/Iy]]); C_x_in = np.array([[1, 0]])
    Az = np.array([[0, 1], [0, 0]]); Bz = np.array([[0], [1/m]]); Cz = np.array([[1, 0]])
    
    try:
        Kp_xo, Ki_xo, Kd_xo = solver.solve_pid_hinf(A_x_out, B_x_out, C_x_out, np.diag([1, Q_xy]), R_xy, gamma_val)
        Kp_xi, Ki_xi, Kd_xi = solver.solve_pid_hinf(A_x_in, B_x_in, C_x_in, np.diag([10, 5]), 0.05, gamma_val)
        Kp_z, Ki_z, Kd_z = solver.solve_pid_hinf(Az, Bz, Cz, np.diag([1, Q_z]), R_z, gamma_val)
    except Exception as e:
        print(f"Riccati Failed: {e}")
        return 999, 999, 999, 999
        
    pid_x_out = PID(Kp_xo[0,0], Ki_xo[0,0], Kd_xo[0,0], dt, -limits['angle_max'], limits['angle_max'])
    pid_x_in = PID(Kp_xi[0,0], Ki_xi[0,0], Kd_xi[0,0], dt, -limits['tau_rp_max'], limits['tau_rp_max'])
    pid_z = PID(Kp_z[0,0], Ki_z[0,0], Kd_z[0,0], dt, -limits['thrust_max'], limits['thrust_max'])
    
    x, vx, theta, q = 1.0, 0.0, 0.0, 0.0
    z, vz = 1.0, 0.0
    
    x_hist = np.zeros(n_steps)
    z_hist = np.zeros(n_steps)
    
    w_n_sq = 2.25
    two_zeta_wn = 3.0
    filt_x, filt_z = [1.0, 0.0], [1.0, 0.0]
    
    for k in range(n_steps):
        x_hist[k] = x; z_hist[k] = z
        
        filt_x[1] += (w_n_sq * (2.0 - filt_x[0]) - two_zeta_wn * filt_x[1]) * dt
        filt_x[0] += filt_x[1] * dt
        filt_z[1] += (w_n_sq * (2.0 - filt_z[0]) - two_zeta_wn * filt_z[1]) * dt
        filt_z[0] += filt_z[1] * dt
        
        theta_ref = pid_x_out.compute(filt_x[0] - x)
        tau_y_cmd = pid_x_in.compute(theta_ref - theta)
        uz_pid = pid_z.compute(filt_z[0] - z)
        
        # MIXER
        U_cmd = np.array([uz_pid + (m * g), 0, tau_y_cmd, 0])
        w_sq_cmd = M_inv @ U_cmd
        w_cmd = np.sqrt(np.maximum(w_sq_cmd, 0))
        w_cmd = np.clip(w_cmd, w_min, w_max)
        w_actual = w_actual + ((w_cmd - w_actual) / tau_m) * dt
        
        U_actual = M @ (w_actual**2)
        T_pert = U_actual[0] - (m * g)
        tau_y = U_actual[2]
        
        # Fisika
        x += vx * dt; vx += (g * theta) * dt
        theta += q * dt; q += (tau_y / Iy) * dt
        z += vz * dt; vz += (T_pert / m) * dt
        
    os_x, _, ts_x = calc_transient(time, x_hist, 2.0, 1.0)
    os_z, _, ts_z = calc_transient(time, z_hist, 2.0, 1.0)
    return os_x, ts_x, os_z, ts_z

def run_tuner():
    print("[H-INFINITY TUNER (MIXER)] Mencari kombinasi parameter terbaik (Fokus Overshoot Minimum, Ts Kendur)...")
    config_path = os.path.join(os.path.dirname(__file__), '../config/quadrotor_params.yaml')
    with open(config_path, 'r') as f: params_full = yaml.safe_load(f)
    
    print("\n[1] Grid Search X/Y...")
    best_os_x, best_ts_x, best_rx, best_qx = 999, 999, None, None
    for r in [10, 50, 100, 200, 500]:
        for q in [0.01, 0.1, 0.5, 1.0, 5.0]:
            os_x, ts_x, _, _ = simulate_tuning(params_full, q, r, 1.0, 1.0, 50.0)
            print(f"  Test R={r:4d}, Q={q:4.2f} -> OS_X={os_x:5.1f}%, Ts_X={ts_x:4.2f}s")
            
            if ts_x <= 10.0:
                if os_x < best_os_x or (abs(os_x - best_os_x) < 0.1 and ts_x < best_ts_x):
                    best_os_x, best_ts_x = os_x, ts_x
                    best_rx, best_qx = r, q
                    
    print(f"  >>> H-INF X/Y TERBAIK: R = {best_rx}, Q = {best_qx} (OS: {best_os_x:.2f}%, Ts: {best_ts_x:.2f}s) <<<")

    print("\n[2] Grid Search Z...")
    best_os_z, best_ts_z, best_rz, best_qz = 999, 999, None, None
    for r in [0.1, 0.5, 1.0, 5.0, 10.0]:
        for q in [0.1, 1.0, 5.0, 10.0, 20.0]:
            _, _, os_z, ts_z = simulate_tuning(params_full, 0.1, 100, q, r, 50.0)
            print(f"  Test R={r:4.2f}, Q={q:4.2f} -> OS_Z={os_z:5.1f}%, Ts_Z={ts_z:4.2f}s")
            
            if ts_z <= 10.0:
                if os_z < best_os_z or (abs(os_z - best_os_z) < 0.1 and ts_z < best_ts_z):
                    best_os_z, best_ts_z = os_z, ts_z
                    best_rz, best_qz = r, q
                    
    print(f"  >>> H-INF Z TERBAIK: R = {best_rz}, Q = {best_qz} (OS: {best_os_z:.2f}%, Ts: {best_ts_z:.2f}s) <<<")

if __name__ == '__main__':
    run_tuner()
