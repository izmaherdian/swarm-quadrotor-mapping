import numpy as np
import yaml
import os
from solver_pid_lqr import PIDLQRSolver

class PID:
    def __init__(self, Kp, Ki, Kd, dt, out_min=-np.inf, out_max=np.inf):
        self.Kp = Kp; self.Ki = Ki; self.Kd = Kd; self.dt = dt
        self.out_min = out_min; self.out_max = out_max
        self.integral = 0; self.prev_error = 0
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
    if abs(step_size) < 1e-6: return 0.0, 0.0
    y = response - start; y_target = target - start
    max_val = np.max(y)
    overshoot = max(0.0, (max_val - y_target) / y_target * 100.0)
    band = 0.02 * y_target
    outside = np.where(np.abs(y - y_target) > band)[0]
    ts = time[outside[-1]] if len(outside) > 0 else 0.0
    return overshoot, ts

def simulate_QR(params_full, Q_xy_damp, R_xy, Q_z_damp, R_z):
    solver = PIDLQRSolver(params_full['physics'])
    g, m, Iy = solver.g, solver.m, solver.Iy
    limits = params_full['actuator_limits']
    act_phys = params_full['actuator_physics']
    kf, km, tau_m = act_phys['kf'], act_phys['km'], act_phys['tau_m']
    w_max, w_min = act_phys['omega_max'], act_phys['omega_min']
    d = params_full['physics']['arm_length'] * 0.707106781
    
    A_x_out = np.array([[0, 1], [0, 0]]); B_x_out = np.array([[0], [g]]); C_x_out = np.array([[1, 0]])
    Kp, Ki, Kd = solver.solve_pid_lqr(A_x_out, B_x_out, C_x_out, np.diag([1, Q_xy_damp]), np.array([[R_xy]]))
    pid_x_out = PID(Kp[0,0], Ki[0,0], Kd[0,0], 0.01, -limits['angle_max'], limits['angle_max'])
    
    A_x_in = np.array([[0, 1], [0, 0]]); B_x_in = np.array([[0], [1/Iy]]); C_x_in = np.array([[1, 0]])
    Kp, Ki, Kd = solver.solve_pid_lqr(A_x_in, B_x_in, C_x_in, np.diag([10, 5]), np.array([[0.05]]))
    pid_x_in = PID(Kp[0,0], Ki[0,0], Kd[0,0], 0.01, -limits['tau_rp_max'], limits['tau_rp_max'])
    
    Kp, Ki, Kd = solver.solve_pid_lqr(np.array([[0, 1], [0, 0]]), np.array([[0], [1/m]]), np.array([[1, 0]]), np.diag([1, Q_z_damp]), np.array([[R_z]]))
    pid_z = PID(Kp[0,0], Ki[0,0], Kd[0,0], 0.01, -limits['thrust_max'], limits['thrust_max'])

    dt = 0.01; t_end = 15.0; time = np.arange(0, t_end, dt)
    n_steps = len(time)
    
    w_hover = np.sqrt((m * g / 4.0) / kf)
    w_actual = np.array([w_hover, w_hover, w_hover, w_hover])
    
    M = np.array([
        [kf, kf, kf, kf],
        [-kf*d, kf*d, kf*d, -kf*d],
        [-kf*d, kf*d, -kf*d, kf*d],
        [-km, -km, km, km]
    ])
    M_inv = np.linalg.inv(M)
    
    x, vx, theta, q = 1.0, 0.0, 0.0, 0.0
    z, vz = 1.0, 0.0
    filt_x = [1.0, 0.0]; filt_z = [1.0, 0.0]
    w_n_sq = 33.99; two_zeta_wn = 11.66
    
    x_hist = np.zeros(n_steps); z_hist = np.zeros(n_steps)
    
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
        
        x += vx * dt; vx += (g * theta) * dt
        theta += q * dt; q += (tau_y / Iy) * dt
        z += vz * dt; vz += (T_pert / m) * dt
        
    os_x, ts_x = calc_transient(time, x_hist, 2.0, 1.0)
    os_z, ts_z = calc_transient(time, z_hist, 2.0, 1.0)
    return os_x, ts_x, os_z, ts_z

def run_tuner():
    print("[LQR TUNER (MIXER)] Mencari kombinasi parameter terbaik (Fokus Overshoot Minimum, Ts Kendur)...")
    config_path = os.path.join(os.path.dirname(__file__), '../config/quadrotor_params.yaml')
    with open(config_path, 'r') as f: params_full = yaml.safe_load(f)
    
    print("\n[1] Grid Search X/Y...")
    best_os_x, best_ts_x, best_rx, best_qx = 999, 999, None, None
    for r in [50, 100, 300, 500, 800, 1000]:
        for qd in [10, 50, 100, 300, 500, 800]:
            os_x, ts_x, _, _ = simulate_QR(params_full, qd, r, 10, 1)
            print(f"  Test R={r:4d}, Q_damp={qd:3d} -> OS={os_x:5.1f}%, Ts={ts_x:4.2f}s")
            
            # Prioritas 1: Overshoot sekecil mungkin. Prioritas 2: Ts paling cepat (maks 10 detik).
            if ts_x <= 10.0:
                if os_x < best_os_x or (abs(os_x - best_os_x) < 0.1 and ts_x < best_ts_x):
                    best_os_x, best_ts_x = os_x, ts_x
                    best_rx, best_qx = r, qd
                    
    print(f"  >>> LQR X/Y TERBAIK: R = {best_rx}, Q_damping = {best_qx} (OS: {best_os_x:.2f}%, Ts: {best_ts_x:.2f}s) <<<")
        
    print("\n[2] Grid Search Z...")
    best_os_z, best_ts_z, best_rz, best_qz = 999, 999, None, None
    for r in [1, 5, 10, 50, 100]:
        for qd in [10, 50, 100, 300, 500, 800, 1000]:
            _, _, os_z, ts_z = simulate_QR(params_full, 10, 1, qd, r)
            print(f"  Test R={r:4d}, Q_damp={qd:3d} -> OS={os_z:5.1f}%, Ts={ts_z:4.2f}s")
            
            if ts_z <= 10.0:
                if os_z < best_os_z or (abs(os_z - best_os_z) < 0.1 and ts_z < best_ts_z):
                    best_os_z, best_ts_z = os_z, ts_z
                    best_rz, best_qz = r, qd
                    
    print(f"  >>> LQR Z TERBAIK: R = {best_rz}, Q_damping = {best_qz} (OS: {best_os_z:.2f}%, Ts: {best_ts_z:.2f}s) <<<")

if __name__ == '__main__':
    run_tuner()
