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

def simulate_hinf_tuning(params, limits, Q_z, R_z, Q_yaw, R_yaw, gamma_val):
    solver = PIDHinfSolver(params)
    dt = 0.01; time = np.arange(0, 10.0, dt)
    n_steps = len(time)
    
    # We only tune Z and Yaw here
    Az = np.array([[0, 1], [0, 0]])
    Bz = np.array([[0], [1/params['mass']]])
    Cz = np.array([[1, 0]])
    
    Ayaw = np.array([[0, 1], [0, 0]])
    Byaw = np.array([[0], [1/params['iz']]])
    Cyaw = np.array([[1, 0]])
    
    try:
        Kp_z, Ki_z, Kd_z = solver.solve_pid_hinf(Az, Bz, Cz, Q_z, R_z, gamma_val)
        Kp_yaw, Ki_yaw, Kd_yaw = solver.solve_pid_hinf(Ayaw, Byaw, Cyaw, Q_yaw, R_yaw, gamma_val)
    except:
        return 999, 999, 999, 999 # Failed Riccati
        
    pid_z = PID(Kp_z[0,0], Ki_z[0,0], Kd_z[0,0], dt, limits['thrust_min'], limits['thrust_max'])
    pid_yaw = PID(Kp_yaw[0,0], Ki_yaw[0,0], Kd_yaw[0,0], dt, -limits['tau_y_max'], limits['tau_y_max'])
    
    z, vz = 1.0, 0.0
    yaw, r = 0.0, 0.0
    
    z_hist = np.zeros(n_steps)
    yaw_hist = np.zeros(n_steps)
    
    w_n_sq = 2.25
    two_zeta_wn = 3.0
    filt_z, filt_yaw = [1.0, 0.0], [0.0, 0.0]
    
    for k in range(1, n_steps):
        filt_z[1] += (w_n_sq * (2.0 - filt_z[0]) - two_zeta_wn * filt_z[1]) * dt
        filt_z[0] += filt_z[1] * dt
        
        filt_yaw[1] += (w_n_sq * (0.5 - filt_yaw[0]) - two_zeta_wn * filt_yaw[1]) * dt
        filt_yaw[0] += filt_yaw[1] * dt
        
        err_z = filt_z[0] - z
        T_pert = pid_z.compute(err_z)
        vz += (T_pert / params['mass']) * dt
        z += vz * dt
        
        err_yaw = filt_yaw[0] - yaw
        tau_z = pid_yaw.compute(err_yaw)
        r += (tau_z / params['iz']) * dt
        yaw += r * dt
        
        z_hist[k] = z
        yaw_hist[k] = yaw
        
    os_z, tr_z, ts_z = calc_transient(time, z_hist, 2.0, 1.0)
    os_yaw, tr_yaw, ts_yaw = calc_transient(time, yaw_hist, 0.5, 0.0)
    
    return os_z, ts_z, os_yaw, ts_yaw

if __name__ == '__main__':
    config_path = os.path.join(os.path.dirname(__file__), '../config/quadrotor_params.yaml')
    with open(config_path, 'r') as f:
        yaml_data = yaml.safe_load(f)
        params = yaml_data['physics']
        limits = yaml_data['actuator_limits']
        
    print("Mulai Grid Search H-Infinity untuk Z dan Yaw...")
    
    best_z_score = 9999
    best_z_params = None
    
    # Sweep Z
    for q_pos in [1, 5, 10]:
        for r_val in [1, 10, 100, 500]:
            Q = np.diag([q_pos, 1])
            os_z, ts_z, _, _ = simulate_hinf_tuning(params, limits, Q, r_val, np.diag([1,1]), 1, 10.0)
            if os_z < 5.0 and ts_z < 9.0:
                score = ts_z + os_z
                if score < best_z_score:
                    best_z_score = score
                    best_z_params = (q_pos, r_val, os_z, ts_z)
                    
    print(f"Best Z -> Q_pos: {best_z_params[0]}, R: {best_z_params[1]} | OS: {best_z_params[2]:.2f}%, Ts: {best_z_params[3]:.2f}s")
    
    best_yaw_score = 9999
    best_yaw_params = None
    
    # Sweep Yaw
    for q_pos in [0.1, 1, 5]:
        for r_val in [1, 10, 100, 1000]:
            Q = np.diag([q_pos, 0.1])
            _, _, os_yaw, ts_yaw = simulate_hinf_tuning(params, limits, np.diag([1,1]), 1, Q, r_val, 10.0)
            if os_yaw < 5.0 and ts_yaw < 9.0:
                score = ts_yaw + os_yaw
                if score < best_yaw_score:
                    best_yaw_score = score
                    best_yaw_params = (q_pos, r_val, os_yaw, ts_yaw)
                    
    print(f"Best YAW -> Q_pos: {best_yaw_params[0]}, R: {best_yaw_params[1]} | OS: {best_yaw_params[2]:.2f}%, Ts: {best_yaw_params[3]:.2f}s")
