import numpy as np
from scipy.linalg import solve_continuous_are

class PIDHinfSolver:
    def __init__(self, physics_params):
        # Ekstrak parameter fisika
        self.mass = physics_params['mass']
        self.g = physics_params['g']
        self.ix = physics_params['ix']
        self.iy = physics_params['iy']
        self.iz = physics_params['iz']
        
        # Mapping parameter lama (untuk konsistensi dengan skrip Matlab)
        self.m = self.mass
        self.Ix = self.ix
        self.Iy = self.iy
        self.Iz = self.iz

    def solve_pid_hinf(self, A, B, C, Q_val, R_val, gamma):
        """
        Menyelesaikan matriks gain PID menggunakan H-infinity Algebraic Riccati Equation (HARE).
        """
        n = A.shape[0]
        m = B.shape[1]
        
        # 1. AUGMENTASI SISTEM
        A_aug = np.block([
            [A, B],
            [np.zeros((m, n)), np.zeros((m, m))]
        ])
        Bu_aug = np.block([
            [np.zeros((n, m))],
            [np.eye(m)]
        ])
        Bw_aug = np.block([
            [B],
            [np.zeros((m, m))]
        ])
        
        # 2. DESAIN H-INFINITY KONTROL OPTIMAL-ROBAS (HARE)
        Q_aug = np.block([
            [Q_val, np.zeros((n, m))],
            [np.zeros((m, n)), 0.1 * np.eye(m)]
        ])
        
        B_tilde = np.block([Bu_aug, Bw_aug])
        R_tilde = np.block([
            [R_val * np.eye(m), np.zeros((m, m))],
            [np.zeros((m, m)), -gamma**2 * np.eye(m)]
        ])
        
        # Solve CARE untuk H-infinity
        P_inf = solve_continuous_are(A_aug, B_tilde, Q_aug, R_tilde)
        
        # Mengekstrak Gain khusus untuk jalur kontrol u
        Ka = (1.0 / R_val) * Bu_aug.T @ P_inf 
        
        # 3. MATRIKS TRANSFORMASI GAMMA (\Gamma)
        Gamma_mat = np.block([
            [C.T,             (C @ A).T,        (C @ A @ A).T],
            [np.zeros((m, m)), (C @ B).T,        (C @ A @ B).T]
        ])
        
        # 4. EKSTRAKSI PARAMETER PID TERNORMALISASI (\hat{K})
        K_hat = np.linalg.pinv(Gamma_mat) @ Ka.T
        
        k1_hat_col = K_hat[0:m, :]
        k2_hat_col = K_hat[m:2*m, :]
        k3_hat_col = K_hat[2*m:3*m, :]
        
        k1_hat = k1_hat_col.T
        k2_hat = k2_hat_col.T
        k3_hat = k3_hat_col.T
        
        # 5. DENORMALISASI KE PID ASLI (k1, k2, k3) -> Kp, Ki, Kd
        I = np.eye(m)
        Kd = k3_hat @ np.linalg.inv(I + C @ B @ k3_hat)
        c = I - Kd @ C @ B
        Ki = c @ k1_hat
        Kp = c @ k2_hat
        
        return Kp, Ki, Kd

    def get_all_gains(self, gamma_out=15.0, gamma_in=10.0):
        gains = {}
        g = self.g; m = self.m; Ix = self.Ix; Iy = self.Iy; Iz = self.Iz
        
        # ==========================================
        # 1. Subsistem X (Longitudinal -> Pitch)
        # ==========================================
        # Outer Loop: X -> Theta_ref
        A_x_out = np.array([[0, 1], [0, 0]])
        B_x_out = np.array([[0], [g]])
        C_x_out = np.array([[1, 0]])
        Q_x_out = np.diag([1, 5.0])    # UPDATE TUNER: Q = 5.0
        R_x_out = 50.0                 # UPDATE TUNER: R = 50
        Kp, Ki, Kd = self.solve_pid_hinf(A_x_out, B_x_out, C_x_out, Q_x_out, R_x_out, 50.0)
        gains['x_outer'] = {'Kp': Kp[0,0], 'Ki': Ki[0,0], 'Kd': Kd[0,0]}

        # Inner Loop: Theta -> tau_y
        A_x_in = np.array([[0, 1], [0, 0]])
        B_x_in = np.array([[0], [1/Iy]])
        C_x_in = np.array([[1, 0]])
        Q_x_in = np.diag([1, 0.5])      # Relaxed from [10, 1]
        R_x_in = 1.0                    # Relaxed from 0.01 to penalize torque
        Kp, Ki, Kd = self.solve_pid_hinf(A_x_in, B_x_in, C_x_in, Q_x_in, R_x_in, 50.0)
        gains['x_inner'] = {'Kp': Kp[0,0], 'Ki': Ki[0,0], 'Kd': Kd[0,0]}

        # ==========================================
        # 2. Subsistem Y (Lateral -> Roll)
        # ==========================================
        # Outer Loop: Y -> Phi_ref
        A_y_out = np.array([[0, 1], [0, 0]])
        B_y_out = np.array([[0], [-g]])
        C_y_out = np.array([[1, 0]])
        Q_y_out = np.diag([1, 5.0])    # UPDATE TUNER: Q = 5.0
        R_y_out = 50.0                 # UPDATE TUNER: R = 50
        Kp, Ki, Kd = self.solve_pid_hinf(A_y_out, B_y_out, C_y_out, Q_y_out, R_y_out, 50.0)
        gains['y_outer'] = {'Kp': Kp[0,0], 'Ki': Ki[0,0], 'Kd': Kd[0,0]}

        # Inner Loop: Phi -> tau_x
        A_y_in = np.array([[0, 1], [0, 0]])
        B_y_in = np.array([[0], [1/Ix]])
        C_y_in = np.array([[1, 0]])
        Q_y_in = np.diag([1, 0.5])      # Relaxed from [10, 1]
        R_y_in = 1.0                    # Relaxed from 0.01 to penalize torque
        Kp, Ki, Kd = self.solve_pid_hinf(A_y_in, B_y_in, C_y_in, Q_y_in, R_y_in, 50.0)
        gains['y_inner'] = {'Kp': Kp[0,0], 'Ki': Ki[0,0], 'Kd': Kd[0,0]}

        # ==========================================
        # 3. Subsistem Z (Altitude)
        # ==========================================
        Az = np.array([[0, 1], [0, 0]])
        Bz = np.array([[0], [1/m]])
        Cz = np.array([[1, 0]])
        Kp, Ki, Kd = self.solve_pid_hinf(Az, Bz, Cz, np.diag([1, 5.0]), 0.5, 50.0)
        gains['z'] = {'Kp': Kp[0,0], 'Ki': Ki[0,0], 'Kd': Kd[0,0]}

        # ==========================================
        # 4. Subsistem Yaw
        # ==========================================
        Ayaw = np.array([[0, 1], [0, 0]])
        Byaw = np.array([[0], [1/Iz]])
        Cyaw = np.array([[1, 0]])
        # Hasil Tuning: Q_pos = 5, Q_vel = 0.1, R = 1.0
        Kp, Ki, Kd = self.solve_pid_hinf(Ayaw, Byaw, Cyaw, np.diag([5, 0.1]), 1.0, gamma_in)
        gains['yaw'] = {'Kp': Kp[0,0], 'Ki': Ki[0,0], 'Kd': Kd[0,0]}

        return gains

if __name__ == '__main__':
    # Test jalankan solver
    physics = {'mass': 1.0, 'g': 9.81, 'ix': 8.1e-3, 'iy': 8.1e-3, 'iz': 14.2e-3}
    solver = PIDHinfSolver(physics)
    gains = solver.get_all_gains()
    for k, v in gains.items():
        print(f"{k.upper()} -> Kp: {v['Kp']:.4f}, Ki: {v['Ki']:.4f}, Kd: {v['Kd']:.4f}")
