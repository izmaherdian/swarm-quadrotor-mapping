"""
================================================================================
LINEAR QUADRATIC REGULATOR (LQR) PID GAIN SYNTHESIZER
================================================================================
Deskripsi:
Sintesis gain PID optimal berbasis Algebraic Riccati Equation (ARE) untuk
sistem kendali kaskade quadrotor.

Formulasi State-Space Teraugmentasi:
    dot(x_aug) = A_aug * x_aug + B_aug * u_cmd
    
Persamaan Riccati Kontinu (ARE):
    A_aug^T * P + P * A_aug + Q_aug - P * B_aug * R^-1 * B_aug^T * P = 0

Matriks Gain Umpan Balik:
    K = R^-1 * B_aug^T * P
    u_cmd = -K * x_aug = [K_p, K_d, K_i] * error_states
================================================================================
"""

import numpy as np
import scipy.linalg
from typing import Dict, Tuple, Any

class PIDLQRSolver:
    """Solver analitik gain PID berbasis Linear Quadratic Regulator (ARE)."""

    def __init__(self, params: Dict[str, float]):
        """Inisialisasi parameter fisik quadrotor."""
        self.m: float = float(params['mass'])   # [kg]
        self.Ix: float = float(params['ix'])    # [kg.m^2]
        self.Iy: float = float(params['iy'])    # [kg.m^2]
        self.Iz: float = float(params['iz'])    # [kg.m^2]
        self.g: float = float(params['g'])      # [m/s^2]

    def lqr(self, A: np.ndarray, B: np.ndarray, Q: np.ndarray, R: np.ndarray) -> np.ndarray:
        """
        Menghitung matriks gain LQR menggunakan Algebraic Riccati Equation (ARE).
        
        Args:
            A: Matriks dinamika sistem state-space (n x n)
            B: Matriks input aktuasi (n x m)
            Q: Matriks pembobot penalti state (n x n)
            R: Matriks pembobot penalti input kendali (m x m)
            
        Returns:
            Matriks feedback gain K (m x n)
        """
        P = scipy.linalg.solve_continuous_are(A, B, Q, R)
        K = np.linalg.inv(R) @ (B.T @ P)
        return K

    def solve_pid_lqr(self, A: np.ndarray, B: np.ndarray, C: np.ndarray, Q_val: np.ndarray, R_val: float) -> Tuple[float, float, float]:
        """
        Memetakan state-feedback gain LQR teraugmentasi menjadi parameter gain PID (Kp, Ki, Kd).
        """
        n = A.shape[0]
        m = B.shape[1]
        
        # Augmented system (menambahkan state integrator)
        A_aug = np.block([
            [A, B], 
            [np.zeros((m, n)), np.zeros((m, m))]
        ])
        B_aug = np.block([
            [np.zeros((n, m))], 
            [np.eye(m)]
        ])
        
        Q_aug = scipy.linalg.block_diag(Q_val, 0.1 * np.eye(m))
        R_aug = R_val * np.eye(m)
        
        # Hitung gain Ka
        Ka = self.lqr(A_aug, B_aug, Q_aug, R_aug)
        
        # Pemetaan (Mapping) ke arsitektur PID
        Gamma = np.block([
            [C.T, (C @ A).T, (C @ (A @ A)).T],
            [np.zeros((m, m)), (C @ B).T, (C @ A @ B).T]
        ])
        
        # Menggunakan pseudoinverse (pinv) sama seperti di MATLAB
        K_hat = np.linalg.pinv(Gamma) @ Ka.T
        
        k1_hat = K_hat[0:m, :].T
        k2_hat = K_hat[m:2*m, :].T
        k3_hat = K_hat[2*m:3*m, :].T
        
        I = np.eye(m)
        Kd = k3_hat @ np.linalg.inv(I + C @ B @ k3_hat)
        c = I - Kd @ C @ B
        Ki = c @ k1_hat
        Kp = c @ k2_hat
        
        return Kp, Ki, Kd

    def compute_all_gains(self):
        """
        Menghitung seluruh parameter PID untuk 4 subsistem (Lateral, Longitudinal, Altitude, Yaw)
        sesuai dengan skrip quadrotor_PID_LQR_decentralized.m
        """
        g, m, Ix, Iy, Iz = self.g, self.m, self.Ix, self.Iy, self.Iz
        gains = {}

        # ==========================================
        # 1. Subsistem X (Longitudinal -> Pitch)
        # ==========================================
        # Outer Loop: X -> Theta_ref (Bandwidth optimal untuk eliminasi limit-cycle oscillation)
        A_x_out = np.array([[0, 1], [0, 0]])
        B_x_out = np.array([[0], [g]])
        C_x_out = np.array([[1, 0]])
        Q_x_out = np.diag([0.08, 0.35])
        R_x_out = np.array([[1.0]])
        Kp, Ki, Kd = self.solve_pid_lqr(A_x_out, B_x_out, C_x_out, Q_x_out, R_x_out)
        gains['x_outer'] = {'Kp': Kp[0,0], 'Ki': Ki[0,0], 'Kd': Kd[0,0]}

        # Inner Loop: Theta -> tau_y (Bandwidth tinggi untuk tracking sikap presisi dan redaman gyro kuat)
        A_x_in = np.array([[0, 1], [0, 0]])
        B_x_in = np.array([[0], [1/Iy]])
        C_x_in = np.array([[1, 0]])
        Q_x_in = np.diag([6.0, 0.60])
        R_x_in = np.array([[0.05]])
        Kp, Ki, Kd = self.solve_pid_lqr(A_x_in, B_x_in, C_x_in, Q_x_in, R_x_in)
        gains['x_inner'] = {'Kp': Kp[0,0], 'Ki': Ki[0,0], 'Kd': Kd[0,0]}

        # ==========================================
        # 2. Subsistem Y (Lateral -> Roll)
        # ==========================================
        # Outer Loop: Y -> Phi_ref
        A_y_out = np.array([[0, 1], [0, 0]])
        B_y_out = np.array([[0], [-g]])
        C_y_out = np.array([[1, 0]])
        Q_y_out = np.diag([0.08, 0.35])
        R_y_out = np.array([[1.0]])
        Kp, Ki, Kd = self.solve_pid_lqr(A_y_out, B_y_out, C_y_out, Q_y_out, R_y_out)
        gains['y_outer'] = {'Kp': Kp[0,0], 'Ki': Ki[0,0], 'Kd': Kd[0,0]}

        # Inner Loop: Phi -> tau_x
        A_y_in = np.array([[0, 1], [0, 0]])
        B_y_in = np.array([[0], [1/Ix]])
        C_y_in = np.array([[1, 0]])
        Q_y_in = np.diag([6.0, 0.60])
        R_y_in = np.array([[0.05]])
        Kp, Ki, Kd = self.solve_pid_lqr(A_y_in, B_y_in, C_y_in, Q_y_in, R_y_in)
        gains['y_inner'] = {'Kp': Kp[0,0], 'Ki': Ki[0,0], 'Kd': Kd[0,0]}


        # ==========================================
        # 3. Subsistem Z (Altitude)
        # ==========================================
        Az = np.array([[0, 1], [0, 0]])
        Bz = np.array([[0], [1/m]])
        Cz = np.array([[1, 0]])
        Q_z = np.diag([1, 1000])     # Q_damping = 1000 untuk peredaman z mulus
        R_z = np.array([[1]])
        Kp, Ki, Kd = self.solve_pid_lqr(Az, Bz, Cz, Q_z, R_z)
        gains['z'] = {'Kp': Kp[0,0], 'Ki': Ki[0,0], 'Kd': Kd[0,0]}

        # ==========================================
        # 4. SUBSISTEM YAW 
        # ==========================================
        Ayaw = np.array([[0, 1], [0, 0]])
        Byaw = np.array([[0], [1/Iz]])
        Cyaw = np.array([[1, 0]])
        Q_yaw = np.diag([1.0, 4.0])
        R_yaw = np.array([[4.0]])
        Kp, Ki, Kd = self.solve_pid_lqr(Ayaw, Byaw, Cyaw, Q_yaw, R_yaw)
        gains['yaw'] = {'Kp': Kp[0,0], 'Ki': Ki[0,0], 'Kd': Kd[0,0]}

        return gains

if __name__ == '__main__':
    import yaml
    import os
    config_path = os.path.join(os.path.dirname(__file__), '../config/quadrotor_params.yaml')
    with open(config_path, 'r') as f:
        params = yaml.safe_load(f)['physics']
        
    solver = PIDLQRSolver(params)
    hasil_gains = solver.compute_all_gains()
    print("===== HASIL TRANSLASI PID-LQR (MATLAB -> PYTHON) =====")
    for subsistem, k_vals in hasil_gains.items():
        print(f"Subsistem [{subsistem.upper()}]:")
        print(f"  Kp = {k_vals['Kp']:.4f}")
        print(f"  Ki = {k_vals['Ki']:.4f}")
        print(f"  Kd = {k_vals['Kd']:.4f}\n")
