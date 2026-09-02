"""
================================================================================
H-INFINITY ALGEBRAIC RICCATI EQUATION (HARE) PID GAIN SYNTHESIZER
================================================================================
Deskripsi:
Sintesis gain PID berbasis H-infinity untuk sistem kendali quadrotor terdesentralisasi
dengan penolakan gangguan (disturbance attenuation bound gamma).

Formulasi State-Space Teraugmentasi:
    dot(x_aug) = A_aug * x_aug + Bu_aug * u_cmd + Bw_aug * w_dist
    
Persamaan Riccati H-Infinity (HARE):
    A_aug^T * P + P * A_aug + Q_aug - P * (Bu_aug * R^-1 * Bu_aug^T - gamma^-2 * Bw_aug * Bw_aug^T) * P = 0

Matriks Gain Umpan Balik:
    K_a = R^-1 * Bu_aug^T * P
    u_cmd = -K_a * x_aug = [K_p, K_d, K_i] * error_states
================================================================================
"""

import numpy as np
from scipy.linalg import solve_continuous_are
from typing import Dict, Tuple, Any

class PIDHinfSolver:
    """Solver analitik gain PID berbasis H-infinity Riccati Equation."""

    def __init__(self, physics_params: Dict[str, float]):
        # Ekstrak parameter fisika quadrotor
        self.mass: float = float(physics_params['mass'])   # [kg]
        self.g: float = float(physics_params['g'])         # [m/s^2]
        self.ix: float = float(physics_params['ix'])       # [kg.m^2]
        self.iy: float = float(physics_params['iy'])       # [kg.m^2]
        self.iz: float = float(physics_params['iz'])       # [kg.m^2]
        
        self.m = self.mass
        self.Ix = self.ix
        self.Iy = self.iy
        self.Iz = self.iz

    def solve_pid_hinf(self, A: np.ndarray, B: np.ndarray, C: np.ndarray, Q_val: np.ndarray, R_val: float, gamma: float) -> Tuple[float, float, float]:
        """
        Menyelesaikan matriks gain PID menggunakan H-infinity Algebraic Riccati Equation (HARE).
        
        Args:
            A: Matriks dinamika sistem state-space (n x n)
            B: Matriks input aktuasi (n x m)
            C: Matriks output pengukuran (p x n)
            Q_val: Matriks pembobot state (n x n)
            R_val: Skalar pembobot energi kendali
            gamma: Parameter atenuasi gangguan H-infinity
            
        Returns:
            Tuple (Kp, Ki, Kd) gain kendali PID
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
        
        # 2.5. HURWITZ STABILITY CHECK (A_aug - Bu_aug @ Ka)
        # H-infinity can produce matrices that stabilize A_tilde but destabilize the real plant.
        eigvals = np.linalg.eigvals(A_aug - Bu_aug @ Ka)
        if np.max(np.real(eigvals)) >= 0:
            raise ValueError("Non-Hurwitz: Plant is unstable in reality!")
            
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

    # gamma per subsistem, ditetapkan 1.3 x gamma_min (batas kelayakan HARE).
    #
    # Nilai LAMA (gamma_out=80, gamma_in=64) membuat suku -(1/gamma^2)BwBw^T
    # 1.000-51.000 kali lebih kecil daripada suku BuR^-1Bu^T, sehingga HARE
    # mendegenerasi menjadi CARE: H-inf menghasilkan gain yang IDENTIK dengan
    # LQR sampai ~2% (yaw Kp 1.1895 vs 1.1657), dan perbandingan kedua
    # kontroler menjadi tanpa makna. Diukur 30 Agu 2026 pada 12 misi Gazebo.
    #
    # gamma_min (bisection): x_out 2.54, x_in 2.54, z 4.61, yaw 9.02.
    # Memilih gamma mendekati gamma_min adalah praktik baku H-inf: di situlah
    # spesifikasi atenuasi gangguan paling ketat yang masih layak.
    GAMMA_OUT = 3.30      # posisi luar X/Y
    GAMMA_IN = 3.31       # sikap dalam roll/pitch
    GAMMA_Z = 6.00        # ketinggian
    GAMMA_YAW = 11.73     # yaw

    def get_all_gains(self, gamma_out=None, gamma_in=None,
                      gamma_z=None, gamma_yaw=None):
        gamma_out = self.GAMMA_OUT if gamma_out is None else gamma_out
        gamma_in = self.GAMMA_IN if gamma_in is None else gamma_in
        gamma_z = self.GAMMA_Z if gamma_z is None else gamma_z
        gamma_yaw = self.GAMMA_YAW if gamma_yaw is None else gamma_yaw
        gains = {}
        
        g = self.g; m = self.m; Ix = self.Ix; Iy = self.Iy; Iz = self.Iz
        
        # ==========================================
        # 1. Subsistem X (Longitudinal -> Pitch)
        # ==========================================
        # Outer Loop: X -> Theta_ref
        A_x_out = np.array([[0, 1], [0, 0]])
        B_x_out = np.array([[0], [g]])
        C_x_out = np.array([[1, 0]])
        Q_x_out = np.diag([0.10, 0.40])
        R_x_out = 1.0
        Kp, Ki, Kd = self.solve_pid_hinf(A_x_out, B_x_out, C_x_out, Q_x_out, R_x_out, gamma_out)
        gains['x_outer'] = {'Kp': Kp[0,0], 'Ki': Ki[0,0], 'Kd': Kd[0,0]}

        # Inner Loop: Theta -> tau_y
        A_x_in = np.array([[0, 1], [0, 0]])
        B_x_in = np.array([[0], [1/Iy]])
        C_x_in = np.array([[1, 0]])
        Q_x_in = np.diag([4.0, 0.40])
        R_x_in = 0.06
        Kp, Ki, Kd = self.solve_pid_hinf(A_x_in, B_x_in, C_x_in, Q_x_in, R_x_in, gamma_in)
        gains['x_inner'] = {'Kp': Kp[0,0], 'Ki': Ki[0,0], 'Kd': Kd[0,0]}

        # ==========================================
        # 2. Subsistem Y (Lateral -> Roll)
        # ==========================================
        # Outer Loop: Y -> Phi_ref
        A_y_out = np.array([[0, 1], [0, 0]])
        B_y_out = np.array([[0], [-g]])
        C_y_out = np.array([[1, 0]])
        Q_y_out = np.diag([0.10, 0.40])
        R_y_out = 1.0
        Kp, Ki, Kd = self.solve_pid_hinf(A_y_out, B_y_out, C_y_out, Q_y_out, R_y_out, gamma_out)
        gains['y_outer'] = {'Kp': Kp[0,0], 'Ki': Ki[0,0], 'Kd': Kd[0,0]}

        # Inner Loop: Phi -> tau_x
        A_y_in = np.array([[0, 1], [0, 0]])
        B_y_in = np.array([[0], [1/Ix]])
        C_y_in = np.array([[1, 0]])
        Q_y_in = np.diag([4.0, 0.40])
        R_y_in = 0.06
        Kp, Ki, Kd = self.solve_pid_hinf(A_y_in, B_y_in, C_y_in, Q_y_in, R_y_in, gamma_in)
        gains['y_inner'] = {'Kp': Kp[0,0], 'Ki': Ki[0,0], 'Kd': Kd[0,0]}


        # ==========================================
        # 3. Subsistem Z (Altitude)
        # ==========================================
        Az = np.array([[0, 1], [0, 0]])
        Bz = np.array([[0], [1/m]])
        Cz = np.array([[1, 0]])
        Q_z = np.diag([1.0, 1000.0])
        R_z = 1.0
        Kp, Ki, Kd = self.solve_pid_hinf(Az, Bz, Cz, Q_z, R_z, gamma_z)
        gains['z'] = {'Kp': Kp[0,0], 'Ki': Ki[0,0], 'Kd': Kd[0,0]}

        # ==========================================
        # 4. Subsistem Yaw
        # ==========================================
        Ayaw = np.array([[0, 1], [0, 0]])
        Byaw = np.array([[0], [1/Iz]])
        Cyaw = np.array([[1, 0]])
        Q_yaw = np.diag([1.0, 4.0])
        R_yaw = 4.0
        Kp, Ki, Kd = self.solve_pid_hinf(Ayaw, Byaw, Cyaw, Q_yaw, R_yaw, gamma_yaw)
        gains['yaw'] = {'Kp': Kp[0,0], 'Ki': Ki[0,0], 'Kd': Kd[0,0]}

        return gains

if __name__ == '__main__':
    # Test jalankan solver dengan parameter 3DR Iris nominal
    physics = {'mass': 1.50, 'g': 9.81, 'ix': 0.0291, 'iy': 0.0291, 'iz': 0.0552}
    solver = PIDHinfSolver(physics)
    gains = solver.get_all_gains()
    for k, v in gains.items():
        print(f"{k.upper()} -> Kp: {v['Kp']:.4f}, Ki: {v['Ki']:.4f}, Kd: {v['Kd']:.4f}")
