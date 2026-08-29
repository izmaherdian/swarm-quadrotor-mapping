"""Identifikasi plant loop-tertutup: dari gain PID-LQR ke model kecepatan orde-1.

Loop luar low-level (lihat pid_lqr_node.py) menghitung

    theta_ref = Kp*e_x + i_term - Kd*vx + k_ff*vx_cmd,      a_x ~ g*theta_ref

Dengan aturan referensi `x_ref = x + T_lead * v_cmd` maka e_x = T_lead*v_cmd, sehingga

    a = (g*Kp*T_lead + g*k_ff) * v_cmd  -  (g*Kd) * v

Pilih T_lead agar penguatan DC dari v_cmd ke v tepat 1:

    T_lead = (Kd - k_ff) / Kp,      k_v = g*Kd,      a_max = g*tan(angle_max)

Hasilnya plant kecepatan yang dikomandokan:

    v_dot = sat_{±a_max}( k_v * (v_cmd - v) )

Jarak berhenti eksak dari laju s (integrasi v_dot = -min(k_v*v, a_max)):

    D(s) = (s^2 + v_c^2) / (2*a_max)   untuk s > v_c,   v_c = a_max / k_v
    D(s) = s / k_v                     untuk s <= v_c

Angka-angka inilah yang dipakai barrier.py, sehingga tidak ada konstanta ajaib:
semuanya turunan dari quadrotor_params.yaml + gain LQR.
"""
import math
import os

import yaml

# Feedforward sikap di pid_lqr_node/pid_hinf_node: theta_ref += k_ff * vx_body
DEFAULT_K_FF = 0.15


class PlantModel:
    def __init__(self, k_v, a_max, T_lead, k_ff=DEFAULT_K_FF):
        if k_v <= 0.0:
            raise ValueError(f'k_v harus positif, dapat {k_v}')
        if a_max <= 0.0:
            raise ValueError(f'a_max harus positif, dapat {a_max}')
        self.k_v = float(k_v)
        self.a_max = float(a_max)
        self.T_lead = float(T_lead)
        self.k_ff = float(k_ff)

    @property
    def v_c(self):
        """Laju di mana saturasi percepatan berhenti mengikat."""
        return self.a_max / self.k_v

    def stopping_distance(self, s):
        """Jarak berhenti eksak dari laju s di bawah v_dot = -min(k_v*v, a_max)."""
        s = max(0.0, float(s))
        if s <= self.v_c:
            return s / self.k_v
        return (s * s + self.v_c ** 2) / (2.0 * self.a_max)

    def step(self, v, v_cmd, dt):
        """Integrasi satu langkah plant. Dipakai fast-sim dan uji invariance."""
        import numpy as np
        v = np.asarray(v, dtype=float)
        v_cmd = np.asarray(v_cmd, dtype=float)
        a = self.k_v * (v_cmd - v)
        mag = float(np.linalg.norm(a))
        if mag > self.a_max and mag > 1e-12:
            a = a * (self.a_max / mag)
        return v + a * dt

    @classmethod
    def from_gains(cls, gains, angle_max, g=9.81, k_ff=DEFAULT_K_FF):
        """Turunkan dari dict gain LQR/H-inf (butuh kunci 'x_outer')."""
        outer = gains['x_outer']
        Kp, Kd = float(outer['Kp']), float(outer['Kd'])
        T_lead = (Kd - k_ff) / Kp
        return cls(k_v=g * Kd, a_max=g * math.tan(angle_max), T_lead=T_lead, k_ff=k_ff)

    @staticmethod
    def find_quad_yaml():
        """Cari quadrotor_params.yaml baik dari source tree maupun install.

        Modul ini dipakai dari dua tempat — langsung dari src/ saat pengujian
        cepat, dan dari install/ saat dijalankan ROS — sehingga jalur relatif
        tunggal akan selalu salah di salah satunya.
        """
        env = os.environ.get('QUADROTOR_PARAMS')
        if env and os.path.isfile(env):
            return env

        candidates = []
        try:
            from ament_index_python.packages import get_package_share_directory
            candidates.append(os.path.join(
                get_package_share_directory('swarm_low_level'),
                'config', 'quadrotor_params.yaml'))
        except Exception:
            pass

        # Telusuri ke atas dari lokasi modul sampai menemukan workspace.
        here = os.path.dirname(os.path.abspath(__file__))
        node = here
        for _ in range(8):
            candidates.append(os.path.join(
                node, 'src', 'swarm_low_level', 'config', 'quadrotor_params.yaml'))
            candidates.append(os.path.join(
                node, 'swarm_low_level', 'config', 'quadrotor_params.yaml'))
            parent = os.path.dirname(node)
            if parent == node:
                break
            node = parent

        for path in candidates:
            if os.path.isfile(path):
                return path

        raise FileNotFoundError(
            'quadrotor_params.yaml tidak ditemukan. Set QUADROTOR_PARAMS atau '
            'berikan quad_yaml_path secara eksplisit. Sudah dicoba:\n  '
            + '\n  '.join(candidates))

    @classmethod
    def from_config(cls, quad_yaml_path=None, solver='lqr', k_ff=DEFAULT_K_FF):
        """Muat quadrotor_params.yaml lalu selesaikan gain-nya.

        Nilai yang dipakai barrier diturunkan, bukan ditulis tangan.
        """
        if quad_yaml_path is None:
            quad_yaml_path = cls.find_quad_yaml()

        with open(quad_yaml_path, 'r') as f:
            cfg = yaml.safe_load(f)

        physics = cfg['physics']
        angle_max = float(cfg['actuator_limits']['angle_max'])
        g = float(physics.get('g', 9.81))

        if solver == 'hinf':
            from swarm_low_level.solver_pid_hinf import PIDHinfSolver
            gains = PIDHinfSolver(physics).get_all_gains()
        else:
            from swarm_low_level.solver_pid_lqr import PIDLQRSolver
            gains = PIDLQRSolver(physics).compute_all_gains()

        return cls.from_gains(gains, angle_max, g=g, k_ff=k_ff)

    def __repr__(self):
        return (f'PlantModel(k_v={self.k_v:.4f}/s, a_max={self.a_max:.4f} m/s^2, '
                f'T_lead={self.T_lead:.4f} s, v_c={self.v_c:.4f} m/s)')
