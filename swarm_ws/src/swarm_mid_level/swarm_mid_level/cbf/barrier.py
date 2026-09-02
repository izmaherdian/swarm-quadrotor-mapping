"""
================================================================================
PHYSICALLY-CONSISTENT EXTENDED CLASS-K CONTROL BARRIER FUNCTION
================================================================================
Deskripsi:
Menghitung batas laju mendekati rintangan phi(h) yang konsisten dengan aktuator quadrotor,
memperhitungkan waktu tunda sistem (dead time T_d) dan batas percepatan efektif (a_eff):

    s * T_d + (s^2 + v_c^2) / (2 * a_eff) <= h

Solusi Batas Laju (Class-K):
    Untuk h >= 0:
        phi(h) = max(0, -a_eff * T_d + sqrt(max(0, a_eff^2 * T_d^2 + 2 * a_eff * h - v_c^2)))
    Untuk h < 0 (Extended Class-K Active Repulsion):
        phi(h) = gamma_recov * h  ==> Mewajibkan n^T * u >= gamma_recov * |h|
================================================================================
"""

import numpy as np
from typing import Union

def phi(h: Union[float, np.ndarray], a_eff: float, T_d: float, v_c: float) -> Union[float, np.ndarray]:
    """
    Laju mendekat maksimum yang diizinkan pada clearance h.
    
    Args:
        h: Jarak clearance ke permukaan batas rintangan [m] (positif jika di luar, negatif jika di dalam)
        a_eff: Batas percepatan horizontal efektif quadrotor [m/s^2]
        T_d: Total waktu mati kontrol dan aktuator [s]
        v_c: Kecepatan sudut lintasan melingkar [m/s]
        
    Returns:
        phi(h): Batas laju mendekat Class-K [m/s]
    """
    h_arr = np.asarray(h, dtype=float)
    disc = a_eff * a_eff * T_d * T_d + 2.0 * a_eff * h_arr - v_c * v_c
    val_pos = np.maximum(0.0, -a_eff * T_d + np.sqrt(np.maximum(0.0, disc)))
    
    # Extended Class-K CBF: phi(h) < 0 untuk h < 0 -> mewajibkan laju keluar n^T u >= gamma * |h|
    gamma_recov = 1.8
    val_neg = gamma_recov * h_arr
    res = np.where(h_arr < 0.0, val_neg, val_pos)
    return float(res) if np.isscalar(h) else res


def phi_zero_h(a_eff, T_d, v_c):
    """Clearance terbesar dengan phi masih nol (drone wajib berhenti total)."""
    return max(0.0, (v_c * v_c - a_eff * a_eff * T_d * T_d) / (2.0 * a_eff))


def phi_inverse(s, a_eff, T_d, v_c):
    """Clearance minimum agar laju mendekat s masih diizinkan.

    Ini jarak reaksi: pada laju s, drone harus sudah bereaksi sejauh ini.
    Membalik phi:  s = -a*T_d + sqrt(a^2 T_d^2 + 2ah - v_c^2)
                => h = (s^2 + 2*a*T_d*s + v_c^2) / (2a)
    Untuk s = 0 hasilnya salah satu titik pada interval datar phi = 0;
    pakai phi_zero_h() bila yang dicari ujung bawah interval itu.
    """
    s = np.maximum(0.0, np.asarray(s, dtype=float))
    return (s * s + 2.0 * a_eff * T_d * s + v_c * v_c) / (2.0 * a_eff)


def a_eff_for(kind, cfg, plant, obstacle_accel=0.0, wind_accel=0.0):
    """Anggaran percepatan untuk satu kelas constraint.

    Marjin desain dipakai agar constraint tetap dapat dipenuhi di dalam kotak
    rate; sisa anggaran menutup percepatan rintangan dan gangguan angin.
    """
    from . import types as T

    if kind == T.CLASS_WALL:
        margin = cfg.a_margin_wall
    else:
        margin = cfg.a_margin_static

    a = plant.a_max * margin - abs(obstacle_accel) - abs(wind_accel)
    # Jangan pernah nol/negatif: barrier harus tetap terdefinisi.
    return max(0.15 * plant.a_max, a)
