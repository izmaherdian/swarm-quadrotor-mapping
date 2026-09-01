"""Fungsi kelas-K yang konsisten dengan aktuator.

Batas laju mendekat phi(h) adalah laju maksimum yang boleh dipakai drone untuk
menutup jarak h dan MASIH bisa berhenti tepat waktu, dengan memperhitungkan
dead time dan batas percepatan nyata:

    s * T_d  +  D(s)  <=  h,        D(s) = (s^2 + v_c^2) / (2a)

Selesaikan untuk s:

    s^2 + 2*a*T_d*s + (v_c^2 - 2*a*h) <= 0
    s <= -a*T_d + sqrt(a^2*T_d^2 + 2*a*h - v_c^2)

sehingga

    phi(h) = max(0, -a*T_d + sqrt(max(0, a^2*T_d^2 + 2*a*h - v_c^2)))

Constraint CBF-nya adalah  -n^T u <= phi(h),  yaitu laju mendekat sepanjang
normal luar dibatasi phi.

Kenapa bukan alpha linear (gamma*h)? Dekat h -> 0, phi turun lebih cepat dari
linear — phi malah nol untuk h <= (v_c^2 - a^2 T_d^2)/(2a) — jadi TIDAK ADA
gamma konstan yang aman di sekitar batas. Bentuk akar ini keharusan, bukan
penyempurnaan.

Sifat yang diandalkan avoidance.py:
  * phi(h) = 0 untuk h <= h0, dan naik monoton — fungsi kelas-K yang sah
  * phi Lipschitz dengan konstanta <= 1/T_d, jadi laju pengetatan constraint
    tidak pernah melebihi a per detik: rekursif feasible terhadap batas rate.
"""
import numpy as np


def phi(h, a_eff, T_d, v_c):
    """Laju mendekat maksimum yang diizinkan pada clearance h. Menerima skalar/array.
    
    Untuk h >= 0: batasan pengereman kuadratik (akar).
    Untuk h < 0: pemulihan aktif h_dot >= gamma_recov * |h| (Extended Class-K CBF).
    """
    h = np.asarray(h, dtype=float)
    disc = a_eff * a_eff * T_d * T_d + 2.0 * a_eff * h - v_c * v_c
    val_pos = np.maximum(0.0, -a_eff * T_d + np.sqrt(np.maximum(0.0, disc)))
    
    # Extended Class-K CBF: phi(h) < 0 untuk h < 0 -> mewajibkan laju keluar n^T u >= gamma * |h|
    gamma_recov = 1.8
    val_neg = gamma_recov * h
    return np.where(h < 0.0, val_neg, val_pos)


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
