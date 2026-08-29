"""Definisi rintangan arena — SATU-SATUNYA sumber kebenaran.

Sebelumnya daftar ini disalin di tiga tempat (coordinator, auto_tune_scheme3,
fast-sim) dengan risiko saling melenceng dari obstacles.world. Semua konsumen
sekarang mengimpor dari sini.

Harus tetap cocok dengan swarm_ws/src/swarm_sim/worlds/obstacles.world.
"""
import math

import numpy as np

# (id, sel_voronoi, x, y) — radius & tinggi seragam
STATIC_OBSTACLES = (
    (101, 2, -1.5, 9.5),
    (102, 3, 4.0, 6.0),
    (103, 3, 6.5, 9.5),
    (104, 4, -8.0, -2.0),
    (105, 4, -5.0, -7.5),
    (106, 4, -10.5, -12.5),
    (107, 5, 6.0, -4.0),
    (108, 7, 0.0, 2.5),
    (109, 7, 2.5, -9.0),
)

STATIC_RADIUS = 0.40
STATIC_HEIGHT = 4.0

# Dua silinder bergerak menyusuri diagonal membentuk pola 'X'.
DYNAMIC_RADIUS = 0.45
DYNAMIC_AMPLITUDE = 10.0
DYNAMIC_OMEGA = (0.15, 0.11)
DYNAMIC_IDS = (201, 202)


def dynamic_state(t):
    """Posisi & kecepatan analitik kedua rintangan dinamis pada waktu t.

    Kembalian: ((p1, v1), (p2, v2)), masing-masing array (2,).

    Obs 1: (-10,10) <-> (10,-10)   Obs 2: (10,10) <-> (-10,-10)
    """
    A = DYNAMIC_AMPLITUDE
    w1, w2 = DYNAMIC_OMEGA

    c1, s1 = math.cos(w1 * t), math.sin(w1 * t)
    p1 = np.array([-A * c1, A * c1])
    v1 = np.array([A * w1 * s1, -A * w1 * s1])

    c2, s2 = math.cos(w2 * t), math.sin(w2 * t)
    p2 = np.array([A * c2, A * c2])
    v2 = np.array([-A * w2 * s2, -A * w2 * s2])

    return (p1, v1), (p2, v2)


def dynamic_accel_bound(index):
    """Percepatan maksimum rintangan dinamis ke-index (0 atau 1)."""
    return DYNAMIC_AMPLITUDE * DYNAMIC_OMEGA[index] ** 2


def min_clearance(pos_xy, t, drone_radius=0.22):
    """Clearance FISIK terkecil dari satu posisi ke seluruh rintangan.

    Tanpa bantalan keamanan — ini jarak permukaan sebenarnya, dipakai untuk
    menilai hasil, bukan untuk mengendalikan.
    """
    p = np.asarray(pos_xy, dtype=float)[:2]
    best = float('inf')
    for _oid, _cell, ox, oy in STATIC_OBSTACLES:
        d = float(np.linalg.norm(p - np.array([ox, oy])))
        best = min(best, d - (STATIC_RADIUS + drone_radius))
    for (q, _v) in dynamic_state(t):
        d = float(np.linalg.norm(p - q))
        best = min(best, d - (DYNAMIC_RADIUS + drone_radius))
    return best
