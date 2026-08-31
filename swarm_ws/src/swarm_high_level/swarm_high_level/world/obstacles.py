"""Definisi rintangan arena — SATU-SATUNYA sumber kebenaran.

Sebelumnya daftar ini disalin di tiga tempat (coordinator, auto_tune_scheme3,
fast-sim) dengan risiko saling melenceng dari obstacles.world. Semua konsumen
sekarang mengimpor dari sini, dan berkas world Skema 3 DIBANGKITKAN dari tabel
ini oleh ``swarm_ws/tools/gen_obstacle_worlds.py`` — jadi keduanya tidak bisa
lagi menyimpang diam-diam.

Set per wilayah
---------------
Sembilan silinder lama hanya cocok untuk ``rect``: di ``l_shape``/``u_shape``/
``plus`` hanya 6 di antaranya yang jatuh di dalam wilayah pemetaan, sehingga
klaim "9 rintangan statis" tidak seragam antar-wilayah. ``OBSTACLES_BY_REGION``
memberi tiap wilayah 9 rintangan DI DALAM wilayahnya sendiri.

Set non-convex dibangkitkan sekali dengan sisipan rakus max-min (farthest-point)
atas sampel seragam di ``poly.buffer(-2.5)`` lalu DIBEKUKAN sebagai literal di
sini — bukan diacak saat runtime. Kriteria yang dijamin
``test_obstacle_paths.py``: 9 titik di dalam wilayah, jarak ke tepi >= 2.5 m,
jarak antar-rintangan >= 3.5 m, dan **jarak >= 1.6 m dari setiap centroid sel
Voronoi**.

Syarat centroid itu ditambahkan setelah kegagalan nyata. Set pertama dibuat
dengan k-means, yang menyebar titik ke pusat-pusat partisi luas — dan centroid
Lloyd adalah pusat partisi luas dari poligon yang SAMA, jadi keduanya berebut
lokasi. Hasilnya di ``u_shape``: centroid iris_2 jatuh 0.24 m dan iris_7 0.40 m
dari pusat silinder, di dalam radius tabrakan fisik 0.62 m. ``return_to_centroid``
menembak lurus ke centroid tanpa perencanaan rintangan, jadi kedua drone
mendorong terus melawan CBF sampai MENABRAK dan mati. Coordinator kini juga
menggeser centroid keluar zona aman (``_push_centroid_clear``) sebagai lapis
kedua, tapi penempatan yang benar adalah lapis pertamanya.

``rect`` sengaja mempertahankan sembilan koordinat aslinya supaya run Skema 3
lama tetap sebanding.

CATATAN: rintangan TIDAK lagi memiliki bidang "sel pemilik". Kepemilikan
sekarang dihitung geometris (rintangan mana pun yang zona amannya menyentuh sel
tersapu sebuah drone menjadi tanggung jawab drone itu), karena kepemilikan
eksklusif lama membuat drone tetangga menyapu masuk ke zona aman tanpa pernah
merencanakan jalan memutar.
"""
import math

import numpy as np

OBSTACLE_RADIUS = 0.40
OBSTACLE_HEIGHT = 4.0

# Warna RViz per indeks (0..8) — hanya untuk visualisasi.
OBSTACLE_COLORS = (
    (1.00, 0.60, 0.00), (1.00, 0.95, 0.10), (0.10, 0.95, 0.20),
    (0.10, 0.85, 1.00), (0.90, 0.20, 1.00), (1.00, 0.40, 0.40),
    (0.40, 1.00, 0.70), (0.60, 0.60, 1.00), (1.00, 0.80, 0.50),
)

# (id, x, y) — radius & tinggi seragam.
OBSTACLES_BY_REGION = {
    'rect': (
        (101,  -1.5,   9.5),
        (102,   4.0,   6.0),
        (103,   6.5,   9.5),
        (104,  -8.0,  -2.0),
        (105,  -5.0,  -7.5),
        (106, -10.5, -12.5),
        (107,   6.0,  -4.0),
        (108,   0.0,   2.5),
        (109,   2.5,  -9.0),
    ),
    'l_shape': (
        (301,    1.9,  -11.5),
        (302,   11.5,  -11.5),
        (303,  -11.5,  -11.4),
        (304,   -5.8,   -5.8),
        (305,   11.5,   -4.6),
        (306,    4.9,   -2.5),
        (307,  -11.4,    1.9),
        (308,   -2.5,    5.3),
        (309,  -11.4,   11.4),
    ),
    'u_shape': (
        (301,  -11.5,  -11.4),
        (302,   -1.6,  -11.4),
        (303,    7.6,   -7.7),
        (304,   -6.5,   -6.7),
        (305,    0.0,   -4.5),
        (306,   -7.6,    0.0),
        (307,    7.5,    2.3),
        (308,  -11.4,   11.4),
        (309,   11.4,   11.5),
    ),
    'plus': (
        (301,   -2.5,  -11.5),
        (302,    2.5,   -6.6),
        (303,  -11.5,   -2.5),
        (304,    6.3,   -1.4),
        (305,    0.0,   -0.0),
        (306,   -6.5,    2.5),
        (307,   11.5,    2.5),
        (308,    2.4,    6.5),
        (309,   -2.4,   11.5),
    ),
}

# Alias historis — wilayah persegi.
STATIC_OBSTACLES = OBSTACLES_BY_REGION['rect']
STATIC_RADIUS = OBSTACLE_RADIUS
STATIC_HEIGHT = OBSTACLE_HEIGHT

# Dua silinder bergerak menyusuri diagonal membentuk pola 'X'.
# HANYA dipakai Skema 4; Skema 3 statis-saja tidak men-spawn keduanya.
DYNAMIC_RADIUS = 0.45
DYNAMIC_AMPLITUDE = 10.0
DYNAMIC_OMEGA = (0.15, 0.11)
DYNAMIC_IDS = (201, 202)


def obstacles_for_region(region='rect'):
    """Daftar rintangan lengkap untuk sebuah wilayah.

    Kembalian: list ``(id, x, y, radius, height, (r, g, b))`` — bentuk yang
    dipakai coordinator. Wilayah tak dikenal (mis. YAML custom) memakai set
    ``rect``, karena itulah satu-satunya yang berkorespondensi dengan berkas
    world bawaan.
    """
    table = OBSTACLES_BY_REGION.get(region, OBSTACLES_BY_REGION['rect'])
    return [
        (oid, float(ox), float(oy), OBSTACLE_RADIUS, OBSTACLE_HEIGHT,
         OBSTACLE_COLORS[i % len(OBSTACLE_COLORS)])
        for i, (oid, ox, oy) in enumerate(table)
    ]


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


def min_clearance(pos_xy, t, drone_radius=0.22, region='rect',
                  include_dynamic=True):
    """Clearance FISIK terkecil dari satu posisi ke seluruh rintangan.

    Tanpa bantalan keamanan — ini jarak permukaan sebenarnya, dipakai untuk
    menilai hasil, bukan untuk mengendalikan.

    ``include_dynamic=False`` untuk Skema 3 statis-saja: silinder bergerak
    tidak di-spawn di sana, jadi memasukkannya akan menghasilkan clearance
    phantom yang lebih kecil dari kenyataan.
    """
    p = np.asarray(pos_xy, dtype=float)[:2]
    best = float('inf')
    for _oid, ox, oy in OBSTACLES_BY_REGION.get(region, STATIC_OBSTACLES):
        d = float(np.linalg.norm(p - np.array([ox, oy])))
        best = min(best, d - (OBSTACLE_RADIUS + drone_radius))
    if include_dynamic:
        for (q, _v) in dynamic_state(t):
            d = float(np.linalg.norm(p - q))
            best = min(best, d - (DYNAMIC_RADIUS + drone_radius))
    return best
