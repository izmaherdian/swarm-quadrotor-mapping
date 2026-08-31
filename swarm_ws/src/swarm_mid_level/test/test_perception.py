"""Deteksi rintangan dari LiDAR — silinder harus ditemukan, hantu ditolak.

Scan disintesis dengan ray-casting analitik terhadap silinder yang diketahui,
lalu detektor diuji apakah menemukan kembali pusat dan radiusnya. Yang diuji
adalah kemampuan MEMULIHKAN geometri dari pantulan, bukan mencocokkan tabel.
"""
import numpy as np
import pytest

from swarm_mid_level.perception.obstacle_map import (
    ObstacleMap, R_MAX, detect, fit_circle, scan_to_points)

N_RAYS = 360
A_MIN = -np.pi
A_INC = 2.0 * np.pi / N_RAYS
R_MAX_SENSOR = 12.0


def cast(sensor, yaw, cylinders, walls=15.0, n=N_RAYS):
    """Ray-cast analitik: jarak ke silinder terdekat, atau ke dinding arena."""
    sx, sy = sensor
    out = np.full(n, np.inf, dtype=np.float32)
    for i in range(n):
        a = A_MIN + A_INC * i + yaw
        dx, dy = np.cos(a), np.sin(a)
        best = np.inf
        for (cx, cy, r) in cylinders:
            fx, fy = sx - cx, sy - cy
            b = 2.0 * (fx * dx + fy * dy)
            c = fx * fx + fy * fy - r * r
            disc = b * b - 4.0 * c
            if disc < 0:
                continue
            t = (-b - np.sqrt(disc)) / 2.0
            if 0.05 < t < best:
                best = t
        for t_wall in (( walls - sx) / dx if abs(dx) > 1e-9 else np.inf,
                       (-walls - sx) / dx if abs(dx) > 1e-9 else np.inf,
                       ( walls - sy) / dy if abs(dy) > 1e-9 else np.inf,
                       (-walls - sy) / dy if abs(dy) > 1e-9 else np.inf):
            if 0.05 < t_wall < best:
                best = t_wall
        out[i] = best if np.isfinite(best) and best < R_MAX_SENSOR else np.inf
    return out


ARENA = (-15.0, -15.0, 15.0, 15.0)


# ── Geometri dasar ───────────────────────────────────────────────────────

def test_scan_to_points_places_returns_in_world():
    """Satu pantulan lurus ke depan pada yaw 90 derajat jatuh di +Y."""
    r = np.full(N_RAYS, np.inf, dtype=np.float32)
    r[N_RAYS // 2] = 3.0                       # sudut 0 relatif badan
    pts = scan_to_points(r, A_MIN, A_INC, 1.0, 2.0, np.pi / 2)
    assert pts.shape == (1, 2)
    assert pts[0] == pytest.approx([1.0, 5.0], abs=1e-3)


@pytest.mark.parametrize('dist', [2.0, 4.0, 6.0])
def test_fit_circle_recovers_centre_from_near_arc(dist):
    """LiDAR hanya melihat sisi DEKAT; pusat tetap harus mendarat di tengah."""
    cyl = (dist, 0.0, 0.40)
    rng = cast((0.0, 0.0), 0.0, [cyl])
    pts = scan_to_points(rng, A_MIN, A_INC, 0.0, 0.0, 0.0)
    pts = pts[np.linalg.norm(pts - np.array([dist, 0.0]), axis=1) < 1.5]
    ctr, rad = fit_circle(pts, (0.0, 0.0))
    assert np.linalg.norm(ctr - np.array([dist, 0.0])) < 0.35, (
        f'pusat meleset {np.linalg.norm(ctr - np.array([dist, 0.0])):.2f} m')
    assert 0.15 <= rad <= R_MAX


def test_centroid_alone_would_be_biased_towards_sensor():
    """Justifikasi koreksi kedalaman: centroid mentah SELALU terlalu dekat."""
    cyl = (4.0, 0.0, 0.40)
    rng = cast((0.0, 0.0), 0.0, [cyl])
    pts = scan_to_points(rng, A_MIN, A_INC, 0.0, 0.0, 0.0)
    pts = pts[np.linalg.norm(pts - np.array([4.0, 0.0]), axis=1) < 1.5]
    bias = 4.0 - pts.mean(axis=0)[0]
    assert bias > 0.15, 'centroid ternyata tidak bias — koreksi tak diperlukan?'
    ctr, _r = fit_circle(pts, (0.0, 0.0))
    assert abs(4.0 - ctr[0]) < bias, 'koreksi kedalaman tidak memperbaiki apa pun'


# ── Deteksi utuh ─────────────────────────────────────────────────────────

def test_detect_finds_every_cylinder_in_range():
    cyls = [(3.0, 1.0, 0.40), (-2.5, 2.0, 0.40), (0.5, -4.0, 0.40)]
    rng = cast((0.0, 0.0), 0.3, cyls)
    got = detect(rng, A_MIN, A_INC, 0.0, 0.0, 0.3, arena=ARENA)
    for (cx, cy, _r) in cyls:
        d = min(float(np.linalg.norm(np.asarray(c) - np.array([cx, cy])))
                for c, _ in got)
        assert d < 0.40, f'silinder ({cx}, {cy}) tidak ditemukan, terdekat {d:.2f} m'


def test_detect_rejects_arena_walls():
    """Dinding memantulkan ratusan sinar; tanpa saringan ia jadi 'rintangan'."""
    rng = cast((13.0, 0.0), 0.0, [])            # hanya dinding
    got = detect(rng, A_MIN, A_INC, 13.0, 0.0, 0.0, arena=ARENA)
    assert got == [], f'dinding lolos sebagai {len(got)} rintangan'


def test_detect_rejects_other_drones():
    """Sesama drone memantul seperti silinder kecil — harus dibuang."""
    mate = (2.5, 0.0)
    rng = cast((0.0, 0.0), 0.0, [(mate[0], mate[1], 0.22)])
    got = detect(rng, A_MIN, A_INC, 0.0, 0.0, 0.0,
                 others=[mate], arena=ARENA)
    assert got == [], f'drone lain lolos sebagai {len(got)} rintangan'


# ── Peta bersama ─────────────────────────────────────────────────────────

def test_map_needs_repeated_sightings_before_confirming():
    m = ObstacleMap(min_hits=4)
    for _ in range(3):
        m.update([((5.0, 5.0), 0.40)])
    assert m.confirmed() == [], 'terkonfirmasi terlalu cepat'
    m.update([((5.0, 5.0), 0.40)])
    assert len(m.confirmed()) == 1


def test_map_averages_noisy_detections():
    rng = np.random.default_rng(0)
    m = ObstacleMap(min_hits=3)
    for _ in range(40):
        c = np.array([5.0, -3.0]) + rng.normal(0, 0.12, 2)
        m.update([(c, 0.40 + rng.normal(0, 0.03))])
    conf = m.confirmed()
    assert len(conf) == 1, f'derau memecah satu benda jadi {len(conf)} jalur'
    _oid, x, y, r, _h, _c = conf[0]
    assert abs(x - 5.0) < 0.10 and abs(y + 3.0) < 0.10
    assert 0.35 < r < 0.60


def test_map_keeps_distinct_obstacles_apart():
    m = ObstacleMap(min_hits=2)
    for _ in range(5):
        m.update([((0.0, 0.0), 0.4), ((4.0, 0.0), 0.4)])
    assert len(m.confirmed()) == 2


def test_confirmed_matches_planner_tuple_shape():
    """Perencana dan QP membongkar (id, x, y, r, tinggi, warna)."""
    m = ObstacleMap(min_hits=1)
    m.update([((1.0, 2.0), 0.4)])
    oid, x, y, r, h, col = m.confirmed()[0]
    assert isinstance(oid, int) and len(col) == 3
    assert (x, y) == pytest.approx((1.0, 2.0))
    assert h == 4.0


def test_map_builds_full_arena_from_a_flight():
    """Terbang menyusuri arena harus memulihkan seluruh sembilan silinder."""
    cyls = [(-1.5, 9.5, .4), (4.0, 6.0, .4), (6.5, 9.5, .4), (-8.0, -2.0, .4),
            (-5.0, -7.5, .4), (-10.5, -12.5, .4), (6.0, -4.0, .4),
            (0.0, 2.5, .4), (2.5, -9.0, .4)]
    m = ObstacleMap(min_hits=3)
    for y in np.arange(-13.0, 13.01, 1.45):
        for x in np.arange(-13.0, 13.01, 2.0):
            rng = cast((x, y), 0.0, cyls)
            m.update(detect(rng, A_MIN, A_INC, x, y, 0.0, arena=ARENA))
    conf = m.confirmed()
    for (cx, cy, _r) in cyls:
        d = min(float(np.hypot(c[1] - cx, c[2] - cy)) for c in conf)
        assert d < 0.45, f'silinder ({cx}, {cy}) tidak dipetakan (terdekat {d:.2f} m)'


# ── Regresi: rintangan hantu dari sesama drone ───────────────────────────
#
# Terlihat di RViz 31 Agu: ~22 rintangan terpetakan padahal hanya ada 9, dan
# yang palsu menumpuk di tepi bawah dekat landasan. Dua sebabnya diuji di sini.

def test_moving_reflector_is_never_confirmed():
    """Drone melaju 1.6 m/s hanya bergeser 0.16 m antar-scan pada 10 Hz — jauh
    di bawah radius asosiasi 1.20 m — jadi tanpa uji gerak ia terasosiasi ke
    satu jalur, mengumpulkan hit, lalu dikonfirmasi sebagai silinder."""
    m = ObstacleMap(min_hits=4)
    for k in range(40):                       # melintas 6.4 m
        m.update([((-3.0 + 0.16 * k, 1.0), 0.30)])
    assert m.confirmed() == [], (
        f'benda bergerak dikonfirmasi sebagai rintangan statis: {m.confirmed()}')
    _tot, _conf, moving = m.n_tracks()
    assert moving >= 1, 'jalur bergerak tidak ditandai'


def test_static_obstacle_survives_the_motion_test():
    """Uji gerak tidak boleh membuang rintangan diam yang deteksinya berderau."""
    rng = np.random.default_rng(3)
    m = ObstacleMap(min_hits=4)
    for _ in range(60):
        m.update([(np.array([4.0, -2.0]) + rng.normal(0, 0.10, 2), 0.42)])
    assert len(m.confirmed()) == 1, 'rintangan diam ikut tertolak'


def test_low_flying_drone_must_be_masked_too():
    """Masker drone TIDAK boleh bersyarat ketinggian: yang sedang lepas landas
    atau transit rendah tetap memantul dan akan jadi silinder palsu."""
    mate = (3.0, 0.0)                          # drone lain, ketinggian berapa pun
    rng = cast((0.0, 0.0), 0.0, [(mate[0], mate[1], 0.22)])
    assert detect(rng, A_MIN, A_INC, 0.0, 0.0, 0.0,
                  others=[mate], arena=ARENA) == []
    # tanpa masker ia MEMANG terdeteksi — itulah kegagalan yang direproduksi
    assert detect(rng, A_MIN, A_INC, 0.0, 0.0, 0.0,
                  others=[], arena=ARENA) != []


def test_cluster_of_parked_drones_yields_no_obstacles():
    """Tujuh drone berdempetan di landasan: sumber utama hantu di gambar RViz."""
    pad = [(-3.0, -13.0), (-1.0, -13.0), (1.0, -13.0), (3.0, -13.0),
           (-2.0, -12.0), (0.0, -12.0), (2.0, -12.0)]
    m = ObstacleMap(min_hits=3)
    for step in range(30):
        sensor = (0.0, -9.0 + 0.02 * step)
        rng = cast(sensor, 0.0, [(px, py, 0.22) for px, py in pad])
        m.update(detect(rng, A_MIN, A_INC, sensor[0], sensor[1], 0.0,
                        others=pad, arena=ARENA))
    assert m.confirmed() == [], (
        f'{len(m.confirmed())} rintangan hantu dari kelompok drone di landasan')
