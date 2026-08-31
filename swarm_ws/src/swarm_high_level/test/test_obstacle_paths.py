"""Skema 3 — jalur sapuan harus benar-benar menghindari rintangan statis.

Tes ini menjaga perbaikan atas tiga cacat yang membuat Skema 3 tak pernah
stabil, dan SELURUHNYA gagal pada kode sebelum perbaikan:

T1  `_trim_interval_for_obstacles` hanya bisa menggeser UJUNG interval, tidak
    memecahnya. Rintangan di tengah baris dilewati begitu saja; rintangan dekat
    ujung justru membuat baris DIPERPANJANG menembusnya. Terukur: 10 dari 92
    segmen sapuan di `rect` melintasi silinder, mengenai kesembilan rintangan.
T2  Rantai tepi sel (baris pertama & terakhir) tidak sadar-rintangan sama
    sekali.
T3  Kepemilikan rintangan bersifat eksklusif (`contains_point`), padahal zona
    amannya kerap menyeberangi batas sel — drone tetangga menyapu masuk tanpa
    pernah merencanakan jalan memutar.

Angka yang diukur setelah perbaikan (lihat `test_no_segment_hits_obstacle`):
jarak terdekat ke pusat silinder 1.066-1.096 m, jauh di atas batas fisik
0.62 m, dengan cakupan 98.2-99.8%.
"""
import math
import pathlib
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from shapely.geometry import LineString, MultiPoint, Point
from shapely.geometry import Polygon as SpPolygon

from swarm_high_level.world.coverage_path import (
    OBSTACLE_KEEP_OUT, _split_interval_for_obstacles, clip_poly_to_region,
    generate_boustrophedon, route_around_obstacles)
from swarm_high_level.world.obstacles import (
    OBSTACLE_HEIGHT, OBSTACLE_RADIUS, OBSTACLES_BY_REGION, obstacles_for_region)
from swarm_high_level.world.region import (
    REGION_PRESETS, load_region, region_seed_points)

from test_region_boustrophedon import _cells_from_region

DRONE_RADIUS = 0.22
PHYSICAL_MIN = OBSTACLE_RADIUS + DRONE_RADIUS      # 0.62 m — kontak nyata
SENSOR_R = 0.95
WORLDS = pathlib.Path(__file__).resolve().parents[3] / 'src' / 'swarm_sim' / 'worlds'


def _clip_half(poly, pi, pj, margin=0.0):
    """Satu potongan bisector Voronoi — cermin `clip_voronoi` di coordinator."""
    m = (pi + pj) / 2.0
    n = pj - pi
    L = float(np.linalg.norm(n))
    if L < 1e-9:
        return poly
    n = n / L
    m = m - n * margin
    out = []
    for i in range(len(poly)):
        a = np.asarray(poly[i], float)
        b = np.asarray(poly[(i + 1) % len(poly)], float)
        da, db = float(np.dot(a - m, n)), float(np.dot(b - m, n))
        if da <= 0:
            out.append(a)
        if da * db < 0:
            out.append(a + (da / (da - db)) * (b - a))
    return out


def _lloyd_centroids(ring, poly, n=7, iters=25):
    """Centroid sel seperti yang BENAR-BENAR dihitung coordinator.

    `_cells_from_region` memakai Voronoi scipy tanpa relaksasi; coordinator
    memakai Lloyd 25x atas `region_seed_points`. Untuk menguji titik parkir,
    partisinya harus yang dipakai runtime — kalau tidak, tesnya mengukur
    geometri yang tidak pernah terbang.
    """
    seeds = region_seed_points(poly, n)
    for _ in range(iters):
        cells = []
        for i, pi in enumerate(seeds):
            c = [np.asarray(p, float) for p in ring]
            for j, pj in enumerate(seeds):
                if i != j:
                    c = _clip_half(c, pi, pj)
            c, _ = clip_poly_to_region(c, poly)
            cells.append(c)
        seeds = [np.asarray(SpPolygon([(p[0], p[1]) for p in c]).centroid.coords[0])
                 if len(c) >= 3 else s for c, s in zip(cells, seeds)]
    return seeds


def _plan_region(name):
    """Rencana sapuan seluruh sel sebuah wilayah, dengan kepemilikan ala T3."""
    _, poly = load_region(name)
    obs = obstacles_for_region(name)
    paths = []
    for cell in _cells_from_region(poly):
        cp = SpPolygon([(p[0], p[1]) for p in cell])
        # Kepemilikan geometris: rintangan mana pun yang zona amannya menyentuh
        # sel ini — bukan hanya yang berada DI DALAM-nya (perbaikan T3).
        mine = [o for o in obs
                if cp.intersects(Point(o[1], o[2]).buffer(OBSTACLE_KEEP_OUT))]
        paths.append(generate_boustrophedon(
            cell, sweep_spacing=1.45, margin=0.02,
            entry_point=np.array([0.0, -14.0]), obstacles=mine))
    return poly, obs, paths


# ── Cacat T1: pemecahan interval ─────────────────────────────────────────

def test_split_removes_obstacle_from_middle_of_row():
    """Rintangan di TENGAH baris harus memecahnya jadi dua, bukan diabaikan."""
    segs = _split_interval_for_obstacles(-5.0, 5.0, 2.5, [(0.0, 2.5)])
    assert len(segs) == 2, f'baris tidak dipecah: {segs}'
    assert segs[0][1] <= -OBSTACLE_KEEP_OUT + 1e-9
    assert segs[1][0] >= OBSTACLE_KEEP_OUT - 1e-9


@pytest.mark.parametrize('xl,xr', [(-0.5, 5.0), (-5.0, 0.5)])
def test_split_never_extends_a_row_across_an_obstacle(xl, xr):
    """Cacat lama: ujung didorong ke arah SALAH sehingga baris memanjang."""
    for lo, hi in _split_interval_for_obstacles(xl, xr, 2.5, [(0.0, 2.5)]):
        assert not (lo < 0.0 < hi), f'segmen [{lo}, {hi}] masih melintasi (0, 2.5)'
        assert lo >= xl - 1e-9 and hi <= xr + 1e-9, 'interval justru diperpanjang'


def test_split_leaves_clear_row_untouched():
    segs = _split_interval_for_obstacles(-5.0, 5.0, 2.5, [(0.0, 9.0)])
    assert segs == [(-5.0, 5.0)]


# ── Cacat T2: seluruh jalur, termasuk rantai tepi ────────────────────────

@pytest.mark.parametrize('name', list(REGION_PRESETS))
def test_no_segment_hits_obstacle(name):
    """TIDAK ADA segmen jalur yang menyentuh silinder — baris, konektor,
    maupun rantai tepi sel."""
    _poly, obs, paths = _plan_region(name)
    worst, where = float('inf'), None
    for path in paths:
        for k in range(len(path) - 1):
            seg = LineString([path[k], path[k + 1]])
            for oid, ox, oy, *_ in obs:
                d = seg.distance(Point(ox, oy))
                if d < worst:
                    worst, where = d, (oid, tuple(np.round(path[k], 2)))
    assert worst >= PHYSICAL_MIN, (
        f'{name}: segmen melintasi silinder #{where[0]} dari {where[1]} — '
        f'jarak ke pusat {worst:.3f} m < {PHYSICAL_MIN:.2f} m')


def test_route_around_obstacles_handles_edge_chain():
    """Rantai tepi lurus yang menembus rintangan pun harus dialihkan."""
    flat = [np.array([-4.0, 0.0]), np.array([4.0, 0.0])]
    out = route_around_obstacles(flat, [(0.0, 0.0)])
    assert len(out) > 2, 'tidak ada pengalihan yang disisipkan'
    for k in range(len(out) - 1):
        assert LineString([out[k], out[k + 1]]).distance(Point(0, 0)) >= PHYSICAL_MIN


def test_route_around_obstacles_is_noop_when_clear():
    flat = [np.array([-4.0, 5.0]), np.array([4.0, 5.0])]
    out = route_around_obstacles(flat, [(0.0, 0.0)])
    assert len(out) == 2
    assert np.allclose(out[0], flat[0]) and np.allclose(out[1], flat[1])


# ── Kontrak struktur & kelayakan gerak ───────────────────────────────────

@pytest.mark.parametrize('name', list(REGION_PRESETS))
def test_waypoints_stay_paired(name):
    """Coordinator memakai ``len(waypoints) // 2`` sebagai jumlah baris."""
    _poly, _obs, paths = _plan_region(name)
    for path in paths:
        assert len(path) % 2 == 0, 'daftar waypoint ganjil — pasangan baris rusak'


def _short_sweeps(paths, floor=0.60):
    """Panjang segmen SAPUAN di bawah jarak henti 0.60 m pada 1.6 m/s."""
    out = []
    for p in paths:
        for k in range(0, len(p) - 1, 2):
            L = float(np.linalg.norm(np.asarray(p[k + 1]) - np.asarray(p[k])))
            if L < floor:
                out.append(round(L, 3))
    return sorted(out)


@pytest.mark.parametrize('name', list(REGION_PRESETS))
def test_obstacle_routing_adds_no_unbrakeable_segment(name):
    """Pengalihan rute tidak boleh MENAMBAH segmen yang tak bisa direm.

    Ambang mutlaknya tidak dipakai di sini karena `_chain_segments` sudah
    menghasilkan satu segmen pendek di `l_shape` (0.485 m) bahkan TANPA
    rintangan: penggantian titik terakhir rantai tepi (`keep[-1] = pts[-1]`)
    bisa memperpendek segmen sebelumnya. Itu perilaku lama yang sudah terbukti
    di Gazebo pada Skema 1 & 2 (overshoot terukur 0.00 cm), dan mengubahnya
    akan menggeser jalur kedua skema itu setelah hasilnya dilaporkan. Jadi yang
    dijaga di sini adalah REGRESI: rintangan tidak boleh memperburuknya.
    """
    _poly, obs, paths_with = _plan_region(name)
    _ring, poly = load_region(name)
    paths_without = [
        generate_boustrophedon(cell, sweep_spacing=1.45, margin=0.02,
                               entry_point=np.array([0.0, -14.0]))
        for cell in _cells_from_region(poly)]

    with_obs, without = _short_sweeps(paths_with), _short_sweeps(paths_without)
    assert len(with_obs) <= len(without), (
        f'{name}: rintangan menambah segmen tak-terem — '
        f'tanpa {without}, dengan {with_obs}')
    if with_obs:
        assert min(with_obs) >= min(without) - 1e-9, (
            f'{name}: rintangan memperpendek segmen terpendek — '
            f'tanpa {without}, dengan {with_obs}')


@pytest.mark.parametrize('name', list(REGION_PRESETS))
def test_coverage_survives_the_detours(name):
    """Busur pengalih merangkap penyapu cincin di sekeliling silinder, jadi
    memutari rintangan TIDAK boleh meninggalkan lubang cakupan.

    Terukur: rect 99.7, l_shape 99.4, u_shape 99.8, plus 98.2 persen.
    """
    poly, obs, paths = _plan_region(name)
    target = poly
    for _oid, ox, oy, *_ in obs:
        target = target.difference(Point(ox, oy).buffer(OBSTACLE_RADIUS))

    samples = []
    for path in paths:
        for k in range(len(path) - 1):
            a, b = np.asarray(path[k]), np.asarray(path[k + 1])
            L = float(np.linalg.norm(b - a))
            for t in np.linspace(0.0, 1.0, max(2, int(L / 0.15))):
                samples.append(tuple(a + t * (b - a)))
    covered = MultiPoint(samples).buffer(SENSOR_R).intersection(target).area
    frac = covered / target.area
    assert frac >= 0.975, f'{name}: cakupan hanya {frac * 100:.1f}%'


# ── Tabel rintangan per wilayah ──────────────────────────────────────────

@pytest.mark.parametrize('name', list(REGION_PRESETS))
def test_region_has_nine_obstacles_well_placed(name):
    """Tiap wilayah punya 9 rintangan DI DALAM-nya, cukup jauh dari tepi dan
    dari satu sama lain agar drone masih bisa lewat di antaranya."""
    _ring, poly = load_region(name)
    table = OBSTACLES_BY_REGION[name]
    assert len(table) == 9, f'{name} punya {len(table)} rintangan, bukan 9'

    pts = [Point(ox, oy) for _oid, ox, oy in table]
    for (oid, ox, oy), p in zip(table, pts):
        assert poly.contains(p), f'{name}: #{oid} di ({ox}, {oy}) di LUAR wilayah'

    # `rect` adalah set historis (dipertahankan agar sebanding dengan run lama)
    # dan tidak memenuhi kriteria penempatan yang baru — hanya wilayah
    # non-convex yang dibangkitkan ulang.
    if name == 'rect':
        return
    edge = min(poly.exterior.distance(p) for p in pts)
    assert edge >= 2.5, f'{name}: rintangan hanya {edge:.2f} m dari tepi wilayah'
    pair = min(pts[i].distance(pts[j])
               for i in range(len(pts)) for j in range(i + 1, len(pts)))
    assert pair >= 3.5, f'{name}: dua rintangan hanya berjarak {pair:.2f} m'


@pytest.mark.parametrize('name', list(REGION_PRESETS))
def test_no_obstacle_sits_on_a_cell_centroid(name):
    """Centroid sel adalah TITIK PARKIR: `return_to_centroid` menembak lurus ke
    sana tanpa perencanaan rintangan, jadi centroid yang jatuh di atas silinder
    membuat drone mendorong melawan CBF sampai merayap masuk.

    Terukur 31 Agu di `u_shape`: centroid iris_2 hanya 0.24 m dan iris_7 0.40 m
    dari pusat silinder (radius tabrakan fisik 0.62 m) — keduanya MENABRAK dan
    mati. Set rintangan pertama dibuat dengan k-means, yang menyebar titik ke
    pusat partisi luas; centroid Lloyd adalah pusat partisi luas dari poligon
    yang sama, sehingga keduanya berebut lokasi.
    """
    ring, poly = load_region(name)
    obs = obstacles_for_region(name)
    worst, where = float('inf'), None
    for ctr in _lloyd_centroids(ring, poly):
        for oid, ox, oy, *_ in obs:
            d = float(np.hypot(ctr[0] - ox, ctr[1] - oy))
            if d < worst:
                worst, where = d, (oid, round(float(ctr[0]), 2), round(float(ctr[1]), 2))
    assert worst >= OBSTACLE_KEEP_OUT, (
        f'{name}: centroid {where[1:]} hanya {worst:.2f} m dari silinder '
        f'#{where[0]} — titik parkir di dalam zona aman {OBSTACLE_KEEP_OUT} m')


def test_obstacle_ids_unique_within_region():
    for name, table in OBSTACLES_BY_REGION.items():
        ids = [oid for oid, _x, _y in table]
        assert len(set(ids)) == len(ids), f'{name}: id rintangan duplikat'


# ── Berkas world hasil bangkitan ─────────────────────────────────────────

@pytest.mark.parametrize('name', list(OBSTACLES_BY_REGION))
def test_generated_world_matches_table(name):
    """SDF dan tabel Python tidak boleh menyimpang — dulu hanya dijaga komentar."""
    path = WORLDS / f'obstacles_{name}.world'
    assert path.exists(), (
        f'{path.name} belum dibangkitkan — jalankan tools/gen_obstacle_worlds.py')
    root = ET.parse(path).getroot()

    found = {}
    for m in root.iter('model'):
        mname = m.get('name', '')
        if not mname.startswith('static_obs_'):
            continue
        x, y, *_ = (float(v) for v in m.find('pose').text.split())
        found[int(mname.rsplit('_', 1)[1])] = (x, y)

    expected = {oid: (ox, oy) for oid, ox, oy in OBSTACLES_BY_REGION[name]}
    assert found == pytest.approx(expected, abs=1e-3), (
        f'{path.name} tidak sesuai obstacles.py — '
        'jalankan: python3 tools/gen_obstacle_worlds.py')

    for m in root.iter('model'):
        r = m.find('.//cylinder/radius')
        if r is not None and m.get('name', '').startswith('static_obs_'):
            assert float(r.text) == pytest.approx(OBSTACLE_RADIUS)
            assert float(m.find('.//cylinder/length').text) == pytest.approx(
                OBSTACLE_HEIGHT)


@pytest.mark.parametrize('name', list(OBSTACLES_BY_REGION))
def test_generated_world_has_no_dynamic_obstacles(name):
    """Skema 3 = statis saja. Silinder dinamis membentang z 0.25-3.85 m
    sementara drone menjelajah di 2.0 m: bila ter-spawn tapi tidak digerakkan,
    keduanya jadi rintangan diam yang tak dilihat perencana maupun QP."""
    text = (WORLDS / f'obstacles_{name}.world').read_text()
    assert 'dynamic_obs' not in text, (
        f'obstacles_{name}.world masih memuat rintangan dinamis')
