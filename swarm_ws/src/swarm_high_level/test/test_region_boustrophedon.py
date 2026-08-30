"""Uji geometri wilayah non-convex + boustrophedon multi-interval.

Murni numpy/shapely — tanpa ROS/Gazebo, < 2 detik. Menjaga regresi utama:
garis sapuan TIDAK BOLEH menembus keluar poligon sel.
"""
import numpy as np
import pytest
from shapely.geometry import LineString, MultiPoint, Point
from shapely.geometry import Polygon as SpPolygon

from swarm_high_level.world.coverage_path import (
    clip_poly_to_region, generate_boustrophedon,
    polygon_scanline_intersections)
from swarm_high_level.world.region import (
    REGION_PRESETS, grid_region_mask, load_region, region_seed_points)

NONCONVEX = ['l_shape', 'u_shape', 'plus']


# ── Wilayah ──────────────────────────────────────────────────────────────

@pytest.mark.parametrize('name', list(REGION_PRESETS))
def test_preset_loads_valid(name):
    ring, poly = load_region(name)
    assert poly.is_valid and poly.area > 50.0
    assert len(ring) == len(REGION_PRESETS[name])


@pytest.mark.parametrize('name', NONCONVEX)
def test_preset_is_actually_nonconvex(name):
    _, poly = load_region(name)
    # Non-convex: luas < luas convex hull-nya.
    assert poly.area < poly.convex_hull.area - 1.0


@pytest.mark.parametrize('name', list(REGION_PRESETS))
def test_seed_points_inside_region(name):
    _, poly = load_region(name)
    pts = region_seed_points(poly, 7, seed=7)
    assert len(pts) == 7
    for p in pts:
        assert poly.buffer(1e-6).contains(Point(p[0], p[1]))


@pytest.mark.parametrize('name', list(REGION_PRESETS))
def test_grid_region_mask_matches_area(name):
    _, poly = load_region(name)
    x0, y0, x1, y1 = poly.bounds
    x_min, y_min = x0 - 1.0, y0 - 1.0
    dx = dy = (x1 - x0 + 2.0) / 100.0
    mask = grid_region_mask(poly, x_min, y_min, dx, dy, 100)
    approx_area = mask.sum() * dx * dy
    assert approx_area == pytest.approx(poly.area, rel=0.06)


def test_yaml_file_load(tmp_path):
    f = tmp_path / 'r.yaml'
    f.write_text('vertices:\n- [0, 0]\n- [10, 0]\n- [10, 10]\n- [0, 10]\n')
    ring, poly = load_region(str(f))
    assert poly.area == pytest.approx(100.0)
    assert len(ring) == 4


def test_unknown_region_raises():
    with pytest.raises(ValueError):
        load_region('tidak_ada_preset_ini')


# ── Scanline ─────────────────────────────────────────────────────────────

def test_scanline_u_shape_notch_gives_two_intervals():
    _, poly = load_region('u_shape')
    ring = [np.asarray(c) for c in poly.exterior.coords[:-1]]
    xs = polygon_scanline_intersections(ring, 8.0)   # di dalam takik
    assert len(xs) == 4                              # dua interval terpisah


def test_scanline_u_shape_below_notch_one_interval():
    _, poly = load_region('u_shape')
    ring = [np.asarray(c) for c in poly.exterior.coords[:-1]]
    xs = polygon_scanline_intersections(ring, -8.0)  # di bawah takik
    assert len(xs) == 2


# ── Boustrophedon ────────────────────────────────────────────────────────

def _cells_from_region(poly, n=7):
    """Bagi wilayah jadi n sel via titik generator + Voronoi bounding-box."""
    from scipy.spatial import Voronoi
    gens = region_seed_points(poly, n, seed=3)
    x0, y0, x1, y1 = poly.bounds
    far = [(x0 - 100, y0 - 100), (x1 + 100, y0 - 100),
           (x1 + 100, y1 + 100), (x0 - 100, y1 + 100)]
    vor = Voronoi(np.vstack([gens, far]))
    cells = []
    for i in range(len(gens)):
        reg = vor.regions[vor.point_region[i]]
        if not reg or -1 in reg:
            continue
        poly_v = SpPolygon([vor.vertices[k] for k in reg]).intersection(poly)
        if poly_v.geom_type == 'MultiPolygon':
            poly_v = max(poly_v.geoms, key=lambda g: g.area)
        if poly_v.geom_type == 'Polygon' and poly_v.area > 2.0:
            cells.append([np.asarray(c) for c in poly_v.exterior.coords[:-1]])
    return cells


def _coverage_frac(path, cell_poly):
    samples = []
    for k in range(0, len(path) - 1, 2):
        a, b = np.asarray(path[k]), np.asarray(path[k + 1])
        L = np.linalg.norm(b - a)
        for t in np.arange(0.0, 1.0 + 1e-9, 0.1 / max(L, 0.1)):
            samples.append(tuple(a + t * (b - a)))
    if len(samples) < 2:
        return 0.0
    return MultiPoint(samples).buffer(0.95).intersection(cell_poly).area / cell_poly.area


@pytest.mark.parametrize('name', list(REGION_PRESETS))
def test_interior_horizontal_rows_inside_cell(name):
    """Segmen HORIZONTAL (baris interior) tidak boleh keluar sel. Segmen rantai
    tepi (miring) memang menyusuri tepi — dilewati."""
    _, poly = load_region(name)
    for cell in _cells_from_region(poly):
        cell_poly = SpPolygon(cell).buffer(0.12)
        path = generate_boustrophedon(cell, sweep_spacing=1.45, margin=0.05)
        if len(path) < 2:
            continue
        for k in range(0, len(path) - 1, 2):
            a, b = path[k], path[k + 1]
            if abs(a[1] - b[1]) > 1e-6:
                continue
            assert cell_poly.contains(LineString([tuple(a), tuple(b)])), (
                f'{name}: baris horizontal keluar sel di y={a[1]:.2f}')


@pytest.mark.parametrize('name', list(REGION_PRESETS))
def test_path_covers_cell(name):
    """Rute (rantai tepi + baris interior) menutup >= 90% tiap sel."""
    _, poly = load_region(name)
    for cell in _cells_from_region(poly):
        cell_poly = SpPolygon(cell)
        path = generate_boustrophedon(cell, sweep_spacing=1.45, margin=0.05)
        if len(path) < 2:
            continue
        assert _coverage_frac(path, cell_poly) >= 0.90, (
            f'{name}: cakupan sel hanya {_coverage_frac(path, cell_poly):.1%}')


@pytest.mark.parametrize('name', list(REGION_PRESETS))
def test_path_pairs_even_and_inside_region(name):
    _, poly = load_region(name)
    rbuf = poly.buffer(0.20)
    for cell in _cells_from_region(poly):
        path = generate_boustrophedon(cell, sweep_spacing=1.45, margin=0.05)
        assert len(path) % 2 == 0
        for p in path:
            assert rbuf.contains(Point(p[0], p[1])), f'{name}: titik {p} di luar wilayah'


def test_start_point_is_a_cell_corner_not_row_end():
    """Titik masuk misi (path[0]) harus di TEPI sel, bukan ujung baris interior
    yang menggantung di tengah sel (regresi cap_pre lama)."""
    _, poly = load_region('rect')
    for cell in _cells_from_region(poly)[:3]:
        path = generate_boustrophedon(cell, sweep_spacing=1.45, margin=0.05)
        if len(path) < 4:
            continue
        edge = SpPolygon(cell).exterior
        assert edge.distance(Point(path[0][0], path[0][1])) < 0.5


def test_rect_interior_rows_span_full_width():
    _, poly = load_region('rect')
    cell = [np.asarray(c) for c in poly.exterior.coords[:-1]]
    path = generate_boustrophedon(cell, sweep_spacing=1.45, margin=0.02)
    widths = [abs(path[k][0] - path[k + 1][0]) for k in range(0, len(path) - 1, 2)
              if abs(path[k][1] - path[k + 1][1]) < 1e-6]
    assert widths and max(widths) > 27.0


def test_tiny_cell_returns_something():
    thin = [np.array([0.0, 0.0]), np.array([6.0, 0.0]),
            np.array([6.0, 1.0]), np.array([0.0, 1.0])]
    path = generate_boustrophedon(thin, sweep_spacing=1.45, margin=0.02)
    assert len(path) >= 2


# ── clip_poly_to_region ──────────────────────────────────────────────────

def test_clip_returns_input_for_full_rect():
    _, poly = load_region('rect')
    square = [np.array([-5, -5]), np.array([5, -5]),
              np.array([5, 5]), np.array([-5, 5])]
    ring, ctr = clip_poly_to_region(square, poly)
    assert SpPolygon(ring).area == pytest.approx(100.0, rel=0.01)
    assert np.linalg.norm(ctr) < 0.1


def test_clip_centroid_inside_concave_result():
    _, poly = load_region('u_shape')
    big = [np.array([-14, -14]), np.array([14, -14]),
           np.array([14, 14]), np.array([-14, 14])]
    ring, ctr = clip_poly_to_region(big, poly)
    assert poly.buffer(1e-6).contains(Point(ctr[0], ctr[1]))
