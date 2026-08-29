"""Definisi wilayah pemetaan — mendukung poligon non-convex.

Dipakai koordinator untuk tiga hal:
  1. seed generator Lloyd (titik awal di DALAM wilayah),
  2. kliping sel Voronoi ke batas wilayah,
  3. masker grid coverage (sel grid mana yang dihitung).

Semua koordinat meter, frame 'world'. Wilayah harus poligon sederhana tanpa
lubang, muat di dalam arena Gazebo ~±15 m. Preset bawaan + pemuat YAML
(`vertices: [[x, y], ...]`).

Judul abstract yang di-ACC: "... Non-Convex Geodetic Mapping" — semua preset
selain `rect` sengaja cekung untuk menguji jalur non-convex.
"""
import os

import numpy as np
from shapely.geometry import Point, Polygon
from shapely.prepared import prep

# Preset — semua muat dalam ±14 m. `rect` = area aktif lama (28×28 m) supaya
# hasil Skema 1/2 tetap sebanding dengan run-run sebelumnya.
REGION_PRESETS = {
    'rect': [(-14.0, -14.0), (14.0, -14.0), (14.0, 14.0), (-14.0, 14.0)],
    # L: penuh di paruh bawah, hanya sisi kiri di paruh atas.
    'l_shape': [(-14.0, -14.0), (14.0, -14.0), (14.0, 0.0),
                (0.0, 0.0), (0.0, 14.0), (-14.0, 14.0)],
    # U: takik persegi dipotong dari tepi atas-tengah.
    'u_shape': [(-14.0, -14.0), (14.0, -14.0), (14.0, 14.0), (5.0, 14.0),
                (5.0, -2.0), (-5.0, -2.0), (-5.0, 14.0), (-14.0, 14.0)],
    # Plus / salib simetris.
    'plus': [(-5.0, -14.0), (5.0, -14.0), (5.0, -5.0), (14.0, -5.0),
             (14.0, 5.0), (5.0, 5.0), (5.0, 14.0), (-5.0, 14.0),
             (-5.0, 5.0), (-14.0, 5.0), (-14.0, -5.0), (-5.0, -5.0)],
}


def load_region(name_or_path):
    """Kembalikan ``(ring, poly)``.

    ring : list[np.ndarray(2,)]  — cincin-luar tak-tertutup (n vertex)
    poly : shapely.Polygon
    """
    if name_or_path in REGION_PRESETS:
        verts = list(REGION_PRESETS[name_or_path])
    elif os.path.isfile(str(name_or_path)):
        import yaml
        with open(name_or_path) as f:
            data = yaml.safe_load(f)
        verts = [(float(v[0]), float(v[1])) for v in data['vertices']]
    else:
        raise ValueError(
            f"region '{name_or_path}' bukan preset {list(REGION_PRESETS)} "
            f"dan bukan berkas YAML yang ada")

    poly = Polygon(verts)
    if not poly.is_valid or poly.area < 1.0:
        raise ValueError(
            f"region '{name_or_path}' menghasilkan poligon tidak valid "
            f"(valid={poly.is_valid}, area={poly.area:.2f})")
    ring = [np.asarray(v, dtype=float) for v in poly.exterior.coords[:-1]]
    return ring, poly


def largest_polygon(geom):
    """Komponen Polygon terluas dari Polygon / MultiPolygon / GeometryCollection.

    Klip Voronoi × wilayah non-convex bisa memecah sel jadi beberapa keping;
    drone hanya menyapu keping terbesarnya.
    """
    if geom is None or geom.is_empty:
        return None
    if geom.geom_type == 'Polygon':
        return geom
    polys = [g for g in getattr(geom, 'geoms', []) if g.geom_type == 'Polygon']
    return max(polys, key=lambda p: p.area) if polys else None


def poly_ring(poly):
    """shapely.Polygon -> list[np.ndarray(2,)] cincin-luar tak-tertutup."""
    return [np.asarray(c, dtype=float) for c in poly.exterior.coords[:-1]]


def region_seed_points(poly, n, seed=7):
    """n titik generator di DALAM ``poly``, tersebar merata (k-means++ deterministik)."""
    minx, miny, maxx, maxy = poly.bounds
    rng = np.random.default_rng(seed)
    pp = prep(poly)

    # Kandidat interior via rejection sampling grid halus (~4000 titik uji).
    step = max(0.5, ((maxx - minx) * (maxy - miny) / 4000.0) ** 0.5)
    cand = [
        (x, y)
        for y in np.arange(miny + 0.5 * step, maxy, step)
        for x in np.arange(minx + 0.5 * step, maxx, step)
        if pp.contains(Point(x, y))
    ]
    if len(cand) < n:
        rp = poly.representative_point()
        return [np.array([rp.x, rp.y], dtype=float) for _ in range(n)]

    cand = np.asarray(cand, dtype=float)
    chosen = [cand[rng.integers(len(cand))]]
    d2 = np.sum((cand - chosen[0]) ** 2, axis=1)
    for _ in range(1, n):
        nxt = rng.choice(len(cand), p=d2 / d2.sum())
        chosen.append(cand[nxt])
        d2 = np.minimum(d2, np.sum((cand - chosen[-1]) ** 2, axis=1))
    return [np.asarray(c, dtype=float) for c in chosen]


def grid_region_mask(poly, x_min, y_min, dx, dy, grid_n):
    """Masker bool ``(grid_n, grid_n)``: True bila pusat sel di dalam ``poly``.

    Konvensi indeks sama dengan ``cov_grid`` koordinator:
    ``mask[i, j]`` untuk ``x = x_min + (i+0.5)*dx``, ``y = y_min + (j+0.5)*dy``.
    """
    mask = np.zeros((grid_n, grid_n), dtype=bool)
    pp = prep(poly)
    for i in range(grid_n):
        cx = x_min + (i + 0.5) * dx
        for j in range(grid_n):
            cy = y_min + (j + 0.5) * dy
            if pp.contains(Point(cx, cy)):
                mask[i, j] = True
    return mask
