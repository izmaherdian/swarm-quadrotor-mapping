"""Geometri jalur cakupan — scanline, boustrophedon, centroid, klip wilayah.

Dipisah dari coordinator supaya bisa diuji tanpa ROS/Gazebo. Semua fungsi
murni numpy/shapely.

`generate_boustrophedon` sekarang sadar-non-convex: setiap scanline horizontal
dipotong menjadi PASANGAN interval berurutan, jadi sel berlekuk tidak lagi
disapu dengan garis lurus yang menembus keluar poligon.
"""
import math

import numpy as np
from shapely.geometry import Polygon as SpPolygon


def poly_centroid(pts):
    """Titik berat geometris poligon (Shoelace)."""
    p = np.array(pts, dtype=float)
    n = len(p)
    if n < 3:
        return p.mean(axis=0) if n else np.zeros(2)
    A = cx = cy = 0.0
    for i in range(n):
        x0, y0 = p[i]
        x1, y1 = p[(i + 1) % n]
        f = x0 * y1 - x1 * y0
        A += f
        cx += (x0 + x1) * f
        cy += (y0 + y1) * f
    A *= 0.5
    if abs(A) < 1e-9:
        return p.mean(axis=0)
    return np.array([cx / (6 * A), cy / (6 * A)])


def clip_poly_to_region(pts, region_poly):
    """Iris poligon convex (list vertex) dengan wilayah (boleh non-convex).

    Kembalikan ``(ring, centroid)``:
      ring     : vertex komponen terluas hasil irisan (atau ``pts`` apa adanya
                 bila irisan kosong / sangat kecil — mis. wilayah persegi penuh),
      centroid : titik berat yang DIJAMIN di dalam ring.
    """
    if region_poly is None or len(pts) < 3:
        return pts, poly_centroid(pts)
    try:
        inter = SpPolygon([(float(p[0]), float(p[1])) for p in pts]).intersection(region_poly)
    except Exception:
        return pts, poly_centroid(pts)
    if inter.is_empty or inter.area < 0.25:
        return pts, poly_centroid(pts)
    if inter.geom_type == 'MultiPolygon':
        inter = max(inter.geoms, key=lambda g: g.area)
    if inter.geom_type != 'Polygon':
        return pts, poly_centroid(pts)
    ring = [np.array(c, dtype=float) for c in inter.exterior.coords[:-1]]
    ctr = inter.centroid
    if not inter.contains(ctr):
        ctr = inter.representative_point()
    return ring, np.array([ctr.x, ctr.y])


def polygon_scanline_intersections(polygon, y):
    """Titik potong garis horizontal ``y`` dengan sisi-sisi poligon, terurut."""
    xs = []
    pts = [np.array(p, dtype=float) for p in polygon]
    n = len(pts)
    for i in range(n):
        a, b = pts[i], pts[(i + 1) % n]
        ya, yb = a[1], b[1]
        if abs(yb - ya) < 1e-9:
            continue
        if not (min(ya, yb) <= y <= max(ya, yb)):
            continue
        t = (y - ya) / (yb - ya)
        xs.append(a[0] + t * (b[0] - a[0]))
    xs.sort()
    return xs


def _edge_index_of_point(pts, p, tol=0.05):
    """Indeks i sehingga p berada di segmen pts[i] -> pts[(i+1)%n]. -1 bila jauh."""
    n = len(pts)
    best_i, best_d = -1, tol
    for i in range(n):
        a = np.asarray(pts[i], dtype=float)
        b = np.asarray(pts[(i + 1) % n], dtype=float)
        ab = b - a
        L2 = float(ab @ ab)
        if L2 < 1e-12:
            continue
        t = float((np.asarray(p, dtype=float) - a) @ ab) / L2
        t = max(0.0, min(1.0, t))
        d = float(np.hypot(*(np.asarray(p, dtype=float) - (a + t * ab))))
        if d < best_d:
            best_d, best_i = d, i
    return best_i


def _boundary_trace(pts, a, b, y_pad=0.6):
    """Vertex poligon di antara a dan b sepanjang batas sel.

    Dipakai untuk belokan antar-baris: drone menyusuri sisi (miring) sel
    alih-alih memotong lurus, sehingga wedge di dekat tepi ikut tersapu.
    Ambil arc TERPENDEK, buang vertex yang keluar dari pita-y kedua titik
    (supaya belokan tidak melebar jauh).
    """
    pts = [np.asarray(v, dtype=float) for v in pts]
    n = len(pts)
    ia = _edge_index_of_point(pts, a)
    ib = _edge_index_of_point(pts, b)
    if ia < 0 or ib < 0 or ia == ib:
        return []

    fwd = []
    i = (ia + 1) % n
    for _ in range(n):
        fwd.append(pts[i])
        if i == ib:
            break
        i = (i + 1) % n

    bwd = []
    i = ia
    for _ in range(n):
        bwd.append(pts[i])
        if i == (ib + 1) % n:
            break
        i = (i - 1) % n

    def plen(seq):
        path = [np.asarray(a, dtype=float)] + seq + [np.asarray(b, dtype=float)]
        return sum(float(np.hypot(*(path[k + 1] - path[k])))
                   for k in range(len(path) - 1))

    arc = fwd if plen(fwd) <= plen(bwd) else bwd
    ylo = min(a[1], b[1]) - y_pad
    yhi = max(a[1], b[1]) + y_pad
    return [v for v in arc if ylo <= v[1] <= yhi]


def _trim_interval_for_obstacles(x_left, x_right, y, obstacles):
    """Pangkas satu interval [x_left, x_right] agar >= ~1.35 m dari pusat rintangan."""
    for obs in obstacles:
        ox, oy = obs[2], obs[3]
        if abs(y - oy) >= 1.35:
            continue
        d_crit = math.sqrt(max(0.01, 1.35 ** 2 - (y - oy) ** 2))
        if abs(x_left - ox) < d_crit:
            x_left = (ox - d_crit - 0.10) if ox > x_left else (ox + d_crit + 0.10)
        if abs(x_right - ox) < d_crit:
            x_right = (ox + d_crit + 0.10) if ox < x_right else (ox - d_crit - 0.10)
    return x_left, x_right


def generate_boustrophedon(polygon, sweep_spacing=1.45, margin=0.02,
                           start_from_top=False, obstacles=None):
    """Rute sapuan lawnmower zigzag di dalam sel poligon — sadar non-convex.

    Keluaran: ``(waypoints, meta)``.
      waypoints : daftar rata ``[start_0, end_0, start_1, end_1, ...]`` (baris
                  interior tetap HORIZONTAL), dikonsumsi berpasangan.
      meta      : ``{'connectors', 'cap_pre', 'cap_post'}``.
        connectors[i] : titik-titik tepi sel yang ditelusuri drone saat pindah
                        dari ujung baris i ke pangkal baris i+1 — belokan
                        "nempel" sisi miring sel, menutup wedge tepi.
        cap_pre  : telusuri tepi bawah sel sebelum baris 0 (tutup wedge bawah).
        cap_post : telusuri tepi atas sel setelah baris terakhir.

    Setiap scanline dipotong jadi PASANGAN interval berurutan sehingga sel
    cekung tidak disapu garis lurus yang menembus keluar.
    """
    empty_meta = {'connectors': [], 'cap_pre': [], 'cap_post': []}
    if len(polygon) < 3:
        return [poly_centroid(polygon)], empty_meta

    poly_pts = [np.asarray(v, dtype=float) for v in polygon]
    pts = np.array(polygon, dtype=float)
    min_y, max_y = pts[:, 1].min(), pts[:, 1].max()
    scan_min_y = min_y + margin
    scan_max_y = max_y - margin

    if scan_max_y <= scan_min_y:
        scan_y_levels = [0.5 * (min_y + max_y)]
    else:
        n_lines = max(1, int(math.ceil((scan_max_y - scan_min_y) / sweep_spacing)))
        if n_lines == 1:
            scan_y_levels = [0.5 * (scan_min_y + scan_max_y)]
        else:
            scan_y_levels = list(np.linspace(scan_min_y, scan_max_y, n_lines))
        if start_from_top:
            scan_y_levels = scan_y_levels[::-1]

    waypoints = []
    sweep_right = True
    for y in scan_y_levels:
        xs = polygon_scanline_intersections(polygon, y)
        if len(xs) < 2:
            continue

        intervals = []
        for k in range(0, len(xs) - 1, 2):
            xl, xr = xs[k] + margin, xs[k + 1] - margin
            if obstacles:
                xl, xr = _trim_interval_for_obstacles(xl, xr, y, obstacles)
            if (xr - xl) >= 0.45:
                intervals.append((xl, xr))
        if not intervals:
            continue

        for (xl, xr) in (intervals if sweep_right else list(reversed(intervals))):
            if sweep_right:
                waypoints.append(np.array([xl, y]))
                waypoints.append(np.array([xr, y]))
            else:
                waypoints.append(np.array([xr, y]))
                waypoints.append(np.array([xl, y]))
        sweep_right = not sweep_right

    if not waypoints:
        return [poly_centroid(polygon)], empty_meta

    n_rows = len(waypoints) // 2

    # Belokan antar-baris menyusuri tepi sel.
    connectors = []
    for r in range(n_rows - 1):
        end_r = waypoints[2 * r + 1]
        start_next = waypoints[2 * (r + 1)]
        connectors.append(_boundary_trace(poly_pts, end_r, start_next))
    connectors.append([])

    # Tutup wedge di puncak bawah & atas sel: telusuri tepi antara kedua
    # ujung baris terluar, lewat vertex sel yang di luar rentang baris.
    a0, b0 = waypoints[0], waypoints[1]
    cap_pre = _boundary_trace(poly_pts, b0, a0, y_pad=abs(a0[1] - min_y) + 0.3)
    cap_pre = [v for v in cap_pre if v[1] <= max(a0[1], b0[1]) + 0.05]
    aN, bN = waypoints[-2], waypoints[-1]
    cap_post = _boundary_trace(poly_pts, bN, aN, y_pad=abs(max_y - bN[1]) + 0.3)
    cap_post = [v for v in cap_post if v[1] >= min(aN[1], bN[1]) - 0.05]

    return waypoints, {'connectors': connectors,
                       'cap_pre': cap_pre, 'cap_post': cap_post}


def expand_path(waypoints, meta, min_seg=0.15):
    """Gabungkan baris interior + cap_pre + connector + cap_post jadi SATU
    daftar rata ``[s0, e0, s1, e1, ...]`` siap dikonsumsi state machine.

    Tiap segmen — baris horizontal MAUPUN pelacak-tepi — menjadi satu pasangan.
    Baris interior tetap horizontal; hanya segmen cap/connector yang miring.
    """
    wp = [np.asarray(p, dtype=float) for p in waypoints]
    if len(wp) < 2:
        return wp
    meta = meta or {}
    out = []

    def add_chain(chain):
        for k in range(len(chain) - 1):
            p, q = np.asarray(chain[k], float), np.asarray(chain[k + 1], float)
            if float(np.hypot(*(q - p))) > min_seg:
                out.append(p)
                out.append(q)

    pre = list(meta.get('cap_pre') or [])
    if pre:
        add_chain([wp[1]] + pre + [wp[0]])

    n_rows = len(wp) // 2
    conns = list(meta.get('connectors') or [])
    for r in range(n_rows):
        out.append(wp[2 * r])
        out.append(wp[2 * r + 1])
        if r < n_rows - 1:
            c = list(conns[r]) if r < len(conns) else []
            add_chain([wp[2 * r + 1]] + c + [wp[2 * (r + 1)]])

    post = list(meta.get('cap_post') or [])
    if post:
        add_chain([wp[-1]] + post + [wp[-2]])

    return out
