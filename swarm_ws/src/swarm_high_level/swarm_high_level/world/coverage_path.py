"""Geometri jalur cakupan — scanline, boustrophedon, centroid, klip wilayah.

Dipisah dari coordinator supaya bisa diuji tanpa ROS/Gazebo. Semua fungsi
murni numpy/shapely.

`generate_boustrophedon`:
  * baris INTERIOR tetap horizontal (sapuan lawnmower biasa), sadar non-convex
    (scanline dipotong jadi pasangan interval berurutan);
  * baris PERTAMA & TERAKHIR mengikuti tepi bawah / atas sel (rantai vertex),
    sehingga wedge di pojok/tepi miring ikut tersapu TANPA sisipan aneh di
    awal/akhir misi — drone masuk lewat pojok natural dan setelah baris
    terakhir bisa langsung pulang.
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

    Kembalikan ``(ring, centroid)``: ring komponen terluas hasil irisan (atau
    ``pts`` apa adanya bila irisan kosong / sangat kecil), centroid yang
    DIJAMIN di dalam ring.
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


def _augment_ring(polygon, y):
    """Cincin poligon dengan titik potong garis horizontal ``y`` disisipkan."""
    pts = [np.asarray(p, dtype=float) for p in polygon]
    n = len(pts)
    out = []
    for i in range(n):
        a, b = pts[i], pts[(i + 1) % n]
        out.append(a)
        ya, yb = float(a[1]), float(b[1])
        if (ya - y) * (yb - y) < 0.0:          # benar-benar menyeberang
            out.append(a + ((y - ya) / (yb - ya)) * (b - a))
    return out


def _cap_chain(polygon, y, below=True):
    """Rantai batas sel yang berada di BAWAH (atau di ATAS) garis ``y``.

    Inilah "baris" pertama/terakhir: bukan garis horizontal, melainkan tepi
    sel itu sendiri, sehingga wedge di pojok ikut tersapu. Kembalikan daftar
    titik terurut menyusuri batas.
    """
    ring = _augment_ring(polygon, y)
    n = len(ring)
    tol = 1e-6
    keep = [(float(p[1]) <= y + tol) if below else (float(p[1]) >= y - tol)
            for p in ring]
    if all(keep) or not any(keep):
        return []
    start = next(i for i in range(n) if keep[i] and not keep[(i - 1) % n])
    chain, i = [], start
    for _ in range(n):
        if not keep[i]:
            break
        chain.append(ring[i])
        i = (i + 1) % n
    return chain


def _chain_segments(chain, min_seg=0.60):
    """Rantai titik → daftar rata [s0, e0, s1, e1, ...].

    Titik antara yang terlalu rapat DIGABUNG lebih dulu (bukan segmennya
    dibuang, supaya rantai tidak putus). Segmen di bawah ``min_seg`` tidak
    bisa direm oleh drone — pada jarak henti ~0.60 m di 1.6 m/s, segmen 0.3 m
    dijamin terlewati, dan itu muncul sebagai overshoot. Radius sensor 0.95 m
    tetap menutupi vertex yang digabung, jadi cakupan tidak berkurang.
    """
    pts = [np.asarray(p, dtype=float) for p in chain]
    if len(pts) < 2:
        return []
    keep = [pts[0]]
    for p in pts[1:-1]:
        if float(np.hypot(*(p - keep[-1]))) >= min_seg:
            keep.append(p)
    if float(np.hypot(*(pts[-1] - keep[-1]))) >= min_seg or len(keep) == 1:
        keep.append(pts[-1])
    else:
        keep[-1] = pts[-1]          # ganti titik terakhir, jangan buang ujungnya

    out = []
    for k in range(len(keep) - 1):
        out.extend([keep[k], keep[k + 1]])
    return out


def _orient(chain, ref, far=False):
    """Balik rantai bila perlu agar ujung awalnya TERDEKAT (atau terjauh) ke ``ref``."""
    if ref is None or len(chain) < 2:
        return chain
    d0 = float(np.hypot(*(np.asarray(chain[0], float) - ref)))
    d1 = float(np.hypot(*(np.asarray(chain[-1], float) - ref)))
    want_flip = (d1 > d0) if far else (d1 < d0)
    return chain[::-1] if want_flip else chain


def generate_boustrophedon(polygon, sweep_spacing=1.45, margin=0.02,
                           start_from_top=False, obstacles=None,
                           entry_point=None, prefer_far=False):
    """Rute sapuan sel — kembalikan daftar rata ``[s0, e0, s1, e1, ...]``.

    Struktur SELALU bawah→atas: rantai tepi BAWAH → baris horizontal interior
    → rantai tepi ATAS. Baris interior tetap horizontal; hanya rantai tepi yang
    mengikuti sisi miring sel.

    ``entry_point`` (posisi drone) menentukan di ujung MANA rantai bawah drone
    masuk — dipilih yang terdekat, jadi transit tidak memutar ke sisi jauh.
    Arah tiap baris berikutnya dirantai secara greedy dari ujung baris
    sebelumnya, yang secara alami menghasilkan zigzag.
    ``prefer_far=True`` membalik pilihan itu (dipakai untuk deconflict start).
    """
    if len(polygon) < 3:
        return [poly_centroid(polygon)]

    pts = np.array(polygon, dtype=float)
    min_y, max_y = float(pts[:, 1].min()), float(pts[:, 1].max())
    entry = None if entry_point is None else np.asarray(entry_point, float)[:2]

    # Pita interior [lo, hi]; di luar itu ditangani rantai tepi bawah/atas.
    height = max_y - min_y
    inset = min(0.90, 0.40 * height)
    lo, hi = min_y + inset, max_y - inset

    bottom = _cap_chain(polygon, lo, below=True)
    top = _cap_chain(polygon, hi, below=False)

    # Level-level interior beserta interval-x-nya (bisa >1 di sel cekung).
    levels = []
    if hi - lo >= 0.30:
        n = max(1, int(math.ceil((hi - lo) / sweep_spacing)))
        ys = [0.5 * (lo + hi)] if n == 1 else list(np.linspace(lo, hi, n))
        for y in ys:
            xs = polygon_scanline_intersections(polygon, y)
            segs = []
            for k in range(0, len(xs) - 1, 2):
                xl, xr = xs[k] + margin, xs[k + 1] - margin
                if obstacles:
                    xl, xr = _trim_interval_for_obstacles(xl, xr, y, obstacles)
                if xr - xl >= 0.30:
                    segs.append((xl, xr))
            if segs:
                levels.append((y, segs))

    # Rakit: rantai bawah (ujung terdekat ke drone) → interior → rantai atas.
    bottom = _orient(bottom, entry, far=prefer_far)
    flat = _chain_segments(bottom)
    cur = np.asarray(bottom[-1], float) if bottom else entry

    for (y, segs) in levels:
        remaining = list(segs)
        while remaining:
            best = None
            for idx, (xl, xr) in enumerate(remaining):
                for a, b in (((xl, y), (xr, y)), ((xr, y), (xl, y))):
                    d = 0.0 if cur is None else float(
                        np.hypot(a[0] - cur[0], a[1] - cur[1]))
                    if best is None or d < best[0]:
                        best = (d, idx, np.array(a), np.array(b))
            _, idx, a, b = best
            flat.extend([a, b])
            cur = b
            remaining.pop(idx)

    flat.extend(_chain_segments(_orient(top, cur)))

    if len(flat) < 2:
        return [poly_centroid(polygon)]

    if start_from_top:
        # Balik urutan & arah semua segmen: sapuan jadi atas→bawah. Tidak
        # dipakai selama semua drone lepas landas dari bawah arena.
        flat = flat[::-1]

    return [np.asarray(p, dtype=float) for p in flat]
