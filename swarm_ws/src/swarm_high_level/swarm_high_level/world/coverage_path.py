"""Geometri jalur cakupan — scanline, boustrophedon, centroid, klip wilayah.

Dipisah dari coordinator supaya bisa diuji tanpa ROS/Gazebo. Semua fungsi
murni numpy/shapely.

`generate_boustrophedon`:
  * baris INTERIOR tetap horizontal (sapuan lawnmower biasa), sadar non-convex
    (scanline dipotong jadi pasangan interval berurutan);
  * baris PERTAMA & TERAKHIR mengikuti tepi bawah / atas sel (rantai vertex),
    sehingga wedge di pojok/tepi miring ikut tersapu TANPA sisipan aneh di
    awal/akhir misi — drone masuk lewat pojok natural dan setelah baris
    terakhir bisa langsung pulang;
  * bila ``obstacles`` diberikan, baris DIPECAH pada irisan zona aman lalu
    seluruh jalur dilewatkan ``route_around_obstacles`` — satu lintasan-akhir
    yang menutup baris interior, konektor, dan rantai tepi sekaligus.

Kembalian selalu daftar rata berpanjang GENAP ``[s0, e0, s1, e1, ...]``:
coordinator memakai ``len(waypoints) // 2`` sebagai jumlah baris, jadi struktur
pasangan itu bagian dari kontrak, bukan kebetulan.
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


# ── Rintangan statis ────────────────────────────────────────────────────
#
# Radius zona aman yang DIRENCANAKAN. Kebutuhan fisiknya 0.87 m = 0.40
# (silinder) + 0.22 (drone) + 0.25 (`delta_static` pada CBF). Dipakai 1.30 m
# supaya tali busur pengalih rute pun masih lega: sebuah tali yang menyubtensi
# 70 derajat menyinggung sampai 1.30*cos(35) = 1.065 m dari pusat, masih 0.19 m
# di atas kebutuhan fisik.
#
# Tali busur dibatasi 70 derajat justru agar TIDAK pendek: pada 1.30 m itu
# panjangnya 1.49 m, jauh di atas `min_seg` 0.60 m. Busur ber-titik rapat akan
# digabung oleh `_chain_segments` dan sudutnya terpotong balik ke dalam zona
# aman — persis yang harus dihindari.
OBSTACLE_KEEP_OUT = 1.30

# Lantai jarak bebas saat menyederhanakan busur (lihat `_simplify_chain`).
# Di atas kebutuhan CBF 0.87 m dengan sisa 0.23 m untuk galat lacak, jadi QP
# tidak perlu melawan rencana.
OBSTACLE_CLEAR_MIN = 1.10

_ARC_MAX_STEP = math.radians(70.0)
_MAX_DETOURS = 12


def _obs_centers(obstacles):
    """Pusat rintangan sebagai array (N, 2).

    Menerima ``(x, y)`` maupun ``(id, x, y, ...)`` — dibedakan dari panjangnya,
    supaya tes bisa memakai pasangan polos sementara coordinator mengirim
    tuple lengkapnya.
    """
    cs = []
    for o in (obstacles or ()):
        cs.append((float(o[0]), float(o[1])) if len(o) == 2
                  else (float(o[1]), float(o[2])))
    return np.array(cs, dtype=float).reshape(-1, 2)


def _split_interval_for_obstacles(x_left, x_right, y, obstacles,
                                  keep_out=OBSTACLE_KEEP_OUT, min_len=0.60):
    """Buang irisan zona aman dari ``[x_left, x_right]`` → daftar sub-interval.

    Versi lama hanya bisa menggeser UJUNG interval, sehingga rintangan di
    tengah baris dilewati begitu saja dan rintangan dekat ujung justru membuat
    baris DIPERPANJANG menembusnya. Memecah interval adalah satu-satunya
    perlakuan yang benar; mesin multi-interval untuk sel cekung sudah ada, jadi
    hasil pecahan langsung tertangani.
    """
    ivs = [(float(x_left), float(x_right))]
    for ox, oy in _obs_centers(obstacles):
        dy = abs(y - oy)
        if dy >= keep_out:
            continue
        w = math.sqrt(keep_out ** 2 - dy ** 2)
        a, b = ox - w, ox + w
        nxt = []
        for lo, hi in ivs:
            if hi <= a or lo >= b:
                nxt.append((lo, hi))
                continue
            if lo < a:
                nxt.append((lo, a))
            if hi > b:
                nxt.append((b, hi))
        ivs = nxt
    return [(lo, hi) for lo, hi in ivs if hi - lo >= min_len]


def _push_outside(p, centers, keep_out):
    """Geser ``p`` keluar secara radial bila berada di dalam zona aman."""
    p = np.asarray(p, dtype=float)
    for c in centers:
        d = p - c
        r = float(np.hypot(*d))
        if r < keep_out - 1e-9:
            u = np.array([1.0, 0.0]) if r < 1e-9 else d / r
            p = c + keep_out * u
    return p


def _first_blocking(a, b, centers, keep_out):
    """Pusat rintangan pertama yang ditembus ruas ``a→b`` (None bila bersih)."""
    ab = b - a
    L = float(np.hypot(*ab))
    if L < 1e-9:
        return None
    u = ab / L
    best = None
    for c in centers:
        s = min(max(float(np.dot(c - a, u)), 0.0), L)
        if float(np.hypot(*(a + s * u - c))) < keep_out - 1e-6:
            if best is None or s < best[0]:
                best = (s, c)
    return None if best is None else best[1]


def _arc_points(c, frm, to, keep_out):
    """Titik-titik busur di sekeliling ``c``, dari arah ``frm`` ke arah ``to``.

    Titik terakhir tepat pada sinar c→to, jadi ruas terakhir menuju ``to``
    bersifat radial dan dijamin tidak masuk kembali ke zona aman.
    """
    a0 = math.atan2(frm[1] - c[1], frm[0] - c[0])
    a1 = math.atan2(to[1] - c[1], to[0] - c[0])
    dth = (a1 - a0 + math.pi) % (2.0 * math.pi) - math.pi
    n = max(1, int(math.ceil(abs(dth) / _ARC_MAX_STEP)))
    return [c + keep_out * np.array([math.cos(a0 + dth * i / n),
                                     math.sin(a0 + dth * i / n)])
            for i in range(1, n + 1)]


def _simplify_chain(chain, centers, floor, min_seg=0.60):
    """Buang titik antara selama ruas penggantinya masih >= ``floor`` dari pusat.

    Busur mentah meninggalkan titik kembar (ujung busur jatuh persis di
    tujuannya, karena ``_push_outside`` sudah menaruh tujuan di lingkaran yang
    sama) dan sesekali tali busur 0.1 m. Segmen sependek itu lebih pendek dari
    jarak henti 0.60 m di 1.6 m/s, jadi dijamin terlewati dan muncul sebagai
    overshoot. Penarikan tali di sini hanya membuang titik yang penghapusannya
    TERBUKTI masih aman, jadi jarak bebas tidak pernah ditukar dengan kerapian.
    """
    if len(chain) < 3:
        return chain
    out = [chain[0]]
    for i in range(1, len(chain) - 1):
        if _first_blocking(out[-1], chain[i + 1], centers, floor) is None:
            continue                        # penghapusannya TERBUKTI aman
        out.append(chain[i])
    if float(np.hypot(*(chain[-1] - out[-1]))) > 1e-6:
        out.append(chain[-1])
    else:
        out[-1] = chain[-1]                 # ganti ujungnya, jangan duplikat
    return out


def _safe_chain(a, b, centers, keep_out):
    """Rantai ``[a, ..., b]`` yang menjaga jarak ``keep_out`` dari tiap pusat."""
    chain = [a]
    cur = a
    for _ in range(_MAX_DETOURS):
        c = _first_blocking(cur, b, centers, keep_out)
        if c is None:
            break
        pts = _arc_points(c, cur, b, keep_out)
        chain.extend(pts)
        cur = pts[-1]
    chain.append(b)
    return _simplify_chain(chain, centers, OBSTACLE_CLEAR_MIN)


def route_around_obstacles(flat, obstacles, keep_out=OBSTACLE_KEEP_OUT):
    """Alihkan rute daftar rata ``[s0, e0, s1, e1, ...]`` mengitari rintangan.

    Dijalankan SEKALI di akhir, di atas jalur yang sudah dirakit, sehingga
    baris interior, konektor antar-segmen, DAN rantai tepi sel tertangani
    sekaligus — rantai tepi sebelumnya tidak sadar-rintangan sama sekali.

    Struktur pasangan dipertahankan: indeks genap tetap awal sebuah segmen.
    Bila sebuah pengalihan perlu disisipkan, segmen-segmennya berbagi ujung
    (langkah antar-pasangan jadi nol panjang) — pola yang sama dengan
    ``_chain_segments`` untuk rantai tepi.

    Busur ini merangkap penyapu: pada radius 1.30 m dengan radius sensor
    0.95 m, cincin di sekeliling silinder ikut terpetakan, jadi pengalihan
    rute tidak meninggalkan lubang cakupan.
    """
    centers = _obs_centers(obstacles)
    if not len(centers) or len(flat) < 2:
        return [np.asarray(p, dtype=float) for p in flat]

    pts = [_push_outside(p, centers, keep_out) for p in flat]
    out = []
    for k in range(len(pts) - 1):
        a, b = pts[k], pts[k + 1]
        chain = _safe_chain(a, b, centers, keep_out)
        if len(chain) == 2:
            if k % 2 == 0:                 # baris/segmen sapuan
                out.extend([a, b])
            continue                       # langkah lurus: tetap implisit
        for i in range(len(chain) - 1):    # pengalihan → segmen eksplisit
            out.extend([chain[i], chain[i + 1]])
    return out


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
    """Rute sapuan sel — kembalikan daftar rata [s0, e0, s1, e1, ...].

    Struktur SELALU bawah -> atas secara monoton: baris 0 dimulai langsung dari
    tepi bawah sel tanpa gerakan mundur (backtracking).

    ``entry_point`` (posisi drone) menentukan di ujung mana baris bawah drone
    masuk (kiri atau kanan) — dipilih yang terdekat agar transit efisien.
    ``prefer_far=True`` membalik pilihan itu untuk combinatorial deconfliction.
    """
    if len(polygon) < 3:
        return [poly_centroid(polygon)]

    pts = np.array(polygon, dtype=float)
    min_y, max_y = float(pts[:, 1].min()), float(pts[:, 1].max())
    height = max_y - min_y
    if height < 0.30:
        c = poly_centroid(polygon)
        return [c, c]

    # Inset tepi bawah dan atas (0.35m) berada aman di dalam radius sensor 0.95m
    edge_inset = min(0.40, 0.25 * height)
    y_start = min_y + edge_inset
    y_end = max_y - edge_inset

    n_rows = max(1, int(math.ceil((y_end - y_start) / sweep_spacing)) + 1)
    ys = [0.5 * (min_y + max_y)] if n_rows == 1 else list(np.linspace(y_start, y_end, n_rows))

    levels = []
    for y in ys:
        xs = polygon_scanline_intersections(polygon, y)
        segs = []
        for k in range(0, len(xs) - 1, 2):
            xl, xr = xs[k] + margin, xs[k + 1] - margin
            if obstacles:
                segs.extend(_split_interval_for_obstacles(xl, xr, y, obstacles))
            elif xr - xl >= 0.20:
                segs.append((xl, xr))
        if segs:
            levels.append((y, segs))

    if not levels:
        c = poly_centroid(polygon)
        return [c, c]

    entry = None if entry_point is None else np.asarray(entry_point, float)[:2]

    # Pilih titik mulai baris 0 (kiri atau kanan) berdasarkan entry_point
    y0, segs0 = levels[0]
    xl0, xr0 = segs0[0]
    if entry is not None:
        d_left = float(np.hypot(xl0 - entry[0], y0 - entry[1]))
        d_right = float(np.hypot(xr0 - entry[0], y0 - entry[1]))
        want_right_start = (d_right < d_left) if not prefer_far else (d_right > d_left)
    else:
        want_right_start = False

    flat = []
    cur = None

    for level_idx, (y, segs) in enumerate(levels):
        if level_idx == 0:
            if want_right_start:
                s, e = np.array([xr0, y0]), np.array([xl0, y0])
            else:
                s, e = np.array([xl0, y0]), np.array([xr0, y0])
            flat.extend([s, e])
            cur = e
        else:
            # Sambungkan secara greedy zigzag kontinu dari ujung baris sebelumnya (cur)
            remaining = list(segs)
            while remaining:
                best = None
                for idx, (xl, xr) in enumerate(remaining):
                    for a, b in (((xl, y), (xr, y)), ((xr, y), (xl, y))):
                        d = float(np.hypot(a[0] - cur[0], a[1] - cur[1]))
                        if best is None or d < best[0]:
                            best = (d, idx, np.array(a), np.array(b))
                _, idx, a, b = best
                flat.extend([a, b])
                cur = b
                remaining.pop(idx)

    if obstacles:
        flat = route_around_obstacles(flat, obstacles)

    if start_from_top:
        flat = flat[::-1]

    return [np.asarray(p, dtype=float) for p in flat]
