#!/usr/bin/env python3
"""
Simulator Pemetaan Swarm Drone — Voronoi + Boustrophedon Coverage
Drone bergerak nyata mengikuti jalur zigzag di setiap sel Voronoi.

REVISI v3:
  - Failure recovery sekarang: drone helper SELESAIKAN DULU sisa jalur
    zigzag miliknya sendiri secara utuh (urutan asli, tidak diacak),
    BARU setelah itu mengerjakan titik-titik recovery yang ditempel di
    ujung. Sub-rute recovery-nya sendiri tetap dioptimasi urutannya
    (nearest-neighbor + 2-opt) supaya efisien, tapi tidak pernah
    diselipkan di tengah jalur sendiri.
  - Jumlah drone helper TIDAK selalu 3: default cuma 1 drone terdekat.
    Helper ke-2/ke-3 baru dilibatkan kalau jumlah titik yang perlu
    di-scan ulang melebihi RECOVERY_POINTS_PER_DRONE (area terlalu
    besar untuk satu drone sendirian).
  - Setiap waypoint punya flag `wp_flags` (True = harus tetap di dalam
    sel Voronoi sendiri, False = boleh keluar karena titik recovery),
    dipakai untuk hard-boundary enforcement per-segmen.

Kontrol keyboard:
  1–7 : Toggle drone aktif/mati
  L   : Ganti formasi spawn drone (Grid → Circular → V-Shape)
  B   : Ganti batas wilayah (Rectangle → Circle → Hexagon)
"""
import math
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.path import Path
from matplotlib.patches import Polygon as MplPolygon

# ═══════════════════════════════════════════════
#  GEOMETRI VORONOI
# ═══════════════════════════════════════════════

def clip_voronoi(polygon, pi, pj):
    """Sutherland-Hodgman bisector clipping untuk partisi Voronoi 2D."""
    if not polygon:
        return []
    mid = (pi + pj) / 2.0
    n   = pj - pi
    inside = lambda p: np.dot(p - mid, n) <= 0.0
    def isect(a, b):
        d = b - a
        den = np.dot(d, n)
        return a if abs(den) < 1e-9 else a + np.dot(mid - a, n) / den * d
    out = []
    s = polygon[-1]
    for e in polygon:
        ei, si = inside(e), inside(s)
        if ei:
            if not si: out.append(isect(s, e))
            out.append(e)
        elif si:
            out.append(isect(s, e))
        s = e
    return out


def poly_centroid(pts):
    """Centroid geometris poligon (Shoelace)."""
    p = np.array(pts, dtype=float)
    n = len(p)
    if n < 3:
        return p.mean(axis=0) if n else np.zeros(2)
    A = cx = cy = 0.0
    for i in range(n):
        x0,y0 = p[i]; x1,y1 = p[(i+1)%n]
        f = x0*y1 - x1*y0
        A += f; cx += (x0+x1)*f; cy += (y0+y1)*f
    A *= 0.5
    if abs(A) < 1e-9: return p.mean(axis=0)
    return np.array([cx/(6*A), cy/(6*A)])


def clamp_to_cell(pos, cell_polygon):
    """
    Jika posisi drone berada DI LUAR sel Voronoi-nya, kembalikan ke titik
    terdekat yang masih berada DI DALAM sel (proyeksi ke sisi terdekat).
    """
    if len(cell_polygon) < 3:
        return pos

    poly_path = Path(np.array(cell_polygon))
    if poly_path.contains_point(pos):
        return pos

    best_pt  = None
    best_dist = float('inf')
    pts = [np.array(p, dtype=float) for p in cell_polygon]
    n   = len(pts)
    for i in range(n):
        a = pts[i]
        b = pts[(i+1) % n]
        ab  = b - a
        ab_len2 = ab @ ab
        if ab_len2 < 1e-9:
            closest = a
        else:
            t = max(0.0, min(1.0, (pos - a) @ ab / ab_len2))
            closest = a + t * ab
        d = np.linalg.norm(pos - closest)
        if d < best_dist:
            best_dist = d
            best_pt   = closest

    centroid = poly_centroid(pts)
    inward   = centroid - best_pt
    inward_n = np.linalg.norm(inward)
    eps = 1e-3
    if inward_n > eps:
        best_pt = best_pt + inward / inward_n * eps

    return best_pt


# ═══════════════════════════════════════════════
#  BOUSTROPHEDON PATH PLANNING (ROBUST)
# ═══════════════════════════════════════════════

def polygon_scanline_intersections(polygon, y):
    xs = []
    pts = [np.array(p, dtype=float) for p in polygon]
    n   = len(pts)
    for i in range(n):
        a, b = pts[i], pts[(i+1) % n]
        ya, yb = a[1], b[1]
        if abs(yb - ya) < 1e-9:
            continue
        if not (min(ya,yb) <= y <= max(ya,yb)):
            continue
        t = (y - ya) / (yb - ya)
        x = a[0] + t * (b[0] - a[0])
        xs.append(x)
    xs.sort()
    return xs


def generate_boustrophedon(polygon, sweep_spacing=0.65, margin=0.35, fixed_angle=0.0):
    """
    Jalur Boustrophedon (zigzag lawnmower) untuk poligon arbitrer.
    Setiap segmen hanya dilalui SATU kali → tidak ada path overlap.
    Jika fixed_angle=0.0, seluruh garis Lawnmower dijamin SELALU HORIZONTAL
    (sejajar sumbu X dunia), tidak pernah miring/vertikal/diagonal!
    """
    if len(polygon) < 3:
        return [poly_centroid(polygon)]

    poly = np.array(polygon, dtype=float)
    centroid = poly_centroid(poly)

    if fixed_angle is not None:
        angle = fixed_angle
    else:
        pts_c = poly - centroid
        cov   = pts_c.T @ pts_c / max(len(pts_c) - 1, 1)
        _, evecs = np.linalg.eigh(cov)
        main_axis = evecs[:, 1]
        angle = -math.atan2(main_axis[1], main_axis[0])

    ca, sa = math.cos(angle), math.sin(angle)

    def rot(p):
        p = np.atleast_2d(p).astype(float)
        return np.column_stack([p[:,0]*ca - p[:,1]*sa,
                                p[:,0]*sa + p[:,1]*ca])

    def rot_inv(p):
        p = np.atleast_2d(p).astype(float)
        return np.column_stack([ p[:,0]*ca + p[:,1]*sa,
                                -p[:,0]*sa + p[:,1]*ca])

    poly_r  = rot(poly)
    y_min_r = poly_r[:,1].min()
    y_max_r = poly_r[:,1].max()
    poly_r_list = [tuple(p) for p in poly_r]

    y_start = y_min_r + margin
    y_end   = y_max_r - margin

    if y_start >= y_end:
        ys = np.array([(y_min_r + y_max_r) / 2.0])
    else:
        ys = np.arange(y_start + sweep_spacing * 0.5, y_end, sweep_spacing)
        if len(ys) == 0:
            ys = np.array([(y_start + y_end) / 2.0])

    waypoints_r = []
    go_right    = True

    for y in ys:
        xs = polygon_scanline_intersections(poly_r_list, y)
        for k in range(0, len(xs) - 1, 2):
            x0 = xs[k]   + margin
            x1 = xs[k+1] - margin
            if x1 - x0 < 1e-4:
                xmid = (xs[k] + xs[k+1]) / 2.0
                waypoints_r.append(np.array([xmid, y]))
                waypoints_r.append(np.array([xmid, y]))
                go_right = not go_right
                continue
            if go_right:
                waypoints_r.append(np.array([x0, y]))
                waypoints_r.append(np.array([x1, y]))
            else:
                waypoints_r.append(np.array([x1, y]))
                waypoints_r.append(np.array([x0, y]))
            go_right = not go_right

    if not waypoints_r:
        return [centroid]

    wp_arr   = np.array(waypoints_r)
    wp_world = rot_inv(wp_arr)

    # Scanlines sudah mundur sebesar margin=0.36m dari batas poligon,
    # sehingga wp_world secara alami 100% berada DI DALAM poligon tanpa perlu di-clamp ke centroid!
    path = [np.array(p) for p in wp_world]
    return path


# ═══════════════════════════════════════════════
#  ROUTE OPTIMIZATION (nearest-neighbor + 2-opt)
#  Dipakai untuk menggabungkan sisa jalur sendiri + titik recovery
#  jadi SATU rute yang total jaraknya diminimalkan.
# ═══════════════════════════════════════════════

def optimize_route_indices(start_pos, points, max_2opt_iter=150):
    """
    Kembalikan urutan index (bukan titiknya) dari `points` yang meminimalkan
    total jarak tempuh dimulai dari start_pos.

    Tahap 1: konstruksi awal via nearest-neighbor (greedy tapi FULL —
             semua titik dijamin masuk, bukan cuma pilih 1 lalu berhenti).
    Tahap 2: perbaikan lokal via 2-opt (swap dua edge kalau memperpendek
             total rute) sampai konvergen atau iterasi maksimum tercapai.
    """
    if not points:
        return []
    pts = [np.array(p, dtype=float) for p in points]
    n = len(pts)

    # --- Tahap 1: Nearest-neighbor construction ---
    visited = [False] * n
    order = []
    cur = np.array(start_pos, dtype=float)
    for _ in range(n):
        best_idx, best_d = -1, float('inf')
        for k in range(n):
            if visited[k]:
                continue
            d = np.linalg.norm(pts[k] - cur)
            if d < best_d:
                best_d, best_idx = d, k
        visited[best_idx] = True
        order.append(best_idx)
        cur = pts[best_idx]

    # --- Tahap 2: 2-opt local improvement ---
    def route_len(idx_list):
        total = 0.0
        prev = np.array(start_pos, dtype=float)
        for idx in idx_list:
            total += np.linalg.norm(pts[idx] - prev)
            prev = pts[idx]
        return total

    improved = True
    it = 0
    while improved and it < max_2opt_iter:
        improved = False
        it += 1
        for i in range(len(order) - 1):
            for j in range(i + 1, len(order)):
                new_order = order[:i] + order[i:j+1][::-1] + order[j+1:]
                if route_len(new_order) < route_len(order) - 1e-9:
                    order = new_order
                    improved = True

    return order


# ═══════════════════════════════════════════════
#  SIMULATOR UTAMA
# ═══════════════════════════════════════════════

COLORS = ['#FF1F5B','#00CD6C','#009ADE','#AF58BA','#FFC614','#F28522','#A6761D']

class SwarmSim:

    SPEED    = 1.8
    DT       = 0.05
    SENSOR_R = 0.40
    GRID_N   = 100
    PAUSE_FRAMES = 20       # Jeda pause 1.0 detik (20 frame pada DT=0.05s) saat ada drone mati
    # Ambang batas: berapa titik recovery per-drone.
    # Set ke 6 agar jika waypoint recovery > 6, otomatis dibagi ke 2 atau 3 drone helper!
    RECOVERY_POINTS_PER_DRONE = 6

    def __init__(self):
        self.x_min, self.x_max = 0.0, 10.0
        self.y_min, self.y_max = -5.0,  5.0
        self.nd = 7

        cx, cy, r = 5.0, 0.0, 4.8

        self.bdry_names = ["RECTANGLE", "LARGE RECTANGLE", "CIRCLE", "HEXAGON"]
        self.bdry_presets = [
            np.array([[0,-5],[10,-5],[10,5],[0,5]], dtype=float),
            np.array([[0,-8],[16,-8],[16,8],[0,8]], dtype=float),
            np.array([[cx + r*math.cos(2*math.pi*k/64),
                       cy + r*math.sin(2*math.pi*k/64)] for k in range(64)]),
            np.array([[cx + r*math.cos(2*math.pi*k/6 + math.pi/6),
                       cy + r*math.sin(2*math.pi*k/6 + math.pi/6)] for k in range(6)]),
        ]
        self.bdry_idx = 0
        self.bbox = self.bdry_presets[0].copy()

        self.lay_names = ["GRID", "CIRCULAR", "V-SHAPE"]
        self.layouts = [
            {1:[2,-3],2:[2,3],3:[5,-3.5],4:[5,0],5:[5,3.5],6:[8,-3],7:[8,3]},
            {i+1:[cx+3*math.cos(2*math.pi*i/7), cy+3*math.sin(2*math.pi*i/7)]
             for i in range(7)},
            {1:[2,0],2:[4,-1.8],3:[4,1.8],4:[6,-3.2],
             5:[6,3.2],6:[8,-4.2],7:[8,4.2]},
        ]
        self.layouts = [{k: np.array(v,dtype=float) for k,v in lay.items()}
                        for lay in self.layouts]
        self.lay_idx = 0

        # State drone
        self.active    = {}
        self.collided  = {}
        self.pos       = {}
        self.cells     = {}
        self.paths     = {}    # jalur waypoint (gabungan own + recovery bila ada)
        self.wp_flags  = {}    # paralel dgn paths[i]: True=harus dlm sel sendiri
        self.wp_idx    = {}
        self.seg_s     = {}
        self.seg_e     = {}
        self.seg_t     = {}
        self.history   = {}

        self.cov_grid   = np.zeros((self.GRID_N, self.GRID_N))
        self.needs_plan = True
        self.inited     = False

        self.fig, self.ax = plt.subplots(figsize=(14, 9))
        self.fig.canvas.manager.set_window_title(
            "Swarm Coverage — Voronoi + Boustrophedon (v3: selesaikan sendiri dulu, baru recovery)")
        self.fig.canvas.mpl_connect('key_press_event', self._on_key)

        self._spawn()
        self._setup_plot()

    # ── Reset & spawn ────────────────────────────────────────────────

    def _spawn(self):
        lay = self.layouts[self.lay_idx]
        for i in range(1, self.nd+1):
            self.active[i]   = True
            self.collided[i] = False
            self.pos[i]      = lay[i].copy()
            self.cells[i]    = []
            self.paths[i]        = []
            self.wp_flags[i]     = []
            self.wp_idx[i]       = 0
            self.seg_s[i]        = None
            self.seg_e[i]        = None
            self.seg_t[i]        = 0.0
            self.history[i]      = []
        self.cov_grid.fill(0.0)
        self.needs_plan = True
        self.inited     = False
        self.pause_counter  = 0
        self.newly_died     = []
        self.recovery_mode  = False
        self.pending_recovery_pts = []
        self.merged_dead_comp_polys = []

    # ── Plot setup ───────────────────────────────────────────────────

    def _setup_plot(self):
        ax = self.ax
        ax.clear()
        
        w_span = max(1.0, self.x_max - self.x_min)
        h_span = max(1.0, self.y_max - self.y_min)
        m_x = max(0.8, w_span * 0.05)
        m_y = max(0.8, h_span * 0.05)

        # Alokasikan 26% area kosong di sebelah kanan khusus untuk Sidebar Legend & Status Box
        # agar TIDAK PERNAH saling menutupi atau menumpuk di atas peta!
        ax.set_xlim(self.x_min - m_x, self.x_max + w_span * 0.28)
        ax.set_ylim(self.y_min - m_y, self.y_max + m_y)
        ax.set_aspect('equal')
        ax.grid(True, ls='--', alpha=0.35, color='#cccccc')
        ax.set_facecolor('#f8f9fa')

        ax.set_title(
            "Swarm Quadrotor Coverage Simulator — Centroidal Voronoi + Boustrophedon Recovery\n"
            "[ 1–7: Toggle Drone  |  L: Toggle Layout  |  B: Toggle Boundary ]",
            fontsize=12, pad=10, fontweight='bold', color='#111111')
        ax.set_xlabel("X (meter)"); ax.set_ylabel("Y (meter)")

        self.im_cov = ax.imshow(
            self.cov_grid,
            extent=[self.x_min, self.x_max, self.y_min, self.y_max],
            origin='lower', cmap='YlGn', alpha=0.25,
            zorder=0, vmin=0, vmax=1)

        bv = np.vstack([self.bbox, self.bbox[0]])
        self.h_bdry, = ax.plot(bv[:,0], bv[:,1], 'k-', lw=2.5, zorder=8,
                               label='Batas Wilayah')

        # ── Text Box Coverage (Pojok Kiri Atas Peta) ──────────────────
        self.h_cov_txt = ax.text(
            self.x_min + w_span * 0.02, self.y_max - h_span * 0.03,
            "Coverage: 0.0%",
            fontsize=12, fontweight='bold', color='#155724',
            bbox=dict(fc='#d4edda', alpha=0.92, ec='#155724',
                      boxstyle='round,pad=0.45'), zorder=10)

        # ── Text Box Status Drone (Sidebar Kanan Bawah Legend) ────────
        self.h_stat_txt = ax.text(
            self.x_max + w_span * 0.04, self.y_max - h_span * 0.45,
            "Drone Status:\n—",
            fontsize=8.5, family='monospace', va='top',
            bbox=dict(fc='white', alpha=0.90, ec='#aaaaaa',
                      boxstyle='round,pad=0.45'), zorder=10)

        self.h_plan   = {}
        self.h_trail  = {}
        self.h_marker = {}
        self.h_label  = {}

        for i in range(1, self.nd+1):
            c = COLORS[i-1]
            self.h_plan[i],   = ax.plot([], [], '--', color=c,
                                        lw=1.2, alpha=0.40, zorder=2)
            self.h_trail[i],  = ax.plot([], [], '-',  color=c,
                                        lw=2.2, alpha=0.80, zorder=4)
            self.h_marker[i], = ax.plot([], [], 'o',  color=c, ms=11,
                                        mec='white', mew=1.5,
                                        zorder=6, label=f"Drone {i}")
            self.h_label[i]   = ax.text(0, 0, f"D{i}", fontsize=8,
                                        ha='center', va='bottom',
                                        color=c, fontweight='bold',
                                        zorder=7, visible=False)

        # Legend diletakkan rapi di Sidebar Kanan Atas (TIDAK MENUMPUK DENGAN PETA)
        ax.legend(loc='upper left', bbox_to_anchor=(1.02, 1.0),
                  fontsize=8.5, ncol=2, framealpha=0.90,
                  title=" Keterangan Warna ")
        self.voronoi_patches = []

    # ── Keyboard ────────────────────────────────────────────────────

    def _capture_pending_recovery(self, k):
        """
        Saat drone k mati (dimatikan manual ATAU tabrakan), simpan waypoint
        recovery yang BELUM sempat ia kerjakan (flag False = titik recovery,
        bukan jalur sendiri) supaya tidak hilang begitu saja. Titik ini nanti
        didistribusikan ulang ke drone lain di _handle_failure_recovery().
        """
        pending = []
        if self.wp_idx[k] < len(self.paths[k]):
            for widx in range(self.wp_idx[k], len(self.paths[k])):
                if widx < len(self.wp_flags[k]) and not self.wp_flags[k][widx]:
                    pending.append(self.paths[k][widx])
        if pending:
            self.pending_recovery_pts.extend(pending)
            print(f"  [ORPHAN] Drone {k} mati sambil bawa {len(pending)} titik recovery "
                  f"belum selesai — disimpan, akan didistribusi ulang.")

    def _on_key(self, ev):
        k = ev.key
        if k in [str(n) for n in range(1, self.nd+1)]:
            idx = int(k)
            was_active = self.active[idx]
            self.active[idx] = not self.active[idx]
            if was_active and not self.active[idx]:
                if self.inited:
                    self._capture_pending_recovery(idx)
                    self.newly_died.append(idx)
                    self.pause_counter = self.PAUSE_FRAMES
                    self.recovery_mode = True
                    print(f"[FAILURE] Drone {idx} mati! Pause {self.PAUSE_FRAMES} frame lalu reroute optimal...")
                else:
                    self.needs_plan = True
            elif not was_active and self.active[idx]:
                print(f"[KEY] Drone {idx} → ON")
                self.needs_plan = True
        elif k in ('l','L'):
            self.lay_idx = (self.lay_idx+1) % len(self.layouts)
            print(f"[KEY] Layout → {self.lay_names[self.lay_idx]}")
            self._spawn(); self._setup_plot()
        elif k in ('b','B'):
            self.bdry_idx = (self.bdry_idx+1) % len(self.bdry_presets)
            self.bbox = self.bdry_presets[self.bdry_idx].copy()
            xs = self.bbox[:, 0]
            ys = self.bbox[:, 1]
            self.x_min, self.x_max = xs.min() - 0.5, xs.max() + 0.5
            self.y_min, self.y_max = ys.min() - 0.5, ys.max() + 0.5
            self.cov_grid = np.zeros((self.GRID_N, self.GRID_N))
            print(f"[KEY] Boundary → {self.bdry_names[self.bdry_idx]}")
            self._spawn(); self._setup_plot()

    # ── Path planning (inisialisasi awal) ───────────────────────────

    def _plan(self):
        """Hitung Voronoi + Boustrophedon untuk semua drone aktif (inisialisasi awal)."""
        lay = self.layouts[self.lay_idx]
        gens = {i: lay[i].copy()
                for i in range(1, self.nd+1)
                if self.active[i] and not self.collided[i]}

        # ── Centroidal Voronoi Tessellation (Lloyd's Relaxation) ────────
        # Merelaksasi posisi generator drone secara iteratif ke titik berat (centroid)
        # sel masing-masing. Ini menjamin pembagian sel Voronoi SELALU ADIL (luas
        # area & beban kerja seimbang) untuk bentuk/ukuran boundary apapun!
        if len(gens) > 1:
            for _ in range(12):
                new_gens = {}
                for i in gens:
                    cell_tmp = [np.array(v, dtype=float) for v in self.bbox]
                    for j in gens:
                        if j != i:
                            cell_tmp = clip_voronoi(cell_tmp, gens[i], gens[j])
                    if len(cell_tmp) >= 3:
                        new_gens[i] = poly_centroid(cell_tmp)
                    else:
                        new_gens[i] = gens[i]
                gens = new_gens

        print(f"[PLAN] Centroidal Voronoi (Lloyd) + Boustrophedon untuk {len(gens)} drone...")

        for i in range(1, self.nd+1):
            if i not in gens:
                self.cells[i] = []
                self.paths[i] = []
                self.wp_flags[i] = []
                self.seg_s[i] = None
                continue

            pi = gens[i]

            cell = [np.array(v,dtype=float) for v in self.bbox]
            for j, pj in gens.items():
                if j != i:
                    cell = clip_voronoi(cell, pi, pj)
            self.cells[i] = cell

            if len(cell) < 3:
                self.paths[i] = []
                self.wp_flags[i] = []
                self.seg_s[i] = None
                continue

            spacing = self.SENSOR_R * 1.5
            margin  = self.SENSOR_R * 0.9
            path = generate_boustrophedon(cell, sweep_spacing=spacing, margin=margin)
            self.paths[i] = path
            # Semua waypoint dari _plan() awal WAJIB di dalam sel sendiri
            self.wp_flags[i] = [True] * len(path)
            print(f"  Drone {i}: {len(path)} waypoints")

            if len(path) > 0:
                self.pos[i]     = path[0].copy()
                self.history[i] = [path[0].copy()]

            if len(path) > 1:
                self.wp_idx[i] = 1
                self.seg_s[i]  = path[0].copy()
                self.seg_e[i]  = path[1].copy()
                self.seg_t[i]  = 0.0
            else:
                self.seg_s[i] = None

        self.inited     = True
        self.needs_plan = False

    # ── Failure Recovery (v2: reroute optimal, bukan greedy per-langkah) ──

    def _get_uncovered_cells_in_polygon(self, cell_polygon):
        N  = self.GRID_N
        dx = (self.x_max - self.x_min) / N
        dy = (self.y_max - self.y_min) / N
        poly_path = Path(np.array(cell_polygon))
        uncovered = []
        for r in range(N):
            for c in range(N):
                if self.cov_grid[r, c] == 1.0:
                    continue
                cx2 = self.x_min + (c + 0.5) * dx
                cy2 = self.y_min + (r + 0.5) * dy
                if poly_path.contains_point([cx2, cy2]):
                    uncovered.append(np.array([cx2, cy2]))
        return uncovered

    def _cluster_recovery_points(self, uncovered_pts, sweep_spacing):
        """
        Kelompokkan titik grid uncovered per baris (mengurangi jumlah titik
        yang perlu dioptimasi rute-nya — cukup pakai endpoint kiri/kanan
        tiap baris, karena sensor_radius otomatis cover di antaranya).
        """
        if not uncovered_pts:
            return []
        rows = {}
        for p in uncovered_pts:
            row_key = round(float(p[1]) / sweep_spacing)
            rows.setdefault(row_key, []).append(p)
        pts_out = []
        for yk in sorted(rows.keys()):
            row_pts = sorted(rows[yk], key=lambda p: float(p[0]))
            pts_out.append(np.array(row_pts[0],  dtype=float))
            if len(row_pts) > 1:
                pts_out.append(np.array(row_pts[-1], dtype=float))
        return pts_out

    def _count_cells_in_polygon(self, cell_polygon):
        N  = self.GRID_N
        dx = (self.x_max - self.x_min) / N
        dy = (self.y_max - self.y_min) / N
        poly_path = Path(np.array(cell_polygon))
        count = 0
        for r in range(N):
            for c in range(N):
                cx2 = self.x_min + (c + 0.5) * dx
                cy2 = self.y_min + (r + 0.5) * dy
                if poly_path.contains_point([cx2, cy2]):
                    count += 1
        return count

    def _handle_failure_recovery(self):
        """
        KONSOLIDASI GLOBAL (Global Re-pooling):
        Saat ada event failure baru (meskipun recovery sebelumnya belum selesai),
        sistem mengamankan seluruh sisa tugas recovery yang belum dikunjungi dari
        drone manapun, menggabungkannya dengan sel mati yang baru, lalu
        melakukan redistribusi ulang secara SPASIAL OPTIMAL ke drone aktif tersisa.
        """
        alive = [i for i in range(1, self.nd+1)
                 if self.active[i] and not self.collided[i]]

        if not alive:
            print("[RECOVERY] Tidak ada drone aktif tersisa!")
            self.newly_died.clear()
            self.recovery_mode = False
            return

        spacing = self.SENSOR_R * 1.50   # Overlap 25% (tanpa celah tipis!)
        margin  = self.SENSOR_R * 0.9

        # 1. LUPAKAN / HAPUS seluruh waypoint recovery lama yang belum sempat dikunjungi
        #    (supaya poligon Voronoi sel mati lama & baru bisa DIGABUNGKAN URAIAN ASLI-nya)
        self.pending_recovery_pts = []
        for h in alive:
            # Reset jalur drone h HANYA ke sisa jalur sendiri (wp_flags == True)
            own_wps = []
            own_flags = []
            if self.wp_idx[h] < len(self.paths[h]):
                for idx_w in range(self.wp_idx[h], len(self.paths[h])):
                    if idx_w < len(self.wp_flags[h]) and self.wp_flags[h][idx_w]:
                        own_wps.append(self.paths[h][idx_w])
                        own_flags.append(True)
            new_p = [self.pos[h].copy()] + own_wps
            new_f = [True] + own_flags
            self.paths[h]    = new_p
            self.wp_flags[h] = new_f
            self.wp_idx[h]   = 1
            self.seg_s[h]    = self.pos[h].copy()
            self.seg_e[h]    = new_p[1].copy() if len(new_p) > 1 else None
            self.seg_t[h]    = 0.0

        # 2. HAPUS coverage & GABUNG POLIGON (Polygon Merging via Shapely) untuk seluruh drone mati
        dead_drones = [d for d in range(1, self.nd+1)
                       if not (self.active[d] and not self.collided[d]) and len(self.cells[d]) >= 3]

        all_dead_polys = []
        for dead_idx in dead_drones:
            dead_cell = self.cells[dead_idx]
            N  = self.GRID_N
            dx = (self.x_max - self.x_min) / N
            dy = (self.y_max - self.y_min) / N
            dead_path = Path(np.array(dead_cell))
            erased = 0
            for r in range(N):
                for c in range(N):
                    cx2 = self.x_min + (c + 0.5) * dx
                    cy2 = self.y_min + (r + 0.5) * dy
                    if dead_path.contains_point([cx2, cy2]):
                        if self.cov_grid[r, c] == 1.0:
                            erased += 1
                        self.cov_grid[r, c] = 0.0
            all_dead_polys.append(dead_cell)

        # Merge poligon-poligon sel mati yang saling menempel menggunakan Shapely unary_union
        if all_dead_polys:
            try:
                from shapely.geometry import Polygon as SpPolygon
                from shapely.ops import unary_union
                sp_polys = [SpPolygon(p).buffer(0.05) for p in all_dead_polys if len(p) >= 3]
                merged = unary_union(sp_polys).buffer(-0.05)
                if merged.geom_type == 'Polygon':
                    comp_polys = [np.array(merged.exterior.coords)]
                elif merged.geom_type == 'MultiPolygon':
                    comp_polys = [np.array(p.exterior.coords) for p in merged.geoms]
                else:
                    comp_polys = all_dead_polys
            except Exception as e:
                print(f"[RECOVERY WARN] Shapely merge fallback: {e}")
                comp_polys = all_dead_polys

            self.merged_dead_comp_polys = comp_polys
            for comp in comp_polys:
                full_boust = generate_boustrophedon(comp, sweep_spacing=spacing, margin=margin, fixed_angle=0.0)
                boust_wps = full_boust
                if boust_wps:
                    self.pending_recovery_pts.extend(boust_wps)

        if not self.pending_recovery_pts:
            self.newly_died.clear()
            self.recovery_mode = False
            return

        print(f"[POLYGON MERGING & KONSOLIDASI] Total {len(self.pending_recovery_pts)} waypoint Lawnmower bersambung "
              f"digabungkan & siap didistribusikan ulang ke {len(alive)} drone aktif.")

        def get_helper_end_pos(h):
            if self.seg_s[h] is not None and self.seg_e[h] is not None:
                rem = [self.seg_e[h]] + self.paths[h][self.wp_idx[h]+1:]
                return rem[-1]
            elif self.wp_idx[h] < len(self.paths[h]):
                return self.paths[h][-1]
            return self.pos[h]

        def are_polygons_adjacent(poly1, poly2, tol=0.15):
            if len(poly1) < 3 or len(poly2) < 3:
                return False
            pts1 = np.array(poly1)
            pts2 = np.array(poly2)
            min_d = np.min([np.linalg.norm(p1 - p2) for p1 in pts1 for p2 in pts2])
            if min_d < tol:
                return True
            for i in range(len(pts1)):
                a = pts1[i]
                b = pts1[(i+1)%len(pts1)]
                ab = b - a
                ab2 = np.dot(ab, ab)
                if ab2 < 1e-6: continue
                for p in pts2:
                    t = max(0, min(1, np.dot(p - a, ab) / ab2))
                    proj = a + t * ab
                    if np.linalg.norm(p - proj) < tol:
                        return True
            return False

        # 3. Kumpulkan sel-sel Voronoi dari drone yang mati
        dead_polys = [self.cells[d] for d in range(1, self.nd+1)
                      if not (self.active[d] and not self.collided[d]) and len(self.cells[d]) >= 3]

        # Prioritaskan helper yang sel Voronoi-nya menempel ke salah satu sel mati
        adj_helpers = set()
        for d_poly in dead_polys:
            for h in alive:
                if are_polygons_adjacent(d_poly, self.cells[h]):
                    adj_helpers.add(h)

        candidate_helpers = list(adj_helpers) if adj_helpers else alive
        n_needed = max(1, math.ceil(len(self.pending_recovery_pts) / self.RECOVERY_POINTS_PER_DRONE))
        n_helpers = min(n_needed, 3, len(alive))   # KUNCI: Maksimal HANYA 3 drone helper!

        centroid_global = np.mean(np.array(self.pending_recovery_pts), axis=0)
        dists_to_centroid = [(i, np.linalg.norm(get_helper_end_pos(i) - centroid_global))
                              for i in candidate_helpers]
        dists_to_centroid.sort(key=lambda x: x[1])
        helpers = [idx for idx, _ in dists_to_centroid[:n_helpers]]

        if len(helpers) < n_helpers:
            remaining_alive = [i for i in alive if i not in helpers]
            remaining_alive.sort(key=lambda i: np.linalg.norm(get_helper_end_pos(i) - centroid_global))
            helpers.extend(remaining_alive[:n_helpers - len(helpers)])

        # 4. ALOKASI GARIS LAWNMOWER UTUH + PARALEL SPASIAL (Intact Line Block Alignment):
        #    Mengelompokkan waypoint menjadi BARIS LAWNMOWER UTUH (tidak dipecah jadi titik acak),
        #    lalu mengurutkan blok baris & posisi helper secara paralel sepanjang aksis dominan.
        #    Hasilnya: 100% GARIS LAWNMOWER RAPI & SEJAJAR, TANPA RUTE BERSILANGAN!
        boust_wps = self.pending_recovery_pts
        lines = []
        for i in range(0, len(boust_wps)-1, 2):
            lines.append((boust_wps[i], boust_wps[i+1]))
        if len(boust_wps) % 2 != 0:
            lines.append((boust_wps[-1], boust_wps[-1]))

        n_h = len(helpers)
        if lines and n_h > 0:
            block_sz = math.ceil(len(lines) / n_h)
            blocks = [lines[k*block_sz : (k+1)*block_sz] for k in range(n_h) if lines[k*block_sz : (k+1)*block_sz]]

            block_centers = [np.mean([pt for line in b for pt in line], axis=0) for b in blocks]
            variances = np.var(block_centers, axis=0) if len(block_centers) > 1 else [0, 1]
            axis_idx = 1 if len(variances) > 1 and variances[1] >= variances[0] else 0

            # Urutkan blok baris sepanjang aksis dominan
            sorted_blocks = [b for _, b in sorted(zip([c[axis_idx] for c in block_centers], blocks))]

            # Urutkan helper sepanjang aksis dominan yang SAMA
            sorted_helpers = sorted(helpers, key=lambda h: get_helper_end_pos(h)[axis_idx])

            for h, b in zip(sorted_helpers, sorted_blocks):
                if not b:
                    continue
                h_end = get_helper_end_pos(h)
                # 1. Tentukan garis mana di dalam blok yang lebih dekat dengan h_end (garis pertama vs terakhir)
                first_line = b[0]
                last_line  = b[-1]
                d_first = min(np.linalg.norm(h_end - first_line[0]), np.linalg.norm(h_end - first_line[1]))
                d_last  = min(np.linalg.norm(h_end - last_line[0]),  np.linalg.norm(h_end - last_line[1]))
                
                lines_in_order = list(reversed(b)) if d_last < d_first else list(b)

                # 2. Tentukan arah sapuan kiri/kanan pada garis pertama (pilih titik awal terdekat dari h_end)
                entry_line = lines_in_order[0]
                # p_left adalah titik dengan X lebih kecil, p_right titik dengan X lebih besar
                if entry_line[0][0] <= entry_line[1][0]:
                    p_left, p_right = entry_line[0], entry_line[1]
                else:
                    p_left, p_right = entry_line[1], entry_line[0]

                start_from_left = np.linalg.norm(h_end - p_left) <= np.linalg.norm(h_end - p_right)

                # 3. Susun pola Lawnmower selang-seling (zigzag) yang mulus
                rec_wps_h = []
                go_left_to_right = start_from_left
                for l_start, l_end in lines_in_order:
                    # Pastikan p_a dan p_b terurut kiri-kanan
                    pa = l_start if l_start[0] <= l_end[0] else l_end
                    pb = l_end   if l_start[0] <= l_end[0] else l_start
                    if go_left_to_right:
                        rec_wps_h.extend([pa, pb])
                    else:
                        rec_wps_h.extend([pb, pa])
                    go_left_to_right = not go_left_to_right

                if rec_wps_h:
                    self._append_recovery_boustrophedon(h, rec_wps_h)

        self.pending_recovery_pts = []
        self.newly_died.clear()
        self.recovery_mode = False

    def _append_recovery_boustrophedon(self, h, rec_wps):
        """
        Tempel jalur recovery berpola Lawnmower ke sisa jalur sendiri drone h.
        Menggunakan transisi siku-siku (orthogonal boundary entry) agar penerbangan
        helper dari posisinya ke area recovery tidak memotong miring melintasi peta.
        """
        remaining_own = []
        if self.seg_s[h] is not None and self.seg_e[h] is not None:
            remaining_own.append(self.seg_e[h])
            remaining_own.extend(self.paths[h][self.wp_idx[h]+1:])
        elif self.wp_idx[h] < len(self.paths[h]):
            remaining_own.extend(self.paths[h][self.wp_idx[h]:])

        if not rec_wps:
            return

        ordered_rec = list(rec_wps)

        new_path  = [self.pos[h].copy()] + list(remaining_own) + [np.array(p) for p in ordered_rec]
        new_flags = ([True] * (1 + len(remaining_own)) + [False] * len(ordered_rec))

        self.paths[h]    = new_path
        self.wp_flags[h] = new_flags
        self.wp_idx[h]   = 1
        self.seg_s[h]    = self.pos[h].copy()
        self.seg_e[h]    = new_path[1].copy() if len(new_path) > 1 else None
        self.seg_t[h]    = 0.0

        print(f"  Drone {h}: +{len(ordered_rec)} waypoint Lawnmower recovery (ditempel di akhir jalur sendiri).")


    # ── Coverage ─────────────────────────────────────────────────────

    def _update_cov(self):
        N  = self.GRID_N
        dx = (self.x_max - self.x_min) / N
        dy = (self.y_max - self.y_min) / N
        bp = Path(self.bbox)
        r2 = self.SENSOR_R**2
        rx = int(self.SENSOR_R/dx) + 1
        ry = int(self.SENSOR_R/dy) + 1

        for i in range(1, self.nd+1):
            if not (self.active[i] and not self.collided[i]):
                continue
            xd, yd = self.pos[i]
            col = max(0, min(N-1, int((xd - self.x_min)/dx)))
            row = max(0, min(N-1, int((yd - self.y_min)/dy)))
            for r in range(max(0,row-ry), min(N,row+ry+1)):
                for c in range(max(0,col-rx), min(N,col+rx+1)):
                    cx2 = self.x_min + (c+0.5)*dx
                    cy2 = self.y_min + (r+0.5)*dy
                    if (cx2-xd)**2+(cy2-yd)**2 <= r2:
                        if bp.contains_point([cx2,cy2]):
                            self.cov_grid[r,c] = 1.0

    def _cov_pct(self):
        N  = self.GRID_N
        dx = (self.x_max - self.x_min) / N
        dy = (self.y_max - self.y_min) / N
        bp = Path(self.bbox)
        tot = mapped = 0
        for r in range(N):
            for c in range(N):
                pt = [self.x_min+(c+0.5)*dx, self.y_min+(r+0.5)*dy]
                if bp.contains_point(pt):
                    tot += 1
                    if self.cov_grid[r,c] == 1.0:
                        mapped += 1
        return mapped/tot*100 if tot else 0.0

    # ── Main step ────────────────────────────────────────────────────

    def step(self, frame):
        for p in self.voronoi_patches:
            try: p.remove()
            except: pass
        self.voronoi_patches = []

        if self.needs_plan:
            self._plan()

        if self.pause_counter > 0:
            self.pause_counter -= 1
            self.h_cov_txt.set_text(
                f"[PAUSE] Reroute optimal dalam {self.pause_counter} frame...")
            for i in range(1, self.nd+1):
                x, y = self.pos[i]
                self.h_marker[i].set_data([x], [y])
                self.h_label[i].set_position((x, y+0.18))
                self.h_label[i].set_visible(True)
                if not (self.active[i] and not self.collided[i]):
                    self.h_marker[i].set_color('#cc0000' if self.collided[i] else '#aaaaaa')
                    self.h_marker[i].set_marker('X' if self.collided[i] else 'o')
            self.fig.canvas.draw_idle()
            return

        if self.recovery_mode and self.pause_counter == 0:
            self._handle_failure_recovery()

        # Deteksi tabrakan
        for i in range(1, self.nd+1):
            if not (self.active[i] and not self.collided[i]): continue
            for j in range(i+1, self.nd+1):
                if not (self.active[j] and not self.collided[j]): continue
                if np.linalg.norm(self.pos[i]-self.pos[j]) < 0.28:
                    self.collided[i] = self.collided[j] = True
                    # PENTING: simpan dulu titik recovery yang belum sempat
                    # dikerjakan SEBELUM seg_s direset, lalu picu recovery —
                    # dulu tabrakan tidak memicu ini sama sekali, sehingga
                    # area yang jadi tanggung jawab drone yg tabrakan hilang
                    # permanen (coverage macet tidak pernah 100%).
                    self._capture_pending_recovery(i)
                    self._capture_pending_recovery(j)
                    self.seg_s[i] = self.seg_s[j] = None
                    self.newly_died.extend([i, j])
                    self.pause_counter = self.PAUSE_FRAMES
                    self.recovery_mode = True
                    print(f"[COLLISION] D{i} ↔ D{j} — area & tugas recovery masing-masing "
                          f"akan didistribusi ulang ke drone lain")

        self._update_cov()
        self.im_cov.set_data(self.cov_grid)

        if frame % 5 == 0:
            pct = self._cov_pct()
            self.h_cov_txt.set_text(f"Coverage: {pct:.1f}%")

        st = "Drone Status:\n"
        for i in range(1, self.nd+1):
            if self.collided[i]:    st += f" D{i}: CRASH  \n"
            elif not self.active[i]: st += f" D{i}: OFF    \n"
            elif self.seg_s[i] is None:
                                    st += f" D{i}: DONE ✓ \n"
            else:
                p = self.pos[i]
                st += f" D{i}: ({p[0]:.1f},{p[1]:.1f})\n"
        self.h_stat_txt.set_text(st)

        # Gambar poligon Voronoi GABUNGAN sel mati (Shapely Merged Polygon) dengan warna merah terbayang
        if hasattr(self, 'merged_dead_comp_polys') and self.merged_dead_comp_polys:
            for m_poly in self.merged_dead_comp_polys:
                if len(m_poly) >= 3:
                    pat_m = MplPolygon(np.array(m_poly), closed=True, fc='#ff0000',
                                       alpha=0.15, ec='#cc0000', lw=2.0, ls='--', zorder=1.5)
                    self.ax.add_patch(pat_m)
                    self.voronoi_patches.append(pat_m)

        # ── Per-drone update ──────────────────────────────────────
        for i in range(1, self.nd+1):
            c = COLORS[i-1]
            op = self.active[i] and not self.collided[i]

            if op and len(self.cells[i]) >= 3:
                pa = np.array(self.cells[i])
                pat = MplPolygon(pa, closed=True, fc=c,
                                 alpha=0.09, ec='#555',
                                 lw=1.0, ls=':', zorder=1)
                self.ax.add_patch(pat)
                self.voronoi_patches.append(pat)

            if op and len(self.paths[i]) > 1:
                pp = np.array(self.paths[i])
                self.h_plan[i].set_data(pp[:,0], pp[:,1])
            else:
                self.h_plan[i].set_data([], [])

            if not op:
                x, y = self.pos[i]
                self.h_marker[i].set_data([x], [y])
                self.h_marker[i].set_color('#aaaaaa' if not self.collided[i] else '#cc0000')
                self.h_marker[i].set_marker('X' if self.collided[i] else 'o')
                self.h_label[i].set_position((x, y+0.15))
                self.h_label[i].set_visible(True)
                self.h_trail[i].set_data([], [])
                continue

            # ── Gerakan kinematika (ikuti self.paths[i] SATU rute tunggal) ──
            if self.seg_s[i] is not None and self.seg_e[i] is not None:
                P0 = np.array(self.seg_s[i])
                P1 = np.array(self.seg_e[i])
                d  = np.linalg.norm(P1 - P0)

                dt_t = (self.SPEED * self.DT / d) if d > 1e-4 else 1.0
                self.seg_t[i] = min(self.seg_t[i] + dt_t, 1.0)
                t = self.seg_t[i]

                self.pos[i] = (1-t)*P0 + t*P1

                if t >= 1.0:
                    self.pos[i] = P1.copy()
                    self.wp_idx[i] += 1
                    if self.wp_idx[i] < len(self.paths[i]):
                        self.seg_s[i] = self.pos[i].copy()
                        self.seg_e[i] = self.paths[i][self.wp_idx[i]].copy()
                        self.seg_t[i] = 0.0
                    else:
                        self.seg_s[i] = None
                        print(f"[DONE] Drone {i} selesai semua tugas (termasuk recovery bila ada).")

            # ── HARD BOUNDARY ENFORCEMENT (FIX: Per-waypoint & status selesai) ────
            # Clamp HANYA berlaku jika waypoint saat ini bertipe True (di dalam sel sendiri).
            # Jika waypoint habis (DONE), cek flag waypoint terakhir. Jika terakhir recovery (False),
            # JANGAN dikunci/clamp agar tidak loncat kembali ke sel sendiri!
            enforce_clamp = False
            if self.wp_idx[i] < len(self.wp_flags[i]):
                enforce_clamp = self.wp_flags[i][self.wp_idx[i]]
            if enforce_clamp and len(self.cells[i]) >= 3:
                clamped = clamp_to_cell(self.pos[i], self.cells[i])
                if not np.allclose(clamped, self.pos[i], atol=1e-4):
                    self.pos[i] = clamped

            self.history[i].append(self.pos[i].copy())
            x, y = self.pos[i]
            self.h_marker[i].set_data([x], [y])
            self.h_marker[i].set_color(c)
            self.h_marker[i].set_marker('o')
            self.h_label[i].set_position((x, y+0.18))
            self.h_label[i].set_visible(True)

            if len(self.history[i]) > 1:
                hist = np.array(self.history[i])
                self.h_trail[i].set_data(hist[:,0], hist[:,1])

        self.fig.canvas.draw_idle()


# ════════════════════════════════════════════════
def main():
    sim = SwarmSim()
    ani = animation.FuncAnimation(
        sim.fig,
        sim.step,
        frames=20000,
        interval=50,
        blit=False,
        repeat=False
    )
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()