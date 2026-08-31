"""Deteksi rintangan dari LiDAR 2D + peta rintangan yang dibangun sambil terbang.

Kenapa modul ini ada
--------------------
Sebelumnya rintangan diambil dari tabel koordinat yang sudah diketahui:
perencana memecah baris di koordinat itu, dan QP menerima daftar yang sama.
LiDAR pada model drone ditulis oleh callback-nya lalu TIDAK PERNAH dibaca —
`lidar_ranges` adalah data mati. Artinya Skema 3 sebenarnya "penghindaran
rintangan dengan peta sempurna", dan paper tidak boleh mengklaim penginderaan.

Modul ini menggantikan tabel itu: rintangan DITEMUKAN dari pantulan LiDAR,
diakumulasi ke satu peta bersama, lalu dipakai baik oleh QP maupun perencana.

Murni numpy, tanpa ROS, supaya bisa diuji terhadap scan rekaman.

Batas yang disengaja
--------------------
Yang TIDAK dianggap pengetahuan a-priori (dan memang tersedia di kawanan nyata):
posisi drone lain (odometri kawanan) dan batas arena. Keduanya dipakai untuk
membuang pantulan dari sesama drone dan dari dinding — bukan untuk mengetahui
rintangan. Radius dan pusat silinder sepenuhnya diperkirakan dari data.
"""
import numpy as np

# Pantulan yang lebih jauh dari ini diabaikan: pada jarak besar satu silinder
# hanya menyisakan 3-4 sinar, dan galat pemusatannya jadi lebih besar daripada
# manfaatnya. Zona aman perencana 1.30 m toh jauh lebih kecil.
MAX_USE_RANGE = 8.0

# Dua pantulan berurutan dianggap satu benda bila lompatan jaraknya di bawah
# ini. Silinder r=0.40 m pada 5 m memberi ~9 sinar dengan spasi ~0.09 m.
CLUSTER_GAP = 0.45

# Buang pantulan sedekat ini ke drone lain (radius bodi 0.22 m + margin).
DRONE_MASK_R = 0.55

# Buang pantulan sedekat ini ke batas arena (dinding & pagar grid).
WALL_MARGIN = 0.80

# Radius silinder yang masuk akal; hasil fit di luar rentang ini ditolak.
R_MIN, R_MAX = 0.15, 1.20


def scan_to_points(ranges, angle_min, angle_inc, x, y, yaw,
                   range_max=MAX_USE_RANGE):
    """Pantulan LiDAR -> titik-titik dunia (N, 2)."""
    r = np.asarray(ranges, dtype=float)
    n = r.size
    if n == 0:
        return np.empty((0, 2))
    ok = np.isfinite(r) & (r > 0.05) & (r < range_max)
    if not ok.any():
        return np.empty((0, 2))
    a = angle_min + angle_inc * np.arange(n)[ok] + yaw
    d = r[ok]
    return np.column_stack((x + d * np.cos(a), y + d * np.sin(a)))


def _reject(pts, others, arena):
    """Buang pantulan dari drone lain dan dari dinding arena."""
    if pts.shape[0] == 0:
        return pts
    keep = np.ones(pts.shape[0], dtype=bool)
    if others is not None and len(others):
        o = np.asarray(others, dtype=float).reshape(-1, 2)
        d = np.linalg.norm(pts[:, None, :] - o[None, :, :], axis=2)
        keep &= (d.min(axis=1) > DRONE_MASK_R)
    if arena is not None:
        x0, y0, x1, y1 = arena
        keep &= ((pts[:, 0] > x0 + WALL_MARGIN) & (pts[:, 0] < x1 - WALL_MARGIN) &
                 (pts[:, 1] > y0 + WALL_MARGIN) & (pts[:, 1] < y1 - WALL_MARGIN))
    return pts[keep]


def _clusters(pts, gap=CLUSTER_GAP):
    """Kelompokkan titik berurutan-sudut menjadi benda-benda terpisah."""
    if pts.shape[0] == 0:
        return []
    step = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    cut = np.flatnonzero(step > gap) + 1
    return [c for c in np.split(pts, cut) if c.shape[0] >= 2]


def fit_circle(pts, sensor_xy):
    """Perkirakan (pusat, radius) satu benda dari busur pantulannya.

    LiDAR hanya melihat sisi DEKAT silinder, jadi centroid pantulan selalu
    bias ke arah sensor. Dua jalur:

    * >= 4 titik dengan busur cukup melengkung -> kuadrat terkecil aljabar
      (Kasa). Lingkaran lewat titik-titik itu, pusatnya keluar dengan sendirinya.
    * selain itu -> mundur ke geometri tali busur: setengah-lebar tali `h`
      memberi radius minimal, lalu pusat digeser MENJAUHI sensor sejauh
      sisa jari-jarinya. Tanpa ini benda jauh selalu diperkirakan terlalu dekat.
    """
    p = np.asarray(pts, dtype=float)
    s = np.asarray(sensor_xy, dtype=float)
    mid = p.mean(axis=0)
    view = mid - s
    nv = float(np.linalg.norm(view))
    view = view / nv if nv > 1e-9 else np.array([1.0, 0.0])
    half_chord = 0.5 * float(np.linalg.norm(p[-1] - p[0]))

    if p.shape[0] >= 4:
        A = np.column_stack((2.0 * p[:, 0], 2.0 * p[:, 1], np.ones(p.shape[0])))
        b = (p ** 2).sum(axis=1)
        try:
            sol, *_ = np.linalg.lstsq(A, b, rcond=None)
            cx, cy, c = sol
            rad2 = c + cx * cx + cy * cy
            if rad2 > 0:
                r = float(np.sqrt(rad2))
                ctr = np.array([cx, cy])
                # Fit aljabar bisa meleset jauh pada busur pendek; terima hanya
                # bila masuk akal DAN pusatnya di sisi jauh pantulan.
                if R_MIN <= r <= R_MAX and float(np.dot(ctr - mid, view)) > -0.05:
                    return ctr, r
        except np.linalg.LinAlgError:
            pass

    r = float(np.clip(max(half_chord, R_MIN), R_MIN, R_MAX))
    depth = float(np.sqrt(max(r * r - half_chord * half_chord, 0.0)))
    return mid + view * depth, r


def detect(ranges, angle_min, angle_inc, x, y, yaw,
           others=None, arena=None, range_max=MAX_USE_RANGE):
    """Satu scan -> daftar ``(pusat_xy, radius)`` kandidat rintangan."""
    pts = scan_to_points(ranges, angle_min, angle_inc, x, y, yaw, range_max)
    pts = _reject(pts, others, arena)
    out = []
    for c in _clusters(pts):
        span = float(np.linalg.norm(c[-1] - c[0]))
        if span > 2.0 * R_MAX:          # terlalu lebar untuk sebuah silinder
            continue
        ctr, r = fit_circle(c, (x, y))
        out.append((ctr, r))
    return out


class ObstacleMap:
    """Peta rintangan bersama yang dibangun sambil terbang.

    Satu deteksi tunggal tidak cukup: pantulan tepi dan sesama drone yang lolos
    saringan akan memunculkan benda hantu. Sebuah jalur baru menjadi
    TERKONFIRMASI hanya setelah dilihat ``min_hits`` kali; pusat dan radiusnya
    dirata-rata berjalan supaya derau satu frame tidak menggeser peta.
    """

    def __init__(self, assoc_dist=1.20, min_hits=4, radius_pad=0.05,
                 move_reject=0.70, move_strikes=3):
        self.assoc_dist = float(assoc_dist)
        self.min_hits = int(min_hits)
        self.radius_pad = float(radius_pad)
        self.move_reject = float(move_reject)
        self.move_strikes = int(move_strikes)
        self._tracks = []   # {'c', 'r', 'n', 'c0', 'out', 'moving'}

    def update(self, detections):
        """Gabungkan deteksi satu scan ke peta. Kembalikan jumlah jalur baru."""
        added = 0
        for ctr, r in detections:
            ctr = np.asarray(ctr, dtype=float)
            best, bd = None, self.assoc_dist
            for t in self._tracks:
                d = float(np.linalg.norm(t['c'] - ctr))
                if d < bd:
                    best, bd = t, d
            if best is None:
                self._tracks.append({'c': ctr.copy(), 'r': float(r), 'n': 1,
                                     'c0': ctr.copy(), 'out': 0,
                                     'moving': False})
                added += 1
                continue
            w = 1.0 / (best['n'] + 1.0)
            best['c'] = (1.0 - w) * best['c'] + w * ctr
            best['r'] = (1.0 - w) * best['r'] + w * float(r)
            best['n'] += 1

            # Jangkar awal = rata-rata `min_hits` deteksi PERTAMA, bukan satu
            # deteksi pertama. Satu sampel awal yang meleset akan menggeser
            # jangkar dan membuat seluruh jalur salah dinilai.
            if best['n'] <= self.min_hits:
                k = best['n']
                best['c0'] = ((k - 1) * best['c0'] + ctr) / k

            # BENDA YANG BERPINDAH BUKAN RINTANGAN STATIS. Drone melaju
            # 1.6 m/s hanya bergeser 0.16 m antar-scan pada 10 Hz — jauh di
            # bawah radius asosiasi — jadi tanpa uji ini ia terasosiasi ke
            # jalur yang sama, mengumpulkan hit, lalu dikonfirmasi sebagai
            # silinder.
            #
            # Kriterianya PENCILAN BERTURUT-TURUT, dan dua alternatif yang
            # lebih sederhana sudah terbukti gagal:
            #
            #  * uji `ctr` mentah sekali-lewat -> bukan uji gerak melainkan uji
            #    "pernahkah ada satu pengukuran buruk". Dari 2089 deteksi sebuah
            #    silinder, satu pencilan memvonisnya selamanya. Terukur di misi
            #    31 Agu: kesembilan silinder terlacak dengan koordinat tepat dan
            #    ribuan hit, tapi LIMA dicoret sebagai "bergerak" padahal drift
            #    rata-ratanya hanya 0.1-0.6 m — QP tak pernah diberi tahu dan
            #    enam drone menabrak.
            #  * uji terhadap rata-rata -> benda bergerak lari lebih cepat
            #    daripada rata-ratanya sendiri, melewati `assoc_dist`, lalu
            #    PECAH jadi jalur-jalur baru yang masing-masing lolos.
            #
            # Benda yang benar-benar berpindah menjauh dari jangkarnya secara
            # BERUNTUN; derau menyimpang sesekali lalu kembali.
            if float(np.linalg.norm(ctr - best['c0'])) > self.move_reject:
                best['out'] += 1
                if best['out'] >= self.move_strikes:
                    best['moving'] = True
            else:
                best['out'] = 0
        return added

    def confirmed(self):
        """Rintangan yang sudah cukup sering terlihat.

        Bentuk kembalian ``(id, x, y, radius, tinggi, warna)`` sengaja sama
        dengan tabel lama, supaya perencana dan QP tidak perlu tahu dari mana
        rintangan itu berasal.
        """
        out = []
        for i, t in enumerate(self._tracks):
            if t['n'] >= self.min_hits and not t['moving']:
                out.append((900 + i, float(t['c'][0]), float(t['c'][1]),
                            float(t['r']) + self.radius_pad, 4.0,
                            (0.95, 0.45, 0.1)))
        return out

    def dump(self, limit=12):
        """Ringkasan jalur untuk diagnosis: di MANA jalur-jalur itu berada.

        Jumlah saja tidak cukup. Sebuah misi menghasilkan 132 jalur dengan 97
        ditolak-bergerak sementara kesembilan silinder TIDAK satu pun masuk
        peta; tanpa posisinya mustahil tahu benda apa yang sebenarnya terdeteksi.
        """
        rows = sorted(self._tracks, key=lambda t: -t['n'])[:limit]
        return [(round(float(t['c'][0]), 2), round(float(t['c'][1]), 2),
                 round(float(t['r']), 2), t['n'],
                 round(float(np.linalg.norm(t['c'] - t['c0'])), 2), t['moving'])
                for t in rows]

    def n_tracks(self):
        """(total, terkonfirmasi, ditolak-karena-bergerak)."""
        return (len(self._tracks),
                sum(1 for t in self._tracks
                    if t['n'] >= self.min_hits and not t['moving']),
                sum(1 for t in self._tracks if t['moving']))
