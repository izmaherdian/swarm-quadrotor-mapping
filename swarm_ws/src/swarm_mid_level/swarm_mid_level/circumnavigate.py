"""Manuver mengitari rintangan berjari-jari TETAP, lalu kembali ke baris.

Kenapa ini ada, padahal sudah ada CBF-QP
---------------------------------------
QP mencegah tabrakan, dan itu terbukti. Tapi QP adalah PROYEKSI: ia memangkas
kecepatan yang diusulkan sampai aman, dan bentuk lintasan yang keluar adalah
apa pun yang tersisa dari pemangkasan itu — bukan lingkaran, bukan sesuatu yang
jaraknya tetap. Hasilnya di simulasi terlihat sebagai belokan tak beraturan.

Selain itu penghindaran murni reaktif TIDAK BISA menuntaskan baris yang
tertutup rintangan: QP hanya melarang mendekat, tidak pernah menyarankan lewat
sebelah mana. Terukur 1 Sep: iris_2 berhenti 0.80 m dari silinder di tengah
Baris 8, cakupan mandek 92.3%, misi kena timeout.

Modul ini mengisi celah itu dengan MANUVER, bukan dengan perencanaan ulang:
baris boustrophedon tidak pernah disentuh. Drone menyimpang, mengitari pada
jarak tetap, lalu bergabung kembali ke baris yang SAMA di titik tempat ia
menyimpang. Perencanaan tetap urusan high level; ini murni cara mengeksekusi
satu baris yang kebetulan terhalang.

Pembagian tugas dengan QP tetap utuh: modul ini hanya MENGUSULKAN kecepatan.
QP tetap yang memutuskan, dan tetap menjadi jaring pengaman terakhir.
"""
import math

import numpy as np

# Bantalan di atas radius rintangan HASIL DETEKSI. Jari-jari edar =
# r_terdeteksi + bantalan ini. Adaptif, karena LiDAR menaksir radius silinder
# 0.40 m di rentang 0.34-0.56 m; memakai satu angka tetap akan terlalu ketat
# pada taksiran besar dan boros pada taksiran kecil.
ORBIT_PAD = 0.90

# Mulai mengitari ketika rintangan berada dalam jarak ini DI DEPAN, diukur
# sepanjang baris. Lebih jauh dari ini tidak perlu — baris masih lapang.
TRIGGER_AHEAD = 3.0

# Setengah-lebar koridor baris. Rintangan yang lebih jauh dari ini dari garis
# baris tidak menghalangi: drone lewat begitu saja dan QP menangani sisanya.
CORRIDOR_HALF = 1.10

# Margin melewati pusat rintangan sebelum manuver dianggap selesai. HARUS
# kecil: drone mengedar pada jari-jari r+ORBIT_PAD, jadi jangkauan majunya
# sepanjang baris tidak pernah melebihi jari-jari itu. Ambang 1.20 m dulu
# membuat syarat keluar mustahil dipenuhi dan drone mengorbit 4.4 putaran
# tanpa henti.
EXIT_PAST = 0.15

# Pengaman: kalau sudah memutar sebanyak ini dan belum juga keluar, ada yang
# salah pada geometrinya. Serahkan kembali ke baris dan biarkan QP bekerja —
# lebih baik daripada mengorbit selamanya.
MAX_SWEEP_DEG = 300.0


def _unit(v):
    n = float(np.linalg.norm(v))
    return (v / n, n) if n > 1e-9 else (np.array([1.0, 0.0]), 0.0)


def blocking_obstacle(pos, wp_start, wp_end, obstacles,
                      corridor=CORRIDOR_HALF, ahead=TRIGGER_AHEAD):
    """Rintangan yang benar-benar menghalangi sisa baris ini, atau None.

    Kriterianya dua-duanya harus benar:
      * pusatnya berada dalam koridor baris (jarak tegak lurus < ``corridor``),
      * dan berada DI DEPAN drone sepanjang baris, dalam jarak ``ahead``.

    Rintangan di samping baris atau yang sudah terlewat tidak menghalangi apa
    pun, dan memicu manuver untuknya persis yang membuat drone "berputar
    padahal tidak ada apa-apa".
    """
    u, line_len = _unit(np.asarray(wp_end, float) - np.asarray(wp_start, float))
    if line_len < 1e-6:
        return None
    p = np.asarray(pos, float)[:2]
    s_self = float(np.dot(p - wp_start, u))
    best, best_s = None, float('inf')
    for obs in obstacles or ():
        c = np.array([obs[1], obs[2]], dtype=float)
        d = c - wp_start
        s = float(np.dot(d, u))
        lat = float(np.linalg.norm(d - s * u))
        if lat > corridor + obs[3]:
            continue                       # di samping baris, bukan penghalang
        gap = s - s_self
        if gap < -obs[3] or gap > ahead:
            continue                       # sudah terlewat, atau masih jauh
        if s < best_s:
            best, best_s = obs, s
    return best


class Circumnavigator:
    """Satu manuver mengitari per drone, dengan sisi yang DIKUNCI.

    Sisi (kiri/kanan) dipilih sekali saat manuver dimulai lalu dikunci sampai
    selesai. Tanpa penguncian, drone bisa berganti pikiran di tengah dan
    berayun bolak-balik di depan rintangan — kegagalan yang sama seperti pada
    pemecah kebuntuan V2V.
    """

    def __init__(self, orbit_pad=ORBIT_PAD):
        self.orbit_pad = float(orbit_pad)
        self._act = {}     # aid -> dict(obs_id, c, R, sign, a_start, a_prev, sweep)

    def reset(self, aid):
        self._act.pop(aid, None)

    def active(self, aid):
        return aid in self._act

    def _pick_side(self, pos, u, c, cell_poly):
        """Kiri atau kanan. Bila sel diberikan, pilih sisi yang tetap di DALAM sel.

        Ujung baris yang berhimpit dengan rintangan adalah kasus yang paling
        mudah salah: memutar ke sisi luar akan membawa drone keluar dari sel
        Voronoi-nya dan masuk wilayah drone lain.
        """
        n = np.array([-u[1], u[0]])           # normal kiri
        cand = []
        for sign in (+1.0, -1.0):
            probe = c + sign * n * (self.orbit_pad + 0.3)
            score = 0.0
            if cell_poly is not None:
                score += 0.0 if _inside(probe, cell_poly) else 100.0
            score += float(np.linalg.norm(probe - pos)) * 0.01
            cand.append((score, sign))
        cand.sort()
        return cand[0][1]

    def start(self, aid, pos, u_line, obs, cell_poly=None):
        c = np.array([obs[1], obs[2]], dtype=float)
        R = float(obs[3]) + self.orbit_pad
        sign = self._pick_side(np.asarray(pos, float)[:2], u_line, c, cell_poly)
        a0 = math.atan2(pos[1] - c[1], pos[0] - c[0])
        self._act[aid] = {'oid': obs[0], 'c': c, 'R': R, 'sign': sign,
                          'a_prev': a0, 'sweep': 0.0}

    def step(self, aid, pos, speed):
        """Kecepatan yang diusulkan untuk tetap di lingkaran berjari-jari R.

        Dua suku: tangensial (mengelilingi) dan radial (mengoreksi jari-jari).
        Suku radial itu yang membuat jaraknya BENAR-BENAR tetap, bukan sekadar
        melengkung sekenanya.
        """
        st = self._act[aid]
        d = np.asarray(pos, float)[:2] - st['c']
        n_hat, r = _unit(d)
        t_hat = np.array([-n_hat[1], n_hat[0]]) * st['sign']

        a = math.atan2(d[1], d[0])
        da = (a - st['a_prev'] + math.pi) % (2.0 * math.pi) - math.pi
        st['sweep'] += da * st['sign']
        st['a_prev'] = a

        # Suku radial memakai gain tinggi (4.0) karena inilah yang membuat
        # jaraknya BENAR-BENAR tetap. Dengan gain 1.2 jari-jari mengendap
        # 0.12 m di luar target — melengkung rapi, tapi bukan pada jarak yang
        # diminta.
        v_t = t_hat * speed
        v_r = -n_hat * float(np.clip((r - st['R']) * 4.0, -1.0, 1.0))
        return v_t + v_r

    def should_exit(self, aid, pos, wp_start, u_line, obstacles):
        """Selesai hanya setelah drone benar-benar MELEWATI rintangannya.

        Kriteria "jalur di depan bersih" saja tidak cukup: pada jarak 1.23 m
        SEBELUM silinder, jalur di depan memang terlihat bersih menurut ambang
        mana pun, dan drone berhenti mengitari lalu menabrak lagi ke barrier.
        Yang benar adalah membandingkan posisi LONGITUDINAL sepanjang baris:
        manuver selesai ketika drone sudah berada di seberang rintangan.
        """
        st = self._act.get(aid)
        if st is None:
            return True
        if abs(math.degrees(st['sweep'])) > MAX_SWEEP_DEG:
            return True                     # pengaman anti-mengorbit-selamanya
        p = np.asarray(pos, float)[:2]
        s_self = float(np.dot(p - wp_start, u_line))
        for obs in obstacles or ():
            c = np.array([obs[1], obs[2]], dtype=float)
            d = c - wp_start
            s = float(np.dot(d, u_line))
            lat = float(np.linalg.norm(d - s * u_line))
            if lat > CORRIDOR_HALF + obs[3]:
                continue                    # di samping baris, abaikan
            if s_self < s + EXIT_PAST:
                return False                # belum melewati pusatnya
        return True

    def sweep_deg(self, aid):
        st = self._act.get(aid)
        return 0.0 if st is None else math.degrees(st['sweep'])


def _inside(p, poly):
    """Titik di dalam poligon (ray casting), tanpa dependensi shapely."""
    x, y = float(p[0]), float(p[1])
    n = len(poly)
    inside = False
    for i in range(n):
        x1, y1 = float(poly[i][0]), float(poly[i][1])
        x2, y2 = float(poly[(i + 1) % n][0]), float(poly[(i + 1) % n][1])
        if (y1 > y) != (y2 > y):
            xin = x1 + (y - y1) / (y2 - y1) * (x2 - x1)
            if x < xin:
                inside = not inside
    return inside
