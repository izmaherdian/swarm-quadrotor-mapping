"""Pemecah simetri untuk mencegah kebuntuan CBF.

Aturan keras: constraint mengkodekan KEAMANAN, objektif mengkodekan PREFERENSI.
Tidak ada yang di sini boleh menyentuh A atau b — semuanya hanya menggeser
target v_nom. Dengan begitu liveness tidak pernah ditukar dengan keselamatan.

Kasus buntu klasik: drone menuju tepat ke pusat silinder. Constraint hanya
melarang mendekat, tidak menyarankan arah memutar, sehingga drone berhenti
diam. Solusinya memberi bias tangensial pada target, dengan sisi yang
DIKUNCI per pasangan (drone, rintangan) supaya tidak berayun bolak-balik.
"""
import math

import numpy as np


class DeadlockBreaker:
    def __init__(self, cfg):
        self.cfg = cfg
        self._side = {}        # (aid, oid) -> +1 / -1, terkunci
        self._stall_since = {}  # aid -> waktu mulai diam
        self._stall_dir = {}

    def reset_agent(self, aid):
        self._stall_since.pop(aid, None)
        self._stall_dir.pop(aid, None)
        for key in [k for k in self._side if k[0] == aid]:
            del self._side[key]

    def _pick_side(self, aid, oid, n_hat, u_hat, cell_polygon=None, obs_pos=None):
        """Pilih arah memutar sekali, lalu kunci (histeresis)."""
        key = (aid, oid)
        if key in self._side:
            return self._side[key]

        t_hat = np.array([-n_hat[1], n_hat[0]])
        # Default: menyimpang ke sisi yang sudah lebih dekat dengan arah tujuan.
        sign = 1.0 if float(t_hat @ u_hat) >= 0.0 else -1.0

        # Bila benar-benar simetris, pilih sisi yang masih di dalam sel Voronoi.
        # Logika ini diambil dari kode lama (tie-break MplPath) — bagian yang
        # memang layak dipertahankan.
        if abs(float(t_hat @ u_hat)) < 0.05 and cell_polygon is not None and obs_pos is not None:
            try:
                from matplotlib.path import Path as MplPath
                poly = MplPath(np.asarray(cell_polygon))
                cand_l = obs_pos + t_hat * 1.2
                cand_r = obs_pos - t_hat * 1.2
                in_l, in_r = poly.contains_point(cand_l), poly.contains_point(cand_r)
                if in_l and not in_r:
                    sign = 1.0
                elif in_r and not in_l:
                    sign = -1.0
            except Exception:
                pass

        self._side[key] = sign
        return sign

    def release_side(self, aid, oid):
        self._side.pop((aid, oid), None)

    def bias(self, agent, v_nom, obstacles, neighbours, t_now,
             cell_polygon=None):
        """Kembalikan target yang sudah diberi bias tangensial."""
        cfg = self.cfg
        v = np.array(v_nom, dtype=float)
        speed = float(np.linalg.norm(v))
        if speed < 1e-6:
            return v

        u_hat = v / speed
        cos_cone = math.cos(math.radians(cfg.cone_deg))

        # ── Rintangan: pemulihan jarak + bias tangensial ──
        from . import types as T

        for obs in obstacles:
            d_vec = agent.pos - obs.pos
            d = float(np.linalg.norm(d_vec))
            if d > cfg.include_radius or d < 1e-6:
                self.release_side(agent.aid, obs.oid)
                continue
            n_hat = d_vec / d

            delta = (cfg.delta_dynamic if obs.kind == T.CLASS_DYNAMIC
                     else cfg.delta_static)
            R_zone = obs.radius + cfg.drone_radius + delta
            h = d - R_zone
            if h < 0.0:
                v = v + cfg.k_separate * (-h) * n_hat

            # Pelepasan sisi hanya jika drone sudah benar-benar menjauh keluar dari zona rintangan
            if d > R_zone + 0.60:
                self.release_side(agent.aid, obs.oid)
                continue

            # Jika sudah memiliki sisi terkunci atau sedang menghadap rintangan
            key = (agent.aid, obs.oid)
            is_facing = float(-n_hat @ u_hat) >= cos_cone
            if key in self._side or is_facing:
                sign = self._pick_side(agent.aid, obs.oid, n_hat, u_hat,
                                       cell_polygon, obs.pos)
                t_hat = np.array([-n_hat[1], n_hat[0]])
                fade = max(0.0, min(1.0, (R_zone + 0.60 - d) / 0.60))
                v = v + sign * cfg.kappa * speed * fade * t_hat

        # ── Antar-drone: aturan tangan kanan + pemulihan jarak ──
        #
        # Constraint CBF hanya MENCEGAH mendekat; ia tidak pernah memulihkan
        # jarak yang sudah telanjur menyempit. Begitu beberapa drone terjepit
        # di dalam radius nyaman, phi(h) ~ 0 melarang semua pendekatan dan
        # kalau tetangga mengepung dari segala arah, satu-satunya kecepatan
        # feasible adalah nol: beku total.
        #
        # Karena itu pemulihan dipasang di TARGET, bukan di constraint —
        # keselamatan tidak tersentuh, tetapi gugus yang rapat akan mekar
        # sendiri sampai bisa mengalir lagi.
        for other, _lam in neighbours:
            if not (other.alive and other.airborne):
                continue
            d_vec = agent.pos - other.pos
            d = float(np.linalg.norm(d_vec))
            if d > cfg.v2v_include_radius or d < 1e-6:
                continue
            n_hat = d_vec / d

            h_soft = d - cfg.v2v_soft
            if h_soft < 0.0:
                v = v + cfg.k_separate * (-h_soft) * n_hat

            if float(-n_hat @ u_hat) < cos_cone:
                continue
            t_hat = np.array([-n_hat[1], n_hat[0]])
            v = v - cfg.kappa * 0.5 * speed * t_hat

        # ── Watchdog macet: rotasi gugus untuk membatasi durasi livelock ──
        #
        # Kebuntuan simetris (mis. 7 drone bertemu di satu titik) tidak dapat
        # dipecahkan dengan arah lolos per-drone: tetangga akan memilih arah
        # berlawanan dan saling mengunci. Yang berhasil adalah membuat SELURUH
        # gugus berputar ke arah yang sama mengelilingi centroid lokalnya,
        # sehingga tumpukan itu terurai seperti roda yang berputar.
        v_meas = float(np.linalg.norm(agent.vel))
        if v_meas < cfg.stall_speed and speed > cfg.stall_speed:
            start = self._stall_since.setdefault(agent.aid, t_now)
            held = t_now - start
            if held > cfg.stall_time:
                cluster = [o.pos for o, _ in neighbours
                           if o.alive and o.airborne
                           and np.linalg.norm(agent.pos - o.pos) < cfg.v2v_soft * 2.0]
                if cluster:
                    centroid = np.mean(np.vstack(cluster + [agent.pos]), axis=0)
                    r_vec = agent.pos - centroid
                    r = float(np.linalg.norm(r_vec))
                    # Di centroid persis, pakai arah tujuan sebagai jari-jari semu.
                    r_hat = r_vec / r if r > 1e-3 else u_hat
                else:
                    r_hat = u_hat

                # Handedness GLOBAL (+1 = berlawanan arah jarum jam) supaya
                # setiap drone di gugus yang sama memutar searah.
                t_hat = np.array([-r_hat[1], r_hat[0]])
                ramp = min(1.0, (held - cfg.stall_time) / 1.0)
                v = v + ramp * speed * t_hat
        else:
            self._stall_since.pop(agent.aid, None)
            self._stall_dir.pop(agent.aid, None)

        return v
