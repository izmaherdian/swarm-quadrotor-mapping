"""Manuver mengitari: jarak ke rintangan harus TETAP, dan harus bisa selesai.

Dua cacat di bawah ini sempat lolos ke integrasi dan hanya tertangkap oleh uji
geometri offline — keduanya dikunci di sini.
"""
import math

import numpy as np
import pytest

from swarm_mid_level.circumnavigate import (
    MAX_SWEEP_DEG, ORBIT_PAD, Circumnavigator, blocking_obstacle)

DT = 0.05
SPEED = 1.2


def _obs(x, y, r=0.40, oid=901):
    return (oid, x, y, r, 4.0, None)


def _row(x0, x1, y=0.0):
    a = np.array([x0, y], dtype=float)
    b = np.array([x1, y], dtype=float)
    return a, b, (b - a) / float(np.linalg.norm(b - a))


def _fly(start, obs, cell=None, max_steps=800):
    """Jalankan manuver sampai selesai; kembalikan lintasan dan status."""
    ws, we, u = _row(-6.0, 6.0)
    cn = Circumnavigator()
    p = np.asarray(start, dtype=float).copy()
    cn.start(1, p, u, obs, cell)
    path = [p.copy()]
    for k in range(max_steps):
        v = cn.step(1, p, SPEED)
        p = p + v * DT
        path.append(p.copy())
        if cn.should_exit(1, p, ws, u, [obs]):
            return np.array(path), cn, k, True
    return np.array(path), cn, max_steps, False


# ── Pemicu: hanya yang benar-benar menghalangi ───────────────────────────

def test_obstacle_on_the_row_blocks():
    ws, we, _u = _row(-6.0, 6.0)
    assert blocking_obstacle((-2.0, 0.0), ws, we, [_obs(0.0, 0.0)]) is not None


def test_obstacle_already_passed_does_not_block():
    """Rintangan di belakang bukan penghalang; memicu manuver untuknya persis
    yang membuat drone 'berputar padahal tidak ada apa-apa'."""
    ws, we, _u = _row(-6.0, 6.0)
    assert blocking_obstacle((2.0, 0.0), ws, we, [_obs(0.0, 0.0)]) is None


def test_obstacle_beside_the_row_does_not_block():
    ws, we, _u = _row(-6.0, 6.0)
    assert blocking_obstacle((-2.0, 0.0), ws, we, [_obs(0.0, 2.0)]) is None


def test_far_ahead_obstacle_does_not_block_yet():
    """Rintangan 5 m di depan belum perlu manuver — baris masih lapang."""
    ws, we, _u = _row(-6.0, 6.0)
    assert blocking_obstacle((-5.5, 0.0), ws, we, [_obs(0.0, 0.0)]) is None


# ── Cacat 1: jari-jari harus TETAP dan sesuai target ─────────────────────

@pytest.mark.parametrize('start', [(-1.5, 0.0), (-2.0, 0.35), (-1.2, -0.5)])
def test_orbit_radius_is_held_constant(start):
    """Gain radial terlalu rendah (1.2) membuat jari-jari mengendap 0.12 m di
    LUAR target: melengkung rapi, tapi bukan pada jarak yang diminta."""
    obs = _obs(0.0, 0.0)
    path, _cn, _n, done = _fly(start, obs)
    R = obs[3] + ORBIT_PAD
    r = np.linalg.norm(path[8:], axis=1)
    assert abs(r.mean() - R) < 0.06, f'jari-jari rata-rata {r.mean():.3f}, target {R:.2f}'
    assert r.std() < 0.10, f'jari-jari tidak tetap (sebaran {r.std():.3f} m)'


def test_never_closer_than_orbit_radius():
    obs = _obs(0.0, 0.0)
    path, _cn, _n, _d = _fly((-1.5, 0.0), obs)
    d_min = float(np.linalg.norm(path, axis=1).min())
    assert d_min > obs[3] + 0.22, f'menembus batas fisik: {d_min:.3f} m'
    assert d_min > 1.0, f'terlalu dekat untuk manuver terencana: {d_min:.3f} m'


# ── Cacat 2: manuver HARUS bisa selesai ──────────────────────────────────

@pytest.mark.parametrize('start', [(-1.5, 0.0), (-2.0, 0.35), (-1.2, -0.5)])
def test_manoeuvre_terminates(start):
    """Syarat keluar pernah menuntut drone melewati silinder sejauh 1.20 m +
    radius, padahal jari-jari edarnya sendiri hanya ~1.30 m — jangkauan
    majunya tidak pernah sampai, dan drone mengorbit 1575 derajat tanpa henti."""
    path, cn, n, done = _fly(start, _obs(0.0, 0.0))
    assert done, f'tidak pernah selesai ({n} langkah, {cn.sweep_deg(1):.0f} deg)'
    assert abs(cn.sweep_deg(1)) < 200.0, (
        f'memutar terlalu jauh: {cn.sweep_deg(1):.0f} deg')
    assert n * DT < 6.0, f'terlalu lama: {n * DT:.1f} s'


def test_exits_past_the_obstacle():
    """Keluar harus SETELAH melewati pusat rintangan, bukan sebelumnya."""
    path, _cn, _n, done = _fly((-1.5, 0.0), _obs(0.0, 0.0))
    assert done and path[-1][0] > 0.0, f'keluar di x={path[-1][0]:+.2f}, belum lewat'


def test_orbit_cap_prevents_endless_loop():
    """Pengaman: geometri aneh pun tidak boleh membuat drone mengorbit abadi."""
    cn = Circumnavigator()
    ws, we, u = _row(-6.0, 6.0)
    p = np.array([-1.3, 0.0])
    cn.start(1, p, u, _obs(0.0, 0.0))
    cn._act[1]['sweep'] = math.radians(MAX_SWEEP_DEG + 10.0)
    assert cn.should_exit(1, p, ws, u, [_obs(0.0, 0.0)])


# ── Ujung baris: jangan keluar sel Voronoi ───────────────────────────────

def test_prefers_the_side_that_stays_inside_the_cell():
    """Rintangan di ujung baris: memutar ke sisi luar akan membawa drone keluar
    selnya dan masuk wilayah drone lain."""
    cell = [(-6.0, -3.0), (6.0, -3.0), (6.0, 0.6), (-6.0, 0.6)]   # sel di BAWAH baris
    ws, we, u = _row(-6.0, 6.0)
    cn = Circumnavigator()
    cn.start(1, np.array([-1.5, 0.0]), u, _obs(0.0, 0.0), cell)
    assert cn._act[1]['sign'] < 0, 'memilih sisi di luar sel'


def test_side_is_locked_for_the_whole_manoeuvre():
    """Berganti sisi di tengah manuver akan membuat drone berayun di depan
    rintangan — kegagalan yang sama seperti pada pemecah kebuntuan V2V."""
    cn = Circumnavigator()
    ws, we, u = _row(-6.0, 6.0)
    p = np.array([-1.5, 0.0])
    cn.start(1, p, u, _obs(0.0, 0.0))
    sign0 = cn._act[1]['sign']
    for _ in range(60):
        p = p + cn.step(1, p, SPEED) * DT
    assert cn._act[1]['sign'] == sign0
