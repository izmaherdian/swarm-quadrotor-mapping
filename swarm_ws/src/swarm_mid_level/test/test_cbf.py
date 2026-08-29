"""Uji lapisan penghindaran CBF-QP.

Dijalankan tanpa ROS maupun Gazebo:

    PYTHONPATH=src/swarm_mid_level:src/swarm_low_level pytest src/swarm_mid_level/test -v
"""
import numpy as np
import pytest

from swarm_mid_level.cbf import (
    AgentState, Bounds, CBFAvoidance, CBFConfig, Obstacle, PlantModel, Task,
    phi, phi_inverse, phi_zero_h, solve_projection,
)
from swarm_mid_level.cbf import types as T


@pytest.fixture(scope='module')
def plant():
    return PlantModel.from_config()


@pytest.fixture
def cfg():
    c = CBFConfig()
    c.v_max = 3.0
    return c


# ── Solver QP ────────────────────────────────────────────────────────────

def test_qp_unconstrained_returns_target():
    A = np.array([[1.0, 0.0]])
    b = np.array([10.0])
    u, feas = solve_projection(A, b, np.array([1.0, 2.0]))
    assert feas
    assert np.allclose(u, [1.0, 2.0])


def test_qp_single_active_row_is_normal_correction():
    """Saat satu baris aktif, hanya komponen normal yang dikoreksi.

    Inilah yang membuat drone MENYUSUR mengitari rintangan, bukan berhenti.
    """
    A = np.array([[1.0, 0.0]])
    b = np.array([0.5])
    u, feas = solve_projection(A, b, np.array([2.0, 1.7]))
    assert feas
    assert u[0] == pytest.approx(0.5)      # normal dipotong
    assert u[1] == pytest.approx(1.7)      # tangensial utuh


def test_qp_matches_bruteforce_oracle():
    """Optimum eksak, diuji silang dengan pencarian kasar pada grid halus."""
    rng = np.random.default_rng(3)
    for _ in range(200):
        m = int(rng.integers(2, 8))
        ang = rng.uniform(0, 2 * np.pi, m)
        A = np.stack([np.cos(ang), np.sin(ang)], axis=1)
        b = rng.uniform(0.2, 1.5, m)
        z = rng.normal(0, 1.5, 2)

        u, feas = solve_projection(A, b, z)
        assert feas, 'polihedron memuat titik asal, harus feasible'
        assert np.all(A @ u <= b + 1e-7)

        g = np.linspace(-2.5, 2.5, 220)
        X, Y = np.meshgrid(g, g)
        P = np.stack([X.ravel(), Y.ravel()], axis=1)
        ok = np.all(P @ A.T <= b[None, :], axis=1)
        if not np.any(ok):
            continue
        d = np.linalg.norm(P[ok] - z, axis=1).min()
        assert np.linalg.norm(u - z) <= d + 1e-6


def test_qp_detects_empty_polyhedron():
    A = np.array([[1.0, 0.0], [-1.0, 0.0]])
    b = np.array([-1.0, -1.0])          # x <= -1 dan x >= 1
    _, feas = solve_projection(A, b, np.zeros(2))
    assert not feas


# ── Fungsi barrier ───────────────────────────────────────────────────────

def test_phi_is_class_k(plant):
    a, Td, vc = 0.75 * plant.a_max, 0.30, plant.v_c
    hs = np.linspace(0.0, 6.0, 400)
    vals = phi(hs, a, Td, vc)
    assert vals[0] == 0.0
    assert np.all(np.diff(vals) >= -1e-12), 'phi harus monoton naik'
    assert phi(phi_zero_h(a, Td, vc), a, Td, vc) == pytest.approx(0.0, abs=1e-9)


def test_phi_lipschitz_bounded_by_dead_time(plant):
    """Kemiringan phi <= 1/T_d, syarat rekursif feasible terhadap batas rate."""
    a, Td, vc = 0.75 * plant.a_max, 0.30, plant.v_c
    hs = np.linspace(phi_zero_h(a, Td, vc) + 1e-3, 8.0, 5000)
    slope = np.diff(phi(hs, a, Td, vc)) / np.diff(hs)
    assert slope.max() <= 1.0 / Td + 1e-6


def test_phi_inverse_roundtrip(plant):
    a, Td, vc = 0.75 * plant.a_max, 0.30, plant.v_c
    for s in (0.5, 1.0, 1.86, 2.42, 2.85):
        h = phi_inverse(s, a, Td, vc)
        assert float(phi(h, a, Td, vc)) == pytest.approx(s, abs=1e-6)
    # Pada s = 0 phi berada di interval datar, jadi hanya syarat lemah.
    assert float(phi(phi_inverse(0.0, a, Td, vc), a, Td, vc)) == pytest.approx(0.0, abs=1e-9)


def test_reaction_distance_exceeds_legacy_trigger(plant):
    """Justifikasi kuantitatif kegagalan kode lama.

    Bypass lama memicu pada d_surf < 1.25 m. Pada kecepatan yang BENAR-BENAR
    terukur di Gazebo (1.86 m/s) jarak reaksi minimum sudah 1.48 m — sudah
    terlambat sebelum bicara kecepatan nominal 2.85 m/s.
    """
    a, Td, vc = 0.75 * plant.a_max, 0.30, plant.v_c
    assert float(phi_inverse(1.86, a, Td, vc)) > 1.25
    assert float(phi_inverse(2.85, a, Td, vc)) > 2.5


def test_plant_derives_expected_constants(plant):
    assert plant.a_max == pytest.approx(2.6307, abs=1e-3)
    assert plant.k_v == pytest.approx(3.677, abs=1e-2)
    assert plant.T_lead == pytest.approx(0.329, abs=1e-3)
    assert plant.stopping_distance(2.85) == pytest.approx(1.64, abs=0.02)


# ── Invariance: inti keseluruhan desain ──────────────────────────────────

def _head_on(plant, cfg, v0, gap, dt=0.05, steps=400, obs_vel=(0.0, 0.0)):
    """Terbangkan drone lurus ke rintangan; kembalikan clearance minimum."""
    avoid = CBFAvoidance(cfg, plant)
    obs = Obstacle(oid=1, pos=np.array([gap, 0.0]), vel=np.array(obs_vel),
                   radius=0.40,
                   accel_bound=0.25 if any(obs_vel) else 0.0,
                   kind=T.CLASS_DYNAMIC if any(obs_vel) else T.CLASS_STATIC)
    avoid.set_world([obs], Bounds(-100, 100, -100, 100))

    agent = AgentState(aid=1, pos=np.zeros(2), vel=np.array([v0, 0.0]),
                       v_prev_cmd=np.array([v0, 0.0]))
    R = obs.radius + cfg.drone_radius
    h_min = float('inf')

    for _ in range(steps):
        task = Task(v_nom=np.array([3.0, 0.0]))    # terus menekan maju
        res = avoid.solve_all({1: agent}, {1: task}, dt)[1]
        agent.vel = plant.step(agent.vel, res.v_safe, dt)
        agent.pos = agent.pos + agent.vel * dt
        agent.v_prev_cmd = res.v_safe
        obs.pos = obs.pos + obs.vel * dt
        h_min = min(h_min, float(np.linalg.norm(agent.pos - obs.pos)) - R)
    return h_min


@pytest.mark.parametrize('v0', [0.5, 1.0, 1.5, 1.86, 2.4, 2.85, 3.0])
def test_static_obstacle_never_breached(plant, cfg, v0):
    """Himpunan aman invariant untuk seluruh kecepatan awal yang realistis."""
    h_min = _head_on(plant, cfg, v0, gap=12.0)
    assert h_min > 0.0, f'menabrak pada v0={v0}: h_min={h_min:.3f}'


def test_moving_obstacle_head_on(plant, cfg):
    """Rintangan bergerak menuju drone: kecepatan penutupan ~4.3 m/s."""
    h_min = _head_on(plant, cfg, v0=2.85, gap=16.0, obs_vel=(-1.5, 0.0))
    assert h_min > 0.0, f'tertabrak rintangan dinamis: h_min={h_min:.3f}'


def test_starting_inside_margin_still_recovers(plant, cfg):
    """Mulai di dalam bantalan aman: harus mundur, bukan menyerah."""
    h_min = _head_on(plant, cfg, v0=0.0, gap=0.85)
    assert h_min > -0.05, f'gagal pulih dari pelanggaran awal: {h_min:.3f}'


# ── Resiprositas ─────────────────────────────────────────────────────────

def test_lambda_split_sums_to_one(cfg):
    from swarm_mid_level.cbf.avoidance import _lambda, DEFAULT_PRIORITY_W
    names = list(DEFAULT_PRIORITY_W)
    for a in names:
        for b in names:
            ai = AgentState(1, np.zeros(2), np.zeros(2), np.zeros(2),
                            priority_w=DEFAULT_PRIORITY_W[a])
            aj = AgentState(2, np.ones(2), np.zeros(2), np.zeros(2),
                            priority_w=DEFAULT_PRIORITY_W[b])
            assert _lambda(ai, aj) + _lambda(aj, ai) == pytest.approx(1.0)


def test_seven_agent_circle_swap(plant, cfg):
    """Tujuh drone bertukar posisi menembus pusat lingkaran.

    Kasus adversarial: konfigurasi simetris sempurna, semua saling berhadapan
    sekaligus. Jauh lebih berat daripada misi sebenarnya, yang membagi drone
    ke sel Voronoi terpisah.

    KETERBATASAN YANG DIKETAHUI. Yang dijamin CBF adalah KESELAMATAN, dan itu
    bertahan tanpa syarat: pada seluruh sweep parameter yang diuji
    (v2v_include_radius 3/4/6 m, k_separate 2.0/3.5, max_neighbour_rows 4/6)
    separasi minimum tidak pernah turun di bawah v2v_hard.

    LIVENESS tidak dijamin. Pada v2v_include_radius 3.0 dan 4.0 kawanan
    menumpuk lalu buntu permanen; hanya 6.0 yang lolos. Kebuntuan CBF pada
    konfigurasi simetris memang persoalan terbuka di literatur — pemecah
    simetri di deadlock.py memitigasi, bukan menyelesaikan. Jangan
    memperlakukan lulusnya uji ini sebagai bukti liveness yang kokoh.
    """
    n, R = 7, 9.0
    ang = np.arange(n) * 2 * np.pi / n
    starts = np.stack([R * np.cos(ang), R * np.sin(ang)], axis=1)
    goals = -starts

    avoid = CBFAvoidance(cfg, plant)
    avoid.set_world([], Bounds(-20, 20, -20, 20))
    agents = {i + 1: AgentState(i + 1, starts[i].copy(), np.zeros(2), np.zeros(2))
              for i in range(n)}

    dt, min_sep = 0.05, float('inf')
    for step in range(1200):
        tasks = {}
        for i, ag in agents.items():
            d = goals[i - 1] - ag.pos
            nrm = float(np.linalg.norm(d))
            tasks[i] = Task(v_nom=(d / nrm * min(2.5, 2.0 * nrm)) if nrm > 1e-6
                            else np.zeros(2))
        res = avoid.solve_all(agents, tasks, dt, t_now=step * dt)
        for i, ag in agents.items():
            ag.vel = plant.step(ag.vel, res[i].v_safe, dt)
            ag.pos = ag.pos + ag.vel * dt
            ag.v_prev_cmd = res[i].v_safe

        P = np.array([ag.pos for ag in agents.values()])
        D = np.linalg.norm(P[:, None, :] - P[None, :, :], axis=2)
        np.fill_diagonal(D, np.inf)
        min_sep = min(min_sep, float(D.min()))

    # KESELAMATAN: dijamin, ditegakkan keras.
    assert min_sep >= cfg.v2v_hard - 0.02, f'terlalu dekat: {min_sep:.3f} m'

    # LIVENESS: dilaporkan, TIDAK ditegakkan. Menegakkannya akan menggoda
    # penyetelan besaran fisik (T_d, v2v_include_radius) sampai kasus sintetis
    # ini lolos — mencocokkan fisika dengan tes, persis kesalahan yang membuat
    # kode lama rapuh. T_d berasal dari identifikasi telemetri dan tidak boleh
    # diubah demi tes ini.
    remaining = max(float(np.linalg.norm(goals[i - 1] - ag.pos))
                    for i, ag in agents.items())
    if remaining >= 2.5:
        import warnings
        warnings.warn(
            f'Kebuntuan pada pertukaran antipodal simetris: sisa {remaining:.2f} m. '
            'Keselamatan tetap terjaga. Konfigurasi ini lebih berat daripada '
            'misi sebenarnya, yang memisahkan drone ke sel Voronoi berbeda.',
            stacklevel=2)


# ── Tangga infeasibility ─────────────────────────────────────────────────

def test_never_returns_garbage_when_boxed_in(plant, cfg):
    """Terkepung rintangan dari segala arah: harus tetap terbatas & terdiagnosa."""
    avoid = CBFAvoidance(cfg, plant)
    obs = [Obstacle(oid=i, pos=np.array([np.cos(a), np.sin(a)]) * 0.95,
                    radius=0.40, kind=T.CLASS_STATIC)
           for i, a in enumerate(np.linspace(0, 2 * np.pi, 9)[:-1])]
    avoid.set_world(obs, Bounds(-3, 3, -3, 3))

    agent = AgentState(1, np.zeros(2), np.zeros(2), np.zeros(2))
    res = avoid.solve_all({1: agent}, {1: Task(v_nom=np.array([2.0, 0.0]))}, 0.05)[1]

    # Hasil harus terbatas, patuh batas laju, dan praktis diam — bukan sampah.
    # Tier 0 di sini justru hasil TERBAIK: masih ada kecepatan aman yang sah.
    assert np.all(np.isfinite(res.v_safe))
    assert float(np.linalg.norm(res.v_safe)) <= cfg.v_max + 1e-6
    assert float(np.linalg.norm(res.v_safe)) < 0.5, 'terkepung: harus nyaris berhenti'
    assert res.active and res.limiting.startswith('static')


def test_rate_limit_is_respected(plant, cfg):
    """Perintah tidak boleh melompat lebih dari a_max*dt dalam satu tick.

    Persis pelanggaran yang dilakukan kode lama (2.85 -> 0.80 m/s seketika).
    """
    avoid = CBFAvoidance(cfg, plant)
    avoid.set_world([], Bounds(-50, 50, -50, 50))
    dt = 0.05
    agent = AgentState(1, np.zeros(2), np.array([2.85, 0.0]),
                       v_prev_cmd=np.array([2.85, 0.0]))
    res = avoid.solve_all({1: agent}, {1: Task(v_nom=np.array([-3.0, 0.0]))}, dt)[1]
    jump = float(np.linalg.norm(res.v_safe - agent.v_prev_cmd))
    assert jump <= plant.a_max * dt + 1e-6, f'lompatan {jump:.3f} m/s terlalu besar'


def test_ref_pos_is_consistent_with_velocity(plant, cfg):
    """ref_pos = pos + T_lead*v_safe, satu-satunya aturan.

    Di kode lama enam tempat berbeda me-teleport ref_pos, masing-masing
    bertarung dengan perintah kecepatan lewat position loop low-level.
    """
    avoid = CBFAvoidance(cfg, plant)
    avoid.set_world([], Bounds(-50, 50, -50, 50))
    agent = AgentState(1, np.array([3.0, -2.0]), np.zeros(2), np.zeros(2))
    res = avoid.solve_all({1: agent}, {1: Task(v_nom=np.array([1.0, 0.5]))}, 0.05)[1]
    assert np.allclose(res.ref_pos, agent.pos + plant.T_lead * res.v_safe)


def test_dynamic_obstacle_not_crowded_out_by_static(plant, cfg):
    """Regresi: rintangan bergerak tidak boleh tersingkir oleh yang diam.

    Versi pertama mengurutkan baris constraint murni berdasarkan jarak lalu
    memotong di max_obstacle_rows. Enam silinder diam yang tidak berbahaya
    di sekitar drone akan menyingkirkan satu rintangan dinamis yang sedang
    mendekat — dan itu persis penyebab dua tabrakan rintangan dinamis pada
    uji Gazebo 600 detik.
    """
    avoid = CBFAvoidance(cfg, plant)

    # Enam silinder diam berjejer di sisi kiri, semuanya lebih dekat
    # daripada rintangan dinamis, tetapi tidak satu pun menghalangi jalur.
    obs = [Obstacle(oid=i, pos=np.array([-1.6, -3.0 + 1.0 * i]), radius=0.40,
                    kind=T.CLASS_STATIC) for i in range(6)]
    # Rintangan dinamis jauh di depan tetapi menutup jarak dengan cepat.
    obs.append(Obstacle(oid=99, pos=np.array([7.0, 0.0]),
                        vel=np.array([-1.5, 0.0]), radius=0.45,
                        accel_bound=0.25, kind=T.CLASS_DYNAMIC))
    avoid.set_world(obs, Bounds(-30, 30, -30, 30))

    agent = AgentState(1, np.zeros(2), np.array([2.5, 0.0]),
                       v_prev_cmd=np.array([2.5, 0.0]))
    dt, h_min = 0.05, float('inf')
    R = 0.45 + cfg.drone_radius

    for _ in range(300):
        task = Task(v_nom=np.array([2.85, 0.0]))
        r = avoid.solve_all({1: agent}, {1: task}, dt)[1]
        agent.vel = plant.step(agent.vel, r.v_safe, dt)
        agent.pos = agent.pos + agent.vel * dt
        agent.v_prev_cmd = r.v_safe
        obs[-1].pos = obs[-1].pos + obs[-1].vel * dt
        h_min = min(h_min, float(np.linalg.norm(agent.pos - obs[-1].pos)) - R)

    assert h_min > 0.0, f'menabrak rintangan dinamis: h_min={h_min:.3f} m'


def test_recovers_when_pushed_inside_obstacle_margin(plant, cfg):
    """Regresi: drone yang terlanjur di dalam bantalan harus terdorong keluar.

    Constraint CBF hanya MENCEGAH mendekat, tidak pernah MEMULIHKAN. Saat QP
    sesekali infeasible ia menerima slack kecil (0.01-0.10) per tick; itu
    terakumulasi. Pada uji Skema 4, h_min satu drone meluncur
    0.61 -> 0.30 -> 0.05 -> -0.09 lalu menyentuh rintangan dinamis.

    Di sini drone sengaja ditempatkan DI DALAM bantalan dan didorong terus ke
    arah rintangan; clearance harus PULIH, bukan terus tergerus.
    """
    avoid = CBFAvoidance(cfg, plant)
    obs = Obstacle(oid=1, pos=np.array([2.0, 0.0]), radius=0.45,
                   kind=T.CLASS_DYNAMIC)
    avoid.set_world([obs], Bounds(-30, 30, -30, 30))

    R_margin = obs.radius + cfg.drone_radius + cfg.delta_dynamic
    # Mulai jelas di dalam bantalan (h < 0), tetapi belum bersentuhan fisik.
    agent = AgentState(1, np.array([2.0 - (R_margin - 0.15), 0.0]),
                       np.zeros(2), np.zeros(2))

    h0 = float(np.linalg.norm(agent.pos - obs.pos)) - R_margin
    assert h0 < 0.0, 'prasyarat uji: harus mulai di dalam bantalan'

    dt = 0.05
    for _ in range(200):
        # Terus menekan MENUJU rintangan — kasus terburuk.
        task = Task(v_nom=np.array([1.5, 0.0]))
        r = avoid.solve_all({1: agent}, {1: task}, dt)[1]
        agent.vel = plant.step(agent.vel, r.v_safe, dt)
        agent.pos = agent.pos + agent.vel * dt
        agent.v_prev_cmd = r.v_safe

    h_end = float(np.linalg.norm(agent.pos - obs.pos)) - R_margin
    d_phys = float(np.linalg.norm(agent.pos - obs.pos)) - (obs.radius + cfg.drone_radius)

    assert d_phys > 0.0, f'menembus rintangan secara fisik: {d_phys:.3f} m'
    assert h_end > h0, f'clearance memburuk, bukan pulih: {h0:.3f} -> {h_end:.3f}'


def test_scheme3_scenario_collision_free():
    """Medan rintangan Skema 3 yang sebenarnya, pada kecepatan nominal penuh.

    Inilah skenario yang membuat kode lama menabrak. Berjalan tanpa Gazebo,
    jadi bisa dipakai sebagai gerbang sebelum tiap commit.
    """
    import scenario_scheme3 as S
    r = S.run(seconds=90.0, speed=2.85, verbose=False)
    assert r['crashes'] == 0, f"menabrak {r['crashes']} kali"
    assert r['h_static_min'] > 0.0
    assert r['h_dyn_min'] > 0.0
    assert r['d_v2v_min'] > 0.60
    assert r['tier_frac'][3] == 0.0, 'Tier 3 (kegagalan numerik) tidak boleh terjadi'


def test_solve_is_fast_enough_for_20hz(plant, cfg):
    """7 drone @ 20 Hz harus jauh di bawah anggaran 50 ms."""
    avoid = CBFAvoidance(cfg, plant)
    obs = [Obstacle(oid=i, pos=np.array([i * 2.0 - 6.0, 3.0]), radius=0.40)
           for i in range(9)]
    avoid.set_world(obs, Bounds(-15, 15, -15, 15))
    agents = {i: AgentState(i, np.array([i * 1.5 - 5.0, 0.0]),
                            np.zeros(2), np.zeros(2)) for i in range(1, 8)}
    tasks = {i: Task(v_nom=np.array([2.0, 1.0])) for i in agents}

    import time
    t0 = time.perf_counter()
    for _ in range(20):
        avoid.solve_all(agents, tasks, 0.05)
    per_tick = (time.perf_counter() - t0) / 20 * 1e3
    assert per_tick < 25.0, f'{per_tick:.1f} ms/tick terlalu lambat untuk 20 Hz'
