"""Proyeksi Euklidean eksak ke polihedron 2D, tanpa dependency solver luar.

Menyelesaikan       min ||u - z||^2      s.t.   A u <= b        (u dalam R^2)

Di dua dimensi optimum dari proyeksi titik ke polihedron selalu berada di salah
satu dari:
  (a) titik z itu sendiri, bila feasible;
  (b) proyeksi z ke SATU garis constraint;
  (c) perpotongan SEPASANG garis constraint.

Jadi seluruh himpunan kandidat dapat dienumerasi (1 + m + m(m-1)/2 buah),
disaring dengan uji feasibility, lalu diambil yang terdekat ke z. Hasilnya
optimum eksak — bukan iteratif, tanpa toleransi konvergensi, tanpa kegagalan
diam-diam. Untuk m ~ 24 seluruhnya sekitar 300 kandidat: satu perkalian matriks.

Alasan tidak memakai qpsolvers/osqp/quadprog: tidak satu pun terpasang, wheel
cp314 tidak dijamin, dan launch script memanggil `python3` polos setelah
source ROS sehingga instalasi ke .venv belum tentu terlihat. Untuk masalah
2 variabel, ketergantungan itu murni risiko reproduksibilitas.
"""
import numpy as np

FEAS_TOL = 1e-7
PARALLEL_TOL = 1e-9


def _candidates(A, b, z):
    """Bangun seluruh titik kandidat KKT. Mengembalikan array (n, 2)."""
    m = A.shape[0]
    cands = [z.reshape(1, 2)]

    norm_sq = np.einsum('ij,ij->i', A, A)
    good = norm_sq > PARALLEL_TOL

    # (b) proyeksi ke tiap garis tunggal
    if np.any(good):
        Ag, bg, ng = A[good], b[good], norm_sq[good]
        t = (Ag @ z - bg) / ng
        cands.append(z.reshape(1, 2) - t[:, None] * Ag)

    # (c) perpotongan tiap pasangan garis
    if m >= 2:
        i, j = np.triu_indices(m, k=1)
        a_i, a_j = A[i], A[j]
        det = a_i[:, 0] * a_j[:, 1] - a_i[:, 1] * a_j[:, 0]
        ok = np.abs(det) > PARALLEL_TOL
        if np.any(ok):
            a_i, a_j = a_i[ok], a_j[ok]
            b_i, b_j = b[i][ok], b[j][ok]
            det = det[ok]
            px = (b_i * a_j[:, 1] - b_j * a_i[:, 1]) / det
            py = (a_i[:, 0] * b_j - a_j[:, 0] * b_i) / det
            cands.append(np.stack([px, py], axis=1))

    return np.vstack(cands)


def solve_projection(A, b, z):
    """Proyeksi eksak z ke {u : A u <= b}.

    Mengembalikan (u, feasible). Bila polihedron kosong, feasible=False dan u=z.
    """
    z = np.asarray(z, dtype=float).reshape(2)
    if A is None or len(b) == 0:
        return z.copy(), True

    A = np.asarray(A, dtype=float).reshape(-1, 2)
    b = np.asarray(b, dtype=float).reshape(-1)

    if np.all(A @ z <= b + FEAS_TOL):
        return z.copy(), True

    P = _candidates(A, b, z)
    feasible = np.all(P @ A.T <= b[None, :] + FEAS_TOL, axis=1)
    if not np.any(feasible):
        return z.copy(), False

    P = P[feasible]
    d = np.einsum('ij,ij->i', P - z, P - z)
    return P[int(np.argmin(d))].copy(), True


def max_violation(A, b, u):
    """Pelanggaran ternormalisasi terbesar, max_i (a_i^T u - b_i)/||a_i||."""
    if A is None or len(b) == 0:
        return 0.0
    A = np.asarray(A, dtype=float).reshape(-1, 2)
    b = np.asarray(b, dtype=float).reshape(-1)
    n = np.sqrt(np.einsum('ij,ij->i', A, A))
    n[n < PARALLEL_TOL] = 1.0
    return float(np.max((A @ u - b) / n))


def solve_least_violating(A_hard, b_hard, A_keep, b_keep, z):
    """Tier 2: minimalkan pelanggaran terburuk, tetap di dalam himpunan kinematik.

    A_keep/b_keep (batas laju + rate) selalu dipenuhi; A_hard/b_hard boleh
    dilanggar tapi seminimal mungkin. Selalu mengembalikan sesuatu yang
    terbatas dan dapat dieksekusi plant — tidak pernah sampah.
    """
    z = np.asarray(z, dtype=float).reshape(2)

    P = _candidates(np.vstack([A_keep, A_hard]),
                    np.concatenate([b_keep, b_hard]), z)
    ok = np.all(P @ np.asarray(A_keep).T <= np.asarray(b_keep)[None, :] + FEAS_TOL, axis=1)
    if not np.any(ok):
        # Kotak rate selalu memuat u_prev; kalau sampai sini pun kosong,
        # pemanggil akan jatuh ke Tier 3.
        return z.copy(), float('inf')

    P = P[ok]
    A_h = np.asarray(A_hard, dtype=float).reshape(-1, 2)
    b_h = np.asarray(b_hard, dtype=float).reshape(-1)
    n = np.sqrt(np.einsum('ij,ij->i', A_h, A_h))
    n[n < PARALLEL_TOL] = 1.0

    viol = np.max((P @ A_h.T - b_h[None, :]) / n[None, :], axis=1)
    best = float(np.min(viol))

    # Di antara yang sama-sama paling tidak melanggar, ambil yang terdekat ke z.
    tied = np.flatnonzero(viol <= best + 1e-9)
    d = np.einsum('ij,ij->i', P[tied] - z, P[tied] - z)
    return P[tied[int(np.argmin(d))]].copy(), max(0.0, best)


def polygon_rows(center, radius, n_sides):
    """Aproksimasi ||u - center|| <= radius dengan poligon n sisi TERTULIS DI DALAM.

    Setengah-bidang dipasang pada radius*cos(pi/n) sehingga poligon berada di
    dalam lingkaran: batas dijamin tidak pernah terlampaui.
    """
    ang = np.arange(n_sides) * (2.0 * np.pi / n_sides)
    A = np.stack([np.cos(ang), np.sin(ang)], axis=1)
    b = radius * np.cos(np.pi / n_sides) + A @ np.asarray(center, dtype=float)
    return A, b
