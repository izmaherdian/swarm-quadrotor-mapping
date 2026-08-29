# Analisis Skema 3 (Obstacle Avoidance) — Kenapa Drone Masih Menabrak

Dokumen ini menjelaskan **di mana** logika penghindaran rintangan berada dan
**kenapa** Skema 3 (dan karenanya Skema 4) masih gagal. Semua rujukan
`file:baris` menunjuk ke kondisi kode saat commit `5059dbb`.

---

## 1. Peta alur eksekusi Skema 3

```
launch_mapping_demo.sh -s 3
  │
  ├─ gz sim  worlds/obstacles.world        ← 9 silinder statis + 2 silinder dinamis
  │
  ├─ ros2 launch spawn_drones_launch.py  num_drones:=7  use_mid_level:=false
  │     ├─ per-drone: pid_lqr_node / pid_hinf_node   (low-level, subscribe cmd_vel + target_pose + target_velocity)
  │     ├─ per-drone bridge:  .../gpu_lidar/scan  →  /iris_i/lidar_scan
  │     └─ clock bridge + bridge  /model/dynamic_obs_{1,2}/{cmd_vel,odometry}
  │
  └─ experiments/test_7drone_voronoi_mapping.py  -p scheme:=3  -p enable_obstacles:=true
        └─ SEMUA logika penghindaran ada DI SINI (bukan di swarm_mid_level)
```

> **Penting:** `use_mid_level:=false` di `launch_mapping_demo.sh:193`. Jadi node
> `swarm_mid_level/collision_avoidance_node.py` (solver ORCA) **dan** kebijakan
> PPO `ppo_lidar_avoidance.onnx` **tidak dijalankan sama sekali** di Skema 3.
> Keduanya kode mati untuk skenario ini.

---

## 2. Di mana kode penghindaran rintangan

| Bagian | Lokasi |
|--------|--------|
| Definisi 9 rintangan statis (hard-coded, harus cocok dg world) | `experiments/test_7drone_voronoi_mapping.py:442-453` |
| Definisi 2 rintangan dinamis + Kalman filter | `:456-463`, kelas `DynamicObstacleKalmanFilter` `:296` |
| Penggerak rintangan dinamis (publish `cmd_vel` pola cos) | `update_dynamic_obstacles()` `:800-835` |
| Masking grid okupansi (untuk coverage, bukan avoidance) | `:468-480` |
| Subscribe LiDAR fisik | `:507-512`, callback `lidar_callback()` `:786-789` |
| **Inti penghindaran** | `compute_obstacle_avoidance_offset()` `:837-1108` |
| — orbit tangen silinder statis | `:865-998` |
| — lapisan repulsion darurat statis (<0.65 m) | `:1000-1011` |
| — evasion koridor rintangan dinamis | `:1013-1092` |
| — LiDAR hanya dipakai untuk telemetri `min_dist_to_obs` | `:1094-1102` |
| Pemanggilan saat menyapu baris | `:1778-1849` |
| Deteksi tabrakan fisik (logging saja) | `:1503-1535` |
| Trimming ujung baris dekat rintangan statis | `generate_boustrophedon()` `:158-210` |
| Konversi world→body + publish 3 setpoint | `send_world_twist()` `:2119-2169` |

Parameter kunci: `nominal_speed = 2.85 m/s`, `max_cmd_speed = 3.0 m/s`,
`lead_dist = 0.70 m`, loop 20 Hz (`:384-389`, `:546`). LiDAR 10 Hz, jangkauan
0.22–12 m, 360 sampel (`models/iris_base/model.sdf:49-70`). Batas kemiringan
low-level `angle_max = 0.262 rad` (15°) (`config/quadrotor_params.yaml`).

---

## 3. Akar masalah (diurut dari paling berdampak)

### 3.1 Kecepatan jelajah terlalu tinggi vs jarak deteksi — penyebab utama

Bypass silinder statis baru aktif saat `d_surf < 1.25 m` dan
`s_to_entry <= 0.45 m` (`:904`), lalu drone harus melambat dari 2.85 m/s ke
`V_NOM = 0.80 m/s` (`:867`).

Batas fisik: dengan `angle_max = 15°`, percepatan lateral maksimum ≈
`g·tan(15°) ≈ 2.63 m/s²`. Untuk mengerem 2.85 → 0.80 m/s butuh:

- waktu ≈ `2.05 / 2.63 ≈ 0.78 s`
- jarak ≈ `((2.85 + 0.80)/2) · 0.78 ≈ 1.42 m`

**Jarak berhenti (1.42 m) > jarak deteksi (1.25 m)**, bahkan tanpa
memperhitungkan latensi loop 20 Hz + LiDAR 10 Hz + lag motor. Drone
**secara fisika tidak mungkin** menghindar tepat waktu. Untuk rintangan
dinamis lebih parah: kecepatan penutupan bisa `2.85 + 1.5 ≈ 4.35 m/s`,
sedangkan trigger evasion di `dist_curr < 2.8 m` (`:1054`) → hanya ~0.64 s
ke tumbukan.

**Perbaikan:** turunkan `nominal_speed` di Skema 3/4 ke ~1.0–1.2 m/s, dan/atau
naikkan jarak deteksi bypass ke ≥ 3.0 m dengan pengereman feedforward
progresif yang dimulai jauh sebelum rintangan.

### 3.2 Rintangan statis hanya dihindari jika berada di sel Voronoi drone itu

`:870` → `relevant_obs_list = agent.my_static_obstacles`. Daftar ini diisi dari
`poly_raw.contains_point((obs.x, obs.y))` saat penugasan sel (`:655`, `:698-703`).

Akibatnya rintangan **tidak terlihat** oleh logika orbit ketika drone:

- **transit** melintasi arena menuju titik start selnya (`:1643-1653`,
  `:1709-1719`) — melewati sel drone lain;
- **kembali ke centroid** setelah selesai (`:2003-2010`);
- **transit ke blok recovery** setelah ada drone mati (sel di-merge, `:1294`);
- berada dekat batas sel yang bergeser akibat relaksasi Lloyd / kematian drone.

Di semua kasus itu hanya lapisan repulsion `< 0.65 m` (`:1000-1011`, yang
me-loop `self.static_obstacles` penuh) yang menangkap — dan 0.65 m pada
2.85 m/s = ~0.23 s, jelas terlambat. Ini kemungkinan besar sumber "nabrak-nabrak"
yang kamu lihat saat fase transit/pembukaan.

**Perbaikan:** gunakan `self.static_obstacles` (semua) untuk penghindaran runtime,
bukan `my_static_obstacles`. Isolasi per-sel hanya relevan untuk perencanaan
baris, bukan untuk keselamatan.

### 3.3 Rintangan yang sudah pernah dilewati "dilupakan" secara permanen

`:880` → `if obs_id == agent.last_bypassed_obs_id and bypass_state != 'arc_contour': continue`.

`last_bypassed_obs_id` di-set saat orbit selesai (`:963`) tetapi **tidak pernah
di-reset saat pindah baris**. Pada sapuan boustrophedon, drone kembali melewati
silinder yang sama di baris bersebelahan → deteksi di-skip → tinggal andalkan
lapisan 0.65 m → tabrakan pada sapuan balik.

**Perbaikan:** reset `last_bypassed_obs_id = None` setiap kali `row_idx`
bertambah (di sekitar `:1812-1814`), atau ganti mekanisme suppress dengan
histeresis berbasis jarak (mis. hanya skip jika `d_surf > 2.5 m`).

### 3.4 Uji "di dalam koridor" memakai arah garis nominal, bukan kecepatan aktual

`:872` → `u_line = unit_vel` berasal dari `p_nom_dir` (arah garis sapuan ideal).
Uji `abs(w_0) < 0.90` (`:893`) menilai apakah rintangan memotong koridor
**relatif garis ideal**. Padahal kecepatan drone yang sebenarnya juga
dipengaruhi koreksi cross-track (`v_corr_lat`, `:1843`) dan repulsion antar-drone
(`apply_v2v_repulsion`). Rintangan 0.95 m dari garis ideal dianggap "aman" dan
di-skip, walau drone sedang didorong ke arahnya.

**Perbaikan:** hitung `w_0` terhadap vektor kecepatan perintah gabungan
(`v_ff + v_corr_lat + v2v`), atau perbesar ambang koridor ke `R_ORBIT + margin`.

### 3.5 Tidak ada fallback "berhenti total" untuk rintangan statis

Untuk rintangan dinamis, `speed_scale_dyn = 0.0` memaksa drone berhenti maju
(`:1061`, `:1082`). Untuk statis, `speed_scale_static` jadi 0 **hanya** di dalam
cabang `arc_contour` atau repulsion darurat. Jika bypass gagal ter-trigger
(kasus 3.1–3.4), `obs_speed_scale` tetap 1.0 dan drone melaju penuh
`nominal_speed` menembus silinder (`:1840`).

**Perbaikan:** tambahkan cek tanpa-syarat di awal fungsi — jika ada rintangan
mana pun dengan `d_surf < d_brake` dan berada di depan (dot dengan arah gerak
> 0), set `speed_scale = clamp((d_surf - d_min)/(d_brake - d_min), 0, 1)`.

### 3.6 Vektor menghindar dan target posisi bisa saling bertentangan

`send_world_twist()` mem-publish setiap tick: `cmd_vel` (Twist),
`target_velocity` (feedforward), dan `target_pose` = `agent.ref_pos`
(`:2145-2168`). Di `pid_lqr_node`, begitu `target_pose_received = True`
(`:581`), `cmd_vel_callback` **tidak lagi menggeser** `x_cmd/y_cmd` — ia hanya
mengisi feedforward kecepatan (`:614-620`). Jadi low-level pada dasarnya adalah
**pengendali posisi yang mengejar `ref_pos`**.

Pada cabang di mana `obs_speed_scale == 0` tetapi bukan `arc_contour`/darurat
(`:1781-1784`), `ref_pos` **tidak diperbarui** → tetap menempel di titik pada
garis tabrakan. Sementara `v_obs_avoid` hanya masuk sebagai feedforward
kecepatan. LQR menarik drone kembali ke `ref_pos` di garis → gerak neto
menyerempet silinder.

**Perbaikan:** setiap kali menghasilkan `v_obs_avoid` ≠ 0, **selalu** geser
`agent.ref_pos` ke titik aman yang konsisten dengan vektor itu (beberapa cabang
sudah melakukannya; buat konsisten di semua cabang).

### 3.7 LiDAR nyata tidak dipakai untuk menghindar

`lidar_callback` hanya menyimpan `ranges` (`:789`). Di
`compute_obstacle_avoidance_offset` LiDAR hanya membaca `min` untuk telemetri
`agent.min_dist_to_obs` (`:1094-1102`) — **tidak menghasilkan kecepatan
menghindar apa pun**. Seluruh penghindaran bergantung pada daftar rintangan
yang di-hard-code + odometri ground-truth rintangan dinamis.

Artinya: sistem sudah "curang" pakai posisi sempurna dan **tetap** menabrak →
ini menegaskan masalahnya **kontrol/timing (bagian 3.1)**, bukan persepsi.

Bug kecil: `agent.lidar_ranges` diinisialisasi `None` (`:283`), tetapi `:1095`
memanggil `len(agent.lidar_ranges)` — `len(None)` melempar `TypeError` di dalam
loop kontrol bila pesan LiDAR suatu drone belum/berhenti datang. Ganti init ke
`np.array([])` atau cek `is not None` dulu.

### 3.8 Rintangan dinamis: kinematik, cepat, menyapu pusat arena

Model `dynamic_obs_*` bersifat `kinematic=true`, `gravity=false`
(`worlds/obstacles.world:809-847`) → tidak bisa didorong, menembus apa pun.
Amplitudo 10 m, `omega1 = 0.15` → laju puncak `10·0.15 = 1.5 m/s` melintang
diagonal penuh melewati (0,0). Drone **wajib** mengalah sepenuhnya dan lebih
awal; sidestep 1.6–2.5 m/s yang baru mulai di `dist < 2.8 m` tidak cukup untuk
membebaskan radius gabungan ~0.95 m sebelum tumbukan.

Selain itu ada **dua sumber kebenaran** untuk posisi rintangan dinamis:
`update_dynamic_obstacles()` mem-publish `cmd_vel` pola `cos` (`:812-826`),
sementara plugin `VelocityControl` Gazebo mengintegrasikannya sendiri, lalu
`dyn_obs_odom_callback` membaca odometri asli (`:791-798`). Fase antara profil
`cos` analitik dan integrasi plugin bisa melenceng → prediksi Kalman meleset
saat odometri sesaat tidak valid.

**Perbaikan:** perlambat rintangan dinamis (`omega ~0.06`), mulai evasion di
`dist < 6 m` berbasis TTC, dan hentikan mem-publish `cmd_vel` cos — cukup
gerakkan lewat trajektori Gazebo murni lalu percaya odometri.

---

## 4. Kenapa Skema 1 & 2 lolos tapi 3 tidak

Skema 1/2 tidak punya rintangan. Satu-satunya beban di Skema 2 adalah gaya angin
Dryden yang ditangani **low-level** (feedforward `wind_callback` di
`pid_lqr_node:622`). Jalur kritis Skema 1/2 = kualitas tracking garis lurus,
yang memang sudah matang (`nominal_speed = 2.85` aman tanpa halangan).

Skema 3 menambahkan constraint **spasial keras** yang harus ditangani di
**high-level** (coordinator) dengan anggaran waktu < 1 detik — dan seperti
diuraikan di bagian 3, anggaran itu tidak cukup pada kecepatan sekarang dengan
gating deteksi yang sekarang. Skema 4 = Skema 3 + angin, jadi mustahil lulus
sebelum Skema 3 beres.

---

## 5. Urutan perbaikan yang disarankan

1. **Turunkan `nominal_speed` → ~1.1 m/s khusus `scheme in (3,4)`** (satu baris,
   uji dulu — ini menyelesaikan sebagian besar 3.1 & memberi ruang untuk sisanya).
2. **Pakai `self.static_obstacles` penuh** untuk penghindaran runtime (3.2).
3. **Reset `last_bypassed_obs_id` per pergantian baris** (3.3).
4. **Tambah rem kecepatan tanpa-syarat** berbasis `d_surf` terdekat di depan (3.5).
5. **Selalu sinkronkan `ref_pos` dengan `v_obs_avoid`** (3.6).
6. Perbaiki init `lidar_ranges` (3.7) — cepat, cegah crash senyap.
7. Perlambat & perpanjang horizon rintangan dinamis (3.8).
8. Baru pertimbangkan memakai LiDAR nyata sebagai lapisan keamanan terakhir.

Verifikasi tiap langkah dengan `--headless` dulu, cek log
`🚨 [OBSTACLE CRASH]` (`:1516`, `:1532`) dan telemetri `min_dist_to_obs` per
drone, lalu konfirmasi visual di RViz sekali di akhir.
