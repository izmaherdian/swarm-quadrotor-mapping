Kamu adalah AI yang membantu menyelesaikan masalah multi-agent drone swarm obstacle avoidance di simulasi ROS 2 Gazebo. Berikut sistem dan masalahnya:
Sistem
Ada 7 drone quadrotor (Iris) yang terbang formasi dari garis start menuju target X=10, melewati 4 obstacle berbentuk tiang. Setiap drone punya:
1. ORCA (Optimal Reciprocal Collision Avoidance) — menghitung kecepatan aman 2D supaya tidak tabrak drone lain dan obstacle
2. Sensor LiDAR 360° — mendeteksi obstacle statis
3. Gaya tolak (repulsion field) — inverse-square law, bantu dorong drone menjauh
4. COLREGs bias — aturan belok kanan untuk pecah simetri head-on
5. Low-level PID controller — mengeksekusi kecepatan + yaw dari ORCA ke motor
Formasi awal: 7 drone berjejer di Y = -6, -4, -2, 0, +2, +4, +6 (jarak 2m).
Posisi obstacle (tiang, radius ~0.25m):
- A: di Y=-1.5, X=2
- B: di Y=+1.5, X=3.5
- C: di Y=-1.5, X=5
- D: di Y=+2.5, X=6.5
Drone 3 (Y=-2) dan 4 (Y=0) harus lewat di antara obstacle A dan B. Drone 5 (Y=+2) lewat antara B dan D.
Semua drone dikirim waypoint X=10 (Y tetap sama dengan spawn).
Masalah Utama
Simulasi di GUI mode terlihat tidak stabil dan obstacle avoidance hampir gagal:
1. Yaw error sangat besar (sampai 175°) — Drone terbang mundur. Saat ORCA mendorong drone ke samping untuk menghindar, arah yaw yang mengikuti arah waypoint tiba-tiba berubah drastis. Akibatnya PID yaw berantakan dan drone terlihat berputar-putar tidak karuan.
2. Clearance obstacle terlalu tipis — Drone lewat hanya 8cm dari permukaan obstacle (berisiko tabrak).
3. Drone 6 turun drastis — Z drop sampai 1.6m (harusnya 2m), dekat dengan ground.
4. Beberapa drone overshoot Y — Misal Drone 2 yang spawn di Y=-4 melayang sampai Y=-11.64.
Workflow
Berikut urutan langkah yang harus kamu lakukan — kerjakan satu per satu secara berurutan:
Step 1: Pahami Kode
- Baca seluruh file collision_avoidance_node.py (mid-level ORCA + yaw logic)
- Baca file pid_lqr_node.py (low-level PID + yaw controller)
- Baca file empty.world (posisi obstacle)
- Baca file test_waypoints.py (cara kirim waypoint)
- Cari yaw control, ORCA solver, repulsion field, COLREGs bias, velocity limiter, slew rate limiter, dan PID yaw gain
Step 2: Analisis
Identifikasi semua titik yang berkontribusi ke 4 masalah di atas. Cari hubungan antara:
- Yaw direction logic → yaw error → backward flight
- Repulsion cap + COLREGs → lateral overshoot
- max_speed + slew rate → Z stability
- Velocity-direction yaw → positive feedback loop
Step 3: Usulkan Solusi
Beri saran konkret berupa:
- Fungsi/baris mana yang harus diubah
- Nilai parameter baru
- Logika baru (jelaskan dalam pseudocode)
Step 4: Implementasi Satu Per Satu
Jangan langsung implement semua. Kerjakan secara bertahap:
1. Mulai dengan perubahan paling sederhana dulu
2. Setelah setiap perubahan, jalankan test (headless mode)
3. Analisis hasil dari CSV tiap drone (lokasi: folder hasil simulasi)
4. Dari hasil, putuskan apakah perubahan membantu atau perlu di-rollback
5. Lanjut ke perubahan berikutnya
Cara test:
- Jalankan run_multi_agent.sh untuk launch sim headless
- Jalankan test_waypoints.py di terminal lain
- Tunggu sampai semua drone mencapai X=10
- Analisis CSV (kolom: Time, X, Y, Z, Yaw, Ref_Yaw, body rates, RPM)
- Matikan sim, lanjut ke iterasi berikutnya
Jika suatu perubahan membuat hasil lebih buruk (crash, overshoot lebih parah, dll), rollback dan coba pendekatan lain.
Yang Perlu Dibantu
1. Strategi yaw: Apa cara terbaik menentukan arah yaw drone — ikut arah waypoint, arah velocity, atau hybrid? Pertimbangkan trade-off antara stabilitas visual vs akurasi trajectory.
2. Obstacle clearance: Bagaimana meningkatkan jarak aman dari obstacle tanpa membuat drone oleng atau crash?
3. Z stability: Apa penyebab Z drop dan bagaimana mencegahnya?
4. Lateral overshoot: Bagaimana mencegah drone melayang terlalu jauh ke samping saat menghindar?
5. Perbedaan GUI vs headless: Apa yang mungkin berbeda saat sim di-render dengan GUI?