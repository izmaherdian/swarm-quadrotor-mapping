# Product Requirement Document (PRD)
**Project Name:** Swarm Quadrotor Geodesy Cerdas & Resilien
**Target Environment:** ROS 2 Lyrical Luth (Ubuntu 26.04) / Gazebo Harmonic
**Document Status:** Final Draft (Fase 1)

---

## 1. Executive Summary
Proyek ini bertujuan untuk mengembangkan sistem simulasi "Swarm Quadrotor Geodesy Cerdas & Resilien" menggunakan framework ROS 2 Lyrical Luth. Sistem membagi kecerdasan armada quadrotor ke dalam tiga lapisan (High-Level, Mid-Level, dan Low-Level). Proyek ini merupakan studi komprehensif yang tidak hanya membahas algoritma High-Level (Koordinasi Geometris 2D & Self-Healing) dan Mid-Level (AI-Driven Collision Avoidance), tetapi juga melakukan studi komparatif mendalam pada lapisan Low-Level (*Robust Control*). Proyek dirancang dengan standar kualitas tinggi untuk dipublikasikan sebagai paper pada Engineering Physics International Conference (EPIC).

## 2. Product Goals & KPIs
* **Goals:** Mensimulasikan armada 7 quadrotor heterogen yang secara kolaboratif memetakan area geodesi dengan bentuk non-konveks secara otonom. Armada harus tahan terhadap kegagalan salah satu quadrotor (fault-tolerant) dan sanggup menghindari rintangan dinamis secara reaktif.
* **Key Performance Indicators (KPIs) untuk Reviewer EPIC:**
  1. **100% Coverage Area:** Algoritma Voronoi FT-CC memastikan tidak ada *blind spot* pada peta, meskipun ada simulasi kegagalan 1 atau lebih quadrotor di tengah misi.
  2. **Zero Collisions:** Algoritma Deep Reinforcement Learning (DRL) berhasil 100% mencegah tabrakan antar quadrotor maupun dengan objek lain.
  3. **Smooth Control Energy:** Grafik konsumsi energi kontrol terbukti lebih efisien berkat penghalusan jalur Bézier dibandingkan dengan pola belokan tajam *lawnmower* klasik.
  4. **Robustness & Stability (ITAE):** Menunjukkan bahwa optimasi kontroler PID yang diturunkan dari rumusan *H-infinity* mampu meminimalkan nilai *Integral of Time-multiplied Absolute Error* (ITAE) secara signifikan saat terkena gangguan angin (*wind gusts*) dibandingkan baseline.

## 3. System Architecture & Requirements

### 3.1. High-Level Layer: Geometric Coordination & Self-Healing
Bertanggung jawab atas pembagian wilayah dan orkestrasi rute armada di bidang 2D.
* **Input Area:** Peta geodesi non-konveks (seperti bintang/bumerang) dimasukkan melalui daftar koordinat poligon (X, Y) menggunakan file konfigurasi YAML/JSON.
* **Decentralized Voronoi Partitioning:** Algoritma membagi poligon secara adil menjadi 7 sel wilayah operasi secara terdesentralisasi.
* **Bézier Path Smoothing (𝒢¹ Continuity):** Algoritma menghaluskan belokan tajam dari pola *lawnmower* standar, menghasilkan urutan waypoint lintasan 2D yang memperhalus manuver quadrotor.
* **Fault-Tolerant Cooperative Coverage (FT-CC):**
  * **Komunikasi P2P:** Setiap quadrotor mem-publish sinyal `heartbeat` secara lokal via ROS 2 DDS.
  * **Self-Healing:** Jika terjadi `timeout` pada `heartbeat` quadrotor tetangga, quadrotor yang aktif di sekitarnya secara otomatis menghitung ulang sel Voronoi untuk mengambil alih *residual area* (area yang belum terpetakan) tanpa mengulangi jalur yang sudah dilewati.

### 3.2. Mid-Level Layer: AI-driven Reactive Obstacle Avoidance
Bertanggung jawab atas keselamatan dinamis (anti-stuck/anti-tabrak) menggunakan kecerdasan buatan.
* **Virtual Sensor Fusion:** Setiap quadrotor dilengkapi dengan sensor LiDAR virtual 360-derajat yang disimulasikan dari Gazebo untuk membaca jarak (raycast) terhadap gedung atau quadrotor lain.
* **Decentralized AI Policy (DRL - PPO/DQN):** 
  * Pelatihan (Training) dilakukan secara **offline** di lingkungan simulasi ringan (seperti kustom OpenAI Gym dengan Python) agar model konvergen cepat (15-45 menit).
  * Menggunakan pendekatan *Single-Agent Policy* di mana AI dilatih menganggap quadrotor lain sebagai rintangan dinamis biasa (komputasi sangat ringan).
  * Model diekspor ke ONNX/PyTorch.
* **Real-time Inference:** Node ROS 2 memuat model ONNX/PyTorch untuk memberikan perintah secara instan (< 1 ms). Output aksi (Action Space) berupa **Vektor Kecepatan Referensi (Vx, Vy)**.
* **Dynamic Behavior:** Jika ada ancaman terdeteksi, AI mengesampingkan perintah waypoint Bézier secara temporer dan mengambil alih kecepatan quadrotor untuk meliuk dengan halus sampai ancaman hilang.

### 3.3. Low-Level Layer: Robust Physical Dynamics Control & Comparative Study
Lapisan ini berfokus pada eksperimen kestabilan wahana *heterogeneous quadrotor* terhadap gangguan eksternal (hembusan angin) melalui pendekatan *Analytic PID Tuning*.
* **Evaluasi Sekuensial:** Simulasi dijalankan 3 kali berturut-turut untuk membandingkan 3 arsitektur kontrol utama:
  1. **PID-LQR:** Parameter PID (*Kp, Ki, Kd*) diturunkan secara analitis dari kalkulasi *Linear Quadratic Regulator* (LQR) berdasarkan pemetaan matriks *State-Feedback*.
  2. **PID-$H_\\infty$-Synthesis:** Parameter PID didapatkan dari hasil algoritma *robust synthesis* (seperti `musyn`) untuk meredam *worst-case disturbance*.
  3. **PID-$H_\\infty$-Backstepping:** Desain *Backstepping* non-linear yang dikombinasikan dengan kriteria norm-$H_\\infty$ dan direpresentasikan secara analitik menjadi parameter ekuivalen aksi PID.
* **PSO Gain Auto-Tuning (Metaheuristic Optimization):** Particle Swarm Optimization (PSO) digunakan sebagai *optimizer* terpusat untuk mencari parameter teoretis secara otomatis, yaitu matriks **Q/R** untuk LQR, matriks bobot **W** untuk $H_\\infty$-Synthesis, dan parameter *gain* dasar untuk Backstepping. Tujuannya adalah meminimalkan *fitness function* gabungan antara akurasi pelacakan lintasan (*ITAE*) dan efisiensi energi kontrol ($E_u$).
* **Eksekusi Akhir:** Output final dari proses optimasi teoretis ini adalah nilai PID tuning optimal yang akan dieksekusi oleh node *Custom Controller* ROS 2 untuk mem-publish perintah gaya/torsi (*wrench*) langsung ke baling-baling Gazebo.

## 4. User Interface & Visualization
* **RViz2:** Antarmuka visualisai utama. Menggunakan `Marker` dan `MarkerArray` untuk menggambar:
  * Batas poligon luar dan partisi sel Voronoi masing-masing quadrotor.
  * Titik-titik pembacaan LiDAR virtual.
  * Garis lengkung jalur Bézier.
* **ROS 2 Terminal:** Operator menggunakan *Command Line Interface* (CLI) dan skrip `ros2 launch` standar untuk memulai, memantau log, dan menyuntikkan kegagalan simulasi (mematikan node quadrotor tertentu secara paksa).

## 5. Deliverables
1. **Repositori GitHub (ROS 2 Workspace):** Kumpulan *open-source ROS 2 package* yang berisi seluruh *node* (Voronoi, PPO Inference) dan file konfigurasi simulasi (Gazebo Launch Files, YAML Map).
2. **Dokumentasi Riset (Paper):** Draf manuskrip ilmiah untuk konferensi EPIC, lengkap dengan grafik analitik (coverage time, fluktuasi energi, dan rasio tabrakan).

## 6. Out of Scope / Future Work
* Voronoi Partitioning untuk pemetaan permukaan/mesh 3D (saat ini dibatasi pada bidang 2D).
* Multi-Agent Reinforcement Learning (MARL) kompleks.
* Desain mikrokontroler fisik (sepenuhnya disimulasikan).
