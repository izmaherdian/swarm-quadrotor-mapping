# Roadmap & Arsitektur ROS 2 Workspace
**Project:** Swarm Quadrotor Geodesy Cerdas & Resilien
**Pendekatan:** Multi-Package Modular, 100% Python (rclpy), Bottom-Up Execution

---

## 1. Strategi Hardware & Struktur Folder (ROS 2 Workspace)
* **Catatan Hardware (Lenovo Yoga 6 Ryzen 5):** Karena spesifikasi laptop rentan untuk merender 7 quadrotor secara 3D simultan dengan LiDAR dan AI, Gazebo akan didesain berjalan secara **Headless Mode** (tanpa GUI grafis) saat simulasi komputasi berat. Pengamatan visual murni mengandalkan **RViz2** yang jauh lebih ringan.

Workspace proyek ini bernama `swarm_ws`. Di dalamnya terdapat folder `src` yang memuat 5 *package* modular untuk memisahkan setiap fungsionalitas algoritma:

```text
swarm_ws/
└── src/
    ├── swarm_msgs/                 # (Package 1: Custom Messages)
    │   └── msg/                    # Definisi format data antar node (mis. Heartbeat.msg, VoronoiCell.msg)
    │
    ├── swarm_low_level/            # (Package 2: Fisika & Kontrol Robust)
    │   └── swarm_low_level/
    │       ├── pid_lqr_node.py     # Kontroler PID dari LQR Mapping
    │       ├── pid_hinf_node.py    # Kontroler PID dari H-infinity Synthesis
    │       ├── pid_backstep.py     # Kontroler PID dari H-infinity Backstepping
    │       └── pso_tuner.py        # Metaheuristic Optimizer (Offline Tuning)
    │
    ├── swarm_mid_level/            # (Package 3: AI Collision Avoidance)
    │   └── swarm_mid_level/
    │       ├── ppo_inference.py    # Node ROS 2 untuk Load model ONNX/PyTorch
    │       ├── lidar_fusion.py     # Pre-processing data Raycast 360 Gazebo
    │       └── gym_env/            # (Folder untuk training DRL offline)
    │
    ├── swarm_high_level/           # (Package 4: Koordinasi Swarm)
    │   └── swarm_high_level/
    │       ├── voronoi_node.py     # Algoritma partisi geodesi 2D & FT-CC
    │       ├── bezier_path.py      # Algoritma penghalusan lintasan (G1 Continuity)
    │       └── heartbeat_p2p.py    # Publisher/Subscriber sinyal hidup quadrotor
    │
    └── swarm_sim/                  # (Package 5: Gazebo & Visualisasi)
        ├── launch/                 # Script untuk menjalankan node serentak (.launch.py)
        ├── models/                 # File 3D (SDF/URDF) quadrotor quadrotor
        └── worlds/                 # File dunia Gazebo (peta geodesi & gedung)
```

---

## 2. Step-by-Step Roadmap (Bottom-Up)

Karena riset ini sangat kompleks, pengerjaan difokuskan dari level "Paling Fisik" ke level "Paling Cerdas" untuk meminimalisir *bug* yang merembet.

### FASE 0: Pra-Kondisi (Translasi Kode & Template) - **[COMPLETED]**
* **Tujuan:** Menyiapkan jembatan dari aset yang sudah Anda miliki ke ekosistem ROS 2.
* **Modul Pekerjaan:**
  1. ~~**Konversi MATLAB $\rightarrow$ Python:** Menerjemahkan kode PID-LQR dan H-infinity dari Simulink ke dalam blok *script* Python.~~ (Selesai! LQR dan H-Infinity berhasil dimodelkan dalam arsitektur state-space Python).
  2. ~~**Optimalisasi & Benchmarking Kendali:** Mencari tuning terbaik menggunakan Grid Search / Optuna Bayesian Optimization.~~ (Selesai! LQR terbukti unggul secara keseluruhan di turbulensi stokastik dengan RMSE 0.168m, sementara H-Infinity *hyper-tuned* mencapai 0.151m dengan pengorbanan energi motor yang lebih besar).
  3. ~~**Clone Template Quadrotor:** Mengunduh template URDF/SDF *open-source* agar tidak perlu merancang fisika baling-baling dari nol.~~ (Model matematis baling-baling (Kf, Km) telah distandardisasi).

### FASE 1: Membangun Pondasi Fisik (Low-Level & Sim) - **[COMPLETED]**
* **Tujuan:** Memastikan 1 quadrotor bisa terbang stabil dan tahan angin.
* **Modul Pekerjaan:**
  1. ~~Tempatkan template SDF quadrotor hasil *clone* ke dalam `swarm_sim/models`.~~ (Selesai! Menggunakan template `iris_base` dengan parameter yang sudah disesuaikan `mass=1.0`).
  2. ~~Implementasikan `pid_lqr_node.py` dan `pid_hinf_node.py` di `swarm_low_level` menggunakan matriks perhitungan optimal dari Fase 0.~~ (Selesai! Jembatan `ros_gz_bridge` sukses mendistribusikan `actuator_msgs` ke motor).
  3. ~~Buat *launch file* dasar untuk mensimulasikan 1 drone di Gazebo dan menghubungkannya dengan *node* kontrol.~~ (Selesai! `sim_launch.py` terkonfigurasi dengan fitur headless mode).
  4. ~~Berikan gangguan angin (*wind plug-in*) di Gazebo untuk memvalidasi performa dunia nyata.~~ (Selesai! Efek Dryden Turbulence dengan `stddev=3.0` dan *body rigid collision* aktif. Masalah Odom Hz diselesaikan).
  5. ~~(*Checkpoint Paper 1:* Perbandingan performa *trajectory tracking* dan daya tahan turbulensi angin dalam lingkungan fisika 3D Gazebo).~~ (Selesai! Data terekstrak di `gazebo_hinf_plot.png` & `gazebo_crash_report.md`).

### FASE 2: Mengintegrasikan Otak (Mid-Level) - **[COMPLETED]**
* **Tujuan:** Quadrotor tidak lagi menabrak gedung saat terbang menuju *waypoint*.
* **Modul Pekerjaan:**
  1. ~~Gunakan *library* teruji seperti `gym-pybullet-quadrotors` (sangat ringan) untuk melatih model PPO secara *offline*.~~ (Selesai! Model PPO untuk penghindar rintangan berbasis LiDAR telah dilatih).
  2. ~~*Export* model yang sudah konvergen ke format `.onnx` atau `.pth`.~~ (Selesai! ONNX model `ppo_lidar_avoidance.onnx` dimuat secara sukses).
  3. ~~Tulis node `ppo_inference.py` di `swarm_mid_level` untuk membaca sensor LiDAR virtual dan mem-publish target kecepatan.~~ (Selesai! Diimplementasikan dalam `collision_avoidance_node.py` yang melacak odometri & LiDAR scan serta mem-publish `/iris_1/target_pose`).
  4. ~~*Test Flight:* Perintahkan quadrotor menembus gedung, pastikan AI membelokkannya secara mulus.~~ (Selesai! Verifikasi sukses menggunakan peta slalom rintangan merah-hijau-biru-oranye. Penambahan algoritma *Goal-Reacher Stabilization* menahan *overshoot* secara presisi di $X=7.0m$, serta implementasi *Time-Jump Filter* menjamin kestabilan kendali dari lag/lompatan waktu simulator).

### FASE 3: Orkestrasi Armada (High-Level) - **[COMPLETED]**
* **Tujuan:** 1 quadrotor sukses, saatnya digandakan menjadi 7 quadrotor yang berkolaborasi secara P2P.
* **Modul Pekerjaan:**
  1. ~~Integrasikan 7 model quadrotor ke dalam satu *launch file* Gazebo (`swarm_launch.py` mendukung dynamic scaling 1 s.d. 7 drone).~~ (Selesai! Modular dengan namespaces terisolasi dan topic remapping).
  2. ~~Buat definisi *custom message* di `swarm_msgs` (`Heartbeat.msg` untuk koordinat & status keaktifan).~~ (Selesai! Dikompilasi mandiri tanpa error spasi path).
  3. ~~Tulis `voronoi_node.py` dan `bezier_path.py` di `swarm_high_level` untuk membagi wilayah.~~ (Selesai! Menggunakan kliping poligon dinamis Sutherland-Hodgman & kurva kuadratik Bézier G1 continuity).
  4. ~~Aktifkan fitur **FT-CC & Shapely Raw Voronoi Polygon Union ($V_{\text{merged}} = \bigcup V_{\text{dead}}$)**:~~ (Selesai! Jika drone tetangga mati saat *mapping* berjalan, antrian *recovery* lama dibuang, sel Voronoi mati yang menempel dilebur utuh via Shapely `unary_union`, dan dihasilkan rute Lawnmower horizontal lurus baru (`fixed_angle=0.0`) yang dibagi ke maksimal 3 helper).
  5. ~~Buat simulator kinematik mandiri interaktif (`simulator_kinematics.py`) untuk visualisasi 2D cepat.~~ (Selesai! Lengkap dengan *Parallel Spatial Alignment* (0 path crossing), *Orthogonal Boundary Entry Transition*, GUI non-overlapping sidebar, reset instan saat toggle ON, dan integrasi 100% di `.venv`).

### FASE 4: Visualisasi Akhir & Pengambilan Data (Finishing) - **[IN PROGRESS]**
* **Tujuan:** Mempercantik antarmuka untuk di-*screenshot* ke dalam jurnal/paper dan mengambil data metrik performa.
* **Modul Pekerjaan:**
  1. Setup RViz2 untuk memvisualisasikan `Marker` batas Voronoi, garis Bézier, dan titik LiDAR.
  2. Jalankan misi penuh (Fase 1 + 2 + 3 secara serentak).
  3. Ekstrak plot log (Kinerja Baterai, Coverage Area, Error ITAE) untuk ditulis di naskah konferensi EPIC.

