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

### FASE 0: Pra-Kondisi (Translasi Kode & Template)
* **Tujuan:** Menyiapkan jembatan dari aset yang sudah Anda miliki ke ekosistem ROS 2.
* **Modul Pekerjaan:**
  1. **Konversi MATLAB $\\rightarrow$ Python:** Menerjemahkan kode PID-LQR dan H-infinity dari Simulink ke dalam blok *script* Python (menggunakan library `scipy.signal` atau modul `control`).
  2. **Clone Template Quadrotor:** Mengunduh template URDF/SDF *open-source* (misal model *X500* dari Gazebo) agar tidak perlu merancang fisika baling-baling dari nol.

### FASE 1: Membangun Pondasi Fisik (Low-Level & Sim)
* **Tujuan:** Memastikan 1 quadrotor bisa terbang stabil dan tahan angin.
* **Modul Pekerjaan:**
  1. Tempatkan template SDF quadrotor hasil *clone* ke dalam `swarm_sim/models`.
  2. Tulis node `pid_lqr_node.py` di `swarm_low_level` menggunakan hasil perhitungan fungsi dari Fase 0.
  3. Berikan gangguan angin (wind plug-in) di Gazebo.
  4. Jalankan `pso_tuner.py` untuk mengoptimalkan gain kontroler hingga nilai ITAE terkecil. 
  5. (*Checkpoint Paper 1:* Perbandingan grafis stabilitas LQR vs H-inf vs Backstepping sudah bisa diekstrak di fase ini).

### FASE 2: Mengintegrasikan Otak (Mid-Level)
* **Tujuan:** Quadrotor tidak lagi menabrak gedung saat terbang menuju *waypoint*.
* **Modul Pekerjaan:**
  1. Gunakan *library* teruji seperti `gym-pybullet-quadrotors` (sangat ringan) untuk melatih model PPO secara *offline*.
  2. *Export* model yang sudah konvergen ke format `.onnx` atau `.pth`.
  3. Tulis node `ppo_inference.py` di `swarm_mid_level` untuk membaca sensor LiDAR virtual dan mem-publish target kecepatan ($V_x, V_y$).
  4. *Test Flight:* Perintahkan quadrotor menembus gedung, pastikan AI membelokkannya secara mulus.

### FASE 3: Orkestrasi Armada (High-Level)
* **Tujuan:** 1 quadrotor sukses, saatnya digandakan menjadi 7 quadrotor yang berkolaborasi.
* **Modul Pekerjaan:**
  1. Integrasikan 7 model quadrotor ke dalam satu *launch file* Gazebo.
  2. Buat definisi *custom message* di `swarm_msgs` (misalnya `Heartbeat.msg`).
  3. Tulis `voronoi_node.py` dan `bezier_path.py` di `swarm_high_level` untuk membagi-bagi peta geodesi menjadi 7 blok operasi.
  4. Aktifkan fitur **FT-CC**: Matikan paksa node quadrotor nomor 3, dan pastikan quadrotor nomor 2 dan 4 mengambil alih sisa area quadrotor nomor 3.

### FASE 4: Visualisasi Akhir & Pengambilan Data (Finishing)
* **Tujuan:** Mempercantik antarmuka untuk di-*screenshot* ke dalam jurnal/paper.
* **Modul Pekerjaan:**
  1. Setup RViz2 untuk memvisualisasikan `Marker` batas Voronoi, garis Bézier, dan titik LiDAR.
  2. Jalankan misi penuh (Fase 1 + 2 + 3 secara serentak).
  3. Ekstrak plot log (Kinerja Baterai, Coverage Area, Error ITAE) untuk ditulis di naskah konferensi EPIC.
