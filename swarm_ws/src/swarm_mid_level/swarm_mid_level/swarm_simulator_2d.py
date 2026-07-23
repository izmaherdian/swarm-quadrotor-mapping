#!/usr/bin/env python3
import sys
import os
import math
import signal
import numpy as np
import matplotlib.pyplot as plt

# Tambahkan path ke folder swarm_mid_level agar bisa mengimpor ORCASolver2D
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from collision_avoidance_node import ORCASolver2D

# Setup global SIGINT handler agar Ctrl+C langsung menutup jendela dan keluar
def sigint_handler(sig, frame):
    print("\n👋 Simulasi dihentikan oleh pengguna (Ctrl+C). Keluar...")
    plt.close('all')
    sys.exit(0)
signal.signal(signal.SIGINT, sigint_handler)

class SwarmSimulator2D:
    def __init__(self):
        # Definisikan rintangan sesuai model Gazebo (x, y, radius=0.5m)
        self.obstacles = [
            {'pos': np.array([2.0, -1.5], dtype=np.float32), 'radius': 0.5},
            {'pos': np.array([3.5, 1.5], dtype=np.float32), 'radius': 0.5},
            {'pos': np.array([5.0, -1.5], dtype=np.float32), 'radius': 0.5},
            {'pos': np.array([6.5, 2.5], dtype=np.float32), 'radius': 0.5}
        ]

        # Konfigurasi umum
        self.max_speed = 2.5
        self.safety_radius = 0.8
        self.time_horizon = 5.0
        self.dt = 0.1
        self.steps = 300  # Default 30 detik penerbangan
        self.scheme = 1  # 1: 7-drone obstacle swarm, 2: 2-drone head-on collision

        # Inisialisasi ORCA Solver
        self.orca_solver = ORCASolver2D(
            time_horizon=self.time_horizon,
            safety_radius=self.safety_radius,
            max_speed=self.max_speed
        )

        self.reset_simulation()

    def reset_simulation(self):
        """Inisialisasi ulang posisi, target, dan status drone berdasarkan skema aktif"""
        self.positions = []
        self.velocities = []
        self.targets = []
        self.yaw_smooth = []
        self.repulsion_smooth = []
        self.history = {}
        self.step_idx = 0

        if self.scheme == 1:
            self.active_drones = 7
            spacing = 2.0
            for i in range(1, 8):
                spawn_y = float((i - 4.0) * spacing)
                self.positions.append(np.array([0.0, spawn_y], dtype=np.float32))
                self.velocities.append(np.array([0.0, 0.0], dtype=np.float32))
                self.targets.append(np.array([10.0, spawn_y], dtype=np.float32))
                self.yaw_smooth.append(0.0)
                self.repulsion_smooth.append(np.array([0.0, 0.0], dtype=np.float32))
        else:
            # Skema 2: 2 Drone saling berhadapan (head-on collision avoidance)
            self.active_drones = 2
            # Drone 1 (Kiri ke Kanan)
            self.positions.append(np.array([-2.0, 0.0], dtype=np.float32))
            self.velocities.append(np.array([0.0, 0.0], dtype=np.float32))
            self.targets.append(np.array([10.0, 0.0], dtype=np.float32))
            self.yaw_smooth.append(0.0)
            self.repulsion_smooth.append(np.array([0.0, 0.0], dtype=np.float32))
            # Drone 2 (Kanan ke Kiri)
            self.positions.append(np.array([10.0, 0.0], dtype=np.float32))
            self.velocities.append(np.array([0.0, 0.0], dtype=np.float32))
            self.targets.append(np.array([-2.0, 0.0], dtype=np.float32))
            self.yaw_smooth.append(math.pi)
            self.repulsion_smooth.append(np.array([0.0, 0.0], dtype=np.float32))

        for i in range(self.active_drones):
            self.history[i] = []

    def simulate_lidar(self, drone_pos, yaw):
        """Simulasikan pembacaan Lidar 2D (360 ray) terhadap rintangan lingkaran"""
        num_rays = 360
        angles_body = np.linspace(-np.pi, np.pi, num_rays)
        ranges = np.ones(num_rays, dtype=np.float32) * 10.0  # Default max range 10m

        for idx, angle_body in enumerate(angles_body):
            angle_world = yaw + angle_body
            ray_dir = np.array([np.cos(angle_world), np.sin(angle_world)], dtype=np.float32)
            for obs in self.obstacles:
                pos_rel = obs['pos'] - drone_pos
                projection = np.dot(pos_rel, ray_dir)
                if projection < 0:
                    continue  # Arah berlawanan

                closest_approach_sq = np.dot(pos_rel, pos_rel) - projection**2
                if closest_approach_sq < obs['radius']**2:
                    chord = np.sqrt(obs['radius']**2 - closest_approach_sq)
                    dist = projection - chord
                    if 0.1 < dist < ranges[idx]:
                        ranges[idx] = dist
        return ranges

    def run(self, realtime=False):
        if not realtime:
            # Jalankan Skema 1 offline dan simpan plotnya
            print("🤖 Menjalankan simulasi numerik offline untuk Skema 1...")
            self.scheme = 1
            self.reset_simulation()
            self.run_numerical_offline()
            inner_dir = os.path.dirname(os.path.abspath(__file__))
            self.plot_results(os.path.join(inner_dir, "trajectory_simulation.png"))

            # Jalankan Skema 2 offline dan simpan plotnya
            print("🤖 Menjalankan simulasi numerik offline untuk Skema 2...")
            self.scheme = 2
            self.reset_simulation()
            self.run_numerical_offline()
            self.plot_results(os.path.join(inner_dir, "trajectory_simulation_scheme2.png"))
            return

        self.reset_requested = False
        self.scheme = 1
        self.reset_simulation()

        plt.ion()
        fig, ax = plt.subplots(figsize=(10, 8))
        
        # Callback untuk keluar bersih ketika jendela ditutup menggunakan tombol [X]
        def on_close(event):
            print("\n👋 Jendela simulator ditutup oleh pengguna. Keluar...")
            sys.exit(0)
        fig.canvas.mpl_connect('close_event', on_close)

        # Callback keyboard untuk ganti skema
        def on_key(event):
            if event.key in ['b', 'B']:
                self.scheme = 2 if self.scheme == 1 else 1
                print(f"\n🔄 Berpindah ke Skema {self.scheme}...")
                self.reset_requested = True
        fig.canvas.mpl_connect('key_press_event', on_key)

        print("\n💡 PETUNJUK: Klik jendela plot lalu tekan tombol 'B' pada keyboard untuk berganti skema simulasi!")

        # Loop utama simulasi interaktif tak terbatas
        while True:
            if self.reset_requested:
                self.reset_requested = False
                self.reset_simulation()

            ax.clear()

            # Mengatur judul dan visualisasi rintangan statis
            if self.scheme == 1:
                for idx, obs in enumerate(self.obstacles):
                    circle = plt.Circle(obs['pos'], obs['radius'], color='red', alpha=0.6)
                    ax.add_patch(circle)
                    ax.text(obs['pos'][0], obs['pos'][1], f"Obs {chr(65+idx)}", color='white', ha='center', va='center', weight='bold')
                ax.set_title("Skema 1: 7-Drone Obstacle Avoidance (Tekan 'B' untuk pindah skema)", fontsize=13, weight='bold')
                ax.set_xlim(-1.0, 11.0)
                ax.set_ylim(-7.0, 7.0)
            else:
                ax.set_title("Skema 2: Head-On Collision Avoidance (Tekan 'B' untuk pindah skema)", fontsize=13, weight='bold')
                ax.set_xlim(-3.0, 11.0)
                ax.set_ylim(-3.0, 3.0)

            colors = ['blue', 'green', 'orange', 'purple', 'brown', 'cyan', 'magenta']
            traj_lines = []
            drone_markers = []
            safety_circles = []
            
            for i in range(self.active_drones):
                line, = ax.plot([], [], color=colors[i], linewidth=2, label=f"Drone {i+1}")
                traj_lines.append(line)
                marker, = ax.plot([], [], color=colors[i], marker='o', markersize=8)
                drone_markers.append(marker)
                circle = plt.Circle((0,0), 0.4, color=colors[i], fill=False, linestyle='--', alpha=0.4)
                ax.add_patch(circle)
                safety_circles.append(circle)

            lidar_scatter = ax.scatter([], [], color='red', s=12, alpha=0.6, marker='x', label="Point Cloud Lidar" if self.scheme == 1 else "")

            ax.set_xlabel("X Position (m)")
            ax.set_ylabel("Y Position (m)")
            ax.grid(True)
            ax.legend(loc='upper left')

            collisions = 0
            min_dist_to_obs = 999.0

            # Loop perulangan langkah di dalam skema saat ini
            while self.step_idx < self.steps and not self.reset_requested:
                new_positions = []
                all_lidar_points_x = []
                all_lidar_points_y = []

                for i in range(self.active_drones):
                    pos_self = self.positions[i]
                    vel_self = self.velocities[i]
                    target = self.targets[i]

                    self.history[i].append(pos_self.copy())

                    # 1. Preferred Velocity
                    rel_target = target - pos_self
                    dist_target = np.linalg.norm(rel_target)
                    if dist_target < 0.05:
                        pref_vel = np.zeros(2, dtype=np.float32)
                    else:
                        speed = min(self.max_speed, dist_target * 0.8)
                        pref_vel = (rel_target / dist_target) * speed

                    # 1b. Break head-on symmetry (COLREGs Turn-Right Rule)
                    # Jika ada tetangga dekat di depan arah preferred velocity, geser preferred velocity sedikit ke kanan
                    for j in range(self.active_drones):
                        if i == j:
                            continue
                        rel_nbr = self.positions[j] - pos_self
                        dist_nbr = np.linalg.norm(rel_nbr)
                        if dist_nbr < 3.5:
                            pref_speed = np.linalg.norm(pref_vel)
                            if pref_speed > 0.1:
                                unit_pref = pref_vel / pref_speed
                                unit_nbr = rel_nbr / max(dist_nbr, 0.05)
                                dot_front = np.dot(unit_pref, unit_nbr)
                                if dot_front > 0.85:  # Head-on tepat di depan (sudut < 30 derajat)
                                    # Vektor tegak lurus ke kanan: (dy, -dx)
                                    right_vec = np.array([unit_pref[1], -unit_pref[0]], dtype=np.float32)
                                    bias_gain = 0.25 * (1.0 - (dist_nbr / 3.5))
                                    pref_vel += right_vec * (self.max_speed * bias_gain)

                    # 2. Tetangga (Drone Lain)
                    neighbors = []
                    repulsion_vec = np.zeros(2, dtype=np.float32)
                    for j in range(self.active_drones):
                        if i == j:
                            continue
                        neighbors.append({'pos': self.positions[j], 'vel': self.velocities[j], 'is_static': False})
                        rel_nbr = pos_self - self.positions[j]
                        dist_nbr = np.linalg.norm(rel_nbr)
                        if dist_nbr < 2.0:
                            rep_gain = ((2.0 / max(dist_nbr, 0.4)) ** 2) * 0.4
                            repulsion_vec += (rel_nbr / max(dist_nbr, 0.05)) * rep_gain

                    # 3. Lidar (Hanya untuk skema 1 rintangan)
                    if self.scheme == 1:
                        lidar_ranges = self.simulate_lidar(pos_self, self.yaw_smooth[i])
                        angles_world = self.yaw_smooth[i] + np.linspace(-np.pi, np.pi, len(lidar_ranges))
                        obs_mask = lidar_ranges < 4.5

                        if np.any(obs_mask):
                            close_indices = np.where(obs_mask)[0]
                            for idx in close_indices[::6]:
                                d_i = float(lidar_ranges[idx])
                                ang_i_world = float(angles_world[idx])
                                obs_pos_i = pos_self + np.array([d_i * np.cos(ang_i_world), d_i * np.sin(ang_i_world)], dtype=np.float32)
                                neighbors.append({'pos': obs_pos_i, 'vel': np.zeros(2, dtype=np.float32), 'is_static': True, 'radius': 0.35})
                                all_lidar_points_x.append(obs_pos_i[0])
                                all_lidar_points_y.append(obs_pos_i[1])

                            for idx in close_indices[::4]:
                                d_i = float(lidar_ranges[idx])
                                if d_i > 2.2:
                                    continue
                                ang_i_world = float(angles_world[idx])
                                obs_rel_i = np.array([d_i * np.cos(ang_i_world), d_i * np.sin(ang_i_world)], dtype=np.float32)
                                
                                # Hanya terapkan gaya tolak jika rintangan berada di hemisfer depan pergerakan drone
                                is_front = True
                                pref_speed = np.linalg.norm(pref_vel)
                                if pref_speed > 0.1:
                                    is_front = np.dot(obs_rel_i, pref_vel) > 0
                                
                                if is_front:
                                    push_dir = -obs_rel_i / max(d_i, 0.05)
                                    rep_gain_i = ((2.2 / max(d_i, 0.4)) ** 2) * 0.3
                                    repulsion_vec += push_dir * rep_gain_i

                    # Capping gaya tolak
                    rep_len = np.linalg.norm(repulsion_vec)
                    max_rep = self.max_speed * 0.75
                    if rep_len > max_rep:
                        repulsion_vec = (repulsion_vec / rep_len) * max_rep

                    # Skala gaya tolak mengecil saat mendekati target untuk mencegah deadlock hover di akhir
                    repulsion_scale = min(1.0, dist_target / 1.5)
                    repulsion_vec *= repulsion_scale

                    self.repulsion_smooth[i] = 0.7 * self.repulsion_smooth[i] + 0.3 * repulsion_vec
                    if dist_target > 0.3:
                        pref_vel += self.repulsion_smooth[i]
                    else:
                        self.repulsion_smooth[i] = np.zeros(2, dtype=np.float32)

                    # 4. ORCA safe velocity
                    safe_vel = self.orca_solver.compute_orca_velocity(pos_self, vel_self, pref_vel, neighbors, None)
                    ref_vx = np.clip(safe_vel[0], -self.max_speed, self.max_speed)
                    ref_vy = np.clip(safe_vel[1], -1.2, 1.2)

                    self.velocities[i] = np.array([ref_vx, ref_vy], dtype=np.float32)
                    new_pos = pos_self + self.velocities[i] * self.dt
                    new_positions.append(new_pos)

                    safe_speed = np.linalg.norm(safe_vel)
                    if safe_speed > 0.15 and dist_target > 0.8:
                        yaw_target = float(np.arctan2(safe_vel[1], safe_vel[0]))
                        delta_yaw = (yaw_target - self.yaw_smooth[i] + np.pi) % (2 * np.pi) - np.pi
                        self.yaw_smooth[i] += 0.25 * delta_yaw

                self.positions = new_positions

                # Cek Tabrakan Antar Drone
                for a in range(self.active_drones):
                    for b in range(a + 1, self.active_drones):
                        dist = np.linalg.norm(self.positions[a] - self.positions[b])
                        if dist < 0.5:
                            collisions += 1

                # Update visualisasi plot
                for i in range(self.active_drones):
                    hist = np.array(self.history[i])
                    if len(hist) > 0:
                        traj_lines[i].set_data(hist[:, 0], hist[:, 1])
                    drone_markers[i].set_data([self.positions[i][0]], [self.positions[i][1]])
                    safety_circles[i].set_center((self.positions[i][0], self.positions[i][1]))

                if self.scheme == 1 and all_lidar_points_x:
                    lidar_scatter.set_offsets(np.column_stack((all_lidar_points_x, all_lidar_points_y)))
                else:
                    lidar_scatter.set_offsets(np.empty((0, 2)))

                fig.canvas.draw()
                fig.canvas.flush_events()
                plt.pause(0.01)
                self.step_idx += 1

            # Selesai satu putaran skema, simpan grafik saat ini
            scheme_suffix = "" if self.scheme == 1 else "_scheme2"
            inner_dir = os.path.dirname(os.path.abspath(__file__))
            output_plot_path = os.path.join(inner_dir, f"trajectory_simulation{scheme_suffix}.png")
            self.plot_results(output_plot_path)

            # Salin juga ke folder artifact agar ter-render di chat
            artifact_plot_name = f"trajectory_simulation{scheme_suffix}.png"
            artifact_plot_path = os.path.join('/home/izmaherdian/.gemini/antigravity-cli/brain/5957a601-2c4a-4ba8-a185-b5636c8ffa5c', artifact_plot_name)
            try:
                import shutil
                shutil.copy(output_plot_path, artifact_plot_path)
                print(f"📈 Grafik trajectory disalin ke artifact chat: {artifact_plot_path}")
            except Exception as e:
                pass

            # Tunggu pergantian skema setelah simulasi selesai
            print(f"✅ Skema {self.scheme} selesai! Tabrakan: {collisions}. Menunggu tombol 'B' untuk berpindah skema...")
            while not self.reset_requested:
                plt.pause(0.1)

    def run_numerical_offline(self):
        """Simulasi numerik murni offline untuk skema aktif"""
        print(f"🤖 Menjalankan simulasi numerik offline (Skema {self.scheme})...")
        min_dist_to_obs = 999.0
        for step in range(self.steps):
            new_positions = []
            for i in range(self.active_drones):
                pos_self = self.positions[i]
                vel_self = self.velocities[i]
                target = self.targets[i]

                self.history[i].append(pos_self.copy())
                rel_target = target - pos_self
                dist_target = np.linalg.norm(rel_target)
                if dist_target < 0.05:
                    pref_vel = np.zeros(2, dtype=np.float32)
                else:
                    speed = min(self.max_speed, dist_target * 0.8)
                    pref_vel = (rel_target / dist_target) * speed

                # 1b. Break head-on symmetry (COLREGs Turn-Right Rule)
                for j in range(self.active_drones):
                    if i == j:
                        continue
                    rel_nbr = self.positions[j] - pos_self
                    dist_nbr = np.linalg.norm(rel_nbr)
                    if dist_nbr < 3.5:
                        pref_speed = np.linalg.norm(pref_vel)
                        if pref_speed > 0.1:
                            unit_pref = pref_vel / pref_speed
                            unit_nbr = rel_nbr / max(dist_nbr, 0.05)
                            dot_front = np.dot(unit_pref, unit_nbr)
                            if dot_front > 0.85:  # Head-on tepat di depan
                                right_vec = np.array([unit_pref[1], -unit_pref[0]], dtype=np.float32)
                                bias_gain = 0.25 * (1.0 - (dist_nbr / 3.5))
                                pref_vel += right_vec * (self.max_speed * bias_gain)

                neighbors = []
                repulsion_vec = np.zeros(2, dtype=np.float32)
                for j in range(self.active_drones):
                    if i == j: continue
                    neighbors.append({'pos': self.positions[j], 'vel': self.velocities[j], 'is_static': False})
                    rel_nbr = pos_self - self.positions[j]
                    dist_nbr = np.linalg.norm(rel_nbr)
                    if dist_nbr < 2.0:
                        rep_gain = ((2.0 / max(dist_nbr, 0.4)) ** 2) * 0.4
                        repulsion_vec += (rel_nbr / max(dist_nbr, 0.05)) * rep_gain

                # Lidar (Hanya Skema 1)
                if self.scheme == 1:
                    lidar_ranges = self.simulate_lidar(pos_self, self.yaw_smooth[i])
                    angles_world = self.yaw_smooth[i] + np.linspace(-np.pi, np.pi, len(lidar_ranges))
                    obs_mask = lidar_ranges < 4.5

                    if np.any(obs_mask):
                        close_indices = np.where(obs_mask)[0]
                        for idx in close_indices[::6]:
                            d_i = float(lidar_ranges[idx])
                            ang_i_world = float(angles_world[idx])
                            obs_pos_i = pos_self + np.array([d_i * np.cos(ang_i_world), d_i * np.sin(ang_i_world)], dtype=np.float32)
                            neighbors.append({'pos': obs_pos_i, 'vel': np.zeros(2, dtype=np.float32), 'is_static': True, 'radius': 0.35})
                            if d_i < min_dist_to_obs:
                                min_dist_to_obs = d_i
                        for idx in close_indices[::4]:
                            d_i = float(lidar_ranges[idx])
                            if d_i > 2.2:
                                continue
                            ang_i_world = float(angles_world[idx])
                            obs_rel_i = np.array([d_i * np.cos(ang_i_world), d_i * np.sin(ang_i_world)], dtype=np.float32)
                            
                            # Hanya terapkan gaya tolak jika rintangan berada di hemisfer depan pergerakan drone
                            is_front = True
                            pref_speed = np.linalg.norm(pref_vel)
                            if pref_speed > 0.1:
                                is_front = np.dot(obs_rel_i, pref_vel) > 0
                            
                            if is_front:
                                push_dir = -obs_rel_i / max(d_i, 0.05)
                                rep_gain_i = ((2.2 / max(d_i, 0.4)) ** 2) * 0.3
                                repulsion_vec += push_dir * rep_gain_i

                rep_len = np.linalg.norm(repulsion_vec)
                max_rep = self.max_speed * 0.75
                if rep_len > max_rep: repulsion_vec = (repulsion_vec / rep_len) * max_rep
                repulsion_scale = min(1.0, dist_target / 1.5)
                repulsion_vec *= repulsion_scale

                self.repulsion_smooth[i] = 0.7 * self.repulsion_smooth[i] + 0.3 * repulsion_vec
                if dist_target > 0.3:
                    pref_vel += self.repulsion_smooth[i]
                else:
                    self.repulsion_smooth[i] = np.zeros(2, dtype=np.float32)

                safe_vel = self.orca_solver.compute_orca_velocity(pos_self, vel_self, pref_vel, neighbors, None)
                ref_vx = np.clip(safe_vel[0], -self.max_speed, self.max_speed)
                ref_vy = np.clip(safe_vel[1], -1.2, 1.2)

                self.velocities[i] = np.array([ref_vx, ref_vy], dtype=np.float32)
                new_pos = pos_self + self.velocities[i] * self.dt
                new_positions.append(new_pos)

                safe_speed = np.linalg.norm(safe_vel)
                if safe_speed > 0.15 and dist_target > 0.8:
                    yaw_target = float(np.arctan2(safe_vel[1], safe_vel[0]))
                    delta_yaw = (yaw_target - self.yaw_smooth[i] + np.pi) % (2 * np.pi) - np.pi
                    self.yaw_smooth[i] += 0.25 * delta_yaw

            self.positions = new_positions

    def plot_results(self, output_path):
        plt.figure(figsize=(10, 8))
        
        # Plot Rintangan Statis (Hanya untuk Skema 1)
        if self.scheme == 1:
            for idx, obs in enumerate(self.obstacles):
                circle = plt.Circle(obs['pos'], obs['radius'], color='red', alpha=0.6, label='Rintangan' if idx==0 else "")
                plt.gca().add_patch(circle)
                plt.text(obs['pos'][0], obs['pos'][1], f"Obs {chr(65+idx)}", color='white', ha='center', va='center', weight='bold')

        colors = ['blue', 'green', 'orange', 'purple', 'brown', 'cyan', 'magenta']
        for i in range(self.active_drones):
            hist = np.array(self.history[i])
            if len(hist) > 0:
                plt.plot(hist[:, 0], hist[:, 1], color=colors[i], label=f"Drone {i+1}", linewidth=2)
                plt.scatter(hist[0, 0], hist[0, 1], color=colors[i], marker='o') # Spawn
                plt.scatter(hist[-1, 0], hist[-1, 1], color=colors[i], marker='X') # Akhir

        title_str = "Simulasi Penerbangan Swarm Drone 2D (ORCA + Repulsion)" if self.scheme == 1 else "Simulasi Head-On Collision Avoidance 2D"
        plt.title(title_str, fontsize=14, weight='bold')
        plt.xlabel("X Position (m)")
        plt.ylabel("Y Position (m)")
        plt.grid(True)
        plt.legend(loc='upper left')
        
        if self.scheme == 1:
            plt.xlim(-1.0, 11.0)
            plt.ylim(-7.0, 7.0)
        else:
            plt.xlim(-3.0, 11.0)
            plt.ylim(-3.0, 3.0)
        
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        plt.close()
        print(f"📈 Grafik trajectory disimpan ke: {output_path}")

if __name__ == '__main__':
    realtime_mode = '--realtime' in sys.argv or '-r' in sys.argv
    sim = SwarmSimulator2D()
    sim.run(realtime=realtime_mode)
    
    # Tentukan nama file output berdasarkan skema aktif
    scheme_suffix = "" if sim.scheme == 1 else "_scheme2"
    inner_dir = os.path.dirname(os.path.abspath(__file__))
    output_plot_path = os.path.join(inner_dir, f"trajectory_simulation{scheme_suffix}.png")
    sim.plot_results(output_plot_path)
    
    # Salin juga ke folder artifact agar ter-render di chat
    artifact_plot_name = f"trajectory_simulation{scheme_suffix}.png"
    artifact_plot_path = os.path.join('/home/izmaherdian/.gemini/antigravity-cli/brain/5957a601-2c4a-4ba8-a185-b5636c8ffa5c', artifact_plot_name)
    try:
        import shutil
        shutil.copy(output_plot_path, artifact_plot_path)
        print(f"📈 Grafik trajectory disalin ke artifact chat: {artifact_plot_path}")
    except Exception as e:
        print(f"Error copying plot: {e}")
