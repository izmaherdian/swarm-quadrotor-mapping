#!/usr/bin/env python3
import sys
import os
import math
import numpy as np
import matplotlib.pyplot as plt

# Tambahkan path ke folder swarm_mid_level agar bisa mengimpor ORCASolver2D
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from collision_avoidance_node import ORCASolver2D

class SwarmSimulator2D:
    def __init__(self):
        # Definisikan rintangan sesuai model Gazebo (x, y, radius=0.5m)
        self.obstacles = [
            {'pos': np.array([2.0, -1.5], dtype=np.float32), 'radius': 0.5},
            {'pos': np.array([3.5, 1.5], dtype=np.float32), 'radius': 0.5},
            {'pos': np.array([5.0, -1.5], dtype=np.float32), 'radius': 0.5},
            {'pos': np.array([6.5, 2.5], dtype=np.float32), 'radius': 0.5}
        ]

        # Konfigurasi Drone (1 s/d 7)
        self.num_drones = 7
        self.max_speed = 2.5
        self.safety_radius = 0.8
        self.time_horizon = 5.0
        self.dt = 0.1
        self.steps = 300  # 30 detik simulasi

        # Posisi awal (Spawn) dan Target
        self.positions = []
        self.velocities = []
        self.targets = []
        self.yaw = []
        self.yaw_smooth = []
        self.repulsion_smooth = []

        spacing = 2.0
        for i in range(1, self.num_drones + 1):
            spawn_y = float((i - 4.0) * spacing)
            self.positions.append(np.array([0.0, spawn_y], dtype=np.float32))
            self.velocities.append(np.array([0.0, 0.0], dtype=np.float32))
            self.targets.append(np.array([10.0, spawn_y], dtype=np.float32))
            self.yaw.append(0.0)
            self.yaw_smooth.append(0.0)
            self.repulsion_smooth.append(np.array([0.0, 0.0], dtype=np.float32))

        # Inisialisasi ORCA Solver
        self.orca_solver = ORCASolver2D(
            time_horizon=self.time_horizon,
            safety_radius=self.safety_radius,
            max_speed=self.max_speed
        )

        # Log perjalanan untuk plotting
        self.history = {i: [] for i in range(self.num_drones)}

    def simulate_lidar(self, drone_pos, yaw):
        """Simulasikan pembacaan Lidar 2D (360 ray) terhadap 4 rintangan lingkaran dengan arah Yaw drone"""
        num_rays = 360
        angles_body = np.linspace(-np.pi, np.pi, num_rays)
        ranges = np.ones(num_rays, dtype=np.float32) * 10.0  # Default max range 10m

        for idx, angle_body in enumerate(angles_body):
            angle_world = yaw + angle_body
            ray_dir = np.array([np.cos(angle_world), np.sin(angle_world)], dtype=np.float32)
            # Hitung perpotongan sinar dengan masing-masing rintangan berbentuk lingkaran
            for obs in self.obstacles:
                pos_rel = obs['pos'] - drone_pos
                projection = np.dot(pos_rel, ray_dir)
                if projection < 0:
                    continue  # Arah berlawanan

                closest_approach_sq = np.dot(pos_rel, pos_rel) - projection**2
                if closest_approach_sq < obs['radius']**2:
                    # Sinar memotong lingkaran rintangan
                    chord = np.sqrt(obs['radius']**2 - closest_approach_sq)
                    dist = projection - chord
                    if 0.1 < dist < ranges[idx]:
                        ranges[idx] = dist
        return ranges

    def run(self, realtime=False):
        print(f"🤖 Memulai Simulasi Swarm ORCA 2D (Real-Time={realtime})...")
        collisions = 0
        min_dist_to_obs = 999.0

        if realtime:
            plt.ion()
            fig, ax = plt.subplots(figsize=(10, 8))
            # Plot Rintangan Statis
            for idx, obs in enumerate(self.obstacles):
                circle = plt.Circle(obs['pos'], obs['radius'], color='red', alpha=0.6)
                ax.add_patch(circle)
                ax.text(obs['pos'][0], obs['pos'][1], f"Obs {chr(65+idx)}", color='white', ha='center', va='center', weight='bold')

            # Setup visualisasi drone dan lintasan
            colors = ['blue', 'green', 'orange', 'purple', 'brown', 'cyan', 'magenta']
            traj_lines = []
            drone_markers = []
            safety_circles = []
            for i in range(self.num_drones):
                line, = ax.plot([], [], color=colors[i], linewidth=2, label=f"Drone {i+1}")
                traj_lines.append(line)
                marker, = ax.plot([], [], color=colors[i], marker='o', markersize=8)
                drone_markers.append(marker)
                # Lingkaran fisik drone (radius 0.4m)
                circle = plt.Circle((0,0), 0.4, color=colors[i], fill=False, linestyle='--', alpha=0.4)
                ax.add_patch(circle)
                safety_circles.append(circle)

            lidar_scatter = ax.scatter([], [], color='red', s=12, alpha=0.6, marker='x', label="Point Cloud Lidar")

            ax.set_title("Real-Time Simulasi Swarm Drone 2D (ORCA + Repulsion)", fontsize=14, weight='bold')
            ax.set_xlabel("X Position (m)")
            ax.set_ylabel("Y Position (m)")
            ax.grid(True)
            ax.legend(loc='upper left')
            ax.set_xlim(-1.0, 11.0)
            ax.set_ylim(-7.0, 7.0)

        for step in range(self.steps):
            new_positions = []
            all_lidar_points_x = []
            all_lidar_points_y = []
            
            for i in range(self.num_drones):
                pos_self = self.positions[i]
                vel_self = self.velocities[i]
                target = self.targets[i]

                # Simpan histori posisi
                self.history[i].append(pos_self.copy())

                # 1. Preferred Velocity
                rel_target = target - pos_self
                dist_target = np.linalg.norm(rel_target)
                if dist_target < 0.15:
                    pref_vel = np.zeros(2, dtype=np.float32)
                else:
                    speed = min(self.max_speed, dist_target * 0.8)
                    pref_vel = (rel_target / dist_target) * speed

                # 2. Tetangga (Drone Lain)
                neighbors = []
                repulsion_vec = np.zeros(2, dtype=np.float32)
                
                for j in range(self.num_drones):
                    if i == j:
                        continue
                    pos_nbr = self.positions[j]
                    vel_nbr = self.velocities[j]
                    rel_nbr = pos_self - pos_nbr
                    dist_nbr = np.linalg.norm(rel_nbr)

                    neighbors.append({
                        'pos': pos_nbr,
                        'vel': vel_nbr,
                        'is_static': False
                    })

                    # Gaya Tolak Tetangga
                    if dist_nbr < 2.0:
                        rep_gain = ((2.0 / max(dist_nbr, 0.4)) ** 2) * 0.4
                        repulsion_vec += (rel_nbr / max(dist_nbr, 0.05)) * rep_gain

                # 3. Lidar Obstacles (Point-Cloud Spheres)
                lidar_ranges = self.simulate_lidar(pos_self, self.yaw_smooth[i])
                angles_world = self.yaw_smooth[i] + np.linspace(-np.pi, np.pi, len(lidar_ranges))
                obs_mask = lidar_ranges < 4.5

                if np.any(obs_mask):
                    close_indices = np.where(obs_mask)[0]
                    # Representasi Point Cloud
                    for idx in close_indices[::6]:
                        d_i = float(lidar_ranges[idx])
                        ang_i_world = float(angles_world[idx])
                        obs_pos_i = pos_self + np.array([d_i * np.cos(ang_i_world), d_i * np.sin(ang_i_world)], dtype=np.float32)
                        
                        neighbors.append({
                            'pos': obs_pos_i,
                            'vel': np.zeros(2, dtype=np.float32),
                            'is_static': True,
                            'radius': 0.35
                        })

                        if d_i < min_dist_to_obs:
                            min_dist_to_obs = d_i

                        if realtime:
                            all_lidar_points_x.append(obs_pos_i[0])
                            all_lidar_points_y.append(obs_pos_i[1])

                    # Gaya Tolak Rintangan (Hanya aktif untuk jarak dekat < 2.2m untuk mencegah dorongan rintangan belakang)
                    for idx in close_indices[::4]:
                        d_i = float(lidar_ranges[idx])
                        if d_i > 2.2:
                            continue
                        ang_i_world = float(angles_world[idx])
                        obs_rel_i = np.array([d_i * np.cos(ang_i_world), d_i * np.sin(ang_i_world)], dtype=np.float32)
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

                # Saring chattering
                self.repulsion_smooth[i] = 0.7 * self.repulsion_smooth[i] + 0.3 * repulsion_vec
                
                # Tambahkan gaya tolak hanya jika belum dekat target untuk menghindari drifting saat melayang diam
                if dist_target > 0.3:
                    pref_vel += self.repulsion_smooth[i]
                else:
                    self.repulsion_smooth[i] = np.zeros(2, dtype=np.float32)

                # 4. ORCA safe velocity
                safe_vel = self.orca_solver.compute_orca_velocity(
                    pos_self=pos_self,
                    vel_self=vel_self,
                    pref_vel=pref_vel,
                    neighbors=neighbors,
                    lidar_lines=None
                )

                # Cap speed (samakan dengan limit node ROS 2 sebenarnya)
                ref_vx = np.clip(safe_vel[0], -self.max_speed, self.max_speed)
                ref_vy = np.clip(safe_vel[1], -self.max_speed, self.max_speed)

                self.velocities[i] = np.array([ref_vx, ref_vy], dtype=np.float32)
                
                # Update posisi (Euler Integration)
                new_pos = pos_self + self.velocities[i] * self.dt
                new_positions.append(new_pos)

                # Update heading (Yaw)
                safe_speed = np.linalg.norm(safe_vel)
                if safe_speed > 0.15 and dist_target > 0.8:
                    yaw_target = float(np.arctan2(safe_vel[1], safe_vel[0]))
                    delta_yaw = (yaw_target - self.yaw_smooth[i] + np.pi) % (2 * np.pi) - np.pi
                    self.yaw_smooth[i] += 0.25 * delta_yaw

            # Update seluruh posisi drone secara serentak
            self.positions = new_positions

            # Cek Tabrakan Antar Drone
            for a in range(self.num_drones):
                for b in range(a + 1, self.num_drones):
                    dist = np.linalg.norm(self.positions[a] - self.positions[b])
                    if dist < 0.5:  # Tabrakan fisik ujung baling-baling
                        collisions += 1

            if realtime:
                # Update visualisasi plot
                for i in range(self.num_drones):
                    hist = np.array(self.history[i])
                    if len(hist) > 0:
                        traj_lines[i].set_data(hist[:, 0], hist[:, 1])
                    drone_markers[i].set_data([self.positions[i][0]], [self.positions[i][1]])
                    safety_circles[i].set_center((self.positions[i][0], self.positions[i][1]))
                
                if all_lidar_points_x:
                    lidar_scatter.set_offsets(np.column_stack((all_lidar_points_x, all_lidar_points_y)))
                else:
                    lidar_scatter.set_offsets(np.empty((0, 2)))
                
                fig.canvas.draw()
                fig.canvas.flush_events()
                plt.pause(0.01)

        print("✅ Simulasi Selesai!")
        print(f"📊 Total Tabrakan Antar Drone: {collisions}")
        print(f"📊 Jarak Terdekat ke Rintangan Statis: {min_dist_to_obs:.2f}m")
        
        if realtime:
            plt.ioff()
            plt.show()
            
        return collisions, min_dist_to_obs

    def plot_results(self, output_path):
        plt.figure(figsize=(10, 8))
        
        # Plot Rintangan
        for idx, obs in enumerate(self.obstacles):
            circle = plt.Circle(obs['pos'], obs['radius'], color='red', alpha=0.6, label='Rintangan' if idx==0 else "")
            plt.gca().add_patch(circle)
            plt.text(obs['pos'][0], obs['pos'][1], f"Obs {chr(65+idx)}", color='white', ha='center', va='center', weight='bold')

        # Plot Jalur Terbang Masing-masing Drone
        colors = ['blue', 'green', 'orange', 'purple', 'brown', 'cyan', 'magenta']
        for i in range(self.num_drones):
            hist = np.array(self.history[i])
            plt.plot(hist[:, 0], hist[:, 1], color=colors[i], label=f"Drone {i+1}", linewidth=2)
            plt.scatter(hist[0, 0], hist[0, 1], color=colors[i], marker='o') # Spawn
            plt.scatter(hist[-1, 0], hist[-1, 1], color=colors[i], marker='X') # Akhir

        plt.title("Simulasi Penerbangan Swarm Drone 2D (ORCA + Repulsion)", fontsize=14, weight='bold')
        plt.xlabel("X Position (m)")
        plt.ylabel("Y Position (m)")
        plt.grid(True)
        plt.legend(loc='upper left')
        plt.xlim(-1.0, 11.0)
        plt.ylim(-7.0, 7.0)
        
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        plt.close()
        print(f"📈 Grafik trajectory disimpan ke: {output_path}")

if __name__ == '__main__':
    realtime_mode = '--realtime' in sys.argv or '-r' in sys.argv
    sim = SwarmSimulator2D()
    sim.run(realtime=realtime_mode)
    
    # Simpan plot di workspace root agar user bisa langsung lihat & buka
    workspace_plot_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../trajectory_simulation.png'))
    sim.plot_results(workspace_plot_path)
    
    # Salin juga ke folder artifact agar ter-render di chat
    artifact_plot_path = '/home/izmaherdian/.gemini/antigravity-cli/brain/5957a601-2c4a-4ba8-a185-b5636c8ffa5c/trajectory_simulation.png'
    try:
        import shutil
        shutil.copy(workspace_plot_path, artifact_plot_path)
        print(f"📈 Grafik trajectory disalin ke artifact chat: {artifact_plot_path}")
    except Exception as e:
        print(f"Error copying plot: {e}")
