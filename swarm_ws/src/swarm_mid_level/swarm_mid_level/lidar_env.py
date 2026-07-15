import gymnasium as gym
import numpy as np
from gymnasium import spaces

class LidarObstacleAvoidanceEnv(gym.Env):
    """
    Custom Environment for Drone Obstacle Avoidance using 2D Lidar.
    Simulates a 2D kinematic drone trying to reach a target while avoiding obstacles.
    """
    metadata = {"render_modes": ["human"]}

    def __init__(self):
        super().__init__()
        
        # Max speeds and ranges
        self.max_speed = 1.5      # m/s
        self.max_lidar_range = 10.0 # meters
        self.num_lidar_rays = 72
        self.dt = 0.1             # seconds per step
        self.target_radius = 0.3  # meters to count as reached
        self.collision_radius = 0.25 # drone size
        
        # Action space: Vx, Vy command in [-1.0, 1.0] scaled to max_speed
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(2,), dtype=np.float32
        )
        
        # Observation space: 72 (Lidar) + 2 (relative target position dx, dy) + 2 (current velocity vx, vy)
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(self.num_lidar_rays + 4,), dtype=np.float32
        )
        
        # Define obstacles (2D boxes as [xmin, ymin, xmax, ymax])
        # We define a few fixed obstacles matching the Gazebo setup
        self.obstacles = [
            np.array([1.9, 0.0, 2.1, 2.0]),    # obstacle_front-like
            np.array([-2.1, -1.0, -1.9, 1.0]),  # obstacle_left-like
            np.array([-1.0, -2.1, 1.0, -1.9]),  # obstacle_right-like
            # We also add some random boundaries
            np.array([-5.0, -5.1, 5.0, -5.0]),  # South wall
            np.array([-5.0, 5.0, 5.0, 5.1]),   # North wall
            np.array([-5.1, -5.0, -5.0, 5.0]),  # West wall
            np.array([5.0, -5.0, 5.1, 5.0]),   # East wall
        ]
        
        self.reset()

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        
        # Spawn drone randomly in safe area (near origin)
        self.drone_pos = np.array([0.0, 0.0], dtype=np.float32)
        self.drone_vel = np.array([0.0, 0.0], dtype=np.float32)
        
        # Spawn target randomly at a reasonable distance (2.0 to 4.0 meters away)
        angle = self.np_random.uniform(0, 2 * np.pi)
        dist = self.np_random.uniform(2.0, 4.0)
        self.target_pos = self.drone_pos + np.array([np.cos(angle) * dist, np.sin(angle) * dist], dtype=np.float32)
        
        # Ensure target doesn't spawn inside static obstacles
        while self._is_colliding(self.target_pos, radius=self.target_radius):
            angle = self.np_random.uniform(0, 2 * np.pi)
            dist = self.np_random.uniform(2.0, 4.0)
            self.target_pos = self.drone_pos + np.array([np.cos(angle) * dist, np.sin(angle) * dist], dtype=np.float32)
            
        self.steps = 0
        self.max_steps = 150
        
        obs = self._get_obs()
        info = {}
        return obs, info

    def step(self, action):
        self.steps += 1
        
        # Action is [Vx, Vy] reference command in [-1, 1]
        action = np.clip(action, -1.0, 1.0)
        target_vel = action * self.max_speed
        
        # Simple kinematic drone update with inertia (first order low-pass filter)
        self.drone_vel = 0.7 * self.drone_vel + 0.3 * target_vel
        self.drone_pos += self.drone_vel * self.dt
        
        # Calculate distance to target
        dist_to_target = np.linalg.norm(self.target_pos - self.drone_pos)
        
        # Check collision and calculate lidar scan
        lidar_scan = self._get_lidar_scan()
        min_lidar_dist = np.min(lidar_scan)
        is_collision = min_lidar_dist < self.collision_radius
        
        # Reward function design
        reward = 0.0
        terminated = False
        truncated = False
        
        # 1. Target progress reward (difference in distance)
        prev_dist = np.linalg.norm(self.target_pos - (self.drone_pos - self.drone_vel * self.dt))
        progress = prev_dist - dist_to_target
        reward += progress * 15.0  # Encourage moving closer
        
        # 2. Obstacle avoidance penalty
        if min_lidar_dist < 1.0:
            # Quadratic penalty for getting too close to obstacles
            reward -= 0.5 * (1.0 - min_lidar_dist) ** 2
            
        # 3. Collision penalty
        if is_collision:
            reward -= 150.0
            terminated = True
            
        # 4. Target reached reward
        if dist_to_target < self.target_radius:
            reward += 100.0
            terminated = True
            
        # 5. Time penalty (encourages speed)
        reward -= 0.1
        
        # Check step limit
        if self.steps >= self.max_steps:
            truncated = True
            
        obs = self._get_obs(lidar_scan)
        info = {
            "is_collision": is_collision,
            "target_reached": dist_to_target < self.target_radius,
            "dist_to_target": dist_to_target
        }
        
        return obs, reward, terminated, truncated, info

    def _get_obs(self, lidar_scan=None):
        if lidar_scan is None:
            lidar_scan = self._get_lidar_scan()
            
        # Relative target position in drone frame (dx, dy)
        rel_target = self.target_pos - self.drone_pos
        
        # Concatenate Lidar + target + current velocity
        obs = np.concatenate([
            lidar_scan,
            rel_target,
            self.drone_vel
        ]).astype(np.float32)
        return obs

    def _get_lidar_scan(self):
        # Calculate range for each of the 72 rays
        scan = np.ones(self.num_lidar_rays, dtype=np.float32) * self.max_lidar_range
        angles = np.linspace(-np.pi, np.pi, self.num_lidar_rays, endpoint=False)
        
        for i, angle in enumerate(angles):
            ray_dir = np.array([np.cos(angle), np.sin(angle)], dtype=np.float32)
            # Find closest intersection for this ray
            min_t = self.max_lidar_range
            for obs in self.obstacles:
                t = self._intersect_ray_box(self.drone_pos, ray_dir, obs)
                if t < min_t:
                    min_t = t
            scan[i] = min_t
        return scan

    def _intersect_ray_box(self, ray_org, ray_dir, box):
        # Ray-box intersection (Slab method)
        # box is [xmin, ymin, xmax, ymax]
        xmin, ymin, xmax, ymax = box
        
        tx1 = (xmin - ray_org[0]) / (ray_dir[0] + 1e-8)
        tx2 = (xmax - ray_org[0]) / (ray_dir[0] + 1e-8)
        
        tmin = min(tx1, tx2)
        tmax = max(tx1, tx2)
        
        ty1 = (ymin - ray_org[1]) / (ray_dir[1] + 1e-8)
        ty2 = (ymax - ray_org[1]) / (ray_dir[1] + 1e-8)
        
        tmin = max(tmin, min(ty1, ty2))
        tmax = min(tmax, max(ty1, ty2))
        
        if tmax >= tmin and tmax >= 0:
            t = tmin if tmin >= 0 else tmax
            return t
        return self.max_lidar_range

    def _is_colliding(self, pos, radius):
        # Simple check if a circle collides with any obstacle box
        for obs in self.obstacles:
            xmin, ymin, xmax, ymax = obs
            # Find closest point on box to circle center
            closest_x = np.clip(pos[0], xmin, xmax)
            closest_y = np.clip(pos[1], ymin, ymax)
            # Distance from closest point to circle center
            dist = np.linalg.norm(np.array([closest_x, closest_y]) - pos)
            if dist < radius:
                return True
        return False
