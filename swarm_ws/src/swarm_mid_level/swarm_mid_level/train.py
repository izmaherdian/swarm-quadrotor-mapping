import os
import torch
import torch.nn as nn
import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import EvalCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv

# Import our custom environment
from swarm_mid_level.lidar_env import LidarObstacleAvoidanceEnv

class SB3PolicyONNXWrapper(nn.Module):
    """
    Wrapper class to export Stable-Baselines3 PPO Actor Policy to ONNX.
    Only exports the deterministic action output pathway.
    """
    def __init__(self, policy):
        super().__init__()
        self.features_extractor = policy.features_extractor
        self.mlp_extractor = policy.mlp_extractor
        self.action_net = policy.action_net

    def forward(self, observation):
        # Extract features
        features = self.features_extractor(observation)
        # Extract latent policy features
        latent_pi, _ = self.mlp_extractor(features)
        # Output actions
        actions = self.action_net(latent_pi)
        return actions

def train():
    # Directories
    ws_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../"))
    log_dir = os.path.join(ws_dir, "src/swarm_mid_level/tb_logs")
    model_dir = os.path.join(ws_dir, "src/swarm_mid_level/models")
    os.makedirs(log_dir, exist_ok=True)
    os.makedirs(model_dir, exist_ok=True)

    # Instantiate environments
    env = LidarObstacleAvoidanceEnv()
    env = Monitor(env)
    
    # Vectorized environment for Stable Baselines
    vec_env = DummyVecEnv([lambda: env])

    print("Initializing PPO Model...")
    model = PPO(
        "MlpPolicy",
        vec_env,
        learning_rate=3e-4,
        n_steps=2048,
        batch_size=64,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.01,
        verbose=1,
        tensorboard_log=log_dir
    )

    # Train for 200,000 timesteps (takes ~1-2 mins on typical CPU)
    total_timesteps = 200000
    print(f"Starting training for {total_timesteps} steps...")
    model.learn(total_timesteps=total_timesteps, progress_bar=True)
    
    # Save the native zip model
    model_path = os.path.join(model_dir, "ppo_lidar_avoidance")
    model.save(model_path)
    print(f"Saved native SB3 model to: {model_path}.zip")

    # Export to ONNX
    print("Exporting model to ONNX format...")
    # Extract the actor network
    actor = SB3PolicyONNXWrapper(model.policy)
    actor.eval()

    # Create dummy input based on observation space shape (360 Lidar + 4 stats)
    dummy_input = torch.randn(1, 364, dtype=torch.float32)
    onnx_path = os.path.join(model_dir, "ppo_lidar_avoidance.onnx")

    torch.onnx.export(
        actor,
        dummy_input,
        onnx_path,
        export_params=True,
        opset_version=12,
        do_constant_folding=True,
        input_names=['observation'],
        output_names=['action'],
        dynamic_axes={
            'observation': {0: 'batch_size'},
            'action': {0: 'batch_size'}
        }
    )
    print(f"Successfully exported ONNX policy model to: {onnx_path}")

if __name__ == "__main__":
    train()
