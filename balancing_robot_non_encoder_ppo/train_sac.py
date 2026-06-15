"""
STEP 4 — train_ppo.py

Algorithm: PPO — Proximal Policy Optimization

Why PPO:
    Stable and reliable on continuous-control tasks.
    On-policy updates reduce instability.
    Good baseline for balancing robots when tuned carefully.

Terminal 1: python train_ppo.py
Terminal 2: python watch_training.py
Terminal 3: tensorboard --logdir ./logs/tensorboard/

train_ppo.py — FINAL VERSION
PPO + VecNormalize + SaveVecNormalizeCallback
.pkl saved every 10k steps — Ctrl+C safe
"""

from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
from stable_baselines3.common.callbacks import (
    EvalCallback,
    CheckpointCallback,
    BaseCallback,
    StopTrainingOnRewardThreshold
)
from stable_baselines3.common.env_checker import check_env
from balance_env import BalanceBotEnv
import os


class SaveVecNormalizeCallback(BaseCallback):
    """
    Saves VecNormalize stats every save_freq steps.
    Ctrl+C at any point still leaves a valid .pkl behind.
    """
    def __init__(self, save_freq, save_path, verbose=0):
        super().__init__(verbose)
        self.save_freq = save_freq
        self.save_path = save_path

    def _on_step(self) -> bool:
        if self.n_calls % self.save_freq == 0:
            path = os.path.join(self.save_path, "vecnormalize.pkl")
            self.training_env.save(path)

            if self.verbose > 0:
                print(f"Saved VecNormalize stats to {path}")

        return True


print("Checking environment...")
env = BalanceBotEnv()
check_env(env)
env.close()
print("OK\n")


os.makedirs("./models/best/", exist_ok=True)
os.makedirs("./models/checkpoints/", exist_ok=True)
os.makedirs("./logs/tensorboard/", exist_ok=True)


train_env = DummyVecEnv([lambda: BalanceBotEnv()])

train_env = VecNormalize(
    train_env,
    norm_obs=True,
    norm_reward=True,
    clip_obs=10.0,
    clip_reward=10.0
)


eval_env = DummyVecEnv([lambda: BalanceBotEnv()])

eval_env = VecNormalize(
    eval_env,
    norm_obs=True,
    norm_reward=False,
    clip_obs=10.0,
    training=False
)


# Stop training when mean reward exceeds 2100 consistently.
stop_callback = StopTrainingOnRewardThreshold(
    reward_threshold=2100,
    verbose=1
)


eval_callback = EvalCallback(
    eval_env,
    callback_on_new_best=stop_callback,
    best_model_save_path="./models/best/",
    log_path="./logs/",
    eval_freq=5_000,
    n_eval_episodes=10,
    deterministic=True,
    verbose=1
)


checkpoint_callback = CheckpointCallback(
    save_freq=10_000,
    save_path="./models/checkpoints/",
    name_prefix="balancebot_ppo"
)


vecnorm_callback = SaveVecNormalizeCallback(
    save_freq=10_000,
    save_path="./models/",
    verbose=1
)


model = PPO(
    "MlpPolicy",
    train_env,
    verbose=1,

    policy_kwargs=dict(
        net_arch=dict(
            pi=[256, 256],
            vf=[256, 256]
        )
    ),

    # PPO-specific parameters
    learning_rate=1e-4,
    n_steps=2048,
    batch_size=512,
    n_epochs=10,

    gamma=0.99,
    gae_lambda=0.95,

    clip_range=0.2,
    ent_coef=0.01,
    vf_coef=0.5,
    max_grad_norm=0.5,

    tensorboard_log="./logs/tensorboard/"
)


print("Starting PPO training — with position penalty fix...")
print("vecnormalize.pkl saved every 10k steps — Ctrl+C safe")
print("Terminal 2: python watch_training.py")
print("Terminal 3: tensorboard --logdir ./logs/tensorboard/\n")


model.learn(
    total_timesteps=500_000,
    callback=[eval_callback, checkpoint_callback, vecnorm_callback],
    progress_bar=True
)


model.save("./models/balancebot_ppo_final")
train_env.save("./models/vecnormalize.pkl")


print("\nDone.")
print("Best model : ./models/best/best_model.zip")
print("Final model: ./models/balancebot_ppo_final.zip")
print("Norm stats : ./models/vecnormalize.pkl")


train_env.close()
eval_env.close()