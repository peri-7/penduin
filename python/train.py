import numpy as np
import os.path
from env import PenduinEnv
from gymnasium.wrappers import TimeLimit 
from stable_baselines3 import SAC
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.callbacks import CheckpointCallback

if __name__ == "__main__":

    # constants
    max_steps = 1000
    total_steps = 200_000 # 200 episodes
    check_freq = 25_000 # 8 checkpoints
    name = "sac_1"

    # paths
    here = os.path.dirname(os.path.abspath(__file__))
    xml = f"{here}/../model.xml"
    models = f"{here}/../models/"
    runs = f"{here}/../runs/"
    checks = f"{here}/../checkpoints/"
    print("> Scipt in:", here)

    # environment
    env = PenduinEnv(xml_path=xml)
    env = TimeLimit(env, max_episode_steps=max_steps)
    check_env(env)
    print("> Env was wrapped")

    # training
    '''agent = SAC("MlpPolicy", env, verbose=1, tensorboard_log=runs)
    print("> Training starts!")
    check = CheckpointCallback(save_freq=check_freq, save_path=checks, name_prefix=name)
    agent.learn(total_timesteps=total_steps, callback=check, tb_log_name=name)'''
    agent = SAC.load(f"{models}/sac_1.zip", env=env, tensorboard_log=runs)
    check = CheckpointCallback(save_freq=check_freq, save_path=checks, name_prefix=name)
    agent.learn(total_timesteps=200_000, reset_num_timesteps=False, callback=check, tb_log_name=name)
    print("> Agent trained gracefully")
    agent.save(os.path.join(models, name))


