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
    total_steps = 1_000_000 
    check_freq = 25_000 
    seed = 1
    version = 1
    mode = "exp"
    name = f"sac_QR101_k05_ft"
    
    # paths
    here = os.path.dirname(os.path.abspath(__file__))
    xml = f"{here}/../model.xml"
    models = f"{here}/../models/"
    runs = f"{here}/../runs/"
    checks = f"{here}/../checkpoints/"
    print("> Scipt in:", here)

    # environment
    env = PenduinEnv(xml_path=xml, mode=mode, a_dot_k=0.5)
    env = TimeLimit(env, max_episode_steps=max_steps)
    check_env(env)
    print("> Env was wrapped")

    # new training
    policy_kwargs = dict(net_arch=[128, 128])
    agent = SAC("MlpPolicy", env, seed=seed, verbose=1, policy_kwargs=policy_kwargs, tensorboard_log=runs)
    print(f'''
    > Training starts!
                > reward: {mode}
                > version: {version}
                > seed: {seed}
        ''')
    check = CheckpointCallback(save_freq=check_freq, save_path=checks, name_prefix=name)
    # agent.learn(total_timesteps=total_steps, callback=check, tb_log_name=name)
    # extend training / perhaps it doesnt work very well because every re-run forgets the previous expiriences
    agent = SAC.load(f"{models}/sac_QR101_675000_steps.zip", env=env, tensorboard_log=runs)
    check = CheckpointCallback(save_freq=check_freq, save_path=checks, name_prefix=name)
    agent.learn(total_timesteps=total_steps, reset_num_timesteps=True, callback=check, tb_log_name=name)
    
    print("> Agent trained gracefully")
    agent.save(os.path.join(models, name))


