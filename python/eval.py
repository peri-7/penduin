import argparse
import numpy as np
import os.path
import time
from matplotlib import pyplot as plt
from env import PenduinEnv
from gymnasium.wrappers import TimeLimit 
from stable_baselines3 import SAC
import mujoco.viewer


def run_episode(agent, env, seed=7, viewer=None):
    # init env and lists
    a, th, th_dot = [], [], []
    t_log, th_ddot, rew = [0], [0], [0]
    t, dt = 0, env.unwrapped.dt
    max_speed  = env.unwrapped.max_speed
    max_accel  = env.unwrapped.max_accel
    th_limit = env.unwrapped.th_limit
    obs, _ = env.reset(seed=seed) # [sina, cosa, theta, alpha_dot, theta_dot]
    alpha = np.arctan2(obs[0], obs[1])
    a.append(alpha)
    th.append(obs[2]*th_limit)
    th_dot.append(obs[4]*max_speed)

    # loop 
    terminated = truncated = 0
    while not (terminated or truncated):
        # make a step
        action, _ = agent.predict(obs, deterministic=True)
        obs, reward, terminated, truncated, _ = env.step(action)
        t += dt
        # visualize on mujoco
        if viewer is not None:
            if not viewer.is_running():
                break
            viewer.sync()
            time.sleep(dt)
        # record
        alpha = np.arctan2(obs[0], obs[1])
        a.append(alpha)
        th.append(obs[2]*th_limit)
        th_dot.append(obs[4]*max_speed)
        t_log.append(t)
        th_ddot.append(action[0]*max_accel)
        rew.append(reward)

    a = np.array(a)
    th = np.array(th)
    th_dot = np.array(th_dot)
    th_ddot = np.array(th_ddot)
    rew = np.array(rew)
    t_log = np.array(t_log)
    return  a, th, th_dot, th_ddot, rew, t_log




if __name__ == "__main__":

    # i) read args: 1. file 2. watch
    parser = argparse.ArgumentParser()
    parser.add_argument("-f", "--file", help="give model 's file path", required=True)
    parser.add_argument("-m", "--mode", help="give agent 's reward function", required=True, choices=["quad", "exp"])
    parser.add_argument("-w", "--watch", help="option to watch on mujoco", action='store_true')
    args = parser.parse_args()


    # ii) create same env
    agent = SAC.load(args.file)
    here = os.path.dirname(os.path.abspath(__file__))
    xml = f"{here}/../model.xml"
    max_steps = 1000
    env = PenduinEnv(xml_path=xml, mode=args.mode)
    env = TimeLimit(env, max_episode_steps=max_steps)

    # iii) run the episode
    if args.watch:
        with mujoco.viewer.launch_passive(env.unwrapped.model, env.unwrapped.data) as viewer:
            a, th, th_dot, th_ddot, rew, t_log = run_episode(agent, env, viewer=viewer)
    else:
        a, th, th_dot, th_ddot, rew, t_log = run_episode(agent, env)

    # iv) summary report
    up = [abs(x)<np.deg2rad(10) for x in a]
    ts = t_log[up.index(1)] if any(up) else "never"
    print(f'''
            1. reward:          {sum(rew)}
            2. t_log(s):         {t_log[-1]}
            3. t_swing_up(s):   {ts}
            4. up_t_log(%):      {sum(up)/len(up)*100}%
            ''')

    # v) plots 
    fig, ax = plt.subplots(4, 1, sharex=True)
    ax[0].plot(t_log, np.rad2deg(np.unwrap(a)))
    ax[0].axhline(0, ls='--', c='k')
    ax[0].set_title(r' $\alpha$ (deg) - pendulum ')
    th_limit = np.rad2deg(env.unwrapped.th_limit)
    ax[1].plot(t_log, np.rad2deg(th))
    ax[1].axhline(th_limit, ls='--', c='k', alpha=0.5)
    ax[1].axhline(-th_limit, ls='--', c='k', alpha=0.5)
    ax[1].set_title(r' $\theta$ (deg) - arm ')
    ax[2].plot(t_log, th_ddot, 'k')
    ax2 = ax[2].twinx()
    ax2.plot(t_log, th_dot, c='k', alpha=0.5)
    ax[2].set_title(r' $\ddot\theta (rad/s^2) / \dot\theta (rad/s)$ ')
    ax[3].plot(t_log, rew)
    ax[3].set_title(' reward ')
    plt.show()




