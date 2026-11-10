import mujoco
import numpy as np
import os.path
import argparse
import scipy as sp
from matplotlib import pyplot as plt
import mujoco.viewer
import time

from env import PenduinEnv
from gymnasium.wrappers import TimeLimit 



here = os.path.dirname(os.path.abspath(__file__))
xml = f"{here}/../model.xml"
m = mujoco.MjModel.from_xml_path(xml)
data = mujoco.MjData(m)    # the changing state: positions, velocities, controls, ...
pend = m.body("pendulum")


mass = pend.mass[0]     
com  = pend.ipos        # com in pendulum body frame
pos  = pend.pos         #pendulum frame position in ARM frame
g    = -m.opt.gravity[2] 
l1 = pos[0] + com[2]
l2 = -com[0] # lm2 from pendulum joint

Rot = np.zeros(9)
mujoco.mju_quat2Mat(Rot, pend.iquat)
Rot = Rot.reshape(3, 3)
I_com = Rot @ np.diag(pend.inertia) @ Rot.T


def calc_A_B():
    Mg = mass*g*l2 
    Jp = mass*l2**2 + I_com[2][2]
    Mc = mass*l1*l2 + I_com[2][0] 
    A = [[0, 0, 1, 0], [0, 0, 0, 1], [0, 0, 0, 0], [0, Mg/Jp, 0, 0]]
    B = [0, 0, 1, Mc/Jp]
    return np.array(A), np.array(B).reshape(4, 1)


def run_episode(K, env, seed=7, viewer=None):
    # init env and lists
    a, th, th_dot, a_dot = [], [], [], []
    t_log, th_ddot, rew = [0], [0], [0]
    t, dt = 0, env.unwrapped.dt
    max_speed  = env.unwrapped.max_speed
    max_accel  = env.unwrapped.max_accel
    th_limit = env.unwrapped.th_limit
    a_dot_denom = env.unwrapped.a_dot_denom 
    obs, _ = env.reset(seed=seed, options={"alpha0": 0}) # [sina, cosa, theta, alpha_dot, theta_dot]
    alpha = np.arctan2(obs[0], obs[1])
    a.append(alpha)
    a_dot.append(obs[3]*a_dot_denom)
    th.append(obs[2]*th_limit)
    th_dot.append(obs[4]*max_speed)

    # loop 
    terminated = truncated = 0
    while not (terminated or truncated):
        # make a step
        action = np.array(-K@[th[-1], a[-1], th_dot[-1], a_dot[-1]])
        action[0] = action[0]/max_accel
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
        a_dot.append(obs[3]*a_dot_denom)
        th_dot.append(obs[4]*max_speed)
        t_log.append(t)
        th_ddot.append(action[0]*max_accel)
        rew.append(reward)
        

    a = np.array(a)
    a_dot = np.array(a_dot)
    th = np.array(th)
    th_dot = np.array(th_dot)
    th_ddot = np.array(th_ddot)
    rew = np.array(rew)
    t_log = np.array(t_log)
    return  a, a_dot, th, th_dot, th_ddot, rew, t_log


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-m", "--mode", help="give agent 's reward function", required=True, choices=["quad", "exp"])
    parser.add_argument("-w", "--watch", help="option to watch on mujoco", action='store_true')
    args = parser.parse_args()


    max_steps = 1000
    env = PenduinEnv(xml_path=xml, mode=args.mode)
    env = TimeLimit(env, max_episode_steps=max_steps)
    dt = env.unwrapped.dt

    C = np.eye(4)
    D = np.zeros((4, 1))
    A, B = calc_A_B()

    plant = (A, B, C, D)
    (Ad, Bd, Cd, Dd, dt) = sp.signal.cont2discrete(plant, dt, "zoh")
    Q = [[1/20**2, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]]
    R = 1/200**2
    P = sp.linalg.solve_discrete_are(Ad, Bd, Q, R)
    K = np.linalg.inv(R + Bd.T @ P @ Bd) @ (Bd.T @ P @ Ad)
    print(K, np.abs(np.linalg.eigvals(Ad - Bd @ K)))

    if args.watch:
        with mujoco.viewer.launch_passive(env.unwrapped.model, env.unwrapped.data) as viewer:
            a, a_dot, th, th_dot, th_ddot, rew, t_log = run_episode(K, env, viewer=viewer)
    else:
        a, a_dot, th, th_dot, th_ddot, rew, t_log = run_episode(K, env)

    # iv) summary report + smoothness report
    up = [abs(x)<np.deg2rad(10) for x in a]
    if any(up):
        ts_index = up.index(1)
        ts = t_log[ts_index]
        ts_index = min(len(t_log)-2, ts_index+100)
        actions = th_ddot[ts_index:]
        vel = th_dot[ts_index:]
        action_diffs = np.diff(actions)
        action_rms_diff = np.sqrt(np.mean(action_diffs**2))
        action_signs = np.sign(actions)
        action_zc = np.sum(np.diff(action_signs) != 0) # zero_crossings
        vel_var = np.var(vel)
    else:
        ts = "never"
        action_rms_diff = np.inf
        action_zc = np.inf # zero_crossings
        vel_var = np.inf
    print(f'''
    Summary:
            1. reward:          {sum(rew)}
            2. t_log(s):         {t_log[-1]}
            3. t_swing_up(s):   {ts}
            4. up_t_log(%):      {sum(up)/len(up)*100}%
    Smoothness:
            1. ActionDiff RMS:    {action_rms_diff}
            2. Action ZeroCr.:    {action_zc}
            3. ArmVelocity Var:    {vel_var}
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

