import argparse
import glob
import os.path
import re
import numpy as np
from env import PenduinEnv
from eval import run_episode
from gymnasium.wrappers import TimeLimit
from stable_baselines3 import SAC


def smoothness(a, th_dot, th_ddot):
    # same as eval.py: start 100 ticks (1 s) after the first time |alpha| < 10 deg
    up = np.abs(a) < np.deg2rad(10)
    if not up.any():
        return None
    ts_index = min(len(a) - 2, int(np.argmax(up)) + 100)
    action_rms_diff = np.sqrt(np.mean(np.diff(th_ddot[ts_index:])**2))
    vel_var = np.var(th_dot[ts_index:])
    up_after = up[ts_index:].mean()  # fraction of the balancing part spent upright
    return action_rms_diff, vel_var, up_after


def evaluate(path, env, seeds, max_steps, min_up):
    # mean metrics over all seeds; None if any seed fails to swing up and stay up
    agent = SAC.load(path, device="cpu")
    rms, var = [], []
    for seed in seeds:
        a, th, th_dot, th_ddot, rew, t_log = run_episode(agent, env, seed=seed)
        m = smoothness(a, th_dot, th_ddot)
        if m is None or len(a) < max_steps + 1 or m[2] < min_up:  # never up / hit the arm limit / fell
            return None
        rms.append(m[0])
        var.append(m[1])
    return np.mean(rms), np.mean(var)


def steps_of(path):
    match = re.search(r"_(\d+)_steps\.zip$", path)
    return int(match.group(1)) if match else float("inf")  # final model sorts last


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("-n", "--name", help="run name (checkpoint prefix), e.g. sac_QR0501", required=True)
    parser.add_argument("-m", "--mode", help="reward function", default="exp", choices=["quad", "exp"])
    parser.add_argument("-s", "--seeds", help="episodes per checkpoint", type=int, default=5)
    parser.add_argument("-u", "--min_up", help="min upright fraction while balancing", type=float, default=0.95)
    args = parser.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))
    xml = f"{here}/../model.xml"
    max_steps = 1000
    env = PenduinEnv(xml_path=xml, mode=args.mode)
    env = TimeLimit(env, max_episode_steps=max_steps)

    # all checkpoints of the run, plus the final model if it exists
    paths = glob.glob(f"{here}/../checkpoints/{args.name}_*_steps.zip")
    final = f"{here}/../models/{args.name}.zip"
    if os.path.exists(final):
        paths.append(final)
    paths.sort(key=steps_of)
    if not paths:
        raise SystemExit(f"no checkpoints found for '{args.name}'")

    # evaluate
    seeds = list(range(args.seeds))
    results = {}
    for path in paths:
        res = evaluate(path, env, seeds, max_steps, args.min_up)
        label = os.path.basename(path)
        if res is None:
            print(f"  {label:40s} failed")
        else:
            print(f"  {label:40s} ActionDiff RMS {res[0]:8.2f}   ArmVel Var {res[1]:8.4f}")
            results[path] = res
    if not results:
        raise SystemExit("no checkpoint swung up and balanced on every seed")

    # rank-sum: the two metrics have different units, so compare ranks, not values
    names = list(results)
    rms = np.array([results[p][0] for p in names])
    var = np.array([results[p][1] for p in names])
    rank_sum = rms.argsort().argsort() + var.argsort().argsort()
    order = np.lexsort((rms, rank_sum))  # sort by rank-sum, ties by ActionDiff RMS

    print("\nRanking (rank-sum of ActionDiff RMS and ArmVel Var, lower is better):")
    for i in order[:5]:
        print(f"  {os.path.basename(names[i]):40s} {rms[i]:8.2f}   {var[i]:8.4f}   rank-sum {rank_sum[i]}")
    best = os.path.relpath(names[order[0]])
    print(f"\nBest: {best}\nWatch it: python {os.path.relpath(here)}/eval.py -f {best} -m {args.mode} -w")
