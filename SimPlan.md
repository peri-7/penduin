# Penduin — Simulation & RL Plan

Rotary inverted (Furuta) pendulum, controlled by RL. The real robot isn't finished yet, so we
build and train everything in MuJoCo first, designed so it transfers to the hardware later.

## Working style
- Small steps. Claude explains and guides, the user writes the code, Claude reviews.
- Don't move on until the current piece is understood and runs.

## Hardware (see `list.md`)
- Arm: NEMA17 stepper (42BYGHW804) + TMC2209 driver. **No encoder on the arm**: its angle comes
  from counting steps (trusted, as long as no steps are skipped).
- Pendulum: AS5600 magnetic encoder, 12-bit (4096 counts/rev, about 0.09°).
- Controller: Raspberry Pi Pico 2 W (RP2350, has an FPU).
- Arm range limited to ±120° (encoder wires twist).

## Model (`model.xml`)
- Joints: `motor` (arm, theta, range ±120°) and `bearing` (pendulum, alpha). **alpha = 0 is upright**,
  alpha = π is hanging down.
- Actuator: velocity actuator on `motor`, `ctrl` = arm speed in rad/s, ctrlrange ±20,
  kv=0.05, force capped at 0.4 N·m.
- Physics timestep 2 ms.
- Measured in sim: small-swing period ≈ 0.58 s, a 0.1 rad swing decays to 0.014 rad in 5 s.
  Compare these with the real robot later (free-swing test).
- Known approximation: the soft velocity actuator lets the arm give a little, which drains the
  pendulum's energy. A real stepper is nearly rigid. Revisit during system identification.

## Files to build (in order)
1. **`python/env.py`**: Gymnasium environment (in progress).
   - `__init__`: load the model, define the action space and observation space.
   - `reset()`: start an episode (hanging down for swing-up, or near upright for balance-only).
   - `step(action)`: action → motor command, run several physics steps, return
     `obs, reward, terminated, truncated, info`.
   - `_get_obs()`: sensor model (what the real robot measures, with encoder rounding and noise).
   - `_reward()`: pendulum up, arm centered, smooth actions; penalty for hitting the arm limit.
2. **`python/classic_control.py`**: energy-based swing-up + LQR balance. No learning; checks that
   the env and model are sane, gives a baseline, and is a safe first controller on hardware.
3. **`python/train.py`**: Stable-Baselines3 (SAC to start), parallel envs, TensorBoard logging,
   saved into `models/`.
4. **`python/evaluate.py`**: load a policy, watch it in the viewer, plot angles and actions.
5. Later (with hardware): domain randomization in `reset()`, `sysid.py` (fit model params to real
   recordings), export the policy to the Pico, and firmware in `firmware/`.

## Decisions so far
- **Observation** (only what the real robot can measure):
  - alpha: pendulum angle from the AS5600. Given to the network as **sin(alpha), cos(alpha)**
    so there's no jump at ±π.
  - alpha_dot: finite difference of alpha over the control period dt (handle wrap-around).
  - theta: arm angle from step counting (trusted).
  - theta_dot: arm speed. Known exactly, because it is the speed we command.
  - → obs = `[sin α, cos α, θ, α̇, θ̇]` (5 values, this order).
- **Action**: the policy outputs an **arm acceleration** θ̈ (scaled from [-1, 1]). The env
  integrates it into a velocity command `θ̇ += θ̈·dt` (clipped) that is sent to the velocity
  actuator. Reason: a stepper skips steps if its speed jumps, so the policy should only change
  the speed smoothly. On the Pico, the same integration produces the step rate.
- Control rate: **100 Hz** (dt = 10 ms = 5 physics steps of 2 ms, `frame_skip = 5`).
- Limits: `max_accel = 200` rad/s², `max_speed = 20` rad/s.- Policy runs on: tentatively the Pico (small MLP). Undecided.

## Status
- [x] MuJoCo model + `python/sim_demo.py` viewer demo
- [x] Libraries installed in conda env `penduin` (gymnasium, stable-baselines3, torch, tensorboard)
- [x] `python/env.py` `__init__`: model, constants, action and observation spaces
- [x] `python/env.py` `_get_obs()` + `reset()` (encoder quantization → α̇ resolution ≈ 0.15 rad/s)
- [x] `python/env.py` `step()` (passes `check_env`; terminates at |θ| > 2π/3)
- [x] `python/env.py` `_reward()`: `(1+cos α)/2 − Q·θ² − R·act²`, −100 on hitting the arm limit
- [x] Observations scaled in env: θ/th_limit, α̇/20 (≈ max speed falling from upright), θ̇/max_speed.
  `th_limit` currently π/2 (90°). Episode length set in `train.py` with `TimeLimit` (1000 steps).
- [x] `python/train.py` (SAC defaults, checkpoints in `checkpoints/`, logs in `runs/`)
- [x] `python/eval.py` (`-f model.zip [-w]`, plots α/θ/action/reward + summary)
- First run `sac_1` (200k steps): **spinning trap**. Return 578 → 766 between 50k and 200k
  steps; revolutions per episode 24 → 12, |α̇| when passing the top 9.8 → 1.6 rad/s. Still
  improving, not balancing yet.
- Continued `sac_1` for 2 × 200k more steps (600k total): reward plateaus; the policy **swings up and
  balances, but jittery**. The 300k checkpoint was smoother than the final model.
- **Reward tuning postponed** (pipeline proven). Ideas for later: α̇² penalty only near upright
  (anti-spin), penalty on action *change* (anti-jitter), α̇ quantization noise (0.15 rad/s) as a jitter
  source, retrain from scratch after reward changes, longest-upright-streak metric in `eval.py`.
- Next: `classic_control.py` (energy swing-up + LQR) and/or hardware bring-up (motor + encoder tests).

## Before the next retraining (2026-11-09)
All sim changes go into one retraining:
- [x] Bearing friction from the free-swing fit (`python/sysid.py`, part A): `damping="0"`,
  `frictionloss="0.000154"` in `model.xml`. The inertia stays at the OnShape value: the fit only
  scaled it by 0.987, and the mass checks out on a scale (0.023 kg).
- [ ] VMIN dead band in `step()`: send 0 to the motor while |speed| < 0.1 rad/s, as the firmware
  does. Keep integrating `self.speed`, because the firmware's v keeps integrating too.
- [ ] Lower limits: the motor skipped once at a = 200 rad/s² with v = 20 rad/s (`l`, stage 8), and the
  SAC policies use 200 rad/s² together with 15 rad/s. First test the combination you want to train
  with (`l` stage at that a and v). Keep `VMAX` in `stepper.h` equal to `max_speed`.
- [ ] Delay and coupling from experiment B (`sysid.py`, part B), once the step 6 skeleton runs.
- Safety stop on the hardware is ±90°, the same as `th_limit`.
