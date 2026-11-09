"""System identification, part B: the arm drives the hanging pendulum (experiment B).

    python python/sysid_b.py

Replays the actions each run logged through PenduinEnv.step (same speed integration, clamp and
VMIN dead band as the firmware) and compares the pendulum angle with the measurement. Fits:
  - com_z:        the pendulum's centre of mass along the bearing axis (mm, model.xml 14.052). This
                  sets how far it sits from the motor axis, so it scales the coupling b, without
                  changing the period or the gravity term that part A fitted.
  - frictionloss: bearing dry friction (1e-4 N*m), a cross-check of part A
  - delay:        whole ticks between the logged action and its effect, tried one by one
Runs whose name ends in 3 are only used to check the fit.
"""
from pathlib import Path

import matplotlib.pyplot as plt
import mujoco
import numpy as np
from scipy.optimize import least_squares

from env import PenduinEnv

ROOT = Path(__file__).resolve().parent.parent
DATA = ROOT / "data"
DT = 0.01
RAD_PER_STEP = 2 * np.pi / 3200
DELAYS = [0, 1, 2]
NAMES = ["com_z (mm)", "frictionloss (1e-4 N*m)"]


def wrap(x):
    return (x + np.pi) % (2 * np.pi) - np.pi


# ---------- recordings ----------

def load(path):
    """Rows after the column names that have all 10 fields. Stops at the first gap in time_us."""
    lines = path.read_text().splitlines()
    lines = lines[lines.index("time_us,raw,steps,v,action,alpha,alpha_dot,loop_us,late,dropped") + 1:]
    rows = [l.split(",") for l in lines if l[:1].isdigit()]
    rows = [r for r in rows if len(r) == 10]
    d = np.array(rows, dtype=float)
    gaps = np.flatnonzero(np.abs(np.diff(d[:, 0]) - 10000) > 100)
    if len(gaps):
        print(f"  {path.name}: gap in time_us after row {gaps[0]}, using the rows before it")
        d = d[:gaps[0] + 1]
    phi = wrap(d[:, 5] - np.pi)     # pendulum angle from the bottom (logged alpha: upright = 0)
    return {
        "name": path.stem,
        "check": path.stem.endswith("3"),
        "action": d[:, 4],
        "theta": d[:, 2] * RAD_PER_STEP,
        "phi": phi,
        "phi0": phi[:50].mean(),    # the run starts with 1 s at rest
    }


# ---------- simulation ----------

env = PenduinEnv(str(ROOT / "model.xml"))
PEND = env.model.body("pendulum").id
DOF = env.model.joint("bearing").dofadr[0]
QA = env.model.joint("motor").qposadr[0]
QP = env.model.joint("bearing").qposadr[0]
P_MODEL = [env.model.body_ipos[PEND, 2] * 1e3, env.model.dof_frictionloss[DOF] * 1e4]


def set_params(p):
    env.model.body_ipos[PEND, 2] = p[0] * 1e-3
    env.model.dof_frictionloss[DOF] = p[1] * 1e-4


def replay(p, run, delay):
    """Pendulum angle (from the bottom) and arm angle, one sample per logged row."""
    set_params(p)
    env.reset(seed=0)
    env.data.qpos[QA] = 0.0
    env.data.qpos[QP] = np.pi + run["phi0"]
    env.data.qvel[:] = 0.0
    env.speed = 0.0
    mujoco.mj_forward(env.model, env.data)

    n = len(run["action"])
    actions = np.concatenate([np.zeros(delay), run["action"]])[:n]
    phi, theta = np.empty(n), np.empty(n)
    phi[0], theta[0] = run["phi0"], 0.0
    for k in range(n - 1):
        # row k's action acts during tick k; row k+1 is measured at the end of it
        env.step(np.array([actions[k]]))
        phi[k + 1] = wrap(env.data.qpos[QP] - np.pi)
        theta[k + 1] = env.data.qpos[QA]
    return phi, theta


def residuals(p, runs, delay):
    return np.concatenate([replay(p, r, delay)[0] - r["phi"] for r in runs])


def rms_deg(x):
    return np.degrees(np.sqrt(np.mean(x ** 2)))


# ---------- main ----------

def main():
    runs = [load(f) for f in sorted(DATA.glob("expB_*.csv"))]
    train = [r for r in runs if not r["check"]]
    check = [r for r in runs if r["check"]]
    print(f"{len(train)} runs to fit, {len(check)} to check")

    print("\nmodel.xml as it is:")
    for d in DELAYS:
        errs = [rms_deg(replay(P_MODEL, r, d)[0] - r["phi"]) for r in runs]
        print(f"  delay {d}: pendulum RMS per run " + " ".join(f"{e:.2f}" for e in errs) + " deg")

    print("\nfitted, per delay:")
    best = None
    for d in DELAYS:
        fit = least_squares(residuals, P_MODEL, args=(train, d), diff_step=1e-3,
                            bounds=([-20, 0], [60, 10]))
        errs = [rms_deg(replay(fit.x, r, d)[0] - r["phi"]) for r in runs]
        print(f"  delay {d}: " + ", ".join(f"{n} {v:.3f}" for n, v in zip(NAMES, fit.x))
              + " | RMS per run " + " ".join(f"{e:.2f}" for e in errs) + " deg")
        train_err = np.mean([e for e, r in zip(errs, runs) if not r["check"]])
        if best is None or train_err < best[0]:
            best = (train_err, d, fit.x)
    _, delay, p = best

    print(f"\nbest: delay {delay} tick(s), " + ", ".join(f"{n} {v:.3f}" for n, v in zip(NAMES, p)))
    r_model = 0.072 + P_MODEL[0] * 1e-3
    r_fit = 0.072 + p[0] * 1e-3
    print(f"centre of mass from the motor axis: {r_model * 1e3:.1f} mm in model.xml, {r_fit * 1e3:.1f} mm fitted "
          f"(coupling x{r_fit / r_model:.3f}, roughly)")
    print("arm: sim theta vs the step count (the sim's actuator is a velocity servo, the stepper isn't):")
    for r in runs:
        _, th = replay(p, r, delay)
        print(f"  {r['name']}: RMS {rms_deg(th - r['theta']):.2f} deg, worst {np.degrees(np.abs(th - r['theta']).max()):.2f} deg")
    print(f"\nmodel.xml, pendulum <inertial>: pos=\"-0.074141 0 {p[0] * 1e-3:.6f}\"")
    print(f"model.xml, bearing <joint>:     frictionloss=\"{p[1] * 1e-4:.3g}\"")

    # Plot: each run, pendulum (measured, model.xml, fitted) and arm (steps vs sim); plus a zoom
    fig, axs = plt.subplots(2 * len(runs) + 1, 1, figsize=(12, 2.2 * (2 * len(runs) + 1)))
    for i, r in enumerate(runs):
        t = np.arange(len(r["phi"])) * DT
        phi_m, _ = replay(P_MODEL, r, 0)
        phi_f, th_f = replay(p, r, delay)
        ax = axs[2 * i]
        ax.plot(t, np.degrees(r["phi"]), "k", lw=1, label="measured")
        ax.plot(t, np.degrees(phi_m), lw=0.8, alpha=0.6, label="model.xml")
        ax.plot(t, np.degrees(phi_f), lw=0.8, label="fitted")
        ax.set_ylabel(f"{r['name']}\npendulum (deg)" + (" check" if r["check"] else ""), fontsize=7)
        ax = axs[2 * i + 1]
        ax.plot(t, np.degrees(r["theta"]), "k", lw=1, label="steps")
        ax.plot(t, np.degrees(th_f), lw=0.8, label="sim arm")
        ax.set_ylabel("arm (deg)", fontsize=7)
    r = runs[0]
    t = np.arange(len(r["phi"])) * DT
    z = (t > 0.9) & (t < 3.0)
    phi_m, _ = replay(P_MODEL, r, 0)
    phi_f, _ = replay(p, r, delay)
    ax = axs[-1]
    ax.plot(t[z], np.degrees(r["phi"][z]), "k.-", lw=1, ms=3, label="measured")
    ax.plot(t[z], np.degrees(phi_m[z]), ".-", lw=0.8, ms=3, alpha=0.6, label="model.xml")
    ax.plot(t[z], np.degrees(phi_f[z]), ".-", lw=0.8, ms=3, label="fitted")
    ax.set_ylabel(f"zoom {r['name']}\npendulum (deg)", fontsize=7)
    for ax in axs:
        ax.grid(alpha=0.3)
        ax.legend(fontsize=6, loc="upper right")
    axs[-1].set_xlabel("time (s)")
    fig.tight_layout()
    out = DATA / "sysid_B.png"
    fig.savefig(out, dpi=80)
    print(f"\nplot saved to {out}")


if __name__ == "__main__":
    main()
