"""System identification, part A: fit the pendulum's free swing (arm held still).

    python python/sysid.py

Fits four numbers so that MuJoCo's free swing matches the recordings in data/swing*deg*.csv:
  - inertia_scale: scales the pendulum's inertia about its centre of mass (tunes the period)
  - damping:       viscous bearing friction, N*m*s/rad (model.xml `damping`)
  - frictionloss:  dry bearing friction, N*m (model.xml `frictionloss`)
  - offset:        where the true bottom is, relative to the average resting reading

Runs whose name ends in 3 are not used for the fit, only to check it afterwards.
The encoder's sign doesn't matter here: a free swing looks the same both ways.
"""
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib.pyplot as plt
import mujoco
import numpy as np
from scipy.optimize import least_squares

ROOT = Path(__file__).resolve().parent.parent
MODEL = ROOT / "model.xml"
DATA = ROOT / "data"
DT = 0.01                       # logger tick, 100 Hz
RAD_PER_COUNT = 2 * np.pi / 4096


# ---------- recordings ----------

def wrap_counts(d):
    """Shortest signed difference between raw readings, in [-2048, 2048)."""
    return (d + 2048) % 4096 - 2048


def load(path):
    """Raw counts. Lines before 'time_us,raw' are leftovers from before the port opened."""
    lines = path.read_text().splitlines()
    lines = lines[lines.index("time_us,raw") + 1:]
    rows = [l.split(",") for l in lines if l and not l.startswith("#")]
    t = np.array([int(r[0]) for r in rows])
    raw = np.array([int(r[1]) for r in rows])
    assert (raw >= 0).all(), f"{path.name}: failed reads in the recording"
    assert np.all(np.abs(np.diff(t[5:]) - 10000) < 100), f"{path.name}: a tick was late"
    return raw


def mean_raw(raw):
    """Average of raw readings that may straddle the 4095 -> 0 wrap (same idea as raw_average)."""
    return (raw[0] + np.mean(wrap_counts(raw - raw[0]))) % 4096


def cut(phi):
    """Start: the first turning point after release, so the hand plays no part and the speed is
    about 0. End: 0.5 s after the swing has died out."""
    released = np.argmax(np.abs(phi - phi[0]) > np.radians(1))
    above = phi[released:] > 0
    crossings = released + np.flatnonzero(np.diff(above.astype(int)) != 0)
    a, b = crossings[0] + 1, crossings[1] + 1
    start = a + np.argmax(np.abs(phi[a:b]))
    moving = np.flatnonzero(np.abs(phi - phi[-1]) > 2 * RAD_PER_COUNT)
    end = min(len(phi), moving[-1] + 50)
    return start, end


def load_runs():
    files = sorted(DATA.glob("swing*deg*.csv"))
    raws = {f.stem: load(f) for f in files}
    # One common reference for all runs: the average of where they came to rest
    rests = np.array([mean_raw(r[-50:]) for r in raws.values()])
    ref = mean_raw(rests)
    runs = []
    for name, raw in raws.items():
        phi = wrap_counts(raw - ref) * RAD_PER_COUNT   # angle from ref, rad
        start, end = cut(phi)
        runs.append({
            "name": name,
            "check": name.endswith("3"),
            "phi": phi[start:end],
            "phidot0": (phi[start + 1] - phi[start - 1]) / (2 * DT),  # central difference
            "rest": wrap_counts(mean_raw(raw[-50:]) - ref),             # counts from ref
        })
    return runs, ref


# ---------- simulation ----------

model = mujoco.MjModel.from_xml_path(str(MODEL))
data = mujoco.MjData(model)
PEND = model.body("pendulum").id
QPOS = model.joint("bearing").qposadr[0]
DOF = model.joint("bearing").dofadr[0]
INERTIA0 = model.body_inertia[PEND].copy()
NSTEP = round(DT / model.opt.timestep)

# The optimizer works on numbers of about 1, so its finite-difference steps make sense for all of
# them: damping and frictionloss in units of 1e-4, offset in degrees.
NAMES = ["inertia_scale", "damping (1e-4 N*m*s/rad)", "frictionloss (1e-4 N*m)", "offset (deg)"]


def set_params(p):
    model.body_inertia[PEND] = INERTIA0 * p[0]
    model.dof_damping[DOF] = p[1] * 1e-4
    model.dof_frictionloss[DOF] = p[2] * 1e-4


def simulate(p, phi0, phidot0, n):
    """Free swing from angle phi0 (from the bottom) and speed phidot0, n samples at 100 Hz."""
    set_params(p)
    mujoco.mj_resetData(model, data)
    data.qpos[QPOS] = np.pi + phi0      # hanging = pi in the sim
    data.qvel[DOF] = phidot0
    data.ctrl[0] = 0                    # the motor holds the arm still
    out = np.empty(n)
    out[0] = phi0
    for k in range(1, n):
        mujoco.mj_step(model, data, nstep=NSTEP)
        out[k] = data.qpos[QPOS] - np.pi
    return out


def residuals(p, runs):
    """Sim minus measurement for every sample. Each run is divided by its amplitude and by the
    square root of its length, so the small swings count as much as the big ones."""
    res = []
    for r in runs:
        phi = r["phi"] - np.radians(p[3])      # from the true bottom
        sim = simulate(p, phi[0], r["phidot0"], len(phi))
        res.append((sim - phi) / (abs(phi[0]) * np.sqrt(len(phi))))
    return np.concatenate(res)


def rms_deg(p, r):
    phi = r["phi"] - np.radians(p[3])
    sim = simulate(p, phi[0], r["phidot0"], len(phi))
    return np.degrees(np.sqrt(np.mean((sim - phi) ** 2)))


# ---------- main ----------

def main():
    runs, ref = load_runs()
    train = [r for r in runs if not r["check"]]
    check = [r for r in runs if r["check"]]
    print(f"{len(train)} runs to fit, {len(check)} to check, reference raw {ref:.1f}")

    p_model = [1.0, model.dof_damping[DOF] * 1e4, model.dof_frictionloss[DOF] * 1e4, 0.0]
    fit = least_squares(residuals, p_model, args=(train,), diff_step=1e-3,
                        bounds=([0.3, 0, 0, -3], [3, 10, 10, 3]))
    p = fit.x

    print("\nparameter                   model.xml     fitted")
    for name, a, b in zip(NAMES, p_model, p):
        print(f"  {name:26s} {a:9.4f}  {b:9.4f}")

    print("\nRMS error per run (deg), model.xml vs fitted:")
    for r in runs:
        tag = "check" if r["check"] else "fit"
        print(f"  {r['name']:16s} {tag:5s}  {rms_deg(p_model, r):6.2f}  {rms_deg(p, r):6.2f}")

    offset_counts = p[3] / np.degrees(RAD_PER_COUNT)
    print(f"\ntrue bottom: raw {(ref + offset_counts) % 4096:.1f}, "
          f"so raw_up = {(ref + offset_counts + 2048) % 4096:.1f}")
    print("where each run came to rest, in counts from the true bottom:")
    print("  " + " ".join(f"{r['rest'] - offset_counts:+.1f}" for r in runs))

    # What to put in model.xml
    body = ET.parse(MODEL).getroot().find(".//body[@name='pendulum']/inertial")
    full = [float(v) * p[0] for v in body.get("fullinertia").split()]
    print("\nmodel.xml, pendulum <inertial>:  fullinertia=\"" + " ".join(f"{v:.6g}" for v in full) + "\"")
    print(f"model.xml, bearing <joint>:      damping=\"{p[1] * 1e-4:.3g}\" frictionloss=\"{p[2] * 1e-4:.3g}\"")

    # Plot: measured vs fitted sim, one row per run
    fig, axs = plt.subplots(len(runs), 1, figsize=(10, 1.8 * len(runs)), sharex=True)
    for ax, r in zip(axs, runs):
        phi = r["phi"] - np.radians(p[3])
        t = np.arange(len(phi)) * DT
        ax.plot(t, np.degrees(phi), "k", lw=1, label="measured")
        ax.plot(t, np.degrees(simulate(p_model, phi[0], r["phidot0"], len(phi))), lw=0.8, alpha=0.6, label="model.xml")
        ax.plot(t, np.degrees(simulate(p, phi[0], r["phidot0"], len(phi))), lw=0.8, label="fitted")
        ax.set_ylabel(r["name"] + (" (check)" if r["check"] else ""), fontsize=7)
        ax.grid(alpha=0.3)
    axs[0].legend(fontsize=7, loc="upper right")
    axs[-1].set_xlabel("time from the first turning point (s)")
    fig.tight_layout()
    out = DATA / "sysid_A.png"
    fig.savefig(out, dpi=90)
    print(f"\nplot saved to {out}")


if __name__ == "__main__":
    main()
