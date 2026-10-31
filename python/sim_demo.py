"""Minimal MuJoCo loop for the Furuta pendulum: step the physics, command the arm, watch it in the viewer."""
import time

import mujoco
import mujoco.viewer
import numpy as np

model = mujoco.MjModel.from_xml_path("../model.xml")  # the compiled, constant description
data = mujoco.MjData(model)                         # the changing state: positions, velocities, controls, ...

# start with the pendulum hanging down (bearing = 0 is upright)
data.joint("bearing").qpos[0] = np.pi
mujoco.mj_forward(model, data)  # recompute derived quantities (positions of bodies, etc.) without stepping time

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        step_start = time.time()

        # control: target arm speed in rad/s (velocity actuator), here a slow back-and-forth
        data.ctrl[0] = 8.0 * np.sin(2 * np.pi * 1.0 * data.time)

        mujoco.mj_step(model, data)  # advance the simulation by model.opt.timestep (2 ms)

        print(f"t={data.time:6.2f}  arm={data.qpos[0]:+.2f} rad  pendulum={data.qpos[1]:+.2f} rad", end="\r")

        viewer.sync()  # push the new state to the viewer window
        # run at real-time speed (drop this to simulate as fast as possible)
        time.sleep(max(0.0, model.opt.timestep - (time.time() - step_start)))
