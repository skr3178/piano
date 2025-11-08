#!/usr/bin/env python3
"""Simple script to open piano.xml in MuJoCo viewer."""

import mujoco
import mujoco.viewer

# Load the model
model_path = "robopianist/robopianist/models/piano/assets/piano.xml"
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

# Launch viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    print("MuJoCo viewer opened. Press ESC or close window to exit.")
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()

