import matplotlib.pyplot as plt
import mujoco
import numpy as np
import pandas as pd

from .constants import GLIDER_GEOM_NAME


def extract_forwards_accn(world_xml: str) -> list[float]:
    model = mujoco.MjModel.from_xml_string(world_xml)
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)  # Reset state and time.

    time_series = []
    while len(data.contact) < 1:  # Render until landing
        mujoco.mj_step(model, data)
        time_series.append(abs(data.geom(GLIDER_GEOM_NAME).xpos[0]))

    return time_series
