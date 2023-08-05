import mujoco

import visualize
from constants import DEFAULT_STL_FILEPATH, GLIDER_GEOM_NAME
from simulation import drop_test_glider
from vehicle import create_glider_xml


def measure_drop_test(
    stl_filename: str = DEFAULT_STL_FILEPATH,
    orientation: list[int] = [90, 0, 20],
    height=80,
    wind: str = "0 0 0",
) -> float:
    glider_xml, glider_asset = create_glider_xml(
        filename=stl_filename, orientation=orientation
    )
    assert glider_xml
    assert glider_asset

    world_xml = drop_test_glider(glider_xml, glider_asset)

    model = mujoco.MjModel.from_xml_string(world_xml)
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)  # Reset state and time.

    while len(data.contact) < 1:  # Render until landing
        mujoco.mj_step(model, data)

    return abs(data.geom(GLIDER_GEOM_NAME).xpos[0])
