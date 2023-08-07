import mujoco
import pandas as pd

from constants import DEFAULT_STL_FILEPATH
from observability import glider_abs_x_position
from simulation import drop_test_glider
from vehicle import create_glider_xml


def find_best_orientation(
    stl_filename: str = DEFAULT_STL_FILEPATH,
    granularity_deg: int = 50,
    max_results: int = 10,
) -> list[int]:
    # array = [
    #     pd.DataFrame(
    #         [[x, y, z, distance_score(x,y,z)]],
    #         columns=['x', 'y', 'z', 'distance'],
    #         )
    #     for x in range(0,360, granularity)
    #     for y in range(0,360, granularity)
    #     for z in range(0, 360, granularity)
    # ]

    df = pd.DataFrame(columns=["x", "y", "z", "distance"])
    df = pd.concat(
        [
            pd.DataFrame(
                [[x, y, z, measure_drop_test(orientation=[x, y, z])]],
                columns=["x", "y", "z", "distance"],
            )
            for x in range(0, 360, granularity_deg)
            for y in range(0, 360, granularity_deg)
            for z in range(0, 360, granularity_deg)
        ],
        ignore_index=True,
    )

    print("Furthest distance: ", df["distance"].max())
    print("Best orientation", df[df["distance"] == df["distance"].max()])
    return df[df["distance"] == df["distance"].max()]


def measure_drop_test(
    stl_filename: str = DEFAULT_STL_FILEPATH,
    scale: float = 1.0,
    orientation: list[int] = [200, 200, 100],
    height=80,
    wind: str = "0 0 0",
) -> float:
    glider_xml, glider_asset = create_glider_xml(
        filename=stl_filename, orientation=orientation, scale=scale
    )
    assert glider_xml
    assert glider_asset

    world_xml = drop_test_glider(glider_xml, glider_asset)

    model = mujoco.MjModel.from_xml_string(world_xml)
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)  # Reset state and time.

    while len(data.contact) < 1:  # Render until landing
        mujoco.mj_step(model, data)

    return glider_abs_x_position(data)
