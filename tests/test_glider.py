import mujoco

import visualize
from glider import create_glider_xml
from simulation import drop_test_glider


def test_glider():
    glider_xml, glider_asset = create_glider_xml()
    assert glider_xml
    assert glider_asset

    world_xml = drop_test_glider(glider_xml, glider_asset)

    model = mujoco.MjModel.from_xml(world_xml)
    data = mujoco.MjData(model)

    frames = visualize.render_to_collision(model, data, show=False)

    assert len(frames) > 0

    # TODO add heuristic for duration of flight
