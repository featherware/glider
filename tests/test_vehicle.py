import mujoco
import pytest

import visualize
from constants import DEFAULT_STL_FILEPATH
from vehicle import Vehicle, create_glider_xml
from simulation import drop_test_glider


@pytest.fixture
def vertices():
    return [
        [1.0, 1.0, 1.0],
        [0.0, 1.0, 1.0],
        [1.0, 0.0, 1.0],
        [1.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0],
        ]

def test_glider():
    glider_xml, glider_asset = create_glider_xml()
    assert glider_xml
    assert glider_asset

    world_xml = drop_test_glider(glider_xml, glider_asset)

    model = mujoco.MjModel.from_xml_string(world_xml)
    data = mujoco.MjData(model)

    frames = visualize.render_to_collision(model, data, show=False)

    assert len(frames) > 0

    # TODO add heuristic for duration of flight


def test_mutate(vertices):
    vehicle = Vehicle(vertices=vertices)
    v1 = vehicle.vertices
    vehicle.mutate()
    v2 = vehicle.vertices

    assert len(v1) == len(v2)

    for vertex in v2:
        assert vertex not in vertices
