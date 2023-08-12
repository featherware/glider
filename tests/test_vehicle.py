import mujoco
import pytest

import visualize
from simulation import drop_test_glider
from vehicle import Vehicle, create_glider_xml


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


def test_initialize():
    vehicle = Vehicle(num_vertices=8)
    assert len(vehicle.vertices) == 8

    vehicle_2 = Vehicle(num_vertices=10)
    assert len(vehicle_2.vertices) == 10

    vehicle_3 = Vehicle(num_vertices=10)
    for point in vehicle_3.vertices:
        assert len(point) == 3
        assert point not in vehicle_2.vertices


def test_max_dim():
    vehicle = Vehicle(num_vertices=8, max_dim_m=1.5)
    for point in vehicle.vertices:
        for dim in point:
            assert dim <= 1.5


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
