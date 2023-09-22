import mujoco
import pytest

from glider import visualize
from glider.vehicle import Vehicle, create_glider_xml


@pytest.fixture
def cube_vertices():
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


@pytest.fixture
def cube_faces():
    return [
        [0, 1, 6, 3],
        [7, 4, 2, 5],
        [7, 4, 1, 6],
        [7, 3, 0, 5],
        [2, 5, 0, 1],
        [2, 3, 6, 4],
    ]


def concave_prism_vertices():
    return [
        [0.0, 0.0, 0.0],  # inner base point
        [1.0, -1.0, 1.0],  # base
        [1.0, -1.0, -1.0],  # base
        [-1.0, -1.0, -1.0],  # base
        [-1.0, -1.0, 1.0],  # base
        [0.0, 1.0, 0.0],  # pyramid tip
    ]


def concave_prism_faces():
    return [
        [1, 2, 0],
        [2, 3, 0],
        [3, 4, 0],
        [4, 1, 0],
        [2, 1, 5],
        [3, 2, 5],
        [4, 3, 5],
        [1, 4, 5],
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


def test_faces(cube_vertices, cube_faces):
    vehicle = Vehicle(num_vertices=8)
    assert vehicle.faces is not None

    vehicle_2 = Vehicle(vertices=cube_vertices, faces=cube_faces)
    assert vehicle_2.faces == cube_faces

    # TODO Test that this is a cube

    # TODO test that this works for non-convex groups


def test_max_dim():
    vehicle = Vehicle(num_vertices=8, max_dim_m=1.5)
    for point in vehicle.vertices:
        for dim in point:
            assert dim <= 1.5


def test_mutate(cube_vertices):
    vehicle1 = Vehicle(vertices=cube_vertices)
    new_vertices = vehicle1.mutate()

    assert len(new_vertices) == len(vehicle1.vertices)

    # All vertices should be different
    for vertex in new_vertices:
        assert vertex not in vehicle1.vertices

    vehicle2 = Vehicle(vertices=new_vertices)
    assert vehicle2


def test_clone():
    vehicle1 = Vehicle(num_vertices=8)
    vehicle2 = vehicle1.clone()

    assert vehicle1.vertices == vehicle2.vertices
    assert id(vehicle1) != id(vehicle2)


def test_cross_over():
    vehicle1 = Vehicle(num_vertices=8)
    vehicle2 = Vehicle(num_vertices=8)

    new_vertices = vehicle1.cross_over(vehicle2)

    assert new_vertices != vehicle1.vertices
    assert new_vertices != vehicle2.vertices
    assert len(new_vertices) == len(vehicle1.vertices)
