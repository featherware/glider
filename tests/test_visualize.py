import visualize
from constants import FRAME_HEIGHT, FRAME_WIDTH
from vehicle import Vehicle


def test_view_vehicle():
    v = Vehicle(num_vertices=10)
    vehicle_xml, vehicle_asset = v.create_glider_from_vertices()

    pixels = visualize.view_vehicle(
        vehicle_xml,
        vehicle_asset,
    )
    assert pixels.shape == (FRAME_HEIGHT, FRAME_WIDTH, 3)
