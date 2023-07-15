from functools import reduce
from typing import Callable

import numpy as np


PILOT_RGBA = "0.2 0.2 0.8 0.5"
PILOT_DIMENSIONS_M = [1.8, 0.3, 0.6]
PILOT_DENSITY_KG = 68 / reduce(lambda x, y: x * y, PILOT_DIMENSIONS_M)

WING_RGBA = "0.8 0.2 0.2 0.5"


def pilot_xml():
    return f"""<geom name="pilot" type="box" size="{" ".join([ str(dim) for dim in PILOT_DIMENSIONS_M ])}" rgba="{PILOT_RGBA}" pos="0 0 -0.3"/>"""


def box_wing_xml(size: list = [15, 0.3, 30], density=23) -> str:
    return f"""<geom name="main_wing" type="box" size="{" ".join([str(dim) for dim in size])}" density="{density}" rgba="{WING_RGBA}" pos="0 0 0"/>"""


def glider_xml(
        wing_fcn: Callable = box_wing_xml,
        pilot_fcn: Callable = pilot_xml,
        pitch_angle: int = 20
        ) -> str:
    return f"""
<body name="body" pos="0 0 1" euler="90 0 {pitch_angle}">
    <freejoint/>
    <!-- Main Wing -->
    {wing_fcn()}
    <!-- Pilot -->
    {pilot_fcn()}
</body>
"""


def wing_coords(wing_span_m=20, sweep_back_deg=20, wing_chord_m=3) -> tuple:

    sweep_back_rad = sweep_back_deg * 2 * np.pi / 360
    # tan(sweep_back) = wing_trail / (wing_span / 2)
    wing_tip_trail_m = np.tan(sweep_back_rad) * (wing_span_m / 2)

    leading_point = np.array([0, 0, 0])

    wing_tip_left = np.array([
        wing_tip_trail_m,
        0,
        -wing_span_m / 2
    ])

    wing_tip_right = np.array([
        wing_tip_trail_m,
        0,
        wing_span_m / 2
    ])

    trailing_point = np.array([
        wing_chord_m,
        0,
        0
    ])

    return (leading_point, wing_tip_left, trailing_point, wing_tip_right)


def to_vertex_list(points: list) -> str:
    str_points = []

    for point in points:
        str_points.append(" ".join(list(point)))
    return " ".join(str_points)


if __name__ == "__main__":
    print(glider_xml())
