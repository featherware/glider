from functools import reduce

import trimesh

from constants import DEFAULT_STL_FILEPATH, GLIDER_GEOM_NAME

PILOT_RGBA = "0.2 0.2 0.8 0.5"
PILOT_DIMENSIONS_M = [1.8, 0.3, 0.6]
PILOT_DENSITY_KG = 68 / reduce(lambda x, y: x * y, PILOT_DIMENSIONS_M)

WING_RGBA = "0.8 0.2 0.2 0.5"


def pilot_xml():
    return f"""<geom name="pilot" type="box" size="{" ".join(
        [ str(dim) for dim in PILOT_DIMENSIONS_M ])
        }" rgba="{PILOT_RGBA}" pos="0 0 -0.3"/>"""


def geom_xml(geom_name: str, mesh_name: str) -> str:
    geom = (
        f"""<geom name="{geom_name}"  density="40" type="mesh" mesh="{mesh_name}"/>"""
    )
    return geom


def asset_from_stl(filename: str, mesh_name: str = f"{GLIDER_GEOM_NAME}-mesh"):
    with open(filename, "rb") as f:
        mesh = trimesh.load(
            f,
            file_type="stl",
        )
    vertices = mesh.vertices
    asset = f"""
    <asset>
        <mesh name="{mesh_name}" vertex="{to_vertex_list(vertices)}"/>
    </asset>"""

    return asset


def create_glider_xml(
    filename: str = DEFAULT_STL_FILEPATH,
    geom_name: str = GLIDER_GEOM_NAME,
    orientation: list[int] = [90, 0, 20],
) -> tuple[str, str]:
    body = f"""
<body name="body" pos="0 0 1" euler="{' '.join(list(map(str, orientation)))}">
    <freejoint/>
    <!-- Main Wing -->
    {geom_xml(
        geom_name=geom_name,
        mesh_name=geom_name + '-mesh'
    )}
    <camera name="fixed" pos="-100 -100 -10" xyaxes="1 0 0 0 1 2"/>
    <camera name="track" pos="0 0 0" xyaxes="1 2 0 0 1 2" mode="track"/>
</body>
"""

    asset = asset_from_stl(filename=filename, mesh_name=(geom_name + "-mesh"))
    return body, asset


def to_vertex_list(
    points: list,
    scale: float = 8.0,
) -> str:
    str_points = []

    for point in points:
        str_points.append(" ".join([str(coord * scale) for coord in list(point)]))
    return " ".join(str_points)
