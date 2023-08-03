from functools import reduce

import trimesh

import wing

PILOT_RGBA = "0.2 0.2 0.8 0.5"
PILOT_DIMENSIONS_M = [1.8, 0.3, 0.6]
PILOT_DENSITY_KG = 68 / reduce(lambda x, y: x * y, PILOT_DIMENSIONS_M)

WING_RGBA = "0.8 0.2 0.2 0.5"


def pilot_xml():
    return f"""<geom name="pilot" type="box" size="{" ".join([ str(dim) for dim in PILOT_DIMENSIONS_M ])}" rgba="{PILOT_RGBA}" pos="0 0 -0.3"/>"""


def asset_from_stl(filename: str, name: str = "delta-wing"):
    with open(filename, "rb") as f:
        mesh = trimesh.load(
            f,
            file_type="stl",
        )
    vertices = mesh.vertices
    asset = f"""
    <asset>
        <mesh name="{name}-mesh" vertex="{to_vertex_list(vertices)}"/>
    </asset>"""

    return asset


def create_glider_xml(
    mesh_name: str = "stl-wing", pitch_angle: int = 20
) -> tuple[str, str]:
    body = f"""
<body name="body" pos="0 0 1" euler="90 0 {pitch_angle}">
    <freejoint/>
    <!-- Main Wing -->
    {wing.mesh_geom(name=mesh_name)}
</body>
"""

    asset = asset_from_stl("assets/delta_plane.stl", name=mesh_name + "-mesh")
    return body, asset


def to_vertex_list(points: list) -> str:
    str_points = []

    for point in points:
        str_points.append(" ".join([str(coord) for coord in list(point)]))
    return " ".join(str_points)
