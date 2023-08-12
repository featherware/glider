import numpy as np
import torch
import torch.nn as nn
import trimesh

from constants import (DEFAULT_STL_FILEPATH, GLIDER_GEOM_NAME, MUTATION_CHANCE,
                       MUTATION_RATIO)

PILOT_RGBA = "0.2 0.2 0.8 0.5"
PILOT_DIMENSIONS_M = [1.8, 0.3, 0.6]
PILOT_MASS_KG = 68

WING_RGBA = "0.8 0.2 0.2 0.5"


class Vehicle(nn.Module):
    """
    A vehicle comprises a set of vertices for the main wing,
    and a human-sized pilot.

    Orientation is relative to the file default orientation.

    Either vertices or filename must be provided.

    Args:
        vertices (list): A list of vertices for the main wing
        filename (str): The path to an STL file
        orientation (list): The orientation of the vehicle
    """

    def __init__(
        self,
        vertices: list | None = None,
        filename: str | None = None,
        orientation: list[int] = [0, 0, 0],
        wing_density: float = 5,
        num_vertices: int = 30,
        max_dim_m: float = 4.5,
    ):
        super(Vehicle, self).__init__()

        if filename is not None:
            self.vertices = self.load_stl(filename)
        elif vertices is not None:
            self.vertices = vertices
        else:
            self.initialize_vertices(num_vertices, max_dim_m)

        self.vertices_parameter = nn.Parameter(
            torch.tensor(self.vertices, dtype=torch.float32)
        )

    def initialize_vertices(self, num_points: int, max_dim_m: float) -> None:
        self.vertices = []
        for _ in range(num_points):
            vertex = []
            for _ in range(3):
                vertex.append(np.random.random() * max_dim_m)
            self.vertices.append(vertex)

    def mutate(self) -> None:
        new_vertices = []
        for vertex in self.vertices:
            if np.random.random() < MUTATION_CHANCE:
                new_vertex: list[float] = []
                for dim in vertex:
                    if dim == 0:
                        dim += 0.1  # TODO make this better?
                    else:
                        dim *= (np.random.normal() * MUTATION_RATIO) + 1
                    new_vertex.append(dim)
            new_vertices.append(new_vertex)
        self.vertices = new_vertices

    def load_stl(
        self,
        filename: str,
        mesh_name: str = f"{GLIDER_GEOM_NAME}-mesh",
        scale: float = 1.0,
    ):
        with open(filename, "rb") as f:
            mesh = trimesh.load(
                f,
                file_type="stl",
            )
        vertices = mesh.vertices
        if scale != 1.0:
            vertices = [point * scale for point in vertices]

        return vertices

    def asset_from_vertices(
        self,
        vertices: list,
        mesh_name: str = f"{GLIDER_GEOM_NAME}-mesh",
        scale: float = 1.0,
    ):
        return f"""
        <asset>
            <mesh name="{mesh_name}" vertex="{to_vertex_list(vertices)}"/>
        </asset>"""

    def create_glider_from_vertices(
        self,
        geom_name: str = GLIDER_GEOM_NAME,
        orientation: list[float] = [0, 0, 0],
        scale: float = 1.0,
    ) -> tuple[str, str]:
        body = f"""
    <body name="body" pos="0 0 1" euler="{' '.join(list(map(str, orientation)))}">
        <freejoint/>
        <!-- Main Wing -->
        {geom_xml(
            geom_name=geom_name,
            mesh_name=geom_name + '-mesh'
        )}
        <!-- Pilot -->
        {pilot_xml()}
        <camera name="fixed" pos="-100 -100 -10" xyaxes="1 0 0 0 1 2"/>
        <camera name="track" pos="0 0 0" xyaxes="1 2 0 0 1 2" mode="track"/>
    </body>
    """

        asset = self.asset_from_vertices(
            vertices=self.vertices,
            mesh_name=(geom_name + "-mesh"),
            scale=scale,
        )
        return body, asset


def pilot_xml():
    return f"""<geom name="pilot" type="box" size="{" ".join(
        [ str(dim) for dim in PILOT_DIMENSIONS_M ])
        }" mass="{PILOT_MASS_KG}" rgba="{PILOT_RGBA}" pos="0 0 -0.3"/>"""


def geom_xml(
    geom_name: str,
    mesh_name: str,
    density: float = 40,
    rgba: str = WING_RGBA,
) -> str:
    geom = f"""<geom name="{geom_name}"  density="{density}" rgba="{rgba}" type="mesh" mesh="{mesh_name}"/>"""
    return geom


def asset_from_stl(
    filename: str,
    mesh_name: str = f"{GLIDER_GEOM_NAME}-mesh",
    scale: float = 1.0,
):
    with open(filename, "rb") as f:
        mesh = trimesh.load(
            f,
            file_type="stl",
        )
    vertices = mesh.vertices
    if scale != 1.0:
        vertices = [point * scale for point in vertices]

    asset = f"""
    <asset>
        <mesh name="{mesh_name}" vertex="{to_vertex_list(vertices)}"/>
    </asset>"""

    return asset


def create_glider_xml(
    filename: str = DEFAULT_STL_FILEPATH,
    geom_name: str = GLIDER_GEOM_NAME,
    orientation: list[int] = [200, 200, 100],
    scale: float = 1.0,
) -> tuple[str, str]:
    body = f"""
<body name="body" pos="0 0 1" euler="{' '.join(list(map(str, orientation)))}">
    <freejoint/>
    <!-- Main Wing -->
    {geom_xml(
        geom_name=geom_name,
        mesh_name=geom_name + '-mesh'
    )}
    <!-- Pilot -->
    {pilot_xml()}
    <camera name="fixed" pos="-100 -100 -10" xyaxes="1 0 0 0 1 2"/>
    <camera name="track" pos="0 0 0" xyaxes="1 2 0 0 1 2" mode="track"/>
</body>
"""

    asset = asset_from_stl(
        filename=filename,
        mesh_name=(geom_name + "-mesh"),
        scale=scale,
    )
    return body, asset


def to_vertex_list(
    points: list,
) -> str:
    str_points = []

    for point in points:
        str_points.append(" ".join([str(coord) for coord in list(point)]))
    return " ".join(str_points)
