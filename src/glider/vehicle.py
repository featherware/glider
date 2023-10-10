from typing import Any

import mediapy as media
import numpy as np
import trimesh

import glider.visualize as visualize
from .constants import (
    MUTATION_RATIO,
    MUTATION_CHANCE
)

PILOT_RGBA = "0.2 0.2 0.8 0.5"
PILOT_DIMENSIONS_M = [1.8, 0.3, 0.6]
PILOT_MASS_KG = 68

def create_pilot_geom(pos: list[float] = [0, 0, 0]):
    return f"""<geom name="pilot" type="box" size="{' '.join(map(str, PILOT_DIMENSIONS_M))}" pos="{' '.join(map(str,pos))}" >"""

WING_RGBA = "0.8 0.2 0.2 0.5"
WING_DENSITY = 1.67  # Roughly the same as a paraglider

class Vehicle:
    """
    A vehicle comprises a set of vertices for the main wing,
    and a human-sized pilot.

    Orientation is relative to the file default orientation.

    Either vertices or filename must be provided.

    Args:
        vertices (list): A list of vertices for the main wing
        filename (str): The path to an STL file
    """

    def __init__(
        self,
        vertices: list | None = None,
        faces: list | None = None,
        wing_density: float = WING_DENSITY,
        num_vertices: int = 30,
        max_dim_m: float = 4.5,
        mass_kg: float = 0.0,
        orientation: list[int] = [0, 0, 0],
    ):
        super(Vehicle, self).__init__()

        self.max_dim_m = max_dim_m
        self.mass_kg = mass_kg if mass_kg else None
        self.orientation = orientation
        self.wing_density = wing_density
        self.faces = faces if faces else []

        if vertices is not None:
            self.vertices = vertices
        else:
            self.initialize_vertices(num_vertices, max_dim_m)

    def initialize_vertices(self, num_points: int, max_dim_m: float) -> None:
        self.vertices = []
        for _ in range(num_points):
            vertex = []
            for _ in range(3):
                vertex.append(np.random.random() * max_dim_m)
            self.vertices.append(vertex)

    def mutate(self) -> list[list[float]]:
        retries = 10

        for _ in range(retries):
            new_vertices = []
            new_vertex = []

            for vertex in self.vertices:
                new_vertex: list[float] = list()  # type: ignore
                for dim in vertex:
                    if np.random.random() < MUTATION_CHANCE:
                        dim += self.max_dim_m * MUTATION_RATIO * np.random.choice((-1, 1))
                    new_vertex.append(dim)
                new_vertices.append(new_vertex)

            if not Vehicle(new_vertices).exceeds_max_dim():
                return new_vertices
            else:
                continue

        return self.vertices

    def clone(self) -> Any:
        return Vehicle(vertices=self.vertices)

    def load_stl(
        self,
        filename: str,
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
        faces: list = [],
        scale: float = 1.0,
    ):
        if faces:
            return f"""
            <asset>
                <mesh name="{'vehicle-wing-mesh'}" vertex="{to_vertex_list(vertices)}" face="{to_vertex_list(faces)}"/>
            </asset>"""
        else:
            return f"""
            <asset>
                <mesh name="{'vehicle-wing-mesh'}" vertex="{to_vertex_list(vertices)}"/>
            </asset>"""

    def create_glider_from_vertices(
        self,
        scale: float = 1.0,
    ) -> tuple[str, str]:
        density_tag = f"density=\"{WING_DENSITY}\""
        mass_tag = f"mass=\"{self.mass_kg}\""

        body = f"""
    <body name="body" pos="0 0 1" euler="{' '.join(map(str, self.orientation))}">
        <freejoint/>
        <!-- Main Wing -->
        <geom name="{'vehicle-wing'}" {density_tag if not self.mass_kg else mass_tag} rgba="{WING_RGBA}" type="mesh" mesh="{'vehicle-wing-mesh'}"/>
        <camera name="track" pos="0 0 0" xyaxes="1 2 0 0 1 2" mode="track"/>
    </body>
    """

        asset = self.asset_from_vertices(
            vertices=self.vertices,
            faces=self.faces,
            scale=scale,
        )
        return body, asset
    
    def create_xml(
        self,
        scale: float = 1.0,
    ) -> tuple[str, str]:
        body = f"""
    <body name="body" pos="0 0 1" euler="{' '.join(map(str, self.orientation))}">
        <freejoint/>
        <!-- Main Wing -->
        <geom name="{'vehicle-wing'}" {'density='+f"{WING_DENSITY}" if not self.mass_kg else ''}{'mass='+f"{self.mass_kg}" if self.mass_kg else ''} rgba="{WING_RGBA}" type="mesh" mesh="{'vehicle-wing-mesh'}"/>
        <camera name="track" pos="0 0 0" xyaxes="1 2 0 0 1 2" mode="track"/>
    </body>
    """

        asset = self.asset_from_vertices(
            vertices=self.vertices,
            faces=self.faces,
            mesh_name=("vehicle-wing-mesh"),
            scale=scale,
        )
        return body, asset

    def show(self):
        media.show_image(visualize.view_vehicle(*self.create_glider_from_vertices()))

    def exceeds_max_dim(self) -> bool:
        try:
            for vertex in self.vertices:
                for second_vertex in self.vertices:
                    norm = np.linalg.norm(np.array(vertex) - np.array(second_vertex))
                    assert norm <= self.max_dim_m
            return False
        except AssertionError:
            return True


def to_vertex_list(
    points: list,
) -> str:
    str_points = []

    for point in points:
        str_points.append(" ".join([str(coord) for coord in list(point)]))
    return " ".join(str_points)
