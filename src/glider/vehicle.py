from dataclasses import dataclass, field
from typing import Any

import mediapy as media
import numpy as np
import trimesh

import glider.visualization as visualization

from .constants import (DEFAULT_MAX_WING_DIMENSION_M, MUTATION_CHANCE,
                        MUTATION_RATIO, WING_DENSITY, WING_RGBA,
                        PILOT_RGBA, PILOT_DIMENSIONS_M, PILOT_MASS_KG,
                        create_pilot_geom)


@dataclass
class VehicleConfig:
    num_vertices: int = 30
    max_dim_m: float = DEFAULT_MAX_WING_DIMENSION_M
    pilot: bool = False
    mass_kg: float | None = None
    wing_density: float = WING_DENSITY
    orientation: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])


class Vehicle:
    """
    Class responsible for generating the XML for a vehicle.

    Args:
        vertices (list):        A list of vertices for the main wing.
        faces (list):           Lists of indices for the vertices,
                                which define triangluar faces for the mesh.
                                Clockwise order is assumed.
        num_vertices (int):     A number of vertices to randomly generate.
                                This is ignored if 'vertices' is defined.
                                Defaults to 30.
        max_dim_m (float):      The maximum wing dimension of the vehicle.
                                Default: contants.DEFAULT_MAX_WING_DIMENSION_M.
        pilot (bool):           Adds a human-sized pilot to the vehicle.
                                Defaults to False.
        mass_kg (float):        Explicitely sets the mass of the wing.
                                If not defined, the wing density is set
                                to constants.WING_DENSITY.
                                Defaults to None.
        wing_density (float):   The material density for the wing.
                                Ignored if 'mass_kg' is defined explicitely.
                                Defaults to constants.WING_DENSITY.
        orientation (list):     Specify a custom orientation with euler angles.
    """

    def __init__(
        self,
        vertices: list | None = None,
        faces: list | None = None,
        num_vertices: int = 30,
        max_dim_m: float = DEFAULT_MAX_WING_DIMENSION_M,
        pilot: bool = False,
        wing_density: float = WING_DENSITY,
        mass_kg: float | None = None,
        orientation: list[float] = [0.0, 0.0, 0.0],
    ):
        super(Vehicle, self).__init__()

        self.max_dim_m = max_dim_m
        self.mass_kg = mass_kg
        self.orientation = orientation
        self.wing_density = wing_density
        self.faces = faces if faces else []
        self.pilot = pilot

        if vertices is not None:
            self.vertices = vertices
        else:
            self.initialize_vertices(num_vertices, max_dim_m)

    def config(self) -> VehicleConfig:
        return VehicleConfig(
            max_dim_m=self.max_dim_m,
            pilot=self.pilot,
            mass_kg=self.mass_kg,
            wing_density=self.wing_density,
            orientation=self.orientation,
        )

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
                        dim += (
                            self.max_dim_m * MUTATION_RATIO * np.random.choice((-1, 1))
                        )
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

    def get_wing_asset(self):
        if self.faces:
            return f"""
            <asset>
                <mesh name="{'vehicle-wing-mesh'}" vertex="{to_vertex_list(self.vertices)}" face="{to_vertex_list(self.faces)}"/>
            </asset>"""
        else:
            return f"""
            <asset>
                <mesh name="{'vehicle-wing-mesh'}" vertex="{to_vertex_list(self.vertices)}"/>
            </asset>"""

    def xml(self) -> tuple[str, str]:
        density_tag = f'density="{WING_DENSITY}"'
        mass_tag = f'mass="{self.mass_kg}"'
        pos_tag = f'pos="{" ".join([str(-self.max_dim_m // 2) for _ in range(3)])}"'

        body_xml = f"""
    <body name="body" pos="0 0 0" euler="{' '.join(map(str, self.orientation))}">
        <freejoint/>
        <!-- Main Wing -->
        <geom name="{'vehicle-wing'}" {density_tag if not self.mass_kg else mass_tag} {pos_tag} rgba="{WING_RGBA}" type="mesh" mesh="{'vehicle-wing-mesh'}"/>
        <camera name="track" pos="0 0 0" xyaxes="1 2 0 0 1 2" mode="track"/>
        {create_pilot_geom() if self.pilot else ''}
    </body>
    """

        asset_xml = self.get_wing_asset()
        return body_xml, asset_xml

    def show(self):
        media.show_image(visualization.view_vehicle(*self.xml()))

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
