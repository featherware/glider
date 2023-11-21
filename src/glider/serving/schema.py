from pydantic import BaseModel


class VehicleType(BaseModel):
    faces: list[list[int]] | None
    vertices: list[list[float]] | None
    max_dim_m: float | None
    mass_kg: float | None
    orientation: list[float] | None
    wing_density: float | None
    pilot: bool = False
