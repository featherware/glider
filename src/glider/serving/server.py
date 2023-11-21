from fastapi import FastAPI
from schema import VehicleType

from glider import optimization, simulation, vehicle

app = FastAPI()


@app.get("/")
def read_root():
    return {"Hello": "World"}


@app.post("/vehicle/")
async def create_vehicle(
    vertices: list[list[int]] | None = None,
    faces: list[list[int]] | None = None,
):
    return vehicle.Vehicle(vertices=vertices, faces=faces)


@app.post("/vehicle/drop_test/")
async def drop_test_vehicle(v: VehicleType):
    test_vehicle = vehicle.Vehicle(**v.dict())
    return simulation.drop_test_glider(test_vehicle)


@app.post("/vehicle/fitness/")
async def vehicle_fitness(v: VehicleType) -> float:
    test_vehicle = vehicle.Vehicle(**v.dict())
    return optimization.fitness_func(test_vehicle)
