from fastapi import FastAPI
from schema import VehicleType

from glider import simulation, vehicle

app = FastAPI()


@app.get("/")
def read_root():
    return {"Hello": "World"}


@app.post("/vehicle/")
async def create_vehicle(vertices: list[list[int]], faces: list[list[int]]):
    return vehicle.Vehicle(vertices=vertices, faces=faces)


@app.post("/vehicle/drop_test/")
async def drop_test_vehicle(v: VehicleType):
    test_vehicle = vehicle.Vehicle(**v.dict())
    return simulation.drop_test_glider(test_vehicle)
