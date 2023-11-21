from fastapi import FastAPI

from glider import vehicle

app = FastAPI()


@app.get("/")
def read_root():
    return {"Hello": "World"}


@app.post("/vehicle/")
async def create_vehicle(vertices: list[list[int]], faces: list[list[int]]):
    return vehicle.Vehicle(vertices=vertices, faces=faces)
