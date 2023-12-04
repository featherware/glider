from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from PIL import Image
from base64 import b64encode
from io import BytesIO

from .schema import VehicleType

from glider import optimization, vehicle, visualization

app = FastAPI()

origins = ["http://localhost:5173"]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.get("/")
def read_root():
    return {"Hello": "World"}


@app.get("/vehicle/")
async def create_vehicle():
    return vehicle.Vehicle()


@app.post("/vehicle/drop_test/")
async def drop_test_vehicle(v: VehicleType):
    test_vehicle = vehicle.Vehicle(**v.model_dump())
    return optimization.drop_test_glider(test_vehicle)


@app.post("/vehicle/view/")
async def view_vehicle(v: VehicleType):
    test_vehicle = vehicle.Vehicle(**v.model_dump())
    bytes = BytesIO()
    Image.fromarray(visualization.view_vehicle(test_vehicle)).save(bytes, format="PNG")
    b64_image = b64encode(bytes.getvalue())
    return {"data": b64_image.decode("utf-8")}


@app.post("/vehicle/fitness/")
async def vehicle_fitness(v: VehicleType) -> float:
    test_vehicle = vehicle.Vehicle(**v.model_dump())
    return optimization.fitness_func(test_vehicle)
