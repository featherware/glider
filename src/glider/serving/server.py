from fastapi import FastAPI

import glider

app = FastAPI()


@app.get("/")
def read_root():
    return {"Hello": "World"}


@app.get("/vehicle")
def read_vehicle():
    return bool(glider)
