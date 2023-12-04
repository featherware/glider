import pymongo

current_data_version = 'v1'


def get_db():
    try:
        client = pymongo.MongoClient("mongodb://mongo:27017/")
        db = client["gliders"]
        yield db
    finally:
        client.close()


def write(db, data, data_version=current_data_version):
    db[f"data-{data_version}"].insert_one(data)
