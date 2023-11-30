from sqlalchemy import Column, Integer, String, Float

from .database import Base


class Glider(Base):
    __tablename__ = "gliders"

    id = Column(Integer, primary_key=True, index=True, autoincrement=True)
    vertices = Column(String)
    faces = Column(String)
    fitness = Column(Float)
    ancestor_uuid = Column(String)
