from typing import List
from pydantic import BaseModel


class JsonMissionPoint(BaseModel):
    kolejnosc: int
    is_current: int
    frame: int
    command: int
    param1: int
    param2: int
    param3: int
    param4: int
    x_lat: float
    y_long: float
    z_alt: float
    autocontinue: int


class JsonMissionPackage(BaseModel):
    route_id: int
    route_variant: int
    mission_points: List[JsonMissionPoint]


class JsonMissionsMessage(BaseModel):
    region_id: str
    missions: List[JsonMissionPackage]
