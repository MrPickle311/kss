from pydantic import BaseModel


class DroneStateMessage(BaseModel):
    lattitude: float
    longitude: float
    altitude: float
    battery_volt: float
    battery_temp: float


class DroneEventModel(BaseModel):
    event: str
