from pydantic import BaseModel
from typing import List, Union

class TemperatureAndHumiditySensorData(BaseModel):
    idx: int
    ts: int
    c: int
    lb: bool
    t1: float
    h: float


class RainSensorData(BaseModel):
    idx: int
    ts: int
    c: int
    lb: bool
    sc: bool
    t1: float
    r: float
    rf: int
    rr: float


class WindSensorData(BaseModel):
    idx: int
    ts: int
    c: int
    lb: bool
    ws: int
    wg: int
    wd: int


class DeviceInfo(BaseModel):
    deviceid: str
    lastseen: int
    lowbattery: bool
    measurement: Union[TemperatureAndHumiditySensorData, RainSensorData, WindSensorData]


class MeteoTransaction(BaseModel):
    devices: List[DeviceInfo]
    success: bool
