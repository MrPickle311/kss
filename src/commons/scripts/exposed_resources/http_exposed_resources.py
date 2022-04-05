from dataclasses import dataclass


@dataclass
class ExposedResources:
    station_longitude: float
    station_lattitude: float
    drone_battery_level: float
    station_battery_level: int
