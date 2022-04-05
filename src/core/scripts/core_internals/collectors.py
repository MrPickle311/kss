from dataclasses import dataclass
from typing import Dict


@dataclass
class PowerManagementStateCollector:
    is_plc_connected: bool
    modules_power_states: Dict[int, bool]


@dataclass
class AutomationStateCollector:
    is_plc_connected: bool
    current_state: int


@dataclass
class MeteoStateCollector:
    temperature: float
    humidity: float
    rain_mm: float
    wind_speed: int
    wind_gust: int
    wind_direction: int


@dataclass
class MPPTStateCollector:
    solar_panels_voltage: float

    battery_voltage: float

    charging_current: float

    load_voltage: float
    load_current: float

    charging_power: int
    load_power: int

    station_battery_temperature: int

    tracker_internal_temperature: int

    battery_level: int

    tracker_connection_state: int


@dataclass
class DroneStateCollector:
    drone_longitude: float
    drone_lattitude: float
    drone_altitude: float
    drone_battery_voltage: float
    drone_battery_temperature: float


@dataclass
class StationPositionCollector:
    station_longitude: float
    station_lattitude: float


class StationStateCollector:
    station_position = StationPositionCollector(0.0, 0.0)
    power_management_state = PowerManagementStateCollector(False, {})
    automation_state = AutomationStateCollector(False, -1)
    meteo_state = MeteoStateCollector(0.0, 0.0, 0.0, 0, 0, 0)
    mppt_state = MPPTStateCollector(0.0, 0.0, 0.0, 0.0, 0.0, 0, 0, 0, 0, 0, 0)
    drone_state = DroneStateCollector(0.0, 0.0, 0.0, 0, 0.0)


# TODO: add class - event pool
@dataclass
class StationEventFlags:
    drone_state_presence: str
    gui_event_presence: str
    are_images_received_event: bool
    drone_landing_mode: str
    is_mkt_initialized: bool


@dataclass
class ErrorFlags:
    is_drone_battery_temperature_good: bool
    is_station_battery_temperature_good: bool
    is_tracker_temperature_good: bool

    is_automation_connected: bool
    is_power_management_connected: bool
    is_tracker_connected: bool

    is_weather_good: bool

    is_solar_panels_voltage_good: bool
    is_battery_voltage_good: bool

    is_charging_current_good: bool
    is_load_current_good: bool
