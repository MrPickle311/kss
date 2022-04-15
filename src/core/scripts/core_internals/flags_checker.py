from .collectors import StationStateCollector, StationEventFlags, ErrorFlags
from multitimer import MultiTimer


class FlagsChecker:
    DRONE_BATTERY_MAX_TEMP = 40
    STATION_BATTERY_MAX_TEMP = 50
    TRACKER_BATTERY_MAX_TEMP = 50

    ENV_MAX_TEMP = 40.0
    ENV_MAX_HUMIDITY = 0.99
    ENV_MAX_RAIN_MM = 20.0
    ENV_MAX_WIND_SPEED = 10.0
    ENV_MAX_GUST = 25.0

    MIN_SOLAR_PANELS_VOLTAGE = 40

    MAX_CHARGING_CURRENT = 5

    def __init__(self, station_state_collector: StationStateCollector, error_flags: ErrorFlags, check_period: float):
        self._station_state_collector = station_state_collector
        self._error_flags = error_flags

        self._timer = MultiTimer(check_period, self.check_state_collectors)

    def start_checking(self):
        self._timer.start()

    def stop_checking(self):
        self._timer.stop()

    def check_state_collectors(self):
        self.check_drone_battery_temperature()
        self.check_station_battery_temperature()
        self.check_tracker_temperature()

        # self.check_automation_connection()
        # self.check_power_management_connection()
        # self.check_tracker_connection()

        self.check_weather()

        # self.check_solar_panels_voltage()

    def check_drone_battery_temperature(self):
        if self._station_state_collector.drone_state.drone_battery_temperature > self.DRONE_BATTERY_MAX_TEMP:
            self._error_flags.is_drone_battery_temperature_good = False
        else:
            self._error_flags.is_drone_battery_temperature_good = True

    def check_station_battery_temperature(self):
        if self._station_state_collector.mppt_state.station_battery_temperature > self.STATION_BATTERY_MAX_TEMP:
            self._error_flags.is_station_battery_temperature_good = False
        else:
            self._error_flags.is_station_battery_temperature_good = True

    def check_tracker_temperature(self):
        if self._station_state_collector.mppt_state.tracker_internal_temperature > self.TRACKER_BATTERY_MAX_TEMP:
            self._error_flags.is_tracker_temperature_good = False
        else:
            self._error_flags.is_tracker_temperature_good = True

    def check_automation_connection(self):
        if self._station_state_collector.automation_state.is_plc_connected:
            self._error_flags.is_automation_connected = True
        else:
            self._error_flags.is_automation_connected = False

    def check_power_management_connection(self):
        if self._station_state_collector.power_management_state.is_plc_connected:
            self._error_flags.is_power_management_connected = True
        else:
            self._error_flags.is_power_management_connected = False

    # TODO: plug in tracker connection info
    def check_tracker_connection(self):
        if self._station_state_collector.mppt_state.tracker_connection_state == 0:
            self._error_flags.is_tracker_connected = True
        else:
            self._error_flags.is_tracker_connected = False

    def check_weather(self):
        meteo_data = self._station_state_collector.meteo_state
        if meteo_data.rain_mm < self.ENV_MAX_RAIN_MM and \
                meteo_data.temperature < self.ENV_MAX_TEMP and \
                meteo_data.humidity < self.ENV_MAX_HUMIDITY and \
                meteo_data.wind_speed < self.ENV_MAX_WIND_SPEED and \
                meteo_data.wind_gust < self.ENV_MAX_GUST:
            self._error_flags.is_weather_good = True
        else:
            self._error_flags.is_weather_good = False

    def check_solar_panels_voltage(self):
        if self._station_state_collector.mppt_state.solar_panels_voltage < self.MIN_SOLAR_PANELS_VOLTAGE:
            self._error_flags.is_solar_panels_voltage_good = False
        else:
            self._error_flags.is_solar_panels_voltage_good = True
