import time
from abc import ABC, abstractmethod
from .module_controllers import StationModules
from typing import Callable, List
from .collectors import StationStateCollector, StationEventFlags, ErrorFlags
from waiting import wait
from multitimer import MultiTimer
import os
from enums.kss_core import StationStateIndicator, CheckEnum, StationErrors
from commons.msg import MissionsUploadedEvent, HttpServerSimpleEvent, ReceivedImagesFromFtpServerEvent, DroneEvent
from .mission_storage import MissionStorage
from data_models.kss_server import JsonMissionsMessage, JsonMissionPackage

class FireProtectionException(Exception):
    pass


def fire_protected(flags_source: ErrorFlags):
    def _fire_protected(method: Callable[[any], None]):
        def wrapper(*args, **kwargs) -> any:
            if flags_source.is_tracker_temperature_good and \
                    flags_source.is_drone_battery_temperature_good and \
                    flags_source.is_station_battery_temperature_good:
                return method(*args, **kwargs)
            else:
                raise FireProtectionException

        return wrapper

    return _fire_protected


# TODO: oprócz sprawdzania gui eventow, trzeba tez sprawdzac, czy np. pomyslnie zapisano 3 do automatyki


class StationState(ABC):
    current_station_data: StationStateCollector = None
    station_modules: StationModules = None
    station_event_flags: StationEventFlags = None
    error_flags: ErrorFlags = None
    mission_storage: MissionStorage = None
    station_state_indicator: str = StationStateIndicator.STATION_NOT_INITIALIZED

    simple_events = {
        'start_drone': lambda: ...,
        'land_drone_normally': lambda: ...,
        'land_drone_emergency': lambda: ...,
        'abort_drone': lambda: StationState.station_modules.com_client_controller.abort_drone_mission(),
        'on_camera': lambda: StationState.station_modules.power_controller.enable_camera(),
        'off_camera': lambda: StationState.station_modules.power_controller.disable_camera(),
        'open_roof': lambda: StationState.try_open_roof(),
        'position_start': lambda: StationState.try_move_positioners_apart(),
        'position_return': lambda: StationState.try_slide_positioners_off(),
        'close_roof': lambda: StationState.try_close_roof()}

    drone_events = {
        'drone_is_ready_to_land': lambda: ...,
        'drone_landed': lambda: ...,
        'com_initialized': lambda: ...
    }

    def maybe_not_successful(error_name: str, state_to_inject_after_success: str):
        def _maybe_not_successful(method: Callable[[any], bool]):
            def wrapper(*args, **kwargs):
                if method(*args, **kwargs):
                    StationState.update_station_state(state_to_inject_after_success)
                else:
                    StationState.station_modules.http_client_controller.send_error(error_name)

            return wrapper

        return _maybe_not_successful

    def wait_to_be_successful(error_name: str, state_to_inject_after_success: str):
        def _wait_to_be_successful(method: Callable[[any], bool]):
            def wrapper(*args, **kwargs):
                while True:
                    if method(*args, **kwargs):
                        StationState.update_station_state(state_to_inject_after_success)
                        break
                    else:
                        StationState.station_modules.http_client_controller.send_error(error_name)

            return wrapper

        return _wait_to_be_successful

    @staticmethod
    def received_gui_event(simple_event: HttpServerSimpleEvent):
        event = simple_event.event_name.split(sep='/')[-1]
        StationState.simple_events[event]()
        StationState.station_event_flags.gui_event_presence = event
        print(f'received gui event {event}')

    @staticmethod
    def received_drone_event(drone_event: DroneEvent):
        print(f'received drone event {drone_event.event}')
        StationState.drone_events[drone_event.event]()
        StationState.station_event_flags.drone_state_presence = drone_event.event
        print(f'received drone event {drone_event.event}')

    @staticmethod
    def init_station_states_scope():
        StationState.station_modules.http_server_controller.add_signal_receiver('/simple_http_event',
                                                                                HttpServerSimpleEvent,
                                                                                StationState.received_gui_event)

        StationState.station_modules.http_server_controller.add_signal_receiver('/drone_event', DroneEvent,
                                                                                StationState.received_drone_event)

    def __init__(self, check_interval=0.1):
        self._check_interval = check_interval

    @staticmethod
    @maybe_not_successful(StationErrors.CANNOT_OPEN_ROOF, StationStateIndicator.ROOF_OPENED)
    def try_open_roof():
        res = StationState.station_modules.automation_controller.try_open_roof()

        if res:
            StationState.wait_for_roof_open()

        return res

    @staticmethod
    @maybe_not_successful(StationErrors.CANNOT_MOVE_POSITIONERS_APART, StationStateIndicator.MOVED_POSITIONERS_APART)
    def try_move_positioners_apart():
        res = StationState.station_modules.automation_controller.try_move_positioners_apart()

        if res:
            StationState.wait_for_positioners_move_apart()

        return res

    @staticmethod
    @maybe_not_successful(StationErrors.CANNOT_SLIDE_POSITIONERS_OFF, StationStateIndicator.POSITIONERS_SLIDED_OFF)
    def try_slide_positioners_off():
        res = StationState.station_modules.automation_controller.try_slide_positioners_off()

        if res:
            StationState.wait_for_positioners_slide_off()

        return res

    @staticmethod
    @maybe_not_successful(StationErrors.CANNOT_CLOSE_ROOF, StationStateIndicator.ROOF_CLOSED)
    def try_close_roof():
        res = StationState.station_modules.automation_controller.try_close_roof()

        if res:
            StationState.wait_for_roof_close()

        return res

    @staticmethod
    def wait_for_predicate(predicate: Callable[[], bool], check_interval=0.1):
        wait(predicate=predicate, sleep_seconds=check_interval)

    @staticmethod
    def update_station_state(state: str):
        StationState.station_modules.http_client_controller.send_station_status(state)
        StationState.station_state_indicator = state

    @staticmethod
    def wait_for_roof_open():
        @fire_protected(StationState.error_flags)
        def predicate() -> bool:
            return StationState.current_station_data.automation_state.current_state == 2  # TODO: make commons enums

        StationState.wait_for_predicate(predicate)

    @staticmethod
    def wait_for_positioners_move_apart():
        @fire_protected(StationState.error_flags)
        def predicate() -> bool:
            return StationState.current_station_data.automation_state.current_state == 4  # TODO: make commons enums

        StationState.wait_for_predicate(predicate)

    @staticmethod
    def wait_for_positioners_slide_off():
        @fire_protected(StationState.error_flags)
        def predicate() -> bool:
            return StationState.current_station_data.automation_state.current_state == 6  # TODO: make commons enums

        StationState.wait_for_predicate(predicate)

    @staticmethod
    def wait_for_roof_close():
        @fire_protected(StationState.error_flags)
        def predicate() -> bool:
            return StationState.current_station_data.automation_state.current_state == 8  # TODO: make commons enums

        StationState.wait_for_predicate(predicate)

    @staticmethod
    def reset_gui_event_flag():
        StationState.station_event_flags.gui_event_presence = 'none'

    @staticmethod
    def reset_drone_event_flag():
        StationState.station_event_flags.drone_state_presence = 'none'

    def wait_for_gui_events(self, events: List[str]) -> str:
        print(f'waiting for gui events:  {events}')

        caught_event = 'none'

        @fire_protected(self.error_flags)
        def predicate() -> bool:
            if StationState.station_event_flags.gui_event_presence in events:
                nonlocal caught_event
                caught_event = StationState.station_event_flags.gui_event_presence
                return True
            return False

        self.wait_for_predicate(predicate)
        self.reset_gui_event_flag()
        return caught_event

    def wait_for_com_event(self, events: List[str]) -> str:
        print(f'waiting for com events:  {events}')

        caught_event = 'none'

        @fire_protected(self.error_flags)
        def predicate() -> bool:
            if StationState.station_event_flags.drone_state_presence in events:
                nonlocal caught_event
                caught_event = StationState.station_event_flags.drone_state_presence
                return True
            return False

        self.wait_for_predicate(predicate)
        self.reset_drone_event_flag()
        return caught_event

    @abstractmethod
    def execute(self):
        ...


class FireProtectionState(StationState):

    def __init__(self):
        StationState.__init__(self)
        self._sequence = {
            2: lambda: self.simple_events['position_start'](),
            4: lambda: self.simple_events['position_return'](),
            6: lambda: self.simple_events['close_roof']()
        }

    def execute(self):
        self.close_roof()
        self.notify_about_error()
        self.cut_down_power()
        self.shutdown_system()

    def wait_for_automation_state(self, state: int):
        def predicate() -> bool:
            return self.current_station_data.automation_state.current_state == state

        self.wait_for_predicate(predicate)

    def close_roof(self):
        if self.current_station_data.automation_state.current_state in [0, 8]:
            return

        while True:
            state = self.current_station_data.automation_state.current_state
            self.station_modules.automation_controller.try_apply_state(state + 1)
            self.wait_for_automation_state(state + 2)
            if self.current_station_data.automation_state.current_state == 8:
                break

    def notify_about_error(self):
        self.station_modules.http_client_controller.send_error('fire')

    def cut_down_power(self):
        print('Cutting down power')
        power_manager = self.station_modules.power_controller
        power_manager.disable_automation()
        power_manager.disable_drone_charging()
        power_manager.disable_mkt()
        power_manager.disable_web_com()
        power_manager.disable_kss()

    @staticmethod
    def shutdown_system():
        os.system("systemctl poweroff")


class InitializationState(StationState):

    def execute(self):
        self.station_modules.init_modules()
        self.init_attached_peripherals()

        def predicate() -> bool:
            return StationState.station_event_flags.drone_state_presence == 'com_initialized'

        print('Waiting for mkt')
        StationState.wait_for_predicate(predicate)

        self.update_station_state(StationStateIndicator.STATION_INITIALIZED)
        time.sleep(5)
        print('Station initialized')

    @staticmethod
    def wait_for_enable():
        def predicate() -> bool:
            return StationState.station_event_flags.is_mkt_initialized

        StationState.wait_for_predicate(predicate)

    def init_attached_peripherals(self):
        self.station_modules.power_controller.enable_automation()
        print('Automation enabled')
        self.station_modules.power_controller.enable_web_com()
        print('Enabled web com')
        self.station_modules.power_controller.disable_mkt()
        time.sleep(5)
        self.station_modules.power_controller.enable_mkt()
        # self.wait_for_enable()
        print('Enabled MKT')
        print('Initialized attached peripherals')


class IdleState(StationState):
    BATTERY_CHECK_PERIOD = 0.4
    MAX_DRONE_BATTERY_VOLTAGE = 16.6
    MIN_DRONE_BATTERY_VOLTAGE = 15.5

    def __init__(self):
        StationState.__init__(self)
        self.station_modules.http_server_controller.add_signal_receiver('/missions_uploaded_event',
                                                                        MissionsUploadedEvent,
                                                                        self.receive_missions_from_server)
        self._battery_checker = MultiTimer(self.BATTERY_CHECK_PERIOD, self.check_battery_level)
        self._is_charger_turned_on = False

    def execute(self):
        print('Battery checking')
        self.start_battery_checking()
        print('Mission presence checking')
        self.check_mission_lack()
        print('Waiting for station procedure start')
        self.wait_for_start()
        print('Stopping battery checking')
        self.stop_battery_checking()
        if self._is_charger_turned_on:
            self.turn_off_battery_charging()
        print('Idle state finished')

    def receive_missions_from_server(self, missions_event: MissionsUploadedEvent):
        body = JsonMissionsMessage.parse_raw(missions_event.missions_body)
        self.mission_storage.insert_missions(body)

    def wait_for_start(self):

        @fire_protected(self.error_flags)
        def predicate() -> bool:
            return self.is_good_weather() and \
                   self.are_missions_to_do() and \
                   self.is_good_battery_level()

        self.wait_for_predicate(predicate, 1)
        # self.update_station_state(StationStateIndicator.STATION_READY_TO_START)
        print('Station is ready to start')

    def check_battery_level(self):
        if not self.is_good_battery_level():
            print(f'Battery level {self.current_station_data.drone_state.drone_battery_voltage}')
            if self.is_battery_low_level() and not self._is_charger_turned_on:
                print('Battery level is low')
                self.turn_on_battery_charging()
                self._is_charger_turned_on = True
            if self.is_fully_charged() and self._is_charger_turned_on:
                print('Battery level is ok')
                self.turn_off_battery_charging()
                self._is_charger_turned_on = False

    def start_battery_checking(self):
        self._battery_checker.start()

    def stop_battery_checking(self):
        self._battery_checker.stop()

    def is_fully_charged(self):
        return self.current_station_data.drone_state.drone_battery_voltage >= self.MAX_DRONE_BATTERY_VOLTAGE

    def is_good_battery_level(self) -> bool:
        return self.MIN_DRONE_BATTERY_VOLTAGE <= self.current_station_data.drone_state.drone_battery_voltage <= self.MAX_DRONE_BATTERY_VOLTAGE

    def is_battery_low_level(self) -> bool:
        return self.current_station_data.drone_state.drone_battery_voltage < self.MIN_DRONE_BATTERY_VOLTAGE

    def turn_on_battery_charging(self):
        self.station_modules.power_controller.enable_drone_charging()

    def turn_off_battery_charging(self):
        self.station_modules.power_controller.disable_drone_charging()

    def is_good_weather(self):
        return self.error_flags.is_weather_good

    def are_missions_to_do(self):
        return not self.mission_storage.is_empty()

    def check_mission_lack(self):
        if self.mission_storage.is_empty():
            self.ask_for_missions()

    def ask_for_missions(self):
        self.station_modules.http_client_controller.send_station_status(StationStateIndicator.READY_FOR_NEW_MISSIONS)


class MissionSelector:
    def __init__(self, mission_storage: MissionStorage):
        self._mission_storage = mission_storage

    def get_next_mission(self) -> JsonMissionsMessage:
        mission = self._mission_storage.get_next_mission()
        return JsonMissionsMessage(region_id='', missions=[mission])


class StartState(StationState):

    def __init__(self):
        StationState.__init__(self)
        self._mission_selector = MissionSelector(self.mission_storage)

    def execute(self):
        print('Awaiting for automation open')
        self.wait_for_positioners_move_apart()
        print('Awaiting for drone start')
        self.try_start_drone(self._mission_selector.get_next_mission())
        print('Awaiting for automation close')
        self.wait_for_roof_close()
        print('Start state finished')

    @StationState.wait_to_be_successful(StationErrors.CANNOT_START_DRONE, StationStateIndicator.DRONE_STARTED)
    def try_start_drone(self, mission_body: JsonMissionsMessage) -> bool:
        self.wait_for_start_drone()
        return self.station_modules.com_client_controller.try_start_drone(mission_body)

    def wait_for_start_drone(self):
        self.wait_for_gui_events(['start_drone'])


class DroneFlightState(StationState):
    def execute(self):
        print('Awaiting for drone landing ready')
        self.wait_for_landing_ready()
        print('Drone flight state is finished')

    def wait_for_landing_ready(self):
        self.wait_for_com_event(['drone_is_ready_to_land'])
        print('Drone is ready to land')
        self.update_station_state(StationStateIndicator.DRONE_READY_FOR_LANDING)


class DroneLandingServiceState(StationState):
    def execute(self):
        print('Awaiting for automation open')
        self.wait_for_positioners_move_apart()
        print('Trying drone land')
        self.try_drone_land()
        print('Waiting for drone land')
        self.wait_until_drone_land()
        print('Awaiting for automation close')
        self.wait_for_roof_close()
        print('Drone service state is finished')

    def try_land_drone_normally(self) -> bool:
        return self.station_modules.com_client_controller.try_normally_land_drone()

    def try_land_drone_emergency(self) -> bool:
        return self.station_modules.com_client_controller.try_emergency_land_drone()

    @StationState.wait_to_be_successful(StationErrors.CANNOT_LAND_DRONE, StationStateIndicator.DRONE_LANDING)
    def try_drone_land(self):
        landing_mode = self.wait_for_land_drone_confirmation()
        if landing_mode == 'land_drone_normally':
            return self.try_land_drone_normally()
        elif landing_mode == 'land_drone_emergency':
            return self.try_land_drone_emergency()
        return False

    def wait_for_land_drone_confirmation(self) -> str:
        return self.wait_for_gui_events(['land_drone_normally', 'land_drone_emergency'])

    def wait_until_drone_land(self):
        # self.wait_for_com_events(['drone_landed'])
        time.sleep(20)
        self.update_station_state(StationStateIndicator.DRONE_LANDED)


class DroneAfterLandingServiceState(StationState):
    REGION_ID = 0

    def __init__(self):
        StationState.__init__(self)
        self.station_modules.ftp_server_controller.add_signal_receiver('/received_images_from_ftp_event',
                                                                       ReceivedImagesFromFtpServerEvent,
                                                                       self.images_received)

    def execute(self):
        print('Waiting for images from drone')
        self.wait_for_receive_images_from_drone()
        print('Sending images from kss to server')
        self.send_images_from_kss_to_server()
        self.update_station_state(StationStateIndicator.DRONE_SERVICED)
        print('Drone after landing service state is finished')

    def send_images_from_kss_to_server(self):
        self.station_modules.http_client_controller.send_images(self.REGION_ID)

    def wait_for_receive_images_from_drone(self):
        @fire_protected(StationState.error_flags)
        def predicate():
            return self.station_event_flags.are_images_received_event

        self.wait_for_predicate(predicate)
        self.station_event_flags.are_images_received_event = False

    def images_received(self, images_event: ReceivedImagesFromFtpServerEvent):
        self.station_event_flags.are_images_received_event = True
