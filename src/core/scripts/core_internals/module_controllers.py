import time
from typing import List, Callable
import json

from .collectors import DroneStateCollector, MeteoStateCollector, MPPTStateCollector, \
    AutomationStateCollector, \
    PowerManagementStateCollector, StationStateCollector
from exposed_resources.http_exposed_resources import ExposedResources
from module_io.module_interface import AbstractModuleController

from commons.msg import AutomationProcessAction, AutomationProcessFeedback, AutomationProcessResult, \
    AutomationProcessGoal
from commons.msg import FTPServerProcessGoal, FTPServerProcessFeedback, FTPServerProcessResult, FTPServerProcessAction
from commons.msg import HttpClientProcessAction, HttpClientProcessGoal, HttpClientProcessResult, \
    HttpClientProcessFeedback
from commons.msg import HttpServerProcessAction, HttpServerProcessGoal, HttpServerProcessResult, \
    HttpServerProcessFeedback
from commons.msg import MeteoProcessAction, MeteoProcessFeedback, MeteoProcessGoal, MeteoProcessResult
from commons.msg import PowerManagementProcessFeedback, PowerManagementProcessAction, PowerManagementProcessGoal, \
    PowerManagementProcessResult
from commons.msg import TrackerConnectionSignal, HttpExposedResourcesContent, SendImagesToServerEvent, StationState, \
    StationError, DroneState, HttpExposedResourcesContent
from commons.msg import TrackerProcessAction, TrackerProcessFeedback, TrackerProcessGoal, TrackerProcessResult
from commons.srv import AutomationStates, SwitchModulePower
from commons.srv import AutomationStatesRequest, SwitchModulePowerRequest

from commons.srv import DroneSimpleTask, DroneSimpleTaskRequest, DroneSimpleTaskResponse, TryUploadMission, \
    TryUploadMissionRequest, TryUploadMissionResponse, TryLand, TryLandRequest, TryLandResponse

from pathlib import Path

from data_models.kss_server import JsonMissionPackage, JsonMissionsMessage
from std_msgs.msg import Bool


class AutomationModuleController(AbstractModuleController):

    def __init__(self, automation_state: AutomationStateCollector):
        AbstractModuleController.__init__(self, 'automation_interface', AutomationProcessAction)
        self._automation_state = automation_state
        self.add_task_publisher('/state_apply', AutomationStates)

    def start_module(self):
        start_arguments = AutomationProcessGoal()
        start_arguments.host_address = '192.168.1.2'
        # start_arguments.host_address = '127.0.0.1'
        start_arguments.port = 4000
        start_arguments.server_address = 1
        start_arguments.state_register = 0
        AbstractModuleController.start_module(self, start_arguments)

    def process_feedback(self, feedback: AutomationProcessFeedback) -> None:
        self._automation_state.is_plc_connected = feedback.is_plc_connected
        self._automation_state.current_state = feedback.current_state

    def on_process_result_received(self, state: any, result: AutomationProcessResult) -> None:
        print(result.exit_code)

    def try_apply_state(self, state_to_apply: int) -> bool:
        task = AutomationStatesRequest(state_to_apply=state_to_apply)
        resp = self.send_task(AutomationStates, task)
        return resp.is_state_applied

    def try_open_roof(self) -> bool:
        return self.try_apply_state(1)

    def try_move_positioners_apart(self) -> bool:
        return self.try_apply_state(3)

    def try_slide_positioners_off(self) -> bool:
        return self.try_apply_state(5)

    def try_close_roof(self) -> bool:
        return self.try_apply_state(7)


class TrackerModuleController(AbstractModuleController):

    def __init__(self, mppt_state: MPPTStateCollector, connection_state_notify_handler: Callable[[int], None]):
        AbstractModuleController.__init__(self, 'tracker_interface', TrackerProcessAction)
        self._mppt_state = mppt_state
        self._connection_state_notify_handler = connection_state_notify_handler
        self.add_signal_receiver('/tracker_connection_state', TrackerConnectionSignal,
                                 self.receive_tracker_connection_state)

    def start_module(self):
        start_arguments = TrackerProcessGoal()
        start_arguments.serial_port = "/dev/ttyUSB0"
        start_arguments.baud_rate = 9600
        start_arguments.read_timeout = 0.5
        AbstractModuleController.start_module(self, start_arguments)

    def process_feedback(self, feedback: TrackerProcessFeedback) -> None:
        self._mppt_state.solar_panels_voltage = feedback.solar_panels_voltage
        self._mppt_state.battery_voltage = feedback.battery_voltage
        self._mppt_state.charging_current = feedback.charging_current
        self._mppt_state.load_voltage = feedback.load_voltage
        self._mppt_state.load_current = feedback.load_current
        self._mppt_state.charging_power = feedback.charging_power
        self._mppt_state.load_power = feedback.load_power
        self._mppt_state.station_battery_temperature = feedback.battery_temperature
        self._mppt_state.tracker_internal_temperature = feedback.internal_temperature
        self._mppt_state.battery_level = feedback.battery_level

    def on_process_result_received(self, state: any, result: TrackerProcessResult) -> None:
        print(result.exit_code)

    def receive_tracker_connection_state(self, state: TrackerConnectionSignal) -> None:
        ...
        # self._connection_state_notify_handler(state.signal_descriptor)


class PowerManagementModuleController(AbstractModuleController):

    def __init__(self, power_management_state: PowerManagementStateCollector):
        AbstractModuleController.__init__(self, 'power_manager_interface', PowerManagementProcessAction)
        self._power_management_state = power_management_state
        self.add_task_publisher('/power_switching', SwitchModulePower)

    def start_module(self):
        start_arguments = PowerManagementProcessGoal()
        start_arguments.port = 502
        # start_arguments.port = 4001
        start_arguments.host_address = '192.168.1.3'
        # start_arguments.host_address = '127.0.0.1'
        start_arguments.server_address = 1
        start_arguments.coils_offset = 8192
        # start_arguments.coils_offset = 0
        AbstractModuleController.start_module(self, start_arguments)

    def process_feedback(self, feedback: PowerManagementProcessFeedback) -> None:
        self._power_management_state = feedback.is_plc_connected

    def on_process_result_received(self, state: any, result: PowerManagementProcessResult) -> None:
        print(result.exit_code)

    def switch_module_power(self, module_id: int, is_turn_on: bool) -> None:
        task = SwitchModulePowerRequest(module_id=module_id, turn_on=is_turn_on)
        self.send_task(SwitchModulePower, task)

    def enable_edit_mode(self):
        self.switch_module_power(3, True)
        time.sleep(0.5)
        self.switch_module_power(3, False)
        time.sleep(10)

    def enable_drone_charging(self):
        self.enable_edit_mode()
        self.switch_module_power(3, True)
        time.sleep(3)
        self.switch_module_power(3, False)

    def disable_drone_charging(self):
        self.switch_module_power(3, True)
        time.sleep(0.5)
        self.switch_module_power(3, False)

    def enable_automation(self):
        self.switch_module_power(5, True)

    def disable_automation(self):
        self.switch_module_power(5, False)

    def disable_kss(self):
        self.switch_module_power(2, False)

    def enable_web_com(self):
        self.switch_module_power(4, True)

    def disable_web_com(self):
        self.switch_module_power(4, False)

    def enable_mkt(self):
        self.switch_module_power(0, True)

    def disable_mkt(self):
        self.switch_module_power(0, False)

    def enable_camera(self):
        self.switch_module_power(1, True)

    def disable_camera(self):
        self.switch_module_power(1, False)


class MeteoModuleController(AbstractModuleController):

    def __init__(self, meteo_state: MeteoStateCollector):
        AbstractModuleController.__init__(self, 'meteo_interface', MeteoProcessAction)
        self._meteo_state = meteo_state

    def start_module(self):
        start_arguments = MeteoProcessGoal()
        start_arguments.timeout = 20
        AbstractModuleController.start_module(self, start_arguments)

    def process_feedback(self, feedback: MeteoProcessFeedback) -> None:
        self._meteo_state._temperature = feedback.temperature
        self._meteo_state._humidity = feedback.humidity
        self._meteo_state._rain_mm = feedback.rain_mm
        self._meteo_state._wind_speed = feedback.wind_speed
        self._meteo_state._wind_gust = feedback.wind_gust
        self._meteo_state._wind_direction = feedback.wind_direction

    def on_process_result_received(self, state: any, result: MeteoProcessResult) -> None:
        print(result.exit_code)


class HttpServerController(AbstractModuleController):

    def __init__(self, drone_collector: DroneStateCollector):
        AbstractModuleController.__init__(self, 'http_server', HttpServerProcessAction)
        self.add_signal_sender('/exposed_resources', HttpExposedResourcesContent)
        self._drone_state_collector = drone_collector
        self.add_signal_receiver('/drone_state', DroneState, self.update_drone_state)
        self.add_signal_receiver('/is_simulated', Bool, self.switch_simulation)

        self.IS_SIMULATED = False

    def switch_simulation(self, msg: Bool):
        self.IS_SIMULATED = msg.data

    def start_module(self):
        start_arguments = HttpServerProcessGoal()
        start_arguments.host_address = '192.168.1.10'
        # start_arguments.host_address = '192.168.8.199'
        start_arguments.port = 8080
        AbstractModuleController.start_module(self, start_arguments)

    def process_feedback(self, feedback: HttpServerProcessFeedback) -> None:
        print('Feedback received')
        pass

    def on_process_result_received(self, state: any, result: HttpServerProcessResult) -> None:
        print(result.exit_code)

    def update_exposed_resources(self, exposed_resources: ExposedResources) -> None:
        content = HttpExposedResourcesContent()
        content.station_longitude = exposed_resources._station_longitude
        content.station_lattitude = exposed_resources._station_lattitude
        self.send_signal(HttpExposedResourcesContent, content)

    def update_drone_state(self, drone_state: DroneState):
        self._drone_state_collector.drone_lattitude = drone_state.latt
        self._drone_state_collector.drone_longitude = drone_state.long
        self._drone_state_collector.drone_altitude = drone_state.alt
        self._drone_state_collector.drone_battery_voltage = drone_state.voltage

        if self.IS_SIMULATED:
            self._drone_state_collector.drone_battery_temperature = 90
        else:
            self._drone_state_collector.drone_battery_temperature = drone_state.temperature


class HttpClientController(AbstractModuleController):

    def __init__(self, station_collector: StationStateCollector):
        AbstractModuleController.__init__(self, 'http_client', HttpClientProcessAction)
        self._station_collector = station_collector
        self.add_signal_sender('/send_station_state', StationState)
        self.add_signal_sender('/send_station_error', StationError)
        self.add_signal_sender('/send_images', SendImagesToServerEvent)
        self.add_signal_sender('/exposed_resources', HttpExposedResourcesContent)

    def start_module(self):
        start_arguments = HttpClientProcessGoal()
        start_arguments.host_address = '192.168.1.14'
        # start_arguments.host_address = '192.168.8.198'
        start_arguments.port = 8080
        start_arguments.station_id = 0
        start_arguments.images_directory = str(Path.home()) + "/pics/"
        AbstractModuleController.start_module(self, start_arguments)

    def process_feedback(self, feedback: HttpClientProcessFeedback) -> None:
        print('Feedback received')
        pass

    def on_process_result_received(self, state: any, result: HttpClientProcessResult) -> None:
        print(result.exit_code)

    def send_station_status(self, status: str) -> None:
        print(f'Sending state {status}')
        msg = StationState()
        msg.station_state = status
        self.send_signal(StationState, msg)

    def send_error(self, error: str) -> None:
        msg = StationError()
        msg.station_error = error
        self.send_signal(StationError, msg)

    def send_images(self, region_id: int) -> None:
        msg = SendImagesToServerEvent()
        msg.region_id = region_id
        self.send_signal(SendImagesToServerEvent, msg)

    def send_exposed_resources(self):
        msg = HttpExposedResourcesContent()
        msg.drone_longitude = self._station_collector.drone_state.drone_longitude
        msg.drone_lattitude = self._station_collector.drone_state.drone_lattitude
        msg.drone_battery_voltage = self._station_collector.drone_state.drone_battery_voltage
        msg.drone_battery_temperature = self._station_collector.drone_state.drone_battery_temperature
        msg.station_accumulators_temperature = self._station_collector.mppt_state.station_battery_temperature
        msg.station_accumulators_level = self._station_collector.mppt_state.battery_level
        self.send_signal(HttpExposedResourcesContent, msg)


class ComClientController(AbstractModuleController):

    def __init__(self):
        AbstractModuleController.__init__(self, 'com_client', HttpClientProcessAction)
        self.add_task_publisher('/drone_task', DroneSimpleTask)
        self.add_task_publisher('/start_mission', TryUploadMission)

    def start_module(self):
        start_arguments = HttpClientProcessGoal()
        start_arguments.host_address = '192.168.1.16'
        # start_arguments.host_address = '192.168.8.198'
        start_arguments.port = 4000
        start_arguments.station_id = 0
        start_arguments.images_directory = str(Path.home()) + "/pics"
        AbstractModuleController.start_module(self, start_arguments)

    def process_feedback(self, feedback: HttpClientProcessFeedback) -> None:
        print('Feedback received')
        pass

    def on_process_result_received(self, state: any, result: HttpClientProcessResult) -> None:
        print(result.exit_code)

    def do_simple_task(self, task: str) -> bool:
        req = DroneSimpleTaskRequest(task)
        resp: DroneSimpleTaskResponse = self.send_task(DroneSimpleTask, req)
        return resp.is_ok

    def try_start_drone(self, mission_body: JsonMissionsMessage) -> bool:
        req = TryUploadMissionRequest(mission_body.json())
        resp: TryUploadMissionResponse = self.send_task(TryUploadMission, req)
        return resp.is_mission_sent

    def abort_drone_mission(self) -> bool:
        return self.do_simple_task('abort_drone')

    def try_normally_land_drone(self) -> bool:
        return self.do_simple_task('land_drone_normally')

    def try_emergency_land_drone(self) -> bool:
        return self.do_simple_task('land_drone_emergency')


class FTPServerController(AbstractModuleController):

    def __init__(self):
        AbstractModuleController.__init__(self, 'ftp_server', FTPServerProcessAction)

    def start_module(self):
        start_arguments = FTPServerProcessGoal()
        start_arguments.host_address = '192.168.1.10'
        # start_arguments.host_address = '192.168.8.199'
        start_arguments.port = 2121
        start_arguments.ftp_password = 'oze'
        start_arguments.user = 'oze'
        # start_arguments.files_directory = "/home/damiano/pics"
        start_arguments.files_directory = str(Path.home()) + "/pics"
        AbstractModuleController.start_module(self, start_arguments)

    def process_feedback(self, feedback: FTPServerProcessFeedback) -> None:
        pass

    def on_process_result_received(self, state: any, result: FTPServerProcessResult) -> None:
        print(result.exit_code)


class StationModules:
    def __init__(self, state_collector: StationStateCollector,
                 service_mppt_connection_state_callback: Callable[[int], None]):
        self.automation_controller = AutomationModuleController(state_collector.automation_state)
        self.tracker_controller = TrackerModuleController(state_collector.mppt_state,
                                                          service_mppt_connection_state_callback)
        self.power_controller = PowerManagementModuleController(state_collector.power_management_state)
        self.meteo_controller = MeteoModuleController(state_collector.meteo_state)
        self.http_server_controller = HttpServerController(state_collector.drone_state)
        self.http_client_controller = HttpClientController(state_collector)
        self.com_client_controller = ComClientController()
        self.ftp_server_controller = FTPServerController()

    def init_modules(self):
        self.automation_controller.start_module()
        self.power_controller.start_module()
        self.meteo_controller.start_module()
        self.tracker_controller.start_module()
        time.sleep(3)
        self.ftp_server_controller.start_module()
        time.sleep(2)
        self.http_server_controller.start_module()
        self.http_client_controller.start_module()
        self.com_client_controller.start_module()
        time.sleep(2)
