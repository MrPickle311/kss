from server.upload_mission_event import MissionReceiver
from server.kss_receiver import KSSServerAPI
from http_io.simple_server_entities import SimpleResourceGetter, SimpleHttpEventProcessor, HttpEventProcessor

from module_io.module_interface import AbstractModule
from module_utils.signal_handlers import SigIntHandler
from threading import Thread

import rospy
from typing import Dict, Optional, List, Callable
from commons.msg import HttpServerProcessAction, HttpServerProcessGoal, HttpServerProcessResult, \
    HttpServerProcessFeedback
from commons.msg import HttpServerSimpleEvent, MissionsUploadedEvent, HttpExposedResourcesContent, DroneEvent, \
    DroneState

from exposed_resources.http_exposed_resources import ExposedResources

from flask import Request, Response


class ResourceGetters:
    def __init__(self, kss_server_api: KSSServerAPI, exposed_resources: ExposedResources):
        self._kss_server_api = kss_server_api
        self._exposed_resources = exposed_resources

        self._kss_server_api.add_api_path_handler('/api', '/exposed_resources/station_gps_pos',
                                                  SimpleResourceGetter(self.get_station_gps_position))

    def get_station_gps_position(self) -> Dict[str, float]:
        return {'lat': self._exposed_resources.station_lattitude,
                'lng': self._exposed_resources.station_longitude}


class DroneEventProcessor(HttpEventProcessor):
    def process_request(self, incoming_request: Request):
        self._notify_handler(incoming_request.json['event'])
        return Response(status=200)


class DroneStateProcessor(HttpEventProcessor):
    def process_request(self, incoming_request: Request):
        data = incoming_request.json
        self._notify_handler(
            data['lattitude'],
            data['longitude'],
            data['altitude'],
            data['battery_volt'],
            data['battery_temp'])
        return Response(status=200)


class EventNotifiers:
    def __init__(self, kss_server_api: KSSServerAPI,
                 simple_event_notify_handler: Callable[[str], None],
                 drone_event_notify_handler: Callable[[str], None],
                 storage_insertion_handler: Callable[[str], None],
                 drone_state_update_handler: Callable[[float, float, float, float, float], None]):
        self._kss_server_api = kss_server_api

        self._simple_event_processor = SimpleHttpEventProcessor(simple_event_notify_handler)
        self._drone_event_processor = DroneEventProcessor(drone_event_notify_handler)
        self._drone_state_processor = DroneStateProcessor(drone_state_update_handler)

        self._init_simple_event_notifiers()
        self._init_advanced_event_notifiers(storage_insertion_handler)

    def _init_advanced_event_notifiers(self, storage_insertion_handler: Callable[[str], None]):
        self._mission_receiver = MissionReceiver(storage_insertion_handler)
        self._kss_server_api.add_api_path_handler('/api', '/gui_event/upload_mission', self._mission_receiver)

    def _init_simple_event_notifiers(self):
        simple_events = ['start_drone', 'abort_drone', 'on_camera', 'off_camera', 'position_start', 'position_return',
                         'open_roof', 'close_roof', 'land_drone_normally' , 'land_drone_emergency']

        for event in simple_events:
            self._kss_server_api.add_api_path_handler('/api', f'/gui_event/{event}', self._simple_event_processor)

        self._kss_server_api.add_api_path_handler('/com', '/drone_event', self._drone_event_processor)
        self._kss_server_api.add_api_path_handler('/com', '/drone_state', self._drone_state_processor)


class HttpServerModule(AbstractModule):
    def __init__(self, module_name: str):
        AbstractModule.__init__(self, module_name, HttpServerProcessAction)
        self._kss_server: Optional[KSSServerAPI] = None
        self.add_signal_sender('/simple_http_event', HttpServerSimpleEvent)
        self.add_signal_sender('/drone_event', DroneEvent)
        self.add_signal_sender('/drone_state', DroneState)
        self.add_signal_sender('/missions_uploaded_event', MissionsUploadedEvent)
        self.add_signal_receiver('/exposed_resources', HttpExposedResourcesContent, self.receive_exposed_resources)
        self._exposed_resources = ExposedResources(0.0, 0.0, 0.0, 0)

    def _init_http_server(self, host: str, port: int):
        self._kss_server = KSSServerAPI('web_server', port, host)
        self._resource_getters = ResourceGetters(self._kss_server, self._exposed_resources)
        self._event_notifiers = EventNotifiers(self._kss_server, self.notify_simple_event,
                                               self.notify_drone_event,
                                               self.notify_about_new_missions,
                                               self.notify_drone_state)

    def start_module(self, start_args: HttpServerProcessGoal) -> None:
        self._init_http_server(start_args.host_address, start_args.port)

        self._kss_server.run_server()

        self.finish_module(0)

    def send_feedback(self, feedback: HttpServerProcessFeedback) -> None:
        self._server.publish_feedback(feedback)

    def notify_about_new_missions(self, mission_json_body: str) -> None:
        event = MissionsUploadedEvent()
        event.missions_body = mission_json_body
        self.send_signal(MissionsUploadedEvent, event)

    def notify_simple_event(self, event_name: str) -> None:
        event = HttpServerSimpleEvent()
        event.event_name = event_name
        self.send_signal(HttpServerSimpleEvent, event)

    def notify_drone_event(self, event_name: str) -> None:
        event = DroneEvent()
        event.event = event_name
        self.send_signal(DroneEvent, event)

    def notify_drone_state(self, longitude: float, lattitude: float, altitude: float, voltage: float,
                           temperature: float) -> None:
        state = DroneState()
        state.long = longitude
        state.latt = lattitude
        state.alt = altitude
        state.voltage = voltage
        state.temperature = temperature
        self.send_signal(DroneState, state)

    def receive_exposed_resources(self, incoming_resources: HttpExposedResourcesContent) -> None:
        self._exposed_resources.station_longitude = incoming_resources.station_longitude
        self._exposed_resources.station_lattitude = incoming_resources.station_lattitude

    def finish_module(self, exit_code: int) -> None:
        result = HttpServerProcessResult()
        result.exit_code = exit_code
        self._server.set_succeeded(result)


if __name__ == "__main__":
    th = Thread(target=lambda: rospy.init_node('http_server', anonymous=True, disable_signals=True))
    th.run()
    http_server = HttpServerModule('http_server')
    sigint_handler = SigIntHandler()
