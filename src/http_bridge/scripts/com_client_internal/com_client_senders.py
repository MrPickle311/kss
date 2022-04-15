import datetime
from http_io.simple_http_sender import SimpleHttpSender, EmptyMessageHttpSender
from data_models.kss_server import JsonMissionPackage
from data_models.params import DynamicParams


class StartMissionSender(SimpleHttpSender):
    MESSAGE_TYPE = 'kss_event/upload_mission'

    def _create_message(self, mission_package: JsonMissionPackage) -> JsonMissionPackage:
        return mission_package

    def __init__(self, host_address: str, port, station_id: int):
        SimpleHttpSender.__init__(
            self, host_address, port, station_id, self.MESSAGE_TYPE, 'com')


class ParametersSender(SimpleHttpSender):
    MESSAGE_TYPE = 'kss_event/upload_parameters'

    def _create_message(self, parameters_package: DynamicParams) -> DynamicParams:
        return parameters_package

    def __init__(self, host_address: str, port, station_id: int):
        SimpleHttpSender.__init__(
            self, host_address, port, station_id, self.MESSAGE_TYPE, 'com')


class ComEventSender(EmptyMessageHttpSender):
    MESSAGE_TYPE = 'kss_event/'

    def __init__(self, host_address: str, port: int, station_id: int, event_type: str):
        EmptyMessageHttpSender.__init__(
            self, host_address, port, station_id, self.MESSAGE_TYPE + event_type, 'com')
