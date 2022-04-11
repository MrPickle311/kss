import datetime
from http_io.simple_http_sender import SimpleHttpSender
from data_models.kss_clients import ErrorMessage, StateMessage, DiagnosticsMessage, PosistionMessage, DroneProps, \
    KSSPropsModel


class ErrorSender(SimpleHttpSender):
    MESSAGE_TYPE = 'error'

    def _create_message(self, error_id: str) -> ErrorMessage:
        msg = ErrorMessage()
        msg.station_id = self.station_id
        msg.error_id = error_id
        return msg

    def __init__(self, host_address: str, port, station_id: int):
        SimpleHttpSender.__init__(self, host_address, port, station_id, self.MESSAGE_TYPE, 'api')


class StateSender(SimpleHttpSender):
    MESSAGE_TYPE = 'station_state'

    def _create_message(self, state_id: str) -> StateMessage:
        print(f'Got a state from gui server {state_id}')
        return StateMessage(station_id=self.station_id, state_id=state_id)

    def __init__(self, host_address: str, port, station_id: int):
        SimpleHttpSender.__init__(self, host_address, port, station_id, self.MESSAGE_TYPE, 'api')


class DiagnosticsSender(SimpleHttpSender):
    def __init__(self, host_address: str, port: int, station_id: int):
        SimpleHttpSender.__init__(self, host_address, port, station_id, 'telemetry', 'api')

    def _create_message(self, drone_longitude: float, drone_lattitude: float, drone_battery_voltage: float,
                        drone_battery_temperature: float, station_accumulators_temperature: float,
                        station_accumulators_level: float) -> DiagnosticsMessage:
        return DiagnosticsMessage(
            droneProps=DroneProps(
                volt=drone_battery_voltage,
                temp=drone_battery_temperature,
                pos=PosistionMessage(
                    lat=drone_lattitude,
                    lng=drone_longitude
                )
            ),
            KSSProps=KSSPropsModel(
                temp=station_accumulators_temperature,
                battery_level=station_accumulators_level
            )
        )
