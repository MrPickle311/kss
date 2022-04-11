import datetime
from http_io.simple_http_sender import SimpleHttpSender
from data_models.kss_clients import ErrorMessage, StateMessage, DiagnosticsMessage


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
    def __init__(self, host_address: str, port: int, station_id: int, module_id: str):
        SimpleHttpSender.__init__(self, host_address, port, station_id, 'telemetry', 'api')
        self._module_id = module_id

    def _create_message(self) -> DiagnosticsMessage:
        msg = DiagnosticsMessage()
        return msg
