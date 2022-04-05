import requests

from .http_sender import HttpSender
from pydantic import BaseModel
from abc import abstractmethod
from requests import Response


class SimpleMessage(BaseModel):
    station_id: int


class EmptyModel(BaseModel):
    None


# TODO: aggregate host_address . port , station_id into 1 class
# TODO: change message_type -> message_path


class EmptyMessageHttpSender(HttpSender):
    def __init__(self, host_address: str, port: int, station_id: int, message_type: str, domain: str):
        HttpSender.__init__(self, host_address, port, message_type, station_id, domain)

    def send_empty_message(self) -> Response:
        return self.send_message(EmptyModel())


class SimpleHttpSender(HttpSender):
    def __init__(self, host_address: str, port: int, station_id: int, message_type: str, domain: str):
        HttpSender.__init__(self, host_address, port, message_type, station_id, domain)

    @abstractmethod
    def _create_message(self, *args) -> any: ...

    def send_simple_message(self, *args) -> Response:
        data = self._create_message(*args)
        return self.send_message(data)
