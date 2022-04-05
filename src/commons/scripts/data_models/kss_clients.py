from pydantic import BaseModel
from typing import List
from http_io.simple_http_sender import SimpleMessage


class JsonImageFormat(BaseModel):
    name: str
    image: str


class JsonImagesMessageFormat(BaseModel):
    station_id: int = -1
    region_id: int = -1
    files: List[JsonImageFormat] = [{'': ''}]


class ErrorMessage(SimpleMessage):
    error_id: str


class StateMessage(SimpleMessage):
    state_id: str


class DiagnosticsMessage(SimpleMessage):
    module_id: str
    date: str
