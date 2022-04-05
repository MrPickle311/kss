from flask import Request, jsonify, Response
from pydantic import BaseModel
from .http_receiver import HttpPathHandler
from typing import Callable
from flask import Request
from abc import abstractmethod, ABC


class StationPosition(BaseModel):
    lat: float
    lng: float


class SimpleResourceGetter(HttpPathHandler):
    def __init__(self, get_handler: Callable[[], dict]):
        self._get_handler = get_handler

    def process_request(self, incoming_request: Request):
        return jsonify(self._get_handler())


class HttpEventProcessor(HttpPathHandler, ABC):

    def __init__(self, notify_handler: Callable[[any], None]):
        self._notify_handler = notify_handler


class SimpleHttpEventProcessor(HttpEventProcessor):
    def process_request(self, incoming_request: Request):
        self._notify_handler(incoming_request.path)
        return Response(status=200)
