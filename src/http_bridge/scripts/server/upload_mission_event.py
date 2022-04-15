from typing import Callable
import json
from flask import Request, Response
from flask import jsonify
from http_io.simple_server_entities import HttpPathHandler


class JsonReceiver(HttpPathHandler):

    def __init__(self, storage_insertion_handler: Callable[[str], None]):
        HttpPathHandler.__init__(self)
        self._storage_insertion_handler = storage_insertion_handler

    def process_request(self, incoming_request: Request):
        str_json = json.dumps(incoming_request.json)
        self._storage_insertion_handler(str_json)
        return Response(status=200)
