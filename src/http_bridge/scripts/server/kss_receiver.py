from http_io.http_receiver import HttpPathHandler, ServerAPI
from flask import request


class KSSServerAPI(ServerAPI):
    def endpoints_creation(self):
        @self.route('/api/gui_event/<string:gui_event>', methods=['POST'])
        def gui_event(gui_event: str):
            return self._route_handler(request)

        @self.route('/com/drone_event', methods=['POST'])
        def drone_event():
            return self._route_handler(request)

        @self.route('/com/drone_state', methods=['POST'])
        def drone_state():
            return self._route_handler(request)
