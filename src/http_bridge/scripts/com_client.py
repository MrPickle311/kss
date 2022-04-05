#!/usr/bin/env python3
from typing import Optional

import rospy
from commons.msg import HttpClientProcessAction, HttpClientProcessGoal, HttpClientProcessResult, \
    HttpClientProcessFeedback
from commons.srv import DroneSimpleTask, DroneSimpleTaskRequest, DroneSimpleTaskResponse, TryUploadMission, \
    TryUploadMissionRequest, TryUploadMissionResponse
from requests import Response

from com_client_internal.com_client_senders import ComEventSender, StartMissionSender
from module_io.module_interface import AbstractModule
from module_utils.signal_handlers import SigIntHandler
from data_models.kss_server import JsonMissionsMessage


class HttpClientModule(AbstractModule):
    def __init__(self, module_name: str):
        AbstractModule.__init__(self, module_name, HttpClientProcessAction)
        self._start_mission_sender: Optional[StartMissionSender] = None
        self._drone_landing_sender: Optional[ComEventSender] = None
        self.add_task_processor('/drone_task', DroneSimpleTask, self.service_drone_task)
        self.add_task_processor('/start_mission', TryUploadMission, self.service_start_mission)

        self._drone_events = {
            'land_drone_normally': lambda: self._drone_normally_landing_sender.send_empty_message(),
            'land_drone_emergency': lambda: self._drone_emergency_landing_sender.send_empty_message()
        }

    def _init_senders(self, ip_address: str, port: int, station_id: int, images_directory: str):
        self._start_mission_sender = StartMissionSender(ip_address, port, station_id)
        self._drone_normally_landing_sender = ComEventSender(ip_address, port, station_id, 'land_drone_normally')
        self._drone_emergency_landing_sender = ComEventSender(ip_address, port, station_id, 'land_drone_emergency')

    def start_module(self, start_args: HttpClientProcessGoal) -> None:
        self._init_senders(start_args.host_address, start_args.port, start_args.station_id, start_args.images_directory)

        self.wait_for_module_finish()

        self.finish_module(0)

    def send_feedback(self, feedback: HttpClientProcessFeedback) -> None:
        self._server.publish_feedback(feedback)

    def finish_module(self, exit_code: int) -> None:
        result = HttpClientProcessResult()
        result.exit_code = exit_code
        self._server.set_succeeded(result)

    def return_response(self, resp: Response) -> bool:
        return resp.json()['is_ok']

    def service_drone_task(self, task: DroneSimpleTaskRequest) -> DroneSimpleTaskResponse:
        resp: Optional[Response] = self._drone_events[task.task]()
        return DroneSimpleTaskResponse(self.return_response(resp))

    def service_start_mission(self, body: TryUploadMissionRequest) -> TryUploadMissionResponse:
        msg = JsonMissionsMessage.parse_raw(body.waypoints_path)
        resp = self._start_mission_sender.send_simple_message(msg)
        return TryUploadMissionResponse(self.return_response(resp))


def main():
    rospy.init_node('com_client', anonymous=True)
    sigint_handler = SigIntHandler()
    http_client_process = HttpClientModule('com_client')
    rospy.spin()


if __name__ == '__main__':
    main()
