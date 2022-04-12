#!/usr/bin/env python3
from gui_client.image_sender import ImageSender
from gui_client.kss_client_senders import StateSender, ErrorSender, DiagnosticsSender
from typing import List, Dict, Optional
from module_io.module_interface import AbstractModule
from module_utils.signal_handlers import SigIntHandler
from commons.msg import StationState, SendImagesToServerEvent, StationError
from commons.msg import HttpClientProcessAction, HttpClientProcessGoal, HttpClientProcessResult, \
    HttpClientProcessFeedback, HttpExposedResourcesContent
import rospy


class HttpClientModule(AbstractModule):
    def __init__(self, module_name: str):
        AbstractModule.__init__(self, module_name, HttpClientProcessAction)
        self._err_sender: Optional[ErrorSender] = None
        self._image_sender: Optional[ImageSender] = None
        self._state_sender: Optional[StateSender] = None
        self.add_signal_receiver('/send_station_state', StationState, self.send_station_status)
        self.add_signal_receiver('/send_station_error', StationError, self.send_error)
        self.add_signal_receiver('/send_images', SendImagesToServerEvent, self.send_images)
        self.add_signal_receiver('/exposed_resources', HttpExposedResourcesContent, self.send_exposed_resources)

    def _init_senders(self, ip_address: str, port: int, station_id: int, images_directory: str):
        self._err_sender = ErrorSender(ip_address, port, station_id)
        self._image_sender = ImageSender(ip_address, port, station_id, images_directory)
        self._state_sender = StateSender(ip_address, port, station_id)
        self._diagnostics_sender = DiagnosticsSender(ip_address, port, station_id)

    def start_module(self, start_args: HttpClientProcessGoal) -> None:
        self._init_senders(start_args.host_address, start_args.port, start_args.station_id, start_args.images_directory)

        # TODO: sometimes _image_sender is not initialized
        self.wait_for_module_finish()

        self.finish_module(0)

    def send_feedback(self, feedback: HttpClientProcessFeedback) -> None:
        self._server.publish_feedback(feedback)

    def finish_module(self, exit_code: int) -> None:
        result = HttpClientProcessResult()
        result.exit_code = exit_code
        self._server.set_succeeded(result)

    def send_station_status(self, status: StationState) -> None:
        print('Sending state to server...')
        self._state_sender.send_simple_message(status.station_state)

    def send_error(self, error: StationError) -> None:
        self._err_sender.send_simple_message(error.station_error)

    def send_images(self, images_msg: SendImagesToServerEvent) -> None:
        self._image_sender.send_images(images_msg.region_id)

    def send_exposed_resources(self, incoming_resources: HttpExposedResourcesContent) -> None:
        self._diagnostics_sender.send_simple_message(
            incoming_resources.drone_longitude,
            incoming_resources.drone_lattitude,
            incoming_resources.drone_battery_voltage,
            incoming_resources.drone_battery_temperature,
            incoming_resources.station_accumulators_temperature,
            incoming_resources.station_accumulators_level
        )


def main():
    rospy.init_node('http_client', anonymous=True)
    sigint_handler = SigIntHandler()
    http_client_process = HttpClientModule('http_client')
    rospy.spin()


if __name__ == '__main__':
    main()
