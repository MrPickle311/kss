from pyftpdlib.authorizers import DummyAuthorizer
from pyftpdlib.handlers import FTPHandler
from pyftpdlib.servers import FTPServer

import rospy
from module_io.module_interface import AbstractModule
from module_utils.signal_handlers import SigIntHandler
from typing import List, Union, Callable, Optional

from commons.msg import FTPServerProcessGoal, FTPServerProcessFeedback, FTPServerProcessResult, FTPServerProcessAction

from commons.msg import ReceivedImagesFromFtpServerEvent


class FinishHandler(FTPHandler):
    on_disconnect_callback: Callable[[], None] = None

    def on_disconnect(self):
        self.on_disconnect_callback()


class FtpServerModule(AbstractModule):

    def __init__(self, module_name: str):
        AbstractModule.__init__(self, module_name, FTPServerProcessAction)
        self.authorizer = DummyAuthorizer()
        self.ftp_handler = FinishHandler
        FinishHandler.on_disconnect_callback = self.notify_about_sending_finish
        self.ftp_server: Optional[FTPServer] = None
        self.add_signal_sender('/received_images_from_ftp_event', ReceivedImagesFromFtpServerEvent)

    def start_module(self, start_args: FTPServerProcessGoal) -> None:
        self._init_ftp_server(start_args)

        self.run()

        self.finish_module(0)

    def notify_about_sending_finish(self):
        event = ReceivedImagesFromFtpServerEvent()
        self.send_signal(ReceivedImagesFromFtpServerEvent, event)

    def _init_ftp_server(self, start_args: FTPServerProcessGoal):
        self.authorizer.add_user(start_args.user, start_args.ftp_password, start_args.files_directory, perm='elradfmw')
        self.ftp_handler.authorizer = self.authorizer
        self.ftp_handler.banner = "pyftpdlib based ftpd ready."
        address = (start_args.host_address, start_args.port)
        self.ftp_server = FTPServer(address, self.ftp_handler)

    def run(self):
        self.ftp_server.serve_forever()

    def send_feedback(self, transaction: FTPServerProcessFeedback) -> None:
        self._server.publish_feedback(transaction)

    def finish_module(self, exit_code: int) -> None:
        result = FTPServerProcessResult()
        result.exit_code = exit_code
        self._server.set_succeeded(result)


def main():
    rospy.init_node('ftp_server', anonymous=True)
    sigint_handler = SigIntHandler()
    http_client_process = FtpServerModule('ftp_server')
    rospy.spin()


if __name__ == '__main__':
    main()
