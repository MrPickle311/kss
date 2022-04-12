from serial_io.pipe_serial import SerialDataPipe, ISerialReceiver, SerialState, SerialSocket, SerialDevice
from enum import Enum
from typing import Callable, Optional
from commons.msg import TrackerProcessAction, TrackerProcessFeedback, TrackerProcessGoal
from commons.msg import TrackerProcessResult
from module_io.module_interface import AbstractModule
import commons.msg
from commons.msg import TrackerConnectionSignal
from module_utils.signal_handlers import SigIntHandler
from enums.tracker import TrackerState, TrackerMessage
import rospy

tracker_modes = ["IDLE", "CC", "CV", "FLOAT", "STARTING"]


class ParserAlgorithm:

    def __init__(self, extraction_ready_handler: Callable[[TrackerProcessFeedback], None]):
        self._state = TrackerState.STATE_START
        self._target_length = 0
        self._data = []
        self._extraction_ready_handler = extraction_ready_handler

    def _begin_parsing(self):
        self._state = TrackerState.STATE_DATA
        self._data = []
        self._target_length = 255

    def _is_start_state(self) -> bool:
        return self._state == TrackerState.STATE_START

    def _is_receiving_data_state(self) -> bool:
        return self._state == TrackerState.STATE_DATA

    @staticmethod
    def _start_character_detected(byte: int) -> bool:
        return byte == 0xaa

    def _received_enough_data_to_determine_target_length(self) -> bool:
        return len(self._data) == 5

    def _determine_packet_length(self) -> None:
        self._target_length = 6 + self._data[4]

    def _received_whole_packet(self) -> bool:
        return len(self._data) == self._target_length

    def _is_mppt_device(self) -> bool:
        return self._data[2] == 3

    def _is_type_0_packet(self) -> bool:
        return self._data[3] == 0

    def _extract_integer_from_bytes(self, start: int, end: int) -> int:
        return int.from_bytes(self._data[start:end], byteorder='little')

    def _extract_tracker_message(self) -> None:
        extracted_package = TrackerProcessFeedback()

        extracted_package.tracker_mode = str(self._extract_integer_from_bytes(7, 9))
        extracted_package.solar_panels_voltage = self._extract_integer_from_bytes(9, 11) / 10.0
        extracted_package.battery_voltage = self._extract_integer_from_bytes(11, 13) / 10.0
        extracted_package.charging_current = self._extract_integer_from_bytes(13, 15) / 10.0
        extracted_package.load_voltage = self._extract_integer_from_bytes(17, 19) / 10.0
        extracted_package.load_current = self._extract_integer_from_bytes(19, 21) / 10.0
        extracted_package.charging_power = self._extract_integer_from_bytes(21, 23)
        extracted_package.load_power = self._extract_integer_from_bytes(23, 25)
        extracted_package.battery_temperature = self._data[25]
        extracted_package.internal_temperature = self._data[27]
        extracted_package.battery_level = self._data[29]

        self._extraction_ready_handler(extracted_package)

    def _parse_ready_package(self):
        self._state = TrackerState.STATE_START

        if self._is_mppt_device() and self._is_type_0_packet():
            self._extract_tracker_message()

    def _parse_byte(self, byte: int):
        if self._is_start_state():
            if self._start_character_detected(byte):
                self._begin_parsing()

        elif self._is_receiving_data_state():
            self._data.append(byte)

            if self._received_enough_data_to_determine_target_length():
                self._determine_packet_length()

            if self._received_whole_packet():
                self._parse_ready_package()

    def parse(self, bytes_package: bytes):
        for byte in bytes_package:
            self._parse_byte(byte)


class TrackerParser(ISerialReceiver):
    PACKAGE_SIZE = 100  # bytes
    STATE_FETCH_PERIOD = 0.5  # sec

    def __init__(self, socket: SerialSocket, state_handler: Callable[[int], None],
                 extraction_ready_handler: Callable[[TrackerProcessFeedback], None]):
        self._pipe = SerialDataPipe(socket, TrackerParser.PACKAGE_SIZE, TrackerParser.STATE_FETCH_PERIOD, self)
        self._state_handler = state_handler
        self._algorithm = ParserAlgorithm(extraction_ready_handler)

    def run(self):
        self._pipe.start_flushing()

    def flush_handler(self, bytes_package: bytes) -> None:
        self._algorithm.parse(bytes_package)

    def state_handler(self, serial_state: SerialState) -> None:
        self._state_handler(serial_state)

    def data_absence_handler(self):
        self._pipe.write(bytes(TrackerMessage.REQUEST_MESSAGE))


class TrackerModule(AbstractModule):

    def __init__(self, module_name: str):
        AbstractModule.__init__(self, module_name, TrackerProcessAction)
        self._socket: Optional[SerialSocket] = None
        self._tracker_parser: Optional[TrackerParser] = None
        self._init_connection_state_notifier()

    def _init_socket(self, device: SerialDevice) -> None:
        self._socket = SerialSocket(device)

    def _init_tracker_parser(self):
        def extracted_feedback_handler(feedback: TrackerProcessFeedback) -> None:
            self.send_feedback(feedback)

        self._tracker_parser = TrackerParser(self._socket, self._notify_about_connection_state,
                                             extracted_feedback_handler)

    def _init_connection_state_notifier(self):
        self.add_signal_sender('/tracker_connection_state', TrackerConnectionSignal)

    @staticmethod
    def _get_serial_device(start_args: TrackerProcessGoal) -> SerialDevice:
        return SerialDevice(start_args.baud_rate, start_args.serial_port, start_args.read_timeout)

    def start_module(self, start_args: TrackerProcessGoal) -> None:
        device = TrackerModule._get_serial_device(start_args)
        self._init_socket(device)
        self._init_tracker_parser()

        self._tracker_parser.run()

        self.finish_module(0)

    def send_feedback(self, feedback: TrackerProcessFeedback) -> None:
        self._server.publish_feedback(feedback)

    def finish_module(self, exit_code: int) -> None:
        result = commons.msg.TrackerProcessResult
        result.exit_code = exit_code
        self._server.set_succeeded(result)

    def _notify_about_connection_state(self, connection: SerialState):
        print(f'Connection state: {connection}')
        sig = TrackerConnectionSignal()
        sig.signal_descriptor = connection
        self.send_signal(TrackerConnectionSignal, sig)


def main():
    rospy.init_node('tracker_interface', anonymous=True)
    sigint_handler = SigIntHandler()
    automation_process = TrackerModule('tracker_interface')
    rospy.spin()


if __name__ == "__main__":
    main()
