import time
from abc import ABC, abstractmethod
from enum import IntEnum
from threading import Lock
from typing import Optional

import serial
from multitimer import MultiTimer


class ISerialSocket(ABC):

    @abstractmethod
    def read_data(self, count: int) -> bytes: ...

    @abstractmethod
    def write_data(self, data: bytes) -> bool: ...

    @abstractmethod
    def open(self) -> bool: ...

    @abstractmethod
    def close(self) -> bool: ...


class SerialDevice:
    def __init__(self, baud_rate: int, port: str, timeout: Optional[int] = None, byte_size: int = serial.EIGHTBITS,
                 parity: int = serial.PARITY_NONE,
                 stop_bits: int = serial.STOPBITS_ONE):
        self._baud_rate = baud_rate
        self._port = port
        self._timeout = timeout
        self._byte_size = byte_size
        self._parity = parity
        self._stop_bits = stop_bits

    @property
    def baud_rate(self):
        return self._baud_rate

    @property
    def port(self):
        return self._port

    @property
    def timeout(self):
        return self._timeout

    @property
    def byte_size(self):
        return self._byte_size

    @property
    def parity(self):
        return self._parity

    @property
    def stop_bits(self):
        return self._stop_bits


class SerialSocket(ISerialSocket):
    def __init__(self, device: SerialDevice):
        self._socket = serial.Serial(port=device.port, bytesize=device.byte_size, parity=device.parity,
                                     stopbits=device.stop_bits, timeout=device.timeout, baudrate=device.baud_rate)

    def read_data(self, count: int) -> Optional[bytes]:
        return self._socket.read(count)

    def write_data(self, data: bytes) -> bool:
        bytes_written = self._socket.write(data)
        if bytes_written != len(data):
            return False
        return True

    def open(self) -> bool:
        self._socket.open()
        return self._socket.isOpen()

    def avalaible_bytes_count(self):
        return self._socket.inWaiting()

    def close(self) -> bool:
        self._socket.close()
        return not self._socket.isOpen()


class SerialState(IntEnum):
    CONNECTED = 0
    DISCONNECTED = 1
    EMPTY_INPUT = 2
    RECEIVING_DATA = 3


class ISerialReceiver(ABC):

    @abstractmethod
    def flush_handler(self, bytes_package: bytes) -> None: ...

    @abstractmethod
    def state_handler(self, serial_state: SerialState) -> None: ...

    @abstractmethod
    def data_absence_handler(self): ...


class SerialDataPipe:
    def __init__(self, serial_socket: SerialSocket, package_size: int, state_notify_period: float,
                 receiver: ISerialReceiver, data_absence_timeout: int = 5):
        self._is_flushing = False
        self._socket = serial_socket
        self._package_size = package_size
        self._lock = Lock()
        self._receiver = receiver
        self._pipe_state = SerialState.DISCONNECTED
        self._data_absence_timeout = data_absence_timeout

        self._current_data_absence_timeout = 0

        self._init_timer(state_notify_period)
        self._start_timer()

    def _init_timer(self, state_notify_period):

        def state_handler():
            self._receiver.state_handler(self._pipe_state)

        self._timer = MultiTimer(state_notify_period, state_handler, runonstart=False)

    def _start_timer(self):
        self._timer.start()

    def _receive_data(self):
        self._pipe_state = SerialState.RECEIVING_DATA
        self._receiver.flush_handler(self._socket.read_data(self._package_size))

    def _are_byes_avalaible_in_serial_port(self) -> bool:
        return self._socket.avalaible_bytes_count() != 0

    def _data_absence_timeout_occurred(self) -> bool:
        return time.time() - self._current_data_absence_timeout > self._data_absence_timeout

    def _reset_data_absence_timeout(self):
        self._receiver.data_absence_handler()
        self._current_data_absence_timeout = time.time()

    def _try_flush(self):
        try:
            while self._are_byes_avalaible_in_serial_port():
                self._receive_data()

            if self._data_absence_timeout_occurred():
                self._reset_data_absence_timeout()

        except IOError:
            self._socket.close()
            self._try_reopen_serial_port()

    def start_flushing(self):
        with self._lock:
            self._is_flushing = True

        while self._is_flushing:
            self._try_flush()

    def stop_flushing(self):
        with self._lock:
            self._is_flushing = False

    def _try_open_serial_port(self):
        try:
            if self._socket.open():
                self._pipe_state = SerialState.CONNECTED
            else:
                self._socket.close()
        except serial.serialutil.SerialException:
            self._socket.close()

    def _is_pipe_disconnected(self):
        return not self._pipe_state == SerialState.DISCONNECTED

    def _try_reopen_serial_port(self):
        self._pipe_state = SerialState.DISCONNECTED

        while self._is_pipe_disconnected():
            self._try_open_serial_port()

        self._pipe_state = SerialState.CONNECTED

    def write(self, data: bytes) -> None:
        self._socket.write_data(data)
