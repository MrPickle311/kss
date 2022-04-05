from __future__ import annotations
import pymodbus.exceptions
from pymodbus.client.sync import ModbusTcpClient
from typing import List, Callable, Union, Optional
from abc import ABC, abstractmethod
from enum import IntEnum
import multitimer
from pymodbus.register_read_message import ReadHoldingRegistersResponse, ReadInputRegistersResponse
from pymodbus.bit_read_message import ReadCoilsResponse, ReadDiscreteInputsResponse

ModbusUnitTypes = Union[int, bool]
ModbusResponseTypes = Union[
    ReadCoilsResponse, ReadDiscreteInputsResponse, ReadHoldingRegistersResponse, ReadInputRegistersResponse]


class ModbusState(IntEnum):
    CONNECTION_OK = 0
    CONNECTION_ERROR = 1
    RESPONSE_ERROR = 2


StateHandlerType = Callable[[ModbusState], None]


class ModbusAddresses:
    def __init__(self, host_address: str, port: int, server_unit: int):
        self._host_address = host_address
        self._port = port
        self._server_unit = server_unit

    @property
    def host_address(self) -> str:
        return self._host_address

    @property
    def port(self) -> int:
        return self._port

    @property
    def server_unit(self) -> int:
        return self._server_unit


class ModbusDevice:
    def __init__(self, addresses: ModbusAddresses, state_handler: StateHandlerType):
        self._modbus_client = ModbusTcpClient(host=addresses.host_address, port=addresses.port)
        self._server_unit = addresses.server_unit
        self._state_notify_handler = state_handler

    @property
    def modbus_client(self) -> ModbusTcpClient:
        return self._modbus_client

    @property
    def server_unit(self) -> int:
        return self._server_unit

    def call_state_notifier_handler(self, state: ModbusState) -> None:
        self._state_notify_handler(state)

    def connect(self) -> None:
        self._modbus_client.connect()

    def close_connection(self) -> None:
        self._modbus_client.close()

    def _check_response(self, response: ModbusResponseTypes) -> None:
        if response.isError():
            raise pymodbus.exceptions.ConnectionException


class ModbusReader(ModbusDevice):
    def __init__(self, addresses: ModbusAddresses, state_handler: StateHandlerType):
        ModbusDevice.__init__(self, addresses, state_handler)

    def _try_read(self, start_address: int, count: int,
                  read_action: Callable[[int, int], ModbusResponseTypes]) -> Optional[ModbusResponseTypes]:

        try:
            self.connect()
            resp = read_action(start_address, count, unit=self.server_unit)
            self.close_connection()
            self._check_response(resp)
            self.call_state_notifier_handler(ModbusState.CONNECTION_OK)
            return resp
        except:
            self.call_state_notifier_handler(ModbusState.CONNECTION_ERROR)
            return None

    def read_coils(self, start_address: int, count: int) -> List[bool]:
        resp: ReadCoilsResponse = self._try_read(start_address, count, self.modbus_client.read_coils)
        if not isinstance(resp, ReadCoilsResponse):
            return []
        return resp.bits

    def read_discrete_inputs(self, start_address: int, count: int) -> List[bool]:
        resp: ReadDiscreteInputsResponse = self._try_read(start_address, count,
                                                          self.modbus_client.read_discrete_inputs)
        if resp is None:
            return []
        return resp.bits

    def read_holding_registers(self, start_address: int, count: int) -> List[int]:
        resp: ReadHoldingRegistersResponse = self._try_read(start_address, count,
                                                            self.modbus_client.read_holding_registers)
        if resp is None:
            return []
        return resp.registers

    def read_input_registers(self, start_address: int, count: int) -> List[int]:
        resp: ReadInputRegistersResponse = self._try_read(start_address, count,
                                                          self.modbus_client.read_input_registers)
        if resp is None:
            return []
        return resp.registers


class ModbusWriter(ModbusDevice):
    def __init__(self, addresses: ModbusAddresses, state_handler: StateHandlerType):
        ModbusDevice.__init__(self, addresses, state_handler)

    def _try_write(self, start_address: int, content_to_write: List[Union[int, bool]],
                   write_action: Callable[[int, List[any]], any]) \
            -> bool:
        try:
            self.connect()
            resp = write_action(
                start_address, content_to_write, unit=self.server_unit)
            self.close_connection()
            self._check_response(resp)
            self.call_state_notifier_handler(ModbusState.CONNECTION_OK)
            return True
        except pymodbus.exceptions.ConnectionException:
            self.call_state_notifier_handler(ModbusState.CONNECTION_ERROR)
            return False

    def write_coils(self, start_address: int, coils_to_write: List[bool]) -> bool:
        return self._try_write(start_address, coils_to_write, self.modbus_client.write_coils)

    def write_holding_registers(self, start_address: int, registers_to_write: List[int]) -> bool:
        return self._try_write(start_address, registers_to_write, self.modbus_client.write_registers)


class ScanTopic(IntEnum):
    Coils = 0
    DiscreteInputs = 1
    InputRegisters = 2
    HoldingRegisters = 3


class ModbusScanner(ABC):
    def __init__(self, modbus_reader: ModbusReader, period: float, where_to_scan: List[ModbusUnitTypes] = []):
        self._reader = modbus_reader
        self._timer = multitimer.MultiTimer(period, self.timeout_handler, runonstart=False)
        self._desired_value = where_to_scan
        self._scanning_actions = {ScanTopic.Coils: self._reader.read_coils,
                                  ScanTopic.DiscreteInputs: self._reader.read_discrete_inputs,
                                  ScanTopic.InputRegisters: self._reader.read_input_registers,
                                  ScanTopic.HoldingRegisters: self._reader.read_holding_registers}
        self._current_scanning_action: Optional[Callable[[int, int], List[Union[int, bool]]]] = None
        self._current_start_address = 0

    def __del__(self):
        self._timer.join()

    def stop_scanning(self) -> None:
        self._timer.stop()

    def set_scanning_topic(self, what_to_scan: ScanTopic) -> None:
        self._current_scanning_action = self._scanning_actions[what_to_scan]

    def start_scanning(self) -> None:
        self._timer.start()

    def timeout_handler(self) -> None:
        values = self._current_scanning_action(
            self._current_start_address, len(self._desired_value))

        self._service_aquired_values(values)

    @abstractmethod
    def _service_aquired_values(self, values: List[Union[int, bool]]): ...


class DesiredValuesScanner(ModbusScanner):
    def __init__(self, modbus_reader: ModbusReader, period: float):
        ModbusScanner.__init__(self, modbus_reader, period)

    def _apply_new_values(self, desired_value: List[any], start_address: int) -> None:
        self._desired_value = desired_value
        self._current_start_address = start_address

    def set_new_desired_value(self, desired_value: List[any], what_to_scan: ScanTopic, start_address: int) -> None:
        self.set_scanning_topic(what_to_scan)
        self._apply_new_values(desired_value, start_address)

    def _service_aquired_values(self, values: List[Union[int, bool]]):
        if values == self._desired_value:
            self.stop_scanning()
            self.aquired_desired_value()

    @abstractmethod
    def aquired_desired_value(self) -> None:
        pass
