from __future__ import annotations

from abc import ABC, abstractmethod
from enum import IntEnum
from threading import Condition, Lock
from typing import Callable, Optional, Dict
from dataclasses import dataclass

import rospy

import commons.msg
import commons.srv
from modbus.modbus import ModbusReader, ModbusWriter, DesiredValuesScanner, ScanTopic, StateHandlerType, ModbusState, \
    ModbusAddresses
from module_io.module_interface import AbstractModule
from module_utils.signal_handlers import SigIntHandler
from commons.msg import AutomationProcessAction, AutomationProcessGoal
from enums.automation import AutomationState


def EmptyFunction(): return ...


@dataclass
class StationAutomationCurrentState:
    current_state: int


class AutomationAddresses(ModbusAddresses):
    def __init__(self, host_address: str, port: int, server_unit: int, state_register: int):
        ModbusAddresses.__init__(self, host_address, port, server_unit)
        self._state_register = state_register

    @property
    def state_register(self) -> int:
        return self._state_register


class AutomationScanner(DesiredValuesScanner):
    def __init__(self, modbus_reader: ModbusReader, period: float, automation_addresses: AutomationAddresses):
        DesiredValuesScanner.__init__(self, modbus_reader, period)
        self._automation_addresses = automation_addresses
        self._state_presence_notify_handler: Callable[[], None] = EmptyFunction

    @property
    def state_presence_notify_handler(self) -> Callable[[], None]:
        return self._state_presence_notify_handler

    @state_presence_notify_handler.setter
    def state_presence_notify_handler(self, new_state_presence_notify_handler: Callable[[], None]) -> None:
        self._state_presence_notify_handler = new_state_presence_notify_handler

    def aquired_desired_value(self):
        self._state_presence_notify_handler()

    def wait_for_state(self, state: int):
        self.set_new_desired_value(
            [state], ScanTopic.HoldingRegisters, self._automation_addresses.state_register)
        self.start_scanning()


class AutomationWriter(ModbusWriter):
    def __init__(self, automation_addresses: AutomationAddresses, state_handler: StateHandlerType):
        self._automation_addresses = automation_addresses
        ModbusWriter.__init__(self, automation_addresses, state_handler)

    def try_inject_state(self, state: int) -> bool:
        return self.write_holding_registers(self._automation_addresses.state_register, [state])


class IAutomationNode(ABC):

    @abstractmethod
    def set_next(self, handler: IAutomationNode) -> IAutomationNode:
        pass

    @abstractmethod
    def handle(self) -> None:
        pass


class AutomationNode(IAutomationNode):
    _next_node: IAutomationNode = None
    station_state: StationAutomationCurrentState = None

    def set_next(self, new_node: IAutomationNode) -> IAutomationNode:
        self._next_node = new_node
        return self._next_node

    @abstractmethod
    def handle(self) -> None:
        if self._next_node:
            self._next_node.handle()

    def update_state(self, new_state: int):
        self.station_state.current_state = new_state


class StateScanner(AutomationNode):

    def __init__(self, scanner: AutomationScanner, desired_state: int):
        self._scanner = scanner
        self._lock = Lock()
        self._condition_variable = Condition(self._lock)
        self._desired_state = desired_state

    def replace_state_presence_notify_handler(self):
        def notify_about_state():
            with self._condition_variable:
                self._condition_variable.notify()

        self._scanner.state_presence_notify_handler = notify_about_state

    def wait_for_state_presence(self):
        self._scanner.wait_for_state(self._desired_state)
        with self._condition_variable:
            self._condition_variable.wait()

    def notify_about_updated_state(self):
        self._scanner._reader.call_state_notifier_handler(ModbusState.CONNECTION_OK)

    def handle(self) -> None:
        self.replace_state_presence_notify_handler()
        self.wait_for_state_presence()
        self.update_state(self._desired_state)
        self.notify_about_updated_state()
        AutomationNode.handle(self)


class StateInjector(AutomationNode):
    current_result = False

    def __init__(self, writer: AutomationWriter, injected_state: int):
        self._writer = writer
        self._injected_state = injected_state
        self._lock = Lock()
        self._condition_variable = Condition(self._lock)

    def wait(self):
        with self._condition_variable:
            self._condition_variable.wait()

    def notify(self):
        with self._condition_variable:
            self._condition_variable.notify()

    def wait_for_confirmation(self) -> None:
        self.wait()

    def wait_for_result(self):
        self.wait()

    @staticmethod
    def clear_current_result_flag():
        StateInjector.current_result = False

    def try_inject_state(self) -> None:
        self.clear_current_result_flag()
        while StateInjector.current_result is False:
            print('Waiting for confirmation')
            self.wait_for_confirmation()
            StateInjector.current_result = self._writer.try_inject_state(self._injected_state)
            self.confirm_result_presence()
        self.update_state(self._injected_state)

    def handle(self) -> None:
        self.try_inject_state()
        AutomationNode.handle(self)

    def try_confirm_state_apply(self) -> bool:
        self.notify()
        self.wait_for_result()
        return self.current_result

    def confirm_result_presence(self):
        self.notify()


class IAutomationAlgorithm(ABC):

    @abstractmethod
    def run(self):
        pass


class AutomationAlgorithm(IAutomationAlgorithm):

    def __init__(self, scanner: AutomationScanner, writer: AutomationWriter):
        self._scanner = scanner
        self._writer = writer
        self._injectors_map: Dict[int, StateInjector] = {}

        # Responsibility chain composing
        self._initialized_state_scanner = StateScanner(
            self._scanner, AutomationState.INITIALIZED)
        self._roof_opening_state_injector = StateInjector(
            self._writer, AutomationState.ROOF_OPENING)
        self._roof_opened_state_scanner = StateScanner(
            self._scanner, AutomationState.ROOF_OPENED)
        self._moving_positioners_state_injector = StateInjector(
            self._writer, AutomationState.MOVING_POSITIONERS_APART)
        self._moved_positioners_apart_state_scanner = StateScanner(self._scanner,
                                                                   AutomationState.MOVED_POSITIONERS_APART)
        self._sliding_off_positioners_state_injector = StateInjector(self._writer,
                                                                     AutomationState.SLIDING_OFF_POSITIONERS)
        self._positioners_slided_off_state_scanner = StateScanner(
            self._scanner, AutomationState.POSITIONERS_SLIDED_OFF)
        self._closing_roof_state_injector = StateInjector(
            self._writer, AutomationState.CLOSING_ROOF)
        self._roof_closed_state_scanner = StateScanner(
            self._scanner, AutomationState.ROOF_CLOSED)

        self._head_node = self._initialized_state_scanner

        self._initialized_state_scanner. \
            set_next(self._roof_opening_state_injector). \
            set_next(self._roof_opened_state_scanner). \
            set_next(self._moving_positioners_state_injector). \
            set_next(self._moved_positioners_apart_state_scanner). \
            set_next(self._sliding_off_positioners_state_injector). \
            set_next(self._positioners_slided_off_state_scanner). \
            set_next(self._closing_roof_state_injector). \
            set_next(self._roof_closed_state_scanner)

        # automation is initialized only once
        self._roof_closed_state_scanner.set_next(self._roof_opening_state_injector)

        self.map_injectors()

    def map_injectors(self):
        self._injectors_map[AutomationState.ROOF_OPENING] = self._roof_opening_state_injector
        self._injectors_map[AutomationState.MOVING_POSITIONERS_APART] = self._moving_positioners_state_injector
        self._injectors_map[AutomationState.SLIDING_OFF_POSITIONERS] = self._sliding_off_positioners_state_injector
        self._injectors_map[AutomationState.CLOSING_ROOF] = self._closing_roof_state_injector

    def try_confirm_state_apply(self, state_id: int) -> bool:
        return self._injectors_map[state_id].try_confirm_state_apply()

    def run(self) -> None:
        self._head_node.handle()


class AutomationInterface:
    SCAN_PERIOD = 0.2

    def __init__(self, automation_addressed: AutomationAddresses, state_handler: StateHandlerType):
        self._automation_addresses = automation_addressed
        self._reader = ModbusReader(self._automation_addresses, state_handler=state_handler)
        self.current_station_state = StationAutomationCurrentState(AutomationState.NOT_INITIALIZED)
        AutomationNode.station_state = self.current_station_state

        self._scanner = AutomationScanner(
            self._reader, AutomationInterface.SCAN_PERIOD, self._automation_addresses)
        self._writer = AutomationWriter(
            self._automation_addresses, state_handler)

        self._algorithm = AutomationAlgorithm(self._scanner, self._writer)

    def confirm_state_apply(self, state_id: int) -> bool:
        return self._algorithm.try_confirm_state_apply(state_id)

    def run(self):
        self._algorithm.run()


class AutomationModule(AbstractModule):
    STATE_FEEDBACK_MAP = {ModbusState.CONNECTION_OK: True, ModbusState.CONNECTION_ERROR: False}

    def __init__(self, module_name: str):
        AbstractModule.__init__(self, module_name, AutomationProcessAction)
        self._automation_interface: Optional[AutomationInterface] = None
        self._addresses: Optional[AutomationAddresses] = None
        self.init_automation_state_task_processor()

    def try_apply_state(self, state: commons.srv.AutomationStatesRequest) -> commons.srv.AutomationStatesResponse:
        result = self._automation_interface.confirm_state_apply(state.state_to_apply)
        return commons.srv.AutomationStatesResponse(result)

    def init_automation_state_task_processor(self):
        self.add_task_processor('/state_apply', commons.srv.AutomationStates, self.try_apply_state)

    def _initialize_automation_interface(self):
        def handle_error_and_send_feedback(error_code: ModbusState):
            self.send_feedback(AutomationModule.STATE_FEEDBACK_MAP[error_code],
                               self._automation_interface.current_station_state.current_state)

        self._automation_interface = AutomationInterface(self._addresses, handle_error_and_send_feedback)

    def _run_automation_interface(self):
        self._automation_interface.run()

    def start_module(self, start_arguments: AutomationProcessGoal) -> None:
        self._addresses = AutomationAddresses(start_arguments.host_address, start_arguments.port,
                                              start_arguments.server_address, start_arguments.state_register)
        self._initialize_automation_interface()

        print('Running automation interface ')
        self._run_automation_interface()

        self.finish_module(0)

    def send_feedback(self, is_connected: bool, current_state: int) -> None:
        feedback = commons.msg.AutomationProcessFeedback()
        feedback.is_plc_connected = is_connected
        feedback.current_state = current_state
        self._server.publish_feedback(feedback)

    def finish_module(self, exit_code: int) -> None:
        result = AutomationProcessResult()
        result.exit_code = exit_code
        self._server.set_succeeded(result)


if __name__ == "__main__":
    rospy.init_node('automation_interface', anonymous=True)
    sigint_handler = SigIntHandler()
    automation_process = AutomationModule('automation_interface')
    rospy.spin()
