from typing import List, Union, Optional, Callable

from modbus.modbus import ModbusReader, ModbusWriter, ModbusScanner, ScanTopic, ModbusState, \
    ModbusAddresses, StateHandlerType
from module_io.module_interface import AbstractModule
from module_utils.signal_handlers import SigIntHandler
from threading import Lock, Condition
from commons.msg import PowerManagementProcessFeedback, PowerManagementProcessAction, PowerManagementProcessGoal, \
    PowerManagementProcessResult
from commons.srv import SwitchModulePowerRequest, SwitchModulePowerResponse, SwitchModulePower
import rospy


class PowerManagementAddresses(ModbusAddresses):
    def __init__(self, host_address: str, port: int, server_unit: int, coils_offset: int):
        ModbusAddresses.__init__(self, host_address, port, server_unit)
        self._coils_offset = coils_offset

    @property
    def coils_offset(self):
        return self._coils_offset


class ConnectionWatcher(ModbusScanner):
    CONNECTION_SCAN_PERIOD = 1
    JUST_RANDOM_VALUE = 1

    def __init__(self, addresses: PowerManagementAddresses, state_handler: StateHandlerType):
        self._addresses = addresses
        self._reader = ModbusReader(addresses, state_handler)
        ModbusScanner.__init__(self, self._reader, ConnectionWatcher.CONNECTION_SCAN_PERIOD,
                               [ConnectionWatcher.JUST_RANDOM_VALUE])
        self.set_scanning_topic(ScanTopic.Coils)

    def start_connection_watching(self):
        self.start_scanning()

    def _service_aquired_values(self, values: List[Union[int, bool]]): ...


class PowerManager:
    def __init__(self, addresses: PowerManagementAddresses, state_handler: StateHandlerType):
        self._coils_offset = addresses.coils_offset
        self._writer = ModbusWriter(addresses, state_handler)

    def turn_on_module(self, module_offset):
        is_ok = False
        while not is_ok:
            is_ok = self._writer.write_coils(self._coils_offset + module_offset, [True])

    def turn_off_module(self, module_offset):
        is_ok = False
        while not is_ok:
            is_ok = self._writer.write_coils(self._coils_offset + module_offset, [False])


class PowerManagementModule(AbstractModule):

    def __init__(self, module_name: str):
        AbstractModule.__init__(self, module_name, PowerManagementProcessAction)
        self._connection_watcher: Optional[ConnectionWatcher] = None
        self._power_manager: Optional[PowerManager] = None
        self._lock = Lock()
        self._process_finish_guardian = Condition(self._lock)

    def _get_notify_about_connection_state_closure(self) -> Callable[[ModbusState], None]:
        def closure(connection: ModbusState):
            self.send_feedback(connection)

        return closure

    def _init_connection_watcher(self, addresses: PowerManagementAddresses):
        self._connection_watcher = ConnectionWatcher(addresses, self._get_notify_about_connection_state_closure())

    def _init_power_manager(self, addresses: PowerManagementAddresses):
        def switch_power_callback(switch_request: SwitchModulePowerRequest) -> SwitchModulePowerResponse:
            if switch_request.turn_on:
                self._power_manager.turn_on_module(switch_request.module_id)
            else:
                self._power_manager.turn_off_module(switch_request.module_id)

            return SwitchModulePowerResponse()

        self.add_task_processor('/power_switching', SwitchModulePower, switch_power_callback)

        self._power_manager = PowerManager(addresses, self._get_notify_about_connection_state_closure())

    def wait_for_process_finish(self) -> None:
        with self._process_finish_guardian:
            self._process_finish_guardian.wait()

    def finish_process(self) -> None:
        with self._process_finish_guardian:
            self._process_finish_guardian.notify()

    def start_module(self, start_arguments: PowerManagementProcessGoal) -> None:
        addresses = PowerManagementAddresses(start_arguments.host_address, start_arguments.port,
                                             start_arguments.server_address, start_arguments.coils_offset)

        self._init_power_manager(addresses)
        self._init_connection_watcher(addresses)

        self._connection_watcher.start_connection_watching()

        self.wait_for_process_finish()

        self.finish_module(0)

    def send_feedback(self, connection_state: ModbusState) -> None:
        feedback = PowerManagementProcessFeedback()

        if connection_state == ModbusState.CONNECTION_OK:
            feedback.is_plc_connected = True
        else:
            feedback.is_plc_connected = False

        self._server.publish_feedback(feedback)

    def finish_module(self, exit_code: int) -> None:
        result = PowerManagementProcessResult()
        result.exit_code = exit_code
        self._server.set_succeeded(result)


if __name__ == "__main__":
    rospy.init_node('power_manager_interface', anonymous=True)
    sigint_handler = SigIntHandler()
    automation_process = PowerManagementModule('power_manager_interface')
    rospy.spin()
