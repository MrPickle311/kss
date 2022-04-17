from mimetypes import init
from parameters.parameters import ParametersConfigurator
import rospy
from core_internals.collectors import StationStateCollector, StationEventFlags, ErrorFlags
from core_internals.module_controllers import StationModules
from core_internals.station_states import InitializationState, StartState, DroneFlightState, DroneLandingServiceState, \
    DroneAfterLandingServiceState, StationState, FireProtectionException, FireProtectionState, IdleState
from module_utils.signal_handlers import SigIntHandler
from core_internals.flags_checker import FlagsChecker
from core_internals.mission_storage import MissionStorage
from multitimer import MultiTimer

import core_internals.module_controllers
from core_internals.containers import GlobalContainer
# TODO: add background processes class
# TODO: add class for signal bindings
from dependency_injector.wiring import Provide, inject


class StateMachine:
    RESOURCE_UPDATE_PERIOD = 1

    def __init__(self, station_modules: StationModules, station_state_collector: StationStateCollector,
                 station_event_flags: StationEventFlags, error_flags: ErrorFlags, mission_storage: MissionStorage):
        StationState.current_station_data = station_state_collector
        StationState.station_modules = station_modules
        StationState.station_event_flags = station_event_flags
        StationState.error_flags = error_flags
        StationState.mission_storage = mission_storage

        self._init_state = InitializationState()
        self._start_state = StartState()
        self._drone_flight_state = DroneFlightState()
        self._drone_landing_service_state = DroneLandingServiceState()
        self._drone_after_landing_service_state = DroneAfterLandingServiceState()
        self._idle_state = IdleState()
        self._fire_protection_executor = FireProtectionState()

        def update_resources():
            station_modules.http_client_controller.send_exposed_resources()

        self._resource_updater = MultiTimer(
            StateMachine.RESOURCE_UPDATE_PERIOD, update_resources)

    def start_resource_update(self):
        self._resource_updater.start()

    def stop_resource_update(self):
        self._resource_updater.stop()

    def run(self):
        StationState.init_station_states_scope()
        print('Initialized station scope')
        self._init_state.execute()
        print('Initialized module controllers')
        self.start_resource_update()
        print('Initialized exposed resources updater')
        while True:
            try:
                self._idle_state.execute()
                self._start_state.execute()
                self._drone_flight_state.execute()
                self._drone_landing_service_state.execute()
                self._drone_after_landing_service_state.execute()
            except FireProtectionException:
                self.stop_resource_update()
                self._fire_protection_executor.execute()
                break


class Core:
    def __init__(self):
        self._station_state_collector = StationStateCollector()
        self._station_modules = StationModules(
            self._station_state_collector, self.service_mppt_connection_state)
        self._station_event_flags = StationEventFlags(
            'none', 'none', False, '', False)
        self._error_flags = ErrorFlags(
            True, True, True, True, True, True, True, True, True, True, True)
        self._flag_checker = FlagsChecker(
            self._station_state_collector, self._error_flags, 0.4)
        self._mission_storage = MissionStorage()

        # TODO: imrpve the following line with real
        self._station_event_flags.drone_is_near = True

        self._state_machine = StateMachine(self._station_modules, self._station_state_collector,
                                           self._station_event_flags, self._error_flags, self._mission_storage)

    def service_mppt_connection_state(self, state: int):
        print(state)

    def start(self):
        self._flag_checker.start_checking()
        self._state_machine.run()


@inject
def init_container(parametes_updater: ParametersConfigurator = Provide[GlobalContainer.parametes_updater]):
    ...


if __name__ == '__main__':
    rospy.init_node('core')
    container = GlobalContainer()
    container.reset_singletons()
    container.wire(
        modules=[__name__, core_internals.module_controllers.__name__])
    init_container()

    sigint_handler = SigIntHandler()
    core = Core()
    core.start()
    rospy.spin()
