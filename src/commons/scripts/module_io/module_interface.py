from typing import Dict

import actionlib

from .io_entities import *
from threading import Lock, Condition


class IModuleController(ABC):

    @abstractmethod
    def start_module(self, start_args: any) -> None:
        pass

    @abstractmethod
    def process_feedback(self, feedback: any) -> None:
        pass

    @abstractmethod
    def on_process_result_received(self, state, result) -> None:
        pass

    @abstractmethod
    def send_task(self, task_type: type, task: any) -> None:
        pass

    @abstractmethod
    def send_signal(self, signal_type: type, signal: any) -> None:
        pass


class AbstractModuleController(IModuleController, ABC):

    def __init__(self, module_name: str, module_process_type: type):
        self._module_name = module_name
        self._client = actionlib.SimpleActionClient(module_name, module_process_type)
        self._signal_senders: Dict[type, SignalSender] = {}
        self._signal_receivers: Dict[type, SignalReceiver] = {}
        self._task_publishers: Dict[type, TaskPublisher] = {}

    def start_module(self, start_args: any) -> None:
        print(f'Waiting for server...{self._module_name}')
        self._client.wait_for_server()
        self._client.send_goal(start_args, done_cb=self.on_process_result_received, feedback_cb=self.process_feedback)

    def send_task(self, task_type: type, task: any) -> any:
        return self._task_publishers[task_type].post_task(task)

    def send_signal(self, signal_type: type, signal: any) -> None:
        self._signal_senders[signal_type].send_signal(signal)

    def add_signal_sender(self, signal_source_name: str, signal_type: type):
        self._signal_senders[signal_type] = SignalSender(signal_source_name, signal_type)

    def add_task_publisher(self, task_source_name: str, task_type: type):
        self._task_publishers[task_type] = TaskPublisher(task_source_name, task_type)

    def add_signal_receiver(self, signal_source_name: str, signal_type: type, signal_callback: Callable[[any], None]):
        if signal_type not in self._signal_receivers:
            receiver = SignalReceiver(signal_source_name, signal_type)
            receiver.signal_callback = signal_callback
            self._signal_receivers[signal_type] = receiver
        self._signal_receivers[signal_type].add_handler_to_this_signal_source(signal_callback)


class IModule(ABC):

    @abstractmethod
    def start_module(self, start_args: any) -> None:
        pass

    @abstractmethod
    def send_feedback(self, feedback: any) -> None:
        pass

    @abstractmethod
    def finish_module(self, exit_code: int) -> None:
        pass

    @abstractmethod
    def send_signal(self, signal_type: type, signal: any) -> None:
        pass


class AbstractModule(IModule, ABC):

    def __init__(self, module_name: str, module_process_type: type):
        self._module_name = module_name
        self._signal_senders: Dict[type, SignalSender] = {}
        self._signal_receivers: Dict[type, SignalReceiver] = {}
        self._task_processors: Dict[type, TaskProcessor] = {}
        self._server = actionlib.SimpleActionServer(module_name, module_process_type, self.start_module,
                                                    auto_start=False)
        self._lock = Lock()
        self._process_finish_guardian = Condition(self._lock)
        print(f'Starting {self._module_name} server...')
        self._server.start()

    def send_signal(self, signal_type: type, signal: any) -> None:
        self._signal_senders[signal_type].send_signal(signal)

    def add_signal_sender(self, signal_source_name: str, signal_type: type):
        self._signal_senders[signal_type] = SignalSender(signal_source_name, signal_type)

    def add_signal_receiver(self, signal_source_name: str, signal_type: type, signal_callback: Callable[[any], None]):
        if signal_type not in self._signal_receivers:
            receiver = SignalReceiver(signal_source_name, signal_type)
            receiver.signal_callback = signal_callback
            self._signal_receivers[signal_type] = receiver
        self._signal_receivers[signal_type].add_handler_to_this_signal_source(signal_callback)

    def add_task_processor(self, task_source_name: str, task_type: type, task_callback: Callable[[any], any]):
        processor = TaskProcessor(task_source_name, task_type)
        processor.task_callback = task_callback
        self._task_processors[task_type] = processor

    def wait_for_module_finish(self) -> None:
        with self._process_finish_guardian:
            self._process_finish_guardian.wait()

    def notify_about_finish_module(self) -> None:
        with self._process_finish_guardian:
            self._process_finish_guardian.notify()
