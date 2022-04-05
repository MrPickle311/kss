import rospy
from abc import ABC, abstractmethod
from typing import Callable, Optional
from .sockets import *
from typing import List


class SignalReceiver:
    def __init__(self, signal_source_name: str, signal_type: type):
        self._signal_callbacks: List[Callable[[any], None]] = []
        self._input_socket = SignalInputSocket(signal_source_name, signal_type, self._signal_handler)

    def add_handler_to_this_signal_source(self, handler: Callable[[any], None]):
        self._signal_callbacks.append(handler)

    def _signal_handler(self, signal: any) -> None:
        for handler in self._signal_callbacks:
            handler(signal)


class SignalSender:
    def __init__(self, signal_source_name: str, signal_type: type):
        self._output_socket = SignalOutputSocket(signal_source_name, signal_type)

    def send_signal(self, signal: any):
        self._prepare_signal(signal)
        self._output_socket.send_signal(signal)

    def _prepare_signal(self, signal):
        pass


class TaskProcessor:
    def __init__(self, task_source_name: str, task_type: type):
        self._task_callback: Optional[Callable[[any], any]] = None
        self._input_socket = TaskInputSocket(task_source_name, task_type, self.task_handler)

    @property
    def task_callback(self) -> Callable[[any], any]:
        return self._task_callback

    @task_callback.setter
    def task_callback(self, new_task_callback: Callable[[any], any]) -> None:
        self._task_callback = new_task_callback

    def task_handler(self, task: any) -> any:
        return self._task_callback(task)


class TaskPublisher:
    def __init__(self, task_source_name: str, task_type: type):
        self._output_socket = TaskOutputSocket(task_source_name, task_type)

    def post_task(self, task: any) -> any:
        self._prepare_task(task)
        response = self._output_socket.execute_task(task)
        self._finish_task(task)
        return response

    def _prepare_task(self, task):
        pass

    def _finish_task(self, task):
        pass
