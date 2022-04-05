import rospy
from typing import List, Dict, Callable


class SignalInputSocket:
    def __init__(self, signal_source_name: str, message_type: any, callback: Callable[[any], None]):
        self._signal_source_name = signal_source_name
        self._callback = callback
        self._message_type = message_type
        self._subscriber = self._hook_into_signal_source()

    def _hook_into_signal_source(self):
        return rospy.Subscriber(self._signal_source_name, self._message_type, self._callback)


class SignalOutputSocket:
    def __init__(self, signal_source_name: str, message_type: any):
        self._signal_source_name = signal_source_name
        self._message_type = message_type
        self._publisher = self._add_signal_source()

    def _add_signal_source(self):
        return rospy.Publisher(self._signal_source_name, self._message_type, queue_size=1000)

    def send_signal(self, signal_argument: any):
        self._publisher.publish(signal_argument)


class TaskInputSocket:
    def __init__(self, task_source_name: str, task_type: any, task_callback: Callable[[any], any]):
        self._task_source_name = task_source_name
        self._task_type = task_type
        self._task_callback = task_callback
        self._service_server = self._hook_into_task_source()

    def _hook_into_task_source(self):
        print('Creating service socket with name : ', self._task_source_name)
        return rospy.Service(self._task_source_name, self._task_type, self._task_callback)


class TaskOutputSocket:
    def __init__(self, task_source_name: str, task_type: any):
        self._task_source_name = task_source_name
        self._task_type = task_type
        self._task_client = self._add_task_source()

    def _add_task_source(self):
        return rospy.ServiceProxy(self._task_source_name, self._task_type)

    def execute_task(self, task: any) -> any:
        rospy.wait_for_service(self._task_source_name)
        return self._task_client(task)
