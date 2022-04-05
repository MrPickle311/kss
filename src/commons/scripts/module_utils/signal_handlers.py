import signal, sys


class SigIntHandler:
    def __init__(self):
        signal.signal(signal.SIGINT, self.signal_handler)

    @staticmethod
    def signal_handler(inc_signal, frame):
        sys.exit(0)
