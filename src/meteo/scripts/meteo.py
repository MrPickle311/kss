from typing import Callable, Optional
import requests
from multitimer import MultiTimer
from module_io.module_interface import AbstractModule
from module_utils.signal_handlers import SigIntHandler
import rospy

from commons.msg import MeteoProcessGoal, MeteoProcessFeedback, MeteoProcessResult, MeteoProcessAction
from enums.meteo import DevicesStringDescriptors, DevicesIntDescriptors
from data_models.meteo import TemperatureAndHumiditySensorData, RainSensorData, WindSensorData, DeviceInfo, \
    MeteoTransaction


class MeteoReceiver:
    URL = "https://www.data199.com/api/pv1/device/lastmeasurement"
    DEVICES_IDS = '{0},{1},{2}'.format(DevicesStringDescriptors.TEMPERATURE_AND_HUMIDITY_SENSOR_ID,
                                       DevicesStringDescriptors.RAIN_SENSOR_ID, DevicesStringDescriptors.WIND_SENSOR_ID)

    def __init__(self, data_pull_seconds_timeout: int, data_extraction_handler: Callable[[MeteoTransaction], None]):
        self._data_extraction_handler = data_extraction_handler
        self._timer = MultiTimer(data_pull_seconds_timeout, self._request_data_from_meteo_receiver)

    def start_receiving_data(self):
        self._timer.start()

    def stop_receiving_data(self):
        self._timer.stop()

    def _request_data_from_meteo_receiver(self):
        r = requests.post(MeteoReceiver.URL, data={'deviceids': MeteoReceiver.DEVICES_IDS})
        try:
            transaction = MeteoTransaction.parse_obj(r.json())
            self._data_extraction_handler(transaction)
        except Exception:
            # TODO: communicate Core about  an error
            print('Too fast!!')
            pass


class MeteoModule(AbstractModule):

    def __init__(self, module_name: str):
        AbstractModule.__init__(self, module_name, MeteoProcessAction)
        self._meteo_receiver: Optional[MeteoReceiver] = None

    def _init_meteo_receiver(self, start_args: MeteoProcessGoal):
        def feedback_extraction_handler(transaction: MeteoTransaction):
            self.send_feedback(transaction)

        self._meteo_receiver = MeteoReceiver(start_args.timeout, feedback_extraction_handler)

    def start_module(self, start_args: MeteoProcessGoal) -> None:
        self._init_meteo_receiver(start_args)

        self._meteo_receiver.start_receiving_data()
        self.wait_for_module_finish()

        self.finish_module(0)

    @staticmethod
    def _extract_temperature_and_humidity_to_feedback(feedback: MeteoProcessFeedback,
                                                      sensor_data: TemperatureAndHumiditySensorData):
        feedback.temperature = sensor_data.t1
        feedback.humidity = sensor_data.h

    @staticmethod
    def _extract_rain_data_to_feedback(feedback: MeteoProcessFeedback, sensor_data: RainSensorData):
        feedback.rain_mm = sensor_data.r

    @staticmethod
    def _extract_wind_data_to_feedback(feedback: MeteoProcessFeedback, sensor_data: WindSensorData):
        feedback.wind_speed = sensor_data.ws
        feedback.wind_gust = sensor_data.wg
        feedback.wind_direction = sensor_data.wd

    def send_feedback(self, transaction: MeteoTransaction) -> None:
        feedback = MeteoProcessFeedback()

        self._extract_temperature_and_humidity_to_feedback(feedback, transaction.devices[
            DevicesIntDescriptors.TEMPERATURE_AND_HUMIDITY_SENSOR_ID].measurement)

        self._extract_rain_data_to_feedback(feedback, transaction.devices[
            DevicesIntDescriptors.RAIN_SENSOR_ID].measurement)

        self._extract_wind_data_to_feedback(feedback, transaction.devices[
            DevicesIntDescriptors.WIND_SENSOR_ID].measurement)

        self._server.publish_feedback(feedback)

    def finish_module(self, exit_code: int) -> None:
        result = MeteoProcessResult()
        result.exit_code = exit_code
        self._server.set_succeeded(result)


def main():
    rospy.init_node('meteo_interface', anonymous=True)
    sigint_handler = SigIntHandler()
    automation_process = MeteoModule('meteo_interface')
    rospy.spin()


if __name__ == '__main__':
    main()
