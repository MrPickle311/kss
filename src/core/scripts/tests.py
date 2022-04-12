import unittest
import core_internals.station_states

PKG = 'core_internals'


class TestStringMethods(unittest.TestCase):

    def test_value_in_midle_range_ok(self):
        idle_state = core_internals.station_states.IdleState()
        idle_state.current_station_data.drone_state.drone_battery_voltage = idle_state.MIN_DRONE_BATTERY_VOLTAGE + 0.5
        self.assertTrue(idle_state.is_good_battery_level())
        self.assertFalse(idle_state.is_battery_low_level())

    def test_value_in_low_range_ok(self):
        idle_state = core_internals.station_states.IdleState()
        idle_state.current_station_data.drone_state.drone_battery_voltage = idle_state.MIN_DRONE_BATTERY_VOLTAGE - 0.5
        self.assertFalse(idle_state.is_good_battery_level())
        self.assertTrue(idle_state.is_battery_low_level())


if __name__ == '__main__':
    # idle_state = core_internals.station_states.IdleState()
    # import rosunit
    # rosunit.unitrun(PKG, 'tests', TestStringMethods)
