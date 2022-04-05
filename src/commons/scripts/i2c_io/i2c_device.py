from typing import List
import smbus2


class I2CDevice:
    def __init__(self, bus_number: int, device_address) -> None:
        self._bus = smbus2.SMBus(bus_number)
        self._device_address = device_address

    def write_byte(self, byte: int):
        self._bus.write_byte(self._device_address, byte)

    def read_block_data_from(self, register: int, length: int) -> List[int]:
        return self._bus.read_i2c_block_data(self._device_address, register, length)

    def write_block_data_to(self, register: int, data: List[int]) -> None:
        self._bus.write_i2c_block_data(self._device_address, register, data)
