from socket import IPV6_CHECKSUM
import serial
import time
from typing import Literal

ser = serial.Serial(
    port='COM6',
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)
from typing import Literal

if ser.isOpen():
    print(ser.name + ' is open...')

STX = b'\x02'
ETX = b'\x03'

ISOK = 0
ERR = 1


def add_frame_borders(cmd: bytearray):
    return STX + cmd + ETX


def print_incoming_bytes(count: int):
    bytes = ser.read(count)
    print(bytes)
    return bytes


def write_cmd_indicator(cmd_ind: bytearray):
    cmd_indicator = add_frame_borders(cmd_ind)
    ser.write(cmd_indicator)
    print_incoming_bytes(3)


def write_cmd(cmd: Literal):
    cmd = add_frame_borders(cmd)
    ser.write(cmd)


def reset():
    write_cmd_indicator(b'\x03')
    write_cmd(b'\x00')
    print_incoming_bytes(3)


def get_calibration_state():
    write_cmd_indicator(b'\x03')
    write_cmd(b'\x01')
    bytes = print_incoming_bytes(2)

    if bytes[1] == ISOK:
        bytes += print_incoming_bytes(2)
        print(bytes[2])
    else:
        print_incoming_bytes(1)
        print('Error')


def get_movement_state():
    write_cmd_indicator(b'\x03')
    write_cmd(b'\x02')
    bytes = print_incoming_bytes(2)

    if bytes[1] == ISOK:
        bytes += print_incoming_bytes(2)
        print(bytes[2])
    else:
        print_incoming_bytes(1)
        print('Error')


def get_current_location():
    write_cmd_indicator(b'\x03')
    write_cmd(b'\x03')
    print('Collecting cnmd response...')
    bytes = print_incoming_bytes(2)

    if bytes[1] == ISOK:
        bytes += print_incoming_bytes(3)
        pos = int.from_bytes(bytes[2:4], "big", signed=False)
        print(bytes)
        print(pos)
    else:
        print_incoming_bytes(1)
        print('Error')


def go_to_absolute_position(pos: int):
    write_cmd_indicator(b'\x05')
    pos_bytes = int.to_bytes(pos, 2, byteorder='big', signed=False)
    write_cmd(b'\x04' + pos_bytes)
    print('Collecting cnmd response...')
    bytes = print_incoming_bytes(3)

    if bytes[1] == ISOK:
        print('Set position successfully')
    else:
        print('Failed to set position')


CLOCKWISE = 0x00
ANTICLOCKWISE = 0x01


def go_to_relative_position(pos: int, direction: int):
    write_cmd_indicator(b'\x06')
    pos_bytes = int.to_bytes(pos, 2, byteorder='big', signed=False)
    dir_byte = int.to_bytes(direction, 1, byteorder='big', signed=False)
    write_cmd(b'\x05' + pos_bytes + dir_byte)
    print('Collecting cnmd response...')
    bytes = print_incoming_bytes(3)

    if bytes[1] == ISOK:
        print('Set position successfully')
    else:
        print('Failed to set position')


# reset()
# get_calibration_state()
get_movement_state()
# get_current_location()
# go_to_absolute_position(0)
go_to_relative_position(0x2FFF, ANTICLOCKWISE)
