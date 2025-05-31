from __future__ import annotations

import argparse
import serial
import struct
import time

from dataclasses import dataclass
from enum import Enum

# Consistent Overhead Byte Stuffing but limiting to 254 byte packets
DELIM = 0  # 1-255
MAX_PACKET_SIZE = 254

class Opcode(Enum):
    DNU = 0  # block out 0 incase we want to use it as the delimeter
    COMMAND = 1
    GET_KEY_STATE = 2  # Returns list of all pressed keys
    KEYCODE = 3  # returns 2 bytes with row/col
    OLED_DRAW = 4


@dataclass
class Packet:
    opcode: int  # size: 4 bits
    data: int  # depends on id each byte is only 7 bits


def cobs_encode(data: bytes | bytearray):
    buffer: bytearray = bytearray(1)
    delim_cnt = 1
    last_delim = 0

    data = bytearray(data)
    data += b"\x00"

    for idx in range(len(data)):
        val = data[idx]

        buffer += bytes([val])
        if val == DELIM:
            buffer[last_delim] = delim_cnt
            delim_cnt = 1
            last_delim = idx + 1
            continue

        delim_cnt += 1

    return buffer


def cobs_decode(data: bytearray):
    buffer = bytearray()
    delim_num = data.pop(0)
    while (byte := data.pop(0)) != DELIM:
        delim_num -= 1
        if delim_num == 0:
            delim_num = byte
            buffer += b"\x00"
        else:
            buffer += bytes([byte])

    return (buffer, data)


def cobs_self_test():
    data = struct.pack("<BBBBB", Opcode.COMMAND.value, 0, 10, 16, 0)
    print(data)

    packed = cobs_encode(data)
    print(packed.hex())

    unpacked, data = cobs_decode(bytearray(packed) + b"\x04")
    print(f"{unpacked.hex()} - {data}")


def rb_encode_test(ser):
    while True:
        packet = bytearray()
        while (recv := ser.read()) != b"\x00":
            packet += recv
        if len(packet) == 0:
            continue
        packet += b"\x00"
        print(packet)
        data, _ = cobs_decode(packet)
        scan_time, cobs_time = struct.unpack("<QQ", data)
        print(f"{scan_time}us, COBS: {cobs_time}us")


def main():
    parser = argparse.ArgumentParser(prog="Serial tester")
    parser.add_argument("serial", type=str, help="Serial port of device")
    args = parser.parse_args()

    try:
        with serial.Serial(args.serial, 2_000_000, timeout=0.1) as ser:
            rb_encode_test(ser)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
