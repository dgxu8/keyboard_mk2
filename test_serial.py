from __future__ import annotations

import argparse
import serial
import struct
import subprocess
import sys
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


def poll_for_packet(ser):
        packet = bytearray()
        while (recv := ser.read()) != b"\x00":
            packet += recv
        if len(packet) == 0:
            return b""
        packet += b"\x00"
        data, remainder = cobs_decode(packet)
        assert len(remainder) == 0, "Some how we got left over data"
        return data


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


def rb_poll_keys(ser):
    class Toggle:
        def __init__(self, timestamp, col, row, state):
            self.ts = timestamp
            self.col = col
            self.row = row
            self.state = state

    last_press = time.monotonic()
    last_release = time.monotonic()

    log = []
    LOG_FILE = "data.csv"
    try:
        while True:
            while len(data := poll_for_packet(ser)) == 0:
                print("serial timeout looking for timestamp")
            cmd, curr = struct.unpack("<BQ", data)
            assert cmd == 5, "Looking for timestamp id"
            while len(data := poll_for_packet(ser)) == 0:
                print("serial timeout looking for update")
            # print(f"{len(data)}: 0x{data.hex()}")
            cmd, = struct.unpack("<B", data[:1])
            assert cmd == 2 or cmd == 3,  "Looking for key change id"
            if cmd == 3:
                print(f"Full state: 0x{data[1:].hex()}")
                continue
            data, = struct.unpack("<B", data[1:2])
            # data, curr= struct.unpack("<BQ", data)  # curr is micro seconds
            val = data & 0x1
            row = (data >> 1) & 0b111
            col = (data >> 4) & 0b111
            alt = data >> 7
            log.append(Toggle(curr, col, row, val))

            if val == 1:
                if (elapsed := curr - last_press) < 100_000:
                    print(f"double press detected: {elapsed/1000:.3f}")
                    last_press = curr
                    continue
                last_press = curr
            else:
                if (elapsed := curr - last_release) < 100_000:
                    print(f"double release detected: {elapsed/1000:.3f}")
                    last_release = curr
                    continue
                last_release = curr
                print(f"[{alt}]({col}, {row}): {(last_release-last_press)/1000:.3f}")
            # print(f"{curr/1_000:.3f}: ({col}, {row}): {val}")
    except KeyboardInterrupt:
        pass
    start_us = log[0].ts
    last_us = 0
    channels = {}
    for toggle in log:
        channels[f"{toggle.col}:{toggle.row}"] = 0
    channel_cnt = len(channels.keys())
    channel_names = list(channels.keys())
    channel_names.sort()
    with open(LOG_FILE, "w") as fd:
        fd.write(",".join(channel_names))
        fd.write("\n")
        for entry in log:
            entry_ts = entry.ts - start_us
            for _ in range(last_us, entry_ts):
                fd.write(",".join([str(channels[n]) for n in channel_names]))
                fd.write("\n")
            last_us = entry_ts
            for name in channel_names:
                if name == f"{entry.col}:{entry.row}":
                    channels[name] = entry.state
            fd.write(",".join([str(channels[n]) for n in channel_names]))
            fd.write("\n")
    subprocess.run(f"", shell=True)


def rb_decode_test(ser):
    elapsed = 0
    while True:
        data = struct.pack("<Q", int(elapsed))

        start = time.monotonic()
        packet = cobs_encode(data)
        ser.write(packet)
        elapsed = (time.monotonic() - start) * 1_000_000

        msg = bytearray()
        while len(recv := ser.read()) > 0:
            msg += recv
        if len(msg) > 0:
            print(msg.decode("Latin-1"))
        time.sleep(0.5)


def encoder(ser):
    data = []
    try:
        while True:
            recv: bytes = ser.read()
            if len(recv) == 0:
                continue
            data.append(int.from_bytes(recv, "little"))
    except KeyboardInterrupt:
        pass
    with open("sample.csv", "w") as fd:
        fd.write(f"a,b\n")
        for byte in data:
            a = byte & 0x1
            b = (byte >> 4) & 0x1
            fd.write(f"{a},{b}\n")
    # Convert to sr
    subprocess.run("./sigrok-cli -I csv:header=yes -i sample.csv -O srzip -o sample_cw6.sr", shell=True)


def main():
    parser = argparse.ArgumentParser(prog="Serial tester")
    parser.add_argument("serial", type=str, help="Serial port of device")
    args = parser.parse_args()

    with serial.Serial(args.serial, 2_000_000, timeout=None) as ser:
        rb_poll_keys(ser)


if __name__ == "__main__":
    main()
