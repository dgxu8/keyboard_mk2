import argparse
import serial
import struct
import queue
import time
from enum import Enum
from dataclasses import dataclass
from threading import Lock

from textual import work
from textual.app import App, ComposeResult
from textual.containers import Horizontal, Vertical
from textual.validation import Number
from textual.widgets import Button, Log, Static, Switch, Input


DELIM = 0  # 1-255


class Opcode(Enum):
    ACK = 0
    NACK = 1

    KEY_CHANGE = 2  # Returns list of all pressed keys

    GET_STATE = 3  # returns 2 bytes with row/col
    FULL_STATE = 4
    ALT_STATE = 5

    ALT_ENABLE = 6

    CAPSLOCK = 7

    ROTORY = 8

    VOLUME = 9

    TIMESTAMP = 64
    DEFMT_MSG = 128


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


def poll_for_packet(ser) -> (Opcode, bytearray):
    packet = bytearray()
    recv = b""
    while recv != b"\x00":
        recv = ser.read_sync()
        if len(recv) == 0:
            time.sleep(0.01)
        else:
            packet += recv
    if len(packet) == 0:
        return b""
    data, remainder = cobs_decode(packet)
    assert len(remainder) == 0, "Some how we got left over data"
    return Opcode(data[0]), data[1:]


def handle_change(changes, key_state):
    msgs = []
    for change in changes:
        id = change.key_id()
        if id not in key_state:
            key_state[id] = change
            continue
        msg = key_state[id].update_check(change)
        if msg is not None:
            msgs.append(msg)
    return msgs


@dataclass
class KeyState:
    alt: int
    x: int
    y: int
    state: bool
    last_press: int = 0
    last_release: int = 0

    def __init__(self, data, timestamp):
        self.state = data & 0x1
        self.y = (data >> 1) & 0b111
        self.x = (data >> 4) & 0b111
        self.alt = data >> 7
        if self.state:
            self.last_press = timestamp
        else:
            self.last_release = timestamp

    def __str__(self):
        return f"[{self.alt}]({self.x}, {self.y}): {self.state}"

    def update_check(self, new_state) -> str:
        self.state = new_state.state
        if self.state:
            elapsed = new_state.last_press - self.last_press
            self.last_press = new_state.last_press
            if elapsed < 100_000:
                return f"double press detected: {elapsed/1000:.3f}"
        else:
            elapsed = new_state.last_release - self.last_release
            self.last_release = new_state.last_release
            if elapsed < 100_000:
                return f"double release detected: {elapsed/1000:.3f}"
            return f"[{self.alt}]({self.x}, {self.y}): {(self.last_release-self.last_press)/1000:.3f}"
        return None

    def key_id(self):
        return (self.alt, self.x, self.y)


class SerialSync(serial.Serial):
    def __init__(self, *args, **kwargs):
        self.lock = Lock()
        super().__init__(*args, **kwargs)

    def read_sync(self):
        with self.lock:
            val = self.read()
        return val

    def write_sync(self, data):
        with self.lock:
            self.write(data)


class RightboardApp(App):
    def __init__(self, port, *args, **kwargs):
        self.serial = SerialSync(port, 2_000_000, timeout=0.1)
        super().__init__(*args, **kwargs)

    def __del__(self):
        self.serial.close()
        super().__del__()

    async def on_button_pressed(self, event):
        if event.button.id == "get_state":
            data = struct.pack("<B", Opcode.GET_STATE.value)
            self.serial.write_sync(cobs_encode(data))

    async def on_switch_changed(self, change):
        map = {
            "numlock": Opcode.ALT_ENABLE.value,
            "capslock": Opcode.CAPSLOCK.value,
        }
        data = struct.pack("<BB", map[change.switch.id], change.switch.value)
        self.serial.write_sync(cobs_encode(data))

    async def on_input_submitted(self, event):
        if event.validation_result:
            data = struct.pack("<BB", Opcode.VOLUME.value, int(event.value))
            self.serial.write_sync(cobs_encode(data))

    def compose(self) -> ComposeResult:
        with Horizontal():
            with Vertical():
                yield Button("Get State", id="get_state")
                with Horizontal():
                    yield Static("Numlock:  ", classes="label")
                    yield Switch(animate=False, id="numlock")
                with Horizontal():
                    yield Static("Capslock: ", classes="label")
                    yield Switch(animate=False, id="capslock")
                yield Input(
                    placeholder="100",
                    validators=[Number(minimum=0, maximum=100)],
                )
            yield Log()

    def on_ready(self) -> None:
        self.logger = self.query_one(Log)
        self.read_serial()

    @work(thread=True)
    def read_serial(self):
        ts = 0
        states = {}
        while True:
            id, payload = poll_for_packet(self.serial)
            match id:
                case Opcode.KEY_CHANGE:
                    keys = struct.unpack(f"<{len(payload)}B", payload)
                    changes = [KeyState(key, ts) for key in list(keys)]
                    msg = "|".join(handle_change(changes, states))
                    # msg = "|".join([str(state) for state in changes])
                case Opcode.FULL_STATE:
                    msg = f"{id.name}: {int(payload.hex(), 16):064b}"
                case Opcode.ALT_STATE:
                    msg = f"{id.name}: {int(payload.hex(), 16):064b}"
                case Opcode.TIMESTAMP:
                    ts, = struct.unpack("<Q", payload)
                    msg = f"{ts/1_000}ms"
                    continue
                case Opcode.ROTORY:
                    change, = struct.unpack("<b", payload)
                    msg = f"Rotary: {change}"
                case _:
                    msg = f"{id.name}: {payload}"
            self.call_from_thread(self.logger.write_line, msg)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(prog="Serial tester")
    parser.add_argument("serial", type=str, default="/dev/ttyUSB0", nargs="?", help="Serial port of device")
    args = parser.parse_args()
    app = RightboardApp(args.serial, css_path="tui.tcss")
    app.run()
