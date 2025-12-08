import argparse
import serial
import struct
import time
from enum import Enum
from dataclasses import dataclass
from threading import Lock

from textual import work
from textual.app import App, ComposeResult
from textual.containers import Horizontal, Vertical
from textual.validation import Number
from textual.widgets import Button, Log, Static, Switch, Input


BRIDGE_CMD = b"\x03"
END_BRIDGE = b"\x04"

DELIM = 0  # 1-255


keycodes = {
  0x04: "A",
  0x05: "B",
  0x06: "C",
  0x07: "D",
  0x08: "E",
  0x09: "F",
  0x0a: "G",
  0x0b: "H",
  0x0c: "I",
  0x0d: "J",
  0x0e: "K",
  0x0f: "L",
  0x10: "M",
  0x11: "N",
  0x12: "O",
  0x13: "P",
  0x14: "Q",
  0x15: "R",
  0x16: "S",
  0x17: "T",
  0x18: "U",
  0x19: "V",
  0x1a: "W",
  0x1b: "X",
  0x1c: "Y",
  0x1d: "Z",

  0x1e: 1,
  0x1f: 2,
  0x20: 3,
  0x21: 4,
  0x22: 5,
  0x23: 6,
  0x24: 7,
  0x25: 8,
  0x26: 9,
  0x27: 0,

  0x28: "ENTER",
  0x29: "ESC",
  0x2a: "BACKSPACE",  # Keyboard DELETE (Backspace)
  0x2b: "TAB",
  0x2c: "SPACE",
  0x2d: "MINUS",
  0x2e: "EQUAL",
  0x2f: "LBRACE",  # Keyboard [ and {
  0x30: "RBRACE",  # Keyboard ] and }
  0x31: "BACKSLASH",  # Keyboard \ and |
  0x32: "HASHTILDE",  # Keyboard Non-US # and ~
  0x33: "SEMICOLON",  # Keyboard ; and :
  0x34: "APOSTROPHE",  # Keyboard ' and "
  0x35: "GRAVE",  # Keyboard ` and ~
  0x36: "COMMA",  # Keyboard , and <
  0x37: "DOT",  # Keyboard . and >
  0x38: "SLASH",  # Keyboard / and ?
  0x39: "CAPS",

  0x3a: "F1",
  0x3b: "F2",
  0x3c: "F3",
  0x3d: "F4",
  0x3e: "F5",
  0x3f: "F6",
  0x40: "F7",
  0x41: "F8",
  0x42: "F9",
  0x43: "F10",
  0x44: "F11",
  0x45: "F12",

  0x46: "SYSRQ",  # Keyboard Print Screen
  0x47: "SCROLLLOCK",  # Keyboard Scroll Lock
  0x48: "PAUSE",  # Keyboard Pause
  0x49: "INSERT",  # Keyboard Insert
  0x4a: "HOME",  # Keyboard Home
  0x4b: "PAGEUP",  # Keyboard Page Up
  0x4c: "DELETE",  # Keyboard Delete Forward
  0x4d: "END",  # Keyboard End
  0x4e: "PAGEDOWN",  # Keyboard Page Down
  0x4f: "RIGHT",  # Keyboard Right Arrow
  0x50: "LEFT",  # Keyboard Left Arrow
  0x51: "DOWN",  # Keyboard Down Arrow
  0x52: "UP",  # Keyboard Up Arrow

  0x53: "NUMLOCK",  # Keyboard Num Lock and Clear
  0x54: "KPSLASH",  # Keypad /
  0x55: "KPASTERISK",  # Keypad *
  0x56: "KPMINUS",  # Keypad -
  0x57: "KPPLUS",  # Keypad +
  0x58: "KPENTER",  # Keypad ENTER
  0x59: "KP1",  # Keypad 1 and End
  0x5a: "KP2",  # Keypad 2 and Down Arrow
  0x5b: "KP3",  # Keypad 3 and PageDn
  0x5c: "KP4",  # Keypad 4 and Left Arrow
  0x5d: "KP5",  # Keypad 5
  0x5e: "KP6",  # Keypad 6 and Right Arrow
  0x5f: "KP7",  # Keypad 7 and Home
  0x60: "KP8",  # Keypad 8 and Up Arrow
  0x61: "KP9",  # Keypad 9 and Page Up
  0x62: "KP0",  # Keypad 0 and Insert
  0x63: "KPDOT",  # Keypad . and Delete

  0xE0: "LCTRL",
  0xE1: "LSHIFT",
  0xE2: "LALT",
  0xE3: "LWIN",
  0xE4: "RCTRL",
  0xE5: "RSHIFT",
  0xE6: "RALT",
  0xE7: "RWIN",
}


class CmdId(Enum):
    ACK = 0
    NACK = 1

    GET_STATE = 2
    ALT_ENABLE = 3
    CAPSLOCK = 4
    VOLUME = 5
    OLED_MSG = 6


class Rspn(Enum):
    ACK = 0
    NACK = 1

    FULL_STATE = 2
    ALT_STATE = 3

    KEY_CHANGE = 4
    ROTARY = 5

    TIMESTAMP = 64
    DEFMT_MSG = 128


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


def poll_for_packet(ser) -> (Rspn, bytearray):
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
    return Rspn(data[0]), data[1:]


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
    code: str
    state: bool
    last_press: int = 0
    last_release: int = 0

    def __init__(self, data, timestamp):
        self.state = data & 0x80

        if self.state:
            self.last_press = timestamp
        else:
            self.last_release = timestamp

        data = data & 0x7f
        if data < 0x66:
            self.code = keycodes[data]
            return

        data -= 0x66
        if data < 8:
            self.code = keycodes[data + 0xE0]
            return

        data -= 8
        if data == 0:
            self.code = "Mute"
        elif data == 1:
            self.code = "VolInc"
        elif data == 2:
            self.code = "VolDec"
        else:
            self.code = "INVALID"

    def __str__(self):
        return f"{self.code}: {self.state}"

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
            return f"{self.code}: {(self.last_release-self.last_press)/1000:.3f}"
        return None

    def key_id(self):
        return self.code


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
            data = struct.pack("<B", CmdId.GET_STATE.value)
            self.serial.write_sync(cobs_encode(data))

    async def on_switch_changed(self, change):
        map = {
            "numlock": CmdId.ALT_ENABLE.value,
            "capslock": CmdId.CAPSLOCK.value,
        }
        data = struct.pack("<BB", map[change.switch.id], change.switch.value)
        self.serial.write_sync(cobs_encode(data))

    async def on_input_submitted(self, event):
        if event.validation_result:
            data = struct.pack("<BB", CmdId.VOLUME.value, int(event.value))
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
                case Rspn.KEY_CHANGE:
                    keys = struct.unpack(f"<{len(payload)}B", payload)
                    changes = [KeyState(key, ts) for key in list(keys)]
                    msg = "|".join(handle_change(changes, states))
                    # msg = "|".join([str(state) for state in changes])
                case Rspn.FULL_STATE:
                    msg = f"{id.name}: {int(payload.hex(), 16):064b}"
                case Rspn.ALT_STATE:
                    msg = f"{id.name}: {int(payload.hex(), 16):064b}"
                case Rspn.TIMESTAMP:
                    ts, = struct.unpack("<Q", payload)
                    msg = f"{ts/1_000}ms"
                    continue
                case Rspn.ROTARY:
                    change, = struct.unpack("<b", payload)
                    msg = f"Rotary: {change}"
                case _:
                    msg = f"{id.name}: {payload}"
            self.call_from_thread(self.logger.write_line, msg)


def send_serial(path, val):
    ser = serial.Serial(path)
    ser.write(val)
    ser.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(prog="Serial tester")
    parser.add_argument("serial", type=str, default="/dev/ttyUSB0", nargs="?", help="Serial port of device")
    parser.add_argument(
        "-b", "--enable-bridge",
        nargs="?",
        const="/dev/ttyACM0",
        default=None,
        help="Send USB command to enable bridge"
    )
    args = parser.parse_args()
    if args.enable_bridge is not None:
        print("Enabling bridge")
        send_serial(args.enable_bridge, BRIDGE_CMD)
    app = RightboardApp(args.serial, css_path="tui.tcss")
    app.run()

    if args.enable_bridge is not None:
        print("Stopping bridge")
        send_serial(args.enable_bridge, END_BRIDGE)
