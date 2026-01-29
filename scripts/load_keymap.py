import argparse
import serial
import struct
import time
import yaml
import re
try:
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
    from yaml import Loader, Dumper

CLEAR = b"\x06"
SAVE = b"\x07"
UPDATE_LEFTMAP = b"\x08"
UPDATE_RIGHTMAP = b"\x09"
UPDATE_NUMMAP_ALT = b"\x0A"
UPDATE_ROTARY = b"\x0B"

KEYCODE_ID = 0
CONSUMER_ID = 1
CUSTOM_ID = 2
HOLD_ID = 3
TAP_ID = 4

HOLD_STR = "HOLD_ALT"
TAP_STR = "TAP_ALT"

keycodes = {
  "A": 0x04,
  "B": 0x05,
  "C": 0x06,
  "D": 0x07,
  "E": 0x08,
  "F": 0x09,
  "G": 0x0a,
  "H": 0x0b,
  "I": 0x0c,
  "J": 0x0d,
  "K": 0x0e,
  "L": 0x0f,
  "M": 0x10,
  "N": 0x11,
  "O": 0x12,
  "P": 0x13,
  "Q": 0x14,
  "R": 0x15,
  "S": 0x16,
  "T": 0x17,
  "U": 0x18,
  "V": 0x19,
  "W": 0x1a,
  "X": 0x1b,
  "Y": 0x1c,
  "Z": 0x1d,

  1: 0x1e,
  2: 0x1f,
  3: 0x20,
  4: 0x21,
  5: 0x22,
  6: 0x23,
  7: 0x24,
  8: 0x25,
  9: 0x26,
  0: 0x27,

  "ENTER": 0x28,
  "ESC": 0x29,
  "BACKSPACE": 0x2a,  # Keyboard DELETE (Backspace)
  "TAB": 0x2b,
  "SPACE": 0x2c,
  "MINUS": 0x2d,
  "EQUAL": 0x2e,
  "LBRACE": 0x2f,  # Keyboard [ and {
  "RBRACE": 0x30,  # Keyboard ] and }
  "BACKSLASH": 0x31,  # Keyboard \ and |
  "HASHTILDE": 0x32,  # Keyboard Non-US # and ~
  "SEMICOLON": 0x33,  # Keyboard ; and :
  "APOSTROPHE": 0x34,  # Keyboard ' and "
  "GRAVE": 0x35,  # Keyboard ` and ~
  "COMMA": 0x36,  # Keyboard , and <
  "DOT": 0x37,  # Keyboard . and >
  "SLASH": 0x38,  # Keyboard / and ?
  "CAPS": 0x39,

  "F1": 0x3a,
  "F2": 0x3b,
  "F3": 0x3c,
  "F4": 0x3d,
  "F5": 0x3e,
  "F6": 0x3f,
  "F7": 0x40,
  "F8": 0x41,
  "F9": 0x42,
  "F10": 0x43,
  "F11": 0x44,
  "F12": 0x45,

  "SYSRQ": 0x46,  # Keyboard Print Screen
  "SCROLLLOCK": 0x47,  # Keyboard Scroll Lock
  "PAUSE": 0x48,  # Keyboard Pause
  "INSERT": 0x49,  # Keyboard Insert
  "HOME": 0x4a,  # Keyboard Home
  "PAGEUP": 0x4b,  # Keyboard Page Up
  "DELETE": 0x4c,  # Keyboard Delete Forward
  "END": 0x4d,  # Keyboard End
  "PAGEDOWN": 0x4e,  # Keyboard Page Down
  "RIGHT": 0x4f,  # Keyboard Right Arrow
  "LEFT": 0x50,  # Keyboard Left Arrow
  "DOWN": 0x51,  # Keyboard Down Arrow
  "UP": 0x52,  # Keyboard Up Arrow

  "NUMLOCK": 0x53,  # Keyboard Num Lock and Clear
  "KPSLASH": 0x54,  # Keypad /
  "KPASTERISK": 0x55,  # Keypad *
  "KPMINUS": 0x56,  # Keypad -
  "KPPLUS": 0x57,  # Keypad +
  "KPENTER": 0x58,  # Keypad ENTER
  "KP1": 0x59,  # Keypad 1 and End
  "KP2": 0x5a,  # Keypad 2 and Down Arrow
  "KP3": 0x5b,  # Keypad 3 and PageDn
  "KP4": 0x5c,  # Keypad 4 and Left Arrow
  "KP5": 0x5d,  # Keypad 5
  "KP6": 0x5e,  # Keypad 6 and Right Arrow
  "KP7": 0x5f,  # Keypad 7 and Home
  "KP8": 0x60,  # Keypad 8 and Up Arrow
  "KP9": 0x61,  # Keypad 9 and Page Up
  "KP0": 0x62,  # Keypad 0 and Insert
  "KPDOT": 0x63,  # Keypad . and Delete

  "LCTRL": 0xE0,
  "LSHIFT": 0xE1,
  "LALT": 0xE2,
  "LWIN": 0xE3,
  "RCTRL": 0xE4,
  "RSHIFT": 0xE5,
  "RALT": 0xE6,
  "RWIN": 0xE7,
}

consumer_codes = {
  "MUTE": 0xE2,
  "VOL_INC": 0xE9,
  "VOL_DEC": 0xEA,
}

custom_codes = {
  "MOD": 0x00,
  "NONE": 0x01,
}


def flatten_and_sort(keymap, coordmap):
    map = {}
    for (row_coord, row_keymap) in zip(coordmap, keymap):
        for coord, keymap in zip(row_coord, row_keymap):
            map[f"{coord[0]},{coord[1]}"] = keymap

    keymap_list = []
    col = 0
    while len(map) > 0:
        row = 0
        while (keymap := map.get(f"{col},{row}")) is not None:
            keymap_list.append(keymap)
            del map[f"{col},{row}"]
            row += 1
        col += 1
    return keymap_list


def map_code(handle):
    match = re.match(r"(\S+)\((\S+)\)", str(handle))
    if match is not None:
        handle = match.group(2)

    if handle in keycodes:
        category = KEYCODE_ID
        hexcode = keycodes[handle]
    elif handle in consumer_codes:
        category = CONSUMER_ID
        hexcode = consumer_codes[handle]
    elif handle in custom_codes:
        category = CUSTOM_ID
        hexcode = custom_codes[handle]
    else:
        raise KeyError(f"Invalid key: {handle}")

    if match is not None:
        if match.group(1) == HOLD_STR:
            category = HOLD_ID
        elif match.group(1) == TAP_STR:
            category = TAP_ID

    return struct.pack("<BB", category, hexcode)


# board | col | row | catergory | hexcode
def push_keymap(ser, preamble, coord_map, key_map):
    for (coords, handles) in zip(coord_map, key_map):
        for (coord, handle) in zip(coords, handles):
            packet = preamble + struct.pack("<BB", coord[0], coord[1]) + map_code(handle)
            ser.write(packet)
            time.sleep(0.01)


# id | ccw catergory | ccw hexcode  | cw catergory | cw hexcode
def push_rotary(ser, binds):
    packet = UPDATE_ROTARY
    for handle in binds:
        packet += map_code(handle)
    ser.write(packet)
    time.sleep(0.01)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("serial", type=str, default="/dev/ttyACM0", nargs="?", help="Serial port of device")
    parser.add_argument("-k", "--keymap", type=str, default="keymap.yml", help="Keymap yaml file")
    args = parser.parse_args()

    with open(args.keymap) as fd:
        map = yaml.safe_load(fd)

    ser = serial.Serial(args.serial)
    ser.write(CLEAR)
    time.sleep(0.2)
    push_keymap(ser, UPDATE_LEFTMAP, map["lb_coord_map"], map["lb_keymap"])
    push_keymap(ser, UPDATE_RIGHTMAP, map["rb_coord_map"], map["rb_keymap"])
    push_keymap(ser, UPDATE_RIGHTMAP, map["np_coord_map"], map["np_keymap"])
    push_keymap(ser, UPDATE_NUMMAP_ALT, map["np_coord_map"], map["np_keymap_alt"])
    push_rotary(ser, map["rotary_binds"])
    ser.write(SAVE)
    ser.close()

    # out = flatten_and_sort(map["lb_keymap"], map["lb_coord_map"])
    # print(out)

    # ser = serial.Serial(args.serial)
    #
    # ser.write(b"\x06")  # Set to upload leftboard keymap mode
    #
    # ser.close()


if __name__ == "__main__":
    main()
