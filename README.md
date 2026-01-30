[Keyboard v2 Firmware]
Firmware and design files for my second keyboard but now all in rust.

# Structure
/
|
+- rightboard/  # Firmware for right keyboard
|
+- leftboard/   # Firmware for left keyboard
|
+- scripts/
    |
    +- test_serial.py: Test rightboard COBS protocol.
    |
    +- test_tui.py: Equivalent of above test serial but GUI.
    |
    +- convert_to_raw.py: Convert bitmap into raw binary file.
    |
    +- load_keymap.py: Load keymap to keyboard.

# Example Script commands
Convert .bmp into raw file:
`uv run scripts/convert_to_raw.py src/capslock.bmp`

Push keymap to keyboard:
`uv run scripts/load_keymap.py [tty]`

Test rightboard COBS connected ttyUSB0:
`uv run scripts/test_serial.py /dev/ttyUSB0`

TUI for testing rightboard COBS across ttyACM1 bridge:
`uv run scripts/test_tui.py /dev/ttyACM1 -b`

# Installs for cargo-call-stack
## Install llvm
https://askubuntu.com/questions/1286131/how-do-i-install-llvm-10-on-ubuntu-18-04
goto: https://apt.llvm.org/

### Install commands
- sudo apt install libllvm21 llvm-21 llvm-21-dev llvm-21-doc llvm-21-examples llvm-21-runtime
- sudo apt install libpolly-21-dev
- sudo apt install libclang-common-21-dev
- sudo apt install libzstd-dev

Get version: $ llvm-config --version
Get bin path: llvm-config-21 --bindir

## Install cargo-call-stack
- Install cargo-call-stack from: https://github.com/Dirbaio/cargo-call-stack/
$ LLVM_SYS_170_PREFIX=/usr/lib/llvm-21/ cargo +stable install --git https://github.com/Dirbaio/cargo-call-stack

## Run tool
$ cargo build --release
$ cargo-call-stack -i target/thumbv7em-none-eabi/release/leftboard --target "thumbv7em-none-eabi" > cg.dot
