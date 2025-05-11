set -e
cargo build --release
cargo objcopy --bin rightboard --release -- -O binary target/rightboard.bin
STM32_Programmer_CLI -c port=/dev/ttyUSB0 -w target/rightboard.bin 0x08000000
