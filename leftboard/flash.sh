#!/bin/bash

set -ep

cargo objcopy --bin leftboard --release -- -O binary target/leftboard.bin
STM32_Programmer_CLI -c port=USB1 -w target/leftboard.bin 0x08000000
