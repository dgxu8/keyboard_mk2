#!/bin/bash

CMD_TTY="/dev/ttyACM0"
PROGRAM_TTY="/dev/ttyACM1"

if [ -c $PROGRAM_TTY ] && [ -c $CMD_TTY ]; then
    # Make sure port isn't locked
    lsof +c0 /dev/ttyACM0 | grep defmt-print
    if [ $? -eq 0 ]; then
        echo "Something is connected to the command port, please top the program"
        exit 1
    fi
    echo -en "\x02" > $CMD_TTY
    sleep 1
else
    echo "ACM1 does not exist switching to using USB0"
    CMD_TTY=""
    PROGRAM_TTY="/dev/ttyUSB0"
    if [ ! -c $PROGRAM_TTY ]; then
        echo "USB0 also does not exist, exiting..."
        exit 1
    fi
fi

set -e

cargo build --release
cargo objcopy --bin rightboard --release -- -O binary target/rightboard.bin
STM32_Programmer_CLI -c "port=${PROGRAM_TTY}" -w target/rightboard.bin 0x08000000

if [ -n $CMD_TTY ]; then
    echo -en "\x03" > $CMD_TTY
fi

sleep 0.5

./attach.sh
