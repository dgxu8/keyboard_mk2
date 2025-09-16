#!/bin/bash

ELF="target/thumbv7em-none-eabi/release/leftboard"
TTY="/dev/ttyACM0"

if [[ -n $1 ]]; then
    ELF="$1"
fi

if [[ -n $2 ]]; then
    TTY="$2"
fi

echo "Connecting to $TTY - $ELF"
while [ ! -c $TTY ] ; do
    sleep 0.1
done

sleep 0.5

enable_defmt_out() {
    sleep 0.2  # Add a delay so socat is up
    echo -en '\x00' > $TTY
}

enable_defmt_out &

# This locks out the serial port even though it only reads out of the port
# defmt-print --show-skipped-frames -e $ELF serial --path $TTY
socat ${TTY},b115200,raw,echo=0 STDOUT | defmt-print --show-skipped-frames -e $ELF
