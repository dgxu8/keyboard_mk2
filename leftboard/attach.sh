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

echo -en '\x01' > $TTY
defmt-print --show-skipped-frames -e $ELF serial --path $TTY
