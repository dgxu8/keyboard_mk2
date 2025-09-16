#!/bin/bash

ELF="target/thumbv6m-none-eabi/release/rightboard"
TTY="/dev/ttyACM1"
CTRL_TTY="/dev/ttyACM0"

if [[ -n $1 ]]; then
    ELF="$1"
fi

if [[ -n $2 ]]; then
    TTY="$2"
fi

if [[ -n $3 ]]; then
    CTRL_TTY="$3"
fi

if [ ! -f $ELF ] || [ ! -c $TTY ] || [ ! -c $CTRL_TTY ]; then
    echo "Not all inputs valid:"
    echo "${ELF}, ${TTY}, ${CTRL_TTY}"
    exit 1
fi

echo "Connecting to $TTY - $ELF w/ ctl: $CTRL_TTY"
while [ ! -c $TTY ] ; do
    sleep 0.1
done

sleep 0.5

enable_defmt_out() {
    sleep 0.2  # Add a delay so socat is up
    echo -en '\x06' > $CTRL_TTY
}

enable_defmt_out &

defmt-print --show-skipped-frames -e $ELF serial --path $TTY
