#!/bin/bash

STM32_DFU="0483:df11"
KEYBOARD_ID="c0de:cafe"
COM_TTY="/dev/ttyACM0"
OPTION_UPDATE=""

poll_for_id() {
    local id=$1
    while : ; do
        lsusb -d $id > /dev/null
        [[ $? -eq 0 ]] && break
    done
}
export -f poll_for_id

fixup_options() {
    timeout 1 bash -c "poll_for_id $KEYBOARD_ID"
    if [ $? -ne 0 ]; then
        echo "Updating option bytes..."
        poll_for_id $STM32_DFU
        STM32_Programmer_CLI -c port=USB1 -ob nBOOT0=1  -ob nSWBOOT0=1 > /dev/null
    fi
}

lsusb -d $STM32_DFU
if [ $? -ne 0 ]; then
    if [ ! -c $COM_TTY ]; then
        echo "No device found to update"
        exit 1
    fi
    OPTION_UPDATE="set"
    echo -en  '\x01' > $COM_TTY
    # sudo usbreset $KEYBOARD_ID > /dev/null
    poll_for_id $STM32_DFU
fi

set -ep

cargo objcopy --bin leftboard --release -- -O binary target/leftboard.bin
STM32_Programmer_CLI -c port=USB1 -w target/leftboard.bin 0x08000000 -s

set +e
if [ -n $OPTION_UPDATE ]; then
    fixup_options &
fi
echo ""

./attach.sh

# STM32_Programmer_CLI -l usb
# TODO: add a way to parse the USB with the list command above
