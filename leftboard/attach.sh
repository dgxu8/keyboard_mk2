#!/bin/bash

TARGET=target/thumbv7em-none-eabi/debug/leftboard
socat /dev/ttyUSB0,b2000000,raw,echo=0 STDOUT | defmt-print -e $TARGET
