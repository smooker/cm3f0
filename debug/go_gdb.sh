#!/bin/bash
arm-none-eabi-gdb -iex "set auto-load safe-path /" -x ./.gdbinit -x ./script4.gdb ../adc.elf 

