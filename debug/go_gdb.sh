#!/bin/bash
arm-none-eabi-gdb -iex "set auto-load safe-path /" -ex "set mi-async on" -ex "target extended-remote localhost:4242" -ex "monitor reset halt" -ex "monitor arm semihosting enable" -x ./.gdbinit -x ./script4.gdb ../main.elf 

