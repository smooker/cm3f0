#!/bin/bash
export OPENCM3_DIR="../libopencm3"

arm-none-eabi-gcc -mcpu=cortex-m0 -msoft-float -dM -E - < /dev/null >  cm3smooker.h
make $@ V=1

# to include /usr/arm-none-eabi/include/
