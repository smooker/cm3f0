#!/bin/bash
export OPENCM3_DIR="../libopencm3"

make $@ V=1

# to include /usr/arm-none-eabi/include/
