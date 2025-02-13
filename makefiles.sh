#!/bin/bash
find . -name "*.c" > neshto.files
find . -name "*.h" >> neshto.files
echo "Makefile" >> neshto.files
echo "./Core/Inc/" > neshto.includes
echo "./Drivers/STM32F0xx_HAL_Driver/Inc/" >> neshto.includes
echo "./Drivers/CMSIS/Device/ST/STM32F0xx/Include/" >> neshto.includes



