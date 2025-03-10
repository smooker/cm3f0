#set mi-async on
set mem inaccessible-by-default off
#target extended-remote 10.15.0.3:3333
#target extended-remote localhost:4242

file ../main.elf 
load ../main.hex
#compare-sections

define lf
file ../main.elf
load ../main.hex
end

define pu
x/tw USART1_CR1
x/tw USART1_ISR
end

#c
#hbreak main
#watch arrf[0]

#rwatch arrf[0]
#info break
#del 3
#p/x *(uint32_t *)0x20000570
#x/8xb 0x20000570

#>>> p/t USART1_ISR
#$2 = 11000100000000011111000

#>>> x/tw USART1_ISR
#0x6200f8:	00000000011000100000000011111000


#hbreak Core/Src/main.c:258
#next
#watch huart1
#watch data[0]
#python sys.path.insert(1, '/home/smooker/src/qt-creator/share/qtcreator/debugger/')
#python sys.path.append('/home/smooker/src/stm32/gcc-arm-none-eabi-9-2020-q2-update/bin/data-directory/python')
#python from gdbbridge import *
#python theDumper.loadDumpers({"token":11})
#12-interpreter-exec console \"set target-async off\"
#show debug-file-directory
#set debug arm on
#show debug arm
#watch tmgw.rx.cf
#watch tmgw.rx.sf
#watch tmgw.rx.d0
#watch tmgw.rx.d1
#run
#monitor hard_srst


