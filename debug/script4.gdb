set target-async on
set mem inaccessible-by-default off
#target extended-remote 10.15.0.3:3333
target extended-remote localhost:4242

file ../adc.elf 
load ../adc.hex
compare-sections
hbreak main




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


