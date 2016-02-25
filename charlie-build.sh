#! /bin/bash

make distclean

make DEBUG=1 PLAT=axxia bl31 ; ${CROSS_COMPILE}objcopy -I binary -O elf64-littleaarch64 -B aarch64 --rename-section .data=.monitor build/axxia/debug/bl31.bin build/axxia/debug/bl31.o 
#; make PLAT=axxia bl31 ; ${CROSS_COMPILE}objcopy -I binary -O elf64-littleaarch64 -B aarch64 --rename-section .data=.monitor build/axxia/release/bl31.bin build/axxia/release/bl31.o

