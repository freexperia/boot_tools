CROSS_COMPILE=$(TOOLCHAIN)arm-eabi-
CC = $(CROSS_COMPILE)gcc
AR = $(CROSS_COMPILE)ar

ASM_SRC = sc_sa_fuse_ro.S
ASM_OBJ = sc_sa_fuse_ro.o

SRC = clock.c gpio.c hsusb.c mddi.c mddi_console.c nand.c shared.c smem.c smem_ptable.c uart.c vic.c
OBJ = clock.o gpio.o hsusb.o mddi.o mddi_console.o nand.o shared.o smem.o smem_ptable.o uart.o vic.o
#SRC = clock.c gpio.c hsusb.c lcdc.c mddi_console.c nand.c shared.c smem.c smem_ptable.c uart.c vic.c
#OBJ = clock.o gpio.o hsusb.o lcdc.o mddi_console.o nand.o shared.o smem.o smem_ptable.o uart.o vic.o

CFLAGS =  -I../include
CFLAGS += -c -O2 -g -W -Wall
CFLAGS += -march=armv6
CFLAGS += -DQCOM_SCORPION -DSURF8K
CFLAGS += -DENABLE_PROC_COMM

%.o: %.S
	$(CC) $(CFLAGS) $< -o $@

CFLAGS += -fomit-frame-pointer -fstrict-aliasing
CFLAGS += -funswitch-loops -finline-limit=300
CFLAGS += -fno-asynchronous-unwind-tables -fno-unwind-tables -fno-exceptions

%.o: %.c
	$(CC) $(CFLAGS) $< -o $@

ARFLAGS = crs

libboot_arch_qsd8k.a: $(ASM_OBJ) $(OBJ)
	$(AR) $(ARFLAGS) $@ $^

.PHONY : clean
clean:
	rm -rf $(ASM_OBJ) $(OBJ) libboot_arch_qsd8k.a
