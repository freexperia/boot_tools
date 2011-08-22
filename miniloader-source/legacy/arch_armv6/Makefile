CROSS_COMPILE=$(TOOLCHAIN)arm-eabi-
CC = $(CROSS_COMPILE)gcc
AR = $(CROSS_COMPILE)ar

ASM_SRC = dcc.S irq.S jtag.S misc.S
ASM_OBJ = dcc.o irq.o jtag.o misc.o

CFLAGS =  -I../include
CFLAGS += -c -O2 -g -W -Wall
CFLAGS += -march=armv6
CFLAGS += -DQCOM_SCORPION -DSURF8K

%.o: %.S
	$(CC) $(CFLAGS) $< -o $@

ARFLAGS = crs

libboot_arch_armv6.a: $(ASM_OBJ)
	$(AR) $(ARFLAGS) $@ $^

.PHONY : clean
clean:
	rm -rf $(ASM_OBJ) libboot_arch_armv6.a
