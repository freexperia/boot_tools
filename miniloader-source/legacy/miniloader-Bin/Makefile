CROSS_COMPILE=$(TOOLCHAIN)arm-eabi-
CC = $(CROSS_COMPILE)gcc
AS = $(CROSS_COMPILE)gcc
AR = $(CROSS_COMPILE)ar
LD = $(CROSS_COMPILE)ld
OBJ_COPY = $(CROSS_COMPILE)objcopy

ASM_SRC = init.S
ASM_OBJ = init.o

SRC = main.c miniloader.c
OBJ = main.o miniloader.o

CFLAGS =  -I../include
CFLAGS += -I../include/boot
CFLAGS += -I../../core/mkbootimg
CFLAGS += -I../../qsd8250_ffa
CFLAGS += -c -O2 -g -W -Wall
CFLAGS += -march=armv6
CFLAGS += -DPRODUCTNAME='"ES209RA"'
CFLAGS += -DUSE_SMEM
#CFLAGS += -DQCOM_RELOCATE_TO_ZERO
CFLAGS += -DQCOM_SCORPION -DSURF8K

%.o: %.S
	$(CC) $(CFLAGS) $< -o $@

CFLAGS += -fomit-frame-pointer -fstrict-aliasing
CFLAGS += -funswitch-loops -finline-limit=300
CFLAGS += -fno-asynchronous-unwind-tables -fno-unwind-tables -fno-exceptions

%.o: %.c
	$(CC) $(CFLAGS) $< -o $@

miniloader: $(ASM_OBJ) $(OBJ)
	$(LD) --script ../../qsd8250_ffa/boot/bootmini.ld \
		-o miniloader.elf \
		init.o main.o miniloader.o \
		--start-group ../../qsd8250_ffa/boot/libboot_board_qsd8250_ffa.a ../arch_qsd8k/libboot_arch_qsd8k.a ../arch_armv6/libboot_arch_armv6.a ../libboot/libboot.a ../libc/libboot_c.a --end-group \
		`$(CC) -mthumb-interwork -print-libgcc-file-name`
	$(OBJ_COPY) -O binary miniloader.elf miniloader

.PHONY : clean
clean:
	rm -rf $(ASM_OBJ) $(OBJ) miniloader.elf miniloader
