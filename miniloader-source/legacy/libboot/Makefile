CROSS_COMPILE=$(TOOLCHAIN)arm-eabi-
CC = $(CROSS_COMPILE)gcc
AR = $(CROSS_COMPILE)ar

SRC = flash.c poll.c tags_partition.c tags_revision.c tags_serialno.c tags_cmdline.c gpio_keypad.c init.c tags.c
OBJ = flash.o poll.o tags_partition.o tags_revision.o tags_serialno.o tags_cmdline.o gpio_keypad.o init.o tags.o

CFLAGS =  -I../include
CFLAGS += -c -O2 -g -W -Wall
CFLAGS += -march=armv6
CFLAGS += -DQCOM_SCORPION
CFLAGS += -fomit-frame-pointer -fstrict-aliasing
CFLAGS += -funswitch-loops -finline-limit=300
CFLAGS += -fno-asynchronous-unwind-tables -fno-unwind-tables -fno-exceptions

%.o: %.c
	$(CC) $(CFLAGS) $< -o $@

ARFLAGS = crs

libboot.a: $(OBJ)
	$(AR) $(ARFLAGS) $@ $^

.PHONY : clean
clean:
	rm -rf $(OBJ) libboot.a
