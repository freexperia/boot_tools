CROSS_COMPILE=$(TOOLCHAIN)arm-eabi-
CC = $(CROSS_COMPILE)gcc
AR = $(CROSS_COMPILE)ar

SRC = board.c panel.c keypad.c
OBJ = board.o panel.o keypad.o

CFLAGS =  -I../include -I../../legacy/include
CFLAGS += -c -O2 -g -W -Wall
CFLAGS += -march=armv6
CFLAGS += -DQCOM_SCORPION
CFLAGS += -fomit-frame-pointer -fstrict-aliasing
CFLAGS += -funswitch-loops -finline-limit=300
CFLAGS += -fno-asynchronous-unwind-tables -fno-unwind-tables -fno-exceptions

%.o: %.c
	$(CC) $(CFLAGS) $< -o $@

ARFLAGS = crs

libboot_board_qsd8250_ffa.a: $(OBJ)
	$(AR) $(ARFLAGS) $@ $^

.PHONY : clean
clean:
	rm -rf $(OBJ) libboot_board_qsd8250_ffa.a
