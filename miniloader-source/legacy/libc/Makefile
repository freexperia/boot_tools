CROSS_COMPILE=$(TOOLCHAIN)arm-eabi-
CC = $(CROSS_COMPILE)gcc
AR = $(CROSS_COMPILE)ar

SRC = dprintf.c xprintf.c cprintf.c sprintf.c malloc.c memcmp.c memcpy.c memset.c strcpy.c strlen.c strcmp.c strstr.c crypto.c rsa.c sha.c
OBJ = dprintf.o xprintf.o cprintf.o sprintf.o malloc.o memcmp.o memcpy.o memset.o strcpy.o strlen.o strcmp.o strstr.o crypto.o rsa.o sha.o

CFLAGS =  -I../include
CFLAGS +=  -I.
CFLAGS +=  -I../../core/include/mincrypt
CFLAGS += -c -O2 -g -W -Wall
CFLAGS += -march=armv6
CFLAGS += -DQCOM_SCORPION
CFLAGS += -fomit-frame-pointer -fstrict-aliasing
CFLAGS += -funswitch-loops -finline-limit=300
CFLAGS += -fno-asynchronous-unwind-tables -fno-unwind-tables -fno-exceptions

%.o: %.c
	$(CC) $(CFLAGS) $< -o $@

ARFLAGS = crs

libboot_c.a: $(OBJ)
	$(AR) $(ARFLAGS) $@ $^

.PHONY : clean
clean:
	rm -rf $(OBJ) libboot_c.a
