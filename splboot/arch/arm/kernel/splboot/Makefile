#
# Makefile for the linux kernel.
#

BUILDDIR:=$(PWD)
KERNELSRCDIR=$(BUILDDIR)/../../../..
VERBOSE = 0
ARCH=arm
CROSS_COMPILE=arm-linux-
EXTRA_CFLAGS=-I $(PWD)/include

obj-m		:= splboot.o

splboot-objs := splload.o proc-v7.o tlb-v7.o cache-v7.o abort-ev7.o mmu.o

all:
	make -C $(KERNELSRCDIR) ARCH=arm CROSS_COMPILE=arm-linux- SUBDIRS=$(BUILDDIR) KBUILD_VERBOSE=$(VERBOSE) modules

clean:

