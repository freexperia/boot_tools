LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)

LOCAL_ARM_MODE := arm

LOCAL_SRC_FILES := \
	clock.c \
	hsusb.c \
	mddi_console.c \
	nand.c \
	uart.c \
	gpio.c \
	mddi.c \
	vic.c \
	shared.c \
	smem.c \
	smem_ptable.c

LOCAL_C_INCLUDES := $(call include-path-for, bootloader)

LOCAL_CFLAGS := -O2 -g -W -Wall
LOCAL_CFLAGS += -march=armv6

ifneq (, $(filter msm7627_surf msm7627_ffa, $(TARGET_PRODUCT)))
  LOCAL_CFLAGS += -DSURF7X2X
endif

LOCAL_MODULE := libboot_arch_msm7k

include $(BUILD_RAW_STATIC_LIBRARY)
