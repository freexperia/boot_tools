LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)

LOCAL_ARM_MODE := arm

LOCAL_SRC_FILES := board.c panel.c keypad.c

LOCAL_C_INCLUDES := bootable/bootloader/legacy/include

LOCAL_CFLAGS := -O2 -g -W -Wall
LOCAL_CFLAGS += -march=armv6

LOCAL_MODULE := libboot_board_qsd8250_ffa

include $(BUILD_RAW_STATIC_LIBRARY)

include $(LOCAL_PATH)/tools/Android.mk
