LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)
LOCAL_SRC_FILES := mkheader.c
LOCAL_MODULE := mkheader-8x50
include $(BUILD_HOST_EXECUTABLE)

include $(CLEAR_VARS)
LOCAL_MODULE := appsboot.mbn
LOCAL_MODULE_PATH := $(PRODUCT_OUT)
LOCAL_MODULE_CLASS := EXECUTABLES
LOCAL_REQUIRED_MODULES := mkheader-8x50 usbloader
include $(BUILD_SYSTEM)/base_rules.mk
$(LOCAL_BUILT_MODULE): $(HOST_OUT_EXECUTABLES)/mkheader-8x50 \
                       $(PRODUCT_OUT)/usbloader
	@mkdir -p $(dir $@)
	@$(HOST_OUT_EXECUTABLES)/mkheader-8x50 $(PRODUCT_OUT)/usbloader $(dir $@)/appsboothd.mbn
	cat $(dir $@)/appsboothd.mbn $(PRODUCT_OUT)/usbloader > $@
