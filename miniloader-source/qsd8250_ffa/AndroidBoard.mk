# Copyright (c) 2009, Code Aurora Forum.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

LOCAL_PATH := $(call my-dir)

ifeq ($(KERNEL_DEFCONFIG),)
    KERNEL_DEFCONFIG := qsd8650_defconfig
endif

include kernel/AndroidKernel.mk

file := $(INSTALLED_KERNEL_TARGET)
ALL_PREBUILT += $(file)
$(file) : $(TARGET_PREBUILT_KERNEL) | $(ACP)
	$(transform-prebuilt-to-target)

file := $(TARGET_OUT_KEYLAYOUT)/8k_ffa_keypad.kl
ALL_PREBUILT += $(file)
$(file) : $(LOCAL_PATH)/8k_ffa_keypad.kl | $(ACP)
	$(transform-prebuilt-to-target)

include $(CLEAR_VARS)
LOCAL_SRC_FILES := 8k_ffa_keypad.kcm
include $(BUILD_KEY_CHAR_MAP)

# to build the bootloader you need the common boot stuff,
# the architecture specific stuff, and the board specific stuff
include vendor/qcom/$(TARGET_PRODUCT)/boot/Android.mk
# include bootloader/legacy/Android.mk
