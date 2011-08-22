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

# config.mk
#
# Product-specific compile-time definitions.
#
ifeq ($(QC_PROP),true)
    
    BOARD_USES_QCOM_HARDWARE := true
    BOARD_USES_QCOM_8x_CHIPSET := true
    BOARD_USES_ADRENO_200 := true
    BOARD_USE_QCOM_TESTONLY := true

    ifeq ($(BUILD_TINY_ANDROID), true)
    subdirs += \
      vendor/qcom-proprietary/diag \
      vendor/qcom-proprietary/kernel-tests \
      vendor/qcom-proprietary/modem \
      vendor/qcom-proprietary/oncrpc
    else
    BOARD_GPS_LIBRARIES := libloc_api
    BOARD_HAVE_BLUETOOTH := true
    BOARD_CAMERA_LIBRARIES := libcamera
    endif  # BUILD_TINY_ANDROID

    BOARD_USES_QCOM_HARDWARE := true
    BOARD_USES_QCOM_8x_CHIPSET := true
    BOARD_USES_ADRENO_200 := true
    BOARD_USE_QCOM_TESTONLY := true

else
    BOARD_USES_GENERIC_AUDIO := true
    USE_CAMERA_STUB := true

endif # QC_PROP

TARGET_NO_BOOTLOADER := true
TARGET_NO_KERNEL := false
TARGET_NO_RADIOIMAGE := true

TARGET_BOOTLOADER_LIBS := \
	libboot_board_qsd8250_ffa \
	libboot_arch_qsd8k \
	libboot_arch_armv6

TARGET_BOOTLOADER_LINK_SCRIPT := \
	vendor/qcom/$(TARGET_PRODUCT)/boot/boot.ld

BOARD_KERNEL_CMDLINE := mem=160M console=ttyMSM2,115200n8 androidboot.hardware=qcom
