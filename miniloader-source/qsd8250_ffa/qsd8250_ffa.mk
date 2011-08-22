PRODUCT_PACKAGES := \
    IM \
    VoiceDialer \
    SdkSetup


$(call inherit-product, build/target/product/generic_with_google.mk)

#Enabling Ring Tones
include frameworks/base/data/sounds/OriginalAudio.mk

# Overrides
PRODUCT_NAME := qsd8250_ffa
PRODUCT_DEVICE := qsd8250_ffa
