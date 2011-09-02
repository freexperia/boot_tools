#!/bin/sh
cd ramdisk
find . | cpio -o -H newc | gzip > ../ramdisk.gz
cd ..
./mkbootimg --cmdline 'androidboot.hardware=es209ra vmalloc=280M g_android.product_id=0x312E console=ttyMSM0 semcandroidboot.serialno=FreeXperia semcandroidboot.startup=0x00000018 semcandroidboot.000008A2=583130' --kernel zImage --ramdisk ramdisk.gz --base 0x20000000 -o boot.img
