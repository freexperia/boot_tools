/*
 * Copyright (C) 2008 The Android Open Source Project
 * All rights reserved.
 * Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the 
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED 
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <boot/boot.h>
#include <boot/flash.h>
#include <boot/board.h>

#include <boot/bootimg.h>
#include <boot/tags.h>

#include <boot/gpio.h>
#include <boot/boardconfig.h>
#define VERSION "0.5"

#define REQUIRE_SIGNATURE 0

#if REQUIRE_SIGNATURE
unsigned key_engineering[2 + 64 + 64] = {
    64,0x5b022317,-60769447,648742897,-13657530,585562035,591851935,
    454860199,-1809625305,1868200692,-155297008,-1688439840,-1333607631,
    -483027189,-2051438457,1030069735,819944365,2133377257,-1978924214,
    2109678622,1974978919,-1811463608,765849268,1984092281,921245328,
    -1055062768,1487475997,1209618652,871985152,-611178965,-2057018571,
    335641539,-1196119550,1550548229,-356223887,1909799623,1281016007,
    957001635,1005656532,-1027634024,-1576447610,-1917246637,589192795,
    -1137386186,-1958135372,1933245070,64958951,-1820428322,-1577697840,
    1824253519,555306239,-1588272058,-1925773018,1205934271,-836584444,
    -1140961670,-185198349,1293769947,37045923,1516796974,-297288651,
    651582073,-1337054592,-543971216,-1706823885,-1040652818,-594113104,
    260093481,-1277656496,56493468,1577037283,773995876,244894933,
    -2075797967,783894843,880611008,-1433369702,380946504,-2081431477,
    1377832804,2089455451,-410001201,1245307237,-1228170341,-2062569137,
    -1327614308,-1671042654,1242248660,-418803721,40890010,-1806767460,
    -1468529145,-1058158532,1243817302,-527795003,175453645,-210650325,
    -827053868,-571422860,886300657,2129677324,846504590,-1413102805,
    -1287448511,-1991140134,56194155,1375685594,-129884114,1393568535,
    -1098719620,-935279550,1717137954,-1782544741,272581921,-669183778,
    584824755,1434974827,-1122387971,-810584927,-2147338547,-937541680,
    -313561073,5506366,-1594059648,-1744451574,1896015834,1496367069,
    1742853908,508461291,1905056764
};
#endif


const char *board_cmdline(void);
void memdump(const unsigned int* buf, int size);
void txtdump(const char* buf, int size);
void dump_smem_info(void);
void dump_smem_info_n(int item);

static const char *mystrchr(const char*str, int ch){
    if(!str) return 0;
    while(*str != ch){
        if(*str++ == 0) return 0;
    }
    return str;
}

const char *get_fastboot_version(void)
{
    return VERSION;
}

unsigned linux_type = 0;
unsigned linux_tags = 0;

unsigned ramdisk_addr = RAMDISK_ADDR;
unsigned ramdisk_size = 0;
unsigned kernel_addr = KERNEL_ADDR;
unsigned kernel_size = 0;

volatile int spinning = 0;

static void fixup_tags(unsigned *tags, unsigned *out, const char *cmdline)
{
    unsigned *newtags = (unsigned *) NEWTAGS_ADDR;
    unsigned *np = newtags;
    unsigned n;
    char *oldcmdline = "";
    
    if(cmdline == 0) cmdline = "";

        /* CORE */
    *np++ = 2;
    *np++ = 0x54410001;

    if(tags != 0) {
        while(*tags) {
            if(tags[1] == 0x54410001) {
                    /* skip core tag */
                tags += tags[0];
            } else if((tags[1] == 0x54420005) && (ramdisk_size != 0)) {
                    /* skip ramdisk if we have one of our own */
                tags += tags[0];
            } else if((tags[1] == 0x54410009) && (cmdline[0])) {
                    /* skip existing cmdline so we can replace it */
                oldcmdline = (char*) (tags + 2);
                tags += tags[0];
            } else {
                    /* copy any unknown tags */
                n = tags[0];
                while(n-- > 0) {
                    *np++ = *tags++;
                }
            }
        }
    }

        /* create a ramdisk tag if we need to */
    if(ramdisk_size) {
        *np++ = 4;
        *np++ = 0x54420005;
        *np++ = ramdisk_addr;
        *np++ = ramdisk_size;
    }

//    dprintf("oldcmdline: '%s'\n", oldcmdline);
//    dprintf("cmdline: '%s'\n", cmdline);
    
        /* create a cmdline tag if we need to */
    if(cmdline[0]) {
        int len;
        char *str = (char*) (np + 2);
        
        len = strlen(oldcmdline);
        if(len) {
            memcpy(str, oldcmdline, len);
            str += len;
            *str++ = ' ';
        }

        len = strlen(cmdline);
        memcpy(str, cmdline, len);
        str += len;
        *str++ = 0;
        
            /* length in words */
        len = ((str - ((char*) (np + 2))) + 3) / 4;

//        dprintf("CMDLINE: '%s'\n", ((char*) (np + 2)));
        
        *np++ = 2 + len;
        *np++ = 0x54410009;
        
        np += len;
    }

        /* add footer tag */
    *np++ = 0;
    *np++ = 0;
    
        /* copy it all back to the original tags area */
    while(newtags < np) {
        *out++ = *newtags++;
    }
}

static char cmdline[BOOT_ARGS_SIZE];
void mmu_off(void);

static void boot_linux(void)
{
    unsigned *tags = (unsigned*) TAGS_ADDR;
    int ret;
    int (*entry)(unsigned,unsigned,unsigned) = (void*) kernel_addr;

    if(linux_type == 0) {
        linux_type = board_machtype();
    }
    if(!cmdline[0]){
        strcpy(cmdline, board_cmdline());
    }
    
    fixup_tags((unsigned*) linux_tags, tags, cmdline);    
    DISPLAY_MSG("entry=%x, entry(%d, %d, %x)\n", entry, 0, linux_type, tags);
    ret = entry(0, linux_type, tags);
    mmu_off();
    DISPLAY_MSG("ok ret=%x\n", ret);
//goroh for re-entry boot-img loading
//    for(;;);
}

/* convert a boot_image at kernel_addr into a kernel + ramdisk + tags */
static int init_boot_linux(void)
{
    boot_img_hdr *hdr = (void*) kernel_addr;
    unsigned page_mask = 2047;
    unsigned kernel_actual;
    unsigned ramdisk_actual;
    unsigned second_actual;
    
    if((kernel_size < 4096) || memcmp(hdr->magic, BOOT_MAGIC, BOOT_MAGIC_SIZE)){
//        dprintf("bootimg: bad header\n");
        return -1;
    }

    if(hdr->page_size != 2048 &&  hdr->page_size != 4096) {
//        dprintf("bootimg: invalid page size\n");
        return -1;
    }

    kernel_actual = (hdr->kernel_size + page_mask) & (~page_mask);
    ramdisk_actual = (hdr->ramdisk_size + page_mask) & (~page_mask);
    second_actual = (hdr->second_size + page_mask) & (~page_mask);
    
    if(kernel_size != (kernel_actual + ramdisk_actual + second_actual + 2048)) {
//        dprintf("bootimg: invalid image size");
        return -1;
    }

        /* XXX process commandline here */
    if(hdr->cmdline[0]){
        hdr->cmdline[BOOT_ARGS_SIZE - 1] = 0;
        memcpy(cmdline, hdr->cmdline, BOOT_ARGS_SIZE);
    }

#if 0 // for direct boot zImage
        /* XXX how to validate addresses? */
    ramdisk_addr = hdr->magic + 2048 + kernel_actual;
    ramdisk_size = hdr->ramdisk_size;

    kernel_addr = hdr->magic + 2048;
    kernel_size = hdr->kernel_size;
    
//    cprintf("bootimg: kernel addr=%x size=%x\n",
//            kernel_addr, kernel_size);
//    cprintf("bootimg: ramdisk addr=%x size=%x\n",
//            ramdisk_addr, ramdisk_size);
    
    return 0; 
#else
        /* XXX how to validate addresses? */
    ramdisk_addr = hdr->ramdisk_addr;
    ramdisk_size = hdr->ramdisk_size;

    kernel_addr = hdr->kernel_addr;
    kernel_size = hdr->kernel_size;
    
//    cprintf("bootimg: kernel addr=%x size=%x\n",
//            kernel_addr, kernel_size);
//    cprintf("bootimg: ramdisk addr=%x size=%x\n",
//            ramdisk_addr, ramdisk_size);
    
    memcpy((void*) ramdisk_addr, 
           hdr->magic + 2048 + kernel_actual,
           ramdisk_size);
    
    memcpy((void*) kernel_addr,
           hdr->magic + 2048,
           kernel_size);
    
    return 0; 
#endif
}

static unsigned hex2unsigned(char *x)
{
    unsigned n = 0;

    while(*x) {
        switch(*x) {
        case '0': case '1': case '2': case '3': case '4':
        case '5': case '6': case '7': case '8': case '9':
            n = (n << 4) | (*x - '0');
            break;
        case 'a': case 'b': case 'c':
        case 'd': case 'e': case 'f':
            n = (n << 4) | (*x - 'a' + 10);
            break;
        case 'A': case 'B': case 'C':
        case 'D': case 'E': case 'F':
            n = (n << 4) | (*x - 'A' + 10);
            break;
        default:
            return n;
        }
        x++;
    }

    return n;
}

static void num_to_hex8(unsigned n, char *out)
{
    static char tohex[16] = "0123456789abcdef";
    int i;
    for(i = 7; i >= 0; i--) {
        out[i] = tohex[n & 15];
        n >>= 4;
    }
    out[8] = 0;
}

extern char serialno[];

static char signature[SIGNATURE_SIZE];


void boot_from_mem(unsigned int addr, int size)
{
    if(kernel_addr != addr)
        memcpy(kernel_addr, addr, size);
    kernel_size = size;
    if(init_boot_linux()) {
        DISPLAY_MSG("FAILinvalid boot image");
    }
//    DISPLAY_MSG("\nTrying to reset modem...\n");
	 smsm_ack_amss_crash();
//    DISPLAY_MSG("\nbooting linux...\n");
    mdelay(10);
    boot_linux();
    return;
}
