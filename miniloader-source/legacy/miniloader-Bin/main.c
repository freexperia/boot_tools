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
#include <boot/uart.h>
#include <boot/tags.h>
#include <boot/flash.h>
#include <boot/board.h>
#include <qsd8k/mddi.h>
#include <qsd8k/vic.h>
#include <qsd8k/irqs.h>
#include <qsd8k/smem.h>
#include <bootimg.h>
#include <boot/boardconfig.h>
#include <qsd8k/smd_private.h>

#define FLASH_PAGE_SIZE 4096
#define FLASH_PAGE_BITS 11
#define ADDR_TAGS    TAGS_ADDR

#ifdef SURF7X30
#define EBI1_SIZE_198M 0xC600000
#define EBI1_SIZE_123M 0x7B00000
#define EBI1_SIZE_75M  0x4B00000
#define EBI1_ADR1    PHYSICAL_DRAM_BASE
#define EBI1_ADR2    0x40000000
#endif

#ifdef SURF8K
#if 0
#define EBI1_ADR1    0x00000000
#define EBI1_SIZE1   0x0E000000 //224MB for 1024MB contiguous RAM on ST1.5
#define EBI1_ADR2    0x10000000
#define EBI1_SIZE2   0x30000000
#else
#define EBI1_ADR1    0x20000000
#define EBI1_SIZE1   0x0E000000 //224MB
#define EBI1_ADR2    0x30000000
#define EBI1_SIZE2   0 // 0x07f00000 //127MB
#endif
#endif

#define RECOVERY_MODE 0x77665502
#define FASTBOOT_MODE 0x77665500

int raise(int sig) {
      return 0;
}

void boot_from_mem(unsigned int addr, int size);
void uart_putc(unsigned);


extern unsigned linux_type;
extern unsigned linux_tags;

static unsigned revision = 0;

char serialno[32];

void dump_smem_info(void);

#define SMEM_BUILD_ID_LOCATION	137
#define SMEM_BUILD_ID_LENGTH	32

struct smem_build_id
{
    unsigned format;
    unsigned msm_id;
    unsigned msm_version;
    unsigned build_id[SMEM_BUILD_ID_LENGTH];
};


static loader_param* get_loader_param(void)
{
    return (loader_param*)0x300ff000;
}

static void print_modem_build_id( void )
{
    struct smem_build_id *modem_build_id;
    unsigned int build_id_struct_len=0;
    unsigned msm_version_major = 0, msm_version_minor = 0;

#ifdef USE_SMEM
    modem_build_id =
           (struct smem_build_id*)smem_get_entry( SMEM_BUILD_ID_LOCATION,
                                               &build_id_struct_len );
    cprintf("modem_build_id = %x\n", modem_build_id);

    if( modem_build_id && build_id_struct_len)
    {
        DISPLAY_MSG("\nMSM Id: %d\n", modem_build_id->msm_id);
        msm_version_major = (modem_build_id->msm_version >> 16 ) & 0xff;
        msm_version_minor = modem_build_id->msm_version & 0xff;
        DISPLAY_MSG("MSM Version: %d.%d\n", msm_version_major, msm_version_minor);
        DISPLAY_MSG("Modem Build Id: %s\n", modem_build_id->build_id);
    }
#endif

}

int display_initialized = 0;

void txtdump(const char* buf, int size)
{
    int i;
    console_flush_enable(0);
    for(i=0; i<size; i++){
        console_putc(*buf++);
    }
    console_flush_enable(1);
    console_putc('\n');
    console_flush();
}

void memdump(const unsigned int* buf, int size)
{
    unsigned int addr = ((unsigned int)buf) & ~3;
    int i;
    console_flush_enable(0);
    for(i=0; i<size; i++){
        if(i % 4 == 0) cprintf("%x:", addr + (i<<2));
        cprintf(" %x", buf[i]);
        if(i % 4 == 3) cprintf("\n");
    }
    console_flush_enable(1);
    cprintf("\n");
}

void display_init( void )
{
    if(!display_initialized)
    {
        volatile loader_param* param = get_loader_param();
        console_init();
        display_initialized = 1;
        console_flush_enable(0);
        cprintf("Machine ID:    %d v%d\n", linux_type, revision);
        cprintf("Build Date:    "__DATE__", "__TIME__"\n");
        print_modem_build_id();
        if(param->serialno[0]){
            strcpy(serialno, &param->serialno);
        }
        cprintf("Serial Number: %s\n\n", serialno[0] ? serialno : "UNKNOWN");
        console_flush_enable(1);
        console_flush();
        
        flash_dump_ptn();
   }
}

static void display_versions( void )
{
    volatile loader_param* param = get_loader_param();
    console_flush_enable(0);
    cprintf("Parameter Boot Image Addr: %x\n", param->bootimgaddr);
    cprintf("Parameter Boot Image Size: %x\n", param->bootimgsize);
    cprintf("Parameter Machine Type: %d\n", param->machine_type);
    cprintf("Parameter Boot Param: %s\n", param->bootparam);
#if 0
    cprintf("Parameter addr: %x\n", param);
    cprintf("Parameter Version: %d\n", param->version);
    cprintf("Parameter Size: %d\n", param->param_size);
    cprintf("Parameter Mddi Addr: %x\n", param->mddiaddr);
    cprintf("Parameter Serial Number: %s\n\n", param->serialno);
#endif
    console_flush_enable(1);
    console_flush();
}

static volatile int spinning = 1;

void qpst_splash( void )
{
    if(!display_initialized)
    {
        console_init();
        display_initialized = 1;
   }
   cprintf("\n\nQPST download mode\n");
   while (spinning) {
   }
}

int check_for_1gb(void)
{
    unsigned volatile *adr1 = (unsigned *) 0x57000000;
    unsigned volatile *adr2 = (unsigned *) 0x5F000000;
    unsigned value1 = 0x55555555;
    unsigned value2 = 0xAAAAAAAA;

    *adr1 = value1;
    *adr2 = value2;

    if((value1 == *adr1) && (value2 == *adr2))
      return 1;
    else
      return 0;
}

static void create_atags(unsigned taddr, const char *cmdline,
                         unsigned raddr, unsigned rsize)
{
    unsigned n = 0;
    unsigned pcount;
    unsigned *tags = (unsigned *) taddr;

    struct smem_build_id *modem_build_id;
    unsigned int build_id_struct_len=0;
    char *build_type;
    int lpddr2 = 0;

    // ATAG_CORE
    tags[n++] = 2;
    tags[n++] = 0x54410001;

#ifdef SURF7X30
    // ATAG_MEM
    // Runtime detection of board type using build id as different hardware id 
    // is not available at this time.
    #ifdef USE_SMEM
    modem_build_id =
           (struct smem_build_id*)smem_get_entry( SMEM_BUILD_ID_LOCATION,
                                               &build_id_struct_len );
    if( modem_build_id && build_id_struct_len)
    {
        build_type  = (char *)(modem_build_id->build_id) + 8;
        if (*build_type == 'A')
        {
            lpddr2 = 1;
        }
    }
    #endif

    if (!lpddr2){
        // 198M setup
        tags[n++] = 4;
        tags[n++] = 0x54410002;
        tags[n++] = EBI1_SIZE_198M;
        tags[n++] = EBI1_ADR1;
    }else{
        // 123M + 75M setup
        tags[n++] = 4;
        tags[n++] = 0x54410002;
        tags[n++] = EBI1_SIZE_123M;
        tags[n++] = EBI1_ADR1;

        tags[n++] = 4;
        tags[n++] = 0x54410002;
        tags[n++] = EBI1_SIZE_75M;;
        tags[n++] = EBI1_ADR2;
    }

#endif

#ifdef SURF8K
    // ATAG_MEM
    tags[n++] = 4;
    tags[n++] = 0x54410002;
    tags[n++] = EBI1_SIZE1;
    tags[n++] = EBI1_ADR1;

    if (EBI1_SIZE2 > 0) {
        // ATAG_MEM
        tags[n++] = 4;
        tags[n++] = 0x54410002;
        tags[n++] = EBI1_SIZE2;
        tags[n++] = EBI1_ADR2;
    }
#endif

    if(rsize) {
        // ATAG_INITRD2
        tags[n++] = 4;
        tags[n++] = 0x54420005;
        tags[n++] = raddr;
        tags[n++] = rsize;
    }

    if((pcount = flash_get_ptn_count())){
        ptentry *ptn;
        unsigned pn;
        unsigned m = n + 2;

        for(pn = 0; pn < pcount; pn++) {
            ptn = flash_get_ptn(pn);
            memcpy(tags + m, ptn, sizeof(ptentry));
            m += (sizeof(ptentry) / sizeof(unsigned));
        }
        
        tags[n + 0] = m - n;
        tags[n + 1] = 0x4d534d70;
        n = m;
    }
    if(cmdline && cmdline[0]) {
        const char *src;
        char *dst;
        unsigned len = 0;
        
        dst = (char*) (tags + n + 2);
        src = cmdline;
        while((*dst++ = *src++)) len++;
        
        len++;
        len = (len + 3) & (~3);

            // ATAG_CMDLINE
        tags[n++] = 2 + (len / 4);
        tags[n++] = 0x54410009;

        n += (len / 4);
    }
    
        // ATAG_NONE
    tags[n++] = 0;
    tags[n++] = 0;
}

static void boot_linux(unsigned kaddr)
{
    void (*entry)(unsigned,unsigned,unsigned) = (void*) kaddr;

    entry(0, board_machtype(), ADDR_TAGS);
}

unsigned char raw_header[4096];
static int boot_into_recovery = 0;
static int boot_from_flash = 1;

void check_reboot_reason()
{
#ifdef USE_SMEM
#define SMEM_HW_RESET_DETECT 4
#define HW_RESET_SPLASH_DLOAD_MAGIC1 0xFCDE8462
#define AMSS_DLOAD_CHORD_SET_MAGIC   0xFFFFFFFF
    unsigned *reason = 0;
    unsigned reason_len=0;

    reason = (unsigned *)smem_get_entry( SMEM_HW_RESET_DETECT,&reason_len);
    if(reason && reason_len){
        if ((*reason == HW_RESET_SPLASH_DLOAD_MAGIC1) ||
            (*reason == AMSS_DLOAD_CHORD_SET_MAGIC)) {
            qpst_splash();
        }
    }

    reason = (unsigned *)smem_get_entry( SMEM_APPS_BOOT_MODE,&reason_len);
    if( reason && reason_len){
        if (*reason == RECOVERY_MODE){
            boot_into_recovery = 1;
        }else if(*reason == FASTBOOT_MODE){
            boot_from_flash = 0;
        }
    }
#endif

}

int boot_linux_from_flash(void)
{
    boot_img_hdr *hdr = (void*) raw_header;
    unsigned n;
    ptentry *p;
    unsigned offset = 0;
    const char *cmdline;

    if (!boot_into_recovery)
    {
        if((p = flash_find_ptn("boot")) == 0) {
            DISPLAY_MSG("NO BOOT PARTITION\n");
            return -1;
        }
    }
    else
    {
        if((p = flash_find_ptn("recovery")) == 0) {
            DISPLAY_MSG("NO RECOVERY PARTITION\n");
            return -1;
        }
    }

    if(flash_read(p, offset, raw_header, FLASH_PAGE_SIZE)) {
        DISPLAY_MSG("CANNOT READ BOOT IMAGE HEADER\n");
        return -1;
    }
    offset += FLASH_PAGE_SIZE;
    
    if(memcmp(hdr->magic, BOOT_MAGIC, BOOT_MAGIC_SIZE)) {
        DISPLAY_MSG("INVALID BOOT IMAGE HEADER\n");
        return -1;
    }

    n = (hdr->kernel_size + (FLASH_PAGE_SIZE - 1)) & (~(FLASH_PAGE_SIZE - 1));
    if(flash_read(p, offset, (void*) hdr->kernel_addr, n)) {
        DISPLAY_MSG("CANNOT READ KERNEL IMAGE\n");
        return -1;
    }
    offset += n;

    n = (hdr->ramdisk_size + (FLASH_PAGE_SIZE - 1)) & (~(FLASH_PAGE_SIZE - 1));
    if(flash_read(p, offset, (void*) hdr->ramdisk_addr, n)) {
        DISPLAY_MSG("CANNOT READ RAMDISK IMAGE\n");
        return -1;
    }
    offset += n;
    
    dprintf("\nkernel  @ %x (%d bytes)\n", hdr->kernel_addr, hdr->kernel_size);
    dprintf("ramdisk @ %x (%d bytes)\n\n\n", hdr->ramdisk_addr, hdr->ramdisk_size);

    if(hdr->cmdline[0]) {
        cmdline = (char*) hdr->cmdline;
    } else {
        cmdline = board_cmdline();
        if(cmdline == 0) {
            cmdline = "mem=50M console=null";
        }
    }

    DISPLAY_MSG("cmdline = '%s'\n", cmdline);
    
    DISPLAY_MSG("\nBooting Linux\n");

    create_atags(ADDR_TAGS, cmdline,
                 hdr->ramdisk_addr, hdr->ramdisk_size);
    
    boot_linux(hdr->kernel_addr);
    return 0;
}


static void tag_dump(unsigned tag, void *data, unsigned sz, void *cookie)
{
    dprintf("tag type=%x data=%x size=%x\n", tag, (unsigned) data, sz);
}

static struct tag_handler tag_dump_handler = {
    .func = tag_dump,
    .type = 0,
};

void xdcc_putc(unsigned x)
{
    while (dcc_putc(x) < 0) ;
}

#define SERIALNO_STR "semcandroidboot.serialno="
#define SERIALNO_LEN strlen(SERIALNO_STR)

static int tags_okay(unsigned taddr)
{
    unsigned *tags = (unsigned*) taddr;

    if(taddr != ADDR_TAGS) return 0;
    if(tags[0] != 2) return 0;
    if(tags[1] != 0x54410001) return 0;

    return 1;
}

volatile int polling = 1;
void mmu_off(void);
void regs_save(void);
int get_vecflag(void);
void clear_vecflag(void);
void* get_regs_int(void);

static const char* get_vecname(void)
{
    switch(get_vecflag()){
    case 1: return "Undefined";
    case 2: return "SWI";
    case 3: return "prefetch abort";
    case 4: return "data abort";
    case 5: return "reserved";
    case 6: return "IRQ";
    case 7: return "FIQ";
    }
    return "";
}

#define MSM_CSR_BASE 0xAC100000
#define MSM_TRIG_A2M_INT(n) (writel(1, MSM_CSR_BASE + 0x400 + (n) * 4))

void dump_smem_version_info(void);
void dump_smem_alloc_tbl(void);
void dump_smem_state(void);
void dump_smem_ch(int ch);

void smsm_wait_for_modem(void)
{
    int i;
    unsigned* states;
    unsigned states_size;
    states =
           (void *)smem_get_entry(ID_SHARED_STATE, &states_size);
    cprintf("Waiting for Modem...\n");
    cprintf("state=%x\n", states[SMSM_MODEM_STATE]);
    for(i=0; i<100; i++){
        if(states[SMSM_MODEM_STATE] & SMSM_OSENTERED){
            cprintf("ok\n");
            return;
        }
    }
    cprintf("timeout\n");
}

int irqCalled;

unsigned smem_get_version(int n);

void smsm_init(void)
{
    unsigned* states;
    unsigned* masks;
    unsigned states_size;
    unsigned masks_size;
    states =
           (void *)smem_get_entry(ID_SHARED_STATE, &states_size);
    states[SMSM_APPS_STATE] = 0;
    if((smem_get_version(VERSION_MODEM) >> 16) >= 0xb){
        states[SMSM_APPS_DEM] = 0;
    }
    masks =
           (void *)smem_get_entry(SMEM_SMSM_CPU_INTR_MASK, &masks_size);
    if(masks){
        int i;
        for (i = 0; i < SMSM_NUM_ENTRIES; i++)
                    masks[i * SMSM_NUM_HOSTS + SMSM_APPS] = 0xffffffff;
    }
}

void smsm_reset_modem(void)
{
    unsigned* states;
    unsigned states_size;
    states =
           (void *)smem_get_entry(ID_SHARED_STATE, &states_size);
    states[SMSM_APPS_STATE] |= SMSM_RESET | SMSM_MODEM_WAIT;
    MSM_TRIG_A2M_INT(5);
}

void smsm_reset_modem_cont(void)
{
    unsigned* states;
    unsigned states_size;
    states =
           (void *)smem_get_entry(ID_SHARED_STATE, &states_size);
    states[SMSM_APPS_STATE] &= ~SMSM_MODEM_WAIT;
    MSM_TRIG_A2M_INT(5);
}

void smsm_ack_amss_crash(void)
{
#if 0
    unsigned* states;
    unsigned old_state;
    unsigned states_size;
    states =
           (void *)smem_get_entry(ID_SHARED_STATE, &states_size);
    old_state = states[SMSM_APPS_STATE];
    cprintf("Notify apps init to Modem...\n");
//    states[SMSM_APPS_STATE] |= SMSM_APPS_CRASHDUMP; //acpu rebooting
//    states[SMSM_APPS_STATE] |= SMSM_SYSTEM_REBOOT; //acpu rebooting
//    states[SMSM_APPS_STATE] |= SMSM_APPS_REBOOT;
//    states[SMSM_APPS_STATE] |= SMSM_INIT | SMSM_SMDINIT | SMSM_RPCINIT | SMSM_SLEEP;
//    states[SMSM_APPS_STATE] |= SMSM_RESET;
//    states[SMSM_APPS_STATE] |= SMSM_SLEEP;
//    states[SMSM_APPS_STATE] |= SMSM_RPC;
//    states[SMSM_APPS_STATE] |= SMSM_INIT | SMSM_SMDINIT | SMSM_RPCINIT | SMSM_RUN;
    states[SMSM_APPS_STATE] |= SMSM_RESET | SMSM_MODEM_WAIT;
    MSM_TRIG_A2M_INT(5);
    smsm_wait_for_modem();
#else
    cprintf("%s\n", __FUNCTION__);
    mdelay(100);
    cprintf("now!\n");
dump_smem_info_n(ID_DIAG_ERR_MSG);
dump_smem_info_n(SMEM_ERR_CRASH_LOG);
//    smsm_reset_modem();
//    smsm_reset_modem_cont();
#endif
}

irq_handler irq_vector_table[NR_IRQS];

void onIRQ(void)
{
    int ret = 0;
    int i;
    unsigned irq0, irq1;
    unsigned irq0en, irq1en;
    irqCalled++;
    irq0 = readl(VIC_IRQ_STATUS0);
    irq1 = readl(VIC_IRQ_STATUS1);
    irq0en = readl(VIC_INT_EN0);
    irq1en = readl(VIC_INT_EN1);
    //cprintf("IRQ status=%x(%x) %x(%x)\n", irq0, irq0en, irq1, irq1en);
    for(i=0; i<32; i++){
        if(irq0 & irq0en & (1<<i)){
            irq_vector_table[i](i);
            writel(1<<i, VIC_INT_CLEAR0);
        }
    }
    for(i=0; i<(NR_IRQS-32); i++){
        if(irq1 & irq1 & (1<<i)){
            irq_vector_table[i+32](i+32);
            writel(1<<i, VIC_INT_CLEAR1);
        }
    }
}

int _main(unsigned zero, unsigned type, unsigned tags)
{
    const char *cmdline = 0;
    int n;

    regs_save();

#ifdef SURF7X2X
    /* 7x27 target requires pll enable with shared memory lock
     * This shared memory lock is not valid when calling arm11_clock_init
     * in nandwrite context.  So seperating it out here and calling it only
     * when enabling clocks in usbloader
     */
    arm11_smem_pll_enable();
#endif

    mmu_off();

    if(get_vecflag() == 0){
    	console_set_colors(0x0000, 0xFFFF);
    	display_init();
    	DISPLAY_MSG("Bin4ry SPL...\n");
	DISPLAY_MSG("based on Gorohs SPL\n");
    	display_versions();
    }else{
    	console_set_colors(0x0000, 0xFFFF);
    	display_init();
        DISPLAY_MSG("on interrupt(%s)\n", get_vecname());
        memdump(get_regs_int(), 16);
        clear_vecflag();
    }

    DISPLAY_MSG("irqCalled=%d\n", irqCalled);
#ifdef USE_SMEM
    //DISPLAY_MSG("smem_init() .. ");
    smem_init();
    //DISPLAY_MSG("ok!\n");
#endif
#if 0
    rpc_init();
    irq_init();
    irq_install(INT_A9_M2A_0, smd_irq_handler, 1); //edge
    irq_install(INT_A9_M2A_5, smsm_irq_handler, 1); //edge
    irq_unmask(INT_A9_M2A_0);
    irq_unmask(INT_A9_M2A_5);
    smd_irq_handler(INT_A9_M2A_0); // fake call smd_handler
#endif

//goroh: it goes hang up
//    arm11_clock_init();

#if 0
        /* must do this before board_init() so that we
        ** use the partition table in the tags if it 
        ** already exists 
        */
    if((zero == 0) && (type != 0) && tags_okay(tags)) {
        linux_type = type;
        linux_tags = tags;

        cmdline = tags_get_cmdline((void*) linux_tags);
        
        tags_import_partitions((void*) linux_tags);
        revision = tags_get_revision((void*) linux_tags);
        if(revision == 1) {
            console_set_colors(0x03E0, 0xFFFF);
        }
        if(revision == 2) {
            console_set_colors(0x49B2, 0xFFFF);
        }

        /* we're running as a second-stage, so wait for interrupt */
        boot_from_flash = 0;
    } else {
        linux_type = board_machtype();
        linux_tags = 0;
    }
#else
    boot_from_flash = 0;
    linux_type = board_machtype();
    linux_tags = 0;
#endif
#ifdef USE_SMEM
#ifndef SURF7X30
    smem_ptable_init();
#endif
#endif

    board_init();
#ifdef QCOM_ENABLE_KEYPAD
    keypad_init();
#endif

    dprintf_set_putc(uart_putc);

    if(linux_tags == 0) {
            /* generate atags containing partitions
             * from the bootloader, etc
             */
        linux_tags = ADDR_TAGS;
        cmdline = board_cmdline();
//        create_atags(linux_tags, cmdline, 0, 0);
        create_atags(linux_tags, 0, 0, 0);
    }

    if (cmdline) {
        char *sn = strstr(cmdline, SERIALNO_STR);
        if (sn) {
            char *s = serialno;
            sn += SERIALNO_LEN;
            while (*sn && (*sn != ' ') && ((s - serialno) < 31)) {
                *s++ = *sn++;
            }
            *s++ = 0;
        }
    }

#if 0
//goroh: it goes hang up
    flash_init();
#endif

    if(get_loader_param()->bootimgaddr){
        loader_param* param = get_loader_param();
        boot_from_mem(param->bootimgaddr, param->bootimgsize); 
    }

    for(n = 0; n < KEYPAD_DELAY; n++) {
        boot_poll();
    }

     check_reboot_reason();


    if (boot_from_flash) {
        DISPLAY_MSG("\n ** BOOTING LINUX FROM FLASH **\n");
        boot_linux_from_flash();
    }

    return 0;
}
