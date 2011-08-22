/*
 * Copyright (c) 2009, Google Inc.
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

#include <boot/flash.h>
#include <boot/boot.h>

#define SMEM_AARM_PARTITION_TABLE  9

struct smem_ptn {
    char name[16];
    unsigned start;
    unsigned size;
    unsigned attr;
} __attribute__ ((__packed__));

struct smem_ptable {
#define _SMEM_PTABLE_MAGIC_1 0x55ee73aa
#define _SMEM_PTABLE_MAGIC_2 0xe35ebddb
    unsigned magic[2];
    unsigned version;
    unsigned len;
    struct smem_ptn parts[16];
} __attribute__ ((__packed__));

/* partition table from SMEM */
static struct smem_ptable smem_ptable;
static unsigned smem_apps_flash_start = 0xFFFFFFFF;
static struct ptentry modem_ptentry = {
    .name = "",
};

void dump_smem_ptable(void)
{
    int i;

    for (i = 0; i < 16; i++) {
        struct smem_ptn *p = &smem_ptable.parts[i];
        if (p->name[0] == '\0')
            continue;
         //DISPLAY_MSG("%d: %s offs=0x%x size=0x%x attr: 0x%x\n",
            //i, p->name, p->start, p->size, p->attr);
    }
}

void smem_ptable_init(void)
{
    unsigned size;
    unsigned i;
    struct ptname_map* map;
    void* addr;

    addr = smem_get_entry(SMEM_AARM_PARTITION_TABLE, &size);

    if (addr == 0 || size != sizeof(smem_ptable))
        return;

    memcpy(&smem_ptable, addr, size);

    // with the right magic number?
    if (smem_ptable.magic[0] != _SMEM_PTABLE_MAGIC_1 ||
        smem_ptable.magic[1] != _SMEM_PTABLE_MAGIC_2)
        return;

     //DISPLAY_MSG("smem ptable found: ver: %d len: %d\n",
        //smem_ptable.version, smem_ptable.len);

    dump_smem_ptable();

    // find AMSS parition
    for (i = 0; i < smem_ptable.len; i++) {
        if (!strcmp(smem_ptable.parts[i].name, "0:AMSS")) {
            strcpy(modem_ptentry.name, "modem");
            modem_ptentry.start = smem_ptable.parts[i].start;
            modem_ptentry.length = smem_ptable.parts[i].size;
            modem_ptentry.flags = smem_ptable.parts[i].attr;
            break;
        }
    }
    
    // find APPS partition starting point
    for (i = 0; i < smem_ptable.len; i++) {
        if (!strcmp(smem_ptable.parts[i].name, "0:APPS")) {
            smem_apps_flash_start = smem_ptable.parts[i].start;
            break;
        }
    }
}

unsigned smem_ptable_get_apps_flash_start(void)
{
    return smem_apps_flash_start;
}

ptentry* smem_ptable_get_modem_ptentry(void)
{
    if (modem_ptentry.name[0]) {
        return &modem_ptentry;
    }    
    else {
        return 0;
    }
}
