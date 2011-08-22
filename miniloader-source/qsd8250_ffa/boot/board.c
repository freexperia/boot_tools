/* Copyright 2007, Google Inc. */
/* Copyright (c) 2009, Code Aurora Forum.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <boot/boot.h>
#include <boot/flash.h>
#include <boot/board.h>
#include <qsd8k/shared.h>

ptentry PTABLE[] = {
    {
        //.start = 360,
        .start = 420,
        .length = 40,
        .name = "boot",
    },
    {
        //.start = 416,
        .start = 476,
        .length = 512,
        .name = "system",
    },
    {
        //.start = 928,
        .start = 988,
        //.length =  95 + 1024,
        .length =  35 + 1024,
        .name = "userdata",
    },
    {
        .name = "",
    },
};

const char *board_cmdline(void)
{
    volatile loader_param* param = (volatile loader_param*)0x300ff000;
    if(param->bootparam[0] == 0)
        return "semcandroidboot.serialno=CB511GJGAM semcandroidboot.startup=0x0000000d semcandroidboot.000008A2=00 console=null androidboot.hardware=es209ra\0\0\0\0";
    else
        return param->bootparam;
}

unsigned board_machtype(void)
{
    volatile loader_param* param = (volatile loader_param*)0x300ff000;
    return param->machine_type;
}

void board_init()
{
#if 0
    unsigned n;

    /* if we already have partitions from elsewhere,
    ** don't use the hardcoded ones
    */
    if(flash_get_ptn_count() == 0) {
        for(n = 0; PTABLE[n].name[0]; n++) {
            flash_add_ptn(PTABLE + n);
        }
    }

    clock_enable(UART3_CLK);
    clock_set_rate(UART3_CLK, 19200000 / 4);

    uart_init(2);
#endif
}

void board_usb_init(void)
{
}

void board_ulpi_init(void)
{
}

void board_reboot(void)
{
    void smem_reboot(void);
    smem_reboot();
//    reboot();
}

void board_getvar(const char *name, char *value)
{
}
