/*
 * Copyright (c) 2008, Google Inc.
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
#include <msm7k/gpt.h>
#include <msm7k/shared.h>

#ifdef SURF7X2X
    #define SMEM_SPINLOCK_ARRAY_INDEX       7
    #define CLK_REGIME_SPINLOCK_INDEX       7
    #define NULL                            0x0
#endif

static inline unsigned rd_cycle_count(void) 
{
    unsigned cc;
    asm volatile (
        "mrc p15, 0, %0, c15, c12, 1\n" :
        "=r" (cc) 
        );   
    return cc;
}

static inline unsigned rd_dtcm(void) 
{
    unsigned cc;
    asm volatile (
        "mrc p15, 0, %0, c9, c1, 0\n" :
        "=r" (cc) 
        );   
    return cc;
}

static inline unsigned rd_itcm(void) 
{
    unsigned cc;
    asm volatile (
        "mrc p15, 0, %0, c9, c1, 1\n" :
        "=r" (cc) 
        );   
    return cc;
}

static inline void perf_enable(void)
{
    asm volatile (
        "mcr p15, 0, %0, c15, c12, 0\n" : : "r" (7) 
        );
}

static inline void perf_disable(void)
{
    asm volatile (
        "mcr p15, 0, %0, c15, c12, 0\n" : : "r" (0)
        );
}

unsigned cycles_per_second(void)
{
    unsigned T0, T1;

    perf_enable();
    
    writel(0, GPT_CLEAR);
    writel(0, GPT_ENABLE);
    while(readl(GPT_COUNT_VAL) != 0) ;
    
    writel(GPT_ENABLE_EN, GPT_ENABLE);    
    T0 = rd_cycle_count();
    while(readl(GPT_COUNT_VAL) < 32766) ;
    T1 = rd_cycle_count();

    writel(0, GPT_ENABLE);
    writel(0, GPT_CLEAR);

    perf_disable();
    
    return T1-T0;
}

void mdelay(unsigned msecs)
{
    msecs *= 33;
    
    writel(0, GPT_CLEAR);
    writel(0, GPT_ENABLE);
    while(readl(GPT_COUNT_VAL) != 0) ;
    
    writel(GPT_ENABLE_EN, GPT_ENABLE);    
    while(readl(GPT_COUNT_VAL) < msecs) ;

    writel(0, GPT_ENABLE);
    writel(0, GPT_CLEAR);    
}

void udelay(unsigned usecs)
{
    usecs = (usecs * 33 + 1000 - 33) / 1000;
    
    writel(0, GPT_CLEAR);
    writel(0, GPT_ENABLE);
    while(readl(GPT_COUNT_VAL) != 0) ;
    
    writel(GPT_ENABLE_EN, GPT_ENABLE);    
    while(readl(GPT_COUNT_VAL) < usecs) ;

    writel(0, GPT_ENABLE);
    writel(0, GPT_CLEAR);    
}

void print_cpu_speed(void)
{
    unsigned cps = cycles_per_second();
    dprintf("1 second = %d cycles\n%d MHz\n", cps, cps / 1000000);
}

#define A11S_CLK_CNTL 0xC0100100
#define A11S_CLK_SEL  0xC0100104

#define C A11S_CLK_CNTL
#define S A11S_CLK_SEL

#if FROM_APPSBOOT_MBN
static unsigned tbl_old[] = {
    C, 0x640000,
    S, 2,
    C, 0x640010,
    C, 0x64001F,
    S, 3,
    C, 0x64101F,
    C, 0x64171F,
    S, 2,
    C, 0x64171F,
    C, 0x641715,
    S, 3,
    S, 5,
    C, 0x641715,
    C, 0x641315,
    S, 4,
    S, 6,
    C, 0x641315,
    C, 0x641312,
    S, 7,
    C, 0x641312,
    C, 0x641112,
    S, 6,
    0
};
#endif

#define WT_ST_CNT_100 100
#define CLK_SRC_TCXO 0
#define CLK_SRC_PLL2 2

#define DIV_5 4
#define DIV_4 3
#define DIV_3 2
#define DIV_2 1
#define DIV_1 0

#define CLK_SRC0 0
#define CLK_SRC1 1

/* Table to scale up apps processor's
   clock ferquency at bootup*/
static unsigned tbl[] = {
#if EXPLORE
    C, 0x640000, S, 2,
    C, 0x640001, S, 3,
    C, 0x640201, S, 2,
    C, 0x640203, S, 3,
    C, 0x640403, S, 2,
    C, 0x640405, S, 3,
    C, 0x640605, S, 2,
    C, 0x640607, S, 3,
    C, 0x640807, S, 2,
    C, 0x640809, S, 3,
    C, 0x640A09, S, 2,
    C, 0x640A0B, S, 3,
    C, 0x640C0B, S, 2,
    C, 0x640C0D, S, 3,
    C, 0x640E0D, S, 2,
    C, 0x640E0F, S, 3,
#endif
#ifdef SURF7X2X
    C, ((WT_ST_CNT_100 << 16) | \
        (CLK_SRC_TCXO << 12)  | (DIV_1 << 8) | \
        (CLK_SRC_TCXO << 4)   | (DIV_1)),

    S, ((DIV_4 << 1) | (CLK_SRC0)),

    C, ((WT_ST_CNT_100 << 16) | \
        (CLK_SRC_TCXO << 12)  | (DIV_1 << 8) | \
        (CLK_SRC_PLL2 << 4)   | (DIV_5)),

    S, ((DIV_4 << 1) | (CLK_SRC1)),

    C, ((WT_ST_CNT_100 << 16) | \
        (CLK_SRC_PLL2 << 12)  | (DIV_3 << 8) | \
        (CLK_SRC_PLL2 << 4)   | (DIV_5)),

    S, ((DIV_4 << 1) | (CLK_SRC0)),

    C, ((WT_ST_CNT_100 << 16) | \
        (CLK_SRC_PLL2 << 12)  | (DIV_3 << 8) | \
        (CLK_SRC_PLL2 << 4)   | (DIV_2)),

    S, ((DIV_3 << 1) | (CLK_SRC1)),
#else
    C, ((WT_ST_CNT_100 << 16) | \
        (CLK_SRC_TCXO << 12)  | (DIV_1 << 8) | \
        (CLK_SRC_TCXO << 4)   | (DIV_1)),

    S, ((DIV_4 << 1) | (CLK_SRC0)),

    C, ((WT_ST_CNT_100 << 16) | \
        (CLK_SRC_TCXO << 12)  | (DIV_1 << 8) | \
        (CLK_SRC_PLL2 << 4)   | (DIV_4)),

    S, ((DIV_4 << 1) | (CLK_SRC1)),

    C, ((WT_ST_CNT_100 << 16) | \
        (CLK_SRC_PLL2 << 12)  | (DIV_3 << 8) | \
        (CLK_SRC_PLL2 << 4)   | (DIV_4)),

    S, ((DIV_4 << 1) | (CLK_SRC0)),

    C, ((WT_ST_CNT_100 << 16) | \
        (CLK_SRC_PLL2 << 12)  | (DIV_3 << 8) | \
        (CLK_SRC_PLL2 << 4)   | (DIV_2)),

    S, ((DIV_4 << 1) | (CLK_SRC1)),
#endif
    0
};
#undef C
#undef S

#if TESTCASE
unsigned cc_div[16] = {
	1, 2, 3, 4,  5, 8, 6, 16,
	1, 1, 1, 1,  1, 1, 1, 32  /* guess */
};

unsigned cc_base[4] = {
    19200000,
    245760000,
    800000000,
    0
};

unsigned cs_div[4] = {
	1, 2, 3, 4
};

void info(unsigned c, unsigned s)
{
    unsigned src_sel, src_div;

    if(s & 1) {
            /* src1 selected */
        src_sel = (c >> 4) & 0x7;
        src_div = c & 0xF;
    } else {
            /* src0 selected */
        src_sel = (c >> 12) & 0x7;
        src_div = (c >> 8) & 0xF;
    }

    unsigned src = s & 1;
    unsigned pdiv = cs_div[(s >> 1) & 3];
    unsigned div = cc_div[src_div];
    unsigned clk = cc_base[src_sel] / div;
    unsigned pclk = clk / pdiv;

    unsigned cps = cycles_per_second();

    dprintf("CC=0x%x CS=0x%x SRC=%d PDIV=%d SEL=%d DIV=%d CPS=%d ACLK=%d\n",
            c, s, src, pdiv, src_sel, div, cps, clk);
}


void arm11_clock_test(void)
{
    unsigned c, s;
    unsigned *x = tbl;

    while(*x) {
        unsigned *ptr = (unsigned*) *x++;
        unsigned val = *x++;
        *ptr = val;

        if(ptr == ((unsigned*) A11S_CLK_CNTL)) {
            c = val;
        } else {
            s = val;
            info(c, s);
        }
    }
}
#endif

void arm11_smem_pll_enable()
{
#ifdef SURF7X2X
    unsigned int *smem_clkregim_spinlock_addr = (unsigned int *) \
    smem_get_entry(SMEM_SPINLOCK_ARRAY_INDEX, NULL) + CLK_REGIME_SPINLOCK_INDEX;

    acquire_smem_spinlock(smem_clkregim_spinlock_addr);

    pll_enable(2);

    release_smem_spinlock(smem_clkregim_spinlock_addr);
#endif

}

void arm11_clock_init(void)
{
    unsigned *x = tbl;

    while(*x) {
        unsigned *ptr = (unsigned*) *x++;
        unsigned val = *x++;
        *ptr = val;
        udelay(50);
    }
}
