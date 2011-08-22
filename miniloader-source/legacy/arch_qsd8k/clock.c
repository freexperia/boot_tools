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
#include <qsd8k/gpt.h>

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

#define A11S_CLK_CNTL 0xAC100100
#define A11S_CLK_SEL  0xAC100104

#define C A11S_CLK_CNTL
#define S A11S_CLK_SEL

#define SCPLL_CTL  0xA8800004
#define SCPLL_CAL  0xA8800008
#define SCPLL_STAT 0xA8800010
#define SCPLL_CTLE 0xA8800024

/* 7x30 clock control and source registers */
#define SCSS_CLK_CTL 0xC0101004
#define SCSS_CLK_SEL 0xC0101008


#if TESTCASE
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
    /* MSM7K */
    /*
    C, 0x640000, S, 2,
    C, 0x64001F, S, 3,
    C, 0x64171F, S, 2,
    C, 0x641715, S, 5,
    C, 0x641315, S, 6,
    C, 0x641312, S, 7,
    C, 0x641112, S, 6,
    */

    /* QSD8K */
    C, 0x00000000, S, 0,  //  19,200 kHz - tcx0    src 0
    C, 0x0000001F, S, 1,  //  48,000 kHz - glb/16  src 1
    C, 0x00001B1F, S, 0,  //  64,000 kHz - glb/12  src 0
    C, 0x00001B17, S, 1,  //  96,000 KHz - glb/8   src 1
    C, 0x00001517, S, 4,  // 128,000 KHz - glb/6   src 0 scorpion axi bus
    C, 0x00001513, S, 1,  // 192,000 KHz - glb/4   src 1
    C, 0x00001213, S, 0,  // 256,000 KHz - glb/3   src 0
    C, 0x00001230, S, 3,  // 384,000 KHz - sco/1   src 1 scorpion pll
    0
};
#undef C
#undef S

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
#endif  /* TESTCASE */


void arm11_clock_init(void)
{
    unsigned val;

    /* Init Scorpion PLL */
    /* Go to standby */
    writel(0x2, SCPLL_CTL);
    udelay(10);

    /* Calibrate for 384-1497 MHz */
    writel(0x270A0000, SCPLL_CAL);
    writel(0x4, SCPLL_CTL);
    udelay(10);
    while(readl(SCPLL_STAT) & 0x2);

    /* Shot-switch directly to 1190MHz */
    writel(0x001400FC, SCPLL_CTLE);
    writel(0x7, SCPLL_CTL);
    udelay(10);
    while(readl(SCPLL_STAT) & 0x3);

    /* Change Scorpion clock source */
    val = readl(A11S_CLK_SEL);
    val &= ~(0x3 << 1);
    val |= (1 << 1);
    writel(val, A11S_CLK_SEL);
}

void arm11_clock_init_nw(void)
{
    /* Switch to Scorpion PLL if needed */
    if (((readl(A11S_CLK_SEL) & 0x6) >> 1) != 1) {
        arm11_clock_init();
    }
}

