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
#include <boot/board.h>
#include <boot/gpio.h>
#include <qsd8k/lcdc.h>
#include <qsd8k/mdp.h>
#include <qsd8k/shared.h>

unsigned fb_width  = 0;
unsigned fb_height = 0;

static unsigned short *FB;

void wr32(void *_dst, unsigned n)
{
    unsigned char *src = (unsigned char*) &n;
    unsigned char *dst = _dst;

    dst[0] = src[0];
    dst[1] = src[1];
    dst[2] = src[2];
    dst[3] = src[3];
};

void mddi_start_update(void)
{
    return;
}

int mddi_update_done(void)
{
    return 1;
}

static int mdp_drv_init(void)
{
}

static unsigned clk_regime_md_reg_val(unsigned mval, unsigned dval)
{
    // M_VAL [31:16], not(2*D_VAL) [15:0]
    return ((0xFFFF & mval) << 16) | (0xFFFF & ~(2*dval));
}

static unsigned clk_regime_ns_reg_val(unsigned mval, unsigned nval)
{
    // not(N_VAL-M_VAL) [23:16], enable LCDC clocks, dual edge, /4, global
    return (0xFF & (~(nval-mval)))<<16 |
        0x1<<11 | 0x1<<9 | 0x1<<8 | 0x0<<7 | 0x2<<5 | 0x3<<3 | 0x1;
}

void lcdc_clock_init(void)
{
    unsigned m=99, n=256, d=128;  // 768/4=192, 192*99/256=74.25MHz
    writel(clk_regime_md_reg_val(m, d), LCDC_MD_REG);
    writel(clk_regime_ns_reg_val(m, n), LCDC_NS_REG);
}

void mdp_clock_init(void)
{
}

#define LCDC_PIXCLK_IN_PS 26
#define LCDC_FB_PHYS      0x02b00000
#define LCDC_FB_BPP       16

#define LCDC_FB_WIDTH     1366
#define LCDC_FB_HEIGHT    768

#define LCDC_HSYNC_PULSE_WIDTH_DCLK 40
#define LCDC_HSYNC_BACK_PORCH_DCLK  80
#define LCDC_HSYNC_FRONT_PORCH_DCLK 20
#define LCDC_HSYNC_SKEW_DCLK        0

#define LCDC_VSYNC_PULSE_WIDTH_LINES 3
#define LCDC_VSYNC_BACK_PORCH_LINES  22
#define LCDC_VSYNC_FRONT_PORCH_LINES 1

#define BIT(x)  (1<<(x))
#define DMA_DSTC0G_8BITS (BIT(1)|BIT(0))
#define DMA_DSTC1B_8BITS (BIT(3)|BIT(2))
#define DMA_DSTC2R_8BITS (BIT(5)|BIT(4))
#define CLR_G 0x0
#define CLR_B 0x1
#define CLR_R 0x2
#define MDP_GET_PACK_PATTERN(a,x,y,z,bit) (((a)<<(bit*3))|((x)<<(bit*2))|((y)<<bit)|(z))
#define DMA_PACK_ALIGN_LSB 0
#define DMA_PACK_PATTERN_RGB \
        (MDP_GET_PACK_PATTERN(0,CLR_R,CLR_G,CLR_B,2)<<8)
#define DMA_DITHER_EN                       BIT(24)
#define DMA_OUT_SEL_LCDC                    BIT(20)
#define DMA_IBUF_FORMAT_RGB565              BIT(25)

void mddi_init(void)
{
    unsigned n;

    dprintf("mddi_init() configuring for LCDC\n");

    /* MDP init */
    /* seems like not needed */

    /* LCDC init */
#if 0
    clock_enable(MDP_LCDC_PCLK_CLK);
    clock_enable(MDP_LCDC_PAD_PCLK_CLK);
    clock_set_rate(MDP_LCDC_PCLK_CLK, 74250000);
    clock_set_rate(MDP_LCDC_PAD_PCLK_CLK, 74250000);
#endif
    lcdc_clock_init();

    writel(LCDC_FB_PHYS, MSM_MDP_BASE1 + 0x90008);
    writel((LCDC_FB_HEIGHT << 16) | LCDC_FB_WIDTH, MSM_MDP_BASE1 + 0x90004);
    writel(LCDC_FB_WIDTH * LCDC_FB_BPP / 8, MSM_MDP_BASE1 + 0x9000c);
    writel(0, MSM_MDP_BASE1 + 0x90010);

    writel(DMA_PACK_ALIGN_LSB|DMA_PACK_PATTERN_RGB|DMA_DITHER_EN|
               DMA_OUT_SEL_LCDC|DMA_IBUF_FORMAT_RGB565|
               DMA_DSTC0G_8BITS|DMA_DSTC1B_8BITS|DMA_DSTC2R_8BITS,
           MSM_MDP_BASE1 + 0x90000);

    int hsync_period  = LCDC_HSYNC_PULSE_WIDTH_DCLK + LCDC_HSYNC_BACK_PORCH_DCLK + LCDC_FB_WIDTH + LCDC_HSYNC_FRONT_PORCH_DCLK;
    int vsync_period  = (LCDC_VSYNC_PULSE_WIDTH_LINES + LCDC_VSYNC_BACK_PORCH_LINES + LCDC_FB_HEIGHT + LCDC_VSYNC_FRONT_PORCH_LINES) * hsync_period;
    int hsync_ctrl    = (hsync_period << 16) | LCDC_HSYNC_PULSE_WIDTH_DCLK;
    int hsync_start_x = LCDC_HSYNC_PULSE_WIDTH_DCLK + LCDC_HSYNC_BACK_PORCH_DCLK;
    int hsync_end_x   = hsync_period - LCDC_HSYNC_FRONT_PORCH_DCLK - 1;
    int display_hctl  = (hsync_end_x << 16) | hsync_start_x;
    int display_vstart= (LCDC_VSYNC_PULSE_WIDTH_LINES + LCDC_VSYNC_BACK_PORCH_LINES) * hsync_period + LCDC_HSYNC_SKEW_DCLK;
    int display_vend  = vsync_period - (LCDC_VSYNC_FRONT_PORCH_LINES * hsync_period) + LCDC_HSYNC_SKEW_DCLK - 1;

    writel((hsync_period << 16) | LCDC_HSYNC_PULSE_WIDTH_DCLK, MSM_MDP_BASE1 + 0xe0004);
    writel(vsync_period, MSM_MDP_BASE1 + 0xe0008);
    writel(LCDC_VSYNC_PULSE_WIDTH_LINES * hsync_period, MSM_MDP_BASE1 + 0xe000c);
    writel(display_hctl, MSM_MDP_BASE1 + 0xe0010);
    writel(display_vstart, MSM_MDP_BASE1 + 0xe0014);
    writel(display_vend, MSM_MDP_BASE1 + 0xe0018);
    writel(0, MSM_MDP_BASE1 + 0xe0028);
    writel(0xff, MSM_MDP_BASE1 + 0xe002c);
    writel(LCDC_HSYNC_SKEW_DCLK, MSM_MDP_BASE1 + 0xe0030);
    writel(0, MSM_MDP_BASE1 + 0xe0038);
    writel(0, MSM_MDP_BASE1 + 0xe001c);
    writel(0, MSM_MDP_BASE1 + 0xe0020);
    writel(0, MSM_MDP_BASE1 + 0xe0024);

    writel(1, MSM_MDP_BASE1 + 0xe0000);

    //clock_enable(MDP_CLK);
    mdp_clock_init();

    //panel_backlight(0);

    //panel_poweron();

    fb_width  = LCDC_FB_WIDTH;
    fb_height = LCDC_FB_HEIGHT;

    dprintf("panel is %d x %d\n", fb_width, fb_height);

    FB = LCDC_FB_PHYS; //alloc(2 * fb_width * fb_height);

    for(n = 0; n < (fb_width * fb_height); n++) FB[n] = 0;

    gpio_set(32, 1);
    gpio_dir(32, 1);
    mdelay(100);
    gpio_set(20, 1);
    gpio_dir(20, 1);
    mdelay(100);
    gpio_set(155, 1);
    gpio_dir(155, 1);
    //gpio_set(61, 1);
}

void *mddi_framebuffer(void)
{
    return FB;
}

