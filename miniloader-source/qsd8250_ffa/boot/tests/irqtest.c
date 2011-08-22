/* Copyright 2007, Google Inc. */

#include "boot.h"

#include "msm7200gpt.h"
#include "msm7200vic.h"
#include "msm7200uart.h"
#include "irqs.h"

#define uwr(v,a) msm_writel(v, MSM_UART1_BASE + (a))
#define urd(a) msm_readl(MSM_UART1_BASE + (a))

#define DIV 128

static unsigned scount = 0;
static unsigned ticks = 0;
static unsigned total = 0;

void gpt_handler(unsigned n)
{
    msm_writel(1 << INT_GP_TIMER_EXP, VIC_INT_CLEAR0);
    
    scount++;
    total++;
    if(scount == DIV) {
        scount = 0;
        dprintf("TICK %d\n", ticks++);
    }
#if 0
    msm_writel(0, GPT_CLEAR);
    while(msm_readl(GPT_COUNT_VAL) != 0) ;
#endif
}

void uart_handler(unsigned n)
{
    dprintf("uart_handler()\n");
    if(urd(UART_SR) & UART_SR_RX_READY) {
        dprintf("<%c>", urd(UART_RF));
    }

    msm_writel(1 << INT_UART1, VIC_INT_CLEAR0);
}

/* delays about one second at 19.2MHz */
void pause() 
{
    int delay = 19200000 / 25;
    while(delay-- > 0) {
        asm("nop\n");
    }
}

void scantest(void)
{
    unsigned n, r, c, t;
    char map[32];

    map[16] = 0;

    dprintf("scantest\n");

        /* setup rows */
    for(n = 33; n < 36; n++) {
        gpio_output_enable(n, 0);
        gpio_write(n, 0);
    }

        /* setup columns */
    for(n = 36; n < 41; n++) {
        gpio_output_enable(n, 0);
        gpio_write(n, 0);
    }

    for(;;) {
        n = 0;
        for(r = 0; r < 3; r++) {
            gpio_output_enable(33+r, 1);            
            for(c = 0; c < 5; c++) {
                if(gpio_read(36+c)) {
                    map[n++] = '-';
                } else {
                    map[n++] = 'X';
                }
            }
            gpio_output_enable(33+r, 0);
        }

        dprintf("[ %s ]\n", map);
        for(t = 0; t < 100000; t++) asm("nop;");
    }
}

extern unsigned BOOTLOADER_HEAP;

int _main(void) 
{
    unsigned tmp = 0;

    scantest();    
//    arm11_clock_init();
    
	dprintf("\n\nHello, ARM1136 World!\n");

    print_cpu_speed();
    
    irq_init();
    irq_install(INT_GP_TIMER_EXP, gpt_handler, 1);
    irq_install(INT_UART1, uart_handler, 0);
    
    msm_writel(0, GPT_CLEAR);
    msm_writel(32768 / DIV, GPT_MATCH_VAL);
    msm_writel(GPT_ENABLE_EN | GPT_ENABLE_CLR_ON_MATCH_EN, GPT_ENABLE);
    
    irq_unmask(INT_GP_TIMER_EXP);
    irq_unmask(INT_UART1);

    uwr(UART_IMR_RXLEV, UART_IMR);
    
    for(;;) ;
    
}
