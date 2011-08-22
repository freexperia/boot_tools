/* Copyright 2007, Google Inc. */

#include "msm7200gpt.h"
#include "msm7200uart.h"
#include "msm7200vic.h"
#include "irqs.h"

#include "boot.h"

void pause_1s() 
{
    int delay = 19200000 / 25;
    while(delay-- > 0) {
        asm("nop\n");
    }
}

void vic_init(void)
{
        /* select level interrupts */
    msm_writel(0, VIC_INT_TYPE0);
    msm_writel(0, VIC_INT_TYPE1);
        /* select IRQ for all INTs */
    msm_writel(0, VIC_INT_SELECT0);
    msm_writel(0, VIC_INT_SELECT1);
        /* disable all INTs */
    msm_writel(0, VIC_INT_EN0);
    msm_writel(0, VIC_INT_EN1);
        /* enable IRQs */

        /* don't use 1136 vic */
    msm_writel(0, VIC_CONFIG);
    
    msm_writel(1, VIC_INT_MASTEREN);
    enable_irq();
}

volatile int int_count = -10;

int _main(void) 
{
    unsigned r[6];
    unsigned n;
    int last_int_count = int_count;

	dcc_puts("\n\nHello, ARM1136 World!\n");

    read_isa_regs(r);
    for(n = 0; n < 6; n++) {
        dprintf("ISA%d = %x\n", n, r[0]);
    }

    for(;;);
    
    
#if 0    
    uart_init();
    uart_test();
#endif

//  msm_writel(0, GPT_ENABLE);
//    msm_writel(19200000 / 10, GPT_MATCH_VAL);
//    msm_writel(19200, GPT_MATCH_VAL);
    msm_writel(32768, GPT_MATCH_VAL);
    msm_writel(0, GPT_CLEAR);
    msm_writel(GPT_ENABLE_EN | GPT_ENABLE_CLR_ON_MATCH_EN, GPT_ENABLE);
//    msm_writel(GPT_ENABLE_EN, GPT_ENABLE);

    msm_writel(1 << INT_GP_TIMER_EXP, VIC_INT_CLEAR0);

        /* select level interrupts */
    msm_writel(1 << INT_GP_TIMER_EXP, VIC_INT_TYPE0);
    msm_writel(0, VIC_INT_TYPE1);
        /* select IRQ for all INTs */
    msm_writel(0, VIC_INT_SELECT0);
    msm_writel(0, VIC_INT_SELECT1);
        /* disable all INTs */
    msm_writel(0, VIC_INT_EN0);
    msm_writel(0, VIC_INT_EN1);
        /* enable IRQs */

        /* don't use 1136 vic */
    msm_writel(0, VIC_CONFIG);
    
    msm_writel(1, VIC_INT_MASTEREN);

    msm_writel(1 << INT_GP_TIMER_EXP, VIC_INT_EN0);    
    enable_irq();    

    dprintf("WHEE!\n");
    
    for(;;) {
        if(int_count != last_int_count) {
            last_int_count++;
            dprintf("COUNT %d\n", last_int_count);
        }
        n = msm_readl(GPT_COUNT_VAL);
    }
	return 0;
}

void irq_handler(void)
{
    unsigned x, y, n;

    x = msm_readl(VIC_IRQ_VEC_PEND_RD);
    msm_writel(1 << INT_GP_TIMER_EXP, VIC_INT_CLEAR0);
    y = msm_readl(VIC_IRQ_VEC_PEND_RD);
    int_count++;    
}
