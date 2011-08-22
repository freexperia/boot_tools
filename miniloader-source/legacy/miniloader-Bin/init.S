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

.global mmu_off
.global regs_save
.global clear_vecflag
.global get_vecflag
.global get_regs_int

#include <boot/arm.h>
#include <boot/boardconfig.h>

#ifdef QCOM_SCORPION
#include <qsd8k/scorpion_armv7_macros_gas.h>
#include <qsd8k/scorpion_sysini_gas.h>
#endif

v_reset:
	b start
v_undefined:
	b on_undefined
v_swi:
	b on_swi
v_prefetch_abt:
	b on_prefetch_abt
v_data_abt:
	b on_data_abt
v_reserved:
	b on_reserved
v_irq:
	b on_irq
v_fiq:
	b on_fiq

regs_backup: 
	.long 0 @r1
	.long 0 @r2
	.long 0 @r3
	.long 0 @r4
	.long 0 @r5
	.long 0 @r6
	.long 0 @r7
	.long 0 @r8
	.long 0 @r9
	.long 0 @r10
	.long 0 @r11
	.long 0 @r12
	.long 0 @sp
	.long 0 @lr
	.long 0 @pc
	.long 0 @reserved
intregs_backup: 
	.long 0 @r0
	.long 0 @r1
	.long 0 @r2
	.long 0 @r3
	.long 0 @r4
	.long 0 @r5
	.long 0 @r6
	.long 0 @r7
	.long 0 @r8
	.long 0 @r9
	.long 0 @r10
	.long 0 @r11
	.long 0 @r12
	.long 0 @lr
	.long 0 @reserved
	.long 0 @reserved
vec_flag:
	.long 0
regs_flag:
	.long 0

on_undefined:
        ldr    sp, =intregs_backup
	stmia  sp, {r0-r12,lr}
	mov    r0, #1
	str    r0, vec_flag
	b regs_load
on_swi:
        ldr    sp, =intregs_backup
	stmia  sp, {r0-r12,lr}
	mov    r0, #2
	str    r0, vec_flag
	b regs_load
on_prefetch_abt:
        ldr    sp, =intregs_backup
	stmia  sp, {r0-r12,lr}
	mov    r0, #3
	str    r0, vec_flag
	b regs_load
on_data_abt:
        ldr    sp, =intregs_backup
	stmia  sp, {r0-r12,lr}
	mov    r0, #4
	str    r0, vec_flag
	b regs_load
on_reserved:
        ldr    sp, =intregs_backup
	stmia  sp, {r0-r12,lr}
	mov    r0, #5
	str    r0, vec_flag
	b regs_load
on_irq:
#if 0
        ldr    sp, =intregs_backup
	stmia  sp, {r0-r12,lr}
	mov    r0, #6
	str    r0, vec_flag
	b regs_load
#else
        ldr    sp, =intregs_backup
	stmia  sp, {r0-r12,lr}
        ldr    sp, =BOOTLOADER_INTSTACK
	stmfd  sp!, {r0-r12,lr}
	bl     onIRQ
	ldmfd  sp!, {r0-r12,lr}
	subs   pc,  r14,#4
#endif
on_fiq:
        ldr    sp, =intregs_backup
	stmia  sp, {r0-r12,lr}
	mov    r0, #7
	str    r0, vec_flag
	b regs_load

start:
#ifdef QCOM_SCORPION
#ifndef SURF7X30
	/* MMU init */
	ldr r0, =0x00C54078
	mcr p15, 0, r0, c1, c0, 0
	misb
#endif

	/* Initialize Scorpion */
	bl SET_SA
	SYSINI

#ifdef QCOM_CHECK_WARM_BOOT_TAG
	/* Check warm boot tag */
	ldr r0, apps_boot_tag
	cmp r0, #1
	bne cold_boot

	/* Jump to warm boot code */
	mov r0, #PHYSICAL_DRAM_BASE
	mov pc, r0

cold_boot:
	mov r0, #1
	str r0, apps_boot_tag
#endif
#endif

#ifdef QCOM_RELOCATE_TO_ZERO
	ldr r5, =0xfffff000
	ands r4, pc, r5
	beq already_at_zero

	/* we're not loaded at 0 -- relocate us back down to where we belong */
	mov r5, #0
	ldr r6, =BOOTLOADER_END
1:	ldr r7, [r4], #4
	str r7, [r5], #4
	cmp r5, r6
	bne 1b
		
	mov pc, #0
#endif

already_at_zero:		
	/* save registers for main() */
	mov r7, r0
	mov r8, r1
	mov r9, r2
	mov r10, r3

	/* resume if regs backup exists */
	ldr r0, =regs_flag
	ldr r0, [r0]
	cmp r0, #1
	bne normal_boot
	b regs_load
normal_boot:

	/* init stack */
	ldr r0, =BOOTLOADER_STACK
	msr cpsr_c, #(PSR_I | PSR_F | PSR_SVC)
	mov sp, r0

	/* zero the BSS */
	ldr r1, =BOOTLOADER_BSS
	ldr r2, =BOOTLOADER_END
	mov r0, #0
1:	str r0, [r1], #4
	cmp r1, r2
	ble 1b
		
	bl periph_2gb_open

	/* restore registers for main() */
	mov r0, r7
	mov r1, r8
	mov r2, r9
	mov r3, r10
		
	ldr r4, =_main
	blx r4
	b .

#ifdef QCOM_SCORPION
	/* tag for scorpion boot*/
apps_boot_tag: 
	.word 0
#endif

regs_load:
	ldr r0, =regs_flag
	ldr r1, [r0]
	cmp r1, #1
	bne regs_load_ng
	@mov r1, #0
	@str r1, [r0]
	ldr r0, =regs_backup
	ldmia r0, {r1-r12,sp,pc}
	nop
regs_load_ng:
	mov pc, #0

regs_save:
	ldr r0, =regs_backup
	stmia r0, {r1-r12,sp,lr,pc}
	mov r0, #1
	str r0, regs_flag
	mov pc, lr

mmu_off:
#ifndef SURF7X30
	/* MMU init */
	ldr r0, =0x00C54078
	mcr p15, 0, r0, c1, c0, 0
	misb
#endif
	mov pc, lr

clear_vecflag:
	mov r0, #0
	str r0, vec_flag
	mov pc, lr

get_vecflag:
	ldr r0, =vec_flag
	ldr r0, [r0]
	mov pc, lr

get_regs_int:
	ldr r0, =intregs_backup
	mov pc, lr

