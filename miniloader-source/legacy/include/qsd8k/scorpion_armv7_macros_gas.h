/*
 * Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
;------------------------------------------------------------------------------
; FileName    : scorpion_armv7_macros_gas.h
; Description : This header file contains the macros for Scorpion ARMv7
;               instructions that are not supported in RVCT version < 3.0.
;               (Modified for GNU assembler format).
;------------------------------------------------------------------------------
;
;                           EDIT HISTORY FOR FILE
;
; This section contains comments describing changes made to this file.
; Notice that changes are listed in reverse chronological order.
;
; when       who     what, where, why
; --------   ---     ----------------------------------------------------------
; 04/07/09   dng     Modified for GNU assembler format
;                    (based on scorpion_armv7_macros.h#1)
; 06/04/07   MJS     Initial revision.
;============================================================================*/

#ifndef SCORPION_ARMV7_MACROS_GAS_H
#define SCORPION_ARMV7_MACROS_GAS_H


/*===========================================================================

                      PUBLIC DATA DECLARATIONS

===========================================================================*/

//; Options for DMB/DSB/ISB Barriers, define these manually for RVCT < 3.0
.equ SY, 0xf
.equ UN, 0x7
.equ ST, 0xe
.equ UNST, 0x6


/*
;============================================================================
; MACRO mdmb
;
; ARGS
;   NONE
;
; DESCRIPTION
;   Performs a data memory barrier, either using the ARMv7 instruction or the
;   legacy coprocessor instruction.
;
; NOTES
;   For reference see ARM DDI 0406A-03 section A3.8.3.
;============================================================================
*/
.macro mdmb
   .byte 0x50, 0xf0, 0x7f, 0xf5
.endm

/*
;============================================================================
; MACRO mdsb
;
; ARGS
;   NONE
;
; DESCRIPTION
;   Performs a data synchronization barrier, either using the ARMv7 instruction
;   or the legacy coprocessor instruction.
;
; NOTES
;   For reference see ARM DDI 0406A-03 section A3.8.3.
;============================================================================
*/
.macro mdsb
   .byte (0x40 | SY), 0xf0, 0x7f, 0xf5
.endm

/*
;============================================================================
; MACRO misb
;
; ARGS
;   NONE
;
; DESCRIPTION
;   Performs an instruction synchronization barrier, either using the ARMv7
;   instruction or the legacy coprocessor instruction.
;
; NOTES
;   For reference see ARM DDI 0406A-03 section A3.8.3.
;============================================================================
*/
.macro misb
   .byte (0x60 | SY), 0xf0, 0x7f, 0xf5
.endm

#endif  /* SCORPION_ARMV7_MACROS_GAS_H */
