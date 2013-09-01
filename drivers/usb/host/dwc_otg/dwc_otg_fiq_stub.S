/*
 * dwc_otg_fiq_fsm.S
 *
 *  Created on: 21 Aug 2013
 *      Author: jonathan
 * Small stub that is copied directly to 0xFFFF001C - FIQ vector.
 * Brings out the FIQ Mode saved registers (which contain pointers
 * to our FIQ state) and puts them in the right calling convention.
 */


#include <asm/assembler.h>
#include <linux/linkage.h>


.text

.global _dwc_otg_fiq_stub_end;

ENTRY(_dwc_otg_fiq_stub)
	/* Stash unbanked regs - SP will have been set up for us */
	mov ip, sp;
	stmdb sp!, {r0-r12, lr};
	/* r11 = fp, don't trample it */
	mov r4, fp;
	/* set EABI frame size */
	sub fp, ip, #512;

	/* for fiq NOP mode - just need state */
	mov r0, r8;
	/* r9 = num_channels */
	mov r1, r9;
	/* r10 = struct *dma_bufs */
//	mov r2, r10;

	/* r4 = &fiq */
	blx r4;

	ldmia sp!, {r0-r12, lr};
	subs pc, lr, #4;
_dwc_otg_fiq_stub_end:
END(_dwc_otg_fiq_stub)

