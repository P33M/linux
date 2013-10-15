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

/**
  * fiq_stub: entry copied to the FIQ vector page to allow
  * a C-style function call with arguments from the FIQ banked registers
  * r0 = &hcd->fiq_state
  * r1 = &hcd->num_channels
  * r2 = &hcd->dma_buffers
  * Tramples: r0, r1, r2, r4, fp, ip
  */


ENTRY(_dwc_otg_fiq_stub)
	/* Stash unbanked regs - SP will have been set up for us */
	mov ip, sp;
	stmdb sp!, {r0-r12, lr};
#ifdef FIQ_DEBUG
	// Cycle profiling - read cycle counter at start
	mrc p15, 0, r5, c15, c12, 1;
#endif
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

	/* r4 = &fiq_c_function */
	blx r4;
#ifdef FIQ_DEBUG
	mrc p15, 0, r4, c15, c12, 1;
	subs r5, r5, r4;
	// r5 is now the cycle count time for executing the FIQ. Store it somewhere?
#endif
	ldmia sp!, {r0-r12, lr};
	subs pc, lr, #4;
_dwc_otg_fiq_stub_end:
END(_dwc_otg_fiq_stub)

