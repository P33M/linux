/*
 * dwc_otg_fiq_fsm.c - The finite state machine FIQ
 *
 * Copyright (c) 2013 Raspberry Pi Foundation
 *
 * Author: Jonathan Bell <jonathan@raspberrypi.org>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *	* Redistributions of source code must retain the above copyright
 *	  notice, this list of conditions and the following disclaimer.
 *	* Redistributions in binary form must reproduce the above copyright
 *	  notice, this list of conditions and the following disclaimer in the
 *	  documentation and/or other materials provided with the distribution.
 *	* Neither the name of Raspberry Pi nor the
 *	  names of its contributors may be used to endorse or promote products
 *	  derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * This FIQ implements functionality that performs split transactions on
 * the dwc_otg hardware without any outside intervention. A split transaction
 * is "queued" by nominating a specific host channel to perform the entirety
 * of a split transaction. This FIQ will then perform the microframe-precise
 * scheduling required in each phase of the transaction until completion.
 *
 * The FIQ functionality has been surgically implanted into the Synopsys
 * vendor-provided driver.
 *
 * NB: Large parts of this implementation have architecture-specific code.
 * For porting this functionality to other ARM machines, the minimum is required:
 * - An interrupt controller allowing the top-level dwc USB interrupt to be routed
 *   to the FIQ
 * - A method of forcing a software generated interrupt from FIQ mode that then
 *   triggers an IRQ entry (with the dwc USB handler called by this IRQ number)
 * - Guaranteed interrupt routing such that both the FIQ and SGI occur on the same
 *   processor core - there is no locking between the FIQ and IRQ.
 *
 */

#include "dwc_otg_fiq_fsm.h"


char buffer[1000*16];
int wptr;
void _fiq_print(enum fiq_debug_level dbg_lvl, struct fiq_state *state, char *fmt, ...)
{
        enum fiq_debug_level dbg_lvl_req = FIQDBG_INT;
        va_list args;
        char text[17];
        hfnum_data_t hfnum = { .d32 = FIQ_READ(state->dwc_regs_base + 0x408) };
        unsigned long flags;

        local_irq_save(flags);
        local_fiq_disable();
        if((dbg_lvl & dbg_lvl_req) || dbg_lvl == FIQDBG_ERR)
        {
                snprintf(text, 9, " %4d:%1u  ", hfnum.b.frnum/8, hfnum.b.frnum & 7);
                va_start(args, fmt);
                vsnprintf(text+8, 9, fmt, args);
                va_end(args);

                memcpy(buffer + wptr, text, 16);
                wptr = (wptr + 16) % sizeof(buffer);
        }
        local_irq_restore(flags);
}

/*
 * fiq_fsm_transaction_suitable() - Test a QH for compatibility with the FIQ
 * @qh:	pointer to the endpoint's queue head
 *
 * Transaction start/end control flow is grafted onto the existing dwc_otg
 * mechanisms, to avoid spaghettifying the functions more than they already are.
 *
 * Returns: 0 for unsuitable, 1 implies the FIQ can be enabled for this transaction.
 */
int fiq_fsm_transaction_suitable(dwc_otg_qh_t *qh)
{
	int ret = 0;
	if (qh->do_split) {
		/* nonperiodic for now */
		if (qh->ep_type == UE_CONTROL || qh->ep_type == UE_BULK)
			ret = 1;
	}
	return ret;
}

static int nrfiq = 0;


/**
 * fiq_fsm_do_sof() - FSM start-of-frame interrupt handler
 * @state:	Pointer to the state struct passed from banked FIQ mode registers.
 * @num_channels:	set according to the DWC hardware configuration
 *
 * The SOF handler in FSM mode has two functions
 * 1. Hold off SOF from causing schedule advancement in IRQ context if there's
 *    nothing to do
 * 2. Advance certain FSM states that require either a microframe delay, or a microframe
 *    of holdoff.
 *
 * The second part is architecture-specific to mach-bcm2835 -
 * a sane interrupt controller would have a mask register for ARM interrupt sources
 * to be promoted to the nFIQ line, but it doesn't. Instead a single interrupt
 * number (USB) can be enabled. This means that certain parts of the USB specification
 * that require "wait a little while, then issue another packet" cannot be fulfilled with
 * the timing granularity required to achieve optimal throughout. The workaround is to use
 * the SOF "timer" (125uS) to perform this task.
 */
static inline int fiq_fsm_do_sof(struct fiq_state *state, int num_channels)
{
	hfnum_data_t hfnum;
	hfnum.d32 = FIQ_READ(state->dwc_regs_base + HFNUM);
	/* Stub for now. Handle only SOF - FSM advancement comes later */
//	fiq_print(FIQDBG_INT, state, "FIQSOF  ");
//	fiq_print(FIQDBG_INT, state, "%03u%03u", state->np_count, state->np_sent);
//	fiq_print(FIQDBG_INT, state, " %05u:%01u", state->next_sched_frame/8 , state->next_sched_frame % 8 );
	if (state->np_count == state->np_sent &&
			dwc_frame_num_gt(state->next_sched_frame, hfnum.b.frnum)) {
		return 1;
	} else {
		return 0;
	}
}


static inline int fiq_fsm_do_hcintr(struct fiq_state *state, int num_channels, int n /*, struct fiq_perchannel_dma *dma */)
{
	if (state->channel[n].fsm == FIQ_PASSTHROUGH) {
		/* Doesn't belong to us: kick it upstairs */
		return 0;
	}
	switch (state->channel[n].fsm) {
	case FIQ_TEST:
		fiq_print(FIQDBG_INT, state, "HCTEST %d", n);
		state->channel[n].nr_errors = 1;
		return 0;
	default:
		break;
	}
	return 0;
}


/**
 * dwc_otg_fiq_fsm() - Flying State Machine (monster) FIQ
 * @state:		pointer to state struct passed from the banked FIQ mode registers.
 * @num_channels:	set according to the DWC hardware configuration
 * @dma:		pointer to DMA bounce buffers for split transaction slots
 *
 * The FSM FIQ performs the low-level tasks that normally would be performed by the microcode
 * inside an EHCI or similar host controller regarding split transactions. The DWC core
 * interrupts each and every time a split transaction packet is received or sent successfully.
 * This results in either an interrupt storm when everything is working "properly", or
 * the interrupt latency of the system in general breaks time-sensitive periodic split
 * transactions. Pushing the low-level, but relatively easy state machine work into the FIQ
 * solves these problems.
 *
 * Return: void
 */
void dwc_otg_fiq_fsm(struct fiq_state *state, int num_channels /*, struct fiq_perchannel_dma *dma */)
{
	gintsts_data_t gintsts, gintsts_handled;
	gintmsk_data_t gintmsk;
	hfnum_data_t hfnum;
	haint_data_t haint, haint_handled;
	haintmsk_data_t haintmsk;
	int kick_irq = 0;

	gintsts_handled.d32 = 0;
	haint_handled.d32 = 0;

	FLAME_ON(24);
	gintsts.d32 = FIQ_READ(state->dwc_regs_base + GINTSTS);
	gintmsk.d32 = FIQ_READ(state->dwc_regs_base + GINTMSK);
	gintsts.d32 &= gintmsk.d32;

//	fiq_print(FIQDBG_INT, state, "FIQ     ");
//	fiq_print(FIQDBG_INT, state, "%08x", gintsts.d32);
//	fiq_print(FIQDBG_INT, state, "%08x", gintmsk.d32);
//	fiq_print(FIQDBG_INT, state, "%08x", state->gintmsk_saved.d32);
//	fiq_print(FIQDBG_INT, state, "%08x", state->haintmsk_saved.d32);

	if (gintsts.b.sofintr) {
		if (fiq_fsm_do_sof(state, num_channels)) {
			gintsts_handled.b.sofintr = 1;
		} else {
			state->gintmsk_saved.b.sofintr = 0;
			kick_irq |= 1;
		}
	}

	if (gintsts.b.hcintr) {
		int i;
		haint.d32 = FIQ_READ(state->dwc_regs_base + HAINT);
		haintmsk.d32 = FIQ_READ(state->dwc_regs_base + HAINTMSK);
		haint.d32 &= haintmsk.d32;
		haint_handled.d32 = 0;
		for (i=0; i<num_channels; i++) {
			if (haint.b2.chint & (1 << i)) {
				fiq_print(FIQDBG_INT, state, "HCINT  %01d", i);
				if(!fiq_fsm_do_hcintr(state, num_channels, i /*, dma */)) {
					/* HCINT was not handled in FIQ
					 * HAINT is level-sensitive, leading to level-sensitive ginststs.b.hcint bit.
					 * Mask HAINT(i) but keep top-level hcint unmasked.
					 */
					fiq_print(FIQDBG_INT, state, "NO      ");
					state->haintmsk_saved.b2.chint &= ~(1 << i);
					haint_handled.b2.chint |= (1 << i);
				} else {
					/* do_hcintr cleaned up after itself */
					//haint_handled.b2.chint |= (1 << i);
				}
			}
		}

		if (haintmsk.d32 != state->haintmsk_saved.d32) {
			/*
			 * This is necessary to avoid multiple retriggers of the MPHI in the case
			 * where interrupts are held off and HCINTs start to pile up.
			 * Only wake up the IRQ if a new interrupt came in, was not handled and was
			 * masked.
			 */
			haintmsk.d32 &= state->haintmsk_saved.d32;
			FIQ_WRITE(state->dwc_regs_base + HAINTMSK, haintmsk.d32);
			FIQ_WRITE(state->dwc_regs_base + HAINT, haint.d32);
			kick_irq |= 1;
		}
		/* Top-Level interrupt - always handled because it's level-sensitive */
		gintsts_handled.b.hcintr = 1;
	}


	/* Clear the bits in the saved register that were not handled but were triggered. */
	state->gintmsk_saved.d32 &= ~(gintsts.d32 & ~gintsts_handled.d32);

	/* FIQ didn't handle something - mask has changed - write new mask */
	if (gintmsk.d32 != (gintmsk.d32 & state->gintmsk_saved.d32)) {
		gintmsk.d32 &= state->gintmsk_saved.d32;
		FIQ_WRITE(state->dwc_regs_base + GINTMSK, gintmsk.d32);
		kick_irq |= 1;
	}

	if (gintsts_handled.d32) {
		/* Only applies to edge-sensitive bits in GINTSTS */
		FIQ_WRITE(state->dwc_regs_base + GINTSTS, gintsts_handled.d32);
	}

	/* We got an interrupt, didn't handle it. */
	if (kick_irq) {
		state->mphi_int_count++;
//		fiq_print(FIQDBG_INT, state, "FIQ->IRQ");
//		fiq_print(FIQDBG_INT, state, "%08x", gintmsk.d32);
//		fiq_print(FIQDBG_INT, state, "%08x", state->gintmsk_saved.d32);
//		fiq_print(FIQDBG_INT, state, "%08x", state->haintmsk_saved.d32);
		FIQ_WRITE(state->mphi_regs.outdda, (int) state->dummy_send);
		FIQ_WRITE(state->mphi_regs.outddb, (1<<29));

	}
	nrfiq++;
	FLAME_OFF(24);
	mb();
}


/**
 * dwc_otg_fiq_nop() - FIQ "lite"
 * @state:	pointer to state struct passed from the banked FIQ mode registers.
 *
 * The "nop" handler does not intervene on any interrupts other than SOF.
 * It is limited in scope to deciding at each SOF if the IRQ SOF handler (which deals
 * with non-periodic/periodic queues) needs to be kicked.
 *
 * This is done to hold off the SOF interrupt, which occurs at a rate of 8000 per second.
 *
 * Return: void
 */
void dwc_otg_fiq_nop(struct fiq_state *state)
{
	gintsts_data_t gintsts, gintsts_handled;
	gintmsk_data_t gintmsk;
	hfnum_data_t hfnum;

	FLAME_ON(24);
	hfnum.d32 = FIQ_READ(state->dwc_regs_base + HFNUM);
	gintsts.d32 = FIQ_READ(state->dwc_regs_base + GINTSTS);
	gintmsk.d32 = FIQ_READ(state->dwc_regs_base + GINTMSK);
	gintsts.d32 &= gintmsk.d32;
	gintsts_handled.d32 = 0;

	fiq_print(FIQDBG_INT, state, "FIQ     ");
	fiq_print(FIQDBG_INT, state, "%08x", gintsts.d32);
	fiq_print(FIQDBG_INT, state, "%08x", gintmsk.d32);
	fiq_print(FIQDBG_INT, state, "%08x", state->gintmsk_saved.d32);

	if (gintsts.b.sofintr) {
//		fiq_print(FIQDBG_INT, state, "SOF     ");
//		fiq_print(FIQDBG_INT, state, "C %05d ", state->np_count);
//		fiq_print(FIQDBG_INT, state, "S %05d ", state->np_sent);
//		fiq_print(FIQDBG_INT, state, "P%01d %05d", dwc_frame_num_gt(state->next_sched_frame, hfnum.b.frnum), state->next_sched_frame);
		if (state->np_count == state->np_sent && dwc_frame_num_gt(state->next_sched_frame, hfnum.b.frnum)) {
			/* SOF handled, no work to do, just ACK interrupt */
			gintsts_handled.b.sofintr = 1;
		} else {
			/* Kick IRQ */
			state->gintmsk_saved.b.sofintr = 0;
		}
	}

	/* Reset handled interrupts */
	if(gintsts_handled.d32) {
		FIQ_WRITE(state->dwc_regs_base + GINTSTS, gintsts_handled.d32);
	}

	/* Clear the bits in the saved register that were not handled but were triggered. */
	state->gintmsk_saved.d32 &= ~(gintsts.d32 & ~gintsts_handled.d32);

	/* We got an interrupt, didn't handle it and want to mask it */
	if (~(state->gintmsk_saved.d32)) {
		state->mphi_int_count++;
//		fiq_print(FIQDBG_INT, state, "FIQ->IRQ");
		gintmsk.d32 &= state->gintmsk_saved.d32;
//		fiq_print(FIQDBG_INT, state, "%08x", gintmsk.d32);
//		fiq_print(FIQDBG_INT, state, "%08x", state->gintmsk_saved.d32);
		FIQ_WRITE(state->dwc_regs_base + GINTMSK, gintmsk.d32);

		/* An unmasked, unhandled interrupt has occurred, kick IRQ */
		FIQ_WRITE(state->mphi_regs.outdda, (int) state->dummy_send);
		FIQ_WRITE(state->mphi_regs.outddb, (1<<29));

	}
	nrfiq++;
//	if(nrfiq > 10000) {
//		BUG();
//	}
	FLAME_OFF(24);
	mb();
}



