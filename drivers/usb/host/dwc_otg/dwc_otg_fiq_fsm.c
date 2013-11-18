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
 *   processor core - there is no locking between the FIQ and IRQ (aside from
 *   local_fiq_disable)
 *
 */

#include "dwc_otg_fiq_fsm.h"


char buffer[1000*16];
int wptr;
void _fiq_print(enum fiq_debug_level dbg_lvl, volatile struct fiq_state *state, char *fmt, ...)
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


static int nrfiq = 0;

/**
 * fiq_fsm_restart_channel() - Poke channel enable bit for a non-periodic transaction
 * @channel: channel to re-enable
 */
static inline void fiq_fsm_restart_channel(struct fiq_state *st, int n)
{
	hcchar_data_t hcchar = { .d32 = FIQ_READ(st->dwc_regs_base + HC_START + (HC_OFFSET * n) + HCCHAR) };
	hcchar.b.chen = 1;
	FIQ_WRITE(st->dwc_regs_base + HC_START + (HC_OFFSET * n) + HCCHAR, hcchar.d32);
	fiq_print(FIQDBG_INT, st, "F HCGO %01d", n);
	fiq_print(FIQDBG_INT, st, "%08x", hcchar.d32);
}

/**
 * fiq_fsm_setup_np_csplit() - Prepare a host channel for a np csplit transaction stage
 * @st: Pointer to the channel's state
 * @n : channel number
 *
 * Change host channel registers to perform a complete-split transaction. Being mindful of the
 * endpoint direction, set control regs up correctly.
 */
static inline void notrace fiq_fsm_setup_np_csplit(struct fiq_state *st, int n)
{
	hcsplt_data_t hcsplt = { .d32 = FIQ_READ(st->dwc_regs_base + HC_START + (HC_OFFSET * n) + HCSPLT) };
	hctsiz_data_t hctsiz = { .d32 = FIQ_READ(st->dwc_regs_base + HC_START + (HC_OFFSET * n) + HCTSIZ) };
	
	hcsplt.b.compsplt = 1;
	if (st->channel[n].hcchar_copy.b.epdir == 1) {
		// If IN, the CSPLIT result contains the data or a hub handshake. hctsiz = maxpacket.
		hctsiz.b.xfersize = st->channel[n].hctsiz_copy.b.xfersize;
	} else {
		// If OUT, the CSPLIT result contains handshake only.
		hctsiz.b.xfersize = 0;
	}	
	FIQ_WRITE(st->dwc_regs_base + HC_START + (HC_OFFSET * n) + HCSPLT, hcsplt.d32);
	FIQ_WRITE(st->dwc_regs_base + HC_START + (HC_OFFSET * n) + HCTSIZ, hctsiz.d32);
	fiq_print(FIQDBG_INT, st, "FIQ CS %01d", n);
	fiq_print(FIQDBG_INT, st, "%08x", hcsplt.d32);
	fiq_print(FIQDBG_INT, st, "%08x", hctsiz.d32);
	mb();
}


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
static inline int notrace fiq_fsm_do_sof(struct fiq_state *state, int num_channels)
{
	hfnum_data_t hfnum = { .d32 = FIQ_READ(state->dwc_regs_base + HFNUM) };
	int n; 
	/* Stub for now. Handle only SOF - FSM advancement comes later */
//	fiq_print(FIQDBG_INT, state, "FIQSOF  ");
//	fiq_print(FIQDBG_INT, state, "%03u%03u", state->np_count, state->np_sent);
//	fiq_print(FIQDBG_INT, state, " %05u:%01u", state->next_sched_frame/8 , state->next_sched_frame % 8 );
	for (n = 0; n < num_channels; n++) {
		switch (state->channel[n].fsm) {
		case FIQ_NP_SSPLIT_RETRY:
		case FIQ_NP_IN_CSPLIT_RETRY:
		case FIQ_NP_OUT_CSPLIT_RETRY:
			// Kick the HC
			fiq_fsm_restart_channel(state, n);
			break;
		default:
			break;
		}
	}

	if (state->np_count == state->np_sent &&
			dwc_frame_num_gt(state->next_sched_frame, hfnum.b.frnum)) {
		return 1;
	} else {
		return 0;
	}
}


/**
 * fiq_fsm_do_hcintr() - FSM host channel interrupt handler
 * @state: Pointer to the FIQ state struct
 * @num_channels: Number of channels as per hardware config
 * @n: channel for which HAINT(i) was raised
 * 
 * An important property is that only the CHHLT interrupt is unmasked. Unfortunately, AHBerr is as well.
 */
static inline int notrace fiq_fsm_do_hcintr(struct fiq_state *state, int num_channels, int n)
{
	hcint_data_t hcint;
	hcintmsk_data_t hcintmsk;
	int handled = 0;
	int restart = 0;
	struct fiq_channel_state *st = &state->channel[n];

	hcint.d32 = FIQ_READ(state->dwc_regs_base + HC_START + (HC_OFFSET * n) + HCINT);
	hcintmsk.d32 = FIQ_READ(state->dwc_regs_base + HC_START + (HC_OFFSET * n) + HCINTMSK);
	
	if (st->fsm != FIQ_PASSTHROUGH) {
		fiq_print(FIQDBG_INT, state, "HCFIQ %01d", n);
		fiq_print(FIQDBG_INT, state, "ST:%02d   ", st->fsm);
		fiq_print(FIQDBG_INT, state, "%08x", hcint.d32);
		fiq_print(FIQDBG_INT, state, "%08x", hcintmsk.d32);
		nrfiq++;
		if (nrfiq > 10) {
			int *derp = NULL;
			*derp = 1;
		}
	}
	
	switch (st->fsm) {
	
	case FIQ_PASSTHROUGH:
		/* doesn't belong to us, kick it upstairs */
		break;

#ifdef FIQ_DEBUG
	case FIQ_TEST:
		fiq_print(FIQDBG_INT, state, "HCTEST %d", n);
		st->nr_errors = 1;
		break;
#endif
	/* Non-periodic state groups */
	case FIQ_NP_SSPLIT_STARTED:
	case FIQ_NP_SSPLIT_RETRY: // SOF kicked the channel due to a prior NAK response, or retry a previous HS error.
		/* Got a HCINT for a NP SSPLIT. Expected ACK / NAK / fail */
		if (hcint.b.ack) {
			/* SSPLIT complete. For OUT, the data has been sent. For IN, the LS transaction
			 * will start shortly. SOF needs to kick the transaction to prevent a NYET flood. */
			if(st->hcchar_copy.b.epdir == 1)
				st->fsm = FIQ_NP_IN_CSPLIT_RETRY;
			else
				st->fsm = FIQ_NP_OUT_CSPLIT_RETRY;
			st->nr_errors = 0;
			handled = 1;
			fiq_fsm_setup_np_csplit(state, n);
			break;
		} else if (hcint.b.nak) {
			// No buffer space in TT. Retry on a uframe boundary.
			st->fsm = FIQ_NP_SSPLIT_RETRY;
			handled = 1;
			break;
		} else if (hcint.b.xacterr) {
			// The only other one we care about is xacterr. This implies HS bus error - retry.
			st->nr_errors++;
			st->fsm = FIQ_NP_SSPLIT_RETRY;
			if (st->nr_errors >= 3) {
				st->fsm = FIQ_NP_SPLIT_HS_ABORTED;
				handled = 0;
				restart = 0;
			} else {
				handled = 1;
				restart = 1;
			}
			break;
		} else {
			st->fsm = FIQ_NP_SPLIT_LS_ABORTED;
			handled = 0;
			restart = 0;
			break;
		}
		break;
		
	case FIQ_NP_IN_CSPLIT_RETRY:
		/* Received a CSPLIT done interrupt.
		 * Expected Data/NAK/STALL/NYET for IN. */
		if (hcint.b.xfercomp) {
			/* For IN, data is present. */
			st->fsm = FIQ_NP_SPLIT_DONE;
		} else if (hcint.b.nak) {
			/* no endpoint data. Punt it upstairs */
			st->fsm = FIQ_NP_SPLIT_DONE;
		} else if (hcint.b.nyet) {
			/* CSPLIT NYET - retry on a uframe boundary. */
			handled = 1;
			restart = 0;
			st->nr_errors = 0;
		} else if (hcint.b.datatglerr) {
			/* data toggle errors do not set the xfercomp bit. */
			st->fsm = FIQ_NP_SPLIT_LS_ABORTED;
		} else if (hcint.b.xacterr) {
			/* HS error. Retry immediate */
			st->fsm = FIQ_NP_IN_CSPLIT_RETRY;
			st->nr_errors++;
			if (st->nr_errors >= 3) {
				st->fsm = FIQ_NP_SPLIT_HS_ABORTED;
				handled = 0;
				restart = 0;
			} else {
				handled = 1;
				restart = 1;
			}
		} else if (hcint.b.stall) {
			/* A STALL implies either a LS bus error or a genuine STALL. */
			st->fsm = FIQ_NP_SPLIT_LS_ABORTED;
		} else {
			// Something unexpected happened. AHBerror or babble perhaps. Let the IRQ deal with it.
			st->fsm = FIQ_NP_SPLIT_HS_ABORTED;
		}
		break;
	
	case FIQ_NP_OUT_CSPLIT_RETRY:
		/* Received a CSPLIT done interrupt.
		 * Expected ACK/NAK/STALL/NYET for OUT. Xfercomp? */
		if (hcint.b.xfercomp) {
			//FIXME: the response is an ACK handshake - what actually happens?
			st->fsm = FIQ_NP_SPLIT_DONE;
		} else if (hcint.b.nak) {
			// The HCD will implement the holdoff on frame boundaries.
			st->fsm = FIQ_NP_SPLIT_DONE;
		} else if (hcint.b.nyet) {
			// Hub still processing.
			st->fsm = FIQ_NP_OUT_CSPLIT_RETRY;
			handled = 1;
			/* If we are talking to a FS device, we can retry immediate without
			 * too much interrupt spam - at most 5 FIQ retriggers will happen.
			 * LS must wait on a uFrame boundary.
			 */
			restart = 0;
		} else if (hcint.b.xacterr) { 
			/* HS error. retry immediate */
			st->fsm = FIQ_NP_OUT_CSPLIT_RETRY;
			st->nr_errors++;
			if (st->nr_errors >= 3) {
				st->fsm = FIQ_NP_SPLIT_HS_ABORTED;
			} else {
				handled = 1;
				restart = 1;
			}
		} else if (hcint.b.stall) { 
			/* LS bus error or genuine stall */
			st->fsm = FIQ_NP_SPLIT_LS_ABORTED;
		}
		break;
	
	/* Periodic split states */
	case FIQ_PER_SSPLIT_STARTED:
	case FIQ_PER_CSPLIT_NYET1:
	case FIQ_PER_CSPLIT_POLL:
	
	default:
		break;
	}
		
	/* We always clear our respective channel interrupts before restarting the channel. */

	// if (!handled) {
		// /* if we are punting this to the IRQ, save the interrupts and mask. */
		// st->hcint_copy.d32 = hcint.d32;
		// st->hcintmsk_copy.d32 = hcintmsk.d32;
		// hcintmsk.d32 = 0;
		// FIQ_WRITE(state->dwc_regs_base + HC_START + (HC_OFFSET * n) + HCINTMSK, hcintmsk.d32);
	// }
	if (handled) {
		FIQ_WRITE(state->dwc_regs_base + HC_START + (HC_OFFSET * n) + HCINT, hcint.d32);
	}

	if (restart) {
		/* Restart always implies handled. */
		fiq_print(FIQDBG_INT, state, "FRESTART");
		fiq_fsm_restart_channel(state, n);
	}
	if (st->fsm != FIQ_PASSTHROUGH)
		fiq_print(FIQDBG_INT, state, "FSMOUT %01d", st->fsm);
	return handled;
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
void notrace dwc_otg_fiq_fsm(struct fiq_state *state, int num_channels)
{
	gintsts_data_t gintsts, gintsts_handled;
	gintmsk_data_t gintmsk;
	//hfnum_data_t hfnum;
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
				} else {
					/* do_hcintr cleaned up after itself, but clear haint */
					fiq_print(FIQDBG_INT, state, "YES     ");
					haint_handled.b2.chint |= (1 << i);
				}
			}
		}
		
		if (haint_handled.b2.chint) {
			FIQ_WRITE(state->dwc_regs_base + HAINT, haint_handled.d32);
		}

		if (haintmsk.d32 != (haintmsk.d32 & state->haintmsk_saved.d32)) {
			/*
			 * This is necessary to avoid multiple retriggers of the MPHI in the case
			 * where interrupts are held off and HCINTs start to pile up.
			 * Only wake up the IRQ if a new interrupt came in, was not handled and was
			 * masked.
			 */
			// fiq_print(FIQDBG_INT, state, "KICK HA ");
			// fiq_print(FIQDBG_INT, state, "%08x", haintmsk.d32);
			// fiq_print(FIQDBG_INT, state, "%08x", state->haintmsk_saved.d32);
			haintmsk.d32 &= state->haintmsk_saved.d32;
			FIQ_WRITE(state->dwc_regs_base + HAINTMSK, haintmsk.d32);
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
		fiq_print(FIQDBG_INT, state, "KICKGINT");
		fiq_print(FIQDBG_INT, state, "%08x", gintmsk.d32);
		fiq_print(FIQDBG_INT, state, "%08x", state->gintmsk_saved.d32);
		kick_irq |= 1;
	}

	if (gintsts_handled.d32) {
		/* Only applies to edge-sensitive bits in GINTSTS */
		FIQ_WRITE(state->dwc_regs_base + GINTSTS, gintsts_handled.d32);
	}

	/* We got an interrupt, didn't handle it. */
	if (kick_irq) {
		state->mphi_int_count++;
		 fiq_print(FIQDBG_INT, state, "FIQ->IRQ");
		 fiq_print(FIQDBG_INT, state, "%08x", gintmsk.d32);
		 fiq_print(FIQDBG_INT, state, "%08x", state->gintmsk_saved.d32);
		 fiq_print(FIQDBG_INT, state, "%08x", state->haintmsk_saved.d32);
		FIQ_WRITE(state->mphi_regs.outdda, (int) state->dummy_send);
		FIQ_WRITE(state->mphi_regs.outddb, (1<<29));

	}

	FLAME_OFF(24);
//	if (nrfiq > 100) {
//		int *derp = NULL;
//		*derp = 1;
//	}
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
void notrace dwc_otg_fiq_nop(struct fiq_state *state)
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
		/* Force a clear before another dummy send */
		FIQ_WRITE(state->mphi_regs.intstat, (1<<29));
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



