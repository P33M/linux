/*
 * dwc_otg_fiq_fsm.c
 *
 *  Created on: 21 Aug 2013
 *      Author: jonathan
 */

#include "dwc_otg_fiq_fsm.h"

//
//void dwc_otg_fiq_fsm(int num_channels, struct fiq_state *state, struct fiq_perchannel_dma *dma)
//{
//
//}
//

//int fiq_fsm_do_sof(int num_channels, struct fiq_state *state)
//{
//	return 0;
//}

//
//int fiq_fsm_do_hcintr(int num_channels, int channel, struct fiq_state *state,
//			 struct fiq_perchannel_dma *dma)
//{
//	return 0;
//}
//

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


/* IRQ communication
 * Because both level and edge triggered interrupts are present in each of the registers
 * all the way down to the piece of hardware that triggered it (where they edge-triggered).
 * Because unhandled interrupts need to be communicated back to the IRQ, the only way to accomplish
 * this is to mask the triggered interrupt and pass both the & of the new mask with the old mask and the saved reg state.
 * The IRQ should then reenable the masked interrupts that have been saved after it has finished handling them.
 *
 */


static int nrfiq = 0;

static inline int fiq_fsm_do_sof(int num_channels, struct fiq_state *state)
{
	hfnum_data_t hfnum;
	hfnum.d32 = FIQ_READ(state->dwc_regs_base + HFNUM);
	/* Stub for now. Handle only SOF - FSM advancement comes later */
	if (state->np_count == state->np_sent &&
			dwc_frame_num_gt(state->next_sched_frame, hfnum.b.frnum)) {
		return 1;
	} else {
		return 0;
	}
}


static inline int fiq_fsm_do_hcintr(int num_channels, int n, struct fiq_state *state /*, struct fiq_perchannel_dma *dma */) {
	return 0;
}

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

	fiq_print(FIQDBG_INT, state, "FIQ     ");
	fiq_print(FIQDBG_INT, state, "%08x", gintsts.d32);
	fiq_print(FIQDBG_INT, state, "%08x", gintmsk.d32);
	fiq_print(FIQDBG_INT, state, "%08x", state->gintmsk_saved.d32);
	fiq_print(FIQDBG_INT, state, "%08x", state->haintmsk_saved.d32);

	if (gintsts.b.sofintr) {
		if (fiq_fsm_do_sof(num_channels, state)) {
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
		for (i=0; i<num_channels; i++) {
			if (haint.b2.chint & (1 << i)) {
				fiq_print(FIQDBG_INT, state, "HCINT  %01d", i);
				if(!fiq_fsm_do_hcintr(num_channels, i, state /*, dma */)) {
					/* Not handled in FIQ
					 * HAINT is level-sensitive, leading to level-sensitive HCINT bit.
					 * Mask HAINT(i) but keep top-level hcint unmasked.
					 */
					fiq_print(FIQDBG_INT, state, "NO      ");
					state->haintmsk_saved.b2.chint &= ~(1 << i);
				} else {
					/* not necessary */
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
			kick_irq |= 1;
		}
		/* Top-Level interrupt - always handled because it's level-sensitive */
		gintsts_handled.b.hcintr = 1;
	}


	/* Clear the bits in the saved register that were not handled but were triggered. */
	state->gintmsk_saved.d32 &= ~(gintsts.d32 & ~gintsts_handled.d32);

	/* FIQ didn't handle something - mask has changed - write new mask */
	if (gintmsk.d32 != state->gintmsk_saved.d32) {
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
		fiq_print(FIQDBG_INT, state, "FIQ->IRQ");
		fiq_print(FIQDBG_INT, state, "%08x", gintmsk.d32);
		fiq_print(FIQDBG_INT, state, "%08x", state->gintmsk_saved.d32);
		fiq_print(FIQDBG_INT, state, "%08x", state->haintmsk_saved.d32);
		FIQ_WRITE(state->mphi_regs.outdda, (int) state->dummy_send);
		FIQ_WRITE(state->mphi_regs.outddb, (1<<29));

	}
	nrfiq++;
}


/* Basic version of FIQ - only holds off SOF */
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



