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
 * this is to mask the triggered interrupt and pass both the and of the new mask with the old mask and the saved reg state.
 * The IRQ should then reenable the masked interrupts that have been saved after it has finished handling them.
 * This mechanism
 */


static int nrfiq = 0;

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
		fiq_print(FIQDBG_INT, state, "SOF     ");
		fiq_print(FIQDBG_INT, state, " %05d:%01u", hfnum.b.frnum/8, hfnum.b.frnum & 7);
		/* Basically, replication of the first implementation that just nop'd SOFs */
		if (state->np_count == state->np_sent && dwc_frame_num_gt(state->next_sched_frame, hfnum.b.frnum)) {
			/* SOF handled, no work to do, just ACK interrupt */
			gintsts_handled.b.sofintr = 1;
		} else {
			/* IRQ needs to kick it */
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
		fiq_print(FIQDBG_INT, state, "FIQ->IRQ");
		gintmsk.d32 &= state->gintmsk_saved.d32;
		fiq_print(FIQDBG_INT, state, "%08x", gintmsk.d32);
		fiq_print(FIQDBG_INT, state, "%08x", state->gintmsk_saved.d32);
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
