/*
 * dwc_otg_fiq_fsm.h - Finite state machine FIQ header definitions
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
 */

#ifndef DWC_OTG_FIQ_FSM_H_
#define DWC_OTG_FIQ_FSM_H_

#include "dwc_otg_regs.h"
#include "dwc_otg_cil.h"
#include "dwc_otg_hcd.h"
#include <linux/kernel.h>
#include <linux/irqflags.h>
#include <linux/string.h>
#include <asm/barrier.h>

#define FLAME_ON(x) \
do {                                            \
int gpioreg;                                    \
                                                \
gpioreg = readl(__io_address(0x20200000+0x8));  \
gpioreg &= ~(7 << (x-20)*3);                    \
gpioreg |= 0x1 << (x-20)*3;                     \
writel(gpioreg, __io_address(0x20200000+0x8));  \
                                                \
writel(1<<x, __io_address(0x20200000+(0x1C)));  \
} while (0)

#define FLAME_OFF(x)    \
do {            \
writel(1<<x, __io_address(0x20200000+(0x28)));  \
} while (0)

#define FIQ_WRITE(_addr_,_data_) (*(volatile unsigned int *) (_addr_) = (_data_))
#define FIQ_READ(_addr_) (*(volatile unsigned int *) (_addr_))

/* FIQ-ified register definitions. Offsets are from dwc_regs_base. */
#define GINTSTS		0x014
#define GINTMSK		0x018
#define HFNUM		0x408
#define HAINT		0x414
#define HAINTMSK	0x418
#define HPRT0		0x440
/* HC_regs start from an offset of 0x500 */
#define HC_START	0x500
#define HC_OFFSET	0x020

#define HC_DMA		0x514

#define HCCHAR		0x00
#define HCSPLT		0x04
#define HCINT		0x08
#define HCINTMSK	0x0C
#define HCTSIZ		0x10

#define ISOC_XACTPOS_ALL 	0x11
#define ISOC_XACTPOS_BEGIN	0x10
#define ISOC_XACTPOS_MID	0x00
#define ISOC_XACTPOS_END	0x01

typedef struct {
	volatile void* base;
	volatile void* ctrl;
	volatile void* outdda;
	volatile void* outddb;
	volatile void* intstat;
} mphi_regs_t;


enum fiq_debug_level {
	FIQDBG_SCHED = (1 << 0),
	FIQDBG_INT   = (1 << 1),
	FIQDBG_ERR   = (1 << 2),
	FIQDBG_PORTHUB = (1 << 3),
};

struct fiq_state;

extern void _fiq_print (enum fiq_debug_level dbg_lvl, struct fiq_state *state, char *fmt, ...);
#if 1
#define fiq_print _fiq_print
#else
#define fiq_print(x, y, ...)
#endif

extern bool fiq_enable, nak_holdoff_enable, fiq_fsm_enable;


/**
 * enum fiq_fsm_state - The FIQ FSM states.
 *
 * This is the "core" of the FIQ FSM. Broadly, the FSM states follow the
 * USB2.0 specification for host responses to various transaction states.
 * There are modifications to this host state machine because of a variety of
 * quirks and limitations in the dwc_otg hardware.
 *
 * The fsm state is also used to communicate back to the driver on completion of
 * a split transaction. The end states are used in conjunction with the interrupts
 * raised by the final transaction.
 */
enum fiq_fsm_state {
	FIQ_PASSTHROUGH = (1<<1),
	FIQ_SSPLIT_STARTED = (1<<2),
	FIQ_SSPLIT_RETRY = (1<<3),
	FIQ_SSPLIT_DONE = (1<<4),
	FIQ_PER_CSPLIT_WAIT = (1<<5),
	FIQ_PER_CSPLIT_POLL = (1<<6),
	FIQ_CSPLIT_FIRST_NYET = (1<<7),
	FIQ_NP_SPLIT_FIN = (1<<8),
	FIQ_PER_SPLIT_FIN = (1<<9),
	FIQ_NP_CSPLIT_NYET = (1<<10),
	FIQ_SPLIT_ABORTED = (1<<31),
	FIQ_TEST = (1<<16),
#ifdef FIQ_DEBUG
	FIQ_CHAN_DISABLED
#endif
};

struct fiq_stack {
	int magic1;
	uint8_t stack[2048];
	int magic2;
};

/**
 * struct fiq_channel_state - FIQ state machine storage
 * @fsm:	Current state of the channel as understood by the FIQ
 * @nr_errors:	Number of transaction errors on this split-transaction
 * @hub_addr:   SSPLIT/CSPLIT destination hub
 * @port_addr:  SSPLIT/CSPLIT destination port - always 1 if single TT hub
 * @nrpackets:  For isoc OUT, the number of split-OUT packets to transmit. For
 * 		split-IN, number of CSPLIT data packets that were received.
 * @hcchar_copy:
 * @hcsplt_copy:
 * @hcintmsk_copy:
 * @hctsiz_copy:	Copies of the host channel registers. For use as scratch.
 *
 * The fiq_channel_state is state storage between interrupts for a host channel. The
 * FSM state is stored here. Members of this structure must only be set up by the
 * driver prior to enabling the FIQ for this host channel, and not touched until the FIQ
 * has updated the state to either a COMPLETE state group or ABORT state group.
 */

struct fiq_channel_state {
	enum fiq_fsm_state fsm;
	unsigned int nr_errors;
	unsigned int hub_addr;
	unsigned int port_addr;
	/* in/out for communicating number of dma buffers used, or number of ISOC to do */
	unsigned int nrpackets;
	/* copies of registers - in/out communication from/to IRQ handler and for ease of channel setup */
	hcchar_data_t hcchar_copy;
	hcsplt_data_t hcsplt_copy;
	hcint_data_t hcint_copy;
	hcintmsk_data_t hcintmsk_copy;
	hctsiz_data_t hctsiz_copy;
};

/**
 * struct fiq_state - top-level FIQ state machine storage
 * @mphi_regs:		virtual address of the MPHI peripheral register file
 * @dwc_regs_base:	virtual address of the base of the DWC core register file
 * @dummy_send:		Scratch area for sending a fake message to the MPHI peripheral
 * @gintmsk_saved:	Top-level mask of interrupts that the FIQ has not handled.
 * 			Used for determining which interrupts fired to set off the IRQ handler.
 * @haintmsk_saved:	Mask of interrupts from host channels that the FIQ did not handle internally.
 * @np_count:		Non-periodic transactions in the active queue
 * @np_sent:		Count of non-periodic transactions that have completed
 * @next_sched_frame:	For periodic transactions handled by the driver's SOF-driven queuing mechanism,
 * 			this is the next frame on which a SOF interrupt is required. Used to hold off
 * 			passing SOF through to the driver until necessary.
 * @channel[n]:		Per-channel FIQ state. Allocated during init depending on the number of host
 * 			channels configured into the core logic.
 *
 * This is passed as the first argument to the dwc_otg_fiq_fsm top-level FIQ handler from the asm stub.
 * It contains top-level state information.
 */
struct fiq_state {
	mphi_regs_t mphi_regs;
	int mphi_int_count;
	void *dwc_regs_base;
	void *dummy_send;
	gintmsk_data_t gintmsk_saved;
	haintmsk_data_t haintmsk_saved;
	unsigned int np_count;
	unsigned int np_sent;
	unsigned int next_sched_frame;
#ifdef FIQ_DEBUG
	char * buffer;
	unsigned int bufsiz;
#endif
	struct fiq_channel_state channel[0];
};

/**
 * struct fiq_dma_slot - a 188-byte DMA bounce buffer
 * @buf:	Does what it says on the tin. 188 bytes is the maximum per-microframe split transaction
 * 		data size.
 * @len:	For OUT transfers, this is to be programmed into the host channel
 * 		HCTSIZ register. For IN transfers, this is the actual amount of data received.
 */
struct fiq_dma_slot {
	u8 buf[188];
	int len;
};


/**
 * struct fiq_perchannel_dma - coherent DMA bounce buffer for a single host channel
 * @slot[6]:	There can be up to 6 SSPLIT or CSPLIT transactions carrying data in/out
 * 		per full-speed frame. Data to be transmitted OUT is written here by the driver
 * 		for the FIQ to point the host channel's DMA to for each packet,
 * 		and data transmitted IN is written to each of the slots in turn by
 * 		the host channel's DMA.
 * @index:	For OUT, the number of slots that have been filled with data to transmit.
 * 		For IN, the number of slots that have received data.
 *
 * Each slot is 188 bytes of coherently allocated memory. The amount of data in each slot is variable,
 * see the fiq_dma_slot struct.
 */
struct fiq_perchannel_dma {
	/* we need at most 6 slots (max possible split-isoc OUT size) */
	struct fiq_dma_slot slot[6];
	/* for use mid-transaction, or to report that fewer than nrpackets were transferred */
	int index;
};


/* HCD will dma_alloc_coherent the required memory, all-up DMA size is 9kiB for 8 host channels */

/* must be called FIQ disabled if not from FIQ context */
//static inline int fiq_tt_in_use(int channel, int num_channels, struct fiq_channel_state *state)
//{
//	int ret = 0;
//	int i;
//	int hub_addr = state[channel]->hub_addr;
//	int port_addr = state[channel]->port_addr;
//
//	for (i=0; i < num_channels; i++) {
//		if (i == channel)
//			continue;
//
//		/* Transfer isn't actually in progress or is not a split */
//		if (state[i]->fsm & (FIQ_PASSTHROUGH | FIQ_NP_SPLIT_FIN | FIQ_PER_SPLIT_FIN | FIQ_SPLIT_ABORTED))
//			continue;
//
//		if(num_channels & 0xF0 )
//
//		if (state[i]->hub_addr == hub_addr &&
//				state[i]->port_addr == port_addr)
//			ret = 1;
//			break;
//	}
//	return ret;
//}
//
//static inline int fiq_transfer_ok(hcint_data_t hcint)
//{
//	return (hcint.d32 & ((1<<2) | (1<<7) | (1<<8) | (1<<9))) ? 0 : 1;
//}
//
//static inline int fiq_full_frame_rollover(hfnum_data_t hfnum)
//{
//	return ((hfnum.b.frnum & 0x7) == 0) ? 1 : 0;
//}
//
//
//static inline int fiq_too_late(hfnum_data_t hfnum)
//{
//	return (hfnum.b.frnum & 0x7 >= 5) ? 1 : 0;
//}
//
//static inline int fiq_is_periodic(hcchar_data_t hcchar)
//{
//	return (hcchar.d32 & (1<<18)) ? 1 : 0;
//}
//
//static inline int fiq_ep_is_out(hcchar_data_t hcchar)
//{
//	return (hcchar.d32 & (1<<15)) ? 0 : 1;
//}
//
//static inline int fiq_isoc_out(hcchar_data_t hcchar)
//{
//	if ((hcchar.d32 & ((1<<19) | (1<<18))) == 0x4000)
//		if(hcchar.d32 & (1<<15))
//			return 1;
//	return 0;
//}
//
//void dwc_otg_fiq_fsm(int num_channels, struct fiq_state *state, struct fiq_perchannel_dma *dma);
//
extern void dwc_otg_fiq_fsm(struct fiq_state *state, int num_channels);

extern void dwc_otg_fiq_nop(struct fiq_state *state);

#endif /* DWC_OTG_FIQ_FSM_H_ */
