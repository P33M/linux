/*
 * dwc_otg_fiq_fsm.h
 *
 *  Created on: 21 Aug 2013
 *      Author: Jonathan Bell
 *      		jonathan@raspberrypi.org
 *      Structure and data definitions for the FSM-mode FIQ
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

/* FIQ-ified register definitions */
#define GINTSTS		0x014
#define GINTMSK		0x018
#define HFNUM		0x408
#define HAINT		0x414
#define HAINTMSK	0x418
#define HPRT0		0x440
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
#ifdef FIQ_DEBUG
	FIQ_CHAN_DISABLED
#endif
};

struct fiq_stack {
	int magic1;
	uint8_t stack[2048];
	int magic2;
};

//struct fiq_channel_state {
//	enum fiq_fsm_state fsm;
//	unsigned int nr_errors;
//	unsigned int hub_addr;
//	unsigned int port_addr;
//	/* in/out for communicating number of dma buffers used, or number of ISOC to do */
//	unsigned int nrpackets;
//	/* copies of registers - in/out communication from/to IRQ handler */
//	hcchar_data_t hcchar_copy;
//	hcsplt_data_t hcsplt_copy;
//	hcint_data_t hcint_copy;
//	hcintmsk_data_t hcintmsk_copy;
//	hctsiz_data_t hctsiz_copy;
//};

struct fiq_state {
	/* To allow the mphi peripheral to send stuff */
	mphi_regs_t mphi_regs;
	int mphi_int_count;
	/* vaddr of DWC regs */
	void *dwc_regs_base;
	void *dummy_send;
	/* for communicating unhandled interrupts back to the IRQ on FIQ exit */
	gintmsk_data_t gintmsk_saved;
//	haint_data_t haint_saved;
	unsigned int np_count;
	unsigned int np_sent;
	unsigned int next_sched_frame;
#ifdef FIQ_DEBUG
	char * buffer;
	unsigned int bufsiz;
#endif
	/* HCD will allocate fiq_channel_state[nr_channels] */
//	struct fiq_channel_state channel[0];
};

/* set of 188-byte DMA bounce buffers for split transactions. Simple array used for speed. */

//struct fiq_dma_slot {
//	u8 buf[188];
//	/* number of bytes in the buffer actually used */
//	int len;
//};

//struct fiq_perchannel_dma {
//	/* we need at most 6 slots (max possible split-isoc OUT size) */
//	struct fiq_dma_slot slot[6];
//	/* for use mid-transaction, or to report that fewer than nrpackets were transferred */
//	int index;
//};


//struct fiq_stub_glue {
//	/* Hack: grab the linker-generated FIQ size for use in set_fiq_handler() */
//	u32 length;
//	u32 code[0];
//};

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
//int fiq_fsm_do_sof(int num_channels, struct fiq_state *state);
//
//int fiq_fsm_do_hcintr(int num_channels, int channel, struct fiq_state *state,
//			struct fiq_perchannel_dma *dma);

extern void dwc_otg_fiq_nop(struct fiq_state *state);

#endif /* DWC_OTG_FIQ_FSM_H_ */
