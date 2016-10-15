/*
 * Copyright (c) 2004-2011 Atheros Communications Inc.
 * Copyright (c) 2011-2012 Qualcomm Atheros, Inc.
 * Copyright (c) 2016 Kapsch Trafficcom AB
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <linux/module.h>
#include <linux/mmc/card.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/sd.h>
#include "core.h"
#include "bmi.h"
#include "debug.h"
#include "hif.h"
#include "htc.h"
#include "targaddrs.h"
#include "trace.h"
#include "sdio.h"

#define CALC_TXRX_PADDED_LEN(ar_sdio, len) \
	(__ALIGN_MASK((len), (ar_sdio)->mbox_info.block_mask))

static int ath10k_sdio_read_write_sync(struct ath10k *ar, u32 addr, u8 *buf,
				       u32 len, u32 request);
static int ath10k_sdio_hif_diag_read(struct ath10k *ar, u32 address, void *buf,
				     size_t buf_len);
static int ath10k_sdio_hif_diag_read32(struct ath10k *ar, u32 address,
				       u32 *value);

/* HIF mbox interrupt handling */

static int ath10k_sdio_mbox_rx_process_packet(struct ath10k_sdio *ar_sdio,
					      struct ath10k_sdio_rx_data *pkt,
					      u32 *lookaheads,
					      int *n_lookaheads)
{
	int status = 0;
	struct ath10k_htc *htc = &ar_sdio->ar->htc;
	struct sk_buff *skb = pkt->skb;
	struct ath10k_htc_hdr *htc_hdr = (struct ath10k_htc_hdr *)skb->data;
	bool trailer_present = htc_hdr->flags & ATH10K_HTC_FLAG_TRAILER_PRESENT;
	u16 payload_len;

	payload_len = le16_to_cpu(htc_hdr->len);

	if (trailer_present) {
		u8 *trailer;
		enum ath10k_htc_ep_id eid;

		trailer = skb->data + sizeof(struct ath10k_htc_hdr) +
			  payload_len - htc_hdr->trailer_len;

		eid = (enum ath10k_htc_ep_id)htc_hdr->eid;

		status = ath10k_htc_process_trailer(htc,
						    trailer,
						    htc_hdr->trailer_len,
						    eid,
						    lookaheads,
						    n_lookaheads);
		if (status)
			goto err;

		skb_pull(skb, sizeof(*htc_hdr));
		skb_trim(skb, skb->len - htc_hdr->trailer_len);
	}

err:
	return status;
}

static inline void ath10k_sdio_mbox_free_rx_pkt(struct ath10k_sdio_rx_data *pkt)
{
	dev_kfree_skb(pkt->skb);
	pkt->skb = NULL;
	pkt->alloc_len = 0;
	pkt->act_len = 0;
}

static inline int ath10k_sdio_mbox_alloc_rx_pkt(struct ath10k_sdio_rx_data *pkt,
						size_t act_len, size_t full_len,
						bool part_of_bundle,
						bool last_in_bundle)
{
	pkt->skb = dev_alloc_skb(full_len);
	if (!pkt->skb)
		return -ENOMEM;

	pkt->act_len = act_len;
	pkt->alloc_len = full_len;
	pkt->part_of_bundle = part_of_bundle;
	pkt->last_in_bundle = last_in_bundle;

	return 0;
}

static int ath10k_sdio_mbox_rx_process_packets(struct ath10k_sdio *ar_sdio,
					       u32 lookaheads[],
					       int *n_lookahead)
{
	struct ath10k *ar = ar_sdio->ar;
	struct ath10k_htc *htc = &ar->htc;
	struct ath10k_sdio_rx_data *pkt;
	int status = 0, i;

	for (i = 0; i < ar_sdio->n_rx_pkts; i++) {
		struct ath10k_htc_ep *ep;
		enum ath10k_htc_ep_id id;
		u32 *lookaheads_local = lookaheads;
		int *n_lookahead_local = n_lookahead;

		id = ((struct ath10k_htc_hdr *)&lookaheads[i])->eid;

		if (id >= ATH10K_HTC_EP_COUNT) {
			ath10k_err(ar, "Invalid endpoint in look-ahead: %d\n",
				   id);
			status = -ENOMEM;
			goto out;
		}

		ep = &htc->endpoint[id];

		if (ep->service_id == 0) {
			ath10k_err(ar, "ep %d is not connected !\n", id);
			status = -ENOMEM;
			goto out;
		}

		pkt = &ar_sdio->rx_pkts[i];

		if (pkt->part_of_bundle && !pkt->last_in_bundle) {
			/* Only read lookahead's from RX trailers
			 * for the last packet in a bundle.
			 */
			lookaheads_local = NULL;
			n_lookahead_local = NULL;
		}

		status = ath10k_sdio_mbox_rx_process_packet(ar_sdio,
							    pkt,
							    lookaheads_local,
							    n_lookahead_local);
		if (status)
			goto out;

		ep->ep_ops.ep_rx_complete(ar_sdio->ar, pkt->skb);
		/* The RX complete handler now owns the skb...*/
		pkt->skb = NULL;
		pkt->alloc_len = 0;
	}

out:
	/* Free all packets that was not passed on to the RX completion
	 * handler...
	 */
	for (; i < ar_sdio->n_rx_pkts; i++)
		ath10k_sdio_mbox_free_rx_pkt(&ar_sdio->rx_pkts[i]);

	return status;
}

static int alloc_pkt_bundle(struct ath10k *ar,
			    struct ath10k_sdio_rx_data *rx_pkts,
			    struct ath10k_htc_hdr *htc_hdr,
			    size_t full_len, size_t act_len, size_t *bndl_cnt)
{
	int i, status = 0;

	*bndl_cnt = (htc_hdr->flags & ATH10K_HTC_FLAG_BUNDLE_MASK) >>
		    ATH10K_HTC_FLAG_BUNDLE_LSB;

	if (*bndl_cnt > HTC_HOST_MAX_MSG_PER_BUNDLE) {
		ath10k_err(ar,
			   "HTC bundle len %u exceeds maximum %u !\n",
			   le16_to_cpu(htc_hdr->len),
			   HTC_HOST_MAX_MSG_PER_BUNDLE);
		status = -ENOMEM;
		goto out;
	}

	/* Allocate bndl_cnt extra skb's for the bundle.
	 * The package containing the
	 * ATH10K_HTC_FLAG_BUNDLE_MASK flag is not included
	 * in bndl_cnt. The skb for that packet will be
	 * allocated separately.
	 */
	for (i = 0; i < *bndl_cnt; i++) {
		status = ath10k_sdio_mbox_alloc_rx_pkt(&rx_pkts[i],
						       act_len,
						       full_len,
						       true,
						       false);
		if (status)
			goto out;
	}

out:
	return status;
}

static int ath10k_sdio_mbox_rx_alloc(struct ath10k_sdio *ar_sdio,
				     u32 lookaheads[], int n_lookaheads)
{
	int status = 0, i;
	struct ath10k *ar = ar_sdio->ar;

	if (n_lookaheads > ATH10K_SDIO_MAX_RX_MSGS) {
		ath10k_err(ar,
			   "The total number of pkgs to be fetched (%u) exceeds maximum %u !\n",
			   n_lookaheads,
			   ATH10K_SDIO_MAX_RX_MSGS);
		status = -ENOMEM;
		goto err;
	}

	for (i = 0; i < n_lookaheads; i++) {
		struct ath10k_htc_hdr *htc_hdr =
			(struct ath10k_htc_hdr *)&lookaheads[i];
		size_t full_len, act_len;
		bool last_in_bundle = false;

		if (le16_to_cpu(htc_hdr->len) >
		    ATH10K_HTC_MBOX_MAX_PAYLOAD_LENGTH) {
			ath10k_err(ar,
				   "payload len %d exceeds max htc : %u!\n",
				   le16_to_cpu(htc_hdr->len),
				   ATH10K_HTC_MBOX_MAX_PAYLOAD_LENGTH);
			status = -ENOMEM;
			goto err;
		}

		act_len = le16_to_cpu(htc_hdr->len) + sizeof(*htc_hdr);
		full_len = CALC_TXRX_PADDED_LEN(ar_sdio, act_len);

		if (full_len > ATH10K_SDIO_MAX_BUFFER_SIZE) {
			ath10k_warn(ar,
				    "Rx buffer requested with invalid length htc_hdr:eid %d, flags 0x%x, len %d\n",
				    htc_hdr->eid, htc_hdr->flags,
				    le16_to_cpu(htc_hdr->len));
			status = -EINVAL;
			goto err;
		}

		if (htc_hdr->flags & ATH10K_HTC_FLAG_BUNDLE_MASK) {
			/* HTC header indicates that every packet to follow
			 * has the same padded length so that it can be
			 * optimally fetched as a full bundle.
			 */
			size_t bndl_cnt;

			status = alloc_pkt_bundle(ar, &ar_sdio->rx_pkts[i],
						  htc_hdr,
						  full_len, act_len, &bndl_cnt);

			n_lookaheads += bndl_cnt;
			i += bndl_cnt;
			/*Next buffer will be the last in the bundle */
			last_in_bundle = true;
		}

		/* Allocate skb for packet. If the packet had the
		 * ATH10K_HTC_FLAG_BUNDLE_MASK flag set, all bundled
		 * packet skb's have been allocated in the previous step.
		 */
		status = ath10k_sdio_mbox_alloc_rx_pkt(&ar_sdio->rx_pkts[i],
						       act_len,
						       full_len,
						       last_in_bundle,
						       last_in_bundle);
	}

	ar_sdio->n_rx_pkts = i;

	return 0;
err:

	for (i = 0; i < ATH10K_SDIO_MAX_RX_MSGS; i++) {
		if (!ar_sdio->rx_pkts[i].alloc_len)
			break;
		ath10k_sdio_mbox_free_rx_pkt(&ar_sdio->rx_pkts[i]);
	}

	return status;
}

static int ath10k_sdio_mbox_rx_packet(struct ath10k_sdio *ar_sdio,
				      struct ath10k_sdio_rx_data *pkt)
{
	struct ath10k *ar = ar_sdio->ar;
	struct sk_buff *skb = pkt->skb;
	int status;

	status = ath10k_sdio_read_write_sync(ar,
					     ar_sdio->mbox_info.htc_addr,
					     skb->data, pkt->alloc_len,
					     HIF_RD_SYNC_BLOCK_FIX);

	pkt->status = status;
	if (!status)
		skb_put(skb, pkt->act_len);

	return status;
}

static int ath10k_sdio_mbox_rx_fetch(struct ath10k_sdio *ar_sdio)
{
	int i, status = 0;

	for (i = 0; i < ar_sdio->n_rx_pkts; i++) {
		status = ath10k_sdio_mbox_rx_packet(ar_sdio,
						    &ar_sdio->rx_pkts[i]);
		if (status)
			goto err;
	}

	return 0;
err:
	/* Free all packets that was not successfully fetched. */
	for (; i < ar_sdio->n_rx_pkts; i++)
		ath10k_sdio_mbox_free_rx_pkt(&ar_sdio->rx_pkts[i]);

	return status;
}

/* Disable packet reception (used in case the host runs out of buffers)
 * using the interrupt enable registers through the host I/F
 */
static int ath10k_sdio_hif_rx_control(struct ath10k_sdio *ar_sdio,
				      bool enable_rx)
{
	int status = 0;
	struct ath10k_sdio_irq_enable_reg regs;
	struct ath10k_sdio_irq_data *irq_data = &ar_sdio->irq_data;

	ath10k_dbg(ar_sdio->ar, ATH10K_DBG_SDIO, "hif rx %s\n",
		   enable_rx ? "enable" : "disable");

	spin_lock_bh(&irq_data->lock);

	if (enable_rx)
		irq_data->irq_en_reg.int_status_en |=
			SM(0x01, MBOX_INT_STATUS_ENABLE_MBOX_DATA);
	else
		irq_data->irq_en_reg.int_status_en &=
			~SM(0x01, MBOX_INT_STATUS_ENABLE_MBOX_DATA);

	regs = irq_data->irq_en_reg;

	spin_unlock_bh(&irq_data->lock);

	status = ath10k_sdio_read_write_sync(ar_sdio->ar,
					     MBOX_INT_STATUS_ENABLE_ADDRESS,
					     &regs.int_status_en, sizeof(regs),
					     HIF_WR_SYNC_BYTE_INC);

	return status;
}

int ath10k_sdio_mbox_rxmsg_pending_handler(struct ath10k_sdio *ar_sdio,
					   u32 msg_lookahead, bool *done)
{
	struct ath10k *ar = ar_sdio->ar;
	int status = 0;
	u32 lookaheads[ATH10K_SDIO_MAX_RX_MSGS];
	int n_lookaheads = 1;

	*done = true;

	/* Copy the lookahead obtained from the HTC register table into our
	 * temp array as a start value.
	 */
	lookaheads[0] = msg_lookahead;

	for (;;) {
		/* Try to allocate as many HTC RX packets indicated by
		 * n_lookaheads.
		 */
		status = ath10k_sdio_mbox_rx_alloc(ar_sdio, lookaheads,
						   n_lookaheads);
		if (status)
			break;

		if (ar_sdio->n_rx_pkts >= 2)
			/* A recv bundle was detected, force IRQ status
			 * re-check again.
			 */
			*done = false;

		status = ath10k_sdio_mbox_rx_fetch(ar_sdio);

		/* Process fetched packets. This will potentially update
		 * n_lookaheads depending on if the packets contain lookahead
		 * reports.
		 */
		n_lookaheads = 0;
		status = ath10k_sdio_mbox_rx_process_packets(ar_sdio,
							     lookaheads,
							     &n_lookaheads);

		if (!n_lookaheads || status)
			break;

		/* For SYNCH processing, if we get here, we are running
		 * through the loop again due to updated lookaheads. Set
		 * flag that we should re-check IRQ status registers again
		 * before leaving IRQ processing, this can net better
		 * performance in high throughput situations.
		 */
		*done = false;
	}

	if (status && (status != -ECANCELED))
		ath10k_err(ar, "failed to get pending recv messages: %d\n",
			   status);

	if (atomic_read(&ar_sdio->stopping)) {
		ath10k_warn(ar, "host is going to stop. Turning of RX\n");
		ath10k_sdio_hif_rx_control(ar_sdio, false);
	}

	return status;
}

static int ath10k_sdio_mbox_proc_dbg_intr(struct ath10k_sdio *ar_sdio)
{
	int ret;
	u32 dummy;
	struct ath10k *ar = ar_sdio->ar;

	ath10k_warn(ar, "firmware crashed\n");

	/* read counter to clear the interrupt, the debug error interrupt is
	 * counter 0.
	 */
	ret = ath10k_sdio_read_write_sync(ar, MBOX_COUNT_DEC_ADDRESS,
					  (u8 *)&dummy, 4,
					  HIF_RD_SYNC_BYTE_INC);
	if (ret)
		ath10k_warn(ar, "Failed to clear debug interrupt: %d\n", ret);

	return ret;
}

static int ath10k_sdio_mbox_proc_counter_intr(struct ath10k_sdio *ar_sdio)
{
	u8 counter_int_status;
	struct ath10k_sdio_irq_data *irq_data = &ar_sdio->irq_data;

	counter_int_status = irq_data->irq_proc_reg.counter_int_status &
			     irq_data->irq_en_reg.cntr_int_status_en;

	/* NOTE: other modules like GMBOX may use the counter interrupt for
	 * credit flow control on other counters, we only need to check for
	 * the debug assertion counter interrupt.
	 */
	if (counter_int_status & ATH10K_SDIO_TARGET_DEBUG_INTR_MASK)
		return ath10k_sdio_mbox_proc_dbg_intr(ar_sdio);

	return 0;
}

static int ath10k_sdio_mbox_proc_err_intr(struct ath10k_sdio *ar_sdio)
{
	int status;
	u8 error_int_status;
	u8 reg_buf[4];
	struct ath10k_sdio_irq_data *irq_data = &ar_sdio->irq_data;
	struct ath10k *ar = ar_sdio->ar;

	ath10k_dbg(ar, ATH10K_DBG_SDIO, "error interrupt\n");

	error_int_status = irq_data->irq_proc_reg.error_int_status & 0x0F;
	if (!error_int_status) {
		WARN_ON(1);
		return -EIO;
	}

	ath10k_dbg(ar, ATH10K_DBG_SDIO,
		   "valid interrupt source(s) in ERROR_INT_STATUS: 0x%x\n",
		   error_int_status);

	if (MS(error_int_status, MBOX_ERROR_INT_STATUS_WAKEUP))
		ath10k_dbg(ar, ATH10K_DBG_SDIO, "error : wakeup\n");

	if (MS(error_int_status, MBOX_ERROR_INT_STATUS_RX_UNDERFLOW))
		ath10k_err(ar, "rx underflow\n");

	if (MS(error_int_status, MBOX_ERROR_INT_STATUS_TX_OVERFLOW))
		ath10k_err(ar, "tx overflow\n");

	/* Clear the interrupt */
	irq_data->irq_proc_reg.error_int_status &= ~error_int_status;

	/* set W1C value to clear the interrupt, this hits the register first */
	reg_buf[0] = error_int_status;
	reg_buf[1] = 0;
	reg_buf[2] = 0;
	reg_buf[3] = 0;

	status = ath10k_sdio_read_write_sync(ar,
					     MBOX_ERROR_INT_STATUS_ADDRESS,
					     reg_buf, 4, HIF_WR_SYNC_BYTE_FIX);

	WARN_ON(status);

	return status;
}

static int ath10k_sdio_mbox_proc_cpu_intr(struct ath10k_sdio *ar_sdio)
{
	int status;
	struct ath10k_sdio_irq_data *irq_data = &ar_sdio->irq_data;
	struct ath10k *ar = ar_sdio->ar;
	u8 cpu_int_status, reg_buf[4];

	cpu_int_status = irq_data->irq_proc_reg.cpu_int_status &
			 irq_data->irq_en_reg.cpu_int_status_en;
	if (!cpu_int_status) {
		WARN_ON(1);
		return -EIO;
	}

	/* Clear the interrupt */
	irq_data->irq_proc_reg.cpu_int_status &= ~cpu_int_status;

	/* Set up the register transfer buffer to hit the register 4 times ,
	 * this is done to make the access 4-byte aligned to mitigate issues
	 * with host bus interconnects that restrict bus transfer lengths to
	 * be a multiple of 4-bytes.
	 */

	/* set W1C value to clear the interrupt, this hits the register first */
	reg_buf[0] = cpu_int_status;
	/* the remaining are set to zero which have no-effect  */
	reg_buf[1] = 0;
	reg_buf[2] = 0;
	reg_buf[3] = 0;

	status = ath10k_sdio_read_write_sync(ar,
					     MBOX_CPU_INT_STATUS_ADDRESS,
					     reg_buf, 4, HIF_WR_SYNC_BYTE_FIX);

	WARN_ON(status);

	return status;
}

/* process pending interrupts synchronously */
static int ath10k_sdio_mbox_proc_pending_irqs(struct ath10k_sdio *ar_sdio,
					      bool *done)
{
	struct ath10k_sdio_irq_data *irq_data = &ar_sdio->irq_data;
	struct ath10k *ar = ar_sdio->ar;
	struct ath10k_sdio_irq_proc_registers *rg;
	int status = 0;
	u8 host_int_status = 0;
	u32 lookahead = 0;
	u8 htc_mbox = 1 << ATH10K_HTC_MAILBOX;

	/* NOTE: HIF implementation guarantees that the context of this
	 * call allows us to perform SYNCHRONOUS I/O, that is we can block,
	 * sleep or call any API that can block or switch thread/task
	 * contexts. This is a fully schedulable context.
	 */

	/* Process pending intr only when int_status_en is clear, it may
	 * result in unnecessary bus transaction otherwise. Target may be
	 * unresponsive at the time.
	 */
	if (irq_data->irq_en_reg.int_status_en) {
		/* Read the first sizeof(struct ath10k_irq_proc_registers)
		 * bytes of the HTC register table. This
		 * will yield us the value of different int status
		 * registers and the lookahead registers.
		 */
		status = ath10k_sdio_read_write_sync(
				ar,
				MBOX_HOST_INT_STATUS_ADDRESS,
				(u8 *)&irq_data->irq_proc_reg,
				sizeof(irq_data->irq_proc_reg),
				HIF_RD_SYNC_BYTE_INC);
		if (status)
			goto out;

		/* Update only those registers that are enabled */
		host_int_status = irq_data->irq_proc_reg.host_int_status &
				  irq_data->irq_en_reg.int_status_en;

		/* Look at mbox status */
		if (host_int_status & htc_mbox) {
			/* Mask out pending mbox value, we use look ahead as
			 * the real flag for mbox processing.
			 */
			host_int_status &= ~htc_mbox;
			if (irq_data->irq_proc_reg.rx_lookahead_valid &
			    htc_mbox) {
				rg = &irq_data->irq_proc_reg;
				lookahead = le32_to_cpu(
					rg->rx_lookahead[ATH10K_HTC_MAILBOX]);
				if (!lookahead)
					ath10k_err(ar, "lookahead is zero!\n");
			}
		}
	}

	if (!host_int_status && !lookahead) {
		*done = true;
		goto out;
	}

	if (lookahead) {
		ath10k_dbg(ar, ATH10K_DBG_SDIO,
			   "pending mailbox msg, lookahead: 0x%08X\n",
			   lookahead);

		status = ath10k_sdio_mbox_rxmsg_pending_handler(ar_sdio,
								lookahead,
								done);
		if (status)
			goto out;
	}

	/* now, handle the rest of the interrupts */
	ath10k_dbg(ar, ATH10K_DBG_SDIO,
		   "valid interrupt source(s) for other interrupts: 0x%x\n",
		   host_int_status);

	if (MS(host_int_status, MBOX_HOST_INT_STATUS_CPU)) {
		/* CPU Interrupt */
		status = ath10k_sdio_mbox_proc_cpu_intr(ar_sdio);
		if (status)
			goto out;
	}

	if (MS(host_int_status, MBOX_HOST_INT_STATUS_ERROR)) {
		/* Error Interrupt */
		status = ath10k_sdio_mbox_proc_err_intr(ar_sdio);
		if (status)
			goto out;
	}

	if (MS(host_int_status, MBOX_HOST_INT_STATUS_COUNTER))
		/* Counter Interrupt */
		status = ath10k_sdio_mbox_proc_counter_intr(ar_sdio);

out:
	/* An optimization to bypass reading the IRQ status registers
	 * unecessarily which can re-wake the target, if upper layers
	 * determine that we are in a low-throughput mode, we can rely on
	 * taking another interrupt rather than re-checking the status
	 * registers which can re-wake the target.
	 *
	 * NOTE : for host interfaces that makes use of detecting pending
	 * mbox messages at hif can not use this optimization due to
	 * possible side effects, SPI requires the host to drain all
	 * messages from the mailbox before exiting the ISR routine.
	 */

	ath10k_dbg(ar, ATH10K_DBG_SDIO,
		   "%s: (done:%d, status=%d)\n", __func__, *done, status);

	return status;
}

/* Macro to check if DMA buffer is WORD-aligned and DMA-able.
 * Most host controllers assume the buffer is DMA'able and will
 * bug-check otherwise (i.e. buffers on the stack). virt_addr_valid
 * check fails on stack memory.
 */
static inline bool buf_needs_bounce(u8 *buf)
{
	return ((unsigned long)buf & 0x3) || !virt_addr_valid(buf);
}

static inline enum ath10k_htc_ep_id pipe_id_to_eid(u8 pipe_id)
{
	return (enum ath10k_htc_ep_id)pipe_id;
}

static void ath10k_sdio_set_mbox_info(struct ath10k_sdio *ar_sdio)
{
	struct ath10k_mbox_info *mbox_info = &ar_sdio->mbox_info;
	u16 device = ar_sdio->func->device;

	mbox_info->htc_addr = ATH10K_HIF_MBOX_BASE_ADDR;
	mbox_info->block_size = ATH10K_HIF_MBOX_BLOCK_SIZE;
	mbox_info->block_mask = ATH10K_HIF_MBOX_BLOCK_SIZE - 1;
	mbox_info->gmbox_addr = ATH10K_HIF_GMBOX_BASE_ADDR;
	mbox_info->gmbox_sz = ATH10K_HIF_GMBOX_WIDTH;

	mbox_info->ext_info[0].htc_ext_addr = ATH10K_HIF_MBOX0_EXT_BASE_ADDR;

	if ((device & ATH10K_MANUFACTURER_ID_REV_MASK) < 4)
		mbox_info->ext_info[0].htc_ext_sz = ATH10K_HIF_MBOX0_EXT_WIDTH;
	else
		/* from rome 2.0(0x504), the width has been extended
		 * to 56K
		 */
		mbox_info->ext_info[0].htc_ext_sz =
			ATH10K_HIF_MBOX0_EXT_WIDTH_ROME_2_0;

	mbox_info->ext_info[1].htc_ext_addr =
		mbox_info->ext_info[0].htc_ext_addr +
		mbox_info->ext_info[0].htc_ext_sz +
		ATH10K_HIF_MBOX_DUMMY_SPACE_SIZE;
	mbox_info->ext_info[1].htc_ext_sz = ATH10K_HIF_MBOX1_EXT_WIDTH;
}

static inline void ath10k_sdio_set_cmd52_arg(u32 *arg, u8 write, u8 raw,
					     unsigned int address,
					     unsigned char val)
{
	const u8 func = 0;

	*arg = ((write & 1) << 31) |
	       ((func & 0x7) << 28) |
	       ((raw & 1) << 27) |
	       (1 << 26) |
	       ((address & 0x1FFFF) << 9) |
	       (1 << 8) |
	       (val & 0xFF);
}

static int ath10k_sdio_func0_cmd52_wr_byte(struct mmc_card *card,
					   unsigned int address,
					   unsigned char byte)
{
	struct mmc_command io_cmd;

	memset(&io_cmd, 0, sizeof(io_cmd));
	ath10k_sdio_set_cmd52_arg(&io_cmd.arg, 1, 0, address, byte);
	io_cmd.opcode = SD_IO_RW_DIRECT;
	io_cmd.flags = MMC_RSP_R5 | MMC_CMD_AC;

	return mmc_wait_for_cmd(card->host, &io_cmd, 0);
}

static int ath10k_sdio_func0_cmd52_rd_byte(struct mmc_card *card,
					   unsigned int address,
					   unsigned char *byte)
{
	int ret;
	struct mmc_command io_cmd;

	memset(&io_cmd, 0, sizeof(io_cmd));
	ath10k_sdio_set_cmd52_arg(&io_cmd.arg, 0, 0, address, 0);
	io_cmd.opcode = SD_IO_RW_DIRECT;
	io_cmd.flags = MMC_RSP_R5 | MMC_CMD_AC;

	ret = mmc_wait_for_cmd(card->host, &io_cmd, 0);
	if (!ret)
		*byte = io_cmd.resp[0];

	return ret;
}

static int ath10k_sdio_io(struct ath10k_sdio *ar_sdio, u32 request, u32 addr,
			  u8 *buf, u32 len)
{
	int ret = 0;
	struct sdio_func *func = ar_sdio->func;
	struct ath10k *ar = ar_sdio->ar;

	sdio_claim_host(func);

	if (request & HIF_WRITE) {
		if (request & HIF_FIXED_ADDRESS)
			ret = sdio_writesb(func, addr, buf, len);
		else
			ret = sdio_memcpy_toio(func, addr, buf, len);
	} else {
		if (request & HIF_FIXED_ADDRESS)
			ret = sdio_readsb(func, buf, addr, len);
		else
			ret = sdio_memcpy_fromio(func, buf, addr, len);
	}

	sdio_release_host(func);

	ath10k_dbg(ar, ATH10K_DBG_SDIO, "%s addr 0x%x%s buf 0x%p len %d\n",
		   request & HIF_WRITE ? "wr" : "rd", addr,
		   request & HIF_FIXED_ADDRESS ? " (fixed)" : "", buf, len);
	ath10k_dbg_dump(ar, ATH10K_DBG_SDIO_DUMP, NULL,
			request & HIF_WRITE ? "sdio wr " : "sdio rd ",
			buf, len);

	return ret;
}

static struct ath10k_sdio_bus_request
*ath10k_sdio_alloc_busreq(struct ath10k_sdio *ar_sdio)
{
	struct ath10k_sdio_bus_request *bus_req;

	spin_lock_bh(&ar_sdio->lock);

	if (list_empty(&ar_sdio->bus_req_freeq)) {
		spin_unlock_bh(&ar_sdio->lock);
		return NULL;
	}

	bus_req = list_first_entry(&ar_sdio->bus_req_freeq,
				   struct ath10k_sdio_bus_request, list);
	list_del(&bus_req->list);

	spin_unlock_bh(&ar_sdio->lock);

	return bus_req;
}

static void ath10k_sdio_free_bus_req(struct ath10k_sdio *ar_sdio,
				     struct ath10k_sdio_bus_request *bus_req)
{
	spin_lock_bh(&ar_sdio->lock);
	list_add_tail(&bus_req->list, &ar_sdio->bus_req_freeq);
	spin_unlock_bh(&ar_sdio->lock);
}

static int ath10k_sdio_read_write_sync(struct ath10k *ar, u32 addr, u8 *buf,
				       u32 len, u32 request)
{
	struct ath10k_sdio *ar_sdio = ath10k_sdio_priv(ar);
	u8  *tbuf = NULL;
	int ret;
	bool bounced = false;

	if (request & HIF_BLOCK_BASIS)
		len = round_down(len, ar_sdio->mbox_info.block_size);

	if (buf_needs_bounce(buf)) {
		if (!ar_sdio->dma_buffer)
			return -ENOMEM;
		/* FIXME: I am not sure if it is always correct to assume
		 * that the SDIO irq is a "fake" irq and sleep is possible.
		 * (this function will get called from
		 * ath10k_sdio_irq_handler
		 */
		mutex_lock(&ar_sdio->dma_buffer_mutex);
		tbuf = ar_sdio->dma_buffer;

		if (request & HIF_WRITE)
			memcpy(tbuf, buf, len);

		bounced = true;
	} else {
		tbuf = buf;
	}

	ret = ath10k_sdio_io(ar_sdio, request, addr, tbuf, len);
	if ((request & HIF_READ) && bounced)
		memcpy(buf, tbuf, len);

	if (bounced)
		mutex_unlock(&ar_sdio->dma_buffer_mutex);

	return ret;
}

static void __ath10k_sdio_write_async(struct ath10k_sdio *ar_sdio,
				      struct ath10k_sdio_bus_request *req)
{
	int status;
	struct ath10k_htc_ep *ep;
	struct sk_buff *skb;

	skb = req->skb;
	status = ath10k_sdio_read_write_sync(ar_sdio->ar, req->address,
					     skb->data, req->len,
					     req->request);
	ep = &ar_sdio->ar->htc.endpoint[req->eid];
	ath10k_htc_notify_tx_completion(ep, skb);
	ath10k_sdio_free_bus_req(ar_sdio, req);
}

static void ath10k_sdio_write_async_work(struct work_struct *work)
{
	struct ath10k_sdio *ar_sdio;
	struct ath10k_sdio_bus_request *req, *tmp_req;

	ar_sdio = container_of(work, struct ath10k_sdio, wr_async_work);

	spin_lock_bh(&ar_sdio->wr_async_lock);
	list_for_each_entry_safe(req, tmp_req, &ar_sdio->wr_asyncq, list) {
		list_del(&req->list);
		spin_unlock_bh(&ar_sdio->wr_async_lock);
		__ath10k_sdio_write_async(ar_sdio, req);
		spin_lock_bh(&ar_sdio->wr_async_lock);
	}
	spin_unlock_bh(&ar_sdio->wr_async_lock);
}

static void ath10k_sdio_irq_handler(struct sdio_func *func)
{
	int status = 0;
	unsigned long timeout;
	struct ath10k_sdio *ar_sdio;
	bool done = false;

	ar_sdio = sdio_get_drvdata(func);
	atomic_set(&ar_sdio->irq_handling, 1);

	/* Release the host during interrupts so we can pick it back up when
	 * we process commands.
	 */
	sdio_release_host(ar_sdio->func);

	timeout = jiffies + ATH10K_SDIO_HIF_COMMUNICATION_TIMEOUT_HZ;
	while (time_before(jiffies, timeout) && !done) {
		status = ath10k_sdio_mbox_proc_pending_irqs(ar_sdio, &done);
		if (status)
			break;
	}

	sdio_claim_host(ar_sdio->func);

	atomic_set(&ar_sdio->irq_handling, 0);
	wake_up(&ar_sdio->irq_wq);

	WARN_ON(status && status != -ECANCELED);
}

static int ath10k_sdio_hif_disable_intrs(struct ath10k_sdio *ar_sdio)
{
	int ret;
	struct ath10k_sdio_irq_enable_reg regs;
	struct ath10k_sdio_irq_data *irq_data = &ar_sdio->irq_data;

	memset(&regs, 0, sizeof(regs));

	ret = ath10k_sdio_read_write_sync(ar_sdio->ar,
					  MBOX_INT_STATUS_ENABLE_ADDRESS,
					  &regs.int_status_en, sizeof(regs),
					  HIF_WR_SYNC_BYTE_INC);
	if (ret) {
		ath10k_err(ar_sdio->ar, "Unable to disable sdio interrupts\n");
		return ret;
	}

	spin_lock_bh(&irq_data->lock);
	irq_data->irq_en_reg = regs;
	spin_unlock_bh(&irq_data->lock);

	return 0;
}

static int ath10k_sdio_hif_power_up(struct ath10k *ar)
{
	struct ath10k_sdio *ar_sdio = ath10k_sdio_priv(ar);
	struct sdio_func *func = ar_sdio->func;
	int ret = 0;

	if (!ar_sdio->is_disabled)
		return 0;

	ath10k_dbg(ar, ATH10K_DBG_BOOT, "sdio power on\n");

	sdio_claim_host(func);

	ret = sdio_enable_func(func);
	if (ret) {
		ath10k_err(ar, "Unable to enable sdio func: %d)\n", ret);
		sdio_release_host(func);
		return ret;
	}

	sdio_release_host(func);

	/* Wait for hardware to initialise. It should take a lot less than
	 * 20 ms but let's be conservative here.
	 */
	msleep(20);

	ar_sdio->is_disabled = false;

	ret = ath10k_sdio_hif_disable_intrs(ar_sdio);

	return ret;
}

static void ath10k_sdio_hif_power_down(struct ath10k *ar)
{
	struct ath10k_sdio *ar_sdio = ath10k_sdio_priv(ar);
	int ret;

	if (ar_sdio->is_disabled)
		return;

	ath10k_dbg(ar, ATH10K_DBG_BOOT, "sdio power off\n");

	/* Disable the card */
	sdio_claim_host(ar_sdio->func);
	ret = sdio_disable_func(ar_sdio->func);
	sdio_release_host(ar_sdio->func);

	if (ret)
		ath10k_dbg(ar, ATH10K_DBG_BOOT,
			   "Unable to disable sdio: %d\n", ret);

	ar_sdio->is_disabled = true;
}

int ath10k_sdio_hif_tx_sg(struct ath10k *ar, u8 pipe_id,
			  struct ath10k_hif_sg_item *items, int n_items)
{
	int i;
	u32 address;
	struct ath10k_sdio *ar_sdio = ath10k_sdio_priv(ar);
	struct ath10k_sdio_bus_request *bus_req;

	bus_req = ath10k_sdio_alloc_busreq(ar_sdio);

	if (WARN_ON_ONCE(!bus_req))
		return -ENOMEM;

	for (i = 0; i < n_items; i++) {
		bus_req->skb = items[i].transfer_context;
		bus_req->request = HIF_WRITE;
		bus_req->eid = pipe_id_to_eid(pipe_id);
		/* Write TX data to the end of the mbox address space */
		address = ar_sdio->mbox_addr[bus_req->eid] +
			  ar_sdio->mbox_size[bus_req->eid] - bus_req->skb->len;
		bus_req->address = address;
		bus_req->len = CALC_TXRX_PADDED_LEN(ar_sdio, bus_req->skb->len);

		spin_lock_bh(&ar_sdio->wr_async_lock);
		list_add_tail(&bus_req->list, &ar_sdio->wr_asyncq);
		spin_unlock_bh(&ar_sdio->wr_async_lock);
	}

	queue_work(ar_sdio->workqueue, &ar_sdio->wr_async_work);

	return 0;
}

static int ath10k_sdio_hif_enable_intrs(struct ath10k_sdio *ar_sdio)
{
	struct ath10k_sdio_irq_enable_reg regs;
	int status;
	struct ath10k_sdio_irq_data *irq_data = &ar_sdio->irq_data;

	memset(&regs, 0, sizeof(regs));

	/* Enable all but CPU interrupts */
	regs.int_status_en = SM(0x01, MBOX_INT_STATUS_ENABLE_ERROR) |
			     SM(0x01, MBOX_INT_STATUS_ENABLE_CPU) |
			     SM(0x01, MBOX_INT_STATUS_ENABLE_COUNTER);

	/* NOTE: There are some cases where HIF can do detection of
	 * pending mbox messages which is disabled now.
	 */
	regs.int_status_en |= SM(0x01, MBOX_INT_STATUS_ENABLE_MBOX_DATA);

	/* Set up the CPU Interrupt status Register */
	regs.cpu_int_status_en = 0;

	/* Set up the Error Interrupt status Register */
	regs.err_int_status_en =
		SM(0x01, MBOX_ERROR_STATUS_ENABLE_RX_UNDERFLOW) |
		SM(0x01, MBOX_ERROR_STATUS_ENABLE_TX_OVERFLOW);

	/* Enable Counter interrupt status register to get fatal errors for
	 * debugging.
	 */
	regs.cntr_int_status_en = SM(ATH10K_SDIO_TARGET_DEBUG_INTR_MASK,
				     MBOX_COUNTER_INT_STATUS_ENABLE_BIT);

	status = ath10k_sdio_read_write_sync(ar_sdio->ar,
					     MBOX_INT_STATUS_ENABLE_ADDRESS,
					     &regs.int_status_en, sizeof(regs),
					     HIF_WR_SYNC_BYTE_INC);
	if (status) {
		ath10k_err(ar_sdio->ar,
			   "failed to update interrupt ctl reg err: %d\n",
			   status);
		return status;
	}

	spin_lock_bh(&irq_data->lock);
	irq_data->irq_en_reg = regs;
	spin_unlock_bh(&irq_data->lock);

	return 0;
}

static int ath10k_sdio_hif_start(struct ath10k *ar)
{
	int ret;
	struct ath10k_sdio *ar_sdio = ath10k_sdio_priv(ar);
	u32 addr, val;

	addr = host_interest_item_address(HI_ITEM(hi_acs_flags));

	ret = ath10k_sdio_hif_diag_read32(ar, addr, &val);
	if (ret) {
		ath10k_err(ar, "Unable to read diag mem: %d\n", ret);
		goto out;
	}

	if (val & HI_ACS_FLAGS_SDIO_SWAP_MAILBOX_FW_ACK) {
		ath10k_dbg(ar, ATH10K_DBG_SDIO,
			   "Mailbox SWAP Service is enabled\n");
		ar_sdio->swap_mbox = true;
	}

	/* eid 0 always uses the lower part of the extended mailbox address
	 * space (ext_info[0].htc_ext_addr).
	 */
	ar_sdio->mbox_addr[0] = ar_sdio->mbox_info.ext_info[0].htc_ext_addr;
	ar_sdio->mbox_size[0] = ar_sdio->mbox_info.ext_info[0].htc_ext_sz;

	sdio_claim_host(ar_sdio->func);

	/* Register the isr */
	ret =  sdio_claim_irq(ar_sdio->func, ath10k_sdio_irq_handler);
	if (ret) {
		ath10k_err(ar, "Failed to claim sdio irq: %d\n", ret);
		sdio_release_host(ar_sdio->func);
		goto out;
	}

	sdio_release_host(ar_sdio->func);

	ret = ath10k_sdio_hif_enable_intrs(ar_sdio);
	if (ret)
		ath10k_err(ar, "Failed to enable sdio interrupts: %d\n", ret);

out:
	return ret;
}

static bool ath10k_sdio_is_on_irq(struct ath10k *ar)
{
	struct ath10k_sdio *ar_sdio = ath10k_sdio_priv(ar);

	return !atomic_read(&ar_sdio->irq_handling);
}

static void ath10k_sdio_irq_disable(struct ath10k *ar)
{
	struct ath10k_sdio *ar_sdio = ath10k_sdio_priv(ar);
	int ret;

	sdio_claim_host(ar_sdio->func);

	atomic_set(&ar_sdio->stopping, 1);

	if (atomic_read(&ar_sdio->irq_handling)) {
		sdio_release_host(ar_sdio->func);

		ret = wait_event_interruptible(ar_sdio->irq_wq,
					       ath10k_sdio_is_on_irq(ar));
		if (ret)
			return;

		sdio_claim_host(ar_sdio->func);
	}

	ret = sdio_release_irq(ar_sdio->func);
	if (ret)
		ath10k_err(ar, "Failed to release sdio irq: %d\n", ret);

	sdio_release_host(ar_sdio->func);
}

static int ath10k_sdio_config(struct ath10k_sdio *ar_sdio)
{
	struct ath10k *ar = ar_sdio->ar;
	struct sdio_func *func = ar_sdio->func;
	unsigned char byte, asyncintdelay = 2;
	int ret;

	ath10k_dbg(ar, ATH10K_DBG_BOOT, "SDIO configuration\n");

	sdio_claim_host(func);

	byte = 0;
	ret = ath10k_sdio_func0_cmd52_rd_byte(func->card,
					      SDIO_CCCR_DRIVE_STRENGTH,
					      &byte);

	byte = (byte & (~(SDIO_DRIVE_DTSx_MASK << SDIO_DRIVE_DTSx_SHIFT))) |
		SDIO_DTSx_SET_TYPE_D;

	ret = ath10k_sdio_func0_cmd52_wr_byte(func->card,
					      SDIO_CCCR_DRIVE_STRENGTH,
					      byte);

	byte = 0;
	ret = ath10k_sdio_func0_cmd52_rd_byte(
		func->card,
		CCCR_SDIO_DRIVER_STRENGTH_ENABLE_ADDR,
		&byte);

	byte |= (CCCR_SDIO_DRIVER_STRENGTH_ENABLE_A |
		 CCCR_SDIO_DRIVER_STRENGTH_ENABLE_C |
		 CCCR_SDIO_DRIVER_STRENGTH_ENABLE_D);

	ret = ath10k_sdio_func0_cmd52_wr_byte(
		func->card,
		CCCR_SDIO_DRIVER_STRENGTH_ENABLE_ADDR,
		byte);
	if (ret) {
		ath10k_err(ar, "Failed to enable driver strengt\n");
		goto out;
	}

	byte = 0;
	ret = ath10k_sdio_func0_cmd52_rd_byte(func->card,
					      CCCR_SDIO_IRQ_MODE_REG_SDIO3,
					      &byte);

	byte |= SDIO_IRQ_MODE_ASYNC_4BIT_IRQ_SDIO3;

	ret = ath10k_sdio_func0_cmd52_wr_byte(func->card,
					      CCCR_SDIO_IRQ_MODE_REG_SDIO3,
					      byte);
	if (ret) {
		ath10k_err(ar, "Failed to enable 4-bit async irq mode %d\n",
			   ret);
		goto out;
	}

	byte = 0;
	ret = ath10k_sdio_func0_cmd52_rd_byte(func->card,
					      CCCR_SDIO_ASYNC_INT_DELAY_ADDRESS,
					      &byte);

	byte = (byte & ~CCCR_SDIO_ASYNC_INT_DELAY_MASK) |
		((asyncintdelay << CCCR_SDIO_ASYNC_INT_DELAY_LSB) &
		CCCR_SDIO_ASYNC_INT_DELAY_MASK);

	ret = ath10k_sdio_func0_cmd52_wr_byte(func->card,
					      CCCR_SDIO_ASYNC_INT_DELAY_ADDRESS,
					      byte);

	/* give us some time to enable, in ms */
	func->enable_timeout = 100;

	ret = sdio_set_block_size(func, ar_sdio->mbox_info.block_size);
	if (ret) {
		ath10k_err(ar, "Set sdio block size %d failed: %d)\n",
			   ar_sdio->mbox_info.block_size, ret);
		goto out;
	}

out:
	sdio_release_host(func);

	return ret;
}

/* set the window address register (using 4-byte register access ). */
static int ath10k_set_addrwin_reg(struct ath10k *ar, u32 reg_addr, u32 addr)
{
	int status;

	status = ath10k_sdio_read_write_sync(ar, reg_addr, (u8 *)(&addr),
					     4, HIF_WR_SYNC_BYTE_INC);

	if (status) {
		ath10k_err(ar, "%s: failed to write 0x%x to window reg: 0x%X\n",
			   __func__, addr, reg_addr);
		return status;
	}

	return 0;
}

static int ath10k_sdio_hif_diag_read32(struct ath10k *ar, u32 address,
				       u32 *value)
{
	__le32 val = 0;
	int ret;

	ret = ath10k_sdio_hif_diag_read(ar, address, &val, sizeof(val));
	*value = __le32_to_cpu(val);

	return ret;
}

static int ath10k_sdio_hif_diag_read(struct ath10k *ar, u32 address, void *buf,
				     size_t buf_len)
{
	int status;

	/* set window register to start read cycle */
	status = ath10k_set_addrwin_reg(ar, MBOX_WINDOW_READ_ADDR_ADDRESS,
					address);

	if (status)
		return status;

	/* read the data */
	status = ath10k_sdio_read_write_sync(ar, MBOX_WINDOW_DATA_ADDRESS,
					     (u8 *)buf, buf_len,
					     HIF_RD_SYNC_BYTE_INC);
	if (status) {
		ath10k_err(ar, "%s: failed to read from window data addr\n",
			   __func__);
		return status;
	}

	return status;
}

static int ath10k_sdio_diag_write_mem(struct ath10k *ar, u32 address,
				      const void *data, int nbytes)
{
	int status;

	/* set write data */
	status = ath10k_sdio_read_write_sync(ar, MBOX_WINDOW_DATA_ADDRESS,
					     (u8 *)data, nbytes,
					     HIF_WR_SYNC_BYTE_INC);
	if (status) {
		ath10k_err(ar, "%s: failed to write 0x%p to window data addr\n",
			   __func__, data);
		return status;
	}

	/* set window register, which starts the write cycle */
	return ath10k_set_addrwin_reg(ar, MBOX_WINDOW_WRITE_ADDR_ADDRESS,
				      address);
}

static int ath10k_sdio_bmi_credits(struct ath10k *ar)
{
	u32 addr, cmd_credits;
	unsigned long timeout;
	int ret;

	cmd_credits = 0;

	/* Read the counter register to get the command credits */
	addr = MBOX_COUNT_DEC_ADDRESS + ATH10K_HIF_MBOX_NUM_MAX * 4;

	timeout = jiffies + BMI_COMMUNICATION_TIMEOUT_HZ;
	while (time_before(jiffies, timeout) && !cmd_credits) {
		/* Hit the credit counter with a 4-byte access, the first byte
		 * read will hit the counter and cause a decrement, while the
		 * remaining 3 bytes has no effect. The rationale behind this
		 * is to make all HIF accesses 4-byte aligned.
		 */
		ret = ath10k_sdio_read_write_sync(ar, addr,
						  (u8 *)&cmd_credits,
						  sizeof(cmd_credits),
						  HIF_RD_SYNC_BYTE_INC);
		if (ret) {
			ath10k_err(ar,
				   "Unable to decrement the command credit count register: %d\n",
				   ret);
			return ret;
		}

		/* The counter is only 8 bits.
		 * Ignore anything in the upper 3 bytes
		 */
		cmd_credits &= 0xFF;
	}

	if (!cmd_credits) {
		ath10k_err(ar, "bmi communication timeout\n");
		return -ETIMEDOUT;
	}

	return 0;
}

static int ath10k_sdio_bmi_get_rx_lookahead(struct ath10k *ar)
{
	unsigned long timeout;
	u32 rx_word = 0;
	int ret = 0;

	timeout = jiffies + BMI_COMMUNICATION_TIMEOUT_HZ;
	while ((time_before(jiffies, timeout)) && !rx_word) {
		ret = ath10k_sdio_read_write_sync(ar,
						  MBOX_HOST_INT_STATUS_ADDRESS,
						  (u8 *)&rx_word,
						  sizeof(rx_word),
						  HIF_RD_SYNC_BYTE_INC);
		if (ret) {
			ath10k_err(ar, "unable to read RX_LOOKAHEAD_VALID\n");
			return ret;
		}

		 /* all we really want is one bit */
		rx_word &= 1;
	}

	if (!rx_word) {
		ath10k_err(ar, "bmi_recv_buf FIFO empty\n");
		return -EINVAL;
	}

	return ret;
}

int ath10k_sdio_hif_exchange_bmi_msg(struct ath10k *ar,
				     void *req, u32 req_len,
				     void *resp, u32 *resp_len)
{
	int ret = 0;
	u32 addr;
	struct ath10k_sdio *ar_sdio = ath10k_sdio_priv(ar);

	ret = ath10k_sdio_bmi_credits(ar);
	if (ret)
		return ret;

	/* BMI data is written to the end of the mailbox address space */
	addr = ar_sdio->mbox_info.htc_addr + ATH10K_HIF_MBOX_WIDTH - req_len;

	ret = ath10k_sdio_read_write_sync(ar, addr, req, req_len,
					  HIF_WR_SYNC_BYTE_INC);
	if (ret) {
		ath10k_err(ar, "unable to send the bmi data to the device\n");
		return ret;
	}

	if (!resp || !resp_len)
		/* No response expected */
		goto out;

	/* During normal bootup, small reads may be required.
	 * Rather than issue an HIF Read and then wait as the Target
	 * adds successive bytes to the FIFO, we wait here until
	 * we know that response data is available.
	 *
	 * This allows us to cleanly timeout on an unexpected
	 * Target failure rather than risk problems at the HIF level.
	 * In particular, this avoids SDIO timeouts and possibly garbage
	 * data on some host controllers.  And on an interconnect
	 * such as Compact Flash (as well as some SDIO masters) which
	 * does not provide any indication on data timeout, it avoids
	 * a potential hang or garbage response.
	 *
	 * Synchronization is more difficult for reads larger than the
	 * size of the MBOX FIFO (128B), because the Target is unable
	 * to push the 129th byte of data until AFTER the Host posts an
	 * HIF Read and removes some FIFO data.  So for large reads the
	 * Host proceeds to post an HIF Read BEFORE all the data is
	 * actually available to read.  Fortunately, large BMI reads do
	 * not occur in practice -- they're supported for debug/development.
	 *
	 * So Host/Target BMI synchronization is divided into these cases:
	 *  CASE 1: length < 4
	 *        Should not happen
	 *
	 *  CASE 2: 4 <= length <= 128
	 *        Wait for first 4 bytes to be in FIFO
	 *        If CONSERVATIVE_BMI_READ is enabled, also wait for
	 *        a BMI command credit, which indicates that the ENTIRE
	 *        response is available in the the FIFO
	 *
	 *  CASE 3: length > 128
	 *        Wait for the first 4 bytes to be in FIFO
	 *
	 * For most uses, a small timeout should be sufficient and we will
	 * usually see a response quickly; but there may be some unusual
	 * (debug) cases of BMI_EXECUTE where we want an larger timeout.
	 * For now, we use an unbounded busy loop while waiting for
	 * BMI_EXECUTE.
	 *
	 * If BMI_EXECUTE ever needs to support longer-latency execution,
	 * especially in production, this code needs to be enhanced to sleep
	 * and yield.  Also note that BMI_COMMUNICATION_TIMEOUT is currently
	 * a function of Host processor speed.
	 */
	ret = ath10k_sdio_bmi_get_rx_lookahead(ar);
	if (ret)
		goto out;

	/* We always read from the start of the mbox address */
	addr = ar_sdio->mbox_info.htc_addr;
	ret = ath10k_sdio_read_write_sync(ar, addr, resp, *resp_len,
					  HIF_RD_SYNC_BYTE_INC);
	if (ret)
		ath10k_err(ar, "Unable to read the bmi data from the device: %d\n",
			   ret);

out:
	return ret;
}

static void ath10k_sdio_hif_stop(struct ath10k *ar)
{
	struct ath10k_sdio *ar_sdio = ath10k_sdio_priv(ar);
	struct ath10k_sdio_bus_request *req, *tmp_req;

	ath10k_sdio_irq_disable(ar);

	cancel_work_sync(&ar_sdio->wr_async_work);

	spin_lock_bh(&ar_sdio->wr_async_lock);

	list_for_each_entry_safe(req, tmp_req, &ar_sdio->wr_asyncq, list) {
		struct ath10k_htc_ep *ep;

		list_del(&req->list);

		ep = &ar->htc.endpoint[req->eid];
		ath10k_htc_notify_tx_completion(ep, req->skb);
		ath10k_sdio_free_bus_req(ar_sdio, req);
	}

	spin_unlock_bh(&ar_sdio->wr_async_lock);
}

#ifdef CONFIG_PM

static int ath10k_sdio_hif_suspend(struct ath10k *ar)
{
	return -EOPNOTSUPP;
}

static int ath10k_sdio_hif_resume(struct ath10k *ar)
{
	struct ath10k_sdio *ar_sdio = ath10k_sdio_priv(ar);

	switch (ar->state) {
	case ATH10K_STATE_OFF:
		ath10k_dbg(ar, ATH10K_DBG_SDIO,
			   "sdio resume configuring sdio\n");

		/* need to set sdio settings after power is cut from sdio */
		ath10k_sdio_config(ar_sdio);
		break;

	case ATH10K_STATE_ON:
	default:
		break;
	}

	return 0;
}
#endif

int ath10k_sdio_hif_map_service_to_pipe(struct ath10k *ar, u16 service_id,
					u8 *ul_pipe, u8 *dl_pipe)
{
	int ret = 0, i;
	bool ep_found = false;
	enum ath10k_htc_ep_id eid;
	struct ath10k_htc *htc = &ar->htc;
	struct ath10k_sdio *ar_sdio = ath10k_sdio_priv(ar);
	u32 htt_addr, wmi_addr, htt_mbox_size, wmi_mbox_size;

	/* For sdio, we are interested in the mapping between eid
	 * and pipeid rather than service_id to pipe_id.
	 * First we find out which eid has been allocated to the
	 * service...
	 */
	for (i = 0; i < ATH10K_HTC_EP_COUNT; i++) {
		if (htc->endpoint[i].service_id == service_id) {
			eid = htc->endpoint[i].eid;
			ep_found = true;
			break;
		}
	}

	if (!ep_found) {
		ret = -EINVAL;
		goto out;
	}

	/* Then we create the simplest mapping possible between pipeid
	 * and eid
	 */
	*ul_pipe = *dl_pipe = (u8)eid;

	/* Normally, HTT will use the upper part of the extended
	 * mailbox address space (ext_info[1].htc_ext_addr) and WMI ctrl
	 * the lower part (ext_info[0].htc_ext_addr).
	 * If fw wants swapping of mailbox addresses, the opposite is true.
	 */
	if (ar_sdio->swap_mbox) {
		htt_addr = ar_sdio->mbox_info.ext_info[0].htc_ext_addr;
		wmi_addr = ar_sdio->mbox_info.ext_info[1].htc_ext_addr;
		htt_mbox_size = ar_sdio->mbox_info.ext_info[0].htc_ext_sz;
		wmi_mbox_size = ar_sdio->mbox_info.ext_info[1].htc_ext_sz;
	} else {
		htt_addr = ar_sdio->mbox_info.ext_info[1].htc_ext_addr;
		wmi_addr = ar_sdio->mbox_info.ext_info[0].htc_ext_addr;
		htt_mbox_size = ar_sdio->mbox_info.ext_info[1].htc_ext_sz;
		wmi_mbox_size = ar_sdio->mbox_info.ext_info[0].htc_ext_sz;
	}

	switch (service_id) {
	case ATH10K_HTC_SVC_ID_RSVD_CTRL:
		/* HTC ctrl ep mbox address has already been setup in
		 * ath10k_sdio_hif_start
		 */
		break;
	case ATH10K_HTC_SVC_ID_WMI_CONTROL:
		ar_sdio->mbox_addr[eid] = wmi_addr;
		ar_sdio->mbox_size[eid] = wmi_mbox_size;
		ath10k_dbg(ar, ATH10K_DBG_SDIO,
			   "WMI ctrl mbox addr = %x, mbox_size = %x\n",
			   ar_sdio->mbox_addr[eid], ar_sdio->mbox_size[eid]);
		break;
	case ATH10K_HTC_SVC_ID_HTT_DATA_MSG:
		ar_sdio->mbox_addr[eid] = htt_addr;
		ar_sdio->mbox_size[eid] = htt_mbox_size;
		ath10k_dbg(ar, ATH10K_DBG_SDIO,
			   "HTT data mbox addr = %x, mbox_size = %x\n",
			   ar_sdio->mbox_addr[eid], ar_sdio->mbox_size[eid]);
		break;
	default:
		ath10k_err(ar, "Unsupported service ID: %x\n",
			   service_id);
		ret = -EINVAL;
	}

out:
	return ret;
}

void ath10k_sdio_hif_get_default_pipe(struct ath10k *ar,
				      u8 *ul_pipe, u8 *dl_pipe)
{
	ath10k_dbg(ar, ATH10K_DBG_SDIO, "sdio hif get default pipe\n");

	/* HTC ctrl ep (SVC id 1) always has eid (and pipe_id in our
	 * case) == 0
	 */
	*ul_pipe = 0;
	*dl_pipe = 0;
}

static int ath10k_sdio_hif_fetch_cal_eeprom(struct ath10k *ar, void **data,
					    size_t *data_len)
{
	return -EOPNOTSUPP;
}

static void ath10k_sdio_write32(struct ath10k *ar, u32 offset, u32 value)
{
}

static u32 ath10k_sdio_read32(struct ath10k *ar, u32 offset)
{
	return 0;
}

static void ath10k_sdio_hif_send_complete_check(struct ath10k *ar,
						u8 pipe, int force)
{
}

static u16 ath10k_sdio_hif_get_free_queue_number(struct ath10k *ar, u8 pipe)
{
	return 0;
}

static const struct ath10k_hif_ops ath10k_sdio_hif_ops = {
	.tx_sg			= ath10k_sdio_hif_tx_sg,
	.diag_read		= ath10k_sdio_hif_diag_read,
	.diag_write		= ath10k_sdio_diag_write_mem,
	.exchange_bmi_msg	= ath10k_sdio_hif_exchange_bmi_msg,
	.start			= ath10k_sdio_hif_start,
	.stop			= ath10k_sdio_hif_stop,
	.map_service_to_pipe	= ath10k_sdio_hif_map_service_to_pipe,
	.get_default_pipe	= ath10k_sdio_hif_get_default_pipe,
	.send_complete_check	= ath10k_sdio_hif_send_complete_check,
	.get_free_queue_number	= ath10k_sdio_hif_get_free_queue_number,
	.power_up		= ath10k_sdio_hif_power_up,
	.power_down		= ath10k_sdio_hif_power_down,
	.read32			= ath10k_sdio_read32,
	.write32		= ath10k_sdio_write32,
#ifdef CONFIG_PM
	.suspend		= ath10k_sdio_hif_suspend,
	.resume			= ath10k_sdio_hif_resume,
#endif
	.fetch_cal_eeprom	= ath10k_sdio_hif_fetch_cal_eeprom,
};

#ifdef CONFIG_PM_SLEEP

/* Empty handlers so that mmc subsystem doesn't remove us entirely during
 * suspend. We instead follow cfg80211 suspend/resume handlers.
 */
static int ath10k_sdio_pm_suspend(struct device *device)
{
	return 0;
}

static int ath10k_sdio_pm_resume(struct device *device)
{
	return 0;
}

static SIMPLE_DEV_PM_OPS(ath10k_sdio_pm_ops, ath10k_sdio_pm_suspend,
			 ath10k_sdio_pm_resume);

#define ATH10K_SDIO_PM_OPS (&ath10k_sdio_pm_ops)

#else

#define ATH10K_SDIO_PM_OPS NULL

#endif /* CONFIG_PM_SLEEP */

static int ath10k_sdio_probe(struct sdio_func *func,
			     const struct sdio_device_id *id)
{
	int ret;
	struct ath10k_sdio *ar_sdio;
	struct ath10k *ar;
	int i;
	enum ath10k_hw_rev hw_rev;

	hw_rev = ATH10K_HW_QCA6174;

	ar = ath10k_core_create(sizeof(*ar_sdio), &func->dev, ATH10K_BUS_SDIO,
				hw_rev, &ath10k_sdio_hif_ops);
	if (!ar) {
		dev_err(&func->dev, "failed to allocate core\n");
		return -ENOMEM;
	}

	ath10k_dbg(ar, ATH10K_DBG_BOOT,
		   "sdio new func %d vendor 0x%x device 0x%x block 0x%x/0x%x\n",
		   func->num, func->vendor, func->device,
		   func->max_blksize, func->cur_blksize);

	ar_sdio = ath10k_sdio_priv(ar);

	ar_sdio->dma_buffer = kzalloc(ATH10K_HIF_DMA_BUFFER_SIZE, GFP_KERNEL);
	if (!ar_sdio->dma_buffer) {
		ret = -ENOMEM;
		goto err_core_destroy;
	}

	ar_sdio->func = func;
	sdio_set_drvdata(func, ar_sdio);

	ar_sdio->is_disabled = true;
	ar_sdio->ar = ar;

	spin_lock_init(&ar_sdio->lock);
	spin_lock_init(&ar_sdio->wr_async_lock);
	spin_lock_init(&ar_sdio->irq_data.lock);
	mutex_init(&ar_sdio->dma_buffer_mutex);

	INIT_LIST_HEAD(&ar_sdio->bus_req_freeq);
	INIT_LIST_HEAD(&ar_sdio->wr_asyncq);

	INIT_WORK(&ar_sdio->wr_async_work, ath10k_sdio_write_async_work);
	ar_sdio->workqueue = create_singlethread_workqueue("ath10k_sdio_wq");
	if (!ar_sdio->workqueue)
		goto err;

	init_waitqueue_head(&ar_sdio->irq_wq);

	for (i = 0; i < ATH10K_SDIO_BUS_REQUEST_MAX_NUM; i++)
		ath10k_sdio_free_bus_req(ar_sdio, &ar_sdio->bus_req[i]);

	ar->dev_id = id->device;
	ar->id.vendor = id->vendor;
	ar->id.device = id->device;

	ath10k_sdio_set_mbox_info(ar_sdio);

	ret = ath10k_sdio_config(ar_sdio);
	if (ret) {
		ath10k_err(ar, "Failed to config sdio: %d\n", ret);
		goto err;
	}

	ret = ath10k_core_register(ar, 0/*chip_id is not applicaple for SDIO*/);
	if (ret) {
		ath10k_err(ar, "failed to register driver core: %d\n", ret);
		goto err;
	}

	return ret;

err:
	kfree(ar_sdio->dma_buffer);
err_core_destroy:
	ath10k_core_destroy(ar);

	return ret;
}

static void ath10k_sdio_remove(struct sdio_func *func)
{
	struct ath10k_sdio *ar_sdio;
	struct ath10k *ar;

	ar_sdio = sdio_get_drvdata(func);
	ar = ar_sdio->ar;

	ath10k_dbg(ar, ATH10K_DBG_BOOT,
		   "sdio removed func %d vendor 0x%x device 0x%x\n",
		   func->num, func->vendor, func->device);

	(void)ath10k_sdio_hif_disable_intrs(ar_sdio);
	cancel_work_sync(&ar_sdio->wr_async_work);
	ath10k_core_unregister(ar);
	ath10k_core_destroy(ar);

	kfree(ar_sdio->dma_buffer);
}

static const struct sdio_device_id ath10k_sdio_devices[] = {
	{SDIO_DEVICE(ATH10K_MANUFACTURER_CODE,
		     (ATH10K_MANUFACTURER_ID_AR6005_BASE | 0xA))},
	{},
};

MODULE_DEVICE_TABLE(sdio, ath10k_sdio_devices);

static struct sdio_driver ath10k_sdio_driver = {
	.name = "ath10k_sdio",
	.id_table = ath10k_sdio_devices,
	.probe = ath10k_sdio_probe,
	.remove = ath10k_sdio_remove,
	.drv.pm = ATH10K_SDIO_PM_OPS,
};

static int __init ath10k_sdio_init(void)
{
	int ret;

	ret = sdio_register_driver(&ath10k_sdio_driver);
	if (ret)
		pr_err("sdio driver registration failed: %d\n", ret);

	return ret;
}

static void __exit ath10k_sdio_exit(void)
{
	sdio_unregister_driver(&ath10k_sdio_driver);
}

module_init(ath10k_sdio_init);
module_exit(ath10k_sdio_exit);

MODULE_AUTHOR("Qualcomm Atheros");
MODULE_DESCRIPTION("Driver support for Qualcomm Atheros 802.11ac WLAN SDIO devices");
MODULE_LICENSE("Dual BSD/GPL");
