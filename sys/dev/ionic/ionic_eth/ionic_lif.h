/*
 * Copyright (c) 2017-2019 Pensando Systems, Inc.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#ifndef _IONIC_LIF_H_
#define _IONIC_LIF_H_

#include <linux/if_ether.h>

#include <linux/workqueue.h>

#include "ionic_osdep.h"
#include "ionic_rx_filter.h"

#define LIF_NAME_MAX_SZ			8
#define MAX_VLAN_TAG 			4095

struct adminq_stats {
	u64 comp_err;
};

struct tx_stats {
	u64 dma_map_err;
	u64 pkts;
	u64 bytes;
	u64 clean;
	u64 re_queue;
	u64 mbuf_defrag;
	u64 mbuf_defrag_err;
	u64 no_descs;
	u64 no_csum_offload;
	u64 csum_offload;
	u64 comp_err;
	u64 tso_ipv4;
	u64 tso_ipv6;
	u64 tso_max_sg;
	u64 tso_max_size;
	u64 bad_ethtype;	/* Unknown Ethernet frame. */
	u64 wdog_expired;
};

struct rx_stats {
	u64 dma_map_err;
	u64 alloc_err;
	u64 pkts;
	u64 bytes;
	u64 csum_ip_ok;
	u64 csum_ip_bad;
	u64 csum_l4_ok;
	u64 csum_l4_bad;
	/* Only for debugging. */
	u64 mbuf_alloc;
	u64 mbuf_free;
	u64 isr_count; 	// Not required.
	u64 task;	/* Number of time task was invoked. */
	u64 comp_err;
	u64 length_err;

	u64 rss_ip4;
	u64 rss_tcp_ip4;
	u64 rss_udp_ip4;
	u64 rss_ip6;
	u64 rss_tcp_ip6;
	u64 rss_udp_ip6;
	u64 rss_unknown;
};

struct ionic_rx_buf {
	struct mbuf *m;
	bus_dmamap_t dma_map;
	uint32_t sg_buf_len;	/* If SG is used, its buffer length. */
};

struct ionic_tx_buf {
	struct mbuf *m;
	bus_dmamap_t dma_map;
	uint64_t timestamp;
	uint64_t pa_addr; 		/* Cache address to avoid access to command ring. */
};

struct adminq {
	char name[QUEUE_NAME_MAX_SZ];

	struct lif *lif;
	unsigned int num_descs;

	unsigned int pid;

	unsigned int index;
	unsigned int type;
	unsigned int hw_index;
	unsigned int hw_type;
	u64 dbval;

	bus_dma_tag_t buf_tag;
	struct ionic_dma_info cmd_dma; 		/* DMA ring for command and completion. */
	dma_addr_t cmd_ring_pa;
	dma_addr_t comp_ring_pa;
	uint32_t total_ring_size;

	struct mtx mtx;
	char mtx_name[QUEUE_NAME_MAX_SZ];
	unsigned int head_index;		/* Index for buffer and command descriptors. */
	unsigned int tail_index;
	unsigned int comp_index;		/* Index for completion descriptors. */
	int done_color; 			/* Expected comletion color. */

	struct adminq_stats stats;
	struct intr intr;

	struct ionic_admin_ctx **ctx_ring;
	/*
	 * H/w command and completion descriptor rings.
	 * Points to area allocated by DMA.
	 */
	struct admin_cmd *cmd_ring;
	struct admin_comp *comp_ring;
};

struct notifyq {
	char name[QUEUE_NAME_MAX_SZ];

	struct lif *lif;
	unsigned int num_descs;

	unsigned int pid;

	unsigned int index;
	unsigned int type;
	unsigned int hw_index;
	unsigned int hw_type;

	struct ionic_dma_info cmd_dma; 		/* DMA ring for command and completion. */
	dma_addr_t cmd_ring_pa;
	uint32_t total_ring_size;

	struct mtx mtx;
	char mtx_name[QUEUE_NAME_MAX_SZ];
	int comp_index;						/* Index for completion descriptors. */

	struct intr intr;

	struct task task;
	struct taskqueue *taskq;
	/*
	 * H/w command and completion descriptor rings.
	 * Points to area allocated by DMA.
	 */
	struct notifyq_cmd *cmd_ring;
	union notifyq_comp *comp_ring;
};

struct rxque {
	char name[QUEUE_NAME_MAX_SZ];

	struct lif *lif;
	unsigned int num_descs; /* Max number of descriptors. */
	uint32_t descs;			/* Descriptors posted in queue. */

	unsigned int pid;

	unsigned int index;
	unsigned int type;
	unsigned int hw_index;
	unsigned int hw_type;
	u64 dbval;

	struct ionic_rx_buf *rxbuf; /* S/w rx buffer descriptors. */
	bus_dma_tag_t buf_tag;

	struct ionic_dma_info cmd_dma; /* DMA ring for command and completion. */
	dma_addr_t cmd_ring_pa;	
	dma_addr_t comp_ring_pa;
	dma_addr_t sg_ring_pa;
	uint32_t total_ring_size;

	struct mtx rx_mtx;
	char mtx_name[QUEUE_NAME_MAX_SZ];

	unsigned int head_index;
	unsigned int tail_index;
	unsigned int comp_index;		/* Completion index. */
	int done_color; 			/* Expected completion color - 0/1. */

	struct rx_stats stats;
	struct intr intr;

	struct task task;			/* Queue completion handler. */
	struct task tx_task;			/* Tx deferred xmit handler. */
	struct taskqueue *taskq;

	struct lro_ctrl	lro;
	/*
	 * H/w command and completion descriptor rings.
	 * Points to area allocated by DMA.
	 */
	struct rxq_desc *cmd_ring;
	struct rxq_comp *comp_ring;
	struct rxq_sg_desc *sg_ring;	/* SG descriptors. */
};

/*
 * Transmit queue.
 * XXX: Interrupt resource for Tx is part of Rx.
 */
struct txque {
	char name[QUEUE_NAME_MAX_SZ];

	struct lif *lif;
	unsigned int num_descs;

	unsigned int pid;

	unsigned int index;
	unsigned int type;
	unsigned int hw_index;
	unsigned int hw_type;
	u64 dbval;

	struct ionic_tx_buf *txbuf;	/* S/w rx buffer descriptors. */
	bus_dma_tag_t buf_tag;
	struct ionic_dma_info cmd_dma; 	/* DMA ring for command and completion. */
	dma_addr_t cmd_ring_pa;
	dma_addr_t comp_ring_pa;
	dma_addr_t sg_ring_pa;
	uint32_t total_ring_size;

	struct mtx tx_mtx;
	char mtx_name[QUEUE_NAME_MAX_SZ];
	unsigned int head_index;	/* Index for buffer and command descriptors. */
	unsigned int tail_index;
	unsigned int comp_index;	/* Index for completion descriptors. */
	int done_color;			/* Expected completion color status. */

	unsigned long wdog_start;	/* In ticks */
	bool full;
	struct tx_stats stats;
	struct buf_ring	*br;

	/*
	 * H/w command and completion descriptor rings.
	 * Points to area allocated by DMA.
	 */
	struct txq_desc *cmd_ring;
	struct txq_comp *comp_ring;
	struct txq_sg_desc *sg_ring;	/* SG descriptors. */
};

struct ionic_mc_addr {
	u8  addr[ETHER_ADDR_LEN];
	bool present;
};

struct lif {
	char name[LIF_NAME_MAX_SZ];
	struct list_head list;
	struct net_device *netdev;

	u8 dev_addr[ETHER_ADDR_LEN] __aligned(sizeof(int));

	struct ionic *ionic;

	struct ifmedia          media;
	bool registered;

	unsigned int index;
	unsigned int hw_index;

	unsigned int kern_pid;
	u64 __iomem *kern_dbpage;

	struct workqueue_struct *adminq_wq;
	struct adminq *adminq;

	struct mtx wdog_mtx;
	struct workqueue_struct *wdog_wq;

	struct delayed_work adq_hb_work;
	unsigned long adq_hb_interval;
	bool adq_hb_resched;

	struct delayed_work txq_wdog_work;
	unsigned long txq_wdog_timeout;
	bool txq_wdog_resched;

	struct notifyq *notifyq;
	struct txque **txqs;
	struct rxque **rxqs;

	unsigned int nnqs;
	unsigned int neqs;
	unsigned int ntxqs;
	unsigned int nrxqs;

	unsigned int rx_mode;

	int rx_mbuf_size;		/* Rx mbuf size pool. */
	uint16_t max_frame_size;	/* MTU size. */

	u32 hw_features;		/* Features enabled in hardware, e.g. checksum, TSO etc. */

	uint16_t rss_types;
	u8 rss_hash_key[IONIC_RSS_HASH_KEY_SIZE];
	u8 *rss_ind_tbl;
	dma_addr_t rss_ind_tbl_pa;
	struct ionic_dma_info rss_dma;
	u32 rss_ind_tbl_sz;

	int intr_coalesce;		/* Interrupt coalescing value programmed. */

	struct mutex dbid_inuse_lock;
	unsigned long *dbid_inuse;
	unsigned int dbid_count;

	bool stay_registered;
	void *api_private;	/* For RoCE */
	void (*api_reset_cb)(void *api_private);

	uint64_t spurious; /* Spurious interrupt counter in legacy mode. */

	struct sysctl_oid *sysctl_ifnet;
	struct sysctl_ctx_list sysctl_ctx;
	struct sx sx;
	char sx_name[QUEUE_NAME_MAX_SZ];

	struct rx_filters rx_filters;

	struct ionic_mc_addr *mc_addrs;
	int num_mc_addrs;

	/* 4096 bit array for VLAN. */
	uint8_t vlan_bitmap[4096 / 8];
	int num_vlans;
	eventhandler_tag vlan_attach;
	eventhandler_tag vlan_detach;

	u64 last_eid;
	bool link_up;

	u32 link_speed;		/* units of 1Mbps: e.g. 10000 = 10Gbps */
	u16 link_down_count;

	u32 info_sz;
	dma_addr_t info_pa;
	struct ionic_dma_info info_dma;
	struct lif_info *info;

	u64 	num_dev_cmds;
	u64     num_resets;
};


/* lif lock. */
#define IONIC_LIF_LOCK_INIT(x)		sx_init(&(x)->sx, (x)->sx_name)
#define IONIC_LIF_LOCK_DESTROY(x)	sx_destroy(&(x)->sx)
#define IONIC_LIF_LOCK(x)		sx_xlock(&(x)->sx)
#define IONIC_LIF_UNLOCK(x)		sx_xunlock(&(x)->sx)
#define IONIC_LIF_LOCK_OWNED(x)		sx_xlocked(&(x)->sx)

#define IONIC_ADMIN_LOCK_INIT(x) 	mtx_init(&(x)->mtx, (x)->mtx_name, NULL, MTX_DEF)
#define IONIC_ADMIN_LOCK_DESTROY(x)	mtx_destroy(&(x)->mtx)
#define IONIC_ADMIN_LOCK(x)		mtx_lock(&(x)->mtx);
#define IONIC_ADMIN_UNLOCK(x)		mtx_unlock(&(x)->mtx);
#define IONIC_ADMIN_LOCK_OWNED(x) 	mtx_owned(&(x)->mtx)

#define IONIC_TX_LOCK_INIT(x)		mtx_init(&(x)->tx_mtx, (x)->mtx_name, NULL, MTX_DEF)
#define IONIC_TX_LOCK_DESTROY(x) 	mtx_destroy(&(x)->tx_mtx)
#define IONIC_TX_LOCK(x)		mtx_lock(&(x)->tx_mtx)
#define IONIC_TX_TRYLOCK(x)		mtx_trylock(&(x)->tx_mtx)
#define IONIC_TX_UNLOCK(x)		mtx_unlock(&(x)->tx_mtx)
#define IONIC_TX_LOCK_OWNED(x)		mtx_owned(&(x)->tx_mtx)

#define IONIC_RX_LOCK_INIT(x)		mtx_init(&(x)->rx_mtx, (x)->mtx_name, NULL, MTX_DEF)
#define IONIC_RX_LOCK_DESTROY(x)	mtx_destroy(&(x)->rx_mtx)
#define IONIC_RX_LOCK(x)		mtx_lock(&(x)->rx_mtx)
#define IONIC_RX_UNLOCK(x)		mtx_unlock(&(x)->rx_mtx)
#define IONIC_RX_LOCK_OWNED(x)		mtx_owned(&(x)->rx_mtx)

#define IONIC_MOD_INC(q, index) (((q)->index + 1) % (q)->num_descs)
/* Q-full condition, head + 1 == tail. */
#define IONIC_Q_FULL(q)		((((q)->head_index + 1) % (q)->num_descs) == (q)->tail_index)
#define IONIC_Q_EMPTY(q)	((q)->tail_index == (q)->head_index)

int ionic_stop(struct ifnet *ifp);
void ionic_open_or_stop(struct lif *lif);

int ionic_lif_identify(struct ionic *ionic);
int ionic_lifs_alloc(struct ionic *ionic);
void ionic_lifs_free(struct ionic *ionic);
void ionic_lifs_deinit(struct ionic *ionic);
int ionic_lifs_init(struct ionic *ionic);
int ionic_lifs_register(struct ionic *ionic);
void ionic_lifs_unregister(struct ionic *ionic);
int ionic_lifs_size(struct ionic *ionic);

int ionic_adminq_clean(struct adminq* adminq, int limit);
int ionic_notifyq_clean(struct notifyq* notifyq);

int ionic_dev_intr_reserve(struct lif *lif, struct intr *intr);
void ionic_dev_intr_unreserve(struct lif *lif, struct intr *intr);

struct lif *ionic_netdev_lif(struct ifnet *ifp);

int ionic_set_hw_features(struct lif *lif, uint32_t features);

int ionic_lif_rss_config(struct lif *lif, uint16_t types,
	const u8 *key, const u32 *indir);
 
void ionic_rx_fill(struct rxque *rxq);
int ionic_rx_clean(struct rxque *rxq, int rx_limit);
void ionic_rx_input(struct rxque *rxq, struct ionic_rx_buf *buf,
		struct rxq_comp *comp, struct rxq_desc *desc);

void ionic_tx_ring_doorbell(struct txque *txq, int index);
int ionic_tx_clean(struct txque* txq, int tx_limit);

int ionic_change_mtu(struct ifnet *ifp, int new_mtu);
void ionic_set_rx_mode(struct ifnet *ifp);

int ionic_set_multi(struct lif* lif);

int ionic_set_mac(struct ifnet *ifp);
int ionic_lif_reinit(struct lif *lif, bool wdog_reset_path);

void ionic_adminq_hb_resched(struct lif *lif);
void ionic_txq_wdog_resched(struct lif *lif);

int ionic_setup_intr_coal(struct lif *lif, int coal);
int ionic_firmware_update(struct lif *lif, const void *const fw_data, size_t fw_sz);

int ionic_lif_reset_stats(struct lif *lif);

extern int ionic_devcmd_timeout;
extern int ionic_rx_stride;
extern int ionic_tx_stride;
extern int ionic_rx_sg_size;
extern int ionic_tx_descs;
extern int ionic_rx_descs;
extern int adminq_descs;
extern int ionic_notifyq_descs;
extern int ionic_rx_fill_threshold;
extern int ionic_rx_process_limit;
extern int ionic_intr_coalesce;
extern int ionic_adminq_hb_interval;
extern int ionic_txq_wdog_timeout;

#endif /* _IONIC_LIF_H_ */
