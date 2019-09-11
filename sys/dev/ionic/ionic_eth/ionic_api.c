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

#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/printk.h>

#include "ionic_api.h"

#include "ionic.h"
#include "ionic_if.h"
#include "ionic_bus.h"
#include "ionic_dev.h"
#include "ionic_lif.h"
#include "ionic_txrx.h"

struct lif *get_netdev_ionic_lif(struct net_device *netdev,
				 const char *api_version,
				 enum ionic_api_prsn prsn)
{
	struct lif *lif;

	if (strcmp(api_version, IONIC_API_VERSION))
		return NULL;

	lif = ionic_netdev_lif(netdev);

	if (!lif || lif->ionic->is_mgmt_nic || prsn != IONIC_PRSN_RDMA)
		return NULL;

	return lif;
}
EXPORT_SYMBOL_GPL(get_netdev_ionic_lif);

struct device *ionic_api_get_device(struct lif *lif)
{
	return lif->ionic->dev;
}
EXPORT_SYMBOL_GPL(ionic_api_get_device);

bool ionic_api_stay_registered(struct lif *lif)
{
	return lif->stay_registered;
}
EXPORT_SYMBOL_GPL(ionic_api_stay_registered);

struct sysctl_oid *ionic_api_get_debugfs(struct lif *lif)
{
	return lif->sysctl_ifnet;
}
EXPORT_SYMBOL_GPL(ionic_api_get_debugfs);

void ionic_api_request_reset(struct lif *lif)
{
	netdev_warn(lif->netdev, "not implemented\n");
}
EXPORT_SYMBOL_GPL(ionic_api_request_reset);

void *ionic_api_get_private(struct lif *lif, enum ionic_api_prsn prsn)
{
	if (prsn != IONIC_PRSN_RDMA)
		return NULL;

	return lif->api_private;
}
EXPORT_SYMBOL_GPL(ionic_api_get_private);

int ionic_api_set_private(struct lif *lif, void *priv,
			  void (*reset_cb)(void *priv),
			  enum ionic_api_prsn prsn)
{
	if (prsn != IONIC_PRSN_RDMA)
		return -EINVAL;

	if (lif->api_private && priv)
		return -EBUSY;

	lif->api_private = priv;
	lif->api_reset_cb = reset_cb;

	return 0;
}
EXPORT_SYMBOL_GPL(ionic_api_set_private);

const struct ionic_devinfo *ionic_api_get_devinfo(struct lif *lif)
{
	return &lif->ionic->idev.dev_info;
}
EXPORT_SYMBOL_GPL(ionic_api_get_devinfo);

const union lif_identity *ionic_api_get_identity(struct lif *lif, int *lif_id)
{
	*lif_id = lif->index;

	return &lif->ionic->ident.lif;
}
EXPORT_SYMBOL_GPL(ionic_api_get_identity);

int ionic_api_get_intr(struct lif *lif, int *irq)
{
	struct intr intr_obj = {
		.index = INTR_INDEX_NOT_ASSIGNED
	};
	int err;

	if (!lif->neqs)
		return -ENOSPC;

	err = ionic_dev_intr_reserve(lif, &intr_obj);
	if (err)
		return -err;

	err = ionic_get_msix_irq(lif->ionic, intr_obj.index);
	if (err < 0) {
		ionic_dev_intr_unreserve(lif, &intr_obj);
		return err;
	}

	--lif->neqs;

	*irq = err;
	return intr_obj.index;
}
EXPORT_SYMBOL_GPL(ionic_api_get_intr);

void ionic_api_put_intr(struct lif *lif, int intr)
{
	struct intr intr_obj = {
		.index = intr
	};

	ionic_dev_intr_unreserve(lif, &intr_obj);

	++lif->neqs;
}
EXPORT_SYMBOL_GPL(ionic_api_put_intr);

int ionic_api_get_cmb(struct lif *lif, u32 *pgid, phys_addr_t *pgaddr, int order)
{
	struct ionic_dev *idev = &lif->ionic->idev;
	int ret;

	mutex_lock(&idev->cmb_inuse_lock);
	ret = bitmap_find_free_region(idev->cmb_inuse, idev->cmb_npages, order);
	mutex_unlock(&idev->cmb_inuse_lock);

	if (ret < 0)
		return ret;

	*pgid = (u32)ret;
	*pgaddr = idev->phy_cmb_pages + ret * PAGE_SIZE;

	return 0;
}
EXPORT_SYMBOL_GPL(ionic_api_get_cmb);

void ionic_api_put_cmb(struct lif *lif, u32 pgid, int order)
{
	struct ionic_dev *idev = &lif->ionic->idev;

	mutex_lock(&idev->cmb_inuse_lock);
	bitmap_release_region(idev->cmb_inuse, pgid, order);
	mutex_unlock(&idev->cmb_inuse_lock);
}
EXPORT_SYMBOL_GPL(ionic_api_put_cmb);

void ionic_api_kernel_dbpage(struct lif *lif,
			     struct ionic_intr __iomem **intr_ctrl,
			     u32 *dbid, u64 __iomem **dbpage,
			     phys_addr_t *xxx_dbpage_phys)
{
	int dbpage_num;

	*intr_ctrl = lif->ionic->idev.intr_ctrl;

	*dbid = lif->kern_pid;
	*dbpage = lif->kern_dbpage;

	/* XXX remove when rdma drops xxx_kdbid workaround */
	dbpage_num = ionic_db_page_num(lif->ionic, lif->index, 0);
	*xxx_dbpage_phys = ionic_bus_phys_dbpage(lif->ionic, dbpage_num);
}
EXPORT_SYMBOL_GPL(ionic_api_kernel_dbpage);

int ionic_api_get_dbid(struct lif *lif, u32 *dbid, phys_addr_t *addr)
{
	int id, dbpage_num;

	mutex_lock(&lif->dbid_inuse_lock);

	id = find_first_zero_bit(lif->dbid_inuse, lif->dbid_count);

	if (id == lif->dbid_count) {
		mutex_unlock(&lif->dbid_inuse_lock);
		return -ENOMEM;
	}

	set_bit(id, lif->dbid_inuse);

	mutex_unlock(&lif->dbid_inuse_lock);

	dbpage_num = ionic_db_page_num(lif->ionic, lif->index, id);

	*dbid = id;
	*addr = ionic_bus_phys_dbpage(lif->ionic, dbpage_num);

	return 0;
}
EXPORT_SYMBOL_GPL(ionic_api_get_dbid);

void ionic_api_put_dbid(struct lif *lif, int dbid)
{
	clear_bit(dbid, lif->dbid_inuse);
}
EXPORT_SYMBOL_GPL(ionic_api_put_dbid);

#define XXX_DEVCMD_HALF_PAGE 0x800

// XXX temp func to get side-band data from 2nd half page of dev_cmd reg space.
static int SBD_get(struct ionic_dev *idev, void *dst, size_t len)
{
	u32 __iomem *page32 = (void __iomem *)idev->dev_cmd_regs;
	u32 *dst32 = dst;
	unsigned int i, count;

	// check pointer and size alignment
	if ((unsigned long)dst & 0x3 || len & 0x3)
		return -EINVAL;

	// check length fits in 2nd half of page
	if (len > XXX_DEVCMD_HALF_PAGE)
		return -EINVAL;

	page32 += XXX_DEVCMD_HALF_PAGE / sizeof(*page32);
	count = len / sizeof(*page32);

	for (i = 0; i < count; ++i)
		dst32[i] = ioread32(&page32[i]);

	return 0;
}

// XXX temp func to put side-band data into 2nd half page of dev_cmd reg space.
static int SBD_put(struct ionic_dev *idev, void *src, size_t len)
{
	u32 __iomem *page32 = (void __iomem *)idev->dev_cmd_regs;
	u32 *src32 = src;
	unsigned int i, count;

	// check pointer and size alignment
	if ((unsigned long)src & 0x3 || len & 0x3)
		return -EINVAL;

	// check length fits in 2nd half of page
	if (len > XXX_DEVCMD_HALF_PAGE)
		return -EINVAL;

	page32 += XXX_DEVCMD_HALF_PAGE / sizeof(*page32);
	count = len / sizeof(*page32);

	for (i = 0; i < count; ++i)
		iowrite32(src32[i], &page32[i]);

	return 0;
}

static void ionic_adminq_ring_doorbell(struct adminq *adminq, int index)
{
	IONIC_NETDEV_INFO(adminq->lif->netdev, "ring doorbell for index: %d\n", index);

	ionic_dbell_ring(adminq->lif->kern_dbpage,
			 adminq->hw_type,
			 adminq->dbval | index);
}

static bool ionic_adminq_avail(struct adminq *adminq, int want)
{
	int avail;

	avail = ionic_desc_avail(adminq->num_descs, adminq->head_index, adminq->tail_index);

	return (avail > want);
}

static int ionic_api_do_devcmd(struct lif* lif, struct ionic_admin_ctx *ctx)
{
	struct adminq *adminq = lif->adminq;
	struct ionic_dev *idev = &lif->ionic->idev;
	int err;

	IONIC_ADMIN_LOCK(adminq);
	lif->num_dev_cmds++;

#ifdef IONIC_DEBUG
	IONIC_NETDEV_INFO(lif->netdev, "post admin dev command:\n");
	print_hex_dump_debug("cmd ", DUMP_PREFIX_OFFSET, 16, 1,
			     &ctx->cmd, sizeof(ctx->cmd), true);
#endif

	if (ctx->side_data) {
#ifdef IONIC_DEBUG
		print_hex_dump_debug("data ", DUMP_PREFIX_OFFSET, 16, 1,
				     ctx->side_data, ctx->side_data_len, true);
#endif
		err = SBD_put(idev, ctx->side_data, ctx->side_data_len);
		if (err) {
			IONIC_NETDEV_ERROR(lif->netdev, "SBD_put failed, error: %d\n", err);
			goto err_out;
		}
    }

	ionic_dev_cmd_go(idev, (void *)&ctx->cmd);

	/* XXX: sleep while holding spinlock... this is just temporary */
	err = ionic_dev_cmd_wait_check(idev, HZ * 10);
	if (err)
		goto err_out;

	ionic_dev_cmd_comp(idev, &ctx->comp);

	if (ctx->side_data) {
		err = SBD_get(idev, ctx->side_data, ctx->side_data_len);
		if (err) {
			IONIC_NETDEV_ERROR(lif->netdev, "SBD_get failed, error: %d\n", err);
			goto err_out;
		}
#ifdef IONIC_DEBUG
		print_hex_dump_debug("data readback ", DUMP_PREFIX_OFFSET, 16, 1,
				     ctx->side_data, ctx->side_data_len, true);
#endif
	}

#ifdef IONIC_DEBUG
	IONIC_NETDEV_INFO(lif->netdev, "comp admin dev command:\n");
	print_hex_dump_debug("comp ", DUMP_PREFIX_OFFSET, 16, 1,
			     &ctx->comp, sizeof(ctx->comp), true);
#endif

err_out:
	IONIC_ADMIN_UNLOCK(adminq);

	if (!err) {
		complete_all(&ctx->work);
	}

	return err;
}

static int ionic_api_do_adminq(struct lif* lif, struct ionic_admin_ctx *ctx)
{
	struct adminq *adminq = lif->adminq;

	struct admin_cmd *cmd;
	int err = 0;

	IONIC_ADMIN_LOCK(adminq);

	if (!ionic_adminq_avail(adminq, 1)) {
		err = ENOSPC;
		IONIC_QUE_ERROR(adminq, "adminq is hung!, head: %d tail: %d\n",
			adminq->head_index, adminq->tail_index);
		IONIC_ADMIN_UNLOCK(adminq);
		return (err);
	}

	adminq->ctx_ring[adminq->head_index] = ctx;
	cmd =  &adminq->cmd_ring[adminq->head_index];
	memcpy(cmd, &ctx->cmd, sizeof(ctx->cmd));

	IONIC_QUE_INFO(adminq, "post admin queue command %d@%d:\n",
		cmd->opcode, adminq->head_index);
#ifdef IONIC_DEBUG
	print_hex_dump_debug("cmd ", DUMP_PREFIX_OFFSET, 16, 1,
			     &ctx->cmd, sizeof(ctx->cmd), true);
#endif

	adminq->head_index = (adminq->head_index + 1) % adminq->num_descs;
	ionic_adminq_ring_doorbell(adminq, adminq->head_index);

	IONIC_ADMIN_UNLOCK(adminq);

	return err;
}

int ionic_api_adminq_post(struct lif *lif, struct ionic_admin_ctx *ctx)
{

	if (ionic_use_adminq)
		return (ionic_api_do_adminq(lif, ctx));
	else
		return ionic_api_do_devcmd(lif, ctx);
}
EXPORT_SYMBOL_GPL(ionic_api_adminq_post);
