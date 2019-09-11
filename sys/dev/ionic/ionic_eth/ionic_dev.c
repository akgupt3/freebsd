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
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/slab.h>

#include "ionic_dev.h"
#include "ionic.h"

static void ionic_init_devinfo(struct ionic_dev *idev)
{
	idev->dev_info.asic_type = ioread8(&idev->dev_info_regs->asic_type);
	idev->dev_info.asic_rev = ioread8(&idev->dev_info_regs->asic_rev);

	memcpy_fromio(idev->dev_info.fw_version,
		      idev->dev_info_regs->fw_version,
		      IONIC_DEVINFO_FWVERS_BUFLEN);

	memcpy_fromio(idev->dev_info.serial_num,
		      idev->dev_info_regs->serial_num,
		      IONIC_DEVINFO_SERIAL_BUFLEN);

	idev->dev_info.fw_version[IONIC_DEVINFO_FWVERS_BUFLEN] = 0;
	idev->dev_info.serial_num[IONIC_DEVINFO_SERIAL_BUFLEN] = 0;
}

int
ionic_dev_setup(struct ionic *ionic)
{
	struct ionic_dev_bar *bar = ionic->bars;
	unsigned int num_bars = ionic->num_bars;
	struct ionic_dev *idev = &ionic->idev;
	struct device *dev = ionic->dev;
	u32 sig;

	/*
	 * BAR0 resources
	 */

	if (num_bars < 1) {
		dev_info(dev, "No bars found, aborting\n");
		return -EFAULT;
	}

	if (bar->len < BAR0_SIZE) {
		dev_info(dev, "Resource bar size %lu too small, aborting\n",
			 bar->len);
		return -EFAULT;
	}

	idev->dev_info_regs = bar->vaddr + BAR0_DEV_INFO_REGS_OFFSET;
	idev->dev_cmd_regs = bar->vaddr + BAR0_DEV_CMD_REGS_OFFSET;
	idev->intr_status = bar->vaddr + BAR0_INTR_STATUS_OFFSET;
	idev->intr_ctrl = bar->vaddr + BAR0_INTR_CTRL_OFFSET;

	sig = ioread32(&idev->dev_info_regs->signature);
	if (sig != IONIC_DEV_INFO_SIGNATURE) {
		dev_err(dev, "Incompatible firmware signature %x", sig);
		return -EFAULT;
	}

	ionic_init_devinfo(idev);

	dev_info(dev, "ASIC: %s rev: 0x%X serial num: %s fw_ver: %s\n",
		 ionic_dev_asic_name(idev->dev_info.asic_type),
		 idev->dev_info.asic_rev,
		 idev->dev_info.serial_num,
		 idev->dev_info.fw_version);

	/*
	 * BAR1 resources
	 */
	bar++;

	idev->db_pages = bar->vaddr;
	idev->phy_db_pages = bar->bus_addr;

	/* 
	 * BAR2 resources
	 */

	mutex_init(&idev->cmb_inuse_lock);

	bar++;
	if (num_bars < 3) {
		idev->phy_cmb_pages = 0;
		idev->cmb_npages = 0;
		idev->cmb_inuse = NULL;
		return 0;
	}

	idev->phy_cmb_pages = bar->bus_addr;
	idev->cmb_npages = bar->len / PAGE_SIZE;
	idev->cmb_inuse = malloc(BITS_TO_LONGS(idev->cmb_npages) * sizeof(long),
				  M_IONIC, M_WAITOK | M_ZERO);
	if (!idev->cmb_inuse) {
		idev->phy_cmb_pages = 0;
		idev->cmb_npages = 0;
	}

	return 0;
}

/* Devcmd Interface */
u8 ionic_dev_cmd_status(struct ionic_dev *idev)
{
	return ioread8(&idev->dev_cmd_regs->comp.status);
}

bool ionic_dev_cmd_done(struct ionic_dev *idev)
{
	return ioread32(&idev->dev_cmd_regs->done) & DEV_CMD_DONE;
}

void
ionic_dev_cmd_disable(struct ionic_dev *idev)
{
	struct ionic *ionic = container_of(idev, struct ionic, idev);

	KASSERT(IONIC_DEV_LOCK_OWNED(ionic), ("device not locked"));

	if (!ionic_dev_cmd_auto_disable) {
		IONIC_DEV_WARN(ionic->dev,
			       "dev_cmd disable skipped due to flag\n");
		return;
	}

	if (idev->dev_cmd_disabled)
		return;

	/*
	 * Respond to timeout or heartbeat failure by disabling the device
	 * command interface. This will allow the driver to fail quickly,
	 * so the module can be unloaded without waiting for many teardown
	 * commands to time out one after the other.
	 * After the failure, the driver no longer knows what state the
	 * device is in. The only way to recover the device and clear this
	 * flag is to unload and reload the module or reboot the system.
	 */
	IONIC_DEV_ERROR(ionic->dev, "disabling dev_cmd interface\n");
	idev->dev_cmd_disabled = true;
}

bool
ionic_dev_cmd_disabled(struct ionic_dev *idev)
{
	return idev->dev_cmd_disabled;
}

void
ionic_dev_cmd_comp(struct ionic_dev *idev, void *mem)
{
	union dev_cmd_comp *comp = mem;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(comp->words); i++)
		comp->words[i] = ioread32(&idev->dev_cmd_regs->comp.words[i]);
}

void
ionic_dev_cmd_go(struct ionic_dev *idev, union dev_cmd *cmd)
{
	unsigned int i;

	/* Bail out if the interface was disabled in response to an error */
	if (unlikely(ionic_dev_cmd_disabled(idev)))
		return;

	for (i = 0; i < ARRAY_SIZE(cmd->words); i++)
		iowrite32(cmd->words[i], &idev->dev_cmd_regs->cmd.words[i]);
	iowrite32(0, &idev->dev_cmd_regs->done);
	iowrite32(1, &idev->dev_cmd_regs->doorbell);
}

void
ionic_dev_cmd_nop(struct ionic_dev *idev)
{
	union dev_cmd cmd = {
		.nop.opcode = CMD_OPCODE_NOP,
	};

	ionic_dev_cmd_go(idev, &cmd);
}

/* Device commands */
void ionic_dev_cmd_identify(struct ionic_dev *idev, u16 ver)
{
	union dev_cmd cmd = {
		.identify.opcode = CMD_OPCODE_IDENTIFY,
		.identify.ver = ver,
	};

	ionic_dev_cmd_go(idev, &cmd);
}

void ionic_dev_cmd_init(struct ionic_dev *idev)
{
	union dev_cmd cmd = {
		.init.opcode = CMD_OPCODE_INIT,
		.init.type = 0,
	};

	ionic_dev_cmd_go(idev, &cmd);
}

void ionic_dev_cmd_reset(struct ionic_dev *idev)
{
	union dev_cmd cmd = {
		.reset.opcode = CMD_OPCODE_RESET,
	};

	ionic_dev_cmd_go(idev, &cmd);
}

/* Port commands */
void ionic_dev_cmd_port_identify(struct ionic_dev *idev)
{
	union dev_cmd cmd = {
		.port_init.opcode = CMD_OPCODE_PORT_IDENTIFY,
		.port_init.index = 0,
	};

	ionic_dev_cmd_go(idev, &cmd);
}

void ionic_dev_cmd_port_init(struct ionic_dev *idev)
{
	union dev_cmd cmd = {
		.port_init.opcode = CMD_OPCODE_PORT_INIT,
		.port_init.index = 0,
		.port_init.info_pa = idev->port_info_pa,
	};

	ionic_dev_cmd_go(idev, &cmd);
}

void ionic_dev_cmd_port_reset(struct ionic_dev *idev)
{
	union dev_cmd cmd = {
		.port_reset.opcode = CMD_OPCODE_PORT_RESET,
		.port_reset.index = 0,
	};

	ionic_dev_cmd_go(idev, &cmd);
}

void ionic_dev_cmd_port_state(struct ionic_dev *idev, uint8_t state)
{
	union dev_cmd cmd = {
		.port_setattr.opcode = CMD_OPCODE_PORT_SETATTR,
		.port_setattr.index = 0,
		.port_setattr.attr = IONIC_PORT_ATTR_STATE,
		.port_setattr.state = state,
	};

	ionic_dev_cmd_go(idev, &cmd);
}

void ionic_dev_cmd_port_speed(struct ionic_dev *idev, uint32_t speed)
{
	union dev_cmd cmd = {
		.port_setattr.opcode = CMD_OPCODE_PORT_SETATTR,
		.port_setattr.index = 0,
		.port_setattr.attr = IONIC_PORT_ATTR_SPEED,
		.port_setattr.speed = speed,
	};

	ionic_dev_cmd_go(idev, &cmd);
}

void ionic_dev_cmd_port_mtu(struct ionic_dev *idev, uint32_t mtu)
{
	union dev_cmd cmd = {
		.port_setattr.opcode = CMD_OPCODE_PORT_SETATTR,
		.port_setattr.index = 0,
		.port_setattr.attr = IONIC_PORT_ATTR_MTU,
		.port_setattr.mtu = mtu,
	};

	ionic_dev_cmd_go(idev, &cmd);
}

void ionic_dev_cmd_port_autoneg(struct ionic_dev *idev, uint8_t an_enable)
{
	union dev_cmd cmd = {
		.port_setattr.opcode = CMD_OPCODE_PORT_SETATTR,
		.port_setattr.index = 0,
		.port_setattr.attr = IONIC_PORT_ATTR_AUTONEG,
		.port_setattr.an_enable = an_enable,
	};

	ionic_dev_cmd_go(idev, &cmd);
}

void ionic_dev_cmd_port_fec(struct ionic_dev *idev, uint8_t fec_type)
{
	union dev_cmd cmd = {
		.port_setattr.opcode = CMD_OPCODE_PORT_SETATTR,
		.port_setattr.index = 0,
		.port_setattr.attr = IONIC_PORT_ATTR_FEC,
		.port_setattr.fec_type = fec_type,
	};

	ionic_dev_cmd_go(idev, &cmd);
}

void ionic_dev_cmd_port_pause(struct ionic_dev *idev, uint8_t pause_type)
{
	union dev_cmd cmd = {
		.port_setattr.opcode = CMD_OPCODE_PORT_SETATTR,
		.port_setattr.index = 0,
		.port_setattr.attr = IONIC_PORT_ATTR_PAUSE,
		.port_setattr.pause_type = pause_type,
	};

	ionic_dev_cmd_go(idev, &cmd);
}

void ionic_dev_cmd_port_loopback(struct ionic_dev *idev, uint8_t loopback_mode)
{
	union dev_cmd cmd = {
		.port_setattr.opcode = CMD_OPCODE_PORT_SETATTR,
		.port_setattr.index = 0,
		.port_setattr.attr = IONIC_PORT_ATTR_LOOPBACK,
		.port_setattr.loopback_mode = loopback_mode,
	};

	ionic_dev_cmd_go(idev, &cmd);
}

void ionic_dev_cmd_port_reset_stats(struct ionic_dev *idev)
{
	union dev_cmd cmd = {
		.port_setattr.opcode = CMD_OPCODE_PORT_SETATTR,
		.port_setattr.index = 0,
		.port_setattr.attr = IONIC_PORT_ATTR_STATS_CTRL,
		.port_setattr.stats_ctl = STATS_CTL_RESET,
	};

	ionic_dev_cmd_go(idev, &cmd);
}
/* LIF commands */
void ionic_dev_cmd_lif_identify(struct ionic_dev *idev, u8 type, u8 ver)
{
	union dev_cmd cmd = {
		.lif_identify.opcode = CMD_OPCODE_LIF_IDENTIFY,
		.lif_identify.type = type,
		.lif_identify.ver = ver,
	};

	ionic_dev_cmd_go(idev, &cmd);
}

void ionic_dev_cmd_lif_init(struct ionic_dev *idev, u32 index, dma_addr_t addr)
{
	union dev_cmd cmd = {
		.lif_init.opcode = CMD_OPCODE_LIF_INIT,
		.lif_init.index = index,
		.lif_init.info_pa = addr,
	};

	ionic_dev_cmd_go(idev, &cmd);
}

void ionic_dev_cmd_lif_reset(struct ionic_dev *idev, u32 index)
{
	union dev_cmd cmd = {
		.lif_init.opcode = CMD_OPCODE_LIF_RESET,
		.lif_init.index = index,
	};

	ionic_dev_cmd_go(idev, &cmd);
}

/* QoS commands */
void
ionic_dev_cmd_qos_class_identify(struct ionic_dev *idev)
{
	union dev_cmd cmd = {
		.qos_identify.opcode = CMD_OPCODE_QOS_CLASS_IDENTIFY,
	};

	ionic_dev_cmd_go(idev, &cmd);
}

void
ionic_dev_cmd_qos_class_init(struct ionic_dev *idev, uint8_t group)
{
	union dev_cmd cmd = {
		.qos_init.opcode = CMD_OPCODE_QOS_CLASS_INIT,
		.qos_init.group = group,
	};

	ionic_dev_cmd_go(idev, &cmd);
}

void
ionic_dev_cmd_qos_class_reset(struct ionic_dev *idev, uint8_t group)
{
	union dev_cmd cmd = {
		.qos_reset.opcode = CMD_OPCODE_QOS_CLASS_RESET,
		.qos_reset.group = group,
	};

	ionic_dev_cmd_go(idev, &cmd);
}

char *ionic_dev_asic_name(u8 asic_type)
{
	switch (asic_type) {
	case ASIC_TYPE_CAPRI:
		return "Capri";
	default:
		return "Unknown";
	}
}

int ionic_db_page_num(struct ionic *ionic, int lif_id, int pid)
{
	return lif_id * ionic->ident.dev.ndbpgs_per_lif + pid;
}

int ionic_intr_init(struct ionic_dev *idev, struct intr *intr,
		    unsigned long index)
{
	ionic_intr_clean(idev->intr_ctrl, index);
	intr->index = index;

	return 0;
}

int ionic_desc_avail(int ndescs, int head, int tail) 
{
	int avail = tail;

	if (head >= tail)
		avail += ndescs - head - 1;
	else
		avail -= head + 1;

	return avail;
}

static void
ionic_cmd_hb_work(struct work_struct *work)
{
	struct ionic_dev *idev =
		container_of(work, struct ionic_dev, cmd_hb_work.work);
	struct ionic *ionic = container_of(idev, struct ionic, idev);
	int err;

	if (!idev->cmd_hb_interval)
		return;

	/* Send a NOP command to monitor dev command queue */
	IONIC_DEV_LOCK(ionic);
	ionic_dev_cmd_nop(idev);
	err = ionic_dev_cmd_sleep_check(idev, ionic_devcmd_timeout * HZ);
	if (ionic_wdog_error_trigger == IONIC_WDOG_TRIG_DEVCMD) {
		IONIC_DEV_WARN(ionic->dev, "injecting error\n");
		err = -1;
		ionic_wdog_error_trigger = 0;
	}
	if (err) {
		IONIC_DEV_ERROR(ionic->dev, "command heartbeat failed\n");
		ionic_dev_cmd_disable(idev);
		IONIC_DEV_UNLOCK(ionic);

		/* Disable the heartbeat */
		idev->cmd_hb_interval = 0;
		return;
	}
	IONIC_DEV_UNLOCK(ionic);

	IONIC_WDOG_LOCK(idev);
	if (idev->cmd_hb_resched)
		queue_delayed_work(idev->wdog_wq, &idev->cmd_hb_work,
				   idev->cmd_hb_interval);
	IONIC_WDOG_UNLOCK(idev);
}

static void
ionic_cmd_hb_stop(struct ionic_dev *idev)
{
	IONIC_WDOG_LOCK(idev);
	idev->cmd_hb_resched = false;
	IONIC_WDOG_UNLOCK(idev);
	cancel_delayed_work_sync(&idev->cmd_hb_work);
}

void
ionic_cmd_hb_resched(struct ionic_dev *idev)
{
	/* Cancel all outstanding work */
	ionic_cmd_hb_stop(idev);

	/* Start again with the new hb_interval */
	idev->cmd_hb_resched = true;
	queue_delayed_work(idev->wdog_wq, &idev->cmd_hb_work,
			   idev->cmd_hb_interval);
}

static void
ionic_fw_hb_work(struct work_struct *work)
{
	struct ionic_dev *idev =
		container_of(work, struct ionic_dev, fw_hb_work.work);
	struct ionic *ionic = container_of(idev, struct ionic, idev);
	u8 fw_status;
	u32 fw_heartbeat;

	if (idev->fw_hb_state == IONIC_FW_HB_DISABLED ||
	    idev->fw_hb_state == IONIC_FW_HB_UNSUPPORTED)
		return;

	fw_status = ioread8(&idev->dev_info_regs->fw_status);
	if (ionic_wdog_error_trigger == IONIC_WDOG_TRIG_FWSTAT) {
		/* Persistent, don't reset trigger to 0 */
		/* NB: Set with a hint */
		IONIC_DEV_WARN(ionic->dev, "injecting fw_status 0\n");
		fw_status = 0;
	}
	/* If FW is ready, check fw_heartbeat; otherwise reschedule */
	if (fw_status != 0) {
		fw_heartbeat = ioread32(&idev->dev_info_regs->fw_heartbeat);

		if (ionic_wdog_error_trigger == IONIC_WDOG_TRIG_FWHB0) {
			/* NB: Set with a hint */
			IONIC_DEV_WARN(ionic->dev,
				       "injecting fw_heartbeat 0\n");
			fw_heartbeat = 0;
			ionic_wdog_error_trigger = 0;
		} else if (ionic_wdog_error_trigger == IONIC_WDOG_TRIG_FWHB1) {
			/* Persistent, don't reset trigger to 0 */
			IONIC_DEV_WARN(ionic->dev,
				       "injecting fw_heartbeat 1\n");
			fw_heartbeat = 1;
		}
		if (idev->fw_hb_state == IONIC_FW_HB_INIT) {
			if (fw_heartbeat == 0) {
				/* Unsupported firmware */
				IONIC_DEV_WARN(ionic->dev,
					       "fw heartbeat not supported\n");
				idev->fw_hb_state = IONIC_FW_HB_UNSUPPORTED;
				idev->fw_hb_interval = 0;
				return;
			} else {
				/* First reading; go RUNNING */
				idev->fw_hb_state = IONIC_FW_HB_RUNNING;
			}
		} else if (fw_heartbeat == idev->fw_hb_last) {
			/* Duplicate reading; go STALE or time out */
			if (idev->fw_hb_state == IONIC_FW_HB_RUNNING) {
				idev->fw_hb_state = IONIC_FW_HB_STALE;
			} else if (idev->fw_hb_state == IONIC_FW_HB_STALE) {
				IONIC_DEV_ERROR(ionic->dev,
						"fw heartbeat stuck (%u)\n",
						fw_heartbeat);
				IONIC_DEV_LOCK(ionic);
				ionic_dev_cmd_disable(idev);
				IONIC_DEV_UNLOCK(ionic);

				/* Disable the heartbeat */
				idev->fw_hb_state = IONIC_FW_HB_DISABLED;
				idev->fw_hb_interval = 0;
				return;
			}
		} else {
			/* Update stored value; go RUNNING */
			idev->fw_hb_last = fw_heartbeat;
			if (idev->fw_hb_state == IONIC_FW_HB_STALE) {
				idev->fw_hb_state = IONIC_FW_HB_RUNNING;
			}
		}
	}

	IONIC_WDOG_LOCK(idev);
	if (idev->fw_hb_resched)
		queue_delayed_work(idev->wdog_wq, &idev->fw_hb_work,
				   idev->fw_hb_interval);
	IONIC_WDOG_UNLOCK(idev);
}

static void
ionic_fw_hb_stop(struct ionic_dev *idev)
{
	IONIC_WDOG_LOCK(idev);
	idev->fw_hb_resched = false;
	IONIC_WDOG_UNLOCK(idev);
	cancel_delayed_work_sync(&idev->fw_hb_work);
}

void
ionic_fw_hb_resched(struct ionic_dev *idev)
{
	/* Cancel all outstanding work */
	ionic_fw_hb_stop(idev);

	/* Start again with the new hb_interval */
	idev->fw_hb_resched = true;
	queue_delayed_work(idev->wdog_wq, &idev->fw_hb_work,
			   idev->fw_hb_interval);
}

int
ionic_wdog_init(struct ionic *ionic)
{
	struct ionic_dev *idev = &ionic->idev;
	char name[16];

	snprintf(name, sizeof(name), "devwdwq%d",
		 le32_to_cpu(idev->port_info->status.id));
	idev->wdog_wq = create_singlethread_workqueue(name);
	IONIC_WDOG_LOCK_INIT(idev);

	/* Device command heartbeat watchdog */
	if (ionic_cmd_hb_interval > 0 &&
	    ionic_cmd_hb_interval < IONIC_WDOG_MIN_MS) {
		IONIC_DEV_WARN(ionic->dev,
			       "limiting cmd_hb_interval to %ums\n",
			       IONIC_WDOG_MIN_MS);
		ionic_cmd_hb_interval = IONIC_WDOG_MIN_MS;
	}
	idev->cmd_hb_interval =
		(unsigned long)ionic_cmd_hb_interval * HZ / 1000;

	INIT_DELAYED_WORK(&idev->cmd_hb_work, ionic_cmd_hb_work);
	idev->cmd_hb_resched = true;
	queue_delayed_work(idev->wdog_wq, &idev->cmd_hb_work,
			   idev->cmd_hb_interval);

	/* Firmware heartbeat */
	if (ionic_fw_hb_interval > 0 &&
	    ionic_fw_hb_interval < IONIC_WDOG_MIN_MS) {
		IONIC_DEV_WARN(ionic->dev,
			       "limiting fw_hb_interval to %ums\n",
			       IONIC_WDOG_MIN_MS);
		ionic_fw_hb_interval = IONIC_WDOG_MIN_MS;
	}
	if (ionic_fw_hb_interval > 0 &&
	    ionic_fw_hb_interval < IONIC_WDOG_FW_WARN_MS) {
		IONIC_DEV_WARN(ionic->dev,
			       "setting fw_hb_interval below %ums will "
			       "cause spurious timeouts\n",
			       IONIC_WDOG_FW_WARN_MS);
	}
	idev->fw_hb_interval =
		(unsigned long)ionic_fw_hb_interval * HZ / 1000;
	idev->fw_hb_state = ionic_fw_hb_interval ?
		IONIC_FW_HB_INIT : IONIC_FW_HB_DISABLED;

	INIT_DELAYED_WORK(&idev->fw_hb_work, ionic_fw_hb_work);
	idev->fw_hb_resched = true;
	queue_delayed_work(idev->wdog_wq, &idev->fw_hb_work,
	                   idev->fw_hb_interval);

	return 0;
}

void
ionic_wdog_deinit(struct ionic *ionic)
{
	struct ionic_dev *idev = &ionic->idev;

	ionic_cmd_hb_stop(idev);
	ionic_fw_hb_stop(idev);
	destroy_workqueue(idev->wdog_wq);
	IONIC_WDOG_LOCK_DESTROY(idev);
}
