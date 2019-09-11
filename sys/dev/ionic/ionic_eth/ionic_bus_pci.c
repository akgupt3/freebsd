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

#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/pci.h>

#include "ionic.h"
#include "ionic_lif.h"
#include "ionic_bus.h"

#define PCI_VENDOR_ID_PENSANDO					0x1dd8

#define PCI_DEVICE_ID_PENSANDO_IONIC_ETH_PF		0x1002
#define PCI_DEVICE_ID_PENSANDO_IONIC_ETH_VF		0x1003
#define PCI_DEVICE_ID_PENSANDO_IONIC_ETH_MGMT	0x1004

/* Supported devices */
static const struct pci_device_id ionic_id_table[] = {
	{ PCI_VDEVICE(PENSANDO, PCI_DEVICE_ID_PENSANDO_IONIC_ETH_PF) },
	{ PCI_VDEVICE(PENSANDO, PCI_DEVICE_ID_PENSANDO_IONIC_ETH_VF) },
	{ PCI_VDEVICE(PENSANDO, PCI_DEVICE_ID_PENSANDO_IONIC_ETH_MGMT) },
	{ 0, }	/* end of table */
};

MODULE_DEVICE_TABLE(pci, ionic_id_table);

int ionic_get_msix_irq(struct ionic *ionic, unsigned int num)
{
	return ionic->pdev->dev.irq_start + num;
}

const char *ionic_bus_info(struct ionic *ionic)
{
	return pci_name(ionic->pdev);
}

int ionic_alloc_msix_vectors(struct ionic *ionic, unsigned int nintrs)
{
	struct resource_list_entry *rle;
	int avail, ret;

	avail = pci_msix_count(ionic->pdev->dev.bsddev);

	if (avail < nintrs) {
		IONIC_DEV_ERROR(ionic->dev, "Device need %d interrupts, available MSI/X vectors %u\n", nintrs, avail);
		return -EINVAL;
	}
	avail = nintrs;

	ret = pci_alloc_msix(ionic->pdev->dev.bsddev, &avail);

	if (ret) {
		IONIC_DEV_ERROR(ionic->dev, "try alloc nintrs %u avail %u ret %u\n", nintrs, avail, ret);
		return ret;
	}

	rle = linux_pci_get_rle(ionic->pdev, SYS_RES_IRQ, 1);
	ionic->pdev->dev.irq_start = rle->start;
	ionic->pdev->dev.irq_end = rle->start + avail;

	return avail;
}

void ionic_free_msix_vector(struct ionic *ionic)
{
	if (ionic->pdev->dev.irq_start)
		pci_release_msi(ionic->pdev->dev.bsddev);
}

static int ionic_map_bars(struct ionic *ionic)
{
	struct pci_dev *pdev = ionic->pdev;
	struct device *dev = ionic->dev;
	struct ionic_dev_bar *bars = ionic->bars;
	unsigned int i, j;

	ionic->num_bars = 0;
	for (i = 0, j = 0; i < IONIC_BARS_MAX; i++) {
		if (!(pci_resource_flags(pdev, i) & IORESOURCE_MEM))
			continue;
		bars[j].bus_addr = pci_resource_start(pdev, i);
		bars[j].len = pci_resource_len(pdev, i);

		if (j == 0) {
			bars[j].vaddr = ioremap(bars[j].bus_addr, bars[j].len);
			if (!bars[j].vaddr) {
				dev_err(dev, "Cannot memory-map BAR %d, aborting\n", j);
				return ENODEV;
			}
		}

		ionic->num_bars++;
		j++;
	}

	/* First two BARs are required. */
	if (j < 2) {
		IONIC_DEV_ERROR(dev, "Cannot memory-map BAR%u\n", j);
		return ENODEV;
	}

	return 0;
}

static void ionic_unmap_bars(struct ionic *ionic)
{
	struct ionic_dev_bar *bars = ionic->bars;
	unsigned int i;

	for (i = 0; i < IONIC_BARS_MAX; i++)
		if (bars[i].vaddr)
			iounmap(bars[i].vaddr);
}

phys_addr_t ionic_bus_phys_dbpage(struct ionic *ionic, int page_num)
{
	phys_addr_t addr = ionic->bars[IONIC_PCI_BAR_DBELL].bus_addr;
	phys_addr_t offset = (phys_addr_t)page_num << PAGE_SHIFT;

	return addr + offset;
}

void __iomem *ionic_bus_map_dbpage(struct ionic *ionic, int page_num)
{
	phys_addr_t addr = ionic_bus_phys_dbpage(ionic, page_num);

	return ioremap(addr, PAGE_SIZE);
}

void ionic_bus_unmap_dbpage(struct ionic *ionic, void __iomem *page)
{
	iounmap(page);
}

static int ionic_pci_init(struct pci_dev *pdev)
{
	struct device *dev = &pdev->dev;
	struct ionic *ionic = pci_get_drvdata(pdev);
	int err;

	pci_set_master(pdev);
	pci_enable_device(pdev);

	err = pci_request_regions(pdev, DRV_NAME);
	if (err) {
		IONIC_DEV_ERROR(dev, "Cannot request PCI regions, aborting\n");
		return (err);
	}

	err = pci_enable_io(pdev->dev.bsddev, SYS_RES_MEMORY);
	if (err) {
		IONIC_DEV_ERROR(dev, "Cannot enable PCI device, aborting\n");
		pci_release_regions(pdev);
		return (err);
	}

	/* Set DMA mask for RDMA device. */
	err = ionic_set_dma_mask(ionic);
	if (err) {
		IONIC_DEV_ERROR(dev, "Cannot set DMA mask, aborting\n");
		pci_release_regions(pdev);
		return (err);
	}

	err = ionic_map_bars(ionic);
	if (err) {
		pci_release_regions(pdev);
		return (err);
	}

	return (0);
}

static void ionic_pci_deinit(struct pci_dev *pdev)
{

	pci_release_regions(pdev);
}

static int ionic_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	struct device *dev = &pdev->dev;
	struct ionic *ionic;
	int err;

	ionic = malloc(sizeof(*ionic), M_IONIC, M_WAITOK | M_ZERO);
	if (!ionic)
		return -ENOMEM;

	ionic->pdev = pdev;
	ionic->dev = dev;
	pci_set_drvdata(pdev, ionic);

	ionic->is_mgmt_nic = ent->device == PCI_DEVICE_ID_PENSANDO_IONIC_ETH_MGMT;

	err = ionic_pci_init(pdev);
	if (err) {
		goto err_out_unmap_bars;
	}

	IONIC_DEV_LOCK_INIT(ionic);
	/* Configure the device */
	err = ionic_dev_setup(ionic);
	if (err) {
		IONIC_DEV_ERROR(dev, "Cannot setup device, aborting\n");
		goto err_out_unmap_bars;
	}

	err = ionic_identify(ionic);
	if (err) {
		IONIC_DEV_ERROR(dev, "Cannot identify device, aborting\n");
		goto err_out_unmap_bars;
	}

	err = ionic_init(ionic);
	if (err) {
		IONIC_DEV_ERROR(dev, "Cannot init device, aborting\n");
		goto err_out_unmap_bars;
	}

	/* Configure the ports */
	err = ionic_port_identify(ionic);
	if (err) {
		dev_err(dev, "Cannot identify port: %d, aborting\n", err);
		goto err_out_unmap_bars;
	}

	err = ionic_port_init(ionic);
	if (err) {
		dev_err(dev, "Cannot init port: %d, aborting\n", err);
		goto err_out_unmap_bars;
	}

	/* Qos */
	err = ionic_qos_class_identify(ionic);
	if (err) {
		dev_err(dev, "Cannot identify qos configuration: %d, aborting\n", err);
		goto err_out_unmap_bars;
	}

	/* Configure LIFs */
	err = ionic_lif_identify(ionic);
	if (err) {
		dev_err(dev, "Cannot identify LIFs: %d, aborting\n", err);
		goto err_out_unmap_bars;
	}

	err = ionic_lifs_size(ionic);
	if (err) {
		IONIC_DEV_ERROR(dev, "Cannot size LIFs, aborting\n");
		goto err_out_unmap_bars;
	}

	err = ionic_lifs_alloc(ionic);
	if (err) {
		IONIC_DEV_ERROR(dev, "Cannot allocate LIFs, aborting\n");
		goto err_out_free_lifs;
	}

	err = ionic_lifs_init(ionic);
	if (err) {
		IONIC_DEV_ERROR(dev, "Cannot init LIFs, aborting\n");
		goto err_out_deinit_lifs;
	}

	err = ionic_lifs_register(ionic);
	if (err) {
		IONIC_DEV_ERROR(dev, "Cannot register LIFs, aborting\n");
		goto err_out_deinit_lifs;
	}

	/* Configure command and firmware watchdogs */
	err = ionic_wdog_init(ionic);
	if (err) {
		 IONIC_DEV_ERROR(dev, "Cannot start device watchdogs\n");
	}

	pci_save_state(pdev->dev.bsddev);

	return 0;

err_out_deinit_lifs:
	ionic_lifs_deinit(ionic);
err_out_free_lifs:
	ionic_lifs_free(ionic);
	ionic_free_msix_vector(ionic);
err_out_unmap_bars:
	ionic_unmap_bars(ionic);
	ionic_pci_deinit(pdev);
	pci_set_drvdata(pdev, NULL);
	if(ionic->idev.cmb_inuse)
		free(ionic->idev.cmb_inuse, M_IONIC);
	free(ionic, M_IONIC);

	return err;
}

static void ionic_remove(struct pci_dev *pdev)
{
	struct ionic *ionic = pci_get_drvdata(pdev);

	KASSERT(ionic, ("ionic is NULL"));

	ionic_wdog_deinit(ionic);
	ionic_lifs_unregister(ionic);
	ionic_lifs_deinit(ionic);
	ionic_lifs_free(ionic);
	ionic_port_reset(ionic);
	ionic_reset(ionic);
	ionic_free_msix_vector(ionic);
	ionic_unmap_bars(ionic);
	ionic_pci_deinit(pdev);
	pci_set_drvdata(pdev, NULL);

	if (ionic->idev.cmb_inuse)
		free(ionic->idev.cmb_inuse, M_IONIC);
	free(ionic, M_IONIC);
}

static void ionic_shutdown(struct pci_dev *pdev)
{
	pci_disable_device(pdev);
}

static struct pci_driver ionic_driver = {
	/* XXX: used for interrupt description, limited in space. */
	.name = "ion", //DRV_NAME,
	.id_table = ionic_id_table,
	.probe = ionic_probe,
	.remove = ionic_remove,
	.shutdown	= ionic_shutdown,
};

int ionic_bus_register_driver(void)
{

	return pci_register_driver(&ionic_driver);
}

void ionic_bus_unregister_driver(void)
{
	pci_unregister_driver(&ionic_driver);
}
