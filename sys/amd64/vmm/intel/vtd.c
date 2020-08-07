/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2011 NetApp, Inc.
 * All rights reserved.
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
 * THIS SOFTWARE IS PROVIDED BY NETAPP, INC ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL NETAPP, INC OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD$
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/systm.h>
#include <sys/malloc.h>
#include <sys/sysctl.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/bus.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <dev/pci/pcireg.h>

#include <machine/vmparam.h>
#include <contrib/dev/acpica/include/acpi.h>

#include <machine/intr_machdep.h>
#include <machine/vmm.h>

#include "vmm_lapic.h"
#include "io/iommu.h"

/*
 * Documented in the "Intel Virtualization Technology for Directed I/O",
 * Architecture Spec, September 2008.
 */

#define VTD_DRHD_INCLUDE_PCI_ALL(Flags)  (((Flags) >> 0) & 0x1)

/* Section 10.4 "Register Descriptions" */
struct vtdmap {
	volatile uint32_t	version;
	volatile uint32_t	res0;
	volatile uint64_t	cap;
	volatile uint64_t	ext_cap;
	volatile uint32_t	gcr;
	volatile uint32_t	gsr;
	volatile uint64_t	rta;
	volatile uint64_t	ccr;
	volatile uint32_t	:32;/* Reserved 0x30-0x33 */
	volatile uint32_t	fsr;
	volatile uint32_t	fcr;
	volatile uint32_t	fdr;
	volatile uint32_t	far;
	volatile uint32_t	fuar;
 
	uint8_t			pad[0x70];
	volatile uint64_t	iata;	/* Offset 0xB8 */
} __attribute__ ((__packed__));

CTASSERT(offsetof(struct vtdmap, fsr) == 0x34);
CTASSERT(offsetof(struct vtdmap, iata) == 0xB8);

/*
 * Section 10.4 "Register Descriptions"
 * VT-d capability.
 */
#define	VTD_CAP_SAGAW(cap)	(((cap) >> 8) & 0x1F)
#define	VTD_CAP_ND(cap)		((cap) & 0x7)
#define	VTD_CAP_CM(cap)		(((cap) >> 7) & 0x1)
#define	VTD_CAP_SPS(cap)	(((cap) >> 34) & 0xF)
#define	VTD_CAP_RWBF(cap)	(((cap) >> 4) & 0x1)
#define	VTD_CAP_PI(cap)		(((cap) >> 59) & 0x1)
#define	VTD_CAP_NFR(cap)	(((cap) >> 40) & 0xFF)
#define	VTD_CAP_FRO(cap)	((((cap) >> 24) & 0x3FF) * 16)

/* VT-d extended capability. */
#define	VTD_ECAP_DI(ecap)	(((ecap) >> 2) & 0x1)
#define	VTD_ECAP_COHERENCY(ecap) ((ecap) & 0x1)
#define	VTD_ECAP_IRO(ecap)	(((ecap) >> 8) & 0x3FF)
#define	VTD_ECAP_IR(ecap)	(((ecap) >> 3) & 0x1)

/* VT-d command register. */
#define	VTD_GCR_CFI		(1 << 23)
#define	VTD_GCR_SIRTP		(1 << 24)
#define	VTD_GCR_IRE		(1 << 25)
#define	VTD_GCR_WBF		(1 << 27)
#define	VTD_GCR_SRTP		(1 << 30)
#define	VTD_GCR_TE		(1U << 31)

/* VT-d global register. */
#define	VTD_GSR_CFIS		(1 << 23)
#define	VTD_GSR_IRTPS		(1 << 24)
#define	VTD_GSR_IRES		(1 << 25) /* IR is enabled. */
#define	VTD_GSR_WBFS		(1 << 27)
#define	VTD_GSR_RTPS		(1 << 30)
#define	VTD_GSR_TES		(1U << 31)

#define VTD_GSR_IR_ENABLED(gsr)			\
	(((gsr) & (VTD_GSR_IRTPS | VTD_GSR_IRES)) == (VTD_GSR_IRTPS | VTD_GSR_IRES))

/* VT-d fault status register. */
#define VTD_FSR_INDEX(fsr)	((fsr >> 8) & 0xFF)

/* VT-d context command register. */
#define	VTD_CCR_ICC		(1UL << 63)	/* invalidate context cache */
#define	VTD_CCR_CIRG_GLOBAL	(1UL << 61)	/* global invalidation */

#define	VTD_IIR_IVT		(1UL << 63)	/* invalidation IOTLB */
#define	VTD_IIR_IIRG_GLOBAL	(1ULL << 60)	/* global IOTLB invalidation */
#define	VTD_IIR_IIRG_DOMAIN	(2ULL << 60)	/* domain IOTLB invalidation */
#define	VTD_IIR_IIRG_PAGE	(3ULL << 60)	/* page IOTLB invalidation */
#define	VTD_IIR_DRAIN_READS	(1ULL << 49)	/* drain pending DMA reads */
#define	VTD_IIR_DRAIN_WRITES	(1ULL << 48)	/* drain pending DMA writes */
#define	VTD_IIR_DOMAIN_P	32

#define	VTD_ROOT_PRESENT	0x1
#define	VTD_CTX_PRESENT		0x1
#define	VTD_CTX_TT_ALL		(1UL << 2)

#define	VTD_PTE_RD		(1UL << 0)
#define	VTD_PTE_WR		(1UL << 1)
#define	VTD_PTE_SUPERPAGE	(1UL << 7)
#define	VTD_PTE_ADDR_M		(0x000FFFFFFFFFF000UL)

#define VTD_RID2IDX(rid)	(((rid) & 0xff) * 2)

struct domain {
	uint64_t	*ptp;		/* first level page table page */
	int		pt_levels;	/* number of page table levels */
	int		addrwidth;	/* 'AW' field in context entry */
	int		spsmask;	/* supported super page sizes */
	u_int		id;		/* domain id */
	vm_paddr_t	maxaddr;	/* highest address to be mapped */
	SLIST_ENTRY(domain) next;
};

static SLIST_HEAD(, domain) domhead;

#define	DRHD_MAX_UNITS	8
static ACPI_DMAR_HARDWARE_UNIT	*drhds[DRHD_MAX_UNITS];
static int			drhd_num;
static struct vtdmap		*vtdmaps[DRHD_MAX_UNITS];
static int			max_domains;
typedef int			(*drhd_ident_func_t)(void);

static uint64_t root_table[PAGE_SIZE / sizeof(uint64_t)] __aligned(4096);
static uint64_t ctx_tables[256][PAGE_SIZE / sizeof(uint64_t)] __aligned(4096);

/* Maximum number of IR entries, can go upto 2 ^ 16. */
#define	IR_MAX_ENTRIES_POWER2	12
#define IR_MAX_ENTRIES		(1 << IR_MAX_ENTRIES_POWER2)

/*
 * See section 9.10 and 9.11 of VT-d spec, all IRTE are of
 * 128 bit long.
 */
static struct irte {
	uint64_t lsb;
	uint64_t msb;
} intr_remap_tbl[IR_MAX_ENTRIES] __aligned(4096);

/* Interrupt Remap common definitions. */
#define IRE_IR_PRESENT		1
#define IRE_IR_FPD		0x2
#define	IRE_VECTOR(vec)		(((vec) & 0xFF) << 16)

/* IRTE for Remapped Interrupts, see section 9.10 of Intel VT-d spec. */
#define IRE_IR_DM_LOGICAL	0x4
#define IRE_IR_RH		0x8
#define IRE_IR_TM_LEVEL		0x10
#define	IRE_IR_DST_xAPIC(d)	(((uint64_t)(d) & 0xFF) << 40)

/* IRTE for Posted Interrupt, section 9.11 of Intel VT-d spec. */
#define IRE_PIR_FLAG		0x8000
#define IRE_PIR_URG		0x4000
#define IRE_PIR_DESC_MSB_MASK	0xFFFFFFFF00000000UL
#define IRE_PIR_DESC_LSB_MASK	0xFFFFFFC0UL
#define IRE_PIR_DESC_LSB_SHIFT	32

#define IR_IOAPIC_INDEX_BIT15(i)	(((i) >> 15) & 1)
#define IR_IOAPIC_LO_INDEX_SHIFT	11

#define IR_IOAPIC_INDEX_LSB_BITS(i)	((i) & 0x7FFF)
#define IR_IOAPIC_HI_INDEX_SHIFT	17	/* BIT[63:49] */
#define IR_IOAPIC_HI_INDEX(index)	(IR_IOAPIC_INDEX_LSB_BITS(index) <<  \
						IR_IOAPIC_HI_INDEX_SHIFT);
#define IR_IOAPIC_HI_IR_FMT		0x10000 /* Interrupt Format, Bit[48] */
#define IR_IOAPIC_TM_LEVEL		0x8000	/* Trigger mode, level */

/*
 * Fault recording register.
 * See register definition in section 10.4.14 of vt-d spec.
 */
struct vtd_fault_reg {
	uint16_t :12;
	uint64_t :36;	/* XXX: fix for DMA remap */
	uint16_t ir_index;
	uint16_t sid;
	uint16_t :16;
	uint8_t	fr;	/* Bit[103:96] */
	uint32_t :23;
	uint8_t	 is_fault:1;
};
CTASSERT(sizeof(struct vtd_fault_reg) == 16);


static MALLOC_DEFINE(M_VTD, "vtd", "vtd");

SYSCTL_DECL(_hw_vmm);
SYSCTL_NODE(_hw_vmm, OID_AUTO, vtd, CTLFLAG_RD, NULL, "Intel VT-d/iommu bhyve device.");

static int ire_index;
SYSCTL_INT(_hw_vmm_vtd, OID_AUTO, irte_entries, CTLFLAG_RD, &ire_index,
	0, "VT-d IR entries used.");

static int enabled_vtd_ir = 0;
SYSCTL_INT(_hw_vmm_vtd, OID_AUTO, intr_remap, CTLFLAG_RD, &enabled_vtd_ir,
	0, "VT-d Interrupt remap enabled/disabled.");

static int enabled_vtd_pir = 0;
SYSCTL_INT(_hw_vmm_vtd, OID_AUTO, pir, CTLFLAG_RD,
	&enabled_vtd_pir, 1, "VT-d Posted Interrupt.");

/* XXX: for now make use of compatibility. */
static int enable_intr_remap_cfi = 1;
//TUNABLE_INT("hw.vtd_enable_ir_cfi", &enable_intr_remap_cfi);
SYSCTL_INT(_hw_vmm_vtd, OID_AUTO, intr_remap_cfi, CTLFLAG_RD,
	&enable_intr_remap_cfi, 0, "VT-d Interrupt remap allow CFI.");

int vtd_pir_enabled(void);
uint32_t vtd_get_apic_id(struct vm *vm, int vcpu);

static void vtd_fault_handler(void *arg);

/* Fault handler related. */
static struct callout vtd_fault_callout;
static struct mtx vtd_fault_mtx;

static uint64_t iata_val;
static struct mtx ir_mtx;

extern int ioapic_intr_remap(struct intsrc *isrc);
extern struct pic msi_pic;
extern uint64_t vmx_get_pir(struct vm *vm, int vcpu);

static int
vtd_max_domains(struct vtdmap *vtdmap)
{
	int nd;

	nd = VTD_CAP_ND(vtdmap->cap);

	switch (nd) {
	case 0:
		return (16);
	case 1:
		return (64);
	case 2:
		return (256);
	case 3:
		return (1024);
	case 4:
		return (4 * 1024);
	case 5:
		return (16 * 1024);
	case 6:
		return (64 * 1024);
	default:
		panic("vtd_max_domains: invalid value of nd (0x%0x)", nd);
	}
}

static u_int
domain_id(void)
{
	u_int id;
	struct domain *dom;

	/* Skip domain id 0 - it is reserved when Caching Mode field is set */
	for (id = 1; id < max_domains; id++) {
		SLIST_FOREACH(dom, &domhead, next) {
			if (dom->id == id)
				break;
		}
		if (dom == NULL)
			break;		/* found it */
	}
	
	if (id >= max_domains)
		panic("domain ids exhausted");

	return (id);
}

static struct vtdmap *
vtd_device_scope(uint16_t rid)
{
	int i, remaining, pathremaining;
	char *end, *pathend;
	struct vtdmap *vtdmap;
	ACPI_DMAR_HARDWARE_UNIT *drhd;
	ACPI_DMAR_DEVICE_SCOPE *device_scope;
	ACPI_DMAR_PCI_PATH *path;

	for (i = 0; i < drhd_num; i++) {
		drhd = drhds[i];

		if (VTD_DRHD_INCLUDE_PCI_ALL(drhd->Flags)) {
			/*
			 * From Intel VT-d arch spec, version 3.0:
			 * If a DRHD structure with INCLUDE_PCI_ALL flag Set is reported
			 * for a Segment, it must be enumerated by BIOS after all other
			 * DRHD structures for the same Segment.
			 */
			vtdmap = vtdmaps[i];
			return(vtdmap);
		}

		end = (char *)drhd + drhd->Header.Length;
		remaining = drhd->Header.Length - sizeof(ACPI_DMAR_HARDWARE_UNIT);
		while (remaining > sizeof(ACPI_DMAR_DEVICE_SCOPE)) {
			device_scope = (ACPI_DMAR_DEVICE_SCOPE *)(end - remaining);
			remaining -= device_scope->Length;

			switch (device_scope->EntryType){
				/* 0x01 and 0x02 are PCI device entries */
				case 0x01:
				case 0x02:
					break;
				default:
					continue;
			}

			if (PCI_RID2BUS(rid) != device_scope->Bus)
				continue;

			pathend = (char *)device_scope + device_scope->Length;
			pathremaining = device_scope->Length - sizeof(ACPI_DMAR_DEVICE_SCOPE);
			while (pathremaining >= sizeof(ACPI_DMAR_PCI_PATH)) {
				path = (ACPI_DMAR_PCI_PATH *)(pathend - pathremaining);
				pathremaining -= sizeof(ACPI_DMAR_PCI_PATH);

				if (PCI_RID2SLOT(rid) != path->Device)
					continue;
				if (PCI_RID2FUNC(rid) != path->Function)
					continue;

				vtdmap = vtdmaps[i];
				return (vtdmap);
			}
		}
	}

	/* No matching scope */
	return (NULL);
}

static void
vtd_wbflush(struct vtdmap *vtdmap)
{

	if (VTD_ECAP_COHERENCY(vtdmap->ext_cap) == 0)
		pmap_invalidate_cache();

	if (VTD_CAP_RWBF(vtdmap->cap)) {
		vtdmap->gcr |= VTD_GCR_WBF;
		while ((vtdmap->gsr & VTD_GSR_WBFS) != 0)
			;
	}
}

static void
vtd_ctx_global_invalidate(struct vtdmap *vtdmap)
{

	vtdmap->ccr = VTD_CCR_ICC | VTD_CCR_CIRG_GLOBAL;
	while ((vtdmap->ccr & VTD_CCR_ICC) != 0)
		;
}

static void
vtd_iotlb_global_invalidate(struct vtdmap *vtdmap)
{
	int offset;
	volatile uint64_t *iotlb_reg, val;

	vtd_wbflush(vtdmap);

	offset = VTD_ECAP_IRO(vtdmap->ext_cap) * 16;
	iotlb_reg = (volatile uint64_t *)((caddr_t)vtdmap + offset + 8);
	
	*iotlb_reg =  VTD_IIR_IVT | VTD_IIR_IIRG_GLOBAL |
		      VTD_IIR_DRAIN_READS | VTD_IIR_DRAIN_WRITES;

	while (1) {
		val = *iotlb_reg;
		if ((val & VTD_IIR_IVT) == 0)
			break;
	}
}

static void
vtd_translation_enable(struct vtdmap *vtdmap)
{

	vtdmap->gcr |= VTD_GCR_TE;
	while ((vtdmap->gsr & VTD_GSR_TES) == 0)
		;
}

static void
vtd_translation_disable(struct vtdmap *vtdmap)
{

	vtdmap->gcr = 0;
	while ((vtdmap->gsr & VTD_GSR_TES) != 0)
		;
}

static int
vtd_init(void)
{
	int i, units, remaining, tmp;
	struct vtdmap *vtdmap;
	vm_paddr_t ctx_paddr;
	char *end, envname[32];
	unsigned long mapaddr;
	ACPI_STATUS status;
	ACPI_TABLE_DMAR *dmar;
	ACPI_DMAR_HEADER *hdr;
	ACPI_DMAR_HARDWARE_UNIT *drhd;

	callout_init(&vtd_fault_callout, CALLOUT_MPSAFE);
	mtx_init(&vtd_fault_mtx, "vt-d fault", NULL, MTX_DEF);

	/*
	 * Allow the user to override the ACPI DMAR table by specifying the
	 * physical address of each remapping unit.
	 *
	 * The following example specifies two remapping units at
	 * physical addresses 0xfed90000 and 0xfeda0000 respectively.
	 * set vtd.regmap.0.addr=0xfed90000
	 * set vtd.regmap.1.addr=0xfeda0000
	 */
	for (units = 0; units < DRHD_MAX_UNITS; units++) {
		snprintf(envname, sizeof(envname), "vtd.regmap.%d.addr", units);
		if (getenv_ulong(envname, &mapaddr) == 0)
			break;
		vtdmaps[units] = (struct vtdmap *)PHYS_TO_DMAP(mapaddr);
	}

	if (units > 0)
		goto skip_dmar;

	/* Search for DMAR table. */
	status = AcpiGetTable(ACPI_SIG_DMAR, 0, (ACPI_TABLE_HEADER **)&dmar);
	if (ACPI_FAILURE(status))
		return (ENXIO);

	end = (char *)dmar + dmar->Header.Length;
	remaining = dmar->Header.Length - sizeof(ACPI_TABLE_DMAR);
	while (remaining > sizeof(ACPI_DMAR_HEADER)) {
		hdr = (ACPI_DMAR_HEADER *)(end - remaining);
		if (hdr->Length > remaining)
			break;
		/*
		 * From Intel VT-d arch spec, version 1.3:
		 * BIOS implementations must report mapping structures
		 * in numerical order, i.e. All remapping structures of
		 * type 0 (DRHD) enumerated before remapping structures of
		 * type 1 (RMRR) and so forth.
		 */
		if (hdr->Type != ACPI_DMAR_TYPE_HARDWARE_UNIT)
			break;

		drhd = (ACPI_DMAR_HARDWARE_UNIT *)hdr;
		drhds[units] = drhd;
		vtdmaps[units] = (struct vtdmap *)PHYS_TO_DMAP(drhd->Address);
		if (++units >= DRHD_MAX_UNITS)
			break;
		remaining -= hdr->Length;
	}

	if (units <= 0)
		return (ENXIO);

	callout_reset(&vtd_fault_callout, 1 * hz, vtd_fault_handler, NULL);
skip_dmar:
	drhd_num = units;

	max_domains = 64 * 1024; /* maximum valid value */
	for (i = 0; i < drhd_num; i++){
		vtdmap = vtdmaps[i];

		if (VTD_CAP_CM(vtdmap->cap) != 0)
			panic("vtd_init: invalid caching mode");

		/* take most compatible (minimum) value */
		if ((tmp = vtd_max_domains(vtdmap)) < max_domains)
			max_domains = tmp;
	}

	/*
	 * Set up the root-table to point to the context-entry tables
	 */
	for (i = 0; i < 256; i++) {
		ctx_paddr = vtophys(ctx_tables[i]);
		if (ctx_paddr & PAGE_MASK)
			panic("ctx table (0x%0lx) not page aligned", ctx_paddr);

		root_table[i * 2] = ctx_paddr | VTD_ROOT_PRESENT;
	}

	/* Call fault handler every second. */
	callout_reset(&vtd_fault_callout, 1 * hz, vtd_fault_handler, NULL);
	
	return (0);
}

static void
vtd_cleanup(void)
{
	callout_drain(&vtd_fault_callout);
}

static void
vtd_enable(void)
{
	int i;
	struct vtdmap *vtdmap;

	for (i = 0; i < drhd_num; i++) {
		vtdmap = vtdmaps[i];
		vtd_wbflush(vtdmap);

		/* Update the root table address */
		vtdmap->rta = vtophys(root_table);
		vtdmap->gcr |= VTD_GCR_SRTP;
		while ((vtdmap->gsr & VTD_GSR_RTPS) == 0)
			;

		vtd_ctx_global_invalidate(vtdmap);
		vtd_iotlb_global_invalidate(vtdmap);

		vtd_translation_enable(vtdmap);
	}
}

static void
vtd_disable(void)
{
	int i;
	struct vtdmap *vtdmap;

	for (i = 0; i < drhd_num; i++) {
		vtdmap = vtdmaps[i];
		vtd_translation_disable(vtdmap);
	}
}

static void
vtd_add_device(void *arg, uint16_t rid)
{
	int idx;
	uint64_t *ctxp;
	struct domain *dom = arg;
	vm_paddr_t pt_paddr;
	struct vtdmap *vtdmap;
	uint8_t bus;

	bus = PCI_RID2BUS(rid);
	ctxp = ctx_tables[bus];
	pt_paddr = vtophys(dom->ptp);
	idx = VTD_RID2IDX(rid);

	if (ctxp[idx] & VTD_CTX_PRESENT) {
		panic("vtd_add_device: device %x is already owned by "
		      "domain %d", rid,
		      (uint16_t)(ctxp[idx + 1] >> 8));
	}

	if ((vtdmap = vtd_device_scope(rid)) == NULL)
		panic("vtd_add_device: device %x is not in scope for "
		      "any DMA remapping unit", rid);

	/*
	 * Order is important. The 'present' bit is set only after all fields
	 * of the context pointer are initialized.
	 */
	ctxp[idx + 1] = dom->addrwidth | (dom->id << 8);

	if (VTD_ECAP_DI(vtdmap->ext_cap))
		ctxp[idx] = VTD_CTX_TT_ALL;
	else
		ctxp[idx] = 0;

	ctxp[idx] |= pt_paddr | VTD_CTX_PRESENT;

	/*
	 * 'Not Present' entries are not cached in either the Context Cache
	 * or in the IOTLB, so there is no need to invalidate either of them.
	 */
}

static void
vtd_remove_device(void *arg, uint16_t rid)
{
	int i, idx;
	uint64_t *ctxp;
	struct vtdmap *vtdmap;
	uint8_t bus;

	bus = PCI_RID2BUS(rid);
	ctxp = ctx_tables[bus];
	idx = VTD_RID2IDX(rid);

	/*
	 * Order is important. The 'present' bit is must be cleared first.
	 */
	ctxp[idx] = 0;
	ctxp[idx + 1] = 0;

	/*
	 * Invalidate the Context Cache and the IOTLB.
	 *
	 * XXX use device-selective invalidation for Context Cache
	 * XXX use domain-selective invalidation for IOTLB
	 */
	for (i = 0; i < drhd_num; i++) {
		vtdmap = vtdmaps[i];
		vtd_ctx_global_invalidate(vtdmap);
		vtd_iotlb_global_invalidate(vtdmap);
	}
}

#define	CREATE_MAPPING	0
#define	REMOVE_MAPPING	1

static uint64_t
vtd_update_mapping(void *arg, vm_paddr_t gpa, vm_paddr_t hpa, uint64_t len,
		   int remove)
{
	struct domain *dom;
	int i, spshift, ptpshift, ptpindex, nlevels;
	uint64_t spsize, *ptp;

	dom = arg;
	ptpindex = 0;
	ptpshift = 0;

	KASSERT(gpa + len > gpa, ("%s: invalid gpa range %#lx/%#lx", __func__,
	    gpa, len));
	KASSERT(gpa + len <= dom->maxaddr, ("%s: gpa range %#lx/%#lx beyond "
	    "domain maxaddr %#lx", __func__, gpa, len, dom->maxaddr));

	if (gpa & PAGE_MASK)
		panic("vtd_create_mapping: unaligned gpa 0x%0lx", gpa);

	if (hpa & PAGE_MASK)
		panic("vtd_create_mapping: unaligned hpa 0x%0lx", hpa);

	if (len & PAGE_MASK)
		panic("vtd_create_mapping: unaligned len 0x%0lx", len);

	/*
	 * Compute the size of the mapping that we can accommodate.
	 *
	 * This is based on three factors:
	 * - supported super page size
	 * - alignment of the region starting at 'gpa' and 'hpa'
	 * - length of the region 'len'
	 */
	spshift = 48;
	for (i = 3; i >= 0; i--) {
		spsize = 1UL << spshift;
		if ((dom->spsmask & (1 << i)) != 0 &&
		    (gpa & (spsize - 1)) == 0 &&
		    (hpa & (spsize - 1)) == 0 &&
		    (len >= spsize)) {
			break;
		}
		spshift -= 9;
	}

	ptp = dom->ptp;
	nlevels = dom->pt_levels;
	while (--nlevels >= 0) {
		ptpshift = 12 + nlevels * 9;
		ptpindex = (gpa >> ptpshift) & 0x1FF;

		/* We have reached the leaf mapping */
		if (spshift >= ptpshift) {
			break;
		}

		/*
		 * We are working on a non-leaf page table page.
		 *
		 * Create a downstream page table page if necessary and point
		 * to it from the current page table.
		 */
		if (ptp[ptpindex] == 0) {
			void *nlp = malloc(PAGE_SIZE, M_VTD, M_WAITOK | M_ZERO);
			ptp[ptpindex] = vtophys(nlp)| VTD_PTE_RD | VTD_PTE_WR;
		}

		ptp = (uint64_t *)PHYS_TO_DMAP(ptp[ptpindex] & VTD_PTE_ADDR_M);
	}

	if ((gpa & ((1UL << ptpshift) - 1)) != 0)
		panic("gpa 0x%lx and ptpshift %d mismatch", gpa, ptpshift);

	/*
	 * Update the 'gpa' -> 'hpa' mapping
	 */
	if (remove) {
		ptp[ptpindex] = 0;
	} else {
		ptp[ptpindex] = hpa | VTD_PTE_RD | VTD_PTE_WR;

		if (nlevels > 0)
			ptp[ptpindex] |= VTD_PTE_SUPERPAGE;
	}

	return (1UL << ptpshift);
}

static uint64_t
vtd_create_mapping(void *arg, vm_paddr_t gpa, vm_paddr_t hpa, uint64_t len)
{

	return (vtd_update_mapping(arg, gpa, hpa, len, CREATE_MAPPING));
}

static uint64_t
vtd_remove_mapping(void *arg, vm_paddr_t gpa, uint64_t len)
{

	return (vtd_update_mapping(arg, gpa, 0, len, REMOVE_MAPPING));
}

static void
vtd_invalidate_tlb(void *dom)
{
	int i;
	struct vtdmap *vtdmap;

	/*
	 * Invalidate the IOTLB.
	 * XXX use domain-selective invalidation for IOTLB
	 */
	for (i = 0; i < drhd_num; i++) {
		vtdmap = vtdmaps[i];
		vtd_iotlb_global_invalidate(vtdmap);
	}
}

static void *
vtd_create_domain(vm_paddr_t maxaddr)
{
	struct domain *dom;
	vm_paddr_t addr;
	int tmp, i, gaw, agaw, sagaw, res, pt_levels, addrwidth;
	struct vtdmap *vtdmap;

	if (drhd_num <= 0)
		panic("vtd_create_domain: no dma remapping hardware available");

	/*
	 * Calculate AGAW.
	 * Section 3.4.2 "Adjusted Guest Address Width", Architecture Spec.
	 */
	addr = 0;
	for (gaw = 0; addr < maxaddr; gaw++)
		addr = 1ULL << gaw;

	res = (gaw - 12) % 9;
	if (res == 0)
		agaw = gaw;
	else
		agaw = gaw + 9 - res;

	if (agaw > 64)
		agaw = 64;

	/*
	 * Select the smallest Supported AGAW and the corresponding number
	 * of page table levels.
	 */
	pt_levels = 2;
	sagaw = 30;
	addrwidth = 0;

	tmp = ~0;
	for (i = 0; i < drhd_num; i++) {
		vtdmap = vtdmaps[i];
		/* take most compatible value */
		tmp &= VTD_CAP_SAGAW(vtdmap->cap);
	}

	for (i = 0; i < 5; i++) {
		if ((tmp & (1 << i)) != 0 && sagaw >= agaw)
			break;
		pt_levels++;
		addrwidth++;
		sagaw += 9;
		if (sagaw > 64)
			sagaw = 64;
	}

	if (i >= 5) {
		panic("vtd_create_domain: SAGAW 0x%x does not support AGAW %d",
		      tmp, agaw);
	}

	dom = malloc(sizeof(struct domain), M_VTD, M_ZERO | M_WAITOK);
	dom->pt_levels = pt_levels;
	dom->addrwidth = addrwidth;
	dom->id = domain_id();
	dom->maxaddr = maxaddr;
	dom->ptp = malloc(PAGE_SIZE, M_VTD, M_ZERO | M_WAITOK);
	if ((uintptr_t)dom->ptp & PAGE_MASK)
		panic("vtd_create_domain: ptp (%p) not page aligned", dom->ptp);

#ifdef notyet
	/*
	 * XXX superpage mappings for the iommu do not work correctly.
	 *
	 * By default all physical memory is mapped into the host_domain.
	 * When a VM is allocated wired memory the pages belonging to it
	 * are removed from the host_domain and added to the vm's domain.
	 *
	 * If the page being removed was mapped using a superpage mapping
	 * in the host_domain then we need to demote the mapping before
	 * removing the page.
	 *
	 * There is not any code to deal with the demotion at the moment
	 * so we disable superpage mappings altogether.
	 */
	dom->spsmask = ~0;
	for (i = 0; i < drhd_num; i++) {
		vtdmap = vtdmaps[i];
		/* take most compatible value */
		dom->spsmask &= VTD_CAP_SPS(vtdmap->cap);
	}
#endif

	SLIST_INSERT_HEAD(&domhead, dom, next);

	return (dom);
}

static void
vtd_free_ptp(uint64_t *ptp, int level)
{
	int i;
	uint64_t *nlp;

	if (level > 1) {
		for (i = 0; i < 512; i++) {
			if ((ptp[i] & (VTD_PTE_RD | VTD_PTE_WR)) == 0)
				continue;
			if ((ptp[i] & VTD_PTE_SUPERPAGE) != 0)
				continue;
			nlp = (uint64_t *)PHYS_TO_DMAP(ptp[i] & VTD_PTE_ADDR_M);
			vtd_free_ptp(nlp, level - 1);
		}
	}

	bzero(ptp, PAGE_SIZE);
	free(ptp, M_VTD);
}

static void
vtd_destroy_domain(void *arg)
{
	struct domain *dom;
	
	dom = arg;

	SLIST_REMOVE(&domhead, dom, domain, next);
	vtd_free_ptp(dom->ptp, dom->pt_levels);
	free(dom, M_VTD);
}

#if 0 /* XXX: used  for legacy and non-posted MSI/X*/
static void
vtd_ir_irte_val(struct irte *ir, uint32_t srcID, uint32_t dstID, uint8_t vec,
	uint8_t dlm, bool tm, bool rh, bool phys)
{
	/*
	 * XXX: doesn't support x2 APIC.
	 */
	if (dstID >= 256) {
		printf("INTR_REMAP: x2 APIC not supported.");
		return;
	}

	/* XXX: SID verification */
	ir->msb = 0;
	ir->lsb = IRE_VECTOR(vec) | IRE_IR_DST_xAPIC(dstID) | IRE_IR_PRESENT |
		(phys ? 0 : IRE_IR_DM_LOGICAL) |
		(rh ? IRE_IR_RH : 0) |
		(tm ? IRE_IR_TM_LEVEL : 0);
}
#endif
/* XXX: srcID verification here???? */
static void
vtd_ir_pir_val(struct irte *ir, uint64_t pir_desc, uint16_t srcID,
	uint8_t vec)
{

	printf("PIR desc: 0x%lx\n", pir_desc);
	ir->msb = (pir_desc & IRE_PIR_DESC_MSB_MASK) | srcID;
	ir->lsb = IRE_VECTOR(vec) | IRE_IR_PRESENT |
		IRE_PIR_FLAG | IRE_PIR_URG |
		((pir_desc & IRE_PIR_DESC_LSB_MASK) << IRE_PIR_DESC_LSB_SHIFT);
}

static struct irte *
intel_ire_alloc(int *index, int count)
{
	mtx_lock_spin(&ir_mtx);
	if ((ire_index >= IR_MAX_ENTRIES) ||
		((ire_index + count) > IR_MAX_ENTRIES)) {
		printf("No more IRE slots.\n");
	}

	*index = ire_index;
	ire_index += count;
	mtx_unlock_spin(&ir_mtx);

	return (&intr_remap_tbl[*index]);
}


#define MSI_INTEL_IR_FMT		0x10	/* IR format, Bit[4] */
#define MSI_INTEL_IR_SHV		0x8	/* SHV valid, Bit[3] */

/* Setup MSI address and data format as per IR. */
static void
vtd_ir_msi_init(uint64_t *ir_addr, uint64_t *ir_data, int index, int n)
{
	*ir_addr =  MSI_INTEL_ADDR_BASE | (index << 5) | MSI_INTEL_IR_FMT |
			MSI_INTEL_IR_SHV;
			//((n > 1) ? MSI_INTEL_IR_SHV : 0);
	*ir_data = 0;
}

/*
 * Note: Addr and data is what is programmed by guest.
 */
static int
vtd_ir_msi_pir_setup(void *arg, uint64_t *addr, uint64_t *data,
	uint16_t srcID, int num)
{
	struct vm *vm;
	struct irte *ire;
	uint64_t ir_addr, ir_data, pir_desc;
	int i, index;
	uint32_t dstID;
	uint8_t vec, dlm;
	bool tm, phys;

	printf("%s is called\n", __func__);
	vm = arg;
	ire = intel_ire_alloc(&index, num);
	vtd_ir_msi_init(&ir_addr, &ir_data, index, num);

	if (!enabled_vtd_pir) {
		printf("warning, VT-d pir is not enabled");
		return (EIO);
	}

	KASSERT(enabled_vtd_pir, ("VT-d PIR is not enabled"));
	/* Decode the message and determine various bits. */
	msi_decode(*addr, *data, &dstID, &vec, &dlm, &tm, &phys);

	if (!phys) {
		printf("VT-d PIR doesn't support logical mode.");
		return (ENXIO);
	}
	/*
	 * Get the vcpu Posted Descriptor, IR with PIR will write
	 * directly to vcpu PIR.
	 * Note: dstID is vcpu APIC ID.
	 */
	pir_desc = vmx_get_pir(vm, dstID);
	vtd_ir_pir_val(ire, pir_desc, srcID, vec);

	printf("MSI INTR_REMAP PIR[0x%x] VM Addr new: 0x%lx(old: 0x%lx) Data new: 0x%lx(old: 0x%lx),"
		" IRE[0x%lx 0x%lx], entries=%d\n",
		index, ir_addr, *addr, ir_data, *data, ire->msb, ire->lsb, num);
	for ( i = 1; i < num ; i++) {
		ire[i].msb = ire[0].msb;
		/* Present bit is in LSB, write it at the end. */
		ire[i].lsb = ire[0].lsb;
	}

	*addr = ir_addr;
	*data = ir_data;
	return (0);
}

int
vtd_pir_enabled(void)
{
	return (enabled_vtd_pir);
}

static void
vtd_ir_verify(struct vtdmap *vtdmap)
{
	if (!enabled_vtd_ir)
		return;

	if (!VTD_GSR_IR_ENABLED(vtdmap->gsr))
		printf("INTR_REMAP disabled GSR:0x%x.\n", vtdmap->gsr);
	if (vtdmap->iata != iata_val)
		printf("IRTA mismatch, expected 0x%lx val 0x%lx\n", iata_val,
			vtdmap->iata);
}

/* Simple VT-d fault handler, called periodic. */
static void
vtd_fault_handler(void *arg)
{
	struct vtdmap *vtdmap;
	struct vtd_fault_reg *fault;
	uint32_t offset, *raw;
	uint8_t *ptr;
	int i, j, max, index;
	static int count;

	mtx_lock(&vtd_fault_mtx);
	count++;
	for (i = 0; i < drhd_num; i++) {
		vtdmap = vtdmaps[i];
		vtd_ir_verify(vtdmap);

		//printf("VT-d[%d] vtdmap: %p index: %d\n", i, vtdmap, offset);
		//if (!(vtdmap->fsr & 0x2))
		if (!(vtdmap->fsr))
			continue;
		offset = VTD_CAP_FRO(vtdmap->cap);
		ptr = (uint8_t *)vtdmap;
		printf("VT-d[%d] vtdmap: %p index: %d\n", i, vtdmap, offset);
		fault = (struct vtd_fault_reg *)&ptr[offset];

		index = VTD_FSR_INDEX(vtdmap->fsr);
		max = VTD_CAP_NFR(vtdmap->cap);
		for ( j = index; j < max; j++, fault++) {
			if (!fault->is_fault) {
				continue;
			}
			raw = (uint32_t *)fault;
			printf("  VT-d[%d] FSR:0x%x(max %d) "
				"Fault[%d]:0x%x INTR_REMAP index %d"
				"SID:0x%x [RAW 0x%x 0x%x 0x%x 0x%x]\n",
				i, vtdmap->fsr, max, j, fault->fr,
				fault->ir_index, fault->sid,
				 raw[3], raw[2], raw[1], raw[0]);
			fault->is_fault = 1;
		}
	}
	mtx_unlock(&vtd_fault_mtx);

	callout_reset(&vtd_fault_callout, 1 * hz, vtd_fault_handler, NULL);
}

static int
vtd_ir_enable(struct vtdmap *vtdmap)
{
	int i;

	if (VTD_ECAP_IR(vtdmap->ext_cap) == 0) {
		printf("VT-d extcap: 0x%lx\n", vtdmap->ext_cap);
		return (ENXIO);
	}

	/* First program INTR_REMAP table of size 1K entries. */
	iata_val = vtophys(intr_remap_tbl) | 5;//(IR_MAX_ENTRIES_POWER2 - 1);
	vtdmap->iata = iata_val;
	printf("INTR_REMAP IRTA:0x%lx\n", vtdmap->iata);
	printf("INTR_REMAP GCR:0x%x\n", vtdmap->gcr);

	vtdmap->gcr |= (VTD_GCR_SIRTP | VTD_GCR_IRE);
	printf("INTR_REMAP GCR:0x%x\n", vtdmap->gcr);
	
	/*
	 * XXX: Allow old/compatible and new(IR) format interrupts to co-exist
	 * in system to not change non passthrough devices. 
	 *  - Only passthrough devices will have to use IR format and use IR.
	 *  - other devices remian untouched????
	 */ 
	if (enable_intr_remap_cfi)
		vtdmap->gcr |= VTD_GCR_CFI;

	/* XXX: force IRE, SIRTP and CFI for debug. */
	vtdmap->gcr |= 0x3800000;
	printf("INTR_REMAP GCR:0x%x\n", vtdmap->gcr);
	i = 0;
	while (((vtdmap->gsr & VTD_GSR_IRTPS) != 0) && ( i < 1000)) {
		DELAY(1000);		/* 1 ms */
		i++;
	}

	printf("INTR_REMAP GCR:0x%x\n", vtdmap->gcr);
	if (!(VTD_GSR_IR_ENABLED(vtdmap->gsr))) {
		printf("Failed to enable Interrupt Remap, GSR:0x%x.\n",
			vtdmap->gsr);
		return (ENXIO);
	}

	printf("VT-d interrupt remap is enabled, GSR:0x%x\n", vtdmap->gsr);

	return (0);
}

static int
vtd_ir_init(void)
{
	struct vtdmap *vtdmap;
	int i, error;

	if (!drhd_num) {
		printf("No VT-d engine found.\n");
		return (EIO);
	}

	printf("Number of DRHD found: %d\n", drhd_num);

	for (i = 0; i < drhd_num; i++) {
		vtdmap = vtdmaps[i];
		error = vtd_ir_enable(vtdmap);
		if (error)
			return (error);

		enabled_vtd_ir = 1;
		printf("VT-d[%d] CAP: 0x%lx\n", i, vtdmap->cap);
		if (VTD_CAP_PI(vtdmap->cap)) {
			enabled_vtd_pir = 1;
			//intr_remap->ir_msi_pir = vtd_ir_msi_pir_setup;
		}
	}

	mtx_init(&ir_mtx, "intel-ir", NULL, MTX_SPIN);

	printf("INTR REMAP %s supported and enabled\n",
		enabled_vtd_pir ? "and POSTED INTR" : "");

	return (0);
}

struct iommu_ops iommu_ops_intel = {
	vtd_init,
	vtd_cleanup,
	vtd_enable,
	vtd_disable,
	vtd_create_domain,
	vtd_destroy_domain,
	vtd_create_mapping,
	vtd_remove_mapping,
	vtd_add_device,
	vtd_remove_device,
	vtd_invalidate_tlb,

	/* IR related. */
	vtd_ir_init,
	vtd_ir_msi_pir_setup,
};
