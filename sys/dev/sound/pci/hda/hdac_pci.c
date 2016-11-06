/*-
 * Copyright (c) 2006 Stephane E. Potvin <sepotvin@videotron.ca>
 * Copyright (c) 2006 Ariff Abdullah <ariff@FreeBSD.org>
 * Copyright (c) 2008-2012 Alexander Motin <mav@FreeBSD.org>
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

/*
 * Intel High Definition Audio (Controller) driver for FreeBSD.
 */

#ifdef HAVE_KERNEL_OPTION_HEADERS
#include "opt_snd.h"
#endif

#include <dev/sound/pcm/sound.h>
#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>

#include <sys/ctype.h>
#include <sys/taskqueue.h>

#include <dev/sound/pci/hda/hdac_private.h>
#include <dev/sound/pci/hda/hdac_reg.h>
#include <dev/sound/pci/hda/hda_reg.h>
#include <dev/sound/pci/hda/hdac.h>

#define HDA_DRV_TEST_REV	"20120126_0002"
#define PCIS_MULTIMEDIA_HDA	0x03

SND_DECLARE_FILE("$FreeBSD$");

static const struct {
	uint32_t	model;
	const char	*desc;
	char		quirks_on;
	char		quirks_off;
} hdac_devices[] = {
	{ HDA_INTEL_OAK,     "Intel Oaktrail",	0, 0 },
	{ HDA_INTEL_CMLKLP,  "Intel Comet Lake-LP",	0, 0 },
	{ HDA_INTEL_CMLKH,   "Intel Comet Lake-H",	0, 0 },
	{ HDA_INTEL_BAY,     "Intel BayTrail",	0, 0 },
	{ HDA_INTEL_HSW1,    "Intel Haswell",	0, 0 },
	{ HDA_INTEL_HSW2,    "Intel Haswell",	0, 0 },
	{ HDA_INTEL_HSW3,    "Intel Haswell",	0, 0 },
	{ HDA_INTEL_BDW1,    "Intel Broadwell",	0, 0 },
	{ HDA_INTEL_BDW2,    "Intel Broadwell",	0, 0 },
	{ HDA_INTEL_BXTNT,   "Intel Broxton-T",	0, 0 },
	{ HDA_INTEL_CPT,     "Intel Cougar Point",	0, 0 },
	{ HDA_INTEL_PATSBURG,"Intel Patsburg",  0, 0 },
	{ HDA_INTEL_PPT1,    "Intel Panther Point",	0, 0 },
	{ HDA_INTEL_BR,      "Intel Braswell",	0, 0 },
	{ HDA_INTEL_LPT1,    "Intel Lynx Point",	0, 0 },
	{ HDA_INTEL_LPT2,    "Intel Lynx Point",	0, 0 },
	{ HDA_INTEL_WCPT,    "Intel Wildcat Point",	0, 0 },
	{ HDA_INTEL_WELLS1,  "Intel Wellsburg",	0, 0 },
	{ HDA_INTEL_WELLS2,  "Intel Wellsburg",	0, 0 },
	{ HDA_INTEL_LPTLP1,  "Intel Lynx Point-LP",	0, 0 },
	{ HDA_INTEL_LPTLP2,  "Intel Lynx Point-LP",	0, 0 },
	{ HDA_INTEL_SRPTLP,  "Intel Sunrise Point-LP",	0, 0 },
	{ HDA_INTEL_KBLKLP,  "Intel Kaby Lake-LP",	0, 0 },
	{ HDA_INTEL_SRPT,    "Intel Sunrise Point",	0, 0 },
	{ HDA_INTEL_KBLK,    "Intel Kaby Lake",	0, 0 },
	{ HDA_INTEL_KBLKH,   "Intel Kaby Lake-H",	0, 0 },
	{ HDA_INTEL_CFLK,    "Intel Coffee Lake",	0, 0 },
	{ HDA_INTEL_CMLKS,   "Intel Comet Lake-S",	0, 0 },
	{ HDA_INTEL_CNLK,    "Intel Cannon Lake",	0, 0 },
	{ HDA_INTEL_ICLK,    "Intel Ice Lake",		0, 0 },
	{ HDA_INTEL_CMLKLP,  "Intel Comet Lake-LP",	0, 0 },
	{ HDA_INTEL_CMLKH,   "Intel Comet Lake-H",	0, 0 },
	{ HDA_INTEL_TGLK,    "Intel Tiger Lake",	0, 0 },
	{ HDA_INTEL_GMLK,    "Intel Gemini Lake",	0, 0 },
	{ HDA_INTEL_82801F,  "Intel 82801F",	0, 0 },
	{ HDA_INTEL_63XXESB, "Intel 631x/632xESB",	0, 0 },
	{ HDA_INTEL_82801G,  "Intel 82801G",	0, 0 },
	{ HDA_INTEL_82801H,  "Intel 82801H",	0, 0 },
	{ HDA_INTEL_82801I,  "Intel 82801I",	0, 0 },
	{ HDA_INTEL_JLK,     "Intel Jasper Lake",	0, 0 },
	{ HDA_INTEL_82801JI, "Intel 82801JI",	0, 0 },
	{ HDA_INTEL_82801JD, "Intel 82801JD",	0, 0 },
	{ HDA_INTEL_PCH,     "Intel Ibex Peak",	0, 0 },
	{ HDA_INTEL_PCH2,    "Intel Ibex Peak",	0, 0 },
	{ HDA_INTEL_ELLK,    "Intel Elkhart Lake",	0, 0 },
	{ HDA_INTEL_JLK2,    "Intel Jasper Lake",	0, 0 },
	{ HDA_INTEL_BXTNP,   "Intel Broxton-P",	0, 0 },
	{ HDA_INTEL_SCH,     "Intel SCH",	0, 0 },
	{ HDA_NVIDIA_MCP51,  "NVIDIA MCP51",	0, HDAC_QUIRK_MSI },
	{ HDA_NVIDIA_MCP55,  "NVIDIA MCP55",	0, HDAC_QUIRK_MSI },
	{ HDA_NVIDIA_MCP61_1, "NVIDIA MCP61",	0, 0 },
	{ HDA_NVIDIA_MCP61_2, "NVIDIA MCP61",	0, 0 },
	{ HDA_NVIDIA_MCP65_1, "NVIDIA MCP65",	0, 0 },
	{ HDA_NVIDIA_MCP65_2, "NVIDIA MCP65",	0, 0 },
	{ HDA_NVIDIA_MCP67_1, "NVIDIA MCP67",	0, 0 },
	{ HDA_NVIDIA_MCP67_2, "NVIDIA MCP67",	0, 0 },
	{ HDA_NVIDIA_MCP73_1, "NVIDIA MCP73",	0, 0 },
	{ HDA_NVIDIA_MCP73_2, "NVIDIA MCP73",	0, 0 },
	{ HDA_NVIDIA_MCP78_1, "NVIDIA MCP78",	0, HDAC_QUIRK_64BIT },
	{ HDA_NVIDIA_MCP78_2, "NVIDIA MCP78",	0, HDAC_QUIRK_64BIT },
	{ HDA_NVIDIA_MCP78_3, "NVIDIA MCP78",	0, HDAC_QUIRK_64BIT },
	{ HDA_NVIDIA_MCP78_4, "NVIDIA MCP78",	0, HDAC_QUIRK_64BIT },
	{ HDA_NVIDIA_MCP79_1, "NVIDIA MCP79",	0, 0 },
	{ HDA_NVIDIA_MCP79_2, "NVIDIA MCP79",	0, 0 },
	{ HDA_NVIDIA_MCP79_3, "NVIDIA MCP79",	0, 0 },
	{ HDA_NVIDIA_MCP79_4, "NVIDIA MCP79",	0, 0 },
	{ HDA_NVIDIA_MCP89_1, "NVIDIA MCP89",	0, 0 },
	{ HDA_NVIDIA_MCP89_2, "NVIDIA MCP89",	0, 0 },
	{ HDA_NVIDIA_MCP89_3, "NVIDIA MCP89",	0, 0 },
	{ HDA_NVIDIA_MCP89_4, "NVIDIA MCP89",	0, 0 },
	{ HDA_NVIDIA_0BE2,   "NVIDIA (0x0be2)",	0, HDAC_QUIRK_MSI },
	{ HDA_NVIDIA_0BE3,   "NVIDIA (0x0be3)",	0, HDAC_QUIRK_MSI },
	{ HDA_NVIDIA_0BE4,   "NVIDIA (0x0be4)",	0, HDAC_QUIRK_MSI },
	{ HDA_NVIDIA_GT100,  "NVIDIA GT100",	0, HDAC_QUIRK_MSI },
	{ HDA_NVIDIA_GT104,  "NVIDIA GT104",	0, HDAC_QUIRK_MSI },
	{ HDA_NVIDIA_GT106,  "NVIDIA GT106",	0, HDAC_QUIRK_MSI },
	{ HDA_NVIDIA_GT108,  "NVIDIA GT108",	0, HDAC_QUIRK_MSI },
	{ HDA_NVIDIA_GT116,  "NVIDIA GT116",	0, HDAC_QUIRK_MSI },
	{ HDA_NVIDIA_GF119,  "NVIDIA GF119",	0, 0 },
	{ HDA_NVIDIA_GF110_1, "NVIDIA GF110",	0, HDAC_QUIRK_MSI },
	{ HDA_NVIDIA_GF110_2, "NVIDIA GF110",	0, HDAC_QUIRK_MSI },
	{ HDA_ATI_SB450,     "ATI SB450",	0, 0 },
	{ HDA_ATI_SB600,     "ATI SB600",	0, 0 },
	{ HDA_ATI_RS600,     "ATI RS600",	0, 0 },
	{ HDA_ATI_RS690,     "ATI RS690",	0, 0 },
	{ HDA_ATI_RS780,     "ATI RS780",	0, 0 },
	{ HDA_ATI_R600,      "ATI R600",	0, 0 },
	{ HDA_ATI_RV610,     "ATI RV610",	0, 0 },
	{ HDA_ATI_RV620,     "ATI RV620",	0, 0 },
	{ HDA_ATI_RV630,     "ATI RV630",	0, 0 },
	{ HDA_ATI_RV635,     "ATI RV635",	0, 0 },
	{ HDA_ATI_RV710,     "ATI RV710",	0, 0 },
	{ HDA_ATI_RV730,     "ATI RV730",	0, 0 },
	{ HDA_ATI_RV740,     "ATI RV740",	0, 0 },
	{ HDA_ATI_RV770,     "ATI RV770",	0, 0 },
	{ HDA_ATI_RV810,     "ATI RV810",	0, 0 },
	{ HDA_ATI_RV830,     "ATI RV830",	0, 0 },
	{ HDA_ATI_RV840,     "ATI RV840",	0, 0 },
	{ HDA_ATI_RV870,     "ATI RV870",	0, 0 },
	{ HDA_ATI_RV910,     "ATI RV910",	0, 0 },
	{ HDA_ATI_RV930,     "ATI RV930",	0, 0 },
	{ HDA_ATI_RV940,     "ATI RV940",	0, 0 },
	{ HDA_ATI_RV970,     "ATI RV970",	0, 0 },
	{ HDA_ATI_R1000,     "ATI R1000",	0, 0 },
	{ HDA_AMD_X370,      "AMD X370",	0, 0 },
	{ HDA_AMD_X570,      "AMD X570",	0, 0 },
	{ HDA_AMD_STONEY,    "AMD Stoney",	0, 0 },
	{ HDA_AMD_RAVEN,     "AMD Raven",	0, 0 },
	{ HDA_AMD_HUDSON2,   "AMD Hudson-2",	0, 0 },
	{ HDA_RDC_M3010,     "RDC M3010",	0, 0 },
	{ HDA_VIA_VT82XX,    "VIA VT8251/8237A",0, 0 },
	{ HDA_SIS_966,       "SiS 966/968",	0, 0 },
	{ HDA_ULI_M5461,     "ULI M5461",	0, 0 },
	/* Unknown */
	{ HDA_INTEL_ALL,  "Intel",		0, 0 },
	{ HDA_NVIDIA_ALL, "NVIDIA",		0, 0 },
	{ HDA_ATI_ALL,    "ATI",		0, 0 },
	{ HDA_AMD_ALL,    "AMD",		0, 0 },
	{ HDA_CREATIVE_ALL,    "Creative",	0, 0 },
	{ HDA_VIA_ALL,    "VIA",		0, 0 },
	{ HDA_SIS_ALL,    "SiS",		0, 0 },
	{ HDA_ULI_ALL,    "ULI",		0, 0 },
};

static const struct {
	uint16_t vendor;
	uint8_t reg;
	uint8_t mask;
	uint8_t enable;
} hdac_pcie_snoop[] = {
	{  INTEL_VENDORID, 0x00, 0x00, 0x00 },
	{    ATI_VENDORID, 0x42, 0xf8, 0x02 },
	{    AMD_VENDORID, 0x42, 0xf8, 0x02 },
	{ NVIDIA_VENDORID, 0x4e, 0xf0, 0x0f },
};

/****************************************************************************
 * Function prototypes
 ****************************************************************************/
static int	hdac_irq_alloc(struct hdac_softc *);
static void	hdac_irq_free(struct hdac_softc *);
static int	hdac_mem_alloc(struct hdac_softc *);
static void	hdac_mem_free(struct hdac_softc *);

static int	hdac_probe(device_t);
static int	hdac_attach(device_t);
static int	hdac_detach(device_t);

/****************************************************************************
 * void hdac_mem_free(struct hdac_softc *)
 *
 * Free up resources previously allocated by hdac_mem_alloc.
 ****************************************************************************/
static void
hdac_mem_free(struct hdac_softc *sc)
{
	struct hdac_mem *mem;

	mem = &sc->mem;
	if (mem->mem_res != NULL)
		bus_release_resource(sc->dev, SYS_RES_MEMORY, mem->mem_rid,
		    mem->mem_res);
	mem->mem_res = NULL;
}

/****************************************************************************
 * int hdac_mem_alloc(struct hdac_softc *)
 *
 * Allocate all the bus resources necessary to speak with the physical
 * controller.
 ****************************************************************************/
static int
hdac_mem_alloc(struct hdac_softc *sc)
{
	struct hdac_mem *mem;

	mem = &sc->mem;
	mem->mem_rid = PCIR_BAR(0);
	mem->mem_res = bus_alloc_resource_any(sc->dev, SYS_RES_MEMORY,
	    &mem->mem_rid, RF_ACTIVE);
	if (mem->mem_res == NULL) {
		device_printf(sc->dev,
		    "%s: Unable to allocate memory resource\n", __func__);
		return (ENOMEM);
	}
	mem->mem_tag = rman_get_bustag(mem->mem_res);
	mem->mem_handle = rman_get_bushandle(mem->mem_res);

	return (0);
}

/****************************************************************************
 * int hdac_irq_alloc(struct hdac_softc *)
 *
 * Allocate the resources necessary for interrupt handling.
 ****************************************************************************/
static int
hdac_irq_alloc(struct hdac_softc *sc)
{
	struct hdac_irq *irq;
	int result;

	irq = &sc->irq;
	irq->irq_rid = 0x0;

	if ((sc->quirks_off & HDAC_QUIRK_MSI) == 0 &&
	    (result = pci_msi_count(sc->dev)) == 1 &&
	    pci_alloc_msi(sc->dev, &result) == 0)
		irq->irq_rid = 0x1;

	irq->irq_res = bus_alloc_resource_any(sc->dev, SYS_RES_IRQ,
	    &irq->irq_rid, RF_SHAREABLE | RF_ACTIVE);
	if (irq->irq_res == NULL) {
		device_printf(sc->dev, "%s: Unable to allocate irq\n",
		    __func__);
		goto hdac_irq_alloc_fail;
	}

	return (0);

hdac_irq_alloc_fail:
	hdac_irq_free(sc);

	return (ENXIO);
}

/****************************************************************************
 * void hdac_irq_free(struct hdac_softc *)
 *
 * Free up resources previously allocated by hdac_irq_alloc.
 ****************************************************************************/
static void
hdac_irq_free(struct hdac_softc *sc)
{
	struct hdac_irq *irq;

	irq = &sc->irq;
	if (irq->irq_res != NULL && irq->irq_handle != NULL)
		bus_teardown_intr(sc->dev, irq->irq_res, irq->irq_handle);
	if (irq->irq_res != NULL)
		bus_release_resource(sc->dev, SYS_RES_IRQ, irq->irq_rid,
		    irq->irq_res);
	if (irq->irq_rid == 0x1)
		pci_release_msi(sc->dev);
	irq->irq_handle = NULL;
	irq->irq_res = NULL;
	irq->irq_rid = 0x0;
}



/****************************************************************************
 * Device Methods
 ****************************************************************************/

/****************************************************************************
 * int hdac_probe(device_t)
 *
 * Probe for the presence of an hdac. If none is found, check for a generic
 * match using the subclass of the device.
 ****************************************************************************/
static int
hdac_probe(device_t dev)
{
	int i, result;
	uint32_t model;
	uint16_t class, subclass;
	char desc[64];

	model = (uint32_t)pci_get_device(dev) << 16;
	model |= (uint32_t)pci_get_vendor(dev) & 0x0000ffff;
	class = pci_get_class(dev);
	subclass = pci_get_subclass(dev);

	bzero(desc, sizeof(desc));
	result = ENXIO;
	for (i = 0; i < nitems(hdac_devices); i++) {
		if (hdac_devices[i].model == model) {
			strlcpy(desc, hdac_devices[i].desc, sizeof(desc));
			result = BUS_PROBE_DEFAULT;
			break;
		}
		if (HDA_DEV_MATCH(hdac_devices[i].model, model) &&
		    class == PCIC_MULTIMEDIA &&
		    subclass == PCIS_MULTIMEDIA_HDA) {
			snprintf(desc, sizeof(desc), "%s (0x%04x)",
			    hdac_devices[i].desc, pci_get_device(dev));
			result = BUS_PROBE_GENERIC;
			break;
		}
	}
	if (result == ENXIO && class == PCIC_MULTIMEDIA &&
	    subclass == PCIS_MULTIMEDIA_HDA) {
		snprintf(desc, sizeof(desc), "Generic (0x%08x)", model);
		result = BUS_PROBE_GENERIC;
	}
	if (result != ENXIO) {
		strlcat(desc, " HDA Controller", sizeof(desc));
		device_set_desc_copy(dev, desc);
	}

	return (result);
}


static int
hdac_attach(device_t dev)
{
	struct hdac_softc *sc;
	int result;
	int i, devid = -1;
	uint32_t model;
	uint16_t class, subclass;
	uint16_t vendor;
	uint8_t v;

	sc = device_get_softc(dev);
	sc->dev = dev;
	HDA_BOOTVERBOSE(
		device_printf(dev, "PCI card vendor: 0x%04x, device: 0x%04x\n",
		    pci_get_subvendor(dev), pci_get_subdevice(dev));
		device_printf(dev, "HDA Driver Revision: %s\n",
		    HDA_DRV_TEST_REV);
	);

	model = (uint32_t)pci_get_device(dev) << 16;
	model |= (uint32_t)pci_get_vendor(dev) & 0x0000ffff;
	class = pci_get_class(dev);
	subclass = pci_get_subclass(dev);

	for (i = 0; i < nitems(hdac_devices); i++) {
		if (hdac_devices[i].model == model) {
			devid = i;
			break;
		}
		if (HDA_DEV_MATCH(hdac_devices[i].model, model) &&
		    class == PCIC_MULTIMEDIA &&
		    subclass == PCIS_MULTIMEDIA_HDA) {
			devid = i;
			break;
		}
	}

	if (devid >= 0) {
		sc->quirks_on = hdac_devices[devid].quirks_on;
		sc->quirks_off = hdac_devices[devid].quirks_off;
	} else {
		sc->quirks_on = 0;
		sc->quirks_off = 0;
	}
	if (resource_int_value(device_get_name(dev),
	    device_get_unit(dev), "msi", &i) == 0) {
		if (i == 0)
			sc->quirks_off |= HDAC_QUIRK_MSI;
		else {
			sc->quirks_on |= HDAC_QUIRK_MSI;
			sc->quirks_off |= ~HDAC_QUIRK_MSI;
		}
	}

	pci_enable_busmaster(dev);

	vendor = pci_get_vendor(dev);
	if (vendor == INTEL_VENDORID) {
		/* TCSEL -> TC0 */
		v = pci_read_config(dev, 0x44, 1);
		pci_write_config(dev, 0x44, v & 0xf8, 1);
		HDA_BOOTHVERBOSE(
			device_printf(dev, "TCSEL: 0x%02d -> 0x%02d\n", v,
			    pci_read_config(dev, 0x44, 1));
		);
	}

#if defined(__i386__) || defined(__amd64__)
	sc->flags |= HDAC_F_DMA_NOCACHE;

	if (resource_int_value(device_get_name(dev),
	    device_get_unit(dev), "snoop", &i) == 0 && i != 0) {
#else
	sc->flags &= ~HDAC_F_DMA_NOCACHE;
#endif
		/*
		 * Try to enable PCIe snoop to avoid messing around with
		 * uncacheable DMA attribute. Since PCIe snoop register
		 * config is pretty much vendor specific, there are no
		 * general solutions on how to enable it, forcing us (even
		 * Microsoft) to enable uncacheable or write combined DMA
		 * by default.
		 *
		 * http://msdn2.microsoft.com/en-us/library/ms790324.aspx
		 */
		for (i = 0; i < nitems(hdac_pcie_snoop); i++) {
			if (hdac_pcie_snoop[i].vendor != vendor)
				continue;
			sc->flags &= ~HDAC_F_DMA_NOCACHE;
			if (hdac_pcie_snoop[i].reg == 0x00)
				break;
			v = pci_read_config(dev, hdac_pcie_snoop[i].reg, 1);
			if ((v & hdac_pcie_snoop[i].enable) ==
			    hdac_pcie_snoop[i].enable)
				break;
			v &= hdac_pcie_snoop[i].mask;
			v |= hdac_pcie_snoop[i].enable;
			pci_write_config(dev, hdac_pcie_snoop[i].reg, v, 1);
			v = pci_read_config(dev, hdac_pcie_snoop[i].reg, 1);
			if ((v & hdac_pcie_snoop[i].enable) !=
			    hdac_pcie_snoop[i].enable) {
				HDA_BOOTVERBOSE(
					device_printf(dev,
					    "WARNING: Failed to enable PCIe "
					    "snoop!\n");
				);
#if defined(__i386__) || defined(__amd64__)
				sc->flags |= HDAC_F_DMA_NOCACHE;
#endif
			}
			break;
		}
#if defined(__i386__) || defined(__amd64__)
	}
#endif

	HDA_BOOTHVERBOSE(
		device_printf(dev, "DMA Coherency: %s / vendor=0x%04x\n",
		    (sc->flags & HDAC_F_DMA_NOCACHE) ?
		    "Uncacheable" : "PCIe snoop", vendor);
	);

	/* Allocate resources */
	result = hdac_mem_alloc(sc);
	if (result != 0)
		goto hdac_attach_fail;

	result = hdac_irq_alloc(sc);
	if (result != 0)
		goto hdac_attach_fail;

	result = hdac_attach_subclass(dev);
	if (result != 0)
		goto hdac_attach_fail;

	return (0);

hdac_attach_fail:
	hdac_irq_free(sc);
	hdac_mem_free(sc);

	return (ENXIO);
}



/****************************************************************************
 * int hdac_detach(device_t)
 *
 * Detach and free up resources utilized by the hdac device.
 ****************************************************************************/
static int
hdac_detach(device_t dev)
{
	struct hdac_softc *sc = device_get_softc(dev);
	int error;


	error = hdac_detach_subclass(dev);
	if (error != 0)
		return (error);
	hdac_irq_free(sc);
	hdac_mem_free(sc);
	return (0);
}

static int
hdac_pci_read_ivar(device_t dev, device_t child, int which, uintptr_t *result)
{

	switch (which) {
	case HDA_IVAR_SUBVENDOR_ID:
		*result = pci_get_subvendor(dev);
		break;
	case HDA_IVAR_SUBDEVICE_ID:
		*result = pci_get_subdevice(dev);
		break;
	default:
		return (hdac_read_ivar(dev, child, which, result));
	}
	return (0);
}

static device_method_t hdac_methods[] = {
	/* device interface */
	DEVMETHOD(device_probe,		hdac_probe),
	DEVMETHOD(device_attach,	hdac_attach),
	DEVMETHOD(device_detach,	hdac_detach),
	/* Bus interface */
	DEVMETHOD(bus_read_ivar,	hdac_pci_read_ivar),

	DEVMETHOD_END
};

static devclass_t hdac_devclass;
DEFINE_CLASS_1(hdac, hdac_pci_driver, hdac_methods,
    sizeof(struct hdac_softc), hdac_base_driver);
DRIVER_MODULE(snd_hda, pci, hdac_pci_driver, hdac_devclass, NULL, NULL);
