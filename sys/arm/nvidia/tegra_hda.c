/*-
 * Copyright (c) 2016 Michal Meloun <mmel@FreeBSD.org>
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

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

/*
 * HDA driver for Tegra SoCs.
 */

#include "opt_snd.h"

#include <sys/param.h>
#include <sys/module.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/taskqueue.h>

#include <machine/fdt.h>
#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/extres/clk/clk.h>
#include <dev/extres/hwreset/hwreset.h>
#include <dev/fdt/fdt_common.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/sound/pci/hda/hdac.h>
#include <dev/sound/pci/hda/hdac_private.h>

#include "hdac_if.h"

/* Offset and size of HDA register bank */
#define	TEGRA_HDA_REG_OFF	0x8000
#define	TEGRA_HDA_REG_SIZE	0x8000

#define	HDA_IPFS_BAR0			0x80
#define	 HDA_IPFS_BAR0_START			0x40

#define	HDA_IPFS_CONFIG			0x180
#define	 HDA_IPFS_EN_FPCI			0x1

#define	HDA_IPFS_INTR			0x188
#define	 HDA_IPFS_EN_INTR			(1 << 16)

#define	HDA_CFG_CMD			0x1004
#define	 HDA_DISABLE_INTR			(1 << 10)
#define	 HDA_ENABLE_SERR			(1 << 8)
#define	 HDA_ENABLE_BUS_MASTER			(1 << 2)
#define	 HDA_ENABLE_MEM_SPACE			(1 << 1)
#define	 HDA_ENABLE_IO_SPACE			(1 << 0)
#define	HDA_CFG_BAR0			0x1010
#define	 HDA_CFG_BAR0_SIZE 			0x00004000

#define	WR4(_sc, _r, _v)	bus_write_4((_sc)->mem_res, (_r), (_v))
#define	RD4(_sc, _r)		bus_read_4((_sc)->mem_res, (_r))

#define	LOCK(_sc)		mtx_lock(&(_sc)->mtx)
#define	UNLOCK(_sc)		mtx_unlock(&(_sc)->mtx)
#define	SLEEP(_sc, timeout)						\
	mtx_sleep(sc, &sc->mtx, 0, "tegra_hda_wait", timeout);
#define	LOCK_INIT(_sc)							\
	mtx_init(&_sc->mtx, device_get_nameunit(_sc->dev), "tegra_hda", MTX_DEF)
#define	LOCK_DESTROY(_sc)	mtx_destroy(&_sc->mtx)
#define	ASSERT_LOCKED(_sc)	mtx_assert(&_sc->mtx, MA_OWNED)
#define	ASSERT_UNLOCKED(_sc)	mtx_assert(&_sc->mtx, MA_NOTOWNED)

static struct ofw_compat_data compat_data[] = {
	{"nvidia,tegra124-hda",	1},
	{NULL,			0}
};

struct tegra_hda_sc {
	struct hdac_softc	hdac_sc;	/* Must be first */
	device_t		dev;
	struct resource		*mem_res;
	struct resource		*irq_res;
	void			*irq_ih;
	struct mtx		mtx;

	clk_t			clk_hda;
	clk_t			clk_hda2hdmi;
	clk_t			clk_hda2codec_2x;
	hwreset_t		hwreset_hda;
	hwreset_t		hwreset_hda2hdmi;
	hwreset_t		hwreset_hda2codec_2x;
};

static uint8_t
tegra_hda_read_1(device_t dev, struct hdac_mem *mem, bus_size_t offs)
{
	uint32_t val32;

	val32 = bus_space_read_4(mem->mem_tag, mem->mem_handle, offs & ~3);
	return ((val32 >> (offs & 3) * 8) & 0xff);
}

static uint16_t
tegra_hda_read_2(device_t dev, struct hdac_mem *mem, bus_size_t offs)
{
	uint32_t val32;

	val32 = bus_space_read_4(mem->mem_tag, mem->mem_handle, offs & ~3);
	return ((val32 >> (offs & 3) * 8) & 0xffff);
}

static uint32_t
tegra_hda_read_4(device_t dev, struct hdac_mem *mem, bus_size_t offs)
{

	return(bus_space_read_4(mem->mem_tag, mem->mem_handle, offs));
}

static void
tegra_hda_write_1(device_t dev, struct hdac_mem *mem, bus_size_t offs,
    uint8_t val)
{
	uint32_t val32;

	val32 = bus_space_read_4(mem->mem_tag, mem->mem_handle, offs & ~3);
	val32 &= ~(0xff << (offs & 3) * 8);
	val32 |= (val << (offs & 3) * 8);
	bus_space_write_4(mem->mem_tag, mem->mem_handle, offs & ~3, val32);
}

static void
tegra_hda_write_2(device_t dev, struct hdac_mem *mem, bus_size_t offs,
   uint16_t val)
{
	uint32_t val32;

	val32 = bus_space_read_4(mem->mem_tag, mem->mem_handle, offs & ~3);
	val32 &= ~(0xffff << (offs & 3) * 8);
	val32 |= ((val & 0xffff) << (offs & 3) * 8);
	bus_space_write_4(mem->mem_tag, mem->mem_handle, offs & ~3, val32);
}

static void
tegra_hda_write_4(device_t dev, struct hdac_mem *mem, bus_size_t offs,
    uint32_t val)
{

	return (bus_space_write_4(mem->mem_tag, mem->mem_handle, offs, val));
}

static void
tegra_hda_intr(void *arg)
{
	struct tegra_hda_sc *sc;

	sc = arg;
	hdac_intr_handler(&sc->hdac_sc);
}

static int
tegra_hda_irq_setup(device_t dev)
{
	struct tegra_hda_sc *sc;
	int rv;

	sc = device_get_softc(dev);
	rv = bus_setup_intr(sc->dev, sc->irq_res, INTR_MPSAFE | INTR_TYPE_AV,
	    NULL, tegra_hda_intr, sc, &sc->irq_ih);
	if (rv != 0) {
		device_printf(sc->dev,
		    "%s: Could not setup interrupt handler (%x)\n",
		    __func__, rv);
		return (ENXIO);
	}

	return (0);
}

static void
tegra_hda_irq_teardown(device_t dev)
{
	struct tegra_hda_sc *sc;

	sc = device_get_softc(dev);
	if ( sc->irq_ih != NULL) {
		bus_teardown_intr(dev, sc->irq_res, sc->irq_ih);
		 sc->irq_ih = NULL;
	}
}

static void
tegra_hda_init(struct tegra_hda_sc *sc)
{
	uint32_t val;

	val = RD4(sc, HDA_IPFS_CONFIG);
	val |= HDA_IPFS_EN_FPCI;
	WR4(sc, HDA_IPFS_CONFIG, val);

	val = RD4(sc, HDA_CFG_CMD);
	val &= ~HDA_DISABLE_INTR;
	val |= HDA_ENABLE_SERR;
	val |= HDA_ENABLE_BUS_MASTER;
	val |= HDA_ENABLE_MEM_SPACE;
	val |= HDA_ENABLE_IO_SPACE;
	WR4(sc, HDA_CFG_CMD, val);

	WR4(sc, HDA_CFG_BAR0, 0xFFFFFFFF);
	WR4(sc, HDA_CFG_BAR0, HDA_CFG_BAR0_SIZE);

	WR4(sc, HDA_IPFS_BAR0, HDA_IPFS_BAR0_START);

	val = RD4(sc, HDA_IPFS_INTR);
	val |= HDA_IPFS_EN_INTR;
	WR4(sc, HDA_IPFS_INTR, val);
}

static int
get_fdt_resources(struct tegra_hda_sc *sc, phandle_t node)
{
	int rv;

	rv = hwreset_get_by_ofw_name(sc->dev, 0, "hda", &sc->hwreset_hda);
	if (rv != 0) {
		device_printf(sc->dev, "Cannot get 'hda' reset\n");
		return (ENXIO);
	}
	rv = hwreset_get_by_ofw_name(sc->dev, 0, "hda2hdmi",
	    &sc->hwreset_hda2hdmi);
	if (rv != 0) {
		device_printf(sc->dev, "Cannot get 'hda2hdmi' reset\n");
		return (ENXIO);
	}
	rv = hwreset_get_by_ofw_name(sc->dev, 0, "hda2codec_2x",
	    &sc->hwreset_hda2codec_2x);
	if (rv != 0) {
		device_printf(sc->dev, "Cannot get 'hda2codec_2x' reset\n");
		return (ENXIO);
	}

	rv = clk_get_by_ofw_name(sc->dev, 0, "hda", &sc->clk_hda);
	if (rv != 0) {
		device_printf(sc->dev, "Cannot get 'hda' clock\n");
		return (ENXIO);
	}
	rv = clk_get_by_ofw_name(sc->dev, 0, "hda2hdmi", &sc->clk_hda2hdmi);
	if (rv != 0) {
		device_printf(sc->dev, "Cannot get 'hda2hdmi' clock\n");
		return (ENXIO);
	}
	rv = clk_get_by_ofw_name(sc->dev, 0, "hda2codec_2x",
	     &sc->clk_hda2codec_2x);
	if (rv != 0) {
		device_printf(sc->dev, "Cannot get 'hda2codec_2x' clock\n");
		return (ENXIO);
	}
	return (0);
}

static int
enable_fdt_resources(struct tegra_hda_sc *sc)
{
	int rv;

	rv = hwreset_assert(sc->hwreset_hda);
	if (rv != 0) {
		device_printf(sc->dev, "Cannot assert 'hda' reset\n");
		return (rv);
	}
	rv = hwreset_assert(sc->hwreset_hda2codec_2x);
	if (rv != 0) {
		device_printf(sc->dev, "Cannot assert 'hda2codec_2x' reset\n");
		return (rv);
	}
	rv = hwreset_assert(sc->hwreset_hda2hdmi);
	if (rv != 0) {
		device_printf(sc->dev, "Cannot assert 'hda2hdmi' reset\n");
		return (rv);
	}

	rv = clk_enable(sc->clk_hda);
	if (rv != 0) {
		device_printf(sc->dev, "Cannot enable 'hda' clock\n");
		return (rv);
	}
	rv = clk_enable(sc->clk_hda2codec_2x);
	if (rv != 0) {
		device_printf(sc->dev, "Cannot enable 'hda2codec_2x' clock\n");
		return (rv);
	}

	rv = clk_enable(sc->clk_hda2hdmi);
	if (rv != 0) {
		device_printf(sc->dev, "Cannot enable 'hda2hdmi' clock\n");
		return (rv);
	}

	rv = hwreset_deassert(sc->hwreset_hda);
	if (rv != 0) {
		device_printf(sc->dev, "Cannot unreset 'hda' reset\n");
		return (rv);
	}
	rv = hwreset_deassert(sc->hwreset_hda2codec_2x);
	if (rv != 0) {
		device_printf(sc->dev, "Cannot unreset 'hda2codec_2x' reset\n");
		return (rv);
	}
	rv = hwreset_deassert(sc->hwreset_hda2hdmi);
	if (rv != 0) {
		device_printf(sc->dev, "Cannot unreset 'hda2hdmi' reset\n");
		return (rv);
	}

	return (0);
}

static int
tegra_hda_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_search_compatible(dev, compat_data)->ocd_data)
		return (ENXIO);

	device_set_desc_copy(dev, "Tegra HDA controller");
	return (BUS_PROBE_DEFAULT);
}

static int
tegra_hda_attach(device_t dev)
{
	struct tegra_hda_sc *sc;
	phandle_t node;
	int rid, rv;

	sc = device_get_softc(dev);
	sc->dev = dev;

	node = ofw_bus_get_node(sc->dev);
	LOCK_INIT(sc);

	rid = 0;
	sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->mem_res == NULL) {
		device_printf(dev, "Cannot allocate memory resources\n");
		goto fail;
	}

	rid = 0;
	sc->irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid, RF_ACTIVE);
	if (sc->irq_res == NULL) {
		device_printf(dev, "Cannot allocate IRQ resources\n");
		goto fail;
	}

	rv = get_fdt_resources(sc, node);
	if (rv != 0) {
		device_printf(dev, "Cannot parse FDT resources\n");
		goto fail;
	}
	rv = enable_fdt_resources(sc);
	if (rv != 0) {
		device_printf(dev, "Cannot enable FDT resources\n");
		goto fail;
	}

	tegra_hda_init(sc);

	sc->hdac_sc.mem.mem_rid = 0;
	sc->hdac_sc.mem.mem_res = NULL;
	sc->hdac_sc.mem.mem_tag = rman_get_bustag(sc->mem_res);
	rv = bus_space_subregion(rman_get_bustag(sc->mem_res),
	    rman_get_bushandle(sc->mem_res),
	    TEGRA_HDA_REG_OFF, TEGRA_HDA_REG_SIZE,
	    &sc->hdac_sc.mem.mem_handle);
	if (rv != 0) {
		device_printf(dev, "Could not create HDA memory subregion\n");
		goto fail;
	}

	WR4(sc, TEGRA_HDA_REG_OFF + 0x20, 0);
	rv = hdac_attach_subclass(sc->dev);
	if (rv != 0) {
		device_printf(dev, "Could not initialize HDA\n");
		goto fail;
	}

	return (0);

fail:
	if (sc->clk_hda != NULL)
		clk_release(sc->clk_hda);
	if (sc->clk_hda2hdmi != NULL)
		clk_release(sc->clk_hda2hdmi);
	if (sc->clk_hda2codec_2x != NULL)
		clk_release(sc->clk_hda2codec_2x);
	if (sc->hwreset_hda != NULL)
		hwreset_release(sc->hwreset_hda);
	if (sc->hwreset_hda2hdmi != NULL)
		hwreset_release(sc->hwreset_hda2hdmi);
	if (sc->hwreset_hda2codec_2x != NULL)
		hwreset_release(sc->hwreset_hda2codec_2x);
	if (sc->irq_res != NULL)
		bus_release_resource(dev, SYS_RES_IRQ, 0, sc->irq_res);
	if (sc->mem_res != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->mem_res);
	LOCK_DESTROY(sc);

	return (ENXIO);
}

static int
tegra_hda_detach(device_t dev)
{
	struct tegra_hda_sc *sc;

	sc = device_get_softc(dev);
	hdac_detach_subclass(dev);
	return (0);
}

static device_method_t tegra_hda_methods[] = {
	DEVMETHOD(device_probe,		tegra_hda_probe),
	DEVMETHOD(device_attach,	tegra_hda_attach),
	DEVMETHOD(device_detach,	tegra_hda_detach),
	DEVMETHOD(hdac_irq_setup,	tegra_hda_irq_setup),
	DEVMETHOD(hdac_irq_teardown,	tegra_hda_irq_teardown),
	DEVMETHOD(hdac_write_1,		tegra_hda_write_1),
	DEVMETHOD(hdac_write_2,		tegra_hda_write_2),
	DEVMETHOD(hdac_write_4,		tegra_hda_write_4),
	DEVMETHOD(hdac_read_1,		tegra_hda_read_1),
	DEVMETHOD(hdac_read_2,		tegra_hda_read_2),
	DEVMETHOD(hdac_read_4,		tegra_hda_read_4),
	DEVMETHOD_END
};
devclass_t tegra_hda_devclass;
DEFINE_CLASS_1(hdac, tegra_hdac_driver, tegra_hda_methods,
    sizeof(struct tegra_hda_sc), hdac_base_driver);
DRIVER_MODULE(tegra_hda, simplebus, tegra_hdac_driver, tegra_hda_devclass, NULL, NULL);
