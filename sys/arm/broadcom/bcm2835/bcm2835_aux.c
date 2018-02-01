/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2018 Michal Meloun <mmel@FreeBSD.org>
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

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/kobj.h>
#include <sys/module.h>
#include <sys/malloc.h>
#include <sys/rman.h>
#include <sys/lock.h>
#include <sys/mutex.h>

#include <machine/bus.h>
#include <machine/cpu.h>

#include <dev/extres/clk/clk_gate.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <gnu/dts/include/dt-bindings/clock/bcm2835-aux.h>

#include "clkdev_if.h"

#define BCM2835_AUX_IRQ		0
#define BCM2835_AUX_ENB		4

#define	RD4(sc, reg, val)	CLKDEV_READ_4((sc)->clkdev, reg, val)
#define	WR4(sc, reg, val)	CLKDEV_WRITE_4((sc)->clkdev, reg, val)
#define	MD4(sc, reg, m, s)	CLKDEV_MODIFY_4((sc)->clkdev, reg, m, s)
#define	DEVICE_LOCK(sc)		CLKDEV_DEVICE_LOCK((sc)->clkdev)
#define	DEVICE_UNLOCK(sc)	CLKDEV_DEVICE_UNLOCK((sc)->clkdev)

struct  bcm2835_aux_softc {
	device_t	dev;
	struct resource	*mem_res;
	struct mtx	mtx;
	clk_t		clk;
	const char 	*clk_name;
	struct clkdom	*clkdom;
};


/* Standard gate. */
#define	GATE(_id, cname, o, s)						\
{									\
	.clkdef.id = _id,						\
	.clkdef.name = cname,						\
	.clkdef.parent_cnt = 1,						\
	.clkdef.flags = CLK_NODE_STATIC_STRINGS,			\
	.offset = o,							\
	.shift = s,							\
	.mask = 1,							\
	.on_value = 1,							\
	.off_value = 0,							\
}

static struct clk_gate_def gate_clks[] = {
  GATE(BCM2835_AUX_CLOCK_UART, "aux_uart", BCM2835_AUX_ENB, 0),
  GATE(BCM2835_AUX_CLOCK_SPI1, "aux_spi1", BCM2835_AUX_ENB, 1),
  GATE(BCM2835_AUX_CLOCK_SPI2, "aux_spi2", BCM2835_AUX_ENB, 2),
};

static struct ofw_compat_data compat_data[] = {
	{"brcm,bcm2835-aux",		1},
	{"broadcom,bcm2835-aux",	1},
	{NULL,		 		0},
};



/* -----------------------   Device  --------------------------------------- */

static void
register_clocks(device_t dev)
{
	struct bcm2835_aux_softc *sc;
	int i, rv;

	sc = device_get_softc(dev);
	sc->clkdom = clkdom_create(dev);
	if (sc->clkdom == NULL)
		panic("clkdom == NULL");


	for (i = 0; i < nitems(gate_clks); i++) {
		gate_clks[i].clkdef.parent_names = &sc->clk_name;
		rv = clknode_gate_register(sc->clkdom, gate_clks + i);
		if (rv != 0)
			panic("periph_register failed");
	}

	clkdom_finit(sc->clkdom);
	if (bootverbose)
		clkdom_dump(sc->clkdom);
}

static int
bcm2835_aux_clkdev_read_4(device_t dev, bus_addr_t addr, uint32_t *val)
{
	struct bcm2835_aux_softc *sc;

	sc = device_get_softc(dev);
	*val = bus_read_4(sc->mem_res, addr);
	return (0);
}

static int
bcm2835_aux_clkdev_write_4(device_t dev, bus_addr_t addr, uint32_t val)
{
	struct bcm2835_aux_softc *sc;

	sc = device_get_softc(dev);
	bus_write_4(sc->mem_res, addr, val);
	return (0);
}

static int
bcm2835_aux_clkdev_modify_4(device_t dev, bus_addr_t addr,
    uint32_t clear_mask, uint32_t set_mask)
{
	struct bcm2835_aux_softc *sc;
	uint32_t reg;

	sc = device_get_softc(dev);
	reg = bus_read_4(sc->mem_res, addr);
	reg &= ~clear_mask;
	reg |= set_mask;
	bus_write_4(sc->mem_res, addr, reg);
	return (0);
}

static void
bcm2835_aux_clkdev_device_lock(device_t dev)
{
	struct bcm2835_aux_softc *sc;

	sc = device_get_softc(dev);
	mtx_lock(&sc->mtx);
}

static void
bcm2835_aux_clkdev_device_unlock(device_t dev)
{
	struct bcm2835_aux_softc *sc;

	sc = device_get_softc(dev);
	mtx_unlock(&sc->mtx);
}

static int
bcm2835_aux_detach(device_t dev)
{

	device_printf(dev, "Error: Clock driver cannot be detached\n");
	return (EBUSY);
}

static int
bcm2835_aux_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data != 0) {
		device_set_desc(dev, "BCM283x Aux");
		return (BUS_PROBE_DEFAULT);
	}

	return (ENXIO);
}

static int
bcm2835_aux_attach(device_t dev)
{
	struct bcm2835_aux_softc *sc;
	int rid, rv;

	sc = device_get_softc(dev);
	sc->dev = dev;

	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);

	/* Resource setup. */
	rid = 0;
	sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (!sc->mem_res) {
		device_printf(dev, "cannot allocate memory resource\n");
		rv = ENXIO;
		goto fail;
	}
	rv = clk_get_by_ofw_index(sc->dev, 0, 0, &sc->clk);
	if (rv != 0) {
		device_printf(sc->dev, "Cannot get 'parent' clock\n");
		rv = ENXIO;
		goto fail;
	}
	sc->clk_name = clk_get_name(sc->clk);
	register_clocks(dev);
	return (0);

fail:
	if (sc->mem_res)
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->mem_res);

	return (rv);
}

static device_method_t bcm2835_aux_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		bcm2835_aux_probe),
	DEVMETHOD(device_attach,	bcm2835_aux_attach),
	DEVMETHOD(device_detach,	bcm2835_aux_detach),

	/* Clkdev  interface*/
	DEVMETHOD(clkdev_read_4,	bcm2835_aux_clkdev_read_4),
	DEVMETHOD(clkdev_write_4,	bcm2835_aux_clkdev_write_4),
	DEVMETHOD(clkdev_modify_4,	bcm2835_aux_clkdev_modify_4),
	DEVMETHOD(clkdev_device_lock,	bcm2835_aux_clkdev_device_lock),
	DEVMETHOD(clkdev_device_unlock,	bcm2835_aux_clkdev_device_unlock),

	DEVMETHOD_END
};

static devclass_t bcm2835_aux_devclass;
static DEFINE_CLASS_0(bcm2835_aux, bcm2835_aux_driver, bcm2835_aux_methods,
    sizeof(struct bcm2835_aux_softc));
EARLY_DRIVER_MODULE(bcm2835_aux, simplebus, bcm2835_aux_driver,
    bcm2835_aux_devclass, NULL, NULL, BUS_PASS_INTERRUPT + BUS_PASS_ORDER_LATE);
