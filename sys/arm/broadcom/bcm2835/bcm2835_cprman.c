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

#include <dev/extres/clk/clk_div.h>
#include <dev/extres/clk/clk_fixed.h>
#include <dev/extres/clk/clk_gate.h>
#include <dev/extres/clk/clk_mux.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <gnu/dts/include/dt-bindings/clock/bcm2835.h>

#include "clkdev_if.h"

#define DPRINTF(...)	printf(__VA_ARGS__)

#define CM_GNRICCTL		0x0000
/* For all CTL registers */
#define  CM_CTL_FRAC			(1 << 9)
#define  CM_CTL_FLIP			(1 << 8)
#define  CM_CTL_BUSY			(1 << 7)
#define  CM_CTL_GATE			(1 << 6)
#define  CM_CTL_KILL			(1 << 5)
#define  CM_CTL_ENAB			(1 << 4)
#define  CM_CTL_MUX(x)			(((x) & 0x0F)  << 0)
#define  CM_CTL_MUX_GET(x)		(((x) >> 0)  & 0x0F)

#define CM_GNRICDIV		0x0004
#define CM_VPUCTL		0x0008
#define CM_VPUDIV		0x000C
#define CM_SYSCTL		0x0010
#define CM_SYSDIV		0x0014
#define CM_PERIACTL		0x0018
#define CM_PERIADIV		0x001C
#define CM_PERIICTL		0x0020
#define CM_PERIIDIV		0x0024
#define CM_H264CTL		0x0028
#define CM_H264DIV		0x002C
#define CM_ISPCTL		0x0030
#define CM_ISPDIV		0x0034
#define CM_V3DCTL		0x0038
#define CM_V3DDIV		0x003C
#define CM_CAM0CTL		0x0040
#define CM_CAM0DIV		0x0044
#define CM_CAM1CTL		0x0048
#define CM_CAM1DIV		0x004C
#define CM_CCP2CTL		0x0050
#define CM_CCP2DIV		0x0054
#define CM_DSI0ECTL		0x0058
#define CM_DSI0EDIV		0x005C
#define CM_DSI0PCTL		0x0060
#define CM_DSI0PDIV		0x0064
#define CM_DPICTL		0x0068
#define CM_DPIDIV		0x006C
#define CM_GP0CTL		0x0070
#define CM_GP0DIV		0x0074
#define CM_GP1CTL		0x0078
#define CM_GP1DIV		0x007C
#define CM_GP2CTL		0x0080
#define CM_GP2DIV		0x0084
#define CM_HSMCTL		0x0088
#define CM_HSMDIV		0x008C
#define CM_OTPCTL		0x0090
#define CM_OTPDIV		0x0094
#define CM_PCMCTL		0x0098
#define CM_PCMDIV		0x009C
#define CM_PWMCTL		0x00A0
#define CM_PWMDIV		0x00A4
#define CM_SLIMCTL		0x00A8
#define CM_SLIMDIV		0x00AC
#define CM_SMICTL		0x00B0
#define CM_SMIDIV		0x00B4
#define CM_TCNTCTL		0x00C0
#define CM_TCNTCNT		0x00C4
#define CM_TECCTL		0x00C8
#define CM_TECDIV		0x00CC
#define CM_TD0CTL		0x00D0
#define CM_TD0DIV		0x00D4
#define CM_TD1CTL		0x00D8
#define CM_TD1DIV		0x00DC
#define CM_TSENSCTL		0x00E0
#define CM_TSENSDIV		0x00E4
#define CM_TIMERCTL		0x00E8
#define CM_TIMERDIV		0x00EC
#define CM_UARTCTL		0x00F0
#define CM_UARTDIV		0x00F4
#define CM_VECCTL		0x00F8
#define CM_VECDIV		0x00FC
#define CM_OSCCOUNT		0x0100
#define CM_PLLA			0x0104
/* For all CM_PLLx registers */
#define  CM_PLL_ANARST			(1 << 8)

#define CM_PLLC			0x0108
#define CM_PLLD			0x010C
#define CM_PLLH			0x0110
#define CM_LOCK			0x0114
#define CM_EVENT		0x0118
#define CM_DSI1ECTL		0x0158
#define CM_DSI1EDIV		0x015C
#define CM_DSI1PCTL		0x0160
#define CM_DSI1PDIV		0x0164
#define CM_DFTCTL		0x0168
#define CM_DFTDIV		0x016C
#define CM_PLLB			0x0170
#define CM_PULSECTL		0x0190
#define CM_PULSEDIV		0x0194
#define CM_SDCCTL		0x01A8
#define CM_SDCDIV		0x01AC
#define CM_ARMCTL		0x01B0
#define CM_AVEOCTL		0x01B8
#define CM_AVEODIV		0x01BC
#define CM_EMMCCTL		0x01C0
#define CM_EMMCDIV		0x01C4
#define A2W_PLLA_CTRL		0x1100
/* For all A2W_PLLx_CTRL registers */
#define  A2W_PLL_CTRL_PRST_DIS		(1 << 17)
#define  A2W_PLL_CTRL_PWRDN		(1 << 16)
#define  A2W_PLL_CTRL_PDIV(x)		(((x) & 0x7) << 12)
#define  A2W_PLL_CTRL_PDIV_GET(x)	(((x) >> 12) & 0x7)
#define  A2W_PLL_CTRL_NDIV(x)		(((x) & 0x3FF) << 0)
#define  A2W_PLL_CTRL_NDIV_GET(x)	(((x) >> 0) & 0x3FF)

#define A2W_PLLC_CTRL		0x1120
#define A2W_PLLD_CTRL		0x1140
#define A2W_PLLH_CTRL		0x1160
#define A2W_PLLB_CTRL		0x11E0
#define A2W_PLLA_ANA0		0x1010
/* All A2W_PLLx_ANA but A2W_PLLH_ANA */
#define A2W_PLL_ANA1_KI(x)	 	(((x) & 0x07) << 19)
#define A2W_PLL_ANA1_KP(x)	 	(((x) & 0x0F) << 15)
#define A2W_PLL_ANA1_FB_PREDIV	 	(1 << 14)
#define A2W_PLL_ANA3_KA(x)	 	(((x) & 0x07) <<  7)

#define A2W_PLLC_ANA0		0x1030
#define A2W_PLLD_ANA0		0x1050
#define A2W_PLLH_ANA0		0x1070

#define A2W_PLLH_ANA0_KI_LOW(x)	 	(((x) & 0x03) << 22)
#define A2W_PLLH_ANA0_KA(x)	 	(((x) & 0x07) << 19)
#define A2W_PLLH_ANA1_FB_PREDIV	 	(1 << 11)
#define A2W_PLLH_ANA1_KP(x)	 	(((x) & 0x0F) << 1)
#define A2W_PLLH_ANA1_KI_HIGH(x)	(((x) & 0x01) << 0)

#define A2W_PLLB_ANA0		0x10F0
#define A2W_XOSC_CTRL		0x1190
#define A2W_PLLA_FRAC		0x1200
/* For all A2W_PLLx_FRAC registers. */
#define  A2W_PLL_FRAC_BITS		20
#define  A2W_PLL_FRAC_FDIV(x)		(((x) & 0xFFFFF) << 0)
#define  A2W_PLL_FRAC_FDIV_GET(x)	(((x) >> 0) & 0xFFFFF)

#define A2W_PLLC_FRAC		0x1220
#define A2W_PLLD_FRAC		0x1240
#define A2W_PLLH_FRAC		0x1260
#define A2W_PLLB_FRAC		0x12E0

#define A2W_PLLA_DSI0		0x1300
/* For all A2W_PLLx_ registers, but CTRL, ANA, FRAC */
#define  A2W_PLLDIV_DISABLE		(1 < 8)
#define  A2W_PLLDIV_DIV(x)		(((x) >> 0) & 0x0F)
#define  A2W_PLLDIV_DIV_GET(x)		(((x) & 0x0F) >> 0)

#define A2W_PLLC_CORE2		0x1320
#define A2W_PLLD_DSI0		0x1340
#define A2W_PLLH_AUX		0x1360
#define A2W_PLLB_ARM		0x13E0
#define A2W_PLLA_CORE		0x1400
#define A2W_PLLC_CORE1		0x1420
#define A2W_PLLD_CORE		0x1440
#define A2W_PLLH_RCAL		0x1460
#define A2W_PLLB_SP0		0x14E0
#define A2W_PLLA_PER		0x1500
#define A2W_PLLC_PER		0x1520
#define A2W_PLLD_PER		0x1540
#define A2W_PLLH_PIX		0x1560
#define A2W_PLLB_SP1		0x15E0
#define A2W_PLLA_CCP2		0x1600
#define A2W_PLLC_CORE0		0x1620
#define A2W_PLLD_DSI1		0x1640
#define A2W_PLLH_STS		0x1660
#define A2W_PLLB_SP2		0x16E0
#define A2W_PLLH_CTRLR		0x1960
#define A2W_PLLH_FRACR		0x1A60
#define A2W_PLLH_AUXR		0x1B60
#define A2W_PLLH_RCALR		0x1C60
#define A2W_PLLH_PIXR		0x1D60
#define A2W_PLLH_STSR		0x1E60


#define PLL_PFD_MAX		1750000000ULL
#define PLL_VCO_MIN		 600000000ULL

#define	RD4(sc, reg, val)	CLKDEV_READ_4((sc)->clkdev, reg, val)
#define	WR4(sc, reg, val)	CLKDEV_WRITE_4((sc)->clkdev, reg, val)
#define	MD4(sc, reg, m, s)	CLKDEV_MODIFY_4((sc)->clkdev, reg, m, s)
#define	DEVICE_LOCK(sc)		CLKDEV_DEVICE_LOCK((sc)->clkdev)
#define	DEVICE_UNLOCK(sc)	CLKDEV_DEVICE_UNLOCK((sc)->clkdev)

struct  bcm2835_cprman_softc {
	device_t	dev;
	struct resource	*mem_res;
	struct mtx	mtx;
	struct clkdom	*clkdom;
};

/* Fixed rate clock. */
#define	FRATE(_id, _name, _freq)					\
{									\
	.clkdef.id = _id,						\
	.clkdef.name = _name,						\
	.clkdef.parent_names = NULL,					\
	.clkdef.parent_cnt = 0,						\
	.clkdef.flags = CLK_NODE_STATIC_STRINGS,			\
	.freq = _freq,							\
}

/* Fixed rate multipier/divider. */
#define	FACT(_id, _name, _pname, _mult, _div)				\
{									\
	.clkdef.id = _id,						\
	.clkdef.name = _name,						\
	.clkdef.parent_names = (const char *[]){_pname},		\
	.clkdef.parent_cnt = 1,						\
	.clkdef.flags = CLK_NODE_STATIC_STRINGS,			\
	.mult = _mult,							\
	.div = _div,							\
}

struct clk_pll_def {
	struct clknode_init_def clkdef;
	uint32_t		cm_reg;
	uint32_t		a2w_reg;
	uint32_t		frac_reg;
	uint32_t		ana_reg;
	uint32_t		ena_mask;
	uint32_t		lock_mask;
	uint64_t		vco_max;
};

/* PLL definition */
#define	PLL(_id, _name, _cm, _aw, _fr, _an, _en, _lc, _vmax)	\
{									\
	.clkdef.id = _id,						\
	.clkdef.name = _name,						\
	.clkdef.parent_names = (const char *[]){"xosc"},		\
	.clkdef.parent_cnt = 1,						\
	.clkdef.flags = CLK_NODE_STATIC_STRINGS,			\
	.cm_reg = _cm,							\
	.a2w_reg = _aw,							\
	.frac_reg = _fr,						\
	.ana_reg = _an,							\
	.ena_mask = (1U << _en),					\
	.lock_mask = (1U << _lc),					\
	.vco_max = _vmax,						\
}

struct clk_plldiv_def {
	struct clknode_init_def clkdef;
	uint32_t		cm_reg;
	uint32_t		a2w_reg;
	int			load_bit;
	int			hold_bit;
	int			divider;
};

#define	PLLDIV(_id, _name, _pname, _cm, _aw, _lb, _hb)			\
{									\
	.clkdef.id = _id,						\
	.clkdef.name = _name,						\
	.clkdef.parent_names = (const char *[]){_pname},		\
	.clkdef.parent_cnt = 1,						\
	.clkdef.flags = CLK_NODE_STATIC_STRINGS,			\
	.cm_reg = _cm,							\
	.a2w_reg = _aw,							\
	.load_bit = _lb,						\
	.hold_bit = _hb,						\
}

#define CLK_FLAG_MASH		0x0001		/* Have MASH filter block. */
#define CLK_FLAG_NOFRAC		0x0002		/* Low jitter - don't use  */
						/*  fractional divider */
struct clk_periph_def {
	struct clknode_init_def clkdef;
	uint32_t		flags;
	uint32_t		ctrl_reg;
	uint32_t		div_reg;
	uint32_t		div_i_width;
	uint32_t		div_i_mask;
	uint32_t		div_f_width;
	uint32_t		div_f_mask;
};

#define	PER(_id, _name, _pl, _ct, _dv, _diw, _dfw, _flg)		\
{									\
	.clkdef.id = _id,						\
	.clkdef.name = _name,						\
	.clkdef.parent_names = _pl,					\
	.clkdef.parent_cnt = nitems(_pl),				\
	.clkdef.flags = CLK_NODE_STATIC_STRINGS,			\
	.ctrl_reg = _ct,						\
	.div_reg = _dv,							\
	.div_i_width = _diw,						\
	.div_f_width = _dfw,	 					\
	.flags = _flg,		 					\
}

/* Standard gate. */
#define	GATE(_id, cname, plist, o, s)					\
{									\
	.clkdef.id = _id,						\
	.clkdef.name = cname,						\
	.clkdef.parent_names = (const char *[]){plist},			\
	.clkdef.parent_cnt = 1,						\
	.clkdef.flags = CLK_NODE_STATIC_STRINGS,			\
	.offset = o,							\
	.shift = s,							\
	.mask = 1,							\
	.on_value = 1,							\
	.off_value = 0,							\
}


#define	PLIST(x) static const char *x[]
/* Parent lists. */
PLIST(mux_per_srcs) = {
	"gnd", "xosc", "testdebug0", "testdebug1",
	"plla_per", "pllc_per", "plld_per", "pllh_aux",
};

PLIST(mux_vpu_srcs) = {
	"gnd", "xosc", "testdebug0", "testdebug1",
	"plla_core", "pllc_core0", "plld_core", "pllh_aux",
	"pllc_core1", "pllc_core2",
};

PLIST(mux_pcm_srcs) = {
	"gnd", "xosc", "gnd", "gnd",
	"gnd", "gnd", "plld_core",
};

PLIST(mux_osc_srcs) = {
	"gnd", "xosc", "testdebug0", "testdebug1",
};

PLIST(mux_ds0_srcs) = {
	"gnd", 	"xosc", "testdebug0", "testdebug1",
#if 0
	"dsi0_ddr", "dsi0_ddr_inv", "dsi0_ddr2", "dsi0_ddr2_inv",
	"dsi0_byte", "dsi0_byte_inv",
#else
	"gnd", "gnd", "gnd", "gnd",
	"gnd", "gnd",
#endif
};

PLIST(mux_ds1_srcs) = {
	"gnd", "xosc", "testdebug0", "testdebug1",
#if 0
	"dsi1_ddr", "dsi1_ddr_inv", "dsi1_ddr2", "dsi1_ddr2_inv",
	"dsi1_byte", "dsi1_byte_inv",
#else
	"gnd", "gnd", "gnd", "gnd",
	"gnd", "gnd",
#endif
};

/* Input clocks - its renamef in runtime */
static struct clk_fixed_def xosc_clk =
	FACT(0, "xosc", "osc", 1, 1);

/* Fixed rate(or not yet implemented) or fixed divider clocks. */
static struct clk_fixed_def bmc2835_fixed_clks[] = {
	FRATE(0, "gnd", 0),
	FRATE(0, "testdebug0", 0),
	FRATE(0, "testdebug1", 0),
	FACT(BCM2835_PLLH_RCAL, "pllh_rcal", "pllh_rcal_p", 1, 10),
	FACT(BCM2835_PLLH_AUX,  "pllh_aux",  "pllh_aux_p",  1, 10),
	FACT(BCM2835_PLLH_PIX,  "pllh_pix",  "pllh_pix_p",  1, 10),
};

/* PLLs. */
static struct clk_pll_def bmc2835_pll_clks[] = {
  PLL(BCM2835_PLLA, "plla", CM_PLLA, A2W_PLLA_CTRL, A2W_PLLA_FRAC, A2W_PLLA_ANA0,
    6,  8, 2400000000ULL),
  PLL(BCM2835_PLLB, "pllb", CM_PLLB, A2W_PLLB_CTRL, A2W_PLLB_FRAC, A2W_PLLB_ANA0,
    7,  9, 3000000000ULL),
  PLL(BCM2835_PLLC, "pllc", CM_PLLC, A2W_PLLC_CTRL, A2W_PLLC_FRAC, A2W_PLLC_ANA0,
    0, 10, 3000000000ULL),
  PLL(BCM2835_PLLD, "plld", CM_PLLD, A2W_PLLD_CTRL, A2W_PLLD_FRAC, A2W_PLLD_ANA0,
    4, 11, 2400000000ULL),
  PLL(BCM2835_PLLH, "pllh", CM_PLLH, A2W_PLLH_CTRL, A2W_PLLH_FRAC, A2W_PLLH_ANA0,
    0, 12, 3000000000ULL),  /* XXX Enable bit is incorrect */
};

/* PLL branch postdividers. */
static struct clk_plldiv_def bmc2835_plldiv_clks[] = {
  PLLDIV(BCM2835_PLLA_CORE,  "plla_core",  "plla", CM_PLLA, A2W_PLLA_CORE,  4,  5),
  PLLDIV(BCM2835_PLLA_PER,   "plla_per",   "plla", CM_PLLA, A2W_PLLA_PER,   6,  7),
  PLLDIV(BCM2835_PLLB_ARM,   "pllb_arm",   "pllb", CM_PLLB, A2W_PLLB_ARM,   0,  1),
  PLLDIV(BCM2835_PLLC_CORE0, "pllc_core0", "pllc", CM_PLLC, A2W_PLLC_CORE0, 0,  1),
  PLLDIV(BCM2835_PLLC_CORE1, "pllc_core1", "pllc", CM_PLLC, A2W_PLLC_CORE1, 2,  3),
  PLLDIV(BCM2835_PLLC_CORE2, "pllc_core2", "pllc", CM_PLLC, A2W_PLLC_CORE2, 4,  5),
  PLLDIV(BCM2835_PLLC_PER,   "pllc_per",   "pllc", CM_PLLC, A2W_PLLC_PER,   6,  7),
  PLLDIV(BCM2835_PLLD_CORE,  "plld_core",  "plld", CM_PLLD, A2W_PLLD_CORE,  4,  5),
  PLLDIV(BCM2835_PLLD_PER,   "plld_per",   "plld", CM_PLLD, A2W_PLLD_PER,   6,  7),
  PLLDIV(0,                  "pllh_rcal_p","pllh", CM_PLLH, A2W_PLLH_RCAL,  2, -1),
  PLLDIV(0,                  "pllh_aux_p", "pllh", CM_PLLH, A2W_PLLH_AUX,   1, -1),
  PLLDIV(0,                  "pllh_pix_p", "pllh", CM_PLLH, A2W_PLLH_PIX,   0, -1),
};

/* Peripheral blocks. */
static struct clk_periph_def bmc2835_periph_clks[] = {
  PER(BCM2835_CLOCK_TIMER, "timer", mux_osc_srcs, CM_TIMERCTL, CM_TIMERDIV,  6, 12, 0),
  PER(BCM2835_CLOCK_OTP,   "otp",   mux_osc_srcs, CM_OTPCTL,   CM_OTPDIV,    4,  0, 0),
  PER(BCM2835_CLOCK_TSENS, "tsens", mux_osc_srcs, CM_TSENSCTL, CM_TSENSDIV,  5,  0, 0),
  PER(BCM2835_CLOCK_VPU,   "vpu",   mux_vpu_srcs, CM_VPUCTL,   CM_VPUDIV,   12,  8, 0),
  PER(BCM2835_CLOCK_V3D,   "v3d",   mux_vpu_srcs, CM_V3DCTL,   CM_V3DDIV,    4,  8, 0),
  PER(BCM2835_CLOCK_ISP,   "isp",   mux_vpu_srcs, CM_ISPCTL,   CM_ISPDIV,    4,  8, 0),
  PER(BCM2835_CLOCK_H264,  "h264",  mux_vpu_srcs, CM_H264CTL,  CM_H264DIV,   4,  8, 0),
  PER(BCM2835_CLOCK_SDRAM, "sdram", mux_vpu_srcs, CM_SDCCTL,   CM_SDCDIV,    6,  0, 0),
  PER(BCM2835_CLOCK_PCM,   "pcm",   mux_pcm_srcs, CM_PCMCTL,   CM_PCMDIV,   12, 12, CLK_FLAG_MASH | CLK_FLAG_NOFRAC),
  PER(BCM2835_CLOCK_AVEO,  "aveo",  mux_per_srcs, CM_AVEOCTL,  CM_AVEODIV,   4,  0, 0),
  PER(BCM2835_CLOCK_CAM0,  "cam0",  mux_per_srcs, CM_CAM0CTL,  CM_CAM0DIV,   4,  8, 0),
  PER(BCM2835_CLOCK_CAM1,  "cam1",  mux_per_srcs, CM_CAM1CTL,  CM_CAM1DIV,   4,  8, 0),
  PER(BCM2835_CLOCK_DFT,   "dft",   mux_per_srcs, CM_DFTCTL,   CM_DFTDIV,    4,  8, 0),
  PER(BCM2835_CLOCK_DPI,   "dpi",   mux_per_srcs, CM_DPICTL,   CM_DPIDIV,    4,  8, 0),
  PER(BCM2835_CLOCK_VEC,   "vec",   mux_per_srcs, CM_VECCTL,   CM_VECDIV,    4,  0, 0),
  PER(BCM2835_CLOCK_GP0,   "gp0",   mux_per_srcs, CM_GP0CTL,   CM_GP0DIV,   12, 12, CLK_FLAG_MASH),
  PER(BCM2835_CLOCK_GP1,   "gp1",   mux_per_srcs, CM_GP1CTL,   CM_GP1DIV,   12, 12, CLK_FLAG_MASH),
  PER(BCM2835_CLOCK_GP2,   "gp2",   mux_per_srcs, CM_GP2CTL,   CM_GP2DIV,   12, 12, 0),
  PER(BCM2835_CLOCK_PWM,   "pwm",   mux_per_srcs, CM_PWMCTL,   CM_PWMDIV,   12, 12, CLK_FLAG_MASH),
  PER(BCM2835_CLOCK_SLIM,  "slim",  mux_per_srcs, CM_SLIMCTL,  CM_SLIMDIV,  12, 12, CLK_FLAG_MASH),
  PER(BCM2835_CLOCK_SMI,   "smi",   mux_per_srcs, CM_SMICTL,   CM_SMIDIV,    4,  8, 0),
  PER(BCM2835_CLOCK_UART,  "uart",  mux_per_srcs, CM_UARTCTL,  CM_UARTDIV,  10, 12, 0),
  PER(BCM2835_CLOCK_HSM,   "hsm",   mux_per_srcs, CM_HSMCTL,   CM_HSMDIV,    4,  8, 0),
  PER(BCM2835_CLOCK_EMMC,  "emmc",  mux_per_srcs, CM_EMMCCTL,  CM_EMMCDIV,   4,  8, 0),
  PER(BCM2835_CLOCK_DSI0E, "dsi0e", mux_per_srcs, CM_DSI0ECTL, CM_DSI0EDIV,  4,  8, 0),
  PER(BCM2835_CLOCK_DSI1E, "dsi1e", mux_per_srcs, CM_DSI1ECTL, CM_DSI1EDIV,  4,  8, 0),
  PER(BCM2835_CLOCK_DSI0P, "dsi0p", mux_ds0_srcs, CM_DSI0PCTL, CM_DSI0PDIV,  0,  0, 0),
  PER(BCM2835_CLOCK_DSI1P, "dsi1p", mux_ds1_srcs, CM_DSI1PCTL, CM_DSI1PDIV,  0,  0, 0),
};

static struct clk_gate_def bmc2835_gate_clks[] = {
  GATE(BCM2835_CLOCK_PERI_IMAGE, "peri_image", "vpu", CM_PERIICTL, 6),
};

static struct ofw_compat_data compat_data[] = {
	{"brcm,bcm2835-cprman",		1},
	{"broadcom,bcm2835-cprman",	1},
	{NULL,		 		0},
};

/* ---------------------   PLL class   ------------------------------------- */

static int pll_init(struct clknode *clk, device_t dev);
static int pll_set_gate(struct clknode *clk, bool enable);
static int pll_recalc(struct clknode *clk, uint64_t *freq);
static int pll_set_freq(struct clknode *clknode, uint64_t fin,
    uint64_t *fout, int flags, int *stop);

struct pll_sc {
	device_t		clkdev;
	uintptr_t		id;
	uint32_t		cm_reg;
	uint32_t		a2w_reg;
	uint32_t		frac_reg;
	uint32_t		ana_reg;
	uint32_t		ena_mask;
	uint32_t		lock_mask;
	uint64_t		vco_max;
};

static clknode_method_t pll_methods[] = {
	/* Device interface */
	CLKNODEMETHOD(clknode_init,		pll_init),
	CLKNODEMETHOD(clknode_set_gate,		pll_set_gate),
	CLKNODEMETHOD(clknode_recalc_freq,	pll_recalc),
	CLKNODEMETHOD(clknode_set_freq,		pll_set_freq),
	CLKNODEMETHOD_END
};
DEFINE_CLASS_1(bcm2835_pll, pll_class, pll_methods,
   sizeof(struct pll_sc), clknode_class);

static int
pll_wait_for_lock(struct pll_sc *sc, struct clknode *clk)
{
	int i;
	uint32_t reg;

	for (i = 100000; i > 0; i--) {
		RD4(sc, CM_LOCK, &reg);
		if ((reg & sc->lock_mask) != 0)
			break;
		DELAY(10);
	}
	if (i <= 0) {
		printf("%s: %s - Timedout when waiting for PLL lock.\n",
		    __func__, clknode_get_name(clk));
		return (ETIMEDOUT);
	}
	return (0);
}

static uint64_t
pll_get_freq(uint64_t fin, uint32_t ndiv, uint32_t fdiv, uint32_t pdiv)
{
	uint64_t fout;
	uint64_t tmp;

	tmp = (ndiv << A2W_PLL_FRAC_BITS) + fdiv;
	fout   = fin * tmp;
	fout  /= pdiv;
	fout >>= A2W_PLL_FRAC_BITS;
	return (fout);
}

static void
pll_program_ana(struct pll_sc *sc, bool set_fbdiv, uint32_t pmask)
{
	uint32_t ana[4];

	RD4(sc, sc->ana_reg + (3 * 4), ana + 3);
	RD4(sc, sc->ana_reg + (2 * 4), ana + 2);
	RD4(sc, sc->ana_reg + (1 * 4), ana + 1);
	RD4(sc, sc->ana_reg + (0 * 4), ana + 0);

	if (set_fbdiv)
		ana[1] |= pmask;
	else
		ana[1] &= pmask;

	if (sc->id == BCM2835_PLLH) {
		ana[0] &= ~A2W_PLLH_ANA0_KA(~0);
		ana[0] |=  A2W_PLLH_ANA0_KA(2);
		ana[0] &= ~A2W_PLLH_ANA0_KI_LOW(~0);
		ana[0] |=  A2W_PLLH_ANA0_KI_LOW(2);

		ana[1] &= ~A2W_PLLH_ANA1_KI_HIGH(~0);
		ana[1] |=  A2W_PLLH_ANA1_KI_HIGH(0);
		ana[1] &= ~A2W_PLLH_ANA1_KP(~0);
		ana[1] |=  A2W_PLLH_ANA1_KP(6);
	} else {
		ana[1] &= ~A2W_PLL_ANA1_KI(~0);
		ana[1] |=  A2W_PLL_ANA1_KI(2);
		ana[1] &= ~A2W_PLL_ANA1_KP(~0);
		ana[1] |=  A2W_PLL_ANA1_KP(8);

		ana[3] &= ~A2W_PLL_ANA3_KA(~0);
		ana[3] |=  A2W_PLL_ANA3_KA(2);
	}

	WR4(sc, sc->ana_reg + (3 * 4), ana[3]);
	WR4(sc, sc->ana_reg + (2 * 4), ana[2]);
	WR4(sc, sc->ana_reg + (1 * 4), ana[1]);
	WR4(sc, sc->ana_reg + (0 * 4), ana[0]);
}

static int
pll_init(struct clknode *clk, device_t dev)
{

	clknode_init_parent_idx(clk, 0);
	return(0);
}

static int
pll_set_gate(struct clknode *clk, bool enable)
{
	int rv;
	struct pll_sc *sc;

	sc = clknode_get_softc(clk);
	rv = 0;

	DEVICE_LOCK(sc);
	if (enable) {
		MD4(sc, sc->a2w_reg, A2W_PLL_CTRL_PWRDN, 0);
		MD4(sc, sc->cm_reg, CM_PLL_ANARST, 0);
		rv = pll_wait_for_lock(sc, clk);
	} else {
		MD4(sc, sc->cm_reg, 0, CM_PLL_ANARST);
		MD4(sc, sc->a2w_reg, 0, A2W_PLL_CTRL_PWRDN);
	}
	DEVICE_UNLOCK(sc);
	return (rv);
}

static int
pll_recalc(struct clknode *clk, uint64_t *freq)
{
	struct pll_sc *sc;
	uint32_t ndiv, pdiv, fdiv, pmask;
	uint32_t lock_reg, frac_reg, a2w_reg, ana_reg;

	sc = clknode_get_softc(clk);

	DEVICE_LOCK(sc);
	RD4(sc, CM_LOCK, &lock_reg);
	RD4(sc, sc->frac_reg, &frac_reg);
	RD4(sc, sc->a2w_reg, &a2w_reg);
	RD4(sc, sc->ana_reg + (1 * 4), &ana_reg);
	DEVICE_UNLOCK(sc);

	ndiv = A2W_PLL_CTRL_NDIV_GET(a2w_reg);
	pdiv = A2W_PLL_CTRL_PDIV_GET(a2w_reg);
	fdiv = A2W_PLL_FRAC_FDIV_GET(frac_reg);

	/* Take a FB predivider into account. */
	pmask =  (sc->id == BCM2835_PLLH) ?
	    A2W_PLLH_ANA1_FB_PREDIV : A2W_PLL_ANA1_FB_PREDIV;
	if (ana_reg & pmask)
		*freq /= 2;

	DPRINTF("%s: %s  - n: %d, p: %d, f: %d - %s\n",
	    __func__, clknode_get_name(clk), ndiv, pdiv, fdiv,
	    (lock_reg & sc->lock_mask) ? "locked" : "unlocked");

	if (pdiv == 0) {
		*freq = 0;
		return (EINVAL);
	}

	*freq = pll_get_freq(*freq, ndiv, fdiv, pdiv);
	return (0);
}

static int
pll_set_freq(struct clknode *clk, uint64_t fin, uint64_t *fout, int flags,
    int *stop)
{

	struct pll_sc *sc;
	uint32_t ndiv, pdiv, fdiv, pmask;
	uint32_t reg, ana;
	uint64_t _fout, div;
	bool set_fbdiv, used_fbdiv, delay_ana;

	sc = clknode_get_softc(clk);

	if (*fout < PLL_VCO_MIN || *fout > sc->vco_max) {
		printf("%s: %s - requested PLL frequency out of range.\n",
		    __func__, clknode_get_name(clk));
		return (ERANGE);
	}

	/* Compute dividers. */
	set_fbdiv = false;
	if (*fout > PLL_PFD_MAX) {
		set_fbdiv = true;
		fin /= 2;
	}
	div = *fout << A2W_PLL_FRAC_BITS;
	div /= fin;
	ndiv = div >> A2W_PLL_FRAC_BITS;
	fdiv = div & ((1 << A2W_PLL_FRAC_BITS) - 1);
	pdiv = 1;

	*stop = 1;
	_fout = pll_get_freq(fin, ndiv, fdiv, pdiv);
	if (flags & CLK_SET_DRYRUN) {
		if (((flags & (CLK_SET_ROUND_UP | CLK_SET_ROUND_DOWN)) == 0) &&
		    (*fout != _fout))
			return (ERANGE);

		*fout = _fout;
		return (0);
	}

	DEVICE_LOCK(sc);

	/* Redback current feedback predivider. */
	RD4(sc, sc->ana_reg + (1 * 4), &ana);
	pmask =  (sc->id == BCM2835_PLLH) ?
	    A2W_PLLH_ANA1_FB_PREDIV : A2W_PLL_ANA1_FB_PREDIV;
	used_fbdiv =  (ana & pmask) ? true: false;

	/*
	 * If feedback predivider was not used in previous setup but we use it
	 * now, we must delay ANA registers setup after divider registers
	 * are programmed. Otherwise we can produce doubled frequency until
	 * dividers are programmed.
	 */
	delay_ana = (!used_fbdiv && set_fbdiv) ? true: false;

	/* Enable reference clock. */
	MD4(sc, A2W_XOSC_CTRL, 0, sc->ena_mask);

	if (!delay_ana)
		pll_program_ana(sc, set_fbdiv, pmask);

	/* Fractional divider first */
	MD4(sc, sc->frac_reg, A2W_PLL_FRAC_FDIV(~0), A2W_PLL_FRAC_FDIV(fdiv));

	/* Then ndiv and pdiv in single write. */
	RD4(sc, sc->a2w_reg, &reg);
	reg &= ~A2W_PLL_CTRL_PDIV(~0);
	reg |= A2W_PLL_CTRL_PDIV(pdiv);
	reg &= ~A2W_PLL_CTRL_NDIV(~0);
	reg |= A2W_PLL_CTRL_NDIV(ndiv);
	WR4(sc, sc->a2w_reg, reg);

	if (delay_ana)
		pll_program_ana(sc, set_fbdiv, pmask);

	DEVICE_UNLOCK(sc);
	*fout = _fout;

	return (0);
}

static int
pll_register(struct clkdom *clkdom, struct clk_pll_def *clkdef)
{
	struct clknode *clk;
	struct pll_sc *sc;

	clk = clknode_create(clkdom, &pll_class, &clkdef->clkdef);
	if (clk == NULL)
		return (ENXIO);

	sc = clknode_get_softc(clk);
	sc->clkdev = clknode_get_device(clk);
	sc->id = clkdef->clkdef.id;
	sc->cm_reg = clkdef->cm_reg;
	sc->a2w_reg = clkdef->a2w_reg;
	sc->frac_reg = clkdef->frac_reg;
	sc->ana_reg = clkdef->ana_reg;
	sc->ena_mask = clkdef->ena_mask;
	sc->lock_mask = clkdef->lock_mask;
	sc->vco_max = clkdef->vco_max;
	clknode_register(clkdom, clk);
	return (0);
}

/* -----------------   PLL divider class   --------------------------------- */

static int plldiv_init(struct clknode *clk, device_t dev);
static int plldiv_set_gate(struct clknode *clk, bool enable);
static int plldiv_recalc(struct clknode *clk, uint64_t *freq);
static int plldiv_set_freq(struct clknode *clknode, uint64_t fin,
    uint64_t *fout, int flags, int *stop);


struct plldiv_sc {
	device_t		clkdev;
	uint32_t		cm_reg;
	uint32_t		a2w_reg;
	uint32_t		load_mask;
	bool 			have_hold;
	uint32_t		hold_mask;

	int			divider;
};

static clknode_method_t plldiv_methods[] = {
	/* Device interface */
	CLKNODEMETHOD(clknode_init,		plldiv_init),
	CLKNODEMETHOD(clknode_set_gate,		plldiv_set_gate),
	CLKNODEMETHOD(clknode_recalc_freq,	plldiv_recalc),
	CLKNODEMETHOD(clknode_set_freq,		plldiv_set_freq),
	CLKNODEMETHOD_END
};
DEFINE_CLASS_1(bcm2835_plldiv, plldiv_class, plldiv_methods,
   sizeof(struct plldiv_sc), clknode_class);

static int
plldiv_init(struct clknode *clk, device_t dev)
{
	struct plldiv_sc *sc;
	uint32_t div;
	uint32_t reg;

	sc = clknode_get_softc(clk);

	DEVICE_LOCK(sc);
	RD4(sc, sc->a2w_reg, &reg);
	div = A2W_PLLDIV_DIV_GET(reg);
	DEVICE_UNLOCK(sc);

	/* Parse divider. */
	if (div == 0)
		div = 256;
	sc->divider = div;
	DPRINTF("%s: %s  - divier: %d, %s\n",
	    __func__, clknode_get_name(clk), sc->divider,
	    (reg & A2W_PLLDIV_DISABLE) ? "dsabled": "enabled");

	clknode_init_parent_idx(clk, 0);
	return(0);
}

static int
plldiv_set_gate(struct clknode *clk, bool enable)
{
	struct plldiv_sc *sc;

	sc = clknode_get_softc(clk);
	DEVICE_LOCK(sc);

	if (enable) {
		MD4(sc, sc->a2w_reg, A2W_PLLDIV_DISABLE, 0);
		if (sc->have_hold)
			MD4(sc, sc->cm_reg, sc->hold_mask, 0);
	} else {
		MD4(sc, sc->a2w_reg, 0, A2W_PLLDIV_DISABLE);
	}

	DEVICE_UNLOCK(sc);

	return (0);
}

static int
plldiv_recalc(struct clknode *clk, uint64_t *freq)
{
	struct plldiv_sc *sc;

	sc = clknode_get_softc(clk);
	*freq = *freq / sc->divider;
	return (0);
}

static int
plldiv_set_freq(struct clknode *clk, uint64_t fin, uint64_t *fout,
   int flags, int *stop)
{
	struct plldiv_sc *sc;
	uint64_t divider, _fout;
	uint32_t hw_div;

	sc = clknode_get_softc(clk);

	divider = (fin + *fout / 2) / *fout;
	_fout = fin / divider;

	/* Rounding. */
	if ((flags & CLK_SET_ROUND_UP) && (*fout < _fout))
		divider--;
	else if ((flags & CLK_SET_ROUND_DOWN) && (*fout > _fout))
		divider++;

	if (divider == 0) {
		printf("%s: %s - integer divider is zero!\n",
		     __func__,clknode_get_name(clk));
		return(EINVAL);
	}

	*stop = 1;
	if (divider > 256) {
		divider = 256;
		*stop = 0;
	}

	if ((flags & CLK_SET_DRYRUN) == 0) {
		if ((*stop != 0) &&
		    ((flags & (CLK_SET_ROUND_UP | CLK_SET_ROUND_DOWN)) == 0) &&
		    (*fout != (fin / divider)))
			return (ERANGE);

		if (divider == 0)
			return (ERANGE);
		hw_div = divider;
		if (hw_div == 256)
			hw_div = 0;

		DEVICE_LOCK(sc);
		MD4(sc, sc->a2w_reg, A2W_PLLDIV_DIV(~0),
		    A2W_PLLDIV_DIV(hw_div));
		MD4(sc, sc->a2w_reg, 0, sc->load_mask);
		MD4(sc, sc->a2w_reg, sc->load_mask, 0);
		DEVICE_UNLOCK(sc);

		sc->divider = divider;
	}

	*fout = fin / divider;
	return (0);
}

static int
plldiv_register(struct clkdom *clkdom, struct clk_plldiv_def *clkdef)
{
	struct clknode *clk;
	struct plldiv_sc *sc;

	clk = clknode_create(clkdom, &plldiv_class, &clkdef->clkdef);
	if (clk == NULL)
		return (ENXIO);

	sc = clknode_get_softc(clk);
	sc->clkdev = clknode_get_device(clk);
	sc->a2w_reg = clkdef->a2w_reg;
	sc->load_mask = 1U << clkdef->load_bit;
	if (clkdef->hold_bit == -1) {
		sc->have_hold = false;
		sc->hold_mask = 0;
	} else {
		sc->have_hold = true;
		sc->hold_mask = 1U << clkdef->hold_bit;
	}
	clknode_register(clkdom, clk);
	return (0);
}

/* --------   Perpheral mux, divider and gate class   ---------------------- */

static int periph_init(struct clknode *clk, device_t dev);
static int periph_set_gate(struct clknode *clk, bool enable);
static int periph_recalc(struct clknode *clk, uint64_t *freq);
static int periph_set_freq(struct clknode *clk, uint64_t fin,
    uint64_t *fout, int flags, int *stop);
static int periph_set_mux(struct clknode *clk, int idx);

struct periph_sc {
	device_t		clkdev;
	uintptr_t		id;
	uint32_t		flags;
	uint32_t		ctrl_reg;
	uint32_t		div_reg;
	uint32_t		i_width;
	uint32_t		i_mask;
	uint32_t		i_shift;
	uint32_t		f_width;
	uint32_t		f_mask;
	uint32_t		f_shift;

	int 			mux;
	uint32_t		divider;
};

static clknode_method_t periph_methods[] = {
	/* Device interface */
	CLKNODEMETHOD(clknode_init,		periph_init),
	CLKNODEMETHOD(clknode_set_gate,		periph_set_gate),
	CLKNODEMETHOD(clknode_recalc_freq,	periph_recalc),
	CLKNODEMETHOD(clknode_set_freq,		periph_set_freq),
	CLKNODEMETHOD(clknode_set_mux, 		periph_set_mux),
	CLKNODEMETHOD_END
};
DEFINE_CLASS_1(bcm3835_periph, periph_class, periph_methods,
   sizeof(struct periph_sc), clknode_class);

static int
periph_busy_wait(struct periph_sc *sc, struct clknode *clk)
{
	int i;
	uint32_t reg;

	for (i = 10000; i > 0; i--) {
		RD4(sc, sc->ctrl_reg, &reg);
		if ((reg & CM_CTL_BUSY) == 0)
			break;
		DELAY(10);
	}
	if (i <= 0) {
		printf("%s: %s - Timedout when waiting for not BUSY.\n",
		    __func__, clknode_get_name(clk));
		return (ETIMEDOUT);
	}
	return (0);
}

static int
periph_init(struct clknode *clk, device_t dev)
{
	struct periph_sc *sc;
	uint32_t i_div, f_div;
	uint32_t ctrl_reg, div_reg;

	sc = clknode_get_softc(clk);

	DEVICE_LOCK(sc);
	RD4(sc, sc->ctrl_reg, &ctrl_reg);
	RD4(sc, sc->div_reg, &div_reg);
	DEVICE_UNLOCK(sc);

	/* Get multiplexer settings. */
	sc->mux = CM_CTL_MUX_GET(ctrl_reg);

	/* Parse divider. */
	i_div = (div_reg >> sc->i_shift) & sc->i_mask;
	f_div = (div_reg >> sc->f_shift) & sc->f_mask;
	sc->divider = i_div << sc->f_width | f_div;
	DPRINTF("%s: %s  - divider: %d, (%u:%u), bits: %d, %s (raw: 0x%08X)\n",
	    __func__, clknode_get_name(clk), sc->divider,
	    i_div, f_div, sc->f_width,
	    (ctrl_reg & CM_CTL_ENAB) ? "enabled" : "disabled", ctrl_reg);
	clknode_init_parent_idx(clk, sc->mux);

	return(0);
}

static int
periph_set_gate(struct clknode *clk, bool enable)
{
	struct periph_sc *sc;
	int rv;

	/* VPU block doesn't have enable gate */
	sc = clknode_get_softc(clk);
	if (sc->id == BCM2835_CLOCK_VPU)
		return (0);

	DEVICE_LOCK(sc);
	rv = periph_busy_wait(sc, clk);
	if (rv != 0)
		goto fail;

	if (enable)
		MD4(sc, sc->ctrl_reg, 0, CM_CTL_ENAB | CM_CTL_GATE);
	else
		MD4(sc, sc->ctrl_reg, CM_CTL_ENAB, 0);

	rv = periph_busy_wait(sc, clk);
fail:
	DEVICE_UNLOCK(sc);

	return (rv);
}

static int
periph_set_mux(struct clknode *clk, int idx)
{
	struct periph_sc *sc;
	int rv;

	sc = clknode_get_softc(clk);

	DEVICE_LOCK(sc);
	rv = periph_busy_wait(sc, clk);
	if (rv != 0)
		goto fail;

	MD4(sc, sc->ctrl_reg, CM_CTL_MUX(~0), CM_CTL_MUX(idx));
	sc->mux = idx;

	rv = periph_busy_wait(sc, clk);
fail:
	DEVICE_UNLOCK(sc);


	return(rv);
}

static int
periph_recalc(struct clknode *clk, uint64_t *freq)
{
	struct periph_sc *sc;

	sc = clknode_get_softc(clk);
	if (sc->i_width == 0) 	/* Node without divider */
		return (0);

uint64_t tmpf = *freq;
	clk_div_lin_div_to_freq(*freq, sc->divider, CLK_DIV_ZERO_BASED,
	    sc->i_width, sc->f_width, freq);
//	*freq = (*freq << sc->f_width) / sc->divider;

if (*freq != ((tmpf << sc->f_width) / sc->divider))
printf("Error freq: %llu != %llu\n", *freq, ((tmpf << sc->f_width) / sc->divider));		
	return (0);
}

static int
periph_set_freq(struct clknode *clk, uint64_t fin, uint64_t *fout,
   int flags, int *stop)
{
	struct periph_sc *sc;
	uint64_t divider, _fin, _fout;
	uint32_t i_div, f_div;
	int rv;

	sc = clknode_get_softc(clk);

	/* Fractional divider. */
	_fin = fin << sc->f_width;
	divider = (_fin + *fout / 2) / *fout;
	if (divider == 0)
		return (ERANGE);

	_fout = _fin / divider;

	/* Rounding. */
	if ((flags & CLK_SET_ROUND_UP) && (*fout < _fout))
		divider--;
	else if ((flags & CLK_SET_ROUND_DOWN) && (*fout > _fout))
		divider++;
	if (divider == 0)
		return (ERANGE);

	/* Break divider into integer and fractional parts. */
	i_div = divider >> sc->f_width;
	f_div = divider & sc->f_mask;

	if (i_div == 0) {
		printf("%s: %s - integer divider is zero!\n",
		     __func__, clknode_get_name(clk));
		return(ERANGE);
	}

	*stop = 1;
	if (i_div > sc->i_mask) {
		printf("%s: %s - integer divider is too big: %u\n",
		    __func__, clknode_get_name(clk), i_div);
		i_div = sc->i_mask;
		*stop = 0;
	}

	divider = i_div << sc->f_width | f_div;


	if ((flags & CLK_SET_DRYRUN) == 0) {
		if ((*stop != 0) &&
		    ((flags & (CLK_SET_ROUND_UP | CLK_SET_ROUND_DOWN)) == 0) &&
		    (*fout != (_fin / divider)))
			return (ERANGE);

		DEVICE_LOCK(sc);
		rv = periph_busy_wait(sc, clk);
		if (rv != 0)
			goto fail;
		MD4(sc, sc->div_reg,
		    (sc->i_mask << sc->i_shift) | (sc->f_mask << sc->f_shift),
		    (i_div << sc->i_shift) | (f_div << sc->f_shift));
		rv = periph_busy_wait(sc, clk);
fail:
		DEVICE_UNLOCK(sc);
		if (rv != 0)
			return (rv);
		sc->divider = divider;
	}

	*fout = _fin / divider;
	return (0);
}

static int
periph_register(struct clkdom *clkdom, struct clk_periph_def *clkdef)
{
	struct clknode *clk;
	struct periph_sc *sc;

	clk = clknode_create(clkdom, &periph_class, &clkdef->clkdef);
	if (clk == NULL)
		return (1);

	sc = clknode_get_softc(clk);
	sc->clkdev = clknode_get_device(clk);
	sc->id = clkdef->clkdef.id;
	sc->flags = clkdef->flags;
	sc->ctrl_reg = clkdef->ctrl_reg;
	sc->div_reg = clkdef->div_reg;
	/*
	 * The fractional divider have always 12.12 register layout,
	 * but only some bits are implemented.
	 */
	sc->i_width = clkdef->div_i_width;
	sc->i_mask = (1 <<clkdef->div_i_width) - 1;
	sc->i_shift = 12;
	sc->f_width = clkdef->div_f_width;
	sc->f_mask = (1 <<clkdef->div_f_width) - 1;
	sc->f_shift = 12 - clkdef->div_f_width;

	clknode_register(clkdom, clk);
	return (0);
}

/* -----------------------   Device  --------------------------------------- */

static void
register_clocks(device_t dev)
{
	struct bcm2835_cprman_softc *sc;
	int i, rv;

	sc = device_get_softc(dev);
	sc->clkdom = clkdom_create(dev);
	if (sc->clkdom == NULL)
		panic("clkdom == NULL");

	rv = clknode_fixed_register(sc->clkdom, &xosc_clk);
	if (rv != 0)
		panic("clk_fixed_register failed");

	for (i = 0; i < nitems(bmc2835_fixed_clks); i++) {
		rv = clknode_fixed_register(sc->clkdom, bmc2835_fixed_clks + i);
		if (rv != 0)
			panic("clk_fixed_register failed");
	}

	for (i = 0; i < nitems(bmc2835_pll_clks); i++) {
		rv = pll_register(sc->clkdom, bmc2835_pll_clks + i);
		if (rv != 0)
			panic("pll_register failed");
	}

	for (i = 0; i < nitems(bmc2835_plldiv_clks); i++) {
		rv = plldiv_register(sc->clkdom, bmc2835_plldiv_clks + i);
		if (rv != 0)
			panic("plldiv_register failed");
	}

	for (i = 0; i < nitems(bmc2835_periph_clks); i++) {
		rv = periph_register(sc->clkdom, bmc2835_periph_clks + i);
		if (rv != 0)
			panic("periph_register failed");
	}

	for (i = 0; i < nitems(bmc2835_gate_clks); i++) {
		rv = clknode_gate_register(sc->clkdom, bmc2835_gate_clks + i);
		if (rv != 0)
			panic("periph_register failed");
	}

	clkdom_finit(sc->clkdom);
	if (bootverbose)
		clkdom_dump(sc->clkdom);
}

static int
bcm2835_cprman_clkdev_read_4(device_t dev, bus_addr_t addr, uint32_t *val)
{
	struct bcm2835_cprman_softc *sc;

	sc = device_get_softc(dev);
	*val = bus_read_4(sc->mem_res, addr);
	return (0);
}

static int
bcm2835_cprman_clkdev_write_4(device_t dev, bus_addr_t addr, uint32_t val)
{
	struct bcm2835_cprman_softc *sc;

	sc = device_get_softc(dev);
	bus_write_4(sc->mem_res, addr, val | 0x5a000000);      /* CM_PASSWORD */
	return (0);
}

static int
bcm2835_cprman_clkdev_modify_4(device_t dev, bus_addr_t addr,
    uint32_t clear_mask, uint32_t set_mask)
{
	struct bcm2835_cprman_softc *sc;
	uint32_t reg;

	sc = device_get_softc(dev);
	reg = bus_read_4(sc->mem_res, addr);
	reg &= ~clear_mask;
	reg |= set_mask;
	bus_write_4(sc->mem_res, addr, reg | 0x5a000000);      /* CM_PASSWORD */
	return (0);
}

static void
bcm2835_cprman_clkdev_device_lock(device_t dev)
{
	struct bcm2835_cprman_softc *sc;

	sc = device_get_softc(dev);
	mtx_lock(&sc->mtx);
}

static void
bcm2835_cprman_clkdev_device_unlock(device_t dev)
{
	struct bcm2835_cprman_softc *sc;

	sc = device_get_softc(dev);
	mtx_unlock(&sc->mtx);
}

static int
bcm2835_cprman_detach(device_t dev)
{

	device_printf(dev, "Error: Clock driver cannot be detached\n");
	return (EBUSY);
}

static int
bcm2835_cprman_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data != 0) {
		device_set_desc(dev, "BCM283x Clocks");
		return (BUS_PROBE_DEFAULT);
	}

	return (ENXIO);
}

static int
bcm2835_cprman_attach(device_t dev)
{
	struct bcm2835_cprman_softc *sc;
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

	register_clocks(dev);
	return (0);

fail:
	if (sc->mem_res)
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->mem_res);

	return (rv);
}

static device_method_t bcm2835_cprman_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		bcm2835_cprman_probe),
	DEVMETHOD(device_attach,	bcm2835_cprman_attach),
	DEVMETHOD(device_detach,	bcm2835_cprman_detach),

	/* Clkdev  interface*/
	DEVMETHOD(clkdev_read_4,	bcm2835_cprman_clkdev_read_4),
	DEVMETHOD(clkdev_write_4,	bcm2835_cprman_clkdev_write_4),
	DEVMETHOD(clkdev_modify_4,	bcm2835_cprman_clkdev_modify_4),
	DEVMETHOD(clkdev_device_lock,	bcm2835_cprman_clkdev_device_lock),
	DEVMETHOD(clkdev_device_unlock,	bcm2835_cprman_clkdev_device_unlock),

	DEVMETHOD_END
};

static devclass_t bcm2835_cprman_devclass;
static DEFINE_CLASS_0(bcm2835_cprman, bcm2835_cprman_driver,
    bcm2835_cprman_methods, sizeof(struct bcm2835_cprman_softc));
EARLY_DRIVER_MODULE(bcm2835_cprman, simplebus, bcm2835_cprman_driver,
    bcm2835_cprman_devclass, NULL, NULL,
    BUS_PASS_BUS + BUS_PASS_ORDER_MIDDLE + 1);
