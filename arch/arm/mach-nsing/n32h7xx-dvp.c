// SPDX-License-Identifier: GPL-2.0-only
/*
 * NSING N32H7xx DVP (parallel camera port) SoC support.
 *
 * The DVP capture units themselves are programmed by
 * drivers/media/platform/nsing/n32h7xx-dvp.c.  Everything that is a property
 * of the N32H7xx series rather than of the DVP logic lives here: the shared
 * GRAPHICS power-domain reference, the APB6/AXI clock gates, the DVP
 * module clock divider and mux, the peripheral reset, and the MMU module
 * memory bit that lets the DVP AHB master reach SDRAM.
 *
 * The register layout and the power-up order are taken from the vendor
 * N32H7xx_Library (n32h7xx_rcc.c / n32h7xx_pwr.c) and cross-checked against
 * the N32H7xx user manual: the PWR chapter and chapter 46 (DVP).
 */

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/soc/nsing/n32h7xx-dvp.h>
#include <linux/soc/nsing/n32h7xx-graphics.h>

/* AHB5/AHB6 register blocks touched here. */
#define N32H7XX_RCC_BASE		0x58030000
#define N32H7XX_MMU_BASE		0x51050000

#define N32H7XX_DVP1_BASE		0x50048000
#define N32H7XX_DVP2_BASE		0x50049000

/* RCC */
#define N32H7XX_RCC_AXIDIV1		0x138
#define N32H7XX_RCC_AXISEL1		0x13c
#define N32H7XX_RCC_AXIEN2		0x144
#define N32H7XX_RCC_AXIRST2		0x154
#define N32H7XX_RCC_CFG2		0x160
#define N32H7XX_RCC_CFG4		0x168

/* AXIDIV1.DVPMAXIDIV[3:0] selects the DVP module clock divide (bit 11 is not
 * part of the encoding: DIV16 is 0x7 << 8).
 */
#define N32H7XX_RCC_AXIDIV1_DVPMAXIDIV	GENMASK(11, 8)
#define N32H7XX_RCC_DVPMCLK_DIV16	FIELD_PREP(N32H7XX_RCC_AXIDIV1_DVPMAXIDIV, 7)

/* AXISEL1.DVP2MSEL[1:0]: 0 = AXI clock divided by DVPMAXIDIV. */
#define N32H7XX_RCC_AXISEL1_DVP2MSEL	GENMASK(9, 8)
#define N32H7XX_RCC_AXISEL1_DVP1MSEL	GENMASK(11, 10)

/* AXIEN2: M7DVP2EN | M7DVP2LPEN | M7DVP2APBEN | M7DVP2APBLPEN. */
#define N32H7XX_RCC_AXIEN2_DVP2		(BIT(11) | BIT(9) | BIT(7) | BIT(5))
#define N32H7XX_RCC_AXIEN2_DVP1		(BIT(19) | BIT(17) | BIT(15) | BIT(13))
#define N32H7XX_RCC_AXIRST2_DVP2	BIT(8)
#define N32H7XX_RCC_AXIRST2_DVP1	BIT(16)
#define N32H7XX_RCC_CFG2_M7MMU		(BIT(23) | BIT(22))
#define N32H7XX_RCC_CFG4_APB6CLKEN	BIT(19)

/* MMU: per-master module-memory enable, bit 0 = EN. */
#define N32H7XX_MMU_DVP1ME		0x144
#define N32H7XX_MMU_DVP2ME		0x148
#define N32H7XX_MMU_ME_EN		BIT(0)

#define N32H7XX_DVP_RESET_SETTLE_US	10

struct n32h7xx_dvp_hw {
	void __iomem *rcc;
	void __iomem *mmu;
	unsigned int instance;		/* 1 or 2 */
};

static int n32h7xx_dvp_instance(struct device_node *np, unsigned int *instance)
{
	u32 base;
	int ret;

	ret = of_property_read_u32_index(np, "reg", 0, &base);
	if (ret)
		return ret;

	switch (base) {
	case N32H7XX_DVP1_BASE:
		*instance = 1;
		return 0;
	case N32H7XX_DVP2_BASE:
		*instance = 2;
		return 0;
	default:
		return -EINVAL;
	}
}

int n32h7xx_dvp_soc_init(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct n32h7xx_dvp_hw hw;
	u32 reg;
	int ret;

	ret = n32h7xx_dvp_instance(np, &hw.instance);
	if (ret)
		return dev_err_probe(dev, ret, "unknown DVP instance\n");

	hw.rcc = devm_ioremap(dev, N32H7XX_RCC_BASE, 0x400);
	hw.mmu = devm_ioremap(dev, N32H7XX_MMU_BASE, 0x400);
	if (!hw.rcc || !hw.mmu)
		return -ENOMEM;

	ret = n32h7xx_graphics_acquire(dev, N32H7XX_GRAPHICS_DVP);
	if (ret)
		return ret;

	/*
	 * DVP1/DVP2, LCDC and the MMU all hang off the AHB6/APB6 clock domain,
	 * and the MMU needs the M7 clock gate open before its module-memory
	 * registers can be written.
	 */
	reg = readl(hw.rcc + N32H7XX_RCC_CFG4);
	writel(reg | N32H7XX_RCC_CFG4_APB6CLKEN, hw.rcc + N32H7XX_RCC_CFG4);

	reg = readl(hw.rcc + N32H7XX_RCC_CFG2);
	writel(reg | N32H7XX_RCC_CFG2_M7MMU, hw.rcc + N32H7XX_RCC_CFG2);

	/* DVP module clock: AXI divided by 16, no separate peripheral source. */
	reg = readl(hw.rcc + N32H7XX_RCC_AXISEL1);
	reg &= ~(hw.instance == 2 ? N32H7XX_RCC_AXISEL1_DVP2MSEL :
				  N32H7XX_RCC_AXISEL1_DVP1MSEL);
	writel(reg, hw.rcc + N32H7XX_RCC_AXISEL1);

	reg = readl(hw.rcc + N32H7XX_RCC_AXIDIV1);
	reg &= ~N32H7XX_RCC_AXIDIV1_DVPMAXIDIV;
	reg |= N32H7XX_RCC_DVPMCLK_DIV16;
	writel(reg, hw.rcc + N32H7XX_RCC_AXIDIV1);

	/* AHB/peripheral gates, then a pulse of the peripheral reset. */
	reg = readl(hw.rcc + N32H7XX_RCC_AXIEN2);
	writel(reg | (hw.instance == 2 ? N32H7XX_RCC_AXIEN2_DVP2 :
				      N32H7XX_RCC_AXIEN2_DVP1),
	       hw.rcc + N32H7XX_RCC_AXIEN2);

	reg = readl(hw.rcc + N32H7XX_RCC_AXIRST2);
	writel(reg | (hw.instance == 2 ? N32H7XX_RCC_AXIRST2_DVP2 :
				       N32H7XX_RCC_AXIRST2_DVP1),
	       hw.rcc + N32H7XX_RCC_AXIRST2);
	udelay(N32H7XX_DVP_RESET_SETTLE_US);
	writel(reg & ~(hw.instance == 2 ? N32H7XX_RCC_AXIRST2_DVP2 :
				        N32H7XX_RCC_AXIRST2_DVP1),
	       hw.rcc + N32H7XX_RCC_AXIRST2);
	udelay(N32H7XX_DVP_RESET_SETTLE_US);

	/*
	 * The DVP is an AHB master and reaches SDRAM through the MMU.  With
	 * the DVP2 module-memory bit clear the master is blocked and a capture
	 * silently writes nothing.
	 */
	reg = readl(hw.mmu + (hw.instance == 2 ? N32H7XX_MMU_DVP2ME :
						 N32H7XX_MMU_DVP1ME));
	writel(reg | N32H7XX_MMU_ME_EN,
	       hw.mmu + (hw.instance == 2 ? N32H7XX_MMU_DVP2ME :
					    N32H7XX_MMU_DVP1ME));

	return 0;
}
EXPORT_SYMBOL_GPL(n32h7xx_dvp_soc_init);
