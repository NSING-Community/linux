// SPDX-License-Identifier: GPL-2.0-only
/*
 * NSING N32H7xx JPEG SoC glue.
 *
 * The platform driver owns the JPEG blocks and their clock/reset resources.
 * This file only enables the series-specific infrastructure that lets those
 * blocks reach SDRAM: the shared GRAPHICS power domain, APB6 and M7 MMU
 * gates, and the JPEG module-memory enable.
 */

#include <linux/bits.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/soc/nsing/n32h7xx-graphics.h>
#include <linux/soc/nsing/n32h7xx-jpeg.h>

int n32h7xx_jpeg_soc_init(struct device *dev)
{
	void __iomem *rcc;
	void __iomem *mmu;
	u32 reg;
	int ret;

	rcc = devm_ioremap(dev, N32H7XX_RCC_BASE, 0x400);
	mmu = devm_ioremap(dev, N32H7XX_MMU_BASE, 0x400);
	if (!rcc || !mmu)
		return -ENOMEM;

	ret = n32h7xx_graphics_acquire(dev, N32H7XX_GRAPHICS_JPEG);
	if (ret)
		return ret;

	reg = readl(rcc + N32H7XX_RCC_CFG4);
	writel(reg | N32H7XX_RCC_CFG4_APB6CLKEN,
	       rcc + N32H7XX_RCC_CFG4);

	reg = readl(rcc + N32H7XX_RCC_CFG2);
	writel(reg | N32H7XX_RCC_CFG2_M7MMU,
	       rcc + N32H7XX_RCC_CFG2);

	/*
	 * JPEG's SGDMA masters reach SDRAM through the MMU.  Keep this bit set
	 * for the lifetime of the device; clearing it blocks both directions.
	 */
	reg = readl(mmu + N32H7XX_MMU_JPEGME);
	writel(reg | N32H7XX_MMU_ME_EN, mmu + N32H7XX_MMU_JPEGME);

	return 0;
}
EXPORT_SYMBOL_GPL(n32h7xx_jpeg_soc_init);
