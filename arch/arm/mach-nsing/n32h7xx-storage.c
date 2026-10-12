// SPDX-License-Identifier: GPL-2.0-only
/*
 * NSING N32H7xx storage-controller SoC support.
 *
 * Everything here is a property of the series rather than of a storage
 * controller: the RCC clock gates and resets, the PWR power domains and the
 * AFIO/GPIO pin mux.  The controllers themselves (drivers/spi/nsing and
 * drivers/mmc/host/nsing) stay free of SoC register knowledge.
 */

#include <linux/bitops.h>
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/of.h>
#include <linux/printk.h>
#include <linux/soc/nsing/n32h7xx-storage.h>

#define N32H7XX_RCC_BASE		0x58030000
#define N32H7XX_PWR_BASE		0x58031000
#define N32H7XX_AFIO_BASE		0x58032400

#define N32H7XX_RCC_PLL2CTRL1	0x10
#define N32H7XX_RCC_PLL2CTRL2	0x14
#define N32H7XX_RCC_PLL3CTRL1	0x20
#define N32H7XX_RCC_SRCCTRL1	0x30
#define N32H7XX_RCC_PLL2DIV	0x38
#define N32H7XX_PLL_SRC_MASK	GENMASK(29, 28)
#define N32H7XX_PLL_SRC_HSE	GENMASK(29, 28)
#define N32H7XX_PLL_LOCK		BIT(20)
#define N32H7XX_PLL_LDO		BIT(19)
#define N32H7XX_PLL_ENABLE	BIT(18)
#define N32H7XX_PLL_RESET		BIT(17)
#define N32H7XX_PLL_POWER_DOWN	BIT(16)
#define N32H7XX_PLL_BWAJ		GENMASK(11, 0)
#define N32H7XX_PLL_A_DIV		GENMASK(5, 0)
#define N32H7XX_HSE_READY		BIT(3)
#define N32H7XX_HSE_ENABLE	BIT(2)

/* AHB5: GPIO ports and AFIO share the same block as the Ethernet glue uses. */
#define N32H7XX_RCC_AHB5EN1		0x0e4
#define N32H7XX_RCC_AHB5EN2		0x0e8

#define N32H7XX_AHB5EN1_GPIO		(BIT(31) | BIT(27) | BIT(23) | \
					 BIT(19) | BIT(15) | BIT(11) | \
					 BIT(7) | BIT(3))
#define N32H7XX_AHB5EN2_GPIO		(BIT(31) | BIT(27) | BIT(23))
#define N32H7XX_AHB5EN2_AFIO		BIT(3)

/* AXI clock/divider/select/enable/reset registers. */
#define N32H7XX_RCC_AXIDIV1		0x138
#define N32H7XX_RCC_AXISEL1		0x13c
#define N32H7XX_RCC_AXIEN1		0x140
#define N32H7XX_RCC_AXIEN4		0x14c
#define N32H7XX_RCC_AXIRST1		0x150
#define N32H7XX_RCC_AXIRST4		0x15c

/* XSPI2: AXIEN4 run bit27 / LP bit25, AXIRST4 bit24, SSI source select [2:0]. */
#define N32H7XX_AXIEN4_XSPI2		(BIT(27) | BIT(25))
#define N32H7XX_AXIRST4_XSPI2		BIT(24)
#define N32H7XX_AXISEL1_XSPI2SSISEL	GENMASK(2, 0)

/* SDMMC1: kernel clock select [22:20], AXI divider [23:20]. */
#define N32H7XX_AXISEL1_SDMMC1KERSEL	GENMASK(22, 20)
#define N32H7XX_AXIDIV1_SDMMC1AXIDIV	GENMASK(23, 20)
#define N32H7XX_SDMMC1KERSEL_AXIDIV	0
#define N32H7XX_SDMMC1KERSEL_PLL2A	2
#define N32H7XX_SDMMC1AXIDIV_DIV8	4
#define N32H7XX_AXIEN1_SDMMC1		(BIT(11) | BIT(9))
#define N32H7XX_AXIRST1_SDMMC1		BIT(9)
#define N32H7XX_AXIRST1_SDHOST1		BIT(8)

/* PWR: SDMMC1 memory power gate (IPMEMCTRL/IPMEMCTRLSTS bit10). */
#define N32H7XX_PWR_IPMEMCTRL		0x58
#define N32H7XX_PWR_IPMEMCTRLSTS	0x5c
#define N32H7XX_PWR_SDMMC1		BIT(10)

/* AFIO route: feed the SDMMC1 clock back into the controller. */
#define N32H7XX_AFIO_RMP_CFG		0x00
#define N32H7XX_AFIO_RMP_CFG_SDMMC1_CLKFB	BIT(24)

#define N32H7XX_POWER_TIMEOUT_US	100000

/*
 * The AXI clock is PLL1A / 2, and PLL1A is left at 600 MHz by the bootloader,
 * so SDMMC1 on AXIDIV/8 supplies the 37.5 MHz fallback. The H787 board can
 * instead use PLL2A at 100 MHz for the standard 50 MHz SD high-speed bus.
 */
/* SDMMC1AXIDIV lives in AXIDIV1 bits [23:20]; the field value is a code,
 * not a power of two: 0 = /1, 1 = /2, 2 = /4, 4 = /8, ...
 */
#define N32H7XX_SDMMC1_AXIDIV_SHIFT	20
#define N32H7XX_AXI_CLOCK_HZ		300000000
#define N32H7XX_SDMMC1_CLOCK_HZ		(N32H7XX_AXI_CLOCK_HZ / 8)

static unsigned long n32h7xx_sdmmc1_rate = N32H7XX_SDMMC1_CLOCK_HZ;

/* H787's 25 MHz HSE: CLKF=20*16384, CLKR=0, PLL2=500 MHz, A=/5. */
#define N32H7XX_PLL2_500M_CLKF	(20U * 16384)
#define N32H7XX_PLL2_A_DIV5	5

static int n32h7xx_sdmmc1_pll2_init(void __iomem *rcc)
{
	u32 ctrl, reg;
	int ret;

	/* This PLL configuration requires the HMI board's 25 MHz crystal. */
	if (!of_machine_is_compatible("nsing,n32h787-hmi"))
		return -EOPNOTSUPP;

	ctrl = readl(rcc + N32H7XX_RCC_PLL2CTRL1);
	if (ctrl & N32H7XX_PLL_ENABLE) {
		/* Do not retune an enabled PLL potentially shared by other users. */
		if ((ctrl & N32H7XX_PLL_SRC_MASK) != N32H7XX_PLL_SRC_HSE ||
		    readl(rcc + N32H7XX_RCC_PLL2CTRL2) != N32H7XX_PLL2_500M_CLKF ||
		    FIELD_GET(N32H7XX_PLL_A_DIV,
			      readl(rcc + N32H7XX_RCC_PLL2DIV)) != N32H7XX_PLL2_A_DIV5)
			return -EBUSY;
		return readl_poll_timeout(rcc + N32H7XX_RCC_PLL2CTRL1, reg,
					  reg & N32H7XX_PLL_LOCK, 10, 100000);
	}
	reg = readl(rcc + N32H7XX_RCC_SRCCTRL1);
	writel(reg | N32H7XX_HSE_ENABLE, rcc + N32H7XX_RCC_SRCCTRL1);
	ret = readl_poll_timeout(rcc + N32H7XX_RCC_SRCCTRL1, reg,
				 reg & N32H7XX_HSE_READY, 10, 100000);
	if (ret)
		return ret;

	ctrl &= ~(N32H7XX_PLL_SRC_MASK | N32H7XX_PLL_BWAJ);
	ctrl |= N32H7XX_PLL_SRC_HSE | N32H7XX_PLL_RESET |
		N32H7XX_PLL_POWER_DOWN | FIELD_PREP(N32H7XX_PLL_BWAJ, 9);
	writel(ctrl, rcc + N32H7XX_RCC_PLL2CTRL1);
	writel(N32H7XX_PLL2_500M_CLKF, rcc + N32H7XX_RCC_PLL2CTRL2);
	/* The vendor PLL2 sequence enables this supply in PLL3CTRL1.
	 * Only set its enable bit; never reset, retune or power down PLL3,
	 * which supplies the running SDRAM.
	 */
	reg = readl(rcc + N32H7XX_RCC_PLL3CTRL1);
	writel(reg | N32H7XX_PLL_LDO, rcc + N32H7XX_RCC_PLL3CTRL1);
	udelay(10);
	ctrl &= ~N32H7XX_PLL_POWER_DOWN;
	writel(ctrl, rcc + N32H7XX_RCC_PLL2CTRL1);
	udelay(10);
	ctrl &= ~N32H7XX_PLL_RESET;
	writel(ctrl, rcc + N32H7XX_RCC_PLL2CTRL1);
	ret = readl_poll_timeout(rcc + N32H7XX_RCC_PLL2CTRL1, reg,
				 reg & N32H7XX_PLL_LOCK, 10, 100000);
	if (ret) {
		writel(ctrl | N32H7XX_PLL_RESET | N32H7XX_PLL_POWER_DOWN,
		       rcc + N32H7XX_RCC_PLL2CTRL1);
		return ret;
	}
	writel(ctrl | N32H7XX_PLL_ENABLE, rcc + N32H7XX_RCC_PLL2CTRL1);
	/* Program the output divider after enabling PLL2, as the SDK does. */
	reg = readl(rcc + N32H7XX_RCC_PLL2DIV);
	reg = (reg & ~N32H7XX_PLL_A_DIV) | N32H7XX_PLL2_A_DIV5;
	writel(reg, rcc + N32H7XX_RCC_PLL2DIV);
	if (FIELD_GET(N32H7XX_PLL_A_DIV,
		      readl(rcc + N32H7XX_RCC_PLL2DIV)) != N32H7XX_PLL2_A_DIV5)
		return -EIO;
	return 0;
}

static void n32h7xx_storage_aux_clocks(void __iomem *rcc)
{
	u32 reg;

	reg = readl(rcc + N32H7XX_RCC_AHB5EN1);
	writel(reg | N32H7XX_AHB5EN1_GPIO, rcc + N32H7XX_RCC_AHB5EN1);

	reg = readl(rcc + N32H7XX_RCC_AHB5EN2);
	writel(reg | N32H7XX_AHB5EN2_GPIO | N32H7XX_AHB5EN2_AFIO,
	       rcc + N32H7XX_RCC_AHB5EN2);

	/* The PWR register block must be clocked before the next access. */
	readl(rcc + N32H7XX_RCC_AHB5EN2);
}

int n32h7xx_xspi2_soc_init(struct device *dev)
{
	void __iomem *rcc;
	u32 reg;

	rcc = devm_ioremap(dev, N32H7XX_RCC_BASE, 0x400);
	if (!rcc)
		return -ENOMEM;

	n32h7xx_storage_aux_clocks(rcc);

	/* Open the XSPI2 run and low-power gates. */
	reg = readl(rcc + N32H7XX_RCC_AXIEN4);
	writel(reg | N32H7XX_AXIEN4_XSPI2, rcc + N32H7XX_RCC_AXIEN4);

	/* Pulse the peripheral reset, as the vendor XSPI_DeInit() does. */
	reg = readl(rcc + N32H7XX_RCC_AXIRST4);
	writel(reg | N32H7XX_AXIRST4_XSPI2, rcc + N32H7XX_RCC_AXIRST4);
	udelay(10);
	writel(reg & ~N32H7XX_AXIRST4_XSPI2, rcc + N32H7XX_RCC_AXIRST4);
	udelay(10);

	/* The SSI input is the AXI bus clock; leave the selector at its reset value. */
	reg = readl(rcc + N32H7XX_RCC_AXISEL1);
	reg &= ~N32H7XX_AXISEL1_XSPI2SSISEL;
	writel(reg, rcc + N32H7XX_RCC_AXISEL1);

	return 0;
}
EXPORT_SYMBOL_GPL(n32h7xx_xspi2_soc_init);

int n32h7xx_sdmmc1_soc_init(struct device *dev)
{
	void __iomem *rcc, *pwr, *afio;
	u32 reg, val, source = N32H7XX_SDMMC1KERSEL_AXIDIV;
	int ret;

	rcc = devm_ioremap(dev, N32H7XX_RCC_BASE, 0x400);
	pwr = devm_ioremap(dev, N32H7XX_PWR_BASE, 0x400);
	afio = devm_ioremap(dev, N32H7XX_AFIO_BASE, 0x400);
	if (!rcc || !pwr || !afio)
		return -ENOMEM;

	n32h7xx_storage_aux_clocks(rcc);
	n32h7xx_sdmmc1_rate = N32H7XX_SDMMC1_CLOCK_HZ;
	ret = n32h7xx_sdmmc1_pll2_init(rcc);
	if (!ret) {
		source = N32H7XX_SDMMC1KERSEL_PLL2A;
		n32h7xx_sdmmc1_rate = 100000000;
	} else {
		dev_warn(dev, "PLL2 unavailable (%d), using 37.5 MHz SD source\n", ret);
	}

	/* Release the SDMMC1 memory power gate and wait for it to come up. */
	reg = readl(pwr + N32H7XX_PWR_IPMEMCTRL);
	writel(reg & ~N32H7XX_PWR_SDMMC1, pwr + N32H7XX_PWR_IPMEMCTRL);

	ret = readl_poll_timeout(pwr + N32H7XX_PWR_IPMEMCTRLSTS, val,
				 val & N32H7XX_PWR_SDMMC1, 10,
				 N32H7XX_POWER_TIMEOUT_US);
	if (ret)
		return dev_err_probe(dev, ret,
				     "SDMMC1 memory power did not become ready\n");

	/* Preserve the AXI/8 fallback even when the PLL2 source is selected. */
	reg = readl(rcc + N32H7XX_RCC_AXIDIV1);
	reg &= ~N32H7XX_AXIDIV1_SDMMC1AXIDIV;
	reg |= N32H7XX_SDMMC1AXIDIV_DIV8 << N32H7XX_SDMMC1_AXIDIV_SHIFT;
	writel(reg, rcc + N32H7XX_RCC_AXIDIV1);

	reg = readl(rcc + N32H7XX_RCC_AXISEL1);
	reg &= ~N32H7XX_AXISEL1_SDMMC1KERSEL;
	reg |= FIELD_PREP(N32H7XX_AXISEL1_SDMMC1KERSEL, source);
	writel(reg, rcc + N32H7XX_RCC_AXISEL1);

	/* Open the run and low-power gates before touching either register block. */
	reg = readl(rcc + N32H7XX_RCC_AXIEN1);
	writel(reg | N32H7XX_AXIEN1_SDMMC1, rcc + N32H7XX_RCC_AXIEN1);

	/* The controller and its wrapper have independent resets. */
	reg = readl(rcc + N32H7XX_RCC_AXIRST1);
	writel(reg | N32H7XX_AXIRST1_SDMMC1 | N32H7XX_AXIRST1_SDHOST1,
	       rcc + N32H7XX_RCC_AXIRST1);
	udelay(10);
	writel(reg & ~(N32H7XX_AXIRST1_SDMMC1 | N32H7XX_AXIRST1_SDHOST1),
	       rcc + N32H7XX_RCC_AXIRST1);
	udelay(10);

	/* Route the SDMMC1 output clock back to the controller's RX path. */
	reg = readl(afio + N32H7XX_AFIO_RMP_CFG);
	writel(reg | N32H7XX_AFIO_RMP_CFG_SDMMC1_CLKFB,
	       afio + N32H7XX_AFIO_RMP_CFG);

	return 0;
}
EXPORT_SYMBOL_GPL(n32h7xx_sdmmc1_soc_init);

unsigned long n32h7xx_sdmmc1_soc_clock_rate(void)
{
	return n32h7xx_sdmmc1_rate;
}
EXPORT_SYMBOL_GPL(n32h7xx_sdmmc1_soc_clock_rate);
