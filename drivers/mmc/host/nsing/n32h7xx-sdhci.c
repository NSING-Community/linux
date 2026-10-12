// SPDX-License-Identifier: GPL-2.0-only
/*
 * NSING N32H7xx SDHCI host driver.
 *
 * The SDMMC controller is a standard SDHCI core behind an NSING wrapper
 * which describes its capabilities to the core.  SoC clocks, resets,
 * power gates and pin mux are handled by mach-nsing.
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/mmc/host.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/soc/nsing/n32h7xx-storage.h>

#include "../sdhci.h"
#include "../sdhci-pltfm.h"

#define N32H7XX_SDMMC_CFG1		0x00
#define N32H7XX_SDMMC_CFG2		0x04
#define N32H7XX_SDMMC_CFG3		0x08

#define N32H7XX_SDMMC_CFG1_MBL_MASK	GENMASK(23, 22)
#define N32H7XX_SDMMC_CFG1_BCLKF_MASK	GENMASK(21, 14)
#define N32H7XX_SDMMC_CFG1_TCLKU	BIT(13)
#define N32H7XX_SDMMC_CFG1_TCNT_MASK	GENMASK(6, 1)
#define N32H7XX_SDMMC_CFG1_WSGM		BIT(0)
#define N32H7XX_SDMMC_CFG1_TUNING_COUNT	0x20

#define N32H7XX_SDMMC_CFG2_SPIBMOD	BIT(19)
#define N32H7XX_SDMMC_CFG2_SPIMOD	BIT(18)
#define N32H7XX_SDMMC_CFG2_DDR50	BIT(14)
#define N32H7XX_SDMMC_CFG2_SDR104	BIT(13)
#define N32H7XX_SDMMC_CFG2_SDR50	BIT(12)
#define N32H7XX_SDMMC_CFG2_STYP_MASK	GENMASK(11, 10)
#define N32H7XX_SDMMC_CFG2_ASYNCINT	BIT(9)
#define N32H7XX_SDMMC_CFG2_VS33		BIT(5)
#define N32H7XX_SDMMC_CFG2_SRS		BIT(4)
#define N32H7XX_SDMMC_CFG2_SDMA		BIT(3)
#define N32H7XX_SDMMC_CFG2_HS		BIT(2)
#define N32H7XX_SDMMC_CFG2_ADMA2	BIT(1)
#define N32H7XX_SDMMC_CFG2_EMBUS	BIT(0)

#define N32H7XX_SDMMC_CFG3_UTFSDR50	BIT(4)

/*
 * SD bus voltage select (bits 11:9) and SD bus power switch (bit 8) live in
 * the 32-bit HOST_CONTROL/POWER_CONTROL word at 0x28.  The core only latches
 * them on a 32-bit store: a byte write to POWER_CONTROL updates the voltage
 * field but silently drops the power bit, so the bus stays unpowered, the
 * controller accepts a command, waits for a response that can never come and
 * never raises a command interrupt either.  Mirror the vendor sequence
 * instead: a 32-bit read-modify-write to settle the voltage, then a second
 * one to close the power switch, and verify the switch really closed.
 */
#define N32H7XX_SDHCI_CTRL1_SDBVSEL	GENMASK(11, 9)
#define N32H7XX_SDHCI_CTRL1_SDPWR	BIT(8)

/*
 * Card-detect select and test level live in CTRL1 bits 7 and 6.  This board
 * has no wired CD pin (the DT says broken-cd), so the controller's card-detect
 * input floats and Present State reports "no card" (CINS=0).  With CINS=0 the
 * core will not run commands at all: a command is accepted into the command
 * register but never issued, produces no response, and never even times out.
 * The bootloader avoids this with the vendor programCardDetect() call, which
 * forces the test level high; do the same here.
 */
#define N32H7XX_SDHCI_CTRL1_CDSD	BIT(7)
#define N32H7XX_SDHCI_CTRL1_CDTL	BIT(6)

#define N32H7XX_SDHCI_POWER_SETTLE_US	10000
#define N32H7XX_SDHCI_POWER_TRIES	200

static void n32h7xx_sdhci_configure_wrapper(void __iomem *wrapper,
					    unsigned long clock_hz)
{
	unsigned long base_mhz = DIV_ROUND_UP(clock_hz, 1000000);
	u32 reg;

	/* Match the validated bootloader wrapper configuration. */
	reg = readl(wrapper + N32H7XX_SDMMC_CFG1);
	reg &= ~(N32H7XX_SDMMC_CFG1_MBL_MASK |
		 N32H7XX_SDMMC_CFG1_BCLKF_MASK |
		 N32H7XX_SDMMC_CFG1_TCLKU |
		 N32H7XX_SDMMC_CFG1_TCNT_MASK |
		 N32H7XX_SDMMC_CFG1_WSGM);
	reg |= FIELD_PREP(N32H7XX_SDMMC_CFG1_BCLKF_MASK, base_mhz);
	reg |= FIELD_PREP(N32H7XX_SDMMC_CFG1_TCNT_MASK,
			  N32H7XX_SDMMC_CFG1_TUNING_COUNT);
	reg |= N32H7XX_SDMMC_CFG1_WSGM;
	writel(reg, wrapper + N32H7XX_SDMMC_CFG1);

	/*
	 * Expose only the baseline SD/MMC capabilities for now.  DMA is
	 * SDMA/ADMA2, the bus is 3.3 V and high-speed signalling is the fastest
	 * mode used by the current board.
	 */
	reg = readl(wrapper + N32H7XX_SDMMC_CFG2);
	reg &= ~(N32H7XX_SDMMC_CFG2_SPIBMOD |
		 N32H7XX_SDMMC_CFG2_SPIMOD |
		 N32H7XX_SDMMC_CFG2_DDR50 |
		 N32H7XX_SDMMC_CFG2_SDR104 |
		 N32H7XX_SDMMC_CFG2_SDR50 |
		 N32H7XX_SDMMC_CFG2_STYP_MASK |
		 N32H7XX_SDMMC_CFG2_SRS |
		 N32H7XX_SDMMC_CFG2_EMBUS);
	reg |= N32H7XX_SDMMC_CFG2_ASYNCINT |
	       N32H7XX_SDMMC_CFG2_VS33 |
	       N32H7XX_SDMMC_CFG2_SDMA |
	       N32H7XX_SDMMC_CFG2_ADMA2 |
	       N32H7XX_SDMMC_CFG2_HS;
	writel(reg, wrapper + N32H7XX_SDMMC_CFG2);

	reg = readl(wrapper + N32H7XX_SDMMC_CFG3);
	reg &= ~N32H7XX_SDMMC_CFG3_UTFSDR50;
	writel(reg, wrapper + N32H7XX_SDMMC_CFG3);
}

static unsigned int n32h7xx_sdhci_get_max_clock(struct sdhci_host *host)
{
	return n32h7xx_sdmmc1_soc_clock_rate();
}

static void n32h7xx_sdhci_set_power(struct sdhci_host *host, unsigned char mode,
				    unsigned short vdd)
{
	void __iomem *reg = host->ioaddr + SDHCI_HOST_CONTROL;
	u8 pwr = 0;
	u32 ctrl1;
	int i;

	if (mode != MMC_POWER_OFF)
		pwr = sdhci_get_vdd_value(vdd);

	if (host->pwr == pwr)
		return;
	host->pwr = pwr;

	/* Settle the selected voltage first; the power switch latches after. */
	ctrl1 = readl(reg);
	ctrl1 &= ~(N32H7XX_SDHCI_CTRL1_SDBVSEL | N32H7XX_SDHCI_CTRL1_SDPWR);
	if (pwr)
		ctrl1 |= ((u32)pwr << 8) & N32H7XX_SDHCI_CTRL1_SDBVSEL;
	writel(ctrl1, reg);
	mdelay(N32H7XX_SDHCI_POWER_SETTLE_US / 1000);

	if (!pwr)
		return;

	for (i = 0; i < N32H7XX_SDHCI_POWER_TRIES; i++) {
		writel(ctrl1 | N32H7XX_SDHCI_CTRL1_SDPWR, reg);
		mdelay(N32H7XX_SDHCI_POWER_SETTLE_US / 1000);
		if (readl(reg) & N32H7XX_SDHCI_CTRL1_SDPWR)
			return;
	}

	dev_err(&host->mmc->class_dev,
		"SD bus power did not enable after %d attempts\n",
		N32H7XX_SDHCI_POWER_TRIES);
}

static void n32h7xx_sdhci_writew(struct sdhci_host *host, u16 val, int reg)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);

	switch (reg) {
	case SDHCI_TRANSFER_MODE:
		/*
		 * This controller latches the transfer-mode settings only when
		 * the command register is written as one 32-bit TMODE+CMD
		 * word, so hold the value and fold it into the command write
		 * below.
		 */
		pltfm_host->xfer_mode_shadow = val;
		return;
	case SDHCI_COMMAND:
		writel((val << 16) | pltfm_host->xfer_mode_shadow,
		       host->ioaddr + SDHCI_TRANSFER_MODE);
		return;
	}

	writew(val, host->ioaddr + reg);
}

static void n32h7xx_sdhci_reset(struct sdhci_host *host, u8 mask);

static const struct sdhci_ops n32h7xx_sdhci_ops = {
	.write_w = n32h7xx_sdhci_writew,
	.get_max_clock = n32h7xx_sdhci_get_max_clock,
	.set_clock = sdhci_set_clock,
	.set_power = n32h7xx_sdhci_set_power,
	.set_bus_width = sdhci_set_bus_width,
	.set_uhs_signaling = sdhci_set_uhs_signaling,
	.reset = n32h7xx_sdhci_reset,
};

static const struct sdhci_pltfm_data n32h7xx_sdhci_data = {
	.ops = &n32h7xx_sdhci_ops,
	/* Match the vendor's 65532-byte maximum ADMA transfer descriptor. */
	.quirks = SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN |
		  SDHCI_QUIRK_BROKEN_ADMA_ZEROLEN_DESC,
	/* Non-data commands must not inherit the previous DMA transfer mode. */
	.quirks2 = SDHCI_QUIRK2_PRESET_VALUE_BROKEN |
		   SDHCI_QUIRK2_CLEAR_TRANSFERMODE_REG_BEFORE_CMD,
};

/*
 * This core latches INTSTS only while the interrupt-enable registers are set:
 * with INT_ENABLE/SIGNAL_ENABLE clear it records no status at all.  A command
 * then finishes on the wire -- the response lands in SDHCI_RESPONSE -- but no
 * interrupt is ever raised, so sdhci_send_command() times out and reports
 * -ETIMEDOUT.  A reset-all clears both registers and the generic core does not
 * put them back, so re-arm them after every reset.
 */
#define N32H7XX_SDHCI_INT_DEFAULT					\
	(SDHCI_INT_BUS_POWER | SDHCI_INT_DATA_END_BIT |			\
	 SDHCI_INT_DATA_CRC | SDHCI_INT_DATA_TIMEOUT | SDHCI_INT_INDEX |	\
	 SDHCI_INT_END_BIT | SDHCI_INT_CRC | SDHCI_INT_TIMEOUT |	\
	 SDHCI_INT_DATA_END | SDHCI_INT_RESPONSE)

static void n32h7xx_sdhci_reset(struct sdhci_host *host, u8 mask)
{
	u32 ier = host->ier ? host->ier : N32H7XX_SDHCI_INT_DEFAULT;

	sdhci_reset(host, mask);

	/*
	 * A reset-all also drops the forced card-detect test level; this board
	 * has no wired CD pin, so without it Present State reports "no card"
	 * and the core refuses to run any command.
	 */
	if (mask & SDHCI_RESET_ALL) {
		struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
		u32 ctrl1 = sdhci_readl(host, SDHCI_HOST_CONTROL);

		pltfm_host->xfer_mode_shadow = 0;
		ctrl1 |= N32H7XX_SDHCI_CTRL1_CDSD | N32H7XX_SDHCI_CTRL1_CDTL;
		sdhci_writel(host, ctrl1, SDHCI_HOST_CONTROL);
	}

	sdhci_writel(host, ier, SDHCI_INT_ENABLE);
	sdhci_writel(host, ier, SDHCI_SIGNAL_ENABLE);
}
static int n32h7xx_sdhci_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct sdhci_host *host;
	void __iomem *wrapper;
	int ret;

	ret = n32h7xx_sdmmc1_soc_init(dev);
	if (ret)
		return ret;

	host = sdhci_pltfm_init(pdev, &n32h7xx_sdhci_data, 0);
	if (IS_ERR(host))
		return PTR_ERR(host);

	wrapper = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(wrapper))
		return PTR_ERR(wrapper);

	n32h7xx_sdhci_configure_wrapper(wrapper,
				       n32h7xx_sdmmc1_soc_clock_rate());

	sdhci_get_property(pdev);

	ret = mmc_of_parse(host->mmc);
	if (ret)
		return dev_err_probe(dev, ret, "failed to parse MMC properties\n");

	ret = sdhci_add_host(host);
	if (ret)
		return dev_err_probe(dev, ret, "failed to add SDHCI host\n");

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dev_info(dev, "NSING N32H7xx SDHCI host at %pa, %u Hz max\n",
		 &res->start, host->max_clk);

	return 0;
}

static const struct of_device_id n32h7xx_sdhci_of_match[] = {
	{ .compatible = "nsing,n32h7xx-sdhci" },
	{ }
};
MODULE_DEVICE_TABLE(of, n32h7xx_sdhci_of_match);

static struct platform_driver n32h7xx_sdhci_driver = {
	.driver = {
		.name = "n32h7xx-sdhci",
		.of_match_table = n32h7xx_sdhci_of_match,
		.pm = &sdhci_pltfm_pmops,
	},
	.probe = n32h7xx_sdhci_probe,
	.remove = sdhci_pltfm_remove,
};
module_platform_driver(n32h7xx_sdhci_driver);

MODULE_DESCRIPTION("NSING N32H7xx SDHCI driver");
MODULE_LICENSE("GPL");
