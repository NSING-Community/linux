// SPDX-License-Identifier: GPL-2.0-only
/*
 * NSING N32H7xx RTC SoC support.
 *
 * The RTC counts the 32.768 kHz LSE crystal, and both the oscillator and the
 * RTC clock select are RCC/PWR bits that the RTC peripheral knows nothing
 * about.  That is series knowledge, so it stays here; the driver in
 * drivers/rtc/nsing only touches the calendar registers.
 *
 * Power model: the board has no VBAT battery and the backup domain is fed
 * from VDD.  A warm reset therefore keeps the RTC running, while a power
 * cycle loses the calendar and all 32 backup registers.  The domain is only
 * reset when it has never been configured, which is the one case that has to
 * reset it: RTCSEL and RTCEN are latched values that a reset is what makes
 * take effect, and until then nothing is clocked and nothing can be lost.
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/printk.h>
#include <linux/soc/nsing/n32h7xx-rtc.h>

#define N32H7XX_RCC_BASE		0x58030000
#define N32H7XX_PWR_BASE		0x58031000
#define N32H7XX_AFEC_BASE		0x58000400

/* RCC */
#define N32H7XX_RCC_AHB5EN2		0x0e8
#define N32H7XX_RCC_APB5EN2		0x100
#define N32H7XX_RCC_BDCTRL		0x124

#define N32H7XX_AHB5EN2_PWR		BIT(15)

#define N32H7XX_APB5EN2_RTCPCLK		BIT(11)
#define N32H7XX_APB5EN2_RTCPCLK_LP	BIT(9)

#define N32H7XX_BDCTRL_RTCSEL		GENMASK(17, 16)
#define N32H7XX_BDCTRL_RTCSEL_LSE	BIT(16)
#define N32H7XX_BDCTRL_RTCEN		BIT(18)
#define N32H7XX_BDCTRL_BDRST		BIT(12)
#define N32H7XX_BDCTRL_LSELDOEN		BIT(28)
#define N32H7XX_BDCTRL_LSERDCNTEN	BIT(22)
#define N32H7XX_BDCTRL_LSEBP		BIT(6)
#define N32H7XX_BDCTRL_LSERDEN		BIT(7)
#define N32H7XX_BDCTRL_LSEEN		BIT(4)
#define N32H7XX_BDCTRL_LSERDF		BIT(5)
#define N32H7XX_BDCTRL_AFELSERDF	BIT(31)
#define N32H7XX_BDCTRL_LSE_READY \
	(N32H7XX_BDCTRL_LSERDF | N32H7XX_BDCTRL_AFELSERDF)

/* PWR */
#define N32H7XX_PWR_SYSCTRL1		0x40
#define N32H7XX_PWR_DBKP		BIT(8)

/* AFEC: analog trim for the LSE amplifier. */
#define N32H7XX_AFEC_TRIMR7		0x1c
#define N32H7XX_AFEC_TRIMR7_LSE_AGCOPT	BIT(22)

/*
 * The crystal takes tens of milliseconds to settle.  The vendor library
 * spins on a 0x18000000-iteration counter; a generous real-time bound is
 * both faster on success and a real error on failure.
 */
#define N32H7XX_LSE_TIMEOUT_US		3000000

/*
 * PWR has to be clocked before its registers accept writes, and BDCTRL plus
 * the backup registers stay write protected until DBP is set.
 */
static void n32h7xx_rtc_backup_unlock(void __iomem *rcc, void __iomem *pwr)
{
	u32 reg;

	reg = readl(rcc + N32H7XX_RCC_AHB5EN2);
	writel(reg | N32H7XX_AHB5EN2_PWR, rcc + N32H7XX_RCC_AHB5EN2);
	readl(rcc + N32H7XX_RCC_AHB5EN2);

	reg = readl(pwr + N32H7XX_PWR_SYSCTRL1);
	writel(reg | N32H7XX_PWR_DBKP, pwr + N32H7XX_PWR_SYSCTRL1);
	readl(pwr + N32H7XX_PWR_SYSCTRL1);
}

/*
 * RTCSEL and RTCEN live in the backup domain and only take effect once that
 * domain has been reset, so the very first configuration of it has to pulse
 * BDRST.  Nothing is clocked at that point, so no calendar is lost.  The
 * pulse stops the LSE, so it has to happen before the oscillator is started.
 */
static void n32h7xx_rtc_backup_reset(void __iomem *rcc)
{
	writel(readl(rcc + N32H7XX_RCC_BDCTRL) | N32H7XX_BDCTRL_BDRST,
	       rcc + N32H7XX_RCC_BDCTRL);
	readl(rcc + N32H7XX_RCC_BDCTRL);

	writel(readl(rcc + N32H7XX_RCC_BDCTRL) & ~N32H7XX_BDCTRL_BDRST,
	       rcc + N32H7XX_RCC_BDCTRL);
	readl(rcc + N32H7XX_RCC_BDCTRL);
}

static int n32h7xx_rtc_start_lse(struct device *dev, void __iomem *rcc,
				 void __iomem *afec)
{
	u32 reg, val;
	int ret;

	reg = readl(rcc + N32H7XX_RCC_BDCTRL);
	if ((reg & N32H7XX_BDCTRL_LSE_READY) == N32H7XX_BDCTRL_LSE_READY)
		return 0;

	/* Crystal mode, ready counter off, LSE low-drive output on. */
	reg = readl(rcc + N32H7XX_RCC_BDCTRL);
	reg &= ~(N32H7XX_BDCTRL_LSEBP | N32H7XX_BDCTRL_LSERDCNTEN);
	reg |= N32H7XX_BDCTRL_LSELDOEN;
	writel(reg, rcc + N32H7XX_RCC_BDCTRL);
	fsleep(60);

	/* AGC for the crystal amplifier, as the vendor library programs it. */
	reg = readl(afec + N32H7XX_AFEC_TRIMR7);
	writel(reg | N32H7XX_AFEC_TRIMR7_LSE_AGCOPT,
	       afec + N32H7XX_AFEC_TRIMR7);

	reg = readl(rcc + N32H7XX_RCC_BDCTRL);
	writel(reg | N32H7XX_BDCTRL_LSEEN | N32H7XX_BDCTRL_LSERDEN,
	       rcc + N32H7XX_RCC_BDCTRL);
	fsleep(10);
	reg = readl(rcc + N32H7XX_RCC_BDCTRL);
	writel(reg | N32H7XX_BDCTRL_LSERDCNTEN, rcc + N32H7XX_RCC_BDCTRL);

	ret = readl_poll_timeout(rcc + N32H7XX_RCC_BDCTRL, val,
				 (val & N32H7XX_BDCTRL_LSE_READY) ==
				 N32H7XX_BDCTRL_LSE_READY,
				 1000, N32H7XX_LSE_TIMEOUT_US);
	if (ret)
		return dev_err_probe(dev, ret, "LSE oscillator did not start\n");

	return 0;
}

int n32h7xx_rtc_soc_init(struct device *dev)
{
	void __iomem *rcc, *pwr, *afec;
	u32 reg;
	int ret;

	rcc = devm_ioremap(dev, N32H7XX_RCC_BASE, 0x400);
	pwr = devm_ioremap(dev, N32H7XX_PWR_BASE, 0x400);
	afec = devm_ioremap(dev, N32H7XX_AFEC_BASE, 0x400);
	if (!rcc || !pwr || !afec)
		return -ENOMEM;

	n32h7xx_rtc_backup_unlock(rcc, pwr);

	reg = readl(rcc + N32H7XX_RCC_BDCTRL);
	switch (reg & N32H7XX_BDCTRL_RTCSEL) {
	case 0:
		/*
		 * The domain has never been configured, so nothing is clocked
		 * and there is no calendar to preserve: reset it, otherwise
		 * the RTCSEL and RTCEN written below would never latch.
		 */
		n32h7xx_rtc_backup_reset(rcc);
		break;
	case N32H7XX_BDCTRL_RTCSEL_LSE:
		/* Already on the LSE; leave the running calendar alone. */
		break;
	default:
		return dev_err_probe(dev, -EINVAL,
				     "RTC is clocked from something other than the LSE\n");
	}

	ret = n32h7xx_rtc_start_lse(dev, rcc, afec);
	if (ret)
		return ret;

	reg = readl(rcc + N32H7XX_RCC_BDCTRL);
	if ((reg & N32H7XX_BDCTRL_RTCSEL) == 0) {
		writel((reg & ~N32H7XX_BDCTRL_RTCSEL) |
		       N32H7XX_BDCTRL_RTCSEL_LSE, rcc + N32H7XX_RCC_BDCTRL);
		readl(rcc + N32H7XX_RCC_BDCTRL);
	}

	/* Open the RTC APB clock for the register block, then the RTC itself. */
	reg = readl(rcc + N32H7XX_RCC_APB5EN2);
	writel(reg | N32H7XX_APB5EN2_RTCPCLK | N32H7XX_APB5EN2_RTCPCLK_LP,
	       rcc + N32H7XX_RCC_APB5EN2);
	readl(rcc + N32H7XX_RCC_APB5EN2);

	reg = readl(rcc + N32H7XX_RCC_BDCTRL);
	writel(reg | N32H7XX_BDCTRL_RTCEN, rcc + N32H7XX_RCC_BDCTRL);
	readl(rcc + N32H7XX_RCC_BDCTRL);

	return 0;
}
EXPORT_SYMBOL_GPL(n32h7xx_rtc_soc_init);
