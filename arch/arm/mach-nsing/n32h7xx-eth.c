// SPDX-License-Identifier: GPL-2.0-only
/*
 * NSING N32H7xx Ethernet MAC SoC support.
 *
 * This file owns everything about the Ethernet MAC that is a property of the
 * N32H7xx series rather than of the MAC hardware itself: the RCC gates and
 * reset, the PWR/HSC1 power domain, the AFIO interface select and the GPIO
 * pin mux.  The MAC, MTL and DMA programming lives in the network driver.
 */

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/gpio/consumer.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/of.h>
#include <linux/printk.h>
#include <linux/string.h>
#include <linux/soc/nsing/n32h7xx-eth.h>
#include <linux/soc/nsing/n32h7xx-ucid.h>

/* AHB5 register blocks touched here. */
#define N32H7XX_RCC_BASE		0x58030000
#define N32H7XX_AFIO_BASE		0x58032400
#define N32H7XX_PWR_BASE		0x58031000

/* RCC */
#define N32H7XX_RCC_AHB5EN1		0x0e4
#define N32H7XX_RCC_AHB5EN2		0x0e8
#define N32H7XX_RCC_AHB2DIV1		0x0a8
#define N32H7XX_RCC_AHB2SEL1		0x0ac
#define N32H7XX_RCC_AHB2RST1		0x0b4
#define N32H7XX_RCC_AHB2EN2		0x1b4

#define N32H7XX_AHB5EN1_GPIO		(BIT(31) | BIT(27) | BIT(23) | \
					 BIT(19) | BIT(15) | BIT(11) | \
					 BIT(7) | BIT(3))
#define N32H7XX_AHB5EN2_GPIO		(BIT(31) | BIT(27) | BIT(23))
#define N32H7XX_AHB5EN2_AFIO		BIT(3)
#define N32H7XX_AHB5EN2_PWR		BIT(15)

#define N32H7XX_AHB2EN2_ETH1_TX		BIT(11)
#define N32H7XX_AHB2EN2_ETH1_RX		BIT(7)
#define N32H7XX_AHB2EN2_ETH1_MAC	BIT(3)
#define N32H7XX_AHB2EN2_ETH1		(N32H7XX_AHB2EN2_ETH1_TX | \
					 N32H7XX_AHB2EN2_ETH1_RX | \
					 N32H7XX_AHB2EN2_ETH1_MAC)
#define N32H7XX_AHB2RST1_ETH1		BIT(16)

/* ETH1PTPSEL[1:0] and ETH1SYSDIV[3:0]. */
#define N32H7XX_AHB2SEL1_ETH1_PTP	GENMASK(21, 20)
#define N32H7XX_AHB2DIV1_ETH1_DIV	GENMASK(31, 28)

/* PWR */
#define N32H7XX_PWR_SYSCTRL3		0x4c
#define N32H7XX_PWR_IPMEMCTRL		0x58
#define N32H7XX_PWR_IPMEMCTRLSTS	0x5c

#define N32H7XX_PWR_HSC1_PSWACK1	BIT(21)
#define N32H7XX_PWR_HSC1_PWRRDY		BIT(17)
#define N32H7XX_PWR_HSC1_ISNEN		BIT(9)
#define N32H7XX_PWR_HSC1_FUCEN		BIT(5)
#define N32H7XX_PWR_HSC1_PWREN		BIT(1)
#define N32H7XX_PWR_ETH1_PWREN		BIT(8)

/* AFIO */
#define N32H7XX_AFIO_RMP_CFG		0x00
#define N32H7XX_AFIO_ETH1_PHY		GENMASK(14, 13)

#define N32H7XX_PHY_RESET_SETTLE_MS	20
#define N32H7XX_POWER_TIMEOUT_US	100000

static void n32h7xx_enable_aux_clocks(void __iomem *rcc)
{
	u32 reg;

	/* GPIO ports A..K, AFIO and PWR all live on AHB5. */
	reg = readl(rcc + N32H7XX_RCC_AHB5EN1);
	writel(reg | N32H7XX_AHB5EN1_GPIO, rcc + N32H7XX_RCC_AHB5EN1);

	reg = readl(rcc + N32H7XX_RCC_AHB5EN2);
	writel(reg | N32H7XX_AHB5EN2_GPIO | N32H7XX_AHB5EN2_AFIO |
	       N32H7XX_AHB5EN2_PWR, rcc + N32H7XX_RCC_AHB5EN2);

	/* The PWR register block must be clocked before the next access. */
	readl(rcc + N32H7XX_RCC_AHB5EN2);
}

static int n32h7xx_enable_hsc1_power(struct device *dev, void __iomem *pwr)
{
	u32 reg, val;
	int ret;

	/*
	 * The ETH1 memory gate is released by clearing its IPMEMCTRL bit; the
	 * matching IPMEMCTRLSTS bit then reads back as set once the memory has
	 * powered up.
	 */
	reg = readl(pwr + N32H7XX_PWR_IPMEMCTRL);
	writel(reg & ~N32H7XX_PWR_ETH1_PWREN, pwr + N32H7XX_PWR_IPMEMCTRL);

	ret = readl_poll_timeout(pwr + N32H7XX_PWR_IPMEMCTRLSTS, val,
				 val & N32H7XX_PWR_ETH1_PWREN, 10,
				 N32H7XX_POWER_TIMEOUT_US);
	if (ret)
		return dev_err_probe(dev, ret,
				     "ETH1 memory power did not become ready\n");

	if (readl(pwr + N32H7XX_PWR_SYSCTRL3) & N32H7XX_PWR_HSC1_PWRRDY)
		return 0;

	reg = readl(pwr + N32H7XX_PWR_SYSCTRL3);
	writel(reg | N32H7XX_PWR_HSC1_PSWACK1, pwr + N32H7XX_PWR_SYSCTRL3);

	reg = readl(pwr + N32H7XX_PWR_SYSCTRL3);
	writel(reg | N32H7XX_PWR_HSC1_PWREN, pwr + N32H7XX_PWR_SYSCTRL3);

	ret = readl_poll_timeout(pwr + N32H7XX_PWR_SYSCTRL3, val,
				 val & N32H7XX_PWR_HSC1_PWRRDY, 10,
				 N32H7XX_POWER_TIMEOUT_US);
	if (ret)
		return dev_err_probe(dev, ret,
				     "HSC1 power domain did not become ready\n");

	reg = readl(pwr + N32H7XX_PWR_SYSCTRL3);
	writel(reg | N32H7XX_PWR_HSC1_FUCEN, pwr + N32H7XX_PWR_SYSCTRL3);

	reg = readl(pwr + N32H7XX_PWR_SYSCTRL3);
	writel(reg | N32H7XX_PWR_HSC1_ISNEN, pwr + N32H7XX_PWR_SYSCTRL3);

	return 0;
}

static int n32h7xx_eth1_phy_sel(struct device_node *np, u32 *sel)
{
	const char *mode;
	int ret;

	ret = of_property_read_string(np, "phy-mode", &mode);
	if (ret)
		mode = "rmii";

	if (!strcmp(mode, "rmii"))
		*sel = 2;
	else if (!strcmp(mode, "mii"))
		*sel = 3;
	else if (!strcmp(mode, "gmii"))
		*sel = 0;
	else
		return -EINVAL;

	return 0;
}

/*
 * The board describes the PHY reset line as "phy-reset-gpios" plus a hold
 * time.  The line is requested already asserted so the PHY stays in reset
 * from the first register access until the MAC is ready for it.
 */
static int n32h7xx_phy_reset(struct device *dev)
{
	struct gpio_desc *reset;
	u32 duration = N32H7XX_PHY_RESET_SETTLE_MS;

	reset = devm_gpiod_get_optional(dev, "phy-reset", GPIOD_OUT_HIGH);
	if (IS_ERR(reset))
		return dev_err_probe(dev, PTR_ERR(reset),
				     "cannot get PHY reset GPIO\n");
	if (!reset)
		return 0;

	device_property_read_u32(dev, "phy-reset-duration", &duration);
	msleep(duration);

	/* Release, then let the PHY strap pins settle before using the bus. */
	gpiod_set_value_cansleep(reset, 0);
	msleep(N32H7XX_PHY_RESET_SETTLE_MS);

	return 0;
}

int n32h7xx_eth_soc_init(struct device *dev)
{
	struct device_node *np = dev->of_node;
	void __iomem *rcc, *afio, *pwr;
	u32 sel, reg;
	int ret;

	ret = n32h7xx_eth1_phy_sel(np, &sel);
	if (ret)
		return dev_err_probe(dev, ret, "unsupported phy-mode\n");

	rcc = devm_ioremap(dev, N32H7XX_RCC_BASE, 0x400);
	afio = devm_ioremap(dev, N32H7XX_AFIO_BASE, 0x400);
	pwr = devm_ioremap(dev, N32H7XX_PWR_BASE, 0x400);
	if (!rcc || !afio || !pwr)
		return -ENOMEM;

	n32h7xx_enable_aux_clocks(rcc);

	ret = n32h7xx_enable_hsc1_power(dev, pwr);
	if (ret)
		return ret;

	/* Open the ETH1 MAC/TX/RX gates, then pulse the peripheral reset. */
	reg = readl(rcc + N32H7XX_RCC_AHB2EN2);
	writel(reg | N32H7XX_AHB2EN2_ETH1, rcc + N32H7XX_RCC_AHB2EN2);

	reg = readl(rcc + N32H7XX_RCC_AHB2RST1);
	writel(reg | N32H7XX_AHB2RST1_ETH1, rcc + N32H7XX_RCC_AHB2RST1);
	udelay(10);
	writel(reg & ~N32H7XX_AHB2RST1_ETH1, rcc + N32H7XX_RCC_AHB2RST1);
	udelay(10);

	/* The PTP timestamp clock is the sysbus clock divided by two. */
	reg = readl(rcc + N32H7XX_RCC_AHB2DIV1);
	reg &= ~N32H7XX_AHB2DIV1_ETH1_DIV;
	reg |= FIELD_PREP(N32H7XX_AHB2DIV1_ETH1_DIV, 1);
	writel(reg, rcc + N32H7XX_RCC_AHB2DIV1);

	reg = readl(rcc + N32H7XX_RCC_AHB2SEL1);
	reg &= ~N32H7XX_AHB2SEL1_ETH1_PTP;
	writel(reg, rcc + N32H7XX_RCC_AHB2SEL1);

	/* Route the MAC onto the RMII/MII/GMII pins of the package. */
	reg = readl(afio + N32H7XX_AFIO_RMP_CFG);
	reg &= ~N32H7XX_AFIO_ETH1_PHY;
	reg |= FIELD_PREP(N32H7XX_AFIO_ETH1_PHY, sel);
	writel(reg, afio + N32H7XX_AFIO_RMP_CFG);

	return n32h7xx_phy_reset(dev);
}
EXPORT_SYMBOL_GPL(n32h7xx_eth_soc_init);

/*
 * The part has no factory MAC address, and the Cortex-M7 CPUID only reports a
 * part number, so the only per-board identity available is the 128-bit UCID
 * in OTP.  Build the address from it with the NSING USB vendor ID (0x19f5) in
 * the top two octets and a 32-bit fold of the UCID in the remaining four:
 *
 *   1a:f5:xx:xx:xx:xx
 *
 * The vendor ID cannot be copied verbatim.  0x19 as the first octet has the
 * multicast (I/G) bit set, which is not a valid unicast station address, and
 * a value taken from a USB VID is not a registered Ethernet OUI either.  The
 * I/G bit is cleared and the locally administered (U/L) bit is set, which is
 * the standard way to make up an address that will not collide with a real
 * vendor assignment.  Folding all four UCID words (rather than using one)
 * keeps the whole 128-bit ID contributing to the 32 bits a MAC address can
 * carry.
 */
int n32h7xx_eth_soc_mac_addr(u8 *addr)
{
	u32 ucid[N32H7XX_UCID_WORDS];
	u32 id;

	if (n32h7xx_ucid_read(ucid))
		return -ENODEV;

	id = ucid[0] ^ ucid[1] ^ ucid[2] ^ ucid[3];
	if (!id)
		id = 0x00000001;	/* never emit a zero station address */

	addr[0] = 0x1a;			/* USB VID 0x19f5, U/L set, I/G clear */
	addr[1] = 0xf5;
	addr[2] = (id >> 24) & 0xff;
	addr[3] = (id >> 16) & 0xff;
	addr[4] = (id >> 8) & 0xff;
	addr[5] = id & 0xff;

	return 0;
}
EXPORT_SYMBOL_GPL(n32h7xx_eth_soc_mac_addr);
