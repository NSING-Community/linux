// SPDX-License-Identifier: GPL-2.0-only
/* Integrated USB HS PHY and wrapper resources, shared with a DWC2 core. */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>

#define WRPCFG		0x04
#define PHYCLKSEL	GENMASK(2, 0)
#define PLLEN		BIT(3)
#define IDEN		BIT(9)
#define IDSIG		BIT(12)
#define SYSCTRL3		0x4c
#define IPMEMCTRL	0x58
#define IPMEMCTRLSTS	0x5c

struct n32h7xx_usb_phy {
	void __iomem *base;
	struct regmap *pwr;
	struct clk *ref;
	struct clk *bus;
	struct reset_control *por;
	u32 memory;
	u32 domain;
	u32 saved_config;
	enum phy_mode mode;
	bool external_id;
};

static int n32h7xx_usb_phy_init(struct phy *phy)
{
	struct n32h7xx_usb_phy *usb = phy_get_drvdata(phy);
	static const unsigned long rates[] = {
		10000000, 12000000, 25000000, 30000000,
		19200000, 24000000, 27000000, 40000000,
	};
	u32 val, ready = BIT(16 + usb->domain);
	int i, ret;

	ret = clk_prepare_enable(usb->ref);
	if (ret)
		return ret;
	ret = clk_prepare_enable(usb->bus);
	if (ret)
		goto disable_ref;
	for (i = 0; i < ARRAY_SIZE(rates); i++)
		if (clk_get_rate(usb->ref) == rates[i])
			break;
	if (i == ARRAY_SIZE(rates)) {
		ret = -EINVAL;
		goto disable_bus;
	}
	ret = regmap_update_bits(usb->pwr, IPMEMCTRL, usb->memory, 0);
	if (ret)
		goto disable_bus;
	ret = regmap_read_poll_timeout(usb->pwr, IPMEMCTRLSTS, val,
				       val & usb->memory, 10, 100000);
	if (ret)
		goto disable_bus;
	/* HSC domains are shared with ETH/SDMMC; never power them off here. */
	ret = regmap_update_bits(usb->pwr, SYSCTRL3,
				 BIT(20 + usb->domain) | BIT(usb->domain),
				 BIT(20 + usb->domain) | BIT(usb->domain));
	if (ret)
		goto disable_bus;
	ret = regmap_read_poll_timeout(usb->pwr, SYSCTRL3, val,
				       val & ready, 10, 100000);
	if (ret)
		goto disable_bus;
	ret = regmap_update_bits(usb->pwr, SYSCTRL3,
				 BIT(4 + usb->domain) | BIT(8 + usb->domain),
				 BIT(4 + usb->domain) | BIT(8 + usb->domain));
	if (ret)
		goto disable_bus;
	usb->saved_config = readl(usb->base + WRPCFG);
	val = usb->saved_config & ~(PHYCLKSEL | PLLEN | IDEN | IDSIG);
	val |= i | PLLEN;
	if (usb->external_id)
		val |= IDEN;
	else if (usb->mode != PHY_MODE_USB_HOST)
		val |= IDSIG;
	writel(val, usb->base + WRPCFG);
	readl(usb->base + WRPCFG);
	ret = reset_control_reset(usb->por);
	if (ret) {
		writel(usb->saved_config, usb->base + WRPCFG);
		goto disable_bus;
	}
	/* Match the PHY POR settling time in the vendor initialization. */
	usleep_range(1000, 2000);
	/* POR clears the internal ID override; restore the selected role. */
	writel(val, usb->base + WRPCFG);
	readl(usb->base + WRPCFG);
	return 0;

disable_bus:
	clk_disable_unprepare(usb->bus);
disable_ref:
	clk_disable_unprepare(usb->ref);
	return ret;
}

static int n32h7xx_usb_phy_exit(struct phy *phy)
{
	struct n32h7xx_usb_phy *usb = phy_get_drvdata(phy);

	writel(readl(usb->base + WRPCFG) & ~PLLEN, usb->base + WRPCFG);
	readl(usb->base + WRPCFG);
	clk_disable_unprepare(usb->bus);
	clk_disable_unprepare(usb->ref);
	return 0;
}

static int n32h7xx_usb_phy_set_mode(struct phy *phy, enum phy_mode mode,
				    int submode)
{
	struct n32h7xx_usb_phy *usb = phy_get_drvdata(phy);
	u32 val;

	if (usb->external_id)
		return 0;
	if (mode != PHY_MODE_USB_HOST && mode != PHY_MODE_USB_DEVICE)
		return -EINVAL;
	usb->mode = mode;
	val = readl(usb->base + WRPCFG) & ~IDSIG;
	if (mode == PHY_MODE_USB_DEVICE)
		val |= IDSIG;
	writel(val, usb->base + WRPCFG);
	readl(usb->base + WRPCFG);
	return 0;
}

static const struct phy_ops n32h7xx_usb_phy_ops = {
	.init = n32h7xx_usb_phy_init,
	.exit = n32h7xx_usb_phy_exit,
	.set_mode = n32h7xx_usb_phy_set_mode,
	.owner = THIS_MODULE,
};

static int n32h7xx_usb_phy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct phy_provider *provider;
	struct n32h7xx_usb_phy *usb;
	struct phy *phy;
	u32 index;
	int ret;

	usb = devm_kzalloc(dev, sizeof(*usb), GFP_KERNEL);
	if (!usb)
		return -ENOMEM;
	usb->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(usb->base))
		return PTR_ERR(usb->base);
	ret = of_property_read_u32(dev->of_node, "nsing,controller", &index);
	if (ret || index < 1 || index > 2)
		return -EINVAL;
	usb->domain = index;
	usb->memory = index == 1 ? BIT(9) : BIT(6);
	usb->external_id = of_property_read_bool(dev->of_node, "nsing,external-id");
	usb->pwr = syscon_regmap_lookup_by_phandle(dev->of_node, "nsing,pwr");
	if (IS_ERR(usb->pwr))
		return dev_err_probe(dev, PTR_ERR(usb->pwr), "cannot get PWR\n");
	usb->ref = devm_clk_get(dev, "ref");
	if (IS_ERR(usb->ref))
		return dev_err_probe(dev, PTR_ERR(usb->ref), "cannot get PHY clock\n");
	usb->bus = devm_clk_get(dev, "bus");
	if (IS_ERR(usb->bus))
		return dev_err_probe(dev, PTR_ERR(usb->bus), "cannot get bus clock\n");
	usb->por = devm_reset_control_get_exclusive(dev, "por");
	if (IS_ERR(usb->por))
		return dev_err_probe(dev, PTR_ERR(usb->por), "cannot get PHY reset\n");
	phy = devm_phy_create(dev, NULL, &n32h7xx_usb_phy_ops);
	if (IS_ERR(phy))
		return PTR_ERR(phy);
	phy_set_drvdata(phy, usb);
	provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	return PTR_ERR_OR_ZERO(provider);
}

static const struct of_device_id n32h7xx_usb_phy_match[] = {
	{ .compatible = "nsing,n32h7xx-usb-phy" },
	{ }
};
MODULE_DEVICE_TABLE(of, n32h7xx_usb_phy_match);

static struct platform_driver n32h7xx_usb_phy_driver = {
	.probe = n32h7xx_usb_phy_probe,
	.driver = {
		.name = "n32h7xx-usb-phy",
		.of_match_table = n32h7xx_usb_phy_match,
	},
};
module_platform_driver(n32h7xx_usb_phy_driver);

MODULE_DESCRIPTION("NSING N32H7xx USB HS PHY");
MODULE_LICENSE("GPL");
