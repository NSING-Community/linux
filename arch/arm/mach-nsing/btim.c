// SPDX-License-Identifier: GPL-2.0-only
/* N32H7xx 32-bit basic timer, reserved for system timekeeping. */
#include <linux/clk.h>
#include <linux/clocksource.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/reset.h>

#define N32H7XX_BTIM_CTRL1	0x00
#define N32H7XX_BTIM_STS		0x08
#define N32H7XX_BTIM_EVTGEN	0x0c
#define N32H7XX_BTIM_DINTEN	0x14
#define N32H7XX_BTIM_PSC		0x40
#define N32H7XX_BTIM_AR		0x44
#define N32H7XX_BTIM_CNT		0x48
#define N32H7XX_BTIM_CNTEN	BIT(0)
#define N32H7XX_BTIM_UDGN		BIT(8)
#define N32H7XX_BTIM_RATE		10000000UL

static struct reset_control *n32h7xx_btim_reset;
static bool n32h7xx_btim_registered;

static int __init n32h7xx_btim_init(struct device_node *np)
{
	struct reset_control *reset;
	void __iomem *base;
	unsigned long rate, divisor;
	struct clk *clk;
	int ret;

	base = of_iomap(np, 0);
	if (!base)
		return -ENOMEM;
	clk = of_clk_get(np, 0);
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		goto unmap;
	}
	ret = clk_prepare_enable(clk);
	if (ret)
		goto put_clock;
	rate = clk_get_rate(clk);
	divisor = rate / N32H7XX_BTIM_RATE;
	if (!divisor || divisor > 65536 || rate % N32H7XX_BTIM_RATE) {
		ret = -EINVAL;
		goto disable_clock;
	}
	reset = of_reset_control_get_exclusive(np, NULL);
	if (IS_ERR(reset)) {
		ret = PTR_ERR(reset);
		goto disable_clock;
	}
	ret = reset_control_reset(reset);
	/* Retain ownership; early SRCU cleanup cannot flush work yet. */
	n32h7xx_btim_reset = reset;
	if (ret)
		goto disable_clock;
	writel(0, base + N32H7XX_BTIM_CTRL1);
	writel(0, base + N32H7XX_BTIM_DINTEN);
	writel(divisor - 1, base + N32H7XX_BTIM_PSC);
	writel(U32_MAX, base + N32H7XX_BTIM_AR);
	/* Load the prescaler shadow before starting the counter. */
	writel(N32H7XX_BTIM_UDGN, base + N32H7XX_BTIM_EVTGEN);
	writel(0, base + N32H7XX_BTIM_STS);
	writel(0, base + N32H7XX_BTIM_CNT);
	writel(N32H7XX_BTIM_CNTEN, base + N32H7XX_BTIM_CTRL1);
	ret = clocksource_mmio_init(base + N32H7XX_BTIM_CNT, "n32h7xx-btim1",
				  N32H7XX_BTIM_RATE, 300, 32,
				  clocksource_mmio_readl_up);
	if (ret) {
		writel(0, base + N32H7XX_BTIM_CTRL1);
		goto disable_clock;
	}
	/* Keep the clock and mapping for the clocksource's lifetime. */
	n32h7xx_btim_registered = true;
	pr_info("%pOF: 32-bit clocksource at %lu Hz\n", np, N32H7XX_BTIM_RATE);
	return 0;

disable_clock:
	clk_disable_unprepare(clk);
put_clock:
	clk_put(clk);
unmap:
	iounmap(base);
	return ret;
}
TIMER_OF_DECLARE(n32h7xx_btim, "nsing,n32h7xx-btim", n32h7xx_btim_init);

static int __init n32h7xx_btim_release_failed_reset(void)
{
	/* Failed early initialization can release SRCU resources at this stage. */
	if (!n32h7xx_btim_registered)
		reset_control_put(n32h7xx_btim_reset);
	return 0;
}
late_initcall(n32h7xx_btim_release_failed_reset);
