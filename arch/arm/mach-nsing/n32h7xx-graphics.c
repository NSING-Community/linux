// SPDX-License-Identifier: GPL-2.0-only
/*
 * NSING N32H7xx GRAPHICS power domain.
 *
 * DVP, JPEG and the other GRAPHICS peripherals have independent IP memory
 * gates but share one switchable power domain.  Drivers therefore acquire a
 * module reference here instead of touching PWR directly: the last user may
 * power the domain down, while removing one peripheral never cuts power to
 * another peripheral that is still active.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/bitops.h>
#include <linux/bug.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/soc/nsing/n32h7xx-graphics.h>

#define N32H7XX_PWR_BASE		0x58031000
#define N32H7XX_PWR_SYSCTRL3		0x4c
#define N32H7XX_PWR_IPMEMCTRL		0x58
#define N32H7XX_PWR_IPMEMCTRLSTS	0x5c

#define N32H7XX_PWR_IPMEMCTRL_JPEG	BIT(2)
#define N32H7XX_PWR_IPMEMCTRL_DVP	BIT(4)
#define N32H7XX_PWR_IPMEMCTRL_GRAPHICS	GENMASK(4, 0)

#define N32H7XX_PWR_GRC_PWREN		BIT(0)
#define N32H7XX_PWR_GRC_FUCEN		BIT(4)
#define N32H7XX_PWR_GRC_ISNEN		BIT(8)
#define N32H7XX_PWR_GRC_PWRRDY		BIT(16)
#define N32H7XX_PWR_GRC_PSWACK1		BIT(20)

#define N32H7XX_GRAPHICS_TIMEOUT_US	100000

static DEFINE_MUTEX(n32h7xx_graphics_lock);
static void __iomem *n32h7xx_graphics_pwr;
static unsigned int n32h7xx_graphics_users;
static unsigned int n32h7xx_graphics_module_users[2];

struct n32h7xx_graphics_ref {
	struct device *dev;
	enum n32h7xx_graphics_module module;
};

static u32 n32h7xx_graphics_gate(enum n32h7xx_graphics_module module)
{
	switch (module) {
	case N32H7XX_GRAPHICS_DVP:
		return N32H7XX_PWR_IPMEMCTRL_DVP;
	case N32H7XX_GRAPHICS_JPEG:
		return N32H7XX_PWR_IPMEMCTRL_JPEG;
	default:
		return 0;
	}
}

static int n32h7xx_graphics_module_enable(struct device *dev, u32 gate)
{
	u32 val;
	int ret;

	writel(readl(n32h7xx_graphics_pwr + N32H7XX_PWR_IPMEMCTRL) & ~gate,
	       n32h7xx_graphics_pwr + N32H7XX_PWR_IPMEMCTRL);
	ret = readl_poll_timeout(n32h7xx_graphics_pwr +
				 N32H7XX_PWR_IPMEMCTRLSTS, val,
				 val & gate, 10,
				 N32H7XX_GRAPHICS_TIMEOUT_US);
	if (ret)
		return dev_err_probe(dev, ret,
				     "GRAPHICS memory gate 0x%x did not become ready\n",
				     gate);

	return 0;
}

static int n32h7xx_graphics_module_disable(u32 gate)
{
	u32 val;

	writel(readl(n32h7xx_graphics_pwr + N32H7XX_PWR_IPMEMCTRL) | gate,
	       n32h7xx_graphics_pwr + N32H7XX_PWR_IPMEMCTRL);
	return readl_poll_timeout(n32h7xx_graphics_pwr +
				  N32H7XX_PWR_IPMEMCTRLSTS, val,
				  !(val & gate), 10,
				  N32H7XX_GRAPHICS_TIMEOUT_US);
}

static int n32h7xx_graphics_domain_enable(struct device *dev)
{
	u32 val;
	int ret;

	val = readl(n32h7xx_graphics_pwr + N32H7XX_PWR_SYSCTRL3);
	if (!(val & N32H7XX_PWR_GRC_PWRRDY)) {
		writel(val | N32H7XX_PWR_GRC_PSWACK1,
		       n32h7xx_graphics_pwr + N32H7XX_PWR_SYSCTRL3);
		writel(readl(n32h7xx_graphics_pwr + N32H7XX_PWR_SYSCTRL3) |
		       N32H7XX_PWR_GRC_PWREN,
		       n32h7xx_graphics_pwr + N32H7XX_PWR_SYSCTRL3);
		ret = readl_poll_timeout(n32h7xx_graphics_pwr +
					 N32H7XX_PWR_SYSCTRL3, val,
					 val & N32H7XX_PWR_GRC_PWRRDY, 10,
					 N32H7XX_GRAPHICS_TIMEOUT_US);
		if (ret)
			return dev_err_probe(dev, ret,
					     "GRAPHICS power domain did not become ready\n");
	}

	writel(readl(n32h7xx_graphics_pwr + N32H7XX_PWR_SYSCTRL3) |
	       N32H7XX_PWR_GRC_FUCEN,
	       n32h7xx_graphics_pwr + N32H7XX_PWR_SYSCTRL3);
	writel(readl(n32h7xx_graphics_pwr + N32H7XX_PWR_SYSCTRL3) |
	       N32H7XX_PWR_GRC_ISNEN,
	       n32h7xx_graphics_pwr + N32H7XX_PWR_SYSCTRL3);

	return 0;
}

static int n32h7xx_graphics_domain_disable(void)
{
	u32 val;

	writel(readl(n32h7xx_graphics_pwr + N32H7XX_PWR_SYSCTRL3) &
	       ~N32H7XX_PWR_GRC_ISNEN,
	       n32h7xx_graphics_pwr + N32H7XX_PWR_SYSCTRL3);
	writel(readl(n32h7xx_graphics_pwr + N32H7XX_PWR_SYSCTRL3) &
	       ~N32H7XX_PWR_GRC_FUCEN,
	       n32h7xx_graphics_pwr + N32H7XX_PWR_SYSCTRL3);
	writel(readl(n32h7xx_graphics_pwr + N32H7XX_PWR_SYSCTRL3) &
	       ~N32H7XX_PWR_GRC_PWREN,
	       n32h7xx_graphics_pwr + N32H7XX_PWR_SYSCTRL3);

	return readl_poll_timeout(n32h7xx_graphics_pwr + N32H7XX_PWR_SYSCTRL3,
				  val, !(val & N32H7XX_PWR_GRC_PWRRDY), 10,
				  N32H7XX_GRAPHICS_TIMEOUT_US);
}

static bool n32h7xx_graphics_domain_last_module(u32 gate)
{
	u32 active;

	/* Match the vendor PWR sequence: other GRAPHICS users keep it powered. */
	active = readl(n32h7xx_graphics_pwr + N32H7XX_PWR_IPMEMCTRLSTS) &
		 N32H7XX_PWR_IPMEMCTRL_GRAPHICS;
	return active == gate;
}

static void n32h7xx_graphics_release(struct device *dev, u32 gate,
				     enum n32h7xx_graphics_module module)
{
	int ret;

	mutex_lock(&n32h7xx_graphics_lock);
	if (WARN_ON_ONCE(!n32h7xx_graphics_users ||
			 !n32h7xx_graphics_module_users[module])) {
		mutex_unlock(&n32h7xx_graphics_lock);
		return;
	}

	if (!--n32h7xx_graphics_module_users[module]) {
		/* Follow the vendor sequence: domain off before its last gate. */
		if (n32h7xx_graphics_users == 1 &&
		    n32h7xx_graphics_domain_last_module(gate)) {
			ret = n32h7xx_graphics_domain_disable();
			if (ret)
				dev_warn(dev, "GRAPHICS power-down timed out: %d\n",
					 ret);
		}
		ret = n32h7xx_graphics_module_disable(gate);
		if (ret)
			dev_warn(dev, "GRAPHICS memory gate 0x%x disable timed out: %d\n",
				 gate, ret);
	}
	n32h7xx_graphics_users--;
	mutex_unlock(&n32h7xx_graphics_lock);
}

static void n32h7xx_graphics_devm_release(void *data)
{
	struct n32h7xx_graphics_ref *ref = data;

	n32h7xx_graphics_release(ref->dev,
				 n32h7xx_graphics_gate(ref->module),
				 ref->module);
}

int n32h7xx_graphics_acquire(struct device *dev,
			     enum n32h7xx_graphics_module module)
{
	struct n32h7xx_graphics_ref *ref;
	u32 gate = n32h7xx_graphics_gate(module);
	int ret;

	if (!gate)
		return -EINVAL;

	ref = devm_kzalloc(dev, sizeof(*ref), GFP_KERNEL);
	if (!ref)
		return -ENOMEM;
	ref->dev = dev;
	ref->module = module;

	mutex_lock(&n32h7xx_graphics_lock);
	if (!n32h7xx_graphics_pwr) {
		n32h7xx_graphics_pwr = ioremap(N32H7XX_PWR_BASE, 0x400);
		if (!n32h7xx_graphics_pwr) {
			ret = -ENOMEM;
			goto unlock;
		}
	}

	if (!n32h7xx_graphics_module_users[module]) {
		ret = n32h7xx_graphics_module_enable(dev, gate);
		if (ret)
			goto unlock;
	}

	ret = n32h7xx_graphics_domain_enable(dev);
	if (ret) {
		if (!n32h7xx_graphics_module_users[module])
			n32h7xx_graphics_module_disable(gate);
		goto unlock;
	}

	n32h7xx_graphics_module_users[module]++;
	n32h7xx_graphics_users++;
	mutex_unlock(&n32h7xx_graphics_lock);

	return devm_add_action_or_reset(dev, n32h7xx_graphics_devm_release, ref);

unlock:
	mutex_unlock(&n32h7xx_graphics_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(n32h7xx_graphics_acquire);
