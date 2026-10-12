// SPDX-License-Identifier: GPL-2.0-only
/*
 * NSING N32H7xx ARMv7-M SysTick clock event driver.
 *
 * SysTick is used as the scheduler clock event source.  The timekeeping core
 * falls back to the jiffies clocksource until a separate free-running timer is
 * added for high-resolution timekeeping.
 */

#include <linux/bitops.h>
#include <linux/clockchips.h>
#include <linux/cpumask.h>
#include <linux/hardirq.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/of.h>
#include <linux/of_address.h>

#include <asm/irq_regs.h>
#include <asm/delay.h>
#include <asm/ptrace.h>

#define N32H7XX_SYST_CSR		0x00
#define N32H7XX_SYST_RVR		0x04
#define N32H7XX_SYST_CVR		0x08

#define N32H7XX_SYST_CSR_ENABLE		BIT(0)
#define N32H7XX_SYST_CSR_TICKINT		BIT(1)
#define N32H7XX_SYST_CSR_CLKSOURCE	BIT(2)

#define N32H7XX_SYST_RVR_MASK		0x00ffffff

#define N32H7XX_DEMCR			0xe000edfc
#define N32H7XX_DWT_CTRL		0xe0001000
#define N32H7XX_DWT_CYCCNT		0xe0001004

#define N32H7XX_DEMCR_TRCENA		BIT(24)
#define N32H7XX_DWT_CTRL_CYCCNTENA	BIT(0)
#define N32H7XX_DWT_CTRL_NOCYCCNT	BIT(25)

static void __iomem *n32h7xx_systick_base;
static u32 n32h7xx_systick_rate;

static unsigned long notrace n32h7xx_read_current_timer(void)
{
	return readl_relaxed((void __iomem *)N32H7XX_DWT_CYCCNT);
}

static struct delay_timer n32h7xx_delay_timer = {
	.read_current_timer = n32h7xx_read_current_timer,
};

static int __init n32h7xx_dwt_init(u32 rate)
{
	void __iomem *demcr = (void __iomem *)N32H7XX_DEMCR;
	void __iomem *dwt_ctrl = (void __iomem *)N32H7XX_DWT_CTRL;
	u32 ctrl;

	ctrl = readl_relaxed(dwt_ctrl);
	if (ctrl & N32H7XX_DWT_CTRL_NOCYCCNT)
		return -ENODEV;

	writel_relaxed(readl_relaxed(demcr) | N32H7XX_DEMCR_TRCENA, demcr);
	writel_relaxed(0, (void __iomem *)N32H7XX_DWT_CYCCNT);
	writel_relaxed(ctrl | N32H7XX_DWT_CTRL_CYCCNTENA, dwt_ctrl);

	n32h7xx_delay_timer.freq = rate;
	register_current_timer_delay(&n32h7xx_delay_timer);
	return 0;
}

void armv7m_systick_handle_irq(struct pt_regs *regs);

static int n32h7xx_systick_shutdown(struct clock_event_device *evt)
{
	writel_relaxed(0, n32h7xx_systick_base + N32H7XX_SYST_CSR);
	writel_relaxed(0, n32h7xx_systick_base + N32H7XX_SYST_RVR);
	writel_relaxed(0, n32h7xx_systick_base + N32H7XX_SYST_CVR);
	return 0;
}

static int n32h7xx_systick_set_periodic(struct clock_event_device *evt)
{
	unsigned long cycles = DIV_ROUND_CLOSEST(n32h7xx_systick_rate, HZ);

	if (cycles < 2 || cycles > N32H7XX_SYST_RVR_MASK + 1)
		return -EINVAL;

	writel_relaxed(0, n32h7xx_systick_base + N32H7XX_SYST_CSR);
	writel_relaxed(cycles - 1, n32h7xx_systick_base + N32H7XX_SYST_RVR);
	writel_relaxed(0, n32h7xx_systick_base + N32H7XX_SYST_CVR);
	writel_relaxed(N32H7XX_SYST_CSR_ENABLE | N32H7XX_SYST_CSR_TICKINT |
		       N32H7XX_SYST_CSR_CLKSOURCE,
		       n32h7xx_systick_base + N32H7XX_SYST_CSR);
	return 0;
}

static int n32h7xx_systick_set_oneshot(struct clock_event_device *evt)
{
	return n32h7xx_systick_shutdown(evt);
}

static int n32h7xx_systick_set_next_event(unsigned long delta,
					struct clock_event_device *evt)
{
	if (delta < 2)
		return -ETIME;
	if (delta > N32H7XX_SYST_RVR_MASK + 1)
		return -EINVAL;

	writel_relaxed(0, n32h7xx_systick_base + N32H7XX_SYST_CSR);
	writel_relaxed(delta - 1, n32h7xx_systick_base + N32H7XX_SYST_RVR);
	writel_relaxed(0, n32h7xx_systick_base + N32H7XX_SYST_CVR);
	writel_relaxed(N32H7XX_SYST_CSR_ENABLE | N32H7XX_SYST_CSR_TICKINT |
		       N32H7XX_SYST_CSR_CLKSOURCE,
		       n32h7xx_systick_base + N32H7XX_SYST_CSR);
	return 0;
}

static struct clock_event_device n32h7xx_systick_clockevent = {
	.name			= "n32h7xx_systick",
	.features		= CLOCK_EVT_FEAT_PERIODIC |
				  CLOCK_EVT_FEAT_ONESHOT,
	.rating			= 300,
	.set_state_shutdown	= n32h7xx_systick_shutdown,
	.set_state_periodic	= n32h7xx_systick_set_periodic,
	.set_state_oneshot	= n32h7xx_systick_set_oneshot,
	.set_state_oneshot_stopped = n32h7xx_systick_shutdown,
	.set_next_event		= n32h7xx_systick_set_next_event,
};

void armv7m_systick_handle_irq(struct pt_regs *regs)
{
	struct pt_regs *old_regs;

	if (!n32h7xx_systick_base)
		return;

	/* Reading CSR clears COUNTFLAG and acknowledges the SysTick exception. */
	readl_relaxed(n32h7xx_systick_base + N32H7XX_SYST_CSR);
	/* SysTick reloads automatically, including in oneshot mode. */
	if (clockevent_state_oneshot(&n32h7xx_systick_clockevent))
		writel_relaxed(0, n32h7xx_systick_base + N32H7XX_SYST_CSR);

	irq_enter();
	old_regs = set_irq_regs(regs);
	if (n32h7xx_systick_clockevent.event_handler)
		n32h7xx_systick_clockevent.event_handler(&n32h7xx_systick_clockevent);
	set_irq_regs(old_regs);
	irq_exit();
}

static int __init n32h7xx_systick_init(struct device_node *np)
{
	void __iomem *base;
	u32 rate;
	int ret;

	base = of_iomap(np, 0);
	if (!base) {
		pr_warn("n32h7xx-systick: invalid base address\n");
		return -ENXIO;
	}

	ret = of_property_read_u32(np, "clock-frequency", &rate);
	if (ret || !rate) {
		pr_warn("n32h7xx-systick: invalid clock-frequency\n");
		iounmap(base);
		return ret ?: -EINVAL;
	}

	n32h7xx_systick_base = base;
	n32h7xx_systick_rate = rate;

	if (n32h7xx_dwt_init(rate))
		pr_warn("n32h7xx: DWT cycle counter unavailable, using jiffies entropy fallback\n");

	n32h7xx_systick_clockevent.cpumask = cpumask_of(0);
	clockevents_config_and_register(&n32h7xx_systick_clockevent, rate, 2,
					N32H7XX_SYST_RVR_MASK + 1);
	return 0;
}

TIMER_OF_DECLARE(n32h7xx_systick, "nsing,n32h7xx-systick", n32h7xx_systick_init);
