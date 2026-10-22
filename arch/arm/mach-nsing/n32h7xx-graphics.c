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
#include <linux/irqflags.h>
#include <linux/log2.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/slab.h>
#include <linux/sizes.h>
#include <linux/soc/nsing/n32h7xx-graphics.h>

#include <asm/cacheflush.h>
#include <asm/v7m.h>

#define N32H7XX_PWR_BASE		0x58031000
#define N32H7XX_PWR_SYSCTRL3		0x4c
#define N32H7XX_PWR_IPMEMCTRL		0x58
#define N32H7XX_PWR_IPMEMCTRLSTS	0x5c

#define N32H7XX_PWR_IPMEMCTRL_JPEG	BIT(2)
#define N32H7XX_PWR_IPMEMCTRL_LCDC	BIT(1)
#define N32H7XX_PWR_IPMEMCTRL_GPU	BIT(0)
#define N32H7XX_PWR_IPMEMCTRL_DVP	BIT(4)
#define N32H7XX_PWR_IPMEMCTRL_GRAPHICS	GENMASK(4, 0)

#define N32H7XX_AXI_LCDC_RQOS		0x51045100
#define N32H7XX_AXI_LCDC_RQOS_HIGH	0xf

#define N32H7XX_PWR_GRC_PWREN		BIT(0)
#define N32H7XX_PWR_GRC_FUCEN		BIT(4)
#define N32H7XX_PWR_GRC_ISNEN		BIT(8)
#define N32H7XX_PWR_GRC_PWRRDY		BIT(16)
#define N32H7XX_PWR_GRC_PSWACK1		BIT(20)

#define N32H7XX_GRAPHICS_TIMEOUT_US	100000

static DEFINE_MUTEX(n32h7xx_graphics_lock);
static void __iomem *n32h7xx_graphics_pwr;
static void __iomem *n32h7xx_graphics_rcc;
static void __iomem *n32h7xx_graphics_mmu;
static void __iomem *n32h7xx_graphics_axi_lcdc_rqos;
static unsigned int n32h7xx_graphics_users;
static unsigned int n32h7xx_graphics_module_users[N32H7XX_GRAPHICS_MODULE_COUNT];

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
	case N32H7XX_GRAPHICS_LCDC:
		return N32H7XX_PWR_IPMEMCTRL_LCDC;
	case N32H7XX_GRAPHICS_GPU:
		return N32H7XX_PWR_IPMEMCTRL_GPU;
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

static void n32h7xx_graphics_set_lcdc_qos(struct device *dev)
{
	if (!n32h7xx_graphics_axi_lcdc_rqos) {
		n32h7xx_graphics_axi_lcdc_rqos =
			ioremap(N32H7XX_AXI_LCDC_RQOS, sizeof(u32));
		if (!n32h7xx_graphics_axi_lcdc_rqos) {
			dev_warn(dev, "cannot map LCDC read QoS register\n");
			return;
		}
	}

	/* Keep LCDC scanout reads ahead of GPU writes to SDRAM. */
	writel(N32H7XX_AXI_LCDC_RQOS_HIGH,
	       n32h7xx_graphics_axi_lcdc_rqos);
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

	if (module == N32H7XX_GRAPHICS_LCDC ||
	    module == N32H7XX_GRAPHICS_GPU) {
		u32 offset = module == N32H7XX_GRAPHICS_LCDC ?
			     N32H7XX_MMU_LCDME : N32H7XX_MMU_GPUME;

		if (!n32h7xx_graphics_rcc)
			n32h7xx_graphics_rcc = ioremap(N32H7XX_RCC_BASE, 0x400);
		if (!n32h7xx_graphics_mmu)
			n32h7xx_graphics_mmu = ioremap(N32H7XX_MMU_BASE, 0x400);
		if (!n32h7xx_graphics_rcc || !n32h7xx_graphics_mmu) {
			ret = -ENOMEM;
			if (!n32h7xx_graphics_module_users[module])
				n32h7xx_graphics_module_disable(gate);
			goto unlock;
		}
		/* Shared APB6/M7 memory gateway, then the master's SDRAM view. */
		writel(readl(n32h7xx_graphics_rcc + N32H7XX_RCC_CFG4) |
		       N32H7XX_RCC_CFG4_APB6CLKEN,
		       n32h7xx_graphics_rcc + N32H7XX_RCC_CFG4);
		writel(readl(n32h7xx_graphics_rcc + N32H7XX_RCC_CFG2) |
		       N32H7XX_RCC_CFG2_M7MMU,
		       n32h7xx_graphics_rcc + N32H7XX_RCC_CFG2);
		writel(readl(n32h7xx_graphics_mmu + offset) | N32H7XX_MMU_ME_EN,
		       n32h7xx_graphics_mmu + offset);
	}
	if (module == N32H7XX_GRAPHICS_LCDC)
		n32h7xx_graphics_set_lcdc_qos(dev);

	n32h7xx_graphics_module_users[module]++;
	n32h7xx_graphics_users++;
	mutex_unlock(&n32h7xx_graphics_lock);

	return devm_add_action_or_reset(dev, n32h7xx_graphics_devm_release, ref);

unlock:
	mutex_unlock(&n32h7xx_graphics_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(n32h7xx_graphics_acquire);

/*
 * NOMMU ioremap_wc() is an identity mapping, not a cache-attribute change.
 * Reserve free MPU slots above the bootloader's SDRAM and DTB slots. Keep
 * them for the life of the system: GEM mappings can survive driver unbind.
 *
 * An MPU region has to be naturally aligned to its size, so a pool that is
 * only aligned to part of its size needs more than one region.  The 3 MiB
 * shared pool at 0xC1A00000 is 2 MiB aligned: 2 MiB plus 1 MiB regions
 * cover it.
 * Splitting instead of moving the pool keeps the optional capture pool at
 * the bottom of the reserved band, where disabling it extends main RAM
 * contiguously rather than leaving an island.
 */
#define N32H7XX_GRAPHICS_MPU_CHUNKS	8

static int n32h7xx_graphics_mpu_chunks(const struct reserved_mem *pool,
				       phys_addr_t *addr, size_t *size, int max)
{
	phys_addr_t pos = pool->base;
	size_t left = pool->size;
	int count = 0;

	while (left) {
		size_t chunk = rounddown_pow_of_two(left);

		while (chunk > left || !IS_ALIGNED(pos, chunk))
			chunk >>= 1;
		if (chunk < 32)
			return -EINVAL;
		if (count == max)
			return -E2BIG;
		addr[count] = pos;
		size[count] = chunk;
		count++;
		pos += chunk;
		left -= chunk;
	}

	return count;
}

int n32h7xx_graphics_dma_prepare(struct device *dev)
{
	static struct reserved_mem *graphics_pool;
	struct device_node *node;
	struct reserved_mem *pool;
	phys_addr_t chunk_addr[N32H7XX_GRAPHICS_MPU_CHUNKS];
	size_t chunk_size[N32H7XX_GRAPHICS_MPU_CHUNKS];
	int regions[N32H7XX_GRAPHICS_MPU_CHUNKS];
	unsigned long flags;
	u32 saved_rnr, count;
	int chunks, i, nregions, region, ret = 0;

	if (IS_ENABLED(CONFIG_MMU) || IS_ENABLED(CONFIG_ARM_MPU))
		return dev_err_probe(dev, -EOPNOTSUPP,
				     "graphics DMA requires bootloader-managed MPU\n");
	node = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (!node)
		return -EINVAL;
	pool = of_reserved_mem_lookup(node);
	if (!of_device_is_compatible(node, "shared-dma-pool") ||
	    !of_property_read_bool(node, "no-map"))
		pool = NULL;
	of_node_put(node);
	if (!pool || pool->size < PAGE_SIZE || pool->size > SZ_32M ||
	    !IS_ALIGNED(pool->size, PAGE_SIZE) ||
	    !IS_ALIGNED(pool->base, PAGE_SIZE))
		return dev_err_probe(dev, -EINVAL, "invalid graphics DMA pool geometry\n");

	chunks = n32h7xx_graphics_mpu_chunks(pool, chunk_addr, chunk_size,
					     N32H7XX_GRAPHICS_MPU_CHUNKS);
	if (chunks < 0)
		return dev_err_probe(dev, chunks,
				     "cannot describe graphics DMA pool with MPU regions\n");

	mutex_lock(&n32h7xx_graphics_lock);
	if (graphics_pool) {
		ret = graphics_pool == pool ? 0 : -EINVAL;
		goto unlock;
	}
	if (!(readl(BASEADDR_V7M_SCB + V7M_SCB_CCR) & V7M_SCB_CCR_DC))
		goto ready;
	if (!(readl(BASEADDR_V7M_SCB + MPU_CTRL) & MPU_CTRL_ENABLE)) {
		ret = -EOPNOTSUPP;
		goto unlock;
	}
	count = (readl(BASEADDR_V7M_SCB + MPU_TYPE) >> 8) & 0xff;

	/* No DMA allocation or userspace mapping can touch this pool yet. */
	dmac_flush_range((void *)(unsigned long)pool->base,
			 (void *)(unsigned long)(pool->base + pool->size));
	dsb(sy);
	local_irq_save(flags);
	saved_rnr = readl(BASEADDR_V7M_SCB + PMSAv7_RNR);
	nregions = 0;
	for (region = (int)count - 1; region > 8 && nregions < chunks; region--) {
		u64 base, size;
		u32 rasr;

		writel(region, BASEADDR_V7M_SCB + PMSAv7_RNR);
		rasr = readl(BASEADDR_V7M_SCB + PMSAv7_RASR);
		if (!(rasr & BIT(0))) {
			regions[nregions++] = region;
			continue;
		}
		size = 1ULL << (((rasr >> 1) & 31) + 1);
		base = readl(BASEADDR_V7M_SCB + PMSAv7_RBAR) & GENMASK(31, 5);
		base &= ~(size - 1);
		/*
		 * Every region still to be picked has a lower number, so a
		 * higher-priority region that overlaps any chunk would win.
		 */
		for (i = 0; i < chunks; i++) {
			if (base < chunk_addr[i] + chunk_size[i] &&
			    chunk_addr[i] < base + size) {
				ret = -EBUSY;
				goto restore;
			}
		}
	}
	if (nregions < chunks) {
		ret = -ENOSPC;
	} else {
		for (i = 0; i < chunks; i++) {
			/* XN, full access, TEX=1/C=0/B=0: non-cacheable. */
			u32 attr = BIT(28) | (3 << 24) | BIT(19) |
				   ((ilog2(chunk_size[i]) - 1) << 1) | BIT(0);

			writel(regions[i], BASEADDR_V7M_SCB + PMSAv7_RNR);
			writel(chunk_addr[i], BASEADDR_V7M_SCB + PMSAv7_RBAR);
			writel(attr, BASEADDR_V7M_SCB + PMSAv7_RASR);
			dsb(sy);
			isb();
			if (readl(BASEADDR_V7M_SCB + PMSAv7_RASR) != attr ||
			    (readl(BASEADDR_V7M_SCB + PMSAv7_RBAR) &
			     GENMASK(31, 5)) != chunk_addr[i]) {
				ret = -EIO;
				goto restore;
			}
		}
	}
restore:
	writel(saved_rnr, BASEADDR_V7M_SCB + PMSAv7_RNR);
	local_irq_restore(flags);
	if (ret)
		goto unlock;
	for (i = 0; i < chunks; i++)
		dev_info(dev, "graphics DMA pool %pa+%zu non-cacheable, MPU region %d\n",
			 &chunk_addr[i], chunk_size[i], regions[i]);
ready:
	graphics_pool = pool;
unlock:
	mutex_unlock(&n32h7xx_graphics_lock);
	if (ret)
		dev_err(dev, "cannot establish coherent graphics DMA: %d\n", ret);
	return ret;
}
EXPORT_SYMBOL_GPL(n32h7xx_graphics_dma_prepare);
