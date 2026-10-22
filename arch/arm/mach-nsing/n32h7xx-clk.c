// SPDX-License-Identifier: GPL-2.0-only
/* N32H7xx console, timekeeping, USB and audio clocks. */
#include <linux/bitops.h>
#include <linux/bitfield.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/reset-controller.h>
#include <linux/spinlock.h>

#define N32H7XX_RCC_PLL2CTRL1 0x10
#define N32H7XX_RCC_PLL2CTRL2 0x14
#define N32H7XX_RCC_PLL3CTRL1 0x20
#define N32H7XX_RCC_SRCCTRL1  0x30
#define N32H7XX_RCC_PLL2DIV   0x38
#define N32H7XX_RCC_AXIDIV1   0x138
#define N32H7XX_RCC_AXISEL1   0x13c

#define N32H7XX_APB1DIV1 0x74
#define N32H7XX_APB1EN3  0x88
#define N32H7XX_APB1EN1 0x80
#define N32H7XX_APB1RST1 0x94
#define N32H7XX_NUM_CLOCKS 15

#define N32H7XX_PLL_SRC_MASK		GENMASK(29, 28)
#define N32H7XX_PLL_SRC_HSE		GENMASK(29, 28)
#define N32H7XX_PLL_LOCK		BIT(20)
#define N32H7XX_PLL_LDO		BIT(19)
#define N32H7XX_PLL_ENABLE		BIT(18)
#define N32H7XX_PLL_RESET		BIT(17)
#define N32H7XX_PLL_POWER_DOWN		BIT(16)
#define N32H7XX_PLL_BWAJ		GENMASK(11, 0)
#define N32H7XX_PLL2_A_DIV		GENMASK(5, 0)
#define N32H7XX_PLL2_C_DIV		GENMASK(21, 16)
#define N32H7XX_HSE_READY		BIT(3)
#define N32H7XX_HSE_ENABLE		BIT(2)

#define N32H7XX_PLL2_500M_CLKF		(20U * 16384)
#define N32H7XX_PLL2_A_DIV5		5U
#define N32H7XX_PLL2_C_DIV20		20U

#define N32H7XX_AXIDIV1_LCDCAXIDIV	GENMASK(15, 12)
#define N32H7XX_AXISEL1_LCDCKERSEL	GENMASK(13, 12)
#define N32H7XX_LCDCKERSEL_PLL2C	2U
#define N32H7XX_LCD_PIXEL_RATE_HZ	25000000UL

static DEFINE_SPINLOCK(n32h7xx_rcc_lock);

struct n32h7xx_periph_clk {
	struct clk_hw hw;
	void __iomem *base;
	u32 enable_reg;
	u32 divider_reg;
	u32 select_reg;
	u8 divider_shift;
	u8 gate_shift;
	u8 select_shift;
	bool bus_clock;
	bool i2s_clock;
	bool pixel_clock;
};

#define to_periph_clk(hw) container_of(hw, struct n32h7xx_periph_clk, hw)

static int n32h7xx_lcdc_pll2_init(void __iomem *base)
{
	u32 ctrl, reg;
	int ret;

	/* PLL2's divider values below assume the HMI board's 25 MHz HSE. */
	if (!of_machine_is_compatible("nsing,n32h787-hmi"))
		return -EOPNOTSUPP;

	ctrl = readl(base + N32H7XX_RCC_PLL2CTRL1);
	if (ctrl & N32H7XX_PLL_ENABLE) {
		/* PLL2A is shared with SDMMC; never retune a running PLL. */
		if ((ctrl & N32H7XX_PLL_SRC_MASK) != N32H7XX_PLL_SRC_HSE ||
		    readl(base + N32H7XX_RCC_PLL2CTRL2) != N32H7XX_PLL2_500M_CLKF ||
		    FIELD_GET(N32H7XX_PLL2_A_DIV,
			      readl(base + N32H7XX_RCC_PLL2DIV)) !=
			      N32H7XX_PLL2_A_DIV5)
			return -EBUSY;
		ret = readl_poll_timeout(base + N32H7XX_RCC_PLL2CTRL1, reg,
					 reg & N32H7XX_PLL_LOCK, 10, 100000);
		if (ret)
			return ret;
	} else {
		reg = readl(base + N32H7XX_RCC_SRCCTRL1);
		writel(reg | N32H7XX_HSE_ENABLE, base + N32H7XX_RCC_SRCCTRL1);
		ret = readl_poll_timeout(base + N32H7XX_RCC_SRCCTRL1, reg,
					 reg & N32H7XX_HSE_READY, 10, 100000);
		if (ret)
			return ret;

		ctrl &= ~(N32H7XX_PLL_SRC_MASK | N32H7XX_PLL_BWAJ);
		ctrl |= N32H7XX_PLL_SRC_HSE | N32H7XX_PLL_RESET |
			N32H7XX_PLL_POWER_DOWN |
			FIELD_PREP(N32H7XX_PLL_BWAJ, 9);
		writel(ctrl, base + N32H7XX_RCC_PLL2CTRL1);
		writel(N32H7XX_PLL2_500M_CLKF, base + N32H7XX_RCC_PLL2CTRL2);

		/* The vendor sequence enables the shared PLL supply in PLL3CTRL1. */
		reg = readl(base + N32H7XX_RCC_PLL3CTRL1);
		writel(reg | N32H7XX_PLL_LDO, base + N32H7XX_RCC_PLL3CTRL1);
		udelay(10);
		ctrl &= ~N32H7XX_PLL_POWER_DOWN;
		writel(ctrl, base + N32H7XX_RCC_PLL2CTRL1);
		udelay(10);
		ctrl &= ~N32H7XX_PLL_RESET;
		writel(ctrl, base + N32H7XX_RCC_PLL2CTRL1);
		ret = readl_poll_timeout(base + N32H7XX_RCC_PLL2CTRL1, reg,
					 reg & N32H7XX_PLL_LOCK, 10, 100000);
		if (ret) {
			writel(ctrl | N32H7XX_PLL_RESET |
			       N32H7XX_PLL_POWER_DOWN,
			       base + N32H7XX_RCC_PLL2CTRL1);
			return ret;
		}
		writel(ctrl | N32H7XX_PLL_ENABLE,
		       base + N32H7XX_RCC_PLL2CTRL1);

		reg = readl(base + N32H7XX_RCC_PLL2DIV);
		reg = (reg & ~N32H7XX_PLL2_A_DIV) |
			FIELD_PREP(N32H7XX_PLL2_A_DIV, N32H7XX_PLL2_A_DIV5);
		writel(reg, base + N32H7XX_RCC_PLL2DIV);
	}

	/* PLL2C = 500 MHz / 20 = 25 MHz, inside the panel's 23-27 MHz range. */
	reg = readl(base + N32H7XX_RCC_PLL2DIV);
	reg = (reg & ~N32H7XX_PLL2_C_DIV) |
		FIELD_PREP(N32H7XX_PLL2_C_DIV, N32H7XX_PLL2_C_DIV20);
	writel(reg, base + N32H7XX_RCC_PLL2DIV);
	if (FIELD_GET(N32H7XX_PLL2_C_DIV,
		      readl(base + N32H7XX_RCC_PLL2DIV)) !=
	    N32H7XX_PLL2_C_DIV20)
		return -EIO;

	return 0;
}

static int n32h7xx_periph_clk_enable(struct clk_hw *hw)
{
	struct n32h7xx_periph_clk *clk = to_periph_clk(hw);
	void __iomem *reg = clk->base + clk->enable_reg;
	unsigned long flags;
	int ret;

	if (clk->pixel_clock) {
		ret = n32h7xx_lcdc_pll2_init(clk->base);
		if (ret)
			return ret;
	}

	spin_lock_irqsave(&n32h7xx_rcc_lock, flags);
	if (clk->pixel_clock) {
		/* Keep AXI/8 as the fallback, then select the 25 MHz PLL2C path. */
		writel((readl(clk->base + N32H7XX_RCC_AXIDIV1) &
			~N32H7XX_AXIDIV1_LCDCAXIDIV) | (4 << 12),
		       clk->base + N32H7XX_RCC_AXIDIV1);
		writel((readl(clk->base + N32H7XX_RCC_AXISEL1) &
			~N32H7XX_AXISEL1_LCDCKERSEL) |
			FIELD_PREP(N32H7XX_AXISEL1_LCDCKERSEL,
				   N32H7XX_LCDCKERSEL_PLL2C),
		       clk->base + N32H7XX_RCC_AXISEL1);
		spin_unlock_irqrestore(&n32h7xx_rcc_lock, flags);
		return 0;
	}
	if (clk->i2s_clock) {
		/* I2S kernel clocks use the sysbus / 8 path. */
		writel(readl(clk->base + clk->select_reg) &
		       ~(3U << clk->select_shift), clk->base + clk->select_reg);
		writel((readl(clk->base + clk->divider_reg) &
			~(7U << clk->divider_shift)) | (6U << clk->divider_shift),
		       clk->base + clk->divider_reg);
	}
	writel(readl(reg) | BIT(clk->gate_shift), reg);
	readl(reg);
	/* The LP gate is valid only after enabling the run gate. */
	writel(readl(reg) | BIT(clk->gate_shift - 2), reg);
	readl(reg);
	spin_unlock_irqrestore(&n32h7xx_rcc_lock, flags);
	return 0;
}

static void n32h7xx_periph_clk_disable(struct clk_hw *hw)
{
	struct n32h7xx_periph_clk *clk = to_periph_clk(hw);
	void __iomem *reg = clk->base + clk->enable_reg;
	unsigned long flags;

	if (clk->pixel_clock)
		return;

	spin_lock_irqsave(&n32h7xx_rcc_lock, flags);
	writel(readl(reg) & ~(BIT(clk->gate_shift) |
			     BIT(clk->gate_shift - 2)), reg);
	readl(reg);
	spin_unlock_irqrestore(&n32h7xx_rcc_lock, flags);
}

static int n32h7xx_periph_clk_is_enabled(struct clk_hw *hw)
{
	struct n32h7xx_periph_clk *clk = to_periph_clk(hw);
	u32 val = readl(clk->base + clk->enable_reg);

	u32 mask = BIT(clk->gate_shift) | BIT(clk->gate_shift - 2);

	return clk->pixel_clock || (val & mask) == mask;
}

static unsigned long n32h7xx_periph_clk_recalc_rate(struct clk_hw *hw,
						 unsigned long parent_rate)
{
	struct n32h7xx_periph_clk *clk = to_periph_clk(hw);
	u32 div;

	if (clk->pixel_clock)
		return N32H7XX_LCD_PIXEL_RATE_HZ;

	if (clk->bus_clock)
		return parent_rate;

	div = (readl(clk->base + clk->divider_reg) >>
	       clk->divider_shift) & 7;

	/* Encodings 0..3 divide by one, 4..7 divide by 2, 4, 8, 16. */
	return parent_rate >> (div < 4 ? 0 : div - 3);
}

static const struct clk_ops n32h7xx_periph_clk_ops = {
	.enable = n32h7xx_periph_clk_enable,
	.disable = n32h7xx_periph_clk_disable,
	.is_enabled = n32h7xx_periph_clk_is_enabled,
	.recalc_rate = n32h7xx_periph_clk_recalc_rate,
};

struct n32h7xx_rcc {
	struct n32h7xx_periph_clk clocks[N32H7XX_NUM_CLOCKS];
	struct clk_hw_onecell_data *data;
	struct reset_controller_dev resets;
};

static int n32h7xx_reset_update(struct reset_controller_dev *rcdev,
			      unsigned long id, bool assert)
{
	struct n32h7xx_rcc *rcc = container_of(rcdev, struct n32h7xx_rcc, resets);
	static const u32 regs[] = { N32H7XX_APB1RST1, 0xb4, 0x64,
				   0xb4, 0x64, 0xd8, 0xa0, 0x6c, 0x64,
				   0x150, 0x150, 0x154, 0, 0x158, 0 };
	static const u32 masks[] = { BIT(28), BIT(20), BIT(20),
				    BIT(21), BIT(21), BIT(28), BIT(24),
				    BIT(16), BIT(16), BIT(20), BIT(28),
				    BIT(24), 0, BIT(0), 0 };
	void __iomem *reg;
	unsigned long flags;
	u32 val;

	if (id >= ARRAY_SIZE(regs) || !masks[id])
		return -EINVAL;
	reg = rcc->clocks[0].base + regs[id];
	spin_lock_irqsave(&n32h7xx_rcc_lock, flags);
	val = readl(reg);
	writel(assert ? val | masks[id] : val & ~masks[id], reg);
	readl(reg);
	spin_unlock_irqrestore(&n32h7xx_rcc_lock, flags);
	return 0;
}

static int n32h7xx_reset_assert(struct reset_controller_dev *rcdev,
			      unsigned long id)
{
	return n32h7xx_reset_update(rcdev, id, true);
}

static int n32h7xx_reset_deassert(struct reset_controller_dev *rcdev,
				unsigned long id)
{
	return n32h7xx_reset_update(rcdev, id, false);
}

static int n32h7xx_reset(struct reset_controller_dev *rcdev, unsigned long id)
{
	int ret = n32h7xx_reset_assert(rcdev, id);

	if (ret)
		return ret;
	udelay(2);
	return n32h7xx_reset_deassert(rcdev, id);
}

static const struct reset_control_ops n32h7xx_reset_ops = {
	.assert = n32h7xx_reset_assert,
	.deassert = n32h7xx_reset_deassert,
	.reset = n32h7xx_reset,
};

static void __init n32h7xx_clk_init(struct device_node *np)
{
	const char *parent = of_clk_get_parent_name(np, 0);
	const char *hse = of_clk_get_parent_name(np, 1);
	const char *axi = of_clk_get_parent_name(np, 2);
	static const char * const names[] = { "n32h7xx-usart1", "n32h7xx-btim1",
					    "n32h7xx-usb1", "n32h7xx-usb2",
					    NULL, "n32h7xx-dma1", "n32h7xx-dmamux1",
					    "n32h7xx-i2s1", "n32h7xx-i2s4",
					    "n32h7xx-jpeg-encoder",
					    "n32h7xx-jpeg-decoder", "n32h7xx-lcdc",
					    "n32h7xx-lcdc-apb", "n32h7xx-gpu",
					    "n32h7xx-lcdc-pixel" };
	static const u32 enable_regs[] = { 0x88, 0x80, 0xb0, 0x54, 0,
					  0x5c, 0x54, 0xc8, 0x8c, 0x140, 0x140,
					  0x144, 0x144, 0x148, 0x144 };
	static const u8 gate_shifts[] = { 31, 31, 23, 23, 0, 19, 19, 31, 27,
					 23, 31, 27, 23, 3, 27 };
	struct clk_init_data init = {
		.ops = &n32h7xx_periph_clk_ops,
		.parent_names = &parent,
		.num_parents = 1,
	};
	struct n32h7xx_rcc *rcc;
	void __iomem *base;
	int i, ret;

	if (!parent)
		return;
	rcc = kzalloc_obj(*rcc);
	if (!rcc)
		return;
	rcc->data = kzalloc(struct_size(rcc->data, hws, N32H7XX_NUM_CLOCKS),
			    GFP_KERNEL);
	if (!rcc->data)
		goto free;
	base = of_iomap(np, 0);
	if (!base)
		goto free_data;
	rcc->data->num = N32H7XX_NUM_CLOCKS;
	for (i = 0; i < ARRAY_SIZE(names); i++) {
		struct n32h7xx_periph_clk *clk = &rcc->clocks[i];

		if (!names[i])
			continue;
		clk->base = base;
		clk->enable_reg = enable_regs[i];
		clk->divider_reg = N32H7XX_APB1DIV1;
		clk->divider_shift = i ? 24 : 28;
		clk->gate_shift = gate_shifts[i];
		clk->bus_clock = (i >= 2 && i < 7) || i >= 9;
		if (i == 7 || i == 8) {
			clk->i2s_clock = true;
			clk->divider_reg = i == 7 ? 0xb8 : 0x74;
			clk->divider_shift = i == 7 ? 16 : 8;
			clk->select_reg = i == 7 ? 0xbc : 0x78;
			clk->select_shift = i == 7 ? 0 : 2;
		}
		clk->pixel_clock = i == 14;
		init.parent_names = i >= 11 && axi ? &axi : &parent;
		init.name = names[i];
		clk->hw.init = &init;
		/* Preserve early console gates before registering its clock. */
		if (!i)
			n32h7xx_periph_clk_enable(&clk->hw);
		ret = clk_hw_register(NULL, &clk->hw);
		if (ret)
			goto unregister;
		rcc->data->hws[i] = &clk->hw;
	}
	if (hse) {
		/* Feed the PHY PLL directly from HSE, without prescaling. */
		writel(readl(base + 0xa8) & ~GENMASK(3, 0), base + 0xa8);
		readl(base + 0xa8);
		rcc->data->hws[4] = clk_hw_register_fixed_factor(NULL,
					"n32h7xx-usb-ref", hse, 0, 1, 1);
		if (IS_ERR(rcc->data->hws[4]))
			goto unregister;
	} else {
		rcc->data->hws[4] = ERR_PTR(-ENOENT);
	}
	rcc->resets.ops = &n32h7xx_reset_ops;
	rcc->resets.of_node = np;
	rcc->resets.nr_resets = N32H7XX_NUM_CLOCKS;
	ret = reset_controller_register(&rcc->resets);
	if (ret)
		goto unregister;
	ret = of_clk_add_hw_provider(np, of_clk_hw_onecell_get, rcc->data);
	if (!ret)
		return;
	reset_controller_unregister(&rcc->resets);
unregister:
	if (!IS_ERR_OR_NULL(rcc->data->hws[4]))
		clk_hw_unregister_fixed_factor(rcc->data->hws[4]);
	while (i--)
		if (i != 4)
			clk_hw_unregister(&rcc->clocks[i].hw);
	iounmap(base);
free_data:
	kfree(rcc->data);
free:
	kfree(rcc);
	pr_err("%pOF: failed to register RCC clocks and resets\n", np);
}
CLK_OF_DECLARE(n32h7xx_clk, "nsing,n32h7xx-clock", n32h7xx_clk_init);
