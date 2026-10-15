// SPDX-License-Identifier: GPL-2.0-only
/* N32H7xx console, timekeeping, USB and audio clocks. */
#include <linux/bitops.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/reset-controller.h>
#include <linux/spinlock.h>

#define N32H7XX_APB1DIV1 0x74
#define N32H7XX_APB1EN3  0x88
#define N32H7XX_APB1EN1 0x80
#define N32H7XX_APB1RST1 0x94
#define N32H7XX_NUM_CLOCKS 9

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
};

#define to_periph_clk(hw) container_of(hw, struct n32h7xx_periph_clk, hw)

static int n32h7xx_periph_clk_enable(struct clk_hw *hw)
{
	struct n32h7xx_periph_clk *clk = to_periph_clk(hw);
	void __iomem *reg = clk->base + clk->enable_reg;
	unsigned long flags;

	spin_lock_irqsave(&n32h7xx_rcc_lock, flags);
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

	return (val & mask) == mask;
}

static unsigned long n32h7xx_periph_clk_recalc_rate(struct clk_hw *hw,
						 unsigned long parent_rate)
{
	struct n32h7xx_periph_clk *clk = to_periph_clk(hw);
	u32 div;

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
				   0xb4, 0x64, 0xd8, 0xa0, 0x6c, 0x64 };
	static const u32 masks[] = { BIT(28), BIT(20), BIT(20),
				    BIT(21), BIT(21), BIT(28), BIT(24),
				    BIT(16), BIT(16) };
	void __iomem *reg;
	unsigned long flags;
	u32 val;

	if (id >= ARRAY_SIZE(regs))
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
	static const char * const names[] = { "n32h7xx-usart1", "n32h7xx-btim1",
					    "n32h7xx-usb1", "n32h7xx-usb2",
					    NULL, "n32h7xx-dma1", "n32h7xx-dmamux1",
					    "n32h7xx-i2s1", "n32h7xx-i2s4" };
	static const u32 enable_regs[] = { 0x88, 0x80, 0xb0, 0x54, 0,
					  0x5c, 0x54, 0xc8, 0x8c };
	static const u8 gate_shifts[] = { 31, 31, 23, 23, 0, 19, 19, 31, 27 };
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
		clk->bus_clock = i >= 2 && i < 7;
		if (i >= 7) {
			clk->i2s_clock = true;
			clk->divider_reg = i == 7 ? 0xb8 : 0x74;
			clk->divider_shift = i == 7 ? 16 : 8;
			clk->select_reg = i == 7 ? 0xbc : 0x78;
			clk->select_shift = i == 7 ? 0 : 2;
		}
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
	rcc->resets.nr_resets = 9;
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
