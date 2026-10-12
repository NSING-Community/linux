// SPDX-License-Identifier: GPL-2.0-only
/* N32H7xx console and timekeeping clocks. */
#include <linux/bitops.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/reset-controller.h>

#define N32H7XX_APB1DIV1 0x74
#define N32H7XX_APB1EN3  0x88
#define N32H7XX_PERIPH_GATES (BIT(31) | BIT(29))
#define N32H7XX_APB1EN1 0x80
#define N32H7XX_APB1RST1 0x94
#define N32H7XX_BTIM1_RESET BIT(28)
#define N32H7XX_NUM_CLOCKS 2

struct n32h7xx_periph_clk {
	struct clk_hw hw;
	void __iomem *base;
	u32 enable_reg;
	u8 divider_shift;
};

#define to_periph_clk(hw) container_of(hw, struct n32h7xx_periph_clk, hw)

static int n32h7xx_periph_clk_enable(struct clk_hw *hw)
{
	struct n32h7xx_periph_clk *clk = to_periph_clk(hw);
	void __iomem *reg = clk->base + clk->enable_reg;

	writel(readl(reg) | BIT(31), reg);
	readl(reg);
	/* The LP gate is valid only after enabling the run gate. */
	writel(readl(reg) | BIT(29), reg);
	readl(reg);
	return 0;
}

static void n32h7xx_periph_clk_disable(struct clk_hw *hw)
{
	struct n32h7xx_periph_clk *clk = to_periph_clk(hw);
	void __iomem *reg = clk->base + clk->enable_reg;

	writel(readl(reg) & ~N32H7XX_PERIPH_GATES, reg);
	readl(reg);
}

static int n32h7xx_periph_clk_is_enabled(struct clk_hw *hw)
{
	struct n32h7xx_periph_clk *clk = to_periph_clk(hw);
	u32 val = readl(clk->base + clk->enable_reg);

	return (val & N32H7XX_PERIPH_GATES) == N32H7XX_PERIPH_GATES;
}

static unsigned long n32h7xx_periph_clk_recalc_rate(struct clk_hw *hw,
						 unsigned long parent_rate)
{
	struct n32h7xx_periph_clk *clk = to_periph_clk(hw);
	u32 div = (readl(clk->base + N32H7XX_APB1DIV1) >>
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

static int n32h7xx_btim1_reset(struct reset_controller_dev *rcdev,
			     unsigned long id)
{
	struct n32h7xx_rcc *rcc = container_of(rcdev, struct n32h7xx_rcc, resets);
	void __iomem *reg = rcc->clocks[1].base + N32H7XX_APB1RST1;

	if (id)
		return -EINVAL;
	writel(readl(reg) | N32H7XX_BTIM1_RESET, reg);
	readl(reg);
	udelay(2);
	writel(readl(reg) & ~N32H7XX_BTIM1_RESET, reg);
	readl(reg);
	return 0;
}

static const struct reset_control_ops n32h7xx_reset_ops = {
	.reset = n32h7xx_btim1_reset,
};

static void __init n32h7xx_clk_init(struct device_node *np)
{
	const char *parent = of_clk_get_parent_name(np, 0);
	static const char * const names[] = { "n32h7xx-usart1", "n32h7xx-btim1" };
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
	for (i = 0; i < N32H7XX_NUM_CLOCKS; i++) {
		struct n32h7xx_periph_clk *clk = &rcc->clocks[i];

		clk->base = base;
		clk->enable_reg = i ? N32H7XX_APB1EN1 : N32H7XX_APB1EN3;
		clk->divider_shift = i ? 24 : 28;
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
	rcc->resets.ops = &n32h7xx_reset_ops;
	rcc->resets.of_node = np;
	rcc->resets.nr_resets = 1;
	ret = reset_controller_register(&rcc->resets);
	if (ret)
		goto unregister;
	ret = of_clk_add_hw_provider(np, of_clk_hw_onecell_get, rcc->data);
	if (!ret)
		return;
	reset_controller_unregister(&rcc->resets);
unregister:
	while (i--)
		clk_hw_unregister(&rcc->clocks[i].hw);
	iounmap(base);
free_data:
	kfree(rcc->data);
free:
	kfree(rcc);
	pr_err("%pOF: failed to register RCC clocks and resets\n", np);
}
CLK_OF_DECLARE(n32h7xx_clk, "nsing,n32h7xx-clock", n32h7xx_clk_init);
