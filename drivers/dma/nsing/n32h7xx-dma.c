// SPDX-License-Identifier: GPL-2.0-only
/* N32H7xx AHB DMA: per-channel IRQs and hardware cyclic slave transfers. */
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of_dma.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include "../virt-dma.h"

#define N32_CHANNELS 8
#define N32_PERIODS 64
#define CH_STRIDE 0x58
#define CH_SRC 0x00
#define CH_DST 0x08
#define CH_LLP 0x10
#define CH_CTRL 0x18
#define CH_COUNT 0x1c
#define CH_CFG 0x40
#define CH_CFG_HI 0x44
#define INT_BLOCK 0x2f0
#define INT_ERROR 0x308
#define MASK_BLOCK 0x318
#define MASK_ERROR 0x330
#define CLEAR_BASE 0x338
#define DMA_CFG 0x398
#define DMA_EN 0x3a0

struct n32_lli {
	u32 src, dst, next, ctrl, count;
};

struct n32_desc {
	struct virt_dma_desc vd;
	struct n32_lli *lli;
	dma_addr_t lli_dma, buffer;
	size_t length;
	enum dma_transfer_direction direction;
	bool error;
};

struct n32_dma;
struct n32_chan {
	struct virt_dma_chan vc;
	struct n32_dma *dma;
	struct dma_slave_config config;
	struct n32_desc *active;
	unsigned int id, request;
	int irq;
};

struct n32_dma {
	struct dma_device engine;
	struct n32_chan chan[N32_CHANNELS];
	void __iomem *base, *mux;
	struct clk_bulk_data clocks[2];
	struct dma_pool *pool;
};

static struct n32_chan *to_n32_chan(struct dma_chan *c)
{
	return container_of(c, struct n32_chan, vc.chan);
}

static void n32_clear(struct n32_chan *c)
{
	int i;

	for (i = 0; i < 5; i++)
		writel(BIT(c->id), c->dma->base + CLEAR_BASE + i * 8);
}

static int n32_stop(struct n32_chan *c)
{
	u32 val;
	int ret;

	writel(BIT(c->id + 8), c->dma->base + DMA_EN);
	ret = readl_poll_timeout_atomic(c->dma->base + DMA_EN, val,
					!(val & BIT(c->id)), 1, 10000);
	writel(BIT(c->id + 8), c->dma->base + MASK_BLOCK);
	writel(BIT(c->id + 8), c->dma->base + MASK_ERROR);
	n32_clear(c);
	return ret;
}

static void n32_desc_free(struct virt_dma_desc *vd)
{
	struct n32_desc *d = container_of(vd, struct n32_desc, vd);
	struct n32_chan *c = to_n32_chan(vd->tx.chan);

	dma_pool_free(c->dma->pool, d->lli, d->lli_dma);
	kfree(d);
}

static int n32_config(struct dma_chan *chan, struct dma_slave_config *config)
{
	struct n32_chan *c = to_n32_chan(chan);

	c->config = *config;
	return 0;
}

static struct dma_async_tx_descriptor *
n32_prep_cyclic(struct dma_chan *chan, dma_addr_t buffer, size_t length,
		size_t period, enum dma_transfer_direction direction,
		unsigned long flags)
{
	struct n32_chan *c = to_n32_chan(chan);
	struct n32_desc *d;
	unsigned int width, periods, i;
	u32 ctrl;

	if (direction == DMA_DEV_TO_MEM)
		width = c->config.src_addr_width;
	else if (direction == DMA_MEM_TO_DEV)
		width = c->config.dst_addr_width;
	else
		return NULL;
	if (width != 1 && width != 2 && width != 4)
		return NULL;
	if (!period || !length || length % period || period % width ||
	    buffer % width || period / width > 4095)
		return NULL;
	periods = length / period;
	if (periods < 2 || periods > N32_PERIODS)
		return NULL;
	d = kzalloc_obj(*d, GFP_ATOMIC);
	if (!d)
		return NULL;
	d->lli = dma_pool_alloc(c->dma->pool, GFP_ATOMIC, &d->lli_dma);
	if (!d->lli) {
		kfree(d);
		return NULL;
	}
	d->buffer = buffer;
	d->length = length;
	d->direction = direction;
	/* Burst one, equal source/destination widths and linked addresses. */
	ctrl = BIT(0) | (ilog2(width) << 1) | (ilog2(width) << 4) |
	       BIT(27) | BIT(28);
	if (direction == DMA_DEV_TO_MEM)
		ctrl |= (2U << 9) | (2U << 20);
	else
		ctrl |= (2U << 7) | (1U << 20);
	for (i = 0; i < periods; i++) {
		struct n32_lli *l = &d->lli[i];

		l->src = direction == DMA_DEV_TO_MEM ? c->config.src_addr :
			 buffer + i * period;
		l->dst = direction == DMA_DEV_TO_MEM ? buffer + i * period :
			 c->config.dst_addr;
		l->next = d->lli_dma + ((i + 1) % periods) * sizeof(*l);
		l->ctrl = ctrl;
		l->count = period / width;
	}
	return vchan_tx_prep(&c->vc, &d->vd, flags);
}

static void n32_issue_pending(struct dma_chan *chan)
{
	struct n32_chan *c = to_n32_chan(chan);
	struct virt_dma_desc *vd;
	struct n32_desc *d;
	void __iomem *base = c->dma->base + c->id * CH_STRIDE;
	unsigned long flags;
	u32 cfg;

	spin_lock_irqsave(&c->vc.lock, flags);
	if (!vchan_issue_pending(&c->vc) || c->active)
		goto unlock;
	vd = vchan_next_desc(&c->vc);
	if (!vd)
		goto unlock;
	if (n32_stop(c)) {
		dev_err(c->dma->engine.dev, "channel %u failed to stop\n", c->id);
		goto unlock;
	}
	list_del(&vd->node);
	d = container_of(vd, struct n32_desc, vd);
	c->active = d;
	writel(c->request, c->dma->mux + c->id * 4);
	writel(d->lli[0].src, base + CH_SRC);
	writel(d->lli[0].dst, base + CH_DST);
	/*
	 * With LLPSRCEN and LLPDSTEN both set the channel takes its whole
	 * transfer state - source, destination, control and block size - from
	 * the descriptor that LLP points at, and it fetches that descriptor as
	 * soon as the channel is enabled.  LLP therefore has to hold the first
	 * descriptor, not the second one: pointing it at lli[0].next starts the
	 * ring one period in, so the very first status read reports hw_ptr a
	 * full period past appl_ptr and the core flags an immediate xrun.
	 */
	writel(d->lli_dma, base + CH_LLP);
	writel(d->lli[0].ctrl, base + CH_CTRL);
	writel(d->lli[0].count, base + CH_COUNT);
	cfg = 7U << 5;
	if (d->direction == DMA_DEV_TO_MEM)
		cfg |= BIT(10); /* Destination software handshake. */
	else
		cfg |= BIT(11); /* Source software handshake. */
	writel(cfg, base + CH_CFG);
	writel(d->direction == DMA_DEV_TO_MEM ? c->id << 7 : c->id << 11,
	       base + CH_CFG_HI);
	n32_clear(c);
	writel(BIT(c->id + 8) | BIT(c->id), c->dma->base + MASK_BLOCK);
	writel(BIT(c->id + 8) | BIT(c->id), c->dma->base + MASK_ERROR);
	dma_wmb();
	writel(BIT(c->id + 8) | BIT(c->id), c->dma->base + DMA_EN);
unlock:
	spin_unlock_irqrestore(&c->vc.lock, flags);
}

static irqreturn_t n32_irq(int irq, void *arg)
{
	struct n32_chan *c = arg;
	struct n32_dma *dma = c->dma;
	u32 block = readl(dma->base + INT_BLOCK) & BIT(c->id);
	u32 error = readl(dma->base + INT_ERROR) & BIT(c->id);
	unsigned long flags;

	if (!block && !error)
		return IRQ_NONE;
	spin_lock_irqsave(&c->vc.lock, flags);
	n32_clear(c);
	if (error && c->active) {
		c->active->error = true;
		n32_stop(c);
		dev_err_ratelimited(dma->engine.dev, "channel %u DMA error\n", c->id);
	} else if (block && c->active) {
		vchan_cyclic_callback(&c->active->vd);
	}
	spin_unlock_irqrestore(&c->vc.lock, flags);
	return IRQ_HANDLED;
}

static int n32_terminate_all(struct dma_chan *chan)
{
	struct n32_chan *c = to_n32_chan(chan);
	unsigned long flags;
	LIST_HEAD(head);
	int ret;

	spin_lock_irqsave(&c->vc.lock, flags);
	ret = n32_stop(c);
	if (!ret) {
		list_splice_tail_init(&c->vc.desc_allocated, &head);
		list_splice_tail_init(&c->vc.desc_submitted, &head);
		list_splice_tail_init(&c->vc.desc_issued, &head);
		list_splice_tail_init(&c->vc.desc_completed, &head);
		/* Cyclic callbacks must drain before their descriptor is freed. */
		if (c->active) {
			vchan_terminate_vdesc(&c->active->vd);
			c->active = NULL;
		}
	}
	spin_unlock_irqrestore(&c->vc.lock, flags);
	vchan_dma_desc_free_list(&c->vc, &head);
	return ret;
}

static void n32_synchronize(struct dma_chan *chan)
{
	vchan_synchronize(&to_n32_chan(chan)->vc);
}

static enum dma_status n32_status(struct dma_chan *chan, dma_cookie_t cookie,
				  struct dma_tx_state *state)
{
	struct n32_chan *c = to_n32_chan(chan);
	struct n32_desc *d;
	unsigned long flags;
	enum dma_status status;
	dma_addr_t position;

	status = dma_cookie_status(chan, cookie, state);
	if (status == DMA_COMPLETE)
		return status;
	spin_lock_irqsave(&c->vc.lock, flags);
	d = c->active;
	if (d && d->vd.tx.cookie == cookie) {
		if (d->error)
			status = DMA_ERROR;
		position = readl(c->dma->base + c->id * CH_STRIDE +
				 (d->direction == DMA_DEV_TO_MEM ? CH_DST : CH_SRC));
		if (state && position >= d->buffer &&
		    position <= d->buffer + d->length)
			dma_set_residue(state, d->length - (position - d->buffer));
	}
	spin_unlock_irqrestore(&c->vc.lock, flags);
	return status;
}

static int n32_alloc_resources(struct dma_chan *chan)
{
	dma_cookie_init(chan);
	return 0;
}

static void n32_free_resources(struct dma_chan *chan)
{
	if (n32_terminate_all(chan))
		return;
	n32_synchronize(chan);
	vchan_free_chan_resources(&to_n32_chan(chan)->vc);
}

static struct dma_chan *n32_xlate(struct of_phandle_args *spec,
				  struct of_dma *ofdma)
{
	struct n32_dma *dma = ofdma->of_dma_data;
	struct dma_chan *chan;

	if (spec->args_count != 1 || spec->args[0] > 254)
		return NULL;
	chan = dma_get_any_slave_channel(&dma->engine);
	if (chan)
		to_n32_chan(chan)->request = spec->args[0];
	return chan;
}

static int n32_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct n32_dma *dma;
	struct reset_control *rst;
	int i, ret;

	dma = devm_kzalloc(dev, sizeof(*dma), GFP_KERNEL);
	if (!dma)
		return -ENOMEM;
	dma->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(dma->base))
		return PTR_ERR(dma->base);
	dma->mux = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(dma->mux))
		return PTR_ERR(dma->mux);
	dma->clocks[0].id = "dma";
	dma->clocks[1].id = "mux";
	ret = devm_clk_bulk_get(dev, 2, dma->clocks);
	if (ret)
		return ret;
	ret = clk_bulk_prepare_enable(2, dma->clocks);
	if (ret)
		return ret;
	rst = devm_reset_control_array_get_exclusive(dev);
	if (IS_ERR(rst)) {
		ret = PTR_ERR(rst);
		goto disable_clocks;
	}
	ret = reset_control_reset(rst);
	if (ret)
		goto disable_clocks;
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		goto disable_clocks;
	dma->pool = dma_pool_create("n32h7xx-lli", dev,
				    N32_PERIODS * sizeof(struct n32_lli), 32, 0);
	if (!dma->pool) {
		ret = -ENOMEM;
		goto disable_clocks;
	}
	writel(0xff00, dma->base + DMA_EN);
	for (i = 0; i < 5; i++) {
		writel(0xff00, dma->base + 0x310 + i * 8);
		writel(0xff, dma->base + CLEAR_BASE + i * 8);
	}
	writel(1, dma->base + DMA_CFG);
	INIT_LIST_HEAD(&dma->engine.channels);
	dma->engine.dev = dev;
	dma_cap_set(DMA_SLAVE, dma->engine.cap_mask);
	dma_cap_set(DMA_CYCLIC, dma->engine.cap_mask);
	dma->engine.src_addr_widths = BIT(DMA_SLAVE_BUSWIDTH_1_BYTE) |
		BIT(DMA_SLAVE_BUSWIDTH_2_BYTES) | BIT(DMA_SLAVE_BUSWIDTH_4_BYTES);
	dma->engine.dst_addr_widths = dma->engine.src_addr_widths;
	dma->engine.directions = BIT(DMA_DEV_TO_MEM) | BIT(DMA_MEM_TO_DEV);
	dma->engine.residue_granularity = DMA_RESIDUE_GRANULARITY_BURST;
	dma->engine.device_alloc_chan_resources = n32_alloc_resources;
	dma->engine.device_free_chan_resources = n32_free_resources;
	dma->engine.device_config = n32_config;
	dma->engine.device_prep_dma_cyclic = n32_prep_cyclic;
	dma->engine.device_issue_pending = n32_issue_pending;
	dma->engine.device_terminate_all = n32_terminate_all;
	dma->engine.device_synchronize = n32_synchronize;
	dma->engine.device_tx_status = n32_status;
	for (i = 0; i < N32_CHANNELS; i++) {
		struct n32_chan *c = &dma->chan[i];

		c->dma = dma;
		c->id = i;
		c->irq = platform_get_irq(pdev, i);
		if (c->irq < 0) {
			ret = c->irq;
			goto destroy_pool;
		}
		c->vc.desc_free = n32_desc_free;
		vchan_init(&c->vc, &dma->engine);
		ret = devm_request_irq(dev, c->irq, n32_irq, 0, dev_name(dev), c);
		if (ret)
			goto destroy_pool;
	}
	ret = dma_async_device_register(&dma->engine);
	if (ret)
		goto destroy_pool;
	ret = of_dma_controller_register(dev->of_node, n32_xlate, dma);
	if (ret) {
		dma_async_device_unregister(&dma->engine);
		goto destroy_pool;
	}
	platform_set_drvdata(pdev, dma);
	return 0;
destroy_pool:
	dma_pool_destroy(dma->pool);
disable_clocks:
	clk_bulk_disable_unprepare(2, dma->clocks);
	return ret;
}

static void n32_remove(struct platform_device *pdev)
{
	struct n32_dma *dma = platform_get_drvdata(pdev);
	int i;

	of_dma_controller_free(pdev->dev.of_node);
	dma_async_device_unregister(&dma->engine);
	for (i = 0; i < N32_CHANNELS; i++) {
		n32_free_resources(&dma->chan[i].vc.chan);
		synchronize_irq(dma->chan[i].irq);
	}
	writel(0, dma->base + DMA_CFG);
	dma_pool_destroy(dma->pool);
	clk_bulk_disable_unprepare(2, dma->clocks);
}

static const struct of_device_id n32_match[] = {
	{ .compatible = "nsing,n32h7xx-dma" },
	{ }
};
MODULE_DEVICE_TABLE(of, n32_match);

static struct platform_driver n32_driver = {
	.probe = n32_probe,
	.remove = n32_remove,
	.driver = { .name = "n32h7xx-dma", .of_match_table = n32_match },
};
module_platform_driver(n32_driver);
MODULE_DESCRIPTION("N32H7xx cyclic slave DMA controller");
MODULE_LICENSE("GPL");
