// SPDX-License-Identifier: GPL-2.0-only
/* NSING N32H7xx DWC SSI platform integration. */

#include <linux/clk.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/soc/nsing/n32h7xx-storage.h>
#include <linux/spi/spi-mem.h>

#include "../spi-dw.h"

/* Bound the IRQ-masked standard PIO transaction at the board's SPI rate. */
#define N32H7XX_XSPI_MAX_DATA	256
#define N32H7XX_XSPI_TIMEOUT_US	10000

static int n32h7xx_xspi_wait_mem_op_done(struct dw_spi *dws)
{
	unsigned long ns;
	u32 nents, sr;
	int ret;

	nents = dw_readl(dws, DW_SPI_TXFLR);
	ns = NSEC_PER_SEC / dws->current_freq * nents;
	ns *= dws->n_bytes * BITS_PER_BYTE;

	/* Allow the final frame to finish even when TXFLR has reached zero. */
	ret = readl_poll_timeout_atomic(dws->regs + DW_SPI_SR, sr,
					!(sr & DW_SPI_SR_BUSY),
					max_t(unsigned long,
					      DIV_ROUND_UP(ns, NSEC_PER_USEC), 1),
					N32H7XX_XSPI_TIMEOUT_US);
	if (ret) {
		dev_err(&dws->ctlr->dev, "Mem op hanged up\n");
		return -EIO;
	}

	return 0;
}

static int n32h7xx_xspi_adjust_op_size(struct spi_mem *mem,
				     struct spi_mem_op *op)
{
	u32 limit = op->data.buswidth > 1 ? DW_SPI_NDF_MASK + 1 :
		    N32H7XX_XSPI_MAX_DATA;

	op->data.nbytes = min_t(unsigned int, op->data.nbytes, limit);
	return 0;
}

static int n32h7xx_xspi_exec_op(struct spi_mem *mem,
			      const struct spi_mem_op *op)
{
	struct dw_spi *dws = spi_controller_get_devdata(mem->spi->controller);
	struct spi_mem_op standard = *op;
	u32 frames = op->cmd.nbytes + op->addr.nbytes + op->dummy.nbytes;
	u32 threshold = 0;
	u32 control;

	if (op->data.buswidth > 1)
		return dw_spi_exec_enh_mem_op(mem, op);

	/* Register commands and programming retain the validated standard path. */
	standard.max_freq = min(op->max_freq, 9375000U);
	dw_spi_enable_chip(dws, 0);
	control = dw_readl(dws, DW_SPI_SPI_CTRLR0);
	control &= ~DW_SPI_ENH_CTRLR0_CLK_STRETCH_EN;
	if (op->data.dir != SPI_MEM_DATA_IN) {
		if (op->data.dir == SPI_MEM_DATA_OUT)
			frames += op->data.nbytes;
		/* N32H7xx TX-only mode requires the actual frame count. */
		dw_writel(dws, DW_SPI_CTRLR1, frames);
		threshold = min_t(u32, frames, dws->fifo_len) - 1;
		control |= DW_SPI_ENH_CTRLR0_CLK_STRETCH_EN;
	}
	/* The upper field controls how many entries start transmission. */
	dw_writel(dws, DW_SPI_TXFTLR, threshold << 16);
	dw_writel(dws, DW_SPI_SPI_CTRLR0, control);
	return dw_spi_exec_mem_op(mem, &standard);
}

static void n32h7xx_xspi_remove(void *data)
{
	dw_spi_remove_controller(data);
}

static int n32h7xx_xspi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct dw_spi *dws;
	struct clk *clk;
	int ret;

	dws = devm_kzalloc(dev, sizeof(*dws), GFP_KERNEL);
	if (!dws)
		return -ENOMEM;

	dws->regs = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(dws->regs))
		return PTR_ERR(dws->regs);
	dws->paddr = res->start;

	ret = n32h7xx_xspi2_soc_init(dev);
	if (ret)
		return ret;

	clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "failed to enable SSI clock\n");

	dws->max_freq = clk_get_rate(clk);
	if (!dws->max_freq)
		return -EINVAL;

	dws->irq = platform_get_irq(pdev, 0);
	if (dws->irq < 0)
		return dev_err_probe(dev, dws->irq, "failed to get SSI interrupt\n");

	/* HSSI has DFS at bit 0, TMOD at bit 10 and the master bit at bit 31. */
	dws->ip = DW_HSSI_ID;
	dws->reg_io_width = 4;
	dws->fifo_len = 16;
	dws->num_cs = 1;
	dws->caps = DW_SPI_CAP_EMODE | DW_SPI_CAP_MEM_RX32 |
		    DW_SPI_CAP_MEM_POLL | DW_SPI_CAP_TXONLY_NDF;
	dws->bus_num = pdev->id;
	dws->max_mem_freq = dws->max_freq;
	dws->mem_ops.supports_op = dw_spi_supports_enh_mem_op;
	dws->mem_ops.adjust_op_size = n32h7xx_xspi_adjust_op_size;
	dws->mem_ops.exec_op = n32h7xx_xspi_exec_op;
	dws->wait_mem_op_done = n32h7xx_xspi_wait_mem_op_done;

	ret = pm_runtime_set_active(dev);
	if (ret)
		return ret;
	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;

	ret = dw_spi_add_controller(dev, dws);
	if (ret)
		return ret;

	return devm_add_action_or_reset(dev, n32h7xx_xspi_remove, dws);
}

static const struct of_device_id n32h7xx_xspi_of_match[] = {
	{ .compatible = "nsing,n32h7xx-xspi" },
	{ }
};
MODULE_DEVICE_TABLE(of, n32h7xx_xspi_of_match);

static struct platform_driver n32h7xx_xspi_driver = {
	.probe = n32h7xx_xspi_probe,
	.driver = {
		.name = "n32h7xx-xspi",
		.of_match_table = n32h7xx_xspi_of_match,
	},
};
module_platform_driver(n32h7xx_xspi_driver);

MODULE_DESCRIPTION("NSING N32H7xx DWC SSI platform driver");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("SPI_DW_CORE");
