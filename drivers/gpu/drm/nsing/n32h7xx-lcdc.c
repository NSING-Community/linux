// SPDX-License-Identifier: GPL-2.0-only
/* NSING N32H7xx parallel RGB LCD controller. */

#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/soc/nsing/n32h7xx-graphics.h>

#include <drm/clients/drm_client_setup.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_fbdev_dma.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_of.h>
#include <drm/drm_print.h>
#include <drm/drm_simple_kms_helper.h>
#include <drm/drm_vblank.h>

#include "n32h7xx-drm.h"

#define LCDC_SYNCCTRL		0x08
#define LCDC_BPCTRL		0x0c
#define LCDC_AWCTRL		0x10
#define LCDC_TWCTRL		0x14
#define LCDC_GCTRL		0x18
#define LCDC_SRCTRL		0x24
#define LCDC_BGCCTRL		0x2c
#define LCDC_INTEN		0x34
#define LCDC_INTSTS		0x38
#define LCDC_INTCLR		0x3c
#define LCDC_LINTPCTRL		0x40
#define LCDC_LSRCTRL(n)		(0x100 * (n) + 0x08)
#define LCDC_LCTRL(n)		(0x100 * (n) + 0x0c)
#define LCDC_WHPCTRL		0x110
#define LCDC_WVPCTRL		0x114
#define LCDC_PFCTRL		0x11c
#define LCDC_CACTRL		0x120
#define LCDC_DCCTRL		0x124
#define LCDC_BFCTRL		0x128
#define LCDC_CFBADDR		0x134
#define LCDC_CFBLEN		0x138
#define LCDC_CFBLNUM		0x13c

#define LCDC_ENABLE		BIT(0)
#define LCDC_DE_LOW		BIT(29)
#define LCDC_VSYNC_HIGH		BIT(30)
#define LCDC_HSYNC_HIGH		BIT(31)
#define LCDC_RELOAD_NOW		BIT(0)
#define LCDC_RELOAD_VBLANK	BIT(1)
#define LCDC_IRQ_LINE		BIT(0)
#define LCDC_IRQ_UNDERRUN		BIT(1)
#define LCDC_IRQ_BUS_ERROR	BIT(2)
#define LCDC_IRQ_RELOAD		BIT(3)
#define LCDC_IRQ_MASK		GENMASK(3, 0)

struct n32h7xx_lcdc {
	struct drm_device drm;
	struct drm_simple_display_pipe pipe;
	void __iomem *regs;
	struct clk *pixel;
	struct drm_pending_vblank_event *event;
};

static struct n32h7xx_lcdc *pipe_to_lcdc(struct drm_simple_display_pipe *pipe)
{
	return container_of(pipe, struct n32h7xx_lcdc, pipe);
}

static void n32h7xx_lcdc_set_fb(struct n32h7xx_lcdc *lcdc)
{
	struct drm_plane_state *state = lcdc->pipe.plane.state;
	struct drm_framebuffer *fb = state->fb;
	dma_addr_t addr = drm_fb_dma_get_gem_addr(fb, state, 0);
	u32 line = fb->width * fb->format->cpp[0];

	writel(fb->format->format == DRM_FORMAT_RGB565 ? 4 : 0,
	       lcdc->regs + LCDC_PFCTRL);
	writel(addr, lcdc->regs + LCDC_CFBADDR);
	/* The bus fetch length includes the controller's seven-byte bias. */
	writel((fb->pitches[0] << 16) | (line + 7), lcdc->regs + LCDC_CFBLEN);
	writel(fb->height, lcdc->regs + LCDC_CFBLNUM);
}

static enum drm_mode_status
n32h7xx_lcdc_mode_valid(struct drm_simple_display_pipe *pipe,
			const struct drm_display_mode *mode)
{
	struct n32h7xx_lcdc *lcdc = pipe_to_lcdc(pipe);

	if (mode->flags & (DRM_MODE_FLAG_INTERLACE | DRM_MODE_FLAG_DBLSCAN))
		return MODE_NO_INTERLACE;
	if (mode->hdisplay > 2048 || mode->htotal > 4096 ||
	    mode->vdisplay > 2048 || mode->vtotal > 4096)
		return MODE_BAD;
	if (mode->clock != clk_get_rate(lcdc->pixel) / 1000)
		return MODE_CLOCK_RANGE;
	return MODE_OK;
}

static int n32h7xx_lcdc_check(struct drm_simple_display_pipe *pipe,
			      struct drm_plane_state *state,
			      struct drm_crtc_state *crtc_state)
{
	struct drm_framebuffer *fb = state->fb;

	if (!fb)
		return 0;
	if (state->src_x || state->src_y ||
	    fb->width != crtc_state->mode.hdisplay ||
	    fb->height != crtc_state->mode.vdisplay ||
	    fb->pitches[0] > 0xffff || fb->pitches[0] & 3 ||
	    fb->offsets[0] & 3)
		return -EINVAL;
	return 0;
}

static void n32h7xx_lcdc_enable(struct drm_simple_display_pipe *pipe,
				struct drm_crtc_state *crtc_state,
				struct drm_plane_state *plane_state)
{
	static const struct drm_vblank_crtc_config vblank_config = {
		.offdelay_ms = 0,
	};
	struct n32h7xx_lcdc *lcdc = pipe_to_lcdc(pipe);
	const struct drm_display_mode *m = &crtc_state->adjusted_mode;
	u32 hsw = m->hsync_end - m->hsync_start;
	u32 vsw = m->vsync_end - m->vsync_start;
	u32 hbp = m->htotal - m->hsync_start;
	u32 vbp = m->vtotal - m->vsync_start;
	u32 ctrl = LCDC_DE_LOW;
	int i;

	if (m->flags & DRM_MODE_FLAG_PHSYNC)
		ctrl |= LCDC_HSYNC_HIGH;
	if (m->flags & DRM_MODE_FLAG_PVSYNC)
		ctrl |= LCDC_VSYNC_HIGH;
	writel(ctrl, lcdc->regs + LCDC_GCTRL);
	writel(((hsw - 1) << 16) | (vsw - 1), lcdc->regs + LCDC_SYNCCTRL);
	writel(((hbp - 1) << 16) | (vbp - 1), lcdc->regs + LCDC_BPCTRL);
	writel(((hbp + m->hdisplay - 1) << 16) | (vbp + m->vdisplay - 1),
	       lcdc->regs + LCDC_AWCTRL);
	writel(((m->htotal - 1) << 16) | (m->vtotal - 1),
	       lcdc->regs + LCDC_TWCTRL);
	writel(0, lcdc->regs + LCDC_BGCCTRL);
	for (i = 1; i <= 4; i++) {
		writel(0, lcdc->regs + LCDC_LCTRL(i));
		writel(LCDC_RELOAD_NOW, lcdc->regs + LCDC_LSRCTRL(i));
	}
	writel(((hbp + m->hdisplay - 1) << 16) | hbp,
	       lcdc->regs + LCDC_WHPCTRL);
	writel(((vbp + m->vdisplay - 1) << 16) | vbp,
	       lcdc->regs + LCDC_WVPCTRL);
	writel(255, lcdc->regs + LCDC_CACTRL);
	writel(0, lcdc->regs + LCDC_DCCTRL);
	/* Constant alpha, opaque layer: BF1 = CA, BF2 = 1 - CA. */
	writel(0x405, lcdc->regs + LCDC_BFCTRL);
	n32h7xx_lcdc_set_fb(lcdc);
	writel(LCDC_ENABLE, lcdc->regs + LCDC_LCTRL(1));
	writel(LCDC_RELOAD_NOW, lcdc->regs + LCDC_SRCTRL);
	writel(vbp + m->vdisplay, lcdc->regs + LCDC_LINTPCTRL);
	writel(LCDC_IRQ_MASK, lcdc->regs + LCDC_INTCLR);
	writel(LCDC_IRQ_MASK, lcdc->regs + LCDC_INTEN);
	writel(ctrl | LCDC_ENABLE, lcdc->regs + LCDC_GCTRL);
	drm_crtc_vblank_on_config(&pipe->crtc, &vblank_config);
	/* Initial plane update runs before enable; complete it on the first VB. */
	writel(LCDC_RELOAD_VBLANK, lcdc->regs + LCDC_SRCTRL);
}

static void n32h7xx_lcdc_disable(struct drm_simple_display_pipe *pipe)
{
	struct n32h7xx_lcdc *lcdc = pipe_to_lcdc(pipe);
	struct drm_device *drm = &lcdc->drm;
	struct drm_crtc_state *state = pipe->crtc.state;
	unsigned long flags;

	writel(0, lcdc->regs + LCDC_INTEN);
	writel(0, lcdc->regs + LCDC_GCTRL);
	drm_crtc_vblank_off(&pipe->crtc);
	spin_lock_irqsave(&drm->event_lock, flags);
	/*
	 * drm_atomic_helper_setup_commit() attaches an event to every atomic
	 * commit; the disable commit has no plane update to consume it, so
	 * complete it here or flip_done waits 10 s per commit.
	 */
	if (state->event) {
		drm_crtc_send_vblank_event(&pipe->crtc, state->event);
		state->event = NULL;
	}
	if (lcdc->event) {
		drm_crtc_send_vblank_event(&pipe->crtc, lcdc->event);
		lcdc->event = NULL;
	}
	spin_unlock_irqrestore(&drm->event_lock, flags);
}

static void n32h7xx_lcdc_update(struct drm_simple_display_pipe *pipe,
				struct drm_plane_state *old_state)
{
	struct n32h7xx_lcdc *lcdc = pipe_to_lcdc(pipe);
	struct drm_crtc_state *state = pipe->crtc.state;
	unsigned long flags;

	if (!pipe->plane.state->fb)
		return;

	/* Publish the event and reload together; IRQ completes after latching. */
	spin_lock_irqsave(&lcdc->drm.event_lock, flags);
	writel(LCDC_IRQ_RELOAD, lcdc->regs + LCDC_INTCLR);
	n32h7xx_lcdc_set_fb(lcdc);
	if (state->event) {
		lcdc->event = state->event;
		state->event = NULL;
	}
	writel(LCDC_RELOAD_VBLANK, lcdc->regs + LCDC_SRCTRL);
	spin_unlock_irqrestore(&lcdc->drm.event_lock, flags);
}

static int n32h7xx_lcdc_enable_vblank(struct drm_simple_display_pipe *pipe)
{
	/* The line interrupt stays enabled for the entire active mode. */
	return 0;
}

static void n32h7xx_lcdc_disable_vblank(struct drm_simple_display_pipe *pipe)
{
}

static irqreturn_t n32h7xx_lcdc_irq(int irq, void *data)
{
	struct n32h7xx_lcdc *lcdc = data;
	u32 status = readl(lcdc->regs + LCDC_INTSTS) &
		     readl(lcdc->regs + LCDC_INTEN);
	unsigned long flags;

	if (!status)
		return IRQ_NONE;
	if (status & LCDC_IRQ_LINE) {
		writel(LCDC_IRQ_LINE, lcdc->regs + LCDC_INTCLR);
		drm_crtc_handle_vblank(&lcdc->pipe.crtc);
	}
	/* Serialize reload acknowledgment with publication of the next flip. */
	spin_lock_irqsave(&lcdc->drm.event_lock, flags);
	status = readl(lcdc->regs + LCDC_INTSTS) &
		 readl(lcdc->regs + LCDC_INTEN) & ~LCDC_IRQ_LINE;
	writel(status, lcdc->regs + LCDC_INTCLR);
	if (status & LCDC_IRQ_RELOAD) {
		if (lcdc->event) {
			drm_crtc_send_vblank_event(&lcdc->pipe.crtc, lcdc->event);
			lcdc->event = NULL;
		}
	}
	spin_unlock_irqrestore(&lcdc->drm.event_lock, flags);
	if (status & (LCDC_IRQ_UNDERRUN | LCDC_IRQ_BUS_ERROR))
		dev_warn_ratelimited(lcdc->drm.dev, "LCD fetch error: %#x\n", status);
	return IRQ_HANDLED;
}

static const struct drm_simple_display_pipe_funcs n32h7xx_lcdc_pipe_funcs = {
	.mode_valid = n32h7xx_lcdc_mode_valid,
	.check = n32h7xx_lcdc_check,
	.enable = n32h7xx_lcdc_enable,
	.disable = n32h7xx_lcdc_disable,
	.update = n32h7xx_lcdc_update,
	.enable_vblank = n32h7xx_lcdc_enable_vblank,
	.disable_vblank = n32h7xx_lcdc_disable_vblank,
};

static const struct drm_mode_config_funcs n32h7xx_lcdc_mode_funcs = {
	.fb_create = drm_gem_fb_create,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

DEFINE_DRM_GEM_DMA_FOPS(n32h7xx_lcdc_fops);

static const struct drm_driver n32h7xx_lcdc_driver = {
	.driver_features = DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	DRM_GEM_DMA_DRIVER_OPS,
	DRM_FBDEV_DMA_DRIVER_OPS,
	.fops = &n32h7xx_lcdc_fops,
	.name = "n32h7xx-lcdc",
	.desc = "NSING N32H7xx LCD controller",
	.major = 1,
};

static int n32h7xx_lcdc_probe(struct platform_device *pdev)
{
	static const u32 formats[] = { DRM_FORMAT_RGB565, DRM_FORMAT_XRGB8888 };
	struct device *dev = &pdev->dev;
	struct n32h7xx_lcdc *lcdc;
	struct drm_device *drm;
	struct drm_bridge *bridge;
	struct reset_control *reset;
	struct clk *clk;
	int ret, irq, error_irq;

	lcdc = devm_drm_dev_alloc(dev, &n32h7xx_lcdc_driver,
				  struct n32h7xx_lcdc, drm);
	if (IS_ERR(lcdc))
		return PTR_ERR(lcdc);
	drm = &lcdc->drm;
	bridge = devm_drm_of_get_bridge(dev, dev->of_node, 0, 0);
	if (IS_ERR(bridge))
		return dev_err_probe(dev, PTR_ERR(bridge), "cannot find LCD panel\n");
	ret = n32h7xx_graphics_acquire(dev, N32H7XX_GRAPHICS_LCDC);
	if (ret)
		return ret;
	clk = devm_clk_get_enabled(dev, "bus");
	if (IS_ERR(clk))
		return PTR_ERR(clk);
	clk = devm_clk_get_enabled(dev, "apb");
	if (IS_ERR(clk))
		return PTR_ERR(clk);
	lcdc->pixel = devm_clk_get_enabled(dev, "pixel");
	if (IS_ERR(lcdc->pixel))
		return PTR_ERR(lcdc->pixel);
	reset = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(reset))
		return PTR_ERR(reset);
	ret = reset_control_reset(reset);
	if (ret)
		return ret;
	lcdc->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(lcdc->regs))
		return PTR_ERR(lcdc->regs);
	writel(0, lcdc->regs + LCDC_INTEN);
	ret = n32h7xx_drm_init_memory(drm);
	if (ret)
		return ret;
	ret = drmm_mode_config_init(drm);
	if (ret)
		return ret;
	drm->mode_config.funcs = &n32h7xx_lcdc_mode_funcs;
	drm->mode_config.max_width = 2048;
	drm->mode_config.max_height = 2048;
	ret = drm_simple_display_pipe_init(drm, &lcdc->pipe,
					   &n32h7xx_lcdc_pipe_funcs, formats,
					   ARRAY_SIZE(formats), NULL, NULL);
	if (ret)
		return ret;
	ret = drm_simple_display_pipe_attach_bridge(&lcdc->pipe, bridge);
	if (ret)
		return ret;
	ret = drm_vblank_init(drm, 1);
	if (ret)
		return ret;
	irq = platform_get_irq_byname(pdev, "event");
	if (irq < 0)
		return irq;
	error_irq = platform_get_irq_byname(pdev, "error");
	if (error_irq < 0)
		return error_irq;
	ret = devm_request_irq(dev, irq, n32h7xx_lcdc_irq, 0, dev_name(dev), lcdc);
	if (ret)
		return ret;
	ret = devm_request_irq(dev, error_irq, n32h7xx_lcdc_irq, 0,
			       dev_name(dev), lcdc);
	if (ret)
		return ret;
	drm_mode_config_reset(drm);
	platform_set_drvdata(pdev, drm);
	ret = drm_dev_register(drm, 0);
	if (ret)
		return ret;
	drm_client_setup_with_fourcc(drm, DRM_FORMAT_RGB565);
	/*
	 * The fbdev client probes a mode and allocates the framebuffer, but
	 * leaves the commit that enables the CRTC to fbcon's fb_set_par().
	 * This board has no virtual terminal (the console is on the serial
	 * port), so nothing would apply it: the panel stays dark until the
	 * first DRM client closes its file and the lastclose restore runs.
	 * Apply the probed configuration here instead.
	 */
#if IS_ENABLED(CONFIG_DRM_FBDEV_EMULATION)
	if (drm->fb_helper) {
		ret = drm_fb_helper_restore_fbdev_mode_unlocked(drm->fb_helper,
								false);
		if (ret)
			drm_warn(drm, "cannot apply the initial mode: %d\n", ret);
	}
#endif
	return 0;
}

static void n32h7xx_lcdc_remove(struct platform_device *pdev)
{
	struct drm_device *drm = platform_get_drvdata(pdev);

	drm_dev_unplug(drm);
	drm_atomic_helper_shutdown(drm);
}

static void n32h7xx_lcdc_shutdown(struct platform_device *pdev)
{
	drm_atomic_helper_shutdown(platform_get_drvdata(pdev));
}

static const struct of_device_id n32h7xx_lcdc_match[] = {
	{ .compatible = "nsing,n32h7xx-lcdc" },
	{ }
};
MODULE_DEVICE_TABLE(of, n32h7xx_lcdc_match);

static struct platform_driver n32h7xx_lcdc_platform_driver = {
	.probe = n32h7xx_lcdc_probe,
	.remove = n32h7xx_lcdc_remove,
	.shutdown = n32h7xx_lcdc_shutdown,
	.driver = {
		.name = "n32h7xx-lcdc",
		.of_match_table = n32h7xx_lcdc_match,
	},
};
module_platform_driver(n32h7xx_lcdc_platform_driver);

MODULE_DESCRIPTION("NSING N32H7xx LCD controller DRM driver");
MODULE_LICENSE("GPL");
