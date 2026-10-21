/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _N32H7XX_DRM_H
#define _N32H7XX_DRM_H

#include <linux/dma-mapping.h>
#include <linux/of_reserved_mem.h>
#include <linux/soc/nsing/n32h7xx-graphics.h>

#include <drm/drm_device.h>
#include <drm/drm_managed.h>

static inline void n32h7xx_drm_release_memory(struct drm_device *drm, void *data)
{
	of_reserved_mem_device_release(drm->dev);
}

static inline int n32h7xx_drm_init_memory(struct drm_device *drm)
{
	int ret;

	ret = n32h7xx_graphics_dma_prepare(drm->dev);
	if (ret)
		return ret;
	ret = dma_set_mask_and_coherent(drm->dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	/* Require a private pool; never consume the small USB/SD DMA pool. */
	ret = of_reserved_mem_device_init(drm->dev);
	if (ret)
		return dev_err_probe(drm->dev, ret, "missing graphics DMA pool\n");

	/* GEM objects may outlive platform unbind and still need this pool. */
	return drmm_add_action_or_reset(drm, n32h7xx_drm_release_memory, NULL);
}

#endif
