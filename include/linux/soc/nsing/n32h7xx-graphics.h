/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * NSING N32H7xx GRAPHICS power-domain support.
 */
#ifndef __LINUX_SOC_NSING_N32H7XX_GRAPHICS_H
#define __LINUX_SOC_NSING_N32H7XX_GRAPHICS_H

#include <linux/types.h>

struct device;

enum n32h7xx_graphics_module {
	N32H7XX_GRAPHICS_DVP,
	N32H7XX_GRAPHICS_JPEG,
};

/*
 * Acquire the shared GRAPHICS power domain and the memory gate for one
 * module.  The reference is released automatically when the device is
 * removed or its probe fails.  Callers only manage the gate named here;
 * they must not power the shared domain up or down directly.
 */
int n32h7xx_graphics_acquire(struct device *dev,
			     enum n32h7xx_graphics_module module);

#endif /* __LINUX_SOC_NSING_N32H7XX_GRAPHICS_H */
