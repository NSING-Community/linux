/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * NSING N32H7xx GRAPHICS power-domain support.
 */
#ifndef __LINUX_SOC_NSING_N32H7XX_GRAPHICS_H
#define __LINUX_SOC_NSING_N32H7XX_GRAPHICS_H

#include <linux/bits.h>
#include <linux/types.h>

struct device;

/* AHB6 graphics register block and its shared memory-management unit. */
#define N32H7XX_RCC_BASE		0x58030000
#define N32H7XX_MMU_BASE		0x51105000

#define N32H7XX_RCC_CFG2		0x160
#define N32H7XX_RCC_CFG4		0x168
#define N32H7XX_RCC_CFG2_M7MMU		(BIT(23) | BIT(22))
#define N32H7XX_RCC_CFG4_APB6CLKEN	BIT(19)

/* Per-master module-memory gates in the shared GRAPHICS MMU. */
#define N32H7XX_MMU_DVP1ME		0x144
#define N32H7XX_MMU_DVP2ME		0x148
#define N32H7XX_MMU_JPEGME		0x15c
#define N32H7XX_MMU_LCDME		0x160
#define N32H7XX_MMU_GPUME		0x164
#define N32H7XX_MMU_ME_EN		BIT(0)

enum n32h7xx_graphics_module {
	N32H7XX_GRAPHICS_DVP,
	N32H7XX_GRAPHICS_JPEG,
	N32H7XX_GRAPHICS_LCDC,
	N32H7XX_GRAPHICS_GPU,
	N32H7XX_GRAPHICS_MODULE_COUNT,
};

/*
 * Acquire the shared GRAPHICS power domain and the memory gate for one
 * module.  The reference is released automatically when the device is
 * removed or its probe fails.  Callers only manage the gate named here;
 * they must not power the shared domain up or down directly.
 */
int n32h7xx_graphics_acquire(struct device *dev,
			     enum n32h7xx_graphics_module module);

/* Make the reserved framebuffer/GPU pool coherent on the NOMMU Cortex-M7. */
int n32h7xx_graphics_dma_prepare(struct device *dev);

#endif /* __LINUX_SOC_NSING_N32H7XX_GRAPHICS_H */
