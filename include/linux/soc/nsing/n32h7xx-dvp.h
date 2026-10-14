/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * NSING N32H7xx DVP (parallel camera) SoC support.
 *
 * The register blocks that power, clock, reset and route the on-chip DVP
 * capture units are a property of the N32H7xx series, not of the DVP
 * hardware itself, so they live with the rest of the mach-nsing glue.  The
 * media driver in drivers/media/platform/nsing only programs the DVP
 * register block, the frame buffers and the V4L2 plumbing.
 */
#ifndef __LINUX_SOC_NSING_N32H7XX_DVP_H
#define __LINUX_SOC_NSING_N32H7XX_DVP_H

#include <linux/types.h>

struct device;

/*
 * Bring the DVP instance described by the device tree node out of reset:
 * release its memory power gate together with the GRAPHICS sub-domain, open
 * the AHB/APB clocks and the module-memory view, then pulse the peripheral
 * reset.  Returns 0 on success or a negative errno.  Safe to call more than
 * once.
 */
int n32h7xx_dvp_soc_init(struct device *dev);

#endif /* __LINUX_SOC_NSING_N32H7XX_DVP_H */
