/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * NSING N32H7xx Ethernet MAC SoC support.
 *
 * The register blocks that gate, reset and route the on-chip Ethernet MAC
 * (RCC, PWR, AFIO and the GPIO alternate-function mux) are specific to the
 * N32H7xx series, so they live with the rest of the mach-nsing glue instead
 * of in the network driver.  The driver in drivers/net/ethernet/nsing only
 * owns the MAC, MTL and DMA programming.
 */
#ifndef __LINUX_SOC_NSING_N32H7XX_ETH_H
#define __LINUX_SOC_NSING_N32H7XX_ETH_H

#include <linux/types.h>

struct device;

/*
 * Bring the ETH1 instance of the MAC out of reset and route its pins for the
 * interface mode described by the device tree.  Returns 0 on success or a
 * negative errno.  Safe to call more than once.
 */
int n32h7xx_eth_soc_init(struct device *dev);

/*
 * Derive a stable, locally administered MAC address for the on-chip MAC from
 * the SoC unique ID.  The part has no factory MAC address, so this is the
 * fallback when neither the device tree nor the bootloader supplies one.
 * Fills @addr (ETH_ALEN bytes) and returns 0 on success, or a negative errno
 * if the chip ID could not be read.
 */
int n32h7xx_eth_soc_mac_addr(u8 *addr);

#endif /* __LINUX_SOC_NSING_N32H7XX_ETH_H */
