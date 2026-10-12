/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * NSING N32H7xx storage-controller SoC support.
 *
 * The RCC clock gates, resets, PWR power domains, AFIO routing and pin mux
 * needed by the on-chip XSPI (serial flash) and SDMMC controllers are a
 * property of the N32H7xx series, not of the controllers themselves.  They are
 * programmed here so that the actual controller drivers stay free of SoC
 * register knowledge.
 */
#ifndef __LINUX_SOC_NSING_N32H7XX_STORAGE_H
#define __LINUX_SOC_NSING_N32H7XX_STORAGE_H

struct device;

/*
 * Bring up the XSPI2 quad-SPI controller: GPIO/AFIO clocks, the AXI run and
 * low-power gates, the peripheral reset and the board pin mux.  Returns 0 on
 * success or a negative error code.
 */
int n32h7xx_xspi2_soc_init(struct device *dev);

/*
 * Bring up the SDMMC1 SDHCI host: GPIO/AFIO clocks, the SDMMC1 kernel clock
 * source and divider, the AXI run and low-power gates, the SDMMC1/SDHOST1
 * resets, the SDMMC1 power domain, the AFIO clock-feedback route and the board
 * pin mux.
 */
int n32h7xx_sdmmc1_soc_init(struct device *dev);

/* Base clock (Hz) the SDMMC1 kernel clock is configured to. */
unsigned long n32h7xx_sdmmc1_soc_clock_rate(void);

#endif /* __LINUX_SOC_NSING_N32H7XX_STORAGE_H */
