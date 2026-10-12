/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * NSING N32H7xx RTC SoC support.
 *
 * Starting the 32.768 kHz LSE oscillator and selecting it as the RTC clock
 * source means touching RCC and PWR registers that have nothing to do with
 * the RTC peripheral's own register block, so that knowledge lives with the
 * rest of the mach-nsing glue.  The driver in drivers/rtc/nsing only
 * programs the calendar.
 */
#ifndef __LINUX_SOC_NSING_N32H7XX_RTC_H
#define __LINUX_SOC_NSING_N32H7XX_RTC_H

struct device;

/*
 * Bring up the LSE oscillator, select it as the RTC clock and release the
 * RTC APB gates.  Returns 0 on success or a negative errno.  Safe to call
 * more than once, and never resets the backup domain, so a calendar that is
 * already running survives.
 */
int n32h7xx_rtc_soc_init(struct device *dev);

#endif /* __LINUX_SOC_NSING_N32H7XX_RTC_H */
