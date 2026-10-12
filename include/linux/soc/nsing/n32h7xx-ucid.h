/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * NSING N32H7xx unique chip ID (UCID).
 *
 * Every N32H7xx carries a 128-bit ID in on-chip OTP.  It is the only
 * chip-unique value the part exposes: there is no factory-programmed MAC
 * address and the Cortex-M7 CPUID reports only a part number, so anything
 * that needs a per-board identity (a MAC address, for instance) derives it
 * from this ID.  The OTP block is not memory mapped; the mach-nsing code
 * reads it through the OTPC controller.
 */
#ifndef __LINUX_SOC_NSING_N32H7XX_UCID_H
#define __LINUX_SOC_NSING_N32H7XX_UCID_H

#include <linux/types.h>

/* The ID is 16 bytes wide, addressed as four OTPC words. */
#define N32H7XX_UCID_WORDS	4

/*
 * Read the chip unique ID.  Fills @words[0..N32H7XX_UCID_WORDS-1] in
 * ascending OTPC word order and returns 0 on success, or a negative errno if
 * the OTP controller could not be read.  The result is read once and cached,
 * so this is cheap enough for probe paths.
 */
int n32h7xx_ucid_read(u32 words[N32H7XX_UCID_WORDS]);

#endif /* __LINUX_SOC_NSING_N32H7XX_UCID_H */
