// SPDX-License-Identifier: GPL-2.0-only
/*
 * NSING N32H7xx ARMv7-M bring-up idle workaround.
 * Keep WFI out of this diagnostic kernel until low-power wakeup is verified.
 * The earlier idle-dead symptom also occurred without WFI and was corrected by
 * the bootloader MPU guard; WFI clock gating is therefore not a proven cause.
 */
#include <linux/cpu.h>
#include <linux/init.h>
#include <asm/system_misc.h>

static void __ref n32h7xx_idle(void)
{
	cpu_relax();
}

static int __init n32h7xx_idle_init(void)
{
	arm_pm_idle = n32h7xx_idle;
	return 0;
}
device_initcall(n32h7xx_idle_init);
