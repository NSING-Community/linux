// SPDX-License-Identifier: GPL-2.0-only
/*
 * NSING ARMv7-M platform support
 */

#include <linux/io.h>
#include <linux/of_platform.h>
#include <linux/reboot.h>
#include <linux/string.h>
#include <asm/barrier.h>
#include <asm/mach/arch.h>
#include <asm/v7m.h>

static const char *const nsing_dt_compat[] __initconst = {
	"nsing,n32h787",
	"nsing,n32h7xx",
	NULL
};

/*
 * `reboot msc` (LINUX_REBOOT_CMD_RESTART2) asks the CM7 bootloader in on-chip
 * flash to run its USB mass-storage mode instead of starting Linux.  That is
 * how the SPI NOR boot medium gets rewritten without reaching for the probe or
 * KEY1, and the whole protocol is two words in retained internal SRAM:
 *
 *	0x30015f00 = 0x544f4f42 ("BOOT")
 *	0x30015f04 = ~0x544f4f42
 *
 * bootloader/CM7/src/boot_app.c implements the other side and clears both words
 * before it acts on them, so an ordinary reset leaves them zeroed and boots
 * Linux as usual.  SRAM survives a system reset; the MPU background region maps
 * the whole address space strongly-ordered, so these stores reach the memory
 * instead of sitting in a cache.
 */
#define N32H7XX_MSC_FLAG_ADDR	0x30015f00
#define N32H7XX_MSC_FLAG_MAGIC	0x544f4f42	/* "BOOT" */

static void n32h7xx_restart(enum reboot_mode mode, const char *cmd)
{
	u32 __iomem *flag = (u32 __iomem *)N32H7XX_MSC_FLAG_ADDR;

	if (cmd && !strcmp(cmd, "msc")) {
		writel(N32H7XX_MSC_FLAG_MAGIC, flag);
		writel((u32)~N32H7XX_MSC_FLAG_MAGIC, flag + 1);
		dsb();
		pr_emerg("Restarting into USB MSC mode\n");
	}

	armv7m_restart(mode, cmd);
}

DT_MACHINE_START(NSING_N32H7XX, "NSING N32H7xx (Device Tree Support)")
	.dt_compat = nsing_dt_compat,
	.restart = n32h7xx_restart,
MACHINE_END
