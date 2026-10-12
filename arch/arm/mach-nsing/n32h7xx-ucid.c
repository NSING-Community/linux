// SPDX-License-Identifier: GPL-2.0-only
/*
 * NSING N32H7xx unique chip ID.
 *
 * Every N32H7xx has a 128-bit UCID burned into on-chip OTP.  It is *not*
 * memory mapped: the vendor CMSIS header advertises UCID_BASE = 0x1FFFC238
 * (and a commented out UID_BASE = 0x1FFFC760), but reading those addresses
 * faults, because they are not AHB targets.  The only access path is the OTPC
 * controller, which exposes OTP words through a small register block:
 *
 *   0x51118000 + 0x00  CTRL   bit1 PRMD: 1 = write, 0 = read
 *   0x51118000 + 0x04  STS    bit0 BUSY, bits[6:2] error flags
 *   0x51118000 + 0x10  ADDR   OTP word address
 *   0x51118000 + 0x14  RDATA  data of the last read
 *
 * A read is: wait for !BUSY, select read mode, write ADDR, wait for !BUSY
 * again, then take RDATA.  The UCID lives at OTPC word address 0x0238 and
 * spans four words (User Manual "UCID 读取"; matches the vendor
 * examples/OTPC/OtpcReadWrite sample, which does OTPC_ReadWord(0x0238 + n)).
 * The OTPC AHB clock gate is already open out of reset, so reading needs no
 * RCC access of its own.
 *
 * n32h7xx_ucid_read() is the exported accessor; the driver that needs a
 * per-board MAC address is one user.  The result is also published as the
 * "Serial" line of /proc/cpuinfo.  arch/arm's init_machine_late() fills
 * system_serial at initcall level 7, either from the DT root "serial-number"
 * property or from system_serial_low/high.  A plain arch_initcall would be
 * overwritten by that generic fallback, so this runs at level 7s, which the
 * linker script places after every level 7 initcall.
 */

#include <linux/bitops.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/printk.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <asm/system_info.h>
#include <linux/soc/nsing/n32h7xx-ucid.h>

#define N32H7XX_OTPC_BASE		0x51118000
#define N32H7XX_OTPC_CTRL		0x00
#define N32H7XX_OTPC_STS		0x04
#define N32H7XX_OTPC_ADDR		0x10
#define N32H7XX_OTPC_RDATA		0x14

#define N32H7XX_OTPC_CTRL_PRMD		BIT(1)
#define N32H7XX_OTPC_STS_BUSY		BIT(0)
#define N32H7XX_OTPC_STS_ERR		GENMASK(6, 2)

#define N32H7XX_OTPC_TIMEOUT_US		10000

/* First UCID word as an OTPC word address; the ID is 16 bytes = 4 words. */
#define N32H7XX_UCID_ADDR		0x0238

static int n32h7xx_otpc_read_word(void __iomem *otpc, u32 addr, u32 *val)
{
	u32 sts;
	int ret;

	ret = readl_poll_timeout(otpc + N32H7XX_OTPC_STS, sts,
				 !(sts & N32H7XX_OTPC_STS_BUSY), 1,
				 N32H7XX_OTPC_TIMEOUT_US);
	if (ret)
		return ret;

	/*
	 * The error flags are sticky and cleared by writing 1 to them.  The
	 * boot ROM leaves a stale read-protection error behind on this part,
	 * so a clean slate is required before judging the result.
	 */
	writel(N32H7XX_OTPC_STS_ERR, otpc + N32H7XX_OTPC_STS);

	writel(readl(otpc + N32H7XX_OTPC_CTRL) & ~N32H7XX_OTPC_CTRL_PRMD,
	       otpc + N32H7XX_OTPC_CTRL);
	writel(addr, otpc + N32H7XX_OTPC_ADDR);

	ret = readl_poll_timeout(otpc + N32H7XX_OTPC_STS, sts,
				 !(sts & N32H7XX_OTPC_STS_BUSY), 1,
				 N32H7XX_OTPC_TIMEOUT_US);
	if (ret)
		return ret;
	if (sts & N32H7XX_OTPC_STS_ERR)
		return -EIO;

	*val = readl(otpc + N32H7XX_OTPC_RDATA);
	return 0;
}

static DEFINE_MUTEX(n32h7xx_ucid_lock);
static u32 n32h7xx_ucid_cache[N32H7XX_UCID_WORDS];
static bool n32h7xx_ucid_cached;
static int n32h7xx_ucid_errno;		/* latched read error, 0 = none */

int n32h7xx_ucid_read(u32 words[N32H7XX_UCID_WORDS])
{
	void __iomem *otpc;
	u32 ucid[N32H7XX_UCID_WORDS];
	int i, ret;

	mutex_lock(&n32h7xx_ucid_lock);

	if (n32h7xx_ucid_cached) {
		memcpy(words, n32h7xx_ucid_cache, sizeof(n32h7xx_ucid_cache));
		ret = 0;
		goto out;
	}
	if (n32h7xx_ucid_errno) {
		ret = n32h7xx_ucid_errno;
		goto out;
	}

	otpc = ioremap(N32H7XX_OTPC_BASE, 0x4000);
	if (!otpc) {
		ret = -ENOMEM;
		goto fail;
	}

	for (i = 0; i < N32H7XX_UCID_WORDS; i++) {
		ret = n32h7xx_otpc_read_word(otpc, N32H7XX_UCID_ADDR + i,
					     &ucid[i]);
		if (ret) {
			pr_warn("n32h7xx: UCID read failed at OTPC word %#x (%d)\n",
				N32H7XX_UCID_ADDR + i, ret);
			iounmap(otpc);
			goto fail;
		}
	}
	iounmap(otpc);

	memcpy(n32h7xx_ucid_cache, ucid, sizeof(n32h7xx_ucid_cache));
	n32h7xx_ucid_cached = true;
	memcpy(words, n32h7xx_ucid_cache, sizeof(n32h7xx_ucid_cache));
	ret = 0;
	goto out;

fail:
	n32h7xx_ucid_errno = ret;
out:
	mutex_unlock(&n32h7xx_ucid_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(n32h7xx_ucid_read);

static int __init n32h7xx_ucid_init(void)
{
	struct device_node *root;
	u32 ucid[N32H7XX_UCID_WORDS];
	int ret;

	if (!of_machine_is_compatible("nsing,n32h7xx"))
		return 0;

	/* A serial number supplied by the firmware/bootloader wins. */
	root = of_find_node_by_path("/");
	if (root && of_property_present(root, "serial-number")) {
		of_node_put(root);
		return 0;
	}
	of_node_put(root);

	ret = n32h7xx_ucid_read(ucid);
	if (ret)
		return ret;

	/*
	 * The 128-bit ID does not fit the 64-bit system_serial_low/high pair,
	 * so only the string is installed; it is what /proc/cpuinfo prints.
	 * Words are printed in ascending OTPC address order.
	 */
	kfree(system_serial);
	system_serial = kasprintf(GFP_KERNEL, "%08x%08x%08x%08x",
				  ucid[0], ucid[1], ucid[2], ucid[3]);
	if (!system_serial)
		return -ENOMEM;

	return 0;
}
late_initcall_sync(n32h7xx_ucid_init);
