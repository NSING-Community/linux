// SPDX-License-Identifier: GPL-2.0-only
/*
 * NSING N32H7xx RTC driver.
 *
 * The RTC is a BCD calendar counter in the backup domain, clocked from the
 * 32.768 kHz LSE crystal and divided down to 1 Hz.  Starting that oscillator
 * and selecting it as the RTC clock is series hardware bring-up and lives in
 * arch/arm/mach-nsing; this driver only programs the calendar registers.
 *
 * Power model: there is no VBAT battery, the backup domain is fed from VDD,
 * so a warm reset keeps counting while a power cycle clears the calendar and
 * the backup registers together.  BKP1 holds a magic word meaning "somebody
 * set this calendar since the backup domain last lost power"; without it
 * read_time() returns -EINVAL instead of a meaningless year, which leaves
 * the kernel at the epoch until NTP steps it, and lets a later "hwclock -w"
 * make the value trustworthy again.
 */

#include <linux/bcd.h>
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>
#include <linux/soc/nsing/n32h7xx-rtc.h>

#define N32H7XX_RTC_TSH			0x00
#define N32H7XX_RTC_DATE		0x04
#define N32H7XX_RTC_CTRL		0x08
#define N32H7XX_RTC_INITSTS		0x0c
#define N32H7XX_RTC_PRE			0x10
#define N32H7XX_RTC_WRP			0x24
#define N32H7XX_RTC_SUBS		0x28
#define N32H7XX_RTC_CALIB		0x3c
#define N32H7XX_RTC_BKP			0x50

/* "RTCK": BKP1 says the calendar was set since the last backup reset. */
#define N32H7XX_RTC_BKP_MAGIC		0x5254434b

/*
 * Calendar fields.  The reserved masks leave out the bits the peripheral
 * does not define, and are the same ones the vendor library masks with.
 */
#define N32H7XX_RTC_TSH_MASK		(BIT(22) | GENMASK(21, 16) | \
					 GENMASK(14, 8) | GENMASK(6, 0))
#define N32H7XX_RTC_DATE_MASK		(GENMASK(23, 16) | GENMASK(15, 13) | \
					 GENMASK(12, 8) | GENMASK(5, 0))

#define N32H7XX_RTC_TSH_HOUR		GENMASK(21, 16)
#define N32H7XX_RTC_TSH_MIN		GENMASK(14, 8)
#define N32H7XX_RTC_TSH_SEC		GENMASK(6, 0)
#define N32H7XX_RTC_DATE_YEAR		GENMASK(23, 16)
#define N32H7XX_RTC_DATE_WDAY		GENMASK(15, 13)
#define N32H7XX_RTC_DATE_MON		GENMASK(12, 8)
#define N32H7XX_RTC_DATE_DAY		GENMASK(5, 0)

/* CTRL.HFMT selects the 12-hour calendar; clear keeps it 24-hour. */
#define N32H7XX_RTC_CTRL_HFMT		BIT(6)

#define N32H7XX_RTC_INITSTS_INITM	BIT(7)
#define N32H7XX_RTC_INITSTS_INITF	BIT(6)
#define N32H7XX_RTC_INITSTS_RSYF	BIT(5)
#define N32H7XX_RTC_INITSTS_RECPF	BIT(16)

#define N32H7XX_RTC_CALIB_CP		BIT(15)
#define N32H7XX_RTC_CALIB_CM		GENMASK(8, 0)
#define N32H7XX_RTC_CALIB_CYCLES		1048576

/* PRE: synchronous prescaler in [14:0], asynchronous in [22:16]. */
#define N32H7XX_RTC_PRE_SYNC		GENMASK(14, 0)
#define N32H7XX_RTC_PRE_LSE_1HZ \
	(FIELD_PREP(N32H7XX_RTC_PRE_SYNC, 0xff) | (0x7f << 16))

/* 1 January 2000, a Monday. */
#define N32H7XX_RTC_DATE_EPOCH \
	(FIELD_PREP(N32H7XX_RTC_DATE_WDAY, 1) | \
	 FIELD_PREP(N32H7XX_RTC_DATE_MON, 1) | \
	 FIELD_PREP(N32H7XX_RTC_DATE_DAY, 1))

#define N32H7XX_RTC_POLL_TIMEOUT_US	1000000
#define N32H7XX_RTC_INIT_TRIES		100000
#define N32H7XX_RTC_READ_TRIES		4

/* Years the RTC can represent, as tm_year counts from 1900. */
#define N32H7XX_RTC_MIN_YEAR		100
#define N32H7XX_RTC_MAX_YEAR		199

struct n32h7xx_rtc {
	struct rtc_device *rtc;
	void __iomem *base;
	/* Serialises the read/write sequences against each other. */
	struct mutex lock;
};

static void n32h7xx_rtc_unlock(void __iomem *base)
{
	writel(0xca, base + N32H7XX_RTC_WRP);
	writel(0x53, base + N32H7XX_RTC_WRP);
}

static void n32h7xx_rtc_lock(void __iomem *base)
{
	writel(0xff, base + N32H7XX_RTC_WRP);
}

static int n32h7xx_rtc_enter_init(struct device *dev, void __iomem *base)
{
	u32 pre, subs, val;
	unsigned int tries;
	int ret;

	if (readl(base + N32H7XX_RTC_INITSTS) & N32H7XX_RTC_INITSTS_INITF)
		return 0;

	/*
	 * The sub-second counter has to be at least 3 and different from the
	 * synchronous prescaler before INITM is accepted; that is the
	 * documented way to stop the first second after entering init mode
	 * from being lost.
	 */
	pre = readl(base + N32H7XX_RTC_PRE) & N32H7XX_RTC_PRE_SYNC;
	for (tries = 0; tries < N32H7XX_RTC_INIT_TRIES; tries++) {
		subs = readl(base + N32H7XX_RTC_SUBS) & N32H7XX_RTC_PRE_SYNC;
		if (subs >= 3 && subs != pre)
			break;
		udelay(1);
	}
	if (tries == N32H7XX_RTC_INIT_TRIES)
		return dev_err_probe(dev, -ETIMEDOUT,
				     "RTC sub-second counter never moved\n");

	writel(N32H7XX_RTC_INITSTS_INITM, base + N32H7XX_RTC_INITSTS);

	ret = readl_poll_timeout(base + N32H7XX_RTC_INITSTS, val,
				 val & N32H7XX_RTC_INITSTS_INITF, 10,
				 N32H7XX_RTC_POLL_TIMEOUT_US);
	if (ret)
		return dev_err_probe(dev, ret, "RTC did not enter init mode\n");

	return 0;
}

static void n32h7xx_rtc_exit_init(void __iomem *base)
{
	u32 reg = readl(base + N32H7XX_RTC_INITSTS);

	writel(reg & ~N32H7XX_RTC_INITSTS_INITM, base + N32H7XX_RTC_INITSTS);
}

/*
 * The calendar shadow registers only follow the counter once the APB clock
 * has been running for a bit; wait for RSYF rather than reading a stale
 * value, which is what happens right after the RTC is first enabled.
 */
static int n32h7xx_rtc_wait_sync(struct device *dev, void __iomem *base)
{
	u32 val;
	int ret;

	n32h7xx_rtc_unlock(base);
	/* Writing a zero clears the flag; keep the other INITSTS bits as read. */
	writel(readl(base + N32H7XX_RTC_INITSTS) & ~N32H7XX_RTC_INITSTS_RSYF,
	       base + N32H7XX_RTC_INITSTS);

	ret = readl_poll_timeout(base + N32H7XX_RTC_INITSTS, val,
				 val & N32H7XX_RTC_INITSTS_RSYF, 10,
				 N32H7XX_RTC_POLL_TIMEOUT_US);
	n32h7xx_rtc_lock(base);

	if (ret)
		return dev_err_probe(dev, ret, "RTC shadow registers never synced\n");

	return 0;
}

/* Program the prescaler and give the untrusted counter a defined start. */
static int n32h7xx_rtc_init_calendar(struct device *dev)
{
	struct n32h7xx_rtc *rtc = dev_get_drvdata(dev);
	void __iomem *base = rtc->base;
	u32 reg;
	int ret;

	n32h7xx_rtc_unlock(base);

	ret = n32h7xx_rtc_enter_init(dev, base);
	if (ret)
		goto out;

	reg = readl(base + N32H7XX_RTC_CTRL) & ~N32H7XX_RTC_CTRL_HFMT;
	writel(reg, base + N32H7XX_RTC_CTRL);
	/* 32768 Hz / (0xff + 1) / (0x7f + 1) = 1 Hz. */
	writel(N32H7XX_RTC_PRE_LSE_1HZ, base + N32H7XX_RTC_PRE);
	writel(0, base + N32H7XX_RTC_TSH);
	writel(N32H7XX_RTC_DATE_EPOCH, base + N32H7XX_RTC_DATE);

	n32h7xx_rtc_exit_init(base);
out:
	n32h7xx_rtc_lock(base);

	if (ret)
		return ret;

	return n32h7xx_rtc_wait_sync(dev, base);
}

static int n32h7xx_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct n32h7xx_rtc *rtc = dev_get_drvdata(dev);
	void __iomem *base = rtc->base;
	u32 tsh, tsh_check, date;
	unsigned int tries = 0;

	guard(mutex)(&rtc->lock);

	if (readl(base + N32H7XX_RTC_BKP) != N32H7XX_RTC_BKP_MAGIC) {
		dev_warn_ratelimited(dev,
				     "no trusted calendar since the backup domain lost power\n");
		return -EINVAL;
	}

	/*
	 * Reading TSH latches the shadow registers and reading DATE releases
	 * them again, so those two are one consistent sample.  Sampling TSH
	 * once more catches the rollover that would pair an old date with a
	 * new time.
	 */
	do {
		tsh = readl(base + N32H7XX_RTC_TSH);
		date = readl(base + N32H7XX_RTC_DATE);
		tsh_check = readl(base + N32H7XX_RTC_TSH);
	} while (tsh != tsh_check && ++tries < N32H7XX_RTC_READ_TRIES);

	/* Release the shadow registers the last read of TSH just latched. */
	readl(base + N32H7XX_RTC_DATE);

	if (tsh != tsh_check)
		return -EIO;

	tsh &= N32H7XX_RTC_TSH_MASK;
	date &= N32H7XX_RTC_DATE_MASK;

	tm->tm_sec = bcd2bin(FIELD_GET(N32H7XX_RTC_TSH_SEC, tsh));
	tm->tm_min = bcd2bin(FIELD_GET(N32H7XX_RTC_TSH_MIN, tsh));
	tm->tm_hour = bcd2bin(FIELD_GET(N32H7XX_RTC_TSH_HOUR, tsh));
	tm->tm_mday = bcd2bin(FIELD_GET(N32H7XX_RTC_DATE_DAY, date));
	tm->tm_mon = bcd2bin(FIELD_GET(N32H7XX_RTC_DATE_MON, date)) - 1;
	tm->tm_year = bcd2bin(FIELD_GET(N32H7XX_RTC_DATE_YEAR, date)) +
		      N32H7XX_RTC_MIN_YEAR;
	tm->tm_wday = FIELD_GET(N32H7XX_RTC_DATE_WDAY, date);

	return 0;
}

static int n32h7xx_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct n32h7xx_rtc *rtc = dev_get_drvdata(dev);
	void __iomem *base = rtc->base;
	u32 reg;
	int ret;

	if (tm->tm_year < N32H7XX_RTC_MIN_YEAR ||
	    tm->tm_year > N32H7XX_RTC_MAX_YEAR)
		return -EINVAL;

	guard(mutex)(&rtc->lock);

	n32h7xx_rtc_unlock(base);
	ret = n32h7xx_rtc_enter_init(dev, base);
	if (ret)
		goto out;

	reg = FIELD_PREP(N32H7XX_RTC_TSH_HOUR, bin2bcd(tm->tm_hour)) |
	      FIELD_PREP(N32H7XX_RTC_TSH_MIN, bin2bcd(tm->tm_min)) |
	      FIELD_PREP(N32H7XX_RTC_TSH_SEC, bin2bcd(tm->tm_sec));
	writel(reg, base + N32H7XX_RTC_TSH);

	reg = FIELD_PREP(N32H7XX_RTC_DATE_YEAR,
			 bin2bcd(tm->tm_year - N32H7XX_RTC_MIN_YEAR)) |
	      FIELD_PREP(N32H7XX_RTC_DATE_MON, bin2bcd(tm->tm_mon + 1)) |
	      FIELD_PREP(N32H7XX_RTC_DATE_DAY, bin2bcd(tm->tm_mday)) |
	      FIELD_PREP(N32H7XX_RTC_DATE_WDAY, tm->tm_wday);
	writel(reg, base + N32H7XX_RTC_DATE);

	n32h7xx_rtc_exit_init(base);

	/*
	 * The calendar means something again, so a warm reset can trust it.
	 * BKP1 is covered by the same WRP lock as the calendar registers, so
	 * this has to happen before the lock goes back on.
	 */
	writel(N32H7XX_RTC_BKP_MAGIC, base + N32H7XX_RTC_BKP);
	readl(base + N32H7XX_RTC_BKP);
out:
	n32h7xx_rtc_lock(base);

	if (ret)
		return ret;

	return n32h7xx_rtc_wait_sync(dev, base);
}

static int n32h7xx_rtc_read_offset(struct device *dev, long *offset)
{
	struct n32h7xx_rtc *rtc = dev_get_drvdata(dev);
	s32 pulses;
	u32 val;

	guard(mutex)(&rtc->lock);
	val = readl(rtc->base + N32H7XX_RTC_CALIB);
	/* Shorter windows mask the low CM bits, preserving this ratio. */
	pulses = (val & N32H7XX_RTC_CALIB_CP ? 512 : 0) -
		 FIELD_GET(N32H7XX_RTC_CALIB_CM, val);
	*offset = div_s64((s64)pulses * 1000000000,
			  N32H7XX_RTC_CALIB_CYCLES - pulses);
	return 0;
}

static int n32h7xx_rtc_set_offset(struct device *dev, long offset)
{
	struct n32h7xx_rtc *rtc = dev_get_drvdata(dev);
	s32 pulses;
	u32 val;
	int ret;

	/* ppb limits for removing 511 or inserting 512 pulses per 32 s. */
	if (offset < -487089 || offset > 488520)
		return -ERANGE;

	pulses = DIV_ROUND_CLOSEST_ULL((u64)abs(offset) *
				     N32H7XX_RTC_CALIB_CYCLES,
				     1000000000L + offset);
	if (offset < 0)
		pulses = -pulses;
	guard(mutex)(&rtc->lock);
	ret = readl_poll_timeout(rtc->base + N32H7XX_RTC_INITSTS, val,
				 !(val & N32H7XX_RTC_INITSTS_RECPF), 10,
				 N32H7XX_RTC_POLL_TIMEOUT_US);
	if (ret)
		return ret;
	if (val & N32H7XX_RTC_INITSTS_INITF)
		return -EBUSY;

	val = pulses > 0 ? N32H7XX_RTC_CALIB_CP | (512 - pulses) : -pulses;
	n32h7xx_rtc_unlock(rtc->base);
	/* Use the full 32-second window for the finest resolution. */
	writel(val, rtc->base + N32H7XX_RTC_CALIB);
	n32h7xx_rtc_lock(rtc->base);
	return 0;
}

static const struct rtc_class_ops n32h7xx_rtc_ops = {
	.read_time = n32h7xx_rtc_read_time,
	.set_time = n32h7xx_rtc_set_time,
	.read_offset = n32h7xx_rtc_read_offset,
	.set_offset = n32h7xx_rtc_set_offset,
};

static int n32h7xx_rtc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct n32h7xx_rtc *rtc;
	int ret;

	rtc = devm_kzalloc(dev, sizeof(*rtc), GFP_KERNEL);
	if (!rtc)
		return -ENOMEM;

	rtc->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(rtc->base))
		return PTR_ERR(rtc->base);

	platform_set_drvdata(pdev, rtc);
	mutex_init(&rtc->lock);

	ret = n32h7xx_rtc_soc_init(dev);
	if (ret)
		return ret;

	if (readl(rtc->base + N32H7XX_RTC_BKP) == N32H7XX_RTC_BKP_MAGIC) {
		ret = n32h7xx_rtc_wait_sync(dev, rtc->base);
		if (ret)
			return ret;
		dev_info(dev, "calendar kept across the reset\n");
	} else {
		ret = n32h7xx_rtc_init_calendar(dev);
		if (ret)
			return ret;
		dev_info(dev,
			 "backup domain was reset; counting from 2000-01-01 until the clock is set\n");
	}

	rtc->rtc = devm_rtc_allocate_device(dev);
	if (IS_ERR(rtc->rtc))
		return PTR_ERR(rtc->rtc);

	rtc->rtc->ops = &n32h7xx_rtc_ops;
	rtc->rtc->range_min = RTC_TIMESTAMP_BEGIN_2000;
	rtc->rtc->range_max = RTC_TIMESTAMP_END_2099;

	return devm_rtc_register_device(rtc->rtc);
}

static const struct of_device_id n32h7xx_rtc_of_match[] = {
	{ .compatible = "nsing,n32h7xx-rtc" },
	{ }
};
MODULE_DEVICE_TABLE(of, n32h7xx_rtc_of_match);

static struct platform_driver n32h7xx_rtc_driver = {
	.probe = n32h7xx_rtc_probe,
	.driver = {
		.name = "n32h7xx-rtc",
		.of_match_table = n32h7xx_rtc_of_match,
	},
};
module_platform_driver(n32h7xx_rtc_driver);

MODULE_AUTHOR("NSING");
MODULE_DESCRIPTION("NSING N32H7xx RTC driver");
MODULE_LICENSE("GPL");
