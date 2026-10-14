// SPDX-License-Identifier: GPL-2.0-only
/*
 * NSING N32H7xx DVP parallel camera capture.
 *
 * The DVP block is an AHB master, not a DMA-controller client: the CPU hands
 * it the address of a frame buffer and the port writes the pixels into it.
 * Capture is therefore one frame at a time.  The SoC side of the block (the
 * GRAPHICS power sub-domain, the AHB/APB clock gates, the port reset and the
 * MMU module-memory bit that lets the DVP reach SDRAM) lives in
 * arch/arm/mach-nsing/n32h7xx-dvp.c and is invoked through
 * n32h7xx_dvp_soc_init(); this file only programs the port and drives V4L2.
 *
 * The sensor behind the port is a normal V4L2 subdevice.  It is picked up
 * through the device tree so this driver does not have to know which sensor
 * is fitted; the formats below are the media-bus codes a sensor can send over
 * the 8-bit parallel bus and that userspace can consume as-is.
 */

#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/gfp.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/soc/nsing/n32h7xx-dvp.h>
#include <linux/spinlock.h>
#include <linux/videodev2.h>

#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-vmalloc.h>

/* DVP register block, offsets straight out of the N32H7xx reference manual. */
#define N32H7XX_DVP_CTRL		0x00
#define N32H7XX_DVP_INTEN		0x04
#define N32H7XX_DVP_INTSTS		0x08
#define N32H7XX_DVP_PORTCFG		0x0c
#define N32H7XX_DVP_FIFOCFG		0x10
#define N32H7XX_DVP_SMADDR1		0x14
#define N32H7XX_DVP_FBS			0x1c
#define N32H7XX_DVP_FPBC1		0x20

/* CTRL */
#define N32H7XX_DVP_CTRL_DVPEN		BIT(0)
#define N32H7XX_DVP_CTRL_CM		BIT(1)	/* 1 = capture a single frame */
/* VSYNC blanking glitch filter width, 4 bits. */
#define N32H7XX_DVP_CTRL_VBFLT		GENMASK(31, 28)

/* INTEN */
#define N32H7XX_DVP_INT_M1TCIE		BIT(2)
#define N32H7XX_DVP_INT_AHBERRIE	BIT(7)
#define N32H7XX_DVP_INT_FOIE		BIT(5)

/*
 * INTSTS.  Write-1-to-clear, except DVPDIS (bit 24) which is how the port is
 * told to stop: clearing CTRL.DVPEN is not possible, the only disable path is
 * this bit.
 */
#define N32H7XX_DVP_INTSTS_M1SF		BIT(0)
#define N32H7XX_DVP_INTSTS_M1TCF	BIT(2)
#define N32H7XX_DVP_INTSTS_M1FOF	BIT(4)
#define N32H7XX_DVP_INTSTS_AHBERR1F	BIT(6)
#define N32H7XX_DVP_INTSTS_M1OF		BIT(8)
#define N32H7XX_DVP_INTSTS_CERRF	BIT(17)
#define N32H7XX_DVP_INTSTS_SERRF	BIT(18)
#define N32H7XX_DVP_INTSTS_DVPDIS	BIT(24)

/*
 * Flags that can be pending when a capture attempt finishes and have to be
 * cleared before the next one.  M1O is deliberately not treated as a failure:
 * the hardware raises it on the way to a normal single-shot completion.
 */
#define N32H7XX_DVP_ATTEMPT_FLAGS					\
	(N32H7XX_DVP_INTSTS_M1SF | N32H7XX_DVP_INTSTS_M1TCF |		\
	 N32H7XX_DVP_INTSTS_M1FOF | N32H7XX_DVP_INTSTS_AHBERR1F |	\
	 N32H7XX_DVP_INTSTS_M1OF | N32H7XX_DVP_INTSTS_CERRF |		\
	 N32H7XX_DVP_INTSTS_SERRF)

/* Flags that mean the frame in flight is unusable. */
#define N32H7XX_DVP_FATAL_FLAGS						\
	(N32H7XX_DVP_INTSTS_M1FOF | N32H7XX_DVP_INTSTS_AHBERR1F |	\
	 N32H7XX_DVP_INTSTS_CERRF | N32H7XX_DVP_INTSTS_SERRF)

/* PORTCFG: pixel clock rising edge, HSYNC low, VSYNC high, 8-bit data mode 1. */
#define N32H7XX_DVP_PORTCFG_PCLK_FALL	BIT(0)
#define N32H7XX_DVP_PORTCFG_HSYNC_LOW	BIT(1)
#define N32H7XX_DVP_PORTCFG_VSYNC_HIGH	BIT(2)
#define N32H7XX_DVP_PORTCFG_EMBSEN	BIT(7)
#define N32H7XX_DVP_PORTCFG_DBIT	GENMASK(10, 8)
#define N32H7XX_DVP_PORTCFG_DBIT_M1	0
#define N32H7XX_DVP_PORTCFG_DBIT_M5	(4 << 8)	/* 16-bit physical port */
#define N32H7XX_DVP_PORTCFG_PIXELDB	BIT(12)
/*
 * The PORTCFG fields software may program.  BMAP (the byte reordering inside
 * each group of four received bytes) stays reset: it is only meaningful for
 * the narrow 8-bit modes and nothing here needs it.
 */
#define N32H7XX_DVP_PORTCFG_RW_MASK					\
	(N32H7XX_DVP_PORTCFG_PCLK_FALL | N32H7XX_DVP_PORTCFG_HSYNC_LOW | \
	 N32H7XX_DVP_PORTCFG_VSYNC_HIGH | N32H7XX_DVP_PORTCFG_EMBSEN | \
	 N32H7XX_DVP_PORTCFG_DBIT | N32H7XX_DVP_PORTCFG_PIXELDB)
#define N32H7XX_DVP_PORTCFG_DEFAULT					\
	(N32H7XX_DVP_PORTCFG_HSYNC_LOW | N32H7XX_DVP_PORTCFG_VSYNC_HIGH | \
	 N32H7XX_DVP_PORTCFG_DBIT_M1)

/* FIFOCFG */
#define N32H7XX_DVP_M1ADDREN		BIT(0)
#define N32H7XX_DVP_TXFTH		GENMASK(10, 8)
#define N32H7XX_DVP_TXBURSZ		GENMASK(18, 16)

/* FBS is 22 bits wide; asking for more would silently wrap. */
#define N32H7XX_DVP_MAX_FRAME_SIZE	0x3fffff

#define N32H7XX_DVP_DRV_NAME		"n32h7xx-dvp"
#define N32H7XX_DVP_MIN_WIDTH		16
#define N32H7XX_DVP_MIN_HEIGHT		16
#define N32H7XX_DVP_MAX_WIDTH		2592
#define N32H7XX_DVP_MAX_HEIGHT		1944
#define N32H7XX_DVP_DEFAULT_WIDTH	320
#define N32H7XX_DVP_DEFAULT_HEIGHT	240

struct n32h7xx_dvp_format {
	u32	pixelformat;
	u32	mbus_code;
	u8	bpp;
};

/*
 * The formats the DVP can hand to userspace unchanged.  Each one is a straight
 * byte stream taken off the parallel bus; the sensor subdevice is responsible
 * for producing the matching media-bus format.
 */
static const struct n32h7xx_dvp_format n32h7xx_dvp_formats[] = {
	{
		.pixelformat	= V4L2_PIX_FMT_GREY,
		.mbus_code	= MEDIA_BUS_FMT_Y8_1X8,
		.bpp		= 8,
	}, {
		.pixelformat	= V4L2_PIX_FMT_RGB565,
		.mbus_code	= MEDIA_BUS_FMT_RGB565_2X8_LE,
		.bpp		= 16,
	}, {
		.pixelformat	= V4L2_PIX_FMT_YUYV,
		.mbus_code	= MEDIA_BUS_FMT_YUYV8_2X8,
		.bpp		= 16,
	}, {
		.pixelformat	= V4L2_PIX_FMT_JPEG,
		.mbus_code	= MEDIA_BUS_FMT_JPEG_1X8,
		.bpp		= 16,
	},
};

struct n32h7xx_dvp_buffer {
	struct vb2_v4l2_buffer	vb;
	struct list_head	entry;
};

struct n32h7xx_dvp {
	struct device		*dev;
	void __iomem		*base;
	int			irq;

	struct v4l2_device	v4l2_dev;
	struct v4l2_async_notifier notifier;
	struct v4l2_subdev	*sensor;

	struct video_device	vdev;
	struct vb2_queue	queue;
	struct mutex		lock;		/* serialises the ioctls */

	const struct n32h7xx_dvp_format *fmt;
	struct v4l2_pix_format	pix;

	/*
	 * True when the frame buffers come from the board's reserved coherent
	 * pool instead of the page allocator; decided once at probe time and
	 * what the buffer allocation, the address of a buffer and the size
	 * probe all follow.
	 */
	bool			pooled;

	spinlock_t		queue_lock;	/* protects the two below */
	struct list_head	buffers;	/* queued and not yet armed */
	struct n32h7xx_dvp_buffer *armed;	/* buffer the hardware writes to */
	bool			streaming;
};

static const struct n32h7xx_dvp_format *
n32h7xx_dvp_find_pixelformat(u32 pixelformat)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(n32h7xx_dvp_formats); i++)
		if (n32h7xx_dvp_formats[i].pixelformat == pixelformat)
			return &n32h7xx_dvp_formats[i];

	return NULL;
}

/*
 * Frame buffer budget.
 *
 * There is no MMU on this target and the port writes a whole frame in one
 * go, so every capture buffer has to be one physically contiguous block
 * that the running system can actually hand out.
 *
 * The board reserves that memory as the port's "memory-region": a fixed run
 * of SDRAM the allocator cannot fragment away, large enough for the biggest
 * frame the port can address.  A board that reserves nothing still gets a
 * working port through the ordinary allocator, but then the limit is
 * whatever contiguous block it can produce at the time -- on this SoC a
 * couple of megabytes that vary with how long the board has been up.
 *
 * One number covers both kinds of format:
 *
 *  - JPEG is variable length, so the budget is simply the largest buffer one
 *    frame may use.  The port stops writing at the programmed size and raises
 *    M1OF when a frame needs more; the interrupt handler turns that into a
 *    failed frame, so a scene that really does overflow the budget is
 *    reported rather than silently truncated.  A compressed frame therefore
 *    reaches any resolution whose buffer fits, which is what makes the
 *    sensor's 2592x1944 mode usable: the pool is sized for it.
 *  - The uncompressed formats are exactly one frame and nothing more, so the
 *    budget is what limits the resolution: a request that would need a larger
 *    buffer is scaled down to the largest one that fits (see
 *    n32h7xx_dvp_negotiate() and n32h7xx_dvp_fit_frame()).
 *
 * The ceiling in either case is the port itself: FBS is a 22-bit size, so
 * no single frame buffer can be larger than 4 MiB.
 *
 * Zero means "work it out at load time" from the reserved pool, or from the
 * largest physically contiguous allocation the driver can get at probe time
 * when the board reserves nothing.  Set it lower to keep more of the pool
 * free; it cannot be raised past what the pool can serve.
 */
static uint n32h7xx_dvp_max_frame_bytes;
module_param_named(max_frame_bytes, n32h7xx_dvp_max_frame_bytes, uint, 0644);
MODULE_PARM_DESC(max_frame_bytes,
		 "largest capture buffer to allocate, in bytes "
		 "(0 = as much as the reserved pool or the largest "
		 "contiguous block can serve)");

#define N32H7XX_DVP_MIN_FRAME_BYTES	(64 * 1024)
#define N32H7XX_DVP_PROBE_MAX_BYTES	(4 * 1024 * 1024)

/* Resolved budget; only the probe result is cached. */
static u32 n32h7xx_dvp_probed_bytes = N32H7XX_DVP_MIN_FRAME_BYTES;

/*
 * Largest frame buffer that can actually be handed to the port.
 *
 * A capture buffer has to be one physically contiguous run, so ask the
 * allocator itself instead of guessing from free-page statistics.  Halving
 * from the probe ceiling finds the answer in a handful of attempts, and each
 * one is released immediately so the caller gets the memory back.
 *
 * @pooled selects which allocator the buffers will come from, so the probe
 * measures the same thing the capture path will use: the reserved coherent
 * pool when the board provides one, the page allocator otherwise.
 */
static u32 n32h7xx_dvp_allocatable_bytes(struct device *dev, bool pooled)
{
	u32 bytes;

	for (bytes = N32H7XX_DVP_PROBE_MAX_BYTES;
	     bytes > N32H7XX_DVP_MIN_FRAME_BYTES; bytes >>= 1) {
		dma_addr_t handle = 0;
		void *block;

		if (pooled)
			block = dma_alloc_coherent(dev, bytes, &handle,
						   GFP_KERNEL | __GFP_NOWARN);
		else
			block = alloc_pages_exact(bytes,
						  GFP_KERNEL | __GFP_NOWARN);
		if (!block)
			continue;

		if (pooled)
			dma_free_coherent(dev, bytes, block, handle);
		else
			free_pages_exact(block, bytes);

		return min_t(u32, bytes, N32H7XX_DVP_MAX_FRAME_SIZE);
	}

	return N32H7XX_DVP_MIN_FRAME_BYTES;
}

static u32 n32h7xx_dvp_frame_budget(void)
{
	if (n32h7xx_dvp_max_frame_bytes)
		return clamp_t(u32, n32h7xx_dvp_max_frame_bytes,
			       N32H7XX_DVP_MIN_FRAME_BYTES,
			       n32h7xx_dvp_probed_bytes);

	return n32h7xx_dvp_probed_bytes;
}

/* Size of one frame buffer for @fmt at the given resolution. */
static u32 n32h7xx_dvp_sizeimage(const struct n32h7xx_dvp_format *fmt,
				 u32 width, u32 height)
{
	u32 raw = width * height * fmt->bpp / 8;

	/*
	 * The raw formats are one frame of uncompressed pixels and nothing
	 * more, so their buffer is exactly the frame.  A JPEG frame is
	 * variable length and always shorter than the uncompressed image it
	 * was made from, so its buffer is the budget rather than two bytes
	 * per pixel: the budget is both the ceiling a frame may reach and
	 * the largest buffer the port can be given.
	 */
	if (fmt->pixelformat == V4L2_PIX_FMT_JPEG)
		return min(raw, n32h7xx_dvp_frame_budget());

	return raw;
}

/*
 * Scale @width x @height down until one uncompressed frame fits the budget,
 * keeping the aspect ratio.  Both edges shrink by the same factor, so a few
 * percent below the largest that would fit is as close as this gets without
 * a 64-bit division, which a module cannot use on this target.  Compressed
 * frames keep their resolution: their buffer is a ceiling, not the frame, so
 * the budget does not limit them.
 */
static void n32h7xx_dvp_fit_frame(const struct n32h7xx_dvp_format *fmt,
				  u32 budget, u32 *width, u32 *height)
{
	if (fmt->pixelformat == V4L2_PIX_FMT_JPEG)
		return;

	while ((u32)(*width * *height) * fmt->bpp / 8 > budget) {
		if (*width <= N32H7XX_DVP_MIN_WIDTH &&
		    *height <= N32H7XX_DVP_MIN_HEIGHT)
			break;

		*width = max_t(u32, *width * 15 / 16,
			       (u32)N32H7XX_DVP_MIN_WIDTH);
		*height = max_t(u32, *height * 15 / 16,
				(u32)N32H7XX_DVP_MIN_HEIGHT);
	}
}

static const struct n32h7xx_dvp_format *
n32h7xx_dvp_clamp_format(struct n32h7xx_dvp *dvp, u32 pixelformat)
{
	const struct n32h7xx_dvp_format *fmt;

	fmt = n32h7xx_dvp_find_pixelformat(pixelformat);
	if (fmt)
		return fmt;

	/* Anything else falls back to the format already in use. */
	return dvp->fmt;
}

static void n32h7xx_dvp_fill_format(struct n32h7xx_dvp *dvp,
				    const struct n32h7xx_dvp_format *fmt,
				    struct v4l2_pix_format *pix)
{
	pix->width = clamp(pix->width, (u32)N32H7XX_DVP_MIN_WIDTH,
			   (u32)N32H7XX_DVP_MAX_WIDTH);
	pix->height = clamp(pix->height, (u32)N32H7XX_DVP_MIN_HEIGHT,
			    (u32)N32H7XX_DVP_MAX_HEIGHT);
	n32h7xx_dvp_fit_frame(fmt, n32h7xx_dvp_frame_budget(),
			      &pix->width, &pix->height);
	pix->pixelformat = fmt->pixelformat;
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = pix->width * fmt->bpp / 8;
	pix->sizeimage = n32h7xx_dvp_sizeimage(fmt, pix->width, pix->height);
	pix->colorspace = fmt->pixelformat == V4L2_PIX_FMT_JPEG ?
			  V4L2_COLORSPACE_JPEG : V4L2_COLORSPACE_SRGB;
	pix->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	pix->quantization = V4L2_QUANTIZATION_DEFAULT;
	pix->xfer_func = V4L2_XFER_FUNC_DEFAULT;
	pix->priv = 0;
}

/*
 * Ask the sensor for the format userspace wants.  The sensor answers with the
 * mode it actually selected, which can be a different resolution than the
 * request, so the reply is what gets stored.
 *
 * The port's pixel-clock ceiling is a property of the sensor link and is
 * enforced by the sensor itself (its ovti,max-dvp-pclk-hz), so the frame rate
 * is left to the sensor's own default.  Userspace can still change it through
 * VIDIOC_S_PARM on the subdevice.
 */
static int n32h7xx_dvp_program_sensor(struct n32h7xx_dvp *dvp,
				      const struct n32h7xx_dvp_format *fmt,
				      struct v4l2_pix_format *pix)
{
	struct v4l2_subdev_format sd_fmt = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
		.pad = 0,
		.format = {
			.width = pix->width,
			.height = pix->height,
			.code = fmt->mbus_code,
			.field = V4L2_FIELD_NONE,
			.colorspace = pix->colorspace,
		},
	};
	int ret;

	if (!dvp->sensor)
		return -ENODEV;

	ret = v4l2_subdev_call(dvp->sensor, pad, set_fmt, NULL, &sd_fmt);
	if (ret == -ENOIOCTLCMD)
		return 0;
	if (ret)
		return ret;

	pix->width = sd_fmt.format.width;
	pix->height = sd_fmt.format.height;

	return 0;
}

/* ------------------------------------------------------------------ DVP -- */

static int n32h7xx_dvp_stop_port(struct n32h7xx_dvp *dvp, u32 *bytes)
{
	u32 status;
	int ret;

	writel(N32H7XX_DVP_INTSTS_DVPDIS, dvp->base + N32H7XX_DVP_INTSTS);
	ret = readl_poll_timeout_atomic(dvp->base + N32H7XX_DVP_INTSTS,
				       status, status & N32H7XX_DVP_INTSTS_DVPDIS,
				       1, 2000);
	if (ret) {
		dev_err(dvp->dev, "port stop timeout, status=%#x\n", status);
		return ret;
	}
	/* The byte count must include the final AHB writes before cleanup. */
	if (bytes)
		*bytes = readl(dvp->base + N32H7XX_DVP_FPBC1);
	writel(readl(dvp->base + N32H7XX_DVP_FIFOCFG) & ~N32H7XX_DVP_M1ADDREN,
	       dvp->base + N32H7XX_DVP_FIFOCFG);
	writel(N32H7XX_DVP_ATTEMPT_FLAGS | BIT(16) | BIT(19),
	       dvp->base + N32H7XX_DVP_INTSTS);
	return 0;
}

/*
 * Board constants.
 *
 * PORTCFG: sample on the pixel clock's rising edge, HSYNC active low, VSYNC
 * active high, 8-bit data mode 1.  These are properties of the module wiring
 * and match the sensor endpoint's hsync-active/vsync-active/pclk-sample.
 *
 * The VSYNC blanking glitch filter and the FIFO burst/threshold settings are
 * the values the vendor reference code programs and that this board was
 * characterised with.
 */
#define N32H7XX_DVP_VBFLT		8
#define N32H7XX_DVP_BURST_BYTES		8
#define N32H7XX_DVP_THRESHOLD_BYTES	8

/*
 * FIFOCFG's burst and threshold fields are both encoded, not plain byte
 * counts: TXBURSZ counts in units of 8 bytes starting at 8, TXFTH in powers
 * of two starting at 8.  These are the values the vendor reference code
 * programs, and the 1 KiB boundary crossing bit is left clear as the vendor
 * driver leaves it.
 */
#define N32H7XX_DVP_FIFOCFG_BITS					\
	(FIELD_PREP(N32H7XX_DVP_TXBURSZ,				\
		    N32H7XX_DVP_BURST_BYTES / 8 - 1) |			\
	 FIELD_PREP(N32H7XX_DVP_TXFTH, 0))

/*
 * How far behind the end of a JPEG frame to look for its EOI marker.
 *
 * The sensor keeps the parallel bus running to the end of the line time
 * after the compressed frame, so the port stores a short run of zero bytes
 * behind the marker: on this board that is 0..17 bytes, never anywhere near
 * this window.  The search is bounded because a frame the port truncated has
 * no EOI at all and has to keep the length the port reported.
 */
#define N32H7XX_DVP_JPEG_EOI_WINDOW	64

/*
 * Length of the JPEG stream inside a frame the port stored in @frame, out of
 * the @bytes it counted.
 *
 * Everything behind the EOI marker is transport padding rather than picture,
 * so hand userspace the stream itself: a file written with bytesused then
 * ends exactly at EOI instead of carrying a handful of stray zeros.
 */
static u32 n32h7xx_dvp_jpeg_length(const u8 *frame, u32 bytes)
{
	u32 end = bytes;

	if (bytes < 2)
		return bytes;

	while (end > 2 && frame[end - 1] == 0x00 &&
	       bytes - end < N32H7XX_DVP_JPEG_EOI_WINDOW)
		end--;

	if (frame[end - 1] == 0xd9 && frame[end - 2] == 0xff)
		return end;

	return bytes;
}

/*
 * Bus address the port should write the frame to.  The capture path decides
 * at probe time which allocator backs the buffers, and the two report their
 * address differently; this hides that from the rest of the driver.
 */
static dma_addr_t n32h7xx_dvp_plane_dma_addr(struct vb2_buffer *vb)
{
	struct n32h7xx_dvp *dvp = vb2_get_drv_priv(vb->vb2_queue);

	if (dvp->pooled)
		return vb2_dma_contig_plane_dma_addr(vb, 0);

	return virt_to_phys(vb2_plane_vaddr(vb, 0));
}

static void n32h7xx_dvp_arm(struct n32h7xx_dvp *dvp,
			    struct n32h7xx_dvp_buffer *buf)
{
	struct vb2_buffer *vb = &buf->vb.vb2_buf;
	dma_addr_t addr = n32h7xx_dvp_plane_dma_addr(vb);
	u32 size = min_t(u32, vb2_plane_size(vb, 0),
			 N32H7XX_DVP_MAX_FRAME_SIZE);
	u32 reg;

	/*
	 * The buffer comes from the port's reserved DMA pool and is mapped
	 * uncached, so there is normally nothing to flush.  Sync anyway: this
	 * is also correct on a board that hands the driver ordinary memory.
	 */
	dma_sync_single_for_device(dvp->dev, addr, size, DMA_FROM_DEVICE);

	if (n32h7xx_dvp_stop_port(dvp, NULL)) {
		vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);
		return;
	}

	writel((u32)addr, dvp->base + N32H7XX_DVP_SMADDR1);
	writel(size, dvp->base + N32H7XX_DVP_FBS);
	writel(N32H7XX_DVP_FIFOCFG_BITS, dvp->base + N32H7XX_DVP_FIFOCFG);

	/* Snapshot mode: the port stops after one frame and says so. */
	reg = readl(dvp->base + N32H7XX_DVP_CTRL);
	reg |= N32H7XX_DVP_CTRL_CM;
	writel(reg, dvp->base + N32H7XX_DVP_CTRL);

	writel(N32H7XX_DVP_FIFOCFG_BITS | N32H7XX_DVP_M1ADDREN,
	       dvp->base + N32H7XX_DVP_FIFOCFG);
	writel(readl(dvp->base + N32H7XX_DVP_CTRL) | N32H7XX_DVP_CTRL_DVPEN,
	       dvp->base + N32H7XX_DVP_CTRL);

	dvp->armed = buf;
}

/*
 * Take the next queued buffer and start it, unless one is already in flight.
 * Callers must hold queue_lock and must have checked dvp->streaming.
 */
static void n32h7xx_dvp_arm_next_locked(struct n32h7xx_dvp *dvp)
{
	struct n32h7xx_dvp_buffer *buf;

	if (dvp->armed || list_empty(&dvp->buffers))
		return;
	buf = list_first_entry(&dvp->buffers, struct n32h7xx_dvp_buffer, entry);
	list_del(&buf->entry);
	n32h7xx_dvp_arm(dvp, buf);
}

static irqreturn_t n32h7xx_dvp_irq(int irq, void *priv)
{
	struct n32h7xx_dvp *dvp = priv;
	struct n32h7xx_dvp_buffer *buf;
	struct vb2_buffer *vb;
	dma_addr_t addr;
	u32 status, bytes;
	bool error;

	status = readl(dvp->base + N32H7XX_DVP_INTSTS);
	if (!status)
		return IRQ_NONE;

	/* Keep completion latched until the frame has been inspected. */
	spin_lock(&dvp->queue_lock);

	buf = dvp->armed;
	if (!buf) {
		writel(status & N32H7XX_DVP_ATTEMPT_FLAGS,
		       dvp->base + N32H7XX_DVP_INTSTS);
		spin_unlock(&dvp->queue_lock);
		return IRQ_HANDLED;
	}

	error = status & N32H7XX_DVP_FATAL_FLAGS;
	if (!error && !(status & N32H7XX_DVP_INTSTS_M1TCF)) {
		/* Some other flag; the frame is still in flight. */
		writel(status & N32H7XX_DVP_ATTEMPT_FLAGS,
		       dvp->base + N32H7XX_DVP_INTSTS);
		spin_unlock(&dvp->queue_lock);
		return IRQ_HANDLED;
	}

	if (status & N32H7XX_DVP_FATAL_FLAGS)
		dev_warn_ratelimited(dvp->dev,
				     "capture flagged, INTSTS=%#010x\n", status);

	/* Retain the initial count for diagnostics; stop_port() drains it. */
	vb = &buf->vb.vb2_buf;
	bytes = readl(dvp->base + N32H7XX_DVP_FPBC1);

	dvp->armed = NULL;
	if (n32h7xx_dvp_stop_port(dvp, &bytes)) {
		error = true;
		dvp->streaming = false;
		writel(0, dvp->base + N32H7XX_DVP_INTEN);
	}

	if (!error && (bytes == 0 || bytes > vb2_plane_size(vb, 0))) {
		dev_warn_ratelimited(dvp->dev,
				     "invalid frame length: %u bytes for a %lu byte buffer\n",
				     bytes, vb2_plane_size(vb, 0));
		error = true;
	}

	/*
	 * A raw buffer is exactly one frame.  Do not hand a FIFO-overflow
	 * truncation to userspace as a good frame just because its byte count
	 * is non-zero and still fits in the allocation.  A compressed buffer
	 * is only a ceiling, and filling it means the bitstream was cut short;
	 * a JPEG missing its tail is not a frame worth handing out.
	 */
	if (!error && dvp->fmt &&
	    dvp->fmt->pixelformat != V4L2_PIX_FMT_JPEG &&
	    bytes != dvp->pix.sizeimage) {
		dev_warn_ratelimited(dvp->dev,
				     "short raw frame: %u of %u bytes\n",
				     bytes, dvp->pix.sizeimage);
		error = true;
	}

	if (!error && dvp->fmt &&
	    dvp->fmt->pixelformat == V4L2_PIX_FMT_JPEG &&
	    bytes >= vb2_plane_size(vb, 0)) {
		dev_warn_ratelimited(dvp->dev,
				     "compressed frame reached the %lu byte buffer limit, raise max_frame_bytes\n",
				     vb2_plane_size(vb, 0));
		error = true;
	}

	/*
	 * Report the byte count even when the frame failed.  buf_prepare()
	 * seeds bytesused with the whole buffer, and a compressed buffer is a
	 * ceiling rather than a frame size, so leaving it alone on error makes
	 * a dropped frame look like a complete one that filled the buffer.
	 * V4L2_BUF_FLAG_ERROR is what marks it bad; bytesused should still say
	 * how much the port really wrote.
	 */
	if (bytes > vb2_plane_size(vb, 0))
		bytes = vb2_plane_size(vb, 0);

	addr = n32h7xx_dvp_plane_dma_addr(vb);
	/*
	 * Invalidate the whole plane, not just the captured part: the tail of
	 * the frame shares a cache line with bytes past it.
	 */
	dma_sync_single_for_cpu(dvp->dev, addr, vb2_plane_size(vb, 0),
				DMA_FROM_DEVICE);

	/*
	 * A compressed frame is the only one whose stored length is not the
	 * picture length: strip the transport padding behind the EOI marker
	 * so bytesused is the JPEG stream and nothing else.
	 */
	if (dvp->fmt && dvp->fmt->pixelformat == V4L2_PIX_FMT_JPEG)
		bytes = n32h7xx_dvp_jpeg_length(vb2_plane_vaddr(vb, 0), bytes);

	vb2_set_plane_payload(vb, 0, bytes);

	vb2_buffer_done(vb, error ? VB2_BUF_STATE_ERROR : VB2_BUF_STATE_DONE);
	n32h7xx_dvp_arm_next_locked(dvp);

	spin_unlock(&dvp->queue_lock);

	return IRQ_HANDLED;
}

/* --------------------------------------------------------------- vb2 ops -- */

static int n32h7xx_dvp_queue_setup(struct vb2_queue *q,
				   unsigned int *nbuffers, unsigned int *nplanes,
				   unsigned int *sizes,
				   struct device *alloc_devs[])
{
	struct n32h7xx_dvp *dvp = vb2_get_drv_priv(q);

	if (*nplanes) {
		if (sizes[0] < dvp->pix.sizeimage)
			return -EINVAL;
		return 0;
	}

	*nplanes = 1;
	sizes[0] = dvp->pix.sizeimage;
	return 0;
}

static int n32h7xx_dvp_buf_prepare(struct vb2_buffer *vb)
{
	struct n32h7xx_dvp *dvp = vb2_get_drv_priv(vb->vb2_queue);

	if (vb2_plane_size(vb, 0) < dvp->pix.sizeimage)
		return -EINVAL;

	vb2_set_plane_payload(vb, 0, dvp->pix.sizeimage);

	return 0;
}

static int n32h7xx_dvp_start_streaming(struct vb2_queue *q, unsigned int count)
{
	unsigned long flags;
	struct n32h7xx_dvp *dvp = vb2_get_drv_priv(q);
	int ret;

	if (!dvp->sensor) {
		dev_err(dvp->dev, "no sensor bound to the DVP port\n");
		ret = -ENODEV;
		goto err_buffers;
	}

	/* Port configuration: polarity and bus width are fixed by the board. */
	writel(N32H7XX_DVP_PORTCFG_DEFAULT & N32H7XX_DVP_PORTCFG_RW_MASK,
	       dvp->base + N32H7XX_DVP_PORTCFG);
	writel(FIELD_PREP(N32H7XX_DVP_CTRL_VBFLT, N32H7XX_DVP_VBFLT & 0xf),
	       dvp->base + N32H7XX_DVP_CTRL);
	writel(N32H7XX_DVP_INT_M1TCIE | N32H7XX_DVP_INT_AHBERRIE |
	       N32H7XX_DVP_INT_FOIE, dvp->base + N32H7XX_DVP_INTEN);
	n32h7xx_dvp_stop_port(dvp, NULL);

	/*
	 * Arm the port before the sensor starts sending.  The reference manual
	 * requires DVPEN to be set at least four pixel clocks before VSYNC goes
	 * active, and its operation flow puts "enable the DVP port" before
	 * "start the sensor"; a frame the port is not ready for is dropped.
	 * Arming after s_stream() therefore starts the port in the middle of a
	 * frame, which is what made the longer 16-bit modes lose data.
	 */
	spin_lock_irqsave(&dvp->queue_lock, flags);
	dvp->streaming = true;
	n32h7xx_dvp_arm_next_locked(dvp);
	spin_unlock_irqrestore(&dvp->queue_lock, flags);

	ret = v4l2_subdev_call(dvp->sensor, video, s_stream, 1);
	if (ret && ret != -ENOIOCTLCMD) {
		dev_err(dvp->dev, "failed to start the sensor: %d\n", ret);
		goto err_buffers;
	}

	return 0;

err_buffers:
	spin_lock_irqsave(&dvp->queue_lock, flags);
	dvp->streaming = false;
	if (dvp->armed) {
		struct n32h7xx_dvp_buffer *armed = dvp->armed;

		dvp->armed = NULL;
		list_add(&armed->entry, &dvp->buffers);
	}
	n32h7xx_dvp_stop_port(dvp, NULL);
	while (!list_empty(&dvp->buffers)) {
		struct n32h7xx_dvp_buffer *buf;

		buf = list_first_entry(&dvp->buffers,
				       struct n32h7xx_dvp_buffer, entry);
		list_del(&buf->entry);
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_QUEUED);
	}
	spin_unlock_irqrestore(&dvp->queue_lock, flags);

	return ret;
}

static void n32h7xx_dvp_stop_streaming(struct vb2_queue *q)
{
	unsigned long flags;
	struct n32h7xx_dvp *dvp = vb2_get_drv_priv(q);
	struct n32h7xx_dvp_buffer *buf;

	spin_lock_irqsave(&dvp->queue_lock, flags);
	dvp->streaming = false;

	buf = dvp->armed;
	dvp->armed = NULL;
	n32h7xx_dvp_stop_port(dvp, NULL);

	if (buf)
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
	while (!list_empty(&dvp->buffers)) {
		buf = list_first_entry(&dvp->buffers,
				       struct n32h7xx_dvp_buffer, entry);
		list_del(&buf->entry);
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
	}
	spin_unlock_irqrestore(&dvp->queue_lock, flags);

	if (dvp->sensor)
		v4l2_subdev_call(dvp->sensor, video, s_stream, 0);
}

static void n32h7xx_dvp_buf_queue(struct vb2_buffer *vb)
{
	unsigned long flags;
	struct n32h7xx_dvp *dvp = vb2_get_drv_priv(vb->vb2_queue);
	struct n32h7xx_dvp_buffer *buf = container_of(vb,
						      struct n32h7xx_dvp_buffer,
						      vb.vb2_buf);

	spin_lock_irqsave(&dvp->queue_lock, flags);
	list_add_tail(&buf->entry, &dvp->buffers);
	if (dvp->streaming)
		n32h7xx_dvp_arm_next_locked(dvp);
	spin_unlock_irqrestore(&dvp->queue_lock, flags);
}

static const struct vb2_ops n32h7xx_dvp_vb2_ops = {
	.queue_setup		= n32h7xx_dvp_queue_setup,
	.buf_prepare		= n32h7xx_dvp_buf_prepare,
	.start_streaming	= n32h7xx_dvp_start_streaming,
	.stop_streaming		= n32h7xx_dvp_stop_streaming,
	.buf_queue		= n32h7xx_dvp_buf_queue,
};

/* -------------------------------------------------------------- ioctls ---- */

static int n32h7xx_dvp_querycap(struct file *file, void *fh,
				struct v4l2_capability *cap)
{
	strscpy(cap->driver, N32H7XX_DVP_DRV_NAME, sizeof(cap->driver));
	strscpy(cap->card, "N32H7xx DVP", sizeof(cap->card));

	return 0;
}

static int n32h7xx_dvp_enum_fmt(struct file *file, void *fh,
				struct v4l2_fmtdesc *f)
{
	if (f->index >= ARRAY_SIZE(n32h7xx_dvp_formats))
		return -EINVAL;

	f->pixelformat = n32h7xx_dvp_formats[f->index].pixelformat;

	return 0;
}

static int n32h7xx_dvp_g_fmt(struct file *file, void *fh,
			     struct v4l2_format *f)
{
	struct n32h7xx_dvp *dvp = video_drvdata(file);

	f->fmt.pix = dvp->pix;

	return 0;
}

static int n32h7xx_dvp_try_fmt(struct file *file, void *fh,
			       struct v4l2_format *f)
{
	struct n32h7xx_dvp *dvp = video_drvdata(file);
	const struct n32h7xx_dvp_format *fmt;

	/*
	 * The sensor is only asked to commit a format on S_FMT, so TRY_FMT
	 * clamps against the driver's own limits.  A sensor that cannot do the
	 * requested resolution replies with the mode it picked on S_FMT.
	 */
	fmt = n32h7xx_dvp_clamp_format(dvp, f->fmt.pix.pixelformat);
	n32h7xx_dvp_fill_format(dvp, fmt, &f->fmt.pix);

	return 0;
}

/*
 * Settle on a resolution that both the sensor and the frame buffer budget
 * can live with.
 *
 * The sensor answers a request with the closest mode it has, and that mode
 * is routinely larger than what was asked for, so a single round trip is not
 * enough.  A 2592x1944 request for an uncompressed format is first scaled to
 * fit the budget; the sensor then rounds that up to a real mode, which may
 * not fit after all.  Ask again one step below whatever came back until the
 * answer fits the budget, and fail if no sensor mode does.
 *
 * Compressed frames keep their resolution: their buffer is a ceiling rather
 * than the frame itself, so any mode the port can address is acceptable.
 */
static int n32h7xx_dvp_negotiate(struct n32h7xx_dvp *dvp,
				 const struct n32h7xx_dvp_format *fmt,
				 struct v4l2_pix_format *pix)
{
	u32 budget = n32h7xx_dvp_frame_budget();
	unsigned int tries;

	for (tries = 0; tries < 8; tries++) {
		struct v4l2_pix_format want = *pix;
		u32 width, height;
		int ret;

		n32h7xx_dvp_fill_format(dvp, fmt, &want);

		ret = n32h7xx_dvp_program_sensor(dvp, fmt, &want);
		if (ret)
			return ret;

		want.bytesperline = want.width * fmt->bpp / 8;
		want.sizeimage = n32h7xx_dvp_sizeimage(fmt, want.width,
						       want.height);
		if (want.sizeimage <= budget) {
			*pix = want;
			return 0;
		}

		/* The mode the sensor picked is too big; ask below it. */
		*pix = want;
		width = want.width;
		height = want.height;
		n32h7xx_dvp_fit_frame(fmt, budget, &width, &height);
		if (width == want.width && height == want.height)
			break;

		pix->width = width;
		pix->height = height;
	}

	dev_err(dvp->dev,
		"sensor mode %ux%u needs a %u byte buffer, over the %u byte limit\n",
		pix->width, pix->height, pix->sizeimage, budget);

	return -EINVAL;
}

static int n32h7xx_dvp_s_fmt(struct file *file, void *fh,
			     struct v4l2_format *f)
{
	struct n32h7xx_dvp *dvp = video_drvdata(file);
	const struct n32h7xx_dvp_format *fmt;
	struct v4l2_pix_format pix = f->fmt.pix;
	int ret;

	if (vb2_is_busy(&dvp->queue))
		return -EBUSY;

	fmt = n32h7xx_dvp_clamp_format(dvp, pix.pixelformat);
	ret = n32h7xx_dvp_negotiate(dvp, fmt, &pix);
	if (ret)
		return ret;

	dvp->fmt = fmt;
	dvp->pix = pix;
	f->fmt.pix = pix;

	dev_dbg(dvp->dev, "capture format %ux%u (0x%08x), %u bytes/frame\n",
		pix.width, pix.height, pix.pixelformat, pix.sizeimage);

	return 0;
}

static const struct v4l2_ioctl_ops n32h7xx_dvp_ioctl_ops = {
	.vidioc_querycap	= n32h7xx_dvp_querycap,
	.vidioc_enum_fmt_vid_cap = n32h7xx_dvp_enum_fmt,
	.vidioc_g_fmt_vid_cap	= n32h7xx_dvp_g_fmt,
	.vidioc_try_fmt_vid_cap	= n32h7xx_dvp_try_fmt,
	.vidioc_s_fmt_vid_cap	= n32h7xx_dvp_s_fmt,
	.vidioc_reqbufs		= vb2_ioctl_reqbufs,
	.vidioc_querybuf	= vb2_ioctl_querybuf,
	.vidioc_qbuf		= vb2_ioctl_qbuf,
	.vidioc_expbuf		= vb2_ioctl_expbuf,
	.vidioc_dqbuf		= vb2_ioctl_dqbuf,
	.vidioc_create_bufs	= vb2_ioctl_create_bufs,
	.vidioc_prepare_buf	= vb2_ioctl_prepare_buf,
	.vidioc_streamon	= vb2_ioctl_streamon,
	.vidioc_streamoff	= vb2_ioctl_streamoff,
};

static const struct v4l2_file_operations n32h7xx_dvp_fops = {
	.owner		= THIS_MODULE,
	.open		= v4l2_fh_open,
	.release	= vb2_fop_release,
	.read		= vb2_fop_read,
	.poll		= vb2_fop_poll,
	.mmap		= vb2_fop_mmap,
	/*
	 * No-MMU only.  A MAP_SHARED mapping of a character device has to be
	 * told where to live, and there is no MMU to place it: the mapping is
	 * sited at the buffer's own kernel address, which is what
	 * vb2_get_unmapped_area() returns.  Without this hook do_mmap() in
	 * mm/nommu.c gets -ENOSYS back, has no MAP_COPY fallback for a shared
	 * mapping, and fails the whole mmap() with -ENODEV.  This is the same
	 * pairing stm32-dcmi and uvc use.
	 */
#ifndef CONFIG_MMU
	.get_unmapped_area = vb2_fop_get_unmapped_area,
#endif
	.unlocked_ioctl	= video_ioctl2,
};

/* -------------------------------------------------------------- notifier -- */

/*
 * The async core hands over the sensor subdevice as soon as its driver has
 * registered, which can be before or after this driver's probe.  Keep the
 * pointer so S_FMT and streaming can talk to it, and re-apply the format the
 * capture device is currently set to in case it was the sensor that arrived
 * last (init_format() could not program anything at probe time then).
 */
static int n32h7xx_dvp_notifier_bound(struct v4l2_async_notifier *notifier,
				      struct v4l2_subdev *sd,
				      struct v4l2_async_connection *asc)
{
	struct n32h7xx_dvp *dvp = container_of(notifier, struct n32h7xx_dvp,
					       notifier);

	dvp->sensor = sd;
	dev_info(dvp->dev, "sensor %s bound to the DVP port\n", sd->name);

	/*
	 * The sensor owns the controls that matter to a camera user: exposure,
	 * gain, white balance, the flips and the ISP test pattern.  Lend its
	 * handler to the video node so they are reachable with the ordinary
	 * control ioctls on /dev/videoN, without asking userspace to find a
	 * second device node for the same sensor.
	 *
	 * The pointer is picked up per file handle by v4l2_fh_open(), and that
	 * is also what makes the control ioctls pass the v4l2 core's
	 * valid_ioctls check: those bits are computed at
	 * video_register_device() time, when no sensor has bound yet and this
	 * field is still NULL.  atmel-isi and pxa_camera wire a subdev up the
	 * same way.
	 */
	if (sd->ctrl_handler)
		dvp->vdev.ctrl_handler = sd->ctrl_handler;

	n32h7xx_dvp_program_sensor(dvp, dvp->fmt, &dvp->pix);

	return 0;
}

/*
 * A sensor can only go away while its module is being unloaded.  Drop the
 * pointers so a still-open /dev/videoN cannot reach a freed subdevice; the
 * next S_FMT then reports -ENODEV instead of dereferencing stale memory.
 */
static void n32h7xx_dvp_notifier_unbind(struct v4l2_async_notifier *notifier,
					struct v4l2_subdev *sd,
					struct v4l2_async_connection *asc)
{
	struct n32h7xx_dvp *dvp = container_of(notifier, struct n32h7xx_dvp,
					       notifier);

	if (dvp->sensor != sd)
		return;

	dvp->vdev.ctrl_handler = NULL;
	dvp->sensor = NULL;
}

static int n32h7xx_dvp_notifier_complete(struct v4l2_async_notifier *notifier)
{
	struct n32h7xx_dvp *dvp = container_of(notifier, struct n32h7xx_dvp,
					       notifier);

	dev_info(dvp->dev, "capture device /dev/video%d ready (sensor %s)\n",
		 dvp->vdev.num, dvp->sensor ? dvp->sensor->name : "?");

	return 0;
}

static const struct v4l2_async_notifier_operations n32h7xx_dvp_notifier_ops = {
	.bound		= n32h7xx_dvp_notifier_bound,
	.unbind		= n32h7xx_dvp_notifier_unbind,
	.complete	= n32h7xx_dvp_notifier_complete,
};

/* --------------------------------------------------------------- probe ---- */

static void n32h7xx_dvp_release_pool(void *data)
{
	of_reserved_mem_device_release(data);
}

/*
 * Work out how big a frame buffer can be before anything is allocated: the
 * answer limits the resolution the uncompressed formats can offer and is the
 * ceiling a compressed frame may reach.
 *
 * The reserved pool from the device tree is the intended source, because it
 * is one contiguous run of SDRAM the allocator cannot fragment away.  A board
 * that reserves nothing still gets a working port, limited to whatever
 * contiguous block the allocator can produce at the time.
 */
static int n32h7xx_dvp_init_budget(struct n32h7xx_dvp *dvp)
{
	struct device *dev = dvp->dev;
	int ret;

	/*
	 * Claim the reserved pool before anything is allocated from it.  The
	 * device tree is the contract here: the pool is what makes a frame
	 * bigger than the page allocator's largest block possible.
	 */
	if (of_find_property(dev->of_node, "memory-region", NULL)) {
		ret = of_reserved_mem_device_init(dev);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to claim the frame buffer pool\n");

		ret = devm_add_action_or_reset(dev,
					       n32h7xx_dvp_release_pool, dev);
		if (ret)
			return ret;

		dvp->pooled = true;
	}

	n32h7xx_dvp_probed_bytes = n32h7xx_dvp_allocatable_bytes(dev,
								 dvp->pooled);
	dev_info(dev, "frame buffer limit %u KiB (%s)\n",
		 n32h7xx_dvp_frame_budget() / 1024,
		 dvp->pooled ? "reserved pool" : "largest contiguous block");

	return 0;
}

static int n32h7xx_dvp_init_queue(struct n32h7xx_dvp *dvp)
{
	struct vb2_queue *q = &dvp->queue;

	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_DMABUF;
	q->drv_priv = dvp;
	q->buf_struct_size = sizeof(struct n32h7xx_dvp_buffer);
	q->ops = &n32h7xx_dvp_vb2_ops;
	/*
	 * A reserved pool has to be claimed through dma_alloc_coherent(), the
	 * page allocator path is the one the other backends use.  Both hand
	 * back one physically contiguous block per buffer on this no-MMU
	 * target, which is all the port needs.
	 */
	q->mem_ops = dvp->pooled ? &vb2_dma_contig_memops
				 : &vb2_vmalloc_memops;
	q->dev = dvp->dev;
	q->lock = &dvp->lock;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	/*
	 * Leave min_queued_buffers at its default of 0.  The port is an AHB
	 * master that is armed one frame at a time from .buf_queue, so a single
	 * buffer is a complete queue.  Raising it would also raise vb2's
	 * minimum reqbufs allocation (min_queued_buffers + 1), and on this
	 * no-MMU target every buffer is physically contiguous: a 640x480
	 * 16-bit frame is 600 KiB, and the board has one 1 MiB block, so a
	 * three-buffer minimum can never be satisfied.  Letting userspace pick
	 * the count keeps the pipeline deep where RAM allows and still accepts
	 * the single buffer where it does not.
	 */

	return vb2_queue_init(q);
}

static int n32h7xx_dvp_register_sensor(struct n32h7xx_dvp *dvp)
{
	struct fwnode_handle *ep;
	struct v4l2_async_connection *asc;
	int ret;

	ep = fwnode_graph_get_next_endpoint(dev_fwnode(dvp->dev), NULL);
	if (!ep)
		return dev_err_probe(dvp->dev, -ENODEV,
				     "DVP node has no port/endpoint\n");

	v4l2_async_nf_init(&dvp->notifier, &dvp->v4l2_dev);

	asc = v4l2_async_nf_add_fwnode_remote(&dvp->notifier, ep,
					     struct v4l2_async_connection);
	fwnode_handle_put(ep);
	if (IS_ERR(asc))
		return dev_err_probe(dvp->dev, PTR_ERR(asc),
				     "failed to add the sensor endpoint\n");

	/*
	 * The sensor subdevice is registered with the V4L2 core by its own
	 * driver; the async core hands it to us through the notifier once it
	 * shows up, so the video node can be registered first.
	 */
	dvp->notifier.ops = &n32h7xx_dvp_notifier_ops;

	ret = v4l2_async_nf_register(&dvp->notifier);
	if (ret)
		return dev_err_probe(dvp->dev, ret,
				     "failed to register the async notifier\n");

	return 0;
}

/*
 * Frames are captured into the sensor's chosen resolution, so the sensor is
 * asked for a sane default before userspace says anything.
 */
static int n32h7xx_dvp_init_format(struct n32h7xx_dvp *dvp)
{
	const struct n32h7xx_dvp_format *fmt = &n32h7xx_dvp_formats[0];
	struct v4l2_pix_format pix = {
		.width = N32H7XX_DVP_DEFAULT_WIDTH,
		.height = N32H7XX_DVP_DEFAULT_HEIGHT,
		.pixelformat = fmt->pixelformat,
	};

	n32h7xx_dvp_fill_format(dvp, fmt, &pix);

	/* Best effort: a sensor that is not bound yet answers -ENODEV. */
	n32h7xx_dvp_program_sensor(dvp, fmt, &pix);

	dvp->fmt = fmt;
	dvp->pix = pix;

	return 0;
}

static int n32h7xx_dvp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct n32h7xx_dvp *dvp;
	int ret;

	dvp = devm_kzalloc(dev, sizeof(*dvp), GFP_KERNEL);
	if (!dvp)
		return -ENOMEM;

	dvp->dev = dev;
	spin_lock_init(&dvp->queue_lock);
	INIT_LIST_HEAD(&dvp->buffers);
	mutex_init(&dvp->lock);

	dvp->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(dvp->base))
		return PTR_ERR(dvp->base);

	dvp->irq = platform_get_irq(pdev, 0);
	if (dvp->irq < 0)
		return dvp->irq;

	ret = n32h7xx_dvp_soc_init(dev);
	if (ret)
		return ret;

	ret = v4l2_device_register(dev, &dvp->v4l2_dev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register v4l2_device\n");

	ret = n32h7xx_dvp_init_budget(dvp);
	if (ret)
		goto err_v4l2;

	ret = n32h7xx_dvp_init_format(dvp);
	if (ret)
		goto err_v4l2;

	ret = n32h7xx_dvp_init_queue(dvp);
	if (ret) {
		dev_err_probe(dev, ret, "failed to initialise the vb2 queue\n");
		goto err_v4l2;
	}

	ret = devm_request_irq(dev, dvp->irq, n32h7xx_dvp_irq, 0,
			       dev_name(dev), dvp);
	if (ret) {
		dev_err_probe(dev, ret, "failed to request IRQ %d\n", dvp->irq);
		goto err_queue;
	}

	dvp->vdev.queue = &dvp->queue;
	dvp->vdev.v4l2_dev = &dvp->v4l2_dev;
	dvp->vdev.fops = &n32h7xx_dvp_fops;
	dvp->vdev.ioctl_ops = &n32h7xx_dvp_ioctl_ops;
	dvp->vdev.release = video_device_release_empty;
	dvp->vdev.lock = &dvp->lock;
	dvp->vdev.device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	dvp->vdev.vfl_dir = VFL_DIR_RX;
	strscpy(dvp->vdev.name, N32H7XX_DVP_DRV_NAME, sizeof(dvp->vdev.name));
	video_set_drvdata(&dvp->vdev, dvp);

	ret = video_register_device(&dvp->vdev, VFL_TYPE_VIDEO, -1);
	if (ret) {
		dev_err_probe(dev, ret, "failed to register the video device\n");
		goto err_queue;
	}

	ret = n32h7xx_dvp_register_sensor(dvp);
	if (ret)
		goto err_video;

	platform_set_drvdata(pdev, dvp);

	dev_info(dev, "N32H7xx DVP capture at %pR, irq %d, /dev/video%d\n",
		 &pdev->resource[0], dvp->irq, dvp->vdev.num);

	return 0;

err_video:
	video_unregister_device(&dvp->vdev);
err_queue:
	vb2_queue_release(&dvp->queue);
err_v4l2:
	v4l2_device_unregister(&dvp->v4l2_dev);

	return ret;
}

static void n32h7xx_dvp_remove(struct platform_device *pdev)
{
	struct n32h7xx_dvp *dvp = platform_get_drvdata(pdev);

	v4l2_async_nf_unregister(&dvp->notifier);
	v4l2_async_nf_cleanup(&dvp->notifier);
	video_unregister_device(&dvp->vdev);
	vb2_queue_release(&dvp->queue);
	v4l2_device_unregister(&dvp->v4l2_dev);
}

static const struct of_device_id n32h7xx_dvp_of_match[] = {
	{ .compatible = "nsing,n32h7xx-dvp" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, n32h7xx_dvp_of_match);

static struct platform_driver n32h7xx_dvp_driver = {
	.probe	= n32h7xx_dvp_probe,
	.remove	= n32h7xx_dvp_remove,
	.driver	= {
		.name		= N32H7XX_DVP_DRV_NAME,
		.of_match_table	= n32h7xx_dvp_of_match,
	},
};
module_platform_driver(n32h7xx_dvp_driver);

MODULE_AUTHOR("NSING Technologies Inc.");
MODULE_DESCRIPTION("NSING N32H7xx DVP parallel camera capture driver");
MODULE_LICENSE("GPL");
