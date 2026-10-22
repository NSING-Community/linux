// SPDX-License-Identifier: GPL-2.0-only
/* NSING N32H7xx D/AVE HD: bounded, synchronous 2D jobs. */

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/dma-fence.h>
#include <linux/dma-resv.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/ktime.h>
#include <linux/math64.h>
#include <linux/math.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/processor.h>
#include <linux/reset.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/soc/nsing/n32h7xx-graphics.h>
#include <linux/string.h>
#include <linux/uaccess.h>

#include <drm/drm_drv.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_ioctl.h>
#include <drm/drm_managed.h>
#include <uapi/drm/n32h7xx_drm.h>

#include "n32h7xx-drm.h"

#define GPU_VERSION		0x00
#define GPU_CONFIG3		0x0c
#define GPU_BUSY			0x10
#define GPU_CONTROL		0x14
#define GPU_IRQ_STATUS		0x18
#define GPU_STREAM_ADDR		0x1c
#define GPU_RING_PAUSE		0x20
#define GPU_IRQ_STOP		BIT(0)
#define GPU_IRQ_BUS_ERROR	BIT(3)
#define GPU_IRQ_MASK		GENMASK(4, 0)
#define GPU_STOP_CODE		GENMASK(10, 8)
#define GPU_CLOCK_ALWAYS_ON	BIT(8)
#define GPU_LIMIT_PREFETCH	BIT(14)
/* CONFIG_3: bit 16 is CLR_AVAILABLE, bit 17 is ZSA_AVAILABLE. */
#define GPU_CLEAR_PRESENT	BIT(16)
#define GPU_ZSA_PRESENT		BIT(17)
#define GPU_CONFIG3_EXTENDED	BIT(31)

/* Stream opcodes contain a word-based register index in bits 23:8. */
#define GPU_OP_WAIT(mask)	((1U << 24) | ((mask) << 8))
#define GPU_OP_WRITE(reg, count)	(BIT(31) | (4U << 24) | ((reg) << 8) | ((count) - 1))
/* DUMP: copy one STC register to a GPU address; two parameter words follow. */
#define GPU_OP_DUMP(reg)	(BIT(31) | (0x05U << 24) | ((reg) << 8) | 1)
#define GPU_OP_END		((0x7fU << 24) | (0x3ff << 8))
#define GPU_WAIT_WRITEBACK	GENMASK(8, 3)
/* Stream writer finished and the register write landed: a DUMP is in memory. */
#define GPU_WAIT_STW_REG	(BIT(2) | BIT(3))
#define GPU_WAIT_CLEAR		(BIT(9) | BIT(3))
/* Wait for every pipeline stage between the register file and the blender. */
#define GPU_WAIT_PIPE		GENMASK(9, 3)
#define GPU_CLR_VALUE_INDEX	0x3c0
#define GPU_CLR_BPP		GENMASK(21, 20)
#define GPU_CLR_BYTE_MASK	GENMASK(19, 16)
#define GPU_MAX_BUFFER		SZ_4M
#define GPU_TIMEOUT		msecs_to_jiffies(1000)

/*
 * Register blocks addressed by the stream are indexed in words (byte offset
 * divided by four) from the GPU base address.
 */
#define GPU_WORD(off)		((off) / 4)
#define GPU_PSU_BBOX_MIN	GPU_WORD(0x0200)
#define GPU_PSU_BBOX_MAX	GPU_WORD(0x0204)
#define GPU_PSU_START		GPU_WORD(0x0208)
#define GPU_PSU_CTRL		GPU_WORD(0x020c)
#define GPU_PSU_STRIPE		GPU_WORD(0x0210)
#define GPU_PSU_BEZ_CTRL	GPU_WORD(0x0214)
#define GPU_PSU_BEZ_VOFF0	GPU_WORD(0x0218)
#define GPU_PSU_BEZ_VOFF1	GPU_WORD(0x021c)
#define GPU_PSU_BEZ_AA		GPU_WORD(0x0220)
#define GPU_PSU_VSTART0		GPU_WORD(0x0224)
#define GPU_PSU_CLIP		GPU_WORD(0x0270)
#define GPU_TXA_U0		GPU_WORD(0x0300)
#define GPU_TXA_V0		GPU_WORD(0x0304)
#define GPU_TXA_DUX0		GPU_WORD(0x0308)
#define GPU_TXA_DUY0		GPU_WORD(0x030c)
#define GPU_TXA_DVX0		GPU_WORD(0x0310)
#define GPU_TXA_DVY0		GPU_WORD(0x0314)
#define GPU_TXA_SIZE0		GPU_WORD(0x0610)
#define GPU_TXA_ACCESS0		GPU_WORD(0x0614)
#define GPU_TEX_GLOBAL		GPU_WORD(0x0800)
#define GPU_FBS_CONFIG		GPU_WORD(0x0808)
#define GPU_FBS_PITCH		GPU_WORD(0x080c)
#define GPU_FBS_SPAN		GPU_WORD(0x0810)
#define GPU_COL_GLOBAL		GPU_WORD(0x0804)
#define GPU_COL_CONST0		GPU_WORD(0x0818)
#define GPU_TEX_MODE0		GPU_WORD(0x0a00)
#define GPU_TXC_START0		GPU_WORD(0x0a04)
#define GPU_TXP_CTRL0		GPU_WORD(0x0a08)
#define GPU_COL_OP1A0		GPU_WORD(0x0b00)
#define GPU_COL_OP1B0		GPU_WORD(0x0b04)
#define GPU_COL_OP2A0		GPU_WORD(0x0b08)
#define GPU_COL_OP2B0		GPU_WORD(0x0b0c)
#define GPU_COL_OP3_0		GPU_WORD(0x0b10)
#define GPU_COL_OUT0		GPU_WORD(0x0b14)
#define GPU_BLU_PIXEL_ORG	GPU_WORD(0x0c00)
#define GPU_BLU_FB_ADDRESS	GPU_WORD(0x0c04)
#define GPU_BLU_BLEND		GPU_WORD(0x0c08)
#define GPU_BLU_DITHER		GPU_WORD(0x0c0c)
#define GPU_BLU_WRITE		GPU_WORD(0x0c10)
#define GPU_PFC_ENABLE		GPU_WORD(0x0e00)
#define GPU_PFC_CLEAR		GPU_WORD(0x0e04)
#define GPU_PFC_EVENT_SELECT	GPU_WORD(0x0e08)
/* STC register file entry 0x20 + n holds performance counter n. */
#define GPU_STC_PERF_COUNTER(n)	(0x20 + (n))
/* Highest event id the four counters accept. */
#define GPU_PFC_EVENT_MAX	69

/*
 * The command buffer is one page.  A profiling job is a fixed head that
 * clears and programs the counters, one rectangle body per iteration, and a
 * fixed tail that waits for the pipeline, dumps the four counter registers and
 * switches the counters off again.  The per-iteration estimate is rounded up
 * from the 28 words the rectangle body really writes so that the job can
 * never run past the end of the buffer.
 */
/* A batch of small rectangles is one job; the stream holds up to 128 of them. */
#define GPU_STREAM_SIZE			SZ_16K
#define GPU_STREAM_WORDS		(GPU_STREAM_SIZE / 4)
#define GPU_PROFILE_FIXED_WORDS	30
#define GPU_PROFILE_ITER_WORDS	32
#define GPU_PROFILE_TAIL_WORDS	17
#define GPU_PROFILE_MAX_ITER 	((GPU_STREAM_WORDS - GPU_PROFILE_FIXED_WORDS - GPU_PROFILE_TAIL_WORDS) / 	 GPU_PROFILE_ITER_WORDS)

/*
 * Colour unit operand: bits 11:10, 9:8 and 7:6 select which channel of the
 * selected operand feeds R, G and B; 3 means "the same channel", which is the
 * identity mapping and the reset value.  Bits 4:0 and bits 20:16 select the
 * operand itself for RGB and for alpha; 0 is the constant 0 and 19 is texture
 * unit 0.
 */
#define GPU_COL_OP_IDENTITY	0x06c0
#define GPU_COL_OP3_TEX0	(GPU_COL_OP_IDENTITY | (19 << 16) | 19)
/* op3 selects constant colour 0, which is what the rectangle paths draw. */
#define GPU_COL_OP3_CONST0	(GPU_COL_OP_IDENTITY | (3 << 16) | 3)
/* COL_ARGB_OUT reset value: unity gain and no channel swapping. */
#define GPU_COL_OUT_UNITY	0x10011001

/* TEX_MODE: discard the texture alpha (alpha becomes 1.0), clip U/V. */
#define GPU_TEX_DISCARD_ALPHA	BIT(28)

/* FBD_CONFIG: every source pixel passes the alpha test. */
#define GPU_FBS_ALPHA_PASS	0x7
/* FBS_SPAN_CONFIG reset value; the cache limits are not used by this driver. */
#define GPU_FBS_SPAN_DEFAULT	0x001000ff
/* BLU_WRITE: enable all colour channels. */
#define GPU_BLU_WRITE_ALL	0xf
/* BLU_WRITE bit 16: blend the result with the destination by the coverage. */
#define GPU_BLU_WRITE_COVERAGE	BIT(16)

/*
 * BLU_DITHER: enable the 4x4 ordered dither after blending.  Bits 18:16,
 * 22:20 and 26:24 hold the destination width of each channel; the hardware
 * adds the matrix value to the fractional part of the converted colour.
 * The pattern follows the absolute buffer coordinates.
 */
#define GPU_BLU_DITHER_ENABLE	BIT(0)
#define GPU_BLU_DITHER_RED(v)	((u32)(v) << 16)
#define GPU_BLU_DITHER_GREEN(v)	((u32)(v) << 20)
#define GPU_BLU_DITHER_BLUE(v)	((u32)(v) << 24)
/* BASE is two pairs of two bits: x in 5:4 and y in 7:6. */
#define GPU_BLU_DITHER_BASE(x, y) ((((x) & 3) | (((y) & 3) << 2)) << 4)
/* LIM_BBOX_MAX clip register reset value: no additional clipping. */
#define GPU_CLIP_DEFAULT	0x0fff0fff
/* Limiter control: enable, contribute coverage, enumerate; or no ramp. */
#define GPU_LIM_CTRL_AA		0x5
#define GPU_LIM_CTRL_NO_AA	0x7
/* LIM_CTRL bit 28 enables the Bezier post processor.  Bit 29 selects the
 * concave side, which is not exposed: the hardware then emits coverage only
 * in unstable 1 px strips next to the chord instead of a filled region.
 */
#define GPU_LIM_CTRL_BEZIER	BIT(28)
/* BEZ_AA_CTRL: 4.6 reciprocal of the ramp width, one pixel is 0x40. */
#define GPU_BEZ_AA_ONE_PX	(0x40 << 16)

/*
 * Alpha blending, measured on hardware.  BLU_BLEND has two independent
 * factor pairs: the colour channels (bits 11:8) and the alpha channel
 * (bits 25:16).  The colour pair seeds the source factor from the source
 * alpha (bit 10), the destination factor from the source alpha too (bit 11)
 * and inverts it (bit 9): out = src * a + dst * (1 - a).  Writing only bit 10
 * instead multiplies the destination by the source RGB and gives a saturating
 * add.
 *
 * The alpha pair must be programmed separately.  The alpha source factor is
 * ONE, built from "constant 0, inverted" (select 2 in bits 17:16 plus the
 * invert bit 24); the destination factor is the source alpha inverted
 * (select 0 in bits 21:20 plus the invert bit 25).  That gives
 * out_a = a + dst_a * (1 - a), the straight-alpha source-over result.
 * Without bit 17 the alpha output collapses to a * dst_a * (1 - a); the
 * colour channels are unaffected, so a colour-only test still passes and the
 * mistake only shows up on a surface that stores alpha.  Verified on hardware
 * over six (src, dst) alpha pairs with a maximum error of one 8-bit step.
 */
#define GPU_BLU_BLEND_SRC_OVER	(BIT(25) | BIT(24) | (2 << 16) | \
				 BIT(11) | BIT(10) | BIT(9))
/* Opaque write: source factor is the (forced 1.0) alpha, destination unused. */
#define GPU_BLU_BLEND_OPAQUE	BIT(10)

/*
 * FBD_CONFIG read decision flags: read the destination when the source alpha
 * is not 1.0 (bit 4) or not 0.0 (bit 5).  Together they cover every alpha, so
 * a fully transparent source still reads back and rewrites the destination
 * instead of zeroing it.  Without any flag the destination is not read at all
 * and the blend operates on black.
 */
#define GPU_FBS_READ_FOR_BLEND	(BIT(5) | BIT(4))

/* FBC_START_ADDRESS ignores its low three bits. */
#define GPU_FB_ADDRESS_ALIGN	8

/*
 * LIM_STRIPE width for a rotated blit, in destination pixels.  A 90/270 blit
 * walks the source column by column, so a full-width destination row touches
 * one texture cache line per pixel and the cache never helps.  Enumerating in
 * narrow vertical stripes keeps only that many source lines live at once; the
 * L1 texture cache has four lines per set, and two leaves room for the
 * framebuffer traffic and for source rows that map to the same set.
 * Measured on hardware: 400x240 rot 90 goes from 172 ms to 8.9-14.7 ms
 * depending on pitch, and every stripe width stays pixel exact.
 */
#define GPU_BLIT_STRIPE		2

/*
 * Pixel formats we can write.  org is the FB_PIXEL_ORG encoding the blend
 * unit writes, tex_org the M_TEX_ORG encoding the texture unit reads; both
 * were measured on this part, and hold the same value for every format we
 * expose.  alpha marks formats whose alpha channel carries data.
 */
struct n32h7xx_format {
	u32 fourcc;
	u8 cpp;
	u8 org;
	u8 tex_org;
	bool alpha;
	u8 r_bits;
	u8 g_bits;
	u8 b_bits;
};

static const struct n32h7xx_format n32h7xx_formats[] = {
	{ DRM_FORMAT_RGB565,	2, 6, 6, false, 5, 6, 5 },
	{ DRM_FORMAT_XRGB8888,	4, 0, 0, false, 8, 8, 8 },
	{ DRM_FORMAT_ARGB8888,	4, 0, 0, true,  8, 8, 8 },
	{ DRM_FORMAT_RGBA8888,	4, 1, 1, true,  8, 8, 8 },
	{ DRM_FORMAT_ARGB4444,	2, 2, 2, true,  4, 4, 4 },
	{ DRM_FORMAT_RGBA4444,	2, 3, 3, true,  4, 4, 4 },
	{ DRM_FORMAT_ARGB1555,	2, 4, 4, true,  5, 5, 5 },
	{ DRM_FORMAT_RGBA5551,	2, 5, 5, true,  5, 5, 5 },
};

static const struct n32h7xx_format *n32h7xx_format_of(u32 fourcc)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(n32h7xx_formats); i++)
		if (n32h7xx_formats[i].fourcc == fourcc)
			return &n32h7xx_formats[i];
	return NULL;
}

struct n32h7xx_poly_batch_item;

struct n32h7xx_gpu {
	struct drm_device drm;
	void __iomem *regs;
	struct reset_control *reset;
	struct mutex lock; /* Serializes the command buffer, jobs and recovery. */
	struct completion done;
	int irq;
	u32 irq_status;
	u32 *stream;
	dma_addr_t stream_dma;
	/* Destination of the counter DUMP commands; the STC cannot be read. */
	u32 *dump;
	dma_addr_t dump_dma;
	u64 fence_context;
	u64 seqno;
	bool wedged;
	struct drm_gem_object *stuck;
	/* Reused so a batch ioctl does not allocate on every call. */
	struct mutex batch_lock;
	struct drm_n32h7xx_rect *batch_rects;
	struct n32h7xx_batch_item *batch_items;
	struct drm_n32h7xx_poly *batch_polys;
	struct n32h7xx_poly_batch_item *batch_poly_items;
};

struct n32h7xx_fence {
	struct dma_fence base;
	spinlock_t lock; /* Protects dma_fence state. */
};

/* Pixel pipeline arguments, shared by the rectangle and blend ioctls. */
struct n32h7xx_draw {
	u32 handle;
	u32 format;
	u32 offset;
	u32 pitch;
	u32 width;
	u32 height;
	u32 color;
	u32 flags;
};

/*
 * A blit job needs both surfaces, the rotated/reflected geometry and the six
 * texture coordinates the hardware interpolates over the destination block.
 */
struct n32h7xx_blit {
	u32 src_offset;
	u32 src_pitch_px;
	u32 src_tex_org;
	bool blend;
	u32 src_width;
	u32 src_height;
	u32 dst_offset;
	u32 dst_pitch_px;
	u32 dst_org;
	u32 width;
	u32 height;
	u32 stripe;
	u32 xmin;
	u32 u0;
	u32 v0;
	u32 dux;
	u32 duy;
	u32 dvx;
	u32 dvy;
};

/* One limiter edge: v = vstart + (x - xmin) * dx + (y - ymin) * dy. */
struct n32h7xx_edge {
	s32 vstart;
	s32 dx;
	s32 dy;
};

/* Limiter setup and enumeration box derived from a convex polygon. */
struct n32h7xx_poly_geom {
	unsigned int count;
	u32 xmin;
	u32 ymin;
	u32 xmax;
	u32 ymax;
	struct n32h7xx_edge edge[6];
};

/* Per-rectangle state a batch needs after validation. */
struct n32h7xx_batch_item {
	u32 offset;
	u32 width;
	u32 height;
	u32 color;
	u32 org;
	u32 pitch_px;
	u32 xmin;
	u32 dither;
};

/* A batch entry is validated through the single-rectangle path. */
static_assert(sizeof(struct drm_n32h7xx_rect) == sizeof(struct n32h7xx_draw));

/* The caller's rectangle array and the validated form of it. */
struct n32h7xx_batch {
	const struct drm_n32h7xx_rect *rects;
	struct n32h7xx_batch_item *item;
	unsigned int count;
};

/* Validated polygon and the state its emitter needs outside the geometry. */
struct n32h7xx_poly_batch_item {
	struct n32h7xx_poly_geom geom;
	u32 org;
	u32 pitch_px;
};

struct n32h7xx_poly_batch {
	const struct drm_n32h7xx_poly *polys;
	struct n32h7xx_poly_batch_item *item;
	unsigned int count;
};

/* One submitted 2D operation, independent of which engine executes it. */
struct n32h7xx_job {
	int (*validate)(struct drm_gem_object *obj, struct n32h7xx_job *job);
	void (*emit)(struct n32h7xx_gpu *gpu, const struct n32h7xx_job *job,
		     dma_addr_t addr);
	const struct n32h7xx_draw *draw;
	const struct drm_n32h7xx_poly *poly;
	const struct drm_n32h7xx_curve *curve;
	struct n32h7xx_batch *batch;
	struct n32h7xx_poly_batch *poly_batch;
	struct n32h7xx_poly_geom geom;
	struct drm_n32h7xx_profile *profile;
	u32 iterations;
	bool blend;
	u32 cpp;
	u32 org;
	u32 dither;
	u32 pitch_px;
	u32 xmin;
};

static const char *n32h7xx_fence_driver_name(struct dma_fence *fence)
{
	return "n32h7xx-gpu";
}

static const char *n32h7xx_fence_timeline_name(struct dma_fence *fence)
{
	return "clear";
}

static const struct dma_fence_ops n32h7xx_fence_ops = {
	.get_driver_name = n32h7xx_fence_driver_name,
	.get_timeline_name = n32h7xx_fence_timeline_name,
};

static irqreturn_t n32h7xx_gpu_irq(int irq, void *data)
{
	struct n32h7xx_gpu *gpu = data;
	u32 status = readl(gpu->regs + GPU_IRQ_STATUS);

	if (!(status & GPU_IRQ_MASK))
		return IRQ_NONE;
	writel(status & GPU_IRQ_MASK, gpu->regs + GPU_IRQ_STATUS);
	if (status & (GPU_IRQ_STOP | GPU_IRQ_BUS_ERROR)) {
		WRITE_ONCE(gpu->irq_status, status);
		complete(&gpu->done);
	}
	return IRQ_HANDLED;
}

static int n32h7xx_gpu_create(struct drm_device *drm, void *data,
			      struct drm_file *file)
{
	struct drm_n32h7xx_create *args = data;
	struct drm_gem_dma_object *obj;
	int ret, idx;

	if (args->flags || !args->size || args->size > GPU_MAX_BUFFER)
		return -EINVAL;
	if (!drm_dev_enter(drm, &idx))
		return -ENODEV;
	obj = drm_gem_dma_create(drm, args->size);
	if (IS_ERR(obj)) {
		ret = PTR_ERR(obj);
		goto exit;
	}
	ret = drm_gem_handle_create(file, &obj->base, &args->handle);
	if (!ret) {
		args->size = obj->base.size;
		args->mmap_offset = drm_vma_node_offset_addr(&obj->base.vma_node);
	}
	drm_gem_object_put(&obj->base);
exit:
	drm_dev_exit(idx);
	return ret;
}

static int n32h7xx_gpu_validate_fill(struct drm_gem_object *obj,
				     struct n32h7xx_job *job)
{
	const struct n32h7xx_draw *args = job->draw;
	const struct n32h7xx_format *fmt = n32h7xx_format_of(args->format);
	u32 *cpp = &job->cpp;
	u64 end;
	u32 bytes;

	if (!fmt)
		return -EINVAL;
	*cpp = fmt->cpp;
	if (args->flags || !args->width || args->width > 4096 ||
	    !args->height || args->height > 4096)
		return -EINVAL;
	bytes = args->width * *cpp;
	if (bytes > 16384 || args->pitch < bytes || args->pitch > 32767 ||
	    !IS_ALIGNED(args->pitch, *cpp) ||
	    !IS_ALIGNED(args->offset, *cpp))
		return -EINVAL;
	end = (u64)args->offset + (u64)(args->height - 1) * args->pitch + bytes;
	if (end > obj->size)
		return -EINVAL;
	return 0;
}

static int n32h7xx_gpu_validate_rect(struct drm_gem_object *obj,
				     struct n32h7xx_job *job)
{
	const struct n32h7xx_draw *args = job->draw;
	const struct n32h7xx_format *fmt = n32h7xx_format_of(args->format);
	u64 end;
	u32 cpp, bytes;

	if (!fmt)
		return -EINVAL;
	cpp = fmt->cpp;
	job->org = fmt->org;
	job->cpp = cpp;
	if (args->flags & ~DRM_N32H7XX_DITHER)
		return -EINVAL;
	if (!args->width || args->width > 4096 ||
	    !args->height || args->height > 4096)
		return -EINVAL;
	bytes = args->width * cpp;
	if (bytes > 16384 || args->pitch < bytes || args->pitch > 32767 ||
	    !IS_ALIGNED(args->pitch, cpp) || !IS_ALIGNED(args->offset, cpp))
		return -EINVAL;
	end = (u64)args->offset + (u64)(args->height - 1) * args->pitch + bytes;
	if (end > obj->size)
		return -EINVAL;
	job->pitch_px = args->pitch / cpp;
	if (job->pitch_px > 8191)
		return -EINVAL;
	if (args->offset % GPU_FB_ADDRESS_ALIGN % cpp)
		return -EINVAL;
	job->xmin = (args->offset % GPU_FB_ADDRESS_ALIGN) / cpp;
	if (job->xmin + args->width > 8192)
		return -EINVAL;
	/*
	 * The dither scale is the number of remaining framebuffer bits per
	 * channel, held in three bits.  Only the 16-bit formats fit.  The
	 * matrix restarts at the first pixel of every job, so shift it back
	 * by the rectangle's position in the surface: drawing an area in
	 * pieces must not move the pattern at the tile boundaries.
	 */
	if (args->flags & DRM_N32H7XX_DITHER) {
		u32 col, row;

		if (fmt->r_bits > 7)
			return -EINVAL;
		col = (args->offset % args->pitch) / cpp;
		row = args->offset / args->pitch;
		job->dither = GPU_BLU_DITHER_ENABLE |
			      GPU_BLU_DITHER_RED(fmt->r_bits) |
			      GPU_BLU_DITHER_GREEN(fmt->g_bits) |
			      GPU_BLU_DITHER_BLUE(fmt->b_bits) |
			      GPU_BLU_DITHER_BASE(job->xmin - col, 0 - row);
	}
	return 0;
}

static void n32h7xx_gpu_emit_fill(struct n32h7xx_gpu *gpu,
				  const struct n32h7xx_job *job, dma_addr_t addr)
{
	const struct n32h7xx_draw *args = job->draw;
	u32 cpp = job->cpp;
	u32 color = args->color;

	if (cpp == 2)
		color = (color & 0xffff) * 0x10001;
	gpu->stream[0] = GPU_OP_WAIT(GPU_WAIT_WRITEBACK);
	gpu->stream[1] = GPU_OP_WRITE(GPU_CLR_VALUE_INDEX, 4);
	gpu->stream[2] = color;
	gpu->stream[3] = (args->pitch << 16) | (args->width * cpp - 1);
	gpu->stream[4] = FIELD_PREP(GPU_CLR_BPP, cpp - 1) |
			 FIELD_PREP(GPU_CLR_BYTE_MASK, 15) | (args->height - 1);
	gpu->stream[5] = addr + args->offset;
	gpu->stream[6] = GPU_OP_WAIT(GPU_WAIT_CLEAR);
	gpu->stream[7] = GPU_OP_END;
}

/*
 * Enumeration starts when the limiter control register is written, so that
 * write must come last.  With every limiter disabled the bounding box is
 * filled completely, which is what a solid rectangle needs.
 */
static unsigned int n32h7xx_gpu_emit_rect_body(const struct n32h7xx_job *job,
					      dma_addr_t addr, u32 *s)
{
	const struct n32h7xx_draw *args = job->draw;
	u32 fbd_config = GPU_FBS_ALPHA_PASS;
	u32 blend_mode = GPU_BLU_BLEND_SRC_OVER;
	u32 org = job->org;
	u32 pitch_px = job->pitch_px;
	u32 xmin = job->xmin;
	unsigned int n = 0;
	u32 x1 = xmin + args->width - 1;
	u32 y1 = args->height - 1;
	u32 color = args->color;

	if (job->blend) {
		/* Read the destination so the blend unit sees real pixels. */
		fbd_config |= GPU_FBS_READ_FOR_BLEND;
	} else {
		/* A plain rectangle writes every pixel, so it ignores alpha. */
		blend_mode = GPU_BLU_BLEND_OPAQUE;
		color |= 0xff000000;
	}

	s[n++] = GPU_OP_WRITE(GPU_FBS_CONFIG, 3);
	s[n++] = fbd_config;
	s[n++] = pitch_px;
	s[n++] = GPU_FBS_SPAN_DEFAULT;
	/* Disable the texture units: with one left enabled by an earlier blit
	 * the pixel pipeline takes about three times as long per rectangle,
	 * even though nothing samples a texture.
	 */
	s[n++] = GPU_OP_WRITE(GPU_TEX_GLOBAL, 1);
	s[n++] = 0;
	s[n++] = GPU_OP_WRITE(GPU_COL_GLOBAL, 1);
	s[n++] = 1;
	s[n++] = GPU_OP_WRITE(GPU_COL_CONST0, 1);
	s[n++] = color;
	/* Program the whole colour chain: the previous job may have left the
	 * texture unit selected here, and a rectangle must not sample it.
	 */
	s[n++] = GPU_OP_WRITE(GPU_COL_OP1A0, 6);
	s[n++] = GPU_COL_OP_IDENTITY;
	s[n++] = GPU_COL_OP_IDENTITY;
	s[n++] = GPU_COL_OP_IDENTITY;
	s[n++] = GPU_COL_OP_IDENTITY;
	s[n++] = GPU_COL_OP3_CONST0;
	s[n++] = GPU_COL_OUT_UNITY;
	s[n++] = GPU_OP_WRITE(GPU_BLU_PIXEL_ORG, 5);
	s[n++] = org;
	s[n++] = ALIGN_DOWN(addr + args->offset, GPU_FB_ADDRESS_ALIGN);
	s[n++] = blend_mode;
	s[n++] = job->dither;
	s[n++] = GPU_BLU_WRITE_ALL;
	/* A rotated blit leaves stripe enumeration enabled; rectangles walk the
	 * destination row by row and must not inherit it.
	 */
	s[n++] = GPU_OP_WRITE(GPU_PSU_STRIPE, 2);
	s[n++] = 0;
	s[n++] = 0;
	s[n++] = GPU_OP_WRITE(GPU_PSU_BBOX_MIN, 4);
	s[n++] = xmin;
	s[n++] = (y1 << 16) | x1;
	s[n++] = 0;
	s[n++] = 0;
	return n;
}

static void n32h7xx_gpu_emit_rect(struct n32h7xx_gpu *gpu,
				  const struct n32h7xx_job *job, dma_addr_t addr)
{
	u32 *s = gpu->stream;
	unsigned int n;

	n = n32h7xx_gpu_emit_rect_body(job, addr, s);
	s[n++] = GPU_OP_WAIT(GPU_WAIT_PIPE);
	s[n++] = GPU_OP_END;
}

static int n32h7xx_gpu_validate_rect_batch(struct drm_gem_object *obj,
					   struct n32h7xx_job *job)
{
	struct n32h7xx_batch *batch = job->batch;
	unsigned int i;

	for (i = 0; i < batch->count; i++) {
		const struct drm_n32h7xx_rect *rect = &batch->rects[i];
		struct n32h7xx_batch_item *item = &batch->item[i];
		struct n32h7xx_job one = {
			.draw = (const struct n32h7xx_draw *)rect,
		};
		int ret;

		if (rect->handle != job->draw->handle)
			return -EINVAL;
		ret = n32h7xx_gpu_validate_rect(obj, &one);
		if (ret)
			return ret;
		item->offset = rect->offset;
		item->width = rect->width;
		item->height = rect->height;
		item->color = rect->color;
		item->org = one.org;
		item->pitch_px = one.pitch_px;
		item->xmin = one.xmin;
		item->dither = one.dither;
	}
	return 0;
}

/*
 * One job for the whole array.  The pixel pipeline keeps its state between
 * rectangles, so only what differs from the previous rectangle is written
 * again: colour, framebuffer base, dither setup, and finally the bounding box
 * whose control word starts enumeration.  The first rectangle emits the full
 * setup because an earlier job may have left the texture unit selected.
 *
 * Enumeration starts when the last limiter-control word is written.  The
 * stream does not wait between rectangles: register writes queue behind the
 * running walk, and the hardware latches a new bounding box only after the
 * previous one is finished.  A 128-rectangle job with a different colour,
 * offset and size in every entry was checked pixel by pixel to confirm that
 * no state leaks between consecutive rectangles.  One drain at the end of the
 * job is enough.
 *
 * Worst case is 18 words of common setup plus 17 words per rectangle (new
 * pitch, colour, origin/dither and bounding box), so 128 rectangles need
 * about 8.6 KiB of the 16 KiB stream buffer.
 */
static void n32h7xx_gpu_emit_rect_batch(struct n32h7xx_gpu *gpu,
					const struct n32h7xx_job *job,
					dma_addr_t addr)
{
	const struct n32h7xx_batch *batch = job->batch;
	u32 *s = gpu->stream;
	u32 last_fbd = 0, last_pitch = 0, last_base = 0;
	u32 last_dither = 0, last_color = 0, last_org = 0;
	bool seen = false;
	unsigned int n = 0, i;

	for (i = 0; i < batch->count; i++) {
		const struct n32h7xx_batch_item *item = &batch->item[i];
		u32 fbd = GPU_FBS_ALPHA_PASS;
		u32 base = ALIGN_DOWN(addr + item->offset, GPU_FB_ADDRESS_ALIGN);
		u32 color = item->color | 0xff000000;
		u32 x1 = item->xmin + item->width - 1;

		if (!seen) {
			s[n++] = GPU_OP_WRITE(GPU_TEX_GLOBAL, 1);
			s[n++] = 0;
			s[n++] = GPU_OP_WRITE(GPU_COL_GLOBAL, 1);
			s[n++] = 1;
			s[n++] = GPU_OP_WRITE(GPU_COL_OP1A0, 6);
			s[n++] = GPU_COL_OP_IDENTITY;
			s[n++] = GPU_COL_OP_IDENTITY;
			s[n++] = GPU_COL_OP_IDENTITY;
			s[n++] = GPU_COL_OP_IDENTITY;
			s[n++] = GPU_COL_OP3_CONST0;
			s[n++] = GPU_COL_OUT_UNITY;
			s[n++] = GPU_OP_WRITE(GPU_PSU_STRIPE, 2);
			s[n++] = 0;
			s[n++] = 0;
		}
		if (!seen || fbd != last_fbd || item->pitch_px != last_pitch) {
			s[n++] = GPU_OP_WRITE(GPU_FBS_CONFIG, 3);
			s[n++] = fbd;
			s[n++] = item->pitch_px;
			s[n++] = GPU_FBS_SPAN_DEFAULT;
			last_fbd = fbd;
			last_pitch = item->pitch_px;
		}
		if (!seen || color != last_color) {
			s[n++] = GPU_OP_WRITE(GPU_COL_CONST0, 1);
			s[n++] = color;
			last_color = color;
		}
		if (!seen || base != last_base || item->dither != last_dither ||
		    item->org != last_org) {
			s[n++] = GPU_OP_WRITE(GPU_BLU_PIXEL_ORG, 5);
			s[n++] = item->org;
			s[n++] = base;
			s[n++] = GPU_BLU_BLEND_OPAQUE;
			s[n++] = item->dither;
			s[n++] = GPU_BLU_WRITE_ALL;
			last_base = base;
			last_dither = item->dither;
			last_org = item->org;
		}
		s[n++] = GPU_OP_WRITE(GPU_PSU_BBOX_MIN, 4);
		s[n++] = item->xmin;
		s[n++] = ((item->height - 1) << 16) | x1;
		s[n++] = 0;
		s[n++] = 0;
		seen = true;
	}
	s[n++] = GPU_OP_WAIT(GPU_WAIT_PIPE);
	s[n++] = GPU_OP_END;
}

/*
 * Profiling: draw the same rectangle several times with the performance
 * counters running.  The counters are cleared inside the job, so the reported
 * values describe this call alone, and they are switched off again before the
 * job ends.  A DUMP command copies each counter into the driver's own page
 * because the counter registers live in the STC register file and have no
 * memory mapping.
 */
static int n32h7xx_gpu_validate_profile(struct drm_gem_object *obj,
					struct n32h7xx_job *job)
{
	const struct drm_n32h7xx_profile *args = job->profile;
	unsigned int i;
	int ret;

	if (!args->iterations || args->iterations > GPU_PROFILE_MAX_ITER)
		return -EINVAL;
	for (i = 0; i < ARRAY_SIZE(args->event); i++)
		if (args->event[i] != DRM_N32H7XX_EVENT_OFF &&
		    args->event[i] > GPU_PFC_EVENT_MAX)
			return -EINVAL;
	ret = n32h7xx_gpu_validate_rect(obj, job);
	if (ret)
		return ret;
	job->iterations = args->iterations;
	return 0;
}

static void n32h7xx_gpu_emit_profile(struct n32h7xx_gpu *gpu,
				     const struct n32h7xx_job *job,
				     dma_addr_t addr)
{
	const struct drm_n32h7xx_profile *args = job->profile;
	u32 *s = gpu->stream;
	unsigned int n = 0, i;
	u32 enable = 0;

	for (i = 0; i < ARRAY_SIZE(args->event); i++)
		if (args->event[i] != DRM_N32H7XX_EVENT_OFF)
			enable |= BIT(i);

	/*
	 * The counters free run: drain the pipeline before clearing them, or
	 * pixels of the previous job still in flight end up in this sample.
	 */
	s[n++] = GPU_OP_WAIT(GPU_WAIT_PIPE | BIT(0));
	s[n++] = GPU_OP_WRITE(GPU_PFC_CLEAR, 1);
	s[n++] = 0xf;
	s[n++] = GPU_OP_WRITE(GPU_PFC_EVENT_SELECT, 4);
	for (i = 0; i < ARRAY_SIZE(args->event); i++)
		s[n++] = args->event[i] == DRM_N32H7XX_EVENT_OFF ? 0 :
			 args->event[i];
	s[n++] = GPU_OP_WRITE(GPU_PFC_ENABLE, 1);
	s[n++] = enable;

	for (i = 0; i < job->iterations; i++) {
		/* Validation already bounds iterations; keep the write safe
		 * anyway in case the rectangle body grows.
		 */
		if (n + GPU_PROFILE_ITER_WORDS + GPU_PROFILE_TAIL_WORDS >
		    GPU_STREAM_WORDS)
			break;
		n += n32h7xx_gpu_emit_rect_body(job, addr, s + n);
	}

	s[n++] = GPU_OP_WAIT(GPU_WAIT_PIPE);
	for (i = 0; i < ARRAY_SIZE(args->event); i++) {
		s[n++] = GPU_OP_DUMP(GPU_STC_PERF_COUNTER(i));
		s[n++] = gpu->dump_dma + i * sizeof(u32);
		s[n++] = 0;
	}
	s[n++] = GPU_OP_WAIT(GPU_WAIT_STW_REG);
	s[n++] = GPU_OP_WRITE(GPU_PFC_ENABLE, 1);
	s[n++] = 0;
	s[n++] = GPU_OP_END;
}

/* Vertices are 16.16; keep the products below well inside 64 bits. */
#define N32H7XX_POLY_COORD_MAX	(4096 << 16)
#define N32H7XX_POLY_NO_AA	BIT(0)
/* Limiter bounding box and clip fields are 12 bits wide. */
#define N32H7XX_POLY_LIMIT	4095
#define N32H7XX_CURVE_NO_AA	BIT(0)

static s64 n32h7xx_poly_floor(s64 v)
{
	return v >= 0 ? v >> 16 : -(((-v) + 0xffff) >> 16);
}

static s64 n32h7xx_poly_ceil(s64 v)
{
	return -n32h7xx_poly_floor(-v);
}

/*
 * One polygon side becomes one limiter, which evaluates the signed distance
 * to the side in pixels (positive inside) at every pixel's top-left corner.
 * The hardware turns that into coverage clamp(v, 0, 1), so the ramp spans one
 * pixel and ends on the side.  Sampling the corner alone would put the ramp
 * half a pixel off for a side whose interior is to the right or below, so the
 * offset of the pixel's far corner along the inward normal, max(nx, 0) plus
 * max(ny, 0), is added: that makes an axis aligned side cover exactly the
 * fraction of the pixel it overlaps.
 *
 * The value is anchored at the enumeration start (xmin, ymin), which is what
 * the hardware adds dx/dy to per pixel; all three coordinates are 16.16.
 * vstart comes out in 14.18 fixed point because dot is in 2^32 units, len in
 * 2^16 and the 4 = 2^18 / 2^16 factor below is what turns pixels into 14.18.
 */
static int n32h7xx_poly_edge(struct n32h7xx_edge *edge,
			     const struct drm_n32h7xx_poly *args, unsigned int i,
			     s64 cx, s64 cy, s64 xmin, s64 ymin, bool aa)
{
	unsigned int n = args->vertex_count;
	s64 x0 = args->x[i], y0 = args->y[i];
	s64 x1 = args->x[(i + 1) % n], y1 = args->y[(i + 1) % n];
	s64 ex = x1 - x0, ey = y1 - y0;
	s64 nx = -ey, ny = ex;
	s64 dot, vstart, far;
	u32 len;

	if (nx * (cx - x0) + ny * (cy - y0) < 0) {
		nx = -nx;
		ny = -ny;
	}
	len = int_sqrt64((u64)(ex * ex) + (u64)(ey * ey));
	if (!len)
		return -EINVAL;
	dot = nx * (xmin - x0) + ny * (ymin - y0);
	/*
	 * Without the ramp the hardware keeps a pixel whose limiter value is
	 * zero, which is exactly the pixel whose top-left corner is on the
	 * side, so the far corner offset must not be added there: it would
	 * widen the polygon by a pixel.
	 */
	far = aa ? (nx > 0 ? nx : 0) + (ny > 0 ? ny : 0) : 0;
	vstart = div64_s64(4 * dot + (far << 18), (s64)len);
	if (vstart > S32_MAX || vstart < S32_MIN)
		return -EINVAL;
	edge->vstart = vstart;
	edge->dx = div64_s64(nx << 18, len);
	edge->dy = div64_s64(ny << 18, len);
	if (edge->dx > 0x7ffff || edge->dx < -0x80000 ||
	    edge->dy > 0x7ffff || edge->dy < -0x80000)
		return -EINVAL;
	return 0;
}

static int n32h7xx_gpu_validate_poly(struct drm_gem_object *obj,
				     struct n32h7xx_job *job)
{
	const struct drm_n32h7xx_poly *args = job->poly;
	const struct n32h7xx_format *fmt = n32h7xx_format_of(args->format);
	struct n32h7xx_poly_geom *geom = &job->geom;
	s64 cx = 0, cy = 0, minx, maxx, miny, maxy;
	s64 gx0, gx1, gy0, gy1, last_row;
	unsigned int i, cpp;
	u32 org, pitch_px;
	u64 end;
	int ret;

	/* Coverage blending into a surface that stores alpha is not
	 * characterised yet; only the plain blend path writes alpha.
	 */
	if (!fmt || fmt->alpha)
		return -EINVAL;
	cpp = fmt->cpp;
	org = fmt->org;
	if ((args->flags & ~N32H7XX_POLY_NO_AA) || args->reserved ||
	    args->vertex_count < 3 || args->vertex_count > 6)
		return -EINVAL;
	if (!IS_ALIGNED(args->offset, GPU_FB_ADDRESS_ALIGN) ||
	    !IS_ALIGNED(args->pitch, cpp) || !args->pitch || args->pitch > 32767)
		return -EINVAL;
	pitch_px = args->pitch / cpp;
	minx = miny = S64_MAX;
	maxx = maxy = S64_MIN;
	for (i = 0; i < args->vertex_count; i++) {
		s64 x = args->x[i], y = args->y[i];

		if (x > N32H7XX_POLY_COORD_MAX || x < -N32H7XX_POLY_COORD_MAX ||
		    y > N32H7XX_POLY_COORD_MAX || y < -N32H7XX_POLY_COORD_MAX)
			return -EINVAL;
		cx += x;
		cy += y;
		minx = min(minx, x);
		maxx = max(maxx, x);
		miny = min(miny, y);
		maxy = max(maxy, y);
	}
	cx = div_s64(cx, args->vertex_count);
	cy = div_s64(cy, args->vertex_count);
	gx0 = n32h7xx_poly_floor(minx);
	gx1 = n32h7xx_poly_ceil(maxx) - 1;
	gy0 = n32h7xx_poly_floor(miny);
	gy1 = n32h7xx_poly_ceil(maxy) - 1;
	if (gx0 < 0 || gy0 < 0 || gx1 < gx0 || gy1 < gy0 ||
	    gx1 >= pitch_px || gx1 > N32H7XX_POLY_LIMIT ||
	    gy1 > N32H7XX_POLY_LIMIT)
		return -EINVAL;
	end = (u64)args->offset + (u64)gy1 * args->pitch + (gx1 + 1) * cpp;
	if (end > obj->size)
		return -EINVAL;

	/* The ramp can reach one pixel past the outline; clip that to the
	 * surface rather than writing outside it.
	 */
	geom->count = args->vertex_count;
	geom->xmin = max_t(s64, gx0 - 1, 0);
	geom->ymin = max_t(s64, gy0 - 1, 0);
	geom->xmax = min_t(s64, gx1 + 1, (s64)pitch_px - 1);
	last_row = (obj->size - args->offset) / args->pitch - 1;
	end = (u64)args->offset + (u64)(gy1 + 1) * args->pitch +
	      (gx1 + 1) * cpp;
	if (gy1 + 1 <= last_row && end <= obj->size)
		geom->ymax = min_t(s64, gy1 + 1, N32H7XX_POLY_LIMIT);
	else
		geom->ymax = gy1;

	for (i = 0; i < args->vertex_count; i++) {
		ret = n32h7xx_poly_edge(&geom->edge[i], args, i, cx, cy,
					geom->xmin << 16, geom->ymin << 16,
					!(args->flags & N32H7XX_POLY_NO_AA));
		if (ret)
			return ret;
	}
	job->cpp = cpp;
	job->org = org;
	job->pitch_px = pitch_px;
	return 0;
}

static int n32h7xx_gpu_validate_poly_batch(struct drm_gem_object *obj,
					   struct n32h7xx_job *job)
{
	struct n32h7xx_poly_batch *batch = job->poly_batch;
	unsigned int i;

	for (i = 0; i < batch->count; i++) {
		struct n32h7xx_poly_batch_item *item = &batch->item[i];
		struct n32h7xx_job one = {
			.poly = &batch->polys[i],
		};
		int ret;

		if (one.poly->handle != job->draw->handle)
			return -EINVAL;
		ret = n32h7xx_gpu_validate_poly(obj, &one);
		if (ret)
			return ret;
		item->geom = one.geom;
		item->org = one.org;
		item->pitch_px = one.pitch_px;
	}
	return 0;
}

static s64 n32h7xx_round_shift16(s64 v)
{
	return v >= 0 ? (v + 0x8000) >> 16 : -((-v + 0x8000) >> 16);
}

/*
 * A curve leg is the side of the control triangle from the tangent
 * intersection to a tangency point.  The Bezier unit compares
 * sqrt(u) + sqrt(v) with 1, so both legs are normalised to 1 at the opposite
 * tangency point: value = dot(n, p - p0) / dot(n, opposite - p0).  The raw
 * limiter would be a distance in pixels; here the coefficients are divided by
 * that distance, which is scale invariant and needs no square root.
 */
static int n32h7xx_curve_leg(struct n32h7xx_edge *edge, s64 x0, s64 y0,
			     s64 x1, s64 y1, s64 ix, s64 iy,
			     s64 xmin, s64 ymin)
{
	s64 nx = -(y1 - y0), ny = x1 - x0;
	s64 dot_i, cx, cy, vstart, den;

	if (nx * (ix - x0) + ny * (iy - y0) < 0) {
		nx = -nx;
		ny = -ny;
	}
	dot_i = nx * (ix - x0) + ny * (iy - y0);
	den = dot_i >> 16;
	if (den <= 0)
		return -EINVAL;
	/*
	 * div_s64() takes a 32-bit divisor on this architecture and would
	 * silently truncate den (a leg of a few hundred pixels is already
	 * above 2^31 once it is in 2^16 units), so use the 64-bit divide.
	 */
	cx = div64_s64(nx << 18, den);
	cy = div64_s64(ny << 18, den);
	if (cx > 0x7ffff || cx < -0x80000 ||
	    cy > 0x7ffff || cy < -0x80000)
		return -EINVAL;
	vstart = n32h7xx_round_shift16(cx * (xmin - x0) + cy * (ymin - y0));
	if (vstart > S32_MAX || vstart < S32_MIN)
		return -EINVAL;
	edge->vstart = vstart;
	edge->dx = cx;
	edge->dy = cy;
	return 0;
}

/*
 * The chord is a hard edge: with the curve unit enabled every limiter that has
 * its antialias bit set becomes another curve leg, and enabling AA here blanks
 * the whole primitive.  A hard edge can only round, so it rounds at the pixel
 * centre, half the pixel's extent along the normal; rounding at the top-left
 * corner instead drops every pixel the chord only just enters and eroded a
 * 4:1 chord by 65 px^2 on hardware.
 */
static int n32h7xx_curve_chord(struct n32h7xx_edge *edge, s64 x0, s64 y0,
			       s64 x1, s64 y1, s64 ix, s64 iy,
			       s64 xmin, s64 ymin)
{
	s64 ex = x1 - x0, ey = y1 - y0;
	s64 nx = -ey, ny = ex;
	s64 dot, vstart, far;
	u32 len;

	if (nx * (ix - x0) + ny * (iy - y0) < 0) {
		nx = -nx;
		ny = -ny;
	}
	len = int_sqrt64((u64)(ex * ex) + (u64)(ey * ey));
	if (!len)
		return -EINVAL;
	dot = nx * (xmin - x0) + ny * (ymin - y0);
	far = (abs(nx) + abs(ny)) >> 1;
	vstart = div64_s64(4 * dot + (far << 18), (s64)len);
	if (vstart > S32_MAX || vstart < S32_MIN)
		return -EINVAL;
	edge->vstart = vstart;
	edge->dx = div64_s64(nx << 18, len);
	edge->dy = div64_s64(ny << 18, len);
	return 0;
}

static int n32h7xx_gpu_validate_curve(struct drm_gem_object *obj,
				      struct n32h7xx_job *job)
{
	const struct drm_n32h7xx_curve *args = job->curve;
	const struct n32h7xx_format *fmt = n32h7xx_format_of(args->format);
	struct n32h7xx_poly_geom *geom = &job->geom;
	const s32 *vx[3] = { &args->x0, &args->x1, &args->x2 };
	const s32 *vy[3] = { &args->y0, &args->y1, &args->y2 };
	s64 x[3], y[3], minx, maxx, miny, maxy, gx0, gx1, gy0, gy1, last_row;
	unsigned int i, cpp;
	u32 org, pitch_px;
	u64 end;
	int ret;

	/* Coverage blending into a surface that stores alpha is not
	 * characterised yet; only the plain blend path writes alpha.
	 */
	if (!fmt || fmt->alpha)
		return -EINVAL;
	cpp = fmt->cpp;
	org = fmt->org;
	if ((args->flags & ~N32H7XX_CURVE_NO_AA) || args->reserved)
		return -EINVAL;
	if (!IS_ALIGNED(args->offset, GPU_FB_ADDRESS_ALIGN) ||
	    !IS_ALIGNED(args->pitch, cpp) || !args->pitch || args->pitch > 32767)
		return -EINVAL;
	pitch_px = args->pitch / cpp;
	minx = miny = S64_MAX;
	maxx = maxy = S64_MIN;
	for (i = 0; i < 3; i++) {
		x[i] = *vx[i];
		y[i] = *vy[i];
		if (x[i] > N32H7XX_POLY_COORD_MAX || x[i] < -N32H7XX_POLY_COORD_MAX ||
		    y[i] > N32H7XX_POLY_COORD_MAX || y[i] < -N32H7XX_POLY_COORD_MAX)
			return -EINVAL;
		minx = min(minx, x[i]);
		maxx = max(maxx, x[i]);
		miny = min(miny, y[i]);
		maxy = max(maxy, y[i]);
	}
	if ((x[1] - x[0]) * (y[2] - y[0]) ==
	    (y[1] - y[0]) * (x[2] - x[0]))
		return -EINVAL;
	gx0 = n32h7xx_poly_floor(minx);
	gx1 = n32h7xx_poly_ceil(maxx) - 1;
	gy0 = n32h7xx_poly_floor(miny);
	gy1 = n32h7xx_poly_ceil(maxy) - 1;
	if (gx0 < 0 || gy0 < 0 || gx1 < gx0 || gy1 < gy0 ||
	    gx1 >= pitch_px || gx1 > N32H7XX_POLY_LIMIT ||
	    gy1 > N32H7XX_POLY_LIMIT)
		return -EINVAL;
	end = (u64)args->offset + (u64)gy1 * args->pitch + (gx1 + 1) * cpp;
	if (end > obj->size)
		return -EINVAL;

	geom->count = 3;
	geom->xmin = max_t(s64, gx0 - 1, 0);
	geom->ymin = max_t(s64, gy0 - 1, 0);
	geom->xmax = min_t(s64, gx1 + 1, (s64)pitch_px - 1);
	last_row = (obj->size - args->offset) / args->pitch - 1;
	end = (u64)args->offset + (u64)(gy1 + 1) * args->pitch +
	      (gx1 + 1) * cpp;
	if (gy1 + 1 <= last_row && end <= obj->size)
		geom->ymax = min_t(s64, gy1 + 1, N32H7XX_POLY_LIMIT);
	else
		geom->ymax = gy1;

	ret = n32h7xx_curve_leg(&geom->edge[0], x[0], y[0], x[1], y[1],
				x[2], y[2], geom->xmin << 16, geom->ymin << 16);
	if (ret)
		return ret;
	ret = n32h7xx_curve_leg(&geom->edge[1], x[0], y[0], x[2], y[2],
				x[1], y[1], geom->xmin << 16, geom->ymin << 16);
	if (ret)
		return ret;
	ret = n32h7xx_curve_chord(&geom->edge[2], x[1], y[1], x[2], y[2],
				  x[0], y[0], geom->xmin << 16, geom->ymin << 16);
	if (ret)
		return ret;
	job->cpp = cpp;
	job->org = org;
	job->pitch_px = pitch_px;
	return 0;
}

static unsigned int n32h7xx_gpu_emit_poly_body(const struct n32h7xx_job *job,
					       dma_addr_t addr, u32 *s)
{
	const struct drm_n32h7xx_poly *args = job->poly;
	const struct drm_n32h7xx_curve *curve = job->curve;
	const struct n32h7xx_poly_geom *geom = &job->geom;
	u32 color = curve ? curve->color : args->color;
	u32 offset = curve ? curve->offset : args->offset;
	u32 bez_aa = 0;
	u32 lim_ctrl;
	unsigned int n = 0, i;
	u32 ctrl = 0;

	if (curve) {
		/* Limiter 0 and 1 are the curve legs (hard edges, the Bezier
		 * unit supplies the coverage) and limiter 2 is the chord.
		 */
		/* Every antialiased limiter becomes another curve leg, so the
		 * chord has to stay a hard edge; only the legs ramp.
		 */
		lim_ctrl = GPU_LIM_CTRL_NO_AA | (GPU_LIM_CTRL_NO_AA << 4);
		lim_ctrl |= GPU_LIM_CTRL_NO_AA << 8;
		lim_ctrl |= GPU_LIM_CTRL_BEZIER;
		bez_aa = (curve->flags & N32H7XX_CURVE_NO_AA) ?
			 0 : GPU_BEZ_AA_ONE_PX;
	} else {
		lim_ctrl = (args->flags & N32H7XX_POLY_NO_AA) ?
			   GPU_LIM_CTRL_NO_AA : GPU_LIM_CTRL_AA;
	}
	for (i = 0; i < geom->count; i++)
		ctrl |= lim_ctrl << (4 * i);

	/* The destination is read: the blend unit needs it for both the
	 * source-over alpha and the coverage ramp.
	 */
	s[n++] = GPU_OP_WRITE(GPU_FBS_CONFIG, 3);
	s[n++] = GPU_FBS_ALPHA_PASS | GPU_FBS_READ_FOR_BLEND;
	s[n++] = job->pitch_px;
	s[n++] = GPU_FBS_SPAN_DEFAULT;
	s[n++] = GPU_OP_WRITE(GPU_TEX_GLOBAL, 1);
	s[n++] = 0;
	s[n++] = GPU_OP_WRITE(GPU_COL_GLOBAL, 1);
	s[n++] = 1;
	s[n++] = GPU_OP_WRITE(GPU_COL_CONST0, 1);
	s[n++] = color;
	s[n++] = GPU_OP_WRITE(GPU_COL_OP1A0, 6);
	s[n++] = GPU_COL_OP_IDENTITY;
	s[n++] = GPU_COL_OP_IDENTITY;
	s[n++] = GPU_COL_OP_IDENTITY;
	s[n++] = GPU_COL_OP_IDENTITY;
	s[n++] = GPU_COL_OP3_CONST0;
	s[n++] = GPU_COL_OUT_UNITY;
	/* Six limiter blocks of (vstart, dx, dy); unused ones are cleared. */
	s[n++] = GPU_OP_WRITE(GPU_PSU_VSTART0, 18);
	for (i = 0; i < 6; i++) {
		s[n++] = i < geom->count ? (u32)geom->edge[i].vstart : 0;
		s[n++] = i < geom->count ? (u32)geom->edge[i].dx : 0;
		s[n++] = i < geom->count ? (u32)geom->edge[i].dy : 0;
	}
	/* Bezier scale and offsets stay zero; the legs are normalised by the
	 * driver.  Writing them every job keeps state from leaking.
	 */
	s[n++] = GPU_OP_WRITE(GPU_PSU_BEZ_CTRL, 4);
	s[n++] = 0;
	s[n++] = 0;
	s[n++] = 0;
	s[n++] = bez_aa;
	/* No stripe enumeration. */
	s[n++] = GPU_OP_WRITE(GPU_PSU_STRIPE, 2);
	s[n++] = 0;
	s[n++] = 0;
	s[n++] = GPU_OP_WRITE(GPU_PSU_CLIP, 1);
	s[n++] = GPU_CLIP_DEFAULT;
	s[n++] = GPU_OP_WRITE(GPU_BLU_PIXEL_ORG, 5);
	s[n++] = job->org;
	s[n++] = ALIGN_DOWN(addr + offset, GPU_FB_ADDRESS_ALIGN);
	s[n++] = GPU_BLU_BLEND_SRC_OVER;
	s[n++] = 0;
	s[n++] = GPU_BLU_WRITE_ALL | GPU_BLU_WRITE_COVERAGE;
	s[n++] = GPU_OP_WRITE(GPU_PSU_BBOX_MIN, 4);
	s[n++] = geom->xmin;
	s[n++] = (geom->ymax << 16) | geom->xmax;
	s[n++] = geom->ymin << 16;
	s[n++] = ctrl;
	return n;
}

/* The batch limit is chosen so full per-polygon state still fits one stream. */
#define GPU_POLY_BODY_WORDS	58
#define GPU_POLY_BATCH_MAX	DRM_N32H7XX_POLY_BATCH_MAX
static_assert(GPU_POLY_BODY_WORDS * GPU_POLY_BATCH_MAX + 2 <= GPU_STREAM_WORDS);

static void n32h7xx_gpu_emit_poly(struct n32h7xx_gpu *gpu,
				  const struct n32h7xx_job *job, dma_addr_t addr)
{
	u32 *s = gpu->stream;
	unsigned int n;

	n = n32h7xx_gpu_emit_poly_body(job, addr, s);
	s[n++] = GPU_OP_WAIT(GPU_WAIT_PIPE);
	s[n++] = GPU_OP_END;
}

static void n32h7xx_gpu_emit_poly_batch(struct n32h7xx_gpu *gpu,
					const struct n32h7xx_job *job,
					dma_addr_t addr)
{
	const struct n32h7xx_poly_batch *batch = job->poly_batch;
	u32 *s = gpu->stream;
	unsigned int n = 0, i;

	for (i = 0; i < batch->count; i++) {
		const struct n32h7xx_poly_batch_item *item = &batch->item[i];
		struct n32h7xx_job one = {
			.poly = &batch->polys[i],
			.geom = item->geom,
			.org = item->org,
			.pitch_px = item->pitch_px,
		};

		/* Validation and the compile-time worst-case check above keep
		 * this from overrunning the coherent command buffer.
		 */
		if (n + GPU_POLY_BODY_WORDS + 2 > GPU_STREAM_WORDS)
			break;
		n += n32h7xx_gpu_emit_poly_body(&one, addr, s + n);
	}
	s[n++] = GPU_OP_WAIT(GPU_WAIT_PIPE);
	s[n++] = GPU_OP_END;
}

/*
 * A short job finishes while the caller is still running.  Going to sleep and
 * being woken by the interrupt costs tens of microseconds on this single-core
 * no-MMU system, which is most of the cost of one small rectangle.  Spin for
 * a bounded time first, then fall back to the normal completion wait, so a
 * long job does not burn CPU.
 */
#define GPU_SPIN_US	40

static bool n32h7xx_gpu_wait_spin(struct n32h7xx_gpu *gpu)
{
	ktime_t deadline = ktime_add_us(ktime_get(), GPU_SPIN_US);

	do {
		if (READ_ONCE(gpu->irq_status) &
		    (GPU_IRQ_STOP | GPU_IRQ_BUS_ERROR))
			return true;
		cpu_relax();
	} while (ktime_before(ktime_get(), deadline));

	return false;
}

static int n32h7xx_gpu_run(struct n32h7xx_gpu *gpu)
{
	u32 status;
	int ret;

	reinit_completion(&gpu->done);
	WRITE_ONCE(gpu->irq_status, 0);
	writel(GPU_IRQ_MASK, gpu->regs + GPU_IRQ_STATUS);
	writel(0, gpu->regs + GPU_RING_PAUSE);
	writel(GPU_IRQ_STOP | GPU_IRQ_BUS_ERROR | GPU_CLOCK_ALWAYS_ON |
	       GPU_LIMIT_PREFETCH, gpu->regs + GPU_CONTROL);
	dma_wmb();
	writel(gpu->stream_dma, gpu->regs + GPU_STREAM_ADDR);
	/* Do not drop buffer references on a signal while DMA is in progress. */
	if (!n32h7xx_gpu_wait_spin(gpu) &&
	    !wait_for_completion_timeout(&gpu->done, GPU_TIMEOUT)) {
		ret = -ETIMEDOUT;
		goto reset;
	}
	status = READ_ONCE(gpu->irq_status);
	if (status & (GPU_IRQ_BUS_ERROR | GPU_STOP_CODE)) {
		ret = -EIO;
		goto reset;
	}
	ret = readl_poll_timeout(gpu->regs + GPU_BUSY, status, !status, 10, 10000);
	if (ret)
		goto reset;
	dma_rmb();
	return 0;
reset:
	{
		u32 irq_status = READ_ONCE(gpu->irq_status);
		u32 busy = readl(gpu->regs + GPU_BUSY);

		writel(0, gpu->regs + GPU_CONTROL);
		if (reset_control_reset(gpu->reset))
			gpu->wedged = true;
		synchronize_irq(gpu->irq);
		dev_err_ratelimited(gpu->drm.dev,
				    "GPU job failed: %d (irq %#x busy %#x)\n",
				    ret, irq_status, busy);
	}
	return ret;
}

/*
 * Look the buffer up, serialise with the other users of the shared command
 * buffer, build the stream for this job and run it to completion.
 */
static int n32h7xx_gpu_exec(struct drm_device *drm, struct drm_file *file,
			    u32 handle, struct n32h7xx_job *job)
{
	struct n32h7xx_gpu *gpu = container_of(drm, struct n32h7xx_gpu, drm);
	struct drm_gem_object *obj;
	struct n32h7xx_fence *fence;
	long waited;
	int ret, idx;

	if (!drm_dev_enter(drm, &idx))
		return -ENODEV;
	obj = drm_gem_object_lookup(file, handle);
	if (!obj) {
		ret = -ENOENT;
		goto exit;
	}
	ret = job->validate(obj, job);
	if (ret)
		goto put;
	ret = mutex_lock_interruptible(&gpu->lock);
	if (ret)
		goto put;
	if (gpu->wedged) {
		ret = -EIO;
		goto unlock;
	}
	ret = dma_resv_lock_interruptible(obj->resv, NULL);
	if (ret)
		goto unlock;
	waited = dma_resv_wait_timeout(obj->resv, DMA_RESV_USAGE_READ, true,
				       GPU_TIMEOUT);
	if (waited <= 0) {
		ret = waited ? waited : -ETIMEDOUT;
		goto unlock_resv;
	}
	ret = dma_resv_reserve_fences(obj->resv, 1);
	if (ret)
		goto unlock_resv;
	fence = kzalloc_obj(*fence);
	if (!fence) {
		ret = -ENOMEM;
		goto unlock_resv;
	}
	spin_lock_init(&fence->lock);
	dma_fence_init(&fence->base, &n32h7xx_fence_ops, &fence->lock,
		       gpu->fence_context, ++gpu->seqno);
	dma_resv_add_fence(obj->resv, &fence->base, DMA_RESV_USAGE_WRITE);
	dma_resv_unlock(obj->resv);
	job->emit(gpu, job, to_drm_gem_dma_obj(obj)->dma_addr);
	ret = n32h7xx_gpu_run(gpu);
	if (!ret && job->profile) {
		unsigned int i;

		for (i = 0; i < ARRAY_SIZE(job->profile->counter); i++)
			job->profile->counter[i] = gpu->dump[i];
	}
	if (gpu->wedged) {
		/* An unconfirmed reset must never return live DMA memory to a pool. */
		drm_gem_object_get(obj);
		gpu->stuck = obj;
	}
	if (ret)
		dma_fence_set_error(&fence->base, ret);
	dma_fence_signal(&fence->base);
	dma_fence_put(&fence->base);
	goto unlock;
unlock_resv:
	dma_resv_unlock(obj->resv);
unlock:
	mutex_unlock(&gpu->lock);
put:
	drm_gem_object_put(obj);
exit:
	drm_dev_exit(idx);
	return ret;
}

static struct n32h7xx_draw n32h7xx_draw_of(const void *args)
{
	const struct n32h7xx_draw *draw = args;

	return *draw;
}

static int n32h7xx_gpu_fill(struct drm_device *drm, void *data,
			    struct drm_file *file)
{
	struct n32h7xx_draw draw = n32h7xx_draw_of(data);
	struct n32h7xx_job job = {
		.validate = n32h7xx_gpu_validate_fill,
		.emit = n32h7xx_gpu_emit_fill,
		.draw = &draw,
	};

	return n32h7xx_gpu_exec(drm, file, draw.handle, &job);
}

static int n32h7xx_gpu_rect(struct drm_device *drm, void *data,
			    struct drm_file *file)
{
	struct n32h7xx_draw draw = n32h7xx_draw_of(data);
	struct n32h7xx_job job = {
		.validate = n32h7xx_gpu_validate_rect,
		.emit = n32h7xx_gpu_emit_rect,
		.draw = &draw,
	};

	return n32h7xx_gpu_exec(drm, file, draw.handle, &job);
}

/*
 * The rectangle array is copied in before the job starts, so a bad pointer or
 * a fault part way through the array cannot leave half a batch drawn.
 */
static int n32h7xx_gpu_rect_batch(struct drm_device *drm, void *data,
				  struct drm_file *file)
{
	struct drm_n32h7xx_rect_batch *args = data;
	struct n32h7xx_gpu *gpu = container_of(drm, struct n32h7xx_gpu, drm);
	struct n32h7xx_draw draw = { .handle = args->handle };
	struct n32h7xx_batch batch;
	struct n32h7xx_job job = {
		.validate = n32h7xx_gpu_validate_rect_batch,
		.emit = n32h7xx_gpu_emit_rect_batch,
		.draw = &draw,
		.batch = &batch,
	};
	int ret;

	if (args->reserved || args->pad || !args->count || !args->rects ||
	    args->count > DRM_N32H7XX_BATCH_MAX)
		return -EINVAL;
	if (mutex_lock_interruptible(&gpu->batch_lock))
		return -ERESTARTSYS;
	if (copy_from_user(gpu->batch_rects, u64_to_user_ptr(args->rects),
			   array_size(args->count, sizeof(*gpu->batch_rects)))) {
		mutex_unlock(&gpu->batch_lock);
		return -EFAULT;
	}
	batch.rects = gpu->batch_rects;
	batch.item = gpu->batch_items;
	batch.count = args->count;
	ret = n32h7xx_gpu_exec(drm, file, draw.handle, &job);
	mutex_unlock(&gpu->batch_lock);
	return ret;
}

static int n32h7xx_gpu_blend(struct drm_device *drm, void *data,
			     struct drm_file *file)
{
	struct n32h7xx_draw draw = n32h7xx_draw_of(data);
	struct n32h7xx_job job = {
		.validate = n32h7xx_gpu_validate_rect,
		.emit = n32h7xx_gpu_emit_rect,
		.draw = &draw,
		.blend = true,
	};

	return n32h7xx_gpu_exec(drm, file, draw.handle, &job);
}

/* The DRM core hands _IOW commands a kernel copy of the argument. */
static int n32h7xx_gpu_poly(struct drm_device *drm, void *data,
			    struct drm_file *file)
{
	const struct drm_n32h7xx_poly *poly = data;
	struct n32h7xx_job job = {
		.validate = n32h7xx_gpu_validate_poly,
		.emit = n32h7xx_gpu_emit_poly,
		.poly = poly,
	};

	return n32h7xx_gpu_exec(drm, file, poly->handle, &job);
}

static int n32h7xx_gpu_poly_batch(struct drm_device *drm, void *data,
				  struct drm_file *file)
{
	struct drm_n32h7xx_poly_batch *args = data;
	struct n32h7xx_gpu *gpu = container_of(drm, struct n32h7xx_gpu, drm);
	struct n32h7xx_draw draw = { .handle = args->handle };
	struct n32h7xx_poly_batch batch;
	struct n32h7xx_job job = {
		.validate = n32h7xx_gpu_validate_poly_batch,
		.emit = n32h7xx_gpu_emit_poly_batch,
		.draw = &draw,
		.poly_batch = &batch,
	};
	int ret;

	if (args->reserved || args->pad || !args->count || !args->polys ||
	    args->count > DRM_N32H7XX_POLY_BATCH_MAX)
		return -EINVAL;
	if (mutex_lock_interruptible(&gpu->batch_lock))
		return -ERESTARTSYS;
	if (copy_from_user(gpu->batch_polys, u64_to_user_ptr(args->polys),
			   array_size(args->count, sizeof(*gpu->batch_polys)))) {
		mutex_unlock(&gpu->batch_lock);
		return -EFAULT;
	}
	batch.polys = gpu->batch_polys;
	batch.item = gpu->batch_poly_items;
	batch.count = args->count;
	ret = n32h7xx_gpu_exec(drm, file, draw.handle, &job);
	mutex_unlock(&gpu->batch_lock);
	return ret;
}

static int n32h7xx_gpu_curve(struct drm_device *drm, void *data,
			     struct drm_file *file)
{
	const struct drm_n32h7xx_curve *curve = data;
	struct n32h7xx_job job = {
		.validate = n32h7xx_gpu_validate_curve,
		.emit = n32h7xx_gpu_emit_poly,
		.curve = curve,
	};

	return n32h7xx_gpu_exec(drm, file, curve->handle, &job);
}

/*
 * Texture coordinates are IEEE-754 single precision normalised to the source
 * rectangle and the hardware texel index is floor(u * size), clamped to the
 * rectangle.  A destination pixel centre therefore sits (x + 0.5) * size /
 * extent texels into the source.  A mirrored axis samples size - 1 - f, which
 * floor() reproduces only while f is not a whole texel, so both directions are
 * biased a quarter of a texel inwards: nearest samples are then at least a
 * quarter texel from any boundary, and an exact tie reads the higher texel
 * instead of one ulp of arithmetic deciding it.
 *
 * Build the bit patterns with integer arithmetic: the kernel never touches the
 * FPU, and 64-bit division goes through div_u64() because this build has no
 * libgcc helpers for __aeabi_uldivmod.
 */
static u32 n32h7xx_float_div(u32 num, u32 den)
{
	u32 mantissa;
	int scale;
	u64 q;

	if (!num)
		return 0;
	/* q = num/den * 2^scale with as many bits as a u64 holds.  Keeping 24
	 * of them is finer than the 1/(2 * 4096) step the coordinates need.
	 */
	scale = 63 - fls(num);
	q = div_u64((u64)num << scale, den);
	while (q >= (1U << 24)) {
		q >>= 1;
		scale--;
	}
	while (q < (1U << 23)) {
		q <<= 1;
		scale++;
	}
	mantissa = (u32)q;
	return ((u32)(150 - scale) << 23) | (mantissa - (1U << 23));
}

#define N32H7XX_FLOAT_NEG(v)	((v) ^ 0x80000000u)

/*
 * Normalised start of one source axis that `extent` destination pixels step
 * through: half a destination step plus the bias, from either end.
 */
static u32 n32h7xx_axis_start(u32 size, u32 extent, bool mirrored)
{
	u32 den = 4 * extent * size;

	if (!mirrored)
		return n32h7xx_float_div(2 * size + 1, den);
	return n32h7xx_float_div(den - 2 * size - 1, den);
}

static int n32h7xx_blit_format(u32 format, u32 *cpp, u32 *org, u32 *tex_org)
{
	const struct n32h7xx_format *fmt = n32h7xx_format_of(format);

	if (!fmt)
		return -EINVAL;
	*cpp = fmt->cpp;
	*org = fmt->org;
	*tex_org = fmt->tex_org;
	return 0;
}

/* Check one side of a blit: the block must lie inside the buffer. */
static int n32h7xx_blit_surface(struct drm_gem_object *obj, u32 format,
				u32 offset, u32 pitch, u32 width, u32 height,
				u32 *cpp, u32 *org, u32 *tex_org, u32 *pitch_px)
{
	u64 end;
	u32 bytes;
	int ret;

	ret = n32h7xx_blit_format(format, cpp, org, tex_org);
	if (ret)
		return ret;
	if (!IS_ALIGNED(offset, *cpp) || !IS_ALIGNED(pitch, *cpp))
		return -EINVAL;
	bytes = width * *cpp;
	if (pitch < bytes || pitch > 32767)
		return -EINVAL;
	*pitch_px = pitch / *cpp;
	if (*pitch_px > 8191)
		return -EINVAL;
	end = (u64)offset + (u64)(height - 1) * pitch + bytes;
	if (end > obj->size)
		return -EINVAL;
	return 0;
}

static int n32h7xx_gpu_blit_validate(struct drm_gem_object **objs,
				     const struct drm_n32h7xx_blit *args,
				     struct n32h7xx_blit *blit)
{
	u32 cpp, org, tex_org, step_x, step_y, u_extent, v_extent;
	int ux, uy, vx, vy, ret;

	if ((args->flags & ~DRM_N32H7XX_BLIT_BLEND) ||
	    (args->rotation & ~(DRM_MODE_ROTATE_MASK | DRM_MODE_REFLECT_MASK)))
		return -EINVAL;
	if (!args->width || args->width > 4096 || !args->height ||
	    args->height > 4096 || !args->src_width || !args->src_height ||
	    args->src_width > 4096 || args->src_height > 4096)
		return -EINVAL;
	/* Both sides are reserved in one ww transaction; the same buffer twice
	 * would make that transaction wait on itself.
	 */
	if (args->src_handle == args->dst_handle)
		return -EINVAL;

	ret = n32h7xx_blit_surface(objs[0], args->src_format, args->src_offset,
				   args->src_pitch, args->src_width,
				   args->src_height, &cpp, &org, &tex_org,
				   &blit->src_pitch_px);
	if (ret)
		return ret;
	/*
	 * Blending needs a sampled alpha.  RGB565 and XRGB8888 have no alpha
	 * field, and their spare byte is not an opacity value, so they cannot
	 * be the source of a blend.  The destination may be any format; its
	 * alpha is only consulted for the alpha output.
	 * n32h7xx_blit_surface() above has already rejected an unknown source
	 * format, so the lookup cannot be NULL here.
	 */
	if ((args->flags & DRM_N32H7XX_BLIT_BLEND) &&
	    !n32h7xx_format_of(args->src_format)->alpha)
		return -EINVAL;
	blit->blend = !!(args->flags & DRM_N32H7XX_BLIT_BLEND);
	blit->src_offset = args->src_offset;
	blit->src_tex_org = tex_org;
	blit->src_width = args->src_width;
	blit->src_height = args->src_height;

	ret = n32h7xx_blit_surface(objs[1], args->dst_format, args->dst_offset,
				   args->dst_pitch, args->width, args->height,
				   &cpp, &org, &tex_org, &blit->dst_pitch_px);
	if (ret)
		return ret;
	blit->dst_offset = args->dst_offset;
	blit->dst_org = org;
	blit->width = args->width;
	blit->height = args->height;
	/* FBC_START_ADDRESS ignores its low three bits. */
	if (args->dst_offset % GPU_FB_ADDRESS_ALIGN % cpp)
		return -EINVAL;
	blit->xmin = (args->dst_offset % GPU_FB_ADDRESS_ALIGN) / cpp;
	if (blit->xmin + args->width > 8192)
		return -EINVAL;

	/*
	 * Which way the source runs inside the destination block.  Rotation
	 * comes first and the reflection mirrors the rotated image, so a
	 * reflection flips the sign of the source axis that maps along the
	 * reflected destination axis.
	 */
	switch (args->rotation & DRM_MODE_ROTATE_MASK) {
	case 0:
	case DRM_MODE_ROTATE_0:
		ux = 1; uy = 0; vx = 0; vy = 1;
		break;
	case DRM_MODE_ROTATE_90:	/* sx = width - 1 - y, sy = x */
		ux = 0; uy = -1; vx = 1; vy = 0;
		break;
	case DRM_MODE_ROTATE_180:
		ux = -1; uy = 0; vx = 0; vy = -1;
		break;
	case DRM_MODE_ROTATE_270:	/* sx = y, sy = height - 1 - x */
		ux = 0; uy = 1; vx = -1; vy = 0;
		break;
	default:
		return -EINVAL;
	}
	if (args->rotation & DRM_MODE_REFLECT_X) {
		ux = -ux;
		vx = -vx;
	}
	if (args->rotation & DRM_MODE_REFLECT_Y) {
		uy = -uy;
		vy = -vy;
	}

	/* A source axis steps once per destination pixel along the axis it is
	 * mapped onto, which is the destination height for a rotated 90/270.
	 */
	u_extent = ux ? args->width : args->height;
	v_extent = vx ? args->width : args->height;
	step_x = n32h7xx_float_div(1, args->width);
	step_y = n32h7xx_float_div(1, args->height);
	blit->u0 = n32h7xx_axis_start(args->src_width, u_extent,
				      (ux ? ux : uy) < 0);
	blit->v0 = n32h7xx_axis_start(args->src_height, v_extent,
				      (vx ? vx : vy) < 0);
	blit->dux = ux ? (ux > 0 ? step_x : N32H7XX_FLOAT_NEG(step_x)) : 0;
	blit->duy = uy ? (uy > 0 ? step_y : N32H7XX_FLOAT_NEG(step_y)) : 0;
	blit->dvx = vx ? (vx > 0 ? step_x : N32H7XX_FLOAT_NEG(step_x)) : 0;
	blit->dvy = vy ? (vy > 0 ? step_y : N32H7XX_FLOAT_NEG(step_y)) : 0;
	/*
	 * Only a rotated blit reads the source along its columns.  For 0, 180
	 * and the reflections every destination row is still a run of adjacent
	 * texels, and splitting those rows into stripes costs an order of
	 * magnitude (measured: 400x240 1:1 goes from 6.1 ms to 88 ms).
	 */
	blit->stripe = vx ? min(GPU_BLIT_STRIPE, args->width) : 0;
	return 0;
}

/*
 * One texture unit samples the source and the colour unit passes it through
 * unchanged.  An opaque copy discards the texture alpha (alpha becomes 1.0)
 * and does not read the destination; a blend keeps the sampled alpha, reads
 * the destination back and composites source-over, exactly like the BLEND
 * rectangle.  The limiter control write starts the enumeration, so it comes
 * last.
 */
static void n32h7xx_gpu_emit_blit(struct n32h7xx_gpu *gpu,
				  const struct n32h7xx_blit *blit,
				  dma_addr_t src, dma_addr_t dst)
{
	u32 *s = gpu->stream;
	unsigned int n = 0;

	s[n++] = GPU_OP_WRITE(GPU_TEX_GLOBAL, 1);
	s[n++] = 1;			/* one active texture unit */
	s[n++] = GPU_OP_WRITE(GPU_TEX_MODE0, 3);
	s[n++] = (blit->blend ? 0 : GPU_TEX_DISCARD_ALPHA) |
		 (blit->src_tex_org << 8);
	s[n++] = src + blit->src_offset;
	s[n++] = 0;			/* TXP_CTRL: linear texture, no CLUT */
	s[n++] = GPU_OP_WRITE(GPU_TXA_SIZE0, 2);
	s[n++] = (blit->src_height << 16) | blit->src_width;
	s[n++] = blit->src_pitch_px;
	s[n++] = GPU_OP_WRITE(GPU_TXA_U0, 6);
	s[n++] = blit->u0;
	s[n++] = blit->v0;
	s[n++] = blit->dux;
	s[n++] = blit->duy;
	s[n++] = blit->dvx;
	s[n++] = blit->dvy;
	s[n++] = GPU_OP_WRITE(GPU_COL_GLOBAL, 1);
	s[n++] = 1;			/* one colour pass */
	s[n++] = GPU_OP_WRITE(GPU_COL_OP1A0, 6);
	s[n++] = GPU_COL_OP_IDENTITY;	/* op1a: constant 0 */
	s[n++] = GPU_COL_OP_IDENTITY;	/* op1b: constant 0 */
	s[n++] = GPU_COL_OP_IDENTITY;	/* op2a: constant 0 */
	s[n++] = GPU_COL_OP_IDENTITY;	/* op2b: constant 0 */
	s[n++] = GPU_COL_OP3_TEX0;	/* op3: the sampled texel */
	s[n++] = GPU_COL_OUT_UNITY;
	s[n++] = GPU_OP_WRITE(GPU_FBS_CONFIG, 3);
	/* A blend reads the destination back; an opaque copy does not. */
	s[n++] = GPU_FBS_ALPHA_PASS |
		 (blit->blend ? GPU_FBS_READ_FOR_BLEND : 0);
	s[n++] = blit->dst_pitch_px;
	s[n++] = GPU_FBS_SPAN_DEFAULT;
	s[n++] = GPU_OP_WRITE(GPU_BLU_PIXEL_ORG, 5);
	s[n++] = blit->dst_org;
	s[n++] = ALIGN_DOWN(dst + blit->dst_offset, GPU_FB_ADDRESS_ALIGN);
	s[n++] = blit->blend ? GPU_BLU_BLEND_SRC_OVER : GPU_BLU_BLEND_OPAQUE;
	s[n++] = 0;
	s[n++] = GPU_BLU_WRITE_ALL;
	/* Enumeration stripes for a rotated source; zero disables them and also
	 * clears the bezier value a previous curve job may have left.
	 */
	s[n++] = GPU_OP_WRITE(GPU_PSU_STRIPE, 2);
	s[n++] = blit->stripe;
	s[n++] = 0;
	s[n++] = GPU_OP_WRITE(GPU_PSU_BBOX_MIN, 4);
	s[n++] = blit->xmin;
	s[n++] = ((blit->height - 1) << 16) | (blit->xmin + blit->width - 1);
	s[n++] = 0;
	s[n++] = 0;			/* no limiters: fill the bounding box */
	s[n++] = GPU_OP_WAIT(GPU_WAIT_PIPE);
	s[n++] = GPU_OP_END;
}

/*
 * A blit touches two buffers, so both reservations have to be taken in one
 * ww transaction and both get a fence: the source is read, the destination is
 * written.
 */
static int n32h7xx_gpu_exec_blit(struct drm_device *drm, struct drm_file *file,
				 struct drm_n32h7xx_blit *args)
{
	struct n32h7xx_gpu *gpu = container_of(drm, struct n32h7xx_gpu, drm);
	struct drm_gem_object *objs[2] = { };
	struct ww_acquire_ctx ctx;
	struct n32h7xx_fence *fence;
	struct n32h7xx_blit blit = { };
	long waited;
	int ret, idx, i;

	if (!drm_dev_enter(drm, &idx))
		return -ENODEV;
	objs[0] = drm_gem_object_lookup(file, args->src_handle);
	objs[1] = drm_gem_object_lookup(file, args->dst_handle);
	if (!objs[0] || !objs[1]) {
		ret = -ENOENT;
		goto put;
	}
	ret = n32h7xx_gpu_blit_validate(objs, args, &blit);
	if (ret)
		goto put;
	ret = mutex_lock_interruptible(&gpu->lock);
	if (ret)
		goto put;
	if (gpu->wedged) {
		ret = -EIO;
		goto unlock;
	}
	ret = drm_gem_lock_reservations(objs, 2, &ctx);
	if (ret)
		goto unlock;
	for (i = 0; i < 2; i++) {
		waited = dma_resv_wait_timeout(objs[i]->resv,
					       DMA_RESV_USAGE_WRITE, true,
					       GPU_TIMEOUT);
		if (waited <= 0) {
			ret = waited ? waited : -ETIMEDOUT;
			goto unlock_resv;
		}
		ret = dma_resv_reserve_fences(objs[i]->resv, 1);
		if (ret)
			goto unlock_resv;
	}
	fence = kzalloc_obj(*fence);
	if (!fence) {
		ret = -ENOMEM;
		goto unlock_resv;
	}
	spin_lock_init(&fence->lock);
	dma_fence_init(&fence->base, &n32h7xx_fence_ops, &fence->lock,
		       gpu->fence_context, ++gpu->seqno);
	dma_resv_add_fence(objs[0]->resv, &fence->base, DMA_RESV_USAGE_READ);
	dma_resv_add_fence(objs[1]->resv, &fence->base, DMA_RESV_USAGE_WRITE);
	drm_gem_unlock_reservations(objs, 2, &ctx);
	n32h7xx_gpu_emit_blit(gpu, &blit,
			      to_drm_gem_dma_obj(objs[0])->dma_addr,
			      to_drm_gem_dma_obj(objs[1])->dma_addr);
	ret = n32h7xx_gpu_run(gpu);
	if (gpu->wedged) {
		/* An unconfirmed reset must never return live DMA memory to a pool. */
		drm_gem_object_get(objs[1]);
		gpu->stuck = objs[1];
	}
	if (ret)
		dma_fence_set_error(&fence->base, ret);
	dma_fence_signal(&fence->base);
	dma_fence_put(&fence->base);
	goto unlock;
unlock_resv:
	drm_gem_unlock_reservations(objs, 2, &ctx);
unlock:
	mutex_unlock(&gpu->lock);
put:
	if (objs[0])
		drm_gem_object_put(objs[0]);
	if (objs[1])
		drm_gem_object_put(objs[1]);
	drm_dev_exit(idx);
	return ret;
}

static int n32h7xx_gpu_blit(struct drm_device *drm, void *data,
			    struct drm_file *file)
{
	return n32h7xx_gpu_exec_blit(drm, file, data);
}

static int n32h7xx_gpu_profile(struct drm_device *drm, void *data,
			       struct drm_file *file)
{
	struct drm_n32h7xx_profile *args = data;
	struct n32h7xx_draw draw = {
		.handle = args->handle,
		.format = args->format,
		.offset = args->offset,
		.pitch = args->pitch,
		.width = args->width,
		.height = args->height,
		.color = args->color,
	};
	struct n32h7xx_job job = {
		.validate = n32h7xx_gpu_validate_profile,
		.emit = n32h7xx_gpu_emit_profile,
		.draw = &draw,
		.profile = args,
	};

	return n32h7xx_gpu_exec(drm, file, draw.handle, &job);
}

static const struct drm_ioctl_desc n32h7xx_gpu_ioctls[] = {
	DRM_IOCTL_DEF_DRV(N32H7XX_CREATE, n32h7xx_gpu_create, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(N32H7XX_FILL, n32h7xx_gpu_fill, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(N32H7XX_RECT, n32h7xx_gpu_rect, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(N32H7XX_RECT_BATCH, n32h7xx_gpu_rect_batch,
			  DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(N32H7XX_BLEND, n32h7xx_gpu_blend, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(N32H7XX_BLIT, n32h7xx_gpu_blit, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(N32H7XX_POLY, n32h7xx_gpu_poly, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(N32H7XX_POLY_BATCH, n32h7xx_gpu_poly_batch,
			  DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(N32H7XX_CURVE, n32h7xx_gpu_curve, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(N32H7XX_PROFILE, n32h7xx_gpu_profile,
			  DRM_RENDER_ALLOW),
};

DEFINE_DRM_GEM_DMA_FOPS(n32h7xx_gpu_fops);

static const struct drm_driver n32h7xx_gpu_driver = {
	.driver_features = DRIVER_GEM | DRIVER_RENDER,
	DRM_GEM_DMA_DRIVER_OPS,
	.fops = &n32h7xx_gpu_fops,
	.ioctls = n32h7xx_gpu_ioctls,
	.num_ioctls = ARRAY_SIZE(n32h7xx_gpu_ioctls),
	.name = "n32h7xx-gpu",
	.desc = "NSING N32H7xx 2D clear engine",
	.major = 1,
};

static void n32h7xx_gpu_free_buffers(struct drm_device *drm, void *data)
{
	struct n32h7xx_gpu *gpu = container_of(drm, struct n32h7xx_gpu, drm);

	if (gpu->dump)
		dma_free_coherent(drm->dev, PAGE_SIZE, gpu->dump, gpu->dump_dma);
	if (gpu->stream)
		dma_free_coherent(drm->dev, GPU_STREAM_SIZE, gpu->stream,
				  gpu->stream_dma);
}

static int n32h7xx_gpu_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct n32h7xx_gpu *gpu;
	struct clk *clk;
	u32 version, config;
	int ret, irq;

	gpu = devm_drm_dev_alloc(dev, &n32h7xx_gpu_driver, struct n32h7xx_gpu, drm);
	if (IS_ERR(gpu))
		return PTR_ERR(gpu);
	mutex_init(&gpu->lock);
	mutex_init(&gpu->batch_lock);
	gpu->batch_rects = devm_kcalloc(dev, DRM_N32H7XX_BATCH_MAX,
					sizeof(*gpu->batch_rects), GFP_KERNEL);
	gpu->batch_items = devm_kcalloc(dev, DRM_N32H7XX_BATCH_MAX,
					sizeof(*gpu->batch_items), GFP_KERNEL);
	gpu->batch_polys = devm_kcalloc(dev, DRM_N32H7XX_POLY_BATCH_MAX,
					sizeof(*gpu->batch_polys), GFP_KERNEL);
	gpu->batch_poly_items = devm_kcalloc(dev, DRM_N32H7XX_POLY_BATCH_MAX,
					    sizeof(*gpu->batch_poly_items),
					    GFP_KERNEL);
	if (!gpu->batch_rects || !gpu->batch_items || !gpu->batch_polys ||
	    !gpu->batch_poly_items)
		return -ENOMEM;
	init_completion(&gpu->done);
	gpu->fence_context = dma_fence_context_alloc(1);
	ret = n32h7xx_graphics_acquire(dev, N32H7XX_GRAPHICS_GPU);
	if (ret)
		return ret;
	clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(clk))
		return PTR_ERR(clk);
	gpu->reset = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(gpu->reset))
		return PTR_ERR(gpu->reset);
	ret = reset_control_reset(gpu->reset);
	if (ret)
		return ret;
	gpu->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(gpu->regs))
		return PTR_ERR(gpu->regs);
	writel(0, gpu->regs + GPU_CONTROL);
	version = readl(gpu->regs + GPU_VERSION);
	config = readl(gpu->regs + GPU_CONFIG3);
	if (version >> 24 != 0xd4 ||
	    ((config & GPU_CONFIG3_EXTENDED) && !(config & GPU_CLEAR_PRESENT)))
		return dev_err_probe(dev, -ENODEV, "GPU clear engine unavailable\n");
	ret = n32h7xx_drm_init_memory(&gpu->drm);
	if (ret)
		return ret;
	gpu->stream = dma_alloc_coherent(dev, GPU_STREAM_SIZE, &gpu->stream_dma,
					 GFP_KERNEL);
	if (!gpu->stream)
		return -ENOMEM;
	gpu->dump = dma_alloc_coherent(dev, PAGE_SIZE, &gpu->dump_dma,
				       GFP_KERNEL);
	if (!gpu->dump) {
		dma_free_coherent(dev, PAGE_SIZE, gpu->stream, gpu->stream_dma);
		gpu->stream = NULL;
		return -ENOMEM;
	}
	ret = drmm_add_action_or_reset(&gpu->drm, n32h7xx_gpu_free_buffers,
				       NULL);
	if (ret)
		return ret;
	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
	gpu->irq = irq;
	ret = devm_request_irq(dev, irq, n32h7xx_gpu_irq, 0, dev_name(dev), gpu);
	if (ret)
		return ret;
	platform_set_drvdata(pdev, gpu);
	ret = drm_dev_register(&gpu->drm, 0);
	if (ret)
		return ret;
	dev_info(dev, "D/AVE HD %#x, config3 %#x, synchronous rectangle fill\n",
		 version, config);
	return 0;
}

static void n32h7xx_gpu_remove(struct platform_device *pdev)
{
	struct n32h7xx_gpu *gpu = platform_get_drvdata(pdev);

	drm_dev_unplug(&gpu->drm);
	writel(0, gpu->regs + GPU_CONTROL);
	if (!reset_control_assert(gpu->reset) && gpu->stuck) {
		drm_gem_object_put(gpu->stuck);
		gpu->stuck = NULL;
	}
}

static void n32h7xx_gpu_shutdown(struct platform_device *pdev)
{
	n32h7xx_gpu_remove(pdev);
}

static const struct of_device_id n32h7xx_gpu_match[] = {
	{ .compatible = "nsing,n32h7xx-gpu" },
	{ }
};
MODULE_DEVICE_TABLE(of, n32h7xx_gpu_match);

static struct platform_driver n32h7xx_gpu_platform_driver = {
	.probe = n32h7xx_gpu_probe,
	.remove = n32h7xx_gpu_remove,
	.shutdown = n32h7xx_gpu_shutdown,
	.driver = {
		.name = "n32h7xx-gpu",
		.of_match_table = n32h7xx_gpu_match,
	},
};
module_platform_driver(n32h7xx_gpu_platform_driver);

MODULE_DESCRIPTION("NSING N32H7xx 2D GPU DRM driver");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("DMA_BUF");
