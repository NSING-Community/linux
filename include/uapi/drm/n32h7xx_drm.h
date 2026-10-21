/* SPDX-License-Identifier: GPL-2.0-only WITH Linux-syscall-note */
#ifndef _UAPI_N32H7XX_DRM_H
#define _UAPI_N32H7XX_DRM_H

#include "drm.h"

/*
 * Pixel formats the GPU ioctls accept, all in little-endian byte order:
 * DRM_FORMAT_RGB565, DRM_FORMAT_XRGB8888, DRM_FORMAT_ARGB8888,
 * DRM_FORMAT_RGBA8888, DRM_FORMAT_ARGB4444, DRM_FORMAT_RGBA4444,
 * DRM_FORMAT_ARGB1555 and DRM_FORMAT_RGBA5551.
 *
 * The clear engine writes pixels unchanged, so a fill takes a raw pixel in
 * the target format.  The pixel pipeline writes a constant ARGB8888 colour
 * and lets the hardware convert it to the target format.  Any other format
 * is rejected with EINVAL.
 */

/* Allocate a zeroed, page-aligned GEM DMA buffer; flags must be zero. */
struct drm_n32h7xx_create {
	__u64 size;
	__u32 flags;
	__u32 handle;
	__u64 mmap_offset;
};

/*
 * Synchronous rectangle fill. pitch and offset are bytes. color is a raw
 * pixel in the buffer's own format: the low 16 bits for a 16-bit format, the
 * full 32 bits otherwise. Nothing is blended and no conversion happens.
 * The rectangle starts at offset; pitch is the distance between its rows.
 * reserved must be zero. The ioctl returns after the GPU has finished.
 * Share buffers with KMS using the standard PRIME handle/fd ioctls.
 */
struct drm_n32h7xx_fill {
	__u32 handle;
	__u32 format;
	__u32 offset;
	__u32 pitch;
	__u32 width;
	__u32 height;
	__u32 color;
	__u32 reserved;
};

#define DRM_N32H7XX_CREATE	0x00
#define DRM_N32H7XX_FILL		0x01
#define DRM_N32H7XX_RECT		0x02
#define DRM_N32H7XX_BLEND	0x03
#define DRM_N32H7XX_BLIT		0x04
#define DRM_N32H7XX_POLY		0x05
#define DRM_N32H7XX_CURVE	0x06
#define DRM_N32H7XX_PROFILE	0x07
#define DRM_N32H7XX_RECT_BATCH	0x08

/*
 * flags bit 0 of DRM_N32H7XX_RECT and DRM_N32H7XX_BLEND: enable the blend
 * unit's 4x4 ordered dither.  The hardware dithers towards the channel widths
 * of a 16-bit destination, so the flag is rejected for 32-bit formats.  The
 * pattern is keyed to absolute buffer coordinates, which keeps it stable when
 * an area is redrawn in pieces.  All other bits must be zero.
 */
#define DRM_N32H7XX_DITHER	0x1

/*
 * Solid rectangle drawn by the 2D pixel pipeline (bounding box + constant
 * colour), as opposed to DRM_N32H7XX_FILL which uses the clear engine.  The
 * rectangle is bounded by its extent, so nothing outside it is written.
 * color is a packed ARGB8888 value regardless of the target format; the
 * hardware converts it.  The rectangle is written fully opaque: the alpha
 * byte of color is ignored.  Use DRM_N32H7XX_BLEND to composite instead.
 * pitch and offset are in bytes.  flags is DRM_N32H7XX_DITHER or zero.
 */
struct drm_n32h7xx_rect {
	__u32 handle;
	__u32 format;
	__u32 offset;
	__u32 pitch;
	__u32 width;
	__u32 height;
	__u32 color;
	__u32 flags;
};

/*
 * Solid rectangle composited over the existing contents of the buffer with
 * the DRM KMS default pixel blend mode (non-premultiplied source-over):
 *
 *     out = src * color.a + dst * (1 - color.a)
 *
 * color is a straight-alpha ARGB8888 value.  The destination is read back,
 * blended and rewritten, so only the rectangle's extent is touched and
 * nothing outside it changes.  The alpha channel is blended independently
 * with the same factors, so a destination that stores alpha gets
 *
 *     out.a = src.a + dst.a * (1 - src.a)
 *
 * and every format from DRM_N32H7XX_RECT, including the ARGB/RGBA layouts,
 * is accepted.  Other fields, units and validation match DRM_N32H7XX_RECT.
 */
struct drm_n32h7xx_blend {
	__u32 handle;
	__u32 format;
	__u32 offset;
	__u32 pitch;
	__u32 width;
	__u32 height;
	__u32 color;
	__u32 flags;
};

/*
 * Draw many solid rectangles with one job.  rects points at count entries of
 * struct drm_n32h7xx_rect, which are validated exactly like
 * DRM_N32H7XX_RECT; every entry must name handle, but pitch, offset, extent,
 * colour and the dither flag may change from one entry to the next.
 * Entries are drawn in array order and all of them share one fence, so the
 * ioctl returns when the last rectangle has been written.  This exists
 * because a job costs 35-60 us of setup and wakeup regardless of its
 * size: a UI that submits one small rectangle per ioctl pays that per
 * rectangle, while a batch pays it once.
 *
 * count must be 1..DRM_N32H7XX_BATCH_MAX, rects must not be NULL and
 * reserved must be zero.  The array is copied in before any drawing starts.
 */
#define DRM_N32H7XX_BATCH_MAX	128

struct drm_n32h7xx_rect_batch {
	__u32 handle;
	__u32 count;
	__u64 rects;
	__u32 reserved;
	__u32 pad;
};

/*
 * Copy a source rectangle to a destination block, with optional scaling,
 * rotation and reflection.  The rectangle is src_width x src_height pixels
 * starting at src_offset; the destination block is width x height pixels
 * starting at dst_offset.  offset and pitch are in bytes, and pitch is the
 * distance between rows of the surface, so a rectangle narrower than its
 * surface is fine.  Source and destination may have different formats; the
 * texture and colour units convert between them.
 *
 * rotation uses the DRM plane encoding: one of DRM_MODE_ROTATE_0, _90, _180
 * or _270 (counter clockwise, 0 when no bit is set) plus DRM_MODE_REFLECT_X
 * and/or DRM_MODE_REFLECT_Y.  The rotation is applied first, then the
 * reflection, exactly like a DRM plane.  When the destination block differs
 * in size from the rotated rectangle the copy is scaled, sampling the nearest
 * source pixel for each destination pixel centre.  A sample that lands exactly
 * on a texel boundary takes the higher texel; nothing is filtered.
 *
 * With flags = 0 the copy is opaque: the source alpha channel is ignored and
 * the destination alpha is written as 1.0.  DRM_N32H7XX_BLIT_BLEND keeps the
 * sampled alpha and composites the result over the destination with the DRM
 * KMS default pixel blend mode (non-premultiplied source-over):
 *
 *     out.rgb = src.rgb * src.a + dst.rgb * (1 - src.a)
 *     out.a   = src.a + dst.a * (1 - src.a)
 *
 * Blending needs a source that carries alpha (ARGB8888, RGBA8888, ARGB4444,
 * RGBA4444, ARGB1555 or RGBA5551); an RGB565 or XRGB8888 source has no alpha
 * to read and is rejected.  The destination may be any format.  The two
 * handles must name different buffers.
 */

/* Composite the source over the destination instead of copying opaquely. */
#define DRM_N32H7XX_BLIT_BLEND	0x1

struct drm_n32h7xx_blit {
	__u32 src_handle;
	__u32 src_format;
	__u32 src_offset;
	__u32 src_pitch;
	__u32 src_width;
	__u32 src_height;
	__u32 dst_handle;
	__u32 dst_format;
	__u32 dst_offset;
	__u32 dst_pitch;
	__u32 width;
	__u32 height;
	__u32 rotation;
	__u32 flags;
};

/*
 * Count hardware events while drawing the same rectangle several times.
 *
 * The engine has four counters; each one is programmed with an event id
 * (0..69, see the hardware manual) and counts that event until the job ends.
 * DRM_N32H7XX_EVENT_OFF leaves the counter disabled and reports zero.  The
 * rectangle, its format and the validation are those of DRM_N32H7XX_RECT;
 * every iteration redraws exactly the same block, so a counter divided by
 * iterations is the cost of one rectangle.
 *
 * counters are cleared at the start of the job, so the values cover this
 * call only, and the counters are switched off again before the ioctl
 * returns.  The draw itself still happens: the pixels are written as usual.
 *
 * Useful events: 9 visible pixels, 26 span splits on the span limit,
 * 28/29 framebuffer burst reads/writes, 32/35 framebuffer pixels read/written,
 * 39 texture prefetch misses, 53 stream commands, 59 stream controller busy
 * cycles, 63 pixel selection busy cycles, 68 clear unit busy cycles and
 * 69 every clock.  iterations must be at least 1 and no more than the command
 * buffer holds; anything else is rejected with EINVAL.
 */
#define DRM_N32H7XX_EVENT_OFF	0xffffffffu

struct drm_n32h7xx_profile {
	__u32 handle;
	__u32 format;
	__u32 offset;
	__u32 pitch;
	__u32 width;
	__u32 height;
	__u32 color;
	__u32 iterations;
	__u32 event[4];
	__u32 counter[4];
};

/*
 * Convex polygon filled by the hardware pixel-selection unit: 3 to 6
 * vertices, each a 16.16 fixed-point coordinate in pixels relative to offset.
 * The winding order is free; the interior is the side of every edge that
 * contains the vertex centroid.  color is a straight-alpha ARGB8888 value and
 * is composited source-over with its alpha scaled by the hardware's 8-bit
 * coverage, so alpha 0 leaves the destination untouched.  Edges are
 * antialiased with a one pixel wide ramp unless flags bit 0 is set.  The
 * polygon must lie inside the buffer; offset must be 8-byte aligned and pitch
 * is in bytes.  format must not carry an alpha channel (RGB565 or XRGB8888).
 * reserved must be zero.
 */
struct drm_n32h7xx_poly {
	__u32 handle;
	__u32 format;
	__u32 offset;
	__u32 pitch;
	__u32 color;
	__u32 vertex_count;
	__u32 flags;
	__u32 reserved;
	__s32 x[6];
	__s32 y[6];
};

#define DRM_IOCTL_N32H7XX_CREATE \
	DRM_IOWR(DRM_COMMAND_BASE + DRM_N32H7XX_CREATE, struct drm_n32h7xx_create)
#define DRM_IOCTL_N32H7XX_FILL \
	DRM_IOW(DRM_COMMAND_BASE + DRM_N32H7XX_FILL, struct drm_n32h7xx_fill)
#define DRM_IOCTL_N32H7XX_RECT \
	DRM_IOW(DRM_COMMAND_BASE + DRM_N32H7XX_RECT, struct drm_n32h7xx_rect)
#define DRM_IOCTL_N32H7XX_RECT_BATCH \
	DRM_IOW(DRM_COMMAND_BASE + DRM_N32H7XX_RECT_BATCH, \
		struct drm_n32h7xx_rect_batch)
#define DRM_IOCTL_N32H7XX_BLEND \
	DRM_IOW(DRM_COMMAND_BASE + DRM_N32H7XX_BLEND, struct drm_n32h7xx_blend)
#define DRM_IOCTL_N32H7XX_BLIT \
	DRM_IOW(DRM_COMMAND_BASE + DRM_N32H7XX_BLIT, struct drm_n32h7xx_blit)
#define DRM_IOCTL_N32H7XX_POLY \
	DRM_IOW(DRM_COMMAND_BASE + DRM_N32H7XX_POLY, struct drm_n32h7xx_poly)

/*
 * Filled quadratic curve segment drawn by the pixel-selection unit.  (x1,y1)
 * and (x2,y2) are the segment end points and (x0,y0) is the intersection of
 * the tangents there; the hardware draws the parabola that is tangent to both
 * legs at the end points.  All coordinates are 16.16 fixed-point pixels
 * relative to offset.  The filled region is the part of the control
 * triangle between the curve and the chord (x1,y1)-(x2,y2), i.e. the side
 * away from (x0,y0).  The curve always bulges towards (x0,y0); the hardware's
 * complementary "concave" fill is not exposed because it does not produce a
 * stable filled region.
 * color is a straight-alpha ARGB8888 value composited with the hardware's
 * 8-bit coverage, exactly like DRM_N32H7XX_POLY.  The two curve legs are
 * antialiased with a one pixel wide ramp unless flags bit 0 is set.  The
 * chord cannot ramp (the curve unit treats every antialiased limiter as
 * another leg), so it is rasterised hard, rounded at the pixel centre;
 * bits 1..31 are reserved and must be zero.  The three points must
 * form a non-degenerate triangle that lies inside the buffer; offset must be
 * 8-byte aligned and pitch is in bytes.  format must not carry an alpha
 * channel (RGB565 or XRGB8888).  reserved must be zero.
 */
struct drm_n32h7xx_curve {
	__u32 handle;
	__u32 format;
	__u32 offset;
	__u32 pitch;
	__u32 color;
	__u32 flags;
	__u32 reserved;
	__s32 x0;
	__s32 y0;
	__s32 x1;
	__s32 y1;
	__s32 x2;
	__s32 y2;
};

#define DRM_IOCTL_N32H7XX_CURVE \
	DRM_IOW(DRM_COMMAND_BASE + DRM_N32H7XX_CURVE, struct drm_n32h7xx_curve)
#define DRM_IOCTL_N32H7XX_PROFILE \
	DRM_IOWR(DRM_COMMAND_BASE + DRM_N32H7XX_PROFILE, \
		 struct drm_n32h7xx_profile)

#endif
