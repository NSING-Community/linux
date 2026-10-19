/* SPDX-License-Identifier: GPL-2.0-only WITH Linux-syscall-note */
#ifndef _UAPI_LINUX_N32JPEG_H
#define _UAPI_LINUX_N32JPEG_H

#include <linux/ioctl.h>
#include <linux/types.h>

#define N32JPEG_ABI_VERSION	1
#define N32JPEG_MAX_DIMENSION	65535U

enum n32jpeg_format {
	N32JPEG_FORMAT_GREY8 = 1,
	/* Reserved for the follow-up hardware format work. */
	N32JPEG_FORMAT_RGB24 = 2,
	N32JPEG_FORMAT_YUV422 = 3,
};

struct n32jpeg_query {
	__u32 abi_version;
	__u32 formats;		/* Bitmask of enum n32jpeg_format. */
	__u32 min_width;
	__u32 min_height;
	__u32 max_width;
	__u32 max_height;
	__u32 input_alignment;
	__u32 output_alignment;
	__u32 flags;
	__u32 reserved[7];
};

struct n32jpeg_encode {
	__u32 struct_size;
	__u32 flags;
	__u32 format;
	__u32 width;
	__u32 height;
	__u32 input_stride;
	__u32 input_size;
	__u32 quality;		/* 1..100 */
	__aligned_u64 input;
	__aligned_u64 output;
	__u32 output_capacity;
	__u32 output_size;	/* Set by the driver. */
	__u32 reserved[6];
};

#define N32JPEG_IOC_MAGIC	'J'

#define N32JPEG_QUERY		_IOR(N32JPEG_IOC_MAGIC, 0x00, \
				     struct n32jpeg_query)
#define N32JPEG_ENCODE		_IOWR(N32JPEG_IOC_MAGIC, 0x01, \
				     struct n32jpeg_encode)

#endif /* _UAPI_LINUX_N32JPEG_H */
