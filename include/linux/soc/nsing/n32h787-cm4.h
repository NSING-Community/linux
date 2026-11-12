/* SPDX-License-Identifier: GPL-2.0-only OR BSD-3-Clause */
#ifndef __LINUX_SOC_NSING_N32H787_CM4_H
#define __LINUX_SOC_NSING_N32H787_CM4_H

#ifdef __KERNEL__
typedef u8 n32h787_cm4_u8;
typedef u16 n32h787_cm4_u16;
typedef u32 n32h787_cm4_u32;
#else
#include <stdint.h>
typedef uint8_t n32h787_cm4_u8;
typedef uint16_t n32h787_cm4_u16;
typedef uint32_t n32h787_cm4_u32;
#endif

#define N32H787_CM4_ABI_MAGIC		0x4e333243U /* "N32C" */
#define N32H787_CM4_ABI_VERSION		1U

#define N32H787_CM4_SRAM_BASE		0x30000000U
#define N32H787_CM4_VECTOR_BASE		0x30000000U

/*
 * The first 0x320 bytes contain the architectural vector table.  Keep the
 * platform ABI after it and reserve through 0x800 for non-cache-coherent IPC.
 * Every mutable producer/consumer counter has its own 32-byte line.
 */
#define N32H787_CM4_ABI_BASE		0x30000320U
#define N32H787_CM4_ABI_END		0x30000800U
#define N32H787_CM4_ABI_SIZE		(N32H787_CM4_ABI_END - \
					 N32H787_CM4_ABI_BASE)

#define N32H787_CM4_H2C_HEAD		0x30000340U
#define N32H787_CM4_H2C_DROPPED		0x30000344U
#define N32H787_CM4_H2C_TAIL		0x30000360U
#define N32H787_CM4_C2H_HEAD		0x30000380U
#define N32H787_CM4_C2H_DROPPED		0x30000384U
#define N32H787_CM4_C2H_TAIL		0x300003a0U
#define N32H787_CM4_H2C_DATA		0x30000400U
#define N32H787_CM4_C2H_DATA		0x30000600U

#define N32H787_CM4_RING_SLOTS		8U
#define N32H787_CM4_RING_MASK		(N32H787_CM4_RING_SLOTS - 1U)
#define N32H787_CM4_MESSAGE_SIZE	64U
#define N32H787_CM4_MESSAGE_PAYLOAD	52U

#define N32H787_CM4_DOORBELL		0x434d3452U
#define N32H787_CM4_MSG_REPLY		0x8000U

enum n32h787_cm4_type {
	N32H787_CM4_MSG_USER = 1,
};

enum n32h787_cm4_state {
	N32H787_CM4_STATE_OFFLINE = 0,
	N32H787_CM4_STATE_READY = 1,
	N32H787_CM4_STATE_RUNNING = 2,
	N32H787_CM4_STATE_EARLY_BOOT = 3,
};

struct n32h787_cm4_control {
	n32h787_cm4_u32 magic;
	n32h787_cm4_u32 version;
	n32h787_cm4_u32 flags;
	n32h787_cm4_u32 size;
	n32h787_cm4_u32 heartbeat;
	n32h787_cm4_u32 state;
	n32h787_cm4_u32 error;
	n32h787_cm4_u32 reserved;
};

struct n32h787_cm4_message {
	n32h787_cm4_u32 sequence;
	n32h787_cm4_u16 type;
	n32h787_cm4_u16 flags;
	n32h787_cm4_u16 payload_len;
	n32h787_cm4_u16 reserved;
	n32h787_cm4_u8 payload[N32H787_CM4_MESSAGE_PAYLOAD];
};

#endif /* __LINUX_SOC_NSING_N32H787_CM4_H */
