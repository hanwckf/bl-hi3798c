/*
 * (C) Copyright 2017 Linaro
 * Jorge Ramirez-Ortiz <jorge.ramirez-ortiz@linaro.org>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __HIGMAC_UTIL_H__
#define __HIGMAC_UTIL_H__

#include "higmac.h"

#define HIGMAC_TRACE_LEVEL 8

#define higmac_trace(level, msg...) do { \
	if ((level) >= HIGMAC_TRACE_LEVEL) { \
	    printf("higmac_trace:%s:%d: ", __FILE__, __LINE__); \
	    printf(msg); \
	    printf("\n"); \
	} \
    } while (0)

#define higmac_error(args...) do { \
	printf("higmac:%s:%d: ", __FILE__, __LINE__); \
	printf(args); \
	printf("\n"); \
    } while (0)

#define higmac_assert(cond) do { \
	if (!(cond)) \
	    printf("Assert:higmac:%s:%d\n", __FILE__, __LINE__);\
    } while (0)

static inline u32
higmac_readl(struct higmac_netdev_local *ld, u32 ofs)
{
	u32 value = readl(ld->iobase + ofs);

	higmac_trace(2, "readl(0x%04X) = 0x%08X", ofs, value);

	return value;
}

static inline void
higmac_writel(struct higmac_netdev_local *ld, u32 value, u32 ofs)
{
	writel(value, ld->iobase + ofs);
	higmac_trace(2, "writel(0x%04X) = 0x%08X", ofs, value);
}

/*
 * Define a bit mask by storing its offset (shift) in the upper
 * portion and width (nbits) in its lower portion of a 32-bit
 * "descriptor".
 */
#define MK_BITS(shift, nbits) ((((shift) & 0x1F) << 16) | ((nbits) & 0x3F))

static inline u32
higmac_readl_bits(struct higmac_netdev_local *ld, u32 ofs, u32 desc)
{
	u32 bits = desc & 0x3f;
	u32 shift = desc >> 16;
	u32 mask = (bits < 32 ? ((1 << bits) - 1) : 0xffffffff) << shift;
	u32 val = higmac_readl(ld, ofs);

	return (val & mask) >> shift;
}

static inline void
higmac_writel_bits(struct higmac_netdev_local *ld, u32 value, u32 ofs, u32 desc)
{
	u32 bits = desc & 0x3f;
	u32 shift = desc >> 16;
	u32 mask = (bits < 32 ? ((1 << bits) - 1) : 0xffffffff) << shift;
	u32 update = (value << shift) & mask;
	u32 val;

	/* Read the old value, update it, and write it back */
	val = higmac_readl(ld, ofs);
	val = (val & ~mask) | update;
	higmac_writel(ld, val, ofs);
}

#endif
