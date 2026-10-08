/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Allwinner A733 DRAM Controller definitions
 *
 * (C) Copyright 2026 Alastair D'Silva <alastair@d-silva.org>
 */

#ifndef _SUNXI_DRAM_SUN60I_A733_H
#define _SUNXI_DRAM_SUN60I_A733_H

#include <linux/bitops.h>

enum sunxi_dram_type {
	SUNXI_DRAM_TYPE_LPDDR4 = 8,
	SUNXI_DRAM_TYPE_LPDDR5 = 9,
};

#endif /* _SUNXI_DRAM_SUN60I_A733_H */
