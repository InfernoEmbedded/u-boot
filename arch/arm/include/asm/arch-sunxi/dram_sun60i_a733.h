/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Allwinner A733 DRAM Controller definitions
 *
 * (C) Copyright 2026 Alastair D'Silva <alastair@d-silva.org>
 */

#ifndef _SUNXI_DRAM_SUN60I_A733_H
#define _SUNXI_DRAM_SUN60I_A733_H

#include <linux/bitops.h>

#include <linux/types.h>

enum sunxi_dram_type {
	SUNXI_DRAM_TYPE_DDR3 = 3,
	SUNXI_DRAM_TYPE_DDR4 = 4,
	SUNXI_DRAM_TYPE_LPDDR3 = 7,
	SUNXI_DRAM_TYPE_LPDDR4 = 8,
	SUNXI_DRAM_TYPE_LPDDR5 = 9,
};

/**
 * struct dram_geometry - A733 DRAM controller configuration parameters and geometry
 * @clk: Memory clock frequency in MHz (e.g. 1800 for LPDDR5-3600)
 * @type: DRAM memory type (e.g. SUNXI_DRAM_TYPE_LPDDR5)
 * @cols: Column address bit width (e.g. 10)
 * @rows: Row address bit width (e.g. 16)
 * @banks: Bank address bit width (e.g. 2 for 4 banks)
 * @bank_groups: Bank group bit width (e.g. 2 for 4 bank groups)
 * @ranks: Number of ranks per channel (1 or 2)
 * @channels: Number of active channels (1 or 2)
 * @density_3_4: true if board uses 3/4 density capacity scaling (e.g. 6GB = 3/4 * 8GB)
 */
struct dram_geometry {
	u32 clk;
	enum sunxi_dram_type type;
	u8 cols;
	u8 rows;
	u8 banks;
	u8 bank_groups;
	u8 ranks;
	u8 channels;
	bool density_3_4;
};

#endif /* _SUNXI_DRAM_SUN60I_A733_H */
