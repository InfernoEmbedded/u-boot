/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Allwinner A733 DRAM Controller definitions
 *
 * (C) Copyright 2026 Alastair D'Silva <alastair@d-silva.org>
 */

#ifndef _SUNXI_DRAM_SUN60I_A733_H
#define _SUNXI_DRAM_SUN60I_A733_H

#include <linux/bitops.h>

/**
 * enum sunxi_dram_type - Allwinner A733 supported DRAM device types
 * @SUNXI_DRAM_TYPE_LPDDR5: Low-Power DDR5 SDRAM (type code 9)
 */
enum sunxi_dram_type {
	SUNXI_DRAM_TYPE_LPDDR5 = 9,
};

/**
 * sunxi_dram_init() - Initialize the Allwinner A733 DRAM subsystem
 *
 * Configures the AXP8191 PMIC power rails, unlocks system bus and security
 * gates, brings up the dual-channel uMCTL2 controllers and Cadence DDR PHY,
 * executes the ARC EM4 PMU training firmware, and verifies memory integrity.
 *
 * Return: Total detected DRAM capacity in bytes, or 0 on failure.
 */
unsigned long sunxi_dram_init(void);

#endif /* _SUNXI_DRAM_SUN60I_A733_H */
