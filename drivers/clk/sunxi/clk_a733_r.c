// SPDX-License-Identifier: (GPL-2.0+ OR MIT)
/*
 * Copyright (C) 2026 Alastair D'Silva <alastair@d-silva.org>
 */

#include <clk-uclass.h>
#include <dm.h>
#include <errno.h>
#include <clk/sunxi.h>
#include <linux/bitops.h>

#include <dt-bindings/clock/sun60i-a733-r-ccu.h>
#include <dt-bindings/reset/sun60i-a733-r-ccu.h>

static struct ccu_clk_gate a733_r_gates[] = {
	[CLK_R_AHB]		= GATE_DUMMY,
	[CLK_R_APBS0]		= GATE_DUMMY,
	[CLK_R_APBS1]		= GATE_DUMMY,

	[CLK_R_TWI0]		= GATE(0x19c, BIT(0)),
	[CLK_R_TWI1]		= GATE(0x19c, BIT(1)),
	[CLK_R_UART0]		= GATE(0x18c, BIT(0)),
	[CLK_R_SPI]		= GATE(0x1ac, BIT(0)),
};

static struct ccu_reset a733_r_resets[] = {
	[RST_BUS_R_TWI0]	= RESET(0x19c, BIT(16)),
	[RST_BUS_R_TWI1]	= RESET(0x19c, BIT(17)),
	[RST_BUS_R_UART0]	= RESET(0x18c, BIT(16)),
	[RST_BUS_R_SPI]		= RESET(0x1ac, BIT(16)),
};

const struct ccu_desc a733_r_ccu_desc = {
	.gates	= a733_r_gates,
	.resets	= a733_r_resets,
	.num_gates = ARRAY_SIZE(a733_r_gates),
	.num_resets = ARRAY_SIZE(a733_r_resets),
};
