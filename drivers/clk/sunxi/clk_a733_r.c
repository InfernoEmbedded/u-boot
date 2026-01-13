// SPDX-License-Identifier: (GPL-2.0+ OR MIT)
/*
 * Copyright (C) 2025-2026 Yixun Lan <dlan@gentoo.org>
 * Copyright (C) 2026 Alastair D'Silva <alastair@d-silva.org>
 */

#include <clk/sunxi.h>
#include <linux/bitops.h>
#include <linux/kernel.h>

#include <dt-bindings/clock/allwinner,sun60i-a733-r-ccu.h>
#include <dt-bindings/reset/allwinner,sun60i-a733-r-ccu.h>

static const struct ccu_clk_gate a733_r_gates[] = {
	[CLK_R_AHB]		= GATE_DUMMY,
	[CLK_R_APB0]		= GATE_DUMMY,
	[CLK_R_APB1]		= GATE_DUMMY,
	[CLK_R_SPI]		= GATE(0x150, BIT(31)),
	[CLK_BUS_R_SPI]		= GATE(0x15c, BIT(0)),
	[CLK_BUS_R_UART0]	= GATE(0x18c, BIT(0)),
	[CLK_BUS_R_UART1]	= GATE(0x18c, BIT(1)),
	[CLK_BUS_R_I2C0]	= GATE(0x19c, BIT(0)),
	[CLK_BUS_R_I2C1]	= GATE(0x19c, BIT(1)),
	[CLK_BUS_R_I2C2]	= GATE(0x19c, BIT(2)),
	[CLK_BUS_R_RTC]		= GATE(0x20c, BIT(0)),
	[CLK_BUS_R_CPUCFG]	= GATE(0x22c, BIT(0)),
};

static const struct ccu_reset a733_r_resets[] = {
	[RST_BUS_R_SPI]		= RESET(0x15c, BIT(16)),
	[RST_BUS_R_UART0]	= RESET(0x18c, BIT(16)),
	[RST_BUS_R_UART1]	= RESET(0x18c, BIT(17)),
	[RST_BUS_R_I2C0]	= RESET(0x19c, BIT(16)),
	[RST_BUS_R_I2C1]	= RESET(0x19c, BIT(17)),
	[RST_BUS_R_I2C2]	= RESET(0x19c, BIT(18)),
	[RST_BUS_R_RTC]		= RESET(0x20c, BIT(16)),
	[RST_BUS_R_CPUCFG]	= RESET(0x22c, BIT(16)),
};

const struct ccu_desc a733_r_ccu_desc = {
	.gates		= a733_r_gates,
	.resets		= a733_r_resets,
	.num_gates	= ARRAY_SIZE(a733_r_gates),
	.num_resets	= ARRAY_SIZE(a733_r_resets),
};
