// SPDX-License-Identifier: (GPL-2.0+ OR MIT)
/*
 * Copyright (C) 2026 Alastair D'Silva <alastair@d-silva.org>
 */

#include <clk-uclass.h>
#include <dm.h>
#include <errno.h>
#include <clk/sunxi.h>
#include <linux/bitops.h>

#include <dt-bindings/clock/sun60i-a733-ccu.h>
#include <dt-bindings/reset/sun60i-a733-ccu.h>

static struct ccu_clk_gate a733_gates[] = {
	[CLK_PLL_PERI0_200M]	= GATE_DUMMY,
	[CLK_APB1]		= GATE_DUMMY,
	[CLK_APB_UART]		= GATE_DUMMY,

	[CLK_SMHC0]		= GATE(0x0d00, BIT(31)),
	[CLK_BUS_SMHC0]		= GATE(0x0d0c, BIT(0)),
	[CLK_SMHC1]		= GATE(0x0d10, BIT(31)),
	[CLK_BUS_SMHC1]		= GATE(0x0d1c, BIT(0)),
	[CLK_SMHC2]		= GATE(0x0d20, BIT(31)),
	[CLK_BUS_SMHC2]		= GATE(0x0d2c, BIT(0)),

	[CLK_UART0]		= GATE(0x0e00, BIT(0)),
	[CLK_UART1]		= GATE(0x0e04, BIT(0)),
	[CLK_UART2]		= GATE(0x0e08, BIT(0)),
	[CLK_UART3]		= GATE(0x0e0c, BIT(0)),
	[CLK_UART4]		= GATE(0x0e10, BIT(0)),
	[CLK_UART5]		= GATE(0x0e14, BIT(0)),
	[CLK_UART6]		= GATE(0x0e18, BIT(0)),

	[CLK_TWI0]		= GATE(0x0e80, BIT(0)),
	[CLK_TWI1]		= GATE(0x0e84, BIT(0)),
	[CLK_TWI2]		= GATE(0x0e88, BIT(0)),
	[CLK_TWI3]		= GATE(0x0e8c, BIT(0)),

	[CLK_SPI0]		= GATE(0x0f00, BIT(31)),
	[CLK_BUS_SPI0]		= GATE(0x0f04, BIT(0)),
	[CLK_SPI1]		= GATE(0x0f08, BIT(31)),
	[CLK_BUS_SPI1]		= GATE(0x0f0c, BIT(0)),

	[CLK_GMAC_PTP]		= GATE(0x1400, BIT(31)),
	[CLK_GMAC0_PHY]		= GATE(0x1410, BIT(31)),
	[CLK_GMAC0]		= GATE(0x141c, BIT(0) | BIT(1)),
	[CLK_GMAC1_PHY]		= GATE(0x1420, BIT(31)),
	[CLK_GMAC1]		= GATE(0x142c, BIT(0) | BIT(1)),

	[CLK_USB0_OHCI]		= GATE(0x1370, BIT(31)),
	[CLK_USB1_OHCI]		= GATE(0x1374, BIT(31)),
	[CLK_USB0_EHCI]		= GATE(0x138c, BIT(4)),
	[CLK_USB1_EHCI]		= GATE(0x138c, BIT(5)),
	[CLK_USB]		= GATE(0x138c, BIT(8)),
};

static struct ccu_reset a733_resets[] = {
	[RST_BUS_SMHC0]		= RESET(0x0d0c, BIT(16)),
	[RST_BUS_SMHC1]		= RESET(0x0d1c, BIT(16)),
	[RST_BUS_SMHC2]		= RESET(0x0d2c, BIT(16)),

	[RST_BUS_UART0]		= RESET(0x0e00, BIT(16)),
	[RST_BUS_UART1]		= RESET(0x0e04, BIT(16)),
	[RST_BUS_UART2]		= RESET(0x0e08, BIT(16)),
	[RST_BUS_UART3]		= RESET(0x0e0c, BIT(16)),
	[RST_BUS_UART4]		= RESET(0x0e10, BIT(16)),
	[RST_BUS_UART5]		= RESET(0x0e14, BIT(16)),
	[RST_BUS_UART6]		= RESET(0x0e18, BIT(16)),

	[RST_BUS_TWI0]		= RESET(0x0e80, BIT(16)),
	[RST_BUS_TWI1]		= RESET(0x0e84, BIT(16)),
	[RST_BUS_TWI2]		= RESET(0x0e88, BIT(16)),
	[RST_BUS_TWI3]		= RESET(0x0e8c, BIT(16)),

	[RST_BUS_SPI0]		= RESET(0x0f04, BIT(16)),
	[RST_BUS_SPI1]		= RESET(0x0f0c, BIT(16)),

	[RST_BUS_GMAC0]		= RESET(0x141c, BIT(16)),
	[RST_BUS_GMAC1]		= RESET(0x142c, BIT(16)),

	[RST_USB_0_OHCI]	= RESET(0x138c, BIT(16)),
	[RST_USB_1_OHCI]	= RESET(0x138c, BIT(17)),
	[RST_USB_0_EHCI]	= RESET(0x138c, BIT(20)),
	[RST_USB_1_EHCI]	= RESET(0x138c, BIT(21)),
	[RST_USB_0_DEVICE]	= RESET(0x138c, BIT(24)),
};

const struct ccu_desc a733_ccu_desc = {
	.gates	= a733_gates,
	.resets	= a733_resets,
	.num_gates = ARRAY_SIZE(a733_gates),
	.num_resets = ARRAY_SIZE(a733_resets),
};
