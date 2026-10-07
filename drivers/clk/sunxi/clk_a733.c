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
	[CLK_PLL_PERIPH0_200M]	= GATE_DUMMY,
	[CLK_APB1]		= GATE_DUMMY,

	[CLK_MMC0]		= GATE(0x0d00, BIT(31)),
	[CLK_BUS_MMC0]		= GATE(0x0d0c, BIT(0)),
	[CLK_MMC1]		= GATE(0x0d10, BIT(31)),
	[CLK_BUS_MMC1]		= GATE(0x0d1c, BIT(0)),
	[CLK_MMC2]		= GATE(0x0d20, BIT(31)),
	[CLK_BUS_MMC2]		= GATE(0x0d2c, BIT(0)),

	[CLK_BUS_UART0]		= GATE(0x0e00, BIT(0)),
	[CLK_BUS_UART1]		= GATE(0x0e04, BIT(0)),
	[CLK_BUS_UART2]		= GATE(0x0e08, BIT(0)),
	[CLK_BUS_UART3]		= GATE(0x0e0c, BIT(0)),
	[CLK_BUS_UART4]		= GATE(0x0e10, BIT(0)),
	[CLK_BUS_UART5]		= GATE(0x0e14, BIT(0)),
	[CLK_BUS_UART6]		= GATE(0x0e18, BIT(0)),
	[CLK_BUS_UART7]		= GATE(0x0e1c, BIT(0)),

	[CLK_BUS_I2C0]		= GATE(0x1e80, BIT(0)),
	[CLK_BUS_I2C1]		= GATE(0x1e84, BIT(0)),
	[CLK_BUS_I2C2]		= GATE(0x1e88, BIT(0)),
	[CLK_BUS_I2C3]		= GATE(0x1e8c, BIT(0)),

	[CLK_SPI0]		= GATE(0x0f00, BIT(31)),
	[CLK_BUS_SPI0]		= GATE(0x0f04, BIT(0)),
	[CLK_SPI1]		= GATE(0x0f08, BIT(31)),
	[CLK_BUS_SPI1]		= GATE(0x0f0c, BIT(0)),
	[CLK_SPI2]		= GATE(0x0f10, BIT(31)),
	[CLK_BUS_SPI2]		= GATE(0x0f14, BIT(0)),

	[CLK_EMAC0_25M]		= GATE(0x0970, BIT(31) | BIT(30)),
	[CLK_BUS_EMAC0]		= GATE(0x097c, BIT(0)),
	[CLK_EMAC1_25M]		= GATE(0x0974, BIT(31) | BIT(30)),
	[CLK_BUS_EMAC1]		= GATE(0x098c, BIT(0)),

	[CLK_USB_OHCI0]		= GATE(0x2300, BIT(31)),
	[CLK_USB_OHCI1]		= GATE(0x2308, BIT(31)),
	[CLK_BUS_OHCI0]		= GATE(0x2304, BIT(0)),
	[CLK_BUS_OHCI1]		= GATE(0x230c, BIT(0)),
	[CLK_BUS_EHCI0]		= GATE(0x2304, BIT(4)),
	[CLK_BUS_EHCI1]		= GATE(0x230c, BIT(4)),
	[CLK_BUS_OTG]		= GATE(0x2304, BIT(8)),

	[CLK_GPADC0]		= GATE(0x0fc0, BIT(31)),
	[CLK_BUS_GPADC0]	= GATE(0x0fc4, BIT(0)),
	[CLK_BUS_THS]		= GATE(0x0fe4, BIT(0)),
	[CLK_BUS_LRADC]		= GATE(0x1024, BIT(0)),
	[CLK_BUS_CE]		= GATE(0x0ac4, BIT(0)),
};

static struct ccu_reset a733_resets[] = {
	[RST_BUS_MMC0]		= RESET(0x0d0c, BIT(16)),
	[RST_BUS_MMC1]		= RESET(0x0d1c, BIT(16)),
	[RST_BUS_MMC2]		= RESET(0x0d2c, BIT(16)),

	[RST_BUS_UART0]		= RESET(0x0e00, BIT(16)),
	[RST_BUS_UART1]		= RESET(0x0e04, BIT(16)),
	[RST_BUS_UART2]		= RESET(0x0e08, BIT(16)),
	[RST_BUS_UART3]		= RESET(0x0e0c, BIT(16)),
	[RST_BUS_UART4]		= RESET(0x0e10, BIT(16)),
	[RST_BUS_UART5]		= RESET(0x0e14, BIT(16)),
	[RST_BUS_UART6]		= RESET(0x0e18, BIT(16)),
	[RST_BUS_UART7]		= RESET(0x0e1c, BIT(16)),

	[RST_BUS_I2C0]		= RESET(0x1e80, BIT(16)),
	[RST_BUS_I2C1]		= RESET(0x1e84, BIT(16)),
	[RST_BUS_I2C2]		= RESET(0x1e88, BIT(16)),
	[RST_BUS_I2C3]		= RESET(0x1e8c, BIT(16)),

	[RST_BUS_SPI0]		= RESET(0x0f04, BIT(16)),
	[RST_BUS_SPI1]		= RESET(0x0f0c, BIT(16)),
	[RST_BUS_SPI2]		= RESET(0x0f14, BIT(16)),

	[RST_BUS_EMAC0]		= RESET(0x097c, BIT(16)),
	[RST_BUS_EMAC1]		= RESET(0x098c, BIT(16) | BIT(17)),

	[RST_USB_PHY0]		= RESET(0x2300, BIT(30)),
	[RST_USB_PHY1]		= RESET(0x2308, BIT(30)),
	[RST_BUS_OHCI0]		= RESET(0x2304, BIT(16)),
	[RST_BUS_OHCI1]		= RESET(0x230c, BIT(16)),
	[RST_BUS_EHCI0]		= RESET(0x2304, BIT(20)),
	[RST_BUS_EHCI1]		= RESET(0x230c, BIT(20)),
	[RST_BUS_OTG]		= RESET(0x2304, BIT(24)),

	[RST_BUS_GPADC0]	= RESET(0x0fc4, BIT(16)),
	[RST_BUS_THS]		= RESET(0x0fe4, BIT(16)),
	[RST_BUS_LRADC]		= RESET(0x1024, BIT(16)),
	[RST_BUS_CE]		= RESET(0x0ac4, BIT(16)),
};

const struct ccu_desc a733_ccu_desc = {
	.gates	= a733_gates,
	.resets	= a733_resets,
	.num_gates = ARRAY_SIZE(a733_gates),
	.num_resets = ARRAY_SIZE(a733_resets),
};
