// SPDX-License-Identifier: (GPL-2.0+ OR MIT)
/*
 * Copyright (C) 2025-2026 Yixun Lan <dlan@gentoo.org>
 * Copyright (C) 2026 Alastair D'Silva <alastair@d-silva.org>
 */

#include <clk/sunxi.h>
#include <linux/bitops.h>
#include <linux/kernel.h>

#include <dt-bindings/clock/allwinner,sun60i-a733-ccu.h>
#include <dt-bindings/reset/allwinner,sun60i-a733-ccu.h>

static const struct ccu_clk_gate a733_gates[] = {
	[CLK_SYS_24M]		= GATE_DUMMY,
	[CLK_PLL_PERIPH0_200M]	= GATE_DUMMY,
	[CLK_PLL_PERIPH0_300M]	= GATE_DUMMY,
	[CLK_PLL_PERIPH1_300M]	= GATE_DUMMY,
	[CLK_APB1]		= GATE_DUMMY,
	[CLK_APB_UART]		= GATE_DUMMY,

	[CLK_MBUS_GMAC0]	= GATE(0x05e4, BIT(11)),
	[CLK_MBUS_SERDES]	= GATE(0x0584, BIT(0)),
	[CLK_BUS_CE]		= GATE(0x0ac4, BIT(0)),

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

	[CLK_BUS_I2C0]		= GATE(0x0e80, BIT(0)),
	[CLK_BUS_I2C1]		= GATE(0x0e84, BIT(0)),
	[CLK_BUS_I2C2]		= GATE(0x0e88, BIT(0)),
	[CLK_BUS_I2C3]		= GATE(0x0e8c, BIT(0)),
	[CLK_BUS_I2C4]		= GATE(0x0e90, BIT(0)),
	[CLK_BUS_I2C5]		= GATE(0x0e94, BIT(0)),
	[CLK_BUS_I2C6]		= GATE(0x0e98, BIT(0)),
	[CLK_BUS_I2C7]		= GATE(0x0e9c, BIT(0)),
	[CLK_BUS_I2C8]		= GATE(0x0ea0, BIT(0)),
	[CLK_BUS_I2C9]		= GATE(0x0ea4, BIT(0)),
	[CLK_BUS_I2C10]		= GATE(0x0ea8, BIT(0)),
	[CLK_BUS_I2C11]		= GATE(0x0eac, BIT(0)),
	[CLK_BUS_I2C12]		= GATE(0x0eb0, BIT(0)),

	[CLK_SPI0]		= GATE(0x0f00, BIT(31)),
	[CLK_BUS_SPI0]		= GATE(0x0f04, BIT(0)),
	[CLK_SPI1]		= GATE(0x0f08, BIT(31)),
	[CLK_BUS_SPI1]		= GATE(0x0f0c, BIT(0)),
	[CLK_SPI2]		= GATE(0x0f10, BIT(31)),
	[CLK_BUS_SPI2]		= GATE(0x0f14, BIT(0)),

	[CLK_GPADC0_24M]	= GATE(0x0fc0, BIT(31)),
	[CLK_BUS_GPADC0]	= GATE(0x0fc4, BIT(0)),
	[CLK_BUS_THS0]		= GATE(0x0fe4, BIT(0)),
	[CLK_BUS_LRADC]		= GATE(0x1024, BIT(0)),

	[CLK_USB_OHCI0]		= GATE(0x1300, BIT(31)),
	[CLK_BUS_OHCI0]		= GATE(0x1304, BIT(0)),
	[CLK_BUS_EHCI0]		= GATE(0x1304, BIT(4)),
	[CLK_BUS_OTG]		= GATE(0x1304, BIT(8)),
	[CLK_USB_OHCI1]		= GATE(0x1308, BIT(31)),
	[CLK_BUS_OHCI1]		= GATE(0x130c, BIT(0)),
	[CLK_BUS_EHCI1]		= GATE(0x130c, BIT(4)),

	[CLK_PCIE_AUX]		= GATE(0x1380, BIT(31)),
	[CLK_PCIE_AXI_SLV]	= GATE(0x1384, BIT(31)),
	[CLK_SERDES_PHY]	= GATE(0x13c0, BIT(31)),
	[CLK_AHB_SERDES]	= GATE(0x13c4, BIT(0)),

	[CLK_GMAC0_PHY]		= GATE(0x1410, BIT(31)),
	[CLK_BUS_GMAC0]		= GATE(0x141c, BIT(0)),
};

static const struct ccu_reset a733_resets[] = {
	[RST_BUS_CE]		= RESET(0x0ac4, BIT(16)),

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

	[RST_BUS_I2C0]		= RESET(0x0e80, BIT(16)),
	[RST_BUS_I2C1]		= RESET(0x0e84, BIT(16)),
	[RST_BUS_I2C2]		= RESET(0x0e88, BIT(16)),
	[RST_BUS_I2C3]		= RESET(0x0e8c, BIT(16)),
	[RST_BUS_I2C4]		= RESET(0x0e90, BIT(16)),
	[RST_BUS_I2C5]		= RESET(0x0e94, BIT(16)),
	[RST_BUS_I2C6]		= RESET(0x0e98, BIT(16)),
	[RST_BUS_I2C7]		= RESET(0x0e9c, BIT(16)),
	[RST_BUS_I2C8]		= RESET(0x0ea0, BIT(16)),
	[RST_BUS_I2C9]		= RESET(0x0ea4, BIT(16)),
	[RST_BUS_I2C10]		= RESET(0x0ea8, BIT(16)),
	[RST_BUS_I2C11]		= RESET(0x0eac, BIT(16)),
	[RST_BUS_I2C12]		= RESET(0x0eb0, BIT(16)),

	[RST_BUS_SPI0]		= RESET(0x0f04, BIT(16)),
	[RST_BUS_SPI1]		= RESET(0x0f0c, BIT(16)),
	[RST_BUS_SPI2]		= RESET(0x0f14, BIT(16)),

	[RST_BUS_GPADC0]	= RESET(0x0fc4, BIT(16)),
	[RST_BUS_THS0]		= RESET(0x0fe4, BIT(16)),
	[RST_BUS_LRADC]		= RESET(0x1024, BIT(16)),

	[RST_USB_PHY0]		= RESET(0x1300, BIT(30)),
	[RST_BUS_OHCI0]		= RESET(0x1304, BIT(16)),
	[RST_BUS_EHCI0]		= RESET(0x1304, BIT(20)),
	[RST_BUS_OTG]		= RESET(0x1304, BIT(24)),
	[RST_USB_PHY1]		= RESET(0x1308, BIT(30)),
	[RST_BUS_OHCI1]		= RESET(0x130c, BIT(16)),
	[RST_BUS_EHCI1]		= RESET(0x130c, BIT(20)),

	[RST_BUS_PCIE]		= RESET(0x138c, BIT(16)),
	[RST_BUS_PCIE_PWRUP]	= RESET(0x138c, BIT(17)),
	[RST_BUS_SERDES]	= RESET(0x13c4, BIT(16)),

	[RST_BUS_GMAC0]		= RESET(0x141c, BIT(16)),
	[RST_BUS_GMAC0_AXI]	= RESET(0x141c, BIT(17)),
};

const struct ccu_desc a733_ccu_desc = {
	.gates		= a733_gates,
	.resets		= a733_resets,
	.num_gates	= ARRAY_SIZE(a733_gates),
	.num_resets	= ARRAY_SIZE(a733_resets),
};
