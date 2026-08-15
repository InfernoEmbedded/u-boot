// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2026 Alastair D'Silva <alastair@d-silva.org>
 */

#include <asm/arch/cpu.h>
#include <asm/cache.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <clk.h>
#include <dm.h>
#include <dm/device_compat.h>
#include <dm/pinctrl.h>
#include <errno.h>
#include <eth_phy.h>
#include <linux/delay.h>
#include <malloc.h>
#include <net.h>
#include <reset.h>
#include <syscon.h>

#include "dwc_eth_qos.h"

#define SUNXI_DWMAC210_SYSCON		0x00
#define SUNXI_DWMAC210_CLK_GATE		0x80

#define SUNXI_DWMAC210_MODE_MII		0x0
#define SUNXI_DWMAC210_MODE_RGMII	0x4
#define SUNXI_DWMAC210_MODE_RMII	0x8

#define SUNXI_DWMAC210_ETXDC		GENMASK(12, 10)
#define SUNXI_DWMAC210_ERXDC		GENMASK(9, 5)

#define SUNXI_DWMAC210_TX_CLK_SRC_MII	BIT(9)
#define SUNXI_DWMAC210_TX_CLK_SRC_EXT	BIT(8)
#define SUNXI_DWMAC210_TX_CLK_SRC_INT	0x0

#define SUNXI_DWMAC210_ETCS_INT_GMII	0x2

struct sunxi_platform_data {
	void __iomem *syscon_regs;
	struct reset_ctl_bulk resets;
	struct clk_bulk clks;
	phy_interface_t interface;
};

static phy_interface_t eqos_get_interface_sunxi(const struct udevice *dev)
{
	return dev_read_phy_mode(dev);
}

static int eqos_probe_resources_sunxi(struct udevice *dev)
{
	struct eth_pdata *pdata = dev_get_plat(dev);
	struct eqos_priv *eqos = dev_get_priv(dev);
	struct sunxi_platform_data *data;
	fdt_addr_t syscon_addr;
	int ret;

	data = calloc(1, sizeof(*data));
	if (!data)
		return -ENOMEM;

	pdata->priv_pdata = data;
	data->interface = dev_read_phy_mode(dev);

	ret = eqos_get_base_addr_dt(dev);
	if (ret)
		return ret;

	syscon_addr = dev_read_addr_index(dev, 1);
	if (syscon_addr != FDT_ADDR_T_NONE) {
		data->syscon_regs = (void __iomem *)syscon_addr;
	} else {
		data->syscon_regs = (void __iomem *)SUNXI_GMAC0_BASE + 0x8000;
	}

	ret = reset_get_bulk(dev, &data->resets);
	if (ret)
		dev_warn(dev, "Failed to get resets: %d\n", ret);

	ret = clk_get_bulk(dev, &data->clks);
	if (ret)
		dev_warn(dev, "Failed to get clocks: %d\n", ret);

	pinctrl_select_state(dev, "default");

	/* Configure Port H (PH0..PH15) pinmux to function 5 (GMAC0) and pull-up */
	writel(0x55555555, (void __iomem *)(SUNXI_PIO_BASE + 0x400));
	writel(0x55555555, (void __iomem *)(SUNXI_PIO_BASE + 0x404));
	writel(0x55555555, (void __iomem *)(SUNXI_PIO_BASE + 0x430));

	/* Pulse PHY reset on PH16 */
	clrsetbits_le32((void __iomem *)(SUNXI_PIO_BASE + 0x408), 0xf, 0x1);
	clrbits_le32((void __iomem *)(SUNXI_PIO_BASE + 0x410), BIT(16));
	mdelay(20);
	setbits_le32((void __iomem *)(SUNXI_PIO_BASE + 0x410), BIT(16));
	mdelay(50);

	gpio_request_by_name(dev, "snps,reset-gpio", 0,
			     &eqos->phy_reset_gpio, GPIOD_IS_OUT);

	return 0;
}

static int eqos_remove_resources_sunxi(struct udevice *dev)
{
	struct eth_pdata *pdata = dev_get_plat(dev);
	struct sunxi_platform_data *data = pdata->priv_pdata;

	reset_assert_bulk(&data->resets);
	clk_disable_bulk(&data->clks);
	free(data);

	return 0;
}

static int eqos_start_resets_sunxi(struct udevice *dev)
{
	struct eth_pdata *pdata = dev_get_plat(dev);
	struct sunxi_platform_data *data = pdata->priv_pdata;

	return reset_deassert_bulk(&data->resets);
}

static int eqos_stop_resets_sunxi(struct udevice *dev)
{
	struct eth_pdata *pdata = dev_get_plat(dev);
	struct sunxi_platform_data *data = pdata->priv_pdata;

	return reset_assert_bulk(&data->resets);
}

static int eqos_start_clks_sunxi(struct udevice *dev)
{
	struct eth_pdata *pdata = dev_get_plat(dev);
	struct sunxi_platform_data *data = pdata->priv_pdata;
	int ret;

	ret = clk_enable_bulk(&data->clks);
	if (ret)
		return ret;

	/* Enable GMAC0 MBUS clock gate (CCU offset 0x05E4, bit 11) for DMA */
	setbits_le32((void __iomem *)(SUNXI_CCM_BASE + 0x05E4), BIT(11));

	if (data->syscon_regs) {
		u32 val = 0;

		switch (data->interface) {
		case PHY_INTERFACE_MODE_RGMII:
		case PHY_INTERFACE_MODE_RGMII_ID:
		case PHY_INTERFACE_MODE_RGMII_RXID:
		case PHY_INTERFACE_MODE_RGMII_TXID:
			val = (3 << 10) | (10 << 5) | SUNXI_DWMAC210_MODE_RGMII | SUNXI_DWMAC210_ETCS_INT_GMII;
			break;
		case PHY_INTERFACE_MODE_RMII:
			val = SUNXI_DWMAC210_MODE_RMII;
			break;
		case PHY_INTERFACE_MODE_MII:
		default:
			val = SUNXI_DWMAC210_MODE_MII;
			break;
		}

		writel(val, data->syscon_regs + SUNXI_DWMAC210_SYSCON);
		/* Active-low clock gate */
		writel(0x0, data->syscon_regs + SUNXI_DWMAC210_CLK_GATE);
	}

	return 0;
}

static int eqos_stop_clks_sunxi(struct udevice *dev)
{
	struct eth_pdata *pdata = dev_get_plat(dev);
	struct sunxi_platform_data *data = pdata->priv_pdata;

	if (data->syscon_regs)
		writel(0x1, data->syscon_regs + SUNXI_DWMAC210_CLK_GATE);

	return clk_disable_bulk(&data->clks);
}

static int eqos_set_tx_clk_speed_sunxi(struct udevice *dev)
{
	return 0;
}

static ulong eqos_get_tick_clk_rate_sunxi(struct udevice *dev)
{
	return 200000000;
}

static struct eqos_ops eqos_sunxi_ops = {
	.eqos_inval_desc = eqos_inval_desc_generic,
	.eqos_flush_desc = eqos_flush_desc_generic,
	.eqos_inval_buffer = eqos_inval_buffer_generic,
	.eqos_flush_buffer = eqos_flush_buffer_generic,
	.eqos_probe_resources = eqos_probe_resources_sunxi,
	.eqos_remove_resources = eqos_remove_resources_sunxi,
	.eqos_stop_resets = eqos_stop_resets_sunxi,
	.eqos_start_resets = eqos_start_resets_sunxi,
	.eqos_stop_clks = eqos_stop_clks_sunxi,
	.eqos_start_clks = eqos_start_clks_sunxi,
	.eqos_calibrate_pads = eqos_null_ops,
	.eqos_disable_calibration = eqos_null_ops,
	.eqos_set_tx_clk_speed = eqos_set_tx_clk_speed_sunxi,
	.eqos_get_enetaddr = eqos_null_ops,
	.eqos_get_tick_clk_rate = eqos_get_tick_clk_rate_sunxi,
};

struct eqos_config eqos_sunxi_config = {
	.reg_access_always_ok = false,
	.mdio_wait = 10,
	.swr_wait = 50,
	.config_mac = EQOS_MAC_RXQ_CTRL0_RXQ0EN_ENABLED_DCB,
	.config_mac_mdio = EQOS_MAC_MDIO_ADDRESS_CR_100_150,
	.axi_bus_width = EQOS_AXI_WIDTH_64,
	.interface = eqos_get_interface_sunxi,
	.ops = &eqos_sunxi_ops
};
