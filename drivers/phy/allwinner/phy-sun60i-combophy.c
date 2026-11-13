// SPDX-License-Identifier: GPL-2.0+
/*
 * Allwinner A733 (sun60i) Cadence Combo PHY driver
 *
 * Copyright (c) 2026 Armbian Project
 */

#include <clk.h>
#include <dm.h>
#include <dm/device_compat.h>
#include <dt-bindings/phy/phy.h>
#include <generic-phy.h>
#include <linux/bitops.h>
#include <linux/iopoll.h>
#include <reset.h>
#include <asm/io.h>

/* SerDes subsystem register offsets (relative to serdes base) */
#define SERDES_PCIE_BGR_REG		0x0004
#define SERDES_PCIE_BGR_DEASSERT	(BIT(18) | BIT(17) | BIT(16) | \
					 BIT(1) | BIT(0))
#define SERDES_CTRL0_REG		0x0010
#define SERDES_CTRL0_PCIE_EN		BIT(0)
#define SERDES_PCIE_APP_SUB_CTRL_REG	0x0020
#define SERDES_PCIE_APP_CLK_EN		BIT(0)
#define SERDES_CTRL1_REG		0x00f0
#define SERDES_CTRL1_PCIE_MODE		(BIT(29) | BIT(10))
#define SERDES_PCIE_ITS_TAGT_ADDR_REG	0x0220
#define SERDES_PCIE_ITS_TAGT_DEFAULT	0x00000345
#define SERDES_AXI2TO1_TH_REG		0x0300
#define SERDES_AXI2TO1_TH_DEFAULT	0x00ff00ff
#define SERDES_COMB1_PIPE_REG		0x6c44
#define SERDES_COMB1_PIPE_PCIE_SEL	BIT(0)

/* Combo PHY1 Top register offsets (serdes base + 0x2000) */
#define COMBOPHY_TOP_OFFSET		0x2000
#define COMBOPHY_LINK_RST_REG		0x0000
#define COMBOPHY_LINK_RST_N		BIT(0)
#define COMBOPHY_CLK_CTRL_REG		0x0004
#define COMBOPHY_CLK_REF_EN		(BIT(24) | BIT(20) | BIT(0))
#define COMBOPHY_CLK_PIPE_OUT_EN	BIT(28)
#define COMBOPHY_LANE0_RST_REG		0x0100
#define COMBOPHY_LANE0_PCIE_RST_N	BIT(0)
#define COMBOPHY_PLL_STATUS_REG		0x0900
#define COMBOPHY_PLL_LOCK_BIT		BIT(0)

/* Combo PHY1 Analog register offsets */
#define COMBOPHY_ANALOG_TX_CTRL0	0x0098
#define COMBOPHY_ANALOG_TX_CTRL1	0x00a0
#define COMBOPHY_ANALOG_TX_CTRL2	0x00a8
#define COMBOPHY_ANALOG_TX_CTRL3	0x00d8
#define COMBOPHY_ANALOG_TX_IDLE_MODE	0x0002
#define COMBOPHY_ANALOG_TX_POST_PLL	0x0270
#define COMBOPHY_ANALOG_TX_EN_BIT	BIT(4)
#define COMBOPHY_ANALOG_EXT_OFFSET	0x18000

struct sun60i_combophy_priv {
	void __iomem *serdes_base;
	void __iomem *top_base;
	void __iomem *phy_base;
	struct clk_bulk clks;
	struct reset_ctl_bulk rsts;
};

struct phy_reg_val {
	u32 off;
	u16 val;
};

/*
 * Vendor-characterized Cadence Combo PHY analog/PLL/CDR register tuning table
 * for PCIe Gen3 (8.0 GT/s) operation on Allwinner A733.
 */
static const struct phy_reg_val cphy_analog_regs[] = {
	{ 0x44, 0x1a },
	{ 0x54, 0x34 }, { 0x58, 0xda },
	{ 0x64, 0x34 }, { 0x68, 0xda },
	{ 0xc8, 0x82 }, { 0xca, 0x82 },
	{ 0xe8, 0x1a },
	{ 0x208, 0x20 }, { 0x20a, 0x7 },
	{ 0x218, 0x20 }, { 0x21a, 0x7 },
	{ 0x228, 0x30c }, { 0x22a, 0x7 },
	{ 0x248, 0x7 }, { 0x24a, 0x3 }, { 0x24c, 0xf }, { 0x250, 0x132 },
	{ 0x8180, 0x208 }, { 0x8184, 0x9c },
	{ 0x10088, 0x1a }, { 0x1008a, 0x82 },
	{ 0x10098, 0x1a }, { 0x1009a, 0x82 },
	{ 0x8246, 0xa28 },
	{ 0x128, 0x4 }, { 0x148, 0x4 }, { 0x1a8, 0x4 },
	{ 0x348, 0x509 }, { 0x368, 0x509 }, { 0x388, 0x509 },
	{ 0x34a, 0xf00 }, { 0x36a, 0xf00 }, { 0x38a, 0xf00 },
	{ 0x34c, 0xf08 }, { 0x36c, 0xf08 }, { 0x38c, 0xf08 },
	{ 0x120, 0x180 }, { 0x140, 0x133 }, { 0x1a0, 0x133 },
	{ 0x122, 0x9d8a }, { 0x142, 0xb13b }, { 0x1a2, 0xb13b },
	{ 0x124, 0x2 }, { 0x144, 0x2 }, { 0x1a4, 0x2 },
	{ 0x126, 0x102 }, { 0x146, 0xce }, { 0x1a6, 0xce },
	{ 0x340, 0x22 }, { 0x360, 0x22 }, { 0x380, 0x22 },
	{ 0x130, 0x1 }, { 0x150, 0x1 }, { 0x1b0, 0x1 },
	{ 0x132, 0x45f }, { 0x152, 0x2f0 }, { 0x1b2, 0x399 },
	{ 0x134, 0x6b }, { 0x154, 0x68 }, { 0x1b4, 0x68 },
	{ 0x136, 0x4 }, { 0x156, 0x4 }, { 0x1b6, 0x4 },
	{ 0x108, 0x104 }, { 0x188, 0x104 },
	{ 0x10a, 0x5 }, { 0x18a, 0x5 },
	{ 0x10c, 0x337 }, { 0x18c, 0x337 },
	{ 0x110, 0x3dbe }, { 0x190, 0x3dbe },
	{ 0x104, 0x3 }, { 0x184, 0x3 },
	{ 0x138, 0x14 }, { 0x1b8, 0x14 },
	{ 0x13c, 0x192 }, { 0x1bc, 0x192 },
	{ 0x13e, 0x6 }, { 0x1be, 0x6 },
	{ 0x103c0, 0x0 },
	{ 0x102e2, 0x19 }, { 0x102e4, 0x19 },
	{ 0x103fe, 0x1 },
};

static int sun60i_combophy_xlate(struct phy *phy,
				 struct ofnode_phandle_args *args)
{
	if (args->args_count != 1)
		return -EINVAL;

	if (args->args[0] != PHY_TYPE_PCIE) {
		dev_err(phy->dev, "Unsupported PHY type %u\n", args->args[0]);
		return -EINVAL;
	}

	phy->id = args->args[0];

	return 0;
}

static int sun60i_combophy_init(struct phy *phy)
{
	struct sun60i_combophy_priv *priv = dev_get_priv(phy->dev);
	int ret;

	ret = clk_enable_bulk(&priv->clks);
	if (ret)
		return ret;

	ret = reset_deassert_bulk(&priv->rsts);
	if (ret) {
		clk_disable_bulk(&priv->clks);
		return ret;
	}

	/* Configure SerDes subsystem for PCIe */
	writel(SERDES_PCIE_BGR_DEASSERT, priv->serdes_base + SERDES_PCIE_BGR_REG);
	writel(SERDES_CTRL0_PCIE_EN, priv->serdes_base + SERDES_CTRL0_REG);
	writel(SERDES_PCIE_APP_CLK_EN,
	       priv->serdes_base + SERDES_PCIE_APP_SUB_CTRL_REG);
	writel(SERDES_CTRL1_PCIE_MODE, priv->serdes_base + SERDES_CTRL1_REG);
	writel(SERDES_COMB1_PIPE_PCIE_SEL,
	       priv->serdes_base + SERDES_COMB1_PIPE_REG);
	writel(SERDES_PCIE_ITS_TAGT_DEFAULT,
	       priv->serdes_base + SERDES_PCIE_ITS_TAGT_ADDR_REG);
	writel(SERDES_AXI2TO1_TH_DEFAULT,
	       priv->serdes_base + SERDES_AXI2TO1_TH_REG);

	return 0;
}

static int sun60i_combophy_exit(struct phy *phy)
{
	struct sun60i_combophy_priv *priv = dev_get_priv(phy->dev);

	reset_assert_bulk(&priv->rsts);
	clk_disable_bulk(&priv->clks);

	return 0;
}

static int sun60i_combophy_power_on(struct phy *phy)
{
	struct sun60i_combophy_priv *priv = dev_get_priv(phy->dev);
	u32 val;
	int i, ret;

	writel(COMBOPHY_CLK_REF_EN, priv->top_base + COMBOPHY_CLK_CTRL_REG);

	for (i = 0; i < ARRAY_SIZE(cphy_analog_regs); i++)
		writew(cphy_analog_regs[i].val,
		       priv->phy_base + cphy_analog_regs[i].off);

	writew(COMBOPHY_ANALOG_TX_IDLE_MODE,
	       priv->phy_base + COMBOPHY_ANALOG_TX_CTRL0);
	writew(COMBOPHY_ANALOG_TX_IDLE_MODE,
	       priv->phy_base + COMBOPHY_ANALOG_TX_CTRL2);
	writew(COMBOPHY_ANALOG_TX_IDLE_MODE,
	       priv->phy_base + COMBOPHY_ANALOG_TX_CTRL3);

	/* Deassert link & lane 0 reset with Lane 0 Mode = PCIe */
	writel(COMBOPHY_LINK_RST_N, priv->top_base + COMBOPHY_LINK_RST_REG);
	writel(COMBOPHY_LANE0_PCIE_RST_N,
	       priv->top_base + COMBOPHY_LANE0_RST_REG);

	ret = readl_poll_timeout(priv->top_base + COMBOPHY_PLL_STATUS_REG, val,
				 (val & COMBOPHY_PLL_LOCK_BIT), 50000);
	if (ret) {
		dev_err(phy->dev, "Combo PHY PLL lock timeout (val=0x%08x)\n",
			val);
		return ret;
	}

	/* Post PLL lock setup */
	writew(COMBOPHY_ANALOG_TX_POST_PLL,
	       priv->phy_base + COMBOPHY_ANALOG_TX_CTRL1);
	writew(readw(priv->phy_base + COMBOPHY_ANALOG_TX_CTRL0) |
	       COMBOPHY_ANALOG_TX_EN_BIT,
	       priv->phy_base + COMBOPHY_ANALOG_TX_CTRL0);
	writew(0x0001, priv->phy_base + COMBOPHY_ANALOG_EXT_OFFSET);
	writel(COMBOPHY_CLK_REF_EN | COMBOPHY_CLK_PIPE_OUT_EN,
	       priv->top_base + COMBOPHY_CLK_CTRL_REG);

	return 0;
}

static int sun60i_combophy_power_off(struct phy *phy)
{
	struct sun60i_combophy_priv *priv = dev_get_priv(phy->dev);

	writel(0x0, priv->top_base + COMBOPHY_LANE0_RST_REG);
	writel(0x0, priv->top_base + COMBOPHY_LINK_RST_REG);

	return 0;
}

static int sun60i_combophy_of_to_plat(struct udevice *dev)
{
	struct sun60i_combophy_priv *priv = dev_get_priv(dev);

	priv->serdes_base = dev_read_addr_name_ptr(dev, "serdes");
	if (!priv->serdes_base)
		return -EINVAL;

	priv->top_base = priv->serdes_base + COMBOPHY_TOP_OFFSET;

	priv->phy_base = dev_read_addr_name_ptr(dev, "phy");
	if (!priv->phy_base)
		return -EINVAL;

	return 0;
}

static int sun60i_combophy_probe(struct udevice *dev)
{
	struct sun60i_combophy_priv *priv = dev_get_priv(dev);
	int ret;

	ret = clk_get_bulk(dev, &priv->clks);
	if (ret)
		return ret;

	ret = reset_get_bulk(dev, &priv->rsts);
	if (ret) {
		clk_release_bulk(&priv->clks);
		return ret;
	}

	return 0;
}

static int sun60i_combophy_remove(struct udevice *dev)
{
	struct sun60i_combophy_priv *priv = dev_get_priv(dev);

	reset_release_bulk(&priv->rsts);
	clk_release_bulk(&priv->clks);

	return 0;
}

static const struct phy_ops sun60i_combophy_ops = {
	.of_xlate	= sun60i_combophy_xlate,
	.init		= sun60i_combophy_init,
	.exit		= sun60i_combophy_exit,
	.power_on	= sun60i_combophy_power_on,
	.power_off	= sun60i_combophy_power_off,
};

static const struct udevice_id sun60i_combophy_ids[] = {
	{ .compatible = "allwinner,sun60i-a733-combophy" },
	{ }
};

U_BOOT_DRIVER(sun60i_combophy) = {
	.name		= "sun60i_combophy",
	.id		= UCLASS_PHY,
	.of_match	= sun60i_combophy_ids,
	.ops		= &sun60i_combophy_ops,
	.of_to_plat	= sun60i_combophy_of_to_plat,
	.probe		= sun60i_combophy_probe,
	.remove		= sun60i_combophy_remove,
	.priv_auto	= sizeof(struct sun60i_combophy_priv),
};
