// SPDX-License-Identifier: GPL-2.0+
/*
 * Allwinner A733 DesignWare based PCIe host controller driver
 *
 * Copyright (c) 2026 Armbian Project
 */

#include <clk.h>
#include <dm.h>
#include <dm/device_compat.h>
#include <generic-phy.h>
#include <pci.h>
#include <power/regulator.h>
#include <reset.h>
#include <asm/global_data.h>
#include <asm/io.h>
#include <asm-generic/gpio.h>
#include <linux/delay.h>
#include <linux/iopoll.h>

#include "pcie_dw_common.h"

DECLARE_GLOBAL_DATA_PTR;

struct sunxi_pcie {
	/* Must be first member of the struct */
	struct pcie_dw dw;
	void __iomem *app_base;
	struct udevice *vpcie3v3;
	struct phy phy;
	struct clk_bulk clks;
	struct reset_ctl_bulk rsts;
	struct gpio_desc rst_gpio;
	u32 gen;
	u32 num_lanes;
};

/* Allwinner PCIe application register offsets */
#define PCIE_APP_CLK_CTRL		0x0800
#define PCIE_APP_CLK_CTRL_DEFAULT	0x00a023f0
#define PCIE_LTSSM_CTRL			0x0c00
#define PCIE_DEVICE_TYPE_RC		BIT(6)
#define PCIE_LINK_TRAINING		BIT(0)
#define PCIE_LINK_STAT			0x0e0c
#define SMLH_LINK_UP			BIT(0)
#define RDLH_LINK_UP			BIT(1)

/* Synopsys DWC port logic / coherent control offsets */
#define PCIE_PORT_DEBUG0		0x0728
#define PCIE_PORT_DEBUG0_LTSSM_MASK	GENMASK(5, 0)
#define PCIE_PORT_DEBUG0_LTSSM_L0	0x11
#define PCIE_AXI_MISC_CTRL		0x0718
#define PCIE_AXI_CTO_RESP_EN		BIT(19)
#define PCIE_COHERENCY_CTRL1		0x08e0
#define PCIE_COHERENCY_CTRL2		0x08e8

/* Unrolled Inbound iATU offset within atu_base */
#define PCIE_ATU_INB_UNR_OFFSET		0x100

static int sunxi_pcie_prog_inbound_atu(struct sunxi_pcie *priv, int index,
				       int type, u64 cpu_addr, u64 pci_addr,
				       u64 size)
{
	void __iomem *inb = priv->dw.atu_base + PCIE_ATU_INB_UNR_OFFSET +
			    PCIE_GET_ATU_OUTB_UNR_REG_OFFSET(index);
	u32 val;
	int ret;

	dw_pcie_dbi_write_enable(&priv->dw, true);

	writel(lower_32_bits(pci_addr), inb + PCIE_ATU_UNR_LOWER_BASE);
	writel(upper_32_bits(pci_addr), inb + PCIE_ATU_UNR_UPPER_BASE);
	writel(lower_32_bits(pci_addr + size - 1), inb + PCIE_ATU_UNR_LIMIT);
	writel(upper_32_bits(pci_addr + size - 1), inb + PCIE_ATU_UNR_UPPER_LIMIT);
	writel(lower_32_bits(cpu_addr), inb + PCIE_ATU_UNR_LOWER_TARGET);
	writel(upper_32_bits(cpu_addr), inb + PCIE_ATU_UNR_UPPER_TARGET);
	writel(type, inb + PCIE_ATU_UNR_REGION_CTRL1);
	writel(PCIE_ATU_ENABLE, inb + PCIE_ATU_UNR_REGION_CTRL2);

	ret = readl_poll_timeout(inb + PCIE_ATU_UNR_REGION_CTRL2, val,
				 val & PCIE_ATU_ENABLE,
				 LINK_WAIT_MAX_IATU_RETRIES * LINK_WAIT_IATU);
	dw_pcie_dbi_write_enable(&priv->dw, false);
	if (ret) {
		dev_err(priv->dw.dev, "inbound iATU is not being enabled\n");
		return ret;
	}

	return 0;
}

static int sunxi_pcie_link_up(struct udevice *dev, struct sunxi_pcie *priv)
{
	u32 val;
	int ret;

	/* Set LTSSM Enable with RC Mode */
	writel(PCIE_DEVICE_TYPE_RC | PCIE_LINK_TRAINING,
	       priv->app_base + PCIE_LTSSM_CTRL);

	/* Wait for SMLH link up and L0 active */
	ret = readl_poll_timeout(priv->app_base + PCIE_LINK_STAT, val,
				 ((val & SMLH_LINK_UP) &&
				  ((val & RDLH_LINK_UP) ||
				   ((readl(priv->dw.dbi_base + PCIE_PORT_DEBUG0) &
				     PCIE_PORT_DEBUG0_LTSSM_MASK) ==
				    PCIE_PORT_DEBUG0_LTSSM_L0))),
				 300000);
	if (ret) {
		dev_err(dev, "Link training timeout (LINK_STAT=0x%08x, DBG0=0x%08x)\n",
			readl(priv->app_base + PCIE_LINK_STAT),
			readl(priv->dw.dbi_base + PCIE_PORT_DEBUG0));
		return -ETIMEDOUT;
	}

	return 0;
}

static int sunxi_pcie_of_to_plat(struct udevice *dev)
{
	struct sunxi_pcie *priv = dev_get_priv(dev);

	priv->dw.dbi_base = dev_read_addr_name_ptr(dev, "dbi");
	if (!priv->dw.dbi_base)
		return -EINVAL;

	priv->app_base = dev_read_addr_name_ptr(dev, "app");
	if (!priv->app_base)
		return -EINVAL;

	priv->dw.cfg_base =
		dev_read_addr_size_name_ptr(dev, "config", &priv->dw.cfg_size);
	if (!priv->dw.cfg_base)
		return -EINVAL;

	priv->gen = dev_read_u32_default(dev, "max-link-speed",
					 LINK_SPEED_GEN_3);
	priv->num_lanes = dev_read_u32_default(dev, "num-lanes", 1);

	return 0;
}

static int sunxi_pcie_probe(struct udevice *dev)
{
	struct sunxi_pcie *priv = dev_get_priv(dev);
	u64 dma_size;
	u32 val;
	int ret;

	priv->dw.first_busno = dev_seq(dev);
	priv->dw.dev = dev;

	ret = gpio_request_by_name(dev, "reset-gpios", 0, &priv->rst_gpio,
				   GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE);
	if (ret && ret != -ENOENT)
		return ret;

	ret = device_get_supply_regulator(dev, "vpcie3v3-supply",
					  &priv->vpcie3v3);
	if (ret && ret != -ENOENT && ret != -ENOSYS)
		goto err_gpio;

	ret = clk_get_bulk(dev, &priv->clks);
	if (ret)
		goto err_gpio;

	ret = reset_get_bulk(dev, &priv->rsts);
	if (ret)
		goto err_clk_rel;

	ret = generic_phy_get_by_index(dev, 0, &priv->phy);
	if (ret)
		goto err_rst_rel;

	if (priv->vpcie3v3) {
		ret = regulator_set_enable_if_allowed(priv->vpcie3v3, true);
		if (ret && ret != -ENOSYS) {
			dev_err(dev, "Failed to enable vpcie3v3: %d\n", ret);
			goto err_rst_rel;
		}
		mdelay(50);
	}

	/* Assert PERST# during PHY and controller initialization */
	if (dm_gpio_is_valid(&priv->rst_gpio))
		dm_gpio_set_value(&priv->rst_gpio, 1);

	ret = clk_enable_bulk(&priv->clks);
	if (ret)
		goto err_reg;

	ret = reset_deassert_bulk(&priv->rsts);
	if (ret)
		goto err_clk;

	ret = generic_phy_init(&priv->phy);
	if (ret)
		goto err_rst;

	ret = generic_phy_power_on(&priv->phy);
	if (ret)
		goto err_phy_exit;

	/* Enable core, AXI, DBI, and AUX clock gates with PM defaults */
	writel(PCIE_APP_CLK_CTRL_DEFAULT, priv->app_base + PCIE_APP_CLK_CTRL);

	/* Hold PERST# asserted for 20 ms, then deassert and wait 100 ms */
	if (dm_gpio_is_valid(&priv->rst_gpio)) {
		mdelay(20);
		dm_gpio_set_value(&priv->rst_gpio, 0);
		mdelay(100);
	}

	/* Setup DWC controller host registers */
	pcie_dw_setup_host(&priv->dw);

	/* Clear PORT_LOGIC_SPEED_CHANGE set by pcie_dw_setup_host before link up */
	clrbits_le32(priv->dw.dbi_base + PCIE_LINK_WIDTH_SPEED_CONTROL,
		     PORT_LOGIC_SPEED_CHANGE);

	/*
	 * Clear RC BAR0/1 so U-Boot's PCI BAR scan does not allocate outbound
	 * memory space for unused root complex BARs, program max link
	 * capability/target speed and lane count, and enable CTO/coherency
	 * defaults for Allwinner A733.
	 */
	dw_pcie_dbi_write_enable(&priv->dw, true);
	writel(0x00000000, priv->dw.dbi_base + PCI_BASE_ADDRESS_0);
	writel(0x00000000, priv->dw.dbi_base + PCI_BASE_ADDRESS_1);
	clrsetbits_le32(priv->dw.dbi_base + PCIE_LINK_CAPABILITY,
			TARGET_LINK_SPEED_MASK, priv->gen);
	clrsetbits_le32(priv->dw.dbi_base + PCIE_LINK_CTL_2,
			TARGET_LINK_SPEED_MASK, priv->gen);
	dw_pcie_link_set_max_link_width(&priv->dw, priv->num_lanes);
	setbits_le32(priv->dw.dbi_base + PCIE_AXI_MISC_CTRL,
		     PCIE_AXI_CTO_RESP_EN);
	writel(0x00000000, priv->dw.dbi_base + PCIE_COHERENCY_CTRL1);
	writel(0x00000000, priv->dw.dbi_base + PCIE_COHERENCY_CTRL2);
	dw_pcie_dbi_write_enable(&priv->dw, false);

	/* Start LTSSM link training and wait for SMLH & RDLH link up */
	ret = sunxi_pcie_link_up(dev, priv);
	if (ret)
		goto err_phy_power;

	/* Trigger speed change after Gen1 link is established */
	if (priv->gen > LINK_SPEED_GEN_1) {
		dw_pcie_dbi_write_enable(&priv->dw, true);
		setbits_le32(priv->dw.dbi_base + PCIE_LINK_WIDTH_SPEED_CONTROL,
			     PORT_LOGIC_SPEED_CHANGE);
		ret = readl_poll_timeout(priv->dw.dbi_base +
					 PCIE_LINK_WIDTH_SPEED_CONTROL,
					 val, !(val & PORT_LOGIC_SPEED_CHANGE),
					 200000);
		if (ret)
			dev_warn(dev, "Gen%d speed change timeout\n",
				 priv->gen);
		dw_pcie_dbi_write_enable(&priv->dw, false);
	}

	/*
	 * Program 32-bit inbound DMA ATU window matching the DT dma-ranges
	 * property (0x40000000..0xffffffff) for system DRAM.
	 */
	if (gd->ram_base < 0x100000000ULL) {
		dma_size = min_t(u64, gd->ram_size,
				 0x100000000ULL - gd->ram_base);
		if (dma_size) {
			ret = sunxi_pcie_prog_inbound_atu(priv,
							  PCIE_ATU_REGION_INDEX0,
							  PCIE_ATU_TYPE_MEM,
							  gd->ram_base,
							  gd->ram_base,
							  dma_size);
			if (ret)
				goto err_phy_power;
		}
	}

	/* Program outbound MEM ATU */
	ret = pcie_dw_prog_outbound_atu_unroll(&priv->dw,
					       PCIE_ATU_REGION_INDEX0,
					       PCIE_ATU_TYPE_MEM,
					       priv->dw.mem.phys_start,
					       priv->dw.mem.bus_start,
					       priv->dw.mem.size);
	if (ret)
		goto err_phy_power;

	dev_info(dev, "Link up (Gen%d-x%d, Bus%d)\n",
		 pcie_dw_get_link_speed(&priv->dw),
		 pcie_dw_get_link_width(&priv->dw),
		 priv->dw.first_busno);

	return 0;

err_phy_power:
	generic_phy_power_off(&priv->phy);
err_phy_exit:
	generic_phy_exit(&priv->phy);
err_rst:
	reset_assert_bulk(&priv->rsts);
err_clk:
	clk_disable_bulk(&priv->clks);
err_reg:
	if (priv->vpcie3v3)
		regulator_set_enable_if_allowed(priv->vpcie3v3, false);
err_rst_rel:
	reset_release_bulk(&priv->rsts);
err_clk_rel:
	clk_release_bulk(&priv->clks);
err_gpio:
	if (dm_gpio_is_valid(&priv->rst_gpio))
		dm_gpio_free(dev, &priv->rst_gpio);
	return ret;
}

static int sunxi_pcie_remove(struct udevice *dev)
{
	struct sunxi_pcie *priv = dev_get_priv(dev);

	generic_phy_power_off(&priv->phy);
	generic_phy_exit(&priv->phy);
	reset_assert_bulk(&priv->rsts);
	clk_disable_bulk(&priv->clks);
	if (priv->vpcie3v3)
		regulator_set_enable_if_allowed(priv->vpcie3v3, false);
	if (dm_gpio_is_valid(&priv->rst_gpio))
		dm_gpio_free(dev, &priv->rst_gpio);
	reset_release_bulk(&priv->rsts);
	clk_release_bulk(&priv->clks);

	return 0;
}

static const struct dm_pci_ops sunxi_pcie_ops = {
	.read_config	= pcie_dw_read_config,
	.write_config	= pcie_dw_write_config,
};

static const struct udevice_id sunxi_pcie_ids[] = {
	{ .compatible = "allwinner,sun60i-a733-pcie" },
	{ }
};

U_BOOT_DRIVER(sunxi_pcie) = {
	.name		= "sunxi_pcie",
	.id		= UCLASS_PCI,
	.of_match	= sunxi_pcie_ids,
	.ops		= &sunxi_pcie_ops,
	.of_to_plat	= sunxi_pcie_of_to_plat,
	.probe		= sunxi_pcie_probe,
	.remove		= sunxi_pcie_remove,
	.priv_auto	= sizeof(struct sunxi_pcie),
};
