// SPDX-License-Identifier: GPL-2.0+
/*
 * Allwinner A733 DesignWare based PCIe host controller driver
 *
 * Copyright (c) 2026 Armbian Project
 */

#include <clk.h>
#include <dm.h>
#include <generic-phy.h>
#include <pci.h>
#include <reset.h>
#include <asm/global_data.h>
#include <asm/io.h>
#include <asm-generic/gpio.h>
#include <dm/device_compat.h>
#include <linux/bitfield.h>
#include <linux/iopoll.h>
#include <linux/delay.h>
#include <power/regulator.h>

#include "pcie_dw_common.h"

DECLARE_GLOBAL_DATA_PTR;

struct sunxi_pcie {
	/* Must be first member of the struct */
	struct pcie_dw dw;
	void __iomem *app_base;
	struct gpio_desc rst_gpio;
	struct gpio_desc pwr_gpio;
	struct gpio_desc wake_gpio;
	u32 gen;
	u32 num_lanes;
};

#define PCIE_LTSSM_CTRL			0x0c00
#define PCIE_DEVICE_TYPE_RC		BIT(6)
#define PCIE_LINK_TRAINING		BIT(0)
#define PCIE_LINK_STAT			0x0e0c
#define SMLH_LINK_UP			BIT(0)
#define RDLH_LINK_UP			BIT(1)

#define SUNXI_RTC_XO_CTRL1		0x0709016c
#define SUNXI_CCU_BASE			0x02002000
#define SUNXI_NSI_BASE			0x02020000
#define SUNXI_SERDES_SUBSYS_BASE	0x06c00000
#define SUNXI_COMBOPHY1_TOP_BASE	0x06c02000
#define SUNXI_COMBOPHY1_PHY_BASE	0x06ca0000

struct phy_reg_val {
	u32 off;
	u16 val;
};

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
	{ 0x103fe, 0x1 }
};

static int sunxi_pcie_hw_init(struct sunxi_pcie *priv)
{
	void __iomem *ccu;
	void __iomem *subsys;
	void __iomem *phy_top;
	void __iomem *phy_analog;
	u32 val;
	int i, p, ret;

	/* 1. Turn on M.2 NVMe Slot 3.3V Power via PL3 (U90 Buck Regulator) */
	if (dm_gpio_is_valid(&priv->pwr_gpio)) {
		dm_gpio_set_value(&priv->pwr_gpio, 1);
		mdelay(50);
	}

	if (dm_gpio_is_valid(&priv->wake_gpio)) {
		dm_gpio_set_value(&priv->wake_gpio, 1);
	}

	/* Assert PERST# LOW during PHY clock / PLL initialization */
	if (dm_gpio_is_valid(&priv->rst_gpio)) {
		dm_gpio_set_value(&priv->rst_gpio, 0);
	}

	/* Ensure PD20 is muxed to Function 6 (PCIE-CLKREQN) */
	clrsetbits_le32((void __iomem *)0x02000208, 0x000f0000, 0x00060000);

	/* 2. Configure RTC DCXO clock feed to Combo PHY */
	writel(0x00000530, (void __iomem *)SUNXI_RTC_XO_CTRL1);

	/* 3. Configure CCU Clocks & Resets */
	ccu = (void __iomem *)SUNXI_CCU_BASE;
	writel(0x80000000, ccu + 0x1380); /* PCIE0_AUX Clock */
	writel(0x82000000, ccu + 0x1384); /* PCIE0_AXI_SLV Clock (400MHz) */
	writel(0x00030003, ccu + 0x138c); /* PCIE0 Bus Gating Reset */
	writel(0x81000005, ccu + 0x13c0); /* SERDES_PHY_CFG Clock: PERI0_600M / 6 = 100MHz */
	writel(0x00010001, ccu + 0x13c4); /* SERDES_PHY_BGR */
	writel(0x00020001, ccu + 0x1b28); /* CM PCIE0 Mode */
	writel(0x00010002, ccu + 0x0574); /* MBUS/NSI */
	writel(0xc3000000, ccu + 0x0580);
	writel(0x00010001, ccu + 0x0584);
	writel(0x81000000, ccu + 0x0588);
	writel(0x00010007, ccu + 0x058c);
	writel(0x00010007, ccu + 0x05b4);
	writel(0xc0000000, (void __iomem *)0x02001540);
	writel(0x00010001, (void __iomem *)0x0200154c);

	/* 4. Configure SerDes Subsystem */
	subsys = (void __iomem *)SUNXI_SERDES_SUBSYS_BASE;
	writel(0x00070003, subsys + 0x0004); /* SUBSYS_PCIE_BGR */
	writel(0x00330033, subsys + 0x0008); /* SUBSYS_USB3P1_BGR */
	writel(0x00000001, subsys + 0x0010);
	writel(0x00000001, subsys + 0x0020); /* SUBSYS_PCIE_APP_SUB_CTRL */
	writel(0x20000400, subsys + 0x00f0);
	writel(0x00000001, subsys + 0x6c44); /* SUBSYS_COMB1_PIPE = 1 (PCIe) */
	writel(0x00000345, subsys + 0x0220); /* SUBSYS_PCIE_ITS_TAGT_ADDR */
	writel(0x00ff00ff, subsys + 0x0300); /* SUBSYS_AXI2TO1_TH: 16 burst */
	writel(0x00ff00ff, (void __iomem *)0x08868020);

	/* 5. Configure NSI Interconnect (Ports 0..15 QoS & Bandwidth) */
	for (p = 0; p < 16; p++) {
		void __iomem *port = (void __iomem *)(SUNXI_NSI_BASE + (0x200 * p));
		writel(0x00000001, port + 0x0c);
		writel(0x00000001, port + 0x18);
		writel(0x000003e8, port + 0x28);
		writel(0x000000ff, port + 0x2c);
		writel(0x00000001, port + 0xcc);
	}

	/* 6. Program Cadence Combo PHY 1 */
	phy_top = (void __iomem *)SUNXI_COMBOPHY1_TOP_BASE;
	phy_analog = (void __iomem *)SUNXI_COMBOPHY1_PHY_BASE;

	writel(0x01100001, phy_top + 0x0004);

	for (i = 0; i < ARRAY_SIZE(cphy_analog_regs); i++)
		writew(cphy_analog_regs[i].val, phy_analog + cphy_analog_regs[i].off);

	writel(0x00000002, phy_analog + 0x0098);
	writel(0x00000002, phy_analog + 0x00a8);
	writel(0x00000002, phy_analog + 0x00d8);

	/* Deassert link & lane 0 reset, ensuring Lane 0 Mode is PCIe (bits[5:4]=00) */
	writel(0x00000001, phy_top + 0x0000);
	writel(0x00000001, phy_top + 0x0100);

	/* Poll for Combo PHY 1 PLL lock (0x0900 & 0x1) */
	ret = readl_poll_timeout(phy_top + 0x0900, val, (val & 0x1), 50000);
	if (ret) {
		printf("sunxi_pcie: Combo PHY 1 PLL lock timeout! (val=0x%08x)\n", val);
		return ret;
	}

	/* Post PLL lock setup */
	writew(0x0270, phy_analog + 0x00a0);
	writew(readw(phy_analog + 0x98) | BIT(4), phy_analog + 0x0098);
	writew(0x0001, (void __iomem *)0x06cb8000);
	writel(0x11100001, phy_top + 0x0004);
	writel(0x00a023f0, priv->app_base + 0x800);

	/* 7. PERST# Deassert Sequence (20ms LOW, deassert HIGH, 100ms settling) */
	if (dm_gpio_is_valid(&priv->rst_gpio)) {
		mdelay(20);
		dm_gpio_set_value(&priv->rst_gpio, 1);
		mdelay(100);
	}

	return 0;
}

static int sunxi_pcie_link_up(struct sunxi_pcie *priv)
{
	u32 val;
	int ret;

	/* Set LTSSM Enable with RC Mode */
	writel(PCIE_DEVICE_TYPE_RC | PCIE_LINK_TRAINING, priv->app_base + PCIE_LTSSM_CTRL);

	/* Wait for SMLH link up and L0 active */
	ret = readl_poll_timeout(priv->app_base + PCIE_LINK_STAT, val,
				 ((val & SMLH_LINK_UP) &&
				  (((val & RDLH_LINK_UP)) ||
				   ((readl(priv->dw.dbi_base + 0x728) & 0x3f) == 0x11))),
				 300000);
	if (ret) {
		printf("sunxi_pcie: Link training timeout! (LINK_STAT=0x%08x, DBG0=0x%08x)\n",
		       readl(priv->app_base + PCIE_LINK_STAT),
		       readl(priv->dw.dbi_base + 0x728));
		return -ETIMEDOUT;
	}

	return 0;
}

static int sunxi_pcie_parse_dt(struct udevice *dev)
{
	struct sunxi_pcie *priv = dev_get_priv(dev);

	priv->dw.dbi_base = dev_read_addr_name_ptr(dev, "dbi");
	if (!priv->dw.dbi_base)
		priv->dw.dbi_base = dev_read_addr_index_ptr(dev, 0);
	if (!priv->dw.dbi_base)
		return -EINVAL;

	priv->app_base = dev_read_addr_name_ptr(dev, "app");
	if (!priv->app_base)
		priv->app_base = dev_read_addr_index_ptr(dev, 1);
	if (!priv->app_base)
		priv->app_base = (void __iomem *)0x06400000;

	priv->dw.cfg_base = dev_read_addr_size_name_ptr(dev, "config", &priv->dw.cfg_size);
	if (!priv->dw.cfg_base)
		priv->dw.cfg_base = dev_read_addr_size_index_ptr(dev, 2, &priv->dw.cfg_size);

	gpio_request_by_name(dev, "reset-gpios", 0, &priv->rst_gpio, GPIOD_IS_OUT);
	gpio_request_by_name(dev, "power-gpios", 0, &priv->pwr_gpio, GPIOD_IS_OUT);
	gpio_request_by_name(dev, "wake-gpios", 0, &priv->wake_gpio, GPIOD_IS_OUT);

	priv->gen = dev_read_u32_default(dev, "max-link-speed", LINK_SPEED_GEN_3);
	priv->num_lanes = dev_read_u32_default(dev, "num-lanes", 1);

	return 0;
}

static void sunxi_pcie_prog_outbound_atu(struct sunxi_pcie *priv, int index, int type,
					 u64 cpu_addr, u64 pci_addr, u64 size)
{
	void __iomem *dbi = priv->dw.dbi_base;
	u32 val;
	int i;

	dw_pcie_dbi_write_enable(&priv->dw, true);

	/* 1. Viewport Mode */
	writel(index, dbi + 0x900);
	writel(lower_32_bits(cpu_addr), dbi + 0x90c);
	writel(upper_32_bits(cpu_addr), dbi + 0x910);
	writel(lower_32_bits(cpu_addr + size - 1), dbi + 0x914);
	writel(lower_32_bits(pci_addr), dbi + 0x918);
	writel(upper_32_bits(pci_addr), dbi + 0x91c);
	writel(type, dbi + 0x904);
	writel(PCIE_ATU_ENABLE, dbi + 0x908);

	/* 2. Unrolled Outbound Mode (offset 0x300000) */
	if (priv->dw.atu_base) {
		writel(lower_32_bits(cpu_addr), priv->dw.atu_base + (index * 0x200) + 0x08);
		writel(upper_32_bits(cpu_addr), priv->dw.atu_base + (index * 0x200) + 0x0c);
		writel(lower_32_bits(cpu_addr + size - 1), priv->dw.atu_base + (index * 0x200) + 0x10);
		writel(upper_32_bits(cpu_addr + size - 1), priv->dw.atu_base + (index * 0x200) + 0x20);
		writel(lower_32_bits(pci_addr), priv->dw.atu_base + (index * 0x200) + 0x14);
		writel(upper_32_bits(pci_addr), priv->dw.atu_base + (index * 0x200) + 0x18);
		writel(type, priv->dw.atu_base + (index * 0x200) + 0x00);
		writel(PCIE_ATU_ENABLE, priv->dw.atu_base + (index * 0x200) + 0x04);

		for (i = 0; i < 1000; i++) {
			val = readl(priv->dw.atu_base + (index * 0x200) + 0x04);
			if (val & PCIE_ATU_ENABLE)
				break;
			udelay(10);
		}
	}

	dw_pcie_dbi_write_enable(&priv->dw, false);
}

static void sunxi_pcie_prog_inbound_atu(struct sunxi_pcie *priv, int index, int type,
					u64 cpu_addr, u64 pci_addr, u64 size)
{
	void __iomem *dbi = priv->dw.dbi_base;

	dw_pcie_dbi_write_enable(&priv->dw, true);

	/* 1. Viewport Mode (bit 31 selects inbound) */
	writel(0x80000000 | index, dbi + 0x900);
	writel(lower_32_bits(cpu_addr), dbi + 0x90c);
	writel(upper_32_bits(cpu_addr), dbi + 0x910);
	writel(lower_32_bits(cpu_addr + size - 1), dbi + 0x914);
	writel(lower_32_bits(pci_addr), dbi + 0x918);
	writel(upper_32_bits(pci_addr), dbi + 0x91c);
	writel(type, dbi + 0x904);
	writel(PCIE_ATU_ENABLE, dbi + 0x908);

	/* 2. Unrolled Inbound Mode (offset 0x300100) */
	if (priv->dw.atu_base) {
		writel(lower_32_bits(cpu_addr), priv->dw.atu_base + 0x100 + (index * 0x200) + 0x08);
		writel(upper_32_bits(cpu_addr), priv->dw.atu_base + 0x100 + (index * 0x200) + 0x0c);
		writel(lower_32_bits(cpu_addr + size - 1), priv->dw.atu_base + 0x100 + (index * 0x200) + 0x10);
		writel(upper_32_bits(cpu_addr + size - 1), priv->dw.atu_base + 0x100 + (index * 0x200) + 0x20);
		writel(lower_32_bits(pci_addr), priv->dw.atu_base + 0x100 + (index * 0x200) + 0x14);
		writel(upper_32_bits(pci_addr), priv->dw.atu_base + 0x100 + (index * 0x200) + 0x18);
		writel(type, priv->dw.atu_base + 0x100 + (index * 0x200) + 0x00);
		writel(PCIE_ATU_ENABLE, priv->dw.atu_base + 0x100 + (index * 0x200) + 0x04);
	}

	dw_pcie_dbi_write_enable(&priv->dw, false);
}

static int sunxi_pcie_probe(struct udevice *dev)
{
	struct sunxi_pcie *priv = dev_get_priv(dev);
	u32 val;
	int ret;

	priv->dw.first_busno = dev_seq(dev);
	priv->dw.dev = dev;

	ret = sunxi_pcie_parse_dt(dev);
	if (ret)
		return ret;

	ret = sunxi_pcie_hw_init(priv);
	if (ret)
		return ret;

	/* Setup DWC Controller Host Registers */
	pcie_dw_setup_host(&priv->dw);
	dw_pcie_link_set_max_link_width(&priv->dw, priv->num_lanes);

	/* Clear PORT_LOGIC_SPEED_CHANGE set by pcie_dw_setup_host before link is trained */
	clrbits_le32(priv->dw.dbi_base + PCIE_LINK_WIDTH_SPEED_CONTROL, PORT_LOGIC_SPEED_CHANGE);

	/* Configure DWC Core Registers for Allwinner A733 */
	dw_pcie_dbi_write_enable(&priv->dw, true);
	writew(0x0604, priv->dw.dbi_base + 0x0a); /* PCI_CLASS_BRIDGE_PCI */
	writel(0x00000000, priv->dw.dbi_base + PCI_BASE_ADDRESS_0);
	writel(0x00000000, priv->dw.dbi_base + PCI_BASE_ADDRESS_1);
	val = readl(priv->dw.dbi_base + 0x718);
	writel(val | (1 << 19), priv->dw.dbi_base + 0x718); /* Enable AXI response on Completion Timeout (CTO) */
	writel(0x00000000, priv->dw.dbi_base + 0x8e0); /* DWC Non-Coherent */
	writel(0x00000000, priv->dw.dbi_base + 0x8e8);

	dw_pcie_dbi_write_enable(&priv->dw, false);

	/* Start LTSSM Link Training and wait for SMLH & RDLH link up */
	ret = sunxi_pcie_link_up(priv);
	if (ret)
		return ret;

	/* Once Gen1 link is established, trigger speed change if target Gen > 1 */
	if (priv->gen > LINK_SPEED_GEN_1) {
		dw_pcie_dbi_write_enable(&priv->dw, true);
		val = readl(priv->dw.dbi_base + PCIE_LINK_WIDTH_SPEED_CONTROL);
		writel(val | PORT_LOGIC_SPEED_CHANGE, priv->dw.dbi_base + PCIE_LINK_WIDTH_SPEED_CONTROL);
		for (ret = 0; ret < 200; ret++) {
			val = readl(priv->dw.dbi_base + PCIE_LINK_WIDTH_SPEED_CONTROL);
			if (!(val & PORT_LOGIC_SPEED_CHANGE))
				break;
			udelay(1000);
		}
		dw_pcie_dbi_write_enable(&priv->dw, false);
	}

	/* 1. Program Inbound DMA ATU Region 0 (0x40000000..0xFFFFFFFF -> DRAM 0x40000000) */
	sunxi_pcie_prog_inbound_atu(priv, 0, PCIE_ATU_TYPE_MEM, 0x40000000, 0x40000000, 0xc0000000ULL);

	/* 2. Program Outbound MEM ATU Region 0 (0x22000000..0x2FFFFFFF) */
	sunxi_pcie_prog_outbound_atu(priv, 0, PCIE_ATU_TYPE_MEM,
				     priv->dw.mem.phys_start,
				     priv->dw.mem.bus_start,
				     priv->dw.mem.size);

	printf("sunxi_pcie: Root Complex initialized, link up (Gen%d-x%d)\n",
	       priv->gen, priv->num_lanes);
	return 0;
}

static int sunxi_pcie_read_config(const struct udevice *bus, pci_dev_t bdf,
				  uint offset, ulong *valuep,
				  enum pci_size_t size)
{
	struct udevice *ctlr = pci_get_controller((struct udevice *)bus);
	struct sunxi_pcie *priv = dev_get_priv(ctlr);
	int rel_bus = PCI_BUS(bdf) - priv->dw.first_busno;
	u32 busdev, val;
	int type;

	debug("sunxi_pcie_read_config: dev=%s bdf=%02x:%02x.%d off=0x%02x rel_bus=%d\n",
	      bus ? bus->name : "null", PCI_BUS(bdf), PCI_DEV(bdf), PCI_FUNC(bdf), offset, rel_bus);

	/* Root Complex config space (Bus 0) */
	if (rel_bus == 0) {
		if (PCI_DEV(bdf) > 0 || PCI_FUNC(bdf) > 0) {
			*valuep = pci_conv_32_to_size(0xffffffff, offset, size);
			return 0;
		}
		if (offset == PCI_BASE_ADDRESS_0 || offset == PCI_BASE_ADDRESS_1) {
			*valuep = 0;
			return 0;
		}
		val = readl(priv->dw.dbi_base + (offset & ~0x3));
		*valuep = pci_conv_32_to_size(val, offset, size);
		return 0;
	}

	/* Downstream bus: only Bus 1 Dev 0 exists */
	if (rel_bus > 1 || PCI_DEV(bdf) > 0) {
		*valuep = pci_conv_32_to_size(0xffffffff, offset, size);
		return 0;
	}

	/* Guard config space read against link down */
	if (!(readl(priv->app_base + PCIE_LINK_STAT) & (SMLH_LINK_UP | RDLH_LINK_UP))) {
		*valuep = pci_conv_32_to_size(0xffffffff, offset, size);
		return 0;
	}

	if (rel_bus == 1)
		type = PCIE_ATU_TYPE_CFG0;
	else
		type = PCIE_ATU_TYPE_CFG1;

	busdev = (rel_bus << 24) | (PCI_DEV(bdf) << 19) | (PCI_FUNC(bdf) << 16);

	sunxi_pcie_prog_outbound_atu(priv, 1, type, (u64)priv->dw.cfg_base, busdev, 0x1000);

	val = readl(priv->dw.cfg_base + (offset & ~0x3));
	*valuep = pci_conv_32_to_size(val, offset, size);
	return 0;
}

static int sunxi_pcie_write_config(struct udevice *bus, pci_dev_t bdf,
				   uint offset, ulong value,
				   enum pci_size_t size)
{
	struct udevice *ctlr = pci_get_controller(bus);
	struct sunxi_pcie *priv = dev_get_priv(ctlr);
	int rel_bus = PCI_BUS(bdf) - priv->dw.first_busno;
	u32 busdev, old, val;
	int type;

	/* Root Complex config space (Bus 0) */
	if (rel_bus == 0) {
		if (PCI_DEV(bdf) > 0 || PCI_FUNC(bdf) > 0)
			return 0;
		if (offset == PCI_BASE_ADDRESS_0 || offset == PCI_BASE_ADDRESS_1)
			return 0;
		dw_pcie_dbi_write_enable(&priv->dw, true);
		old = readl(priv->dw.dbi_base + (offset & ~0x3));
		val = pci_conv_size_to_32(old, value, offset, size);
		writel(val, priv->dw.dbi_base + (offset & ~0x3));
		dw_pcie_dbi_write_enable(&priv->dw, false);
		return 0;
	}

	/* Downstream bus: only Bus 1 Dev 0 exists */
	if (rel_bus > 1 || PCI_DEV(bdf) > 0)
		return 0;

	/* Guard config space write against link down */
	if (!(readl(priv->app_base + PCIE_LINK_STAT) & (SMLH_LINK_UP | RDLH_LINK_UP)))
		return 0;

	if (rel_bus == 1)
		type = PCIE_ATU_TYPE_CFG0;
	else
		type = PCIE_ATU_TYPE_CFG1;

	busdev = (rel_bus << 24) | (PCI_DEV(bdf) << 19) | (PCI_FUNC(bdf) << 16);

	sunxi_pcie_prog_outbound_atu(priv, 1, type, (u64)priv->dw.cfg_base, busdev, 0x1000);

	old = readl(priv->dw.cfg_base + (offset & ~0x3));
	val = pci_conv_size_to_32(old, value, offset, size);
	writel(val, priv->dw.cfg_base + (offset & ~0x3));
	return 0;
}

static const struct dm_pci_ops sunxi_pcie_ops = {
	.read_config	= sunxi_pcie_read_config,
	.write_config	= sunxi_pcie_write_config,
};

static const struct udevice_id sunxi_pcie_ids[] = {
	{ .compatible = "allwinner,sun60i-a733-pcie" },
	{ .compatible = "allwinner,sunxi-pcie-v300-rc" },
	{ }
};

U_BOOT_DRIVER(sunxi_pcie) = {
	.name		= "sunxi_pcie",
	.id		= UCLASS_PCI,
	.of_match	= sunxi_pcie_ids,
	.ops		= &sunxi_pcie_ops,
	.probe		= sunxi_pcie_probe,
	.priv_auto	= sizeof(struct sunxi_pcie),
};
