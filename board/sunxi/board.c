// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2012-2013 Henrik Nordstrom <henrik@henriknordstrom.net>
 * (C) Copyright 2013 Luke Kenneth Casson Leighton <lkcl@lkcl.net>
 *
 * (C) Copyright 2007-2011
 * Allwinner Technology Co., Ltd. <www.allwinnertech.com>
 * Tom Cubie <tangliang@allwinnertech.com>
 *
 * Some board init for the Allwinner A10-evb board.
 */

#include <clock_legacy.h>
#include <dm.h>
#include <env.h>
#include <hang.h>
#include <i2c.h>
#include <image.h>
#include <init.h>
#include <log.h>
#include <mmc.h>
#include <axp_pmic.h>
#include <generic-phy.h>
#include <phy-sun4i-usb.h>
#include <asm/arch/clock.h>
#include <asm/arch/cpu.h>
#include <asm/arch/display.h>
#include <asm/arch/dram.h>
#include <asm/arch/mmc.h>
#include <asm/arch/prcm.h>
#include <asm/arch/pmic_bus.h>
#include <asm/arch/spl.h>
#include <asm/arch/sys_proto.h>
#include <asm/global_data.h>
#include <linux/delay.h>
#include <linux/printk.h>
#include <linux/types.h>
#ifndef CONFIG_ARM64
#include <asm/armv7.h>
#else
#endif
#include <asm/gpio.h>
#include <sunxi_gpio.h>
#include <asm/io.h>
#include <u-boot/crc.h>
#include <env_internal.h>
#include <linux/libfdt.h>
#include <fdt_support.h>
#include <nand.h>
#include <net.h>
#include <spl.h>
#include <sy8106a.h>
#include <asm/setup.h>
#if CONFIG_IS_ENABLED(DM_REGULATOR)
#include <power/regulator.h>
#endif
#if CONFIG_IS_ENABLED(DM_THERMAL)
#include <thermal.h>
#endif

DECLARE_GLOBAL_DATA_PTR;

void i2c_init_board(void)
{
#ifdef CONFIG_I2C0_ENABLE
#if defined(CONFIG_MACH_SUN4I) || \
    defined(CONFIG_MACH_SUN5I) || \
    defined(CONFIG_MACH_SUN7I) || \
    defined(CONFIG_MACH_SUN8I_R40)
	sunxi_gpio_set_cfgpin(SUNXI_GPB(0), SUN4I_GPB_TWI0);
	sunxi_gpio_set_cfgpin(SUNXI_GPB(1), SUN4I_GPB_TWI0);
	clock_twi_onoff(0, 1);
#elif defined(CONFIG_MACH_SUN6I)
	sunxi_gpio_set_cfgpin(SUNXI_GPH(14), SUN6I_GPH_TWI0);
	sunxi_gpio_set_cfgpin(SUNXI_GPH(15), SUN6I_GPH_TWI0);
	clock_twi_onoff(0, 1);
#elif defined(CONFIG_MACH_SUN8I_V3S)
	sunxi_gpio_set_cfgpin(SUNXI_GPB(6), SUN8I_V3S_GPB_TWI0);
	sunxi_gpio_set_cfgpin(SUNXI_GPB(7), SUN8I_V3S_GPB_TWI0);
	clock_twi_onoff(0, 1);
#elif defined(CONFIG_MACH_SUN8I)
	sunxi_gpio_set_cfgpin(SUNXI_GPH(2), SUN8I_GPH_TWI0);
	sunxi_gpio_set_cfgpin(SUNXI_GPH(3), SUN8I_GPH_TWI0);
	clock_twi_onoff(0, 1);
#elif defined(CONFIG_MACH_SUN50I)
	sunxi_gpio_set_cfgpin(SUNXI_GPH(0), SUN50I_GPH_TWI0);
	sunxi_gpio_set_cfgpin(SUNXI_GPH(1), SUN50I_GPH_TWI0);
	clock_twi_onoff(0, 1);
#endif
#endif

#ifdef CONFIG_I2C1_ENABLE
#if defined(CONFIG_MACH_SUN4I) || \
    defined(CONFIG_MACH_SUN7I) || \
    defined(CONFIG_MACH_SUN8I_R40)
	sunxi_gpio_set_cfgpin(SUNXI_GPB(18), SUN4I_GPB_TWI1);
	sunxi_gpio_set_cfgpin(SUNXI_GPB(19), SUN4I_GPB_TWI1);
	clock_twi_onoff(1, 1);
#elif defined(CONFIG_MACH_SUN5I)
	sunxi_gpio_set_cfgpin(SUNXI_GPB(15), SUN5I_GPB_TWI1);
	sunxi_gpio_set_cfgpin(SUNXI_GPB(16), SUN5I_GPB_TWI1);
	clock_twi_onoff(1, 1);
#elif defined(CONFIG_MACH_SUN6I)
	sunxi_gpio_set_cfgpin(SUNXI_GPH(16), SUN6I_GPH_TWI1);
	sunxi_gpio_set_cfgpin(SUNXI_GPH(17), SUN6I_GPH_TWI1);
	clock_twi_onoff(1, 1);
#elif defined(CONFIG_MACH_SUN8I)
	sunxi_gpio_set_cfgpin(SUNXI_GPH(4), SUN8I_GPH_TWI1);
	sunxi_gpio_set_cfgpin(SUNXI_GPH(5), SUN8I_GPH_TWI1);
	clock_twi_onoff(1, 1);
#elif defined(CONFIG_MACH_SUN50I)
	sunxi_gpio_set_cfgpin(SUNXI_GPH(2), SUN50I_GPH_TWI1);
	sunxi_gpio_set_cfgpin(SUNXI_GPH(3), SUN50I_GPH_TWI1);
	clock_twi_onoff(1, 1);
#endif
#endif

#ifdef CONFIG_R_I2C_ENABLE
#ifdef CONFIG_MACH_SUN50I
	clock_twi_onoff(5, 1);
	sunxi_gpio_set_cfgpin(SUNXI_GPL(8), SUN50I_GPL_R_TWI);
	sunxi_gpio_set_cfgpin(SUNXI_GPL(9), SUN50I_GPL_R_TWI);
#elif defined(CONFIG_MACH_SUN50I_H616)
	clock_twi_onoff(5, 1);
	sunxi_gpio_set_cfgpin(SUNXI_GPL(0), SUN50I_H616_GPL_R_TWI);
	sunxi_gpio_set_cfgpin(SUNXI_GPL(1), SUN50I_H616_GPL_R_TWI);
#elif CONFIG_MACH_SUN55I_A523
	clock_twi_onoff(5, 1);
	sunxi_gpio_set_cfgpin(SUNXI_GPL(0), SUN50I_GPL_R_TWI);
	sunxi_gpio_set_cfgpin(SUNXI_GPL(1), SUN50I_GPL_R_TWI);
#else
	clock_twi_onoff(5, 1);
	sunxi_gpio_set_cfgpin(SUNXI_GPL(0), SUN8I_H3_GPL_R_TWI);
	sunxi_gpio_set_cfgpin(SUNXI_GPL(1), SUN8I_H3_GPL_R_TWI);
#endif
#endif
}

/*
 * Try to use the environment from the boot source first.
 * For MMC, this means a FAT partition on the boot device (SD or eMMC).
 * If the raw MMC environment is also enabled, this is tried next.
 * When booting from NAND we try UBI first, then NAND directly.
 * SPI flash falls back to FAT (on SD card).
 */
enum env_location env_get_location(enum env_operation op, int prio)
{
	if (prio > 1)
		return ENVL_UNKNOWN;

	/* NOWHERE is exclusive, no other option can be defined. */
	if (IS_ENABLED(CONFIG_ENV_IS_NOWHERE))
		return ENVL_NOWHERE;

	switch (sunxi_get_boot_device()) {
	case BOOT_DEVICE_MMC1:
	case BOOT_DEVICE_MMC2:
		if (prio == 0 && IS_ENABLED(CONFIG_ENV_IS_IN_FAT))
			return ENVL_FAT;
		if (IS_ENABLED(CONFIG_ENV_IS_IN_MMC))
			return ENVL_MMC;
		break;
	case BOOT_DEVICE_NAND:
		if (prio == 0 && IS_ENABLED(CONFIG_ENV_IS_IN_UBI))
			return ENVL_UBI;
		if (IS_ENABLED(CONFIG_ENV_IS_IN_NAND))
			return ENVL_NAND;
		break;
	case BOOT_DEVICE_SPI:
		if (prio == 0 && IS_ENABLED(CONFIG_ENV_IS_IN_SPI_FLASH))
			return ENVL_SPI_FLASH;
		if (IS_ENABLED(CONFIG_ENV_IS_IN_FAT))
			return ENVL_FAT;
		break;
	case BOOT_DEVICE_BOARD:
		break;
	default:
		break;
	}

	/*
	 * If we come here for the first time, we *must* return a valid
	 * environment location other than ENVL_UNKNOWN, or the setup sequence
	 * in board_f() will silently hang. This is arguably a bug in
	 * env_init(), but for now pick one environment for which we know for
	 * sure to have a driver for. For all defconfigs this is either FAT
	 * or UBI, or NOWHERE, which is already handled above.
	 */
	if (prio == 0) {
		if (IS_ENABLED(CONFIG_ENV_IS_IN_FAT))
			return ENVL_FAT;
		if (IS_ENABLED(CONFIG_ENV_IS_IN_UBI))
			return ENVL_UBI;
	}

	return ENVL_UNKNOWN;
}

#ifdef CONFIG_MACH_SUN60I_A733
struct cphy_reg {
	u32 off;
	u16 val;
};

static const struct cphy_reg cphy_regs[] = {
	{ 0x44, 0x1a },
	{ 0x54, 0x34 },
	{ 0x58, 0xda },
	{ 0x64, 0x34 },
	{ 0x68, 0xda },
	{ 0xc8, 0x82 },
	{ 0xca, 0x82 },
	{ 0xe8, 0x1a },
	{ 0x208, 0x20 },
	{ 0x20a, 0x7 },
	{ 0x218, 0x20 },
	{ 0x21a, 0x7 },
	{ 0x228, 0x30c },
	{ 0x22a, 0x7 },
	{ 0x248, 0x7 },
	{ 0x24a, 0x3 },
	{ 0x24c, 0xf },
	{ 0x250, 0x132 },
	{ 0x10088, 0x1a },
	{ 0x10488, 0x1a },
	{ 0x10888, 0x1a },
	{ 0x10c88, 0x1a },
	{ 0x1008a, 0x82 },
	{ 0x1048a, 0x82 },
	{ 0x1088a, 0x82 },
	{ 0x10c8a, 0x82 },
	{ 0x10098, 0x1a },
	{ 0x10498, 0x1a },
	{ 0x10898, 0x1a },
	{ 0x10c98, 0x1a },
	{ 0x1009a, 0x82 },
	{ 0x1049a, 0x82 },
	{ 0x1089a, 0x82 },
	{ 0x10c9a, 0x82 },
	{ 0x8246, 0xa28 },
	{ 0x8646, 0xa28 },
	{ 0x8a46, 0xa28 },
	{ 0x8e46, 0xa28 },
	{ 0x342, 0x8600 },
	{ 0x382, 0x601 },
	{ 0x89ca, 0x12 },
	{ 0x8dca, 0x12 },
	{ 0x81cc, 0x0 },
	{ 0x85cc, 0x0 },
	{ 0x81ce, 0x1 },
	{ 0x85ce, 0x1 },
	{ 0x81ca, 0x12 },
	{ 0x85ca, 0x12 },
	{ 0x81ca, 0x41 },
	{ 0x85ca, 0x41 },
	{ 0x89cc, 0x1 },
	{ 0x8dcc, 0x1 },
	{ 0x89ce, 0x0 },
	{ 0x8dce, 0x0 },
	{ 0x89ca, 0x19 },
	{ 0x8dca, 0x19 },
	{ 0x128, 0x4 },
	{ 0x1a8, 0x4 },
	{ 0x348, 0x509 },
	{ 0x388, 0x509 },
	{ 0x34a, 0xf00 },
	{ 0x38a, 0xf00 },
	{ 0x34c, 0xf08 },
	{ 0x38c, 0xf08 },
	{ 0x120, 0x180 },
	{ 0x1a0, 0x19f },
	{ 0x122, 0x9d8a },
	{ 0x1a2, 0x6276 },
	{ 0x124, 0x2 },
	{ 0x1a4, 0x2 },
	{ 0x126, 0x102 },
	{ 0x1a6, 0x116 },
	{ 0x340, 0x2 },
	{ 0x380, 0x2 },
	{ 0x130, 0x1 },
	{ 0x1b0, 0x1 },
	{ 0x132, 0x45f },
	{ 0x1b2, 0x4c4 },
	{ 0x134, 0x6b },
	{ 0x1b4, 0x6a },
	{ 0x136, 0x4 },
	{ 0x1b6, 0x4 },
	{ 0x108, 0x104 },
	{ 0x188, 0x104 },
	{ 0x10a, 0x5 },
	{ 0x18a, 0x5 },
	{ 0x10c, 0x337 },
	{ 0x18c, 0x337 },
	{ 0x110, 0x335 },
	{ 0x190, 0x335 },
	{ 0x104, 0x3 },
	{ 0x184, 0x3 },
	{ 0x138, 0xcf },
	{ 0x1b8, 0xcf },
	{ 0x13c, 0xce },
	{ 0x1bc, 0xce },
	{ 0x13e, 0x5 },
	{ 0x1be, 0x5 },
	{ 0x18020, 0x5100 },
	{ 0x18022, 0x100 },
	{ 0x18022, 0x10f },
	{ 0x18030, 0xa0a },
	{ 0x18034, 0x1008 },
	{ 0x18036, 0x10 },
	{ 0x82, 0x8200 },
	{ 0x8e, 0x8200 },
	{ 0x8e00, 0xff },
	{ 0x8e02, 0x4af },
	{ 0x8e04, 0x4ae },
	{ 0x8e06, 0x4ae },
	{ 0x10800, 0x91d },
	{ 0x10802, 0x91d },
	{ 0x10804, 0x900 },
	{ 0x10806, 0x0 },
	{ 0x8c80, 0x2a84 },
	{ 0x8c9a, 0x11 },
	{ 0x10920, 0xc },
	{ 0x10a10, 0x9 },
	{ 0x10a92, 0xc02 },
	{ 0x10aee, 0x6f6 },
	{ 0x10af0, 0x4606 },
	{ 0x89d6, 0x6 },
	{ 0x10ae2, 0x519 },
	{ 0x10ae4, 0x519 },
	{ 0x10bd0, 0x1002 },
	{ 0x10bca, 0xb98 },
	{ 0x10bc4, 0xc01 },
	{ 0x10bc6, 0x0 },
	{ 0x10bea, 0x0 },
	{ 0x10be8, 0x311 },
	{ 0x10bfe, 0x0 },
	{ 0x10ffe, 0x0 },
	{ 0x10900, 0x10a },
	{ 0x10904, 0x3 },
	{ 0x8dd4, 0xff },
	{ 0x89d4, 0xff },
	{ 0x8200, 0x2ff },
	{ 0x8202, 0x6af },
	{ 0x8204, 0x6ae },
	{ 0x8206, 0x6ae },
	{ 0x10400, 0xd1d },
	{ 0x10402, 0xd1d },
	{ 0x10404, 0xd00 },
	{ 0x10406, 0x500 },
	{ 0x8080, 0x2a82 },
	{ 0x809a, 0x14 },
	{ 0x10520, 0x13 },
	{ 0x10610, 0x0 },
	{ 0x10692, 0xc02 },
	{ 0x106ee, 0x330 },
	{ 0x106f0, 0x300 },
	{ 0x85d6, 0x3 },
	{ 0x106e2, 0x19 },
	{ 0x106e4, 0x19 },
	{ 0x107d0, 0x1004 },
	{ 0x107ca, 0xf9 },
	{ 0x107c4, 0xc01 },
	{ 0x107c6, 0x2 },
	{ 0x107ea, 0x0 },
	{ 0x107e8, 0x31 },
	{ 0x107fe, 0x1 },
	{ 0x103fe, 0x2 },
	{ 0x10500, 0x18c },
	{ 0x10504, 0x3 },
	{ 0x81d4, 0xf },
	{ 0x85d4, 0xf0 },
	{ 0x1c006, 0x1 },
	{ 0x206, 0x7f },
	{ 0x216, 0x7f },
};

__maybe_unused static void sunxi_a733_usb_init(void)
{
	void __iomem *ccu = (void __iomem *)0x02002000;
	void __iomem *subsys = (void __iomem *)0x06c00000;
	void __iomem *phy_top0 = (void __iomem *)0x06c01000;
	void __iomem *phy_analog0 = (void __iomem *)0x06c80000;
	u32 val;
	int i;

	/* 1. CCU Master Gates & USB2 / SerDes Clocks */
	/* AHB Master Gate @ 0x5C0: Enable SerDes (bit 8) and USB (bit 9) */
	val = readl(ccu + 0x05c0);
	writel(val | BIT(8) | BIT(9), ccu + 0x05c0);

	/* MBUS Master Gate @ 0x5E0: Enable SerDes (bit 28) */
	val = readl(ccu + 0x05e0);
	writel(val | BIT(28), ccu + 0x05e0);

	/* High speed clocks & resets */
	writel(0x00000001, ccu + 0x1b30); /* CM_USB2_CFG_REG: enable USB2 module */
	writel(0x81000000, ccu + 0x1348); /* USB2_U2_REF_CLK */
	writel(0x81000000, ccu + 0x1350); /* USB2_SUSPEND_CLK */
	writel(0x81000000, ccu + 0x1354); /* USB2_MF_CLK (300MHz AXI) */
	writel(0x00010001, ccu + 0x135c); /* USB2_BGR_REG (Reset deassert) */
	writel(0x81000000, ccu + 0x1360); /* USB2_U3_UTMI_CLK */
	writel(0x81000000, ccu + 0x1364); /* USB2_U2_PIPE_CLK (480MHz PLL) */

	/* SERDES_PHY_CFG: 100MHz refclk with divisor 11 (0x8100000b) */
	writel(0x00010001, (void *)0x02001aac); /* RST_BUS_PCIE_USB3 */
	writel(0x81000000, ccu + 0x1348);       /* CLK_COMBPHY0 */
	writel(0x00010001, ccu + 0x134c);       /* RST_COMBPHY0 */
	writel(0x8100000b, ccu + 0x13c0);       /* SERDES_PHY_CFG */
	writel(0x00010001, ccu + 0x13c4);       /* SERDES_BGR */

	/* 2. Subsystem Bus Gating & Routing */
	writel(0x00330033, subsys + 0x0008); /* SUBSYS_USB3P1_BGR */
	writel(0x20000002, subsys + 0x00f0); /* SUBSYS_DBG_CTL */
	writel(0x00000000, subsys + 0x6100); /* PIPE CLK MAP -> Combo 0 */
	writel(0x00000000, subsys + 0x6104); /* PIPE RXD MAP -> Combo 0 */
	writel(0x00003210, subsys + 0x6c24); /* TOP_PIPE_MAP */
	writel(0x00000000, subsys + 0x6c40); /* TOP_PIPE_SEL0 -> Combo 0 to USB3 */
	writel(0x00000ff1, subsys + 0x6004); /* TOP_ISOLATION release */

	/* 3. Program Cadence Combo PHY 0 (Blue Type-A USB3 SerDes) */
	/* Clear IDDQ */
	val = readl(phy_top0 + 0x0008);
	val &= ~(BIT(1) | BIT(0));
	writel(val, phy_top0 + 0x0008);

	/* Set Refclk mode */
	writel(0x11100001, phy_top0 + 0x0004);

	/* Write analog SerDes registers */
	for (i = 0; i < ARRAY_SIZE(cphy_regs); i++)
		writew(cphy_regs[i].val, phy_analog0 + cphy_regs[i].off);

	/* PHY Reset Toggle */
	val = readl(phy_top0 + 0x0000);
	val &= ~BIT(0);
	writel(val, phy_top0 + 0x0000);
	mdelay(1);
	val |= BIT(0) | BIT(4); /* PHY0_RESET_N | PHY0_PIPE_LINK_RESET_N_SOFT */
	writel(val, phy_top0 + 0x0000);

	/* Enable PLL */
	val = readl(phy_top0 + 0x000c);
	val |= BIT(0); /* PHY0_PMA_XCVR_PLLCLK_EN_LN */
	writel(val, phy_top0 + 0x000c);

	/* Power State Req LN(1) */
	val = readl(phy_top0 + 0x000c);
	val |= (1 << 4);
	writel(val, phy_top0 + 0x000c);

	/* DP link reset soft */
	val = readl(phy_top0 + 0x0000);
	val |= BIT(8);
	writel(val, phy_top0 + 0x0000);

	/* PMA Fixup: 0xC008 << 1 = 0x18010 */
	val = readw(phy_analog0 + 0x18010);
	val &= ~(0xF << 8);
	writew(val, phy_analog0 + 0x18010);

	/* Poll for PMA_CMN_READY (0x06c01900 BIT(0)) */
	for (i = 0; i < 1000; i++) {
		if (readl(phy_top0 + 0x0900) & BIT(0))
			break;
		udelay(10);
	}

	/* DWC3 LLUCTL: Invert Sync Header for Cadence Combo SerDes */
	writel(0x408b8080, (void *)0x06a0d024);

	/* 4. Enable 5V USB VBUS power (PB7 = LOW for active-low USB1 VBUS, PB8 = HIGH, PD20 = HIGH) */
	/* Bank B (PB7): Base 0x02000100, CFG0 @ 0x02000100 bit 28 = 1 (output), DAT @ 0x02000110 bit 7 = 0 (low) */
	clrsetbits_le32((void *)0x02000100, 0xf0000000, 0x10000000);
	clrbits_le32((void *)0x02000110, BIT(7));

	/* Bank B (PB8): CFG1 @ 0x02000104 bit 0 = 1 (output), DAT @ 0x02000110 bit 8 = 1 (high) */
	clrsetbits_le32((void *)0x02000104, 0x0000000f, 0x00000001);
	setbits_le32((void *)0x02000110, BIT(8));

	/* Bank D (PD20): Base 0x02000200, CFG2 @ 0x02000208 bit 16 = 1 (output), DAT @ 0x02000210 bit 20 = 1 (high) */
	clrsetbits_le32((void *)0x02000208, 0x000f0000, 0x00010000);
	setbits_le32((void *)0x02000210, BIT(20));
	mdelay(100);
}
#endif

/* called only from U-Boot proper */
int board_init(void)
{
	__maybe_unused int id_pfr1, ret;

	gd->bd->bi_boot_params = (PHYS_SDRAM_0 + 0x100);

#if !defined(CONFIG_ARM64) && !defined(CONFIG_MACH_SUNIV)
	asm volatile("mrc p15, 0, %0, c0, c1, 1" : "=r"(id_pfr1));
	debug("id_pfr1: 0x%08x\n", id_pfr1);
	/* Generic Timer Extension available? */
	if ((id_pfr1 >> CPUID_ARM_GENTIMER_SHIFT) & 0xf) {
		uint32_t freq;

		debug("Setting CNTFRQ\n");

		/*
		 * CNTFRQ is a secure register, so we will crash if we try to
		 * write this from the non-secure world (read is OK, though).
		 * In case some bootcode has already set the correct value,
		 * we avoid the risk of writing to it.
		 */
		asm volatile("mrc p15, 0, %0, c14, c0, 0" : "=r"(freq));
		if (freq != CONFIG_COUNTER_FREQUENCY) {
			debug("arch timer frequency is %d Hz, should be %d, fixing ...\n",
			      freq, CONFIG_COUNTER_FREQUENCY);
#ifdef CONFIG_NON_SECURE
			printf("arch timer frequency is wrong, but cannot adjust it\n");
#else
			asm volatile("mcr p15, 0, %0, c14, c0, 0"
				     : : "r"(CONFIG_COUNTER_FREQUENCY));
#endif
		}
	}
#endif /* !CONFIG_ARM64 && !CONFIG_MACH_SUNIV */

	ret = axp_gpio_init();
	if (ret)
		return ret;

#if defined(CONFIG_MACH_SUN60I_A733) && !defined(CONFIG_SPL_BUILD)
		/* Enable PLL_DE (0x020022E0) with output gates 0 and 1 un-gated (0xEC125600) for DE3.5 */
		writel(0xEC125600, (void *)0x020022E0);

		/* Enable Clock Matrix for DE and Video Out: CM_DESYS (0x02003B04), CM_VO (0x02003B34), CM_VO1 (0x02003B38) */
		writel(0x00020001, (void *)0x02003B04);
		writel(0x00020001, (void *)0x02003B34);
		writel(0x00020001, (void *)0x02003B38);

		/* Deassert DE resets and enable bus gating and module clock */
		writel(0x00010001, (void *)0x02002A74); /* DE_SYS_BGR_REG */
		writel(0x00010001, (void *)0x02002A04); /* DE0_BGR_REG */
		writel(0x80000000, (void *)0x02002A00); /* DE0_CLK_REG: DEPLL3X, div 1, gate ON */

		sunxi_a733_usb_init();
#endif

	eth_init_board();

	return 0;
}

/*
 * On older SoCs the SPL is actually at address zero, so using NULL as
 * an error value does not work.
 */
#define INVALID_SPL_HEADER ((void *)~0UL)

static struct boot_file_head * get_spl_header(uint8_t req_version)
{
	struct boot_file_head *spl = (void *)(ulong)SPL_ADDR;
	uint8_t spl_header_version = spl->spl_signature[3];

	/* Is there really the SPL header (still) there? */
	if (memcmp(spl->spl_signature, SPL_SIGNATURE, 3) != 0)
		return INVALID_SPL_HEADER;

	if (spl_header_version < req_version) {
		printf("sunxi SPL version mismatch: expected %u, got %u\n",
		       req_version, spl_header_version);
		return INVALID_SPL_HEADER;
	}

	return spl;
}

static const char *get_spl_dt_name(void)
{
	struct boot_file_head *spl = get_spl_header(SPL_DT_HEADER_VERSION);

	/* Check if there is a DT name stored in the SPL header. */
	if (spl != INVALID_SPL_HEADER && spl->dt_name_offset)
		return (char *)spl + spl->dt_name_offset;

	return NULL;
}

int dram_init(void)
{
	struct boot_file_head *spl = get_spl_header(SPL_DRAM_HEADER_VERSION);

	if (spl != INVALID_SPL_HEADER && spl->dram_size)
		gd->ram_size = (phys_addr_t)spl->dram_size << 20;
	else
		gd->ram_size = get_ram_size((long *)PHYS_SDRAM_0, PHYS_SDRAM_0_SIZE);

	if (gd->ram_size > CONFIG_SUNXI_DRAM_MAX_SIZE)
		gd->ram_size = CONFIG_SUNXI_DRAM_MAX_SIZE;

	return 0;
}

#if defined(CONFIG_NAND_SUNXI) && defined(CONFIG_XPL_BUILD)
static void nand_pinmux_setup(void)
{
	unsigned int pin;

	for (pin = SUNXI_GPC(0); pin <= SUNXI_GPC(19); pin++)
		sunxi_gpio_set_cfgpin(pin, SUNXI_GPC_NAND);

#if defined CONFIG_MACH_SUN4I || defined CONFIG_MACH_SUN7I
	for (pin = SUNXI_GPC(20); pin <= SUNXI_GPC(22); pin++)
		sunxi_gpio_set_cfgpin(pin, SUNXI_GPC_NAND);
#endif
	/* sun4i / sun7i do have a PC23, but it is not used for nand,
	 * only sun7i has a PC24 */
#ifdef CONFIG_MACH_SUN7I
	sunxi_gpio_set_cfgpin(SUNXI_GPC(24), SUNXI_GPC_NAND);
#endif
}

static void nand_clock_setup(void)
{
	void * const ccm = (void *)SUNXI_CCM_BASE;

#if defined(CONFIG_MACH_SUN50I_H616) || defined(CONFIG_MACH_SUN50I_H6)
	setbits_le32(ccm + CCU_H6_NAND_GATE_RESET,
		     (1 << GATE_SHIFT) | (1 << RESET_SHIFT));
	setbits_le32(ccm + CCU_H6_MBUS_GATE, (1 << MBUS_GATE_OFFSET_NAND));
	setbits_le32(ccm + CCU_NAND1_CLK_CFG, CCM_NAND_CTRL_ENABLE |
		     CCM_NAND_CTRL_N(0) | CCM_NAND_CTRL_M(1));
#else
	setbits_le32(ccm + CCU_AHB_GATE0,
		     (CLK_GATE_OPEN << AHB_GATE_OFFSET_NAND0));
#if defined CONFIG_MACH_SUN6I || defined CONFIG_MACH_SUN8I || \
    defined CONFIG_MACH_SUN9I || defined CONFIG_MACH_SUN50I
	setbits_le32(ccm + CCU_AHB_RESET0_CFG, (1 << AHB_GATE_OFFSET_NAND0));
#endif
#endif
	setbits_le32(ccm + CCU_NAND0_CLK_CFG, CCM_NAND_CTRL_ENABLE |
		     CCM_NAND_CTRL_N(0) | CCM_NAND_CTRL_M(1));
}

void board_nand_init(void)
{
	nand_pinmux_setup();
	nand_clock_setup();
}
#endif /* CONFIG_NAND_SUNXI */

#ifdef CONFIG_MMC
static void mmc_pinmux_setup(int sdc)
{
	unsigned int pin;

	switch (sdc) {
	case 0:
		/* SDC0: PF0-PF5 */
		for (pin = SUNXI_GPF(0); pin <= SUNXI_GPF(5); pin++) {
			sunxi_gpio_set_cfgpin(pin, SUNXI_GPF_SDC0);
			sunxi_gpio_set_pull(pin, SUNXI_GPIO_PULL_UP);
			sunxi_gpio_set_drv(pin, 2);
		}
		break;

	case 1:
#if defined(CONFIG_MACH_SUN4I) || defined(CONFIG_MACH_SUN7I) || \
    defined(CONFIG_MACH_SUN8I_R40)
		if (IS_ENABLED(CONFIG_MMC1_PINS_PH)) {
			/* SDC1: PH22-PH-27 */
			for (pin = SUNXI_GPH(22); pin <= SUNXI_GPH(27); pin++) {
				sunxi_gpio_set_cfgpin(pin, SUN4I_GPH_SDC1);
				sunxi_gpio_set_pull(pin, SUNXI_GPIO_PULL_UP);
				sunxi_gpio_set_drv(pin, 2);
			}
		} else {
			/* SDC1: PG0-PG5 */
			for (pin = SUNXI_GPG(0); pin <= SUNXI_GPG(5); pin++) {
				sunxi_gpio_set_cfgpin(pin, SUN4I_GPG_SDC1);
				sunxi_gpio_set_pull(pin, SUNXI_GPIO_PULL_UP);
				sunxi_gpio_set_drv(pin, 2);
			}
		}
#elif defined(CONFIG_MACH_SUN5I)
		/* SDC1: PG3-PG8 */
		for (pin = SUNXI_GPG(3); pin <= SUNXI_GPG(8); pin++) {
			sunxi_gpio_set_cfgpin(pin, SUN5I_GPG_SDC1);
			sunxi_gpio_set_pull(pin, SUNXI_GPIO_PULL_UP);
			sunxi_gpio_set_drv(pin, 2);
		}
#elif defined(CONFIG_MACH_SUN6I)
		/* SDC1: PG0-PG5 */
		for (pin = SUNXI_GPG(0); pin <= SUNXI_GPG(5); pin++) {
			sunxi_gpio_set_cfgpin(pin, SUN6I_GPG_SDC1);
			sunxi_gpio_set_pull(pin, SUNXI_GPIO_PULL_UP);
			sunxi_gpio_set_drv(pin, 2);
		}
#elif defined(CONFIG_MACH_SUN8I)
		/* SDC1: PG0-PG5 */
		for (pin = SUNXI_GPG(0); pin <= SUNXI_GPG(5); pin++) {
			sunxi_gpio_set_cfgpin(pin, SUN8I_GPG_SDC1);
			sunxi_gpio_set_pull(pin, SUNXI_GPIO_PULL_UP);
			sunxi_gpio_set_drv(pin, 2);
		}
#endif
		break;

	case 2:
#if defined(CONFIG_MACH_SUN4I) || defined(CONFIG_MACH_SUN7I)
		/* SDC2: PC6-PC11 */
		for (pin = SUNXI_GPC(6); pin <= SUNXI_GPC(11); pin++) {
			sunxi_gpio_set_cfgpin(pin, SUNXI_GPC_SDC2);
			sunxi_gpio_set_pull(pin, SUNXI_GPIO_PULL_UP);
			sunxi_gpio_set_drv(pin, 2);
		}
#elif defined(CONFIG_MACH_SUN5I)
		/* SDC2: PC6-PC15 */
		for (pin = SUNXI_GPC(6); pin <= SUNXI_GPC(15); pin++) {
			sunxi_gpio_set_cfgpin(pin, SUNXI_GPC_SDC2);
			sunxi_gpio_set_pull(pin, SUNXI_GPIO_PULL_UP);
			sunxi_gpio_set_drv(pin, 2);
		}
#elif defined(CONFIG_MACH_SUN6I)
		/* SDC2: PC6-PC15, PC24 */
		for (pin = SUNXI_GPC(6); pin <= SUNXI_GPC(15); pin++) {
			sunxi_gpio_set_cfgpin(pin, SUNXI_GPC_SDC2);
			sunxi_gpio_set_pull(pin, SUNXI_GPIO_PULL_UP);
			sunxi_gpio_set_drv(pin, 2);
		}

		sunxi_gpio_set_cfgpin(SUNXI_GPC(24), SUNXI_GPC_SDC2);
		sunxi_gpio_set_pull(SUNXI_GPC(24), SUNXI_GPIO_PULL_UP);
		sunxi_gpio_set_drv(SUNXI_GPC(24), 2);
#elif defined(CONFIG_MACH_SUN8I_R40)
		/* SDC2: PC6-PC15, PC24 */
		for (pin = SUNXI_GPC(6); pin <= SUNXI_GPC(15); pin++) {
			sunxi_gpio_set_cfgpin(pin, SUNXI_GPC_SDC2);
			sunxi_gpio_set_pull(pin, SUNXI_GPIO_PULL_UP);
			sunxi_gpio_set_drv(pin, 2);
		}

		sunxi_gpio_set_cfgpin(SUNXI_GPC(24), SUNXI_GPC_SDC2);
		sunxi_gpio_set_pull(SUNXI_GPC(24), SUNXI_GPIO_PULL_UP);
		sunxi_gpio_set_drv(SUNXI_GPC(24), 2);
#elif defined(CONFIG_MACH_SUN8I) || defined(CONFIG_MACH_SUN50I)
		/* SDC2: PC5-PC6, PC8-PC16 */
		for (pin = SUNXI_GPC(5); pin <= SUNXI_GPC(6); pin++) {
			sunxi_gpio_set_cfgpin(pin, SUNXI_GPC_SDC2);
			sunxi_gpio_set_pull(pin, SUNXI_GPIO_PULL_UP);
			sunxi_gpio_set_drv(pin, 2);
		}

		for (pin = SUNXI_GPC(8); pin <= SUNXI_GPC(16); pin++) {
			sunxi_gpio_set_cfgpin(pin, SUNXI_GPC_SDC2);
			sunxi_gpio_set_pull(pin, SUNXI_GPIO_PULL_UP);
			sunxi_gpio_set_drv(pin, 2);
		}
#elif defined(CONFIG_MACH_SUN50I_H6)
		/* SDC2: PC4-PC14 */
		for (pin = SUNXI_GPC(4); pin <= SUNXI_GPC(14); pin++) {
			sunxi_gpio_set_cfgpin(pin, SUNXI_GPC_SDC2);
			sunxi_gpio_set_pull(pin, SUNXI_GPIO_PULL_UP);
			sunxi_gpio_set_drv(pin, 2);
		}
#elif defined(CONFIG_MACH_SUN50I_H616) || defined(CONFIG_MACH_SUN50I_A133) || \
      defined(CONFIG_MACH_SUN55I_A523) || defined(CONFIG_MACH_SUN60I_A733)
		/* SDC2: PC0-PC1, PC5-PC6, PC8-PC11, PC13-PC16 */
		for (pin = SUNXI_GPC(0); pin <= SUNXI_GPC(16); pin++) {
			if (pin > SUNXI_GPC(1) && pin < SUNXI_GPC(5))
				continue;
			if (pin == SUNXI_GPC(7) || pin == SUNXI_GPC(12))
				continue;
			sunxi_gpio_set_cfgpin(pin, SUNXI_GPC_SDC2);
			sunxi_gpio_set_pull(pin, SUNXI_GPIO_PULL_UP);
			sunxi_gpio_set_drv(pin, 3);
		}
#elif defined(CONFIG_MACH_SUN9I)
		/* SDC2: PC6-PC16 */
		for (pin = SUNXI_GPC(6); pin <= SUNXI_GPC(16); pin++) {
			sunxi_gpio_set_cfgpin(pin, SUNXI_GPC_SDC2);
			sunxi_gpio_set_pull(pin, SUNXI_GPIO_PULL_UP);
			sunxi_gpio_set_drv(pin, 2);
		}
#elif defined(CONFIG_MACH_SUN8I_R528)
                /* SDC2: PC2-PC7 */
                for (pin = SUNXI_GPC(2); pin <= SUNXI_GPC(7); pin++) {
                        sunxi_gpio_set_cfgpin(pin, SUNXI_GPC_SDC2);
                        sunxi_gpio_set_pull(pin, SUNXI_GPIO_PULL_UP);
                        sunxi_gpio_set_drv(pin, 2);
                }
#else
		puts("ERROR: No pinmux setup defined for MMC2!\n");
#endif
		break;

	case 3:
#if defined(CONFIG_MACH_SUN4I) || defined(CONFIG_MACH_SUN7I) || \
    defined(CONFIG_MACH_SUN8I_R40)
		/* SDC3: PI4-PI9 */
		for (pin = SUNXI_GPI(4); pin <= SUNXI_GPI(9); pin++) {
			sunxi_gpio_set_cfgpin(pin, SUNXI_GPI_SDC3);
			sunxi_gpio_set_pull(pin, SUNXI_GPIO_PULL_UP);
			sunxi_gpio_set_drv(pin, 2);
		}
#elif defined(CONFIG_MACH_SUN6I)
		/* SDC3: PC6-PC15, PC24 */
		for (pin = SUNXI_GPC(6); pin <= SUNXI_GPC(15); pin++) {
			sunxi_gpio_set_cfgpin(pin, SUN6I_GPC_SDC3);
			sunxi_gpio_set_pull(pin, SUNXI_GPIO_PULL_UP);
			sunxi_gpio_set_drv(pin, 2);
		}

		sunxi_gpio_set_cfgpin(SUNXI_GPC(24), SUN6I_GPC_SDC3);
		sunxi_gpio_set_pull(SUNXI_GPC(24), SUNXI_GPIO_PULL_UP);
		sunxi_gpio_set_drv(SUNXI_GPC(24), 2);
#endif
		break;

	default:
		printf("sunxi: invalid MMC slot %d for pinmux setup\n", sdc);
		break;
	}
}

int board_mmc_init(struct bd_info *bis)
{
	/*
	 * The BROM always accesses MMC port 0 (typically an SD card), and
	 * most boards seem to have such a slot. The others haven't reported
	 * any problem with unconditionally enabling this in the SPL.
	 */
	if (!IS_ENABLED(CONFIG_UART0_PORT_F)) {
		mmc_pinmux_setup(0);
		if (!sunxi_mmc_init(0))
			return -1;
	}

	if (CONFIG_MMC_SUNXI_SLOT_EXTRA != -1) {
		mmc_pinmux_setup(CONFIG_MMC_SUNXI_SLOT_EXTRA);
		if (!sunxi_mmc_init(CONFIG_MMC_SUNXI_SLOT_EXTRA))
			return -1;
	}

	return 0;
}

#ifdef CONFIG_ENV_MMC_DEVICE_INDEX
int mmc_get_env_dev(void)
{
	switch (sunxi_get_boot_device()) {
	case BOOT_DEVICE_MMC1:
		return 0;
	case BOOT_DEVICE_MMC2:
		return 1;
	default:
		return CONFIG_ENV_MMC_DEVICE_INDEX;
	}
}
#endif
#endif /* CONFIG_MMC */

#ifdef CONFIG_XPL_BUILD

void *board_spl_fit_buffer_addr(ulong fit_size, int sectors, int bl_len)
{
	printf("SPL: board_spl_fit_buffer_addr: fit_size=%lu sectors=%d bl_len=%d -> buffer @ 0x44000000\n",
	       fit_size, sectors, bl_len);
	return (void *)0x44000000;
}

static void sunxi_spl_store_dram_size(phys_addr_t dram_size)
{
	printf("SPL: sunxi_spl_store_dram_size starting...\n");
	struct boot_file_head *spl = get_spl_header(SPL_DT_HEADER_VERSION);

	if (spl == INVALID_SPL_HEADER) {
		printf("SPL: get_spl_header returned INVALID_SPL_HEADER\n");
		return;
	}

	printf("SPL: spl header valid, signature: %.4s\n", spl->spl_signature);
	/* Promote the header version for U-Boot proper, if needed. */
	if (spl->spl_signature[3] < SPL_DRAM_HEADER_VERSION)
		spl->spl_signature[3] = SPL_DRAM_HEADER_VERSION;

	spl->dram_size = dram_size >> 20;
	printf("SPL: header dram_size stored: %u MiB\n", spl->dram_size);
	printf("SPL: sunxi_spl_store_dram_size finished\n");
}

static void status_led_init(void)
{
#if CONFIG_IS_ENABLED(SUNXI_LED_STATUS)
	unsigned int state = IS_ENABLED(CONFIG_SPL_SUNXI_LED_STATUS_ACTIVE_HIGH);
	unsigned int gpio = CONFIG_SPL_SUNXI_LED_STATUS_GPIO;

	gpio_request(gpio, "gpio_led");
	gpio_direction_output(gpio, state);
#endif
}

void sunxi_board_init(void)
{
	int power_failed = 0;

	if (CONFIG_IS_ENABLED(SUNXI_LED_STATUS))
		status_led_init();

#ifdef CONFIG_SY8106A_POWER
	power_failed = sy8106a_set_vout1(CONFIG_SY8106A_VOUT1_VOLT);
#endif

#if defined CONFIG_AXP152_POWER || defined CONFIG_AXP209_POWER || \
	defined CONFIG_AXP221_POWER || defined CONFIG_AXP305_POWER || \
	defined CONFIG_AXP809_POWER || defined CONFIG_AXP818_POWER || \
	defined CONFIG_AXP313_POWER || defined CONFIG_AXP717_POWER || \
	defined CONFIG_AXP803_POWER
	power_failed = axp_init();

	if (IS_ENABLED(CONFIG_AXP_DISABLE_BOOT_ON_POWERON) && !power_failed) {
		u8 boot_reason;

		pmic_bus_read(AXP_POWER_STATUS, &boot_reason);
		if (boot_reason & AXP_POWER_STATUS_ALDO_IN) {
			printf("Power on by plug-in, shutting down.\n");
			pmic_bus_write(0x32, BIT(7));
		}
	}

#ifdef CONFIG_AXP_DCDC1_VOLT
	power_failed |= axp_set_dcdc1(CONFIG_AXP_DCDC1_VOLT);
#endif
#ifdef CONFIG_AXP_DCDC2_VOLT
	power_failed |= axp_set_dcdc2(CONFIG_AXP_DCDC2_VOLT);
#endif
#ifdef CONFIG_AXP_DCDC3_VOLT
	power_failed |= axp_set_dcdc3(CONFIG_AXP_DCDC3_VOLT);
#endif
#ifdef CONFIG_AXP_DCDC4_VOLT
	power_failed |= axp_set_dcdc4(CONFIG_AXP_DCDC4_VOLT);
#endif
#ifdef CONFIG_AXP_DCDC5_VOLT
	power_failed |= axp_set_dcdc5(CONFIG_AXP_DCDC5_VOLT);
#endif

	/*
	 * On AXP717, CLDO3 supplies VCC-CARD (SD card power). The BROM
	 * enables it to read SPL, but after jumping to U-Boot SPL the
	 * regulator stays at its power-on default which may not be 3.3V
	 * on all boards/revisions. Explicitly configure it here so the
	 * SPL MMC driver can reliably access the SD card.
	 *
	 * AXP717 CLDO3 registers (from axp_regulator.c):
	 *   0x91 bit 2 = enable
	 *   0x9d bits[4:0] = voltage (500mV + val*100mV)
	 */
	if (IS_ENABLED(CONFIG_AXP717_POWER) &&
	    IS_ENABLED(CONFIG_MACH_SUN55I_A523) && !power_failed) {
		u8 reg_val;

		/* Set CLDO3 voltage to 3300mV: (3300-500)/100 = 0x1c */
		power_failed |= pmic_bus_read(0x9d, &reg_val);
		if (!power_failed) {
			reg_val = (reg_val & ~0x1f) | 0x1c;
			power_failed |= pmic_bus_write(0x9d, reg_val);
		}

		/* Enable CLDO3 (set bit 2 in register 0x91) */
		if (!power_failed) {
			power_failed |= pmic_bus_read(0x91, &reg_val);
			if (!power_failed && !(reg_val & BIT(2))) {
				printf("PMIC: enabling CLDO3 (VCC-CARD) at 3.3V\n");
				power_failed |= pmic_bus_write(0x91,
							      reg_val | BIT(2));
				/* Wait for LDO to stabilize */
				mdelay(50);
			}
		}
	}

#ifdef CONFIG_AXP_ALDO1_VOLT
	power_failed |= axp_set_aldo1(CONFIG_AXP_ALDO1_VOLT);
#endif
#ifdef CONFIG_AXP_ALDO2_VOLT
	power_failed |= axp_set_aldo2(CONFIG_AXP_ALDO2_VOLT);
#endif
#ifdef CONFIG_AXP_ALDO3_VOLT
	power_failed |= axp_set_aldo3(CONFIG_AXP_ALDO3_VOLT);
#endif
#ifdef CONFIG_AXP_ALDO4_VOLT
	power_failed |= axp_set_aldo4(CONFIG_AXP_ALDO4_VOLT);
#endif

#ifdef CONFIG_AXP_DLDO1_VOLT
	power_failed |= axp_set_dldo(1, CONFIG_AXP_DLDO1_VOLT);
	power_failed |= axp_set_dldo(2, CONFIG_AXP_DLDO2_VOLT);
#endif
#ifdef CONFIG_AXP_DLDO3_VOLT
	power_failed |= axp_set_dldo(3, CONFIG_AXP_DLDO3_VOLT);
	power_failed |= axp_set_dldo(4, CONFIG_AXP_DLDO4_VOLT);
#endif
#ifdef CONFIG_AXP_ELDO1_VOLT
	power_failed |= axp_set_eldo(1, CONFIG_AXP_ELDO1_VOLT);
	power_failed |= axp_set_eldo(2, CONFIG_AXP_ELDO2_VOLT);
	power_failed |= axp_set_eldo(3, CONFIG_AXP_ELDO3_VOLT);
#endif

#ifdef CONFIG_AXP_FLDO1_VOLT
	power_failed |= axp_set_fldo(1, CONFIG_AXP_FLDO1_VOLT);
	power_failed |= axp_set_fldo(2, CONFIG_AXP_FLDO2_VOLT);
	power_failed |= axp_set_fldo(3, CONFIG_AXP_FLDO3_VOLT);
#endif

#if defined CONFIG_AXP809_POWER || defined CONFIG_AXP818_POWER
	power_failed |= axp_set_sw(IS_ENABLED(CONFIG_AXP_SW_ON));
#endif
#endif	/* CONFIG_AXPxxx_POWER */
	printf("DRAM:");
	gd->ram_size = sunxi_dram_init();
	printf(" %d MiB\n", (int)(gd->ram_size >> 20));
	if (!gd->ram_size)
		hang();

	sunxi_spl_store_dram_size(gd->ram_size);

	printf("SPL: probing DRAM @ 0x40800000...\n");
	volatile u32 *p40 = (volatile u32 *)0x40800000;
	*p40 = 0x12345678;
	printf("SPL: 0x40800000 readback: 0x%08x\n", *p40);

	printf("SPL: probing DRAM @ 0x4fe00000 (STACK_R)...\n");
	volatile u32 *p4fe = (volatile u32 *)0x4fe00000;
	*p4fe = 0x87654321;
	printf("SPL: 0x4fe00000 readback: 0x%08x\n", *p4fe);

	printf("SPL: probing DRAM @ 0x4ff80000 (BSS)...\n");
	volatile u32 *pbss = (volatile u32 *)0x4ff80000;
	*pbss = 0xdeadbeef;
	printf("SPL: 0x4ff80000 readback: 0x%08x\n", *pbss);

	/*
	 * Only clock up the CPU to full speed if we are reasonably
	 * assured it's being powered with suitable core voltage
	 */
	if (!power_failed) {
		printf("SPL: calling clock_set_pll1...\n");
		clock_set_pll1(get_board_sys_clk());
		printf("SPL: clock_set_pll1 done\n");
	} else {
		printf("Failed to set core voltage! Can't set CPU frequency\n");
	}
	printf("SPL: sunxi_board_init completed!\n");
}
#endif /* CONFIG_XPL_BUILD */

#ifdef CONFIG_USB_GADGET
int g_dnl_board_usb_cable_connected(void)
{
	struct udevice *dev;
	struct phy phy;
	int ret;

	ret = uclass_get_device(UCLASS_USB_GADGET_GENERIC, 0, &dev);
	if (ret) {
		pr_err("%s: Cannot find USB device\n", __func__);
		return ret;
	}

	ret = generic_phy_get_by_name(dev, "usb", &phy);
	if (ret) {
		pr_err("failed to get %s USB PHY\n", dev->name);
		return ret;
	}

	ret = generic_phy_init(&phy);
	if (ret) {
		pr_debug("failed to init %s USB PHY\n", dev->name);
		return ret;
	}

	return sun4i_usb_phy_vbus_detect(&phy);
}
#endif /* CONFIG_USB_GADGET */

#ifdef CONFIG_SERIAL_TAG
void get_board_serial(struct tag_serialnr *serialnr)
{
	char *serial_string;
	unsigned long long serial;

	serial_string = env_get("serial#");

	if (serial_string) {
		serial = simple_strtoull(serial_string, NULL, 16);

		serialnr->high = (unsigned int) (serial >> 32);
		serialnr->low = (unsigned int) (serial & 0xffffffff);
	} else {
		serialnr->high = 0;
		serialnr->low = 0;
	}
}
#endif

/*
 * Check the SPL header for the "sunxi" variant. If found: parse values
 * that might have been passed by the loader ("fel" utility), and update
 * the environment accordingly.
 */
static void parse_spl_header(const uint32_t spl_addr)
{
	struct boot_file_head *spl = get_spl_header(SPL_ENV_HEADER_VERSION);

	if (spl == INVALID_SPL_HEADER)
		return;

	if (!spl->fel_script_address)
		return;

	if (spl->fel_uEnv_length != 0) {
		/*
		 * data is expected in uEnv.txt compatible format, so "env
		 * import -t" the string(s) at fel_script_address right away.
		 */
		himport_r(&env_htab, (char *)(uintptr_t)spl->fel_script_address,
			  spl->fel_uEnv_length, '\n', H_NOCLEAR, 0, 0, NULL);
		return;
	}
	/* otherwise assume .scr format (mkimage-type script) */
	env_set_hex("fel_scriptaddr", spl->fel_script_address);
}

static bool get_unique_sid(unsigned int *sid)
{
	if (sunxi_get_sid(sid) != 0)
		return false;

	if (!sid[0])
		return false;

	/*
	 * The single words 1 - 3 of the SID have quite a few bits
	 * which are the same on many models, so we take a crc32
	 * of all 3 words, to get a more unique value.
	 *
	 * Note we only do this on newer SoCs as we cannot change
	 * the algorithm on older SoCs since those have been using
	 * fixed mac-addresses based on only using word 3 for a
	 * long time and changing a fixed mac-address with an
	 * u-boot update is not good.
	 */
#if !defined(CONFIG_MACH_SUN4I) && !defined(CONFIG_MACH_SUN5I) && \
    !defined(CONFIG_MACH_SUN6I) && !defined(CONFIG_MACH_SUN7I) && \
    !defined(CONFIG_MACH_SUN8I_A23) && !defined(CONFIG_MACH_SUN8I_A33)
	sid[3] = crc32(0, (unsigned char *)&sid[1], 12);
#endif

	/* Ensure the NIC specific bytes of the mac are not all 0 */
	if ((sid[3] & 0xffffff) == 0)
		sid[3] |= 0x800000;

	return true;
}

/*
 * Note this function gets called multiple times.
 * It must not make any changes to env variables which already exist.
 */
static void setup_environment(const void *fdt)
{
	char serial_string[17] = { 0 };
	unsigned int sid[4];
	uint8_t mac_addr[6];
	char ethaddr[16];
	int i;

	if (!get_unique_sid(sid))
		return;

	for (i = 0; i < 4; i++) {
		sprintf(ethaddr, "ethernet%d", i);
		if (!fdt_get_alias(fdt, ethaddr))
			continue;

		if (i == 0)
			strcpy(ethaddr, "ethaddr");
		else
			sprintf(ethaddr, "eth%daddr", i);

		if (env_get(ethaddr))
			continue;

		/* Non OUI / registered MAC address */
		mac_addr[0] = (i << 4) | 0x02;
		mac_addr[1] = (sid[0] >>  0) & 0xff;
		mac_addr[2] = (sid[3] >> 24) & 0xff;
		mac_addr[3] = (sid[3] >> 16) & 0xff;
		mac_addr[4] = (sid[3] >>  8) & 0xff;
		mac_addr[5] = (sid[3] >>  0) & 0xff;

		eth_env_set_enetaddr(ethaddr, mac_addr);
	}

	if (!env_get("serial#")) {
		snprintf(serial_string, sizeof(serial_string),
			"%08x%08x", sid[0], sid[3]);

		env_set("serial#", serial_string);
	}
}

int misc_init_r(void)
{
	const char *spl_dt_name;
	uint boot;

	env_set("fel_booted", NULL);
	env_set("fel_scriptaddr", NULL);
	env_set("mmc_bootdev", NULL);

	boot = sunxi_get_boot_device();
	/* determine if we are running in FEL mode */
	if (boot == BOOT_DEVICE_BOARD) {
		env_set("fel_booted", "1");
		parse_spl_header(SPL_ADDR);
	/* or if we booted from MMC, and which one */
	} else if (boot == BOOT_DEVICE_MMC1) {
		env_set("mmc_bootdev", "0");
	} else if (boot == BOOT_DEVICE_MMC2) {
		env_set("mmc_bootdev", "1");
	}

	/* Set fdtfile to match the FIT configuration chosen in SPL. */
	spl_dt_name = get_spl_dt_name();
	if (spl_dt_name) {
		const char *prefix = "";
		char str[64];

		if (IS_ENABLED(CONFIG_ARM64) && !IS_ENABLED(CONFIG_OF_UPSTREAM))
			prefix = "allwinner/";

		snprintf(str, sizeof(str), "%s%s.dtb", prefix, spl_dt_name);
		env_set("fdtfile", str);
	}

	setup_environment(gd->fdt_blob);

	return 0;
}

int board_late_init(void)
{
#ifdef CONFIG_USB_ETHER
	usb_ether_init();
#endif

#if defined(CONFIG_MACH_SUN60I_A733) && !defined(CONFIG_SPL_BUILD)
#if CONFIG_IS_ENABLED(DM_REGULATOR)
	/* Ensure all critical PMIC rails are active */
	struct udevice *reg;
	regulator_autoset_by_name("vcc-pk", &reg);
	regulator_autoset_by_name("vcc-pj", &reg);
	regulator_autoset_by_name("vcc-1v8-audio", &reg);
	regulator_autoset_by_name("vcc-ephy", &reg);
	regulator_autoset_by_name("vcc-gmac-io", &reg);
#endif

#if CONFIG_IS_ENABLED(DM_THERMAL)
	/* Read and display CPU temperature */
	struct udevice *thermal_dev;
	int temp;

	if (!uclass_first_device_err(UCLASS_THERMAL, &thermal_dev)) {
		if (!thermal_get_temp(thermal_dev, &temp))
			printf("CPU Temperature: %d C\n", temp);
	}
#endif
#endif

	return 0;
}

static void bluetooth_dt_fixup(void *blob)
{
	/* Some devices ship with a Bluetooth controller default address.
	 * Set a valid address through the device tree.
	 */
	uchar tmp[ETH_ALEN], bdaddr[ETH_ALEN];
	unsigned int sid[4];
	int i;

	if (!CONFIG_BLUETOOTH_DT_DEVICE_FIXUP[0])
		return;

	if (eth_env_get_enetaddr("bdaddr", tmp)) {
		/* Convert between the binary formats of the corresponding stacks */
		for (i = 0; i < ETH_ALEN; ++i)
			bdaddr[i] = tmp[ETH_ALEN - i - 1];
	} else {
		if (!get_unique_sid(sid))
			return;

		bdaddr[0] = ((sid[3] >>  0) & 0xff) ^ 1;
		bdaddr[1] = (sid[3] >>  8) & 0xff;
		bdaddr[2] = (sid[3] >> 16) & 0xff;
		bdaddr[3] = (sid[3] >> 24) & 0xff;
		bdaddr[4] = (sid[0] >>  0) & 0xff;
		bdaddr[5] = 0x02;
	}

	do_fixup_by_compat(blob, CONFIG_BLUETOOTH_DT_DEVICE_FIXUP,
			   "local-bd-address", bdaddr, ETH_ALEN, 1);
}

#define PINEPHONE_LIS3MDL_I2C_ADDR	0x1e
#define PINEPHONE_LIS3MDL_I2C_BUS	1 /* I2C1 */

static void board_dt_fixup(void *blob)
{
	struct udevice *bus, *dev;

	if (IS_ENABLED(CONFIG_PINEPHONE_DT_SELECTION) &&
	    !fdt_node_check_compatible(blob, 0, "pine64,pinephone-1.2")) {
		if (!uclass_get_device_by_seq(UCLASS_I2C,
					      PINEPHONE_LIS3MDL_I2C_BUS,
					      &bus)) {
			dm_i2c_probe(bus, PINEPHONE_LIS3MDL_I2C_ADDR, 0, &dev);
			fdt_set_status_by_compatible(blob, "st,lis3mdl-magn",
				dev ? FDT_STATUS_OKAY  : FDT_STATUS_DISABLED);
			fdt_set_status_by_compatible(blob, "voltafield,af8133j",
				dev ? FDT_STATUS_DISABLED : FDT_STATUS_OKAY);
		}
	}
}

int ft_board_setup(void *blob, struct bd_info *bd)
{
	int __maybe_unused r;

	/*
	 * Call setup_environment and fdt_fixup_ethernet again
	 * in case the boot fdt has ethernet aliases the u-boot
	 * copy does not have.
	 */
	setup_environment(blob);
	fdt_fixup_ethernet(blob);

	bluetooth_dt_fixup(blob);
	board_dt_fixup(blob);

#ifdef CONFIG_VIDEO_DT_SIMPLEFB
	r = sunxi_simplefb_setup(blob);
	if (r)
		return r;
#endif
	return 0;
}

#ifdef CONFIG_SPL_LOAD_FIT
static void set_spl_dt_name(const char *name)
{
	struct boot_file_head *spl = get_spl_header(SPL_ENV_HEADER_VERSION);

	if (spl == INVALID_SPL_HEADER)
		return;

	/* Promote the header version for U-Boot proper, if needed. */
	if (spl->spl_signature[3] < SPL_DT_HEADER_VERSION)
		spl->spl_signature[3] = SPL_DT_HEADER_VERSION;

	strcpy((char *)&spl->string_pool, name);
	spl->dt_name_offset = offsetof(struct boot_file_head, string_pool);
}

int board_fit_config_name_match(const char *name)
{
	const char *best_dt_name = get_spl_dt_name();
	int ret;

#ifdef CONFIG_DEFAULT_DEVICE_TREE
	if (best_dt_name == NULL)
		best_dt_name = CONFIG_DEFAULT_DEVICE_TREE;
#endif

	printf("SPL: board_fit_config_name_match: name='%s' best_dt_name='%s'\n",
	       name ? name : "NULL", best_dt_name ? best_dt_name : "NULL");

	if (best_dt_name == NULL) {
		/* No DT name was provided, so accept the first config. */
		return 0;
	}
#ifdef CONFIG_PINE64_DT_SELECTION
	if (strstr(best_dt_name, "-pine64-plus")) {
		/* Differentiate the Pine A64 boards by their DRAM size. */
		if (gd->ram_size == SZ_512M)
			best_dt_name = "sun50i-a64-pine64";
	}
#endif
#ifdef CONFIG_PINEPHONE_DT_SELECTION
	if (strstr(best_dt_name, "-pinephone")) {
		/* Differentiate the PinePhone revisions by GPIO inputs. */
		prcm_apb0_enable(PRCM_APB0_GATE_PIO);
		sunxi_gpio_set_pull(SUNXI_GPL(6), SUNXI_GPIO_PULL_UP);
		sunxi_gpio_set_cfgpin(SUNXI_GPL(6), SUNXI_GPIO_INPUT);
		udelay(100);

		/* PL6 is pulled low by the modem on v1.2. */
		if (gpio_get_value(SUNXI_GPL(6)) == 0)
			best_dt_name = "sun50i-a64-pinephone-1.2";
		else
			best_dt_name = "sun50i-a64-pinephone-1.1";

		sunxi_gpio_set_cfgpin(SUNXI_GPL(6), SUNXI_GPIO_DISABLE);
		sunxi_gpio_set_pull(SUNXI_GPL(6), SUNXI_GPIO_PULL_DISABLE);
		prcm_apb0_disable(PRCM_APB0_GATE_PIO);
	}
#endif

	ret = strcmp(name, best_dt_name);

	/*
	 * If one of the FIT configurations matches the most accurate DT name,
	 * update the SPL header to provide that DT name to U-Boot proper.
	 */
	if (ret == 0)
		set_spl_dt_name(best_dt_name);

	return ret;
}
#endif /* CONFIG_SPL_LOAD_FIT */
