// SPDX-License-Identifier: GPL-2.0+

#include <asm/io.h>
#include <asm/arch/cpu.h>
#include <asm/arch/clock.h>
#include <asm/arch/prcm.h>
#include <linux/delay.h>

#ifndef SUNXI_CPU_PLL_CFG_BASE
#define SUNXI_CPU_PLL_CFG_BASE 0
#endif

#ifdef CONFIG_XPL_BUILD
#ifdef CONFIG_MACH_SUN60I_A733
#define SUNXI_CCU_MBUS_MAT_CLK_GATING		(SUNXI_CCM_BASE + 0x05c0UL)
#define SUNXI_CCU_MBUS_MAT_CLK_GATING_VAL	0x110103ffU
#define SUNXI_CCU_MBUS_MAT_CLK_ENABLE		(SUNXI_CCM_BASE + 0x05e0UL)
#define SUNXI_CCU_MBUS_MAT_CLK_ENABLE_VAL	0xf1055803U
#define SUNXI_CCU_MBUS_MASTER_GATE		(SUNXI_CCM_BASE + 0x05e4UL)
#define SUNXI_CCU_MBUS_MASTER_GATE_VAL		0x0007ffffU

#define SUNXI_SYSCTRL_RESCAL_CTRL		(SUNXI_SRAMC_BASE + 0x0160UL)
#define SUNXI_SYSCTRL_RESCAL_CTRL_EN		BIT(8)
#define SUNXI_SYSCTRL_RESCAL_CAL		(SUNXI_SRAMC_BASE + 0x0168UL)
#define SUNXI_SYSCTRL_RESCAL_CAL_MASK		0xffU

/*
 * On A733, the BROM configures PLL_PERIPH0 and APB/AXI root dividers before
 * SPL entry, while TF-A BL31 (sunxi_security_setup) configures TrustZone SPC,
 * CCU/PRCM security switches, and IOMMU bypass before Non-Secure EL2 entry.
 * SPL only needs to ungate the MBUS matrix clocks and enable SYSCTRL ZQ/pad
 * resistor calibration prior to LPDDR5 training.
 */
static void sunxi_a733_bus_and_rescal_init(void)
{
	writel(SUNXI_CCU_MBUS_MAT_CLK_GATING_VAL,
	       (void __iomem *)SUNXI_CCU_MBUS_MAT_CLK_GATING);
	writel(SUNXI_CCU_MBUS_MAT_CLK_ENABLE_VAL,
	       (void __iomem *)SUNXI_CCU_MBUS_MAT_CLK_ENABLE);
	writel(SUNXI_CCU_MBUS_MASTER_GATE_VAL,
	       (void __iomem *)SUNXI_CCU_MBUS_MASTER_GATE);

	setbits_le32((void __iomem *)SUNXI_SYSCTRL_RESCAL_CTRL,
		     SUNXI_SYSCTRL_RESCAL_CTRL_EN);
	clrbits_le32((void __iomem *)SUNXI_SYSCTRL_RESCAL_CAL,
		     SUNXI_SYSCTRL_RESCAL_CAL_MASK);
}
#endif

void clock_init_safe(void)
{
#ifdef CONFIG_MACH_SUN60I_A733
	sunxi_a733_bus_and_rescal_init();
#else
	void *const ccm = (void *)SUNXI_CCM_BASE;
	void *const prcm = (void *)SUNXI_PRCM_BASE;

	if (IS_ENABLED(CONFIG_MACH_SUN50I_H616))
		setbits_le32(prcm + CCU_PRCM_SYS_PWROFF_GATING, 0x10);
	if (IS_ENABLED(CONFIG_MACH_SUN55I_A523))
		setbits_le32(prcm + CCU_PRCM_SYS_PWROFF_GATING, 0x200);
	udelay(1);

	if (IS_ENABLED(CONFIG_MACH_SUN50I_H616) ||
	    IS_ENABLED(CONFIG_MACH_SUN55I_A523))
		setbits_le32(prcm + CCU_PRCM_RES_CAL_CTRL, 2);
	udelay(1);

	if (IS_ENABLED(CONFIG_MACH_SUN50I_H616) ||
	    IS_ENABLED(CONFIG_MACH_SUN50I_H6) ||
	    IS_ENABLED(CONFIG_MACH_SUN55I_A523)) {
		clrbits_le32(prcm + CCU_PRCM_RES_CAL_CTRL, 1);
		udelay(1);
		setbits_le32(prcm + CCU_PRCM_RES_CAL_CTRL, 1);
	}

	if (IS_ENABLED(CONFIG_MACH_SUN50I_H6)) {
		/* set key field for ldo enable */
		setbits_le32(prcm + CCU_PRCM_PLL_LDO_CFG, 0xA7000000);
		/* set PLL VDD LDO output to 1.14 V */
		setbits_le32(prcm + CCU_PRCM_PLL_LDO_CFG, 0x60000);
	}

	clock_set_pll1(408000000);

	writel(CCM_PLL6_DEFAULT, ccm + CCU_H6_PLL6_CFG);
	while (!(readl(ccm + CCU_H6_PLL6_CFG) & CCM_PLL_LOCK))
		;

	if (!IS_ENABLED(CONFIG_MACH_SUN55I_A523))
		clrsetbits_le32(ccm + CCU_H6_CPU_AXI_CFG,
				CCM_CPU_AXI_APB_MASK | CCM_CPU_AXI_AXI_MASK,
				CCM_CPU_AXI_DEFAULT_FACTORS);

	writel(CCM_PSI_AHB1_AHB2_DEFAULT, ccm + CCU_H6_PSI_AHB1_AHB2_CFG);
#ifdef CCM_AHB3_DEFAULT
	writel(CCM_AHB3_DEFAULT, ccm + CCU_H6_AHB3_CFG);
#endif
	writel(CCM_APB1_DEFAULT, ccm + CCU_H6_APB1_CFG);

	/*
	 * The mux and factor are set, but the clock will be enabled in
	 * DRAM initialization code.
	 */
	if (IS_ENABLED(CONFIG_MACH_SUN55I_A523)) {
		writel(MBUS_RESET, ccm + CCU_H6_MBUS_CFG);
		udelay(1);
		writel(MBUS_UPDATE | MBUS_CLK_SRC_OSCM24 | MBUS_CLK_M(4),
		       ccm + CCU_H6_MBUS_CFG);
	} else {
		writel(MBUS_CLK_SRC_PLL6X2 | MBUS_CLK_M(3),
		       ccm + CCU_H6_MBUS_CFG);
	}
#endif
}

void clock_init_uart(void)
{
	void *const ccm = (void *)SUNXI_CCM_BASE;

	if (IS_ENABLED(CONFIG_MACH_SUN60I_A733)) {
		/* A733: APB_UART clock source is OSC24M / 1 */
		writel(APB2_CLK_SRC_OSC24M |
		       APB2_CLK_RATE_N_1 |
		       APB2_CLK_RATE_M(1),
		       ccm + CCU_A733_APB_UART_CFG);

		/* Deassert reset and enable clock gate for console UART */
		setbits_le32(ccm + CCU_A733_UART_BGR_REG(CONFIG_CONS_INDEX - 1),
			     BIT(RESET_SHIFT) | BIT(GATE_SHIFT));
		return;
	}

	/* uart clock source is apb2 */
	writel(APB2_CLK_SRC_OSC24M|
	       APB2_CLK_RATE_N_1|
	       APB2_CLK_RATE_M(1),
	       ccm + CCU_H6_APB2_CFG);

	/* open the clock for uart */
	setbits_le32(ccm + CCU_H6_UART_GATE_RESET,
		     1 << (CONFIG_CONS_INDEX - 1));

	/* deassert uart reset */
	setbits_le32(ccm + CCU_H6_UART_GATE_RESET,
		     1 << (RESET_SHIFT + CONFIG_CONS_INDEX - 1));
}

static bool has_pll_output_gate(void)
{
	return (IS_ENABLED(CONFIG_SUNXI_GEN_NCAT2) ||
		IS_ENABLED(CONFIG_MACH_SUN50I_H616) ||
		IS_ENABLED(CONFIG_MACH_SUN50I_A133));
}

/* A shared routine to program the CPU PLLs for H6, H616, T113, A523 */
static void clock_set_pll(u32 *reg, unsigned int n)
{
	u32 val = readl(reg);

	/* clear the lock enable bit */
	val &= ~CCM_PLL_LOCK_EN;
	writel(val, reg);

	/* gate the output on the newer SoCs */
	if (has_pll_output_gate()) {
		val &= ~CCM_PLL_OUT_EN;
		writel(val, reg);
	}

	val &= ~(CCM_PLL1_CTRL_N_MASK | GENMASK(3, 0) | GENMASK(21, 16));
	val |= CCM_PLL1_CTRL_N(n);
	writel(val, reg);			/* program parameter */

	val |= CCM_PLL_CTRL_EN;
	if (IS_ENABLED(CONFIG_SUNXI_GEN_NCAT2))
		val |= CCM_PLL_LDO_EN;
	writel(val, reg);			/* enable PLL */

	val |= CCM_PLL_LOCK_EN;
	if (IS_ENABLED(CONFIG_MACH_SUN55I_A523))
		val |= CCM_PLL1_UPDATE;
	writel(val, reg);			/* start locking process */

	while (!(readl(reg) & CCM_PLL_LOCK)) {	/* wait for lock bit */
	}
	udelay(20);				/* wait as per manual */

	/* un-gate the output on the newer SoCs */
	if (has_pll_output_gate()) {
		val |= CCM_PLL_OUT_EN;
		writel(val, reg);
	}
}

/* Program the PLLs for both clusters plus the DSU. */
static void clock_a523_set_cpu_plls(unsigned int n_factor)
{
	void *const cpc = (void *)SUNXI_CPU_PLL_CFG_BASE;
	u32 val;

	val = CPU_CLK_SRC_HOSC | CPU_CLK_CTRL_P(0) |
	       CPU_CLK_APB_DIV(4) | CPU_CLK_PERI_DIV(2) |
	       CPU_CLK_AXI_DIV(2);

	/* Switch CPU clock source to 24MHz HOSC while changing the PLL */
	writel(val, cpc + CPC_CPUA_CLK_REG);
	writel(val, cpc + CPC_CPUB_CLK_REG);
	udelay(20);
	writel(CPU_CLK_SRC_HOSC | CPU_CLK_CTRL_P(0),
	       cpc + CPC_DSU_CLK_REG);
	udelay(20);

	clock_set_pll(cpc + CPC_CPUA_PLL_CTRL, n_factor);
	clock_set_pll(cpc + CPC_CPUB_PLL_CTRL, n_factor);
	clock_set_pll(cpc + CPC_DSU_PLL_CTRL, n_factor);

	/* Switch CPU clock source to the CPU PLL */
	clrsetbits_le32(cpc + CPC_CPUA_CLK_REG, CPU_CLK_SRC_HOSC,
			CPU_CLK_SRC_CPUPLL);
	clrsetbits_le32(cpc + CPC_CPUB_CLK_REG, CPU_CLK_SRC_HOSC,
			CPU_CLK_SRC_CPUPLL);
	clrsetbits_le32(cpc + CPC_DSU_CLK_REG, CPU_CLK_SRC_HOSC,
			CPU_CLK_SRC_CPUPLL);
}

static void clock_h6_set_cpu_pll(unsigned int n_factor)
{
	void *const ccm = (void *)SUNXI_CCM_BASE;
	u32 val;

	/* Switch CPU clock source to 24MHz HOSC while changing the PLL */
	val = readl(ccm + CCU_H6_CPU_AXI_CFG);
	val &= ~CCM_CPU_AXI_MUX_MASK;
	val |= CCM_CPU_AXI_MUX_OSC24M;
	writel(val, ccm + CCU_H6_CPU_AXI_CFG);

	clock_set_pll(ccm + CCU_H6_PLL1_CFG, n_factor);

	/* Switch CPU clock source to the CPU PLL */
	val = readl(ccm + CCU_H6_CPU_AXI_CFG);
	val &= ~CCM_CPU_AXI_MUX_MASK;
	val |= CCM_CPU_AXI_MUX_PLL_CPUX;
	writel(val, ccm + CCU_H6_CPU_AXI_CFG);
}

void clock_set_pll1(unsigned int clk)
{
	/* Do not support clocks < 288MHz as they need factor P */
	if (clk < 288000000)
		clk = 288000000;

	clk /= 24000000;

	/* BROM and TF-A configure cluster PLLs on A733 */
	if (IS_ENABLED(CONFIG_MACH_SUN60I_A733))
		return;

	if (IS_ENABLED(CONFIG_MACH_SUN55I_A523))
		clock_a523_set_cpu_plls(clk);
	else
		clock_h6_set_cpu_pll(clk);
}

int clock_twi_onoff(int port, int state)
{
	void *const ccm = (void *)SUNXI_CCM_BASE;
	void *const prcm = (void *)SUNXI_PRCM_BASE;
	u32 value, *ptr;
	int shift;

	value = BIT(GATE_SHIFT) | BIT (RESET_SHIFT);

	if (port == 5) {
		shift = 0;
		ptr = prcm + CCU_PRCM_I2C_GATE_RESET;
	} else if (IS_ENABLED(CONFIG_MACH_SUN60I_A733)) {
		shift = 0;
		ptr = ccm + CCU_A733_I2C_BGR_REG(port);
	} else {
		shift = port;
		ptr = ccm + CCU_H6_I2C_GATE_RESET;
	}

	/* set the apb clock gate and reset for twi */
	if (state)
		setbits_le32(ptr, value << shift);
	else
		clrbits_le32(ptr, value << shift);

	return 0;
}
#endif /* CONFIG_XPL_BUILD */

/* PLL_PERIPH0 clock, used by the MMC driver */
unsigned int clock_get_pll6(void)
{
	void *const ccm = (void *)SUNXI_CCM_BASE;
	int div1, div2, m, n;
	u32 rval;

	if (IS_ENABLED(CONFIG_MACH_SUN60I_A733))
		rval = readl(ccm + CCU_A733_PLL6_CFG);
	else
		rval = readl(ccm + CCU_H6_PLL6_CFG);

	n = ((rval & CCM_PLL6_CTRL_N_MASK) >> CCM_PLL6_CTRL_N_SHIFT) + 1;
	div2 = ((rval & CCM_PLL6_CTRL_DIV2_MASK) >>
		CCM_PLL6_CTRL_DIV2_SHIFT) + 1;

	if (IS_ENABLED(CONFIG_MACH_SUN60I_A733)) {
		div1 = ((rval & CCM_A733_PLL6_CTRL_P0_MASK) >>
			CCM_A733_PLL6_CTRL_P0_SHIFT) + 1;
	} else if (IS_ENABLED(CONFIG_SUNXI_GEN_NCAT2)) {
		div1 = ((rval & CCM_PLL6_CTRL_P0_MASK) >>
			CCM_PLL6_CTRL_P0_SHIFT) + 1;
	} else {
		div1 = ((rval & CCM_PLL6_CTRL_DIV1_MASK) >>
			CCM_PLL6_CTRL_DIV1_SHIFT) + 1;
	}

	/*
	 * The factors encoded in the register describe the doubled clock
	 * frequency, expect for the H6, where it's the quadrupled frequency.
	 * Compensate for that here.
	 */
	if (IS_ENABLED(CONFIG_MACH_SUN50I_H6))
		m = 4;
	else
		m = 2;

	return 24000000U * n / m / div1 / div2;
}

#ifndef CONFIG_XPL_BUILD
#ifdef CONFIG_MACH_SUN60I_A733
#define A733_RTC_XO_CTRL1_OFF		0x016c
#define A733_RTC_XO_CLK_OUT_EN		(BIT(9) | BIT(8) | BIT(5) | BIT(4))
#define A733_CCU_MBUS_CLK_REG		0x0574
#define A733_CCU_MBUS_MAT_CLK_GATE_REG	0x0580
#define A733_CCU_MBUS_MAT_SERDES_EN	(BIT(31) | BIT(30) | BIT(25) | BIT(24))
#define A733_CCU_MBUS_SERDES_GATE_REG	0x0584
#define A733_CCU_MBUS_SERDES_CFG_REG	0x0588
#define A733_CCU_MBUS_CLK_CFG0_REG	0x058c
#define A733_CCU_MBUS_CLK_CFG1_REG	0x05b4
#define A733_CCU_PCIE_AXI_SLV_REG	0x1384
#define A733_CCU_SERDES_PHY_CFG_REG	0x13c0
#define A733_CCU_CM_PCIE0_MODE_REG	0x1b28

#define A733_SYS_PLL_CTRL_REG		0x02001540UL
#define A733_SYS_PLL_BGR_REG		0x0200154cUL
#define A733_NSI_BASE			0x02020000UL
#define A733_NSI_PORT_STRIDE		0x200UL
#define A733_NSI_NUM_PORTS		16
#define A733_PCIE_ITS_AXI_REG		0x08868020UL

/*
 * Configure SoC-level RTC DCXO feed, CCU muxes/dividers, MBUS/SYS_PLL gates,
 * PCIe ITS AXI bridge, and NSI QoS not yet modeled by U-Boot's gate-only
 * sunxi CCU/RTC/NSI drivers.
 */
void sunxi_a733_pcie_soc_init(void)
{
	void *const rtc = (void *)(ulong)SUNXI_RTC_BASE;
	void *const ccm = (void *)(ulong)SUNXI_CCM_BASE;
	int p;

	/* Enable RTC DCXO clock feed to Combo PHY without clobbering siblings */
	setbits_le32(rtc + A733_RTC_XO_CTRL1_OFF, A733_RTC_XO_CLK_OUT_EN);

	/* Configure PCIe/SerDes CCU muxes/dividers and MBUS/NSI gating */
	writel(0x82000000, ccm + A733_CCU_PCIE_AXI_SLV_REG); /* 400 MHz */
	writel(0x81000005, ccm + A733_CCU_SERDES_PHY_CFG_REG); /* 100 MHz */
	writel(0x00020001, ccm + A733_CCU_CM_PCIE0_MODE_REG);
	writel(0x00010002, ccm + A733_CCU_MBUS_CLK_REG);
	setbits_le32(ccm + A733_CCU_MBUS_MAT_CLK_GATE_REG,
		     A733_CCU_MBUS_MAT_SERDES_EN);
	writel(0x00010001, ccm + A733_CCU_MBUS_SERDES_GATE_REG);
	writel(0x81000000, ccm + A733_CCU_MBUS_SERDES_CFG_REG);
	setbits_le32(ccm + A733_CCU_MBUS_CLK_CFG0_REG, 0x00010007);
	setbits_le32(ccm + A733_CCU_MBUS_CLK_CFG1_REG, 0x00010007);

	writel(0xc0000000, (void *)(ulong)A733_SYS_PLL_CTRL_REG);
	writel(0x00010001, (void *)(ulong)A733_SYS_PLL_BGR_REG);

	/* Configure NSI interconnect QoS & bandwidth across ports 0..15 */
	for (p = 0; p < A733_NSI_NUM_PORTS; p++) {
		void *port = (void *)(ulong)(A733_NSI_BASE +
					     A733_NSI_PORT_STRIDE * p);

		writel(0x00000001, port + 0x0c);
		writel(0x00000001, port + 0x18);
		writel(0x000003e8, port + 0x28);
		writel(0x000000ff, port + 0x2c);
		writel(0x00000001, port + 0xcc);
	}

	writel(0x00ff00ff, (void *)(ulong)A733_PCIE_ITS_AXI_REG);
}
#endif
#endif
