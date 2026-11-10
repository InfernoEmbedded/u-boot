// SPDX-License-Identifier: GPL-2.0+
/*
 * Allwinner sun60i A733 / T736 DRAM Controller and Cadence DDR PHY driver
 *
 * (C) Copyright 2026 Alastair D'Silva <alastair@d-silva.org>
 */

#include <init.h>
#include <log.h>
#include <stdio.h>
#include <asm/arch/dram.h>
#include <asm/barriers.h>
#include <asm/io.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/sizes.h>
#include <linux/types.h>

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

#include "pmu/sunxi_pmu_abi.h"
#include "dram_sun60i_a733_tables.h"
#include "dram_sun60i_a733_pmu_fw.h"
#define pmu_imem_image sun60i_a733_pmu_imem

/* ========================================================================= */
/* 1. PRCM & R_TWI & AXP8191 PMIC Definitions                      */
/* ========================================================================= */

#define SUNXI_PRCM_R_TWI_BGR		0x0701019cUL
#define SUNXI_R_PIO_PL_CFG0		0x07025000UL
#define SUNXI_R_PIO_PL_DRV0		0x07025014UL
#define SUNXI_R_PIO_PL_PULL0		0x07025024UL

#define SUNXI_R_TWI_BASE		0x07083000UL
#define R_TWI_ADDR			(SUNXI_R_TWI_BASE + 0x00)
#define R_TWI_XADDR			(SUNXI_R_TWI_BASE + 0x04)
#define R_TWI_DATA			(SUNXI_R_TWI_BASE + 0x08)
#define R_TWI_CNTR			(SUNXI_R_TWI_BASE + 0x0c)
#define R_TWI_STAT			(SUNXI_R_TWI_BASE + 0x10)
#define R_TWI_CCR			(SUNXI_R_TWI_BASE + 0x14)
#define R_TWI_SRST			(SUNXI_R_TWI_BASE + 0x18)
#define R_TWI_LCR			(SUNXI_R_TWI_BASE + 0x1c)
#define R_TWI_EFR			(SUNXI_R_TWI_BASE + 0x20)

#define R_TWI_CNTR_BUS_EN		BIT(6)
#define R_TWI_CNTR_START		BIT(5)
#define R_TWI_CNTR_STOP			BIT(4)
#define R_TWI_CNTR_INT_FLAG		BIT(3)
#define R_TWI_CNTR_A_ACK		BIT(2)

#define R_TWI_STAT_START		0x08
#define R_TWI_STAT_REPEATED_START	0x10
#define R_TWI_STAT_ADDR_W_ACK		0x18
#define R_TWI_STAT_DATA_W_ACK		0x28
#define R_TWI_STAT_ADDR_R_ACK		0x40
#define R_TWI_STAT_DATA_R_ACK		0x50
#define R_TWI_STAT_DATA_R_NAK		0x58
#define R_TWI_STAT_IDLE			0xf8

#define R_TWI_CCR_400KHZ		0x28
#define R_TWI_SRST_RESET		0x01

#define AXP8191_I2C_ADDR		0x36
#define AXP8191_CHIP_ID_REG		0x0e
#define AXP8191_CHIP_ID_VAL		0x03

/**
 * struct axp_reg_val - AXP8191 PMIC register write descriptor
 * @reg: PMIC internal register offset
 * @val: 8-bit register value to program
 */
struct axp_reg_val {
	u8 reg;
	u8 val;
};

static const struct axp_reg_val axp8191_power_rails[] = {
	/* DCDC Voltages */
	{ 0x13, 0x9e }, /* DCDC2: VDD_CPU = 800 mV */
	{ 0x14, 0xb7 }, /* DCDC3: Core logic / system rails */
	{ 0x15, 0x9e }, /* DCDC4: System rail */
	{ 0x16, 0xb2 }, /* DCDC5: System rail */
	{ 0x17, 0x86 }, /* DCDC6: VDDQ (LPDDR5 memory I/O) = 560 mV */
	{ 0x18, 0xba }, /* DCDC7: VDD2 (LPDDR5 memory core) = 1080 mV */
	{ 0x19, 0xe4 }, /* DCDC8: Peripheral rail */
	{ 0x1a, 0xc8 }, /* DCDC9: Peripheral rail */

	/* DCDC Enables & Sequencing */
	{ 0x10, 0xff }, /* DCDC Enable 1: Enable DCDC1 through DCDC8 */
	{ 0x11, 0x1f }, /* DCDC Enable 2: Enable DCDC9, DC1SW1, DC1SW2 */
	{ 0x12, 0x17 }, /* DCDC Control: Sequencing and mode control */

	/* LDO Voltages */
	{ 0x2a, 0x0d }, /* BLDO1: VDD1 (LPDDR5 analog core) = 1800 mV */
	{ 0x2b, 0x0d }, /* BLDO2: VDD1 secondary = 1800 mV */
	{ 0x2d, 0x0d }, /* BLDO4: Auxiliary analog = 1800 mV */
	{ 0x3a, 0x17 }, /* ELDO1: PHY analog VDD = 1080 mV */

	/* LDO Enables */
	{ 0x20, 0xc1 }, /* LDO Bank 1 Enable */
	{ 0x21, 0xbe }, /* LDO Bank 2 Enable (BLDOs on) */
	{ 0x22, 0xe1 }, /* LDO Bank 3 Enable (ELDOs on) */
	{ 0x23, 0x08 }, /* LDO Bank 4 Enable */
};

/* ========================================================================= */
/* 2. Security Protection Controller & CCU Matrix Bus Security    */
/* ========================================================================= */

#define SUNXI_SPC_BASE				0x02054000UL
#define SUNXI_SPC_PORT_COUNT			24
#define SUNXI_SPC_PORT_GATE_VAL			0xffffffffU

#define SUNXI_CCMU_SEC_SW			0x02003f00UL
#define SUNXI_CCMU_SEC_SW_VAL			0x00000007U
#define SUNXI_SYS_CFG_SEC			0x03000044UL
#define SUNXI_SYS_CFG_SEC_VAL			0x00000001U

#define SUNXI_CCU_MBUS_MAT_CLK_GATING		0x020025c0UL
#define SUNXI_CCU_MBUS_MAT_CLK_GATING_VAL	0x110103ffU
#define SUNXI_CCU_MBUS_MAT_CLK_ENABLE		0x020025e0UL
#define SUNXI_CCU_MBUS_MAT_CLK_ENABLE_VAL	0xf1055803U
#define SUNXI_CCU_MBUS_MASTER_GATE		0x020025e4UL
#define SUNXI_CCU_MBUS_MASTER_GATE_VAL		0x0007ffffU

#define SUNXI_CCU_SEC_GATE_BASE			0x02000804UL
#define SUNXI_CCU_SEC_GATE_COUNT		57
#define SUNXI_CCU_SEC_GATE_VAL			0xffffffffU

#define SUNXI_PRCM_SEC_GATE0			0x07002004UL
#define SUNXI_PRCM_SEC_GATE1			0x07002014UL
#define SUNXI_PRCM_SEC_GATE2			0x07002024UL
#define SUNXI_PRCM_SEC_GATE_VAL			0xffffffffU
#define SUNXI_PRCM_CLK_GATING			0x07010290UL
#define SUNXI_PRCM_CLK_GATING_BITS		0x07U

#define SUNXI_IOMMU0_BYPASS			0x03900030UL
#define SUNXI_IOMMU1_BYPASS			0x03910030UL
#define SUNXI_IOMMU_BYPASS_VAL			0xffffffffU

#define SUNXI_SYSCTRL_RESCAL_CTRL		0x03000160UL
#define SUNXI_SYSCTRL_RESCAL_CAL		0x03000168UL

/* ========================================================================= */
/* 3. Register Sequencer & Aperture Boundaries                    */
/* ========================================================================= */

#define SUNXI_MMIO_MIN_ADDR		0x02000000UL
#define SUNXI_MMIO_MAX_ADDR		0x40000000UL

#define CCU_PLL_DDR_CTRL_REG		0x02002020UL
#define CCU_PLL_DDR_LOCK_BIT		BIT(28)

#define PRCM_MEM_ISOLATION_REG		0x07010250UL

#define UMCTL2_CH0_DRAMTMG0		0x0a100000UL
#define UMCTL2_CH0_DRAMTMG2		0x0a100008UL
#define UMCTL2_CH0_DRAMTMG4		0x0a100010UL
#define UMCTL2_CH0_BASE			0x0a110000UL
#define UMCTL2_CH1_BASE			0x0a510000UL
#define UMCTL2_MSTR_OFFSET		0x00UL
#define UMCTL2_MRCTRL0_OFFSET		0x80UL
#define UMCTL2_MRCTRL1_OFFSET		0x84UL

#define UMCTL2_MSTR_LPDDR5		BIT(19)
#define UMCTL2_MSTR_RANKS_SHIFT		24
#define UMCTL2_MSTR_RANKS_MASK		0x0fU

#define DRAMC_CH_EN_REG			0x0a020014UL
#define DRAMC_MR_DATA_CH0_REG		0x0a02002cUL
#define DRAMC_MR_DATA_CH1_REG		0x0a020040UL

/* ========================================================================= */
/* 4. Cadence PHY & Controller Apertures (SPEC-021 / SPEC-022)               */
/* ========================================================================= */

#define SUNXI_DRAM_PHY_CTRL_BASE	0x0aaa0000UL
#define SUNXI_DRAM_PHY_IMEM_BASE	0x0a9a0000UL
#define SUNXI_DRAM_PHY_DMEM_BASE	0x0a9b0000UL
#define SUNXI_UMCTL2_CH0_STAT		0x0a110014UL
#define SUNXI_UMCTL2_CH1_STAT		0x0a510014UL

#define SUNXI_PRCM_CH0_DMC_CLK_REG	0x07010310UL
#define SUNXI_PRCM_CH0_DMC_FEAT_REG	0x07010314UL
#define SUNXI_PRCM_CH1_DMC_CLK_REG	0x07410310UL
#define SUNXI_PRCM_CH1_DMC_FEAT_REG	0x07410314UL

#define A733_DRAM_CH0_BASE		0x40000000UL
#define A733_DRAM_CH1_BASE		0xc0000000UL
#define A733_BOOT0_TEST_WORDS		4096

static const uintptr_t sparse_probe_addrs[] = {
	0x40000000UL, /* Ch0 Base */
	0x40800000UL, /* +8 MiB */
	0x44000000UL, /* +64 MiB, FIT image load aperture */
	0x4a000000UL, /* +160 MiB, U-Boot text base aperture */
	0x50000000UL, /* +256 MiB */
	0xc0000000UL, /* Ch1 Base */
};

#ifndef dram_halt
/**
 * dram_halt() - Halt execution in an infinite loop on fatal DRAM error
 */
static inline void dram_halt(void)
{
	while (1)
		;
}
#endif

/* ========================================================================= */
/* SECTION 1: R_TWI Bus & AXP8191 PMIC Power Subsystem            */
/* ========================================================================= */

/**
 * r_twi_wait_status() - Poll R_TWI interrupt flag and verify bus status
 * @expected: Expected 8-bit TWI status code in R_TWI_STAT
 *
 * Waits up to 100 ms for the R_TWI controller to assert R_TWI_CNTR_INT_FLAG
 * and verifies that the resulting status code matches @expected (also
 * accepting R_TWI_STAT_START when R_TWI_STAT_REPEATED_START is requested).
 *
 * Return: 0 on match, -EIO on unexpected bus state, or -ETIMEDOUT on timeout.
 */
static int r_twi_wait_status(u32 expected)
{
	int timeout = 10000;

	while (timeout--) {
		if (readl((void __iomem *)R_TWI_CNTR) & R_TWI_CNTR_INT_FLAG) {
			u32 status = readl((void __iomem *)R_TWI_STAT);

			if (status == expected ||
			    (expected == R_TWI_STAT_REPEATED_START &&
			     status == R_TWI_STAT_START))
				return 0;
			return -EIO;
		}
		udelay(10);
	}
	return -ETIMEDOUT;
}

/**
 * r_twi_start() - Generate an I2C START condition on the R_TWI bus
 *
 * Return: 0 on success, or negative errno on bus error/timeout.
 */
static int r_twi_start(void)
{
	writel(R_TWI_CNTR_BUS_EN | R_TWI_CNTR_START | R_TWI_CNTR_INT_FLAG,
	       (void __iomem *)R_TWI_CNTR);
	return r_twi_wait_status(R_TWI_STAT_START);
}

/**
 * r_twi_restart() - Generate a repeated I2C START condition on the R_TWI bus
 *
 * Return: 0 on success, or negative errno on bus error/timeout.
 */
static int r_twi_restart(void)
{
	writel(R_TWI_CNTR_BUS_EN | R_TWI_CNTR_START | R_TWI_CNTR_INT_FLAG,
	       (void __iomem *)R_TWI_CNTR);
	return r_twi_wait_status(R_TWI_STAT_REPEATED_START);
}

/**
 * r_twi_send_byte() - Transmit a single byte over R_TWI and check ACK status
 * @byte: Data or slave-address byte to transmit
 * @expected: Expected R_TWI_STAT ACK state (e.g. R_TWI_STAT_DATA_W_ACK)
 *
 * Return: 0 on expected ACK, or negative errno on NAK/timeout.
 */
static int r_twi_send_byte(u8 byte, u32 expected)
{
	writel(byte, (void __iomem *)R_TWI_DATA);
	writel(R_TWI_CNTR_BUS_EN | R_TWI_CNTR_INT_FLAG,
	       (void __iomem *)R_TWI_CNTR);
	return r_twi_wait_status(expected);
}

/**
 * r_twi_recv_byte() - Receive a single byte over R_TWI
 * @byte: Output pointer to store the received byte
 * @send_ack: True to assert master ACK, false to assert NAK on final byte
 *
 * Return: 0 on success, or negative errno on bus error/timeout.
 */
static int r_twi_recv_byte(u8 *byte, bool send_ack)
{
	u32 ctrl = R_TWI_CNTR_BUS_EN | R_TWI_CNTR_INT_FLAG;
	u32 expected = send_ack ? R_TWI_STAT_DATA_R_ACK : R_TWI_STAT_DATA_R_NAK;
	int ret;

	if (send_ack)
		ctrl |= R_TWI_CNTR_A_ACK;

	writel(ctrl, (void __iomem *)R_TWI_CNTR);
	ret = r_twi_wait_status(expected);
	if (ret)
		return ret;

	*byte = (u8)readl((void __iomem *)R_TWI_DATA);
	return 0;
}

/**
 * r_twi_stop() - Generate an I2C STOP condition and wait for bus idle
 */
static void r_twi_stop(void)
{
	int timeout = 1000;

	writel(R_TWI_CNTR_BUS_EN | R_TWI_CNTR_STOP | R_TWI_CNTR_INT_FLAG,
	       (void __iomem *)R_TWI_CNTR);

	while (timeout--) {
		if (readl((void __iomem *)R_TWI_STAT) == R_TWI_STAT_IDLE)
			break;
		udelay(10);
	}
}

/**
 * axp_i2c_write() - Write an 8-bit register on the AXP8191 PMIC
 * @reg: PMIC register address
 * @val: Value to write
 *
 * Return: 0 on success, or negative errno on I2C transfer failure.
 */
static int axp_i2c_write(u8 reg, u8 val)
{
	int ret;

	ret = r_twi_start();
	if (ret)
		goto err;

	ret = r_twi_send_byte((AXP8191_I2C_ADDR << 1) | 0,
			      R_TWI_STAT_ADDR_W_ACK);
	if (ret)
		goto err;

	ret = r_twi_send_byte(reg, R_TWI_STAT_DATA_W_ACK);
	if (ret)
		goto err;

	ret = r_twi_send_byte(val, R_TWI_STAT_DATA_W_ACK);
	if (ret)
		goto err;

	r_twi_stop();
	return 0;

err:
	r_twi_stop();
	return ret;
}

/**
 * axp_i2c_read() - Read an 8-bit register from the AXP8191 PMIC
 * @reg: PMIC register address
 * @val: Output pointer to receive the register value
 *
 * Return: 0 on success, or negative errno on I2C transfer failure.
 */
static int axp_i2c_read(u8 reg, u8 *val)
{
	int ret;

	ret = r_twi_start();
	if (ret)
		goto err;

	ret = r_twi_send_byte((AXP8191_I2C_ADDR << 1) | 0,
			      R_TWI_STAT_ADDR_W_ACK);
	if (ret)
		goto err;

	ret = r_twi_send_byte(reg, R_TWI_STAT_DATA_W_ACK);
	if (ret)
		goto err;

	ret = r_twi_restart();
	if (ret)
		goto err;

	ret = r_twi_send_byte((AXP8191_I2C_ADDR << 1) | 1,
			      R_TWI_STAT_ADDR_R_ACK);
	if (ret)
		goto err;

	ret = r_twi_recv_byte(val, false);
	if (ret)
		goto err;

	r_twi_stop();
	return 0;

err:
	r_twi_stop();
	return ret;
}

/**
 * r_twi_bus_clear_if_needed() - Recover stuck I2C SDA/SCL lines via clock pulses
 *
 * Inspects the R_TWI line state and toggles SCL up to 10 cycles if either
 * bus line is held low across a warm reset.
 */
static void r_twi_bus_clear_if_needed(void)
{
	unsigned int i;

	if ((readl((void __iomem *)R_TWI_EFR) & 0x30) == 0x30)
		return;

	writel(0x05, (void __iomem *)R_TWI_EFR);

	for (i = 0; i < 10; i++) {
		setbits_le32((void __iomem *)R_TWI_LCR, 0x0a);
		mdelay(1);
		clrbits_le32((void __iomem *)R_TWI_LCR, 0x0a);
		mdelay(1);
	}

	writel(0x00, (void __iomem *)R_TWI_EFR);
	writel(0x00, (void __iomem *)R_TWI_LCR);
}

/**
 * r_twi_init() - Initialize the PRCM R_TWI0 controller and PL0/PL1 pinmux
 *
 * Enables the R_TWI0 clock and reset in PRCM, configures PL0/PL1 for R_TWI0
 * with pull-ups, performs a controller soft-reset, and sets the bus rate to
 * 400 kHz.
 *
 * Return: 0 on success, or -ETIMEDOUT if controller soft-reset fails to clear.
 */
static int r_twi_init(void)
{
	int timeout = 1000;

	setbits_le32((void __iomem *)SUNXI_PRCM_R_TWI_BGR, BIT(16) | BIT(0));
	clrsetbits_le32((void __iomem *)SUNXI_R_PIO_PL_CFG0, 0x000000ff, 0x22);
	clrsetbits_le32((void __iomem *)SUNXI_R_PIO_PL_DRV0, 0x0000000f, 0x0a);
	clrsetbits_le32((void __iomem *)SUNXI_R_PIO_PL_PULL0, 0x0000000f, 0x05);

	writel(R_TWI_SRST_RESET, (void __iomem *)R_TWI_SRST);
	while (timeout--) {
		if (!(readl((void __iomem *)R_TWI_SRST) & R_TWI_SRST_RESET))
			break;
		udelay(10);
	}
	if (timeout < 0)
		return -ETIMEDOUT;

	writel(R_TWI_CCR_400KHZ, (void __iomem *)R_TWI_CCR);
	r_twi_bus_clear_if_needed();
	writel(R_TWI_CNTR_BUS_EN, (void __iomem *)R_TWI_CNTR);

	return 0;
}

/**
 * sunxi_pmic_init_a733() - Probe AXP8191 PMIC and configure LPDDR5 power rails
 *
 * Verifies the AXP8191 chip ID over R_TWI0 and programs DCDC2..9, BLDO1..4,
 * and ELDO1 (including VDDQ = 560 mV, VDD2 = 1080 mV, and VDD1 = 1800 mV),
 * waiting 10 ms for rail stabilization.
 *
 * Return: 0 on success, -ENODEV on chip ID mismatch, or negative I2C errno.
 */
static int sunxi_pmic_init_a733(void)
{
	unsigned int i;
	u8 chip_id = 0;
	int ret;

	ret = r_twi_init();
	if (ret)
		return ret;

	ret = axp_i2c_read(AXP8191_CHIP_ID_REG, &chip_id);
	if (ret)
		return ret;

	if (chip_id != AXP8191_CHIP_ID_VAL)
		return -ENODEV;

	for (i = 0; i < ARRAY_SIZE(axp8191_power_rails); i++) {
		ret = axp_i2c_write(axp8191_power_rails[i].reg,
				    axp8191_power_rails[i].val);
		if (ret)
			return ret;
	}

	mdelay(10);
	return 0;
}

/* ========================================================================= */
/* SECTION 2: Security Protection & CCU Matrix Bus Security*/
/* ========================================================================= */

/**
 * sunxi_security_and_bus_init() - Unlock SPC, CCU/PRCM security gates, and MBUS
 *
 * Configures the Security Protection Controller (SPC) master ports, CCU and
 * PRCM security switches, MBUS matrix clock gates, IOMMU0/1 bypass registers,
 * and SYSCTRL RESCAL pad impedance calibration required before DRAM access.
 */
static void sunxi_security_and_bus_init(void)
{
	unsigned int i;

	/* Step 1: Security Protection Controller (SPC) Master Ports Unlock */
	for (i = 0; i < SUNXI_SPC_PORT_COUNT; i++)
		writel(SUNXI_SPC_PORT_GATE_VAL,
		       (void __iomem *)(SUNXI_SPC_BASE + (i * 4)));

	/* Step 2: Clock & System Configuration Security Switches */
	writel(SUNXI_CCMU_SEC_SW_VAL, (void __iomem *)SUNXI_CCMU_SEC_SW);
	writel(SUNXI_SYS_CFG_SEC_VAL, (void __iomem *)SUNXI_SYS_CFG_SEC);

	/* Step 3: CCU MBUS Matrix Clock Gating */
	writel(SUNXI_CCU_MBUS_MAT_CLK_GATING_VAL,
	       (void __iomem *)SUNXI_CCU_MBUS_MAT_CLK_GATING);
	writel(SUNXI_CCU_MBUS_MAT_CLK_ENABLE_VAL,
	       (void __iomem *)SUNXI_CCU_MBUS_MAT_CLK_ENABLE);
	writel(SUNXI_CCU_MBUS_MASTER_GATE_VAL,
	       (void __iomem *)SUNXI_CCU_MBUS_MASTER_GATE);

	/* Step 4: CCU Bus Security Gate Array (57 gates) */
	for (i = 0; i < SUNXI_CCU_SEC_GATE_COUNT; i++)
		writel(SUNXI_CCU_SEC_GATE_VAL,
		       (void __iomem *)(SUNXI_CCU_SEC_GATE_BASE + (i * 4)));

	/* Step 5: PRCM Bus Security & Clocks */
	writel(SUNXI_PRCM_SEC_GATE_VAL, (void __iomem *)SUNXI_PRCM_SEC_GATE0);
	writel(SUNXI_PRCM_SEC_GATE_VAL, (void __iomem *)SUNXI_PRCM_SEC_GATE1);
	writel(SUNXI_PRCM_SEC_GATE_VAL, (void __iomem *)SUNXI_PRCM_SEC_GATE2);
	setbits_le32((void __iomem *)SUNXI_PRCM_CLK_GATING,
		     SUNXI_PRCM_CLK_GATING_BITS);

	/* Step 6: System IOMMU Auto-Bypass */
	writel(SUNXI_IOMMU_BYPASS_VAL, (void __iomem *)SUNXI_IOMMU0_BYPASS);
	writel(SUNXI_IOMMU_BYPASS_VAL, (void __iomem *)SUNXI_IOMMU1_BYPASS);

	/* Step 7: SYSCTRL RESCAL Pad Impedance Calibration */
	setbits_le32((void __iomem *)SUNXI_SYSCTRL_RESCAL_CTRL, BIT(8));
	clrbits_le32((void __iomem *)SUNXI_SYSCTRL_RESCAL_CAL, 0xffU);
}

/* ========================================================================= */
/* SECTION 3: Hardware Register Sequencer & State Polling         */
/* ========================================================================= */

/**
 * mctl_poll_reg32() - Poll a 32-bit MMIO register until masked bits match
 * @addr: Physical address of status register to poll
 * @mask: Bitmask to apply to the register value
 * @expected: Expected masked value
 * @timeout_us: Maximum poll iterations (1 us per iteration)
 * @desc: Diagnostic description logged on timeout
 * @trig_addr: Trigger register address logged on timeout
 */
static void mctl_poll_reg32(uintptr_t addr, u32 mask, u32 expected,
			    int timeout_us, const char *desc,
			    uintptr_t trig_addr)
{
	while (timeout_us--) {
		if ((readl((void __iomem *)addr) & mask) == expected)
			return;
		udelay(1);
	}

	pr_warn("DRAM: %s timeout at 0x%08lx\n", desc,
		(unsigned long)trig_addr);
}

/**
 * mctl_sync_hardware_state() - Synchronize hardware state after a register write
 * @addr: Physical MMIO address that was just written
 * @val: 32-bit value that was written to @addr
 *
 * Polls hardware status registers or inserts required settling delays when
 * specific control registers (PLL_DDR, uMCTL2 SWCTL/MRCTRL0/DFIMISC, or PRCM
 * memory isolation) are written during table replay.
 */
static void mctl_sync_hardware_state(uintptr_t addr, u32 val)
{
	/* 3.1 CCU PLL_DDR Lock */
	if (addr == CCU_PLL_DDR_CTRL_REG &&
	    (val & 0xe0000000U) == 0xe0000000U &&
	    (val & 0x08000000U) == 0U) {
		udelay(20);
		mctl_poll_reg32(CCU_PLL_DDR_CTRL_REG, CCU_PLL_DDR_LOCK_BIT,
				CCU_PLL_DDR_LOCK_BIT, 100000,
				"PLL_DDR lock", addr);
		return;
	}

	/* 3.2 uMCTL2 DQS Calibration Completion */
	if ((addr == 0x0a110b84UL || addr == 0x0a510b84UL) && val == 1U) {
		u32 mask = (0x03U << 25) | (0x03U << 28);

		mctl_poll_reg32(addr + 4, mask, mask, 100000,
				"DQS calibration", addr);
		return;
	}

	/* 3.3 uMCTL2 Software Control Handshake */
	if ((addr == 0x0a110c80UL || addr == 0x0a510c80UL) && val == 1U) {
		mctl_poll_reg32(addr + 4, 0x01U, 0x01U, 10000,
				"SWCTL handshake", addr);
		return;
	}

	/* 3.4 uMCTL2 Power Control Exit to Normal Operating Mode */
	if ((addr == 0x0a110180UL || addr == 0x0a510180UL) && val == 0U) {
		uintptr_t stat_addr = addr - 0x180 + 0x14;

		mctl_poll_reg32(stat_addr, 0x03U << 4, 0U, 100000,
				"Normal mode entry phase 1", addr);
		mctl_poll_reg32(stat_addr, 0x03U, 0x01U, 100000,
				"Normal mode entry phase 2", addr);
		return;
	}

	/* 3.5 uMCTL2 Mode Register Command Trigger */
	if ((addr == 0x0a110080UL || addr == 0x0a510080UL) &&
	    (val & 0x80000000U) != 0U) {
		mctl_poll_reg32(addr, 0x80000000U, 0U, 10000,
				"MR command trigger", addr);
		return;
	}

	/* 3.6 PRCM Memory Isolation Deassertion */
	if (addr == PRCM_MEM_ISOLATION_REG) {
		mdelay(2);
		return;
	}

	/* 3.7 uMCTL2 Operating Mode 2 Transition */
	if ((addr == 0x0a110184UL || addr == 0x0a510184UL) && val == 2U) {
		mctl_poll_reg32(addr - 0x170, 0x07U, 2U, 50000,
				"Mode 2 transition", addr);
		return;
	}
}

/**
 * mctl_write_single_reg() - Write a single MMIO register and synchronize state
 * @addr: Physical MMIO address
 * @val: Value to write (16-bit for DDR PHY APB, 32-bit otherwise)
 */
static void mctl_write_single_reg(uintptr_t addr, u32 val)
{
	if (addr < SUNXI_MMIO_MIN_ADDR || addr >= SUNXI_MMIO_MAX_ADDR)
		return;

	if (addr >= 0x0a900000UL && addr < 0x0ab00000UL)
		writew((u16)val, (void __iomem *)addr);
	else
		writel(val, (void __iomem *)addr);

	mctl_sync_hardware_state(addr, val);
}

/**
 * mctl_write_regs() - Replay an array of width- and broadcast-tagged MMIO writes
 * @regs: Pointer to array of struct sunxi_dram_reg descriptors
 * @count: Number of descriptors in @regs
 *
 * Writes each 16- or 32-bit register entry (expanding PHY_BCAST_DBYTE,
 * PHY_BCAST_AC, and PHY_BCAST_9LANE broadcast tags across the corresponding
 * PHY slices and lanes) and invokes mctl_sync_hardware_state() after each write.
 */
static void mctl_write_regs(const struct sunxi_dram_reg *regs, u32 count)
{
	u32 i;

	if (!regs || !count)
		return;

	for (i = 0; i < count; i++) {
		u32 raw_addr = regs[i].addr;
		uintptr_t base = raw_addr & ~PHY_BCAST_MASK;
		u32 val = regs[i].val;
		unsigned int s, l;

		if (raw_addr & PHY_BCAST_9LANE) {
			for (s = 0; s < 4; s++) {
				for (l = 0; l < 9; l++)
					mctl_write_single_reg(base + s * 0x2000UL +
							      l * 0x200UL, val);
			}
		} else if (raw_addr & PHY_BCAST_DBYTE) {
			for (s = 0; s < 4; s++)
				mctl_write_single_reg(base + s * 0x2000UL, val);
		} else if (raw_addr & PHY_BCAST_AC) {
			for (s = 0; s < 2; s++)
				mctl_write_single_reg(base + s * 0x2000UL, val);
		} else {
			mctl_write_single_reg(base, val);
		}
	}
}

/**
 * mctl_write_regs_offset() - Replay an array of MMIO register writes with channel offset
 * @regs: Pointer to array of struct sunxi_dram_reg descriptors
 * @count: Number of descriptors in @regs
 * @ch_offset: Address offset added to controller registers (0x0a100000..0x0a4fffff)
 *
 * Writes each register entry, adding @ch_offset to uMCTL2/port addresses while
 * leaving global PHY/CCU addresses unchanged, and invokes
 * mctl_sync_hardware_state() after each write.
 */
static void mctl_write_regs_offset(const struct sunxi_dram_reg *regs, u32 count,
				   uintptr_t ch_offset)
{
	u32 i;

	for (i = 0; i < count; i++) {
		uintptr_t addr = regs[i].addr;
		u32 val = regs[i].val;

		if (addr >= 0x0a100000UL && addr < 0x0a500000UL)
			addr += ch_offset;

		if (addr >= 0x0a900000UL && addr < 0x0ab00000UL)
			writew((u16)val, (void __iomem *)addr);
		else
			writel(val, (void __iomem *)addr);

		mctl_sync_hardware_state(addr, val);
	}
}

/**
 * mctl_write_mr_all() - Issue a JEDEC Mode Register Write across both channels and ranks
 * @mr_type: MRCTRL0 command type (0 for MRW, 1 for VRCG/FSP control)
 * @mr_addr: Mode Register number (e.g. 14, 15, 16)
 * @mr_val: 8-bit Mode Register payload
 *
 * Writes @mr_val to @mr_addr on Rank 0 and Rank 1 of both Channel 0 and
 * Channel 1 via uMCTL2 MRCTRL0/MRCTRL1 and polls for completion.
 */
static void mctl_write_mr_all(u32 mr_type, u8 mr_addr, u8 mr_val)
{
	unsigned int ch, rank;

	for (ch = 0; ch < 2; ch++) {
		uintptr_t base = (ch == 0) ? UMCTL2_CH0_BASE : UMCTL2_CH1_BASE;
		uintptr_t mrctrl0 = base + UMCTL2_MRCTRL0_OFFSET;
		uintptr_t mrctrl1 = base + UMCTL2_MRCTRL1_OFFSET;

		for (rank = 0; rank < 2; rank++) {
			u32 ctrl0 = (BIT(rank) << 4) | (mr_type & 1U);
			u32 ctrl1 = ((u32)mr_addr << 8) | mr_val;

			writel(ctrl0, (void __iomem *)mrctrl0);
			writel(ctrl1, (void __iomem *)mrctrl1);
			writel(ctrl0 | BIT(31), (void __iomem *)mrctrl0);
			mctl_sync_hardware_state(mrctrl0, ctrl0 | BIT(31));
		}
	}
}

/* ========================================================================= */
/* SECTION 4: Cadence DDR PHY & PMU Initializer                   */
/* ========================================================================= */

/**
 * mctl_pmu_load_sram() - Load a 16-bit image into a 64 KiB PMU SRAM aperture
 * @sram_base: Physical base address of the PMU IMEM or DMEM aperture
 * @data: Pointer to array of 16-bit halfwords to load
 * @count: Number of halfwords in @data
 *
 * Switches the PHY APB mux to host access and writes all 32,768 halfwords
 * (64 KiB) of the target ICCM or DCCM SRAM to initialize hardware parity bits
 * and zero any trailing words.
 */
static void mctl_pmu_load_sram(uintptr_t sram_base, const u16 *data,
			       size_t count)
{
	size_t i;

	writew(0x0000,
	       (void __iomem *)(SUNXI_DRAM_PHY_CTRL_BASE + PMU_REG_APB_MUX));

	for (i = 0; i < count; i++)
		writew(data[i], (void __iomem *)(sram_base + i * 2));
	for (; i < 0x8000; i++)
		writew(0x0000, (void __iomem *)(sram_base + i * 2));
}

/**
 * mctl_pmu_log_error() - Log PMU training failure diagnostics from DMEM
 * @msg: Error summary prefix string
 * @status: 32-bit PMU status word read from the mailbox registers
 *
 * Reads the current training stage and assertion code from the PMU message
 * block in DMEM and prints an error diagnostic.
 */
static void mctl_pmu_log_error(const char *msg, u32 status)
{
	uintptr_t dmem = SUNXI_DRAM_PHY_DMEM_BASE;
	size_t assert_off = offsetof(struct pmu_message_block, assert_code);
	size_t seq_off = offsetof(struct pmu_message_block, sequence_ctrl);
	u32 assert_code = (u32)readw((void __iomem *)(dmem + assert_off)) |
			  ((u32)readw((void __iomem *)(dmem + assert_off + 2)) << 16);
	u16 stage = readw((void __iomem *)(dmem + seq_off));

	pr_err("DRAM: %s, status = 0x%08x (stage=0x%04x, assert=0x%08x)\n",
	       msg, status, stage, assert_code);
}

/**
 * mctl_pmu_run_training() - Hand over APB, trigger ARC EM4 PMU, and await completion
 *
 * Switches the PHY APB mux to the PMU microsequencer, releases the ARC core
 * from reset, polls the mailbox status until training begins and completes,
 * and acknowledges completion.
 *
 * Return: 0 on successful training completion, or negative errno on failure.
 */
static int mctl_pmu_run_training(void)
{
	uintptr_t ctrl = SUNXI_DRAM_PHY_CTRL_BASE;
	u32 status;
	int timeout;

	/* Hand over APB to PMU Microsequencer and release ARC reset */
	writew(0x0001, (void __iomem *)(ctrl + PMU_REG_APB_MUX));
	writew(0x0009, (void __iomem *)(ctrl + PMU_REG_RESET));
	writew(0x0001, (void __iomem *)(ctrl + PMU_REG_RESET));
	writew(0x0001, (void __iomem *)(ctrl + PMU_REG_RESET));
	writew(0x0000, (void __iomem *)(ctrl + PMU_REG_RESET));
	writew(0x0001, (void __iomem *)(ctrl + PMU_REG_HANDOFF_1));
	writew(0x0001, (void __iomem *)(ctrl + PMU_REG_MAILBOX_INT));

	/* Wait for training to begin (bit 0 of PMU_REG_POLL_STAT == 1) */
	timeout = 100000;
	while (timeout--) {
		if (readw((void __iomem *)(ctrl + PMU_REG_POLL_STAT)) & 0x01)
			break;
		udelay(1);
	}
	if (timeout <= 0) {
		pr_err("DRAM: PMU training start timeout\n");
		return -ETIMEDOUT;
	}

	/* Wait for training to complete (bit 0 of PMU_REG_POLL_STAT == 0) */
	timeout = 5000000;
	while (timeout--) {
		if (!(readw((void __iomem *)(ctrl + PMU_REG_POLL_STAT)) & 0x01))
			break;
		udelay(1);
	}
	if (timeout <= 0) {
		mctl_pmu_log_error("PMU training completion timeout", 0);
		return -ETIMEDOUT;
	}

	/* Read PMU status word before clearing triggers (Invariant 4.2) */
	status = (u32)readw((void __iomem *)(ctrl + PMU_REG_STATUS_LO)) |
		 ((u32)readw((void __iomem *)(ctrl + PMU_REG_STATUS_HI)) << 16);

	/* Complete handshake with PMU */
	writew(0x0000, (void __iomem *)(ctrl + PMU_REG_HANDOFF_1));
	writew(0x0001, (void __iomem *)(ctrl + PMU_REG_HANDOFF_1));
	writew(0x0001, (void __iomem *)(ctrl + PMU_REG_RESET));

	if (status != PMU_STATUS_SUCCESS && status != PMU_STATUS_FATAL_ERROR) {
		mctl_pmu_log_error("PMU training failed", status);
		return -EIO;
	}

	return 0;
}

/**
 * mctl_phy_dump_slices() - Dump trained PHY slice delay registers for debug
 *
 * Temporarily switches the PHY APB mux to host access, logs key trained delay
 * registers across all four data slices, and restores the mux state.
 */
static void mctl_phy_dump_slices(void)
{
	uintptr_t ctrl = SUNXI_DRAM_PHY_CTRL_BASE;
	unsigned int i;

	writew(0x0000, (void __iomem *)(ctrl + PMU_REG_APB_MUX));
	udelay(10);
	for (i = 0; i < 4; i++) {
		uintptr_t base = 0x0a920000UL + (i * 0x2000UL);

		debug("Slice %u: CLK1E=0x%04x RxEn=0x%04x TxEn=0x%04x CompEn=0x%04x RxBias=0x%04x LCDL162=0x%04x STAT16A=0x%04x RFIFO=0x%04x\n",
		      i,
		      readw((void __iomem *)(base + 0x001e)),
		      readw((void __iomem *)(base + 0x0112)),
		      readw((void __iomem *)(base + 0x0114)),
		      readw((void __iomem *)(base + 0x0116)),
		      readw((void __iomem *)(base + 0x0118)),
		      readw((void __iomem *)(base + 0x0162)),
		      readw((void __iomem *)(base + 0x016a)),
		      readw((void __iomem *)(base + 0x0146)));
	}
	writew(0x0001, (void __iomem *)(ctrl + PMU_REG_APB_MUX));
	udelay(10);
}

/**
 * mctl_umctl2_wait_normal_mode() - Wait for a uMCTL2 channel to enter Normal Mode
 * @stat_reg: Physical address of the channel's UMCTL2 STAT register
 * @ch: Channel index (0 or 1) for error reporting
 *
 * Return: 0 when operating_mode[1:0] == 0x01, or -ETIMEDOUT on timeout.
 */
static int mctl_umctl2_wait_normal_mode(uintptr_t stat_reg, unsigned int ch)
{
	int timeout = 100000;

	while (timeout--) {
		if ((readl((void __iomem *)stat_reg) & 0x03U) == 0x01U)
			return 0;
		udelay(1);
	}

	pr_err("DRAM: Channel %u uMCTL2 failed to enter Normal Mode\n", ch);
	return -ETIMEDOUT;
}

/**
 * mctl_phy_load_pin_swizzle() - Program AC and DBYTE pin swizzle maps into the PHY
 *
 * Programs the board-specific CA0..CA6 pin permutation into AC slices 0..1
 * and the DQ0..DQ7 pin permutation into DBYTE slices 0..3 prior to PMU
 * training execution.
 */
static void mctl_phy_load_pin_swizzle(void)
{
	uintptr_t ctrl = SUNXI_DRAM_PHY_CTRL_BASE;
	unsigned int ac, slice, pin;

	writew(0x0001, (void __iomem *)(ctrl + PMU_REG_APB_MUX));
	writew(0x0000, (void __iomem *)(ctrl + PMU_REG_APB_MUX));

	for (ac = 0; ac < 2; ac++) {
		uintptr_t ac_base = 0x0a960120UL + ac * 0x2000UL;

		for (pin = 0; pin < 7; pin++)
			writew(dram_ac_pin_swizzle[ac][pin],
			       (void __iomem *)(ac_base + pin * 2));
	}

	for (pin = 0; pin < 8; pin++) {
		for (slice = 0; slice < 4; slice++) {
			uintptr_t dq_addr = 0x0a920100UL + slice * 0x2000UL +
					    pin * 2;

			writew(dram_dq_pin_swizzle[slice][pin],
			       (void __iomem *)dq_addr);
		}
	}

	writew(0x0001, (void __iomem *)(ctrl + PMU_REG_APB_MUX));
}

/**
 * mctl_load_pie_table() - Stream sparse non-zero entries into a PIE SRAM window
 * @base: Physical base address of the target PIE SRAM aperture
 * @total_words: Total number of 16-bit halfwords in the aperture
 * @words: Ordered array of sparse non-zero (offset, value) entries
 * @num_words: Number of entries in @words
 *
 * Writes @total_words consecutive 16-bit halfwords starting at @base,
 * inserting non-zero values from @words at their matching byte offsets and
 * zero-filling all other locations.
 */
static void mctl_load_pie_table(uintptr_t base, u32 total_words,
				const struct sunxi_pie_word *words,
				size_t num_words)
{
	size_t w = 0;
	u32 i;

	for (i = 0; i < total_words; i++) {
		u16 byte_off = (u16)(i * 2);
		u16 val = 0x0000;

		if (w < num_words && words[w].off == byte_off) {
			val = words[w].val;
			w++;
		}

		writew(val, (void __iomem *)(base + byte_off));
	}
}

/**
 * mctl_apply_stage3_config() - Program Stage 3 PIE sequencer, ACSM SRAM, and PHY CSRs
 *
 * Initializes the PHY Init Engine (PIE) instruction tables, streams the ACSM
 * command sequence SRAM image, programs the FSP pointer tables and lane
 * calibration registers, and triggers the post-training PIE state machine.
 */
static void mctl_apply_stage3_config(void)
{
	unsigned int i, lane, reg;

	mctl_write_regs(dram_stage3_head_regs,
			ARRAY_SIZE(dram_stage3_head_regs));
	mctl_load_pie_table(0x0a982010UL, 176, dram_stage3_pie_init,
			    ARRAY_SIZE(dram_stage3_pie_init));
	mctl_write_regs(dram_stage3_phy_setup_regs,
			ARRAY_SIZE(dram_stage3_phy_setup_regs));
	mctl_load_pie_table(0x0a982000UL, 8, NULL, 0);
	mctl_load_pie_table(0x0a982710UL, 2256, dram_stage3_pie_seq,
			    ARRAY_SIZE(dram_stage3_pie_seq));

	for (i = 0; i < ARRAY_SIZE(dram_stage3_acsm_sram); i++)
		writew(dram_stage3_acsm_sram[i],
		       (void __iomem *)(0x0aa2005cUL + i * 2));

	mctl_write_regs(dram_stage3_acsm_ctrl_regs,
			ARRAY_SIZE(dram_stage3_acsm_ctrl_regs));

	for (i = 0; i < ARRAY_SIZE(dram_stage3_pie_ptrs) / 2; i++) {
		writew(dram_stage3_pie_ptrs[i * 2],
		       (void __iomem *)(0x0a940248UL + i * 2));
		writew(dram_stage3_pie_ptrs[i * 2 + 1],
		       (void __iomem *)(0x0a940316UL + i * 2));
	}

	mctl_write_regs(dram_stage3_pie_patch_regs,
			ARRAY_SIZE(dram_stage3_pie_patch_regs));
	mctl_load_pie_table(0x0a982010UL, 176, dram_stage3_pie_init,
			    ARRAY_SIZE(dram_stage3_pie_init));
	mctl_write_regs(dram_stage3_pre_lane_regs,
			ARRAY_SIZE(dram_stage3_pre_lane_regs));

	for (lane = 0; lane < 9; lane++) {
		uintptr_t lane_base = 0x0a9e0048UL + lane * 0x200UL;

		for (reg = 0; reg < 8; reg++)
			writew(0x5a3c, (void __iomem *)(lane_base + reg * 2));
	}

	mctl_write_regs(dram_stage3_tail_regs,
			ARRAY_SIZE(dram_stage3_tail_regs));
}

/**
 * mctl_apply_post_pmu_config() - Apply post-PMU uMCTL2 timing, QoS, and MR handoff
 *
 * Programs the post-training uMCTL2 timing updates across Channel 0 and
 * Channel 1, configures MBUS/RAXI QoS ports, and issues the final LPDDR5 Mode
 * Register handoff sequence (MR16, MR14, MR15) across all ranks.
 */
static void mctl_apply_post_pmu_config(void)
{
	uintptr_t ctrl = SUNXI_DRAM_PHY_CTRL_BASE;

	mctl_write_regs_offset(dram_post_pmu_ch_regs,
			       ARRAY_SIZE(dram_post_pmu_ch_regs), 0x00000000UL);

	writew(0x0000, (void __iomem *)(ctrl + PMU_REG_APB_MUX));
	writew(0x0003, (void __iomem *)0x0aa80100UL);
	writew(0x0000, (void __iomem *)0x0aa80100UL);
	writew(0x0001, (void __iomem *)(ctrl + PMU_REG_APB_MUX));

	mctl_write_regs_offset(dram_post_pmu_ch_regs,
			       ARRAY_SIZE(dram_post_pmu_ch_regs), 0x00400000UL);

	mctl_write_regs(dram_post_pmu_tail_regs,
			ARRAY_SIZE(dram_post_pmu_tail_regs));

	writew(0x0000, (void __iomem *)(ctrl + PMU_REG_APB_MUX));
	mctl_write_mr_all(0U, 16, 0x05);
	mctl_write_mr_all(1U, 14, 0x00);
	mctl_write_mr_all(1U, 15, 0x00);
	mctl_write_mr_all(0U, 16, 0x04);
	writew(0x0001, (void __iomem *)(ctrl + PMU_REG_APB_MUX));
}

/**
 * mctl_stage2_pre_imem_init() - Program CCU PLL_DDR, dual-channel uMCTL2, and PHY init
 *
 * Configures the CCU DDR PLL and MBUS clocks, replicates the uMCTL2 controller
 * timing and QoS configuration across Channel 0 (0x0a100000) and Channel 1
 * (0x0a500000), and programs the pre-IMEM Cadence PHY registers.
 */
static void mctl_stage2_pre_imem_init(void)
{
	mctl_write_regs(dram_stage2_ccu_regs,
			ARRAY_SIZE(dram_stage2_ccu_regs));
	mctl_write_regs_offset(dram_stage2_umctl2_ch_regs,
			       ARRAY_SIZE(dram_stage2_umctl2_ch_regs),
			       0x00000000UL);
	mctl_write_regs_offset(dram_stage2_umctl2_ch_regs,
			       ARRAY_SIZE(dram_stage2_umctl2_ch_regs),
			       0x00400000UL);
	mctl_write_regs(dram_stage2_phy_init_regs,
			ARRAY_SIZE(dram_stage2_phy_init_regs));
}

/**
 * mctl_phy_init() - Initialize the Cadence DDR PHY and run ARC EM4 PMU training
 *
 * Programs Stage 2 pre-IMEM/pre-DMEM controller and PHY tables, loads the
 * clean-room ARC EM4 PMU firmware into IMEM and training parameter block into
 * DMEM, triggers the PMU microsequencer handshake, applies Stage 3 post-PMU
 * configuration, and waits for both uMCTL2 channels to enter Normal Mode.
 *
 * Return: 0 on successful training and normal mode entry, or negative errno.
 */
static int mctl_phy_init(void)
{
	int ret;

	/* Phase 1-5: Stage 2 Pre/Post Tables & PMU IMEM/DMEM Load */
	mctl_stage2_pre_imem_init();
	mctl_pmu_load_sram(SUNXI_DRAM_PHY_IMEM_BASE, pmu_imem_image,
			   ARRAY_SIZE(pmu_imem_image));

	mctl_write_regs(dram_stage2_pre_dmem_regs,
			ARRAY_SIZE(dram_stage2_pre_dmem_regs));
	mctl_pmu_load_sram(SUNXI_DRAM_PHY_DMEM_BASE,
			   (const u16 *)&dram_pmu_msg_block,
			   sizeof(dram_pmu_msg_block) / sizeof(u16));

	mctl_phy_load_pin_swizzle();

	/* Phase 6-7: PMU Execution Handshake Sequence */
	ret = mctl_pmu_run_training();
	if (ret)
		return ret;

	/* Phase 8: APB Reconnect & Stage 3 Post-Training Configuration */
	writew(0x0000,
	       (void __iomem *)(SUNXI_DRAM_PHY_CTRL_BASE + PMU_REG_APB_MUX));
	mctl_apply_stage3_config();
	mctl_apply_post_pmu_config();
	mctl_phy_dump_slices();

	/* Phase 9: Operating Mode Verification */
	ret = mctl_umctl2_wait_normal_mode(SUNXI_UMCTL2_CH0_STAT, 0);
	if (ret)
		return ret;

	return mctl_umctl2_wait_normal_mode(SUNXI_UMCTL2_CH1_STAT, 1);
}

/* ========================================================================= */
/* SECTION 5: Memory Diagnostics & Dynamic Capacity Probing       */
/* ========================================================================= */

/**
 * mctl_read_mr() - Read a JEDEC Mode Register from an active LPDDR5 rank
 * @ch: Controller channel index (0 or 1)
 * @rank: Rank index within @ch (0 or 1)
 * @mr_addr: Mode Register number (e.g. 5 for Manufacturer ID, 8 for Density/IO)
 *
 * Issues a Mode Register Read (MRR) command via the channel's uMCTL2 MRCTRL0
 * and MRCTRL1 registers, waits for the MR transaction bit (bit 31) to clear,
 * and extracts the 8-bit MR response from the DRAMC MR data register.
 *
 * Return: 8-bit Mode Register value.
 */
static u8 mctl_read_mr(unsigned int ch, unsigned int rank, u8 mr_addr)
{
	uintptr_t base = (ch == 0) ? UMCTL2_CH0_BASE : UMCTL2_CH1_BASE;
	uintptr_t mr_data_reg = (ch == 0) ? DRAMC_MR_DATA_CH0_REG :
					    DRAMC_MR_DATA_CH1_REG;
	u32 rank_sel = BIT(rank) << 4;
	u32 ctrl0 = rank_sel | 0x01U;

	writel(ctrl0, (void __iomem *)(base + UMCTL2_MRCTRL0_OFFSET));
	writel((u32)mr_addr << 8,
	       (void __iomem *)(base + UMCTL2_MRCTRL1_OFFSET));
	writel(ctrl0 | BIT(31),
	       (void __iomem *)(base + UMCTL2_MRCTRL0_OFFSET));
	mctl_poll_reg32(base + UMCTL2_MRCTRL0_OFFSET, BIT(31), 0U, 10000,
			"MR read", base + UMCTL2_MRCTRL0_OFFSET);
	udelay(10);

	return (u8)((readl((void __iomem *)mr_data_reg) >> 8) & 0xffU);
}

/**
 * mctl_mr8_rank_size_mb() - Decode JEDEC LPDDR5 MR8 into rank capacity in MiB
 * @mr8: Raw 8-bit JEDEC Mode Register 8 (JESD209-5) value
 *
 * Decodes the die density code (bits [5:2]) and I/O width (bits [7:6]) from
 * MR8. Handles both power-of-two dies and non-power-of-two 3/4-density dies
 * (such as 6 Gb, 12 Gb, and 24 Gb) as well as x8 byte-mode ranks where two
 * dies populate a 16-bit channel.
 *
 * Return: Rank capacity in MiB, or 0 if @mr8 is invalid.
 */
static unsigned long mctl_mr8_rank_size_mb(u8 mr8)
{
	u8 density_code = (mr8 >> 2) & 0x0fU;
	u8 io_width = (mr8 >> 6) & 0x03U;
	unsigned long die_mb;

	if (mr8 == 0x00U || mr8 == 0xffU || density_code > 8U)
		return 0;

	die_mb = ((density_code & 1U) ? 384UL : 256UL) <<
		 (density_code >> 1);

	/* In x8 byte mode (io_width == 1), two x8 dies form one 16-bit rank */
	if (io_width == 1U)
		die_mb <<= 1;

	return die_mb;
}

/**
 * sun60i_a733_get_dram_size() - Dynamically detect total physical DRAM capacity
 *
 * Queries the active channel mask and uMCTL2 MSTR register to determine the
 * active LPDDR5 ranks, then reads JEDEC Mode Register 8 (MR8) from each
 * populated channel and rank to compute exact capacity (including
 * non-power-of-two 3/4-density dies such as 6 GiB and 12 GiB). Falls back to
 * U-Boot's standard get_ram_size() address-aliasing probe if MR8 probing
 * yields 0.
 *
 * Return: Total detected DRAM capacity in bytes.
 */
static unsigned long sun60i_a733_get_dram_size(void)
{
	u32 mstr = readl((void __iomem *)(UMCTL2_CH0_BASE + UMCTL2_MSTR_OFFSET));
	u32 ch_mask = readl((void __iomem *)DRAMC_CH_EN_REG) & 0x03U;
	u32 rank_mask = (mstr >> UMCTL2_MSTR_RANKS_SHIFT) &
			UMCTL2_MSTR_RANKS_MASK;

	if (mstr & UMCTL2_MSTR_LPDDR5) {
		unsigned long total_mb = 0;
		unsigned int ch, rank;

		if (!ch_mask)
			ch_mask = 0x01U;

		for (ch = 0; ch < 2; ch++) {
			if (!(ch_mask & BIT(ch)))
				continue;

			for (rank = 0; rank < 2; rank++) {
				u8 mr8;

				if (!(rank_mask & BIT(rank)))
					continue;

				mr8 = mctl_read_mr(ch, rank, 8);
				total_mb += mctl_mr8_rank_size_mb(mr8);
			}
		}

		debug("DRAM: MR5 vendor=0x%02x, MR8 rank0=0x%02x -> %lu MiB\n",
		      mctl_read_mr(0, 0, 5), mctl_read_mr(0, 0, 8), total_mb);

		if (total_mb > 0)
			return total_mb << 20;
	}

	return get_ram_size((long *)CFG_SYS_SDRAM_BASE, SZ_2G);
}

/**
 * a733_test_channel_aperture() - Verify a 16 KiB test window on one DRAM channel
 * @base: Physical base address of the channel test aperture
 * @seed: 32-bit pattern seed added to the word index
 * @label: Channel label ("Ch0" or "Ch1") for error reporting
 *
 * Return: Number of word readback mismatches detected.
 */
static unsigned int a733_test_channel_aperture(uintptr_t base, u32 seed,
					       const char *label)
{
	unsigned int errors = 0;
	unsigned int i;

	for (i = 0; i < A733_BOOT0_TEST_WORDS; i++) {
		u32 exp = seed + i;
		u32 act = readl((void __iomem *)(base + (i * 4)));

		if (act != exp) {
			errors++;
			if (errors <= 5)
				printf("  %s mismatch @ 0x%08lx: exp 0x%08x, act 0x%08x\n",
				       label, (unsigned long)(base + (i * 4)),
				       exp, act);
		}
	}

	return errors;
}

/**
 * a733_dump_and_test_dram() - Run post-training dual-channel DRAM verification
 * @phase: Diagnostic phase label for debug/error logging
 *
 * Executes a sparse address probe across key physical apertures followed by
 * a 16 KiB dual-channel pattern write and readback verification across
 * Channel 0 (0x40000000) and Channel 1 (0xc0000000).
 */
static void a733_dump_and_test_dram(const char *phase)
{
	unsigned int probe_errors = 0;
	unsigned int ch0_errors, ch1_errors;
	unsigned int i;

	debug("DRAM test [%s]: Starting memory diagnostics...\n",
	      phase ? phase : "unspecified");

	/* 1. Single-Word Sparse Probe Test across 6 target addresses */
	for (i = 0; i < ARRAY_SIZE(sparse_probe_addrs); i++) {
		uintptr_t addr = sparse_probe_addrs[i];
		u32 pat = 0xa5a50000U | ((u32)(addr >> 16) & 0xffffU);
		u32 val;

		writel(pat, (void __iomem *)addr);
		dsb();
		isb();

		val = readl((void __iomem *)addr);
		if (val != pat) {
			probe_errors++;
			printf("  Probe [0x%08lx]: FAIL (exp 0x%08x, act 0x%08x)\n",
			       (unsigned long)addr, pat, val);
		} else {
			debug("  Probe [0x%08lx]: PASS (0x%08x)\n",
			      (unsigned long)addr, val);
		}
	}

	/* 2. Dual-Aperture Pattern Test (16 KiB Ch0 + 16 KiB Ch1) */
	for (i = 0; i < A733_BOOT0_TEST_WORDS; i++) {
		writel(0x01234567U + i,
		       (void __iomem *)(A733_DRAM_CH0_BASE + (i * 4)));
		writel(0xfedcba98U + i,
		       (void __iomem *)(A733_DRAM_CH1_BASE + (i * 4)));
	}

	dsb();
	isb();

	ch0_errors = a733_test_channel_aperture(A733_DRAM_CH0_BASE,
						0x01234567U, "Ch0");
	ch1_errors = a733_test_channel_aperture(A733_DRAM_CH1_BASE,
						0xfedcba98U, "Ch1");

	if (probe_errors || ch0_errors || ch1_errors) {
		printf("DRAM test [%s]: Probe errors=%u, Ch0 16KB errors=%u, Ch1 16KB errors=%u -> FAIL\n",
		       phase ? phase : "unspecified",
		       probe_errors, ch0_errors, ch1_errors);
	} else {
		debug("DRAM test [%s]: Probe errors=0, Ch0 16KB errors=0, Ch1 16KB errors=0 -> PASS\n",
		      phase ? phase : "unspecified");
	}
}

/* ========================================================================= */
/* SECTION 6: DMC Clocks & Master DRAM Initialization             */
/* ========================================================================= */

/**
 * sun60i_a733_dmc_clock_setup() - Configure PRCM DMC channel clocks and resets
 *
 * Configures dual-channel mode, sets the PRCM DMC feature registers for
 * LPDDR5 operation, and sequences reset deassertion and clock gating for
 * DRAM Channels 0 and 1.
 */
static void sun60i_a733_dmc_clock_setup(void)
{
	u32 val;

	/* Step 1: Configure dual-channel mode */
	val = readl((void __iomem *)SUNXI_PRCM_CH0_DMC_CLK_REG);
	val = (val & 0xffffc0c0U) | 0x2728U;
	writel(val, (void __iomem *)SUNXI_PRCM_CH0_DMC_CLK_REG);
	writel(val, (void __iomem *)SUNXI_PRCM_CH1_DMC_CLK_REG);

	/* Step 2: Configure DRAM memory type */
	val = readl((void __iomem *)SUNXI_PRCM_CH0_DMC_FEAT_REG);
	val = (val & 0xfffffffcU) | 0x03U;
	writel(val, (void __iomem *)SUNXI_PRCM_CH0_DMC_FEAT_REG);
	writel(val, (void __iomem *)SUNXI_PRCM_CH1_DMC_FEAT_REG);

	/* Step 3: Deassert channel resets and enable clock */
	val = readl((void __iomem *)SUNXI_PRCM_CH0_DMC_CLK_REG);
	val = (val & 0xffffffefU) | 0x14U;
	writel(val, (void __iomem *)SUNXI_PRCM_CH0_DMC_CLK_REG);
	writel(val, (void __iomem *)SUNXI_PRCM_CH1_DMC_CLK_REG);

	val = (val & 0xfffffffeU) | 0x02U;
	writel(val, (void __iomem *)SUNXI_PRCM_CH0_DMC_CLK_REG);
	writel(val, (void __iomem *)SUNXI_PRCM_CH1_DMC_CLK_REG);
	udelay(1);

	val |= 0x03U;
	writel(val, (void __iomem *)SUNXI_PRCM_CH0_DMC_CLK_REG);
	writel(val, (void __iomem *)SUNXI_PRCM_CH1_DMC_CLK_REG);
	udelay(1);
}

/**
 * sun60i_a733_dram_type_name() - Read active DRAM generation name from uMCTL2
 *
 * Inspects the active DRAM type bit in the Channel 0 uMCTL2 MSTR register
 * and returns a human-readable string.
 *
 * Return: Human-readable DRAM generation string.
 */
static const char *sun60i_a733_dram_type_name(void)
{
	u32 mstr = readl((void __iomem *)(UMCTL2_CH0_BASE + UMCTL2_MSTR_OFFSET));

	if (mstr & UMCTL2_MSTR_LPDDR5)
		return "LPDDR5";

	return "Unknown";
}

/**
 * sunxi_dram_init - Top-level DRAM subsystem initialization and bringup
 *
 * Return: Total operational DRAM capacity in bytes, or 0 on failure.
 */
unsigned long sunxi_dram_init(void)
{
	int ret;

	/* 3.1 Security & Power Bringup */
	sunxi_security_and_bus_init();

	ret = sunxi_pmic_init_a733();
	if (ret) {
		pr_err("DRAM: PMIC power initialization failed (%d)\n", ret);
		return 0;
	}

	/* 3.2 PRCM DMC Channel Clock Setup */
	sun60i_a733_dmc_clock_setup();

	/* 3.3 PHY Calibration & Training */
	ret = mctl_phy_init();
	if (ret) {
		pr_err("DRAM: Cadence PHY training failed (%d)\n", ret);
		dram_halt();
		return 0;
	}

	/* 3.4 Memory Diagnostics & Dynamic Capacity Detection */
	a733_dump_and_test_dram("Normal DRAM Read/Write Test");

	return sun60i_a733_get_dram_size();
}

/**
 * board_add_ram_info() - Report trained DRAM type, data rate, and latency
 * @use_default: Unused default-configuration flag
 */
void board_add_ram_info(int use_default)
{
	u32 pll = readl((void __iomem *)CCU_PLL_DDR_CTRL_REG);
	u32 tmg0 = readl((void __iomem *)UMCTL2_CH0_DRAMTMG0);
	u32 tmg2 = readl((void __iomem *)UMCTL2_CH0_DRAMTMG2);
	u32 tmg4 = readl((void __iomem *)UMCTL2_CH0_DRAMTMG4);
	u32 n = ((pll >> 8) & 0xffU) + 1U;
	u32 m1 = ((pll >> 1) & 0x01U) + 1U;
	u32 m0 = (pll & 0x01U) + 1U;
	u32 clk_mhz = (24U * n) / (m0 * m1);
	u32 cl = (tmg2 >> 16) & 0x3fU;
	u32 trcd = (tmg4 >> 24) & 0x1fU;
	u32 trp = tmg4 & 0x1fU;
	u32 tras = tmg0 & 0x3fU;

	(void)use_default;
	printf(" (%s, %u MT/s, CL=%u-%u-%u-%u)",
	       sun60i_a733_dram_type_name(),
	       clk_mhz * 2U, cl, trcd, trp, tras);
}
