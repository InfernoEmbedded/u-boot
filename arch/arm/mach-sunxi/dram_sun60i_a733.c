// SPDX-License-Identifier: GPL-2.0+
/*
 * Allwinner sun60i A733 / T736 DRAM Controller and Synopsys DDR PHY driver
 *
 * (C) Copyright 2026 Alastair D'Silva <alastair@d-silva.org>
 */

#include <hang.h>
#include <init.h>
#include <log.h>
#include <stdio.h>
#include <asm/arch/dram.h>
#include <asm/barriers.h>
#include <asm/io.h>
#include <linux/bitops.h>
#include <linux/build_bug.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/sizes.h>
#include <linux/types.h>

#include "pmu/sunxi_pmu_abi.h"

#define CCU_PLL_DDR_CTRL_REG		0x02002020UL
#define CCU_DRAM0_CLK_REG		0x02002c00UL
#define UMCTL2_CH0_BASE			0x0a110000UL
#define UMCTL2_MSTR_OFFSET		0x000UL
#define UMCTL2_MSTR_LPDDR5		BIT(19)

#ifdef CONFIG_XPL_BUILD
#include "dram_sun60i_a733_pmu_fw.h"

#ifdef SUN60I_A733_PMU_FW_MISSING
unsigned long sunxi_dram_init(void)
{
	(void)sun60i_a733_pmu_imem;
	pr_err("DRAM: ARC PMU firmware missing\n");
	return 0;
}
#else
#include "dram_sun60i_a733_tables.h"

/* ========================================================================= */
/* 1. Register Sequencer & Aperture Boundaries                               */
/* ========================================================================= */

#define SUNXI_MMIO_MIN_ADDR		0x02000000UL
#define SUNXI_MMIO_MAX_ADDR		0x40000000UL

#define CCU_PLL_DDR_LOCK_BIT		BIT(28)
#define CCU_PLL_DDR_OUTPUT_GATE		BIT(27)
#define CCU_MBUS_CLK_REG		0x02002580UL
#define CCU_MBUS_BGR_REG		0x02002584UL
#define CCU_DRAM_MUX_CLK_REG		0x02002588UL
#define CCU_DRAM0_BGR_REG		0x02002c0cUL

#define PRCM_MEM_ISOLATION_REG		0x07010250UL

#define UMCTL2_CH0_APERTURE_BASE	0x0a100000UL
#define UMCTL2_CH0_APERTURE_END		0x0a500000UL
#define UMCTL2_CH1_BASE			0x0a510000UL
#define UMCTL2_CH_STRIDE		0x00400000UL
#define UMCTL2_STAT_OFFSET		0x014UL
#define UMCTL2_MRCTRL0_OFFSET		0x080UL
#define UMCTL2_MRCTRL1_OFFSET		0x084UL
#define UMCTL2_PWRCTL_OFFSET		0x180UL
#define UMCTL2_SWCTL_OFFSET		0xc80UL
#define UMCTL2_SWSTAT_OFFSET		0xc84UL
#define UMCTL2_DERATEINT_OFFSET		0xc9cUL

#define UMCTL2_MSTR_RANKS_SHIFT		24
#define UMCTL2_MSTR_RANKS_MASK		0x0fU

#define DRAMC_SYS_CFG_REG		0x0a020010UL
#define DRAMC_CH_EN_REG			0x0a020014UL
#define DRAMC_MR_DATA_CH0_REG		0x0a02002cUL
#define DRAMC_MR_DATA_CH1_REG		0x0a020040UL
#define DRAMC_CH0_PORT_CFG_REG		0x0a02021cUL
#define DRAMC_CH0_PORT_EN_REG		0x0a020220UL
#define DRAMC_CH1_PORT_CFG_REG		0x0a020224UL
#define DRAMC_CH1_PORT_EN_REG		0x0a020228UL

/* ========================================================================= */
/* 2. Synopsys PHY & Controller Apertures                                    */
/* ========================================================================= */

#define SUNXI_UMCTL2_CH0_STAT		(UMCTL2_CH0_BASE + UMCTL2_STAT_OFFSET)
#define SUNXI_UMCTL2_CH1_STAT		(UMCTL2_CH1_BASE + UMCTL2_STAT_OFFSET)

#define SUNXI_PRCM_CH0_DMC_CLK_REG	0x07010310UL
#define SUNXI_PRCM_CH0_DMC_FEAT_REG	0x07010314UL
#define SUNXI_PRCM_CH1_DMC_CLK_REG	0x07410310UL
#define SUNXI_PRCM_CH1_DMC_FEAT_REG	0x07410314UL

#define SUNXI_DRAM_PHY_APB_BASE		0x0a900000UL
#define SUNXI_DRAM_PHY_APB_END		0x0ab00000UL
#define PHY_SLICE_STRIDE		0x2000UL
#define PHY_LANE_STRIDE			0x0200UL

#define PHY_DBYTE0_BASE			0x0a920000UL
#define PHY_DBYTE_CLK_GATE_OFF		0x001eUL
#define PHY_DBYTE0_DQ_SWIZZLE_BASE	0x0a920100UL
#define PHY_DBYTE_RX_EN_OFF		0x0112UL
#define PHY_DBYTE_TX_EN_OFF		0x0114UL
#define PHY_DBYTE_COMP_EN_OFF		0x0116UL
#define PHY_DBYTE_RX_BIAS_OFF		0x0118UL
#define PHY_DBYTE_RFIFO_OFF		0x0146UL
#define PHY_DBYTE_LCDL_OFF		0x0162UL
#define PHY_DBYTE_STAT_OFF		0x016aUL

#define PHY_PIE_PTR_LO_BASE		0x0a940248UL
#define PHY_PIE_PTR_HI_BASE		0x0a940316UL
#define PHY_AC0_SWIZZLE_BASE		0x0a960120UL
#define PHY_PIE_ZERO_SRAM_BASE		0x0a982000UL
#define PHY_PIE_INIT_SRAM_BASE		0x0a982010UL
#define PHY_PIE_SEQ_SRAM_BASE		0x0a982710UL
#define PHY_PIE_LANE_DELAY_BASE		0x0a9e0048UL
#define PHY_PIE_LANE_DELAY_INIT		0x5a3cU
#define PHY_ACSM_SRAM_BASE		0x0aa2005cUL
#define PHY_PUB_POST_PMU_CFG_REG	0x0aa80100UL

#ifdef DEBUG
#define A733_DRAM_CH0_BASE		0x40000000UL
#define A733_DRAM_CH1_BASE		0xc0000000UL
#define A733_DRAM_TEST_WORDS		4096

static const uintptr_t sparse_probe_addrs[] __section(".init.rodata") = {
	0x40000000UL, /* Ch0 Base */
	0x40800000UL, /* +8 MiB */
	0x44000000UL, /* +64 MiB, FIT image load aperture */
	0x4a000000UL, /* +160 MiB, U-Boot text base aperture */
	0x50000000UL, /* +256 MiB */
	0xc0000000UL, /* Ch1 Base */
};
#endif

/* ========================================================================= */
/* SECTION 1: Hardware Register Sequencer & State Polling                    */
/* ========================================================================= */

/**
 * mctl_poll_reg32() - Poll a 32-bit MMIO register until masked bits match
 * @addr: Physical address of status register to poll
 * @mask: Bitmask to apply to the register value
 * @expected: Expected masked value
 * @timeout_us: Maximum poll iterations (1 us per iteration)
 * @desc: Diagnostic description logged on timeout
 * @trig_addr: Trigger register address logged on timeout
 *
 * Return: 0 on success, or -ETIMEDOUT on timeout.
 */
static int mctl_poll_reg32(uintptr_t addr, u32 mask, u32 expected,
			   int timeout_us, const char *desc,
			   uintptr_t trig_addr)
{
	for (; timeout_us > 0; timeout_us--) {
		if ((readl((void __iomem *)addr) & mask) == expected)
			return 0;
		udelay(1);
	}

	pr_err("DRAM: %s timeout at 0x%08lx\n", desc,
	       (unsigned long)trig_addr);
	return -ETIMEDOUT;
}

/**
 * mctl_sync_hardware_state() - Synchronize hardware state after a register write
 * @addr: Physical MMIO address that was just written
 * @val: 32-bit value that was written to @addr
 *
 * Polls hardware status registers or inserts required settling delays when
 * specific control registers (PLL_DDR, uMCTL2 SWCTL/MRCTRL0/DFIMISC, or PRCM
 * memory isolation) are written during table replay.
 *
 * Return: 0 on success, or negative errno on poll failure.
 */
static int mctl_sync_hardware_state(uintptr_t addr, u32 val)
{
	/* CCU PLL_DDR lock before output gate enable */
	if (addr == CCU_PLL_DDR_CTRL_REG &&
	    (val & 0xe0000000U) == 0xe0000000U &&
	    !(val & CCU_PLL_DDR_OUTPUT_GATE)) {
		udelay(20);
		return mctl_poll_reg32(CCU_PLL_DDR_CTRL_REG, CCU_PLL_DDR_LOCK_BIT,
				       CCU_PLL_DDR_LOCK_BIT, 100000,
				       "PLL_DDR lock", addr);
	}

	/* uMCTL2 SWCTL -> SWSTAT quasi-dynamic register update ack */
	if ((addr == UMCTL2_CH0_BASE + UMCTL2_SWCTL_OFFSET ||
	     addr == UMCTL2_CH1_BASE + UMCTL2_SWCTL_OFFSET) && val == 1U) {
		return mctl_poll_reg32(addr - UMCTL2_SWCTL_OFFSET + UMCTL2_SWSTAT_OFFSET,
				       0x01U, 0x01U, 10000, "SWCTL handshake", addr);
	}

	/* uMCTL2 PWRCTL self-refresh exit to Normal Operating Mode */
	if ((addr == UMCTL2_CH0_BASE + UMCTL2_PWRCTL_OFFSET ||
	     addr == UMCTL2_CH1_BASE + UMCTL2_PWRCTL_OFFSET) && val == 0U) {
		uintptr_t stat_addr = addr - UMCTL2_PWRCTL_OFFSET +
				      UMCTL2_STAT_OFFSET;
		int ret;

		ret = mctl_poll_reg32(stat_addr, 0x03U << 4, 0U, 100000,
				      "Self-refresh exit", addr);
		if (ret)
			return ret;
		return mctl_poll_reg32(stat_addr, 0x03U, 0x01U, 100000,
				       "Normal mode entry", addr);
	}

	/* uMCTL2 MRCTRL0 Mode Register command completion */
	if ((addr == UMCTL2_CH0_BASE + UMCTL2_MRCTRL0_OFFSET ||
	     addr == UMCTL2_CH1_BASE + UMCTL2_MRCTRL0_OFFSET) &&
	    (val & BIT(31))) {
		return mctl_poll_reg32(addr, BIT(31), 0U, 10000,
				       "MR command trigger", addr);
	}

	/* PRCM memory isolation release settle delay */
	if (addr == PRCM_MEM_ISOLATION_REG) {
		mdelay(2);
		return 0;
	}

	return 0;
}

/**
 * mctl_write_single_reg() - Write a single MMIO register and synchronize state
 * @addr: Physical MMIO address
 * @val: Value to write (16-bit for DDR PHY APB, 32-bit otherwise)
 */
static int mctl_write_single_reg(uintptr_t addr, u32 val)
{
	if (addr < SUNXI_MMIO_MIN_ADDR || addr >= SUNXI_MMIO_MAX_ADDR)
		return -EINVAL;

	if (addr >= SUNXI_DRAM_PHY_APB_BASE && addr < SUNXI_DRAM_PHY_APB_END)
		writew((u16)val, (void __iomem *)addr);
	else
		writel(val, (void __iomem *)addr);

	return mctl_sync_hardware_state(addr, val);
}

/**
 * mctl_write_regs() - Replay an array of width- and broadcast-tagged MMIO writes
 * @regs: Pointer to array of struct sunxi_dram_reg descriptors
 * @count: Number of descriptors in @regs
 *
 * Writes each 16- or 32-bit register entry (expanding PHY_BCAST_DBYTE,
 * PHY_BCAST_AC, and PHY_BCAST_9LANE broadcast tags across the corresponding
 * PHY slices and lanes) and invokes mctl_sync_hardware_state() after each write.
 *
 * Return: 0 on success, or negative errno on sync timeout.
 */
static int mctl_write_regs(const struct sunxi_dram_reg *regs, u32 count)
{
	u32 i;
	int ret;

	if (!regs || !count)
		return 0;

	for (i = 0; i < count; i++) {
		u32 raw_addr = regs[i].addr;
		uintptr_t base = raw_addr & ~PHY_BCAST_MASK;
		u32 val = regs[i].val;
		unsigned int s, l;

		if (raw_addr & PHY_BCAST_9LANE) {
			for (s = 0; s < 4; s++) {
				for (l = 0; l < 9; l++) {
					ret = mctl_write_single_reg(base + s * PHY_SLICE_STRIDE +
								    l * PHY_LANE_STRIDE, val);
					if (ret)
						return ret;
				}
			}
		} else if (raw_addr & PHY_BCAST_DBYTE) {
			for (s = 0; s < 4; s++) {
				ret = mctl_write_single_reg(base + s * PHY_SLICE_STRIDE, val);
				if (ret)
					return ret;
			}
		} else if (raw_addr & PHY_BCAST_AC) {
			for (s = 0; s < 2; s++) {
				ret = mctl_write_single_reg(base + s * PHY_SLICE_STRIDE, val);
				if (ret)
					return ret;
			}
		} else {
			ret = mctl_write_single_reg(base, val);
			if (ret)
				return ret;
		}
	}

	return 0;
}

/**
 * mctl_write_regs_offset() - Replay an array of MMIO register writes with channel offset
 * @regs: Pointer to array of struct sunxi_dram_reg descriptors
 * @count: Number of descriptors in @regs
 * @ch_offset: Address offset added to controller registers (0x0a100000..0x0a4fffff)
 * @rank_mask: Active rank bitmask (0x03 for dual-rank, 0x01 for single-rank)
 *
 * Writes each register entry, adding @ch_offset to uMCTL2/port addresses while
 * leaving global PHY/CCU addresses unchanged, and invokes
 * mctl_sync_hardware_state() after each write.
 *
 * Return: 0 on success, or negative errno on sync timeout.
 */
static int mctl_write_regs_offset(const struct sunxi_dram_reg *regs, u32 count,
				  uintptr_t ch_offset, u8 rank_mask)
{
	u32 i;
	int ret;

	for (i = 0; i < count; i++) {
		uintptr_t addr = regs[i].addr;
		u32 val = regs[i].val;

		if (addr == UMCTL2_CH0_BASE + UMCTL2_MSTR_OFFSET)
			val = (val & ~(UMCTL2_MSTR_RANKS_MASK << UMCTL2_MSTR_RANKS_SHIFT)) |
			      ((u32)rank_mask << UMCTL2_MSTR_RANKS_SHIFT);
		else if (addr == UMCTL2_CH0_BASE + UMCTL2_DERATEINT_OFFSET)
			val = ((u32)rank_mask << 8) | rank_mask;

		if (addr >= UMCTL2_CH0_APERTURE_BASE &&
		    addr < UMCTL2_CH0_APERTURE_END)
			addr += ch_offset;

		if (addr >= SUNXI_DRAM_PHY_APB_BASE &&
		    addr < SUNXI_DRAM_PHY_APB_END)
			writew((u16)val, (void __iomem *)addr);
		else
			writel(val, (void __iomem *)addr);

		ret = mctl_sync_hardware_state(addr, val);
		if (ret)
			return ret;
	}

	return 0;
}

/**
 * mctl_write_mr_all() - Issue a JEDEC Mode Register Write across both channels and ranks
 * @mr_type: MRCTRL0 command type (0 for MRW, 1 for VRCG/FSP control)
 * @mr_addr: Mode Register number (e.g. 14, 15, 16)
 * @mr_val: 8-bit Mode Register payload
 *
 * Writes @mr_val to @mr_addr on active ranks of both Channel 0 and
 * Channel 1 via uMCTL2 MRCTRL0/MRCTRL1 and polls for completion.
 *
 * Return: 0 on success, or negative errno on sync timeout.
 */
static int mctl_write_mr_all(u32 mr_type, u8 mr_addr, u8 mr_val)
{
	unsigned int ch, rank;
	int ret;

	for (ch = 0; ch < 2; ch++) {
		uintptr_t base = (ch == 0) ? UMCTL2_CH0_BASE : UMCTL2_CH1_BASE;
		uintptr_t mrctrl0 = base + UMCTL2_MRCTRL0_OFFSET;
		uintptr_t mrctrl1 = base + UMCTL2_MRCTRL1_OFFSET;
		u32 ch_mstr = readl((void __iomem *)(base + UMCTL2_MSTR_OFFSET));
		u32 rank_mask = (ch_mstr >> UMCTL2_MSTR_RANKS_SHIFT) &
				UMCTL2_MSTR_RANKS_MASK;

		for (rank = 0; rank < 2; rank++) {
			u32 ctrl0 = (BIT(rank) << 4) | (mr_type & 1U);
			u32 ctrl1 = ((u32)mr_addr << 8) | mr_val;

			if (!(rank_mask & BIT(rank)))
				continue;

			writel(ctrl0, (void __iomem *)mrctrl0);
			writel(ctrl1, (void __iomem *)mrctrl1);
			writel(ctrl0 | BIT(31), (void __iomem *)mrctrl0);
			ret = mctl_sync_hardware_state(mrctrl0, ctrl0 | BIT(31));
			if (ret)
				return ret;
		}
	}

	return 0;
}

/* ========================================================================= */
/* SECTION 2: Synopsys DDR PHY & PMU Initializer                             */
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
 * Stalls the ARC core, switches the PHY APB mux back to host ARM access, and
 * reads the active training mode and assertion code from DMEM.
 */
static void mctl_pmu_log_error(const char *msg, u32 status)
{
	uintptr_t ctrl = SUNXI_DRAM_PHY_CTRL_BASE;
	uintptr_t dmem = SUNXI_DRAM_PHY_DMEM_BASE;
	size_t assert_off = offsetof(struct pmu_message_block, assert_code);
	size_t mode_off = offsetof(struct pmu_message_block, train_mode);
	u32 assert_code;
	u16 mode;

	writew(0x0001, (void __iomem *)(ctrl + PMU_REG_RESET));
	writew(0x0000, (void __iomem *)(ctrl + PMU_REG_APB_MUX));

	assert_code = (u32)readw((void __iomem *)(dmem + assert_off)) |
		      ((u32)readw((void __iomem *)(dmem + assert_off + 2)) << 16);
	mode = readw((void __iomem *)(dmem + mode_off));

	pr_err("DRAM: %s, status = 0x%08x (mode=0x%04x, assert=0x%08x)\n",
	       msg, status, mode, assert_code);
}

/**
 * mctl_pmu_run_training() - Hand over APB, trigger ARC EM4 PMU, and await completion
 * @log_err: Log training failures at error level (false for speculative probes)
 *
 * Description: Switches the PHY APB mux to the PMU microsequencer, releases
 * the ARC core from reset, and services the 4-phase Synopsys mailbox handshake
 * across any intermediate streaming messages until terminal completion or
 * fatal error.
 *
 * Return: 0 on successful training completion, or negative errno on failure.
 */
static int mctl_pmu_run_training(bool log_err)
{
	uintptr_t ctrl = SUNXI_DRAM_PHY_CTRL_BASE;
	u32 status = 0;
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
	for (timeout = 100000; timeout > 0; timeout--) {
		if (readw((void __iomem *)(ctrl + PMU_REG_POLL_STAT)) & 0x01)
			break;
		udelay(1);
	}
	if (!timeout) {
		if (log_err)
			mctl_pmu_log_error("PMU training start timeout", 0);
		else
			goto err_stall;
		return -ETIMEDOUT;
	}

	/* Service 4-phase PMU mailbox messages until terminal completion */
	timeout = 5000000;
	while (timeout > 0) {
		if (!(readw((void __iomem *)(ctrl + PMU_REG_POLL_STAT)) & 0x01)) {
			status = (u32)readw((void __iomem *)(ctrl + PMU_REG_STATUS_LO)) |
				 ((u32)readw((void __iomem *)(ctrl + PMU_REG_STATUS_HI)) << 16);

			/* Acknowledge message (DctWriteProt = 0) and await release */
			writew(0x0000, (void __iomem *)(ctrl + PMU_REG_HANDOFF_1));
			while (!(readw((void __iomem *)(ctrl + PMU_REG_POLL_STAT)) & 0x01)) {
				if (--timeout <= 0) {
					if (log_err)
						mctl_pmu_log_error("PMU mailbox ack timeout",
								   status);
					else
						goto err_stall;
					return -ETIMEDOUT;
				}
				udelay(1);
			}
			writew(0x0001, (void __iomem *)(ctrl + PMU_REG_HANDOFF_1));

			if (status == PMU_STATUS_SUCCESS) {
				writew(0x0001, (void __iomem *)(ctrl + PMU_REG_RESET));
				return 0;
			}
			if (status == PMU_STATUS_FATAL_ERROR) {
				if (log_err)
					mctl_pmu_log_error("PMU training failed",
							   status);
				else
					goto err_stall;
				return -EIO;
			}
			debug("DRAM: PMU mailbox msg 0x%08x\n", status);
			udelay(1);
			timeout--;
			continue;
		}
		udelay(1);
		timeout--;
	}

	if (log_err)
		mctl_pmu_log_error("PMU training completion timeout", 0);
err_stall:
	if (!log_err) {
		writew(0x0001, (void __iomem *)(ctrl + PMU_REG_RESET));
		writew(0x0000, (void __iomem *)(ctrl + PMU_REG_APB_MUX));
	}
	return (status == PMU_STATUS_FATAL_ERROR) ? -EIO : -ETIMEDOUT;
}

/**
 * mctl_phy_dump_slices() - Dump trained PHY slice delay registers for debug
 *
 * Temporarily switches the PHY APB mux to host access, logs key trained delay
 * registers across all four data slices, and restores the mux state.
 */
static void mctl_phy_dump_slices(void)
{
#ifdef DEBUG
	uintptr_t ctrl = SUNXI_DRAM_PHY_CTRL_BASE;
	unsigned int i;

	writew(0x0000, (void __iomem *)(ctrl + PMU_REG_APB_MUX));
	udelay(10);
	for (i = 0; i < 4; i++) {
		uintptr_t base = PHY_DBYTE0_BASE + (i * PHY_SLICE_STRIDE);

		debug("Slice %u: CLK1E=0x%04x RxEn=0x%04x TxEn=0x%04x CompEn=0x%04x RxBias=0x%04x LCDL162=0x%04x STAT16A=0x%04x RFIFO=0x%04x\n",
		      i,
		      readw((void __iomem *)(base + PHY_DBYTE_CLK_GATE_OFF)),
		      readw((void __iomem *)(base + PHY_DBYTE_RX_EN_OFF)),
		      readw((void __iomem *)(base + PHY_DBYTE_TX_EN_OFF)),
		      readw((void __iomem *)(base + PHY_DBYTE_COMP_EN_OFF)),
		      readw((void __iomem *)(base + PHY_DBYTE_RX_BIAS_OFF)),
		      readw((void __iomem *)(base + PHY_DBYTE_LCDL_OFF)),
		      readw((void __iomem *)(base + PHY_DBYTE_STAT_OFF)),
		      readw((void __iomem *)(base + PHY_DBYTE_RFIFO_OFF)));
	}
	writew(0x0001, (void __iomem *)(ctrl + PMU_REG_APB_MUX));
	udelay(10);
#endif
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
	int timeout;

	for (timeout = 100000; timeout > 0; timeout--) {
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

	if (!IS_ENABLED(CONFIG_DRAM_SUN60I_A733_PIN_SWIZZLE))
		return;

	writew(0x0001, (void __iomem *)(ctrl + PMU_REG_APB_MUX));
	writew(0x0000, (void __iomem *)(ctrl + PMU_REG_APB_MUX));

	for (ac = 0; ac < 2; ac++) {
		uintptr_t ac_base = PHY_AC0_SWIZZLE_BASE +
				    ac * PHY_SLICE_STRIDE;

		for (pin = 0; pin < 7; pin++)
			writew(dram_ac_pin_swizzle[ac][pin],
			       (void __iomem *)(ac_base + pin * 2));
	}

	for (pin = 0; pin < 8; pin++) {
		for (slice = 0; slice < 4; slice++) {
			uintptr_t dq_addr = PHY_DBYTE0_DQ_SWIZZLE_BASE +
					    slice * PHY_SLICE_STRIDE + pin * 2;

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
 * mctl_apply_pie_config() - Program PIE sequencer, ACSM SRAM, and PHY CSRs
 *
 * Initializes the PHY Init Engine (PIE) instruction tables, streams the ACSM
 * command sequence SRAM image, programs the FSP pointer tables and lane
 * calibration registers, and triggers the post-training PIE state machine.
 */
static int mctl_apply_pie_config(void)
{
	unsigned int i, lane, reg;
	int ret;

	ret = mctl_write_regs(dram_pie_head_regs,
			      ARRAY_SIZE(dram_pie_head_regs));
	if (ret)
		return ret;
	mctl_load_pie_table(PHY_PIE_INIT_SRAM_BASE, 176, dram_pie_init,
			    ARRAY_SIZE(dram_pie_init));
	ret = mctl_write_regs(dram_pie_phy_setup_regs,
			      ARRAY_SIZE(dram_pie_phy_setup_regs));
	if (ret)
		return ret;
	mctl_load_pie_table(PHY_PIE_ZERO_SRAM_BASE, 8, NULL, 0);
	mctl_load_pie_table(PHY_PIE_SEQ_SRAM_BASE, 2256, dram_pie_seq,
			    ARRAY_SIZE(dram_pie_seq));

	for (i = 0; i < ARRAY_SIZE(dram_acsm_sram); i++)
		writew(dram_acsm_sram[i],
		       (void __iomem *)(PHY_ACSM_SRAM_BASE + i * 2));

	ret = mctl_write_regs(dram_acsm_ctrl_regs,
			      ARRAY_SIZE(dram_acsm_ctrl_regs));
	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(dram_pie_ptrs) / 2; i++) {
		writew(dram_pie_ptrs[i * 2],
		       (void __iomem *)(PHY_PIE_PTR_LO_BASE + i * 2));
		writew(dram_pie_ptrs[i * 2 + 1],
		       (void __iomem *)(PHY_PIE_PTR_HI_BASE + i * 2));
	}

	ret = mctl_write_regs(dram_pie_patch_regs,
			      ARRAY_SIZE(dram_pie_patch_regs));
	if (ret)
		return ret;
	mctl_load_pie_table(PHY_PIE_INIT_SRAM_BASE, 176, dram_pie_init,
			    ARRAY_SIZE(dram_pie_init));
	ret = mctl_write_regs(dram_pie_pre_lane_regs,
			      ARRAY_SIZE(dram_pie_pre_lane_regs));
	if (ret)
		return ret;

	for (lane = 0; lane < 9; lane++) {
		uintptr_t lane_base = PHY_PIE_LANE_DELAY_BASE +
				      lane * PHY_LANE_STRIDE;

		for (reg = 0; reg < 8; reg++)
			writew(PHY_PIE_LANE_DELAY_INIT,
			       (void __iomem *)(lane_base + reg * 2));
	}

	return mctl_write_regs(dram_pie_tail_regs,
			       ARRAY_SIZE(dram_pie_tail_regs));
}

/**
 * mctl_apply_post_pmu_config() - Apply post-PMU uMCTL2 timing, QoS, and MR handoff
 * @rank_mask: Active rank bitmask (0x03 for dual-rank, 0x01 for single-rank)
 *
 * Programs the post-training uMCTL2 timing updates across Channel 0 and
 * Channel 1, configures MBUS/RAXI QoS ports, and issues the final LPDDR5 Mode
 * Register handoff sequence (MR16, MR14, MR15) across all ranks.
 *
 * Return: 0 on success, or negative errno on sync timeout.
 */
static int mctl_apply_post_pmu_config(u8 rank_mask)
{
	uintptr_t ctrl = SUNXI_DRAM_PHY_CTRL_BASE;
	int ret;

	ret = mctl_write_regs_offset(dram_post_pmu_ch_regs,
				     ARRAY_SIZE(dram_post_pmu_ch_regs),
				     0x00000000UL, rank_mask);
	if (ret)
		return ret;

	writew(0x0000, (void __iomem *)(ctrl + PMU_REG_APB_MUX));
	writew(0x0003, (void __iomem *)PHY_PUB_POST_PMU_CFG_REG);
	writew(0x0000, (void __iomem *)PHY_PUB_POST_PMU_CFG_REG);
	writew(0x0001, (void __iomem *)(ctrl + PMU_REG_APB_MUX));

	ret = mctl_write_regs_offset(dram_post_pmu_ch_regs,
				     ARRAY_SIZE(dram_post_pmu_ch_regs),
				     UMCTL2_CH_STRIDE, rank_mask);
	if (ret)
		return ret;

	ret = mctl_write_regs(dram_post_pmu_tail_regs,
			      ARRAY_SIZE(dram_post_pmu_tail_regs));
	if (ret)
		return ret;

	writew(0x0000, (void __iomem *)(ctrl + PMU_REG_APB_MUX));
	ret = mctl_write_mr_all(0U, 16, 0x05);
	if (ret)
		return ret;
	ret = mctl_write_mr_all(1U, 14, 0x00);
	if (ret)
		return ret;
	ret = mctl_write_mr_all(1U, 15, 0x00);
	if (ret)
		return ret;
	ret = mctl_write_mr_all(0U, 16, 0x04);
	if (ret)
		return ret;
	writew(0x0001, (void __iomem *)(ctrl + PMU_REG_APB_MUX));

	return 0;
}

/**
 * mctl_ccu_init() - Configure CCU PLL_DDR, MBUS, DRAMC ports, and PRCM isolation
 *
 * Resets and ungates the DRAM0 and MBUS bus interfaces, locks PLL_DDR at
 * 1800 MHz (3600 MT/s LPDDR5 WCK), enables dual-channel DRAMC AXI ports, and
 * releases PRCM DRAM power isolation.
 *
 * Return: 0 on success, or negative errno on sync timeout.
 */
static int mctl_ccu_init(void)
{
	u32 pll_n = clamp((u32)CONFIG_DRAM_CLK / 24U, 1U, 256U) - 1U;
	u32 n_bits = pll_n << 8;
	int ret;

	/* Reset MBUS and DRAM0 bus gates before reconfiguring PLL_DDR */
	writel(0x08000000, (void __iomem *)CCU_DRAM_MUX_CLK_REG);
	writel(0x00000000, (void __iomem *)CCU_MBUS_BGR_REG);
	writel(0x00000000, (void __iomem *)CCU_DRAM0_BGR_REG);
	writel(0x00010000, (void __iomem *)CCU_DRAM0_BGR_REG);
	writel(0x00010001, (void __iomem *)CCU_DRAM0_BGR_REG);

	writel(0x00000100, (void __iomem *)DRAMC_SYS_CFG_REG);
	writel(0x00002600, (void __iomem *)DRAMC_SYS_CFG_REG);

	/* Program PLL_DDR (24 MHz * (N + 1)), poll lock, then ungate */
	writel(0x10000000, (void __iomem *)CCU_PLL_DDR_CTRL_REG);
	writel(0x08000000, (void __iomem *)CCU_DRAM0_CLK_REG);
	writel(0x11000000U | n_bits, (void __iomem *)CCU_PLL_DDR_CTRL_REG);
	writel(0xd1000000U | n_bits, (void __iomem *)CCU_PLL_DDR_CTRL_REG);
	ret = mctl_write_single_reg(CCU_PLL_DDR_CTRL_REG, 0xf1000000U | n_bits);
	if (ret)
		return ret;
	writel(0xf9000000U | n_bits, (void __iomem *)CCU_PLL_DDR_CTRL_REG);

	/* Enable DRAM0 and MBUS clocks */
	writel(0x08000003, (void __iomem *)CCU_DRAM0_CLK_REG);
	writel(0x89000000, (void __iomem *)CCU_DRAM_MUX_CLK_REG);
	writel(0x00010000, (void __iomem *)CCU_MBUS_BGR_REG);
	writel(0x00010001, (void __iomem *)CCU_MBUS_BGR_REG);
	writel(0xcb000000, (void __iomem *)CCU_MBUS_CLK_REG);
	writel(0x88000003, (void __iomem *)CCU_DRAM0_CLK_REG);

	/* Enable dual-channel DRAMC and AXI host ports */
	writel(0x00002728, (void __iomem *)DRAMC_SYS_CFG_REG);
	writel(0x00000003, (void __iomem *)DRAMC_CH_EN_REG);
	writel(0x00000103, (void __iomem *)DRAMC_CH0_PORT_CFG_REG);
	writel(0x000013ff, (void __iomem *)DRAMC_CH0_PORT_EN_REG);
	writel(0x00000103, (void __iomem *)DRAMC_CH1_PORT_CFG_REG);
	writel(0x000013ff, (void __iomem *)DRAMC_CH1_PORT_EN_REG);
	writel(0x00000107, (void __iomem *)DRAMC_CH0_PORT_CFG_REG);
	writel(0x00000107, (void __iomem *)DRAMC_CH1_PORT_CFG_REG);

	/* Release PRCM DRAM power isolation */
	return mctl_write_single_reg(PRCM_MEM_ISOLATION_REG, 0x00000001);
}

/**
 * mctl_pre_pmu_init() - Program CCU PLL_DDR, dual-channel uMCTL2, and PHY init
 * @rank_mask: Active rank bitmask (0x03 for dual-rank, 0x01 for single-rank)
 *
 * Configures the CCU DDR PLL and MBUS clocks, replicates the uMCTL2 controller
 * timing and QoS configuration across Channel 0 (0x0a100000) and Channel 1
 * (0x0a500000), and programs the pre-PMU Synopsys PHY registers.
 *
 * Return: 0 on success, or negative errno on sync timeout.
 */
static int mctl_pre_pmu_init(u8 rank_mask)
{
	int ret;

	ret = mctl_ccu_init();
	if (ret)
		return ret;
	ret = mctl_write_regs_offset(dram_umctl2_ch_regs,
				     ARRAY_SIZE(dram_umctl2_ch_regs),
				     0x00000000UL, rank_mask);
	if (ret)
		return ret;
	ret = mctl_write_regs_offset(dram_umctl2_ch_regs,
				     ARRAY_SIZE(dram_umctl2_ch_regs),
				     UMCTL2_CH_STRIDE, rank_mask);
	if (ret)
		return ret;
	return mctl_write_regs(dram_phy_init_regs, ARRAY_SIZE(dram_phy_init_regs));
}

/**
 * mctl_phy_init() - Initialize the Synopsys DDR PHY and run ARC EM4 PMU training
 * @rank_mask: Active rank bitmask (0x03 for dual-rank, 0x01 for single-rank)
 *
 * Description: Programs pre-training controller and PHY registers, loads the
 * ARC EM4 PMU firmware into IMEM and training parameter block into DMEM, runs
 * the PMU microsequencer training handshake, applies post-training PIE and
 * uMCTL2 configuration, and waits for both channels to enter Normal Mode.
 *
 * Return: 0 on successful training and normal mode entry, or negative errno.
 */
static int mctl_phy_init(u8 rank_mask, bool log_err)
{
	struct pmu_message_block msg = dram_pmu_msg_block;
	int ret;

	BUILD_BUG_ON(sizeof(msg) != 0x106);
	BUILD_BUG_ON(ARRAY_SIZE(sun60i_a733_pmu_imem) > 0x8000);
	BUILD_BUG_ON(CONFIG_DRAM_CLK % 24 != 0);

	msg.dram_rate_mts = clamp((u32)CONFIG_DRAM_CLK / 24U, 1U, 256U) * 48U;
	msg.ch0_rank_en = rank_mask;
	msg.ch1_rank_en = rank_mask;
	if (!IS_ENABLED(CONFIG_DRAM_SUN60I_A733_PIN_SWIZZLE))
		msg.dq_swap_en = 0;

	ret = mctl_pre_pmu_init(rank_mask);
	if (ret)
		return ret;

	mctl_pmu_load_sram(SUNXI_DRAM_PHY_IMEM_BASE, sun60i_a733_pmu_imem,
			   ARRAY_SIZE(sun60i_a733_pmu_imem));

	ret = mctl_write_regs(dram_phy_pre_dmem_regs,
			      ARRAY_SIZE(dram_phy_pre_dmem_regs));
	if (ret)
		return ret;
	mctl_pmu_load_sram(SUNXI_DRAM_PHY_DMEM_BASE,
			   (const u16 *)&msg,
			   sizeof(msg) / sizeof(u16));

	mctl_phy_load_pin_swizzle();

	ret = mctl_pmu_run_training(log_err);
	if (ret)
		return ret;

	writew(0x0000,
	       (void __iomem *)(SUNXI_DRAM_PHY_CTRL_BASE + PMU_REG_APB_MUX));
	ret = mctl_apply_pie_config();
	if (ret)
		return ret;
	ret = mctl_apply_post_pmu_config(rank_mask);
	if (ret)
		return ret;
	mctl_phy_dump_slices();

	ret = mctl_umctl2_wait_normal_mode(SUNXI_UMCTL2_CH0_STAT, 0);
	if (ret)
		return ret;

	return mctl_umctl2_wait_normal_mode(SUNXI_UMCTL2_CH1_STAT, 1);
}

/* ========================================================================= */
/* SECTION 3: Memory Diagnostics & Dynamic Capacity Probing                  */
/* ========================================================================= */

/**
 * mctl_read_mr() - Read a JEDEC Mode Register from an active LPDDR5 rank
 * @ch: Controller channel index (0 or 1)
 * @rank: Rank index within @ch (0 or 1)
 * @mr_addr: Mode Register number (e.g. 5 for Manufacturer ID, 8 for Density/IO)
 * @val: Pointer to store 8-bit Mode Register value
 *
 * Issues a Mode Register Read (MRR) command via the channel's uMCTL2 MRCTRL0
 * and MRCTRL1 registers, waits for the MR transaction bit (bit 31) to clear,
 * and extracts the 8-bit MR response from the DRAMC MR data register.
 *
 * Return: 0 on success, or -ETIMEDOUT on poll timeout.
 */
static int mctl_read_mr(unsigned int ch, unsigned int rank, u8 mr_addr, u8 *val)
{
	uintptr_t base = (ch == 0) ? UMCTL2_CH0_BASE : UMCTL2_CH1_BASE;
	uintptr_t mr_data_reg = (ch == 0) ? DRAMC_MR_DATA_CH0_REG :
					    DRAMC_MR_DATA_CH1_REG;
	u32 rank_sel = BIT(rank) << 4;
	u32 ctrl0 = rank_sel | 0x01U;
	int ret;

	writel(ctrl0, (void __iomem *)(base + UMCTL2_MRCTRL0_OFFSET));
	writel((u32)mr_addr << 8,
	       (void __iomem *)(base + UMCTL2_MRCTRL1_OFFSET));
	writel(ctrl0 | BIT(31),
	       (void __iomem *)(base + UMCTL2_MRCTRL0_OFFSET));
	ret = mctl_poll_reg32(base + UMCTL2_MRCTRL0_OFFSET, BIT(31), 0U, 10000,
			      "MR read", base + UMCTL2_MRCTRL0_OFFSET);
	if (ret)
		return ret;

	udelay(10);

	*val = (u8)((readl((void __iomem *)mr_data_reg) >> 8) & 0xffU);
	return 0;
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
	u8 type = mr8 & 0x03U;
	u8 density_code = (mr8 >> 2) & 0x0fU;
	u8 io_width = (mr8 >> 6) & 0x03U;
	unsigned long die_mb;

	if (mr8 == 0x00U || mr8 == 0xffU || type > 2U || io_width > 1U ||
	    density_code > 8U)
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
 * Description: Queries the active channel mask and uMCTL2 MSTR register to
 * determine the active LPDDR5 ranks, then reads JEDEC Mode Register 8 (MR8)
 * from each populated channel and rank to compute exact capacity (including
 * non-power-of-two 3/4-density dies such as 6 Gb and 12 Gb). Falls back to
 * U-Boot's standard get_ram_size() address-aliasing probe if MR8 probing
 * yields 0.
 *
 * Return: Total detected DRAM capacity in bytes.
 */
static unsigned long sun60i_a733_get_dram_size(void)
{
	u32 ch0_mstr = readl((void __iomem *)(UMCTL2_CH0_BASE + UMCTL2_MSTR_OFFSET));
	u32 ch_mask = readl((void __iomem *)DRAMC_CH_EN_REG) & 0x03U;

	if (ch0_mstr & UMCTL2_MSTR_LPDDR5) {
		unsigned long total_mb = 0;
		unsigned int ch, rank;

		if (!ch_mask)
			ch_mask = 0x01U;

		for (ch = 0; ch < 2; ch++) {
			uintptr_t ch_base = (ch == 0) ? UMCTL2_CH0_BASE : UMCTL2_CH1_BASE;
			u32 mstr = readl((void __iomem *)(ch_base + UMCTL2_MSTR_OFFSET));
			u32 rank_mask = (mstr >> UMCTL2_MSTR_RANKS_SHIFT) &
					UMCTL2_MSTR_RANKS_MASK;

			if (!(ch_mask & BIT(ch)))
				continue;

			for (rank = 0; rank < 2; rank++) {
				u8 mr8;

				if (!(rank_mask & BIT(rank)))
					continue;

				if (mctl_read_mr(ch, rank, 8, &mr8) < 0)
					continue;

				total_mb += mctl_mr8_rank_size_mb(mr8);
			}
		}

#ifdef DEBUG
		{
			u8 mr5 = 0, mr8_r0 = 0;

			mctl_read_mr(0, 0, 5, &mr5);
			mctl_read_mr(0, 0, 8, &mr8_r0);
			debug("DRAM: MR5 mfg_id=0x%02x, MR8 rank0=0x%02x -> %lu MiB\n",
			      mr5, mr8_r0, total_mb);
		}
#endif

		if (total_mb > 0)
			return total_mb << 20;
	}

	return get_ram_size((long *)CFG_SYS_SDRAM_BASE, SZ_4G);
}

#ifdef DEBUG
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

	for (i = 0; i < A733_DRAM_TEST_WORDS; i++) {
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
 * @dram_size: Total detected physical DRAM capacity in bytes
 * @phase: Diagnostic phase label for debug/error logging
 *
 * Executes a sparse address probe across key physical apertures followed by
 * a 16 KiB dual-channel pattern write and readback verification across
 * Channel 0 (0x40000000) and Channel 1 (0xc0000000, if populated).
 */
static void a733_dump_and_test_dram(unsigned long dram_size, const char *phase)
{
	unsigned int probe_errors = 0;
	unsigned int ch0_errors = 0, ch1_errors = 0;
	unsigned int i;

	debug("DRAM test [%s]: Starting memory diagnostics...\n",
	      phase ? phase : "unspecified");

	/* 1. Single-Word Sparse Probe Test across target addresses within bounds */
	for (i = 0; i < ARRAY_SIZE(sparse_probe_addrs); i++) {
		uintptr_t addr = sparse_probe_addrs[i];
		u32 pat = 0xa5a50000U | ((u32)(addr >> 16) & 0xffffU);
		u32 val;

		if (addr >= CFG_SYS_SDRAM_BASE + dram_size)
			continue;

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

	/* 2. Dual-Aperture Pattern Test (16 KiB Ch0 + 16 KiB Ch1 if in bounds) */
	for (i = 0; i < A733_DRAM_TEST_WORDS; i++) {
		writel(0x01234567U + i,
		       (void __iomem *)(A733_DRAM_CH0_BASE + (i * 4)));
		if (CFG_SYS_SDRAM_BASE + dram_size > A733_DRAM_CH1_BASE + (i * 4)) {
			writel(0xfedcba98U + i,
			       (void __iomem *)(A733_DRAM_CH1_BASE + (i * 4)));
		}
	}

	dsb();
	isb();

	ch0_errors = a733_test_channel_aperture(A733_DRAM_CH0_BASE,
						0x01234567U, "Ch0");
	if (CFG_SYS_SDRAM_BASE + dram_size > A733_DRAM_CH1_BASE) {
		ch1_errors = a733_test_channel_aperture(A733_DRAM_CH1_BASE,
							0xfedcba98U, "Ch1");
	}

	if (probe_errors || ch0_errors || ch1_errors) {
		printf("DRAM test [%s]: Probe errors=%u, Ch0 16KB errors=%u, Ch1 16KB errors=%u -> FAIL\n",
		       phase ? phase : "unspecified",
		       probe_errors, ch0_errors, ch1_errors);
	} else {
		debug("DRAM test [%s]: Probe errors=0, Ch0 16KB errors=0, Ch1 16KB errors=0 -> PASS\n",
		      phase ? phase : "unspecified");
	}
}
#else
static inline void a733_dump_and_test_dram(unsigned long dram_size __maybe_unused,
					   const char *phase __maybe_unused)
{
}
#endif

/* ========================================================================= */
/* SECTION 4: DMC Clocks & Master DRAM Initialization                        */
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
	void __iomem *ch0_clk = (void __iomem *)SUNXI_PRCM_CH0_DMC_CLK_REG;
	void __iomem *ch1_clk = (void __iomem *)SUNXI_PRCM_CH1_DMC_CLK_REG;
	void __iomem *ch0_feat = (void __iomem *)SUNXI_PRCM_CH0_DMC_FEAT_REG;
	void __iomem *ch1_feat = (void __iomem *)SUNXI_PRCM_CH1_DMC_FEAT_REG;

	/* Step 1: Configure dual-channel mode */
	clrsetbits_le32(ch0_clk, 0x3f3fU, 0x2728U);
	writel(readl(ch0_clk), ch1_clk);

	/* Step 2: Configure DRAM memory type */
	clrsetbits_le32(ch0_feat, 0x03U, 0x03U);
	writel(readl(ch0_feat), ch1_feat);

	/* Step 3: Pulse channel resets and enable clock */
	clrsetbits_le32(ch0_clk, 0x13U, 0x14U);
	writel(readl(ch0_clk), ch1_clk);
	udelay(1);

	clrsetbits_le32(ch0_clk, 0x01U, 0x02U);
	writel(readl(ch0_clk), ch1_clk);
	udelay(1);

	setbits_le32(ch0_clk, 0x03U);
	writel(readl(ch0_clk), ch1_clk);
	udelay(1);
}

/**
 * sunxi_dram_init - Top-level DRAM subsystem initialization and bringup
 *
 * Return: Total operational DRAM capacity in bytes (hangs on training failure).
 */
unsigned long sunxi_dram_init(void)
{
	unsigned long dram_size;
	int ret;

	/* Allow LPDDR5 VDD1/VDD2/VDDQ rails configured by SPL PMIC init to settle */
	mdelay(10);

	/* PRCM DMC Channel Clock Setup */
	sun60i_a733_dmc_clock_setup();

	/* Try dual-rank training first, falling back to single-rank */
	ret = mctl_phy_init(0x03, false);
	if (ret) {
		debug("DRAM: Dual-rank training failed (%d), retrying single-rank\n",
		      ret);
		sun60i_a733_dmc_clock_setup();
		ret = mctl_phy_init(0x01, true);
	}
	if (ret) {
		pr_err("DRAM: Synopsys PHY training failed (%d)\n", ret);
		hang();
	}

	dram_size = sun60i_a733_get_dram_size();

	/* Optional Debug Diagnostics */
	a733_dump_and_test_dram(dram_size, "Normal DRAM Read/Write Test");

	return dram_size;
}
#endif /* !SUN60I_A733_PMU_FW_MISSING */
#else /* !CONFIG_XPL_BUILD */
/**
 * sun60i_a733_dram_type_name() - Read active DRAM generation name from uMCTL2
 *
 * Description: Inspects the active DRAM type bit in the Channel 0 uMCTL2 MSTR
 * register and returns a human-readable string.
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
 * board_add_ram_info() - Report trained DRAM type and effective data rate
 * @use_default: Unused default-configuration flag
 */
void board_add_ram_info(int use_default __maybe_unused)
{
	u32 pll = readl((void __iomem *)CCU_PLL_DDR_CTRL_REG);
	u32 n = ((pll >> 8) & 0xffU) + 1U;
	u32 m_out = ((pll >> 20) & 0x07U) + 1U;
	u32 p_in = ((pll >> 1) & 0x01U) + 1U;
	u32 pll_freq_mhz = (24U * n) / (p_in * m_out);
	u32 dram_clk_reg = readl((void __iomem *)CCU_DRAM0_CLK_REG);
	u32 m_dram = (dram_clk_reg & 0x1fU) + 1U;
	u32 ck_mhz = pll_freq_mhz / m_dram;
	u32 rate_mts = ck_mhz * 8U;

	printf(" (%s, %u MT/s)",
	       sun60i_a733_dram_type_name(),
	       rate_mts);
}
#endif /* CONFIG_XPL_BUILD */
