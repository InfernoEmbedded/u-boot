// SPDX-License-Identifier: GPL-2.0+
/*
 * Allwinner sun60i A733 / T736 DRAM Controller and Synopsys DWC DDR PHY driver
 *
 * (C) Copyright 2026 Alastair D'Silva <alastair@d-silva.org>
 * Assisted-by: Antigravity <antigravity@google.com>
 */

#include <init.h>
#include <log.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/dram.h>
#include <asm/arch/cpu.h>
#include <asm/arch/prcm.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <asm/armv8/mmu.h>
#include <hang.h>
#include <asm/arch/spl_spi.h>

/*
 * IOMEM access macro
 */
#ifndef IOMEM
#define IOMEM(addr)		((void __iomem *)(uintptr_t)(addr))
#endif

#include "dram_sun60i_a733_fw.h"
#include "dram_sun60i_a733_tables.h"
#include "pmu/dwc_ddrphy_pmu_regs.h"

#define DRAM_CACHE_MAGIC       0x4452414d  /* "DRAM" */
#define DRAM_CACHE_VERSION     4
#define DRAM_CACHE_SPI_SECTOR  0x003f0000  /* 4032 KB offset in SPI NOR (sector 1008) */
#define DRAM_RETENTION_SRAM    0x0008f000  /* Free on-chip SRAM before stack */
#define DRAM_CACHE_MAX_REGS    600

/*
 * =============================================================================
 * ALLWINNER A733 REFERENCE MANUAL REGISTER & PERIPHERAL BASE DEFINITIONS
 * =============================================================================
 */

/* Clock Controller Unit (CCU - User Manual Section 4.1.4, Base 0x02002000) */
#define SUNXI_CCU_BASE				0x02002000UL
#define CCU_PLL_DDR_CTRL_REG			(SUNXI_CCU_BASE + 0x0020) /* Sec 4.1.6.1: DRAM PLL control */
#define CCU_PLL_DDR_LOCK_BIT			(1U << 28)                /* Bit 28: 1 = PLL locked & stable */
#define CCU_NSI_CLK_REG				(SUNXI_CCU_BASE + 0x0580) /* NSI interconnect clock */
#define CCU_MBUS_BGR_REG			(SUNXI_CCU_BASE + 0x0584) /* MBUS bus gating & reset */
#define CCU_MBUS_CLK_REG			(SUNXI_CCU_BASE + 0x0588) /* Sec 4.1.6.72: MBUS clock control */
#define CCU_AHB_MAT_CLK_GATING_REG		(SUNXI_CCU_BASE + 0x05C0) /* Sec 4.1.6.82: AHB Master clock gating */
#define CCU_MBUS_MAT_CLK_GATING_REG		(SUNXI_CCU_BASE + 0x05E0) /* Sec 4.1.6.84: MBUS Master clock gating */
#define CCU_MBUS_GATE_EN_REG			(SUNXI_CCU_BASE + 0x05E4) /* Sec 4.1.6.85: MBUS Gate enable */
#define CCU_DRAM0_CLK_REG			(SUNXI_CCU_BASE + 0x0C00) /* Sec 4.1.6.123: DRAM0 clock control */
#define CCU_DRAM0_BGR_REG			(SUNXI_CCU_BASE + 0x0C0C) /* Sec 4.1.6.124: DRAM0 bus gating & reset */
#define CCU_SEC_CTRL_START_REG			(SUNXI_CCU_BASE + 0x0804) /* CCU bus security start */
#define CCU_SEC_CTRL_END_REG			(SUNXI_CCU_BASE + 0x08E4) /* CCU bus security end */
#define CCU_SEC_SWITCH_REG			(SUNXI_CCU_BASE + 0x1F00) /* CCMU Security Switch register */

/* Standby Power Reset Clock Management (STBY_PRCM - User Manual Sec 4.2.4, Base 0x07010000) */
#define SUNXI_STBY_PRCM_BASE			0x07010000UL
#define SUNXI_PRCM_SEC_BASE			0x07002000UL /* PRCM Bus Security Base */
#define PRCM_SEC_CTRL_REG0			(SUNXI_PRCM_SEC_BASE + 0x0004)
#define PRCM_SEC_CTRL_REG1			(SUNXI_PRCM_SEC_BASE + 0x0014)
#define PRCM_SEC_CTRL_REG2			(SUNXI_PRCM_SEC_BASE + 0x0024)
#define PRCM_S_TWI_BGR_REG			(SUNXI_STBY_PRCM_BASE + 0x019C) /* Sec 4.2.5: R_TWI gating & reset */
#define PRCM_RTC_BGR_REG			(SUNXI_STBY_PRCM_BASE + 0x020C) /* RTC gating & reset */
#define PRCM_BGR_GATING_ENABLE			BIT(0)
#define PRCM_BGR_RST_DEASSERT			BIT(16)
#define PRCM_BGR_UNRESET_GATE			(PRCM_BGR_RST_DEASSERT | PRCM_BGR_GATING_ENABLE)
#define PRCM_VCC_DRAM_ISO_REG			(SUNXI_STBY_PRCM_BASE + 0x0250) /* DRAM Power Isolation Control */
#define PRCM_VCC_DRAM_ISO_RELEASE		BIT(0)
#define PRCM_SEC_SWITCH_REG			(SUNXI_STBY_PRCM_BASE + 0x0290) /* PRCM Security Switch */
#define PRCM_DRAM_CH0_CLK_REG			(SUNXI_STBY_PRCM_BASE + 0x0310) /* DRAM Channel 0 clock control */
#define PRCM_DRAM_CH0_CFG_REG			(SUNXI_STBY_PRCM_BASE + 0x0314) /* DRAM Channel 0 configuration */
#define PRCM_DRAM_CH1_CLK_REG			(SUNXI_STBY_PRCM_BASE + 0x00400000 + 0x0310) /* DRAM Channel 1 clock control */
#define PRCM_DRAM_CH1_CFG_REG			(SUNXI_STBY_PRCM_BASE + 0x00400000 + 0x0314) /* DRAM Channel 1 configuration */

/* CPU Subsystem Control (CPU_SUBSYS_CTRL - User Manual Sec 5.1.5.1, Base 0x08000000) */
#define SUNXI_CPU_SUBSYS_CTRL_BASE		0x08000000UL
#define CPU_DA_DDR_CTRL_REG			(SUNXI_CPU_SUBSYS_CTRL_BASE + 0x0200) /* Sec 5.1.6.5: Direct Access DDR Mux */
#define CPU_DA_DDR_MUX_NSI			0x0 /* CPU accesses DDR through NSI Interconnect */
#define CPU_DA_DDR_MUX_AXI2HIF			0x1 /* CPU accesses DDR through AXI2HIF Direct Interface */

/* Real-Time Clock / Retention Registers (RTC - Base 0x07090000) */
#ifndef SUNXI_RTC_BASE
#define SUNXI_RTC_BASE				0x07090000UL
#endif
#define RTC_GP_DATA_REG0			(SUNXI_RTC_BASE + 0x0100) /* General Purpose Retention Data Reg 0 */
#define DRAM_RTC_MAGIC_REG			RTC_GP_DATA_REG0

/* DRAMC Common / Security Registers (DRAMC Common - User Manual Sec 3.2, Base 0x0A020000) */
#define SUNXI_DRAMC_COMMON_BASE			0x0a020000UL
#define DRAMC_MAER_REG0				(SUNXI_DRAMC_COMMON_BASE + 0x1800) /* Memory Access Enable / Firewall 0 */
#define DRAMC_MAER_REG1				(SUNXI_DRAMC_COMMON_BASE + 0x1808) /* Memory Access Enable / Firewall 1 */

/* Security Protection Controller (SPC) & System Interconnect Registers */
#define SUNXI_SPC_BASE				0x02054000UL /* Security Protection Controller Base */
#define SPC_PORT_START_REG			(SUNXI_SPC_BASE + 0x0000)
#define SPC_PORT_END_REG			(SUNXI_SPC_BASE + 0x005C)
#define SUNXI_IOMMU0_AUTO_BYPASS_REG		0x03900030UL
#define SUNXI_IOMMU1_AUTO_BYPASS_REG		0x03910030UL
#define SYSCTRL_RESCAL_CTRL_REG			0x03000160UL
#define SYSCTRL_RESCAL_CAL_REG			0x03000168UL

/* Synopsys DWC DDR PHY Physical Address Map (Host ARM perspective) */
#define SUNXI_DRAM_PHY_DBYTE_BASE		0x0a920000UL /* DBYTE Slices Base */
#define SUNXI_DRAM_PHY_DBYTE_STRIDE		0x00002000UL /* DBYTE Slice Stride (8KB) */
#define SUNXI_DRAM_PHY_COMMON_BASE		0x0a940000UL /* Common PHY Control / Status Base */
#define SUNXI_DRAM_PHY_AC_SLICE_BASE		0x0a960000UL /* AC Slices Base */
#define SUNXI_DRAM_PHY_AC_SLICE_STRIDE		0x00002000UL /* AC Slice Stride (8KB) */
#define SUNXI_DRAM_PHY_SHADOW_RAM_BASE		0x0a982000UL /* PHY Shadow RAM Base */
#define SUNXI_DRAM_PHY_STREAM29_BASE		0x0a982710UL /* PHY Stream 29 Base */
#define SUNXI_DRAM_PHY_AC_BASE			0x0a9e0000UL /* Address/Command Slice Base */
#define SUNXI_DRAM_PHY_PUB_STREAM_BASE		0x0aa2005cUL /* PUB Configuration Stream Base */
#define SUNXI_DRAM_PHY_PUB_BASE			0x0aa80000UL /* PHY Utility Block (PUB) Base */
#define PHY_REG_PUB_MICRO_RST			0x0100       /* PMU microsequencer reset/stall */

/*
 * Synopsys uMCTL2 Dual-Channel Memory Controller (User Manual Sec 3.2)
 *
 * Each channel consists of:
 * - Core registers: DRAM timing, address mapping, AXI port arbitration
 *   Channel 0 @ 0x0a100000, Channel 1 @ 0x0a500000 (stride 0x00400000)
 * - Subsystem CSRs: Controller status, mode registers, derating, SWCTL
 *   Channel 0 @ 0x0a110000, Channel 1 @ 0x0a510000 (stride 0x00400000)
 */
#define SUNXI_UMCTL2_CH_STRIDE			0x00400000UL
#define SUNXI_UMCTL2_CORE_CH0_BASE		0x0a100000UL /* Channel 0 uMCTL2 Core base */
#define SUNXI_UMCTL2_CORE_CH1_BASE		0x0a500000UL /* Channel 1 uMCTL2 Core base */
#define SUNXI_UMCTL2_CH0_BASE			0x0a110000UL /* Channel 0 uMCTL2 CSR base */
#define SUNXI_UMCTL2_CH1_BASE			0x0a510000UL /* Channel 1 uMCTL2 CSR base */

/* uMCTL2 Core Timing Registers (offsets relative to CORE base 0x0a100000) */
#define UMCTL2_REG_DRAMTMG0			0x0000
#define UMCTL2_REG_DRAMTMG1			0x0004
#define UMCTL2_REG_DRAMTMG2			0x0008
#define UMCTL2_REG_DRAMTMG3			0x000c
#define UMCTL2_REG_DRAMTMG4			0x0010
#define UMCTL2_REG_DRAMTMG5			0x0014
#define UMCTL2_REG_DRAMTMG6			0x0018
#define UMCTL2_REG_DRAMTMG7			0x001c
#define UMCTL2_REG_DRAMTMG9			0x0024
#define UMCTL2_REG_INIT0			0x0030
#define UMCTL2_REG_INIT1			0x0034
#define UMCTL2_REG_INIT2			0x0038
#define UMCTL2_REG_ODTCFG			0x005c
#define UMCTL2_REG_RFSHTMG0			0x0060
#define UMCTL2_REG_RFSHTMG1			0x0064
#define UMCTL2_REG_ZQCTL0			0x0078
#define UMCTL2_REG_DFITMG0			0x0080

/* uMCTL2 Core Address Mapping Registers (offsets relative to CORE base 0x0a100000) */
#define UMCTL2_REG_ADDRMAP0			0x0580       /* CS / Rank bit mapping */
#define UMCTL2_REG_ADDRMAP1			0x0584       /* Bank bit mapping (B0..B2) */
#define UMCTL2_REG_ADDRMAP2			0x0588       /* Column bit mapping (Col2..Col5) */
#define UMCTL2_REG_ADDRMAP4			0x0590       /* Column bit mapping (Col10..Col13) */
#define UMCTL2_REG_ADDRMAP5			0x0594       /* Row bit mapping (Row0..Row1, Row2_10) */

/* uMCTL2 Core AXI Port Arbitration (offsets relative to CORE base 0x0a100000) */
#define UMCTL2_PORT_STRIDE			0x00100000UL
#define UMCTL2_REG_PORT_PCTRL			0x0c00
#define UMCTL2_PORT_PRIO_GATE_EN		0x000000ff   /* Enable priority & gating on all 8 AXI channels */

/* uMCTL2 Subsystem CSRs (offsets relative to CSR base 0x0a110000) */
#define UMCTL2_REG_STAT				0x0014       /* Operating Status: [2:0] 1 = Normal Mode */
#define UMCTL2_STAT_NORMAL_MODE			0x1
#define UMCTL2_REG_MRCTRL0			0x0080       /* Mode Register Control 0 */
#define UMCTL2_REG_MRCTRL1			0x0084       /* Mode Register Control 1 */
#define UMCTL2_REG_MRSTAT			0x0090       /* Mode Register Status: bit 0 = mr_busy */
#define UMCTL2_REG_PWRCTL			0x0180       /* Low power control */
#define UMCTL2_REG_PWRSTAT			0x0184       /* Low power status */
#define UMCTL2_REG_RFSHCTL3			0x0208       /* Refresh control 3 */
#define UMCTL2_REG_PCTRL_0			0x0490       /* Port 0 control */
#define UMCTL2_REG_DERATEEN			0x0510       /* Temperature derate enable */
#define UMCTL2_REG_DERATEINT			0x0514       /* Temperature derate interval */
#define UMCTL2_REG_DQSCFG0			0x0b84       /* DQS calibration trigger */
#define UMCTL2_REG_DQSSTAT0			0x0b88       /* DQS calibration status */
#define UMCTL2_REG_SWCTL			0x0c80       /* Software done control */
#define UMCTL2_REG_SWSTAT			0x0c84       /* Software done status: bit 0 = sw_done_ack */
#define UMCTL2_REG_DFISTAT			0x0c88       /* DFI interface status */

/* uMCTL2 Register Bitfield Definitions */
#define UMCTL2_SWCTL_SW_DONE			BIT(0)
#define UMCTL2_SWSTAT_SW_DONE_ACK		BIT(0)
#define UMCTL2_MRSTAT_BUSY			BIT(0)
#define UMCTL2_MRCTRL0_CMD_READ			BIT(0)
#define UMCTL2_MRCTRL0_RANK0			BIT(4)
#define UMCTL2_MRCTRL0_RANK1			BIT(5)
#define UMCTL2_MRCTRL0_TRIGGER			BIT(31)

/* uMCTL2 Temperature Derating Control (DERATEEN) Bitfields */
#define UMCTL2_DERATEEN_EN			BIT(0)
#define UMCTL2_DERATEEN_VAL_2			BIT(2)
#define UMCTL2_DERATEEN_BYTE			BIT(4)
#define UMCTL2_DERATEEN_MODE_32			BIT(5)
#define UMCTL2_DERATEEN_MR_RD			BIT(12)
#define UMCTL2_DERATEEN_MR4_RD			BIT(16)

/* Temperature derating intermediate and final configurations for quasi-dynamic commit */
#define UMCTL2_DERATEEN_STAGE1_VAL		(UMCTL2_DERATEEN_MR4_RD | UMCTL2_DERATEEN_VAL_2)
#define UMCTL2_DERATEEN_STAGE2_VAL		(UMCTL2_DERATEEN_MR4_RD | UMCTL2_DERATEEN_BYTE | UMCTL2_DERATEEN_VAL_2)
#define UMCTL2_DERATEEN_NORMAL_VAL		(UMCTL2_DERATEEN_MR4_RD | UMCTL2_DERATEEN_MR_RD | \
						 UMCTL2_DERATEEN_BYTE | UMCTL2_DERATEEN_VAL_2 | \
						 UMCTL2_DERATEEN_EN)
#define UMCTL2_DERATEEN_MODE32_VAL		(UMCTL2_DERATEEN_NORMAL_VAL | UMCTL2_DERATEEN_MODE_32)
#define UMCTL2_DERATEEN_ACTIVE_VAL		(UMCTL2_DERATEEN_MR4_RD | UMCTL2_DERATEEN_MR_RD | \
						 UMCTL2_DERATEEN_BYTE | UMCTL2_DERATEEN_VAL_2)
#define UMCTL2_DERATEEN_FINAL_VAL		(UMCTL2_DERATEEN_MR4_RD | BIT(14) | UMCTL2_DERATEEN_MR_RD | \
						 UMCTL2_DERATEEN_BYTE | UMCTL2_DERATEEN_VAL_2)

/*
 * Address Mapping Constants for Allwinner A733 Dual-Channel LPDDR5 (6GB)
 *
 * Mappings configure Host Interface (HIF) address bus bit routing to SDRAM
 * Rank (CS), Bank, Column, and Row bits for optimal dual-channel interleaving:
 * - ADDRMAP0: CS0 bit positioned at HIF[29] (base 6 + 0x17), CS1 disabled (0x02),
 *             CS2 disabled (0x2f), CS3 mapped to HIF[9] (base 6 + 0x03).
 * - ADDRMAP1: Bank bits B0..B2 mapped to HIF[5..7] (base 2 + 0x03, base 3 + 0x03,
 *             base 4 + 0x07) for 8-bank geometry.
 * - ADDRMAP2: Column bits Col2..Col5 with Col2=HIF[25] (base 2 + 0x17), Col3 disabled
 *             (0x2f), Col4=HIF[28] (base 4 + 0x18), Col5=HIF[5] (base 5 + 0).
 * - ADDRMAP4: Higher column bits Col10..Col13 (0x11, 0x04, 0x08, 0x10).
 * - ADDRMAP5: Row bits Row0..Row1 at HIF[6..7] and Row2..Row10 block mapping (0x0f).
 */
#define ADDRMAP_BYTE0(v)			((v) & 0x1f)
#define ADDRMAP_BYTE1(v)			(((v) & 0x1f) << 8)
#define ADDRMAP_BYTE2(v)			(((v) & 0x3f) << 16)
#define ADDRMAP_BYTE3(v)			(((v) & 0x1f) << 24)

#define ADDRMAP0_LPDDR5_VAL			(ADDRMAP_BYTE3(0x03) | ADDRMAP_BYTE2(0x2f) | \
						 ADDRMAP_BYTE1(0x02) | ADDRMAP_BYTE0(0x17))
#define ADDRMAP1_LPDDR5_VAL			(ADDRMAP_BYTE2(0x07) | ADDRMAP_BYTE1(0x03) | \
						 ADDRMAP_BYTE0(0x03))
#define ADDRMAP2_LPDDR5_VAL			(ADDRMAP_BYTE2(0x18) | ADDRMAP_BYTE1(0x2f) | \
						 ADDRMAP_BYTE0(0x17))
#define ADDRMAP4_LPDDR5_VAL			(ADDRMAP_BYTE3(0x10) | ADDRMAP_BYTE2(0x08) | \
						 ADDRMAP_BYTE1(0x04) | ADDRMAP_BYTE0(0x11))
#define ADDRMAP5_LPDDR5_VAL			(ADDRMAP_BYTE3(0x04) | ADDRMAP_BYTE2(0x0c) | \
						 ADDRMAP_BYTE1(0x00) | ((0x0f) & 0x0f))

/**
 * struct dram_trained_reg - Trained DDR PHY register delta entry
 * @addr: Physical MMIO address of PHY register
 * @val: Trained 16-bit register value
 */
struct dram_trained_reg {
	u32 addr;
	u16 val;
} __packed;

/**
 * struct dram_cache_header - Non-volatile DRAM training cache header
 * @magic: Cache identification magic (DRAM_CACHE_MAGIC)
 * @version: Cache format version (DRAM_CACHE_VERSION)
 * @flags: Cache state flags (bit 0: trained, bit 1: valid)
 * @dram_clk: DRAM clock frequency in MHz (e.g. 1800)
 * @dram_type: DRAM memory type (e.g. 9 = LPDDR5)
 * @reg_count: Number of trained register delta entries
 * @data_size: Payload byte length (reg_count * sizeof(struct dram_trained_reg))
 * @crc32: Standard CRC32 checksum over the payload
 * @pmu_status: PMU completion status recorded at calibration (0x00000007)
 * @boot_count: Total boot cycles using this cache entry
 * @ranks: Probed number of ranks (1 or 2)
 * @rows: Probed number of row address bits (15 or 16)
 * @density_3_4: Probed 3/4 density indicator (1 for 6GB/12GB, 0 for 4GB/8GB/16GB)
 * @reserved: Reserved padding for 32-bit alignment
 *
 * Stored in Tier 1 retention SRAM and Tier 2 SPI NOR flash sector (0x003f0000).
 */
struct dram_cache_header {
	u32 magic;          /* DRAM_CACHE_MAGIC */
	u16 version;        /* DRAM_CACHE_VERSION */
	u16 flags;          /* Flags: 1 = trained, 2 = valid */
	u32 dram_clk;       /* Clock frequency (e.g. 1800) */
	u32 dram_type;      /* DRAM type (9 = LPDDR5) */
	u32 reg_count;      /* Number of trained register entries */
	u32 data_size;      /* Size of payload following header (reg_count * 6) */
	u32 crc32;          /* CRC32 of payload */
	u32 pmu_status;     /* 0x00000007 */
	u32 boot_count;     /* Incremented each boot */
	u8  ranks;          /* Probed number of ranks (1 or 2) */
	u8  rows;           /* Probed number of row address bits (15 or 16) */
	u8  density_3_4;    /* Probed 3/4 density indicator (1 for 6GB/12GB) */
	u8  reserved;       /* Reserved padding for 32-bit alignment */
};

/**
 * mctl_write_regs() - Batch write register configuration array
 * @regs: Pointer to array of register address/size/value tuples
 * @count: Number of entries in the register array
 *
 * Iterates through the register table, executing 8-bit, 16-bit, or 32-bit MMIO
 * writes to hardware peripherals. Handles special polling requirements for
 * PLL_DDR clock lock, uMCTL2 DQS calibration triggers, SWCTL handshakes,
 * PWRCTL low-power transitions, MRCTRL0 mode register writes, and PRCM DRAM
 * isolation deassertion delays.
 */
static void mctl_write_regs(const struct sunxi_dram_reg *regs, u32 count)
{
	for (u32 i = 0; i < count; i++) {
		uintptr_t addr = regs[i].addr;

		/* Only write actual MMIO addresses */
		if (addr < 0x02000000 || addr >= 0x40000000)
			continue;

		if (regs[i].size == 4)
			writel(regs[i].val, IOMEM(addr));
		else if (regs[i].size == 2)
			writew(regs[i].val, IOMEM(addr));
		else if (regs[i].size == 1)
			writeb(regs[i].val, IOMEM(addr));

		/*
		 * If writing PLL_DDR setting the enable bit without the lock bypass,
		 * poll CCU_PLL_DDR_CTRL_REG bit 28 (LOCK) until stable (Ref Manual Sec 4.1.6.1).
		 */
		if (addr == CCU_PLL_DDR_CTRL_REG && (regs[i].val & 0xe0000000) == 0xe0000000 && !(regs[i].val & 0x08000000)) {
			int to = 100000;
			udelay(20);
			while (!(readl(IOMEM(CCU_PLL_DDR_CTRL_REG)) & CCU_PLL_DDR_LOCK_BIT) && --to) {
				udelay(1);
			}
			if (!to)
				printf("WARNING: PLL_DDR lock timeout!\n");
		}

		/* DQS calibration poll: when DQSCFG0 (0xb84) is triggered, poll DQSSTAT0 (0xb88) for lock */
		if ((addr == (SUNXI_UMCTL2_CH0_BASE + UMCTL2_REG_DQSCFG0) ||
		     addr == (SUNXI_UMCTL2_CH1_BASE + UMCTL2_REG_DQSCFG0)) && regs[i].val == 1) {
			void __iomem *stat = IOMEM(addr + 4);
			int to = 100000;
			while ((((readl(stat) >> 25) & 3) != 3 || ((readl(stat) >> 28) & 3) != 3) && --to)
				udelay(1);
			if (!to)
				printf("WARNING: DQS cal poll timeout at 0x%08lx: 0x%08x\n", (ulong)addr + 4, readl(stat));
		}

		/* If setting sw_done in SWCTL (0xc80), wait for sw_done_ack in SWSTAT (0xc84) */
		if ((addr == (SUNXI_UMCTL2_CH0_BASE + UMCTL2_REG_SWCTL) ||
		     addr == (SUNXI_UMCTL2_CH1_BASE + UMCTL2_REG_SWCTL)) && regs[i].val == 1) {
			int to = 10000;
			while (!(readl(IOMEM(addr + 4)) & 1) && --to)
				udelay(1);
			if (!to)
				printf("WARNING: SWSTAT timeout at 0x%08lx: 0x%08x\n", (ulong)addr + 4, readl(IOMEM(addr + 4)));
		}

		/* If clearing PWRCTL (0x180 = 0), wait for normal mode (STAT [2:0] == 1) */
		if ((addr == (SUNXI_UMCTL2_CH0_BASE + UMCTL2_REG_PWRCTL) ||
		     addr == (SUNXI_UMCTL2_CH1_BASE + UMCTL2_REG_PWRCTL)) && regs[i].val == 0) {
			void __iomem *stat = IOMEM(addr - UMCTL2_REG_PWRCTL + UMCTL2_REG_STAT);
			int to = 100000;
			while ((readl(stat) & 0x30) != 0 && --to)
				udelay(1);
			to = 100000;
			while ((readl(stat) & 0x03) != 1 && --to)
				udelay(1);
			if (!to)
				printf("WARNING: STAT normal mode timeout at 0x%08lx: 0x%08x\n", (ulong)(uintptr_t)stat, readl(stat));
		}

		/* If triggering MR write/read in MRCTRL0 (0x80), wait for mr_wr (bit 31) to clear */
		if ((addr == (SUNXI_UMCTL2_CH0_BASE + UMCTL2_REG_MRCTRL0) ||
		     addr == (SUNXI_UMCTL2_CH1_BASE + UMCTL2_REG_MRCTRL0)) && (regs[i].val & 0x80000000)) {
			int to = 10000;
			while ((readl(IOMEM(addr)) & 0x80000000) && --to)
				udelay(1);
			if (!to)
				printf("WARNING: MRCTRL0 timeout at 0x%08lx: 0x%08x\n", (ulong)addr, readl(IOMEM(addr)));
		}

		/* PRCM Memory Isolation Deassert delay: wait 2ms after writing VCC_DRAM_ISO_REG */
		if (addr == PRCM_VCC_DRAM_ISO_REG) {
			mdelay(2);
		}

		/* uMCTL2 operating mode 2 transition wait (PWRSTAT == 2) */
		if ((addr == (SUNXI_UMCTL2_CH0_BASE + UMCTL2_REG_PWRSTAT) ||
		     addr == (SUNXI_UMCTL2_CH1_BASE + UMCTL2_REG_PWRSTAT)) && regs[i].val == 2) {
			int to = 50000;
			while (((readl(IOMEM(addr - 0x170)) & 7) != 2) && --to)
				udelay(1);
		}
	}
}



/**
 * mctl_write_mr() - Program LPDDR5 Mode Register via uMCTL2
 * @ch: uMCTL2 memory channel index (0 or 1)
 * @rank: Target DRAM rank index (0 or 1)
 * @mr_addr: Mode register index (0..255)
 * @mr_val: 16-bit value to program into mode register
 *
 * Dispatches an MR write command through the uMCTL2 MRCTRL0/1 registers
 * and polls until the controller clears the busy flag.
 */
static void mctl_write_mr(u32 ch, u32 rank, u32 mr_addr, u32 mr_val)
{
	u32 base = (ch == 0) ? SUNXI_UMCTL2_CH0_BASE : SUNXI_UMCTL2_CH1_BASE;
	u32 rank_bit = (rank == 0) ? UMCTL2_MRCTRL0_RANK0 : UMCTL2_MRCTRL0_RANK1;
	u32 ctrl = rank_bit | (mr_addr & 0xf);
	int to;

	writel(ctrl, IOMEM(base + UMCTL2_REG_MRCTRL0));
	writel(mr_val, IOMEM(base + UMCTL2_REG_MRCTRL1));
	writel(UMCTL2_MRCTRL0_TRIGGER | ctrl, IOMEM(base + UMCTL2_REG_MRCTRL0));

	to = 10000;
	while ((readl(IOMEM(base + UMCTL2_REG_MRCTRL0)) & UMCTL2_MRCTRL0_TRIGGER) && --to)
		udelay(1);
	if (!to)
		printf("WARNING: MRCTRL0 timeout ch%u rank%u MR%u\n", ch, rank, mr_addr);
}

/**
 * mctl_set_timing() - Configure JEDEC DRAM timing registers for uMCTL2 channel
 * @ch_core: Base MMIO address of target uMCTL2 channel
 * @dram_clk: DRAM clock frequency in MHz (e.g. 1800)
 *
 * Calculates and programs JEDEC LPDDR5 timing parameters (DRAMTMG0..9,
 * RFSHTMG0..1, ODTCFG, ZQCTL0) into the memory controller based on the
 * controller clock period in 1:2 frequency mode.
 */
static void mctl_set_timing(uintptr_t ch_core, u32 dram_clk)
{
	u32 tck_ps = 2000000 / dram_clk; /* controller clock period in 1:2 mode */

	/* DRAMTMG0 */
	u32 t_ras_min = (dram_clk == 1800) ? 19 : ((21000 + tck_ps - 1) / tck_ps);
	u32 t_ras_max = 15;
	u32 t_faw = (dram_clk == 1800) ? 9 : ((10000 + tck_ps - 1) / tck_ps);
	u32 wr2pre = 31;
	writel((wr2pre << 24) | (t_faw << 16) | (t_ras_max << 8) | t_ras_min,
	       IOMEM(ch_core + UMCTL2_REG_DRAMTMG0));

	/* DRAMTMG1 */
	u32 t_rp = (dram_clk == 1800) ? 10 : ((10500 + tck_ps - 1) / tck_ps);
	u32 t_rc = t_ras_min + t_rp;
	u32 rd2pre = 4;
	u32 t_xp = 4;
	writel((t_xp << 16) | (rd2pre << 8) | t_rc,
	       IOMEM(ch_core + UMCTL2_REG_DRAMTMG1));

	/* DRAMTMG2 */
	writel(0x070d1011, IOMEM(ch_core + UMCTL2_REG_DRAMTMG2));

	/* DRAMTMG3 */
	writel(0x000a1b24, IOMEM(ch_core + UMCTL2_REG_DRAMTMG3));

	/* DRAMTMG4 */
	u32 t_rcd = 3;
	u32 t_ccd = 4;
	u32 t_rrd = (dram_clk == 1800) ? 9 : ((10000 + tck_ps - 1) / tck_ps);
	writel((t_rrd << 24) | (t_ccd << 16) | (t_rcd << 8) | t_rp,
	       IOMEM(ch_core + UMCTL2_REG_DRAMTMG4));

	/* DRAMTMG5 */
	writel(0x02030706, IOMEM(ch_core + UMCTL2_REG_DRAMTMG5));

	/* DRAMTMG6 */
	writel(0x00000006, IOMEM(ch_core + UMCTL2_REG_DRAMTMG6));

	/* DRAMTMG7 */
	writel(0x00000002, IOMEM(ch_core + UMCTL2_REG_DRAMTMG7));

	/* DRAMTMG9 */
	writel(0x0002040d, IOMEM(ch_core + UMCTL2_REG_DRAMTMG9));

	/* INIT0..2 */
	writel(0x00030000, IOMEM(ch_core + UMCTL2_REG_INIT0));
	writel(0x0a100002, IOMEM(ch_core + UMCTL2_REG_INIT1));
	writel(0x001200af, IOMEM(ch_core + UMCTL2_REG_INIT2));

	/* ODTCFG */
	writel(0x00180006, IOMEM(ch_core + UMCTL2_REG_ODTCFG));

	/* RFSHTMG0..1 */
	u32 t_rfc = (dram_clk == 1800) ? 524 : ((280000 + tck_ps - 1) / tck_ps);
	u32 t_refi = (dram_clk == 1800) ? 14 : (3900000 / (tck_ps * 32 * 8));
	writel((t_refi << 16) | (1 << 12) | (t_rfc & 0xfff),
	       IOMEM(ch_core + UMCTL2_REG_RFSHTMG0));
	writel(0x00001f04, IOMEM(ch_core + UMCTL2_REG_RFSHTMG1));

	/* ZQCTL0 */
	writel(0x00556262, IOMEM(ch_core + UMCTL2_REG_ZQCTL0));

	/* DFITMG0 */
	writel(0x00000000, IOMEM(ch_core + UMCTL2_REG_DFITMG0));
}

/**
 * mctl_set_addrmap() - Configure uMCTL2 address mapping registers
 * @ch_core: Base MMIO address of target uMCTL2 channel core (0x0a100000 or 0x0a500000)
 *
 * Programs bank, column, and row bit mapping geometry into ADDRMAP0..5
 * registers for optimal address interleaving across channels and ranks.
 */
static void mctl_set_addrmap(uintptr_t ch_core)
{
	writel(ADDRMAP0_LPDDR5_VAL, IOMEM(ch_core + UMCTL2_REG_ADDRMAP0));
	writel(ADDRMAP1_LPDDR5_VAL, IOMEM(ch_core + UMCTL2_REG_ADDRMAP1));
	writel(ADDRMAP2_LPDDR5_VAL, IOMEM(ch_core + UMCTL2_REG_ADDRMAP2));
	writel(ADDRMAP4_LPDDR5_VAL, IOMEM(ch_core + UMCTL2_REG_ADDRMAP4));
	writel(ADDRMAP5_LPDDR5_VAL, IOMEM(ch_core + UMCTL2_REG_ADDRMAP5));
}

/**
 * mctl_channel_swctl_commit() - Commit quasi-dynamic uMCTL2 register configuration
 * @ch: Channel index (0 for Channel 0, 1 for Channel 1)
 *
 * Toggles the software control done bit (SWCTL.sw_done) to latch quasi-dynamic
 * timing and derating register updates into active controller registers.
 */
static void mctl_channel_swctl_commit(int ch)
{
	uintptr_t base = (ch == 0) ? SUNXI_UMCTL2_CH0_BASE : SUNXI_UMCTL2_CH1_BASE;

	/* Clear DQS calibration trigger */
	writel(0, IOMEM(base + UMCTL2_REG_DQSCFG0));

	/*
	 * Quasi-dynamic register programming sequence via SWCTL:
	 * 1. Clear sw_done (SWCTL = 0)
	 * 2. Program intermediate derating configuration
	 * 3. Set sw_done (SWCTL = UMCTL2_SWCTL_SW_DONE) to latch into active logic
	 * 4. Clear sw_done and program final derating configuration
	 * 5. Set sw_done to complete latching
	 */
	writel(0, IOMEM(base + UMCTL2_REG_SWCTL));
	writel(UMCTL2_DERATEEN_STAGE1_VAL, IOMEM(base + UMCTL2_REG_DERATEEN));
	writel(UMCTL2_SWCTL_SW_DONE, IOMEM(base + UMCTL2_REG_SWCTL));

	writel(0, IOMEM(base + UMCTL2_REG_SWCTL));
	writel(UMCTL2_DERATEEN_STAGE2_VAL, IOMEM(base + UMCTL2_REG_DERATEEN));
	writel(UMCTL2_SWCTL_SW_DONE, IOMEM(base + UMCTL2_REG_SWCTL));
}

/**
 * calc_crc32() - Calculate standard CRC32 checksum
 * @crc: Initial CRC seed (typically 0)
 * @buf: Pointer to buffer of bytes to checksum
 * @len: Number of bytes to process
 *
 * Computes standard IEEE 802.3 Ethernet CRC32 with polynomial 0xEDB88320.
 *
 * Return: Final 32-bit CRC checksum.
 */
static u32 calc_crc32(u32 crc, const void *buf, size_t len)
{
	const u8 *p = buf;
	crc = ~crc;
	while (len--) {
		crc ^= *p++;
		for (int k = 0; k < 8; k++)
			crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
	}
	return ~crc;
}

/**
 * a733_quick_dram_verify() - Verify retained memory contents across both channels
 *
 * Performs a rapid read-modify-write sanity check on Channel 0 (0x40000000)
 * and Channel 1 (0xc0000000), restoring original data, to confirm DRAM is
 * fully retained and accessible.
 *
 * Return: true if memory tests pass, false on mismatch.
 */
static bool a733_quick_dram_verify(void)
{
	volatile u32 *ch0 = (volatile u32 *)0x40000000;
	volatile u32 *ch1 = (volatile u32 *)0xc0000000;

	u32 w0 = *ch0;
	u32 w1 = *ch1;

	*ch0 = 0x5a5a1234;
	*ch1 = 0xa5a54321;
	dsb(); isb();

	bool ok = (*ch0 == 0x5a5a1234) && (*ch1 == 0xa5a54321);

	*ch0 = w0;
	*ch1 = w1;
	dsb(); isb();

	return ok;
}

/**
 * dram_check_tier1_retention() - Validate Tier 1 DRAM cache in retention SRAM
 * @dram_clk: Expected DRAM clock frequency in MHz
 * @dram_type: Expected DRAM type identifier
 *
 * Inspects the cache header at DRAM_RETENTION_SRAM, checking magic, version,
 * frequency, type, PMU status, and CRC32 payload integrity.
 *
 * Return: true if valid Tier 1 cache found, false otherwise.
 */
static bool dram_check_tier1_retention(u32 dram_clk, u32 dram_type)
{
	struct dram_cache_header *hdr = (struct dram_cache_header *)DRAM_RETENTION_SRAM;

	/* Check if Retention SRAM holds valid cached register dump */
	if (hdr->magic == DRAM_CACHE_MAGIC &&
	    hdr->version == DRAM_CACHE_VERSION &&
	    hdr->dram_clk == dram_clk &&
	    hdr->dram_type == dram_type &&
	    hdr->pmu_status == PMU_STATUS_SUCCESS &&
	    hdr->reg_count > 0 &&
	    hdr->reg_count <= DRAM_CACHE_MAX_REGS &&
	    hdr->data_size == hdr->reg_count * sizeof(struct dram_trained_reg)) {
		u8 *payload = (u8 *)(DRAM_RETENTION_SRAM + sizeof(*hdr));
		u32 crc = calc_crc32(0, payload, hdr->data_size);
		if (crc == hdr->crc32) {
			printf("DRAM [Tier 1]: Valid Retention SRAM cache (boot_count=%u, %u regs, CRC=0x%08x)!\n",
			       hdr->boot_count, hdr->reg_count, crc);
			return true;
		}
	}

	return false;
}

/**
 * dram_load_tier2_spinor() - Load Tier 2 DRAM training cache from SPI NOR flash
 * @dram_clk: Expected DRAM clock frequency in MHz
 * @dram_type: Expected DRAM type identifier
 *
 * Reads 4KB from SPI NOR sector 0x003f0000 via SPI0 controller, validates
 * magic, version, frequency, type, and payload CRC32.
 *
 * Return: true if valid Tier 2 cache found and loaded, false otherwise.
 */
static bool dram_load_tier2_spinor(u32 dram_clk, u32 dram_type)
{
	struct dram_cache_header *hdr = (struct dram_cache_header *)DRAM_RETENTION_SRAM;
	u8 *buf = (u8 *)DRAM_RETENTION_SRAM;

	spi0_init();
	spi0_read_data(buf, DRAM_CACHE_SPI_SECTOR, 4096);
	spi0_deinit();

	if (hdr->magic != DRAM_CACHE_MAGIC ||
	    hdr->version != DRAM_CACHE_VERSION ||
	    hdr->dram_clk != dram_clk ||
	    hdr->dram_type != dram_type ||
	    hdr->pmu_status != PMU_STATUS_SUCCESS ||
	    hdr->reg_count == 0 ||
	    hdr->reg_count > DRAM_CACHE_MAX_REGS ||
	    hdr->data_size != hdr->reg_count * sizeof(struct dram_trained_reg)) {
		printf("DRAM [Tier 2]: SPI NOR cache not found (magic=0x%08x, v=%u, clk=%u)\n",
		       hdr->magic, hdr->version, hdr->dram_clk);
		return false;
	}

	u8 *payload = buf + sizeof(*hdr);
	u32 crc = calc_crc32(0, payload, hdr->data_size);
	if (crc != hdr->crc32) {
		printf("DRAM [Tier 2]: SPI NOR cache CRC mismatch (exp 0x%08x, got 0x%08x)\n",
		       hdr->crc32, crc);
		return false;
	}

	printf("DRAM [Tier 2]: Valid SPI NOR training cache found (boot_count=%u, %u regs, CRC=0x%08x)!\n",
	       hdr->boot_count, hdr->reg_count, crc);
	return true;
}

/**
 * dram_apply_cached_training() - Restore cached training registers to DDR PHY
 *
 * Writes all cached register address/value pairs from DRAM_RETENTION_SRAM
 * directly into the PHY CSRs, bypassing the full PMU training sweep to boot
 * in ~6.6 ms.
 */
static void dram_apply_cached_training(void)
{
	const struct dram_cache_header *hdr = (const struct dram_cache_header *)DRAM_RETENTION_SRAM;
	const struct dram_trained_reg *regs = (const struct dram_trained_reg *)(DRAM_RETENTION_SRAM + sizeof(*hdr));

	printf("DRAM: Restoring %u cached PMU training registers...\n", hdr->reg_count);

	/* Connect APB to host to write PHY registers */
	writew(0x0, IOMEM(SUNXI_DRAM_PHY_CTRL_BASE + PMU_REG_APB_MUX));

	/* Restore all trained registers */
	for (u32 i = 0; i < hdr->reg_count; i++)
		writew(regs[i].val, IOMEM(regs[i].addr));

	/* Ensure PHY clock gating and reset state match PMU post-training */
	writew(0x1, IOMEM(SUNXI_DRAM_PHY_CTRL_BASE + PMU_REG_HANDOFF_1));
	writew(0x1, IOMEM(SUNXI_DRAM_PHY_CTRL_BASE + PMU_REG_RESET));

	/* Keep APB connected to host for Stage 3 */
	writew(0x0, IOMEM(SUNXI_DRAM_PHY_CTRL_BASE + PMU_REG_APB_MUX));
}

/**
 * struct dram_phy_range - Memory window of trained DDR PHY CSRs
 * @base: Starting physical address of PHY register block
 * @size: Byte span of register block
 */
struct dram_phy_range {
	u32 base;
	u32 size;
};

static const struct dram_phy_range dram_trained_phy_ranges[] = {
	/* DBYTE slices 0..3 */
	{ SUNXI_DRAM_PHY_DBYTE_BASE + 0 * SUNXI_DRAM_PHY_DBYTE_STRIDE, 0x1200 },
	{ SUNXI_DRAM_PHY_DBYTE_BASE + 1 * SUNXI_DRAM_PHY_DBYTE_STRIDE, 0x1200 },
	{ SUNXI_DRAM_PHY_DBYTE_BASE + 2 * SUNXI_DRAM_PHY_DBYTE_STRIDE, 0x1200 },
	{ SUNXI_DRAM_PHY_DBYTE_BASE + 3 * SUNXI_DRAM_PHY_DBYTE_STRIDE, 0x1200 },
	/* AC slices 0..1 */
	{ 0x0a960000, 0x0800 },
	{ 0x0a962000, 0x0800 },
	/* Common PHY blocks */
	{ SUNXI_DRAM_PHY_COMMON_BASE, 0x0400 },
	{ SUNXI_DRAM_PHY_AC_BASE, 0x0400 },
	{ 0x0aa20000, 0x0400 },
};

/**
 * dram_collect_trained_regs() - Collect trained PHY register deltas
 *
 * Compares post-PMU PHY registers against pre-training baseline snapshots
 * across DBYTE slices, AC slices, and common blocks, recording modified
 * registers into the retention cache array.
 */
static void dram_collect_trained_regs(void)
{
	struct dram_cache_header *hdr = (struct dram_cache_header *)DRAM_RETENTION_SRAM;
	struct dram_trained_reg *regs = (struct dram_trained_reg *)(DRAM_RETENTION_SRAM + sizeof(*hdr));
	const u16 *cmp = (const u16 *)0x00080000;
	u32 count = 0;

	/* Connect APB to host to read PHY registers */
	writew(0x0, IOMEM(SUNXI_DRAM_PHY_CTRL_BASE));

	for (size_t r = 0; r < ARRAY_SIZE(dram_trained_phy_ranges); r++) {
		u32 base = dram_trained_phy_ranges[r].base;
		for (u32 off = 0; off < dram_trained_phy_ranges[r].size; off += 2) {
			u16 before = *cmp++;
			u16 after = readw(IOMEM(base + off));
			if (before != after && count < DRAM_CACHE_MAX_REGS) {
				regs[count].addr = base + off;
				regs[count].val = after;
				count++;
			}
		}
	}

	hdr->reg_count = count;
	hdr->data_size = count * sizeof(struct dram_trained_reg);
	printf("DRAM: Collected %u trained registers (%u bytes)\n",
	       hdr->reg_count, hdr->data_size);
}

/**
 * dram_save_cache_both_tiers() - Persist trained register cache to Tier 1 & 2
 * @dram_clk: Operating DRAM clock frequency in MHz
 * @dram_type: Operating DRAM type identifier
 * @para: Pointer to DRAM parameter structure containing probed geometry
 *
 * Formats the cache header, computes payload CRC32, updates Tier 1 retention
 * SRAM and RTC general purpose register, and writes the cache image to
 * Tier 2 SPI NOR flash sector 0x003f0000.
 */
static void dram_save_cache_both_tiers(u32 dram_clk, u32 dram_type, const struct dram_para *para)
{
	struct dram_cache_header *hdr = (struct dram_cache_header *)DRAM_RETENTION_SRAM;
	struct dram_trained_reg *regs = (struct dram_trained_reg *)(DRAM_RETENTION_SRAM + sizeof(*hdr));

	if (hdr->reg_count == 0 || hdr->reg_count > DRAM_CACHE_MAX_REGS) {
		printf("DRAM: Cannot save cache, invalid reg_count=%u\n", hdr->reg_count);
		return;
	}

	hdr->boot_count++;
	hdr->magic = DRAM_CACHE_MAGIC;
	hdr->version = DRAM_CACHE_VERSION;
	hdr->flags = 3;
	hdr->dram_clk = dram_clk;
	hdr->dram_type = dram_type;
	hdr->ranks = para->ranks;
	hdr->rows = para->rows;
	hdr->density_3_4 = para->density_3_4 ? 1 : 0;
	hdr->reserved = 0;
	hdr->data_size = hdr->reg_count * sizeof(struct dram_trained_reg);
	hdr->crc32 = calc_crc32(0, regs, hdr->data_size);
	hdr->pmu_status = 7;

	/* Save Tier 1 RTC flag */
	writel(DRAM_CACHE_MAGIC, IOMEM(DRAM_RTC_MAGIC_REG));
	printf("DRAM [Tier 1]: Saved training state to Retention SRAM (%u regs, CRC=0x%08x)\n",
	       hdr->reg_count, hdr->crc32);

	/* Save Tier 2: SPI NOR Flash */
	printf("DRAM [Tier 2]: Saving training cache to SPI NOR sector 0x%08x...\n", DRAM_CACHE_SPI_SECTOR);
	spi0_init();
	int ret = spi0_erase_sector(DRAM_CACHE_SPI_SECTOR);
	if (ret == 0) {
		ret = spi0_write_data(DRAM_CACHE_SPI_SECTOR, (void *)hdr, sizeof(*hdr) + hdr->data_size);
		if (ret == 0) {
			struct dram_cache_header vhdr;
			spi0_read_data(&vhdr, DRAM_CACHE_SPI_SECTOR, sizeof(vhdr));
			if (vhdr.magic == DRAM_CACHE_MAGIC && vhdr.crc32 == hdr->crc32)
				printf("DRAM [Tier 2]: Successfully persisted and verified training cache to SPI NOR flash!\n");
			else
				printf("DRAM [Tier 2]: SPI NOR write verification FAILED (read magic=0x%08x, CRC=0x%08x, exp 0x%08x, 0x%08x)\n",
				       vhdr.magic, vhdr.crc32, DRAM_CACHE_MAGIC, hdr->crc32);
		} else {
			printf("DRAM [Tier 2]: SPI write failed (%d)\n", ret);
		}
	} else {
		printf("DRAM [Tier 2]: SPI erase failed (%d)\n", ret);
	}
	spi0_deinit();
}

/**
 * pmu_dump_diagnostics() - Dump comprehensive hardware state on training failure
 *
 * Inspects and prints ARC hardware exception frames (ECR, ERET, stack),
 * PMU mailbox status registers, common PHY registers, per-slice status,
 * DMEM parameter blocks, assert error codes, call stack return addresses,
 * and raw stack memory.
 */
static void pmu_dump_diagnostics(void)
{
	printf("\n=================== PMU TRAINING FAILURE DIAGNOSTICS ===================\n");
	/* Ensure APB is routed to host CPU so we can read registers & memory */
	writew(0x0, IOMEM(SUNXI_DRAM_PHY_CTRL_BASE + 0x00));
	udelay(10);

	/* 0. Check for Synopsys ARC CPU Hardware Exception */
	u32 exc_magic = readl(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + 0x00));
	if (exc_magic == 0x1234dead) {
		printf("\n*** ARC PMU HARDWARE EXCEPTION DETECTED! ***\n");
		printf("  ECR  (Exception Cause): 0x%08x\n", readl(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + 0x04)));
		printf("  ERET (Faulting PC):     0x%08x\n", readl(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + 0x08)));
		printf("  r0:                     0x%08x\n", readl(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + 0x0c)));
		printf("  r1:                     0x%08x\n", readl(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + 0x10)));
		printf("  sp (Stack Pointer):     0x%08x\n", readl(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + 0x14)));
		printf("  fp (Frame Pointer):     0x%08x\n", readl(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + 0x18)));
		printf("  Trailer:                0x%08x\n\n", readl(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + 0x1c)));
	}

	/* 1. PMU Microsequencer & Handshake Registers (0x0aaa0000) */
	printf("--- PMU Controller Registers (0x0aaa0000) ---\n");
	printf("  0x00 (APB MUX):   0x%04x\n", readw(IOMEM(SUNXI_DRAM_PHY_CTRL_BASE + PMU_REG_APB_MUX)));
	printf("  0x08 (PMU Poll):  0x%04x\n", readw(IOMEM(SUNXI_DRAM_PHY_CTRL_BASE + PMU_REG_POLL_STAT)));
	printf("  0x60 (Handoff 0): 0x%04x\n", readw(IOMEM(SUNXI_DRAM_PHY_CTRL_BASE + PMU_REG_HANDOFF_0)));
	printf("  0x62 (Handoff 1): 0x%04x\n", readw(IOMEM(SUNXI_DRAM_PHY_CTRL_BASE + PMU_REG_HANDOFF_1)));
	printf("  0x64 (Status Lo): 0x%04x\n", readw(IOMEM(SUNXI_DRAM_PHY_CTRL_BASE + PMU_REG_STATUS_LO)));
	printf("  0x66 (Ack):       0x%04x\n", readw(IOMEM(SUNXI_DRAM_PHY_CTRL_BASE + PMU_REG_MAILBOX_INT)));
	printf("  0x68 (Status Hi): 0x%04x\n", readw(IOMEM(SUNXI_DRAM_PHY_CTRL_BASE + PMU_REG_STATUS_HI)));
	printf("  0x100 (Micro Rst):0x%04x\n", readw(IOMEM(SUNXI_DRAM_PHY_CTRL_BASE + 0x100)));
	printf("  0x130 (Reset 0):  0x%04x\n", readw(IOMEM(SUNXI_DRAM_PHY_CTRL_BASE + 0x130)));
	printf("  0x132 (Reset 1):  0x%04x\n", readw(IOMEM(SUNXI_DRAM_PHY_CTRL_BASE + PMU_REG_RESET)));
	printf("  0x1dc (PUB CTL):  0x%04x\n", readw(IOMEM(SUNXI_DRAM_PHY_CTRL_BASE + 0x1dc)));
	printf("  0x21e:            0x%04x\n", readw(IOMEM(SUNXI_DRAM_PHY_CTRL_BASE + 0x21e)));

	/* 2. Common PHY Status & Clock Registers */
	printf("\n--- Common PHY Status Registers ---\n");
	printf("  0x0a940018 (Slice Stat):    0x%04x\n", readw(IOMEM(SUNXI_DRAM_PHY_COMMON_BASE + 0x0018)));
	printf("  0x0a940040 (Clk Gate):      0x%04x\n", readw(IOMEM(SUNXI_DRAM_PHY_COMMON_BASE + 0x0040)));
	printf("  0x0a940042 (Clk Gate Stat): 0x%04x\n", readw(IOMEM(SUNXI_DRAM_PHY_COMMON_BASE + 0x0042)));
	printf("  0x0a9400c0 (PHY Status):    0x%04x\n", readw(IOMEM(SUNXI_DRAM_PHY_COMMON_BASE + 0x00c0)));
	printf("  0x0a940120 (Init Complete): 0x%04x\n", readw(IOMEM(SUNXI_DRAM_PHY_COMMON_BASE + 0x0120)));
	printf("  0x0a97e034 (PLL Status):    0x%04x\n", readw(IOMEM(0x0a970000UL + 0xe034)));

	/* 3. PHY DBYTE Slice Status Registers (Slices 0..3) */
	printf("\n--- PHY Slice Status Registers ---\n");
	for (int s = 0; s < 4; s++) {
		u32 base = SUNXI_DRAM_PHY_DBYTE_BASE + s * SUNXI_DRAM_PHY_DBYTE_STRIDE;
		printf("  Slice %d: Mode(0x12)=0x%04x, Cfg(0x14)=0x%04x, Stat(0x18)=0x%04x, Ctr(0x52)=0x%04x, Ctr(0x54)=0x%04x, LCDL(0x16a)=0x%04x\n",
		       s,
		       readw(IOMEM(base + 0x12)),
		       readw(IOMEM(base + 0x14)),
		       readw(IOMEM(base + 0x18)),
		       readw(IOMEM(base + 0x52)),
		       readw(IOMEM(base + 0x54)),
		       readw(IOMEM(base + 0x16a)));
	}

	/* 4. Critical PMU DMEM Parameters and Mailboxes (0x0a9b0000) */
	printf("\n--- PMU DMEM State & Mailboxes (0x0a9b0000) ---\n");
	printf("  SequenceCtrl (0x10): 0x%04x\n", readw(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + 0x10)));
	printf("  DRAM Type (0x00):    0x%04x\n", readw(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + 0x00)));
	printf("  Freq / Fsp (0x04):   0x%04x\n", readw(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + 0x04)));
	printf("  DMEM[0x5a]: 0x%02x, DMEM[0x8e]: 0x%02x, DMEM[0x1b]: 0x%02x\n",
	       readb(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + 0x5a)),
	       readb(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + 0x8e)),
	       readb(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + 0x1b)));
	printf("  DMEM[0xb66]: 0x%02x, DMEM[0xb68]: 0x%02x, DMEM[0xb6b]: 0x%02x\n",
	       readb(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + 0xb66)),
	       readb(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + 0xb68)),
	       readb(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + 0xb6b)));
	printf("  DMEM[0xb6e..0xb71]: %02x %02x %02x %02x\n",
	       readb(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + 0xb6e)),
	       readb(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + 0xb6f)),
	       readb(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + 0xb70)),
	       readb(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + 0xb71)));
	printf("  DMEM[0xb96]: 0x%02x, DMEM[0xb98]: 0x%02x, DMEM[0xb9c]: 0x%08x\n",
	       readb(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + 0xb96)),
	       readb(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + 0xb98)),
	       readl(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + 0xb9c)));
	u32 assert_code = readw(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + 0x0c)) |
		((u32)readw(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + 0x0e)) << 16);

	printf("  DMEM[0xa7c]: 0x%02x, DMEM[0x0e]: 0x%04x, DMEM[0x08]: 0x%04x, Assert Code (0x0c): 0x%08x\n",
	       readb(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + 0xa7c)),
	       readw(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + 0x0e)),
	       readw(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + 0x08)),
	       assert_code);
	printf("  DMEM[0x0b]: 0x%02x, DMEM[0x20]: 0x%02x, DMEM[0x21]: 0x%02x, DMEM[0x25]: 0x%02x, DMEM[0x40]: 0x%02x\n",
	       readb(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + 0x0b)),
	       readb(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + 0x20)),
	       readb(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + 0x21)),
	       readb(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + 0x25)),
	       readb(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + 0x40)));
	printf("  DMEM[0x62]: 0x%02x, DMEM[0xe7]: 0x%02x, DMEM[0x402]: 0x%02x, DMEM[0x403]: 0x%02x\n",
	       readb(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + 0x62)),
	       readb(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + 0xe7)),
	       readb(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + 0x402)),
	       readb(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + 0x403)));
	printf("  Active Calibration (0x406..0x408): Rank=%d, Slice=%d, Pin=%d\n",
	       readb(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + 0x406)),
	       readb(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + 0x407)),
	       readb(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + 0x408)));

	printf("  Slice 0 Pin Trailers & Mid-Eye Samples:\n");
	for (u32 p = 0; p < 10; p++) {
		u32 base = PMU_PIN_SAMPLE_BASE + p * PMU_PIN_SAMPLE_STRIDE;
		u16 base_tap = readw(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + base + 128));
		u8 center = readb(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + base + 130));
		u8 target = readb(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + base + 131));
		u16 s30 = readw(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + base + 60));
		u16 s32 = readw(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + base + 64));

		printf("    Pin %d [0x%04x]: base=0x%04x, ctr=%d, tgt=%d, s[30]=0x%04x, s[32]=0x%04x\n",
		       p, base, base_tap, center, target, s30, s32);
	}

	/* 5. PMU ARC Call Stack Trace Candidates (Return addresses pointing into IMEM) */
	printf("\n--- PMU ARC Call Stack Trace Candidates (IMEM Return Addresses) ---\n");
	for (u32 off = 0xfe00; off < 0x10000; off += 2) {
		u32 word = readw(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + off)) |
			((u32)readw(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + off + 2)) << 16);
		if (word >= 0x00000010 && word < 0x00010000 && (word & 1) == 0) {
			printf("  [DMEM 0x%04x / ARC SP+0x%03x]: Return Address -> 0x%04x\n",
			       off, off - 0xfe00, word);
		}
	}

	/* 6. PMU ARC Raw Stack Dump (Top of DMEM: 0x0a9c0000 down 512 bytes) */
	printf("\n--- PMU ARC Stack Dump (0x0a9bfe00 - 0x0a9c0000) ---\n");
	for (u32 off = 0xfe00; off < 0x10000; off += 16) {
		printf("  [0x%04x / 0x%08x]: %04x %04x %04x %04x %04x %04x %04x %04x\n",
		       off, 0x80000000U + off,
		       readw(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + off + 0)),
		       readw(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + off + 2)),
		       readw(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + off + 4)),
		       readw(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + off + 6)),
		       readw(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + off + 8)),
		       readw(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + off + 10)),
		       readw(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + off + 12)),
		       readw(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + off + 14)));
	}

	/* 7. PMU Calibration Structure Area (0x0a9b09a0 - 0x0a9b0a60) */
	printf("\n--- PMU Calibration Structure Area (0x0a9b09a0 - 0x0a9b0a60) ---\n");
	for (u32 off = 0x09a0; off < 0x0a60; off += 16) {
		printf("  [0x%04x / 0x%08x]: %04x %04x %04x %04x %04x %04x %04x %04x\n",
		       off, 0x80000000U + off,
		       readw(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + off + 0)),
		       readw(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + off + 2)),
		       readw(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + off + 4)),
		       readw(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + off + 6)),
		       readw(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + off + 8)),
		       readw(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + off + 10)),
		       readw(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + off + 12)),
		       readw(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + off + 14)));
	}
	printf("========================================================================\n\n");
}

/**
 * mctl_phy_init() - Initialize DDR PHY and perform training or cached restore
 * @dram_clk: Target DRAM clock frequency in MHz
 * @dram_type: Target DRAM memory type (e.g. 9 = LPDDR5)
 * @force_training: Force full PMU sweep regardless of cached state
 * @trained_out: Output pointer set to true if fresh training occurred, false if cached
 *
 * Executes the full PHY bringing-up sequence: CCU/MBUS clocks, uMCTL2 channel
 * initialization, pre-IMEM/DMEM PHY tables, firmware loading, PMU execution,
 * register restoration or collection, Stage 3 post-streaming, power isolation
 * release, and uMCTL2 operating mode verification.
 *
 * Return: 0 on success, or -1 on fatal training timeout/failure.
 */
static int mctl_phy_init(u32 dram_clk, u32 dram_type, bool force_training, bool *trained_out)
{
	u32 val = 0;
	bool use_cache = false;

	u32 stat0 = readl(IOMEM(SUNXI_UMCTL2_CH0_BASE + UMCTL2_REG_STAT));
	u32 stat1 = readl(IOMEM(SUNXI_UMCTL2_CH1_BASE + UMCTL2_REG_STAT));
	u32 rtc_magic = readl(IOMEM(DRAM_RTC_MAGIC_REG));

	/* Ultra-fast warm reboot bypass: uMCTL2 is already initialized and in NORMAL MODE */
	if (IS_ENABLED(CONFIG_SUNXI_PMU_TRAINING_CACHE) &&
	    (stat0 & 3) == UMCTL2_STAT_NORMAL_MODE &&
	    (stat1 & 3) == UMCTL2_STAT_NORMAL_MODE &&
	    rtc_magic == DRAM_CACHE_MAGIC) {
		printf("DRAM [Tier 1]: uMCTL2 already in NORMAL MODE (STAT ch0=0x%x, ch1=0x%x)!\n", stat0, stat1);
		if (a733_quick_dram_verify()) {
			printf("DRAM [Tier 1]: Warm boot memory check PASS! Bypassing all training (<1ms)!\n");
			*trained_out = false;
			return 0;
		}
	}

	if (IS_ENABLED(CONFIG_SUNXI_PMU_TRAINING_CACHE) && !force_training) {
		if (dram_check_tier1_retention(dram_clk, dram_type)) {
			use_cache = true;
		} else if (dram_load_tier2_spinor(dram_clk, dram_type)) {
			use_cache = true;
		}
	}

	/* 1. Pre-IMEM PHY setup & DMC / uMCTL2 channel configuration (Boot0 writes 0..253) */
	printf("Applying pre-IMEM CCU & MBUS registers (%u writes)...\n",
	       (unsigned int)ARRAY_SIZE(dram_stage2_pre_imem_ccu_mbus));
	mctl_write_regs(dram_stage2_pre_imem_ccu_mbus, ARRAY_SIZE(dram_stage2_pre_imem_ccu_mbus));

	printf("Applying dual-channel uMCTL2 configuration (2x %u static + math timing)...\n",
	       (unsigned int)ARRAY_SIZE(dram_stage2_pre_imem_channel));
	for (int ch = 0; ch < 2; ch++) {
		u32 ch_offset = ch * SUNXI_UMCTL2_CH_STRIDE;
		u32 ch_core = SUNXI_UMCTL2_CORE_CH0_BASE + ch_offset;

		/* AXI Ports 0..3 priority/gate enable */
		for (int p = 0; p < 4; p++)
			writel(UMCTL2_PORT_PRIO_GATE_EN,
			       IOMEM(ch_core + p * UMCTL2_PORT_STRIDE + UMCTL2_REG_PORT_PCTRL));

		/* Apply static channel registers (53 writes: config, QoS, queues, derating) */
		for (size_t i = 0; i < ARRAY_SIZE(dram_stage2_pre_imem_channel); i++) {
			u32 addr = dram_stage2_pre_imem_channel[i].addr + ch_offset;
			writel(dram_stage2_pre_imem_channel[i].val, IOMEM(addr));
		}

		/* Program dynamically calculated JEDEC LPDDR5 timings for dram_clk */
		mctl_set_timing(ch_core, dram_clk);

		/* Program address mapping registers from geometry */
		mctl_set_addrmap(ch_core);

		/* Commit quasi-dynamic registers via SWCTL */
		mctl_channel_swctl_commit(ch);
	}

	printf("Applying pre-IMEM PHY setup (%u writes)...\n",
	       (unsigned int)ARRAY_SIZE(dram_stage2_pre_imem_phy));
	mctl_write_regs(dram_stage2_pre_imem_phy, ARRAY_SIZE(dram_stage2_pre_imem_phy));

	printf("Applying pre-DMEM common PHY registers (%u writes)...\n",
	       (unsigned int)ARRAY_SIZE(dram_stage2_pre_dmem_common));
	mctl_write_regs(dram_stage2_pre_dmem_common, ARRAY_SIZE(dram_stage2_pre_dmem_common));

	printf("Applying pre-DMEM AC lanes (2x %u writes)...\n",
	       (unsigned int)ARRAY_SIZE(dram_stage2_pre_dmem_ac));
	for (int ac = 0; ac < 2; ac++) {
		u32 ac_offset = ac * SUNXI_DRAM_PHY_AC_SLICE_STRIDE;
		for (size_t i = 0; i < ARRAY_SIZE(dram_stage2_pre_dmem_ac); i++) {
			u32 addr = dram_stage2_pre_dmem_ac[i].addr + ac_offset;
			writew(dram_stage2_pre_dmem_ac[i].val, IOMEM(addr));
		}
	}

	printf("Applying pre-DMEM DBYTE slices (4x %u static + loops)...\n",
	       (unsigned int)ARRAY_SIZE(dram_stage2_pre_dmem_dbyte));
	for (int slice = 0; slice < 4; slice++) {
		u32 slice_base = SUNXI_DRAM_PHY_DBYTE_BASE + slice * SUNXI_DRAM_PHY_DBYTE_STRIDE;

		/* Contiguous general config block: 8 registers from 0xbc to 0xca = 0x0366 */
		for (int r = 0; r < 8; r++)
			writew(0x0366, IOMEM(slice_base + 0xbc + r * 2));

		/* 9 bit-lanes x 4 registers: receiver impedance/slew = 0x0035 */
		for (int lane = 0; lane < 9; lane++) {
			for (int r = 0; r < 4; r++)
				writew(0x0035, IOMEM(slice_base + lane * 0x200 + 0x9c + r * 2));
		}

		for (size_t i = 0; i < ARRAY_SIZE(dram_stage2_pre_dmem_dbyte); i++) {
			u32 addr = dram_stage2_pre_dmem_dbyte[i].addr + slice * 0x2000;
			writew(dram_stage2_pre_dmem_dbyte[i].val, IOMEM(addr));
		}
	}

	printf("Applying pre-DMEM end registers (%u writes)...\n",
	       (unsigned int)ARRAY_SIZE(dram_stage2_pre_dmem_end));
	mctl_write_regs(dram_stage2_pre_dmem_end, ARRAY_SIZE(dram_stage2_pre_dmem_end));

	/* Post-DMEM PHY setup (bit mapping registers) */
	printf("Applying post-DMEM PHY registers (%u writes)...\n", (unsigned int)ARRAY_SIZE(dram_stage2_post_dmem_regs));
	mctl_write_regs(dram_stage2_post_dmem_regs, ARRAY_SIZE(dram_stage2_post_dmem_regs));
	mctl_write_regs(dram_stage2_post_dmem_regs, ARRAY_SIZE(dram_stage2_post_dmem_regs));


	/* 2. Connect APB to host to load training firmware into PHY IMEM */
	writew(0x0, IOMEM(SUNXI_DRAM_PHY_CTRL_BASE + PMU_REG_APB_MUX));
	printf("Loading PMU firmware (%u halfwords = %u bytes)...\n",
	       (unsigned int)ARRAY_SIZE(sun60i_a733_lpddr5_fw), (unsigned int)sizeof(sun60i_a733_lpddr5_fw));
	for (size_t i = 0; i < ARRAY_SIZE(sun60i_a733_lpddr5_fw); i++)
		writew(sun60i_a733_lpddr5_fw[i], IOMEM(SUNXI_DRAM_PHY_IMEM_BASE + i * 2));
	for (size_t i = ARRAY_SIZE(sun60i_a733_lpddr5_fw); i < (0x10000 / 2); i++)
		writew(0, IOMEM(SUNXI_DRAM_PHY_IMEM_BASE + i * 2));

	/* 3. Connect APB to host to load training parameters into PHY DMEM */
	writew(0x0, IOMEM(SUNXI_DRAM_PHY_CTRL_BASE + PMU_REG_APB_MUX));

	u16 seq_ctrl = use_cache ? PMU_SEQ_FAST_BOOT : PMU_SEQ_FULL_TRAIN;
	size_t cur_word = 0;

	/* Stream parameter blocks in a single strictly-monotonic forward pass */
	for (size_t b = 0; b < ARRAY_SIZE(dram_dmem_blocks); b++) {
		const struct pmu_dmem_block *block = &dram_dmem_blocks[b];
		size_t target_word = block->offset / 2;

		while (cur_word < target_word) {
			writew(0, IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + cur_word * 2));
			cur_word++;
		}

		for (size_t i = 0; i < block->count; i++) {
			u16 val = block->data[i];
			if (b == 0 && i == 6 && IS_ENABLED(CONFIG_SUNXI_PMU_FAST_WRITE_DQS))
				val |= 0x2000; /* Bit 5 of DMEM[0x0d]: Fast Single-Pass Write DQS */
			if (b == 0 && i == 8)
				val = seq_ctrl;
			if (b == 0 && i == 128)
				val = (val & 0xff00) | (CONFIG_SUNXI_PMU_SCAN_STEP & 0xff);
			writew(val, IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + cur_word * 2));
			cur_word++;
		}
	}

	/* Zero the remainder of DMEM up to 64 KB boundary */
	while (cur_word < (0x10000 / 2)) {
		writew(0, IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + cur_word * 2));
		cur_word++;
	}

	u16 sc_read = readw(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + 0x10));
	printf("DRAM: PMU SequenceCtrl = 0x%04x (%s, readback = 0x%04x)\n",
	       seq_ctrl, use_cache ? "fast boot" : "full training", sc_read);

	/* Test host access to PHY loopback / scratch registers BEFORE handing over to PMU */
	writew(0x0, IOMEM(SUNXI_DRAM_PHY_CTRL_BASE + PMU_REG_APB_MUX));
	writew(0x715d, IOMEM(SUNXI_DRAM_PHY_DBYTE_BASE + 0x0052));
	writew(0x00ce, IOMEM(SUNXI_DRAM_PHY_DBYTE_BASE + 0x0054));
	writew(0x715d, IOMEM(SUNXI_DRAM_PHY_DBYTE_BASE + 0x005c));
	writew(0x00ce, IOMEM(SUNXI_DRAM_PHY_DBYTE_BASE + 0x005e));

	/* Snapshot PHY registers to identify exact PMU training outputs on fresh training */
	if (!use_cache) {
		u16 *snap = (u16 *)0x00080000;
		for (size_t r = 0; r < ARRAY_SIZE(dram_trained_phy_ranges); r++) {
			u32 base = dram_trained_phy_ranges[r].base;
			for (u32 off = 0; off < dram_trained_phy_ranges[r].size; off += 2)
				*snap++ = readw(IOMEM(base + off));
		}
	}

	/* 4. Hand APB control to PMU ARC microsequencer */
	writew(0x1, IOMEM(SUNXI_DRAM_PHY_CTRL_BASE + PMU_REG_APB_MUX));

	printf("Triggering PMU calibration...\n");
	/* 5. PMU Reset & Trigger Sequence */
	writew(0x9, IOMEM(SUNXI_DRAM_PHY_CTRL_BASE + PMU_REG_RESET));
	writew(0x1, IOMEM(SUNXI_DRAM_PHY_CTRL_BASE + PMU_REG_RESET));
	writew(0x0, IOMEM(SUNXI_DRAM_PHY_CTRL_BASE + PMU_REG_RESET));
	writew(0x1, IOMEM(SUNXI_DRAM_PHY_CTRL_BASE + PMU_REG_HANDOFF_1));
	writew(0x1, IOMEM(SUNXI_DRAM_PHY_CTRL_BASE + PMU_REG_MAILBOX_INT));

	/* Wait while bit 0 of PMU_REG_POLL_STAT is 1 (training in progress) */
	int pmu_to = 15000000;
	while ((readw(IOMEM(SUNXI_DRAM_PHY_CTRL_BASE + PMU_REG_POLL_STAT)) & 1) != 0 && --pmu_to) {
		udelay(1);
		if ((pmu_to % 1000000) == 0)
			printf(".");
	}
	if (pmu_to < 15000000)
		printf("\n");
	printf("PMU finish poll: 0x08=0x%04x (pmu_to=%d, elapsed=%d us)\n",
	       readw(IOMEM(SUNXI_DRAM_PHY_CTRL_BASE + PMU_REG_POLL_STAT)), pmu_to, 15000000 - pmu_to);
	if (!pmu_to) {
		printf("WARNING: PMU training poll timeout!\n");
		pmu_dump_diagnostics();
	}

	/* Read status BEFORE clearing PMU_REG_HANDOFF_1 or asserting reset on PMU_REG_RESET */
	val = readw(IOMEM(SUNXI_DRAM_PHY_CTRL_BASE + PMU_REG_STATUS_LO)) |
	      (readw(IOMEM(SUNXI_DRAM_PHY_CTRL_BASE + PMU_REG_STATUS_HI)) << 16);
	printf("PMU Status: 0x%08x\n", val);

	/* Complete handshake with PMU */
	writew(0x0, IOMEM(SUNXI_DRAM_PHY_CTRL_BASE + PMU_REG_HANDOFF_1));
	int hs_to = 100000;
	while ((readw(IOMEM(SUNXI_DRAM_PHY_CTRL_BASE + PMU_REG_POLL_STAT)) == 0) && --hs_to) {
		udelay(1);
	}
	writew(0x1, IOMEM(SUNXI_DRAM_PHY_CTRL_BASE + PMU_REG_HANDOFF_1));
	writew(0x1, IOMEM(SUNXI_DRAM_PHY_CTRL_BASE + PMU_REG_RESET));

	if (val != PMU_STATUS_SUCCESS) {
		printf("PMU training returned status 0x%08x\n", val);
		pmu_dump_diagnostics();
		return -1;
	}
	printf("PMU initialized successfully (Status 0x7)!\n");

	/* Connect APB back to host CPU for Stage 3 PHY register configuration! */
	writew(0x0, IOMEM(SUNXI_DRAM_PHY_CTRL_BASE + PMU_REG_APB_MUX));

	if (!use_cache && IS_ENABLED(CONFIG_SUNXI_PMU_DEBUG_TELEMETRY)) {
		static const char * const stage_names[12] = {
			"Stage 0/1: DEV_INIT & LPCA",
			"Stage 6: 2D Eye Search (SEARCH_WIN)",
			"Stage 2 (post): Vref Centering (D804)",
			"Stage 3 (post): DCD Correction (D7BC)",
			"Stage 5 (post): DQS Centering (D720)",
			"Stage 8: Deskew Sweep Type 2 (Read DQS/DQ)",
			"Stage 8: Deskew Sweep Type 3 (WCK/CK)",
			"Stage 8: Post-DQS Centering (D720)",
			"Stage 8: Deskew Sweep Type 1 (Write DQS/DQ)",
			"Stage 8: Deskew Sweep Type 0 (Write DQ)",
			"Stage 10: Margin Envelope (MARGIN_STEP)",
			"Completion / Handoff"
		};
		printf("\n=== PMU Stage Telemetry Profile ===\n");
		u32 t_prev = readw(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + 0x300)) |
			     (readw(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + 0x302)) << 16);
		for (int i = 0; i < 12; i++) {
			u32 t_curr = readw(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + 0x304 + i * 4)) |
				     (readw(IOMEM(SUNXI_DRAM_PHY_DMEM_BASE + 0x306 + i * 4)) << 16);
			u32 delta = t_curr - t_prev;
			printf("  [%02d] %-42s: %10u ticks\n", i, stage_names[i], delta);
			t_prev = t_curr;
		}
		printf("===================================\n\n");
	}

	if (use_cache) {
		/* Restore pre-trained delay lines and deskew registers on top of DevInit */
		dram_apply_cached_training();
		*trained_out = false;
	} else {
		/* Collect trained PHY registers for Retention SRAM & SPI NOR caching */
		dram_collect_trained_regs();
		*trained_out = true;
	}

	/* Execute all Stage 3 configuration/mailboxes */
	printf("Applying Stage 3 PHY pre-stream registers (%u writes + shadow zero)...\n",
	       (unsigned int)ARRAY_SIZE(dram_stage3_pre_regs));

	/* DBYTE 4 slices: 0x1e = 0x0988, 0xae = 0x0000 */
	for (int slice = 0; slice < 4; slice++) {
		u32 base = SUNXI_DRAM_PHY_DBYTE_BASE + slice * SUNXI_DRAM_PHY_DBYTE_STRIDE;
		writew(0x0988, IOMEM(base + 0x001e));
		writew(0x0000, IOMEM(base + 0x00ae));
	}

	/* AC 2 slices: 0x94 = 0, 0x96 = 0 */
	for (int ac = 0; ac < 2; ac++) {
		u32 base = SUNXI_DRAM_PHY_AC_SLICE_BASE + ac * SUNXI_DRAM_PHY_AC_SLICE_STRIDE;
		writew(0x0000, IOMEM(base + 0x0094));
		writew(0x0000, IOMEM(base + 0x0096));
	}

	/* Zero-fill shadow RAM (184 halfwords: 0x0a982000..0x0a98216e) */
	for (int i = 0; i < 184; i++)
		writew(0x0000, IOMEM(SUNXI_DRAM_PHY_SHADOW_RAM_BASE + i * 2));

	/* Apply sparse non-zero shadow overrides (34 writes) */
	for (size_t i = 0; i < ARRAY_SIZE(dram_stage3_pre_sparse); i++) {
		u32 addr = SUNXI_DRAM_PHY_SHADOW_RAM_BASE + dram_stage3_pre_sparse[i].offset;
		writew(dram_stage3_pre_sparse[i].val, IOMEM(addr));
	}

	/* Static pre-stream registers (30 writes) */
	mctl_write_regs(dram_stage3_pre_regs, ARRAY_SIZE(dram_stage3_pre_regs));

	/* Stream 29: zero 2256 halfwords (0x0a982710..0x0a9838ae) and apply sparse non-zero overrides */
	for (size_t i = 0; i < 2256; i++)
		writew(0, IOMEM(SUNXI_DRAM_PHY_STREAM29_BASE + i * 2));
	for (size_t i = 0; i < ARRAY_SIZE(dram_stage3_sparse29); i++) {
		u32 addr = SUNXI_DRAM_PHY_STREAM29_BASE + dram_stage3_sparse29[i].offset;
		writew(dram_stage3_sparse29[i].val, IOMEM(addr));
	}

	/* Stream 30: block copy 1290 halfwords to 0x0aa2005c..0x0aa20a6e */
	for (size_t i = 0; i < ARRAY_SIZE(dram_stage3_stream30); i++)
		writew(dram_stage3_stream30[i], IOMEM(SUNXI_DRAM_PHY_PUB_STREAM_BASE + i * 2));

	/* DBYTE post registers (4 slices x 5 registers) */
	for (int slice = 0; slice < 4; slice++) {
		u32 base = SUNXI_DRAM_PHY_DBYTE_BASE + slice * SUNXI_DRAM_PHY_DBYTE_STRIDE;
		writew(0x0001, IOMEM(base + 0x0172));
		writew(0x0180, IOMEM(base + 0x0162));
		writew(0x0001, IOMEM(base + 0x0174));
		writew(0x0000, IOMEM(base + 0x0144));
		writew(0x0001, IOMEM(base + 0x016a));
	}

	/* Stage 3: PHY shadow grid (9 lanes x 8 regs = 0x5a3c) */
	for (int lane = 0; lane < 9; lane++) {
		for (int r = 0; r < 8; r++)
			writew(0x5a3c, IOMEM(SUNXI_DRAM_PHY_AC_BASE + 0x0048 + lane * 0x200 + r * 2));
	}
	writew(0x01ff, IOMEM(SUNXI_DRAM_PHY_AC_BASE + 0x00ca));

	printf("Applying Stage 3 PHY post-stream registers (%u writes)...\n",
	       (unsigned int)ARRAY_SIZE(dram_stage3_post_regs));
	mctl_write_regs(dram_stage3_post_regs, ARRAY_SIZE(dram_stage3_post_regs));

	/* Apply post-PMU channel timing and interconnect configuration */
	printf("Applying post-PMU channel timing and interconnect configuration...\n");
	for (int ch = 0; ch < 2; ch++) {
		u32 ch_offset = ch * SUNXI_UMCTL2_CH_STRIDE;
		for (size_t i = 0; i < ARRAY_SIZE(dram_post_pmu_channel); i++) {
			u32 addr = dram_post_pmu_channel[i].addr;
			if (addr != SUNXI_DRAM_PHY_CTRL_BASE)
				addr += ch_offset;
			if (dram_post_pmu_channel[i].size == 4)
				writel(dram_post_pmu_channel[i].val, IOMEM(addr));
			else
				writew(dram_post_pmu_channel[i].val, IOMEM(addr));
		}
		if (ch == 0) {
			writew(0x0, IOMEM(SUNXI_DRAM_PHY_CTRL_BASE + PMU_REG_APB_MUX));
			writew(0x3, IOMEM(SUNXI_DRAM_PHY_PUB_BASE + PHY_REG_PUB_MICRO_RST));
			writew(0x0, IOMEM(SUNXI_DRAM_PHY_PUB_BASE + PHY_REG_PUB_MICRO_RST));
			writew(0x1, IOMEM(SUNXI_DRAM_PHY_CTRL_BASE + PMU_REG_APB_MUX));
		}
	}

	/* Transition uMCTL2 to normal mode and unmask ports */
	for (int ch = 0; ch < 2; ch++) {
		u32 base = (ch == 0) ? SUNXI_UMCTL2_CH0_BASE : SUNXI_UMCTL2_CH1_BASE;
		writel(0x0, IOMEM(base + UMCTL2_REG_SWCTL));
		writel(UMCTL2_DERATEEN_NORMAL_VAL, IOMEM(base + UMCTL2_REG_DERATEEN));
		writel(UMCTL2_DERATEEN_MODE32_VAL, IOMEM(base + UMCTL2_REG_DERATEEN));
		writel(UMCTL2_SWCTL_SW_DONE, IOMEM(base + UMCTL2_REG_SWCTL));
	}

	/* Release DRAM power isolation via STBY_PRCM (Allwinner User Manual Sec 4.2.4) */
	writel(PRCM_VCC_DRAM_ISO_RELEASE, IOMEM(PRCM_VCC_DRAM_ISO_REG));
	mdelay(2);

	for (int ch = 0; ch < 2; ch++) {
		u32 base = (ch == 0) ? SUNXI_UMCTL2_CH0_BASE : SUNXI_UMCTL2_CH1_BASE;
		writel(0x0, IOMEM(base + UMCTL2_REG_SWCTL));
		writel(UMCTL2_DERATEEN_NORMAL_VAL, IOMEM(base + UMCTL2_REG_DERATEEN));
		writel(0x0, IOMEM(base + UMCTL2_REG_PWRCTL));
		writel(UMCTL2_DERATEEN_ACTIVE_VAL, IOMEM(base + UMCTL2_REG_DERATEEN));
		writel(UMCTL2_SWCTL_SW_DONE, IOMEM(base + UMCTL2_REG_SWCTL));

		writel(0x0, IOMEM(base + UMCTL2_REG_SWCTL));
		writel(0x0, IOMEM(base + UMCTL2_REG_RFSHCTL3));
		writel(UMCTL2_DERATEEN_FINAL_VAL, IOMEM(base + UMCTL2_REG_DERATEEN));
		writel(UMCTL2_SWCTL_SW_DONE, IOMEM(base + UMCTL2_REG_SWCTL));
	}

	mctl_write_regs(dram_post_pmu_interconnect, ARRAY_SIZE(dram_post_pmu_interconnect));

	/* Program LPDDR5 Mode Registers (MR16, MR17) across both channels and ranks */
	writew(0x0, IOMEM(SUNXI_DRAM_PHY_CTRL_BASE + PMU_REG_APB_MUX));
	{
		static const struct {
			u8 mr;
			u16 val;
		} mr_cfgs[] = {
			{ 16, 0x1005 },
			{ 17, 0x0e00 },
			{ 17, 0x0f00 },
			{ 16, 0x1004 },
		};
		for (size_t m = 0; m < ARRAY_SIZE(mr_cfgs); m++) {
			for (int ch = 0; ch < 2; ch++) {
				for (int rank = 0; rank < 2; rank++)
					mctl_write_mr(ch, rank, mr_cfgs[m].mr, mr_cfgs[m].val);
			}
		}
	}
	writew(0x1, IOMEM(SUNXI_DRAM_PHY_CTRL_BASE + PMU_REG_APB_MUX));

	printf("MAER Firewall Status: 0x0a021800=0x%08x, 0x0a021808=0x%08x\n",
	       readl(IOMEM(DRAMC_MAER_REG0)), readl(IOMEM(DRAMC_MAER_REG1)));

	/* Check uMCTL2 operating status on both channels (offset 0x14) */
	stat0 = readl(IOMEM(SUNXI_UMCTL2_CH0_BASE + UMCTL2_REG_STAT));
	stat1 = readl(IOMEM(SUNXI_UMCTL2_CH1_BASE + UMCTL2_REG_STAT));
	printf("uMCTL2 Operating Status: Ch0 STAT(0x0a110014)=0x%08x (%s), Ch1 STAT(0x0a510014)=0x%08x (%s)\n",
	       stat0, (stat0 & 3) == 1 ? "NORMAL MODE" : "NOT NORMAL",
	       stat1, (stat1 & 3) == 1 ? "NORMAL MODE" : "NOT NORMAL");
	printf("uMCTL2 Controller Status Summary:\n"
	       "  Ch0: PWRCTL=0x%08x, PCTRL_0=0x%08x, PSTAT=0x%08x, SWSTAT=0x%08x\n"
	       "  Ch1: PWRCTL=0x%08x, PCTRL_0=0x%08x, PSTAT=0x%08x, SWSTAT=0x%08x\n"
	       "  PHY APB mux (0x0aaa0000)=0x%04x, VCC_DRAM_ISO (0x07010250)=0x%08x\n",
	       readl(IOMEM(SUNXI_UMCTL2_CH0_BASE + UMCTL2_REG_PWRCTL)),
	       readl(IOMEM(SUNXI_UMCTL2_CH0_BASE + UMCTL2_REG_DERATEEN)),
	       readl(IOMEM(SUNXI_UMCTL2_CH0_BASE + UMCTL2_REG_DERATEINT)),
	       readl(IOMEM(SUNXI_UMCTL2_CH0_BASE + UMCTL2_REG_SWSTAT)),
	       readl(IOMEM(SUNXI_UMCTL2_CH1_BASE + UMCTL2_REG_PWRCTL)),
	       readl(IOMEM(SUNXI_UMCTL2_CH1_BASE + UMCTL2_REG_DERATEEN)),
	       readl(IOMEM(SUNXI_UMCTL2_CH1_BASE + UMCTL2_REG_DERATEINT)),
	       readl(IOMEM(SUNXI_UMCTL2_CH1_BASE + UMCTL2_REG_SWSTAT)),
	       readw(IOMEM(SUNXI_DRAM_PHY_CTRL_BASE + PMU_REG_APB_MUX)),
	       readl(IOMEM(PRCM_VCC_DRAM_ISO_REG)));

	printf("uMCTL2 ready!\n");
	return 0;
}

/**
 * sun60i_a733_calc_dram_size() - Calculate DRAM size from geometry parameters
 * @para: Pointer to DRAM parameter structure
 *
 * Decodes row, column, bank, bank group, rank, and channel geometry,
 * calculating total memory capacity in bytes.
 *
 * Return: Total DRAM capacity in bytes.
 */
static unsigned long sun60i_a733_calc_dram_size(const struct dram_para *para)
{
	u32 sum = para->ranks + para->bank_groups + para->rows + para->banks + para->cols;
	u32 shift = (para->channels == 1) ? (sum - 19) : (sum - 18);
	u32 size_mb = 1U << (shift & 0xff);

	if (para->density_3_4)
		size_mb = (size_mb * 3) >> 2;

	return (unsigned long)size_mb << 20;
}

/* R_TWI (PRCM I2C0) registers */
#define SUNXI_R_TWI_BASE		0x07083000
#define R_TWI_ADDR			(SUNXI_R_TWI_BASE + 0x00)
#define R_TWI_XADDR			(SUNXI_R_TWI_BASE + 0x04)
#define R_TWI_DATA			(SUNXI_R_TWI_BASE + 0x08)
#define R_TWI_CNTR			(SUNXI_R_TWI_BASE + 0x0c)
#define R_TWI_STAT			(SUNXI_R_TWI_BASE + 0x10)
#define R_TWI_CCR			(SUNXI_R_TWI_BASE + 0x14)
#define R_TWI_SRST			(SUNXI_R_TWI_BASE + 0x18)
#define R_TWI_LCR			(SUNXI_R_TWI_BASE + 0x1c)
#define R_TWI_EFR			(SUNXI_R_TWI_BASE + 0x20)

/* R_TWI Control Register (CNTR) bits */
#define R_TWI_CNTR_AACK			BIT(2)
#define R_TWI_CNTR_INTFLAG		BIT(3)
#define R_TWI_CNTR_STOP			BIT(4)
#define R_TWI_CNTR_START		BIT(5)
#define R_TWI_CNTR_BUSEN		BIT(6)

/* R_TWI Status Register (STAT) codes (Philips/NXP standard) */
#define R_TWI_STAT_START		0x08
#define R_TWI_STAT_REPEATED_START	0x10
#define R_TWI_STAT_ADDR_W_ACK		0x18
#define R_TWI_STAT_DATA_TX_ACK		0x28
#define R_TWI_STAT_ADDR_R_ACK		0x40
#define R_TWI_STAT_DATA_RX_ACK		0x50
#define R_TWI_STAT_DATA_RX_NAK		0x58

/* R_TWI Software Reset */
#define R_TWI_SRST_RESET		BIT(0)

/* R_TWI Clock Control Register (400kHz @ 24MHz APB: CLK_M=2, CLK_N=0 -> 0x28) */
#define R_TWI_CCR_400KHZ		0x28

/* R_TWI Enhanced Feature Register (EFR) */
#define R_TWI_EFR_SDA_SCL_MASK		0x30
#define R_TWI_EFR_CLEAR_CMD		0x05
#define R_TWI_EFR_CLK_TOGGLE		0x0a

/* R_PIO Port L Pinmux for R_TWI0 (PL0=SCK func 2, PL1=SDA func 2) */
#define R_PIO_PL_CFG0			0x07025000
#define R_PIO_PL_DRV0			0x07025014
#define R_PIO_PL_PULL0			0x07025024
#define R_PIO_PL_CFG0_PL0_PL1_S_TWI0	0x22
#define R_PIO_PL_PULL0_PL0_PL1_PULLUP	0x05
#define R_PIO_PL_DRV0_PL0_PL1_LEVEL2	0x0a

/*
 * AXP8191 PMIC Register Definitions
 */
#define AXP8191_I2C_ADDR		0x36
#define AXP8191_REG_CHIP_ID		0x0e
#define AXP8191_CHIP_ID_VAL		0x03

#define AXP8191_REG_DCDC_CTRL1		0x10
#define AXP8191_REG_DCDC_CTRL2		0x11
#define AXP8191_REG_DC1_VOLT		0x12
#define AXP8191_REG_DC2_VOLT		0x13
#define AXP8191_REG_DC3_VOLT		0x14
#define AXP8191_REG_DC4_VOLT		0x15
#define AXP8191_REG_DC5_VOLT		0x16
#define AXP8191_REG_DC6_VOLT		0x17
#define AXP8191_REG_DC7_VOLT		0x18
#define AXP8191_REG_DC8_VOLT		0x19
#define AXP8191_REG_DC9_VOLT		0x1a

#define AXP8191_REG_LDO_CTRL1		0x20
#define AXP8191_REG_LDO_CTRL2		0x21
#define AXP8191_REG_LDO_CTRL3		0x22
#define AXP8191_REG_LDO_CTRL4		0x23

#define AXP8191_REG_ALDO1_VOLT		0x24
#define AXP8191_REG_ALDO2_VOLT		0x25
#define AXP8191_REG_ALDO3_VOLT		0x26
#define AXP8191_REG_ALDO4_VOLT		0x27
#define AXP8191_REG_ALDO5_VOLT		0x28
#define AXP8191_REG_ALDO6_VOLT		0x29

#define AXP8191_REG_BLDO1_VOLT		0x2a
#define AXP8191_REG_BLDO2_VOLT		0x2b
#define AXP8191_REG_BLDO3_VOLT		0x2c
#define AXP8191_REG_BLDO4_VOLT		0x2d
#define AXP8191_REG_BLDO5_VOLT		0x2e

#define AXP8191_REG_CLDO1_VOLT		0x2f
#define AXP8191_REG_CLDO2_VOLT		0x30
#define AXP8191_REG_CLDO3_VOLT		0x31
#define AXP8191_REG_CLDO4_VOLT		0x32
#define AXP8191_REG_CLDO5_VOLT		0x33

#define AXP8191_REG_DLDO1_VOLT		0x34
#define AXP8191_REG_DLDO2_VOLT		0x35
#define AXP8191_REG_DLDO3_VOLT		0x36
#define AXP8191_REG_DLDO4_VOLT		0x37
#define AXP8191_REG_DLDO5_VOLT		0x38
#define AXP8191_REG_DLDO6_VOLT		0x39

#define AXP8191_REG_ELDO1_VOLT		0x3a
#define AXP8191_REG_ELDO2_VOLT		0x3b
#define AXP8191_REG_ELDO3_VOLT		0x3c
#define AXP8191_REG_ELDO4_VOLT		0x3d
#define AXP8191_REG_ELDO5_VOLT		0x3e
#define AXP8191_REG_ELDO6_VOLT		0x3f

/* AXP8191 DCDC & LDO Enable Masks */
#define AXP8191_DCDC1_8_ALL_ON		0xff
#define AXP8191_DCDC9_DC1SW_ALL_ON	0x1f
#define AXP8191_LDO_CTRL1_VAL		0xc1
#define AXP8191_LDO_CTRL2_VAL		0xbe
#define AXP8191_LDO_CTRL3_VAL		0xe1
#define AXP8191_LDO_CTRL4_VAL		0x08

/* AXP8191 Voltage Presets for LPDDR5 and SoC Rails */
#define AXP8191_DC1_VAL_3300MV		0x17
#define AXP8191_DC2_VAL_VDD_CPU_800MV	0x9e
#define AXP8191_DC3_VAL_1050MV		0xb7
#define AXP8191_DC4_VAL_800MV		0x9e
#define AXP8191_DC5_VAL_1000MV		0xb2
#define AXP8191_DC6_VAL_VDDQ_560MV	0x86
#define AXP8191_DC7_VAL_VDD2_1080MV	0xba
#define AXP8191_DC8_VAL_1800MV		0xe4
#define AXP8191_DC9_VAL_1200MV		0xc8

#define AXP8191_ALDO1_VAL_3300MV	0x1c
#define AXP8191_ALDO2_VAL_2800MV	0x17
#define AXP8191_ALDO3_VAL_2800MV	0x17
#define AXP8191_ALDO4_VAL_2800MV	0x17
#define AXP8191_ALDO5_VAL_3300MV	0x1c
#define AXP8191_ALDO6_VAL_1800MV	0x0d

#define AXP8191_BLDO1_VAL_VDD1_1800MV	0x0d
#define AXP8191_BLDO2_VAL_1800MV	0x0d
#define AXP8191_BLDO3_VAL_1200MV	0x07
#define AXP8191_BLDO4_VAL_1800MV	0x0d
#define AXP8191_BLDO5_VAL_1800MV	0x0d

#define AXP8191_CLDO1_VAL_1800MV	0x0d
#define AXP8191_CLDO2_VAL_1800MV	0x0d
#define AXP8191_CLDO3_VAL_1800MV	0x0d
#define AXP8191_CLDO4_VAL_1800MV	0x0d
#define AXP8191_CLDO5_VAL_1800MV	0x0d

#define AXP8191_DLDO1_VAL_1800MV	0x0d
#define AXP8191_DLDO2_VAL_3300MV	0x1c
#define AXP8191_DLDO3_VAL_2800MV	0x17
#define AXP8191_DLDO4_VAL_3300MV	0x1c
#define AXP8191_DLDO5_VAL_2800MV	0x17
#define AXP8191_DLDO6_VAL_2500MV	0x14

#define AXP8191_ELDO1_VAL_1080MV	0x17
#define AXP8191_ELDO2_VAL_800MV		0x0c
#define AXP8191_ELDO3_VAL_1200MV		0x1c
#define AXP8191_ELDO4_VAL_1200MV		0x1c
#define AXP8191_ELDO5_VAL_1200MV		0x1c
#define AXP8191_ELDO6_VAL_800MV		0x0c

/**
 * r_twi_wait_status() - Wait for R_TWI controller status code
 * @expected: Expected 8-bit status code from R_TWI_STAT register
 *
 * Polls the R_TWI interrupt flag until the controller transitions to the
 * expected bus state or a timeout occurs.
 *
 * Return: 0 if expected status matched, -1 on timeout, -2 on unexpected status.
 */
static int r_twi_wait_status(u32 expected)
{
	int to = 10000;
	while (!(readl(IOMEM(R_TWI_CNTR)) & R_TWI_CNTR_INTFLAG) && --to)
		udelay(1);
	if (!to) {
		printf("r_twi: timeout! CNTR=0x%02x STAT=0x%02x EFR=0x%02x\n",
			readl(IOMEM(R_TWI_CNTR)),
			readl(IOMEM(R_TWI_STAT)),
			readl(IOMEM(R_TWI_EFR)));
		return -1;
	}
	u32 st = readl(IOMEM(R_TWI_STAT)) & 0xff;
	if (st != expected) {
		printf("r_twi: STAT=0x%02x (expected 0x%02x), CNTR=0x%02x\n",
			st, expected, readl(IOMEM(R_TWI_CNTR)));
		return -2;
	}
	return 0;
}

/**
 * r_twi_start() - Transmit START condition on R_TWI I2C bus
 *
 * Return: 0 on success (status 0x08), negative error code on failure.
 */
static int r_twi_start(void)
{
	writel(0x00, IOMEM(R_TWI_LCR));
	writel(R_TWI_CNTR_BUSEN | R_TWI_CNTR_START | R_TWI_CNTR_INTFLAG, IOMEM(R_TWI_CNTR));
	return r_twi_wait_status(R_TWI_STAT_START);
}

/**
 * r_twi_stop() - Transmit STOP condition on R_TWI I2C bus
 */
static void r_twi_stop(void)
{
	writel(R_TWI_CNTR_BUSEN | R_TWI_CNTR_STOP | R_TWI_CNTR_INTFLAG, IOMEM(R_TWI_CNTR));
	int to = 10000;
	while ((readl(IOMEM(R_TWI_CNTR)) & R_TWI_CNTR_STOP) && --to)
		udelay(1);
}

/**
 * r_twi_send_byte() - Transmit a single byte over R_TWI bus
 * @byte: Byte to transmit
 * @expected: Expected bus status code following transmission
 *
 * Return: 0 on success, negative error code on failure.
 */
static int r_twi_send_byte(u8 byte, u32 expected)
{
	writel(byte, IOMEM(R_TWI_DATA));
	writel(R_TWI_CNTR_BUSEN | R_TWI_CNTR_INTFLAG, IOMEM(R_TWI_CNTR));
	return r_twi_wait_status(expected);
}

/**
 * r_twi_recv_byte() - Receive a single byte over R_TWI bus
 * @byte: Output pointer to store received byte
 * @ack: true to send ACK after reception, false to send NACK
 *
 * Return: 0 on success, negative error code on failure.
 */
static int r_twi_recv_byte(u8 *byte, bool ack)
{
	u32 ctrl = R_TWI_CNTR_BUSEN | R_TWI_CNTR_INTFLAG | (ack ? R_TWI_CNTR_AACK : 0);
	writel(ctrl, IOMEM(R_TWI_CNTR));
	int ret = r_twi_wait_status(ack ? R_TWI_STAT_DATA_RX_ACK : R_TWI_STAT_DATA_RX_NAK);
	if (ret == 0)
		*byte = readl(IOMEM(R_TWI_DATA)) & 0xff;
	return ret;
}

/**
 * axp_i2c_write() - Write register on PMIC over R_TWI bus
 * @chip: 7-bit I2C device address
 * @reg: PMIC register offset
 * @val: Byte value to write
 *
 * Return: 0 on success, negative error code on communication failure.
 */
static int axp_i2c_write(u8 chip, u8 reg, u8 val)
{
	if (r_twi_start() != 0) {
		r_twi_stop();
		return -1;
	}
	if (r_twi_send_byte(chip << 1, R_TWI_STAT_ADDR_W_ACK) != 0) {
		r_twi_stop();
		return -2;
	}
	if (r_twi_send_byte(reg, R_TWI_STAT_DATA_TX_ACK) != 0) {
		r_twi_stop();
		return -3;
	}
	if (r_twi_send_byte(val, R_TWI_STAT_DATA_TX_ACK) != 0) {
		r_twi_stop();
		return -4;
	}
	r_twi_stop();
	return 0;
}

/**
 * axp_i2c_read() - Read register on PMIC over R_TWI bus
 * @chip: 7-bit I2C device address
 * @reg: PMIC register offset
 * @val: Output pointer to store read byte
 *
 * Return: 0 on success, negative error code on communication failure.
 */
static int axp_i2c_read(u8 chip, u8 reg, u8 *val)
{
	if (r_twi_start() != 0) {
		r_twi_stop();
		return -1;
	}
	if (r_twi_send_byte(chip << 1, R_TWI_STAT_ADDR_W_ACK) != 0) {
		r_twi_stop();
		return -2;
	}
	if (r_twi_send_byte(reg, R_TWI_STAT_DATA_TX_ACK) != 0) {
		r_twi_stop();
		return -3;
	}
	/* Repeated START */
	writel(R_TWI_CNTR_BUSEN | R_TWI_CNTR_START | R_TWI_CNTR_INTFLAG, IOMEM(R_TWI_CNTR));
	int to = 10000;
	while (!(readl(IOMEM(R_TWI_CNTR)) & R_TWI_CNTR_INTFLAG) && --to)
		udelay(1);
	u32 st = readl(IOMEM(R_TWI_STAT)) & 0xff;
	if (st != R_TWI_STAT_REPEATED_START && st != R_TWI_STAT_START) {
		printf("r_twi rep_start: unexpected STAT=0x%02x\n", st);
		r_twi_stop();
		return -4;
	}
	if (r_twi_send_byte((chip << 1) | 1, R_TWI_STAT_ADDR_R_ACK) != 0) {
		r_twi_stop();
		return -5;
	}
	if (r_twi_recv_byte(val, false) != 0) {
		r_twi_stop();
		return -6;
	}
	r_twi_stop();
	return 0;
}

/**
 * sunxi_pmic_init_a733() - Initialize PMIC and configure memory voltage rails
 * @dram_type: DRAM memory type (e.g. SUNXI_DRAM_TYPE_LPDDR5)
 *
 * Initializes R_TWI0 controller, configures PL0/PL1 pinmux, probes AXP8191 PMIC,
 * and sets voltages and enables for VDD2 (1080mV), VDDQ (560mV), BLDOs (1800mV),
 * and VDD_CPU (800mV).
 *
 * Currently only LPDDR5 is supported. Further work is required to configure
 * appropriate voltage rails and power-up sequences for other memory types
 * (such as LPDDR4X or DDR4).
 *
 * Return: 0 on success, negative error code on failure or unsupported DRAM type.
 */
static int sunxi_pmic_init_a733(u32 dram_type)
{
	u8 chip = AXP8191_I2C_ADDR;
	u8 val = 0;
	int ret;

	if (dram_type != SUNXI_DRAM_TYPE_LPDDR5) {
		printf("PMIC: DRAM type %u not supported (only LPDDR5 implemented; further work required for other memory)\n",
		       dram_type);
		return -EINVAL;
	}

	printf("PMIC: initializing A733 R_TWI PMIC interface...\n");

	/* 1. Ungate PRCM R_TWI clock & deassert reset */
	writel(PRCM_BGR_UNRESET_GATE, IOMEM(PRCM_S_TWI_BGR_REG));

	/* 2. Configure Pinmux for PL0 (SCK, func 2: s_twi0) and PL1 (SDA, func 2: s_twi0) with pull-ups */
	clrsetbits_le32(IOMEM(R_PIO_PL_CFG0), 0xff, R_PIO_PL_CFG0_PL0_PL1_S_TWI0);
	clrsetbits_le32(IOMEM(R_PIO_PL_PULL0), 0x0f, R_PIO_PL_PULL0_PL0_PL1_PULLUP);
	clrsetbits_le32(IOMEM(R_PIO_PL_DRV0), 0x0f, R_PIO_PL_DRV0_PL0_PL1_LEVEL2);

	/* 3. Initialize R_TWI controller */
	writel(R_TWI_SRST_RESET, IOMEM(R_TWI_SRST));
	udelay(100);

	/* 4. Bus clear if SDA or SCL lines are held low */
	if ((readl(IOMEM(R_TWI_EFR)) & R_TWI_EFR_SDA_SCL_MASK) != R_TWI_EFR_SDA_SCL_MASK) {
		writel(R_TWI_EFR_CLEAR_CMD, IOMEM(R_TWI_EFR));
		udelay(500);
		for (int i = 0; i < 10; i++) {
			writel(readl(IOMEM(R_TWI_EFR)) | R_TWI_EFR_CLK_TOGGLE, IOMEM(R_TWI_EFR));
			udelay(1000);
			writel(readl(IOMEM(R_TWI_EFR)) & ~R_TWI_EFR_CLK_TOGGLE, IOMEM(R_TWI_EFR));
			udelay(1000);
		}
		writel(0x00, IOMEM(R_TWI_EFR));
		udelay(500);
	}

	/* 5. Set baud rate (400 kHz) and enable bus */
	writel(R_TWI_CCR_400KHZ, IOMEM(R_TWI_CCR));
	writel(R_TWI_CNTR_BUSEN, IOMEM(R_TWI_CNTR));
	writel(0x00, IOMEM(R_TWI_LCR));
	udelay(100);

	/* 6. Probe PMIC (AXP8191 at 0x36), fallback to bus scan if not responding */
	ret = axp_i2c_read(chip, AXP8191_REG_CHIP_ID, &val);
	if (ret != 0) {
		printf("PMIC: failed to read chip ID at 0x%02x (ret=%d), scanning bus...\n", chip, ret);
		for (u8 a = 1; a < 0x78; a++) {
			if (r_twi_start() == 0) {
				if (r_twi_send_byte(a << 1, R_TWI_STAT_ADDR_W_ACK) == 0) {
					printf("PMIC: device responded at 0x%02x!\n", a);
					chip = a;
				}
				r_twi_stop();
			}
		}
		ret = axp_i2c_read(chip, AXP8191_REG_CHIP_ID, &val);
		if (ret != 0) {
			printf("PMIC: retry at 0x%02x failed (ret=%d)!\n", chip, ret);
			return ret;
		}
	}
	printf("PMIC: detected Chip ID 0x%02x\n", val);
	if (val != AXP8191_CHIP_ID_VAL) {
		printf("PMIC: unexpected Chip ID 0x%02x (expected 0x%02x for AXP8191)!\n",
		       val, AXP8191_CHIP_ID_VAL);
	}

	/* 7. LPDDR5 PMIC Sequence from running Linux / boot0 axp8191_live_regs.txt:
	 * Sets DCDC1..9 voltages and enables, DC1SW1/2 switches, and all LDOs (BLDOs @ 1.8V for DRAM VDD1) */
	static const struct {
		u8 reg;
		u8 val;
	} axp8191_lpddr5_seq[] = {
		/* DCDC voltages */
		{ AXP8191_REG_DC2_VOLT,   AXP8191_DC2_VAL_VDD_CPU_800MV }, /* DCDC2 = 800mV (VDD_CPU) */
		{ AXP8191_REG_DC3_VOLT,   AXP8191_DC3_VAL_1050MV },
		{ AXP8191_REG_DC4_VOLT,   AXP8191_DC4_VAL_800MV },
		{ AXP8191_REG_DC5_VOLT,   AXP8191_DC5_VAL_1000MV },
		{ AXP8191_REG_DC6_VOLT,   AXP8191_DC6_VAL_VDDQ_560MV },    /* DCDC6 = 560mV (VDDQ LPDDR5) */
		{ AXP8191_REG_DC7_VOLT,   AXP8191_DC7_VAL_VDD2_1080MV },   /* DCDC7 = 1080mV (VDD2 LPDDR5) */
		{ AXP8191_REG_DC8_VOLT,   AXP8191_DC8_VAL_1800MV },
		{ AXP8191_REG_DC9_VOLT,   AXP8191_DC9_VAL_1200MV },

		/* DCDC enables & DC1 voltage */
		{ AXP8191_REG_DCDC_CTRL1, AXP8191_DCDC1_8_ALL_ON },        /* DCDC1..8 enable */
		{ AXP8191_REG_DCDC_CTRL2, AXP8191_DCDC9_DC1SW_ALL_ON },    /* DCDC9, DC1SW1, DC1SW2 enable */
		{ AXP8191_REG_DC1_VOLT,   AXP8191_DC1_VAL_3300MV },

		/* LDO voltages */
		{ AXP8191_REG_ALDO1_VOLT, AXP8191_ALDO1_VAL_3300MV },
		{ AXP8191_REG_ALDO2_VOLT, AXP8191_ALDO2_VAL_2800MV },
		{ AXP8191_REG_ALDO3_VOLT, AXP8191_ALDO3_VAL_2800MV },
		{ AXP8191_REG_ALDO4_VOLT, AXP8191_ALDO4_VAL_2800MV },
		{ AXP8191_REG_ALDO5_VOLT, AXP8191_ALDO5_VAL_3300MV },
		{ AXP8191_REG_ALDO6_VOLT, AXP8191_ALDO6_VAL_1800MV },

		{ AXP8191_REG_BLDO1_VOLT, AXP8191_BLDO1_VAL_VDD1_1800MV }, /* BLDO1 = 1.8V (VDD1 LPDDR5) */
		{ AXP8191_REG_BLDO2_VOLT, AXP8191_BLDO2_VAL_1800MV },      /* BLDO2 = 1.8V */
		{ AXP8191_REG_BLDO3_VOLT, AXP8191_BLDO3_VAL_1200MV },
		{ AXP8191_REG_BLDO4_VOLT, AXP8191_BLDO4_VAL_1800MV },      /* BLDO4 = 1.8V */
		{ AXP8191_REG_BLDO5_VOLT, AXP8191_BLDO5_VAL_1800MV },

		{ AXP8191_REG_CLDO1_VOLT, AXP8191_CLDO1_VAL_1800MV },
		{ AXP8191_REG_CLDO2_VOLT, AXP8191_CLDO2_VAL_1800MV },
		{ AXP8191_REG_CLDO3_VOLT, AXP8191_CLDO3_VAL_1800MV },
		{ AXP8191_REG_CLDO4_VOLT, AXP8191_CLDO4_VAL_1800MV },
		{ AXP8191_REG_CLDO5_VOLT, AXP8191_CLDO5_VAL_1800MV },

		{ AXP8191_REG_DLDO1_VOLT, AXP8191_DLDO1_VAL_1800MV },
		{ AXP8191_REG_DLDO2_VOLT, AXP8191_DLDO2_VAL_3300MV },
		{ AXP8191_REG_DLDO3_VOLT, AXP8191_DLDO3_VAL_2800MV },
		{ AXP8191_REG_DLDO4_VOLT, AXP8191_DLDO4_VAL_3300MV },
		{ AXP8191_REG_DLDO5_VOLT, AXP8191_DLDO5_VAL_2800MV },
		{ AXP8191_REG_DLDO6_VOLT, AXP8191_DLDO6_VAL_2500MV },

		{ AXP8191_REG_ELDO1_VOLT, AXP8191_ELDO1_VAL_1080MV },      /* ELDO1 = 1080mV */
		{ AXP8191_REG_ELDO2_VOLT, AXP8191_ELDO2_VAL_800MV },
		{ AXP8191_REG_ELDO3_VOLT, AXP8191_ELDO3_VAL_1200MV },
		{ AXP8191_REG_ELDO4_VOLT, AXP8191_ELDO4_VAL_1200MV },
		{ AXP8191_REG_ELDO5_VOLT, AXP8191_ELDO5_VAL_1200MV },
		{ AXP8191_REG_ELDO6_VOLT, AXP8191_ELDO6_VAL_800MV },

		/* LDO enables */
		{ AXP8191_REG_LDO_CTRL1,  AXP8191_LDO_CTRL1_VAL },          /* LDO on/off 1 */
		{ AXP8191_REG_LDO_CTRL2,  AXP8191_LDO_CTRL2_VAL },          /* LDO on/off 2 (BLDOs on) */
		{ AXP8191_REG_LDO_CTRL3,  AXP8191_LDO_CTRL3_VAL },          /* LDO on/off 3 (ELDOs on) */
		{ AXP8191_REG_LDO_CTRL4,  AXP8191_LDO_CTRL4_VAL },
	};

	for (size_t i = 0; i < ARRAY_SIZE(axp8191_lpddr5_seq); i++) {
		axp_i2c_write(chip, axp8191_lpddr5_seq[i].reg, axp8191_lpddr5_seq[i].val);
	}

	printf("PMIC: all power rails configured: VDD2=1080mV (DCDC7), VDDQ=560mV (DCDC6), BLDOs=1800mV (VDD1), VDD_CPU=800mV (DCDC2)\n");

	/* Delay 10ms for power rails to ramp and stabilize before DRAM PHY PLL starts */
	mdelay(10);

	return 0;
}



/**
 * mctl_mr_write() - Write mode register to both ranks of a channel
 * @ch: Channel index (0 or 1)
 * @mr_addr: Mode register address
 * @val_rank0: Value to write to Rank 0
 * @val_rank1: Value to write to Rank 1
 */
static __maybe_unused void mctl_mr_write(int ch, u8 mr_addr, u8 val_rank0, u8 val_rank1)
{
	uintptr_t base = (ch == 0) ? SUNXI_UMCTL2_CH0_BASE : SUNXI_UMCTL2_CH1_BASE;
	int timeout;

	/* Poll MRSTAT until controller not busy */
	timeout = 50000;
	while ((readl(IOMEM(base + UMCTL2_REG_MRSTAT)) & UMCTL2_MRSTAT_BUSY) && --timeout)
		udelay(1);
	if (!timeout)
		printf("A733 DRAM: Ch%d MRSTAT busy timeout (before Rank 0)\n", ch);

	/* Write to Rank 0 */
	writel(UMCTL2_MRCTRL0_RANK0, IOMEM(base + UMCTL2_REG_MRCTRL0));
	writel(((u32)mr_addr << 8) | val_rank0, IOMEM(base + UMCTL2_REG_MRCTRL1));
	writel(UMCTL2_MRCTRL0_TRIGGER | UMCTL2_MRCTRL0_RANK0, IOMEM(base + UMCTL2_REG_MRCTRL0));
	timeout = 50000;
	while ((readl(IOMEM(base + UMCTL2_REG_MRCTRL0)) & UMCTL2_MRCTRL0_TRIGGER) && --timeout)
		udelay(1);
	if (!timeout)
		printf("A733 DRAM: Ch%d MRCTRL0 mr_wr timeout for Rank 0 MR%d\n", ch, mr_addr);

	/* Poll MRSTAT until controller not busy */
	timeout = 50000;
	while ((readl(IOMEM(base + UMCTL2_REG_MRSTAT)) & UMCTL2_MRSTAT_BUSY) && --timeout)
		udelay(1);
	if (!timeout)
		printf("A733 DRAM: Ch%d MRSTAT busy timeout (before Rank 1)\n", ch);

	/* Write to Rank 1 */
	writel(UMCTL2_MRCTRL0_RANK1, IOMEM(base + UMCTL2_REG_MRCTRL0));
	writel(((u32)mr_addr << 8) | val_rank1, IOMEM(base + UMCTL2_REG_MRCTRL1));
	writel(UMCTL2_MRCTRL0_TRIGGER | UMCTL2_MRCTRL0_RANK1, IOMEM(base + UMCTL2_REG_MRCTRL0));
	timeout = 50000;
	while ((readl(IOMEM(base + UMCTL2_REG_MRCTRL0)) & UMCTL2_MRCTRL0_TRIGGER) && --timeout)
		udelay(1);
	if (!timeout)
		printf("A733 DRAM: Ch%d MRCTRL0 mr_wr timeout for Rank 1 MR%d\n", ch, mr_addr);

	/* Poll MRSTAT until controller not busy */
	timeout = 50000;
	while ((readl(IOMEM(base + UMCTL2_REG_MRSTAT)) & UMCTL2_MRSTAT_BUSY) && --timeout)
		udelay(1);
}

/**
 * mctl_mr_read() - Trigger mode register read handshake on a channel
 * @ch: Channel index (0 or 1)
 * @mr_addr: Mode register address to read
 */
static __maybe_unused void mctl_mr_read(int ch, u8 mr_addr)
{
	uintptr_t base = (ch == 0) ? SUNXI_UMCTL2_CH0_BASE : SUNXI_UMCTL2_CH1_BASE;
	int timeout;

	/* Poll MRSTAT until controller not busy */
	timeout = 50000;
	while ((readl(IOMEM(base + UMCTL2_REG_MRSTAT)) & UMCTL2_MRSTAT_BUSY) && --timeout)
		udelay(1);

	/* Read from Rank 0 */
	writel(UMCTL2_MRCTRL0_RANK0 | UMCTL2_MRCTRL0_CMD_READ, IOMEM(base + UMCTL2_REG_MRCTRL0));
	writel((u32)mr_addr << 8, IOMEM(base + UMCTL2_REG_MRCTRL1));
	writel(UMCTL2_MRCTRL0_TRIGGER | UMCTL2_MRCTRL0_RANK0 | UMCTL2_MRCTRL0_CMD_READ,
	       IOMEM(base + UMCTL2_REG_MRCTRL0));
	timeout = 50000;
	while ((readl(IOMEM(base + UMCTL2_REG_MRCTRL0)) & UMCTL2_MRCTRL0_TRIGGER) && --timeout)
		udelay(1);

	/* Poll MRSTAT until controller not busy */
	timeout = 50000;
	while ((readl(IOMEM(base + UMCTL2_REG_MRSTAT)) & UMCTL2_MRSTAT_BUSY) && --timeout)
		udelay(1);

	/* Read from Rank 1 */
	writel(UMCTL2_MRCTRL0_RANK1 | UMCTL2_MRCTRL0_CMD_READ, IOMEM(base + UMCTL2_REG_MRCTRL0));
	writel((u32)mr_addr << 8, IOMEM(base + UMCTL2_REG_MRCTRL1));
	writel(UMCTL2_MRCTRL0_TRIGGER | UMCTL2_MRCTRL0_RANK1 | UMCTL2_MRCTRL0_CMD_READ,
	       IOMEM(base + UMCTL2_REG_MRCTRL0));
	timeout = 50000;
	while ((readl(IOMEM(base + UMCTL2_REG_MRCTRL0)) & UMCTL2_MRCTRL0_TRIGGER) && --timeout)
		udelay(1);

	/* Poll MRSTAT until controller not busy */
	timeout = 50000;
	while ((readl(IOMEM(base + UMCTL2_REG_MRSTAT)) & UMCTL2_MRSTAT_BUSY) && --timeout)
		udelay(1);
}

/**
 * a733_dram_switch_to_normal_op() - Switch DRAM and controller into normal operation
 *
 * Writes MR16 and reads MR14/MR15 across channels to finalize operational mode.
 */
static __maybe_unused void a733_dram_switch_to_normal_op(void)
{
	/* Switch PHY APB to controller (Boot0 0x576fc) */
	writew(0, IOMEM(SUNXI_DRAM_PHY_CTRL_BASE + PMU_REG_APB_MUX));

	/* Mode Register writes matching Boot0 0x576e4 & 0x4b554:
	 * 1. Set MR16: FSP-OP=1, CBT=0, FSP-WR=1 (val=5)
	 */
	mctl_mr_write(0, 0x10, 5, 5);
	mctl_mr_write(1, 0x10, 5, 5);

	/* 2. MR14 read handshake (VREF_DQ latch, Boot0 0x4b5c8) */
	mctl_mr_read(0, 0x0e);
	mctl_mr_read(1, 0x0e);

	/* 3. MR15 read handshake (VREF_CA latch, Boot0 0x4b5c8) */
	mctl_mr_read(0, 0x0f);
	mctl_mr_read(1, 0x0f);

	/* 4. Lock normal operation with MR16: FSP-OP=1, CBT=0, FSP-WR=0 (val=4) */
	mctl_mr_write(0, 0x10, 4, 4);
	mctl_mr_write(1, 0x10, 4, 4);

	/* Restore PHY host APB access (Boot0 0x57892) */
	writew(1, IOMEM(SUNXI_DRAM_PHY_CTRL_BASE + PMU_REG_APB_MUX));
	udelay(100);
	dsb();
	isb();
	printf("A733 DRAM: Mode Registers configured (MR16=0x04 normal op, CBT disabled)\n");
}

/**
 * a733_dump_and_test_dram() - Run comprehensive DRAM read/write tests
 * @phase: Human-readable description string for test banner
 *
 * Tests single-word read/write across memory regions and executes the
 * dual-channel 16KB+16KB pattern test (4096 words per channel).
 *
 * Return: Number of memory mismatches (0 on success).
 */
static int a733_dump_and_test_dram(const char *phase)
{
	printf("\n=================== DRAM DIAGNOSTICS: %s ===================\n", phase);

	/* Memory Access Tests on Normal DRAM regions */
	static const uintptr_t test_addrs[] = {
		0x40000000, /* Ch0 base */
		0x40800000, /* 8MB offset */
		0x44000000, /* 64MB offset (FIT load buffer) */
		0x4a000000, /* 160MB offset (U-Boot text base) */
		0x50000000, /* 256MB offset */
		0xc0000000, /* Ch1 base (2GB offset) */
	};

	u32 orig_da = readl(IOMEM(CPU_DA_DDR_CTRL_REG));
	printf("Initial CPU_DA_DDR_CTRL_REG (0x08000200) = 0x%08x (%s)\n",
	       orig_da, (orig_da & 1) ? "Direct AXI2HIF" : "NSI Interconnect");

	/* Probe combinations of Mode (NSI vs Direct AXI2HIF) and PCTRL_0 port_en (bit 0) */
	for (int port_en = 0; port_en <= 1; port_en++) {
		if (port_en) {
			printf("Toggling PCTRL_0 port_en (bit 0 = 1) on both channels...\n");
			for (u32 ch = 0; ch < 2; ch++) {
				u32 base = (ch == 0) ? SUNXI_UMCTL2_CH0_BASE : SUNXI_UMCTL2_CH1_BASE;
				void __iomem *swctl = IOMEM(base + UMCTL2_REG_SWCTL);
				void __iomem *swstat = IOMEM(base + UMCTL2_REG_SWSTAT);
				void __iomem *pctrl = IOMEM(base + UMCTL2_REG_DERATEEN);

				writel(0, swctl);
				writel(readl(pctrl) | 1, pctrl);
				writel(1, swctl);
				while ((readl(swstat) & 1) == 0)
					udelay(1);
			}
		}

		for (int mode = 0; mode <= 1; mode++) {
			writel(mode, IOMEM(CPU_DA_DDR_CTRL_REG));
			dsb();
			isb();
			printf("Testing with 0x08000200=%d (%s), PCTRL_0 port_en=%d:\n",
			       mode, mode ? "Direct AXI2HIF" : "NSI Interconnect", port_en);
			int passes = 0;
			for (size_t i = 0; i < ARRAY_SIZE(test_addrs); i++) {
				uintptr_t addr = test_addrs[i];
				u32 pat = 0xa5a50000 | (u32)(addr >> 16);
				writel(pat, IOMEM(addr));
				dsb(); isb();
				u32 val = readl(IOMEM(addr));
				bool ok = (val == pat);
				if (ok)
					passes++;
				printf("  0x%08lx: wrote 0x%08x, read 0x%08x (%s)\n",
				       addr, pat, val, ok ? "PASS" : "FAIL");
			}
			if (passes == ARRAY_SIZE(test_addrs)) {
				printf("  >> PASSED all single-word tests! (mode=%d, port_en=%d)\n", mode, port_en);
				goto test_done;
			}
		}
	}
test_done:

	/* Boot0 0x4b4cc Dual-Channel Memory Pattern Test (4096 words per channel) */
	printf("Running Boot0 0x4b4cc dual-channel pattern test (4096 words Ch0 @ 0x40000000, Ch1 @ 0xc0000000)...\n");
	volatile u32 *ch0_buf = (volatile u32 *)0x40000000;
	volatile u32 *ch1_buf = (volatile u32 *)0xc0000000;
	int b0_errors = 0;
	for (int i = 0; i < 4096; i++) {
		ch0_buf[i] = 0x01234567 + i;
		ch1_buf[i] = 0xfedcba98 + i;
	}
	dsb(); isb();
	for (int i = 0; i < 4096; i++) {
		u32 r0 = ch0_buf[i];
		u32 r1 = ch1_buf[i];
		if (r0 != (0x01234567 + i)) {
			if (b0_errors < 4)
				printf("  Ch0 mismatch at [%d]: exp 0x%08x, read 0x%08x\n", i, 0x01234567 + i, r0);
			b0_errors++;
		}
		if (r1 != (0xfedcba98 + i)) {
			if (b0_errors < 4)
				printf("  Ch1 mismatch at [%d]: exp 0x%08x, read 0x%08x\n", i, 0xfedcba98 + i, r1);
			b0_errors++;
		}
	}
	if (b0_errors == 0)
		printf("  >> Boot0 Dual-Channel 16KB+16KB Pattern Test: ALL PASS!\n");
	else
		printf("  >> Boot0 Dual-Channel Pattern Test: %d MISMATCHES!\n", b0_errors);

	printf("============================================================\n\n");
	return b0_errors;
}

/**
 * sunxi_security_and_bus_init() - Unlock security firewalls and initialize buses
 *
 * Configures Security Protection Controller (SPC), CCMU Non-Secure switch,
 * AHB/MBUS matrix clock gating, PRCM bus security gates, IOMMU bypass,
 * and RESCAL impedance calibration.
 */
static void sunxi_security_and_bus_init(void)
{
	uintptr_t addr;

	/* 1. Security Protection Controller (SPC): ungate all 24 master ports */
	for (addr = SPC_PORT_START_REG; addr <= SPC_PORT_END_REG; addr += 4)
		writel(0xffffffff, IOMEM(addr));

	/* 2. CCMU Security Switch (CCU + 0x1F00): unlock Non-Secure access to MBUS, BUS, and PLL clock registers */
	writel(0x00000007, IOMEM(CCU_SEC_SWITCH_REG));

	/* 3. Matrix clock gating & MBUS Gate enable (Allwinner User Manual Sec 4.1.6.82 - 4.1.6.85) */
	writel(0x110103ff, IOMEM(CCU_AHB_MAT_CLK_GATING_REG));
	writel(0xf1055803, IOMEM(CCU_MBUS_MAT_CLK_GATING_REG));
	writel(0x0007ffff, IOMEM(CCU_MBUS_GATE_EN_REG));

	/* 4. CCU bus security registers */
	for (addr = CCU_SEC_CTRL_START_REG; addr <= CCU_SEC_CTRL_END_REG; addr += 4)
		writel(0xffffffff, IOMEM(addr));

	/* 5. PRCM bus security & clocks (Allwinner User Manual Sec 4.2.4 & 4.2.5) */
	writel(0xffffffff, IOMEM(PRCM_SEC_CTRL_REG0));
	writel(0xffffffff, IOMEM(PRCM_SEC_CTRL_REG1));
	writel(0xffffffff, IOMEM(PRCM_SEC_CTRL_REG2));
	writel(readl(IOMEM(PRCM_SEC_SWITCH_REG)) | 0x7, IOMEM(PRCM_SEC_SWITCH_REG));
	writel(PRCM_BGR_UNRESET_GATE, IOMEM(PRCM_RTC_BGR_REG)); /* Un-gate & deassert reset for R_RTC */

	/* 6. IOMMU auto-bypass for physical DRAM addressing */
	writel(0xffffffff, IOMEM(SUNXI_IOMMU0_AUTO_BYPASS_REG));
	writel(0xffffffff, IOMEM(SUNXI_IOMMU1_AUTO_BYPASS_REG));

	/* 7. RESCAL enable (SYSCTRL RESCAL enable and calibration trigger) */
	writel(readl(IOMEM(SYSCTRL_RESCAL_CTRL_REG)) | 0x100, IOMEM(SYSCTRL_RESCAL_CTRL_REG));
	writel(readl(IOMEM(SYSCTRL_RESCAL_CAL_REG)) & ~0xff, IOMEM(SYSCTRL_RESCAL_CAL_REG));

	/*
	 * We intentionally do NOT write to CPU_DA_DDR_CTRL_REG (CPU Direct Access DDR Mux):
	 * keeping it at 0 (CPU_DA_DDR_MUX_NSI) routes CPU traffic through the NSI NoC
	 * interconnect with full cache-coherency, rather than raw AXI2HIF bypass.
	 */
}

/**
 * a733_check_mem_valid() - Verify if physical memory address stores and retains data
 * @addr: Physical address to test
 *
 * Performs write/readback with two alternating bit patterns (0x12345678 and
 * 0xedcba987) to verify physical DRAM responds and retains data without floating
 * or open-circuit bus behavior. Restores original content.
 *
 * Return: true if memory retains data correctly, false otherwise.
 */
static bool a733_check_mem_valid(uintptr_t addr)
{
	u32 orig, readback;

	orig = readl(IOMEM(addr));

	writel(0x12345678, IOMEM(addr));
	dsb();
	readback = readl(IOMEM(addr));
	if (readback != 0x12345678) {
		writel(orig, IOMEM(addr));
		return false;
	}

	writel(0xedcba987, IOMEM(addr));
	dsb();
	readback = readl(IOMEM(addr));
	writel(orig, IOMEM(addr));
	dsb();

	return (readback == 0xedcba987);
}

/**
 * a733_check_mem_match() - Check if two addresses alias to the same DRAM cells
 * @addr1: First physical address
 * @addr2: Second physical address
 *
 * Writes distinct patterns to both addresses and verifies if writing to
 * addr2 modified the contents at addr1 (indicating address wraparound/aliasing).
 * Restores original contents.
 *
 * Return: true if addresses alias to the same physical memory, false otherwise.
 */
static bool a733_check_mem_match(uintptr_t addr1, uintptr_t addr2)
{
	u32 val1, val2;
	bool match;

	val1 = readl(IOMEM(addr1));
	val2 = readl(IOMEM(addr2));

	writel(0x55aa55aa, IOMEM(addr1));
	writel(0xaa55aa55, IOMEM(addr2));
	dsb();

	match = (readl(IOMEM(addr1)) == 0xaa55aa55);

	writel(val1, IOMEM(addr1));
	writel(val2, IOMEM(addr2));
	dsb();

	return match;
}

/**
 * a733_read_mr() - Read DRAM Mode Register via uMCTL2 hardware interface
 * @ch: uMCTL2 channel (0 or 1)
 * @rank: Rank index (0 or 1)
 * @mr_addr: Mode register address to read (e.g. 8 for MR8)
 *
 * Issues a hardware Mode Register Read command through uMCTL2 MRCTRL0/1
 * and retrieves the byte returned by the DRAM chip from the DRAMC common block.
 *
 * Return: Byte value read from DRAM Mode Register, or 0xff on timeout.
 */
static u8 a733_read_mr(u32 ch, u32 rank, u32 mr_addr)
{
	uintptr_t ch_offset = (uintptr_t)ch * 0x400000UL;
	void __iomem *mrstat = IOMEM(0x0a110090 + ch_offset);
	void __iomem *mrctrl0 = IOMEM(0x0a110080 + ch_offset);
	void __iomem *mrctrl1 = IOMEM(0x0a110084 + ch_offset);
	void __iomem *data_reg = (ch == 0) ? IOMEM(0x0a02002c) : IOMEM(0x0a020040);
	u32 reg, timeout = 100000;

	/* Clear readback data register before triggering read */
	writel(0, data_reg);

	while ((readl(mrstat) & 0x80000000) && --timeout)
		udelay(1);
	if (!timeout)
		return 0xff;

	reg = (1 << (rank + 4)) | 1;
	writel(reg, mrctrl0);
	writel(mr_addr << 8, mrctrl1);
	writel(reg | 0x80000000, mrctrl0);

	timeout = 100000;
	while ((readl(mrctrl0) & 0x80000000) && --timeout)
		udelay(1);
	if (!timeout)
		return 0xff;

	return (readl(data_reg) >> 8) & 0xff;
}

/**
 * sun60i_a733_auto_detect_geometry() - Dynamically probe physical DRAM capacity and geometry
 * @para: Pointer to DRAM parameter structure to update with probed geometry
 *
 * Probes the installed LPDDR5 memory configuration across all Orange Pi 4 Pro
 * board variants (4GB, 6GB, 8GB, 12GB, 16GB) using JEDEC Mode Register interrogation
 * and physical memory verification after uMCTL2 controller initialization:
 * 1. Interrogates JEDEC Mode Register 5 (MR5) to identify memory manufacturer
 *    (Samsung, SK Hynix, CXMT, Micron).
 * 2. Interrogates JEDEC Mode Register 8 (MR8) to decode physical silicon die
 *    density (16Gb, 24Gb non-power-of-two 3/4 density, or 32Gb).
 * 3. Probes Rank 2 presence at +8GiB offset to distinguish single-rank (4/6/8GB)
 *    from dual-rank (12/16GB).
 * 4. Verifies physical retention at base memory before committing parameters.
 */
static void sun60i_a733_auto_detect_geometry(struct dram_para *para)
{
	uintptr_t base = CFG_SYS_SDRAM_BASE;
	uintptr_t test_off = 0x1000;
	bool rank2_present = false;

	printf("DRAM: Probing installed memory geometry...\n");

	u8 mr5 = a733_read_mr(0, 0, 5);
	u8 mr8 = a733_read_mr(0, 0, 8);

	/* Verify base memory at 0x40001000 is functional */
	if (!a733_check_mem_valid(base + test_off)) {
		printf("DRAM: Base memory test failed at 0x%lx! Halting.\n",
		       (unsigned long)(base + test_off));
		hang();
	}

	/*
	 * JEDEC LPDDR5 Mode Register 8 (MR8) defines device density and geometry:
	 * Bits [5:2]:
	 *   0x4: 16Gb die per channel (4GB single-rank, 8GB dual-rank, 15 rows)
	 *   0x5: 24Gb die per channel (6GB single-rank, 12GB dual-rank, 16 rows, 3/4 non-power-of-two)
	 *   0x6: 32Gb die per channel (8GB single-rank, 16GB dual-rank, 16 rows)
	 */
	u8 density_code = (mr8 >> 2) & 0x0f;
	const char *mfr_str;

	switch (mr5) {
	case 0x01:
		mfr_str = "Samsung";
		break;
	case 0x06:
		mfr_str = "SK Hynix";
		break;
	case 0x13:
		mfr_str = "CXMT";
		break;
	case 0xff:
		mfr_str = "Micron";
		break;
	default:
		mfr_str = "Unknown";
		break;
	}

	printf("DRAM: LPDDR5 %s (MR5=0x%02x, MR8=0x%02x, density_code=0x%x)\n",
	       mfr_str, mr5, mr8, density_code);

	/* Probe Rank 2 presence at +8GiB */
	uintptr_t rank1_base = base + 0x200000000ULL + test_off;

	if (a733_check_mem_valid(rank1_base) && !a733_check_mem_match(base + test_off, rank1_base))
		rank2_present = true;

	para->ranks = rank2_present ? 2 : 1;
	para->cols = 10;
	para->banks = 2;
	para->bank_groups = 2;

	switch (density_code) {
	case 0x4: /* 16Gb die: 4GB single-rank, 8GB dual-rank */
		para->rows = 15;
		para->density_3_4 = false;
		break;
	case 0x5: /* 24Gb die: 6GB single-rank, 12GB dual-rank */
		para->rows = 16;
		para->density_3_4 = true;
		break;
	case 0x6: /* 32Gb die: 8GB single-rank, 16GB dual-rank */
	default:
		para->rows = 16;
		para->density_3_4 = false;
		break;
	}

	unsigned long size_mb = sun60i_a733_calc_dram_size(para) >> 20;

	printf("DRAM: Detected %lu MiB (%lu GiB) [ranks=%u, rows=%u, cols=%u, 3/4=%s]\n",
	       size_mb, size_mb >> 10, para->ranks, para->rows, para->cols,
	       para->density_3_4 ? "yes" : "no");
}

/**
 * sunxi_dram_init() - Top-level DRAM subsystem initialization entry point
 *
 * Called by U-Boot SPL during board initialization. Unlocks system security
 * domains, configures PMIC power rails, initializes CCU clocks, executes
 * DDR PHY initialization (fast-boot cached restore or full PMU training),
 * verifies memory with read/write pattern tests, dynamically probes memory
 * geometry, and returns detected size.
 *
 * Return: Detected DRAM capacity in bytes.
 */
unsigned long sunxi_dram_init(void)
{
	int ret;

	struct dram_para para = {
		.clk = CONFIG_DRAM_CLK,
		.type = SUNXI_DRAM_TYPE_LPDDR5,
		.cols = 10,
		.rows = 16,
		.banks = 2,
		.bank_groups = 2,
		.ranks = 1,
		.channels = 2,
		.density_3_4 = true,
	};
	u32 dram_clk = para.clk;
	u32 dram_type = para.type;
	const char *type_str;

	switch (dram_type) {
	case SUNXI_DRAM_TYPE_DDR3:
		type_str = "DDR3";
		break;
	case SUNXI_DRAM_TYPE_DDR4:
		type_str = "DDR4";
		break;
	case SUNXI_DRAM_TYPE_LPDDR3:
		type_str = "LPDDR3";
		break;
	case SUNXI_DRAM_TYPE_LPDDR4:
		type_str = "LPDDR4";
		break;
	case SUNXI_DRAM_TYPE_LPDDR5:
		type_str = "LPDDR5";
		break;
	default:
		type_str = "UNKNOWN";
		break;
	}

	printf("A733 sunxi_dram_init: starting %s @ %uMHz...\n", type_str, dram_clk);

	/* Unlock security domains so we can access uMCTL2/PMIC/PRCM/CCU */
	sunxi_security_and_bus_init();

	/* Check Tier 1 Warm Boot Fast-Bypass */
	u32 stat0 = readl(IOMEM(SUNXI_UMCTL2_CH0_BASE + UMCTL2_REG_STAT));
	u32 stat1 = readl(IOMEM(SUNXI_UMCTL2_CH1_BASE + UMCTL2_REG_STAT));
	u32 rtc_magic = readl(IOMEM(DRAM_RTC_MAGIC_REG));

	if (!IS_ENABLED(CONFIG_SUNXI_PMU_FORCE_TRAINING) &&
	    (stat0 & 3) == UMCTL2_STAT_NORMAL_MODE &&
	    (stat1 & 3) == UMCTL2_STAT_NORMAL_MODE &&
	    rtc_magic == DRAM_CACHE_MAGIC) {
		printf("DRAM [Tier 1]: uMCTL2 already in NORMAL MODE (STAT ch0=0x%x, ch1=0x%x)!\n", stat0, stat1);
		if (a733_quick_dram_verify()) {
			const struct dram_cache_header *hdr =
				(const struct dram_cache_header *)DRAM_RETENTION_SRAM;

			printf("DRAM [Tier 1]: Warm boot memory check PASS! Bypassing entire DRAM init (<1ms)!\n");
			if (hdr->magic == DRAM_CACHE_MAGIC && hdr->version == DRAM_CACHE_VERSION) {
				para.ranks = hdr->ranks;
				para.rows = hdr->rows;
				para.density_3_4 = (hdr->density_3_4 != 0);
			}
			return sun60i_a733_calc_dram_size(&para);
		}
	}

	/* Configure PMIC voltages for LPDDR5 before starting DRAM PHY */
	ret = sunxi_pmic_init_a733(dram_type);
	if (ret)
		return 0;

	/* Stage 1: CCU, DMC, uMCTL2 Channel 0/1 setup (missed by emulator) */
	u32 val = readl(IOMEM(PRCM_DRAM_CH0_CLK_REG));

	val = (val & 0xffffc0c0) | 0x2728; /* Dual channel */
	writel(val, IOMEM(PRCM_DRAM_CH0_CLK_REG));
	writel(val, IOMEM(PRCM_DRAM_CH1_CLK_REG));

	val = readl(IOMEM(PRCM_DRAM_CH0_CFG_REG));
	val = (val & 0xfffffffc) | 3; /* Dual channel + LPDDR5 */
	writel(val, IOMEM(PRCM_DRAM_CH0_CFG_REG));
	writel(val, IOMEM(PRCM_DRAM_CH1_CFG_REG));

	/* Un-reset and enable clock */
	val = readl(IOMEM(PRCM_DRAM_CH0_CLK_REG));
	val = (val & 0xffffffef) | 0x14;
	writel(val, IOMEM(PRCM_DRAM_CH0_CLK_REG));
	writel(val, IOMEM(PRCM_DRAM_CH1_CLK_REG));

	val = (val & 0xfffffffe) | 2;
	writel(val, IOMEM(PRCM_DRAM_CH0_CLK_REG));
	writel(val, IOMEM(PRCM_DRAM_CH1_CLK_REG));
	udelay(1);

	val |= 3;
	writel(val, IOMEM(PRCM_DRAM_CH0_CLK_REG));
	writel(val, IOMEM(PRCM_DRAM_CH1_CLK_REG));
	udelay(1);

	/* Stage 2 & 3: PHY calibration, firmware training, and handoff */
	bool fresh_trained = false;

	ret = mctl_phy_init(dram_clk, dram_type,
			    !IS_ENABLED(CONFIG_SUNXI_PMU_TRAINING_CACHE) ||
			    IS_ENABLED(CONFIG_SUNXI_PMU_FORCE_TRAINING),
			    &fresh_trained);
	if (ret != 0) {
		printf("PMU training failed! System halted.\n");
		hang();
	}

	/* Test DRAM read/write directly with MMU off */
	int test_errors = a733_dump_and_test_dram("Normal DRAM Read/Write Test");

	if (test_errors > 0) {
		if (!fresh_trained) {
			printf("DRAM: Cached timings failed pattern test! Invalidate caches and retrain with full PMU...\n");
			/* Invalidate Tier 1 (Retention SRAM) and RTC */
			struct dram_cache_header *hdr = (struct dram_cache_header *)DRAM_RETENTION_SRAM;

			hdr->magic = 0;
			writel(0, IOMEM(DRAM_RTC_MAGIC_REG));
			/* Invalidate Tier 2 (SPI NOR Flash) */
			spi0_init();
			spi0_erase_sector(DRAM_CACHE_SPI_SECTOR);
			spi0_deinit();

			ret = mctl_phy_init(dram_clk, dram_type, true, &fresh_trained);
			if (ret != 0) {
				printf("PMU re-training failed! System halted.\n");
				hang();
			}
			test_errors = a733_dump_and_test_dram("Retrained DRAM Read/Write Test");
			if (test_errors > 0) {
				printf("PMU training verification failed! System halted.\n");
				hang();
			}
		} else {
			printf("PMU training verification failed! System halted.\n");
			hang();
		}
	}

	/* Dynamically probe installed memory geometry */
	sun60i_a733_auto_detect_geometry(&para);

	if (fresh_trained) {
		if (IS_ENABLED(CONFIG_SUNXI_PMU_TRAINING_CACHE)) {
			printf("DRAM: Fresh training passed. Persisting cache to Tier 1 (Retention SRAM) and Tier 2 (SPI NOR)...\n");
			dram_save_cache_both_tiers(dram_clk, dram_type, &para);
		}
	} else {
		struct dram_cache_header *hdr = (struct dram_cache_header *)DRAM_RETENTION_SRAM;
		hdr->boot_count++;
		writel(DRAM_CACHE_MAGIC, IOMEM(DRAM_RTC_MAGIC_REG));
		printf("DRAM: Fast boot successful using cached timings (boot_count=%u, %lu MiB)!\n",
		       hdr->boot_count, sun60i_a733_calc_dram_size(&para) >> 20);
	}
	return sun60i_a733_calc_dram_size(&para);
}
