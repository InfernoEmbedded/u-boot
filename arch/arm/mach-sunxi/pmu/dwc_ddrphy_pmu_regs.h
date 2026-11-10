/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Synopsys DesignWare DDR PHY PMU Microcontroller Register & Memory Definitions
 * Target Architecture: Synopsys ARC EM4 (ARCv2 ISA)
 * SoC: Allwinner A733 (Sun60i) / LPDDR4 & LPDDR5 PHY
 *
 * =============================================================================
 * PHY SILICON ARCHITECTURE & TERMINOLOGY EXPLAINED
 * =============================================================================
 *
 * Inside the Synopsys DesignWare DDR PHY Type 9 silicon on Allwinner A733:
 *
 * 1. DBYTE Slices (Data Byte Slices 0..3):
 *    Each DBYTE slice manages the physical analog signals for one 8-bit byte lane
 *    (DQ0..DQ7, Data Mask DM, and differential Data Strobe DQS/DQSN).
 *    Because the A733 uses dual 16-bit channels (Ch0 = 16-bit, Ch1 = 16-bit),
 *    there are 4 DBYTE slices in total:
 *      - Slice 0: Ch0 Byte Lane 0 (DQ0..DQ7)
 *      - Slice 1: Ch0 Byte Lane 1 (DQ8..DQ15)
 *      - Slice 2: Ch1 Byte Lane 0 (DQ0..DQ7)
 *      - Slice 3: Ch1 Byte Lane 1 (DQ8..DQ15)
 *
 * 2. AC Slices (Address/Command Slices 0..1):
 *    Manage the clock (CK/CKN) and Command/Address (CA0..CA6) signals sent to the
 *    LPDDR5 memory chips for each channel.
 *
 * 3. PUB (PHY Utility Block):
 *    The central control core containing master PLL configuration, clock gating,
 *    PMU execution mailboxes, analog bias generators, and host CPU APB interface.
 *
 * 4. LCDL (Local Clock Delay Line):
 *    Analog tapped delay lines consisting of coarse stages and fine vernier stages.
 *    Adjusting LCDL delay taps shifts signal transitions by picoseconds, enabling
 *    the PMU to place sampling clock edges precisely in the center of the data eye.
 *
 * 5. CBT (Command Bus Training):
 *    A JEDEC-standard training mode where the memory controller transmits command
 *    patterns to the DRAM, and the DRAM returns feedback on the DQ lines. This allows
 *    the PHY to align the CA signals to the memory clock with picosecond accuracy.
 *
 * 6. ZQ Calibration:
 *    Calibrates transmitter output impedance (Ron) and receiver on-die termination
 *    (ODT) against an external precision resistor to prevent transmission line
 *    signal reflections.
 * =============================================================================
 */

#ifndef _DWC_DDRPHY_PMU_REGS_H
#define _DWC_DDRPHY_PMU_REGS_H

#include "sunxi_pmu_abi.h"

/* ========================================================================== */
/* ARC PMU Internal Memory Map (DCCM: 0x80000000)                             */
/* ========================================================================== */

#define PMU_DMEM_BASE			0x80000000UL

/* Configuration & State Variables in DCCM */
#define PMU_DMEM_CLK_GATE_FLAG		0x80000001UL /* Byte: Clock gate / broadcast flag */
#define PMU_DMEM_CAL_PARAM_05		0x80000005UL /* Byte: Calibration parameter 0x05 */
#define PMU_DMEM_DRAM_FREQ_OFF		0x06         /* Half: Clock in MHz offset */
#define PMU_DMEM_DRAM_TYPE		0x08         /* Byte: DRAM type (2 = LPDDR5) */
#define PMU_DMEM_DRAM_CFG_FLAGS		0x0a         /* Half: DRAM config bitmask */
#define PMU_DMEM_CAL_SELECT_FLAGS	0x0b         /* Byte: Calibration selector flags */
#define PMU_DMEM_SEQ_CTRL_10		0x80000010UL /* Half: Training sequence control mask */
#define PMU_DMEM_POST_THRESH_12		0x80000012UL /* Byte: Post mailbox threshold */
#define PMU_DMEM_TRAIN_MODE_14		0x80000014UL /* Half: Active training stage mode */
#define PMU_DMEM_RX_CLK_EN_19		0x80000019UL /* Byte: Receiver clock enable flag */
#define PMU_DMEM_CAL_STEP_CONFIG	0x8000001aUL /* Half: Calibration step configuration */
#define PMU_DMEM_LANE_MASK_1C		0x1c         /* Byte: Lane active mask */
#define PMU_DMEM_CH0_ANALOG_EN		0x80000025UL /* Byte: Ch0 analog power enable */
#define PMU_DMEM_CH1_ACTIVE_FLAG	0x80000040UL /* Byte: Ch1 active enable flag */
#define PMU_DMEM_PARAM_8A		0x8a         /* Byte: DRAM protocol status offset */
#define PMU_DMEM_DRAM_TYPE_REG		0x8000008eUL /* Byte: DRAM channel/rank protocol mode */
#define PMU_DMEM_TRAIN_CFG96_REG	0x80000096UL /* Byte: DRAM training config byte 0x96 */

#define PMU_DMEM_MARGIN_STAT_405	0x80000405UL /* Byte: Stage 10 margin status byte */
#define PMU_DMEM_STEP_CFG_RAW		0x8000041aUL /* Half: Raw step config register */
#define PMU_DMEM_ACTIVE_CH_OFFSET	0x8000041cUL /* Word: Active channel CSR offset */
#define PMU_DMEM_CAL_TABLE_43C		0x8000043cUL /* Half: 2D calibration parameter table */
#define PMU_DMEM_CBT_STATE_45A		0x8000045aUL /* Byte: CBT state latch */
#define PMU_DMEM_CBT_CONFIG_46A		0x8000046aUL /* Byte: CBT configuration value */
#define PMU_DMEM_NORM_STEP_STRIDE	0x8000047cUL /* Half: Normalized step stride */

#define PMU_DMEM_AC_CAL_ACTIVE		0x80000a7cUL /* Byte: AC calibration active flag */
#define PMU_DMEM_DQ_PIN_MAP_BASE	0x80000a7eUL /* Base: DBYTE DQ pin swizzle table */
#define PMU_DMEM_ACTIVE_RANK		0x80000b67UL /* Byte: Active rank / byte index */

/* ========================================================================== */
/* Synopsys DDR PHY Internal APB Registers (ARC Address Space)                */
/* ========================================================================== */

/* DBYTE & Operating Mode Registers */
#define PHY_REG_DBYTE_BASE		0x90020000UL /* Base address for DBYTE slice CSRs */
#define PHY_REG_OP_MODE_0012		0x90020012UL /* Channel-indexed PHY operating mode */
#define PHY_REG_DBYTE_CAL_TRIG_176	0x90020176UL /* DBYTE calibration strobe sequence */

/* Master PUB / Clock / PLL Registers */
#define PHY_REG_MASTER_CFG		0x9004000eUL /* Master PUB configuration */
#define PHY_REG_SLICE_STAT_0018		0x90040018UL /* Slice / lane status register */
#define PHY_REG_CLK_GATE		0x90040040UL /* Master PHY clock gate enable */
#define PHY_REG_CLK_GATE_STAT		0x90040042UL /* Master clock gate acknowledge */
#define PHY_REG_DESKEW_DLY0_44		0x90040044UL /* Deskew delay tap 0 */
#define PHY_REG_DESKEW_DLY1_46		0x90040046UL /* Deskew delay tap 1 */
#define PHY_REG_CLK_TIMING_CTRL		0x9004004eUL /* Clock timing control */
#define PHY_REG_PLL_CLK_CTRL		0x90040062UL /* Master PLL & clock control */
#define PHY_REG_VREF_CTRL		0x90040082UL /* Receiver Vref control */
#define PHY_REG_PHY_STATUS		0x900400c0UL /* Master PHY status & receiver clock */
#define PHY_REG_CH0_ANALOG_PWR		0x900400e4UL /* Ch0 analog power configuration */
#define PHY_REG_CH1_ANALOG_PWR		0x900400e6UL /* Ch1 analog power configuration */
#define PHY_REG_RESET_PULSE_0F2		0x900400f2UL /* Soft reset pulse toggle */
#define PHY_REG_CBT_CTRL		0x90040144UL /* CBT control register */
#define PHY_REG_CAL_TRIG_146		0x90040146UL /* Calibration strobe trigger */
#define PHY_REG_CAL_STAT_148		0x90040148UL /* Calibration status & configuration */
#define PHY_REG_CBT_PLL_1F0		0x900401f0UL /* CBT PLL control register */
#define PHY_REG_CMD_DISPATCH_238	0x90040238UL /* Command dispatch strobe */

/* Secondary Clock & Strobe Registers */
#define PHY_REG_CLK_GATE_E0A4		0x9005e0a4UL /* Secondary clock gate toggle */
#define PHY_REG_CAL_STROBE_E140		0x9005e140UL /* Secondary calibration strobe */

/* AC Slice & Broadcast Registers */
#define PHY_AC_PIN_DELAY_BASE		0x90060002UL /* AC pin delay base */
#define PHY_AC_BASE_OFFSET_CTRL		0x90060004UL /* AC base offset control */
#define PHY_AC_SWIZZLE_S0_BASE		0x90060120UL /* AC slice 0 pin swizzle base */
#define PHY_AC_MASTER_DESKEW0		0x90061002UL /* AC master deskew 0 */
#define PHY_AC_MASTER_DESKEW1		0x90061202UL /* AC master deskew 1 */
#define PHY_AC_GATE_CTRL_RESET		0x90061e02UL /* AC gate control reset */
#define PHY_AC_SWIZZLE_S1_BASE		0x90062120UL /* AC slice 1 pin swizzle base */
#define PHY_AC_PROFILE_LANE_BASE	0x9007e002UL /* AC profile lane base */
#define PHY_AC_PROFILE_MASTER_CTRL	0x9007e004UL /* AC profile master control */
#define PHY_REG_PLL_STATUS		0x9007e034UL /* PLL lock status */
#define PHY_REG_PLL_CFG_00C2		0x900e00c2UL /* Master PLL configuration parameter 0 */
#define PHY_REG_AC_TX_EN_BCAST		0x900e00caUL /* AC transmitter broadcast enable */

/* Mailbox Streaming & Status Registers */
#define PHY_REG_MAILBOX_STAT		0x90180008UL /* Mailbox status */
#define PHY_REG_MAILBOX_STREAM		0x90180064UL /* Mailbox stream low */
#define PHY_REG_PUB_CTL			0x901801dcUL /* PUB control */

/* Synopsys PMU Message / Event Codes */

/* Batch 2 DCCM, APB & Auxiliary Timer Definitions */
#define PMU_DMEM_STATUS_FLAG_04		0x80000004UL /* Byte: Global PMU status / error bitmask */
#define PMU_DMEM_ASSERT_CODE_0C		0x8000000cUL /* Word: Fatal assertion / panic code */
#define PMU_DMEM_VREF_FLAG_62		0x80000062UL /* Byte: Signed Vref training mode flags */
#define PMU_DMEM_VREF_CTRL_72		0x80000072UL /* Byte: Vref DAC control configuration */
#define PMU_DMEM_LANE_STATUS_B97	0x80000b97UL /* Byte: Latched slice lane status */

#define PHY_REG_TIMER_DUMMY_048		0x90040048UL /* Half: Dummy APB turnaround read register */

#define PMU_AUX_RTC_CTRL		0x103        /* AUX 259: RTC control / enable register */
#define PMU_AUX_RTC_LOW			0x104        /* AUX 260: RTC low 32-bit counter */
#define PMU_AUX_RTC_HIGH		0x105        /* AUX 261: RTC high 32-bit counter */

/* Batch 3 DCCM Variables, Tables & PHY APB Registers */
#define PMU_DMEM_DRAM_FREQ_MHZ		0x80000006UL /* Half: Operating DRAM frequency (MHz) */
#define PMU_DMEM_FREQ_ROUND_BIAS	0x80000008UL /* Byte: Frequency delay step rounding bias */
#define PMU_DMEM_VREF_CTRL_E7		0x800000e7UL /* Byte: Vref DAC override control flags */
#define PMU_DMEM_LANE_ERR_MASK_101	0x80000101UL /* Byte: Per-lane deskew error status mask */
#define PMU_DMEM_MIN_DELAY_EN_A7C	0x80000a7cUL /* Byte: Minimum delay clamp enable flag */
#define PMU_DMEM_DESKEW_DIR_B6B		0x80000b6bUL /* Byte: Active deskew sweep direction */
#define PMU_DMEM_DBYTE_BACKUP_BA0	0x80000ba0UL /* Base: DBYTE delay register backup buffer */
#define PMU_DMEM_EYE_BACKUP_BC0		0x80000bc0UL /* Base: 2D eye table backup buffer */
#define PMU_DMEM_CAL_TRACKER_D9C	0x80000d9cUL /* Base: Calibration marker scratchpad state */

#define PHY_REG_DBYTE_DELAY_100		0x90020100UL /* Half: DBYTE Slice 0 delay register base */

/* Batch 4 DCCM Variables, Tables & PHY APB Registers */
#define PMU_DMEM_CH0_ANALOG_EN_25	0x80000025UL /* Byte: Ch0 rank & analog bias */
#define PMU_DMEM_CH1_ANALOG_EN_40	0x80000040UL /* Byte: Ch1 rank & analog bias */
#define PMU_DMEM_COST_WEIGHT_2_410	0x80000410UL /* Half: Eye overflow weight */
#define PMU_DMEM_SHADOW_TRACE_PTR	0x80000418UL /* Word: Shadow trace buffer ptr */
#define PMU_DMEM_PSTATE_OFFSET_41C	0x8000041cUL /* Word: Active PState offset */
#define PMU_DMEM_COST_WEIGHT_1_422	0x80000422UL /* Half: Eye deviation weight */
#define PMU_DMEM_SAVED_CAL_STAT_42A	0x8000042aUL /* Half: Saved cal status word */
#define PMU_DMEM_DBYTE_DLY_SAVE_430	0x80000430UL /* Base: Saved DBYTE delay array */
#define PMU_DMEM_CH_RANGE_B68		0x80000b68UL /* Base: Active ch [min, max] */
#define PMU_DMEM_CH_ACTIVE_RANGE_B6A	0x80000b6aUL /* Base: Primary ch [min, max] */
#define PMU_DMEM_CH_SEC_RANGE_B6C	0x80000b6cUL /* Base: Secondary ch [min, max] */
#define PMU_DMEM_SAVED_CAL_STAT_B7E	0x80000b7eUL /* Half: Archived cal status */
#define PMU_DMEM_EYE_CLEAR_BUF_DAC	0x80000dacUL /* Base: Eye scratchpad buffer */

#define PHY_REG_APB_BASE		0x90000000UL /* Half: PHY APB base aperture */
#define PHY_REG_STREAM_ALT_BASE		0x9001e000UL /* Half: Alt APB stream base */
#define PHY_REG_DBYTE_SLICE_BASE	0x90020000UL /* Half: DBYTE slice profile base */
#define PHY_REG_DBYTE_WCK_DLY_146	0x90020146UL /* Half: DBYTE WCK delay reg */
#define PHY_REG_DBYTE_COARSE_154	0x90020154UL /* Half: DBYTE coarse gate delay */
#define PHY_REG_DBYTE_SAMPLE_16A	0x9002016aUL /* Half: DBYTE sample ready reg */
#define PHY_REG_DBYTE_PLL_LOCK_1C0	0x900201c0UL /* Half: DBYTE PLL lock status */
#define PHY_REG_DBYTE_GATE_1F6A		0x90021f6aUL /* Half: DBYTE gate control reg */
#define PHY_REG_DBYTE_BCAST_CTRL_00A	0x9003e00aUL /* Half: DBYTE bcast pulse ctrl */
#define PHY_REG_DBYTE_BCAST_WIDTH	0x9003e016UL /* Half: DBYTE bcast pulse width */
#define PHY_REG_DBYTE_SAMPLE_3F16A	0x9003f16aUL /* Half: Upper DBYTE sample A */
#define PHY_REG_DBYTE_SAMPLE_3FF6A	0x9003ff6aUL /* Half: Upper DBYTE sample B */
#define PHY_REG_CAL_STAT_CTRL_00C	0x9004000cUL /* Half: PHY cal status ctrl */
#define PHY_REG_CAL_STAT_CTRL_022	0x90040022UL /* Half: PHY cal status ctrl 0x22 */
#define PHY_REG_PUB_ACK_02A		0x9004002aUL /* Half: PUB status ack reg */
#define PHY_REG_ANALOG_CFG0		0x900400e4UL /* Half: ChA analog rx bias cfg */
#define PHY_REG_ANALOG_CFG1		0x900400e6UL /* Half: ChB analog rx bias cfg */
#define PHY_REG_AC_PLL_LOCK_6017E	0x9006017eUL /* Half: AC PLL lock status B */
#define PHY_REG_CSR_RESULT_STREAM	0x90082000UL /* Half: PHY CSR result stream */
#define PHY_REG_DESKEW_QUAD_TAP		0x900e0048UL /* Half: Deskew quad-tap base */
#define PHY_REG_DESKEW_QUAD_MASK_0CA	0x900e00caUL /* Half: Deskew ch status mask B */
#define PHY_REG_SHADOW_TRACK_BASE	0x90140000UL /* Half: Shadow track aperture */

/* Batch 5 DCCM Variables, Tables & PHY APB Registers */
#define PMU_DMEM_SIGNED_STEP_0E		0x8000000eUL /* SByte: Signed AC timing step */
#define PMU_DMEM_DESKEW_LIM_1C		0x8000001cUL /* Byte: Dual-rank deskew cfg */
#define PMU_DMEM_TOTAL_BITS_24		0x80000024UL /* Byte: Total DQ bit width Ch0 */
#define PMU_DMEM_RANK_DIV_3F		0x8000003fUL /* Byte: Total DQ bit width Ch1 */
#define PMU_DMEM_PLL_GATE_E8		0x800000e8UL /* Base: Per-rank shadow base */
#define PMU_DMEM_PLL_GATE_F2		0x800000f2UL /* Half: PLL param seq enable */
#define PMU_DMEM_AC_PROFILE_EN_F4	0x800000f4UL /* Byte: AC slice profile en */
#define PMU_DMEM_CA_VREF_STATE_103	0x80000103UL /* Byte: CA Vref state Rank 1 */
#define PMU_DMEM_CA_VREF_STATE_104	0x80000104UL /* Byte: CA Vref state Rank 0 */
#define PMU_DMEM_GATE_DESC_118		0x80000118UL /* Base: Gate eye margin desc */
#define PMU_DMEM_AC_PROFILE_CFG_444	0x80000444UL /* Base: AC profile step cfg A */
#define PMU_DMEM_AC_PROFILE_CFG_448	0x80000448UL /* Base: AC profile step cfg B */
#define PMU_DMEM_CAL_CMD_TABLE_46C	0x8000046cUL /* Base: Shadow cal cmd tbl */
#define PMU_DMEM_SLICE_CNT_LIMIT_B20	0x80000b20UL /* Base: Slice cnt limit array */
#define PMU_DMEM_DESKEW_STAT_TBL_B6E	0x80000b6eUL /* Base: Deskew lane stat tbl */
#define PMU_DMEM_EYE_SWEEP_STAT_B82	0x80000b82UL /* Base: Eye sweep status arr */
#define PMU_DMEM_ACTIVE_SLICE_MASK	0x80000b98UL /* Byte: Active slice bitmask */
#define PMU_DMEM_DELAY_TAP_LUT_E7B8	0x8000e7b8UL /* Base: Pad delay tap LUT */

#define PHY_REG_DBYTE_GATE_STAT_11A	0x9002011aUL /* Half: DBYTE sample status */
#define PHY_REG_DBYTE_EYE_CTRL_158	0x90020158UL /* Half: DBYTE eye sweep ctrl */
#define PHY_REG_DBYTE_PERIOD_1AA	0x900201aaUL /* Half: DBYTE period meas reg */
#define PHY_REG_MASTER_DELAY_01A	0x9004001aUL /* Half: Master profile delay */
#define PHY_REG_CA_VREF_DAC_058		0x90040058UL /* Half: CA Vref DAC ctrl A */
#define PHY_REG_PUB_CAL_STROBE		0x9005e140UL /* Half: PUB sec cal strobe */
#define PHY_REG_AC_SLICE_PROFILE	0x90060000UL /* Half: AC slice profile base */

/* Batch 6 DCCM Variables, Tables & PHY APB Registers */
#define PMU_DMEM_ANALOG_CFG_15		0x80000015UL /* Byte: Analog cfg flag */
#define PMU_DMEM_AC_DRIVE_DESC_0F4	0x800000f4UL /* Base: AC drive descriptor */
#define PMU_DMEM_CBT_LANE_LIM_403	0x80000403UL /* Byte: CBT strobe/lane limit */
#define PMU_DMEM_STAGE_PARAM_406	0x80000406UL /* Byte: Stage active rank idx */
#define PMU_DMEM_STAGE_PARAM_407	0x80000407UL /* Byte: Stage active slice idx */
#define PMU_DMEM_STAGE_PARAM_408	0x80000408UL /* Byte: Stage active lane idx */
#define PMU_DMEM_AC_SIGNED_STEP		0x80000a7dUL /* Byte: AC signed step offset */
#define PMU_DMEM_SLICE_IMP_CFG_B24	0x80000b24UL /* Base: Sec rank slice map */
#define PMU_DMEM_TIMING_LIM_B64		0x80000b64UL /* Byte: Gate timing limit */
#define PMU_DMEM_AC_TIMING_TBL_DC8	0x80000dc8UL /* Base: Primary AC timing tbl */
#define PMU_DMEM_AC_TIMING_DESC_3708	0x80003708UL /* Base: Sec AC timing desc tbl */

#define PHY_REG_DBYTE_GATE_01E		0x9002001eUL /* Half: DBYTE xcvr gate ctrl */
#define PHY_REG_DBYTE_ANALOG_054	0x90020054UL /* Half: DBYTE analog delay tap */
#define PHY_REG_DBYTE_COARSE_STEP_0E4	0x900200e4UL /* Half: DBYTE coarse step reg */
#define PHY_REG_DBYTE_BCAST_GATE_01E	0x9003e01eUL /* Half: DBYTE bcast gate ctrl */
#define PHY_REG_GATE_SETTLE_084		0x90040084UL /* Half: Gate settle timing reg */
#define PHY_REG_GATE_STAT_092		0x90040092UL /* Half: Gate pulse status reg */
#define PHY_REG_CBT_STAT_180		0x90040180UL /* Half: CBT status reg 0x180 */
#define PHY_REG_AC_CTRL_E022		0x900e0022UL /* Half: AC ctrl read source */
#define PHY_REG_AC_BCAST_FE022		0x900fe022UL /* Half: AC bcast write target */

/* Batch 7 DCCM Variables, Tables & PHY APB Registers */
#define PMU_DMEM_ACTIVE_SLICE_CNT_28	0x80000028UL /* Byte: Active DBYTE slice cnt */
#define PMU_DMEM_SLICE_PROFILE_5A	0x8000005aUL /* Base: 29-entry slice profile */
#define PMU_DMEM_CAL_OVERRIDE_CE	0x800000ceUL /* Byte: Eye margin override */
#define PMU_DMEM_EYE_SWEEP_OP17_102	0x80000102UL /* Byte: Eye sweep op17 param */
#define PMU_DMEM_CAL_STRUCT_BASE_9A4	0x800009a4UL /* Base: 54B cal struct base */
#define PMU_DMEM_CH_MASK_9B8		0x800009b8UL /* Byte: Active channel mask */

#define PHY_REG_DBYTE_DELAY_09C		0x9002009cUL /* Half: DBYTE eye sweep delay */
#define PHY_REG_DBYTE_BCAST_VALID_002	0x9003e002UL /* Half: DBYTE bcast win valid */
#define PHY_REG_DBYTE_BCAST_MODE_012	0x9003e012UL /* Half: DBYTE bcast mode reg */
#define PHY_REG_DBYTE_BCAST_VREF_014	0x9003e014UL /* Half: DBYTE bcast Vref ctrl */
#define PHY_REG_PAD_CAL_CTRL_050	0x90040050UL /* Half: Master pad cal ctrl */
#define PHY_REG_ZQ_STAT_094		0x90040094UL /* Half: ZQ telemetry status 0 */
#define PHY_REG_SEC_CTRL_0EE		0x9005e0eeUL /* Half: Sec PUB ctrl 0x0ee */
#define PHY_REG_SEC_CTRL_14A		0x9005e14aUL /* Half: Sec PUB ctrl 0x14a */

/* Batch 8 DCCM Variables, Tables & PHY APB Registers */
#define PMU_DMEM_WCK_PHASE_D2		0x800000d2UL /* Base: WCK align phase codes */
#define PMU_DMEM_TEST_PAT_E4		0x800000e4UL /* Half: Custom CBT test pattern */
#define PMU_DMEM_DESKEW_OFFSET_E6	0x800000e6UL /* Byte: Deskew offset override */
#define PMU_DMEM_RFIFO_BAK_130		0x80000130UL /* Base: 7-halfword RFIFO backup */
#define PMU_DMEM_CAL_SEQ_MAP_480	0x80000480UL /* Base: 21-byte cal seq idx map */
#define PMU_DMEM_LANE_PARAM_TBL_E48	0x80000e48UL /* Base: Per-lane parameter tbl */
#define PMU_DMEM_CBT_COEFF_6098		0x80006098UL /* Base: 10-lane CBT coeff tbl A */
#define PMU_DMEM_CBT_COEFF_6138		0x80006138UL /* Base: 10-lane CBT coeff tbl B */
#define PMU_DMEM_CBT_FLAGS_61D8		0x800061d8UL /* Base: 10-lane CBT byte flags */
#define PMU_DMEM_CBT_DELAY_ARR_E798	0x8000e798UL /* Base: CBT per-slice delay arr */

#define PHY_REG_AC_TIMING_0024		0x90000024UL /* Half: PHY AC timing register */
#define PHY_REG_STROBE_CAL_EN_0148	0x90000148UL /* Half: Strobe cal mode enable */
#define PHY_REG_DBYTE_LCDL_STAT_16E	0x9002016eUL /* Half: DBYTE LCDL gate status */
#define PHY_REG_DBYTE_BCAST_TRIG_176	0x9003e176UL /* Half: DBYTE bcast gate commit */
#define PHY_REG_RFIFO_CFG_110		0x90040110UL /* Half: RFIFO cfg base */
#define PHY_REG_RFIFO_SUPPRESS_116	0x90040116UL /* Half: RFIFO suppress ctrl */
#define PHY_REG_AC_BCAST_PAT_0C0	0x900fe0c0UL /* Half: AC bcast test pattern A */
#define PHY_REG_AC_BCAST_PAT_0CA	0x900fe0caUL /* Half: AC bcast test pattern B */
#define PHY_REG_AC_BCAST_PAT_1AC	0x900fe1acUL /* Half: AC bcast pattern ctrl B */
#define PHY_REG_AC_BCAST_PAT_1AE	0x900fe1aeUL /* Half: AC bcast pattern ctrl */
#define PHY_REG_AC_TIMING_0020		0x90000020UL /* Half: PHY AC coarse timing reg */
#define PHY_REG_DBYTE_DQ_DLY0_160	0x90020160UL /* Half: DBYTE DQ delay 0 reg */
#define PHY_REG_DBYTE_PLL_TIER_1B2	0x900201b2UL /* Half: DBYTE PLL rate tier cfg */
#define PHY_REG_DBYTE_BCAST_CTRL_02C	0x9003e02cUL /* Half: DBYTE bcast ctrl 0x2c */
#define PHY_REG_DBYTE_UPPER_DLY_E20	0x9003fe20UL /* Half: DBYTE upper delay 0 */
#define PHY_REG_DBYTE_UPPER_DLY_E22	0x9003fe22UL /* Half: DBYTE upper delay 1 */
#define PHY_REG_DBYTE_UPPER_DLY_E24	0x9003fe24UL /* Half: DBYTE upper delay 2 */
#define PHY_REG_DBYTE_UPPER_DLY_E26	0x9003fe26UL /* Half: DBYTE upper delay 3 */
#define PHY_REG_CAL_STAT_CTRL_008	0x90040008UL /* Half: PHY cal status ctrl A */
#define PHY_REG_CBT_STAT_182		0x90040182UL /* Half: CBT status reg 0x182 */
#define PHY_REG_CBT_STAT_184		0x90040184UL /* Half: CBT status reg 0x184 */
#define PHY_REG_SHADOW_CNT_21E		0x9018021eUL /* Half: Shadow stream count reg */

#define PMU_DMEM_CH0_DESKEW_COARSE_26	0x80000026UL /* Base: Ch0 coarse deskew diff */
#define PMU_DMEM_CH0_DESKEW_CROSS_2C	0x8000002cUL /* Base: Ch0 cross deskew diff */
#define PMU_DMEM_CH0_DESKEW_FINE_30	0x80000030UL /* Base: Ch0 fine deskew diff */
#define PMU_DMEM_PLL_GATE_EA		0x800000eaUL /* Half: Ch1 shadow base */
#define PMU_DMEM_SHADOW_BASE_108	0x80000108UL /* Half: Initial shadow idx */
#define PMU_DMEM_SHADOW_DIFF_10A	0x8000010aUL /* Half: Phase shadow count */
#define PMU_DMEM_SHADOW_BASE_10C	0x8000010cUL /* Half: Midpoint shadow idx */
#define PMU_DMEM_SHADOW_DIFF_10E	0x8000010eUL /* Half: Final shadow count */
#define PMU_DMEM_LANE_PARAM_TBL_E4A	0x80000e4aUL /* Base: Per-lane signed step tbl */
#define PMU_DMEM_CAL_TAP_STAT_378B	0x8000378bUL /* Base: Calibrated tap status tbl */
#define PMU_DMEM_CBT_FLAGS_6228		0x80006228UL /* Base: 10-lane CBT base tbl */
#define PMU_DMEM_CBT_FLAGS_6278		0x80006278UL /* Base: 10-lane CBT init stat tbl */
#define PMU_DMEM_CBT_FLAGS_62C8		0x800062c8UL /* Base: 10-lane CBT track stat tbl */
#define PMU_DMEM_WCK_TAP_C0R0_D1	0x00d1U
#define PMU_DMEM_WCK_TAP_C1R0_D6	0x00d6U
#define PMU_DMEM_WCK_TAP_C0R1_DB	0x00dbU
#define PMU_DMEM_WCK_TAP_C1R1_E0	0x00e0U
#define PMU_DMEM_DESKEW_SWAP_B72	0x0b72U
#define PMU_DMEM_DESKEW_NORM_B76	0x0b76U
#define PMU_DMEM_BEST_TAP_ARR_B7A	0x0b7aU
#define PMU_DMEM_SWEEP_BLINK_OFF	0x6594U
#define PMU_DMEM_CBT_DELAY_ARR_OFF	0xe798U

/* Batch 9 DCCM Variables, Tables & PHY APB Registers */
#define PMU_DMEM_CHANNEL_CFG		0x0004
#define PMU_DMEM_DRAM_TYPE_FLAGS	0x0009
#define PMU_DMEM_CAL_MISC_FLAGS		0x000d
#define PMU_DMEM_CAL_CTRL_0F		0x000f
#define PMU_DMEM_TRAIN_MODE_HI		0x000f
#define PMU_DMEM_SEQUENCE_CTRL		0x0010
#define PMU_DMEM_VREF_DAC_CTRL		0x0072
#define PMU_DMEM_TRAIN_MODE		0x0014
#define PMU_DMEM_CTRL_17		0x0017
#define PMU_DMEM_CTRL_18		0x0018
#define PMU_DMEM_CAL_DISABLE_19		0x0019
#define PMU_DMEM_CAL_MODE_1B		0x001b
#define PMU_DMEM_CH0_CTRL_20		0x0020
#define PMU_DMEM_CH0_CTRL_21		0x0021
#define PMU_DMEM_CH0_RANK_EN		0x0025
#define PMU_DMEM_CH0_RANK_CFG		0x0032
#define PMU_DMEM_SEARCH_WIN_COARSE	0x0037
#define PMU_DMEM_SEARCH_WIN_FINE	0x0039
#define PMU_DMEM_CH1_RANK_EN		0x0040
#define PMU_DMEM_CH1_RANK_CFG		0x004d
#define PMU_DMEM_SLICE_PROFILE_TABLE	0x005aUL
#define PMU_DMEM_VREF_FLAG_SIGNED	0x0062
#define PMU_DMEM_TRAIN_PARAM_CTRL	0x0096
#define PMU_DMEM_CAL_OVERRIDE_FLAG	0x00ce
#define PMU_DMEM_DESKEW_OFFSET_OVERRIDE	0x00e6
#define PMU_DMEM_TRAIN_STATUS		0x00e7
#define PMU_DMEM_DQ_SWAP_BASE		0x0100
#define PMU_DMEM_TRAIN_FEATURE_MASK	0x0102
#define PMU_DMEM_DIFF_TAP_PAIR_105	0x0105
#define PMU_DMEM_CAL_STEP_106		0x0106
#define PMU_DMEM_SEARCH_WINDOW_STEPS	0x0b64
#define PMU_DMEM_SEARCH_WINDOW_SCALE	0x0b65
#define PMU_DMEM_SEARCH_WINDOW_BIAS_LO	0x0110
#define PMU_DMEM_SEARCH_WINDOW_BIAS_HI	0x0111
#define PMU_DMEM_DELAY_TAP_SUB1_E7C4	0x8000e7c4UL
#define PMU_DMEM_ALT_TRACKER_D70	0x80000d70UL

/* Slice calibration descriptor table geometry & MR byte offsets (0x800009a4) */
#define PMU_SLICE_DESC_SIZE		54U
#define PMU_SLICE_DESC_COUNT		4U
#define PMU_SLICE_DESC_TOTAL_BYTES	(PMU_SLICE_DESC_COUNT * PMU_SLICE_DESC_SIZE)
#define PMU_SLICE_DESC_MR1_OFF		0x01U
#define PMU_SLICE_DESC_MR2_OFF		0x02U
#define PMU_SLICE_DESC_MR3_OFF		0x03U
#define PMU_SLICE_DESC_MR16_OFF		0x10U
#define PMU_SLICE_DESC_MR41_OFF		0x29U

/* Calibration step tracker geometry (0x80000d9c) */
#define PMU_TRACKER_SLOTS_BYTES		32U

/* PHY DBYTE slice address stride and additional PUB/AC CSRs */
#define PHY_DBYTE_SLICE_STRIDE		0x2000UL
#define PHY_REG_DBYTE_DELAY_CNT_028	0x90020028UL
#define PHY_REG_DBYTE_BCAST_STROBE_080	0x9003e080UL
#define PHY_REG_DBYTE_BCAST_CBT_144	0x9003e144UL
#define PHY_REG_DBYTE_BCAST_EYE_158	0x9003e158UL
#define PHY_REG_DBYTE_BCAST_SAMPLE_16A	0x9003e16aUL
#define PHY_REG_DBYTE_BCAST_LANE_17C	0x9003e17cUL
#define PHY_REG_DBYTE_BCAST_WCK_1B2	0x9003e1b2UL
#define PHY_REG_DBYTE_UPPER_COARSE_F54	0x9003ff54UL
#define PHY_REG_DBYTE_UPPER_BCAST_F7C	0x9003ff7cUL
#define PHY_REG_MASTER_CAL_CTRL_0DC	0x900400dcUL
#define PHY_REG_CH0_RANK_CFG_100	0x90040100UL
#define PHY_REG_CH1_RANK_CFG_102	0x90040102UL
#define PHY_REG_AC_PLL_CTRL_1C0		0x900401c0UL
#define PHY_REG_CLK_PHASE_BASE_600	0x90040600UL
#define PHY_REG_CLK_PHASE_DELAY_628	0x90040628UL
#define PHY_REG_SEC_CAL_CTRL_0B2	0x9005e0b2UL
#define PHY_REG_AC_SLICE_CFG_05C	0x9006005cUL
#define PHY_REG_AC_DRIVE_CFG_05C	0x9007e05cUL
#define PHY_REG_AC_BCAST_GATE_108	0x9007e108UL
#define PHY_REG_AC_PAT_CTRL_0C0		0x900e00c0UL
#define PHY_REG_BIST_PAT_CTRL_E4C	0x900ffe4cUL
#define PHY_REG_BIST_PAT_CTRL_E4E	0x900ffe4eUL
#define PHY_REG_CAL_LATCH_STREAM_1000	0x90121000UL
#define PHY_REG_CAL_RESTORE_TRIG_050	0x9013e050UL
#define PHY_REG_PUB_CFG_100		0x90180100UL

/* PMU firmware version and build identifiers */
#define PMU_FW_VERSION_ID		0x230bU
#define PMU_FW_BUILD_ID_LO		0x715dU
#define PMU_FW_BUILD_ID_HI		0x00ceU
#define PMU_DMEM_HIGH_FREQ_FLAG		0x0401
#define PMU_DMEM_CAL_ACTIVE_FLAG	0x0402
#define PMU_DMEM_FREQ_MODE		0x0403
#define PMU_DMEM_METRIC_TABLE_ENTRIES	0x0409
#define PMU_DMEM_ACTIVE_CSR_OFFSET	0x041c
#define PMU_DMEM_METRIC_COEFF_X		0x0420
#define PMU_DMEM_METRIC_COEFF_Y		0x0422
#define PMU_DMEM_ITER_COUNT		0x0428
#define PMU_DMEM_2D_STEP_CTRL		0x0438
#define PMU_DMEM_2D_STEP_PARAM_43A	0x043a
#define PMU_DMEM_CAL_STRIDE		0x047c
#define PMU_DMEM_DESKEW_TARGET_DELAY	0x047e
#define PMU_DMEM_CAL_STRUCT_BASE	0x09a4UL
#define PMU_DMEM_AC_STEP_FLAG		0x0a7c
#define PMU_DMEM_RANK_SLICE_COUNTS	0x0b20
#define PMU_DMEM_RANK_SLICE_MAP		0x0b44
#define PMU_DMEM_CAL_BYTE		0x0b67
#define PMU_DMEM_SLICE_START		0x0b68
#define PMU_DMEM_SLICE_END		0x0b69
#define PMU_DMEM_RANK0_SLICE_START	0x0b6a
#define PMU_DMEM_RANK_BOUNDARY		0x0b6b
#define PMU_DMEM_RANK1_SLICE_START	0x0b6c
#define PMU_DMEM_RANK1_SLICE_END	0x0b6d
#define PMU_DMEM_BIST_PATTERN_LO	0x00e4
#define PMU_DMEM_BIST_PATTERN_HI	0x00e5
#define PMU_DMEM_DELAY_CAL_RESULTS	0x0b70
#define PMU_DMEM_LANE_ACTIVE_FLAGS	0x0b96
#define PMU_DMEM_ACTIVE_SLICE_MASK_CAL	0x0b98
#define PMU_DMEM_EYE_SAMPLE_BUF		0x0dc8
#define PMU_DMEM_SWEEP_SAMPLE_BUF	0x6318

#define PHY_REG_PUB_BASE		0x90000000UL
#define PHY_REG_DBYTE_BCAST_BASE	0x9003e000UL
#define PHY_REG_DBYTE_BCAST_SAMPLE	0x9003e000UL
#define PHY_REG_DBYTE_BCAST_PULSE	0x9003e002UL
#define PHY_REG_DBYTE_BCAST_GATE	0x9003e00aUL
#define PHY_REG_DBYTE_BCAST_MODE	0x9003e012UL
#define PHY_REG_DBYTE_BCAST_RX_EN	0x9003e14eUL
#define PHY_REG_DBYTE_BCAST_DQ_DLY0	0x9003e160UL
#define PHY_REG_DBYTE_BCAST_DQ_DLY1	0x9003e162UL
#define PHY_REG_DBYTE_BCAST_DQ_DLY2	0x9003e166UL
#define PHY_REG_DBYTE_BCAST_DQ_DLY3	0x9003e168UL
#define PHY_REG_DBYTE_BCAST_DQS_DLY0	0x9003e172UL
#define PHY_REG_DBYTE_BCAST_DQS_DLY1	0x9003e174UL
#define PHY_REG_MASTER_BASE		0x90040000UL
#define PHY_REG_VREF_TRIM_CFG		0x90040054UL
#define PHY_REG_VREF_STAT		0x90040084UL
#define PHY_REG_DRAM_TYPE_CFG		0x90040088UL
#define PHY_REG_RESET_TRIGGER		0x900400f2UL
#define PHY_REG_PUB_SEC_BASE		0x9005e000UL
#define PHY_REG_AC_BASE			0x90060000UL
#define PHY_REG_BIST_PLL_CFG		0x900e0022UL
#define PHY_REG_BIST_CTRL_MODE		0x900fe0c0UL
#define PMU_DMEM_LANE_SWEEP_CTRL	0x0059U

/* PMU fatal assertion error codes passed to pmu_assert_or_halt() */
#define PMU_ERR_PLL_LOCK_TIMEOUT	0x00240000UL
#define PMU_ERR_ZQ_CAL_LOW_FREQ		0x00430000UL
#define PMU_ERR_ZQ_CAL_HIGH_FREQ	0x00470000UL
#define PMU_ERR_EYE_RANK_LIMIT_1	0x00920001UL
#define PMU_ERR_EYE_RANK_MODE_INVALID	0x00930001UL
#define PMU_ERR_RANK_BOUNDS_BASE	0x009d0003UL
#define PMU_ERR_EYE_STEP_ALIGN_MISMATCH	0x00a40003UL
#define PMU_ERR_DESKEW_SAMPLE_CHECK	0x01160000UL
#define PMU_ERR_RX_FIFO_SCAN_OVERFLOW	0x011c0000UL
#define PMU_ERR_LPCA_CLK_GATE_PRECOND	0x01280000UL
#define PMU_ERR_AC_STEP_COORD_ADJUST	0x019f0003UL
#define PMU_ERR_BIST_PATTERN_IDX_INVALID 0x01a40001UL
#define PMU_ERR_BIST_VALIDATION_MODE	0x01a50000UL
#define PMU_ERR_BIST_CMD_MODE_INVALID	0x01a80001UL
#define PMU_ERR_EYE_SLICE_REG_SOURCE	0x01ab0000UL
#define PMU_ERR_EYE_WINDOW_TOO_NARROW	0x01b10006UL
#define PMU_ERR_EYE_SWEEP_CONVERGENCE	0x01f60000UL
#define PMU_ERR_CMD_SUBCODE_INVALID	0x02190001UL
#define PMU_ERR_FW_BUILD_ID_MISMATCH	0x02240000UL
#define PMU_ERR_WCK_EYE_ZERO_WIDTH	0x022e0003UL
#define PMU_ERR_VREF_STEP_DIR_INVALID	0x02350000UL
#define PMU_ERR_BIST_PATTERN_MODE_RANGE	0x027e0001UL
#define PMU_ERR_COARSE_DELAY_OVERFLOW	0x02a80000UL

#endif /* _DWC_DDRPHY_PMU_REGS_H */

