/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2026 Alastair D'Silva <alastair@d-silva.org>
 *
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

/* ========================================================================== */
/* ARC PMU DCCM Addresses (0x80000000 Base)                                   */
/* ========================================================================== */

#define PMU_DMEM_BASE			0x80000000UL
#define PMU_DMEM_CLK_GATE_FLAG		0x80000001UL /* Byte: Clock gate / broadcast flag */
#define PMU_DMEM_PSTATE_FLAGS		0x80000004UL /* Byte: PState / channel status flags */
#define PMU_DMEM_CAL_MODE_FLAG_ADDR	0x80000005UL /* Byte: Calibration parameter */
#define PMU_DMEM_DRAM_FREQ_MHZ		0x80000006UL /* Half: Operating DRAM data rate (MT/s) */
#define PMU_DMEM_PLL_RATIO		0x80000008UL /* Byte: WCK:CK ratio (4 = 4:1, 2 = 2:1) */
#define PMU_DMEM_ASSERT_CODE		0x8000000cUL /* Word: Fatal assertion / panic code */
#define PMU_DMEM_SIGNED_STEP		0x8000000eUL /* SByte: Signed AC timing step */
#define PMU_DMEM_SEQ_CTRL		0x80000010UL /* Half: Training sequence control mask */
#define PMU_DMEM_POST_THRESH		0x80000012UL /* Byte: Post mailbox threshold */
#define PMU_DMEM_TRAIN_MODE_ADDR	0x80000014UL /* Half: Active training stage mode */
#define PMU_DMEM_ANALOG_CFG		0x80000015UL /* Byte: Analog cfg flag */
#define PMU_DMEM_RX_CLK_EN		0x80000019UL /* Byte: Receiver clock enable flag */
#define PMU_DMEM_CAL_STEP_CONFIG	0x8000001aUL /* Half: Calibration step configuration */
#define PMU_DMEM_DESKEW_LIM		0x8000001cUL /* Byte: Dual-rank deskew cfg */
#define PMU_DMEM_CH0_DQ_WIDTH		0x80000024UL /* Byte: Total DQ bit width Ch0 */
#define PMU_DMEM_CH0_ANALOG_EN		0x80000025UL /* Byte: Ch0 analog power enable */
#define PMU_DMEM_CH0_DESKEW_COARSE	0x80000026UL /* Base: Ch0 coarse deskew diff */
#define PMU_DMEM_ACTIVE_SLICE_CNT	0x80000028UL /* Byte: Active DBYTE slice cnt */
#define PMU_DMEM_CH0_DESKEW_CROSS	0x8000002cUL /* Base: Ch0 cross deskew diff */
#define PMU_DMEM_CH0_DESKEW_FINE	0x80000030UL /* Base: Ch0 fine deskew diff */
#define PMU_DMEM_CH1_DQ_WIDTH		0x8000003fUL /* Byte: Total DQ bit width Ch1 */
#define PMU_DMEM_CH1_ACTIVE_FLAG	0x80000040UL /* Byte: Ch1 active enable flag */
#define PMU_DMEM_SLICE_PROFILE		0x8000005aUL /* Base: 29-entry slice profile */
#define PMU_DMEM_MR3_ADDR		0x80000062UL /* Byte: Signed Vref training mode flags */
#define PMU_DMEM_MR13_ADDR		0x80000072UL /* Byte: Vref DAC control configuration */
#define PMU_DMEM_MR20_ADDR		0x8000008eUL /* Byte: JEDEC MR20 RDQS mode byte */
#define PMU_DMEM_MR22_ADDR		0x80000096UL /* Byte: DRAM training config byte */
#define PMU_DMEM_CAL_OVERRIDE		0x800000ceUL /* Byte: Eye margin override */
#define PMU_DMEM_WCK_PHASE		0x800000d2UL /* Base: WCK align phase codes */
#define PMU_DMEM_TEST_PAT		0x800000e4UL /* Half: Custom CBT test pattern */
#define PMU_DMEM_DESKEW_OFFSET		0x800000e6UL /* Byte: Deskew offset override */
#define PMU_DMEM_VREF_OVR_CTRL_ADDR	0x800000e7UL /* Byte: Vref DAC override control flags */
#define PMU_DMEM_PLL_GATE_CH0_ADDR	0x800000e8UL /* Base: Per-rank shadow base */
#define PMU_DMEM_PLL_GATE_CH1_ADDR	0x800000eaUL /* Half: Ch1 shadow base */
#define PMU_DMEM_PLL_SEQ_EN_ADDR	0x800000f2UL /* Half: PLL param seq enable */
#define PMU_DMEM_AC_PROFILE_EN_ADDR	0x800000f4UL /* Byte: AC slice profile en */
#define PMU_DMEM_LANE_ERR_MASK		0x80000101UL /* Byte: Per-lane deskew error status mask */
#define PMU_DMEM_TRAIN_FEAT_MASK_ADDR	0x80000102UL /* Byte: Eye sweep op17 param */
#define PMU_DMEM_CA_VREF_RANK1_ADDR	0x80000103UL /* Byte: CA Vref state Rank 1 */
#define PMU_DMEM_CA_VREF_RANK0_ADDR	0x80000104UL /* Byte: CA Vref state Rank 0 */
#define PMU_DMEM_SHADOW_INIT_IDX_ADDR	0x80000108UL /* Half: Initial shadow idx */
#define PMU_DMEM_SHADOW_PHASE_CNT_ADDR	0x8000010aUL /* Half: Phase shadow count */
#define PMU_DMEM_SHADOW_MID_IDX_ADDR	0x8000010cUL /* Half: Midpoint shadow idx */
#define PMU_DMEM_SHADOW_FINAL_CNT_ADDR	0x8000010eUL /* Half: Final shadow count */
#define PMU_DMEM_GATE_DESC		0x80000118UL /* Base: Gate eye margin desc */
#define PMU_DMEM_RFIFO_BAK		0x80000130UL /* Base: 7-halfword RFIFO backup */
#define PMU_DMEM_CBT_LANE_LIM		0x80000403UL /* Byte: CBT strobe/lane limit */
#define PMU_DMEM_MARGIN_STAT		0x80000405UL /* Byte: Stage 10 margin status byte */
#define PMU_DMEM_STAGE_RANK_IDX_ADDR	0x80000406UL /* Byte: Stage active rank idx */
#define PMU_DMEM_STAGE_SLICE_IDX_ADDR	0x80000407UL /* Byte: Stage active slice idx */
#define PMU_DMEM_STAGE_LANE_IDX_ADDR	0x80000408UL /* Byte: Stage active lane idx */
#define PMU_DMEM_COST_WEIGHT_2		0x80000410UL /* Half: Eye overflow weight */
#define PMU_DMEM_SHADOW_TRACE_PTR	0x80000418UL /* Word: Shadow trace buffer ptr */
#define PMU_DMEM_STEP_CFG_RAW		0x8000001aUL /* Half: Raw step config register */
#define PMU_DMEM_ACTIVE_CH_OFFSET	0x8000041cUL /* Word: Active channel CSR offset */
#define PMU_DMEM_COST_WEIGHT_1		0x80000422UL /* Half: Eye deviation weight */
#define PMU_DMEM_CAL_SAVE_CFG_ADDR	0x8000042aUL /* Half: Saved cal status word */
#define PMU_DMEM_DBYTE_DLY_SAVE		0x80000430UL /* Base: Saved DBYTE delay array */
#define PMU_DMEM_CAL_TABLE		0x8000043cUL /* Half: 2D calibration parameter table */
#define PMU_DMEM_AC_PROFILE_DLY_ADDR	0x80000444UL /* Base: AC profile step cfg A */
#define PMU_DMEM_AC_PROFILE_CFG_ADDR	0x80000448UL /* Base: AC profile step cfg B */
#define PMU_DMEM_CBT_STATE		0x8000045aUL /* Byte: CBT state latch */
#define PMU_DMEM_CBT_CONFIG		0x8000046aUL /* Byte: CBT configuration value */
#define PMU_DMEM_CAL_CMD_TABLE		0x8000046cUL /* Base: Shadow cal cmd tbl */
#define PMU_DMEM_NORM_STEP_STRIDE	0x8000047cUL /* Half: Normalized step stride */
#define PMU_DMEM_CAL_SEQ_MAP		0x80000480UL /* Base: 21-byte cal seq idx map */
#define PMU_DMEM_CAL_STRUCT_ADDR	0x800009a4UL /* Base: 54B cal struct base */
#define PMU_DMEM_CH_MASK		0x800009b8UL /* Byte: Active channel mask */
#define PMU_DMEM_AC_CAL_ACTIVE		0x80000a7cUL /* Byte: AC calibration active flag */
#define PMU_DMEM_AC_SIGNED_STEP		0x80000a7dUL /* Byte: AC signed step offset */
#define PMU_DMEM_DQ_PIN_MAP_BASE	0x80000a7eUL /* Base: DBYTE DQ pin swizzle table */
#define PMU_DMEM_SLICE_CNT_LIMIT	0x80000b20UL /* Base: Slice cnt limit array */
#define PMU_DMEM_SLICE_IMP_CFG		0x80000b24UL /* Base: Sec rank slice map */
#define PMU_DMEM_TIMING_LIM		0x80000b64UL /* Byte: Gate timing limit */
#define PMU_DMEM_ACTIVE_RANK		0x80000b67UL /* Byte: Active rank / byte index */
#define PMU_DMEM_CH_RANGE		0x80000b68UL /* Base: Active ch [min, max] */
#define PMU_DMEM_CH_ACTIVE_RANGE	0x80000b6aUL /* Base: Primary ch [min, max] */
#define PMU_DMEM_DESKEW_DIR		0x80000b6bUL /* Byte: Active deskew sweep direction */
#define PMU_DMEM_CH_SEC_RANGE		0x80000b6cUL /* Base: Secondary ch [min, max] */
#define PMU_DMEM_DESKEW_STAT_TBL	0x80000b6eUL /* Base: Deskew lane stat tbl */
#define PMU_DMEM_ARCHIVED_CAL_STAT_ADDR	0x80000b7eUL /* Half: Archived cal status */
#define PMU_DMEM_EYE_SWEEP_STAT		0x80000b82UL /* Base: Eye sweep status arr */
#define PMU_DMEM_LANE_STATUS		0x80000b97UL /* Byte: Latched slice lane status */
#define PMU_DMEM_ACTIVE_SLICE_MASK	0x80000b98UL /* Byte: Active slice bitmask */
#define PMU_DMEM_DBYTE_BACKUP		0x80000ba0UL /* Base: DBYTE delay register backup buffer */
#define PMU_DMEM_EYE_BACKUP		0x80000bc0UL /* Base: 2D eye table backup buffer */
#define PMU_DMEM_ALT_TRACKER		0x80000d70UL
#define PMU_DMEM_CAL_TRACKER		0x80000d9cUL /* Base: Calibration marker scratchpad state */
#define PMU_DMEM_EYE_CLEAR_BUF		0x80000dacUL /* Base: Eye scratchpad buffer */
#define PMU_DMEM_AC_TIMING_TBL		0x80000dc8UL /* Base: Primary AC timing tbl */
#define PMU_DMEM_LANE_PARAM_TBL_ADDR	0x80000e48UL /* Base: Per-lane parameter tbl */
#define PMU_DMEM_LANE_STEP_TBL_ADDR	0x80000e4aUL /* Base: Per-lane signed step tbl */
#define PMU_DMEM_AC_TIMING_DESC		0x80003708UL /* Base: Sec AC timing desc tbl */
#define PMU_DMEM_CAL_TAP_STAT		0x8000378bUL /* Base: Calibrated tap status tbl */
#define PMU_DMEM_CBT_COEFF_A_ADDR	0x80006098UL /* Base: 10-lane CBT coeff tbl A */
#define PMU_DMEM_CBT_COEFF_B_ADDR	0x80006138UL /* Base: 10-lane CBT coeff tbl B */
#define PMU_DMEM_CBT_LANE_FLAGS_ADDR	0x800061d8UL /* Base: 10-lane CBT byte flags */
#define PMU_DMEM_CBT_BASE_TBL_ADDR	0x80006228UL /* Base: 10-lane CBT base tbl */
#define PMU_DMEM_CBT_INIT_STAT_ADDR	0x80006278UL /* Base: 10-lane CBT init stat tbl */
#define PMU_DMEM_CBT_TRACK_STAT_ADDR	0x800062c8UL /* Base: 10-lane CBT track stat tbl */
#define PMU_DMEM_RX_SCAN_MATRIX		0x80008000UL /* Base: 3.5 KiB RX gate scan scratch */
#define PMU_DMEM_CBT_DELAY_ARR		0x8000e798UL /* Base: CBT per-slice delay arr */
#define PMU_DMEM_DELAY_TAP_LUT		0x8000e7b8UL /* Base: Pad delay tap LUT */
#define PMU_DMEM_DELAY_TAP_SUB1		0x8000e7c4UL

/* ========================================================================== */
/* ARC PMU DCCM Relative Byte Offsets                                         */
/* ========================================================================== */

#define PMU_DMEM_DRAM_FREQ_OFF		0x06 /* Half: Clock in MHz offset */
#define PMU_DMEM_DRAM_TYPE_FLAGS	0x0009
#define PMU_DMEM_DRAM_CFG_FLAGS		0x0a /* Half: DRAM config bitmask */
#define PMU_DMEM_CAL_SELECT_FLAGS	0x0b /* Byte: Calibration selector flags */
#define PMU_DMEM_CAL_MISC_FLAGS		0x000d
#define PMU_DMEM_TRAIN_MODE_HI		0x000f
#define PMU_DMEM_SEQUENCE_CTRL		0x0010
#define PMU_DMEM_TRAIN_MODE		0x0014
#define PMU_DMEM_CAL_CTRL_HI		0x0017
#define PMU_DMEM_CAL_MODE_EXT		0x0018
#define PMU_DMEM_CAL_DISABLE		0x0019
#define PMU_DMEM_CAL_MODE		0x001b
#define PMU_DMEM_LANE_MASK		0x1c /* Byte: Lane active mask */
#define PMU_DMEM_CH0_CTRL_LO		0x0020
#define PMU_DMEM_CH0_CTRL_HI		0x0021
#define PMU_DMEM_CH0_RANK_EN		0x0025
#define PMU_DMEM_CH0_RANK_CFG		0x0032
#define PMU_DMEM_SEARCH_WIN_COARSE	0x0037
#define PMU_DMEM_SEARCH_WIN_FINE	0x0039
#define PMU_DMEM_CH1_RANK_EN		0x0040
#define PMU_DMEM_CH1_RANK_CFG		0x004d
#define PMU_DMEM_LANE_SWEEP_CTRL	0x0059U
#define PMU_DMEM_SLICE_PROFILE_TABLE	0x005aUL
#define PMU_DMEM_VREF_FLAG_SIGNED	0x0062
#define PMU_DMEM_VREF_DAC_CTRL		0x0072
#define PMU_DMEM_MR19_OFF		0x8a /* Byte: DRAM protocol status offset */
#define PMU_DMEM_TRAIN_PARAM_CTRL	0x0096
#define PMU_DMEM_CAL_OVERRIDE_FLAG	0x00ce
#define PMU_DMEM_WCK_TAP_C0R0		0x00d1U
#define PMU_DMEM_WCK_TAP_C1R0		0x00d6U
#define PMU_DMEM_WCK_TAP_C0R1		0x00dbU
#define PMU_DMEM_WCK_TAP_C1R1		0x00e0U
#define PMU_DMEM_BIST_PATTERN_LO	0x00e4
#define PMU_DMEM_BIST_PATTERN_HI	0x00e5
#define PMU_DMEM_DESKEW_OFFSET_OVERRIDE	0x00e6
#define PMU_DMEM_TRAIN_STATUS		0x00e7
#define PMU_DMEM_DQ_SWAP_BASE		0x0100
#define PMU_DMEM_TRAIN_FEATURE_MASK	0x0102
#define PMU_DMEM_DIFF_TAP_PAIR		0x0105
#define PMU_DMEM_CAL_STEP		0x0106
#define PMU_DMEM_SEARCH_WINDOW_BIAS_LO	0x0110
#define PMU_DMEM_SEARCH_WINDOW_BIAS_HI	0x0111
#define PMU_DMEM_HIGH_FREQ_FLAG		0x0401
#define PMU_DMEM_CAL_ACTIVE_FLAG	0x0402
#define PMU_DMEM_FREQ_MODE		0x0403
#define PMU_DMEM_METRIC_TABLE_ENTRIES	0x0409
#define PMU_DMEM_ACTIVE_CSR_OFFSET	0x041c
#define PMU_DMEM_METRIC_COEFF_X		0x0420
#define PMU_DMEM_METRIC_COEFF_Y		0x0422
#define PMU_DMEM_ITER_COUNT		0x0428
#define PMU_DMEM_2D_STEP_CTRL		0x0438
#define PMU_DMEM_2D_STEP_PARAM		0x043a
#define PMU_DMEM_CAL_STRIDE		0x047c
#define PMU_DMEM_DESKEW_TARGET_DELAY	0x047e
#define PMU_DMEM_CAL_STRUCT_BASE	0x09a4UL
#define PMU_DMEM_AC_STEP_FLAG		0x0a7c
#define PMU_DMEM_RANK_SLICE_COUNTS	0x0b20
#define PMU_DMEM_RANK_SLICE_MAP		0x0b44
#define PMU_DMEM_SEARCH_WINDOW_STEPS	0x0b64
#define PMU_DMEM_SEARCH_WINDOW_SCALE	0x0b65
#define PMU_DMEM_CAL_BYTE		0x0b67
#define PMU_DMEM_SLICE_START		0x0b68
#define PMU_DMEM_SLICE_END		0x0b69
#define PMU_DMEM_RANK0_SLICE_START	0x0b6a
#define PMU_DMEM_RANK_BOUNDARY		0x0b6b
#define PMU_DMEM_RANK1_SLICE_START	0x0b6c
#define PMU_DMEM_RANK1_SLICE_END	0x0b6d
#define PMU_DMEM_DELAY_CAL_RESULTS	0x0b70
#define PMU_DMEM_DESKEW_SWAP		0x0b72U
#define PMU_DMEM_DESKEW_NORM		0x0b76U
#define PMU_DMEM_BEST_TAP_ARR		0x0b7aU
#define PMU_DMEM_LANE_ACTIVE_FLAGS	0x0b96
#define PMU_DMEM_ACTIVE_SLICE_MASK_CAL	0x0b98
#define PMU_DMEM_EYE_SAMPLE_BUF		0x0dc8
#define PMU_DMEM_SWEEP_SAMPLE_BUF	0x6318
#define PMU_DMEM_SWEEP_BLINK_OFF	0x6594U
#define PMU_DMEM_CBT_DELAY_ARR_OFF	0xe798U

/* ========================================================================== */
/* Calibration Descriptor & Step Tracker Geometry                             */
/* ========================================================================== */

#define PMU_SLICE_DESC_SIZE		54U
#define PMU_SLICE_DESC_COUNT		4U
#define PMU_SLICE_DESC_TOTAL_BYTES	(PMU_SLICE_DESC_COUNT * PMU_SLICE_DESC_SIZE)
#define PMU_SLICE_DESC_MR1_OFF		0x01U
#define PMU_SLICE_DESC_MR2_OFF		0x02U
#define PMU_SLICE_DESC_MR3_OFF		0x03U
#define PMU_SLICE_DESC_MR16_OFF		0x10U
#define PMU_SLICE_DESC_MR41_OFF		0x29U
#define PMU_TRACKER_SLOTS_BYTES		32U

/* ========================================================================== */
/* Synopsys DDR PHY Internal APB CSRs (0x90000000 Base)                       */
/* ========================================================================== */

#define PHY_DBYTE_SLICE_STRIDE		0x2000UL
#define PHY_REG_PUB_BASE		0x90000000UL /* Half: PHY APB base aperture */
#define PHY_REG_AC_COARSE_TIMING	0x90000020UL /* Half: PHY AC coarse timing reg */
#define PHY_REG_AC_FINE_TIMING		0x90000024UL /* Half: PHY AC timing register */
#define PHY_REG_STROBE_CAL_EN		0x90000148UL /* Half: Strobe cal mode enable */
#define PHY_REG_STREAM_ALT_BASE		0x9001e000UL /* Half: Alt APB stream base */
#define PHY_REG_DBYTE_BASE		0x90020000UL /* Base address for DBYTE slice CSRs */
#define PHY_REG_OP_MODE			0x90020012UL /* Channel-indexed PHY operating mode */
#define PHY_REG_DBYTE_XCVR_GATE		0x9002001eUL /* Half: DBYTE xcvr gate ctrl */
#define PHY_REG_DBYTE_DELAY_CNT		0x90020028UL
#define PHY_REG_DBYTE_ANALOG		0x90020054UL /* Half: DBYTE analog delay tap */
#define PHY_REG_DBYTE_EYE_DELAY		0x9002009cUL /* Half: DBYTE eye sweep delay */
#define PHY_REG_DBYTE_COARSE_STEP	0x900200e4UL /* Half: DBYTE coarse step reg */
#define PHY_REG_DBYTE_DQ_SWIZZLE_BASE	0x90020100UL /* Half: DBYTE Slice 0 delay register base */
#define PHY_REG_DBYTE_GATE_STAT		0x9002011aUL /* Half: DBYTE sample status */
#define PHY_REG_DBYTE_WCK_DLY		0x90020146UL /* Half: DBYTE WCK delay reg */
#define PHY_REG_DBYTE_COARSE		0x90020154UL /* Half: DBYTE coarse gate delay */
#define PHY_REG_DBYTE_EYE_CTRL		0x90020158UL /* Half: DBYTE eye sweep ctrl */
#define PHY_REG_DBYTE_DQ_DLY0		0x90020160UL /* Half: DBYTE DQ delay 0 reg */
#define PHY_REG_DBYTE_SAMPLE_RDY	0x9002016aUL /* Half: DBYTE sample ready reg */
#define PHY_REG_DBYTE_LCDL_STAT		0x9002016eUL /* Half: DBYTE LCDL gate status */
#define PHY_REG_DBYTE_CAL_TRIG		0x90020176UL /* DBYTE calibration strobe sequence */
#define PHY_REG_DBYTE_PERIOD		0x900201aaUL /* Half: DBYTE period meas reg */
#define PHY_REG_DBYTE_PLL_TIER		0x900201b2UL /* Half: DBYTE PLL rate tier cfg */
#define PHY_REG_DBYTE_PLL_LOCK		0x900201c0UL /* Half: DBYTE PLL lock status */
#define PHY_REG_DBYTE_UPPER_GATE	0x90021f6aUL /* Half: DBYTE gate control reg */
#define PHY_REG_DBYTE_BCAST_BASE	0x9003e000UL
#define PHY_REG_DBYTE_BCAST_PULSE	0x9003e002UL /* Half: DBYTE bcast win valid */
#define PHY_REG_DBYTE_BCAST_GATE	0x9003e00aUL /* Half: DBYTE bcast pulse ctrl */
#define PHY_REG_DBYTE_BCAST_MODE	0x9003e012UL /* Half: DBYTE bcast mode reg */
#define PHY_REG_DBYTE_BCAST_VREF	0x9003e014UL /* Half: DBYTE bcast Vref ctrl */
#define PHY_REG_DBYTE_BCAST_WIDTH	0x9003e016UL /* Half: DBYTE bcast pulse width */
#define PHY_REG_DBYTE_BCAST_XCVR_GATE	0x9003e01eUL /* Half: DBYTE bcast gate ctrl */
#define PHY_REG_DBYTE_BCAST_TX_CTRL	0x9003e02cUL /* Half: DBYTE bcast ctrl */
#define PHY_REG_DBYTE_BCAST_PWR_GATE	0x9003e034UL /* Half: DBYTE bcast power gate */
#define PHY_REG_DBYTE_BCAST_STROBE	0x9003e080UL
#define PHY_REG_DBYTE_BCAST_CBT		0x9003e144UL
#define PHY_REG_DBYTE_BCAST_RX_EN	0x9003e14eUL
#define PHY_REG_DBYTE_BCAST_EYE		0x9003e158UL
#define PHY_REG_DBYTE_BCAST_DQ_DLY0	0x9003e160UL
#define PHY_REG_DBYTE_BCAST_DQ_DLY1	0x9003e162UL
#define PHY_REG_DBYTE_BCAST_DQ_DLY2	0x9003e166UL
#define PHY_REG_DBYTE_BCAST_DQ_DLY3	0x9003e168UL
#define PHY_REG_DBYTE_BCAST_SAMPLE_RDY	0x9003e16aUL
#define PHY_REG_DBYTE_BCAST_DQS_DLY0	0x9003e172UL
#define PHY_REG_DBYTE_BCAST_DQS_DLY1	0x9003e174UL
#define PHY_REG_DBYTE_BCAST_TRIG	0x9003e176UL /* Half: DBYTE bcast gate commit */
#define PHY_REG_DBYTE_BCAST_LANE	0x9003e17cUL
#define PHY_REG_DBYTE_BCAST_WCK		0x9003e1b2UL
#define PHY_REG_DBYTE_UPPER_SAMPLE_A	0x9003f16aUL /* Half: Upper DBYTE sample A */
#define PHY_REG_DBYTE_UPPER_DLY0	0x9003fe20UL /* Half: DBYTE upper delay 0 */
#define PHY_REG_DBYTE_UPPER_DLY1	0x9003fe22UL /* Half: DBYTE upper delay 1 */
#define PHY_REG_DBYTE_UPPER_DLY2	0x9003fe24UL /* Half: DBYTE upper delay 2 */
#define PHY_REG_DBYTE_UPPER_DLY3	0x9003fe26UL /* Half: DBYTE upper delay 3 */
#define PHY_REG_DBYTE_UPPER_COARSE	0x9003ff54UL
#define PHY_REG_DBYTE_UPPER_SAMPLE_B	0x9003ff6aUL /* Half: Upper DBYTE sample B */
#define PHY_REG_DBYTE_UPPER_BCAST	0x9003ff7cUL
#define PHY_REG_MASTER_BASE		0x90040000UL
#define PHY_REG_CAL_STAT_CTRL0		0x90040008UL /* Half: PHY cal status ctrl A */
#define PHY_REG_CAL_STAT_CTRL1		0x9004000cUL /* Half: PHY cal status ctrl */
#define PHY_REG_MASTER_CFG		0x9004000eUL /* Master PUB configuration */
#define PHY_REG_SLICE_STAT		0x90040018UL /* Slice / lane status register */
#define PHY_REG_MASTER_DELAY		0x9004001aUL /* Half: Master profile delay */
#define PHY_REG_CAL_STAT_CTRL2		0x90040022UL /* Half: PHY cal status ctrl */
#define PHY_REG_PUB_ACK			0x9004002aUL /* Half: PUB status ack reg */
#define PHY_REG_MASTER_PWR_GATE		0x90040034UL /* Half: Master power gate ctrl */
#define PHY_REG_CLK_GATE		0x90040040UL /* Master PHY clock gate enable */
#define PHY_REG_CLK_GATE_STAT		0x90040042UL /* Master clock gate acknowledge */
#define PHY_REG_DESKEW_DLY0		0x90040044UL /* Deskew delay tap 0 */
#define PHY_REG_DESKEW_DLY1		0x90040046UL /* Deskew delay tap 1 */
#define PHY_REG_TIMER_DUMMY		0x90040048UL /* Half: Dummy APB turnaround read register */
#define PHY_REG_CLK_TIMING_CTRL		0x9004004eUL /* Clock timing control */
#define PHY_REG_PAD_CAL_CTRL		0x90040050UL /* Half: Master pad cal ctrl */
#define PHY_REG_VREF_TRIM_CFG		0x90040054UL
#define PHY_REG_CA_VREF_DAC		0x90040058UL /* Half: CA Vref DAC ctrl A */
#define PHY_REG_PLL_CLK_CTRL		0x90040062UL /* Master PLL & clock control */
#define PHY_REG_VREF_CTRL		0x90040082UL /* Receiver Vref control */
#define PHY_REG_VREF_STAT		0x90040084UL /* Half: Gate settle timing reg */
#define PHY_REG_DRAM_TYPE_CFG		0x90040088UL
#define PHY_REG_GATE_STAT		0x90040092UL /* Half: Gate pulse status reg */
#define PHY_REG_ZQ_STAT			0x90040094UL /* Half: ZQ telemetry status 0 */
#define PHY_REG_PHY_STATUS		0x900400c0UL /* Master PHY status & receiver clock */
#define PHY_REG_MASTER_CAL_CTRL		0x900400dcUL
#define PHY_REG_CH0_ANALOG_PWR		0x900400e4UL /* Ch0 analog power configuration */
#define PHY_REG_CH1_ANALOG_PWR		0x900400e6UL /* Ch1 analog power configuration */
#define PHY_REG_RESET_TRIGGER		0x900400f2UL /* Soft reset pulse toggle */
#define PHY_REG_GATE_MASK		0x900400f4UL
#define PHY_REG_GATE_CTRL		0x900400fcUL
#define PHY_REG_CH0_RANK_CFG		0x90040100UL
#define PHY_REG_CH1_RANK_CFG		0x90040102UL
#define PHY_REG_RFIFO_CFG		0x90040110UL /* Half: RFIFO cfg base */
#define PHY_REG_RFIFO_SUPPRESS		0x90040116UL /* Half: RFIFO suppress ctrl */
#define PHY_REG_MASTER_INIT_CTRL	0x90040120UL /* Half: Master init ctrl */
#define PHY_REG_CBT_CTRL		0x90040144UL /* CBT control register */
#define PHY_REG_CAL_TRIG		0x90040146UL /* Calibration strobe trigger */
#define PHY_REG_CAL_STAT		0x90040148UL /* Calibration status & configuration */
#define PHY_REG_MASTER_COARSE_CFG	0x90040154UL /* Half: Master coarse gate cfg */
#define PHY_REG_CBT_STAT0		0x90040180UL /* Half: CBT status reg */
#define PHY_REG_CBT_STAT1		0x90040182UL /* Half: CBT status reg */
#define PHY_REG_CBT_STAT2		0x90040184UL /* Half: CBT status reg */
#define PHY_REG_AC_PLL_CTRL		0x900401c0UL
#define PHY_REG_CBT_PLL			0x900401f0UL /* CBT PLL control register */
#define PHY_REG_CMD_DISPATCH		0x90040238UL /* Command dispatch strobe */
#define PHY_REG_CLK_PHASE_BASE		0x90040600UL
#define PHY_REG_CLK_PHASE_DELAY		0x90040628UL
#define PHY_REG_PUB_SEC_BASE		0x9005e000UL
#define PHY_REG_SEC_CLK_GATE		0x9005e0a4UL /* Secondary clock gate toggle */
#define PHY_REG_SEC_CAL_CTRL		0x9005e0b2UL
#define PHY_REG_SEC_PUB_CTRL0		0x9005e0eeUL /* Half: Sec PUB ctrl */
#define PHY_REG_PUB_CAL_STROBE		0x9005e140UL /* Secondary calibration strobe */
#define PHY_REG_SEC_PUB_CTRL1		0x9005e14aUL /* Half: Sec PUB ctrl */
#define PHY_REG_AC_BASE			0x90060000UL /* Half: AC slice profile base */
#define PHY_AC_PIN_DELAY_BASE		0x90060002UL /* AC pin delay base */
#define PHY_AC_BASE_OFFSET_CTRL		0x90060004UL /* AC base offset control */
#define PHY_REG_AC_SLICE_CFG		0x9006005cUL
#define PHY_AC_SWIZZLE_S0_BASE		0x90060120UL /* AC slice 0 pin swizzle base */
#define PHY_REG_AC_PLL_LOCK		0x9006017eUL /* Half: AC PLL lock status B */
#define PHY_AC_MASTER_DESKEW0		0x90061002UL /* AC master deskew 0 */
#define PHY_AC_MASTER_DESKEW1		0x90061202UL /* AC master deskew 1 */
#define PHY_AC_GATE_CTRL_RESET		0x90061e02UL /* AC gate control reset */
#define PHY_AC_SWIZZLE_S1_BASE		0x90062120UL /* AC slice 1 pin swizzle base */
#define PHY_AC_PROFILE_LANE_BASE	0x9007e002UL /* AC profile lane base */
#define PHY_AC_PROFILE_MASTER_CTRL	0x9007e004UL /* AC profile master control */
#define PHY_REG_PLL_STATUS		0x9007e034UL /* PLL lock status */
#define PHY_REG_AC_DRIVE_CFG0		0x9007e05cUL
#define PHY_REG_AC_DRIVE_CFG1		0x9007e066UL
#define PHY_REG_AC_DRIVE_CFG2		0x9007e068UL
#define PHY_REG_AC_DRIVE_CFG3		0x9007e080UL
#define PHY_REG_AC_DRIVE_CFG4		0x9007e082UL
#define PHY_REG_AC_DRIVE_CFG5		0x9007e084UL
#define PHY_REG_AC_DRIVE_CFG6		0x9007e086UL
#define PHY_REG_AC_BCAST_GATE		0x9007e108UL
#define PHY_REG_CSR_RESULT_STREAM	0x90082000UL /* Half: PHY CSR result stream */
#define PHY_REG_BIST_PLL_CFG		0x900e0022UL /* Half: AC ctrl read source */
#define PHY_REG_DESKEW_QUAD_TAP		0x900e0048UL /* Half: Deskew quad-tap base */
#define PHY_REG_AC_PAT_CTRL		0x900e00c0UL
#define PHY_REG_PLL_CFG			0x900e00c2UL /* Master PLL configuration parameter 0 */
#define PHY_REG_AC_TX_EN_BCAST		0x900e00caUL /* AC transmitter broadcast enable */
#define PHY_REG_AC_BCAST		0x900fe022UL /* Half: AC bcast write target */
#define PHY_REG_BIST_CTRL_MODE		0x900fe0c0UL /* Half: AC bcast test pattern A */
#define PHY_REG_AC_BCAST_PAT_DATA	0x900fe0caUL /* Half: AC bcast test pattern B */
#define PHY_REG_AC_BCAST_PAT_CFG	0x900fe1acUL /* Half: AC bcast pattern ctrl B */
#define PHY_REG_AC_BCAST_PAT_CTRL	0x900fe1aeUL /* Half: AC bcast pattern ctrl */
#define PHY_REG_BIST_PAT_CTRL0		0x900ffe4cUL
#define PHY_REG_BIST_PAT_CTRL1		0x900ffe4eUL
#define PHY_REG_CSR_SCRATCH_BASE	0x90120052UL /* Half: PUB CSR scratchpad */
#define PHY_REG_CAL_LATCH_STREAM	0x90121000UL
#define PHY_REG_CAL_RESTORE_TRIG	0x9013e050UL
#define PHY_REG_SHADOW_TRACK_BASE	0x90140000UL /* Half: Shadow track aperture */
#define PHY_REG_MAILBOX_STAT		0x90180008UL /* Mailbox status */
#define PHY_REG_MAILBOX_STREAM		0x90180064UL /* Mailbox stream low */
#define PHY_REG_MAILBOX_TRIG		0x90180066UL /* Mailbox interrupt trigger */
#define PHY_REG_PUB_CFG			0x90180100UL
#define PHY_REG_PUB_CTL			0x901801dcUL /* PUB control */
#define PHY_REG_SHADOW_CNT		0x9018021eUL /* Half: Shadow stream count reg */

/* ========================================================================== */
/* ARC EM4 Auxiliary Timer Registers                                          */
/* ========================================================================== */

#define PMU_AUX_RTC_CTRL		0x103 /* AUX 259: RTC control / enable register */
#define PMU_AUX_RTC_LOW			0x104 /* AUX 260: RTC low 32-bit counter */
#define PMU_AUX_RTC_HIGH		0x105 /* AUX 261: RTC high 32-bit counter */

/* ========================================================================== */
/* PMU Firmware Identifiers & Assertion Error Codes                           */
/* ========================================================================== */

#define PMU_FW_VERSION_ID		0x230bU
#define PHY_CSR_SCRATCH_MAGIC_LO	0x55aaU
#define PHY_CSR_SCRATCH_MAGIC_HI	0xaa55U
#define PMU_ERR_PLL_LOCK_TIMEOUT	0x00240000UL
#define PMU_ERR_CLK_GATE_TIMEOUT	0x00250000UL
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
#define PMU_ERR_BIST_PATTERN_IDX_INVALID	0x01a40001UL
#define PMU_ERR_BIST_VALIDATION_MODE	0x01a50000UL
#define PMU_ERR_BIST_CMD_MODE_INVALID	0x01a80001UL
#define PMU_ERR_EYE_SLICE_REG_SOURCE	0x01ab0000UL
#define PMU_ERR_EYE_WINDOW_TOO_NARROW	0x01b10006UL
#define PMU_ERR_EYE_SWEEP_CONVERGENCE	0x01f60000UL
#define PMU_ERR_CMD_SUBCODE_INVALID	0x02190001UL
#define PMU_ERR_CSR_BUS_ACCESS_FAULT	0x02240000UL
#define PMU_ERR_WCK_EYE_ZERO_WIDTH	0x022e0003UL
#define PMU_ERR_VREF_STEP_DIR_INVALID	0x02350000UL
#define PMU_ERR_BIST_PATTERN_MODE_RANGE	0x027e0001UL
#define PMU_ERR_COARSE_DELAY_OVERFLOW	0x02a80000UL

#endif /* _DWC_DDRPHY_PMU_REGS_H */
