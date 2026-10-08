/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Synopsys DesignWare DDR PHY PMU Microcontroller Register & Memory Definitions
 * Target Architecture: Synopsys ARC EM4 (ARCv2 ISA)
 * SoC: Allwinner A733 (Sun60i) / LPDDR4 & LPDDR5 PHY
 *
 * For detailed PHY silicon architecture and slice descriptions,
 * see doc/board/allwinner/a733-pmu.rst.
 */

#ifndef _DWC_DDRPHY_PMU_REGS_H
#define _DWC_DDRPHY_PMU_REGS_H

#include "sunxi_pmu_abi.h"

/* ========================================================================== */
/* ARC PMU Internal Memory Map                                                */
/* ========================================================================== */

/*
 * IMEM: Instruction Memory (64 KB)
 *   Host ARM MMIO: 0x0a9a0000
 *   ARC Internal:  0x00000000 (ICCM: Instruction Closely-Coupled Memory)
 */

/*
 * DMEM: Data Memory (64 KB)
 *   Host ARM MMIO: 0x0a9b0000
 *   ARC Internal:  0x80000000 (DCCM: Data Closely-Coupled Memory)
 */
#define PMU_DMEM_BASE               0x80000000

/* DMEM Global Pointer (gp) Base & Key Parameter Offsets */

/* 1. Host Message Block & Top-Level Configuration (0x00..0x1f) */
#define PMU_DMEM_HOST_MSG_BASE          0x00    /* Base of host message block */
#define PMU_DMEM_CLK_GATE_FLAG          0x01    /* Byte: Clock gate pulse enable flag */
#define PMU_DMEM_CHANNEL_CFG            0x04    /* Byte: Channel configuration & geometry flags */
#define PMU_DMEM_PLL_BYPASS             0x05    /* Byte: PLL bypass & fast clock toggle mode */
#define PMU_DMEM_DRAM_FREQ_OFF          0x06    /* Half: Clock in MHz offset */
#define PMU_DMEM_DRAM_TYPE              0x08    /* Byte: DRAM type (3 = LPDDR5, 2 = LPDDR4) */
#define PMU_DMEM_DRAM_TYPE_FLAGS        0x09    /* Byte: Additional DRAM type flags */
#define PMU_DMEM_DRAM_CFG_FLAGS         0x0a    /* Half: DRAM config bitmask (bit 13 flag) */
#define PMU_DMEM_CAL_SELECT_FLAGS       0x0b    /* Byte: Calibration selector flags (bit 2) */
#define PMU_DMEM_CAL_SELECT_FLAGS_0C    0x0c    /* Byte: Secondary calibration selector flags */
#define PMU_DMEM_CAL_MISC_FLAGS         0x0d    /* Byte: Calibration misc flags (bits 0, 2, 5) */
#define PMU_DMEM_CAL_SHIFT_CFG          0x0e    /* Byte: Calibration shift config & signed bias */
#define PMU_DMEM_CAL_CTRL_0F            0x0f    /* Byte: Calibration control register 0x0f */
#define PMU_DMEM_SEQUENCE_CTRL          0x10    /* Half: SequenceCtrl training stage bitmask */
#define PMU_DMEM_POST_THRESHOLD         0x12    /* Byte: POST mailbox telemetry log threshold */
#define PMU_POST_CMD_THRESH_200         200     /* POST command dispatch threshold */
#define PMU_DMEM_TRAIN_MODE             0x14    /* Half: Training mode flags (bit 3: 2D vs 1D) */
#define PMU_DMEM_TRAIN_MODE_HI          0x15    /* Byte: Upper byte of train_mode */
#define PMU_DMEM_TRAIN_FLAGS_16         0x16    /* Byte: Training flags register 0x16 */
#define PMU_DMEM_TRAIN_CTRL_17          0x17    /* Byte: Training control register 0x17 */
#define PMU_DMEM_TRAIN_CTRL_18          0x18    /* Byte: Training control register 0x18 */
#define PMU_DMEM_CAL_DISABLE_19         0x19    /* Byte: Calibration disable / bypass flag */
#define PMU_DMEM_CAL_STEP_CONFIG        0x1a    /* Byte: Calibration step & CA eye margin config */
#define PMU_DMEM_CAL_MODE_1B            0x1b    /* Byte: Calibration mode register 0x1b */
#define PMU_DMEM_ACTIVE_LANES           0x1c    /* Byte: Active lane / rank bitmask */

/* 2. Channel & Rank Top-Level Geometries (0x20..0x4f) */
#define PMU_DMEM_CH0_CTRL_20            0x20    /* Byte: Channel 0 control register 0x20 */
#define PMU_DMEM_CH0_CTRL_21            0x21    /* Byte: Channel 0 control register 0x21 */
#define PMU_DMEM_CH0_DQ_WIDTH           0x24    /* Byte: Channel 0 DQ width in bits (8/16/32) */
#define PMU_DMEM_CH0_RANK_EN            0x25    /* Byte: Channel 0 rank active / enable mask */
#define PMU_DMEM_CH0_RANK_CFG           0x32    /* Byte: Channel 0 rank configuration flags */
#define PMU_DMEM_CH0_RANK_TIMING_33     0x33    /* Byte: Channel 0 Rank timing register 0x33 */
#define PMU_DMEM_CH0_RANK_TIMING_34     0x34    /* Byte: Channel 0 Rank timing register 0x34 */
#define PMU_DMEM_CH1_DQ_WIDTH           0x3f    /* Byte: Channel 1 DQ width in bits (8/16/32) */
#define PMU_DMEM_CH1_RANK_EN            0x40    /* Byte: Channel 1 rank active / enable mask */
#define PMU_DMEM_CH1_RANK_CFG           0x4d    /* Byte: Channel 1 rank configuration flags */
#define PMU_DMEM_CH1_RANK_TIMING_4E     0x4e    /* Byte: Channel 1 Rank timing register 0x4e */
#define PMU_DMEM_CH1_RANK_TIMING_4F     0x4f    /* Byte: Channel 1 Rank timing register 0x4f */

/* 3. 2D Search Windows & Timing Controls (0x37..0x9f) */
#define PMU_DMEM_SEARCH_WIN_COARSE      0x37    /* Byte: Coarse 2D eye search window buffer */
#define PMU_DMEM_SEARCH_WIN_FINE        0x39    /* Byte: Fine 2D eye search window buffer */
#define PMU_DMEM_WINDOW_STATE_52        0x52    /* Base: Window state buffer offset 0x52 */
#define PMU_DMEM_LANE_SWEEP_CTRL        0x59    /* Byte: Multi-lane window sweep control flags */
#define PMU_DMEM_SLICE_PROFILE_TABLE    0x5a    /* Byte: 4-byte DBYTE slice profile offset table */
#define PMU_DMEM_VREF_FLAG_SIGNED       0x62    /* Byte: Signed Vref direction / limit flag */
#define PMU_DMEM_VREF_DAC_CTRL          0x72    /* Byte: Vref DAC step adjustment control flag */
#define PMU_DMEM_TIMING_MODE_CTRL       0x8a    /* Byte: DRAM timing window budget selector */
#define PMU_TIMING_MODE_MASK            0x03    /* Bits [1:0] mode mask */
#define PMU_DMEM_WCK_SYNC_MODE          0x8e    /* Byte: LPDDR5 WCK sync mode control */
#define PMU_WCK_SYNC_MODE_MASK          0x03    /* Bits [1:0] mode mask */
#define PMU_DMEM_TRAIN_PARAM_CTRL       0x96    /* Byte: DRAM training config & latency offset */
#define PMU_TRAIN_PARAM_NIBBLE_MASK     0x0f    /* Lower 4-bit nibble mask */

/* 4. Best Calibrated Taps & Stage Telemetry Results (0xd0..0x11f) */
#define PMU_DMEM_BEST_TAP_C0R0          0xd1    /* Byte: Channel 0 Rank 0 calibrated best tap */
#define PMU_DMEM_BEST_TAP_C1R0          0xd6    /* Byte: Channel 1 Rank 0 calibrated best tap */
#define PMU_DMEM_BEST_TAP_C0R1          0xdb    /* Byte: Channel 0 Rank 1 calibrated best tap */
#define PMU_DMEM_BEST_TAP_C1R1          0xe0    /* Byte: Channel 1 Rank 1 calibrated best tap */
#define PMU_DMEM_TRAIN_STATUS           0xe7    /* Byte: Train status (bit 0=pass, 1=fail) */
#define PMU_DMEM_CAL_OVERRIDE_FLAG      0xce    /* Byte: Calibration override / bypass flag */
#define PMU_DMEM_BIST_PATTERN_LO        0xe4    /* Byte: 16-bit BIST pattern low byte */
#define PMU_DMEM_BIST_PATTERN_HI        0xe5    /* Byte: 16-bit BIST pattern high byte */
#define PMU_DMEM_DESKEW_OFFSET_OVERRIDE 0xe6    /* Byte: Deskew offset override value */
#define PMU_DMEM_RANK_BASE_VAL_E8       0xe8    /* Half: Rank base value E8 */
#define PMU_DMEM_RANK_BASE_VAL_EA       0xea    /* Half: Rank base value EA */
#define PMU_DMEM_AC_PROFILE_BASE        0xf0    /* Byte: AC profile parameter buffer base */
#define PMU_DMEM_CAL_STEP_METRIC_F2     0xf2    /* Half: Calibration step metric offset 0xf2 */
#define PMU_DMEM_AC_PROFILE_EN          0xf4    /* Byte: AC profile parameter valid / enable flag */
#define PMU_DMEM_DQ_SWAP_BASE           0x100   /* Byte: DQ swap register base */
#define PMU_DMEM_DQ_SWAP_MASK           0x101   /* Byte: DQ/DQS swap & polarity mask */
#define PMU_DMEM_TRAIN_FEATURE_MASK     0x102   /* Byte: Training feature bitmask */
#define PMU_DMEM_DIFF_TAP_PAIR          0x103   /* Byte: Differential delay tap pair buffer */
#define PMU_DMEM_DIFF_TAP_PAIR_105      0x105   /* Byte: Diff delay tap pair reg 0x105 */
#define PMU_DMEM_CAL_STEP_106           0x106   /* Byte: Calibration step register 0x106 */
#define PMU_DMEM_CAL_STEP_108           0x108   /* Byte: Calibration step register 0x108 */
#define PMU_DMEM_CAL_STEP_10A           0x10a   /* Byte: Calibration step register 0x10a */
#define PMU_DMEM_CAL_STEP_10C           0x10c   /* Byte: Calibration step register 0x10c */
#define PMU_DMEM_CAL_STEP_10E           0x10e   /* Byte: Calibration step register 0x10e */
#define PMU_DMEM_SEARCH_WINDOW_BIAS     0x110   /* Byte: Signed calibration search window bias */
#define PMU_DMEM_SEARCH_WINDOW_BIAS_HI  0x111   /* Byte: Search window bias high byte */
#define PMU_DMEM_RANK_TAP_130           0x130   /* Byte: Rank tap register 0x130 */
#define PMU_DMEM_RANK_TAP_132           0x132   /* Byte: Rank tap register 0x132 */
#define PMU_DMEM_RANK_TAP_134           0x134   /* Byte: Rank tap register 0x134 */
#define PMU_DMEM_RANK_TAP_136           0x136   /* Byte: Rank tap register 0x136 */
#define PMU_DMEM_RANK_TAP_138           0x138   /* Byte: Rank tap register 0x138 */
#define PMU_DMEM_RANK_TAP_13A           0x13a   /* Byte: Rank tap register 0x13a */
#define PMU_DMEM_RANK_TAP_13C           0x13c   /* Byte: Rank tap register 0x13c */

/* 5. Operating States & CSR Context (0x400..0x43f) */
#define PMU_DMEM_CAL_CFG_400            0x400   /* Byte: Calibration configuration 0x400 */
#define PMU_DMEM_HIGH_FREQ_FLAG         0x401   /* Byte: High frequency indicator flag */
#define PMU_DMEM_CAL_ACTIVE_FLAG        0x402   /* Byte: Calibration stage active flag */
#define PMU_DMEM_FREQ_MODE              0x403   /* Byte: Frequency mode / Pstate index */
#define PMU_DMEM_STAGE_STATUS           0x404   /* Byte: Calibration stage status */
#define PMU_DMEM_STAGE_STEP_MODE        0x405   /* Byte: Calibration stage step mode */
#define PMU_DMEM_STAGE_PARAM_406        0x406   /* Byte: Stage parameter 0x406 */
#define PMU_DMEM_STAGE_PARAM_407        0x407   /* Byte: Stage parameter 0x407 */
#define PMU_DMEM_STAGE_PARAM_408        0x408   /* Byte: Stage parameter 0x408 */
#define PMU_DMEM_METRIC_TABLE_ENTRIES   0x409   /* Byte: Number of metric table entries */
#define PMU_DMEM_ACTIVE_SLICE_BITMAP    0x40c   /* Word: Active slice bitmap / metric limit */
#define PMU_DMEM_CAL_PARAM_410          0x410   /* Word: Calibration parameter 0x410 */
#define PMU_DMEM_CAL_PARAM_414          0x414   /* Word: Calibration parameter 0x414 */
#define PMU_DMEM_SHADOW_TRACE_PTR       0x418   /* Word: Ptr into shadow trace log buffer */
#define PMU_DMEM_ACTIVE_CSR_OFFSET      0x41c   /* Word: Currently active slice CSR offset */
#define PMU_DMEM_METRIC_COEFF_X         0x420   /* Half: Metric quadratic coeff for X */
#define PMU_DMEM_METRIC_COEFF_Y         0x422   /* Half: Metric quadratic coeff for Y */
#define PMU_DMEM_METRIC_COEFF_Z         0x424   /* Halfword: Delay metric coefficient for Z */
#define PMU_DMEM_METRIC_COEFF_W         0x426   /* Halfword: Delay metric coefficient for W */
#define PMU_DMEM_ITER_COUNT             0x428   /* Halfword: Calibration iteration / rank counter */
#define PMU_DMEM_SAVED_CAL_STAT         0x42a   /* Half: PHY register 0x90040022 saved value */
#define PMU_DMEM_ACTIVE_SLICE_COUNT     0x42c   /* Byte: Total active PHY DBYTE slices */
#define PMU_DMEM_SAVED_WCK_DELAY        0x430   /* Base: WCK delay register backup array */
#define PMU_DMEM_2D_STEP_CTRL           0x438   /* Halfword: 2D step control register 0x438 */
#define PMU_DMEM_2D_STEP_PARAM_43A      0x43a   /* Halfword: 2D step parameter 0x43a */
#define PMU_DMEM_2D_STEP_TABLE          0x43c   /* Base: 2D calibration step parameter table */
#define PMU_DMEM_2D_STEP_RANK0          0x43c   /* Word: 2D step parameter pair for Rank 0 */
#define PMU_DMEM_2D_STEP_RANK1          0x440   /* Word: 2D step parameter pair for Rank 1 */

/* 6. AC Step Tables, CBT State & Deskew Targets (0x444..0x4ff) */
#define PMU_DMEM_AC_STEP_WORDS          0x444   /* Base: AC slice step 16-bit register table */
#define PMU_DMEM_AC_STEP_BYTES          0x448   /* Base: AC slice step 8-bit register table */
#define PMU_DMEM_CAL_MARKER_A           0x458   /* Calibration target marker A */
#define PMU_DMEM_CBT_STATE              0x45a   /* CBT state latch */
#define PMU_DMEM_CBT_CAL_STATUS_SHADOW  0x45b   /* Byte: Shadow of CBT calibration active status */
#define PMU_DMEM_CBT_CAL_STATUS         0x45c   /* CBT calibration active status flag */
#define PMU_DMEM_CAL_MARKER_B           0x45e   /* Calibration target marker B */
#define PMU_DMEM_CBT_PARAM              0x460   /* Byte: CBT calibration parameter / mode */
#define PMU_DMEM_CBT_PARAM_464          0x464   /* Word: CBT parameter word 0x464 */
#define PMU_DMEM_CBT_CTRL_468           0x468   /* Byte: CBT control register 0x468 */
#define PMU_DMEM_CBT_CONFIG             0x46a   /* CBT configuration value */
#define PMU_DMEM_CBT_STEP_STATUS        0x46b   /* CBT step status flag */
#define PMU_DMEM_CAL_CMD_TABLE          0x46c   /* Base: Calibration command opcode table */
#define PMU_DMEM_ACTIVE_SLICE_IDX       0x470   /* Word: Active slice/lane index */
#define PMU_DMEM_RANK_SLICE_TAP_STEPS   0x474   /* Base: 2-rank x 4-slice delay tap step table */
#define PMU_DMEM_CAL_STRIDE             0x47c   /* Byte: Calibration stride */
#define PMU_DMEM_DESKEW_TARGET_DELAY    0x47e   /* Deskew target delay parameter */
#define PMU_DMEM_CAL_SEQ_MAP            0x480   /* Base: 21-byte calibration sequence index map */
#define PMU_DMEM_PHASE_DETECT_TABLE_A   0x4a4   /* Base: Phase detector table A (8 bytes) */
#define PMU_DMEM_PHASE_DETECT_TABLE_D   0x4ac   /* Base: Phase detector table D (12 bytes) */
#define PMU_DMEM_PHASE_DETECT_TABLE_E   0x4b8   /* Base: Phase detector table E (14 bytes) */
#define PMU_DMEM_WCK_INIT_STREAM        0x4c6   /* Base: WCK init register stream (18B) */
#define PMU_DMEM_PHASE_DETECT_TABLE_B   0x4d8   /* Base: Phase detector table B (22 bytes) */
#define PMU_DMEM_PHASE_DETECT_TABLE_C   0x4ee   /* Base: Phase detector table C (28 bytes) */

/* 7. CBT Lookup Tables & Multi-Rank Structures (0x500..0xaff) */
#define PMU_DMEM_CBT_OFFSET_TABLE       0x50c   /* Base: CBT calibration delay offset table */
#define PMU_DMEM_CBT_LOOKUP_TABLE       0x724   /* Base: CBT pattern lookup table */
#define PMU_DMEM_CAL_STRUCT_BASE        0x9a4   /* Base: Multi-rank calibration struct (216B) */
#define PMU_DMEM_CAL_DESC_BACKUP_99C    0x99c   /* Base: 6-byte calibration descriptor backup */
#define PMU_DMEM_ACTIVE_SLICE_MASK      0x9b8   /* Byte: Active DBYTE slice bitmask (0x0f) */
#define PMU_DMEM_AC_STEP_FLAG           0xa7c   /* Byte: AC step calibration valid flag */
#define PMU_DMEM_AC_STEP_CFG            0xa7d   /* Byte: AC profile step configuration register */
#define PMU_DMEM_DQ_PIN_MAP_BASE        0x80000a7e /* Base of DBYTE DQ pin swizzle/mapping table */
#define PMU_DQ_PIN_MAP_ENTRY_SIZE       18      /* Bytes per entry in DQ pin map table */

/* 8. Multi-Rank Slice Tables & Calibration Shadow Buffers (0xb00..0xfff) */
#define PMU_DMEM_RANK_SLICE_COUNTS      0xb20   /* Base: Rank slice count configuration table */
#define PMU_DMEM_RANK_SLICE_MAP         0xb44   /* Base: Rank slice list mapping table */
#define PMU_DMEM_SEARCH_WINDOW_STEPS    0xb64   /* Byte: 2D search window delay steps */
#define PMU_DMEM_SEARCH_WINDOW_SCALE    0xb65   /* Byte: 2D search window scaling factor */
#define PMU_DMEM_CAL_RANK               0xb66   /* Byte: Current rank index */
#define PMU_DMEM_CAL_BYTE               0xb67   /* Byte: Current byte lane index */
#define PMU_DMEM_SLICE_START            0xb68   /* Start byte lane / slice index */
#define PMU_DMEM_SLICE_END              0xb69   /* End byte lane / slice index */
#define PMU_DMEM_RANK0_SLICE_START      0xb6a   /* Byte: Primary start slice index (Rank 0) */
#define PMU_DMEM_RANK_BOUNDARY          0xb6b   /* Rank boundary threshold (Rank 0 end slice) */
#define PMU_DMEM_RANK1_SLICE_START      0xb6c   /* Byte: Secondary start slice index (Rank 1) */
#define PMU_DMEM_RANK1_SLICE_END        0xb6d   /* Byte: Secondary end slice index (Rank 1) */
#define PMU_DMEM_DELAY_CAL_RESULTS      0xb6e   /* Base: Calibrated delay result table */
#define PMU_DMEM_DELAY_CAL_RESULT_70    0xb70   /* Base: Calibrated delay result table 0x70 */
#define PMU_DMEM_RANK_CH_TAP_TABLE      0xb7a   /* Base: 4-entry table [rank][ch] of best taps */
#define PMU_DMEM_DUAL_RANK_REG_TABLE    0xb82   /* Base: Dual-rank PHY register status table */
#define PMU_DMEM_LANE_ACTIVE_FLAGS      0xb96   /* Byte: Lane active flags */
#define PMU_DMEM_LANE_STATUS            0xb97   /* Lane status latch */
#define PMU_DMEM_ACTIVE_SLICE_MASK_CAL  0xb98   /* Byte: Active slice bitmask for cal */
#define PMU_DMEM_CAL_STRUCT_PTR         0xb9c   /* Word: Pointer to calibration data structure */
#define PMU_DMEM_SAVED_DQ_DESKEW_TABLE  0xba0   /* Base: DBYTE slice delay parameter array */
#define PMU_DMEM_CAL_SHADOW_TABLE       0xbc0   /* Base: 16-bit calibration shadow register table */
#define PMU_DMEM_CBT_PATTERN_BASE       0xd70   /* Base: CBT pattern registers 0xd70..0xd8a */
#define PMU_DMEM_CBT_PAT_SHIFT_LO       0xd70   /* CBT pattern shift low */
#define PMU_DMEM_CBT_PAT_STATUS         0xd76   /* CBT pattern status flag */
#define PMU_DMEM_CBT_PAT_TAP            0xd78   /* CBT pattern tap index */
#define PMU_DMEM_CBT_PAT_SLICE_LO       0xd7a   /* CBT pattern slice index low */
#define PMU_DMEM_CBT_PAT_SHIFT_HI       0xd80   /* CBT pattern shift high */
#define PMU_DMEM_CBT_PAT_CFG            0xd86   /* CBT pattern configuration */
#define PMU_DMEM_CBT_PAT_CODE           0xd88   /* CBT pattern param code */
#define PMU_DMEM_CBT_PAT_SLICE_HI       0xd8a   /* CBT pattern slice index high */
#define PMU_DMEM_CBT_DEBUG_BASE         0xdac   /* Base: CBT debug registers 0xdac..0xdb8 */
#define PMU_DMEM_CBT_DEBUG_0            0xdac   /* CBT debug telemetry word 0 */
#define PMU_DMEM_CBT_DEBUG_1            0xdb0   /* CBT debug telemetry word 1 */
#define PMU_DMEM_CBT_DEBUG_2            0xdb4   /* CBT debug telemetry word 2 */
#define PMU_DMEM_CBT_DEBUG_3            0xdb8   /* CBT debug telemetry word 3 */
#define PMU_DMEM_CBT_LOCK_STATUS        0xdbc   /* Halfword: CBT lock status */
#define PMU_DMEM_CAL_TRACKER            0xd9c   /* Base: Calibration tracker struct */
#define PMU_DMEM_CAL_TRACKER_STATUS     0xdbe   /* Half: Calibration tracker status word (0x22) */
#define PMU_DMEM_CBT_PHASE_STATUS       0xdc0   /* Halfword: CBT phase status */
#define PMU_DMEM_EYE_SAMPLE_BUF         0xdc8   /* Base: Mid-eye telemetry sample buffer */
#define PMU_PIN_SAMPLE_BASE             0x0dc8  /* Base offset for slice 0 pin telemetry */
#define PMU_PIN_SAMPLE_STRIDE           132     /* Byte stride between consecutive pin records */
#define PMU_DMEM_PIN_CAL_BASE_TAPS      0xe48   /* Base: Per-pin calibrated base delay taps */
#define PMU_DMEM_PIN_CAL_DELTA_TAPS     0xe4a   /* Base: Per-pin calibrated delta offset taps */
#define PMU_DMEM_METRIC_CAL_E7B8        0xe7b8  /* Base: Calibration metric results table */
#define PMU_DMEM_METRIC_SUB_E7C4        0xe7c4  /* Base: Calibration metric sub-table */

/* Common Algorithm Constants */
#define PMU_CAL_DEFAULT_PARAM           0x00080008 /* Default calibration parameter pair */
#define PMU_FREQ_THRESHOLD_3200         3200    /* DRAM frequency threshold in MHz */
#define PMU_CAL_STROBE_TICKS            0x20    /* Strobe pulse delay duration in ticks */
#define PMU_LCDL_FINE_MASK              0x3f    /* Fine delay bitmask (6 bits) */
#define PMU_LCDL_COARSE_SHIFT           6       /* Coarse delay bit-shift */
#define PMU_LUT_2B_IDENTITY_E4          0xe4    /* 2-bit identity table (11 10 01 00b) */
#define PMU_LCDL_NOMINAL_MIDPOINT       0x40    /* Nominal LCDL delay midpoint (64) */
#define PMU_CFG_FLAG_BIT13_SHIFT        13      /* Bit 13 shift in DRAM config flags */
#define PMU_LCDL_STEP_COARSE            0x20    /* Coarse delay adjustment step */
#define PMU_LCDL_STEP_OVERFLOW_MASK     0x03c0  /* Mask to test coarse step overflow */
#define PMU_LCDL_STEP_WRAP_OFFSET       0x0400  /* Delay line wrap-around compensation */
#define PHY_REG_CAL_STAT_CTRL       0x90040022  /* PHY calibration status & control */
#define PHY_REG_MASTER_CFG          0x9004000e  /* Master configuration register */
#define PHY_REG_VREF_TRIM_CFG       0x90040054  /* Master VREF configuration */
#define PHY_REG_DBYTE_BCAST_BASE    0x9003e000  /* DBYTE broadcast register base */
#define PHY_REG_DBYTE_BCAST_PULSE   0x9003e002  /* DBYTE broadcast pulse control */
#define PHY_REG_DBYTE_BCAST_GATE    0x9003e00a  /* DBYTE broadcast gate control */
#define PHY_REG_DBYTE_BCAST_SAMPLE  0x9003e00e  /* DBYTE broadcast sample control */
#define PHY_REG_DBYTE_BCAST_MODE    0x9003e012  /* DBYTE broadcast mode configuration */
#define PHY_REG_DBYTE_BCAST_TIMING  0x9003e014  /* DBYTE broadcast timing configuration */
#define PHY_REG_DBYTE_BCAST_WIDTH   0x9003e016  /* DBYTE broadcast pulse width */
#define PHY_REG_DBYTE_BCAST_RX_EN   0x9003e14e  /* DBYTE broadcast RX enable control */
#define PHY_REG_DBYTE_BCAST_STEP    0x9005e01a  /* DBYTE broadcast step configuration */
#define PHY_REG_AC_STAT_LOW         0x9007e002  /* AC status register low */
#define PHY_REG_AC_STAT_HIGH        0x9007e004  /* AC status register high */
#define PHY_REG_SHADOW_COUNT        0x9018021e  /* Shadow register write count */
#define PHY_REG_PUB_ID_REVISION     0x90180100  /* PUB revision ID register */
#define PHY_REG_CLK_PHASE_TOGGLE    0x90040600  /* Master PHY clock phase toggle */
#define PHY_REG_AC_PLL_LOCK_STAT    0x9006017e  /* AC PLL lock status register */
#define PHY_REG_DBYTE_LOCK_STAT     0x900201c0  /* DBYTE slice 0 lock status register */
#define PHY_REG_APB_BASE            0x90000000  /* ARC Internal APB Base */
#define PHY_REG_PUB_BASE            0x90000000  /* ARC Internal PUB/APB Base */
#define PHY_REG_MASTER_BASE         0x90040000  /* Master PHY Utility Block (PUB) Base */
#define PHY_REG_PUB_SEC_BASE        0x9005e000  /* Secondary PHY Utility Block Base */
#define PHY_REG_AC_BASE             0x90060000  /* AC (Address/Command) Slice Base */
#define PHY_REG_AC_PROFILE_BASE     0x9007e000  /* AC Profile Register Base */
#define PHY_REG_PLL_CFG_BASE        0x900e0000  /* PHY PLL Configuration Base */
#define PHY_REG_VREF_STAT           0x90040084  /* Master VREF status / trim register */
#define PHY_REG_DRAM_TYPE_CFG       0x90040088  /* DRAM type configuration register */
#define PHY_REG_INTERRUPT_MASK      0x900400f4  /* PHY interrupt mask register */
#define PHY_REG_INTERRUPT_CLEAR     0x900400fc  /* PHY interrupt clear register */
#define PHY_REG_PUB_GATE_CTRL       0x900401c0  /* PUB gate control register */
#define PHY_REG_CLK_TIMING_DELAY0   0x90040628  /* Master clock timing delay register 0 */
#define PHY_REG_CLK_TIMING_DELAY1   0x9004062a  /* Master clock timing delay register 1 */
#define PHY_REG_DBYTE_BCAST_PARAM   0x9003e01e  /* DBYTE broadcast parameter configuration */
#define PHY_REG_DBYTE_BCAST_LCDL    0x9003e144  /* DBYTE broadcast LCDL configuration */
#define PHY_REG_DBYTE_BCAST_DQ_DLY0 0x9003e160  /* DBYTE broadcast DQ delay tap 0 */
#define PHY_REG_DBYTE_BCAST_DQ_DLY1 0x9003e162  /* DBYTE broadcast DQ delay tap 1 */
#define PHY_REG_DBYTE_BCAST_DQ_DLY2 0x9003e166  /* DBYTE broadcast DQ delay tap 2 */
#define PHY_REG_DBYTE_BCAST_DQ_DLY3 0x9003e168  /* DBYTE broadcast DQ delay tap 3 */
#define PHY_REG_DBYTE_BCAST_DQS_DLY0 0x9003e172 /* DBYTE broadcast DQS delay tap 0 */
#define PHY_REG_DBYTE_BCAST_DQS_DLY1 0x9003e174 /* DBYTE broadcast DQS delay tap 1 */
#define PHY_REG_DBYTE_BCAST_CLEAR   0x9003ff6a  /* DBYTE broadcast status clear register */
#define PHY_REG_BIST_PLL_CFG        0x900e0022  /* PLL BIST configuration register */
#define PHY_REG_BIST_CMD            0x900fe022  /* BIST command execution register */
#define PHY_REG_BIST_CTRL_MODE      0x900fe0c0  /* BIST mode control register */
#define PHY_REG_BIST_CTRL_TRIG      0x900fe0ca  /* BIST trigger control register */
#define PHY_REG_BIST_STAT_WORD      0x900fe1ac  /* BIST status / mask word */
#define PHY_REG_BIST_STAT_FAIL      0x900fe1ae  /* BIST failure status register */
#define PMU_DMEM_TRAINED_DLY_BUF    0x6098      /* Trained delay parameters buffer */
#define PMU_DMEM_SWEEP_SAMPLE_BUF   0x6318      /* Sweep sample buffer (10 lanes * 848B) */

/* LPDDR5 Trained Delay Parameters Export (Host U-Boot reads 0x0a9b0026..0x0a9b004c) */

/* ========================================================================== */
/* Synopsys DDR PHY Internal APB Registers (ARC Address Space)                */
/* ========================================================================== */

/* Power / Clock / PLL Registers */
#define PHY_REG_CLK_GATE            0x90040040  /* Master PHY clock gate enable */
#define PHY_REG_CLK_GATE_STAT       0x90040042  /* Master clock gate acknowledge status */
#define PHY_REG_DESKEW_DLY0         0x90040044  /* Deskew delay tap 0 */
#define PHY_REG_DESKEW_DLY1         0x90040046  /* Deskew delay tap 1 */
#define PHY_REG_CLK_TIMING_CTRL     0x9004004e  /* Clock timing control */
#define PHY_REG_SLICE_STATUS        0x90040018  /* Slice / lane status register */
#define PHY_REG_PLL_CLK_CTRL        0x90040062  /* Master PLL & clock control */
#define PHY_REG_RESET_TRIGGER       0x900400f2  /* Soft reset pulse toggle */
#define PHY_REG_INIT_COMPLETE       0x90040120
#define PHY_REG_CAL_TRIG            0x90040146  /* Calibration strobe trigger */
#define PHY_REG_CAL_STATUS          0x90040148  /* Calibration status & configuration */
#define PHY_REG_CBT_PLL_CTRL        0x900401f0  /* CBT PLL control register */
#define PHY_REG_CMD_DISPATCH        0x90040238  /* Command dispatch strobe */
#define PHY_REG_PUB_CLK_GATE_SYNC   0x9005e0a4  /* Secondary clock gate toggle */
#define PHY_REG_PUB_CAL_STROBE      0x9005e140  /* Secondary calibration strobe */
#define PHY_REG_PLL_CTRL            0x90040034
#define PHY_REG_PLL_STATUS          0x9007e034
#define PHY_REG_PUB_CTL             0x901801dc
#define PHY_REG_PHY_STATUS          0x900400c0

/* WCK Clock and Timing Calibration Registers (LPDDR5) */
#define PHY_REG_WCK_DELAY           0x90020146

/* Datapath Delay Line Registers (DX = Byte Lane 0..3) */

/* Receiver & Analog Calibration Registers */
#define PHY_REG_VREF_CTRL           0x90040082
#define PHY_REG_ANALOG_CFG0         0x900400e4  /* Target PHY configuration register 0 */
#define PHY_REG_ANALOG_CFG1         0x900400e6  /* Target PHY configuration register 1 */
#define PHY_REG_CBT_CTRL            0x90040144

#define PHY_REG_BIST_MASK_DX0       0x900e00c2  /* Master PLL configuration parameter 0 */
#define PHY_REG_BIST_MASK_DX1       0x900e00c4  /* Master PLL configuration parameter 1 */
#define PHY_REG_BIST_MASK_DX2       0x900e00c6  /* Master PLL configuration parameter 2 */
#define PHY_REG_BIST_MASK_DX3       0x900e00c8  /* Master PLL configuration parameter 3 */

#define PHY_REG_DBYTE_BASE          0x90020000  /* Base address for DBYTE slice CSRs */
#define PHY_REG_DBYTE_STRIDE        0x2000      /* DBYTE slice MMIO register stride */
#define PHY_REG_DBYTE_LCDL_DLY      0x0154      /* Slice LCDL delay register offset */
#define PHY_REG_DBYTE_LCDL_STATUS   0x016a      /* DBYTE lane 0 LCDL status CSR */
#define PHY_REG_DBYTE_LCDL_CTRL     0x1f6a      /* Slice DBYTE LCDL status/control CSR */
#define PHY_REG_DBYTE_CAL_TRIG      0x0176      /* DBYTE calibration strobe sequence register */

/* Mailbox Streaming & Status Registers */
#define PHY_REG_MAILBOX_STAT        0x90180008
#define PHY_REG_MAILBOX_STREAM      0x90180064
#define PHY_REG_MAILBOX_INT         0x90180066
#define PHY_REG_MAILBOX_STREAM_HI   0x90180068

/* ========================================================================== */
/* Synopsys PMU Message / Event Codes (Logged via pmu_log_msg)                */
/* Format: [31:16] Major Stage ID, [15:0] Minor Code / Argument Count         */
/* ========================================================================== */
#define PMU_MSG_GATE_TRAIN_PASS     0x01390000

/* LPDDR5 Specific Messages */

#endif /* _DWC_DDRPHY_PMU_REGS_H */
