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

/* Configuration Variables in DMEM */
#define PMU_DMEM_CLK_GATE_FLAG      0x01        /* Byte: Clock gate pulse enable flag */
#define PMU_DMEM_CAL_PARAM_05       0x05        /* Byte: Calibration parameter */
#define PMU_DMEM_DRAM_FREQ_OFF      0x06        /* Half: Clock in MHz offset */
#define PMU_DMEM_DRAM_CFG_FLAGS     0x0a        /* Half: DRAM config bitmask (bit 13 flag) */
#define PMU_DMEM_CAL_SELECT_FLAGS   0x0b        /* Byte: Calibration selector flags (bit 2) */
#define PMU_DMEM_TRAIN_MODE_14      0x14        /* Half: Training mode flags (bit 3) */
#define PMU_DMEM_POST_THRESH_12     0x12        /* Byte: Post mailbox threshold */
#define PMU_POST_CMD_THRESH_200     200         /* POST command dispatch threshold */
#define PMU_DMEM_LANE_MASK_1C       0x1c        /* Byte: Lane active mask */
#define PMU_DMEM_DQ_SWAP_MASK       0x101       /* Byte: DQ/DQS swap & polarity mask */
#define PMU_DMEM_CAL_TABLE_43C      0x43c       /* 2D calibration parameter table */
#define PMU_DMEM_CAL_TABLE_440      0x440       /* 2D calibration parameter table word 1 */
#define PMU_DMEM_CAL_MARKER_A       0x458       /* Calibration target marker A */
#define PMU_DMEM_CBT_STATE_45A      0x45a       /* CBT state latch */
#define PMU_DMEM_CBT_CAL_STAT_45C   0x45c       /* CBT calibration active status flag */
#define PMU_DMEM_CAL_MARKER_B       0x45e       /* Calibration target marker B */
#define PMU_DMEM_CBT_CONFIG_46A     0x46a       /* CBT configuration value */
#define PMU_DMEM_CBT_STEP_STAT_46B  0x46b       /* CBT step status flag */
#define PMU_DMEM_DESKEW_DELAY_47E   0x47e       /* Deskew delay parameter */
#define PMU_DMEM_SLICE_START        0xb68       /* Start byte lane / slice index */
#define PMU_DMEM_SLICE_END          0xb69       /* End byte lane / slice index */
#define PMU_DMEM_RANK_BOUNDARY      0xb6b       /* Rank boundary threshold */
#define PMU_DMEM_LANE_STATUS        0xb97       /* Lane status latch */
#define PMU_DMEM_DRAM_TYPE          0x08        /* Byte: DRAM type (2 = LPDDR5) */
#define PMU_DMEM_CAL_PARAM_25       0x25        /* Byte: Calibration parameter 0x25 */
#define PMU_DMEM_CAL_PARAM_40       0x40        /* Byte: Calibration parameter 0x40 */
#define PMU_DMEM_CAL_FLAG_62        0x62        /* Byte: Signed calibration flag */
#define PMU_DMEM_CAL_STATUS_E7      0xe7        /* Byte: Calibration status word */
#define PMU_DMEM_CAL_RANK           0xb66       /* Byte: Current rank index */
#define PMU_DMEM_CAL_BYTE           0xb67       /* Byte: Current byte lane index */
#define PMU_DMEM_CAL_STRUCT_PTR     0xb9c       /* Word: Pointer to calibration data structure */
#define PMU_DMEM_CAL_SAVE_42A       0x42a       /* Half: PHY register 0x90040022 save location */
#define PMU_DMEM_PARAM_TABLE_430    0x430       /* Base of calibration parameter array */
#define PMU_DMEM_SLICE_IDX_470      0x470       /* Word: Active slice/lane index */
#define PMU_DMEM_SLICE_START_B6A    0xb6a       /* Byte: Primary start slice index */
#define PMU_DMEM_SLICE_START_B6C    0xb6c       /* Byte: Secondary start slice index */
#define PMU_DMEM_SLICE_END_B6D      0xb6d       /* Byte: Secondary end slice index */
#define PHY_REG_CAL_STAT_022        0x90040022  /* PHY calibration status & control */
#define PMU_DMEM_CAL_TRACKER_D9C    0xd9c       /* Calibration tracker struct base */
#define PMU_DMEM_CAL_TRACKER_DBE    0xdbe       /* Calibration tracker status word (0x22) */
#define PMU_DMEM_CAL_NUM_SLICES_42C 0x42c       /* Byte: Total active PHY DBYTE slices */
#define PMU_DMEM_DQ_PIN_MAP_BASE    0x80000a7e  /* Base of DBYTE DQ pin swizzle/mapping table */
#define PMU_DQ_PIN_MAP_ENTRY_SIZE   18          /* Bytes per entry in DQ pin map table */
#define PMU_DMEM_SLICE_DLY_BA0      0xba0       /* DBYTE slice delay parameter array base */
#define PMU_DMEM_CAL_SHADOW_BC0     0xbc0       /* 16-bit calibration shadow table base */
#define PMU_FREQ_THRESHOLD_3200     3200        /* DRAM frequency threshold in MHz */
#define PMU_DMEM_METRIC_COEFF_X_420 0x420       /* Halfword: Delay metric quadratic coefficient for X */
#define PMU_DMEM_METRIC_COEFF_Y_422 0x422       /* Halfword: Delay metric quadratic coefficient for Y */
#define PMU_CAL_STROBE_TICKS        0x20        /* Strobe pulse delay duration in ticks */
#define PMU_LCDL_FINE_MASK          0x3f        /* Fine delay bitmask (6 bits) */
#define PMU_LCDL_COARSE_SHIFT       6           /* Coarse delay bit-shift */
#define PMU_DMEM_PARAM_8A           0x8a        /* Byte: DRAM status parameter */
#define PMU_PARAM_8A_MODE_MASK      0x03        /* Bits [1:0] mode mask */
#define PMU_DMEM_PARAM_8E           0x8e        /* Byte: DRAM channel / rank config parameter */
#define PMU_PARAM_8E_MODE_MASK      0x03        /* Bits [1:0] mode mask */
#define PMU_DMEM_PARAM_96           0x96        /* Byte: DRAM training config parameter */
#define PMU_PARAM_96_NIBBLE_MASK    0x0f        /* Lower 4-bit nibble mask */
#define PMU_LUT_2B_IDENTITY_E4      0xe4        /* 2-bit identity table (11 10 01 00b) */
#define PMU_LCDL_NOMINAL_MIDPOINT   0x40        /* Nominal LCDL delay midpoint (64) */
#define PMU_CFG_FLAG_BIT13_SHIFT    13          /* Bit 13 shift in DRAM config flags */
#define PMU_DMEM_PARAM_41C          0x41c       /* Word: Active slice CSR offset index */

#define PMU_CAL_DEFAULT_PARAM       0x00080008  /* Default calibration parameter pair */
#define PMU_LCDL_STEP_COARSE        0x20        /* Coarse delay adjustment step */
#define PMU_LCDL_STEP_OVERFLOW_MASK 0x03c0      /* Mask to test coarse step overflow */
#define PMU_LCDL_STEP_WRAP_OFFSET   0x0400      /* Delay line wrap-around compensation */

/* LPDDR5 Trained Delay Parameters Export (Host U-Boot reads 0x0a9b0026..0x0a9b004c) */

/* ========================================================================== */
/* Synopsys DDR PHY Internal APB Registers (ARC Address Space)                */
/* ========================================================================== */

/* Power / Clock / PLL Registers */
#define PHY_REG_CLK_GATE            0x90040040  /* Master PHY clock gate enable */
#define PHY_REG_CLK_GATE_STAT       0x90040042  /* Master clock gate acknowledge status */
#define PHY_REG_DESKEW_DLY0_44      0x90040044  /* Deskew delay tap 0 */
#define PHY_REG_DESKEW_DLY1_46      0x90040046  /* Deskew delay tap 1 */
#define PHY_REG_CLK_TIMING_CTRL     0x9004004e  /* Clock timing control */
#define PHY_REG_SLICE_STAT_0018     0x90040018  /* Slice / lane status register */
#define PHY_REG_PLL_CLK_CTRL        0x90040062  /* Master PLL & clock control */
#define PHY_REG_RESET_PULSE_0F2     0x900400f2  /* Soft reset pulse toggle */
#define PHY_REG_INIT_COMPLETE       0x90040120
#define PHY_REG_CAL_TRIG_146        0x90040146  /* Calibration strobe trigger */
#define PHY_REG_CAL_STAT_148        0x90040148  /* Calibration status & configuration */
#define PHY_REG_CBT_PLL_1F0         0x900401f0  /* CBT PLL control register */
#define PHY_REG_CMD_DISPATCH_238    0x90040238  /* Command dispatch strobe */
#define PHY_REG_CLK_GATE_E0A4       0x9005e0a4  /* Secondary clock gate toggle */
#define PHY_REG_CAL_STROBE_E140     0x9005e140  /* Secondary calibration strobe */
#define PHY_REG_PLL_CTRL            0x90040034
#define PHY_REG_PLL_STATUS          0x9007e034
#define PHY_REG_PUB_CTL             0x901801dc
#define PHY_REG_PHY_STATUS          0x900400c0

/* WCK Clock and Timing Calibration Registers (LPDDR5) */
#define PHY_REG_WCK_DELAY           0x90020146

/* Datapath Delay Line Registers (DX = Byte Lane 0..3) */

/* Receiver & Analog Calibration Registers */
#define PHY_REG_VREF_CTRL           0x90040082
#define PHY_REG_CFG_00E4            0x900400e4  /* Target PHY configuration register 0 */
#define PHY_REG_CFG_00E6            0x900400e6  /* Target PHY configuration register 1 */
#define PHY_REG_CBT_CTRL            0x90040144


#define PHY_REG_PLL_CFG_00C2        0x900e00c2  /* Master PLL configuration parameter 0 */
#define PHY_REG_PLL_CFG_00C4        0x900e00c4  /* Master PLL configuration parameter 1 */
#define PHY_REG_PLL_CFG_00C6        0x900e00c6  /* Master PLL configuration parameter 2 */
#define PHY_REG_PLL_CFG_00C8        0x900e00c8  /* Master PLL configuration parameter 3 */

#define PHY_REG_DBYTE_BASE          0x90020000  /* Base address for DBYTE slice CSRs */
#define PHY_REG_DBYTE_STRIDE        0x2000      /* DBYTE slice MMIO register stride */
#define PHY_REG_DBYTE_DLY_154       0x0154      /* Slice LCDL delay register offset */
#define PHY_REG_DBYTE_016A          0x016a      /* DBYTE lane 0 LCDL status CSR */
#define PHY_REG_DBYTE_1F6A          0x1f6a      /* Slice DBYTE LCDL status/control CSR */
#define PHY_REG_DBYTE_CAL_TRIG_176  0x0176      /* DBYTE calibration strobe sequence register */


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


/* PMU Mid-Eye Telemetry Sample Buffer Layout (DMEM) */
#define PMU_PIN_SAMPLE_BASE         0x0dc8      /* Base offset for slice 0 pin telemetry */
#define PMU_PIN_SAMPLE_STRIDE       132         /* Byte stride between consecutive pin records */

#endif /* _DWC_DDRPHY_PMU_REGS_H */
