/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Synopsys DesignWare DDR PHY PMU Firmware & SPL Hardware ABI Definitions
 * SoC: Allwinner A733 (Sun60i) / LPDDR5 PHY (Type 9)
 *
 * For detailed architecture, training theory, and memory map alignment,
 * see doc/board/allwinner/a733-pmu.rst.
 */

#ifndef _SUNXI_PMU_ABI_H_
#define _SUNXI_PMU_ABI_H_

#if defined(__KERNEL__) || defined(__UBOOT__)
#include <linux/types.h>
#include <linux/bitops.h>
#else
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifndef BIT
#define BIT(nr)			(1UL << (nr))
#endif

#ifndef _PMU_TYPES_DEFINED_
#define _PMU_TYPES_DEFINED_
typedef unsigned char      u8;
typedef unsigned short     u16;
typedef unsigned int       u32;
typedef unsigned long long u64;
typedef signed char        s8;
typedef signed short       s16;
typedef signed int         s32;
typedef signed long long   s64;
#endif
#endif

/*
 * Memory Map Base Addresses: Host ARM Physical vs ARC EM4 Microsequencer
 */
#define SUNXI_DRAM_PHY_CTRL_BASE    0x0aaa0000UL    /* PHY CSRs (APB Slave interface) */
#define SUNXI_DRAM_PHY_IMEM_BASE    0x0a9a0000UL    /* ARC ICCM: Instruction Memory (64 KB) */
#define SUNXI_DRAM_PHY_DMEM_BASE    0x0a9b0000UL    /* ARC DCCM: Data Memory (64 KB) */

/*
 * PMU Control & Mailbox Register Offsets (relative to SUNXI_DRAM_PHY_CTRL_BASE / 0x0aaa0000)
 * Note: The PHY APB slave interface is strictly 16-bit. All host accesses
 *       must use 16-bit readw / writew operations.
 */
#define PMU_REG_APB_MUX             0x0000  /* 0 = Host ARM owns APB, 1 = PMU ARC owns APB */
#define PMU_REG_POLL_STAT           0x0008  /* Bit 0: 1 = PMU training busy, 0 = complete / idle */
#define PMU_REG_HANDOFF_0           0x0060  /* Handshake handoff 0 */
#define PMU_REG_HANDOFF_1           0x0062  /* Handshake handoff 1 (write 1 to trigger, 0 to ack) */
#define PMU_REG_STATUS_LO           0x0064  /* PMU status low 16 bits */
#define PMU_REG_MAILBOX_INT         0x0066  /* Mailbox interrupt pulse / trigger */
#define PMU_REG_STATUS_HI           0x0068  /* PMU status high 16 bits */
#define PMU_REG_RESET               0x0132  /* Microsequencer reset & clock gating (1=run, 0=rst) */

/*
 * PMU Return Status Codes (read from PMU_REG_STATUS_LO | (PMU_REG_STATUS_HI << 16))
 */
#define PMU_STATUS_SUCCESS          0x00000007U /* Training completed successfully */
#define PMU_STATUS_FATAL_ERROR      0x000000ffU /* Training failed assertion / aborted */

/*
 * PMU SequenceCtrl Flags (DMEM offset 0x10)
 * Selects which calibration and training stages the PMU microsequencer executes.
 */
#define PMU_SEQ_DEV_INIT            BIT(0)  /* Bit 0:  Device init, PLL lock, CBT entry, ZQ cal */
#define PMU_SEQ_STAGE_VREF          BIT(1)  /* Bit 1:  Receiver Vref voltage centering */
#define PMU_SEQ_STAGE_DCD           BIT(2)  /* Bit 2:  Duty cycle distortion correction */
#define PMU_SEQ_STAGE_DQS           BIT(3)  /* Bit 3:  Read/Write DQS strobe timing centering */
#define PMU_SEQ_BIST_SEARCH_WIN     BIT(4)  /* Bit 4:  BIST test pattern search window setup */
#define PMU_SEQ_SEARCH_WIN          BIT(6)  /* Bit 6:  High-resolution 2D eye margin search */
#define PMU_SEQ_ZQ_CAL              BIT(7)  /* Bit 7:  ZQ pad output impedance & ODT calibration */
#define PMU_SEQ_DESKEW_SWEEP        BIT(8)  /* Bit 8:  Per-bit DQ deskew delay line tuning */
#define PMU_SEQ_MARGIN_STEP         BIT(9)  /* Bit 9:  Margin envelope boundary evaluation */
#define PMU_SEQ_LANE_WIN_SWEEP      BIT(10) /* Bit 10: Lane window sweep & cross-lane matrix scan */
#define PMU_SEQ_LPCA_INIT           BIT(12) /* Bit 12: Low-Power Command/Address training & init */

/*
 * Operating Modes:
 * - FAST_BOOT: Executes hardware device init + LPCA in 6.6ms, then restores cached CSRs.
 * - FULL_TRAIN: Complete multi-stage calibration sweep (2.42s).
 */
#define PMU_SEQ_FAST_BOOT           (PMU_SEQ_DEV_INIT | PMU_SEQ_LPCA_INIT)		/* 0x1001 */
#define PMU_SEQ_FULL_TRAIN          (PMU_SEQ_DEV_INIT | PMU_SEQ_STAGE_VREF | \
				     PMU_SEQ_STAGE_DCD | PMU_SEQ_STAGE_DQS | \
				     PMU_SEQ_BIST_SEARCH_WIN | PMU_SEQ_SEARCH_WIN | \
				     PMU_SEQ_MARGIN_STEP | PMU_SEQ_LPCA_INIT)	/* 0x125f */

/**
 * struct pmu_message_block - Host-to-PMU parameter handshake mailbox
 * @dram_type: DRAM memory type (3 = LPDDR5, 2 = LPDDR4)
 * @cfg_flags: Hardware configuration flags
 * @reserved_02: Reserved padding at offset 0x02
 * @pstate_flags: Power-state flags and channel offset (0x04)
 * @reserved_05: Reserved padding at offset 0x05
 * @dram_freq_mhz: DRAM data rate in MT/s (3600 for 1800 MHz clock)
 * @pll_ratio: PLL multiplication ratio (0x08)
 * @reserved_09: Reserved padding at offset 0x09
 * @dfi_freq_ratio: DFI frequency divider ratio (0x0a)
 * @assert_code: PMU assert error code (read by SPL on failure)
 * @sequence_ctrl: Training stage bitmask (PMU_SEQ_*)
 * @post_threshold: Telemetry log filter threshold (0xff = disabled)
 * @reserved_13: Reserved padding at offset 0x13
 * @train_mode: Training mode / calibration state (0x14)
 * @cal_ctrl_16: Calibration control flags (0x16)
 * @reserved_18: Reserved padding at offsets 0x18..0x23
 * @ch0_dq_width: Channel 0 total DQ bit width (16)
 * @ch0_rank_en: Channel 0 active rank bitmask (0x03 = dual-rank)
 * @reserved_26: Reserved padding at offsets 0x26..0x3e
 * @ch1_dq_width: Channel 1 total DQ bit width (16)
 * @ch1_rank_en: Channel 1 active rank bitmask (0x03 = dual-rank)
 * @reserved_41: Reserved padding at offsets 0x41..0x59
 * @mr1: Per-slice JEDEC MR1 values (clock and WL set config)
 * @mr2: Per-slice JEDEC MR2 values (RL and nWR config)
 * @mr3: Per-slice JEDEC MR3 values (PDDS, DBI, and bank org)
 * @mr10: Per-slice JEDEC MR10 values (RDQS preamble/postamble)
 * @mr11: Per-slice JEDEC MR11 values (CA and DQ ODT)
 * @mr12: Per-slice JEDEC MR12 values (CA Vref)
 * @mr13: Per-slice JEDEC MR13 values
 * @mr14: Per-slice JEDEC MR14 values (DQ[7:0] Vref)
 * @mr15: Per-slice JEDEC MR15 values (DQ[15:8] Vref)
 * @mr16: Per-slice JEDEC MR16 values (FSP-WR/OP and CBT)
 * @mr17: Per-slice JEDEC MR17 values (CK/CA ODT)
 * @mr18: Per-slice JEDEC MR18 values (WCK ODT and CK-to-WCK sync)
 * @mr19: Per-slice JEDEC MR19 values (DVFSC and WCK2DQ osc)
 * @mr20: Per-slice JEDEC MR20 values (RDQS mode)
 * @mr21: Per-slice JEDEC MR21 values
 * @mr22: Per-slice JEDEC MR22 values
 * @mr24: Per-slice JEDEC MR24 values (DFE)
 * @mr25: Per-slice JEDEC MR25 values
 * @mr26: Per-slice JEDEC MR26 values
 * @mr27: Per-slice JEDEC MR27 values
 * @mr28: Per-slice JEDEC MR28 values (ZQ reset/stop/interval)
 * @mr30: Per-slice JEDEC MR30 values (DCA)
 * @mr31: Per-slice JEDEC MR31 values
 * @mr32: Per-slice JEDEC MR32 values
 * @mr33: Per-slice JEDEC MR33 values
 * @mr34: Per-slice JEDEC MR34 values
 * @mr37: Per-slice JEDEC MR37 values (WCK2DQI interval)
 * @mr40: Per-slice JEDEC MR40 values (WCK2DQO interval)
 * @mr41: Per-slice JEDEC MR41 values (NT-ODT and PPRE)
 * @reserved_ce: Reserved padding at offsets 0xce..0xe7
 * @pll_gate_ch: Per-channel PLL clock gate thresholds (0xe8..0xeb)
 * @reserved_ec: Reserved padding at offsets 0xec..0xff
 * @dq_swap_en: Per-bit DQ pin swizzle enable flag (0x100)
 * @lane_err_mask: Per-lane deskew error status mask (0x101)
 * @train_feature_mask: Training feature override bitmask (0x102)
 * @ca_vref_rank1: Initial CA Vref DAC setting for Rank 1 (0x103)
 * @ca_vref_rank0: Initial CA Vref DAC setting for Rank 0 (0x104)
 * @diff_tap_pair: Differential tap pair offset (0x105)
 *
 * Placed at the base of PMU DCCM (0x80000000 in ARC address space,
 * 0x0a9b0000 in host physical address space). Written by U-Boot SPL
 * prior to triggering PMU training execution.
 */
struct pmu_message_block {
	u8  dram_type;          /* 0x000 */
	u8  cfg_flags;          /* 0x001 */
	u16 reserved_02;        /* 0x002 */
	u8  pstate_flags;       /* 0x004 */
	u8  reserved_05;        /* 0x005 */
	u16 dram_freq_mhz;      /* 0x006 */
	u8  pll_ratio;          /* 0x008 */
	u8  reserved_09;        /* 0x009 */
	u16 dfi_freq_ratio;     /* 0x00a */
	u32 assert_code;        /* 0x00c */
	u16 sequence_ctrl;      /* 0x010 */
	u8  post_threshold;     /* 0x012 */
	u8  reserved_13;        /* 0x013 */
	u16 train_mode;         /* 0x014 */
	u16 cal_ctrl_16;        /* 0x016 */
	u8  reserved_18[12];    /* 0x018..0x023 */
	u8  ch0_dq_width;       /* 0x024 */
	u8  ch0_rank_en;        /* 0x025 */
	u8  reserved_26[25];    /* 0x026..0x03e */
	u8  ch1_dq_width;       /* 0x03f */
	u8  ch1_rank_en;        /* 0x040 */
	u8  reserved_41[25];    /* 0x041..0x059 */
	u8  mr1[4];             /* 0x05a */
	u8  mr2[4];             /* 0x05e */
	u8  mr3[4];             /* 0x062 */
	u8  mr10[4];            /* 0x066 */
	u8  mr11[4];            /* 0x06a */
	u8  mr12[4];            /* 0x06e */
	u8  mr13[4];            /* 0x072 */
	u8  mr14[4];            /* 0x076 */
	u8  mr15[4];            /* 0x07a */
	u8  mr16[4];            /* 0x07e */
	u8  mr17[4];            /* 0x082 */
	u8  mr18[4];            /* 0x086 */
	u8  mr19[4];            /* 0x08a */
	u8  mr20[4];            /* 0x08e */
	u8  mr21[4];            /* 0x092 */
	u8  mr22[4];            /* 0x096 */
	u8  mr24[4];            /* 0x09a */
	u8  mr25[4];            /* 0x09e */
	u8  mr26[4];            /* 0x0a2 */
	u8  mr27[4];            /* 0x0a6 */
	u8  mr28[4];            /* 0x0aa */
	u8  mr30[4];            /* 0x0ae */
	u8  mr31[4];            /* 0x0b2 */
	u8  mr32[4];            /* 0x0b6 */
	u8  mr33[4];            /* 0x0ba */
	u8  mr34[4];            /* 0x0be */
	u8  mr37[4];            /* 0x0c2 */
	u8  mr40[4];            /* 0x0c6 */
	u8  mr41[4];            /* 0x0ca */
	u8  reserved_ce[26];    /* 0x0ce..0x0e7 */
	u16 pll_gate_ch[2];     /* 0x0e8..0x0eb */
	u8  reserved_ec[20];    /* 0x0ec..0x0ff */
	u8  dq_swap_en;         /* 0x100 */
	u8  lane_err_mask;      /* 0x101 */
	u8  train_feature_mask; /* 0x102 */
	u8  ca_vref_rank1;      /* 0x103 */
	u8  ca_vref_rank0;      /* 0x104 */
	u8  diff_tap_pair;      /* 0x105 */
};

#endif /* _SUNXI_PMU_ABI_H_ */
