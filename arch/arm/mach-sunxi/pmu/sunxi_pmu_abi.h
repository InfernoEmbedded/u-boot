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
typedef uint8_t   u8;
typedef uint16_t  u16;
typedef uint32_t  u32;
typedef uint64_t  u64;
typedef int8_t    s8;
typedef int16_t   s16;
typedef int32_t   s32;
typedef int64_t   s64;
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
 * @pstate_flags: Power-state flags and channel offset
 * @reserved_05: Reserved padding at offset 0x05
 * @dram_freq_mhz: DRAM data rate in MT/s (3600 for 1800 MHz clock)
 * @pll_ratio: PLL multiplication ratio
 * @reserved_09: Reserved padding at offset 0x09
 * @dfi_freq_ratio: DFI frequency divider ratio
 * @assert_code: PMU assert error code (read by SPL on failure)
 * @sequence_ctrl: Training stage bitmask (PMU_SEQ_*)
 * @post_threshold: Telemetry log filter threshold (0xff = disabled)
 * @reserved_13: Reserved padding at offset 0x13
 * @train_mode: Training mode / calibration state
 * @reserved_16: Reserved padding at offset 0x16
 *
 * Placed at the base of PMU DCCM (0x80000000 in ARC address space,
 * 0x0a9b0000 in host physical address space). Written by U-Boot SPL
 * prior to triggering PMU training execution.
 */
struct pmu_message_block {
	u8  dram_type;         /* 0x00: DRAM Type (3 = LPDDR5, 2 = LPDDR4) */
	u8  cfg_flags;         /* 0x01: Configuration flags */
	u16 reserved_02;       /* 0x02 */
	u8  pstate_flags;      /* 0x04: Power-state flags & channel offset */
	u8  reserved_05;       /* 0x05 */
	u16 dram_freq_mhz;     /* 0x06: DRAM data rate in MT/s (3600 for 1800 MHz clock) */
	u8  pll_ratio;         /* 0x08: PLL multiplication ratio */
	u8  reserved_09;       /* 0x09 */
	u16 dfi_freq_ratio;    /* 0x0a: DFI frequency divider ratio */
	u32 assert_code;       /* 0x0c: PMU assert error code (read by SPL on failure) */
	u16 sequence_ctrl;     /* 0x10: Training stage bitmask (PMU_SEQ_*) */
	u8  post_threshold;    /* 0x12: Telemetry log filter (0xff = disabled) */
	u8  reserved_13;       /* 0x13 */
	u16 train_mode;        /* 0x14: Training mode / calibration state */
	u16 reserved_16;       /* 0x16 */
};

/**
 * struct pmu_dmem_block - Contiguous chunk of non-zero PMU DMEM parameters
 * @offset: Byte offset in PMU DMEM
 * @count: Number of 16-bit halfwords in this block
 * @data: Pointer to array of 16-bit parameter values
 *
 * Splits the sparse 1,232-word DMEM parameter image into contiguous
 * non-zero blocks, eliminating unused zeros from the SPL binary.
 */
struct pmu_dmem_block {
	u16 offset;             /* Byte offset in PMU DMEM */
	u16 count;              /* Number of 16-bit halfwords */
	const u16 *data;        /* Pointer to parameter data */
};

#endif /* _SUNXI_PMU_ABI_H_ */
