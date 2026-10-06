/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Synopsys DesignWare DDR PHY PMU Firmware & SPL Hardware ABI Definitions
 * SoC: Allwinner A733 (Sun60i) / LPDDR5 PHY (Type 9)
 *
 * =============================================================================
 * LAYPERSON'S GUIDE TO THE PMU (PROGRAMMABLE MICROCODE UNIT) & DRAM TRAINING
 * =============================================================================
 *
 * 1. Why does high-speed RAM require "Training"?
 * -----------------------------------------------
 * Modern LPDDR5 memory operates at blistering speeds—in our board's case,
 * 1800 MHz clock yielding 3600 MT/s (million transfers per second). At these
 * frequencies, each individual data bit (DQ) is valid for only ~277 picoseconds
 * (the "data eye").
 *
 * Because copper PCB traces have slightly different physical lengths, and silicon
 * transistors inside the SoC and DRAM vary with manufacturing process, operating
 * voltage, and temperature (PVT), signals arrive at slightly different times.
 * If the memory controller sampled data with fixed delays, electrical noise and
 * clock skew would corrupt reads and writes instantly.
 *
 * "Training" is the process of electronically calibrating:
 *   a) Timing Deskew: Adjusting per-bit programmable delay lines (LCDLs) so all
 *      bits (DQ0..DQ7, DM) and byte strobes (DQS) arrive at the exact same
 *      fraction of a nanosecond.
 *   b) Vref Voltage Centering: Adjusting internal DAC reference voltages to place
 *      the logic 0/1 decision threshold exactly in the vertical center of the eye.
 *   c) Command/Address (CA) Training: Synchronizing command lines with clock.
 *   d) ZQ Impedance Calibration: Matching output driver and on-die termination
 *      (ODT) impedance against an external 240-ohm precision calibration resistor.
 *
 * 2. Why is there a dedicated ARC Processor inside the DDR PHY?
 * -------------------------------------------------------------
 * Performing millions of microsecond-precision analog delay line measurements
 * using the main host CPU (ARM Cortex-A76 / A55) would require millions of slow
 * bus round-trips and complex OS scaffolding.
 *
 * To solve this, Synopsys integrated a dedicated, self-contained 32-bit RISC
 * microcontroller core—an ARC EM4—directly inside the PHY silicon. This core is
 * called the PMU (Programmable Microcode Unit).
 * The PMU has direct, cycle-accurate access to the analog delay lines and PHY
 * registers, allowing it to sweep timing envelopes, evaluate BIST test patterns,
 * and lock in optimal values autonomously.
 *
 * 3. Execution Lifecycle: How SPL and PMU coordinate
 * --------------------------------------------------
 *   Step 1: Host owns APB bus (PMU_REG_APB_MUX = 0).
 *           U-Boot SPL powers up memory rails via PMIC (VDD2=1.08V, VDDQ=0.56V,
 *           VDD1=1.8V) and enables CCU clocks (PLL_DDR at 1800MHz).
 *   Step 2: Host writes PMU training firmware code into PHY IMEM (0x0a9a0000).
 *   Step 3: Host writes hardware parameters into PHY DMEM (0x0a9b0000), including
 *           SequenceCtrl (DMEM 0x10) selecting desired training stages.
 *   Step 4: Host hands APB bus control over to the PMU (PMU_REG_APB_MUX = 1).
 *   Step 5: Host releases PMU reset and sends an interrupt pulse (PMU_REG_RESET,
 *           PMU_REG_HANDOFF_1, PMU_REG_MAILBOX_INT).
 *   Step 6: PMU ARC EM4 boots from vector 0 (pmu_start), checks SequenceCtrl,
 *           runs calibration stages, writes final trained values to PHY registers,
 *           and writes completion status 0x00000007 to the mailbox CSRs.
 *   Step 7: PMU executes ARC 'flag 1' instruction to enter low-power sleep.
 *   Step 8: Host detects PMU poll bit drop (PMU_REG_POLL_STAT), verifies success
 *           status (PMU_STATUS_SUCCESS = 0x7), reclaims APB bus (PMU_REG_APB_MUX = 0),
 *           and hands off uMCTL2 to normal memory controller operation.
 *
 * =============================================================================
 * ALLWINNER A733 REFERENCE MANUAL REGISTER & MEMORY MAP ALIGNMENT
 * =============================================================================
 *
 * The Allwinner A733 User Manual documents the memory subsystem architecture
 * under Section 3.2 (System Memory Map), Section 4.1 (CCU), Section 4.2 (PRCM),
 * Section 5.1 (CPU Subsystem Control), and Section 11.1 (SDRAM Controller DRAMC):
 *
 * Peripheral Module           Host ARM Physical   ARC Internal    Description / Ref Manual Section
 * --------------------------------------------------------------------------------------------------
 * MEMC0 Subsystem Base        0x0A000000          -               17MB Memory Subsystem Window (Sec 3.2)
 * SMC0 / SMC1                 0x0A000000/010000   -               Security Memory Controllers (64KB each)
 * DRAMC Common Base           0x0A020000          -               64KB DRAMC Common registers (Sec 3.2)
 *   DRAMC MAER Reg 0          0x0A021800          -               Memory Access Enable / Firewall 0
 *   DRAMC MAER Reg 1          0x0A021808          -               Memory Access Enable / Firewall 1
 * CPU AXI2HIF0 / AXI2HIF1     0x0A030000/040000   -               CPU AXI to Host Interface channels
 * DRAM Controller 0 (uMCTL2)  0x0A100000          -               4MB Channel 0 uMCTL2 Window (Sec 3.2)
 *   uMCTL2 Ch0 CSR Base       0x0A110000          -               Synopsys uMCTL2 Core Ch0 registers
 * DRAM Controller 1 (uMCTL2)  0x0A500000          -               4MB Channel 1 uMCTL2 Window (Sec 3.2)
 *   uMCTL2 Ch1 CSR Base       0x0A510000          -               Synopsys uMCTL2 Core Ch1 registers
 * DRAM PHY Base               0x0A900000          -               8MB Synopsys DDR PHY Window (Sec 3.2)
 *   PMU Instruction Memory    0x0A9A0000          0x00000000      64KB PMU ICCM (Instruction Memory)
 *   PMU Data Memory           0x0A9B0000          0x80000000      64KB PMU DCCM (Data Memory)
 *   PMU CSRs / APB Slave      0x0AAA0000          0x90180000      64KB PHY Control & Status Registers
 *
 * Related System Controllers (Allwinner A733 Reference Manual):
 *   CCU Base                  0x02002000                          Clock Controller Unit (Sec 4.1.4)
 *     PLL_DDR_CTRL_REG        0x02002020 (CCU + 0x0020)           DRAM Subsystem PLL control (Sec 4.1.6.1)
 *     MBUS_CLK_REG            0x02002588 (CCU + 0x0588)           MBUS Clock register (Sec 4.1.6.72)
 *     DRAM0_CLK_REG           0x02002C00 (CCU + 0x0C00)           DRAM0 Clock register (Sec 4.1.6.123)
 *     DRAM0_BGR_REG           0x02002C0C (CCU + 0x0C0C)           DRAM0 Bus Gating Reset (Sec 4.1.6.124)
 *   CPU_SUBSYS_CTRL Base      0x08000000                          CPU Subsystem Control (Sec 5.1.5.1)
 *     CPU_DA_DDR_CTRL_REG     0x08000200 (CPU + 0x0200)           CPU Direct Access DDR Mux (Sec 5.1.6.5)
 *                                                                 Bit 0: 0 = NSI Interconnect, 1 = AXI2HIF
 *   STBY_PRCM Base            0x07010000                          Standby PRCM (Sec 4.2.4)
 *     VCC_DRAM_ISO_REG        0x07010250 (PRCM + 0x0250)          DRAM Power Isolation (Bit 0 = un-isolate)
 *     RTC_BGR_REG             0x0701020C (PRCM + 0x020C)          RTC Bus Gating Reset register
 *   RTC Base                  0x07090000                          Real-Time Clock / Retention registers
 *     RTC_GP_DATA_REG0        0x07090100 (RTC + 0x0100)           General Purpose Retention Data Reg 0
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
#define PMU_REG_APB_MUX             0x0000  /* 0 = Host ARM owns APB bus, 1 = PMU ARC owns APB bus */
#define PMU_REG_POLL_STAT           0x0008  /* Bit 0: 1 = PMU training busy, 0 = complete / idle */
#define PMU_REG_HANDOFF_0           0x0060  /* Handshake handoff 0 */
#define PMU_REG_HANDOFF_1           0x0062  /* Handshake handoff 1 (write 1 to trigger, 0 to ack) */
#define PMU_REG_STATUS_LO           0x0064  /* PMU status low 16 bits */
#define PMU_REG_MAILBOX_INT         0x0066  /* Mailbox interrupt pulse / trigger */
#define PMU_REG_STATUS_HI           0x0068  /* PMU status high 16 bits */
#define PMU_REG_RESET               0x0132  /* PMU microsequencer reset & clock gating (1=run, 0=rst) */

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
#define PMU_SEQ_STAGE_D804          BIT(1)  /* Bit 1:  Receiver Vref voltage centering */
#define PMU_SEQ_STAGE_D7BC          BIT(2)  /* Bit 2:  Duty cycle distortion correction */
#define PMU_SEQ_STAGE_D720          BIT(3)  /* Bit 3:  Read/Write DQS strobe timing centering */
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
#define PMU_SEQ_FULL_TRAIN          (PMU_SEQ_DEV_INIT | PMU_SEQ_STAGE_D804 | \
				     PMU_SEQ_STAGE_D7BC | PMU_SEQ_STAGE_D720 | \
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
 * Splits the sparse 1,232-word vendor DMEM parameter image into contiguous
 * non-zero blocks, eliminating unused zeros from the SPL binary.
 */
struct pmu_dmem_block {
	u16 offset;             /* Byte offset in PMU DMEM */
	u16 count;              /* Number of 16-bit halfwords */
	const u16 *data;        /* Pointer to parameter data */
};

#endif /* _SUNXI_PMU_ABI_H_ */
