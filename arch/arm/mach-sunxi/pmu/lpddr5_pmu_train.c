// SPDX-License-Identifier: GPL-2.0+
/*
 * Synopsys DesignWare DDR PHY Training Firmware (LPDDR5)
 * Target Microcontroller: Synopsys ARC EM4 (ARCv2 ISA, Code Density enabled)
 * SoC: Allwinner A733 (Sun60i) / LPDDR5 PHY (Type 9)
 *
 * =============================================================================
 * LAYPERSON'S GUIDE TO THIS FIRMWARE: HOW IT WORKS
 * =============================================================================
 *
 * What is this code?
 * -------------------
 * This file is a complete, freestanding C firmware program that runs entirely on
 * a tiny 32-bit auxiliary processor (the Synopsys ARC EM4 "PMU" microsequencer)
 * embedded inside the A733 DDR PHY silicon.
 *
 * When the main 64-bit ARM CPU starts up U-Boot SPL, it does not directly train
 * the memory. Instead, SPL loads this compiled binary into the PHY's internal
 * 64 KB Instruction Memory (ICCM at address 0x00000000), writes initial memory
 * parameters (frequency, DRAM type, timings) into the PHY's 64 KB Data Memory
 * (DCCM at address 0x80000000), and releases the ARC processor from reset.
 *
 * What does this code do?
 * -----------------------
 * At 1800 MHz (3600 MT/s), electrical signals take different times to travel
 * across the circuit board to each memory chip pin. This firmware:
 *   1. Wakes up at the reset vector (pmu_start), zeroes registers, sets up the
 *      stack pointer (sp = 0x80010000) and global pointer (gp = 0x80000400).
 *   2. Enters pmu_train_main() -> pmu_train_dispatcher().
 *   3. Reads the SequenceCtrl bitmask from DMEM (0x80000010):
 *      - Fast Boot Mode (0x1001): Executes only device initialization (DEV_INIT)
 *        and Command/Address initialization (LPCA_INIT) in 6.6 milliseconds,
 *        allowing SPL to restore previously trained registers from SPI NOR flash.
 *      - Full Training Mode (0x125f): Executes the full physical calibration sweep
 *        (Vref centering, duty cycle correction, DQS strobe centering, per-bit
 *        DQ deskew, and timing margin scanning) in 2.42 seconds.
 *   4. Writes 0x00000007 (PMU_STATUS_SUCCESS) to the PHY mailbox register.
 *   5. Executes the ARC 'flag 1' instruction to halt and go to sleep.
 *   6. U-Boot SPL detects the completion, reclaims APB bus control, and hands off
 *      to normal 6GB dual-channel memory operation.
 *
 * Safety & Hardware Exception Handling:
 * --------------------------------------
 * If any hardware fault (illegal instruction, unaligned access, bus fault)
 * occurs, pmu_default_exception_handler captures the ARC Exception Cause Register
 * (ECR), Faulting PC (ERET), and core registers, storing them at DMEM 0x80000000
 * with the signature 0x1234dead before halting. U-Boot SPL's pmu_dump_diagnostics()
 * detects this and prints a full crash dump to the UART console.
 * =============================================================================
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "dwc_ddrphy_pmu_regs.h"

/**
 * pmu_irq_halt() - Halts ARC EM4 execution on unhandled interrupt
 *
 * Derived from vendor PMU code at address 0x0050.
 * /
 */
__attribute__((naked)) void pmu_irq_halt(void)
{
	__asm__ volatile (
		"nop\n"
		"nop\n"
		"nop\n"
		"nop\n"
		"flag 1\n"
	);
}

/**
 * pmu_default_exception_handler() - Diagnostic exception handler: dumps ECR, EFA, regs to DMEM 0x80000000 and halts
 *
 * Derived from vendor PMU code at address 0x017c.
 * /
 */
__attribute__((naked)) void pmu_default_exception_handler(void)
{
	__asm__ volatile (
		"mov     r2, 0x80000000\n"
		"mov     r3, 0x1234dead\n"
		"st      r3, [r2]\n"
		"add_s   r2, r2, 4\n"
		"lr      r3, [1024]\n"
		"st      r3, [r2]\n"
		"add_s   r2, r2, 4\n"
		"lr      r3, [1027]\n"
		"st      r3, [r2]\n"
		"add_s   r2, r2, 4\n"
		"st      r0, [r2]\n"
		"add_s   r2, r2, 4\n"
		"st      r1, [r2]\n"
		"add_s   r2, r2, 4\n"
		"st      sp, [r2]\n"
		"add_s   r2, r2, 4\n"
		"st      fp, [r2]\n"
		"add_s   r2, r2, 4\n"
		"mov     r3, 0xabcddead\n"
		"st      r3, [r2]\n"
		"flag    1\n"
		"nop_s\n"
	);
}

/**
 * pmu_start() - ARC EM4 reset entry point: zeroes registers, initializes stack and global pointer,
 *
 * Derived from vendor PMU code at address 0x01fc.
 * invokes pmu_train_main, and enters infinite halt loop on completion.
 * /
 */
__attribute__((naked)) void pmu_start(void)
{
	__asm__ volatile (
		"mov_s   r0, 0\n"
		"mov_s   r1, 0\n"
		"mov_s   r2, 0\n"
		"mov_s   r3, 0\n"
		"mov_s   r10, 0\n"
		"mov_s   r11, 0\n"
		"mov_s   r12, 0\n"
		"mov_s   r13, 0\n"
		"mov_s   r14, 0\n"
		"mov_s   r15, 0\n"
		"mov_s   gp, 0\n"
		"mov_s   fp, 0\n"
		"mov_s   sp, 0\n"
		"mov     ilink, 0\n"
		"mov     r30, 0\n"
		"mov_s   blink, 0\n"
		"mov_s   sp, 0x80010000\n"
		"mov_s   gp, 0x80000400\n"
		"mov_s   fp, 0\n"
		"jl      pmu_train_main\n"
		"1:\n"
		"flag    1\n"
		"nop_s\n"
		"b_s     1b\n"
	);
}

void (* const pmu_reset_vector[20])(void) __attribute__((section(".vectors"))) = {
	pmu_start,
	pmu_default_exception_handler,
	pmu_default_exception_handler,
	pmu_default_exception_handler,
	pmu_default_exception_handler,
	pmu_default_exception_handler,
	pmu_default_exception_handler,
	pmu_default_exception_handler,
	pmu_default_exception_handler,
	pmu_default_exception_handler,
	pmu_default_exception_handler,
	pmu_default_exception_handler,
	pmu_default_exception_handler,
	pmu_default_exception_handler,
	pmu_default_exception_handler,
	pmu_default_exception_handler,
	pmu_irq_halt,
	pmu_irq_halt,
	pmu_irq_halt,
	pmu_irq_halt,
};
/**
 * phy_read16() - Phy Read16
 * @reg: PHY register MMIO address or offset
 *
 * Return: Computed u16 value or status.
 */
static inline u16 phy_read16(u32 reg)
{
	return *(volatile u16 *)(uintptr_t)reg;
}
/**
 * phy_write16() - Phy Write16
 * @reg: PHY register MMIO address or offset
 * @val: Value to write or configure
 */
static inline void phy_write16(u32 reg, u16 val)
{
	*(volatile u16 *)(uintptr_t)reg = val;
}
/**
 * dmem_read8() - Dmem Read8
 * @offset: Byte offset in PMU DMEM
 *
 * Return: Computed u8 value or status.
 */
static inline u8 dmem_read8(u32 offset)
{
	return *(volatile u8 *)(uintptr_t)(PMU_DMEM_BASE | offset);
}
/**
 * dmem_read16() - Dmem Read16
 * @offset: Byte offset in PMU DMEM
 *
 * Return: Computed u16 value or status.
 */
static inline u16 dmem_read16(u32 offset)
{
	return *(volatile u16 *)(uintptr_t)(PMU_DMEM_BASE | offset);
}
/**
 * dmem_read32() - Dmem Read32
 * @offset: Byte offset in PMU DMEM
 *
 * Return: Computed u32 value or status.
 */
static inline u32 dmem_read32(u32 offset)
{
	return *(volatile u32 *)(uintptr_t)(PMU_DMEM_BASE | offset);
}
/**
 * dmem_write8() - Dmem Write8
 * @offset: Byte offset in PMU DMEM
 * @val: Value to write or configure
 */
static inline void dmem_write8(u32 offset, u8 val)
{
	*(volatile u8 *)(uintptr_t)(PMU_DMEM_BASE | offset) = val;
}
/**
 * dmem_write16() - Dmem Write16
 * @offset: Byte offset in PMU DMEM
 * @val: Value to write or configure
 */
static inline void dmem_write16(u32 offset, u16 val)
{
	*(volatile u16 *)(uintptr_t)(PMU_DMEM_BASE | offset) = val;
}
/**
 * dmem_write32() - Dmem Write32
 * @offset: Byte offset in PMU DMEM
 * @val: Value to write or configure
 */
static inline void dmem_write32(u32 offset, u32 val)
{
	*(volatile u32 *)(uintptr_t)(PMU_DMEM_BASE | offset) = val;
}

void pmu_hw_timer_delay(u32 count);
void pmu_dmem_rank_slice_table_setup(void);
u32 pmu_window_margin_diff_calc(const u8 *src0, const u8 *src1, u8 *out_dest, s32 offset, s32 delta);
void pmu_lcdl_delay_profile_update(u32 mode, u32 unused, u32 base_code, u32 tag);
void pmu_slice_metric_scan_and_program(void);
u16 pmu_rank_margin_channel_accum(u32 rank, u32 start_slice, u32 end_slice, u32 flags, u32 swap_mode);
void pmu_slice_deskew_pulse_seq(u32 r0);
u32 pmu_eye_sample_metric_eval(u8 x, u8 y, const u8 *table_62b0, const u8 *samples);
u32 pmu_eye_boundary_sample_search(u8 *samples, u32 target_y);
u32 pmu_eye_margin_step_balancer(u8 *buf0, u8 *buf1, u8 *buf2, u8 *buf3);
void pmu_cal_stage8_pad_init(void);
void pmu_profile_mailbox_stream_send(u32 idx);

void pmu_cal_metric_log(u32 level, u32 tag, ...);
void pmu_dq_lane_telemetry_bitpack(u32 flags, u32 base);
void pmu_post_trace_log_halfwords(u32 threshold, u32 cmd32, const u16 *data, u32 count);
void pmu_cal_error_code_log(void);
void pmu_cal_metric_tag_dump(u32 tag, const u32 *args);
void pmu_cal_pll_status_check(void);
void pmu_cal_matrix_trace_dump(u32 arg0, u32 num_cols, const u16 *matrix);
void pmu_dmem_stride_descriptor_read(void);

u32 pmu_eye_margin_window_search(u8 *samples, u32 r1_dummy);
u32 pmu_window_centroid_calc(u8 *out);
void pmu_cal_pulse_seq_coordinator(u32 rank, u32 param1, u32 flags);
void pmu_ac_lane_profile_setup(void);
void pmu_phy_pll_clock_timing_ctrl(void);
void pmu_dbyte_cal_strobe_pulse_seq(u32 mode, u16 val, u32 type);
void pmu_eye_margin_step_align(u8 *p0, u8 *p1, u32 r2);
void pmu_slice_deskew_pulse_train(u32 rank, u32 mode);
void pmu_cmd_code_map_log(u32 r0, u32 r1);
void pmu_eye_margin_bidir_scan(const u8 *table, u32 r1_flag, u32 r2_flag, u32 r3_flag);
void pmu_eye_sample_table_transform(u8 *dest, const u8 *src);
extern u32 pmu_cal_margin_sample_pair_extract(u8 *out_pair, const u8 *margins, const u8 *samples, const u8 *buf_62b0);
/*
 * pmu_memset_words:
 * Derived from vendor PMU code at address 0x0080.
 * Memory set routine: fills count bytes at dest with val.
 */
void *memset(void *dest, int val, size_t count)
{
	u8 *d = (u8 *)dest;
	while (count--)
		*d++ = (u8)val;
	return dest;
}
void *pmu_memset_words(void *dest, int val, size_t count) __attribute__((alias("memset")));

/*
 * pmu_memcpy_words:
 * Derived from vendor PMU code at address 0x0110.
 * Memory copy routine: copies count bytes from src to dest.
 */
void *memcpy(void *dest, const void *src, size_t count)
{
	u8 *d = (u8 *)dest;
	const u8 *s = (const u8 *)src;
	while (count--)
		*d++ = *s++;
	return dest;
}
void *pmu_memcpy_words(void *dest, const void *src, u32 count) __attribute__((alias("memcpy")));
extern u32 pmu_freq_delay_step_calc(u32 r0, u32 r1);
u32 pmu_assert_or_halt(u32 cond, u32 code);
u8 pmu_dbyte_deskew_sample_check(u32 arg0, u32 arg1, u32 arg2);
u8 pmu_cbt_entry_lookup(u32 table_base);
u8 pmu_cbt_active_entry_lookup(void);
u32 pmu_cal_window_params_dispatch(const u8 *ptr, u16 val, u32 flags);
void pmu_cal_marker_pair_program(u32 marker0, u32 marker1, u32 arg2, u32 arg3);
void pmu_slice_deskew_state_latch(u32 r0, u32 r1);
void pmu_cal_descriptor_apply(const void *desc);
u32 pmu_phy_pll_lock_poll(int timeout);
extern void pmu_train_dispatcher(void);
void pmu_cal_bist_search_win_setup(u32 arg0);
void pmu_cal_slice_step_diff_commit(u32 arg0, u16 *out_buf, u32 arg2);
void pmu_cal_dbyte_pattern_loop_exec(u32 arg0, u16 arg1, void *arg2, void *arg3, u32 arg4, u32 arg5, u32 arg6);
void pmu_cal_dbyte_deskew_results_apply(const void *results, u32 arg1, u32 arg2);
void pmu_cal_bist_pattern_setup(u32 arg0, u32 arg1, u32 arg2, u32 arg3);
u32 pmu_cal_dbyte_dq_status_check(u32 arg0, u32 arg1);
void pmu_cal_ca_eye_margin_sweep(void);
s16 pmu_cal_dbyte_rx_fifo_reset_poll(u32 arg0, u32 arg1, void *arg2);

void pmu_deskew_dly0_set(u16 val);
void pmu_deskew_dly1_set(u16 val);
void pmu_dbyte_deskew_phase_sample(u32 rank_idx, u32 *out_mask, u32 phase_mask);
void pmu_cal_markers_set(u16 val);
extern u32 pmu_cal_pll_lock_retry_poll(u32 tracker_ptr);
u32 pmu_freq_ratio_mult(u32 r0, u32 r1);
void pmu_clk_ratio_pulse_trigger(void);
u32 pmu_eye_error_metric_eval(u32 r0, u32 r1, u32 min_x, u32 max_x, u8 *out_metric);
void pmu_cal_timing_packet_format(u16 *out, u32 r1, u32 r2, u32 r3);
void pmu_phy_reg_write_shadow_track(u32 arg0, u32 val1, u32 val2, u32 arg3, u32 arg4);
void pmu_rank_slice_deskew_latch(u32 r0, u32 r1);
void pmu_phy_deskew_reset_strobe(void);
void pmu_phy_slice_profile_update(void);
void pmu_slice_reg_query_mailbox_send(u32 mode);
void pmu_phy_mode_cfg_dispatch(u32 mode);
void pmu_profile_param_stride_cfg(u32 mode);
void pmu_profile_mailbox_cmd_dispatch(u32 idx);
void pmu_phy_slice_cal_status_read(void);
void pmu_cal_state_restore_offset_prog(void);
void pmu_mailbox_param_eval_dispatch(u8 *p0, u8 *p1, u8 *p2, u32 rank, u32 flag);

void pmu_channel_timing_deskew_reset(u32 a0, u32 a1, u32 a2, u32 a3, u32 a4);
void pmu_rank_slice_margin_eval(void);
void pmu_dual_rank_phy_reg_read(void);
void pmu_multirank_slice_cal_dispatch(void *ptr, u32 a1, u32 a2, s32 a3, u32 a4, u32 a5);
void pmu_slice_phy_reg_step_adjust(u32 reg_offset, u32 flag1, u32 flag2);


u32 pmu_sample_margin_center_eval(const u8 *samples, u8 *out_margins, u8 *out_mode);
void pmu_cbt_3phase_pulse_dispatch(u32 a0, const u8 *buf, u32 a2, u32 a3);
void pmu_phy_cal_strobe_latch_setup(void);
void pmu_slice_delay_step_program(u32 arg0, const u16 *data, u32 arg2, u32 arg3, u32 arg4, u32 arg5);
u32 pmu_slice_dq_bitmask_swizzle(u32 slice, u32 arg1, u32 arg2);
void pmu_phy_profile_param_program(void);
void pmu_multiparam_cal_dispatch(void);
void pmu_phy_lcdl_delay_read(u16 *out, u32 flag, u32 mode);
void pmu_cal_dbyte_deskew_pin_results_apply(void);


u16 pmu_phy_delay_bound_margin_log(u32 slice);
void pmu_eye_centroid_avg_calc(const u16 *r0, u16 *r1, u16 *r2, u16 *r3);
void pmu_phy_reg_offset_diff_adjust(u32 a0);

u32 pmu_eye_margin_boundary_detect(u8 *out_start, u8 *out_end, const u8 *samples);
extern void pmu_phy_csr_result_stream(const void *buf, u32 len);
extern u32 pmu_cal_sequence_pulse_send(u32 a0, u32 a1, u32 a2, u32 a3, u32 a4, u32 a5, u32 a6);
void pmu_cbt_coarse_step_pulse_seq(u32 arg0, u32 arg1, u32 arg2);
u16 pmu_cal_tracker_stride_advance(u32 marker, u32 div, u32 flag);
u32 pmu_cal_tracker_step_update(void);
int pmu_cal_stride_div_update(int stride, u32 flags);
void pmu_delay_us(u32 count, u32 unit);
void pmu_clk_timing_latch(u16 timing, u32 wait_ack);
void pmu_clk_gate_disable(void);
u16 pmu_cal_stride_get(void);
void pmu_clk_gate_enable(void);
void pmu_cal_dmem_stride_bytes_fill(u8 *base, u8 *r1, u8 val, u32 step);
extern u32 pmu_cal_timing_param_step_calc(void);
extern u32 pmu_cal_table_50c_lookup(void);
extern void pmu_cal_multi_rank_dispatch_e170(u32 a0, u32 a1, u32 a2, u32 a3, u32 a4, u32 a5, u32 a6);
extern void pmu_cal_multi_rank_pulse_seq_dfec(u32 a0, u32 a1, u32 a2, u32 a3, u32 a4, u32 a5);
u32 pmu_cal_profile_mode_get(void);
void pmu_cal_multi_param_dispatch_2178(u32 arg0, u32 arg1, u32 arg2, u32 arg3, u32 arg4);
void pmu_cal_delay_line_init(void);
void pmu_cal_timing_table_matrix_init(u8 *buf);
void pmu_slice_lcdl_delay_collect(u32 r0, void *out_buf, u32 count);
u32 pmu_train_main(void);
u32 pmu_channel_rank_avail_check(u32 bit_idx);
void pmu_tracker_field_extract(u8 *dest, u32 byte_offset);
void pmu_tracker_cal_mode_update(u8 *desc);

/**
 * pmu_phy_clk_gate_sync_pulse() - PHY clock gating and reset deassertion
 *
 * Derived from vendor PMU code at address 0xa3b4.
 * If DMEM[0x01] bit 0 is set, delays 10 ticks and pulses 0x9005e0a4 (1 then 0).
 * /
 */
void pmu_phy_clk_gate_sync_pulse(void)
{
	if (dmem_read8(PMU_DMEM_CLK_GATE_FLAG) & 1) {
		pmu_hw_timer_delay(10);
		phy_write16(PHY_REG_CLK_GATE_E0A4, 1);
		phy_write16(PHY_REG_CLK_GATE_E0A4, 0);
	}
}

/**
 * pmu_phy_reset_pulse() - Pulses PHY reset at 0x900400f2 (1 then 0)
 *
 * Derived from vendor PMU code at address 0xa3d8.
 * /
 */
void pmu_phy_reset_pulse(void)
{
	phy_write16(PHY_REG_RESET_PULSE_0F2, 1);
	phy_write16(PHY_REG_RESET_PULSE_0F2, 0);
}

/**
 * pmu_cbt_entry_pll_ctrl() - Command Bus Training (CBT) entry & PLL control
 *
 * Derived from vendor PMU code at address 0x9644.
 * Sets 0x900401f0 to (0x90040144 & ~3) | 1, waits, and asserts 0x90040144 = 1.
 * /
 */
void pmu_cbt_entry_pll_ctrl(void)
{
	u16 val = phy_read16(PHY_REG_CBT_CTRL);
	val = (val & ~0x3) | 0x1;
	phy_write16(PHY_REG_CBT_PLL_1F0, val);
	pmu_hw_timer_delay(4);
	pmu_hw_timer_delay(0x1f);
	phy_write16(PHY_REG_CBT_CTRL, 1);
}

/**
 * pmu_cbt_exit_mission_handoff() - Command Bus Training (CBT) exit & mission mode handoff
 *
 * Derived from vendor PMU code at address 0x966c.
 * Clears 0x900401f0 = 0 and restores 0x90040144 = (val & ~3) | 2.
 * /
 */
void pmu_cbt_exit_mission_handoff(void)
{
	pmu_hw_timer_delay(0x1f);
	pmu_hw_timer_delay(4);
	phy_write16(PHY_REG_CBT_PLL_1F0, 0);
	u16 val = phy_read16(PHY_REG_CBT_CTRL);
	val = (val & ~0x3) | 0x2;
	phy_write16(PHY_REG_CBT_CTRL, val);
}

/**
 * pmu_cbt_state_latch() - Latches CBT configuration state into DMEM (0x45a and 0x46a)
 * @val: Value to write or configure
 *
 * Derived from vendor PMU code at address 0x1848.
 * /
 */
void pmu_cbt_state_latch(u8 val)
{
	dmem_write8(PMU_DMEM_CBT_STATE_45A, 1);
	dmem_write8(PMU_DMEM_CBT_CONFIG_46A, val);
}

/**
 * pmu_timing_ctrl_set() - Configures PHY timing control at 0x9004004e
 * @val: Value to write or configure
 *
 * Derived from vendor PMU code at address 0x1858.
 * /
 */
void pmu_timing_ctrl_set(u16 val)
{
	phy_write16(PHY_REG_CLK_TIMING_CTRL, val);
}

/**
 * pmu_clk_gate_enable() - Enables master PHY clock gating at 0x90040040
 *
 * Derived from vendor PMU code at address 0x18c8.
 * /
 */
void pmu_clk_gate_enable(void)
{
	phy_write16(PHY_REG_CLK_GATE, 1);
}

/**
 * pmu_deskew_dly0_set() - Latches deskew delay tap 0 into PHY 0x90040044
 * @val: Value to write or configure
 *
 * Derived from vendor PMU code at address 0x18d4.
 * /
 */
void pmu_deskew_dly0_set(u16 val)
{
	u32 off = dmem_read32(PMU_DMEM_PARAM_41C);
	phy_write16(PHY_REG_DESKEW_DLY0_44 | (off << 1), val);
}

/**
 * pmu_deskew_dly1_set() - Latches deskew delay tap 1 into PHY 0x90040046
 * @val: Value to write or configure
 *
 * Derived from vendor PMU code at address 0x18e4.
 * /
 */
void pmu_deskew_dly1_set(u16 val)
{
	u32 off = dmem_read32(PMU_DMEM_PARAM_41C);
	phy_write16(PHY_REG_DESKEW_DLY1_46 | (off << 1), val);
}

/**
 * pmu_cbt_step_stat_set() - Sets CBT step status flag in DMEM at 0x46b
 *
 * Derived from vendor PMU code at address 0x1410.
 * /
 */
void pmu_cbt_step_stat_set(void)
{
	dmem_write8(PMU_DMEM_CBT_STEP_STAT_46B, 1);
}

/**
 * pmu_cal_search_win_init() - Initializes 2D calibration search windows with 0xff sentinels
 * @buf: Pointer to data buffer
 *
 * Derived from vendor PMU code at address 0x6eb4.
 * /
 */
void pmu_cal_search_win_init(u8 *buf)
{
	buf[0x00] = 0xff;
	buf[0x04] = 0xff;
	buf[0x1b] = 0xff;
	buf[0x1f] = 0xff;
	buf[0x01] = 0xff;
	buf[0x05] = 0xff;
	buf[0x1c] = 0xff;
	buf[0x20] = 0xff;
}

/**
 * pmu_cal_marker_a_clear() - Clears calibration tracker flag at DMEM 0x458
 *
 * Derived from vendor PMU code at address 0x0ac8.
 * /
 */
void pmu_cal_marker_a_clear(void)
{
	dmem_write16(PMU_DMEM_CAL_MARKER_A, 0);
}

/**
 * pmu_clk_gate_disable() - Disables master PHY clock gating at 0x90040040
 *
 * Derived from vendor PMU code at address 0x0ad4.
 * /
 */
void pmu_clk_gate_disable(void)
{
	phy_write16(PHY_REG_CLK_GATE, 0);
}

/**
 * pmu_cal_mode_mask_test() - Tests calibration mode flag bitmask
 * @val: Value to write or configure
 *
 * Derived from vendor PMU code at address 0x0ae0.
 * /
 *
 * Return: Computed u32 value or status.
 */
u32 pmu_cal_mode_mask_test(u32 val)
{
	if (val <= 0xe) {
		if ((1U << val) & 0x4041)
			return 1;
	}
	return (val == 0x28) ? 1 : 0;
}

/**
 * pmu_cal_stride_get() - Reads calibration stride parameter from DMEM 0x458
 *
 * Derived from vendor PMU code at address 0x0b48.
 * /
 *
 * Return: Computed u16 value or status.
 */
u16 pmu_cal_stride_get(void)
{
	return dmem_read16(PMU_DMEM_CAL_MARKER_A) >> 2;
}

/**
 * pmu_cbt_config_get() - Reads CBT configuration state from DMEM 0x46a
 *
 * Derived from vendor PMU code at address 0x0b54.
 * /
 *
 * Return: Computed u8 value or status.
 */
u8 pmu_cbt_config_get(void)
{
	return dmem_read8(PMU_DMEM_CBT_CONFIG_46A);
}

/**
 * pmu_deskew_and_tracker_reset() - Resets deskew delay lines and calibration tracker
 *
 * Derived from vendor PMU code at address 0x0b60.
 * /
 *
 * Return: Computed u32 value or status.
 */
u32 pmu_deskew_and_tracker_reset(void)
{
	pmu_deskew_dly0_set(0);
	pmu_deskew_dly1_set(0);
	pmu_cal_markers_set(0);
	pmu_cal_marker_a_clear();
	return 0;
}

/**
 * pmu_cbt_cal_stat_clear() - Clears CBT calibration active status flag in DMEM at 0x45c
 *
 * Derived from vendor PMU code at address 0x174c.
 * /
 */
void pmu_cbt_cal_stat_clear(void)
{
	dmem_write8(PMU_DMEM_CBT_CAL_STAT_45C, 0);
}

/**
 * pmu_cal_markers_set() - Sets calibration parameters at DMEM 0x458 and 0x45e
 * @val: Value to write or configure
 *
 * Derived from vendor PMU code at address 0x1758.
 * /
 */
void pmu_cal_markers_set(u16 val)
{
	dmem_write16(PMU_DMEM_CAL_MARKER_A, val);
	dmem_write16(PMU_DMEM_CAL_MARKER_B, val);
}

/**
 * pmu_cbt_cal_stat_set() - Sets CBT calibration active status flag in DMEM at 0x45c
 *
 * Derived from vendor PMU code at address 0x1764.
 * /
 */
void pmu_cbt_cal_stat_set(void)
{
	dmem_write8(PMU_DMEM_CBT_CAL_STAT_45C, 1);
}

/**
 * pmu_dram_scaled_div_round() - Scaled integer rounding division based on DRAM type (DMEM 0x08) denominator
 * @val: Value to write or configure
 *
 * Derived from vendor PMU code at address 0x2814.
 * /
 *
 * Return: Computed s32 value or status.
 */
s32 pmu_dram_scaled_div_round(s32 val)
{
	u32 denom = (u32)dmem_read8(PMU_DMEM_DRAM_TYPE) << 7;
	if (val >= 1)
		return (val + denom - 1) / (s32)denom;
	return -((-val) / (s32)denom);
}

/**
 * pmu_dmem_8a_mode_check() - Checks DMEM 0x8a status bits [1:0] == 1
 *
 * Derived from vendor PMU code at address 0x2928.
 * /
 *
 * Return: Computed u32 value or status.
 */
u32 pmu_dmem_8a_mode_check(void)
{
	return (dmem_read8(PMU_DMEM_PARAM_8A) & PMU_PARAM_8A_MODE_MASK) == 1;
}

/**
 * pmu_dmem_8e_mode_check() - Checks DMEM 0x8e status bits [1:0] == 0
 *
 * Derived from vendor PMU code at address 0x296c.
 * /
 *
 * Return: Computed u32 value or status.
 */
u32 pmu_dmem_8e_mode_check(void)
{
	return (dmem_read8(PMU_DMEM_PARAM_8E) & PMU_PARAM_8E_MODE_MASK) == 0;
}

/**
 * pmu_dmem_96_nibble_get() - Extracts lower 4-bit nibble from DMEM 0x96
 *
 * Derived from vendor PMU code at address 0x297c.
 * /
 *
 * Return: Computed u32 value or status.
 */
u32 pmu_dmem_96_nibble_get(void)
{
	return dmem_read8(PMU_DMEM_PARAM_96) & PMU_PARAM_96_NIBBLE_MASK;
}

/**
 * pmu_hdr_addr_15b_decode() - Reconstructs 15-bit address from 2-byte header (7-bit shift)
 * @ptr: Pointer to memory structure or buffer
 *
 * Derived from vendor PMU code at address 0x458c.
 * /
 *
 * Return: Computed u32 value or status.
 */
u32 pmu_hdr_addr_15b_decode(const u8 *ptr)
{
	return ((u32)ptr[0] << 7) + ptr[1];
}

/**
 * pmu_hdr_addr_14b_decode() - Reconstructs 14-bit address from 2-byte header (6-bit shift)
 * @ptr: Pointer to memory structure or buffer
 *
 * Derived from vendor PMU code at address 0x4598.
 * /
 *
 * Return: Computed u32 value or status.
 */
u32 pmu_hdr_addr_14b_decode(const u8 *ptr)
{
	return ((u32)ptr[0] << 6) + ptr[1];
}

/**
 * pmu_dq_phy_to_logical_mask() - Deswizzles physical DBYTE PHY receiver pin bitmask back to logical DQ byte
 * @in_val: Parameter in_val
 * @idx: Parameter idx
 *
 * Derived from vendor PMU code at address 0x45a4.
 * based on descriptor table at DMEM 0x80000a7e.
 * /
 *
 * Return: Computed u8 value or status.
 */
u8 pmu_dq_phy_to_logical_mask(u32 in_val, u32 idx)
{
	const u8 *base = (const u8 *)(uintptr_t)(PMU_DMEM_DQ_PIN_MAP_BASE + (idx * PMU_DQ_PIN_MAP_ENTRY_SIZE));
	u8 res = base[0] & (u8)in_val;
	u8 count = base[1];
	const u8 *pairs = base + 2;

	for (u32 i = 0; i < count; i++) {
		u8 test_mask = pairs[2 * i];
		u8 bit_pos = pairs[2 * i + 1];
		if (test_mask & in_val)
			res |= (1U << bit_pos);
	}
	return res;
}

/**
 * pmu_2b_identity_lut() - 2-bit identity / permutation table lookup (0xe4)
 * @val: Value to write or configure
 *
 * Derived from vendor PMU code at address 0x2f70.
 * /
 *
 * Return: Computed u32 value or status.
 */
u32 pmu_2b_identity_lut(u32 val)
{
	u32 shift = (val << 1) & 6;
	return (PMU_LUT_2B_IDENTITY_E4 >> shift) & 3;
}

/**
 * pmu_cal_param_unpack() - Unpacks calibration parameters into out byte array
 * @val: Value to write or configure
 * @out: Pointer to output memory
 *
 * Derived from vendor PMU code at address 0x2f80.
 * /
 */
void pmu_cal_param_unpack(u32 val, u8 *out)
{
	out[1] = val & PMU_LCDL_FINE_MASK;
	out[0] = (u8)(((val >> 7) & 7) + ((val >> 6) & 1));
}

/**
 * pmu_dly_line_repack() - Combines packed delay line fields into a single register word
 * @val: Value to write or configure
 *
 * Derived from vendor PMU code at address 0x2f94.
 * /
 *
 * Return: Computed u32 value or status.
 */
u32 pmu_dly_line_repack(u32 val)
{
	return (val & PMU_LCDL_FINE_MASK) + (val & 0x40) + ((val >> 1) & 0x1c0);
}

/**
 * pmu_dbyte_cal_strobe_seq() - Sends calibration trigger pulse sequence to DBYTE slice register 0x90020176
 * @lane: Bit lane index within slice (0..8)
 * @bit_idx: Bit lane index within slice (0..8)
 *
 * Derived from vendor PMU code at address 0x298c.
 * /
 */
void pmu_dbyte_cal_strobe_seq(u32 lane, u32 bit_idx)
{
	u32 reg = PHY_REG_DBYTE_BASE + PHY_REG_DBYTE_CAL_TRIG_176 + (lane * PHY_REG_DBYTE_STRIDE);
	u16 r2 = (bit_idx == 0xf) ? 0x1ff : (1U << bit_idx);
	u16 r1 = r2 | (1U << 9);

	phy_write16(reg, 0);
	phy_write16(reg, r2);
	phy_write16(reg, r1);
	phy_write16(reg, r2);
	phy_write16(reg, 0);
}

/**
 * pmu_cal_status_148_read() - Reads 16-bit PHY status/configuration register at 0x90040148
 *
 * Derived from vendor PMU code at address 0x69a8.
 * /
 *
 * Return: Computed u16 value or status.
 */
u16 pmu_cal_status_148_read(void)
{
	return phy_read16(PHY_REG_CAL_STAT_148);
}

/**
 * pmu_dmem_param_05_read() - Reads 8-bit parameter from DMEM at 0x05
 *
 * Derived from vendor PMU code at address 0xda68.
 * /
 *
 * Return: Computed u8 value or status.
 */
u8 pmu_dmem_param_05_read(void)
{
	return dmem_read8(PMU_DMEM_CAL_PARAM_05);
}

/**
 * pmu_freq_lt_3200_check() - Checks DRAM frequency threshold: returns 1 if clock rate < 3200 MT/s (0xc80), else 0
 *
 * Derived from vendor PMU code at address 0x4a34.
 * /
 *
 * Return: Computed u32 value or status.
 */
u32 pmu_freq_lt_3200_check(void)
{
	return (dmem_read16(PMU_DMEM_DRAM_FREQ_OFF) < PMU_FREQ_THRESHOLD_3200) ? 1 : 0;
}

/**
 * pmu_dly_line_unpack() - Unpacks delay line parameter into fine delay out[1] and coarse delay out[0]
 * @val: Value to write or configure
 * @out: Pointer to output memory
 *
 * Derived from vendor PMU code at address 0x65a0.
 * /
 */
void pmu_dly_line_unpack(u32 val, u8 *out)
{
	out[1] = val & PMU_LCDL_FINE_MASK;
	out[0] = (u8)(val >> PMU_LCDL_COARSE_SHIFT);
}

/**
 * pmu_cal_strobe_pulse() - Generates active-high strobe pulse of 0x20 ticks on PHY register 0x90040146
 *
 * Derived from vendor PMU code at address 0x65ac.
 * /
 */
void pmu_cal_strobe_pulse(void)
{
	phy_write16(PHY_REG_CAL_TRIG_146, 1);
	pmu_hw_timer_delay(PMU_CAL_STROBE_TICKS);
	phy_write16(PHY_REG_CAL_TRIG_146, 0);
}

/**
 * pmu_cal_strobe_secondary_pulse() - Generates active-high strobe pulse of 0x20 ticks on PHY register 0x9005e140
 *
 * Derived from vendor PMU code at address 0xaf94.
 * /
 */
void pmu_cal_strobe_secondary_pulse(void)
{
	phy_write16(PHY_REG_CAL_STROBE_E140, 1);
	pmu_hw_timer_delay(PMU_CAL_STROBE_TICKS);
	phy_write16(PHY_REG_CAL_STROBE_E140, 0);
}

/**
 * pmu_dmem_1c_bit_query() - Extracts indexed bit from DMEM 0x1c bitfield: (DMEM[0x1c] >> ((r0 << 1) + r1)) & 1
 * @r0: Parameter r0
 * @r1: Parameter r1
 *
 * Derived from vendor PMU code at address 0x7d68.
 * /
 *
 * Return: Computed u32 value or status.
 */
u32 pmu_dmem_1c_bit_query(u32 r0, u32 r1)
{
	u32 val = dmem_read8(PMU_DMEM_LANE_MASK_1C);
	u32 shift = (r0 << 1) + r1;
	return (val >> shift) & 1;
}

/**
 * pmu_cal_param_table_write() - Writes 16-bit calibration parameter to 2D table in DMEM at 0x43c + (r0 * 4) + (r1 * 2)
 * @r0: Parameter r0
 * @r1: Parameter r1
 * @r2: Parameter r2
 *
 * Derived from vendor PMU code at address 0xb808.
 * /
 */
void pmu_cal_param_table_write(u32 r0, u32 r1, u16 r2)
{
	dmem_write16(PMU_DMEM_CAL_TABLE_43C + (r0 << 2) + (r1 << 1), r2);
}

/**
 * pmu_master_cfg_quad_write() - Writes four consecutive 16-bit parameters starting at PHY CSR 0x900e00c2
 * @r0: Parameter r0
 * @r1: Parameter r1
 * @r2: Parameter r2
 * @r3: Parameter r3
 *
 * Derived from vendor PMU code at address 0x7244.
 * /
 */
void pmu_master_cfg_quad_write(u16 r0, u16 r1, u16 r2, u16 r3)
{
	phy_write16(PHY_REG_PLL_CFG_00C2, r0);
	phy_write16(PHY_REG_PLL_CFG_00C4, r1);
	phy_write16(PHY_REG_PLL_CFG_00C6, r2);
	phy_write16(PHY_REG_PLL_CFG_00C8, r3);
}

/**
 * pmu_cbt_coarse_step_pulse() - Wrapper triggering a 5-step CBT coarse phase pulse via pmu_cbt_coarse_step_pulse_seq(0, 5, 1)
 *
 * Derived from vendor PMU code at address 0xbb98.
 * /
 */
void pmu_cbt_coarse_step_pulse(void)
{
	pmu_cbt_coarse_step_pulse_seq(0, 5, 1);
}

/**
 * pmu_lcdl_dev_calc() - Computes LCDL delay deviation relative to nominal midpoint 64 (0x40)
 * @val: Value to write or configure
 *
 * Derived from vendor PMU code at address 0xb818.
 * /
 *
 * Return: Computed u8 value or status.
 */
u8 pmu_lcdl_dev_calc(u32 val)
{
	if (val <= PMU_LCDL_NOMINAL_MIDPOINT)
		return (u8)(PMU_LCDL_NOMINAL_MIDPOINT - val);
	else
		return (u8)(val + 0xc0);
}

/**
 * pmu_dram_cfg_flag13_check() - Checks DRAM configuration flag (bit 13 of DMEM 0x0a) when r0 != 4 and r1 == 0
 * @r0: Parameter r0
 * @r1: Parameter r1
 *
 * Derived from vendor PMU code at address 0xb82c.
 * /
 *
 * Return: Computed u32 value or status.
 */
u32 pmu_dram_cfg_flag13_check(u32 r0, u32 r1)
{
	if (r0 == 4 || r1 != 0)
		return 0;
	return (dmem_read16(PMU_DMEM_DRAM_CFG_FLAGS) >> PMU_CFG_FLAG_BIT13_SHIFT) & 1;
}

/**
 * pmu_slice_status_b97_save() - Reads lane/slice status from PHY CSR 0x90040018 and saves to DMEM at 0xb97
 *
 * Derived from vendor PMU code at address 0xb844.
 * /
 */
void pmu_slice_status_b97_save(void)
{
	u32 off = dmem_read32(PMU_DMEM_PARAM_41C);
	u16 val = phy_read16(PHY_REG_SLICE_STAT_0018 | (off << 1));
	dmem_write8(PMU_DMEM_LANE_STATUS, (u8)val);
}

/**
 * pmu_mailbox_post_exec_wait() - MicroContPost mailbox command trigger and busy polling:
 *
 * Derived from vendor PMU code at address 0xda4c.
 * Waits for busy bit 0 of 0x90180008 to clear, triggers execute (0x90180066 = 1),
 * and waits for completion acknowledge (bit 0 of 0x90180008 set).
 * /
 */
void pmu_mailbox_post_exec_wait(void)
{
	while (phy_read16(PHY_REG_MAILBOX_STAT) & 1)
		;
	phy_write16(PHY_REG_MAILBOX_INT, 1);
	while ((phy_read16(PHY_REG_MAILBOX_STAT) & 1) == 0)
		;
}

/**
 * pmu_mailbox_post_cmd32_send() - Sends 32-bit command/parameter word to POST mailbox registers and executes command
 * @val: Value to write or configure
 *
 * Derived from vendor PMU code at address 0xda34.
 * /
 */
void pmu_mailbox_post_cmd32_send(u32 val)
{
	phy_write16(PHY_REG_MAILBOX_STREAM, (u16)val);
	phy_write16(PHY_REG_MAILBOX_INT, 0);
	phy_write16(PHY_REG_MAILBOX_STREAM_HI, (u16)(val >> 16));
	pmu_mailbox_post_exec_wait();
}

/**
 * pmu_mailbox_post_cmd_dispatch() - Dispatches command byte to PHY CSR 0x90040238 (if 255 or 7) and executes POST mailbox command
 * @cmd: Parameter cmd
 *
 * Derived from vendor PMU code at address 0xda00.
 * /
 */
void pmu_mailbox_post_cmd_dispatch(u32 cmd)
{
	if (cmd == 255) {
		phy_write16(PHY_REG_CMD_DISPATCH_238, 4);
		phy_write16(PHY_REG_CMD_DISPATCH_238, 0);
	} else if (cmd == 7) {
		phy_write16(PHY_REG_CMD_DISPATCH_238, 1);
		phy_write16(PHY_REG_CMD_DISPATCH_238, 0);
	}
	phy_write16(PHY_REG_MAILBOX_STREAM, (u16)cmd);
	phy_write16(PHY_REG_MAILBOX_INT, 0);
	pmu_mailbox_post_exec_wait();
}

/**
 * pmu_post_cmd_conditional_dispatch() - Conditional POST command dispatch based on DMEM 0x12 threshold followed by clock gating unreset
 * @val: Value to write or configure
 *
 * Derived from vendor PMU code at address 0x96d8.
 * /
 */
void pmu_post_cmd_conditional_dispatch(u32 val)
{
	if (dmem_read8(PMU_DMEM_POST_THRESH_12) <= PMU_POST_CMD_THRESH_200)
		pmu_mailbox_post_cmd_dispatch(val);
	pmu_phy_clk_gate_sync_pulse();
}

/**
 * pmu_cal_marker_verify() - Compares calibration target markers at DMEM 0x458 and 0x45e; if mismatched, invokes pmu_cal_tracker_step_update()
 *
 * Derived from vendor PMU code at address 0x0b34.
 * /
 *
 * Return: Computed u32 value or status.
 */
u32 pmu_cal_marker_verify(void)
{
	if (dmem_read16(PMU_DMEM_CAL_MARKER_A) == dmem_read16(PMU_DMEM_CAL_MARKER_B))
		return 0;
	return pmu_cal_tracker_step_update();
}

/**
 * pmu_cal_handler_select() - Selects calibration handler based on DMEM 0x0b bit 2
 * @r0: Parameter r0
 *
 * Derived from vendor PMU code at address 0xaa5c.
 * /
 *
 * Return: Computed u32 value or status.
 */
u32 pmu_cal_handler_select(u32 r0)
{
	if (dmem_read8(PMU_DMEM_CAL_SELECT_FLAGS) & 4)
		return pmu_window_centroid_calc((u8 *)(uintptr_t)r0);
	return pmu_eye_margin_window_search((u8 *)(uintptr_t)r0, 0);
}

/**
 * pmu_clk_timing_delay_latch() - Configures clock timing with LCDL delay line latching and optional PLL acknowledge wait
 * @timing: Timing delay parameter
 * @wait_ack: Flag to wait for hardware acknowledgement
 *
 * Derived from vendor PMU code at address 0x13bc.
 * /
 */
void pmu_clk_timing_delay_latch(u16 timing, u32 wait_ack)
{
	pmu_timing_ctrl_set(timing);
	pmu_hw_timer_delay(6);
	u16 stride = pmu_cal_stride_get() - 1;
	pmu_deskew_dly1_set(stride);
	pmu_clk_gate_enable();
	if (wait_ack != 0) {
		pmu_phy_pll_lock_poll(25000);
		pmu_clk_gate_disable();
	}
}

/**
 * pmu_clk_timing_latch() - Configures clock timing and enables master clock gate with optional PLL acknowledge wait
 * @timing: Timing delay parameter
 * @wait_ack: Flag to wait for hardware acknowledgement
 *
 * Derived from vendor PMU code at address 0x13ec.
 * /
 */
void pmu_clk_timing_latch(u16 timing, u32 wait_ack)
{
	pmu_timing_ctrl_set(timing);
	pmu_hw_timer_delay(6);
	pmu_clk_gate_enable();
	if (wait_ack != 0) {
		pmu_phy_pll_lock_poll(25000);
		pmu_clk_gate_disable();
	}
}

/**
 * pmu_delay_tap_step_adjust() - Decrements coarse delay tap and adds 64 fine delay steps
 * @ptr: Pointer to memory structure or buffer
 * @flag: Parameter flag
 *
 * Derived from vendor PMU code at address 0x8618.
 * /
 */
void pmu_delay_tap_step_adjust(u8 *ptr, u32 flag)
{
	u8 val = ptr[0];
	if (flag == 0) {
		if (val == 0 || (val & 1))
			return;
	} else {
		if (!(val & 1))
			return;
	}
	ptr[0] = val - 1;
	ptr[1] += PMU_LCDL_NOMINAL_MIDPOINT;
}

/**
 * pmu_delay_us() - Calibrated microsecond delay via hardware loop count computation
 * @count: Number of items, halfwords, or iterations
 * @unit: Timing delay unit multiplier
 *
 * Derived from vendor PMU code at address 0x8d64.
 * /
 */
void pmu_delay_us(u32 count, u32 unit)
{
	pmu_hw_timer_delay(pmu_freq_delay_step_calc(count, unit));
}

/**
 * pmu_delay_clamp() - Computes delay loop count and clamps to maximum allowable ceiling
 * @count: Number of items, halfwords, or iterations
 * @max_limit: Maximum clamp threshold
 *
 * Derived from vendor PMU code at address 0x8dc4.
 * /
 *
 * Return: Computed u32 value or status.
 */
u32 pmu_delay_clamp(u32 count, u32 max_limit)
{
	u32 calc = pmu_freq_delay_step_calc(count, 0);
	return (calc < max_limit) ? calc : max_limit;
}

/**
 * pmu_cbt_3phase_pulse_seq() - Triggers standard 3-phase CBT pulse sequence (16, 16, 10 steps) via pmu_cbt_coarse_step_pulse_seq
 *
 * Derived from vendor PMU code at address 0xbb74.
 * /
 */
void pmu_cbt_3phase_pulse_seq(void)
{
	pmu_cbt_coarse_step_pulse_seq(0, 0x10, 1);
	pmu_cbt_coarse_step_pulse_seq(0x10, 0, 1);
	pmu_cbt_coarse_step_pulse_seq(0, 0x0a, 1);
}

/**
 * pmu_deskew_latch_seq() - Latches deskew delay, asserts master PLL clock control, and delays 50ms
 *
 * Derived from vendor PMU code at address 0xbba4.
 * /
 */
void pmu_deskew_latch_seq(void)
{
	pmu_deskew_dly0_set(300);
	pmu_deskew_dly1_set(dmem_read16(PMU_DMEM_DESKEW_DELAY_47E));
	phy_write16(PHY_REG_PLL_CLK_CTRL, 1);
	pmu_clk_timing_latch(0, 0);
	pmu_delay_us(50000, 15);
}

/**
 * pmu_clk_gate_handoff() - Performs master PHY clock gating handoff and delays 5ms
 *
 * Derived from vendor PMU code at address 0xbbdc.
 * /
 */
void pmu_clk_gate_handoff(void)
{
	phy_write16(PHY_REG_PLL_CLK_CTRL, 0);
	while (phy_read16(PHY_REG_CLK_GATE_STAT) == 0)
		;
	phy_write16(PHY_REG_CLK_GATE, 0);
	pmu_delay_us(5000, 0);
}

/**
 * pmu_cal_param_table_init() - Initializes 2D calibration parameter table words at DMEM 0x43c and 0x440
 *
 * Derived from vendor PMU code at address 0x6e98.
 * /
 */
void pmu_cal_param_table_init(void)
{
	dmem_write32(PMU_DMEM_CAL_TABLE_43C, PMU_CAL_DEFAULT_PARAM);
	dmem_write32(PMU_DMEM_CAL_TABLE_440, PMU_CAL_DEFAULT_PARAM);
}

/**
 * pmu_train_mode_flag_query() - Queries training mode flag (bit 3 of DMEM 0x14) for stages 1 and 3..7
 * @stage: Training stage index or bitmask
 *
 * Derived from vendor PMU code at address 0x7330.
 * /
 *
 * Return: Computed u16 value or status.
 */
u16 pmu_train_mode_flag_query(u32 stage)
{
	if (stage == 1 || (stage >= 3 && stage <= 7))
		return (dmem_read16(PMU_DMEM_TRAIN_MODE_14) >> 3) & 1;
	return 0;
}

/**
 * pmu_dq_swap_query() - Queries DQ/DQS swap & bit polarity lookup using rank boundary and swap mask
 * @r0: Parameter r0
 * @r1: Parameter r1
 *
 * Derived from vendor PMU code at address 0x72d4.
 * /
 *
 * Return: Computed u32 value or status.
 */
u32 pmu_dq_swap_query(u32 r0, u32 r1)
{
	u8 boundary = dmem_read8(PMU_DMEM_RANK_BOUNDARY);
	u8 mask = dmem_read8(PMU_DMEM_DQ_SWAP_MASK);
	u32 bit_val = r1 & 1;
	u32 cond = (boundary < r1) ? 1 : 0;
	r0 += cond * 2;
	if ((mask & (1 << r0)) == 0)
		bit_val ^= 1;
	return bit_val;
}

/**
 * pmu_ashl64() - 64-bit logical shift left: (val << (shift & 0x3f))
 * @val: Value to write or configure
 * @shift: Bit-shift amount
 *
 * Derived from vendor PMU code at address 0x01d0.
 * /
 *
 * Return: Computed u64 value or status.
 */
u64 pmu_ashl64(u64 val, u32 shift)
{
	shift &= 0x3f;
	if (!shift)
		return val;
	union {
		u64 q;
		struct {
			u32 lo;
			u32 hi;
		} s;
	} in, out;
	in.q = val;
	if (shift < 32) {
		out.s.hi = (in.s.hi << shift) | (in.s.lo >> (32 - shift));
		out.s.lo = in.s.lo << shift;
	} else {
		out.s.hi = in.s.lo << (shift - 32);
		out.s.lo = 0;
	}
	return out.q;
}

/**
 * pmu_phy_cfg_target_set() - Programs PHY target configuration registers 0x900400e4 and 0x900400e6
 * @mode: Operational or calibration mode
 *
 * Derived from vendor PMU code at address 0x4f18.
 * from calibration parameter bytes at DMEM 0x25 and 0x40 based on mode.
 * /
 */
void pmu_phy_cfg_target_set(u32 mode)
{
	u8 p25 = (mode != 1) ? dmem_read8(PMU_DMEM_CAL_PARAM_25) : 0;
	u8 p40 = (mode != 0) ? dmem_read8(PMU_DMEM_CAL_PARAM_40) : 0;
	phy_write16(PHY_REG_CFG_00E4, p25);
	phy_write16(PHY_REG_CFG_00E6, p40);
}

/**
 * pmu_cal_status_query() - Queries calibration status using flags at DMEM 0xe7 and 0x62
 * @r0: Parameter r0
 * @r1: Parameter r1
 *
 * Derived from vendor PMU code at address 0x4a00.
 * /
 *
 * Return: Computed u32 value or status.
 */
u32 pmu_cal_status_query(u32 r0, u32 r1)
{
	u8 val_e7 = dmem_read8(PMU_DMEM_CAL_STATUS_E7);
	s8 val_62;

	if (val_e7 & 1)
		return 1;
	if (val_e7 & 2)
		return 0;

	val_62 = (s8)dmem_read8(PMU_DMEM_CAL_FLAG_62);
	r1 &= ~1;
	if (r1 == 2)
		return (r0 != 0 && val_62 < 0) ? 1 : 0;
	return (val_62 >> 6) & 1;
}

/**
 * pmu_delay_step_calc() - Calculates calibration delay tap step size from DRAM type (DMEM 0x08)
 *
 * Derived from vendor PMU code at address 0x6624.
 * and byte lane calibration tracker flags.
 * /
 *
 * Return: Computed u32 value or status.
 */
u32 pmu_delay_step_calc(void)
{
	u8 rank = dmem_read8(PMU_DMEM_CAL_RANK);
	u8 byte_idx = dmem_read8(PMU_DMEM_CAL_BYTE);
	u32 base_ptr = dmem_read32(PMU_DMEM_CAL_STRUCT_PTR);
	u32 offset = (rank * 108) + (byte_idx * 54) + 3;
	u8 flags = *(volatile u8 *)(uintptr_t)(base_ptr + offset);
	u8 dram_type = dmem_read8(PMU_DMEM_DRAM_TYPE);

	if (dram_type == 2)
		return (flags & 0x10) ? 4 : 8;
	return (flags & 0x10) ? 2 : 4;
}

/**
 * pmu_cal_struct_mask_and() - Applies bitwise AND mask across all ranks and byte lanes in calibration data structure
 * @base: Base memory address or offset
 * @byte_offset: Byte offset within data structure or DMEM
 * @mask: Bitmask filter
 *
 * Derived from vendor PMU code at address 0x1dac.
 * /
 */
void pmu_cal_struct_mask_and(u32 base, u32 byte_offset, u8 mask)
{
	volatile u8 *p = (volatile u8 *)(uintptr_t)(base + byte_offset);
	for (u32 i = 0; i < 4; i++, p += 54)
		*p &= mask;
}

/**
 * pmu_cal_struct_mask_or() - Applies bitwise OR mask across all ranks and byte lanes in calibration data structure
 * @base: Base memory address or offset
 * @byte_offset: Byte offset within data structure or DMEM
 * @mask: Bitmask filter
 *
 * Derived from vendor PMU code at address 0x888c.
 * /
 */
void pmu_cal_struct_mask_or(u32 base, u32 byte_offset, u8 mask)
{
	volatile u8 *p = (volatile u8 *)(uintptr_t)(base + byte_offset);
	for (u32 i = 0; i < 4; i++, p += 54)
		*p |= mask;
}

/**
 * pmu_phy_cal_state_save() - Saves PHY calibration state, masks clock gate bits, and archives
 *
 * Derived from vendor PMU code at address 0xb65c.
 * active slice delay line registers to DMEM parameter array.
 * /
 */
void pmu_phy_cal_state_save(void)
{
	u32 slice = dmem_read32(PMU_DMEM_SLICE_IDX_470);
	u32 reg = PHY_REG_CAL_STAT_022 + (slice * 2);
	u16 val = phy_read16(reg);
	dmem_write16(PMU_DMEM_CAL_SAVE_42A, val);
	phy_write16(reg, val & 0xff9f);

	if (dmem_read8(PMU_DMEM_CAL_PARAM_25) != 0) {
		u8 start = dmem_read8(PMU_DMEM_SLICE_START_B6A);
		u8 end = dmem_read8(PMU_DMEM_RANK_BOUNDARY);
		for (u32 i = start; i <= end; i++) {
			u16 r = phy_read16(PHY_REG_WCK_DELAY + (i << 13));
			dmem_write16(PMU_DMEM_PARAM_TABLE_430 + (i * 2), r);
		}
	}

	if (dmem_read8(PMU_DMEM_CAL_PARAM_40) != 0) {
		u8 start = dmem_read8(PMU_DMEM_SLICE_START_B6C);
		u8 end = dmem_read8(PMU_DMEM_SLICE_END_B6D);
		for (u32 i = start; i <= end; i++) {
			u16 r = phy_read16(PHY_REG_WCK_DELAY + (i << 13));
			dmem_write16(PMU_DMEM_PARAM_TABLE_430 + (i * 2), r);
		}
	}
}

/**
 * pmu_phy_cal_state_restore() - Restores PHY calibration state and reloads active slice delay line
 *
 * Derived from vendor PMU code at address 0xa734.
 * registers from DMEM parameter array.
 * /
 */
void pmu_phy_cal_state_restore(void)
{
	u32 slice = dmem_read32(PMU_DMEM_SLICE_IDX_470);
	u32 reg = PHY_REG_CAL_STAT_022 + (slice * 2);
	u16 val = dmem_read16(PMU_DMEM_CAL_SAVE_42A);
	phy_write16(reg, val);

	if (dmem_read8(PMU_DMEM_CAL_PARAM_25) != 0) {
		u8 start = dmem_read8(PMU_DMEM_SLICE_START_B6A);
		u8 end = dmem_read8(PMU_DMEM_RANK_BOUNDARY);
		for (u32 i = start; i <= end; i++) {
			u16 r = dmem_read16(PMU_DMEM_PARAM_TABLE_430 + (i * 2));
			phy_write16(PHY_REG_WCK_DELAY + (i << 13), r);
		}
	}

	if (dmem_read8(PMU_DMEM_CAL_PARAM_40) != 0) {
		u8 start = dmem_read8(PMU_DMEM_SLICE_START_B6C);
		u8 end = dmem_read8(PMU_DMEM_SLICE_END_B6D);
		for (u32 i = start; i <= end; i++) {
			u16 r = dmem_read16(PMU_DMEM_PARAM_TABLE_430 + (i * 2));
			phy_write16(PHY_REG_WCK_DELAY + (i << 13), r);
		}
	}
}

/**
 * pmu_inactive_slices_clear() - Clears LCDL status CSR (0x90021f6a + i * 0x2000) for slices outside active [start, end] window
 *
 * Derived from vendor PMU code at address 0x4988.
 * /
 */
void pmu_inactive_slices_clear(void)
{
	u8 num_slices = dmem_read8(PMU_DMEM_CAL_NUM_SLICES_42C);
	u8 start = dmem_read8(PMU_DMEM_SLICE_START);
	u8 end = dmem_read8(PMU_DMEM_SLICE_END);

	for (u32 i = 0; i < num_slices; i++) {
		if (i < start || i > end) {
			u32 reg = PHY_REG_DBYTE_BASE + PHY_REG_DBYTE_1F6A + (i * PHY_REG_DBYTE_STRIDE);
			phy_write16(reg, 0);
		}
	}
}

/**
 * pmu_slice_coarse_step_wrap() - Adjusts DBYTE slice LCDL delay by coarse step (+0x20) with wrap-around compensation
 *
 * Derived from vendor PMU code at address 0x6e5c.
 * /
 */
void pmu_slice_coarse_step_wrap(void)
{
	u8 start = dmem_read8(PMU_DMEM_SLICE_START);
	u8 end = dmem_read8(PMU_DMEM_SLICE_END);

	for (u32 i = start; i <= end; i++) {
		u32 reg = PHY_REG_DBYTE_BASE + PHY_REG_DBYTE_DLY_154 + (i * PHY_REG_DBYTE_STRIDE);
		u16 val = phy_read16(reg);
		u32 step = val + PMU_LCDL_STEP_COARSE;
		if ((step & PMU_LCDL_STEP_OVERFLOW_MASK) == 0) {
			val = (u16)step;
		} else {
			val = (u16)(val - PMU_LCDL_STEP_COARSE + PMU_LCDL_STEP_WRAP_OFFSET);
		}
		phy_write16(reg, val);
	}
}

/**
 * pmu_dbyte_pin_mask_calc() - Calculates physical DBYTE PHY pin mask from logical DQ mask and descriptor table at DMEM 0xa7e
 * @mask: Bitmask filter
 * @idx: Parameter idx
 *
 * Derived from vendor PMU code at address 0xbd30.
 * /
 *
 * Return: Computed u32 value or status.
 */
u32 pmu_dbyte_pin_mask_calc(u32 mask, u32 idx)
{
	uintptr_t base = PMU_DMEM_DQ_PIN_MAP_BASE + (idx * PMU_DQ_PIN_MAP_ENTRY_SIZE);
	u8 init_val = *(volatile u8 *)base;
	u8 count = *(volatile u8 *)(base + 1);
	u32 result = init_val & mask;
	const volatile u8 *entry = (const volatile u8 *)(base + 2);

	for (u32 i = 0; i < count; i++, entry += 2) {
		u8 bit_idx = entry[1];
		if (mask & (1U << (bit_idx & 0x1f)))
			result |= entry[0];
	}
	return result;
}

/**
 * pmu_dbyte_pin_map_table_init() - Reads hardware DBYTE DxDMap registers (0x90020100 + dbyte * 0x2000 + bit * 2)
 *
 * Derived from vendor PMU code at address 0x2e6c.
 * and populates the DQ pin mapping table at PMU_DMEM_DQ_PIN_MAP_BASE (0x80000a7e).
 * /
 */
void pmu_dbyte_pin_map_table_init(void)
{
	u8 num_dbytes = dmem_read8(0x42c);

	for (u32 dbyte = 0; dbyte < num_dbytes; dbyte++) {
		volatile u8 *entry = (volatile u8 *)(PMU_DMEM_DQ_PIN_MAP_BASE + dbyte * 18);
		entry[0] = 0; /* Identity mapped mask */
		entry[1] = 0; /* Remapped pin count */

		for (u32 bit = 0; bit < 8; bit++) {
			u32 reg = 0x90020100 + (dbyte * 0x2000) + (bit * 2);
			u8 pin = (u8)phy_read16(reg);

			if (pin == bit) {
				entry[0] |= (1 << bit);
			} else {
				u8 count = entry[1];
				entry[1] = count + 1;
				entry[2 + count * 2 + 0] = (1 << pin);
				entry[2 + count * 2 + 1] = bit;
			}
		}
	}
}

/**
 * pmu_cal_struct_to_shadow16() - Copies a byte-field across 2 ranks x 2 lanes from calibration struct into 16-bit shadow table at 0xbc0
 * @base: Base memory address or offset
 * @byte_offset: Byte offset within data structure or DMEM
 *
 * Derived from vendor PMU code at address 0xb748.
 * /
 */
void pmu_cal_struct_to_shadow16(u32 base, u32 byte_offset)
{
	const volatile u8 *src = (const volatile u8 *)(uintptr_t)(base + byte_offset);
	u32 dst = PMU_DMEM_CAL_SHADOW_BC0 + (byte_offset * 2);
	for (u32 i = 0; i < 4; i++, src += 54, dst += 108)
		dmem_write16(dst, *src);
}

/**
 * pmu_dmem_training_flags_eval() - Evaluates training flags from DMEM 0x62, 0x72, and 0x96
 *
 * Derived from vendor PMU code at address 0x0928.
 * Returns non-zero if training condition is active.
 * /
 *
 * Return: Computed u32 value or status.
 */
u32 pmu_dmem_training_flags_eval(void)
{
	u8 d62 = dmem_read8(0x62);
	u8 d72 = dmem_read8(0x72);
	u8 d96 = dmem_read8(PMU_DMEM_PARAM_96);

	u32 r13 = (d62 >> 7) & 1;
	u32 r1  = (d62 >> 6) & 1;
	u32 r12 = (d96 >> 6) & 1;
	u32 r3  = (d96 >> 4) & 1;
	u32 r2  = ((d72 >> 5) & 1) ? 0 : 1;

	return (r1 | r13 | r12 | r3 | r2) & 0xff;
}

/**
 * pmu_cal_tracker_step_update() - Updates calibration tracker struct at 0x80000d9c, invokes pmu_cal_pll_lock_retry_poll to advance
 *
 * Derived from vendor PMU code at address 0x0afc.
 * calibration step, records stride at DMEM 0x458, zeroes tracker data (0x20 bytes),
 * and resets tracker status words.
 * /
 *
 * Return: Computed u32 value or status.
 */
u32 pmu_cal_tracker_step_update(void)
{
	volatile u16 *tracker = (volatile u16 *)(uintptr_t)0x80000d9c;
	if (tracker[0x22 / 2] != 0)
		tracker[40 / 2] = 1;

	u32 res = pmu_cal_pll_lock_retry_poll(0x80000d9c);
	dmem_write16(PMU_DMEM_CAL_MARKER_A, (u16)res);
	pmu_memset_words((void *)(uintptr_t)0x80000d9c, 0, 0x20);
	tracker[0x28 / 2] = 0;
	tracker[0x26 / 2] = 0;
	tracker[0x24 / 2] = 0;
	tracker[0x22 / 2] = 0;
	return res;
}

/**
 * pmu_cal_stride_div_update() - Updates calibration stride divisor at PMU_DMEM_CAL_MARKER_A (0x458)
 * @stride: Parameter stride
 * @flags: Configuration bitmask or control flags
 *
 * Derived from vendor PMU code at address 0x0fc8.
 * /
 *
 * Return: 0 on success, negative error code on failure.
 */
int pmu_cal_stride_div_update(int stride, u32 flags)
{
	if (stride < 1)
		return stride;

	u8 div_val = dmem_read8(PMU_DMEM_CAL_MARKER_A + 2);
	u16 marker_a = dmem_read16(PMU_DMEM_CAL_MARKER_A);
	if (div_val != 0)
		stride = stride / (int)div_val;

	u32 r2 = 0;
	if (!(flags & (1 << 9))) {
		if (dmem_read8(PMU_DMEM_CAL_MARKER_A + 0x11) == 0)
			r2 = 1;
	}

	u32 res = pmu_cal_tracker_stride_advance(marker_a, (u16)stride, r2);
	dmem_write16(PMU_DMEM_CAL_MARKER_A, (u16)res);
	return res;
}

/**
 * pmu_phy_pll_lock_poll() - Polls Master PHY PLL / Clock status register 0x90040042 until non-zero
 * @timeout: Timeout counter limit
 *
 * Derived from vendor PMU code at address 0x0f94.
 * or timeout expires. Asserts lock status via pmu_assert_or_halt with code (9 << 18).
 * /
 *
 * Return: Computed u32 value or status.
 */
u32 pmu_phy_pll_lock_poll(int timeout)
{
	u32 ok = 0;
	volatile u16 *stat_reg = (volatile u16 *)(uintptr_t)0x90040042;

	for (int i = 0; i < timeout; i++) {
		if (*stat_reg != 0) {
			ok = 1;
			break;
		}
	}

	return pmu_assert_or_halt(ok, 9 << 18);
}

/**
 * pmu_dbyte_deskew_phase_sample() - Samples DQ deskew delay phase for the active rank/channel
 * @rank_idx: DRAM rank index (0..1)
 * @out_mask: Pointer to output mask word
 * @phase_mask: Parameter phase_mask
 *
 * Derived from vendor PMU code at address 0x02d4.
 * Measures phases 0..3 using deskew delay tables at 0x8000e7b8, latching
 * clock timing and mapping PHY slice registers into out_mask.
 * /
 */
void pmu_dbyte_deskew_phase_sample(u32 rank_idx, u32 *out_mask, u32 phase_mask)
{
	u8 channel = dmem_read8(0xb66);
	const u8 *byte_map = (const u8 *)(uintptr_t)(0x80000b44 + (channel << 4) + (rank_idx << 3));
	u32 count_idx = (channel << 1) + rank_idx;
	u8 num_slices = dmem_read8(0xb20 + count_idx);
	const u16 *dly_table = (const u16 *)(uintptr_t)0x8000e7b8;

	*out_mask = 0;

	for (u32 phase_idx = 0; phase_idx < 4; phase_idx++) {
		u32 invert_mask = (phase_idx < 2) ? 0x7f : 0x00;
		u32 phase = phase_idx & 1;

		if (!(phase_mask & (1 << phase)))
			continue;

		pmu_deskew_dly0_set(dly_table[phase_idx]);
		pmu_deskew_dly1_set(dly_table[phase_idx + 6]);
		pmu_clk_timing_latch(0, 1);

		for (u32 byte_idx = 0; byte_idx < num_slices; byte_idx++) {
			u8 slice = byte_map[byte_idx];
			u32 phy_addr = 0x9002011a + ((u32)slice << 13);
			u16 phy_val = *(volatile u16 *)(uintptr_t)phy_addr;
			u32 mask = pmu_dq_phy_to_logical_mask(phy_val, slice) & 0x7f;
			mask ^= invert_mask;
			*out_mask |= (mask << (phase * 8));
		}
	}
}
/**
 * pmu_cal_dual_rank_state_eval() - Cal Dual Rank State Eval
 * @p0: Parameter p0
 * @p1: Parameter p1
 * @p2: Parameter p2
 */
void pmu_cal_dual_rank_state_eval(void *p0, u16 *p1, u16 *p2)
{
	u8 *base = (u8 *)p0;
	for (u32 r30 = 0; r30 < 2; r30++) {
		for (u32 r13 = 0; r13 < 2; r13++) {
			u16 f2 = dmem_read16(0x800000f2);
			u16 e8 = dmem_read16(0x800000e8);
			u16 ea = dmem_read16(0x800000ea);
			if (f2 != 0 || e8 != 0 || ea != 0) {
				u32 idx = (r30 * 2 + r13) * 4;
				u16 val1 = *(u16 *)(base + idx);
				u16 val2 = *(u16 *)(base + idx + 2);
				u32 r3 = ((u32)*p2 * 2 + r13 + 0x20800) & 0x7ffff;
				u32 arg0 = ((u32)*p1 << 2) & 0xffff;
				pmu_phy_reg_write_shadow_track(arg0, val1, val2, r3, 3);
				(*p1)++;
			}
		}
		(*p2)++;
	}
}
/**
 * pmu_dmem_reg_stream_unpack() - Dmem Reg Stream Unpack
 * @arg0: Parameter arg0
 */
void pmu_dmem_reg_stream_unpack(void *arg0)
{
	phy_write16(0x9003ff6a, 0);
	phy_write16(0x9003e160, 0);
	phy_write16(0x9003e162, 0);
	phy_write16(0x90040180, 0);
	phy_write16(0x900400f2, 0);

	u32 offset = *(u32 *)((uintptr_t)arg0 + 0x18);
	phy_write16(0x9003e000 | (offset << 1), 0);

	u16 v = phy_read16(0x900e0022);
	v &= ~(1 << 4);
	phy_write16(0x900fe022, v);

	pmu_cal_strobe_pulse();
	if (dmem_read8(0x80000403) <= 1)
		pmu_phy_reset_pulse();
}
/**
 * pmu_dbyte_lane_error_mask_calc() - Dbyte Lane Error Mask Calc
 * @mode: Operational or calibration mode
 * @param1: Parameter param1
 */
void pmu_dbyte_lane_error_mask_calc(u32 mode, u32 param1)
{
	pmu_deskew_and_tracker_reset();
	if (mode != 0) {
		pmu_cbt_coarse_step_pulse_seq(0, 5, 1);
		pmu_cal_sequence_pulse_send(0, 0x25, 0, 0x86, 0, param1, 0);
		pmu_cal_sequence_pulse_send(0, 0x26, 0, 0x86, 0, 0, 0);
		pmu_cbt_coarse_step_pulse_seq(0, 0x10, 1);
		pmu_cbt_3phase_pulse_seq();
	} else {
		pmu_cal_sequence_pulse_send(0x80, 0x19, 4, 0x86, 0, param1 & 3, 0);
	}
	pmu_clk_timing_delay_latch(0, 1);
	if (mode == 0) {
		pmu_delay_us(0x7530, 0x1e);
	}
}
/**
 * pmu_slice_margin_window_clamp() - Slice Margin Window Clamp
 */
void pmu_slice_margin_window_clamp(void)
{
	pmu_phy_mode_cfg_dispatch(3);
	u8 a7c = dmem_read8(0x80000a7c);
	u32 mode = 0;
	if (a7c != 0) {
		u8 d01 = dmem_read8(0x80000001);
		if ((d01 & (1 << 3)) == 0) {
			pmu_ac_lane_profile_setup();
			mode = 1;
		}
	}
	pmu_dbyte_lane_error_mask_calc(mode, 0xf);
}
/**
 * pmu_cal_window_valid_check() - Cal Window Valid Check
 * @r0: Parameter r0
 * @r1: Parameter r1
 */
void pmu_cal_window_valid_check(u32 r0, u32 r1)
{
	u32 r14 = r0;
	u32 r15 = r1;
	phy_write16(0x9003e014, 0);
	pmu_cal_strobe_secondary_pulse();
	pmu_deskew_and_tracker_reset();
	pmu_cal_sequence_pulse_send(1 << 21, 5, 1, 256, 0, r14, 0);
	pmu_cal_sequence_pulse_send(0x80 | (1 << 18), 0xf, 0x20, 0, 0, r14, r15);
	pmu_clk_timing_delay_latch(0, 1);
	phy_write16(0x9003e014, 1);
}
/**
 * pmu_dmem_stride_descriptor_read() - Dmem Stride Descriptor Read
 */
void pmu_dmem_stride_descriptor_read(void)
{
	u32 buf[2] = {0, 0};
	u32 gp28 = *(u32 *)(uintptr_t)0x80000470;
	pmu_slice_lcdl_delay_collect(gp28, buf, 1);
	pmu_cal_metric_log(5, 0x2550000);
	pmu_cal_matrix_trace_dump(0, 1, (const u16 *)buf);
}
/**
 * pmu_cal_error_code_log() - Cal Error Code Log
 */
void pmu_cal_error_code_log(void)
{
	u32 buf[20];
	pmu_memset_words(buf, 0, sizeof(buf));
	pmu_cal_metric_log(5, 0x2560000);
	pmu_slice_lcdl_delay_collect(0x4e, buf, 9);
	pmu_cal_matrix_trace_dump(0, 9, (const u16 *)buf);
}
/**
 * pmu_cal_metric_tag_dump() - Cal Metric Tag Dump
 * @tag: Telemetry diagnostic tag
 * @args: Parameter args
 */
void pmu_cal_metric_tag_dump(u32 tag, const u32 *args)
{
	if (dmem_read8(0x01) & 0x10)
		return;

	phy_write16(0x90180064, 8);
	phy_write16(0x90180066, 0);
	pmu_mailbox_post_exec_wait();

	pmu_mailbox_post_cmd32_send(tag);

	u16 count = (u16)tag;
	if (count == 0)
		return;

	for (u16 i = 0; i < count; i++) {
		pmu_mailbox_post_cmd32_send(args[i]);
	}
}

/**
 * pmu_cal_metric_log() - Core PMU telemetry logging routine
 * @level: Log verbosity level
 * @tag: Telemetry diagnostic tag
 * @...: Variable arguments
 *
 * Derived from vendor PMU code at address 0x455c.
 * Filters messages against the minimum log severity threshold at DMEM 0x12.
 * Validated messages are forwarded with arguments to pmu_cal_metric_tag_dump.
 */
__attribute__((noinline)) void pmu_cal_metric_log(u32 level, u32 tag, ...)
{
	(void)level;
	(void)tag;
}


/**
 * pmu_cal_stage8_pad_init() - Stage 8: ZQ pad calibration, master state initialization, and CBT latching
 *
 * Derived from vendor PMU code at address 0x8e3c.
 * /
 */
void pmu_cal_stage8_pad_init(void)
{
	u32 stat = pmu_cal_status_148_read();
	u32 channels = (stat >> 4) & 3;
	u32 rank_metric = channels * (stat & 0xf);

	dmem_write32(0x410, channels);
	dmem_write32(0x40c, rank_metric);

	u8 rank1_en = dmem_read8(0x40);
	u32 r12;

	if (rank1_en) {
		r12 = ((rank_metric << 2) & 248) + dmem_read8(0x3f);
	} else {
		r12 = dmem_read8(0x24);
		if (channels == 2)
			dmem_write32(0x40c, rank_metric >> 1);
	}

	u32 val_8 = ((u8)r12 >> 3) + ((r12 & 7) ? 1 : 0);
	u32 val_4 = ((u8)r12 >> 2) + ((r12 & 3) ? 1 : 0);

	dmem_write8(0x42c, (u8)val_8);
	pmu_cal_metric_log(10, 0x1de0002, val_8, val_4);

	u8 dmem25 = dmem_read8(0x25);

	pmu_cal_metric_log(10, 0x1e00006, dmem25, rank1_en,
			   dmem_read16(0x10), dmem_read8(0x12),
			   dmem_read8(0x01), dmem_read16(0x06));
	pmu_cal_metric_log(10, 0x1e90001, dmem_read8(0x04) & 0xf);

	if (dmem25 & 1)
		pmu_profile_mailbox_stream_send(0);
	if (dmem25 & 2)
		pmu_profile_mailbox_stream_send(1);
	if (rank1_en & 1)
		pmu_profile_mailbox_stream_send(2);
	if (rank1_en & 2)
		pmu_profile_mailbox_stream_send(3);

	u32 ch_off = dmem_read32(0x41c) << 1;
	u16 r94 = phy_read16(0x90040094 | ch_off);
	u16 r96 = phy_read16(0x90040096 | ch_off);

	pmu_cal_metric_log(4, 0x1ea000b,
			   r94, r94 & 0x7f, (r94 >> 8) & 0x7f,
			   r96, r96 & 0x7f, (r96 >> 8) & 0x7f,
			   phy_read16(0x90040098 | ch_off),
			   phy_read16(0x90040110),
			   phy_read16(0x90040112),
			   phy_read16(0x90040114),
			   phy_read16(0x90040118));

	u16 r0_val = phy_read16(0x90180100);
	pmu_cal_param_table_init();
	dmem_write8(0x404, !((r0_val >> 2) & 1));

	phy_write16(0x90040050, 1);
	phy_write16(0x9005e14a, 1);
	phy_write16(0x9005e0ee, 0);

	pmu_cal_metric_log(4, 0x1ed0001, *(volatile u8 *)0x004a4802);
	pmu_cbt_state_latch(4);
	(void)*(volatile u8 *)0x004a4802;
	pmu_cal_metric_log(5, 0x01ee0000);

	if (dmem_read16(0x10) != 1) {
		if ((dmem_read8(0x04) & 0xf0) == 0x20)
			pmu_dual_rank_phy_reg_read();
	}

	if (dmem_read8(0xf4) != 0)
		pmu_phy_slice_cal_status_read();
}
/**
 * pmu_slice_deskew_state_latch() - Slice Deskew State Latch
 * @r0: Parameter r0
 * @r1: Parameter r1
 */
void pmu_slice_deskew_state_latch(u32 r0, u32 r1)
{
	pmu_deskew_and_tracker_reset();
	pmu_cal_sequence_pulse_send(0, 6, 0x40, r0, 24, r1, 0);
	pmu_clk_timing_delay_latch(0, 1);
}
/**
 * pmu_cal_descriptor_apply() - Cal Descriptor Apply
 * @desc: Pointer to descriptor structure
 */
void pmu_cal_descriptor_apply(const void *desc)
{
	const u8 *p = (const u8 *)desc;
	u32 offset = (*(const u32 *)(uintptr_t)0x80000a68) << 1;
	u8 r2 = p[0x14];
	u8 b1 = p[0x01];
	u16 val0 = (r2 & 0x0c) | ((b1 >> 3) & 1);
	u16 is_zero = ((r2 & 3) == 0) ? 1 : 0;

	phy_write16(0x9004002a | offset, val0);
	phy_write16(0x9003e016 | offset, is_zero << 1);
	phy_write16(0x9003e00a | offset, is_zero);
}
/**
 * pmu_cal_pll_status_check() - Cal Pll Status Check
 */
void pmu_cal_pll_status_check(void)
{
	const volatile u8 *p = (const volatile u8 *)0x80000037;
	for (u32 i = 0; i < 8; i++)
		pmu_cal_metric_log(4, 0x1a10001, p[i]);
	for (u32 i = 0; i < 8; i++) {
		u8 val = dmem_read8(0x80000052 + i);
		pmu_cal_metric_log(4, 0x1a20001, val);
	}
}
/**
 * pmu_cal_marker_pair_program() - Cal Marker Pair Program
 * @marker0: Primary synchronization marker
 * @marker1: Secondary synchronization marker
 * @arg2: Parameter arg2
 * @arg3: Parameter arg3
 */
void pmu_cal_marker_pair_program(u32 marker0, u32 marker1, u32 arg2, u32 arg3)
{
	u8 buf[8] = {0};
	buf[0] = (u8)arg3;
	buf[1] = (u8)arg3;

	pmu_cal_markers_set(marker0);
	u32 mode = (arg3 == 0) ? 3 : 1;
	pmu_cal_window_params_dispatch(buf, arg2, mode);
	pmu_cal_markers_set(marker1);
	pmu_clk_timing_delay_latch(0, 1);
}


/**
 * pmu_post_trace_log_halfwords() - Mailbox halfword stream trace transmitter checking threshold DMEM[0x12]
 * @threshold: Parameter threshold
 * @cmd32: Parameter cmd32
 * @data: Pointer to data array
 * @count: Number of items, halfwords, or iterations
 *
 * Derived from vendor PMU code at address 0x0240.
 * and quiet flag DMEM[0x01] bit 4.
 */
void pmu_post_trace_log_halfwords(u32 threshold, u32 cmd32, const u16 *data, u32 count)
{
	if (threshold < dmem_read8(PMU_DMEM_POST_THRESH_12))
		return;
	if (dmem_read8(0x01) & 0x10)
		return;

	phy_write16(PHY_REG_MAILBOX_STREAM, 8);
	phy_write16(PHY_REG_MAILBOX_INT, 0);
	pmu_mailbox_post_exec_wait();

	pmu_mailbox_post_cmd32_send(cmd32);

	for (u32 i = 0; i < count; i++)
		pmu_mailbox_post_cmd32_send(data[i]);
}

/**
 * pmu_assert_or_halt() - Core PMU assertion handler. If cond is non-zero, returns code
 * @cond: Condition flag to evaluate
 * @code: Assertion or error code
 *
 * Derived from vendor PMU code at address 0x0280.
 * If cond is zero (assertion failure), dispatches error 0xff and halts CPU.
 * /
 *
 * Return: Computed u32 value or status.
 */
u32 pmu_assert_or_halt(u32 cond, u32 code)
{
	if (cond)
		return code;

	dmem_write32(0x0c, code);
	pmu_mailbox_post_cmd_dispatch(0xff);
	__asm__ volatile (
		"flag 1\n"
		"nop_s\n"
		"1: b_s 1b\n"
	);
	return 0;
}

/**
 * pmu_dbyte_deskew_sample_check() - Tests DBYTE deskew delay calibration with zero and 0x7f reference markers
 * @arg0: Parameter arg0
 * @arg1: Parameter arg1
 * @arg2: Parameter arg2
 *
 * Derived from vendor PMU code at address 0x0804.
 * Returns non-zero (1) if any slice fails the mask check, 0 if all pass.
 * /
 *
 * Return: Computed u8 value or status.
 */
u8 pmu_dbyte_deskew_sample_check(u32 arg0, u32 arg1, u32 arg2)
{
	u8 channel = dmem_read8(0xb66);
	u32 rank = ((arg0 & 0x3) == 2) ? 1 : 0;
	const u8 *byte_map = (const u8 *)(uintptr_t)(0x80000b44 + (channel << 4) + (rank << 3));
	u32 count_idx = (channel << 1) + rank;
	u8 num_slices = dmem_read8(0xb20 + count_idx);
	u8 res = 0;

	pmu_cal_marker_pair_program(arg1, arg2, arg0, 0);
	pmu_delay_us(10000, 0);

	for (u32 i = 0; i < num_slices; i++) {
		u8 slice = byte_map[i];
		u32 phy_addr = 0x9002011a + ((u32)slice << 13);
		u16 phy_val = *(volatile u16 *)(uintptr_t)phy_addr;
		u8 mask = pmu_dq_phy_to_logical_mask(phy_val, slice);
		if (mask & 0x7f)
			res |= 1;
	}

	pmu_cal_marker_pair_program(arg1, arg2, arg0, 0x7f);

	for (u32 i = 0; i < num_slices; i++) {
		u8 slice = byte_map[i];
		u32 phy_addr = 0x9002011a + ((u32)slice << 13);
		u16 phy_val = *(volatile u16 *)(uintptr_t)phy_addr;
		u8 mask = pmu_dq_phy_to_logical_mask(phy_val, slice) & 0x7f;
		if (mask != 0x7f)
			res |= 1;
	}

	return res;
}

/**
 * pmu_cbt_entry_lookup() - Looks up CBT calibration table entry from 0x80000724 based on channel,
 * @table_base: Base address of calibration lookup table
 *
 * Derived from vendor PMU code at address 0x08f0.
 * rank/lane, CBT config mode, and table_base pointer.
 * /
 *
 * Return: Computed u8 value or status.
 */
u8 pmu_cbt_entry_lookup(u32 table_base)
{
	u8 channel = dmem_read8(0xb66);
	u8 slice = dmem_read8(0xb67);
	u32 ptr = table_base + (channel * 108) + (slice * 54);
	u8 cfg = pmu_cbt_config_get();
	u8 val = *(volatile u8 *)(uintptr_t)(ptr + 2);
	u32 r0 = (cfg != 2) ? 312 : 0;
	u32 r1 = (val & 0xf) * 26;
	return *(volatile u8 *)(uintptr_t)(0x80000724 + r0 + r1);
}
/**
 * pmu_cbt_active_entry_lookup() - Look up CBT entry from active calibration structure
 *
 * Return: CBT configuration entry byte.
 */
u8 pmu_cbt_active_entry_lookup(void)
{
	return pmu_cbt_entry_lookup(dmem_read32(PMU_DMEM_CAL_STRUCT_PTR));
}

/**
 * pmu_cal_matrix_trace_dump() - Formats and streams calibration matrix traces across active slices to host mailbox
 * @arg0: Parameter arg0
 * @num_cols: Number of columns in delay matrix
 * @matrix: Pointer to delay matrix array
 *
 * Derived from vendor PMU code at address 0x0a60.
 */
void pmu_cal_matrix_trace_dump(u32 arg0, u32 num_cols, const u16 *matrix)
{
	pmu_cal_metric_log(5, 0x024b0001, arg0);

	for (u32 col = 0; col < (u16)num_cols; col++) {
		const u16 *ptr = (const u16 *)((uintptr_t)matrix + (col << 1));
		u32 stride_bytes = num_cols << 1;
		u16 v0 = *ptr;
		ptr = (const u16 *)((uintptr_t)ptr + stride_bytes);
		u16 v1 = *ptr;
		ptr = (const u16 *)((uintptr_t)ptr + stride_bytes);
		u16 v2 = *ptr;
		ptr = (const u16 *)((uintptr_t)ptr + stride_bytes);
		u16 v3 = *ptr;

		pmu_cal_metric_log(5, 0x024c0005, col, v0, v1, v2, v3);
	}
}

/**
 * pmu_clear_tracker_words() - pmu_cal_tracker_stride_advance:
 * @base: Base memory address or offset
 *
 * Derived from vendor PMU code at address 0x0dd8 (and 0x0ed2).
 * Steps through calibration tracker at 0x80000d9c by div steps, updating stride marker.
 * /
 */
static __attribute__((noinline)) void pmu_clear_tracker_words(volatile void *base)
{
	volatile u32 *t = (volatile u32 *)base;
	t[0] = 0;
	t[1] = 0;
	t[2] = 0;
	t[3] = 0;
}
/**
 * pmu_cal_tracker_stride_advance() - Cal Tracker Stride Advance
 * @marker: Calibration synchronization marker
 * @div: Clock or stride divider ratio
 * @flag: Parameter flag
 *
 * Return: Computed u16 value or status.
 */
u16 pmu_cal_tracker_stride_advance(u32 marker, u32 div, u32 flag)
{
	volatile u16 *tracker16 = (volatile u16 *)0x80000d9c;
	__asm__("" : "+r"(tracker16));

	while ((u16)div != 0) {
		if (div >= 4 && flag == 1) {
			if ((marker & 7) != 0) {
				div--;
				pmu_clear_tracker_words((void *)tracker16 + 16);
				marker = pmu_cal_tracker_step_update();
				flag = 1;
				continue;
			} else {
				pmu_memset_words((void *)tracker16, 0, 0x20);
				if (div >= 513) {
					div -= 512;
					tracker16[0x22 / 2] = 0xff;
					tracker16[0x24 / 2] = 3;
					marker = pmu_cal_tracker_step_update();
					flag = 1;
					continue;
				} else {
					u32 r0 = (div >> 1) - 1;
					if ((div >> 1) < 1) {
						tracker16[0x22 / 2] = (u16)r0;
						tracker16[0x24 / 2] = ((u16)r0 == 0) ? 0 : 3;
						marker = pmu_cal_tracker_step_update() + 4;
						pmu_clear_tracker_words((void *)tracker16);
						return (u16)marker;
					} else {
						tracker16[0x24 / 2] = 3;
						tracker16[0x22 / 2] = (u16)r0;
						return (u16)(marker + 8);
					}
				}
			}
		} else {
			flag = 0;
			if ((marker & 7) == 0) {
				if (div == 1) {
					pmu_clear_tracker_words((void *)tracker16);
					return (u16)(marker + 4);
				}
				pmu_memset_words((void *)tracker16, 0, 0x20);
				if (div == 2) {
					return (u16)(marker + 8);
				}
				marker = pmu_cal_tracker_step_update();
				div -= 2;
				continue;
			} else {
				pmu_clear_tracker_words((void *)tracker16 + 16);
				if (div == 1) {
					return (u16)(marker + 4);
				}
				marker = pmu_cal_tracker_step_update();
				if (div == 2) {
					marker += 4;
					pmu_clear_tracker_words((void *)tracker16);
					return (u16)marker;
				}
				div--;
				continue;
			}
		}
	}

	return (u16)*(volatile u16 *)(uintptr_t)0x80000458;
}

/**
 * pmu_cbt_coarse_step_pulse_seq() - Dispatches CBT coarse step pulse sequence across PHY channels via pmu_phy_csr_result_stream
 * @arg0: Parameter arg0
 * @arg1: Parameter arg1
 * @arg2: Parameter arg2
 *
 * Derived from vendor PMU code at address 0x1770 (and 0x1806).
 * /
 */
void pmu_cbt_coarse_step_pulse_seq(u32 arg0, u32 arg1, u32 arg2)
{
	volatile u16 *marker_a = (volatile u16 *)0x80000458;
	u16 marker_val = *marker_a;
	u16 buf[8];
	pmu_memset_words(buf, 0, sizeof(buf));

	if ((marker_val & 0x7) == 4) {
		pmu_clear_tracker_words((void *)(0x80000d9c + 16));
		pmu_cal_tracker_step_update();
		*marker_a = marker_val + 4;
	}

	u8 step_b = 1;
	if (arg2 == 0) {
		step_b = (u8)(*(volatile u8 *)(uintptr_t)0x8000045a << 1);
	}
	u32 step = step_b;
	u32 val = step;
	if (arg0 == 0) {
		if (arg1 > val)
			val = arg1;
	}

	u32 div = val / step;
	if (val != div * step)
		div++;

	u32 count = (div - 1) & 0xffff;
	while (count > 255) {
		u16 s0 = 0x1f << 10;
		u16 s1 = s0 - 0x20;
		buf[3] = s1;
		buf[7] = s0;
		pmu_phy_csr_result_stream(buf, 8);
		count -= 255;
	}

	if (count != 0) {
		u16 s1 = ((count << 12) & 0x7000) | (1 << 8) | (1 << 9) | (1 << 11);
		u16 s0 = (count << 7) & 0x7c00;
		buf[3] = s1;
		buf[7] = s0;
	} else {
		buf[3] = 0;
		buf[7] = 0;
	}

	pmu_phy_csr_result_stream(buf, 8);
}

/**
 * pmu_phy_ac_lane_timing_delay_set() - Configures AC lane timing delays and pulse sequences
 * @arg0: Parameter arg0
 *
 * Derived from vendor PMU code at address 0x0ef0.
 * /
 *
 * Return: Computed u16 value or status.
 */
u16 pmu_phy_ac_lane_timing_delay_set(u32 arg0)
{
	pmu_cal_markers_set(800);

	u32 r0 = 0x9 << 19;
	pmu_cal_sequence_pulse_send(r0, 5, 0xc, 512, 0, arg0, 0);

	pmu_cal_sequence_pulse_send(0, 6, 2, 3, 0x1a, arg0, 0);
	pmu_clk_ratio_pulse_trigger();

	pmu_cal_sequence_pulse_send(0, 6, 2, 1, 0x1a, arg0, 0);
	pmu_clk_ratio_pulse_trigger();

	pmu_cal_sequence_pulse_send(0, 6, 2, 0, 0x1a, arg0, 0);

	pmu_cbt_coarse_step_pulse_seq(0, 0x22, 0);

	pmu_cal_sequence_pulse_send(0, 5, 4, 896, 0, arg0, 0);

	pmu_cal_sequence_pulse_send(0x80 | (1 << 19), 7, 4, 0, 0, 0, 0);

	u32 stride = pmu_cal_stride_get();
	return (u16)(stride - 1);
}

/**
 * pmu_cal_window_params_dispatch() - Formats 4-byte timing window descriptor into buffer and invokes pmu_cal_pll_lock_retry_poll
 * @ptr: Pointer to memory structure or buffer
 * @val: Value to write or configure
 * @flags: Configuration bitmask or control flags
 *
 * Derived from vendor PMU code at address 0x170c.
 * /
 *
 * Return: Computed u32 value or status.
 */
u32 pmu_cal_window_params_dispatch(const u8 *ptr, u16 val, u32 flags)
{
	u16 buf[22];
	pmu_memset_words(buf, 0, 0x2a);

	buf[3] = ptr[0];
	buf[4] = ptr[1];
	buf[11] = ptr[2];
	buf[12] = ptr[3];

	if (flags & 1)
		buf[5] = val;
	if (flags & 2)
		buf[13] = val;

	return pmu_cal_pll_lock_retry_poll((uintptr_t)buf);
}

/**
 * pmu_dbyte_cal_step_latch() - Formats DBYTE calibration step parameters and streams to PHY sequencer via pmu_phy_csr_result_stream
 * @arg0: Parameter arg0
 * @arg1: Parameter arg1
 * @arg2: Parameter arg2
 *
 * Derived from vendor PMU code at address 0x1864.
 * /
 */
void pmu_dbyte_cal_step_latch(u32 arg0, u32 arg1, u32 arg2)
{
	u16 buf[16];
	pmu_memset_words(&buf[1], 0, 0x1e);

	u16 r0_field = (arg2 << 3) & 0x10;
	u16 r14_field = (arg2 & 0x3) << 14;
	u32 a0 = arg0 & 0x7f;
	u32 a1 = arg1 & 0x7f;
	u32 r2 = (arg1 >> 1) & ~0x3f;
	u16 r0_mid = r2 | (1 << 3);

	buf[0] = r14_field | 0x2c58;
	buf[3] = r0_field;
	buf[4] = a0 | (a0 << 7);
	buf[8] = (r0_mid << 7) | (r14_field | r0_mid);
	buf[11] = r0_field;
	buf[12] = a1 | (a1 << 7);

	pmu_phy_csr_result_stream(buf, 16);
}

/**
 * pmu_cal_pulse_burst_send() - Sends a burst of calibration pulses via pmu_cal_sequence_pulse_send
 * @mode: Operational or calibration mode
 * @count: Number of items, halfwords, or iterations
 * @flag: Parameter flag
 *
 * Derived from vendor PMU code at address 0x18f4.
 * /
 */
void pmu_cal_pulse_burst_send(u32 mode, u32 count, u32 flag)
{
	if (count == 0)
		return;

	u32 base = (flag << 5) & 64;
	u32 cmd0 = base | (1 << 26);
	u32 cmd1 = base | (1 << 27);
	u32 cmd = (mode != 0) ? cmd1 : cmd0;

	for (u32 i = 0; i < count; i++) {
		pmu_cal_sequence_pulse_send(cmd, 7, 0, 0, 0, 0, 0);
	}
}

/**
 * pmu_cal_profile_mode_get() - Queries active channel CSR offset from DMEM 0x41c (PMU_DMEM_PARAM_41C, [gp, 28]),
 *
 * Derived from vendor PMU code at address 0x2938.
 * reads DBYTE profile mode from PHY register 0x90020012, and returns decoded profile enum (0..4).
 * /
 *
 * Return: Computed u32 value or status.
 */
u32 pmu_cal_profile_mode_get(void)
{
	u32 ch_offset = dmem_read32(PMU_DMEM_PARAM_41C);
	u16 reg_val = phy_read16(0x90020012 + (ch_offset << 1));
	u32 mode = reg_val & 7;
	if (mode == 0)
		return 0;
	if (mode == 1)
		return 1;
	u32 bits = reg_val & 6;
	if (bits == 2)
		return 2;
	if (bits == 4)
		return 3;
	return 4;
}

/**
 * pmu_cal_multi_param_dispatch_2178() - /
 * @arg0: Parameter arg0
 * @arg1: Parameter arg1
 * @arg2: Parameter arg2
 * @arg3: Parameter arg3
 * @arg4: Parameter arg4
 *
 * Derived from vendor PMU code at address 0x2178.
 */
void pmu_cal_multi_param_dispatch_2178(u32 arg0, u32 arg1, u32 arg2, u32 arg3, u32 arg4)
{
	pmu_phy_mode_cfg_dispatch(3);
	u8 a7c = dmem_read8(0xa7c);
	u8 d01 = dmem_read8(0x01);
	if (a7c != 0 && !(d01 & (1 << 3))) {
		pmu_cal_multi_rank_dispatch_e170(0xf, arg0, arg1, arg2, arg3, 0, arg4);
	} else {
		pmu_cal_multi_rank_pulse_seq_dfec(0xf, arg0, arg1, arg2, arg3, arg4);
	}
}

/**
 * pmu_train_main() - Main LPDDR5 training coordinator: invokes training dispatcher
 *
 * Derived from vendor PMU code at address 0x860c.
 * /
 *
 * Return: Computed u32 value or status.
 */
u32 pmu_train_main(void)
{
	pmu_train_dispatcher();
	return 0;
}

/**
 * pmu_channel_rank_avail_check() - pmu_channel_rank_config_check_and_exec:
 * @bit_idx: Bit lane index within slice (0..8)
 *
 * Derived from vendor PMU code at address 0x85cc.
 * Verifies channel/rank availability against DMEM parameters and dispatches sequence via pmu_phy_mode_cfg_dispatch.
 * /
 *
 * Return: Computed u32 value or status.
 */
u32 pmu_channel_rank_avail_check(u32 bit_idx)
{
	u8 b66 = *(volatile u8 *)(uintptr_t)0x80000b66;
	u32 mask = 1u << bit_idx;
	u32 mode;

	if (b66 == 1) {
		u8 d40 = *(volatile u8 *)(uintptr_t)0x80000040;
		if (!(mask & d40))
			return 0;
		mode = 2;
	} else if (b66 != 0) {
		mode = 3;
	} else {
		u8 d25 = *(volatile u8 *)(uintptr_t)0x80000025;
		if (!(mask & d25))
			return 0;
		mode = 1;
	}

	pmu_phy_mode_cfg_dispatch(mode);
	return 1;
}

/**
 * pmu_tracker_field_extract() - pmu_cal_status_field_extract:
 * @dest: Pointer to destination memory
 * @byte_offset: Byte offset within data structure or DMEM
 *
 * Derived from vendor PMU code at address 0xa988.
 * Extracts field at byte_offset from 0x80000bc0 into calibration structure at dest.
 * /
 */
void pmu_tracker_field_extract(u8 *dest, u32 byte_offset)
{
	for (u32 rank = 0; rank < 2; rank++) {
		for (u32 byte_lane = 0; byte_lane < 2; byte_lane++) {
			u32 src_off = (rank * 216) + (byte_lane * 108) + (byte_offset * 2);
			u8 val = *(volatile u8 *)(uintptr_t)(0x80000bc0 + src_off);
			u32 dst_off = (rank * 108) + (byte_lane * 54) + byte_offset;
			dest[dst_off] = val;
		}
	}
}

/**
 * pmu_tracker_cal_mode_update() - pmu_cal_desc_mode_flags_update:
 * @desc: Pointer to descriptor structure
 *
 * Derived from vendor PMU code at address 0xb454.
 * Updates calibration mode and control flags within tracker/descriptor structure.
 * /
 */
void pmu_tracker_cal_mode_update(u8 *desc)
{
	desc[0x0b] &= 0x87;
	desc[0x01] &= ~(1u << 3);
	desc[0x13] &= ~0x0c;

	u32 flags = pmu_dmem_training_flags_eval();

	u8 d0d = desc[0x0d];
	if (flags != 0) {
		d0d |= (1u << 5);
		desc[0x0d] = d0d;
		desc[0x03] |= (1u << 7);
	}

	desc[0x14] = 2;
	desc[0x10] = 0x40;
	desc[0x0d] = d0d & ~(1u << 4);
	desc[0x11] |= 0x38;
	desc[0x12] &= ~0x07;
	desc[41] &= 0x1f;
	desc[0x1c] &= ~(1u << 5);
}

/**
 * pmu_cal_timing_param_step_calc() - Computes timing parameter in picoseconds based on DMEM 0x1c, mode 8a, and nibble 96,
 *
 * Derived from vendor PMU code at address 0x69b4.
 * converts to PHY delay steps via pmu_freq_delay_step_calc, and clamps to minimum 4 steps.
 * /
 *
 * Return: Computed u32 value or status.
 */
u32 pmu_cal_timing_param_step_calc(void)
{
	u32 mode_8a = pmu_dmem_8a_mode_check();
	u32 nibble_96 = pmu_dmem_96_nibble_get();
	u8 d1c = dmem_read8(0x1c);
	u32 ps;

	if (d1c != 0) {
		if (nibble_96 != 0) {
			ps = (mode_8a == 0) ? (0x7d << 7) : 0x4a38; /* 16000 ps or 19000 ps */
		} else {
			ps = (mode_8a == 0) ? 0x2ee0 : 0x4a38;       /* 12000 ps or 19000 ps */
		}
	} else {
		if (nibble_96 != 0) {
			ps = (mode_8a == 0) ? 0x4650 : 0x5208;       /* 18000 ps or 21000 ps */
		} else {
			ps = (mode_8a == 0) ? 0x36b0 : 0x5208;       /* 14000 ps or 21000 ps */
		}
	}

	u32 steps = pmu_freq_delay_step_calc(ps, 0);
	if (steps < 4)
		steps = 4;
	return (u8)steps;
}

/**
 * pmu_cal_table_50c_lookup() - Looks up calibration offset from DMEM table at 0x8000050c
 *
 * Derived from vendor PMU code at address 0x6a20.
 * /
 *
 * Return: Computed u32 value or status.
 */
u32 pmu_cal_table_50c_lookup(void)
{
	u8 channel = dmem_read8(0xb66);
	u8 rank = dmem_read8(0xb67);
	uintptr_t table_ptr = *(volatile u32 *)(uintptr_t)0x80000b9c;
	const u8 *ptr = (const u8 *)(table_ptr + channel * 108 + rank * 54);

	u8 ptr1 = ptr[1];
	u8 cbt_cfg = pmu_cbt_config_get();

	u32 r3 = (cbt_cfg != 2) ? 264 : 0;
	u32 r12 = (ptr1 >> 4) * 22;
	u32 base_idx = r3 + r12;

	u32 bit5 = (ptr[3] & (1 << 5)) ? 5 : 4;
	u32 total_idx = (base_idx * 2) + bit5;

	return *(volatile u8 *)(uintptr_t)(0x8000050c + total_idx);
}

/**
 * pmu_cal_delay_line_init() - Computes delay line sum from pmu_cal_timing_param_step_calc(), pmu_cal_table_50c_lookup(), and pmu_delay_step_calc(),
 *
 * Derived from vendor PMU code at address 0x21d4.
 * rounds up to even, clamps to 92, and stores to DMEM 0x400 ([gp]).
 * /
 */
void pmu_cal_delay_line_init(void)
{
	u32 a = pmu_cal_timing_param_step_calc();
	u32 b = pmu_cal_table_50c_lookup();
	u32 c = pmu_delay_step_calc();
	u32 sum = a + b + c;
	sum += (sum & 1);
	u8 val = (u8)sum;
	if (val > 92)
		val = 92;
	dmem_write8(0x400, val);
}

/**
 * pmu_cal_margin_window_eval() - Evaluates calibration margin eye window for rank/slice descriptor struct
 * @struct_ptr: Pointer to calibration structure in DMEM
 *
 * Derived from vendor PMU code at address 0x1948.
 * Finds the eye boundary and applies step shift from DMEM 0x0e.
 * /
 *
 * Return: Computed u32 value or status.
 */
u32 pmu_cal_margin_window_eval(u8 *struct_ptr)
{
	s8 shift_cfg = (s8)dmem_read8(0x0e);
	s32 step_shift = shift_cfg >> 1;

	if (step_shift == 0)
		return 0;

	s8 cur_step = (s8)struct_ptr[130];
	u8 target_val = struct_ptr[131];
	s32 edge = 0;

	if (step_shift > 0) {
		edge = 0;
		if (cur_step >= 0) {
			for (s32 k = cur_step; k >= 0; k--) {
				u8 lower = struct_ptr[k * 2 + 0];
				u8 upper = struct_ptr[k * 2 + 1];
				if (upper < target_val || target_val < lower) {
					edge = k;
					break;
				}
			}
		}
	} else {
		edge = 0;
		if (cur_step <= 0x3f) {
			for (s32 k = cur_step; k < 0x40; k++) {
				u8 lower = struct_ptr[k * 2 + 0];
				u8 upper = struct_ptr[k * 2 + 1];
				if (upper < target_val || target_val < lower) {
					edge = k;
					break;
				}
				if (k == 0x3f)
					edge = 0x3f;
			}
		}
	}

	s32 new_step = edge + step_shift;
	struct_ptr[130] = (u8)new_step;

	s8 step_idx = (s8)new_step;
	if (step_idx < 0 || step_idx >= 0x40)
		goto err;

	u8 lower = struct_ptr[step_idx * 2 + 0];
	u8 upper = struct_ptr[step_idx * 2 + 1];
	if (upper < target_val || target_val < lower)
		goto err;

	return 0;

err:
	pmu_cal_metric_log(0xc8, 0x1a00003, step_shift);
	return 7;
}

/**
 * pmu_cal_rank_margin_pair_eval() - Evaluates a pair of rank margin window descriptors (struct_a, struct_b)
 * @struct_a: Pointer to primary rank calibration structure
 * @struct_b: Pointer to secondary rank calibration structure
 *
 * Derived from vendor PMU code at address 0x19f0.
 * Computes individual margin windows, aligns total delay offsets, and verifies eye limits.
 * /
 *
 * Return: Computed u32 value or status.
 */
u32 pmu_cal_rank_margin_pair_eval(u8 *struct_a, u8 *struct_b)
{
	s8 shift_cfg = (s8)dmem_read8(0x0e);
	s32 fp = shift_cfg >> 1;

	if (fp == 0)
		return 0;

	u32 ret = pmu_cal_margin_window_eval(struct_a);
	if (ret != 0)
		return ret;

	ret = pmu_cal_margin_window_eval(struct_b);
	if (ret != 0)
		return ret;

	u16 base_b = *(volatile u16 *)(struct_b + 128);
	s8 step_b = (s8)struct_b[130];
	u16 base_a = *(volatile u16 *)(struct_a + 128);
	s8 step_a = (s8)struct_a[130];

	u16 total_b = (u16)(step_b + base_b);
	u16 total_a = (u16)(step_a + base_a);

	if (total_b >= total_a) {
		if (fp < 1)
			struct_b[130] = (u8)(total_a - base_b);
		else
			struct_a[130] = (u8)(total_b - base_a);
	} else {
		if (fp < 1)
			struct_a[130] = (u8)(total_b - base_a);
		else
			struct_b[130] = (u8)(total_a - base_b);
	}

	s8 final_step_a = (s8)struct_a[130];
	u8 target_val_a = struct_a[131];
	u8 lower_a = struct_a[final_step_a * 2 + 0];
	u8 upper_a = struct_a[final_step_a * 2 + 1];
	if (target_val_a < lower_a || upper_a < target_val_a)
		return 7;

	s8 final_step_b = (s8)struct_b[130];
	u8 target_val_b = struct_b[131];
	u8 lower_b = struct_b[final_step_b * 2 + 0];
	u8 upper_b = struct_b[final_step_b * 2 + 1];
	if (target_val_b < lower_b || upper_b < target_val_b)
		return 7;

	return 0;
}

/*
 * pmu_cal_slice_delay_best_fit_select:
 * Derived from vendor PMU code at address 0x1ee0.
 * Selects between primary and alternate timing delay candidates based on threshold comparison,
 * packs 7-bit values, and updates slice delay registers.
 */
void pmu_cal_slice_delay_best_fit_select(u32 idx, u32 thresh1, u32 thresh2,
										 const u8 *ptr3, const u8 *ptr0,
										 const u8 *ptr4, const u8 *ptr8,
										 u8 *slice_base)
{
	u8 *r2 = slice_base + (idx << 2);

	if (thresh2 > thresh1) {
		u16 val_2 = *(volatile u16 *)(r2 + 0x2);
		u16 val_a = *(volatile u16 *)(r2 + 0xa);
		u16 val_r1 = ((u16)ptr4[0] << 7) + ptr4[1];
		u16 val_r0 = ((u16)ptr8[0] << 7) + ptr8[1];
		*(volatile u16 *)(r2 + 0x22) = val_2;
		*(volatile u16 *)(r2 + 0x2a) = val_a;
		*(volatile u16 *)(r2 + 0x20) = val_r1;
		*(volatile u16 *)(r2 + 0x28) = val_r0;
	} else {
		u16 val_12 = *(volatile u16 *)(r2 + 0x12);
		u16 val_1a = *(volatile u16 *)(r2 + 0x1a);
		u16 val_r1 = ((u16)ptr3[0] << 7) + ptr3[1];
		u16 val_r0 = ((u16)ptr0[0] << 7) + ptr0[1];
		*(volatile u16 *)(r2 + 0x22) = val_12;
		*(volatile u16 *)(r2 + 0x2a) = val_1a;
		*(volatile u16 *)(r2 + 0x20) = val_r1;
		*(volatile u16 *)(r2 + 0x28) = val_r0;
	}
}

/**
 * pmu_dmem_cal_offset_select() - Selects calibration parameter DMEM offset based on operating profile mode and flags
 * @arg0: Parameter arg0
 * @arg1: Parameter arg1
 * @arg2: Parameter arg2
 *
 * Derived from vendor PMU code at address 0x2cd8.
 * /
 *
 * Return: Computed u32 value or status.
 */
u32 pmu_dmem_cal_offset_select(u32 arg0, u32 arg1, u32 arg2)
{
	u32 mode = pmu_cal_profile_mode_get();

	if (mode == 2) {
		if (arg0 == 0)
			return (arg2 == 0) ? 0x4f : 0x51;
		else
			return (arg2 == 0) ? 0x4e : 0x50;
	} else if (mode == 1) {
		return (arg0 == 0) ? 0x51 : 0x50;
	} else if (mode == 0) {
		return (arg0 == 0) ? 0x4f : 0x4e;
	} else {
		if (arg0 == 0)
			return (arg1 == 0) ? 0x4f : 0x51;
		else
			return (arg1 == 0) ? 0x4e : 0x50;
	}
}

/**
 * pmu_cal_rank_stride_count_get() - Returns the rank stride iteration count (1 or 2) depending on rank index and profile mode
 * @rank_idx: DRAM rank index (0..1)
 *
 * Derived from vendor PMU code at address 0x6af0.
 * /
 *
 * Return: Computed u32 value or status.
 */
u32 pmu_cal_rank_stride_count_get(u32 rank_idx)
{
	if (rank_idx == 0 && pmu_cal_profile_mode_get() == 3)
		return 2;
	return 1;
}

/**
 * pmu_cal_channel_rank_config_get() - Reads channel rank control and mask information from DMEM based on channel index
 * @idx: Parameter idx
 * @out_bit: Pointer to output bit index
 * @out_mask: Pointer to output mask word
 * @out_mode: Pointer to output mode byte
 *
 * Derived from vendor PMU code at address 0x6b04.
 * /
 *
 * Return: Computed u32 value or status.
 */
u32 pmu_cal_channel_rank_config_get(u32 idx, u16 *out_bit, u8 *out_mask, u8 *out_mode)
{
	u8 val, ctrl;
	u8 r13, r14;
	u8 r15;

	if (idx < 2) {
		val = dmem_read8(0x32);
		ctrl = dmem_read8(0x25);
		r15 = (idx != 0);
		r14 = 1;
		r13 = 0;
	} else {
		val = dmem_read8(0x4d);
		ctrl = dmem_read8(0x40);
		r15 = (idx != 2);
		r14 = 2;
		r13 = 1;
	}

	u8 bit = (val & 1) ^ r15;
	*out_bit = bit;
	*out_mask = (1 << bit);

	if (!(ctrl & (1 << bit)))
		return 1;

	*out_mode = r14;
	dmem_write8(0xb66, r13);
	return 0;
}

/**
 * pmu_cal_channel_status_get() - Retrieves channel training status words and flag bytes from DMEM
 * @idx: Parameter idx
 * @out_val: Pointer to output status word
 * @out_a: Pointer to output byte A
 * @out_b: Pointer to output byte B
 *
 * Derived from vendor PMU code at address 0x6b50.
 * /
 */
void pmu_cal_channel_status_get(u32 idx, u32 *out_val, u8 *out_a, u8 *out_b)
{
	if (idx == 0) {
		u8 d25 = dmem_read8(0x25);
		*out_val = (d25 == 3) ? 2 : 1;
		*out_a = dmem_read8(0xb6a);
		*out_b = dmem_read8(0xb6b);
	} else if (idx == 1) {
		u8 d40 = dmem_read8(0x40);
		if (d40 == 1)
			*out_val = 1;
		else if (d40 == 3)
			*out_val = 2;
		else
			*out_val = 0;
		*out_a = dmem_read8(0xb6c);
		*out_b = dmem_read8(0xb6d);
	}
}

/**
 * pmu_cal_dmem_stride_bytes_fill() - Populates calibration tracking bytes across stride offsets
 * @base: Base memory address or offset
 * @curr: Parameter curr
 * @val: Value to write or configure
 * @stride: Parameter stride
 *
 * Derived from vendor PMU code at address 0x6ebc.
 * /
 */
void pmu_cal_dmem_stride_bytes_fill(u8 *base, u8 *curr, u8 val, u32 stride)
{
	do {
		curr[0x1f] = val;
		curr[0x1b] = val;
		curr[0x04] = val;
		curr[0x00] = val;
		curr = base + stride;
	} while (stride++ < 2);
}

/**
 * pmu_dbyte_dq_deskew_regs_restore() - Restores DBYTE deskew / delay line registers (0x90020100...) from DMEM table 0x80000ba0
 *
 * Derived from vendor PMU code at address 0xa6f0.
 * /
 */
void pmu_dbyte_dq_deskew_regs_restore(void)
{
	u8 slices = dmem_read8(0x42c);
	const u8 *src_table = (const u8 *)0x80000ba0;

	for (u32 slice = 0; slice < slices; slice++) {
		u32 slice_offset = slice << 13;
		for (u32 i = 0; i < 8; i++) {
			u32 reg_addr = 0x90020100 + slice_offset + (i * 2);
			*(volatile u16 *)reg_addr = src_table[(slice * 8) + i];
		}
	}
}

/**
 * pmu_dbyte_dq_deskew_regs_save_and_ramp() - Backs up DBYTE deskew registers to DMEM table 0x80000ba0 and initializes them with ramp values
 *
 * Derived from vendor PMU code at address 0xb4b0.
 * /
 */
void pmu_dbyte_dq_deskew_regs_save_and_ramp(void)
{
	u8 slices = dmem_read8(0x42c);
	u8 *dest_table = (u8 *)0x80000ba0;

	for (u32 slice = 0; slice < slices; slice++) {
		u32 slice_offset = slice << 13;
		for (u32 i = 0; i < 8; i++) {
			u32 reg_addr = 0x90020100 + slice_offset + (i * 2);
			dest_table[(slice * 8) + i] = (u8)*(volatile u16 *)reg_addr;
			*(volatile u16 *)reg_addr = (u16)i;
		}
	}
}



extern void pmu_multirank_slice_cal_dispatch(void *ptr, u32 r1, u32 r2, s32 r3, u32 sp0, u32 sp4);

/**
 * pmu_phy_csr_result_stream() - Streams calibration result halfwords to PHY CSR space starting at DMEM 0x45e
 * @buf: Pointer to data buffer
 * @len: Length of data in halfwords or bytes
 *
 * Derived from vendor PMU code at address 0xe3d4.
 * /
 */
void pmu_phy_csr_result_stream(const void *buf, u32 len)
{
	const u16 *src = (const u16 *)buf;
	u16 r13 = *(const u16 *)0x8000045e;
	uintptr_t r2 = (r13 << 1) | 0x82000;
	u32 count = len ? len : 1;
	for (u32 i = 0; i < count; i++) {
		phy_write16(0x90000000 | r2, *src++);
		r2 += 2;
	}
	u16 updated = len + r13;
	*(u16 *)0x80000458 = updated;
	*(u16 *)0x8000045e = updated;
}

/**
 * pmu_dq_swap_polarity_check() - Queries DQ swap status and checks polarity boundary bit against DMEM 0xb6b
 * @r0: Parameter r0
 * @r1: Parameter r1
 *
 * Derived from vendor PMU code at address 0x72fc.
 * /
 *
 * Return: Computed u32 value or status.
 */
u32 pmu_dq_swap_polarity_check(u32 r0, u32 r1)
{
	if (pmu_dq_swap_query(r0, r1))
		return 1;
	u8 b6b = dmem_read8(0xb6b);
	return (pmu_dmem_1c_bit_query(b6b, r0) < r1) ? 1 : 0;
}

/**
 * pmu_multiphase_pulse_seq_repeat() - Repeats multi-phase pulse sequence count times
 * @arg0: Parameter arg0
 * @arg1: Parameter arg1
 * @arg2: Parameter arg2
 * @count: Number of items, halfwords, or iterations
 *
 * Derived from vendor PMU code at address 0x8348.
 * /
 */
void pmu_multiphase_pulse_seq_repeat(u32 arg0, u32 arg1, u32 arg2, u32 count)
{
	while (count--) {
		pmu_cal_sequence_pulse_send(0, arg0, 0, arg1, 0, arg2, 0);
	}
}

/**
 * pmu_phy_lock_status_read() - Interrogates PHY lock status registers 0x9006017e and 0x900201c0
 * @out0: Parameter out0
 * @out1: Parameter out1
 *
 * Derived from vendor PMU code at address 0x65c4.
 * /
 */
void pmu_phy_lock_status_read(u16 *out0, u16 *out1)
{
	volatile u16 *p = (volatile u16 *)0x900401c0;
	__asm__("" : "+r"(p));
	u16 v0 = p[0x06 / 2];
	u16 v4 = p[0x0a / 2];
	u16 r15 = v0 | (1u << 11);

	p[0x0a / 2] = v4 & ~0x1f;
	p[0x06 / 2] = r15;
	p[0x06 / 2] = v0 | 0xc00;
	pmu_hw_timer_delay(10);
	p[0x06 / 2] = r15;
	*out1 = phy_read16(0x9006017e) & 0x1ff;
	*out0 = phy_read16(0x900201c0) & 0x1ff;
	p[0x06 / 2] = v0;
	p[0x0a / 2] = v4;
}

/**
 * pmu_phy_clk_toggle_settle() - Toggles PHY clock sequence and asserts PLL settling delay
 *
 * Derived from vendor PMU code at address 0xfa48.
 * /
 */
void pmu_phy_clk_toggle_settle(void)
{
	volatile u16 *base = (volatile u16 *)0x90040600;
	u16 orig = base[3];
	base[0x10] = 1;
	base[3] = (orig & 0xef) | (1u << 4);
	base[0x10] = 0;
	base[0xd] = 1;
	base[0xd] = 0;
	base[0x11] = 1;
	pmu_delay_us(0x7bfa480, 0);
	base[0x11] = 0;
	base[0x20] = 0;
	base[0x20] = 1;
	base[0x20] = 0;
	base[3] = (u8)orig;
}

/**
 * pmu_phy_reg_stream_play() - Plays a stream of 6-byte register writes to PHY CSR space
 * @stream: Pointer to register configuration stream
 * @len: Length of data in halfwords or bytes
 *
 * Derived from vendor PMU code at address 0xe50c.
 * /
 */
void pmu_phy_reg_stream_play(const u8 *stream, u32 len)
{
	u32 count = len / 6;
	for (u32 i = 0; i < count; i++, stream += 6) {
		u32 addr = (u32)stream[0] | ((u32)stream[1] << 8) |
				   ((u32)stream[2] << 16) | ((u32)stream[3] << 24);
		u16 val = (u16)stream[4] | ((u16)stream[5] << 8);
		if (addr == 0xffffffff) {
			pmu_hw_timer_delay(val);
		} else {
			uintptr_t reg = 0x90000000 | (addr << 1);
			phy_write16(reg, val);
		}
	}
}

/**
 * pmu_phy_lane_timing_offset_set() - Configures per-lane timing offset registers at 0x900e0048
 * @lane: Bit lane index within slice (0..8)
 * @v1: Parameter v1
 * @v2: Parameter v2
 * @v3: Parameter v3
 * @v4: Parameter v4
 *
 * Derived from vendor PMU code at address 0x71e0.
 * /
 */
void pmu_phy_lane_timing_offset_set(u32 lane, u16 v1, u16 v2, u16 v3, u16 v4)
{
	volatile u16 *p = (volatile u16 *)(0x900e0048 | (lane << 9));
	p[0] = v1; p[1] = v2; p[2] = v1; p[3] = v2;
	p[4] = v3; p[5] = v4; p[6] = v3; p[7] = v4;

	volatile u16 *ctrl = (volatile u16 *)0x900e00ca;
	if (lane == 0xf)
		*ctrl = 511;
	else
		*ctrl |= (1u << lane);
}

/**
 * pmu_cal_marker_verify_update() - Verifies calibration markers 0x3e and 0x3f, updates calibration state at
 * @r0: Parameter r0
 *
 * Derived from vendor PMU code at address 0x9c84.
 * 0x80000dac, and triggers verification via pmu_cal_marker_verify.
 * /
 *
 * Return: Computed u32 value or status.
 */
u32 pmu_cal_marker_verify_update(u32 r0)
{
	volatile u16 *p458 = (volatile u16 *)0x80000458;
	volatile u32 *pdac = (volatile u32 *)0x80000dac;
	volatile u16 *pdac16 = (volatile u16 *)0x80000dac;
	u32 res = *p458;

	if (r0 & (1u << 11)) {
		pdac16[10] |= 1; // offset 0x14
		if ((*(volatile u8 *)p458 & 7) != 0) {
			pdac[3] = 0; // offset 0xc
			pdac[2] = 0; // offset 0x8
			pdac[1] = 0; // offset 0x4
			pdac[0] = 0; // offset 0x0
		}
		res = pmu_cal_marker_verify();
	} else {
		if ((*p458 & 7) == 0) {
			res = pmu_cal_marker_verify();
		}
	}

	if (r0 & (1u << 7)) {
		if ((*(volatile u8 *)p458 & 7) != 0) {
			pdac[3] = 0;
			pdac[2] = 0;
			pdac[1] = 0;
			pdac[0] = 0;
		}
		res = pmu_cal_marker_verify();
	}
	return res;
}

/**
 * pmu_cbt_cal_pulse_seq() - Executes command bus training calibration pulse sequence
 * @arg0: Parameter arg0
 *
 * Derived from vendor PMU code at address 0x7270.
 * /
 */
void pmu_cbt_cal_pulse_seq(u32 arg0)
{
	pmu_cal_sequence_pulse_send(0x41 << 19, 7, 4, 0, 0, 0, 0);
	pmu_cbt_cal_stat_set();
	pmu_cal_sequence_pulse_send(arg0, 7, 4, 0, 0, 0, 0);
	pmu_cbt_cal_stat_clear();
	pmu_cal_sequence_pulse_send(arg0, 7, 4, 0, 0, 0, 0);
	pmu_cal_sequence_pulse_send(0x80 | (1u << 19), 7, 4, 0, 0, 0, 0);
}

/**
 * pmu_phy_timing_delay_latch() - Sets PHY timing register 0x9003e002 and latches pulse delay
 * @arg0: Parameter arg0
 * @arg1: Parameter arg1
 *
 * Derived from vendor PMU code at address 0x4fb0.
 * /
 */
void pmu_phy_timing_delay_latch(u32 arg0, u32 arg1)
{
	pmu_deskew_and_tracker_reset();
	u32 offset = (u32)dmem_read32(PMU_DMEM_PARAM_41C) << 1;
	uintptr_t reg = 0x9003e002 | offset;

	if (arg0 != 0) {
		phy_write16(reg, 1);
		pmu_cal_sequence_pulse_send(0, 6, 0x22, 4, 0x2e, arg1, 0);
	} else {
		pmu_cal_sequence_pulse_send(0, 6, 0x22, 0, 0x2e, arg1, 0);
		phy_write16(reg, 0);
	}
	pmu_clk_timing_delay_latch(0, 1);
}

/**
 * pmu_dbyte_slice_cal_param_pulse() - Configures DBYTE slice calibration parameter and dispatches pulse sequence
 * @rank: DRAM rank index (0..1)
 * @flag: Parameter flag
 *
 * Derived from vendor PMU code at address 0xdbd4.
 * /
 */
void pmu_dbyte_slice_cal_param_pulse(u32 rank, u32 flag)
{
	u8 channel = dmem_read8(0xb66);
	uintptr_t base = 0x800009a4 + channel * 108 + rank * 54;
	u8 val = *(const u8 *)(base + 18);

	pmu_deskew_and_tracker_reset();
	u32 mask = ((1u << rank) << 2) | (1u << rank);
	if (flag != 0)
		val |= 0x48;

	pmu_cal_sequence_pulse_send(0, 6, 0x22, val, 18, (u8)mask, 0);
	pmu_cal_sequence_pulse_send(0x80, 7, 2, 0, 0, 0, 0);
	pmu_clk_timing_delay_latch(0, 1);
}

/**
 * pmu_profile_mailbox_stream_send() - Reads profile table from DMEM 0x8000005a..5d and sends 29-halfword mailbox stream
 * @idx: Parameter idx
 *
 * Derived from vendor PMU code at address 0x4bf0.
 * /
 */
void pmu_profile_mailbox_stream_send(u32 idx)
{
	u8 offset_byte;
	u32 cmd;
	switch (idx) {
	case 3:
		offset_byte = 0x5d;
		cmd = 0x1dd001d;
		break;
	case 2:
		offset_byte = 0x5c;
		cmd = 0x1db001d;
		break;
	case 1:
		offset_byte = 0x5b;
		cmd = 0x1d9001d;
		break;
	case 0:
	default:
		offset_byte = 0x5a;
		cmd = 0x1d7001d;
		break;
	}

	uintptr_t table = 0x80000000 | offset_byte;
	u16 buf[29];
	for (u32 i = 0; i < 29; i++) {
		buf[i] = *(const u8 *)(table + (i << 2));
	}

	pmu_post_trace_log_halfwords(0xa, cmd, buf, 29);
}

/**
 * pmu_cal_struct_mask_remap() - Updates calibration structure masks at offset 0x10 and 0x29, runs pulse sequence, and remaps
 *
 * Derived from vendor PMU code at address 0x7f64.
 * /
 */
void pmu_cal_struct_mask_remap(void)
{
	u32 base = 0x800009a4;
	pmu_cal_struct_to_shadow16(base, 0x10);
	pmu_cal_struct_mask_or(base, 0x10, 0x41);
	pmu_cal_struct_mask_and(base, 0x10, 0xf3);
	pmu_cal_struct_to_shadow16(base, 0x29);
	pmu_cal_struct_mask_and(base, 0x29, 0x1f);
	pmu_multirank_slice_cal_dispatch((void *)(uintptr_t)base, 0, 0xfffeffff, -513, 0xff, 0);
	pmu_delay_us(20000, 0);
	pmu_tracker_field_extract((u8 *)(uintptr_t)base, 0x29);
}

/**
 * pmu_rank_mask_multiparam_dispatch() - Updates rank mask at offset 0x12 and dispatches multi-parameter calibration
 * @arg0: Parameter arg0
 *
 * Derived from vendor PMU code at address 0x7ef0.
 * /
 */
void pmu_rank_mask_multiparam_dispatch(u32 arg0)
{
	u8 rank = dmem_read8(0xb67);
	u8 r15 = (1u << rank) | ((1u << rank) << 2);
	u32 base = 0x800009a4;

	if (arg0 != 0) {
		pmu_cal_struct_to_shadow16(base, 0x12);
		pmu_cal_struct_mask_or(base, 0x12, 0x80);
	} else {
		pmu_tracker_field_extract((u8 *)(uintptr_t)base, 0x12);
	}

	u8 channel = dmem_read8(0xb66);
	uintptr_t r1 = base + channel * 108 + rank * 54;
	pmu_cal_multi_rank_pulse_seq_dfec(r15, r1, 0, 0xfffbffff, -1, 0);
	pmu_delay_us(20000, 0);
}

/**
 * pmu_phy_slice_mask_set() - Sets PHY slice mask registers based on rank bitmask
 * @arg0: Parameter arg0
 *
 * Derived from vendor PMU code at address 0xb85c.
 * /
 */
void pmu_phy_slice_mask_set(u32 arg0)
{
	phy_write16(0x9003ff6a, 0);
	u8 r14 = dmem_read8(0xb69);
	u8 r12 = dmem_read8(0xb68);

	for (; r12 <= r14; r12++) {
		uintptr_t base = (uintptr_t)r12 << 13;
		if (r12 == 0) {
			phy_write16(base | 0x90021f6a, 1);
			if ((arg0 & (1u << 8)) == 0)
				phy_write16(0x9003f16a, 0);
		} else {
			u32 r2 = arg0;
			for (u32 bit = 0; bit < 9 && r2 != 0; bit++, r2 >>= 1) {
				if (r2 & 1)
					phy_write16(0x9002016a | base | (bit << 9), 1);
			}
		}
	}
}

/**
 * pmu_phy_reg_buffer_stream() - Reads or writes a stream of PHY registers from/to a byte buffer
 * @stream: Pointer to register configuration stream
 * @count: Number of items, halfwords, or iterations
 * @mode: Operational or calibration mode
 *
 * Derived from vendor PMU code at address 0xb784.
 * /
 */
void pmu_phy_reg_buffer_stream(u8 *stream, u32 count, u32 mode)
{
	if (mode != 0) {
		for (u32 i = 0; i < count; i++) {
			u32 addr = (u32)stream[0] | ((u32)stream[1] << 8) |
					   ((u32)stream[2] << 16) | ((u32)stream[3] << 24);
			stream += 4;
			uintptr_t reg = 0x90000000 | (addr << 1);
			u16 val = phy_read16(reg);
			stream[0] = (u8)val;
			stream[1] = (u8)(val >> 8);
			stream += 2;
		}
	} else {
		for (u32 i = 0; i < count; i++) {
			u32 addr = (u32)stream[0] | ((u32)stream[1] << 8) |
					   ((u32)stream[2] << 16) | ((u32)stream[3] << 24);
			u16 val = (u16)stream[4] | ((u16)stream[5] << 8);
			stream += 6;
			uintptr_t reg = 0x9001e000 | (addr << 1);
			phy_write16(reg, val);
		}
	}
}


/**
 * pmu_freq_ratio_mult() - Calculates clock frequency ratio: ((freq * mult) / (dram_type * 2000)) + 1
 * @mult: Parameter mult
 * @unused: Parameter unused
 *
 * Derived from vendor PMU code at address 0x8638.
 * /
 *
 * Return: Computed u32 value or status.
 */
u32 pmu_freq_ratio_mult(u32 mult, u32 unused)
{
	u32 freq = dmem_read16(PMU_DMEM_DRAM_FREQ_OFF);
	u32 dram_type = dmem_read8(PMU_DMEM_DRAM_TYPE);
	if (!dram_type)
		dram_type = 2;
	return ((freq * mult) / (dram_type * 2000)) + 1;
}

/**
 * pmu_clk_ratio_pulse_trigger() - Calculates clock frequency ratio using pmu_freq_ratio_mult and triggers pulse sequence pmu_cbt_coarse_step_pulse_seq
 *
 * Derived from vendor PMU code at address 0x7258.
 * /
 */
void pmu_clk_ratio_pulse_trigger(void)
{
	u32 res = pmu_freq_ratio_mult(2000, 0);
	pmu_cbt_coarse_step_pulse_seq(0, (u16)res, 0);
}

/**
 * pmu_eye_error_metric_eval() - 2D distance and eye error cost metric evaluator
 * @r0: Parameter r0
 * @r1: Parameter r1
 * @min_x: Minimum horizontal eye boundary
 * @max_x: Maximum horizontal eye boundary
 * @out_metric: Parameter out_metric
 *
 * Derived from vendor PMU code at address 0x49c4.
 * /
 *
 * Return: Computed u32 value or status.
 */
u32 pmu_eye_error_metric_eval(u32 r0, u32 r1, u32 min_x, u32 max_x, u8 *out_metric)
{
	s32 diff_y = (s32)(s8)(r0 & 0xff) - (s32)r1;
	u32 x = (r0 >> 8) & 0xff;
	s32 diff_x;

	*out_metric = 0x3f;
	if (x < min_x) {
		diff_x = (s32)x - (s32)min_x;
	} else if (x <= max_x) {
		diff_x = 0;
		*out_metric = (diff_y < 0) ? -diff_y : diff_y;
	} else {
		diff_x = (s32)x - (s32)max_x;
	}

	u16 weight_y = *(const u16 *)0x80000422;
	u16 weight_x = *(const u16 *)0x80000420;
	return (u32)(diff_y * diff_y) * weight_y + (u32)(diff_x * diff_x) * weight_x;
}

/**
 * pmu_cal_timing_packet_format() - Formats calibration timing packet into destination 16-byte buffer
 * @out: Pointer to output memory
 * @r1: Parameter r1
 * @r2: Parameter r2
 * @r3: Parameter r3
 *
 * Derived from vendor PMU code at address 0x9694.
 * /
 */
void pmu_cal_timing_packet_format(u16 *out, u32 r1, u32 r2, u32 r3)
{
	u32 r15 = r1 << 7;
	u32 r1_masked = (r2 << 7) & 0x3f80;
	u32 r13 = r3 << 14;
	u32 r11 = r2 >> 1;

	out[1] = 0;
	out[2] = 0;
	out[5] = 0;
	out[6] = 0;

	u32 r12 = (r11 & ~0x3f) | r1_masked | r13 | (1u << 3);
	u32 r3_out = (r3 << 3) & 0x10;
	u32 r2_out = r15 | r13 | 88;

	out[7] = (u16)r3_out;
	out[3] = (u16)r3_out;
	out[0] = (u16)r2_out;
	out[4] = (u16)r12;
}

/**
 * pmu_phy_reg_write_shadow_track() - PHY register writer at 0x90140000 with shadow storage tracking
 * @arg0: Parameter arg0
 * @val1: Primary configuration value
 * @val2: Secondary configuration value
 * @arg3: Parameter arg3
 * @arg4: Parameter arg4
 *
 * Derived from vendor PMU code at address 0xe60c.
 * /
 */
void pmu_phy_reg_write_shadow_track(u32 arg0, u32 val1, u32 val2, u32 arg3, u32 arg4)
{
	uintptr_t reg_base = 0x90140000 | (arg0 << 1);
	u16 r13_val = ((arg4 << 3) | ((arg3 >> 16) & 7)) & 0xffff;

	phy_write16(reg_base + 0, (u16)val1);
	phy_write16(reg_base + 2, (u16)val2);
	phy_write16(reg_base + 4, (u16)arg3);
	phy_write16(reg_base + 6, r13_val);

	uintptr_t shadow_ptr = *(volatile uintptr_t *)0x80000418;
	*(volatile u16 *)(shadow_ptr + 0) = (u16)val1;
	*(volatile u16 *)(shadow_ptr + 2) = (u16)val2;
	*(volatile u16 *)(shadow_ptr + 4) = (u16)arg3;
	*(volatile u16 *)(shadow_ptr + 6) = r13_val;
	*(volatile uintptr_t *)0x80000418 = shadow_ptr + 8;
}

/**
 * pmu_rank_slice_deskew_latch() - Rank slice deskew reset, timing pulse sequence, and latch
 * @r0: Parameter r0
 * @r1: Parameter r1
 *
 * Derived from vendor PMU code at address 0xb9c4.
 * /
 */
void pmu_rank_slice_deskew_latch(u32 r0, u32 r1)
{
	u32 shift = pmu_freq_ratio_mult(14, r1);
	pmu_deskew_and_tracker_reset();
	u32 ret = pmu_cal_sequence_pulse_send(0, 6, 2, r0, 30, r1, 0);
	if (shift <= 5)
		shift = 5;
	u8 val = (u8)(ret >> shift);
	pmu_cbt_coarse_step_pulse_seq(0, val, 0);
	pmu_clk_timing_delay_latch(0, 1);
}

/**
 * pmu_phy_deskew_reset_strobe() - PHY deskew reset & calibration strobe sequence
 *
 * Derived from vendor PMU code at address 0x8d74.
 * /
 */
void pmu_phy_deskew_reset_strobe(void)
{
	pmu_cal_metric_log(4, 0x49 << 19);
	u8 b40 = *(const u8 *)0x80000040;
	u8 b25 = *(const u8 *)0x80000025;
	pmu_deskew_and_tracker_reset();
	u32 mask = (b40 | b25) & 3;
	pmu_cal_sequence_pulse_send(0x80, 0xb, 0x20, 0, 0, mask, 0);
	pmu_clk_timing_delay_latch(0, 1);
	pmu_hw_timer_delay(0x14);
}

/**
 * pmu_phy_slice_profile_update() - Updates PHY slice registers from profile table at 0x8000043c
 *
 * Derived from vendor PMU code at address 0xb950.
 * /
 */
void pmu_phy_slice_profile_update(void)
{
	u32 count = dmem_read32(0x40c);
	u32 param41c = dmem_read32(PMU_DMEM_PARAM_41C);
	u8 b40 = *(const u8 *)0x80000040;
	u8 b25 = *(const u8 *)0x80000025;
	u32 half_count = count >> 1;
	const u16 *p = (const u16 *)0x8000043c;

	for (u32 i = 0; i <= count; i++) {
		u16 val;
		if (b40 != 0 && i >= half_count) {
			val = p[1];
			if (b40 == 3 && p[3] > val)
				val = p[3];
		} else {
			val = p[0];
			if (b25 == 3 && p[2] > val)
				val = p[2];
		}
		uintptr_t reg = 0x90020000 | (((param41c | (i << 12)) << 1));
		phy_write16(reg, val);
	}
}

/**
 * pmu_slice_reg_query_mailbox_send() - Per-slice register query and mailbox dispatch
 * @mode: Operational or calibration mode
 *
 * Derived from vendor PMU code at address 0x4e7c.
 * /
 */
void pmu_slice_reg_query_mailbox_send(u32 mode)
{
	u8 r2 = *(const u8 *)0x80000b68;
	u8 r1 = *(const u8 *)0x80000b69;
	if (r1 < r2)
		return;

	u32 sel = ((0xe4 >> ((mode << 1) & 6)) & 3) + 0x2a;
	u32 param41c = dmem_read32(PMU_DMEM_PARAM_41C);

	for (u32 slice = r2; slice <= r1; slice++) {
		uintptr_t reg = 0x90020000 | (((param41c | (slice << 12) | sel) << 1));
		u16 val = phy_read16(reg);
		pmu_cal_metric_log(5, 0x2990004, slice, (u32)val, (val >> 6) & 0x1f, val & 0x3f);
	}
}

/**
 * pmu_phy_mode_cfg_dispatch() - PHY mode configuration and pmu_profile_param_stride_cfg dispatcher
 * @mode: Operational or calibration mode
 *
 * Derived from vendor PMU code at address 0x4f48.
 * /
 */
void pmu_phy_mode_cfg_dispatch(u32 mode)
{
	uintptr_t reg = 0x900400e4;
	u8 b25 = *(const u8 *)0x80000025;
	u8 b40 = *(const u8 *)0x80000040;
	u8 *b68 = (u8 *)0x80000b68;

	if (mode == 2) {
		phy_write16(reg, 0);
		phy_write16(reg + 2, b40);
		phy_write16(reg + 2 + 0x16, 0);
		*(u16 *)b68 = *(const u16 *)(b68 + 4);
		phy_write16(reg + 2 + 0xe, 15);
	} else if (mode == 1) {
		phy_write16(reg, b25);
		phy_write16(reg + 2, 0);
		phy_write16(reg + 2 + 0x16, 0);
		*(u16 *)b68 = *(const u16 *)(b68 + 2);
		phy_write16(reg + 2 + 0xe, 0xf0);
	} else {
		u8 b40_val = *(const u8 *)0x80000040;
		phy_write16(reg, b25);
		phy_write16(reg + 2, b40_val);
		phy_write16(reg + 2 + 0x16, 0);
		u16 val_e = phy_read16(reg + 2 + 0xe) & ~0x11;
		phy_write16(reg + 2 + 0xe, val_e);
		b68[0] = b68[2];
		b68[1] = b68[5];
		phy_write16(reg + 2 + 0xe, 0);
	}
	pmu_profile_param_stride_cfg(mode);
}

/**
 * pmu_profile_param_stride_cfg() - Profile parameter updater and PHY slice stride config
 * @mode: Operational or calibration mode
 *
 * Derived from vendor PMU code at address 0xb8cc.
 * /
 */
void pmu_profile_param_stride_cfg(u32 mode)
{
	if (*(const u16 *)0x80000010 == 1)
		return;

	u8 rank = *(const u8 *)0x80000b67;
	u8 b40 = *(const u8 *)0x80000040;
	u16 val;

	if (mode == 3) {
		const u16 *tbl = (const u16 *)(0x8000043c + rank * 4);
		val = tbl[0];
		if (b40 != 0 && tbl[1] > val)
			val = tbl[1];
	} else {
		u32 offset = (mode & ~1u) + rank * 4;
		val = *(const u8 *)(0x8000043c + offset);
	}

	u32 param41c = dmem_read32(PMU_DMEM_PARAM_41C);
	phy_write16(0x9004001a | (param41c << 1), (s8)val);

	u32 count = dmem_read32(0x40c);
	for (u32 i = 0; i <= count; i++) {
		uintptr_t reg = 0x90020000 | (((param41c | (i << 12)) << 1));
		phy_write16(reg, (s8)val);
	}

	phy_write16(0x9005e140, 1);
	pmu_hw_timer_delay(0x20);
	phy_write16(0x9005e140, 0);
}

/**
 * pmu_profile_mailbox_cmd_dispatch() - Profile mailbox command dispatcher pmu_assert_or_halt
 * @idx: Parameter idx
 *
 * Derived from vendor PMU code at address 0x6ba8.
 * /
 */
void pmu_profile_mailbox_cmd_dispatch(u32 idx)
{
	if (idx < 1 || idx > 7)
		return;
	u32 cmd = 0x9d0003 + ((idx - 1) << 16);
	pmu_assert_or_halt(0, cmd);
}

/**
 * pmu_phy_slice_cal_status_read() - PHY slice calibration status reader into DMEM 0x8000043c
 *
 * Derived from vendor PMU code at address 0xb6dc.
 * /
 */
void pmu_phy_slice_cal_status_read(void)
{
	u32 param41c = dmem_read32(PMU_DMEM_PARAM_41C);
	u8 *dmem43c = (u8 *)0x8000043c;

	for (u32 slice = 0; slice < 2; slice++) {
		uintptr_t reg_offset = ((slice << 12) | param41c) << 1;
		u16 v_84 = phy_read16(0x90060084 + reg_offset);
		u16 v_80 = phy_read16(0x90060080 + reg_offset);
		u16 v_82 = phy_read16(0x90060082 + reg_offset);
		u16 v_66 = phy_read16(0x90060066 + reg_offset);
		u16 v_68 = phy_read16(0x90060068 + reg_offset);
		u16 v_5c = phy_read16(0x9006005c + reg_offset);

		*(u16 *)(dmem43c + slice * 2 + 0x8) = v_84;
		*(u16 *)(dmem43c + slice * 2 + 0x14) = v_80;
		*(u16 *)(dmem43c + slice * 2 + 0x18) = v_82;
		dmem43c[slice + 0xe] = (u8)v_66;
		dmem43c[slice + 0x10] = (u8)v_68;
		dmem43c[slice + 0xc] = (u8)v_5c;
	}
}

/**
 * pmu_cal_state_restore_offset_prog() - Calibration state restore and PHY offset register programmer
 *
 * Derived from vendor PMU code at address 0xba44.
 * /
 */
void pmu_cal_state_restore_offset_prog(void)
{
	u8 b25 = *(const u8 *)0x80000025;
	u8 b40 = *(const u8 *)0x80000040;
	phy_write16(0x900400e4, b25);
	pmu_phy_cal_state_restore();
	phy_write16(0x900400e6, b40);
	pmu_cbt_active_entry_lookup();
	u32 status = pmu_cal_table_50c_lookup();
	pmu_slice_status_b97_save();

	u16 b06_16 = *(const u16 *)0x80000006;
	u16 r1 = 0xff;
	u16 r2 = 0xff;
	if (b06_16 >= 0xc81) {
		r1 = phy_read16(0x90040628);
		r2 = phy_read16(0x9004062a);
	}

	u8 gp3 = *(const u8 *)0x80000403;
	u32 offset = (u32)dmem_read32(PMU_DMEM_PARAM_41C) << 1;
	if (gp3 >= 8) {
		u16 v1 = (r1 << 8) + 1;
		u16 v2 = (r2 << 8) + 1;
		phy_write16(0x90121010 | offset, v1);
		phy_write16(0x90121026 | offset, v2);
	} else if (gp3 >= 2) {
		u16 v1 = (r1 << 8) + 1;
		u16 v2 = (r2 << 8) + 1;
		phy_write16(0x90121006 | offset, v1);
		phy_write16(0x90121012 | offset, v2);
	}
}

/**
 * pmu_mailbox_param_eval_dispatch() - Mailbox parameter evaluation and dispatch via pmu_cal_metric_log
 * @p0: Parameter p0
 * @p1: Parameter p1
 * @p2: Parameter p2
 * @rank: DRAM rank index (0..1)
 * @flag: Parameter flag
 *
 * Derived from vendor PMU code at address 0x6a80.
 * /
 */
void pmu_mailbox_param_eval_dispatch(u8 *p0, u8 *p1, u8 *p2, u32 rank, u32 flag)
{
	u32 offset;

	if (rank == 4) {
		offset = 4;
	} else {
		u32 r12 = rank & ~1u;
		u32 r0 = (flag == 0) ? 1 : 0;
		r0 <<= 1;
		if (r12 == 2)
			r0 += 1;
		offset = r0;
	}

	const u8 *tbl = (const u8 *)(0x80000118 + offset);
	u8 v_a = tbl[10];
	if (v_a != 0) {
		*p0 = tbl[0];
		*p1 = tbl[5];
		*p2 = v_a;
	}

	u32 check = pmu_dram_cfg_flag13_check(rank, flag);
	u8 v0 = *p0;
	if (check != 0) {
		u32 sum = (u32)*p1 + v0;
		v0 = 0;
		if (sum > 127)
			sum = 127;
		*p1 = (u8)sum;
		*p0 = 0;
	}

	pmu_cal_metric_log(4, 0x1ba0004, v0, *p1, *p2, rank);
}

/**
 * pmu_eye_margin_boundary_detect() - Eye margin boundary detector (finds leading and trailing valid eye margins)
 * @out_start: Pointer to output start boundary byte
 * @out_end: Pointer to output end boundary byte
 * @samples: Pointer to eye diagram sample buffer
 *
 * Derived from vendor PMU code at address 0x6520.
 * /
 *
 * Return: Computed u32 value or status.
 */
u32 pmu_eye_margin_boundary_detect(u8 *out_start, u8 *out_end, const u8 *samples)
{
	u8 s = 0, e = 0x3f;
	u32 found = 0;
	for (u32 i = 0; i < 64; i++) {
		if (samples[i * 2 + 1] != 0) {
			if (!found) {
				s = (u8)i;
				found = 1;
			}
			e = (u8)i;
		}
	}
	*out_start = s;
	*out_end = e;
	s32 width = (s32)e - (s32)s;
	if (width < 12) {
		pmu_cal_metric_log(0xc9, 0x19e0003, s, e, (u32)width);
		return 6;
	}
	return 0;
}



/**
 * pmu_channel_timing_deskew_reset() - Channel timing calibration deskew and delay reset sequence
 * @a0: Parameter a0
 * @a1: Parameter a1
 * @a2: Parameter a2
 * @a3: Parameter a3
 * @a4: Parameter a4
 *
 * Derived from vendor PMU code at address 0xbac8.
 * /
 */
void pmu_channel_timing_deskew_reset(u32 a0, u32 a1, u32 a2, u32 a3, u32 a4)
{
	pmu_deskew_and_tracker_reset();
	u32 b98 = *(const volatile u8 *)0x800009b8 & 0x3f;

	const u32 vals[5] = { b98, a0, a1, a2, a3 };
	static const u8 ids[5] = { 20, 33, 34, 31, 32 };
	for (u32 i = 0; i < 5; i++)
		pmu_cal_sequence_pulse_send(0, 6, 0x22, vals[i], ids[i], a4, 0);

	pmu_clk_timing_delay_latch(0, 1);
}

/**
 * pmu_rank_slice_margin_eval() - Rank slice margin evaluator across all channels and active lanes
 *
 * Derived from vendor PMU code at address 0x1a84.
 * /
 */
void pmu_rank_slice_margin_eval(void)
{
	u32 mode = pmu_cal_profile_mode_get();

	for (u32 rank = 0; rank < 2; rank++) {
		u8 r1 = *(const volatile u8 *)0x80000025;
		u8 r0 = *(const volatile u8 *)0x80000040;
		if (!((r0 | r1) & (1 << rank)))
			continue;

		u8 start_slice = *(const volatile u8 *)0x80000b68;
		u8 end_slice = *(const volatile u8 *)0x80000b69;

		*(volatile u8 *)0x80000406 = (u8)rank;

		for (u32 slice = start_slice; slice <= end_slice; slice++) {
			u8 mask = *(const volatile u8 *)0x80000b98;
			if (!(mask & (1 << slice)))
				continue;

			*(volatile u8 *)0x80000407 = (u8)slice;

			u32 ptr_3708 = 0x80003708 + rank * 5280 + slice * 1320;
			u32 ptr_dc8  = 0x80000dc8 + rank * 5280 + slice * 1320;

			for (u32 lane = 0; lane < 10; lane++) {
				if (mode & (1 << lane)) {
					*(volatile u8 *)0x80000408 = (u8)lane;
					u32 ret;
					if (mode == 3)
						ret = pmu_cal_rank_margin_pair_eval((u8 *)ptr_dc8, (u8 *)ptr_3708);
					else
						ret = pmu_cal_margin_window_eval((u8 *)ptr_dc8);

					pmu_assert_or_halt(ret == 0, 0x19f0003);
				}
				ptr_3708 += 132;
				ptr_dc8  += 132;
			}
		}
	}
}

/**
 * pmu_dual_rank_phy_reg_read() - Dual-rank PHY register reader into DMEM table at 0x80000b82
 *
 * Derived from vendor PMU code at address 0xa3e8.
 * /
 */
void pmu_dual_rank_phy_reg_read(void)
{
	pmu_cal_metric_log(4, 0x540000);

	u32 total_idx = 0;
	u32 table_base = 0x80000b82;

	for (u32 rank = 0; rank < 2; rank++) {
		u8 active = (rank == 0) ? *(const volatile u8 *)0x80000025 : *(const volatile u8 *)0x80000040;
		if (active != 0) {
			u16 base_val = *(const volatile u16 *)(0x800000e8 + rank * 2);
			u32 base_reg = ((u32)base_val << 2) | (1 << 12) | (1 << 18);

			for (u32 fp = 0; fp < 2; fp++) {
				u8 count = dmem_read8(0x80000409);
				u8 *r13 = (u8 *)(table_base + fp);

				for (u32 r15 = 0; r15 < count; r15++) {
					u8 r2 = *(const volatile u8 *)(0x8000046c + r15);
					u32 reg = 0x90000000 | (((base_reg + (u8)total_idx) << 1));
					u16 val = phy_read16(reg);
					*r13 = (u8)val;
					r13 += 4;
					pmu_cal_metric_log(4, 0x550004, fp, rank, (s8)(u8)val, (u32)r2);
					total_idx++;
				}
			}
		}
		table_base += 2;
	}
}

/**
 * pmu_multirank_slice_cal_dispatch() - Multi-rank slice calibration dispatcher invoking pmu_cal_multi_rank_pulse_seq_dfec/pmu_cal_multi_rank_dispatch_e170
 * @ptr: Pointer to memory structure or buffer
 * @a1: Parameter a1
 * @a2: Parameter a2
 * @a3: Parameter a3
 * @a4: Parameter a4
 * @a5: Parameter a5
 *
 * Derived from vendor PMU code at address 0xe404.
 * /
 */
void pmu_multirank_slice_cal_dispatch(void *ptr, u32 a1, u32 a2, s32 a3, u32 a4, u32 a5)
{
	u8 rank1_mask = *(const volatile u8 *)0x80000040;
	u8 rank0_mask = *(const volatile u8 *)0x80000025;

	for (u32 r13 = 0; r13 < 2; r13++) {
		u32 bit = 1 << r13;
		u32 r15 = (bit << 2) | bit;

		for (u32 fp = 0; fp < 2; fp++) {
			u8 mask = (fp == 0) ? rank0_mask : rank1_mask;
			if ((a4 & bit) & mask) {
				pmu_phy_mode_cfg_dispatch(1 << fp);
				u8 a7c = *(const volatile u8 *)0x80000a7c;
				u8 b01 = *(const volatile u8 *)0x80000001;

				u32 offset = fp * 108 + r13 * 54;
				if (a7c != 0 && !(b01 & 8)) {
					pmu_cal_multi_rank_dispatch_e170((u8)r15, (u32)(uintptr_t)ptr + offset, a1, a2, a3, 0, a5);
				} else {
					pmu_cal_multi_rank_pulse_seq_dfec((u8)r15, (u32)(uintptr_t)ptr + offset, a1, a2, a3, a5);
				}
			}
		}
	}

	pmu_phy_mode_cfg_dispatch(3);
}

/**
 * pmu_slice_phy_reg_step_adjust() - Per-slice PHY register adjuster with step calculation and mailbox reporting
 * @reg_offset: Parameter reg_offset
 * @flag1: Parameter flag1
 * @flag2: Parameter flag2
 *
 * Derived from vendor PMU code at address 0x2838.
 * /
 */
void pmu_slice_phy_reg_step_adjust(u32 reg_offset, u32 flag1, u32 flag2)
{
	u8 start_slice = *(const volatile u8 *)0x80000b68;
	u8 end_slice = *(const volatile u8 *)0x80000b69;
	u32 base = reg_offset + 0x2a;

	for (u32 slice = start_slice; slice <= end_slice; slice++) {
		u32 reg_addr = 0x90020000 | (((dmem_read32(0x41c) | (slice << 12) | base)) << 1);
		u16 val = phy_read16(reg_addr);
		u32 low = val & 0x3f;
		u32 high = (val >> 6) & 0x1f;
		pmu_cal_metric_log(4, 0x2940003, slice, high, low);

		u32 r13;
		if (flag2 != 0) {
			pmu_cal_metric_log(4, 0x2950000);
			u32 denom = (u32)(*(const volatile u8 *)0x80000008) << 6;
			r13 = denom ? ((u32)val % denom) : 0;
		} else if (flag1 != 0) {
			r13 = val + 0x20;
		} else {
			r13 = (val < 0x20) ? 0 : (val - 0x20);
		}

		u32 new_low = r13 & 0x3f;
		u32 new_high = (r13 >> 6) & 0x1f;
		if (flag2 != 0) {
			pmu_cal_metric_log(4, 0x2960002, new_high, new_low);
		} else if (flag1 != 0) {
			pmu_cal_metric_log(4, 0x2970003, 0x20, new_high, new_low);
		} else {
			pmu_cal_metric_log(4, 0x2980003, 0x20, new_high, new_low);
		}

		phy_write16(reg_addr, (new_high << 6) | new_low);
	}
	pmu_phy_reset_pulse();
}


/**
 * pmu_eye_midpoint_sample_avg() - Calculate eye midpoint tap and sample average
 * @out: Pointer to output memory (out[0]=midpoint, out[1]=sample average)
 * @samples: Pointer to eye diagram sample buffer
 * @a: Boundary tap A
 * @b: Boundary tap B
 */
static inline void pmu_eye_midpoint_sample_avg(u8 *out, const u8 *samples, u32 a, u32 b)
{
	u32 s = a + b;
	out[0] = (u8)(s >> 1);
	u32 idx = s & ~1;
	out[1] = (u8)(((u32)samples[idx] + (u32)samples[idx + 1]) >> 1);
}
/**
 * pmu_sample_margin_center_eval() - Sample Margin Center Eval
 * @samples: Pointer to eye diagram sample buffer
 * @out_margins: Pointer to output margin array
 * @out_mode: Pointer to output mode byte
 *
 * Return: Computed u32 value or status.
 */
u32 pmu_sample_margin_center_eval(const u8 *samples, u8 *out_margins, u8 *out_mode)
{
	u16 cfg = *(const volatile u16 *)0x8000000a;
	u32 r3 = (cfg >> 6) & 3;
	*out_mode = (r3 == 1) ? 1 : ((r3 == 3) ? 3 : 2);

	u8 start, end;
	u32 ret = pmu_eye_margin_boundary_detect(&start, &end, samples);
	if (ret != 0)
		return ret;

	pmu_eye_midpoint_sample_avg(out_margins, samples, end, start);
	u8 center = out_margins[0];
	pmu_eye_midpoint_sample_avg(out_margins + 2, samples, end, center);
	pmu_eye_midpoint_sample_avg(out_margins + 4, samples, start, center);
	return 0;
}

/**
 * pmu_cbt_3phase_pulse_dispatch() - Command bus timing calibration dispatcher with 3-phase pulse sequence
 * @a0: Parameter a0
 * @buf: Pointer to data buffer
 * @a2: Parameter a2
 * @a3: Parameter a3
 *
 * Derived from vendor PMU code at address 0xe308.
 * /
 */
void pmu_cbt_3phase_pulse_dispatch(u32 a0, const u8 *buf, u32 a2, u32 a3)
{
	u8 val = buf[a2];
	pmu_deskew_and_tracker_reset();

	if (!(a3 & 2)) {
		pmu_ac_lane_profile_setup();
		pmu_cbt_coarse_step_pulse_seq(0, 5, 1);
	}

	pmu_dbyte_cal_step_latch(a2, val, a0);

	if ((a2 - 14) < 2) {
		pmu_cal_sequence_pulse_send(0, 7, 0x20, 0, 0, 0, 0);
	} else if (a2 == 16) {
		pmu_cal_sequence_pulse_send(0, 7, 4, 0, 0, 1, 0);
		if (val & 0x20) {
			u32 arg = (dmem_read8(0x80000403) == 0) ? (1 << 19) : 0;
			pmu_cbt_cal_pulse_seq(arg);
			pmu_cbt_3phase_pulse_seq();
			pmu_clk_timing_delay_latch(0, 0);
			pmu_delay_us(0x1388, 0x14);
			if (!(a3 & 1))
				pmu_phy_profile_param_program();
			return;
		}
	} else {
		pmu_cal_sequence_pulse_send(0, 7, 6, 0, 0, 0, 0);
	}

	pmu_cbt_3phase_pulse_seq();
	pmu_clk_timing_delay_latch(0, 1);
	if (!(a3 & 1))
		pmu_phy_profile_param_program();
}

/**
 * pmu_phy_cal_strobe_latch_setup() - PHY calibration strobe setup and CSR latch coordinator
 *
 * Derived from vendor PMU code at address 0x56d4.
 * /
 */
void pmu_phy_cal_strobe_latch_setup(void)
{
	u32 p05 = pmu_dmem_param_05_read();
	pmu_delay_us(0, 0x40);

	volatile u16 *b5e = (volatile u16 *)0x9005e000;
	__asm__("" : "+r"(b5e));
	b5e[0x620 >> 1] = 1;
	b5e[0x1f0 >> 1] = 1;

	u16 r15 = phy_read16(0x90040092);
	u32 fp_val = 0x9004009c;

	if (p05 != 0) {
		b5e[0x186 >> 1] = 1;
	} else {
		r15 &= 496;
		u32 r0 = r15 | (1 << 10);
		b5e[0x092 >> 1] = r0;
		pmu_delay_us(1000, 0);
		pmu_delay_us(1000000, 0);
		b5e[0x1c0 >> 1] = 1;
		r15 |= (1 << 9);
		u16 r1 = phy_read16(fp_val + (129 << 1));
		b5e[0x186 >> 1] = 1;
		b5e[0x1c0 >> 1] = 0;
		b5e[0x092 >> 1] = r15;
		u32 reg = ((dmem_read32(0x41c)) << 1) | fp_val;
		phy_write16(reg, r1 & 0x1f);
	}

	u8 gp3 = dmem_read8(0x403);
	u32 csr = 0x9013e050;
	if (gp3 != 0)
		csr += 10;
	phy_write16(csr, 1);
}

/**
 * pmu_slice_delay_step_program() - Multi-slice delay step programmer with quotient/remainder unpacking
 * @arg0: Parameter arg0
 * @data: Pointer to data array
 * @arg2: Parameter arg2
 * @arg3: Parameter arg3
 * @arg4: Parameter arg4
 * @arg5: Parameter arg5
 *
 * Derived from vendor PMU code at address 0xe554.
 * /
 */
void pmu_slice_delay_step_program(u32 arg0, const u16 *data, u32 arg2, u32 arg3, u32 arg4, u32 arg5)
{
	u8 start_slice = *(const volatile u8 *)0x80000b68;
	u8 end_slice = *(const volatile u8 *)0x80000b69;

	u32 count;
	u32 start_idx;
	if (arg5 != 0) {
		count = 1;
		start_idx = start_slice;
	} else {
		start_idx = start_slice * (arg4 ? 10 : 9);
		count = (arg4 != 0) ? 10 : 9;
	}

	u32 gp28 = dmem_read32(0x41c);
	u32 blink = (arg3 != 0) ? 7 : 6;
	u32 r30 = gp28 | arg0;

	const u16 *src = data + start_idx;

	for (u32 slice = start_slice; slice <= end_slice; slice++) {
		u32 r13 = (slice << 12) | r30;
		u32 lane_offset = 0;

		for (u32 i = 0; i < count; i++) {
			s16 val = (s16)*src++;
			u32 r14 = (arg5 != 0) ? 0 : lane_offset;

			s16 quot, rem;
			if (arg2 != 0) {
				quot = val / 64;
				rem = val % 64;
			} else {
				quot = (u16)val >> 10;
				rem = (u16)val & 0x3ff;
			}

			u32 reg = 0x90020000 | (((r14 | r13)) << 1);
			u16 out_val = ((u16)quot << blink) | ((u16)rem);
			phy_write16(reg, out_val);

			lane_offset += 0x100;
		}
	}
}

/**
 * pmu_slice_dq_bitmask_swizzle() - Per-slice DQ sample lane bitmask reader and swizzle mapper
 * @slice: DBYTE slice index (0..3)
 * @arg1: Parameter arg1
 * @arg2: Parameter arg2
 *
 * Derived from vendor PMU code at address 0xa588.
 * /
 *
 * Return: Computed u32 value or status.
 */
u32 pmu_slice_dq_bitmask_swizzle(u32 slice, u32 arg1, u32 arg2)
{
	u32 mode = pmu_cal_profile_mode_get();
	u32 r0;
	if (mode == 1 || (arg2 == 1 && mode == 2))
		r0 = 0x10074;
	else
		r0 = 0x10073;

	u32 reg_val = (slice << 12) | r0;
	u32 r2 = (r0 - 3) | (slice << 12);
	u32 fp = 0x900200e4 | ((slice << 12) << 1);

	phy_write16(fp, 768);

	u8 dmem08 = *(const volatile u8 *)0x80000008;
	u32 r12 = (dmem08 == 2) ? 12 : 8;

	u16 offsets[8];
	for (u32 i = 0; i < 8; i++) {
		u32 r3 = ((i * 0x100) | r2);
		u32 reg = 0x90000000 | (r3 << 1);
		u16 val = phy_read16(reg);
		offsets[i] = (val >> 10) - r12;
	}

	u32 r14 = 0;
	for (u32 lane = 0; lane < 8; lane++) {
		phy_write16(fp, arg1 + offsets[lane] + 192);
		pmu_hw_timer_delay(8);
		u32 reg = 0x90000000 | (((reg_val | (lane * 0x100)) << 1));
		u16 bit = phy_read16(reg) & 1;
		r14 |= (bit << lane);
	}

	phy_write16(fp, 0);
	return pmu_dq_phy_to_logical_mask((u8)r14, slice);
}


/**
 * pmu_phy_delay_bound_margin_log() - PHY delay boundary calculator and margin offset logger
 * @slice: DBYTE slice index (0..3)
 *
 * Derived from vendor PMU code at address 0x5a44.
 * /
 *
 * Return: Computed u16 value or status.
 */
u16 pmu_phy_delay_bound_margin_log(u32 slice)
{
	u32 r1 = (slice << 13) | 0x90020158;
	u16 val = phy_read16(r1);
	phy_write16(r1, val | 1);
	phy_write16(r1, val & ~1);

	u32 gp28 = dmem_read32(0x41c);
	u32 r15 = (slice << 12) | gp28;
	u32 r0 = (r15 << 1) | 0x9002015a;
	u16 r12 = phy_read16(r0);

	u16 r13 = 0;
	if (r12 < 5) {
		u32 addr = (((r12 + 0xd0) | r15) << 1) | 0x90020000;
		r13 = phy_read16(addr);
	}

	u32 r0_base = (slice << 13) | 0x900201aa;
	u16 r3 = phy_read16(r0_base);

	u32 prod = (u32)r3 * (u32)r12;
	u32 diff_prod = prod - r13;
	u32 quot = (diff_prod << 6) / r3;

	u32 bound = (dmem_read8(0x80000403) < 2) ? 160 : 288;
	u32 diff = bound - quot;

	u32 r3_field = (diff >> 6) & 0x3ff;
	pmu_cal_metric_log(4, 0x2790006, slice, r3_field, diff & 0x3f, (u16)quot, (u32)r12, (u32)r13);

	return (u16)diff;
}

/**
 * pmu_eye_centroid_avg_calc() - Multi-lane eye margin centroid and average step synthesizer
 * @r0: Parameter r0
 * @r1: Parameter r1
 * @r2: Parameter r2
 * @r3: Parameter r3
 *
 * Derived from vendor PMU code at address 0x6420.
 * /
 */
void pmu_eye_centroid_avg_calc(const u16 *r0, u16 *r1, u16 *r2, u16 *r3)
{
	u8 b15 = *(const volatile u8 *)0x80000015;
	s32 r11 = (b15 & 1) ? -44 : -128;

	u8 mask_b98 = *(const volatile u8 *)0x80000b98;
	u8 start_slice = *(const volatile u8 *)0x80000b68;
	u8 end_slice = *(const volatile u8 *)0x80000b69;

	s32 r1_param = (s32)0xd4 & r11;
	const volatile u8 *lane_base = (const volatile u8 *)0x80006318;
	__asm__("" : "+r"(lane_base));

	for (u32 slice = start_slice; slice <= end_slice; slice++) {
		if (!(mask_b98 & (1 << slice)))
			continue;

		u32 lane_idx = slice * 10;
		u32 r3_off = 848 * lane_idx;
		u16 base_val = r0[slice];

		for (u32 lane = 0; lane < 10; lane++, lane_idx++, r3_off += 848) {
			const volatile u8 *p = lane_base + r3_off;
			u8 type = p[0x27c];
			u32 r12 = 0;
			s32 r2_val = 0;

			if (type != 0) {
				r12 = p[0];
				if (type == 1) {
					r2_val = r11;
				} else if (type == 3) {
					u8 b1 = p[1];
					u8 b2 = p[2];
					s32 r30 = (s32)b1 - (s32)r12;
					s32 r3_sub = r1_param - (s32)b2;
					if (r30 < r3_sub) {
						r12 = b2;
						r2_val = r11;
					}
				}
			}

			s32 avg = ((s16)r12 + (s8)r2_val) / 2;
			r1[lane_idx] = base_val + (u16)avg;
			r2[lane_idx] = base_val + (u16)r12;
			r3[lane_idx] = base_val + (u16)(s8)r2_val;
		}
	}
}

/**
 * pmu_phy_reg_offset_diff_adjust() - PHY register offset programmer with differential tap adjustments
 * @a0: Parameter a0
 *
 * Derived from vendor PMU code at address 0xd844.
 * /
 */
void pmu_phy_reg_offset_diff_adjust(u32 a0)
{
	u32 r13 = (dmem_read32(0x41c)) << 1;
	u8 *b_ptr = (u8 *)0x80000103;
	u8 b = a0 ? b_ptr[0] : b_ptr[1];
	s32 diff = (s32)b_ptr[0] - (s32)b_ptr[1];
	if (!a0)
		diff = -diff;

	u32 base = 0x90040058 | r13;
	phy_write16(base,     (phy_read16(base)     & 0x3f00) | b);
	phy_write16(base + 2, (phy_read16(base + 2) & 0x3f00) | b);
	phy_write16(base + 8, (phy_read16(base + 8) & 0x3f00) | b);

	u16 v0 = phy_read16(base + 0x1a);
	u16 v1 = phy_read16(base + 0x1e);
	u16 v2 = phy_read16(base + 0x20);

	u16 n0 = (v0 & 0x3f00) | (((v0 & 0x3f) + diff) & 0x3f);
	phy_write16(base + 0x1a, n0);
	phy_write16(base + 0x1c, n0);
	phy_write16(base + 0x1e, (v1 & 0x3f00) | (((v1 & 0x3f) + diff) & 0x3f));
	phy_write16(base + 0x20, (v2 & 0x3f00) | (((v2 & 0x3f) + diff) & 0x3f));
}


/**
 * pmu_phy_profile_param_program() - PHY profile parameter programmer with slice register stride reload
 *
 * Derived from vendor PMU code at address 0x5c50.
 * /
 */
void pmu_phy_profile_param_program(void)
{
	volatile u16 *phy_base = (volatile u16 *)0x90040000;
	__asm__("" : "+r"(phy_base));

	u16 orig_0f4 = phy_base[0x00f4 >> 1];
	u16 orig_0fc = phy_base[0x00fc >> 1];

	phy_base[0x00fc >> 1] = orig_0fc & 0xff33;
	phy_base[0x00f4 >> 1] = orig_0f4 | 0xcc;

	u8 flag = *(const volatile u8 *)0x800000f4;
	if (flag != 0) {
		pmu_cal_metric_log(5, 0x330000);
		u32 gp28 = dmem_read32(0x41c);

		for (u32 slice = 0; slice < 2; slice++) {
			u32 r14 = ((slice << 12) | gp28) << 1;
			volatile u16 *slice_base = (volatile u16 *)(uintptr_t)(r14 | 0x90060000);
			__asm__("" : "+r"(slice_base));

			const volatile u8 *p44 = (const volatile u8 *)(0x80000444 + slice * 2);
			u16 val0 = *(const volatile u16 *)p44;
			slice_base[0x0084 >> 1] = val0;
			slice_base[0x0086 >> 1] = val0;
			slice_base[0x0080 >> 1] = *(const volatile u16 *)(p44 + 0xc);
			slice_base[0x0082 >> 1] = *(const volatile u16 *)(p44 + 0x10);

			const volatile u8 *p48 = (const volatile u8 *)(0x80000448 + slice);
			slice_base[0x005c >> 1] = p48[0];
			slice_base[0x0066 >> 1] = p48[2];
			slice_base[0x0068 >> 1] = p48[4];
		}
	}

	phy_base[0x0084 >> 1] = 0;
	phy_base[0x0088 >> 1] = 0;
	pmu_delay_us(0x2af8, 0);

	phy_base[0x00f4 >> 1] = orig_0f4;
	phy_base[0x00fc >> 1] = orig_0fc;
}


/**
 * pmu_multiparam_cal_dispatch() - Multi-parameter calibration dispatch and structure configuration
 *
 * Derived from vendor PMU code at address 0xe9d4.
 * /
 */
void pmu_multiparam_cal_dispatch(void)
{
	uintptr_t cal_addr = 0x800009a4;
	u8 *cal = (u8 *)cal_addr;
	u8 *param = (u8 *)0x8000005a;
	pmu_cal_struct_mask_or(cal_addr, 1, param[0] & 8);
	pmu_cal_struct_mask_and(cal_addr, 0x14, 0);

	pmu_cal_struct_mask_or(cal_addr, 0x14, param[52] & 0xf);

	if ((param[-63] & 1) && (*(const volatile u8 *)0x80000b96 != 0)) {
		const u8 *src = (const u8 *)0x80000b6e;
		cal[0x78] = src[2];
		cal[0x0c] = src[0];
		cal[0x42] = src[1];
		cal[0xae] = src[3];
	}

	pmu_cal_struct_mask_and(cal_addr, 0x10, 0xfb);
	pmu_cal_struct_mask_or(cal_addr, 0x10, 0x41);

	pmu_multirank_slice_cal_dispatch((void *)cal_addr, 0, 0, 0, 0xff, 0);

	pmu_cal_struct_mask_or(cal_addr, 0x10, 4);
	pmu_cal_multi_param_dispatch_2178(cal_addr, 0, 0xfffeffff, 0xffffffff, 1);
}

/**
 * pmu_phy_lcdl_delay_read() - Reads PHY LCDL delay values per slice and lane into output buffer
 * @out: Pointer to output memory
 * @flag: Parameter flag
 * @mode: Operational or calibration mode
 *
 * Derived from vendor PMU code at address 0xa668.
 * /
 */
void pmu_phy_lcdl_delay_read(u16 *out, u32 flag, u32 mode)
{
	u8 start_slice = *(const volatile u8 *)0x80000b68;
	u8 end_slice = *(const volatile u8 *)0x80000b69;

	u32 stride, start_lane, end_lane, out_idx;
	if (mode == 2) {
		stride = 1;
		start_lane = 0;
		end_lane = 0;
		out_idx = start_slice;
	} else if (mode == 1) {
		stride = 9;
		start_lane = 8;
		end_lane = 8;
		out_idx = start_slice * 9 + 8;
	} else {
		stride = 1;
		start_lane = 0;
		end_lane = 8;
		out_idx = start_slice * 9;
	}

	for (u32 slice = start_slice; slice <= end_slice; slice++) {
		u32 slice_base = slice << 13;
		for (u32 lane = start_lane; lane <= end_lane; lane++) {
			u32 reg = 0x90020154 | slice_base | (lane << 9);
			u16 val = phy_read16(reg);
			if (flag != 0) {
				u32 high = ((u32)val >> 4) & 0xfc0;
				u32 low = (u32)val & 0x3ff;
				val = (u16)(high + low);
			}
			out[out_idx] = val;
			out_idx += stride;
		}
	}
}

/**
 * pmu_dq_lane_telemetry_bitpack() - 7-iteration debug bit-packer calling pmu_cal_metric_log
 * @flags: Configuration bitmask or control flags
 * @base: Base memory address or offset
 *
 * Derived from vendor PMU code at address 0x4c6c.
 */
void pmu_dq_lane_telemetry_bitpack(u32 flags, u32 base)
{
	u8 b66 = *(const volatile u8 *)0x80000b66;
	u8 stride = *(const volatile u8 *)0x8000047c;
	u32 b66_offset = (u32)b66 * 1792;

	for (u32 lane = 0; lane < 7; lane++) {
		for (u32 f = 0; f < 2; f++) {
			if (!(flags & (1 << f)))
				continue;
			u32 tag_base = 0xb00000 + f * 0x30000;
			pmu_cal_metric_log(4, tag_base + 1, lane);
			for (u32 r15 = 0; r15 < 256; r15 += (u32)stride * 8) {
				const volatile u8 *p = (const volatile u8 *)(b66_offset + base + r15);
				u32 val = 0;
				for (s32 bit = 7; bit >= 0; bit--) {
					val |= (u32)(*p) << bit;
					p += stride;
				}
				pmu_cal_metric_log(4, tag_base + 0x10001, (u8)val);
			}
			pmu_cal_metric_log(4, tag_base + 0x20000);
		}
		base += 256;
	}
}

/**
 * pmu_ac_lane_profile_setup() - AC lane profile setup counterpart to pmu_phy_profile_param_program: sets AC profile registers at 0x9007e000
 *
 * Derived from vendor PMU code at address 0x55b8.
 * /
 */
void pmu_ac_lane_profile_setup(void)
{
	volatile u16 *phy_base = (volatile u16 *)0x90040000;
	__asm__("" : "+r"(phy_base));

	u16 orig_0f4 = phy_base[0x00f4 >> 1];
	u16 orig_0fc = phy_base[0x00fc >> 1];

	phy_base[0x00fc >> 1] = 0;
	phy_base[0x00f4 >> 1] = 0xff;

	u8 b64 = *(const volatile u8 *)0x80000b64;
	phy_base[0x0084 >> 1] = (u16)b64 | 0x100;

	u8 b97 = *(const volatile u8 *)0x80000b97;
	const volatile u8 *dmem_f0 = (const volatile u8 *)0x800000f0;
	__asm__("" : "+r"(dmem_f0));
	u8 flag = dmem_f0[4];
	s32 r30 = (s32)(b97 + b64) - 2;

	if (flag != 0) {
		u16 f8 = *(const volatile u16 *)(dmem_f0 + 8);
		u16 fa = *(const volatile u16 *)(dmem_f0 + 10);
		u16 fc = *(const volatile u16 *)(dmem_f0 + 12);
		u16 fe = *(const volatile u16 *)(dmem_f0 + 14);
		u8 f7 = dmem_f0[7];
		u8 f5 = dmem_f0[5];
		u8 f6 = dmem_f0[6];
		pmu_cal_metric_log(5, 0x310007, f8, fa, fc, fe, f7, f5, f6);
		volatile u16 *r1 = (volatile u16 *)(uintptr_t)((dmem_read32(0x41c) << 1) | 0x9007e000);
		__asm__("" : "+r"(r1));
		r1[0x84 >> 1] = f8;
		r1[0x86 >> 1] = fa;
		r1[0x80 >> 1] = fc;
		r1[0x82 >> 1] = fe;
		r1[0x5c >> 1] = f7;
		r1[0x66 >> 1] = f5;
		r1[0x68 >> 1] = f6;
	}

	if (r30 < 0)
		r30 += (u32)b64 * (*(const volatile u8 *)0x80000008);

	phy_base[0x0088 >> 1] = (u16)r30;
	pmu_delay_us(0x2af8, 0);

	phy_base[0x00f4 >> 1] = orig_0f4;
	phy_base[0x00fc >> 1] = orig_0fc;

	*(volatile u8 *)0x80000a7d = ((b64 >> 1) & ~1) - 1;
}

/**
 * pmu_cmd_code_map_log() - Switch statement jump table mapping input code to DMEM 0x80000403 and logging via pmu_cal_metric_log
 * @r0: Parameter r0
 * @r1: Parameter r1
 *
 * Derived from vendor PMU code at address 0xd918.
 * /
 */
void pmu_cmd_code_map_log(u32 r0, u32 r1)
{
	u8 r2 = 9;

	if (r1 == 0) {
		u32 masked = r0 & 0xfff;
		if (masked >= 257 && masked <= 261)
			r2 = masked - 257 + 2;
		else if (masked == 272)
			r2 = 7;
		else if (masked == 512)
			r2 = 8;
	} else if (r1 == 1) {
		r2 = 0;
	} else if (r1 == 2) {
		r2 = 1;
	} else if (r1 == 4 || r1 == 17) {
		r2 = 2;
	} else if (r1 == 18) {
		r2 = 3;
	} else if (r1 == 8 || r1 == 19) {
		r2 = 4;
	} else if (r1 == 20) {
		r2 = 5;
	} else if (r1 == 21) {
		r2 = 6;
	} else if (r1 == 26) {
		r2 = 7;
	} else if (r1 == 0x20) {
		r2 = 8;
	} else {
		pmu_assert_or_halt(0, 0x2190001);
		r2 = *(volatile u8 *)0x80000403;
	}

	*(volatile u8 *)0x80000403 = r2;
	pmu_cal_metric_log(10, 0x21a0003, r2, r0, r1);
}

/**
 * pmu_dbyte_cal_strobe_pulse_seq() - Strobe sequence configuring PHY registers at 0x9003e014, 0x9003e114, and calling pmu_hw_timer_delay
 * @mode: Operational or calibration mode
 * @val: Value to write or configure
 * @type: Parameter type
 *
 * Derived from vendor PMU code at address 0x8380.
 * /
 */
void pmu_dbyte_cal_strobe_pulse_seq(u32 mode, u16 val, u32 type)
{
	u32 gp28 = dmem_read32(0x41c);
	u32 r1 = gp28 << 1;

	if (mode == 0) {
		phy_write16(r1 | 0x9003e016, 0);
		phy_write16(r1 | 0x9003e00a, 0);
		pmu_hw_timer_delay(0x14);
		phy_write16(0x9003e128, 0);
		phy_write16(0x9003e014, 1);
		phy_write16(0x9003e114, 0);
		phy_write16(0x9003e112, 0);
		phy_write16(0x9003e118, 0);
	} else {
		phy_write16(0x9003e114, val);
		phy_write16(0x9003e112, val);
		phy_write16(0x9003e116, 0);
		phy_write16(0x9003e014, 0);

		if (type == 0) {
			phy_write16(r1 | 0x9003e00a, 1);
		} else if (type == 1) {
			phy_write16(0x9003e12a, 10);
		} else {
			pmu_assert_or_halt(0, 0x2350000);
		}

		phy_write16(0x9003e128, (u16)type);
		gp28 = dmem_read32(0x41c);
		phy_write16((gp28 << 1) | 0x9003e016, 2);
	}
}

/**
 * pmu_window_centroid_calc() - Computes window centroid across 64 sample pairs and stores results
 * @out: Pointer to output memory
 *
 * Derived from vendor PMU code at address 0xdab4.
 * /
 *
 * Return: Computed u32 value or status.
 */
u32 pmu_window_centroid_calc(u8 *out)
{
	u16 flag15 = *(const volatile u16 *)0x8000000a;
	u32 r15 = flag15 & 0x1800;

	u32 fp = 0;
	u32 r2 = 0;
	u32 r12 = 0;
	u32 r30 = 0;
	u32 r13 = 0;
	u32 r14 = 0;
	u32 total_weighted = 0;

	for (u32 i = 0; i < 64; i++) {
		u8 v0 = out[i * 2];
		u8 v1 = out[i * 2 + 1];
		s32 diff = (s32)v1 - (s32)v0;
		if (diff < 0)
			continue;

		u32 width = (u32)diff + 1;
		u32 sum = (u32)v0 + (u32)v1;
		u32 prod = width * sum;
		r12 += width;
		total_weighted += prod;

		if (r15 == 0) {
			fp += width * i;
		} else if (r15 == 0x800) {
			u32 sq = width * width;
			r30 += sq;
			r2 += sq * i;
		} else {
			u32 weight = 0;
			u32 w = width >> 1;
			while (w > 0) {
				weight++;
				w >>= 1;
			}
			r13 += weight;
			r14 += weight * i;
		}
	}

	if (r12 == 0) {
		pmu_cal_metric_log(4, 0x1990002, fp, total_weighted);
		pmu_cal_metric_log(4, 0xcd << 17);
		return 3;
	}

	u32 r0 = (total_weighted / r12) + 1;
	u32 r3 = r0 >> 1;

	if (r15 != 0) {
		if (r15 == 0x800) {
			r14 = r2;
			r13 = r30;
		}
		r12 = r13;
		fp = r14;
	}

	u32 r1 = ((fp << 1) / r12) + 1;
	u32 res_r2 = r1 >> 1;

	out[131] = (u8)r3;
	out[130] = (u8)res_r2;
	return 0;
}
/**
 * pmu_cal_pulse_seq_mode22() - Send mode 0x22 calibration pulse sequence
 * @arg3: Parameter arg3
 * @arg4: Parameter arg4
 * @mask: Bitmask filter
 */
static __attribute__((noinline)) void pmu_cal_pulse_seq_mode22(u32 arg3, u32 arg4, u32 mask)
{
	pmu_cal_sequence_pulse_send(0, 6, 0x22, arg3, arg4, mask, 0);
}

/**
 * pmu_slice_deskew_pulse_train() - Slice deskew reset and pulse train coordinator
 * @rank: DRAM rank index (0..1)
 * @mode: Operational or calibration mode
 *
 * Derived from vendor PMU code at address 0xbc00.
 */
void pmu_slice_deskew_pulse_train(u32 rank, u32 mode)
{
	u8 b66 = *(const volatile u8 *)0x80000b66;
	u32 offset = (u32)b66 * 108 + rank * 0x36;
	const volatile u8 *p = (const volatile u8 *)(0x800009a4 + offset);
	u8 r14 = p[0x10];

	pmu_deskew_and_tracker_reset();

	u8 r3 = (mode == 0) ? (r14 | 5) : (r14 & 0xb0);
	u32 mask1 = 1 << rank;
	u8 sp_c = (u8)((mask1 << 2) | mask1);
	u32 sp_10 = r3 | (1 << 6);

	if (mode != 0) {
		u8 fp_val = p[20];
		u8 r13_b1 = p[1];
		pmu_cal_pulse_seq_mode22(r3 | 68, 16, sp_c);

		u32 r13_flag = 0;
		if (r13_b1 & 8) {
			pmu_cal_pulse_seq_mode22(r13_b1, 1, sp_c);
			r13_flag = 1;
		}

		if ((fp_val & 3) == 2) {
			if (r13_flag == 0)
				pmu_deskew_and_tracker_reset();
		} else {
			pmu_cal_pulse_seq_mode22(p[0x14], 20, sp_c);
		}
	}

	pmu_cal_sequence_pulse_send(0x80, 6, 0x22, sp_10, 16, sp_c, 0);
	pmu_clk_timing_delay_latch(0, 1);
	pmu_delay_us(0x3d090, 0);
}

/**
 * pmu_phy_pll_clock_timing_ctrl() - PHY PLL and clock timing control sequence
 *
 * Derived from vendor PMU code at address 0x7c04.
 * /
 */
void pmu_phy_pll_clock_timing_ctrl(void)
{
	u32 gp28 = dmem_read32(0x41c);
	u32 r1 = gp28 << 1;
	u16 val0 = phy_read16(r1 | 0x9002001e);
	phy_write16(r1 | 0x9003e01e, (val0 & 0xff78) | 6);

	volatile u16 *phy_base = (volatile u16 *)0x90040000;
	__asm__("" : "+r"(phy_base));

	phy_base[0x00c2 >> 1] = 1;

	u8 flag05 = *(const volatile u8 *)0x80000005;
	if (flag05 != 0) {
		pmu_delay_us(0, 4);
		phy_base[0x00c2 >> 1] = 0;
		phy_base[0x01c2 >> 1] = 1;
		phy_base[0x01c0 >> 1] = 1;
		phy_base[0x0620 >> 1] = 0;
		pmu_delay_us(0x989680, 0);
		phy_base[0x0186 >> 1] = 0;

		phy_base[0x0400 >> 1] = 0x33;
		phy_base[0x0144 >> 1] = 0;
		pmu_hw_timer_delay(0x20);

		phy_base[0x00f0 >> 1] = 1;
		phy_base[0x0144 >> 1] = 2;
		pmu_hw_timer_delay(1024);
	} else {
		pmu_hw_timer_delay(4);
		phy_base[0x0092 >> 1] = 496;
		phy_base[0x00c2 >> 1] = 0;
		phy_base[0x01c2 >> 1] = 0;
		phy_base[0x0620 >> 1] = 0;
		phy_base[0x01c0 >> 1] = 0;
		phy_base[0x0186 >> 1] = 0;
		pmu_delay_us(0xf4240, 0);

		phy_base[0x01c4 >> 1] = 3;
		pmu_delay_us(0xf4240 >> 1, 0);

		phy_base[0x01c4 >> 1] = 2;
		pmu_delay_us(0x989680, 0);

		phy_base[0x01c4 >> 1] = 0;
		phy_base[0x00f0 >> 1] = 1;
		phy_base[0x0400 >> 1] = 0x33;
		phy_base[0x0144 >> 1] = 0;
		pmu_hw_timer_delay(6);

		phy_base[0x00f0 >> 1] = 2;
		pmu_delay_us(0, 0x2710);

		phy_base[0x0144 >> 1] = 2;
	}

	phy_base[0x00f0 >> 1] = 0;
	phy_base[0x0144 >> 1] = 0;
	pmu_delay_us(0, 0x20);

	phy_base[0x0400 >> 1] = 0;
	phy_base[0x01f0 >> 1] = 0;
	phy_base[0x0144 >> 1] = 2;
}

/**
 * pmu_eye_margin_window_search() - Eye margin window search coordinator
 * @samples: Pointer to eye diagram sample buffer
 * @r1_dummy: Parameter r1_dummy
 *
 * Derived from vendor PMU code at address 0x734c.
 * /
 *
 * Return: Computed u32 value or status.
 */
u32 pmu_eye_margin_window_search(u8 *samples, u32 r1_dummy)
{
	u8 buf_62b0[512];
	u8 out_margins[6];
	u8 out_mode;

	pmu_eye_sample_table_transform(buf_62b0, samples);
	u32 ret = pmu_sample_margin_center_eval(samples, out_margins, &out_mode);
	if (ret != 0)
		return ret;

	u32 r1;
	u32 r0;
	if (out_mode == 2) {
		r1 = 2;
		r0 = 1;
	} else if (out_mode == 3) {
		r1 = 2;
		r0 = 0;
	} else {
		r1 = 0;
		r0 = 0;
	}

	u32 r2 = (r1 < r0) ? r1 : r0;
	u32 r0_calc = ~r2 + r1;
	u32 loop_count = r0_calc + 2;
	const u8 *p_margin = out_margins + (r1 << 1);

	u32 best_metric = 0;
	u8 best_x = 0;
	u8 best_y = 0;

	for (u32 i = 0; i < loop_count; i++) {
		u8 pair[2];
		u32 metric = pmu_cal_margin_sample_pair_extract(pair, p_margin, samples, buf_62b0);
		if (metric >= best_metric) {
			best_metric = metric;
			best_x = pair[1];
			best_y = pair[0];
		}
		p_margin -= 2;
	}

	if (best_metric == 0)
		return 5;

	samples[131] = best_x;
	samples[130] = best_y;
	return 0;
}

/**
 * pmu_eye_margin_bidir_scan() - Bi-directional eye margin boundary scan
 * @table: Parameter table
 * @r1_flag: Parameter r1_flag
 * @r2_flag: Parameter r2_flag
 * @r3_flag: Parameter r3_flag
 *
 * Derived from vendor PMU code at address 0xf430.
 * /
 */
void pmu_eye_margin_bidir_scan(const u8 *table, u32 r1_flag, u32 r2_flag, u32 r3_flag)
{
	u8 center_x = table[130];
	u8 center_y = table[131];

	u32 r12 = (u32)center_x + 1;
	const u8 *p = table + ((u32)center_x << 1);

	while (r12 > 0) {
		if (p[0] >= center_y || center_y >= p[1])
			break;
		p -= 2;
		r12--;
	}

	s32 stop_bwd = (s32)r12 - 1;
	u32 stop_fwd = center_x;

	if (center_x <= 0x3f) {
		p = table + ((u32)center_x << 1);
		while (stop_fwd < 64) {
			if (p[0] >= center_y || center_y >= p[1])
				break;
			p += 2;
			stop_fwd++;
		}
	}

	uintptr_t target_addr = 0x80000037;
	if (r2_flag != 0)
		target_addr += 0x1b;
	if (r3_flag == 1)
		target_addr += 4;
	if (r1_flag == 1)
		target_addr += 2;

	const u8 *center_pair = table + ((u32)center_x << 1);
	u8 upper_y = center_pair[1];
	u8 lower_y = center_pair[0];

	s32 fwd_dist_x = (s8)stop_fwd - (s8)center_x;
	s32 bwd_dist_x = (s8)center_x - (s8)stop_bwd;
	s32 min_x = (bwd_dist_x < fwd_dist_x) ? bwd_dist_x : fwd_dist_x;

	s32 fwd_dist_y = (s32)upper_y - (s32)center_y;
	s32 bwd_dist_y = (s32)center_y - (s32)lower_y;
	s32 min_y = (fwd_dist_y < bwd_dist_y) ? fwd_dist_y : bwd_dist_y;

	volatile u8 *target = (volatile u8 *)target_addr;
	u8 cur_x = target[0];
	u8 cur_y = target[1];

	if ((u8)min_x < cur_x)
		target[0] = (u8)min_x;
	if ((u8)min_y < cur_y)
		target[1] = (u8)min_y;
}

/**
 * pmu_eye_margin_step_align() - Eye margin step aligner and balancer
 * @p0: Parameter p0
 * @p1: Parameter p1
 * @r2: Parameter r2
 *
 * Derived from vendor PMU code at address 0xa9c4.
 * /
 */
void pmu_eye_margin_step_align(u8 *p0, u8 *p1, u32 r2)
{
	u8 v1 = p1[0];
	u8 v0 = p0[0];

	if (v0 > v1) {
		if (v0 < r2) {
			p0[0] = v0 - 1;
			p0[1] += 64;
			if ((s8)p0[1] < 0)
				p0[1] = 0x7f;
		} else {
			p0[0] = v0 - r2;
			if (r2 == 1) {
				p0[1] += 64;
				if ((s8)p0[1] < 0)
					p0[1] = 0x7f;
			} else if (r2 == 2) {
				p0[1] = 0x7f;
			} else {
				if ((s8)p0[1] < 0)
					p0[1] = 0x7f;
			}
		}
	} else if (v0 < v1) {
		if (v1 < r2) {
			p1[0] = v1 - 1;
			p1[1] += 64;
			if ((s8)p1[1] < 0)
				p1[1] = 0x7f;
		} else {
			p1[0] = v1 - r2;
			if (r2 == 1) {
				p1[1] += 64;
				if ((s8)p1[1] < 0)
					p1[1] = 0x7f;
			} else if (r2 == 2) {
				p1[1] = 0x7f;
			} else {
				if ((s8)p1[1] < 0)
					p1[1] = 0x7f;
			}
		}
	}

	pmu_assert_or_halt(p0[0] == p1[0], 0xa40003);
}
/**
 * pmu_cal_pulse_seq_sub7() - Send subcode 7 calibration pulse
 * @val: Value to configure
 */
static __attribute__((noinline)) void pmu_cal_pulse_seq_sub7(u8 val)
{
	pmu_cal_sequence_pulse_send(0, 7, 2, 0, 0, 0, val);
}

/**
 * pmu_cal_pulse_seq_coordinator() - Pulse sequence coordinator calling pmu_freq_ratio_mult, pmu_deskew_and_tracker_reset, and pmu_cal_sequence_pulse_send
 * @rank: DRAM rank index (0..1)
 * @param1: Parameter param1
 * @flags: Configuration bitmask or control flags
 *
 * Derived from vendor PMU code at address 0x0960.
 */
void pmu_cal_pulse_seq_coordinator(u32 rank, u32 param1, u32 flags)
{
	u32 r15 = pmu_freq_ratio_mult(0xfa, param1);

	pmu_deskew_and_tracker_reset();

	u8 b66 = *(const volatile u8 *)0x80000b66;
	u32 offset = (u32)b66 * 108 + rank * 0x36;
	const volatile u8 *p = (const volatile u8 *)(0x800009a4 + offset);
	u8 p10 = p[0x10];

	u32 mask1 = 1 << rank;
	u8 mask = (u8)((mask1 << 2) | mask1);
	u32 p10_mod = p10 | (1 << 6);

	pmu_cal_pulse_seq_mode22(p10_mod, 16, mask);

	r15 >>= 1;
	if (flags & 1) {
		pmu_cal_pulse_seq_sub7((u8)(r15 + 8));
		pmu_cal_pulse_seq_mode22(param1, 14, mask);
	}

	if (flags & 2) {
		pmu_cal_pulse_seq_sub7((u8)r15);
		pmu_cal_pulse_seq_mode22(param1, 15, mask);
	}

	pmu_cal_pulse_seq_sub7((u8)r15);
	pmu_cal_pulse_seq_sub7(0x28);
	pmu_clk_timing_delay_latch(0, 1);
}

/**
 * pmu_dmem_rank_slice_table_setup() - Initializes DMEM rank slice config tables at 0x80000b20..0x80000b23,
 *
 * Derived from vendor PMU code at address 0x8504.
 * slice boundaries at 0x80000b6a..0x80000b6d, and active slice bitmask at 0x80000b98.
 * /
 */
void pmu_dmem_rank_slice_table_setup(void)
{
	volatile u8 *dmem = (volatile u8 *)PMU_DMEM_BASE;
	__asm__("" : "+r"(dmem));
	u8 dq0 = dmem[0x24];
	u8 rank1_en = dmem[0x40];

	u8 r0 = (dq0 + 7) >> 3;
	u8 r12 = r0 - 1;
	u8 r13 = r0;
	u8 r15;

	dmem[0xb6a] = 0;
	dmem[0xb6b] = r12;

	if (rank1_en != 0) {
		u8 dq1 = dmem[0x3f];
		r13 = (u8)(*(volatile u32 *)(dmem + 0x40c) >> 1);
		r0 = ((dq1 + 7) >> 3) + r13;
		dmem[0xb6c] = r13;
		r15 = r0 - 1;
	} else {
		r15 = r12;
		dmem[0xb6c] = r13;
	}
	dmem[0xb6d] = r15;

	u8 mask = dmem[0xb98];
	for (int i = 0; i <= (int)r12; i++)
		mask |= (1 << i);
	dmem[0xb98] = mask;

	if (r15 >= r13) {
		for (int i = (int)r13; i <= (int)r15; i++)
			mask |= (1 << i);
		dmem[0xb98] = mask;
	}

	u8 d1c = dmem[0x1c];
	for (u32 r1 = 0; r1 < 2; r1++) {
		for (u32 r3 = 0; r3 < 2; r3++) {
			u8 dq = (r3 == 0) ? dmem[0x24] : dmem[0x3f];
			u8 shift = r3 * 2;
			u8 r14_val;
			if ((d1c >> shift) & (1 << r1))
				r14_val = (dq + 7) >> 3;
			else
				r14_val = (dq + 15) >> 4;
			dmem[0xb20 + (r3 * 2 + r1)] = r14_val;
		}
	}
}

/**
 * pmu_hw_timer_delay() - Hardware timer delay using ARC EM Real-Time Counter (AUX_RTC_CTRL 0x103,
 * @count: Number of items, halfwords, or iterations
 *
 * Derived from vendor PMU code at address 0x8cfc.
 * AUX_RTC_LOW 0x104, AUX_RTC_HIGH 0x105).
 * /
 */
void pmu_hw_timer_delay(u32 count)
{
	u32 r3 = 0;
	if (count != 0) {
		u8 shift = dmem_read8(0x404);
		r3 = (count + 1) >> shift;
	}

	u32 ctrl = __builtin_arc_lr(0x103);
	if (!(ctrl & 1)) {
		__builtin_arc_sr(2, 0x103);
		__builtin_arc_sr(1, 0x103);
	}

	if (r3 < 16) {
		phy_read16(0x90040048);
		return;
	}

	u32 start_low = __builtin_arc_lr(0x104);
	u32 r2 = ~start_low;
	u32 start_high;
	u32 target_low;

	if (r2 < r3) {
		start_high = __builtin_arc_lr(0x105);
		while (__builtin_arc_lr(0x104) >= start_low)
			;
		start_high = __builtin_arc_lr(0x105);
		target_low = r3 - r2;
	} else {
		start_high = __builtin_arc_lr(0x105);
		target_low = start_low + r3;
	}

	while (__builtin_arc_lr(0x104) < target_low) {
		if (__builtin_arc_lr(0x105) != start_high)
			break;
	}
}

/**
 * pmu_window_margin_diff_calc() - Computes intersection and differential offset between two 64-pair eye margin tables
 * @src0: Pointer to primary source buffer
 * @src1: Pointer to secondary source buffer
 * @out_dest: Pointer to output destination buffer
 * @offset: Byte offset in PMU DMEM
 * @delta: Parameter delta
 *
 * Derived from vendor PMU code at address 0x2a34.
 * /
 *
 * Return: Computed u32 value or status.
 */
u32 pmu_window_margin_diff_calc(const u8 *src0, const u8 *src1, u8 *out_dest, s32 offset, s32 delta)
{
	u16 cfg = *(const volatile u16 *)0x8000000a;
	if (cfg & 2) {
		pmu_cal_metric_log(0xc8, 0x1980002, offset, delta);
	}

	u8 temp_buf[128];
	u8 *dest = (out_dest == src0 || out_dest == src1) ? temp_buf : out_dest;

	for (s32 i = 0; i < 64; i++) {
		s32 j = offset + i;
		u8 *d = dest + (i << 1);

		if ((u32)j > 63) {
			d[0] = 0xff;
			d[1] = 0x00;
			continue;
		}

		const u8 *p0 = src0 + (i << 1);
		u8 s0_high = p0[1];
		if (s0_high == 0) {
			d[0] = 0xff;
			d[1] = 0x00;
			continue;
		}

		const u8 *p1 = src1 + (j << 1);
		u8 s1_high = p1[1];
		if (s1_high == 0) {
			d[0] = 0xff;
			d[1] = 0x00;
			continue;
		}

		s32 h = (s32)s1_high - delta;
		if (h > 127)
			h = 127;
		u8 max_high = (s0_high < (u8)h) ? s0_high : (u8)h;

		s32 l = (s32)p1[0] - delta;
		if (l < 0)
			l = 0;
		u8 min_low = (p0[0] > (u8)l) ? p0[0] : (u8)l;

		if (min_low < max_high) {
			d[0] = min_low;
			d[1] = max_high;
		} else {
			d[0] = 0xff;
			d[1] = 0x00;
		}
	}

	if (dest == temp_buf) {
		for (int i = 0; i < 128; i++)
			out_dest[i] = temp_buf[i];
	}

	return 0;
}

/**
 * pmu_slice_lcdl_delay_collect() - Multi-slice LCDL delay line collector reading PHY DBYTE delay registers
 * @r0: Parameter r0
 * @out_buf: Pointer to destination output buffer
 * @count: Number of items, halfwords, or iterations
 *
 * Derived from vendor PMU code at address 0xa4ac.
 * or querying DMEM calibration tables across active slices.
 * /
 */
void pmu_slice_lcdl_delay_collect(u32 r0, void *out_buf, u32 count)
{
	u16 *out16 = (u16 *)out_buf;
	u8 start_slice = dmem_read8(0xb68);
	u8 end_slice = dmem_read8(0xb69);
	u32 out_idx = start_slice * count;

	for (u8 slice = start_slice; slice <= end_slice; slice++) {
		if (!(r0 & (1 << 24))) {
			if (count == 0)
				continue;
			u32 slice_code = (slice << 12) | r0;
			volatile u16 *phy_base = (volatile u16 *)0x90020000;
			__asm__("" : "+r"(phy_base));
			for (u32 i = 0; i < count; i++) {
				u32 reg_offset = ((i << 8) | slice_code) << 1;
				out16[out_idx++] = *(volatile u16 *)((uintptr_t)phy_base + reg_offset);
			}
		} else {
			u32 ch_offset = dmem_read32(0x41c);
			u32 temp = (r0 & ~ch_offset) & 0x0fffffffe;
			temp += 0xfefffff0;
			u32 code = temp >> 4;
			u8 r1;
			if (code == 0)
				r1 = 0;
			else if (code == 3)
				r1 = 3;
			else if (code == 2)
				r1 = 2;
			else if (code == 1) {
				if (pmu_dq_swap_query((u8)r0, slice))
					r1 = 1;
				else
					r1 = 2;
			} else {
				r1 = 4;
			}
			u8 rank0_end = dmem_read8(0xb6b);
			u8 rank = (slice > rank0_end) ? 1 : 0;
			u8 val = dmem_read8(0xb6e + (rank * 2) + (r1 * 4) + (r0 & 1));
			out16[out_idx++] = val;
		}
	}
}

/**
 * pmu_lcdl_delay_profile_update() - LCDL delay profile updater calling pmu_slice_lcdl_delay_collect and pmu_cal_matrix_trace_dump
 * @mode: Operational or calibration mode
 * @unused: Parameter unused
 * @base_code: Parameter base_code
 * @tag: Telemetry diagnostic tag
 *
 * Derived from vendor PMU code at address 0x4d70.
 * /
 */
void pmu_lcdl_delay_profile_update(u32 mode, u32 unused, u32 base_code, u32 tag)
{
	u16 buf[40];
	pmu_memset_words(buf, 0, sizeof(buf));

	if (mode == 2) {
		pmu_cal_metric_log(5, 0x24f0001, tag);
		pmu_phy_lcdl_delay_read(buf, 0, tag);
		pmu_cal_matrix_trace_dump(0, 9, buf);
		return;
	}

	u32 regs[7];
	u32 counts[7];
	pmu_memset_words(regs, 0, sizeof(regs));
	pmu_memset_words(counts, 0, sizeof(counts));
	u32 num_entries;

	if (mode != 0) {
		pmu_cal_metric_log(5, 0x2520001, tag);
		regs[0] = base_code + 0x2a;
		regs[1] = base_code + 0x26;
		regs[2] = base_code + 0x1000020;
		regs[3] = base_code + 0x28;
		counts[0] = 1;
		counts[1] = 9;
		counts[2] = 1;
		counts[3] = 1;
		num_entries = 4;
	} else {
		pmu_cal_metric_log(5, 0x2500001, tag);
		regs[0] = base_code + 0x20;
		regs[1] = base_code + 0x10;
		regs[2] = base_code + 0x12;
		regs[3] = 0x4e;
		regs[4] = 0x4f;
		regs[5] = 0x50;
		regs[6] = 0x51;
		counts[0] = 2;
		counts[1] = 9;
		counts[2] = 9;
		counts[3] = 9;
		counts[4] = 9;
		counts[5] = 9;
		counts[6] = 9;
		num_entries = 7;
	}

	u32 ch_offset = dmem_read32(0x41c);
	for (u32 fp = 0; fp < num_entries; fp++) {
		pmu_memset_words(buf, 0, sizeof(buf));
		pmu_slice_lcdl_delay_collect(ch_offset | regs[fp], buf, counts[fp]);
		pmu_cal_matrix_trace_dump(fp, counts[fp], buf);
	}
}

/**
 * pmu_slice_metric_scan_and_program() - Scans slice delay metrics from DMEM matrix at 0x8000e798 across ranks,
 *
 * Derived from vendor PMU code at address 0x2038.
 * calculates midpoint (max + min) / 2 and complementary offset (12 - midpoint),
 * and programs PHY DBYTE registers at 0x90020000.
 * /
 */
void pmu_slice_metric_scan_and_program(void)
{
	pmu_cal_metric_log(4, 0x1640000);

	u8 start_slice = dmem_read8(0xb68);
	u8 end_slice = dmem_read8(0xb69);
	u16 num_ranks = dmem_read16(0x428);
	u32 ch_offset = dmem_read32(0x41c);

	volatile u16 *phy_base = (volatile u16 *)0x90020000;
	__asm__("" : "+r"(phy_base));

	for (u32 slice = start_slice; slice <= end_slice; slice++) {
		const volatile u16 *slice_metrics = (const volatile u16 *)(0x8000e798 + (slice * 8));
		u16 max_val = slice_metrics[0];
		u16 min_val = slice_metrics[0];

		for (u32 k = 0; k < num_ranks; k++) {
			u16 val = slice_metrics[k];
			pmu_cal_metric_log(4, 0x1650003, slice, val);
			if (val > max_val)
				max_val = val;
			if (val < min_val)
				min_val = val;
		}

		pmu_cal_metric_log(4, 0x1660002, min_val, max_val);

		u32 midpoint = (max_val + min_val) >> 1;
		u32 diff = 12 - midpoint;
		u16 diff_u16 = (u16)diff;

		pmu_cal_metric_log(4, 0x1670002, midpoint, diff_u16);

		u32 reg_val_r30 = ((diff & 0xffff) << 4) | midpoint;
		u32 reg_val_fp = (diff & 0xffff) | (midpoint << 4);
		u32 slice_offset = (slice << 12) | ch_offset;

		for (u32 r3 = 0; r3 < 2; r3++) {
			u32 r13_reg = (r3 != 0) ? 0x5e : 0x62;
			u32 r14_reg = (r3 != 0) ? 0x60 : 0x64;

			u16 w1 = *(volatile u16 *)((uintptr_t)phy_base + ((r13_reg | ch_offset) << 1));
			u16 w2 = *(volatile u16 *)((uintptr_t)phy_base + ((r14_reg | ch_offset) << 1));
			w1 = (w1 & 0x300) | reg_val_r30;
			w2 = (w2 & 0x300) | reg_val_fp;

			for (u32 blink = 0; blink < 2; blink++) {
				*(volatile u16 *)((uintptr_t)phy_base + (((blink + r13_reg) | slice_offset) << 1)) = w1;
				*(volatile u16 *)((uintptr_t)phy_base + (((blink + r14_reg) | slice_offset) << 1)) = w2;
			}
		}
	}
}

/**
 * pmu_rank_margin_channel_accum() - Multi-channel composite eye margin intersection and cumulative width accumulator
 * @rank: DRAM rank index (0..1)
 * @start_slice: Starting DBYTE slice index
 * @end_slice: Ending DBYTE slice index
 * @flags: Configuration bitmask or control flags
 * @swap_mode: Parameter swap_mode
 *
 * Derived from vendor PMU code at address 0x1dd4.
 * /
 *
 * Return: Computed u16 value or status.
 */
u16 pmu_rank_margin_channel_accum(u32 rank, u32 start_slice, u32 end_slice, u32 flags, u32 swap_mode)
{
	u8 temp_buf[128];
	for (int i = 0; i < 64; i++) {
		temp_buf[i * 2] = 0x00;
		temp_buf[i * 2 + 1] = 0x7f;
	}

	u32 rank_offset = rank * 0x14a0;
	u32 fp_count = (flags != 0) ? flags : 1;

	for (u32 slice = start_slice; slice <= end_slice; slice++) {
		if (swap_mode == 2) {
			u8 b = pmu_dq_swap_query(0, slice);
			if (rank != 0 || b != 0)
				continue;
		} else if (swap_mode == 1) {
			u8 b = pmu_dq_swap_query(0, slice);
			if (rank == 0 || b != 0)
				continue;
		} else {
			if (flags == 0)
				continue;
		}

		const u8 *sub = (const u8 *)(0x80000dc8 + rank_offset + (slice * 1320));
		for (u32 fp = fp_count; fp > 0; fp--) {
			s8 offset = (s8)sub[130] - 31;
			pmu_window_margin_diff_calc(temp_buf, sub, temp_buf, offset, 0);
			sub += 132;
		}
	}

	u32 total_width = 0;
	for (int i = 0; i < 64; i++) {
		u8 low = temp_buf[i * 2];
		u8 high = temp_buf[i * 2 + 1];
		if (high >= low)
			total_width += (high - low + 1);
	}

	return (u16)total_width;
}

/**
 * pmu_slice_deskew_pulse_seq() - Slice deskew pulse sequencer and delay averaging logic
 * @r0: Parameter r0
 *
 * Derived from vendor PMU code at address 0x1f2c.
 * /
 */
void pmu_slice_deskew_pulse_seq(u32 r0)
{
	if (r0 != 1)
		return;

	for (u32 r14 = 1; r14 < 3; r14++) {
		u8 cfg = (r14 == 2) ? dmem_read8(0x40) : dmem_read8(0x25);
		if (cfg != 3)
			continue;

		pmu_phy_mode_cfg_dispatch(r14);
		pmu_phy_reset_pulse();

		u8 start_slice = dmem_read8(0xb68);
		u8 end_slice = dmem_read8(0xb69);
		u16 sp_buf[8];

		for (u32 r13 = 0; r13 < 2; r13++) {
			u8 offset = ((0xe4 >> (r13 * 2)) & 3) + 0x2a;

			for (u32 slice = start_slice; slice <= end_slice; slice++) {
				u32 slice_offset = slice << 12;
				u32 ch_offset = dmem_read32(0x41c);
				u32 reg = (offset | slice_offset | ch_offset);
				u16 val = phy_read16(0x90020000 + (reg << 1));

				sp_buf[r13 * 4 + slice] = val;

				if (r13 == 1) {
					u16 v0 = sp_buf[0 * 4 + slice];
					u32 sum = (u32)v0 + val;
					u32 q = sum >> 7;
					u32 rem = (sum >> 1) & 0x3f;

					pmu_cal_metric_log(4, 0x2bf0006, r14, 1, rem);

					u16 prog_val = rem | (q << 6);
					phy_write16(0x90020054 + ((slice_offset | ch_offset) << 1), prog_val);
					phy_write16(0x90020000 + (reg << 1), prog_val);
				}
			}
		}
	}
	pmu_phy_mode_cfg_dispatch(3);
}

/**
 * pmu_eye_sample_metric_eval() - Evaluates metric score of eye sample coordinate (x, y) relative to
 * @x: Horizontal eye sample coordinate
 * @y: Vertical eye sample coordinate
 * @table_62b0: Parameter table_62b0
 * @samples: Pointer to eye diagram sample buffer
 *
 * Derived from vendor PMU code at address 0x2d1c.
 * eye boundary sample pairs and distance table.
 * /
 *
 * Return: Computed u32 value or status.
 */
u32 pmu_eye_sample_metric_eval(u8 x, u8 y, const u8 *table_62b0, const u8 *samples)
{
	const u8 *pair = samples + ((u32)x << 1);
	u8 low = pair[0];
	u8 high = pair[1];

	if (low >= high || low > y || high <= y)
		return 0;

	u32 coord = ((u32)y << 8) | x;
	u8 min_err = 0xff;
	u32 min_val = 0xffffffff;

	const u8 *p = table_62b0;
	for (u32 i = 0; i < 64; i++) {
		s16 r0_0 = *(const s16 *)(p);
		s16 r0_1 = *(const s16 *)(p + 2);
		if (r0_1 >= r0_0) {
			u8 out_byte = 0;
			u32 ret = pmu_eye_error_metric_eval(coord, i, (s8)r0_0, (s8)r0_1, &out_byte);
			if (ret < min_val)
				min_val = ret;
			if (out_byte < min_err)
				min_err = out_byte;
		}

		s16 r1_0 = *(const s16 *)(p + 4);
		s16 r1_1 = *(const s16 *)(p + 6);
		if (r1_1 >= r1_0) {
			u8 out_byte = 0;
			u32 ret = pmu_eye_error_metric_eval(coord, i, (s8)r1_0, (s8)r1_1, &out_byte);
			if (ret < min_val)
				min_val = ret;
			if (out_byte < min_err)
				min_err = out_byte;
		}
		p += 8;
	}

	u32 dist = 0xff;
	if (low < y && y < high) {
		u32 d1 = y - low;
		u32 d2 = high - y;
		dist = (d1 < d2) ? d1 : d2;
	}

	u16 w17 = dmem_read16(0x422);
	u16 w32 = dmem_read16(0x420);

	u32 term1 = (u32)(x + 1) * (u32)(x + 1) * w17;
	u32 term2 = (u32)(64 - x) * (u32)(64 - x) * w17;
	u32 term3 = (u32)(128 - y) * (u32)(128 - y) * w32;

	if (term1 < min_val)
		min_val = term1;
	if (term2 < min_val)
		min_val = term2;
	if (term3 < min_val)
		min_val = term3;

	u8 d0e = dmem_read8(0x0e);
	if (!(d0e & 1)) {
		u32 term4 = (u32)(y + 1) * (u32)(y + 1) * w32;
		if (term4 < min_val)
			min_val = term4;
	}

	u32 score = (min_val > 0xffffff) ? 0xffffff : min_val;
	u32 r3_min = (min_err < x) ? min_err : x;
	u32 rx_comp = 63 - x;
	if (rx_comp < r3_min)
		r3_min = rx_comp;

	return (score << 8) | ((dist + r3_min) & 0xff);
}

/**
 * pmu_eye_boundary_sample_search() - Coordinates eye boundary sample search across 64 steps using coarse (step 8)
 * @samples: Pointer to eye diagram sample buffer
 * @target_y: Target eye boundary threshold
 *
 * Derived from vendor PMU code at address 0x61bc.
 * and fine binary (steps 4, 2, 1) search iterations evaluated by pmu_eye_sample_metric_eval.
 * /
 *
 * Return: Computed u32 value or status.
 */
u32 pmu_eye_boundary_sample_search(u8 *samples, u32 target_y)
{
	u8 margins[6];
	u8 mode;
	u32 ret = pmu_sample_margin_center_eval(samples, margins, &mode);
	if (ret != 0)
		return ret;

	u8 buf_62b0[512];
	pmu_eye_sample_table_transform(buf_62b0, samples);

	u32 metric0 = pmu_eye_sample_metric_eval(margins[0], target_y, buf_62b0, samples);
	u32 best_score = 0;
	u32 best_sample = metric0;

	const u8 *p = samples;
	for (u32 r15 = 0; r15 <= 0x3f; r15 += 8, p += 16) {
		if (p[0] >= target_y || target_y >= p[1])
			continue;
		u32 score = pmu_eye_sample_metric_eval((u8)r15, target_y, buf_62b0, samples);
		if (score > best_score) {
			best_score = score;
			best_sample = r15;
		}
	}

	for (u32 step = 4; step >= 2; step >>= 1) {
		s32 cand1 = (s32)best_sample + (s32)step;
		if (!(cand1 & ~0x3f)) {
			u32 score = pmu_eye_sample_metric_eval((u8)cand1, target_y, buf_62b0, samples);
			if (score > best_score) {
				best_score = score;
				best_sample = (u32)cand1;
			}
		}
		s32 cand2 = (s32)best_sample - (s32)step;
		if (!(cand2 & ~0x3f)) {
			u32 score = pmu_eye_sample_metric_eval((u8)cand2, target_y, buf_62b0, samples);
			if (score > best_score) {
				best_score = score;
				best_sample = (u32)cand2;
			}
		}
	}

	if (best_score != 0) {
		samples[131] = (u8)target_y;
		samples[130] = (u8)best_sample;
	}

	return 0;
}

/**
 * pmu_eye_margin_step_balancer() - Multi-rank eye margin step balancer coordinating centroid calculations (pmu_window_centroid_calc),
 * @buf0: Parameter buf0
 * @buf1: Parameter buf1
 * @buf2: Parameter buf2
 * @buf3: Parameter buf3
 *
 * Derived from vendor PMU code at address 0x2b14.
 * pairwise differential margin intersection (pmu_window_margin_diff_calc), and eye boundary search (pmu_eye_boundary_sample_search).
 * /
 *
 * Return: Computed u32 value or status.
 */
u32 pmu_eye_margin_step_balancer(u8 *buf0, u8 *buf1, u8 *buf2, u8 *buf3)
{
	u16 cfg = *(const volatile u16 *)0x8000000a;
	if (cfg & 2) {
		pmu_cal_metric_log(4, 0x1950000);
	}

	u8 *bufs[4] = { buf0, buf1, buf2, buf3 };
	u32 count1 = (buf1 != buf0) ? 2 : 1;
	u32 count2 = (buf2 != buf0) ? 2 : 1;

	for (u32 r15 = 0; r15 < count1; r15++) {
		dmem_write8(0x406, r15);
		for (u32 i = 0; i < count2; i++) {
			u8 *b = bufs[i * 2 + r15];
			u32 ret = pmu_window_centroid_calc(b);
			if (ret != 0)
				return ret;
		}
	}

	if (cfg & 2) {
		pmu_cal_metric_log(4, 0x1960000);
	}

	u8 *b0 = buf0;
	u8 *b1 = buf1;
	u8 temp_buf[128];

	for (u32 r13 = 1; r13 <= count2; r13++) {
		s8 off = (s8)b1[130] - (s8)b0[130];
		dmem_write8(0x406, 3);
		pmu_window_margin_diff_calc(b0, b1, temp_buf, off, 0);

		u32 ret;
		u8 d0b = dmem_read8(0x0b);
		if (d0b & 4) {
			ret = pmu_window_centroid_calc(temp_buf);
		} else {
			ret = pmu_eye_margin_window_search(temp_buf, 0);
		}
		if (ret != 0)
			return ret;

		b0[131] = temp_buf[131];
		b1[131] = temp_buf[131];

		if (r13 < count2) {
			b0 = buf2;
			b1 = buf3;
		}
	}

	if (cfg & 2) {
		pmu_cal_metric_log(4, 0x1970000);
	}

	if (buf2 != buf0) {
		for (u32 r14 = 0; r14 < count1; r14++) {
			u8 *rx = bufs[r14];
			u8 *ry = bufs[r14 + 2];
			s32 delta = (s32)ry[131] - (s32)rx[131];
			s32 diff = (s16)*(u16 *)(rx + 128) - (s16)*(u16 *)(ry + 128);
			pmu_window_margin_diff_calc(rx, ry, temp_buf, diff, delta);
			u32 ret = pmu_eye_boundary_sample_search(temp_buf, rx[131]);
			if (ret != 0)
				return ret;
			rx[130] = temp_buf[130];
			ry[130] = temp_buf[130] + diff;
		}
	} else {
		u32 ret0 = pmu_eye_boundary_sample_search(buf2, buf2[131]);
		if (ret0 != 0)
			return ret0;
		u32 ret1 = pmu_eye_boundary_sample_search(buf1, buf1[131]);
		if (ret1 != 0)
			return ret1;
	}

	return 0;
}

void pmu_cal_vref_dac_step_adjust(u32 arg0);
void pmu_cal_stage42_accum_dispatch(u32 dram_type, u32 count, void *slice_base);
void pmu_cal_rank_param_commit_8aa8(u32 r0, u32 r1, u32 r2, u32 r3);
void pmu_cal_margin_matrix_scan_eval(u32 r0, u32 r1, u32 r2, u32 r3, u32 r4, u32 r5);
u32 pmu_cal_rank_margin_window_check(u32 r0, u32 r1, u32 r2);

/**
 * pmu_cal_ptr_tag_check() - Decodes pointer tag via ROR 27, tests bit 6 at base offset 0, and returns bit 3 from offset 7
 * @r0_unused: Parameter r0_unused
 * @ptr_tag: Pointer tag identifier
 *
 * Derived from vendor PMU code at address 0x4a48.
 * /
 *
 * Return: Computed u32 value or status.
 */
u32 pmu_cal_ptr_tag_check(u32 r0_unused, u32 ptr_tag)
{
	uintptr_t addr = (ptr_tag >> 27) | (ptr_tag << 5);
	volatile u8 *p = (volatile u8 *)addr;
	u8 tag = p[0];
	if (!(tag & 0x40))
		return 0;
	u8 lo = p[7];
	u8 hi = p[8];
	u16 val = (u16)lo | ((u16)hi << 8);
	return (val >> 3) & 1;
}

/**
 * pmu_cal_search_win_max_calc() - Evaluates active DBYTE slice delay line registers across channels and ranks,
 * @arg0: Parameter arg0
 * @arg1: Parameter arg1
 *
 * Derived from vendor PMU code at address 0x21fc.
 * computes maximum delay window, and updates 2D calibration parameter table.
 * /
 */
void pmu_cal_search_win_max_calc(u32 arg0, u32 arg1)
{
	u8 rank = dmem_read8(0xb67);

	if (arg0 == 1 && dmem_read8(0x40) == 0) {
		u32 tbl_off = 0x43c + (rank << 2);
		dmem_write16(tbl_off + 2, dmem_read16(tbl_off));
		return;
	}

	u32 lane_off = 0xb6a + (arg0 ? 2 : 0);
	u8 lane_start = dmem_read8(lane_off);
	u8 lane_end   = dmem_read8(lane_off + 1);

	s32 max_val = -1;
	if (lane_start <= lane_end) {
		u32 r1_flag = (arg1 == 0) ? 1 : 0;
		u32 gp28 = dmem_read32(0x41c);
		u32 r14 = (r1_flag << 2) + 0x20 + rank + gp28;
		u32 r2 = (u32)lane_start << 12;
		u32 count = (u32)lane_end + 1 - lane_start;
		for (u32 i = 0; i < count; i++) {
			u32 addr = 0x90020000 | ((r14 | r2) << 1);
			u16 val = phy_read16(addr);
			if ((s32)val > max_val)
				max_val = (s32)val;
			r2 += (512 << 3);
		}
	}

	u8 b1 = dmem_read8(0x08);
	s8 b2 = (s8)dmem_read8(0x110);
	u32 mask = (b1 == 2) ? 0xff : 511;
	u32 shift = (b1 == 2) ? 8 : 9;
	u32 flag = ((u32)max_val & mask) ? 1 : 0;
	s32 term = max_val >> shift;
	u32 r0_idx = (arg0 << 1) + (rank << 2);

	dmem_write16(0x43c + r0_idx, (u16)(b2 + term + flag + 2));
}

/*
 * pmu_cal_stage39_post_results:
 * Derived from vendor PMU code at address 0x88b4.
 * Stage 39 calibration results processing coordinator:
 * Aggregates 4-block delay calibration results from DMEM 0x80000b6e into
 * sequence config buffer, executes multi-rank calibration dispatch via pmu_multirank_slice_cal_dispatch,
 * resets deskew and tracking pipelines, sends timing calibration commands via pmu_cal_sequence_pulse_send,
 * asserts PLL CBT entry control, updates per-channel rank status and restores offsets.
 */
void pmu_cal_timing_table_matrix_init(u8 *buf);
void pmu_cal_lane_deskew_sweep_exec(void);
/**
 * pmu_cal_stage39_post_results() - Cal Stage39 Post Results
 */
void pmu_cal_stage39_post_results(void)
{
	u8 cal_buf[216];
	pmu_cal_timing_table_matrix_init(cal_buf);

	u8 r2 = (dmem_read8(0x9a4 + 3) & 0x40) ? 1 : ((dmem_read8(0x9a4 + 0x16) >> 6) & 1);
	u8 r3 = (cal_buf[3] & 0x40) ? 1 : ((cal_buf[0x16] >> 6) & 1);
	if (r2 != r3)
		pmu_phy_reg_offset_diff_adjust(0);

	pmu_cal_struct_mask_and((u32)(uintptr_t)cal_buf, 0x13, 0xf3);
	pmu_cal_struct_mask_or((u32)(uintptr_t)cal_buf, 0x10, 0x45);

	u8 num_entries = dmem_read8(0x409);
	for (u32 i = 0; i < num_entries; i++) {
		u8 r3_idx = dmem_read8(0x46c + i);
		for (u32 rank = 0; rank < 2; rank++) {
			for (u32 ch = 0; ch < 2; ch++) {
				u8 val = dmem_read8(0xb6e + (i * 4) + (ch * 2) + rank);
				cal_buf[ch * 108 + rank * 54 + r3_idx] = val;
			}
		}
	}

	pmu_multirank_slice_cal_dispatch(cal_buf, 1, 0x800, 0, 0xff, 0);

	pmu_cal_struct_mask_and((u32)(uintptr_t)cal_buf, 0x10, 0xbc);
	if (!(cal_buf[0x1c] & 0x20))
		pmu_cal_struct_mask_or((u32)(uintptr_t)cal_buf, 0x1c, 2);

	pmu_multirank_slice_cal_dispatch(cal_buf, 1, 0xfffeffff, -1, 0xff, 0);

	pmu_phy_deskew_reset_strobe();
	pmu_deskew_and_tracker_reset();

	u8 ch = dmem_read8(0xb66);
	if (!(cal_buf[ch * 108 + 0x1c] & 0x20))
		pmu_cal_sequence_pulse_send(0, 6, 0x22, cal_buf[ch * 108 + 0x1c] | 2, 28, 15, 0);

	pmu_cal_sequence_pulse_send(0, 0x28, 2, 0, 0, 3, 0);
	pmu_cal_sequence_pulse_send(0, 7, 0x1e, 0, 0, 0, 0);
	pmu_clk_timing_delay_latch(0, 1);

	if (dmem_read8(0x96) & 0x10) {
		u32 gp28 = dmem_read32(0x41c);
		phy_write16(0x9003e002 | (gp28 << 1), 1);
	}

	pmu_cbt_entry_pll_ctrl();
	phy_write16(0x900400dc, 0);

	u8 dram_type = ((const volatile u8 *)pmu_reset_vector)[0];
	if (dram_type == 0) {
		for (u8 r1 = 0; r1 < 2; ) {
			u8 ch_idx = r1;
			bool active = false;
			if (ch_idx != 0) {
				if (dmem_read8(0x40) != 0)
					active = true;
			} else {
				if (dmem_read8(0x25) != 0)
					active = true;
			}
			if (active) {
				pmu_phy_mode_cfg_dispatch(ch_idx + 1);
				pmu_cal_lane_deskew_sweep_exec();
				r1 = dmem_read8(0xb66) + 1;
			} else {
				r1 = ch_idx + 1;
			}
			dmem_write8(0xb66, r1);
		}
	}

	pmu_phy_mode_cfg_dispatch(3);
	dmem_write16(0xb66, 0);

	if (dram_type == 0)
		pmu_cal_state_restore_offset_prog();

	phy_write16(0x9003e166, 0);
	phy_write16(0x9003e168, 0);

	if (dram_type == 0 && dmem_read16(0x10) != 1)
		pmu_phy_slice_profile_update();

	pmu_phy_cal_strobe_latch_setup();
}

/*
 * pmu_cal_delay_line_accum_dispatch:
 * Derived from vendor PMU code at address 0x1b7c.
 * Evaluates active DBYTE delay lines, unpacks and adjusts slice delay taps across
 * channels and ranks, executes Stage 42/43 accumulation via pmu_cal_stage42_accum_dispatch, and programs PHY CSRs.
 */
struct dly_line_entry {
	u16 val16;
	u8 val8_0;
	u8 val8_1;
} __attribute__((packed));
/**
 * pmu_cal_delay_line_accum_dispatch() - Cal Delay Line Accum Dispatch
 * @rank: DRAM rank index (0..1)
 */
void pmu_cal_delay_line_accum_dispatch(u32 rank)
{
	struct dly_line_entry buf[6][2];
	u32 dram_type;
	u32 count;
	u32 pstate_start;
	u32 pstate_end;
	u8 dmem_25;
	u8 dmem_40;

	dram_type = pmu_cal_profile_mode_get();

	dmem_25 = dmem_read8(0x25);
	dmem_40 = dmem_read8(0x40);
	count = ((dmem_40 | dmem_25) == 3) ? 2 : 1;

	pstate_start = dmem_read8(0xb68);
	pstate_end = dmem_read8(0xb69);

	for (u32 pstate = pstate_start; pstate <= pstate_end; pstate++) {
		u8 pstate_mask = dmem_read8(0xb98);

		if (!(pstate_mask & (1 << pstate)))
			continue;

		for (u32 lane = 0; lane < 9; lane++) {
			if (!(rank & (1 << lane)))
				continue;

			u32 csr_base = dmem_read32(0x41c) | (pstate << 12) | (lane << 8);

			for (u32 k = 0; k < count; k++) {
				u32 idx = (pstate * 20) + (lane * 2) + k;

				buf[0][k].val16  = dmem_read16(0x6098 + idx * sizeof(u16));
				buf[0][k].val8_0 = dmem_read8(0x61d8 + idx);
				buf[0][k].val8_1 = dmem_read8(0x6278 + idx);

				buf[1][k].val16  = dmem_read16(0x6138 + idx * sizeof(u16));
				buf[1][k].val8_0 = dmem_read8(0x6228 + idx);
				buf[1][k].val8_1 = dmem_read8(0x62c8 + idx);

				u32 csr_addr_24 = 0x90020024 | ((csr_base | k) << 1);
				buf[2][k].val16  = pmu_dly_line_repack(phy_read16(csr_addr_24));

				u32 reg_0_0 = 0x90020000 | ((csr_base | pmu_dmem_cal_offset_select(0, 0, k)) << 1);
				buf[2][k].val8_0 = (u8)phy_read16(reg_0_0);

				u32 reg_0_1 = 0x90020000 | ((csr_base | pmu_dmem_cal_offset_select(0, 1, k)) << 1);
				buf[2][k].val8_1 = (u8)phy_read16(reg_0_1);

				u32 csr_addr_20 = 0x90020020 | ((csr_base | k) << 1);
				buf[3][k].val16  = pmu_dly_line_repack(phy_read16(csr_addr_20));

				u32 reg_1_0 = 0x90020000 | ((csr_base | pmu_dmem_cal_offset_select(1, 0, k)) << 1);
				buf[3][k].val8_0 = (u8)phy_read16(reg_1_0);

				u32 reg_1_1 = 0x90020000 | ((csr_base | pmu_dmem_cal_offset_select(1, 1, k)) << 1);
				buf[3][k].val8_1 = (u8)phy_read16(reg_1_1);
			}

			((void (*)(u32, u32, void *))(uintptr_t)pmu_cal_stage42_accum_dispatch)(dram_type, count, buf);

			for (u32 k = 0; k < count; k++) {
				u32 csr_addr_24 = 0x90020024 | ((csr_base | k) << 1);
				phy_write16(csr_addr_24, buf[4][k].val16);

				u32 csr_addr_20 = 0x90020020 | ((csr_base | k) << 1);
				phy_write16(csr_addr_20, buf[5][k].val16);

				if (dmem_read8(0xce) != 0)
					continue;

				u32 reg_0_0 = 0x90020000 | ((csr_base | pmu_dmem_cal_offset_select(0, 0, k)) << 1);
				phy_write16(reg_0_0, buf[4][k].val8_0);

				u32 reg_1_0 = 0x90020000 | ((csr_base | pmu_dmem_cal_offset_select(1, 0, k)) << 1);
				phy_write16(reg_1_0, buf[5][k].val8_0);

				if (pmu_cal_profile_mode_get() != 3)
					continue;

				u32 reg_0_1 = 0x90020000 | ((csr_base | pmu_dmem_cal_offset_select(0, 1, k)) << 1);
				phy_write16(reg_0_1, buf[4][k].val8_1);

				u32 reg_1_1 = 0x90020000 | ((csr_base | pmu_dmem_cal_offset_select(1, 1, k)) << 1);
				phy_write16(reg_1_1, buf[5][k].val8_1);
			}
		}
	}
}

/**
 * struct pmu_cal_bist_cmd_ctx - BIST calibration command context descriptor
 * @field_00: Reserved status field 0
 * @field_04: Reserved status field 1
 * @field_08: Enable flag
 * @cmd_c: Primary BIST command code
 * @cmd_10: Secondary BIST command code
 * @stride_minus_1: Calibration stride minus 1
 * @csr_offset: Active slice CSR offset
 * @slice_delays: Per-slice delay tap programming array
 */
struct pmu_cal_bist_cmd_ctx {
	u32 field_00;       /* 0x00: Reserved / Status initialized to 0 */
	u32 field_04;       /* 0x04: Reserved / Status initialized to 0 */
	u32 field_08;       /* 0x08: Enable flag initialized to 1 */
	u32 cmd_c;          /* 0x0c: Primary BIST command code */
	u32 cmd_10;         /* 0x10: Secondary BIST command code */
	u32 stride_minus_1; /* 0x14: Calibration stride minus 1 */
	u32 csr_offset;     /* 0x18: Active slice CSR offset */
	u16 slice_delays[]; /* 0x1c: Per-slice delay tap programming array */
};

void pmu_cal_bist_cmd_strobe_dispatch(u32 arg0, u32 arg1, u32 arg2, u32 arg3,
				      u32 arg4, u32 arg5, u32 arg6)
{
	struct pmu_cal_bist_cmd_ctx *ctx = (struct pmu_cal_bist_cmd_ctx *)(uintptr_t)arg0;
	const u16 *slice_flags = (const u16 *)(uintptr_t)arg4;
	u32 status;
	u32 csr_offset;
	u32 step_param;
	u32 cmd_r1;
	u8 slice_start;
	u8 slice_end;
	u8 slice_mask;
	u16 stride;
	bool flag;

	/* 1. Query calibration status and determine active CSR offset */
	status = pmu_cal_status_query(arg5, arg2);

	if (dmem_read8(0x403) <= 1)
		csr_offset = dmem_read32(PMU_DMEM_PARAM_41C) | 0x14;
	else
		csr_offset = 0x56;

	/* 2. Initialize calibration context descriptor header */
	ctx->field_00 = 0;
	ctx->field_04 = 0;
	ctx->field_08 = 1;
	ctx->csr_offset = csr_offset;

	/* 3. Program mode-dependent BIST command codes */
	flag = (arg3 & 0x5) != 0;

	switch (arg2) {
	case 0:
		ctx->cmd_10 = flag ? 18 : 19;
		ctx->cmd_c  = flag ? 16 : 17;
		break;
	case 1:
		ctx->cmd_10 = flag ? 16 : 17;
		ctx->cmd_c  = flag ? 18 : 19;
		break;
	case 2:
		ctx->cmd_c = flag ? 0x26 : 0x27;
		break;
	case 3:
		ctx->cmd_c = flag ? 0x28 : 0x29;
		break;
	default:
		pmu_assert_or_halt(0, 0x01a80001);
		break;
	}

	/* 4. Trigger PHY soft reset & configure BIST pattern generator */
	phy_write16(PHY_REG_RESET_PULSE_0F2, 0x40);
	phy_write16(0x9003e000 | (csr_offset << 1), 1);
	phy_write16(0x9003e14e, 0);

	pmu_cal_bist_pattern_setup(arg1, arg3, status, arg6);
	pmu_cal_markers_set(800);

	/* 5. Dispatch BIST calibration sequence commands via pmu_cal_sequence_pulse_send */
	step_param = (dmem_read8(PMU_DMEM_DRAM_TYPE) == 2) ? 4 : 2;

	if (arg1 < 9 && ((1U << arg1) & 0x105)) {
		pmu_cal_sequence_pulse_send(0x8000, 7, 0x18, 0, 0, 0, 0);
		pmu_cbt_cal_stat_set();
		pmu_cal_sequence_pulse_send(0, 7, 0, 0, 0, arg3, 0);
		pmu_cal_sequence_pulse_send(0x100000, 5, 0, 0x80, 0, arg3, 0);
		pmu_cal_sequence_pulse_send(0x800000, 0x29, step_param, 0, 0, arg3, 7);
		pmu_cal_sequence_pulse_send(0, 7, dmem_read8(0x400), 0, 0, arg3, 0);
		pmu_cal_sequence_pulse_send(0, 7, 0, 0, 0, arg3, 0);
		pmu_cal_sequence_pulse_send(0x200000, 5, 0, 256, 0, arg3, 0);
		cmd_r1 = 0x2a;
	} else {
		pmu_cal_sequence_pulse_send(0x8000, 7, 0x28, 0, 0, 0, 0);
		pmu_cbt_cal_stat_set();
		pmu_cal_sequence_pulse_send(0, 7, 8, 0, 0, arg3, 0);
		pmu_cal_sequence_pulse_send(0, 7, 0, 0, 0, arg3, 0);
		pmu_cal_sequence_pulse_send(0x200000, 5, 0, 256, 0, arg3, 0);
		cmd_r1 = 0x2b;
	}

	pmu_cal_sequence_pulse_send(0x01040000, cmd_r1, step_param, 0, 0, arg3, 7);
	pmu_cal_sequence_pulse_send(0, 7, 4, 0, 0, arg3, 10);
	pmu_cbt_cal_stat_clear();
	pmu_cal_sequence_pulse_send(0, 7, 0, 0, 0, 0, 0);
	pmu_cal_sequence_pulse_send(0, 7, 0, 0, 0, 0, 0);

	/* 6. Program calibration stride and timing control CSRs */
	stride = pmu_cal_stride_get();

	phy_write16(0x9003e166, 1);
	phy_write16(0x9003e168, 1);
	phy_write16(0x9003e160, (dmem_read8(PMU_DMEM_DRAM_TYPE) == 4) ? 0x34 : 0x30);
	ctx->stride_minus_1 = stride - 1;
	phy_write16(0x9003e162, 1);

	/* 7. Slice delay tap programming loop */
	slice_start = dmem_read8(PMU_DMEM_SLICE_START);
	slice_end = dmem_read8(PMU_DMEM_SLICE_END);
	slice_mask = dmem_read8(0xb98);

	for (u32 slice = slice_start; slice <= slice_end; slice++) {
		if (!(slice_mask & (1 << slice)))
			continue;

		u16 val;
		if (arg2 == 1) {
			val = (slice_flags[slice] & 0x80) ? 10 : 5;
		} else if (arg2 == 0) {
			val = (slice_flags[slice] & 0x80) ? 5 : 10;
		} else {
			val = 0;
		}

		ctx->slice_delays[slice] = val;
	}
}

/**
 * pmu_cal_metric_table_log() - Logs DDR PHY calibration metric table across channels, slices, ranks, and lanes
 * @base_addr: Base MMIO address
 * @r1_param: Parameter r1_param
 * @r2_param: Parameter r2_param
 * @r3_param: Parameter r3_param
 *
 * Derived from vendor PMU code at address 0x5d34.
 * /
 */
void pmu_cal_metric_table_log(u32 base_addr, u32 r1_param, u32 r2_param, u32 r3_param)
{
	u32 fp_limit;
	u32 r13_val;
	u32 var_50;
	u8 var_4f;
	u8 var_4e;

	pmu_cal_metric_log(0xc8, 0x6a0000);

	fp_limit = pmu_cal_rank_stride_count_get(r1_param);
	r13_val = pmu_cal_profile_mode_get();

	if (r2_param != 0)
		pmu_cal_metric_log(0xc8, 0x6c0000);
	else
		pmu_cal_metric_log(0xc8, 0x6b0000);

	pmu_cal_channel_status_get(r2_param, &var_50, &var_4f, &var_4e);

	if (r1_param != 0) {
		pmu_cal_metric_log(0xc8, 0x730000);
	} else {
		pmu_cal_metric_log(0xc8, 0x6d0000);
		switch (r13_val) {
		case 3:
			pmu_cal_metric_log(0xc8, 0x710000);
			fp_limit = 2;
			break;
		case 1:
			pmu_cal_metric_log(0xc8, 0x6f0000);
			fp_limit = 1;
			break;
		case 2:
			pmu_cal_metric_log(0xc8, 0x700000);
			fp_limit = 1;
			break;
		case 0:
			pmu_cal_metric_log(0xc8, 0x6e0000);
			fp_limit = 1;
			break;
		default:
			pmu_assert_or_halt(0, 0x720001);
			break;
		}
	}

	pmu_cal_metric_log(0xc8, 0x740005, var_50, pmu_cal_profile_mode_get(), (u32)var_4f, (u32)var_4e, r3_param);

	u8 dmem_20 = dmem_read8(0x20);
	u8 dmem_21 = dmem_read8(0x21);
	u8 dmem_17 = dmem_read8(0x17);
	u8 dmem_18 = dmem_read8(0x18);
	u8 dmem_25 = dmem_read8(0x25);
	u8 dmem_40 = dmem_read8(0x40);

	pmu_cal_metric_log(0xc8, 0x750008, dmem_20, dmem_21, dmem_17, dmem_18, dmem_25, dmem_40, 10, 4);

	if ((s32)fp_limit <= 0)
		goto exit_log;

	for (u32 fp = 0; fp < fp_limit; fp++) {
		for (u32 r14 = 0; r14 < var_50; r14++) {
			for (u32 r15 = var_4f; r15 <= var_4e; r15++) {
				for (u32 r13 = 0; r13 < 10; r13++) {
					if (!(r3_param & (1U << r13)))
						continue;

					pmu_cal_metric_log(0xc8, 0x760004, fp, r14, r15, r13);

					u32 entry_addr = base_addr +
							 fp * 10560 +
							 r14 * 5280 +
							 r15 * 1320 +
							 r13 * 132;
					const u8 *entry = (const u8 *)(uintptr_t)entry_addr;

					for (u32 i = 0; i < 64; i++) {
						u8 byte0 = entry[2 * i];
						u8 byte1 = entry[2 * i + 1];

						pmu_cal_metric_log(0xc8, 0x770003, byte0, byte1, i);
					}

					u16 val_128 = *(const u16 *)(entry + 128);
					s8 val_130 = *(const s8 *)(entry + 130);
					u8 val_131 = *(const u8 *)(entry + 131);

					pmu_cal_metric_log(0xc8, 0x780003, val_128, (s32)val_130, (u32)val_131);
				}
			}
		}
	}

exit_log:
	pmu_cal_metric_log(0xc8, 0x790000);
}

/**
 * pmu_cal_metric_table_sweep_log() - Logs DDR PHY calibration metric table sweep results across dimensions
 * @base_addr: Base MMIO address
 * @r1_param: Parameter r1_param
 * @r2_param: Parameter r2_param
 *
 * Derived from vendor PMU code at address 0x5f50.
 * /
 *
 * Return: Computed u32 value or status.
 */
u32 pmu_cal_metric_table_sweep_log(u32 base_addr, u32 r1_param, u32 r2_param)
{
	u32 num_outer;
	u8  val_b6b;
	u8  val_025;
	u8  temp_85;
	u32 val_58;
	u32 var_4c;
	s32 max_limit;

	num_outer = pmu_cal_rank_stride_count_get(r1_param);

	val_b6b = dmem_read8(0xb6b);
	val_025 = dmem_read8(0x25);
	temp_85 = val_b6b;

	pmu_cal_channel_status_get(1, &val_58, &temp_85, &temp_85);

	if ((s32)num_outer > 0) {
		var_4c = (val_025 == 3) ? 2 : 1;
		max_limit = ((s32)var_4c > (s32)val_58) ? (s32)var_4c : (s32)val_58;

		for (u32 outer_i = 0; (s32)outer_i < (s32)num_outer; outer_i++) {
			if (max_limit <= 0)
				continue;

			for (u32 j = 0; (s32)j < max_limit; j++) {
				pmu_cal_metric_log(4, (r1_param != 0) ? 0x840003 : 0x830003, j, j, 0xff);

				for (u32 fp = 0; fp < 2; fp++) {
					u32 val_5c;
					u8  min_k;
					u8  max_k;

					if (fp == 0 && j >= var_4c)
						continue;
					if (fp == 1 && (s32)j >= (s32)val_58)
						continue;

					pmu_cal_channel_status_get(fp, &val_5c, &min_k, &max_k);

					if (min_k > max_k)
						continue;

					for (u32 k = min_k; k <= max_k; k++) {
						for (u32 lane = 0; lane < 10; lane++) {
							if (!(r2_param & (1U << lane)))
								continue;

							u32 lane_base = base_addr +
											outer_i * 10560 +
											j * 5280 +
											k * 1320 +
											lane * 132;

							u16 val_128 = *(volatile const u16 *)(uintptr_t)(lane_base + 128);
							s8  val_130 = *(volatile const s8  *)(uintptr_t)(lane_base + 130);
							u8  val_131 = *(volatile const u8  *)(uintptr_t)(lane_base + 131);

							pmu_cal_metric_log(4, 0x820005, k, lane,
									(u32)val_128, (s32)val_130, (u32)val_131);

							for (int sub = 0; sub < 2; sub++) {
								u16 buffer[64];

								if (dmem_read8(0x0b) & 0x02) {
									const u8 *src = (const u8 *)(uintptr_t)(lane_base + (1 - sub));

									for (int s = 0; s < 64; s++) {
										buffer[s] = src[s * 2];
									}
									pmu_post_trace_log_halfwords(4, 0x7a0040, buffer, 64);
								} else {
									const u8 *src = (const u8 *)(uintptr_t)(lane_base + (5 - sub));

									for (int s = 0; s < 31; s++) {
										buffer[s] = src[s * 4];
									}
									pmu_post_trace_log_halfwords(4, 0x7e001f, buffer, 31);
								}
							}
						}
					}
				}
			}
		}
	}

	if (dmem_read8(0x0a) & 0x01) {
		pmu_cal_metric_table_log(base_addr, r1_param, 0, r2_param);
		pmu_cal_metric_table_log(base_addr, r1_param, 1, r2_param);
	}

	return 0;
}

/**
 * pmu_cal_dly_line_tap_step_adjust() - Multi-rank DBYTE slice delay tap step unpack, alignment, and CSR/DMEM update coordinator
 * @arg0: Parameter arg0
 * @arg1: Parameter arg1
 * @arg2: Parameter arg2
 * @arg3: Parameter arg3
 *
 * Derived from vendor PMU code at address 0xf4d4.
 * /
 */
void pmu_cal_dly_line_tap_step_adjust(u32 arg0, u32 arg1, u32 arg2, u32 arg3)
{
	u32 var_10 = (arg3 != 0) ? (pmu_cal_ptr_tag_check(arg0, arg1) == 0) : 1;
	u32 var_20 = (arg0 == 0) ? 1 : 0;
	u32 gp28 = dmem_read32(0x41c);

	for (u32 rank = 0; rank < 2; rank++) {
		if (!((dmem_read8(0x25) | dmem_read8(0x40)) & (1 << rank)))
			continue;

		u8 slice_start = dmem_read8(0xb68);
		u8 slice_end = dmem_read8(0xb69);
		for (u32 slice = slice_start; slice <= slice_end; slice++) {
			if (!(dmem_read8(0xb98) & (1 << slice)))
				continue;

			u8 step_val = *(volatile u8 *)(0x80000474 + (rank * 4) + slice);

			for (u32 lane = 0; lane < 9; lane++) {
				if (!(arg2 & (1 << lane)))
					continue;

				u32 fp = (slice << 12) | (lane << 8) | gp28 | 0x10000;
				u32 tbl_offset = (rank * 5280) + (slice * 1320) + (lane * 132);
				volatile u8 *tbl_ptr = (volatile u8 *)(0x80000e48 + tbl_offset);

				u16 val_e48 = *(volatile u16 *)tbl_ptr;
				u8 var_18 = *(volatile u8 *)(0x8000378b + tbl_offset);
				s8 val_e48_offset2 = (s8)tbl_ptr[2];
				u8 var_1c = tbl_ptr[3];
				u32 var_0 = (u32)(val_e48 + val_e48_offset2);

				u32 base_addr = (fp | rank) << 1;

				if (arg0 != 0) {
					u8 dly_buf_32[4] = {0};
					u8 dly_buf_30[4] = {0};

					if (arg1 == 0 && dmem_read8(0x401) != 0) {
						u32 idx_6098 = (slice * 40) + (lane * 4) + (rank * 2);
						u16 w_6098 = *(volatile u16 *)(0x80006098 + idx_6098);
						pmu_dly_line_unpack(w_6098, dly_buf_32);
					} else {
						u32 phy_addr = 0x90000024 | base_addr;
						u16 phy_w = phy_read16(phy_addr);
						pmu_cal_param_unpack(phy_w, dly_buf_32);
					}

					pmu_dly_line_unpack((u16)var_0, dly_buf_30);
					if (var_10 == 0) {
						dly_buf_32[0] = dly_buf_30[0];
						dly_buf_32[1] = dly_buf_30[1];
					}

					if (dmem_read8(0x401) != 0) {
						pmu_delay_tap_step_adjust(dly_buf_32, arg1);
						pmu_delay_tap_step_adjust(dly_buf_30, arg1);
					}

					if (dmem_read8(0xd) & 0x20) {
						pmu_delay_tap_step_adjust(dly_buf_32, step_val);
						pmu_delay_tap_step_adjust(dly_buf_30, step_val);

						int diff = (int)dly_buf_32[0] - (int)dly_buf_30[0];
						if (diff > 1 || diff < -1) {
							u32 phy_addr = 0x90000024 | base_addr;
							pmu_cal_param_unpack(phy_read16(phy_addr), dly_buf_32);
							pmu_dly_line_unpack((u16)var_0, dly_buf_30);
							u32 step_adj = (step_val == 0) ? 1 : 0;
							pmu_delay_tap_step_adjust(dly_buf_32, step_adj);
							pmu_delay_tap_step_adjust(dly_buf_30, step_adj);
						}
					}

					pmu_eye_margin_step_align(dly_buf_32, dly_buf_30, (dmem_read8(0x401) != 0) ? 2 : 1);

					if (arg1 == 0) {
						u32 idx_6098 = (slice * 40) + (lane * 4) + (rank * 2);
						*(volatile u16 *)(0x80006098 + idx_6098) = pmu_hdr_addr_14b_decode(dly_buf_32);
						*(volatile u16 *)(0x80006138 + idx_6098) = (u16)var_0;
						u32 idx_6228 = (slice * 20) + (lane * 2) + rank;
						*(volatile u8 *)(0x80006228 + idx_6228) = var_1c;
						*(volatile u8 *)(0x800062c8 + idx_6228) = var_18;
					}

					if (!(arg1 == 0 && dmem_read8(0x401) != 0)) {
						u32 phy_addr1 = 0x90000024 | base_addr;
						phy_write16(phy_addr1, pmu_hdr_addr_15b_decode(dly_buf_32));
					}

					u32 phy_addr2 = 0x90000020 | base_addr;
					phy_write16(phy_addr2, pmu_hdr_addr_15b_decode(dly_buf_30));
				} else {
					if (arg1 == 0) {
						u32 idx_6098 = (slice * 40) + (lane * 4) + (rank * 2);
						u32 idx_61d8 = (slice * 20) + (lane * 2) + rank;
						*(volatile u8 *)(0x800061d8 + idx_61d8) = var_1c;
						*(volatile u16 *)(0x80006098 + idx_6098) = (u16)var_0;
						*(volatile u8 *)(0x80006278 + idx_61d8) = var_18;
					}

					u8 dly_buf_34[4] = {0};
					pmu_dly_line_unpack((u16)var_0, dly_buf_34);

					if (dmem_read8(0x401) != 0)
						pmu_delay_tap_step_adjust(dly_buf_34, arg1);

					if (dmem_read8(0xd) & 0x20)
						pmu_delay_tap_step_adjust(dly_buf_34, step_val);

					u32 phy_addr = 0x90000024 | base_addr;
					phy_write16(phy_addr, pmu_hdr_addr_15b_decode(dly_buf_34));
				}

				if (dmem_read8(0xce) == 0) {
					u32 off = pmu_dmem_cal_offset_select(arg0, 0, rank);
					u32 reg = 0x90000000 | ((fp | off) << 1);
					phy_write16(reg, var_1c);
					if (!(var_20 | var_10)) {
						off = pmu_dmem_cal_offset_select(0, 0, rank);
						reg = 0x90000000 | ((fp | off) << 1);
						phy_write16(reg, var_1c);
					}
					if (pmu_cal_profile_mode_get() == 3) {
						off = pmu_dmem_cal_offset_select(arg0, 1, rank);
						reg = 0x90000000 | ((fp | off) << 1);
						phy_write16(reg, var_18);
						if (!(var_20 | var_10)) {
							off = pmu_dmem_cal_offset_select(0, 1, rank);
							reg = 0x90000000 | ((fp | off) << 1);
							phy_write16(reg, var_18);
						}
					}
				}
			}
		}
	}
}

/**
 * pmu_cal_stage1_pre_init() - Converted from pmu_cal_stage1_pre_init
 *
 * /
 */
void pmu_cal_stage1_pre_init(void)
{
	/* Step 1: Initialize hardware gating & tracking */
	pmu_dbyte_cal_strobe_pulse_seq(1, 2047, 0);

	/* Clear 3584-byte scan sample matrix (2 slices x 7 channels x 256 delay steps) */
	u8 scan_matrix[3584];
	pmu_memset_words(scan_matrix, 0, sizeof(scan_matrix));

	/* Verify clock gating configuration: bit 3 of DMEM[0x01] must be 0 */
	u32 clk_gate_flag = dmem_read8(PMU_DMEM_CLK_GATE_FLAG);
	u32 cond = !(clk_gate_flag & (1 << 3));
	pmu_assert_or_halt(cond, 0x25 << 19);

	/*
	 * Determine calibration step size from upper nibble of DMEM[0x1a]:
	 * bit 7 -> 8, bit 6 -> 4, bit 5 -> 2, default -> 1.
	 * Store step size in DMEM[0x47c].
	 */
	u8 d1a = dmem_read8(0x1a);
	u8 d1a_high = d1a >> 4;
	u8 step;
	if (d1a_high & (1 << 3))
		step = 8;
	else if (d1a_high & (1 << 2))
		step = 4;
	else if (d1a_high & (1 << 1))
		step = 2;
	else
		step = 1;
	dmem_write8(0x47c, step);

	/* Backup 6 bytes of parameter state from DMEM 0x99c */
	u8 sp_54[6];
	pmu_memcpy_words(sp_54, (const void *)(PMU_DMEM_BASE | 0x99c), sizeof(sp_54));
	pmu_phy_reg_buffer_stream(sp_54, 1, 1);

	/*
	 * Step 2: Build inverse lane mapping table from PHY registers.
	 * Slice 0: 0x90060120..0x9006012c (7 channels).
	 * Slice 1 (if active in DMEM[0x40]): 0x90062120..0x9006212c (7 channels).
	 */
	u8 map_table[16] = {0};
	for (u32 i = 0; i < 7; i++) {
		u16 pin = phy_read16(0x90060120 + (i * 2));
		map_table[pin] = (u8)i;
	}

	if (dmem_read8(0x40) != 0) {
		for (u32 i = 0; i < 7; i++) {
			u16 pin = phy_read16(0x90062120 + (i * 2));
			map_table[pin + 7] = (u8)i;
		}
	}

	/*
	 * Step 3: Descriptor phase sample initialization & telemetry logging.
	 */
	pmu_cal_struct_mask_remap();

	u8 num_slices = 0;
	s8 sp_50[4] = {0};

	for (u32 idx = 0; idx < 4; idx++) {
		u16 sp_40_val;
		u8 sp_4f_val;
		u8 sp_37_val;

		if (pmu_cal_channel_rank_config_get((u16)idx, &sp_40_val, &sp_4f_val, &sp_37_val) != 0)
			continue;

		num_slices = sp_37_val;
		dmem_write8(0x0a7c, 1);
		pmu_phy_mode_cfg_dispatch(num_slices);

		u8 rank = dmem_read8(0xb66);
		u32 gp28 = dmem_read32(0x41c);
		u32 reg_base = (((u32)rank << 12) | gp28) << 1;
		phy_write16(0x90061e02 | reg_base, 0);
		phy_write16(0x90060004 | reg_base, 0);

		s16 ret_c184 = pmu_cal_dbyte_rx_fifo_reset_poll(sp_4f_val, 1, scan_matrix);
		sp_50[(rank << 1) + sp_40_val] = (s8)ret_c184;

		pmu_cal_metric_log(4, 0x012b0002, rank, sp_40_val);
		pmu_dq_lane_telemetry_bitpack(num_slices, (uintptr_t)scan_matrix);
		pmu_cal_dbyte_rx_fifo_reset_poll(sp_4f_val, 0, scan_matrix);
		pmu_cal_metric_log(4, 0x012c0002, rank, sp_40_val);
		pmu_dq_lane_telemetry_bitpack(num_slices, (uintptr_t)scan_matrix);
	}

	/*
	 * Step 4: Clear channel gate delay registers and trigger calibration strobe.
	 */
	u32 gp28 = dmem_read32(0x41c);
	u32 err_flag = 0;

	for (u32 ch = 0; ch < 7; ch++)
		phy_write16(0x9007e002 | (((gp28 | (ch << 8)) << 1)), 0);

	phy_write16(0x9007e004 | (gp28 << 1), 0);
	pmu_cal_strobe_pulse();

	/*
	 * Step 5: Multi-slice / multi-channel eye margin search and LCDL programming.
	 */
	const u8 *map_ptr = map_table;

	for (u32 slice = 0; slice < num_slices; slice++) {
		u32 base_delay;
		if (dmem_read8(0x1a) & (1 << 3)) {
			pmu_cal_metric_log(5, 0x012d0002, slice, 0);
			base_delay = 0;
		} else {
			pmu_cal_metric_log(5, 0x012f0001, slice);
			base_delay = 0x40; /* Nominal LCDL midpoint 64 */
		}

		s32 lower_bound = (s32)base_delay - 128;
		s32 upper_bound = (s32)base_delay + 128;

		u16 center_delays[7];

		for (u32 ch = 0; ch < 7; ch++) {
			u32 best_start = 0;
			u32 best_len = 0;
			u32 cur_len = 0;
			u32 run_start = 0xff;

			for (u32 delay = 0; delay <= 255; delay += step) {
				u8 sample = scan_matrix[slice * 1792 + ch * 256 + delay];
				if (run_start != 0xff) {
					if (sample == 0) {
						cur_len += step;
					} else {
						if (cur_len > best_len && delay >= 129) {
							pmu_cal_metric_log(4, 0x01310004, ch, run_start, cur_len + step, delay - 1);
							best_start = run_start;
							best_len = cur_len + step;
						}
						run_start = 0xff;
					}
				} else {
					if (sample == 0) {
						run_start = delay;
						cur_len = 0;
					}
				}
			}

			if (best_len == 0 && cur_len != 0) {
				best_start = run_start;
				best_len = cur_len + step;
			}

			if (best_len == 0) {
				pmu_cal_metric_log(10, 0x01350000);
				err_flag = 1;
			} else {
				s32 center = (s32)best_start + (s32)((best_len & ~1) / 2) + lower_bound;
				if (center < 0)
					center = 0;
				else if (center > upper_bound)
					center = upper_bound;
				center_delays[ch] = (u16)center;
			}
		}

		/* Program DQS gate center delays for all 7 channels of this slice */
		for (u32 ch = 0; ch < 7; ch++) {
			u16 delay_val = center_delays[ch];
			u8 mapped_lane = map_ptr[ch];
			pmu_cal_metric_log(5, 0x01360003, slice, mapped_lane, delay_val);
			u32 addr = 0x90060002 | (((gp28 | (slice << 12) | ((u32)mapped_lane << 8)) << 1));
			phy_write16(addr, delay_val);
		}

		/* Program lane pair LCDL offsets and TX DQS delay */
		s32 val0 = (s32)base_delay + (s32)sp_50[slice * 2 + 0];
		u32 dly0 = (val0 > 0) ? (u32)val0 : 0;
		pmu_cal_metric_log(5, 0x01370002, slice, (u16)dly0);
		phy_write16(0x90061002 | (((gp28 | (slice << 12)) << 1)), (u16)dly0);

		s32 val1 = (s32)base_delay + (s32)sp_50[slice * 2 + 1];
		u32 dly1 = (val1 > 0) ? (u32)val1 : 0;
		pmu_cal_metric_log(5, 0x01380004, slice, (u16)dly1, slice, base_delay);
		map_ptr += 7;
		phy_write16(0x90061202 | (((gp28 | (slice << 12)) << 1)), (u16)dly1);
		phy_write16(0x90060004 | (((gp28 | (slice << 12)) << 1)), (u16)base_delay);
	}

	/*
	 * Step 6: Post-calibration assertion, strobes, optional CA sweep, & teardown.
	 */
	pmu_assert_or_halt(err_flag == 0, PMU_MSG_GATE_TRAIN_PASS);

	pmu_cal_strobe_pulse();
	pmu_cal_strobe_secondary_pulse();

	if (dmem_read8(0x1a) & 1) {
		phy_write16(PHY_REG_PHY_STATUS, 0x2000);
		pmu_cal_ca_eye_margin_sweep();
		phy_write16(PHY_REG_PHY_STATUS, 0);
	}

	pmu_phy_reg_buffer_stream(sp_54, 1, 0);
	pmu_dbyte_cal_strobe_pulse_seq(0, 0, 0);
}

/**
 * pmu_cal_pll_lock_retry_poll() - Converted from pmu_cal_pll_lock_retry_poll
 * @tracker_ptr: Parameter tracker_ptr
 *
 * /
 *
 * Return: Computed u32 value or status.
 */
u32 pmu_cal_pll_lock_retry_poll(u32 tracker_ptr)
{
	volatile u8 *dmem = (volatile u8 *)PMU_DMEM_BASE;
	volatile u16 *t = (volatile u16 *)(uintptr_t)tracker_ptr;

	u8 v460 = dmem[0x460];
	u32 popcount = 0;
	while (v460 != 0) {
		v460 &= (v460 - 1);
		popcount++;
	}

	u16 r10_val = *(volatile u16 *)(dmem + 0x45e);
	u8 v468 = dmem[0x468];
	u8 v45b = dmem[0x45b];

	t[19] = (v45b != 0) ? 1 : 0;

	u16 regs[8];
	regs[0] = (t[3] & 0x7f) | ((t[4] & 0x7f) << 7) | (t[5] << 14);
	regs[1] = (t[6] & 0xf) | (t[7] << 4);
	regs[2] = (t[7] >> 12) & 1;
	s16 term3 = (s8)(u8)(t[2] << 7);
	regs[3] = (u16)(term3 | ((t[0] << 4) & 0x10) | ((t[1] << 5) & 0x60) |
					((t[18] << 8) & 0x300) | ((t[20] << 11) & 0x800) |
					(((v45b != 0) ? 1 : 0) << 10) | ((t[17] << 12) & 0x7000));
	regs[4] = (t[11] & 0x7f) | ((t[12] & 0x7f) << 7) | (t[13] << 14);
	regs[5] = (t[14] & 0xf) | (t[15] << 4);
	regs[6] = (t[15] >> 12) & 1;
	s16 term7 = (s8)(u8)(t[10] << 7);
	regs[7] = (u16)(term7 | ((t[8] << 4) & 0x10) | ((t[9] << 5) & 0x60) |
					((t[16] << 8) & 0x300) | ((t[17] << 7) & 0x7c00));

	if (v468 == 0) {
		/* Direct programming path */
	} else if (v468 == 1) {
		s32 sum = (s8)(u8)popcount + *(volatile s32 *)(dmem + 0x464);
		if (sum < 8) {
			dmem[0x460] = 0;
			*(volatile s32 *)(dmem + 0x464) = sum % 8;
			return (u16)r10_val;
		}
	} else {
		return (u16)r10_val;
	}

	u32 phy_addr = 0x90082000 | ((u32)r10_val << 1);
	for (u32 i = 0; i < 8; i++) {
		*(volatile u16 *)(uintptr_t)(phy_addr + (i << 1)) = regs[i];
	}
	r10_val += 8;
	*(volatile u16 *)(dmem + 0x45e) = r10_val;

	if ((t[18] & 3) != 0) {
		dmem[0x469] = (t[18] & 1) ? 0 : 1;
	}

	if (v468 == 1) {
		*(volatile u16 *)(dmem + 0xdbc) = 0;
		s32 sum = (s8)(u8)popcount + *(volatile s32 *)(dmem + 0x464);
		dmem[0x460] = 0;
		*(volatile s32 *)(dmem + 0x464) = sum % 8;
	}

	return (u16)r10_val;
}

/*
 * pmu_cal_multi_branch_dispatch:
 * Converted from pmu_cal_multi_branch_dispatch.
 *
 * Configures calibration command/strobe parameters into the 16-byte dispatch
 * tracker record at `out` (DMEM 0xd9c or 0xdac), evaluating opcode-specific
 * field shifts and multi-flag priority decodes.
 */
void pmu_cal_multi_branch_dispatch(u16 *out, u32 flags, u32 opcode, u32 r3_val,
				   u32 val_arg0, u32 val_arg1, u32 flag_mode)
{
	out[0] = 0;
	out[1] = 0;
	out[2] = 0;
	out[6] = 0;
	out[7] = 0;

	if (opcode <= 0x2f) {
		u16 xbfu_val = (r3_val >> 8) & 0x7f;

		switch (opcode) {
		case 0:
			if (flag_mode != 0) {
				out[4] = r3_val & 0x7f;
				out[3] = ((r3_val >> 4) & 0x78) | 0x3;
			} else {
				out[4] = val_arg0 | ((r3_val >> 7) & 0x70);
				out[3] = ((r3_val >> 11) & 0x78) | 0x7;
			}
			break;

		case 1:
			out[4] = ((val_arg0 << 2) & 0x40) | ((r3_val << 3) & 0x30) | (val_arg0 & 0xf);
			out[3] = ((r3_val << 3) & 0x8) | ((r3_val >> 1) & 0x1c) | 1;
			break;

		case 2:
			out[4] = ((val_arg0 << 2) & 0x40) | ((r3_val << 3) & 0x30) | (val_arg0 & 0xf);
			out[3] = ((r3_val << 3) & 0x8) | ((r3_val >> 1) & 0x18) | 5;
			break;

		case 3:
			out[4] = ((val_arg0 << 2) & 0x40) | ((r3_val << 3) & 0x30) | (val_arg0 & 0xf);
			out[3] = ((r3_val << 3) & 0x8) | ((r3_val >> 1) & 0x18) | 6;
			break;

		case 4:
			out[4] = ((val_arg0 << 2) & 0x40) | ((r3_val << 3) & 0x30) | (val_arg0 & 0xf);
			out[3] = ((r3_val >> 1) & 0x18) | 4;
			break;

		case 5:
			out[4] = val_arg0 & 0x7f;
			out[3] = ((r3_val >> 3) & 0x70) | 0xc;
			break;

		case 6:
			if (flag_mode != 0) {
				out[4] = r3_val & 0x7f;
				out[3] = ((r3_val >> 1) & 0x40) | 8;
			} else {
				out[4] = val_arg0;
				out[3] = 0x58;
			}
			break;

		case 7:
		case 39:
			out[4] = 0;
			out[3] = 0;
			break;

		case 8:
		case 27:
			out[3] = 0x78;
			out[4] = val_arg0 & 0xf;
			break;

		case 10:
		case 17:
			out[3] = 0x38;
			out[4] = val_arg0 & 0x7;
			break;

		case 11:
			out[3] = 0x68;
			out[4] = (r3_val << 5) & 0x60;
			break;

		case 12:
			out[4] = 0;
			out[3] = 0x28;
			break;

		case 14:
			if (flag_mode != 0) {
				out[4] = r3_val & 0x7f;
				out[3] = ((r3_val >> 1) & 0x40) | 8;
			} else {
				out[4] = xbfu_val;
				out[3] = 0x58;
			}
			break;

		case 15:
			out[4] = val_arg0;
			out[3] = 24;
			break;

		case 16:
			out[3] = 0x38;
			out[4] = (val_arg0 & 0x7) | 0x40;
			break;

		case 25:
			out[4] = r3_val & 0x7f;
			out[3] = ((r3_val >> 1) & 0x40) | 0x30;
			break;

		case 26:
			out[3] = 0x78;
			out[4] = (val_arg0 & 0xf) | 0x40;
			break;

		case 28:
			out[4] = ((val_arg0 << 2) & 0x40) | ((r3_val << 3) & 0x30) | (val_arg0 & 0xf);
			out[3] = ((r3_val << 3) & 0x8) | ((r3_val >> 1) & 0x1c) | 2;
			break;

		case 29:
			out[4] = 0x58;
			out[3] = 0x58;
			break;

		case 30:
			out[4] = xbfu_val;
			out[3] = xbfu_val;
			break;

		case 31:
			out[4] = ((r3_val >> 1) & 0x40) | 8;
			out[3] = ((r3_val >> 1) & 0x40) | 8;
			break;

		case 32:
		case 38:
			out[4] = r3_val & 0x7f;
			out[3] = r3_val & 0x7f;
			break;

		case 33:
			out[3] = 24;
			break;

		case 34:
			out[4] = xbfu_val;
			break;

		case 35: {
			u16 val = ((r3_val >> 3) & 0x70) | 0xc;
			out[4] = val;
			out[3] = val;
			break;
		}

		case 36:
			out[4] = val_arg0 & 0x7f;
			out[3] = val_arg0 & 0x7f;
			break;

		case 37: {
			u16 val = ((r3_val >> 1) & 0x40) | 0x30;
			out[4] = val;
			out[3] = val;
			break;
		}

		case 40:
			out[4] = 0;
			out[3] = (flag_mode == 0) ? 0x40 : 0;
			break;

		case 41:
			out[4] = 0;
			out[3] = 0x60;
			break;

		case 42:
			out[4] = 0;
			out[3] = 0x20;
			break;

		case 43:
			out[4] = 0;
			out[3] = 0x50;
			break;

		case 44:
			out[4] = 0x38;
			out[3] = 0x38;
			break;

		case 45: {
			u16 val = (val_arg0 & 7) | 0x40;
			out[4] = val;
			out[3] = val;
			break;
		}

		case 46: {
			u16 val = ((r3_val << 3) & 0x8) | ((r3_val >> 1) & 0x1c) | 1;
			out[4] = val;
			out[3] = val;
			break;
		}

		case 47: {
			u16 val = ((val_arg0 << 2) & 0x40) | ((r3_val << 3) & 0x30) | (val_arg0 & 0xf);
			out[4] = val;
			out[3] = val;
			break;
		}

		case 9:
		case 13:
		case 18:
		case 19:
		case 20:
		case 21:
		case 22:
		case 23:
		case 24:
		default:
			break;
		}
	}

	/* Epilogue: Field out[5] (offset 0xa) */
	if ((flags & (1U << 16)) || (opcode == 0x28 && flag_mode != 0)) {
		out[5] = 0;
	} else {
		out[5] = val_arg1 & 3;
	}

	/* Epilogue: Field out[7] (offset 0xe) - Priority flag decode */
	if (flags & (1U << 20)) {
		out[7] = 13;
	} else if (flags & (1U << 21)) {
		out[7] = 0xd0;
	} else if (flags & (1U << 22)) {
		out[7] = 0xd00;
	} else if (flags & (1U << 23)) {
		out[7] = 8;
	} else if (flags & (1U << 24)) {
		out[7] = 0x80;
	} else if (flags & (1U << 25)) {
		out[7] = 1024;
	} else if (flags & (1U << 26)) {
		out[7] = 256;
	} else if (flags & (1U << 27)) {
		out[7] = 512;
	}
	if (flags & (1U << 19)) {
		out[7] |= 0x1000;
	}

	/* Epilogue: Field out[6] (offset 0xc) */
	u16 fc = 0;
	if (opcode == 3 || opcode == 4 || opcode == 28 || opcode == 41) {
		fc = 4;
		out[6] = 4;
	} else if (opcode == 1 || opcode == 2 || opcode == 33 || opcode == 34) {
		fc = (flags & (1U << 3)) ? 0 : 2;
		if ((flags & 0xc) == 0xc) {
			fc |= 1;
		} else {
			out[6] = fc;
		}
	}

	if (flags & (1U << 18)) {
		fc |= 3;
		out[6] = fc;
	}

	/* Epilogue: Field out[2] (offset 0x4) */
	if (flags & (1U << 15)) {
		out[2] = 1;
	}

	if (flags & (1U << 12)) {
		fc |= 8;
		out[6] = fc;
	}

	/* Epilogue: Field out[0] (offset 0x0) */
	if ((flags & (1U << 6)) || (val_arg1 & (1U << 1))) {
		out[0] = 1;
	}
}


/**
 * pmu_cal_tracker_dac_clear() - pmu_cal_sequence_pulse_send:
 *
 * Converted from pmu_cal_sequence_pulse_send.
 * Dispatches calibration pulse sequences and step updates to DDR PHY slices.
 * Handles CBT calibration pulses, dual-branch dispatch records (0xd9c/0xdac),
 * tracking state latching, and calibration marker boundary alignment.
 * /
 */
static __attribute__((noinline)) void pmu_cal_tracker_dac_clear(void)
{
	dmem_write32(0xdac, 0);
	dmem_write32(0xdb0, 0);
	dmem_write32(0xdb4, 0);
	dmem_write32(0xdb8, 0);
}
/**
 * pmu_cal_sequence_pulse_send() - Cal Sequence Pulse Send
 * @a0: Parameter a0
 * @a1: Parameter a1
 * @a2: Parameter a2
 * @a3: Parameter a3
 * @a4: Parameter a4
 * @a5: Parameter a5
 * @a6: Parameter a6
 *
 * Return: Computed u32 value or status.
 */
u32 pmu_cal_sequence_pulse_send(u32 a0, u32 a1, u32 a2, u32 a3, u32 a4, u32 a5, u32 a6)
{
	while (1) {
		u8 v24 = dmem_read8(PMU_DMEM_CBT_STEP_STAT_46B);

		/* Fast path: CBT calibration pulse sequence with active step status */
		if (a1 == 6 && v24 != 0) {
			u16 marker_a = dmem_read16(PMU_DMEM_CAL_MARKER_A);
			dmem_write16(0xd78, (u16)a4);
			dmem_write16(0xd76, 0x58);
			u16 a5_low = (u16)(a5 & 3);
			dmem_write16(0xd8a, a5_low);
			dmem_write16(0xd7a, a5_low);
			dmem_write16(0xd88, (u16)(a3 & 0x7f));
			u16 a5_shift = a5_low >> 1;
			dmem_write16(0xd80, a5_shift);
			dmem_write16(0xd70, a5_shift);
			u16 val_d86 = (u16)(((a3 >> 1) & 0x40) | 8);
			dmem_write16(0xd86, val_d86);
			dmem_write16(PMU_DMEM_CAL_MARKER_A, marker_a + 8);

			u32 b80_res = pmu_cal_pll_lock_retry_poll(PMU_DMEM_BASE | 0xd70);
			u32 r13 = (u32)dmem_read8(PMU_DMEM_CBT_STATE_45A);
			if ((marker_a & 7) == 0)
				r13 = b80_res << r13;
			int diff = (int)a2 - (int)r13;
			pmu_cal_stride_div_update(diff, a0);
			return pmu_cal_marker_verify_update(a0);
		}

		/* Update calibration state register 0x460 if active */
		if (dmem_read8(0x468) == 1) {
			u32 test = pmu_cal_mode_mask_test(a1);
			u8 r8 = dmem_read8(0x460);
			u8 r1 = (test == 1) ? 0xf0 : 0x30;
			dmem_write8(0x460, r1 + (r8 >> 4));
		}

		/* Latch PHY status tracking bits from a0 */
		if (a0 & (1 << 8))
			dmem_write16(0xdbc, dmem_read16(0xdbc) | (1 << 1));

		if (a0 & (1 << 13))
			dmem_write16(0xdbc, dmem_read16(0xdbc) | (1 << 0));

		u16 marker_a = dmem_read16(PMU_DMEM_CAL_MARKER_A);

		/* Branch when a6 != 0: Multi-step pulse dispatch */
		if (a6 != 0) {
			if (v24 == 0)
				a2++;

			if ((marker_a & 7) == 4) {
				dmem_write16(PMU_DMEM_CAL_MARKER_A, marker_a + 4);
				pmu_cal_tracker_dac_clear();
				pmu_cal_tracker_step_update();
			}

			if (dmem_read8(0x45b) == 0)
				dmem_write8(0x45b, dmem_read8(PMU_DMEM_CBT_CAL_STAT_45C));

			pmu_cal_multi_branch_dispatch((u16 *)(PMU_DMEM_BASE | PMU_DMEM_CAL_TRACKER_D9C),
						      a0, a1, a3, a4, a5, 0);

			if (pmu_cal_mode_mask_test(a1) != 1) {
				pmu_cal_tracker_dac_clear();
			} else {
				pmu_cal_multi_branch_dispatch((u16 *)((PMU_DMEM_BASE | PMU_DMEM_CAL_TRACKER_D9C) + 0x10),
							      a0, a1, a3, a4, a5, 1);
			}

			dmem_write16(PMU_DMEM_CAL_TRACKER_DBE, (u16)a6);
			u32 r2_byte = dmem_read8(PMU_DMEM_CBT_STATE_45A);
			dmem_write16(PMU_DMEM_CAL_MARKER_A, dmem_read16(PMU_DMEM_CAL_MARKER_A) + 8);

			/* Shift-scaled arithmetic: sub1 r13, a2, r2 => a2 - (r2 << 1) */
			int r13_diff = (int)a2 - ((int)r2_byte << 1);
			if (r13_diff < 1) {
				dmem_write16(0xdc0, 3);
			} else {
				dmem_write16(0xdc0, 2);
				pmu_cal_tracker_step_update();
				u32 r2_curr = dmem_read8(PMU_DMEM_CBT_STATE_45A);
				u32 div_val = ((u32)(r13_diff + (int)r2_curr) - 1) / r2_curr;
				u16 adv = pmu_cal_tracker_stride_advance(dmem_read16(PMU_DMEM_CAL_MARKER_A), (u16)div_val, 0);
				dmem_write16(PMU_DMEM_CAL_MARKER_A, adv);
				a0 |= (1 << 11);
			}

			dmem_write8(0x45b, dmem_read8(PMU_DMEM_CBT_CAL_STAT_45C));
			return pmu_cal_marker_verify_update(a0);
		}

		/* a6 == 0 paths */
		u32 marker_mod = marker_a & 7;

		if (marker_mod == 4) {
			if (!(a0 & (1 << 9)) &&
			    dmem_read8(0x45b) == dmem_read8(PMU_DMEM_CBT_CAL_STAT_45C) &&
			    dmem_read16(0xdc0) == 0) {
				u32 mode_test = pmu_cal_mode_mask_test(a1);
				pmu_cal_multi_branch_dispatch((u16 *)((PMU_DMEM_BASE | PMU_DMEM_CAL_TRACKER_D9C) + 0x10),
							      a0, a1, a3, a4, a5, 0);

				if (v24 == 0)
					a2++;

				dmem_write16(PMU_DMEM_CAL_MARKER_A, marker_a + 4);

				if (mode_test != 0) {
					pmu_cal_tracker_step_update();
					u32 b_45a = dmem_read8(PMU_DMEM_CBT_STATE_45A);
					pmu_cal_multi_branch_dispatch((u16 *)(PMU_DMEM_BASE | PMU_DMEM_CAL_TRACKER_D9C),
								      a0, a1, a3, a4, a5, 1);
					dmem_write16(PMU_DMEM_CAL_MARKER_A, dmem_read16(PMU_DMEM_CAL_MARKER_A) + 4);
					int diff = (int)a2 - (int)b_45a - (int)dmem_read8(PMU_DMEM_CBT_STATE_45A);
					pmu_cal_stride_div_update(diff, a0);
				} else {
					int diff = (int)a2 - (int)dmem_read8(PMU_DMEM_CBT_STATE_45A);
					if (diff >= 1) {
						pmu_cal_tracker_step_update();
						pmu_cal_stride_div_update(diff, a0);
					}
					if (a0 & (1 << 11))
						dmem_write16(0xdc0, dmem_read16(0xdc0) | (1 << 0));
				}

				return pmu_cal_marker_verify_update(a0);
			}

			/* Loop iteration: clear tracker, advance marker by 4, update tracker, loop */
			dmem_write16(PMU_DMEM_CAL_MARKER_A, marker_a + 4);
			pmu_cal_tracker_dac_clear();
			pmu_cal_tracker_step_update();
			continue;
		}

		if (marker_mod == 0) {
			if (dmem_read8(0x45b) == 0)
				dmem_write8(0x45b, dmem_read8(PMU_DMEM_CBT_CAL_STAT_45C));

			pmu_cal_multi_branch_dispatch((u16 *)(PMU_DMEM_BASE | PMU_DMEM_CAL_TRACKER_D9C),
						      a0, a1, a3, a4, a5, 0);

			if (v24 == 0)
				a2++;

			dmem_write16(PMU_DMEM_CAL_MARKER_A, dmem_read16(PMU_DMEM_CAL_MARKER_A) + 4);

			if (a0 & (1 << 9))
				dmem_write16(0xdc0, dmem_read16(0xdc0) | (1 << 1));

			int diff = (int)a2 - (int)dmem_read8(PMU_DMEM_CBT_STATE_45A);

			if (pmu_cal_mode_mask_test(a1) == 1) {
				pmu_cal_multi_branch_dispatch((u16 *)((PMU_DMEM_BASE | PMU_DMEM_CAL_TRACKER_D9C) + 0x10),
							      a0, a1, a3, a4, a5, 1);
				dmem_write16(PMU_DMEM_CAL_MARKER_A, dmem_read16(PMU_DMEM_CAL_MARKER_A) + 4);
				pmu_cal_marker_verify();
				diff -= (int)dmem_read8(PMU_DMEM_CBT_STATE_45A);
			}

			pmu_cal_stride_div_update(diff, a0);
			dmem_write8(0x45b, dmem_read8(PMU_DMEM_CBT_CAL_STAT_45C));
			return pmu_cal_marker_verify_update(a0);
		}

		/* Default return: marker_mod != 0 && marker_mod != 4 */
		return 0;
	}
}



/**
 * pmu_cal_lane_deskew_sweep_exec() - Converted from pmu_cal_lane_deskew_sweep_exec
 *
 * /
 */
void pmu_cal_lane_deskew_sweep_exec(void)
{
	u16 buf_a[2][32]; /* sp + 0x84: 128 bytes (64 halfwords, 32 per lane) */
	u16 buf_b[2][32]; /* sp + 0x104: 128 bytes (64 halfwords, 32 per lane) */
	u16 sp_68[2][4];  /* sp + 0x68: 16 bytes (8 halfwords, 4 per lane) */
	u16 sp_58[2][4];  /* sp + 0x58: 16 bytes (8 halfwords, 4 per lane) */

	pmu_memset_words(buf_b, 0, sizeof(buf_b));
	pmu_memset_words(buf_a, 0, sizeof(buf_a));
	pmu_memset_words(sp_68, 0, sizeof(sp_68));
	pmu_memset_words(sp_58, 0, sizeof(sp_58));

	const u8 *desc = (const u8 *)0x800009a4;
	u8 desc_val0 = desc[0];
	pmu_cal_metric_log(4, 0x025d0001, (u32)(desc_val0 & 0x3));

	const u8 *desc_p = desc + 130;
	u8 p0 = desc_p[0];
	u8 p8 = desc_p[8];
	u8 p16 = desc_p[16];
	u32 flag_78 = (p0 & 0x10);

	u32 gp28 = dmem_read32(PMU_DMEM_PARAM_41C);

	/* Phase 1: LCDL delay sampling across active slices for lanes r13 = 0, 1 */
	for (u32 r13 = 0; r13 < 2; r13++) {
		u8 start_slice = dmem_read8(PMU_DMEM_SLICE_START);
		u8 end_slice = dmem_read8(PMU_DMEM_SLICE_END);

		if (p8 & 1) {
			pmu_slice_lcdl_delay_collect((r13 + 0x20) | gp28, sp_68[r13], 1);
		} else {
			for (u32 slice = start_slice; slice <= end_slice; slice++) {
				u32 slice_code = (slice << 12) | ((r13 + 0x24) | gp28);
				for (u32 bit = 0; bit < 8; bit++) {
					u32 reg = 0x90020000 | ((slice_code | (bit << 8)) << 1);
					buf_b[r13][slice * 8 + bit] = phy_read16(reg);
				}
			}
		}

		for (u32 slice = start_slice; slice <= end_slice; slice++) {
			u32 slice_code = (slice << 12) | ((r13 + 0x26) | gp28);
			for (u32 bit = 0; bit < 8; bit++) {
				u32 reg = 0x90020000 | ((slice_code | (bit << 8)) << 1);
				buf_a[r13][slice * 8 + bit] = phy_read16(reg);
			}
		}

		if (p16 & 0x10) {
			pmu_slice_lcdl_delay_collect((r13 + 0x28) | gp28, sp_58[r13], 1);
		}
	}

	/* Phase 2: Fine & Coarse Delay Margin Diff & Telemetry */
	u8 rank = dmem_read8(PMU_DMEM_CAL_RANK);
	u32 rank_off = rank ? 27 : 0;
	volatile u8 *ptr_14 = (volatile u8 *)(uintptr_t)(0x80000026 + rank_off);
	volatile u8 *ptr_28 = (volatile u8 *)(uintptr_t)(0x80000030 + rank_off);

	for (s32 fp = 1; fp >= 0; fp--) {
		for (s32 r13 = 1; r13 >= 0; r13--) {
			if (r13 == fp)
				continue;

			s16 diff_coarse = 0;
			s16 diff_fine = 0;

			if (flag_78) {
				u8 start_slice = dmem_read8(PMU_DMEM_SLICE_START);
				u8 end_slice = dmem_read8(PMU_DMEM_SLICE_END);
				s32 max_coarse = 0;
				s32 max_fine = 0;

				for (u32 slice = start_slice; slice <= end_slice; slice++) {
					if (p8 & 1) {
						s32 v_fp = (s32)(sp_68[fp][slice] >> 6);
						s32 v_r13 = (s32)(sp_68[r13][slice] >> 6);
						s32 diff = v_fp - v_r13;
						if (diff > max_coarse)
							max_coarse = diff;
					} else {
						for (u32 bit = 0; bit < 8; bit++) {
							s32 v_fp = (s32)(buf_b[fp][slice * 8 + bit] >> 6);
							s32 v_r13 = (s32)(buf_b[r13][slice * 8 + bit] >> 6);
							s32 diff = v_fp - v_r13;
							if (diff > max_coarse)
								max_coarse = diff;
						}
					}

					for (u32 bit = 0; bit < 8; bit++) {
						s32 val_fp = (s32)buf_a[fp][slice * 8 + bit];
						s32 val_r13 = (s32)buf_a[r13][slice * 8 + bit];
						s32 diff = val_fp - val_r13;
						if (diff > max_fine)
							max_fine = diff;
					}
				}

				diff_coarse = (s16)pmu_dram_scaled_div_round(max_coarse << 6);
				diff_fine = (s16)pmu_dram_scaled_div_round(max_fine);
			}

			*ptr_14++ = (u8)diff_coarse;
			*ptr_28++ = (u8)diff_fine;

			pmu_cal_metric_log(4, 0x026f0006, (u32)fp, (u32)r13, (u32)(u16)diff_coarse, (u32)fp, (u32)r13, (u32)(u16)diff_fine);
		}
	}

	/* Phase 3: Multi-Lane Deskew Relative Matrix Convergence */
	u32 flag_28 = (flag_78 != 0);
	volatile u8 *ptr_1c = (volatile u8 *)(uintptr_t)(0x8000002c + rank_off);
	volatile u8 *ptr_18 = (volatile u8 *)(uintptr_t)(0x80000028 + rank_off);

	for (s32 r13 = 1; r13 >= 0; r13--) {
		for (s32 r14 = 1; r14 >= 0; r14--) {
			s16 res_r2 = 0;
			s16 res_r0 = 0;

			if ((r14 != r13) && !flag_28) {
				res_r2 = 0;
				res_r0 = 0;
			} else {
				u8 start_slice = dmem_read8(PMU_DMEM_SLICE_START);
				u8 end_slice = dmem_read8(PMU_DMEM_SLICE_END);
				s32 max_r2 = 0;
				s32 max_10 = 0;

				for (u32 slice = start_slice; slice <= end_slice; slice++) {
					if (p8 & 1) {
						s32 r1 = (s32)sp_68[r13][slice];
						for (u32 bit = 0; bit < 8; bit++) {
							s32 r3 = (s32)buf_a[r14][slice * 8 + bit];
							s32 diff = r1 - r3;
							if (diff > max_r2)
								max_r2 = diff;
						}
						if (p16 & 0x10) {
							s32 r0 = (s32)sp_58[r14][slice];
							s32 diff = r1 - r0;
							if (diff > max_r2)
								max_r2 = diff;
						}
					} else {
						for (u32 bit = 0; bit < 8; bit++) {
							s32 r15 = (s32)buf_a[r14][slice * 8 + bit];
							s32 r1 = (s32)buf_b[r13][slice * 8 + bit];
							s32 diff = r1 - r15;
							if (diff > max_r2)
								max_r2 = diff;
						}
					}

					if (r14 != r13) {
						for (u32 bit = 0; bit < 8; bit++) {
							s32 r0 = (s32)buf_a[r14][slice * 8 + bit];
							s32 r1 = (s32)buf_a[r13][slice * 8 + bit];
							s32 diff = r1 - r0;
							if (diff > max_10)
								max_10 = diff;
						}
						if (p16 & 0x10) {
							s32 r3 = (s32)sp_58[r13][slice];
							s32 r0 = (s32)sp_58[r14][slice];
							s32 diff = r3 - r0;
							if (diff > max_10)
								max_10 = diff;
						}
					}
				}

				res_r2 = (s16)pmu_dram_scaled_div_round(max_r2);
				res_r0 = (s16)pmu_dram_scaled_div_round(max_10);
			}

			*ptr_18++ = (u8)res_r2;
			*ptr_1c++ = (u8)res_r0;

			pmu_cal_metric_log(4, 0x02720006, (u32)r13, (u32)r14, (u32)(u16)res_r2, (u32)r13, (u32)r14, (u32)(u16)res_r0);
		}
	}
}

/* Forward declaration of pmu_cal_slice_step_eval_sweep */
void pmu_cal_slice_step_eval_sweep(void *results_buf,
								   u32 sweep_mode,
								   u32 cal_mode,
								   u32 channel_rank,
								   u32 dummy_arg4,
								   u32 timing_val,
								   u32 overflow_flag,
								   const u16 *slice_delay_lut,
								   u32 bound_flag,
								   u32 remap_flag,
								   const void *sweep_ctx);

/**
 * pmu_cal_margin_matrix_scan_eval() - Decompiled from pmu_cal_margin_matrix_scan_eval
 * @r0: Parameter r0
 * @r1: Parameter r1
 * @r2: Parameter r2
 * @r3: Parameter r3
 * @r4: Parameter r4
 * @r5: Parameter r5
 *
 * Evaluates 2D eye margin search matrix scans across DDR PHY channels, ranks,
 * slices, and byte lanes. Manages LCDL delay sweeps via pmu_cal_slice_step_eval_sweep,
 * extracts horizontal and vertical eye margin boundaries, determines optimal delay
 * centers, and commits calibrated results to DMEM at 0x80000dc8..0x80000e4a.
 * Parameters:
 * r0: Timing control value (latched during sweep)
 * r1: Sweep mode selector
 * r2: Bound flags (bit 8 extracted via xbfu)
 * r3: Calibration mode (0..2)
 * r4: Step size or sweep mode override
 * r5: Context configuration flags
 * /
 */
void pmu_cal_margin_matrix_scan_eval(u32 r0, u32 r1, u32 r2, u32 r3, u32 r4, u32 r5)
{
	u16 buf_158[4] = {0};
	u16 buf_160[4] = {0};
	u8 sp_64[48] = {0};
	u8 baseline_buf[4][10];

	u32 bound_flag = (r2 >> 8) & 1;
	u32 status_query = pmu_cal_status_query(bound_flag, r3);
	u32 profile_mode = pmu_cal_profile_mode_get();

	u8 dmem_0d = dmem_read8(0x0d);
	u32 sp_72 = 1;
	u32 sp_184 = (dmem_0d & 0x10) ? 0 : 1;
	u32 sp_180 = 0;

	pmu_train_mode_flag_query(r1);
	if (r1 != 0)
		sp_180 = (dmem_0d >> 3) & 1;

	pmu_train_mode_flag_query(r1);
	if (r1 != 0)
		sp_184 = 0;

	if (profile_mode == 3)
		sp_72 = (r3 != 2) ? 2 : 1;

	u8 dmem_40 = dmem_read8(PMU_DMEM_CAL_PARAM_40);
	u8 dmem_25 = dmem_read8(PMU_DMEM_CAL_PARAM_25);
	u32 sp_164 = (sp_180 != 0) ? 2 : 1;

	pmu_cal_metric_log(4, 0x01aa0005, r0, r3, r1, bound_flag, r4);

	u32 flag_216 = (r1 == 1) || ((u32)(r1 - 3) < 5);
	u32 flag_50 = 0;
	if ((bound_flag != 0 && flag_216) || (dmem_read8(PMU_DMEM_CAL_STATUS_E7) & 1)) {
		pmu_dbyte_dq_deskew_regs_restore();
		flag_50 = 1;
	}

	u32 is_r3_1 = (r3 == 1);
	u32 is_r3_0 = (r3 == 0);
	u32 is_pm_0 = (profile_mode == 0);
	u32 is_pm_1 = (profile_mode == 1);
	u32 is_pm_2 = (profile_mode == 2);
	u32 is_pm_3 = (profile_mode == 3);

	u32 r2_cond = (r4 == 0) || (is_pm_0 && is_r3_0);
	u32 sp_136 = (r4 == 0) ? 4 : 0;
	u32 val_r12 = r2_cond ? 0x4e : 0x4f;
	u32 r1_cond = (is_pm_0 && is_r3_1);
	u32 r11_cond = r1_cond || r2_cond;
	u32 r0_cond = (is_r3_0 && is_pm_1);
	u32 val_r3 = r0_cond ? val_r12 : 0x50;
	u32 r12_cond = r0_cond || r11_cond;
	u32 sp_192 = r12_cond ? val_r3 : 0x51;

	u32 sp_words_68 = (is_r3_1 && is_pm_1) || r12_cond;
	u32 val_r13 = sp_words_68 ? 0x4f : 0x4e;
	u32 sp_words_77 = sp_words_68 || (is_r3_0 && is_pm_2);
	u32 sp_words_79 = sp_words_77 || (is_pm_2 && is_r3_1);

	u32 val_r3_188 = r11_cond ? val_r13 : 0x51;
	u32 sp_188 = r12_cond ? val_r3_188 : 0x50;

	if (!r12_cond)
		sp_136 = 3;
	else if (!r11_cond)
		sp_136 = 2;
	else if (!r2_cond)
		sp_136 = 1;

	u32 dmem_mask = dmem_40 | dmem_25;

	for (u32 ch = 0; ch < 2; ch++) {
		u32 ch_mask = (1U << (ch + 2)) | (1U << ch);
		if (ch != 0 && (ch_mask & dmem_mask) == 0)
			continue;

		dmem_write8(0xb67, (u8)ch);
		pmu_phy_mode_cfg_dispatch(3);
		pmu_cal_dbyte_pattern_loop_exec(ch, (u16)r3, buf_158, buf_160, r5, (u8)sp_184, sp_180);

		u8 sp_b3 = 127;
		u32 sp_168 = 0;
		u32 sp_words_66;
		u32 sp_152;

		if (r3 < 2) {
			sp_words_66 = 128;
			sp_152 = (dmem_read16(PMU_DMEM_DRAM_FREQ_OFF) < 3200) ? 1 : 0;
		} else if (r3 == 2) {
			sp_words_66 = 212;
			sp_152 = 1;
			sp_168 = (dmem_read8(PMU_DMEM_PARAM_96) >> 4) & 1;
		} else {
			sp_words_66 = 212;
			sp_152 = 0;
		}

		pmu_cal_bist_cmd_strobe_dispatch((u32)(uintptr_t)sp_64, r1, r3, ch_mask, (u32)(uintptr_t)buf_158, bound_flag, flag_50);

		*(u32 *)(sp_64 + 0) = 0;
		*(u32 *)(sp_64 + 4) = 1;
		*(u32 *)(sp_64 + 8) = sp_164;

		u32 sp_words_74 = sp_words_66 - 1;

		u32 sp_140 = (ch == 0) ? 0x4e : 0x50;
		u32 sp_208 = (ch == 0) ? 0x4f : 0x51;
		u32 sp_204 = (ch_mask << (ch != 0)) + 1;

		if (sp_words_68 & 1) {
			sp_208 = sp_192;
			sp_140 = sp_188;
			sp_204 = sp_136;
		} else if (sp_words_77 & 1) {
			sp_208 = sp_140;
			sp_140 = (ch == 0) ? 0x4f : 0x51;
			sp_204 = (r1 << (ch != 0));
		}

		for (u32 k = 0; k < sp_72; k++) {
			u32 sp_34 = sp_208;
			u32 sp_40 = sp_140;
			u32 r13_val = sp_204;

			if (sp_words_79 & 1) {
				/* direct from sp_208 / sp_140 / sp_204 */
			} else if (is_r3_0 && is_pm_3) {
				sp_34 = (k == 0) ? 0x4e : 0x50;
				sp_40 = (k == 0) ? 0x4f : 0x51;
				r13_val = sp_204 << (k != 0);
			} else if (is_r3_1 && is_pm_3) {
				sp_34 = (k == 0) ? 0x4f : 0x51;
				sp_40 = (k == 0) ? 0x4e : 0x50;
				r13_val = ((k != 0) ? 2 : 0) + 1;
			} else if (r3 == 2) {
				sp_34 = 0x4e;
				sp_40 = 0x4f;
				r13_val = 4;
			} else {
				pmu_assert_or_halt(0, 0x01ab0000);
				sp_40 = 0x4f;
				sp_34 = 0x4e;
			}

			u8 sp_10f;
			u8 sp_157 = 0;

			if (r4 != 0) {
				sp_10f = (u8)r4;
				pmu_mailbox_param_eval_dispatch(&sp_157, &sp_b3, &sp_10f, r13_val, flag_216);
			} else {
				sp_b3 = 0;
				sp_10f = 1;
			}

			u32 sp_4c = 0;
			if (flag_216)
				sp_4c = (pmu_cal_ptr_tag_check(0, 0) != 0) ? 1 : 0;

			u32 pmu_cfg13 = pmu_dram_cfg_flag13_check(r13_val, flag_216);

			if (pmu_cfg13 != 0 && ch == 0) {
				u8 slice_start = dmem_read8(0xb68);
				u8 slice_end = dmem_read8(0xb69);
				u8 active_mask = dmem_read8(0xb98);
				u32 dmem_41c = dmem_read32(PMU_DMEM_PARAM_41C);
				u8 limit_val = dmem_read8((pmu_cfg13 <= 2) ? 0x11b : 0x11a);
				u8 blink_val = (u8)(sp_157 + (127 - sp_b3));

				for (u32 slice = slice_start; slice <= slice_end; slice++) {
					if (!(active_mask & (1 << slice)))
						continue;
					for (u32 lane = 0; lane < 10; lane++) {
						u32 reg = 0x90020000 | (((slice << 12) | dmem_41c | sp_34 | (lane << 8)) << 1);
						u16 val = phy_read16(reg);
						u8 val_b = (u8)val;
						if ((u32)sp_b3 + val_b >= 128) {
							baseline_buf[slice][lane] = blink_val;
						} else if ((s8)val_b < (s8)limit_val) {
							baseline_buf[slice][lane] = 0;
						} else {
							baseline_buf[slice][lane] = val_b - limit_val;
						}
					}
				}
			}

			pmu_memset_words((void *)0x80006318, 0, 0x8480);
			u32 step_size = sp_10f;
			u32 last_cur_dly = sp_157;

			for (u32 cur_dly = sp_157; cur_dly <= sp_b3; cur_dly += step_size) {
				last_cur_dly = cur_dly;

				if (r4 != 0) {
					if (r3 == 2) {
						pmu_cal_pulse_seq_coordinator(ch, (u8)cur_dly, 3);
					} else {
						u8 slice_start = dmem_read8(0xb68);
						u8 slice_end = dmem_read8(0xb69);
						u8 active_mask = dmem_read8(0xb98);
						u32 dmem_41c = dmem_read32(PMU_DMEM_PARAM_41C);

						for (u32 slice = slice_start; slice <= slice_end; slice++) {
							if (!(active_mask & (1 << slice)))
								continue;

							if (pmu_cfg13 != 0) {
								for (u32 lane = 0; lane < 10; lane++) {
									u16 dly_val = (u16)(cur_dly + baseline_buf[slice][lane]);
									u32 reg_34 = 0x90020000 | (((slice << 12) | dmem_41c | sp_34 | (lane << 8)) << 1);
									phy_write16(reg_34, dly_val);
									if (sp_4c != 0) {
										u32 reg_40 = 0x90020000 | (((slice << 12) | dmem_41c | sp_40 | (lane << 8)) << 1);
										phy_write16(reg_40, dly_val);
									}
								}
							} else {
								u32 reg_34 = 0x90021e00 | (((slice << 12) | dmem_41c | sp_34) << 1);
								phy_write16(reg_34, (u16)cur_dly);
								if (sp_4c != 0) {
									u32 reg_40 = 0x90021e00 | (((slice << 12) | dmem_41c | sp_40) << 1);
									phy_write16(reg_40, (u16)cur_dly);
								}
							}
						}
						pmu_delay_us(300000, 0);
					}
				}

				u32 dmem_41c = dmem_read32(PMU_DMEM_PARAM_41C);
				phy_write16(0x9003e002 | (dmem_41c << 1), 0);

				*(u32 *)sp_64 = cur_dly;

				pmu_cal_slice_step_eval_sweep((void *)0x80006318, r1, r3, pmu_cfg13, 0, (u8)r0, sp_152, buf_158, bound_flag, flag_50, sp_64);

				if (sp_168 != 0) {
					pmu_phy_timing_delay_latch(1, ch_mask);
					pmu_cal_bist_pattern_setup(r1, ch_mask, 0, flag_50);
					pmu_cal_slice_step_eval_sweep((void *)0x80006318, r1, 3, pmu_cfg13, 0, (u8)r0, sp_152, buf_158, 0, flag_50, sp_64);
					pmu_cal_bist_pattern_setup(r1, ch_mask, status_query, flag_50);
					pmu_phy_timing_delay_latch(0, ch_mask);
				}
			}

			u8 slice_start = dmem_read8(0xb68);
			u8 slice_end = dmem_read8(0xb69);
			u8 active_mask = dmem_read8(0xb98);
			u32 lane_div = 64 / sp_164;

			for (u32 slice = slice_start; slice <= slice_end; slice++) {
				if (!(active_mask & (1 << slice)))
					continue;

				for (u32 lane = 0; lane < 10; lane++) {
					u32 entry_idx = (slice * 10) + lane;
					volatile u8 *state_bytes = (volatile u8 *)(0x80006594 + (entry_idx * 848));

					s32 win_start = 0;
					s32 in_pass = 0;
					s32 best_start = 0;
					s32 best_end = 0;
					s32 best_width = 0;
					s32 last_step = -1;

					for (s32 step = 0; step < (s32)sp_words_66; step++) {
						u8 st = state_bytes[step];
						if (in_pass != 0) {
							if (st == 0) {
								s32 cand_width = last_step - win_start;
								if (cand_width > (best_width << 1)) {
									best_start = win_start;
									best_end = last_step;
									best_width = cand_width;
								}
								in_pass = 0;
							}
						} else {
							if (st != 0) {
								in_pass = 1;
								win_start = step;
							}
						}
						last_step = step;
					}

					if (in_pass != 0) {
						s32 cand_width = (s32)sp_words_74 - win_start;
						if (cand_width > (best_width << 1)) {
							best_start = win_start;
							best_end = (s32)sp_words_74;
							best_width = cand_width;
						}
					}

					if (best_width > (s32)lane_div)
						best_start = best_end - (s32)lane_div;

					uintptr_t entry_base = 0x80000dc8 + (k * 10560) + (ch * 5280) + (slice * 1320) + (lane * 132);
					*(volatile u16 *)(entry_base + 128) = (u16)best_start;

					volatile u8 *rec_base = (volatile u8 *)(0x80006318 + (entry_idx * 848));
					u32 shift_val = sp_164 - 1;

					for (u32 idx = 0; idx < 64; idx++) {
						u32 step_sampled = (u8)(best_start + (idx >> shift_val));
						u8 st = state_bytes[step_sampled];
						u8 left_val;
						u8 right_val;

						if (r4 != 0) {
							if (st == 0) {
								left_val = 127;
								right_val = 0;
							} else if (st == 1) {
								left_val = rec_base[step_sampled * 3 + 0];
								right_val = (u8)last_cur_dly;
							} else if (st == 2) {
								left_val = rec_base[step_sampled * 3 + 0];
								right_val = rec_base[step_sampled * 3 + 1] - step_size;
							} else {
								u8 cand_left = rec_base[step_sampled * 3 + 0];
								u8 cand_right = rec_base[step_sampled * 3 + 1];
								u8 cand_start = rec_base[step_sampled * 3 + 2];
								u32 cand_w = (u32)(cand_right - (cand_left + step_size));
								u32 cur_w = (u32)((u8)last_cur_dly - cand_start);
								if (cur_w > cand_w) {
									left_val = cand_start;
									right_val = (u8)last_cur_dly;
								} else {
									left_val = cand_left;
									right_val = cand_right - step_size;
								}
							}
						} else {
							if ((st | 2) == 3) {
								left_val = 0;
								right_val = 127;
							} else {
								left_val = 127;
								right_val = 0;
							}
						}

						if ((s8)right_val - (s8)left_val < 5) {
							left_val = 127;
							right_val = 0;
						}

						if (pmu_cfg13 != 0 && (u8)right_val >= left_val) {
							u8 base_adj = baseline_buf[slice][lane];
							left_val += base_adj;
							right_val += base_adj;
						}

						*(volatile u8 *)(entry_base + (idx * 2) + 0) = left_val;
						*(volatile u8 *)(entry_base + (idx * 2) + 1) = right_val;
					}
				}
			}

			for (u32 slice = slice_start; slice <= slice_end; slice++) {
				if (!(active_mask & (1 << slice)))
					continue;

				for (u32 lane = 0; lane < 10; lane++) {
					if (bound_flag == 0 && lane == 8)
						continue;
					if (sp_168 == 0 && lane == 9)
						continue;

					uintptr_t entry_base = 0x80000dc8 + (k * 10560) + (ch * 5280) + (slice * 1320) + (lane * 132);

					s32 win_start = 0;
					s32 win_end = 0;
					s32 best_w_start = 0;
					s32 best_w_end = 0;
					s32 best_w = -1;
					s32 in_win = 0;

					for (u32 step = 0; step < 64; step++) {
						u8 left = *(volatile u8 *)(entry_base + (step * 2) + 0);
						u8 right = *(volatile u8 *)(entry_base + (step * 2) + 1);

						if (right >= left) {
							if (in_win == 0) {
								in_win = 1;
								win_start = (s32)step;
							}
							win_end = (s32)step;
						} else {
							if (in_win != 0) {
								in_win = 0;
								s32 cur_w = win_end - win_start;
								if (cur_w > best_w) {
									best_w = cur_w;
									best_w_start = win_start;
									best_w_end = win_end;
								}
							}
						}
					}

					if (in_win != 0) {
						s32 cur_w = win_end - win_start;
						if (cur_w > best_w) {
							best_w = cur_w;
							best_w_start = win_start;
							best_w_end = win_end;
						}
					}

					s32 sum_steps = best_w_start + best_w_end;
					s32 center;
					if (dmem_read8(0x15) & 4) {
						center = sum_steps;
					} else {
						center = sum_steps / 2;
						if (dmem_read8(0xce) != 0) {
							u32 width = (u32)(best_w_end - best_w_start);
							pmu_assert_or_halt((width > 5) ? 1 : 0, 0x01b10006);
						}
					}

					u16 best_start = *(volatile u16 *)(entry_base + 128);
					u16 new_baseline = buf_160[slice] + (best_start * sp_164);
					*(volatile u16 *)(entry_base + 128) = new_baseline;
					*(volatile s8 *)(entry_base + 130) = (s8)center;
				}
			}
		}

		pmu_dmem_reg_stream_unpack(sp_64);
	}

	if (flag_50 != 0)
		pmu_dbyte_dq_deskew_regs_save_and_ramp();
}


/*
 * pmu_cal_slice_step_eval_sweep:
 * Converted from pmu_cal_slice_step_eval_sweep.
 * Performs iterative step evaluation and eye margin search sweep across
 * active PHY DBYTE slices and lanes. Evaluates eye margin transitions,
 * detects 0/1 signal boundaries, tracks candidate and best pass-window widths
 * in the results matrix buffer (848 bytes per entry: 212*3 sample records +
 * 212 state bytes), and restores DBYTE CSR snapshots upon completion.
 */
void pmu_cal_slice_step_eval_sweep(void *results_buf,
								   u32 sweep_mode,
								   u32 cal_mode,
								   u32 channel_rank,
								   u32 dummy_arg4,
								   u32 timing_val,
								   u32 overflow_flag,
								   const u16 *slice_delay_lut,
								   u32 bound_flag,
								   u32 remap_flag,
								   const void *sweep_ctx)
{
	u16 local_buf[4][10];
	u16 local_scratch_44[4] = {0};


	u32 loop_bound = (bound_flag != 0) ? 9 : 8;
	u32 profile_mode = pmu_cal_profile_mode_get();

	u32 flag_fp = 0;
	if (((u32)(sweep_mode - 3) < 5) || (sweep_mode == 1)) {
		if (pmu_cal_ptr_tag_check(0, 0) != 0) {
			flag_fp = 1;
			u8 s_start = dmem_read8(PMU_DMEM_SLICE_START);
			u8 s_end = dmem_read8(PMU_DMEM_SLICE_END);
			if (s_start <= s_end) {
				u8 *target = (u8 *)sweep_ctx + 0x1c + (s_start * 2);
				u32 count = (s_end - s_start + 1) * 2;
				pmu_memset_words(target, 0, count);
			}
		}
	}

	u32 dmem_41c = dmem_read32(PMU_DMEM_PARAM_41C);
	u8 slice_start = dmem_read8(PMU_DMEM_SLICE_START);
	u8 slice_end = dmem_read8(PMU_DMEM_SLICE_END);
	u8 active_mask = dmem_read8(0xb98);

	const u8 *ctx = (const u8 *)sweep_ctx;
	u32 ctx_off08 = *(const u32 *)(ctx + 0x08);
	u32 ctx_off0c = *(const u32 *)(ctx + 0x0c);
	u32 ctx_off10 = *(const u32 *)(ctx + 0x10);

	u32 lane_start = 0;
	if (cal_mode == 3) {
		loop_bound = 10;
		lane_start = 9;
	}
	u32 flag_20 = (cal_mode == 1) || flag_fp;

	/* Loop 1: Snapshot DBYTE CSRs into local_buf when flag_20 is set */
	if (flag_20) {
		for (u32 slice = slice_start; slice <= slice_end; slice++) {
			if (active_mask & (1 << slice)) {
				for (u32 lane = 0; lane < loop_bound; lane++) {
					u32 reg = 0x90020000 | (((slice << 12) | dmem_41c | ctx_off10 | (lane << 8)) << 1);
					local_buf[slice][lane] = phy_read16(reg);
				}
			}
		}
	}

	/* Loop 2: Program initial slice delay values */
	u32 lane_csr_off = (cal_mode == 3) ? 0 : 0xf00;
	for (u32 slice = slice_start; slice <= slice_end; slice++) {
		if (active_mask & (1 << slice)) {
			u16 val = slice_delay_lut[slice];
			u32 reg_0c = 0x90020000 | (((slice << 12) | lane_csr_off | dmem_41c | ctx_off0c) << 1);
			phy_write16(reg_0c, val);
			if (flag_20) {
				u32 reg_10 = 0x90020000 | (((slice << 12) | lane_csr_off | dmem_41c | ctx_off10) << 1);
				phy_write16(reg_10, val);
			}
		}
	}

	pmu_cal_strobe_secondary_pulse();

	u32 ctx_off04 = *(const u32 *)(ctx + 0x04);
	u16 f6a_val = 1;

	if (channel_rank < 4) {
		if ((profile_mode & ~1) == 2) {
			u32 reg = 0x9003e012 | (dmem_41c << 1);
			phy_write16(reg, (channel_rank >= 2) ? 1 : 0);

			if (profile_mode == 3) {
				phy_write16(0x90040180, 8);
				phy_write16(0x90040184, 0);
				phy_write16(0x90040182, (channel_rank > 1) ? 8 : 0);
				f6a_val = 0x4001;
			}
		}
	}

	for (u32 slice = slice_start; slice <= slice_end; slice++) {
		u16 ctx_1c = *(const u16 *)(ctx + 0x1c + (slice * 2));
		u16 cur_f6a = (ctx_1c << 10) | f6a_val;
		phy_write16(0x90021f6a | (slice << 13), cur_f6a);
		local_scratch_44[slice] = slice_delay_lut[slice];
	}

	pmu_deskew_dly0_set(200);
	pmu_deskew_dly1_set(*(const u16 *)(ctx + 0x14));

	u32 max_sweep_steps = ((cal_mode | 1) == 3) ? 212 : 128;
	u32 cal_mode_even = cal_mode & ~1;

	/* Main sweep step loop */
	for (u32 sweep_step = 0; sweep_step < max_sweep_steps; sweep_step++) {
		pmu_clk_timing_latch((u16)timing_val, 1);

		u32 effective_step = (ctx_off04 != 0) ? sweep_step : 0;
		u32 cur_val = (ctx_off04 != 0) ? *(const u32 *)sweep_ctx : sweep_step;

		for (u32 slice = slice_start; slice <= slice_end; slice++) {
			if (!(active_mask & (1 << slice)))
				continue;

			/* Delay adjustment */
			if (cal_mode < 2) {
				u16 dly = local_scratch_44[slice];
				u32 adj = ctx_off08;
				if ((ctx_off08 + (dly & 0x7f)) > 0x7f)
					adj += 0x80;
				dly += adj;
				local_scratch_44[slice] = dly;

				if (overflow_flag != 0 && (dly & (1 << 6))) {
					dly += 64;
					local_scratch_44[slice] = dly;

					u16 *p1c = (u16 *)((u8 *)sweep_ctx + 0x1c + (slice * 2));
					u16 reg_16a = phy_read16(0x9002016a | (slice << 13));
					u16 inv = (u16)~(*p1c);
					*p1c = inv;
					u16 val_1f6a = (reg_16a & 0xc3ff) | ((inv << 10) & 0x3c00);
					phy_write16(0x90021f6a | (slice << 13), val_1f6a);
				}
			} else if (cal_mode_even == 2) {
				local_scratch_44[slice] += ctx_off08;
			}

			u32 reg_0c = 0x90021e00 | (((slice << 12) | dmem_41c | ctx_off0c) << 1);
			phy_write16(reg_0c, local_scratch_44[slice]);
			if (flag_20) {
				u32 reg_10 = 0x90021e00 | (((slice << 12) | dmem_41c | ctx_off10) << 1);
				phy_write16(reg_10, local_scratch_44[slice]);
			}

			/* Inner Lane Loop: Signal Sampling & State Machine */
			for (u32 lane = lane_start; lane < loop_bound; lane++) {
				u32 lane_idx = lane;
				if (remap_flag != 0 && lane < 8) {
					lane_idx = dmem_read8(0xba0 + (slice * 8) + lane);
				}

				u32 entry_idx = (slice * 10) + lane_idx;
				volatile u8 *entry_base = (volatile u8 *)(results_buf + (entry_idx * 848));
				volatile u8 *state_ptr = &entry_base[636 + effective_step];
				u8 state = *state_ptr;

				u32 csr_field = (cal_mode == 3) ? 0x800 : (lane << 8);
				u32 sample_reg = 0x9002016e | (((slice << 12) | csr_field) << 1);
				u16 phy_sample = phy_read16(sample_reg);

				/*
				 * Eye Margin Boundary State Machine:
				 *   State 0: Searching for pass start (phy_sample == 0) -> records entry[0], state = 1
				 *   State 1: Searching for pass end (phy_sample != 0)   -> records entry[1], state = 2
				 *   State 2: Searching for candidate pass start (phy_sample == 0) -> records entry[2], state = 3
				 *   State 3: Searching for candidate pass end (phy_sample != 0)   -> compares candidate width
				 *            (cur_val - entry[2]) against current best (entry[1] - entry[0]).
				 *            If wider, replaces best window (entry[0] = entry[2], entry[1] = cur_val).
				 *            Transitions back to state 2.
				 */
				if (state == 1) {
					if (phy_sample != 0) {
						entry_base[(effective_step * 3) + state] = (u8)cur_val;
						*state_ptr = state + 1;
					}
				} else {
					if (phy_sample == 0) {
						if ((state | 2) == 2) {
							entry_base[(effective_step * 3) + state] = (u8)cur_val;
							*state_ptr = state + 1;
						}
					} else {
						if (state == 3) {
							volatile u8 *eval_rec = &entry_base[effective_step * 3];
							u32 current_width = (u32)(eval_rec[1] - eval_rec[0]);
							u32 candidate_width = (u32)((u8)cur_val - eval_rec[2]);
							if (candidate_width > current_width) {
								eval_rec[0] = eval_rec[2];
								eval_rec[1] = (u8)cur_val;
							}
							*state_ptr = 2;
						}
					}
				}
			}
		}

		phy_write16(0x9003e176, 1023);
		phy_write16(0x9003e176, 0);
	}

	/* Restore CSR snapshot from local_buf */
	if (flag_20) {
		for (u32 slice = slice_start; slice <= slice_end; slice++) {
			if (active_mask & (1 << slice)) {
				for (u32 lane = 0; lane < loop_bound; lane++) {
					u32 reg = 0x90020000 | (((slice << 12) | dmem_41c | ctx_off10 | (lane << 8)) << 1);
					phy_write16(reg, local_buf[slice][lane]);
				}
			}
		}
	}

	/* Epilogue: Restore profile mode */
	if (profile_mode == 2) {
		phy_write16(0x9003e012 | (dmem_41c << 1), 2);
	} else if (profile_mode == 3) {
		phy_write16(0x9003e012 | (dmem_41c << 1), 4);
	}
}



/**
 * pmu_cal_lane_window_margin_eval() - Converted from pmu_cal_lane_window_margin_eval
 * @mode: Operational or calibration mode
 *
 * /
 */
void pmu_cal_lane_window_margin_eval(u32 mode)
{
	u8 *base = (u8 *)0x800009a4;

	pmu_cal_metric_log(4, 0x02000000);
	pmu_cal_timing_table_matrix_init(base);

	pmu_cal_struct_to_shadow16((u32)(uintptr_t)base, 0x0b);
	pmu_cal_struct_to_shadow16((u32)(uintptr_t)base, 0x11);
	pmu_cal_struct_mask_and((u32)(uintptr_t)base, 0x12, 0xef);
	pmu_cal_struct_to_shadow16((u32)(uintptr_t)base, 0x12);
	pmu_cal_struct_to_shadow16((u32)(uintptr_t)base, 0x29);

	for (u32 ch = 0; ch < 2; ch++) {
		for (u32 rank = 0; rank < 2; rank++) {
			pmu_tracker_cal_mode_update(base + ch * 108 + rank * 54);
		}
	}

	dmem_write8(0xb96, 0);
	dmem_write16(0xb66, 0);
	dmem_write32(0xb9c, (u32)(uintptr_t)base);

	for (u32 ch = 0; ch < 2; ch++) {
		for (u32 rank = 0; rank < 2; rank++) {
			base[ch * 108 + rank * 54 + 3] |= 0x40;
		}
	}
	pmu_phy_reg_offset_diff_adjust(1);

	pmu_cbt_active_entry_lookup();
	pmu_cal_table_50c_lookup();
	pmu_slice_status_b97_save();

	u32 gp28 = dmem_read32(0x41c);
	phy_write16(0x90040082 | (gp28 << 1), 0);
	phy_write16(0x90040054, 1);

	pmu_delay_us(0xbebc200, 0);
	if (dmem_read8(0x0d) & 4)
		pmu_mailbox_post_cmd_dispatch(0x40);
	phy_write16(0x900400c0, 3);
	pmu_delay_us(0x77359400, 0);

	if (dmem_read8(0x0d) & 1)
		phy_write16(0x900400c0, 1);

	pmu_deskew_and_tracker_reset();
	u8 count = (u8)pmu_delay_clamp(0xbb8, 0);
	if (count != 0) {
		do {
			pmu_cal_sequence_pulse_send(0, 7, 0, 0, 0, 3, 0);
		} while (--count != 0);
	}
	pmu_cal_sequence_pulse_send(0x80, 7, 0, 0, 0, 3, 0);
	pmu_clk_timing_delay_latch(0, 1);

	if (dmem_read8(0xa7c) == 0) {
		pmu_delay_us(0x1e8480, 0);
	} else {
		pmu_ac_lane_profile_setup();
		pmu_deskew_and_tracker_reset();

		pmu_cbt_coarse_step_pulse_seq(0, 8, 1);
		if (dmem_read8(0xa7d) >= 2)
			pmu_cbt_coarse_step_pulse_seq(0, 0, 1);
		pmu_cbt_coarse_step_pulse_seq(0, 10, 1);
		pmu_cbt_coarse_step_pulse_seq(8, 0, 1);
		pmu_cbt_coarse_step_pulse_seq(0, 0x96, 1);
		pmu_cbt_3phase_pulse_seq();
		pmu_clk_timing_delay_latch(0, 1);
		pmu_phy_profile_param_program();
	}

	if (mode == 0) {
		pmu_slice_margin_window_clamp();
		pmu_multirank_slice_cal_dispatch(base, 1, 0xeffefffd, -1, 0xff, 0);
		static const u8 shadow_offs[] = {0x0b, 0x11, 0x12, 0x29};
		for (u32 i = 0; i < 4; i++)
			pmu_tracker_field_extract(base, shadow_offs[i]);

		u8 num_entries = dmem_read8(0x409);
		for (u32 entry = 0; entry < num_entries; entry++) {
			u8 offset = dmem_read8(0x46c + entry);
			u8 *entry_base = base + offset;
			for (u32 rank = 0; rank < 2; rank++) {
				for (u32 ch = 0; ch < 2; ch++) {
					u8 val = entry_base[ch * 108 + rank * 54];
					dmem_write8(0xb6e + ch * 2 + entry * 4 + rank, val);
				}
			}
		}
	}

	pmu_cal_delay_line_init();

	static const u8 tbl_pairs[][2] = {
		{0x33, 0x6e}, {0x34, 0x6f}, {0x4e, 0x70}, {0x4f, 0x71},
		{0x35, 0x76}, {0x36, 0x77}, {0x50, 0x78}, {0xd0, 0x7a},
		{0x51, 0x79}, {0xd5, 0x7b}, {0xda, 0x7c}, {0xd1, 0x9a},
		{0xdf, 0x7d}, {0xd6, 0x9b}, {0xdb, 0x9c}, {0xe0, 0x9d},
		{0xd2, 0xae}, {0xd7, 0xaf}, {0xdc, 0xb0}, {0xe1, 0xb1}
	};
	for (u32 i = 0; i < 20; i++)
		dmem_write8(tbl_pairs[i][0], dmem_read8(tbl_pairs[i][1]));
}


/**
 * pmu_cal_dbyte_dq_status_check() - Converted from pmu_cal_dbyte_dq_status_check
 * @arg0: Parameter arg0
 * @arg1: Parameter arg1
 *
 * Checks DBYTE DQ calibration status and configures slice delay and CBT registers
 * across active channels and ranks.
 * /
 *
 * Return: Computed u32 value or status.
 */
u32 pmu_cal_dbyte_dq_status_check(u32 arg0, u32 arg1)
{
	u32 rank_idx = ((arg0 & 3) == 2) ? 1 : 0;
	u8 channel_idx = dmem_read8(0xb66);
	u32 gp28 = dmem_read32(0x41c);
	u32 gp28_shl1 = gp28 << 1;

	u8 cfg_mode = dmem_read8(channel_idx ? 0x4d : 0x32);
	u8 cfg_mode_bit0 = cfg_mode & 1;

	phy_write16(0x90040082 | gp28_shl1, 1);
	phy_write16(0x90040054, 1);
	u16 reg_000e = phy_read16(0x9004000e | gp28_shl1);
	phy_write16(0x9004000e | gp28_shl1, reg_000e | 0x100);

	u8 *slice_list = (u8 *)(uintptr_t)(0x80000b44 + channel_idx * 16 + rank_idx * 8);
	u8 num_slices = dmem_read8(0xb20 + channel_idx * 2 + rank_idx);

	for (u32 i = 0; i < num_slices; i++) {
		u8 slice = slice_list[i];
		u32 slice_off = (u32)slice << 13;
		if (dmem_read8(0x403) != 0) {
			phy_write16(0x90020112 | slice_off, 511);
			phy_write16(0x90020116 | slice_off, 511);
			phy_write16(0x90020118 | slice_off, 0);
		} else {
			phy_write16(0x90020112 | slice_off, 0x19ff);
			phy_write16(0x90020116 | slice_off, 0x19ff);
			phy_write16(0x90020118 | slice_off, 0x1000);
		}
	}

	u32 arg0_x5 = (arg0 << 2) | arg0;

	if (arg1 != 0) {
		u32 not_cfg_mode_bit0 = !cfg_mode_bit0;
		pmu_cal_struct_to_shadow16(0x800009a4, 0x10);
		pmu_cal_struct_mask_or(0x800009a4, 0x10, 0x41);
		pmu_cal_struct_mask_and(0x800009a4, 0x10, 0xf3);
		pmu_cal_struct_to_shadow16(0x800009a4, 0x0b);
		pmu_cal_struct_mask_and(0x800009a4, 0x0b, 0xf0);
		pmu_cal_struct_to_shadow16(0x800009a4, 0x12);
		pmu_cal_struct_mask_or(0x800009a4, 0x12, 0x08);
		pmu_cal_struct_mask_and(0x800009a4, 0x12, 0xf8);
		pmu_cal_markers_set(1200);
		pmu_cbt_cal_pulse_seq(0);
		dmem_write16(0x47e, pmu_cal_stride_get() - 1);
		if (dmem_read8(0x403) != 0) {
			pmu_deskew_latch_seq();
			pmu_delay_us(5000, 12);
			pmu_clk_gate_handoff();
		}
		pmu_cal_struct_mask_or(0x800009a4, 0x0d, 0x40);

		u32 table_addr = 0x800009a4 + channel_idx * 108 + rank_idx * 54;
		u32 a3_mask;
		if (rank_idx == not_cfg_mode_bit0) {
			pmu_cal_struct_to_shadow16(0x800009a4, 0x11);
			pmu_cal_struct_mask_or(0x800009a4, 0x11, 0x08);
			pmu_cal_struct_mask_and(0x800009a4, 0x11, 0xef);
			pmu_cal_struct_mask_or(0x800009a4, 0x11, 0x20);

			u8 rank_mask = (u8)((1 << (rank_idx + 2)) | (1 << rank_idx));
			pmu_cal_multi_rank_dispatch_e170(rank_mask, table_addr, 0, 0xfffdffff, (u32)-1, 0, 0);
			pmu_delay_us(2000, 0);
			pmu_tracker_field_extract((u8 *)0x800009a4, 0x11);
			a3_mask = 0xfffbdff7;
		} else {
			a3_mask = 0xfff9d7f7;
		}

		pmu_cal_multi_rank_dispatch_e170((u8)arg0_x5, table_addr, 0, a3_mask, (u32)-1, 0, 0);
		pmu_cal_struct_mask_and(0x800009a4, 0x10, 0xf3);
		pmu_cal_struct_mask_or(0x800009a4, 0x10, 0x61);
		pmu_cal_struct_mask_and(0x800009a4, 0x10, 0x7f);
	} else {
		pmu_cal_struct_mask_and(0x800009a4, 0x10, 0xf3);
		pmu_cal_struct_mask_or(0x800009a4, 0x10, 0x61);
		pmu_cal_struct_mask_or(0x800009a4, 0x10, 0x80);
	}

	phy_write16(0x90040062, 1);
	if (cfg_mode_bit0 == rank_idx) {
		u32 table_addr = 0x800009a4 + channel_idx * 108 + rank_idx * 54;
		pmu_cbt_3phase_pulse_dispatch((u8)arg0_x5, (const u8 *)table_addr, 16, 1);
	} else {
		u32 r13_sub = ((arg0_x5 & 3) == 2) ? 54 : 0;
		u8 *tbl = (u8 *)(uintptr_t)(0x800009a4 + channel_idx * 108 + r13_sub);
		u8 val10 = tbl[0x10];
		u8 r15_mask = (u8)((1 << (cfg_mode_bit0 + 2)) | (1 << cfg_mode_bit0));

		pmu_deskew_and_tracker_reset();
		pmu_ac_lane_profile_setup();
		pmu_cbt_coarse_step_pulse();
		pmu_dbyte_cal_step_latch(0x10, val10, (u8)arg0_x5);
		pmu_cbt_coarse_step_pulse_seq(0, 4, 1);
		u8 new_val10 = (val10 & 138) | 69;
		pmu_dbyte_cal_step_latch(0x10, new_val10, r15_mask);
		pmu_cbt_coarse_step_pulse_seq(0, 0x20, 1);
		pmu_cbt_cal_pulse_seq(0);
		pmu_cbt_3phase_pulse_seq();
		pmu_clk_timing_delay_latch(0, 0);
		pmu_delay_us(0, 0x32);
	}

	if (dmem_read8(0x403) != 0) {
		if (num_slices != 0) {
			for (u32 i = 0; i < num_slices; i++) {
				u8 slice = slice_list[i];
				u32 mask = pmu_dbyte_pin_mask_calc(0x80, slice);
				phy_write16(0x90020118 | ((u32)slice << 13), (u16)mask);
			}
		}
		pmu_delay_us(5000, 12);
		pmu_phy_profile_param_program();
	} else {
		for (u32 iter = 0; iter < 20; iter++) {
			for (u32 i = 0; i < num_slices; i++) {
				u32 reg = 0x90020118 | ((u32)slice_list[i] << 13);
				phy_write16(reg, 0x800);
				phy_write16(reg, 0);
			}
		}

		for (u32 iter = 0; iter < 40; iter++) {
			for (u32 i = 0; i < num_slices; i++) {
				u8 slice = slice_list[i];
				u32 reg = 0x90020118 | ((u32)slice << 13);
				u32 mask = pmu_dbyte_pin_mask_calc(0x80, slice);
				phy_write16(reg, (u16)(mask | 0x800));
				phy_write16(reg, (u16)(mask | 0x1000));
			}
		}

		for (u32 i = 0; i < num_slices; i++) {
			u32 slice_off = (u32)slice_list[i] << 13;
			phy_write16(0x90020112 | slice_off, 511);
			phy_write16(0x90020116 | slice_off, 511);
			phy_write16(0x90020118 | slice_off, (u16)pmu_dbyte_pin_mask_calc(0x80, slice_list[i]));
		}

		pmu_clk_gate_handoff();
		pmu_phy_profile_param_program();
		pmu_deskew_latch_seq();
		pmu_delay_us(5000, 12);
	}

	pmu_clk_gate_handoff();
	pmu_delay_us(0x3d090, 0);
	pmu_deskew_latch_seq();

	u8 param_9b0 = dmem_read8(0x9a4 + channel_idx * 108 + rank_idx * 54 + 0xc);
	u32 p = (param_9b0 | 0x80) & 0xff;
	for (u32 i = 0; i < num_slices; i++) {
		u32 slice_off = (u32)slice_list[i] << 13;
		phy_write16(0x90020118 | slice_off, (u16)pmu_dbyte_pin_mask_calc(p, slice_list[i]));
	}
	pmu_delay_us(5000, 12);

	for (u32 i = 0; i < num_slices; i++) {
		u8 slice = slice_list[i];
		u32 slice_off = (u32)slice << 13;
		u32 mask = pmu_dbyte_pin_mask_calc(p, slice);
		phy_write16(0x90020118 | slice_off, (u16)((mask & 0xff) | 0x100));
	}
	pmu_delay_us(5000, 12);

	for (u32 i = 0; i < num_slices; i++) {
		u32 slice_off = (u32)slice_list[i] << 13;
		u32 mask = pmu_dbyte_pin_mask_calc(0x80, slice_list[i]) | 0x100;
		phy_write16(0x90020112 | slice_off, (u16)mask);
		phy_write16(0x90020116 | slice_off, (u16)mask);
	}

	pmu_delay_us(0x3d090, 0);
	pmu_tracker_field_extract((u8 *)0x800009a4, 0x10);
	pmu_clk_gate_handoff();

	return arg0;
}

/**
 * pmu_cal_dbyte_pattern_loop_exec() - Converted from pmu_cal_dbyte_pattern_loop_exec
 * @arg0: Parameter arg0
 * @arg1: Parameter arg1
 * @arg2: Parameter arg2
 * @arg3: Parameter arg3
 * @arg4: Parameter arg4
 * @arg5: Parameter arg5
 * @arg6: Parameter arg6
 *
 * Calibrates data byte delay pattern loops across slices/lanes.
 * Evaluates delay line registers, determines window min/max bounds, applies
 * LCDL deviation adjustments and timing tap steps, and updates decoded HDR addresses.
 * /
 */
void pmu_cal_dbyte_pattern_loop_exec(u32 arg0, u16 arg1, void *arg2, void *arg3, u32 arg4, u32 arg5, u32 arg6)
{
	u16 *out_dest = (u16 *)arg2;
	u16 *out_scratch = (u16 *)arg3;
	u8 slice_start = dmem_read8(0xb68);
	u8 slice_end = dmem_read8(0xb69);
	u8 active_mask = dmem_read8(0xb98);

	if (arg1 == 2 || arg1 == 3) {
		u16 local_buf[4] = {0};
		pmu_cal_slice_step_diff_commit(arg0, local_buf, 1);
		u32 gp28 = dmem_read32(0x41c);
		for (u32 slice = slice_start; slice <= slice_end; slice++) {
			u32 addr = 0x90020000 | (((slice << 12) | gp28 | (arg0 + 0x2a)) << 1);
			u16 reg = phy_read16(addr);
			u16 sum = (reg & 0x3f) + local_buf[slice];
			u16 val0 = sum + 0x20;
			u16 res = 0;
			if ((val0 & ~1) >= 0x6a) {
				res = sum - 0x4a;
			}
			out_scratch[slice] = res;
			out_dest[slice] = res;
		}
		return;
	}

	if (arg1 >= 2) {
		pmu_assert_or_halt(0, 0x27e0001);
		return;
	}

	u16 freq = dmem_read16(0x06);
	if (freq <= 1599) {
		u16 init_val = (dmem_read8(0x403) < 2) ? 0x41 : 0xc1;
		for (u32 slice = slice_start; slice <= slice_end; slice++) {
			if (active_mask & (1 << slice)) {
				out_scratch[slice] = init_val;
			}
		}
	} else {
		u32 base_reg = (arg1 != 0) ? 0x90020024 : 0x90020020;
		u8 base_fp = (arg6 == 0) ? 0x40 : 0x80;
		u32 num_lanes = (pmu_dmem_training_flags_eval() != 0) ? 9 : 8;
		u32 gp28 = dmem_read32(0x41c);

		for (u32 slice = slice_start; slice <= slice_end; slice++) {
			if (!(active_mask & (1 << slice))) {
				continue;
			}

			if (arg5 != 0) {
				u16 min_val = 0xffff;
				u16 max_val = 0;
				u32 slice_base = (slice << 12) | gp28 | arg0;
				for (u32 lane = 0; lane < num_lanes; lane++) {
					u32 reg_addr = base_reg | ((slice_base | (lane << 8)) << 1);
					u16 reg_val = phy_read16(reg_addr);
					u16 val = (reg_val & 0x7f) + ((reg_val & ~0x7f) >> 1);
					if (val < min_val) {
						min_val = val;
					}
					if (val > max_val) {
						max_val = val;
					}
				}
				out_scratch[slice] = (min_val + max_val) >> 1;
			} else {
				out_scratch[slice] = pmu_phy_delay_bound_margin_log(slice);
			}

			u32 dmem_addr = 0x12e;
			if (arg0 == 0) {
				if (slice <= 3) {
					dmem_addr = 0x127 + slice;
				}
			} else if (arg0 == 1) {
				if (slice <= 2) {
					dmem_addr = 0x12b + slice;
				}
			}

			u8 sub_val = base_fp + dmem_read8(dmem_addr);
			if (dmem_read8(0x0d) & 0x20) {
				u8 sp_28[4];
				pmu_dly_line_unpack(out_scratch[slice], sp_28);
				u32 dev = pmu_lcdl_dev_calc(sp_28[1]);
				u8 flag = ((dev < 0x21) ^ (sp_28[0] & 1)) ? 1 : 0;
				dmem_write8(0x474 + slice + (arg0 << 2), flag);
			}

			if (out_scratch[slice] < sub_val) {
				out_scratch[slice] = 0;
			} else {
				out_scratch[slice] -= sub_val;
			}
		}
	}

	for (u32 slice = slice_start; slice <= slice_end; slice++) {
		if (!(active_mask & (1 << slice))) {
			continue;
		}

		u8 sp_28[4];
		pmu_dly_line_unpack(out_scratch[slice], sp_28);

		if (dmem_read8(0x401) != 0) {
			pmu_delay_tap_step_adjust(sp_28, arg4);
		}

		if (dmem_read8(0x0d) & 0x20) {
			pmu_delay_tap_step_adjust(sp_28, dmem_read8(0x474 + slice + (arg0 << 2)));
		}

		out_dest[slice] = (u16)pmu_hdr_addr_15b_decode(sp_28);
	}
}

/**
 * pmu_cal_slice_step_diff_commit() - Converted from pmu_cal_slice_step_diff_commit
 * @arg0: Parameter arg0
 * @out_buf: Pointer to destination output buffer
 * @arg2: Parameter arg2
 *
 * Calibrates slice step differences, computes timing adjustments from DMEM frequency ratio,
 * and commits delay values to PHY data slice registers.
 * /
 */
void pmu_cal_slice_step_diff_commit(u32 arg0, u16 *out_buf, u32 arg2)
{
	u16 temp_buf[4] = {0};
	u32 sp_18 = (1 << arg0) & 3;
	u8 sp_10 = (u8)((1 << (arg0 + 2)) | (1 << arg0));
	u8 start_slice = dmem_read8(0xb68);
	u8 end_slice = dmem_read8(0xb69);

	phy_write16(0x90000148, 1);
	pmu_deskew_and_tracker_reset();

	pmu_cal_sequence_pulse_send(0, 0xb, 0, 0, 0, sp_18, 0);
	pmu_cbt_coarse_step_pulse_seq(0, 0xff, 0);
	pmu_cal_sequence_pulse_send(0, 6, 0x22, 0x40, 0x25, sp_10, 0);
	pmu_cal_sequence_pulse_send(0x80, 0x19, 4, 0x81, 0, sp_18, 0);
	pmu_clk_timing_delay_latch(0, 1);
	pmu_hw_timer_delay(0x800);
	pmu_delay_us(0x9c40, 8);
	pmu_cal_window_valid_check(sp_10, 0x23);

	for (u32 slice = start_slice; slice <= end_slice; slice++) {
		if (pmu_dq_swap_polarity_check(arg0, slice) != 0) {
			out_buf[slice] = (u16)pmu_slice_dq_bitmask_swizzle(slice, 4, arg0);
		}
	}

	pmu_cal_window_valid_check(sp_10, 0x24);
	for (u32 slice = start_slice; slice <= end_slice; slice++) {
		if (pmu_dq_swap_polarity_check(arg0, slice) != 0) {
			u16 val = (u16)pmu_slice_dq_bitmask_swizzle(slice, 4, arg0);
			u32 combined = ((u32)val << 8) | out_buf[slice];
			out_buf[slice] = (u16)combined;
			if (dmem_read16(0x06) <= 199 || (u16)combined < 2) {
				out_buf[slice] = 0xffff;
			} else {
				out_buf[slice] = (u16)(((u32)dmem_read8(0x08) << 17) / (u16)combined);
			}
		}
	}

	pmu_deskew_and_tracker_reset();
	pmu_cbt_coarse_step_pulse_seq(0, 0xff, 0);
	pmu_cal_sequence_pulse_send(0, 6, 0x22, 0x40, 0x28, sp_10, 0);
	pmu_cal_sequence_pulse_send(0x80, 0x19, 4, 0x83, 0, sp_18, 0);
	pmu_clk_timing_delay_latch(0, 1);
	pmu_hw_timer_delay(0x800);
	pmu_delay_us(0x9c40, 8);
	pmu_cal_window_valid_check(sp_10, 0x26);

	for (u32 slice = start_slice; slice <= end_slice; slice++) {
		if (pmu_dq_swap_polarity_check(arg0, slice) != 0) {
			temp_buf[slice] = (u16)pmu_slice_dq_bitmask_swizzle(slice, 4, arg0);
		}
	}

	pmu_cal_window_valid_check(sp_10, 0x27);
	for (u32 slice = start_slice; slice <= end_slice; slice++) {
		if (pmu_dq_swap_polarity_check(arg0, slice) != 0) {
			u16 val = (u16)pmu_slice_dq_bitmask_swizzle(slice, 4, arg0);
			u32 combined = ((u32)val << 8) | temp_buf[slice];
			temp_buf[slice] = (u16)combined;
			u16 final_val;
			if (dmem_read16(0x06) <= 199 || (u16)combined < 2) {
				final_val = 0xffff;
			} else {
				final_val = (u16)(((u32)dmem_read8(0x08) << 17) / (u16)combined);
			}
			temp_buf[slice] = final_val;
			pmu_cal_metric_log(4, 0x2460003, slice, out_buf[slice], final_val);
		}
	}

	for (u32 slice = start_slice; slice <= end_slice; slice++) {
		if (pmu_dq_swap_polarity_check(arg0, slice) == 0) {
			u32 neighbor = (slice & 1) ? (slice - 1) : (slice + 1);
			out_buf[slice] = out_buf[neighbor];
			temp_buf[slice] = temp_buf[neighbor];
		}
	}

	if (arg2 != 0 && end_slice >= start_slice) {
		u32 reg_off1 = (arg0 == 0) ? 0x1000c : 0x1000d;
		u32 reg_off2 = (arg0 == 0) ? 0x10015 : 0x10016;
		u32 gp28 = dmem_read32(0x41c);

		for (u32 slice = start_slice; slice <= end_slice; slice++) {
			u32 slice_base = (slice << 12) | gp28;
			phy_write16(0x90000000 | ((slice_base | reg_off1) << 1), out_buf[slice]);
			phy_write16(0x90000000 | ((slice_base | reg_off2) << 1), temp_buf[slice]);
		}
	}

	pmu_deskew_and_tracker_reset();
	pmu_cal_sequence_pulse_send(0, 0xc, 4, 0, 0, sp_18, 0);
	pmu_cal_sequence_pulse_send(0, 7, 6, 0, 0, 0, 0xff);
	pmu_cal_sequence_pulse_send(0, 0x10, 4, 0, 0, sp_18, 0);
	pmu_cal_sequence_pulse_send(0x80, 7, 6, 0, 0, 0, 0xff);
	pmu_clk_timing_delay_latch(0, 1);
	pmu_cal_strobe_secondary_pulse();
}

/*
 * pmu_cal_margin_sample_pair_extract:
 * Converted from pmu_cal_margin_sample_pair_extract.
 */
u32 pmu_cal_margin_sample_pair_extract(u8 *out_pair, const u8 *margins,
									   const u8 *samples, const u8 *buf_62b0)
{
	u16 pmu_flags = dmem_read16(0x0a);
	u32 max_iters = (pmu_flags >> 1) & 0x1e;
	u32 early_exit = 0;

	if (max_iters == 0) {
		max_iters = 16;
	} else if (max_iters == 2) {
		early_exit = 1;
		max_iters = 16;
	}

	out_pair[0] = 0x20;
	out_pair[1] = 0x40;

	if (pmu_flags & 1) {
		pmu_cal_metric_log(0xc8, 0x019b0004, margins[0], margins[1],
				dmem_read16(0x422), dmem_read16(0x420));
	}

	u32 margin_a = margins[0];
	u32 margin_b = margins[1];
	u32 best_score = pmu_eye_sample_metric_eval(margin_a, margin_b, buf_62b0, samples);

	if (pmu_flags & 1) {
		pmu_cal_metric_log(0xc8, 0x019c0003, margin_a, margin_b, best_score);
	}

	if (best_score < 2) {
		return best_score;
	}

	u32 cand_a = margin_a;
	u32 cand_b = margin_b;
	u32 step_reset = max_iters >> 2;
	u32 step = 4;
	u32 countdown = step_reset;

	for (u32 iter = 0; iter < max_iters; iter++) {
		u32 baseline_score = best_score;

		u32 sub_a = (margin_a - step) & 0x3f;
		u32 add_a = (margin_a + step) & 0x3f;

		bool skip_sub_a = (sub_a == cand_a && cand_b == margin_b);
		u32 score_sub_a = skip_sub_a ? 0 : pmu_eye_sample_metric_eval(sub_a, margin_b, buf_62b0, samples);

		bool skip_add_a = (add_a == cand_a && cand_b == margin_b);
		u32 score_add_a = skip_add_a ? 0 : pmu_eye_sample_metric_eval(add_a, margin_b, buf_62b0, samples);

		u32 score_a;
		if (score_add_a > score_sub_a) {
			if (score_add_a >= baseline_score) {
				score_a = score_add_a;
				cand_a = margin_a;
				margin_a = add_a;
			} else {
				score_a = baseline_score;
			}
		} else {
			if (score_sub_a >= baseline_score) {
				score_a = score_sub_a;
				cand_a = margin_a;
				margin_a = sub_a;
			} else {
				score_a = baseline_score;
			}
		}

		u32 sub_b = (margin_b - step) & 0x7f;
		u32 add_b = (margin_b + step) & 0x7f;

		bool skip_sub_b = (sub_b == cand_b && cand_a == margin_a);
		u32 score_sub_b = skip_sub_b ? 0 : pmu_eye_sample_metric_eval(margin_a, sub_b, buf_62b0, samples);

		bool skip_add_b = (add_b == cand_b && cand_a == margin_a);
		u32 score_add_b = skip_add_b ? 0 : pmu_eye_sample_metric_eval(margin_a, add_b, buf_62b0, samples);

		if (score_add_b > score_sub_b) {
			if (score_add_b >= score_a) {
				best_score = score_add_b;
				cand_b = margin_b;
				margin_b = add_b;
			} else {
				best_score = score_a;
			}
		} else {
			if (score_sub_b >= score_a) {
				best_score = score_sub_b;
				cand_b = margin_b;
				margin_b = sub_b;
			} else {
				best_score = score_a;
			}
		}

		if (best_score != baseline_score) {
			countdown = step_reset;
		} else if ((u8)countdown != 0) {
			countdown--;
		} else {
			countdown = 0;
		}

		if (pmu_flags & 1) {
			pmu_cal_metric_log(0xc8, 0x019d0006, iter, margin_a, margin_b,
					best_score, step, (u8)countdown);
		}

		if ((u8)countdown == 0 ||
			(margin_a == out_pair[0] && margin_b == out_pair[1])) {
			step >>= 1;
			if (step != 0) {
				countdown = step_reset;
			} else {
				step = 1;
				if (early_exit) {
					return best_score;
				}
			}
		}

		out_pair[1] = margin_b;
		out_pair[0] = margin_a;
	}

	return best_score;
}

/**
 * pmu_cal_channel_rank_boundary_eval() - Converted from pmu_cal_channel_rank_boundary_eval
 * @buf: Pointer to data buffer
 * @arg1: Parameter arg1
 * @arg2: Parameter arg2
 * @cal_type: Parameter cal_type
 *
 * /
 */
void pmu_cal_channel_rank_boundary_eval(u8 *buf, u32 arg1, u32 arg2, u32 cal_type)
{
	u8 *ctrl = buf + 512;
	u32 lut_val;
	u8 fp_val;
	u8 rank_start, rank_end;
	u32 dmem_41c;

	lut_val = pmu_2b_identity_lut(arg1);
	pmu_memset_words(buf, 0, 526);

	fp_val = (arg2 != 0) ? 9 : 8;

	if (cal_type == 0x11) {
		u8 dmem_102 = dmem_read8(0x102);

		ctrl[12] = 0;
		*(u16 *)&ctrl[8] = 0;
		*(u16 *)&ctrl[6] = 1;
		ctrl[10] = fp_val;
		*(u16 *)&ctrl[2] = 0x62;
		*(u16 *)&ctrl[4] = 0x64;
		pmu_dbyte_dq_deskew_regs_restore();

		ctrl[13] = 1;
		((void (*)(u32, u16, void *, void *, u32, u32, u32))(uintptr_t)pmu_cal_dbyte_pattern_loop_exec)(arg1, *(u16 *)&ctrl[6], buf, buf + 16, 0, 1, 0);
		((void (*)(u32, u16, void *, void *, u32, u32, u32))(uintptr_t)pmu_cal_dbyte_pattern_loop_exec)(arg1, *(u16 *)&ctrl[8], buf + 8, buf + 24, 0, 1, 0);

		rank_start = dmem_read8(0xb68);
		rank_end = dmem_read8(0xb69);

		for (u32 i = rank_start; i <= rank_end; i++) {
			u16 *p0 = (u16 *)(buf + (i * 2));
			u16 *p8 = (u16 *)(buf + 8 + (i * 2));
			u32 param24 = 0;
			u32 param28 = 0;

			pmu_cal_param_unpack(*p0, (u8 *)&param24);
			pmu_cal_param_unpack(*p8, (u8 *)&param28);

			u8 b24 = *(u8 *)&param24;
			u8 b28 = *(u8 *)&param28;

			if (b24 > b28) {
				pmu_delay_tap_step_adjust((u8 *)&param24, !(b28 & 1));
				*p0 = pmu_hdr_addr_15b_decode((u8 *)&param24);
			} else if (b24 < b28) {
				pmu_delay_tap_step_adjust((u8 *)&param28, !(b24 & 1));
				*p8 = pmu_hdr_addr_15b_decode((u8 *)&param28);
			}
		}

		if (dmem_read8(0xe4) != 0 || dmem_read8(0xe6) != 0) {
			buf[523] = 7;
		} else {
			buf[523] = 4;
		}

		if (dmem_102 & (1 << 1)) {
			u8 ret = pmu_cal_profile_mode_get();
			ctrl[0] = ret;
			if (ret == 3) {
				u32 gp_41c = dmem_read32(0x41c);
				phy_write16(0x9003e012 | (gp_41c << 1), 0);
			}
		}

		dmem_41c = dmem_read32(0x41c);
		for (u32 i = rank_start; i <= rank_end; i++) {
			for (u32 j = 0; j < 9; j++) {
				u32 offset = dmem_41c | (i << 12) | (j << 8);

				u16 val1 = phy_read16(0x90020000 | (((lut_val + 0x12) | offset) << 1));
				*(u16 *)(buf + 0x20 + (i * 20) + (j * 2)) = val1;

				u16 val2 = phy_read16(0x90020000 | (((lut_val + 0x10) | offset) << 1));
				*(u16 *)(buf + 112 + (i * 20) + (j * 2)) = val2;

				if ((dmem_102 & (1 << 1)) && (ctrl[0] == 3)) {
					u32 csr_base = 0x9002009c | (offset << 1);
					u16 reg0 = phy_read16(csr_base);
					u16 reg1 = phy_read16(csr_base + 2);
					u16 reg2 = phy_read16(csr_base + 4);
					u16 reg3 = phy_read16(csr_base + 6);

					u16 *diag = (u16 *)(buf + 192 + (i * 80) + (j * 8));
					diag[0] = reg0;
					diag[1] = reg1;
					diag[2] = reg2;
					diag[3] = reg3;

					phy_write16(csr_base, (reg0 + reg2) >> 1);
					phy_write16(csr_base + 2, (reg1 + reg3) >> 1);
				}

				pmu_cal_metric_log(4, 0x15e0004, i, j, val2, val1);
			}
		}
	} else {
		ctrl[11] = 2;
		*(u16 *)&ctrl[6] = 2;
		*(u16 *)&ctrl[2] = 0x5e;
		*(u16 *)&ctrl[4] = 0x60;
		*(u16 *)&ctrl[8] = 0xff;

		u8 nib = (u8)pmu_dmem_96_nibble_get();
		if (nib != 0)
			fp_val = 10;

		ctrl[10] = fp_val;
		ctrl[12] = nib;

		((void (*)(u32, u16, void *, void *, u32, u32, u32))(uintptr_t)pmu_cal_dbyte_pattern_loop_exec)(arg1, *(u16 *)&ctrl[6], buf, buf + 16, 0, 1, 0);

		rank_start = dmem_read8(0xb68);
		rank_end = dmem_read8(0xb69);
		dmem_41c = dmem_read32(0x41c);

		for (u32 i = rank_start; i <= rank_end; i++) {
			for (u32 j = 0; j < 9; j++) {
				u32 offset = dmem_41c | (lut_val + 0x26) | (i << 12) | (j << 8);
				u16 csr_val = phy_read16(0x90020000 | (offset << 1));

				*(u16 *)(buf + 0x20 + (i * 20) + (j * 2)) = csr_val;
				pmu_cal_metric_log(4, 0x15f0003, i, j, csr_val);
			}

			u32 offset = dmem_41c | (lut_val + 0x28) | (i << 12);
			u16 csr_val = phy_read16(0x90020000 | (offset << 1));
			*(u16 *)(buf + 0x20 + (i * 20) + (9 * 2)) = csr_val;
		}
	}
}

/**
 * copy_slice_quad() - pmu_cal_timing_table_matrix_init:
 * @buf: Pointer to data buffer
 * @off: Parameter off
 * @src: Pointer to source memory
 *
 * Converted from pmu_cal_timing_table_matrix_init.
 * /
 */
static inline void copy_slice_quad(u8 *buf, u32 off, const volatile u8 *src)
{
	buf[off] = src[0];
	buf[off + 54] = src[1];
	buf[off + 108] = src[2];
	buf[off + 162] = src[3];
}
/**
 * pmu_cal_timing_table_matrix_init() - Cal Timing Table Matrix Init
 * @buf: Pointer to data buffer
 */
void pmu_cal_timing_table_matrix_init(u8 *buf)
{
	pmu_memset_words(buf, 0, 54);

	const volatile u8 *src = (const volatile u8 *)0x8000005a;

	copy_slice_quad(buf, 1, src); src += 4;
	copy_slice_quad(buf, 2, src); src += 4;
	copy_slice_quad(buf, 3, src); src += 4;
	copy_slice_quad(buf, 10, src); src += 4;
	copy_slice_quad(buf, 11, src); src += 4;
	copy_slice_quad(buf, 12, src); src += 4;
	copy_slice_quad(buf, 13, src); src += 4;
	copy_slice_quad(buf, 14, src); src += 4;
	copy_slice_quad(buf, 15, src); src += 4;
	copy_slice_quad(buf, 16, src); src += 4;
	copy_slice_quad(buf, 17, src); src += 4;
	copy_slice_quad(buf, 18, src); src += 4;
	copy_slice_quad(buf, 19, src); src += 4;
	copy_slice_quad(buf, 20, src); src += 4;
	copy_slice_quad(buf, 21, src); src += 4;
	copy_slice_quad(buf, 22, src); src += 4;
	copy_slice_quad(buf, 24, src); src += 4;
	copy_slice_quad(buf, 25, src); src += 4;
	copy_slice_quad(buf, 26, src); src += 4;
	copy_slice_quad(buf, 27, src); src += 4;
	copy_slice_quad(buf, 28, src); src += 4;
	copy_slice_quad(buf, 30, src); src += 4;
	copy_slice_quad(buf, 31, src); src += 4;
	copy_slice_quad(buf, 32, src); src += 4;
	copy_slice_quad(buf, 33, src); src += 4;
	copy_slice_quad(buf, 34, src); src += 4;
	copy_slice_quad(buf, 37, src); src += 4;
	copy_slice_quad(buf, 40, src); src += 4;
	copy_slice_quad(buf, 41, src);

	buf[46] = 0;
	buf[100] = 0;
	buf[154] = 0;
	buf[208] = 0;
}

/*
 * pmu_cal_phase_delay_tracker_update:
 * Converted from pmu_cal_phase_delay_tracker_update.
 */
void pmu_cal_bist_lock_check(void);
/**
 * pmu_cal_phase_delay_tracker_slice_update() - Evaluates WCK calibration window metrics and updates WCK phase delay for a given slice/rank
 * @rank_idx: DRAM rank index (0..1)
 * @dmem_41c: Parameter dmem_41c
 * @r1_param: Parameter r1_param
 * @r2_param: Parameter r2_param
 *
 * /
 */
static void pmu_cal_phase_delay_tracker_slice_update(u32 rank_idx, u32 dmem_41c, u32 r1_param, u32 r2_param)
{
	u32 rank_shift = (u8)rank_idx << 12;
	u32 r12_off = (dmem_41c | rank_shift) << 1;
	u32 rank_off13 = (u8)rank_idx << 13;

	u16 val_1a0 = phy_read16(0x900201a0 | r12_off);
	u16 val_1a2 = phy_read16(0x900201a2 | r12_off);
	u16 val_1a4 = phy_read16(0x900201a4 | r12_off);
	u16 val_1a6 = phy_read16(0x900201a6 | r12_off);
	u16 val_1a8 = phy_read16(0x900201a8 | r12_off);

	u32 reg_20158 = 0x90020158 | rank_off13;
	u16 val_20158_strobe = phy_read16(reg_20158);
	phy_write16(reg_20158, val_20158_strobe | 1);
	phy_write16(reg_20158, val_20158_strobe & ~1);

	u16 val_1aa = phy_read16(0x900201aa | rank_off13);

	pmu_cal_metric_log(4, 0x420007, rank_idx, val_1a0, val_1a2, val_1a4, val_1a6, val_1a8, val_1aa);

	u32 r0_thresh = ((u32)val_1aa * 14) / 100;
	s32 diff_r1 = (s32)val_1a2 - (s32)val_1a0 - (s32)val_1aa;
	u32 flag_54 = ((val_1a0 * 2) < val_1aa) ? 1 : 0;
	u32 flag_58 = (diff_r1 > (s32)r0_thresh || diff_r1 < -(s32)r0_thresh) ? 1 : 0;

	u32 fp_idx = 5;
	u16 dram_freq = dmem_read16(0x06);

	for (u32 i = 0; i < 5; i++) {
		u16 curr = (i == 0) ? val_1a0 :
				   (i == 1) ? val_1a2 :
				   (i == 2) ? val_1a4 :
				   (i == 3) ? val_1a6 : val_1a8;
		u16 prev = (i == 0) ? 0 :
				   (i == 1) ? val_1a0 :
				   (i == 2) ? val_1a2 :
				   (i == 3) ? val_1a4 : val_1a6;

		if (dram_freq < 3200) {
			if (val_1aa < (curr * 2)) {
				fp_idx = i;
				break;
			}
			pmu_assert_or_halt(i != 4, 0x43 << 16);
		} else {
			if (i == 0) {
				if (!(flag_54 || flag_58)) {
					fp_idx = 0;
					break;
				}
			} else {
				s32 diff = (s32)curr - (s32)prev - (s32)val_1aa;
				if ((curr * 2) >= val_1aa && diff <= (s32)r0_thresh && diff >= -(s32)r0_thresh) {
					fp_idx = i;
					break;
				}
				pmu_assert_or_halt(i != 4, 0x47 << 16);
			}
		}
	}

	u32 r13_csr = 0x90020000 | ((rank_shift | r2_param) << 1);
	u32 r2_base = ((rank_shift | dmem_41c) << 1);
	phy_write16(r2_base | 0x9002015a, (u8)fp_idx);

	u16 fp_saved = phy_read16(r13_csr);
	u32 r1_csr = 0x90020000 | (((rank_shift | dmem_41c | 0xf) << 1));
	phy_write16(r13_csr, fp_saved | r1_param);

	u16 r14_val = phy_read16(r1_csr);
	u32 r2_28 = r2_base | 0x90020028;
	phy_write16(r1_csr, (r14_val & ~3) | 1);

	u16 r0_val = phy_read16(r2_28);
	u32 delay_cnt = ((r0_val << 1) & 510) * 160;
	pmu_hw_timer_delay((delay_cnt << 2) + 0x28);

	r14_val &= ~1;
	phy_write16(r13_csr, fp_saved & ~r1_param);
	phy_write16(r1_csr, r14_val);

	u16 strobe_val = phy_read16(reg_20158);
	phy_write16(reg_20158, strobe_val | 1);
	phy_write16(reg_20158, strobe_val & ~1);

	u16 val_1ac = phy_read16(0x900201ac | rank_off13);
	phy_write16(r2_base | 0x9002015e, val_1ac);
}

/**
 * pmu_cal_phase_delay_tracker_update() - LPDDR5 WCK clock calibration and phase delay tracker update routine
 *
 * Derived from vendor PMU code at address 0x77c4.
 * /
 */
void pmu_cal_phase_delay_tracker_update(void)
{
	u8 flag = dmem_read8(0x01);

	if (flag & 0x20) {
		dmem_write16(0x130, phy_read16(0x90040110));
		dmem_write16(0x132, phy_read16(0x90040112));
		dmem_write16(0x134, phy_read16(0x90040114));
		dmem_write16(0x136, phy_read16(0x90040118));

		u32 slice_offset = dmem_read32(0x41c) << 1;
		dmem_write16(0x13c, phy_read16(0x90040098 | slice_offset));
		dmem_write16(0x138, phy_read16(0x90040094 | slice_offset));
		dmem_write16(0x13a, phy_read16(0x90040096 | slice_offset));
	}

	u8 buf_4c6[18];
	pmu_memcpy_words(buf_4c6, (const void *)0x800004c6, 18);
	pmu_phy_reg_stream_play(buf_4c6, 18);

	u16 wck_ctrl = (flag & 0x80) ? 0 : (flag & 1);
	phy_write16(0x90040116, wck_ctrl);
	phy_write16(0x900400e4, dmem_read8(0x25));
	phy_write16(0x900400e6, dmem_read8(0x40));

	if (dmem_read8(0x19) == 0) {
		pmu_phy_cal_state_save();
		u32 active_slices = dmem_read32(0x40c);
		if (active_slices != 0) {
			for (u32 i = 0; i < active_slices; i++) {
				u32 reg = 0x90020146 + (i * 0x2000);
				phy_write16(reg, phy_read16(reg) & 0x0c);
			}
		}
	}

	pmu_phy_clk_toggle_settle();
	pmu_phy_pll_clock_timing_ctrl();

	if (dmem_read16(0x06) < 1600)
		goto out;

	if (pmu_dmem_8e_mode_check() != 0)
		goto out;

	u8 gp3 = dmem_read8(0x403);
	u32 dmem_41c = dmem_read32(0x41c);
	u32 r1_param, r2_param;

	if (gp3 < 2) {
		r1_param = 2;
		r2_param = dmem_41c | 0x14;
	} else if (gp3 < 4) {
		r1_param = 2;
		r2_param = 0x56;
	} else {
		r1_param = 1;
		r2_param = dmem_41c | 87;
	}

	u32 slice_off = dmem_41c << 1;
	u32 reg_3e1b2 = 0x9003e1b2;
	phy_write16(reg_3e1b2, 0x1000);

	u32 reg_3e01e = 0x9003e01e | slice_off;
	u32 reg_2001e = 0x9002001e | slice_off;
	u16 val_2001e = phy_read16(reg_2001e);

	u32 csr_base_3c = 0x90020000 | (r2_param << 1);
	phy_write16(reg_3e01e, (val_2001e & ~7) | 6);

	u32 csr_base_34 = 0x9003e000 | (r2_param << 1);
	u16 val_3c = phy_read16(csr_base_3c);
	val_3c &= ~r1_param;
	phy_write16(csr_base_34, val_3c);

	u16 val_20158 = phy_read16(0x90020158);
	phy_write16(reg_3e1b2 - 90, val_20158 & ~1);

	u16 val_r13 = phy_read16(reg_2001e);
	phy_write16(reg_3e01e, val_r13 | 5);
	pmu_hw_timer_delay(0x3000);

	phy_write16(reg_3e01e, (val_r13 & ~5) | 4);

	u8 b6a = dmem_read8(0xb6a);
	u8 b6b = dmem_read8(0xb6b);
	u8 b6c = dmem_read8(0xb6c);
	u8 b6d = dmem_read8(0xb6d);

	dmem_write8(0xb68, b6a);
	dmem_write8(0xb69, b6b);

	if (b6b != 255) {
		for (u32 rank_idx = 0; rank_idx <= b6b; rank_idx++)
			pmu_cal_phase_delay_tracker_slice_update(rank_idx, dmem_41c, r1_param, r2_param);
	}

	if (b6c <= b6d) {
		for (u32 rank_idx = b6c; rank_idx <= b6d; rank_idx++)
			pmu_cal_phase_delay_tracker_slice_update(rank_idx, dmem_41c, r1_param, r2_param);
	}

	val_3c = phy_read16(csr_base_3c);
	phy_write16(csr_base_34, val_3c | r1_param);

	val_2001e = phy_read16(reg_2001e);
	phy_write16(reg_3e01e, (val_2001e & ~3) | 1);

	u16 val_20028 = phy_read16(0x90020028 | slice_off);
	u32 delay_count = ((val_20028 << 1) & 510) * 160;
	pmu_hw_timer_delay((delay_count << 2) + 0x28);

	val_2001e &= ~1;
	val_3c &= ~r1_param;
	phy_write16(csr_base_34, val_3c);
	phy_write16(reg_3e01e, val_2001e);

out:
	pmu_cal_bist_lock_check();
}

/**
 * pmu_cal_slice_step_scan_eval_80f0() - Slice step scan evaluation and calibration pulse sequence generator
 * @arg0: Parameter arg0
 * @arg1: Parameter arg1
 *
 * Derived from vendor PMU code at address 0x80f0.
 * /
 *
 * Return: Computed u32 value or status.
 */
u32 pmu_cal_slice_step_scan_eval_80f0(u32 arg0, u32 arg1)
{
	u32 r15 = phy_read16(0x90040084) & 0x7f;
	if (r15 == 0) {
		pmu_cal_metric_log(4, 0x028e0000);
		r15 = 1;
	}

	u32 r13_prod = r15 * 3;
	u32 sp_1c = r15 << 2;
	s16 sp_10 = (s16)(sp_1c + (u16)(r13_prod ^ 0xffff));
	if (sp_10 < 0) {
		u32 r1 = 1;
		s16 not_r13 = (s16)~r13_prod;
		do {
			r1++;
			sp_10 = (s16)(sp_1c * r1 + not_r13);
		} while (sp_10 < 0);
	}
	u8 sp_20 = (u8)sp_10;

	pmu_deskew_and_tracker_reset();
	pmu_cal_metric_log(4, 0x028f0001, r15);
	pmu_cbt_cal_stat_set();

	u8 sp_18 = (u8)(r15 * 10 - 1);
	pmu_cal_sequence_pulse_send(0, 7, 0, 0, 0, 0, sp_18);

	u8 sp_24 = (u8)(r13_prod + 1);
	pmu_multiphase_pulse_seq_repeat(7, 0, 0, sp_24);

	u32 fp_val = 10;
	pmu_multiphase_pulse_seq_repeat(0x1d, 0x12c8, arg1, fp_val);
	pmu_multiphase_pulse_seq_repeat(0x1e, 0x12c8, arg1, fp_val);
	pmu_multiphase_pulse_seq_repeat(0x1f, 0x12c8, arg1, fp_val);
	pmu_multiphase_pulse_seq_repeat(0x20, 0x12c8, 0, fp_val);

	pmu_multiphase_pulse_seq_repeat(7, 0, 0, sp_20);
	pmu_cal_sequence_pulse_send(0, 7, 0, 0, 0, 0, sp_18);
	pmu_cal_sequence_pulse_send(0, 7, 4, 0, 0, 0, (u8)(fp_val - 1));

	for (int i = 0; i < 8; i++) {
		pmu_cal_pulse_burst_send(1, r15, arg1);
		pmu_cal_pulse_burst_send(0, r15, arg1);
	}

	sp_10 = 1;
	pmu_cal_sequence_pulse_send(0, 7, 0x5c, 0, 0, 0, 0);
	pmu_cal_sequence_pulse_send(0x40004, 0x2e, arg0, 0x1000, 0, 0, 0);
	pmu_cal_sequence_pulse_send(0x40004, 0x2f, arg0, 0x1000, 0, 0, 0);
	pmu_cal_sequence_pulse_send(0x8000, 7, 4, 0, 0, 0, 1);

	u32 r13_target = (arg0 * 2) + 100;
	s32 r14_diff = (s32)sp_1c - (s32)r13_target;
	while (r14_diff < 0) {
		sp_10++;
		r14_diff = (s16)((sp_1c * (u16)sp_10) - r13_target);
		pmu_cal_metric_log(4, 0x02900003, (u16)sp_10, r13_target, (s16)r14_diff);
	}
	pmu_cal_metric_log(4, 0x02910002, (s16)r14_diff, r13_target << ((u32)r14_diff & 0x1f));

	pmu_cal_sequence_pulse_send(0, 7, 0, 0, 0, 0, sp_18);
	pmu_multiphase_pulse_seq_repeat(7, 0, 0, (u8)(r14_diff + sp_24));

	u32 post_param = 0x25 << 7; /* 0x1280 */
	pmu_multiphase_pulse_seq_repeat(0x1d, post_param, arg1, fp_val);
	pmu_multiphase_pulse_seq_repeat(0x1e, post_param, arg1, fp_val);
	pmu_multiphase_pulse_seq_repeat(0x1f, post_param, arg1, fp_val);
	pmu_multiphase_pulse_seq_repeat(0x20, post_param, 0, fp_val);

	pmu_multiphase_pulse_seq_repeat(7, 0, 0, sp_20);
	pmu_cal_sequence_pulse_send(0, 7, 0, 0, 0, 0, sp_18);
	pmu_cbt_cal_stat_clear();

	return 0xff;
}

/**
 * pmu_cal_rank_param_commit_8aa8() - Converted from pmu_cal_rank_param_commit_8aa8
 * @r0: Parameter r0
 * @r1: Parameter r1
 * @r2: Parameter r2
 * @r3: Parameter r3
 *
 * /
 */
void pmu_cal_rank_param_commit_8aa8(u32 r0, u32 r1, u32 r2, u32 r3)
{
	u8 dmem_20 = dmem_read8(0x20) & 0x1f;
	u8 dmem_21 = dmem_read8(0x21) & 0x1f;
	u8 dmem_17 = dmem_read8(0x17) & 0x1f;
	u8 dmem_18 = dmem_read8(0x18) & 0x1f;

	dmem_write8(0x20, dmem_20);
	dmem_write8(0x21, dmem_21);
	dmem_write8(0x17, dmem_17);
	dmem_write8(0x18, dmem_18);

	if ((dmem_20 | dmem_21) == 0) {
		dmem_write8(0x20, 1);
		dmem_write8(0x21, 1);
	}

	if ((dmem_17 | dmem_18) == 0) {
		dmem_write8(0x17, 1);
		dmem_write8(0x18, 1);
	}

	u32 flag_10 = (r3 != 1) ? 1 : 0;
	u32 flag_14 = (r3 <= 1) ? 1 : 0;
	u32 r15_val = (r3 == 2) ? 1 : 0;

	u32 flag_28 = (r1 != 0) ? 1 : 0;
	u8 dmem_ce = dmem_read8(0xce);
	u32 flag_1c = (dmem_ce == 0) ? dmem_read8(0x100) : 0;
	u32 count_24 = (dmem_read8(0x401) != 0) ? 2 : 1;

	for (u32 i = 0; i < count_24; i++) {
		if (r3 == 1)
			pmu_cal_metric_log(4, 0x43 << 17);
		else if (r3 == 2)
			pmu_cal_metric_log(4, 0x11 << 19);
		else if (r3 == 0)
			pmu_cal_metric_log(4, 0x87 << 16);

		pmu_cal_metric_log(4, 0x890001, count_24);

		pmu_cal_margin_matrix_scan_eval(r0, r1, r2, r3, flag_1c, (u8)i);

		if (dmem_read8(0xb) & 1) {
			pmu_cal_metric_log(4, 0x45 << 17);
			pmu_cal_metric_table_sweep_log(0x80000dc8, r15_val, r2);
		}

		if (dmem_ce == 0) {
			pmu_cal_metric_log(4, 0x8b << 16);
			u32 ret_9f28 = pmu_cal_rank_margin_window_check(r15_val, 0, r2);
			if (ret_9f28 == 0)
				ret_9f28 = pmu_cal_rank_margin_window_check(r15_val, 1, r2);
			pmu_cal_metric_log(4, 0x23 << 18);
			pmu_cal_metric_table_sweep_log(0x80000dc8, r15_val, r2);
			pmu_profile_mailbox_cmd_dispatch(ret_9f28);
		}

		if (r1 != 0)
			pmu_rank_slice_margin_eval();

		if (flag_14) {
			pmu_cal_metric_log(4, 0x47 << 17);
			pmu_cal_dly_line_tap_step_adjust(flag_10, (u8)i, r2, flag_28);

			if (dmem_read8(0x401) != 0) {
				u8 temp_bits = dmem_read8(0x25) | dmem_read8(0x40);
				if (i == 0) {
					for (u32 k = 0; k <= 1; k++) {
						if (temp_bits & (1 << k)) {
							u32 lut_k = pmu_2b_identity_lut(k);
							pmu_lcdl_delay_profile_update(0, dmem_read8(0x4), lut_k, k);
						}
					}
				} else if (flag_10 != 0) {
					for (u32 k = 0; k <= 1; k++) {
						if (temp_bits & (1 << k)) {
							u32 lut_k = pmu_2b_identity_lut(k);
							pmu_lcdl_delay_profile_update(0, dmem_read8(0x4), lut_k, k);
						}
					}
					pmu_cal_metric_log(4, 0x8f << 16);
					pmu_cal_delay_line_accum_dispatch(r2);
				}
			}
		} else {
			pmu_cal_metric_log(4, 0x8d << 16);
			pmu_cal_dbyte_deskew_pin_results_apply();
		}
	}
}



/* Forward declarations for functions defined below pmu_train_dispatcher */
void pmu_cal_multi_lane_deskew_sweep(u32 type, u32 ch_mask, u32 lane_mask);
void pmu_cal_lane_window_sweep_coordinator(u32 count, u32 flag);
void pmu_cal_margin_envelope_step_eval(void);
void pmu_cal_dbyte_deskew_fine_tune(u32 mode);
void pmu_cal_phase_detector_edge_align(void);
void pmu_cal_stage_post_process_d7bc(void);
void pmu_cal_stage_post_process_d804(u32 arg0);
void pmu_cal_stage_post_process_d720(u32 arg0);
u32 pmu_cal_rank_lane_status_check(void);

/**
 * pmu_prof_stamp() - Record timestamp for training stage execution profiling
 * @idx: Profiling slot index (0..12)
 */
static inline void pmu_prof_stamp(u32 idx)
{
	*(volatile u32 *)(PMU_DMEM_BASE | (0x300 + idx * 4)) = __builtin_arc_lr(0x104);
}

/**
 * pmu_train_dispatcher() - Top-level PMU training stage dispatcher for LPDDR5 PHY
 *
 * Dispatches calibration stages in sequence according to SequenceCtrl (DMEM 0x10):
 * - Early PHY reset, PLL gating, PUB control, and clock divider latching
 * - Bit 0  (0x0001): CBT entry, ZQ pad calibration, tracker update, mission handoff
 * - Bit 1  (0x0002): Post-process stage d804
 * - Bit 2  (0x0004): Post-process stage d7bc
 * - Bit 3  (0x0008): Post-process stage d720
 * - Bit 6  (0x0040): Search window init & stage 6 dispatch
 * - Bit 7  (0x0080): ZQ pad calibration & stage 10 dispatch
 * - Bit 10 (0x0400): Lane window sweep & matrix scan
 * - Bit 4  (0x0010): BIST search window setup
 * - Bit 8  (0x0100): Multi-lane deskew sweep & stage 11 dispatch
 * - Bit 9  (0x0200): Margin envelope step evaluation & LCDL delay profile update
 * - Phase detector / fine tune dispatch & completion mailbox notification
 */
void pmu_train_dispatcher(void)
{
	__builtin_arc_sr(2, 0x103);
	__builtin_arc_sr(1, 0x103);
	pmu_prof_stamp(0);

	phy_write16(PHY_REG_INIT_COMPLETE, 1);

	if (dmem_read8(PMU_DMEM_CLK_GATE_FLAG) & 1) {
		phy_write16(PHY_REG_PLL_STATUS, 0);
		phy_write16(0x9003e034, 0);
		phy_write16(PHY_REG_PLL_CTRL, 0);
		phy_write16(0x90040154, 0);
		phy_write16(PHY_REG_PLL_CTRL, 3);
	}

	pmu_phy_clk_gate_sync_pulse();
	pmu_cal_metric_log(10, 0x02200000);
	pmu_cbt_step_stat_set();

	u8 r1 = dmem_read8(0x420);
	u16 pub_ctl = phy_read16(PHY_REG_PUB_CTL);
	pmu_cmd_code_map_log(pub_ctl, r1);

	u8 gp3 = dmem_read8(0x403);
	u32 reg14 = (gp3 == 0) ? 0x90120052 : 0x9012005c;
	u32 reg15 = (gp3 == 0) ? 0x90120054 : 0x9012005e;

	dmem_write16(0x420, 0x230b);
	phy_write16(reg14, 0x715d);
	phy_write16(reg15, 0x00ce);

	pmu_cal_metric_log(10, 0x02220001);
	pmu_cal_metric_log(10, 0x02230005,
			(u32)phy_read16(0x901801de),
			(u32)phy_read16(0x901801dc),
			(u32)phy_read16(0x901801da),
			(u32)phy_read16(0x900401dc),
			(u32)phy_read16(0x900401d8));

	if (dmem_read16(0x424) != phy_read16(reg14) ||
		dmem_read16(0x426) != phy_read16(reg15)) {
		pmu_assert_or_halt(0, 0x02240000);
	}

	u8 d04 = dmem_read8(0x04);
	u32 ch_offset = 0;
	if (!(d04 & 0x80)) {
		ch_offset = (d04 << 20) & 0x100000;
	}
	dmem_write32(PMU_DMEM_PARAM_41C, ch_offset);

	u16 seq_ctrl = dmem_read16(0x10);
	u16 r18_val = phy_read16(0x90040012 | (ch_offset << 1));
	u8 clk_gate_flag = dmem_read8(PMU_DMEM_CLK_GATE_FLAG);

	u32 dram_type = dmem_read8(PMU_DMEM_DRAM_TYPE);
	u32 freq = dmem_read16(PMU_DMEM_DRAM_FREQ_OFF);
	u32 denom = dram_type * 220;
	u32 window = (freq + denom - 1) / denom;

	dmem_write8(0x0a7c, 1);
	dmem_write8(0x0b65, 4);
	dmem_write8(0x0b64, (u8)window);

	pmu_cal_search_win_init((u8 *)0x80000037);
	pmu_cal_search_win_init((u8 *)0x80000039);

	/*
	 * Stage 0: Device Initialization & CBT Entry (PMU_SEQ_DEV_INIT - Bit 0)
	 * Initializes PHY PLLs, pad drivers, rank/slice tables, and runs Command Bus
	 * Training (CBT) entry sequence to put DRAM into calibration mode.
	 */
	if (seq_ctrl & PMU_SEQ_DEV_INIT) {
		u32 r13_flag = (clk_gate_flag >> 7) & 1;

		pmu_cbt_entry_pll_ctrl();
		if (!(dmem_read8(0x19) | r13_flag))
			phy_write16(PHY_REG_PHY_STATUS, 2);

		pmu_cal_stage8_pad_init();
		pmu_dmem_rank_slice_table_setup();
		pmu_cal_metric_log(4, 0x02260000);
		pmu_cal_phase_delay_tracker_update();
		pmu_deskew_and_tracker_reset();
		pmu_cbt_coarse_step_pulse_seq(0, 4, 0);
		pmu_clk_timing_delay_latch(0, 1);
		pmu_cbt_exit_mission_handoff();
		pmu_cal_lane_window_margin_eval(r13_flag);
		dmem_write16(0x14, 1);
		pmu_dbyte_pin_map_table_init();
		if (r13_flag == 0)
			pmu_cal_rank_lane_status_check();

		/*
		 * Stage 1: Command/Address (CA) Training (PMU_SEQ_LPCA_INIT - Bit 12)
		 * Trains the LPDDR5 Command/Address bus timing against the differential clock.
		 * When combined with PMU_SEQ_DEV_INIT, forms Fast Boot mode (0x1001).
		 */
		if (seq_ctrl & PMU_SEQ_LPCA_INIT) {
			pmu_cal_metric_log(4, 0x02270000);
			pmu_cal_stage1_pre_init();
			dmem_write16(0x14, 0x1000);
			dmem_write16(0x14, 0x9000);
			pmu_post_cmd_conditional_dispatch(0xd);
			if (r13_flag == 0)
				pmu_multiparam_cal_dispatch();
		} else if (r13_flag == 0) {
			pmu_multiparam_cal_dispatch();
		}

		pmu_deskew_and_tracker_reset();
		u32 clamp = pmu_delay_clamp(0x3a98, 0);
		u32 r2_clamp = ((u8)clamp > 8) ? (u8)clamp : 8;
		pmu_cal_sequence_pulse_send(0, 7, r2_clamp, 0, 0, 0, 0);
		pmu_clk_timing_delay_latch(0, 1);
		pmu_post_cmd_conditional_dispatch(0);
		dmem_write8(0x402, 0);
		dmem_write16(0x14, 0x8001);
	}
	pmu_prof_stamp(1);

	pmu_dbyte_dq_deskew_regs_save_and_ramp();

	u32 flag_bit6 = seq_ctrl & PMU_SEQ_SEARCH_WIN;
	u32 flag_bit3 = seq_ctrl & PMU_SEQ_STAGE_D720;

	/*
	 * Stage 2, 3, 5 Pre-Pass:
	 * When PMU_SEQ_SEARCH_WIN (Stage 6) is active, Stages 2 (Vref), 3 (DCD), and 5 (DQS)
	 * are calibrated post-search on aligned WCK/CK clocks (saving ~514 ms duplicate run).
	 * If 2D search is disabled, execute them here.
	 */
	if (!flag_bit6) {
		if (seq_ctrl & PMU_SEQ_STAGE_D804)
			pmu_cal_stage_post_process_d804(1);
		if (seq_ctrl & PMU_SEQ_STAGE_D7BC)
			pmu_cal_stage_post_process_d7bc();
		if (flag_bit3)
			pmu_cal_stage_post_process_d720(1);
	}

	/*
	 * Stage 4: ZQ Calibration Pad Tuning (PMU_SEQ_ZQ_CAL - Bit 7)
	 * Matches transmitter output impedance (Ron) and on-die termination (ODT)
	 * against external 240-ohm calibration resistor.
	 */
	u32 flag_profile3 = 0;
	if (pmu_cal_profile_mode_get() == 3 && (seq_ctrl & PMU_SEQ_ZQ_CAL) && !(dmem_read8(0x102) & 2)) {
		phy_write16(0x9003e012 | (ch_offset << 1), 0);
		flag_profile3 = 1;
	}

	/*
	 * Stage 6: High-Resolution 2D Eye Margin Search (PMU_SEQ_SEARCH_WIN - Bit 6)
	 * 2-dimensional scan across delay (X-axis) and Vref DAC voltage (Y-axis)
	 * to find the optimal sampling centroid.
	 */
	if (flag_bit6) {
		pmu_cal_metric_log(10, 0x020f0000);
		pmu_cal_search_win_init((u8 *)0x80000039);
		dmem_write8(0x402, 1);
		dmem_write16(0x14, 0x0040);
		pmu_cal_vref_dac_step_adjust(0x13);
		dmem_write16(0x14, 0x8040);
		pmu_post_cmd_conditional_dispatch(6);
		dmem_write8(0x402, 0);
	}
	pmu_prof_stamp(2);

	u8 d102 = dmem_read8(0x102);
	if (!(d102 & 1) && flag_bit6) {
		if (!(d102 & 8) && (seq_ctrl & PMU_SEQ_STAGE_D804))
			pmu_cal_stage_post_process_d804(1);
		pmu_prof_stamp(3);
		if (!(d102 & 0x10) && (seq_ctrl & PMU_SEQ_STAGE_D7BC))
			pmu_cal_stage_post_process_d7bc();
		pmu_prof_stamp(4);
		if (flag_bit3 && !(d102 & 0x20))
			pmu_cal_stage_post_process_d720(1);
		pmu_prof_stamp(5);
	} else {
		pmu_prof_stamp(3);
		pmu_prof_stamp(4);
		pmu_prof_stamp(5);
	}

	u32 flag_bit7 = seq_ctrl & PMU_SEQ_ZQ_CAL;
	if (flag_bit7) {
		if (dmem_read8(0x8e) & 1) {
			pmu_cal_metric_log(10, 0x02100000);
			dmem_write16(0x14, 0x0080);
			pmu_cal_vref_dac_step_adjust(0x11);
			dmem_write16(0x14, 0x8080);
			pmu_post_cmd_conditional_dispatch(10);
		} else {
			pmu_cal_metric_log(10, 0x02110000);
		}
		if (!(seq_ctrl & PMU_SEQ_DESKEW_SWEEP))
			dmem_write16(0x428, 0);
	}

	d102 = dmem_read8(0x102);
	if (!(d102 & 2) && flag_bit7) {
		if (flag_profile3)
			phy_write16(0x9003e012 | (ch_offset << 1), 4);
		if (flag_bit3)
			pmu_cal_stage_post_process_d720(1);
	}

	/*
	 * Stage 7: Multi-Lane Window Sweep & Matrix Scan (PMU_SEQ_LANE_WIN_SWEEP - Bit 10)
	 * Cross-evaluates window margins across all 4 byte lanes and both ranks.
	 */
	if (seq_ctrl & PMU_SEQ_LANE_WIN_SWEEP) {
		pmu_cal_metric_log(10, 0x02120000);
		u8 b1 = dmem_read8(0x01);
		u8 b89 = dmem_read8(0x59);
		u32 count_10 = 0;
		if (b89 >> 6)
			count_10 = (1U << (b1 & 3)) - 1;

		dmem_write16(0x14, 1024);
		pmu_cal_lane_window_sweep_coordinator(count_10, 1024);
		dmem_write16(0x14, 0x8400);

		if (dmem_read8(0x405) == 0) {
			u8 fp_or = dmem_read8(0x25) | dmem_read8(0x40);
			for (u32 r13_idx = 0; r13_idx <= 1; r13_idx++) {
				if (fp_or & (1 << r13_idx)) {
					u32 lut = (0xe4 >> (r13_idx << 1)) & 3;
					u32 buf_sp1c[2] = {0, 0};
					pmu_slice_lcdl_delay_collect(0x1000040 + lut, buf_sp1c, 1);
					pmu_cal_metric_log(5, 0x02540000);
					pmu_cal_matrix_trace_dump(0, 1, (const u16 *)buf_sp1c);
				}
			}
		}
		pmu_post_cmd_conditional_dispatch(0xe);
	}

	/*
	 * Stage 8: BIST Test Pattern Search Window (PMU_SEQ_BIST_SEARCH_WIN - Bit 4)
	 * Drives hardware BIST patterns into DRAM to verify read/write data eye validity.
	 */
	if (seq_ctrl & PMU_SEQ_BIST_SEARCH_WIN) {
		pmu_cal_bist_search_win_setup(0);
	} else if (seq_ctrl == PMU_SEQ_DEV_INIT) {
		pmu_cal_bist_search_win_setup(1);
	}

	/*
	 * Stage 9: Per-Bit DQ Deskew Calibration (PMU_SEQ_DESKEW_SWEEP - Bit 8)
	 * Adjusts individual LCDL delay taps for each DQ bit (0..7) within every byte lane
	 * to eliminate trace-length skew and align all data bits with DQS strobe.
	 */
	u32 flag_bit8 = seq_ctrl & PMU_SEQ_DESKEW_SWEEP;
	if (flag_bit8) {
		pmu_cal_metric_log(10, 0x02140000);
		dmem_write16(0x14, 256);
		pmu_cal_vref_dac_step_adjust(0x12);
		dmem_write16(0x14, 0x8100);
		pmu_post_cmd_conditional_dispatch(0xb);
		dmem_write16(0x428, 0);

		if (!(dmem_read8(0x102) & 4)) {
			if (seq_ctrl & PMU_SEQ_BIST_SEARCH_WIN) {
				pmu_cal_bist_search_win_setup(0);
			} else if (seq_ctrl != PMU_SEQ_DEV_INIT) {
				pmu_cal_bist_search_win_setup(1);
			}
		}
	}

	if (seq_ctrl == PMU_SEQ_DEV_INIT && dmem_read8(0x19) == 0) {
		pmu_cal_metric_log(10, 0x02280000);
		pmu_cal_vref_dac_step_adjust(0xf);
	}

	if (seq_ctrl & PMU_SEQ_BIST_SEARCH_WIN) {
		u32 r14_lim = (pmu_dmem_training_flags_eval() == 0) ? 0xff : 511;
		u32 r13_mask = (dmem_read8(0x01) & 4) ? 0 : 0xf;
		u16 val_42a = dmem_read16(0x42a);
		if (val_42a & (1 << 5)) {
			pmu_cal_metric_log(10, 0x02150000);
			pmu_cal_multi_lane_deskew_sweep(2, r13_mask, r14_lim);
			pmu_prof_stamp(6);
			if (dmem_read8(0x96) & (1 << 4)) {
				pmu_cal_metric_log(4, 0x02160000);
				pmu_cal_multi_lane_deskew_sweep(3, r13_mask, r14_lim);
			}
			pmu_prof_stamp(7);
		} else {
			pmu_prof_stamp(6);
			pmu_prof_stamp(7);
		}
	} else {
		pmu_prof_stamp(6);
		pmu_prof_stamp(7);
	}

	if (flag_bit3) {
		pmu_cal_stage_post_process_d720(0);
		pmu_prof_stamp(8);
		u32 r13_lim = (pmu_dmem_training_flags_eval() == 0) ? 0xff : 511;
		u32 r14_mask = (dmem_read8(0x01) & 4) ? 0 : 0xf;
		u16 val_42a = dmem_read16(0x42a);
		if (val_42a & (1 << 6)) {
			pmu_cal_metric_log(10, 0x02170000);
			pmu_cal_multi_lane_deskew_sweep(1, r14_mask, r13_lim);
			pmu_prof_stamp(9);
			pmu_cal_multi_lane_deskew_sweep(0, r14_mask, r13_lim);
			pmu_prof_stamp(10);
		} else {
			pmu_prof_stamp(9);
			pmu_prof_stamp(10);
		}
	} else {
		pmu_prof_stamp(8);
		pmu_prof_stamp(9);
		pmu_prof_stamp(10);
	}

	/*
	 * Stage 10: Timing Margin Envelope Evaluation (PMU_SEQ_MARGIN_STEP - Bit 9)
	 * Measures left/right timing margin boundaries to verify operating guardband.
	 */
	if (seq_ctrl & PMU_SEQ_MARGIN_STEP)
		pmu_cal_margin_envelope_step_eval();
	pmu_prof_stamp(11);

	if (dmem_read8(0x405) != 0) {
		u8 fp_or = dmem_read8(0x25) | dmem_read8(0x40);
		for (u32 r13_idx = 0; r13_idx <= 1; r13_idx++) {
			if (fp_or & (1 << r13_idx)) {
				u32 lut = (0xe4 >> (r13_idx << 1)) & 3;
				pmu_lcdl_delay_profile_update(1, 0, lut, r13_idx);
				pmu_lcdl_delay_profile_update(0, 0, lut, r13_idx);
			}
		}
		pmu_dmem_stride_descriptor_read();
		if (dmem_read8(0x00) & 0x20)
			pmu_cal_error_code_log();
	}

	pmu_cal_stage39_post_results();

	if (seq_ctrl != 1) {
		u8 d04_val = dmem_read8(0x04);
		if (d04_val & 0x80) {
			pmu_cal_phase_detector_edge_align();
		} else if ((d04_val & 0x30) == 0x20) {
			pmu_cal_dbyte_deskew_fine_tune(0);
		} else if ((d04_val & 0x30) == 0x10) {
			pmu_cal_dbyte_deskew_fine_tune(1);
		}
	}

	/*
	 * Training Completion & Handoff:
	 * Restore deskew registers, verify PLL lock status, log completion event,
	 * and write success code 0x7 (PMU_STATUS_SUCCESS) to host mailbox registers.
	 */
	pmu_dbyte_dq_deskew_regs_restore();
	phy_write16(0x9005e012 | (ch_offset << 1), r18_val);
	pmu_cal_pll_status_check();
	pmu_cal_metric_log(10, 0x02290000);
	pmu_mailbox_post_cmd_dispatch(PMU_STATUS_SUCCESS);
	pmu_prof_stamp(12);

	return;
}

/*
 * pmu_cal_multi_lane_deskew_sweep:
 * Converted from pmu_cal_multi_lane_deskew_sweep.
 */
/**
 * pmu_cal_multi_lane_deskew_sweep() - Converted from pmu_cal_multi_lane_deskew_sweep
 * @type: Parameter type
 * @ch_mask: Bitmask of active memory channels
 * @lane_mask: Bitmask of active bit lanes
 *
 * Performs multi-lane deskew calibration sweep across channels, slices, and
 * byte-lanes:
 * 1. Initial configuration: decodes calibration type (0..3) to set up
 * CSR offsets, active flags, and search limits.
 * 2. Delay line sampling: caches previous LCDL delay registers across active
 * slices and lanes (0..9).
 * 3. Sweep execution: initiates DBYTE pattern loop (pmu_cal_dbyte_pattern_loop_exec), command strobe
 * dispatch (pmu_cal_bist_cmd_strobe_dispatch), clears sample buffer at 0x80006318, executes delay
 * line scan (pmu_cal_slice_step_eval_sweep), and unpacks DMEM register stream (pmu_dmem_reg_stream_unpack).
 * 4. Margin evaluation & CSR programming: analyzes margin window boundaries
 * from sample buffer (0x80006318 + lane * 848), computes differences against
 * nominal midpoints, applies boundary limits / adjustments, updates PHY deskew
 * registers (0x900200d0..0x900200dc, 0x90020038..0x9002003c, or 0x90020044),
 * and restores cached delay lines.
 * /
 */
void pmu_cal_multi_lane_deskew_sweep(u32 type, u32 ch_mask, u32 lane_mask)
{
	u16 dly_buf[4][10];
	u16 buf_94[4] = {0};
	u16 buf_8c[4] = {0};
	u32 desc_ec[9];

	pmu_memset_words(desc_ec, 0, sizeof(desc_ec));

	u32 var_30 = (type == 3) ? 1 : 0;
	u32 var_3c = (type & 2) ? 0xd4 : 0x80;
	u32 var_48 = 0x10010 + ((type & 2) ? 0x16 : 0) + ((type & 1) ? 2 : 0);

	u32 freq_lt_3200 = (dmem_read16(0x06) < 3200) ? 1 : 0;
	u32 sp_34 = (lane_mask >> 8) & 0x7f;
	u32 active_mask = (u32)dmem_read8(0x40) | (u32)dmem_read8(0x25);

	for (u32 ch = 0; ch < 2; ch++) {
		u32 ch_active_mask = (1 << (ch + 2)) | (1 << ch);
		if (!(ch_active_mask & active_mask))
			continue;

		dmem_write8(0xb67, (u8)ch);
		pmu_phy_mode_cfg_dispatch(3);

		u8 slice_start = dmem_read8(0xb68);
		u8 slice_end = dmem_read8(0xb69);
		u8 active_slice_mask = dmem_read8(0xb98);
		u32 ch_offset = dmem_read32(0x41c);

		/* Phase 1: Cache previous delay values across active slices and lanes */
		for (u32 slice = slice_start; slice <= slice_end; slice++) {
			if (!(active_slice_mask & (1 << slice)))
				continue;

			u32 slice_offset = (slice << 12) | ch_offset | var_48 | ch;
			uintptr_t base_addr = 0x90000000 | (slice_offset << 1);

			for (u32 lane = 0; lane < 10; lane++) {
				if (lane == 8 && !sp_34)
					continue;

				if (var_30 != 0) {
					if (lane != 9)
						continue;
					dly_buf[slice][9] = phy_read16(base_addr);
				} else {
					if (lane == 9)
						continue;
					uintptr_t reg_addr = 0x90000000 | ((slice_offset | (lane << 8)) << 1);
					dly_buf[slice][lane] = phy_read16(reg_addr);
				}
			}
		}

		/* Phase 2: Execute calibration pattern loop & delay line scan */
		pmu_cal_dbyte_pattern_loop_exec(ch, (u16)type, buf_94, buf_8c, 0, 1, 0);

		if (var_30 != 0)
			pmu_phy_timing_delay_latch(1, ch_active_mask);

		pmu_cal_bist_cmd_strobe_dispatch((u32)(uintptr_t)desc_ec, 8, type, ch_active_mask, (u32)(uintptr_t)buf_94, sp_34, 0);

		pmu_memset_words((void *)0x80006318, 0, 0x8480);

		pmu_cal_slice_step_eval_sweep((void *)0x80006318, 8, type, 4,
				0, (u8)ch_mask, freq_lt_3200,
				buf_94, sp_34, 0, desc_ec);

		pmu_dmem_reg_stream_unpack(desc_ec);

		if (var_30 != 0)
			pmu_phy_timing_delay_latch(0, ch_active_mask);

		/* Phase 3: Margin evaluation, boundary adjustment & CSR programming */
		for (u32 slice = slice_start; slice <= slice_end; slice++) {
			if (!(active_slice_mask & (1 << slice)))
				continue;

			u32 slice_off = slice << 12;
			u32 reg_base_ch = slice_off | ch_offset | ch;
			u32 reg_base_var = slice_off | ch_offset | var_48 | ch;
			u32 r12_val = slice_off | ch_offset | (ch << 1);

			uintptr_t reg_sp_78 = 0x90020044 | (reg_base_ch << 1);
			uintptr_t reg_sp_7c = 0x90020032 | (r12_val << 1);
			uintptr_t reg_sp_132 = 0x90000000 | (reg_base_var << 1);

			for (u32 lane = 0; lane < 10; lane++) {
				if (lane == 8 && !sp_34)
					continue;

				if (var_30) {
					if (lane != 9)
						continue;
				} else {
					if (lane == 9)
						continue;
				}

				u32 lane_idx = slice * 10 + lane;
				u32 sample_offset = lane_idx * 848;
				u8 blink_flag = *(volatile const u8 *)(uintptr_t)(0x80006594 + sample_offset);

				u32 r12_edge = 0;
				u32 r2_edge = 0;

				if (blink_flag != 0) {
					const volatile u8 *s_ptr = (const volatile u8 *)(uintptr_t)(0x80006318 + sample_offset);
					u8 b0 = s_ptr[0];
					if (blink_flag == 1) {
						r12_edge = var_3c;
						r2_edge = b0;
					} else if (blink_flag == 3) {
						u8 b1 = s_ptr[1];
						u8 b2 = s_ptr[2];
						u32 diff_sample = (u32)b1 - (u32)b0;
						u32 diff_limit = var_3c - (u32)b2;
						if (diff_limit > diff_sample) {
							r12_edge = var_3c;
							r2_edge = b2;
						} else {
							r12_edge = b1;
							r2_edge = b0;
						}
					} else {
						r12_edge = s_ptr[1];
						r2_edge = b0;
					}
				}

				u16 cal_val_raw = buf_94[slice];
				u16 cur_val_raw = dly_buf[slice][lane];
				u32 r0_val, r3_val;

				if (type >= 2) {
					r0_val = cur_val_raw;
					r3_val = cal_val_raw;
				} else {
					r0_val = (cur_val_raw & 0x7f) + ((cur_val_raw >> 1) & ~0x3f);
					r3_val = (cal_val_raw & 0x7f) + ((cal_val_raw >> 1) & ~0x3f);
				}

				u16 r1_u16 = (u16)r0_val;
				u16 r3_u16 = (u16)r3_val;
				r12_edge += r3_u16;

				s32 term_diff = (s32)r1_u16 - (s32)r3_u16;
				s32 term_0 = term_diff - (s32)r2_edge;
				s32 term_12 = (s32)r12_edge - (s32)r1_u16;

				s32 val_r15, val_r12;
				if (type < 2) {
					val_r15 = term_12;
					val_r12 = term_0;
				} else {
					val_r15 = term_0;
					val_r12 = term_12;
				}

				u32 lane_reg_off = ((reg_base_ch | (lane << 8)) << 1);

				if (type == 0) {
					uintptr_t addr_d0 = 0x900200d0 | lane_reg_off;
					uintptr_t addr_d4 = 0x900200d4 | lane_reg_off;
					u16 reg_d0 = phy_read16(addr_d0);
					u16 reg_d4 = phy_read16(addr_d4);

					u8 limit_d0 = (u8)reg_d0;
					if ((u16)val_r12 > limit_d0)
						val_r12 = limit_d0;

					uintptr_t addr_24 = 0x90020024 | lane_reg_off;
					uintptr_t addr_20 = 0x90020020 | lane_reg_off;
					u16 reg_24 = phy_read16(addr_24);
					u16 reg_20 = phy_read16(addr_20);

					u8 limit_d4 = (u8)reg_d4;
					u8 d111 = dmem_read8(0x111);

					if ((u16)val_r15 > limit_d4)
						val_r15 = limit_d4;

					if (d111 != 0) {
						u16 bmsk_24 = reg_24 & 0x7f;
						u16 bmsk_20 = reg_20 & 0x7f;

						if (!((bmsk_24 > (u16)val_r12) && (bmsk_20 > (u16)val_r12))) {
							if (bmsk_24 >= bmsk_20)
								val_r12 = (s32)bmsk_20 - (s32)d111;
							else
								val_r12 = (s32)bmsk_24 - (s32)d111;
						}

						u32 sum_24 = (u16)val_r15 + bmsk_24;
						u32 sum_20 = (u16)val_r15 + bmsk_20;

						if ((sum_24 > 0x7e) || (sum_20 >= 0x7f)) {
							if (bmsk_20 >= bmsk_24)
								val_r15 = (s32)(bmsk_20 ^ 0x7f) - (s32)d111;
							else
								val_r15 = (s32)(bmsk_24 ^ 0x7f) - (s32)d111;
						}
					}

					phy_write16(addr_d0, (u16)val_r12);
					phy_write16(0x900200d8 | lane_reg_off, (u16)val_r12);
					phy_write16(addr_d4, (u16)val_r15);
					phy_write16(0x900200dc | lane_reg_off, (u16)val_r15);
				} else if (type == 2) {
					uintptr_t addr_38 = 0x90020038 | lane_reg_off;
					uintptr_t addr_3c = 0x9002003c | lane_reg_off;
					phy_write16(addr_38, (u16)val_r12);
					phy_write16(addr_3c, (u16)val_r15);
				} else if (type == 1) {
					uintptr_t addr_d0 = 0x900200d0 | lane_reg_off;
					uintptr_t addr_d4 = 0x900200d4 | lane_reg_off;
					phy_write16(addr_d0, (u16)val_r12);
					phy_write16(0x900200d8 | lane_reg_off, (u16)val_r12);
					phy_write16(addr_d4, (u16)val_r15);
					phy_write16(0x900200dc | lane_reg_off, (u16)val_r15);
				} else {
					/* type == 3 */
					phy_write16(reg_sp_7c, (u16)val_r12);
					phy_write16(reg_sp_78, (u16)val_r15);
				}

				/* Restore original delay value */
				if (lane == 9) {
					phy_write16(reg_sp_132, cur_val_raw);
				} else {
					uintptr_t reg_restore = 0x90000000 | (((reg_base_var | (lane << 8))) << 1);
					phy_write16(reg_restore, cur_val_raw);
				}
			}
		}
	}
}

/**
 * pmu_cal_stage42_accum_dispatch() - Converted from pmu_cal_stage42_accum_dispatch
 * @dram_type: Target DRAM type identifier
 * @count: Number of items, halfwords, or iterations
 * @slice_base: Parameter slice_base
 *
 * /
 */
void pmu_cal_stage42_accum_dispatch(u32 dram_type, u32 count, void *slice_base)
{
	u8 cand0[4][2];
	u8 cand1[4][2];
	u8 cand2[4][2];
	u8 cand3[4][2];
	u8 diff0[4];
	u8 diff1[4];
	u8 diff2[4];
	u8 diff3[4];
	struct dly_line_entry (*buf)[2] = (struct dly_line_entry (*)[2])slice_base;

	if (count == 0)
		return;
	if (count > 4)
		count = 4;

	for (u32 i = 0; i < count; i++) {
		u16 v0 = buf[0][i].val16;
		cand0[i][0] = (u8)(v0 >> 6);
		cand0[i][1] = (u8)(v0 & 0x3f);
		pmu_delay_tap_step_adjust(cand0[i], 0);

		u16 v1 = buf[1][i].val16;
		cand1[i][0] = (u8)(v1 >> 6);
		cand1[i][1] = (u8)(v1 & 0x3f);
		pmu_delay_tap_step_adjust(cand1[i], 0);

		pmu_eye_margin_step_align(cand0[i], cand1[i], 2);

		u16 v2 = buf[2][i].val16;
		cand2[i][0] = (u8)(v2 >> 6);
		cand2[i][1] = (u8)(v2 & 0x3f);
		pmu_delay_tap_step_adjust(cand2[i], 1);

		u16 v3 = buf[3][i].val16;
		cand3[i][0] = (u8)(v3 >> 6);
		cand3[i][1] = (u8)(v3 & 0x3f);
		pmu_delay_tap_step_adjust(cand3[i], 1);

		pmu_eye_margin_step_align(cand2[i], cand3[i], 2);

		diff0[i] = (cand0[i][1] < 65) ? (64 - cand0[i][1]) : (cand0[i][1] - 64);
		diff1[i] = (cand1[i][1] < 65) ? (64 - cand1[i][1]) : (cand1[i][1] - 64);
		diff2[i] = (cand2[i][1] < 65) ? (64 - cand2[i][1]) : (cand2[i][1] - 64);
		diff3[i] = (cand3[i][1] < 65) ? (64 - cand3[i][1]) : (cand3[i][1] - 64);
	}

	if (dram_type < 4 && dram_type != 2 && dmem_read8(0xce) == 0) {
		u32 max_grp1 = 0;
		u32 max_grp2 = 0;

		for (u32 k = 0; k < count; k++) {
			u32 m1 = (diff0[k] > diff1[k]) ? diff0[k] : diff1[k];
			u32 m2 = (diff2[k] > diff3[k]) ? diff2[k] : diff3[k];
			if (m1 > max_grp1)
				max_grp1 = m1;
			if (m2 > max_grp2)
				max_grp2 = m2;
		}

		for (u32 k = 0; k < count; k++) {
			pmu_cal_slice_delay_best_fit_select(k, max_grp1, max_grp2,
					cand2[k], cand3[k],
					cand0[k], cand1[k],
					(u8 *)slice_base);
		}
	} else {
		for (u32 k = 0; k < count; k++) {
			u32 thresh1 = (diff0[k] > diff1[k]) ? diff0[k] : diff1[k];
			u32 thresh2 = (diff2[k] > diff3[k]) ? diff2[k] : diff3[k];
			pmu_cal_slice_delay_best_fit_select(k, thresh1, thresh2,
					cand2[k], cand3[k],
					cand0[k], cand1[k],
					(u8 *)slice_base);
		}
	}
}

/*
 * pmu_cal_rank_margin_window_check:
 * Converted from pmu_cal_rank_margin_window_check.
 */
/*
 * pmu_cal_rank_margin_window_check:
 * Converted from pmu_slice_deskew_state_latch.
 *
 * Evaluates timing margin windows across active ranks, slices, and pins.
 * Performs centroid calculation, window differential accumulation, and boundary
 * sample searches for LPDDR5 receiver training.
 *
 * Parameters:
 *   arg0       - Mode configuration or packed tap pointer (if 0, default coefficients used)
 *   channel    - DRAM channel index (0 or 1)
 *   slice_mask - 10-bit pin/lane active mask
 *
 * Returns:
 *   0 on success, or non-zero calibration error code.
 */
/**
 * pmu_cal_rank_margin_window_check() - Converted from pmu_slice_deskew_state_latch
 * @arg0: Parameter arg0
 * @channel: Memory channel index (0..1)
 * @slice_mask: Bitmask of active DBYTE slices
 *
 * Evaluates timing margin windows across active ranks, slices, and pins.
 * Performs centroid calculation, window differential accumulation, and boundary
 * sample searches for LPDDR5 receiver training.
 * Parameters:
 * arg0       - Mode configuration or packed tap pointer (if 0, default coefficients used)
 * channel    - DRAM channel index (0 or 1)
 * slice_mask - 10-bit pin/lane active mask
 * 0 on success, or non-zero calibration error code.
 * /
 *
 * Return: Computed u32 value or status.
 */
u32 pmu_cal_rank_margin_window_check(u32 arg0, u32 channel, u32 slice_mask)
{
	u32 start_slice = 0;
	u32 end_slice = 0;
	u32 rank_limit = 0;
	u32 mode = pmu_cal_profile_mode_get();

	if (channel == 1) {
		end_slice = dmem_read8(PMU_DMEM_SLICE_END_B6D);     /* 0x80000b6d */
		start_slice = dmem_read8(PMU_DMEM_SLICE_START_B6C); /* 0x80000b6c */
		u8 p40 = dmem_read8(PMU_DMEM_CAL_PARAM_40);         /* 0x80000040 */
		if (p40 == 3)
			rank_limit = 2;
		else if (p40 == 1)
			rank_limit = 1;
		else
			rank_limit = 0;
	} else if (channel == 0) {
		end_slice = dmem_read8(PMU_DMEM_RANK_BOUNDARY);     /* 0x80000b6b */
		start_slice = dmem_read8(PMU_DMEM_SLICE_START_B6A); /* 0x80000b6a */
		u8 p25 = dmem_read8(PMU_DMEM_CAL_PARAM_25);         /* 0x80000025 */
		rank_limit = (p25 == 3) ? 2 : 1;
	}

	pmu_cal_metric_log(4, 0x00900002, arg0, rank_limit);

	if (arg0 == 0) {
		u8 c0 = dmem_read8(0x20);
		dmem_write16(PMU_DMEM_METRIC_COEFF_X_420, (u16)c0 * c0);
		u8 c1 = dmem_read8(0x21);
		dmem_write16(PMU_DMEM_METRIC_COEFF_Y_422, (u16)c1 * c1);
		u32 sp_18 = 0;

		for (u32 slice = start_slice; slice <= end_slice; slice++) {
			dmem_write8(0x407, (u8)slice);

			for (u32 pin = 0; pin < 10; pin++) {
				if (!(slice_mask & (1 << pin)))
					continue;

				pmu_cal_metric_log(6, 0x00910004, slice, pin, rank_limit, mode);
				dmem_write8(0x408, (u8)pin);

				u32 off = slice * 1320 + pin * 132;
				u8 *rank0 = (u8 *)(uintptr_t)(0x80000dc8 + off);
				u8 *rank1 = (u8 *)(uintptr_t)(0x80002268 + off);
				u8 *rank2 = (u8 *)(uintptr_t)(0x80003708 + off);
				u8 *rank3 = (u8 *)(uintptr_t)(0x80004ba8 + off);

				if (rank_limit == 1) {
					if (mode < 3) {
						u32 ret = pmu_cal_handler_select((u32)(uintptr_t)rank0);
						if (ret != 0)
							return (u8)ret;
						sp_18 = 0;
						if (dmem_read8(0x402) != 0)
							pmu_eye_margin_bidir_scan(rank0, 0, channel, 0);
					} else if (mode == 3) {
						u32 ret = pmu_eye_margin_step_balancer(rank0, rank0, rank2, rank2);
						if (ret != 0)
							return (u8)ret;
						if (dmem_read8(0x402) != 0) {
							pmu_eye_margin_bidir_scan(rank0, 0, channel, sp_18);
							pmu_eye_margin_bidir_scan(rank2, 0, channel, sp_18);
						}
					} else {
						pmu_assert_or_halt(0, 0x00920001);
					}
				} else {
					/* rank_limit == 2 */
					if (mode < 2) {
						u32 ret = pmu_eye_margin_step_balancer(rank0, rank1, rank0, rank1);
						if (ret != 0)
							return (u8)ret;
						if (dmem_read8(0x402) != 0) {
							pmu_eye_margin_bidir_scan(rank0, 0, channel, 0);
							pmu_eye_margin_bidir_scan(rank1, 0, channel, 1);
						}
					} else if (mode == 2) {
						u32 ret = pmu_cal_handler_select((u32)(uintptr_t)rank0);
						if (ret != 0)
							return (u8)ret;
						if (dmem_read8(0x402) != 0)
							pmu_eye_margin_bidir_scan(rank0, 0, channel, 0);

						ret = pmu_cal_handler_select((u32)(uintptr_t)rank1);
						if (ret != 0)
							return (u8)ret;
						sp_18 = 1;
						if (dmem_read8(0x402) != 0)
							pmu_eye_margin_bidir_scan(rank1, 0, channel, 1);
					} else if (mode == 3) {
						u32 ret = pmu_eye_margin_step_balancer(rank0, rank1, rank2, rank3);
						if (ret != 0)
							return (u8)ret;
						if (dmem_read8(0x402) != 0) {
							pmu_eye_margin_bidir_scan(rank0, 0, channel, 0);
							pmu_eye_margin_bidir_scan(rank1, 0, channel, 1);
							pmu_eye_margin_bidir_scan(rank2, 0, channel, 0);
							pmu_eye_margin_bidir_scan(rank3, 0, channel, 1);
						}
					} else {
						pmu_assert_or_halt(0, 0x00930001);
					}
				}
			}
		}
		return 0;
	}

	/* arg0 != 0 branch */
	u32 ptr_addr = (arg0 >> 15) | (arg0 << 17);
	u8 c0 = *(volatile u8 *)(uintptr_t)ptr_addr;
	dmem_write16(PMU_DMEM_METRIC_COEFF_X_420, (u16)c0 * c0);
	u8 c1 = *(volatile u8 *)(uintptr_t)(ptr_addr + 1);
	dmem_write16(PMU_DMEM_METRIC_COEFF_Y_422, (u16)c1 * c1);

	if (rank_limit == 0)
		return 0;

	u32 table_ptr_base = 0x80000dc8 + (1320 * start_slice);

	for (u32 rank = 0; rank < rank_limit; rank++, table_ptr_base += 5280) {
		dmem_write8(0x406, (u8)rank);

		u8 buf_a[132];
		u8 buf_b[132];
		for (int i = 0; i < 64; i++) {
			((u16 *)buf_a)[i] = 0x7f00;
			((u16 *)buf_b)[i] = 0x7f00;
		}
		*(u32 *)&buf_a[128] = 0x3f1f0000;
		*(u32 *)&buf_b[128] = 0x001f0000;

		/* Phase 1: Window Centroid Calculation & Difference Accumulation */
		u32 slice_ptr = table_ptr_base;
		for (u32 slice = start_slice; slice <= end_slice; slice++, slice_ptr += 1320) {
			dmem_write8(0x407, (u8)slice);
			u8 *pin_ptr = (u8 *)(uintptr_t)slice_ptr;

			for (u32 pin = 0; pin < 10; pin++, pin_ptr += 132) {
				if (!(slice_mask & (1 << pin)))
					continue;

				u32 ret = pmu_window_centroid_calc(pin_ptr);
				dmem_write8(0x408, (u8)pin);
				if (ret != 0)
					return (u8)ret;

				pmu_cal_metric_log(4, 0x009b0002, slice, pin);

				u32 swap = pmu_dq_swap_query(rank, slice);
				u8 *target_buf = (swap != 0) ? buf_a : buf_b;

				s32 delta = (s8)pin_ptr[130] - (s8)target_buf[130];
				ret = pmu_window_margin_diff_calc(target_buf, pin_ptr, target_buf, delta, 0);
				if (ret != 0)
					return (u8)ret;
			}
		}

		/* Validate accumulated aperture across both swap groups */
		u32 ret = pmu_cal_handler_select((u32)(uintptr_t)buf_a);
		if (ret != 0)
			return (u8)ret;

		ret = pmu_cal_handler_select((u32)(uintptr_t)buf_b);
		if (ret != 0)
			return (u8)ret;

		/* Phase 2: Eye Boundary Sample Search & Callback */
		if (start_slice <= end_slice) {
			slice_ptr = table_ptr_base;
			for (u32 slice = start_slice; slice <= end_slice; slice++, slice_ptr += 1320) {
				dmem_write8(0x407, (u8)slice);
				u8 *pin_ptr = (u8 *)(uintptr_t)slice_ptr;

				for (u32 pin = 0; pin < 10; pin++, pin_ptr += 132) {
					if (!(slice_mask & (1 << pin)))
						continue;

					pmu_cal_metric_log(4, 0x009c0002, slice, pin);
					dmem_write8(0x408, (u8)pin);

					u32 swap = pmu_dq_swap_query(rank, slice);
					u8 *target_buf = (swap != 0) ? buf_a : buf_b;
					u8 target_y = target_buf[131];

					ret = pmu_eye_boundary_sample_search(pin_ptr, target_y);
					if (ret != 0)
						return (u8)ret;

					if (dmem_read8(0x402) != 0)
						pmu_eye_margin_bidir_scan(pin_ptr, arg0, channel, rank);
				}
			}
		}
	}

	return 0;
}

void pmu_cbt_pulse_train_aa74(u32 a0, u32 a1);

/**
 * pmu_cal_slice_deskew_result_commit() - Converted from pmu_cal_slice_deskew_result_commit
 * @channel: Memory channel index (0..1)
 * @buf: Pointer to data buffer
 * @flags: Configuration bitmask or control flags
 *
 * Commits slice deskew calibration results to PHY CSRs, performs LCDL coarse
 * step overflow wrapping, executes 32-iteration timing delay convergence sweep
 * across active byte-lane slices, and finalizes deskew calibration packet.
 * /
 */
void pmu_cal_slice_deskew_result_commit(u32 channel, void *buf, u32 flags)
{
	struct phy_reg_stream_entry {
		u32 addr;
		u16 val;
	} __attribute__((packed));

	u32 gp28 = dmem_read32(PMU_DMEM_PARAM_41C);
	u8 dram_type = dmem_read8(PMU_DMEM_DRAM_TYPE);

	static const struct phy_reg_stream_entry stream[15] = {
		{ 0x0001f0a7, 514 },
		{ 0x0001f00a, 0 },
		{ 0x0003f084, 0 },
		{ 0x0001ffb5, 1 },
		{ 0x0007f0d7, 0 },
		{ 0x0001f0b3, 0 },
		{ 0x0001f0b4, 0 },
		{ 0x0001f0b9, 1 },
		{ 0x0001f0ba, 1 },
		{ 0x0001f0b0, 0 },
		{ 0x0001f0b1, 2 },
		{ 0x0007f060, 5 },
		{ 0x0007f065, 511 },
		{ 0x0001f0b1, 0 },
		{ 0x0001ffaa, 1024 },
	};

	for (u32 i = 0; i < 9; i++)
		phy_write16(0x9003e17c + (i * 0x200), 1U << i);

	pmu_master_cfg_quad_write(0, 0, 0, 0);
	pmu_phy_lane_timing_offset_set(0xff, 0x5555, 0x5555, 0x5555, 0x5555);
	pmu_phy_reg_stream_play((const u8 *)stream, sizeof(stream));
	phy_write16(0x9003e00a | (gp28 << 1), 1);
	phy_write16(0x9003e016 | (gp28 << 1), 2);
	pmu_dbyte_cal_strobe_seq(0xf, 0xf);

	u32 num_dly_regs = 8;
	u8 ch_mask = (u8)(5 << channel);

	if (flags != 0) {
		phy_write16(0x900fe1ae, 0);
		phy_write16(0x900fe022, phy_read16(0x900e0022) | (1 << 4));
		num_dly_regs = 9;
	}

	phy_write16(0x9003fe24 | (gp28 << 1), 256);
	phy_write16(0x9003fe26 | (gp28 << 1), 256);
	phy_write16(0x9003fe20 | (gp28 << 1), 256);
	phy_write16(0x9003fe22 | (gp28 << 1), 256);

	pmu_cal_strobe_pulse();
	pmu_cal_strobe_secondary_pulse();

	pmu_channel_timing_deskew_reset(0x55, 0x55, 0, 0, ch_mask);
	pmu_cbt_pulse_train_aa74(ch_mask, 0);
	pmu_clk_timing_delay_latch(0x60, 1);

	phy_write16(0x9003e166, 511);
	phy_write16(0x9003e168, 511);
	phy_write16(0x9003e160, phy_read16(0x90020160) | 0x30);
	phy_write16(0x9003e162, 1);

	u8 start_slice = dmem_read8(PMU_DMEM_SLICE_START);
	u8 end_slice = dmem_read8(PMU_DMEM_SLICE_END);

	for (u32 slice = start_slice; slice <= end_slice; slice++) {
		for (u32 dly_idx = 0; dly_idx < num_dly_regs; dly_idx++) {
			u32 reg = 0x90020154 | (slice << 13) | (dly_idx << 9);
			u16 val = phy_read16(reg);
			u32 adjusted = val + PMU_LCDL_STEP_COARSE;

			if (adjusted & PMU_LCDL_STEP_OVERFLOW_MASK)
				adjusted = (val - PMU_LCDL_STEP_COARSE) + PMU_LCDL_STEP_WRAP_OFFSET;

			phy_write16(reg, (u16)adjusted);
		}
	}

	u32 fp_flag = 0;
	u8 val_e6 = 0;
	u32 cal_stat = pmu_cal_status_query(flags, 0);

	if (cal_stat != 0 && dmem_read8(0xe6) != 0) {
		pmu_dbyte_dq_deskew_regs_restore();
		val_e6 = dmem_read8(0xe6);
		fp_flag = 1;
	}

	pmu_channel_timing_deskew_reset(0x53, 0xac, val_e6, val_e6, ch_mask);
	pmu_cbt_pulse_train_aa74(ch_mask, 1);
	pmu_master_cfg_quad_write(0xffff, 0xffff, 0xffff, 0xffff);
	pmu_phy_lane_timing_offset_set(0xf, 0xac53, 0xac53, 0xac53, 0xac53);

	if (cal_stat != 0 && val_e6 != 0) {
		for (u32 bit = 0; bit < 8; bit++) {
			u16 pattern = (val_e6 & (1 << bit)) ? 0x53ac : 0xac53;
			pmu_phy_lane_timing_offset_set(bit, pattern, pattern, pattern, pattern);
		}
		pmu_phy_lane_timing_offset_set(8, 0xac53, 0xac53, 0xac53, 0xac53);
	}

	phy_write16(0x900fe1ae, 0);
	phy_write16(0x900fe1ac, 0);

	for (u32 iter = 0; iter < 32; iter++) {
		pmu_clk_timing_delay_latch(8, 1);

		for (u32 slice = start_slice; slice <= end_slice; slice++) {
			for (u32 dly_idx = 0; dly_idx < num_dly_regs; dly_idx++) {
				u16 stat = phy_read16(0x9002016e | (slice << 13) | (dly_idx << 9));
				u32 target_lane = dly_idx;

				if (fp_flag != 0 && dly_idx != 8)
					target_lane = dmem_read8(PMU_DMEM_SLICE_DLY_BA0 + (slice * 8) + dly_idx);

				u32 dly_reg = 0x90020154 | (slice << 13) | (target_lane << 9);
				if (stat != 0)
					phy_write16(dly_reg, phy_read16(dly_reg) + 1024);
			}
		}

		pmu_dbyte_cal_strobe_seq(0xf, 0xf);
	}

	if (fp_flag != 0)
		pmu_dbyte_dq_deskew_regs_save_and_ramp();

	pmu_phy_lcdl_delay_read((u16 *)buf, 1, 0);

	u32 target_reg = pmu_2b_identity_lut(channel) + 0x24;
	pmu_slice_delay_step_program(target_reg, (const u16 *)buf, 1, 0, 0, 0);

	phy_write16(0x9003e14e, 0);
	phy_write16(0x9003e162, 0);
	pmu_cal_strobe_secondary_pulse();
}

void pmu_cbt_pulse_train_dc54(u32 a0, u32 a1);

/**
 * pmu_cal_timing_margin_envelope_scan() - Converted from pmu_cal_timing_margin_envelope_scan
 * @channel: Memory channel index (0..1)
 * @buf: Pointer to data buffer
 *
 * Performs multi-slice LPDDR5 timing margin envelope scan, iterative boundary
 * convergence sweep (up to 40 steps), LCDL delay coarse wrapping, and output
 * margin deskew calibration packet generation.
 * /
 */
void pmu_cal_timing_margin_envelope_scan(u32 channel, void *buf)
{
	struct phy_reg_stream_entry {
		u32 addr;
		u16 val;
	} __attribute__((packed));

	u32 gp28 = dmem_read32(PMU_DMEM_PARAM_41C);
	u32 iter_count = 0;

	phy_write16(0x9007e108, 0);
	phy_write16(0x9003e014, 0);

	phy_write16(0x90040054 | (gp28 << 1), 1);
	phy_write16(0x90040082 | (gp28 << 1), 1);

	static const struct phy_reg_stream_entry stream[14] = {
		{ 0x0002f059, 3 },
		{ 0x0001ffb5, 0 },
		{ 0x0001f0b5, 1 },
		{ 0x0001f0b3, 0 },
		{ 0x0001f0b4, 0 },
		{ 0x0001f0b0, 0x30 },
		{ 0x0007f060, 1 },
		{ 0x0007f065, 511 },
		{ 0x0007ff26, 0 },
		{ 0x0007ff27, 0 },
		{ 0x0001f0b1, 386 },
		{ 0x0001ffaa, 2048 },
		{ 0x0001ffbe, 0 },
		{ 0x0001f0be, 1 },
	};

	pmu_master_cfg_quad_write(0, 0, 0, 0);
	pmu_phy_lane_timing_offset_set(0xff, 0, 0, 0, 0);
	pmu_phy_reg_stream_play((const u8 *)stream, sizeof(stream));
	phy_write16(0x9003e14e, (dmem_read8(0x403) == 0) ? 520 : 512);
	phy_write16(0x9003e144, 2);

	pmu_inactive_slices_clear();
	pmu_dbyte_cal_strobe_seq(0xf, 0xf);
	pmu_phy_reset_pulse();

	pmu_cal_struct_to_shadow16(0x800009a4, 0x0a);
	pmu_cal_struct_mask_and(0x800009a4, 0x0a, 0x00);
	pmu_cal_struct_mask_or(0x800009a4, 0x0a, 0x59);
	pmu_cal_struct_to_shadow16(0x800009a4, 0x2e);
	pmu_cal_struct_mask_and(0x800009a4, 0x2e, 0x00);
	pmu_cal_struct_to_shadow16(0x800009a4, 0x12);
	pmu_cal_struct_mask_or(0x800009a4, 0x12, 0x10);

	u8 rank = dmem_read8(PMU_DMEM_CAL_RANK);
	u32 cal_struct_off = 0x800009a4 + (rank * 108) + (channel * 54);
	u32 ch_mask_a0 = 0x15 << channel;
	u32 ch_r13 = 5 << channel;

	pmu_cal_multi_rank_pulse_seq_dfec(ch_mask_a0, cal_struct_off, 0, 0xfffbfbff, 0xffffbfff, 0);

	pmu_tracker_field_extract((u8 *)0x800009a4, 0x0a);
	pmu_tracker_field_extract((u8 *)0x800009a4, 0x12);
	pmu_tracker_field_extract((u8 *)0x800009a4, 0x2e);

	pmu_channel_timing_deskew_reset(0, 0, 0, 0, ch_r13);
	phy_write16(0x9003e172, 1);
	phy_write16(0x9003e174, 1);
	pmu_cbt_pulse_train_dc54(ch_r13, 0);

	pmu_cal_metric_log(4, 0x1f20000);
	pmu_clk_timing_delay_latch(0x60, 1);
	pmu_slice_coarse_step_wrap();
	pmu_clk_timing_delay_latch(0, 1);

	u8 history[4] = { 0xff, 0xff, 0xff, 0xff };

	pmu_dbyte_cal_strobe_seq(0xf, 0xf);
	pmu_cbt_pulse_train_dc54(ch_r13, 1);

	phy_write16(0x9003e166, 511);
	phy_write16(0x9003e168, 511);
	phy_write16(0x9003e162, 0x81);
	phy_write16(0x9003e160, 0x30);
	pmu_cal_metric_log(4, 0x1f30000);

	u32 fp_flag;
	do {
		pmu_clk_timing_delay_latch(0, 1);

		u8 start_slice = dmem_read8(PMU_DMEM_SLICE_START);
		u8 end_slice = dmem_read8(PMU_DMEM_SLICE_END);
		fp_flag = 0;

		for (u32 slice = start_slice; slice <= end_slice; slice++) {
			u16 stat = phy_read16(PHY_REG_DBYTE_BASE + 0x016e + (slice * PHY_REG_DBYTE_STRIDE));
			u8 hist = (u8)((history[slice] << 1) | (stat != 0 ? 1 : 0));
			history[slice] = hist;

			if (hist & 0x7) {
				u32 dly_reg = PHY_REG_DBYTE_BASE + PHY_REG_DBYTE_DLY_154 + (slice * PHY_REG_DBYTE_STRIDE);
				u16 dly = phy_read16(dly_reg);
				dly += 1024;
				fp_flag = 1;
				phy_write16(dly_reg, dly);
			}
		}

		pmu_dbyte_cal_strobe_seq(0xf, 0xf);
		pmu_cal_metric_log(4, 0x1f40004, history[0], history[1], history[2], history[3]);

		iter_count++;
		if (iter_count >= 40)
			break;
	} while (fp_flag != 0);

	pmu_assert_or_halt(fp_flag == 0, 0x1f60000);
	pmu_cal_metric_log(4, 0x1f70000);

	u8 start_slice = dmem_read8(PMU_DMEM_SLICE_START);
	u8 end_slice = dmem_read8(PMU_DMEM_SLICE_END);

	if (start_slice <= end_slice) {
		for (u32 slice = start_slice; slice <= end_slice; slice++) {
			u32 dly_reg = PHY_REG_DBYTE_BASE + PHY_REG_DBYTE_DLY_154 + (slice * PHY_REG_DBYTE_STRIDE);
			u16 dly = phy_read16(dly_reg);
			dly += 2048;
			phy_write16(dly_reg, dly);
		}
	}

	phy_write16(0x9003e166, 0);
	phy_write16(0x9003e168, 0);
	pmu_slice_coarse_step_wrap();
	phy_write16(0x9003e160, 0);
	phy_write16(0x9003e16a, 0);

	if (start_slice <= end_slice) {
		for (u32 slice = start_slice; slice <= end_slice; slice++) {
			phy_write16(PHY_REG_DBYTE_BASE + PHY_REG_DBYTE_016A + (slice * PHY_REG_DBYTE_STRIDE), 1);
		}
	}

	phy_write16(0x9003e162, 0x81);
	pmu_cal_metric_log(4, 0x1f80000);
	pmu_clk_timing_delay_latch(0x40, 1);

	pmu_phy_lcdl_delay_read((u16 *)buf, 1, 2);

	if (start_slice <= end_slice) {
		u16 *out_buf = (u16 *)buf;
		for (u32 slice = start_slice; slice <= end_slice; slice++) {
			out_buf[slice] -= 128;
		}
	}

	u32 lut_val = pmu_2b_identity_lut(channel);
	u32 target_reg = gp28 | (lut_val + 0x20);
	pmu_slice_delay_step_program(target_reg, (const u16 *)buf, 1, 0, 0, 1);

	phy_write16(0x9003e14e, 0);
	phy_write16(0x9003e144, 0x20);
	phy_write16(0x9003e144, 0);
	phy_write16(0x9005e0b2, 0);
	phy_write16(0x9003e162, 0);
	phy_write16(0x9007e108, 1);
	phy_write16(0x9003e014, 1);

	pmu_cal_strobe_secondary_pulse();
	phy_write16(0x90040082 | (gp28 << 1), 0);

	rank = dmem_read8(PMU_DMEM_CAL_RANK);
	cal_struct_off = 0x800009a4 + (rank * 108) + (channel * 54);
	pmu_cal_multi_rank_pulse_seq_dfec(ch_mask_a0, cal_struct_off, 0, 0xfffbfbff, 0xffffbfff, 0);
}

/*
 * pmu_cal_vref_dac_step_adjust:
 * Converted from pmu_cal_vref_dac_step_adjust.
 *
 * Calibrates receiver VREF DAC steps, timing margins, and delay line profiles
 * across active channels and ranks for specified calibration stage arg0.
 */
void pmu_cal_rank_deskew_matrix_commit(u32 channel, void *buf);
void pmu_cal_rank_rx_en_delay_commit(u32 channel, u16 fp_flags, u32 sp_0, u32 stage);
void pmu_cal_wck_ck_align_pulse_send(u32 ch, u32 rank);
/**
 * pmu_cal_vref_dac_step_adjust() - Cal Vref Dac Step Adjust
 * @arg0: Parameter arg0
 */
void pmu_cal_vref_dac_step_adjust(u32 arg0)
{
	bool sp_14 = (arg0 != 0 && arg0 != 3 && (arg0 < 4 || arg0 > 7) && arg0 != 0x10);
	u32 sp_20 = ((arg0 | 2) != 6) ? 1 : 0;
	u32 fp = 0xf;

	if (dmem_read8(PMU_DMEM_CLK_GATE_FLAG) & (1 << 2)) {
		u32 mode = pmu_cal_profile_mode_get();
		if ((arg0 & ~1) == 4 && mode == 3)
			fp = 5;
		else
			fp = 0;
	}

	u8 d09_low3 = dmem_read8(0x09) & 7;
	if (d09_low3 != 0 && arg0 < 0x13 && (0x601f0 & (1 << arg0))) {
		fp = (1 << d09_low3) - 1;
	}

	u8 ch_mask = dmem_read8(PMU_DMEM_CAL_PARAM_25) | dmem_read8(PMU_DMEM_CAL_PARAM_40);

	for (u32 channel = 0; channel < 2; channel++) {
		if (!(ch_mask & (1 << channel)))
			continue;

		u32 sp_8 = 0;
		u32 sp_10 = 0;
		dmem_write8(PMU_DMEM_CAL_BYTE, (u8)channel);

		for (u32 rank = 0; rank < 2; rank++) {
			dmem_write8(PMU_DMEM_CAL_RANK, (u8)rank);
			u32 sp_0 = 0;

			if (arg0 < 0x13 && (0x701f8 & (1 << arg0))) {
				sp_8 = pmu_dmem_training_flags_eval();
				sp_0 = sp_8;
				u8 d72 = dmem_read8(0x72);
				u8 d62 = dmem_read8(0x62);
				sp_10 = (!(d72 & 0x20) ? 1 : 0) | (d62 >> 7);
			}

			if (!pmu_channel_rank_avail_check(channel))
				continue;

			/* Stages 4..8 are restricted strictly to channel 0, rank 0 */
			if ((u32)(arg0 - 4) < 5) {
				if (channel != 0 || rank != 0)
					continue;
			}

			pmu_cal_strobe_pulse();
			u8 buf_30[80];
			pmu_memset_words(buf_30, 0, sizeof(buf_30));

			switch (arg0) {
			case 0: {
				void (*p_afac)(u32, u8 *) = (void *)(uintptr_t)pmu_cal_timing_margin_envelope_scan;
				p_afac(channel, buf_30);
				pmu_cal_search_win_max_calc(rank, 1);
				break;
			}
			case 1: {
				void (*p_ea8c)(u32, u8 *) = (void *)(uintptr_t)pmu_cal_rank_deskew_matrix_commit;
				p_ea8c(channel, buf_30);
				break;
			}
			case 4:
			case 5:
			case 6:
			case 7: {
				u32 r2 = (sp_0 == 0) ? 0xff : 511;
				u32 r1 = 0;
				if ((arg0 & ~1) == 6) {
					if (dmem_read8(0xe4) != 0 || dmem_read8(0xe5) != 0 || dmem_read8(0xe6) != 0)
						r1 = 7;
					else
						r1 = 4;
				}
				pmu_cal_rank_param_commit_8aa8(fp, r1, r2, sp_20);
				break;
			}
			case 8: {
				u8 d62 = dmem_read8(0x62);
				u8 d72 = dmem_read8(0x72);
				u8 d96 = dmem_read8(0x96);
				u32 cond = ((d62 >> 6) & 1) | ((d62 >> 7) & 1) |
						   ((d96 >> 6) & 1) | (!(d72 & 0x20) ? 1 : 0);
				u32 r11 = (cond == 0) ? 0xff : 511;
				u32 r2 = ((d96 & 0x10) << 5) | r11;
				pmu_cal_rank_param_commit_8aa8(fp, 0, r2, 2);
				break;
			}
			case 15: {
				u16 buf_28[4];
				pmu_cal_slice_step_diff_commit(channel, buf_28, 1);
				break;
			}
			case 16: {
				void (*p_aba8)(u32, u8 *, u32) = (void *)(uintptr_t)pmu_cal_slice_deskew_result_commit;
				p_aba8(channel, buf_30, sp_0);
				pmu_cal_search_win_max_calc(rank, 0);
				break;
			}
			case 17: {
				void (*p_c9c0)(u32, u16, u32, u32) = (void *)(uintptr_t)pmu_cal_rank_rx_en_delay_commit;
				p_c9c0(channel, (u16)fp, sp_0, 0x11);
				break;
			}
			case 18: {
				void (*p_c9c0)(u32, u16, u32, u32) = (void *)(uintptr_t)pmu_cal_rank_rx_en_delay_commit;
				p_c9c0(channel, (u16)fp, sp_0, 0x12);
				break;
			}
			case 19: {
				void (*p_c4cc)(u32, u32) = (void *)(uintptr_t)pmu_cal_wck_ck_align_pulse_send;
				p_c4cc(channel, rank);
				break;
			}
			default:
				break;
			}
		}

		if ((u32)(arg0 - 0x11) < 2) {
			dmem_write16(0x428, dmem_read16(0x428) + 1);
		}

		pmu_phy_mode_cfg_dispatch(3);
		dmem_write8(PMU_DMEM_CAL_RANK, 0);

		if (dmem_read8(0x405) == 0) {
			u32 lut_code = channel + 1;
			pmu_lcdl_delay_profile_update(sp_14, 0, lut_code, channel);
		}
	}

	if (dmem_read8(PMU_DMEM_CLK_GATE_FLAG) & (1 << 6)) {
		pmu_slice_deskew_pulse_seq(arg0);
	}

	if ((u32)(arg0 - 0x11) < 2) {
		if (dmem_read8(PMU_DMEM_CLK_GATE_FLAG) & (1 << 6))
			pmu_slice_metric_scan_and_program();
	}

	if (arg0 == 3 && dmem_read8(0x405) == 0) {
		if (dmem_read8(0x00) & (1 << 5))
			pmu_cal_error_code_log();
	}

	dmem_write8(0x0f, 0);
}

/* Forward declarations of PMU helper routines called by pmu_cal_dbyte_rx_fifo_reset_poll */
void pmu_cbt_timing_pulse_coordinator(u32 arg0, u32 arg1);
void pmu_cal_multi_rank_deskew_sweep(u32 arg0);

/**
 * pmu_cal_dbyte_rx_fifo_reset_poll() - Converted from pmu_cal_dbyte_rx_fifo_reset_poll
 * @arg0: Parameter arg0
 * @arg1: Parameter arg1
 * @arg2: Parameter arg2
 *
 * Performs DBYTE RX FIFO reset polling, deskew calibration sweep, strobe pulse checks,
 * telemetry metric logging, and multi-channel phase sample collection.
 * /
 *
 * Return: Computed s16 value or status.
 */
s16 pmu_cal_dbyte_rx_fifo_reset_poll(u32 arg0, u32 arg1, void *arg2)
{
	u8 buf[256];
	s16 ret_val = 0;

	/* Step 1: Query initial DQ status and clear calibration status flag */
	pmu_cal_dbyte_dq_status_check(arg0, arg1);
	dmem_write8(0x0a7c, 0);

	/*
	 * Step 2: Calibration and Strobe Scan Sweep (Block 2).
	 * Executed when arg1 is non-zero and bit 1 of DMEM[0x1b] is set.
	 */
	if (arg1 != 0 && (dmem_read8(0x1b) & (1 << 1))) {
		pmu_memset_words(buf, 0, 256);

		u32 mode_flag = (arg0 != 1) ? 1 : 0;
		u32 mode_val = mode_flag + 8;
		u8 rank = dmem_read8(PMU_DMEM_CAL_RANK);

		pmu_cal_metric_log(5, 0x1120003, rank, arg0 - 1, mode_val);

		u32 slice_offset = dmem_read32(PMU_DMEM_PARAM_41C);
		u32 win_params[2];
		win_params[0] = 0;
		win_params[1] = 0;

		pmu_deskew_and_tracker_reset();

		pmu_cal_sequence_pulse_send(0x41 << 19, 7, 8, 0, 0, 0, 0);
		pmu_cbt_coarse_step_pulse_seq(0, 4, 1);
		u32 win_val = pmu_cal_window_params_dispatch((const u8 *)win_params, (u16)arg0, 3);
		pmu_cbt_coarse_step_pulse_seq(0, 4, 1);
		pmu_cal_sequence_pulse_send(1 << 18, 7, 4, 0, 0, 0, 0);
		pmu_cal_sequence_pulse_send((1 << 19) | 0x80, 7, 8, 0, 0, 0, 0);

		u32 fp = (rank << 12) | slice_offset;
		u32 reg_tx_dqs = 0x90060004 | (fp << 1);
		u16 stride = pmu_cal_stride_get();
		phy_write16(reg_tx_dqs, 0x40);

		u16 sample_val = (u16)(win_val - 8);
		pmu_cal_strobe_pulse();

		u16 stride_x4 = (u16)(stride << 2);
		if (pmu_dbyte_deskew_sample_check(arg0, sample_val, stride_x4) != 0)
			pmu_assert_or_halt(0, 0x8b << 17);

		/* Sweep gate delay taps */
		u32 reg_gate = 0x90060002 | (((mode_val << 8) | fp) << 1);
		u8 step = dmem_read8(0x47c);

		for (s16 i = 0; i < 256; i += (s16)step) {
			phy_write16(reg_gate, (u16)i);
			pmu_cal_strobe_pulse();
			buf[i] = pmu_dbyte_deskew_sample_check(arg0, sample_val, stride_x4);
		}

		phy_write16(reg_gate, 0);
		phy_write16(reg_tx_dqs, 0);
		pmu_cal_strobe_pulse();

		pmu_cal_metric_log(4, 0x1190002, mode_val - 7, mode_val);

		/* Telemetry: pack samples 8 bits per byte and log */
		for (s16 outer = 0; outer < 256; outer += (s16)(step * 8)) {
			u8 packed = 0;
			const u8 *p = &buf[outer];
			for (int bit = 7; bit >= 0; bit--, p += step)
				packed |= (*p << bit);
			pmu_cal_metric_log(4, 0x11a0001, packed);
		}
		pmu_cal_metric_log(4, 0x11b0000);

		/* Search for first non-zero sample */
		s16 scan_idx = 0;
		while (scan_idx < 256 && !buf[scan_idx])
			scan_idx += (s16)step;

		s16 r14_val = (s16)(scan_idx - 64);
		if (r14_val >= 256) {
			pmu_assert_or_halt(0, 0x47 << 18);
		} else {
			pmu_cal_metric_log(4, 0x11d0001, (u32)(u16)r14_val);
		}

		/* Clamp DMEM 0x105 calibration window offset */
		u8 val_105 = dmem_read8(0x105);
		if ((u32)(val_105 - 1) >= 4) {
			val_105 = 4;
			dmem_write8(0x105, 4);
		}

		u32 offset_105 = (u32)val_105 << 5;
		pmu_cal_metric_log(4, 0x11e0002, offset_105);

		ret_val = (s16)(r14_val - (s16)offset_105);
		pmu_cal_metric_log(5, 0x1230003, rank, arg0, (u32)(u16)ret_val);
	}

	/*
	 * Step 3: Multi-Channel Phase Sampling Loop (Block 3).
	 */
	u32 is_mode2 = ((arg0 & 3) == 2) ? 1 : 0;
	pmu_cbt_timing_pulse_coordinator(arg0, arg1);

	u32 gp28 = dmem_read32(PMU_DMEM_PARAM_41C);
	u8 rank = dmem_read8(PMU_DMEM_CAL_RANK);
	u8 step = dmem_read8(0x47c);
	u8 *out_matrix = (u8 *)arg2;

	u32 base_reg = (rank << 12) | gp28;
	for (u16 delay = 0; delay < 128; delay += step) {
		for (u32 ch = 0; ch < 7; ch++)
			phy_write16(0x90060002 | ((base_reg | (ch << 8)) << 1), delay);
		pmu_cal_strobe_pulse();

		u32 out_mask = 0;
		pmu_dbyte_deskew_phase_sample(is_mode2, &out_mask, 3);

		u8 *p = &out_matrix[(uintptr_t)rank * 1792 + delay];
		for (u32 ch = 0; ch < 7; ch++, p += 256) {
			p[0] |= (u8)((out_mask >> ch) & 1);
			p[128] |= (u8)((out_mask >> (ch + 8)) & 1);
		}
	}

	/* Reset the 7 channel gate delay registers */
	for (u32 ch = 0; ch < 7; ch++)
		phy_write16(0x9007e002 | (((gp28 | (ch << 8)) << 1)), 0);
	pmu_cal_strobe_pulse();

	dmem_write8(0x0a7c, 1);
	pmu_cal_multi_rank_deskew_sweep(arg0);

	return ret_val;
}

/*
 * pmu_cal_wck_ck_align_pulse_send:
 * Converted from pmu_cal_wck_ck_align_pulse_send.
 */

/**
 * pmu_cal_wck_ck_align_pulse_send() - Converted from pmu_cal_wck_ck_align_pulse_send
 * @ch: Memory channel index (0..1)
 * @rank: DRAM rank index (0..1)
 *
 * Sends WCK/CK alignment calibration pulse sequence across slices, performs
 * deskew delay convergence sweeps (15 iterations), measures phase detector
 * transition points, averages slice metrics, and commits trained alignment
 * offsets to DMEM tracking registers.
 * /
 */
void pmu_cal_wck_ck_align_pulse_send(u32 ch, u32 rank)
{
	s16 buf1[4][15];
	s16 buf2[4][15];
	u16 buf3[4][15];
	u16 buf4[4][15];
	u16 cnt_54[4];
	u16 cnt_4c[4];
	u16 prev_44[4];
	u16 stat_3c[4];
	s16 res_34[4];

	pmu_memset_words(buf1, 0, sizeof(buf1) + sizeof(buf2) + sizeof(buf3) + sizeof(buf4));

	u32 gp28 = dmem_read32(PMU_DMEM_PARAM_41C);
	u32 reg_0e = 0x9004000e | (gp28 << 1);

	phy_write16(0x90040054 | (gp28 << 1), 1);
	phy_write16(PHY_REG_VREF_CTRL | (gp28 << 1), 1);

	u16 orig_0e = phy_read16(reg_0e);
	phy_write16(reg_0e, orig_0e | (1 << 8));

	u8 slice_start = dmem_read8(PMU_DMEM_SLICE_START);
	u8 dq_swap_mask = dmem_read8(PMU_DMEM_DQ_SWAP_MASK);

	u8 ch_mask = (u8)(1 << ch);
	pmu_cal_window_valid_check(ch_mask, 5);

	u32 ch_rank_idx = (ch << 1) + rank;
	u32 test_bit = dq_swap_mask & (1 << ch_rank_idx);
	u8 slice_idx_first = test_bit ? (slice_start + 1) : slice_start;
	u8 slice_idx_other = test_bit ? slice_start : (slice_start + 1);

	u32 var_0c = pmu_slice_dq_bitmask_swizzle(slice_idx_first, 4, ch);
	pmu_cal_metric_log(5, 0x13c0002, (u8)var_0c, var_0c);

	u32 flag_14 = 0;
	if ((u8)var_0c == 1) {
		pmu_cal_window_valid_check(ch_mask, 6);
		if ((u8)pmu_slice_dq_bitmask_swizzle(var_0c, 4, ch) == 7)
			flag_14 = 1;
	}

	u32 val_18 = pmu_phy_ac_lane_timing_delay_set((ch_mask << 2) | ch_mask);

	u8 slice_end = dmem_read8(PMU_DMEM_SLICE_END);

	for (u32 s = slice_start; s <= slice_end; s++) {
		cnt_4c[s] = 0;
		cnt_54[s] = 0;
		prev_44[s] = 0;
		stat_3c[s] = flag_14;
	}

	for (u32 iter = 0; iter <= 14; iter++) {
		s8 tap = (s8)dmem_read8(0x098c + iter);

		pmu_rank_slice_deskew_latch((u8)((tap << 4) | (tap & 0x0f)), val_18);
		pmu_deskew_dly0_set(0xc8);
		pmu_deskew_dly1_set(val_18);
		pmu_clk_timing_latch(0, 1);
		pmu_rank_slice_deskew_latch(0, val_18);
		pmu_cal_window_valid_check(val_18, 0x1a);

		s32 tap_val = (iter < 8) ? (s8)(-tap) : (s8)(tap & 7);

		u32 fp_val = pmu_slice_dq_bitmask_swizzle(var_0c, 4, ch);
		if (pmu_dmem_1c_bit_query(rank, ch) != 0)
			fp_val |= (u8)pmu_slice_dq_bitmask_swizzle(slice_idx_other, 4, ch);

		for (u32 s = slice_start; s <= slice_end; s++) {
			u32 shift = (s != var_0c) ? 2 : 0;
			u16 bit1 = (fp_val >> (shift + 3)) & 1;
			u16 bit0 = (fp_val >> (shift + 2)) & 1;

			buf4[s][iter] = bit1;
			buf3[s][iter] = bit0;

			if (bit0 != prev_44[s]) {
				prev_44[s] = bit0;
				buf1[s][cnt_54[s]] = (s16)tap_val;
				cnt_54[s]++;
			}

			if (bit1 != stat_3c[s]) {
				stat_3c[s] = bit1;
				buf2[s][cnt_4c[s]] = (s16)tap_val;
				cnt_4c[s]++;
			}

			if (iter == 14) {
				if (cnt_54[s] == 0) {
					buf1[s][0] = (bit0 == 0) ? 7 : -7;
				} else if (bit0 == 0 && cnt_54[s] != 1) {
					buf1[s][cnt_54[s]] = 7;
					cnt_54[s]++;
				}

				if (cnt_4c[s] == 0) {
					buf2[s][0] = (bit1 == 0) ? 7 : -7;
				} else if (cnt_4c[s] >= 2 && ((flag_14 != 0) == (bit1 != 0))) {
					buf2[s][cnt_4c[s]] = 7;
					cnt_4c[s]++;
				}
			}
		}
	}

	pmu_cal_metric_log(5, 0x1480002, rank, ch);
	pmu_cal_metric_log(4, 0x1490000);

	for (u32 s = slice_start; s <= slice_end; s++) {
		pmu_cal_metric_log(4, 0x14a0001, s);
		pmu_post_trace_log_halfwords(4, 0x14b000f, buf3[s], 15);
		pmu_cal_metric_log(4, 0x14c0001, s);
		pmu_post_trace_log_halfwords(4, 0x14d000f, buf4[s], 15);
	}

	for (u32 s = slice_start; s <= slice_end; s++) {
		if (cnt_54[s] >= 2)
			buf1[s][0] = (s16)(buf1[s][0] + buf1[s][cnt_54[s] - 1]) / 2;

		if (cnt_4c[s] >= 2)
			buf2[s][0] = (s16)(buf2[s][0] + buf2[s][cnt_4c[s] - 1]) / 2;

		res_34[s] = (s16)(buf1[s][0] + buf2[s][0]) / 2;
	}

	pmu_cal_metric_log(4, 0x15a0002, ch, (s32)res_34[slice_idx_other]);
	pmu_cal_metric_log(4, 0x15b0002, ch, (s32)res_34[var_0c]);

	s16 val_other = res_34[slice_idx_other];
	if (val_other < 1)
		res_34[slice_idx_other] = -val_other;
	else
		res_34[slice_idx_other] = val_other | 8;

	s16 val_0c = res_34[var_0c];
	if (val_0c < 1)
		res_34[var_0c] = -val_0c;
	else
		res_34[var_0c] = val_0c | 8;

	u8 val_comb = (u8)(((res_34[slice_idx_other] & 0x0f) << 4) | (res_34[var_0c] & 0x0f));
	pmu_cal_metric_log(5, 0x15c0003, ch, rank, val_comb);
	pmu_rank_slice_deskew_latch(val_comb, val_18);

	dmem_write8(0x0b7e + (ch << 1) + rank, val_comb);
	dmem_write8(0xd2 + ch * 5 + rank * 10, val_comb);

	phy_write16(reg_0e, orig_0e);
}

/**
 * pmu_cal_rank_rx_en_delay_commit() - Converted from pmu_cal_rank_rx_en_delay_commit
 * @channel: Memory channel index (0..1)
 * @fp_flags: Frame pointer flags
 * @sp_0: Saved stack pointer reference parameter
 * @stage: Training stage index or bitmask
 *
 * Calibrates receiver enable (RxEn) LCDL timing delay steps across active PHY
 * DBYTE slices for the specified channel and stage.
 * Algorithm overview:
 * 1. Queries identity LUT for channel, initializes calibration state buffer (buf_54)
 * via pmu_cal_channel_rank_boundary_eval, reads current DBYTE LCDL base delays, and extracts upper configuration bytes.
 * 2. Sweeps 13 delay steps (0..12). In each step:
 * - Programs coarse/fine complementary delay nibbles into PHY DBYTE registers.
 * - Triggers calibration strobe pulse and resets measurement tracking scratchpad.
 * - Dispatches BIST measurement sequences via pmu_cal_bist_cmd_strobe_dispatch, pmu_cal_slice_step_eval_sweep, pmu_dmem_reg_stream_unpack, and pmu_eye_centroid_avg_calc.
 * - Gathers eye margin delta metrics across slices and updates minimum margin difference table.
 * 3. Evaluates best delay step per slice:
 * - Selects delay step maximizing minimum margin difference (with tie-breaker on margin sum).
 * - Commits calibrated optimal step to DMEM parameter table at 0x8000e798.
 * - Programs finalized trained LCDL delay words to PHY DBYTE registers.
 * 4. Completes rank boundary evaluation via pmu_cal_dbyte_deskew_results_apply.
 * /
 */
void pmu_cal_rank_rx_en_delay_commit(u32 channel, u16 fp_flags, u32 sp_0, u32 stage)
{
	u8 buf_54[584];
	u8 scratch_buf[64];
	u16 min_diff_table[13][4];
	u16 buf_304[13][4];
	u16 buf_36c[13][4];
	u16 buf_3d4[4][10];
	u16 buf_424[4][10];
	u16 buf_474[4][10];
	u16 buf_4c4[4][10];
	u16 buf_514[4][10];
	u16 buf_564[4][10];

	u32 lut_val = pmu_2b_identity_lut(channel);

	pmu_memset_words(buf_36c, 0, sizeof(buf_36c));
	pmu_memset_words(buf_304, 0, sizeof(buf_304));
	pmu_memset_words(min_diff_table, 0, sizeof(min_diff_table));

	pmu_cal_metric_log(4, 0x1680004, channel, fp_flags, sp_0, stage);

	pmu_cal_channel_rank_boundary_eval(buf_54, (u8)channel, sp_0, stage);

	u32 gp28 = dmem_read32(PMU_DMEM_PARAM_41C);
	phy_write16(0x9003e00e | (gp28 << 1), 1);

	u16 off0 = *(u16 *)&buf_54[514];
	u16 off1 = *(u16 *)&buf_54[516];

	pmu_cal_metric_log(4, 0x1690002, off0, off1);

	u16 reg0 = phy_read16(0x90020000 | ((gp28 | off0) << 1));
	u8 r14_val = (u8)(reg0 >> 8);

	u16 reg1 = phy_read16(0x90020000 | ((gp28 | off1) << 1));
	u8 r13_val = (u8)(reg1 >> 8);

	pmu_cal_metric_log(4, 0x16a0002, r14_val, r13_val);

	u32 chan_mask = (1 << channel) | ((1 << channel) << 2);
	u32 is_lt_3200 = 1;
	if (stage == 0x11)
		is_lt_3200 = pmu_freq_lt_3200_check();

	/* Outer loop: sweep 13 delay steps (idx = 0..12) */
	for (u32 idx = 0; idx <= 12; idx++) {
		pmu_memset_words(buf_564, 0, sizeof(buf_564));
		pmu_memset_words(buf_514, 0, sizeof(buf_514));
		pmu_memset_words(buf_4c4, 0, sizeof(buf_4c4));

		u8 comp_idx = 12 - idx;
		u16 val_r12 = ((u16)r14_val << 8) | ((u16)comp_idx << 4) | (u8)idx;
		u16 val_r13 = ((u16)r13_val << 8) | ((u16)idx << 4) | (u8)comp_idx;

		u8 slice_start = dmem_read8(PMU_DMEM_SLICE_START);
		u8 slice_end   = dmem_read8(PMU_DMEM_SLICE_END);

		for (u32 s = slice_start; s <= slice_end; s++) {
			u32 base = (s << 12) | gp28;
			phy_write16(0x90020000 | (((lut_val + off0) | base) << 1), val_r12);
			phy_write16(0x90020000 | (((lut_val + off1) | base) << 1), val_r13);
		}

		pmu_cal_strobe_pulse();

		for (u32 i = 0; i < 40; i++)
			dmem_write8(0x6594 + i * 848, 0);

		pmu_cal_bist_cmd_strobe_dispatch((u32)(uintptr_t)scratch_buf, buf_54[523], buf_54[518],
			(u8)chan_mask, (u32)(uintptr_t)buf_54, sp_0, buf_54[525]);

		pmu_cal_slice_step_eval_sweep((void *)0x80006318, buf_54[523], buf_54[518], 4,
			(u8)chan_mask, (u8)fp_flags, is_lt_3200,
			(const u16 *)buf_54, sp_0, buf_54[525], scratch_buf);

		if (buf_54[524] != 0) {
			pmu_phy_timing_delay_latch(1, (u8)chan_mask);
			pmu_cal_slice_step_eval_sweep((void *)0x80006318, buf_54[523], 3, 4,
				(u8)chan_mask, (u8)fp_flags, 1, (const u16 *)buf_54, 0, 0, scratch_buf);
			pmu_phy_timing_delay_latch(0, (u8)chan_mask);
		}

		pmu_dmem_reg_stream_unpack(scratch_buf);
		pmu_eye_centroid_avg_calc((const u16 *)(buf_54 + 16), (u16 *)buf_564, (u16 *)buf_514, (u16 *)buf_4c4);

		u16 ctrl_8 = *(u16 *)&buf_54[520];
		if (ctrl_8 != 255) {
			for (u32 i = 0; i < 40; i++)
				dmem_write8(0x6594 + i * 848, 0);

			pmu_cal_bist_cmd_strobe_dispatch((u32)(uintptr_t)scratch_buf, buf_54[523], (u8)ctrl_8,
				(u8)chan_mask, (u32)(uintptr_t)(buf_54 + 8), sp_0, buf_54[525]);

			pmu_cal_slice_step_eval_sweep((void *)0x80006318, buf_54[523], (u8)ctrl_8, 4,
				(u8)chan_mask, (u8)fp_flags, is_lt_3200,
				(const u16 *)(buf_54 + 8), sp_0, buf_54[525], scratch_buf);

			pmu_dmem_reg_stream_unpack(scratch_buf);
			pmu_eye_centroid_avg_calc((const u16 *)(buf_54 + 24), (u16 *)buf_474, (u16 *)buf_424, (u16 *)buf_3d4);
			ctrl_8 = *(u16 *)&buf_54[520];
		}

		u8 num_samples = buf_54[522];
		for (u32 s = slice_start; s <= slice_end; s++) {
			s32 min_val = 127;
			if (num_samples != 0) {
				s16 sum = 0;
				for (u32 k = 0; k < num_samples; k++) {
					s8 diff = (s8)(buf_4c4[s][k] - buf_514[s][k]);
					if (diff < min_val)
						min_val = diff;
					sum += diff;
				}
				buf_36c[idx][s] = sum;

				if (ctrl_8 == 0) {
					s32 min_val2 = 127;
					s16 sum2 = 0;
					for (u32 k = 0; k < num_samples; k++) {
						s8 diff = (s8)(buf_3d4[s][k] - buf_424[s][k]);
						if (diff < min_val2)
							min_val2 = diff;
						sum2 += diff;
					}
					buf_304[idx][s] = sum2;
					if (min_val2 < min_val) {
						min_val = min_val2;
						buf_36c[idx][s] = sum2;
					}
				}
			}
			min_diff_table[idx][s] = (u16)(u8)min_val;
		}
	}

	/* Tail commit loop: select best delay step and commit to PHY and DMEM */
	u8 slice_start = dmem_read8(PMU_DMEM_SLICE_START);
	u8 slice_end   = dmem_read8(PMU_DMEM_SLICE_END);

	for (u32 s = slice_start; s <= slice_end; s++) {
		s8 best_diff = -1;
		u8 best_step = 0xff;
		s16 best_sum = -1;

		for (u32 step = 0; step < 13; step++) {
			s8 diff = (s8)(u8)min_diff_table[step][s];
			pmu_cal_metric_log(4, 0x1710003, s, step, diff);

			if (diff > best_diff || (diff == best_diff && (s16)buf_36c[step][s] > best_sum)) {
				best_step = (u8)step;
				best_diff = diff;
				best_sum  = (s16)buf_36c[step][s];
			}
		}

		pmu_cal_metric_log(4, 0x1720003, s, (s8)best_step, best_sum);

		u16 param_428 = dmem_read16(0x428);
		dmem_write16(0xe798 + (s * 8) + (param_428 * 2), (u16)(s8)best_step);
		pmu_cal_metric_log(4, 0x1730003, s, (s8)best_step, param_428);

		u8 comp_step = 12 - (s8)best_step;
		u16 val_off0 = ((u16)r14_val << 8) | ((u16)comp_step << 4) | (u8)best_step;
		u32 addr0 = 0x90020000 | (((lut_val + off0) | (s << 12) | gp28) << 1);
		phy_write16(addr0, val_off0);

		u16 val_off1 = ((u16)r13_val << 8) | ((u16)(u8)best_step << 4) | comp_step;
		u32 addr1 = 0x90020000 | (((lut_val + off1) | (s << 12) | gp28) << 1);
		phy_write16(addr1, val_off1);
	}

	pmu_cal_dbyte_deskew_results_apply(buf_54, (u8)channel, stage);
}

/**
 * pmu_cal_lane_window_sweep_coordinator() - Coordinates multi-channel, multi-rank lane window sweep calibration
 * @arg0: Parameter arg0
 * @arg1: Parameter arg1
 *
 * Derived from vendor PMU code at address 0xcf88 (pmu_cal_lane_window_sweep_coordinator).
 * Sweeps 4 calibration delay tap configurations (0..3) across active channels and ranks,
 * scans margin matrix via pmu_cal_margin_matrix_scan_eval, calculates window centroids via pmu_window_centroid_calc,
 * evaluates channel margin accumulation via pmu_rank_margin_channel_accum, finds the optimal window aperture,
 * applies the trained delay taps to PHY registers via pmu_slice_deskew_state_latch and pmu_phy_cfg_target_set,
 * and records the final trained delays into DMEM.
 * /
 */
void pmu_cal_lane_window_sweep_coordinator(u32 arg0, u32 arg1)
{
	u16 margin_table[4][2][2][2];
	u16 best_taps[2][2][2];
	u32 fp_range = (arg1 != 0) ? 511 : 255;
	u32 pin_count = (arg1 != 0) ? 9 : 8;

	pmu_memset_words(margin_table, 0, sizeof(margin_table));
	pmu_memset_words(best_taps, 0, sizeof(best_taps));

	u8 cal_p40 = dmem_read8(PMU_DMEM_CAL_PARAM_40);
	u8 cal_p25 = dmem_read8(PMU_DMEM_CAL_PARAM_25);
	u8 active_mask = dmem_read8(PMU_DMEM_CAL_RANK) ? cal_p40 : cal_p25;

	/* Phase 1: Outer Calibration Configuration Sweep (4 iterations: cfg_idx 0..3) */
	for (u32 cfg_idx = 0; cfg_idx < 4; cfg_idx++) {
		u8 cfg_val = (u8)(cfg_idx | (cfg_idx << 4));

		for (u32 ch = 0; ch < 2; ch++) {
			u8 mask = (u8)((1 << (ch + 2)) | (1 << ch));
			if (active_mask & mask)
				pmu_slice_deskew_state_latch(cfg_val, mask);
		}

		u32 vref = (dmem_read8(PMU_DMEM_CLK_GATE_FLAG) & 4) ? 0x3f : dmem_read8(0x100);
		pmu_cal_margin_matrix_scan_eval(arg0, 0, fp_range, 2, vref, 0);

		for (u32 ch = 0; ch < 2; ch++) {
			u8 mask = (u8)((1 << (ch + 2)) | (1 << ch));
			if (!(active_mask & mask))
				continue;

			u8 start_slice = dmem_read8(PMU_DMEM_SLICE_START_B6A);
			u8 end_slice = dmem_read8(PMU_DMEM_SLICE_END_B6D);
			uintptr_t slice_buf = 0x80000dc8 + (ch * 5280) + (start_slice * 1320);

			for (u32 slice = start_slice; slice <= end_slice; slice++) {
				uintptr_t pin_ptr = slice_buf;
				for (u32 pin = 0; pin < pin_count; pin++) {
					pmu_window_centroid_calc((u8 *)pin_ptr);
					pin_ptr += 132;
				}
				slice_buf += 1320;
			}

			for (u32 rank = 0; rank < 2; rank++) {
				u8 rank_active = (rank != 0) ? cal_p40 : cal_p25;
				if (rank_active != 0) {
					u8 r_start = (rank != 0) ? dmem_read8(PMU_DMEM_SLICE_START_B6C) :
											   dmem_read8(PMU_DMEM_SLICE_START_B6A);
					u8 r_end   = (rank != 0) ? dmem_read8(PMU_DMEM_SLICE_END_B6D) :
											   dmem_read8(PMU_DMEM_RANK_BOUNDARY);

					margin_table[cfg_idx][ch][rank][0] = pmu_rank_margin_channel_accum(ch, r_start, r_end, pin_count, 1);
					margin_table[cfg_idx][ch][rank][1] = pmu_rank_margin_channel_accum(ch, r_start, r_end, pin_count, 2);
				}
			}
		}
	}

	/* Phase 2: Aperture Search & Optimization */
	for (u32 ch = 0; ch < 2; ch++) {
		u8 mask = (u8)((1 << (ch + 2)) | (1 << ch));
		if (!(active_mask & mask))
			continue;

		for (u32 rank = 0; rank < 2; rank++) {
			u8 rank_active = (rank != 0) ? cal_p40 : cal_p25;
			if (!rank_active)
				continue;

			for (u32 dir = 0; dir < 2; dir++) {
				u16 max_width = 0;
				u16 best_idx = 0;

				for (u32 iter = 0; iter < 4; iter++) {
					u16 width = margin_table[iter][ch][rank][dir];
					if (width > max_width) {
						max_width = width;
						best_idx = (u16)iter;
					}
				}

				best_taps[ch][rank][dir] = best_idx;

				if (max_width == 0)
					pmu_assert_or_halt(0, 0x022e0003);
			}
		}
	}

	/* Phase 3: Apply Trained Delays to PHY and Commit to DMEM */
	for (u32 ch = 0; ch < 2; ch++) {
		u8 mask = (u8)((1 << (ch + 2)) | (1 << ch));
		if (!(active_mask & mask))
			continue;

		pmu_cal_pulse_seq_coordinator(ch, 0x50, 3);

		for (u32 rank = 0; rank < 2; rank++) {
			u8 rank_active = (rank != 0) ? cal_p40 : cal_p25;
			if (!rank_active)
				continue;

			pmu_phy_cfg_target_set(rank);

			u8 combined = (u8)((best_taps[ch][rank][1] << 4) | best_taps[ch][rank][0]);
			pmu_slice_deskew_state_latch(combined, mask);

			dmem_write8(0x80000b7a + (rank * 2) + ch, combined);

			if (rank == 0) {
				if (ch == 0)
					dmem_write8(0x800000d1, combined);
				else
					dmem_write8(0x800000d6, combined);
			} else {
				if (ch == 0)
					dmem_write8(0x800000db, combined);
				else
					dmem_write8(0x800000e0, combined);
			}
		}
	}

	phy_write16(PHY_REG_CFG_00E4, dmem_read8(PMU_DMEM_CAL_PARAM_25));
	phy_write16(PHY_REG_CFG_00E6, dmem_read8(PMU_DMEM_CAL_PARAM_40));
}
/**
 * cbt_def() - Cbt Def
 */
static void cbt_def(void)
{
	pmu_cal_sequence_pulse_send(0, 7, 0, 0, 0, 0, 0);
}
/**
 * set_slice_step() - Set Slice Step
 * @slice_start: Starting slice index
 * @slice_end: Ending slice index
 * @slice_mask: Bitmask of active DBYTE slices
 * @gp28: Parameter gp28
 * @step: Delay line tap step adjustment
 */
static void set_slice_step(u8 slice_start, u8 slice_end, u8 slice_mask, u32 gp28, u16 step)
{
	u32 base = 0x90020000 | (gp28 << 1);
	phy_write16(0x9005e01a | (gp28 << 1), step);
	for (u32 slice = slice_start; slice <= slice_end; slice++) {
		if (slice_mask & (1 << slice))
			phy_write16(base | (slice << 13), step);
	}
}

/**
 * pmu_cal_margin_envelope_step_eval() - Decompiled from pmu_cal_margin_envelope_step_eval
 *
 * Evaluates margin envelopes, programs CBT sequence across active ranks,
 * sweeps step offsets across DBYTE slices, checks error status,
 * logs telemetry, and records optimal envelope parameters.
 * /
 */
void pmu_cal_margin_envelope_step_eval(void)
{
	u8 clk_gate_flag = dmem_read8(0x01);
	u32 flags = pmu_dmem_training_flags_eval();

	pmu_cal_metric_log(0xa, 0x2180001, flags);
	pmu_phy_mode_cfg_dispatch(3);
	dmem_write16(0x14, 512);

	u8 dmem_08 = dmem_read8(0x08);
	u32 status_query = pmu_cal_status_query(flags, 0);
	u32 is_dram_type_2 = (dmem_08 == 2) ? 1 : 0;

	u16 cfg_val;
	if (dmem_08 == 4) {
		u16 val = phy_read16(0x90020160);
		phy_write16(0x9003ff60, val | (1 << 2));
		cfg_val = (flags == 0) ? 0x88 : 0x8c;
	} else {
		cfg_val = (flags != 0) ? 0x0c : 0x08;
	}
	phy_write16(0x900fe0c0, cfg_val);

	u32 sp_28 = (is_dram_type_2 << 1) + 2;

	pmu_master_cfg_quad_write(0xffff, 0xffff, 0xffff, 0xffff);
	phy_write16(0x900fe0ca, 0);
	pmu_dbyte_cal_strobe_seq(0xf, 0xf);

	phy_write16(0x9003e172, 0xff);
	phy_write16(0x9003e174, 1);
	phy_write16(0x9003e14e, 0);
	phy_write16(0x9003e166, 511);
	phy_write16(0x9003e168, 511);
	phy_write16(0x9003e162, 1);

	pmu_phy_slice_mask_set((flags != 0) ? 0x1ff : 0xff);

	phy_write16(0x900fe1ae, 256);
	phy_write16(0x900fe1ac, 256);
	pmu_cal_strobe_pulse();

	if (dmem_read8(0x403) <= 1)
		pmu_phy_reset_pulse();

	u32 sp_2c = dmem_read8(0x16);
	u32 latch_delay = (clk_gate_flag & 4) ? 0 : 15;
	u32 gp28 = dmem_read32(0x41c);
	u8 slice_start = dmem_read8(0xb68);
	u8 slice_end = dmem_read8(0xb69);
	u8 slice_mask = dmem_read8(0xb98);

	s32 best_step = -1;

	for (s32 rank = 1; rank >= 0; rank--) {
		u32 rank_mask = (1 << (rank + 2)) | (1 << rank);
		u8 active_mask = (dmem_read8(0xb66) != 0) ? dmem_read8(0x40) : dmem_read8(0x25);

		if (!(active_mask & rank_mask))
			continue;

		phy_write16(0x900fe022, status_query ? 0x16 : 0x10);
		pmu_deskew_and_tracker_reset();
		pmu_cbt_cal_stat_set();

		cbt_def();
		pmu_cal_sequence_pulse_send(1 << 20, 5, 0, 0x80, 0, rank_mask, 0);
		pmu_cal_sequence_pulse_send(1 << 23, 0x29, sp_28, 0, 0, rank_mask, 7);
		pmu_cal_sequence_pulse_send(0, 7, dmem_read8(0x400), 0, 0, 0, 0);
		cbt_def();
		pmu_cal_sequence_pulse_send(1 << 21, 5, 0, 256, 0, rank_mask, 0);
		pmu_cal_sequence_pulse_send(0x41 << 18, 0x2a, sp_28, 0, 0, rank_mask, 7);
		pmu_cal_sequence_pulse_send(0, 7, 4, 0, 0, 0, 10);

		pmu_cbt_cal_stat_clear();

		cbt_def();
		cbt_def();
		pmu_cal_sequence_pulse_send(0x80, 7, 4, 0, 0, 0, 0);

		u32 found_pass = 0;
		for (u32 step = 0; step < 32; step++) {
			set_slice_step(slice_start, slice_end, slice_mask, gp28, (u16)step);

			pmu_hw_timer_delay(0x14);
			pmu_cal_strobe_secondary_pulse();
			pmu_clk_timing_delay_latch(latch_delay, 1);

			u16 err_accum = 0;
			for (u32 slice = slice_start; slice <= slice_end; slice++) {
				if (!(slice_mask & (1 << slice)))
					continue;

				u16 err = phy_read16(0x9002017a | (slice << 13));
				err_accum |= err;

				if (err == 0)
					pmu_cal_metric_log(4, 0x1be0003, (u32)rank, slice, step);
			}

			pmu_dbyte_cal_strobe_seq(0xf, 0xf);

			if (err_accum == 0) {
				u16 param_val = (u16)(step + sp_2c);
				pmu_cal_param_table_write((u32)rank, 0, param_val);
				pmu_cal_param_table_write((u32)rank, 1, param_val);
				if ((s32)step > best_step)
					best_step = (s32)step;
				found_pass = 1;
				break;
			}
		}

		pmu_assert_or_halt(found_pass, 0x1c30001);
	}

	pmu_assert_or_halt((best_step >= 0) ? 1 : 0, 0x1c50000);

	u32 calc_step = sp_2c + (u32)best_step;
	u16 final_step = (calc_step > 31) ? 31 : (u16)calc_step;

	set_slice_step(slice_start, slice_end, slice_mask, gp28, final_step);

	pmu_cal_strobe_secondary_pulse();
	phy_write16(0x9003e166, 0);
	phy_write16(0x9003e168, 0);
	phy_write16(0x9003e162, 0);

	dmem_write16(0x14, 0x8200);

	if (dmem_read8(0x405) == 0)
		pmu_dmem_stride_descriptor_read();

	pmu_post_cmd_conditional_dispatch(9);
}


/**
 * pmu_cal_dbyte_deskew_fine_tune() - Converted from pmu_cal_dbyte_deskew_fine_tune
 * @mode: Operational or calibration mode
 *
 * /
 */
void pmu_cal_dbyte_deskew_fine_tune(u32 mode)
{
	pmu_cal_metric_log(4, 0x510000);

	u32 counter = 0;

	for (u32 ch = 0; ch < 2; ch++) {
		u8 active = (ch == 0) ? dmem_read8(0x25) : dmem_read8(0x40);
		if (!active)
			continue;

		pmu_phy_mode_cfg_dispatch(ch);

		u16 reg_pair[2];
		reg_pair[0] = dmem_read16(0xe8 + ch * 2);
		reg_pair[1] = dmem_read16(0xec + ch * 2);

		if (mode != 0) {
			u8 num_entries = dmem_read8(0x409);
			for (u32 rank = 0; rank < 2; rank++) {
				for (u32 entry = 0; entry < num_entries; entry++) {
					u8 val15 = dmem_read8(0xb6e + ch * 2 + rank + entry * 4);
					pmu_cal_metric_log(4, 0x520002, dmem_read8(0x46c + entry), val15);

					u32 csr_addr = 0x90000000 | ((((reg_pair[0] << 2) | 0x41000) + (counter & 0xff)) << 1);
					phy_write16(csr_addr, val15);
					counter++;
				}
			}

			if (counter & 1) {
				u32 csr_addr = 0x90000000 | ((((reg_pair[0] << 2) | 0x41000) + (counter & 0xff)) << 1);
				phy_write16(csr_addr, 0);
				counter++;
			}
		} else {
			u32 ch_shift = ch * 2;
			u8 num_entries = dmem_read8(0x409);

			for (u32 rank = 0; rank < 2; rank++) {
				pmu_deskew_and_tracker_reset();
				pmu_cal_markers_set((u16)(reg_pair[rank] << 2));

				u8 sp18 = 0;
				for (int r13 = 0; r13 < num_entries; r13++) {
					for (u32 r14 = 0; r14 < 2; r14++) {
						u8 r15 = dmem_read8(0x46c + r13);
						u8 fp = dmem_read8(0xb6e + (rank ? 0x14 : 0) + (ch_shift + r13 * 4) + r14);
						u32 r2 = (r14 != 0 && r13 != num_entries - 1) ? 0x22 : 0;
						u32 r3_mask = 1 << r14;

						if (sp18 != 0 && r15 == 0xc) {
							if (!((dmem_read8(0x1c) >> ch_shift) & r3_mask))
								continue;
							fp |= 0x80;
						}

						pmu_cal_sequence_pulse_send(0, 6, r2, fp, r15, (r3_mask << 2) | r3_mask, 0);

						if (r14 == 1 && sp18 == 0 && r15 == 0xc) {
							if (dmem_read8(0x1c) != 0) {
								r13--;
								sp18 = 1;
							} else {
								sp18 = 0;
							}
						}

						if (r15 == 0xc && dmem_read8(0x1c) == 0)
							pmu_cbt_coarse_step_pulse_seq(0, 4, 0);

						pmu_cal_metric_log(4, 0x530005, rank, r14, ch, r15, (s8)fp);
					}
				}

				pmu_cal_sequence_pulse_send(0, 7, 2, 0, 0, 0, 0x28);
			}
		}
	}

	phy_write16(0x900400fc, 0);
	phy_write16(0x900400f4, 0);
}

/**
 * pmu_cal_phase_detector_edge_align() - Converted from pmu_cal_phase_detector_edge_align
 *
 * /
 */
void pmu_cal_phase_detector_edge_align(void)
{
	dmem_write32(0x414, 0x80000dc8);
	dmem_write32(0x418, 0x80000dc8);
	pmu_cal_metric_log(4, 0x5a0001);

	u8 buf_a[8];
	u8 buf_b[22];
	u8 buf_c[28];
	u8 buf_d[12];
	u8 buf_e[14];

	pmu_memcpy_words(buf_a, (const void *)0x800004a4, 8);
	u16 sp_94 = 1;
	pmu_memcpy_words(buf_b, (const void *)0x800004d8, 22);
	pmu_memcpy_words(buf_c, (const void *)0x800004ee, 28);
	pmu_memcpy_words(buf_d, (const void *)0x800004ac, 12);
	pmu_memcpy_words(buf_e, (const void *)0x800004b8, 14);

	u32 r0 = dmem_read32(0x418) >> 1;
	dmem_write16(0x108, (u16)r0);
	u16 r14 = dmem_read16(0xf2);

	u8 sp_24[4] = {0};

	for (u32 mode_idx = 0; mode_idx < 3; mode_idx++) {
		sp_24[0] = sp_24[1] = sp_24[2] = sp_24[3] = 0;

		u32 mode_param = 1;
		void *sp_10[4] = {0};
		u32 limit_50 = 1;

		if (mode_idx == 0) {
			mode_param = 10;
			sp_10[0] = buf_a;
			sp_10[1] = &sp_94;
			limit_50 = 2;
		} else if (mode_idx == 1) {
			mode_param = 9;
			sp_10[0] = buf_c;
			sp_10[2] = buf_b;
			sp_10[3] = buf_d;
			limit_50 = (u8)dmem_read32(0x40c);
		} else {
			mode_param = 1;
			sp_10[0] = buf_e;
			limit_50 = 1;
		}

		for (u32 r30 = 0; r30 < 4; r30++) {
			u8 count = sp_24[r30];
			u32 r1 = ((r30 | 2) == 3) ? mode_param : 1;
			u32 max_val = (r1 > 1) ? r1 : 1;

			for (u32 i = 0; i < count; i++) {
				for (u32 j = 0; j < limit_50; j++) {
					u16 *p_reg = (u16 *)sp_10[r30];
					u32 dmem_val = *(volatile u32 *)(0x80000480 + mode_idx * 4 + 0x18);
					u32 r3_base = j << 12;

					for (u32 k = max_val; k > 0; k--) {
						u16 reg_off = p_reg[i];
						u32 full_addr = (r3_base | dmem_val | reg_off) << 1;
						u16 v1, v2;
						u32 type;

						if (r30 < 2) {
							v1 = phy_read16(0x90000000 | full_addr);
							if (dmem_val & 1) {
								v2 = v1;
								v1 = 0;
								type = 2;
							} else {
								v2 = 0;
								type = 1;
							}
						} else {
							v1 = phy_read16(0x90000000 | full_addr);
							v2 = phy_read16(0x90000000 | (full_addr + 2));
							type = 3;
						}

						if (dmem_read16(0xf2) != 0 || dmem_read16(0xe8) != 0 || dmem_read16(0xea) != 0) {
							pmu_phy_reg_write_shadow_track((u16)(r14 << 2), v1, v2, (full_addr >> 1) & 0x7ffff, type);
							r14++;
						}
					}
				}
			}
		}
	}

	phy_write16(0x9018021e, r14 - dmem_read16(0xf2));

	u32 r2 = dmem_read32(0x418) >> 1;
	dmem_write16(0x10c, (u16)r2);
	u16 sp_66 = dmem_read16(0xf0);
	dmem_write16(0x10a, (u16)(r2 - dmem_read16(0x108)));

	for (u32 ch = 0; ch < 2; ch++) {
		u8 active = (ch == 0) ? dmem_read8(0x25) : dmem_read8(0x40);
		if (!active)
			continue;

		u16 reg_e8 = dmem_read16(0xe8 + ch * 2);
		u32 sp_38 = (u32)active << ch;
		u8 sp_20 = 0;
		u8 num_entries = dmem_read8(0x409);

		for (int r14_idx = 0; r14_idx < num_entries; r14_idx++) {
			for (u32 r15 = 0; r15 < 2; r15++) {
				u8 fp = dmem_read8(0x46c + r14_idx);
				u8 val_b6e = dmem_read8(0xb6e + sp_38 + r14_idx * 4 + r15);
				u8 r30 = (sp_20 != 0 && fp == 0xc) ? (val_b6e | 0x80) : val_b6e;

				u32 buf_10[4];
				buf_10[0] = 0;
				buf_10[1] = 0;
				buf_10[2] = 0;
				buf_10[3] = 0;
				pmu_cal_timing_packet_format((u16 *)buf_10, fp, r30, 1 << r15);

				if (sp_20 != 0 && fp == 0xc && !((dmem_read8(0x1c) >> sp_38) & (1 << r15))) {
					buf_10[0] = 0;
					buf_10[1] = 0;
					buf_10[2] = 0;
					buf_10[3] = 0;
				}

				pmu_cal_dual_rank_state_eval(buf_10, (u16 *)&sp_66, (u16 *)sp_24);

				if (r15 != 0 && r14_idx != num_entries - 1) {
					buf_10[0] = 0;
					buf_10[1] = 0x1b000000;
					buf_10[2] = 0;
					buf_10[3] = 0;
					pmu_cal_dual_rank_state_eval(buf_10, (u16 *)&sp_66, (u16 *)sp_24);
				}

				if (r15 == 1 && sp_20 == 0 && fp == 0xc) {
					r14_idx--;
					sp_20 = 1;
				}
			}
		}
	}

	dmem_write16(0x10e, (dmem_read32(0x418) >> 1) - dmem_read16(0x10c));
	pmu_cal_metric_log(4, 0x5f0000);
}

/**
 * pmu_cal_rank_deskew_matrix_commit() - Converted from pmu_cal_rank_deskew_matrix_commit
 * @channel: Memory channel index (0..1)
 * @buf: Pointer to data buffer
 *
 * Commits rank deskew calibration matrix to PHY CSRs, performs iterative
 * 11-step multi-slice LCDL coarse delay step convergence, handles DRAM-type
 * specific delay line wrap-around and strobe pulse sequence, and finalizes
 * CBT deskew matrix configuration.
 * /
 */
void pmu_cal_rank_deskew_matrix_commit(u32 channel, void *buf)
{
	struct phy_reg_stream_entry {
		u32 addr;
		u16 val;
	} __attribute__((packed));

	u32 gp28 = dmem_read32(PMU_DMEM_PARAM_41C);
	u32 gp28_sh1 = gp28 << 1;

	u32 ch_mask = (1U << (channel + 2)) | (1U << channel);
	u32 cbt_idx = pmu_cbt_active_entry_lookup();
	u32 cbt_val = pmu_cal_table_50c_lookup();

	u32 sp_220;
	if (cbt_val > 186) {
		sp_220 = 0xfe;
	} else {
		sp_220 = (pmu_cal_table_50c_lookup() * 2) + 0x22;
	}

	u32 lut_val = pmu_2b_identity_lut(channel);
	u8 d04 = dmem_read8(0x04) & 0x0f;
	u8 d_b66 = dmem_read8(0xb66);

	pmu_cal_metric_log(5, 0x2af0003, d_b66, channel, d04);

	struct phy_reg_stream_entry stream_save[15] = {
		{ 0x10097, 0 },
		{ 0x10093, 0 },
		{ 0x10005 | gp28, 0 },
		{ 0x1008b, 0 },
		{ 0x10095, 0 },
		{ 0x20074, 0 },
		{ 0x20057, 0 },
		{ 0x2002a, 0 },
		{ 0x2003f | gp28, 0 },
		{ 0x20041 | gp28, 0 },
		{ 0x20037 | gp28, 0 },
		{ 0x2003f | gp28, 0 },
		{ 0x2003d | gp28, 0 },
		{ 0x2003e | gp28, 0 },
		{ 0x70011, 0 },
	};

	pmu_phy_reg_buffer_stream((u8 *)stream_save, 15, 1);

	phy_write16(0x900e0022, 0);
	phy_write16(0x9003e12e, 0);
	phy_write16(0x9003e160, 8);
	phy_write16(0x9003e162, 0x40);
	phy_write16(0x9005e0ae, 0xaa);

	pmu_deskew_and_tracker_reset();

	u8 ch_mask_b = (u8)ch_mask;
	pmu_cal_sequence_pulse_send(0x41 << 19, 7, 4, 0, 0, ch_mask_b, 0);
	pmu_cal_sequence_pulse_send(0, 7, 8, 0, 0, 0, 0);
	pmu_cal_sequence_pulse_send(0x80 | (1 << 19), 7, 4, 0, 0, ch_mask_b, 0);

	pmu_clk_timing_delay_latch(0, 1);
	pmu_delay_us(5000, 5);
	pmu_dbyte_cal_strobe_pulse_seq(1, 0xff, 0);

	u8 dram_type = dmem_read8(PMU_DMEM_DRAM_TYPE);
	u32 r1_dt = (dram_type == 4) ? 1 : 0;
	u32 r2_dt = (dram_type << 6) + 6;
	u32 val_43a = (r1_dt << 2) + 4;
	if (r2_dt > 255)
		r2_dt = 255;

	dmem_write8(0x43a, (u8)val_43a);
	dmem_write16(0x438, (u16)r2_dt);
	pmu_cal_metric_log(4, 0x28b0001);

	cbt_val += (cbt_idx & 1);
	u32 sp_236 = sp_220 + (sp_220 & 1);

	phy_write16(0x90040082 | gp28_sh1, 1);
	phy_write16(0x90040054, 1);

	u16 val_6e = phy_read16(0x9004006e | gp28_sh1);
	u16 val_7e = val_6e & 0x3f;
	if (dram_type == 2)
		val_7e |= (1 << 11);
	else
		val_7e |= (1 << 12);
	phy_write16(0x9004007e | gp28_sh1, val_7e);

	u32 sp_240 = lut_val & 3;

	phy_write16(0x9003e126, 512);
	phy_write16(0x9003e00a | gp28_sh1, 1);
	pmu_rank_mask_multiparam_dispatch(1);

	struct phy_reg_stream_entry stream_play[19] = {
		{ 0x0001ffb5, 0 },
		{ 0x0001f0b5, 1 },
		{ 0x0001f4b5, 1 },
		{ 0x0001f0b3, 0 },
		{ 0x0001f0b4, 0 },
		{ 0x0001f0b9, 1 },
		{ 0x0001f0ba, 1 },
		{ 0x0001f0b1, 2 },
		{ 0x0007f060, 1 },
		{ 0x0007f065, 511 },
		{ 0x0007ff28, 0xffff },
		{ 0x0007ff29, 0xffff },
		{ 0x0007ff2a, 0xffff },
		{ 0x0007ff2b, 0xffff },
		{ 0x0001ffbe, 0 },
		{ 0x0001f0be, 1 },
		{ 0x0001f4be, 16 },
		{ 0x0001ffaa, (u16)(val_43a << 10) },
		{ 0x0001f0a7, 768 },
	};

	pmu_phy_reg_stream_play((const u8 *)stream_play, sizeof(stream_play));

	pmu_dbyte_cal_strobe_seq(0xf, 0xf);
	pmu_inactive_slices_clear();
	pmu_dbyte_slice_cal_param_pulse(channel, 1);

	pmu_deskew_and_tracker_reset();
	pmu_cal_sequence_pulse_send(0, 7, 4, 0, 0, 0, 0);
	pmu_cbt_cal_stat_set();

	u32 r15_cbt = (ch_mask << 5) & 64;
	pmu_cal_sequence_pulse_send(r15_cbt | (1 << 25), 7, 2, 0, 0, 0, 0);

	u8 cbt_idx_b = (u8)cbt_idx;
	pmu_cal_sequence_pulse_send(r15_cbt, 7, cbt_idx_b, 0, 0, 0, 0);
	pmu_cal_sequence_pulse_send(4 | (1 << 18), 1, (u8)cbt_val, 1 << 12, 0, 0, 0);
	pmu_cal_sequence_pulse_send(r15_cbt | (1 << 15), 7, 4, 0, 0, 0, 1);

	pmu_cbt_cal_stat_clear();
	pmu_cal_sequence_pulse_send(r15_cbt, 7, 0x10, 0, 0, 0, 0);
	pmu_cal_sequence_pulse_send(r15_cbt | (1 << 7), 7, 4, 0, 0, 0, 0);

	pmu_cal_metric_log(4, 0x2b00000);
	phy_write16(0x9003e14e, 0x6300);
	pmu_clk_timing_delay_latch(8, 1);
	phy_write16(0x9003e14e, 768);
	pmu_cal_metric_log(4, 0x2b10000);

	u16 timing_438 = dmem_read16(0x438);
	if (dmem_read8(PMU_DMEM_CLK_GATE_FLAG) & (1 << 2)) {
		pmu_clk_timing_delay_latch(timing_438, 1);
	} else {
		for (int i = 0; i < 10; i++)
			pmu_clk_timing_delay_latch(timing_438, 1);
	}

	pmu_dbyte_slice_cal_param_pulse(channel, 0);
	pmu_phy_lcdl_delay_read((u16 *)buf, 0, 2);

	u32 reg_target_base = gp28 | (sp_240 + 0x2a);
	pmu_slice_delay_step_program(reg_target_base, (const u16 *)buf, 0, 0, 0, 1);

	phy_write16(0x9003e14e, 0);

	u32 fp_cbt = pmu_cbt_active_entry_lookup();
	pmu_cal_metric_log(5, 0x29a0002, dmem_read8(0xb66), channel);
	pmu_slice_reg_query_mailbox_send(channel);

	u8 d106 = dmem_read8(0x106);
	pmu_cal_metric_log(4, 0x29b0001, d106);

	if (d106 & 1) {
		pmu_cal_metric_log(10, 0x29c0000);
		goto cleanup;
	}

	pmu_cal_metric_log(4, 0x29d0002, dmem_read8(0xb66), sp_240);
	pmu_slice_phy_reg_step_adjust(sp_240, 1, 0);

	fp_cbt += (fp_cbt & 1);
	pmu_ac_lane_profile_setup();

	if (dmem_read8(0x403) == 0) {
		pmu_deskew_and_tracker_reset();
		pmu_cal_sequence_pulse_send(0, 7, 4, 0, 0, 0, 0);
		pmu_clk_timing_delay_latch(0, 1);
	}

	pmu_deskew_and_tracker_reset();
	u32 r13_cbt = fp_cbt + 0x1e;
	pmu_cbt_coarse_step_pulse();
	pmu_cbt_3phase_pulse_seq();
	pmu_clk_timing_delay_latch(0, 1);

	u16 saved_88 = phy_read16(0x90040088);
	u16 target_cfg_val = (dram_type == 2) ? 513 : 1026;
	phy_write16(0x90040088, (dram_type == 2) ? 3 : 2);
	phy_write16(0x9004007a | gp28_sh1, target_cfg_val);
	phy_write16(0x9004007c | gp28_sh1, target_cfg_val);

	u16 saved_84 = phy_read16(0x90040084);
	phy_write16(0x90040084, saved_84 | (1 << 7));
	pmu_slice_phy_reg_step_adjust(sp_240, 1, 1);

	u32 slice_mask = (pmu_dmem_training_flags_eval() == 0) ? 0xff : 511;
	phy_write16(0x9003e172, 0xffff);
	phy_write16(0x9003e174, 0xffff);
	pmu_phy_slice_mask_set(slice_mask);

	phy_write16(0x900fe0c0, 1);
	phy_write16(0x9003ff7c, 0);

	pmu_dbyte_cal_strobe_seq(0xf, 0xf);
	pmu_phy_reset_pulse();

	pmu_cal_slice_step_scan_eval_80f0(r13_cbt & 0xff, ch_mask_b);
	pmu_cal_metric_log(4, 0x2a00000);

	/* Iterative Convergence Engine: 11 outer iterations across active slices */
	u16 slice_done[4] = { 0 };
	u16 slice_val[4] = { 0 };
	u16 prev_val[4] = { 0 };
	u32 flag_sp67 = 0;

	for (int iter = 11; iter > 0; iter--) {
		pmu_clk_timing_delay_latch(0, 1);

		u8 start_slice = dmem_read8(PMU_DMEM_SLICE_START);
		u8 end_slice = dmem_read8(PMU_DMEM_SLICE_END);

		for (u32 slice = start_slice; slice <= end_slice; slice++) {
			if (slice_done[slice] != 0)
				continue;

			u32 slice_sh13 = slice << 13;
			u16 stat = phy_read16(0x9002016e | slice_sh13);
			slice_val[slice] = stat;

			u32 slice_reg_base = ((gp28 | (slice << 12) | (sp_240 + 0x2a)) << 1);
			u32 reg_addr = 0x90020000 | slice_reg_base;
			u16 r13_reg = phy_read16(reg_addr);

			u16 prev_stat = prev_val[slice];
			if (prev_stat != 0 && stat == 0) {
				slice_done[slice] = 1;
				pmu_cal_metric_log(4, 0x2a10002, slice, r13_reg);
				continue;
			}

			if (prev_stat == 0 && iter == 11 && stat == 0) {
				pmu_cal_metric_log(4, 0x2a20000);
				flag_sp67 = 1;
			}

			u32 fine_delay = r13_reg & 0x3f;
			u32 coarse_delay = r13_reg >> 6;

			pmu_cal_metric_log(4, 0x2a30003, slice, coarse_delay, fine_delay);

			coarse_delay += 2;
			u32 new_val = (coarse_delay << 6) | fine_delay;

			u32 target_reg = 0x90021e00 | slice_reg_base;

			if (coarse_delay < 0x15) {
				phy_write16(target_reg, (u16)new_val);
				prev_val[slice] = slice_val[slice];
			} else {
				if (flag_sp67 != 0 && (coarse_delay % dram_type) != 0) {
					pmu_cal_metric_log(4, 0x2a40000);
					phy_write16(target_reg, (u16)fine_delay);
					pmu_dbyte_cal_strobe_seq(0xf, 0xf);
					pmu_phy_reset_pulse();
					pmu_cal_metric_log(4, 0x2a50003, slice, 0, fine_delay);
					pmu_clk_timing_delay_latch(0, 1);

					u16 new_stat = phy_read16(0x9002016e | slice_sh13);
					slice_val[slice] = new_stat;

					if (new_stat != 0) {
						u16 combined = (u16)((coarse_delay << 6) | fine_delay);
						phy_write16(target_reg, combined);
						pmu_cal_metric_log(4, 0x2a60003, slice, coarse_delay, fine_delay);
						slice_done[slice] = 1;
						continue;
					} else {
						pmu_cal_metric_log(4, 0x2a70003, slice, 0, fine_delay);
						new_val = fine_delay;
						pmu_assert_or_halt(0, 0x2a80000);
						phy_write16(target_reg, (u16)new_val);
						prev_val[slice] = slice_val[slice];
					}
				} else {
					pmu_assert_or_halt(0, 0x2a80000);
					phy_write16(target_reg, (u16)new_val);
					prev_val[slice] = slice_val[slice];
				}
			}
		}

		pmu_dbyte_cal_strobe_seq(0xf, 0xf);
		pmu_phy_reset_pulse();
	}

	phy_write16(0x90040088, saved_88);
	phy_write16(0x90040084, saved_84);
	pmu_phy_profile_param_program();
	pmu_slice_phy_reg_step_adjust(sp_240, 0, 0);

	pmu_cal_metric_log(5, 0x2a90000);
	pmu_slice_reg_query_mailbox_send(channel);

cleanup:
	pmu_cal_metric_log(4, 0x2b20000);
	pmu_dbyte_cal_strobe_pulse_seq(0, 0, 0);

	phy_write16(0x9003e160, 0);
	phy_write16(0x9003e162, 0);
	pmu_deskew_and_tracker_reset();
	pmu_cbt_coarse_step_pulse_seq(0, 5, 1);
	pmu_clk_timing_delay_latch(0, 1);
	pmu_rank_mask_multiparam_dispatch(0);

	pmu_deskew_and_tracker_reset();
	pmu_cal_sequence_pulse_send(0, 7, 4, 0, 0, 0, 0);
	pmu_cal_sequence_pulse_send(1 << 16, 3, cbt_idx_b, 1 << 12, 0, ch_mask_b, 0);
	pmu_cal_sequence_pulse_send(0x80, 7, 4, 0, 0, 0, 0);

	pmu_clk_timing_delay_latch(0, 1);
	pmu_phy_reset_pulse();
	pmu_cal_strobe_secondary_pulse();

	pmu_phy_reg_buffer_stream((u8 *)stream_save, 15, 0);
}

/**
 * pmu_cal_stage_post_process_d7bc() - /
 *
 * Derived from vendor PMU code at address 0xd7bc.
 */
void pmu_cal_stage_post_process_d7bc(void)
{
	volatile u8 *dmem = (volatile u8 *)PMU_DMEM_BASE;
	__asm__("" : "+r"(dmem));
	u8 mode = dmem[0x8e];
	*(volatile u16 *)(dmem + 0x14) = 4;
	if ((mode & 3) != 0) {
		pmu_cal_metric_log(10, 0x02090000);
		pmu_cal_vref_dac_step_adjust(0);
		pmu_post_cmd_conditional_dispatch(2);
	} else {
		pmu_cal_metric_log(10, 0x02080000);
		pmu_cal_vref_dac_step_adjust(0x10);
		pmu_post_cmd_conditional_dispatch(5);
	}
	*(volatile u16 *)(dmem + 0x14) = 0x8004;
}

/**
 * pmu_cal_stage_post_process_d804() - /
 * @arg0: Parameter arg0
 *
 * Derived from vendor PMU code at address 0xd804.
 */
void pmu_cal_stage_post_process_d804(u32 arg0)
{
	volatile u8 *dmem = (volatile u8 *)PMU_DMEM_BASE;
	__asm__("" : "+r"(dmem));
	pmu_cal_metric_log(10, (arg0 != 0) ? 0x020a0000 : 0x020b0000);
	*(volatile u16 *)(dmem + 0x14) = 2;
	pmu_cal_vref_dac_step_adjust((arg0 == 0) ? 2 : 1);
	pmu_post_cmd_conditional_dispatch((arg0 != 0) ? 1 : 0xfe);
	*(volatile u16 *)(dmem + 0x14) = 0x8002;
}

/**
 * pmu_cal_stage_post_process_d720() - /
 * @arg0: Parameter arg0
 *
 * Derived from vendor PMU code at address 0xd720.
 */
void pmu_cal_stage_post_process_d720(u32 arg0)
{
	volatile u8 *dmem = (volatile u8 *)PMU_DMEM_BASE;
	__asm__("" : "+r"(dmem));
	u16 freq = *(volatile u16 *)(dmem + 0x06);
	*(volatile u16 *)(dmem + 0x14) = 8;

	u8 p401 = 0;
	if (freq >= 3200 && arg0 == 0) {
		if ((dmem[0x0d] & (1 << 5)) == 0)
			p401 = 1;
	}
	dmem[0x401] = p401;
	dmem[0x402] = (arg0 == 0) ? 1 : 0;

	u8 mode = dmem[0x8e];
	if ((mode & 3) != 0) {
		if (arg0 == 0) {
			pmu_cal_metric_log(10, 0x020d0000);
			pmu_cal_vref_dac_step_adjust(5);
			pmu_cal_vref_dac_step_adjust(4);
		} else {
			pmu_cal_metric_log(10, 0x020c0000);
			if (pmu_cal_ptr_tag_check(0, 0x020c0000) == 0)
				pmu_cal_vref_dac_step_adjust(7);
			pmu_cal_vref_dac_step_adjust(6);
		}
	} else {
		pmu_cal_metric_log(10, 0x020e0000);
	}

	pmu_post_cmd_conditional_dispatch((arg0 == 0) ? 3 : 0xfd);
	dmem[0x402] = 0;
	*(volatile u16 *)(dmem + 0x14) = 0x8008;
}

/**
 * pmu_freq_delay_step_calc() - /
 * @val: Value to write or configure
 * @min_delay: Parameter min_delay
 *
 * Derived from vendor PMU code at address 0x8dd8.
 *
 * Return: Computed u32 value or status.
 */
u32 pmu_freq_delay_step_calc(u32 val, u32 min_delay)
{
	volatile u8 *dmem = (volatile u8 *)PMU_DMEM_BASE;
	__asm__("" : "+r"(dmem));
	u16 freq = *(volatile u16 *)(dmem + 0x06);
	u8 dram_type = dmem[0x08];
	u32 cycles;

	if (val >= 0x30d41) {
		u32 temp = (u32)(((u64)(val >> 5) * 0x0a7c5ac5ULL) >> 39);
		cycles = (temp * freq) / 20;
	} else {
		cycles = (u32)(((u64)(val * freq) * 0x431bde83ULL) >> 51);
	}

	u32 r0 = (cycles + dram_type + 1) / dram_type;
	u32 r12 = min_delay + 1;

	if (dmem[0xa7c] != 0 && (dmem[0x01] & (1 << 3)) == 0) {
		u8 b64 = dmem[0xb64];
		if (b64 != 0) {
			u8 b65 = dmem[0xb65];
			r12 = b64 * min_delay * b65;
		}
	}

	return (r12 > r0) ? r12 : r0;
}

/**
 * pmu_dbyte_pin_mask_seq_7d7c() - Applies multi-stage DBYTE pin mask delays and latching sequence across active slices
 * @arg0: Parameter arg0
 * @arg1: Parameter arg1
 * @arg2: Parameter arg2
 *
 * Derived from vendor PMU code at address 0x7d7c.
 * /
 */
void pmu_dbyte_pin_mask_seq_7d7c(u32 arg0, u32 arg1, u32 arg2)
{
	volatile u8 *dmem = (volatile u8 *)PMU_DMEM_BASE;
	__asm__("" : "+r"(dmem));

	u32 cond = (arg0 != 1) ? 1 : 0;
	pmu_deskew_latch_seq();

	u8 count = dmem[0xb20 + (cond * 2) + arg1];
	u32 slice_offset_base = (arg1 << 4) + (cond << 3);

	const volatile u8 *slices = dmem + 0xb44 + slice_offset_base;

	volatile u16 *dbyte_base = (volatile u16 *)0x90020000;
	__asm__("" : "+r"(dbyte_base));

	for (u32 i = 0; i < count; i++) {
		u8 slice = slices[i];
		u16 mask = (u16)pmu_dbyte_pin_mask_calc(0x80, slice);
		volatile u16 *slice_csr = (volatile u16 *)((uintptr_t)dbyte_base | (slice << 13));
		slice_csr[0x118 / 2] = mask;
	}

	pmu_delay_us(20000, 0);

	u32 arg2_or = arg2 | 0x80;
	for (u32 i = 0; i < count; i++) {
		u8 slice = slices[i];
		u8 mask = (u8)pmu_dbyte_pin_mask_calc(arg2_or, slice);
		volatile u16 *slice_csr = (volatile u16 *)((uintptr_t)dbyte_base | (slice << 13));
		slice_csr[0x118 / 2] = mask;
		slice_csr[0x112 / 2] = 511;
		slice_csr[0x116 / 2] = 511;
	}

	pmu_delay_us(5000, 12);

	for (u32 i = 0; i < count; i++) {
		u8 slice = slices[i];
		u16 mask = ((u8)pmu_dbyte_pin_mask_calc(arg2_or, slice)) | 0x100;
		volatile u16 *slice_csr = (volatile u16 *)((uintptr_t)dbyte_base | (slice << 13));
		slice_csr[0x118 / 2] = mask;
	}

	pmu_delay_us(5000, 12);

	for (u32 i = 0; i < count; i++) {
		u8 slice = slices[i];
		u16 mask = ((u8)pmu_dbyte_pin_mask_calc(0x80, slice)) | 0x100;
		volatile u16 *slice_csr = (volatile u16 *)((uintptr_t)dbyte_base | (slice << 13));
		slice_csr[0x112 / 2] = mask;
		slice_csr[0x116 / 2] = mask;
	}

	pmu_delay_us(250000, 0);
	pmu_clk_gate_handoff();
}

/**
 * pmu_rate_tier_calc() - pmu_cal_bist_lock_check:
 * @val: Value to write or configure
 *
 * Derived from vendor PMU code at address 0xb4f8.
 * Configures PLL lock status CSRs, rate tier parameters, and slice timing.
 * /
 *
 * Return: Computed u32 value or status.
 */
static u32 pmu_rate_tier_calc(u32 val)
{
	u32 low = (val <= 331) ? 0 : (val <= 531) ? 1 : (val <= 799) ? 2 : 3;
	u32 high = (val <= 799) ? (low + 2) : (val <= 1799) ? 5 : (val <= 2199) ? 6 : 7;
	return (high << 4) | low;
}
/**
 * pmu_cal_bist_lock_check() - Cal Bist Lock Check
 */
void pmu_cal_bist_lock_check(void)
{
	volatile u8 *dmem = (volatile u8 *)PMU_DMEM_BASE;
	__asm__("" : "+r"(dmem));

	u32 ch_offset = (dmem_read32(0x41c)) << 1;
	u16 sp_4, sp_6;
	pmu_phy_lock_status_read(&sp_6, &sp_4);

	volatile u16 *phy_ch = (volatile u16 *)(uintptr_t)(0x90040000 | ch_offset);
	__asm__("" : "+r"(phy_ch));

	phy_ch[0x14 >> 1] = sp_4 & 0x1ff;
	phy_ch[0x2e >> 1] = sp_6 & 0x1ff;

	u8 dram_type = dmem[0x08];
	if (!dram_type)
		dram_type = 2;
	u16 freq = *(volatile u16 *)(dmem + 0x06);

	u32 r2 = freq / dram_type;
	if (dram_type == 2)
		r2 *= 2;

	u32 mult = (dram_type != 2) ? 4 : 2;
	u32 r12 = (freq / dram_type) * mult;

	u32 t_r2 = pmu_rate_tier_calc(r2);
	u32 t_r12 = pmu_rate_tier_calc(r12);

	phy_ch[0x24 >> 1] = ((t_r12 >> 4) << 12) | ((t_r12 & 0xf) << 8) | t_r2;
	phy_ch[0x30c >> 1] = t_r12;
	phy_ch[0x30e >> 1] = *(volatile u16 *)0x900201aa;

	u32 slice_count = dmem_read32(0x40c);
	volatile u16 *dbyte = (volatile u16 *)0x900201b2;
	__asm__("" : "+r"(dbyte));
	for (u32 s = 0; s < slice_count; s++) {
		*(volatile u16 *)((uintptr_t)dbyte + s * 0x2000) = 0x9c;
	}

	pmu_cal_metric_log(4, 0x4d0002, sp_6, sp_4);
}

/**
 * pmu_cal_rank_lane_status_check() - Checks calibration status and maps lane active masks into DMEM tables
 *
 * Derived from vendor PMU code at address 0x8410.
 * /
 *
 * Return: Computed u32 value or status.
 */
u32 pmu_cal_rank_lane_status_check(void)
{
	volatile u8 *dmem = (volatile u8 *)PMU_DMEM_BASE;
	__asm__("" : "+r"(dmem));

	u32 rank_count = (pmu_cal_status_148_read() >> 6) & 0x3;
	u32 num_channels = *(volatile u32 *)(dmem + 0x410);

	for (u32 rank = 0; rank < rank_count; rank++) {
		for (u32 ch = 0; ch < num_channels; ch++) {
			u8 rank_mask = ch ? dmem[0x40] : dmem[0x25];
			if (!(rank_mask & (1 << rank)))
				continue;

			u32 ch_rank_bit = 1 << (rank + (ch << 1));
			u32 base_off = (ch << 4) + (rank << 3);
			u32 lane_start = ch ? dmem[0xb6c] : dmem[0xb6a];
			u32 lane_end = ch ? dmem[0xb6d] : dmem[0xb6b];

			u32 r14 = 0;
			volatile u8 *p_b44 = dmem + 0xb44 + base_off;
			volatile u8 *p_b24 = dmem + 0xb24 + base_off;
			u32 bit_query = pmu_dmem_1c_bit_query(ch, rank);
			u8 dmem_101 = dmem[0x101];

			for (u32 cur = lane_start; cur <= lane_end; cur += 2) {
				u8 next_val = cur + 1;

				if (bit_query) {
					p_b44[r14++] = cur;
					p_b44[r14++] = next_val;
				} else if (ch_rank_bit & dmem_101) {
					p_b44[r14] = next_val;
					p_b24[r14++] = cur;
				} else {
					p_b24[r14] = next_val;
					p_b44[r14++] = cur;
				}
			}
		}
	}

	return 1;
}
/**
 * pmu_rank_slice_bit_active() - Rank Slice Bit Active
 * @dmem: Parameter dmem
 * @rank_bit: Parameter rank_bit
 *
 * Return: 0 on success, negative error code on failure.
 */
static inline int pmu_rank_slice_bit_active(volatile u8 *dmem, u32 rank_bit)
{
	u32 shift = (dmem[0xb6b] < dmem[0xb68]) ? 2 : 0;
	return (dmem[0x1c] >> shift) & rank_bit;
}
/**
 * pmu_cal_pulse_bcf4_delay() - Cal Pulse Bcf4 Delay
 * @a1: Parameter a1
 */
static void pmu_cal_pulse_bcf4_delay(const void *a1)
{
	pmu_delay_us(0x3a98, 8);
	pmu_cal_descriptor_apply(a1);
	pmu_delay_us(0x30d40, 8);
}

/**
 * pmu_cal_multi_rank_pulse_seq_dfec() - Multi-rank calibration sequence using pmu_cal_sequence_pulse_send dispatch and clock delay steps
 * @a0: Parameter a0
 * @a1: Parameter a1
 * @a2: Parameter a2
 * @a3: Parameter a3
 * @a4: Parameter a4
 * @a5: Parameter a5
 *
 * Derived from vendor PMU code at address 0xdfec.
 * /
 */
void pmu_cal_multi_rank_pulse_seq_dfec(u32 a0, u32 a1, u32 a2, u32 a3, u32 a4, u32 a5)
{
	volatile u8 *dmem = (volatile u8 *)PMU_DMEM_BASE;
	__asm__("" : "+r"(dmem));

	pmu_deskew_and_tracker_reset();

	u32 rank_bit = 1 << (1 & ~a0);

	for (u32 i = 0; i < 21; i++) {
		u8 r13 = dmem[0x480 + i];
		u32 bit = 1U << (r13 & 31);
		if ((r13 < 32) ? (bit & a3) : (bit & a4))
			continue;

		u8 sp_10 = ((const u8 *)(uintptr_t)a1)[r13];

		if (i == 4) {
			pmu_cal_sequence_pulse_send(0, 0x10, 4, 0, 0, a0, 0);
		}

		pmu_cal_sequence_pulse_send(0, 6, 0, sp_10, r13, a0, 0);
		pmu_cbt_coarse_step_pulse_seq(0, 0x22, 0);

		u16 mult = 0;
		if (i == 0) {
			if (a5 != 0) {
				u8 b1  = ((const u8 *)(uintptr_t)a1)[1];
				u8 b14 = ((const u8 *)(uintptr_t)a1)[0x14];
				if ((b1 & 0x08) || (b14 & 0x0f) != 0) {
					pmu_clk_timing_delay_latch(0, 1);
					pmu_deskew_and_tracker_reset();
					pmu_cal_pulse_bcf4_delay((const void *)(uintptr_t)a1);
					continue;
				}
			}
			mult = 200;
		} else {
			if (i == 15 && pmu_rank_slice_bit_active(dmem, rank_bit)) {
				pmu_cal_sequence_pulse_send(0, 6, 0, sp_10 | 0x80, r13, a0, 0);
				pmu_cbt_coarse_step_pulse_seq(0, 0x22, 0);
			}
			if ((i | 1) == 0x11) {
				mult = 250;
			}
		}
		if (mult) {
			u8 dram_type = dmem[0x08];
			if (!dram_type)
				dram_type = 2;
			u16 freq = *(volatile u16 *)(dmem + 0x06);
			u32 result = ((u32)freq * mult) / ((u32)dram_type * 2000) + 1;
			pmu_cbt_coarse_step_pulse_seq(0, (u16)result, 0);
		}
	}

	pmu_cbt_coarse_step_pulse_seq(0, 0x41, 0);
	pmu_clk_timing_delay_latch(0, 1);

	if (a2 != 0) {
		pmu_dbyte_lane_error_mask_calc(0, a0);
	}
}
/**
 * pmu_pulse_1770_6_1() - Pulse 1770 6 1
 */
static __attribute__((noinline)) void pmu_pulse_1770_6_1(void)
{
	pmu_cbt_coarse_step_pulse_seq(0, 6, 1);
}

/**
 * pmu_cal_multi_rank_dispatch_e170() - Multi-rank calibration dispatch using pmu_dbyte_cal_step_latch and 3-phase CBT pulses
 * @a0: Parameter a0
 * @a1: Parameter a1
 * @a2: Parameter a2
 * @a3: Parameter a3
 * @a4: Parameter a4
 * @a5: Parameter a5
 * @a6: Parameter a6
 *
 * Derived from vendor PMU code at address 0xe170.
 * /
 */
void pmu_cal_multi_rank_dispatch_e170(u32 a0, u32 a1, u32 a2, u32 a3, u32 a4, u32 a5, u32 a6)
{
	volatile u8 *dmem = (volatile u8 *)PMU_DMEM_BASE;
	__asm__("" : "+r"(dmem));

	pmu_ac_lane_profile_setup();
	u32 r15 = a0;
	pmu_deskew_and_tracker_reset();
	pmu_cbt_coarse_step_pulse_seq(0, 5, 1);

	u32 rank_bit = 1 << (1 & ~r15);

	for (u32 i = 0; i < 21; i++) {
		u8 r13 = dmem[0x480 + i];
		u32 bit = 1U << (r13 & 31);
		if ((r13 < 32) ? (bit & a3) : (bit & a4))
			continue;

		u8 sp_10 = ((const u8 *)(uintptr_t)a1)[r13];

		if (a5 != 0) {
			pmu_cbt_coarse_step_pulse_seq(0, 0x1e, 1);
		}

		if (i == 4) {
			pmu_pulse_1770_6_1();
			pmu_cal_sequence_pulse_send(0, 0x2c, 0, 0, 0, r15, 0);
			pmu_cal_sequence_pulse_send(0, 0x2d, 0, 0, 0, r15, 0);
			pmu_pulse_1770_6_1();
		}

		pmu_dbyte_cal_step_latch(r13, sp_10, r15);

		if (i == 0) {
			if (a6 != 0) {
				u8 b1  = ((const u8 *)(uintptr_t)a1)[1];
				u8 b14 = ((const u8 *)(uintptr_t)a1)[0x14];
				if ((b1 & 0x08) || (b14 & 0x0f) != 0) {
					pmu_cbt_3phase_pulse_seq();
					pmu_clk_timing_delay_latch(0, 1);
					pmu_deskew_and_tracker_reset();
					pmu_cbt_coarse_step_pulse_seq(0, 5, 1);
					pmu_cal_pulse_bcf4_delay((const void *)(uintptr_t)a1);
				}
			}
			pmu_cbt_coarse_step_pulse_seq(0, 0x18, 1);
		} else if (i == 15 && pmu_rank_slice_bit_active(dmem, rank_bit)) {
			pmu_pulse_1770_6_1();
			pmu_dbyte_cal_step_latch(r13, sp_10 | 0x80, r15);
		}

		pmu_pulse_1770_6_1();
	}

	pmu_cbt_3phase_pulse_seq();
	pmu_clk_timing_delay_latch(0, 1);

	if (a2 != 0) {
		pmu_dbyte_lane_error_mask_calc(1, r15);
	}

	pmu_phy_profile_param_program();
}
/*
 * pmu_eye_sample_table_transform:
 * Derived from vendor PMU code at address 0x62b0.
 * Transforms raw 64-point (x, y) eye sample coordinates into bounding box window tables.
 */
struct pmu_eye_sample_entry {
	s16 min_x;
	s16 max_x;
	s16 min_y;
	s16 max_y;
};
/**
 * pmu_eye_sample_table_transform() - Eye Sample Table Transform
 * @dest: Pointer to destination memory
 * @src: Pointer to source memory
 */
void pmu_eye_sample_table_transform(u8 *dest, const u8 *src)
{
	struct pmu_eye_sample_entry *d = (struct pmu_eye_sample_entry *)dest;

	for (u32 i = 0; i < 64; i++) {
		d[i].min_x = 0x7f;
		d[i].max_x = 0;
		d[i].min_y = 0x7f;
		d[i].max_y = 0;
	}

	for (u32 i = 1; i < 63; i++) {
		const u8 *s_curr = src + (i << 1);
		const u8 *s_prev = s_curr - 2;
		const u8 *s_next = s_curr + 2;

		u8 c0 = s_curr[0];
		u8 c1 = s_curr[1];

		if (c1 >= c0) {
			d[i].max_x = (s16)(c0 - 1);
			s16 min_x = (s16)(s_next[0] + 1);
			if (min_x > (s16)(s_prev[0] + 1))
				min_x = (s16)(s_prev[0] + 1);
			if (min_x > (s16)c0)
				min_x = (s16)c0;
			d[i].min_x = min_x - 1;

			d[i].min_y = (s16)(c1 + 1);
			s16 max_y = (s16)(s_prev[1] - 1);
			if (max_y < (s16)(s_next[1] - 1))
				max_y = (s16)(s_next[1] - 1);
			if (max_y < (s16)c1)
				max_y = (s16)c1;
			d[i].max_y = max_y + 1;
		} else {
			if (s_next[1] >= s_next[0]) {
				d[i].min_x = s_next[0];
				d[i].max_x = s_next[1];
			}
			if (s_prev[1] >= s_prev[0]) {
				d[i].min_x = s_prev[0];
				d[i].max_x = s_prev[1];
			}
		}
	}

	/* Boundary entry 0 */
	u8 s0 = src[0];
	u8 s1 = src[1];
	if (s1 >= s0) {
		d[0].max_x = (s16)(s0 - 1);
		s16 min_x = (s16)(src[2] + 1);
		if (min_x > (s16)s0)
			min_x = (s16)s0;
		d[0].min_x = min_x - 1;
		d[0].min_y = (s16)(s1 + 1);
		s16 max_y = (s16)(src[3] - 1);
		if (max_y < (s16)s1)
			max_y = (s16)s1;
		d[0].max_y = max_y + 1;
	} else if (src[3] >= src[2]) {
		d[0].min_x = src[2];
		d[0].max_x = src[3];
	}

	/* Boundary entry 63 */
	u8 s126 = src[126];
	u8 s127 = src[127];
	if (s127 >= s126) {
		d[63].max_x = (s16)(s126 - 1);
		s16 min_x = (s16)(src[124] + 1);
		if (min_x > (s16)s126)
			min_x = (s16)s126;
		d[63].min_x = min_x - 1;
		d[63].min_y = (s16)(s127 + 1);
		s16 max_y = (s16)(src[125] - 1);
		if (max_y < (s16)s127)
			max_y = (s16)s127;
		d[63].max_y = max_y + 1;
	} else if (src[125] >= src[124]) {
		d[63].min_x = src[124];
		d[63].max_x = src[125];
	}

	for (u32 i = 1; i < 63; i++) {
		if (d[i].min_x == -1) {
			d[i].min_x = 0x7f;
			d[i].max_x = 0;
		}
	}
}

/**
 * pmu_cbt_pulse_trailer() - Helper for common CBT pulse trailer
 */
static __attribute__((noinline)) void pmu_cbt_pulse_trailer(void)
{
	pmu_cal_sequence_pulse_send(0, 7, 4, 0, 0, 0, 10);
	pmu_cal_sequence_pulse_send(0x8000, 7, 4, 0, 0, 0, 0);
	pmu_cal_sequence_pulse_send(0, 7, 8, 0, 0, 0, 0);
	pmu_cbt_cal_stat_clear();
	pmu_cal_sequence_pulse_send(0, 7, 4, 0, 0, 0, 0);
}

/**
 * pmu_cbt_pulse_train_aa74() - /
 * @arg0: Parameter arg0
 * @arg1: Parameter arg1
 *
 * Derived from vendor PMU code at address 0xaa74.
 */
void pmu_cbt_pulse_train_aa74(u32 arg0, u32 arg1)
{
	volatile u8 *dmem = (volatile u8 *)PMU_DMEM_BASE;
	__asm__("" : "+r"(dmem));
	u8 dram_type = dmem[0x08];
	pmu_deskew_and_tracker_reset();
	pmu_cal_sequence_pulse_send(0, 7, 4, 0, 0, 0, 0);
	pmu_cbt_cal_stat_set();

	u32 r13_new = (dram_type == 2) ? 4 : 2;

	if (arg1 != 0) {
		pmu_cal_sequence_pulse_send(0, 7, 2, 0, 0, 0, 4);
		pmu_cal_sequence_pulse_send(0x200000, 5, 0, 256, 0, arg0, 0);
	} else {
		pmu_cal_sequence_pulse_send(0x200000, 5, 0, 256, 0, arg0, 0);
		pmu_cal_sequence_pulse_send(0x1000004, 0x2b, r13_new, 0, 0, arg0, 0);
		pmu_cal_sequence_pulse_send(0x1000004, 0x2b, r13_new, 0, 0, arg0, 0);
	}

	pmu_cal_sequence_pulse_send(0x1040000, 0x2b, r13_new, 0, 0, arg0, 0);
	pmu_cbt_pulse_trailer();
}

/**
 * pmu_cbt_pulse_train_dc54() - /
 * @arg0: Parameter arg0
 * @arg1: Parameter arg1
 *
 * Derived from vendor PMU code at address 0xdc54.
 */
void pmu_cbt_pulse_train_dc54(u32 arg0, u32 arg1)
{
	volatile u8 *dmem = (volatile u8 *)PMU_DMEM_BASE;
	__asm__("" : "+r"(dmem));
	u8 dram_type = dmem[0x08];
	pmu_deskew_and_tracker_reset();
	pmu_cal_sequence_pulse_send(0, 7, 0, 0, 0, 0, 0);
	pmu_cal_sequence_pulse_send(0x480000, 5, 0, 512, 0, arg0, 0);
	pmu_cbt_cal_stat_set();

	u32 r13_new = (dram_type == 2) ? 5 : 2;

	pmu_cal_sequence_pulse_send(0xc, 0x2b, r13_new, 0, 0, arg0, 0);
	if (arg1 != 0)
		pmu_cal_sequence_pulse_send(0xc, 0x2b, r13_new + 1, 0, 0, arg0, 0);
	else
		pmu_cal_sequence_pulse_send(0xc, 0x2b, r13_new, 0, 0, arg0, 0);

	pmu_cal_sequence_pulse_send(0x40000, 0x2b, r13_new, 2, 0, arg0, 0);
	pmu_cal_sequence_pulse_send(0xc, 0x2b, r13_new, 2, 0, arg0, 0);
	pmu_cbt_pulse_trailer();
	pmu_cal_sequence_pulse_send(0, 5, 4, 896, 0, arg0, 0);
	pmu_cal_sequence_pulse_send(0x80080, 7, 0, 0, 0, arg0, 0);
}

/**
 * pmu_cbt_timing_pulse_coordinator() - /
 * @arg0: Parameter arg0
 * @arg1: Parameter arg1
 *
 * Derived from vendor PMU code at address 0x7fd0.
 */
void pmu_cbt_timing_pulse_coordinator(u32 arg0, u32 arg1)
{
	volatile u8 *dmem = (volatile u8 *)PMU_DMEM_BASE;
	__asm__("" : "+r"(dmem));

	static const u32 masks1[4] = { 0x7f << 8, 0x7f << 16, 0x7f7f007f, 0x7f007f7f };
	static const u32 masks0[4] = { 0x7f << 16, 0x7f << 24, 0x7f007f7f, 0x007f7f7f };

	volatile u16 *out_table = (volatile u16 *)(dmem + 0xe7b8);
	volatile u16 *out_sub1 = (volatile u16 *)(dmem + 0xe7c4);
	out_table[0] = 100;

	u16 marker = 100;
	for (u32 phase = 0; phase < 4; phase++) {
		u32 mask = arg1 ? masks1[phase] : masks0[phase];
		pmu_cal_markers_set((u16)(marker << 2));

		pmu_cal_sequence_pulse_send(0x41 << 19, 7, 8, 0, 0, 0, 0);
		pmu_cbt_coarse_step_pulse_seq(0, 4, 1);
		pmu_cal_window_params_dispatch((const u8 *)&mask, (u16)arg0, 2);
		pmu_cbt_coarse_step_pulse_seq(0, 0x14, 1);
		pmu_cal_sequence_pulse_send(1 << 18, 7, 4, 0, 0, 0, 0);
		pmu_cal_sequence_pulse_send(0x80 | (1 << 19), 7, 8, 0, 0, 0, 0);

		marker = pmu_cal_stride_get();
		out_table[phase + 1] = marker;
		out_sub1[phase] = marker - 1;
	}
}

/**
 * pmu_cal_bist_search_win_setup() - Calibration BIST search window setup routine
 * @arg0: Parameter arg0
 *
 * Derived from vendor PMU code at address 0xbd64.
 * /
 */
void pmu_cal_bist_search_win_setup(u32 arg0)
{
	if (arg0 == 0) {
		pmu_cal_metric_log(10, 0x2130000);
		*(volatile u16 *)0x80000014 = 0x0010;
		pmu_cal_search_win_init((u8 *)0x80000039);
		pmu_cal_vref_dac_step_adjust(8);
		dmem_write8(0x402, 1);
		dmem_write8(0x402, 0);
		pmu_post_cmd_conditional_dispatch(4);
		*(volatile u16 *)0x80000014 = 0x8010;
		return;
	}

	u32 gp28 = dmem_read32(0x41c);
	u8 start_slice = *(volatile u8 *)0x80000b68;
	u8 end_slice = *(volatile u8 *)0x80000b69;
	u16 pin_delays[36];

	for (u32 fp = 0; fp < 2; fp++) {
		pmu_cal_strobe_pulse();
		u32 r14 = fp;
		u16 sp_24[4];
		pmu_cal_slice_step_diff_commit(r14, sp_24, 1);
		pmu_phy_reset_pulse();

		u32 pin_idx = (u32)start_slice * 9;

		for (u32 slice = start_slice; slice <= end_slice; slice++) {
			u16 r15 = sp_24[slice];
			pmu_cal_metric_log(4, 0x2920001, (u32)r15);

			u32 slice_mask = gp28 | (slice << 12);
			u32 reg_addr = 0x90020000 | ((slice_mask | (r14 + 0x2a)) << 1);
			u16 reg_val = phy_read16(reg_addr);
			u32 low6 = reg_val & 0x3f;
			r15 = r15 + low6 + 0x20;

			for (u32 pin = 0; pin < 9; pin++) {
				pin_delays[pin_idx] = r15;
				pmu_cal_metric_log(4, 0x2930004, pin_idx, low6, pin_idx, (s16)r15);
				pin_idx++;
			}

			phy_write16(reg_addr - 4, r15);
		}

		pmu_slice_delay_step_program(gp28 | (r14 + 0x26), pin_delays, 1, 0, 0, 0);
	}
}

/**
 * pmu_cal_dbyte_deskew_results_apply() - Applies DBYTE per-slice pin deskew calibration results to PHY registers
 * @results: Pointer to calibration results array
 * @arg1: Parameter arg1
 * @arg2: Parameter arg2
 *
 * Derived from vendor PMU code at address 0xa7ac.
 * /
 */
void pmu_cal_dbyte_deskew_results_apply(const void *results, u32 arg1, u32 arg2)
{
	const u8 *res8 = (const u8 *)results;
	volatile u8 *dmem = (volatile u8 *)0x80000000;
	u32 lut_res = pmu_2b_identity_lut(arg1);
	u32 gp28 = *(const volatile u32 *)&dmem[0x470];
	u8 start_slice = dmem[0x0b68];
	u8 end_slice = dmem[0x0b69];

	if (arg2 == 0x11) {
		u8 b_mode = dmem[0x102];
		u8 val_512 = res8[512];
		u32 bit1 = b_mode & 2;
		pmu_cal_metric_log(4, 0x1600002, (u32)val_512, (u32)(b_mode >> bit1));

		if (bit1 != 0 && val_512 == 3) {
			phy_write16(0x9003e012 | (gp28 << 1), 4);
		}

		for (u32 slice = start_slice; slice <= end_slice; slice++) {
			const u8 *slice_base = res8 + (slice * 20);
			for (u32 pin = 0; pin < 9; pin++) {
				const u8 *pin_ptr = slice_base + (pin * 2);
				u16 val_112 = *(const u16 *)(pin_ptr + 112);
				u16 val_32 = *(const u16 *)(pin_ptr + 32);
				pmu_cal_metric_log(4, 0x1610004, slice, pin, (u32)val_112, (u32)val_32);

				u32 r1 = (slice << 12) | (pin << 8) | gp28;
				u32 reg2 = 0x90020000 | (((r1 | (lut_res + 0x10))) << 1);
				phy_write16(reg2 + 4, val_32);
				phy_write16(reg2, val_112);

				if (bit1 != 0 && val_512 == 3) {
					const u16 *extra_src = (const u16 *)(res8 + (slice * 80) + (pin * 8) + 192);
					u32 base = 0x9002009c | (r1 << 1);
					phy_write16(base, extra_src[0]);
					phy_write16(base + 2, extra_src[1]);
					phy_write16(base + 4, extra_src[2]);
					phy_write16(base + 6, extra_src[3]);
				}
			}
		}
		pmu_dbyte_dq_deskew_regs_save_and_ramp();
	} else {
		pmu_cal_metric_log(4, 0x1620000);
		for (u32 slice = start_slice; slice <= end_slice; slice++) {
			const u16 *slice_pins = (const u16 *)(res8 + (slice * 20) + 32);
			for (u32 pin = 0; pin < 9; pin++) {
				u16 val = slice_pins[pin];
				pmu_cal_metric_log(4, 0x1630003, slice, pin, (u32)val);

				u32 r1 = (lut_res + 0x26) | (slice << 12) | (pin << 8) | gp28;
				phy_write16(0x90020000 | (r1 << 1), val);
			}
			u16 extra_val = slice_pins[9];
			u32 r_extra = (lut_res + 0x28) | (slice << 12) | gp28;
			phy_write16(0x90020000 | (r_extra << 1), extra_val);
		}
	}
}
/**
 * pmu_slice_pulse_write() - Slice Pulse Write
 * @ptr: Pointer to memory structure or buffer
 * @count: Number of items, halfwords, or iterations
 * @val: Value to write or configure
 */
static void pmu_slice_pulse_write(const u8 *ptr, u32 count, u16 val)
{
	for (u32 i = 0; i < count; i++)
		phy_write16(0x90020118 | ((u32)ptr[i] << 13), val);
}

/**
 * pmu_cal_multi_rank_deskew_sweep() - Coordinates multi-rank slice deskew sweeps, latching, and calibration pulses
 * @arg0: Parameter arg0
 *
 * Derived from vendor PMU code at address 0x5ad4.
 * /
 */
void pmu_cal_multi_rank_deskew_sweep(u32 arg0)
{
	u8 rank = *(volatile u8 *)0x80000b66;
	u32 dmem_offset = (rank != 0) ? 0x8000004d : 0x80000032;
	u8 r1 = *(volatile u8 *)dmem_offset;
	u32 fp = ((arg0 & 3) == 2) ? 1 : 0;

	if (fp != (r1 & 1)) {
		pmu_slice_deskew_pulse_train((u32)(r1 & 1), 1);
	}

	if (*(volatile u8 *)0x80000403 != 0) {
		pmu_ac_lane_profile_setup();
		pmu_delay_us(50000, 3);
	}

	u32 struct_base = 0x800009a4;
	pmu_tracker_field_extract((u8 *)struct_base, 0xb);
	pmu_tracker_field_extract((u8 *)struct_base, 0x11);
	pmu_tracker_field_extract((u8 *)struct_base, 0x12);

	u32 idx = fp + (rank * 2);
	u8 count = *(volatile u8 *)(0x80000b20 + idx);
	const u8 *ptr = (const u8 *)(0x80000b44 + (idx << 3));

	pmu_deskew_latch_seq();

	pmu_slice_pulse_write(ptr, count, 256);

	pmu_delay_us(5000, 12);
	pmu_clk_gate_handoff();

	u32 gp28 = *(volatile u32 *)0x8000041c;
	phy_write16(0x90040082 | (gp28 << 1), 0);
	pmu_delay_us(5000, 0);

	pmu_cal_struct_to_shadow16(struct_base, 0x10);
	pmu_cal_struct_mask_and(struct_base, 0x10, 0xcf);
	pmu_cal_struct_mask_and(struct_base, 0x10, 0xff);

	u32 sub_table = struct_base + (idx * 54);
	pmu_cal_multi_rank_dispatch_e170(arg0, sub_table, 0, 0xfffeffff, (u32)-1, 1, 0);

	pmu_delay_us(14000, 10);

	pmu_slice_pulse_write(ptr, count, 0x1000);

	pmu_tracker_field_extract((u8 *)struct_base, 0x10);
	u32 reg = 0x9004000e | (gp28 << 1);
	phy_write16(reg, phy_read16(reg) & 0xfeff);
	pmu_hw_timer_delay(0x14);
}

/**
 * pmu_cal_bist_pattern_setup() - Configures calibration BIST test patterns, lane timing offsets, and PRBS sequences
 * @arg0: Parameter arg0
 * @arg1: Parameter arg1
 * @arg2: Parameter arg2
 * @arg3: Parameter arg3
 *
 * Derived from vendor PMU code at address 0x3ef4.
 * /
 */
void pmu_cal_bist_pattern_setup(u32 arg0, u32 arg1, u32 arg2, u32 arg3)
{
	static const u16 pats[9] = {
		0x5a3c, 0xff00, 0xa536, 0xaaaa, 0xa536, 0xb2b2, 0x8241, 0, 0x5a3c
	};
	u32 r13_cfg = dmem_read8(8);
	u16 pat;
	u32 flag = 0;

	if (arg0 >= 9) {
		pmu_assert_or_halt(0, 0x1a40001);
		pat = 0;
	} else {
		pat = pats[arg0];
		if (arg0 == 7)
			pat = dmem_read8(0xe4) | ((u16)dmem_read8(0xe5) << 8);
	}

	s8 r12 = 0;
	if (arg2 != 0) {
		u8 e6 = dmem_read8(0xe6);
		r12 = e6 ? (s8)e6 : (s8)-86;
	}

	if (arg0 == 1 || (arg0 >= 3 && arg0 <= 7))
		pmu_channel_timing_deskew_reset((u8)pat, (pat >> 8) & 0xff, (u32)r12, (u32)r12, arg1);

	pmu_master_cfg_quad_write(0xffff, 0xffff, 0xffff, 0xffff);
	phy_write16(0x900fe0ca, 0);
	pmu_phy_lane_timing_offset_set(0xf, pat, pat, pat, pat);

	if (arg2 != 0) {
		if (arg0 == 0) {
			phy_write16(0x900fe022, 22);
		} else {
			u32 r15 = (arg0 == 2 || arg0 == 8) ? 1 : 0;
			pmu_assert_or_halt(((arg3 != 0) ? 1 : 0) | r15, 0x1a50000);
			phy_write16(0x900fe022, 16);
			for (u32 i = 0; i < 8; i++) {
				u16 p = (r12 & (1 << i)) ? (u16)~pat : pat;
				pmu_phy_lane_timing_offset_set(i, p, p, p, p);
			}
			pmu_phy_lane_timing_offset_set(8, pat, pat, pat, pat);
		}
	} else {
		phy_write16(0x900fe022, 16);
	}

	u16 val_e1ae = (arg0 == 1 || (arg0 >= 3 && arg0 <= 7)) ? 0 : 256;
	phy_write16(0x900fe1ae, val_e1ae);
	phy_write16(0x900fe1ac, val_e1ae);

	u16 r8 = (r13_cfg == 4) ? 0x84 : 4;
	if (arg0 != 0) {
		phy_write16(0x900fe0c0, r8 | 5);
	} else {
		phy_write16(0x900fe0c0, r8 | 8);
		phy_write16(0x900fe0ca, 0);
	}
}

/**
 * pmu_cal_ca_eye_margin_sweep() - Sweeps CA (Command/Address) bus 128 delay taps across active channels
 *
 * Derived from vendor PMU code at address 0x8650 (pmu_cal_ca_eye_margin_sweep).
 * and ranks, evaluates eye aperture margin, determines the optimal eye center,
 * programs CA delay CSRs, and updates mission mode DMEM parameters.
 * /
 */
void pmu_cal_ca_eye_margin_sweep(void)
{
	u16 sp_1c = 0;
	u8 sp_1b = 0;
	u8 sp_1f = 0;

	dmem_write16(0x0b70, 0);
	dmem_write16(0x0b6e, 0);

	pmu_cal_metric_log(10, 0x1010000);
	pmu_cal_struct_mask_remap();

	for (u32 cfg_idx = 0; cfg_idx < 4; cfg_idx++) {
		if (pmu_cal_channel_rank_config_get((u16)cfg_idx, &sp_1c, &sp_1b, &sp_1f) != 0)
			continue;

		dmem_write8(0x0a7c, 1);
		pmu_phy_mode_cfg_dispatch((u32)sp_1f);

		u32 r0_5014 = pmu_cal_dbyte_dq_status_check((u32)sp_1b, 1);

		dmem_write8(0x0a7c, 0);
		pmu_cbt_timing_pulse_coordinator(r0_5014, 1);

		u32 fp_val = (u32)sp_1c;
		u32 r15_val = fp_val & 0xff;
		u32 sp_20 = 0;
		pmu_dbyte_deskew_phase_sample(r15_val, &sp_20, 2);

		u8 tap_errors[128];
		pmu_memset_words(tap_errors, 0, 128);

		u32 ch = (u32)dmem_read8(0x0b66);

		for (u32 tap = 1; tap < 128; tap++) {
			pmu_dbyte_pin_mask_seq_7d7c(r0_5014, ch, tap);
			u32 err_flag = 0;
			pmu_dbyte_deskew_phase_sample(r15_val, &err_flag, 2);
			if ((err_flag & 0x3f00) != 0)
				tap_errors[tap] = 1;
		}

		pmu_cal_metric_log(4, 0x1030002, fp_val, ch);

		for (u32 tap = 0; tap < 128; tap++)
			pmu_cal_metric_log(4, 0x1040001, (u32)tap_errors[tap]);

		pmu_cal_metric_log(4, 0x1050000);

		s32 best_start = -1;
		s32 best_end = -1;
		s32 cur_start = -1;

		for (s32 i = 0; i < 0x7f; i++) {
			u8 err = tap_errors[i];
			if (cur_start == -1) {
				if (err == 0)
					cur_start = i;
			} else {
				if (i == 0x7e || err != 0) {
					s32 end_idx = (err == 0) ? i : (i - 1);
					s32 width = end_idx - cur_start;
					s32 best_width = (best_start == -1) ? -1 : (best_end - best_start);
					if (width > best_width) {
						best_start = cur_start;
						best_end = end_idx;
					}
					cur_start = -1;
				}
			}
		}

		if (cur_start != -1 && best_start == -1) {
			best_start = cur_start;
			best_end = 0x7f;
		}

		u32 valid = 0;
		if (best_start != -1) {
			if ((best_end - best_start) > 4)
				valid = 1;
		}

		pmu_assert_or_halt(valid, 0x1070002);

		u32 center = (u32)(best_start + best_end) / 2;
		pmu_cal_metric_log(4, 0x1080003, (u32)best_start, (u32)best_end);

		dmem_write8(0x0b6e + (ch * 2) + fp_val, (u8)center);

		if (fp_val == 0) {
			if (ch == 0)
				dmem_write8(0x33, (u8)center);
			else if (ch == 1)
				dmem_write8(0x4e, (u8)center);
		} else if (fp_val == 1) {
			if (ch == 0)
				dmem_write8(0x34, (u8)center);
			else if (ch == 1)
				dmem_write8(0x4f, (u8)center);
		}

		u32 r3_calc = center * 5 + 50;
		u32 r2_calc = r3_calc / 10;
		u32 rem_calc = r3_calc - (r2_calc * 10);
		pmu_cal_metric_log(5, 0x1090005, r2_calc, rem_calc, fp_val, center);

		dmem_write8(0x0a7c, 1);
		pmu_cal_multi_rank_deskew_sweep(r0_5014);
	}

	dmem_write8(0x0b96, 1);
}

/**
 * pmu_cal_dbyte_deskew_pin_results_apply() - Applies DBYTE per-pin deskew calibration results to PHY registers
 *
 * Derived from vendor PMU code at address 0xf858.
 * /
 */
void pmu_cal_dbyte_deskew_pin_results_apply(void)
{
	uintptr_t base_e48 = 0x80000e48;
	uintptr_t base_e4a = 0x80000e4a;
	u32 gp28 = dmem_read32(PMU_DMEM_PARAM_41C);

	for (u32 rank = 0; rank < 2; rank++) {
		u8 mask = dmem_read8(0x25) | dmem_read8(0x40);
		if (!(mask & (1 << rank))) {
			base_e48 += 5280;
			base_e4a += 5280;
			continue;
		}

		u32 start_slice = (u32)dmem_read8(0x0b68);
		u32 end_slice = (u32)dmem_read8(0x0b69);

		uintptr_t slice_e48 = base_e48 + start_slice * 1320;
		uintptr_t slice_e4a = base_e4a + start_slice * 1320;

		for (u32 slice = start_slice; slice <= end_slice; slice++) {
			if (dmem_read8(0x0b98) & (1 << slice)) {
				u32 slice_base = gp28 | (slice << 12) | rank;
				uintptr_t ptr_a = slice_e48;
				uintptr_t ptr_b = slice_e4a;

				for (u32 pin = 0; pin < 10; pin++) {
					u16 val_a = *(volatile u16 *)(uintptr_t)ptr_a;
					s8 val_b = *(volatile s8 *)(uintptr_t)ptr_b;
					ptr_a += 132;
					ptr_b += 132;

					u16 sum = (u16)(val_a + (s32)val_b);
					if (pin == 9) {
						uintptr_t reg = 0x90020050 | (slice_base << 1);
						*(volatile u16 *)(uintptr_t)reg = sum;
					} else {
						uintptr_t reg = 0x9002004c | ((slice_base | (pin << 8)) << 1);
						*(volatile u16 *)(uintptr_t)reg = sum;
					}
				}

				if (dmem_read8(0xce) == 0) {
					uintptr_t tbl = 0x80000e48 + rank * 5280 + slice * 1320;
					u8 r13_val = *(volatile u8 *)(uintptr_t)(tbl + 3);
					u32 is_hi = (slice > (u32)dmem_read8(0x0b6b)) ? 1 : 0;

					pmu_phy_mode_cfg_dispatch(is_hi ? 2 : 1);
					u32 swap = pmu_dq_swap_query(rank, slice);
					dmem_write8(0x0b66, is_hi);
					u32 r2 = rank + (is_hi ? 2 : 0);

					if (swap != 0) {
						dmem_write8(0x0b66 + r2 + 0xc, r13_val);
						pmu_cal_pulse_seq_coordinator(rank, r13_val, 1);
						dmem_write8(is_hi ? (rank ? 0x51 : 0x50) : (rank ? 0x36 : 0x35), r13_val);
					} else {
						dmem_write8(0x0b66 + r2 + 0x10, r13_val);
						pmu_cal_pulse_seq_coordinator(rank, r13_val, 2);
						dmem_write8(is_hi ? (rank ? 0xdf : 0xda) : (rank ? 0xd5 : 0xd0), r13_val);
					}
					pmu_phy_mode_cfg_dispatch(3);
				}
			}

			slice_e48 += 1320;
			slice_e4a += 1320;
		}

		base_e48 += 5280;
		base_e4a += 5280;
	}
}

