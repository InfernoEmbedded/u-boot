// SPDX-License-Identifier: GPL-2.0+
/*
 * Synopsys DesignWare DDR PHY Training Firmware (LPDDR5)
 * Command-Bus Training (CBT) pulse sequences and calibration tables
 * Target Microcontroller: Synopsys ARC EM4 (ARCv2 ISA, Code Density enabled)
 * SoC: Allwinner A733 (Sun60i) / LPDDR5 PHY (Type 9)
 */

#include "lpddr5_pmu_internal.h"

/**
 * pmu_cbt_entry_pll_ctrl() - Configure PLL and enter Command Bus Training mode
 *
 * Sets PHY_REG_CBT_PLL_CTRL to (PHY_REG_CBT_CTRL & ~3) | 1, waits, and
 * asserts PHY_REG_CBT_CTRL = 1.
 */
void pmu_cbt_entry_pll_ctrl(void)
{
	u16 val = phy_read16(PHY_REG_CBT_CTRL);
	val = (val & ~0x3) | 0x1;
	phy_write16(PHY_REG_CBT_PLL_CTRL, val);
	pmu_hw_timer_delay(4);
	pmu_hw_timer_delay(0x1f);
	phy_write16(PHY_REG_CBT_CTRL, 1);
}

/**
 * pmu_cbt_exit_mission_handoff() - Exit Command Bus Training and hand off to mission mode
 *
 * Clears PHY_REG_CBT_PLL_CTRL = 0 and restores PHY_REG_CBT_CTRL =
 * (val & ~3) | 2.
 */
void pmu_cbt_exit_mission_handoff(void)
{
	pmu_hw_timer_delay(0x1f);
	pmu_hw_timer_delay(4);
	phy_write16(PHY_REG_CBT_PLL_CTRL, 0);
	u16 val = phy_read16(PHY_REG_CBT_CTRL);
	val = (val & ~0x3) | 0x2;
	phy_write16(PHY_REG_CBT_CTRL, val);
}

/**
 * pmu_cbt_state_latch() - Record CBT configuration state and operating mode in DMEM
 * @val: CBT training state flags (phase and operating mode) to record in DMEM
 */
void pmu_cbt_state_latch(u8 val)
{
	dmem_write8(PMU_DMEM_CBT_STATE, 1);
	dmem_write8(PMU_DMEM_CBT_CONFIG, val);
}

/**
 * pmu_cbt_step_stat_set() - Set CBT calibration step active status flag in DMEM
 */
void pmu_cbt_step_stat_set(void)
{
	dmem_write8(PMU_DMEM_CBT_STEP_STATUS, 1);
}

/**
 * pmu_cbt_config_get() - Read CBT configuration state byte from DMEM
 *
 * Return: CBT configuration state byte from DMEM.
 */
u8 pmu_cbt_config_get(void)
{
	return dmem_read8(PMU_DMEM_CBT_CONFIG);
}

/**
 * pmu_cbt_cal_stat_clear() - Clear CBT calibration active status flag in DMEM
 */
void pmu_cbt_cal_stat_clear(void)
{
	dmem_write8(PMU_DMEM_CBT_CAL_STATUS, 0);
}

/**
 * pmu_cbt_cal_stat_set() - Set CBT calibration active status flag in DMEM
 */
void pmu_cbt_cal_stat_set(void)
{
	dmem_write8(PMU_DMEM_CBT_CAL_STATUS, 1);
}

/**
 * pmu_cbt_coarse_step_pulse() - Trigger 5-step CBT coarse phase pulse sequence
 */
void pmu_cbt_coarse_step_pulse(void)
{
	pmu_cbt_coarse_step_pulse_seq(0, 5, 1);
}

/**
 * pmu_cbt_3phase_pulse_seq() - Trigger standard 3-phase CBT pulse sequence
 *
 * Executes 16, 16, and 10 steps via pmu_cbt_coarse_step_pulse_seq.
 */
void pmu_cbt_3phase_pulse_seq(void)
{
	pmu_cbt_coarse_step_pulse_seq(0, 0x10, 1);
	pmu_cbt_coarse_step_pulse_seq(0x10, 0, 1);
	pmu_cbt_coarse_step_pulse_seq(0, 0x0a, 1);
}

/**
 * pmu_cbt_entry_lookup() - Query CBT table entry byte at specified DMEM base
 * @table_base: Base DMEM offset of the CBT lookup table
 *
 * Return: CBT configuration entry byte.
 */
u8 pmu_cbt_entry_lookup(u32 table_base)
{
	u8 channel = dmem_read8(PMU_DMEM_CAL_RANK);
	u8 slice = dmem_read8(PMU_DMEM_CAL_BYTE);
	u32 ptr = table_base + (channel * 108) + (slice * 54);
	u8 cfg = pmu_cbt_config_get();
	u8 val = *(volatile u8 *)(uintptr_t)(ptr + 2);
	u32 table_offset = (cfg != 2) ? 312 : 0;
	u32 entry_stride = (val & 0xf) * 26;
	return *(volatile u8 *)(uintptr_t)((PMU_DMEM_BASE | PMU_DMEM_CBT_LOOKUP_TABLE) + table_offset + entry_stride);
}

/**
 * pmu_cbt_active_entry_lookup() - Query CBT lookup entry byte for active channel and rank
 *
 * Return: CBT lookup entry byte from lookup table.
 */
u8 pmu_cbt_active_entry_lookup(void)
{
	return pmu_cbt_entry_lookup(dmem_read32(PMU_DMEM_CAL_STRUCT_PTR));
}

/**
 * pmu_cbt_coarse_step_pulse_seq() - Dispatch CBT coarse step pulse sequence across PHY channels
 * @bypass_min_clamp: If non-zero, skip clamping target steps against step size
 * @steps: Total number of CBT coarse phase steps to pulse
 * @use_fixed_unit: If non-zero, use unit step size; if 0, read step from CBT state
 */
void pmu_cbt_coarse_step_pulse_seq(u32 bypass_min_clamp, u32 steps, u32 use_fixed_unit)
{
	volatile u16 *marker_a = (volatile u16 *)(PMU_DMEM_BASE | PMU_DMEM_CAL_MARKER_A);
	u16 marker_val = *marker_a;
	u16 buf[8];
	memset(buf, 0, sizeof(buf));

	if ((marker_val & 0x7) == 4) {
		pmu_clear_tracker_words((void *)((PMU_DMEM_BASE | PMU_DMEM_CAL_TRACKER) + 16));
		pmu_cal_tracker_step_update();
		*marker_a = marker_val + 4;
	}

	u8 step_b = 1;
	if (use_fixed_unit == 0) {
		step_b = (u8)(*(volatile u8 *)(uintptr_t)(PMU_DMEM_BASE | PMU_DMEM_CBT_STATE) << 1);
	}
	u32 step = step_b;
	u32 val = step;
	if (bypass_min_clamp == 0) {
		if (steps > val)
			val = steps;
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
 * pmu_cbt_delay_offset_lookup() - Query CBT delay offset from DMEM lookup table
 *
 * Return: 0 on normal training completion.
 */
u32 pmu_cbt_delay_offset_lookup(void)
{
	u8 channel = dmem_read8(PMU_DMEM_CAL_RANK);
	u8 rank = dmem_read8(PMU_DMEM_CAL_BYTE);
	uintptr_t table_ptr = *(volatile u32 *)(uintptr_t)(PMU_DMEM_BASE | PMU_DMEM_CAL_STRUCT_PTR);
	const u8 *ptr = (const u8 *)(table_ptr + channel * 108 + rank * 54);

	u8 ptr1 = ptr[1];
	u8 cbt_cfg = pmu_cbt_config_get();

	u32 cfg_offset = (cbt_cfg != 2) ? 264 : 0;
	u32 entry_offset = (ptr1 >> 4) * 22;
	u32 base_idx = cfg_offset + entry_offset;

	u32 bit5 = (ptr[3] & (1 << 5)) ? 5 : 4;
	u32 total_idx = (base_idx * 2) + bit5;

	return *(volatile u8 *)(uintptr_t)((PMU_DMEM_BASE | PMU_DMEM_CBT_OFFSET_TABLE) + total_idx);
}

/**
 * pmu_cbt_cal_pulse_seq() - Execute Command Bus Training calibration pulse sequence
 * @pulse_cmd: Command bus pulse sequence control word
 */
void pmu_cbt_cal_pulse_seq(u32 pulse_cmd)
{
	pmu_cal_sequence_pulse_send(0x41 << 19, 7, 4, 0, 0, 0, 0);
	pmu_cbt_cal_stat_set();
	pmu_cal_sequence_pulse_send(pulse_cmd, 7, 4, 0, 0, 0, 0);
	pmu_cbt_cal_stat_clear();
	pmu_cal_sequence_pulse_send(pulse_cmd, 7, 4, 0, 0, 0, 0);
	pmu_cal_sequence_pulse_send(0x80 | (1u << 19), 7, 4, 0, 0, 0, 0);
}

/**
 * pmu_cbt_3phase_pulse_dispatch() - Dispatch Command Bus Training 3-phase calibration pulse sequence
 * @cal_mode: CBT calibration mode / rank selector
 * @step_table: Pointer to CBT calibration delay step table
 * @step_idx: Index into step table selecting target delay tap
 * @flags: Control flags (bit 1 suppresses AC lane profile setup)
 */
void pmu_cbt_3phase_pulse_dispatch(u32 cal_mode, const u8 *step_table, u32 step_idx, u32 flags)
{
	u8 val = step_table[step_idx];
	pmu_deskew_and_tracker_reset();

	if (!(flags & 2)) {
		pmu_ac_lane_profile_setup();
		pmu_cbt_coarse_step_pulse_seq(0, 5, 1);
	}

	pmu_dbyte_cal_step_latch(step_idx, val, cal_mode);

	if ((step_idx - 14) < 2) {
		pmu_cal_sequence_pulse_send(0, 7, 0x20, 0, 0, 0, 0);
	} else if (step_idx == 16) {
		pmu_cal_sequence_pulse_send(0, 7, 4, 0, 0, 1, 0);
		if (val & 0x20) {
			u32 arg = (dmem_read8((PMU_DMEM_BASE | PMU_DMEM_FREQ_MODE)) == 0) ? (1 << 19) : 0;
			pmu_cbt_cal_pulse_seq(arg);
			pmu_cbt_3phase_pulse_seq();
			pmu_clk_timing_delay_latch(0, 0);
			pmu_delay_us(0x1388, 0x14);
			if (!(flags & 1))
				pmu_phy_profile_param_program();
			return;
		}
	} else {
		pmu_cal_sequence_pulse_send(0, 7, 6, 0, 0, 0, 0);
	}

	pmu_cbt_3phase_pulse_seq();
	pmu_clk_timing_delay_latch(0, 1);
	if (!(flags & 1))
		pmu_phy_profile_param_program();
}

/**
 * cbt_def() - Send default Command Bus Training null pulse sequence
 *
 * Sends a default null pulse (type 7, phase 0) to clear CBT sequence state.
 */
void cbt_def(void)
{
	pmu_cal_sequence_pulse_send(0, 7, 0, 0, 0, 0, 0);
}

/**
 * pmu_cbt_pulse_trailer() - Execute common CBT pulse sequence completion trailer
 */
__attribute__((noinline)) void pmu_cbt_pulse_trailer(void)
{
	pmu_cal_sequence_pulse_send(0, 7, 4, 0, 0, 0, 10);
	pmu_cal_sequence_pulse_send(0x8000, 7, 4, 0, 0, 0, 0);
	pmu_cal_sequence_pulse_send(0, 7, 8, 0, 0, 0, 0);
	pmu_cbt_cal_stat_clear();
	pmu_cal_sequence_pulse_send(0, 7, 4, 0, 0, 0, 0);
}

/**
 * pmu_cbt_strobe_pulse_train() - Transmit command bus training pulse train sequence (stage 1)
 * @pulse_cmd: Command bus pulse sequence control word
 * @is_stage2: Non-zero to execute stage 2 strobe pulses
 */
void pmu_cbt_strobe_pulse_train(u32 pulse_cmd, u32 is_stage2)
{
	volatile u8 *dmem = (volatile u8 *)PMU_DMEM_BASE;
	__asm__("" : "+r"(dmem));
	u8 dram_type = dmem[0x08];
	pmu_deskew_and_tracker_reset();
	pmu_cal_sequence_pulse_send(0, 7, 4, 0, 0, 0, 0);
	pmu_cbt_cal_stat_set();

	u32 pulse_repeat_count = (dram_type == 2) ? 4 : 2;

	if (is_stage2 != 0) {
		pmu_cal_sequence_pulse_send(0, 7, 2, 0, 0, 0, 4);
		pmu_cal_sequence_pulse_send(0x200000, 5, 0, 256, 0, pulse_cmd, 0);
	} else {
		pmu_cal_sequence_pulse_send(0x200000, 5, 0, 256, 0, pulse_cmd, 0);
		pmu_cal_sequence_pulse_send(0x1000004, 0x2b, pulse_repeat_count, 0, 0, pulse_cmd, 0);
		pmu_cal_sequence_pulse_send(0x1000004, 0x2b, pulse_repeat_count, 0, 0, pulse_cmd, 0);
	}

	pmu_cal_sequence_pulse_send(0x1040000, 0x2b, pulse_repeat_count, 0, 0, pulse_cmd, 0);
	pmu_cbt_pulse_trailer();
}

/**
 * pmu_cbt_deskew_pulse_train() - Transmit command bus training pulse train sequence (stage 2)
 * @pulse_cmd: Command bus pulse sequence control word
 * @is_alternate: Non-zero to send alternate offset pulse
 */
void pmu_cbt_deskew_pulse_train(u32 pulse_cmd, u32 is_alternate)
{
	volatile u8 *dmem = (volatile u8 *)PMU_DMEM_BASE;
	__asm__("" : "+r"(dmem));
	u8 dram_type = dmem[0x08];
	pmu_deskew_and_tracker_reset();
	pmu_cal_sequence_pulse_send(0, 7, 0, 0, 0, 0, 0);
	pmu_cal_sequence_pulse_send(0x480000, 5, 0, 512, 0, pulse_cmd, 0);
	pmu_cbt_cal_stat_set();

	u32 pulse_repeat_count = (dram_type == 2) ? 5 : 2;

	pmu_cal_sequence_pulse_send(0xc, 0x2b, pulse_repeat_count, 0, 0, pulse_cmd, 0);
	if (is_alternate != 0)
		pmu_cal_sequence_pulse_send(0xc, 0x2b, pulse_repeat_count + 1, 0, 0, pulse_cmd, 0);
	else
		pmu_cal_sequence_pulse_send(0xc, 0x2b, pulse_repeat_count, 0, 0, pulse_cmd, 0);

	pmu_cal_sequence_pulse_send(0x40000, 0x2b, pulse_repeat_count, 2, 0, pulse_cmd, 0);
	pmu_cal_sequence_pulse_send(0xc, 0x2b, pulse_repeat_count, 2, 0, pulse_cmd, 0);
	pmu_cbt_pulse_trailer();
	pmu_cal_sequence_pulse_send(0, 5, 4, 896, 0, pulse_cmd, 0);
	pmu_cal_sequence_pulse_send(0x80080, 7, 0, 0, 0, pulse_cmd, 0);
}

/**
 * pmu_cbt_timing_pulse_coordinator() - Coordinate CBT timing pulse bitmasks and dispatch sequence
 * @cal_mode: CBT calibration mode / rank selector
 * @mask_set: Mask array selector (0 selects masks0, 1 selects masks1)
 */
void pmu_cbt_timing_pulse_coordinator(u32 cal_mode, u32 mask_set)
{
	volatile u8 *dmem = (volatile u8 *)PMU_DMEM_BASE;
	__asm__("" : "+r"(dmem));

	static const u32 masks1[4] = { 0x7f << 8, 0x7f << 16, 0x7f7f007f, 0x7f007f7f };
	static const u32 masks0[4] = { 0x7f << 16, 0x7f << 24, 0x7f007f7f, 0x007f7f7f };

	volatile u16 *out_table = (volatile u16 *)(dmem + PMU_DMEM_METRIC_CAL_E7B8);
	volatile u16 *out_sub1 = (volatile u16 *)(dmem + PMU_DMEM_METRIC_SUB_E7C4);
	out_table[0] = 100;

	u16 marker = 100;
	for (u32 phase = 0; phase < 4; phase++) {
		u32 mask = mask_set ? masks1[phase] : masks0[phase];
		pmu_cal_markers_set((u16)(marker << 2));

		pmu_cal_sequence_pulse_send(0x41 << 19, 7, 8, 0, 0, 0, 0);
		pmu_cbt_coarse_step_pulse_seq(0, 4, 1);
		pmu_cal_window_params_dispatch((const u8 *)&mask, (u16)cal_mode, 2);
		pmu_cbt_coarse_step_pulse_seq(0, 0x14, 1);
		pmu_cal_sequence_pulse_send(1 << 18, 7, 4, 0, 0, 0, 0);
		pmu_cal_sequence_pulse_send(0x80 | (1 << 19), 7, 8, 0, 0, 0, 0);

		marker = pmu_cal_stride_get();
		out_table[phase + 1] = marker;
		out_sub1[phase] = marker - 1;
	}
}
