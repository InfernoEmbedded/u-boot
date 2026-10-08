// SPDX-License-Identifier: GPL-2.0+
/*
 * Synopsys DesignWare DDR PHY Training Firmware (LPDDR5)
 * DBYTE & AC slice delay sweeps, LCDL line tuning, and BIST routines
 * Target Microcontroller: Synopsys ARC EM4 (ARCv2 ISA, Code Density enabled)
 * SoC: Allwinner A733 (Sun60i) / LPDDR5 PHY (Type 9)
 */

#include "lpddr5_pmu_internal.h"

/**
 * pmu_deskew_dly0_set() - Latch coarse deskew delay tap into PHY
 * @val: 16-bit coarse deskew delay tap configuration word
 */
void pmu_deskew_dly0_set(u16 val)
{
	u32 off = dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET);
	phy_write16(PHY_REG_DESKEW_DLY0 | (off << 1), val);
}

/**
 * pmu_deskew_dly1_set() - Latch fine deskew delay tap into PHY
 * @val: 16-bit fine deskew delay tap configuration word
 */
void pmu_deskew_dly1_set(u16 val)
{
	u32 off = dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET);
	phy_write16(PHY_REG_DESKEW_DLY1 | (off << 1), val);
}

/**
 * pmu_deskew_and_tracker_reset() - Reset deskew delay lines and calibration tracker state
 */
void pmu_deskew_and_tracker_reset(void)
{
	pmu_deskew_dly0_set(0);
	pmu_deskew_dly1_set(0);
	pmu_cal_markers_set(0);
	pmu_cal_marker_a_clear();
}

/**
 * pmu_dbyte_cal_strobe_seq() - Trigger calibration strobe pulse on target DBYTE slice
 * @lane: DBYTE slice index (0..3)
 * @bit_idx: Bit lane index within slice (0..8)
 */
void pmu_dbyte_cal_strobe_seq(u32 lane, u32 bit_idx)
{
	u32 reg = PHY_REG_DBYTE_BASE + PHY_REG_DBYTE_CAL_TRIG + (lane * PHY_REG_DBYTE_STRIDE);
	u16 strobe_base = (bit_idx == 0xf) ? 0x1ff : (1U << bit_idx);
	u16 strobe_trig = strobe_base | (1U << 9);

	phy_write16(reg, 0);
	phy_write16(reg, strobe_base);
	phy_write16(reg, strobe_trig);
	phy_write16(reg, strobe_base);
	phy_write16(reg, 0);
}

/**
 * pmu_bist_lane_mask_set_all() - Broadcast BIST lane comparison mask to all DBYTE slices
 * @mask: Comparison bitmask (0x0000 = unmask all lanes, 0xffff = mask all lanes)
 */
void pmu_bist_lane_mask_set_all(u16 mask)
{
	phy_write16(PHY_REG_BIST_MASK_DX0, mask);
	phy_write16(PHY_REG_BIST_MASK_DX1, mask);
	phy_write16(PHY_REG_BIST_MASK_DX2, mask);
	phy_write16(PHY_REG_BIST_MASK_DX3, mask);
}

/**
 * pmu_lcdl_dev_calc() - Compute LCDL delay deviation relative to nominal midpoint (64)
 * @val: Current LCDL delay tap position to evaluate against midpoint
 *
 * Return: LCDL delay deviation relative to nominal midpoint.
 */
u8 pmu_lcdl_dev_calc(u32 val)
{
	if (val <= PMU_LCDL_NOMINAL_MIDPOINT)
		return (u8)(PMU_LCDL_NOMINAL_MIDPOINT - val);
	else
		return (u8)(val + 0xc0);
}

/**
 * pmu_slice_status_b97_save() - Read slice status CSR and save to DMEM buffer
 */
void pmu_slice_status_b97_save(void)
{
	u32 off = dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET);
	u16 val = phy_read16(PHY_REG_SLICE_STATUS | (off << 1));
	dmem_write8(PMU_DMEM_LANE_STATUS, (u8)val);
}

/**
 * pmu_deskew_latch_seq() - Latch deskew delay, assert master PLL clock, and settle
 */
void pmu_deskew_latch_seq(void)
{
	pmu_deskew_dly0_set(300);
	pmu_deskew_dly1_set(dmem_read16(PMU_DMEM_DESKEW_TARGET_DELAY));
	phy_write16(PHY_REG_PLL_CLK_CTRL, 1);
	pmu_clk_timing_latch(0, 0);
	pmu_delay_us(50000, 15);
}

/**
 * pmu_slice_coarse_step_wrap() - Adjust DBYTE slice LCDL delay by coarse step with wrap compensation
 */
void pmu_slice_coarse_step_wrap(void)
{
	u8 start = dmem_read8(PMU_DMEM_SLICE_START);
	u8 end = dmem_read8(PMU_DMEM_SLICE_END);

	for (u32 i = start; i <= end; i++) {
		u32 reg = PHY_REG_DBYTE_BASE + PHY_REG_DBYTE_LCDL_DLY + (i * PHY_REG_DBYTE_STRIDE);
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
 * pmu_dbyte_pin_mask_calc() - Calculate physical DBYTE pin mask from logical DQ mask
 * @mask: Logical DQ bitmask to remap
 * @map_idx: DQ pin remap descriptor table index (slice/channel)
 *
 * Return: Physical DBYTE pin mask corresponding to active DQ lines.
 */
u32 pmu_dbyte_pin_mask_calc(u32 mask, u32 map_idx)
{
	uintptr_t base = PMU_DMEM_DQ_PIN_MAP_BASE + (map_idx * PMU_DQ_PIN_MAP_ENTRY_SIZE);
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
 * pmu_dbyte_pin_map_table_init() - Populate DQ pin mapping table from hardware DxDMap registers
 */
void pmu_dbyte_pin_map_table_init(void)
{
	u8 num_dbytes = dmem_read8(PMU_DMEM_ACTIVE_SLICE_COUNT);

	for (u32 dbyte = 0; dbyte < num_dbytes; dbyte++) {
		volatile u8 *entry = (volatile u8 *)(PMU_DMEM_DQ_PIN_MAP_BASE + dbyte * 18);
		entry[0] = 0; /* Identity mapped mask */
		entry[1] = 0; /* Remapped pin count */

		for (u32 bit = 0; bit < 8; bit++) {
			u32 reg = (PHY_REG_DBYTE_BASE | 0x0100) + (dbyte * 0x2000) + (bit * 2);
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
 * pmu_dbyte_deskew_phase_sample() - Sample DQ deskew delay phase for active rank
 * @rank_idx: DRAM rank index (0..1)
 * @out_mask: Pointer to output word receiving sampled phase bitmask
 * @phase_mask: Phase selection bitmask (phases 0..3)
 */
void pmu_dbyte_deskew_phase_sample(u32 rank_idx, u32 *out_mask, u32 phase_mask)
{
	u8 channel = dmem_read8(PMU_DMEM_CAL_RANK);
	const u8 *byte_map = (const u8 *)(uintptr_t)((PMU_DMEM_BASE | PMU_DMEM_RANK_SLICE_MAP) + (channel << 4) + (rank_idx << 3));
	u32 count_idx = (channel << 1) + rank_idx;
	u8 num_slices = dmem_read8(0xb20 + count_idx);
	const u16 *dly_table = (const u16 *)(uintptr_t)(PMU_DMEM_BASE | PMU_DMEM_METRIC_CAL_E7B8);

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
			u32 phy_addr = (PHY_REG_DBYTE_BASE | 0x011a) + ((u32)slice << 13);
			u16 phy_val = *(volatile u16 *)(uintptr_t)phy_addr;
			u32 mask = pmu_dq_phy_to_logical_mask(phy_val, slice) & 0x7f;
			mask ^= invert_mask;
			*out_mask |= (mask << (phase * 8));
		}
	}
}

/**
 * pmu_dbyte_lane_error_mask_calc() - Calculate DBYTE lane error mask and issue pulse sequence
 * @mode: Operating mode (0 = direct slice pulse, non-zero = multi-phase CBT sequence)
 * @slice_or_rank: Target DBYTE slice index or rank selector
 */
void pmu_dbyte_lane_error_mask_calc(u32 mode, u32 slice_or_rank)
{
	pmu_deskew_and_tracker_reset();
	if (mode != 0) {
		pmu_cbt_coarse_step_pulse_seq(0, 5, 1);
		pmu_cal_sequence_pulse_send(0, 0x25, 0, 0x86, 0, slice_or_rank, 0);
		pmu_cal_sequence_pulse_send(0, 0x26, 0, 0x86, 0, 0, 0);
		pmu_cbt_coarse_step_pulse_seq(0, 0x10, 1);
		pmu_cbt_3phase_pulse_seq();
	} else {
		pmu_cal_sequence_pulse_send(0x80, 0x19, 4, 0x86, 0, slice_or_rank & 3, 0);
	}
	pmu_clk_timing_delay_latch(0, 1);
	if (mode == 0) {
		pmu_delay_us(0x7530, 0x1e);
	}
}

/**
 * pmu_slice_deskew_state_latch() - Latch slice deskew delay state for selected lanes
 * @step_delay: Coarse step delay code
 * @lane_mask: Byte lane selection mask
 */
void pmu_slice_deskew_state_latch(u32 step_delay, u32 lane_mask)
{
	pmu_deskew_and_tracker_reset();
	pmu_cal_sequence_pulse_send(0, 6, 0x40, step_delay, 24, lane_mask, 0);
	pmu_clk_timing_delay_latch(0, 1);
}

/**
 * pmu_dbyte_deskew_sample_check() - Verify DBYTE deskew delay calibration with reference markers
 * @mode_rank: Rank and mode selector (bit 1 selects rank 1)
 * @marker0: Primary calibration marker value
 * @marker1: Secondary calibration marker value
 *
 * Return: Deskew calibration status code (0 on success).
 */
u8 pmu_dbyte_deskew_sample_check(u32 mode_rank, u32 marker0, u32 marker1)
{
	u8 channel = dmem_read8(PMU_DMEM_CAL_RANK);
	u32 rank = ((mode_rank & 0x3) == 2) ? 1 : 0;
	const u8 *byte_map = (const u8 *)(uintptr_t)((PMU_DMEM_BASE | PMU_DMEM_RANK_SLICE_MAP) + (channel << 4) + (rank << 3));
	u32 count_idx = (channel << 1) + rank;
	u8 num_slices = dmem_read8(0xb20 + count_idx);
	u8 res = 0;

	pmu_cal_marker_pair_program(marker0, marker1, mode_rank, 0);
	pmu_delay_us(10000, 0);

	for (u32 i = 0; i < num_slices; i++) {
		u8 slice = byte_map[i];
		u32 phy_addr = (PHY_REG_DBYTE_BASE | 0x011a) + ((u32)slice << 13);
		u16 phy_val = *(volatile u16 *)(uintptr_t)phy_addr;
		u8 mask = pmu_dq_phy_to_logical_mask(phy_val, slice);
		if (mask & 0x7f)
			res |= 1;
	}

	pmu_cal_marker_pair_program(marker0, marker1, mode_rank, 0x7f);

	for (u32 i = 0; i < num_slices; i++) {
		u8 slice = byte_map[i];
		u32 phy_addr = (PHY_REG_DBYTE_BASE | 0x011a) + ((u32)slice << 13);
		u16 phy_val = *(volatile u16 *)(uintptr_t)phy_addr;
		u8 mask = pmu_dq_phy_to_logical_mask(phy_val, slice) & 0x7f;
		if (mask != 0x7f)
			res |= 1;
	}

	return res;
}

/**
 * pmu_dbyte_cal_step_latch() - Format and stream DBYTE calibration step parameters to PHY
 * @tap_fine: Fine delay tap value (bits 0..6)
 * @tap_coarse: Coarse delay tap value and step field
 * @slice_flags: Slice and rank control flags
 */
void pmu_dbyte_cal_step_latch(u32 tap_fine, u32 tap_coarse, u32 slice_flags)
{
	u16 buf[16];
	memset(&buf[1], 0, 0x1e);

	u16 slice_ctrl_bit = (slice_flags << 3) & 0x10;
	u16 slice_cfg_field = (slice_flags & 0x3) << 14;
	u32 a0 = tap_fine & 0x7f;
	u32 a1 = tap_coarse & 0x7f;
	u32 step_field = (tap_coarse >> 1) & ~0x3f;
	u16 step_val_mid = step_field | (1 << 3);

	buf[0] = slice_cfg_field | 0x2c58;
	buf[3] = slice_ctrl_bit;
	buf[4] = a0 | (a0 << 7);
	buf[8] = (step_val_mid << 7) | (slice_cfg_field | step_val_mid);
	buf[11] = slice_ctrl_bit;
	buf[12] = a1 | (a1 << 7);

	pmu_phy_csr_result_stream(buf, 16);
}

/**
 * pmu_cal_delay_line_init() - Initialize delay line registers across channels and slices
 */
void pmu_cal_delay_line_init(void)
{
	u32 a = pmu_cal_timing_param_step_calc();
	u32 b = pmu_cbt_delay_offset_lookup();
	u32 c = pmu_delay_step_calc();
	u32 sum = a + b + c;
	sum += (sum & 1);
	u8 val = (u8)sum;
	if (val > 92)
		val = 92;
	dmem_write8(PMU_DMEM_CAL_CFG_400, val);
}

/**
 * pmu_dbyte_dq_deskew_regs_restore() - Restore DBYTE deskew delay registers from DMEM table
 */
void pmu_dbyte_dq_deskew_regs_restore(void)
{
	u8 slices = dmem_read8(PMU_DMEM_ACTIVE_SLICE_COUNT);
	const u8 *src_table = (const u8 *)(PMU_DMEM_BASE | PMU_DMEM_SAVED_DQ_DESKEW_TABLE);

	for (u32 slice = 0; slice < slices; slice++) {
		u32 slice_offset = slice << 13;
		for (u32 i = 0; i < 8; i++) {
			u32 reg_addr = (PHY_REG_DBYTE_BASE | 0x0100) + slice_offset + (i * 2);
			*(volatile u16 *)reg_addr = src_table[(slice * 8) + i];
		}
	}
}

/**
 * pmu_dbyte_dq_deskew_regs_save_and_ramp() - Back up DBYTE deskew registers to DMEM and initialize with ramp
 */
void pmu_dbyte_dq_deskew_regs_save_and_ramp(void)
{
	u8 slices = dmem_read8(PMU_DMEM_ACTIVE_SLICE_COUNT);
	u8 *dest_table = (u8 *)(PMU_DMEM_BASE | PMU_DMEM_SAVED_DQ_DESKEW_TABLE);

	for (u32 slice = 0; slice < slices; slice++) {
		u32 slice_offset = slice << 13;
		for (u32 i = 0; i < 8; i++) {
			u32 reg_addr = (PHY_REG_DBYTE_BASE | 0x0100) + slice_offset + (i * 2);
			dest_table[(slice * 8) + i] = (u8)*(volatile u16 *)reg_addr;
			*(volatile u16 *)reg_addr = (u16)i;
		}
	}
}

/**
 * pmu_dbyte_slice_cal_param_pulse() - Configure DBYTE slice calibration parameter and dispatch pulse
 * @rank: DRAM rank index (0..1)
 * @slice_mask: Target DBYTE slice bitmask
 */
void pmu_dbyte_slice_cal_param_pulse(u32 rank, u32 slice_mask)
{
	u8 channel = dmem_read8(PMU_DMEM_CAL_RANK);
	uintptr_t base = (PMU_DMEM_BASE | PMU_DMEM_CAL_STRUCT_BASE) + channel * 108 + rank * 54;
	u8 val = *(const u8 *)(base + 18);

	pmu_deskew_and_tracker_reset();
	u32 mask = ((1u << rank) << 2) | (1u << rank);
	if (slice_mask != 0)
		val |= 0x48;

	pmu_cal_sequence_pulse_send(0, 6, 0x22, val, 18, (u8)mask, 0);
	pmu_cal_sequence_pulse_send(0x80, 7, 2, 0, 0, 0, 0);
	pmu_clk_timing_delay_latch(0, 1);
}

/**
 * pmu_rank_slice_deskew_latch() - Latch deskew delay across active rank slices
 * @step_delay: Coarse step delay code
 * @lane_mask: Byte lane selection mask
 */
void pmu_rank_slice_deskew_latch(u32 step_delay, u32 lane_mask)
{
	u32 shift = pmu_freq_ratio_mult(14, lane_mask);
	pmu_deskew_and_tracker_reset();
	u32 ret = pmu_cal_sequence_pulse_send(0, 6, 2, step_delay, 30, lane_mask, 0);
	if (shift <= 5)
		shift = 5;
	u8 val = (u8)(ret >> shift);
	pmu_cbt_coarse_step_pulse_seq(0, val, 0);
	pmu_clk_timing_delay_latch(0, 1);
}

/**
 * pmu_channel_timing_deskew_reset() - Reset channel timing calibration deskew and apply delay taps
 * @dly_ac0: Timing delay code for AC tap 33
 * @dly_ac1: Timing delay code for AC tap 34
 * @dly_ac2: Timing delay code for AC tap 31
 * @dly_ac3: Timing delay code for AC tap 32
 * @target_slice: Target DBYTE slice index or lane selector
 */
void pmu_channel_timing_deskew_reset(u32 dly_ac0, u32 dly_ac1, u32 dly_ac2, u32 dly_ac3, u32 target_slice)
{
	pmu_deskew_and_tracker_reset();
	u32 slice_mask = *(const volatile u8 *)(PMU_DMEM_BASE | PMU_DMEM_ACTIVE_SLICE_MASK) & 0x3f;

	const u32 vals[5] = { slice_mask, dly_ac0, dly_ac1, dly_ac2, dly_ac3 };
	static const u8 ids[5] = { 20, 33, 34, 31, 32 };
	for (u32 i = 0; i < 5; i++)
		pmu_cal_sequence_pulse_send(0, 6, 0x22, vals[i], ids[i], target_slice, 0);

	pmu_clk_timing_delay_latch(0, 1);
}

/**
 * pmu_slice_phy_reg_step_adjust() - Adjust PHY slice register delay steps based on conditions
 * @reg_offset: PHY CSR register address offset
 * @polarity_cond: Polarity condition flag
 * @step_mode: Mode selector (0 = coarse step adjustment, non-zero = fine step adjustment)
 */
void pmu_slice_phy_reg_step_adjust(u32 reg_offset, u32 polarity_cond, u32 step_mode)
{
	u8 start_slice = *(const volatile u8 *)(PMU_DMEM_BASE | PMU_DMEM_SLICE_START);
	u8 end_slice = *(const volatile u8 *)(PMU_DMEM_BASE | PMU_DMEM_SLICE_END);
	u32 base = reg_offset + 0x2a;

	for (u32 slice = start_slice; slice <= end_slice; slice++) {
		u32 reg_addr = PHY_REG_DBYTE_BASE | (((dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET) | (slice << 12) | base)) << 1);
		u16 val = phy_read16(reg_addr);
		u32 low = val & 0x3f;
		u32 high = (val >> 6) & 0x1f;
		pmu_cal_metric_log(4, 0x2940003, slice, high, low);

		u32 adjusted_val;
		if (step_mode != 0) {
			pmu_cal_metric_log(4, 0x2950000);
			u32 denom = (u32)dmem_read8(PMU_DMEM_DRAM_TYPE) << 6;
			adjusted_val = denom ? ((u32)val % denom) : 0;
		} else if (polarity_cond != 0) {
			adjusted_val = val + 0x20;
		} else {
			adjusted_val = (val < 0x20) ? 0 : (val - 0x20);
		}

		u32 new_low = adjusted_val & 0x3f;
		u32 new_high = (adjusted_val >> 6) & 0x1f;
		if (step_mode != 0) {
			pmu_cal_metric_log(4, 0x2960002, new_high, new_low);
		} else if (polarity_cond != 0) {
			pmu_cal_metric_log(4, 0x2970003, 0x20, new_high, new_low);
		} else {
			pmu_cal_metric_log(4, 0x2980003, 0x20, new_high, new_low);
		}

		phy_write16(reg_addr, (new_high << 6) | new_low);
	}
	pmu_phy_reset_pulse();
}

/**
 * pmu_slice_delay_step_program() - Program multi-slice delay steps with fine/coarse unpacking
 * @reg_offset: Base CSR register offset added to DBYTE base
 * @data: Pointer to array of 16-bit delay step values
 * @is_64_div: Non-zero to divide by 64 for fine/coarse split; 0 for 10-bit shift
 * @shift_7: Non-zero to shift coarse field left by 7 bits; 0 for 6 bits
 * @lanes_10: Non-zero to process 10 lanes per slice; 0 for 9 lanes
 * @single_val: Non-zero to program single value per slice instead of array
 */
void pmu_slice_delay_step_program(u32 reg_offset, const u16 *data, u32 is_64_div, u32 shift_7, u32 lanes_10, u32 single_val)
{
	u8 start_slice = *(const volatile u8 *)(PMU_DMEM_BASE | PMU_DMEM_SLICE_START);
	u8 end_slice = *(const volatile u8 *)(PMU_DMEM_BASE | PMU_DMEM_SLICE_END);

	u32 count;
	u32 start_idx;
	if (single_val != 0) {
		count = 1;
		start_idx = start_slice;
	} else {
		start_idx = start_slice * (lanes_10 ? 10 : 9);
		count = (lanes_10 != 0) ? 10 : 9;
	}

	u32 csr_offset = dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET);
	u32 blink = (shift_7 != 0) ? 7 : 6;
	u32 reg_base = csr_offset | reg_offset;

	const u16 *src = data + start_idx;

	for (u32 slice = start_slice; slice <= end_slice; slice++) {
		u32 slice_base = (slice << 12) | reg_base;
		u32 lane_offset = 0;

		for (u32 i = 0; i < count; i++) {
			s16 val = (s16)*src++;
			u32 eff_offset = (single_val != 0) ? 0 : lane_offset;

			s16 quot, rem;
			if (is_64_div != 0) {
				quot = val / 64;
				rem = val % 64;
			} else {
				quot = (u16)val >> 10;
				rem = (u16)val & 0x3ff;
			}

			u32 reg = PHY_REG_DBYTE_BASE | (((eff_offset | slice_base)) << 1);
			u16 out_val = ((u16)quot << blink) | ((u16)rem);
			phy_write16(reg, out_val);

			lane_offset += 0x100;
		}
	}
}

/**
 * pmu_slice_dq_bitmask_swizzle() - Read and remap per-slice DQ sample lane bitmask
 * @slice: DBYTE slice index (0..3)
 * @dly_offset: Base delay offset tap added to per-lane offsets
 * @mode_sel: Mode selector (1 selects alternate CSR 0x10074 in profile mode 2)
 *
 * Return: Logical bitmask for active DQ pins within slice.
 */
u32 pmu_slice_dq_bitmask_swizzle(u32 slice, u32 dly_offset, u32 mode_sel)
{
	u32 mode = pmu_cal_profile_mode_get();
	u32 csr_id;
	if (mode == 1 || (mode_sel == 1 && mode == 2))
		csr_id = 0x10074;
	else
		csr_id = 0x10073;

	u32 reg_val = (slice << 12) | csr_id;
	u32 slice_csr = (csr_id - 3) | (slice << 12);
	u32 fp = (PHY_REG_DBYTE_BASE | 0x00e4) | ((slice << 12) << 1);

	phy_write16(fp, 768);

	u8 dmem08 = dmem_read8(PMU_DMEM_DRAM_TYPE);
	u32 delay_adj = (dmem08 == 2) ? 12 : 8;

	u16 offsets[8];
	for (u32 i = 0; i < 8; i++) {
		u32 lane_csr = ((i * 0x100) | slice_csr);
		u32 reg = PHY_REG_APB_BASE | (lane_csr << 1);
		u16 val = phy_read16(reg);
		offsets[i] = (val >> 10) - delay_adj;
	}

	u32 lane_bits = 0;
	for (u32 lane = 0; lane < 8; lane++) {
		phy_write16(fp, dly_offset + offsets[lane] + 192);
		pmu_hw_timer_delay(8);
		u32 reg = PHY_REG_APB_BASE | (((reg_val | (lane * 0x100)) << 1));
		u16 bit = phy_read16(reg) & 1;
		lane_bits |= (bit << lane);
	}

	phy_write16(fp, 0);
	return pmu_dq_phy_to_logical_mask((u8)lane_bits, slice);
}

/**
 * pmu_phy_lcdl_delay_read() - Read PHY LCDL delay values per slice and lane into output buffer
 * @out: Output buffer receiving 16-bit delay tap values
 * @from_phy: Query mode (0 = read from DMEM, 1 = read from PHY registers)
 * @dly_type: Delay line type selector
 */
void pmu_phy_lcdl_delay_read(u16 *out, u32 from_phy, u32 dly_type)
{
	u8 start_slice = *(const volatile u8 *)(PMU_DMEM_BASE | PMU_DMEM_SLICE_START);
	u8 end_slice = *(const volatile u8 *)(PMU_DMEM_BASE | PMU_DMEM_SLICE_END);

	u32 stride, start_lane, end_lane, out_idx;
	if (dly_type == 2) {
		stride = 1;
		start_lane = 0;
		end_lane = 0;
		out_idx = start_slice;
	} else if (dly_type == 1) {
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
			u32 reg = (PHY_REG_DBYTE_BASE | PHY_REG_DBYTE_LCDL_DLY) | slice_base | (lane << 9);
			u16 val = phy_read16(reg);
			if (from_phy != 0) {
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
 * pmu_dbyte_cal_strobe_pulse_seq() - Configure DBYTE broadcast registers and trigger strobe delay
 * @mode: Calibration operating mode selector
 * @val: 16-bit timing configuration value written to broadcast register
 * @type: Pulse sequence phase type (0 = standard, 1 = inverted)
 */
void pmu_dbyte_cal_strobe_pulse_seq(u32 mode, u16 val, u32 type)
{
	u32 csr_offset = dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET);
	u32 csr_reg = csr_offset << 1;

	if (mode == 0) {
		phy_write16(csr_reg | PHY_REG_DBYTE_BCAST_WIDTH, 0);
		phy_write16(csr_reg | PHY_REG_DBYTE_BCAST_GATE, 0);
		pmu_hw_timer_delay(0x14);
		phy_write16((PHY_REG_DBYTE_BCAST_BASE | 0x0128), 0);
		phy_write16(PHY_REG_DBYTE_BCAST_TIMING, 1);
		phy_write16((PHY_REG_DBYTE_BCAST_BASE | 0x0114), 0);
		phy_write16((PHY_REG_DBYTE_BCAST_BASE | 0x0112), 0);
		phy_write16((PHY_REG_DBYTE_BCAST_BASE | 0x0118), 0);
	} else {
		phy_write16((PHY_REG_DBYTE_BCAST_BASE | 0x0114), val);
		phy_write16((PHY_REG_DBYTE_BCAST_BASE | 0x0112), val);
		phy_write16((PHY_REG_DBYTE_BCAST_BASE | 0x0116), 0);
		phy_write16(PHY_REG_DBYTE_BCAST_TIMING, 0);

		if (type == 0) {
			phy_write16(csr_reg | PHY_REG_DBYTE_BCAST_GATE, 1);
		} else if (type == 1) {
			phy_write16((PHY_REG_DBYTE_BCAST_BASE | 0x012a), 10);
		} else {
			pmu_assert_or_halt(0, 0x2350000);
		}

		phy_write16((PHY_REG_DBYTE_BCAST_BASE | 0x0128), (u16)type);
		csr_offset = dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET);
		phy_write16((csr_offset << 1) | PHY_REG_DBYTE_BCAST_WIDTH, 2);
	}
}

/**
 * pmu_slice_deskew_pulse_train() - Coordinate slice deskew reset and pulse train sequence
 * @rank: DRAM rank index (0..1)
 * @mode: Deskew sequence mode (0 = assert pulse, 1 = clear pulse)
 */
void pmu_slice_deskew_pulse_train(u32 rank, u32 mode)
{
	u8 ch_idx = dmem_read8(PMU_DMEM_CAL_RANK);
	u32 offset = (u32)ch_idx * 108 + rank * 0x36;
	const volatile u8 *p = (const volatile u8 *)((PMU_DMEM_BASE | PMU_DMEM_CAL_STRUCT_BASE) + offset);
	u8 cfg10 = p[0x10];

	pmu_deskew_and_tracker_reset();

	u8 cal_cmd = (mode == 0) ? (cfg10 | 5) : (cfg10 & 0xb0);
	u32 mask1 = 1 << rank;
	u8 slice_pair_mask = (u8)((mask1 << 2) | mask1);
	u32 deskew_cmd = cal_cmd | (1 << 6);

	if (mode != 0) {
		u8 fp_val = p[20];
		u8 flag_b1 = p[1];
		pmu_cal_pulse_seq_strobe(cal_cmd | 68, 16, slice_pair_mask);

		u32 pulse_sent = 0;
		if (flag_b1 & 8) {
			pmu_cal_pulse_seq_strobe(flag_b1, 1, slice_pair_mask);
			pulse_sent = 1;
		}

		if ((fp_val & 3) == 2) {
			if (pulse_sent == 0)
				pmu_deskew_and_tracker_reset();
		} else {
			pmu_cal_pulse_seq_strobe(p[0x14], 20, slice_pair_mask);
		}
	}

	pmu_cal_sequence_pulse_send(0x80, 6, 0x22, deskew_cmd, 16, slice_pair_mask, 0);
	pmu_clk_timing_delay_latch(0, 1);
	pmu_delay_us(0x3d090, 0);
}

/**
 * pmu_slice_lcdl_delay_collect() - Collect LCDL delay line registers across active byte slices
 * @slice_csr: DBYTE slice CSR register base offset
 * @out_buf: Pointer to destination buffer receiving delay records
 * @count: Number of delay words to collect per slice
 */
void pmu_slice_lcdl_delay_collect(u32 slice_csr, void *out_buf, u32 count)
{
	u16 *out16 = (u16 *)out_buf;
	u8 start_slice = dmem_read8(PMU_DMEM_SLICE_START);
	u8 end_slice = dmem_read8(PMU_DMEM_SLICE_END);
	u32 out_idx = start_slice * count;

	for (u8 slice = start_slice; slice <= end_slice; slice++) {
		if (!(slice_csr & (1 << 24))) {
			if (count == 0)
				continue;
			u32 slice_code = (slice << 12) | slice_csr;
			volatile u16 *phy_base = (volatile u16 *)PHY_REG_DBYTE_BASE;
			__asm__("" : "+r"(phy_base));
			for (u32 i = 0; i < count; i++) {
				u32 reg_offset = ((i << 8) | slice_code) << 1;
				out16[out_idx++] = *(volatile u16 *)((uintptr_t)phy_base + reg_offset);
			}
		} else {
			u32 csr_offset = dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET);
			u32 temp = (slice_csr & ~csr_offset) & 0x0fffffffe;
			temp += 0xfefffff0;
			u32 code = temp >> 4;
			u8 mode_code;
			if (code == 0)
				mode_code = 0;
			else if (code == 3)
				mode_code = 3;
			else if (code == 2)
				mode_code = 2;
			else if (code == 1) {
				if (pmu_dq_swap_query((u8)slice_csr, slice))
					mode_code = 1;
				else
					mode_code = 2;
			} else {
				mode_code = 4;
			}
			u8 rank0_end = dmem_read8(PMU_DMEM_RANK_BOUNDARY);
			u8 rank = (slice > rank0_end) ? 1 : 0;
			u8 val = dmem_read8(0xb6e + (rank * 2) + (mode_code * 4) + (slice_csr & 1));
			out16[out_idx++] = val;
		}
	}
}

/**
 * pmu_slice_deskew_pulse_seq() - Execute slice deskew calibration pulse sequence
 * @enable: Non-zero to enable deskew pulses, 0 to clear
 */
void pmu_slice_deskew_pulse_seq(u32 enable)
{
	if (enable != 1)
		return;

	for (u32 rank = 1; rank < 3; rank++) {
		u8 cfg = (rank == 2) ? dmem_read8(PMU_DMEM_CH1_RANK_EN) : dmem_read8(PMU_DMEM_CH0_RANK_EN);
		if (cfg != 3)
			continue;

		pmu_phy_mode_cfg_dispatch(rank);
		pmu_phy_reset_pulse();

		u8 start_slice = dmem_read8(PMU_DMEM_SLICE_START);
		u8 end_slice = dmem_read8(PMU_DMEM_SLICE_END);
		u16 sp_buf[8];

		for (u32 lane = 0; lane < 2; lane++) {
			u8 offset = ((PMU_LUT_2B_IDENTITY_E4 >> (lane * 2)) & 3) + 0x2a;

			for (u32 slice = start_slice; slice <= end_slice; slice++) {
				u32 slice_offset = slice << 12;
				u32 csr_offset = dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET);
				u32 reg = (offset | slice_offset | csr_offset);
				u16 val = phy_read16(PHY_REG_DBYTE_BASE + (reg << 1));

				sp_buf[lane * 4 + slice] = val;

				if (lane == 1) {
					u16 v0 = sp_buf[0 * 4 + slice];
					u32 sum = (u32)v0 + val;
					u32 q = sum >> 7;
					u32 rem = (sum >> 1) & 0x3f;

					pmu_cal_metric_log(4, 0x2bf0006, rank, 1, rem);

					u16 prog_val = rem | (q << 6);
					phy_write16((PHY_REG_DBYTE_BASE | 0x0054) + ((slice_offset | csr_offset) << 1), prog_val);
					phy_write16(PHY_REG_DBYTE_BASE + (reg << 1), prog_val);
				}
			}
		}
	}
	pmu_phy_mode_cfg_dispatch(3);
}

/*
 * pmu_cal_delay_line_accum_dispatch:
 * Evaluates active DBYTE delay lines, unpacks and adjusts slice delay taps across
 * channels and ranks, executes Stage 42/43 accumulation via pmu_cal_stage42_accum_dispatch, and programs PHY CSRs.
 */
/**
 * pmu_cal_delay_line_accum_dispatch() - Dispatch delay line parameter accumulation for rank
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

	dmem_25 = dmem_read8(PMU_DMEM_CH0_RANK_EN);
	dmem_40 = dmem_read8(PMU_DMEM_CH1_RANK_EN);
	count = ((dmem_40 | dmem_25) == 3) ? 2 : 1;

	pstate_start = dmem_read8(PMU_DMEM_SLICE_START);
	pstate_end = dmem_read8(PMU_DMEM_SLICE_END);

	for (u32 pstate = pstate_start; pstate <= pstate_end; pstate++) {
		u8 pstate_mask = dmem_read8(PMU_DMEM_ACTIVE_SLICE_MASK_CAL);

		if (!(pstate_mask & (1 << pstate)))
			continue;

		for (u32 lane = 0; lane < 9; lane++) {
			if (!(rank & (1 << lane)))
				continue;

			u32 csr_base = dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET) | (pstate << 12) | (lane << 8);

			for (u32 k = 0; k < count; k++) {
				u32 idx = (pstate * 20) + (lane * 2) + k;

				buf[0][k].val16  = dmem_read16(0x6098 + idx * sizeof(u16));
				buf[0][k].val8_0 = dmem_read8(0x61d8 + idx);
				buf[0][k].val8_1 = dmem_read8(0x6278 + idx);

				buf[1][k].val16  = dmem_read16(0x6138 + idx * sizeof(u16));
				buf[1][k].val8_0 = dmem_read8(0x6228 + idx);
				buf[1][k].val8_1 = dmem_read8(0x62c8 + idx);

				u32 csr_addr_24 = (PHY_REG_DBYTE_BASE | 0x0024) | ((csr_base | k) << 1);
				buf[2][k].val16  = pmu_dly_line_repack(phy_read16(csr_addr_24));

				u32 reg_0_0 = PHY_REG_DBYTE_BASE | ((csr_base | pmu_dmem_cal_offset_select(0, 0, k)) << 1);
				buf[2][k].val8_0 = (u8)phy_read16(reg_0_0);

				u32 reg_0_1 = PHY_REG_DBYTE_BASE | ((csr_base | pmu_dmem_cal_offset_select(0, 1, k)) << 1);
				buf[2][k].val8_1 = (u8)phy_read16(reg_0_1);

				u32 csr_addr_20 = (PHY_REG_DBYTE_BASE | 0x0020) | ((csr_base | k) << 1);
				buf[3][k].val16  = pmu_dly_line_repack(phy_read16(csr_addr_20));

				u32 reg_1_0 = PHY_REG_DBYTE_BASE | ((csr_base | pmu_dmem_cal_offset_select(1, 0, k)) << 1);
				buf[3][k].val8_0 = (u8)phy_read16(reg_1_0);

				u32 reg_1_1 = PHY_REG_DBYTE_BASE | ((csr_base | pmu_dmem_cal_offset_select(1, 1, k)) << 1);
				buf[3][k].val8_1 = (u8)phy_read16(reg_1_1);
			}

			((void (*)(u32, u32, void *))(uintptr_t)pmu_cal_stage42_accum_dispatch)(dram_type, count, buf);

			for (u32 k = 0; k < count; k++) {
				u32 csr_addr_24 = (PHY_REG_DBYTE_BASE | 0x0024) | ((csr_base | k) << 1);
				phy_write16(csr_addr_24, buf[4][k].val16);

				u32 csr_addr_20 = (PHY_REG_DBYTE_BASE | 0x0020) | ((csr_base | k) << 1);
				phy_write16(csr_addr_20, buf[5][k].val16);

				if (dmem_read8(PMU_DMEM_CAL_OVERRIDE_FLAG) != 0)
					continue;

				u32 reg_0_0 = PHY_REG_DBYTE_BASE | ((csr_base | pmu_dmem_cal_offset_select(0, 0, k)) << 1);
				phy_write16(reg_0_0, buf[4][k].val8_0);

				u32 reg_1_0 = PHY_REG_DBYTE_BASE | ((csr_base | pmu_dmem_cal_offset_select(1, 0, k)) << 1);
				phy_write16(reg_1_0, buf[5][k].val8_0);

				if (pmu_cal_profile_mode_get() != 3)
					continue;

				u32 reg_0_1 = PHY_REG_DBYTE_BASE | ((csr_base | pmu_dmem_cal_offset_select(0, 1, k)) << 1);
				phy_write16(reg_0_1, buf[4][k].val8_1);

				u32 reg_1_1 = PHY_REG_DBYTE_BASE | ((csr_base | pmu_dmem_cal_offset_select(1, 1, k)) << 1);
				phy_write16(reg_1_1, buf[5][k].val8_1);
			}
		}
	}
}

/**
 * pmu_cal_bist_cmd_strobe_dispatch() - Dispatch BIST command strobe sequence across active slices
 * @ctx_addr: Base address of BIST calibration command context structure in DMEM
 * @pattern_mode: BIST pattern generator mode selector
 * @cmd_mode: BIST command mode (0 = read, 1 = write, 2 = strobe A, 3 = strobe B)
 * @target_mask: Bitmask of target channels and ranks for calibration pulse dispatch
 * @slice_flags_addr: Base address of per-slice calibration flag array
 * @status_idx: Calibration status query index
 * @setup_flags: Auxiliary BIST pattern generator configuration flags
 */
void pmu_cal_bist_cmd_strobe_dispatch(u32 ctx_addr, u32 pattern_mode, u32 cmd_mode, u32 target_mask,
					  u32 slice_flags_addr, u32 status_idx, u32 setup_flags)
{
	struct pmu_cal_bist_cmd_ctx *ctx = (struct pmu_cal_bist_cmd_ctx *)(uintptr_t)ctx_addr;
	const u16 *slice_flags = (const u16 *)(uintptr_t)slice_flags_addr;
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
	status = pmu_cal_status_query(status_idx, cmd_mode);

	if (dmem_read8(PMU_DMEM_FREQ_MODE) <= 1)
		csr_offset = dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET) | 0x14;
	else
		csr_offset = 0x56;

	/* 2. Initialize calibration context descriptor header */
	ctx->field_00 = 0;
	ctx->field_04 = 0;
	ctx->field_08 = 1;
	ctx->csr_offset = csr_offset;

	/* 3. Program mode-dependent BIST command codes */
	flag = (target_mask & 0x5) != 0;

	switch (cmd_mode) {
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
	phy_write16(PHY_REG_RESET_TRIGGER, 0x40);
	phy_write16(PHY_REG_DBYTE_BCAST_BASE | (csr_offset << 1), 1);
	phy_write16(PHY_REG_DBYTE_BCAST_RX_EN, 0);

	pmu_cal_bist_pattern_setup(pattern_mode, target_mask, status, setup_flags);
	pmu_cal_markers_set(800);

	/* 5. Dispatch BIST calibration sequence commands via pmu_cal_sequence_pulse_send */
	step_param = (dmem_read8(PMU_DMEM_DRAM_TYPE) == 2) ? 4 : 2;

	if (pattern_mode < 9 && ((1U << pattern_mode) & 0x105)) {
		pmu_cal_sequence_pulse_send(0x8000, 7, 0x18, 0, 0, 0, 0);
		pmu_cbt_cal_stat_set();
		pmu_cal_sequence_pulse_send(0, 7, 0, 0, 0, target_mask, 0);
		pmu_cal_sequence_pulse_send(0x100000, 5, 0, 0x80, 0, target_mask, 0);
		pmu_cal_sequence_pulse_send(0x800000, 0x29, step_param, 0, 0, target_mask, 7);
		pmu_cal_sequence_pulse_send(0, 7, dmem_read8(PMU_DMEM_CAL_CFG_400), 0, 0, target_mask, 0);
		pmu_cal_sequence_pulse_send(0, 7, 0, 0, 0, target_mask, 0);
		pmu_cal_sequence_pulse_send(0x200000, 5, 0, 256, 0, target_mask, 0);
		cmd_r1 = 0x2a;
	} else {
		pmu_cal_sequence_pulse_send(0x8000, 7, 0x28, 0, 0, 0, 0);
		pmu_cbt_cal_stat_set();
		pmu_cal_sequence_pulse_send(0, 7, 8, 0, 0, target_mask, 0);
		pmu_cal_sequence_pulse_send(0, 7, 0, 0, 0, target_mask, 0);
		pmu_cal_sequence_pulse_send(0x200000, 5, 0, 256, 0, target_mask, 0);
		cmd_r1 = 0x2b;
	}

	pmu_cal_sequence_pulse_send(0x01040000, cmd_r1, step_param, 0, 0, target_mask, 7);
	pmu_cal_sequence_pulse_send(0, 7, 4, 0, 0, target_mask, 10);
	pmu_cbt_cal_stat_clear();
	pmu_cal_sequence_pulse_send(0, 7, 0, 0, 0, 0, 0);
	pmu_cal_sequence_pulse_send(0, 7, 0, 0, 0, 0, 0);

	/* 6. Program calibration stride and timing control CSRs */
	stride = pmu_cal_stride_get();

	phy_write16(PHY_REG_DBYTE_BCAST_DQ_DLY2, 1);
	phy_write16(PHY_REG_DBYTE_BCAST_DQ_DLY3, 1);
	phy_write16(PHY_REG_DBYTE_BCAST_DQ_DLY0, (dmem_read8(PMU_DMEM_DRAM_TYPE) == 4) ? 0x34 : 0x30);
	ctx->stride_minus_1 = stride - 1;
	phy_write16(PHY_REG_DBYTE_BCAST_DQ_DLY1, 1);

	/* 7. Slice delay tap programming loop */
	slice_start = dmem_read8(PMU_DMEM_SLICE_START);
	slice_end = dmem_read8(PMU_DMEM_SLICE_END);
	slice_mask = dmem_read8(PMU_DMEM_ACTIVE_SLICE_MASK_CAL);

	for (u32 slice = slice_start; slice <= slice_end; slice++) {
		if (!(slice_mask & (1 << slice)))
			continue;

		u16 val;
		if (cmd_mode == 1) {
			val = (slice_flags[slice] & 0x80) ? 10 : 5;
		} else if (cmd_mode == 0) {
			val = (slice_flags[slice] & 0x80) ? 5 : 10;
		} else {
			val = 0;
		}

		ctx->slice_delays[slice] = val;
	}
}

/**
 * pmu_cal_dly_line_tap_step_adjust() - Adjust multi-rank DBYTE slice delay tap steps in CSR and DMEM
 * @mode: Adjustment mode (0 = direct CSR update, non-zero = delay buffer unpack)
 * @ptr_tag: Pointer tag descriptor evaluated by pmu_cal_ptr_tag_check
 * @lane_mask: Bitmask of active bit lanes to adjust (0..8)
 * @check_tag: Flag to enforce pointer tag validation
 */
void pmu_cal_dly_line_tap_step_adjust(u32 mode, u32 ptr_tag, u32 lane_mask, u32 check_tag)
{
	u32 tag_valid = (check_tag != 0) ? (pmu_cal_ptr_tag_check(mode, ptr_tag) == 0) : 1;
	u32 is_initial_mode = (mode == 0) ? 1 : 0;
	u32 csr_offset = dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET);

	for (u32 rank = 0; rank < 2; rank++) {
		if (!((dmem_read8(PMU_DMEM_CH0_RANK_EN) | dmem_read8(PMU_DMEM_CH1_RANK_EN)) & (1 << rank)))
			continue;

		u8 slice_start = dmem_read8(PMU_DMEM_SLICE_START);
		u8 slice_end = dmem_read8(PMU_DMEM_SLICE_END);
		for (u32 slice = slice_start; slice <= slice_end; slice++) {
			if (!(dmem_read8(PMU_DMEM_ACTIVE_SLICE_MASK_CAL) & (1 << slice)))
				continue;

			u8 step_val = *(volatile u8 *)((PMU_DMEM_BASE | PMU_DMEM_RANK_SLICE_TAP_STEPS) + (rank * 4) + slice);

			for (u32 lane = 0; lane < 9; lane++) {
				if (!(lane_mask & (1 << lane)))
					continue;

				u32 fp = (slice << 12) | (lane << 8) | csr_offset | 0x10000;
				u32 tbl_offset = (rank * 5280) + (slice * 1320) + (lane * 132);
				volatile u8 *tbl_ptr = (volatile u8 *)((PMU_DMEM_BASE | PMU_DMEM_PIN_CAL_BASE_TAPS) + tbl_offset);

				u16 val_e48 = *(volatile u16 *)tbl_ptr;
				u8 cal_tap_status = *(volatile u8 *)((PMU_DMEM_BASE | 0x378b) + tbl_offset);
				s8 val_e48_offset2 = (s8)tbl_ptr[2];
				u8 cal_tap_base = tbl_ptr[3];
				u32 tap_step_target = (u32)(val_e48 + val_e48_offset2);

				u32 base_addr = (fp | rank) << 1;

				if (mode != 0) {
					u8 dly_buf_32[4] = {0};
					u8 dly_buf_30[4] = {0};

					if (ptr_tag == 0 && dmem_read8(PMU_DMEM_HIGH_FREQ_FLAG) != 0) {
						u32 idx_6098 = (slice * 40) + (lane * 4) + (rank * 2);
						u16 w_6098 = *(volatile u16 *)((PMU_DMEM_BASE | PMU_DMEM_TRAINED_DLY_BUF) + idx_6098);
						pmu_dly_line_unpack(w_6098, dly_buf_32);
					} else {
						u32 phy_addr = (PHY_REG_APB_BASE | 0x0024) | base_addr;
						u16 phy_w = phy_read16(phy_addr);
						pmu_cal_param_unpack(phy_w, dly_buf_32);
					}

					pmu_dly_line_unpack((u16)tap_step_target, dly_buf_30);
					if (tag_valid == 0) {
						dly_buf_32[0] = dly_buf_30[0];
						dly_buf_32[1] = dly_buf_30[1];
					}

					if (dmem_read8(PMU_DMEM_HIGH_FREQ_FLAG) != 0) {
						pmu_delay_tap_step_adjust(dly_buf_32, ptr_tag);
						pmu_delay_tap_step_adjust(dly_buf_30, ptr_tag);
					}

					if (dmem_read8(PMU_DMEM_CAL_MISC_FLAGS) & 0x20) {
						pmu_delay_tap_step_adjust(dly_buf_32, step_val);
						pmu_delay_tap_step_adjust(dly_buf_30, step_val);

						int diff = (int)dly_buf_32[0] - (int)dly_buf_30[0];
						if (diff > 1 || diff < -1) {
							u32 phy_addr = (PHY_REG_APB_BASE | 0x0024) | base_addr;
							pmu_cal_param_unpack(phy_read16(phy_addr), dly_buf_32);
							pmu_dly_line_unpack((u16)tap_step_target, dly_buf_30);
							u32 step_adj = (step_val == 0) ? 1 : 0;
							pmu_delay_tap_step_adjust(dly_buf_32, step_adj);
							pmu_delay_tap_step_adjust(dly_buf_30, step_adj);
						}
					}

					pmu_eye_margin_step_align(dly_buf_32, dly_buf_30, (dmem_read8(PMU_DMEM_HIGH_FREQ_FLAG) != 0) ? 2 : 1);

					if (ptr_tag == 0) {
						u32 idx_6098 = (slice * 40) + (lane * 4) + (rank * 2);
						*(volatile u16 *)((PMU_DMEM_BASE | PMU_DMEM_TRAINED_DLY_BUF) + idx_6098) = pmu_hdr_addr_14b_decode(dly_buf_32);
						*(volatile u16 *)((PMU_DMEM_BASE | 0x6138) + idx_6098) = (u16)tap_step_target;
						u32 idx_6228 = (slice * 20) + (lane * 2) + rank;
						*(volatile u8 *)((PMU_DMEM_BASE | 0x6228) + idx_6228) = cal_tap_base;
						*(volatile u8 *)((PMU_DMEM_BASE | 0x62c8) + idx_6228) = cal_tap_status;
					}

					if (!(ptr_tag == 0 && dmem_read8(PMU_DMEM_HIGH_FREQ_FLAG) != 0)) {
						u32 phy_addr1 = (PHY_REG_APB_BASE | 0x0024) | base_addr;
						phy_write16(phy_addr1, pmu_hdr_addr_15b_decode(dly_buf_32));
					}

					u32 phy_addr2 = (PHY_REG_APB_BASE | 0x0020) | base_addr;
					phy_write16(phy_addr2, pmu_hdr_addr_15b_decode(dly_buf_30));
				} else {
					if (ptr_tag == 0) {
						u32 idx_6098 = (slice * 40) + (lane * 4) + (rank * 2);
						u32 idx_61d8 = (slice * 20) + (lane * 2) + rank;
						*(volatile u8 *)((PMU_DMEM_BASE | 0x61d8) + idx_61d8) = cal_tap_base;
						*(volatile u16 *)((PMU_DMEM_BASE | PMU_DMEM_TRAINED_DLY_BUF) + idx_6098) = (u16)tap_step_target;
						*(volatile u8 *)((PMU_DMEM_BASE | 0x6278) + idx_61d8) = cal_tap_status;
					}

					u8 dly_buf_34[4] = {0};
					pmu_dly_line_unpack((u16)tap_step_target, dly_buf_34);

					if (dmem_read8(PMU_DMEM_HIGH_FREQ_FLAG) != 0)
						pmu_delay_tap_step_adjust(dly_buf_34, ptr_tag);

					if (dmem_read8(PMU_DMEM_CAL_MISC_FLAGS) & 0x20)
						pmu_delay_tap_step_adjust(dly_buf_34, step_val);

					u32 phy_addr = (PHY_REG_APB_BASE | 0x0024) | base_addr;
					phy_write16(phy_addr, pmu_hdr_addr_15b_decode(dly_buf_34));
				}

				if (dmem_read8(PMU_DMEM_CAL_OVERRIDE_FLAG) == 0) {
					u32 off = pmu_dmem_cal_offset_select(mode, 0, rank);
					u32 reg = PHY_REG_APB_BASE | ((fp | off) << 1);
					phy_write16(reg, cal_tap_base);
					if (!(is_initial_mode | tag_valid)) {
						off = pmu_dmem_cal_offset_select(0, 0, rank);
						reg = PHY_REG_APB_BASE | ((fp | off) << 1);
						phy_write16(reg, cal_tap_base);
					}
					if (pmu_cal_profile_mode_get() == 3) {
						off = pmu_dmem_cal_offset_select(mode, 1, rank);
						reg = PHY_REG_APB_BASE | ((fp | off) << 1);
						phy_write16(reg, cal_tap_status);
						if (!(is_initial_mode | tag_valid)) {
							off = pmu_dmem_cal_offset_select(0, 1, rank);
							reg = PHY_REG_APB_BASE | ((fp | off) << 1);
							phy_write16(reg, cal_tap_status);
						}
					}
				}
			}
		}
	}
}

/**
 * pmu_cal_lane_deskew_sweep_exec() - Execute lane deskew sweep and latch calibrated delay taps
 */
void pmu_cal_lane_deskew_sweep_exec(void)
{
	u16 lane_metric_buf_a[2][32]; /* sp + 0x84: 128 bytes (64 halfwords, 32 per lane) */
	u16 lane_metric_buf_b[2][32]; /* sp + 0x104: 128 bytes (64 halfwords, 32 per lane) */
	u16 lane_tap_steps_b[2][4];  /* sp + 0x68: 16 bytes (8 halfwords, 4 per lane) */
	u16 lane_tap_steps_a[2][4];  /* sp + 0x58: 16 bytes (8 halfwords, 4 per lane) */

	memset(lane_metric_buf_b, 0, sizeof(lane_metric_buf_b));
	memset(lane_metric_buf_a, 0, sizeof(lane_metric_buf_a));
	memset(lane_tap_steps_b, 0, sizeof(lane_tap_steps_b));
	memset(lane_tap_steps_a, 0, sizeof(lane_tap_steps_a));

	const u8 *desc = (const u8 *)(PMU_DMEM_BASE | PMU_DMEM_CAL_STRUCT_BASE);
	u8 desc_val0 = desc[0];
	pmu_cal_metric_log(4, 0x025d0001, (u32)(desc_val0 & 0x3));

	const u8 *desc_p = desc + 130;
	u8 desc_mode = desc_p[0];
	u8 desc_flag8 = desc_p[8];
	u8 cal_cfg16 = desc_p[16];
	u32 flag_78 = (desc_mode & 0x10);

	u32 csr_offset = dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET);

	/* Phase 1: LCDL delay sampling across active slices for lanes phase = 0, 1 */
	for (u32 phase = 0; phase < 2; phase++) {
		u8 start_slice = dmem_read8(PMU_DMEM_SLICE_START);
		u8 end_slice = dmem_read8(PMU_DMEM_SLICE_END);

		if (desc_flag8 & 1) {
			pmu_slice_lcdl_delay_collect((phase + 0x20) | csr_offset, lane_tap_steps_b[phase], 1);
		} else {
			for (u32 slice = start_slice; slice <= end_slice; slice++) {
				u32 slice_code = (slice << 12) | ((phase + 0x24) | csr_offset);
				for (u32 bit = 0; bit < 8; bit++) {
					u32 reg = PHY_REG_DBYTE_BASE | ((slice_code | (bit << 8)) << 1);
					lane_metric_buf_b[phase][slice * 8 + bit] = phy_read16(reg);
				}
			}
		}

		for (u32 slice = start_slice; slice <= end_slice; slice++) {
			u32 slice_code = (slice << 12) | ((phase + 0x26) | csr_offset);
			for (u32 bit = 0; bit < 8; bit++) {
				u32 reg = PHY_REG_DBYTE_BASE | ((slice_code | (bit << 8)) << 1);
				lane_metric_buf_a[phase][slice * 8 + bit] = phy_read16(reg);
			}
		}

		if (cal_cfg16 & 0x10) {
			pmu_slice_lcdl_delay_collect((phase + 0x28) | csr_offset, lane_tap_steps_a[phase], 1);
		}
	}

	/* Phase 2: Fine & Coarse Delay Margin Diff & Telemetry */
	u8 rank = dmem_read8(PMU_DMEM_CAL_RANK);
	u32 rank_off = rank ? 27 : 0;
	volatile u8 *ptr_14 = (volatile u8 *)(uintptr_t)((PMU_DMEM_BASE | (PMU_DMEM_CH0_RANK_EN + 1)) + rank_off);
	volatile u8 *ptr_28 = (volatile u8 *)(uintptr_t)((PMU_DMEM_BASE | (PMU_DMEM_CH0_RANK_EN + 11)) + rank_off);

	for (s32 fp = 1; fp >= 0; fp--) {
		for (s32 ph = 1; ph >= 0; ph--) {
			if (ph == fp)
				continue;

			s16 diff_coarse = 0;
			s16 diff_fine = 0;

			if (flag_78) {
				u8 start_slice = dmem_read8(PMU_DMEM_SLICE_START);
				u8 end_slice = dmem_read8(PMU_DMEM_SLICE_END);
				s32 max_coarse = 0;
				s32 max_fine = 0;

				for (u32 slice = start_slice; slice <= end_slice; slice++) {
					if (desc_flag8 & 1) {
						s32 v_fp = (s32)(lane_tap_steps_b[fp][slice] >> 6);
						s32 v_ph = (s32)(lane_tap_steps_b[ph][slice] >> 6);
						s32 diff = v_fp - v_ph;
						if (diff > max_coarse)
							max_coarse = diff;
					} else {
						for (u32 bit = 0; bit < 8; bit++) {
							s32 v_fp = (s32)(lane_metric_buf_b[fp][slice * 8 + bit] >> 6);
							s32 v_ph = (s32)(lane_metric_buf_b[ph][slice * 8 + bit] >> 6);
							s32 diff = v_fp - v_ph;
							if (diff > max_coarse)
								max_coarse = diff;
						}
					}

					for (u32 bit = 0; bit < 8; bit++) {
						s32 val_fp = (s32)lane_metric_buf_a[fp][slice * 8 + bit];
						s32 val_ph = (s32)lane_metric_buf_a[ph][slice * 8 + bit];
						s32 diff = val_fp - val_ph;
						if (diff > max_fine)
							max_fine = diff;
					}
				}

				diff_coarse = (s16)pmu_dram_scaled_div_round(max_coarse << 6);
				diff_fine = (s16)pmu_dram_scaled_div_round(max_fine);
			}

			*ptr_14++ = (u8)diff_coarse;
			*ptr_28++ = (u8)diff_fine;

			pmu_cal_metric_log(4, 0x026f0006, (u32)fp, (u32)ph, (u32)(u16)diff_coarse, (u32)fp, (u32)ph, (u32)(u16)diff_fine);
		}
	}

	/* Phase 3: Multi-Lane Deskew Relative Matrix Convergence */
	u32 flag_28 = (flag_78 != 0);
	volatile u8 *ptr_1c = (volatile u8 *)(uintptr_t)((PMU_DMEM_BASE | (PMU_DMEM_CH0_RANK_EN + 7)) + rank_off);
	volatile u8 *ptr_18 = (volatile u8 *)(uintptr_t)((PMU_DMEM_BASE | (PMU_DMEM_CH0_RANK_EN + 3)) + rank_off);

	for (s32 phase_b = 1; phase_b >= 0; phase_b--) {
		for (s32 phase_a = 1; phase_a >= 0; phase_a--) {
			s16 res_diff = 0;
			s16 res_cross = 0;

			if ((phase_a != phase_b) && !flag_28) {
				res_diff = 0;
				res_cross = 0;
			} else {
				u8 start_slice = dmem_read8(PMU_DMEM_SLICE_START);
				u8 end_slice = dmem_read8(PMU_DMEM_SLICE_END);
				s32 max_diff = 0;
				s32 max_cross = 0;

				for (u32 slice = start_slice; slice <= end_slice; slice++) {
					if (desc_flag8 & 1) {
						s32 ref_dly = (s32)lane_tap_steps_b[phase_b][slice];
						for (u32 bit = 0; bit < 8; bit++) {
							s32 samp_dly = (s32)lane_metric_buf_a[phase_a][slice * 8 + bit];
							s32 diff = ref_dly - samp_dly;
							if (diff > max_diff)
								max_diff = diff;
						}
						if (cal_cfg16 & 0x10) {
							s32 aux_dly = (s32)lane_tap_steps_a[phase_a][slice];
							s32 diff = ref_dly - aux_dly;
							if (diff > max_diff)
								max_diff = diff;
						}
					} else {
						for (u32 bit = 0; bit < 8; bit++) {
							s32 tap_a = (s32)lane_metric_buf_a[phase_a][slice * 8 + bit];
							s32 tap_b = (s32)lane_metric_buf_b[phase_b][slice * 8 + bit];
							s32 diff = tap_b - tap_a;
							if (diff > max_diff)
								max_diff = diff;
						}
					}

					if (phase_a != phase_b) {
						for (u32 bit = 0; bit < 8; bit++) {
							s32 cross_a = (s32)lane_metric_buf_a[phase_a][slice * 8 + bit];
							s32 cross_b = (s32)lane_metric_buf_a[phase_b][slice * 8 + bit];
							s32 diff = cross_b - cross_a;
							if (diff > max_cross)
								max_cross = diff;
						}
						if (cal_cfg16 & 0x10) {
							s32 aux_b = (s32)lane_tap_steps_a[phase_b][slice];
							s32 aux_a = (s32)lane_tap_steps_a[phase_a][slice];
							s32 diff = aux_b - aux_a;
							if (diff > max_cross)
								max_cross = diff;
						}
					}
				}

				res_diff = (s16)pmu_dram_scaled_div_round(max_diff);
				res_cross = (s16)pmu_dram_scaled_div_round(max_cross);
			}

			*ptr_18++ = (u8)res_diff;
			*ptr_1c++ = (u8)res_cross;

			pmu_cal_metric_log(4, 0x02720006, (u32)phase_b, (u32)phase_a, (u32)(u16)res_diff, (u32)phase_b, (u32)phase_a, (u32)(u16)res_cross);
		}
	}
}

/**
 * pmu_cal_slice_step_eval_sweep() - Execute slice step evaluation sweep and latch timing delays
 * @results_buf: Pointer to output evaluation matrix buffer (848 bytes per entry)
 * @sweep_mode: Eye margin sweep mode selector
 * @cal_mode: Calibration command mode (read/write/strobe)
 * @channel_rank: Channel and rank index (0..3)
 * @unused: Unused parameter reserved for ABI alignment
 * @timing_val: Timing delay threshold value
 * @overflow_flag: Overflow handling and status flag
 * @slice_delay_lut: Pointer to per-slice delay lookup table
 * @bound_flag: Non-zero to extend pin scan loop to 9 pins (8 pins otherwise)
 * @remap_flag: Pin remapping control flag
 * @sweep_ctx: Pointer to BIST command context structure
 *
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
								   u32 unused,
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
				memset(target, 0, count);
			}
		}
	}

	u32 active_csr_offset = dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET);
	u8 slice_start = dmem_read8(PMU_DMEM_SLICE_START);
	u8 slice_end = dmem_read8(PMU_DMEM_SLICE_END);
	u8 active_mask = dmem_read8(PMU_DMEM_ACTIVE_SLICE_MASK_CAL);

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
					u32 reg = PHY_REG_DBYTE_BASE | (((slice << 12) | active_csr_offset | ctx_off10 | (lane << 8)) << 1);
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
			u32 reg_0c = PHY_REG_DBYTE_BASE | (((slice << 12) | lane_csr_off | active_csr_offset | ctx_off0c) << 1);
			phy_write16(reg_0c, val);
			if (flag_20) {
				u32 reg_10 = PHY_REG_DBYTE_BASE | (((slice << 12) | lane_csr_off | active_csr_offset | ctx_off10) << 1);
				phy_write16(reg_10, val);
			}
		}
	}

	pmu_cal_strobe_secondary_pulse();

	u32 ctx_off04 = *(const u32 *)(ctx + 0x04);
	u16 f6a_val = 1;

	if (channel_rank < 4) {
		if ((profile_mode & ~1) == 2) {
			u32 reg = PHY_REG_DBYTE_BCAST_MODE | (active_csr_offset << 1);
			phy_write16(reg, (channel_rank >= 2) ? 1 : 0);

			if (profile_mode == 3) {
				phy_write16((PHY_REG_MASTER_BASE | 0x0180), 8);
				phy_write16((PHY_REG_MASTER_BASE | 0x0184), 0);
				phy_write16((PHY_REG_MASTER_BASE | 0x0182), (channel_rank > 1) ? 8 : 0);
				f6a_val = 0x4001;
			}
		}
	}

	for (u32 slice = slice_start; slice <= slice_end; slice++) {
		u16 ctx_1c = *(const u16 *)(ctx + 0x1c + (slice * 2));
		u16 cur_f6a = (ctx_1c << 10) | f6a_val;
		phy_write16((PHY_REG_DBYTE_BASE | PHY_REG_DBYTE_LCDL_CTRL) | (slice << 13), cur_f6a);
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
					u16 reg_16a = phy_read16((PHY_REG_DBYTE_BASE | PHY_REG_DBYTE_LCDL_STATUS) | (slice << 13));
					u16 inv = (u16)~(*p1c);
					*p1c = inv;
					u16 val_1f6a = (reg_16a & 0xc3ff) | ((inv << 10) & 0x3c00);
					phy_write16((PHY_REG_DBYTE_BASE | PHY_REG_DBYTE_LCDL_CTRL) | (slice << 13), val_1f6a);
				}
			} else if (cal_mode_even == 2) {
				local_scratch_44[slice] += ctx_off08;
			}

			u32 reg_0c = (PHY_REG_DBYTE_BASE | 0x1e00) | (((slice << 12) | active_csr_offset | ctx_off0c) << 1);
			phy_write16(reg_0c, local_scratch_44[slice]);
			if (flag_20) {
				u32 reg_10 = (PHY_REG_DBYTE_BASE | 0x1e00) | (((slice << 12) | active_csr_offset | ctx_off10) << 1);
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
				u32 sample_reg = (PHY_REG_DBYTE_BASE | 0x016e) | (((slice << 12) | csr_field) << 1);
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

		phy_write16((PHY_REG_DBYTE_BCAST_BASE | 0x0176), 1023);
		phy_write16((PHY_REG_DBYTE_BCAST_BASE | 0x0176), 0);
	}

	/* Restore CSR snapshot from local_buf */
	if (flag_20) {
		for (u32 slice = slice_start; slice <= slice_end; slice++) {
			if (active_mask & (1 << slice)) {
				for (u32 lane = 0; lane < loop_bound; lane++) {
					u32 reg = PHY_REG_DBYTE_BASE | (((slice << 12) | active_csr_offset | ctx_off10 | (lane << 8)) << 1);
					phy_write16(reg, local_buf[slice][lane]);
				}
			}
		}
	}

	/* Epilogue: Restore profile mode */
	if (profile_mode == 2) {
		phy_write16(PHY_REG_DBYTE_BCAST_MODE | (active_csr_offset << 1), 2);
	} else if (profile_mode == 3) {
		phy_write16(PHY_REG_DBYTE_BCAST_MODE | (active_csr_offset << 1), 4);
	}
}

/**
 * pmu_cal_dbyte_dq_status_check() - Verify DBYTE DQ status and configure slice delay and CBT registers
 * @mode_rank: Rank and mode selection flags (bit 1 selects rank 1)
 * @flag: Calibration status verification flag
 *
 * Return: 0 on valid DQ status, error code otherwise.
 */
u32 pmu_cal_dbyte_dq_status_check(u32 mode_rank, u32 flag)
{
	u32 rank_idx = ((mode_rank & 3) == 2) ? 1 : 0;
	u8 channel_idx = dmem_read8(PMU_DMEM_CAL_RANK);
	u32 csr_offset = dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET);
	u32 csr_offset_shl1 = csr_offset << 1;

	u8 cfg_mode = dmem_read8(channel_idx ? 0x4d : 0x32);
	u8 cfg_mode_bit0 = cfg_mode & 1;

	phy_write16(PHY_REG_VREF_CTRL | csr_offset_shl1, 1);
	phy_write16(PHY_REG_VREF_TRIM_CFG, 1);
	u16 reg_000e = phy_read16(PHY_REG_MASTER_CFG | csr_offset_shl1);
	phy_write16(PHY_REG_MASTER_CFG | csr_offset_shl1, reg_000e | 0x100);

	u8 *slice_list = (u8 *)(uintptr_t)((PMU_DMEM_BASE | PMU_DMEM_RANK_SLICE_MAP) + channel_idx * 16 + rank_idx * 8);
	u8 num_slices = dmem_read8(0xb20 + channel_idx * 2 + rank_idx);

	for (u32 i = 0; i < num_slices; i++) {
		u8 slice = slice_list[i];
		u32 slice_off = (u32)slice << 13;
		if (dmem_read8(PMU_DMEM_FREQ_MODE) != 0) {
			phy_write16((PHY_REG_DBYTE_BASE | 0x0112) | slice_off, 511);
			phy_write16((PHY_REG_DBYTE_BASE | 0x0116) | slice_off, 511);
			phy_write16((PHY_REG_DBYTE_BASE | 0x0118) | slice_off, 0);
		} else {
			phy_write16((PHY_REG_DBYTE_BASE | 0x0112) | slice_off, 0x19ff);
			phy_write16((PHY_REG_DBYTE_BASE | 0x0116) | slice_off, 0x19ff);
			phy_write16((PHY_REG_DBYTE_BASE | 0x0118) | slice_off, 0x1000);
		}
	}

	u32 arg0_x5 = (mode_rank << 2) | mode_rank;

	if (flag != 0) {
		u32 not_cfg_mode_bit0 = !cfg_mode_bit0;
		pmu_cal_struct_to_shadow16((PMU_DMEM_BASE | PMU_DMEM_CAL_STRUCT_BASE), 0x10);
		pmu_cal_struct_mask_or((PMU_DMEM_BASE | PMU_DMEM_CAL_STRUCT_BASE), 0x10, 0x41);
		pmu_cal_struct_mask_and((PMU_DMEM_BASE | PMU_DMEM_CAL_STRUCT_BASE), 0x10, 0xf3);
		pmu_cal_struct_to_shadow16((PMU_DMEM_BASE | PMU_DMEM_CAL_STRUCT_BASE), 0x0b);
		pmu_cal_struct_mask_and((PMU_DMEM_BASE | PMU_DMEM_CAL_STRUCT_BASE), 0x0b, 0xf0);
		pmu_cal_struct_to_shadow16((PMU_DMEM_BASE | PMU_DMEM_CAL_STRUCT_BASE), 0x12);
		pmu_cal_struct_mask_or((PMU_DMEM_BASE | PMU_DMEM_CAL_STRUCT_BASE), 0x12, 0x08);
		pmu_cal_struct_mask_and((PMU_DMEM_BASE | PMU_DMEM_CAL_STRUCT_BASE), 0x12, 0xf8);
		pmu_cal_markers_set(1200);
		pmu_cbt_cal_pulse_seq(0);
		dmem_write16(PMU_DMEM_DESKEW_TARGET_DELAY, pmu_cal_stride_get() - 1);
		if (dmem_read8(PMU_DMEM_FREQ_MODE) != 0) {
			pmu_deskew_latch_seq();
			pmu_delay_us(5000, 12);
			pmu_clk_gate_handoff();
		}
		pmu_cal_struct_mask_or((PMU_DMEM_BASE | PMU_DMEM_CAL_STRUCT_BASE), 0x0d, 0x40);

		u32 table_addr = (PMU_DMEM_BASE | PMU_DMEM_CAL_STRUCT_BASE) + channel_idx * 108 + rank_idx * 54;
		u32 a3_mask;
		if (rank_idx == not_cfg_mode_bit0) {
			pmu_cal_struct_to_shadow16((PMU_DMEM_BASE | PMU_DMEM_CAL_STRUCT_BASE), 0x11);
			pmu_cal_struct_mask_or((PMU_DMEM_BASE | PMU_DMEM_CAL_STRUCT_BASE), 0x11, 0x08);
			pmu_cal_struct_mask_and((PMU_DMEM_BASE | PMU_DMEM_CAL_STRUCT_BASE), 0x11, 0xef);
			pmu_cal_struct_mask_or((PMU_DMEM_BASE | PMU_DMEM_CAL_STRUCT_BASE), 0x11, 0x20);

			u8 rank_mask = (u8)((1 << (rank_idx + 2)) | (1 << rank_idx));
			pmu_cal_multi_rank_step_commit(rank_mask, table_addr, 0, 0xfffdffff, (u32)-1, 0, 0);
			pmu_delay_us(2000, 0);
			pmu_tracker_field_extract((u8 *)(PMU_DMEM_BASE | PMU_DMEM_CAL_STRUCT_BASE), 0x11);
			a3_mask = 0xfffbdff7;
		} else {
			a3_mask = 0xfff9d7f7;
		}

		pmu_cal_multi_rank_step_commit((u8)arg0_x5, table_addr, 0, a3_mask, (u32)-1, 0, 0);
		pmu_cal_struct_mask_and((PMU_DMEM_BASE | PMU_DMEM_CAL_STRUCT_BASE), 0x10, 0xf3);
		pmu_cal_struct_mask_or((PMU_DMEM_BASE | PMU_DMEM_CAL_STRUCT_BASE), 0x10, 0x61);
		pmu_cal_struct_mask_and((PMU_DMEM_BASE | PMU_DMEM_CAL_STRUCT_BASE), 0x10, 0x7f);
	} else {
		pmu_cal_struct_mask_and((PMU_DMEM_BASE | PMU_DMEM_CAL_STRUCT_BASE), 0x10, 0xf3);
		pmu_cal_struct_mask_or((PMU_DMEM_BASE | PMU_DMEM_CAL_STRUCT_BASE), 0x10, 0x61);
		pmu_cal_struct_mask_or((PMU_DMEM_BASE | PMU_DMEM_CAL_STRUCT_BASE), 0x10, 0x80);
	}

	phy_write16(PHY_REG_PLL_CLK_CTRL, 1);
	if (cfg_mode_bit0 == rank_idx) {
		u32 table_addr = (PMU_DMEM_BASE | PMU_DMEM_CAL_STRUCT_BASE) + channel_idx * 108 + rank_idx * 54;
		pmu_cbt_3phase_pulse_dispatch((u8)arg0_x5, (const u8 *)table_addr, 16, 1);
	} else {
		u32 dq_offset_sub = ((arg0_x5 & 3) == 2) ? 54 : 0;
		u8 *tbl = (u8 *)(uintptr_t)((PMU_DMEM_BASE | PMU_DMEM_CAL_STRUCT_BASE) + channel_idx * 108 + dq_offset_sub);
		u8 val10 = tbl[0x10];
		u8 dq_pin_mask = (u8)((1 << (cfg_mode_bit0 + 2)) | (1 << cfg_mode_bit0));

		pmu_deskew_and_tracker_reset();
		pmu_ac_lane_profile_setup();
		pmu_cbt_coarse_step_pulse();
		pmu_dbyte_cal_step_latch(0x10, val10, (u8)arg0_x5);
		pmu_cbt_coarse_step_pulse_seq(0, 4, 1);
		u8 new_val10 = (val10 & 138) | 69;
		pmu_dbyte_cal_step_latch(0x10, new_val10, dq_pin_mask);
		pmu_cbt_coarse_step_pulse_seq(0, 0x20, 1);
		pmu_cbt_cal_pulse_seq(0);
		pmu_cbt_3phase_pulse_seq();
		pmu_clk_timing_delay_latch(0, 0);
		pmu_delay_us(0, 0x32);
	}

	if (dmem_read8(PMU_DMEM_FREQ_MODE) != 0) {
		if (num_slices != 0) {
			for (u32 i = 0; i < num_slices; i++) {
				u8 slice = slice_list[i];
				u32 mask = pmu_dbyte_pin_mask_calc(0x80, slice);
				phy_write16((PHY_REG_DBYTE_BASE | 0x0118) | ((u32)slice << 13), (u16)mask);
			}
		}
		pmu_delay_us(5000, 12);
		pmu_phy_profile_param_program();
	} else {
		for (u32 iter = 0; iter < 20; iter++) {
			for (u32 i = 0; i < num_slices; i++) {
				u32 reg = (PHY_REG_DBYTE_BASE | 0x0118) | ((u32)slice_list[i] << 13);
				phy_write16(reg, 0x800);
				phy_write16(reg, 0);
			}
		}

		for (u32 iter = 0; iter < 40; iter++) {
			for (u32 i = 0; i < num_slices; i++) {
				u8 slice = slice_list[i];
				u32 reg = (PHY_REG_DBYTE_BASE | 0x0118) | ((u32)slice << 13);
				u32 mask = pmu_dbyte_pin_mask_calc(0x80, slice);
				phy_write16(reg, (u16)(mask | 0x800));
				phy_write16(reg, (u16)(mask | 0x1000));
			}
		}

		for (u32 i = 0; i < num_slices; i++) {
			u32 slice_off = (u32)slice_list[i] << 13;
			phy_write16((PHY_REG_DBYTE_BASE | 0x0112) | slice_off, 511);
			phy_write16((PHY_REG_DBYTE_BASE | 0x0116) | slice_off, 511);
			phy_write16((PHY_REG_DBYTE_BASE | 0x0118) | slice_off, (u16)pmu_dbyte_pin_mask_calc(0x80, slice_list[i]));
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
		phy_write16((PHY_REG_DBYTE_BASE | 0x0118) | slice_off, (u16)pmu_dbyte_pin_mask_calc(p, slice_list[i]));
	}
	pmu_delay_us(5000, 12);

	for (u32 i = 0; i < num_slices; i++) {
		u8 slice = slice_list[i];
		u32 slice_off = (u32)slice << 13;
		u32 mask = pmu_dbyte_pin_mask_calc(p, slice);
		phy_write16((PHY_REG_DBYTE_BASE | 0x0118) | slice_off, (u16)((mask & 0xff) | 0x100));
	}
	pmu_delay_us(5000, 12);

	for (u32 i = 0; i < num_slices; i++) {
		u32 slice_off = (u32)slice_list[i] << 13;
		u32 mask = pmu_dbyte_pin_mask_calc(0x80, slice_list[i]) | 0x100;
		phy_write16((PHY_REG_DBYTE_BASE | 0x0112) | slice_off, (u16)mask);
		phy_write16((PHY_REG_DBYTE_BASE | 0x0116) | slice_off, (u16)mask);
	}

	pmu_delay_us(0x3d090, 0);
	pmu_tracker_field_extract((u8 *)(PMU_DMEM_BASE | PMU_DMEM_CAL_STRUCT_BASE), 0x10);
	pmu_clk_gate_handoff();

	return mode_rank;
}

/**
 * pmu_cal_dbyte_pattern_loop_exec() - Execute DBYTE pattern test loop across active byte slices
 * @reg_offset: Base register offset within DBYTE slice space
 * @pattern_mode: Pattern test loop mode (2, 3, etc.)
 * @dest_buf: Pointer to destination output buffer
 * @scratch_buf: Pointer to scratch output buffer
 * @bound_start: Loop boundary start parameter
 * @bound_end: Loop boundary end parameter
 * @flags: Auxiliary execution flags
 */
void pmu_cal_dbyte_pattern_loop_exec(u32 reg_offset, u16 pattern_mode, void *dest_buf, void *scratch_buf, u32 bound_start, u32 bound_end, u32 flags)
{
	u16 *out_dest = (u16 *)dest_buf;
	u16 *out_scratch = (u16 *)scratch_buf;
	u8 slice_start = dmem_read8(PMU_DMEM_SLICE_START);
	u8 slice_end = dmem_read8(PMU_DMEM_SLICE_END);
	u8 active_mask = dmem_read8(PMU_DMEM_ACTIVE_SLICE_MASK_CAL);

	if (pattern_mode == 2 || pattern_mode == 3) {
		u16 local_buf[4] = {0};
		pmu_cal_slice_step_diff_commit(reg_offset, local_buf, 1);
		u32 csr_offset = dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET);
		for (u32 slice = slice_start; slice <= slice_end; slice++) {
			u32 addr = PHY_REG_DBYTE_BASE | (((slice << 12) | csr_offset | (reg_offset + 0x2a)) << 1);
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

	if (pattern_mode >= 2) {
		pmu_assert_or_halt(0, 0x27e0001);
		return;
	}

	u16 freq = dmem_read16(PMU_DMEM_DRAM_FREQ_OFF);
	if (freq <= 1599) {
		u16 init_val = (dmem_read8(PMU_DMEM_FREQ_MODE) < 2) ? 0x41 : 0xc1;
		for (u32 slice = slice_start; slice <= slice_end; slice++) {
			if (active_mask & (1 << slice)) {
				out_scratch[slice] = init_val;
			}
		}
	} else {
		u32 base_reg = (pattern_mode != 0) ? (PHY_REG_DBYTE_BASE | 0x0024) : (PHY_REG_DBYTE_BASE | 0x0020);
		u8 base_fp = (flags == 0) ? 0x40 : 0x80;
		u32 num_lanes = (pmu_dmem_training_flags_eval() != 0) ? 9 : 8;
		u32 csr_offset = dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET);

		for (u32 slice = slice_start; slice <= slice_end; slice++) {
			if (!(active_mask & (1 << slice))) {
				continue;
			}

			if (bound_end != 0) {
				u16 min_val = 0xffff;
				u16 max_val = 0;
				u32 slice_base = (slice << 12) | csr_offset | reg_offset;
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
			if (reg_offset == 0) {
				if (slice <= 3) {
					dmem_addr = 0x127 + slice;
				}
			} else if (reg_offset == 1) {
				if (slice <= 2) {
					dmem_addr = 0x12b + slice;
				}
			}

			u8 sub_val = base_fp + dmem_read8(dmem_addr);
			if (dmem_read8(PMU_DMEM_CAL_MISC_FLAGS) & 0x20) {
				u8 pattern_bytes[4];
				pmu_dly_line_unpack(out_scratch[slice], pattern_bytes);
				u32 dev = pmu_lcdl_dev_calc(pattern_bytes[1]);
				u8 flag = ((dev < 0x21) ^ (pattern_bytes[0] & 1)) ? 1 : 0;
				dmem_write8(0x474 + slice + (reg_offset << 2), flag);
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

		u8 pattern_bytes[4];
		pmu_dly_line_unpack(out_scratch[slice], pattern_bytes);

		if (dmem_read8(PMU_DMEM_HIGH_FREQ_FLAG) != 0) {
			pmu_delay_tap_step_adjust(pattern_bytes, bound_start);
		}

		if (dmem_read8(PMU_DMEM_CAL_MISC_FLAGS) & 0x20) {
			pmu_delay_tap_step_adjust(pattern_bytes, dmem_read8(0x474 + slice + (reg_offset << 2)));
		}

		out_dest[slice] = (u16)pmu_hdr_addr_15b_decode(pattern_bytes);
	}
}

/**
 * pmu_cal_slice_step_diff_commit() - Compute slice step differentials and commit to PHY registers
 * @slice_idx: Target slice or channel index
 * @out_buf: Pointer to destination buffer receiving computed adjustments
 * @cal_mode: Calibration step mode selector
 */
void pmu_cal_slice_step_diff_commit(u32 slice_idx, u16 *out_buf, u32 cal_mode)
{
	u16 temp_buf[4] = {0};
	u32 slice_sub_idx = (1 << slice_idx) & 3;
	u8 slice_pair_mask = (u8)((1 << (slice_idx + 2)) | (1 << slice_idx));
	u8 start_slice = dmem_read8(PMU_DMEM_SLICE_START);
	u8 end_slice = dmem_read8(PMU_DMEM_SLICE_END);

	phy_write16((PHY_REG_APB_BASE | 0x0148), 1);
	pmu_deskew_and_tracker_reset();

	pmu_cal_sequence_pulse_send(0, 0xb, 0, 0, 0, slice_sub_idx, 0);
	pmu_cbt_coarse_step_pulse_seq(0, 0xff, 0);
	pmu_cal_sequence_pulse_send(0, 6, 0x22, 0x40, 0x25, slice_pair_mask, 0);
	pmu_cal_sequence_pulse_send(0x80, 0x19, 4, 0x81, 0, slice_sub_idx, 0);
	pmu_clk_timing_delay_latch(0, 1);
	pmu_hw_timer_delay(0x800);
	pmu_delay_us(0x9c40, 8);
	pmu_cal_window_valid_check(slice_pair_mask, 0x23);

	for (u32 slice = start_slice; slice <= end_slice; slice++) {
		if (pmu_dq_swap_polarity_check(slice_idx, slice) != 0) {
			out_buf[slice] = (u16)pmu_slice_dq_bitmask_swizzle(slice, 4, slice_idx);
		}
	}

	pmu_cal_window_valid_check(slice_pair_mask, 0x24);
	for (u32 slice = start_slice; slice <= end_slice; slice++) {
		if (pmu_dq_swap_polarity_check(slice_idx, slice) != 0) {
			u16 val = (u16)pmu_slice_dq_bitmask_swizzle(slice, 4, slice_idx);
			u32 combined = ((u32)val << 8) | out_buf[slice];
			out_buf[slice] = (u16)combined;
			if (dmem_read16(PMU_DMEM_DRAM_FREQ_OFF) <= 199 || (u16)combined < 2) {
				out_buf[slice] = 0xffff;
			} else {
				out_buf[slice] = (u16)(((u32)dmem_read8(PMU_DMEM_DRAM_TYPE) << 17) / (u16)combined);
			}
		}
	}

	pmu_deskew_and_tracker_reset();
	pmu_cbt_coarse_step_pulse_seq(0, 0xff, 0);
	pmu_cal_sequence_pulse_send(0, 6, 0x22, 0x40, 0x28, slice_pair_mask, 0);
	pmu_cal_sequence_pulse_send(0x80, 0x19, 4, 0x83, 0, slice_sub_idx, 0);
	pmu_clk_timing_delay_latch(0, 1);
	pmu_hw_timer_delay(0x800);
	pmu_delay_us(0x9c40, 8);
	pmu_cal_window_valid_check(slice_pair_mask, 0x26);

	for (u32 slice = start_slice; slice <= end_slice; slice++) {
		if (pmu_dq_swap_polarity_check(slice_idx, slice) != 0) {
			temp_buf[slice] = (u16)pmu_slice_dq_bitmask_swizzle(slice, 4, slice_idx);
		}
	}

	pmu_cal_window_valid_check(slice_pair_mask, 0x27);
	for (u32 slice = start_slice; slice <= end_slice; slice++) {
		if (pmu_dq_swap_polarity_check(slice_idx, slice) != 0) {
			u16 val = (u16)pmu_slice_dq_bitmask_swizzle(slice, 4, slice_idx);
			u32 combined = ((u32)val << 8) | temp_buf[slice];
			temp_buf[slice] = (u16)combined;
			u16 final_val;
			if (dmem_read16(PMU_DMEM_DRAM_FREQ_OFF) <= 199 || (u16)combined < 2) {
				final_val = 0xffff;
			} else {
				final_val = (u16)(((u32)dmem_read8(PMU_DMEM_DRAM_TYPE) << 17) / (u16)combined);
			}
			temp_buf[slice] = final_val;
			pmu_cal_metric_log(4, 0x2460003, slice, out_buf[slice], final_val);
		}
	}

	for (u32 slice = start_slice; slice <= end_slice; slice++) {
		if (pmu_dq_swap_polarity_check(slice_idx, slice) == 0) {
			u32 neighbor = (slice & 1) ? (slice - 1) : (slice + 1);
			out_buf[slice] = out_buf[neighbor];
			temp_buf[slice] = temp_buf[neighbor];
		}
	}

	if (cal_mode != 0 && end_slice >= start_slice) {
		u32 reg_off1 = (slice_idx == 0) ? 0x1000c : 0x1000d;
		u32 reg_off2 = (slice_idx == 0) ? 0x10015 : 0x10016;
		u32 csr_offset = dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET);

		for (u32 slice = start_slice; slice <= end_slice; slice++) {
			u32 slice_base = (slice << 12) | csr_offset;
			phy_write16(PHY_REG_APB_BASE | ((slice_base | reg_off1) << 1), out_buf[slice]);
			phy_write16(PHY_REG_APB_BASE | ((slice_base | reg_off2) << 1), temp_buf[slice]);
		}
	}

	pmu_deskew_and_tracker_reset();
	pmu_cal_sequence_pulse_send(0, 0xc, 4, 0, 0, slice_sub_idx, 0);
	pmu_cal_sequence_pulse_send(0, 7, 6, 0, 0, 0, 0xff);
	pmu_cal_sequence_pulse_send(0, 0x10, 4, 0, 0, slice_sub_idx, 0);
	pmu_cal_sequence_pulse_send(0x80, 7, 6, 0, 0, 0, 0xff);
	pmu_clk_timing_delay_latch(0, 1);
	pmu_cal_strobe_secondary_pulse();
}

/**
 * pmu_cal_channel_rank_boundary_eval() - Evaluate timing boundaries for channel and rank
 * @buf: Pointer to boundary evaluation data buffer in DMEM
 * @channel: Memory channel index (0 or 1)
 * @rank: DRAM rank index (0 or 1)
 * @cal_type: Calibration boundary type identifier
 */
void pmu_cal_channel_rank_boundary_eval(u8 *buf, u32 channel, u32 rank, u32 cal_type)
{
	u8 *ctrl = buf + 512;
	u32 lut_val;
	u8 fp_val;
	u8 rank_start, rank_end;
	u32 active_csr_offset;

	lut_val = pmu_2b_identity_lut(channel);
	memset(buf, 0, 526);

	fp_val = (rank != 0) ? 9 : 8;

	if (cal_type == 0x11) {
		u8 dmem_102 = dmem_read8(PMU_DMEM_TRAIN_FEATURE_MASK);

		ctrl[12] = 0;
		*(u16 *)&ctrl[8] = 0;
		*(u16 *)&ctrl[6] = 1;
		ctrl[10] = fp_val;
		*(u16 *)&ctrl[2] = 0x62;
		*(u16 *)&ctrl[4] = 0x64;
		pmu_dbyte_dq_deskew_regs_restore();

		ctrl[13] = 1;
		((void (*)(u32, u16, void *, void *, u32, u32, u32))(uintptr_t)pmu_cal_dbyte_pattern_loop_exec)(channel, *(u16 *)&ctrl[6], buf, buf + 16, 0, 1, 0);
		((void (*)(u32, u16, void *, void *, u32, u32, u32))(uintptr_t)pmu_cal_dbyte_pattern_loop_exec)(channel, *(u16 *)&ctrl[8], buf + 8, buf + 24, 0, 1, 0);

		rank_start = dmem_read8(PMU_DMEM_SLICE_START);
		rank_end = dmem_read8(PMU_DMEM_SLICE_END);

		for (u32 i = rank_start; i <= rank_end; i++) {
			u16 *ptr_lower = (u16 *)(buf + (i * 2));
			u16 *ptr_upper = (u16 *)(buf + 8 + (i * 2));
			u32 param24 = 0;
			u32 param28 = 0;

			pmu_cal_param_unpack(*ptr_lower, (u8 *)&param24);
			pmu_cal_param_unpack(*ptr_upper, (u8 *)&param28);

			u8 tap_lower = *(u8 *)&param24;
			u8 tap_upper = *(u8 *)&param28;

			if (tap_lower > tap_upper) {
				pmu_delay_tap_step_adjust((u8 *)&param24, !(tap_upper & 1));
				*ptr_lower = pmu_hdr_addr_15b_decode((u8 *)&param24);
			} else if (tap_lower < tap_upper) {
				pmu_delay_tap_step_adjust((u8 *)&param28, !(tap_lower & 1));
				*ptr_upper = pmu_hdr_addr_15b_decode((u8 *)&param28);
			}
		}

		if (dmem_read8(PMU_DMEM_BIST_PATTERN_LO) != 0 || dmem_read8(PMU_DMEM_DESKEW_OFFSET_OVERRIDE) != 0) {
			buf[523] = 7;
		} else {
			buf[523] = 4;
		}

		if (dmem_102 & (1 << 1)) {
			u8 ret = pmu_cal_profile_mode_get();
			ctrl[0] = ret;
			if (ret == 3) {
				u32 gp_41c = dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET);
				phy_write16(PHY_REG_DBYTE_BCAST_MODE | (gp_41c << 1), 0);
			}
		}

		active_csr_offset = dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET);
		for (u32 i = rank_start; i <= rank_end; i++) {
			for (u32 j = 0; j < 9; j++) {
				u32 offset = active_csr_offset | (i << 12) | (j << 8);

				u16 val1 = phy_read16(PHY_REG_DBYTE_BASE | (((lut_val + 0x12) | offset) << 1));
				*(u16 *)(buf + 0x20 + (i * 20) + (j * 2)) = val1;

				u16 val2 = phy_read16(PHY_REG_DBYTE_BASE | (((lut_val + 0x10) | offset) << 1));
				*(u16 *)(buf + 112 + (i * 20) + (j * 2)) = val2;

				if ((dmem_102 & (1 << 1)) && (ctrl[0] == 3)) {
					u32 csr_base = (PHY_REG_DBYTE_BASE | 0x009c) | (offset << 1);
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

		u8 nib = (u8)pmu_dmem_train_param_nibble_get();
		if (nib != 0)
			fp_val = 10;

		ctrl[10] = fp_val;
		ctrl[12] = nib;

		((void (*)(u32, u16, void *, void *, u32, u32, u32))(uintptr_t)pmu_cal_dbyte_pattern_loop_exec)(channel, *(u16 *)&ctrl[6], buf, buf + 16, 0, 1, 0);

		rank_start = dmem_read8(PMU_DMEM_SLICE_START);
		rank_end = dmem_read8(PMU_DMEM_SLICE_END);
		active_csr_offset = dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET);

		for (u32 i = rank_start; i <= rank_end; i++) {
			for (u32 j = 0; j < 9; j++) {
				u32 offset = active_csr_offset | (lut_val + 0x26) | (i << 12) | (j << 8);
				u16 csr_val = phy_read16(PHY_REG_DBYTE_BASE | (offset << 1));

				*(u16 *)(buf + 0x20 + (i * 20) + (j * 2)) = csr_val;
				pmu_cal_metric_log(4, 0x15f0003, i, j, csr_val);
			}

			u32 offset = active_csr_offset | (lut_val + 0x28) | (i << 12);
			u16 csr_val = phy_read16(PHY_REG_DBYTE_BASE | (offset << 1));
			*(u16 *)(buf + 0x20 + (i * 20) + (9 * 2)) = csr_val;
		}
	}
}

/**
 * copy_slice_quad() - Copy 4-byte slice parameters from source to DMEM destination
 * @buf: Destination buffer in DMEM
 * @off: Byte offset within destination buffer
 * @src: Pointer to 4-byte source array
 */
void copy_slice_quad(u8 *buf, u32 off, const volatile u8 *src)
{
	buf[off] = src[0];
	buf[off + 54] = src[1];
	buf[off + 108] = src[2];
	buf[off + 162] = src[3];
}

/**
 * pmu_cal_slice_step_scan_eval() - Evaluate slice step scan and generate calibration pulses
 * @step_val: Calibration delay step value
 * @slice_mask: Target DBYTE slice bitmask
 *
 * Return: Step scan evaluation status code.
 */
u32 pmu_cal_slice_step_scan_eval(u32 step_val, u32 slice_mask)
{
	u32 vref_code = phy_read16(PHY_REG_VREF_STAT) & 0x7f;
	if (vref_code == 0) {
		pmu_cal_metric_log(4, 0x028e0000);
		vref_code = 1;
	}

	u32 vref_scaled = vref_code * 3;
	u32 vref_shifted = vref_code << 2;
	s16 diff_bound = (s16)(vref_shifted + (u16)(vref_scaled ^ 0xffff));
	if (diff_bound < 0) {
		u32 mult = 1;
		s16 not_r13 = (s16)~vref_scaled;
		do {
			mult++;
			diff_bound = (s16)(vref_shifted * mult + not_r13);
		} while (diff_bound < 0);
	}
	u8 bound_low = (u8)diff_bound;

	pmu_deskew_and_tracker_reset();
	pmu_cal_metric_log(4, 0x028f0001, vref_code);
	pmu_cbt_cal_stat_set();

	u8 vref_scaled_dec = (u8)(vref_code * 10 - 1);
	pmu_cal_sequence_pulse_send(0, 7, 0, 0, 0, 0, vref_scaled_dec);

	u8 bound_high = (u8)(vref_scaled + 1);
	pmu_multiphase_pulse_seq_repeat(7, 0, 0, bound_high);

	u32 fp_val = 10;
	pmu_multiphase_pulse_seq_repeat(0x1d, 0x12c8, slice_mask, fp_val);
	pmu_multiphase_pulse_seq_repeat(0x1e, 0x12c8, slice_mask, fp_val);
	pmu_multiphase_pulse_seq_repeat(0x1f, 0x12c8, slice_mask, fp_val);
	pmu_multiphase_pulse_seq_repeat(0x20, 0x12c8, 0, fp_val);

	pmu_multiphase_pulse_seq_repeat(7, 0, 0, bound_low);
	pmu_cal_sequence_pulse_send(0, 7, 0, 0, 0, 0, vref_scaled_dec);
	pmu_cal_sequence_pulse_send(0, 7, 4, 0, 0, 0, (u8)(fp_val - 1));

	for (int i = 0; i < 8; i++) {
		pmu_cal_pulse_burst_send(1, vref_code, slice_mask);
		pmu_cal_pulse_burst_send(0, vref_code, slice_mask);
	}

	diff_bound = 1;
	pmu_cal_sequence_pulse_send(0, 7, 0x5c, 0, 0, 0, 0);
	pmu_cal_sequence_pulse_send(0x40004, 0x2e, step_val, 0x1000, 0, 0, 0);
	pmu_cal_sequence_pulse_send(0x40004, 0x2f, step_val, 0x1000, 0, 0, 0);
	pmu_cal_sequence_pulse_send(0x8000, 7, 4, 0, 0, 0, 1);

	u32 step_target = (step_val * 2) + 100;
	s32 target_diff = (s32)vref_shifted - (s32)step_target;
	while (target_diff < 0) {
		diff_bound++;
		target_diff = (s16)((vref_shifted * (u16)diff_bound) - step_target);
		pmu_cal_metric_log(4, 0x02900003, (u16)diff_bound, step_target, (s16)target_diff);
	}
	pmu_cal_metric_log(4, 0x02910002, (s16)target_diff, step_target << ((u32)target_diff & 0x1f));

	pmu_cal_sequence_pulse_send(0, 7, 0, 0, 0, 0, vref_scaled_dec);
	pmu_multiphase_pulse_seq_repeat(7, 0, 0, (u8)(target_diff + bound_high));

	u32 post_param = 0x25 << 7; /* 0x1280 */
	pmu_multiphase_pulse_seq_repeat(0x1d, post_param, slice_mask, fp_val);
	pmu_multiphase_pulse_seq_repeat(0x1e, post_param, slice_mask, fp_val);
	pmu_multiphase_pulse_seq_repeat(0x1f, post_param, slice_mask, fp_val);
	pmu_multiphase_pulse_seq_repeat(0x20, post_param, 0, fp_val);

	pmu_multiphase_pulse_seq_repeat(7, 0, 0, bound_low);
	pmu_cal_sequence_pulse_send(0, 7, 0, 0, 0, 0, vref_scaled_dec);
	pmu_cbt_cal_stat_clear();

	return 0xff;
}

/**
 * pmu_cal_rank_param_commit() - Commit calibrated timing parameters for channel and rank
 * @channel: Memory channel index (0 or 1)
 * @rank: DRAM rank index (0 or 1)
 * @slice: DBYTE slice index (0..3)
 * @mode: Operating calibration mode
 */
void pmu_cal_rank_param_commit(u32 channel, u32 rank, u32 slice, u32 mode)
{
	u8 dmem_20 = dmem_read8(PMU_DMEM_CH0_CTRL_20) & 0x1f;
	u8 dmem_21 = dmem_read8(PMU_DMEM_CH0_CTRL_21) & 0x1f;
	u8 dmem_17 = dmem_read8(PMU_DMEM_TRAIN_CTRL_17) & 0x1f;
	u8 dmem_18 = dmem_read8(PMU_DMEM_TRAIN_CTRL_18) & 0x1f;

	dmem_write8(PMU_DMEM_CH0_CTRL_20, dmem_20);
	dmem_write8(PMU_DMEM_CH0_CTRL_21, dmem_21);
	dmem_write8(PMU_DMEM_TRAIN_CTRL_17, dmem_17);
	dmem_write8(PMU_DMEM_TRAIN_CTRL_18, dmem_18);

	if ((dmem_20 | dmem_21) == 0) {
		dmem_write8(PMU_DMEM_CH0_CTRL_20, 1);
		dmem_write8(PMU_DMEM_CH0_CTRL_21, 1);
	}

	if ((dmem_17 | dmem_18) == 0) {
		dmem_write8(PMU_DMEM_TRAIN_CTRL_17, 1);
		dmem_write8(PMU_DMEM_TRAIN_CTRL_18, 1);
	}

	u32 flag_10 = (mode != 1) ? 1 : 0;
	u32 flag_14 = (mode <= 1) ? 1 : 0;
	u32 commit_flag = (mode == 2) ? 1 : 0;

	u32 flag_28 = (rank != 0) ? 1 : 0;
	u8 dmem_ce = dmem_read8(PMU_DMEM_CAL_OVERRIDE_FLAG);
	u32 flag_1c = (dmem_ce == 0) ? dmem_read8(PMU_DMEM_DQ_SWAP_BASE) : 0;
	u32 count_24 = (dmem_read8(PMU_DMEM_HIGH_FREQ_FLAG) != 0) ? 2 : 1;

	for (u32 i = 0; i < count_24; i++) {
		if (mode == 1)
			pmu_cal_metric_log(4, 0x43 << 17);
		else if (mode == 2)
			pmu_cal_metric_log(4, 0x11 << 19);
		else if (mode == 0)
			pmu_cal_metric_log(4, 0x87 << 16);

		pmu_cal_metric_log(4, 0x890001, count_24);

		pmu_cal_margin_matrix_scan_eval(channel, rank, slice, mode, flag_1c, (u8)i);

		if (dmem_read8(PMU_DMEM_CAL_SELECT_FLAGS) & 1) {
			pmu_cal_metric_log(4, 0x45 << 17);
			pmu_cal_metric_table_sweep_log(PMU_DMEM_BASE | PMU_DMEM_EYE_SAMPLE_BUF, commit_flag, slice);
		}

		if (dmem_ce == 0) {
			pmu_cal_metric_log(4, 0x8b << 16);
			u32 ret_9f28 = pmu_cal_rank_margin_window_check(commit_flag, 0, slice);
			if (ret_9f28 == 0)
				ret_9f28 = pmu_cal_rank_margin_window_check(commit_flag, 1, slice);
			pmu_cal_metric_log(4, 0x23 << 18);
			pmu_cal_metric_table_sweep_log(PMU_DMEM_BASE | PMU_DMEM_EYE_SAMPLE_BUF, commit_flag, slice);
			pmu_profile_mailbox_cmd_dispatch(ret_9f28);
		}

		if (rank != 0)
			pmu_rank_slice_margin_eval();

		if (flag_14) {
			pmu_cal_metric_log(4, 0x47 << 17);
			pmu_cal_dly_line_tap_step_adjust(flag_10, (u8)i, slice, flag_28);

			if (dmem_read8(PMU_DMEM_HIGH_FREQ_FLAG) != 0) {
				u8 temp_bits = dmem_read8(PMU_DMEM_CH0_RANK_EN) | dmem_read8(PMU_DMEM_CH1_RANK_EN);
				if (i == 0) {
					for (u32 k = 0; k <= 1; k++) {
						if (temp_bits & (1 << k)) {
							u32 lut_k = pmu_2b_identity_lut(k);
							pmu_lcdl_delay_profile_update(0, dmem_read8(PMU_DMEM_CHANNEL_CFG), lut_k, k);
						}
					}
				} else if (flag_10 != 0) {
					for (u32 k = 0; k <= 1; k++) {
						if (temp_bits & (1 << k)) {
							u32 lut_k = pmu_2b_identity_lut(k);
							pmu_lcdl_delay_profile_update(0, dmem_read8(PMU_DMEM_CHANNEL_CFG), lut_k, k);
						}
					}
					pmu_cal_metric_log(4, 0x8f << 16);
					pmu_cal_delay_line_accum_dispatch(slice);
				}
			}
		} else {
			pmu_cal_metric_log(4, 0x8d << 16);
			pmu_cal_dbyte_deskew_pin_results_apply();
		}
	}
}

/*
 * pmu_cal_multi_lane_deskew_sweep:
 */
/**
 * pmu_cal_multi_lane_deskew_sweep() - Execute multi-lane deskew calibration sweep across active slices
 * @type: Calibration type selector (0..3) configuring search bounds
 * @ch_mask: Bitmask of active memory channels
 * @lane_mask: Bitmask of active bit lanes
 */
void pmu_cal_multi_lane_deskew_sweep(u32 type, u32 ch_mask, u32 lane_mask)
{
	u16 dly_buf[4][10];
	u16 lane_metric_max[4] = {0};
	u16 lane_metric_min[4] = {0};
	u32 desc_ec[9];

	memset(desc_ec, 0, sizeof(desc_ec));

	u32 is_lpddr5_type = (type == 3) ? 1 : 0;
	u32 profile_stride = (type & 2) ? 0xd4 : 0x80;
	u32 profile_base_addr = 0x10010 + ((type & 2) ? 0x16 : 0) + ((type & 1) ? 2 : 0);

	u32 freq_lt_3200 = (dmem_read16(PMU_DMEM_DRAM_FREQ_OFF) < 3200) ? 1 : 0;
	u32 active_lane_mask = (lane_mask >> 8) & 0x7f;
	u32 active_mask = (u32)dmem_read8(PMU_DMEM_CH1_RANK_EN) | (u32)dmem_read8(PMU_DMEM_CH0_RANK_EN);

	for (u32 ch = 0; ch < 2; ch++) {
		u32 ch_active_mask = (1 << (ch + 2)) | (1 << ch);
		if (!(ch_active_mask & active_mask))
			continue;

		dmem_write8(PMU_DMEM_CAL_BYTE, (u8)ch);
		pmu_phy_mode_cfg_dispatch(3);

		u8 slice_start = dmem_read8(PMU_DMEM_SLICE_START);
		u8 slice_end = dmem_read8(PMU_DMEM_SLICE_END);
		u8 active_slice_mask = dmem_read8(PMU_DMEM_ACTIVE_SLICE_MASK_CAL);
		u32 ch_offset = dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET);

		/* Phase 1: Cache previous delay values across active slices and lanes */
		for (u32 slice = slice_start; slice <= slice_end; slice++) {
			if (!(active_slice_mask & (1 << slice)))
				continue;

			u32 slice_offset = (slice << 12) | ch_offset | profile_base_addr | ch;
			uintptr_t base_addr = PHY_REG_APB_BASE | (slice_offset << 1);

			for (u32 lane = 0; lane < 10; lane++) {
				if (lane == 8 && !active_lane_mask)
					continue;

				if (is_lpddr5_type != 0) {
					if (lane != 9)
						continue;
					dly_buf[slice][9] = phy_read16(base_addr);
				} else {
					if (lane == 9)
						continue;
					uintptr_t reg_addr = PHY_REG_APB_BASE | ((slice_offset | (lane << 8)) << 1);
					dly_buf[slice][lane] = phy_read16(reg_addr);
				}
			}
		}

		/* Phase 2: Execute calibration pattern loop & delay line scan */
		pmu_cal_dbyte_pattern_loop_exec(ch, (u16)type, lane_metric_max, lane_metric_min, 0, 1, 0);

		if (is_lpddr5_type != 0)
			pmu_phy_timing_delay_latch(1, ch_active_mask);

		pmu_cal_bist_cmd_strobe_dispatch((u32)(uintptr_t)desc_ec, 8, type, ch_active_mask, (u32)(uintptr_t)lane_metric_max, active_lane_mask, 0);

		memset((void *)(PMU_DMEM_BASE | PMU_DMEM_SWEEP_SAMPLE_BUF), 0, 0x8480);

		pmu_cal_slice_step_eval_sweep((void *)(PMU_DMEM_BASE | PMU_DMEM_SWEEP_SAMPLE_BUF), 8, type, 4,
				0, (u8)ch_mask, freq_lt_3200,
				lane_metric_max, active_lane_mask, 0, desc_ec);

		pmu_dmem_reg_stream_unpack(desc_ec);

		if (is_lpddr5_type != 0)
			pmu_phy_timing_delay_latch(0, ch_active_mask);

		/* Phase 3: Margin evaluation, boundary adjustment & CSR programming */
		for (u32 slice = slice_start; slice <= slice_end; slice++) {
			if (!(active_slice_mask & (1 << slice)))
				continue;

			u32 slice_off = slice << 12;
			u32 reg_base_ch = slice_off | ch_offset | ch;
			u32 reg_base_var = slice_off | ch_offset | profile_base_addr | ch;
			u32 slice_offset_val = slice_off | ch_offset | (ch << 1);

			uintptr_t reg_sp_78 = (PHY_REG_DBYTE_BASE | 0x0044) | (reg_base_ch << 1);
			uintptr_t reg_sp_7c = (PHY_REG_DBYTE_BASE | 0x0032) | (slice_offset_val << 1);
			uintptr_t reg_sp_132 = PHY_REG_APB_BASE | (reg_base_var << 1);

			for (u32 lane = 0; lane < 10; lane++) {
				if (lane == 8 && !active_lane_mask)
					continue;

				if (is_lpddr5_type) {
					if (lane != 9)
						continue;
				} else {
					if (lane == 9)
						continue;
				}

				u32 lane_idx = slice * 10 + lane;
				u32 sample_offset = lane_idx * 848;
				u8 blink_flag = *(volatile const u8 *)(uintptr_t)((PMU_DMEM_BASE | 0x6594) + sample_offset);

				u32 right_edge = 0;
				u32 left_edge = 0;

				if (blink_flag != 0) {
					const volatile u8 *s_ptr = (const volatile u8 *)(uintptr_t)((PMU_DMEM_BASE | PMU_DMEM_SWEEP_SAMPLE_BUF) + sample_offset);
					u8 sample_low = s_ptr[0];
					if (blink_flag == 1) {
						right_edge = profile_stride;
						left_edge = sample_low;
					} else if (blink_flag == 3) {
						u8 sample_mid = s_ptr[1];
						u8 sample_high = s_ptr[2];
						u32 diff_sample = (u32)sample_mid - (u32)sample_low;
						u32 diff_limit = profile_stride - (u32)sample_high;
						if (diff_limit > diff_sample) {
							right_edge = profile_stride;
							left_edge = sample_high;
						} else {
							right_edge = sample_mid;
							left_edge = sample_low;
						}
					} else {
						right_edge = s_ptr[1];
						left_edge = sample_low;
					}
				}

				u16 cal_val_raw = lane_metric_max[slice];
				u16 cur_val_raw = dly_buf[slice][lane];
				u32 raw_metric_0, raw_metric_1;

				if (type >= 2) {
					raw_metric_0 = cur_val_raw;
					raw_metric_1 = cal_val_raw;
				} else {
					raw_metric_0 = (cur_val_raw & 0x7f) + ((cur_val_raw >> 1) & ~0x3f);
					raw_metric_1 = (cal_val_raw & 0x7f) + ((cal_val_raw >> 1) & ~0x3f);
				}

				u16 metric_val_0 = (u16)raw_metric_0;
				u16 metric_val_1 = (u16)raw_metric_1;
				right_edge += metric_val_1;

				s32 term_diff = (s32)metric_val_0 - (s32)metric_val_1;
				s32 term_0 = term_diff - (s32)left_edge;
				s32 term_12 = (s32)right_edge - (s32)metric_val_0;

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
					uintptr_t addr_d0 = (PHY_REG_DBYTE_BASE | 0x00d0) | lane_reg_off;
					uintptr_t addr_d4 = (PHY_REG_DBYTE_BASE | 0x00d4) | lane_reg_off;
					u16 reg_d0 = phy_read16(addr_d0);
					u16 reg_d4 = phy_read16(addr_d4);

					u8 limit_d0 = (u8)reg_d0;
					if ((u16)val_r12 > limit_d0)
						val_r12 = limit_d0;

					uintptr_t addr_24 = (PHY_REG_DBYTE_BASE | 0x0024) | lane_reg_off;
					uintptr_t addr_20 = (PHY_REG_DBYTE_BASE | 0x0020) | lane_reg_off;
					u16 reg_24 = phy_read16(addr_24);
					u16 reg_20 = phy_read16(addr_20);

					u8 limit_d4 = (u8)reg_d4;
					u8 cal_margin_offset = dmem_read8(PMU_DMEM_SEARCH_WINDOW_BIAS_HI);

					if ((u16)val_r15 > limit_d4)
						val_r15 = limit_d4;

					if (cal_margin_offset != 0) {
						u16 bmsk_24 = reg_24 & 0x7f;
						u16 bmsk_20 = reg_20 & 0x7f;

						if (!((bmsk_24 > (u16)val_r12) && (bmsk_20 > (u16)val_r12))) {
							if (bmsk_24 >= bmsk_20)
								val_r12 = (s32)bmsk_20 - (s32)cal_margin_offset;
							else
								val_r12 = (s32)bmsk_24 - (s32)cal_margin_offset;
						}

						u32 sum_24 = (u16)val_r15 + bmsk_24;
						u32 sum_20 = (u16)val_r15 + bmsk_20;

						if ((sum_24 > 0x7e) || (sum_20 >= 0x7f)) {
							if (bmsk_20 >= bmsk_24)
								val_r15 = (s32)(bmsk_20 ^ 0x7f) - (s32)cal_margin_offset;
							else
								val_r15 = (s32)(bmsk_24 ^ 0x7f) - (s32)cal_margin_offset;
						}
					}

					phy_write16(addr_d0, (u16)val_r12);
					phy_write16((PHY_REG_DBYTE_BASE | 0x00d8) | lane_reg_off, (u16)val_r12);
					phy_write16(addr_d4, (u16)val_r15);
					phy_write16((PHY_REG_DBYTE_BASE | 0x00dc) | lane_reg_off, (u16)val_r15);
				} else if (type == 2) {
					uintptr_t addr_38 = (PHY_REG_DBYTE_BASE | 0x0038) | lane_reg_off;
					uintptr_t addr_3c = (PHY_REG_DBYTE_BASE | 0x003c) | lane_reg_off;
					phy_write16(addr_38, (u16)val_r12);
					phy_write16(addr_3c, (u16)val_r15);
				} else if (type == 1) {
					uintptr_t addr_d0 = (PHY_REG_DBYTE_BASE | 0x00d0) | lane_reg_off;
					uintptr_t addr_d4 = (PHY_REG_DBYTE_BASE | 0x00d4) | lane_reg_off;
					phy_write16(addr_d0, (u16)val_r12);
					phy_write16((PHY_REG_DBYTE_BASE | 0x00d8) | lane_reg_off, (u16)val_r12);
					phy_write16(addr_d4, (u16)val_r15);
					phy_write16((PHY_REG_DBYTE_BASE | 0x00dc) | lane_reg_off, (u16)val_r15);
				} else {
					/* type == 3 */
					phy_write16(reg_sp_7c, (u16)val_r12);
					phy_write16(reg_sp_78, (u16)val_r15);
				}

				/* Restore original delay value */
				if (lane == 9) {
					phy_write16(reg_sp_132, cur_val_raw);
				} else {
					uintptr_t reg_restore = PHY_REG_APB_BASE | (((reg_base_var | (lane << 8))) << 1);
					phy_write16(reg_restore, cur_val_raw);
				}
			}
		}
	}
}

/**
 * pmu_cal_slice_deskew_result_commit() - Commit slice deskew results to PHY CSRs and converge delays
 * @channel: Memory channel index (0 or 1)
 * @buf: Pointer to deskew calibration results buffer in DMEM
 * @flags: Deskew commit control flags
 */
void pmu_cal_slice_deskew_result_commit(u32 channel, void *buf, u32 flags)
{
	struct phy_reg_stream_entry {
		u32 addr;
		u16 val;
	} __attribute__((packed));

	u32 csr_offset = dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET);
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
		phy_write16((PHY_REG_DBYTE_BCAST_BASE | 0x017c) + (i * 0x200), 1U << i);

	pmu_bist_lane_mask_set_all(0);
	pmu_phy_lane_timing_offset_set(0xff, 0x5555, 0x5555, 0x5555, 0x5555);
	pmu_phy_reg_stream_play((const u8 *)stream, sizeof(stream));
	phy_write16(PHY_REG_DBYTE_BCAST_GATE | (csr_offset << 1), 1);
	phy_write16(PHY_REG_DBYTE_BCAST_WIDTH | (csr_offset << 1), 2);
	pmu_dbyte_cal_strobe_seq(0xf, 0xf);

	u32 num_dly_regs = 8;
	u8 ch_mask = (u8)(5 << channel);

	if (flags != 0) {
		phy_write16(PHY_REG_BIST_STAT_FAIL, 0);
		phy_write16(PHY_REG_BIST_CMD, phy_read16(PHY_REG_BIST_PLL_CFG) | (1 << 4));
		num_dly_regs = 9;
	}

	phy_write16((PHY_REG_PUB_BASE | 0x3fe24) | (csr_offset << 1), 256);
	phy_write16((PHY_REG_PUB_BASE | 0x3fe26) | (csr_offset << 1), 256);
	phy_write16((PHY_REG_PUB_BASE | 0x3fe20) | (csr_offset << 1), 256);
	phy_write16((PHY_REG_PUB_BASE | 0x3fe22) | (csr_offset << 1), 256);

	pmu_cal_strobe_pulse();
	pmu_cal_strobe_secondary_pulse();

	pmu_channel_timing_deskew_reset(0x55, 0x55, 0, 0, ch_mask);
	pmu_cbt_strobe_pulse_train(ch_mask, 0);
	pmu_clk_timing_delay_latch(0x60, 1);

	phy_write16(PHY_REG_DBYTE_BCAST_DQ_DLY2, 511);
	phy_write16(PHY_REG_DBYTE_BCAST_DQ_DLY3, 511);
	phy_write16(PHY_REG_DBYTE_BCAST_DQ_DLY0, phy_read16((PHY_REG_DBYTE_BASE | 0x0160)) | 0x30);
	phy_write16(PHY_REG_DBYTE_BCAST_DQ_DLY1, 1);

	u8 start_slice = dmem_read8(PMU_DMEM_SLICE_START);
	u8 end_slice = dmem_read8(PMU_DMEM_SLICE_END);

	for (u32 slice = start_slice; slice <= end_slice; slice++) {
		for (u32 dly_idx = 0; dly_idx < num_dly_regs; dly_idx++) {
			u32 reg = (PHY_REG_DBYTE_BASE | PHY_REG_DBYTE_LCDL_DLY) | (slice << 13) | (dly_idx << 9);
			u16 val = phy_read16(reg);
			u32 adjusted = val + PMU_LCDL_STEP_COARSE;

			if (adjusted & PMU_LCDL_STEP_OVERFLOW_MASK)
				adjusted = (val - PMU_LCDL_STEP_COARSE) + PMU_LCDL_STEP_WRAP_OFFSET;

			phy_write16(reg, (u16)adjusted);
		}
	}

	u32 fp_flag = 0;
	u8 deskew_offset = 0;
	u32 cal_stat = pmu_cal_status_query(flags, 0);

	if (cal_stat != 0 && dmem_read8(PMU_DMEM_DESKEW_OFFSET_OVERRIDE) != 0) {
		pmu_dbyte_dq_deskew_regs_restore();
		deskew_offset = dmem_read8(PMU_DMEM_DESKEW_OFFSET_OVERRIDE);
		fp_flag = 1;
	}

	pmu_channel_timing_deskew_reset(0x53, 0xac, deskew_offset, deskew_offset, ch_mask);
	pmu_cbt_strobe_pulse_train(ch_mask, 1);
	pmu_bist_lane_mask_set_all(0xffff);
	pmu_phy_lane_timing_offset_set(0xf, 0xac53, 0xac53, 0xac53, 0xac53);

	if (cal_stat != 0 && deskew_offset != 0) {
		for (u32 bit = 0; bit < 8; bit++) {
			u16 pattern = (deskew_offset & (1 << bit)) ? 0x53ac : 0xac53;
			pmu_phy_lane_timing_offset_set(bit, pattern, pattern, pattern, pattern);
		}
		pmu_phy_lane_timing_offset_set(8, 0xac53, 0xac53, 0xac53, 0xac53);
	}

	phy_write16(PHY_REG_BIST_STAT_FAIL, 0);
	phy_write16(PHY_REG_BIST_STAT_WORD, 0);

	for (u32 iter = 0; iter < 32; iter++) {
		pmu_clk_timing_delay_latch(8, 1);

		for (u32 slice = start_slice; slice <= end_slice; slice++) {
			for (u32 dly_idx = 0; dly_idx < num_dly_regs; dly_idx++) {
				u16 stat = phy_read16((PHY_REG_DBYTE_BASE | 0x016e) | (slice << 13) | (dly_idx << 9));
				u32 target_lane = dly_idx;

				if (fp_flag != 0 && dly_idx != 8)
					target_lane = dmem_read8(PMU_DMEM_SAVED_DQ_DESKEW_TABLE + (slice * 8) + dly_idx);

				u32 dly_reg = (PHY_REG_DBYTE_BASE | PHY_REG_DBYTE_LCDL_DLY) | (slice << 13) | (target_lane << 9);
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

	phy_write16(PHY_REG_DBYTE_BCAST_RX_EN, 0);
	phy_write16(PHY_REG_DBYTE_BCAST_DQ_DLY1, 0);
	pmu_cal_strobe_secondary_pulse();
}

/**
 * pmu_cal_dbyte_rx_fifo_reset_poll() - Poll DBYTE RX FIFO reset and execute calibration strobe scan
 * @mode_sel: Calibration operating mode and channel selector
 * @enable_scan: Non-zero to enable strobe scan sweep and metric logging
 * @unused_buf: Auxiliary buffer pointer
 *
 * Return: RX FIFO offset delta, or 0 if sampling skipped.
 */
s16 pmu_cal_dbyte_rx_fifo_reset_poll(u32 mode_sel, u32 enable_scan, void *unused_buf)
{
	u8 buf[256];
	s16 ret_val = 0;

	/* Step 1: Query initial DQ status and clear calibration status flag */
	pmu_cal_dbyte_dq_status_check(mode_sel, enable_scan);
	dmem_write8(PMU_DMEM_AC_STEP_FLAG, 0);

	/*
	 * Step 2: Calibration and Strobe Scan Sweep (Block 2).
	 * Executed when enable_scan is non-zero and bit 1 of PMU_DMEM_CAL_MODE_1B is set.
	 */
	if (enable_scan != 0 && (dmem_read8(PMU_DMEM_CAL_MODE_1B) & (1 << 1))) {
		memset(buf, 0, 256);

		u32 mode_flag = (mode_sel != 1) ? 1 : 0;
		u32 mode_val = mode_flag + 8;
		u8 rank = dmem_read8(PMU_DMEM_CAL_RANK);

		pmu_cal_metric_log(5, 0x1120003, rank, mode_sel - 1, mode_val);

		u32 slice_offset = dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET);
		u32 win_params[2];
		win_params[0] = 0;
		win_params[1] = 0;

		pmu_deskew_and_tracker_reset();

		pmu_cal_sequence_pulse_send(0x41 << 19, 7, 8, 0, 0, 0, 0);
		pmu_cbt_coarse_step_pulse_seq(0, 4, 1);
		u32 win_val = pmu_cal_window_params_dispatch((const u8 *)win_params, (u16)mode_sel, 3);
		pmu_cbt_coarse_step_pulse_seq(0, 4, 1);
		pmu_cal_sequence_pulse_send(1 << 18, 7, 4, 0, 0, 0, 0);
		pmu_cal_sequence_pulse_send((1 << 19) | 0x80, 7, 8, 0, 0, 0, 0);

		u32 fp = (rank << 12) | slice_offset;
		u32 reg_tx_dqs = (PHY_REG_AC_BASE | 0x0004) | (fp << 1);
		u16 stride = pmu_cal_stride_get();
		phy_write16(reg_tx_dqs, 0x40);

		u16 sample_val = (u16)(win_val - 8);
		pmu_cal_strobe_pulse();

		u16 stride_x4 = (u16)(stride << 2);
		if (pmu_dbyte_deskew_sample_check(mode_sel, sample_val, stride_x4) != 0)
			pmu_assert_or_halt(0, 0x8b << 17);

		/* Sweep gate delay taps */
		u32 reg_gate = (PHY_REG_AC_BASE | 0x0002) | (((mode_val << 8) | fp) << 1);
		u8 step = dmem_read8(PMU_DMEM_CAL_STRIDE);

		for (s16 i = 0; i < 256; i += (s16)step) {
			phy_write16(reg_gate, (u16)i);
			pmu_cal_strobe_pulse();
			buf[i] = pmu_dbyte_deskew_sample_check(mode_sel, sample_val, stride_x4);
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

		s16 fifo_diff_signed = (s16)(scan_idx - 64);
		if (fifo_diff_signed >= 256) {
			pmu_assert_or_halt(0, 0x47 << 18);
		} else {
			pmu_cal_metric_log(4, 0x11d0001, (u32)(u16)fifo_diff_signed);
		}

		/* Clamp PMU_DMEM_DIFF_TAP_PAIR_105 calibration window offset */
		u8 val_105 = dmem_read8(PMU_DMEM_DIFF_TAP_PAIR_105);
		if ((u32)(val_105 - 1) >= 4) {
			val_105 = 4;
			dmem_write8(PMU_DMEM_DIFF_TAP_PAIR_105, 4);
		}

		u32 offset_105 = (u32)val_105 << 5;
		pmu_cal_metric_log(4, 0x11e0002, offset_105);

		ret_val = (s16)(fifo_diff_signed - (s16)offset_105);
		pmu_cal_metric_log(5, 0x1230003, rank, mode_sel, (u32)(u16)ret_val);
	}

	/*
	 * Step 3: Multi-Channel Phase Sampling Loop (Block 3).
	 */
	u32 is_mode2 = ((mode_sel & 3) == 2) ? 1 : 0;
	pmu_cbt_timing_pulse_coordinator(mode_sel, enable_scan);

	u32 csr_offset = dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET);
	u8 rank = dmem_read8(PMU_DMEM_CAL_RANK);
	u8 step = dmem_read8(PMU_DMEM_CAL_STRIDE);
	u8 *out_matrix = (u8 *)unused_buf;

	u32 base_reg = (rank << 12) | csr_offset;
	for (u16 delay = 0; delay < 128; delay += step) {
		for (u32 ch = 0; ch < 7; ch++)
			phy_write16((PHY_REG_AC_BASE | 0x0002) | ((base_reg | (ch << 8)) << 1), delay);
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
		phy_write16(PHY_REG_AC_STAT_LOW | (((csr_offset | (ch << 8)) << 1)), 0);
	pmu_cal_strobe_pulse();

	dmem_write8(PMU_DMEM_AC_STEP_FLAG, 1);
	pmu_cal_multi_rank_deskew_sweep(mode_sel);

	return ret_val;
}

/*
 * pmu_cal_wck_ck_align_pulse_send:
 */

/**
 * pmu_cal_wck_ck_align_pulse_send() - Send WCK-to-CK synchronization alignment pulse
 * @ch: Memory channel index (0 or 1)
 * @rank: DRAM rank index (0 or 1)
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

	memset(buf1, 0, sizeof(buf1) + sizeof(buf2) + sizeof(buf3) + sizeof(buf4));

	u32 csr_offset = dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET);
	u32 reg_0e = PHY_REG_MASTER_CFG | (csr_offset << 1);

	phy_write16(PHY_REG_VREF_TRIM_CFG | (csr_offset << 1), 1);
	phy_write16(PHY_REG_VREF_CTRL | (csr_offset << 1), 1);

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

	u32 dq_swizzle_mask = pmu_slice_dq_bitmask_swizzle(slice_idx_first, 4, ch);
	pmu_cal_metric_log(5, 0x13c0002, (u8)dq_swizzle_mask, dq_swizzle_mask);

	u32 flag_14 = 0;
	if ((u8)dq_swizzle_mask == 1) {
		pmu_cal_window_valid_check(ch_mask, 6);
		if ((u8)pmu_slice_dq_bitmask_swizzle(dq_swizzle_mask, 4, ch) == 7)
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

		u32 fp_val = pmu_slice_dq_bitmask_swizzle(dq_swizzle_mask, 4, ch);
		if (pmu_dmem_active_lane_query(rank, ch) != 0)
			fp_val |= (u8)pmu_slice_dq_bitmask_swizzle(slice_idx_other, 4, ch);

		for (u32 s = slice_start; s <= slice_end; s++) {
			u32 shift = (s != dq_swizzle_mask) ? 2 : 0;
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
	pmu_cal_metric_log(4, 0x15b0002, ch, (s32)res_34[dq_swizzle_mask]);

	s16 val_other = res_34[slice_idx_other];
	if (val_other < 1)
		res_34[slice_idx_other] = -val_other;
	else
		res_34[slice_idx_other] = val_other | 8;

	s16 val_0c = res_34[dq_swizzle_mask];
	if (val_0c < 1)
		res_34[dq_swizzle_mask] = -val_0c;
	else
		res_34[dq_swizzle_mask] = val_0c | 8;

	u8 val_comb = (u8)(((res_34[slice_idx_other] & 0x0f) << 4) | (res_34[dq_swizzle_mask] & 0x0f));
	pmu_cal_metric_log(5, 0x15c0003, ch, rank, val_comb);
	pmu_rank_slice_deskew_latch(val_comb, val_18);

	dmem_write8(0x0b7e + (ch << 1) + rank, val_comb);
	dmem_write8(0xd2 + ch * 5 + rank * 10, val_comb);

	phy_write16(reg_0e, orig_0e);
}

/**
 * pmu_cal_rank_rx_en_delay_commit() - Commit calibrated RX enable delays for channel and rank
 * @channel: Memory channel index (0 or 1)
 * @fp_flags: Calibration floating-point and status flags
 * @cal_stage_mode: Calibration stage mode selector
 * @stage: Calibration stage identifier
 */
void pmu_cal_rank_rx_en_delay_commit(u32 channel, u16 fp_flags, u32 cal_stage_mode, u32 stage)
{
	u8 deskew_work_buf[584];
	u8 scratch_buf[64];
	u16 min_diff_table[13][4];
	u16 rx_en_delays_a[13][4];
	u16 rx_en_delays_b[13][4];
	u16 slice_matrix_a[4][10];
	u16 slice_matrix_b[4][10];
	u16 slice_matrix_c[4][10];
	u16 slice_matrix_d[4][10];
	u16 slice_matrix_e[4][10];
	u16 slice_matrix_f[4][10];

	u32 lut_val = pmu_2b_identity_lut(channel);

	memset(rx_en_delays_b, 0, sizeof(rx_en_delays_b));
	memset(rx_en_delays_a, 0, sizeof(rx_en_delays_a));
	memset(min_diff_table, 0, sizeof(min_diff_table));

	pmu_cal_metric_log(4, 0x1680004, channel, fp_flags, cal_stage_mode, stage);

	pmu_cal_channel_rank_boundary_eval(deskew_work_buf, (u8)channel, cal_stage_mode, stage);

	u32 csr_offset = dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET);
	phy_write16(PHY_REG_DBYTE_BCAST_SAMPLE | (csr_offset << 1), 1);

	u16 off0 = *(u16 *)&deskew_work_buf[514];
	u16 off1 = *(u16 *)&deskew_work_buf[516];

	pmu_cal_metric_log(4, 0x1690002, off0, off1);

	u16 reg0 = phy_read16(PHY_REG_DBYTE_BASE | ((csr_offset | off0) << 1));
	u8 rx_en_val_lo = (u8)(reg0 >> 8);

	u16 reg1 = phy_read16(PHY_REG_DBYTE_BASE | ((csr_offset | off1) << 1));
	u8 rx_en_val_hi = (u8)(reg1 >> 8);

	pmu_cal_metric_log(4, 0x16a0002, rx_en_val_lo, rx_en_val_hi);

	u32 chan_mask = (1 << channel) | ((1 << channel) << 2);
	u32 is_lt_3200 = 1;
	if (stage == 0x11)
		is_lt_3200 = pmu_freq_lt_3200_check();

	/* Outer loop: sweep 13 delay steps (idx = 0..12) */
	for (u32 idx = 0; idx <= 12; idx++) {
		memset(slice_matrix_f, 0, sizeof(slice_matrix_f));
		memset(slice_matrix_e, 0, sizeof(slice_matrix_e));
		memset(slice_matrix_d, 0, sizeof(slice_matrix_d));

		u8 comp_idx = 12 - idx;
		u16 val_r12 = ((u16)rx_en_val_lo << 8) | ((u16)comp_idx << 4) | (u8)idx;
		u16 val_r13 = ((u16)rx_en_val_hi << 8) | ((u16)idx << 4) | (u8)comp_idx;

		u8 slice_start = dmem_read8(PMU_DMEM_SLICE_START);
		u8 slice_end   = dmem_read8(PMU_DMEM_SLICE_END);

		for (u32 s = slice_start; s <= slice_end; s++) {
			u32 base = (s << 12) | csr_offset;
			phy_write16(PHY_REG_DBYTE_BASE | (((lut_val + off0) | base) << 1), val_r12);
			phy_write16(PHY_REG_DBYTE_BASE | (((lut_val + off1) | base) << 1), val_r13);
		}

		pmu_cal_strobe_pulse();

		for (u32 i = 0; i < 40; i++)
			dmem_write8(0x6594 + i * 848, 0);

		pmu_cal_bist_cmd_strobe_dispatch((u32)(uintptr_t)scratch_buf, deskew_work_buf[523], deskew_work_buf[518],
			(u8)chan_mask, (u32)(uintptr_t)deskew_work_buf, cal_stage_mode, deskew_work_buf[525]);

		pmu_cal_slice_step_eval_sweep((void *)(PMU_DMEM_BASE | PMU_DMEM_SWEEP_SAMPLE_BUF), deskew_work_buf[523], deskew_work_buf[518], 4,
			(u8)chan_mask, (u8)fp_flags, is_lt_3200,
			(const u16 *)deskew_work_buf, cal_stage_mode, deskew_work_buf[525], scratch_buf);

		if (deskew_work_buf[524] != 0) {
			pmu_phy_timing_delay_latch(1, (u8)chan_mask);
			pmu_cal_slice_step_eval_sweep((void *)(PMU_DMEM_BASE | PMU_DMEM_SWEEP_SAMPLE_BUF), deskew_work_buf[523], 3, 4,
				(u8)chan_mask, (u8)fp_flags, 1, (const u16 *)deskew_work_buf, 0, 0, scratch_buf);
			pmu_phy_timing_delay_latch(0, (u8)chan_mask);
		}

		pmu_dmem_reg_stream_unpack(scratch_buf);
		pmu_eye_centroid_avg_calc((const u16 *)(deskew_work_buf + 16), (u16 *)slice_matrix_f, (u16 *)slice_matrix_e, (u16 *)slice_matrix_d);

		u16 ctrl_8 = *(u16 *)&deskew_work_buf[520];
		if (ctrl_8 != 255) {
			for (u32 i = 0; i < 40; i++)
				dmem_write8(0x6594 + i * 848, 0);

			pmu_cal_bist_cmd_strobe_dispatch((u32)(uintptr_t)scratch_buf, deskew_work_buf[523], (u8)ctrl_8,
				(u8)chan_mask, (u32)(uintptr_t)(deskew_work_buf + 8), cal_stage_mode, deskew_work_buf[525]);

			pmu_cal_slice_step_eval_sweep((void *)(PMU_DMEM_BASE | PMU_DMEM_SWEEP_SAMPLE_BUF), deskew_work_buf[523], (u8)ctrl_8, 4,
				(u8)chan_mask, (u8)fp_flags, is_lt_3200,
				(const u16 *)(deskew_work_buf + 8), cal_stage_mode, deskew_work_buf[525], scratch_buf);

			pmu_dmem_reg_stream_unpack(scratch_buf);
			pmu_eye_centroid_avg_calc((const u16 *)(deskew_work_buf + 24), (u16 *)slice_matrix_c, (u16 *)slice_matrix_b, (u16 *)slice_matrix_a);
			ctrl_8 = *(u16 *)&deskew_work_buf[520];
		}

		u8 num_samples = deskew_work_buf[522];
		for (u32 s = slice_start; s <= slice_end; s++) {
			s32 min_val = 127;
			if (num_samples != 0) {
				s16 sum = 0;
				for (u32 k = 0; k < num_samples; k++) {
					s8 diff = (s8)(slice_matrix_d[s][k] - slice_matrix_e[s][k]);
					if (diff < min_val)
						min_val = diff;
					sum += diff;
				}
				rx_en_delays_b[idx][s] = sum;

				if (ctrl_8 == 0) {
					s32 min_val2 = 127;
					s16 sum2 = 0;
					for (u32 k = 0; k < num_samples; k++) {
						s8 diff = (s8)(slice_matrix_a[s][k] - slice_matrix_b[s][k]);
						if (diff < min_val2)
							min_val2 = diff;
						sum2 += diff;
					}
					rx_en_delays_a[idx][s] = sum2;
					if (min_val2 < min_val) {
						min_val = min_val2;
						rx_en_delays_b[idx][s] = sum2;
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

			if (diff > best_diff || (diff == best_diff && (s16)rx_en_delays_b[step][s] > best_sum)) {
				best_step = (u8)step;
				best_diff = diff;
				best_sum  = (s16)rx_en_delays_b[step][s];
			}
		}

		pmu_cal_metric_log(4, 0x1720003, s, (s8)best_step, best_sum);

		u16 param_428 = dmem_read16(PMU_DMEM_ITER_COUNT);
		dmem_write16(0xe798 + (s * 8) + (param_428 * 2), (u16)(s8)best_step);
		pmu_cal_metric_log(4, 0x1730003, s, (s8)best_step, param_428);

		u8 comp_step = 12 - (s8)best_step;
		u16 val_off0 = ((u16)rx_en_val_lo << 8) | ((u16)comp_step << 4) | (u8)best_step;
		u32 addr0 = PHY_REG_DBYTE_BASE | (((lut_val + off0) | (s << 12) | csr_offset) << 1);
		phy_write16(addr0, val_off0);

		u16 val_off1 = ((u16)rx_en_val_hi << 8) | ((u16)(u8)best_step << 4) | comp_step;
		u32 addr1 = PHY_REG_DBYTE_BASE | (((lut_val + off1) | (s << 12) | csr_offset) << 1);
		phy_write16(addr1, val_off1);
	}

	pmu_cal_dbyte_deskew_results_apply(deskew_work_buf, (u8)channel, stage);
}

/**
 * pmu_cal_lane_window_sweep_coordinator() - Coordinate per-lane window margin sweeps across channels
 * @ch_mask: Active memory channel bitmask
 * @wide_range: Non-zero to select extended 511-step range and 9 pins; 0 for 255-step and 8 pins
 */
void pmu_cal_lane_window_sweep_coordinator(u32 ch_mask, u32 wide_range)
{
	u16 margin_table[4][2][2][2];
	u16 best_taps[2][2][2];
	u32 fp_range = (wide_range != 0) ? 511 : 255;
	u32 pin_count = (wide_range != 0) ? 9 : 8;

	memset(margin_table, 0, sizeof(margin_table));
	memset(best_taps, 0, sizeof(best_taps));

	u8 cal_p40 = dmem_read8(PMU_DMEM_CH1_RANK_EN);
	u8 cal_p25 = dmem_read8(PMU_DMEM_CH0_RANK_EN);
	u8 active_mask = dmem_read8(PMU_DMEM_CAL_RANK) ? cal_p40 : cal_p25;

	/* Phase 1: Outer Calibration Configuration Sweep (4 iterations: cfg_idx 0..3) */
	for (u32 cfg_idx = 0; cfg_idx < 4; cfg_idx++) {
		u8 cfg_val = (u8)(cfg_idx | (cfg_idx << 4));

		for (u32 ch = 0; ch < 2; ch++) {
			u8 mask = (u8)((1 << (ch + 2)) | (1 << ch));
			if (active_mask & mask)
				pmu_slice_deskew_state_latch(cfg_val, mask);
		}

		u32 vref = (dmem_read8(PMU_DMEM_CLK_GATE_FLAG) & 4) ? 0x3f : dmem_read8(PMU_DMEM_DQ_SWAP_BASE);
		pmu_cal_margin_matrix_scan_eval(ch_mask, 0, fp_range, 2, vref, 0);

		for (u32 ch = 0; ch < 2; ch++) {
			u8 mask = (u8)((1 << (ch + 2)) | (1 << ch));
			if (!(active_mask & mask))
				continue;

			u8 start_slice = dmem_read8(PMU_DMEM_RANK0_SLICE_START);
			u8 end_slice = dmem_read8(PMU_DMEM_RANK1_SLICE_END);
			uintptr_t slice_buf = (PMU_DMEM_BASE | PMU_DMEM_EYE_SAMPLE_BUF) + (ch * 5280) + (start_slice * 1320);

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
					u8 r_start = (rank != 0) ? dmem_read8(PMU_DMEM_RANK1_SLICE_START) :
											   dmem_read8(PMU_DMEM_RANK0_SLICE_START);
					u8 r_end   = (rank != 0) ? dmem_read8(PMU_DMEM_RANK1_SLICE_END) :
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

			dmem_write8((PMU_DMEM_BASE | PMU_DMEM_RANK_CH_TAP_TABLE) + (rank * 2) + ch, combined);

			if (rank == 0) {
				if (ch == 0)
					dmem_write8((PMU_DMEM_BASE | PMU_DMEM_BEST_TAP_C0R0), combined);
				else
					dmem_write8((PMU_DMEM_BASE | PMU_DMEM_BEST_TAP_C1R0), combined);
			} else {
				if (ch == 0)
					dmem_write8((PMU_DMEM_BASE | PMU_DMEM_BEST_TAP_C0R1), combined);
				else
					dmem_write8((PMU_DMEM_BASE | PMU_DMEM_BEST_TAP_C1R1), combined);
			}
		}
	}

	phy_write16(PHY_REG_ANALOG_CFG0, dmem_read8(PMU_DMEM_CH0_RANK_EN));
	phy_write16(PHY_REG_ANALOG_CFG1, dmem_read8(PMU_DMEM_CH1_RANK_EN));
}

/**
 * set_slice_step() - Program delay step into broadcast and active slice registers
 * @slice_start: Starting slice index (0..3)
 * @slice_end: Ending slice index (0..3)
 * @slice_mask: Bitmask of active DBYTE slices
 * @csr_offset: Active CSR offset word
 * @step: Delay line tap step adjustment value
 */
void set_slice_step(u8 slice_start, u8 slice_end, u8 slice_mask, u32 csr_offset, u16 step)
{
	u32 base = PHY_REG_DBYTE_BASE | (csr_offset << 1);
	phy_write16(PHY_REG_DBYTE_BCAST_STEP | (csr_offset << 1), step);
	for (u32 slice = slice_start; slice <= slice_end; slice++) {
		if (slice_mask & (1 << slice))
			phy_write16(base | (slice << 13), step);
	}
}

/**
 * pmu_cal_dbyte_deskew_fine_tune() - Fine-tune DBYTE deskew delay line taps across active slices
 * @mode: Operating profile mode determining fine-tune parameters
 */
void pmu_cal_dbyte_deskew_fine_tune(u32 mode)
{
	pmu_cal_metric_log(4, 0x510000);

	u32 counter = 0;

	for (u32 ch = 0; ch < 2; ch++) {
		u8 active = (ch == 0) ? dmem_read8(PMU_DMEM_CH0_RANK_EN) : dmem_read8(PMU_DMEM_CH1_RANK_EN);
		if (!active)
			continue;

		pmu_phy_mode_cfg_dispatch(ch);

		u16 reg_pair[2];
		reg_pair[0] = dmem_read16(0xe8 + ch * 2);
		reg_pair[1] = dmem_read16(0xec + ch * 2);

		if (mode != 0) {
			u8 num_entries = dmem_read8(PMU_DMEM_METRIC_TABLE_ENTRIES);
			for (u32 rank = 0; rank < 2; rank++) {
				for (u32 entry = 0; entry < num_entries; entry++) {
					u8 val15 = dmem_read8(0xb6e + ch * 2 + rank + entry * 4);
					pmu_cal_metric_log(4, 0x520002, dmem_read8(0x46c + entry), val15);

					u32 csr_addr = PHY_REG_APB_BASE | ((((reg_pair[0] << 2) | 0x41000) + (counter & 0xff)) << 1);
					phy_write16(csr_addr, val15);
					counter++;
				}
			}

			if (counter & 1) {
				u32 csr_addr = PHY_REG_APB_BASE | ((((reg_pair[0] << 2) | 0x41000) + (counter & 0xff)) << 1);
				phy_write16(csr_addr, 0);
				counter++;
			}
		} else {
			u32 ch_shift = ch * 2;
			u8 num_entries = dmem_read8(PMU_DMEM_METRIC_TABLE_ENTRIES);

			for (u32 rank = 0; rank < 2; rank++) {
				pmu_deskew_and_tracker_reset();
				pmu_cal_markers_set((u16)(reg_pair[rank] << 2));

				u8 sp18 = 0;
				for (int entry_idx = 0; entry_idx < num_entries; entry_idx++) {
					for (u32 lane = 0; lane < 2; lane++) {
						u8 cal_cmd = dmem_read8(PMU_DMEM_CAL_CMD_TABLE + entry_idx);
						u8 delay_val = dmem_read8(PMU_DMEM_DELAY_CAL_RESULTS + (rank ? 0x14 : 0) + (ch_shift + entry_idx * 4) + lane);
						u32 pulse_mode = (lane != 0 && entry_idx != num_entries - 1) ? 0x22 : 0;
						u32 lane_bitmask = 1 << lane;

						if (sp18 != 0 && cal_cmd == 0xc) {
							if (!((dmem_read8(PMU_DMEM_ACTIVE_LANES) >> ch_shift) & lane_bitmask))
								continue;
							delay_val |= 0x80;
						}

						pmu_cal_sequence_pulse_send(0, 6, pulse_mode, delay_val, cal_cmd, (lane_bitmask << 2) | lane_bitmask, 0);

						if (lane == 1 && sp18 == 0 && cal_cmd == 0xc) {
							if (dmem_read8(PMU_DMEM_ACTIVE_LANES) != 0) {
								entry_idx--;
								sp18 = 1;
							} else {
								sp18 = 0;
							}
						}

						if (cal_cmd == 0xc && dmem_read8(PMU_DMEM_ACTIVE_LANES) == 0)
							pmu_cbt_coarse_step_pulse_seq(0, 4, 0);

						pmu_cal_metric_log(4, 0x530005, rank, lane, ch, cal_cmd, (s8)delay_val);
					}
				}

				pmu_cal_sequence_pulse_send(0, 7, 2, 0, 0, 0, 0x28);
			}
		}
	}

	phy_write16(PHY_REG_INTERRUPT_CLEAR, 0);
	phy_write16(PHY_REG_INTERRUPT_MASK, 0);
}

/**
 * pmu_cal_phase_detector_edge_align() - Align phase detector edges across active slices
 */
void pmu_cal_phase_detector_edge_align(void)
{
	dmem_write32(PMU_DMEM_CAL_PARAM_414, (PMU_DMEM_BASE | PMU_DMEM_EYE_SAMPLE_BUF));
	dmem_write32(PMU_DMEM_SHADOW_TRACE_PTR, (PMU_DMEM_BASE | PMU_DMEM_EYE_SAMPLE_BUF));
	pmu_cal_metric_log(4, 0x5a0001);

	u8 phase_tbl_a[8];
	u8 phase_tbl_b[22];
	u8 phase_tbl_c[28];
	u8 phase_tbl_d[12];
	u8 phase_tbl_e[14];

	memcpy(phase_tbl_a, (const void *)(PMU_DMEM_BASE | PMU_DMEM_PHASE_DETECT_TABLE_A), 8);
	u16 align_complete = 1;
	memcpy(phase_tbl_b, (const void *)(PMU_DMEM_BASE | PMU_DMEM_PHASE_DETECT_TABLE_B), 22);
	memcpy(phase_tbl_c, (const void *)(PMU_DMEM_BASE | PMU_DMEM_PHASE_DETECT_TABLE_C), 28);
	memcpy(phase_tbl_d, (const void *)(PMU_DMEM_BASE | PMU_DMEM_PHASE_DETECT_TABLE_D), 12);
	memcpy(phase_tbl_e, (const void *)(PMU_DMEM_BASE | PMU_DMEM_PHASE_DETECT_TABLE_E), 14);

	u32 shadow_ptr = dmem_read32(PMU_DMEM_SHADOW_TRACE_PTR) >> 1;
	dmem_write16(PMU_DMEM_CAL_STEP_108, (u16)shadow_ptr);
	u16 shadow_idx = dmem_read16(PMU_DMEM_CAL_STEP_METRIC_F2);

	u8 channel_detect_mask[4] = {0};

	for (u32 mode_idx = 0; mode_idx < 3; mode_idx++) {
		channel_detect_mask[0] = channel_detect_mask[1] = channel_detect_mask[2] = channel_detect_mask[3] = 0;

		u32 mode_param = 1;
		void *tbl_ptrs[4] = {0};
		u32 limit_50 = 1;

		if (mode_idx == 0) {
			mode_param = 10;
			tbl_ptrs[0] = phase_tbl_a;
			tbl_ptrs[1] = &align_complete;
			limit_50 = 2;
		} else if (mode_idx == 1) {
			mode_param = 9;
			tbl_ptrs[0] = phase_tbl_c;
			tbl_ptrs[2] = phase_tbl_b;
			tbl_ptrs[3] = phase_tbl_d;
			limit_50 = (u8)dmem_read32(PMU_DMEM_ACTIVE_SLICE_BITMAP);
		} else {
			mode_param = 1;
			tbl_ptrs[0] = phase_tbl_e;
			limit_50 = 1;
		}

		for (u32 slot = 0; slot < 4; slot++) {
			u8 count = channel_detect_mask[slot];
			u32 repeat_cnt = ((slot | 2) == 3) ? mode_param : 1;
			u32 max_val = (repeat_cnt > 1) ? repeat_cnt : 1;

			for (u32 i = 0; i < count; i++) {
				for (u32 j = 0; j < limit_50; j++) {
					u16 *p_reg = (u16 *)tbl_ptrs[slot];
					u32 dmem_val = *(volatile u32 *)((PMU_DMEM_BASE | PMU_DMEM_CAL_SEQ_MAP) + mode_idx * 4 + 0x18);
					u32 slice_apb_offset = j << 12;

					for (u32 k = max_val; k > 0; k--) {
						u16 reg_off = p_reg[i];
						u32 full_addr = (slice_apb_offset | dmem_val | reg_off) << 1;
						u16 v1, v2;
						u32 type;

						if (slot < 2) {
							v1 = phy_read16(PHY_REG_APB_BASE | full_addr);
							if (dmem_val & 1) {
								v2 = v1;
								v1 = 0;
								type = 2;
							} else {
								v2 = 0;
								type = 1;
							}
						} else {
							v1 = phy_read16(PHY_REG_APB_BASE | full_addr);
							v2 = phy_read16(PHY_REG_APB_BASE | (full_addr + 2));
							type = 3;
						}

						if (dmem_read16(PMU_DMEM_CAL_STEP_METRIC_F2) != 0 || dmem_read16(PMU_DMEM_RANK_BASE_VAL_E8) != 0 || dmem_read16(PMU_DMEM_RANK_BASE_VAL_EA) != 0) {
							pmu_phy_reg_write_shadow_track((u16)(shadow_idx << 2), v1, v2, (full_addr >> 1) & 0x7ffff, type);
							shadow_idx++;
						}
					}
				}
			}
		}
	}

	phy_write16(PHY_REG_SHADOW_COUNT, shadow_idx - dmem_read16(PMU_DMEM_CAL_STEP_METRIC_F2));

	u32 shadow_end = dmem_read32(PMU_DMEM_SHADOW_TRACE_PTR) >> 1;
	dmem_write16(PMU_DMEM_CAL_STEP_10C, (u16)shadow_end);
	u16 ac_profile_word = dmem_read16(PMU_DMEM_AC_PROFILE_BASE);
	dmem_write16(PMU_DMEM_CAL_STEP_10A, (u16)(shadow_end - dmem_read16(PMU_DMEM_CAL_STEP_108)));

	for (u32 ch = 0; ch < 2; ch++) {
		u8 active = (ch == 0) ? dmem_read8(PMU_DMEM_CH0_RANK_EN) : dmem_read8(PMU_DMEM_CH1_RANK_EN);
		if (!active)
			continue;

		u16 reg_e8 = dmem_read16(0xe8 + ch * 2);
		u32 lane_shift_mask = (u32)active << ch;
		u8 phase_detect_status = 0;
		u8 num_entries = dmem_read8(PMU_DMEM_METRIC_TABLE_ENTRIES);

		for (int entry_idx = 0; entry_idx < num_entries; entry_idx++) {
			for (u32 rank_idx = 0; rank_idx < 2; rank_idx++) {
				u8 cal_cmd = dmem_read8(PMU_DMEM_CAL_CMD_TABLE + entry_idx);
				u8 val_b6e = dmem_read8(PMU_DMEM_DELAY_CAL_RESULTS + lane_shift_mask + entry_idx * 4 + rank_idx);
				u8 cal_byte = (phase_detect_status != 0 && cal_cmd == 0xc) ? (val_b6e | 0x80) : val_b6e;

				u32 phase_reg_buf[4];
				phase_reg_buf[0] = 0;
				phase_reg_buf[1] = 0;
				phase_reg_buf[2] = 0;
				phase_reg_buf[3] = 0;
				pmu_cal_timing_packet_format((u16 *)phase_reg_buf, cal_cmd, cal_byte, 1 << rank_idx);

				if (phase_detect_status != 0 && cal_cmd == 0xc && !((dmem_read8(PMU_DMEM_ACTIVE_LANES) >> lane_shift_mask) & (1 << rank_idx))) {
					phase_reg_buf[0] = 0;
					phase_reg_buf[1] = 0;
					phase_reg_buf[2] = 0;
					phase_reg_buf[3] = 0;
				}

				pmu_cal_dual_rank_state_eval(phase_reg_buf, (u16 *)&ac_profile_word, (u16 *)channel_detect_mask);

				if (rank_idx != 0 && entry_idx != num_entries - 1) {
					phase_reg_buf[0] = 0;
					phase_reg_buf[1] = 0x1b000000;
					phase_reg_buf[2] = 0;
					phase_reg_buf[3] = 0;
					pmu_cal_dual_rank_state_eval(phase_reg_buf, (u16 *)&ac_profile_word, (u16 *)channel_detect_mask);
				}

				if (rank_idx == 1 && phase_detect_status == 0 && cal_cmd == 0xc) {
					entry_idx--;
					phase_detect_status = 1;
				}
			}
		}
	}

	dmem_write16(PMU_DMEM_CAL_STEP_10E, (dmem_read32(PMU_DMEM_SHADOW_TRACE_PTR) >> 1) - dmem_read16(PMU_DMEM_CAL_STEP_10C));
	pmu_cal_metric_log(4, 0x5f0000);
}

/**
 * pmu_cal_rank_deskew_matrix_commit() - Commit rank deskew calibration matrix to PHY registers
 * @channel: Memory channel index (0 or 1)
 * @buf: Pointer to deskew calibration matrix buffer in DMEM
 */
void pmu_cal_rank_deskew_matrix_commit(u32 channel, void *buf)
{
	struct phy_reg_stream_entry {
		u32 addr;
		u16 val;
	} __attribute__((packed));

	u32 csr_offset = dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET);
	u32 csr_offset_sh1 = csr_offset << 1;

	u32 ch_mask = (1U << (channel + 2)) | (1U << channel);
	u32 cbt_idx = pmu_cbt_active_entry_lookup();
	u32 cbt_val = pmu_cbt_delay_offset_lookup();

	u32 cbt_lut_offset;
	if (cbt_val > 186) {
		cbt_lut_offset = 0xfe;
	} else {
		cbt_lut_offset = (pmu_cbt_delay_offset_lookup() * 2) + 0x22;
	}

	u32 lut_val = pmu_2b_identity_lut(channel);
	u8 ch_cfg_04 = dmem_read8(PMU_DMEM_CHANNEL_CFG) & 0x0f;
	u8 rank = dmem_read8(PMU_DMEM_CAL_RANK);

	pmu_cal_metric_log(5, 0x2af0003, rank, channel, ch_cfg_04);

	struct phy_reg_stream_entry stream_save[15] = {
		{ 0x10097, 0 },
		{ 0x10093, 0 },
		{ 0x10005 | csr_offset, 0 },
		{ 0x1008b, 0 },
		{ 0x10095, 0 },
		{ 0x20074, 0 },
		{ 0x20057, 0 },
		{ 0x2002a, 0 },
		{ 0x2003f | csr_offset, 0 },
		{ 0x20041 | csr_offset, 0 },
		{ 0x20037 | csr_offset, 0 },
		{ 0x2003f | csr_offset, 0 },
		{ 0x2003d | csr_offset, 0 },
		{ 0x2003e | csr_offset, 0 },
		{ 0x70011, 0 },
	};

	pmu_phy_reg_buffer_stream((u8 *)stream_save, 15, 1);

	phy_write16(PHY_REG_BIST_PLL_CFG, 0);
	phy_write16((PHY_REG_DBYTE_BCAST_BASE | 0x012e), 0);
	phy_write16(PHY_REG_DBYTE_BCAST_DQ_DLY0, 8);
	phy_write16(PHY_REG_DBYTE_BCAST_DQ_DLY1, 0x40);
	phy_write16((PHY_REG_PUB_SEC_BASE | 0x0ae), 0xaa);

	pmu_deskew_and_tracker_reset();

	u8 ch_mask_b = (u8)ch_mask;
	pmu_cal_sequence_pulse_send(0x41 << 19, 7, 4, 0, 0, ch_mask_b, 0);
	pmu_cal_sequence_pulse_send(0, 7, 8, 0, 0, 0, 0);
	pmu_cal_sequence_pulse_send(0x80 | (1 << 19), 7, 4, 0, 0, ch_mask_b, 0);

	pmu_clk_timing_delay_latch(0, 1);
	pmu_delay_us(5000, 5);
	pmu_dbyte_cal_strobe_pulse_seq(1, 0xff, 0);

	u8 dram_type = dmem_read8(PMU_DMEM_DRAM_TYPE);
	u32 is_dram_type_lpddr5 = (dram_type == 4) ? 1 : 0;
	u32 dram_type_cfg_val = (dram_type << 6) + 6;
	u32 val_43a = (is_dram_type_lpddr5 << 2) + 4;
	if (dram_type_cfg_val > 255)
		dram_type_cfg_val = 255;

	dmem_write8(PMU_DMEM_2D_STEP_PARAM_43A, (u8)val_43a);
	dmem_write16(PMU_DMEM_2D_STEP_CTRL, (u16)dram_type_cfg_val);
	pmu_cal_metric_log(4, 0x28b0001);

	cbt_val += (cbt_idx & 1);
	u32 cbt_dly_aligned = cbt_lut_offset + (cbt_lut_offset & 1);

	phy_write16(PHY_REG_VREF_CTRL | csr_offset_sh1, 1);
	phy_write16(PHY_REG_VREF_TRIM_CFG, 1);

	u16 val_6e = phy_read16((PHY_REG_MASTER_BASE | 0x006e) | csr_offset_sh1);
	u16 val_7e = val_6e & 0x3f;
	if (dram_type == 2)
		val_7e |= (1 << 11);
	else
		val_7e |= (1 << 12);
	phy_write16((PHY_REG_MASTER_BASE | 0x007e) | csr_offset_sh1, val_7e);

	u32 lut_mode_bits = lut_val & 3;

	phy_write16((PHY_REG_DBYTE_BCAST_BASE | 0x0126), 512);
	phy_write16(PHY_REG_DBYTE_BCAST_GATE | csr_offset_sh1, 1);
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

	u32 cbt_channel_mask = (ch_mask << 5) & 64;
	pmu_cal_sequence_pulse_send(cbt_channel_mask | (1 << 25), 7, 2, 0, 0, 0, 0);

	u8 cbt_idx_b = (u8)cbt_idx;
	pmu_cal_sequence_pulse_send(cbt_channel_mask, 7, cbt_idx_b, 0, 0, 0, 0);
	pmu_cal_sequence_pulse_send(4 | (1 << 18), 1, (u8)cbt_val, 1 << 12, 0, 0, 0);
	pmu_cal_sequence_pulse_send(cbt_channel_mask | (1 << 15), 7, 4, 0, 0, 0, 1);

	pmu_cbt_cal_stat_clear();
	pmu_cal_sequence_pulse_send(cbt_channel_mask, 7, 0x10, 0, 0, 0, 0);
	pmu_cal_sequence_pulse_send(cbt_channel_mask | (1 << 7), 7, 4, 0, 0, 0, 0);

	pmu_cal_metric_log(4, 0x2b00000);
	phy_write16(PHY_REG_DBYTE_BCAST_RX_EN, 0x6300);
	pmu_clk_timing_delay_latch(8, 1);
	phy_write16(PHY_REG_DBYTE_BCAST_RX_EN, 768);
	pmu_cal_metric_log(4, 0x2b10000);

	u16 timing_438 = dmem_read16(PMU_DMEM_2D_STEP_CTRL);
	if (dmem_read8(PMU_DMEM_CLK_GATE_FLAG) & (1 << 2)) {
		pmu_clk_timing_delay_latch(timing_438, 1);
	} else {
		for (int i = 0; i < 10; i++)
			pmu_clk_timing_delay_latch(timing_438, 1);
	}

	pmu_dbyte_slice_cal_param_pulse(channel, 0);
	pmu_phy_lcdl_delay_read((u16 *)buf, 0, 2);

	u32 reg_target_base = csr_offset | (lut_mode_bits + 0x2a);
	pmu_slice_delay_step_program(reg_target_base, (const u16 *)buf, 0, 0, 0, 1);

	phy_write16(PHY_REG_DBYTE_BCAST_RX_EN, 0);

	u32 fp_cbt = pmu_cbt_active_entry_lookup();
	pmu_cal_metric_log(5, 0x29a0002, dmem_read8(PMU_DMEM_CAL_RANK), channel);
	pmu_slice_reg_query_mailbox_send(channel);

	u8 metric_flag_106 = dmem_read8(PMU_DMEM_CAL_STEP_106);
	pmu_cal_metric_log(4, 0x29b0001, metric_flag_106);

	if (metric_flag_106 & 1) {
		pmu_cal_metric_log(10, 0x29c0000);
		goto cleanup;
	}

	pmu_cal_metric_log(4, 0x29d0002, dmem_read8(PMU_DMEM_CAL_RANK), lut_mode_bits);
	pmu_slice_phy_reg_step_adjust(lut_mode_bits, 1, 0);

	fp_cbt += (fp_cbt & 1);
	pmu_ac_lane_profile_setup();

	if (dmem_read8(PMU_DMEM_FREQ_MODE) == 0) {
		pmu_deskew_and_tracker_reset();
		pmu_cal_sequence_pulse_send(0, 7, 4, 0, 0, 0, 0);
		pmu_clk_timing_delay_latch(0, 1);
	}

	pmu_deskew_and_tracker_reset();
	u32 cbt_dly_offset = fp_cbt + 0x1e;
	pmu_cbt_coarse_step_pulse();
	pmu_cbt_3phase_pulse_seq();
	pmu_clk_timing_delay_latch(0, 1);

	u16 saved_88 = phy_read16(PHY_REG_DRAM_TYPE_CFG);
	u16 target_cfg_val = (dram_type == 2) ? 513 : 1026;
	phy_write16(PHY_REG_DRAM_TYPE_CFG, (dram_type == 2) ? 3 : 2);
	phy_write16((PHY_REG_MASTER_BASE | 0x007a) | csr_offset_sh1, target_cfg_val);
	phy_write16((PHY_REG_MASTER_BASE | 0x007c) | csr_offset_sh1, target_cfg_val);

	u16 saved_84 = phy_read16(PHY_REG_VREF_STAT);
	phy_write16(PHY_REG_VREF_STAT, saved_84 | (1 << 7));
	pmu_slice_phy_reg_step_adjust(lut_mode_bits, 1, 1);

	u32 slice_mask = (pmu_dmem_training_flags_eval() == 0) ? 0xff : 511;
	phy_write16(PHY_REG_DBYTE_BCAST_DQS_DLY0, 0xffff);
	phy_write16(PHY_REG_DBYTE_BCAST_DQS_DLY1, 0xffff);
	pmu_phy_slice_mask_set(slice_mask);

	phy_write16(PHY_REG_BIST_CTRL_MODE, 1);
	phy_write16((PHY_REG_PUB_BASE | 0x3ff7c), 0);

	pmu_dbyte_cal_strobe_seq(0xf, 0xf);
	pmu_phy_reset_pulse();

	pmu_cal_slice_step_scan_eval(cbt_dly_offset & 0xff, ch_mask_b);
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
			u16 stat = phy_read16((PHY_REG_DBYTE_BASE | 0x016e) | slice_sh13);
			slice_val[slice] = stat;

			u32 slice_reg_base = ((csr_offset | (slice << 12) | (lut_mode_bits + 0x2a)) << 1);
			u32 reg_addr = PHY_REG_DBYTE_BASE | slice_reg_base;
			u16 phy_status_reg = phy_read16(reg_addr);

			u16 prev_stat = prev_val[slice];
			if (prev_stat != 0 && stat == 0) {
				slice_done[slice] = 1;
				pmu_cal_metric_log(4, 0x2a10002, slice, phy_status_reg);
				continue;
			}

			if (prev_stat == 0 && iter == 11 && stat == 0) {
				pmu_cal_metric_log(4, 0x2a20000);
				flag_sp67 = 1;
			}

			u32 fine_delay = phy_status_reg & 0x3f;
			u32 coarse_delay = phy_status_reg >> 6;

			pmu_cal_metric_log(4, 0x2a30003, slice, coarse_delay, fine_delay);

			coarse_delay += 2;
			u32 new_val = (coarse_delay << 6) | fine_delay;

			u32 target_reg = (PHY_REG_DBYTE_BASE | 0x1e00) | slice_reg_base;

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

					u16 new_stat = phy_read16((PHY_REG_DBYTE_BASE | 0x016e) | slice_sh13);
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

	phy_write16(PHY_REG_DRAM_TYPE_CFG, saved_88);
	phy_write16(PHY_REG_VREF_STAT, saved_84);
	pmu_phy_profile_param_program();
	pmu_slice_phy_reg_step_adjust(lut_mode_bits, 0, 0);

	pmu_cal_metric_log(5, 0x2a90000);
	pmu_slice_reg_query_mailbox_send(channel);

cleanup:
	pmu_cal_metric_log(4, 0x2b20000);
	pmu_dbyte_cal_strobe_pulse_seq(0, 0, 0);

	phy_write16(PHY_REG_DBYTE_BCAST_DQ_DLY0, 0);
	phy_write16(PHY_REG_DBYTE_BCAST_DQ_DLY1, 0);
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
 * pmu_dbyte_pin_mask_seq() - Apply multi-stage DBYTE pin mask delays across active slices
 * @mode_cond: Channel condition selector flag
 * @rank: DRAM rank index (0 = rank 0, 1 = rank 1)
 * @pin_mask: Logical pin bitmask
 */
void pmu_dbyte_pin_mask_seq(u32 mode_cond, u32 rank, u32 pin_mask)
{
	volatile u8 *dmem = (volatile u8 *)PMU_DMEM_BASE;
	__asm__("" : "+r"(dmem));

	u32 cond = (mode_cond != 1) ? 1 : 0;
	pmu_deskew_latch_seq();

	u8 count = dmem[0xb20 + (cond * 2) + rank];
	u32 slice_offset_base = (rank << 4) + (cond << 3);

	const volatile u8 *slices = dmem + PMU_DMEM_RANK_SLICE_MAP + slice_offset_base;

	volatile u16 *dbyte_base = (volatile u16 *)PHY_REG_DBYTE_BASE;
	__asm__("" : "+r"(dbyte_base));

	for (u32 i = 0; i < count; i++) {
		u8 slice = slices[i];
		u16 mask = (u16)pmu_dbyte_pin_mask_calc(0x80, slice);
		volatile u16 *slice_csr = (volatile u16 *)((uintptr_t)dbyte_base | (slice << 13));
		slice_csr[0x118 / 2] = mask;
	}

	pmu_delay_us(20000, 0);

	u32 active_pin_mask = pin_mask | 0x80;
	for (u32 i = 0; i < count; i++) {
		u8 slice = slices[i];
		u8 mask = (u8)pmu_dbyte_pin_mask_calc(active_pin_mask, slice);
		volatile u16 *slice_csr = (volatile u16 *)((uintptr_t)dbyte_base | (slice << 13));
		slice_csr[0x118 / 2] = mask;
		slice_csr[0x112 / 2] = 511;
		slice_csr[0x116 / 2] = 511;
	}

	pmu_delay_us(5000, 12);

	for (u32 i = 0; i < count; i++) {
		u8 slice = slices[i];
		u16 mask = ((u8)pmu_dbyte_pin_mask_calc(active_pin_mask, slice)) | 0x100;
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
 * pmu_cal_bist_lock_check() - Verify BIST PLL lock status and configure rate tier timing
 */
void pmu_cal_bist_lock_check(void)
{
	volatile u8 *dmem = (volatile u8 *)PMU_DMEM_BASE;
	__asm__("" : "+r"(dmem));

	u32 ch_offset = (dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET)) << 1;
	u16 lock_val_lo, lock_val_hi;
	pmu_phy_lock_status_read(&lock_val_hi, &lock_val_lo);

	volatile u16 *phy_ch = (volatile u16 *)(uintptr_t)(PHY_REG_MASTER_BASE | ch_offset);
	__asm__("" : "+r"(phy_ch));

	phy_ch[0x14 >> 1] = lock_val_lo & 0x1ff;
	phy_ch[0x2e >> 1] = lock_val_hi & 0x1ff;

	u8 dram_type = dmem[0x08];
	if (!dram_type)
		dram_type = 2;
	u16 freq = *(volatile u16 *)(dmem + PMU_DMEM_DRAM_FREQ_OFF);

	u32 clock_ratio = freq / dram_type;
	if (dram_type == 2)
		clock_ratio *= 2;

	u32 mult = (dram_type != 2) ? 4 : 2;
	u32 rate_calc = (freq / dram_type) * mult;

	u32 tier_clock = pmu_rate_tier_calc(clock_ratio);
	u32 tier_rate = pmu_rate_tier_calc(rate_calc);

	phy_ch[0x24 >> 1] = ((tier_rate >> 4) << 12) | ((tier_rate & 0xf) << 8) | tier_clock;
	phy_ch[0x30c >> 1] = tier_rate;
	phy_ch[0x30e >> 1] = *(volatile u16 *)(PHY_REG_DBYTE_BASE | 0x01aa);

	u32 slice_count = dmem_read32(PMU_DMEM_ACTIVE_SLICE_BITMAP);
	volatile u16 *dbyte = (volatile u16 *)(PHY_REG_DBYTE_BASE | 0x01b2);
	__asm__("" : "+r"(dbyte));
	for (u32 s = 0; s < slice_count; s++) {
		*(volatile u16 *)((uintptr_t)dbyte + s * 0x2000) = 0x9c;
	}

	pmu_cal_metric_log(4, 0x4d0002, lock_val_hi, lock_val_lo);
}

/**
 * pmu_cal_bist_search_win_setup() - Configure calibration BIST search window and VREF DAC
 * @stage: Setup stage selector (0 = fine search window clear, non-zero = active CSR setup)
 */
void pmu_cal_bist_search_win_setup(u32 stage)
{
	if (stage == 0) {
		pmu_cal_metric_log(10, 0x2130000);
		*(volatile u16 *)(PMU_DMEM_BASE | PMU_DMEM_TRAIN_MODE) = 0x0010;
		pmu_cal_search_win_init((u8 *)(PMU_DMEM_BASE | PMU_DMEM_SEARCH_WIN_FINE));
		pmu_cal_vref_dac_step_adjust(8);
		dmem_write8(PMU_DMEM_CAL_ACTIVE_FLAG, 1);
		dmem_write8(PMU_DMEM_CAL_ACTIVE_FLAG, 0);
		pmu_post_cmd_conditional_dispatch(4);
		*(volatile u16 *)(PMU_DMEM_BASE | PMU_DMEM_TRAIN_MODE) = 0x8010;
		return;
	}

	u32 csr_offset = dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET);
	u8 start_slice = *(volatile u8 *)(PMU_DMEM_BASE | PMU_DMEM_SLICE_START);
	u8 end_slice = *(volatile u8 *)(PMU_DMEM_BASE | PMU_DMEM_SLICE_END);
	u16 pin_delays[36];

	for (u32 fp = 0; fp < 2; fp++) {
		pmu_cal_strobe_pulse();
		u32 cal_mode = fp;
		u16 window_params_buf[4];
		pmu_cal_slice_step_diff_commit(cal_mode, window_params_buf, 1);
		pmu_phy_reset_pulse();

		u32 pin_idx = (u32)start_slice * 9;

		for (u32 slice = start_slice; slice <= end_slice; slice++) {
			u16 step_diff = window_params_buf[slice];
			pmu_cal_metric_log(4, 0x2920001, (u32)step_diff);

			u32 slice_mask = csr_offset | (slice << 12);
			u32 reg_addr = PHY_REG_DBYTE_BASE | ((slice_mask | (cal_mode + 0x2a)) << 1);
			u16 reg_val = phy_read16(reg_addr);
			u32 low6 = reg_val & 0x3f;
			step_diff = step_diff + low6 + 0x20;

			for (u32 pin = 0; pin < 9; pin++) {
				pin_delays[pin_idx] = step_diff;
				pmu_cal_metric_log(4, 0x2930004, pin_idx, low6, pin_idx, (s16)step_diff);
				pin_idx++;
			}

			phy_write16(reg_addr - 4, step_diff);
		}

		pmu_slice_delay_step_program(csr_offset | (cal_mode + 0x26), pin_delays, 1, 0, 0, 0);
	}
}

/**
 * pmu_cal_dbyte_deskew_results_apply() - Apply DBYTE per-slice pin deskew calibration results to PHY
 * @results: Pointer to calibration results array in DMEM
 * @format_sel: 2-bit selection index passed to pmu_2b_identity_lut
 * @stage_id: Telemetry stage identifier (0x11 for VREF/deskew stage)
 */
void pmu_cal_dbyte_deskew_results_apply(const void *results, u32 format_sel, u32 stage_id)
{
	const u8 *res8 = (const u8 *)results;
	volatile u8 *dmem = (volatile u8 *)PMU_DMEM_BASE;
	u32 lut_res = pmu_2b_identity_lut(format_sel);
	u32 csr_offset = *(const volatile u32 *)&dmem[0x470];
	u8 start_slice = dmem[0x0b68];
	u8 end_slice = dmem[0x0b69];

	if (stage_id == 0x11) {
		u8 b_mode = dmem[0x102];
		u8 val_512 = res8[512];
		u32 bit1 = b_mode & 2;
		pmu_cal_metric_log(4, 0x1600002, (u32)val_512, (u32)(b_mode >> bit1));

		if (bit1 != 0 && val_512 == 3) {
			phy_write16(PHY_REG_DBYTE_BCAST_MODE | (csr_offset << 1), 4);
		}

		for (u32 slice = start_slice; slice <= end_slice; slice++) {
			const u8 *slice_base = res8 + (slice * 20);
			for (u32 pin = 0; pin < 9; pin++) {
				const u8 *pin_ptr = slice_base + (pin * 2);
				u16 val_112 = *(const u16 *)(pin_ptr + 112);
				u16 val_32 = *(const u16 *)(pin_ptr + 32);
				pmu_cal_metric_log(4, 0x1610004, slice, pin, (u32)val_112, (u32)val_32);

				u32 pin_reg = (slice << 12) | (pin << 8) | csr_offset;
				u32 reg2 = PHY_REG_DBYTE_BASE | (((pin_reg | (lut_res + 0x10))) << 1);
				phy_write16(reg2 + 4, val_32);
				phy_write16(reg2, val_112);

				if (bit1 != 0 && val_512 == 3) {
					const u16 *extra_src = (const u16 *)(res8 + (slice * 80) + (pin * 8) + 192);
					u32 base = (PHY_REG_DBYTE_BASE | 0x009c) | (pin_reg << 1);
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

				u32 pin_reg = (lut_res + 0x26) | (slice << 12) | (pin << 8) | csr_offset;
				phy_write16(PHY_REG_DBYTE_BASE | (pin_reg << 1), val);
			}
			u16 extra_val = slice_pins[9];
			u32 extra_reg = (lut_res + 0x28) | (slice << 12) | csr_offset;
			phy_write16(PHY_REG_DBYTE_BASE | (extra_reg << 1), extra_val);
		}
	}
}

/**
 * pmu_slice_pulse_write() - Write calibration pulse value across array of byte slices
 * @slices: Pointer to array of DBYTE slice indices
 * @num_slices: Number of slices in array to program
 * @val: 16-bit calibration value written to slice register
 */
void pmu_slice_pulse_write(const u8 *slices, u32 num_slices, u16 val)
{
	for (u32 i = 0; i < num_slices; i++)
		phy_write16((PHY_REG_DBYTE_BASE | 0x0118) | ((u32)slices[i] << 13), val);
}

/**
 * pmu_cal_multi_rank_deskew_sweep() - Coordinate multi-rank slice deskew sweeps and calibration pulses
 * @mode: Deskew calibration mode flags ((mode & 3) == 2 selects rank 1)
 */
void pmu_cal_multi_rank_deskew_sweep(u32 mode)
{
	u8 rank = *(volatile u8 *)(PMU_DMEM_BASE | PMU_DMEM_CAL_RANK);
	u32 dmem_offset = (rank != 0) ? (PMU_DMEM_BASE | PMU_DMEM_CH1_RANK_CFG) : (PMU_DMEM_BASE | PMU_DMEM_CH0_RANK_CFG);
	u8 cal_cfg = *(volatile u8 *)dmem_offset;
	u32 mode_flag = ((mode & 3) == 2) ? 1 : 0;

	if (mode_flag != (cal_cfg & 1)) {
		pmu_slice_deskew_pulse_train((u32)(cal_cfg & 1), 1);
	}

	if (*(volatile u8 *)(PMU_DMEM_BASE | PMU_DMEM_FREQ_MODE) != 0) {
		pmu_ac_lane_profile_setup();
		pmu_delay_us(50000, 3);
	}

	u32 struct_base = PMU_DMEM_BASE | PMU_DMEM_CAL_STRUCT_BASE;
	pmu_tracker_field_extract((u8 *)struct_base, 0xb);
	pmu_tracker_field_extract((u8 *)struct_base, 0x11);
	pmu_tracker_field_extract((u8 *)struct_base, 0x12);

	u32 idx = mode_flag + (rank * 2);
	u8 count = *(volatile u8 *)((PMU_DMEM_BASE | PMU_DMEM_RANK_SLICE_COUNTS) + idx);
	const u8 *ptr = (const u8 *)((PMU_DMEM_BASE | PMU_DMEM_RANK_SLICE_MAP) + (idx << 3));

	pmu_deskew_latch_seq();

	pmu_slice_pulse_write(ptr, count, 256);

	pmu_delay_us(5000, 12);
	pmu_clk_gate_handoff();

	u32 csr_offset = *(volatile u32 *)(PMU_DMEM_BASE | PMU_DMEM_ACTIVE_CSR_OFFSET);
	phy_write16(PHY_REG_VREF_CTRL | (csr_offset << 1), 0);
	pmu_delay_us(5000, 0);

	pmu_cal_struct_to_shadow16(struct_base, 0x10);
	pmu_cal_struct_mask_and(struct_base, 0x10, 0xcf);
	pmu_cal_struct_mask_and(struct_base, 0x10, 0xff);

	u32 sub_table = struct_base + (idx * 54);
	pmu_cal_multi_rank_step_commit(mode, sub_table, 0, 0xfffeffff, (u32)-1, 1, 0);

	pmu_delay_us(14000, 10);

	pmu_slice_pulse_write(ptr, count, 0x1000);

	pmu_tracker_field_extract((u8 *)struct_base, 0x10);
	u32 reg = PHY_REG_MASTER_CFG | (csr_offset << 1);
	phy_write16(reg, phy_read16(reg) & 0xfeff);
	pmu_hw_timer_delay(0x14);
}

/**
 * pmu_cal_bist_pattern_setup() - Configure BIST test patterns, lane timing offsets, and PRBS sequences
 * @pattern_idx: BIST test pattern index (0..8 selecting from predefined pattern table)
 * @slice_or_lane: Target DBYTE slice or lane mask
 * @enable_deskew_reset: Non-zero to reset channel timing deskew and apply pattern offsets
 * @validation_mode: Validation mode parameter
 */
void pmu_cal_bist_pattern_setup(u32 pattern_idx, u32 slice_or_lane, u32 enable_deskew_reset, u32 validation_mode)
{
	static const u16 pats[9] = {
		0x5a3c, 0xff00, 0xa536, 0xaaaa, 0xa536, 0xb2b2, 0x8241, 0, 0x5a3c
	};
	u32 dram_type = dmem_read8(PMU_DMEM_DRAM_TYPE);
	u16 pat;
	u32 flag = 0;

	if (pattern_idx >= 9) {
		pmu_assert_or_halt(0, 0x1a40001);
		pat = 0;
	} else {
		pat = pats[pattern_idx];
		if (pattern_idx == 7)
			pat = dmem_read8(PMU_DMEM_BIST_PATTERN_LO) | ((u16)dmem_read8(PMU_DMEM_BIST_PATTERN_HI) << 8);
	}

	s8 offset_val = 0;
	if (enable_deskew_reset != 0) {
		u8 e6 = dmem_read8(PMU_DMEM_DESKEW_OFFSET_OVERRIDE);
		offset_val = e6 ? (s8)e6 : (s8)-86;
	}

	if (pattern_idx == 1 || (pattern_idx >= 3 && pattern_idx <= 7))
		pmu_channel_timing_deskew_reset((u8)pat, (pat >> 8) & 0xff, (u32)offset_val, (u32)offset_val, slice_or_lane);

	pmu_bist_lane_mask_set_all(0xffff);
	phy_write16(PHY_REG_BIST_CTRL_TRIG, 0);
	pmu_phy_lane_timing_offset_set(0xf, pat, pat, pat, pat);

	if (enable_deskew_reset != 0) {
		if (pattern_idx == 0) {
			phy_write16(PHY_REG_BIST_CMD, 22);
		} else {
			u32 valid_mode = (pattern_idx == 2 || pattern_idx == 8) ? 1 : 0;
			pmu_assert_or_halt(((validation_mode != 0) ? 1 : 0) | valid_mode, 0x1a50000);
			phy_write16(PHY_REG_BIST_CMD, 16);
			for (u32 i = 0; i < 8; i++) {
				u16 p = (offset_val & (1 << i)) ? (u16)~pat : pat;
				pmu_phy_lane_timing_offset_set(i, p, p, p, p);
			}
			pmu_phy_lane_timing_offset_set(8, pat, pat, pat, pat);
		}
	} else {
		phy_write16(PHY_REG_BIST_CMD, 16);
	}

	u16 val_e1ae = (pattern_idx == 1 || (pattern_idx >= 3 && pattern_idx <= 7)) ? 0 : 256;
	phy_write16(PHY_REG_BIST_STAT_FAIL, val_e1ae);
	phy_write16(PHY_REG_BIST_STAT_WORD, val_e1ae);

	u16 bist_ctrl = (dram_type == 4) ? 0x84 : 4;
	if (pattern_idx != 0) {
		phy_write16(PHY_REG_BIST_CTRL_MODE, bist_ctrl | 5);
	} else {
		phy_write16(PHY_REG_BIST_CTRL_MODE, bist_ctrl | 8);
		phy_write16(PHY_REG_BIST_CTRL_TRIG, 0);
	}
}

/**
 * pmu_cal_dbyte_deskew_pin_results_apply() - Apply DBYTE per-pin deskew calibration results to PHY registers
 */
void pmu_cal_dbyte_deskew_pin_results_apply(void)
{
	uintptr_t base_e48 = (PMU_DMEM_BASE | PMU_DMEM_PIN_CAL_BASE_TAPS);
	uintptr_t base_e4a = (PMU_DMEM_BASE | PMU_DMEM_PIN_CAL_DELTA_TAPS);
	u32 csr_offset = dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET);

	for (u32 rank = 0; rank < 2; rank++) {
		u8 mask = dmem_read8(PMU_DMEM_CH0_RANK_EN) | dmem_read8(PMU_DMEM_CH1_RANK_EN);
		if (!(mask & (1 << rank))) {
			base_e48 += 5280;
			base_e4a += 5280;
			continue;
		}

		u32 start_slice = (u32)dmem_read8(PMU_DMEM_SLICE_START);
		u32 end_slice = (u32)dmem_read8(PMU_DMEM_SLICE_END);

		uintptr_t slice_e48 = base_e48 + start_slice * 1320;
		uintptr_t slice_e4a = base_e4a + start_slice * 1320;

		for (u32 slice = start_slice; slice <= end_slice; slice++) {
			if (dmem_read8(PMU_DMEM_ACTIVE_SLICE_MASK_CAL) & (1 << slice)) {
				u32 slice_base = csr_offset | (slice << 12) | rank;
				uintptr_t ptr_a = slice_e48;
				uintptr_t ptr_b = slice_e4a;

				for (u32 pin = 0; pin < 10; pin++) {
					u16 val_a = *(volatile u16 *)(uintptr_t)ptr_a;
					s8 val_b = *(volatile s8 *)(uintptr_t)ptr_b;
					ptr_a += 132;
					ptr_b += 132;

					u16 sum = (u16)(val_a + (s32)val_b);
					if (pin == 9) {
						uintptr_t reg = (PHY_REG_DBYTE_BASE | 0x0050) | (slice_base << 1);
						*(volatile u16 *)(uintptr_t)reg = sum;
					} else {
						uintptr_t reg = (PHY_REG_DBYTE_BASE | 0x004c) | ((slice_base | (pin << 8)) << 1);
						*(volatile u16 *)(uintptr_t)reg = sum;
					}
				}

				if (dmem_read8(PMU_DMEM_CAL_OVERRIDE_FLAG) == 0) {
					uintptr_t tbl = (PMU_DMEM_BASE | PMU_DMEM_PIN_CAL_BASE_TAPS) + rank * 5280 + slice * 1320;
					u8 cal_step = *(volatile u8 *)(uintptr_t)(tbl + 3);
					u32 is_hi = (slice > (u32)dmem_read8(PMU_DMEM_RANK_BOUNDARY)) ? 1 : 0;

					pmu_phy_mode_cfg_dispatch(is_hi ? 2 : 1);
					u32 swap = pmu_dq_swap_query(rank, slice);
					dmem_write8(PMU_DMEM_CAL_RANK, is_hi);
					u32 rank_idx = rank + (is_hi ? 2 : 0);

					if (swap != 0) {
						dmem_write8(PMU_DMEM_CAL_RANK + rank_idx + 0xc, cal_step);
						pmu_cal_pulse_seq_coordinator(rank, cal_step, 1);
						dmem_write8(is_hi ? (rank ? 0x51 : 0x50) : (rank ? 0x36 : 0x35), cal_step);
					} else {
						dmem_write8(PMU_DMEM_CAL_RANK + rank_idx + 0x10, cal_step);
						pmu_cal_pulse_seq_coordinator(rank, cal_step, 2);
						dmem_write8(is_hi ? (rank ? 0xdf : 0xda) : (rank ? 0xd5 : 0xd0), cal_step);
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
