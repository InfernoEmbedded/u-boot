// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2026 Alastair D'Silva <alastair@d-silva.org>
 *
 * Allwinner A733 Synopsys DDR PHY PMU Mailbox POST IPC & Assertion Handlers
 */

#include "pmu_fw.h"

static void pmu_mailbox_post_exec_wait(void);

/**
 * pmu_mailbox_post_cmd_dispatch() - Dispatch 16-bit POST command to PHY mailbox
 * @cmd: Command code to transmit (pulses 0x90040238 for terminal codes 0xff or 7)
 */
noinline void pmu_mailbox_post_cmd_dispatch(u32 cmd)
{
	if (cmd == 255U || cmd == 7U) {
		phy_write16(PHY_REG_CMD_DISPATCH,
			    (cmd == 255U) ? 0x0004U : 0x0001U);
		phy_write16(PHY_REG_CMD_DISPATCH, 0x0000U);
	}

	phy_write16(PHY_REG_MAILBOX_STREAM, (u16)cmd);
	phy_write16(PHY_REG_MAILBOX_STREAM + 2U, 0x0000U);
	pmu_mailbox_post_exec_wait();
}

/**
 * pmu_mailbox_post_exec_wait() - Handshake PHY mailbox interrupt and busy status
 *
 * Waits for bit 0 of PHY register 0x90180008 to clear, triggers the mailbox
 * interrupt at 0x90180066 with 1, and waits for bit 0 of 0x90180008 to assert.
 */
static noinline void pmu_mailbox_post_exec_wait(void)
{
	while ((phy_read16(PHY_REG_MAILBOX_STAT) & 1U) != 0U)
		;
	phy_write16(PHY_REG_MAILBOX_TRIG, 0x0001U);
	while ((phy_read16(PHY_REG_MAILBOX_STAT) & 1U) == 0U)
		;
}

/**
 * pmu_assert_or_halt() - Validate condition or record panic code and halt PMU
 * @cond: Condition value (non-zero continues execution; zero triggers fatal halt)
 * @assert_code: 32-bit diagnostic code recorded at DCCM 0x8000000c on failure
 */
void pmu_assert_or_halt(u32 cond, u32 assert_code)
{
	if (cond != 0U)
		return;

	*(__volatile__ u32 *)PMU_DMEM_ASSERT_CODE = assert_code;
	*(__volatile__ u8 *)PMU_DMEM_PSTATE_FLAGS |= 0x02U;
	pmu_mailbox_post_cmd_dispatch(255U);
	__builtin_arc_sr(0U, PMU_AUX_RTC_CTRL);
	__builtin_arc_flag(1);
	for (;;)
		;
}

/**
 * pmu_ac_lane_profile_setup() - Program AC lane profile registers and gate settle delay
 */
void pmu_ac_lane_profile_setup(void)
{
	const u8 *ac_desc = (const u8 *)PMU_DMEM_AC_PROFILE_EN_ADDR;
	u16 saved_gate_mask;
	u16 saved_gate_ctrl;
	u8 cal_status;
	s32 delay_diff;

	saved_gate_mask = phy_read16(PHY_REG_GATE_MASK);
	saved_gate_ctrl = phy_read16(PHY_REG_GATE_CTRL);

	phy_write16(PHY_REG_GATE_CTRL, 0U);
	phy_write16(PHY_REG_GATE_MASK, 0x00ffU);

	cal_status = *(const u8 *)PMU_DMEM_TIMING_LIM;
	phy_write16(PHY_REG_VREF_STAT, (u16)cal_status | 0x0100U);

	delay_diff = (s32)(*(const u8 *)PMU_DMEM_LANE_STATUS + cal_status) - 2;

	if (ac_desc[0] != 0U) {
		const u16 *ac_words = (const u16 *)(ac_desc + 4);
		u32 csr_off = pmu_sdata()->active_csr_offset << 1;

		phy_write16(PHY_REG_AC_DRIVE_CFG5 + csr_off, ac_words[0]);
		phy_write16(PHY_REG_AC_DRIVE_CFG6 + csr_off, ac_words[1]);
		phy_write16(PHY_REG_AC_DRIVE_CFG3 + csr_off, ac_words[2]);
		phy_write16(PHY_REG_AC_DRIVE_CFG4 + csr_off, ac_words[3]);
		phy_write16(PHY_REG_AC_DRIVE_CFG0 + csr_off, ac_desc[3]);
		phy_write16(PHY_REG_AC_DRIVE_CFG1 + csr_off, ac_desc[1]);
		phy_write16(PHY_REG_AC_DRIVE_CFG2 + csr_off, ac_desc[2]);
	}

	if (delay_diff < 0)
		delay_diff += (s32)((u32)cal_status *
				    *(const u8 *)PMU_DMEM_PLL_RATIO);

	phy_write16(PHY_REG_DRAM_TYPE_CFG, (u16)delay_diff);
	pmu_delay_us(11000U, 0U);

	phy_write16(PHY_REG_GATE_MASK, saved_gate_mask);
	phy_write16(PHY_REG_GATE_CTRL, saved_gate_ctrl);

	*(__volatile__ u8 *)PMU_DMEM_AC_SIGNED_STEP =
		(u8)(((cal_status >> 1) & ~1U) - 1U);
}

/**
 * pmu_cmd_code_map_log() - Map command/subcode to CBT lane limit code and log
 * @cmd_code: Primary command code
 * @sub_code: Sub-command selector (0 selects @cmd_code mapping)
 */
void pmu_cmd_code_map_log(u32 cmd_code, u32 sub_code)
{
	u8 map_code = 9U;

	if (sub_code == 0U) {
		u32 masked = cmd_code & 0x0fffU;

		if (masked >= 257U && masked <= 261U)
			map_code = (u8)(masked - 255U);
		else if (masked == 272U)
			map_code = 7U;
		else if (masked == 512U)
			map_code = 8U;
	} else if (sub_code == 1U) {
		map_code = 0U;
	} else if (sub_code == 2U) {
		map_code = 1U;
	} else if (sub_code == 4U || sub_code == 17U) {
		map_code = 2U;
	} else if (sub_code == 18U) {
		map_code = 3U;
	} else if (sub_code == 8U || sub_code == 19U) {
		map_code = 4U;
	} else if (sub_code == 20U) {
		map_code = 5U;
	} else if (sub_code == 21U) {
		map_code = 6U;
	} else if (sub_code == 26U) {
		map_code = 7U;
	} else if (sub_code == 0x20U) {
		map_code = 8U;
	} else {
		pmu_assert_or_halt(0U, PMU_ERR_CMD_SUBCODE_INVALID);
		map_code = pmu_sdata()->freq_mode;
	}

	pmu_sdata()->freq_mode = map_code;
}

/**
 * pmu_slice_metric_scan_and_program() - Scan slice metrics and program DBYTE delay CSRs
 */
void pmu_slice_metric_scan_and_program(void)
{
	u8 start_slice;
	u8 end_slice;
	u16 num_ranks;
	u32 ch_offset;
	u32 slice;

	start_slice = dmem_read8(PMU_DMEM_SLICE_START);
	end_slice = dmem_read8(PMU_DMEM_SLICE_END);
	num_ranks = pmu_sdata()->num_ranks;
	ch_offset = pmu_sdata()->active_csr_offset;

	for (slice = start_slice; slice <= end_slice; slice++) {
		const __volatile__ u16 *slice_metrics =
			(const __volatile__ u16 *)(PMU_DMEM_CBT_DELAY_ARR +
						   (slice << 3));
		u16 max_val = slice_metrics[0];
		u16 min_val = max_val;
		u32 k;
		u32 midpoint;
		u32 diff;
		u16 reg_val_lo;
		u16 reg_val_hi;
		u32 slice_base;
		u32 bank;

		for (k = 0U; k < num_ranks; k++) {
			u16 val = slice_metrics[k];

			if (val > max_val)
				max_val = val;
			if (val < min_val)
				min_val = val;
		}

		midpoint = (max_val + min_val) >> 1;
		diff = 12U - midpoint;

		reg_val_lo = (u16)((diff << 4) | midpoint);
		reg_val_hi = (u16)(diff | (midpoint << 4));
		slice_base = PHY_REG_DBYTE_BASE |
			     (((slice << 12) | ch_offset) << 1);

		for (bank = 0U; bank < 2U; bank++) {
			u32 reg_low = (bank != 0U) ? 0x5eU : 0x62U;
			u32 reg_high = reg_low + 2U;
			u16 w1 = (*(__volatile__ const u16 *)
				  (PHY_REG_DBYTE_BASE +
				   ((reg_low | ch_offset) << 1)) & 768U) |
				 reg_val_lo;
			u16 w2 = (*(__volatile__ const u16 *)
				  (PHY_REG_DBYTE_BASE +
				   ((reg_high | ch_offset) << 1)) & 768U) |
				 reg_val_hi;
			u32 a1 = slice_base + (reg_low << 1);
			u32 a2 = slice_base + (reg_high << 1);

			*(__volatile__ u16 *)a1 = w1;
			*(__volatile__ u16 *)(a1 + 2U) = w1;
			*(__volatile__ u16 *)a2 = w2;
			*(__volatile__ u16 *)(a2 + 2U) = w2;
		}
	}
}

/**
 * pmu_mr22_bit4_check() - Query bit 4 of training configuration byte 0x96
 *
 * Return: 1 if bit 4 of DCCM 0x80000096 is set, 0 otherwise.
 */
u32 pmu_mr22_bit4_check(void)
{
	return (*(__volatile__ u8 *)PMU_DMEM_MR22_ADDR >> 4) & 1;
}

/**
 * pmu_stage_slice_flag_check() - Extract 2-bit mode field gated by DCCM 0x407 bit 6
 *
 * Return: Bits [4:3] of halfword at DCCM 0x80000408 if bit 6 of DCCM 0x80000407
 *         is set, or 0 otherwise.
 */
u32 pmu_stage_slice_flag_check(void)
{
	u32 flag = pmu_sdata()->stage_slice_idx;

	if (!(flag & 0x40U))
		return 0;

	return (pmu_sdata()->stage_lane_idx >> 3) & 3U;
}

/**
 * pmu_check_freq_lt_3200() - Check whether DRAM data rate is below 3200 MT/s
 *
 * Return: 1 if halfword at DCCM 0x80000006 is less than 3200, 0 otherwise.
 */
u32 pmu_check_freq_lt_3200(void)
{
	return dmem_read16(PMU_DMEM_DRAM_FREQ_OFF) < 3200U;
}

/**
 * pmu_is_lpddr4() - Check if DRAM protocol configuration is set to LPDDR4
 *
 * Return: 1 if bits [1:0] of DCCM 0x8000008a equal 1, 0 otherwise.
 */
u32 pmu_is_lpddr4(void)
{
	return (dmem_read8(PMU_DMEM_MR19_OFF) & 3U) == 1U;
}

/**
 * pmu_is_lpddr5() - Check if DRAM protocol mode bits [1:0] at 0x8e are zero
 *
 * Return: 1 if bits [1:0] of DCCM 0x8000008e equal 0, 0 otherwise.
 */
u32 pmu_is_lpddr5(void)
{
	return (dmem_read8(PMU_DMEM_MR20_ADDR) & 3U) == 0U;
}

/**
 * pmu_check_stage() - Check whether stage index belongs to {0, 6, 14, 40}
 * @val: Calibration stage index to test
 *
 * Return: 1 if @val is 0, 6, 14, or 40; 0 otherwise.
 */
u32 pmu_check_stage(u32 val)
{
	if (val <= 14)
		return (0x4041U >> val) & 1;
	return val == 40;
}

/**
 * pmu_cal_marker_clear() - Clear 16-bit calibration state halfword at DCCM 0x80000458
 */
void pmu_cal_marker_clear(void)
{
	pmu_sdata()->cal_marker_a = 0;
}

/**
 * pmu_cbt_cal_active_clear() - Clear byte flag at DCCM 0x8000045c to 0
 */
void pmu_cbt_cal_active_clear(void)
{
	pmu_sdata()->cbt_cal_active = 0;
}

/**
 * pmu_cbt_cal_active_set() - Set byte flag at DCCM 0x8000045c to 1
 */
void pmu_cbt_cal_active_set(void)
{
	pmu_sdata()->cbt_cal_active = 1;
}

/**
 * pmu_cbt_step_stat_set() - Set byte flag at DCCM 0x8000046b to 1
 */
void pmu_cbt_step_stat_set(void)
{
	pmu_sdata()->cbt_step_stat = 1;
}

/**
 * pmu_cbt_config_get() - Read calibration status byte from DCCM 0x8000046a
 *
 * Return: 8-bit value stored at DCCM 0x8000046a.
 */
u8 pmu_cbt_config_get(void)
{
	return pmu_sdata()->cbt_config;
}

/**
 * pmu_cal_mode_flag_get() - Read configuration byte from DCCM 0x80000005
 *
 * Return: 8-bit value stored at DCCM 0x80000005.
 */
u8 pmu_cal_mode_flag_get(void)
{
	return *(__volatile__ u8 *)PMU_DMEM_CAL_MODE_FLAG_ADDR;
}

/**
 * pmu_lane_mask_bit_get() - Extract bit from DCCM 0x8000001c status byte
 * @word_idx: 2-bit pair index (multiplied by 2)
 * @bit_shift: Additional bit offset within the pair
 *
 * Return: Extracted bit ((dmem[0x1c] >> ((word_idx << 1) + bit_shift)) & 1).
 */
u32 pmu_lane_mask_bit_get(u32 word_idx, u32 bit_shift)
{
	u32 val = dmem_read8(PMU_DMEM_LANE_MASK);
	u32 shift = (word_idx << 1) + bit_shift;

	return (val >> shift) & 1U;
}

/**
 * pmu_phy_query_operating_mode() - Query normalized PHY operating mode
 *
 * Return: Operating mode index (0..4) derived from PHY register 0x90020012.
 *
 * Reads the channel-adjusted PHY operating mode register at 0x90020012 and
 * maps raw mode bits [2:0] into the normalized PMU operating mode code.
 */
u32 pmu_phy_query_operating_mode(void)
{
	u32 ch_offset = *(__volatile__ u32 *)PMU_DMEM_ACTIVE_CH_OFFSET;
	u16 val = *(__volatile__ u16 *)(PHY_REG_OP_MODE | (ch_offset << 1));
	u32 mode = val & 7;

	if (mode <= 1)
		return mode;

	return ((val & 6) >> 1) + 1;
}

/**
 * pmu_cal_marker_pair_set() - Write 16-bit value to DCCM 0x80000458 and 0x8000045e
 * @val: Halfword value to store at both offsets
 */
void pmu_cal_marker_pair_set(u16 val)
{
	pmu_sdata()->cal_marker_a = val;
	pmu_sdata()->cal_marker_b = val;
}

/**
 * pmu_cbt_config_set() - Set DCCM 0x8000045a flag to 1 and store byte at 0x8000046a
 * @val: Byte value to write to DCCM 0x8000046a
 */
void pmu_cbt_config_set(u8 val)
{
	pmu_sdata()->cbt_state = 1;
	pmu_sdata()->cbt_config = val;
}

/**
 * pmu_post_cmd_conditional_dispatch() - Conditionally dispatch post-command and sync clock gate
 * @val: Command code to pass to pmu_mailbox_post_cmd_dispatch()
 *
 * Dispatches @val to pmu_mailbox_post_cmd_dispatch() when DCCM 0x80000012 is
 * at most 0xc8, then pulses the PHY clock gate synchronizer.
 */
void pmu_post_cmd_conditional_dispatch(u32 val)
{
	u8 threshold = *(__volatile__ u8 *)PMU_DMEM_POST_THRESH;

	if (threshold <= 0xc8)
		pmu_mailbox_post_cmd_dispatch(val);

	pmu_phy_clk_gate_sync_pulse();
}

/**
 * pmu_cal_stride_get() - Read calibration step stride from DCCM 0x80000458
 *
 * Return: 16-bit halfword at DCCM 0x80000458 logically shifted right by 2 bits.
 */
u16 pmu_cal_stride_get(void)
{
	return pmu_sdata()->cal_marker_a >> 2;
}

/**
 * pmu_cal_marker_verify() - Compare calibration markers A and B and dispatch fallback
 *
 * Return: 0 if halfwords at DCCM 0x80000458 and 0x8000045e match, or the result
 *         of pmu_cal_marker_status_update() when they differ.
 */
u16 pmu_cal_marker_verify(void)
{
	u32 marker_a;
	u32 marker_b;
	u32 diff;

	marker_a = pmu_sdata()->cal_marker_a;
	marker_b = pmu_sdata()->cal_marker_b;
	diff = marker_a ^ marker_b;
	if (diff == 0U)
		return (u16)diff;

	return pmu_cal_marker_status_update();
}

/**
 * pmu_dram_cfg_bit13_check() - Query bit 13 of DRAM config flags at DCCM 0x0a
 * @rank: Rank index (returns 0 if equal to 4)
 * @flag: Secondary gating flag (returns 0 if non-zero)
 *
 * Return: Bit 13 of DCCM 0x8000000a when @rank != 4 and @flag == 0, else 0.
 */
u32 pmu_dram_cfg_bit13_check(u32 rank, u32 flag)
{
	u16 cfg;

	if (rank == 4U || flag != 0U)
		return 0U;

	cfg = *(__volatile__ u16 *)(PMU_DMEM_BASE + PMU_DMEM_DRAM_CFG_FLAGS);
	return (u32)((cfg >> 13) & 1U);
}

/**
 * pmu_dmem_training_flags_eval() - Assemble 5-bit composite Vref/training mode byte
 *
 * Return: 5-bit composite flag byte combining DCCM 0x62, 0x72, and 0x96.
 */
u8 pmu_dmem_training_flags_eval(void)
{
	u32 mr3_val = dmem_read8(PMU_DMEM_MR3_ADDR);
	u32 mr13_val = dmem_read8(PMU_DMEM_MR13_ADDR);
	u32 mr22_val = dmem_read8(PMU_DMEM_MR22_ADDR);
	u32 res = (mr3_val >> 7) | ((mr3_val >> 5) & 2U);

	if (mr22_val & 0x40U)
		res |= 4U;
	if (mr22_val & 0x10U)
		res |= 8U;
	if (!(mr13_val & 0x20U))
		res |= 16U;

	return (u8)res;
}

/**
 * pmu_cal_marker_status_update() - Recompute calibration marker A and clear tracker
 *
 * Return: Updated 16-bit calibration marker written to DCCM 0x80000458.
 *
 * Sets DCCM 0x80000dc4 to 1 when 0x80000dbe is non-zero, invokes
 * (u16)pmu_cal_tracker_csr_commit(0x80000d9c) to compute the updated marker stored at
 * DCCM 0x80000458, zeroes the 32-byte tracker block at 0x80000d9c, and clears
 * the four trailing status halfwords at 0x80000dbe..0x80000dc4.
 */
u16 pmu_cal_marker_status_update(void)
{
	__volatile__ struct pmu_cal_tracker *trk = pmu_tracker();
	u16 zero = 0U;
	u16 marker;

	if (trk->repeat_cnt != 0U)
		trk->commit_flag = 1U;

	marker = (u16)pmu_cal_tracker_csr_commit(trk);
	pmu_sdata()->cal_marker_a = marker;
	memset((void *)trk->slot0, 0, PMU_TRACKER_SLOTS_BYTES);
	trk->commit_flag = zero;
	trk->ext_flag = zero;
	trk->slot0_mode = zero;
	trk->repeat_cnt = zero;
	return marker;
}

/**
 * pmu_cal_pad_voltage_dac_set() - Evaluate Vref DAC polarity and override state
 * @val: Input step/direction indicator (tested for non-zero byte value)
 * @mode: Vref domain selector ((@mode & ~1) == 2 selects bit 7 of DCCM 0x62, else bit 6)
 *
 * Return: 1 or 0 from DCCM 0x800000e7 override bits [1:0] when active, or
 *         ((u8)@val != 0) masked by the selected polarity bit of DCCM 0x80000062.
 */
u32 pmu_cal_pad_voltage_dac_set(u32 val, u32 mode)
{
	u32 ctrl = dmem_read8(PMU_DMEM_VREF_OVR_CTRL_ADDR);
	u32 dir;
	u32 bit;

	if (ctrl & 3U)
		return ctrl & 1U;

	dir = dmem_read8(PMU_DMEM_MR3_ADDR);
	bit = ((mode & ~1U) == 2U) ? (dir >> 7) : ((dir >> 6) & 1U);
	return ((u8)val != 0U) & bit;
}

/**
 * pmu_cal_channel_rank_config_get() - Interrogate channel and rank configuration
 * @rank: Logical rank/slice index (0..1 for channel 0, 2..3 for channel 1)
 * @out_cfg: Output pointer receiving the 16-bit rank polarity bit (0 or 1)
 * @out_mode_sel: Output pointer receiving the 1-hot rank bitmask (1 << bit)
 * @out_num_slices: Output pointer receiving the active channel mode (1 or 2)
 *
 * Return: 0 if the requested rank is enabled in the channel mask (updating
 *         *@out_num_slices), or 1 if disabled.
 */
u32 pmu_cal_channel_rank_config_get(u32 rank, u16 *out_cfg, u8 *out_mode_sel,
				    u8 *out_num_slices)
{
	u32 idx = (u16)rank;
	u32 is_ch1 = (idx >= 2U);
	u32 base = is_ch1 ? PMU_DMEM_CH1_ACTIVE_FLAG :
			    PMU_DMEM_CH0_ANALOG_EN;
	const __volatile__ u8 *cfg_base =
		(const __volatile__ u8 *)dmem_ptr(base);
	u32 bit = (cfg_base[13] ^ idx) & 1U;
	u8 mask = (u8)(1U << bit);

	*out_cfg = (u16)bit;
	*out_mode_sel = mask;
	if ((cfg_base[0] & mask) == 0U)
		return 1U;

	*out_num_slices = (u8)(is_ch1 + 1U);
	return 0U;
}

/**
 * pmu_cal_channel_rank_bounds_verify() - Dispatch fatal assertion for status 1..7
 * @err_id: Error status code (1..7)
 */
void pmu_cal_channel_rank_bounds_verify(u32 err_id)
{
	u32 idx = (err_id & 0xffU) - 1U;

	if (idx < 7U)
		pmu_assert_or_halt(0U, PMU_ERR_RANK_BOUNDS_BASE + (idx << 16));
}

/**
 * pmu_dbyte_dq_deskew_regs_save_and_ramp() - Extract and clamp gate eye margin descriptors
 * @out_a: Output pointer for primary margin descriptor byte
 * @out_b: Output pointer for secondary margin descriptor byte
 * @out_c: Output pointer for tertiary margin descriptor byte
 * @flag: Mode/rank flag selector
 * @extra: Secondary offset selector
 */
void pmu_dbyte_dq_deskew_regs_save_and_ramp(u8 *out_a, u8 *out_b, u8 *out_c,
					    u32 flag, u32 extra)
{
	u32 f8 = (u8)flag;
	u32 e8 = (u8)extra;
	u32 offset = (f8 == 4U) ? 4U :
		     (((e8 == 0U) ? 2U : 0U) + (((f8 >> 1) == 1U) ? 1U : 0U));
	const u8 *desc = (const u8 *)dmem_ptr(PMU_DMEM_GATE_DESC + offset);
	u8 c = desc[10];

	if (c == 0U)
		return;

	*out_a = desc[0];
	*out_b = desc[5];
	*out_c = c;

	if (f8 != 4U && e8 == 0U) {
		if ((dmem_read16(PMU_DMEM_DRAM_CFG_FLAGS) & 0x2000U) != 0U) {
			u32 sum = (u32)(*out_b) + (u32)(*out_a);
			u32 lim = 127U;

			if (sum > lim)
				sum = lim;
			*out_b = (u8)sum;
			*out_a = 0U;
		}
	}
}
