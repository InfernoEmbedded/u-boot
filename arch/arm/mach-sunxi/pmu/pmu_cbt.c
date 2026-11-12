// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2026 Alastair D'Silva <alastair@d-silva.org>
 *
 * Allwinner A733 Synopsys DDR PHY PMU Command Bus Training (CBT) Helpers
 */

#include "pmu_fw.h"

/**
 * pmu_cal_rank_offset_get() - Compute JEDEC CBT read latency entry for rank and channel
 * @rank: Active DRAM rank index (0 or 1)
 * @channel: Active DRAM channel index (0 or 1)
 *
 * Extracts the 4-bit MR2[3:0] RL index from byte +2 of the 54-byte per-slice
 * descriptor at DCCM 0x800009a4 + (@rank * 2 + @channel) * 54 and computes the
 * corresponding JEDEC read latency (doubled when pmu_cbt_config_get() == 2).
 *
 * Return: 8-bit CBT read latency entry for @rank and @channel.
 */
u8 pmu_cal_rank_offset_get(u32 rank, u32 channel)
{
	u32 slice_idx = (rank << 1) + channel;
	u32 desc_addr = PMU_DMEM_CAL_STRUCT_ADDR +
			slice_idx * PMU_SLICE_DESC_SIZE;
	u32 idx = dmem_read8(desc_addr + PMU_SLICE_DESC_MR2_OFF) & 0x0fU;
	u32 rl;

	if (idx >= 12U)
		return 0U;

	rl = idx + 3U + (idx >= 4U) + (idx >= 7U) + (idx >= 9U);
	if (pmu_cbt_config_get() == 2U)
		return (idx < 6U) ? (u8)(rl << 1) : 0U;

	return (u8)rl;
}

/**
 * pmu_cal_cbt_desc_unpack() - Unpack CBT step multiplier from slice descriptor
 * @rank: Active DRAM rank index (0 or 1)
 * @channel: Active DRAM channel index (0 or 1)
 *
 * Return: 8 or 4 when DCCM 0x80000008 == 2 (else 4 or 2) based on bit 4 of
 *         descriptor byte +3 at 0x800009a4 + (@rank * 2 + @channel) * 54.
 */
u32 pmu_cal_cbt_desc_unpack(u32 rank, u32 channel)
{
	u32 slice_idx = (rank << 1) + channel;
	u8 byte_val = dmem_read8(PMU_DMEM_CAL_STRUCT_ADDR +
				 slice_idx * PMU_SLICE_DESC_SIZE +
				 PMU_SLICE_DESC_MR3_OFF);
	u32 res = (byte_val & 0x10U) ? 2U : 4U;

	if (dmem_read8(PMU_DMEM_PLL_RATIO) == 2U)
		res <<= 1;

	return res;
}

/**
 * pmu_cal_search_win_max_calc() - Scan slice registers and update master delay table
 * @channel: Target DRAM channel index (0 or 1)
 * @rank: Target DRAM rank index (0 or 1)
 * @dir: Direction step selector (0 adds +4 offset, non-zero adds 0)
 */
void pmu_cal_search_win_max_calc(u32 channel, u32 rank, u32 dir)
{
	u8 slice_start;
	u8 slice_end;
	s32 max_val = -1;
	u32 addr;
	s32 i;
	u8 pll_ratio;
	u32 shift;
	s32 v;

	if (rank == 1U && dmem_read8(PMU_DMEM_CH1_ACTIVE_FLAG) == 0U) {
		__volatile__ u16 *tbl = pmu_sdata()->master_dly_tbl[channel];

		tbl[1] = tbl[0];
		return;
	}

	slice_start = dmem_read8((rank != 0U) ? PMU_DMEM_RANK1_SLICE_START :
						PMU_DMEM_RANK0_SLICE_START);
	slice_end = dmem_read8((rank != 0U) ? PMU_DMEM_RANK1_SLICE_END :
					      PMU_DMEM_RANK_BOUNDARY);
	addr = PHY_REG_DBYTE_BASE |
	       (((pmu_sdata()->active_csr_offset |
		  (channel + (((dir == 0U) ? 1U : 0U) << 2) + 0x20U) |
		  ((u32)slice_start << 12)) << 1));

	for (i = (s32)slice_end + 1 - (s32)slice_start; i > 0; i--) {
		s16 val = (s16)phy_read16(addr);

		if ((s32)val > max_val)
			max_val = (s32)val;
		addr += PHY_DBYTE_SLICE_STRIDE;
	}

	pll_ratio = dmem_read8(PMU_DMEM_PLL_RATIO);
	shift = 511U >> (pll_ratio == 2U);

	v = (max_val >> ((pll_ratio >> 1) + 7U)) +
	    (s8)dmem_read8(PMU_DMEM_SEARCH_WINDOW_BIAS_LO) +
	    (((u32)max_val & shift) != 0U ? 1 : 0) + 2;
	pmu_sdata()->master_dly_tbl[channel][rank] = (u16)v;
}

/**
 * pmu_phy_reg_offset_diff_adjust() - Program CA Vref DAC and trim registers for rank
 * @rank: Target rank index (0 or non-zero)
 */
void pmu_phy_reg_offset_diff_adjust(u32 rank)
{
	__volatile__ u16 *apb =
		(__volatile__ u16 *)(PHY_REG_CA_VREF_DAC +
				     (pmu_sdata()->active_csr_offset << 1));
	u8 ca_vref_r1 = dmem_read8(PMU_DMEM_CA_VREF_RANK1_ADDR);
	u8 ca_vref_r0 = dmem_read8(PMU_DMEM_CA_VREF_RANK0_ADDR);
	u8 val = (rank == 0U) ? ca_vref_r0 : ca_vref_r1;
	s32 diff;
	u16 ca_reg_a, ca_reg_b, ca_reg_c;
	u16 ca_reg_val;

	apb[0] = (apb[0] & 0x3f00U) | val;
	apb[1] = (apb[1] & 0x3f00U) | val;
	apb[4] = (apb[4] & 0x3f00U) | val;

	diff = (s32)ca_vref_r1 - (s32)ca_vref_r0;
	if (rank == 0U)
		diff = -diff;

	ca_reg_a = apb[13];
	ca_reg_val = (u16)((ca_reg_a & 0x3f00U) | (((ca_reg_a & 0x3fU) + diff) & 0x3fU));
	apb[13] = ca_reg_val;
	apb[14] = ca_reg_val;

	ca_reg_b = apb[15];
	apb[15] = (u16)((ca_reg_b & 0x3f00U) | (((ca_reg_b & 0x3fU) + diff) & 0x3fU));

	ca_reg_c = apb[16];
	apb[16] = (u16)((ca_reg_c & 0x3f00U) | (((ca_reg_c & 0x3fU) + diff) & 0x3fU));
}

/**
 * pmu_dmem_reg_stream_unpack() - Clear CBT/deskew status CSRs and pulse reset
 * @cfg_desc: Pointer to CBT configuration descriptor (32-bit offset at +0x18)
 */
void pmu_dmem_reg_stream_unpack(const void *cfg_desc)
{
	__volatile__ u16 *apb = (__volatile__ u16 *)PHY_REG_RESET_TRIGGER;
	__volatile__ u16 *bcast_base = (__volatile__ u16 *)PHY_REG_DBYTE_BCAST_BASE;
	const u8 *desc = (const u8 *)cfg_desc;
	u32 rank_cfg_off = *(const u32 *)(desc + 0x18U);
	u32 zero = 0U;
	u16 val;

	phy_write16(PHY_REG_DBYTE_UPPER_SAMPLE_B, (u16)zero);
	bcast_base[176] = (u16)zero;
	bcast_base[177] = (u16)zero;
	apb[71] = (u16)zero;
	apb[0] = (u16)zero;
	bcast_base[rank_cfg_off] = (u16)zero;

	val = phy_read16(PHY_REG_BIST_PLL_CFG);
	phy_write16(PHY_REG_AC_BCAST, val & (u16)~0x0010U);

	pmu_phy_cal_trig_pulse();
	if (pmu_sdata()->freq_mode <= 1U)
		pmu_phy_soft_reset_pulse();
}

/**
 * pmu_cbt_delay_offset_lookup() - Compute JEDEC CBT write latency offset for rank and channel
 * @rank: Active DRAM rank index (0 or 1)
 * @channel: Active DRAM channel index (0 or 1)
 *
 * Extracts the 4-bit MR1[7:4] clock-frequency index and MR3[5] write-latency
 * set selector from the 54-byte per-slice descriptor at DCCM 0x800009a4 and
 * computes the corresponding JEDEC WL Set A or Set B delay entry.
 *
 * Return: 8-bit CBT write latency offset for @rank and @channel.
 */
u32 pmu_cbt_delay_offset_lookup(u32 rank, u32 channel)
{
	u32 slice_idx = (rank << 1) + channel;
	u32 desc_addr = PMU_DMEM_CAL_STRUCT_ADDR +
			slice_idx * PMU_SLICE_DESC_SIZE;
	u32 idx = (u32)(dmem_read8(desc_addr + PMU_SLICE_DESC_MR1_OFF) >> 4);
	u32 wl;

	if (idx >= 12U)
		return 0U;

	if (dmem_read8(desc_addr + PMU_SLICE_DESC_MR3_OFF) & 0x20U)
		wl = idx + 2U + (idx >= 4U) + (idx >= 7U) + (idx >= 9U);
	else
		wl = ((idx * 11U + 2U) >> 4) + 2U;

	if (pmu_cbt_config_get() == 2U)
		return (idx < 6U) ? (wl << 1) : 0U;

	return wl;
}

/**
 * pmu_cbt_cal_pulse_seq() - Execute 6-step CBT calibration pulse sequence
 * @pulse_cmd: Pulse command flags for intermediate phases
 */
noinline void pmu_cbt_cal_pulse_seq(u32 pulse_cmd)
{
	u32 zero = 0U;

	pmu_cal_sequence_pulse_send(0x41U << 19, 7U, 4U, 0U, zero, zero, zero);
	pmu_cbt_cal_active_set();
	pmu_cal_sequence_pulse_send(pulse_cmd, 7U, 4U, 0U, zero, zero, zero);
	pmu_cbt_cal_active_clear();
	pmu_cal_sequence_pulse_send(pulse_cmd, 7U, 4U, 0U, zero, zero, zero);
	pmu_cal_sequence_pulse_send(0x80U | (1U << 19), 7U, 4U, 0U,
				    zero, zero, zero);
}

/**
 * pmu_cbt_timing_pulse_coordinator() - Coordinate 4-channel CBT timing pulse sweep
 * @cal_mode: Calibration mode halfword forwarded to pmu_cal_window_params_dispatch()
 * @phase_sel: Phase pattern selector (0 for primary masks, non-zero for alternate)
 */
void pmu_cbt_timing_pulse_coordinator(u16 cal_mode, u32 phase_sel)
{
	u32 zero = 0U;
	__volatile__ u16 *out_tbl =
		(__volatile__ u16 *)PMU_DMEM_DELAY_TAP_LUT;
	__volatile__ u16 *out_sub1 =
		(__volatile__ u16 *)PMU_DMEM_DELAY_TAP_SUB1;
	u32 mask_word;
	u32 phase;

	out_tbl[0] = 100U;

	for (phase = 0U; phase < 4U; phase++) {
		u32 pulse_mask;
		u16 step;
		u16 marker;

		if (phase_sel == 0U) {
			if (phase == 0U)
				pulse_mask = 0x7f00U;
			else if (phase == 1U)
				pulse_mask = 0x7f0000U;
			else if (phase == 2U)
				pulse_mask = 0x7f007f7fUL;
			else
				pulse_mask = 0x7f7f007fUL;
		} else {
			if (phase == 0U)
				pulse_mask = 0x7fU;
			else if (phase == 1U)
				pulse_mask = 0x7f000000UL;
			else if (phase == 2U)
				pulse_mask = 0x7f007f7fUL;
			else
				pulse_mask = 0x007f7f7fUL;
		}

		mask_word = pulse_mask;
		step = out_tbl[phase];
		pmu_cal_marker_pair_set((u16)(step << 2));

		pmu_cal_sequence_pulse_send(0x41UL << 19, 7U, 8U, 0U,
					    zero, zero, zero);
		pmu_cal_pad_timing_delay_set(0U, 4U, 1U);

		pmu_cal_window_params_dispatch((const u8 *)&mask_word, 2U, cal_mode);

		pmu_cal_pad_timing_delay_set(0U, 20U, 1U);
		pmu_cal_sequence_pulse_send(1UL << 18, 7U, 4U, 0U,
					    zero, zero, zero);
		pmu_cal_sequence_pulse_send((1UL << 19) | 0x80UL, 7U, 8U, 0U,
					    zero, zero, zero);

		marker = pmu_cal_stride_get();
		out_tbl[phase + 1U] = marker;
		out_sub1[phase] = (u16)(marker - 1U);
	}
}

/**
 * pmu_cbt_strobe_pulse_train() - Generate CBT strobe pulse train
 * @pulse_cmd: Slice / pulse command selector
 * @is_stage2: Non-zero for stage 2 short sequence, 0 for full repeat sequence
 */
void pmu_cbt_strobe_pulse_train(u32 pulse_cmd, u32 is_stage2)
{
	u32 zero = 0U;
	u8 pll_ratio = dmem_read8(PMU_DMEM_PLL_RATIO);
	u32 repeat_cnt;
	u32 i;

	pmu_cal_strobe_pulse_init();
	pmu_cal_sequence_pulse_send(0U, 7U, 4U, 0U, zero, zero, zero);
	pmu_cbt_cal_active_set();

	repeat_cnt = (pll_ratio == 2U) ? 4U : 2U;

	if (is_stage2 != 0U)
		pmu_cal_sequence_pulse_send(0U, 7U, 2U, 0U, zero, zero, 4U);

	pmu_cal_sequence_pulse_send(0x200000U, 5U, 0U, 256U, zero,
				    pulse_cmd, zero);

	if (is_stage2 == 0U) {
		pmu_cal_sequence_pulse_send(0x1000004U, 0x2bU, repeat_cnt, 0U,
					    zero, pulse_cmd, zero);
		pmu_cal_sequence_pulse_send(0x1000004U, 0x2bU, repeat_cnt, 0U,
					    zero, pulse_cmd, zero);
	}

	pmu_cal_sequence_pulse_send(0x1040000U, 0x2bU, repeat_cnt, 0U, zero,
				    pulse_cmd, zero);
	pmu_cal_sequence_pulse_send(0U, 7U, 10U, 0U, zero, zero, zero);

	for (i = 0U; i < 2U; i++) {
		pmu_cal_sequence_pulse_send(1U << (15U + i), 7U, 0U, 0U,
					    zero, zero, zero);
		pmu_cal_sequence_pulse_send(1U << (16U + i), 7U, 0U, 0U,
					    zero, zero, zero);
		if (i == 0U)
			pmu_cbt_cal_active_clear();
	}

	pmu_cal_sequence_pulse_send(0x80U | (1U << 19), 7U, 4U, 0U,
				    zero, zero, zero);
}

/**
 * pmu_cbt_deskew_pulse_train() - Generate CBT deskew pulse train
 * @pulse_cmd: Slice / pulse command selector
 * @is_alternate: Non-zero to add 1 to second repeat count
 */
void pmu_cbt_deskew_pulse_train(u32 pulse_cmd, u32 is_alternate)
{
	u32 zero = 0U;
	u8 pll_ratio = dmem_read8(PMU_DMEM_PLL_RATIO);
	u32 repeat_cnt;
	u32 second_cnt;

	pmu_cal_strobe_pulse_init();
	pmu_cal_sequence_pulse_send(0U, 7U, 0U, 0U, zero, zero, zero);
	pmu_cal_sequence_pulse_send(0x480000U, 5U, 0U, 512U, zero,
				    pulse_cmd, zero);
	pmu_cbt_cal_active_set();

	repeat_cnt = (pll_ratio == 2U) ? 5U : 2U;
	second_cnt = (is_alternate != 0U) ? (repeat_cnt + 1U) : repeat_cnt;

	pmu_cal_sequence_pulse_send(0x0cU, 0x2bU, repeat_cnt, 0U, zero,
				    pulse_cmd, zero);
	pmu_cal_sequence_pulse_send(0x0cU, 0x2bU, second_cnt, 0U, zero,
				    pulse_cmd, zero);
	pmu_cal_sequence_pulse_send(0x40000U, 0x2bU, repeat_cnt, 2U, zero,
				    pulse_cmd, zero);
	pmu_cal_sequence_pulse_send(0x0cU, 0x2bU, repeat_cnt, 2U, zero,
				    pulse_cmd, zero);

	pmu_cal_sequence_pulse_send(0U, 7U, 4U, 0U, zero, zero, 10U);
	pmu_cal_sequence_pulse_send(1U << 15, 7U, 4U, 0U, zero, zero, zero);
	pmu_cal_sequence_pulse_send(0U, 7U, 8U, 0U, zero, zero, zero);
	pmu_cbt_cal_active_clear();
	pmu_cal_sequence_pulse_send(0U, 7U, 4U, 0U, zero, zero, zero);

	pmu_cal_sequence_pulse_send(0U, 5U, 4U, 896U, zero, pulse_cmd, zero);
	pmu_cal_sequence_pulse_send((1U << 19) | 0x80U, 7U, 0U, 0U, zero,
				    pulse_cmd, zero);
}

/**
 * pmu_cbt_3phase_pulse_dispatch() - Dispatch 3-phase CBT calibration pulse sequence
 * @cal_mode: Target calibration mode / rank selector
 * @step_table: Base pointer to calibration step table
 * @step_idx: Step index within @step_table
 * @flags: Control bitmask (bit 0 skips profile program, bit 1 skips AC setup)
 */
void pmu_cbt_3phase_pulse_dispatch(u32 cal_mode, const u8 *step_table,
				   u32 step_idx, u32 flags)
{
	u8 val = step_table[step_idx];
	u32 is_final_step = (step_idx == 16U) ? 1U : 0U;
	u32 p2 = ((step_idx - 14U) < 2U) ? 0x20U : (is_final_step ? 4U : 6U);
	u32 is_final_ovr = 0U;

	pmu_cal_strobe_pulse_init();
	if ((flags & 2U) == 0U) {
		pmu_ac_lane_profile_setup();
		pmu_cal_pad_timing_delay_set(0U, 5U, 1U);
	}

	pmu_dbyte_cal_step_latch(step_idx, val, cal_mode);
	pmu_cal_sequence_pulse_send(0U, 7U, p2, 0U, 0U, is_final_step, 0U);

	if (is_final_step != 0U && (val & 0x20U) != 0U) {
		u32 arg = (dmem_read8(PMU_DMEM_CBT_LANE_LIM) == 0U) ?
			  (1U << 19) : 0U;

		pmu_cbt_cal_pulse_seq(arg);
		is_final_ovr = 1U;
	}

	pmu_cbt_3phase_pulse_seq();
	pmu_cal_slice_step_diff_latch(0U, is_final_ovr ^ 1U);
	if (is_final_ovr != 0U)
		pmu_delay_us(0x1388U, 0x14U);
	if ((flags & 1U) == 0U)
		pmu_phy_profile_param_program();
}

