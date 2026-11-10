// SPDX-License-Identifier: GPL-2.0+
/*
 * Allwinner A733 Cadence DDR PHY PMU Slice Deskew & Lane Status Helpers
 */

#include "pmu_fw.h"

static void pmu_cal_channel_rank_boundary_eval(u8 *buf, u32 channel, u32 rank, u32 cal_type);
static void pmu_cal_dbyte_deskew_pin_results_apply(void);
static void pmu_cal_dbyte_deskew_results_apply(void *slice_base, u32 param1, u32 opcode);
static u32 pmu_cal_dbyte_dq_status_check(u32 channel_idx, u32 mode_rank, u32 flag);
static void pmu_cal_dly_line_tap_step_adjust(u32 mode, u32 ptr_tag, u32 lane_mask, u32 check_tag);
static u32 pmu_cal_lane_deskew_step_advance(u32 lane, u32 phase);
static void pmu_cal_multi_rank_deskew_sweep(u32 channel, u32 mode);
static u32 pmu_cal_slice_step_scan_eval(u32 step_val, u32 slice_mask);
static u8 pmu_dbyte_deskew_sample_check(u32 channel, u32 mode_rank, u32 marker0, u32 marker1);
static void pmu_dbyte_slice_cal_param_pulse(u32 rank, u32 channel, u32 slice_mask);
static int pmu_odd_byte_win_span_check(u8 *out_min, u8 *out_max, const u8 *scan_buf);
static void pmu_rank_slice_deskew_latch(u32 step_delay, u32 lane_mask);
static void pmu_slice_deskew_pulse_train(u32 ch_idx, u32 rank, u32 mode);
static void pmu_slice_deskew_state_latch(u32 step_delay, u32 lane_mask);
static void pmu_slice_dly_tap_coeff_prog(u32 rank);
static void pmu_slice_phy_reg_step_adjust(u32 reg_offset, u32 polarity_cond, u32 step_mode);

/**
 * pmu_slice_status_b97_save() - Latch active slice status into DCCM 0x80000b97
 *
 * Reads the active channel CSR offset from DCCM 0x8000041c, samples the 16-bit
 * PHY slice status register at 0x90040018 + (offset << 1), and stores the low
 * byte to DCCM 0x80000b97.
 */
void pmu_slice_status_b97_save(void)
{
	u32 off = pmu_sdata()->active_csr_offset;
	u16 val = *(__volatile__ u16 *)(PHY_REG_SLICE_STAT_0018 + (off << 1));

	*(__volatile__ u8 *)PMU_DMEM_LANE_STATUS_B97 = (u8)val;
}

/**
 * pmu_cal_lane_error_status_check() - Evaluate per-lane deskew error mask bit
 * @lane: Logical lane/rank index (0..3)
 * @phase: Phase polarity selector
 *
 * Return: 1-bit error status from DCCM 0x80000101 and direction 0x80000b6b.
 */
noinline u32 pmu_cal_lane_error_status_check(u32 lane, u32 phase)
{
	u32 dir = dmem_read8(PMU_DMEM_DESKEW_DIR_B6B);
	u32 mask = dmem_read8(PMU_DMEM_LANE_ERR_MASK_101);
	u32 bit = lane + (((u8)phase > dir) ? 2U : 0U);

	return (((mask >> bit) ^ phase) ^ 1U) & 1U;
}

/**
 * pmu_cal_lane_deskew_step_advance() - Check if lane deskew step should advance
 * @lane: Logical lane/rank index (0..3)
 * @phase: Current step phase threshold
 *
 * Return: 1 if lane error status or DCCM 0x1c limit comparison requires step
 *         advance, 0 otherwise.
 */
static noinline u32 pmu_cal_lane_deskew_step_advance(u32 lane, u32 phase)
{
	u32 err = pmu_cal_lane_error_status_check(lane, 1U);

	if (err != 0U)
		return err;
	err = pmu_cal_lane_error_status_check(lane, phase);
	if (err != 0U)
		return err;

	return pmu_get_dmem_1c_bit(*(const u8 *)PMU_DMEM_DESKEW_DIR_B6B, lane) <
	       (phase & 0xffU);
}

/**
 * pmu_phy_dbyte_cfg_unpack() - Restore DBYTE delay registers from DCCM 0x80000ba0
 *
 * Unpacks 8 bytes per active DBYTE slice (count in DCCM 0x8000042c) from the
 * backup array at DCCM 0x80000ba0 into PHY registers 0x90020100 + (slice << 13).
 */
noinline void pmu_phy_dbyte_cfg_unpack(void)
{
	u32 count = pmu_sdata()->dbyte_slice_cnt;
	u32 s;
	u32 i;

	for (s = 0U; s < count; s++) {
		__volatile__ u16 *reg =
			(__volatile__ u16 *)(PHY_REG_DBYTE_DELAY_100 + (s << 13));
		const __volatile__ u8 *src =
			(const __volatile__ u8 *)(PMU_DMEM_DBYTE_BACKUP_BA0 + (s << 3));

		for (i = 0U; i < 8U; i++)
			reg[i] = src[i];
	}
}

/**
 * pmu_cal_pad_impedance_lock() - Save DBYTE DXDMAP registers and set linear ramp
 *
 * Reads 8 halfword registers per active DBYTE slice (count in DCCM 0x8000042c)
 * from PHY 0x90020100 + (slice << 13), packs their low bytes into DCCM
 * 0x80000ba0, and initializes each register to its linear pin index 0..7.
 */
noinline void pmu_cal_pad_impedance_lock(void)
{
	u32 count = pmu_sdata()->dbyte_slice_cnt;
	u32 s;
	u32 i;

	for (s = 0U; s < count; s++) {
		__volatile__ u16 *reg =
			(__volatile__ u16 *)(PHY_REG_DBYTE_DELAY_100 + (s << 13));
		__volatile__ u8 *dst =
			(__volatile__ u8 *)(PMU_DMEM_DBYTE_BACKUP_BA0 + (s << 3));

		for (i = 0U; i < 8U; i++) {
			dst[i] = (u8)reg[i];
			reg[i] = (u16)i;
		}
	}
}

/**
 * pmu_cal_rx_dqs_delay_center() - Populate per-channel DQ pin map matrix at 0x80000a7e
 *
 * Samples 8 lane delay registers per active DBYTE slice from
 * 0x90020100 + (ch << 13) + (lane << 1) and records matched lanes in entry[0]
 * and mismatched (1 << tap, lane) pairs in entry[2..17].
 */
void pmu_cal_rx_dqs_delay_center(void)
{
	u32 max_ch = pmu_sdata()->dbyte_slice_cnt;
	u32 ch;
	u32 lane;

	for (ch = 0U; ch < max_ch; ch++) {
		__volatile__ u8 *entry =
			(__volatile__ u8 *)(PMU_DMEM_DQ_PIN_MAP_BASE + (ch * 18U));
		const __volatile__ u16 *reg =
			(const __volatile__ u16 *)(PHY_REG_DBYTE_DELAY_100 + (ch << 13));

		*(__volatile__ u16 *)entry = 0U;
		for (lane = 0U; lane < 8U; lane++) {
			u32 tap = (u8)reg[lane];

			if (tap == lane) {
				entry[0] |= (u8)(1U << lane);
			} else {
				u32 count = (u32)entry[1] + 1U;

				entry[1] = (u8)count;
				entry[count * 2U] = (u8)(1U << tap);
				entry[count * 2U + 1U] = (u8)lane;
			}
		}
	}
}

/**
 * pmu_cal_deskew_quad_tap_set() - Write 4 deskew delay tap pairs and latch channel mask
 * @ch: Channel/slice index (0x0f for broadcast)
 * @tap0: First deskew tap value
 * @tap1: Second deskew tap value
 * @tap2: Third deskew tap value
 * @tap3: Fourth deskew tap value
 */
noinline void pmu_cal_deskew_quad_tap_set(u32 ch, u16 tap0, u16 tap1,
					  u16 tap2, u16 tap3)
{
	__volatile__ u16 *reg =
		(__volatile__ u16 *)(PHY_REG_DESKEW_QUAD_TAP | ((ch & 0xffU) << 9));

	reg[0] = tap0;
	reg[1] = tap1;
	reg[2] = tap0;
	reg[3] = tap1;
	reg[4] = tap2;
	reg[5] = tap3;
	reg[6] = tap2;
	reg[7] = tap3;

	if ((ch & 0xffU) == 0x0fU)
		*(__volatile__ u16 *)PHY_REG_DESKEW_QUAD_MASK_0CA = 0x01ffU;
	else
		*(__volatile__ u16 *)PHY_REG_DESKEW_QUAD_MASK_0CA |=
			(u16)(1U << (ch & 0xffU));
}

/**
 * pmu_phy_reg_buffer_stream() - Stream 6-byte [addr, val] tuples between buffer and PHY APB
 * @buf: Pointer to array of 6-byte [u32 addr, u16 val] tuples
 * @count: Number of 6-byte tuples to transfer
 * @is_save: Non-zero to save PHY APB registers into @buf, 0 to restore @buf to PHY APB
 */
void pmu_phy_reg_buffer_stream(void *buf, u32 count, u32 is_save)
{
	u8 *ptr = (u8 *)buf;
	u32 i;

	if (is_save != 0U) {
		for (i = 0U; i < count; i++) {
			u32 addr = (u32)ptr[0] | ((u32)ptr[1] << 8) |
				   ((u32)ptr[2] << 16) | ((u32)ptr[3] << 24);
			u16 val = *(__volatile__ const u16 *)(PHY_REG_APB_BASE |
							      (addr << 1));

			ptr[4] = (u8)val;
			ptr[5] = (u8)(val >> 8);
			ptr += 6U;
		}
	} else {
		for (i = 0U; i < count; i++) {
			u32 addr = (u32)ptr[0] | ((u32)ptr[1] << 8) |
				   ((u32)ptr[2] << 16) | ((u32)ptr[3] << 24);
			u16 val = (u16)((u16)ptr[4] | ((u16)ptr[5] << 8));

			*(__volatile__ u16 *)(PHY_REG_STREAM_ALT_BASE |
					      (addr << 1)) = val;
			ptr += 6U;
		}
	}
}

/**
 * pmu_sample_margin_center_eval() - Evaluate gate eye bounds and midpoint sample pairs
 * @samples: Input 128-byte scan sample buffer
 * @out_margins: Output 6-byte midpoint and sample-average array
 * @out_mode: Output mode selector derived from DCCM 0x8000000a bits [7:6]
 *
 * Return: 0 on success, or non-zero error status from pmu_odd_byte_win_span_check().
 */
int pmu_sample_margin_center_eval(const u8 *samples, u8 *out_margins,
				  u8 *out_mode)
{
	u16 cfg = *(__volatile__ const u16 *)(PMU_DMEM_BASE +
					      PMU_DMEM_DRAM_CFG_FLAGS);
	u32 mode_field = (cfg >> 6) & 3U;
	u8 start, end, center;
	u32 s, idx;
	int ret;

	*out_mode = (mode_field == 1U) ? 1U : ((mode_field == 3U) ? 3U : 2U);

	ret = pmu_odd_byte_win_span_check(&end, &start, samples);
	if (ret != 0)
		return ret;

	s = (u32)end + (u32)start;
	center = (u8)(s >> 1);
	out_margins[0] = center;
	idx = s & ~1U;
	out_margins[1] = (u8)(((u32)samples[idx] + (u32)samples[idx + 1U]) >> 1);

	s = (u32)end + (u32)center;
	out_margins[2] = (u8)(s >> 1);
	idx = s & ~1U;
	out_margins[3] = (u8)(((u32)samples[idx] + (u32)samples[idx + 1U]) >> 1);

	s = (u32)start + (u32)center;
	out_margins[4] = (u8)(s >> 1);
	idx = s & ~1U;
	out_margins[5] = (u8)(((u32)samples[idx] + (u32)samples[idx + 1U]) >> 1);

	return 0;
}

/**
 * pmu_odd_byte_win_span_check() - Find odd-byte non-zero window and verify >= 12 span
 * @out_min: Output pointer receiving first active index (0..64)
 * @out_max: Output pointer receiving last active index
 * @scan_buf: Input 128-byte scan buffer (odd indices inspected)
 *
 * Return: 0 if (*out_max - *out_min) >= 12, or 6 on narrow window error.
 */
static int pmu_odd_byte_win_span_check(u8 *out_min, u8 *out_max, const u8 *scan_buf)
{
	u32 min_idx = 0U;
	s32 max_idx = 63;
	s32 span;

	while (min_idx < 64U) {
		if (scan_buf[min_idx * 2U + 1U] != 0U)
			break;
		min_idx++;
	}
	*out_min = (u8)min_idx;

	while (max_idx >= 0) {
		if (scan_buf[(u32)max_idx * 2U + 1U] != 0U)
			break;
		max_idx--;
	}
	*out_max = (u8)max_idx;

	span = (s32)(*out_max) - (s32)(*out_min);
	if (span >= 12)
		return 0;

	return 6;
}

/**
 * pmu_slice_lcdl_delay_collect() - Collect PHY slice status halfwords or mapped lane error status
 * @param: Register offset / control word (bit 24 selects lane error status mode)
 * @out_buf: Output halfword buffer
 * @count: Number of halfwords per slice when bit 24 of @param is clear
 */
void pmu_slice_lcdl_delay_collect(u32 param, void *out_buf, u32 count)
{
	const u8 *b68 = (const u8 *)PMU_DMEM_CH_RANGE_B68;
	u32 start_slice;
	u32 end_slice;
	u16 *out;
	u32 slice;

	start_slice = b68[0];
	end_slice = b68[1];
	out = (u16 *)out_buf + count * start_slice;

	for (slice = start_slice; slice <= end_slice; slice++) {
		if ((param & (1U << 24)) == 0U) {
			u32 i;

			for (i = 0U; i < count; i++) {
				u32 reg = PHY_REG_DBYTE_SLICE_BASE |
					  ((((slice << 12) | param |
					     (i << 8)) << 1));

				*out++ = *(__volatile__ const u16 *)reg;
			}
		} else {
			u32 r0 = (param & ~pmu_sdata()->active_csr_offset) &
				 0x0fffffffeUL;
			u32 status_code;
			u32 is_lo;
			u32 tbl_off;

			r0 += 0xfefffff0UL;
			r0 = (r0 >> 4) | (r0 << 28);
			if (r0 == 1U) {
				u8 err = pmu_cal_lane_error_status_check((u8)param,
									     (u8)slice);

				status_code = err ? 1U : 2U;
			} else {
				status_code = (r0 > 3U) ? 4U : r0;
			}

			is_lo = (b68[3] < slice) ? 1U : 0U;
			tbl_off = (is_lo << 1) + status_code * 4U +
				  (param & 1U);
			*out++ = b68[6U + tbl_off];
		}
	}
}

/**
 * pmu_slice_deskew_pulse_seq() - Average and program slice deskew delay taps across ranks
 * @enable: Execution enable flag (returns immediately if != 1)
 */
void pmu_slice_deskew_pulse_seq(u32 enable)
{
	u32 rank;

	if (enable != 1U)
		return;

	for (rank = 1U; rank < 3U; rank++) {
		u8 cfg = (rank == 2U) ?
			 *(__volatile__ const u8 *)PMU_DMEM_CH1_ACTIVE_FLAG :
			 *(__volatile__ const u8 *)PMU_DMEM_CH0_ANALOG_EN;
		const u8 *b68 = (const u8 *)PMU_DMEM_CH_RANGE_B68;
		u8 start_slice, end_slice;
		u16 sp_buf[8];
		u32 lane;

		if (cfg != 3U)
			continue;

		pmu_phy_mode_cfg_dispatch(rank);
		pmu_pulse_phy_f2();

		start_slice = b68[0];
		end_slice = b68[1];

		for (lane = 0U; lane < 2U; lane++) {
			u32 offset = ((0xe4U >> (lane * 2U)) & 3U) + 0x2aU;
			u32 slice;

			for (slice = start_slice; slice <= end_slice; slice++) {
				u32 slice_offset = slice << 12;
				u32 csr_offset = pmu_sdata()->active_csr_offset;
				u32 reg = offset | slice_offset | csr_offset;
				u16 val = *(__volatile__ const u16 *)
					(PHY_REG_DBYTE_BASE + (reg << 1));

				sp_buf[lane * 4U + slice] = val;

				if (lane == 1U) {
					u16 v0 = sp_buf[slice];
					u32 sum = (u32)v0 + val;
					u32 q = sum >> 7;
					u32 rem = (sum >> 1) & 0x3fU;
					u16 prog_val = (u16)(rem | (q << 6));

					*(__volatile__ u16 *)
						(PHY_REG_DBYTE_ANALOG_054 +
						 ((slice_offset |
						   csr_offset) << 1)) =
						prog_val;
					*(__volatile__ u16 *)
						(PHY_REG_DBYTE_BASE +
						 (reg << 1)) = prog_val;
				}
			}
		}
	}

	pmu_phy_mode_cfg_dispatch(3U);
}

/**
 * pmu_cal_delay_line_init() - Compute clamped CBT delay line initial offset
 */
void pmu_cal_delay_line_init(void)
{
	u32 sum = (u32)pmu_freq_scaled_timing_step_calc() +
		  (u32)pmu_cbt_delay_offset_lookup(0U, 0U) +
		  (u32)pmu_cal_cbt_desc_unpack(0U, 0U) + 1U;

	sum = (u8)sum;
	sum &= ~1U;
	if (sum > 92U)
		sum = 92U;

	pmu_sdata()->cbt_dly_init = (u8)sum;
}

/**
 * pmu_slice_phy_reg_step_adjust() - Adjust DBYTE slice delay register step across range
 * @reg_offset: Relative register offset added to 0x2a
 * @polarity_cond: Non-zero to add +0x20 when @step_mode == 0, 0 to subtract 0x20
 * @step_mode: Non-zero to modulo-wrap by (dmem[0x08] << 6), 0 for +/- 0x20 step
 */
static void pmu_slice_phy_reg_step_adjust(u32 reg_offset, u32 polarity_cond,
					  u32 step_mode)
{
	const u8 *b68 = (const u8 *)PMU_DMEM_CH_RANGE_B68;
	u32 slice, end_slice;
	u32 base = reg_offset + 0x2aU;

	slice = b68[0];
	end_slice = b68[1];

	for (; slice <= end_slice; slice++) {
		u32 reg_addr = PHY_REG_DBYTE_BASE |
			       (((pmu_sdata()->active_csr_offset |
				  (slice << 12) | base)) << 1);
		u16 val = *(__volatile__ const u16 *)reg_addr;
		u32 adjusted_val;

		if (step_mode != 0U) {
			u32 denom = (u32)(*(__volatile__ const u8 *)
					  PMU_DMEM_FREQ_ROUND_BIAS) << 6;

			adjusted_val = (denom != 0U) ? ((u32)val % denom) : 0U;
			adjusted_val &= 0x07ffU;
		} else {
			if (polarity_cond != 0U)
				adjusted_val = (u32)val + 0x20U;
			else
				adjusted_val = (val < 0x20U) ?
					       0U : ((u32)val - 0x20U);
			adjusted_val &= 0x07ffU;
		}

		*(__volatile__ u16 *)reg_addr = (u16)adjusted_val;
	}

	pmu_pulse_phy_f2();
}

/**
 * pmu_rate_tier_calc_inline() - Encode DDR clock frequency into high/low rate tier nibble pair
 * @val: Scaled frequency metric value
 *
 * Return: Packed 8-bit tier code ((high << 4) | low).
 */
static __always_inline u32 pmu_rate_tier_calc_inline(u32 val)
{
	u32 low = (val <= 331U) ? 0U :
		  (val <= 531U) ? 1U :
		  (val <= 799U) ? 2U : 3U;
	u32 high = (val <= 799U) ? (low + 2U) :
		   (val <= 1799U) ? 5U :
		   (val <= 2199U) ? 6U : 7U;

	return (high << 4) | low;
}

/**
 * pmu_pll_lock_freq_tier_cfg() - Check PLL lock and program frequency rate tiers
 */
void pmu_pll_lock_freq_tier_cfg(void)
{
	u32 ch_offset = pmu_sdata()->active_csr_offset << 1;
	u16 lock_val_hi, lock_val_lo;
	__volatile__ u16 *phy_ch;
	u32 dram_type;
	u32 freq;
	u32 q;
	u32 clock_ratio;
	u32 rate_calc;
	u32 tier_clock;
	u32 tier_rate;
	u32 slice_count;
	u32 s;

	pmu_cal_pll_lock_settle_spin(&lock_val_hi, &lock_val_lo);

	phy_ch = (__volatile__ u16 *)(PHY_REG_MASTER_BASE | ch_offset);
	phy_ch[0x14 >> 1] = lock_val_lo & 0x01ffU;
	phy_ch[0x2e >> 1] = lock_val_hi & 0x01ffU;

	dram_type = *(const __volatile__ u8 *)PMU_DMEM_FREQ_ROUND_BIAS;
	if (dram_type == 0U)
		dram_type = 2U;
	freq = *(const __volatile__ u16 *)PMU_DMEM_DRAM_FREQ_MHZ;

	q = freq / dram_type;
	clock_ratio = (dram_type == 2U) ? (q << 1) : q;
	rate_calc = (dram_type != 2U) ? (q << 2) : (q << 1);

	tier_clock = pmu_rate_tier_calc_inline(clock_ratio);
	tier_rate = pmu_rate_tier_calc_inline(rate_calc);

	phy_ch[0x24 >> 1] = (u16)(((tier_rate >> 4) << 12) |
				  ((tier_rate & 0x0fU) << 8) | tier_clock);
	phy_ch[0x30c >> 1] = (u16)tier_rate;
	phy_ch[0x30e >> 1] = *(const __volatile__ u16 *)PHY_REG_DBYTE_PERIOD_1AA;

	slice_count = pmu_sdata()->active_slice_cnt;
	for (s = 0U; s < slice_count; s++)
		*(__volatile__ u16 *)(PHY_REG_DBYTE_PLL_TIER_1B2 +
				      (s * PHY_DBYTE_SLICE_STRIDE)) = 0x009cU;
}

/**
 * pmu_dbyte_deskew_sample_check() - Check DBYTE deskew sample status across slices
 * @channel: Target channel index (0 or 1)
 * @mode_rank: Rank and mode selector
 * @marker0: First 16-bit calibration marker
 * @marker1: Second 16-bit calibration marker
 *
 * Return: 1 if any slice sample mask fails expected low/high checks, else 0.
 */
static u8 pmu_dbyte_deskew_sample_check(u32 channel, u32 mode_rank,
					u32 marker0, u32 marker1)
{
	const __volatile__ u8 *b20 =
		(const __volatile__ u8 *)PMU_DMEM_SLICE_CNT_LIMIT_B20;
	u32 rank = ((mode_rank & 0x3U) == 0x2U) ? 1U : 0U;
	const u8 *byte_map;
	u8 num_slices;
	u8 res = 0U;
	u32 i;

	byte_map = (const u8 *)(b20 + 0x24U + (channel << 4) + (rank << 3));
	num_slices = b20[(channel << 1) + rank];

	pmu_cal_marker_win_step_prog(marker0, marker1, mode_rank, 0U);
	pmu_delay_us(10000U, 0U);

	for (i = 0U; i < num_slices; i++) {
		u8 slice = byte_map[i];
		u16 phy_val = *(__volatile__ const u16 *)
			(PHY_REG_DBYTE_GATE_STAT_11A + ((u32)slice << 13));
		u8 mask = pmu_map_mask(phy_val, slice);

		if ((mask & 0x7fU) != 0U)
			res |= 1U;
	}

	pmu_cal_marker_win_step_prog(marker0, marker1, mode_rank, 0x7fU);

	for (i = 0U; i < num_slices; i++) {
		u8 slice = byte_map[i];
		u16 phy_val = *(__volatile__ const u16 *)
			(PHY_REG_DBYTE_GATE_STAT_11A + ((u32)slice << 13));
		u8 mask = pmu_map_mask(phy_val, slice) & 0x7fU;

		if (mask != 0x7fU)
			res |= 1U;
	}

	return res;
}

/**
 * pmu_cal_dbyte_deskew_results_apply() - Apply calibrated DBYTE deskew results to CSRs
 * @slice_base: Pointer to deskew results buffer
 * @param1: Format selector shifted through 0xe4 lookup
 * @opcode: Calibration stage identifier (0x11 for full quad-tap mode)
 */
static void pmu_cal_dbyte_deskew_results_apply(void *slice_base, u32 param1,
					       u32 opcode)
{
	u32 bit_val = pmu_bit_lookup_e4(param1, 0U);
	u8 slice_limit;
	u32 slice, lane;

	if (opcode == 0x11U) {
		u8 op102 = *(const __volatile__ u8 *)PMU_DMEM_EYE_SWEEP_OP17_102;
		u8 base_flag = ((const u8 *)slice_base)[512];
		u32 flag_bit = (op102 & 2U) >> 1;

		if (flag_bit != 0U && base_flag == 3U) {
			u32 ch_off = pmu_sdata()->active_csr_offset << 1;

			*(__volatile__ u16 *)(PHY_REG_DBYTE_BCAST_MODE_012 |
					      ch_off) = 4U;
		}

		slice_limit = dmem_read8(PMU_DMEM_SLICE_END);
		for (slice = 0U; slice <= slice_limit; slice++) {
			u16 *p_slice = (u16 *)((uintptr_t)slice_base +
					       slice * 20U);

			for (lane = 0U; lane < 9U; lane++) {
				u16 dly_fine = p_slice[lane + 16U];
				u16 dly_coarse = p_slice[lane + 56U];
				u32 csr_addr;

				csr_addr = PHY_REG_DBYTE_BASE |
					   (((lane << 8) | (slice << 12) |
					     bit_val |
					     pmu_sdata()->active_csr_offset) << 1);
				*(__volatile__ u16 *)(csr_addr + 0x24U) =
					dly_fine;
				*(__volatile__ u16 *)(csr_addr + 0x20U) =
					dly_coarse;

				if (flag_bit != 0U && base_flag == 3U) {
					u16 *p_quad = (u16 *)
						((uintptr_t)slice_base +
						 slice * 80U + lane * 8U +
						 192U);
					__volatile__ u16 *quad_csr =
						(__volatile__ u16 *)
						(PHY_REG_DBYTE_DELAY_09C |
						 (slice << 13));

					quad_csr[0] = p_quad[0];
					quad_csr[1] = p_quad[1];
					quad_csr[2] = p_quad[2];
					quad_csr[3] = p_quad[3];
				}
			}
		}

		pmu_cal_pad_impedance_lock();
		return;
	}

	slice_limit = dmem_read8(PMU_DMEM_SLICE_END);

	for (slice = 0U; slice <= slice_limit; slice++) {
		u16 *p_slice = (u16 *)((uintptr_t)slice_base + slice * 20U);
		u16 val_slice_dly;
		u32 slice_csr;

		for (lane = 0U; lane < 9U; lane++) {
			u16 val_dly = p_slice[lane + 16U];
			u32 csr_addr;

			csr_addr = PHY_REG_DBYTE_BASE |
				   (((lane << 8) | (slice << 12) |
				     bit_val |
				     pmu_sdata()->active_csr_offset) << 1);
			*(__volatile__ u16 *)(csr_addr + 0x26U) = val_dly;
		}

		val_slice_dly = p_slice[25U];
		slice_csr = PHY_REG_DBYTE_BASE |
			    (((slice << 12) | bit_val |
			      pmu_sdata()->active_csr_offset) << 1);
		*(__volatile__ u16 *)(slice_csr + 0x28U) = val_slice_dly;
	}
}

static __always_inline u16
pmu_phase_edge_stream_slot(u32 slot, u8 count, u32 max_val, u32 limit_50,
			   const u16 *p_reg, u32 dmem_val, u16 shadow_idx)
{
	u32 i, j, k;

	for (i = 0U; i < count; i++) {
		for (j = 0U; j < limit_50; j++) {
			u32 slice_apb_offset = j << 12;

			for (k = max_val; k > 0U; k--) {
				u16 reg_off = p_reg[i];
				u32 full_addr = (slice_apb_offset | dmem_val |
						 reg_off) << 1;
				u16 v1 = *(const __volatile__ u16 *)
					(uintptr_t)(PHY_REG_APB_BASE | full_addr);
				u16 v2 = 0U;
				u32 type = 1U;

				if (slot < 2U) {
					if ((dmem_val & 1U) != 0U) {
						v2 = v1;
						v1 = 0U;
						type = 2U;
					}
				} else {
					v2 = *(const __volatile__ u16 *)
						(uintptr_t)(PHY_REG_APB_BASE |
							    (full_addr + 2U));
					type = 3U;
				}

				if (shadow_idx != 0U ||
				    *(const __volatile__ u16 *)PMU_DMEM_PLL_GATE_E8 != 0U ||
				    *(const __volatile__ u16 *)PMU_DMEM_PLL_GATE_EA != 0U)
					pmu_phy_pll_cfg_quad_write_stream((u16)(shadow_idx << 2),
									  v1, v2,
									  full_addr >> 1,
									  type);
				shadow_idx++;
			}
		}
	}

	return shadow_idx;
}

/**
 * pmu_cal_phase_detector_edge_align() - Align phase detector edges across channels
 */
void pmu_cal_phase_detector_edge_align(void)
{
	u32 tbl_base = PMU_DMEM_CAL_SEQ_MAP_480;
	const void *phase_tbl_a = (const void *)(tbl_base + 0xb0U);
	const void *phase_tbl_b = (const void *)(tbl_base + 0x16aU);
	const void *phase_tbl_c = (const void *)(tbl_base + 0x1baU);
	const void *phase_tbl_d = (const void *)(tbl_base + 0x1beU);
	const void *phase_tbl_e = (const void *)(tbl_base + 0x3baU);
	u8 channel_detect_mask[4];
	const void *tbl_ptrs[4];
	u16 param_block[8];
	u16 param_block_1b[8];
	u16 shadow_idx = (u16)(pmu_sdata()->shadow_trace_ptr >> 1);
	u16 shadow_base;
	u32 final_shadow;
	u32 mode_idx, slot, i, ch, rank;

	*(__volatile__ u16 *)PMU_DMEM_PLL_GATE_F2 = shadow_idx;

	for (mode_idx = 0U; mode_idx < 3U; mode_idx++) {
		u32 limit_50;
		u32 mode_param;

		if (mode_idx == 0U) {
			channel_detect_mask[0] = 91U;
			channel_detect_mask[1] = 35U;
			channel_detect_mask[2] = 2U;
			channel_detect_mask[3] = 0U;
			mode_param = (u8)pmu_sdata()->active_ch_cnt;
			tbl_ptrs[0] = phase_tbl_a;
			tbl_ptrs[1] = phase_tbl_b;
			tbl_ptrs[2] = phase_tbl_c;
			tbl_ptrs[3] = NULL;
			limit_50 = pmu_sdata()->active_slice_cnt;
		} else if (mode_idx == 1U) {
			channel_detect_mask[0] = 171U;
			channel_detect_mask[1] = 20U;
			channel_detect_mask[2] = 0U;
			channel_detect_mask[3] = 0U;
			mode_param = 1U;
			tbl_ptrs[0] = phase_tbl_d;
			tbl_ptrs[1] = phase_tbl_e - 0x28U;
			tbl_ptrs[2] = NULL;
			tbl_ptrs[3] = NULL;
			limit_50 = (u8)(pmu_sdata()->active_ch_cnt >> 16);
		} else {
			channel_detect_mask[0] = 63U;
			channel_detect_mask[1] = 0U;
			channel_detect_mask[2] = 0U;
			channel_detect_mask[3] = 0U;
			mode_param = 1U;
			tbl_ptrs[0] = phase_tbl_e;
			tbl_ptrs[1] = NULL;
			tbl_ptrs[2] = NULL;
			tbl_ptrs[3] = NULL;
			limit_50 = 1U;
		}

		for (slot = 0U; slot < 4U; slot++) {
			u32 repeat_cnt = ((slot | 2U) == 3U) ? mode_param : 1U;
			u32 max_val = (repeat_cnt > 1U) ? repeat_cnt : 1U;
			u32 dmem_val = *(const __volatile__ u32 *)
				(tbl_base + mode_idx * 4U + 0x18U);

			shadow_idx = pmu_phase_edge_stream_slot(slot,
								channel_detect_mask[slot],
								max_val, limit_50,
								(const u16 *)tbl_ptrs[slot],
								dmem_val,
								shadow_idx);
		}
	}

	*(__volatile__ u16 *)PHY_REG_SHADOW_CNT_21E =
		(u16)(shadow_idx - *(const __volatile__ u16 *)PMU_DMEM_PLL_GATE_F2);
	shadow_base = (u16)(pmu_sdata()->shadow_trace_ptr >> 1);
	*(__volatile__ u16 *)PMU_DMEM_SHADOW_BASE_10C = shadow_base;
	*(__volatile__ u16 *)PMU_DMEM_SHADOW_DIFF_10A =
		(u16)(shadow_base - *(const __volatile__ u16 *)PMU_DMEM_SHADOW_BASE_108);

	for (i = 0U; i < 8U; i++)
		param_block_1b[i] = 0U;
	param_block_1b[3] = 0x1b00U;

	for (ch = 0U; ch < 2U; ch++) {
		u8 active = (ch != 0U) ?
			    *(const __volatile__ u8 *)PMU_DMEM_CH1_ANALOG_EN_40 :
			    *(const __volatile__ u8 *)PMU_DMEM_CH0_ANALOG_EN_25;
		u16 pll_cfg;
		u32 ch_shift;
		u8 limit_entry;

		if (active == 0U)
			continue;

		pll_cfg = (ch != 0U) ?
			  *(const __volatile__ u16 *)PMU_DMEM_PLL_GATE_EA :
			  *(const __volatile__ u16 *)PMU_DMEM_PLL_GATE_E8;
		ch_shift = ch << 1;
		limit_entry = pmu_sdata()->metric_tbl_entries;

		for (rank = 0U; rank < limit_entry; rank++) {
			u8 param_b = pmu_sdata()->cal_cmd_tbl[rank];
			u8 param_c =
				*(const __volatile__ u8 *)
				(PMU_DMEM_DESKEW_STAT_TBL_B6E + ch_shift + rank);
			u32 eval_mask = 0U;

			if (param_b == 12U)
				param_c |= 0x80U;

			if (((*(const __volatile__ u8 *)PMU_DMEM_DESKEW_LIM_1C >>
			      ch_shift) & 1U) != 0U)
				eval_mask = (param_b != 12U) ? 1U : 0U;

			pmu_cal_eye_param_block_init(param_block, param_b,
						     param_c, eval_mask);
			pmu_pll_quad_param_tuple_stream(param_block, &shadow_idx,
							&pll_cfg);

			if (rank < (u32)(limit_entry - 1U))
				pmu_pll_quad_param_tuple_stream(param_block_1b,
								&shadow_idx,
							    &pll_cfg);
		}
	}

	final_shadow = pmu_sdata()->shadow_trace_ptr >> 1;
	*(__volatile__ u16 *)PMU_DMEM_SHADOW_DIFF_10E =
		(u16)(final_shadow - *(const __volatile__ u16 *)PMU_DMEM_SHADOW_BASE_10C);
}

/**
 * pmu_dbyte_lane_error_mask_calc() - Trigger lane error mask evaluation pulse sequence
 * @mode: Non-zero for two-step 0x25/0x26 pulse sequence, 0 for single 0x19 pulse
 * @slice_or_rank: Target slice or rank bitmask
 */
noinline void pmu_dbyte_lane_error_mask_calc(u32 mode, u32 slice_or_rank)
{
	u32 zero = 0U;

	pmu_cal_strobe_pulse_init();
	if (mode != 0U) {
		pmu_cal_pad_timing_delay_set(zero, 5U, 1U);
		pmu_cal_sequence_pulse_send(zero, 0x25U, zero, 0x86U, zero,
					    slice_or_rank, zero);
		pmu_cal_sequence_pulse_send(zero, 0x26U, zero, 0x86U, zero,
					    zero, zero);
		pmu_cal_pad_timing_delay_set(zero, 0x10U, 1U);
		pmu_cbt_3phase_pulse_seq();
	} else {
		pmu_cal_sequence_pulse_send(0x80U, 0x19U, 4U, 0x86U, zero,
					    slice_or_rank & 3U, zero);
	}
	pmu_cal_slice_step_diff_latch(zero, 1U);
	if (mode == 0U)
		pmu_delay_us(0x7530U, zero);
}

/**
 * pmu_rank_slice_deskew_latch() - Latch rank slice deskew step delay
 * @step_delay: Calibrated step delay value
 * @lane_mask: Target lane/slice bitmask
 */
static void pmu_rank_slice_deskew_latch(u32 step_delay, u32 lane_mask)
{
	u32 shift = pmu_freq_ratio_mult(14U);

	pmu_cal_strobe_pulse_init();
	pmu_cal_sequence_pulse_send(0U, 6U, 2U, step_delay, 30U,
				    lane_mask, 0U);
	if (shift < 5U)
		shift = 5U;
	pmu_cal_pad_timing_delay_set(0U, (u8)shift, 0U);
	pmu_cal_slice_step_diff_latch(0U, 1U);
}

/**
 * pmu_slice_deskew_state_latch() - Latch slice deskew state via opcode 24 pulse
 * @step_delay: Calibrated step delay value
 * @lane_mask: Target lane/slice bitmask
 */
static void pmu_slice_deskew_state_latch(u32 step_delay, u32 lane_mask)
{
	u32 zero = 0U;

	pmu_cal_strobe_pulse_init();
	pmu_cal_sequence_pulse_send(0U, 6U, 0x40U, step_delay, 24U,
				    lane_mask, zero);
	pmu_cal_slice_step_diff_latch(0U, 1U);
}

/**
 * pmu_channel_timing_deskew_reset() - Reset channel timing deskew parameters
 * @dly_ac0: AC delay parameter 0 (tap 33)
 * @dly_ac1: AC delay parameter 1 (tap 34)
 * @dly_ac2: AC delay parameter 2 (tap 31)
 * @dly_ac3: AC delay parameter 3 (tap 32)
 * @target_slice: Target slice bitmask
 */
void pmu_channel_timing_deskew_reset(u32 dly_ac0, u32 dly_ac1, u32 dly_ac2,
				     u32 dly_ac3, u32 target_slice)
{
	u32 zero = 0U;
	u32 val0;

	pmu_cal_strobe_pulse_init();
	val0 = *(__volatile__ const u8 *)PMU_DMEM_CH_MASK_9B8 & 0x3fU;

	pmu_cal_sequence_pulse_send(0U, 6U, 0x22U, val0, 20U,
				    target_slice, zero);
	pmu_cal_sequence_pulse_send(0U, 6U, 0x22U, dly_ac0, 33U,
				    target_slice, zero);
	pmu_cal_sequence_pulse_send(0U, 6U, 0x22U, dly_ac1, 34U,
				    target_slice, zero);
	pmu_cal_sequence_pulse_send(0U, 6U, 0x22U, dly_ac2, 31U,
				    target_slice, zero);
	pmu_cal_sequence_pulse_send(0U, 6U, 0x22U, dly_ac3, 32U,
				    target_slice, zero);

	pmu_cal_slice_step_diff_latch(0U, 1U);
}

/**
 * pmu_slice_deskew_pulse_train() - Execute slice deskew pulse train for rank
 * @ch_idx: Target channel index (0 or 1)
 * @rank: Target rank index
 * @mode: Non-zero for full multi-pulse sequence, 0 for short sequence
 */
static void pmu_slice_deskew_pulse_train(u32 ch_idx, u32 rank, u32 mode)
{
	u32 zero = 0U;
	const u8 *p = (const u8 *)(PMU_DMEM_CAL_STRUCT_BASE_9A4 +
				   (ch_idx * 2U + rank) * PMU_SLICE_DESC_SIZE);
	u32 mask1 = 1U << rank;
	u8 slice_pair_mask = (u8)((mask1 << 2) | mask1);
	u32 deskew_cmd;

	deskew_cmd = ((mode == 0U) ? (p[0x10] | 5U) : (p[0x10] & 0xb0U)) |
		     (1U << 6);

	pmu_cal_strobe_pulse_init();

	if (mode != 0U) {
		pmu_cal_sequence_pulse_send(zero, 6U, 0x22U, deskew_cmd | 4U,
					    16U, slice_pair_mask, zero);

		if ((p[1] & 8U) != 0U)
			pmu_cal_sequence_pulse_send(zero, 6U, 0x22U, p[1], 1U,
						    slice_pair_mask, zero);
		else if ((p[20] & 3U) == 2U)
			pmu_cal_strobe_pulse_init();

		if ((p[20] & 3U) != 2U)
			pmu_cal_sequence_pulse_send(zero, 6U, 0x22U, p[20],
						    20U, slice_pair_mask, zero);
	}

	pmu_cal_sequence_pulse_send(0x80U, 6U, 0x22U, deskew_cmd, 16U,
				    slice_pair_mask, zero);
	pmu_cal_slice_step_diff_latch(zero, 1U);
	pmu_delay_us(0x3d090U, zero);
}

/**
 * pmu_dbyte_slice_cal_param_pulse() - Send DBYTE slice calibration parameter pulse
 * @rank: Target rank index
 * @channel: Target channel index (0 or 1)
 * @slice_mask: Non-zero to set bits 3 and 6 in parameter byte
 */
static void pmu_dbyte_slice_cal_param_pulse(u32 rank, u32 channel, u32 slice_mask)
{
	u32 zero = 0U;
	u8 val;
	u32 mask1;

	pmu_cal_strobe_pulse_init();

	val = *(const u8 *)(PMU_DMEM_CAL_STRUCT_BASE_9A4 +
			    (rank * 2U + channel) * PMU_SLICE_DESC_SIZE + 18U);
	if (slice_mask != 0U)
		val |= 0x48U;

	mask1 = 1U << channel;
	pmu_cal_sequence_pulse_send(zero, 6U, 0x22U, val, 18U,
				    (u8)((mask1 << 2) | mask1), zero);
	pmu_cal_sequence_pulse_send(0x80U, 7U, 2U, zero, zero, zero, zero);
	pmu_cal_slice_step_diff_latch(zero, 1U);
}

static __always_inline void
pmu_fine_tune_pulse_rank(const __volatile__ u8 *dmem, u32 ch, u32 rank,
			 u16 reg_val, u8 num_entries)
{
	u32 ch_shift = ch * 2U;
	u8 sp18 = 0U;
	int entry_idx;

	pmu_cal_strobe_pulse_init();
	pmu_set_458_pair((u16)(reg_val << 2));

	for (entry_idx = 0; entry_idx < (int)num_entries; entry_idx++) {
		u32 lane;

		for (lane = 0U; lane < 2U; lane++) {
			u8 cal_cmd = dmem[0x46cU + entry_idx];
			u8 delay_val = dmem[0x474U + (rank ? 0x14U : 0U) +
					    (ch_shift + entry_idx * 4) + lane];
			u32 pulse_mode = (lane != 0U &&
					  entry_idx != (int)num_entries - 1) ?
					 0x22U : 0U;
			u32 lane_bitmask = 1U << lane;

			if (sp18 != 0U && cal_cmd == 0x0cU) {
				if (((dmem[0x1c] >> ch_shift) &
				     lane_bitmask) == 0U)
					continue;
				delay_val |= 0x80U;
			}

			pmu_cal_sequence_pulse_send(0U, 6U, pulse_mode,
						    delay_val, cal_cmd,
						    (lane_bitmask << 2) |
						    lane_bitmask, 0U);

			if (lane == 1U && sp18 == 0U && cal_cmd == 0x0cU) {
				if (dmem[0x1c] != 0U) {
					entry_idx--;
					sp18 = 1U;
				} else {
					sp18 = 0U;
				}
			}

			if (cal_cmd == 0x0cU && dmem[0x1c] == 0U)
				pmu_cal_pad_timing_delay_set(0U, 4U, 0U);
		}
	}

	pmu_cal_sequence_pulse_send(0U, 7U, 2U, 0U, 0U, 0U, 0x28U);
}

/**
 * pmu_cal_dbyte_deskew_fine_tune() - Execute DBYTE deskew fine-tune pass
 * @mode: Non-zero for CSR writeback mode, 0 for pulse sequence mode
 */
void pmu_cal_dbyte_deskew_fine_tune(u32 mode)
{
	const __volatile__ u8 *dmem = (const __volatile__ u8 *)PMU_DMEM_BASE;
	u32 counter = 0U;
	u32 ch;

	for (ch = 0U; ch < 2U; ch++) {
		u8 active = (ch == 0U) ? dmem[0x25] : dmem[0x40];
		u16 reg_pair[2];
		u8 num_entries;
		u32 rank;

		if (active == 0U)
			continue;

		pmu_phy_mode_cfg_dispatch(ch);

		reg_pair[0] = *(const __volatile__ u16 *)
			(dmem + 0xe8U + ch * 2U);
		reg_pair[1] = *(const __volatile__ u16 *)
			(dmem + 0xecU + ch * 2U);
		num_entries = pmu_sdata()->metric_tbl_entries;

		if (mode != 0U) {
			u32 entry;

			for (rank = 0U; rank < 2U; rank++) {
				for (entry = 0U; entry < num_entries; entry++) {
					u8 val15 = dmem[0xb6eU + ch * 2U +
							rank + entry * 4U];
					u32 csr_addr;

					csr_addr = PHY_REG_APB_BASE |
						   ((((reg_pair[0] << 2) |
						      0x41000U) +
						     (counter & 0xffU)) << 1);
					*(__volatile__ u16 *)csr_addr = val15;
					counter++;
				}
			}

			if ((counter & 1U) != 0U) {
				u32 csr_addr = PHY_REG_APB_BASE |
					       ((((reg_pair[0] << 2) |
						  0x41000U) +
						 (counter & 0xffU)) << 1);

				*(__volatile__ u16 *)csr_addr = 0U;
				counter++;
			}
		} else {
			for (rank = 0U; rank < 2U; rank++)
				pmu_fine_tune_pulse_rank(dmem, ch, rank,
							 reg_pair[rank],
							 num_entries);
		}
	}

	*(__volatile__ u16 *)PHY_REG_CAL_STAT_CTRL_008 = 0U;
	*(__volatile__ u16 *)PHY_REG_CAL_STAT_CTRL_00C = 0U;
}

/**
 * pmu_cal_gate_eye_master_trigger() - Trigger AC lane profile and gate eye error check
 */
void pmu_cal_gate_eye_master_trigger(void)
{
	u8 a7c;
	u32 mode = 0U;

	pmu_phy_mode_cfg_dispatch(3U);
	a7c = *(__volatile__ const u8 *)PMU_DMEM_AC_CAL_ACTIVE;

	if (a7c != 0U) {
		u8 d1 = *(__volatile__ const u8 *)PMU_DMEM_CLK_GATE_FLAG;

		if ((d1 & (1U << 3)) == 0U) {
			pmu_ac_lane_profile_setup();
			mode = 1U;
		}
	}

	pmu_dbyte_lane_error_mask_calc(mode, 0x0fU);
}

struct dly_line_entry {
	u16 val16;
	u8 val8[2];
};

/**
 * pmu_slice_dly_tap_coeff_prog() - Accumulate and program per-slice delay line tap coefficients
 * @rank: Active 9-bit lane bitmask
 */
static void pmu_slice_dly_tap_coeff_prog(u32 rank)
{
	struct dly_line_entry buf[6][2];
	u32 dram_type = pmu_phy_query_operating_mode();
	u32 count = ((dmem_read8(PMU_DMEM_CH1_ANALOG_EN_40) |
		      dmem_read8(PMU_DMEM_CH0_ANALOG_EN_25)) == 3U) ?
		    2U : 1U;
	u32 pstate = dmem_read8(PMU_DMEM_SLICE_START);
	u32 pstate_end = dmem_read8(PMU_DMEM_SLICE_END);

	for (; pstate <= pstate_end; pstate++) {
		u32 lane;

		if ((dmem_read8(PMU_DMEM_ACTIVE_SLICE_MASK) &
		     (1U << pstate)) == 0U)
			continue;

		for (lane = 0U; lane < 9U; lane++) {
			u32 csr_base;
			u32 k;

			if ((rank & (1U << lane)) == 0U)
				continue;

			csr_base = pmu_sdata()->active_csr_offset |
				   (pstate << 12) | (lane << 8);

			for (k = 0U; k < count; k++) {
				u32 idx = pstate * 20U + lane * 2U + k;
				__volatile__ const u16 *p6098 =
					(__volatile__ const u16 *)
					(PMU_DMEM_CBT_COEFF_6098 + idx * 2U);
				__volatile__ const u8 *p61d8 =
					(__volatile__ const u8 *)
					(PMU_DMEM_CBT_FLAGS_61D8 + idx);
				u32 d;

				buf[0][k].val16 = p6098[0];
				buf[0][k].val8[0] = p61d8[0];
				buf[0][k].val8[1] = p61d8[0xa0];

				buf[1][k].val16 = p6098[0x50];
				buf[1][k].val8[0] = p61d8[0x50];
				buf[1][k].val8[1] = p61d8[0xf0];

				for (d = 0U; d < 2U; d++) {
					u32 c_addr = (PHY_REG_DBYTE_BASE |
						      (d ? 0x0020UL : 0x0024UL)) |
						     ((csr_base | k) << 1);
					u16 raw_c = *(__volatile__ const u16 *)c_addr;
					u32 s_idx;

					buf[2U + d][k].val16 = pmu_dly_line_repack(raw_c);
					for (s_idx = 0U; s_idx < 2U; s_idx++) {
						u32 off = pmu_dly_reg_offset_select(d,
											  s_idx,
											  k);
						u32 r_addr = PHY_REG_DBYTE_BASE |
							     ((csr_base | off) << 1);

						buf[2U + d][k].val8[s_idx] =
							(u8)(*(__volatile__ const u16 *)r_addr);
					}
				}
			}

			pmu_cal_stage42_accum_dispatch(dram_type, count, buf);

			for (k = 0U; k < count; k++) {
				u32 d;
				u32 num_s;
				u32 s_idx;

				for (d = 0U; d < 2U; d++) {
					u32 c_addr = (PHY_REG_DBYTE_BASE |
						      (d ? 0x0020UL : 0x0024UL)) |
						     ((csr_base | k) << 1);

					*(__volatile__ u16 *)c_addr =
						buf[4U + d][k].val16;
				}

				if (dmem_read8(PMU_DMEM_CAL_OVERRIDE_CE) != 0U)
					continue;

				num_s = (pmu_phy_query_operating_mode() == 3U) ?
					2U : 1U;
				for (s_idx = 0U; s_idx < num_s; s_idx++) {
					for (d = 0U; d < 2U; d++) {
						u32 off = pmu_dly_reg_offset_select(d,
											  s_idx,
											  k);
						u32 w_addr = PHY_REG_DBYTE_BASE |
							     ((csr_base | off) << 1);

						*(__volatile__ u16 *)w_addr =
							(u16)buf[4U + d][k].val8[s_idx];
					}
				}
			}
		}
	}
}

#pragma GCC push_options
#pragma GCC optimize("no-expensive-optimizations,no-tree-dominator-opts")
static __always_inline void
deskew_eval_coarse_fine_slice(u32 slice, s32 fp, s32 ph, u8 desc_flag8,
			      u16 lane_tap_steps_b[2][4],
			      u16 lane_metric_buf_b[2][32],
			      u16 lane_metric_buf_a[2][32],
			      s32 *max_coarse, s32 *max_fine)
{
	u32 bit;

	if ((desc_flag8 & 1U) != 0U) {
		s32 v_fp = (s32)(lane_tap_steps_b[fp][slice] >> 6);
		s32 v_ph = (s32)(lane_tap_steps_b[ph][slice] >> 6);
		s32 diff = v_fp - v_ph;

		if (diff > *max_coarse)
			*max_coarse = diff;
	} else {
		for (bit = 0U; bit < 8U; bit++) {
			s32 v_fp = (s32)(lane_metric_buf_b[fp][slice * 8U + bit] >> 6);
			s32 v_ph = (s32)(lane_metric_buf_b[ph][slice * 8U + bit] >> 6);
			s32 diff = v_fp - v_ph;

			if (diff > *max_coarse)
				*max_coarse = diff;
		}
	}

	for (bit = 0U; bit < 8U; bit++) {
		s32 val_fp = (s32)lane_metric_buf_a[fp][slice * 8U + bit];
		s32 val_ph = (s32)lane_metric_buf_a[ph][slice * 8U + bit];
		s32 diff = val_fp - val_ph;

		if (diff > *max_fine)
			*max_fine = diff;
	}
}

static __always_inline void
deskew_eval_cross_slice(u32 slice, s32 phase_b, s32 phase_a, u8 desc_flag8,
			u8 cal_cfg16, u16 lane_tap_steps_b[2][4],
			u16 lane_tap_steps_a[2][4],
			u16 lane_metric_buf_b[2][32],
			u16 lane_metric_buf_a[2][32],
			s32 *max_diff, s32 *max_cross)
{
	u32 bit;

	if ((desc_flag8 & 1U) != 0U) {
		s32 ref_dly = (s32)lane_tap_steps_b[phase_b][slice];

		for (bit = 0U; bit < 8U; bit++) {
			s32 samp_dly = (s32)lane_metric_buf_a[phase_a][slice * 8U + bit];
			s32 diff = ref_dly - samp_dly;

			if (diff > *max_diff)
				*max_diff = diff;
		}
		if ((cal_cfg16 & 0x10U) != 0U) {
			s32 aux_dly = (s32)lane_tap_steps_a[phase_a][slice];
			s32 diff = ref_dly - aux_dly;

			if (diff > *max_diff)
				*max_diff = diff;
		}
	} else {
		for (bit = 0U; bit < 8U; bit++) {
			s32 tap_a = (s32)lane_metric_buf_a[phase_a][slice * 8U + bit];
			s32 tap_b = (s32)lane_metric_buf_b[phase_b][slice * 8U + bit];
			s32 diff = tap_b - tap_a;

			if (diff > *max_diff)
				*max_diff = diff;
		}
	}

	if (phase_a != phase_b) {
		for (bit = 0U; bit < 8U; bit++) {
			s32 cross_a = (s32)lane_metric_buf_a[phase_a][slice * 8U + bit];
			s32 cross_b = (s32)lane_metric_buf_a[phase_b][slice * 8U + bit];
			s32 diff = cross_b - cross_a;

			if (diff > *max_cross)
				*max_cross = diff;
		}
		if ((cal_cfg16 & 0x10U) != 0U) {
			s32 aux_b = (s32)lane_tap_steps_a[phase_b][slice];
			s32 aux_a = (s32)lane_tap_steps_a[phase_a][slice];
			s32 diff = aux_b - aux_a;

			if (diff > *max_cross)
				*max_cross = diff;
		}
	}
}

/**
 * pmu_cal_lane_deskew_sweep_exec() - Execute per-lane deskew sweep across active slices
 * @channel: Target channel index (0 or 1)
 */
void pmu_cal_lane_deskew_sweep_exec(u32 channel)
{
	u16 lane_metric_buf_a[2][32];
	u16 lane_metric_buf_b[2][32];
	u16 lane_tap_steps_b[2][4];
	u16 lane_tap_steps_a[2][4];
	const u8 *desc = (const u8 *)dmem_ptr(PMU_DMEM_CHANNEL_CFG);
	const u8 *desc_p;
	u8 desc_mode;
	u8 desc_flag8;
	u8 cal_cfg16;
	u32 flag_78;
	u32 csr_offset;
	u32 phase;
	u32 rank_off;
	__volatile__ u8 *ptr_14;
	__volatile__ u8 *ptr_28;
	s32 fp;
	s32 ph;
	u32 flag_28;
	__volatile__ u8 *ptr_1c;
	__volatile__ u8 *ptr_18;
	s32 phase_b;
	s32 phase_a;

	memset(lane_metric_buf_b, 0, sizeof(lane_metric_buf_b));
	memset(lane_metric_buf_a, 0, sizeof(lane_metric_buf_a));
	memset(lane_tap_steps_b, 0, sizeof(lane_tap_steps_b));
	memset(lane_tap_steps_a, 0, sizeof(lane_tap_steps_a));

	desc_p = desc + 130U;
	desc_mode = desc_p[0];
	desc_flag8 = desc_p[8];
	cal_cfg16 = desc_p[16];
	flag_78 = desc_mode & 0x10U;

	csr_offset = pmu_sdata()->active_csr_offset;

	for (phase = 0U; phase < 2U; phase++) {
		u8 start_slice = dmem_read8(PMU_DMEM_SLICE_START);
		u8 end_slice = dmem_read8(PMU_DMEM_SLICE_END);
		u32 slice;

		if ((desc_flag8 & 1U) != 0U) {
			pmu_slice_lcdl_delay_collect((phase + 0x20U) | csr_offset,
						     lane_tap_steps_b[phase], 1U);
		} else {
			for (slice = start_slice; slice <= end_slice; slice++) {
				u32 slice_code = (slice << 12) |
						 ((phase + 0x24U) | csr_offset);
				u32 bit;

				for (bit = 0U; bit < 8U; bit++) {
					u32 reg = PHY_REG_DBYTE_BASE |
						  ((slice_code |
						    (bit << 8)) << 1);

					lane_metric_buf_b[phase][slice * 8U + bit] =
						*(__volatile__ const u16 *)reg;
				}
			}
		}

		for (slice = start_slice; slice <= end_slice; slice++) {
			u32 slice_code = (slice << 12) |
					 ((phase + 0x26U) | csr_offset);
			u32 bit;

			for (bit = 0U; bit < 8U; bit++) {
				u32 reg = PHY_REG_DBYTE_BASE |
					  ((slice_code | (bit << 8)) << 1);

				lane_metric_buf_a[phase][slice * 8U + bit] =
					*(__volatile__ const u16 *)reg;
			}
		}

		if ((cal_cfg16 & 0x10U) != 0U) {
			pmu_slice_lcdl_delay_collect((phase + 0x28U) | csr_offset,
						     lane_tap_steps_a[phase], 1U);
		}
	}

	rank_off = (channel != 0U) ? 27U : 0U;
	ptr_14 = (__volatile__ u8 *)(PMU_DMEM_CH0_DESKEW_COARSE_26 + rank_off);
	ptr_28 = (__volatile__ u8 *)(PMU_DMEM_CH0_DESKEW_FINE_30 + rank_off);

	for (fp = 1; fp >= 0; fp--) {
		for (ph = 1; ph >= 0; ph--) {
			s16 diff_coarse = 0;
			s16 diff_fine = 0;

			if (ph == fp)
				continue;

			if (flag_78 != 0U) {
				u8 start_slice = dmem_read8(PMU_DMEM_SLICE_START);
				u8 end_slice = dmem_read8(PMU_DMEM_SLICE_END);
				s32 max_coarse = 0;
				s32 max_fine = 0;
				u32 slice;

				for (slice = start_slice; slice <= end_slice;
				     slice++) {
					deskew_eval_coarse_fine_slice(slice, fp, ph,
								      desc_flag8,
								      lane_tap_steps_b,
								      lane_metric_buf_b,
								      lane_metric_buf_a,
								      &max_coarse,
								      &max_fine);
				}

				diff_coarse = (s16)pmu_math_freq_scale_div(max_coarse << 6);
				diff_fine = (s16)pmu_math_freq_scale_div(max_fine);
			}

			*ptr_14++ = (u8)diff_coarse;
			*ptr_28++ = (u8)diff_fine;
		}
	}

	flag_28 = (flag_78 != 0U) ? 1U : 0U;
	ptr_1c = (__volatile__ u8 *)(PMU_DMEM_CH0_DESKEW_CROSS_2C + rank_off);
	ptr_18 = (__volatile__ u8 *)(PMU_DMEM_ACTIVE_SLICE_CNT_28 + rank_off);

	for (phase_b = 1; phase_b >= 0; phase_b--) {
		for (phase_a = 1; phase_a >= 0; phase_a--) {
			s16 res_diff = 0;
			s16 res_cross = 0;

			if (!(phase_a != phase_b && flag_28 == 0U)) {
				u8 start_slice = dmem_read8(PMU_DMEM_SLICE_START);
				u8 end_slice = dmem_read8(PMU_DMEM_SLICE_END);
				s32 max_diff = 0;
				s32 max_cross = 0;
				u32 slice;

				for (slice = start_slice; slice <= end_slice;
				     slice++) {
					deskew_eval_cross_slice(slice, phase_b, phase_a,
								desc_flag8, cal_cfg16,
								lane_tap_steps_b,
								lane_tap_steps_a,
								lane_metric_buf_b,
								lane_metric_buf_a,
								&max_diff, &max_cross);
				}

				res_diff = (s16)pmu_math_freq_scale_div(max_diff);
				res_cross = (s16)pmu_math_freq_scale_div(max_cross);
			}

			*ptr_18++ = (u8)res_diff;
			*ptr_1c++ = (u8)res_cross;
		}
	}
}

#pragma GCC pop_options

#pragma GCC push_options
#pragma GCC optimize("no-tree-switch-conversion,-fno-reorder-blocks")
#pragma GCC optimize("-fno-if-conversion2,-fno-expensive-optimizations")
/**
 * pmu_cal_bist_pattern_setup() - Configure BIST lane masks and pattern generators
 * @pattern_idx: BIST pattern index (0..8)
 * @slice_or_lane: Target lane index or 0x0f for broadcast
 * @enable_deskew_reset: Non-zero to reset channel timing deskew
 * @validation_mode: Validation mode selector
 */
void pmu_cal_bist_pattern_setup(u32 pattern_idx, u32 slice_or_lane,
				u32 enable_deskew_reset, u32 validation_mode)
{
	u16 pat = 0x5a3cU;
	u32 valid_mode = 1U;
	s8 offset_val = 0;
	u32 is_mid;
	u16 val_e1ae;
	u16 bctrl;

	switch (pattern_idx) {
	case 0U:
	case 8U:
		break;
	case 1U:
		pat = 0xff00U;
		valid_mode = 0U;
		break;
	case 2U:
	case 4U:
		pat = 0xa536U;
		valid_mode = 0U;
		break;
	case 3U:
		pat = 0xaaaaU;
		valid_mode = 0U;
		break;
	case 5U:
		pat = 0xb2b2U;
		valid_mode = 0U;
		break;
	case 6U:
		pat = 0x8241U;
		valid_mode = 0U;
		break;
	case 7U:
		pat = *(__volatile__ const u16 *)PMU_DMEM_TEST_PAT_E4;
		valid_mode = 0U;
		break;
	default:
		pmu_assert_or_halt(0U, PMU_ERR_BIST_PATTERN_IDX_INVALID);
		pat = 0U;
		valid_mode = 0U;
		break;
	}

	if (enable_deskew_reset != 0U) {
		u8 e6 = *(__volatile__ const u8 *)PMU_DMEM_DESKEW_OFFSET_E6;

		offset_val = (e6 != 0U) ? (s8)e6 : (s8)-86;
	}

	is_mid = (pattern_idx == 1U || (pattern_idx - 3U < 5U)) ? 1U : 0U;
	if (is_mid != 0U)
		pmu_channel_timing_deskew_reset((u8)pat,
						(pat >> 8) & 0xffU,
						    (u32)offset_val,
						    (u32)offset_val,
						    slice_or_lane);

	pmu_write_phy_c2_quad(0xffffU);
	*(__volatile__ u16 *)PHY_REG_AC_BCAST_PAT_0CA = 0U;
	pmu_cal_deskew_quad_tap_set(0x0fU, pat, pat, pat, pat);

	if (enable_deskew_reset != 0U) {
		if (pattern_idx == 0U) {
			*(__volatile__ u16 *)PHY_REG_AC_BCAST_FE022 = 22U;
		} else {
			u32 i;

			pmu_assert_or_halt(((validation_mode != 0U) ? 1U : 0U) |
					   valid_mode,
					   PMU_ERR_BIST_VALIDATION_MODE);
			*(__volatile__ u16 *)PHY_REG_AC_BCAST_FE022 = 16U;
			for (i = 0U; i < 8U; i++) {
				u16 p = ((offset_val & (1 << i)) != 0) ?
					(u16)~pat : pat;

				pmu_cal_deskew_quad_tap_set(i, p, p, p, p);
			}
			pmu_cal_deskew_quad_tap_set(8U, pat, pat, pat, pat);
		}
	} else {
		*(__volatile__ u16 *)PHY_REG_AC_BCAST_FE022 = 16U;
	}

	val_e1ae = (is_mid != 0U) ? 0U : 256U;
	*(__volatile__ u16 *)PHY_REG_AC_BCAST_PAT_1AE = val_e1ae;
	*(__volatile__ u16 *)PHY_REG_AC_BCAST_PAT_1AC = val_e1ae;

	bctrl = ((*(__volatile__ const u8 *)PMU_DMEM_FREQ_ROUND_BIAS == 4U) ?
		 0x84U : 4U) | ((pattern_idx != 0U) ? 5U : 8U);
	*(__volatile__ u16 *)PHY_REG_AC_BCAST_PAT_0C0 = bctrl;
	if (pattern_idx == 0U)
		*(__volatile__ u16 *)PHY_REG_AC_BCAST_PAT_0CA = 0U;
}

#pragma GCC pop_options

#pragma GCC push_options
#pragma GCC optimize("no-expensive-optimizations,no-tree-dominator-opts")
/**
 * pmu_sweep_save_or_restore_lanes() - Save or restore per-lane delay registers across slices
 * @local_buf: 40-halfword per-slice/lane backup buffer
 * @slice_start: First slice index
 * @slice_end: Last slice index
 * @active_mask: Active slice bitmask
 * @active_csr_offset: Shifted channel CSR offset
 * @ctx_off10: Shifted secondary register offset from sweep context
 * @loop_bound: Number of lanes to save or restore per slice
 * @save: true to read from PHY into @local_buf, false to write @local_buf back to PHY
 */
static void pmu_sweep_save_or_restore_lanes(u16 *local_buf, u8 slice_start,
					    u8 slice_end, u8 active_mask,
					    u32 active_csr_offset,
					    u32 ctx_off10, u32 loop_bound,
					    bool save)
{
	u32 slice;

	for (slice = slice_start; slice <= slice_end; slice++) {
		u32 base;
		u16 *p_buf;
		u32 lane;

		if ((active_mask & (1U << slice)) == 0U)
			continue;

		base = PHY_REG_DBYTE_BASE | (slice << 13) | active_csr_offset |
		       ctx_off10;
		p_buf = &local_buf[slice * 10U];

		for (lane = 0U; lane < loop_bound; lane++) {
			u32 addr = base | (lane << 9);

			if (save)
				p_buf[lane] = *(__volatile__ const u16 *)addr;
			else
				*(__volatile__ u16 *)addr = p_buf[lane];
		}
	}
}

/**
 * pmu_sweep_update_lane_state() - Update per-lane eye edge state machine for one sample
 * @entry: Pointer to 848-byte lane sweep record
 * @eff_step: Effective step index within the record
 * @cur_val: Current sweep step value
 * @phy_sample: Sampled 16-bit PHY comparator status
 */
static void pmu_sweep_update_lane_state(__volatile__ u8 *entry, u32 eff_step,
					u8 cur_val, u16 phy_sample)
{
	__volatile__ u8 *state_ptr = entry + 636U + eff_step;
	__volatile__ u8 *rec = entry + eff_step * 3U;
	u8 state = *state_ptr;

	if (state == 1U) {
		if (phy_sample != 0U) {
			rec[1] = cur_val;
			*state_ptr = 2U;
		}
	} else if (phy_sample == 0U) {
		if ((state | 2U) == 2U) {
			rec[state] = cur_val;
			*state_ptr = state + 1U;
		}
	} else if (state == 3U) {
		if ((u8)(cur_val - rec[2]) > (u8)(rec[1] - rec[0])) {
			rec[0] = rec[2];
			rec[1] = cur_val;
		}
		*state_ptr = 2U;
	}
}

/**
 * pmu_cal_slice_step_eval_sweep() - Execute slice delay step evaluation sweep (0x36ac)
 * @results_buf: Output buffer for per-slice/lane eye edge records
 * @sweep_mode: Sweep mode selector
 * @cal_mode: Calibration mode (0..3)
 * @channel_rank: Active channel/rank selector
 * @unused: Unused parameter
 * @timing_val: Timing latch parameter
 * @overflow_flag: Non-zero to enable coarse step wrapping
 * @slice_delay_lut: 4-entry per-slice initial delay table
 * @bound_flag: Non-zero for 9 lanes, 0 for 8 lanes
 * @remap_flag: Non-zero to remap lane indices via 0x80000ba0
 * @sweep_ctx: Context structure holding step parameters and valid masks at +0x1c
 */
void pmu_cal_slice_step_eval_sweep(void *results_buf, u32 sweep_mode,
				   u32 cal_mode, u32 channel_rank, u32 unused,
				   u32 timing_val, u32 overflow_flag,
				   const u16 *slice_delay_lut, u32 bound_flag,
				   u32 remap_flag, const void *sweep_ctx)
{
	u16 local_buf[40];
	u16 local_scratch_44[4] = { 0 };
	u32 loop_bound = (bound_flag != 0U) ? 9U : 8U;
	u32 profile_mode = pmu_phy_query_operating_mode();
	u32 flag_fp = 0U;
	u32 active_csr_offset;
	__volatile__ const u8 *b68 = (__volatile__ const u8 *)PMU_DMEM_CH_RANGE_B68;
	u8 slice_start, slice_end, active_mask;
	const u32 *ctx32;
	u32 ctx_off04, ctx_off08, ctx_off0c, ctx_off10;
	u32 lane_start = 0U;
	u32 flag_20, lane_csr_off, slice;
	u16 f6a_val = 1U;
	u32 max_sweep_steps, cal_mode_even, sweep_step;

	(void)unused;
	memset(local_buf, 0, 80U);

	if (((u32)(sweep_mode - 3U) < 5U || sweep_mode == 1U) &&
	    pmu_check_dmem_407_flag() != 0U) {
		u8 s_start = b68[0];
		u8 s_end = b68[1];

		flag_fp = 1U;
		if (s_start <= s_end)
			memset((u8 *)sweep_ctx + 0x1cU + s_start * 2U, 0,
			       (s_end - s_start + 1U) * 2U);
	}

	active_csr_offset = pmu_sdata()->active_csr_offset << 1;
	slice_start = b68[0];
	slice_end = b68[1];
	active_mask = b68[0x30];

	ctx32 = (const u32 *)sweep_ctx;
	ctx_off04 = ctx32[1];
	ctx_off08 = ctx32[2];
	ctx_off0c = ctx32[3] << 1;
	ctx_off10 = ctx32[4] << 1;

	if (cal_mode == 3U) {
		loop_bound = 10U;
		lane_start = 9U;
	}
	flag_20 = (cal_mode == 1U || flag_fp != 0U) ? 1U : 0U;

	if (flag_20 != 0U)
		pmu_sweep_save_or_restore_lanes(local_buf, slice_start,
						slice_end, active_mask,
						active_csr_offset, ctx_off10,
						loop_bound, true);

	lane_csr_off = (cal_mode == 3U) ? 0U : 0x1e00U;
	for (slice = slice_start; slice <= slice_end; slice++) {
		if ((active_mask & (1U << slice)) != 0U) {
			u16 val = slice_delay_lut[slice];
			u32 s_base = PHY_REG_DBYTE_BASE | (slice << 13) |
				     lane_csr_off | active_csr_offset;

			*(__volatile__ u16 *)(s_base | ctx_off0c) = val;
			if (flag_20 != 0U)
				*(__volatile__ u16 *)(s_base | ctx_off10) = val;
		}
	}

	pmu_pulse_phy_140();

	if (channel_rank < 4U && (profile_mode & ~1U) == 2U) {
		*(__volatile__ u16 *)(PHY_REG_DBYTE_BCAST_MODE |
				      active_csr_offset) =
			(channel_rank >= 2U) ? 1U : 0U;
		if (profile_mode == 3U) {
			*(__volatile__ u16 *)PHY_REG_CBT_STAT_180 = 8U;
			*(__volatile__ u16 *)PHY_REG_CBT_STAT_184 = 0U;
			*(__volatile__ u16 *)PHY_REG_CBT_STAT_182 =
				(channel_rank > 1U) ? 8U : 0U;
			f6a_val = 0x4001U;
		}
	}

	for (slice = slice_start; slice <= slice_end; slice++) {
		u16 *p1c = (u16 *)((u8 *)sweep_ctx + 0x1cU + slice * 2U);

		*(__volatile__ u16 *)(PHY_REG_DBYTE_GATE_1F6A | (slice << 13)) =
			(u16)((*p1c << 10) | f6a_val);
		local_scratch_44[slice] = slice_delay_lut[slice];
	}

	pmu_set_phy_44(200U);
	pmu_set_phy_46(*(const u16 *)((const u8 *)sweep_ctx + 0x14U));

	max_sweep_steps = ((cal_mode | 1U) == 3U) ? 212U : 128U;
	cal_mode_even = cal_mode & ~1U;

	for (sweep_step = 0U; sweep_step < max_sweep_steps; sweep_step++) {
		u32 eff_step = (ctx_off04 != 0U) ? sweep_step : 0U;
		u8 cur_val = (ctx_off04 != 0U) ? (u8)*ctx32 : (u8)sweep_step;

		pmu_cal_slice_timing_latch((u16)timing_val, 1U);

		for (slice = slice_start; slice <= slice_end; slice++) {
			u32 slice_base, s_base, lane;
			u16 dly;
			__volatile__ u8 *res_slice;

			if ((active_mask & (1U << slice)) == 0U)
				continue;

			slice_base = PHY_REG_DBYTE_BASE | (slice << 13);
			dly = local_scratch_44[slice];
			if (cal_mode < 2U) {
				dly += (u16)(ctx_off08 +
					     (((ctx_off08 + (dly & 0x7fU)) >
					       0x7fU) ? 0x80U : 0U));
				if (overflow_flag != 0U &&
				    (dly & 0x40U) != 0U) {
					u16 *p1c = (u16 *)((u8 *)sweep_ctx +
							   0x1cU + slice * 2U);
					u16 reg_16a = *(__volatile__ const u16 *)
						(slice_base | 0x016aUL);
					u16 inv = (u16)~(*p1c);

					dly += 64U;
					*p1c = inv;
					*(__volatile__ u16 *)
						(slice_base | 0x1f6aUL) =
						(reg_16a & 0xc3ffU) |
						((inv << 10) & 0x3c00U);
				}
			} else if (cal_mode_even == 2U) {
				dly += (u16)ctx_off08;
			}
			local_scratch_44[slice] = dly;

			s_base = slice_base | 0x1e00U | active_csr_offset;
			*(__volatile__ u16 *)(s_base | ctx_off0c) = dly;
			if (flag_20 != 0U)
				*(__volatile__ u16 *)(s_base | ctx_off10) = dly;

			res_slice = (__volatile__ u8 *)results_buf +
				    slice * 8480U;
			for (lane = lane_start; lane < loop_bound; lane++) {
				u32 lane_idx = (remap_flag != 0U && lane < 8U) ?
					       b68[0x38U + slice * 8U + lane] :
					       lane;
				u32 csr_field = (cal_mode == 3U) ?
						0x1000U : (lane << 9);
				u16 phy_sample = *(__volatile__ const u16 *)
					(slice_base | 0x016eUL | csr_field);

				pmu_sweep_update_lane_state(res_slice +
							    lane_idx * 848U,
							    eff_step, cur_val,
							    phy_sample);
			}
		}

		*(__volatile__ u16 *)PHY_REG_DBYTE_BCAST_TRIG_176 = 1023U;
		*(__volatile__ u16 *)PHY_REG_DBYTE_BCAST_TRIG_176 = 0U;
	}

	if (flag_20 != 0U)
		pmu_sweep_save_or_restore_lanes(local_buf, slice_start,
						slice_end, active_mask,
						active_csr_offset, ctx_off10,
						loop_bound, false);

	if ((profile_mode & ~1U) == 2U) {
		*(__volatile__ u16 *)(PHY_REG_DBYTE_BCAST_MODE |
				      active_csr_offset) =
			(profile_mode == 2U) ? 2U : 4U;
	}
}

#pragma GCC pop_options

/**
 * calc_step_val() - Compute reciprocal delay step scaling factor from combined tap count
 * @combined: Measured tap span across consecutive edges
 *
 * Return: Reciprocal step value in Q17 format, or 0xffff if out of bounds.
 */
static inline u16 calc_step_val(u16 combined)
{
	if (*(__volatile__ const u16 *)PMU_DMEM_DRAM_FREQ_MHZ <= 199U ||
	    combined < 2U)
		return 0xffffU;
	return (u16)(((u32)(*(__volatile__ const u8 *)
			    PMU_DMEM_FREQ_ROUND_BIAS) << 17) / combined);
}

#pragma GCC push_options
#pragma GCC optimize("no-expensive-optimizations,no-tree-dominator-opts")
/**
 * pmu_cal_slice_step_diff_commit() - Measure and commit per-slice step delay differences
 * @slice_idx: Sub-slice index (0 or 1)
 * @out_buf: Output 4-halfword slice delay buffer
 * @cal_mode: Non-zero to write calibrated delays to PHY CSRs
 */
void pmu_cal_slice_step_diff_commit(u32 slice_idx, u16 *out_buf, u32 cal_mode)
{
	u16 temp_buf[4];
	u32 slice_sub_idx = (1U << slice_idx) & 3U;
	u8 slice_pair_mask = (u8)((1U << (slice_idx + 2U)) | (1U << slice_idx));
	u8 start_slice = dmem_read8(PMU_DMEM_SLICE_START);
	u8 end_slice = dmem_read8(PMU_DMEM_SLICE_END);
	u32 pass;
	u32 slice;

	temp_buf[0] = 0U;
	temp_buf[1] = 0U;
	temp_buf[2] = 0U;
	temp_buf[3] = 0U;

	*(__volatile__ u16 *)PHY_REG_STROBE_CAL_EN_0148 = 1U;
	pmu_cal_strobe_pulse_init();
	pmu_cal_sequence_pulse_send(0U, 0x0bU, 0U, 0U, 0U, slice_sub_idx, 0U);

	for (pass = 0U; pass < 2U; pass++) {
		u16 *buf = (pass != 0U) ? temp_buf : out_buf;
		u32 p_code = (pass != 0U) ? 0x28U : 0x25U;
		u32 s_code = (pass != 0U) ? 0x83U : 0x81U;
		u32 w_code = (pass != 0U) ? 0x26U : 0x23U;

		pmu_cal_pad_timing_delay_set(0U, 0xffU, 0U);
		pmu_cal_sequence_pulse_send(0U, 6U, 0x22U, 0x40U, p_code,
					    slice_pair_mask, 0U);
		pmu_cal_sequence_pulse_send(0x80U, 0x19U, 4U, s_code, 0U,
					    slice_sub_idx, 0U);
		pmu_cal_slice_step_diff_latch(0U, 1U);
		pmu_hw_timer_delay(0x800U);
		pmu_delay_us(0x9c40U, 8U);
		pmu_vref_signed_direction_pulse_dispatch(slice_pair_mask,
							 w_code);

		for (slice = start_slice; slice <= end_slice; slice++) {
			if (pmu_cal_lane_deskew_step_advance(slice_idx,
							     slice) != 0U) {
				buf[slice] = (u16)pmu_dbyte_coarse_step_mask_sweep(slice, 4U,
									     slice_idx);
			}
		}

		pmu_vref_signed_direction_pulse_dispatch(slice_pair_mask,
							 w_code + 1U);
		for (slice = start_slice; slice <= end_slice; slice++) {
			if (pmu_cal_lane_deskew_step_advance(slice_idx,
							     slice) != 0U) {
				u16 val = (u16)pmu_dbyte_coarse_step_mask_sweep(slice, 4U,
									  slice_idx);
				u16 combined = (u16)((val << 8) | buf[slice]);
				u16 final_val = calc_step_val(combined);

				buf[slice] = final_val;
			}
		}
		pmu_cal_strobe_pulse_init();
	}

	for (slice = start_slice; slice <= end_slice; slice++) {
		if (pmu_cal_lane_deskew_step_advance(slice_idx,
						     slice) == 0U) {
			u32 neighbor = ((slice & 1U) != 0U) ?
				       (slice - 1U) : (slice + 1U);

			out_buf[slice] = out_buf[neighbor];
			temp_buf[slice] = temp_buf[neighbor];
		}
	}

	if (cal_mode != 0U && end_slice >= start_slice) {
		u32 csr_offset = pmu_sdata()->active_csr_offset;
		u32 reg_off1 = (slice_idx == 0U) ? 0x1000cU : 0x1000dU;
		u32 reg_off2 = (slice_idx == 0U) ? 0x10015U : 0x10016U;

		for (slice = start_slice; slice <= end_slice; slice++) {
			u32 slice_base = (slice << 12) | csr_offset;

			*(__volatile__ u16 *)(PHY_REG_APB_BASE |
					      ((slice_base | reg_off1) << 1)) =
				out_buf[slice];
			*(__volatile__ u16 *)(PHY_REG_APB_BASE |
					      ((slice_base | reg_off2) << 1)) =
				temp_buf[slice];
		}
	}

	pmu_cal_strobe_pulse_init();
	pmu_cal_sequence_pulse_send(0U, 0x0cU, 4U, 0U, 0U, slice_sub_idx, 0U);
	pmu_cal_sequence_pulse_send(0U, 7U, 6U, 0U, 0U, 0U, 0xffU);
	pmu_cal_sequence_pulse_send(0U, 0x10U, 4U, 0U, 0U, slice_sub_idx, 0U);
	pmu_cal_sequence_pulse_send(0x80U, 7U, 6U, 0U, 0U, 0U, 0xffU);
	pmu_cal_slice_step_diff_latch(0U, 1U);
	pmu_pulse_phy_140();
}

#pragma GCC pop_options

/**
 * pmu_cal_slice_step_scan_eval() - Evaluate multiphase Vref and slice step pulse scan
 * @step_val: Target delay step value
 * @slice_mask: Active DBYTE slice bitmask
 *
 * Return: 0xff on completion.
 */
static u32 pmu_cal_slice_step_scan_eval(u32 step_val, u32 slice_mask)
{
	u32 vref_code = *(__volatile__ const u16 *)PHY_REG_VREF_STAT & 0x7fU;
	u32 vref_scaled;
	u32 vref_shifted;
	s16 diff_bound;
	u8 bound_low;
	u8 vref_scaled_dec;
	u8 bound_high;
	u32 fp_val = 10U;
	int i;
	u32 step_target;
	s32 target_diff;
	u32 post_param = 0x1280U;

	if (vref_code == 0U)
		vref_code = 1U;

	vref_scaled = vref_code * 3U;
	vref_shifted = vref_code << 2;
	diff_bound = (s16)(vref_shifted + (u16)(vref_scaled ^ 0xffffU));
	if (diff_bound < 0) {
		u32 mult = 1U;
		s16 not_scaled = (s16)~vref_scaled;

		do {
			mult++;
			diff_bound = (s16)(vref_shifted * mult + not_scaled);
		} while (diff_bound < 0);
	}
	bound_low = (u8)diff_bound;

	pmu_cal_strobe_pulse_init();
	pmu_flag_45c_set();

	vref_scaled_dec = (u8)(vref_code * 10U - 1U);
	pmu_cal_sequence_pulse_send(0U, 7U, 0U, 0U, 0U, 0U, vref_scaled_dec);

	bound_high = (u8)(vref_scaled + 1U);
	pmu_multiphase_pulse_seq_repeat(7U, 0U, 0U, bound_high);

	for (i = 0x1d; i <= 0x20; i++) {
		pmu_multiphase_pulse_seq_repeat((u32)i, 0x12c8U,
						(i == 0x20) ? 0U : slice_mask,
						fp_val);
	}

	pmu_multiphase_pulse_seq_repeat(7U, 0U, 0U, bound_low);
	pmu_cal_sequence_pulse_send(0U, 7U, 0U, 0U, 0U, 0U, vref_scaled_dec);
	pmu_cal_sequence_pulse_send(0U, 7U, 4U, 0U, 0U, 0U, (u8)(fp_val - 1U));

	for (i = 0; i < 16; i++) {
		pmu_cal_pulse_burst_send((u32)((i & 1) ^ 1), vref_code,
					 slice_mask);
	}

	diff_bound = 1;
	pmu_cal_sequence_pulse_send(0U, 7U, 0x5cU, 0U, 0U, 0U, 0U);
	pmu_cal_sequence_pulse_send(0x40004U, 0x2eU, step_val, 0x1000U,
				    0U, 0U, 0U);
	pmu_cal_sequence_pulse_send(0x40004U, 0x2fU, step_val, 0x1000U,
				    0U, 0U, 0U);
	pmu_cal_sequence_pulse_send((1UL << 15), 7U, 4U, 0U, 0U, 0U, 1U);

	step_target = (step_val * 2U) + 100U;
	target_diff = (s32)vref_shifted - (s32)step_target;
	while (target_diff < 0) {
		diff_bound++;
		target_diff = (s16)((vref_shifted * (u16)diff_bound) -
				    step_target);
	}

	pmu_cal_sequence_pulse_send(0U, 7U, 0U, 0U, 0U, 0U, vref_scaled_dec);
	pmu_multiphase_pulse_seq_repeat(7U, 0U, 0U,
					(u8)(target_diff + bound_high));

	for (i = 0x1d; i <= 0x20; i++) {
		pmu_multiphase_pulse_seq_repeat((u32)i, post_param,
						(i == 0x20) ? 0U : slice_mask,
						fp_val);
	}

	pmu_multiphase_pulse_seq_repeat(7U, 0U, 0U, bound_low);
	pmu_cal_sequence_pulse_send(0U, 7U, 0U, 0U, 0U, 0U, vref_scaled_dec);
	pmu_flag_45c_clear();

	return 0xffU;
}

/**
 * pmu_cal_slice_deskew_result_commit() - Sweep LCDL delays and commit slice deskew results
 * @channel: Target memory channel index (0 or 1)
 * @buf: Scratch/output halfword buffer for LCDL delay readback
 * @flags: Non-zero to include DBI lane 8 and query calibration status
 */
void pmu_cal_slice_deskew_result_commit(u32 channel, void *buf, u32 flags)
{
	u32 csr_offset = pmu_sdata()->active_csr_offset;
	u32 i;
	u32 num_dly_regs = 8U;
	u8 ch_mask = (u8)(5U << channel);
	u8 start_slice;
	u8 end_slice;
	u32 slice;
	u32 dly_idx;
	u32 fp_flag = 0U;
	u8 deskew_offset = 0U;
	u32 cal_stat;
	u32 iter;
	u32 target_reg;

	for (i = 0U; i < 9U; i++)
		*(__volatile__ u16 *)(PHY_REG_DBYTE_BCAST_LANE_17C +
				      i * 0x200UL) = (u16)(1U << i);

	pmu_write_phy_c2_quad(0U);
	pmu_cal_deskew_quad_tap_set(0xffU, 0x5555U, 0x5555U,
				    0x5555U, 0x5555U);

	*(__volatile__ u16 *)PHY_REG_DBYTE_BCAST_RX_EN = 514U;
	*(__volatile__ u16 *)PHY_REG_DBYTE_BCAST_VREF_014 = 0U;
	*(__volatile__ u16 *)PHY_REG_AC_BCAST_GATE_108 = 0U;
	*(__volatile__ u16 *)PHY_REG_DBYTE_SAMPLE_3FF6A = 1U;
	*(__volatile__ u16 *)PHY_REG_AC_BCAST_PAT_1AE = 0U;
	*(__volatile__ u16 *)PHY_REG_DBYTE_BCAST_DQ_DLY2 = 0U;
	*(__volatile__ u16 *)PHY_REG_DBYTE_BCAST_DQ_DLY3 = 0U;
	*(__volatile__ u16 *)PHY_REG_DBYTE_BCAST_DQS_DLY0 = 1U;
	*(__volatile__ u16 *)PHY_REG_DBYTE_BCAST_DQS_DLY1 = 1U;
	*(__volatile__ u16 *)PHY_REG_DBYTE_BCAST_DQ_DLY0 = 0U;
	*(__volatile__ u16 *)PHY_REG_DBYTE_BCAST_DQ_DLY1 = 2U;
	*(__volatile__ u16 *)PHY_REG_AC_BCAST_PAT_0C0 = 5U;
	*(__volatile__ u16 *)PHY_REG_AC_BCAST_PAT_0CA = 511U;
	*(__volatile__ u16 *)PHY_REG_DBYTE_BCAST_DQ_DLY1 = 0U;
	*(__volatile__ u16 *)PHY_REG_DBYTE_UPPER_COARSE_F54 = 1024U;

	*(__volatile__ u16 *)(PHY_REG_DBYTE_BCAST_PULSE | (csr_offset << 1)) = 1U;
	*(__volatile__ u16 *)(PHY_REG_DBYTE_BCAST_CTRL_02C | (csr_offset << 1)) = 2U;
	pmu_phy_reg_176_strobe_pulse(0x0fU, 0x0fU);

	if (flags != 0U) {
		*(__volatile__ u16 *)PHY_REG_AC_BCAST_PAT_1AE = 0U;
		*(__volatile__ u16 *)PHY_REG_AC_BCAST_FE022 =
			*(__volatile__ const u16 *)PHY_REG_AC_CTRL_E022 | (1U << 4);
		num_dly_regs = 9U;
	}

	*(__volatile__ u16 *)(PHY_REG_DBYTE_UPPER_DLY_E24 | (csr_offset << 1)) = 256U;
	*(__volatile__ u16 *)(PHY_REG_DBYTE_UPPER_DLY_E26 | (csr_offset << 1)) = 256U;
	*(__volatile__ u16 *)(PHY_REG_DBYTE_UPPER_DLY_E20 | (csr_offset << 1)) = 256U;
	*(__volatile__ u16 *)(PHY_REG_DBYTE_UPPER_DLY_E22 | (csr_offset << 1)) = 256U;

	pmu_pulse_phy_146();
	pmu_pulse_phy_140();

	pmu_channel_timing_deskew_reset(0x55U, 0x55U, 0U, 0U, ch_mask);
	pmu_cbt_strobe_pulse_train(ch_mask, 0U);
	pmu_cal_slice_step_diff_latch(0x60U, 1U);

	*(__volatile__ u16 *)PHY_REG_DBYTE_BCAST_DQ_DLY2 = 511U;
	*(__volatile__ u16 *)PHY_REG_DBYTE_BCAST_DQ_DLY3 = 511U;
	*(__volatile__ u16 *)PHY_REG_DBYTE_BCAST_DQ_DLY0 =
		*(__volatile__ const u16 *)PHY_REG_DBYTE_DQ_DLY0_160 | 0x30U;
	*(__volatile__ u16 *)PHY_REG_DBYTE_BCAST_DQ_DLY1 = 1U;

	start_slice = dmem_read8(PMU_DMEM_SLICE_START);
	end_slice = dmem_read8(PMU_DMEM_SLICE_END);

	for (slice = start_slice; slice <= end_slice; slice++) {
		for (dly_idx = 0U; dly_idx < num_dly_regs; dly_idx++) {
			__volatile__ u16 *reg = (__volatile__ u16 *)
				(PHY_REG_DBYTE_COARSE_154 | (slice << 13) |
				 (dly_idx << 9));
			u16 val = *reg;
			u32 adjusted = val + 0x20U;

			if ((adjusted & 0x03c0U) != 0U)
				adjusted = (val - 0x20U) + 1024U;

			*reg = (u16)adjusted;
		}
	}

	cal_stat = pmu_cal_pad_voltage_dac_set(flags, 0U);

	if (cal_stat != 0U &&
	    *(__volatile__ const u8 *)PMU_DMEM_DESKEW_OFFSET_E6 != 0U) {
		pmu_phy_dbyte_cfg_unpack();
		deskew_offset = *(__volatile__ const u8 *)PMU_DMEM_DESKEW_OFFSET_E6;
		fp_flag = 1U;
	}

	pmu_channel_timing_deskew_reset(0x53U, 0xacU, deskew_offset,
					deskew_offset, ch_mask);
	pmu_cbt_strobe_pulse_train(ch_mask, 1U);
	pmu_write_phy_c2_quad(0xffffU);
	pmu_cal_deskew_quad_tap_set(0x0fU, 0xac53U, 0xac53U,
				    0xac53U, 0xac53U);

	if (cal_stat != 0U && deskew_offset != 0U) {
		u32 bit;

		for (bit = 0U; bit < 8U; bit++) {
			u16 pattern = ((deskew_offset & (1U << bit)) != 0U) ?
				      0x53acU : 0xac53U;

			pmu_cal_deskew_quad_tap_set(bit, pattern, pattern,
						    pattern, pattern);
		}
		pmu_cal_deskew_quad_tap_set(8U, 0xac53U, 0xac53U,
					    0xac53U, 0xac53U);
	}

	*(__volatile__ u16 *)PHY_REG_AC_BCAST_PAT_0C0 = 0U;
	*(__volatile__ u16 *)PHY_REG_AC_BCAST_PAT_0CA = 0U;

	for (iter = 0U; iter < 32U; iter++) {
		pmu_cal_slice_step_diff_latch(8U, 1U);

		for (slice = start_slice; slice <= end_slice; slice++) {
			for (dly_idx = 0U; dly_idx < num_dly_regs; dly_idx++) {
				u16 stat = *(__volatile__ const u16 *)
					(PHY_REG_DBYTE_LCDL_STAT_16E |
					 (slice << 13) | (dly_idx << 9));
				u32 target_lane = dly_idx;
				__volatile__ u16 *dly_reg;

				if (fp_flag != 0U && dly_idx != 8U)
					target_lane = *(__volatile__ const u8 *)
						(PMU_DMEM_DBYTE_BACKUP_BA0 +
						 slice * 8U + dly_idx);

				dly_reg = (__volatile__ u16 *)
					(PHY_REG_DBYTE_COARSE_154 |
					 (slice << 13) | (target_lane << 9));
				if (stat != 0U)
					*dly_reg = (u16)(*dly_reg + 1024U);
			}
		}

		pmu_phy_reg_176_strobe_pulse(0x0fU, 0x0fU);
	}

	if (fp_flag != 0U)
		pmu_cal_pad_impedance_lock();

	pmu_phy_lcdl_delay_read((u16 *)buf, 1U, 0U);

	target_reg = pmu_bit_lookup_e4(channel, channel) + 0x24U;
	pmu_slice_delay_step_program(target_reg, (const s16 *)buf, 1U, 0U, 0U, 0U);

	*(__volatile__ u16 *)PHY_REG_DBYTE_BCAST_DQ_DLY0 = 0U;
	*(__volatile__ u16 *)PHY_REG_DBYTE_BCAST_DQ_DLY1 = 0U;
	pmu_pulse_phy_140();
}

/**
 * fold_res() - Fold signed phase alignment residual into magnitude-plus-sign encoding
 * @v: Signed phase residual value
 *
 * Return: Folded 16-bit residual value.
 */
static inline s16 fold_res(s16 v)
{
	return (v < 1) ? (s16)(-v) : (s16)(v | 8);
}

#pragma GCC push_options
#pragma GCC optimize("no-expensive-optimizations,no-tree-dominator-opts")
/**
 * pmu_cal_wck_ck_align_pulse_send() - Perform 4-quadrant WCK-to-CK phase alignment sweep
 * @ch: Target memory channel index (0 or 1)
 * @rank: Target DRAM rank index (0 or 1)
 */
void pmu_cal_wck_ck_align_pulse_send(u32 ch, u32 rank)
{
	s16 buf1[4][15];
	s16 buf2[4][15];
	u16 buf3[4][15];
	u16 buf4[4][15];
	u16 cnt_54[4] = { 0 };
	u16 cnt_4c[4] = { 0 };
	u16 prev_44[4] = { 0 };
	u16 stat_3c[4];
	s16 res_34[4];
	u32 csr_offset;
	u32 vref_off;
	u32 reg_0e;
	u16 orig_0e;
	u8 slice_start;
	u8 dq_swap_mask;
	u8 ch_mask;
	u32 ch_rank_idx;
	u32 test_bit;
	u8 slice_idx_first;
	u8 slice_idx_other;
	u32 dq_swizzle_mask;
	u32 flag_14 = 0U;
	u32 val_18;
	u8 slice_end;
	u32 s;
	u32 iter;
	s16 val_other;
	s16 val_swiz;
	u8 val_comb;

	memset(buf1, 0,
	       sizeof(buf1) + sizeof(buf2) +
			  sizeof(buf3) + sizeof(buf4));

	csr_offset = pmu_sdata()->active_csr_offset;
	vref_off = csr_offset << 1;
	reg_0e = PHY_REG_MASTER_CFG | vref_off;

	*(__volatile__ u16 *)(PHY_REG_VREF_TRIM_CFG | vref_off) = 1U;
	*(__volatile__ u16 *)(PHY_REG_VREF_CTRL | vref_off) = 1U;

	orig_0e = *(__volatile__ const u16 *)reg_0e;
	*(__volatile__ u16 *)reg_0e = orig_0e | (1U << 8);

	slice_start = dmem_read8(PMU_DMEM_SLICE_START);
	dq_swap_mask = *(__volatile__ const u8 *)PMU_DMEM_LANE_ERR_MASK_101;

	ch_mask = (u8)(1U << ch);
	pmu_vref_signed_direction_pulse_dispatch(ch_mask, 5U);

	ch_rank_idx = (ch << 1) + rank;
	test_bit = dq_swap_mask & (1U << ch_rank_idx);
	slice_idx_first = slice_start + (test_bit != 0U);
	slice_idx_other = slice_start + (test_bit == 0U);

	dq_swizzle_mask = pmu_dbyte_coarse_step_mask_sweep(slice_idx_first, 4U, ch);

	if ((u8)dq_swizzle_mask == 1U) {
		pmu_vref_signed_direction_pulse_dispatch(ch_mask, 6U);
		if ((u8)pmu_dbyte_coarse_step_mask_sweep(dq_swizzle_mask,
							 4U, ch) == 7U)
			flag_14 = 1U;
	}

	val_18 = pmu_phy_ac_lane_timing_delay_set((ch_mask << 2) | ch_mask);
	slice_end = dmem_read8(PMU_DMEM_SLICE_END);

	for (s = slice_start; s <= slice_end; s++)
		stat_3c[s] = (u16)flag_14;

	for (iter = 0U; iter <= 14U; iter++) {
		s8 tap = (iter < 8U) ? (s8)(7U - iter) : (s8)(iter + 1U);
		s32 tap_val;
		u32 fp_val;

		pmu_rank_slice_deskew_latch((u8)(((u8)tap << 4) |
						     ((u8)tap & 0x0fU)),
						val_18);
		pmu_set_phy_44(0xc8U);
		pmu_set_phy_46((u16)val_18);
		pmu_cal_slice_timing_latch(0U, 1U);
		pmu_rank_slice_deskew_latch(0U, val_18);
		pmu_vref_signed_direction_pulse_dispatch(val_18, 0x1aU);

		tap_val = (iter < 8U) ? (s8)(-tap) : (s8)(tap & 7);

		fp_val = pmu_dbyte_coarse_step_mask_sweep(dq_swizzle_mask, 4U, ch);
		if (pmu_get_dmem_1c_bit(rank, ch) != 0U)
			fp_val |= (u8)pmu_dbyte_coarse_step_mask_sweep(slice_idx_other,
								 4U, ch);

		for (s = slice_start; s <= slice_end; s++) {
			u32 val = fp_val >> ((s == dq_swizzle_mask) ? 2U : 4U);
			u16 bit0 = (u16)(val & 1U);
			u16 bit1 = (u16)((val >> 1) & 1U);

			buf4[s][iter] = bit1;
			buf3[s][iter] = bit0;

			if (bit0 != prev_44[s]) {
				prev_44[s] = bit0;
				buf1[s][cnt_54[s]++] = (s16)tap_val;
			}

			if (bit1 != stat_3c[s]) {
				stat_3c[s] = bit1;
				buf2[s][cnt_4c[s]++] = (s16)tap_val;
			}
		}
	}

	for (s = slice_start; s <= slice_end; s++) {
		u16 b0 = buf3[s][14];
		u16 b1 = buf4[s][14];
		s16 sum1;
		s16 sum2;

		if (cnt_54[s] == 0U)
			buf1[s][0] = (b0 == 0U) ? 7 : -7;
		else if (b0 == 0U && cnt_54[s] != 1U)
			buf1[s][cnt_54[s]++] = 7;

		if (cnt_4c[s] == 0U)
			buf2[s][0] = (b1 == 0U) ? 7 : -7;
		else if (cnt_4c[s] >= 2U &&
			 ((flag_14 != 0U) == (b1 != 0U)))
			buf2[s][cnt_4c[s]++] = 7;

		sum1 = (cnt_54[s] >= 2U) ?
		       (s16)(buf1[s][0] + buf1[s][cnt_54[s] - 1U]) / 2 :
		       buf1[s][0];
		sum2 = (cnt_4c[s] >= 2U) ?
		       (s16)(buf2[s][0] + buf2[s][cnt_4c[s] - 1U]) / 2 :
		       buf2[s][0];
		res_34[s] = (s16)(sum1 + sum2) / 2;
	}

	val_other = res_34[slice_idx_other];
	val_swiz = res_34[dq_swizzle_mask];

	val_comb = (u8)(((fold_res(val_other) & 0x0f) << 4) |
			(fold_res(val_swiz) & 0x0f));

	pmu_rank_slice_deskew_latch(val_comb, val_18);

	*(__volatile__ u8 *)(PMU_DMEM_SAVED_CAL_STAT_B7E + ch_rank_idx) = val_comb;
	*(__volatile__ u8 *)(PMU_DMEM_WCK_PHASE_D2 + ch * 5U + rank * 10U) = val_comb;

	*(__volatile__ u16 *)reg_0e = orig_0e;
}

static __always_inline void
dly_line_tap_adjust_lane(u32 mode, u32 ptr_tag, u32 tag_valid,
			 u32 is_initial_mode, u32 csr_offset, u32 rank,
			 u32 slice, u8 step_val, u32 lane)
{
	u32 fp = (slice << 12) | (lane << 8) | csr_offset | 0x10000U;
	u32 tbl_offset = rank * 5280U + slice * 1320U + lane * 132U;
	__volatile__ const u8 *tbl_ptr =
		(__volatile__ const u8 *)(PMU_DMEM_LANE_PARAM_TBL_E48 + tbl_offset);
	u16 val_e48 = *(const __volatile__ u16 *)tbl_ptr;
	u8 cal_tap_status = *(__volatile__ const u8 *)
		(PMU_DMEM_CAL_TAP_STAT_378B + tbl_offset);
	s8 val_e48_offset2 = (s8)tbl_ptr[2];
	u8 cal_tap_base = tbl_ptr[3];
	u32 tap_step_target = (u32)(val_e48 + val_e48_offset2);
	u32 base_addr = (fp | rank) << 1;
	u32 idx_20 = slice * 20U + lane * 2U + rank;
	u32 idx_6098 = idx_20 << 1;
	u8 is_hi_freq = pmu_sdata()->high_freq_flag;

	if (mode != 0U) {
		u8 dly_buf_32[4] = { 0 };
		u8 dly_buf_30[4] = { 0 };

		if (ptr_tag == 0U && is_hi_freq != 0U) {
			u16 w_6098 = *(__volatile__ const u16 *)
				(PMU_DMEM_CBT_COEFF_6098 + idx_6098);

			pmu_unpack_byte_pair(w_6098, dly_buf_32);
		} else {
			u32 phy_addr = PHY_REG_AC_TIMING_0024 | base_addr;
			u16 raw_phy = *(__volatile__ const u16 *)phy_addr;

			pmu_cal_param_unpack(raw_phy, dly_buf_32);
		}

		pmu_unpack_byte_pair((u16)tap_step_target, dly_buf_30);
		if (tag_valid == 0U) {
			dly_buf_32[0] = dly_buf_30[0];
			dly_buf_32[1] = dly_buf_30[1];
		}

		if (is_hi_freq != 0U) {
			pmu_adjust_step_limit(dly_buf_32, ptr_tag);
			pmu_adjust_step_limit(dly_buf_30, ptr_tag);
		}

		if ((dmem_read8(PMU_DMEM_CAL_MISC_FLAGS) & 0x20U) != 0U) {
			int diff;

			pmu_adjust_step_limit(dly_buf_32, step_val);
			pmu_adjust_step_limit(dly_buf_30, step_val);

			diff = (int)dly_buf_32[0] - (int)dly_buf_30[0];
			if (diff > 1 || diff < -1) {
				u32 phy_addr = PHY_REG_AC_TIMING_0024 | base_addr;
				u32 step_adj = (step_val == 0U) ? 1U : 0U;
				u16 raw_phy = *(__volatile__ const u16 *)phy_addr;

				pmu_cal_param_unpack(raw_phy, dly_buf_32);
				pmu_unpack_byte_pair((u16)tap_step_target,
						     dly_buf_30);
				pmu_adjust_step_limit(dly_buf_32, step_adj);
				pmu_adjust_step_limit(dly_buf_30, step_adj);
			}
		}

		pmu_eye_margin_step_align(dly_buf_32, dly_buf_30,
					  (is_hi_freq != 0U) ? 2U : 1U);

		if (ptr_tag == 0U) {
			*(__volatile__ u16 *)(PMU_DMEM_CBT_COEFF_6098 + idx_6098) =
				(u16)pmu_unpack_6(dly_buf_32);
			*(__volatile__ u16 *)(PMU_DMEM_CBT_COEFF_6138 + idx_6098) =
				(u16)tap_step_target;
			*(__volatile__ u8 *)(PMU_DMEM_CBT_FLAGS_6228 + idx_20) =
				cal_tap_base;
			*(__volatile__ u8 *)(PMU_DMEM_CBT_FLAGS_62C8 + idx_20) =
				cal_tap_status;
		}

		if (!(ptr_tag == 0U && is_hi_freq != 0U)) {
			*(__volatile__ u16 *)(PHY_REG_AC_TIMING_0024 | base_addr) =
				(u16)pmu_unpack_7(dly_buf_32);
		}

		*(__volatile__ u16 *)(PHY_REG_AC_TIMING_0020 | base_addr) =
			(u16)pmu_unpack_7(dly_buf_30);
	} else {
		u8 dly_buf_34[4] = { 0 };

		if (ptr_tag == 0U) {
			*(__volatile__ u8 *)(PMU_DMEM_CBT_FLAGS_61D8 + idx_20) =
				cal_tap_base;
			*(__volatile__ u16 *)(PMU_DMEM_CBT_COEFF_6098 + idx_6098) =
				(u16)tap_step_target;
			*(__volatile__ u8 *)(PMU_DMEM_CBT_FLAGS_6278 + idx_20) =
				cal_tap_status;
		}

		pmu_unpack_byte_pair((u16)tap_step_target, dly_buf_34);

		if (is_hi_freq != 0U)
			pmu_adjust_step_limit(dly_buf_34, ptr_tag);

		if ((dmem_read8(PMU_DMEM_CAL_MISC_FLAGS) & 0x20U) != 0U)
			pmu_adjust_step_limit(dly_buf_34, step_val);

		*(__volatile__ u16 *)(PHY_REG_AC_TIMING_0024 | base_addr) =
			(u16)pmu_unpack_7(dly_buf_34);
	}

	if (dmem_read8(PMU_DMEM_CAL_OVERRIDE_FLAG) == 0U) {
		u32 num_t = (pmu_phy_query_operating_mode() == 3U) ? 2U : 1U;
		u32 t;

		for (t = 0U; t < num_t; t++) {
			u8 val = (t != 0U) ? cal_tap_status : cal_tap_base;
			u32 off = pmu_dly_reg_offset_select(mode, t, rank);

			*(__volatile__ u16 *)
				(PHY_REG_APB_BASE | ((fp | off) << 1)) = val;
			if ((is_initial_mode | tag_valid) == 0U) {
				off = pmu_dly_reg_offset_select(0U, t, rank);
				*(__volatile__ u16 *)
					(PHY_REG_APB_BASE | ((fp | off) << 1)) = val;
			}
		}
	}
}

/**
 * pmu_cal_dly_line_tap_step_adjust() - Adjust per-lane delay line taps across active slices
 * @mode: Operating mode selector (0 = initial, non-zero = tracking)
 * @ptr_tag: Pointer/stage tag selector
 * @lane_mask: Active 9-bit lane mask
 * @check_tag: Non-zero to validate @ptr_tag via pmu_check_dmem_407_flag
 */
static void pmu_cal_dly_line_tap_step_adjust(u32 mode, u32 ptr_tag, u32 lane_mask,
					     u32 check_tag)
{
	u32 tag_valid = (check_tag != 0U) ?
			(pmu_check_dmem_407_flag() == 0U) :
			1U;
	u32 is_initial_mode = (mode == 0U) ? 1U : 0U;
	u32 csr_offset = pmu_sdata()->active_csr_offset;
	u32 rank;

	for (rank = 0U; rank < 2U; rank++) {
		u8 rank_en = dmem_read8(PMU_DMEM_CH0_RANK_EN) |
			     dmem_read8(PMU_DMEM_CH1_RANK_EN);
		u8 slice_start;
		u8 slice_end;
		u32 slice;

		if ((rank_en & (1U << rank)) == 0U)
			continue;

		slice_start = dmem_read8(PMU_DMEM_SLICE_START);
		slice_end = dmem_read8(PMU_DMEM_SLICE_END);
		for (slice = slice_start; slice <= slice_end; slice++) {
			u8 step_val;
			u32 lane;

			if ((dmem_read8(PMU_DMEM_ACTIVE_SLICE_MASK_CAL) &
			     (1U << slice)) == 0U)
				continue;

			step_val = pmu_sdata()->slice_desc_474[rank * 4U + slice];

			for (lane = 0U; lane < 9U; lane++) {
				if ((lane_mask & (1U << lane)) == 0U)
					continue;

				dly_line_tap_adjust_lane(mode, ptr_tag,
							 tag_valid,
							 is_initial_mode,
							 csr_offset, rank,
							 slice, step_val, lane);
			}
		}
	}
}

#pragma GCC pop_options

/**
 * pmu_cal_dbyte_deskew_pin_results_apply() - Program per-pin deskew results to DBYTE CSRs
 */
static void pmu_cal_dbyte_deskew_pin_results_apply(void)
{
	uintptr_t base_e48 = PMU_DMEM_LANE_PARAM_TBL_E48;
	uintptr_t base_e4a = PMU_DMEM_LANE_PARAM_TBL_E4A;
	u32 csr_offset = pmu_sdata()->active_csr_offset;
	u32 rank;

	for (rank = 0U; rank < 2U; rank++) {
		u8 mask = dmem_read8(PMU_DMEM_CH0_RANK_EN) |
			  dmem_read8(PMU_DMEM_CH1_RANK_EN);
		u32 start_slice;
		u32 end_slice;
		uintptr_t slice_e48;
		uintptr_t slice_e4a;
		u32 slice;

		if ((mask & (1U << rank)) == 0U) {
			base_e48 += 5280U;
			base_e4a += 5280U;
			continue;
		}

		start_slice = dmem_read8(PMU_DMEM_SLICE_START);
		end_slice = dmem_read8(PMU_DMEM_SLICE_END);

		slice_e48 = base_e48 + start_slice * 1320U;
		slice_e4a = base_e4a + start_slice * 1320U;

		for (slice = start_slice; slice <= end_slice; slice++) {
			if ((dmem_read8(PMU_DMEM_ACTIVE_SLICE_MASK_CAL) &
			     (1U << slice)) != 0U) {
				u32 slice_base = csr_offset | (slice << 12) |
						 rank;
				uintptr_t ptr_a = slice_e48;
				uintptr_t ptr_b = slice_e4a;
				u32 pin;

				for (pin = 0U; pin < 10U; pin++) {
					u16 val_a = *(__volatile__ const u16 *)ptr_a;
					s8 val_b = *(__volatile__ const s8 *)ptr_b;
					u16 sum = (u16)(val_a + (s32)val_b);

					ptr_a += 132U;
					ptr_b += 132U;

					if (pin == 9U) {
						uintptr_t reg =
							(PHY_REG_DBYTE_BASE |
							 0x0050UL) |
							(slice_base << 1);

						*(__volatile__ u16 *)reg = sum;
					} else {
						uintptr_t reg =
							(PHY_REG_DBYTE_BASE |
							 0x004cUL) |
							((slice_base |
							  (pin << 8)) << 1);

						*(__volatile__ u16 *)reg = sum;
					}
				}

				if (dmem_read8(PMU_DMEM_CAL_OVERRIDE_CE) == 0U) {
					uintptr_t tbl = PMU_DMEM_LANE_PARAM_TBL_E48 +
							rank * 5280U +
							slice * 1320U;
					u8 cal_step = *(__volatile__ const u8 *)
						(tbl + 3U);
					u32 is_hi = (slice >
						     dmem_read8(PMU_DMEM_DESKEW_DIR_B6B)) ?
						    1U : 0U;
					u32 swap;
					u32 rank_idx;

					pmu_phy_mode_cfg_dispatch(is_hi ?
								  2U : 1U);
					swap = pmu_cal_lane_error_status_check(rank,
									       slice);
					rank_idx = rank + (is_hi ? 2U : 0U);

					if (swap != 0U) {
						dmem_write8(PMU_DMEM_DESKEW_SWAP_B72 + rank_idx,
							    cal_step);
						pmu_cal_pulse_seq_coordinator(is_hi,
									      rank,
									      cal_step,
									      1U);
						dmem_write8(is_hi ?
							    (rank ? 0x51U : 0x50U) :
							    (rank ? 0x36U : 0x35U),
							    cal_step);
					} else {
						dmem_write8(PMU_DMEM_DESKEW_NORM_B76 + rank_idx,
							    cal_step);
						pmu_cal_pulse_seq_coordinator(is_hi,
									      rank,
									      cal_step,
									      2U);
						dmem_write8(is_hi ?
							    (rank ? 0xdfU : 0xdaU) :
							    (rank ? 0xd5U : 0xd0U),
							    cal_step);
					}
					pmu_phy_mode_cfg_dispatch(3U);
				}
			}

			slice_e48 += 1320U;
			slice_e4a += 1320U;
		}

		base_e48 += 5280U;
		base_e4a += 5280U;
	}
}

/**
 * pmu_cal_bist_cmd_strobe_dispatch() - Configure BIST command context and dispatch pulse sequence
 * @ctx_addr: Address of struct pmu_cal_bist_cmd_ctx
 * @pattern_mode: BIST pattern mode index
 * @cmd_mode: Command mode (0..3)
 * @target_mask: Target channel/rank mask
 * @slice_flags_addr: Address of 4-element u16 slice flags array
 * @status_idx: Status query index
 * @setup_flags: Pattern setup flags
 */
__attribute__((optimize("no-jump-tables")))
noinline void pmu_cal_bist_cmd_strobe_dispatch(uintptr_t ctx_addr,
					       u32 pattern_mode, u32 cmd_mode,
					       u32 target_mask,
					       uintptr_t slice_flags_addr,
					       u32 status_idx, u32 setup_flags)
{
	struct pmu_cal_bist_cmd_ctx *ctx =
		(struct pmu_cal_bist_cmd_ctx *)ctx_addr;
	u32 target_mask_reg = target_mask;
	const u16 *slice_flags = (const u16 *)slice_flags_addr;
	__volatile__ u16 *bcast;
	const __volatile__ u8 *b68;
	u32 status;
	u32 csr_offset;
	u32 step_param;
	u32 cmd_r1;
	u8 slice_start;
	u8 slice_end;
	u8 slice_mask;
	u16 stride;
	bool flag;
	u32 k;
	u32 slice;

	status = pmu_cal_pad_voltage_dac_set(status_idx, cmd_mode);

	if (pmu_sdata()->freq_mode <= 1U)
		csr_offset = pmu_sdata()->active_csr_offset | 0x14U;
	else
		csr_offset = 0x56U;

	ctx->field_00 = 0U;
	ctx->field_04 = 0U;
	ctx->field_08 = 1U;
	ctx->csr_offset = csr_offset;

	flag = (target_mask & 0x5U) != 0U;

	if (cmd_mode == 3U) {
		ctx->cmd_c = flag ? 0x28U : 0x29U;
	} else if (cmd_mode == 1U) {
		ctx->cmd_10 = flag ? 16U : 17U;
		ctx->cmd_c = flag ? 18U : 19U;
	} else if (cmd_mode == 2U) {
		ctx->cmd_c = flag ? 0x26U : 0x27U;
	} else if (cmd_mode == 0U) {
		ctx->cmd_10 = flag ? 18U : 19U;
		ctx->cmd_c = flag ? 16U : 17U;
	} else {
		pmu_assert_or_halt(0U, PMU_ERR_BIST_CMD_MODE_INVALID);
		return;
	}

	phy_write16(PHY_REG_RESET_TRIGGER, 0x40U);
	phy_write16(PHY_REG_DBYTE_BCAST_BASE | (csr_offset << 1), 1U);
	phy_write16(PHY_REG_DBYTE_BCAST_RX_EN, 0U);

	pmu_cal_bist_pattern_setup(pattern_mode, target_mask_reg, status,
				   setup_flags);
	pmu_set_458_pair(800U);

	step_param = (dmem_read8(PMU_DMEM_DRAM_TYPE) == 2U) ? 4U : 2U;

	if (pattern_mode < 9U && ((1U << pattern_mode) & 0x105U)) {
		pmu_cal_sequence_pulse_send((1UL << 15), 7U, 0x18U, 0U, 0U, 0U, 0U);
		pmu_flag_45c_set();
		pmu_cal_sequence_pulse_send(0U, 7U, 0U, 0U, 0U,
					    target_mask_reg, 0U);
		pmu_cal_sequence_pulse_send(0x100000UL, 5U, 0U, 0x80U, 0U,
					    target_mask_reg, 0U);
		pmu_cal_sequence_pulse_send((1UL << 23), 0x29U, step_param, 0U,
					    0U, target_mask_reg, 7U);
		pmu_cal_sequence_pulse_send(0U, 7U, pmu_sdata()->cbt_dly_init,
					    0U, 0U, target_mask_reg, 0U);
		cmd_r1 = 0x2aU;
	} else {
		pmu_cal_sequence_pulse_send((1UL << 15), 7U, 0x28U, 0U, 0U, 0U, 0U);
		pmu_flag_45c_set();
		pmu_cal_sequence_pulse_send(0U, 7U, 8U, 0U, 0U,
					    target_mask_reg, 0U);
		cmd_r1 = 0x2bU;
	}

	pmu_cal_sequence_pulse_send(0U, 7U, 0U, 0U, 0U, target_mask_reg, 0U);
	pmu_cal_sequence_pulse_send(0x200000UL, 5U, 0U, 256U, 0U,
				    target_mask_reg, 0U);

	pmu_cal_sequence_pulse_send(0x01040000UL, cmd_r1, step_param, 0U, 0U,
				    target_mask_reg, 7U);
	pmu_cal_sequence_pulse_send(0U, 7U, 4U, 0U, 0U, target_mask_reg, 10U);
	pmu_flag_45c_clear();
	for (k = 0U; k < 2U; k++)
		pmu_cal_sequence_pulse_send(0U, 7U, 0U, 0U, 0U, 0U, 0U);

	stride = pmu_cal_stride_get();

	bcast = (__volatile__ u16 *)PHY_REG_DBYTE_BCAST_DQ_DLY0;
	b68 = (const __volatile__ u8 *)PMU_DMEM_CH_RANGE_B68;
	bcast[3] = 1U;
	bcast[4] = 1U;
	bcast[0] = (dmem_read8(PMU_DMEM_DRAM_TYPE) == 4U) ? 0x34U : 0x30U;
	ctx->stride_minus_1 = stride - 1U;
	bcast[1] = 1U;

	slice_start = b68[0];
	slice_end = b68[1];
	slice_mask = b68[0x30];

	for (slice = slice_start; slice <= slice_end; slice++) {
		u16 val = 0U;

		if (!(slice_mask & (1U << slice)))
			continue;

		if (cmd_mode <= 1U)
			val = (((slice_flags[slice] >> 7) & 1U) == cmd_mode) ?
			      10U : 5U;

		ctx->slice_delays[slice] = val;
	}
}

/**
 * pmu_dbyte_dq_commit_rank_steps() - Save eye tables and commit multi-rank DQ steps
 * @channel_idx: Target channel index (0 or 1)
 * @rank_idx: Target rank index (0 or 1)
 * @cfg_mode_bit0: Bit 0 of the channel configuration mode byte
 * @arg0_x5: Replicated rank mask ((mode_rank << 2) | mode_rank)
 */
static void pmu_dbyte_dq_commit_rank_steps(u32 channel_idx, u32 rank_idx,
					   u8 cfg_mode_bit0, u32 arg0_x5)
{
	u32 struct_base = PMU_DMEM_BASE | PMU_DMEM_CAL_STRUCT_BASE;
	void *cal_struct = (void *)(uintptr_t)struct_base;
	u32 not_cfg_mode_bit0 = !cfg_mode_bit0;
	u32 table_addr;
	u32 a3_mask;

	pmu_cal_eye_table_save(cal_struct, 0x10U);
	pmu_cal_lane_mask_or_all(struct_base, 0x10U, 0x41U);
	pmu_cal_lane_mask_and_all(struct_base, 0x10U, 0xf3U);
	pmu_cal_eye_table_save(cal_struct, 0x0bU);
	pmu_cal_lane_mask_and_all(struct_base, 0x0bU, 0xf0U);
	pmu_cal_eye_table_save(cal_struct, 0x12U);
	pmu_cal_lane_mask_or_all(struct_base, 0x12U, 0x08U);
	pmu_cal_lane_mask_and_all(struct_base, 0x12U, 0xf8U);
	pmu_set_458_pair(1200U);
	pmu_cbt_cal_pulse_seq(0U);
	dmem_write16(PMU_DMEM_DESKEW_TARGET_DELAY, pmu_cal_stride_get() - 1U);
	if (dmem_read8(PMU_DMEM_FREQ_MODE) != 0U) {
		pmu_cal_pad_timing_step_settle();
		pmu_delay_us(5000U, 12U);
		pmu_clk_gate_handoff();
	}
	pmu_cal_lane_mask_or_all(struct_base, 0x0dU, 0x40U);

	table_addr = struct_base +
		     (channel_idx * 2U + rank_idx) * PMU_SLICE_DESC_SIZE;
	if (rank_idx == not_cfg_mode_bit0) {
		u8 rank_mask;

		pmu_cal_eye_table_save(cal_struct, 0x11U);
		pmu_cal_lane_mask_or_all(struct_base, 0x11U, 0x08U);
		pmu_cal_lane_mask_and_all(struct_base, 0x11U, 0xefU);
		pmu_cal_lane_mask_or_all(struct_base, 0x11U, 0x20U);

		rank_mask = (u8)((1U << (rank_idx + 2U)) | (1U << rank_idx));
		pmu_cal_multi_rank_step_commit(rank_mask, table_addr, 0U,
					       0xfffdffffUL, (u32)-1, 0U, 0U);
		pmu_delay_us(2000U, 0U);
		pmu_cal_eye_table_restore((u8 *)struct_base, 0x11U);
		a3_mask = 0xfffbdff7UL;
	} else {
		a3_mask = 0xfff9d7f7UL;
	}

	pmu_cal_multi_rank_step_commit((u8)arg0_x5, table_addr, 0U,
				       a3_mask, (u32)-1, 0U, 0U);
	pmu_cal_lane_mask_and_all(struct_base, 0x10U, 0xf3U);
	pmu_cal_lane_mask_or_all(struct_base, 0x10U, 0x61U);
	pmu_cal_lane_mask_and_all(struct_base, 0x10U, 0x7fU);
}

/**
 * pmu_dbyte_dq_settle_rx_bias() - Settle DBYTE receiver bias and pad timing across slices
 * @slice_list: Array of active slice indices for the rank
 * @num_slices: Number of active slices in @slice_list
 */
static void pmu_dbyte_dq_settle_rx_bias(const u8 *slice_list, u8 num_slices)
{
	u32 iter, i;

	if (dmem_read8(PMU_DMEM_FREQ_MODE) != 0U) {
		for (i = 0U; i < num_slices; i++) {
			u8 slice = slice_list[i];
			u32 mask = pmu_cal_eye_margin_sample_latch(0x80U, slice);

			phy_write16((PHY_REG_DBYTE_BASE | 0x0118UL) |
				    ((u32)slice << 13), (u16)mask);
		}
		pmu_delay_us(5000U, 12U);
		pmu_phy_profile_param_program();
		return;
	}

	for (iter = 0U; iter < 20U; iter++) {
		for (i = 0U; i < num_slices; i++) {
			u32 reg = (PHY_REG_DBYTE_BASE | 0x0118UL) |
				  ((u32)slice_list[i] << 13);

			phy_write16(reg, 0x800U);
			phy_write16(reg, 0U);
		}
	}

	for (iter = 0U; iter < 40U; iter++) {
		for (i = 0U; i < num_slices; i++) {
			u8 slice = slice_list[i];
			u32 reg = (PHY_REG_DBYTE_BASE | 0x0118UL) |
				  ((u32)slice << 13);
			u32 mask = pmu_cal_eye_margin_sample_latch(0x80U, slice);

			phy_write16(reg, (u16)(mask | 0x800U));
			phy_write16(reg, (u16)(mask | 0x1000U));
		}
	}

	for (i = 0U; i < num_slices; i++) {
		u32 slice_off = (u32)slice_list[i] << 13;

		phy_write16((PHY_REG_DBYTE_BASE | 0x0112UL) | slice_off, 511U);
		phy_write16((PHY_REG_DBYTE_BASE | 0x0116UL) | slice_off, 511U);
		phy_write16((PHY_REG_DBYTE_BASE | 0x0118UL) | slice_off,
			    (u16)pmu_cal_eye_margin_sample_latch(0x80U,
								 slice_list[i]));
	}

	pmu_clk_gate_handoff();
	pmu_phy_profile_param_program();
	pmu_cal_pad_timing_step_settle();
	pmu_delay_us(5000U, 12U);
	pmu_clk_gate_handoff();
	pmu_delay_us(0x3d090UL, 0U);
	pmu_cal_pad_timing_step_settle();
}

/**
 * pmu_cal_dbyte_dq_status_check() - Configure DBYTE DQ pins and check training status (0x5014)
 * @channel_idx: Target channel index (0 or 1)
 * @mode_rank: Encoded mode and rank index
 * @flag: Non-zero for full struct shadow and multi-rank step commit
 *
 * Return: @mode_rank unchanged.
 */
static noinline u32 pmu_cal_dbyte_dq_status_check(u32 channel_idx, u32 mode_rank,
						  u32 flag)
{
	u32 rank_idx = ((mode_rank & 3U) == 2U) ? 1U : 0U;
	u32 csr_offset = dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET);
	u32 csr_offset_shl1 = csr_offset << 1;
	u8 cfg_mode = dmem_read8(channel_idx ? 0x4dU : 0x32U);
	u8 cfg_mode_bit0 = cfg_mode & 1U;
	u16 reg_000e;
	u8 *slice_list;
	u8 num_slices;
	u32 i, arg0_x5, p;
	u8 param_9b0;

	phy_write16(PHY_REG_VREF_CTRL | csr_offset_shl1, 1U);
	phy_write16(PHY_REG_VREF_TRIM_CFG, 1U);
	reg_000e = phy_read16(PHY_REG_MASTER_CFG | csr_offset_shl1);
	phy_write16(PHY_REG_MASTER_CFG | csr_offset_shl1, reg_000e | 0x100U);

	slice_list = (u8 *)(uintptr_t)((PMU_DMEM_BASE | PMU_DMEM_RANK_SLICE_MAP) +
				       channel_idx * 16U + rank_idx * 8U);
	num_slices = dmem_read8(PMU_DMEM_RANK_SLICE_COUNTS +
				channel_idx * 2U + rank_idx);

	for (i = 0U; i < num_slices; i++) {
		u32 slice_off = (u32)slice_list[i] << 13;

		if (dmem_read8(PMU_DMEM_FREQ_MODE) != 0U) {
			phy_write16((PHY_REG_DBYTE_BASE | 0x0112UL) | slice_off, 511U);
			phy_write16((PHY_REG_DBYTE_BASE | 0x0116UL) | slice_off, 511U);
			phy_write16((PHY_REG_DBYTE_BASE | 0x0118UL) | slice_off, 0U);
		} else {
			phy_write16((PHY_REG_DBYTE_BASE | 0x0112UL) | slice_off, 0x19ffU);
			phy_write16((PHY_REG_DBYTE_BASE | 0x0116UL) | slice_off, 0x19ffU);
			phy_write16((PHY_REG_DBYTE_BASE | 0x0118UL) | slice_off, 0x1000U);
		}
	}

	arg0_x5 = (mode_rank << 2) | mode_rank;

	if (flag != 0U) {
		pmu_dbyte_dq_commit_rank_steps(channel_idx, rank_idx,
					       cfg_mode_bit0, arg0_x5);
	} else {
		u32 struct_base = PMU_DMEM_BASE | PMU_DMEM_CAL_STRUCT_BASE;

		pmu_cal_lane_mask_and_all(struct_base, 0x10U, 0xf3U);
		pmu_cal_lane_mask_or_all(struct_base, 0x10U, 0x61U);
		pmu_cal_lane_mask_or_all(struct_base, 0x10U, 0x80U);
	}

	phy_write16(PHY_REG_PLL_CLK_CTRL, 1U);
	if (cfg_mode_bit0 == rank_idx) {
		u32 table_addr = (PMU_DMEM_BASE | PMU_DMEM_CAL_STRUCT_BASE) +
				 (channel_idx * 2U + rank_idx) *
				 PMU_SLICE_DESC_SIZE;

		pmu_cbt_3phase_pulse_dispatch((u8)arg0_x5, (const u8 *)table_addr, 16U, 1U);
	} else {
		u32 dq_offset_sub = ((arg0_x5 & 3U) == 2U) ?
				    PMU_SLICE_DESC_SIZE : 0U;
		u8 *tbl = (u8 *)(uintptr_t)((PMU_DMEM_BASE | PMU_DMEM_CAL_STRUCT_BASE) +
					    channel_idx * (2U * PMU_SLICE_DESC_SIZE) +
					    dq_offset_sub);
		u8 val10 = tbl[0x10];
		u8 dq_pin_mask = (u8)((1U << (cfg_mode_bit0 + 2U)) | (1U << cfg_mode_bit0));

		pmu_cal_strobe_pulse_init();
		pmu_ac_lane_profile_setup();
		pmu_cal_margin_dir_get();
		pmu_dbyte_cal_step_latch(0x10U, val10, (u8)arg0_x5);
		pmu_cal_pad_timing_delay_set(0U, 4U, 1U);
		pmu_dbyte_cal_step_latch(0x10U, (val10 & 138U) | 69U, dq_pin_mask);
		pmu_cal_pad_timing_delay_set(0U, 0x20U, 1U);
		pmu_cbt_cal_pulse_seq(0U);
		pmu_cbt_3phase_pulse_seq();
		pmu_cal_slice_step_diff_latch(0U, 0U);
		pmu_delay_us(0U, 0x32U);
	}

	pmu_dbyte_dq_settle_rx_bias(slice_list, num_slices);

	param_9b0 = dmem_read8(PMU_DMEM_CAL_STRUCT_BASE +
			       (channel_idx * 2U + rank_idx) *
			       PMU_SLICE_DESC_SIZE + 0xcU);
	p = (param_9b0 | 0x80U) & 0xffU;

	for (i = 0U; i < num_slices; i++) {
		u8 slice = slice_list[i];
		u32 slice_off = (u32)slice << 13;
		u32 mask = pmu_cal_eye_margin_sample_latch(p, slice);

		phy_write16((PHY_REG_DBYTE_BASE | 0x0118UL) | slice_off,
			    (u16)(mask & 0xffU));
	}
	pmu_delay_us(5000U, 12U);

	for (i = 0U; i < num_slices; i++) {
		u8 slice = slice_list[i];
		u32 slice_off = (u32)slice << 13;
		u32 mask = pmu_cal_eye_margin_sample_latch(p, slice);

		phy_write16((PHY_REG_DBYTE_BASE | 0x0118UL) | slice_off,
			    (u16)((mask & 0xffU) | 0x100U));
	}
	pmu_delay_us(5000U, 12U);

	for (i = 0U; i < num_slices; i++) {
		u32 slice_off = (u32)slice_list[i] << 13;
		u32 mask = pmu_cal_eye_margin_sample_latch(0x80U, slice_list[i]) | 0x100U;

		phy_write16((PHY_REG_DBYTE_BASE | 0x0112UL) | slice_off, (u16)mask);
		phy_write16((PHY_REG_DBYTE_BASE | 0x0116UL) | slice_off, (u16)mask);
	}

	pmu_delay_us(0x3d090UL, 0U);
	pmu_cal_eye_table_restore((u8 *)(PMU_DMEM_BASE | PMU_DMEM_CAL_STRUCT_BASE), 0x10U);
	pmu_clk_gate_handoff();

	return mode_rank;
}

/**
 * pmu_cal_dbyte_pattern_loop_exec() - Compute slice delay bounds and unpack pattern taps (0x5770)
 * @reg_offset: Channel/register offset index
 * @pattern_mode: Pattern mode (0..3)
 * @dest_buf: Output 4-element u16 array of decoded 15-bit pattern addresses
 * @scratch_buf: Output 4-element u16 array of raw slice delay bounds
 * @bound_start: Step adjust flag for high-frequency mode
 * @bound_end: Non-zero to average min/max across active lanes
 * @flags: Base delay offset selector (0 = 0x40, non-zero = 0x80)
 */
noinline void pmu_cal_dbyte_pattern_loop_exec(u32 reg_offset, u16 pattern_mode,
					      void *dest_buf, void *scratch_buf,
					      u32 bound_start, u32 bound_end,
					      u32 flags)
{
	u16 *out_dest = (u16 *)dest_buf;
	u16 *out_scratch = (u16 *)scratch_buf;
	u8 slice_start = dmem_read8(PMU_DMEM_SLICE_START);
	u8 slice_end = dmem_read8(PMU_DMEM_SLICE_END);
	u8 active_mask = dmem_read8(PMU_DMEM_ACTIVE_SLICE_MASK_CAL);
	u16 freq;
	u32 slice;

	if (pattern_mode == 2U || pattern_mode == 3U) {
		u16 local_buf[4] = { 0 };
		u32 csr_offset;

		pmu_cal_slice_step_diff_commit(reg_offset, local_buf, 1U);
		csr_offset = pmu_sdata()->active_csr_offset;
		for (slice = slice_start; slice <= slice_end; slice++) {
			u32 addr = PHY_REG_DBYTE_BASE |
				(((slice << 12) | csr_offset | (reg_offset + 0x2aU)) << 1);
			u16 reg = phy_read16(addr);
			u16 sum = (reg & 0x3fU) + local_buf[slice];
			u16 val0 = sum + 0x20U;
			u16 res = 0U;

			if ((val0 & ~1U) >= 0x6aU)
				res = sum - 0x4aU;
			out_scratch[slice] = res;
			out_dest[slice] = res;
		}
		return;
	}

	if (pattern_mode >= 2U) {
		pmu_assert_or_halt(0U, PMU_ERR_BIST_PATTERN_MODE_RANGE);
		return;
	}

	freq = dmem_read16(PMU_DMEM_DRAM_FREQ_OFF);
	if (freq <= 1599U) {
		u16 init_val = (pmu_sdata()->freq_mode < 2U) ? 0x41U : 0xc1U;

		for (slice = slice_start; slice <= slice_end; slice++) {
			if (active_mask & (1U << slice))
				out_scratch[slice] = init_val;
		}
	} else {
		u32 base_reg = (pattern_mode != 0U) ?
			       (PHY_REG_DBYTE_BASE | 0x0024UL) :
			       (PHY_REG_DBYTE_BASE | 0x0020UL);
		u8 base_fp = (flags == 0U) ? 0x40U : 0x80U;
		u32 num_lanes = (pmu_dmem_training_flags_eval() != 0U) ? 9U : 8U;
		u32 csr_offset = pmu_sdata()->active_csr_offset;

		for (slice = slice_start; slice <= slice_end; slice++) {
			u32 dmem_addr = 0x12eU;
			u8 sub_val;

			if (!(active_mask & (1U << slice)))
				continue;

			if (bound_end != 0U) {
				u16 min_val = 0xffffU;
				u16 max_val = 0U;
				u32 slice_base = (slice << 12) | csr_offset |
						 reg_offset;
				u32 lane;

				for (lane = 0U; lane < num_lanes; lane++) {
					u32 reg_addr = base_reg |
						((slice_base | (lane << 8)) << 1);
					u16 reg_val = phy_read16(reg_addr);
					u16 val = (reg_val & 0x7fU) +
						  ((reg_val & ~0x7fU) >> 1);

					if (val < min_val)
						min_val = val;
					if (val > max_val)
						max_val = val;
				}
				out_scratch[slice] = (min_val + max_val) >> 1;
			} else {
				out_scratch[slice] =
					pmu_phy_delay_bound_margin_log(slice);
			}

			if (reg_offset == 0U) {
				if (slice <= 3U)
					dmem_addr = 0x127U + slice;
			} else if (reg_offset == 1U) {
				if (slice <= 2U)
					dmem_addr = 0x12bU + slice;
			}

			sub_val = base_fp + dmem_read8(dmem_addr);
			if (dmem_read8(PMU_DMEM_CAL_MISC_FLAGS) & 0x20U) {
				u8 pattern_bytes[4];
				u32 dev;
				u8 flag;

				pmu_unpack_byte_pair(out_scratch[slice],
						     pattern_bytes);
				dev = pmu_midpoint_deviation_calc(pattern_bytes[1]);
				flag = ((dev < 0x21U) ^
					(pattern_bytes[0] & 1U)) ? 1U : 0U;
				pmu_sdata()->slice_desc_474[slice +
							    (reg_offset << 2)] =
					flag;
			}

			if (out_scratch[slice] < sub_val)
				out_scratch[slice] = 0U;
			else
				out_scratch[slice] -= sub_val;
		}
	}

	for (slice = slice_start; slice <= slice_end; slice++) {
		u8 pattern_bytes[4];

		if (!(active_mask & (1U << slice)))
			continue;

		pmu_unpack_byte_pair(out_scratch[slice], pattern_bytes);

		if (pmu_sdata()->high_freq_flag != 0U)
			pmu_adjust_step_limit(pattern_bytes, bound_start);

		if (dmem_read8(PMU_DMEM_CAL_MISC_FLAGS) & 0x20U)
			pmu_adjust_step_limit(pattern_bytes,
					      pmu_sdata()->slice_desc_474[slice +
									  (reg_offset << 2)]);

		out_dest[slice] = (u16)pmu_unpack_7(pattern_bytes);
	}
}

/**
 * pmu_slice_reg_pulse_write() - Write a pulse control value across a list of DBYTE slices
 * @ptr: Array of target DBYTE slice indices
 * @count: Number of slice indices in @ptr
 * @val: 16-bit value to write to register offset 0x0118 of each slice
 */
static inline void pmu_slice_reg_pulse_write(const u8 *ptr, u32 count, u16 val)
{
	while (count--)
		phy_write16((PHY_REG_DBYTE_BASE | 0x0118UL) |
			    ((u32)(*ptr++) << 13), val);
}

/**
 * pmu_cal_multi_rank_deskew_sweep() - Execute multi-rank deskew pulse train and step commit
 * @channel: Target channel index (0 or 1)
 * @mode: Encoded rank/mode selector
 */
static noinline void pmu_cal_multi_rank_deskew_sweep(u32 channel, u32 mode)
{
	u32 struct_base =
		PMU_DMEM_BASE | PMU_DMEM_CAL_STRUCT_BASE;
	const u8 *ptr =
		(const u8 *)(PMU_DMEM_BASE | PMU_DMEM_RANK_SLICE_COUNTS);
	u32 count;
	u8 cal_cfg;
	u32 mode_flag = ((mode & 3U) == 2U) ? 1U : 0U;
	u32 idx;
	u32 csr_offset;
	u32 reg;

	cal_cfg = dmem_read8(channel ? PMU_DMEM_CH1_RANK_CFG :
				       PMU_DMEM_CH0_RANK_CFG);
	if (mode_flag != (cal_cfg & 1U))
		pmu_slice_deskew_pulse_train(channel, (u32)(cal_cfg & 1U), 1U);

	if (pmu_sdata()->freq_mode != 0U) {
		pmu_ac_lane_profile_setup();
		pmu_delay_us(50000U, 3U);
	}

	pmu_cal_eye_table_restore((u8 *)struct_base, 0x0bU);
	pmu_cal_eye_table_restore((u8 *)struct_base, 0x11U);
	pmu_cal_eye_table_restore((u8 *)struct_base, 0x12U);

	idx = mode_flag + (channel * 2U);
	count = ptr[idx];
	ptr += 0x24 + (idx << 3);

	pmu_cal_pad_timing_step_settle();

	pmu_slice_reg_pulse_write(ptr, count, 256U);

	pmu_delay_us(5000U, 12U);
	pmu_clk_gate_handoff();

	csr_offset = pmu_sdata()->active_csr_offset;
	phy_write16(PHY_REG_VREF_CTRL | (csr_offset << 1), 0U);
	pmu_delay_us(5000U, 0U);

	pmu_cal_eye_table_save((void *)(uintptr_t)(struct_base), 0x10U);
	pmu_cal_lane_mask_and_all(struct_base, 0x10U, 0xcfU);
	pmu_cal_lane_mask_and_all(struct_base, 0x10U, 0xffU);

	pmu_cal_multi_rank_step_commit(mode,
				       struct_base + (idx * PMU_SLICE_DESC_SIZE),
				       0U, 0xfffeffffUL, (u32)-1, 1U, 0U);

	pmu_delay_us(14000U, 10U);

	pmu_slice_reg_pulse_write(ptr, count, 0x1000U);

	pmu_cal_eye_table_restore((u8 *)struct_base, 0x10U);
	reg = PHY_REG_MASTER_CFG | (pmu_sdata()->active_csr_offset << 1);
	phy_write16(reg, phy_read16(reg) & 0xfeffU);
	pmu_hw_timer_delay(0x14U);
}

/**
 * pmu_cal_channel_rank_boundary_eval() - Evaluate channel/rank boundary delays and unpack CSRs
 * @buf: 584-byte work buffer receiving slice bounds and diagnostic CSR snapshots
 * @channel: Target channel index (0 or 1)
 * @rank: Target rank index
 * @cal_type: Calibration stage type (0x11 or 0x12)
 */
static noinline void pmu_cal_channel_rank_boundary_eval(u8 *buf, u32 channel, u32 rank,
							u32 cal_type)
{
	u8 *ctrl = buf + 512;
	u32 lut_val;
	u8 fp_val;
	u8 rank_start;
	u8 rank_end;
	u32 active_csr_offset;
	u32 i;

	lut_val = pmu_bit_lookup_e4(channel, channel);
	memset(buf, 0, 526U);

	fp_val = (rank != 0U) ? 9U : 8U;
	rank_start = dmem_read8(PMU_DMEM_SLICE_START);
	rank_end = dmem_read8(PMU_DMEM_SLICE_END);
	active_csr_offset = pmu_sdata()->active_csr_offset;

	if (cal_type == 0x11U) {
		u8 dmem_102 = dmem_read8(PMU_DMEM_TRAIN_FEATURE_MASK);

		ctrl[12] = 0U;
		*(u16 *)&ctrl[8] = 0U;
		*(u16 *)&ctrl[6] = 1U;
		ctrl[10] = fp_val;
		*(u16 *)&ctrl[2] = 0x62U;
		*(u16 *)&ctrl[4] = 0x64U;
		pmu_phy_dbyte_cfg_unpack();

		ctrl[13] = 1U;
		pmu_cal_dbyte_pattern_loop_exec(channel, 1U, buf, buf + 16,
						0U, 1U, 0U);
		pmu_cal_dbyte_pattern_loop_exec(channel, 0U, buf + 8,
						buf + 24, 0U, 1U, 0U);

		for (i = rank_start; i <= rank_end; i++) {
			u16 *p = (u16 *)buf + i;
			u32 p24 = 0U;
			u32 p28 = 0U;
			u8 tl;
			u8 tu;

			pmu_cal_param_unpack(p[0], (u8 *)&p24);
			pmu_cal_param_unpack(p[4], (u8 *)&p28);
			tl = (u8)p24;
			tu = (u8)p28;
			if (tl > tu) {
				pmu_adjust_step_limit((u8 *)&p24, !(tu & 1U));
				p[0] = (u16)pmu_unpack_7((u8 *)&p24);
			} else if (tl < tu) {
				pmu_adjust_step_limit((u8 *)&p28, !(tl & 1U));
				p[4] = (u16)pmu_unpack_7((u8 *)&p28);
			}
		}

		buf[523] = (dmem_read8(PMU_DMEM_BIST_PATTERN_LO) ||
			    dmem_read8(PMU_DMEM_DESKEW_OFFSET_OVERRIDE)) ?
			   7U : 4U;

		if (dmem_102 & 2U) {
			u8 ret = (u8)pmu_phy_query_operating_mode();

			ctrl[0] = ret;
			if (ret == 3U)
				phy_write16(PHY_REG_DBYTE_BCAST_MODE |
					    (active_csr_offset << 1), 0U);
		}

		for (i = rank_start; i <= rank_end; i++) {
			u32 j;

			for (j = 0U; j < 9U; j++) {
				u32 offset = active_csr_offset | (i << 12) |
					     (j << 8);
				u16 val1 = phy_read16(PHY_REG_DBYTE_BASE |
					(((lut_val + 0x12U) | offset) << 1));
				u16 val2 = phy_read16(PHY_REG_DBYTE_BASE |
					(((lut_val + 0x10U) | offset) << 1));

				*(u16 *)(buf + 0x20 + (i * 20U) + (j * 2U)) = val1;
				*(u16 *)(buf + 112 + (i * 20U) + (j * 2U)) = val2;

				if ((dmem_102 & 2U) && ctrl[0] == 3U) {
					u32 csr_base = (PHY_REG_DBYTE_BASE | 0x009cUL) |
						       (offset << 1);
					u16 reg0 = phy_read16(csr_base);
					u16 reg1 = phy_read16(csr_base + 2U);
					u16 reg2 = phy_read16(csr_base + 4U);
					u16 reg3 = phy_read16(csr_base + 6U);
					u16 *diag = (u16 *)(buf + 192 + (i * 80U) + (j * 8U));

					diag[0] = reg0;
					diag[1] = reg1;
					diag[2] = reg2;
					diag[3] = reg3;

					phy_write16(csr_base, (reg0 + reg2) >> 1);
					phy_write16(csr_base + 2U, (reg1 + reg3) >> 1);
				}
			}
		}
	} else {
		u8 nib;

		ctrl[11] = 2U;
		*(u16 *)&ctrl[6] = 2U;
		*(u16 *)&ctrl[2] = 0x5eU;
		*(u16 *)&ctrl[4] = 0x60U;
		*(u16 *)&ctrl[8] = 0xffU;

		nib = (u8)pmu_get_train_cfg96_bit4();
		ctrl[10] = nib ? 10U : fp_val;
		ctrl[12] = nib;

		pmu_cal_dbyte_pattern_loop_exec(channel, 2U, buf, buf + 16,
						0U, 1U, 0U);

		for (i = rank_start; i <= rank_end; i++) {
			u32 base_off = active_csr_offset | (i << 12);
			u32 j;
			u16 csr_val;

			for (j = 0U; j < 9U; j++) {
				u32 offset = base_off | (lut_val + 0x26U) |
					     (j << 8);

				csr_val = phy_read16(PHY_REG_DBYTE_BASE |
						     (offset << 1));

				*(u16 *)(buf + 0x20 + (i * 20U) + (j * 2U)) = csr_val;
			}

			csr_val = phy_read16(PHY_REG_DBYTE_BASE |
				((base_off | (lut_val + 0x28U)) << 1));
			*(u16 *)(buf + 0x20 + (i * 20U) + (9U * 2U)) = csr_val;
		}
	}
}

/**
 * pmu_cal_rank_param_commit() - Coordinate matrix scan, window check, and tap commit (0x8aa8)
 * @fp: Frequency point / channel mask
 * @mode_cfg: Mode configuration (0, 4, or 7)
 * @step_lim: Lane mask / step limit
 * @type: Calibration type (0, 1, or 2)
 */
void pmu_cal_rank_param_commit(u32 fp, u32 mode_cfg, u32 step_lim, u32 type)
{
	u8 c20;
	u8 c21;
	u8 c17;
	u8 c18;
	u32 flag_10;
	u32 flag_14;
	u32 r15_val;
	u32 flag_28;
	u32 flag_1c;
	u32 count_24;
	u32 i;

	c20 = dmem_read8(PMU_DMEM_CH0_CTRL_20);
	c21 = dmem_read8(PMU_DMEM_CH0_CTRL_21);
	if (((c20 & 0x1fU) | (c21 & 0x1fU)) == 0U) {
		dmem_write8(PMU_DMEM_CH0_CTRL_20, 1U);
		dmem_write8(PMU_DMEM_CH0_CTRL_21, 1U);
	}

	c17 = dmem_read8(PMU_DMEM_CTRL_17);
	c18 = dmem_read8(PMU_DMEM_CTRL_18);
	if (((c17 & 0x1fU) | (c18 & 0x1fU)) == 0U) {
		dmem_write8(PMU_DMEM_CTRL_17, 1U);
		dmem_write8(PMU_DMEM_CTRL_18, 1U);
	}

	flag_10 = (type != 1U) ? 1U : 0U;
	flag_14 = (type <= 1U) ? 1U : 0U;
	r15_val = (type == 2U) ? 1U : 0U;
	flag_28 = (mode_cfg != 0U) ? 1U : 0U;
	flag_1c = (dmem_read8(PMU_DMEM_CAL_OVERRIDE_FLAG) == 0U) ?
		  dmem_read8(PMU_DMEM_DQ_SWAP_BASE) : 0U;
	count_24 = (dmem_read8(PMU_DMEM_HIGH_FREQ_FLAG) != 0U) ? 2U : 1U;

	for (i = 0U; i < count_24; i++) {
		u32 ret;

		pmu_cal_margin_matrix_scan_eval(fp, mode_cfg, step_lim, type,
						flag_1c, (u8)i);

		if (dmem_read8(PMU_DMEM_CAL_OVERRIDE_FLAG) == 0U) {
			ret = pmu_cal_rank_margin_window_check(r15_val, 0U,
							       step_lim);
			if (ret == 0U)
				ret = pmu_cal_rank_margin_window_check(r15_val,
								       1U,
								       step_lim);
			pmu_cal_channel_rank_bounds_verify(ret);
		}

		if (mode_cfg != 0U)
			pmu_rank_slice_margin_eval();

		if (flag_14 != 0U) {
			pmu_cal_dly_line_tap_step_adjust(flag_10, (u8)i,
							 step_lim, flag_28);
			if (pmu_sdata()->high_freq_flag != 0U && i == 1U &&
			    flag_10 != 0U)
				pmu_slice_dly_tap_coeff_prog(step_lim);
		} else {
			pmu_cal_dbyte_deskew_pin_results_apply();
		}
	}
}

/**
 * pmu_deskew_sweep_save_delays() - Save current per-slice/lane delay registers
 * @dly_buf: Output 4x10 halfword delay backup matrix
 * @slice_start: First slice index
 * @slice_end: Last slice index
 * @active_slice_mask: Active slice bitmask
 * @ch_offset: Channel CSR offset
 * @profile_base_addr: Profile register base offset
 * @ch: Channel index (0 or 1)
 * @active_lane_mask: Non-zero if lane 8 (DBI/DM) is active
 * @is_lpddr5_type: Non-zero for LPDDR5 lane-9-only mode
 */
static void pmu_deskew_sweep_save_delays(u16 dly_buf[4][10], u8 slice_start,
					 u8 slice_end, u8 active_slice_mask,
					 u32 ch_offset, u32 profile_base_addr,
					 u32 ch, u32 active_lane_mask,
					 u32 is_lpddr5_type)
{
	u32 slice;

	for (slice = slice_start; slice <= slice_end; slice++) {
		u32 slice_offset;
		uintptr_t base_addr;
		u32 lane;

		if (!(active_slice_mask & (1U << slice)))
			continue;

		slice_offset = (slice << 12) | ch_offset | profile_base_addr | ch;
		base_addr = PHY_REG_APB_BASE | (slice_offset << 1);

		for (lane = 0U; lane < 10U; lane++) {
			if (lane == 8U && !active_lane_mask)
				continue;

			if (is_lpddr5_type != 0U) {
				if (lane == 9U)
					dly_buf[slice][9] = phy_read16(base_addr);
			} else if (lane != 9U) {
				uintptr_t reg_addr = PHY_REG_APB_BASE |
					((slice_offset | (lane << 8)) << 1);

				dly_buf[slice][lane] = phy_read16(reg_addr);
			}
		}
	}
}

/**
 * pmu_deskew_write_type0_margins() - Clamp and write type-0 DBYTE lane margin registers
 * @lane_reg_off: Shifted slice/channel/lane register offset
 * @val_r12: Computed left/low margin value
 * @val_r15: Computed right/high margin value
 */
static void pmu_deskew_write_type0_margins(u32 lane_reg_off, s32 val_r12,
					   s32 val_r15)
{
	uintptr_t addr_d0 = (PHY_REG_DBYTE_BASE | 0x00d0UL) | lane_reg_off;
	uintptr_t addr_d4 = (PHY_REG_DBYTE_BASE | 0x00d4UL) | lane_reg_off;
	u8 limit_d0 = (u8)phy_read16(addr_d0);
	u8 limit_d4 = (u8)phy_read16(addr_d4);
	u16 reg_24 = phy_read16((PHY_REG_DBYTE_BASE | 0x0024UL) | lane_reg_off);
	u16 reg_20 = phy_read16((PHY_REG_DBYTE_BASE | 0x0020UL) | lane_reg_off);
	u8 cal_margin_offset = dmem_read8(PMU_DMEM_SEARCH_WINDOW_BIAS_HI);

	if ((u16)val_r12 > limit_d0)
		val_r12 = limit_d0;
	if ((u16)val_r15 > limit_d4)
		val_r15 = limit_d4;

	if (cal_margin_offset != 0U) {
		u16 bmsk_24 = reg_24 & 0x7fU;
		u16 bmsk_20 = reg_20 & 0x7fU;
		u16 min_bmsk = (bmsk_24 >= bmsk_20) ? bmsk_20 : bmsk_24;
		u16 max_bmsk = (bmsk_20 >= bmsk_24) ? bmsk_20 : bmsk_24;
		u32 sum_24 = (u16)val_r15 + bmsk_24;
		u32 sum_20 = (u16)val_r15 + bmsk_20;

		if (!(bmsk_24 > (u16)val_r12 && bmsk_20 > (u16)val_r12))
			val_r12 = (s32)min_bmsk - (s32)cal_margin_offset;
		if (sum_24 > 0x7eU || sum_20 >= 0x7fU)
			val_r15 = (s32)(max_bmsk ^ 0x7fU) -
				  (s32)cal_margin_offset;
	}

	phy_write16(addr_d0, (u16)val_r12);
	phy_write16((PHY_REG_DBYTE_BASE | 0x00d8UL) | lane_reg_off, (u16)val_r12);
	phy_write16(addr_d4, (u16)val_r15);
	phy_write16((PHY_REG_DBYTE_BASE | 0x00dcUL) | lane_reg_off, (u16)val_r15);
}

/**
 * pmu_deskew_decode_lane_edges() - Decode left and right eye edges from sweep sample buffer
 * @sample_offset: Byte offset of the lane's 848-byte sample record
 * @profile_stride: Maximum sweep tap stride (0x80 or 0xd4)
 * @left_out: Output pointer for left edge tap
 * @right_out: Output pointer for right edge tap
 */
static void pmu_deskew_decode_lane_edges(u32 sample_offset, u32 profile_stride,
					 u32 *left_out, u32 *right_out)
{
	u8 blink_flag = *(__volatile__ const u8 *)(uintptr_t)
		((PMU_DMEM_BASE | 0x6594UL) + sample_offset);
	const __volatile__ u8 *s_ptr;
	u8 sample_low;

	if (blink_flag == 0U) {
		*left_out = 0U;
		*right_out = 0U;
		return;
	}

	s_ptr = (const __volatile__ u8 *)(uintptr_t)
		((PMU_DMEM_BASE | PMU_DMEM_SWEEP_SAMPLE_BUF) + sample_offset);
	sample_low = s_ptr[0];

	if (blink_flag == 1U) {
		*right_out = profile_stride;
		*left_out = sample_low;
	} else if (blink_flag == 3U) {
		u8 sample_mid = s_ptr[1];
		u8 sample_high = s_ptr[2];
		u32 diff_sample = (u32)sample_mid - (u32)sample_low;
		u32 diff_limit = profile_stride - (u32)sample_high;
		bool use_hi = diff_limit > diff_sample;

		*right_out = use_hi ? profile_stride : sample_mid;
		*left_out = use_hi ? sample_high : sample_low;
	} else {
		*right_out = s_ptr[1];
		*left_out = sample_low;
	}
}

/**
 * pmu_cal_multi_lane_deskew_sweep() - Execute multi-lane deskew sweep and program DBYTE margins
 * @type: Sweep type (0..3)
 * @ch_mask: Channel bitmask
 * @lane_mask: Active lane bitmask
 */
void pmu_cal_multi_lane_deskew_sweep(u32 type, u32 ch_mask, u32 lane_mask)
{
	u16 dly_buf[4][10];
	u16 lane_metric_max[4] = { 0 };
	u16 lane_metric_min[4] = { 0 };
	u32 desc_ec[9];
	u32 is_lpddr5_type, profile_stride, profile_base_addr;
	u32 freq_lt_3200, active_lane_mask, active_mask, ch;

	memset(desc_ec, 0, sizeof(desc_ec));

	is_lpddr5_type = (type == 3U) ? 1U : 0U;
	profile_stride = (type & 2U) ? 0xd4U : 0x80U;
	profile_base_addr = 0x10010UL + ((type & 2U) ? 0x16U : 0U) +
			    ((type & 1U) ? 2U : 0U);

	freq_lt_3200 = (dmem_read16(PMU_DMEM_DRAM_FREQ_OFF) < 3200U) ? 1U : 0U;
	active_lane_mask = (lane_mask >> 8) & 0x7fU;
	active_mask = (u32)dmem_read8(PMU_DMEM_CH1_RANK_EN) |
		      (u32)dmem_read8(PMU_DMEM_CH0_RANK_EN);

	for (ch = 0U; ch < 2U; ch++) {
		u32 ch_active_mask = (1U << (ch + 2U)) | (1U << ch);
		u8 slice_start, slice_end, active_slice_mask;
		u32 ch_offset, slice;

		if (!(ch_active_mask & active_mask))
			continue;

		dmem_write8(PMU_DMEM_CAL_BYTE, (u8)ch);
		pmu_phy_mode_cfg_dispatch(3U);

		slice_start = dmem_read8(PMU_DMEM_SLICE_START);
		slice_end = dmem_read8(PMU_DMEM_SLICE_END);
		active_slice_mask = dmem_read8(PMU_DMEM_ACTIVE_SLICE_MASK_CAL);
		ch_offset = dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET);

		pmu_deskew_sweep_save_delays(dly_buf, slice_start, slice_end,
					     active_slice_mask, ch_offset,
					     profile_base_addr, ch,
					     active_lane_mask, is_lpddr5_type);

		pmu_cal_dbyte_pattern_loop_exec(ch, (u16)type, lane_metric_max,
						lane_metric_min, 0U, 1U, 0U);

		if (is_lpddr5_type != 0U)
			pmu_dbyte_rx_en_slice_pulse_eval(1U, ch_active_mask);

		pmu_cal_bist_cmd_strobe_dispatch((u32)(uintptr_t)desc_ec, 8U,
						 type, ch_active_mask,
						 (u32)(uintptr_t)lane_metric_max,
						 active_lane_mask, 0U);

		memset((void *)(PMU_DMEM_BASE | PMU_DMEM_SWEEP_SAMPLE_BUF),
		       0, 0x8480U);

		pmu_cal_slice_step_eval_sweep((void *)(PMU_DMEM_BASE | PMU_DMEM_SWEEP_SAMPLE_BUF),
					      8U, type, 4U, 0U, (u8)ch_mask,
					      freq_lt_3200, lane_metric_max,
					      active_lane_mask, 0U, desc_ec);

		pmu_dmem_reg_stream_unpack(desc_ec);

		if (is_lpddr5_type != 0U)
			pmu_dbyte_rx_en_slice_pulse_eval(0U, ch_active_mask);

		for (slice = slice_start; slice <= slice_end; slice++) {
			u32 slice_off, reg_base_ch, reg_base_var, slice_offset_val;
			uintptr_t reg_sp_78, reg_sp_7c, reg_sp_132;
			u32 lane;

			if (!(active_slice_mask & (1U << slice)))
				continue;

			slice_off = slice << 12;
			reg_base_ch = slice_off | ch_offset | ch;
			reg_base_var = slice_off | ch_offset | profile_base_addr | ch;
			slice_offset_val = slice_off | ch_offset | (ch << 1);

			reg_sp_78 = (PHY_REG_DBYTE_BASE | 0x0044UL) | (reg_base_ch << 1);
			reg_sp_7c = (PHY_REG_DBYTE_BASE | 0x0032UL) | (slice_offset_val << 1);
			reg_sp_132 = PHY_REG_APB_BASE | (reg_base_var << 1);

			for (lane = 0U; lane < 10U; lane++) {
				u32 right_edge, left_edge;
				u16 cal_val_raw, cur_val_raw, metric_val_0, metric_val_1;
				s32 term_0, term_12, val_r15, val_r12;
				u32 lane_reg_off;

				if (lane == 8U && !active_lane_mask)
					continue;
				if (is_lpddr5_type ? lane != 9U : lane == 9U)
					continue;

				pmu_deskew_decode_lane_edges((slice * 10U + lane) * 848U,
							     profile_stride,
							     &left_edge, &right_edge);

				cal_val_raw = lane_metric_max[slice];
				cur_val_raw = dly_buf[slice][lane];

				if (type >= 2U) {
					metric_val_0 = cur_val_raw;
					metric_val_1 = cal_val_raw;
				} else {
					metric_val_0 = (u16)((cur_val_raw & 0x7fU) +
							     ((cur_val_raw >> 1) & ~0x3fU));
					metric_val_1 = (u16)((cal_val_raw & 0x7fU) +
							     ((cal_val_raw >> 1) & ~0x3fU));
				}

				right_edge += metric_val_1;
				term_0 = ((s32)metric_val_0 - (s32)metric_val_1) -
					 (s32)left_edge;
				term_12 = (s32)right_edge - (s32)metric_val_0;

				val_r15 = (type < 2U) ? term_12 : term_0;
				val_r12 = (type < 2U) ? term_0 : term_12;
				lane_reg_off = (reg_base_ch | (lane << 8)) << 1;

				if (type == 0U) {
					pmu_deskew_write_type0_margins(lane_reg_off,
								       val_r12,
								       val_r15);
				} else if (type == 2U) {
					phy_write16((PHY_REG_DBYTE_BASE | 0x0038UL) | lane_reg_off,
						    (u16)val_r12);
					phy_write16((PHY_REG_DBYTE_BASE | 0x003cUL) | lane_reg_off,
						    (u16)val_r15);
				} else if (type == 1U) {
					phy_write16((PHY_REG_DBYTE_BASE | 0x00d0UL) | lane_reg_off,
						    (u16)val_r12);
					phy_write16((PHY_REG_DBYTE_BASE | 0x00d8UL) | lane_reg_off,
						    (u16)val_r12);
					phy_write16((PHY_REG_DBYTE_BASE | 0x00d4UL) | lane_reg_off,
						    (u16)val_r15);
					phy_write16((PHY_REG_DBYTE_BASE | 0x00dcUL) | lane_reg_off,
						    (u16)val_r15);
				} else {
					phy_write16(reg_sp_7c, (u16)val_r12);
					phy_write16(reg_sp_78, (u16)val_r15);
				}

				if (lane == 9U)
					phy_write16(reg_sp_132, cur_val_raw);
				else
					phy_write16(PHY_REG_APB_BASE |
						    (((reg_base_var | (lane << 8))) << 1),
						    cur_val_raw);
			}
		}
	}
}

/**
 * pmu_cal_bist_search_win_setup() - Setup BIST search window or program slice delay steps (0xbd64)
 * @stage: 0 for search window and Vref DAC setup, non-zero for slice delay step program
 */
void pmu_cal_bist_search_win_setup(u32 stage)
{
	u32 csr_offset;
	u8 start_slice;
	u8 end_slice;
	u16 pin_delays[36];
	u32 fp;

	if (stage == 0U) {
		dmem_write16(PMU_DMEM_TRAIN_MODE, 0x0010U);
		pmu_cal_search_win_init(PMU_DMEM_BASE | PMU_DMEM_SEARCH_WIN_FINE);
		pmu_cal_vref_dac_step_adjust(8U);
		pmu_sdata()->cal_active_flag = 1U;
		pmu_sdata()->cal_active_flag = 0U;
		pmu_post_cmd_conditional_dispatch(4U);
		dmem_write16(PMU_DMEM_TRAIN_MODE, 0x8010U);
		return;
	}

	csr_offset = pmu_sdata()->active_csr_offset;
	start_slice = dmem_read8(PMU_DMEM_SLICE_START);
	end_slice = dmem_read8(PMU_DMEM_SLICE_END);

	for (fp = 0U; fp < 2U; fp++) {
		u32 cal_mode = fp;
		u16 window_params_buf[4];
		u32 pin_idx;
		u32 slice;

		pmu_pulse_phy_146();
		pmu_cal_slice_step_diff_commit(cal_mode, window_params_buf, 1U);
		pmu_pulse_phy_f2();

		pin_idx = (u32)start_slice * 9U;

		for (slice = start_slice; slice <= end_slice; slice++) {
			u16 step_diff = window_params_buf[slice];
			u32 slice_mask = csr_offset | (slice << 12);
			u32 reg_addr = PHY_REG_DBYTE_BASE |
				       ((slice_mask | (cal_mode + 0x2aU)) << 1);
			u32 low6;
			u32 end_pin = pin_idx + 9U;

			low6 = phy_read16(reg_addr) & 0x3fU;
			step_diff = step_diff + low6 + 0x20U;

			for (; pin_idx < end_pin; pin_idx++)
				pin_delays[pin_idx] = step_diff;

			phy_write16(reg_addr - 4U, step_diff);
		}

		pmu_slice_delay_step_program(csr_offset | (cal_mode + 0x26U),
					     (const s16 *)(pin_delays), 1U, 0U, 0U, 0U);
	}
}

/**
 * pmu_cal_dbyte_rx_fifo_reset_poll() - Poll DBYTE Rx FIFO reset and sample multi-channel phase
 * @channel: Target channel index (0 or 1)
 * @mode_sel: Mode/rank selector
 * @enable_scan: Non-zero to enable gate delay sweep
 * @unused_buf: Output phase sample matrix buffer
 *
 * Return: Signed 16-bit FIFO delay offset difference.
 */
s16 pmu_cal_dbyte_rx_fifo_reset_poll(u32 channel, u32 mode_sel,
				     u32 enable_scan, void *unused_buf)
{
	u8 buf[256];
	s16 ret_val = 0;
	u32 is_mode2;
	u32 csr_offset;
	u8 step;
	u8 *out_matrix;
	u32 base_reg;
	u16 delay;
	u32 ch;
	u32 addr;

	pmu_cal_dbyte_dq_status_check(channel, mode_sel, enable_scan);
	dmem_write8(PMU_DMEM_AC_STEP_FLAG, 0U);

	if (enable_scan != 0U && (dmem_read8(PMU_DMEM_CAL_MODE_1B) & (1U << 1))) {
		u32 mode_flag = (mode_sel != 1U) ? 1U : 0U;
		u32 mode_val = mode_flag + 8U;
		u32 slice_offset;
		u32 win_params[2];
		u32 win_val;
		u32 fp;
		u32 reg_tx_dqs;
		u16 stride;
		u16 sample_val;
		u16 stride_x4;
		u32 reg_gate;
		u32 i;
		u32 scan_idx = 0U;
		s16 fifo_diff_signed;
		u8 val_105;
		u32 offset_105;

		memset(buf, 0, 256U);

		slice_offset = pmu_sdata()->active_csr_offset;
		win_params[0] = 0U;
		win_params[1] = 0U;

		pmu_cal_strobe_pulse_init();

		pmu_cal_sequence_pulse_send(0x41UL << 19, 7U, 8U, 0U, 0U, 0U, 0U);
		pmu_cal_pad_timing_delay_set(0U, 4U, 1U);
		win_val = pmu_cal_window_params_dispatch((const u8 *)win_params,
							 (u16)mode_sel, 3U);
		pmu_cal_pad_timing_delay_set(0U, 4U, 1U);
		pmu_cal_sequence_pulse_send(1UL << 18, 7U, 4U, 0U, 0U, 0U, 0U);
		pmu_cal_sequence_pulse_send((1UL << 19) | 0x80UL, 7U, 8U, 0U, 0U,
					    0U, 0U);

		fp = (channel << 12) | slice_offset;
		reg_tx_dqs = (PHY_REG_AC_BASE | 0x0004UL) | (fp << 1);
		stride = pmu_cal_stride_get();
		phy_write16(reg_tx_dqs, 0x40U);

		sample_val = (u16)(win_val - 8U);
		pmu_pulse_phy_146();

		stride_x4 = (u16)(stride << 2);
		pmu_assert_or_halt(pmu_dbyte_deskew_sample_check(channel, mode_sel,
								 (u16)(sample_val),
								 (u16)(stride_x4)) == 0U,
				   PMU_ERR_DESKEW_SAMPLE_CHECK);

		reg_gate = (PHY_REG_AC_BASE | 0x0002UL) |
			   (((mode_val << 8) | fp) << 1);
		step = dmem_read8(PMU_DMEM_CAL_STRIDE);

		for (i = 0U; i < 256U; i += step) {
			phy_write16(reg_gate, (u16)i);
			pmu_pulse_phy_146();
			buf[i] = (u8)pmu_dbyte_deskew_sample_check(channel, mode_sel,
								   (u16)(sample_val),
								   (u16)(stride_x4));
		}

		phy_write16(reg_gate, 0U);
		phy_write16(reg_tx_dqs, 0U);
		pmu_pulse_phy_146();

		while (scan_idx < 256U && !buf[scan_idx])
			scan_idx += step;

		fifo_diff_signed = (s16)(scan_idx - 64U);
		pmu_assert_or_halt(fifo_diff_signed < 256,
				   PMU_ERR_RX_FIFO_SCAN_OVERFLOW);

		val_105 = dmem_read8(PMU_DMEM_DIFF_TAP_PAIR_105);
		if ((u32)(val_105 - 1U) >= 4U) {
			val_105 = 4U;
			dmem_write8(PMU_DMEM_DIFF_TAP_PAIR_105, 4U);
		}

		offset_105 = (u32)val_105 << 5;
		ret_val = (s16)(fifo_diff_signed - (s16)offset_105);
	}

	is_mode2 = ((mode_sel & 3U) == 2U) ? 1U : 0U;
	pmu_cbt_timing_pulse_coordinator(mode_sel, enable_scan);

	csr_offset = pmu_sdata()->active_csr_offset;
	step = dmem_read8(PMU_DMEM_CAL_STRIDE);
	out_matrix = (u8 *)unused_buf;

	base_reg = (PHY_REG_AC_BASE | 0x0002UL) |
		   (((channel << 12) | csr_offset) << 1);
	for (delay = 0U; delay < 128U; delay += step) {
		u32 out_mask = 0U;
		u8 *p;

		for (ch = 0U, addr = base_reg; ch < 7U; ch++, addr += 0x200U)
			phy_write16(addr, delay);
		pmu_pulse_phy_146();

		pmu_dbyte_deskew_phase_sample(is_mode2, &out_mask, 3U);

		{
			u32 mask = out_mask;

			p = &out_matrix[(uintptr_t)channel * 1792U + delay];
			for (ch = 0U; ch < 7U; ch++, p += 256U, mask >>= 1) {
				p[0] |= (u8)(mask & 1U);
				p[128] |= (u8)((mask >> 8) & 1U);
			}
		}
	}

	addr = PHY_AC_PROFILE_LANE_BASE | (csr_offset << 1);
	for (ch = 0U; ch < 7U; ch++, addr += 0x200U)
		phy_write16(addr, 0U);
	pmu_pulse_phy_146();

	dmem_write8(PMU_DMEM_AC_STEP_FLAG, 1U);
	pmu_cal_multi_rank_deskew_sweep(channel, mode_sel);

	return ret_val;
}

/**
 * pmu_rx_en_select_and_commit_best() - Select best RxEn delay step per slice and commit to PHY
 * @min_diff_table: 13x4 table of minimum per-lane eye widths per step and slice
 * @rx_en_delays_b: 13x4 table of summed per-lane eye widths per step and slice
 * @lut_val: Channel LUT offset
 * @csr_offset: Channel CSR offset
 * @off0: First DBYTE RxEn register offset
 * @off1: Second DBYTE RxEn register offset
 * @rx_en_val_lo: Upper byte of initial off0 register
 * @rx_en_val_hi: Upper byte of initial off1 register
 */
static void pmu_rx_en_select_and_commit_best(const u16 min_diff_table[13][4],
					     const u16 rx_en_delays_b[13][4],
					     u32 lut_val, u32 csr_offset,
					     u16 off0, u16 off1,
					     u8 rx_en_val_lo, u8 rx_en_val_hi)
{
	u8 slice_start = dmem_read8(PMU_DMEM_SLICE_START);
	u8 slice_end = dmem_read8(PMU_DMEM_SLICE_END);
	u32 s;

	for (s = slice_start; s <= slice_end; s++) {
		s8 best_diff = -1;
		u8 best_step = 0xffU;
		s16 best_sum = -1;
		u32 step, addr0, addr1;
		u16 param_428, val_off0, val_off1;
		u8 comp_step;

		for (step = 0U; step < 13U; step++) {
			s8 diff = (s8)(u8)min_diff_table[step][s];

			if (diff > best_diff ||
			    (diff == best_diff &&
			     (s16)rx_en_delays_b[step][s] > best_sum)) {
				best_step = (u8)step;
				best_diff = diff;
				best_sum = (s16)rx_en_delays_b[step][s];
			}
		}

		param_428 = dmem_read16(PMU_DMEM_ITER_COUNT);
		dmem_write16(PMU_DMEM_CBT_DELAY_ARR_OFF + (s * 8U) +
			     (param_428 * 2U), (u16)(s8)best_step);

		comp_step = (u8)(12 - (s8)best_step);
		val_off0 = ((u16)rx_en_val_lo << 8) |
			   ((u16)comp_step << 4) | (u8)best_step;
		addr0 = PHY_REG_DBYTE_BASE |
			(((lut_val + off0) | (s << 12) | csr_offset) << 1);
		phy_write16(addr0, val_off0);

		val_off1 = ((u16)rx_en_val_hi << 8) |
			   ((u16)(u8)best_step << 4) | comp_step;
		addr1 = PHY_REG_DBYTE_BASE |
			(((lut_val + off1) | (s << 12) | csr_offset) << 1);
		phy_write16(addr1, val_off1);
	}
}

/**
 * pmu_cal_rank_rx_en_delay_commit() - Sweep 13 RxEn delay steps and commit best step (0xc9c0)
 * @channel: Target channel index (0 or 1)
 * @fp_flags: Frequency point flags
 * @cal_stage_mode: Calibration stage mode
 * @stage: Stage identifier (0x11 or 0x12)
 */
void pmu_cal_rank_rx_en_delay_commit(u32 channel, u16 fp_flags,
				     u32 cal_stage_mode, u32 stage)
{
	u8 deskew_work_buf[584];
	u8 scratch_buf[64];
	u16 min_diff_table[13][4];
	u16 rx_en_delays_a[13][4];
	u16 rx_en_delays_b[13][4];
	u16 slice_matrix_a[4][10], slice_matrix_b[4][10], slice_matrix_c[4][10];
	u16 slice_matrix_d[4][10], slice_matrix_e[4][10], slice_matrix_f[4][10];
	u32 lut_val = pmu_bit_lookup_e4(channel, channel);
	u32 csr_offset, chan_mask, is_lt_3200 = 1U, idx, s;
	u16 off0, off1, reg0, reg1;
	u8 rx_en_val_lo, rx_en_val_hi, slice_start, slice_end;

	memset(rx_en_delays_b, 0, sizeof(rx_en_delays_b));
	memset(rx_en_delays_a, 0, sizeof(rx_en_delays_a));
	memset(min_diff_table, 0, sizeof(min_diff_table));

	pmu_cal_channel_rank_boundary_eval(deskew_work_buf, (u8)channel,
					   cal_stage_mode, stage);

	csr_offset = pmu_sdata()->active_csr_offset;
	phy_write16(PHY_REG_DBYTE_BCAST_SAMPLE | (csr_offset << 1), 1U);

	off0 = *(u16 *)&deskew_work_buf[514];
	off1 = *(u16 *)&deskew_work_buf[516];

	reg0 = phy_read16(PHY_REG_DBYTE_BASE | ((csr_offset | off0) << 1));
	rx_en_val_lo = (u8)(reg0 >> 8);

	reg1 = phy_read16(PHY_REG_DBYTE_BASE | ((csr_offset | off1) << 1));
	rx_en_val_hi = (u8)(reg1 >> 8);

	chan_mask = (1U << channel) | ((1U << channel) << 2);
	if (stage == 0x11U)
		is_lt_3200 = pmu_check_freq_lt_3200();

	for (idx = 0U; idx <= 12U; idx++) {
		u8 comp_idx = (u8)(12U - idx);
		u16 val_r12 = ((u16)rx_en_val_lo << 8) | ((u16)comp_idx << 4) | (u8)idx;
		u16 val_r13 = ((u16)rx_en_val_hi << 8) | ((u16)idx << 4) | (u8)comp_idx;
		void *sweep_buf = (void *)(PMU_DMEM_BASE | PMU_DMEM_SWEEP_SAMPLE_BUF);
		u32 i;
		u16 ctrl_8;
		u8 num_samples;

		memset(slice_matrix_f, 0, sizeof(slice_matrix_f));
		memset(slice_matrix_e, 0, sizeof(slice_matrix_e));
		memset(slice_matrix_d, 0, sizeof(slice_matrix_d));

		slice_start = dmem_read8(PMU_DMEM_SLICE_START);
		slice_end = dmem_read8(PMU_DMEM_SLICE_END);

		for (s = slice_start; s <= slice_end; s++) {
			u32 base = (s << 12) | csr_offset;

			phy_write16(PHY_REG_DBYTE_BASE | (((lut_val + off0) | base) << 1),
				    val_r12);
			phy_write16(PHY_REG_DBYTE_BASE | (((lut_val + off1) | base) << 1),
				    val_r13);
		}

		pmu_pulse_phy_146();

		for (i = 0U; i < 40U; i++)
			dmem_write8(PMU_DMEM_SWEEP_BLINK_OFF + i * 848U, 0U);

		pmu_cal_bist_cmd_strobe_dispatch((u32)(uintptr_t)scratch_buf,
						 deskew_work_buf[523],
						 deskew_work_buf[518],
						 (u8)chan_mask,
						 (u32)(uintptr_t)deskew_work_buf,
						 cal_stage_mode,
						 deskew_work_buf[525]);

		pmu_cal_slice_step_eval_sweep(sweep_buf, deskew_work_buf[523],
					      deskew_work_buf[518], 4U,
					      (u8)chan_mask, (u8)fp_flags,
					      is_lt_3200,
					      (const u16 *)deskew_work_buf,
					      cal_stage_mode,
					      deskew_work_buf[525],
					      scratch_buf);

		if (deskew_work_buf[524] != 0U) {
			pmu_dbyte_rx_en_slice_pulse_eval(1U, (u8)chan_mask);
			pmu_cal_slice_step_eval_sweep(sweep_buf,
						      deskew_work_buf[523], 3U, 4U,
						      (u8)chan_mask, (u8)fp_flags,
						      1U, (const u16 *)deskew_work_buf,
						      0U, 0U, scratch_buf);
			pmu_dbyte_rx_en_slice_pulse_eval(0U, (u8)chan_mask);
		}

		pmu_dmem_reg_stream_unpack(scratch_buf);
		pmu_eye_centroid_avg_calc((const u16 *)(deskew_work_buf + 16),
					  (u16 *)slice_matrix_f,
					  (u16 *)slice_matrix_e,
					  (u16 *)slice_matrix_d);

		ctrl_8 = *(u16 *)&deskew_work_buf[520];
		if (ctrl_8 != 255U) {
			for (i = 0U; i < 40U; i++)
				dmem_write8(PMU_DMEM_SWEEP_BLINK_OFF +
					    i * 848U, 0U);

			pmu_cal_bist_cmd_strobe_dispatch((u32)(uintptr_t)scratch_buf,
							 deskew_work_buf[523],
							 (u8)ctrl_8,
							 (u8)chan_mask,
							 (u32)(uintptr_t)(deskew_work_buf + 8),
							 cal_stage_mode,
							 deskew_work_buf[525]);

			pmu_cal_slice_step_eval_sweep(sweep_buf,
						      deskew_work_buf[523],
						      (u8)ctrl_8, 4U,
						      (u8)chan_mask,
						      (u8)fp_flags,
						      is_lt_3200,
						      (const u16 *)(deskew_work_buf + 8),
						      cal_stage_mode,
						      deskew_work_buf[525],
						      scratch_buf);

			pmu_dmem_reg_stream_unpack(scratch_buf);
			pmu_eye_centroid_avg_calc((const u16 *)(deskew_work_buf + 24),
						  (u16 *)slice_matrix_c,
						  (u16 *)slice_matrix_b,
						  (u16 *)slice_matrix_a);
			ctrl_8 = *(u16 *)&deskew_work_buf[520];
		}

		num_samples = deskew_work_buf[522];
		for (s = slice_start; s <= slice_end; s++) {
			s32 min_val = 127;

			if (num_samples != 0U) {
				s16 sum = 0;
				u32 k;

				for (k = 0U; k < num_samples; k++) {
					s8 diff = (s8)(slice_matrix_d[s][k] -
						       slice_matrix_e[s][k]);

					if (diff < min_val)
						min_val = diff;
					sum += diff;
				}
				rx_en_delays_b[idx][s] = sum;

				if (ctrl_8 == 0U) {
					s32 min_val2 = 127;
					s16 sum2 = 0;

					for (k = 0U; k < num_samples; k++) {
						s8 diff = (s8)(slice_matrix_a[s][k] -
							       slice_matrix_b[s][k]);

						min_val2 = (diff < min_val2) ? diff : min_val2;
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

	pmu_rx_en_select_and_commit_best(min_diff_table, rx_en_delays_b,
					 lut_val, csr_offset, off0, off1,
					 rx_en_val_lo, rx_en_val_hi);

	pmu_cal_dbyte_deskew_results_apply(deskew_work_buf, (u8)channel,
					   stage);
}

/**
 * pmu_cal_lane_window_sweep_coordinator() - Coordinate 4-tap lane window sweep and commit best taps
 * @ch_mask: Channel mask
 * @wide_range: Non-zero for 9 pins (511 mask), 0 for 8 pins (255 mask)
 */
void pmu_cal_lane_window_sweep_coordinator(u32 ch_mask, u32 wide_range)
{
	u16 margin_table[4][2][2][2];
	u16 best_taps[2][2][2];
	u32 fp_range = (wide_range != 0U) ? 511U : 255U;
	u32 pin_count = (wide_range != 0U) ? 9U : 8U;
	u8 cal_p40;
	u8 cal_p25;
	u8 active_mask;
	u32 cfg_idx;
	u32 ch;

	memset(margin_table, 0, sizeof(margin_table));
	memset(best_taps, 0, sizeof(best_taps));

	cal_p40 = dmem_read8(PMU_DMEM_CH1_RANK_EN);
	cal_p25 = dmem_read8(PMU_DMEM_CH0_RANK_EN);
	active_mask = cal_p25;

	for (cfg_idx = 0U; cfg_idx < 4U; cfg_idx++) {
		u8 cfg_val = (u8)(cfg_idx | (cfg_idx << 4));
		u32 vref;

		for (ch = 0U; ch < 2U; ch++) {
			u8 mask = (u8)((1U << (ch + 2U)) | (1U << ch));

			if (active_mask & mask)
				pmu_slice_deskew_state_latch(cfg_val, mask);
		}

		vref = (dmem_read8(PMU_DMEM_CLK_GATE_FLAG) & 4U) ?
		       0x3fU : dmem_read8(PMU_DMEM_DQ_SWAP_BASE);
		pmu_cal_margin_matrix_scan_eval(ch_mask, 0U, fp_range, 2U,
						vref, 0U);

		for (ch = 0U; ch < 2U; ch++) {
			u8 mask = (u8)((1U << (ch + 2U)) | (1U << ch));
			u8 start_slice;
			u8 end_slice;
			uintptr_t slice_buf;
			u32 slice;
			u32 rank;

			if (!(active_mask & mask))
				continue;

			start_slice = dmem_read8(PMU_DMEM_RANK0_SLICE_START);
			end_slice = dmem_read8(PMU_DMEM_RANK1_SLICE_END);
			slice_buf = (PMU_DMEM_BASE | PMU_DMEM_EYE_SAMPLE_BUF) +
				    (ch * 5280U) + (start_slice * 1320U);

			for (slice = start_slice; slice <= end_slice; slice++) {
				uintptr_t pin_ptr = slice_buf;
				u32 pin;

				for (pin = 0U; pin < pin_count; pin++) {
					pmu_window_centroid_calc((u8 *)pin_ptr);
					pin_ptr += 132U;
				}
				slice_buf += 1320U;
			}

			for (rank = 0U; rank < 2U; rank++) {
				u8 rank_active = (rank != 0U) ? cal_p40 : cal_p25;

				if (rank_active != 0U) {
					u8 r_start = (rank != 0U) ?
						dmem_read8(PMU_DMEM_RANK1_SLICE_START) :
						dmem_read8(PMU_DMEM_RANK0_SLICE_START);
					u8 r_end = (rank != 0U) ?
						dmem_read8(PMU_DMEM_RANK1_SLICE_END) :
						dmem_read8(PMU_DMEM_RANK_BOUNDARY);

					margin_table[cfg_idx][ch][rank][0] =
						pmu_rank_margin_channel_accum(ch, r_start, r_end,
									      pin_count, 1U);
					margin_table[cfg_idx][ch][rank][1] =
						pmu_rank_margin_channel_accum(ch, r_start, r_end,
									      pin_count, 2U);
				}
			}
		}
	}

	for (ch = 0U; ch < 2U; ch++) {
		u8 mask = (u8)((1U << (ch + 2U)) | (1U << ch));
		u32 rank;

		if (!(active_mask & mask))
			continue;

		for (rank = 0U; rank < 2U; rank++) {
			u8 rank_active = (rank != 0U) ? cal_p40 : cal_p25;
			u32 dir;

			if (!rank_active)
				continue;

			for (dir = 0U; dir < 2U; dir++) {
				u16 max_width = 0U;
				u16 best_idx = 0U;
				u32 iter;

				for (iter = 0U; iter < 4U; iter++) {
					u16 width = margin_table[iter][ch][rank][dir];

					if (width > max_width) {
						max_width = width;
						best_idx = (u16)iter;
					}
				}

				best_taps[ch][rank][dir] = best_idx;

				if (max_width == 0U)
					pmu_assert_or_halt(0U, PMU_ERR_WCK_EYE_ZERO_WIDTH);
			}
		}
	}

	for (ch = 0U; ch < 2U; ch++) {
		u8 mask = (u8)((1U << (ch + 2U)) | (1U << ch));
		u32 rank;

		if (!(active_mask & mask))
			continue;

		pmu_cal_pulse_seq_coordinator(0U, ch, 0x50U, 3U);

		for (rank = 0U; rank < 2U; rank++) {
			u8 rank_active = (rank != 0U) ? cal_p40 : cal_p25;
			u8 combined;

			if (!rank_active)
				continue;

			pmu_phy_analog_cfg_mode_set(rank);

			combined = (u8)((best_taps[ch][rank][1] << 4) |
					best_taps[ch][rank][0]);
			pmu_slice_deskew_state_latch(combined, mask);

			dmem_write8(PMU_DMEM_BEST_TAP_ARR_B7A + (rank * 2U) + ch,
				    combined);

			if (ch == 0U) {
				if (rank == 0U)
					dmem_write8(PMU_DMEM_WCK_TAP_C0R0_D1, combined);
				else
					dmem_write8(PMU_DMEM_WCK_TAP_C0R1_DB, combined);
			} else {
				if (rank == 0U)
					dmem_write8(PMU_DMEM_WCK_TAP_C1R0_D6, combined);
				else
					dmem_write8(PMU_DMEM_WCK_TAP_C1R1_E0, combined);
			}
		}
	}

	phy_write16(PHY_REG_ANALOG_CFG0, dmem_read8(PMU_DMEM_CH0_RANK_EN));
	phy_write16(PHY_REG_ANALOG_CFG1, dmem_read8(PMU_DMEM_CH1_RANK_EN));
}

/**
 * pmu_rank_deskew_converge_slices() - Run 11-iteration coarse/fine deskew convergence across slices
 * @csr_offset: Channel CSR offset
 * @lut_mode_bits: 2-bit channel LUT mode offset
 * @dram_type: Active DRAM type code from DMEM
 */
static void pmu_rank_deskew_converge_slices(u32 csr_offset, u32 lut_mode_bits,
					    u8 dram_type)
{
	u16 slice_done[4] = { 0 };
	u16 slice_val[4] = { 0 };
	u16 prev_val[4] = { 0 };
	u32 flag_sp67 = 0U;
	int iter;

	for (iter = 11; iter > 0; iter--) {
		u8 start_slice = dmem_read8(PMU_DMEM_SLICE_START);
		u8 end_slice = dmem_read8(PMU_DMEM_SLICE_END);
		u32 slice;

		pmu_cal_slice_step_diff_latch(0U, 1U);

		for (slice = start_slice; slice <= end_slice; slice++) {
			u32 slice_sh13, slice_reg_base, reg_addr;
			u16 stat, phy_status_reg, prev_stat;
			u32 fine_delay, coarse_delay, new_val, target_reg;

			if (slice_done[slice] != 0U)
				continue;

			slice_sh13 = slice << 13;
			stat = phy_read16((PHY_REG_DBYTE_BASE | 0x016eUL) | slice_sh13);
			slice_val[slice] = stat;

			slice_reg_base = (csr_offset | (slice << 12) |
					  (lut_mode_bits + 0x2aU)) << 1;
			reg_addr = PHY_REG_DBYTE_BASE | slice_reg_base;
			phy_status_reg = phy_read16(reg_addr);
			prev_stat = prev_val[slice];

			if (prev_stat != 0U && stat == 0U) {
				slice_done[slice] = 1U;
				continue;
			}

			if (prev_stat == 0U && iter == 11 && stat == 0U)
				flag_sp67 = 1U;

			fine_delay = phy_status_reg & 0x3fU;
			coarse_delay = phy_status_reg >> 6;

			coarse_delay += 2U;
			new_val = (coarse_delay << 6) | fine_delay;
			target_reg = (PHY_REG_DBYTE_BASE | 0x1e00UL) |
				     slice_reg_base;

			if (coarse_delay >= 0x15U && flag_sp67 != 0U &&
			    (coarse_delay % dram_type) != 0U) {
				u16 new_stat;

				phy_write16(target_reg, (u16)fine_delay);
				pmu_phy_reg_176_strobe_pulse(0x0fU, 0x0fU);
				pmu_pulse_phy_f2();
				pmu_cal_slice_step_diff_latch(0U, 1U);

				new_stat = phy_read16((PHY_REG_DBYTE_BASE | 0x016eUL) |
						      slice_sh13);
				slice_val[slice] = new_stat;

				if (new_stat != 0U) {
					phy_write16(target_reg, (u16)new_val);
					slice_done[slice] = 1U;
					continue;
				}
				new_val = fine_delay;
			}
			if (coarse_delay >= 0x15U)
				pmu_assert_or_halt(0U, PMU_ERR_COARSE_DELAY_OVERFLOW);

			phy_write16(target_reg, (u16)new_val);
			prev_val[slice] = slice_val[slice];
		}

		pmu_phy_reg_176_strobe_pulse(0x0fU, 0x0fU);
		pmu_pulse_phy_f2();
	}
}

/**
 * pmu_rank_deskew_run_convergence_stage() - Setup PHY BIST state and run deskew convergence
 * @channel: Target channel index (0 or 1)
 * @rank: Target rank index (0 or 1)
 * @csr_offset: Channel CSR offset
 * @lut_mode_bits: 2-bit channel LUT mode offset
 * @fp_cbt: Initial CBT delay index
 * @dram_type: Active DRAM type code from DMEM
 * @ch_mask_b: Active channel bitmask byte
 */
static void pmu_rank_deskew_run_convergence_stage(u32 channel, u32 rank,
						  u32 csr_offset,
						  u32 lut_mode_bits,
						  u32 fp_cbt, u8 dram_type,
						  u8 ch_mask_b)
{
	u32 csr_offset_sh1 = csr_offset << 1;
	u32 cbt_dly_offset, slice_mask;
	u16 saved_88, target_cfg_val, saved_84;

	(void)channel;
	(void)rank;

	pmu_slice_phy_reg_step_adjust(lut_mode_bits, 1U, 0U);

	fp_cbt += (fp_cbt & 1U);
	pmu_ac_lane_profile_setup();

	if (dmem_read8(PMU_DMEM_FREQ_MODE) == 0U) {
		pmu_cal_strobe_pulse_init();
		pmu_cal_sequence_pulse_send(0U, 7U, 4U, 0U, 0U, 0U, 0U);
		pmu_cal_slice_step_diff_latch(0U, 1U);
	}

	pmu_cal_strobe_pulse_init();
	cbt_dly_offset = fp_cbt + 0x1eU;

	pmu_cal_margin_dir_get();
	pmu_cbt_3phase_pulse_seq();
	pmu_cal_slice_step_diff_latch(0U, 1U);

	saved_88 = phy_read16(PHY_REG_DRAM_TYPE_CFG);
	target_cfg_val = (dram_type == 2U) ? 513U : 1026U;

	phy_write16(PHY_REG_DRAM_TYPE_CFG, (dram_type == 2U) ? 3U : 2U);
	phy_write16((PHY_REG_MASTER_BASE | 0x007aUL) | csr_offset_sh1,
		    target_cfg_val);
	phy_write16((PHY_REG_MASTER_BASE | 0x007cUL) | csr_offset_sh1,
		    target_cfg_val);

	saved_84 = phy_read16(PHY_REG_VREF_STAT);
	phy_write16(PHY_REG_VREF_STAT, saved_84 | (1U << 7));
	pmu_slice_phy_reg_step_adjust(lut_mode_bits, 1U, 1U);

	slice_mask = (pmu_dmem_training_flags_eval() == 0U) ? 0xffU : 511U;
	phy_write16(PHY_REG_DBYTE_BCAST_DQS_DLY0, 0xffffU);
	phy_write16(PHY_REG_DBYTE_BCAST_DQS_DLY1, 0xffffU);
	pmu_cal_dbyte_sample_ready_poll_all(slice_mask);

	phy_write16(PHY_REG_BIST_CTRL_MODE, 1U);
	phy_write16((PHY_REG_PUB_BASE | 0x3ff7cUL), 0U);

	pmu_phy_reg_176_strobe_pulse(0x0fU, 0x0fU);
	pmu_pulse_phy_f2();

	pmu_cal_slice_step_scan_eval(cbt_dly_offset & 0xffU, ch_mask_b);

	pmu_rank_deskew_converge_slices(csr_offset, lut_mode_bits, dram_type);

	phy_write16(PHY_REG_DRAM_TYPE_CFG, saved_88);
	phy_write16(PHY_REG_VREF_STAT, saved_84);
	pmu_phy_profile_param_program();
	pmu_slice_phy_reg_step_adjust(lut_mode_bits, 0U, 0U);
}

/**
 * pmu_cal_rank_deskew_matrix_commit() - Execute rank deskew matrix calibration and iterative
 * convergence (0xea8c)
 * @channel: Target channel index (0 or 1)
 * @rank: Target rank index (0 or 1)
 * @buf: Work buffer for LCDL delay readback
 */
void pmu_cal_rank_deskew_matrix_commit(u32 channel, u32 rank, void *buf)
{
	struct phy_reg_stream_entry {
		u32 addr;
		u16 val;
	} __packed;

	u32 csr_offset = pmu_sdata()->active_csr_offset;
	u32 csr_offset_sh1 = csr_offset << 1;
	u32 ch_mask = (1U << (channel + 2U)) | (1U << channel);
	u8 ch_mask_b = (u8)ch_mask;
	u32 cbt_idx = pmu_cal_rank_offset_get(rank, channel);
	u32 cbt_val = pmu_cbt_delay_offset_lookup(rank, channel);
	u32 cbt_lut_offset = (cbt_val > 186U) ? 0xfeU : (cbt_val * 2U + 0x22U);
	u32 lut_val = (0xe4U >> ((channel << 1) & 6U)) & 3U;

	struct phy_reg_stream_entry stream_save[15] = {
		{ 0x10097UL, 0U },
		{ 0x10093UL, 0U },
		{ 0x10005UL | csr_offset, 0U },
		{ 0x1008bUL, 0U },
		{ 0x10095UL, 0U },
		{ 0x20074UL, 0U },
		{ 0x20057UL, 0U },
		{ 0x2002aUL, 0U },
		{ 0x2003fUL | csr_offset, 0U },
		{ 0x20041UL | csr_offset, 0U },
		{ 0x20037UL | csr_offset, 0U },
		{ 0x2003fUL | csr_offset, 0U },
		{ 0x2003dUL | csr_offset, 0U },
		{ 0x2003eUL | csr_offset, 0U },
		{ 0x70011UL, 0U },
	};

	pmu_phy_reg_buffer_stream((u8 *)stream_save, 15U, 1U);

	phy_write16(PHY_REG_BIST_PLL_CFG, 0U);
	phy_write16((PHY_REG_DBYTE_BCAST_BASE | 0x012eUL), 0U);
	phy_write16(PHY_REG_DBYTE_BCAST_DQ_DLY0, 8U);
	phy_write16(PHY_REG_DBYTE_BCAST_DQ_DLY1, 0x40U);
	phy_write16((PHY_REG_PUB_SEC_BASE | 0x0aeUL), 0xaaU);

	pmu_cal_strobe_pulse_init();

	pmu_cal_sequence_pulse_send(0x41UL << 19, 7U, 4U, 0U, 0U, ch_mask_b, 0U);
	pmu_cal_sequence_pulse_send(0U, 7U, 8U, 0U, 0U, 0U, 0U);
	pmu_cal_sequence_pulse_send(0x80UL | (1UL << 19), 7U, 4U, 0U, 0U, ch_mask_b, 0U);

	pmu_cal_slice_step_diff_latch(0U, 1U);
	pmu_delay_us(5000U, 5U);
	pmu_dbyte_cal_strobe_pulse_seq(1U, 0xffU, 0U);

	u8 dram_type = dmem_read8(PMU_DMEM_DRAM_TYPE);
	u32 is_dram_type_lpddr5 = (dram_type == 4U) ? 1U : 0U;
	u32 dram_type_cfg_val = ((u32)dram_type << 6) + 6U;
	u32 val_43a = (is_dram_type_lpddr5 << 2) + 4U;

	if (dram_type_cfg_val > 255U)
		dram_type_cfg_val = 255U;

	dmem_write8(PMU_DMEM_2D_STEP_PARAM_43A, (u8)val_43a);
	dmem_write16(PMU_DMEM_2D_STEP_CTRL, (u16)dram_type_cfg_val);

	u32 cbt_step_adj = cbt_lut_offset + (cbt_lut_offset & 1U);
	u8 cbt_step_adj_b = (u8)cbt_step_adj;
	u32 cbt_dly_adj = (cbt_idx + 0x1eU) + (cbt_idx & 1U);
	u8 cbt_dly_adj_b = (u8)cbt_dly_adj;

	phy_write16(PHY_REG_VREF_CTRL | csr_offset_sh1, 1U);
	phy_write16(PHY_REG_VREF_TRIM_CFG, 1U);

	u16 val_6e = phy_read16((PHY_REG_MASTER_BASE | 0x006eUL) | csr_offset_sh1);
	u16 val_7e = val_6e & 0x3fU;

	if (dram_type == 2U)
		val_7e |= (1U << 11);
	else
		val_7e |= (1U << 12);
	phy_write16((PHY_REG_MASTER_BASE | 0x007eUL) | csr_offset_sh1, val_7e);

	u32 lut_mode_bits = lut_val & 3U;

	phy_write16((PHY_REG_DBYTE_BCAST_BASE | 0x0126UL), 512U);
	phy_write16(PHY_REG_DBYTE_BCAST_GATE | csr_offset_sh1, 1U);
	pmu_rank_mask_multiparam_dispatch(channel, rank, 1U);

	struct phy_reg_stream_entry stream_play[19] = {
		{ 0x0001ffb5UL, 0U },
		{ 0x0001f0b5UL, 1U },
		{ 0x0001f4b5UL, 1U },
		{ 0x0001f0b3UL, 0U },
		{ 0x0001f0b4UL, 0U },
		{ 0x0001f0b9UL, 1U },
		{ 0x0001f0baUL, 1U },
		{ 0x0001f0b1UL, 2U },
		{ 0x0007f060UL, 1U },
		{ 0x0007f065UL, 511U },
		{ 0x0007ff28UL, 0xffffU },
		{ 0x0007ff29UL, 0xffffU },
		{ 0x0007ff2aUL, 0xffffU },
		{ 0x0007ff2bUL, 0xffffU },
		{ 0x0001ffbeUL, 0U },
		{ 0x0001f0beUL, 1U },
		{ 0x0001f4beUL, 16U },
		{ 0x0001ffaaUL, (u16)(val_43a << 10) },
		{ 0x0001f0a7UL, 768U },
	};

	pmu_cal_seq_table_timed_replay((const u8 *)stream_play, sizeof(stream_play));

	pmu_phy_reg_176_strobe_pulse(0x0fU, 0x0fU);
	pmu_cal_dbyte_gate_mask_clear();
	pmu_dbyte_slice_cal_param_pulse(rank, channel, 1U);

	pmu_cal_strobe_pulse_init();
	pmu_cal_sequence_pulse_send(0U, 7U, 4U, 0U, 0U, 0U, 0U);
	pmu_flag_45c_set();

	u32 cbt_channel_mask = (ch_mask << 5) & 64U;

	pmu_cal_sequence_pulse_send(cbt_channel_mask | (1UL << 25), 7U, 2U, 0U,
				    0U, 0U, 0U);
	pmu_cal_sequence_pulse_send(cbt_channel_mask, 7U, cbt_step_adj_b, 0U,
				    0U, 0U, 0U);
	pmu_cal_sequence_pulse_send(4UL | (1UL << 18), 1U, cbt_dly_adj_b,
				    1U << 12, 0U, 0U, 0U);
	pmu_cal_sequence_pulse_send(cbt_channel_mask | (1UL << 15), 7U, 4U, 0U,
				    0U, 0U, 1U);

	pmu_flag_45c_clear();
	pmu_cal_sequence_pulse_send(cbt_channel_mask, 7U, 0x10U, 0U, 0U, 0U, 0U);
	pmu_cal_sequence_pulse_send(cbt_channel_mask | (1U << 7), 7U, 4U, 0U,
				    0U, 0U, 0U);

	phy_write16(PHY_REG_DBYTE_BCAST_RX_EN, 0x6300U);
	pmu_cal_slice_step_diff_latch(8U, 1U);
	phy_write16(PHY_REG_DBYTE_BCAST_RX_EN, 768U);

	u16 timing_438 = dmem_read16(PMU_DMEM_2D_STEP_CTRL);

	if (dmem_read8(PMU_DMEM_CLK_GATE_FLAG) & (1U << 2)) {
		pmu_cal_slice_step_diff_latch(timing_438, 1U);
	} else {
		int i;

		for (i = 0; i < 10; i++)
			pmu_cal_slice_step_diff_latch(timing_438, 1U);
	}

	pmu_dbyte_slice_cal_param_pulse(rank, channel, 0U);
	pmu_phy_lcdl_delay_read((u16 *)buf, 0U, 2U);

	u32 reg_target_base = csr_offset | (lut_mode_bits + 0x2aU);

	pmu_slice_delay_step_program(reg_target_base, (const s16 *)((const u16 *)buf), 0U, 0U,
				     0U, 1U);

	phy_write16(PHY_REG_DBYTE_BCAST_RX_EN, 0U);

	if ((dmem_read8(PMU_DMEM_CAL_STEP_106) & 1U) == 0U)
		pmu_rank_deskew_run_convergence_stage(channel, rank,
						      csr_offset,
						      lut_mode_bits, cbt_idx,
						      dram_type, ch_mask_b);

	pmu_dbyte_cal_strobe_pulse_seq(0U, 0U, 0U);

	phy_write16(PHY_REG_DBYTE_BCAST_DQ_DLY0, 0U);
	phy_write16(PHY_REG_DBYTE_BCAST_DQ_DLY1, 0U);
	pmu_cal_strobe_pulse_init();
	pmu_cal_pad_timing_delay_set(0U, 5U, 1U);
	pmu_cal_slice_step_diff_latch(0U, 1U);
	pmu_rank_mask_multiparam_dispatch(channel, rank, 0U);

	pmu_cal_strobe_pulse_init();
	pmu_cal_sequence_pulse_send(0U, 7U, 4U, 0U, 0U, 0U, 0U);
	pmu_cal_sequence_pulse_send(1UL << 16, 3U, cbt_step_adj_b, 1U << 12,
				    0U, ch_mask_b, 0U);
	pmu_cal_sequence_pulse_send(0x80U, 7U, 4U, 0U, 0U, 0U, 0U);

	pmu_cal_slice_step_diff_latch(0U, 1U);
	pmu_pulse_phy_f2();
	pmu_pulse_phy_140();

	pmu_phy_reg_buffer_stream((u8 *)stream_save, 15U, 0U);
}
