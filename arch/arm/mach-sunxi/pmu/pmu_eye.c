// SPDX-License-Identifier: GPL-2.0+
/*
 * Allwinner A733 Cadence DDR PHY PMU 2D Eye Margin & Search Window Helpers
 */

#include "pmu_fw.h"

static int pmu_ac_step_coord_adjust(u8 *struct_ptr);
static int pmu_ac_step_pair_align_verify(u8 *struct_a, u8 *struct_b);
static void pmu_cal_dbyte_delay_coarse_step(void);
static void pmu_cal_dbyte_delay_restore_all_ch(void);
static u32 pmu_cal_margin_sample_pair_extract(u8 *out_pair, const u8 *margins, const u8 *samples,
					      const u8 *scan_lut);
static u32 pmu_eye_boundary_sample_search(u8 *samples, u32 target_y);
static void pmu_eye_margin_bidir_scan(u8 *buf, u32 flag_a, u32 flag_b, u32 flag_c);
static u32 pmu_eye_margin_step_balancer(u8 *buf0, u8 *buf1, u8 *buf2, u8 *buf3);
static u32 pmu_eye_sample_metric_eval(u8 x, u8 y, const u8 *eye_scan_lut, const u8 *samples);
static void pmu_eye_sample_table_transform(u8 *dest, const u8 *src);
static u32 pmu_window_margin_diff_calc(const u8 *src0, const u8 *src1, u8 *out_dest, s32 offset,
				       s32 delta);

/**
 * pmu_cal_win_bounds_clear() - Reset 2D search window bounds at DCCM 0x8000043c
 *
 * Initializes the two 32-bit search window bound words at DCCM 0x8000043c and
 * 0x80000440 to the default range 0x00080008 (low=8, high=8).
 */
void pmu_cal_win_bounds_clear(void)
{
	u32 val = 0x00080008UL;
	u32 *tbl0 = (u32 *)(uintptr_t)pmu_sdata()->master_dly_tbl[0];
	u32 *tbl1 = (u32 *)(uintptr_t)pmu_sdata()->master_dly_tbl[1];

	*(__volatile__ u32 *)tbl0 = val;
	*(__volatile__ u32 *)tbl1 = val;
}

/**
 * pmu_cal_search_win_init() - Seed 0xff sentinel bounds in search window buffer
 * @base: DCCM base address of the search window descriptor buffer
 *
 * Writes 0xff sentinel bytes into offsets [0..1], [4..5], [27..28], and
 * [31..32] of the search window buffer at @base.
 */
void pmu_cal_search_win_init(u32 base)
{
	__volatile__ u8 *win_buf = (__volatile__ u8 *)base;
	u8 ff = 0xffU;

	win_buf[0] = ff;
	win_buf[4] = ff;
	win_buf[27] = ff;
	win_buf[31] = ff;
	win_buf[1] = ff;
	win_buf[5] = ff;
	win_buf[28] = ff;
	win_buf[32] = ff;
}

/**
 * pmu_cal_eye_table_restore() - Restore byte field across 4 slices from 0x80000bc0
 * @dest: Destination 4-slice descriptor table (strides 108 and 54 bytes)
 * @offset: Element offset within each slice descriptor
 */
noinline void pmu_cal_eye_table_restore(u8 *dest, u32 offset)
{
	u8 *d = dest + offset;
	const __volatile__ u16 *s =
		(const __volatile__ u16 *)PMU_DMEM_EYE_BACKUP_BC0 + offset;
	u32 i;

	for (i = 0U; i < PMU_SLICE_DESC_TOTAL_BYTES; i += PMU_SLICE_DESC_SIZE)
		d[i] = (u8)s[i];
}

/**
 * pmu_cal_eye_table_save() - Save byte field across 4 slices into DCCM 0x80000bc0
 * @src: Source 4-slice descriptor table (strides 108 and 54 bytes)
 * @offset: Element offset within each slice descriptor
 */
noinline void pmu_cal_eye_table_save(const u8 *src, u32 offset)
{
	const u8 *s = src + offset;
	__volatile__ u16 *d =
		(__volatile__ u16 *)PMU_DMEM_EYE_BACKUP_BC0 + offset;
	u32 i;

	for (i = 0U; i < PMU_SLICE_DESC_TOTAL_BYTES; i += PMU_SLICE_DESC_SIZE)
		d[i] = s[i];
}

/**
 * pmu_cal_margin_dir_get() - Configure default pad timing delay (0, 5, 1)
 */
void pmu_cal_margin_dir_get(void)
{
	pmu_cal_pad_timing_delay_set(0U, 5U, 1U);
}

struct pmu_pin_map_pair {
	u8 match;
	u8 bit;
};

struct pmu_pin_map_entry {
	u8 base_mask;
	u8 count;
	struct pmu_pin_map_pair pairs[8];
};

/**
 * pmu_cal_eye_margin_sample_latch() - Translate logical bitmask to PHY pin mask
 * @mask: Logical lane bitmask byte
 * @lane: Slice/lane index into the 18-byte pin map table at DCCM 0x80000a7e
 *
 * Return: Translated 8-bit PHY pin mask combining base_mask and matched pairs.
 */
u32 pmu_cal_eye_margin_sample_latch(u32 mask, u32 lane)
{
	const struct pmu_pin_map_entry *table =
		(const struct pmu_pin_map_entry *)PMU_DMEM_DQ_PIN_MAP_BASE;
	const struct pmu_pin_map_entry *entry = &table[lane];
	u8 res = entry->base_mask & (u8)mask;
	u32 i;

	for (i = 0U; i < entry->count; i++) {
		if (mask & (1U << entry->pairs[i].bit))
			res |= entry->pairs[i].match;
	}

	return res;
}

/**
 * pmu_cal_dbyte_gate_mask_clear() - Clear DBYTE gate mask outside active range
 */
noinline void pmu_cal_dbyte_gate_mask_clear(void)
{
	const __volatile__ u8 *range =
		(const __volatile__ u8 *)PMU_DMEM_CH_RANGE_B68;
	u32 min_ch;
	u32 max_ch;
	u32 ch_limit = pmu_sdata()->dbyte_slice_cnt;
	u32 ch;

	min_ch = range[0];
	max_ch = range[1];
	for (ch = 0U; ch < ch_limit; ch++) {
		if (ch < min_ch || ch > max_ch)
			*(__volatile__ u16 *)(PHY_REG_DBYTE_GATE_1F6A |
					      (ch << 13)) = 0U;
	}
}

/**
 * pmu_cal_dbyte_delay_coarse_step() - Advance DBYTE coarse delay across channels
 */
static noinline void pmu_cal_dbyte_delay_coarse_step(void)
{
	const __volatile__ u8 *range =
		(const __volatile__ u8 *)PMU_DMEM_CH_RANGE_B68;
	u32 min_ch;
	u32 max_ch;
	u32 ch;

	min_ch = range[0];
	max_ch = range[1];
	for (ch = min_ch; ch <= max_ch; ch++) {
		__volatile__ u16 *reg =
			(__volatile__ u16 *)(PHY_REG_DBYTE_COARSE_154 |
					     (ch << 13));
		u16 cur = *reg;
		u16 val = (u16)(cur + 0x20U);

		if ((val & 0x03c0U) != 0U)
			val = (u16)((cur - 0x20U) | 0x0400U);
		*reg = val;
	}
}

/**
 * pmu_cal_eye_param_block_init() - Initialize 8-halfword 2D eye parameter block
 * @param_block: Output 8-halfword parameter block
 * @p1: First phase/delay parameter
 * @p2: Second phase/delay parameter
 * @p3: Control/rank selector parameter
 */
void pmu_cal_eye_param_block_init(u16 *param_block, u32 p1, u32 p2, u32 p3)
{
	u16 ctrl = (u16)((p3 << 3) & 0x0010U);
	u16 p3_hi = (u16)(p3 << 14);

	param_block[1] = 0U;
	param_block[2] = 0U;
	param_block[5] = 0U;
	param_block[6] = 0U;
	param_block[0] = (u16)((p1 << 7) | p3_hi | 88U);
	param_block[3] = ctrl;
	param_block[4] = (u16)(((p2 >> 1) & 0x001fU) | ((p2 << 7) & 0x3f80U) |
			       p3_hi | 8U);
	param_block[7] = ctrl;
}

/**
 * pmu_cal_marker_verify_and_buffer_clear() - Clear eye buffer and verify markers
 * @flags: Stage bitmask controlling bit 11 and bit 7 clear/verify passes
 */
void pmu_cal_marker_verify_and_buffer_clear(u32 flags)
{
	__volatile__ u32 *buf = (__volatile__ u32 *)PMU_DMEM_EYE_CLEAR_BUF_DAC;
	u32 zero = 0U;

	if ((flags & (1UL << 11)) != 0UL) {
		((__volatile__ u16 *)buf)[10] |= 1U;
		if ((pmu_sdata()->cal_marker_a & 0x07U) != 0U) {
			buf[0] = zero;
			buf[1] = zero;
			buf[2] = zero;
			buf[3] = zero;
		}
		pmu_cal_marker_verify();
	} else {
		if ((pmu_sdata()->cal_marker_a & 0x07U) == 0U)
			pmu_cal_marker_verify();
	}
	if ((flags & (1UL << 7)) != 0UL) {
		if ((pmu_sdata()->cal_marker_a & 0x07U) != 0U) {
			buf[0] = zero;
			buf[1] = zero;
			buf[2] = zero;
			buf[3] = zero;
		}
		pmu_cal_marker_verify();
	}
}

/**
 * pmu_cal_dbyte_delay_restore_all_ch() - Restore DBYTE WCK delays across channels
 */
static void pmu_cal_dbyte_delay_restore_all_ch(void)
{
	u32 slice = pmu_sdata()->active_slice_idx;
	const __volatile__ u16 *save =
		(const __volatile__ u16 *)PMU_DMEM_DBYTE_DLY_SAVE_430;
	const u8 *range = (const u8 *)PMU_DMEM_CH_ACTIVE_RANGE_B6A;
	u32 ch;

	*(__volatile__ u16 *)(PHY_REG_CAL_STAT_CTRL_022 + (slice << 1)) =
		dmem_read16(PMU_DMEM_SAVED_CAL_STAT_42A);

	if (dmem_read8(PMU_DMEM_CH0_ANALOG_EN_25) != 0U) {
		for (ch = range[0]; ch <= range[1]; ch++)
			*(__volatile__ u16 *)(PHY_REG_DBYTE_WCK_DLY_146 |
					      (ch << 13)) = save[ch];
	}
	if (dmem_read8(PMU_DMEM_CH1_ANALOG_EN_40) != 0U) {
		for (ch = range[2]; ch <= range[3]; ch++)
			*(__volatile__ u16 *)(PHY_REG_DBYTE_WCK_DLY_146 |
					      (ch << 13)) = save[ch];
	}
}

/**
 * pmu_cal_dbyte_delay_backup_all_ch() - Backup DBYTE WCK delays across channels
 */
void pmu_cal_dbyte_delay_backup_all_ch(void)
{
	u32 slice = pmu_sdata()->active_slice_idx;
	__volatile__ u16 *stat_reg =
		(__volatile__ u16 *)(PHY_REG_CAL_STAT_CTRL_022 + (slice << 1));
	u16 orig = *stat_reg;
	__volatile__ u16 *save =
		(__volatile__ u16 *)PMU_DMEM_DBYTE_DLY_SAVE_430;
	const u8 *range = (const u8 *)PMU_DMEM_CH_ACTIVE_RANGE_B6A;
	u32 reg_base = PHY_REG_DBYTE_WCK_DLY_146;
	u32 ch;

	dmem_write16(PMU_DMEM_SAVED_CAL_STAT_42A, orig);
	*stat_reg = (u16)(orig & 0xff9fU);

	if (dmem_read8(PMU_DMEM_CH0_ANALOG_EN_25) != 0U) {
		for (ch = range[0]; ch <= range[1]; ch++)
			save[ch] = *(__volatile__ const u16 *)
				(reg_base | (ch << 13));
	}
	if (dmem_read8(PMU_DMEM_CH1_ANALOG_EN_40) != 0U) {
		for (ch = range[2]; ch <= range[3]; ch++)
			save[ch] = *(__volatile__ const u16 *)
				(reg_base | (ch << 13));
	}
}

/**
 * pmu_cal_vref_trim_cfg_latch() - Save AC slice calibration status registers
 */
void pmu_cal_vref_trim_cfg_latch(void)
{
	__volatile__ u8 *base = (__volatile__ u8 *)PMU_DMEM_CAL_TABLE_43C;
	u32 s;

	for (s = 0U; s < 2U; s++) {
		const __volatile__ u16 *ac =
			(const __volatile__ u16 *)(PHY_REG_AC_SLICE_CFG_05C |
						   (s << 13));
		__volatile__ u16 *bh = (__volatile__ u16 *)(base + (s << 1));
		__volatile__ u8 *bb = base + s;

		bh[4] = ac[20];
		bh[10] = ac[18];
		bh[12] = ac[19];
		bb[0x0e] = (u8)ac[5];
		bb[0x10] = (u8)ac[6];
		bb[0x0c] = (u8)ac[0];
	}
}

/**
 * pmu_cal_dbyte_sample_ready_poll_all() - Configure DBYTE slice and lane masks
 * @flags: Active lane bitmask (0xff for broadcast gate, or bits 0..8 per lane)
 */
void pmu_cal_dbyte_sample_ready_poll_all(u32 flags)
{
	const __volatile__ u8 *range =
		(const __volatile__ u8 *)PMU_DMEM_CH_RANGE_B68;
	u32 min_ch;
	u32 max_ch;
	u32 ch;

	min_ch = range[0];
	max_ch = range[1];
	*(__volatile__ u16 *)PHY_REG_DBYTE_SAMPLE_3FF6A = 0U;
	*(__volatile__ u16 *)PHY_REG_DBYTE_SAMPLE_3F16A = 1U;

	for (ch = min_ch; ch <= max_ch; ch++) {
		u32 base = PHY_REG_DBYTE_SAMPLE_16A | (ch << 13);

		if ((u8)flags == 0xffU) {
			*(__volatile__ u16 *)(base | 0x1e00UL) = 1U;
			if ((flags & 0x0100U) == 0U)
				*(__volatile__ u16 *)PHY_REG_DBYTE_SAMPLE_3F16A =
					0U;
		} else {
			u32 bitmask = flags & 0x01ffU;

			for (; bitmask != 0U; bitmask >>= 1, base += 0x0200UL) {
				if ((bitmask & 1U) != 0U)
					*(__volatile__ u16 *)base = 1U;
			}
		}
	}
}

/**
 * pmu_cal_vref_search_center_update() - Program calibrated RX VREF across DBYTE slices
 */
void pmu_cal_vref_search_center_update(void)
{
	u32 count = pmu_sdata()->active_slice_cnt;
	u32 param41c = pmu_sdata()->active_csr_offset;
	u8 param_40 = dmem_read8(PMU_DMEM_CH1_ANALOG_EN_40);
	u8 param_25 = dmem_read8(PMU_DMEM_CH0_ANALOG_EN_25);
	u32 half_count = count >> 1;
	const __volatile__ u16 *p =
		(const __volatile__ u16 *)PMU_DMEM_CAL_TABLE_43C;
	u32 i;

	for (i = 0U; i <= count; i++) {
		u16 val;

		if (param_40 != 0U && i >= half_count) {
			val = p[1];
			if (param_40 == 3U && p[3] > val)
				val = p[3];
		} else {
			val = p[0];
			if (param_25 == 3U && p[2] > val)
				val = p[2];
		}

		*(__volatile__ u16 *)(PHY_REG_DBYTE_SLICE_BASE |
				      (((param41c | (i << 12)) << 1))) = val;
	}
}

/**
 * pmu_cal_pad_timing_step_settle() - Latch deskew pad timing and settle 50 ms
 */
void pmu_cal_pad_timing_step_settle(void)
{
	pmu_set_phy_44(300U);
	pmu_set_phy_46(pmu_sdata()->deskew_target_dly);
	*(__volatile__ u16 *)PHY_REG_PLL_CLK_CTRL = 1U;
	pmu_cal_slice_timing_latch(0U, 0U);
	pmu_delay_us(50000UL, 15U);
}

/**
 * pmu_cal_dbyte_bcast_pulse_width_clear() - Apply descriptor pulse width config
 * @cfg: Pointer to slice calibration descriptor
 */
void pmu_cal_dbyte_bcast_pulse_width_clear(const u8 *cfg)
{
	u32 offset = pmu_sdata()->active_slice_idx << 1;
	u8 cfg_word = cfg[0x14];
	u8 bist_flag = cfg[0x01];
	u16 val0 = (u16)((cfg_word & 0x0cU) | ((bist_flag >> 3) & 1U));
	u16 is_zero = ((cfg_word & 3U) == 0U) ? 1U : 0U;

	*(__volatile__ u16 *)(PHY_REG_PUB_ACK_02A | offset) = val0;
	*(__volatile__ u16 *)(PHY_REG_DBYTE_BCAST_WIDTH | offset) =
		(u16)(is_zero << 1);
	*(__volatile__ u16 *)(PHY_REG_DBYTE_BCAST_CTRL_00A | offset) = is_zero;
}

/**
 * pmu_dbyte_deskew_phase_sample() - Sample and pack 4-step DBYTE gate status masks
 * @ch_base: Channel group base index
 * @out_mask: Output pointer receiving accumulated 16-bit lane mask
 * @sel_mask: 2-bit step enable selector mask
 */
void pmu_dbyte_deskew_phase_sample(u32 ch_base, u32 *out_mask, u32 sel_mask)
{
	const u8 *b = (const u8 *)PMU_DMEM_SLICE_CNT_LIMIT_B20;
	u32 base_ch = b[70];
	const u8 *slice_ptr = b + 36 + (base_ch << 4) + (ch_base << 3);
	u32 slice_limit = b[(base_ch << 1) + ch_base];
	u32 step;

	*out_mask = 0U;
	for (step = 0U; step < 4U; step++) {
		const u16 *lut = (const u16 *)PMU_DMEM_DELAY_TAP_LUT_E7B8 +
				 step;
		u32 s;

		if ((sel_mask & (1U << (step & 1U))) == 0U)
			continue;

		pmu_set_phy_44(lut[0]);
		pmu_set_phy_46(lut[6]);
		pmu_cal_slice_timing_latch(0U, 1U);

		for (s = 0U; s < slice_limit; s++) {
			u8 slice_off = slice_ptr[s];
			u16 raw_val = *(__volatile__ const u16 *)
				(PHY_REG_DBYTE_GATE_STAT_11A |
				 ((u32)slice_off << 13));
			u32 mapped;

			mapped = pmu_map_mask(raw_val, slice_off) & 0x7fU;

			if (step < 2U)
				mapped ^= 0x7fU;
			*out_mask |= (mapped << ((step & 1U) << 3));
		}
	}
}

/**
 * pmu_dbyte_cal_step_latch() - Format and stream 16-halfword fine/coarse descriptor
 * @fine_val: Fine delay value (low 7 bits)
 * @coarse_val: Coarse delay value (low 7 bits + upper step bits)
 * @ch_flags: Channel/slice configuration flags
 */
void pmu_dbyte_cal_step_latch(u32 fine_val, u32 coarse_val, u32 ch_flags)
{
	u32 buf32[8];
	u32 ctrl_hi = ((ch_flags << 3) & 0x10U) << 16;
	u32 slice_cfg_field = (ch_flags & 0x03U) << 14;
	u32 a0 = fine_val & 0x7fU;
	u32 a1 = coarse_val & 0x7fU;
	u32 step_val_mid = ((coarse_val >> 1) & 0xffc0U) | 8U;

	buf32[0] = slice_cfg_field | 0x2c58U;
	buf32[1] = ctrl_hi;
	buf32[2] = a0 | (a0 << 7);
	buf32[3] = 0U;
	buf32[4] = (u16)((step_val_mid << 7) | slice_cfg_field | step_val_mid);
	buf32[5] = ctrl_hi;
	buf32[6] = a1 | (a1 << 7);
	buf32[7] = 0U;

	pmu_cal_tracker_reset_and_apb_zero((u16 *)buf32, 16U);
}

/**
 * pmu_ac_step_coord_adjust() - Adjust AC timing step coordinate by signed offset
 * @struct_ptr: Pointer to 132-byte eye/AC scan descriptor structure
 *
 * Return: 0 on valid step window, or 7 on out-of-bounds error.
 */
static int pmu_ac_step_coord_adjust(u8 *struct_ptr)
{
	s8 shift_cfg = *(__volatile__ const s8 *)PMU_DMEM_SIGNED_STEP_0E;
	s32 step_shift = shift_cfg >> 1;
	s8 cur_step;
	u8 target_val;
	s32 edge = 0;
	s32 new_step;
	s8 step_idx;
	u8 lower, upper;

	if (step_shift == 0)
		return 0;

	cur_step = (s8)struct_ptr[130];
	target_val = struct_ptr[131];

	if (step_shift > 0) {
		if (cur_step >= 0) {
			s32 k;

			for (k = cur_step; k >= 0; k--) {
				lower = struct_ptr[k * 2];
				upper = struct_ptr[k * 2 + 1];
				if (upper < target_val || target_val < lower) {
					edge = k;
					break;
				}
			}
		}
	} else {
		if (cur_step <= 0x3f) {
			s32 k;

			for (k = cur_step; k < 0x40; k++) {
				lower = struct_ptr[k * 2];
				upper = struct_ptr[k * 2 + 1];
				if (upper < target_val || target_val < lower) {
					edge = k;
					break;
				}
				if (k == 0x3f)
					edge = 0x3f;
			}
		}
	}

	new_step = edge + step_shift;
	struct_ptr[130] = (u8)new_step;

	step_idx = (s8)new_step;
	if (step_idx < 0 || step_idx >= 0x40)
		goto err;

	lower = struct_ptr[step_idx * 2];
	upper = struct_ptr[step_idx * 2 + 1];
	if (upper < target_val || target_val < lower)
		goto err;

	return 0;

err:
	return 7;
}

/**
 * pmu_ac_step_pair_align_verify() - Align pair of AC timing step descriptors and verify
 * @struct_a: First 132-byte eye/AC scan descriptor structure
 * @struct_b: Second 132-byte eye/AC scan descriptor structure
 *
 * Return: 0 on valid alignment, or non-zero error code (7 on window violation).
 */
static int pmu_ac_step_pair_align_verify(u8 *struct_a, u8 *struct_b)
{
	s8 shift_cfg = *(__volatile__ const s8 *)PMU_DMEM_SIGNED_STEP_0E;
	s32 fp = shift_cfg >> 1;
	int ret;
	u16 base_b, base_a;
	s8 step_b, step_a;
	u16 total_b, total_a;
	s8 final_a, final_b;
	u8 target_a, target_b;

	if (fp == 0)
		return 0;

	ret = pmu_ac_step_coord_adjust(struct_a);
	if (ret != 0)
		return ret;

	ret = pmu_ac_step_coord_adjust(struct_b);
	if (ret != 0)
		return ret;

	base_b = *(__volatile__ const u16 *)(struct_b + 128);
	step_b = (s8)struct_b[130];
	base_a = *(__volatile__ const u16 *)(struct_a + 128);
	step_a = (s8)struct_a[130];

	total_b = (u16)(step_b + base_b);
	total_a = (u16)(step_a + base_a);

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

	final_a = (s8)struct_a[130];
	target_a = struct_a[131];
	if (target_a < struct_a[final_a * 2] ||
	    struct_a[final_a * 2 + 1] < target_a)
		return 7;

	final_b = (s8)struct_b[130];
	target_b = struct_b[131];
	if (target_b < struct_b[final_b * 2] ||
	    struct_b[final_b * 2 + 1] < target_b)
		return 7;

	return 0;
}

/**
 * pmu_phy_delay_bound_margin_log() - Measure DBYTE period and compute gate stage margin
 * @slice: Target DBYTE slice index
 *
 * Return: Computed 16-bit period-normalized gate delay difference.
 */
u16 pmu_phy_delay_bound_margin_log(u32 slice)
{
	u32 base = PHY_REG_DBYTE_SLICE_BASE | (slice << 13);
	__volatile__ u16 *sreg;
	const u16 *cbase;
	u16 val;
	u32 delay_stage;
	u32 stage_offset = 0U;
	u32 period_val;
	u32 quot, bound = 160U, diff;

	sreg = (__volatile__ u16 *)base;
	val = sreg[0x158 >> 1];
	sreg[0x158 >> 1] = val | 1U;
	sreg[0x158 >> 1] = val & ~1U;

	cbase = (const u16 *)(base | (pmu_sdata()->active_csr_offset << 1));
	delay_stage = cbase[0x15a >> 1];

	if (delay_stage < 5U)
		stage_offset = cbase[delay_stage + 0xd0U];

	period_val = ((const u16 *)base)[0x1aa >> 1];
	quot = (((period_val * delay_stage) - stage_offset) << 6) / period_val;

	if (pmu_sdata()->freq_mode >= 2U)
		bound += 128U;
	diff = bound - quot;

	return (u16)diff;
}

/**
 * pmu_phy_profile_param_program() - Program AC slice profile parameters and settle
 */
void pmu_phy_profile_param_program(void)
{
	__volatile__ u16 *phy_base = (__volatile__ u16 *)PHY_REG_MASTER_BASE;
	u16 orig_0f4 = phy_base[0x00f4 >> 1];
	u16 orig_0fc = phy_base[0x00fc >> 1];

	phy_base[0x00fc >> 1] = orig_0fc & 0xff33U;
	phy_base[0x00f4 >> 1] = orig_0f4 | 0x00ccU;

	if (*(__volatile__ const u8 *)PMU_DMEM_AC_PROFILE_EN_F4 != 0U) {
		u32 csr_offset;
		u32 slice;

		csr_offset = *(__volatile__ const u32 *)
			PMU_DMEM_PSTATE_OFFSET_41C;

		for (slice = 0U; slice < 2U; slice++) {
			u32 slice_csr_off = ((slice << 12) | csr_offset) << 1;
			__volatile__ u16 *slice_base = (__volatile__ u16 *)
				(slice_csr_off | PHY_REG_AC_SLICE_PROFILE);
			const __volatile__ u8 *cfg44 =
				(const __volatile__ u8 *)
				(PMU_DMEM_AC_PROFILE_CFG_444 + slice * 2U);
			u16 val0 = *(const __volatile__ u16 *)cfg44;
			const __volatile__ u8 *cfg48 =
				(const __volatile__ u8 *)
				(PMU_DMEM_AC_PROFILE_CFG_448 + slice);

			slice_base[0x0084 >> 1] = val0;
			slice_base[0x0086 >> 1] = val0;
			slice_base[0x0080 >> 1] =
				*(const __volatile__ u16 *)(cfg44 + 0x0cU);
			slice_base[0x0082 >> 1] =
				*(const __volatile__ u16 *)(cfg44 + 0x10U);

			slice_base[0x005c >> 1] = cfg48[0];
			slice_base[0x0066 >> 1] = cfg48[2];
			slice_base[0x0068 >> 1] = cfg48[4];
		}
	}

	phy_base[0x0084 >> 1] = 0U;
	phy_base[0x0088 >> 1] = 0U;
	pmu_delay_us(11000U, 0U);

	phy_base[0x00f4 >> 1] = orig_0f4;
	phy_base[0x00fc >> 1] = orig_0fc;
}

/**
 * pmu_dbyte_cal_strobe_pulse_seq() - Reset or trigger DBYTE broadcast strobe sequence
 * @mode: 0 to de-assert/reset strobe registers, non-zero to trigger strobe
 * @val: Broadcast strobe delay halfword value
 * @step: Broadcast strobe step mode (0 or 1; > 1 triggers assertion)
 */
void pmu_dbyte_cal_strobe_pulse_seq(u32 mode, u32 val, u32 step)
{
	__volatile__ u16 *apb =
		(__volatile__ u16 *)PHY_REG_DBYTE_BCAST_STROBE_080;
	__volatile__ u16 *apb_csr;

	apb_csr = apb + pmu_sdata()->active_csr_offset;
	if ((u8)mode == 0U) {
		apb_csr[-53] = 0U;
		apb_csr[-59] = 0U;
		pmu_hw_timer_delay(20U);
		apb[-54] = 1U;
		apb[74] = 0U;
		apb[76] = 0U;
		apb[84] = 0U;
		apb[73] = 0U;
	} else {
		apb[74] = (u16)val;
		apb[73] = (u16)val;
		apb[75] = 0U;
		if ((u8)step == 0U) {
			apb[-54] = 0U;
			apb_csr[-59] = 1U;
		} else if ((u8)step == 1U) {
			apb[-54] = 0U;
			apb[85] = 10U;
		} else {
			pmu_assert_or_halt(0U, PMU_ERR_VREF_STEP_DIR_INVALID);
		}
		apb[84] = (u16)step;
		apb_csr[-53] = 2U;
	}
}

/**
 * pmu_rank_shadow_status_sample() - Sample shadow calibration status across ranks
 */
void pmu_rank_shadow_status_sample(void)
{
	u32 total_idx = 0U;
	u8 *table_base = (u8 *)PMU_DMEM_EYE_SWEEP_STAT_B82;
	u32 rank;

	for (rank = 0U; rank < 2U; rank++) {
		u8 active = (rank == 0U) ?
			    dmem_read8(PMU_DMEM_CH0_ANALOG_EN_25) :
			    dmem_read8(PMU_DMEM_CH1_ANALOG_EN_40);

		if (active != 0U) {
			u16 base_val = dmem_read16(PMU_DMEM_PLL_GATE_E8 +
						   rank * 2U);
			u32 base_reg = ((u32)base_val << 2) | 0x00041000UL;
			u32 fp;

			for (fp = 0U; fp < 2U; fp++) {
				u8 count = dmem_read8(PMU_DMEM_METRIC_TABLE_ENTRIES);
				u8 *entry = table_base + fp;
				u32 entry_idx;

				for (entry_idx = 0U; entry_idx < count;
				     entry_idx++) {
					u16 val = *(__volatile__ const u16 *)
						(PHY_REG_APB_BASE |
						 ((base_reg +
						   (u8)total_idx) << 1));

					*entry = (u8)val;
					entry += 4U;
					total_idx++;
				}
			}
		}
		table_base += 2U;
	}
}

/**
 * pmu_phy_lcdl_delay_read() - Read and unpack DBYTE coarse/fine lane delays
 * @out_arr: Output halfword array
 * @flag: Non-zero to unpack and sum 10-bit fine and 6-bit coarse subfields
 * @mode: Lane range/stride selector (1 = lane 8 stride 9, 2 = lane 0, else 0..8)
 */
void pmu_phy_lcdl_delay_read(u16 *out_arr, u32 flag, u32 mode)
{
	const u8 *b68 = (const u8 *)PMU_DMEM_CH_RANGE_B68;
	u8 start_ch, end_ch;
	u32 start_lane = 0U, end_lane = 8U, stride = 1U;
	u16 *out;
	u32 ch, lane;

	start_ch = b68[0];
	end_ch = b68[1];

	if (mode == 2U) {
		end_lane = 0U;
	} else if (mode == 1U) {
		stride = 9U;
		start_lane = 8U;
	}

	out = out_arr + ((mode == 2U) ? start_ch :
			 (start_ch * 9U + ((mode == 1U) ? 8U : 0U)));

	for (ch = start_ch; ch <= end_ch; ch++) {
		for (lane = start_lane; lane <= end_lane; lane++) {
			u32 reg = PHY_REG_DBYTE_COARSE_154 |
				  (ch << 13) | (lane << 9);
			u16 val = *(__volatile__ const u16 *)reg;

			if (flag != 0U)
				val = (val & 0x03ffU) + ((val >> 4) & 0x0fc0U);
			*out = val;
			out += stride;
		}
	}
}

/**
 * pmu_eye_margin_step_align() - Align coarse/fine eye margin pair with clamping
 * @margin_cur: First 2-byte [coarse, fine] margin pair
 * @margin_target: Second 2-byte [coarse, fine] margin pair
 * @step_size: Coarse alignment step size
 */
void pmu_eye_margin_step_align(u8 *margin_cur, u8 *margin_target,
			       u32 step_size)
{
	u8 v0 = margin_cur[0];
	u8 v1 = margin_target[0];
	u8 step = (u8)step_size;

	if (v0 > v1) {
		if (v0 < step) {
			margin_cur[0] = v0 - 1U;
			margin_cur[1] = (u8)(margin_cur[1] + 64U);
			if ((s8)margin_cur[1] < 0)
				margin_cur[1] = 0x7fU;
		} else {
			margin_cur[0] = v0 - step;
			if (step == 1U) {
				margin_cur[1] = (u8)(margin_cur[1] + 64U);
				if ((s8)margin_cur[1] < 0)
					margin_cur[1] = 0x7fU;
			} else if (step == 2U || (s8)margin_cur[1] < 0) {
				margin_cur[1] = 0x7fU;
			}
		}
	} else if (v0 < v1) {
		if (v1 < step) {
			margin_target[0] = v1 - 1U;
			margin_target[1] = (u8)(margin_target[1] + 64U);
			if ((s8)margin_target[1] < 0)
				margin_target[1] = 0x7fU;
		} else {
			margin_target[0] = v1 - step;
			if (step == 1U) {
				margin_target[1] = (u8)(margin_target[1] + 64U);
				if ((s8)margin_target[1] < 0)
					margin_target[1] = 0x7fU;
			} else if (step == 2U || (s8)margin_target[1] < 0) {
				margin_target[1] = 0x7fU;
			}
		}
	}

	pmu_assert_or_halt(margin_cur[0] == margin_target[0],
			   PMU_ERR_EYE_STEP_ALIGN_MISMATCH);
}

/**
 * pmu_cbt_3phase_pulse_seq() - Execute 3-step fixed-unit pad timing delay sequence
 */
void pmu_cbt_3phase_pulse_seq(void)
{
	pmu_cal_pad_timing_delay_set(0U, 0x10U, 1U);
	pmu_cal_pad_timing_delay_set(0x10U, 0U, 1U);
	pmu_cal_pad_timing_delay_set(0U, 0x0aU, 1U);
}

/**
 * pmu_eye_margin_bidir_scan() - Scan 2D eye margins and update minimum X/Y bounds
 * @buf: Pointer to 132-byte eye scan buffer (center at +130, threshold at +131)
 * @flag_a: Rank/slice flag A (+2 offset when == 1)
 * @flag_b: Channel/group flag B (+27 offset when != 0)
 * @flag_c: Mode flag C (+4 offset when == 1)
 */
static void pmu_eye_margin_bidir_scan(u8 *buf, u32 flag_a, u32 flag_b, u32 flag_c)
{
	u8 center = buf[130];
	u8 th = buf[131];
	s32 r0_left = (s32)center;
	s32 r0_right = (s32)center;
	u32 dest = (flag_b != 0U) ? 0x52U : 0x37U;
	s32 margin_x_left, margin_x_right, min_x;
	s32 margin_y_lo, margin_y_hi, min_y;
	u8 *d_ptr;

	for (; r0_left >= 0; r0_left--) {
		if (buf[r0_left * 2] >= th || th >= buf[r0_left * 2 + 1])
			break;
	}

	if (center <= 0x3fU) {
		for (; r0_right < 64; r0_right++) {
			if (buf[r0_right * 2] >= th ||
			    th >= buf[r0_right * 2 + 1])
				break;
		}
	}

	if (flag_c == 1U)
		dest += 4U;
	if (flag_a == 1U)
		dest += 2U;
	dest |= PMU_DMEM_BASE;
	d_ptr = (u8 *)dest;

	margin_x_left = (s32)center - r0_left;
	margin_x_right = r0_right - (s32)center;
	min_x = (margin_x_left < margin_x_right) ?
		margin_x_left : margin_x_right;

	margin_y_lo = (s32)th - (s32)buf[center * 2];
	margin_y_hi = (s32)buf[center * 2 + 1] - (s32)th;
	min_y = (margin_y_lo < margin_y_hi) ? margin_y_lo : margin_y_hi;

	if ((u8)min_x < d_ptr[0])
		d_ptr[0] = (u8)min_x;
	if ((u8)min_y < d_ptr[1])
		d_ptr[1] = (u8)min_y;
}

/**
 * pmu_rank_slice_margin_eval() - Evaluate AC timing step margins across active ranks/slices
 */
void pmu_rank_slice_margin_eval(void)
{
	u32 mode = pmu_phy_query_operating_mode();
	u32 rank;

	for (rank = 0U; rank < 2U; rank++) {
		u8 ch0_ranks = *(__volatile__ const u8 *)PMU_DMEM_CH0_ANALOG_EN;
		u8 ch1_ranks = *(__volatile__ const u8 *)
			PMU_DMEM_CH1_ACTIVE_FLAG;
		const u8 *b68 = (const u8 *)PMU_DMEM_CH_RANGE_B68;
		u32 slice, end_slice;

		if (((ch0_ranks | ch1_ranks) & (1U << rank)) == 0U)
			continue;

		slice = b68[0];
		end_slice = b68[1];

		dmem_write8(PMU_DMEM_STAGE_PARAM_406, (u8)rank);

		for (; slice <= end_slice; slice++) {
			u8 mask = b68[0x30];
			u8 *ptr_3708;
			u8 *ptr_dc8;
			u32 lane;

			if ((mask & (1U << slice)) == 0U)
				continue;

			dmem_write8(PMU_DMEM_STAGE_PARAM_407, (u8)slice);

			ptr_3708 = (u8 *)(PMU_DMEM_AC_TIMING_DESC_3708 +
					  rank * 5280U + slice * 1320U);
			ptr_dc8 = (u8 *)(PMU_DMEM_AC_TIMING_TBL_DC8 +
					 rank * 5280U + slice * 1320U);

			for (lane = 0U; lane < 10U; lane++) {
				if ((mode & (1U << lane)) != 0U) {
					int ret;

					dmem_write8(PMU_DMEM_STAGE_PARAM_408,
						    (u8)lane);
					if (mode == 3U)
						ret = pmu_ac_step_pair_align_verify(ptr_dc8,
										    ptr_3708);
					else
						ret = pmu_ac_step_coord_adjust(ptr_dc8);

					pmu_assert_or_halt(ret == 0,
							   PMU_ERR_AC_STEP_COORD_ADJUST);
				}
				ptr_3708 += 132U;
				ptr_dc8 += 132U;
			}
		}
	}
}

struct pmu_eye_sample_entry {
	s16 min_x;
	s16 max_x;
	s16 min_y;
	s16 max_y;
};

/**
 * pmu_eye_sample_table_transform() - Transform 64-pair eye sample array into 4-bound table
 * @dest: Output 512-byte (64 x 8-byte) eye sample bound array
 * @src: Input 128-byte (64 x 2-byte) raw eye sample pair array
 */
static void pmu_eye_sample_table_transform(u8 *dest, const u8 *src)
{
	struct pmu_eye_sample_entry *d = (struct pmu_eye_sample_entry *)dest;
	u32 i;
	u8 s0, s1, s126, s127;

	for (i = 0U; i < 64U; i++) {
		d[i].min_x = 0x7f;
		d[i].max_x = 0;
		d[i].min_y = 0x7f;
		d[i].max_y = 0;
	}

	for (i = 1U; i < 63U; i++) {
		const u8 *s_curr = src + (i << 1);
		const u8 *s_prev = s_curr - 2;
		const u8 *s_next = s_curr + 2;
		u8 c0 = s_curr[0];
		u8 c1 = s_curr[1];

		if (c1 >= c0) {
			s16 min_x = (s16)(s_next[0] + 1);
			s16 max_y = (s16)(s_prev[1] - 1);

			d[i].max_x = (s16)(c0 - 1);
			if (min_x > (s16)(s_prev[0] + 1))
				min_x = (s16)(s_prev[0] + 1);
			if (min_x > (s16)c0)
				min_x = (s16)c0;
			d[i].min_x = min_x - 1;

			d[i].min_y = (s16)(c1 + 1);
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

	s0 = src[0];
	s1 = src[1];
	if (s1 >= s0) {
		s16 min_x = (s16)(src[2] + 1);
		s16 max_y = (s16)(src[3] - 1);

		d[0].max_x = (s16)(s0 - 1);
		if (min_x > (s16)s0)
			min_x = (s16)s0;
		d[0].min_x = min_x - 1;
		d[0].min_y = (s16)(s1 + 1);
		if (max_y < (s16)s1)
			max_y = (s16)s1;
		d[0].max_y = max_y + 1;
	} else if (src[3] >= src[2]) {
		d[0].min_x = src[2];
		d[0].max_x = src[3];
	}

	s126 = src[126];
	s127 = src[127];
	if (s127 >= s126) {
		s16 min_x = (s16)(src[124] + 1);
		s16 max_y = (s16)(src[125] - 1);

		d[63].max_x = (s16)(s126 - 1);
		if (min_x > (s16)s126)
			min_x = (s16)s126;
		d[63].min_x = min_x - 1;
		d[63].min_y = (s16)(s127 + 1);
		if (max_y < (s16)s127)
			max_y = (s16)s127;
		d[63].max_y = max_y + 1;
	} else if (src[125] >= src[124]) {
		d[63].min_x = src[124];
		d[63].max_x = src[125];
	}

	for (i = 1U; i < 63U; i++) {
		if (d[i].min_x == -1) {
			d[i].min_x = 0x7f;
			d[i].max_x = 0;
		}
	}
}

/**
 * pmu_eye_centroid_avg_calc() - Compute average, right, and left eye bounds per lane
 * @base_delays: Per-slice base delay halfword array
 * @out_left: Output midpoint/average delay array (40 halfwords)
 * @out_right: Output right bound delay array (40 halfwords)
 * @out_width: Output left bound delay array (40 halfwords)
 */
void pmu_eye_centroid_avg_calc(const u16 *base_delays, u16 *out_left,
			       u16 *out_right, u16 *out_width)
{
	u8 train_flag15 = *(__volatile__ const u8 *)PMU_DMEM_ANALOG_CFG_15;
	s32 min_margin = ((train_flag15 & 1U) != 0U) ? -44 : -128;
	const u8 *b68 = (const u8 *)PMU_DMEM_CH_RANGE_B68;
	u8 mask_b98;
	u32 slice, end_slice;
	s32 target_margin = (s32)0xd4 & min_margin;
	const __volatile__ u8 *lane_base =
		(const __volatile__ u8 *)(PMU_DMEM_BASE |
					  PMU_DMEM_SWEEP_SAMPLE_BUF);

	slice = b68[0];
	end_slice = b68[1];
	mask_b98 = b68[0x30];

	for (; slice <= end_slice; slice++) {
		u32 lane_idx;
		u32 lane_sample_offset;
		u16 base_val;
		u32 lane;

		if ((mask_b98 & (1U << slice)) == 0U)
			continue;

		lane_idx = slice * 10U;
		lane_sample_offset = 848U * lane_idx;
		base_val = base_delays[slice];

		for (lane = 0U; lane < 10U;
		     lane++, lane_idx++, lane_sample_offset += 848U) {
			const __volatile__ u8 *p =
				lane_base + lane_sample_offset;
			u8 type = p[0x27c];
			u32 right_bound = 0U;
			s32 left_bound = 0;
			s32 avg;

			if (type != 0U) {
				right_bound = p[0];
				if (type == 1U) {
					left_bound = min_margin;
				} else if (type == 3U) {
					u8 val_lo = p[1];
					u8 val_hi = p[2];
					s32 span = (s32)val_lo -
						   (s32)right_bound;
					s32 rem_span = target_margin -
						       (s32)val_hi;

					if (span < rem_span) {
						right_bound = val_hi;
						left_bound = min_margin;
					}
				}
			}

			avg = ((s16)right_bound + (s8)left_bound) / 2;
			out_left[lane_idx] = base_val + (u16)avg;
			out_right[lane_idx] = base_val + (u16)right_bound;
			out_width[lane_idx] = base_val + (u16)(s8)left_bound;
		}
	}
}

/**
 * pmu_dbyte_coarse_step_mask_sweep() - Sweep 8 lanes at coarse step and return mapped mask
 * @ch_idx: Target DBYTE channel/slice index
 * @step_val: Coarse delay step offset
 * @flag: Mode 2 sub-selector flag
 *
 * Return: Remapped 8-bit lane status mask via pmu_map_mask().
 */
u32 pmu_dbyte_coarse_step_mask_sweep(u32 ch_idx, u32 step_val, u32 flag)
{
	u32 mode = pmu_phy_query_operating_mode();
	u32 reg_idx = (mode == 1U || (mode == 2U && flag == 1U)) ?
		      0x74U : 0x73U;
	u32 ch_shift = (ch_idx & 0xffffU) << 12;
	__volatile__ u16 *fp = (__volatile__ u16 *)
		((ch_shift << 1) | PHY_REG_DBYTE_COARSE_STEP_0E4);
	__volatile__ const u16 *csr_base = (__volatile__ const u16 *)
		(PHY_REG_APB_BASE | (((reg_idx | (1U << 16) | ch_shift)) << 1));
	u8 dram_type = *(__volatile__ const u8 *)PMU_DMEM_FREQ_ROUND_BIAS;
	u32 sub_val = ((dram_type == 2U) ? 4U : 0U) + 8U;
	u16 table[8];
	u32 i;
	u32 lane_mask = 0U;
	__volatile__ const u16 *csr;

	*fp = 768U;

	csr = csr_base - 3;
	for (i = 0U; i < 8U; i++) {
		table[i] = (u16)((*csr >> 10) - sub_val);
		csr += 0x100U;
	}

	csr = csr_base;
	for (i = 0U; i < 8U; i++) {
		*fp = (u16)((step_val + table[i]) | 192U);
		pmu_hw_timer_delay(8U);
		if ((*csr & 1U) != 0U)
			lane_mask |= (1U << i);
		csr += 0x100U;
	}

	*fp = 0U;
	return pmu_map_mask((u16)lane_mask, ch_idx);
}

/**
 * pmu_phy_cal_strobe_latch_setup() - Restore DBYTE delays and latch sweep delay registers
 */
void pmu_phy_cal_strobe_latch_setup(void)
{
	u16 delay_a = (0xffU << 8) + 1U;
	u16 delay_b = (0xffU << 8) + 1U;
	u32 freq_mode;

	phy_write16(PHY_REG_CH0_ANALOG_PWR,
		    dmem_read8(PMU_DMEM_CH0_ANALOG_EN_25));
	phy_write16(PHY_REG_CH1_ANALOG_PWR,
		    dmem_read8(PMU_DMEM_CH1_ANALOG_EN_40));

	pmu_cal_dbyte_delay_restore_all_ch();
	pmu_slice_status_b97_save();

	if (dmem_read16(PMU_DMEM_DRAM_FREQ_MHZ) >= 3201U) {
		const __volatile__ u16 *p628 =
			(const __volatile__ u16 *)PHY_REG_CLK_PHASE_DELAY_628;

		delay_a = (u16)((p628[0] << 8) + 1U);
		delay_b = (u16)((p628[1] << 8) + 1U);
	}

	freq_mode = dmem_read8(PMU_DMEM_CBT_LANE_LIM_403);
	if (freq_mode >= 2U) {
		__volatile__ u16 *dst = (__volatile__ u16 *)
			(PHY_REG_CAL_LATCH_STREAM_1000 |
			 (pmu_sdata()->active_csr_offset << 1));

		if (freq_mode >= 8U) {
			dst[0x10 >> 1] = delay_a;
			dst[0x26 >> 1] = delay_b;
		} else {
			dst[0x06 >> 1] = delay_a;
			dst[0x12 >> 1] = delay_b;
		}
	}
}

/**
 * pmu_window_centroid_calc() - Compute 2D eye window centroid and weighted midpoint
 * @table_buf: Pointer to 132-byte eye scan window buffer (outputs at +130, +131)
 *
 * Return: 0 on valid eye window, or 3 when total window width is 0.
 */
u32 pmu_window_centroid_calc(void *table_buf)
{
	u8 *out = (u8 *)table_buf;
	u16 flag15 = *(__volatile__ const u16 *)(PMU_DMEM_BASE +
						 PMU_DMEM_DRAM_CFG_FLAGS);
	u32 weighting_mode = flag15 & 0x1800U;
	u32 weighted_sum = 0U;
	u32 sq_weighted_sum = 0U;
	u32 total_width = 0U;
	u32 sq_weight_sum = 0U;
	u32 log_weight_sum = 0U;
	u32 log_weighted_sum = 0U;
	u32 total_weighted = 0U;
	u32 i;
	u32 centroid_val, centroid_mid, weighted_avg, res_centroid;

	for (i = 0U; i < 64U; i++) {
		u8 v0 = out[i * 2U];
		u8 v1 = out[i * 2U + 1U];
		s32 diff = (s32)v1 - (s32)v0;
		u32 width, sum, prod;

		if (diff < 0)
			continue;

		width = (u32)diff + 1U;
		sum = (u32)v0 + (u32)v1;
		prod = width * sum;
		total_width += width;
		total_weighted += prod;

		if (weighting_mode == 0U) {
			weighted_sum += width * i;
		} else if (weighting_mode == 0x0800U) {
			u32 sq = width * width;

			sq_weight_sum += sq;
			sq_weighted_sum += sq * i;
		} else {
			u32 weight = 0U;
			u32 w = width >> 1;

			while (w > 0U) {
				weight++;
				w >>= 1;
			}
			log_weight_sum += weight;
			log_weighted_sum += weight * i;
		}
	}

	if (total_width == 0U)
		return 3U;

	centroid_val = (total_weighted / total_width) + 1U;
	centroid_mid = centroid_val >> 1;

	if (weighting_mode != 0U) {
		if (weighting_mode == 0x0800U) {
			log_weighted_sum = sq_weighted_sum;
			log_weight_sum = sq_weight_sum;
		}
		total_width = log_weight_sum;
		weighted_sum = log_weighted_sum;
	}

	weighted_avg = ((weighted_sum << 1) / total_width) + 1U;
	res_centroid = weighted_avg >> 1;

	out[131] = (u8)centroid_mid;
	out[130] = (u8)res_centroid;
	return 0U;
}

/**
 * pmu_slice_delay_step_program() - Program per-lane DBYTE delay step deltas across slices
 * @reg_base_off: Base CSR register offset
 * @step_deltas: Signed 16-bit step delta table
 * @format_mode: Non-zero for signed div/mod 64, 0 for packed 6-bit coarse / 10-bit fine
 * @shift_sel: Non-zero for 7-bit coarse shift, 0 for 6-bit coarse shift
 * @stride_flag: Non-zero for 10-lane stride, 0 for 9-lane stride
 * @single_lane_flag: Non-zero to program a single lane per slice
 */
void pmu_slice_delay_step_program(u32 reg_base_off, const s16 *step_deltas,
				  u32 format_mode, u32 shift_sel,
			    u32 stride_flag, u32 single_lane_flag)
{
	const u8 *b68 = (const u8 *)PMU_DMEM_CH_RANGE_B68;
	u32 start_rank;
	u32 end_rank;
	u32 lane_count;
	u32 lane_offset;
	u32 shift_amount;
	const s16 *deltas_ptr;
	u32 reg_csr;
	u32 rank;

	start_rank = b68[0];
	end_rank = b68[1];

	if (single_lane_flag != 0U) {
		lane_count = 1U;
		lane_offset = start_rank;
	} else {
		lane_count = (stride_flag != 0U) ? 10U : 9U;
		lane_offset = start_rank * lane_count;
	}

	shift_amount = (shift_sel != 0U) ? 7U : 6U;
	deltas_ptr = step_deltas + lane_offset;
	reg_csr = PHY_REG_DBYTE_BASE |
		  ((pmu_sdata()->active_csr_offset | reg_base_off) << 1);

	for (rank = start_rank; rank <= end_rank; rank++) {
		u32 reg_addr = reg_csr | (rank << 13);
		u32 i;

		for (i = 0U; i < lane_count; i++, reg_addr += 0x200U) {
			s32 delta = *deltas_ptr++;
			s32 coarse, fine;

			if (format_mode != 0U) {
				coarse = delta / 64;
				fine = delta % 64;
			} else {
				coarse = (delta & (s32)0xfc00) >> 10;
				fine = delta & (s32)0x03ff;
			}

			*(__volatile__ u16 *)reg_addr =
				(u16)fine | (u16)((u16)coarse << shift_amount);
		}
	}
}

/**
 * pmu_vref_signed_direction_pulse_dispatch() - Dispatch signed Vref direction pulse sequence
 * @slice_target: Target slice / rank selector
 * @vref_step: Vref step parameter
 */
void pmu_vref_signed_direction_pulse_dispatch(u32 slice_target, u32 vref_step)
{
	u32 zero = 0U;
	u32 cmd;

	*(__volatile__ u16 *)PHY_REG_DBYTE_BCAST_VREF_014 = zero;
	pmu_pulse_phy_140();
	pmu_cal_strobe_pulse_init();

	cmd = zero;
	cmd |= (1U << 21);
	pmu_cal_sequence_pulse_send(cmd, 5U, 1U, 256U, slice_target,
				    zero, zero);
	cmd = 0x80U;
	cmd |= (1U << 18);
	pmu_cal_sequence_pulse_send(cmd, 0x0fU, 0x20U, zero,
				    slice_target, zero, vref_step);

	pmu_cal_slice_step_diff_latch(zero, 1U);
	*(__volatile__ u16 *)PHY_REG_DBYTE_BCAST_VREF_014 = 1U;
}

/**
 * pmu_dbyte_rx_en_slice_pulse_eval() - Evaluate DBYTE RxEn slice pulse sequence
 * @mode: Non-zero to enable broadcast window valid before pulse, 0 to clear after
 * @slice_mask: Active slice bitmask
 */
void pmu_dbyte_rx_en_slice_pulse_eval(u32 mode, u32 slice_mask)
{
	u32 zero = 0U;

	pmu_cal_strobe_pulse_init();

	if (mode != 0U)
		*(__volatile__ u16 *)(PHY_REG_DBYTE_BCAST_VALID_002 |
				      (pmu_sdata()->active_csr_offset << 1)) = 1U;
	pmu_cal_sequence_pulse_send(zero, 6U, 0x22U, mode ? 4U : 0U, 0x2eU,
				    slice_mask, zero);
	if (mode == 0U)
		*(__volatile__ u16 *)(PHY_REG_DBYTE_BCAST_VALID_002 |
				      (pmu_sdata()->active_csr_offset << 1)) = zero;

	pmu_cal_slice_step_diff_latch(zero, 1U);
}

/**
 * pmu_rank_margin_channel_accum() - Accumulate rank/slice margin window and sum widths
 * @rank: Active rank index (0 or 1)
 * @start_slice: Starting DBYTE slice index
 * @end_slice: Ending DBYTE slice index
 * @flags: Frequency point count (defaults to 1 when 0)
 * @swap_mode: DQ swap filter mode (0, 1, or 2)
 *
 * Return: Accumulated 16-bit window width across 64 delay coordinates.
 */
u16 pmu_rank_margin_channel_accum(u32 rank, u32 start_slice, u32 end_slice,
				  u32 flags, u32 swap_mode)
{
	u8 temp_buf[128];
	u32 rank_offset = rank * 0x14a0U;
	u32 fp_count = (flags != 0U) ? flags : 1U;
	u32 slice;
	u32 total_width = 0U;
	int i;

	for (i = 0; i < 64; i++) {
		temp_buf[i * 2] = 0x00U;
		temp_buf[i * 2 + 1] = 0x7fU;
	}

	for (slice = start_slice; slice <= end_slice; slice++) {
		const u8 *sub;
		u32 fp;

		if (swap_mode == 2U) {
			u8 b = (u8)pmu_cal_lane_error_status_check(0U, slice);

			if (rank != 0U || b != 0U)
				continue;
		} else if (swap_mode == 1U) {
			u8 b = (u8)pmu_cal_lane_error_status_check(0U, slice);

			if (rank == 0U || b != 0U)
				continue;
		} else if (flags == 0U) {
			continue;
		}

		sub = (const u8 *)(PMU_DMEM_AC_TIMING_TBL_DC8 + rank_offset +
				   slice * 1320U);
		for (fp = fp_count; fp > 0U; fp--) {
			s8 offset = (s8)sub[130] - 31;

			pmu_window_margin_diff_calc(temp_buf, sub, temp_buf,
						    offset, 0);
			sub += 132U;
		}
	}

	for (i = 0; i < 64; i++) {
		u8 low = temp_buf[i * 2];
		u8 high = temp_buf[i * 2 + 1];

		if (high >= low)
			total_width += (high - low + 1U);
	}

	return (u16)total_width;
}

#pragma GCC push_options
#pragma GCC optimize("no-expensive-optimizations,no-tree-dominator-opts")
/**
 * pmu_window_margin_diff_calc() - Intersect two 64-entry window margin arrays with offset/delta
 * @src0: First 128-byte (64 x [low, high]) window margin array
 * @src1: Second 128-byte (64 x [low, high]) window margin array
 * @out_dest: Output 128-byte window margin array (may alias @src0 or @src1)
 * @offset: Signed index shift applied to @src1
 * @delta: Signed bound shift subtracted from @src1 bounds
 *
 * Return: Always 0.
 */
static noinline u32 pmu_window_margin_diff_calc(const u8 *src0, const u8 *src1,
						u8 *out_dest, s32 offset, s32 delta)
{
	u8 temp_buf[128];
	u8 *dest;
	s32 i;

	dest = (out_dest == src0 || out_dest == src1) ? temp_buf : out_dest;

	for (i = 0; i < 64; i++) {
		s32 j = offset + i;
		u8 *d = dest + i * 2;
		s32 h;
		u8 max_high;
		s32 l;
		u8 min_low;

		if ((u32)j > 63U || src0[i * 2 + 1] == 0U ||
		    src1[j * 2 + 1] == 0U) {
			*(u16 *)d = 0x00ffU;
			continue;
		}

		h = (s32)src1[j * 2 + 1] - delta;
		if (h > 127)
			h = 127;
		max_high = (src0[i * 2 + 1] < (u8)h) ?
			   src0[i * 2 + 1] : (u8)h;

		l = (s32)src1[j * 2] - delta;
		if (l < 0)
			l = 0;
		min_low = (src0[i * 2] > (u8)l) ? src0[i * 2] : (u8)l;

		if (min_low < max_high) {
			d[0] = min_low;
			d[1] = max_high;
		} else {
			*(u16 *)d = 0x00ffU;
		}
	}

	if (dest != out_dest)
		memcpy(out_dest, temp_buf, 128U);

	return 0U;
}

#pragma GCC pop_options

/**
 * min_u() - Return the minimum of two unsigned 32-bit integers
 * @a: First unsigned value
 * @b: Second unsigned value
 *
 * Return: Smaller of @a and @b.
 */
static __always_inline u32 min_u(u32 a, u32 b)
{
	return (a < b) ? a : b;
}

#pragma GCC push_options
#pragma GCC optimize("no-expensive-optimizations,no-tree-dominator-opts")
/**
 * pmu_eye_sample_metric_eval() - Evaluate 2D eye diagram sample quality score
 * @x: Candidate X delay coordinate (0..63)
 * @y: Candidate Y Vref coordinate (0..127)
 * @eye_scan_lut: 512-byte transformed scan boundary table
 * @samples: 256-byte eye sample margin table
 *
 * Return: Packed 32-bit quality score ((score << 8) | margin_sum), or 0 if invalid.
 */
static noinline u32 pmu_eye_sample_metric_eval(u8 x, u8 y, const u8 *eye_scan_lut,
					       const u8 *samples)
{
	const u8 *pair = samples + ((u32)x << 1);
	u8 low = pair[0];
	u8 high = pair[1];
	u32 coord;
	u8 min_err = 0xffU;
	u32 min_val = 0xffffffffUL;
	const s16 *p;
	u32 i;
	u32 dist;
	u16 w17;
	u16 w32;
	u32 x1;
	u32 x2;
	u32 y1;
	u32 score;
	u32 min_error_val;

	if (low >= high || low > y || high <= y)
		return 0U;

	coord = ((u32)y << 8) | x;
	p = (const s16 *)eye_scan_lut;

	for (i = 0U; i < 64U; i++) {
		u32 k;

		for (k = 0U; k < 4U; k += 2U) {
			s16 v0 = p[k];
			s16 v1 = p[k + 1U];

			if (v1 >= v0) {
				u8 out_byte;
				u32 ret;

				ret = pmu_cal_eye_margin_quadratic_cost_calc(coord, i,
									     (u32)(s32)(s8)v0,
									     (u32)(s32)(s8)v1,
									     &out_byte);

				min_val = min_u(min_val, ret);
				min_err = (u8)min_u(min_err, out_byte);
			}
		}
		p += 4;
	}

	dist = min_u((u32)(y - low), (u32)(high - y));
	w17 = pmu_sdata()->num_ranks;
	w32 = pmu_sdata()->fw_version_id;

	x1 = (u32)(x + 1U);
	x2 = (u32)(64U - x);
	y1 = (u32)(128U - y);

	min_val = min_u(min_val, x1 * x1 * w17);
	min_val = min_u(min_val, x2 * x2 * w17);
	min_val = min_u(min_val, y1 * y1 * w32);

	if ((*(__volatile__ const u8 *)PMU_DMEM_SIGNED_STEP_0E & 1U) == 0U) {
		u32 y2 = (u32)(y + 1U);

		min_val = min_u(min_val, y2 * y2 * w32);
	}

	score = min_u(min_val, 0xffffffUL);
	min_error_val = min_u(min_u(min_err, x), 63U - x);

	return (score << 8) | ((dist + min_error_val) & 0xffU);
}

#pragma GCC pop_options

/**
 * pmu_eye_boundary_sample_search() - Search optimal X sample at fixed Y Vref slice
 * @samples: 256-byte eye sample buffer (updated at [130..131] on success)
 * @target_y: Target Y Vref coordinate
 *
 * Return: 0 on completion, or non-zero error code from center evaluation.
 */
static u32 pmu_eye_boundary_sample_search(u8 *samples, u32 target_y)
{
	u8 margins[6];
	u8 mode;
	u8 scan_lut[512];
	u32 ret = (u32)pmu_sample_margin_center_eval(samples, margins, &mode);
	u32 best_score = 0U;
	u32 best_sample;
	const u8 *p = samples;
	u32 sample_x;
	s32 step;

	if (ret != 0U)
		return ret;

	pmu_eye_sample_table_transform(scan_lut, samples);
	best_sample = pmu_eye_sample_metric_eval(margins[0], (u8)target_y,
						 scan_lut, samples);

	for (sample_x = 0U; sample_x <= 0x3fU; sample_x += 8U, p += 16U) {
		if (p[0] < target_y && target_y < p[1]) {
			u32 score;

			score = pmu_eye_sample_metric_eval((u8)sample_x,
							   (u8)target_y,
							       scan_lut,
							       samples);

			if (score > best_score) {
				best_score = score;
				best_sample = sample_x;
			}
		}
	}

	for (step = 4; step >= 2; step >>= 1) {
		s32 d;

		for (d = step; ; d = -step) {
			s32 cand = (s32)best_sample + d;

			if ((cand & ~0x3f) == 0) {
				u32 score;

				score = pmu_eye_sample_metric_eval((u8)cand,
								   (u8)target_y,
								       scan_lut,
								       samples);

				if (score > best_score) {
					best_score = score;
					best_sample = (u32)cand;
				}
			}
			if (d < 0)
				break;
		}
	}

	if (best_score != 0U) {
		samples[131] = (u8)target_y;
		samples[130] = (u8)best_sample;
	}

	return 0U;
}

#pragma GCC push_options
#pragma GCC optimize("no-expensive-optimizations,no-tree-dominator-opts")
/**
 * pmu_cal_margin_sample_pair_extract() - Optimize 2D (delay, Vref) eye center coordinates
 * @out_pair: Output 2-byte array receiving [0] optimal X and [1] optimal Y
 * @margins: Initial 2-byte seed coordinates [0] X and [1] Y
 * @samples: 256-byte eye sample table
 * @scan_lut: 512-byte transformed scan boundary table
 *
 * Return: Best 2D eye quality score achieved.
 */
static u32 pmu_cal_margin_sample_pair_extract(u8 *out_pair, const u8 *margins,
					      const u8 *samples, const u8 *scan_lut)
{
	u16 pmu_flags = *(__volatile__ const u16 *)(PMU_DMEM_BASE +
						    PMU_DMEM_DRAM_CFG_FLAGS);
	u32 max_iters = (pmu_flags >> 1) & 0x1eU;
	u32 early_exit = (max_iters == 2U) ? 1U : 0U;
	u32 m[2];
	u32 cand[2];
	u32 best_score;
	u32 step_reset;
	u32 step = 4U;
	u32 countdown;
	u32 iter;
	u32 axis;

	if (max_iters <= 2U)
		max_iters = 16U;

	out_pair[0] = 0x20U;
	out_pair[1] = 0x40U;

	m[0] = margins[0];
	m[1] = margins[1];
	cand[0] = m[0];
	cand[1] = m[1];
	best_score = pmu_eye_sample_metric_eval((u8)m[0], (u8)m[1],
						scan_lut, samples);

	if (best_score < 2U)
		return best_score;

	step_reset = max_iters >> 2;
	countdown = step_reset;

	for (iter = 0U; iter < max_iters; iter++) {
		u32 baseline_score = best_score;

		for (axis = 0U; axis < 2U; axis++) {
			u32 other = m[1U - axis];
			u32 other_eq = (other == cand[1U - axis]) ? 1U : 0U;
			u32 mask = (axis != 0U) ? 0x7fU : 0x3fU;
			u32 sub = (m[axis] - step) & mask;
			u32 add = (m[axis] + step) & mask;
			u32 s0 = (axis != 0U) ? other : sub;
			u32 s1 = (axis != 0U) ? sub : other;
			u32 a0 = (axis != 0U) ? other : add;
			u32 a1 = (axis != 0U) ? add : other;
			u32 s_sub = (sub == cand[axis] && other_eq != 0U) ? 0U :
				pmu_eye_sample_metric_eval((u8)s0, (u8)s1,
							   scan_lut,
							       samples);
			u32 s_add = (add == cand[axis] && other_eq != 0U) ? 0U :
				pmu_eye_sample_metric_eval((u8)a0, (u8)a1,
							   scan_lut,
							       samples);
			u32 best_dir = (s_add > s_sub) ? add : sub;
			u32 best_dir_score = (s_add > s_sub) ? s_add : s_sub;

			if (best_dir_score >= best_score) {
				cand[axis] = m[axis];
				m[axis] = best_dir;
				best_score = best_dir_score;
			}
		}

		if (best_score != baseline_score)
			countdown = step_reset;
		else if (countdown != 0U)
			countdown--;

		if ((u8)countdown == 0U ||
		    (m[0] == out_pair[0] && m[1] == out_pair[1])) {
			step >>= 1;
			if (step != 0U) {
				countdown = step_reset;
			} else {
				step = 1U;
				if (early_exit != 0U)
					return best_score;
			}
		}

		out_pair[1] = (u8)m[1];
		out_pair[0] = (u8)m[0];
	}

	return best_score;
}

#pragma GCC pop_options

/**
 * call_multi_rank_step() - Execute multi-rank timing step for the given rank and channel
 * @ch_mask: Active channel bitmask
 * @channel: Target channel index (0 or 1)
 * @rank: Target rank index (0 or 1)
 */
static __always_inline void call_multi_rank_step(u32 ch_mask, u32 channel,
						 u32 rank)
{
	u32 off = (PMU_DMEM_BASE | PMU_DMEM_CAL_STRUCT_BASE) +
		  (u32)(u8)rank * (2U * PMU_SLICE_DESC_SIZE) +
		  channel * PMU_SLICE_DESC_SIZE;

	pmu_cal_multi_rank_timing_step(ch_mask, off, 0U,
				       0xfffbfbffUL, 0xffffbfffUL, 0U);
}

#pragma GCC push_options
#pragma GCC optimize("no-expensive-optimizations,no-tree-dominator-opts")
/**
 * pmu_cal_timing_margin_envelope_scan() - Scan DBYTE timing margin envelope across slices
 * @channel: Target memory channel index (0 or 1)
 * @rank: Target rank index (0 or 1)
 * @buf: Output halfword buffer receiving calibrated slice delays
 */
void pmu_cal_timing_margin_envelope_scan(u32 channel, u32 rank, void *buf)
{
	u32 csr_offset = dmem_read32(PMU_DMEM_ACTIVE_CH_OFFSET);
	u32 vref_off = csr_offset << 1;
	u32 iter_count = 0U;
	u32 struct_base = PMU_DMEM_BASE | PMU_DMEM_CAL_STRUCT_BASE;
	u32 ch_mask_a0 = 0x15U << channel;
	u32 ch_r13 = 5U << channel;
	u32 hist_w = 0xffffffffUL;
	u8 *history = (u8 *)&hist_w;
	u32 fp_flag;
	u32 start_slice;
	u32 end_slice;
	u32 slice;
	u16 *out_buf;
	u32 target_reg;

	*(__volatile__ u16 *)PHY_REG_AC_BCAST_GATE_108 = 0U;
	*(__volatile__ u16 *)PHY_REG_DBYTE_BCAST_VREF_014 = 0U;

	*(__volatile__ u16 *)(PHY_REG_VREF_TRIM_CFG | vref_off) = 1U;
	*(__volatile__ u16 *)(PHY_REG_VREF_CTRL | vref_off) = 1U;

	pmu_write_phy_c2_quad(0U);
	pmu_cal_deskew_quad_tap_set(0xffU, 0U, 0U, 0U, 0U);

	*(__volatile__ u16 *)PHY_REG_SEC_CAL_CTRL_0B2 = 3U;
	*(__volatile__ u16 *)PHY_REG_DBYTE_SAMPLE_3FF6A = 0U;
	*(__volatile__ u16 *)PHY_REG_DBYTE_BCAST_SAMPLE_16A = 1U;
	*(__volatile__ u16 *)PHY_REG_DBYTE_BCAST_DQ_DLY2 = 0U;
	*(__volatile__ u16 *)PHY_REG_DBYTE_BCAST_DQ_DLY3 = 0U;
	*(__volatile__ u16 *)PHY_REG_DBYTE_BCAST_DQ_DLY0 = 0x30U;
	*(__volatile__ u16 *)PHY_REG_AC_BCAST_PAT_0C0 = 1U;
	*(__volatile__ u16 *)PHY_REG_AC_BCAST_PAT_0CA = 511U;
	*(__volatile__ u16 *)PHY_REG_BIST_PAT_CTRL_E4C = 0U;
	*(__volatile__ u16 *)PHY_REG_BIST_PAT_CTRL_E4E = 0U;
	*(__volatile__ u16 *)PHY_REG_DBYTE_BCAST_DQ_DLY1 = 386U;
	*(__volatile__ u16 *)PHY_REG_DBYTE_UPPER_COARSE_F54 = 2048U;
	*(__volatile__ u16 *)PHY_REG_DBYTE_UPPER_BCAST_F7C = 0U;
	*(__volatile__ u16 *)PHY_REG_DBYTE_BCAST_LANE_17C = 1U;

	*(__volatile__ u16 *)PHY_REG_DBYTE_BCAST_RX_EN =
		(dmem_read8(PMU_DMEM_CBT_LANE_LIM_403) == 0U) ? 520U : 512U;
	*(__volatile__ u16 *)PHY_REG_DBYTE_BCAST_CBT_144 = 2U;

	pmu_cal_dbyte_gate_mask_clear();
	pmu_phy_reg_176_strobe_pulse(0x0fU, 0x0fU);
	pmu_pulse_phy_f2();

	pmu_cal_eye_table_save((const u8 *)(uintptr_t)struct_base, 0x0aU);
	pmu_cal_lane_mask_and_all(struct_base, 0x0aU, 0x00U);
	pmu_cal_lane_mask_or_all(struct_base, 0x0aU, 0x59U);
	pmu_cal_eye_table_save((const u8 *)(uintptr_t)struct_base, 0x2eU);
	pmu_cal_lane_mask_and_all(struct_base, 0x2eU, 0x00U);
	pmu_cal_eye_table_save((const u8 *)(uintptr_t)struct_base, 0x12U);
	pmu_cal_lane_mask_or_all(struct_base, 0x12U, 0x10U);

	call_multi_rank_step(ch_mask_a0, channel, rank);

	pmu_cal_eye_table_restore((u8 *)(uintptr_t)struct_base, 0x0aU);
	pmu_cal_eye_table_restore((u8 *)(uintptr_t)struct_base, 0x12U);
	pmu_cal_eye_table_restore((u8 *)(uintptr_t)struct_base, 0x2eU);

	pmu_channel_timing_deskew_reset(0U, 0U, 0U, 0U, ch_r13);
	*(__volatile__ u16 *)PHY_REG_DBYTE_BCAST_DQS_DLY0 = 1U;
	*(__volatile__ u16 *)PHY_REG_DBYTE_BCAST_DQS_DLY1 = 1U;
	pmu_cbt_deskew_pulse_train(ch_r13, 0U);

	pmu_cal_slice_step_diff_latch(0x60U, 1U);
	pmu_cal_dbyte_delay_coarse_step();
	pmu_cal_slice_step_diff_latch(0U, 1U);

	pmu_phy_reg_176_strobe_pulse(0x0fU, 0x0fU);
	pmu_cbt_deskew_pulse_train(ch_r13, 1U);

	*(__volatile__ u16 *)PHY_REG_DBYTE_BCAST_DQ_DLY2 = 511U;
	*(__volatile__ u16 *)PHY_REG_DBYTE_BCAST_DQ_DLY3 = 511U;
	*(__volatile__ u16 *)PHY_REG_DBYTE_BCAST_DQ_DLY1 = 0x81U;
	*(__volatile__ u16 *)PHY_REG_DBYTE_BCAST_DQ_DLY0 = 0x30U;

	start_slice = dmem_read8(PMU_DMEM_SLICE_START);
	end_slice = dmem_read8(PMU_DMEM_SLICE_END);

	do {
		pmu_cal_slice_step_diff_latch(0U, 1U);

		fp_flag = 0U;
		slice = start_slice;
		do {
			u32 slice_off = slice << 13;
			u16 stat = *(__volatile__ const u16 *)
				(PHY_REG_DBYTE_LCDL_STAT_16E + slice_off);
			u8 hist = (u8)((history[slice] << 1) |
				       ((stat != 0U) ? 1U : 0U));

			history[slice] = hist;
			if ((hist & 7U) != 0U) {
				__volatile__ u16 *dly_reg = (__volatile__ u16 *)
					(PHY_REG_DBYTE_COARSE_154 + slice_off);

				*dly_reg = (u16)(*dly_reg + 1024U);
				fp_flag = 1U;
			}
		} while (++slice <= end_slice);

		pmu_phy_reg_176_strobe_pulse(0x0fU, 0x0fU);

		if (++iter_count >= 40U)
			break;
	} while (fp_flag != 0U);

	pmu_assert_or_halt(fp_flag == 0U, PMU_ERR_EYE_SWEEP_CONVERGENCE);

	slice = start_slice;
	do {
		__volatile__ u16 *dly_reg = (__volatile__ u16 *)
			(PHY_REG_DBYTE_COARSE_154 + (slice << 13));

		*dly_reg = (u16)(*dly_reg + 2048U);
	} while (++slice <= end_slice);

	*(__volatile__ u16 *)PHY_REG_DBYTE_BCAST_DQ_DLY2 = 0U;
	*(__volatile__ u16 *)PHY_REG_DBYTE_BCAST_DQ_DLY3 = 0U;
	pmu_cal_dbyte_delay_coarse_step();
	*(__volatile__ u16 *)PHY_REG_DBYTE_BCAST_DQ_DLY0 = 0U;
	*(__volatile__ u16 *)PHY_REG_DBYTE_BCAST_SAMPLE_16A = 0U;

	slice = start_slice;
	do {
		*(__volatile__ u16 *)(PHY_REG_DBYTE_SAMPLE_16A + (slice << 13)) = 1U;
	} while (++slice <= end_slice);

	*(__volatile__ u16 *)PHY_REG_DBYTE_BCAST_DQ_DLY1 = 0x81U;
	pmu_cal_slice_step_diff_latch(0x40U, 1U);

	pmu_phy_lcdl_delay_read((u16 *)buf, 1U, 2U);

	out_buf = (u16 *)buf;
	slice = start_slice;
	do {
		out_buf[slice] -= 128U;
	} while (++slice <= end_slice);

	target_reg = csr_offset | (pmu_bit_lookup_e4(channel, channel) + 0x20U);
	pmu_slice_delay_step_program(target_reg, (const s16 *)buf,
				     1U, 0U, 0U, 1U);

	*(__volatile__ u16 *)PHY_REG_DBYTE_BCAST_RX_EN = 0U;
	*(__volatile__ u16 *)PHY_REG_DBYTE_BCAST_CBT_144 = 0x20U;
	*(__volatile__ u16 *)PHY_REG_DBYTE_BCAST_CBT_144 = 0U;
	*(__volatile__ u16 *)PHY_REG_SEC_CAL_CTRL_0B2 = 0U;
	*(__volatile__ u16 *)PHY_REG_DBYTE_BCAST_DQ_DLY1 = 0U;
	*(__volatile__ u16 *)PHY_REG_AC_BCAST_GATE_108 = 1U;
	*(__volatile__ u16 *)PHY_REG_DBYTE_BCAST_VREF_014 = 1U;

	pmu_pulse_phy_140();
	*(__volatile__ u16 *)(PHY_REG_VREF_CTRL | vref_off) = 0U;

	call_multi_rank_step(ch_mask_a0, channel, rank);
}

#pragma GCC pop_options

/**
 * pmu_eye_margin_step_balancer() - Balance 2D eye margin steps across ranks/points (0x2b14)
 * @buf0: Rank 0 / primary eye sample buffer
 * @buf1: Rank 1 / secondary eye sample buffer
 * @buf2: Alt point Rank 0 eye sample buffer
 * @buf3: Alt point Rank 1 eye sample buffer
 *
 * Return: 0 on success, or non-zero calibration error code.
 */
static noinline u32 pmu_eye_margin_step_balancer(u8 *buf0, u8 *buf1, u8 *buf2,
						 u8 *buf3)
{
	u8 *bufs[4] = { buf0, buf1, buf2, buf3 };
	u32 count1 = (buf1 != buf0) ? 2U : 1U;
	u32 count2 = (buf2 != buf0) ? 2U : 1U;
	u8 *primary_buf;
	u8 *sec_buf;
	u8 temp_buf[132];
	u32 rank_idx;
	u32 pair_idx;

	for (rank_idx = 0U; rank_idx < count1; rank_idx++) {
		u32 i;

		pmu_sdata()->stage_rank_idx = (u8)rank_idx;
		for (i = 0U; i < count2; i++) {
			u8 *b = bufs[i * 2U + rank_idx];
			u32 ret = pmu_window_centroid_calc(b);

			if (ret != 0U)
				return ret;
		}
	}

	primary_buf = buf0;
	sec_buf = buf1;

	for (pair_idx = 1U; pair_idx <= count2; pair_idx++) {
		s8 off = (s8)sec_buf[130] - (s8)primary_buf[130];
		u8 d0b;
		u32 ret;

		pmu_sdata()->stage_rank_idx = 3U;
		pmu_window_margin_diff_calc(primary_buf, sec_buf, temp_buf,
					    (s32)off, 0);

		d0b = dmem_read8(PMU_DMEM_CAL_SELECT_FLAGS);
		if ((d0b & 4U) != 0U)
			ret = pmu_window_centroid_calc(temp_buf);
		else
			ret = pmu_eye_margin_window_search(temp_buf, 0U);
		if (ret != 0U)
			return ret;

		primary_buf[131] = temp_buf[131];
		sec_buf[131] = temp_buf[131];

		if (pair_idx < count2) {
			primary_buf = buf2;
			sec_buf = buf3;
		}
	}

	if (buf2 != buf0) {
		u32 buf_idx;

		for (buf_idx = 0U; buf_idx < count1; buf_idx++) {
			u8 *rx = bufs[buf_idx];
			u8 *ry = bufs[buf_idx + 2U];
			s32 delta = (s32)ry[131] - (s32)rx[131];
			s32 diff = (s32)(s16)*(const __volatile__ u16 *)(rx + 128) -
				   (s32)(s16)*(const __volatile__ u16 *)(ry + 128);
			u32 ret;

			pmu_window_margin_diff_calc(rx, ry, temp_buf, diff,
						    delta);
			ret = pmu_eye_boundary_sample_search(temp_buf,
							     (u32)rx[131]);
			if (ret != 0U)
				return ret;
			rx[130] = temp_buf[130];
			ry[130] = (u8)((s32)temp_buf[130] + diff);
		}
	} else {
		u32 ret0 = pmu_eye_boundary_sample_search(buf2,
							      (u32)buf2[131]);
		u32 ret1;

		if (ret0 != 0U)
			return ret0;
		ret1 = pmu_eye_boundary_sample_search(buf1, (u32)buf1[131]);
		if (ret1 != 0U)
			return ret1;
	}

	return 0U;
}

static __always_inline void
pmu_cal_margin_matrix_prog_step(u32 cur_dly, u8 pmu_cfg13,
				u32 target_timing_reg, u32 source_timing_reg,
				u8 accumulated_bias, u8 baseline_buf[4][10])
{
	u8 s_start = dmem_read8(PMU_DMEM_SLICE_START);
	u8 s_end = dmem_read8(PMU_DMEM_SLICE_END);
	u8 a_mask = dmem_read8(PMU_DMEM_ACTIVE_SLICE_MASK_CAL);
	u32 csr_off = dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET);
	u32 s_idx;

	for (s_idx = s_start; s_idx <= s_end; s_idx++) {
		if (!(a_mask & (1U << s_idx)))
			continue;

		if (pmu_cfg13 != 0U) {
			u32 lane;

			for (lane = 0U; lane < 10U; lane++) {
				u16 dly_val = (u16)(cur_dly + baseline_buf[s_idx][lane]);
				u32 lane_csr = (s_idx << 12) | csr_off | (lane << 8);
				u32 reg_34 = PHY_REG_DBYTE_BASE |
					     ((lane_csr | target_timing_reg) << 1);

				phy_write16(reg_34, dly_val);
				if (accumulated_bias != 0U) {
					u32 reg_40 = PHY_REG_DBYTE_BASE |
						     ((lane_csr | source_timing_reg) << 1);

					phy_write16(reg_40, dly_val);
				}
			}
		} else {
			u32 reg_34 = (PHY_REG_DBYTE_BASE | 0x1e00UL) |
				(((s_idx << 12) | csr_off | target_timing_reg) << 1);

			phy_write16(reg_34, (u16)cur_dly);
			if (accumulated_bias != 0U) {
				u32 reg_40 = (PHY_REG_DBYTE_BASE | 0x1e00UL) |
					(((s_idx << 12) | csr_off | source_timing_reg) << 1);

				phy_write16(reg_40, (u16)cur_dly);
			}
		}
	}
	pmu_delay_us(300000U, 0U);
}

static __always_inline void
pmu_cal_margin_matrix_eval_lane(u32 k, u32 ch, u32 s_idx, u32 lane,
				u16 sp_words_66, u16 sp_words_74, u32 lane_div,
				u8 scan_multiplier, u32 flag, u32 last_cur_dly,
				u8 step_size, u8 pmu_cfg13, u8 base_adj)
{
	u32 entry_idx = (s_idx * 10U) + lane;
	__volatile__ u8 *state_bytes =
		(__volatile__ u8 *)((PMU_DMEM_BASE | PMU_DMEM_SWEEP_SAMPLE_BUF) +
				    0x27cUL + (entry_idx * 848U));
	s32 win_start = 0;
	s32 in_pass = 0;
	s32 best_start = 0;
	s32 best_end = 0;
	s32 best_width = 0;
	s32 last_step = -1;
	s32 st_step;
	uintptr_t entry_base;
	__volatile__ u8 *rec_base;
	u32 shift_val;
	u32 idx;

	for (st_step = 0; st_step < (s32)sp_words_66; st_step++) {
		u8 st = state_bytes[st_step];

		if (in_pass != 0) {
			if (st == 0U) {
				s32 cand_width = last_step - win_start;

				if (cand_width > (best_width << 1)) {
					best_start = win_start;
					best_end = last_step;
					best_width = cand_width;
				}
				in_pass = 0;
			}
		} else if (st != 0U) {
			in_pass = 1;
			win_start = st_step;
		}
		last_step = st_step;
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

	entry_base = (PMU_DMEM_BASE | PMU_DMEM_EYE_SAMPLE_BUF) +
		     (k * 10560U) + (ch * 5280U) + (s_idx * 1320U) + (lane * 132U);
	*(__volatile__ u16 *)(entry_base + 128U) = (u16)best_start;

	rec_base = (__volatile__ u8 *)((PMU_DMEM_BASE | PMU_DMEM_SWEEP_SAMPLE_BUF) +
				       (entry_idx * 848U));
	shift_val = scan_multiplier - 1U;

	for (idx = 0U; idx < 64U; idx++) {
		u32 step_sampled = (u8)(best_start + (idx >> shift_val));
		u8 st = state_bytes[step_sampled];
		u8 left_val;
		u8 right_val;

		if (flag != 0U) {
			if (st == 0U) {
				left_val = 127U;
				right_val = 0U;
			} else if (st == 1U) {
				left_val = rec_base[step_sampled * 3U + 0U];
				right_val = (u8)last_cur_dly;
			} else if (st == 2U) {
				left_val = rec_base[step_sampled * 3U + 0U];
				right_val = rec_base[step_sampled * 3U + 1U] - step_size;
			} else {
				u8 cand_left = rec_base[step_sampled * 3U + 0U];
				u8 cand_right = rec_base[step_sampled * 3U + 1U];
				u8 cand_start = rec_base[step_sampled * 3U + 2U];
				u32 cand_w = (u32)(cand_right - (cand_left + step_size));
				u32 cur_w = (u32)((u8)last_cur_dly - cand_start);
				bool pick_cur = cur_w > cand_w;

				left_val = pick_cur ? cand_start : cand_left;
				right_val = pick_cur ? (u8)last_cur_dly :
						       (u8)(cand_right - step_size);
			}
		} else {
			left_val = ((st | 2U) == 3U) ? 0U : 127U;
			right_val = ((st | 2U) == 3U) ? 127U : 0U;
		}

		if ((s8)right_val - (s8)left_val < 5) {
			left_val = 127U;
			right_val = 0U;
		}

		if (pmu_cfg13 != 0U && (u8)right_val >= left_val) {
			left_val += base_adj;
			right_val += base_adj;
		}

		*(__volatile__ u8 *)(entry_base + (idx * 2U) + 0U) = left_val;
		*(__volatile__ u8 *)(entry_base + (idx * 2U) + 1U) = right_val;
	}
}

static __always_inline void
pmu_cal_margin_matrix_center_lane(u32 k, u32 ch, u32 s_idx, u32 lane,
				  u16 metric_max, u8 scan_multiplier)
{
	uintptr_t entry_base = (PMU_DMEM_BASE | PMU_DMEM_EYE_SAMPLE_BUF) +
		(k * 10560U) + (ch * 5280U) + (s_idx * 1320U) + (lane * 132U);
	s32 win_start = 0;
	s32 win_end = 0;
	s32 best_w_start = 0;
	s32 best_w_end = 0;
	s32 best_w = -1;
	s32 in_win = 0;
	u32 st_step;
	s32 sum_steps;
	s32 center;
	u16 best_start;
	u16 new_baseline;

	for (st_step = 0U; st_step < 64U; st_step++) {
		u8 left = *(__volatile__ u8 *)(entry_base + (st_step * 2U) + 0U);
		u8 right = *(__volatile__ u8 *)(entry_base + (st_step * 2U) + 1U);

		if (right >= left) {
			if (in_win == 0) {
				in_win = 1;
				win_start = (s32)st_step;
			}
			win_end = (s32)st_step;
		} else if (in_win != 0) {
			s32 cur_w = win_end - win_start;

			in_win = 0;
			if (cur_w > best_w) {
				best_w = cur_w;
				best_w_start = win_start;
				best_w_end = win_end;
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

	sum_steps = best_w_start + best_w_end;
	if (dmem_read8(PMU_DMEM_TRAIN_MODE_HI) & 4U) {
		center = sum_steps;
	} else {
		center = sum_steps / 2;
		if (dmem_read8(PMU_DMEM_CAL_OVERRIDE_FLAG) != 0U) {
			u32 width = (u32)(best_w_end - best_w_start);

			pmu_assert_or_halt((width > 5U) ? 1U : 0U,
					   PMU_ERR_EYE_WINDOW_TOO_NARROW);
		}
	}

	best_start = *(__volatile__ u16 *)(entry_base + 128U);
	new_baseline = metric_max + (best_start * scan_multiplier);
	*(__volatile__ u16 *)(entry_base + 128U) = new_baseline;
	*(__volatile__ s8 *)(entry_base + 130U) = (s8)center;
}

/**
 * pmu_margin_matrix_capture_baseline() - Capture per-slice/lane baseline delay registers
 * @baseline_buf: Output 4x10 byte baseline array
 * @pmu_cfg13: Configuration flag 13 value
 * @target_timing_reg: Target DBYTE timing register offset
 * @scan_direction: Initial scan direction / start tap
 * @vref_boundary_clamp: Maximum Vref/delay boundary clamp
 */
static void pmu_margin_matrix_capture_baseline(u8 baseline_buf[4][10],
					       u32 pmu_cfg13,
					       u32 target_timing_reg,
					       u8 scan_direction,
					       u8 vref_boundary_clamp)
{
	u8 s_start = dmem_read8(PMU_DMEM_SLICE_START);
	u8 s_end = dmem_read8(PMU_DMEM_SLICE_END);
	u8 a_mask = dmem_read8(PMU_DMEM_ACTIVE_SLICE_MASK_CAL);
	u32 active_csr_offset = dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET);
	u8 limit_val = dmem_read8((pmu_cfg13 <= 2U) ? 0x11bU : 0x11aU);
	u8 blink_val = (u8)(scan_direction + (127U - vref_boundary_clamp));
	u32 s_idx;

	for (s_idx = s_start; s_idx <= s_end; s_idx++) {
		u32 lane;

		if (!(a_mask & (1U << s_idx)))
			continue;
		for (lane = 0U; lane < 10U; lane++) {
			u32 lane_csr = (s_idx << 12) | active_csr_offset |
				       target_timing_reg | (lane << 8);
			u8 val_b = (u8)phy_read16(PHY_REG_DBYTE_BASE |
						  (lane_csr << 1));
			bool clamp_hi = (u32)vref_boundary_clamp + val_b >= 128U;
			u8 adj = ((s8)val_b < (s8)limit_val) ?
				 0U : (u8)(val_b - limit_val);

			baseline_buf[s_idx][lane] = clamp_hi ? blink_val : adj;
		}
	}
}

/**
 * pmu_margin_matrix_select_pass_regs() - Resolve source/target timing registers for a scan pass
 * @k: Pass index (0 or 1)
 * @mode: Calibration sweep mode (0..3)
 * @profile_mode: PHY operating profile mode (0..3)
 * @sp_words_79: Non-zero to retain precomputed primary/secondary registers
 * @timing_reg_pri: Precomputed primary timing register
 * @timing_reg_sec: Precomputed secondary timing register
 * @slice_reg_stride: Precomputed slice register stride
 * @target_out: Output pointer for target timing register
 * @source_out: Output pointer for source timing register
 * @stride_out: Output pointer for active slice stride
 */
static void pmu_margin_matrix_select_pass_regs(u32 k, u32 mode, u32 profile_mode,
					       u32 sp_words_79,
					       u32 timing_reg_pri,
					       u32 timing_reg_sec,
					       u32 slice_reg_stride,
					       u32 *target_out,
					       u32 *source_out,
					       u32 *stride_out)
{
	u32 target = timing_reg_sec;
	u32 source = timing_reg_pri;
	u32 stride = slice_reg_stride;

	if (sp_words_79 & 1U) {
		/* Retain precomputed target, source, and stride */
	} else if (mode == 0U && profile_mode == 3U) {
		target = (k == 0U) ? 0x4eU : 0x50U;
		source = (k == 0U) ? 0x4fU : 0x51U;
		stride = slice_reg_stride << (k != 0U);
	} else if (mode == 1U && profile_mode == 3U) {
		target = (k == 0U) ? 0x4fU : 0x51U;
		source = (k == 0U) ? 0x4eU : 0x50U;
		stride = ((k != 0U) ? 2U : 0U) + 1U;
	} else if (mode == 2U) {
		target = 0x4eU;
		source = 0x4fU;
		stride = 4U;
	} else {
		pmu_assert_or_halt(0U, PMU_ERR_EYE_SLICE_REG_SOURCE);
		source = 0x4fU;
		target = 0x4eU;
	}

	*target_out = target;
	*source_out = source;
	*stride_out = stride;
}

/**
 * pmu_cal_margin_matrix_scan_eval() - Execute 2D Vref/delay margin matrix scan
 * @channel: Target channel index (0 or 1)
 * @rank: Target rank index
 * @slice: Slice/boundary mode flags
 * @mode: Calibration sweep mode (0..3)
 * @flag: Vref/delay step flag
 * @step: Sweep pass index
 */
void pmu_cal_margin_matrix_scan_eval(u32 channel, u32 rank, u32 slice,
				     u32 mode, u32 flag, u32 step)
{
	u16 scan_metric_min[4] = { 0 };
	u16 scan_metric_max[4] = { 0 };
	u8 scan_profile_table[48] = { 0 };
	u8 baseline_buf[4][10];
	u32 bound_flag = (slice >> 8) & 1U;
	u32 status_query = pmu_cal_pad_voltage_dac_set(bound_flag, mode);
	u32 profile_mode = pmu_phy_query_operating_mode();
	u8 dmem_0d = dmem_read8(PMU_DMEM_CAL_MISC_FLAGS);
	u32 channel_scan_mask = 1U;
	u32 active_scan_flag = (dmem_0d & 0x10U) ? 0U : 1U;
	u32 is_extended_sweep = 0U;
	u8 dmem_40, dmem_25;
	u32 scan_multiplier, flag_216, flag_50 = 0U;
	u32 is_mode_1, is_mode_0, is_pm_0, is_pm_1, is_pm_2;
	u32 mode_zero_cond, channel_stride_adj, val_r12, is_pm0_mode1;
	u32 is_cond_or_m0, is_mode0_pm1, val_r3, is_eval_active;
	u32 timing_reg_alt_b, sp_words_68, val_r13, sp_words_77, sp_words_79;
	u32 val_r3_188, timing_reg_alt_a, dmem_mask, ch;

	if (rank != 0U) {
		is_extended_sweep = (dmem_0d >> 3) & 1U;
		active_scan_flag = 0U;
	}

	if (profile_mode == 3U)
		channel_scan_mask = (mode != 2U) ? 2U : 1U;

	dmem_40 = dmem_read8(PMU_DMEM_CH1_RANK_EN);
	dmem_25 = dmem_read8(PMU_DMEM_CH0_RANK_EN);
	scan_multiplier = (is_extended_sweep != 0U) ? 2U : 1U;

	flag_216 = (rank == 1U) || ((u32)(rank - 3U) < 5U);
	if ((bound_flag != 0U && flag_216) ||
	    (dmem_read8(PMU_DMEM_TRAIN_STATUS) & 1U)) {
		pmu_phy_dbyte_cfg_unpack();
		flag_50 = 1U;
	}

	is_mode_1 = (mode == 1U);
	is_mode_0 = (mode == 0U);
	is_pm_0 = (profile_mode == 0U);
	is_pm_1 = (profile_mode == 1U);
	is_pm_2 = (profile_mode == 2U);

	mode_zero_cond = (flag == 0U) || (is_pm_0 && is_mode_0);
	channel_stride_adj = (flag == 0U) ? 4U : 0U;
	val_r12 = mode_zero_cond ? 0x4eU : 0x4fU;
	is_pm0_mode1 = (is_pm_0 && is_mode_1);
	is_cond_or_m0 = is_pm0_mode1 || mode_zero_cond;
	is_mode0_pm1 = (is_mode_0 && is_pm_1);
	val_r3 = is_mode0_pm1 ? val_r12 : 0x50U;
	is_eval_active = is_mode0_pm1 || is_cond_or_m0;
	timing_reg_alt_b = is_eval_active ? val_r3 : 0x51U;

	sp_words_68 = (is_mode_1 && is_pm_1) || is_eval_active;
	val_r13 = sp_words_68 ? 0x4fU : 0x4eU;
	sp_words_77 = sp_words_68 || (is_mode_0 && is_pm_2);
	sp_words_79 = sp_words_77 || (is_pm_2 && is_mode_1);

	val_r3_188 = is_cond_or_m0 ? val_r13 : 0x51U;
	timing_reg_alt_a = is_eval_active ? val_r3_188 : 0x50U;

	if (!is_eval_active)
		channel_stride_adj = 3U;
	else if (!is_cond_or_m0)
		channel_stride_adj = 2U;
	else if (!mode_zero_cond)
		channel_stride_adj = 1U;

	dmem_mask = dmem_40 | dmem_25;

	for (ch = 0U; ch < 2U; ch++) {
		u32 ch_mask = (1U << (ch + 2U)) | (1U << ch);
		u8 vref_boundary_clamp = 127U;
		u32 margin_diff_accum = 0U;
		u32 sp_words_66, window_eval_mode, sp_words_74;
		u32 timing_reg_pri, timing_reg_sec, slice_reg_stride, k;

		if (ch != 0U && (ch_mask & dmem_mask) == 0U)
			continue;

		dmem_write8(PMU_DMEM_CAL_BYTE, (u8)ch);
		pmu_phy_mode_cfg_dispatch(3U);
		pmu_cal_dbyte_pattern_loop_exec(ch, (u16)mode, scan_metric_min,
						scan_metric_max, step,
						(u8)active_scan_flag,
						is_extended_sweep);

		if (mode < 2U) {
			sp_words_66 = 128U;
			window_eval_mode =
				(dmem_read16(PMU_DMEM_DRAM_FREQ_OFF) < 3200U) ?
				1U : 0U;
		} else if (mode == 2U) {
			sp_words_66 = 212U;
			window_eval_mode = 1U;
			margin_diff_accum =
				(dmem_read8(PMU_DMEM_TRAIN_PARAM_CTRL) >> 4) & 1U;
		} else {
			sp_words_66 = 212U;
			window_eval_mode = 0U;
		}

		pmu_cal_bist_cmd_strobe_dispatch((u32)(uintptr_t)scan_profile_table,
						 rank, mode, ch_mask,
						 (u32)(uintptr_t)scan_metric_min,
						 bound_flag, flag_50);

		*(u32 *)(scan_profile_table + 0) = 0U;
		*(u32 *)(scan_profile_table + 4) = 1U;
		*(u32 *)(scan_profile_table + 8) = scan_multiplier;

		sp_words_74 = sp_words_66 - 1U;

		timing_reg_pri = (ch == 0U) ? 0x4eU : 0x50U;
		timing_reg_sec = (ch == 0U) ? 0x4fU : 0x51U;
		slice_reg_stride = (ch_mask << (ch != 0U)) + 1U;

		if (sp_words_68 & 1U) {
			timing_reg_sec = timing_reg_alt_b;
			timing_reg_pri = timing_reg_alt_a;
			slice_reg_stride = channel_stride_adj;
		} else if (sp_words_77 & 1U) {
			timing_reg_sec = timing_reg_pri;
			timing_reg_pri = (ch == 0U) ? 0x4fU : 0x51U;
			slice_reg_stride = rank << (ch != 0U);
		}

		for (k = 0U; k < channel_scan_mask; k++) {
			u32 target_timing_reg, source_timing_reg, slice_active_reg;
			u8 dly_step_flag, scan_direction = 0U;
			u32 accumulated_bias = 0U, pmu_cfg13, step_size, last_cur_dly, cur_dly;
			u8 slice_start, slice_end, active_mask;
			u32 lane_div, s_idx;

			pmu_margin_matrix_select_pass_regs(k, mode, profile_mode,
							   sp_words_79,
							   timing_reg_pri,
							   timing_reg_sec,
							   slice_reg_stride,
							   &target_timing_reg,
							   &source_timing_reg,
							   &slice_active_reg);

			if (flag != 0U) {
				dly_step_flag = (u8)flag;
				pmu_dbyte_dq_deskew_regs_save_and_ramp(&scan_direction,
								       &vref_boundary_clamp,
								       &dly_step_flag,
								       slice_active_reg,
								       flag_216);
			} else {
				vref_boundary_clamp = 0U;
				dly_step_flag = 1U;
			}

			if (flag_216)
				accumulated_bias =
					(pmu_check_dmem_407_flag() != 0U) ? 1U : 0U;

			pmu_cfg13 = pmu_dram_cfg_flag13_check(slice_active_reg,
							      flag_216);

			if (pmu_cfg13 != 0U && ch == 0U)
				pmu_margin_matrix_capture_baseline(baseline_buf,
								   pmu_cfg13,
								   target_timing_reg,
								   scan_direction,
								   vref_boundary_clamp);

			memset((void *)(PMU_DMEM_BASE | PMU_DMEM_SWEEP_SAMPLE_BUF), 0, 0x8480U);
			step_size = dly_step_flag;
			last_cur_dly = scan_direction;

			for (cur_dly = scan_direction;
			     cur_dly <= vref_boundary_clamp;
			     cur_dly += step_size) {
				void *sweep_buf = (void *)(PMU_DMEM_BASE |
							   PMU_DMEM_SWEEP_SAMPLE_BUF);
				u32 active_csr_offset;

				last_cur_dly = cur_dly;

				if (flag != 0U) {
					if (mode == 2U)
						pmu_cal_pulse_seq_coordinator(0U, ch, (u8)cur_dly,
									      3U);
					else
						pmu_cal_margin_matrix_prog_step(cur_dly, pmu_cfg13,
										target_timing_reg,
										source_timing_reg,
										accumulated_bias,
										baseline_buf);
				}

				active_csr_offset = dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET);
				phy_write16(PHY_REG_DBYTE_BCAST_PULSE | (active_csr_offset << 1),
					    0U);

				*(u32 *)scan_profile_table = cur_dly;

				pmu_cal_slice_step_eval_sweep(sweep_buf,
							      rank, mode, pmu_cfg13, 0U,
							      (u8)channel,
							      window_eval_mode, scan_metric_min,
							      bound_flag,
							      flag_50, scan_profile_table);

				if (margin_diff_accum != 0U) {
					pmu_dbyte_rx_en_slice_pulse_eval(1U, ch_mask);
					pmu_cal_bist_pattern_setup(rank, ch_mask, 0U, flag_50);
					pmu_cal_slice_step_eval_sweep(sweep_buf,
								      rank, 3U, pmu_cfg13, 0U,
								      (u8)channel,
								      window_eval_mode,
								      scan_metric_min, 0U,
								      flag_50, scan_profile_table);
					pmu_cal_bist_pattern_setup(rank, ch_mask, status_query,
								   flag_50);
					pmu_dbyte_rx_en_slice_pulse_eval(0U, ch_mask);
				}
			}

			slice_start = dmem_read8(PMU_DMEM_SLICE_START);
			slice_end = dmem_read8(PMU_DMEM_SLICE_END);
			active_mask = dmem_read8(PMU_DMEM_ACTIVE_SLICE_MASK_CAL);
			lane_div = 64U / scan_multiplier;

			for (s_idx = slice_start; s_idx <= slice_end; s_idx++) {
				u32 lane;

				if (!(active_mask & (1U << s_idx)))
					continue;

				for (lane = 0U; lane < 10U; lane++)
					pmu_cal_margin_matrix_eval_lane(k, ch, s_idx, lane,
									sp_words_66, sp_words_74,
									lane_div, scan_multiplier,
									flag, last_cur_dly,
									step_size, pmu_cfg13,
									baseline_buf[s_idx][lane]);
			}

			for (s_idx = slice_start; s_idx <= slice_end; s_idx++) {
				u32 lane;

				if (!(active_mask & (1U << s_idx)))
					continue;

				for (lane = 0U; lane < 10U; lane++) {
					if (bound_flag == 0U && lane == 8U)
						continue;
					if (margin_diff_accum == 0U && lane == 9U)
						continue;

					pmu_cal_margin_matrix_center_lane(k, ch, s_idx, lane,
									  scan_metric_max[s_idx],
									  scan_multiplier);
				}
			}
		}

		pmu_dmem_reg_stream_unpack(scan_profile_table);
	}

	if (flag_50 != 0U)
		pmu_cal_pad_impedance_lock();
}

/**
 * pmu_cal_lane_window_margin_eval() - Evaluate lane window margins and update timing tables
 * @mode: Evaluation mode (0 for full rank/slice clamp and dispatch)
 */
void pmu_cal_lane_window_margin_eval(u32 mode)
{
	u8 *base = (u8 *)(PMU_DMEM_BASE | PMU_DMEM_CAL_STRUCT_BASE);
	u32 ch;
	u32 rank;
	u32 csr_offset;
	u8 count;
	u8 dmem0;

	pmu_cal_timing_table_matrix_init(base);

	pmu_cal_eye_table_save((void *)(uintptr_t)((u32)(uintptr_t)base), 0x0bU);
	pmu_cal_eye_table_save((void *)(uintptr_t)((u32)(uintptr_t)base), 0x11U);
	pmu_cal_lane_mask_and_all((u32)(uintptr_t)base, 0x12U, 0xefU);
	pmu_cal_eye_table_save((void *)(uintptr_t)((u32)(uintptr_t)base), 0x12U);
	pmu_cal_eye_table_save((void *)(uintptr_t)((u32)(uintptr_t)base), 0x29U);

	for (ch = 0U; ch < 2U; ch++) {
		for (rank = 0U; rank < 2U; rank++)
			pmu_cal_training_flags_gate_update(base +
							   ch * (2U * PMU_SLICE_DESC_SIZE) +
							   rank * PMU_SLICE_DESC_SIZE);
	}

	dmem_write8(PMU_DMEM_LANE_ACTIVE_FLAGS, 0U);
	dmem_write8(PMU_DMEM_CAL_BYTE, 0U);

	for (ch = 0U; ch < 2U; ch++) {
		for (rank = 0U; rank < 2U; rank++)
			base[ch * (2U * PMU_SLICE_DESC_SIZE) +
			     rank * PMU_SLICE_DESC_SIZE + 3U] |= 0x40U;
	}
	pmu_phy_reg_offset_diff_adjust(1U);

	pmu_slice_status_b97_save();

	csr_offset = dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET);
	phy_write16(PHY_REG_VREF_CTRL | (csr_offset << 1), 0U);
	phy_write16(PHY_REG_VREF_TRIM_CFG, 1U);

	{
		const u8 *d0 = (const u8 *)PMU_DMEM_BASE;

		dmem0 = d0[0];
		if (d0[0x18] == 0U) {
			u32 dly1 = (dmem0 & 2U) ? 0x1e8480UL : 0xbebc200UL;
			u32 dly2 = (dmem0 & 2U) ? 0x1e8480UL : 0x77359400UL;

			pmu_delay_us(dly1, 0U);
			if (d0[0xd] & 4U)
				pmu_mailbox_post_cmd_dispatch(0x40U);
			phy_write16(PHY_REG_PHY_STATUS, 3U);
			pmu_delay_us(dly2, 0U);
		}
		if (d0[0xd] & 1U)
			phy_write16(PHY_REG_PHY_STATUS, 1U);
	}

	pmu_cal_strobe_pulse_init();
	count = (u8)pmu_cal_delay_line_accum_dispatch(0xbb8U, 0U);
	if (count != 0U) {
		do {
			pmu_cal_sequence_pulse_send(0U, 7U, 0U, 0U, 0U, 3U, 0U);
		} while (--count != 0U);
	}
	pmu_cal_sequence_pulse_send(0x80U, 7U, 0U, 0U, 0U, 3U, 0U);
	pmu_cal_slice_step_diff_latch(0U, 1U);

	if (dmem_read8(PMU_DMEM_AC_CAL_ACTIVE) == 0U || (dmem0 & 8U) != 0U) {
		pmu_delay_us(0x1e8480UL, 0U);
	} else {
		pmu_ac_lane_profile_setup();
		pmu_cal_strobe_pulse_init();

		pmu_cal_pad_timing_delay_set(0U, 8U, 1U);
		if (dmem_read8(PMU_DMEM_AC_SIGNED_STEP) >= 2U)
			pmu_cal_pad_timing_delay_set(0U, 0U, 1U);
		pmu_cal_pad_timing_delay_set(0U, 10U, 1U);
		pmu_cal_pad_timing_delay_set(8U, 0U, 1U);
		pmu_cal_pad_timing_delay_set(0U, 0x96U, 1U);
		pmu_cbt_3phase_pulse_seq();
		pmu_cal_slice_step_diff_latch(0U, 1U);
		pmu_phy_profile_param_program();
	}

	if (mode == 0U) {
		u8 num_entries;
		u32 entry;

		pmu_cal_gate_eye_master_trigger();
		pmu_multirank_slice_cal_dispatch(base, 1U, 0xeffefffdUL,
						 (u32)-1, 0xffU, 0U);
		pmu_cal_eye_table_restore(base, 0x0bU);
		pmu_cal_eye_table_restore(base, 0x11U);
		pmu_cal_eye_table_restore(base, 0x12U);
		pmu_cal_eye_table_restore(base, 0x29U);

		num_entries = dmem_read8(PMU_DMEM_METRIC_TABLE_ENTRIES);
		for (entry = 0U; entry < num_entries; entry++) {
			u8 offset = dmem_read8(PMU_DMEM_CAL_CMD_TABLE_46C + entry);
			u8 *entry_base = base + offset;

			for (rank = 0U; rank < 2U; rank++) {
				for (ch = 0U; ch < 2U; ch++) {
					u8 val = entry_base[ch * (2U * PMU_SLICE_DESC_SIZE) +
							    rank * PMU_SLICE_DESC_SIZE];

					dmem_write8(PMU_DMEM_DESKEW_STAT_TBL_B6E +
						    ch * 2U + entry * 4U + rank,
						    val);
				}
			}
		}
	}

	pmu_cal_delay_line_init();

	{
		u8 *dmem = (u8 *)PMU_DMEM_BASE;

		dmem[0x33] = dmem[0x6e];
		dmem[0x34] = dmem[0x6f];
		dmem[0x4e] = dmem[0x70];
		dmem[0x4f] = dmem[0x71];
		dmem[0x35] = dmem[0x76];
		dmem[0x36] = dmem[0x77];
		dmem[0x50] = dmem[0x78];
		dmem[0xd0] = dmem[0x7a];
		dmem[0x51] = dmem[0x79];
		dmem[0xd5] = dmem[0x7b];
		dmem[0xda] = dmem[0x7c];
		dmem[0xd1] = dmem[0x9a];
		dmem[0xdf] = dmem[0x7d];
		dmem[0xd6] = dmem[0x9b];
		dmem[0xdb] = dmem[0x9c];
		dmem[0xe0] = dmem[0x9d];
		dmem[0xd2] = dmem[0xae];
		dmem[0xd7] = dmem[0xaf];
		dmem[0xdc] = dmem[0xb0];
		dmem[0xe1] = dmem[0xb1];
	}
}

/**
 * pmu_eye_margin_window_search() - Search 2D eye margin window for optimal center (0x734c)
 * @samples: 256-byte eye sample buffer (updated at [130..131] on success)
 * @reserved_arg: Reserved parameter
 *
 * Return: 0 on success, 5 if no valid metric found, or error code.
 */
noinline u32 pmu_eye_margin_window_search(u8 *samples, u32 reserved_arg)
{
	u8 *s_reg = samples;
	const u8 *p_margin;
	u32 best_metric = 0U;
	u8 scan_lut[512];
	u8 out_margins[6];
	u8 out_mode;
	u32 ret;
	u32 count;

	(void)reserved_arg;
	pmu_eye_sample_table_transform(scan_lut, s_reg);
	ret = pmu_sample_margin_center_eval(s_reg, out_margins, &out_mode);
	if (ret != 0U)
		return ret;

	if ((u32)(out_mode - 2U) <= 1U) {
		p_margin = out_margins + 4;
		count = out_mode;
	} else {
		p_margin = out_margins;
		count = 1U;
	}

	do {
		u8 pair[2];
		u32 metric;

		metric = pmu_cal_margin_sample_pair_extract(pair,
							    p_margin,
								s_reg,
								scan_lut);

		if (metric != 0U && metric >= best_metric) {
			best_metric = metric;
			s_reg[131] = pair[1];
			s_reg[130] = pair[0];
		}
		p_margin -= 2;
	} while (--count);

	return (best_metric == 0U) ? 5U : 0U;
}

/**
 * pmu_rank_margin_eval_zero_tap() - Evaluate per-channel slice/pin margins when tap_ptr is zero
 * @start_slice: Starting DBYTE slice index
 * @end_slice: Ending DBYTE slice index
 * @rank_limit: Active rank limit (1 or 2)
 * @mode: PHY operating profile mode (0..3)
 * @channel: Target channel index (0 or 1)
 * @slice_mask: Active pin/lane bitmask
 *
 * Return: 0 on success, or non-zero calibration error code.
 */
static u32 pmu_rank_margin_eval_zero_tap(u32 start_slice, u32 end_slice,
					 u32 rank_limit, u32 mode, u32 channel,
					 u32 slice_mask)
{
	u32 margin_window_ret = 0U;
	u32 slice;
	u8 c0, c1;

	c0 = dmem_read8(PMU_DMEM_CH0_CTRL_20);
	dmem_write16(PMU_DMEM_METRIC_COEFF_X, (u16)c0 * c0);
	c1 = dmem_read8(PMU_DMEM_CH0_CTRL_21);
	dmem_write16(PMU_DMEM_METRIC_COEFF_Y, (u16)c1 * c1);

	for (slice = start_slice; slice <= end_slice; slice++) {
		u32 pin;

		dmem_write8(PMU_DMEM_STAGE_PARAM_407, (u8)slice);

		for (pin = 0U; pin < 10U; pin++) {
			u32 off;
			u8 *rank0, *rank1, *rank2, *rank3;

			if (!(slice_mask & (1U << pin)))
				continue;

			dmem_write8(PMU_DMEM_STAGE_PARAM_408, (u8)pin);

			off = slice * 1320U + pin * 132U;
			rank0 = (u8 *)(uintptr_t)((PMU_DMEM_BASE |
						   PMU_DMEM_EYE_SAMPLE_BUF) + off);
			rank1 = rank0 + 5280U;
			rank2 = rank0 + (2U * 5280U);
			rank3 = rank0 + (3U * 5280U);

			if (rank_limit == 1U && mode < 3U) {
				u32 ret = pmu_cal_slice_mask_eval(rank0);

				if (ret != 0U)
					return (u8)ret;
				margin_window_ret = 0U;
				if (dmem_read8(PMU_DMEM_CAL_ACTIVE_FLAG) != 0U)
					pmu_eye_margin_bidir_scan(rank0, 0U, channel, 0U);
			} else if (rank_limit == 1U && mode == 3U) {
				u32 ret = pmu_eye_margin_step_balancer(rank0, rank0,
								       rank2, rank2);

				if (ret != 0U)
					return (u8)ret;
				if (dmem_read8(PMU_DMEM_CAL_ACTIVE_FLAG) != 0U) {
					pmu_eye_margin_bidir_scan(rank0, 0U, channel,
								  margin_window_ret);
					pmu_eye_margin_bidir_scan(rank2, 0U, channel,
								  margin_window_ret);
				}
			} else if (rank_limit == 1U) {
				pmu_assert_or_halt(0U, PMU_ERR_EYE_RANK_LIMIT_1);
			} else if (mode < 2U) {
				u32 ret = pmu_eye_margin_step_balancer(rank0, rank1,
								       rank0, rank1);

				if (ret != 0U)
					return (u8)ret;
				if (dmem_read8(PMU_DMEM_CAL_ACTIVE_FLAG) != 0U) {
					pmu_eye_margin_bidir_scan(rank0, 0U, channel, 0U);
					pmu_eye_margin_bidir_scan(rank1, 0U, channel, 1U);
				}
			} else if (mode == 2U) {
				u32 ret = pmu_cal_slice_mask_eval(rank0);

				if (ret != 0U)
					return (u8)ret;
				if (dmem_read8(PMU_DMEM_CAL_ACTIVE_FLAG) != 0U)
					pmu_eye_margin_bidir_scan(rank0, 0U, channel, 0U);

				ret = pmu_cal_slice_mask_eval(rank1);
				if (ret != 0U)
					return (u8)ret;
				margin_window_ret = 1U;
				if (dmem_read8(PMU_DMEM_CAL_ACTIVE_FLAG) != 0U)
					pmu_eye_margin_bidir_scan(rank1, 0U, channel, 1U);
			} else if (mode == 3U) {
				u32 ret = pmu_eye_margin_step_balancer(rank0, rank1,
								       rank2, rank3);

				if (ret != 0U)
					return (u8)ret;
				if (dmem_read8(PMU_DMEM_CAL_ACTIVE_FLAG) != 0U) {
					pmu_eye_margin_bidir_scan(rank0, 0U, channel, 0U);
					pmu_eye_margin_bidir_scan(rank1, 0U, channel, 1U);
					pmu_eye_margin_bidir_scan(rank2, 0U, channel, 0U);
					pmu_eye_margin_bidir_scan(rank3, 0U, channel, 1U);
				}
			} else {
				pmu_assert_or_halt(0U, PMU_ERR_EYE_RANK_MODE_INVALID);
			}
		}
	}
	return 0U;
}

/**
 * pmu_cal_rank_margin_window_check() - Validate rank/slice margin windows and balance centers
 * @tap_ptr: Packed tap pointer or 0 for default per-channel mode
 * @channel: Target channel index (0 or 1)
 * @slice_mask: Active pin/lane bitmask
 *
 * Return: 0 on success, or non-zero calibration error code.
 */
u32 pmu_cal_rank_margin_window_check(u32 tap_ptr, u32 channel, u32 slice_mask)
{
	u32 start_slice = 0U;
	u32 end_slice = 0U;
	u32 rank_limit = 0U;
	u32 mode = pmu_phy_query_operating_mode();
	u32 ptr_addr;
	u8 c0;
	u8 c1;
	u32 table_ptr_base;
	u32 rank;

	if (channel == 1U) {
		u8 cal_param40;

		end_slice = dmem_read8(PMU_DMEM_RANK1_SLICE_END);
		start_slice = dmem_read8(PMU_DMEM_RANK1_SLICE_START);
		cal_param40 = dmem_read8(PMU_DMEM_CH1_RANK_EN);
		if (cal_param40 == 3U)
			rank_limit = 2U;
		else if (cal_param40 == 1U)
			rank_limit = 1U;
		else
			rank_limit = 0U;
	} else if (channel == 0U) {
		u8 cal_param25;

		end_slice = dmem_read8(PMU_DMEM_RANK_BOUNDARY);
		start_slice = dmem_read8(PMU_DMEM_RANK0_SLICE_START);
		cal_param25 = dmem_read8(PMU_DMEM_CH0_RANK_EN);
		rank_limit = (cal_param25 == 3U) ? 2U : 1U;
	}

	if (tap_ptr == 0U)
		return pmu_rank_margin_eval_zero_tap(start_slice, end_slice,
						     rank_limit, mode, channel,
						     slice_mask);

	ptr_addr = (tap_ptr >> 15) | (tap_ptr << 17);
	c0 = *(__volatile__ u8 *)(uintptr_t)ptr_addr;
	dmem_write16(PMU_DMEM_METRIC_COEFF_X, (u16)c0 * c0);
	c1 = *(__volatile__ u8 *)(uintptr_t)(ptr_addr + 1U);
	dmem_write16(PMU_DMEM_METRIC_COEFF_Y, (u16)c1 * c1);

	if (rank_limit == 0U)
		return 0U;

	table_ptr_base = (PMU_DMEM_BASE | PMU_DMEM_EYE_SAMPLE_BUF) +
			 (1320U * start_slice);

	for (rank = 0U; rank < rank_limit; rank++, table_ptr_base += 5280U) {
		u8 eye_window_buf_a[132];
		u8 eye_window_buf_b[132];
		int i;
		u32 slice_ptr;
		u32 slice;
		u32 ret;

		dmem_write8(PMU_DMEM_STAGE_PARAM_406, (u8)rank);

		for (i = 0; i < 64; i++) {
			((u16 *)eye_window_buf_a)[i] = 0x7f00U;
			((u16 *)eye_window_buf_b)[i] = 0x7f00U;
		}
		*(u32 *)&eye_window_buf_a[128] = 0x3f1f0000UL;
		*(u32 *)&eye_window_buf_b[128] = 0x001f0000UL;

		slice_ptr = table_ptr_base;
		for (slice = start_slice; slice <= end_slice;
		     slice++, slice_ptr += 1320U) {
			u8 *pin_ptr = (u8 *)(uintptr_t)slice_ptr;
			u32 pin;

			dmem_write8(PMU_DMEM_STAGE_PARAM_407, (u8)slice);

			for (pin = 0U; pin < 10U; pin++, pin_ptr += 132U) {
				u32 swap;
				u8 *target_buf;
				s32 delta;

				if (!(slice_mask & (1U << pin)))
					continue;

				ret = pmu_window_centroid_calc(pin_ptr);
				dmem_write8(PMU_DMEM_STAGE_PARAM_408, (u8)pin);
				if (ret != 0U)
					return (u8)ret;

				swap = pmu_cal_lane_error_status_check(rank, slice);
				target_buf = (swap != 0U) ? eye_window_buf_a :
							    eye_window_buf_b;

				delta = (s8)pin_ptr[130] - (s8)target_buf[130];
				ret = pmu_window_margin_diff_calc(target_buf,
								  pin_ptr,
								      target_buf,
								      delta, 0);
				if (ret != 0U)
					return (u8)ret;
			}
		}

		ret = pmu_cal_slice_mask_eval(eye_window_buf_a);
		if (ret != 0U)
			return (u8)ret;

		ret = pmu_cal_slice_mask_eval(eye_window_buf_b);
		if (ret != 0U)
			return (u8)ret;

		if (start_slice <= end_slice) {
			slice_ptr = table_ptr_base;
			for (slice = start_slice; slice <= end_slice;
			     slice++, slice_ptr += 1320U) {
				u8 *pin_ptr = (u8 *)(uintptr_t)slice_ptr;
				u32 pin;

				dmem_write8(PMU_DMEM_STAGE_PARAM_407, (u8)slice);

				for (pin = 0U; pin < 10U;
				     pin++, pin_ptr += 132U) {
					u32 swap;
					u8 *target_buf;
					u8 target_y;

					if (!(slice_mask & (1U << pin)))
						continue;

					dmem_write8(PMU_DMEM_STAGE_PARAM_408,
						    (u8)pin);

					swap = pmu_cal_lane_error_status_check(rank, slice);
					target_buf = (swap != 0U) ?
						eye_window_buf_a :
						eye_window_buf_b;
					target_y = target_buf[131];

					ret = pmu_eye_boundary_sample_search(pin_ptr,
									     target_y);
					if (ret != 0U)
						return (u8)ret;

					if (dmem_read8(PMU_DMEM_CAL_ACTIVE_FLAG) != 0U)
						pmu_eye_margin_bidir_scan(pin_ptr,
									  tap_ptr,
									  channel,
									  rank);
				}
			}
		}
	}

	return 0U;
}
