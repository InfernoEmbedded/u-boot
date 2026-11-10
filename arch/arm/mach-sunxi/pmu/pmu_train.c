// SPDX-License-Identifier: GPL-2.0+
/*
 * Allwinner A733 Cadence DDR PHY PMU Top-Level Calibration Dispatch Helpers
 */

#include "pmu_fw.h"

static void pmu_cal_ca_train_step(void);
static void pmu_cal_ca_vref_step(u32 arg0);
static u32 pmu_cal_dbyte_impedance_fine_tune(void);
static void pmu_cal_multi_branch_dispatch(u16 *out_desc, u32 flags, u32 opcode, u32 dispatch_param,
					  u32 val_arg0, u32 val_arg1, u32 flag_mode);
static void pmu_cal_multi_param_step_commit(u32 param_table, u32 step_val, u32 eval_mask,
					    u32 filter_mask, u32 slice_flags);
static void pmu_cal_phase_delay_tracker_update(void);
static int pmu_cal_scaled_step_exec(int count, u32 flags);
static void pmu_cal_stage1_pre_init(void);
static void pmu_cal_stage39_post_results(void);
static void pmu_cal_stage8_pad_init(void);
static void pmu_cal_state_restore_offset_prog(void);
static void pmu_cal_timing_margin_stage10(void);
static u32 pmu_cal_zq_pad_calibration(u32 is_primary);
static void pmu_multiparam_cal_dispatch(void);
static void pmu_phy_deskew_reset_strobe(void);
static void pmu_train_dispatcher(void);

/**
 * pmu_cal_slice_mask_eval() - Dispatch CA Vref or deskew engine by DCCM 0x0b bit 2
 * @table_buf: Pointer to slice descriptor buffer forwarded in r0
 *
 * Return: Result returned by pmu_window_centroid_calc(@table_buf, 0) when bit 2
 *         of DCCM 0x8000000b is set, or pmu_eye_margin_window_search(@table_buf, 0)
 *         when clear.
 */
u32 pmu_cal_slice_mask_eval(void *table_buf)
{
	if (*(__volatile__ u8 *)(PMU_DMEM_BASE + PMU_DMEM_CAL_SELECT_FLAGS) & 0x04U)
		return pmu_window_centroid_calc(table_buf);
	return pmu_eye_margin_window_search(table_buf, 0U);
}

/**
 * pmu_cal_training_flags_gate_update() - Update gate/mode status bits in descriptor
 * @descriptor: Pointer to calibration control descriptor byte array
 */
void pmu_cal_training_flags_gate_update(u8 *descriptor)
{
	u32 d13;

	descriptor[11] &= 0x87U;
	descriptor[1] &= (u8)~0x08U;
	descriptor[19] &= (u8)~0x0cU;

	d13 = descriptor[13];
	if (pmu_dmem_training_flags_eval() != 0U) {
		u32 d3 = descriptor[3];

		d13 |= 0x20U;
		d3 |= 0x80U;
		descriptor[3] = (u8)d3;
	}

	descriptor[20] = 2U;
	descriptor[13] = (u8)(d13 & ~(1U << 4));
	descriptor[16] = 0x40U;
	descriptor[17] |= 0x38U;
	descriptor[18] &= 0x03U;
	descriptor[28] &= (u8)~(1U << 5);
	descriptor += 20;
	descriptor[21] &= 0x1fU;
}

/**
 * pmu_cal_tracker_reset_and_apb_zero() - Stream halfword results to PHY CSR 0x90082000
 * @buf: Pointer to 16-bit halfword array to stream
 * @len: Number of 16-bit halfwords to stream (minimum 1)
 */
void pmu_cal_tracker_reset_and_apb_zero(const void *buf, u32 len)
{
	const u16 *src = (const u16 *)buf;
	u16 marker = pmu_sdata()->cal_marker_b;
	u32 reg_addr = PHY_REG_CSR_RESULT_STREAM | ((u32)marker << 1);
	u32 i = 0U;
	u16 updated;

	do {
		*(__volatile__ u16 *)reg_addr = *src++;
		reg_addr += 2U;
		i++;
	} while (i < len);

	updated = (u16)(len + marker);
	pmu_sdata()->cal_marker_a = updated;
	pmu_sdata()->cal_marker_b = updated;
}

/**
 * pmu_cal_scaled_step_exec() - Execute scaled calibration step count wrapper
 * @count: Signed calibration step count (returns @count immediately if < 1)
 * @flags: Control flags (bit 9 or DCCM 0x80000469 selects mode 0 vs mode 1)
 *
 * Return: Updated 16-bit calibration marker from pmu_cal_tracker_step_advance(), or
 *         @count when @count < 1.
 */
static int pmu_cal_scaled_step_exec(int count, u32 flags)
{
	__volatile__ u8 *m = (__volatile__ u8 *)&pmu_sdata()->cal_marker_a;
	u16 saved_marker;
	u16 res;
	int quotient;
	u32 mode;

	if (count < 1)
		return count;

	quotient = count / (int)*(const u8 *)(m + 2);
	saved_marker = *(const u16 *)m;
	mode = ((flags & 0x0200UL) != 0UL || *(const u8 *)(m + 17) != 0U) ?
	       0U : 1U;

	res = pmu_cal_tracker_step_advance(saved_marker, (u16)quotient, mode);
	*(__volatile__ u16 *)m = res;
	return (int)res;
}

/**
 * pmu_cal_window_params_dispatch() - Pack 4-byte window parameters into 22-halfword tracker
 * @params_buf: Pointer to 4-byte window parameter array
 * @base_val: Base halfword value placed at index 5 and/or 13
 * @mode_flags: Bitmask (bit 0 sets buf[5], bit 1 sets buf[13])
 *
 * Return: Result from pmu_cal_tracker_csr_commit(buf).
 */
u32 pmu_cal_window_params_dispatch(const u8 *params_buf, u16 base_val, u32 mode_flags)
{
	u16 buf[22];
	u16 *b = memset(buf, 0, sizeof(buf));

	b[3] = params_buf[0];
	b[4] = params_buf[1];
	b[11] = params_buf[2];
	b[12] = params_buf[3];

	if ((mode_flags & 1U) != 0U)
		b[5] = base_val;
	if ((mode_flags & 2U) != 0U)
		b[13] = base_val;

	return pmu_cal_tracker_csr_commit(b);
}

/**
 * pmu_dly_reg_offset_select() - Select delay register offset by rank and mode
 * @rank: Target rank index (0 or non-zero)
 * @unused: Fallback direction selector when operating mode >= 3
 * @direction: Sweep direction selector when operating mode == 2
 *
 * Return: Selected register index in {0x4e, 0x4f, 0x50, 0x51}.
 */
u32 pmu_dly_reg_offset_select(u32 rank, u32 unused, u32 direction)
{
	u32 mode = pmu_phy_query_operating_mode();
	u32 base = (rank == 0U) ? 0x4fU : 0x4eU;
	u32 dir_sel = (mode == 2U) ? direction : unused;

	if (mode == 1U || (mode >= 2U && dir_sel != 0U))
		base += 2U;

	return base;
}

/**
 * pmu_cal_state_restore_offset_prog() - Configure calibration strobe and Vref latch CSRs
 */
static void pmu_cal_state_restore_offset_prog(void)
{
	u32 param05 = pmu_get_dmem_005();
	__volatile__ u16 *b5e = (__volatile__ u16 *)PHY_REG_PUB_SEC_BASE;
	u16 vref_val;
	__volatile__ u16 *c50;

	pmu_delay_us(0U, 0x40U);
	b5e[0x620 >> 1] = 1U;
	b5e[0x1f0 >> 1] = 1U;

	vref_val = *(__volatile__ const u16 *)PHY_REG_GATE_STAT_092;

	if (param05 != 0U) {
		b5e[0x186 >> 1] = 1U;
	} else {
		__volatile__ u16 *p40;
		u16 cal_code;

		vref_val &= 496U;
		b5e[0x092 >> 1] = vref_val | (1U << 10);
		pmu_delay_us(1000U, 0U);
		pmu_delay_us(1000000U, 0U);
		b5e[0x1c0 >> 1] = 1U;
		vref_val |= (1U << 9);
		p40 = (__volatile__ u16 *)PHY_REG_MASTER_BASE;
		cal_code = p40[0x19e >> 1];
		b5e[0x186 >> 1] = 1U;
		b5e[0x1c0 >> 1] = 0U;
		b5e[0x092 >> 1] = vref_val;
		p40[(0x09c >> 1) + pmu_sdata()->active_csr_offset] =
			cal_code & 0x1fU;
	}

	c50 = (__volatile__ u16 *)PHY_REG_CAL_RESTORE_TRIG_050;
	if (pmu_sdata()->freq_mode != 0U)
		c50 += 5;
	*c50 = 1U;
}

/**
 * pmu_freq_scaled_timing_step_calc() - Compute frequency-scaled timing parameter step
 *
 * Return: Clamped 8-bit timing step count (minimum 4).
 */
u32 pmu_freq_scaled_timing_step_calc(void)
{
	u32 mode_8a = pmu_is_lpddr4();
	u32 nibble_96 = pmu_get_train_cfg96_bit4();
	u32 ps = (mode_8a != 0U) ? 19000U :
		 ((nibble_96 != 0U) ? 16000U : 12000U);
	u32 steps;

	if (*(__volatile__ const u8 *)PMU_DMEM_DESKEW_LIM_1C == 0U)
		ps += 2000U;

	steps = pmu_freq_delay_step_calc(ps, 0U);
	if (steps < 4U)
		steps = 4U;

	return (u8)steps;
}

/**
 * pmu_cal_dbyte_impedance_fine_tune() - Build primary/secondary rank slice maps
 *
 * Return: Always 1.
 */
static u32 pmu_cal_dbyte_impedance_fine_tune(void)
{
	const __volatile__ u8 *low =
		(const __volatile__ u8 *)PMU_DMEM_DESKEW_LIM_1C;
	__volatile__ u8 *b24 = (__volatile__ u8 *)PMU_DMEM_SLICE_IMP_CFG_B24;
	u32 rank_count = (pmu_get_phy_148() >> 6) & 0x3U;
	u32 num_channels = pmu_sdata()->active_ch_cnt;
	u32 rank, ch;

	for (rank = 0U; rank < rank_count; rank++) {
		for (ch = 0U; ch < num_channels; ch++) {
			u8 rank_mask = (ch != 0U) ? low[36] : low[9];
			const __volatile__ u8 *lanes;
			u32 base_off;
			u32 lane_start;
			u32 lane_end;
			__volatile__ u8 *pri;
			__volatile__ u8 *sec;
			u32 bit_query;
			u32 swap;
			u32 cur;

			if ((rank_mask & (1U << rank)) == 0U)
				continue;

			lanes = b24 + ((ch != 0U) ? 0x48U : 0x46U);
			lane_start = lanes[0];
			lane_end = lanes[1];
			base_off = (ch << 4) + (rank << 3);

			pri = b24 + 0x20U + base_off;
			sec = b24 + base_off;
			bit_query = pmu_get_dmem_1c_bit(ch, rank);
			swap = ((low[0xe5] >> (rank + (ch << 1))) & 1U);

			for (cur = lane_start; cur <= lane_end; cur += 2U) {
				if (bit_query != 0U) {
					*pri++ = (u8)cur;
					*pri++ = (u8)(cur + 1U);
				} else {
					*pri++ = (u8)(cur + swap);
					*sec++ = (u8)(cur + (swap ^ 1U));
				}
			}
		}
	}

	return 1U;
}

/**
 * pmu_cal_marker_win_step_prog() - Program calibration marker pair and window step diff
 * @marker0: First 16-bit calibration marker value
 * @marker1: Second 16-bit calibration marker value
 * @win_param: Window base halfword parameter
 * @win_mode: Window mode byte
 */
void pmu_cal_marker_win_step_prog(u32 marker0, u32 marker1, u32 win_param,
				  u32 win_mode)
{
	u32 buf_word = (win_mode & 0xffU) * 0x0101U;
	u32 mode = (win_mode == 0U) ? 3U : 1U;

	pmu_set_458_pair((u16)(marker0));
	pmu_cal_window_params_dispatch((const u8 *)&buf_word, (u16)(win_param), (u16)(mode));
	pmu_set_458_pair((u16)(marker1));
	pmu_cal_slice_step_diff_latch(0U, 1U);
}

/**
 * pack_quad_b7() - Pack a 2-bit phase selector and 5-bit value into a 7-bit descriptor field
 * @p: 2-bit phase selector placed in bits [5:4]
 * @v: 5-bit value with bit 4 placed in bit 6 and bits [3:0] placed in bits [3:0]
 *
 * Return: Packed 16-bit descriptor field.
 */
static __always_inline u16 pack_quad_b7(u32 p, u32 v)
{
	return (u16)(((v << 2) & 0x40U) | ((p << 3) & 0x30U) | (v & 0x0fU));
}

/**
 * pmu_cal_encode_opcode_pair() - Encode primary d3/d4 words for a calibration opcode
 * @opcode: Calibration opcode (0..47)
 * @dispatch_param: Opcode-specific parameter word
 * @val_arg0: Primary value argument
 * @flag_mode: Mode selector (0 vs non-zero)
 * @d3_out: Output pointer for descriptor word 3
 * @d4_out: Output pointer for descriptor word 4
 */
static void pmu_cal_encode_opcode_pair(u32 opcode, u32 dispatch_param,
				       u32 val_arg0, u32 flag_mode,
				       u16 *d3_out, u16 *d4_out)
{
	u16 xbfu_val = (u16)((dispatch_param >> 8) & 0x7fU);
	u16 d3 = 0U, d4 = 0U;

	switch (opcode) {
	case 0:
		if (flag_mode != 0U) {
			d4 = (u16)(dispatch_param & 0x7fU);
			d3 = (u16)(((dispatch_param >> 4) & 0x78U) | 3U);
		} else {
			d4 = (u16)(val_arg0 | ((dispatch_param >> 7) & 0x70U));
			d3 = (u16)(((dispatch_param >> 11) & 0x78U) | 7U);
		}
		break;
	case 1:
		d4 = pack_quad_b7(dispatch_param, val_arg0);
		d3 = (u16)(((dispatch_param << 3) & 0x08U) |
			   ((dispatch_param >> 1) & 0x1cU) | 1U);
		break;
	case 2:
		d4 = pack_quad_b7(dispatch_param, val_arg0);
		d3 = (u16)(((dispatch_param << 3) & 0x08U) |
			   ((dispatch_param >> 1) & 0x18U) | 5U);
		break;
	case 3:
		d4 = pack_quad_b7(dispatch_param, val_arg0);
		d3 = (u16)(((dispatch_param << 3) & 0x08U) |
			   ((dispatch_param >> 1) & 0x18U) | 6U);
		break;
	case 4:
		d4 = pack_quad_b7(dispatch_param, val_arg0);
		d3 = (u16)(((dispatch_param >> 1) & 0x18U) | 4U);
		break;
	case 5:
		d4 = (u16)(val_arg0 & 0x7fU);
		d3 = (u16)(((dispatch_param >> 3) & 0x70U) | 0x0cU);
		break;
	case 6:
		if (flag_mode == 0U) {
			d4 = (u16)(val_arg0 & 0x7fU);
			d3 = 0x58U;
		} else {
			d4 = (u16)(dispatch_param & 0x7fU);
			d3 = (u16)(((dispatch_param >> 1) & 0x40U) | 8U);
		}
		break;
	case 7:
	case 39:
		break;
	case 8:
	case 27:
		d3 = 0x78U;
		d4 = (u16)(val_arg0 & 0x0fU);
		break;
	case 10:
	case 17:
		d3 = 0x38U;
		d4 = (u16)(val_arg0 & 0x07U);
		break;
	case 11:
		d3 = 0x68U;
		d4 = (u16)((dispatch_param << 5) & 0x60U);
		break;
	case 12:
		d3 = 0x28U;
		break;
	case 14:
		if (flag_mode == 0U) {
			d4 = xbfu_val;
			d3 = 0x58U;
		} else {
			d4 = (u16)(dispatch_param & 0x7fU);
			d3 = (u16)(((dispatch_param >> 1) & 0x40U) | 8U);
		}
		break;
	case 15:
		d4 = (u16)(val_arg0 & 0x7fU);
		d3 = 0x18U;
		break;
	case 16:
		d3 = 0x38U;
		d4 = (u16)((val_arg0 & 0x07U) | 0x40U);
		break;
	case 25:
		d4 = (u16)(dispatch_param & 0x7fU);
		d3 = (u16)(((dispatch_param >> 1) & 0x40U) | 0x30U);
		break;
	case 26:
		d3 = 0x78U;
		d4 = (u16)((val_arg0 & 0x0fU) | 0x40U);
		break;
	case 28:
		d4 = pack_quad_b7(dispatch_param, val_arg0);
		d3 = (u16)(((dispatch_param >> 1) & 0x1cU) |
			   ((dispatch_param << 3) & 0x08U) | 2U);
		break;
	case 29:
		d3 = 0x58U;
		d4 = 0x58U;
		break;
	case 30:
		d3 = xbfu_val;
		d4 = xbfu_val;
		break;
	case 31:
		d3 = (u16)(((dispatch_param >> 1) & 0x40U) | 8U);
		d4 = d3;
		break;
	case 32:
	case 38:
		d3 = (u16)(dispatch_param & 0x7fU);
		d4 = d3;
		break;
	case 33:
		d3 = 0x18U;
		break;
	case 34:
		d4 = xbfu_val;
		break;
	case 35:
		d3 = (u16)(((dispatch_param >> 3) & 0x70U) | 0x0cU);
		d4 = d3;
		break;
	case 36:
		d3 = (u16)(val_arg0 & 0x7fU);
		d4 = d3;
		break;
	case 37:
		d3 = (u16)(((dispatch_param >> 1) & 0x40U) | 0x30U);
		d4 = d3;
		break;
	case 40:
		d3 = (flag_mode == 0U) ? 0x40U : 0U;
		break;
	case 41:
		d3 = 0x60U;
		break;
	case 42:
		d3 = 0x20U;
		break;
	case 43:
		d3 = 0x50U;
		break;
	case 44:
		d3 = 0x38U;
		d4 = 0x38U;
		break;
	case 45:
		d3 = (u16)((val_arg0 & 7U) | 0x40U);
		d4 = d3;
		break;
	case 46:
		d3 = (u16)(((dispatch_param << 3) & 0x08U) |
			   ((dispatch_param >> 1) & 0x1cU) | 1U);
		d4 = d3;
		break;
	case 47:
		d3 = pack_quad_b7(dispatch_param, val_arg0);
		d4 = d3;
		break;
	default:
		break;
	}

	*d3_out = d3;
	*d4_out = d4;
}

/**
 * pmu_cal_multi_branch_dispatch() - Format 8-halfword calibration command descriptor
 * @out_desc: Output 8-halfword descriptor buffer
 * @flags: Command flag bitmask
 * @opcode: Calibration opcode (0..47)
 * @dispatch_param: Opcode-specific parameter word
 * @val_arg0: Primary value argument
 * @val_arg1: Secondary value / slice mask argument
 * @flag_mode: Mode selector (0 vs non-zero)
 */
static noinline void pmu_cal_multi_branch_dispatch(u16 *out_desc, u32 flags,
						   u32 opcode, u32 dispatch_param,
						   u32 val_arg0, u32 val_arg1,
						   u32 flag_mode)
{
	u16 fc = 0U;

	out_desc[0] = 0U;
	out_desc[1] = 0U;
	out_desc[2] = 0U;
	out_desc[6] = 0U;
	out_desc[7] = 0U;

	if (opcode <= 0x2fU)
		pmu_cal_encode_opcode_pair(opcode, dispatch_param, val_arg0,
					   flag_mode, &out_desc[3],
					   &out_desc[4]);

	if ((flags & (1U << 16)) != 0U ||
	    (opcode == 0x28U && flag_mode != 0U))
		out_desc[5] = 0U;
	else
		out_desc[5] = (u16)(val_arg1 & 3U);

	if ((flags & (1U << 20)) != 0U)
		out_desc[7] = 13U;
	else if ((flags & (1U << 21)) != 0U)
		out_desc[7] = 0xd0U;
	else if ((flags & (1U << 22)) != 0U)
		out_desc[7] = 0xd00U;
	else if ((flags & (1U << 23)) != 0U)
		out_desc[7] = 8U;
	else if ((flags & (1U << 24)) != 0U)
		out_desc[7] = 0x80U;
	else if ((flags & (1U << 25)) != 0U)
		out_desc[7] = 1024U;
	else if ((flags & (1U << 26)) != 0U)
		out_desc[7] = 256U;
	else if ((flags & (1U << 27)) != 0U)
		out_desc[7] = 512U;

	if ((flags & (1U << 19)) != 0U)
		out_desc[7] |= 0x1000U;

	if (opcode == 3U || opcode == 4U || opcode == 28U || opcode == 41U) {
		fc = 4U;
	} else if (opcode == 1U || opcode == 2U ||
		   opcode == 33U || opcode == 34U) {
		fc = ((flags & (1U << 3)) != 0U) ? 0U : 2U;
		if ((flags & 0x0cU) == 0x0cU)
			fc |= 1U;
	}

	if ((flags & (1U << 18)) != 0U)
		fc |= 3U;

	if ((flags & (1U << 15)) != 0U)
		out_desc[2] = 1U;

	if ((flags & (1U << 12)) != 0U)
		fc |= 8U;

	out_desc[6] = fc;

	if ((flags & (1U << 6)) != 0U || (val_arg1 & (1U << 1)) != 0U)
		out_desc[0] = 1U;
}

/**
 * copy_slice_quad_b7() - Scatter 4 per-slice profile bytes into a 4x54-byte slice matrix
 * @buf: Destination 216-byte (4 x 54-byte) slice timing matrix buffer
 * @off: Column offset within each 54-byte slice row (0..53)
 * @src: Source 4-byte per-slice array in DCCM
 */
static __always_inline void copy_slice_quad_b7(u8 *buf, u32 off,
					       const __volatile__ u8 *src)
{
	buf[off] = src[0];
	buf[off + PMU_SLICE_DESC_SIZE] = src[1];
	buf[off + (2U * PMU_SLICE_DESC_SIZE)] = src[2];
	buf[off + (3U * PMU_SLICE_DESC_SIZE)] = src[3];
}

/**
 * pmu_cal_timing_table_matrix_init() - Unpack 29 slice profile quads into 4-slice matrix
 * @buf: Destination 216-byte (4 x 54-byte) slice timing matrix buffer
 */
void pmu_cal_timing_table_matrix_init(u8 *buf)
{
	const __volatile__ u8 *src =
		(const __volatile__ u8 *)PMU_DMEM_SLICE_PROFILE_5A;

	memset(buf, 0, PMU_SLICE_DESC_SIZE);

	copy_slice_quad_b7(buf, 1U, src); src += 4;
	copy_slice_quad_b7(buf, 2U, src); src += 4;
	copy_slice_quad_b7(buf, 3U, src); src += 4;
	copy_slice_quad_b7(buf, 10U, src); src += 4;
	copy_slice_quad_b7(buf, 11U, src); src += 4;
	copy_slice_quad_b7(buf, 12U, src); src += 4;
	copy_slice_quad_b7(buf, 13U, src); src += 4;
	copy_slice_quad_b7(buf, 14U, src); src += 4;
	copy_slice_quad_b7(buf, 15U, src); src += 4;
	copy_slice_quad_b7(buf, 16U, src); src += 4;
	copy_slice_quad_b7(buf, 17U, src); src += 4;
	copy_slice_quad_b7(buf, 18U, src); src += 4;
	copy_slice_quad_b7(buf, 19U, src); src += 4;
	copy_slice_quad_b7(buf, 20U, src); src += 4;
	copy_slice_quad_b7(buf, 21U, src); src += 4;
	copy_slice_quad_b7(buf, 22U, src); src += 4;
	copy_slice_quad_b7(buf, 24U, src); src += 4;
	copy_slice_quad_b7(buf, 25U, src); src += 4;
	copy_slice_quad_b7(buf, 26U, src); src += 4;
	copy_slice_quad_b7(buf, 27U, src); src += 4;
	copy_slice_quad_b7(buf, 28U, src); src += 4;
	copy_slice_quad_b7(buf, 30U, src); src += 4;
	copy_slice_quad_b7(buf, 31U, src); src += 4;
	copy_slice_quad_b7(buf, 32U, src); src += 4;
	copy_slice_quad_b7(buf, 33U, src); src += 4;
	copy_slice_quad_b7(buf, 34U, src); src += 4;
	copy_slice_quad_b7(buf, 37U, src); src += 4;
	copy_slice_quad_b7(buf, 40U, src); src += 4;
	copy_slice_quad_b7(buf, 41U, src);

	buf[46] = 0U;
	buf[100] = 0U;
	buf[154] = 0U;
	buf[208] = 0U;
}

/**
 * pmu_cal_stage8_pad_init() - Initialize Stage 8 pad configuration and log telemetry
 */
static void pmu_cal_stage8_pad_init(void)
{
	u32 stat = pmu_get_phy_148();
	u32 channels = (stat >> 4) & 3U;
	u32 rank_metric = channels * (stat & 0x0fU);
	u8 rank1_en;
	u32 pad_cfg;
	u32 val_8;
	u16 pub_ctl;
	u8 ctl_bit;
	u8 dram_type;

	pmu_sdata()->active_ch_cnt = channels;
	pmu_sdata()->active_slice_cnt = rank_metric;

	rank1_en = dmem_read8(PMU_DMEM_CH1_ANALOG_EN_40);

	if (rank1_en != 0U) {
		pad_cfg = ((rank_metric << 2) & 248U) +
			  dmem_read8(PMU_DMEM_RANK_DIV_3F);
	} else {
		pad_cfg = dmem_read8(PMU_DMEM_TOTAL_BITS_24);
		if (channels == 2U) {
			rank_metric >>= 1;
			pmu_sdata()->active_slice_cnt = rank_metric;
		}
	}

	val_8 = (u32)((u8)pad_cfg + 7U) >> 3;
	pmu_sdata()->dbyte_slice_cnt = (u8)val_8;

	pub_ctl = *(__volatile__ const u16 *)PHY_REG_PUB_CFG_100;
	ctl_bit = 1U & ~((u8)(pub_ctl >> 2));
	pmu_cal_win_bounds_clear();
	pmu_sdata()->timer_shift = ctl_bit;

	phy_write16(PHY_REG_PAD_CAL_CTRL_050, 1U);
	phy_write16(PHY_REG_SEC_CTRL_14A, 1U);
	phy_write16(PHY_REG_SEC_CTRL_0EE, 0U);

	dram_type = dmem_read8(PMU_DMEM_DRAM_TYPE);
	pmu_set_46a(dram_type);

	if (dmem_read16(PMU_DMEM_SEQ_CTRL_10) != 1U &&
	    (dmem_read8(PMU_DMEM_CHANNEL_CFG) & 0xf0U) == 0x20U)
		pmu_rank_shadow_status_sample();

	if (dmem_read8(PMU_DMEM_AC_PROFILE_EN_F4 & 0xffffU) != 0U)
		pmu_cal_vref_trim_cfg_latch();
}

struct dly_line_entry_b7 {
	u16 val16;
	u16 pad;
};

/**
 * pmu_cal_stage42_accum_dispatch() - Unpack, align, and pack 4-group LCDL coarse/fine pairs
 * @dram_type: Active DRAM type identifier
 * @count: Number of active slice entries (clamped to 4)
 * @slice_base: Pointer to delay line entry matrix
 */
void pmu_cal_stage42_accum_dispatch(u32 dram_type, u32 count, void *slice_base)
{
	u8 cand[4][4][2];
	u8 thresh1[4];
	u8 thresh2[4];
	u32 max_grp1 = 0U;
	u32 max_grp2 = 0U;
	struct dly_line_entry_b7 (*buf)[2] =
		(struct dly_line_entry_b7 (*)[2])slice_base;
	u32 use_max;
	u32 i, g, k;

	if (count == 0U)
		return;
	if (count > 4U)
		count = 4U;

	for (i = 0U; i < count; i++) {
		u8 d[4];
		u32 m1, m2;

		for (g = 0U; g < 4U; g++) {
			u16 v = buf[g][i].val16;

			cand[g][i][0] = (u8)(v >> 6);
			cand[g][i][1] = (u8)(v & 0x3fU);
			pmu_adjust_step_limit(cand[g][i], g >> 1);
		}

		pmu_eye_margin_step_align(cand[0][i], cand[1][i], 2U);
		pmu_eye_margin_step_align(cand[2][i], cand[3][i], 2U);

		for (g = 0U; g < 4U; g++) {
			u8 c = cand[g][i][1];

			d[g] = (c < 65U) ? (64U - c) : (c - 64U);
		}

		m1 = (d[0] > d[1]) ? d[0] : d[1];
		m2 = (d[2] > d[3]) ? d[2] : d[3];
		thresh1[i] = (u8)m1;
		thresh2[i] = (u8)m2;
		if (m1 > max_grp1)
			max_grp1 = m1;
		if (m2 > max_grp2)
			max_grp2 = m2;
	}

	use_max = (dram_type < 4U && dram_type != 2U &&
		   *(__volatile__ const u8 *)PMU_DMEM_CAL_OVERRIDE_CE == 0U);

	for (k = 0U; k < count; k++) {
		u32 t1 = use_max ? max_grp1 : thresh1[k];
		u32 t2 = use_max ? max_grp2 : thresh2[k];

		pmu_cal_lcdl_coarse_fine_pair_pack(k, t1, t2,
						   cand[2][k], cand[3][k],
						   cand[0][k], cand[1][k],
						   slice_base);
	}
}

/**
 * pmu_cal_pulse_clear_slot1() - Zero the second 8-halfword command slot in pd9c
 * @pd9c: Base pointer to the 38-halfword command buffer at 0x80000d9c
 */
static void pmu_cal_pulse_clear_slot1(u16 *pd9c)
{
	__volatile__ u32 *slot1 = (__volatile__ u32 *)pd9c + 4;

	slot1[0] = 0U;
	slot1[1] = 0U;
	slot1[2] = 0U;
	slot1[3] = 0U;
}

/**
 * pmu_cal_pulse_handle_type6() - Format and commit a fast-path type-6 pulse
 * @p458: Tracker state pointer at pmu_gp + 88
 * @phase_id: Target phase count
 * @param_code: Calibration parameter code
 * @tap_idx: Delay tap / register index
 * @slice_mask: Target slice or rank bitmask
 *
 * Return: Residual step difference after committing the type-6 tracker descriptor.
 */
static int pmu_cal_pulse_handle_type6(__volatile__ u8 *p458, u32 phase_id,
				      u32 param_code, u32 tap_idx,
				      u32 slice_mask)
{
	__volatile__ u16 *d70 = (__volatile__ u16 *)PMU_DMEM_ALT_TRACKER_D70;
	u16 a5_low = (u16)(slice_mask & 3U);
	u16 a5_shift = a5_low >> 1;
	u16 marker_a = *(__volatile__ u16 *)p458;
	u32 cbt_state;

	d70[0] = a5_shift;
	d70[3] = 0x58U;
	d70[4] = (u16)tap_idx;
	d70[5] = a5_low;
	d70[8] = a5_shift;
	d70[11] = (u16)(((param_code >> 1) & 64U) | 8U);
	d70[12] = (u16)(param_code & 0x7fU);
	d70[13] = a5_low;
	*(__volatile__ u16 *)p458 = marker_a + 8U;

	pmu_cal_tracker_csr_commit((const void *)d70);
	cbt_state = p458[2];
	if ((marker_a & 7U) == 0U)
		cbt_state <<= 1;

	return (int)phase_id - (int)cbt_state;
}

/**
 * pmu_cal_pulse_handle_extra() - Format and advance a multi-step extra pulse
 * @p458: Tracker state pointer at pmu_gp + 88
 * @pd9c: Base pointer to the 38-halfword command buffer at 0x80000d9c
 * @cmd_flags: In/out pointer to command control flags
 * @pulse_type: Pulse opcode / sequence type
 * @phase_id: Phase count after v24 adjustment
 * @param_code: Calibration parameter code
 * @tap_idx: Delay tap / register index
 * @slice_mask: Target slice or rank bitmask
 * @extra: Multi-step pulse count
 */
static void pmu_cal_pulse_handle_extra(__volatile__ u8 *p458, u16 *pd9c,
				       u32 *cmd_flags, u32 pulse_type,
				       u32 phase_id, u32 param_code,
				       u32 tap_idx, u32 slice_mask, u32 extra)
{
	u16 marker_a = *(__volatile__ u16 *)p458;
	u32 cbt_state;
	int sub1_diff;

	if ((marker_a & 7U) == 4U) {
		pmu_cal_pulse_clear_slot1(pd9c);
		*(__volatile__ u16 *)p458 = marker_a + 4U;
		pmu_cal_marker_status_update();
	}

	if (p458[3] == 0U)
		p458[3] = p458[4];

	pmu_cal_multi_branch_dispatch(pd9c, *cmd_flags, pulse_type,
				      param_code, tap_idx, slice_mask, 0U);

	if (pmu_check_stage(pulse_type) == 1U)
		pmu_cal_multi_branch_dispatch(pd9c + 8, *cmd_flags, pulse_type,
					      param_code, tap_idx, slice_mask,
					      1U);
	else
		pmu_cal_pulse_clear_slot1(pd9c);

	pd9c[17] = (u16)extra;
	cbt_state = p458[2];
	marker_a = *(__volatile__ u16 *)p458;
	*(__volatile__ u16 *)p458 = marker_a + 8U;
	sub1_diff = (int)phase_id - (int)(cbt_state << 1);

	if (sub1_diff < 1) {
		pd9c[18] = 3U;
	} else {
		int div_val;
		u16 cur_m;

		pd9c[18] = 2U;
		pmu_cal_marker_status_update();
		cbt_state = p458[2];
		div_val = (sub1_diff + (int)cbt_state - 1) / (int)cbt_state;
		cur_m = *(__volatile__ u16 *)p458;
		*(__volatile__ u16 *)p458 =
			pmu_cal_tracker_step_advance(cur_m, (u16)div_val, 0U);
		*cmd_flags |= (1U << 11);
	}
}

/**
 * pmu_cal_sequence_pulse_send() - Master calibration pulse sequence dispatcher
 * @cmd_flags: Command control flags
 * @pulse_type: Pulse opcode / sequence type
 * @phase_id: Phase count or step target
 * @param_code: Calibration parameter code
 * @tap_idx: Delay tap / register index
 * @slice_mask: Target slice or rank bitmask
 * @extra: Non-zero for multi-step pulse dispatch
 *
 * Return: Calibration marker verification status.
 */
noinline u32 pmu_cal_sequence_pulse_send(u32 cmd_flags, u32 pulse_type,
					 u32 phase_id, u32 param_code,
					 u32 tap_idx, u32 slice_mask,
					 u32 extra)
{
	__volatile__ u8 *p458 = (__volatile__ u8 *)&pmu_sdata()->cal_marker_a;
	u16 *pd9c = (u16 *)pmu_tracker();
	int step_diff;

	while (1) {
		u8 v24 = p458[0x13];
		u16 marker_a;
		u32 chk;

		if (pulse_type == 6U && v24 != 0U) {
			step_diff = pmu_cal_pulse_handle_type6(p458, phase_id,
							       param_code,
							       tap_idx,
							       slice_mask);
			goto out_0fc8;
		}

		if (p458[0x10] == 1U) {
			u32 stage = pmu_check_stage(pulse_type);
			u8 prefix = (stage == 1U) ? 0xf0U : 0x30U;

			p458[8] = prefix + (p458[8] >> 4);
		}

		if ((cmd_flags & (1U << 8)) != 0U)
			pd9c[16] |= (1U << 1);
		if ((cmd_flags & (1U << 13)) != 0U)
			pd9c[16] |= (1U << 0);

		marker_a = *(__volatile__ u16 *)p458;

		if (extra != 0U) {
			if (v24 == 0U)
				phase_id++;
			pmu_cal_pulse_handle_extra(p458, pd9c, &cmd_flags,
						   pulse_type, phase_id,
						   param_code, tap_idx,
						   slice_mask, extra);
			goto out_set3;
		}

		if ((marker_a & 7U) == 0U) {
			if (p458[3] == 0U)
				p458[3] = p458[4];

			pmu_cal_multi_branch_dispatch(pd9c, cmd_flags,
						      pulse_type, param_code,
						      tap_idx, slice_mask, 0U);
			if (v24 == 0U)
				phase_id++;

			*(__volatile__ u16 *)p458 =
				*(__volatile__ u16 *)p458 + 4U;
			if ((cmd_flags & (1U << 9)) != 0U)
				pd9c[18] |= (1U << 1);

			step_diff = (int)phase_id - (int)p458[2];
			if (pmu_check_stage(pulse_type) == 1U) {
				pmu_cal_multi_branch_dispatch(pd9c + 8,
							      cmd_flags,
							      pulse_type,
							      param_code,
							      tap_idx,
							      slice_mask, 1U);
				*(__volatile__ u16 *)p458 =
					*(__volatile__ u16 *)p458 + 4U;
				pmu_cal_marker_verify();
				step_diff -= (int)p458[2];
			}
			goto out_0fc8_set3;
		}

		if ((marker_a & 7U) != 4U)
			break;

		if ((cmd_flags & (1U << 9)) != 0U || p458[4] != p458[3] ||
		    pd9c[18] != 0U) {
			pmu_cal_pulse_clear_slot1(pd9c);
			*(__volatile__ u16 *)p458 = marker_a + 4U;
			pmu_cal_marker_status_update();
			continue;
		}

		chk = pmu_check_stage(pulse_type);
		pmu_cal_multi_branch_dispatch(pd9c + 8, cmd_flags, pulse_type,
					      param_code, tap_idx, slice_mask,
					      0U);
		if (v24 == 0U)
			phase_id++;
		*(__volatile__ u16 *)p458 = *(__volatile__ u16 *)p458 + 4U;

		if (chk == 0U) {
			step_diff = (int)phase_id - (int)p458[2];
			if (step_diff >= 1) {
				pmu_cal_marker_status_update();
				pmu_cal_scaled_step_exec(step_diff, cmd_flags);
			}
			if ((cmd_flags & (1U << 11)) != 0U)
				pd9c[18] |= (1U << 0);
			break;
		}

		pmu_cal_marker_status_update();
		pmu_cal_multi_branch_dispatch(pd9c, cmd_flags, pulse_type,
					      param_code, tap_idx, slice_mask,
					      1U);
		*(__volatile__ u16 *)p458 = *(__volatile__ u16 *)p458 + 4U;
		step_diff = (int)phase_id - ((int)p458[2] << 1);
out_0fc8:
		pmu_cal_scaled_step_exec(step_diff, cmd_flags);
		break;
out_0fc8_set3:
		pmu_cal_scaled_step_exec(step_diff, cmd_flags);
out_set3:
		p458[3] = p458[4];
		break;
	}

	pmu_cal_marker_verify_and_buffer_clear(cmd_flags);
	return 0U;
}

/**
 * pmu_cal_pulse_seq_coordinator() - Coordinate rank calibration pulse sequence
 * @ch_idx: Target channel index (0 or 1)
 * @rank: Target rank index
 * @param1: Frequency ratio parameter and tap 14/15 value
 * @flags: Bitmask controlling tap 14 (bit 0) and tap 15 (bit 1) pulses
 */
void pmu_cal_pulse_seq_coordinator(u32 ch_idx, u32 rank, u32 param1, u32 flags)
{
	u32 ratio = pmu_freq_ratio_mult(0xfaU);
	u32 offset;
	u8 cal_cfg10;
	u32 mask1;
	u8 mask;
	u32 b;

	pmu_cal_strobe_pulse_init();

	offset = (ch_idx * 2U + rank) * PMU_SLICE_DESC_SIZE;
	cal_cfg10 = *(__volatile__ const u8 *)
		(PMU_DMEM_CAL_STRUCT_BASE_9A4 + offset + 0x10U);

	mask1 = 1U << rank;
	mask = (u8)((mask1 << 2) | mask1);

	pmu_cal_sequence_pulse_send(0U, 6U, 0x22U, cal_cfg10 | (1U << 6),
				    16U, mask, 0U);

	ratio >>= 1;
	for (b = 0U; b < 2U; b++) {
		if (((flags >> b) & 1U) != 0U) {
			u32 r = (b == 0U) ? (ratio + 8U) : ratio;

			pmu_cal_sequence_pulse_send(0U, 7U, 2U, 0U, 0U, 0U,
						    (u8)r);
			pmu_cal_sequence_pulse_send(0U, 6U, 0x22U, param1,
						    14U + b, mask, 0U);
		}
	}

	pmu_cal_sequence_pulse_send(0U, 7U, 2U, 0U, 0U, 0U, (u8)ratio);
	pmu_cal_sequence_pulse_send(0U, 7U, 2U, 0U, 0U, 0U, 0x28U);
	pmu_cal_slice_step_diff_latch(0U, 1U);
}

/**
 * pmu_phy_ac_lane_timing_delay_set() - Program AC lane timing delay sequence
 * @lane: Target AC lane selector
 *
 * Return: Updated step stride minus 1.
 */
u16 pmu_phy_ac_lane_timing_delay_set(u32 lane)
{
	u32 zero = 0U;
	u32 i;
	u32 stride;

	pmu_set_458_pair((u16)(800U));

	pmu_cal_sequence_pulse_send(0x09U << 19, 5U, 0x0cU, 512U,
				    zero, lane, zero);
	for (i = 0U; i < 3U; i++) {
		pmu_cal_sequence_pulse_send(0U, 6U, 2U, 3U >> i, 0x1aU,
					    lane, zero);
		if (i < 2U)
			pmu_clk_ratio_pulse_trigger();
	}

	pmu_cal_pad_timing_delay_set(0U, 0x22U, 0U);

	pmu_cal_sequence_pulse_send(0U, 5U, 4U, 896U, zero, lane, zero);
	pmu_cal_sequence_pulse_send(0x80U | (1U << 19), 7U, 4U, 0U,
				    zero, zero, zero);

	stride = pmu_cal_stride_get();
	return (u16)(stride - 1U);
}

/**
 * pmu_cal_pulse_burst_send() - Send repeated calibration pulse burst
 * @burst_type: Non-zero for bit 27 command flag, 0 for bit 26 command flag
 * @count: Number of burst pulses to send
 * @slice: Slice selector (bit 1 mapped to bit 6)
 */
void pmu_cal_pulse_burst_send(u32 burst_type, u32 count, u32 slice)
{
	u32 zero = 0U;
	u32 base;
	u32 cmd;
	u32 i;

	if (count == 0U)
		return;

	base = (slice << 5) & 64U;
	cmd = base | ((burst_type != 0U) ? (1U << 27) : (1U << 26));

	for (i = 0U; i < count; i++)
		pmu_cal_sequence_pulse_send(cmd, 7U, 0U, 0U,
					    zero, zero, zero);
}

/**
 * pmu_multiphase_pulse_seq_repeat() - Repeat multiphase pulse sequence
 * @phase_cmd: Pulse opcode
 * @param_val: Parameter code
 * @slice: Slice mask
 * @count: Repeat count
 */
void pmu_multiphase_pulse_seq_repeat(u32 phase_cmd, u32 param_val, u32 slice,
				     u32 count)
{
	u32 zero = 0U;
	__volatile__ u32 saved_param = param_val;
	__volatile__ u32 saved_slice = slice;

	while (count-- != 0U)
		pmu_cal_sequence_pulse_send(0U, phase_cmd, 0U, saved_param,
					    zero, saved_slice, zero);
}

/**
 * pmu_phy_deskew_reset_strobe() - Issue PHY deskew reset strobe across active channels
 */
static void pmu_phy_deskew_reset_strobe(void)
{
	u32 zero = 0U;
	u8 param_40;
	u8 param_25;
	u32 mask;

	param_40 = *(__volatile__ const u8 *)PMU_DMEM_CH1_ACTIVE_FLAG;
	param_25 = *(__volatile__ const u8 *)PMU_DMEM_CH0_ANALOG_EN;
	pmu_cal_strobe_pulse_init();
	mask = (param_40 | param_25) & 3U;
	pmu_cal_sequence_pulse_send(0x80U, 0x0bU, 0x20U, 0U,
				    zero, mask, zero);
	pmu_cal_slice_step_diff_latch(0U, 1U);
	pmu_hw_timer_delay(0x14U);
}

/**
 * pmu_cal_multi_rank_timing_step() - Step multi-rank timing parameters across 21-entry table
 * @rank_mode: Active rank mode mask
 * @steps_table: Pointer to per-step timing table
 * @eval_error_mask: Non-zero to evaluate lane error mask at completion
 * @skip_mask_low: Skip bitmask for indices 0..31
 * @skip_mask_high: Skip bitmask for indices 32..63
 * @check_settle_flags: Non-zero to check settle flags on step 0
 */
noinline void pmu_cal_multi_rank_timing_step(u32 rank_mode, u32 steps_table,
					     u32 eval_error_mask,
					     u32 skip_mask_low,
					     u32 skip_mask_high,
					     u32 check_settle_flags)
{
	u32 zero = 0U;
	const u8 *tbl = (const u8 *)(uintptr_t)steps_table;
	u32 rank_bit = 1U << (1U & ~rank_mode);
	u32 i;

	pmu_cal_strobe_pulse_init();

	for (i = 0U; i < 21U; i++) {
		u8 cal_idx = pmu_sdata()->cal_seq_map[i];
		u32 bit = 1U << (cal_idx & 31U);
		u8 step_val;
		u16 mult = 0U;

		if ((cal_idx < 32U ? (bit & skip_mask_low) :
				     (bit & skip_mask_high)) != 0U)
			continue;

		step_val = tbl[cal_idx];

		if (i == 4U)
			pmu_cal_sequence_pulse_send(zero, 0x10U, 4U, zero,
						    zero, rank_mode, zero);

		pmu_cal_sequence_pulse_send(zero, 6U, zero, step_val,
					    cal_idx, rank_mode, zero);
		pmu_cal_pad_timing_delay_set(zero, 0x22U, zero);

		if (i == 0U) {
			if (check_settle_flags != 0U &&
			    ((tbl[1] & 0x08U) != 0U ||
			     (tbl[0x14] & 0x0fU) != 0U)) {
				pmu_cal_slice_step_diff_latch(zero, 1U);
				pmu_cal_strobe_pulse_init();
				pmu_cal_dbyte_bcast_pulse_width_clear(tbl);
				continue;
			}
			mult = 200U;
		} else {
			if (i == 15U &&
			    ((dmem_read8(PMU_DMEM_CH0_ANALOG_EN_25) |
			      dmem_read8(PMU_DMEM_CH1_ANALOG_EN_40)) &
			     rank_bit) != 0U) {
				pmu_cal_sequence_pulse_send(zero, 6U, zero,
							    step_val | 0x80U,
								cal_idx,
								rank_mode,
								zero);
				pmu_cal_pad_timing_delay_set(zero, 0x22U, zero);
			}
			if ((i | 1U) == 0x11U)
				mult = 250U;
		}

		if (mult != 0U) {
			u8 dram_type = dmem_read8(PMU_DMEM_DRAM_TYPE);
			u16 freq;
			u32 result;

			if (dram_type == 0U)
				dram_type = 2U;
			freq = dmem_read16(PMU_DMEM_DESKEW_LIM_1C & 0xffffU);
			result = ((u32)freq * mult) /
				 ((u32)dram_type * 2000U) + 1U;
			pmu_cal_pad_timing_delay_set(zero, (u16)result, zero);
		}
	}

	pmu_cal_pad_timing_delay_set(zero, 0x41U, zero);
	pmu_cal_slice_step_diff_latch(zero, 1U);

	if (eval_error_mask != 0U)
		pmu_dbyte_lane_error_mask_calc(zero, rank_mode);
}

typedef void (*pmu_step_fn_t)(u32, u32, u32, u32, u32, u32, u32);

#pragma GCC push_options
#pragma GCC optimize("-fno-reorder-blocks")

static __always_inline void
pmu_cal_phase_delay_tracker_slice_update_inline(u32 rank_idx,
						u32 active_csr_offset,
						u32 csr_step_off,
						u32 slice_param_off)
{
	u32 rank_shift = (rank_idx & 0xffU) << 12;
	u32 slice_reg_base = (active_csr_offset | rank_shift) << 1;
	u32 rank_off13 = (rank_idx & 0xffU) << 13;
	__volatile__ u16 *s_base =
		(__volatile__ u16 *)(PHY_REG_DBYTE_BASE + slice_reg_base);
	__volatile__ u16 *r_base =
		(__volatile__ u16 *)(PHY_REG_DBYTE_BASE + rank_off13);
	u16 snap[5];
	u16 val_20158_strobe;
	u16 val_1aa;
	u32 delay_threshold;
	u32 fp_idx = 5U;
	u16 dram_freq;
	u32 p;
	__volatile__ u16 *target_csr_p;
	u16 fp_saved;
	u16 current_csr_val;
	u32 delay_cnt;

	for (p = 0U; p < 5U; p++)
		snap[p] = s_base[0xd0U + p];

	val_20158_strobe = r_base[0xacU];
	r_base[0xacU] = val_20158_strobe | 1U;
	r_base[0xacU] = val_20158_strobe & ~1U;

	val_1aa = r_base[0xd5U];

	delay_threshold = ((u32)val_1aa * 14U) / 100U;
	dram_freq = *(__volatile__ const u16 *)PMU_DMEM_DRAM_FREQ_MHZ;

	for (p = 0U; p < 5U; p++) {
		u16 curr = snap[p];

		if (dram_freq < 3200U) {
			if ((u32)val_1aa < ((u32)curr * 2U)) {
				fp_idx = p;
				break;
			}
		} else {
			u32 idx = (p == 0U) ? 1U : p;
			s32 diff = (s32)snap[idx] - (s32)snap[idx - 1U] -
				   (s32)val_1aa;

			if (((u32)curr * 2U) >= (u32)val_1aa &&
			    diff <= (s32)delay_threshold &&
			    diff >= -(s32)delay_threshold) {
				fp_idx = p;
				break;
			}
		}
	}
	pmu_assert_or_halt(fp_idx != 5U,
			   (dram_freq < 3200U) ? PMU_ERR_ZQ_CAL_LOW_FREQ :
						 PMU_ERR_ZQ_CAL_HIGH_FREQ);

	target_csr_p = (__volatile__ u16 *)
		(PHY_REG_DBYTE_BASE + ((rank_shift | slice_param_off) << 1));
	s_base[0xadU] = (u8)fp_idx;

	fp_saved = *target_csr_p;
	*target_csr_p = fp_saved | (u16)csr_step_off;

	current_csr_val = s_base[0x0fU];
	s_base[0x0fU] = (current_csr_val & ~3U) | 1U;

	delay_cnt = ((s_base[0x14U] << 1) & 510U) * 160U;
	pmu_hw_timer_delay((delay_cnt << 2) + 0x28U);

	*target_csr_p = fp_saved & ~(u16)csr_step_off;
	s_base[0x0fU] = current_csr_val & ~1U;

	val_20158_strobe = r_base[0xacU];
	r_base[0xacU] = val_20158_strobe | 1U;
	r_base[0xacU] = val_20158_strobe & ~1U;

	s_base[0xafU] = r_base[0xd6U];
}

/**
 * pmu_cal_phase_delay_tracker_update() - Backup RFIFO CSRs and update WCK phase delay tracker
 */
static void pmu_cal_phase_delay_tracker_update(void)
{
	u8 flag = *(__volatile__ const u8 *)PMU_DMEM_CLK_GATE_FLAG;
	u16 wck_ctrl;
	u8 freq_mode;
	u32 csr_offset;
	u32 wck_mode_param;
	u32 wck_reg_param;
	u32 slice_off;
	u32 reg_3e1b2;
	u32 reg_3e01e;
	u32 reg_2001e;
	u16 val_2001e;
	u32 csr_base_3c;
	u32 csr_base_34;
	u16 val_3c;
	u16 val_20158;
	u16 val_r13;
	__volatile__ u8 *p_b68;
	u8 start_b6a;
	u8 bound_b6b;
	u8 start_b6c;
	u8 end_b6d;
	u32 pass;
	u32 rank_idx;
	u16 val_20028;
	u32 delay_count;

	if ((flag & 0x20U) != 0U) {
		__volatile__ const u16 *src =
			(__volatile__ const u16 *)PHY_REG_RFIFO_CFG_110;
		__volatile__ u16 *dst =
			(__volatile__ u16 *)PMU_DMEM_RFIFO_BAK_130;
		u32 slice_offset = pmu_sdata()->active_csr_offset;
		__volatile__ const u16 *src_s = (__volatile__ const u16 *)
			(PHY_REG_ZQ_STAT_094 + (slice_offset << 1));

		*dst++ = *src++;
		*dst++ = *src++;
		*dst++ = *src++;
		*dst++ = src[1];
		dst[0] = src_s[0];
		dst[1] = src_s[1];
		dst[2] = src_s[2];
	}

	phy_write16(PHY_REG_MASTER_CAL_CTRL_0DC, 1U);
	phy_write16(PHY_REG_AC_PAT_CTRL_0C0, 0U);
	phy_write16(PHY_REG_AC_CTRL_E022, 0U);

	wck_ctrl = ((flag & 0x80U) != 0U) ? 0U : (flag & 1U);
	*(__volatile__ u16 *)PHY_REG_RFIFO_SUPPRESS_116 = wck_ctrl;
	*(__volatile__ u16 *)PHY_REG_CH0_RANK_CFG_100 =
		*(__volatile__ const u8 *)PMU_DMEM_CH0_ANALOG_EN_25;
	*(__volatile__ u16 *)PHY_REG_CH1_RANK_CFG_102 =
		*(__volatile__ const u8 *)PMU_DMEM_CH1_ANALOG_EN_40;

	if (*(__volatile__ const u8 *)PMU_DMEM_RX_CLK_EN_19 == 0U) {
		u32 active_slices;

		pmu_cal_dbyte_delay_backup_all_ch();
		active_slices = pmu_sdata()->active_slice_cnt;
		if (active_slices != 0U) {
			u32 i;

			for (i = 0U; i < active_slices; i++) {
				__volatile__ u16 *reg = (__volatile__ u16 *)
					(PHY_REG_DBYTE_WCK_DLY_146 +
					 i * PHY_DBYTE_SLICE_STRIDE);

				*reg &= 0x0cU;
			}
		}
	}

	pmu_phy_clk_phase_toggle_pulse();
	pmu_pll_clk_gate_cfg();

	if (*(__volatile__ const u16 *)PMU_DMEM_DRAM_FREQ_MHZ < 1600U)
		goto out;

	if (pmu_is_lpddr5() != 0U)
		goto out;

	freq_mode = pmu_sdata()->freq_mode;
	csr_offset = pmu_sdata()->active_csr_offset;

	if (freq_mode < 2U) {
		wck_mode_param = 2U;
		wck_reg_param = csr_offset | 0x14U;
	} else if (freq_mode < 4U) {
		wck_mode_param = 2U;
		wck_reg_param = 0x56U;
	} else {
		wck_mode_param = 1U;
		wck_reg_param = csr_offset | 87U;
	}

	slice_off = csr_offset << 1;
	reg_3e1b2 = PHY_REG_DBYTE_BCAST_WCK_1B2;
	*(__volatile__ u16 *)reg_3e1b2 = 0x1000U;

	reg_3e01e = PHY_REG_DBYTE_BCAST_GATE_01E | slice_off;
	reg_2001e = PHY_REG_DBYTE_GATE_01E | slice_off;
	val_2001e = *(__volatile__ const u16 *)reg_2001e;

	csr_base_3c = PHY_REG_DBYTE_BASE | (wck_reg_param << 1);
	*(__volatile__ u16 *)reg_3e01e = (val_2001e & ~7U) | 6U;

	csr_base_34 = PHY_REG_DBYTE_BCAST_BASE | (wck_reg_param << 1);
	val_3c = *(__volatile__ const u16 *)csr_base_3c;
	val_3c &= (u16)~wck_mode_param;
	*(__volatile__ u16 *)csr_base_34 = val_3c;

	val_20158 = *(__volatile__ const u16 *)PHY_REG_DBYTE_EYE_CTRL_158;
	*(__volatile__ u16 *)PHY_REG_DBYTE_BCAST_EYE_158 = val_20158 & ~1U;

	val_r13 = *(__volatile__ const u16 *)reg_2001e;
	*(__volatile__ u16 *)reg_3e01e = val_r13 | 5U;
	pmu_hw_timer_delay(0x3000U);

	*(__volatile__ u16 *)reg_3e01e = (val_r13 & ~5U) | 4U;

	p_b68 = (__volatile__ u8 *)PMU_DMEM_CH_RANGE_B68;
	start_b6a = p_b68[2];
	bound_b6b = p_b68[3];
	start_b6c = p_b68[4];
	end_b6d = p_b68[5];

	p_b68[0] = start_b6a;
	p_b68[1] = bound_b6b;

	for (pass = 0U; pass < 2U; pass++) {
		u32 r_start = (pass == 0U) ? 0U : (u32)start_b6c;
		u32 r_end = (pass == 0U) ? (u32)bound_b6b : (u32)end_b6d;

		if (pass == 0U && bound_b6b == 255U)
			continue;
		for (rank_idx = r_start; rank_idx <= r_end; rank_idx++)
			pmu_cal_phase_delay_tracker_slice_update_inline(rank_idx,
									csr_offset,
									wck_mode_param,
									wck_reg_param);
	}

	val_3c = *(__volatile__ const u16 *)csr_base_3c;
	*(__volatile__ u16 *)csr_base_34 = val_3c | (u16)wck_mode_param;

	val_2001e = *(__volatile__ const u16 *)reg_2001e;
	*(__volatile__ u16 *)reg_3e01e = (val_2001e & ~3U) | 1U;

	val_20028 = *(__volatile__ const u16 *)(PHY_REG_DBYTE_DELAY_CNT_028 |
						slice_off);
	delay_count = ((val_20028 << 1) & 510U) * 160U;
	pmu_hw_timer_delay((delay_count << 2) + 0x28U);

	val_2001e &= ~1U;
	val_3c &= (u16)~wck_mode_param;
	*(__volatile__ u16 *)csr_base_34 = val_3c;
	*(__volatile__ u16 *)reg_3e01e = val_2001e;

out:
	pmu_pll_lock_freq_tier_cfg();
}

#pragma GCC pop_options

/**
 * pmu_rank_mask_multiparam_dispatch() - Update shadow fields and dispatch multi-rank timing step
 * @channel: Target channel index (0 or 1)
 * @rank: Target rank index (0 or 1)
 * @shadow_update: Non-zero to save and mask shadow fields, 0 to restore
 */
void pmu_rank_mask_multiparam_dispatch(u32 channel, u32 rank, u32 shadow_update)
{
	u8 *base = (u8 *)PMU_DMEM_CAL_STRUCT_BASE_9A4;

	if (shadow_update != 0U) {
		pmu_cal_eye_table_save(base, 0x12U);
		pmu_cal_lane_mask_or_all((u32)(uintptr_t)base, 0x12U, 0x80U);
	} else {
		pmu_cal_eye_table_restore(base, 0x12U);
	}

	pmu_cal_multi_rank_timing_step((u8)(5U << channel),
				       (u32)(uintptr_t)base +
					   rank * (2U * PMU_SLICE_DESC_SIZE) +
					   channel * PMU_SLICE_DESC_SIZE,
					   0U, 0xfffbffffUL, 0xffffffffUL, 0U);
	pmu_delay_us(20000U, 0U);
}

/**
 * pmu_train_run_dev_init() - Execute device initialization and LPCA pre-init stage
 * @dmem0: Base pointer to PMU DMEM byte array
 * @dmem0_16: Base pointer to PMU DMEM halfword array
 * @seq_ctrl: Active training sequence control bitmask
 * @clk_gate_flag: Clock gating configuration byte from DMEM
 */
static void pmu_train_run_dev_init(__volatile__ u8 *dmem0,
				   __volatile__ u16 *dmem0_16,
				   u16 seq_ctrl, u8 clk_gate_flag)
{
	u32 clk_sync_flag = (clk_gate_flag >> 7) & 1U;
	u32 clamp;
	u32 clamp_limit;

	pmu_phy_pll_lock_init();
	if (!(dmem0[PMU_DMEM_CAL_DISABLE_19] | clk_sync_flag))
		phy_write16(PHY_REG_PHY_STATUS, 2U);

	pmu_cal_stage8_pad_init();
	pmu_phy_pll_clock_divider_align();
	pmu_cal_phase_delay_tracker_update();
	pmu_cal_strobe_pulse_init();
	pmu_cal_pad_timing_delay_set(0U, 4U, 0U);
	pmu_cal_slice_step_diff_latch(0U, 1U);
	pmu_phy_dbyte_broadcast_enable();
	pmu_cal_lane_window_margin_eval(clk_sync_flag);
	dmem0_16[PMU_DMEM_TRAIN_MODE >> 1] = 1U;
	pmu_cal_rx_dqs_delay_center();
	if (clk_sync_flag == 0U)
		pmu_cal_dbyte_impedance_fine_tune();

	if (seq_ctrl & PMU_SEQ_LPCA_INIT) {
		pmu_cal_stage1_pre_init();
		dmem0_16[PMU_DMEM_TRAIN_MODE >> 1] = 0x1000U;
		dmem0_16[PMU_DMEM_TRAIN_MODE >> 1] = 0x9000U;
		pmu_post_cmd_conditional_dispatch(0x0dU);
	}
	if (clk_sync_flag == 0U)
		pmu_multiparam_cal_dispatch();

	pmu_cal_strobe_pulse_init();
	clamp = pmu_cal_delay_line_accum_dispatch(0x3a98U, 0U);
	clamp_limit = ((u8)clamp > 8U) ? (u8)clamp : 8U;
	pmu_cal_sequence_pulse_send(0U, 7U, clamp_limit, 0U, 0U, 0U, 0U);
	pmu_cal_slice_step_diff_latch(0U, 1U);
	pmu_post_cmd_conditional_dispatch(0U);
	pmu_sdata()->cal_active_flag = 0U;
	dmem0_16[PMU_DMEM_TRAIN_MODE >> 1] = 0x8001U;
}

/**
 * pmu_train_run_lane_win_sweep() - Execute lane window sweep and LCDL trace dump
 * @dmem0: Base pointer to PMU DMEM byte array
 * @dmem0_16: Base pointer to PMU DMEM halfword array
 */
static void pmu_train_run_lane_win_sweep(__volatile__ u8 *dmem0,
					 __volatile__ u16 *dmem0_16)
{
	u8 clk_gate;
	u8 cal_ctrl59;
	u32 count_10 = 0U;
	u32 wide_range;

	clk_gate = dmem0[PMU_DMEM_CLK_GATE_FLAG & 0xffffU];
	cal_ctrl59 = dmem0[PMU_DMEM_LANE_SWEEP_CTRL];
	if (cal_ctrl59 >> 6)
		count_10 = (1U << (clk_gate & 3U)) - 1U;

	wide_range = (cal_ctrl59 >> 6) ? 1U : 0U;
	dmem0_16[PMU_DMEM_TRAIN_MODE >> 1] = 1024U;
	pmu_cal_lane_window_sweep_coordinator(count_10, wide_range);
	dmem0_16[PMU_DMEM_TRAIN_MODE >> 1] = 0x8400U;
	pmu_post_cmd_conditional_dispatch(0x0eU);
}

/**
 * pmu_train_run_bist_and_dqs_deskew() - Run BIST search window and DQS deskew sweeps
 * @dmem0: Base pointer to PMU DMEM byte array
 * @seq_ctrl: Active training sequence control bitmask
 */
static void pmu_train_run_bist_and_dqs_deskew(__volatile__ u8 *dmem0,
					      u16 seq_ctrl)
{
	if (seq_ctrl & PMU_SEQ_BIST_SEARCH_WIN) {
		u32 train_step_lim =
			(pmu_dmem_training_flags_eval() == 0U) ? 0xffU : 511U;
		u32 train_lane_mask =
			(dmem0[PMU_DMEM_CLK_GATE_FLAG & 0xffffU] & 4U) ? 0U : 0x0fU;
		u16 val_42a = pmu_sdata()->cal_save_cfg;

		if (val_42a & (1U << 5)) {
			pmu_cal_multi_lane_deskew_sweep(2U, train_lane_mask,
							train_step_lim);
			if (dmem0[PMU_DMEM_TRAIN_PARAM_CTRL] & (1U << 4))
				pmu_cal_multi_lane_deskew_sweep(3U,
								train_lane_mask,
								train_step_lim);
		}
	}

	if (seq_ctrl & PMU_SEQ_STAGE_DQS) {
		u32 train_loop_lim;
		u32 train_step_mask;
		u16 val_42a;

		pmu_cal_zq_pad_calibration(0U);
		train_loop_lim =
			(pmu_dmem_training_flags_eval() == 0U) ? 0xffU : 511U;
		train_step_mask =
			(dmem0[PMU_DMEM_CLK_GATE_FLAG & 0xffffU] & 4U) ? 0U : 0x0fU;
		val_42a = pmu_sdata()->cal_save_cfg;
		if (val_42a & (1U << 6)) {
			pmu_cal_multi_lane_deskew_sweep(1U, train_step_mask,
							train_loop_lim);
			pmu_cal_multi_lane_deskew_sweep(0U, train_step_mask,
							train_loop_lim);
		}
	}
}

/**
 * pmu_sdata_init() - Initialize PMU .sdata scalars and stage sequence map in DCCM
 */
static void pmu_sdata_init(void)
{
	dmem_write8(PMU_DMEM_METRIC_TABLE_ENTRIES, 5U);
	pmu_sdata()->fw_build_id_lo = PMU_FW_BUILD_ID_LO;
	pmu_sdata()->fw_build_id_hi = PMU_FW_BUILD_ID_HI;
	dmem_write8(PMU_DMEM_2D_STEP_PARAM_43A, 4U);
	dmem_write8(PMU_DMEM_CBT_STATE_45A, 4U);
	dmem_write8(PMU_DMEM_CBT_CONFIG_46A, 4U);

	dmem_write32(PMU_DMEM_CAL_CMD_TABLE_46C, 0x180f0e0cUL);
	dmem_write8(PMU_DMEM_CAL_CMD_TABLE_46C + 4U, 0x1eU);

	dmem_write32(PMU_DMEM_CAL_SEQ_MAP_480 + 0x00U, 0x02011210UL);
	dmem_write32(PMU_DMEM_CAL_SEQ_MAP_480 + 0x04U, 0x1c0a1403UL);
	dmem_write32(PMU_DMEM_CAL_SEQ_MAP_480 + 0x08U, 0x0b112e29UL);
	dmem_write32(PMU_DMEM_CAL_SEQ_MAP_480 + 0x0cU, 0x0c19160dUL);
	dmem_write32(PMU_DMEM_CAL_SEQ_MAP_480 + 0x10U, 0x1e180f0eUL);
	dmem_write8(PMU_DMEM_CAL_SEQ_MAP_480 + 0x14U, 0x13U);
}

/**
 * pmu_train_dispatcher() - Dispatch active training stages per sequence_ctrl (0x8ffc)
 */
static void pmu_train_dispatcher(void)
{
	__volatile__ u8 *dmem0 = (__volatile__ u8 *)PMU_DMEM_BASE;
	__volatile__ u16 *dmem0_16;
	__volatile__ u16 *pub_ctl = (__volatile__ u16 *)PHY_REG_PUB_CTL;
	__volatile__ u16 *m_ctl = (__volatile__ u16 *)(PHY_REG_MASTER_BASE | 0x01dcUL);
	__volatile__ u16 *reg14 = (__volatile__ u16 *)(PHY_REG_APB_BASE | 0x120052UL);
	u8 ch_cfg_04;
	u32 ch_offset = 0U;
	u16 seq_ctrl;
	u32 ch_bcast_csr;
	u16 master_phy_status;
	u8 clk_gate_flag;
	u32 denom;
	u32 window;
	u8 train_feature_mask;
	u32 flag_profile3 = 0U;

	__asm__ __volatile__("flag %0" : : "r"(0x6000));
	dmem0_16 = (__volatile__ u16 *)dmem0;
	pmu_sdata_init();

	m_ctl[(0x120 - 0x1dc) >> 1] = 1U;

	if ((dmem0[PMU_DMEM_CLK_GATE_FLAG & 0xffffU] & 1U) != 0U) {
		__volatile__ u16 *p7e =
			(__volatile__ u16 *)PHY_REG_PLL_STATUS;

		*p7e = 0U;
		*(p7e - 0x20000) = 0U;
		m_ctl[(0x034 - 0x1dc) >> 1] = 0U;
		m_ctl[(0x154 - 0x1dc) >> 1] = 0U;
		m_ctl[(0x034 - 0x1dc) >> 1] = 3U;
	}

	pmu_phy_clk_gate_sync_pulse();
	pmu_flag_46b_set();

	pmu_cmd_code_map_log(*pub_ctl, (u8)pmu_sdata()->fw_version_id);

	if (pmu_sdata()->freq_mode != 0U)
		reg14 += 5;

	pmu_sdata()->fw_version_id = PMU_FW_VERSION_ID;
	reg14[0] = PMU_FW_BUILD_ID_LO;
	reg14[1] = PMU_FW_BUILD_ID_HI;

	if (pmu_sdata()->fw_build_id_lo != reg14[0] ||
	    pmu_sdata()->fw_build_id_hi != reg14[1])
		pmu_assert_or_halt(0U, PMU_ERR_FW_BUILD_ID_MISMATCH);

	ch_cfg_04 = dmem0[PMU_DMEM_CHANNEL_CFG];
	if (!(ch_cfg_04 & 0x80U))
		ch_offset = ((u32)ch_cfg_04 << 20) & 0x100000UL;
	pmu_sdata()->active_csr_offset = ch_offset;

	seq_ctrl = dmem0_16[PMU_DMEM_SEQUENCE_CTRL >> 1];
	ch_bcast_csr = PHY_REG_DBYTE_BCAST_MODE | (ch_offset << 1);
	master_phy_status = phy_read16((PHY_REG_MASTER_BASE | 0x0012UL) |
				       (ch_offset << 1));
	clk_gate_flag = dmem0[PMU_DMEM_CLK_GATE_FLAG & 0xffffU];

	denom = (u32)dmem0[PMU_DMEM_DRAM_TYPE] * 220U;
	window = (dmem0_16[PMU_DMEM_DRAM_FREQ_OFF >> 1] + denom - 1U) / denom;

	dmem0[PMU_DMEM_AC_STEP_FLAG] = 1U;
	dmem0[PMU_DMEM_SEARCH_WINDOW_SCALE] = 4U;
	dmem0[PMU_DMEM_SEARCH_WINDOW_STEPS] = (u8)window;

	pmu_cal_search_win_init((u32)(uintptr_t)(dmem0 + PMU_DMEM_SEARCH_WIN_COARSE));
	pmu_cal_search_win_init((u32)(uintptr_t)(dmem0 + PMU_DMEM_SEARCH_WIN_FINE));

	if (seq_ctrl & PMU_SEQ_DEV_INIT)
		pmu_train_run_dev_init(dmem0, dmem0_16, seq_ctrl,
				       clk_gate_flag);

	pmu_cal_pad_impedance_lock();

	if (seq_ctrl & PMU_SEQ_STAGE_VREF)
		pmu_cal_ca_vref_step(1U);
	if (seq_ctrl & PMU_SEQ_STAGE_DCD)
		pmu_cal_ca_train_step();

	train_feature_mask = (u8)dmem0_16[PMU_DMEM_TRAIN_FEATURE_MASK >> 1];
	if (pmu_phy_query_operating_mode() == 3U && (seq_ctrl & PMU_SEQ_ZQ_CAL) &&
	    !(train_feature_mask & 2U)) {
		phy_write16(ch_bcast_csr, 0U);
		flag_profile3 = 1U;
	}

	if (seq_ctrl & PMU_SEQ_STAGE_DQS)
		pmu_cal_zq_pad_calibration(1U);

	if (seq_ctrl & PMU_SEQ_SEARCH_WIN) {
		pmu_cal_search_win_init((u32)(uintptr_t)(dmem0 + PMU_DMEM_SEARCH_WIN_FINE));
		pmu_sdata()->cal_active_flag = 1U;
		dmem0_16[PMU_DMEM_TRAIN_MODE >> 1] = 0x0040U;
		pmu_cal_vref_dac_step_adjust(0x13U);
		dmem0_16[PMU_DMEM_TRAIN_MODE >> 1] = 0x8040U;
		pmu_post_cmd_conditional_dispatch(6U);
		pmu_sdata()->cal_active_flag = 0U;
	}

	train_feature_mask = (u8)dmem0_16[PMU_DMEM_TRAIN_FEATURE_MASK >> 1];
	if (!(train_feature_mask & 1U) && (seq_ctrl & PMU_SEQ_SEARCH_WIN)) {
		if (!(train_feature_mask & 8U) && (seq_ctrl & PMU_SEQ_STAGE_VREF))
			pmu_cal_ca_vref_step(1U);
		if ((seq_ctrl & PMU_SEQ_STAGE_DCD) && !(train_feature_mask & 0x10U))
			pmu_cal_ca_train_step();
		if ((seq_ctrl & PMU_SEQ_STAGE_DQS) && !(train_feature_mask & 0x20U))
			pmu_cal_zq_pad_calibration(1U);
	}

	if (seq_ctrl & PMU_SEQ_ZQ_CAL) {
		if (dmem0[PMU_DMEM_DRAM_TYPE_REG & 0xffffU] & 3U) {
			dmem0_16[PMU_DMEM_TRAIN_MODE >> 1] = 0x0080U;
			pmu_cal_vref_dac_step_adjust(0x11U);
			dmem0_16[PMU_DMEM_TRAIN_MODE >> 1] = 0x8080U;
			pmu_post_cmd_conditional_dispatch(10U);
		}

		if (!(seq_ctrl & PMU_SEQ_DESKEW_SWEEP))
			pmu_sdata()->iter_count = 0U;
	}

	train_feature_mask = (u8)dmem0_16[PMU_DMEM_TRAIN_FEATURE_MASK >> 1];
	if (!(train_feature_mask & 2U) && (seq_ctrl & PMU_SEQ_ZQ_CAL)) {
		if (flag_profile3)
			phy_write16(ch_bcast_csr, 4U);
		if (seq_ctrl & PMU_SEQ_STAGE_DQS)
			pmu_cal_zq_pad_calibration(1U);
	}

	if (seq_ctrl & PMU_SEQ_LANE_WIN_SWEEP)
		pmu_train_run_lane_win_sweep(dmem0, dmem0_16);

	if (seq_ctrl != PMU_SEQ_DEV_INIT)
		pmu_cal_bist_search_win_setup(!(seq_ctrl & PMU_SEQ_BIST_SEARCH_WIN));

	if (seq_ctrl & PMU_SEQ_DESKEW_SWEEP) {
		dmem0_16[PMU_DMEM_TRAIN_MODE >> 1] = 256U;
		pmu_cal_vref_dac_step_adjust(0x12U);
		dmem0_16[PMU_DMEM_TRAIN_MODE >> 1] = 0x8100U;
		pmu_post_cmd_conditional_dispatch(0x0bU);
		pmu_sdata()->iter_count = 0U;

		if (!(dmem0_16[PMU_DMEM_TRAIN_FEATURE_MASK >> 1] & 4U) &&
		    seq_ctrl != PMU_SEQ_DEV_INIT)
			pmu_cal_bist_search_win_setup(!(seq_ctrl & PMU_SEQ_BIST_SEARCH_WIN));
	}

	if (seq_ctrl == PMU_SEQ_DEV_INIT &&
	    dmem0[PMU_DMEM_CAL_DISABLE_19] == 0U)
		pmu_cal_vref_dac_step_adjust(0x0fU);

	pmu_train_run_bist_and_dqs_deskew(dmem0, seq_ctrl);

	if (seq_ctrl & PMU_SEQ_MARGIN_STEP)
		pmu_cal_timing_margin_stage10();

	pmu_cal_stage39_post_results();

	if (seq_ctrl != PMU_SEQ_DEV_INIT) {
		u8 d04_val = dmem0[PMU_DMEM_CHANNEL_CFG];

		if (d04_val & 0x80U)
			pmu_cal_phase_detector_edge_align();
		else if ((d04_val & 0x30U) == 0x20U)
			pmu_cal_dbyte_deskew_fine_tune(0U);
		else if ((d04_val & 0x30U) == 0x10U)
			pmu_cal_dbyte_deskew_fine_tune(1U);
	}

	pmu_phy_dbyte_cfg_unpack();
	phy_write16((PHY_REG_PUB_SEC_BASE | 0x0012UL) | (ch_offset << 1),
		    master_phy_status);
	pmu_mailbox_post_cmd_dispatch(PMU_STATUS_SUCCESS);
	__asm__ __volatile__("mov_s r0, 0\n\tsr r0, [0x21]\n\tbrk_s\n1: b_s 1b\n");
	__builtin_unreachable();
}

/**
 * pmu_cal_multi_rank_step_commit() - Commit multi-rank calibration steps with AC profile setup
 * @rank_mode: Active rank selector bitmask
 * @steps_table: Base address of per-step calibration table
 * @eval_error_mask: Non-zero to evaluate DBYTE lane error mask at completion
 * @skip_mask_low: Skip bitmask for step indices 0..31
 * @skip_mask_high: Skip bitmask for step indices 32..63
 * @pulse_coarse_pre: Non-zero to send coarse pre-pulse before each step
 * @check_settle_flags: Non-zero to check settle flags on step 0
 */
noinline void pmu_cal_multi_rank_step_commit(u32 rank_mode, u32 steps_table,
					     u32 eval_error_mask,
					     u32 skip_mask_low,
					     u32 skip_mask_high,
					     u32 pulse_coarse_pre,
					     u32 check_settle_flags)
{
	__volatile__ const u8 *dmem = (__volatile__ const u8 *)PMU_DMEM_BASE;
	u32 rank_sel = rank_mode;
	u32 rank_bit = 1U << (1U & ~rank_sel);
	u32 i;

	pmu_ac_lane_profile_setup();
	pmu_cal_strobe_pulse_init();
	pmu_cal_pad_timing_delay_set(0U, 5U, 1U);

	for (i = 0U; i < 21U; i++) {
		u8 cal_idx = pmu_sdata()->cal_seq_map[i];
		u32 bit = 1U << (cal_idx & 31U);
		u32 skip = (cal_idx < 32U) ? skip_mask_low : skip_mask_high;
		u8 step_val;

		if ((bit & skip) != 0U)
			continue;

		step_val = ((const u8 *)(uintptr_t)steps_table)[cal_idx];

		if (pulse_coarse_pre != 0U)
			pmu_cal_pad_timing_delay_set(0U, 0x1eU, 1U);

		if (i == 4U) {
			pmu_cal_margin_dir_get();
			pmu_cal_sequence_pulse_send(0U, 0x2cU, 0U, 0U, 0U,
						    rank_sel, 0U);
			pmu_cal_sequence_pulse_send(0U, 0x2dU, 0U, 0U, 0U,
						    rank_sel, 0U);
			pmu_cal_margin_dir_get();
		}

		pmu_dbyte_cal_step_latch(cal_idx, step_val, rank_sel);

		if (i == 0U) {
			if (check_settle_flags != 0U) {
				const u8 *st = (const u8 *)(uintptr_t)steps_table;
				u8 flag_b1 = st[1];
				u8 flag_b14 = st[0x14];

				if ((flag_b1 & 0x08U) != 0U ||
				    (flag_b14 & 0x0fU) != 0U) {
					pmu_cbt_3phase_pulse_seq();
					pmu_cal_slice_step_diff_latch(0U, 1U);
					pmu_cal_strobe_pulse_init();
					pmu_cal_pad_timing_delay_set(0U, 5U, 1U);
					pmu_cal_dbyte_bcast_pulse_width_clear(st);
				}
			}
			pmu_cal_pad_timing_delay_set(0U, 0x18U, 1U);
		} else if (i == 15U &&
			   (dmem[PMU_DMEM_CH_RANGE_B68 & 0xffffU] & rank_bit) != 0U) {
			pmu_cal_margin_dir_get();
			pmu_dbyte_cal_step_latch(cal_idx, step_val | 0x80U,
						 rank_sel);
		}

		pmu_cal_margin_dir_get();
	}

	pmu_cbt_3phase_pulse_seq();
	pmu_cal_slice_step_diff_latch(0U, 1U);

	if (eval_error_mask != 0U)
		pmu_dbyte_lane_error_mask_calc(1U, rank_sel);

	pmu_phy_profile_param_program();
}

/**
 * pmu_multirank_slice_cal_dispatch() - Dispatch calibration steps across enabled channels and ranks
 * @table_ptr: Base pointer to 4-slice calibration table
 * @eval_error_mask: Non-zero to evaluate lane error masks
 * @skip_mask_low: Skip bitmask for step indices 0..31
 * @skip_mask_high: Skip bitmask for step indices 32..63
 * @ch_enable_mask: Active rank bitmask
 * @check_settle_flags: Non-zero to check settle flags on step 0
 */
void pmu_multirank_slice_cal_dispatch(void *table_ptr, u32 eval_error_mask,
				      u32 skip_mask_low, s32 skip_mask_high,
				      u32 ch_enable_mask,
				      u32 check_settle_flags)
{
	u32 rank;
	u32 fp;

	for (rank = 0U; rank < 2U; rank++) {
		u32 bit = 1U << rank;
		u32 rank_mask = 5U << rank;

		for (fp = 0U; fp < 2U; fp++) {
			u32 mask = (fp == 0U) ?
				   dmem_read8(PMU_DMEM_CH0_ANALOG_EN_25) :
				   dmem_read8(PMU_DMEM_CH1_ANALOG_EN_40);

			if (((ch_enable_mask & bit) & mask) != 0U) {
				u32 addr = (u32)(uintptr_t)table_ptr +
					   fp * (2U * PMU_SLICE_DESC_SIZE) +
					   rank * PMU_SLICE_DESC_SIZE;
				pmu_step_fn_t fn = (pmu_step_fn_t)(uintptr_t)
					pmu_cal_multi_rank_timing_step;
				u32 arg5 = check_settle_flags;

				pmu_phy_mode_cfg_dispatch(1U << fp);
				if (dmem_read8(PMU_DMEM_MIN_DELAY_EN_A7C) != 0U &&
				    (dmem_read8(PMU_DMEM_CLK_GATE_FLAG) & (1U << 3)) == 0U) {
					fn = pmu_cal_multi_rank_step_commit;
					arg5 = 0U;
				}
				fn((u8)rank_mask, addr, eval_error_mask,
				   skip_mask_low, (u32)skip_mask_high,
				   arg5, check_settle_flags);
			}
		}
	}

	pmu_phy_mode_cfg_dispatch(3U);
}

/**
 * pmu_cal_multi_param_step_commit() - Commit multi-parameter step for active channel and rank
 * @param_table: Base address of per-step calibration table
 * @step_val: Non-zero to evaluate lane error mask
 * @eval_mask: Skip bitmask for step indices 0..31
 * @filter_mask: Skip bitmask for step indices 32..63
 * @slice_flags: Non-zero to check settle flags on step 0
 */
static void pmu_cal_multi_param_step_commit(u32 param_table, u32 step_val,
					    u32 eval_mask, u32 filter_mask,
				     u32 slice_flags)
{
	pmu_step_fn_t fn = (pmu_step_fn_t)(uintptr_t)
		pmu_cal_multi_rank_timing_step;
	u32 arg5 = slice_flags;

	pmu_phy_mode_cfg_dispatch(3U);
	if (dmem_read8(PMU_DMEM_MIN_DELAY_EN_A7C) != 0U &&
	    (dmem_read8(PMU_DMEM_CLK_GATE_FLAG) & (1U << 3)) == 0U) {
		fn = pmu_cal_multi_rank_step_commit;
		arg5 = 0U;
	}
	fn(0x0fU, param_table, step_val, eval_mask,
	   filter_mask, arg5, slice_flags);
}

/**
 * pmu_cal_stage39_post_results() - Finalize Stage 39 calibration results and lane deskew (0x88b4)
 */
static void pmu_cal_stage39_post_results(void)
{
	u8 *p_buf;
	u8 cal_buf[PMU_SLICE_DESC_TOTAL_BYTES];
	const u8 *cs;
	u8 num_entries;
	const u8 *cmd_table;
	const u8 *dly;
	u32 i;
	u32 k;
	u8 dram_type;
	__volatile__ u16 *dq_dly;

	p_buf = cal_buf;
	pmu_cal_timing_table_matrix_init(p_buf);

	cs = (const u8 *)PMU_DMEM_CAL_STRUCT_BASE_9A4;
	if (((cs[3] | cs[0x16]) & 0x40U) != ((p_buf[3] | p_buf[0x16]) & 0x40U))
		pmu_phy_reg_offset_diff_adjust(0U);

	pmu_cal_lane_mask_and_all((u32)(uintptr_t)p_buf, 0x13U, 0xf3U);
	pmu_cal_lane_mask_or_all((u32)(uintptr_t)p_buf, 0x10U, 0x45U);

	num_entries = dmem_read8(PMU_DMEM_METRIC_TABLE_ENTRIES);
	cmd_table = (const u8 *)pmu_sdata()->cal_cmd_tbl;
	dly = (const u8 *)PMU_DMEM_DESKEW_STAT_TBL_B6E;

	for (i = 0U; i < num_entries; i++) {
		u8 idx = cmd_table[i];

		p_buf[idx] = *dly++;
		p_buf[idx + PMU_SLICE_DESC_SIZE] = *dly++;
		p_buf[idx + (2U * PMU_SLICE_DESC_SIZE)] = *dly++;
		p_buf[idx + (3U * PMU_SLICE_DESC_SIZE)] = *dly++;
	}

	pmu_multirank_slice_cal_dispatch(p_buf, 1U, 0x800U, 0, 0xffU, 0U);

	pmu_cal_lane_mask_and_all((u32)(uintptr_t)p_buf, 0x10U, 0xbcU);
	if (!(p_buf[0x1c] & 0x20U))
		pmu_cal_lane_mask_or_all((u32)(uintptr_t)p_buf, 0x1cU, 2U);

	pmu_multirank_slice_cal_dispatch(p_buf, 1U, 0xfffeffffUL, -1,
					 0xffU, 0U);

	pmu_phy_deskew_reset_strobe();
	pmu_cal_strobe_pulse_init();

	if (!(p_buf[0x1cU] & 0x20U))
		pmu_cal_sequence_pulse_send(0U, 6U, 0x22U,
					    p_buf[0x1cU] | 2U,
					    28U, 15U, 0U);

	for (k = 0U; k < 2U; k++)
		pmu_cal_sequence_pulse_send(0U, k ? 7U : 0x28U,
					    k ? 0x1eU : 2U, 0U, 0U,
					    k ? 0U : 3U, 0U);
	pmu_cal_slice_step_diff_latch(0U, 1U);

	if (dmem_read8(PMU_DMEM_TRAIN_PARAM_CTRL) & 0x10U)
		phy_write16(PHY_REG_DBYTE_BCAST_PULSE |
			    (pmu_sdata()->active_csr_offset << 1), 1U);

	pmu_phy_pll_lock_init();
	phy_write16((PHY_REG_MASTER_BASE | 0x00dcUL), 0U);

	dram_type = dmem_read8(PMU_DMEM_CAL_DISABLE_19);
	if (dram_type == 0U) {
		u8 ch_idx;

		for (ch_idx = 0U; ch_idx < 2U; ch_idx++) {
			bool active = (ch_idx != 0U) ?
				(dmem_read8(PMU_DMEM_CH1_ANALOG_EN_40) != 0U) :
				(dmem_read8(PMU_DMEM_CH0_ANALOG_EN_25) != 0U);

			if (active) {
				pmu_phy_mode_cfg_dispatch(ch_idx + 1U);
				pmu_cal_lane_deskew_sweep_exec(ch_idx);
			}
		}
	}

	pmu_phy_mode_cfg_dispatch(3U);
	dmem_write8(PMU_DMEM_CAL_BYTE, 0U);

	if (dram_type == 0U)
		pmu_phy_cal_strobe_latch_setup();

	dq_dly = (__volatile__ u16 *)PHY_REG_DBYTE_BCAST_DQ_DLY2;
	dq_dly[0] = 0U;
	dq_dly[1] = 0U;

	if (dmem_read8(PMU_DMEM_CAL_DISABLE_19) == 0U &&
	    dmem_read16(PMU_DMEM_DESKEW_LIM_1C & 0xffffU) != 1U)
		pmu_cal_vref_search_center_update();

	pmu_cal_state_restore_offset_prog();
}

/**
 * pmu_cal_vref_dac_step_adjust() - Dispatch per-stage Vref DAC and delay step adjustments (0xbea0)
 * @stage_step: Stage step selector (0..19)
 */
void pmu_cal_vref_dac_step_adjust(u32 stage_step)
{
	u32 vref_dac_mode = ((stage_step | 2U) != 6U) ? 1U : 0U;
	u32 fp = 0x0fU;
	u8 d09_low3;
	u8 ch_mask;
	u32 channel;

	if (dmem_read8(PMU_DMEM_CLK_GATE_FLAG) & (1U << 2)) {
		u32 mode = pmu_phy_query_operating_mode();

		if ((stage_step & ~1U) == 4U && mode == 3U)
			fp = 5U;
		else
			fp = 0U;
	}

	d09_low3 = dmem_read8(PMU_DMEM_DRAM_TYPE_FLAGS) & 7U;
	if (d09_low3 != 0U && stage_step < 0x13U &&
	    (0x601f0UL & (1UL << stage_step)))
		fp = (1U << d09_low3) - 1U;

	ch_mask = dmem_read8(PMU_DMEM_CH0_RANK_EN) |
		  dmem_read8(PMU_DMEM_CH1_RANK_EN);

	for (channel = 0U; channel < 2U; channel++) {
		u32 vref_accum_step = 0U;
		u32 vref_target_step = 0U;
		u32 rank;

		(void)vref_target_step;
		if (!(ch_mask & (1U << channel)))
			continue;

		dmem_write8(PMU_DMEM_CAL_BYTE, (u8)channel);

		for (rank = 0U; rank < 2U; rank++) {
			u32 rank_vref_status = 0U;
			u8 vref_eval_buf[80];

			if (stage_step < 0x13U &&
			    (0x701f8UL & (1UL << stage_step))) {
				u8 cal_param72;
				u8 cal_flag62;

				vref_accum_step = pmu_dmem_training_flags_eval();
				rank_vref_status = vref_accum_step;
				cal_param72 = dmem_read8(PMU_DMEM_VREF_DAC_CTRL);
				cal_flag62 = dmem_read8(PMU_DMEM_VREF_FLAG_SIGNED);
				vref_target_step =
					(!(cal_param72 & 0x20U) ? 1U : 0U) |
					(cal_flag62 >> 7);
			}

			if (!pmu_channel_rank_avail_check(rank, channel))
				continue;

			if ((u32)(stage_step - 4U) < 5U) {
				if (channel != 0U || rank != 0U)
					continue;
			}

			pmu_pulse_phy_146();
			memset(vref_eval_buf, 0, sizeof(vref_eval_buf));

			switch (stage_step) {
			case 0U:
				pmu_cal_timing_margin_envelope_scan(channel,
								    rank,
								    vref_eval_buf);
				pmu_cal_search_win_max_calc(channel, rank, 1U);
				break;
			case 1U:
				pmu_cal_rank_deskew_matrix_commit(channel,
								  rank,
								  vref_eval_buf);
				break;
			case 4U:
			case 5U:
			case 6U:
			case 7U: {
				u32 commit_param =
					(rank_vref_status == 0U) ? 0xffU : 511U;
				u32 mode_cfg = 0U;

				if ((stage_step & ~1U) == 6U) {
					if (dmem_read8(PMU_DMEM_BIST_PATTERN_LO) != 0U ||
					    dmem_read8(PMU_DMEM_BIST_PATTERN_HI) != 0U ||
					    dmem_read8(PMU_DMEM_DESKEW_OFFSET_OVERRIDE) != 0U)
						mode_cfg = 7U;
					else
						mode_cfg = 4U;
				}
				pmu_cal_rank_param_commit(fp, mode_cfg,
							  commit_param,
							  vref_dac_mode);
				break;
			}
			case 8U: {
				u8 cal_flag62 = dmem_read8(PMU_DMEM_VREF_FLAG_SIGNED);
				u8 cal_param72 = dmem_read8(PMU_DMEM_VREF_DAC_CTRL);
				u8 train_cfg96 = dmem_read8(PMU_DMEM_TRAIN_PARAM_CTRL);
				u32 cond = ((cal_flag62 >> 6) & 1U) |
					   ((cal_flag62 >> 7) & 1U) |
					   ((train_cfg96 >> 6) & 1U) |
					   (!(cal_param72 & 0x20U) ? 1U : 0U);
				u32 step_limit = (cond == 0U) ? 0xffU : 511U;
				u32 commit_param =
					((train_cfg96 & 0x10U) << 5) | step_limit;

				pmu_cal_rank_param_commit(fp, 0U, commit_param,
							  2U);
				break;
			}
			case 15U: {
				u16 vref_steps_buf[4];

				pmu_cal_slice_step_diff_commit(channel,
							       vref_steps_buf,
							       1U);
				break;
			}
			case 16U:
				pmu_cal_slice_deskew_result_commit(channel,
								   vref_eval_buf,
								   rank_vref_status);
				pmu_cal_search_win_max_calc(channel, rank, 0U);
				break;
			case 17U:
				pmu_cal_rank_rx_en_delay_commit(channel,
								(u16)fp,
								rank_vref_status,
								0x11U);
				break;
			case 18U:
				pmu_cal_rank_rx_en_delay_commit(channel,
								(u16)fp,
								rank_vref_status,
								0x12U);
				break;
			case 19U:
				pmu_cal_wck_ck_align_pulse_send(channel, rank);
				break;
			default:
				break;
			}
		}

		if ((u32)(stage_step - 0x11U) < 2U)
			dmem_write16(PMU_DMEM_ITER_COUNT,
				     dmem_read16(PMU_DMEM_ITER_COUNT) + 1U);

		pmu_phy_mode_cfg_dispatch(3U);
	}

	if (dmem_read8(PMU_DMEM_CLK_GATE_FLAG) & (1U << 6))
		pmu_slice_deskew_pulse_seq(stage_step);

	if ((u32)(stage_step - 0x11U) < 2U) {
		if (dmem_read8(PMU_DMEM_CLK_GATE_FLAG) & (1U << 6))
			pmu_slice_metric_scan_and_program();
	}

	dmem_write8(PMU_DMEM_CAL_CTRL_0F, 0U);
}

/**
 * pmu_multiparam_cal_dispatch() - Dispatch multi-parameter calibration structure updates (0xe9d4)
 */
static void pmu_multiparam_cal_dispatch(void)
{
	uintptr_t cal_addr =
		(PMU_DMEM_BASE | PMU_DMEM_CAL_STRUCT_BASE);
	const u8 *param =
		(const u8 *)(PMU_DMEM_BASE | PMU_DMEM_SLICE_PROFILE_TABLE);
	u8 *cal = (u8 *)cal_addr;

	pmu_cal_lane_mask_or_all(cal_addr, 1U, param[0] & 8U);
	pmu_cal_lane_mask_and_all(cal_addr, 0x14U, 0U);

	pmu_cal_lane_mask_or_all(cal_addr, 0x14U, param[52] & 0x0fU);

	if ((dmem_read8(PMU_DMEM_CAL_MODE_1B) & 1U) &&
	    (*(const __volatile__ u8 *)(PMU_DMEM_BASE | PMU_DMEM_LANE_ACTIVE_FLAGS) != 0U)) {
		const u8 *src =
			(const u8 *)(PMU_DMEM_BASE | PMU_DMEM_DELAY_CAL_RESULTS);
		u8 s0;
		u8 s1;
		u8 s2;
		u8 s3;

		s0 = src[0];
		s1 = src[1];
		s2 = src[2];
		s3 = src[3];
		cal[0x78] = s2;
		cal[0x0c] = s0;
		cal[0x42] = s1;
		cal[0xae] = s3;
	}

	pmu_cal_lane_mask_and_all(cal_addr, 0x10U, 0xfbU);
	pmu_cal_lane_mask_or_all(cal_addr, 0x10U, 0x41U);

	pmu_multirank_slice_cal_dispatch((void *)cal_addr, 0U, 0U, 0,
					 0xffU, 0U);

	pmu_cal_lane_mask_or_all(cal_addr, 0x10U, 4U);
	pmu_cal_multi_param_step_commit(cal_addr, 0U, 0xfffeffffUL,
					0xffffffffUL, 1U);
}

/**
 * pmu_main_wrapper() - Dispatch top-level PMU training sequencer
 *
 * Return: Always 0.
 */
int pmu_main_wrapper(void)
{
	pmu_train_dispatcher();
	return 0;
}

/**
 * pmu_stage1_build_swizzle_map() - Populate AC pin swizzle lookup map
 * @swizzle_map: Output 14-byte pin-to-lane swizzle array
 * @ch1_rank: Non-zero if Channel 1 is active
 */
static void pmu_stage1_build_swizzle_map(u8 *swizzle_map, u8 ch1_rank)
{
	u32 ca;

	for (ca = 0; ca < 7; ca++) {
		u16 pin = phy_read16(PHY_AC_SWIZZLE_S0_BASE + (ca << 1));

		if (pin < 7)
			swizzle_map[pin] = (u8)ca;
	}
	if (ch1_rank != 0) {
		for (ca = 0; ca < 7; ca++) {
			u16 pin = phy_read16(PHY_AC_SWIZZLE_S1_BASE +
					     (ca << 1));

			if (pin < 7)
				swizzle_map[pin + 7] = (u8)ca;
		}
	}
}

/**
 * pmu_stage1_poll_rx_fifo_ranks() - Execute dual-scan multi-rank RX FIFO polling
 * @ch0_rank: Active rank mask for Channel 0
 * @ch1_rank: Active rank mask for Channel 1
 * @initial_step: Output 4-element signed initial delay step array
 * @scan_matrix: Output 3584-byte eye scan matrix
 *
 * Return: Updated active slice count (1 or 2).
 */
static u32 pmu_stage1_poll_rx_fifo_ranks(u8 ch0_rank, u8 ch1_rank,
					 s8 *initial_step, u8 *scan_matrix)
{
	u32 num_slices = (ch1_rank != 0) ? 2 : 1;
	u32 idx;

	for (idx = 0; idx < 4; idx++) {
		u16 out_cfg;
		u8 out_mode_sel;
		u8 out_num_slices;
		u32 csr_offset;
		u32 reg_base;
		u8 channel;
		u8 rank;
		s8 ret;

		if (pmu_cal_channel_rank_config_get(idx, &out_cfg,
						    &out_mode_sel,
						    &out_num_slices) != 0)
			continue;

		num_slices = (u32)out_num_slices;
		dmem_write8(PMU_DMEM_AC_CAL_ACTIVE, 1);
		pmu_phy_mode_cfg_dispatch(num_slices);

		/* Invariant §9.144-B: Re-assert both analog receiver power rails */
		phy_write16(PHY_REG_CH0_ANALOG_PWR, (u16)ch0_rank);
		phy_write16(PHY_REG_CH1_ANALOG_PWR, (u16)ch1_rank);

		channel = out_num_slices - 1U;
		rank = dmem_read8(PMU_DMEM_ACTIVE_RANK);
		csr_offset = dmem_read32(PMU_DMEM_ACTIVE_CH_OFFSET);
		reg_base = (((u32)channel << 12) | csr_offset) << 1;

		phy_write16(PHY_AC_GATE_CTRL_RESET | reg_base, 0);
		phy_write16(PHY_AC_BASE_OFFSET_CTRL | reg_base, 0);

		/* Primary Scan: enable_scan = 1 */
		ret = pmu_cal_dbyte_rx_fifo_reset_poll(channel,
						       (u32)out_mode_sel, 1,
						       scan_matrix);
		initial_step[((u32)rank << 1) + (u32)out_cfg] = ret;

		/* Baseline Scan: enable_scan = 0, first arg is out_mode_sel */
		(void)pmu_cal_dbyte_rx_fifo_reset_poll(channel,
						       (u32)out_mode_sel, 0,
						       scan_matrix);
	}

	return num_slices;
}

/**
 * pmu_stage1_search_and_commit_slice() - Find 256-tap eye centers and commit AC slice delays
 * @s: Slice index (0 or 1)
 * @step: Normalized step stride (1, 2, 4, or 8)
 * @swizzle_map: 14-byte pin-to-lane swizzle lookup table
 * @initial_step: 4-element signed initial delay step array
 * @scan_matrix: 3584-byte eye scan matrix
 */
static void pmu_stage1_search_and_commit_slice(u32 s, u8 step,
					       const u8 *swizzle_map,
					       const s8 *initial_step,
					       const u8 *scan_matrix)
{
	u16 center_delay[7];
	s32 base_delay = (dmem_read8(PMU_DMEM_CAL_STEP_CONFIG) &
			  BIT(3)) ? 0 : 0x40;
	s32 lower_bound = base_delay - 128;
	s32 upper_bound = base_delay + 128;
	u32 csr_offset;
	u32 s_base;
	s32 dly0, dly1;
	u32 ca;

	for (ca = 0; ca < 7; ca++) {
		u32 best_start = 0;
		u32 best_len = 0;
		u32 run_start = 0;
		u32 cur_len = 0;
		u32 in_run = 0;
		u32 delay;

		for (delay = 0; delay < 256; delay += step) {
			u32 mat_idx = s * 1792 + ca * 256 + delay;

			if (scan_matrix[mat_idx] == 0) {
				if (!in_run) {
					run_start = delay;
					cur_len = 0;
					in_run = 1;
				}
				cur_len += step;
			} else if (in_run) {
				if (cur_len > best_len && delay >= 129) {
					best_start = run_start;
					best_len = cur_len + step;
				}
				in_run = 0;
				cur_len = 0;
			}
		}

		if (in_run &&
		    (cur_len > best_len || (best_len == 0 && cur_len != 0))) {
			best_start = run_start;
			best_len = cur_len + step;
		}

		if (best_len == 0) {
			/* Invariant §9.148-A: Nominal midpoint fallback (0x40) */
			center_delay[ca] = 0x40;
		} else {
			s32 center = (s32)best_start +
				     (s32)((best_len & ~1U) / 2) +
				     lower_bound;

			if (center < 0)
				center = 0;
			if (center > upper_bound)
				center = upper_bound;
			center_delay[ca] = (u16)center;
		}
	}

	/* Commit Calibrated Delays to Physical AC Slice */
	csr_offset = dmem_read32(PMU_DMEM_ACTIVE_CH_OFFSET);
	for (ca = 0; ca < 7; ca++) {
		u8 mapped_lane = swizzle_map[s * 7 + ca];
		u32 addr = PHY_AC_PIN_DELAY_BASE |
			   ((csr_offset | (s << 12) |
			     ((u32)mapped_lane << 8)) << 1);

		phy_write16(addr, center_delay[ca]);
	}

	csr_offset = dmem_read32(PMU_DMEM_ACTIVE_CH_OFFSET);
	s_base = (csr_offset | (s << 12)) << 1;
	dly0 = base_delay + (s32)initial_step[s * 2 + 0];
	dly1 = base_delay + (s32)initial_step[s * 2 + 1];
	phy_write16(PHY_AC_MASTER_DESKEW0 | s_base,
		    (dly0 < 0) ? 0 : (u16)dly0);
	phy_write16(PHY_AC_MASTER_DESKEW1 | s_base,
		    (dly1 < 0) ? 0 : (u16)dly1);
	phy_write16(PHY_AC_BASE_OFFSET_CTRL | s_base, (u16)base_delay);
}

/**
 * pmu_cal_stage1_pre_init() - Stage 1 Command Bus Training / CA Pre-Initialization
 *
 * Orchestrates initial Command-Address (CA) timing calibration across active
 * DRAM channels, ranks, and Address/Command (AC) slices per SPEC-142 v1.2.0:
 *
 * 1. Checks active channel rank flags (`0x80000025` and `0x80000040`), verifies
 *    the pre-condition clock gating flag (`0x80000001`), powers both Channel 0
 *    and Channel 1 analog receiver rails (`0x900400e4` and `0x900400e6`,
 *    Invariant §9.144-B), enables AC macro pad drivers (`0x900e00ca = 0x01ff`,
 *    Invariant §9.124), asserts Master CKE (`0x9004000e |= 0x0100`, Invariant
 *    §9.124), and enables DBYTE broadcast calibration strobe gating.
 * 2. Backs up the 6-byte AC PHY descriptor block from DCCM `0x8000099c`,
 *    enforcing the Cold-Boot Zero-Token APB Bus Guard (`0x0001008bUL` fallback
 *    when `token == 0`, Invariant §9.86-A / §9.125), and streams it to PHY
 *    registers.
 * 3. Normalizes the 4-bit calibration step configuration (`0x8000041a`) into a
 *    one-hot power-of-2 stride (`1`, `2`, `4`, or `8`) at `0x8000047c` and
 *    zero-initializes the 3,584-byte raw eye scan matrix.
 * 4. Remaps calibration structure masks and reads hardware AC pin swizzle
 *    registers (`0x90060120` and conditionally `0x90062120`, with strict `< 7`
 *    pin bounds on both slices) into `swizzle_map[14]`.
 * 5. Iterates across ranks `0..3` using a 16-bit halfword `out_cfg` output
 *    pointer, re-asserts analog receiver power rails after
 *    `pmu_phy_mode_cfg_dispatch`, resets AC slice timing registers, and
 *    executes dual-phase 3-argument RX FIFO reset polling to populate
 *    `initial_step[4]` and `scan_matrix[3584]`.
 * 6. Clears AC profile lane registers (`0x9007e002` with `ca << 8` stride) and
 *    master register (`0x9007e004`), then pulses the primary calibration strobe.
 * 7. Executes a 256-tap contiguous zero-run eye window search (`delay >= 129`
 *    threshold, nominal midpoint fallback `0x40` per Invariant §9.148-A) and
 *    programs per-lane AC delays and master deskew registers.
 * 8. Pulses primary and secondary calibration strobes, restores the 6-byte AC
 *    PHY descriptor backup, disables DBYTE broadcast strobe gating, and
 *    re-asserts Master CKE (`0x9004000e |= 0x0100`).
 */
__section(".text.pmu_cal_stage1_pre_init")
static void pmu_cal_stage1_pre_init(void)
{
	u8 swizzle_map[14] = { 0 };
	s8 initial_step[4] = { 0 };
	u8 descriptor_backup[6];
	u8 scan_matrix[3584];
	u8 ch0_rank;
	u8 ch1_rank;
	u8 raw_step;
	u32 num_slices;
	u32 ch_offset;
	u8 step;
	u32 ca;
	u32 s;

	ch0_rank = dmem_read8(PMU_DMEM_CH0_ANALOG_EN);
	ch1_rank = dmem_read8(PMU_DMEM_CH1_ACTIVE_FLAG);
	if (ch0_rank == 0 && ch1_rank == 0)
		return;

	/* Pre-condition clock gate check */
	if ((dmem_read8(PMU_DMEM_CLK_GATE_FLAG) & BIT(3)) != 0) {
		pmu_assert_or_halt(0, PMU_ERR_LPCA_CLK_GATE_PRECOND);
		return;
	}

	/* Invariant §9.144-B: Retain analog receiver power rails continuously */
	phy_write16(PHY_REG_CH0_ANALOG_PWR, (u16)ch0_rank);
	phy_write16(PHY_REG_CH1_ANALOG_PWR, (u16)ch1_rank);

	/* Invariant §9.124: Activate AC macro pad drivers & assert Master CKE */
	phy_write16(PHY_REG_AC_TX_EN_BCAST, 0x01ff);
	phy_write16(PHY_REG_MASTER_CFG,
		    phy_read16(PHY_REG_MASTER_CFG) | 0x0100);

	/* Enable DBYTE broadcast strobe gating */
	pmu_dbyte_cal_strobe_pulse_seq(1, 2047, 0);

	/* Save 6-byte AC PHY register 0x0001008b descriptor */
	memset(descriptor_backup, 0, sizeof(descriptor_backup));
	descriptor_backup[0] = 0x8bU;
	descriptor_backup[2] = 0x01U;
	pmu_phy_reg_buffer_stream(descriptor_backup, 1, 1);

	/* One-hot power-of-2 step stride normalization */
	raw_step = dmem_read8(PMU_DMEM_STEP_CFG_RAW) >> 4;
	if (raw_step & 8U)
		step = 8U;
	else if (raw_step & 4U)
		step = 4U;
	else if (raw_step & 2U)
		step = 2U;
	else
		step = 1U;
	dmem_write8(PMU_DMEM_NORM_STEP_STRIDE, step);

	memset(scan_matrix, 0, sizeof(scan_matrix));
	pmu_cal_struct_mask_remap();

	/* Phase 2-3: Build swizzle map and run dual-scan RX FIFO polling */
	pmu_stage1_build_swizzle_map(swizzle_map, ch1_rank);
	num_slices = pmu_stage1_poll_rx_fifo_ranks(ch0_rank, ch1_rank,
						   initial_step, scan_matrix);

	/* Phase 4: Clear AC Profile Registers with ca << 8 stride */
	ch_offset = dmem_read32(PMU_DMEM_ACTIVE_CH_OFFSET);
	for (ca = 0; ca < 7; ca++) {
		u32 addr = PHY_AC_PROFILE_LANE_BASE |
			   ((ch_offset | (ca << 8)) << 1);

		phy_write16(addr, 0);
	}
	phy_write16(PHY_AC_PROFILE_MASTER_CTRL | (ch_offset << 1), 0);
	pmu_pulse_phy_146();

	/* Phase 5: 256-Tap Eye Margin Search with delay >= 129 Bias */
	for (s = 0; s < num_slices; s++)
		pmu_stage1_search_and_commit_slice(s, step, swizzle_map,
						   initial_step, scan_matrix);

	/* Phase 6: Epilogue, Strobes, Descriptor Restore & Master CKE Re-Assertion */
	pmu_pulse_phy_146();
	pmu_pulse_phy_140();
	pmu_phy_reg_buffer_stream(descriptor_backup, 1, 0);
	pmu_dbyte_cal_strobe_pulse_seq(0, 0, 0);

	/* Re-assert Master CKE */
	phy_write16(PHY_REG_MASTER_CFG,
		    phy_read16(PHY_REG_MASTER_CFG) | 0x0100);
}

/**
 * pmu_cal_ca_vref_step() - Execute Stage 2 Command/Address Vref adjustment step
 * @arg0: Pass selector (0 for initial pass, non-zero for secondary pass)
 */
static void pmu_cal_ca_vref_step(u32 arg0)
{
	u32 step;
	u32 post_cmd;

	step = (arg0 == 0) ? 2 : 1;
	*(__volatile__ u16 *)PMU_DMEM_TRAIN_MODE_14 = 2;
	pmu_cal_vref_dac_step_adjust(step);
	*(__volatile__ u16 *)PMU_DMEM_TRAIN_MODE_14 = 0x8002;

	post_cmd = (arg0 != 0) ? 1 : 0xfe;
	pmu_post_cmd_conditional_dispatch(post_cmd);
}

/**
 * pmu_cal_ca_train_step() - Execute Stage 3 Command/Address training step
 *
 * Selects CA Vref DAC adjustment mode and post-command dispatch index based
 * on protocol bits [1:0] at DCCM 0x8000008e, bracketed by Stage 3 mode
 * assertions (0x0004 -> 0x8004) at DCCM 0x80000014.
 */
static void pmu_cal_ca_train_step(void)
{
	u8 flag = *(__volatile__ u8 *)PMU_DMEM_DRAM_TYPE_REG;
	u32 post_cmd;

	*(__volatile__ u16 *)PMU_DMEM_TRAIN_MODE_14 = 4;

	if ((flag & 3) != 0) {
		pmu_cal_vref_dac_step_adjust(0);
		post_cmd = 2;
	} else {
		pmu_cal_vref_dac_step_adjust(0x10);
		post_cmd = 5;
	}

	pmu_post_cmd_conditional_dispatch(post_cmd);
	*(__volatile__ u16 *)PMU_DMEM_TRAIN_MODE_14 = 0x8004;
}

/**
 * pmu_cal_zq_pad_calibration() - Execute Stage 4 Master PUB ZQ pad calibration
 * @is_primary: Non-zero for primary pass, 0 for secondary post-deskew pass
 *
 * Return: Stage 4 completion mode word (0x8008).
 */
static u32 pmu_cal_zq_pad_calibration(u32 is_primary)
{
	dmem_write16(PMU_DMEM_TRAIN_MODE, 8U);

	if (dmem_read16(PMU_DMEM_DRAM_FREQ_OFF) > 0xc7fU && !is_primary) {
		pmu_sdata()->high_freq_flag = 1;
		if ((dmem_read8(PMU_DMEM_CAL_MISC_FLAGS) & 0x20U) == 0U)
			pmu_sdata()->high_freq_flag = 0;
	}

	pmu_sdata()->cal_active_flag = !is_primary;

	if (is_primary) {
		if (!pmu_check_dmem_407_flag())
			pmu_cal_vref_dac_step_adjust(7);
		pmu_cal_vref_dac_step_adjust(6);
		pmu_post_cmd_conditional_dispatch(0xfd);
	} else {
		pmu_cal_vref_dac_step_adjust(5);
		pmu_cal_vref_dac_step_adjust(4);
		pmu_post_cmd_conditional_dispatch(3);
	}

	pmu_sdata()->cal_active_flag = 0;
	dmem_write16(PMU_DMEM_TRAIN_MODE, 0x8008U);
	return 0x8008;
}

/**
 * pmu_cal_timing_margin_stage10() - Execute Stage 10 timing margin evaluation
 *
 * Configures PHY mode 3, updates PMU_DMEM_TRAIN_MODE (0x0200 -> 0x8200),
 * clears PMU_DMEM_MARGIN_STAT_405, and dispatches post-command 9.
 */
static void pmu_cal_timing_margin_stage10(void)
{
	pmu_phy_mode_cfg_dispatch(3);
	*(__volatile__ u16 *)PMU_DMEM_TRAIN_MODE_14 = 0x0200;
	*(__volatile__ u16 *)PMU_DMEM_TRAIN_MODE_14 = 0x8200;
	*(__volatile__ u8 *)PMU_DMEM_MARGIN_STAT_405 = 0;
	pmu_post_cmd_conditional_dispatch(9);
}
