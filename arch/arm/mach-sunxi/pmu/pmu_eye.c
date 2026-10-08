// SPDX-License-Identifier: GPL-2.0+
/*
 * Synopsys DesignWare DDR PHY Training Firmware (LPDDR5)
 * 2D eye diagram margin searching, boundary detection, and centroid alignment
 * Target Microcontroller: Synopsys ARC EM4 (ARCv2 ISA, Code Density enabled)
 * SoC: Allwinner A733 (Sun60i) / LPDDR5 PHY (Type 9)
 */

#include "lpddr5_pmu_internal.h"

/**
 * pmu_slice_margin_window_clamp() - Clamp slice margin search window and calculate error masks
 */
void pmu_slice_margin_window_clamp(void)
{
	pmu_phy_mode_cfg_dispatch(3);
	u8 a7c = dmem_read8(PMU_DMEM_AC_STEP_FLAG);
	u32 mode = 0;
	if (a7c != 0) {
		u8 clk_gate_flag = dmem_read8(PMU_DMEM_CLK_GATE_FLAG);
		if ((clk_gate_flag & (1 << 3)) == 0) {
			pmu_ac_lane_profile_setup();
			mode = 1;
		}
	}
	pmu_dbyte_lane_error_mask_calc(mode, 0xf);
}

/**
 * pmu_cal_margin_window_eval() - Evaluate eye margin window boundaries for calibration structure
 * @struct_ptr: Pointer to calibration structure in DMEM
 *
 * Return: Status code (0 on success, non-zero on boundary error).
 */
u32 pmu_cal_margin_window_eval(u8 *struct_ptr)
{
	s8 shift_cfg = (s8)dmem_read8(PMU_DMEM_CAL_SHIFT_CFG);
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
 * pmu_cal_rank_margin_pair_eval() - Evaluate eye margin window pairs between rank structures
 * @struct_a: Pointer to primary rank calibration structure
 * @struct_b: Pointer to secondary rank calibration structure
 *
 * Return: Status code (0 on success, non-zero on boundary error).
 */
u32 pmu_cal_rank_margin_pair_eval(u8 *struct_a, u8 *struct_b)
{
	s8 shift_cfg = (s8)dmem_read8(PMU_DMEM_CAL_SHIFT_CFG);
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

/**
 * pmu_eye_error_metric_eval() - Evaluate weighted Euclidean error metric for eye diagram sample point
 * @point_xy: Packed 16-bit (X, Y) coordinate pair
 * @ref_y: Reference Y coordinate (nominal midpoint)
 * @min_x: Left eye margin boundary X coordinate
 * @max_x: Right eye margin boundary X coordinate
 * @out_metric: Output pointer for computed 8-bit error metric
 *
 * Return: Weighted Euclidean eye error metric.
 */
u32 pmu_eye_error_metric_eval(u32 point_xy, u32 ref_y, u32 min_x, u32 max_x, u8 *out_metric)
{
	s32 diff_y = (s32)(s8)(point_xy & 0xff) - (s32)ref_y;
	u32 x = (point_xy >> 8) & 0xff;
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

	u16 weight_y = *(const u16 *)(PMU_DMEM_BASE | PMU_DMEM_METRIC_COEFF_Y);
	u16 weight_x = *(const u16 *)(PMU_DMEM_BASE | PMU_DMEM_METRIC_COEFF_X);
	return (u32)(diff_y * diff_y) * weight_y + (u32)(diff_x * diff_x) * weight_x;
}

/**
 * pmu_eye_margin_boundary_detect() - Detect left and right eye margin boundaries from sample table
 * @out_start: Output pointer receiving starting boundary delay
 * @out_end: Output pointer receiving ending boundary delay
 * @samples: Pointer to eye diagram sample array
 *
 * Return: Search status code (0 on success, non-zero error).
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
 * pmu_rank_slice_margin_eval() - Evaluate rank slice margin apertures and program delay CSRs
 */
void pmu_rank_slice_margin_eval(void)
{
	u32 mode = pmu_cal_profile_mode_get();

	for (u32 rank = 0; rank < 2; rank++) {
		u8 ch0_ranks = dmem_read8(PMU_DMEM_CH0_RANK_EN);
		u8 ch1_ranks = dmem_read8(PMU_DMEM_CH1_RANK_EN);
		if (!((ch1_ranks | ch0_ranks) & (1 << rank)))
			continue;

		u8 start_slice = *(const volatile u8 *)(PMU_DMEM_BASE | PMU_DMEM_SLICE_START);
		u8 end_slice = *(const volatile u8 *)(PMU_DMEM_BASE | PMU_DMEM_SLICE_END);

		*(volatile u8 *)(PMU_DMEM_BASE | PMU_DMEM_STAGE_PARAM_406) = (u8)rank;

		for (u32 slice = start_slice; slice <= end_slice; slice++) {
			u8 mask = *(const volatile u8 *)(PMU_DMEM_BASE | PMU_DMEM_ACTIVE_SLICE_MASK_CAL);
			if (!(mask & (1 << slice)))
				continue;

			*(volatile u8 *)(PMU_DMEM_BASE | PMU_DMEM_STAGE_PARAM_407) = (u8)slice;

			u32 ptr_3708 = (PMU_DMEM_BASE | 0x3708) + rank * 5280 + slice * 1320;
			u32 ptr_dc8  = (PMU_DMEM_BASE | PMU_DMEM_EYE_SAMPLE_BUF) + rank * 5280 + slice * 1320;

			for (u32 lane = 0; lane < 10; lane++) {
				if (mode & (1 << lane)) {
					*(volatile u8 *)(PMU_DMEM_BASE | PMU_DMEM_STAGE_PARAM_408) = (u8)lane;
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
 * pmu_eye_midpoint_sample_avg() - Average eye margin sample coordinates across two points
 * @out: Output 2-byte array receiving averaged (X, Y) coordinates
 * @samples: Pointer to eye diagram sample array
 * @a: Index of first sample point
 * @b: Index of second sample point
 */
void pmu_eye_midpoint_sample_avg(u8 *out, const u8 *samples, u32 a, u32 b)
{
	u32 s = a + b;
	out[0] = (u8)(s >> 1);
	u32 idx = s & ~1;
	out[1] = (u8)(((u32)samples[idx] + (u32)samples[idx + 1]) >> 1);
}

/**
 * pmu_sample_margin_center_eval() - Evaluate sample margin center and aperture width
 * @samples: Pointer to eye diagram sample buffer
 * @out_margins: Output array receiving left/right margin limits
 * @out_mode: Output pointer for evaluated margin mode byte
 *
 * Return: Margin center evaluation status code (0 on success).
 */
u32 pmu_sample_margin_center_eval(const u8 *samples, u8 *out_margins, u8 *out_mode)
{
	u16 cfg = dmem_read16(PMU_DMEM_DRAM_CFG_FLAGS);
	u32 mode_field = (cfg >> 6) & 3;
	*out_mode = (mode_field == 1) ? 1 : ((mode_field == 3) ? 3 : 2);

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
 * pmu_eye_centroid_avg_calc() - Synthesize multi-lane eye margin centroid and average step
 * @base_delays: Input array of base slice delay values
 * @out_left: Output array receiving computed left eye boundaries
 * @out_right: Output array receiving computed right eye boundaries
 * @out_width: Output array receiving computed eye window widths
 */
void pmu_eye_centroid_avg_calc(const u16 *base_delays, u16 *out_left, u16 *out_right, u16 *out_width)
{
	u8 train_flag15 = dmem_read8(PMU_DMEM_TRAIN_MODE_HI);
	s32 min_margin = (train_flag15 & 1) ? -44 : -128;

	u8 mask_b98 = *(const volatile u8 *)(PMU_DMEM_BASE | PMU_DMEM_ACTIVE_SLICE_MASK);
	u8 start_slice = dmem_read8(PMU_DMEM_SLICE_START);
	u8 end_slice = dmem_read8(PMU_DMEM_SLICE_END);

	s32 target_margin = (s32)0xd4 & min_margin;
	const volatile u8 *lane_base = (const volatile u8 *)(PMU_DMEM_BASE | 0x6318);
	__asm__("" : "+r"(lane_base));

	for (u32 slice = start_slice; slice <= end_slice; slice++) {
		if (!(mask_b98 & (1 << slice)))
			continue;

		u32 lane_idx = slice * 10;
		u32 lane_sample_offset = 848 * lane_idx;
		u16 base_val = base_delays[slice];

		for (u32 lane = 0; lane < 10; lane++, lane_idx++, lane_sample_offset += 848) {
			const volatile u8 *p = lane_base + lane_sample_offset;
			u8 type = p[0x27c];
			u32 right_bound = 0;
			s32 left_bound = 0;

			if (type != 0) {
				right_bound = p[0];
				if (type == 1) {
					left_bound = min_margin;
				} else if (type == 3) {
					u8 val_lo = p[1];
					u8 val_hi = p[2];
					s32 span = (s32)val_lo - (s32)right_bound;
					s32 rem_span = target_margin - (s32)val_hi;
					if (span < rem_span) {
						right_bound = val_hi;
						left_bound = min_margin;
					}
				}
			}

			s32 avg = ((s16)right_bound + (s8)left_bound) / 2;
			out_left[lane_idx] = base_val + (u16)avg;
			out_right[lane_idx] = base_val + (u16)right_bound;
			out_width[lane_idx] = base_val + (u16)(s8)left_bound;
		}
	}
}

/**
 * pmu_window_centroid_calc() - Compute window centroid across 64 sample pairs and store results
 * @out: Output buffer receiving computed centroid X (out[130]) and Y (out[131])
 *
 * Return: Window centroid evaluation status (0 on success, non-zero error).
 */
u32 pmu_window_centroid_calc(u8 *out)
{
	u16 flag15 = dmem_read16(PMU_DMEM_DRAM_CFG_FLAGS);
	u32 weighting_mode = flag15 & 0x1800;

	u32 weighted_sum = 0;
	u32 sq_weighted_sum = 0;
	u32 total_width = 0;
	u32 sq_weight_sum = 0;
	u32 log_weight_sum = 0;
	u32 log_weighted_sum = 0;
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
		total_width += width;
		total_weighted += prod;

		if (weighting_mode == 0) {
			weighted_sum += width * i;
		} else if (weighting_mode == 0x800) {
			u32 sq = width * width;
			sq_weight_sum += sq;
			sq_weighted_sum += sq * i;
		} else {
			u32 weight = 0;
			u32 w = width >> 1;
			while (w > 0) {
				weight++;
				w >>= 1;
			}
			log_weight_sum += weight;
			log_weighted_sum += weight * i;
		}
	}

	if (total_width == 0) {
		pmu_cal_metric_log(4, 0x1990002, weighted_sum, total_weighted);
		pmu_cal_metric_log(4, 0xcd << 17);
		return 3;
	}

	u32 centroid_val = (total_weighted / total_width) + 1;
	u32 centroid_mid = centroid_val >> 1;

	if (weighting_mode != 0) {
		if (weighting_mode == 0x800) {
			log_weighted_sum = sq_weighted_sum;
			log_weight_sum = sq_weight_sum;
		}
		total_width = log_weight_sum;
		weighted_sum = log_weighted_sum;
	}

	u32 weighted_avg = ((weighted_sum << 1) / total_width) + 1;
	u32 res_centroid = weighted_avg >> 1;

	out[131] = (u8)centroid_mid;
	out[130] = (u8)res_centroid;
	return 0;
}

/**
 * pmu_eye_margin_window_search() - Search for valid eye margin window across sample coordinates
 * @samples: Pointer to 64-coordinate eye diagram sample buffer
 * @reserved_arg: Unused alignment parameter
 *
 * Return: Margin search status code (0 on success, non-zero error).
 */
u32 pmu_eye_margin_window_search(u8 *samples, u32 reserved_arg)
{
	u8 scan_lut[512];
	u8 out_margins[6];
	u8 out_mode;

	pmu_eye_sample_table_transform(scan_lut, samples);
	u32 ret = pmu_sample_margin_center_eval(samples, out_margins, &out_mode);
	if (ret != 0)
		return ret;

	u32 margin_start_idx;
	u32 margin_end_idx;
	if (out_mode == 2) {
		margin_start_idx = 2;
		margin_end_idx = 1;
	} else if (out_mode == 3) {
		margin_start_idx = 2;
		margin_end_idx = 0;
	} else {
		margin_start_idx = 0;
		margin_end_idx = 0;
	}

	u32 min_idx = (margin_start_idx < margin_end_idx) ? margin_start_idx : margin_end_idx;
	u32 center_delta = ~min_idx + margin_start_idx;
	u32 loop_count = center_delta + 2;
	const u8 *p_margin = out_margins + (margin_start_idx << 1);

	u32 best_metric = 0;
	u8 best_x = 0;
	u8 best_y = 0;

	for (u32 i = 0; i < loop_count; i++) {
		u8 pair[2];
		u32 metric = pmu_cal_margin_sample_pair_extract(pair, p_margin, samples, scan_lut);
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
 * pmu_eye_margin_bidir_scan() - Perform bi-directional eye margin boundary scan from centroid
 * @table: Pointer to 64-pair eye margin table with centroid at [130, 131]
 * @lane_idx: Target lane or slice selector index
 * @ch_offset_flag: Channel offset flag (if non-zero, offsets destination by 0x1b)
 * @rank: DRAM rank index (0 = rank 0, 1 = rank 1)
 */
void pmu_eye_margin_bidir_scan(const u8 *table, u32 lane_idx, u32 ch_offset_flag, u32 rank)
{
	u8 center_x = table[130];
	u8 center_y = table[131];

	u32 scan_left = (u32)center_x + 1;
	const u8 *p = table + ((u32)center_x << 1);

	while (scan_left > 0) {
		if (p[0] >= center_y || center_y >= p[1])
			break;
		p -= 2;
		scan_left--;
	}

	s32 stop_bwd = (s32)scan_left - 1;
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

	uintptr_t target_addr = (PMU_DMEM_BASE | PMU_DMEM_SEARCH_WIN_COARSE);
	if (ch_offset_flag != 0)
		target_addr += 0x1b;
	if (rank == 1)
		target_addr += 4;
	if (lane_idx == 1)
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
 * pmu_eye_margin_step_align() - Balance and align eye margin coarse and fine steps
 * @margin_cur: Current eye margin record (margin_cur[0]=coarse, margin_cur[1]=fine)
 * @margin_target: Target eye margin record (margin_target[0]=coarse, margin_target[1]=fine)
 * @step_size: Adjustment step increment
 */
void pmu_eye_margin_step_align(u8 *margin_cur, u8 *margin_target, u32 step_size)
{
	u8 v1 = margin_target[0];
	u8 v0 = margin_cur[0];

	if (v0 > v1) {
		if (v0 < step_size) {
			margin_cur[0] = v0 - 1;
			margin_cur[1] += 64;
			if ((s8)margin_cur[1] < 0)
				margin_cur[1] = 0x7f;
		} else {
			margin_cur[0] = v0 - step_size;
			if (step_size == 1) {
				margin_cur[1] += 64;
				if ((s8)margin_cur[1] < 0)
					margin_cur[1] = 0x7f;
			} else if (step_size == 2) {
				margin_cur[1] = 0x7f;
			} else {
				if ((s8)margin_cur[1] < 0)
					margin_cur[1] = 0x7f;
			}
		}
	} else if (v0 < v1) {
		if (v1 < step_size) {
			margin_target[0] = v1 - 1;
			margin_target[1] += 64;
			if ((s8)margin_target[1] < 0)
				margin_target[1] = 0x7f;
		} else {
			margin_target[0] = v1 - step_size;
			if (step_size == 1) {
				margin_target[1] += 64;
				if ((s8)margin_target[1] < 0)
					margin_target[1] = 0x7f;
			} else if (step_size == 2) {
				margin_target[1] = 0x7f;
			} else {
				if ((s8)margin_target[1] < 0)
					margin_target[1] = 0x7f;
			}
		}
	}

	pmu_assert_or_halt(margin_cur[0] == margin_target[0], 0xa40003);
}

/**
 * pmu_window_margin_diff_calc() - Compute intersection and differential between two eye margin tables
 * @src0: Pointer to primary 64-pair eye margin table
 * @src1: Pointer to secondary 64-pair eye margin table
 * @out_dest: Output destination buffer for differential margin records
 * @offset: Delay tap offset adjustment
 * @delta: Margin difference threshold
 *
 * Return: Margin differential calculation status code.
 */
u32 pmu_window_margin_diff_calc(const u8 *src0, const u8 *src1, u8 *out_dest, s32 offset, s32 delta)
{
	u16 cfg = *(const volatile u16 *)(PMU_DMEM_BASE | PMU_DMEM_DRAM_CFG_FLAGS);
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

		const u8 *src_entry0 = src0 + (i << 1);
		u8 s0_high = src_entry0[1];
		if (s0_high == 0) {
			d[0] = 0xff;
			d[1] = 0x00;
			continue;
		}

		const u8 *src_entry1 = src1 + (j << 1);
		u8 s1_high = src_entry1[1];
		if (s1_high == 0) {
			d[0] = 0xff;
			d[1] = 0x00;
			continue;
		}

		s32 h = (s32)s1_high - delta;
		if (h > 127)
			h = 127;
		u8 max_high = (s0_high < (u8)h) ? s0_high : (u8)h;

		s32 l = (s32)src_entry1[0] - delta;
		if (l < 0)
			l = 0;
		u8 min_low = (src_entry0[0] > (u8)l) ? src_entry0[0] : (u8)l;

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
 * pmu_rank_margin_channel_accum() - Accumulate eye aperture margin across active channel slices
 * @rank: DRAM rank index (0..1)
 * @start_slice: Starting byte slice index (0..3)
 * @end_slice: Ending byte slice index (0..3)
 * @flags: Margin accumulation control flags
 * @swap_mode: DQ pin swap evaluation mode
 *
 * Return: Accumulated margin eye width in delay steps.
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

		const u8 *sub = (const u8 *)((PMU_DMEM_BASE | PMU_DMEM_EYE_SAMPLE_BUF) + rank_offset + (slice * 1320));
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
 * pmu_eye_sample_metric_eval() - Evaluate eye diagram sample metric against scan lookup table
 * @x: Sample X coordinate (delay tap)
 * @y: Sample Y coordinate (VREF DAC level)
 * @eye_scan_lut: Pointer to eye scan lookup table
 * @samples: Pointer to eye diagram sample array
 *
 * Return: Combined score and distance metric word ((score << 8) | dist).
 */
u32 pmu_eye_sample_metric_eval(u8 x, u8 y, const u8 *eye_scan_lut, const u8 *samples)
{
	const u8 *pair = samples + ((u32)x << 1);
	u8 low = pair[0];
	u8 high = pair[1];

	if (low >= high || low > y || high <= y)
		return 0;

	u32 coord = ((u32)y << 8) | x;
	u8 min_err = 0xff;
	u32 min_val = 0xffffffff;

	const u8 *p = eye_scan_lut;
	for (u32 i = 0; i < 64; i++) {
		s16 sample_val0 = *(const s16 *)(p);
		s16 sample_val1 = *(const s16 *)(p + 2);
		if (sample_val1 >= sample_val0) {
			u8 out_byte = 0;
			u32 ret = pmu_eye_error_metric_eval(coord, i, (s8)sample_val0, (s8)sample_val1, &out_byte);
			if (ret < min_val)
				min_val = ret;
			if (out_byte < min_err)
				min_err = out_byte;
		}

		s16 sample_val2 = *(const s16 *)(p + 4);
		s16 sample_val3 = *(const s16 *)(p + 6);
		if (sample_val3 >= sample_val2) {
			u8 out_byte = 0;
			u32 ret = pmu_eye_error_metric_eval(coord, i, (s8)sample_val2, (s8)sample_val3, &out_byte);
			if (ret < min_val)
				min_val = ret;
			if (out_byte < min_err)
				min_err = out_byte;
		}
		p += 8;
	}

	u32 dist = 0xff;
	if (low < y && y < high) {
		u32 dist_low = y - low;
		u32 dist_high = high - y;
		dist = (dist_low < dist_high) ? dist_low : dist_high;
	}

	u16 w17 = dmem_read16(PMU_DMEM_METRIC_COEFF_Y);
	u16 w32 = dmem_read16(PMU_DMEM_METRIC_COEFF_X);

	u32 term1 = (u32)(x + 1) * (u32)(x + 1) * w17;
	u32 term2 = (u32)(64 - x) * (u32)(64 - x) * w17;
	u32 term3 = (u32)(128 - y) * (u32)(128 - y) * w32;

	if (term1 < min_val)
		min_val = term1;
	if (term2 < min_val)
		min_val = term2;
	if (term3 < min_val)
		min_val = term3;

	u8 d0e = dmem_read8(PMU_DMEM_CAL_SHIFT_CFG);
	if (!(d0e & 1)) {
		u32 term4 = (u32)(y + 1) * (u32)(y + 1) * w32;
		if (term4 < min_val)
			min_val = term4;
	}

	u32 score = (min_val > 0xffffff) ? 0xffffff : min_val;
	u32 min_error_val = (min_err < x) ? min_err : x;
	u32 rx_comp = 63 - x;
	if (rx_comp < min_error_val)
		min_error_val = rx_comp;

	return (score << 8) | ((dist + min_error_val) & 0xff);
}

/**
 * pmu_eye_boundary_sample_search() - Search eye boundary sample coordinates matching target Y coordinate
 * @samples: Pointer to eye diagram sample array
 * @target_y: Target VREF level (Y coordinate)
 *
 * Return: Search status code (0 on success, non-zero error).
 */
u32 pmu_eye_boundary_sample_search(u8 *samples, u32 target_y)
{
	u8 margins[6];
	u8 mode;
	u32 ret = pmu_sample_margin_center_eval(samples, margins, &mode);
	if (ret != 0)
		return ret;

	u8 scan_lut[512];
	pmu_eye_sample_table_transform(scan_lut, samples);

	u32 metric0 = pmu_eye_sample_metric_eval(margins[0], target_y, scan_lut, samples);
	u32 best_score = 0;
	u32 best_sample = metric0;

	const u8 *p = samples;
	for (u32 sample_x = 0; sample_x <= 0x3f; sample_x += 8, p += 16) {
		if (p[0] >= target_y || target_y >= p[1])
			continue;
		u32 score = pmu_eye_sample_metric_eval((u8)sample_x, target_y, scan_lut, samples);
		if (score > best_score) {
			best_score = score;
			best_sample = sample_x;
		}
	}

	for (u32 step = 4; step >= 2; step >>= 1) {
		s32 cand1 = (s32)best_sample + (s32)step;
		if (!(cand1 & ~0x3f)) {
			u32 score = pmu_eye_sample_metric_eval((u8)cand1, target_y, scan_lut, samples);
			if (score > best_score) {
				best_score = score;
				best_sample = (u32)cand1;
			}
		}
		s32 cand2 = (s32)best_sample - (s32)step;
		if (!(cand2 & ~0x3f)) {
			u32 score = pmu_eye_sample_metric_eval((u8)cand2, target_y, scan_lut, samples);
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
 * pmu_eye_margin_step_balancer() - Balance eye margin step differentials across 4 sample buffers
 * @buf0: Pointer to first margin sample buffer
 * @buf1: Pointer to second margin sample buffer
 * @buf2: Pointer to third margin sample buffer
 * @buf3: Pointer to fourth margin sample buffer
 *
 * Return: Margin step balance status code (0 on success).
 */
u32 pmu_eye_margin_step_balancer(u8 *buf0, u8 *buf1, u8 *buf2, u8 *buf3)
{
	u16 cfg = *(const volatile u16 *)(PMU_DMEM_BASE | PMU_DMEM_DRAM_CFG_FLAGS);
	if (cfg & 2) {
		pmu_cal_metric_log(4, 0x1950000);
	}

	u8 *bufs[4] = { buf0, buf1, buf2, buf3 };
	u32 count1 = (buf1 != buf0) ? 2 : 1;
	u32 count2 = (buf2 != buf0) ? 2 : 1;

	for (u32 rank_idx = 0; rank_idx < count1; rank_idx++) {
		dmem_write8(PMU_DMEM_STAGE_PARAM_406, rank_idx);
		for (u32 i = 0; i < count2; i++) {
			u8 *b = bufs[i * 2 + rank_idx];
			u32 ret = pmu_window_centroid_calc(b);
			if (ret != 0)
				return ret;
		}
	}

	if (cfg & 2) {
		pmu_cal_metric_log(4, 0x1960000);
	}

	u8 *primary_buf = buf0;
	u8 *sec_buf = buf1;
	u8 temp_buf[128];

	for (u32 pair_idx = 1; pair_idx <= count2; pair_idx++) {
		s8 off = (s8)sec_buf[130] - (s8)primary_buf[130];
		dmem_write8(PMU_DMEM_STAGE_PARAM_406, 3);
		pmu_window_margin_diff_calc(primary_buf, sec_buf, temp_buf, off, 0);

		u32 ret;
		u8 d0b = dmem_read8(PMU_DMEM_CAL_SELECT_FLAGS);
		if (d0b & 4) {
			ret = pmu_window_centroid_calc(temp_buf);
		} else {
			ret = pmu_eye_margin_window_search(temp_buf, 0);
		}
		if (ret != 0)
			return ret;

		primary_buf[131] = temp_buf[131];
		sec_buf[131] = temp_buf[131];

		if (pair_idx < count2) {
			primary_buf = buf2;
			sec_buf = buf3;
		}
	}

	if (cfg & 2) {
		pmu_cal_metric_log(4, 0x1970000);
	}

	if (buf2 != buf0) {
		for (u32 buf_idx = 0; buf_idx < count1; buf_idx++) {
			u8 *rx = bufs[buf_idx];
			u8 *ry = bufs[buf_idx + 2];
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

/* Forward declaration of pmu_cal_slice_step_eval_sweep */
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
								   const void *sweep_ctx);

/**
 * pmu_cal_margin_matrix_scan_eval() - Evaluate 2D margin matrix scan across channels and slices
 * @channel: Memory channel index (0 or 1)
 * @rank: DRAM rank index (0 or 1)
 * @slice: DBYTE slice index (0..3)
 * @mode: Matrix scan operating mode
 * @flag: Calibration control flag
 * @step: Delay step increment
 */
void pmu_cal_margin_matrix_scan_eval(u32 channel, u32 rank, u32 slice, u32 mode, u32 flag, u32 step)
{
	u16 scan_metric_min[4] = {0};
	u16 scan_metric_max[4] = {0};
	u8 scan_profile_table[48] = {0};
	u8 baseline_buf[4][10];

	u32 bound_flag = (slice >> 8) & 1;
	u32 status_query = pmu_cal_status_query(bound_flag, mode);
	u32 profile_mode = pmu_cal_profile_mode_get();

	u8 dmem_0d = dmem_read8(PMU_DMEM_CAL_MISC_FLAGS);
	u32 channel_scan_mask = 1;
	u32 active_scan_flag = (dmem_0d & 0x10) ? 0 : 1;
	u32 is_extended_sweep = 0;

	if (rank != 0) {
		is_extended_sweep = (dmem_0d >> 3) & 1;
		active_scan_flag = 0;
	}

	if (profile_mode == 3)
		channel_scan_mask = (mode != 2) ? 2 : 1;

	u8 dmem_40 = dmem_read8(PMU_DMEM_CH1_RANK_EN);
	u8 dmem_25 = dmem_read8(PMU_DMEM_CH0_RANK_EN);
	u32 scan_multiplier = (is_extended_sweep != 0) ? 2 : 1;

	pmu_cal_metric_log(4, 0x01aa0005, channel, mode, rank, bound_flag, flag);

	u32 flag_216 = (rank == 1) || ((u32)(rank - 3) < 5);
	u32 flag_50 = 0;
	if ((bound_flag != 0 && flag_216) || (dmem_read8(PMU_DMEM_TRAIN_STATUS) & 1)) {
		pmu_dbyte_dq_deskew_regs_restore();
		flag_50 = 1;
	}

	u32 is_mode_1 = (mode == 1);
	u32 is_mode_0 = (mode == 0);
	u32 is_pm_0 = (profile_mode == 0);
	u32 is_pm_1 = (profile_mode == 1);
	u32 is_pm_2 = (profile_mode == 2);
	u32 is_pm_3 = (profile_mode == 3);

	u32 mode_zero_cond = (flag == 0) || (is_pm_0 && is_mode_0);
	u32 channel_stride_adj = (flag == 0) ? 4 : 0;
	u32 val_r12 = mode_zero_cond ? 0x4e : 0x4f;
	u32 is_pm0_mode1 = (is_pm_0 && is_mode_1);
	u32 is_cond_or_m0 = is_pm0_mode1 || mode_zero_cond;
	u32 is_mode0_pm1 = (is_mode_0 && is_pm_1);
	u32 val_r3 = is_mode0_pm1 ? val_r12 : 0x50;
	u32 is_eval_active = is_mode0_pm1 || is_cond_or_m0;
	u32 timing_reg_alt_b = is_eval_active ? val_r3 : 0x51;

	u32 sp_words_68 = (is_mode_1 && is_pm_1) || is_eval_active;
	u32 val_r13 = sp_words_68 ? 0x4f : 0x4e;
	u32 sp_words_77 = sp_words_68 || (is_mode_0 && is_pm_2);
	u32 sp_words_79 = sp_words_77 || (is_pm_2 && is_mode_1);

	u32 val_r3_188 = is_cond_or_m0 ? val_r13 : 0x51;
	u32 timing_reg_alt_a = is_eval_active ? val_r3_188 : 0x50;

	if (!is_eval_active)
		channel_stride_adj = 3;
	else if (!is_cond_or_m0)
		channel_stride_adj = 2;
	else if (!mode_zero_cond)
		channel_stride_adj = 1;

	u32 dmem_mask = dmem_40 | dmem_25;

	for (u32 ch = 0; ch < 2; ch++) {
		u32 ch_mask = (1U << (ch + 2)) | (1U << ch);
		if (ch != 0 && (ch_mask & dmem_mask) == 0)
			continue;

		dmem_write8(PMU_DMEM_CAL_BYTE, (u8)ch);
		pmu_phy_mode_cfg_dispatch(3);
		pmu_cal_dbyte_pattern_loop_exec(ch, (u16)mode, scan_metric_min, scan_metric_max, step, (u8)active_scan_flag, is_extended_sweep);

		u8 vref_boundary_clamp = 127;
		u32 margin_diff_accum = 0;
		u32 sp_words_66;
		u32 window_eval_mode;

		if (mode < 2) {
			sp_words_66 = 128;
			window_eval_mode = (dmem_read16(PMU_DMEM_DRAM_FREQ_OFF) < 3200) ? 1 : 0;
		} else if (mode == 2) {
			sp_words_66 = 212;
			window_eval_mode = 1;
			margin_diff_accum = (dmem_read8(PMU_DMEM_TRAIN_PARAM_CTRL) >> 4) & 1;
		} else {
			sp_words_66 = 212;
			window_eval_mode = 0;
		}

		pmu_cal_bist_cmd_strobe_dispatch((u32)(uintptr_t)scan_profile_table, rank, mode, ch_mask, (u32)(uintptr_t)scan_metric_min, bound_flag, flag_50);

		*(u32 *)(scan_profile_table + 0) = 0;
		*(u32 *)(scan_profile_table + 4) = 1;
		*(u32 *)(scan_profile_table + 8) = scan_multiplier;

		u32 sp_words_74 = sp_words_66 - 1;

		u32 timing_reg_pri = (ch == 0) ? 0x4e : 0x50;
		u32 timing_reg_sec = (ch == 0) ? 0x4f : 0x51;
		u32 slice_reg_stride = (ch_mask << (ch != 0)) + 1;

		if (sp_words_68 & 1) {
			timing_reg_sec = timing_reg_alt_b;
			timing_reg_pri = timing_reg_alt_a;
			slice_reg_stride = channel_stride_adj;
		} else if (sp_words_77 & 1) {
			timing_reg_sec = timing_reg_pri;
			timing_reg_pri = (ch == 0) ? 0x4f : 0x51;
			slice_reg_stride = (rank << (ch != 0));
		}

		for (u32 k = 0; k < channel_scan_mask; k++) {
			u32 target_timing_reg = timing_reg_sec;
			u32 source_timing_reg = timing_reg_pri;
			u32 slice_active_reg = slice_reg_stride;

			if (sp_words_79 & 1) {
				/* direct from timing_reg_sec / timing_reg_pri / slice_reg_stride */
			} else if (is_mode_0 && is_pm_3) {
				target_timing_reg = (k == 0) ? 0x4e : 0x50;
				source_timing_reg = (k == 0) ? 0x4f : 0x51;
				slice_active_reg = slice_reg_stride << (k != 0);
			} else if (is_mode_1 && is_pm_3) {
				target_timing_reg = (k == 0) ? 0x4f : 0x51;
				source_timing_reg = (k == 0) ? 0x4e : 0x50;
				slice_active_reg = ((k != 0) ? 2 : 0) + 1;
			} else if (mode == 2) {
				target_timing_reg = 0x4e;
				source_timing_reg = 0x4f;
				slice_active_reg = 4;
			} else {
				pmu_assert_or_halt(0, 0x01ab0000);
				source_timing_reg = 0x4f;
				target_timing_reg = 0x4e;
			}

			u8 dly_step_flag;
			u8 scan_direction = 0;

			if (flag != 0) {
				dly_step_flag = (u8)flag;
				pmu_mailbox_param_eval_dispatch(&scan_direction, &vref_boundary_clamp, &dly_step_flag, slice_active_reg, flag_216);
			} else {
				vref_boundary_clamp = 0;
				dly_step_flag = 1;
			}

			u32 accumulated_bias = 0;
			if (flag_216)
				accumulated_bias = (pmu_cal_ptr_tag_check(0, 0) != 0) ? 1 : 0;

			u32 pmu_cfg13 = pmu_dram_cfg_flag13_check(slice_active_reg, flag_216);

			if (pmu_cfg13 != 0 && ch == 0) {
				u8 slice_start = dmem_read8(PMU_DMEM_SLICE_START);
				u8 slice_end = dmem_read8(PMU_DMEM_SLICE_END);
				u8 active_mask = dmem_read8(PMU_DMEM_ACTIVE_SLICE_MASK_CAL);
				u32 active_csr_offset = dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET);
				u8 limit_val = dmem_read8((pmu_cfg13 <= 2) ? 0x11b : 0x11a);
				u8 blink_val = (u8)(scan_direction + (127 - vref_boundary_clamp));

				for (u32 slice = slice_start; slice <= slice_end; slice++) {
					if (!(active_mask & (1 << slice)))
						continue;
					for (u32 lane = 0; lane < 10; lane++) {
						u32 reg = PHY_REG_DBYTE_BASE | (((slice << 12) | active_csr_offset | target_timing_reg | (lane << 8)) << 1);
						u16 val = phy_read16(reg);
						u8 val_b = (u8)val;
						if ((u32)vref_boundary_clamp + val_b >= 128) {
							baseline_buf[slice][lane] = blink_val;
						} else if ((s8)val_b < (s8)limit_val) {
							baseline_buf[slice][lane] = 0;
						} else {
							baseline_buf[slice][lane] = val_b - limit_val;
						}
					}
				}
			}

			memset((void *)(PMU_DMEM_BASE | PMU_DMEM_SWEEP_SAMPLE_BUF), 0, 0x8480);
			u32 step_size = dly_step_flag;
			u32 last_cur_dly = scan_direction;

			for (u32 cur_dly = scan_direction; cur_dly <= vref_boundary_clamp; cur_dly += step_size) {
				last_cur_dly = cur_dly;

				if (flag != 0) {
					if (mode == 2) {
						pmu_cal_pulse_seq_coordinator(ch, (u8)cur_dly, 3);
					} else {
						u8 slice_start = dmem_read8(PMU_DMEM_SLICE_START);
						u8 slice_end = dmem_read8(PMU_DMEM_SLICE_END);
						u8 active_mask = dmem_read8(PMU_DMEM_ACTIVE_SLICE_MASK_CAL);
						u32 active_csr_offset = dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET);

						for (u32 slice = slice_start; slice <= slice_end; slice++) {
							if (!(active_mask & (1 << slice)))
								continue;

							if (pmu_cfg13 != 0) {
								for (u32 lane = 0; lane < 10; lane++) {
									u16 dly_val = (u16)(cur_dly + baseline_buf[slice][lane]);
									u32 reg_34 = PHY_REG_DBYTE_BASE | (((slice << 12) | active_csr_offset | target_timing_reg | (lane << 8)) << 1);
									phy_write16(reg_34, dly_val);
									if (accumulated_bias != 0) {
										u32 reg_40 = PHY_REG_DBYTE_BASE | (((slice << 12) | active_csr_offset | source_timing_reg | (lane << 8)) << 1);
										phy_write16(reg_40, dly_val);
									}
								}
							} else {
								u32 reg_34 = (PHY_REG_DBYTE_BASE | 0x1e00) | (((slice << 12) | active_csr_offset | target_timing_reg) << 1);
								phy_write16(reg_34, (u16)cur_dly);
								if (accumulated_bias != 0) {
									u32 reg_40 = (PHY_REG_DBYTE_BASE | 0x1e00) | (((slice << 12) | active_csr_offset | source_timing_reg) << 1);
									phy_write16(reg_40, (u16)cur_dly);
								}
							}
						}
						pmu_delay_us(300000, 0);
					}
				}

				u32 active_csr_offset = dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET);
				phy_write16(PHY_REG_DBYTE_BCAST_PULSE | (active_csr_offset << 1), 0);

				*(u32 *)scan_profile_table = cur_dly;

				pmu_cal_slice_step_eval_sweep((void *)(PMU_DMEM_BASE | PMU_DMEM_SWEEP_SAMPLE_BUF), rank, mode, pmu_cfg13, 0, (u8)channel, window_eval_mode, scan_metric_min, bound_flag, flag_50, scan_profile_table);

				if (margin_diff_accum != 0) {
					pmu_phy_timing_delay_latch(1, ch_mask);
					pmu_cal_bist_pattern_setup(rank, ch_mask, 0, flag_50);
					pmu_cal_slice_step_eval_sweep((void *)(PMU_DMEM_BASE | PMU_DMEM_SWEEP_SAMPLE_BUF), rank, 3, pmu_cfg13, 0, (u8)channel, window_eval_mode, scan_metric_min, 0, flag_50, scan_profile_table);
					pmu_cal_bist_pattern_setup(rank, ch_mask, status_query, flag_50);
					pmu_phy_timing_delay_latch(0, ch_mask);
				}
			}

			u8 slice_start = dmem_read8(PMU_DMEM_SLICE_START);
			u8 slice_end = dmem_read8(PMU_DMEM_SLICE_END);
			u8 active_mask = dmem_read8(PMU_DMEM_ACTIVE_SLICE_MASK_CAL);
			u32 lane_div = 64 / scan_multiplier;

			for (u32 slice = slice_start; slice <= slice_end; slice++) {
				if (!(active_mask & (1 << slice)))
					continue;

				for (u32 lane = 0; lane < 10; lane++) {
					u32 entry_idx = (slice * 10) + lane;
					volatile u8 *state_bytes = (volatile u8 *)((PMU_DMEM_BASE | 0x6594) + (entry_idx * 848));

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

					uintptr_t entry_base = (PMU_DMEM_BASE | PMU_DMEM_EYE_SAMPLE_BUF) + (k * 10560) + (ch * 5280) + (slice * 1320) + (lane * 132);
					*(volatile u16 *)(entry_base + 128) = (u16)best_start;

					volatile u8 *rec_base = (volatile u8 *)((PMU_DMEM_BASE | PMU_DMEM_SWEEP_SAMPLE_BUF) + (entry_idx * 848));
					u32 shift_val = scan_multiplier - 1;

					for (u32 idx = 0; idx < 64; idx++) {
						u32 step_sampled = (u8)(best_start + (idx >> shift_val));
						u8 st = state_bytes[step_sampled];
						u8 left_val;
						u8 right_val;

						if (flag != 0) {
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
					if (margin_diff_accum == 0 && lane == 9)
						continue;

					uintptr_t entry_base = (PMU_DMEM_BASE | PMU_DMEM_EYE_SAMPLE_BUF) + (k * 10560) + (ch * 5280) + (slice * 1320) + (lane * 132);

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
					if (dmem_read8(PMU_DMEM_TRAIN_MODE_HI) & 4) {
						center = sum_steps;
					} else {
						center = sum_steps / 2;
						if (dmem_read8(PMU_DMEM_CAL_OVERRIDE_FLAG) != 0) {
							u32 width = (u32)(best_w_end - best_w_start);
							pmu_assert_or_halt((width > 5) ? 1 : 0, 0x01b10006);
						}
					}

					u16 best_start = *(volatile u16 *)(entry_base + 128);
					u16 new_baseline = scan_metric_max[slice] + (best_start * scan_multiplier);
					*(volatile u16 *)(entry_base + 128) = new_baseline;
					*(volatile s8 *)(entry_base + 130) = (s8)center;
				}
			}
		}

		pmu_dmem_reg_stream_unpack(scan_profile_table);
	}

	if (flag_50 != 0)
		pmu_dbyte_dq_deskew_regs_save_and_ramp();
}

/**
 * pmu_cal_lane_window_margin_eval() - Evaluate per-lane window margins across channels and ranks
 * @mode: Operating profile mode determining window limits
 */
void pmu_cal_lane_window_margin_eval(u32 mode)
{
	u8 *base = (u8 *)(PMU_DMEM_BASE | PMU_DMEM_CAL_STRUCT_BASE);

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

	dmem_write8(PMU_DMEM_LANE_ACTIVE_FLAGS, 0);
	dmem_write16(PMU_DMEM_CAL_RANK, 0);
	dmem_write32(PMU_DMEM_CAL_STRUCT_PTR, (u32)(uintptr_t)base);

	for (u32 ch = 0; ch < 2; ch++) {
		for (u32 rank = 0; rank < 2; rank++) {
			base[ch * 108 + rank * 54 + 3] |= 0x40;
		}
	}
	pmu_phy_reg_offset_diff_adjust(1);
	pmu_slice_status_b97_save();

	u32 csr_offset = dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET);
	phy_write16(PHY_REG_VREF_CTRL | (csr_offset << 1), 0);
	phy_write16(PHY_REG_VREF_TRIM_CFG, 1);

	pmu_delay_us(0xbebc200, 0);
	if (dmem_read8(PMU_DMEM_CAL_MISC_FLAGS) & 4)
		pmu_mailbox_post_cmd_dispatch(0x40);
	phy_write16(PHY_REG_PHY_STATUS, 3);
	pmu_delay_us(0x77359400, 0);

	if (dmem_read8(PMU_DMEM_CAL_MISC_FLAGS) & 1)
		phy_write16(PHY_REG_PHY_STATUS, 1);

	pmu_deskew_and_tracker_reset();
	u8 count = (u8)pmu_delay_clamp(0xbb8, 0);
	if (count != 0) {
		do {
			pmu_cal_sequence_pulse_send(0, 7, 0, 0, 0, 3, 0);
		} while (--count != 0);
	}
	pmu_cal_sequence_pulse_send(0x80, 7, 0, 0, 0, 3, 0);
	pmu_clk_timing_delay_latch(0, 1);

	if (dmem_read8(PMU_DMEM_AC_STEP_FLAG) == 0) {
		pmu_delay_us(0x1e8480, 0);
	} else {
		pmu_ac_lane_profile_setup();
		pmu_deskew_and_tracker_reset();

		pmu_cbt_coarse_step_pulse_seq(0, 8, 1);
		if (dmem_read8(PMU_DMEM_AC_STEP_CFG) >= 2)
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

		u8 num_entries = dmem_read8(PMU_DMEM_METRIC_TABLE_ENTRIES);
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

/*
 * pmu_cal_margin_sample_pair_extract:
 */
u32 pmu_cal_margin_sample_pair_extract(u8 *out_pair, const u8 *margins,
									   const u8 *samples, const u8 *scan_lut)
{
	u16 pmu_flags = dmem_read16(PMU_DMEM_DRAM_CFG_FLAGS);
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
				dmem_read16(PMU_DMEM_METRIC_COEFF_Y), dmem_read16(PMU_DMEM_METRIC_COEFF_X));
	}

	u32 margin_a = margins[0];
	u32 margin_b = margins[1];
	u32 best_score = pmu_eye_sample_metric_eval(margin_a, margin_b, scan_lut, samples);

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
		u32 score_sub_a = skip_sub_a ? 0 : pmu_eye_sample_metric_eval(sub_a, margin_b, scan_lut, samples);

		bool skip_add_a = (add_a == cand_a && cand_b == margin_b);
		u32 score_add_a = skip_add_a ? 0 : pmu_eye_sample_metric_eval(add_a, margin_b, scan_lut, samples);

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
		u32 score_sub_b = skip_sub_b ? 0 : pmu_eye_sample_metric_eval(margin_a, sub_b, scan_lut, samples);

		bool skip_add_b = (add_b == cand_b && cand_a == margin_a);
		u32 score_add_b = skip_add_b ? 0 : pmu_eye_sample_metric_eval(margin_a, add_b, scan_lut, samples);

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

/*
 * pmu_cal_rank_margin_window_check:
 */
/*
 * pmu_cal_rank_margin_window_check:
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
 * pmu_cal_rank_margin_window_check() - Evaluate timing margin windows across active ranks and slices
 * @tap_ptr: Mode configuration or packed tap pointer (if 0, default coefficients used)
 * @channel: Memory channel index (0 or 1)
 * @slice_mask: Bitmask of active DBYTE slices
 *
 * Return: Margin window validation status (0 on success, error code otherwise).
 */
u32 pmu_cal_rank_margin_window_check(u32 tap_ptr, u32 channel, u32 slice_mask)
{
	u32 start_slice = 0;
	u32 end_slice = 0;
	u32 rank_limit = 0;
	u32 mode = pmu_cal_profile_mode_get();

	if (channel == 1) {
		end_slice = dmem_read8(PMU_DMEM_RANK1_SLICE_END);
		start_slice = dmem_read8(PMU_DMEM_RANK1_SLICE_START);
		u8 cal_param40 = dmem_read8(PMU_DMEM_CH1_RANK_EN);
		if (cal_param40 == 3)
			rank_limit = 2;
		else if (cal_param40 == 1)
			rank_limit = 1;
		else
			rank_limit = 0;
	} else if (channel == 0) {
		end_slice = dmem_read8(PMU_DMEM_RANK_BOUNDARY);
		start_slice = dmem_read8(PMU_DMEM_RANK0_SLICE_START);
		u8 cal_param25 = dmem_read8(PMU_DMEM_CH0_RANK_EN);
		rank_limit = (cal_param25 == 3) ? 2 : 1;
	}

	pmu_cal_metric_log(4, 0x00900002, tap_ptr, rank_limit);

	if (tap_ptr == 0) {
		u8 c0 = dmem_read8(PMU_DMEM_CH0_CTRL_20);
		dmem_write16(PMU_DMEM_METRIC_COEFF_X, (u16)c0 * c0);
		u8 c1 = dmem_read8(PMU_DMEM_CH0_CTRL_21);
		dmem_write16(PMU_DMEM_METRIC_COEFF_Y, (u16)c1 * c1);
		u32 margin_window_ret = 0;

		for (u32 slice = start_slice; slice <= end_slice; slice++) {
			dmem_write8(PMU_DMEM_STAGE_PARAM_407, (u8)slice);

			for (u32 pin = 0; pin < 10; pin++) {
				if (!(slice_mask & (1 << pin)))
					continue;

				pmu_cal_metric_log(6, 0x00910004, slice, pin, rank_limit, mode);
				dmem_write8(PMU_DMEM_STAGE_PARAM_408, (u8)pin);

				u32 off = slice * 1320 + pin * 132;
				u8 *rank0 = (u8 *)(uintptr_t)((PMU_DMEM_BASE | PMU_DMEM_EYE_SAMPLE_BUF) + off);
				u8 *rank1 = (u8 *)(uintptr_t)((PMU_DMEM_BASE | 0x2268) + off);
				u8 *rank2 = (u8 *)(uintptr_t)((PMU_DMEM_BASE | 0x3708) + off);
				u8 *rank3 = (u8 *)(uintptr_t)((PMU_DMEM_BASE | 0x4ba8) + off);

				if (rank_limit == 1) {
					if (mode < 3) {
						u32 ret = pmu_cal_handler_select((u32)(uintptr_t)rank0);
						if (ret != 0)
							return (u8)ret;
						margin_window_ret = 0;
						if (dmem_read8(PMU_DMEM_CAL_ACTIVE_FLAG) != 0)
							pmu_eye_margin_bidir_scan(rank0, 0, channel, 0);
					} else if (mode == 3) {
						u32 ret = pmu_eye_margin_step_balancer(rank0, rank0, rank2, rank2);
						if (ret != 0)
							return (u8)ret;
						if (dmem_read8(PMU_DMEM_CAL_ACTIVE_FLAG) != 0) {
							pmu_eye_margin_bidir_scan(rank0, 0, channel, margin_window_ret);
							pmu_eye_margin_bidir_scan(rank2, 0, channel, margin_window_ret);
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
						if (dmem_read8(PMU_DMEM_CAL_ACTIVE_FLAG) != 0) {
							pmu_eye_margin_bidir_scan(rank0, 0, channel, 0);
							pmu_eye_margin_bidir_scan(rank1, 0, channel, 1);
						}
					} else if (mode == 2) {
						u32 ret = pmu_cal_handler_select((u32)(uintptr_t)rank0);
						if (ret != 0)
							return (u8)ret;
						if (dmem_read8(PMU_DMEM_CAL_ACTIVE_FLAG) != 0)
							pmu_eye_margin_bidir_scan(rank0, 0, channel, 0);

						ret = pmu_cal_handler_select((u32)(uintptr_t)rank1);
						if (ret != 0)
							return (u8)ret;
						margin_window_ret = 1;
						if (dmem_read8(PMU_DMEM_CAL_ACTIVE_FLAG) != 0)
							pmu_eye_margin_bidir_scan(rank1, 0, channel, 1);
					} else if (mode == 3) {
						u32 ret = pmu_eye_margin_step_balancer(rank0, rank1, rank2, rank3);
						if (ret != 0)
							return (u8)ret;
						if (dmem_read8(PMU_DMEM_CAL_ACTIVE_FLAG) != 0) {
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

	/* tap_ptr != 0 branch */
	u32 ptr_addr = (tap_ptr >> 15) | (tap_ptr << 17);
	u8 c0 = *(volatile u8 *)(uintptr_t)ptr_addr;
	dmem_write16(PMU_DMEM_METRIC_COEFF_X, (u16)c0 * c0);
	u8 c1 = *(volatile u8 *)(uintptr_t)(ptr_addr + 1);
	dmem_write16(PMU_DMEM_METRIC_COEFF_Y, (u16)c1 * c1);

	if (rank_limit == 0)
		return 0;

	u32 table_ptr_base = (PMU_DMEM_BASE | PMU_DMEM_EYE_SAMPLE_BUF) + (1320 * start_slice);

	for (u32 rank = 0; rank < rank_limit; rank++, table_ptr_base += 5280) {
		dmem_write8(PMU_DMEM_STAGE_PARAM_406, (u8)rank);

		u8 eye_window_buf_a[132];
		u8 eye_window_buf_b[132];
		for (int i = 0; i < 64; i++) {
			((u16 *)eye_window_buf_a)[i] = 0x7f00;
			((u16 *)eye_window_buf_b)[i] = 0x7f00;
		}
		*(u32 *)&eye_window_buf_a[128] = 0x3f1f0000;
		*(u32 *)&eye_window_buf_b[128] = 0x001f0000;

		/* Phase 1: Window Centroid Calculation & Difference Accumulation */
		u32 slice_ptr = table_ptr_base;
		for (u32 slice = start_slice; slice <= end_slice; slice++, slice_ptr += 1320) {
			dmem_write8(PMU_DMEM_STAGE_PARAM_407, (u8)slice);
			u8 *pin_ptr = (u8 *)(uintptr_t)slice_ptr;

			for (u32 pin = 0; pin < 10; pin++, pin_ptr += 132) {
				if (!(slice_mask & (1 << pin)))
					continue;

				u32 ret = pmu_window_centroid_calc(pin_ptr);
				dmem_write8(PMU_DMEM_STAGE_PARAM_408, (u8)pin);
				if (ret != 0)
					return (u8)ret;

				pmu_cal_metric_log(4, 0x009b0002, slice, pin);

				u32 swap = pmu_dq_swap_query(rank, slice);
				u8 *target_buf = (swap != 0) ? eye_window_buf_a : eye_window_buf_b;

				s32 delta = (s8)pin_ptr[130] - (s8)target_buf[130];
				ret = pmu_window_margin_diff_calc(target_buf, pin_ptr, target_buf, delta, 0);
				if (ret != 0)
					return (u8)ret;
			}
		}

		/* Validate accumulated aperture across both swap groups */
		u32 ret = pmu_cal_handler_select((u32)(uintptr_t)eye_window_buf_a);
		if (ret != 0)
			return (u8)ret;

		ret = pmu_cal_handler_select((u32)(uintptr_t)eye_window_buf_b);
		if (ret != 0)
			return (u8)ret;

		/* Phase 2: Eye Boundary Sample Search & Callback */
		if (start_slice <= end_slice) {
			slice_ptr = table_ptr_base;
			for (u32 slice = start_slice; slice <= end_slice; slice++, slice_ptr += 1320) {
				dmem_write8(PMU_DMEM_STAGE_PARAM_407, (u8)slice);
				u8 *pin_ptr = (u8 *)(uintptr_t)slice_ptr;

				for (u32 pin = 0; pin < 10; pin++, pin_ptr += 132) {
					if (!(slice_mask & (1 << pin)))
						continue;

					pmu_cal_metric_log(4, 0x009c0002, slice, pin);
					dmem_write8(PMU_DMEM_STAGE_PARAM_408, (u8)pin);

					u32 swap = pmu_dq_swap_query(rank, slice);
					u8 *target_buf = (swap != 0) ? eye_window_buf_a : eye_window_buf_b;
					u8 target_y = target_buf[131];

					ret = pmu_eye_boundary_sample_search(pin_ptr, target_y);
					if (ret != 0)
						return (u8)ret;

					if (dmem_read8(PMU_DMEM_CAL_ACTIVE_FLAG) != 0)
						pmu_eye_margin_bidir_scan(pin_ptr, tap_ptr, channel, rank);
				}
			}
		}
	}

	return 0;
}

/**
 * pmu_cal_timing_margin_envelope_scan() - Perform multi-slice timing margin envelope scan
 * @channel: Memory channel index (0 or 1)
 * @buf: Pointer to calibration parameter buffer in DMEM
 */
void pmu_cal_timing_margin_envelope_scan(u32 channel, void *buf)
{
	struct phy_reg_stream_entry {
		u32 addr;
		u16 val;
	} __attribute__((packed));

	u32 csr_offset = dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET);
	u32 iter_count = 0;

	phy_write16((PHY_REG_AC_PROFILE_BASE | 0x0108), 0);
	phy_write16(PHY_REG_DBYTE_BCAST_TIMING, 0);

	phy_write16(PHY_REG_VREF_TRIM_CFG | (csr_offset << 1), 1);
	phy_write16(PHY_REG_VREF_CTRL | (csr_offset << 1), 1);

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

	pmu_bist_lane_mask_set_all(0);
	pmu_phy_lane_timing_offset_set(0xff, 0, 0, 0, 0);
	pmu_phy_reg_stream_play((const u8 *)stream, sizeof(stream));
	phy_write16(PHY_REG_DBYTE_BCAST_RX_EN, (dmem_read8(PMU_DMEM_FREQ_MODE) == 0) ? 520 : 512);
	phy_write16(PHY_REG_DBYTE_BCAST_LCDL, 2);

	pmu_inactive_slices_clear();
	pmu_dbyte_cal_strobe_seq(0xf, 0xf);
	pmu_phy_reset_pulse();

	pmu_cal_struct_to_shadow16((PMU_DMEM_BASE | PMU_DMEM_CAL_STRUCT_BASE), 0x0a);
	pmu_cal_struct_mask_and((PMU_DMEM_BASE | PMU_DMEM_CAL_STRUCT_BASE), 0x0a, 0x00);
	pmu_cal_struct_mask_or((PMU_DMEM_BASE | PMU_DMEM_CAL_STRUCT_BASE), 0x0a, 0x59);
	pmu_cal_struct_to_shadow16((PMU_DMEM_BASE | PMU_DMEM_CAL_STRUCT_BASE), 0x2e);
	pmu_cal_struct_mask_and((PMU_DMEM_BASE | PMU_DMEM_CAL_STRUCT_BASE), 0x2e, 0x00);
	pmu_cal_struct_to_shadow16((PMU_DMEM_BASE | PMU_DMEM_CAL_STRUCT_BASE), 0x12);
	pmu_cal_struct_mask_or((PMU_DMEM_BASE | PMU_DMEM_CAL_STRUCT_BASE), 0x12, 0x10);

	u8 rank = dmem_read8(PMU_DMEM_CAL_RANK);
	u32 cal_struct_off = (PMU_DMEM_BASE | PMU_DMEM_CAL_STRUCT_BASE) + (rank * 108) + (channel * 54);
	u32 ch_mask_a0 = 0x15 << channel;
	u32 ch_r13 = 5 << channel;

	pmu_cal_multi_rank_timing_step(ch_mask_a0, cal_struct_off, 0, 0xfffbfbff, 0xffffbfff, 0);

	pmu_tracker_field_extract((u8 *)(PMU_DMEM_BASE | PMU_DMEM_CAL_STRUCT_BASE), 0x0a);
	pmu_tracker_field_extract((u8 *)(PMU_DMEM_BASE | PMU_DMEM_CAL_STRUCT_BASE), 0x12);
	pmu_tracker_field_extract((u8 *)(PMU_DMEM_BASE | PMU_DMEM_CAL_STRUCT_BASE), 0x2e);

	pmu_channel_timing_deskew_reset(0, 0, 0, 0, ch_r13);
	phy_write16(PHY_REG_DBYTE_BCAST_DQS_DLY0, 1);
	phy_write16(PHY_REG_DBYTE_BCAST_DQS_DLY1, 1);
	pmu_cbt_deskew_pulse_train(ch_r13, 0);

	pmu_cal_metric_log(4, 0x1f20000);
	pmu_clk_timing_delay_latch(0x60, 1);
	pmu_slice_coarse_step_wrap();
	pmu_clk_timing_delay_latch(0, 1);

	u8 history[4] = { 0xff, 0xff, 0xff, 0xff };

	pmu_dbyte_cal_strobe_seq(0xf, 0xf);
	pmu_cbt_deskew_pulse_train(ch_r13, 1);

	phy_write16(PHY_REG_DBYTE_BCAST_DQ_DLY2, 511);
	phy_write16(PHY_REG_DBYTE_BCAST_DQ_DLY3, 511);
	phy_write16(PHY_REG_DBYTE_BCAST_DQ_DLY1, 0x81);
	phy_write16(PHY_REG_DBYTE_BCAST_DQ_DLY0, 0x30);
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
				u32 dly_reg = PHY_REG_DBYTE_BASE + PHY_REG_DBYTE_LCDL_DLY + (slice * PHY_REG_DBYTE_STRIDE);
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
			u32 dly_reg = PHY_REG_DBYTE_BASE + PHY_REG_DBYTE_LCDL_DLY + (slice * PHY_REG_DBYTE_STRIDE);
			u16 dly = phy_read16(dly_reg);
			dly += 2048;
			phy_write16(dly_reg, dly);
		}
	}

	phy_write16(PHY_REG_DBYTE_BCAST_DQ_DLY2, 0);
	phy_write16(PHY_REG_DBYTE_BCAST_DQ_DLY3, 0);
	pmu_slice_coarse_step_wrap();
	phy_write16(PHY_REG_DBYTE_BCAST_DQ_DLY0, 0);
	phy_write16((PHY_REG_DBYTE_BCAST_BASE | 0x016a), 0);

	if (start_slice <= end_slice) {
		for (u32 slice = start_slice; slice <= end_slice; slice++) {
			phy_write16(PHY_REG_DBYTE_BASE + PHY_REG_DBYTE_LCDL_STATUS + (slice * PHY_REG_DBYTE_STRIDE), 1);
		}
	}

	phy_write16(PHY_REG_DBYTE_BCAST_DQ_DLY1, 0x81);
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
	u32 target_reg = csr_offset | (lut_val + 0x20);
	pmu_slice_delay_step_program(target_reg, (const u16 *)buf, 1, 0, 0, 1);

	phy_write16(PHY_REG_DBYTE_BCAST_RX_EN, 0);
	phy_write16(PHY_REG_DBYTE_BCAST_LCDL, 0x20);
	phy_write16(PHY_REG_DBYTE_BCAST_LCDL, 0);
	phy_write16((PHY_REG_PUB_SEC_BASE | 0x0b2), 0);
	phy_write16(PHY_REG_DBYTE_BCAST_DQ_DLY1, 0);
	phy_write16((PHY_REG_AC_PROFILE_BASE | 0x0108), 1);
	phy_write16(PHY_REG_DBYTE_BCAST_TIMING, 1);

	pmu_cal_strobe_secondary_pulse();
	phy_write16(PHY_REG_VREF_CTRL | (csr_offset << 1), 0);

	rank = dmem_read8(PMU_DMEM_CAL_RANK);
	cal_struct_off = (PMU_DMEM_BASE | PMU_DMEM_CAL_STRUCT_BASE) + (rank * 108) + (channel * 54);
	pmu_cal_multi_rank_timing_step(ch_mask_a0, cal_struct_off, 0, 0xfffbfbff, 0xffffbfff, 0);
}

/**
 * pmu_cal_margin_envelope_step_eval() - Evaluate timing margin envelope step convergence
 */
void pmu_cal_margin_envelope_step_eval(void)
{
	u8 clk_gate_flag = dmem_read8(PMU_DMEM_CLK_GATE_FLAG);
	u32 flags = pmu_dmem_training_flags_eval();

	pmu_cal_metric_log(0xa, 0x2180001, flags);
	pmu_phy_mode_cfg_dispatch(3);
	dmem_write16(PMU_DMEM_TRAIN_MODE, 512);

	u8 dmem_08 = dmem_read8(PMU_DMEM_DRAM_TYPE);
	u32 status_query = pmu_cal_status_query(flags, 0);
	u32 is_dram_type_2 = (dmem_08 == 2) ? 1 : 0;

	u16 cfg_val;
	if (dmem_08 == 4) {
		u16 val = phy_read16((PHY_REG_DBYTE_BASE | 0x0160));
		phy_write16((PHY_REG_PUB_BASE | 0x3ff60), val | (1 << 2));
		cfg_val = (flags == 0) ? 0x88 : 0x8c;
	} else {
		cfg_val = (flags != 0) ? 0x0c : 0x08;
	}
	phy_write16(PHY_REG_BIST_CTRL_MODE, cfg_val);

	u32 step_stride = (is_dram_type_2 << 1) + 2;

	pmu_bist_lane_mask_set_all(0xffff);
	phy_write16(PHY_REG_BIST_CTRL_TRIG, 0);
	pmu_dbyte_cal_strobe_seq(0xf, 0xf);

	phy_write16(PHY_REG_DBYTE_BCAST_DQS_DLY0, 0xff);
	phy_write16(PHY_REG_DBYTE_BCAST_DQS_DLY1, 1);
	phy_write16(PHY_REG_DBYTE_BCAST_RX_EN, 0);
	phy_write16(PHY_REG_DBYTE_BCAST_DQ_DLY2, 511);
	phy_write16(PHY_REG_DBYTE_BCAST_DQ_DLY3, 511);
	phy_write16(PHY_REG_DBYTE_BCAST_DQ_DLY1, 1);

	pmu_phy_slice_mask_set((flags != 0) ? 0x1ff : 0xff);

	phy_write16(PHY_REG_BIST_STAT_FAIL, 256);
	phy_write16(PHY_REG_BIST_STAT_WORD, 256);
	pmu_cal_strobe_pulse();

	if (dmem_read8(PMU_DMEM_FREQ_MODE) <= 1)
		pmu_phy_reset_pulse();

	u32 train_flags_16 = dmem_read8(PMU_DMEM_TRAIN_FLAGS_16);
	u32 latch_delay = (clk_gate_flag & 4) ? 0 : 15;
	u32 csr_offset = dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET);
	u8 slice_start = dmem_read8(PMU_DMEM_SLICE_START);
	u8 slice_end = dmem_read8(PMU_DMEM_SLICE_END);
	u8 slice_mask = dmem_read8(PMU_DMEM_ACTIVE_SLICE_MASK_CAL);

	s32 best_step = -1;

	for (s32 rank = 1; rank >= 0; rank--) {
		u32 rank_mask = (1 << (rank + 2)) | (1 << rank);
		u8 active_mask = (dmem_read8(PMU_DMEM_CAL_RANK) != 0) ? dmem_read8(PMU_DMEM_CH1_RANK_EN) : dmem_read8(PMU_DMEM_CH0_RANK_EN);

		if (!(active_mask & rank_mask))
			continue;

		phy_write16(PHY_REG_BIST_CMD, status_query ? 0x16 : 0x10);
		pmu_deskew_and_tracker_reset();
		pmu_cbt_cal_stat_set();

		cbt_def();
		pmu_cal_sequence_pulse_send(1 << 20, 5, 0, 0x80, 0, rank_mask, 0);
		pmu_cal_sequence_pulse_send(1 << 23, 0x29, step_stride, 0, 0, rank_mask, 7);
		pmu_cal_sequence_pulse_send(0, 7, dmem_read8(PMU_DMEM_CAL_CFG_400), 0, 0, 0, 0);
		cbt_def();
		pmu_cal_sequence_pulse_send(1 << 21, 5, 0, 256, 0, rank_mask, 0);
		pmu_cal_sequence_pulse_send(0x41 << 18, 0x2a, step_stride, 0, 0, rank_mask, 7);
		pmu_cal_sequence_pulse_send(0, 7, 4, 0, 0, 0, 10);

		pmu_cbt_cal_stat_clear();

		cbt_def();
		cbt_def();
		pmu_cal_sequence_pulse_send(0x80, 7, 4, 0, 0, 0, 0);

		u32 found_pass = 0;
		for (u32 step = 0; step < 32; step++) {
			set_slice_step(slice_start, slice_end, slice_mask, csr_offset, (u16)step);

			pmu_hw_timer_delay(0x14);
			pmu_cal_strobe_secondary_pulse();
			pmu_clk_timing_delay_latch(latch_delay, 1);

			u16 err_accum = 0;
			for (u32 slice = slice_start; slice <= slice_end; slice++) {
				if (!(slice_mask & (1 << slice)))
					continue;

				u16 err = phy_read16((PHY_REG_DBYTE_BASE | 0x017a) | (slice << 13));
				err_accum |= err;

				if (err == 0)
					pmu_cal_metric_log(4, 0x1be0003, (u32)rank, slice, step);
			}

			pmu_dbyte_cal_strobe_seq(0xf, 0xf);

			if (err_accum == 0) {
				u16 param_val = (u16)(step + train_flags_16);
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

	u32 calc_step = train_flags_16 + (u32)best_step;
	u16 final_step = (calc_step > 31) ? 31 : (u16)calc_step;

	set_slice_step(slice_start, slice_end, slice_mask, csr_offset, final_step);

	pmu_cal_strobe_secondary_pulse();
	phy_write16(PHY_REG_DBYTE_BCAST_DQ_DLY2, 0);
	phy_write16(PHY_REG_DBYTE_BCAST_DQ_DLY3, 0);
	phy_write16(PHY_REG_DBYTE_BCAST_DQ_DLY1, 0);

	dmem_write16(PMU_DMEM_TRAIN_MODE, 0x8200);

	if (dmem_read8(PMU_DMEM_STAGE_STEP_MODE) == 0)
		pmu_dmem_stride_descriptor_read();

	pmu_post_cmd_conditional_dispatch(9);
}

/*
 * pmu_eye_sample_table_transform:
 * Transforms raw 64-point (x, y) eye sample coordinates into bounding box window tables.
 */
/**
 * pmu_eye_sample_table_transform() - Transform raw 64-point eye sample coordinates into window bounds
 * @dest: Pointer to destination margin bounds table in DMEM
 * @src: Pointer to source 64-coordinate eye diagram sample array
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
 * pmu_cal_ca_eye_margin_sweep() - Sweep CA bus delay taps and determine optimal eye center
 *
 * Sweeps CA (Command/Address) bus 128 delay taps across active channels
 * and ranks, evaluates eye aperture margin, determines the optimal eye center,
 * programs CA delay CSRs, and updates mission mode DMEM parameters.
 */
void pmu_cal_ca_eye_margin_sweep(void)
{
	u16 vref_window_dly = 0;
	u8 dq_check_flags = 0;
	u8 sample_edge_idx = 0;

	dmem_write16(PMU_DMEM_DELAY_CAL_RESULT_70, 0);
	dmem_write16(PMU_DMEM_DELAY_CAL_RESULTS, 0);

	pmu_cal_metric_log(10, 0x1010000);
	pmu_cal_struct_mask_remap();

	for (u32 cfg_idx = 0; cfg_idx < 4; cfg_idx++) {
		if (pmu_cal_channel_rank_config_get((u16)cfg_idx, &vref_window_dly, &dq_check_flags, &sample_edge_idx) != 0)
			continue;

		dmem_write8(PMU_DMEM_AC_STEP_FLAG, 1);
		pmu_phy_mode_cfg_dispatch((u32)sample_edge_idx);

		u32 dq_status_result = pmu_cal_dbyte_dq_status_check((u32)dq_check_flags, 1);

		dmem_write8(PMU_DMEM_AC_STEP_FLAG, 0);
		pmu_cbt_timing_pulse_coordinator(dq_status_result, 1);

		u32 fp_val = (u32)vref_window_dly;
		u32 fp_byte_val = fp_val & 0xff;
		u32 vref_step_acc = 0;
		pmu_dbyte_deskew_phase_sample(fp_byte_val, &vref_step_acc, 2);

		u8 tap_errors[128];
		memset(tap_errors, 0, 128);

		u32 ch = (u32)dmem_read8(PMU_DMEM_CAL_RANK);

		for (u32 tap = 1; tap < 128; tap++) {
			pmu_dbyte_pin_mask_seq(dq_status_result, ch, tap);
			u32 err_flag = 0;
			pmu_dbyte_deskew_phase_sample(fp_byte_val, &err_flag, 2);
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
				dmem_write8(PMU_DMEM_CH0_RANK_TIMING_33, (u8)center);
			else if (ch == 1)
				dmem_write8(PMU_DMEM_CH1_RANK_TIMING_4E, (u8)center);
		} else if (fp_val == 1) {
			if (ch == 0)
				dmem_write8(PMU_DMEM_CH0_RANK_TIMING_34, (u8)center);
			else if (ch == 1)
				dmem_write8(PMU_DMEM_CH1_RANK_TIMING_4F, (u8)center);
		}

		u32 center_scaled = center * 5 + 50;
		u32 margin_adj = center_scaled / 10;
		u32 rem_calc = center_scaled - (margin_adj * 10);
		pmu_cal_metric_log(5, 0x1090005, margin_adj, rem_calc, fp_val, center);

		dmem_write8(PMU_DMEM_AC_STEP_FLAG, 1);
		pmu_cal_multi_rank_deskew_sweep(dq_status_result);
	}

	dmem_write8(PMU_DMEM_LANE_ACTIVE_FLAGS, 1);
}
