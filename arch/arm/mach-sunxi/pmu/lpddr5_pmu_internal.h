/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Synopsys DesignWare DDR PHY Training Firmware Internal Prototypes
 * Target Microcontroller: Synopsys ARC EM4 (ARCv2 ISA)
 * SoC: Allwinner A733 (Sun60i) / LPDDR5 PHY (Type 9)
 */

#ifndef _LPDDR5_PMU_INTERNAL_H_
#define _LPDDR5_PMU_INTERNAL_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "sunxi_pmu_abi.h"
#include "dwc_ddrphy_pmu_regs.h"

/* Standard Library Declarations */
void *memset(void *dest, int val, size_t count);
void *memcpy(void *dest, const void *src, size_t count);

/* Reset Vector Table */
extern void (* const pmu_reset_vector[])(void);

/* Common Data Structures */
struct dly_line_entry {
	u16 val16;
	u8 val8_0;
	u8 val8_1;
};

struct pmu_cal_bist_cmd_ctx {
	u32 field_00;
	u32 field_04;
	u32 field_08;
	u32 cmd_c;
	u32 cmd_10;
	u32 stride_minus_1;
	u32 csr_offset;
	u16 slice_delays[];
};

struct pmu_eye_sample_entry {
	s16 min_x;
	s16 max_x;
	s16 min_y;
	s16 max_y;
};

/* MMIO & DMEM Inline Accessors */
/**
 * phy_read16() - Read 16-bit register from Synopsys DDR PHY MMIO
 * @reg: PHY register MMIO address or offset
 *
 * Return: 16-bit register value read from PHY MMIO address.
 */
static inline u16 phy_read16(u32 reg)
{
	return *(volatile u16 *)(uintptr_t)reg;
}

/**
 * phy_write16() - Write 16-bit register to Synopsys DDR PHY MMIO
 * @reg: PHY register MMIO address or offset
 * @val: 16-bit data value to write into PHY register
 */
static inline void phy_write16(u32 reg, u16 val)
{
	*(volatile u16 *)(uintptr_t)reg = val;
}

/**
 * dmem_read8() - Read 8-bit parameter from PMU DMEM mailbox/state region
 * @offset: Byte offset within PMU DMEM (0x0000..0xffff)
 *
 * Return: 8-bit value read from PMU DMEM offset.
 */
static inline u8 dmem_read8(u32 offset)
{
	return *(volatile u8 *)(uintptr_t)(PMU_DMEM_BASE | offset);
}

/**
 * dmem_read16() - Read 16-bit parameter from PMU DMEM mailbox/state region
 * @offset: Byte offset within PMU DMEM (0x0000..0xffff)
 *
 * Return: 16-bit value read from PMU DMEM offset.
 */
static inline u16 dmem_read16(u32 offset)
{
	return *(volatile u16 *)(uintptr_t)(PMU_DMEM_BASE | offset);
}

/**
 * dmem_read32() - Read 32-bit parameter from PMU DMEM mailbox/state region
 * @offset: Byte offset within PMU DMEM (0x0000..0xffff)
 *
 * Return: 32-bit value read from PMU DMEM offset.
 */
static inline u32 dmem_read32(u32 offset)
{
	return *(volatile u32 *)(uintptr_t)(PMU_DMEM_BASE | offset);
}

/**
 * dmem_write8() - Write 8-bit parameter to PMU DMEM mailbox/state region
 * @offset: Byte offset within PMU DMEM (0x0000..0xffff)
 * @val: 8-bit data value to write into PMU DMEM
 */
static inline void dmem_write8(u32 offset, u8 val)
{
	*(volatile u8 *)(uintptr_t)(PMU_DMEM_BASE | offset) = val;
}

/**
 * dmem_write16() - Write 16-bit parameter to PMU DMEM mailbox/state region
 * @offset: Byte offset within PMU DMEM (0x0000..0xffff)
 * @val: 16-bit data value to write into PMU DMEM
 */
static inline void dmem_write16(u32 offset, u16 val)
{
	*(volatile u16 *)(uintptr_t)(PMU_DMEM_BASE | offset) = val;
}

/**
 * dmem_write32() - Write 32-bit parameter to PMU DMEM mailbox/state region
 * @offset: Byte offset within PMU DMEM (0x0000..0xffff)
 * @val: 32-bit data value to write into PMU DMEM
 */
static inline void dmem_write32(u32 offset, u32 val)
{
	*(volatile u32 *)(uintptr_t)(PMU_DMEM_BASE | offset) = val;
}

/**
 * pmu_prof_stamp() - Record hardware timestamp for training stage profiling
 * @idx: Profiling slot index (0..12)
 */
static inline void pmu_prof_stamp(u32 idx)
{
	*(volatile u32 *)(PMU_DMEM_BASE | (0x300 + idx * 4)) = __builtin_arc_lr(0x104);
}

/* Firmware Function Prototypes */
void pmu_irq_halt(void);
void pmu_default_exception_handler(void);
void pmu_start(void);
void *memset(void *dest, int val, size_t count);
void *memcpy(void *dest, const void *src, size_t count);
void pmu_phy_clk_gate_sync_pulse(void);
void pmu_phy_reset_pulse(void);
void pmu_cbt_entry_pll_ctrl(void);
void pmu_cbt_exit_mission_handoff(void);
void pmu_cbt_state_latch(u8 val);
void pmu_timing_ctrl_set(u16 val);
void pmu_clk_gate_enable(void);
void pmu_deskew_dly0_set(u16 val);
void pmu_deskew_dly1_set(u16 val);
void pmu_cbt_step_stat_set(void);
void pmu_cal_search_win_init(u8 *buf);
void pmu_cal_marker_a_clear(void);
void pmu_clk_gate_disable(void);
u32 pmu_cal_mode_mask_test(u32 val);
u16 pmu_cal_stride_get(void);
u8 pmu_cbt_config_get(void);
void pmu_deskew_and_tracker_reset(void);
void pmu_cbt_cal_stat_clear(void);
void pmu_cal_markers_set(u16 val);
void pmu_cbt_cal_stat_set(void);
s32 pmu_dram_scaled_div_round(s32 val);
u32 pmu_dmem_timing_mode_check(void);
u32 pmu_dmem_wck_sync_mode_check(void);
u32 pmu_dmem_train_param_nibble_get(void);
u32 pmu_hdr_addr_15b_decode(const u8 *ptr);
u32 pmu_hdr_addr_14b_decode(const u8 *ptr);
u8 pmu_dq_phy_to_logical_mask(u32 phy_mask, u32 map_idx);
u32 pmu_2b_identity_lut(u32 val);
void pmu_cal_param_unpack(u32 val, u8 *out);
u32 pmu_dly_line_repack(u32 val);
void pmu_dbyte_cal_strobe_seq(u32 lane, u32 bit_idx);
u16 pmu_cal_status_148_read(void);
u8 pmu_dmem_pll_bypass_read(void);
u32 pmu_freq_lt_3200_check(void);
void pmu_dly_line_unpack(u32 val, u8 *out);
void pmu_cal_strobe_pulse(void);
void pmu_cal_strobe_secondary_pulse(void);
u32 pmu_dmem_active_lane_query(u32 channel, u32 bit_pos);
void pmu_cal_param_table_write(u32 row, u32 col, u16 val);
void pmu_bist_lane_mask_set_all(u16 mask);
void pmu_cbt_coarse_step_pulse(void);
u8 pmu_lcdl_dev_calc(u32 val);
u32 pmu_dram_cfg_flag13_check(u32 rank, u32 flag);
void pmu_slice_status_b97_save(void);
void pmu_mailbox_post_exec_wait(void);
void pmu_mailbox_post_cmd32_send(u32 val);
void pmu_mailbox_post_cmd_dispatch(u32 cmd);
void pmu_post_cmd_conditional_dispatch(u32 val);
u32 pmu_cal_marker_verify(void);
u32 pmu_cal_handler_select(u32 buf_addr);
void pmu_clk_timing_delay_latch(u16 timing, u32 wait_ack);
void pmu_clk_timing_latch(u16 timing, u32 wait_ack);
void pmu_delay_tap_step_adjust(u8 *delay, u32 odd_parity);
void pmu_delay_us(u32 count, u32 unit);
u32 pmu_delay_clamp(u32 count, u32 max_limit);
void pmu_cbt_3phase_pulse_seq(void);
void pmu_deskew_latch_seq(void);
void pmu_clk_gate_handoff(void);
void pmu_cal_param_table_init(void);
u32 pmu_dq_swap_query(u32 slice, u32 bit_idx);
void pmu_phy_cfg_target_set(u32 mode);
u32 pmu_cal_status_query(u32 check_mode, u32 status_sel);
u32 pmu_delay_step_calc(void);
void pmu_cal_struct_mask_and(u32 base, u32 byte_offset, u8 mask);
void pmu_cal_struct_mask_or(u32 base, u32 byte_offset, u8 mask);
void pmu_phy_cal_state_save(void);
void pmu_phy_cal_state_restore(void);
void pmu_inactive_slices_clear(void);
void pmu_slice_coarse_step_wrap(void);
u32 pmu_dbyte_pin_mask_calc(u32 mask, u32 map_idx);
void pmu_dbyte_pin_map_table_init(void);
void pmu_cal_struct_to_shadow16(u32 base, u32 byte_offset);
u32 pmu_dmem_training_flags_eval(void);
u32 pmu_cal_tracker_step_update(void);
int pmu_cal_stride_div_update(int stride, u32 flags);
void pmu_phy_pll_lock_poll(int timeout);
void pmu_dbyte_deskew_phase_sample(u32 rank_idx, u32 *out_mask, u32 phase_mask);
void pmu_cal_dual_rank_state_eval(void *table_base,
				  u16 *out_reg_count,
				  u16 *out_rank_idx);
void pmu_dmem_reg_stream_unpack(void *desc);
void pmu_dbyte_lane_error_mask_calc(u32 mode, u32 slice_or_rank);
void pmu_slice_margin_window_clamp(void);
void pmu_cal_window_valid_check(u32 window_start, u32 window_end);
void pmu_dmem_stride_descriptor_read(void);
void pmu_cal_error_code_log(void);
void pmu_cal_metric_tag_dump(u32 tag, const u32 *args);
void pmu_cal_metric_log(u32 level, u32 tag, ...);
void pmu_cal_stage8_pad_init(void);
void pmu_slice_deskew_state_latch(u32 step_delay, u32 lane_mask);
void pmu_cal_descriptor_apply(const void *desc);
void pmu_cal_pll_status_check(void);
void pmu_cal_marker_pair_program(u32 marker0, u32 marker1, u32 win_param, u32 win_mode);
void pmu_post_trace_log_halfwords(u32 threshold, u32 cmd32, const u16 *data, u32 count);
void pmu_assert_or_halt(u32 cond, u32 code);
u8 pmu_dbyte_deskew_sample_check(u32 mode_rank, u32 marker0, u32 marker1);
u8 pmu_cbt_entry_lookup(u32 table_base);
u8 pmu_cbt_active_entry_lookup(void);
void pmu_cal_matrix_trace_dump(u32 event_tag, u32 num_cols, const u16 *matrix);
void pmu_clear_tracker_words(volatile void *base);
u16 pmu_cal_tracker_stride_advance(u32 marker, u32 div, u32 flag);
void pmu_cbt_coarse_step_pulse_seq(u32 bypass_min_clamp, u32 steps, u32 use_fixed_unit);
u16 pmu_phy_ac_lane_timing_delay_set(u32 lane);
u32 pmu_cal_window_params_dispatch(const u8 *params_buf, u16 base_val, u32 mode_flags);
void pmu_dbyte_cal_step_latch(u32 tap_fine, u32 tap_coarse, u32 slice_flags);
void pmu_cal_pulse_burst_send(u32 burst_type, u32 count, u32 slice);
u32 pmu_cal_profile_mode_get(void);
void pmu_cal_multi_param_step_commit(u32 param_table,
				     u32 step_val,
				     u32 eval_mask,
				     u32 filter_mask,
				     u32 slice_flags);
u32 pmu_train_main(void);
u32 pmu_channel_rank_avail_check(u32 bit_idx);
void pmu_tracker_field_extract(u8 *dest, u32 byte_offset);
void pmu_tracker_cal_mode_update(u8 *desc);
u32 pmu_cal_timing_param_step_calc(void);
u32 pmu_cbt_delay_offset_lookup(void);
void pmu_cal_delay_line_init(void);
u32 pmu_cal_margin_window_eval(u8 *struct_ptr);
u32 pmu_cal_rank_margin_pair_eval(u8 *struct_a, u8 *struct_b);
void pmu_cal_slice_delay_best_fit_select(u32 idx,
					 u32 thresh1,
					 u32 thresh2,
					 const u8 *ptr3,
					 const u8 *ptr0,
					 const u8 *ptr4,
					 const u8 *ptr8,
					 u8 *slice_base);
u32 pmu_dmem_cal_offset_select(u32 rank, u32 unused, u32 direction);
u32 pmu_cal_rank_stride_count_get(u32 rank_idx);
u32 pmu_cal_channel_rank_config_get(u32 channel,
				    u16 *out_bit,
				    u8 *out_mask,
				    u8 *out_mode);
void pmu_cal_channel_status_get(u32 channel, u32 *out_val, u8 *out_a, u8 *out_b);
void pmu_cal_dmem_stride_bytes_fill(u8 *base, u8 *curr, u8 val, u32 stride);
void pmu_dbyte_dq_deskew_regs_restore(void);
void pmu_dbyte_dq_deskew_regs_save_and_ramp(void);
void pmu_phy_csr_result_stream(const void *buf, u32 len);
u32 pmu_dq_swap_polarity_check(u32 slice, u32 bit_idx);
void pmu_multiphase_pulse_seq_repeat(u32 phase_cmd, u32 param_val, u32 slice, u32 count);
void pmu_phy_lock_status_read(u16 *ac_lock, u16 *dbyte_lock);
void pmu_phy_clk_toggle_settle(void);
void pmu_phy_reg_stream_play(const u8 *stream, u32 len);
void pmu_phy_lane_timing_offset_set(u32 lane, u16 v1, u16 v2, u16 v3, u16 v4);
u32 pmu_cal_marker_verify_update(u32 marker_flags);
void pmu_cbt_cal_pulse_seq(u32 pulse_cmd);
void pmu_phy_timing_delay_latch(u32 enable_gate, u32 delay_code);
void pmu_dbyte_slice_cal_param_pulse(u32 rank, u32 slice_mask);
void pmu_profile_mailbox_stream_send(u32 channel);
void pmu_cal_struct_mask_remap(void);
void pmu_rank_mask_multiparam_dispatch(u32 shadow_update);
void pmu_phy_slice_mask_set(u32 rank_bitmask);
void pmu_phy_reg_buffer_stream(u8 *stream, u32 count, u32 is_write);
u32 pmu_freq_ratio_mult(u32 mult, u32 unused);
void pmu_clk_ratio_pulse_trigger(void);
u32 pmu_eye_error_metric_eval(u32 point_xy,
			      u32 ref_y,
			      u32 min_x,
			      u32 max_x,
			      u8 *out_metric);
void pmu_cal_timing_packet_format(u16 *out, u32 tag, u32 delay_code, u32 status);
void pmu_phy_reg_write_shadow_track(u32 reg_offset,
				    u32 val1,
				    u32 val2,
				    u32 val3,
				    u32 flags);
void pmu_rank_slice_deskew_latch(u32 step_delay, u32 lane_mask);
void pmu_phy_deskew_reset_strobe(void);
void pmu_phy_slice_profile_update(void);
void pmu_slice_reg_query_mailbox_send(u32 mode);
void pmu_phy_mode_cfg_dispatch(u32 mode);
void pmu_profile_param_stride_cfg(u32 mode);
void pmu_profile_mailbox_cmd_dispatch(u32 channel);
void pmu_phy_slice_cal_status_read(void);
void pmu_cal_state_restore_offset_prog(void);
void pmu_mailbox_param_eval_dispatch(u8 *out_val0,
				     u8 *out_val1,
				     u8 *out_val2,
				     u32 rank,
				     u32 alt_eval);
u32 pmu_eye_margin_boundary_detect(u8 *out_start, u8 *out_end, const u8 *samples);
void pmu_channel_timing_deskew_reset(u32 dly_ac0,
				     u32 dly_ac1,
				     u32 dly_ac2,
				     u32 dly_ac3,
				     u32 target_slice);
void pmu_rank_slice_margin_eval(void);
void pmu_dual_rank_phy_reg_read(void);
void pmu_multirank_slice_cal_dispatch(void *table_ptr,
				      u32 eval_error_mask,
				      u32 skip_mask_low,
				      s32 skip_mask_high,
				      u32 ch_enable_mask,
				      u32 check_settle_flags);
void pmu_slice_phy_reg_step_adjust(u32 reg_offset, u32 polarity_cond, u32 step_mode);
void pmu_eye_midpoint_sample_avg(u8 *out, const u8 *samples, u32 a, u32 b);
u32 pmu_sample_margin_center_eval(const u8 *samples, u8 *out_margins, u8 *out_mode);
void pmu_cbt_3phase_pulse_dispatch(u32 cal_mode,
				   const u8 *step_table,
				   u32 step_idx,
				   u32 flags);
void pmu_phy_cal_strobe_latch_setup(void);
void pmu_slice_delay_step_program(u32 reg_offset,
				  const u16 *data,
				  u32 is_64_div,
				  u32 shift_7,
				  u32 lanes_10,
				  u32 single_val);
u32 pmu_slice_dq_bitmask_swizzle(u32 slice, u32 dly_offset, u32 mode_sel);
u16 pmu_phy_delay_bound_margin_log(u32 slice);
void pmu_eye_centroid_avg_calc(const u16 *base_delays,
			       u16 *out_left,
			       u16 *out_right,
			       u16 *out_width);
void pmu_phy_reg_offset_diff_adjust(u32 sel_pair_idx);
void pmu_phy_profile_param_program(void);
void pmu_multiparam_cal_dispatch(void);
void pmu_phy_lcdl_delay_read(u16 *out, u32 from_phy, u32 dly_type);
void pmu_dq_lane_telemetry_bitpack(u32 flags, u32 base);
void pmu_ac_lane_profile_setup(void);
void pmu_cmd_code_map_log(u32 cmd_code, u32 sub_code);
void pmu_dbyte_cal_strobe_pulse_seq(u32 mode, u16 val, u32 type);
u32 pmu_window_centroid_calc(u8 *out);
void pmu_cal_pulse_seq_strobe(u32 param_code, u32 tap_idx, u32 slice_mask);
void pmu_slice_deskew_pulse_train(u32 rank, u32 mode);
void pmu_phy_pll_clock_timing_ctrl(void);
u32 pmu_eye_margin_window_search(u8 *samples, u32 reserved_arg);
void pmu_eye_margin_bidir_scan(const u8 *table,
			       u32 lane_idx,
			       u32 ch_offset_flag,
			       u32 rank);
void pmu_eye_margin_step_align(u8 *margin_cur, u8 *margin_target, u32 step_size);
void pmu_cal_pulse_seq_sub7(u8 val);
void pmu_cal_pulse_seq_coordinator(u32 rank, u32 param1, u32 flags);
void pmu_dmem_rank_slice_table_setup(void);
void pmu_hw_timer_delay(u32 count);
u32 pmu_window_margin_diff_calc(const u8 *src0,
				const u8 *src1,
				u8 *out_dest,
				s32 offset,
				s32 delta);
void pmu_slice_lcdl_delay_collect(u32 slice_csr, void *out_buf, u32 count);
void pmu_lcdl_delay_profile_update(u32 mode, u32 unused, u32 base_code, u32 tag);
void pmu_slice_metric_scan_and_program(void);
u16 pmu_rank_margin_channel_accum(u32 rank,
				  u32 start_slice,
				  u32 end_slice,
				  u32 flags,
				  u32 swap_mode);
void pmu_slice_deskew_pulse_seq(u32 enable);
u32 pmu_eye_sample_metric_eval(u8 x, u8 y, const u8 *eye_scan_lut, const u8 *samples);
u32 pmu_eye_boundary_sample_search(u8 *samples, u32 target_y);
u32 pmu_eye_margin_step_balancer(u8 *buf0, u8 *buf1, u8 *buf2, u8 *buf3);
u32 pmu_cal_ptr_tag_check(u32 unused, u32 ptr_tag);
void pmu_cal_search_win_max_calc(u32 window_type, u32 direction_flag);
void pmu_cal_stage39_post_results(void);
void pmu_cal_delay_line_accum_dispatch(u32 rank);
void pmu_cal_bist_cmd_strobe_dispatch(u32 ctx_addr,
				      u32 pattern_mode,
				      u32 cmd_mode,
				      u32 target_mask,
				      u32 slice_flags_addr,
				      u32 status_idx,
				      u32 setup_flags);
void pmu_cal_metric_table_log(u32 base_addr, u32 param_mode, u32 channel, u32 lane_mask);
u32 pmu_cal_metric_table_sweep_log(u32 base_addr, u32 sweep_start, u32 sweep_end);
void pmu_cal_dly_line_tap_step_adjust(u32 mode,
				      u32 ptr_tag,
				      u32 lane_mask,
				      u32 check_tag);
void pmu_cal_stage1_pre_init(void);
u32 pmu_cal_pll_lock_retry_poll(u32 tracker_ptr);
void pmu_cal_multi_branch_dispatch(u16 *out,
				   u32 flags,
				   u32 opcode,
				   u32 dispatch_param,
				   u32 val_arg0,
				   u32 val_arg1,
				   u32 flag_mode);
void pmu_cal_tracker_dac_clear(void);
u32 pmu_cal_sequence_pulse_send(u32 cmd_flags,
				u32 pulse_type,
				u32 phase_id,
				u32 param_code,
				u32 tap_idx,
				u32 slice_mask,
				u32 extra);
void pmu_cal_lane_deskew_sweep_exec(void);
void pmu_cal_margin_matrix_scan_eval(u32 channel,
				     u32 rank,
				     u32 slice,
				     u32 mode,
				     u32 flag,
				     u32 step);
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
void pmu_cal_lane_window_margin_eval(u32 mode);
u32 pmu_cal_dbyte_dq_status_check(u32 mode_rank, u32 flag);
void pmu_cal_dbyte_pattern_loop_exec(u32 reg_offset,
				     u16 pattern_mode,
				     void *dest_buf,
				     void *scratch_buf,
				     u32 bound_start,
				     u32 bound_end,
				     u32 flags);
void pmu_cal_slice_step_diff_commit(u32 slice_idx, u16 *out_buf, u32 cal_mode);
u32 pmu_cal_margin_sample_pair_extract(u8 *out_pair,
				       const u8 *margins,
				       const u8 *samples,
				       const u8 *scan_lut);
void pmu_cal_channel_rank_boundary_eval(u8 *buf, u32 channel, u32 rank, u32 cal_type);
void copy_slice_quad(u8 *buf, u32 off, const volatile u8 *src);
void pmu_cal_timing_table_matrix_init(u8 *buf);
void pmu_cal_phase_delay_tracker_slice_update(u32 rank_idx,
					      u32 active_csr_offset,
					      u32 csr_step_off,
					      u32 slice_param_off);
void pmu_cal_phase_delay_tracker_update(void);
u32 pmu_cal_slice_step_scan_eval(u32 step_val, u32 slice_mask);
void pmu_cal_rank_param_commit(u32 channel, u32 rank, u32 slice, u32 mode);
void pmu_train_dispatcher(void);
void pmu_cal_multi_lane_deskew_sweep(u32 type, u32 ch_mask, u32 lane_mask);
void pmu_cal_stage42_accum_dispatch(u32 dram_type, u32 count, void *slice_base);
u32 pmu_cal_rank_margin_window_check(u32 tap_ptr, u32 channel, u32 slice_mask);
void pmu_cal_slice_deskew_result_commit(u32 channel, void *buf, u32 flags);
void pmu_cal_timing_margin_envelope_scan(u32 channel, void *buf);
void pmu_cal_vref_dac_step_adjust(u32 stage_step);
s16 pmu_cal_dbyte_rx_fifo_reset_poll(u32 mode_sel, u32 enable_scan, void *unused_buf);
void pmu_cal_wck_ck_align_pulse_send(u32 ch, u32 rank);
void pmu_cal_rank_rx_en_delay_commit(u32 channel,
				     u16 fp_flags,
				     u32 cal_stage_mode,
				     u32 stage);
void pmu_cal_lane_window_sweep_coordinator(u32 ch_mask, u32 wide_range);
void cbt_def(void);
void set_slice_step(u8 slice_start,
		    u8 slice_end,
		    u8 slice_mask,
		    u32 csr_offset,
		    u16 step);
void pmu_cal_margin_envelope_step_eval(void);
void pmu_cal_dbyte_deskew_fine_tune(u32 mode);
void pmu_cal_phase_detector_edge_align(void);
void pmu_cal_rank_deskew_matrix_commit(u32 channel, void *buf);
void pmu_cal_stage_post_process_dcd(void);
void pmu_cal_stage_post_process_vref(u32 vref_mode);
void pmu_cal_stage_post_process_dqs(u32 freq_check_mode);
u32 pmu_freq_delay_step_calc(u32 val, u32 min_delay);
void pmu_dbyte_pin_mask_seq(u32 mode_cond, u32 rank, u32 pin_mask);
u32 pmu_rate_tier_calc(u32 val);
void pmu_cal_bist_lock_check(void);
u32 pmu_cal_rank_lane_status_check(void);
int pmu_rank_slice_bit_active(volatile u8 *dmem, u32 rank_bit);
void pmu_cal_pulse_settle_delay(const void *desc);
void pmu_cal_multi_rank_timing_step(u32 rank_mode,
				    u32 steps_table,
				    u32 eval_error_mask,
				    u32 skip_mask_low,
				    u32 skip_mask_high,
				    u32 check_settle_flags);
void pmu_cal_pulse_sync_strobe(void);
void pmu_cal_multi_rank_step_commit(u32 rank_mode,
				    u32 steps_table,
				    u32 eval_error_mask,
				    u32 skip_mask_low,
				    u32 skip_mask_high,
				    u32 pulse_coarse_pre,
				    u32 check_settle_flags);
void pmu_eye_sample_table_transform(u8 *dest, const u8 *src);
void pmu_cbt_pulse_trailer(void);
void pmu_cbt_strobe_pulse_train(u32 pulse_cmd, u32 is_stage2);
void pmu_cbt_deskew_pulse_train(u32 pulse_cmd, u32 is_alternate);
void pmu_cbt_timing_pulse_coordinator(u32 cal_mode, u32 mask_set);
void pmu_cal_bist_search_win_setup(u32 stage);
void pmu_cal_dbyte_deskew_results_apply(const void *results,
					u32 format_sel,
					u32 stage_id);
void pmu_slice_pulse_write(const u8 *slices, u32 num_slices, u16 val);
void pmu_cal_multi_rank_deskew_sweep(u32 mode);
void pmu_cal_bist_pattern_setup(u32 pattern_idx,
				u32 slice_or_lane,
				u32 enable_deskew_reset,
				u32 validation_mode);
void pmu_cal_ca_eye_margin_sweep(void);
void pmu_cal_dbyte_deskew_pin_results_apply(void);

#endif /* _LPDDR5_PMU_INTERNAL_H_ */
