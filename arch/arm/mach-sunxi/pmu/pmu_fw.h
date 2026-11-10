/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Common declarations and external symbol prototypes for standalone
 * Allwinner A733 DDR PHY PMU firmware modules.
 */

#ifndef _SUNXI_PMU_FW_H
#define _SUNXI_PMU_FW_H

#include "dwc_ddrphy_pmu_regs.h"
#undef __always_inline
#undef __attribute_const__
#include "../../../../include/linux/compiler_attributes.h"

/* ARC EM4 global architectural state registers (r26/gp = 0x80000438, r27/fp = 0x80000450) */
register __volatile__ u32 *pmu_gp __asm__("r26");
register u32 pmu_fp_rsv __asm__("fp");

/**
 * dmem_ptr() - Convert a DCCM byte offset into an ARC DMEM pointer
 * @offset: Byte offset within DCCM (or full 0x8000xxxx address)
 *
 * Return: Pointer to the target DCCM address.
 */
static inline void *dmem_ptr(u32 offset)
{
	return (void *)(uintptr_t)(PMU_DMEM_BASE | offset);
}

/**
 * dmem_read8() - Read an unsigned 8-bit byte from DCCM
 * @offset: Byte offset within DCCM (or full 0x8000xxxx address)
 *
 * Return: 8-bit value read from DCCM.
 */
static inline u8 dmem_read8(u32 offset)
{
	return *(__volatile__ u8 *)dmem_ptr(offset);
}

/**
 * dmem_read16() - Read an unsigned 16-bit halfword from DCCM
 * @offset: Byte-aligned halfword offset within DCCM
 *
 * Return: 16-bit value read from DCCM.
 */
static inline u16 dmem_read16(u32 offset)
{
	return *(__volatile__ u16 *)dmem_ptr(offset);
}

/**
 * dmem_read32() - Read an unsigned 32-bit word from DCCM
 * @offset: Word-aligned offset within DCCM
 *
 * Return: 32-bit value read from DCCM.
 */
static inline u32 dmem_read32(u32 offset)
{
	return *(__volatile__ u32 *)dmem_ptr(offset);
}

/**
 * dmem_write8() - Write an unsigned 8-bit byte to DCCM
 * @offset: Byte offset within DCCM (or full 0x8000xxxx address)
 * @val: 8-bit value to write
 */
static inline void dmem_write8(u32 offset, u8 val)
{
	*(__volatile__ u8 *)dmem_ptr(offset) = val;
}

/**
 * dmem_write16() - Write an unsigned 16-bit halfword to DCCM
 * @offset: Halfword-aligned offset within DCCM
 * @val: 16-bit value to write
 */
static inline void dmem_write16(u32 offset, u16 val)
{
	*(__volatile__ u16 *)dmem_ptr(offset) = val;
}

/**
 * dmem_write32() - Write an unsigned 32-bit word to DCCM
 * @offset: Word-aligned offset within DCCM
 * @val: 32-bit value to write
 */
static inline void dmem_write32(u32 offset, u32 val)
{
	*(__volatile__ u32 *)dmem_ptr(offset) = val;
}

/**
 * phy_read16() - Read a 16-bit DDR PHY CSR register in ARC MMIO space
 * @addr: ARC PHY register address (0x9000xxxx)
 *
 * Return: 16-bit CSR value.
 */
static inline u16 phy_read16(u32 addr)
{
	return *(__volatile__ u16 *)(uintptr_t)addr;
}

/**
 * phy_write16() - Write a 16-bit DDR PHY CSR register in ARC MMIO space
 * @addr: ARC PHY register address (0x9000xxxx)
 * @val: 16-bit CSR value to write
 */
static inline void phy_write16(u32 addr, u16 val)
{
	*(__volatile__ u16 *)(uintptr_t)addr = val;
}

/**
 * struct pmu_sdata - PMU small-data (.sdata) state block at DCCM 0x80000400
 * @cbt_dly_init: Clamped CBT delay line initial offset (0x400)
 * @high_freq_flag: High-frequency operating mode flag (0x401)
 * @cal_active_flag: Active calibration stage flag (0x402)
 * @freq_mode: Frequency mode / CBT lane limit code (0x403)
 * @timer_shift: Hardware timer prescaler shift factor (0x404)
 * @stage_step_mode: Stage 10 margin status / step mode (0x405)
 * @stage_rank_idx: Active calibration rank index (0x406)
 * @stage_slice_idx: Active calibration slice index & flags (0x407)
 * @stage_lane_idx: Active calibration lane index & mode (0x408)
 * @metric_tbl_entries: Calibration metric table entry count (0x409)
 * @pad_0a: Alignment padding halfword (0x40a)
 * @active_slice_cnt: Active DBYTE slice count (0x40c)
 * @active_ch_cnt: Active channel count / quadratic cost weight W (0x410)
 * @pad_14: Reserved word (0x414)
 * @shadow_trace_ptr: Shadow trace buffer pointer (0x418)
 * @active_csr_offset: Active channel PHY CSR address offset (0x41c)
 * @fw_version_id: Firmware version ID / quadratic cost weight X (0x420)
 * @num_ranks: Active rank count / quadratic cost weight Y (0x422)
 * @fw_build_id_lo: Firmware build ID low halfword (0x424)
 * @fw_build_id_hi: Firmware build ID high halfword (0x426)
 * @iter_count: Calibration stage iteration counter (0x428)
 * @cal_save_cfg: Saved hardware calibration configuration mask (0x42a)
 * @dbyte_slice_cnt: Active DBYTE slice limit (0x42c)
 * @pad_2d: Alignment padding bytes (0x42d..0x42f)
 * @rfifo_dly: Per-slice RFIFO delay parameters across 4 slices (0x430..0x437)
 * @step_2d_ctrl: 2D eye sweep step control (0x438)
 * @step_2d_param: 2D eye sweep step parameter (0x43a)
 * @pad_3b: Alignment padding byte (0x43b)
 * @master_dly_tbl: Master delay table [channel][rank] (0x43c..0x443)
 * @ac_profile_dly: AC profile delay halfwords (0x444..0x447)
 * @ac_profile_cfg: AC profile step configuration bytes (0x448..0x453)
 * @saved_cal_stat: Saved calibration status word (0x454)
 * @cal_marker_a: Calibration step marker A (0x458)
 * @cbt_state: CBT state latch (0x45a)
 * @cbt_step_ext: CBT step extension flag (0x45b)
 * @cbt_cal_active: CBT calibration active flag (0x45c)
 * @pad_5d: Alignment padding byte (0x45d)
 * @cal_marker_b: Calibration step marker B (0x45e)
 * @active_slice_mask: Active slice bitmask (0x460)
 * @tracker_popcnt_acc: Tracker popcount accumulator (0x464)
 * @ch_range_lim: Channel active range limit (0x468)
 * @step_mode_ovr: Channel step mode override (0x469)
 * @cbt_config: CBT configuration value (0x46a)
 * @cbt_step_stat: CBT step status flag (0x46b)
 * @cal_cmd_tbl: Shadow calibration command table [0..3] (0x46c..0x46f)
 * @active_slice_idx: Active slice index (byte 0 is cal_cmd_tbl[4]) (0x470)
 * @slice_desc_474: Per-rank/slice timing descriptor bytes (0x474..0x47b)
 * @cal_stride: Normalized power-of-2 step stride (0x47c)
 * @deskew_target_dly: Pad timing / deskew target delay (0x47e)
 * @cal_seq_map: 21-byte calibration sequence index map (0x480..0x494)
 * @pad_95: Alignment padding to 0x4c0 (0x495..0x4bf)
 * @rank_ch_tap_tbl: Per-rank/channel tap table (0x4c0..0x4c3)
 * @best_tap: Best WCK read-gate tap per channel and rank (0x4c4..0x4c7)
 */
struct pmu_sdata {
	u8 cbt_dly_init;
	u8 high_freq_flag;
	u8 cal_active_flag;
	u8 freq_mode;
	u8 timer_shift;
	u8 stage_step_mode;
	u8 stage_rank_idx;
	u8 stage_slice_idx;
	u8 stage_lane_idx;
	u8 metric_tbl_entries;
	u16 pad_0a;
	u32 active_slice_cnt;
	u32 active_ch_cnt;
	u32 pad_14;
	u32 shadow_trace_ptr;
	u32 active_csr_offset;
	u16 fw_version_id;
	u16 num_ranks;
	u16 fw_build_id_lo;
	u16 fw_build_id_hi;
	u16 iter_count;
	u16 cal_save_cfg;
	u8 dbyte_slice_cnt;
	u8 pad_2d[3];
	u16 rfifo_dly[4];
	u16 step_2d_ctrl;
	u8 step_2d_param;
	u8 pad_3b;
	u16 master_dly_tbl[2][2];
	u16 ac_profile_dly[2];
	u8 ac_profile_cfg[12];
	u32 saved_cal_stat;
	u16 cal_marker_a;
	u8 cbt_state;
	u8 cbt_step_ext;
	u8 cbt_cal_active;
	u8 pad_5d;
	u16 cal_marker_b;
	u32 active_slice_mask;
	s32 tracker_popcnt_acc;
	u8 ch_range_lim;
	u8 step_mode_ovr;
	u8 cbt_config;
	u8 cbt_step_stat;
	u8 cal_cmd_tbl[4];
	u32 active_slice_idx;
	u8 slice_desc_474[8];
	u16 cal_stride;
	u16 deskew_target_dly;
	u8 cal_seq_map[21];
	u8 pad_95[43];
	u8 rank_ch_tap_tbl[2][2];
	u8 best_tap[2][2];
};

/**
 * struct pmu_cal_tracker - Multi-channel calibration step tracker (0x80000d9c)
 * @slot0: Primary 8-halfword channel 0 command slot (0x00..0x0f)
 * @slot1: Secondary 8-halfword channel 1 command slot (0x10..0x1f)
 * @slot1_mode: Channel 1 step mode control halfword (0x20)
 * @repeat_cnt: Step repeat count minus one (0x22)
 * @slot0_mode: Channel 0 step mode control halfword (0x24)
 * @ext_flag: Step extension flag halfword (0x26)
 * @commit_flag: Pending step commit flag halfword (0x28)
 * @pad_2a: Alignment padding halfword (0x2a)
 */
struct pmu_cal_tracker {
	u16 slot0[8];
	u16 slot1[8];
	u16 slot1_mode;
	u16 repeat_cnt;
	u16 slot0_mode;
	u16 ext_flag;
	u16 commit_flag;
	u16 pad_2a;
};

/**
 * pmu_sdata() - Access the PMU small-data (.sdata) structure via r26 (gp)
 *
 * Return: Volatile pointer to struct pmu_sdata at DCCM 0x80000400.
 */
static inline __volatile__ struct pmu_sdata *pmu_sdata(void)
{
	return (__volatile__ struct pmu_sdata *)pmu_gp;
}

/**
 * pmu_tracker() - Access the primary calibration step tracker in DCCM
 *
 * Return: Volatile pointer to struct pmu_cal_tracker at DCCM 0x80000d9c.
 */
static inline __volatile__ struct pmu_cal_tracker *pmu_tracker(void)
{
	return (__volatile__ struct pmu_cal_tracker *)PMU_DMEM_CAL_TRACKER_D9C;
}

/**
 * struct pmu_cal_bist_cmd_ctx - BIST command strobe context block
 * @field_00: Primary command control word 0
 * @field_04: Secondary command control word 1
 * @field_08: Tertiary command control word 2
 * @cmd_c: Strobe command pattern word C
 * @cmd_10: Strobe command pattern word 10
 * @stride_minus_1: Loop stride count minus one
 * @pad_16: Alignment padding halfword
 * @csr_offset: Target PHY CSR channel/slice address offset
 * @slice_delays: Per-slice 16-bit delay values across 4 slices
 */
struct pmu_cal_bist_cmd_ctx {
	u32 field_00;
	u32 field_04;
	u32 field_08;
	u32 cmd_c;
	u32 cmd_10;
	u16 stride_minus_1;
	u16 pad_16;
	u32 csr_offset;
	u16 slice_delays[4];
};

/* Cross-Module PMU Firmware Function Prototypes */
void *memcpy(void *dest, const void *src, size_t n);
void *memset(void *s, int c, size_t n);
void pmu_ac_lane_profile_setup(void);
void pmu_adjust_step_limit(u8 *pair, u32 flag);
void pmu_assert_or_halt(u32 cond, u32 assert_code);
u32 pmu_bit_lookup_e4(u32 shift, u32 mask);
void pmu_cal_bist_cmd_strobe_dispatch(uintptr_t ctx_addr,
				      u32 pattern_mode, u32 cmd_mode, u32 target_mask,
				uintptr_t slice_flags_addr, u32 status_idx, u32 setup_flags);
void pmu_cal_bist_pattern_setup(u32 pattern_idx, u32 slice_or_lane,
				u32 enable_deskew_reset, u32 validation_mode);
void pmu_cal_bist_search_win_setup(u32 stage);
u32 pmu_cal_cbt_desc_unpack(u32 rank, u32 channel);
void pmu_cal_channel_rank_bounds_verify(u32 err_id);
u32 pmu_cal_channel_rank_config_get(u32 rank, u16 *out_cfg,
				    u8 *out_mode_sel, u8 *out_num_slices);
void pmu_cal_dbyte_bcast_pulse_width_clear(const u8 *cfg);
void pmu_cal_dbyte_delay_backup_all_ch(void);
void pmu_cal_dbyte_deskew_fine_tune(u32 mode);
void pmu_cal_dbyte_gate_mask_clear(void);
void pmu_cal_dbyte_pattern_loop_exec(u32 reg_offset,
				     u16 pattern_mode, void *dest_buf, void *scratch_buf,
				u32 bound_start, u32 bound_end, u32 flags);
s16 pmu_cal_dbyte_rx_fifo_reset_poll(u32 channel, u32 mode_sel,
				     u32 enable_scan, void *unused_buf);
void pmu_cal_dbyte_sample_ready_poll_all(u32 flags);
u32 pmu_cal_delay_line_accum_dispatch(u32 count, u32 limit);
void pmu_cal_delay_line_init(void);
void pmu_cal_deskew_quad_tap_set(u32 ch, u16 tap0, u16 tap1,
				 u16 tap2, u16 tap3);
u32 pmu_cal_eye_margin_quadratic_cost_calc(u32 val, u32 target,
					   u32 min_bound, u32 max_bound, u8 *out_overflow);
u32 pmu_cal_eye_margin_sample_latch(u32 mask, u32 lane);
void pmu_cal_eye_param_block_init(u16 *param_block, u32 p1, u32 p2,
				  u32 p3);
void pmu_cal_eye_table_restore(u8 *dest, u32 offset);
void pmu_cal_eye_table_save(const u8 *src, u32 offset);
void pmu_cal_gate_eye_master_trigger(void);
void pmu_cal_lane_deskew_sweep_exec(u32 channel);
u32 pmu_cal_lane_error_status_check(u32 lane, u32 phase);
void pmu_cal_lane_mask_and_all(u32 base, u32 offset, u32 mask);
void pmu_cal_lane_mask_or_all(u32 base, u32 offset, u32 mask);
void pmu_cal_lane_window_margin_eval(u32 mode);
void pmu_cal_lane_window_sweep_coordinator(u32 ch_mask, u32 wide_range);
void pmu_cal_lcdl_coarse_fine_pair_pack(u32 ch_idx, u32 threshold,
					u32 limit, const u8 *pair_a,
					const u8 *pair_b, const u8 *pair_c,
					const u8 *pair_d, void *table_base);
void pmu_cal_margin_dir_get(void);
void pmu_cal_margin_matrix_scan_eval(u32 channel, u32 rank,
				     u32 slice, u32 mode, u32 flag, u32 step);
u16 pmu_cal_marker_status_update(void);
u16 pmu_cal_marker_verify(void);
void pmu_cal_marker_verify_and_buffer_clear(u32 flags);
void pmu_cal_marker_win_step_prog(u32 marker0, u32 marker1,
				  u32 win_param, u32 win_mode);
void pmu_cal_multi_lane_deskew_sweep(u32 type, u32 ch_mask,
				     u32 lane_mask);
void pmu_cal_multi_rank_step_commit(u32 rank_mode, u32 steps_table,
				    u32 eval_error_mask, u32 skip_mask_low, u32 skip_mask_high,
				u32 pulse_coarse_pre, u32 check_settle_flags);
void pmu_cal_multi_rank_timing_step(u32 rank_mode, u32 steps_table,
				    u32 eval_error_mask, u32 skip_mask_low, u32 skip_mask_high,
				u32 check_settle_flags);
void pmu_cal_pad_impedance_lock(void);
void pmu_cal_pad_timing_delay_set(u32 bypass_min_clamp, u32 steps,
				  u32 use_fixed_unit);
void pmu_cal_pad_timing_step_settle(void);
u32 pmu_cal_pad_voltage_dac_set(u32 val, u32 mode);
void pmu_cal_param_unpack(u32 val, u8 *out);
void pmu_cal_phase_detector_edge_align(void);
void pmu_cal_pll_lock_settle_spin(u16 *out_dbyte_pll, u16 *out_ac_pll);
void pmu_cal_pulse_burst_send(u32 burst_type, u32 count, u32 slice);
void pmu_cal_pulse_seq_coordinator(u32 ch_idx, u32 rank, u32 param1, u32 flags);
void pmu_cal_rank_deskew_matrix_commit(u32 channel, u32 rank, void *buf);
u32 pmu_cal_rank_margin_window_check(u32 tap_ptr, u32 channel,
				     u32 slice_mask);
u8 pmu_cal_rank_offset_get(u32 rank, u32 channel);
void pmu_cal_rank_param_commit(u32 fp, u32 mode_cfg, u32 step_lim,
			       u32 type);
void pmu_cal_rank_rx_en_delay_commit(u32 channel, u16 fp_flags,
				     u32 cal_stage_mode, u32 stage);
void pmu_cal_rx_dqs_delay_center(void);
void pmu_cal_search_win_init(u32 base);
void pmu_cal_search_win_max_calc(u32 channel, u32 rank, u32 dir);
void pmu_cal_seq_table_timed_replay(const void *seq_table,
				    u32 total_bytes);
u32 pmu_cal_sequence_pulse_send(u32 cmd_flags, u32 pulse_type,
				u32 phase_id, u32 param_code, u32 tap_idx, u32 slice_mask,
				u32 extra);
void pmu_cal_slice_deskew_result_commit(u32 channel, void *buf,
					u32 flags);
u32 pmu_cal_slice_mask_eval(void *table_buf);
void pmu_cal_slice_step_diff_commit(u32 slice_idx, u16 *out_buf,
				    u32 cal_mode);
void pmu_cal_slice_step_diff_latch(u32 timing, u32 wait_ack);
void pmu_cal_slice_step_eval_sweep(void *results_buf,
				   u32 sweep_mode, u32 cal_mode, u32 channel_rank, u32 unused,
				u32 timing_val, u32 overflow_flag, const u16 *slice_delay_lut,
					u32 bound_flag, u32 remap_flag, const void *sweep_ctx);
void pmu_cal_slice_timing_latch(u32 timing, u32 wait_ack);
void pmu_cal_stage42_accum_dispatch(u32 dram_type, u32 count,
				    void *slice_base);
u16 pmu_cal_stride_get(void);
u32 pmu_cal_strobe_pulse_init(void);
void pmu_cal_struct_mask_remap(void);
void pmu_cal_timing_margin_envelope_scan(u32 channel, u32 rank, void *buf);
void pmu_cal_timing_table_matrix_init(u8 *buf);
u32 pmu_cal_tracker_csr_commit(const void *tracker);
void pmu_cal_tracker_reset_and_apb_zero(const void *buf, u32 len);
u32 pmu_cal_tracker_step_advance(u32 marker, u32 count, u32 mode);
void pmu_cal_training_flags_gate_update(u8 *descriptor);
void pmu_cal_vref_dac_step_adjust(u32 stage_step);
void pmu_cal_vref_search_center_update(void);
void pmu_cal_vref_trim_cfg_latch(void);
void pmu_cal_wck_ck_align_pulse_send(u32 ch, u32 rank);
void pmu_cal_win_bounds_clear(void);
u32 pmu_cal_window_params_dispatch(const u8 *params_buf,
				   u16 base_val, u32 mode_flags);
void pmu_cbt_3phase_pulse_dispatch(u32 cal_mode,
				   const u8 *step_table, u32 step_idx, u32 flags);
void pmu_cbt_3phase_pulse_seq(void);
void pmu_cbt_cal_pulse_seq(u32 pulse_cmd);
u32 pmu_cbt_delay_offset_lookup(u32 rank, u32 channel);
void pmu_cbt_deskew_pulse_train(u32 pulse_cmd, u32 is_alternate);
void pmu_cbt_strobe_pulse_train(u32 pulse_cmd, u32 is_stage2);
void pmu_cbt_timing_pulse_coordinator(u16 cal_mode, u32 phase_sel);
u32 pmu_channel_rank_avail_check(u32 channel, u32 rank);
void pmu_channel_timing_deskew_reset(u32 dly_ac0, u32 dly_ac1,
				     u32 dly_ac2, u32 dly_ac3, u32 target_slice);
u32 pmu_check_dmem_407_flag(void);
u32 pmu_check_freq_lt_3200(void);
u32 pmu_check_stage(u32 val);
void pmu_clear_458(void);
void pmu_clk_gate_handoff(void);
void pmu_clk_ratio_pulse_trigger(void);
void pmu_cmd_code_map_log(u32 cmd_code, u32 sub_code);
void pmu_dbyte_cal_step_latch(u32 fine_val, u32 coarse_val,
			      u32 ch_flags);
void pmu_dbyte_cal_strobe_pulse_seq(u32 mode, u32 val, u32 step);
u32 pmu_dbyte_coarse_step_mask_sweep(u32 ch_idx, u32 step_val, u32 flag);
void pmu_dbyte_deskew_phase_sample(u32 ch_base, u32 *out_mask,
				   u32 sel_mask);
void pmu_dbyte_dq_deskew_regs_save_and_ramp(u8 *out_a, u8 *out_b,
					    u8 *out_c, u32 flag, u32 extra);
void pmu_dbyte_lane_error_mask_calc(u32 mode, u32 slice_or_rank);
void pmu_dbyte_rx_en_slice_pulse_eval(u32 mode, u32 slice_mask);
void pmu_default_exception_handler(void);
void pmu_delay_us(u32 count, u32 unit);
u32 pmu_dly_line_repack(u32 val);
u32 pmu_dly_reg_offset_select(u32 rank, u32 unused, u32 direction);
void pmu_dmem_reg_stream_unpack(const void *cfg_desc);
u8 pmu_dmem_training_flags_eval(void);
u32 pmu_dram_cfg_flag13_check(u32 rank, u32 flag);
void pmu_eye_centroid_avg_calc(const u16 *base_delays,
			       u16 *out_left, u16 *out_right, u16 *out_width);
void pmu_eye_margin_step_align(u8 *margin_cur, u8 *margin_target,
			       u32 step_size);
u32 pmu_eye_margin_window_search(u8 *samples, u32 reserved_arg);
void pmu_flag_45c_clear(void);
void pmu_flag_45c_set(void);
void pmu_flag_46b_set(void);
u32 pmu_freq_delay_step_calc(u32 count, u32 unit);
u32 pmu_freq_ratio_mult(u32 mult);
u32 pmu_freq_scaled_timing_step_calc(void);
u8 pmu_get_46a(void);
u8 pmu_get_dmem_005(void);
u32 pmu_get_dmem_1c_bit(u32 word_idx, u32 bit_shift);
u16 pmu_get_phy_148(void);
u32 pmu_get_train_cfg96_bit4(void);
void pmu_hw_timer_delay(u32 count);
u32 pmu_is_lpddr4(void);
u32 pmu_is_lpddr5(void);
void pmu_mailbox_post_cmd_dispatch(u32 cmd);
int pmu_main_wrapper(void);
u8 pmu_map_mask(u8 val, u32 index);
s32 pmu_math_freq_scale_div(s32 val);
s8 pmu_midpoint_deviation_calc(u32 val);
void pmu_multiphase_pulse_seq_repeat(u32 phase_cmd, u32 param_val,
				     u32 slice, u32 count);
void pmu_multirank_slice_cal_dispatch(void *table_ptr,
				      u32 eval_error_mask, u32 skip_mask_low, s32 skip_mask_high,
				u32 ch_enable_mask, u32 check_settle_flags);
u16 pmu_phy_ac_lane_timing_delay_set(u32 lane);
void pmu_phy_analog_cfg_mode_set(u32 mode);
void pmu_phy_cal_strobe_latch_setup(void);
void pmu_phy_clk_gate_sync_pulse(void);
void pmu_phy_clk_phase_toggle_pulse(void);
void pmu_phy_dbyte_broadcast_enable(void);
void pmu_phy_dbyte_cfg_unpack(void);
u16 pmu_phy_delay_bound_margin_log(u32 slice);
void pmu_phy_lcdl_delay_read(u16 *out_arr, u32 flag, u32 mode);
void pmu_phy_mode_cfg_dispatch(u32 mode);
void pmu_phy_pll_cfg_quad_write_stream(u32 reg_offset, u16 p0,
				       u16 p1, u32 p2, u32 flags);
void pmu_phy_pll_clock_divider_align(void);
void pmu_phy_pll_lock_init(void);
void pmu_phy_profile_param_program(void);
u32 pmu_phy_query_operating_mode(void);
void pmu_phy_reg_176_strobe_pulse(u32 slice, u32 bit_idx);
void pmu_phy_reg_buffer_stream(void *buf, u32 count, u32 is_save);
void pmu_phy_reg_offset_diff_adjust(u32 rank);
void pmu_pll_clk_gate_cfg(void);
void pmu_pll_lock_freq_tier_cfg(void);
void pmu_pll_quad_param_tuple_stream(const u16 *param_array,
				     u16 *out_ctr, u16 *base_ptr);
void pmu_post_cmd_conditional_dispatch(u32 val);
void pmu_pulse_phy_140(void);
void pmu_pulse_phy_146(void);
void pmu_pulse_phy_f2(void);
u16 pmu_rank_margin_channel_accum(u32 rank, u32 start_slice,
				  u32 end_slice, u32 flags, u32 swap_mode);
void pmu_rank_mask_multiparam_dispatch(u32 channel, u32 rank,
				       u32 shadow_update);
void pmu_rank_shadow_status_sample(void);
void pmu_rank_slice_margin_eval(void);
int pmu_sample_margin_center_eval(const u8 *samples,
				  u8 *out_margins, u8 *out_mode);
void pmu_set_458_pair(u16 val);
void pmu_set_46a(u8 val);
void pmu_set_phy_44(u16 val);
void pmu_set_phy_46(u16 val);
void pmu_set_phy_4e(u16 val);
void pmu_slice_delay_step_program(u32 reg_base_off,
				  const s16 *step_deltas, u32 format_mode, u32 shift_sel,
				u32 stride_flag, u32 single_lane_flag);
void pmu_slice_deskew_pulse_seq(u32 enable);
void pmu_slice_lcdl_delay_collect(u32 param, void *out_buf, u32 count);
void pmu_slice_metric_scan_and_program(void);
void pmu_slice_status_b97_save(void);
void pmu_start(void);
u32 pmu_unpack_6(const u8 *p);
u32 pmu_unpack_7(const u8 *p);
void pmu_unpack_byte_pair(u32 val, u8 *out);
__section(".vectors") void pmu_vector_table(void);
void pmu_vref_signed_direction_pulse_dispatch(u32 slice_target,
					      u32 vref_step);
u32 pmu_window_centroid_calc(void *table_buf);
void pmu_write_phy_c2_quad(u16 v0);

#endif /* _SUNXI_PMU_FW_H */
