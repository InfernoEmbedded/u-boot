// SPDX-License-Identifier: GPL-2.0+
/*
 * Synopsys DesignWare DDR PHY Training Firmware (LPDDR5)
 * Host mailbox IPC, stage profiling, and diagnostic trace dumps
 * Target Microcontroller: Synopsys ARC EM4 (ARCv2 ISA, Code Density enabled)
 * SoC: Allwinner A733 (Sun60i) / LPDDR5 PHY (Type 9)
 */

#include "lpddr5_pmu_internal.h"

/**
 * pmu_mailbox_post_exec_wait() - Execute MicroContPost mailbox command and wait for completion
 *
 * Waits for busy bit 0 of 0x90180008 to clear, triggers execute (PHY_REG_MAILBOX_INT = 1),
 * and waits for completion acknowledge (bit 0 of 0x90180008 set).
 */
void pmu_mailbox_post_exec_wait(void)
{
	while (phy_read16(PHY_REG_MAILBOX_STAT) & 1)
		;
	phy_write16(PHY_REG_MAILBOX_INT, 1);
	while ((phy_read16(PHY_REG_MAILBOX_STAT) & 1) == 0)
		;
}

/**
 * pmu_mailbox_post_cmd32_send() - Send 32-bit command payload to POST mailbox and execute
 * @val: 32-bit command and argument payload written to mailbox registers
 */
void pmu_mailbox_post_cmd32_send(u32 val)
{
	phy_write16(PHY_REG_MAILBOX_STREAM, (u16)val);
	phy_write16(PHY_REG_MAILBOX_INT, 0);
	phy_write16(PHY_REG_MAILBOX_STREAM_HI, (u16)(val >> 16));
	pmu_mailbox_post_exec_wait();
}

/**
 * pmu_mailbox_post_cmd_dispatch() - Dispatch command byte to PHY CSR and trigger mailbox execution
 * @cmd: Command opcode byte to dispatch to PHY_REG_CMD_DISPATCH
 */
void pmu_mailbox_post_cmd_dispatch(u32 cmd)
{
	if (cmd == 255) {
		phy_write16(PHY_REG_CMD_DISPATCH, 4);
		phy_write16(PHY_REG_CMD_DISPATCH, 0);
	} else if (cmd == 7) {
		phy_write16(PHY_REG_CMD_DISPATCH, 1);
		phy_write16(PHY_REG_CMD_DISPATCH, 0);
	}
	phy_write16(PHY_REG_MAILBOX_STREAM, (u16)cmd);
	phy_write16(PHY_REG_MAILBOX_INT, 0);
	pmu_mailbox_post_exec_wait();
}

/**
 * pmu_post_cmd_conditional_dispatch() - Conditionally dispatch POST command based on sequence control
 * @val: POST mailbox command code to evaluate against sequence threshold
 */
void pmu_post_cmd_conditional_dispatch(u32 val)
{
	if (dmem_read8(PMU_DMEM_POST_THRESHOLD) <= PMU_POST_CMD_THRESH_200)
		pmu_mailbox_post_cmd_dispatch(val);
	pmu_phy_clk_gate_sync_pulse();
}

/**
 * pmu_cal_error_code_log() - Log calibration error code to POST mailbox
 */
void pmu_cal_error_code_log(void)
{
	u32 buf[20];
	memset(buf, 0, sizeof(buf));
	pmu_cal_metric_log(5, 0x2560000);
	pmu_slice_lcdl_delay_collect(0x4e, buf, 9);
	pmu_cal_matrix_trace_dump(0, 9, (const u16 *)buf);
}

/**
 * pmu_cal_metric_tag_dump() - Format and stream telemetry metric tag and argument payload to host
 * @tag: 32-bit telemetry diagnostic tag ([31:16] stage ID, [15:0] arg count / event)
 * @args: Pointer to array of 32-bit argument words
 */
void pmu_cal_metric_tag_dump(u32 tag, const u32 *args)
{
	if (dmem_read8(PMU_DMEM_CLK_GATE_FLAG) & 0x10)
		return;

	phy_write16(PHY_REG_MAILBOX_STREAM, 8);
	phy_write16(PHY_REG_MAILBOX_INT, 0);
	pmu_mailbox_post_exec_wait();

	pmu_mailbox_post_cmd32_send(tag);

	u16 count = (u16)tag;
	if (count == 0)
		return;

	for (u16 i = 0; i < count; i++) {
		pmu_mailbox_post_cmd32_send(args[i]);
	}
}

/**
 * pmu_cal_metric_log() - Filter and log diagnostic telemetry event
 * @level: Log severity and verbosity level
 * @tag: 32-bit telemetry diagnostic event tag
 * @...: Variable argument list matching tag parameter count
 */
__attribute__((noinline)) void pmu_cal_metric_log(u32 level, u32 tag, ...)
{
	(void)level;
	(void)tag;
}

/**
 * pmu_post_trace_log_halfwords() - Format and log halfword trace packet to host mailbox
 * @threshold: Minimum logging threshold severity
 * @cmd32: 32-bit command header word
 * @data: Pointer to array of 16-bit trace data halfwords
 * @count: Number of 16-bit halfwords in data array
 */
void pmu_post_trace_log_halfwords(u32 threshold, u32 cmd32, const u16 *data, u32 count)
{
	if (threshold < dmem_read8(PMU_DMEM_POST_THRESHOLD))
		return;
	if (dmem_read8(PMU_DMEM_CLK_GATE_FLAG) & 0x10)
		return;

	phy_write16(PHY_REG_MAILBOX_STREAM, 8);
	phy_write16(PHY_REG_MAILBOX_INT, 0);
	pmu_mailbox_post_exec_wait();

	pmu_mailbox_post_cmd32_send(cmd32);

	for (u32 i = 0; i < count; i++)
		pmu_mailbox_post_cmd32_send(data[i]);
}

/**
 * pmu_cal_matrix_trace_dump() - Format and stream calibration matrix traces across active slices
 * @event_tag: Telemetry event identifier logged to host mailbox
 * @num_cols: Number of columns in delay matrix
 * @matrix: Pointer to delay matrix array
 */
void pmu_cal_matrix_trace_dump(u32 event_tag, u32 num_cols, const u16 *matrix)
{
	pmu_cal_metric_log(5, 0x024b0001, event_tag);

	for (u32 col = 0; col < (u16)num_cols; col++) {
		const u16 *ptr = (const u16 *)((uintptr_t)matrix + (col << 1));
		u32 stride_bytes = num_cols << 1;
		u16 v0 = *ptr;
		ptr = (const u16 *)((uintptr_t)ptr + stride_bytes);
		u16 v1 = *ptr;
		ptr = (const u16 *)((uintptr_t)ptr + stride_bytes);
		u16 v2 = *ptr;
		ptr = (const u16 *)((uintptr_t)ptr + stride_bytes);
		u16 v3 = *ptr;

		pmu_cal_metric_log(5, 0x024c0005, col, v0, v1, v2, v3);
	}
}

/**
 * pmu_cal_profile_mode_get() - Retrieve active AC/PHY profile mode index from DMEM
 *
 * Return: AC profile mode index (0..3).
 */
u32 pmu_cal_profile_mode_get(void)
{
	u32 ch_offset = dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET);
	u16 reg_val = phy_read16((PHY_REG_DBYTE_BASE | 0x0012) + (ch_offset << 1));
	u32 mode = reg_val & 7;
	if (mode == 0)
		return 0;
	if (mode == 1)
		return 1;
	u32 bits = reg_val & 6;
	if (bits == 2)
		return 2;
	if (bits == 4)
		return 3;
	return 4;
}

/**
 * pmu_profile_mailbox_stream_send() - Read profile table from DMEM and stream 29 halfwords to mailbox
 * @channel: Target memory channel index (0 or 1)
 */
void pmu_profile_mailbox_stream_send(u32 channel)
{
	u8 offset_byte;
	u32 cmd;
	switch (channel) {
	case 3:
		offset_byte = 0x5d;
		cmd = 0x1dd001d;
		break;
	case 2:
		offset_byte = 0x5c;
		cmd = 0x1db001d;
		break;
	case 1:
		offset_byte = 0x5b;
		cmd = 0x1d9001d;
		break;
	case 0:
	default:
		offset_byte = 0x5a;
		cmd = 0x1d7001d;
		break;
	}

	uintptr_t table = PMU_DMEM_BASE | offset_byte;
	u16 buf[29];
	for (u32 i = 0; i < 29; i++) {
		buf[i] = *(const u8 *)(table + (i << 2));
	}

	pmu_post_trace_log_halfwords(0xa, cmd, buf, 29);
}

/**
 * pmu_cal_timing_packet_format() - Format 4-word timing calibration telemetry packet
 * @out: Output array receiving formatted 16-bit packet words
 * @tag: Telemetry event tag
 * @delay_code: Calibrated delay code
 * @status: Calibration completion status
 */
void pmu_cal_timing_packet_format(u16 *out, u32 tag, u32 delay_code, u32 status)
{
	u32 tag_shift = tag << 7;
	u32 delay_masked = (delay_code << 7) & 0x3f80;
	u32 status_shift = status << 14;
	u32 delay_half = delay_code >> 1;

	out[1] = 0;
	out[2] = 0;
	out[5] = 0;
	out[6] = 0;

	u32 packet_word4 = (delay_half & ~0x3f) | delay_masked | status_shift | (1u << 3);
	u32 packet_word3_7 = (status << 3) & 0x10;
	u32 packet_word0 = tag_shift | status_shift | 88;

	out[7] = (u16)packet_word3_7;
	out[3] = (u16)packet_word3_7;
	out[0] = (u16)packet_word0;
	out[4] = (u16)packet_word4;
}

/**
 * pmu_phy_slice_profile_update() - Update PHY slice profile registers from calibrated parameters
 */
void pmu_phy_slice_profile_update(void)
{
	u32 count = dmem_read32(PMU_DMEM_ACTIVE_SLICE_BITMAP);
	u32 param41c = dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET);
	u8 param_40 = dmem_read8(PMU_DMEM_CH1_RANK_EN);
	u8 param_25 = dmem_read8(PMU_DMEM_CH0_RANK_EN);
	u32 half_count = count >> 1;
	const u16 *p = (const u16 *)(PMU_DMEM_BASE | PMU_DMEM_2D_STEP_TABLE);

	for (u32 i = 0; i <= count; i++) {
		u16 val;
		if (param_40 != 0 && i >= half_count) {
			val = p[1];
			if (param_40 == 3 && p[3] > val)
				val = p[3];
		} else {
			val = p[0];
			if (param_25 == 3 && p[2] > val)
				val = p[2];
		}
		uintptr_t reg = PHY_REG_DBYTE_BASE | (((param41c | (i << 12)) << 1));
		phy_write16(reg, val);
	}
}

/**
 * pmu_slice_reg_query_mailbox_send() - Query slice registers across channels and stream to host mailbox
 * @mode: Operating mode filter (0 = all slices, non-zero = active slices)
 */
void pmu_slice_reg_query_mailbox_send(u32 mode)
{
	u8 start_slice = dmem_read8(PMU_DMEM_SLICE_START);
	u8 end_slice = dmem_read8(PMU_DMEM_SLICE_END);
	if (end_slice < start_slice)
		return;

	u32 sel = ((0xe4 >> ((mode << 1) & 6)) & 3) + 0x2a;
	u32 param41c = dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET);

	for (u32 slice = start_slice; slice <= end_slice; slice++) {
		uintptr_t reg = PHY_REG_DBYTE_BASE | (((param41c | (slice << 12) | sel) << 1));
		u16 val = phy_read16(reg);
		pmu_cal_metric_log(5, 0x2990004, slice, (u32)val, (val >> 6) & 0x1f, val & 0x3f);
	}
}

/**
 * pmu_profile_param_stride_cfg() - Configure profile parameter stride based on frequency mode
 * @mode: Frequency mode index determining stride offset
 */
void pmu_profile_param_stride_cfg(u32 mode)
{
	if (dmem_read16(PMU_DMEM_SEQUENCE_CTRL) == 1)
		return;

	u8 byte_idx = dmem_read8(PMU_DMEM_CAL_BYTE);
	u8 param_40 = dmem_read8(PMU_DMEM_CH1_RANK_EN);
	u16 val;

	if (mode == 3) {
		const u16 *tbl = (const u16 *)((PMU_DMEM_BASE | PMU_DMEM_2D_STEP_TABLE) + byte_idx * 4);
		val = tbl[0];
		if (param_40 != 0 && tbl[1] > val)
			val = tbl[1];
	} else {
		u32 offset = (mode & ~1u) + byte_idx * 4;
		val = *(const u8 *)((PMU_DMEM_BASE | PMU_DMEM_2D_STEP_TABLE) + offset);
	}

	u32 param41c = dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET);
	phy_write16((PHY_REG_MASTER_BASE | 0x001a) | (param41c << 1), (s8)val);

	u32 count = dmem_read32(PMU_DMEM_ACTIVE_SLICE_BITMAP);
	for (u32 i = 0; i <= count; i++) {
		uintptr_t reg = PHY_REG_DBYTE_BASE | (((param41c | (i << 12)) << 1));
		phy_write16(reg, (s8)val);
	}

	phy_write16(PHY_REG_PUB_CAL_STROBE, 1);
	pmu_hw_timer_delay(0x20);
	phy_write16(PHY_REG_PUB_CAL_STROBE, 0);
}

/**
 * pmu_profile_mailbox_cmd_dispatch() - Dispatch profile mailbox commands for target channel
 * @channel: Target memory channel index (0 or 1)
 */
void pmu_profile_mailbox_cmd_dispatch(u32 channel)
{
	if (channel < 1 || channel > 7)
		return;
	u32 cmd = 0x9d0003 + ((channel - 1) << 16);
	pmu_assert_or_halt(0, cmd);
}

/**
 * pmu_mailbox_param_eval_dispatch() - Evaluate mailbox parameter results and format status bytes
 * @out_val0: Output pointer for evaluated parameter byte 0
 * @out_val1: Output pointer for evaluated parameter byte 1
 * @out_val2: Output pointer for evaluated parameter byte 2
 * @rank: DRAM rank index (0..1)
 * @alt_eval: Mode flag (0 = standard evaluation, non-zero = alternate evaluation)
 */
void pmu_mailbox_param_eval_dispatch(u8 *out_val0, u8 *out_val1, u8 *out_val2, u32 rank, u32 alt_eval)
{
	u32 offset;

	if (rank == 4) {
		offset = 4;
	} else {
		u32 rank_pair = rank & ~1u;
		u32 computed_off = ((alt_eval == 0) ? 1 : 0) << 1;
		if (rank_pair == 2)
			computed_off += 1;
		offset = computed_off;
	}

	const u8 *tbl = (const u8 *)(PMU_DMEM_BASE | (0x118 + offset));
	u8 v_a = tbl[10];
	if (v_a != 0) {
		*out_val0 = tbl[0];
		*out_val1 = tbl[5];
		*out_val2 = v_a;
	}

	u32 check = pmu_dram_cfg_flag13_check(rank, alt_eval);
	u8 v0 = *out_val0;
	if (check != 0) {
		u32 sum = (u32)*out_val1 + v0;
		v0 = 0;
		if (sum > 127)
			sum = 127;
		*out_val1 = (u8)sum;
		*out_val0 = 0;
	}

	pmu_cal_metric_log(4, 0x1ba0004, v0, *out_val1, *out_val2, rank);
}

/**
 * pmu_phy_delay_bound_margin_log() - Calculate PHY delay boundary and log margin offset
 * @slice: DBYTE slice index (0..3)
 *
 * Return: Difference between delay boundary and logged margin.
 */
u16 pmu_phy_delay_bound_margin_log(u32 slice)
{
	u32 slice_reg = (slice << 13) | (PHY_REG_DBYTE_BASE | 0x0158);
	u16 val = phy_read16(slice_reg);
	phy_write16(slice_reg, val | 1);
	phy_write16(slice_reg, val & ~1);

	u32 csr_offset = dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET);
	u32 slice_csr = (slice << 12) | csr_offset;
	u32 delay_reg = (slice_csr << 1) | (PHY_REG_DBYTE_BASE | 0x015a);
	u16 delay_stage = phy_read16(delay_reg);

	u16 stage_offset = 0;
	if (delay_stage < 5) {
		u32 addr = (((delay_stage + 0xd0) | slice_csr) << 1) | PHY_REG_DBYTE_BASE;
		stage_offset = phy_read16(addr);
	}

	u32 period_reg = (slice << 13) | (PHY_REG_DBYTE_BASE | 0x01aa);
	u16 period_val = phy_read16(period_reg);

	u32 prod = (u32)period_val * (u32)delay_stage;
	u32 diff_prod = prod - stage_offset;
	u32 quot = (diff_prod << 6) / period_val;

	u32 bound = (dmem_read8(PMU_DMEM_FREQ_MODE) < 2) ? 160 : 288;
	u32 diff = bound - quot;

	u32 diff_coarse = (diff >> 6) & 0x3ff;
	pmu_cal_metric_log(4, 0x2790006, slice, diff_coarse, diff & 0x3f, (u16)quot, (u32)delay_stage, (u32)stage_offset);

	return (u16)diff;
}

/**
 * pmu_phy_profile_param_program() - Reload PHY slice profile registers across active slices
 */
void pmu_phy_profile_param_program(void)
{
	volatile u16 *phy_base = (volatile u16 *)PHY_REG_MASTER_BASE;
	__asm__("" : "+r"(phy_base));

	u16 orig_0f4 = phy_base[0x00f4 >> 1];
	u16 orig_0fc = phy_base[0x00fc >> 1];

	phy_base[0x00fc >> 1] = orig_0fc & 0xff33;
	phy_base[0x00f4 >> 1] = orig_0f4 | 0xcc;

	u8 flag = *(const volatile u8 *)(PMU_DMEM_BASE | PMU_DMEM_AC_PROFILE_EN);
	if (flag != 0) {
		pmu_cal_metric_log(5, 0x330000);
		u32 csr_offset = dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET);

		for (u32 slice = 0; slice < 2; slice++) {
			u32 slice_csr_off = ((slice << 12) | csr_offset) << 1;
			volatile u16 *slice_base = (volatile u16 *)(uintptr_t)(slice_csr_off | PHY_REG_AC_BASE);
			__asm__("" : "+r"(slice_base));

			const volatile u8 *step_cfg44 = (const volatile u8 *)((PMU_DMEM_BASE | PMU_DMEM_AC_STEP_WORDS) + slice * 2);
			u16 val0 = *(const volatile u16 *)step_cfg44;
			slice_base[0x0084 >> 1] = val0;
			slice_base[0x0086 >> 1] = val0;
			slice_base[0x0080 >> 1] = *(const volatile u16 *)(step_cfg44 + 0xc);
			slice_base[0x0082 >> 1] = *(const volatile u16 *)(step_cfg44 + 0x10);

			const volatile u8 *step_cfg48 = (const volatile u8 *)((PMU_DMEM_BASE | PMU_DMEM_AC_STEP_BYTES) + slice);
			slice_base[0x005c >> 1] = step_cfg48[0];
			slice_base[0x0066 >> 1] = step_cfg48[2];
			slice_base[0x0068 >> 1] = step_cfg48[4];
		}
	}

	phy_base[0x0084 >> 1] = 0;
	phy_base[0x0088 >> 1] = 0;
	pmu_delay_us(0x2af8, 0);

	phy_base[0x00f4 >> 1] = orig_0f4;
	phy_base[0x00fc >> 1] = orig_0fc;
}

/**
 * pmu_dq_lane_telemetry_bitpack() - Pack DQ lane telemetry status bits and stream diagnostic logs
 * @flags: Lane status bitmask across byte slices
 * @base: Base telemetry event tag identifier
 */
void pmu_dq_lane_telemetry_bitpack(u32 flags, u32 base)
{
	u8 rank = dmem_read8(PMU_DMEM_CAL_RANK);
	u8 stride = dmem_read8(PMU_DMEM_CAL_STRIDE);
	u32 rank_offset = (u32)rank * 1792;

	for (u32 lane = 0; lane < 7; lane++) {
		for (u32 f = 0; f < 2; f++) {
			if (!(flags & (1 << f)))
				continue;
			u32 tag_base = 0xb00000 + f * 0x30000;
			pmu_cal_metric_log(4, tag_base + 1, lane);
			for (u32 offset = 0; offset < 256; offset += (u32)stride * 8) {
				const volatile u8 *p = (const volatile u8 *)(rank_offset + base + offset);
				u32 val = 0;
				for (s32 bit = 7; bit >= 0; bit--) {
					val |= (u32)(*p) << bit;
					p += stride;
				}
				pmu_cal_metric_log(4, tag_base + 0x10001, (u8)val);
			}
			pmu_cal_metric_log(4, tag_base + 0x20000);
		}
		base += 256;
	}
}

/**
 * pmu_ac_lane_profile_setup() - Configure AC lane profile registers at PHY_REG_AC_PROFILE_BASE
 */
void pmu_ac_lane_profile_setup(void)
{
	volatile u16 *phy_base = (volatile u16 *)PHY_REG_MASTER_BASE;
	__asm__("" : "+r"(phy_base));

	u16 orig_0f4 = phy_base[0x00f4 >> 1];
	u16 orig_0fc = phy_base[0x00fc >> 1];

	phy_base[0x00fc >> 1] = 0;
	phy_base[0x00f4 >> 1] = 0xff;

	u8 cal_status = dmem_read8(PMU_DMEM_SEARCH_WINDOW_STEPS);
	phy_base[0x0084 >> 1] = (u16)cal_status | 0x100;

	u8 lane_status = dmem_read8(PMU_DMEM_LANE_STATUS);
	const volatile u8 *ac_profile = (const volatile u8 *)(PMU_DMEM_BASE | PMU_DMEM_AC_PROFILE_BASE);
	__asm__("" : "+r"(ac_profile));
	u8 flag = ac_profile[4];
	s32 delay_diff = (s32)(lane_status + cal_status) - 2;

	if (flag != 0) {
		u16 f8 = *(const volatile u16 *)(ac_profile + 8);
		u16 fa = *(const volatile u16 *)(ac_profile + 10);
		u16 fc = *(const volatile u16 *)(ac_profile + 12);
		u16 fe = *(const volatile u16 *)(ac_profile + 14);
		u8 f7 = ac_profile[7];
		u8 f5 = ac_profile[5];
		u8 f6 = ac_profile[6];
		pmu_cal_metric_log(5, 0x310007, f8, fa, fc, fe, f7, f5, f6);
		volatile u16 *target_phy = (volatile u16 *)(uintptr_t)((dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET) << 1) | PHY_REG_AC_PROFILE_BASE);
		__asm__("" : "+r"(target_phy));
		target_phy[0x84 >> 1] = f8;
		target_phy[0x86 >> 1] = fa;
		target_phy[0x80 >> 1] = fc;
		target_phy[0x82 >> 1] = fe;
		target_phy[0x5c >> 1] = f7;
		target_phy[0x66 >> 1] = f5;
		target_phy[0x68 >> 1] = f6;
	}

	if (delay_diff < 0)
		delay_diff += (u32)cal_status * dmem_read8(PMU_DMEM_DRAM_TYPE);

	phy_base[0x0088 >> 1] = (u16)delay_diff;
	pmu_delay_us(0x2af8, 0);

	phy_base[0x00f4 >> 1] = orig_0f4;
	phy_base[0x00fc >> 1] = orig_0fc;

	dmem_write8(PMU_DMEM_AC_STEP_CFG, ((cal_status >> 1) & ~1) - 1);
}

/**
 * pmu_cmd_code_map_log() - Map command opcode and subcode into telemetry event and log to host
 * @cmd_code: Command opcode identifier
 * @sub_code: Sub-command code or modifier
 */
void pmu_cmd_code_map_log(u32 cmd_code, u32 sub_code)
{
	u8 map_code = 9;

	if (sub_code == 0) {
		u32 masked = cmd_code & 0xfff;
		if (masked >= 257 && masked <= 261)
			map_code = masked - 257 + 2;
		else if (masked == 272)
			map_code = 7;
		else if (masked == 512)
			map_code = 8;
	} else if (sub_code == 1) {
		map_code = 0;
	} else if (sub_code == 2) {
		map_code = 1;
	} else if (sub_code == 4 || sub_code == 17) {
		map_code = 2;
	} else if (sub_code == 18) {
		map_code = 3;
	} else if (sub_code == 8 || sub_code == 19) {
		map_code = 4;
	} else if (sub_code == 20) {
		map_code = 5;
	} else if (sub_code == 21) {
		map_code = 6;
	} else if (sub_code == 26) {
		map_code = 7;
	} else if (sub_code == 0x20) {
		map_code = 8;
	} else {
		pmu_assert_or_halt(0, 0x2190001);
		map_code = dmem_read8(PMU_DMEM_FREQ_MODE);
	}

	dmem_write8(PMU_DMEM_FREQ_MODE, map_code);
	pmu_cal_metric_log(10, 0x21a0003, map_code, cmd_code, sub_code);
}

/**
 * pmu_lcdl_delay_profile_update() - Update LCDL delay profile registers and log telemetry event
 * @mode: Operating mode selector
 * @unused: Unused alignment parameter
 * @base_code: Base telemetry event code
 * @tag: Telemetry tag modifier
 */
void pmu_lcdl_delay_profile_update(u32 mode, u32 unused, u32 base_code, u32 tag)
{
	u16 buf[40];
	memset(buf, 0, sizeof(buf));

	if (mode == 2) {
		pmu_cal_metric_log(5, 0x24f0001, tag);
		pmu_phy_lcdl_delay_read(buf, 0, tag);
		pmu_cal_matrix_trace_dump(0, 9, buf);
		return;
	}

	u32 regs[7];
	u32 counts[7];
	memset(regs, 0, sizeof(regs));
	memset(counts, 0, sizeof(counts));
	u32 num_entries;

	if (mode != 0) {
		pmu_cal_metric_log(5, 0x2520001, tag);
		regs[0] = base_code + 0x2a;
		regs[1] = base_code + 0x26;
		regs[2] = base_code + 0x1000020;
		regs[3] = base_code + 0x28;
		counts[0] = 1;
		counts[1] = 9;
		counts[2] = 1;
		counts[3] = 1;
		num_entries = 4;
	} else {
		pmu_cal_metric_log(5, 0x2500001, tag);
		regs[0] = base_code + 0x20;
		regs[1] = base_code + 0x10;
		regs[2] = base_code + 0x12;
		regs[3] = 0x4e;
		regs[4] = 0x4f;
		regs[5] = 0x50;
		regs[6] = 0x51;
		counts[0] = 2;
		counts[1] = 9;
		counts[2] = 9;
		counts[3] = 9;
		counts[4] = 9;
		counts[5] = 9;
		counts[6] = 9;
		num_entries = 7;
	}

	u32 ch_offset = dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET);
	for (u32 fp = 0; fp < num_entries; fp++) {
		memset(buf, 0, sizeof(buf));
		pmu_slice_lcdl_delay_collect(ch_offset | regs[fp], buf, counts[fp]);
		pmu_cal_matrix_trace_dump(fp, counts[fp], buf);
	}
}

/**
 * pmu_slice_metric_scan_and_program() - Scan slice calibration metrics and program delay registers
 */
void pmu_slice_metric_scan_and_program(void)
{
	pmu_cal_metric_log(4, 0x1640000);

	u8 start_slice = dmem_read8(PMU_DMEM_SLICE_START);
	u8 end_slice = dmem_read8(PMU_DMEM_SLICE_END);
	u16 num_ranks = dmem_read16(PMU_DMEM_ITER_COUNT);
	u32 ch_offset = dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET);

	volatile u16 *phy_base = (volatile u16 *)PHY_REG_DBYTE_BASE;
	__asm__("" : "+r"(phy_base));

	for (u32 slice = start_slice; slice <= end_slice; slice++) {
		const volatile u16 *slice_metrics = (const volatile u16 *)((PMU_DMEM_BASE | 0xe798) + (slice * 8));
		u16 max_val = slice_metrics[0];
		u16 min_val = slice_metrics[0];

		for (u32 k = 0; k < num_ranks; k++) {
			u16 val = slice_metrics[k];
			pmu_cal_metric_log(4, 0x1650003, slice, val);
			if (val > max_val)
				max_val = val;
			if (val < min_val)
				min_val = val;
		}

		pmu_cal_metric_log(4, 0x1660002, min_val, max_val);

		u32 midpoint = (max_val + min_val) >> 1;
		u32 diff = 12 - midpoint;
		u16 diff_u16 = (u16)diff;

		pmu_cal_metric_log(4, 0x1670002, midpoint, diff_u16);

		u32 reg_val_r30 = ((diff & 0xffff) << 4) | midpoint;
		u32 reg_val_fp = (diff & 0xffff) | (midpoint << 4);
		u32 slice_offset = (slice << 12) | ch_offset;

		for (u32 bank = 0; bank < 2; bank++) {
			u32 reg_low = (bank != 0) ? 0x5e : 0x62;
			u32 reg_high = (bank != 0) ? 0x60 : 0x64;

			u16 w1 = *(volatile u16 *)((uintptr_t)phy_base + ((reg_low | ch_offset) << 1));
			u16 w2 = *(volatile u16 *)((uintptr_t)phy_base + ((reg_high | ch_offset) << 1));
			w1 = (w1 & 0x300) | reg_val_r30;
			w2 = (w2 & 0x300) | reg_val_fp;

			for (u32 blink = 0; blink < 2; blink++) {
				*(volatile u16 *)((uintptr_t)phy_base + (((blink + reg_low) | slice_offset) << 1)) = w1;
				*(volatile u16 *)((uintptr_t)phy_base + (((blink + reg_high) | slice_offset) << 1)) = w2;
			}
		}
	}
}

/**
 * pmu_cal_metric_table_log() - Log calibration metric table entries to host mailbox
 * @base_addr: Base address of metric table in DMEM
 * @param_mode: Primary calibration telemetry parameter mode
 * @channel: Memory channel index (0 or 1)
 * @lane_mask: Bitmask of active bit lanes to log (0..9)
 */
void pmu_cal_metric_table_log(u32 base_addr, u32 param_mode, u32 channel, u32 lane_mask)
{
	u32 fp_limit;
	u32 metric_val;
	u32 accum_total;
	u8 field_f;
	u8 field_e;

	pmu_cal_metric_log(0xc8, 0x6a0000);

	fp_limit = pmu_cal_rank_stride_count_get(param_mode);
	metric_val = pmu_cal_profile_mode_get();

	if (channel != 0)
		pmu_cal_metric_log(0xc8, 0x6c0000);
	else
		pmu_cal_metric_log(0xc8, 0x6b0000);

	pmu_cal_channel_status_get(channel, &accum_total, &field_f, &field_e);

	if (param_mode != 0) {
		pmu_cal_metric_log(0xc8, 0x730000);
	} else {
		pmu_cal_metric_log(0xc8, 0x6d0000);
		switch (metric_val) {
		case 3:
			pmu_cal_metric_log(0xc8, 0x710000);
			fp_limit = 2;
			break;
		case 1:
			pmu_cal_metric_log(0xc8, 0x6f0000);
			fp_limit = 1;
			break;
		case 2:
			pmu_cal_metric_log(0xc8, 0x700000);
			fp_limit = 1;
			break;
		case 0:
			pmu_cal_metric_log(0xc8, 0x6e0000);
			fp_limit = 1;
			break;
		default:
			pmu_assert_or_halt(0, 0x720001);
			break;
		}
	}

	pmu_cal_metric_log(0xc8, 0x740005, accum_total, pmu_cal_profile_mode_get(), (u32)field_f, (u32)field_e, lane_mask);

	u8 dmem_20 = dmem_read8(PMU_DMEM_CH0_CTRL_20);
	u8 dmem_21 = dmem_read8(PMU_DMEM_CH0_CTRL_21);
	u8 dmem_17 = dmem_read8(PMU_DMEM_TRAIN_CTRL_17);
	u8 dmem_18 = dmem_read8(PMU_DMEM_TRAIN_CTRL_18);
	u8 dmem_25 = dmem_read8(PMU_DMEM_CH0_RANK_EN);
	u8 dmem_40 = dmem_read8(PMU_DMEM_CH1_RANK_EN);

	pmu_cal_metric_log(0xc8, 0x750008, dmem_20, dmem_21, dmem_17, dmem_18, dmem_25, dmem_40, 10, 4);

	if ((s32)fp_limit <= 0)
		goto exit_log;

	for (u32 fp = 0; fp < fp_limit; fp++) {
		for (u32 rank = 0; rank < accum_total; rank++) {
			for (u32 slice = field_f; slice <= field_e; slice++) {
				for (u32 lane = 0; lane < 10; lane++) {
					if (!(lane_mask & (1U << lane)))
						continue;

					pmu_cal_metric_log(0xc8, 0x760004, fp, rank, slice, lane);

					u32 entry_addr = base_addr +
							 fp * 10560 +
							 rank * 5280 +
							 slice * 1320 +
							 lane * 132;
					const u8 *entry = (const u8 *)(uintptr_t)entry_addr;

					for (u32 i = 0; i < 64; i++) {
						u8 byte0 = entry[2 * i];
						u8 byte1 = entry[2 * i + 1];

						pmu_cal_metric_log(0xc8, 0x770003, byte0, byte1, i);
					}

					u16 val_128 = *(const u16 *)(entry + 128);
					s8 val_130 = *(const s8 *)(entry + 130);
					u8 val_131 = *(const u8 *)(entry + 131);

					pmu_cal_metric_log(0xc8, 0x780003, val_128, (s32)val_130, (u32)val_131);
				}
			}
		}
	}

exit_log:
	pmu_cal_metric_log(0xc8, 0x790000);
}

/**
 * pmu_cal_metric_table_sweep_log() - Sweep calibration metric table and log telemetry packets
 * @base_addr: Base address of metric table in DMEM
 * @sweep_start: Sweep start boundary index
 * @sweep_end: Sweep end boundary index
 *
 * Return: Calibration sweep log status code.
 */
u32 pmu_cal_metric_table_sweep_log(u32 base_addr, u32 sweep_start, u32 sweep_end)
{
	u32 num_outer;
	u8  val_b6b;
	u8  val_025;
	u8  temp_85;
	u32 val_58;
	u32 sweep_accum;
	s32 max_limit;

	num_outer = pmu_cal_rank_stride_count_get(sweep_start);

	val_b6b = dmem_read8(PMU_DMEM_RANK_BOUNDARY);
	val_025 = dmem_read8(PMU_DMEM_CH0_RANK_EN);
	temp_85 = val_b6b;

	pmu_cal_channel_status_get(1, &val_58, &temp_85, &temp_85);

	if ((s32)num_outer > 0) {
		sweep_accum = (val_025 == 3) ? 2 : 1;
		max_limit = ((s32)sweep_accum > (s32)val_58) ? (s32)sweep_accum : (s32)val_58;

		for (u32 outer_i = 0; (s32)outer_i < (s32)num_outer; outer_i++) {
			if (max_limit <= 0)
				continue;

			for (u32 j = 0; (s32)j < max_limit; j++) {
				pmu_cal_metric_log(4, (sweep_start != 0) ? 0x840003 : 0x830003, j, j, 0xff);

				for (u32 fp = 0; fp < 2; fp++) {
					u32 val_5c;
					u8  min_k;
					u8  max_k;

					if (fp == 0 && j >= sweep_accum)
						continue;
					if (fp == 1 && (s32)j >= (s32)val_58)
						continue;

					pmu_cal_channel_status_get(fp, &val_5c, &min_k, &max_k);

					if (min_k > max_k)
						continue;

					for (u32 k = min_k; k <= max_k; k++) {
						for (u32 lane = 0; lane < 10; lane++) {
							if (!(sweep_end & (1U << lane)))
								continue;

							u32 lane_base = base_addr +
											outer_i * 10560 +
											j * 5280 +
											k * 1320 +
											lane * 132;

							u16 val_128 = *(volatile const u16 *)(uintptr_t)(lane_base + 128);
							s8  val_130 = *(volatile const s8  *)(uintptr_t)(lane_base + 130);
							u8  val_131 = *(volatile const u8  *)(uintptr_t)(lane_base + 131);

							pmu_cal_metric_log(4, 0x820005, k, lane,
									(u32)val_128, (s32)val_130, (u32)val_131);

							for (int sub = 0; sub < 2; sub++) {
								u16 buffer[64];

								if (dmem_read8(PMU_DMEM_CAL_SELECT_FLAGS) & 0x02) {
									const u8 *src = (const u8 *)(uintptr_t)(lane_base + (1 - sub));

									for (int s = 0; s < 64; s++) {
										buffer[s] = src[s * 2];
									}
									pmu_post_trace_log_halfwords(4, 0x7a0040, buffer, 64);
								} else {
									const u8 *src = (const u8 *)(uintptr_t)(lane_base + (5 - sub));

									for (int s = 0; s < 31; s++) {
										buffer[s] = src[s * 4];
									}
									pmu_post_trace_log_halfwords(4, 0x7e001f, buffer, 31);
								}
							}
						}
					}
				}
			}
		}
	}

	if (dmem_read8(PMU_DMEM_DRAM_CFG_FLAGS) & 0x01) {
		pmu_cal_metric_table_log(base_addr, sweep_start, 0, sweep_end);
		pmu_cal_metric_table_log(base_addr, sweep_start, 1, sweep_end);
	}

	return 0;
}
