// SPDX-License-Identifier: GPL-2.0+
/*
 * Synopsys DesignWare DDR PHY Training Firmware (LPDDR5)
 * Firmware entry, training stage orchestration, and execution dispatcher
 * Target Microcontroller: Synopsys ARC EM4 (ARCv2 ISA, Code Density enabled)
 * SoC: Allwinner A733 (Sun60i) / LPDDR5 PHY (Type 9)
 */

#include "lpddr5_pmu_internal.h"

/**
 * pmu_cal_search_win_init() - Initialize 2D calibration search window buffer
 * @buf: Pointer to 2D search window buffer in DMEM to initialize with 0xff sentinels
 */
void pmu_cal_search_win_init(u8 *buf)
{
	buf[0x00] = 0xff;
	buf[0x04] = 0xff;
	buf[0x1b] = 0xff;
	buf[0x1f] = 0xff;
	buf[0x01] = 0xff;
	buf[0x05] = 0xff;
	buf[0x1c] = 0xff;
	buf[0x20] = 0xff;
}

/**
 * pmu_cal_marker_a_clear() - Clear calibration tracker marker A in DMEM
 */
void pmu_cal_marker_a_clear(void)
{
	dmem_write16(PMU_DMEM_CAL_MARKER_A, 0);
}

/**
 * pmu_cal_mode_mask_test() - Test if calibration mode bit is set in configuration mask
 * @val: 32-bit calibration mode bitmask to test
 *
 * Return: 1 if calibration mode is supported, 0 otherwise.
 */
u32 pmu_cal_mode_mask_test(u32 val)
{
	if (val <= 0xe) {
		if ((1U << val) & 0x4041)
			return 1;
	}
	return (val == 0x28) ? 1 : 0;
}

/**
 * pmu_cal_markers_set() - Set calibration tracker markers A and B in DMEM
 * @val: 16-bit marker value stored into both marker A and B fields
 */
void pmu_cal_markers_set(u16 val)
{
	dmem_write16(PMU_DMEM_CAL_MARKER_A, val);
	dmem_write16(PMU_DMEM_CAL_MARKER_B, val);
}

/**
 * pmu_dmem_timing_mode_check() - Check if timing calibration mode is active in DMEM
 *
 * Return: True (1) if timing mode is active, false (0) otherwise.
 */
u32 pmu_dmem_timing_mode_check(void)
{
	return (dmem_read8(PMU_DMEM_TIMING_MODE_CTRL) & PMU_TIMING_MODE_MASK) == 1;
}

/**
 * pmu_dmem_wck_sync_mode_check() - Check if WCK synchronization mode is enabled in DMEM
 *
 * Return: True (1) if WCK sync mode is enabled, false (0) otherwise.
 */
u32 pmu_dmem_wck_sync_mode_check(void)
{
	return (dmem_read8(PMU_DMEM_WCK_SYNC_MODE) & PMU_WCK_SYNC_MODE_MASK) == 0;
}

/**
 * pmu_dq_phy_to_logical_mask() - Deswizzle physical DBYTE PHY pin mask to logical DQ byte
 * @phy_mask: Physical DBYTE receiver pin bitmask
 * @map_idx: DQ pin remap descriptor table index (slice/channel)
 *
 * Return: Remapped logical DQ bitmask.
 */
u8 pmu_dq_phy_to_logical_mask(u32 phy_mask, u32 map_idx)
{
	const u8 *base = (const u8 *)(uintptr_t)(PMU_DMEM_DQ_PIN_MAP_BASE + (map_idx * PMU_DQ_PIN_MAP_ENTRY_SIZE));
	u8 res = base[0] & (u8)phy_mask;
	u8 count = base[1];
	const u8 *pairs = base + 2;

	for (u32 i = 0; i < count; i++) {
		u8 test_mask = pairs[2 * i];
		u8 bit_pos = pairs[2 * i + 1];
		if (test_mask & phy_mask)
			res |= (1U << bit_pos);
	}
	return res;
}

/**
 * pmu_cal_status_148_read() - Read 16-bit calibration status register
 *
 * Return: 16-bit calibration status register value.
 */
u16 pmu_cal_status_148_read(void)
{
	return phy_read16(PHY_REG_CAL_STATUS);
}

/**
 * pmu_dmem_pll_bypass_read() - Read PLL bypass configuration byte from DMEM
 *
 * Return: 8-bit PLL bypass configuration byte from DMEM.
 */
u8 pmu_dmem_pll_bypass_read(void)
{
	return dmem_read8(PMU_DMEM_PLL_BYPASS);
}

/**
 * pmu_cal_strobe_pulse() - Generate active-high calibration trigger strobe pulse
 */
void pmu_cal_strobe_pulse(void)
{
	phy_write16(PHY_REG_CAL_TRIG, 1);
	pmu_hw_timer_delay(PMU_CAL_STROBE_TICKS);
	phy_write16(PHY_REG_CAL_TRIG, 0);
}

/**
 * pmu_cal_strobe_secondary_pulse() - Generate active-high secondary calibration strobe pulse
 */
void pmu_cal_strobe_secondary_pulse(void)
{
	phy_write16(PHY_REG_PUB_CAL_STROBE, 1);
	pmu_hw_timer_delay(PMU_CAL_STROBE_TICKS);
	phy_write16(PHY_REG_PUB_CAL_STROBE, 0);
}

/**
 * pmu_dmem_active_lane_query() - Query active lane bit for given channel and bit position
 * @channel: Memory channel index (0 or 1)
 * @bit_pos: Target bit position within active lane mask
 *
 * Return: 1 if the specified lane is active, 0 otherwise.
 */
u32 pmu_dmem_active_lane_query(u32 channel, u32 bit_pos)
{
	u32 val = dmem_read8(PMU_DMEM_ACTIVE_LANES);
	u32 shift = (channel << 1) + bit_pos;
	return (val >> shift) & 1;
}

/**
 * pmu_cal_param_table_write() - Write 16-bit calibration parameter to 2D table in DMEM
 * @row: Table row index (rank or slice index)
 * @col: Table column index (parameter tap index)
 * @val: 16-bit calibration value to store
 */
void pmu_cal_param_table_write(u32 row, u32 col, u16 val)
{
	dmem_write16(PMU_DMEM_2D_STEP_TABLE + (row << 2) + (col << 1), val);
}

/**
 * pmu_dram_cfg_flag13_check() - Test DRAM configuration flag bit 13
 * @rank: DRAM rank index (0..1, 4 = bypass check)
 * @flag: Calibration mode flag (0 = check, non-zero = bypass)
 *
 * Return: 1 if DRAM config flag bit 13 is set, 0 otherwise.
 */
u32 pmu_dram_cfg_flag13_check(u32 rank, u32 flag)
{
	if (rank == 4 || flag != 0)
		return 0;
	return (dmem_read16(PMU_DMEM_DRAM_CFG_FLAGS) >> PMU_CFG_FLAG_BIT13_SHIFT) & 1;
}

/**
 * pmu_cal_marker_verify() - Verify calibration target markers and update tracker if mismatched
 *
 * Return: Tracker step update result, or 0 if markers match.
 */
u32 pmu_cal_marker_verify(void)
{
	if (dmem_read16(PMU_DMEM_CAL_MARKER_A) == dmem_read16(PMU_DMEM_CAL_MARKER_B))
		return 0;
	return pmu_cal_tracker_step_update();
}

/**
 * pmu_cal_handler_select() - Select and execute calibration handler for target buffer
 * @buf_addr: Target calibration buffer address in DMEM
 *
 * Return: Calibration evaluation status code (0 on success, non-zero error).
 */
u32 pmu_cal_handler_select(u32 buf_addr)
{
	if (dmem_read8(PMU_DMEM_CAL_SELECT_FLAGS) & 4)
		return pmu_window_centroid_calc((u8 *)(uintptr_t)buf_addr);
	return pmu_eye_margin_window_search((u8 *)(uintptr_t)buf_addr, 0);
}

/**
 * pmu_cal_param_table_init() - Initialize 2D calibration parameter table in DMEM
 */
void pmu_cal_param_table_init(void)
{
	dmem_write32(PMU_DMEM_2D_STEP_TABLE, PMU_CAL_DEFAULT_PARAM);
	dmem_write32(PMU_DMEM_2D_STEP_RANK1, PMU_CAL_DEFAULT_PARAM);
}

/**
 * pmu_dq_swap_query() - Query DQ/DQS swap polarity bit for slice and line
 * @slice: DBYTE slice index (0..3)
 * @bit_idx: Target DQ bit index (0..7)
 *
 * Return: Polarity bit for the given slice and DQ line.
 */
u32 pmu_dq_swap_query(u32 slice, u32 bit_idx)
{
	u8 boundary = dmem_read8(PMU_DMEM_RANK_BOUNDARY);
	u8 mask = dmem_read8(PMU_DMEM_DQ_SWAP_MASK);
	u32 bit_val = bit_idx & 1;
	u32 cond = (boundary < bit_idx) ? 1 : 0;
	slice += cond * 2;
	if ((mask & (1 << slice)) == 0)
		bit_val ^= 1;
	return bit_val;
}

/**
 * pmu_ashl64() - 64-bit logical shift left
 * @val: 64-bit integer value to shift
 * @shift: Shift count (masked to 0..63)
 *
 * Return: 64-bit value shifted left by shift bits.
 */
u64 pmu_ashl64(u64 val, u32 shift)
{
	shift &= 0x3f;
	if (!shift)
		return val;
	union {
		u64 q;
		struct {
			u32 lo;
			u32 hi;
		} s;
	} in, out;
	in.q = val;
	if (shift < 32) {
		out.s.hi = (in.s.hi << shift) | (in.s.lo >> (32 - shift));
		out.s.lo = in.s.lo << shift;
	} else {
		out.s.hi = in.s.lo << (shift - 32);
		out.s.lo = 0;
	}
	return out.q;
}

/**
 * pmu_phy_cfg_target_set() - Program target analog PHY configuration registers based on mode
 * @mode: Operating mode selector (determines whether DMEM offset 0x25 or 0x40 is applied)
 */
void pmu_phy_cfg_target_set(u32 mode)
{
	u8 param_25 = (mode != 1) ? dmem_read8(PMU_DMEM_CH0_RANK_EN) : 0;
	u8 param_40 = (mode != 0) ? dmem_read8(PMU_DMEM_CH1_RANK_EN) : 0;
	phy_write16(PHY_REG_ANALOG_CFG0, param_25);
	phy_write16(PHY_REG_ANALOG_CFG1, param_40);
}

/**
 * pmu_cal_status_query() - Query calibration status flag using deskew configuration
 * @check_mode: Mode check selector (0 for baseline, non-zero for alternate mode)
 * @status_sel: Calibration status register selector
 *
 * Return: Calibration status flag (1 if active/valid, 0 otherwise).
 */
u32 pmu_cal_status_query(u32 check_mode, u32 status_sel)
{
	u8 val_e7 = dmem_read8(PMU_DMEM_TRAIN_STATUS);
	s8 val_62;

	if (val_e7 & 1)
		return 1;
	if (val_e7 & 2)
		return 0;

	val_62 = (s8)dmem_read8(PMU_DMEM_VREF_FLAG_SIGNED);
	status_sel &= ~1;
	if (status_sel == 2)
		return (check_mode != 0 && val_62 < 0) ? 1 : 0;
	return (val_62 >> 6) & 1;
}

/**
 * pmu_phy_cal_state_save() - Save active PHY calibration registers to DMEM archive
 */
void pmu_phy_cal_state_save(void)
{
	u32 slice = dmem_read32(PMU_DMEM_ACTIVE_SLICE_IDX);
	u32 reg = PHY_REG_CAL_STAT_CTRL + (slice * 2);
	u16 val = phy_read16(reg);
	dmem_write16(PMU_DMEM_SAVED_CAL_STAT, val);
	phy_write16(reg, val & 0xff9f);

	if (dmem_read8(PMU_DMEM_CH0_RANK_EN) != 0) {
		u8 start = dmem_read8(PMU_DMEM_RANK0_SLICE_START);
		u8 end = dmem_read8(PMU_DMEM_RANK_BOUNDARY);
		for (u32 i = start; i <= end; i++) {
			u16 r = phy_read16(PHY_REG_WCK_DELAY + (i << 13));
			dmem_write16(PMU_DMEM_SAVED_WCK_DELAY + (i * 2), r);
		}
	}

	if (dmem_read8(PMU_DMEM_CH1_RANK_EN) != 0) {
		u8 start = dmem_read8(PMU_DMEM_RANK1_SLICE_START);
		u8 end = dmem_read8(PMU_DMEM_RANK1_SLICE_END);
		for (u32 i = start; i <= end; i++) {
			u16 r = phy_read16(PHY_REG_WCK_DELAY + (i << 13));
			dmem_write16(PMU_DMEM_SAVED_WCK_DELAY + (i * 2), r);
		}
	}
}

/**
 * pmu_phy_cal_state_restore() - Restore active PHY calibration registers from DMEM archive
 */
void pmu_phy_cal_state_restore(void)
{
	u32 slice = dmem_read32(PMU_DMEM_ACTIVE_SLICE_IDX);
	u32 reg = PHY_REG_CAL_STAT_CTRL + (slice * 2);
	u16 val = dmem_read16(PMU_DMEM_SAVED_CAL_STAT);
	phy_write16(reg, val);

	if (dmem_read8(PMU_DMEM_CH0_RANK_EN) != 0) {
		u8 start = dmem_read8(PMU_DMEM_RANK0_SLICE_START);
		u8 end = dmem_read8(PMU_DMEM_RANK_BOUNDARY);
		for (u32 i = start; i <= end; i++) {
			u16 r = dmem_read16(PMU_DMEM_SAVED_WCK_DELAY + (i * 2));
			phy_write16(PHY_REG_WCK_DELAY + (i << 13), r);
		}
	}

	if (dmem_read8(PMU_DMEM_CH1_RANK_EN) != 0) {
		u8 start = dmem_read8(PMU_DMEM_RANK1_SLICE_START);
		u8 end = dmem_read8(PMU_DMEM_RANK1_SLICE_END);
		for (u32 i = start; i <= end; i++) {
			u16 r = dmem_read16(PMU_DMEM_SAVED_WCK_DELAY + (i * 2));
			phy_write16(PHY_REG_WCK_DELAY + (i << 13), r);
		}
	}
}

/**
 * pmu_inactive_slices_clear() - Clear LCDL status CSR for inactive byte slices
 */
void pmu_inactive_slices_clear(void)
{
	u8 num_slices = dmem_read8(PMU_DMEM_ACTIVE_SLICE_COUNT);
	u8 start = dmem_read8(PMU_DMEM_SLICE_START);
	u8 end = dmem_read8(PMU_DMEM_SLICE_END);

	for (u32 i = 0; i < num_slices; i++) {
		if (i < start || i > end) {
			u32 reg = PHY_REG_DBYTE_BASE + PHY_REG_DBYTE_LCDL_CTRL + (i * PHY_REG_DBYTE_STRIDE);
			phy_write16(reg, 0);
		}
	}
}

/**
 * pmu_dmem_training_flags_eval() - Evaluate training stage flags from DMEM configuration
 *
 * Return: Combined 8-bit training evaluation flag mask.
 */
u32 pmu_dmem_training_flags_eval(void)
{
	u8 cal_flag_62 = dmem_read8(PMU_DMEM_VREF_FLAG_SIGNED);
	u8 cal_param_72 = dmem_read8(PMU_DMEM_VREF_DAC_CTRL);
	u8 param_96 = dmem_read8(PMU_DMEM_TRAIN_PARAM_CTRL);

	u32 flag_62_b7 = (cal_flag_62 >> 7) & 1;
	u32 flag_62_b6 = (cal_flag_62 >> 6) & 1;
	u32 flag_96_b6 = (param_96 >> 6) & 1;
	u32 flag_96_b4 = (param_96 >> 4) & 1;
	u32 flag_72_b5 = ((cal_param_72 >> 5) & 1) ? 0 : 1;

	return (flag_62_b6 | flag_62_b7 | flag_96_b6 | flag_96_b4 | flag_72_b5) & 0xff;
}

/**
 * pmu_cal_tracker_step_update() - Update calibration tracker and advance step divisor
 *
 * Return: Updated tracker marker value.
 */
u32 pmu_cal_tracker_step_update(void)
{
	volatile u16 *tracker = (volatile u16 *)(uintptr_t)(PMU_DMEM_BASE | PMU_DMEM_CAL_TRACKER);
	if (tracker[0x22 / 2] != 0)
		tracker[40 / 2] = 1;

	u32 res = pmu_cal_pll_lock_retry_poll((PMU_DMEM_BASE | PMU_DMEM_CAL_TRACKER));
	dmem_write16(PMU_DMEM_CAL_MARKER_A, (u16)res);
	memset((void *)(uintptr_t)(PMU_DMEM_BASE | PMU_DMEM_CAL_TRACKER), 0, 0x20);
	tracker[0x28 / 2] = 0;
	tracker[0x26 / 2] = 0;
	tracker[0x24 / 2] = 0;
	tracker[0x22 / 2] = 0;
	return res;
}

/**
 * pmu_cal_dual_rank_state_eval() - Evaluate dual-rank calibration shadow register state
 * @table_base: Pointer to calibration parameter base table in DMEM
 * @out_reg_count: Output counter tracking shadow register count
 * @out_rank_idx: Output tracker for active rank index
 */
void pmu_cal_dual_rank_state_eval(void *table_base, u16 *out_reg_count, u16 *out_rank_idx)
{
	u8 *base = (u8 *)table_base;
	for (u32 rank = 0; rank < 2; rank++) {
		for (u32 lane = 0; lane < 2; lane++) {
			u16 f2 = dmem_read16(PMU_DMEM_CAL_STEP_METRIC_F2);
			u16 e8 = dmem_read16(PMU_DMEM_RANK_BASE_VAL_E8);
			u16 ea = dmem_read16(PMU_DMEM_RANK_BASE_VAL_EA);
			if (f2 != 0 || e8 != 0 || ea != 0) {
				u32 idx = (rank * 2 + lane) * 4;
				u16 val1 = *(u16 *)(base + idx);
				u16 val2 = *(u16 *)(base + idx + 2);
				u32 shadow_reg_addr = ((u32)*out_rank_idx * 2 + lane + 0x20800) & 0x7ffff;
				u32 track_idx = ((u32)*out_reg_count << 2) & 0xffff;
				pmu_phy_reg_write_shadow_track(track_idx, val1, val2, shadow_reg_addr, 3);
				(*out_reg_count)++;
			}
		}
		(*out_rank_idx)++;
	}
}

/**
 * pmu_dmem_reg_stream_unpack() - Reset DBYTE broadcast delays and strobe calibration from descriptor
 * @desc: Pointer to calibration descriptor containing register offset at offset +0x18
 */
void pmu_dmem_reg_stream_unpack(void *desc)
{
	phy_write16(PHY_REG_DBYTE_BCAST_CLEAR, 0);
	phy_write16(PHY_REG_DBYTE_BCAST_DQ_DLY0, 0);
	phy_write16(PHY_REG_DBYTE_BCAST_DQ_DLY1, 0);
	phy_write16((PHY_REG_MASTER_BASE | 0x0180), 0);
	phy_write16(PHY_REG_RESET_TRIGGER, 0);

	u32 offset = *(u32 *)((uintptr_t)desc + 0x18);
	phy_write16(PHY_REG_DBYTE_BCAST_BASE | (offset << 1), 0);

	u16 v = phy_read16(PHY_REG_BIST_PLL_CFG);
	v &= ~(1 << 4);
	phy_write16(PHY_REG_BIST_CMD, v);

	pmu_cal_strobe_pulse();
	if (dmem_read8((PMU_DMEM_BASE | PMU_DMEM_FREQ_MODE)) <= 1)
		pmu_phy_reset_pulse();
}

/**
 * pmu_cal_window_valid_check() - Validate calibration timing window against boundary limits
 * @window_start: Starting delay tap of valid calibration window
 * @window_end: Ending delay tap of valid calibration window
 */
void pmu_cal_window_valid_check(u32 window_start, u32 window_end)
{
	phy_write16(PHY_REG_DBYTE_BCAST_TIMING, 0);
	pmu_cal_strobe_secondary_pulse();
	pmu_deskew_and_tracker_reset();
	pmu_cal_sequence_pulse_send(1 << 21, 5, 1, 256, 0, window_start, 0);
	pmu_cal_sequence_pulse_send(0x80 | (1 << 18), 0xf, 0x20, 0, 0, window_start, window_end);
	pmu_clk_timing_delay_latch(0, 1);
	phy_write16(PHY_REG_DBYTE_BCAST_TIMING, 1);
}

/**
 * pmu_cal_stage8_pad_init() - Execute Stage 8 ZQ pad calibration and master state initialization
 */
void pmu_cal_stage8_pad_init(void)
{
	u32 stat = pmu_cal_status_148_read();
	u32 channels = (stat >> 4) & 3;
	u32 rank_metric = channels * (stat & 0xf);

	dmem_write32(PMU_DMEM_CAL_PARAM_410, channels);
	dmem_write32(PMU_DMEM_ACTIVE_SLICE_BITMAP, rank_metric);

	u8 rank1_en = dmem_read8(PMU_DMEM_CH1_RANK_EN);
	u32 pad_cfg;

	if (rank1_en) {
		pad_cfg = ((rank_metric << 2) & 248) + dmem_read8(PMU_DMEM_CH1_DQ_WIDTH);
	} else {
		pad_cfg = dmem_read8(PMU_DMEM_CH0_DQ_WIDTH);
		if (channels == 2)
			dmem_write32(PMU_DMEM_ACTIVE_SLICE_BITMAP, rank_metric >> 1);
	}

	u32 val_8 = ((u8)pad_cfg >> 3) + ((pad_cfg & 7) ? 1 : 0);
	u32 val_4 = ((u8)pad_cfg >> 2) + ((pad_cfg & 3) ? 1 : 0);

	dmem_write8(PMU_DMEM_ACTIVE_SLICE_COUNT, (u8)val_8);
	pmu_cal_metric_log(10, 0x1de0002, val_8, val_4);

	u8 dmem25 = dmem_read8(PMU_DMEM_CH0_RANK_EN);

	pmu_cal_metric_log(10, 0x1e00006, dmem25, rank1_en,
			   dmem_read16(PMU_DMEM_SEQUENCE_CTRL), dmem_read8(PMU_DMEM_POST_THRESHOLD),
			   dmem_read8(PMU_DMEM_CLK_GATE_FLAG), dmem_read16(PMU_DMEM_DRAM_FREQ_OFF));
	pmu_cal_metric_log(10, 0x1e90001, dmem_read8(PMU_DMEM_CHANNEL_CFG) & 0xf);

	if (dmem25 & 1)
		pmu_profile_mailbox_stream_send(0);
	if (dmem25 & 2)
		pmu_profile_mailbox_stream_send(1);
	if (rank1_en & 1)
		pmu_profile_mailbox_stream_send(2);
	if (rank1_en & 2)
		pmu_profile_mailbox_stream_send(3);

	u32 ch_off = dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET) << 1;
	u16 ddr_telemetry_94 = phy_read16((PHY_REG_MASTER_BASE | 0x0094) | ch_off);
	u16 ddr_telemetry_96 = phy_read16((PHY_REG_MASTER_BASE | 0x0096) | ch_off);

	pmu_cal_metric_log(4, 0x1ea000b,
			   ddr_telemetry_94, ddr_telemetry_94 & 0x7f, (ddr_telemetry_94 >> 8) & 0x7f,
			   ddr_telemetry_96, ddr_telemetry_96 & 0x7f, (ddr_telemetry_96 >> 8) & 0x7f,
			   phy_read16((PHY_REG_MASTER_BASE | 0x0098) | ch_off),
			   phy_read16((PHY_REG_MASTER_BASE | 0x0110)),
			   phy_read16((PHY_REG_MASTER_BASE | 0x0112)),
			   phy_read16((PHY_REG_MASTER_BASE | 0x0114)),
			   phy_read16((PHY_REG_MASTER_BASE | 0x0118)));

	u16 pub_ctl = phy_read16(PHY_REG_PUB_ID_REVISION);
	pmu_cal_param_table_init();
	dmem_write8(PMU_DMEM_STAGE_STATUS, !((pub_ctl >> 2) & 1));

	phy_write16((PHY_REG_MASTER_BASE | 0x0050), 1);
	phy_write16((PHY_REG_PUB_SEC_BASE | 0x14a), 1);
	phy_write16((PHY_REG_PUB_SEC_BASE | 0x0ee), 0);

	pmu_cal_metric_log(4, 0x1ed0001, *(volatile u8 *)0x004a4802);
	pmu_cbt_state_latch(4);
	(void)*(volatile u8 *)0x004a4802;
	pmu_cal_metric_log(5, 0x01ee0000);

	if (dmem_read16(PMU_DMEM_SEQUENCE_CTRL) != 1) {
		if ((dmem_read8(PMU_DMEM_CHANNEL_CFG) & 0xf0) == 0x20)
			pmu_dual_rank_phy_reg_read();
	}

	if (dmem_read8(PMU_DMEM_AC_PROFILE_EN) != 0)
		pmu_phy_slice_cal_status_read();
}

/**
 * pmu_cal_descriptor_apply() - Apply calibration descriptor settings to PHY registers
 * @desc: Pointer to calibration descriptor containing register offsets and values
 */
void pmu_cal_descriptor_apply(const void *desc)
{
	const u8 *p = (const u8 *)desc;
	u32 offset = (*(const u32 *)(uintptr_t)(PMU_DMEM_BASE | 0x0a68)) << 1;
	u8 cfg_word = p[0x14];
	u8 bist_flag = p[0x01];
	u16 val0 = (cfg_word & 0x0c) | ((bist_flag >> 3) & 1);
	u16 is_zero = ((cfg_word & 3) == 0) ? 1 : 0;

	phy_write16((PHY_REG_MASTER_BASE | 0x002a) | offset, val0);
	phy_write16(PHY_REG_DBYTE_BCAST_WIDTH | offset, is_zero << 1);
	phy_write16(PHY_REG_DBYTE_BCAST_GATE | offset, is_zero);
}

/**
 * pmu_cal_marker_pair_program() - Program calibration marker pair and dispatch window search
 * @marker0: Primary synchronization marker value
 * @marker1: Secondary synchronization marker value
 * @win_param: Search window base address parameter in DMEM
 * @win_mode: Window search mode (0 = fine search mode 3, non-zero = coarse mode 1)
 */
void pmu_cal_marker_pair_program(u32 marker0, u32 marker1, u32 win_param, u32 win_mode)
{
	u8 buf[8] = {0};
	buf[0] = (u8)win_mode;
	buf[1] = (u8)win_mode;

	pmu_cal_markers_set(marker0);
	u32 mode = (win_mode == 0) ? 3 : 1;
	pmu_cal_window_params_dispatch(buf, win_param, mode);
	pmu_cal_markers_set(marker1);
	pmu_clk_timing_delay_latch(0, 1);
}

/**
 * pmu_assert_or_halt() - Assert condition or log failure code and halt ARC EM4
 * @cond: Boolean assertion condition (non-zero indicates success)
 * @code: Diagnostic assert error code written to host mailbox on failure
 */
void pmu_assert_or_halt(u32 cond, u32 code)
{
	if (cond)
		return;

	dmem_write32(PMU_DMEM_CAL_SELECT_FLAGS_0C, code);
	pmu_mailbox_post_cmd_dispatch(0xff);
	__asm__ volatile (
		"flag 1\n"
		"nop_s\n"
		"1: b_s 1b\n"
	);
}

/**
 * pmu_clear_tracker_words() - Zero 16 bytes of calibration tracker registers
 * @base: Pointer to calibration tracker memory region in DMEM
 */
__attribute__((noinline)) void pmu_clear_tracker_words(volatile void *base)
{
	volatile u32 *t = (volatile u32 *)base;
	t[0] = 0;
	t[1] = 0;
	t[2] = 0;
	t[3] = 0;
}

/**
 * pmu_phy_ac_lane_timing_delay_set() - Set AC lane timing delay and trigger ratio pulse
 * @lane: Target AC lane index or slice selector
 *
 * Return: Decremented stride value in steps.
 */
u16 pmu_phy_ac_lane_timing_delay_set(u32 lane)
{
	pmu_cal_markers_set(800);

	u32 pulse_cmd = 0x9 << 19;
	pmu_cal_sequence_pulse_send(pulse_cmd, 5, 0xc, 512, 0, lane, 0);

	pmu_cal_sequence_pulse_send(0, 6, 2, 3, 0x1a, lane, 0);
	pmu_clk_ratio_pulse_trigger();

	pmu_cal_sequence_pulse_send(0, 6, 2, 1, 0x1a, lane, 0);
	pmu_clk_ratio_pulse_trigger();

	pmu_cal_sequence_pulse_send(0, 6, 2, 0, 0x1a, lane, 0);

	pmu_cbt_coarse_step_pulse_seq(0, 0x22, 0);

	pmu_cal_sequence_pulse_send(0, 5, 4, 896, 0, lane, 0);

	pmu_cal_sequence_pulse_send(0x80 | (1 << 19), 7, 4, 0, 0, 0, 0);

	u32 stride = pmu_cal_stride_get();
	return (u16)(stride - 1);
}

/**
 * pmu_cal_window_params_dispatch() - Dispatch window parameter pulses based on mode
 * @params_buf: Pointer to calibration parameter buffer in DMEM
 * @base_val: Base calibration parameter value or offset
 * @mode_flags: Window dispatch mode selector flags (1, 2, or 3)
 *
 * Return: PLL lock retry status code.
 */
u32 pmu_cal_window_params_dispatch(const u8 *params_buf, u16 base_val, u32 mode_flags)
{
	u16 buf[22];
	memset(buf, 0, 0x2a);

	buf[3] = params_buf[0];
	buf[4] = params_buf[1];
	buf[11] = params_buf[2];
	buf[12] = params_buf[3];

	if (mode_flags & 1)
		buf[5] = base_val;
	if (mode_flags & 2)
		buf[13] = base_val;

	return pmu_cal_pll_lock_retry_poll((uintptr_t)buf);
}

/**
 * pmu_cal_pulse_burst_send() - Send multi-phase calibration pulse burst to DBYTE slice
 * @burst_type: Pulse burst command selector (0 or 1)
 * @count: Number of pulse cycles to repeat
 * @slice: Target DBYTE slice index or mask
 */
void pmu_cal_pulse_burst_send(u32 burst_type, u32 count, u32 slice)
{
	if (count == 0)
		return;

	u32 base = (slice << 5) & 64;
	u32 cmd0 = base | (1 << 26);
	u32 cmd1 = base | (1 << 27);
	u32 cmd = (burst_type != 0) ? cmd1 : cmd0;

	for (u32 i = 0; i < count; i++) {
		pmu_cal_sequence_pulse_send(cmd, 7, 0, 0, 0, 0, 0);
	}
}

/**
 * pmu_cal_multi_param_step_commit() - Dispatch multi-parameter calibration pulses across active ranks
 * @param_table: DMEM offset of parameter step table
 * @step_val: Calibration step value or flags
 * @eval_mask: Lane evaluation error mask flag
 * @filter_mask: Bitmask of active slices/lanes to filter
 * @slice_flags: Slice control flags
 */
void pmu_cal_multi_param_step_commit(u32 param_table, u32 step_val, u32 eval_mask, u32 filter_mask, u32 slice_flags)
{
	pmu_phy_mode_cfg_dispatch(3);
	u8 a7c = dmem_read8(PMU_DMEM_AC_STEP_FLAG);
	u8 clk_gate_flag = dmem_read8(PMU_DMEM_CLK_GATE_FLAG);
	if (a7c != 0 && !(clk_gate_flag & (1 << 3))) {
		pmu_cal_multi_rank_step_commit(0xf, param_table, step_val, eval_mask, filter_mask, 0, slice_flags);
	} else {
		pmu_cal_multi_rank_timing_step(0xf, param_table, step_val, eval_mask, filter_mask, slice_flags);
	}
}

/**
 * pmu_train_main() - Top-level PMU training firmware entry point
 *
 * Description:
 * Initializes memory structures, coordinates training sequences via
 * pmu_train_dispatcher, logs diagnostic markers, and enters low-power halt.
 *
 * Return: 0 on normal completion.
 */
__attribute__((used)) u32 pmu_train_main(void)
{
	pmu_train_dispatcher();
	return 0;
}

/**
 * pmu_channel_rank_avail_check() - Test whether target channel and rank are enabled and available
 * @bit_idx: Channel and rank bit selector index
 *
 * Return: 1 if channel/rank is available, 0 otherwise.
 */
u32 pmu_channel_rank_avail_check(u32 bit_idx)
{
	u8 rank = dmem_read8(PMU_DMEM_CAL_RANK);
	u32 mask = 1u << bit_idx;
	u32 mode;

	if (rank == 1) {
		u8 param_40 = dmem_read8(PMU_DMEM_CH1_RANK_EN);
		if (!(mask & param_40))
			return 0;
		mode = 2;
	} else if (rank != 0) {
		mode = 3;
	} else {
		u8 param_25 = dmem_read8(PMU_DMEM_CH0_RANK_EN);
		if (!(mask & param_25))
			return 0;
		mode = 1;
	}

	pmu_phy_mode_cfg_dispatch(mode);
	return 1;
}

/**
 * pmu_tracker_field_extract() - Extract tracker field bytes across slices into destination array
 * @dest: Destination buffer receiving extracted bytes
 * @byte_offset: Offset of the target field within each tracker entry
 */
void pmu_tracker_field_extract(u8 *dest, u32 byte_offset)
{
	for (u32 rank = 0; rank < 2; rank++) {
		for (u32 byte_lane = 0; byte_lane < 2; byte_lane++) {
			u32 src_off = (rank * 216) + (byte_lane * 108) + (byte_offset * 2);
			u8 val = *(volatile u8 *)(uintptr_t)((PMU_DMEM_BASE | PMU_DMEM_CAL_SHADOW_TABLE) + src_off);
			u32 dst_off = (rank * 108) + (byte_lane * 54) + byte_offset;
			dest[dst_off] = val;
		}
	}
}

/**
 * pmu_tracker_cal_mode_update() - Update tracker calibration mode and clear status flags
 * @desc: Pointer to calibration descriptor in DMEM
 */
void pmu_tracker_cal_mode_update(u8 *desc)
{
	desc[0x0b] &= 0x87;
	desc[0x01] &= ~(1u << 3);
	desc[0x13] &= ~0x0c;

	u32 flags = pmu_dmem_training_flags_eval();

	u8 d0d = desc[0x0d];
	if (flags != 0) {
		d0d |= (1u << 5);
		desc[0x0d] = d0d;
		desc[0x03] |= (1u << 7);
	}

	desc[0x14] = 2;
	desc[0x10] = 0x40;
	desc[0x0d] = d0d & ~(1u << 4);
	desc[0x11] |= 0x38;
	desc[0x12] &= ~0x07;
	desc[41] &= 0x1f;
	desc[0x1c] &= ~(1u << 5);
}

/**
 * pmu_cal_timing_param_step_calc() - Calculate timing parameter step and update calibration tracker
 *
 * Return: AC profile mode index (0..3).
 */
u32 pmu_cal_timing_param_step_calc(void)
{
	u32 mode_8a = pmu_dmem_timing_mode_check();
	u32 nibble_96 = pmu_dmem_train_param_nibble_get();
	u8 d1c = dmem_read8(PMU_DMEM_ACTIVE_LANES);
	u32 ps;

	if (d1c != 0) {
		if (nibble_96 != 0) {
			ps = (mode_8a == 0) ? (0x7d << 7) : 0x4a38; /* 16000 ps or 19000 ps */
		} else {
			ps = (mode_8a == 0) ? 0x2ee0 : 0x4a38;       /* 12000 ps or 19000 ps */
		}
	} else {
		if (nibble_96 != 0) {
			ps = (mode_8a == 0) ? 0x4650 : 0x5208;       /* 18000 ps or 21000 ps */
		} else {
			ps = (mode_8a == 0) ? 0x36b0 : 0x5208;       /* 14000 ps or 21000 ps */
		}
	}

	u32 steps = pmu_freq_delay_step_calc(ps, 0);
	if (steps < 4)
		steps = 4;
	return (u8)steps;
}

/*
 * pmu_cal_slice_delay_best_fit_select:
 * Selects between primary and alternate timing delay candidates based on threshold comparison,
 * packs 7-bit values, and updates slice delay registers.
 */
void pmu_cal_slice_delay_best_fit_select(u32 idx, u32 thresh1, u32 thresh2,
										 const u8 *ptr3, const u8 *ptr0,
										 const u8 *ptr4, const u8 *ptr8,
										 u8 *slice_base)
{
	u8 *entry = slice_base + (idx << 2);

	if (thresh2 > thresh1) {
		u16 val_2 = *(volatile u16 *)(entry + 0x2);
		u16 val_a = *(volatile u16 *)(entry + 0xa);
		u16 val_phase1 = ((u16)ptr4[0] << 7) + ptr4[1];
		u16 val_phase0 = ((u16)ptr8[0] << 7) + ptr8[1];
		*(volatile u16 *)(entry + 0x22) = val_2;
		*(volatile u16 *)(entry + 0x2a) = val_a;
		*(volatile u16 *)(entry + 0x20) = val_phase1;
		*(volatile u16 *)(entry + 0x28) = val_phase0;
	} else {
		u16 val_12 = *(volatile u16 *)(entry + 0x12);
		u16 val_1a = *(volatile u16 *)(entry + 0x1a);
		u16 val_phase1 = ((u16)ptr3[0] << 7) + ptr3[1];
		u16 val_phase0 = ((u16)ptr0[0] << 7) + ptr0[1];
		*(volatile u16 *)(entry + 0x22) = val_12;
		*(volatile u16 *)(entry + 0x2a) = val_1a;
		*(volatile u16 *)(entry + 0x20) = val_phase1;
		*(volatile u16 *)(entry + 0x28) = val_phase0;
	}
}

/**
 * pmu_dmem_cal_offset_select() - Select calibration parameter DMEM offset based on profile mode
 * @rank: Target DRAM rank index (0 = rank 0, 1 = rank 1)
 * @unused: Unused auxiliary parameter
 * @direction: Direction selector (0 = low/min offset, 1 = high/max offset)
 *
 * Return: Selected calibration table offset.
 */
u32 pmu_dmem_cal_offset_select(u32 rank, u32 unused, u32 direction)
{
	u32 mode = pmu_cal_profile_mode_get();

	if (mode == 2) {
		if (rank == 0)
			return (direction == 0) ? 0x4f : 0x51;
		else
			return (direction == 0) ? 0x4e : 0x50;
	} else if (mode == 1) {
		return (rank == 0) ? 0x51 : 0x50;
	} else if (mode == 0) {
		return (rank == 0) ? 0x4f : 0x4e;
	} else {
		if (rank == 0)
			return (unused == 0) ? 0x4f : 0x51;
		else
			return (unused == 0) ? 0x4e : 0x50;
	}
}

/**
 * pmu_cal_rank_stride_count_get() - Determine active rank stride iteration count (1 or 2)
 * @rank_idx: DRAM rank index (0..1)
 *
 * Return: Number of active rank strides (1 or 2).
 */
u32 pmu_cal_rank_stride_count_get(u32 rank_idx)
{
	if (rank_idx == 0 && pmu_cal_profile_mode_get() == 3)
		return 2;
	return 1;
}

/**
 * pmu_cal_channel_rank_config_get() - Read channel rank control and mask information from DMEM
 * @channel: Target memory channel index (0 or 1)
 * @out_bit: Output pointer for bit index
 * @out_mask: Output pointer for rank mask word
 * @out_mode: Output pointer for operating mode byte
 *
 * Return: 1 if channel/rank combination is enabled, 0 otherwise.
 */
u32 pmu_cal_channel_rank_config_get(u32 channel, u16 *out_bit, u8 *out_mask, u8 *out_mode)
{
	u8 val, ctrl;
	u8 channel_idx, channel_mode;
	u8 rank_invert;

	if (channel < 2) {
		val = dmem_read8(PMU_DMEM_CH0_RANK_CFG);
		ctrl = dmem_read8(PMU_DMEM_CH0_RANK_EN);
		rank_invert = (channel != 0);
		channel_mode = 1;
		channel_idx = 0;
	} else {
		val = dmem_read8(PMU_DMEM_CH1_RANK_CFG);
		ctrl = dmem_read8(PMU_DMEM_CH1_RANK_EN);
		rank_invert = (channel != 2);
		channel_mode = 2;
		channel_idx = 1;
	}

	u8 bit = (val & 1) ^ rank_invert;
	*out_bit = bit;
	*out_mask = (1 << bit);

	if (!(ctrl & (1 << bit)))
		return 1;

	*out_mode = channel_mode;
	dmem_write8(PMU_DMEM_CAL_RANK, channel_idx);
	return 0;
}

/**
 * pmu_cal_channel_status_get() - Retrieve channel training status words and flag bytes from DMEM
 * @channel: Target memory channel index (0 or 1)
 * @out_val: Output pointer for status word
 * @out_a: Output pointer for flag byte A
 * @out_b: Output pointer for flag byte B
 */
void pmu_cal_channel_status_get(u32 channel, u32 *out_val, u8 *out_a, u8 *out_b)
{
	if (channel == 0) {
		u8 cal_param25 = dmem_read8(PMU_DMEM_CH0_RANK_EN);
		*out_val = (cal_param25 == 3) ? 2 : 1;
		*out_a = dmem_read8(PMU_DMEM_RANK0_SLICE_START);
		*out_b = dmem_read8(PMU_DMEM_RANK_BOUNDARY);
	} else if (channel == 1) {
		u8 cal_param40 = dmem_read8(PMU_DMEM_CH1_RANK_EN);
		if (cal_param40 == 1)
			*out_val = 1;
		else if (cal_param40 == 3)
			*out_val = 2;
		else
			*out_val = 0;
		*out_a = dmem_read8(PMU_DMEM_RANK1_SLICE_START);
		*out_b = dmem_read8(PMU_DMEM_RANK1_SLICE_END);
	}
}

/**
 * pmu_phy_csr_result_stream() - Stream calibration result halfwords to PHY CSR space
 * @buf: Pointer to array of 16-bit result halfwords
 * @len: Number of halfwords to stream
 */
void pmu_phy_csr_result_stream(const void *buf, u32 len)
{
	const u16 *src = (const u16 *)buf;
	u16 marker = dmem_read16(PMU_DMEM_CAL_MARKER_B);
	uintptr_t reg_offset = (marker << 1) | 0x82000;
	u32 count = len ? len : 1;
	for (u32 i = 0; i < count; i++) {
		phy_write16(PHY_REG_APB_BASE | reg_offset, *src++);
		reg_offset += 2;
	}
	u16 updated = len + marker;
	dmem_write16(PMU_DMEM_CAL_MARKER_A, updated);
	dmem_write16(PMU_DMEM_CAL_MARKER_B, updated);
}

/**
 * pmu_dq_swap_polarity_check() - Query DQ swap status and verify polarity boundary bit
 * @slice: DBYTE slice index (0..3)
 * @bit_idx: Target DQ bit index (0..7)
 *
 * Return: Polarity status bit (1 if swapped, 0 otherwise).
 */
u32 pmu_dq_swap_polarity_check(u32 slice, u32 bit_idx)
{
	if (pmu_dq_swap_query(slice, bit_idx))
		return 1;
	u8 rank_boundary = dmem_read8(PMU_DMEM_RANK_BOUNDARY);
	return (pmu_dmem_active_lane_query(rank_boundary, slice) < bit_idx) ? 1 : 0;
}

/**
 * pmu_multiphase_pulse_seq_repeat() - Repeat multi-phase pulse sequence for specified count
 * @phase_cmd: Sequence pulse command opcode / phase selector
 * @param_val: Timing parameter or delay code
 * @slice: Target DBYTE slice index
 * @count: Number of pulse iterations to execute
 */
void pmu_multiphase_pulse_seq_repeat(u32 phase_cmd, u32 param_val, u32 slice, u32 count)
{
	while (count--) {
		pmu_cal_sequence_pulse_send(0, phase_cmd, 0, param_val, 0, slice, 0);
	}
}

/**
 * pmu_phy_reg_stream_play() - Play stream of 6-byte register writes to PHY CSR space
 * @stream: Pointer to register configuration stream tuples
 * @len: Length of stream data in bytes
 */
void pmu_phy_reg_stream_play(const u8 *stream, u32 len)
{
	u32 count = len / 6;
	for (u32 i = 0; i < count; i++, stream += 6) {
		u32 addr = (u32)stream[0] | ((u32)stream[1] << 8) |
				   ((u32)stream[2] << 16) | ((u32)stream[3] << 24);
		u16 val = (u16)stream[4] | ((u16)stream[5] << 8);
		if (addr == 0xffffffff) {
			pmu_hw_timer_delay(val);
		} else {
			uintptr_t reg = PHY_REG_APB_BASE | (addr << 1);
			phy_write16(reg, val);
		}
	}
}

/**
 * pmu_phy_lane_timing_offset_set() - Configure per-lane 4-phase timing offsets in PLL space
 * @lane: Bit lane index within slice (0..8)
 * @v1: Timing delay code for phase 0
 * @v2: Timing delay code for phase 1
 * @v3: Timing delay code for phase 2
 * @v4: Timing delay code for phase 3
 */
void pmu_phy_lane_timing_offset_set(u32 lane, u16 v1, u16 v2, u16 v3, u16 v4)
{
	volatile u16 *p = (volatile u16 *)((PHY_REG_PLL_CFG_BASE | 0x048) | (lane << 9));
	p[0] = v1; p[1] = v2; p[2] = v1; p[3] = v2;
	p[4] = v3; p[5] = v4; p[6] = v3; p[7] = v4;

	volatile u16 *ctrl = (volatile u16 *)(PHY_REG_PLL_CFG_BASE | 0x0ca);
	if (lane == 0xf)
		*ctrl = 511;
	else
		*ctrl |= (1u << lane);
}

/**
 * pmu_cal_marker_verify_update() - Verify calibration markers and update tracker status
 * @marker_flags: Marker configuration bitmask
 *
 * Return: Updated calibration marker status.
 */
u32 pmu_cal_marker_verify_update(u32 marker_flags)
{
	volatile u16 *marker_a_ptr = (volatile u16 *)(PMU_DMEM_BASE | PMU_DMEM_CAL_MARKER_A);
	volatile u32 *tracker_dac = (volatile u32 *)(PMU_DMEM_BASE | PMU_DMEM_CBT_DEBUG_BASE);
	volatile u16 *tracker_dac16 = (volatile u16 *)(PMU_DMEM_BASE | PMU_DMEM_CBT_DEBUG_BASE);
	u32 res = *marker_a_ptr;

	if (marker_flags & (1u << 11)) {
		tracker_dac16[10] |= 1; // offset 0x14
		if ((*(volatile u8 *)marker_a_ptr & 7) != 0) {
			tracker_dac[3] = 0; // offset 0xc
			tracker_dac[2] = 0; // offset 0x8
			tracker_dac[1] = 0; // offset 0x4
			tracker_dac[0] = 0; // offset 0x0
		}
		res = pmu_cal_marker_verify();
	} else {
		if ((*marker_a_ptr & 7) == 0) {
			res = pmu_cal_marker_verify();
		}
	}

	if (marker_flags & (1u << 7)) {
		if ((*(volatile u8 *)marker_a_ptr & 7) != 0) {
			tracker_dac[3] = 0;
			tracker_dac[2] = 0;
			tracker_dac[1] = 0;
			tracker_dac[0] = 0;
		}
		res = pmu_cal_marker_verify();
	}
	return res;
}

/**
 * pmu_rank_mask_multiparam_dispatch() - Update rank mask and dispatch multi-parameter calibration
 * @shadow_update: Non-zero to copy and mask shadow structure, 0 to extract tracker field
 */
void pmu_rank_mask_multiparam_dispatch(u32 shadow_update)
{
	u8 rank = dmem_read8(PMU_DMEM_CAL_BYTE);
	u8 rank_mask = (1u << rank) | ((1u << rank) << 2);
	u32 base = PMU_DMEM_BASE | PMU_DMEM_CAL_STRUCT_BASE;

	if (shadow_update != 0) {
		pmu_cal_struct_to_shadow16(base, 0x12);
		pmu_cal_struct_mask_or(base, 0x12, 0x80);
	} else {
		pmu_tracker_field_extract((u8 *)(uintptr_t)base, 0x12);
	}

	u8 channel = dmem_read8(PMU_DMEM_CAL_RANK);
	uintptr_t struct_offset = base + channel * 108 + rank * 54;
	pmu_cal_multi_rank_timing_step(rank_mask, struct_offset, 0, 0xfffbffff, -1, 0);
	pmu_delay_us(20000, 0);
}

/**
 * pmu_phy_slice_mask_set() - Set PHY slice mask registers based on rank bitmask
 * @rank_bitmask: Bitmask of active ranks/lanes across slices
 */
void pmu_phy_slice_mask_set(u32 rank_bitmask)
{
	phy_write16(PHY_REG_DBYTE_BCAST_CLEAR, 0);
	u8 end_slice = dmem_read8(PMU_DMEM_SLICE_END);
	u8 cur_slice = dmem_read8(PMU_DMEM_SLICE_START);

	for (; cur_slice <= end_slice; cur_slice++) {
		uintptr_t base = (uintptr_t)cur_slice << 13;
		if (cur_slice == 0) {
			phy_write16(base | PHY_REG_DBYTE_LCDL_CTRL, 1);
			if ((rank_bitmask & (1u << 8)) == 0)
				phy_write16(PHY_REG_PUB_BASE | 0x3f16a, 0);
		} else {
			u32 bitmask = rank_bitmask;
			for (u32 bit = 0; bit < 9 && bitmask != 0; bit++, bitmask >>= 1) {
				if (bitmask & 1)
					phy_write16(PHY_REG_DBYTE_BASE | PHY_REG_DBYTE_LCDL_STATUS | base | (bit << 9), 1);
			}
		}
	}
}

/**
 * pmu_phy_reg_buffer_stream() - Stream PHY register reads or writes from/to a byte buffer
 * @stream: Pointer to register configuration stream buffer
 * @count: Number of register configuration tuples in stream
 * @is_write: Transfer direction (1 = write to PHY, 0 = read from PHY)
 */
void pmu_phy_reg_buffer_stream(u8 *stream, u32 count, u32 is_write)
{
	if (is_write != 0) {
		for (u32 i = 0; i < count; i++) {
			u32 addr = (u32)stream[0] | ((u32)stream[1] << 8) |
					   ((u32)stream[2] << 16) | ((u32)stream[3] << 24);
			stream += 4;
			uintptr_t reg = PHY_REG_APB_BASE | (addr << 1);
			u16 val = phy_read16(reg);
			stream[0] = (u8)val;
			stream[1] = (u8)(val >> 8);
			stream += 2;
		}
	} else {
		for (u32 i = 0; i < count; i++) {
			u32 addr = (u32)stream[0] | ((u32)stream[1] << 8) |
					   ((u32)stream[2] << 16) | ((u32)stream[3] << 24);
			u16 val = (u16)stream[4] | ((u16)stream[5] << 8);
			stream += 6;
			uintptr_t reg = (PHY_REG_APB_BASE | 0x1e000) | (addr << 1);
			phy_write16(reg, val);
		}
	}
}

/**
 * pmu_phy_reg_write_shadow_track() - Write 4 consecutive PHY registers and mirror to shadow trace buffer
 * @reg_offset: Register word offset within APB PHY window 0x140000
 * @val1: 16-bit primary configuration data written to offset +0
 * @val2: 16-bit secondary configuration data written to offset +2
 * @val3: 16-bit tertiary configuration data written to offset +4
 * @flags: Flag bits shifted into register at offset +6
 */
void pmu_phy_reg_write_shadow_track(u32 reg_offset, u32 val1, u32 val2, u32 val3, u32 flags)
{
	uintptr_t reg_base = (PHY_REG_APB_BASE | 0x140000) | (reg_offset << 1);
	u16 shadow_tag_val = ((flags << 3) | ((val3 >> 16) & 7)) & 0xffff;

	phy_write16(reg_base + 0, (u16)val1);
	phy_write16(reg_base + 2, (u16)val2);
	phy_write16(reg_base + 4, (u16)val3);
	phy_write16(reg_base + 6, shadow_tag_val);

	uintptr_t shadow_ptr = *(volatile uintptr_t *)(PMU_DMEM_BASE | PMU_DMEM_SHADOW_TRACE_PTR);
	*(volatile u16 *)(shadow_ptr + 0) = (u16)val1;
	*(volatile u16 *)(shadow_ptr + 2) = (u16)val2;
	*(volatile u16 *)(shadow_ptr + 4) = (u16)val3;
	*(volatile u16 *)(shadow_ptr + 6) = shadow_tag_val;
	*(volatile uintptr_t *)(PMU_DMEM_BASE | PMU_DMEM_SHADOW_TRACE_PTR) = shadow_ptr + 8;
}

/**
 * pmu_phy_deskew_reset_strobe() - Strobe PHY deskew reset across active byte slices
 */
void pmu_phy_deskew_reset_strobe(void)
{
	pmu_cal_metric_log(4, 0x49 << 19);
	u8 param_40 = dmem_read8(PMU_DMEM_CH1_RANK_EN);
	u8 param_25 = dmem_read8(PMU_DMEM_CH0_RANK_EN);
	pmu_deskew_and_tracker_reset();
	u32 mask = (param_40 | param_25) & 3;
	pmu_cal_sequence_pulse_send(0x80, 0xb, 0x20, 0, 0, mask, 0);
	pmu_clk_timing_delay_latch(0, 1);
	pmu_hw_timer_delay(0x14);
}

/**
 * pmu_phy_mode_cfg_dispatch() - Dispatch PHY mode configuration writes across channels
 * @mode: Channel mode selector bitmask
 */
void pmu_phy_mode_cfg_dispatch(u32 mode)
{
	uintptr_t reg = PHY_REG_ANALOG_CFG0;
	u8 param_25 = dmem_read8(PMU_DMEM_CH0_RANK_EN);
	u8 param_40 = dmem_read8(PMU_DMEM_CH1_RANK_EN);
	u8 *slice_bounds = (u8 *)(PMU_DMEM_BASE | PMU_DMEM_SLICE_START);

	if (mode == 2) {
		phy_write16(reg, 0);
		phy_write16(reg + 2, param_40);
		phy_write16(reg + 2 + 0x16, 0);
		*(u16 *)slice_bounds = *(const u16 *)(slice_bounds + 4);
		phy_write16(reg + 2 + 0xe, 15);
	} else if (mode == 1) {
		phy_write16(reg, param_25);
		phy_write16(reg + 2, 0);
		phy_write16(reg + 2 + 0x16, 0);
		*(u16 *)slice_bounds = *(const u16 *)(slice_bounds + 2);
		phy_write16(reg + 2 + 0xe, 0xf0);
	} else {
		u8 param_40_val = dmem_read8(PMU_DMEM_CH1_RANK_EN);
		phy_write16(reg, param_25);
		phy_write16(reg + 2, param_40_val);
		phy_write16(reg + 2 + 0x16, 0);
		u16 val_e = phy_read16(reg + 2 + 0xe) & ~0x11;
		phy_write16(reg + 2 + 0xe, val_e);
		slice_bounds[0] = slice_bounds[2];
		slice_bounds[1] = slice_bounds[5];
		phy_write16(reg + 2 + 0xe, 0);
	}
	pmu_profile_param_stride_cfg(mode);
}

/**
 * pmu_phy_slice_cal_status_read() - Read calibration status across active byte slices and save to DMEM
 */
void pmu_phy_slice_cal_status_read(void)
{
	u32 param41c = dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET);
	u8 *dmem43c = (u8 *)(PMU_DMEM_BASE | PMU_DMEM_2D_STEP_TABLE);

	for (u32 slice = 0; slice < 2; slice++) {
		uintptr_t reg_offset = ((slice << 12) | param41c) << 1;
		u16 v_84 = phy_read16((PHY_REG_AC_BASE | 0x0084) + reg_offset);
		u16 v_80 = phy_read16((PHY_REG_AC_BASE | 0x0080) + reg_offset);
		u16 v_82 = phy_read16((PHY_REG_AC_BASE | 0x0082) + reg_offset);
		u16 v_66 = phy_read16((PHY_REG_AC_BASE | 0x0066) + reg_offset);
		u16 v_68 = phy_read16((PHY_REG_AC_BASE | 0x0068) + reg_offset);
		u16 v_5c = phy_read16((PHY_REG_AC_BASE | 0x005c) + reg_offset);

		*(u16 *)(dmem43c + slice * 2 + 0x8) = v_84;
		*(u16 *)(dmem43c + slice * 2 + 0x14) = v_80;
		*(u16 *)(dmem43c + slice * 2 + 0x18) = v_82;
		dmem43c[slice + 0xe] = (u8)v_66;
		dmem43c[slice + 0x10] = (u8)v_68;
		dmem43c[slice + 0xc] = (u8)v_5c;
	}
}

/**
 * pmu_cal_state_restore_offset_prog() - Restore PHY state and program calibrated delay offsets
 */
void pmu_cal_state_restore_offset_prog(void)
{
	u8 cal_param25 = dmem_read8(PMU_DMEM_CH0_RANK_EN);
	u8 cal_param40 = dmem_read8(PMU_DMEM_CH1_RANK_EN);
	phy_write16(PHY_REG_ANALOG_CFG0, cal_param25);
	pmu_phy_cal_state_restore();
	phy_write16(PHY_REG_ANALOG_CFG1, cal_param40);
	pmu_slice_status_b97_save();

	u16 dram_freq = dmem_read16(PMU_DMEM_DRAM_FREQ_OFF);
	u16 delay_step_a = 0xff;
	u16 delay_step_b = 0xff;
	if (dram_freq >= 0xc81) {
		delay_step_a = phy_read16(PHY_REG_CLK_TIMING_DELAY0);
		delay_step_b = phy_read16(PHY_REG_CLK_TIMING_DELAY1);
	}

	u8 freq_mode = dmem_read8(PMU_DMEM_FREQ_MODE);
	u32 offset = (u32)dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET) << 1;
	if (freq_mode >= 8) {
		u16 v1 = (delay_step_a << 8) + 1;
		u16 v2 = (delay_step_b << 8) + 1;
		phy_write16((PHY_REG_APB_BASE | 0x121010) | offset, v1);
		phy_write16((PHY_REG_APB_BASE | 0x121026) | offset, v2);
	} else if (freq_mode >= 2) {
		u16 v1 = (delay_step_a << 8) + 1;
		u16 v2 = (delay_step_b << 8) + 1;
		phy_write16((PHY_REG_APB_BASE | 0x121006) | offset, v1);
		phy_write16((PHY_REG_APB_BASE | 0x121012) | offset, v2);
	}
}

/**
 * pmu_dual_rank_phy_reg_read() - Read dual-rank PHY registers and copy to DMEM shadow buffer
 */
void pmu_dual_rank_phy_reg_read(void)
{
	pmu_cal_metric_log(4, 0x540000);

	u32 total_idx = 0;
	u32 table_base = (PMU_DMEM_BASE | PMU_DMEM_DUAL_RANK_REG_TABLE);

	for (u32 rank = 0; rank < 2; rank++) {
		u8 active = (rank == 0) ? *(const volatile u8 *)(PMU_DMEM_BASE | PMU_DMEM_CH0_RANK_EN) : *(const volatile u8 *)(PMU_DMEM_BASE | PMU_DMEM_CH1_RANK_EN);
		if (active != 0) {
			u16 base_val = *(const volatile u16 *)((PMU_DMEM_BASE | PMU_DMEM_RANK_BASE_VAL_E8) + rank * 2);
			u32 base_reg = ((u32)base_val << 2) | (1 << 12) | (1 << 18);

			for (u32 fp = 0; fp < 2; fp++) {
				u8 count = dmem_read8(PMU_DMEM_METRIC_TABLE_ENTRIES);
				u8 *entry = (u8 *)(table_base + fp);

				for (u32 entry_idx = 0; entry_idx < count; entry_idx++) {
					u8 cal_cmd = *(const volatile u8 *)((PMU_DMEM_BASE | PMU_DMEM_CAL_CMD_TABLE) + entry_idx);
					u32 reg = PHY_REG_APB_BASE | (((base_reg + (u8)total_idx) << 1));
					u16 val = phy_read16(reg);
					*entry = (u8)val;
					entry += 4;
					pmu_cal_metric_log(4, 0x550004, fp, rank, (s8)(u8)val, (u32)cal_cmd);
					total_idx++;
				}
			}
		}
		table_base += 2;
	}
}

/**
 * pmu_multirank_slice_cal_dispatch() - Dispatch multi-rank slice calibration across channels and ranks
 * @table_ptr: Pointer to calibration parameter base table in DMEM
 * @eval_error_mask: Lane error evaluation mask flag
 * @skip_mask_low: Bitmask of sequence map indices 0..31 to skip
 * @skip_mask_high: Bitmask of sequence map indices 32..63 to skip
 * @ch_enable_mask: Bitmask of active channels and ranks to process
 * @check_settle_flags: Flag controlling settle delay checks on first iteration
 */
void pmu_multirank_slice_cal_dispatch(void *table_ptr, u32 eval_error_mask, u32 skip_mask_low, s32 skip_mask_high, u32 ch_enable_mask, u32 check_settle_flags)
{
	u8 rank1_mask = dmem_read8(PMU_DMEM_CH1_RANK_EN);
	u8 rank0_mask = dmem_read8(PMU_DMEM_CH0_RANK_EN);

	for (u32 rank = 0; rank < 2; rank++) {
		u32 bit = 1 << rank;
		u32 rank_mask = (bit << 2) | bit;

		for (u32 fp = 0; fp < 2; fp++) {
			u8 mask = (fp == 0) ? rank0_mask : rank1_mask;
			if ((ch_enable_mask & bit) & mask) {
				pmu_phy_mode_cfg_dispatch(1 << fp);
				u8 a7c = *(const volatile u8 *)(PMU_DMEM_BASE | PMU_DMEM_AC_STEP_FLAG);
				u8 clk_gate_flag = dmem_read8(PMU_DMEM_CLK_GATE_FLAG);

				u32 offset = fp * 108 + rank * 54;
				if (a7c != 0 && !(clk_gate_flag & 8)) {
					pmu_cal_multi_rank_step_commit((u8)rank_mask, (u32)(uintptr_t)table_ptr + offset, eval_error_mask, skip_mask_low, skip_mask_high, 0, check_settle_flags);
				} else {
					pmu_cal_multi_rank_timing_step((u8)rank_mask, (u32)(uintptr_t)table_ptr + offset, eval_error_mask, skip_mask_low, skip_mask_high, check_settle_flags);
				}
			}
		}
	}

	pmu_phy_mode_cfg_dispatch(3);
}

/**
 * pmu_phy_cal_strobe_latch_setup() - Configure PHY calibration strobe latching and timing delays
 */
void pmu_phy_cal_strobe_latch_setup(void)
{
	u32 param05 = pmu_dmem_pll_bypass_read();
	pmu_delay_us(0, 0x40);

	volatile u16 *b5e = (volatile u16 *)PHY_REG_PUB_SEC_BASE;
	__asm__("" : "+r"(b5e));
	b5e[0x620 >> 1] = 1;
	b5e[0x1f0 >> 1] = 1;

	u16 vref_val = phy_read16((PHY_REG_MASTER_BASE | 0x0092));
	u32 fp_val = (PHY_REG_MASTER_BASE | 0x009c);

	if (param05 != 0) {
		b5e[0x186 >> 1] = 1;
	} else {
		vref_val &= 496;
		u32 vref_cfg = vref_val | (1 << 10);
		b5e[0x092 >> 1] = vref_cfg;
		pmu_delay_us(1000, 0);
		pmu_delay_us(1000000, 0);
		b5e[0x1c0 >> 1] = 1;
		vref_val |= (1 << 9);
		u16 cal_code = phy_read16(fp_val + (129 << 1));
		b5e[0x186 >> 1] = 1;
		b5e[0x1c0 >> 1] = 0;
		b5e[0x092 >> 1] = vref_val;
		u32 reg = ((dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET)) << 1) | fp_val;
		phy_write16(reg, cal_code & 0x1f);
	}

	u8 freq_mode = dmem_read8(PMU_DMEM_FREQ_MODE);
	u32 csr = (PHY_REG_APB_BASE | 0x13e050);
	if (freq_mode != 0)
		csr += 10;
	phy_write16(csr, 1);
}

/**
 * pmu_phy_reg_offset_diff_adjust() - Program Master PHY register offsets with differential tap adjustments
 * @sel_pair_idx: Tap pair selector (1 selects tap 0, 0 selects tap 1 and inverts delta)
 */
void pmu_phy_reg_offset_diff_adjust(u32 sel_pair_idx)
{
	u32 csr_offset_sh1 = (dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET)) << 1;
	u8 *b_ptr = (u8 *)(PMU_DMEM_BASE | PMU_DMEM_DIFF_TAP_PAIR);
	u8 b = sel_pair_idx ? b_ptr[0] : b_ptr[1];
	s32 diff = (s32)b_ptr[0] - (s32)b_ptr[1];
	if (!sel_pair_idx)
		diff = -diff;

	u32 base = (PHY_REG_MASTER_BASE | 0x0058) | csr_offset_sh1;
	phy_write16(base,     (phy_read16(base)     & 0x3f00) | b);
	phy_write16(base + 2, (phy_read16(base + 2) & 0x3f00) | b);
	phy_write16(base + 8, (phy_read16(base + 8) & 0x3f00) | b);

	u16 v0 = phy_read16(base + 0x1a);
	u16 v1 = phy_read16(base + 0x1e);
	u16 v2 = phy_read16(base + 0x20);

	u16 n0 = (v0 & 0x3f00) | (((v0 & 0x3f) + diff) & 0x3f);
	phy_write16(base + 0x1a, n0);
	phy_write16(base + 0x1c, n0);
	phy_write16(base + 0x1e, (v1 & 0x3f00) | (((v1 & 0x3f) + diff) & 0x3f));
	phy_write16(base + 0x20, (v2 & 0x3f00) | (((v2 & 0x3f) + diff) & 0x3f));
}

/**
 * pmu_multiparam_cal_dispatch() - Dispatch multi-parameter calibration sequence and configure structures
 */
void pmu_multiparam_cal_dispatch(void)
{
	uintptr_t cal_addr = (PMU_DMEM_BASE | PMU_DMEM_CAL_STRUCT_BASE);
	u8 *cal = (u8 *)cal_addr;
	u8 *param = (u8 *)(PMU_DMEM_BASE | PMU_DMEM_SLICE_PROFILE_TABLE);
	pmu_cal_struct_mask_or(cal_addr, 1, param[0] & 8);
	pmu_cal_struct_mask_and(cal_addr, 0x14, 0);

	pmu_cal_struct_mask_or(cal_addr, 0x14, param[52] & 0xf);

	if ((param[-63] & 1) && (*(const volatile u8 *)(PMU_DMEM_BASE | PMU_DMEM_LANE_ACTIVE_FLAGS) != 0)) {
		const u8 *src = (const u8 *)(PMU_DMEM_BASE | PMU_DMEM_DELAY_CAL_RESULTS);
		cal[0x78] = src[2];
		cal[0x0c] = src[0];
		cal[0x42] = src[1];
		cal[0xae] = src[3];
	}

	pmu_cal_struct_mask_and(cal_addr, 0x10, 0xfb);
	pmu_cal_struct_mask_or(cal_addr, 0x10, 0x41);

	pmu_multirank_slice_cal_dispatch((void *)cal_addr, 0, 0, 0, 0xff, 0);

	pmu_cal_struct_mask_or(cal_addr, 0x10, 4);
	pmu_cal_multi_param_step_commit(cal_addr, 0, 0xfffeffff, 0xffffffff, 1);
}

/**
 * pmu_cal_pulse_seq_strobe() - Send timing calibration phase strobe sequence to target slices
 * @param_code: Calibration timing parameter code
 * @tap_idx: Delay tap index or subcode
 * @slice_mask: Target DBYTE slice bitmask
 */
__attribute__((noinline)) void pmu_cal_pulse_seq_strobe(u32 param_code, u32 tap_idx, u32 slice_mask)
{
	pmu_cal_sequence_pulse_send(0, 6, 0x22, param_code, tap_idx, slice_mask, 0);
}

/**
 * pmu_cal_pulse_seq_sub7() - Send subcode 7 calibration pulse with delay parameter
 * @val: 8-bit delay tap or parameter value
 */
__attribute__((noinline)) void pmu_cal_pulse_seq_sub7(u8 val)
{
	pmu_cal_sequence_pulse_send(0, 7, 2, 0, 0, 0, val);
}

/**
 * pmu_cal_pulse_seq_coordinator() - Coordinate frequency-scaled calibration pulse sequence
 * @rank: DRAM rank index (0..1)
 * @param1: Base delay step parameter
 * @flags: Calibration sequence control flags
 */
void pmu_cal_pulse_seq_coordinator(u32 rank, u32 param1, u32 flags)
{
	u32 ratio = pmu_freq_ratio_mult(0xfa, param1);

	pmu_deskew_and_tracker_reset();

	u8 ch_idx = dmem_read8(PMU_DMEM_CAL_RANK);
	u32 offset = (u32)ch_idx * 108 + rank * 0x36;
	const volatile u8 *p = (const volatile u8 *)((PMU_DMEM_BASE | PMU_DMEM_CAL_STRUCT_BASE) + offset);
	u8 cal_cfg10 = p[0x10];

	u32 mask1 = 1 << rank;
	u8 mask = (u8)((mask1 << 2) | mask1);
	u32 p10_mod = cal_cfg10 | (1 << 6);

	pmu_cal_pulse_seq_strobe(p10_mod, 16, mask);

	ratio >>= 1;
	if (flags & 1) {
		pmu_cal_pulse_seq_sub7((u8)(ratio + 8));
		pmu_cal_pulse_seq_strobe(param1, 14, mask);
	}

	if (flags & 2) {
		pmu_cal_pulse_seq_sub7((u8)ratio);
		pmu_cal_pulse_seq_strobe(param1, 15, mask);
	}

	pmu_cal_pulse_seq_sub7((u8)ratio);
	pmu_cal_pulse_seq_sub7(0x28);
	pmu_clk_timing_delay_latch(0, 1);
}

/**
 * pmu_dmem_rank_slice_table_setup() - Initialize DMEM rank slice tables and slice boundaries
 */
void pmu_dmem_rank_slice_table_setup(void)
{
	volatile u8 *dmem = (volatile u8 *)PMU_DMEM_BASE;
	__asm__("" : "+r"(dmem));
	u8 dq0 = dmem[PMU_DMEM_CH0_DQ_WIDTH];
	u8 rank1_en = dmem[PMU_DMEM_CH1_RANK_EN];

	u8 num_slices0 = (dq0 + 7) >> 3;
	u8 end_slice0 = num_slices0 - 1;
	u8 start_slice1 = num_slices0;
	u8 end_slice1;

	dmem[PMU_DMEM_RANK0_SLICE_START] = 0;
	dmem[PMU_DMEM_RANK_BOUNDARY] = end_slice0;

	if (rank1_en != 0) {
		u8 dq1 = dmem[PMU_DMEM_CH1_DQ_WIDTH];
		start_slice1 = (u8)(*(volatile u32 *)(dmem + PMU_DMEM_ACTIVE_SLICE_BITMAP) >> 1);
		num_slices0 = ((dq1 + 7) >> 3) + start_slice1;
		dmem[PMU_DMEM_RANK1_SLICE_START] = start_slice1;
		end_slice1 = num_slices0 - 1;
	} else {
		end_slice1 = end_slice0;
		dmem[PMU_DMEM_RANK1_SLICE_START] = start_slice1;
	}
	dmem[PMU_DMEM_RANK1_SLICE_END] = end_slice1;

	u8 mask = dmem[PMU_DMEM_ACTIVE_SLICE_MASK];
	for (int i = 0; i <= (int)end_slice0; i++)
		mask |= (1 << i);
	dmem[PMU_DMEM_ACTIVE_SLICE_MASK] = mask;

	if (end_slice1 >= start_slice1) {
		for (int i = (int)start_slice1; i <= (int)end_slice1; i++)
			mask |= (1 << i);
		dmem[PMU_DMEM_ACTIVE_SLICE_MASK] = mask;
	}

	u8 d1c = dmem[PMU_DMEM_ACTIVE_LANES];
	for (u32 rank = 0; rank < 2; rank++) {
		for (u32 ch = 0; ch < 2; ch++) {
			u8 dq = (ch == 0) ? dmem[PMU_DMEM_CH0_DQ_WIDTH] : dmem[PMU_DMEM_CH1_DQ_WIDTH];
			u8 shift = ch * 2;
			u8 slice_count_val;
			if ((d1c >> shift) & (1 << rank))
				slice_count_val = (dq + 7) >> 3;
			else
				slice_count_val = (dq + 15) >> 4;
			dmem[PMU_DMEM_RANK_SLICE_COUNTS + (ch * 2 + rank)] = slice_count_val;
		}
	}
}

/**
 * pmu_cal_ptr_tag_check() - Validate pointer tag descriptor and test bit 3
 * @unused: Unused parameter reserved for ABI alignment
 * @ptr_tag: Encoded pointer tag descriptor
 *
 * Return: 1 if pointer tag bit is set, 0 otherwise.
 */
u32 pmu_cal_ptr_tag_check(u32 unused, u32 ptr_tag)
{
	uintptr_t addr = (ptr_tag >> 27) | (ptr_tag << 5);
	volatile u8 *p = (volatile u8 *)addr;
	u8 tag = p[0];
	if (!(tag & 0x40))
		return 0;
	u8 lo = p[7];
	u8 hi = p[8];
	u16 val = (u16)lo | ((u16)hi << 8);
	return (val >> 3) & 1;
}

/**
 * pmu_cal_search_win_max_calc() - Calculate maximum delay window across active byte slices
 * @window_type: Window type (0 = standard search window, 1 = alternate fine window)
 * @direction_flag: Direction selector flag (0 selects direction 1, non-zero selects 0)
 */
void pmu_cal_search_win_max_calc(u32 window_type, u32 direction_flag)
{
	u8 rank = dmem_read8(PMU_DMEM_CAL_BYTE);

	if (window_type == 1 && dmem_read8(PMU_DMEM_CH1_RANK_EN) == 0) {
		u32 tbl_off = 0x43c + (rank << 2);
		dmem_write16(tbl_off + 2, dmem_read16(tbl_off));
		return;
	}

	u32 lane_off = 0xb6a + (window_type ? 2 : 0);
	u8 lane_start = dmem_read8(lane_off);
	u8 lane_end   = dmem_read8(lane_off + 1);

	s32 max_val = -1;
	if (lane_start <= lane_end) {
		u32 dir_flag = (direction_flag == 0) ? 1 : 0;
		u32 csr_offset = dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET);
		u32 reg_base = (dir_flag << 2) + 0x20 + rank + csr_offset;
		u32 slice_step = (u32)lane_start << 12;
		u32 count = (u32)lane_end + 1 - lane_start;
		for (u32 i = 0; i < count; i++) {
			u32 addr = PHY_REG_DBYTE_BASE | ((reg_base | slice_step) << 1);
			u16 val = phy_read16(addr);
			if ((s32)val > max_val)
				max_val = (s32)val;
			slice_step += (512 << 3);
		}
	}

	u8 dram_type = dmem_read8(PMU_DMEM_DRAM_TYPE);
	s8 cal_bias = (s8)dmem_read8(PMU_DMEM_SEARCH_WINDOW_BIAS);
	u32 mask = (dram_type == 2) ? 0xff : 511;
	u32 shift = (dram_type == 2) ? 8 : 9;
	u32 flag = ((u32)max_val & mask) ? 1 : 0;
	s32 term = max_val >> shift;
	u32 window_rank_idx = (window_type << 1) + (rank << 2);

	dmem_write16(PMU_DMEM_2D_STEP_TABLE + window_rank_idx, (u16)(cal_bias + term + flag + 2));
}

/*
 * pmu_cal_stage39_post_results:
 * Stage 39 calibration results processing coordinator:
 * Aggregates 4-block delay calibration results from DMEM (PMU_DMEM_BASE | PMU_DMEM_DELAY_CAL_RESULTS) into
 * sequence config buffer, executes multi-rank calibration dispatch via pmu_multirank_slice_cal_dispatch,
 * resets deskew and tracking pipelines, sends timing calibration commands via pmu_cal_sequence_pulse_send,
 * asserts PLL CBT entry control, updates per-channel rank status and restores offsets.
 */
/**
 * pmu_cal_stage39_post_results() - Post stage 39 calibration results and update telemetry
 */
void pmu_cal_stage39_post_results(void)
{
	u8 cal_buf[216];
	pmu_cal_timing_table_matrix_init(cal_buf);

	u8 orig_flag = (dmem_read8(PMU_DMEM_CAL_STRUCT_BASE + 3) & 0x40) ? 1 : ((dmem_read8(PMU_DMEM_CAL_STRUCT_BASE + 0x16) >> 6) & 1);
	u8 new_flag = (cal_buf[3] & 0x40) ? 1 : ((cal_buf[0x16] >> 6) & 1);
	if (orig_flag != new_flag)
		pmu_phy_reg_offset_diff_adjust(0);

	pmu_cal_struct_mask_and((u32)(uintptr_t)cal_buf, 0x13, 0xf3);
	pmu_cal_struct_mask_or((u32)(uintptr_t)cal_buf, 0x10, 0x45);

	u8 num_entries = dmem_read8(PMU_DMEM_METRIC_TABLE_ENTRIES);
	for (u32 i = 0; i < num_entries; i++) {
		u8 cal_cmd_idx = dmem_read8(PMU_DMEM_CAL_CMD_TABLE + i);
		for (u32 rank = 0; rank < 2; rank++) {
			for (u32 ch = 0; ch < 2; ch++) {
				u8 val = dmem_read8(PMU_DMEM_DELAY_CAL_RESULTS + (i * 4) + (ch * 2) + rank);
				cal_buf[ch * 108 + rank * 54 + cal_cmd_idx] = val;
			}
		}
	}

	pmu_multirank_slice_cal_dispatch(cal_buf, 1, 0x800, 0, 0xff, 0);

	pmu_cal_struct_mask_and((u32)(uintptr_t)cal_buf, 0x10, 0xbc);
	if (!(cal_buf[0x1c] & 0x20))
		pmu_cal_struct_mask_or((u32)(uintptr_t)cal_buf, 0x1c, 2);

	pmu_multirank_slice_cal_dispatch(cal_buf, 1, 0xfffeffff, -1, 0xff, 0);

	pmu_phy_deskew_reset_strobe();
	pmu_deskew_and_tracker_reset();

	u8 ch = dmem_read8(PMU_DMEM_CAL_RANK);
	if (!(cal_buf[ch * 108 + 0x1c] & 0x20))
		pmu_cal_sequence_pulse_send(0, 6, 0x22, cal_buf[ch * 108 + 0x1c] | 2, 28, 15, 0);

	pmu_cal_sequence_pulse_send(0, 0x28, 2, 0, 0, 3, 0);
	pmu_cal_sequence_pulse_send(0, 7, 0x1e, 0, 0, 0, 0);
	pmu_clk_timing_delay_latch(0, 1);

	if (dmem_read8(PMU_DMEM_TRAIN_PARAM_CTRL) & 0x10) {
		u32 csr_offset = dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET);
		phy_write16(PHY_REG_DBYTE_BCAST_PULSE | (csr_offset << 1), 1);
	}

	pmu_cbt_entry_pll_ctrl();
	phy_write16((PHY_REG_MASTER_BASE | 0x00dc), 0);

	u8 dram_type = ((const volatile u8 *)pmu_reset_vector)[0];
	if (dram_type == 0) {
		for (u8 rank_iter = 0; rank_iter < 2; ) {
			u8 ch_idx = rank_iter;
			bool active = false;
			if (ch_idx != 0) {
				if (dmem_read8(PMU_DMEM_CH1_RANK_EN) != 0)
					active = true;
			} else {
				if (dmem_read8(PMU_DMEM_CH0_RANK_EN) != 0)
					active = true;
			}
			if (active) {
				pmu_phy_mode_cfg_dispatch(ch_idx + 1);
				pmu_cal_lane_deskew_sweep_exec();
				rank_iter = dmem_read8(PMU_DMEM_CAL_RANK) + 1;
			} else {
				rank_iter = ch_idx + 1;
			}
			dmem_write8(PMU_DMEM_CAL_RANK, rank_iter);
		}
	}

	pmu_phy_mode_cfg_dispatch(3);
	dmem_write16(PMU_DMEM_CAL_RANK, 0);

	if (dram_type == 0)
		pmu_cal_state_restore_offset_prog();

	phy_write16(PHY_REG_DBYTE_BCAST_DQ_DLY2, 0);
	phy_write16(PHY_REG_DBYTE_BCAST_DQ_DLY3, 0);

	if (dram_type == 0 && dmem_read16(PMU_DMEM_SEQUENCE_CTRL) != 1)
		pmu_phy_slice_profile_update();

	pmu_phy_cal_strobe_latch_setup();
}

/**
 * pmu_cal_stage1_pre_init() - Execute stage 1 pre-initialization and analog setup
 */
void pmu_cal_stage1_pre_init(void)
{
	/* Step 1: Initialize hardware gating & tracking */
	pmu_dbyte_cal_strobe_pulse_seq(1, 2047, 0);

	/* Clear 3584-byte scan sample matrix (2 slices x 7 channels x 256 delay steps) */
	u8 scan_matrix[3584];
	memset(scan_matrix, 0, sizeof(scan_matrix));

	/* Verify clock gating configuration: bit 3 of DMEM[0x01] must be 0 */
	u32 clk_gate_flag = dmem_read8(PMU_DMEM_CLK_GATE_FLAG);
	u32 cond = !(clk_gate_flag & (1 << 3));
	pmu_assert_or_halt(cond, 0x25 << 19);

	/*
	 * Determine calibration step size from upper nibble of PMU_DMEM_CAL_STEP_CONFIG:
	 * bit 7 -> 8, bit 6 -> 4, bit 5 -> 2, default -> 1.
	 * Store step size in PMU_DMEM_CAL_STRIDE.
	 */
	u8 d1a = dmem_read8(PMU_DMEM_CAL_STEP_CONFIG);
	u8 d1a_high = d1a >> 4;
	u8 step;
	if (d1a_high & (1 << 3))
		step = 8;
	else if (d1a_high & (1 << 2))
		step = 4;
	else if (d1a_high & (1 << 1))
		step = 2;
	else
		step = 1;
	dmem_write8(PMU_DMEM_CAL_STRIDE, step);

	/* Backup 6 bytes of parameter state from PMU_DMEM_CAL_DESC_BACKUP_99C */
	u8 desc_backup_buf[6];
	memcpy(desc_backup_buf, (const void *)(PMU_DMEM_BASE | PMU_DMEM_CAL_DESC_BACKUP_99C), sizeof(desc_backup_buf));
	pmu_phy_reg_buffer_stream(desc_backup_buf, 1, 1);

	/*
	 * Step 2: Build inverse lane mapping table from PHY registers.
	 * Slice 0: (PHY_REG_AC_BASE | 0x0120)..(PHY_REG_AC_BASE | 0x012c) (7 channels).
	 * Slice 1 (if active in DMEM[0x40]): (PHY_REG_AC_BASE | 0x2120)..(PHY_REG_AC_BASE | 0x212c) (7 channels).
	 */
	u8 map_table[16] = {0};
	for (u32 i = 0; i < 7; i++) {
		u16 pin = phy_read16((PHY_REG_AC_BASE | 0x0120) + (i * 2));
		map_table[pin] = (u8)i;
	}

	if (dmem_read8(PMU_DMEM_CH1_RANK_EN) != 0) {
		for (u32 i = 0; i < 7; i++) {
			u16 pin = phy_read16((PHY_REG_AC_BASE | 0x2120) + (i * 2));
			map_table[pin + 7] = (u8)i;
		}
	}

	/*
	 * Step 3: Descriptor phase sample initialization & telemetry logging.
	 */
	pmu_cal_struct_mask_remap();

	u8 num_slices = 0;
	s8 init_step_offsets[4] = {0};

	for (u32 idx = 0; idx < 4; idx++) {
		u16 sp_40_val;
		u8 sp_4f_val;
		u8 sp_37_val;

		if (pmu_cal_channel_rank_config_get((u16)idx, &sp_40_val, &sp_4f_val, &sp_37_val) != 0)
			continue;

		num_slices = sp_37_val;
		dmem_write8(PMU_DMEM_AC_STEP_FLAG, 1);
		pmu_phy_mode_cfg_dispatch(num_slices);

		u8 rank = dmem_read8(PMU_DMEM_CAL_RANK);
		u32 csr_offset = dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET);
		u32 reg_base = (((u32)rank << 12) | csr_offset) << 1;
		phy_write16((PHY_REG_AC_BASE | 0x1e02) | reg_base, 0);
		phy_write16((PHY_REG_AC_BASE | 0x0004) | reg_base, 0);

		s16 ret_c184 = pmu_cal_dbyte_rx_fifo_reset_poll(sp_4f_val, 1, scan_matrix);
		init_step_offsets[(rank << 1) + sp_40_val] = (s8)ret_c184;

		pmu_cal_metric_log(4, 0x012b0002, rank, sp_40_val);
		pmu_dq_lane_telemetry_bitpack(num_slices, (uintptr_t)scan_matrix);
		pmu_cal_dbyte_rx_fifo_reset_poll(sp_4f_val, 0, scan_matrix);
		pmu_cal_metric_log(4, 0x012c0002, rank, sp_40_val);
		pmu_dq_lane_telemetry_bitpack(num_slices, (uintptr_t)scan_matrix);
	}

	/*
	 * Step 4: Clear channel gate delay registers and trigger calibration strobe.
	 */
	u32 csr_offset = dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET);
	u32 err_flag = 0;

	for (u32 ch = 0; ch < 7; ch++)
		phy_write16(PHY_REG_AC_STAT_LOW | (((csr_offset | (ch << 8)) << 1)), 0);

	phy_write16(PHY_REG_AC_STAT_HIGH | (csr_offset << 1), 0);
	pmu_cal_strobe_pulse();

	/*
	 * Step 5: Multi-slice / multi-channel eye margin search and LCDL programming.
	 */
	const u8 *map_ptr = map_table;

	for (u32 slice = 0; slice < num_slices; slice++) {
		u32 base_delay;
		if (dmem_read8(PMU_DMEM_CAL_STEP_CONFIG) & (1 << 3)) {
			pmu_cal_metric_log(5, 0x012d0002, slice, 0);
			base_delay = 0;
		} else {
			pmu_cal_metric_log(5, 0x012f0001, slice);
			base_delay = 0x40; /* Nominal LCDL midpoint 64 */
		}

		s32 lower_bound = (s32)base_delay - 128;
		s32 upper_bound = (s32)base_delay + 128;

		u16 center_delays[7];

		for (u32 ch = 0; ch < 7; ch++) {
			u32 best_start = 0;
			u32 best_len = 0;
			u32 cur_len = 0;
			u32 run_start = 0xff;

			for (u32 delay = 0; delay <= 255; delay += step) {
				u8 sample = scan_matrix[slice * 1792 + ch * 256 + delay];
				if (run_start != 0xff) {
					if (sample == 0) {
						cur_len += step;
					} else {
						if (cur_len > best_len && delay >= 129) {
							pmu_cal_metric_log(4, 0x01310004, ch, run_start, cur_len + step, delay - 1);
							best_start = run_start;
							best_len = cur_len + step;
						}
						run_start = 0xff;
					}
				} else {
					if (sample == 0) {
						run_start = delay;
						cur_len = 0;
					}
				}
			}

			if (best_len == 0 && cur_len != 0) {
				best_start = run_start;
				best_len = cur_len + step;
			}

			if (best_len == 0) {
				pmu_cal_metric_log(10, 0x01350000);
				err_flag = 1;
			} else {
				s32 center = (s32)best_start + (s32)((best_len & ~1) / 2) + lower_bound;
				if (center < 0)
					center = 0;
				else if (center > upper_bound)
					center = upper_bound;
				center_delays[ch] = (u16)center;
			}
		}

		/* Program DQS gate center delays for all 7 channels of this slice */
		for (u32 ch = 0; ch < 7; ch++) {
			u16 delay_val = center_delays[ch];
			u8 mapped_lane = map_ptr[ch];
			pmu_cal_metric_log(5, 0x01360003, slice, mapped_lane, delay_val);
			u32 addr = (PHY_REG_AC_BASE | 0x0002) | (((csr_offset | (slice << 12) | ((u32)mapped_lane << 8)) << 1));
			phy_write16(addr, delay_val);
		}

		/* Program lane pair LCDL offsets and TX DQS delay */
		s32 val0 = (s32)base_delay + (s32)init_step_offsets[slice * 2 + 0];
		u32 dly0 = (val0 > 0) ? (u32)val0 : 0;
		pmu_cal_metric_log(5, 0x01370002, slice, (u16)dly0);
		phy_write16((PHY_REG_AC_BASE | 0x1002) | (((csr_offset | (slice << 12)) << 1)), (u16)dly0);

		s32 val1 = (s32)base_delay + (s32)init_step_offsets[slice * 2 + 1];
		u32 dly1 = (val1 > 0) ? (u32)val1 : 0;
		pmu_cal_metric_log(5, 0x01380004, slice, (u16)dly1, slice, base_delay);
		map_ptr += 7;
		phy_write16((PHY_REG_AC_BASE | 0x1202) | (((csr_offset | (slice << 12)) << 1)), (u16)dly1);
		phy_write16((PHY_REG_AC_BASE | 0x0004) | (((csr_offset | (slice << 12)) << 1)), (u16)base_delay);
	}

	/*
	 * Step 6: Post-calibration assertion, strobes, optional CA sweep, & teardown.
	 */
	pmu_assert_or_halt(err_flag == 0, PMU_MSG_GATE_TRAIN_PASS);

	pmu_cal_strobe_pulse();
	pmu_cal_strobe_secondary_pulse();

	if (dmem_read8(PMU_DMEM_CAL_STEP_CONFIG) & 1) {
		phy_write16(PHY_REG_PHY_STATUS, 0x2000);
		pmu_cal_ca_eye_margin_sweep();
		phy_write16(PHY_REG_PHY_STATUS, 0);
	}

	pmu_phy_reg_buffer_stream(desc_backup_buf, 1, 0);
	pmu_dbyte_cal_strobe_pulse_seq(0, 0, 0);
}

/*
 * pmu_cal_multi_branch_dispatch:
 *
 * Configures calibration command/strobe parameters into the 16-byte dispatch
 * tracker record at `out` (DMEM 0xd9c or 0xdac), evaluating opcode-specific
 * field shifts and multi-flag priority decodes.
 */
void pmu_cal_multi_branch_dispatch(u16 *out, u32 flags, u32 opcode, u32 dispatch_param,
				   u32 val_arg0, u32 val_arg1, u32 flag_mode)
{
	out[0] = 0;
	out[1] = 0;
	out[2] = 0;
	out[6] = 0;
	out[7] = 0;

	if (opcode <= 0x2f) {
		u16 xbfu_val = (dispatch_param >> 8) & 0x7f;

		switch (opcode) {
		case 0:
			if (flag_mode != 0) {
				out[4] = dispatch_param & 0x7f;
				out[3] = ((dispatch_param >> 4) & 0x78) | 0x3;
			} else {
				out[4] = val_arg0 | ((dispatch_param >> 7) & 0x70);
				out[3] = ((dispatch_param >> 11) & 0x78) | 0x7;
			}
			break;

		case 1:
			out[4] = ((val_arg0 << 2) & 0x40) | ((dispatch_param << 3) & 0x30) | (val_arg0 & 0xf);
			out[3] = ((dispatch_param << 3) & 0x8) | ((dispatch_param >> 1) & 0x1c) | 1;
			break;

		case 2:
			out[4] = ((val_arg0 << 2) & 0x40) | ((dispatch_param << 3) & 0x30) | (val_arg0 & 0xf);
			out[3] = ((dispatch_param << 3) & 0x8) | ((dispatch_param >> 1) & 0x18) | 5;
			break;

		case 3:
			out[4] = ((val_arg0 << 2) & 0x40) | ((dispatch_param << 3) & 0x30) | (val_arg0 & 0xf);
			out[3] = ((dispatch_param << 3) & 0x8) | ((dispatch_param >> 1) & 0x18) | 6;
			break;

		case 4:
			out[4] = ((val_arg0 << 2) & 0x40) | ((dispatch_param << 3) & 0x30) | (val_arg0 & 0xf);
			out[3] = ((dispatch_param >> 1) & 0x18) | 4;
			break;

		case 5:
			out[4] = val_arg0 & 0x7f;
			out[3] = ((dispatch_param >> 3) & 0x70) | 0xc;
			break;

		case 6:
			if (flag_mode != 0) {
				out[4] = dispatch_param & 0x7f;
				out[3] = ((dispatch_param >> 1) & 0x40) | 8;
			} else {
				out[4] = val_arg0;
				out[3] = 0x58;
			}
			break;

		case 7:
		case 39:
			out[4] = 0;
			out[3] = 0;
			break;

		case 8:
		case 27:
			out[3] = 0x78;
			out[4] = val_arg0 & 0xf;
			break;

		case 10:
		case 17:
			out[3] = 0x38;
			out[4] = val_arg0 & 0x7;
			break;

		case 11:
			out[3] = 0x68;
			out[4] = (dispatch_param << 5) & 0x60;
			break;

		case 12:
			out[4] = 0;
			out[3] = 0x28;
			break;

		case 14:
			if (flag_mode != 0) {
				out[4] = dispatch_param & 0x7f;
				out[3] = ((dispatch_param >> 1) & 0x40) | 8;
			} else {
				out[4] = xbfu_val;
				out[3] = 0x58;
			}
			break;

		case 15:
			out[4] = val_arg0;
			out[3] = 24;
			break;

		case 16:
			out[3] = 0x38;
			out[4] = (val_arg0 & 0x7) | 0x40;
			break;

		case 25:
			out[4] = dispatch_param & 0x7f;
			out[3] = ((dispatch_param >> 1) & 0x40) | 0x30;
			break;

		case 26:
			out[3] = 0x78;
			out[4] = (val_arg0 & 0xf) | 0x40;
			break;

		case 28:
			out[4] = ((val_arg0 << 2) & 0x40) | ((dispatch_param << 3) & 0x30) | (val_arg0 & 0xf);
			out[3] = ((dispatch_param << 3) & 0x8) | ((dispatch_param >> 1) & 0x1c) | 2;
			break;

		case 29:
			out[4] = 0x58;
			out[3] = 0x58;
			break;

		case 30:
			out[4] = xbfu_val;
			out[3] = xbfu_val;
			break;

		case 31:
			out[4] = ((dispatch_param >> 1) & 0x40) | 8;
			out[3] = ((dispatch_param >> 1) & 0x40) | 8;
			break;

		case 32:
		case 38:
			out[4] = dispatch_param & 0x7f;
			out[3] = dispatch_param & 0x7f;
			break;

		case 33:
			out[3] = 24;
			break;

		case 34:
			out[4] = xbfu_val;
			break;

		case 35: {
			u16 val = ((dispatch_param >> 3) & 0x70) | 0xc;
			out[4] = val;
			out[3] = val;
			break;
		}

		case 36:
			out[4] = val_arg0 & 0x7f;
			out[3] = val_arg0 & 0x7f;
			break;

		case 37: {
			u16 val = ((dispatch_param >> 1) & 0x40) | 0x30;
			out[4] = val;
			out[3] = val;
			break;
		}

		case 40:
			out[4] = 0;
			out[3] = (flag_mode == 0) ? 0x40 : 0;
			break;

		case 41:
			out[4] = 0;
			out[3] = 0x60;
			break;

		case 42:
			out[4] = 0;
			out[3] = 0x20;
			break;

		case 43:
			out[4] = 0;
			out[3] = 0x50;
			break;

		case 44:
			out[4] = 0x38;
			out[3] = 0x38;
			break;

		case 45: {
			u16 val = (val_arg0 & 7) | 0x40;
			out[4] = val;
			out[3] = val;
			break;
		}

		case 46: {
			u16 val = ((dispatch_param << 3) & 0x8) | ((dispatch_param >> 1) & 0x1c) | 1;
			out[4] = val;
			out[3] = val;
			break;
		}

		case 47: {
			u16 val = ((val_arg0 << 2) & 0x40) | ((dispatch_param << 3) & 0x30) | (val_arg0 & 0xf);
			out[4] = val;
			out[3] = val;
			break;
		}

		case 9:
		case 13:
		case 18:
		case 19:
		case 20:
		case 21:
		case 22:
		case 23:
		case 24:
		default:
			break;
		}
	}

	/* Epilogue: Field out[5] (offset 0xa) */
	if ((flags & (1U << 16)) || (opcode == 0x28 && flag_mode != 0)) {
		out[5] = 0;
	} else {
		out[5] = val_arg1 & 3;
	}

	/* Epilogue: Field out[7] (offset 0xe) - Priority flag decode */
	if (flags & (1U << 20)) {
		out[7] = 13;
	} else if (flags & (1U << 21)) {
		out[7] = 0xd0;
	} else if (flags & (1U << 22)) {
		out[7] = 0xd00;
	} else if (flags & (1U << 23)) {
		out[7] = 8;
	} else if (flags & (1U << 24)) {
		out[7] = 0x80;
	} else if (flags & (1U << 25)) {
		out[7] = 1024;
	} else if (flags & (1U << 26)) {
		out[7] = 256;
	} else if (flags & (1U << 27)) {
		out[7] = 512;
	}
	if (flags & (1U << 19)) {
		out[7] |= 0x1000;
	}

	/* Epilogue: Field out[6] (offset 0xc) */
	u16 fc = 0;
	if (opcode == 3 || opcode == 4 || opcode == 28 || opcode == 41) {
		fc = 4;
		out[6] = 4;
	} else if (opcode == 1 || opcode == 2 || opcode == 33 || opcode == 34) {
		fc = (flags & (1U << 3)) ? 0 : 2;
		if ((flags & 0xc) == 0xc) {
			fc |= 1;
		} else {
			out[6] = fc;
		}
	}

	if (flags & (1U << 18)) {
		fc |= 3;
		out[6] = fc;
	}

	/* Epilogue: Field out[2] (offset 0x4) */
	if (flags & (1U << 15)) {
		out[2] = 1;
	}

	if (flags & (1U << 12)) {
		fc |= 8;
		out[6] = fc;
	}

	/* Epilogue: Field out[0] (offset 0x0) */
	if ((flags & (1U << 6)) || (val_arg1 & (1U << 1))) {
		out[0] = 1;
	}
}

/**
 * pmu_cal_tracker_dac_clear() - Clear calibration tracker DAC telemetry registers
 */
__attribute__((noinline)) void pmu_cal_tracker_dac_clear(void)
{
	dmem_write32(PMU_DMEM_CBT_DEBUG_0, 0);
	dmem_write32(PMU_DMEM_CBT_DEBUG_1, 0);
	dmem_write32(PMU_DMEM_CBT_DEBUG_2, 0);
	dmem_write32(PMU_DMEM_CBT_DEBUG_3, 0);
}

/**
 * pmu_cal_sequence_pulse_send() - Dispatch calibration pulse sequence and step updates to PHY
 * @cmd_flags: Command control flags and register address offsets
 * @pulse_type: Pulse sequence command opcode
 * @phase_id: Sequence phase identifier or pulse submode
 * @param_code: Calibration parameter code or delay tap step
 * @tap_idx: Delay tap index or CSR target
 * @slice_mask: Target DBYTE slice index or mask
 * @extra: Auxiliary calibration flags
 *
 * Return: Sequence pulse calibration response status.
 */
u32 pmu_cal_sequence_pulse_send(u32 cmd_flags, u32 pulse_type, u32 phase_id, u32 param_code, u32 tap_idx, u32 slice_mask, u32 extra)
{
	while (1) {
		u8 v24 = dmem_read8(PMU_DMEM_CBT_STEP_STATUS);

		/* Fast path: CBT calibration pulse sequence with active step status */
		if (pulse_type == 6 && v24 != 0) {
			u16 marker_a = dmem_read16(PMU_DMEM_CAL_MARKER_A);
			dmem_write16(PMU_DMEM_CBT_PAT_TAP, (u16)tap_idx);
			dmem_write16(PMU_DMEM_CBT_PAT_STATUS, 0x58);
			u16 a5_low = (u16)(slice_mask & 3);
			dmem_write16(PMU_DMEM_CBT_PAT_SLICE_HI, a5_low);
			dmem_write16(PMU_DMEM_CBT_PAT_SLICE_LO, a5_low);
			dmem_write16(PMU_DMEM_CBT_PAT_CODE, (u16)(param_code & 0x7f));
			u16 a5_shift = a5_low >> 1;
			dmem_write16(PMU_DMEM_CBT_PAT_SHIFT_HI, a5_shift);
			dmem_write16(PMU_DMEM_CBT_PAT_SHIFT_LO, a5_shift);
			u16 val_d86 = (u16)(((param_code >> 1) & 0x40) | 8);
			dmem_write16(PMU_DMEM_CBT_PAT_CFG, val_d86);
			dmem_write16(PMU_DMEM_CAL_MARKER_A, marker_a + 8);

			u32 b80_res = pmu_cal_pll_lock_retry_poll(PMU_DMEM_BASE | PMU_DMEM_CBT_PATTERN_BASE);
			u32 cbt_state = (u32)dmem_read8(PMU_DMEM_CBT_STATE);
			if ((marker_a & 7) == 0)
				cbt_state = b80_res << cbt_state;
			int diff = (int)phase_id - (int)cbt_state;
			pmu_cal_stride_div_update(diff, cmd_flags);
			return pmu_cal_marker_verify_update(cmd_flags);
		}

		/* Update calibration state register PMU_DMEM_CBT_PARAM if active */
		if (dmem_read8(PMU_DMEM_CBT_CTRL_468) == 1) {
			u32 test = pmu_cal_mode_mask_test(pulse_type);
			u8 cal_param = dmem_read8(PMU_DMEM_CBT_PARAM);
			u8 mode_prefix = (test == 1) ? 0xf0 : 0x30;
			dmem_write8(PMU_DMEM_CBT_PARAM, mode_prefix + (cal_param >> 4));
		}

		/* Latch PHY status tracking bits from cmd_flags */
		if (cmd_flags & (1 << 8))
			dmem_write16(PMU_DMEM_CBT_LOCK_STATUS, dmem_read16(PMU_DMEM_CBT_LOCK_STATUS) | (1 << 1));

		if (cmd_flags & (1 << 13))
			dmem_write16(PMU_DMEM_CBT_LOCK_STATUS, dmem_read16(PMU_DMEM_CBT_LOCK_STATUS) | (1 << 0));

		u16 marker_a = dmem_read16(PMU_DMEM_CAL_MARKER_A);

		/* Branch when extra != 0: Multi-step pulse dispatch */
		if (extra != 0) {
			if (v24 == 0)
				phase_id++;

			if ((marker_a & 7) == 4) {
				dmem_write16(PMU_DMEM_CAL_MARKER_A, marker_a + 4);
				pmu_cal_tracker_dac_clear();
				pmu_cal_tracker_step_update();
			}

			if (dmem_read8(PMU_DMEM_CBT_CAL_STATUS_SHADOW) == 0)
				dmem_write8(PMU_DMEM_CBT_CAL_STATUS_SHADOW, dmem_read8(PMU_DMEM_CBT_CAL_STATUS));

			pmu_cal_multi_branch_dispatch((u16 *)(PMU_DMEM_BASE | PMU_DMEM_CAL_TRACKER),
							  cmd_flags, pulse_type, param_code, tap_idx, slice_mask, 0);

			if (pmu_cal_mode_mask_test(pulse_type) != 1) {
				pmu_cal_tracker_dac_clear();
			} else {
				pmu_cal_multi_branch_dispatch((u16 *)((PMU_DMEM_BASE | PMU_DMEM_CAL_TRACKER) + 0x10),
								  cmd_flags, pulse_type, param_code, tap_idx, slice_mask, 1);
			}

			dmem_write16(PMU_DMEM_CAL_TRACKER_STATUS, (u16)extra);
			u32 cbt_state_byte = dmem_read8(PMU_DMEM_CBT_STATE);
			dmem_write16(PMU_DMEM_CAL_MARKER_A, dmem_read16(PMU_DMEM_CAL_MARKER_A) + 8);

			/* Shift-scaled arithmetic: sub1 r13, phase_id, r2 => phase_id - (r2 << 1) */
			int step_diff = (int)phase_id - ((int)cbt_state_byte << 1);
			if (step_diff < 1) {
				dmem_write16(PMU_DMEM_CBT_PHASE_STATUS, 3);
			} else {
				dmem_write16(PMU_DMEM_CBT_PHASE_STATUS, 2);
				pmu_cal_tracker_step_update();
				u32 cbt_state_curr = dmem_read8(PMU_DMEM_CBT_STATE);
				u32 div_val = ((u32)(step_diff + (int)cbt_state_curr) - 1) / cbt_state_curr;
				u16 adv = pmu_cal_tracker_stride_advance(dmem_read16(PMU_DMEM_CAL_MARKER_A), (u16)div_val, 0);
				dmem_write16(PMU_DMEM_CAL_MARKER_A, adv);
				cmd_flags |= (1 << 11);
			}

			dmem_write8(PMU_DMEM_CBT_CAL_STATUS_SHADOW, dmem_read8(PMU_DMEM_CBT_CAL_STATUS));
			return pmu_cal_marker_verify_update(cmd_flags);
		}

		/* extra == 0 paths */
		u32 marker_mod = marker_a & 7;

		if (marker_mod == 4) {
			if (!(cmd_flags & (1 << 9)) &&
				dmem_read8(PMU_DMEM_CBT_CAL_STATUS_SHADOW) == dmem_read8(PMU_DMEM_CBT_CAL_STATUS) &&
				dmem_read16(PMU_DMEM_CBT_PHASE_STATUS) == 0) {
				u32 mode_test = pmu_cal_mode_mask_test(pulse_type);
				pmu_cal_multi_branch_dispatch((u16 *)((PMU_DMEM_BASE | PMU_DMEM_CAL_TRACKER) + 0x10),
								  cmd_flags, pulse_type, param_code, tap_idx, slice_mask, 0);

				if (v24 == 0)
					phase_id++;

				dmem_write16(PMU_DMEM_CAL_MARKER_A, marker_a + 4);

				if (mode_test != 0) {
					pmu_cal_tracker_step_update();
					u32 b_45a = dmem_read8(PMU_DMEM_CBT_STATE);
					pmu_cal_multi_branch_dispatch((u16 *)(PMU_DMEM_BASE | PMU_DMEM_CAL_TRACKER),
									  cmd_flags, pulse_type, param_code, tap_idx, slice_mask, 1);
					dmem_write16(PMU_DMEM_CAL_MARKER_A, dmem_read16(PMU_DMEM_CAL_MARKER_A) + 4);
					int diff = (int)phase_id - (int)b_45a - (int)dmem_read8(PMU_DMEM_CBT_STATE);
					pmu_cal_stride_div_update(diff, cmd_flags);
				} else {
					int diff = (int)phase_id - (int)dmem_read8(PMU_DMEM_CBT_STATE);
					if (diff >= 1) {
						pmu_cal_tracker_step_update();
						pmu_cal_stride_div_update(diff, cmd_flags);
					}
					if (cmd_flags & (1 << 11))
						dmem_write16(PMU_DMEM_CBT_PHASE_STATUS, dmem_read16(PMU_DMEM_CBT_PHASE_STATUS) | (1 << 0));
				}

				return pmu_cal_marker_verify_update(cmd_flags);
			}

			/* Loop iteration: clear tracker, advance marker by 4, update tracker, loop */
			dmem_write16(PMU_DMEM_CAL_MARKER_A, marker_a + 4);
			pmu_cal_tracker_dac_clear();
			pmu_cal_tracker_step_update();
			continue;
		}

		if (marker_mod == 0) {
			if (dmem_read8(PMU_DMEM_CBT_CAL_STATUS_SHADOW) == 0)
				dmem_write8(PMU_DMEM_CBT_CAL_STATUS_SHADOW, dmem_read8(PMU_DMEM_CBT_CAL_STATUS));

			pmu_cal_multi_branch_dispatch((u16 *)(PMU_DMEM_BASE | PMU_DMEM_CAL_TRACKER),
							  cmd_flags, pulse_type, param_code, tap_idx, slice_mask, 0);

			if (v24 == 0)
				phase_id++;

			dmem_write16(PMU_DMEM_CAL_MARKER_A, dmem_read16(PMU_DMEM_CAL_MARKER_A) + 4);

			if (cmd_flags & (1 << 9))
				dmem_write16(PMU_DMEM_CBT_PHASE_STATUS, dmem_read16(PMU_DMEM_CBT_PHASE_STATUS) | (1 << 1));

			int diff = (int)phase_id - (int)dmem_read8(PMU_DMEM_CBT_STATE);

			if (pmu_cal_mode_mask_test(pulse_type) == 1) {
				pmu_cal_multi_branch_dispatch((u16 *)((PMU_DMEM_BASE | PMU_DMEM_CAL_TRACKER) + 0x10),
								  cmd_flags, pulse_type, param_code, tap_idx, slice_mask, 1);
				dmem_write16(PMU_DMEM_CAL_MARKER_A, dmem_read16(PMU_DMEM_CAL_MARKER_A) + 4);
				pmu_cal_marker_verify();
				diff -= (int)dmem_read8(PMU_DMEM_CBT_STATE);
			}

			pmu_cal_stride_div_update(diff, cmd_flags);
			dmem_write8(PMU_DMEM_CBT_CAL_STATUS_SHADOW, dmem_read8(PMU_DMEM_CBT_CAL_STATUS));
			return pmu_cal_marker_verify_update(cmd_flags);
		}

		/* Default return: marker_mod != 0 && marker_mod != 4 */
		return 0;
	}
}

/**
 * pmu_cal_timing_table_matrix_init() - Initialize timing table matrix from DMEM parameter array
 * @buf: Pointer to timing table matrix buffer in DMEM
 */
void pmu_cal_timing_table_matrix_init(u8 *buf)
{
	memset(buf, 0, 54);

	const volatile u8 *src = (const volatile u8 *)(PMU_DMEM_BASE | PMU_DMEM_SLICE_PROFILE_TABLE);

	copy_slice_quad(buf, 1, src); src += 4;
	copy_slice_quad(buf, 2, src); src += 4;
	copy_slice_quad(buf, 3, src); src += 4;
	copy_slice_quad(buf, 10, src); src += 4;
	copy_slice_quad(buf, 11, src); src += 4;
	copy_slice_quad(buf, 12, src); src += 4;
	copy_slice_quad(buf, 13, src); src += 4;
	copy_slice_quad(buf, 14, src); src += 4;
	copy_slice_quad(buf, 15, src); src += 4;
	copy_slice_quad(buf, 16, src); src += 4;
	copy_slice_quad(buf, 17, src); src += 4;
	copy_slice_quad(buf, 18, src); src += 4;
	copy_slice_quad(buf, 19, src); src += 4;
	copy_slice_quad(buf, 20, src); src += 4;
	copy_slice_quad(buf, 21, src); src += 4;
	copy_slice_quad(buf, 22, src); src += 4;
	copy_slice_quad(buf, 24, src); src += 4;
	copy_slice_quad(buf, 25, src); src += 4;
	copy_slice_quad(buf, 26, src); src += 4;
	copy_slice_quad(buf, 27, src); src += 4;
	copy_slice_quad(buf, 28, src); src += 4;
	copy_slice_quad(buf, 30, src); src += 4;
	copy_slice_quad(buf, 31, src); src += 4;
	copy_slice_quad(buf, 32, src); src += 4;
	copy_slice_quad(buf, 33, src); src += 4;
	copy_slice_quad(buf, 34, src); src += 4;
	copy_slice_quad(buf, 37, src); src += 4;
	copy_slice_quad(buf, 40, src); src += 4;
	copy_slice_quad(buf, 41, src);

	buf[46] = 0;
	buf[100] = 0;
	buf[154] = 0;
	buf[208] = 0;
}

/*
 * pmu_cal_phase_delay_tracker_update:
 */
/**
 * pmu_cal_phase_delay_tracker_slice_update() - Evaluate WCK window metrics and update phase delay
 * @rank_idx: DRAM rank index (0..1)
 * @active_csr_offset: Active CSR offset word
 * @csr_step_off: CSR step offset
 * @slice_param_off: Slice parameter offset
 */
void pmu_cal_phase_delay_tracker_slice_update(u32 rank_idx, u32 active_csr_offset, u32 csr_step_off, u32 slice_param_off)
{
	u32 rank_shift = (u8)rank_idx << 12;
	u32 slice_csr_offset = (active_csr_offset | rank_shift) << 1;
	u32 rank_off13 = (u8)rank_idx << 13;

	u16 val_1a0 = phy_read16((PHY_REG_DBYTE_BASE | 0x01a0) | slice_csr_offset);
	u16 val_1a2 = phy_read16((PHY_REG_DBYTE_BASE | 0x01a2) | slice_csr_offset);
	u16 val_1a4 = phy_read16((PHY_REG_DBYTE_BASE | 0x01a4) | slice_csr_offset);
	u16 val_1a6 = phy_read16((PHY_REG_DBYTE_BASE | 0x01a6) | slice_csr_offset);
	u16 val_1a8 = phy_read16((PHY_REG_DBYTE_BASE | 0x01a8) | slice_csr_offset);

	u32 reg_20158 = (PHY_REG_DBYTE_BASE | 0x0158) | rank_off13;
	u16 val_20158_strobe = phy_read16(reg_20158);
	phy_write16(reg_20158, val_20158_strobe | 1);
	phy_write16(reg_20158, val_20158_strobe & ~1);

	u16 val_1aa = phy_read16((PHY_REG_DBYTE_BASE | 0x01aa) | rank_off13);

	pmu_cal_metric_log(4, 0x420007, rank_idx, val_1a0, val_1a2, val_1a4, val_1a6, val_1a8, val_1aa);

	u32 delay_threshold = ((u32)val_1aa * 14) / 100;
	s32 diff_r1 = (s32)val_1a2 - (s32)val_1a0 - (s32)val_1aa;
	u32 flag_54 = ((val_1a0 * 2) < val_1aa) ? 1 : 0;
	u32 flag_58 = (diff_r1 > (s32)delay_threshold || diff_r1 < -(s32)delay_threshold) ? 1 : 0;

	u32 fp_idx = 5;
	u16 dram_freq = dmem_read16(PMU_DMEM_DRAM_FREQ_OFF);

	for (u32 i = 0; i < 5; i++) {
		u16 curr = (i == 0) ? val_1a0 :
				   (i == 1) ? val_1a2 :
				   (i == 2) ? val_1a4 :
				   (i == 3) ? val_1a6 : val_1a8;
		u16 prev = (i == 0) ? 0 :
				   (i == 1) ? val_1a0 :
				   (i == 2) ? val_1a2 :
				   (i == 3) ? val_1a4 : val_1a6;

		if (dram_freq < 3200) {
			if (val_1aa < (curr * 2)) {
				fp_idx = i;
				break;
			}
			pmu_assert_or_halt(i != 4, 0x43 << 16);
		} else {
			if (i == 0) {
				if (!(flag_54 || flag_58)) {
					fp_idx = 0;
					break;
				}
			} else {
				s32 diff = (s32)curr - (s32)prev - (s32)val_1aa;
				if ((curr * 2) >= val_1aa && diff <= (s32)delay_threshold && diff >= -(s32)delay_threshold) {
					fp_idx = i;
					break;
				}
				pmu_assert_or_halt(i != 4, 0x47 << 16);
			}
		}
	}

	u32 target_csr_addr = PHY_REG_DBYTE_BASE | ((rank_shift | slice_param_off) << 1);
	u32 slice_reg_base = ((rank_shift | active_csr_offset) << 1);
	phy_write16(slice_reg_base | (PHY_REG_DBYTE_BASE | 0x015a), (u8)fp_idx);

	u16 fp_saved = phy_read16(target_csr_addr);
	u32 active_csr_addr = PHY_REG_DBYTE_BASE | (((rank_shift | active_csr_offset | 0xf) << 1));
	phy_write16(target_csr_addr, fp_saved | csr_step_off);

	u16 current_csr_val = phy_read16(active_csr_addr);
	u32 dbyte_reg_addr = slice_reg_base | (PHY_REG_DBYTE_BASE | 0x0028);
	phy_write16(active_csr_addr, (current_csr_val & ~3) | 1);

	u16 dbyte_status_reg = phy_read16(dbyte_reg_addr);
	u32 delay_cnt = ((dbyte_status_reg << 1) & 510) * 160;
	pmu_hw_timer_delay((delay_cnt << 2) + 0x28);

	current_csr_val &= ~1;
	phy_write16(target_csr_addr, fp_saved & ~csr_step_off);
	phy_write16(active_csr_addr, current_csr_val);

	u16 strobe_val = phy_read16(reg_20158);
	phy_write16(reg_20158, strobe_val | 1);
	phy_write16(reg_20158, strobe_val & ~1);

	u16 val_1ac = phy_read16((PHY_REG_DBYTE_BASE | 0x01ac) | rank_off13);
	phy_write16(slice_reg_base | (PHY_REG_DBYTE_BASE | 0x015e), val_1ac);
}

/**
 * pmu_cal_phase_delay_tracker_update() - Update LPDDR5 WCK clock calibration and phase delay tracker
 */
void pmu_cal_phase_delay_tracker_update(void)
{
	u8 flag = dmem_read8(PMU_DMEM_CLK_GATE_FLAG);

	if (flag & 0x20) {
		dmem_write16(PMU_DMEM_RANK_TAP_130, phy_read16((PHY_REG_MASTER_BASE | 0x0110)));
		dmem_write16(PMU_DMEM_RANK_TAP_132, phy_read16((PHY_REG_MASTER_BASE | 0x0112)));
		dmem_write16(PMU_DMEM_RANK_TAP_134, phy_read16((PHY_REG_MASTER_BASE | 0x0114)));
		dmem_write16(PMU_DMEM_RANK_TAP_136, phy_read16((PHY_REG_MASTER_BASE | 0x0118)));

		u32 slice_offset = dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET) << 1;
		dmem_write16(PMU_DMEM_RANK_TAP_13C, phy_read16((PHY_REG_MASTER_BASE | 0x0098) | slice_offset));
		dmem_write16(PMU_DMEM_RANK_TAP_138, phy_read16((PHY_REG_MASTER_BASE | 0x0094) | slice_offset));
		dmem_write16(PMU_DMEM_RANK_TAP_13A, phy_read16((PHY_REG_MASTER_BASE | 0x0096) | slice_offset));
	}

	u8 stream_regs_buf[18];
	memcpy(stream_regs_buf, (const void *)(PMU_DMEM_BASE | PMU_DMEM_WCK_INIT_STREAM), 18);
	pmu_phy_reg_stream_play(stream_regs_buf, 18);

	u16 wck_ctrl = (flag & 0x80) ? 0 : (flag & 1);
	phy_write16((PHY_REG_MASTER_BASE | 0x0116), wck_ctrl);
	phy_write16(PHY_REG_ANALOG_CFG0, dmem_read8(PMU_DMEM_CH0_RANK_EN));
	phy_write16(PHY_REG_ANALOG_CFG1, dmem_read8(PMU_DMEM_CH1_RANK_EN));

	if (dmem_read8(PMU_DMEM_CAL_DISABLE_19) == 0) {
		pmu_phy_cal_state_save();
		u32 active_slices = dmem_read32(PMU_DMEM_ACTIVE_SLICE_BITMAP);
		if (active_slices != 0) {
			for (u32 i = 0; i < active_slices; i++) {
				u32 reg = PHY_REG_WCK_DELAY + (i * 0x2000);
				phy_write16(reg, phy_read16(reg) & 0x0c);
			}
		}
	}

	pmu_phy_clk_toggle_settle();
	pmu_phy_pll_clock_timing_ctrl();

	if (dmem_read16(PMU_DMEM_DRAM_FREQ_OFF) < 1600)
		goto out;

	if (pmu_dmem_wck_sync_mode_check() != 0)
		goto out;

	u8 freq_mode = dmem_read8(PMU_DMEM_FREQ_MODE);
	u32 csr_offset = dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET);
	u32 wck_mode_param, wck_reg_param;

	if (freq_mode < 2) {
		wck_mode_param = 2;
		wck_reg_param = csr_offset | 0x14;
	} else if (freq_mode < 4) {
		wck_mode_param = 2;
		wck_reg_param = 0x56;
	} else {
		wck_mode_param = 1;
		wck_reg_param = csr_offset | 87;
	}

	u32 slice_off = csr_offset << 1;
	u32 reg_3e1b2 = (PHY_REG_DBYTE_BCAST_BASE | 0x01b2);
	phy_write16(reg_3e1b2, 0x1000);

	u32 reg_3e01e = PHY_REG_DBYTE_BCAST_PARAM | slice_off;
	u32 reg_2001e = (PHY_REG_DBYTE_BASE | 0x001e) | slice_off;
	u16 val_2001e = phy_read16(reg_2001e);

	u32 csr_base_3c = PHY_REG_DBYTE_BASE | (wck_reg_param << 1);
	phy_write16(reg_3e01e, (val_2001e & ~7) | 6);

	u32 csr_base_34 = PHY_REG_DBYTE_BCAST_BASE | (wck_reg_param << 1);
	u16 val_3c = phy_read16(csr_base_3c);
	val_3c &= ~wck_mode_param;
	phy_write16(csr_base_34, val_3c);

	u16 val_20158 = phy_read16((PHY_REG_DBYTE_BASE | 0x0158));
	phy_write16(reg_3e1b2 - 90, val_20158 & ~1);

	u16 val_r13 = phy_read16(reg_2001e);
	phy_write16(reg_3e01e, val_r13 | 5);
	pmu_hw_timer_delay(0x3000);

	phy_write16(reg_3e01e, (val_r13 & ~5) | 4);

	u8 start_b6a = dmem_read8(PMU_DMEM_RANK0_SLICE_START);
	u8 bound_b6b = dmem_read8(PMU_DMEM_RANK_BOUNDARY);
	u8 start_b6c = dmem_read8(PMU_DMEM_RANK1_SLICE_START);
	u8 end_b6d = dmem_read8(PMU_DMEM_RANK1_SLICE_END);

	dmem_write8(PMU_DMEM_SLICE_START, start_b6a);
	dmem_write8(PMU_DMEM_SLICE_END, bound_b6b);

	if (bound_b6b != 255) {
		for (u32 rank_idx = 0; rank_idx <= bound_b6b; rank_idx++)
			pmu_cal_phase_delay_tracker_slice_update(rank_idx, csr_offset, wck_mode_param, wck_reg_param);
	}

	if (start_b6c <= end_b6d) {
		for (u32 rank_idx = start_b6c; rank_idx <= end_b6d; rank_idx++)
			pmu_cal_phase_delay_tracker_slice_update(rank_idx, csr_offset, wck_mode_param, wck_reg_param);
	}

	val_3c = phy_read16(csr_base_3c);
	phy_write16(csr_base_34, val_3c | wck_mode_param);

	val_2001e = phy_read16(reg_2001e);
	phy_write16(reg_3e01e, (val_2001e & ~3) | 1);

	u16 val_20028 = phy_read16((PHY_REG_DBYTE_BASE | 0x0028) | slice_off);
	u32 delay_count = ((val_20028 << 1) & 510) * 160;
	pmu_hw_timer_delay((delay_count << 2) + 0x28);

	val_2001e &= ~1;
	val_3c &= ~wck_mode_param;
	phy_write16(csr_base_34, val_3c);
	phy_write16(reg_3e01e, val_2001e);

out:
	pmu_cal_bist_lock_check();
}

/**
 * pmu_train_dispatcher() - Top-level PMU training stage sequencer for LPDDR5 PHY
 *
 * Sequences training stages based on SequenceCtrl mailbox word.
 */
void pmu_train_dispatcher(void)
{
	__builtin_arc_sr(2, 0x103);
	__builtin_arc_sr(1, 0x103);
	pmu_prof_stamp(0);

	phy_write16(PHY_REG_INIT_COMPLETE, 1);

	if (dmem_read8(PMU_DMEM_CLK_GATE_FLAG) & 1) {
		phy_write16(PHY_REG_PLL_STATUS, 0);
		phy_write16((PHY_REG_DBYTE_BCAST_BASE | 0x0034), 0);
		phy_write16(PHY_REG_PLL_CTRL, 0);
		phy_write16((PHY_REG_MASTER_BASE | 0x0154), 0);
		phy_write16(PHY_REG_PLL_CTRL, 3);
	}

	pmu_phy_clk_gate_sync_pulse();
	pmu_cal_metric_log(10, 0x02200000);
	pmu_cbt_step_stat_set();

	u8 pub_cmd = dmem_read8(PMU_DMEM_METRIC_COEFF_X);
	u16 pub_ctl = phy_read16(PHY_REG_PUB_CTL);
	pmu_cmd_code_map_log(pub_ctl, pub_cmd);

	u8 freq_mode = dmem_read8(PMU_DMEM_FREQ_MODE);
	u32 reg14 = (freq_mode == 0) ? (PHY_REG_APB_BASE | 0x120052) : (PHY_REG_APB_BASE | 0x12005c);
	u32 reg15 = (freq_mode == 0) ? (PHY_REG_APB_BASE | 0x120054) : (PHY_REG_APB_BASE | 0x12005e);

	dmem_write16(PMU_DMEM_METRIC_COEFF_X, 0x230b);
	phy_write16(reg14, 0x715d);
	phy_write16(reg15, 0x00ce);

	pmu_cal_metric_log(10, 0x02220001);
	pmu_cal_metric_log(10, 0x02230005,
			(u32)phy_read16((PHY_REG_PUB_CTL + 2)),
			(u32)phy_read16(PHY_REG_PUB_CTL),
			(u32)phy_read16((PHY_REG_PUB_CTL - 2)),
			(u32)phy_read16(PHY_REG_PUB_CTL),
			(u32)phy_read16((PHY_REG_MASTER_BASE | 0x01d8)));

	if (dmem_read16(PMU_DMEM_METRIC_COEFF_Z) != phy_read16(reg14) ||
		dmem_read16(PMU_DMEM_METRIC_COEFF_W) != phy_read16(reg15)) {
		pmu_assert_or_halt(0, 0x02240000);
	}

	u8 ch_cfg_04 = dmem_read8(PMU_DMEM_CHANNEL_CFG);
	u32 ch_offset = 0;
	if (!(ch_cfg_04 & 0x80)) {
		ch_offset = (ch_cfg_04 << 20) & 0x100000;
	}
	dmem_write32(PMU_DMEM_ACTIVE_CSR_OFFSET, ch_offset);

	u16 seq_ctrl = dmem_read16(PMU_DMEM_SEQUENCE_CTRL);
	u16 master_phy_status = phy_read16((PHY_REG_MASTER_BASE | 0x0012) | (ch_offset << 1));
	u8 clk_gate_flag = dmem_read8(PMU_DMEM_CLK_GATE_FLAG);

	u32 dram_type = dmem_read8(PMU_DMEM_DRAM_TYPE);
	u32 freq = dmem_read16(PMU_DMEM_DRAM_FREQ_OFF);
	u32 denom = dram_type * 220;
	u32 window = (freq + denom - 1) / denom;

	dmem_write8(PMU_DMEM_AC_STEP_FLAG, 1);
	dmem_write8(PMU_DMEM_SEARCH_WINDOW_SCALE, 4);
	dmem_write8(PMU_DMEM_SEARCH_WINDOW_STEPS, (u8)window);

	pmu_cal_search_win_init((u8 *)(PMU_DMEM_BASE | PMU_DMEM_SEARCH_WIN_COARSE));
	pmu_cal_search_win_init((u8 *)(PMU_DMEM_BASE | PMU_DMEM_SEARCH_WIN_FINE));

	/*
	 * Stage 0: Device Initialization & CBT Entry (PMU_SEQ_DEV_INIT - Bit 0)
	 * Initializes PHY PLLs, pad drivers, rank/slice tables, and runs Command Bus
	 * Training (CBT) entry sequence to put DRAM into calibration mode.
	 */
	if (seq_ctrl & PMU_SEQ_DEV_INIT) {
		u32 clk_sync_flag = (clk_gate_flag >> 7) & 1;

		pmu_cbt_entry_pll_ctrl();
		if (!(dmem_read8(PMU_DMEM_CAL_DISABLE_19) | clk_sync_flag))
			phy_write16(PHY_REG_PHY_STATUS, 2);

		pmu_cal_stage8_pad_init();
		pmu_dmem_rank_slice_table_setup();
		pmu_cal_metric_log(4, 0x02260000);
		pmu_cal_phase_delay_tracker_update();
		pmu_deskew_and_tracker_reset();
		pmu_cbt_coarse_step_pulse_seq(0, 4, 0);
		pmu_clk_timing_delay_latch(0, 1);
		pmu_cbt_exit_mission_handoff();
		pmu_cal_lane_window_margin_eval(clk_sync_flag);
		dmem_write16(PMU_DMEM_TRAIN_MODE, 1);
		pmu_dbyte_pin_map_table_init();
		if (clk_sync_flag == 0)
			pmu_cal_rank_lane_status_check();

		/*
		 * Stage 1: Command/Address (CA) Training (PMU_SEQ_LPCA_INIT - Bit 12)
		 * Trains the LPDDR5 Command/Address bus timing against the differential clock.
		 * When combined with PMU_SEQ_DEV_INIT, forms Fast Boot mode (0x1001).
		 */
		if (seq_ctrl & PMU_SEQ_LPCA_INIT) {
			pmu_cal_metric_log(4, 0x02270000);
			pmu_cal_stage1_pre_init();
			dmem_write16(PMU_DMEM_TRAIN_MODE, 0x1000);
			dmem_write16(PMU_DMEM_TRAIN_MODE, 0x9000);
			pmu_post_cmd_conditional_dispatch(0xd);
			if (clk_sync_flag == 0)
				pmu_multiparam_cal_dispatch();
		} else if (clk_sync_flag == 0) {
			pmu_multiparam_cal_dispatch();
		}

		pmu_deskew_and_tracker_reset();
		u32 clamp = pmu_delay_clamp(0x3a98, 0);
		u32 clamp_limit = ((u8)clamp > 8) ? (u8)clamp : 8;
		pmu_cal_sequence_pulse_send(0, 7, clamp_limit, 0, 0, 0, 0);
		pmu_clk_timing_delay_latch(0, 1);
		pmu_post_cmd_conditional_dispatch(0);
		dmem_write8(PMU_DMEM_CAL_ACTIVE_FLAG, 0);
		dmem_write16(PMU_DMEM_TRAIN_MODE, 0x8001);
	}
	pmu_prof_stamp(1);

	pmu_dbyte_dq_deskew_regs_save_and_ramp();

	u32 flag_bit6 = seq_ctrl & PMU_SEQ_SEARCH_WIN;
	u32 flag_bit3 = seq_ctrl & PMU_SEQ_STAGE_DQS;

	/*
	 * Stage 2, 3, 5 Pre-Pass:
	 * When PMU_SEQ_SEARCH_WIN (Stage 6) is active, Stages 2 (Vref), 3 (DCD), and 5 (DQS)
	 * are calibrated post-search on aligned WCK/CK clocks (saving ~514 ms duplicate run).
	 * If 2D search is disabled, execute them here.
	 */
	if (!flag_bit6) {
		if (seq_ctrl & PMU_SEQ_STAGE_VREF)
			pmu_cal_stage_post_process_vref(1);
		if (seq_ctrl & PMU_SEQ_STAGE_DCD)
			pmu_cal_stage_post_process_dcd();
		if (flag_bit3)
			pmu_cal_stage_post_process_dqs(1);
	}

	/*
	 * Stage 4: ZQ Calibration Pad Tuning (PMU_SEQ_ZQ_CAL - Bit 7)
	 * Matches transmitter output impedance (Ron) and on-die termination (ODT)
	 * against external 240-ohm calibration resistor.
	 */
	u32 flag_profile3 = 0;
	if (pmu_cal_profile_mode_get() == 3 && (seq_ctrl & PMU_SEQ_ZQ_CAL) && !(dmem_read8(PMU_DMEM_TRAIN_FEATURE_MASK) & 2)) {
		phy_write16(PHY_REG_DBYTE_BCAST_MODE | (ch_offset << 1), 0);
		flag_profile3 = 1;
	}

	/*
	 * Stage 6: High-Resolution 2D Eye Margin Search (PMU_SEQ_SEARCH_WIN - Bit 6)
	 * 2-dimensional scan across delay (X-axis) and Vref DAC voltage (Y-axis)
	 * to find the optimal sampling centroid.
	 */
	if (flag_bit6) {
		pmu_cal_metric_log(10, 0x020f0000);
		pmu_cal_search_win_init((u8 *)(PMU_DMEM_BASE | PMU_DMEM_SEARCH_WIN_FINE));
		dmem_write8(PMU_DMEM_CAL_ACTIVE_FLAG, 1);
		dmem_write16(PMU_DMEM_TRAIN_MODE, 0x0040);
		pmu_cal_vref_dac_step_adjust(0x13);
		dmem_write16(PMU_DMEM_TRAIN_MODE, 0x8040);
		pmu_post_cmd_conditional_dispatch(6);
		dmem_write8(PMU_DMEM_CAL_ACTIVE_FLAG, 0);
	}
	pmu_prof_stamp(2);

	u8 train_feature_mask = dmem_read8(PMU_DMEM_TRAIN_FEATURE_MASK);
	if (!(train_feature_mask & 1) && flag_bit6) {
		if (!(train_feature_mask & 8) && (seq_ctrl & PMU_SEQ_STAGE_VREF))
			pmu_cal_stage_post_process_vref(1);
		pmu_prof_stamp(3);
		if (!(train_feature_mask & 0x10) && (seq_ctrl & PMU_SEQ_STAGE_DCD))
			pmu_cal_stage_post_process_dcd();
		pmu_prof_stamp(4);
		if (flag_bit3 && !(train_feature_mask & 0x20))
			pmu_cal_stage_post_process_dqs(1);
		pmu_prof_stamp(5);
	} else {
		pmu_prof_stamp(3);
		pmu_prof_stamp(4);
		pmu_prof_stamp(5);
	}

	u32 flag_bit7 = seq_ctrl & PMU_SEQ_ZQ_CAL;
	if (flag_bit7) {
		if (dmem_read8(PMU_DMEM_WCK_SYNC_MODE) & 1) {
			pmu_cal_metric_log(10, 0x02100000);
			dmem_write16(PMU_DMEM_TRAIN_MODE, 0x0080);
			pmu_cal_vref_dac_step_adjust(0x11);
			dmem_write16(PMU_DMEM_TRAIN_MODE, 0x8080);
			pmu_post_cmd_conditional_dispatch(10);
		} else {
			pmu_cal_metric_log(10, 0x02110000);
		}
		if (!(seq_ctrl & PMU_SEQ_DESKEW_SWEEP))
			dmem_write16(PMU_DMEM_ITER_COUNT, 0);
	}

	train_feature_mask = dmem_read8(PMU_DMEM_TRAIN_FEATURE_MASK);
	if (!(train_feature_mask & 2) && flag_bit7) {
		if (flag_profile3)
			phy_write16(PHY_REG_DBYTE_BCAST_MODE | (ch_offset << 1), 4);
		if (flag_bit3)
			pmu_cal_stage_post_process_dqs(1);
	}

	/*
	 * Stage 7: Multi-Lane Window Sweep & Matrix Scan (PMU_SEQ_LANE_WIN_SWEEP - Bit 10)
	 * Cross-evaluates window margins across all 4 byte lanes and both ranks.
	 */
	if (seq_ctrl & PMU_SEQ_LANE_WIN_SWEEP) {
		pmu_cal_metric_log(10, 0x02120000);
		u8 clk_gate = dmem_read8(PMU_DMEM_CLK_GATE_FLAG);
		u8 cal_ctrl59 = dmem_read8(PMU_DMEM_LANE_SWEEP_CTRL);
		u32 count_10 = 0;
		if (cal_ctrl59 >> 6)
			count_10 = (1U << (clk_gate & 3)) - 1;

		dmem_write16(PMU_DMEM_TRAIN_MODE, 1024);
		pmu_cal_lane_window_sweep_coordinator(count_10, 1024);
		dmem_write16(PMU_DMEM_TRAIN_MODE, 0x8400);

		if (dmem_read8(PMU_DMEM_STAGE_STEP_MODE) == 0) {
			u8 fp_or = dmem_read8(PMU_DMEM_CH0_RANK_EN) | dmem_read8(PMU_DMEM_CH1_RANK_EN);
			for (u32 channel_idx = 0; channel_idx <= 1; channel_idx++) {
				if (fp_or & (1 << channel_idx)) {
					u32 lut = (0xe4 >> (channel_idx << 1)) & 3;
					u32 buf_sp1c[2] = {0, 0};
					pmu_slice_lcdl_delay_collect(0x1000040 + lut, buf_sp1c, 1);
					pmu_cal_metric_log(5, 0x02540000);
					pmu_cal_matrix_trace_dump(0, 1, (const u16 *)buf_sp1c);
				}
			}
		}
		pmu_post_cmd_conditional_dispatch(0xe);
	}

	/*
	 * Stage 8: BIST Test Pattern Search Window (PMU_SEQ_BIST_SEARCH_WIN - Bit 4)
	 * Drives hardware BIST patterns into DRAM to verify read/write data eye validity.
	 */
	if (seq_ctrl & PMU_SEQ_BIST_SEARCH_WIN) {
		pmu_cal_bist_search_win_setup(0);
	} else if (seq_ctrl == PMU_SEQ_DEV_INIT) {
		pmu_cal_bist_search_win_setup(1);
	}

	/*
	 * Stage 9: Per-Bit DQ Deskew Calibration (PMU_SEQ_DESKEW_SWEEP - Bit 8)
	 * Adjusts individual LCDL delay taps for each DQ bit (0..7) within every byte lane
	 * to eliminate trace-length skew and align all data bits with DQS strobe.
	 */
	u32 flag_bit8 = seq_ctrl & PMU_SEQ_DESKEW_SWEEP;
	if (flag_bit8) {
		pmu_cal_metric_log(10, 0x02140000);
		dmem_write16(PMU_DMEM_TRAIN_MODE, 256);
		pmu_cal_vref_dac_step_adjust(0x12);
		dmem_write16(PMU_DMEM_TRAIN_MODE, 0x8100);
		pmu_post_cmd_conditional_dispatch(0xb);
		dmem_write16(PMU_DMEM_ITER_COUNT, 0);

		if (!(dmem_read8(PMU_DMEM_TRAIN_FEATURE_MASK) & 4)) {
			if (seq_ctrl & PMU_SEQ_BIST_SEARCH_WIN) {
				pmu_cal_bist_search_win_setup(0);
			} else if (seq_ctrl != PMU_SEQ_DEV_INIT) {
				pmu_cal_bist_search_win_setup(1);
			}
		}
	}

	if (seq_ctrl == PMU_SEQ_DEV_INIT && dmem_read8(PMU_DMEM_CAL_DISABLE_19) == 0) {
		pmu_cal_metric_log(10, 0x02280000);
		pmu_cal_vref_dac_step_adjust(0xf);
	}

	if (seq_ctrl & PMU_SEQ_BIST_SEARCH_WIN) {
		u32 train_step_lim = (pmu_dmem_training_flags_eval() == 0) ? 0xff : 511;
		u32 train_lane_mask = (dmem_read8(PMU_DMEM_CLK_GATE_FLAG) & 4) ? 0 : 0xf;
		u16 val_42a = dmem_read16(PMU_DMEM_SAVED_CAL_STAT);
		if (val_42a & (1 << 5)) {
			pmu_cal_metric_log(10, 0x02150000);
			pmu_cal_multi_lane_deskew_sweep(2, train_lane_mask, train_step_lim);
			pmu_prof_stamp(6);
			if (dmem_read8(PMU_DMEM_TRAIN_PARAM_CTRL) & (1 << 4)) {
				pmu_cal_metric_log(4, 0x02160000);
				pmu_cal_multi_lane_deskew_sweep(3, train_lane_mask, train_step_lim);
			}
			pmu_prof_stamp(7);
		} else {
			pmu_prof_stamp(6);
			pmu_prof_stamp(7);
		}
	} else {
		pmu_prof_stamp(6);
		pmu_prof_stamp(7);
	}

	if (flag_bit3) {
		pmu_cal_stage_post_process_dqs(0);
		pmu_prof_stamp(8);
		u32 train_loop_lim = (pmu_dmem_training_flags_eval() == 0) ? 0xff : 511;
		u32 train_step_mask = (dmem_read8(PMU_DMEM_CLK_GATE_FLAG) & 4) ? 0 : 0xf;
		u16 val_42a = dmem_read16(PMU_DMEM_SAVED_CAL_STAT);
		if (val_42a & (1 << 6)) {
			pmu_cal_metric_log(10, 0x02170000);
			pmu_cal_multi_lane_deskew_sweep(1, train_step_mask, train_loop_lim);
			pmu_prof_stamp(9);
			pmu_cal_multi_lane_deskew_sweep(0, train_step_mask, train_loop_lim);
			pmu_prof_stamp(10);
		} else {
			pmu_prof_stamp(9);
			pmu_prof_stamp(10);
		}
	} else {
		pmu_prof_stamp(8);
		pmu_prof_stamp(9);
		pmu_prof_stamp(10);
	}

	/*
	 * Stage 10: Timing Margin Envelope Evaluation (PMU_SEQ_MARGIN_STEP - Bit 9)
	 * Measures left/right timing margin boundaries to verify operating guardband.
	 */
	if (seq_ctrl & PMU_SEQ_MARGIN_STEP)
		pmu_cal_margin_envelope_step_eval();
	pmu_prof_stamp(11);

	if (dmem_read8(PMU_DMEM_STAGE_STEP_MODE) != 0) {
		u8 fp_or = dmem_read8(PMU_DMEM_CH0_RANK_EN) | dmem_read8(PMU_DMEM_CH1_RANK_EN);
		for (u32 channel_idx = 0; channel_idx <= 1; channel_idx++) {
			if (fp_or & (1 << channel_idx)) {
				u32 lut = (0xe4 >> (channel_idx << 1)) & 3;
				pmu_lcdl_delay_profile_update(1, 0, lut, channel_idx);
				pmu_lcdl_delay_profile_update(0, 0, lut, channel_idx);
			}
		}
		pmu_dmem_stride_descriptor_read();
		if (dmem_read8(PMU_DMEM_HOST_MSG_BASE) & 0x20)
			pmu_cal_error_code_log();
	}

	pmu_cal_stage39_post_results();

	if (seq_ctrl != 1) {
		u8 d04_val = dmem_read8(PMU_DMEM_CHANNEL_CFG);
		if (d04_val & 0x80) {
			pmu_cal_phase_detector_edge_align();
		} else if ((d04_val & 0x30) == 0x20) {
			pmu_cal_dbyte_deskew_fine_tune(0);
		} else if ((d04_val & 0x30) == 0x10) {
			pmu_cal_dbyte_deskew_fine_tune(1);
		}
	}

	/*
	 * Training Completion & Handoff:
	 * Restore deskew registers, verify PLL lock status, log completion event,
	 * and write success code 0x7 (PMU_STATUS_SUCCESS) to host mailbox registers.
	 */
	pmu_dbyte_dq_deskew_regs_restore();
	phy_write16((PHY_REG_PUB_SEC_BASE | 0x012) | (ch_offset << 1), master_phy_status);
	pmu_cal_pll_status_check();
	pmu_cal_metric_log(10, 0x02290000);
	pmu_mailbox_post_cmd_dispatch(PMU_STATUS_SUCCESS);
	pmu_prof_stamp(12);

	return;
}

/**
 * pmu_cal_stage42_accum_dispatch() - Dispatch stage 42 delay parameter accumulation
 * @dram_type: DRAM memory type identifier (e.g. 3 = LPDDR5)
 * @count: Number of parameter entries to accumulate
 * @slice_base: Pointer to base slice parameter structure in DMEM
 */
void pmu_cal_stage42_accum_dispatch(u32 dram_type, u32 count, void *slice_base)
{
	u8 cand0[4][2];
	u8 cand1[4][2];
	u8 cand2[4][2];
	u8 cand3[4][2];
	u8 diff0[4];
	u8 diff1[4];
	u8 diff2[4];
	u8 diff3[4];
	struct dly_line_entry (*buf)[2] = (struct dly_line_entry (*)[2])slice_base;

	if (count == 0)
		return;
	if (count > 4)
		count = 4;

	for (u32 i = 0; i < count; i++) {
		u16 v0 = buf[0][i].val16;
		cand0[i][0] = (u8)(v0 >> 6);
		cand0[i][1] = (u8)(v0 & 0x3f);
		pmu_delay_tap_step_adjust(cand0[i], 0);

		u16 v1 = buf[1][i].val16;
		cand1[i][0] = (u8)(v1 >> 6);
		cand1[i][1] = (u8)(v1 & 0x3f);
		pmu_delay_tap_step_adjust(cand1[i], 0);

		pmu_eye_margin_step_align(cand0[i], cand1[i], 2);

		u16 v2 = buf[2][i].val16;
		cand2[i][0] = (u8)(v2 >> 6);
		cand2[i][1] = (u8)(v2 & 0x3f);
		pmu_delay_tap_step_adjust(cand2[i], 1);

		u16 v3 = buf[3][i].val16;
		cand3[i][0] = (u8)(v3 >> 6);
		cand3[i][1] = (u8)(v3 & 0x3f);
		pmu_delay_tap_step_adjust(cand3[i], 1);

		pmu_eye_margin_step_align(cand2[i], cand3[i], 2);

		diff0[i] = (cand0[i][1] < 65) ? (64 - cand0[i][1]) : (cand0[i][1] - 64);
		diff1[i] = (cand1[i][1] < 65) ? (64 - cand1[i][1]) : (cand1[i][1] - 64);
		diff2[i] = (cand2[i][1] < 65) ? (64 - cand2[i][1]) : (cand2[i][1] - 64);
		diff3[i] = (cand3[i][1] < 65) ? (64 - cand3[i][1]) : (cand3[i][1] - 64);
	}

	if (dram_type < 4 && dram_type != 2 && dmem_read8(PMU_DMEM_CAL_OVERRIDE_FLAG) == 0) {
		u32 max_grp1 = 0;
		u32 max_grp2 = 0;

		for (u32 k = 0; k < count; k++) {
			u32 m1 = (diff0[k] > diff1[k]) ? diff0[k] : diff1[k];
			u32 m2 = (diff2[k] > diff3[k]) ? diff2[k] : diff3[k];
			if (m1 > max_grp1)
				max_grp1 = m1;
			if (m2 > max_grp2)
				max_grp2 = m2;
		}

		for (u32 k = 0; k < count; k++) {
			pmu_cal_slice_delay_best_fit_select(k, max_grp1, max_grp2,
					cand2[k], cand3[k],
					cand0[k], cand1[k],
					(u8 *)slice_base);
		}
	} else {
		for (u32 k = 0; k < count; k++) {
			u32 thresh1 = (diff0[k] > diff1[k]) ? diff0[k] : diff1[k];
			u32 thresh2 = (diff2[k] > diff3[k]) ? diff2[k] : diff3[k];
			pmu_cal_slice_delay_best_fit_select(k, thresh1, thresh2,
					cand2[k], cand3[k],
					cand0[k], cand1[k],
					(u8 *)slice_base);
		}
	}
}

/*
 * pmu_cal_vref_dac_step_adjust:
 *
 * Calibrates receiver VREF DAC steps, timing margins, and delay line profiles
 * across active channels and ranks for specified calibration stage arg0.
 */
/**
 * pmu_cal_vref_dac_step_adjust() - Adjust PHY VREF DAC steps based on stage step selector
 * @stage_step: Stage step selector (0..8, 0x10, etc.)
 */
void pmu_cal_vref_dac_step_adjust(u32 stage_step)
{
	bool is_valid_stage_step = (stage_step != 0 && stage_step != 3 && (stage_step < 4 || stage_step > 7) && stage_step != 0x10);
	u32 vref_dac_mode = ((stage_step | 2) != 6) ? 1 : 0;
	u32 fp = 0xf;

	if (dmem_read8(PMU_DMEM_CLK_GATE_FLAG) & (1 << 2)) {
		u32 mode = pmu_cal_profile_mode_get();
		if ((stage_step & ~1) == 4 && mode == 3)
			fp = 5;
		else
			fp = 0;
	}

	u8 d09_low3 = dmem_read8(PMU_DMEM_DRAM_TYPE_FLAGS) & 7;
	if (d09_low3 != 0 && stage_step < 0x13 && (0x601f0 & (1 << stage_step))) {
		fp = (1 << d09_low3) - 1;
	}

	u8 ch_mask = dmem_read8(PMU_DMEM_CH0_RANK_EN) | dmem_read8(PMU_DMEM_CH1_RANK_EN);

	for (u32 channel = 0; channel < 2; channel++) {
		if (!(ch_mask & (1 << channel)))
			continue;

		u32 vref_accum_step = 0;
		u32 vref_target_step = 0;
		dmem_write8(PMU_DMEM_CAL_BYTE, (u8)channel);

		for (u32 rank = 0; rank < 2; rank++) {
			dmem_write8(PMU_DMEM_CAL_RANK, (u8)rank);
			u32 rank_vref_status = 0;

			if (stage_step < 0x13 && (0x701f8 & (1 << stage_step))) {
				vref_accum_step = pmu_dmem_training_flags_eval();
				rank_vref_status = vref_accum_step;
				u8 cal_param72 = dmem_read8(PMU_DMEM_VREF_DAC_CTRL);
				u8 cal_flag62 = dmem_read8(PMU_DMEM_VREF_FLAG_SIGNED);
				vref_target_step = (!(cal_param72 & 0x20) ? 1 : 0) | (cal_flag62 >> 7);
			}

			if (!pmu_channel_rank_avail_check(channel))
				continue;

			/* Stages 4..8 are restricted strictly to channel 0, rank 0 */
			if ((u32)(stage_step - 4) < 5) {
				if (channel != 0 || rank != 0)
					continue;
			}

			pmu_cal_strobe_pulse();
			u8 vref_eval_buf[80];
			memset(vref_eval_buf, 0, sizeof(vref_eval_buf));

			switch (stage_step) {
			case 0: {
				void (*p_afac)(u32, u8 *) = (void *)(uintptr_t)pmu_cal_timing_margin_envelope_scan;
				p_afac(channel, vref_eval_buf);
				pmu_cal_search_win_max_calc(rank, 1);
				break;
			}
			case 1: {
				void (*p_ea8c)(u32, u8 *) = (void *)(uintptr_t)pmu_cal_rank_deskew_matrix_commit;
				p_ea8c(channel, vref_eval_buf);
				break;
			}
			case 4:
			case 5:
			case 6:
			case 7: {
				u32 commit_param = (rank_vref_status == 0) ? 0xff : 511;
				u32 mode_cfg = 0;
				if ((stage_step & ~1) == 6) {
					if (dmem_read8(PMU_DMEM_BIST_PATTERN_LO) != 0 || dmem_read8(PMU_DMEM_BIST_PATTERN_HI) != 0 || dmem_read8(PMU_DMEM_DESKEW_OFFSET_OVERRIDE) != 0)
						mode_cfg = 7;
					else
						mode_cfg = 4;
				}
				pmu_cal_rank_param_commit(fp, mode_cfg, commit_param, vref_dac_mode);
				break;
			}
			case 8: {
				u8 cal_flag62 = dmem_read8(PMU_DMEM_VREF_FLAG_SIGNED);
				u8 cal_param72 = dmem_read8(PMU_DMEM_VREF_DAC_CTRL);
				u8 train_cfg96 = dmem_read8(PMU_DMEM_TRAIN_PARAM_CTRL);
				u32 cond = ((cal_flag62 >> 6) & 1) | ((cal_flag62 >> 7) & 1) |
						   ((train_cfg96 >> 6) & 1) | (!(cal_param72 & 0x20) ? 1 : 0);
				u32 step_limit = (cond == 0) ? 0xff : 511;
				u32 commit_param = ((train_cfg96 & 0x10) << 5) | step_limit;
				pmu_cal_rank_param_commit(fp, 0, commit_param, 2);
				break;
			}
			case 15: {
				u16 vref_steps_buf[4];
				pmu_cal_slice_step_diff_commit(channel, vref_steps_buf, 1);
				break;
			}
			case 16: {
				void (*p_aba8)(u32, u8 *, u32) = (void *)(uintptr_t)pmu_cal_slice_deskew_result_commit;
				p_aba8(channel, vref_eval_buf, rank_vref_status);
				pmu_cal_search_win_max_calc(rank, 0);
				break;
			}
			case 17: {
				void (*p_c9c0)(u32, u16, u32, u32) = (void *)(uintptr_t)pmu_cal_rank_rx_en_delay_commit;
				p_c9c0(channel, (u16)fp, rank_vref_status, 0x11);
				break;
			}
			case 18: {
				void (*p_c9c0)(u32, u16, u32, u32) = (void *)(uintptr_t)pmu_cal_rank_rx_en_delay_commit;
				p_c9c0(channel, (u16)fp, rank_vref_status, 0x12);
				break;
			}
			case 19: {
				void (*p_c4cc)(u32, u32) = (void *)(uintptr_t)pmu_cal_wck_ck_align_pulse_send;
				p_c4cc(channel, rank);
				break;
			}
			default:
				break;
			}
		}

		if ((u32)(stage_step - 0x11) < 2) {
			dmem_write16(PMU_DMEM_ITER_COUNT, dmem_read16(PMU_DMEM_ITER_COUNT) + 1);
		}

		pmu_phy_mode_cfg_dispatch(3);
		dmem_write8(PMU_DMEM_CAL_RANK, 0);

		if (dmem_read8(PMU_DMEM_STAGE_STEP_MODE) == 0) {
			u32 lut_code = channel + 1;
			pmu_lcdl_delay_profile_update(is_valid_stage_step, 0, lut_code, channel);
		}
	}

	if (dmem_read8(PMU_DMEM_CLK_GATE_FLAG) & (1 << 6)) {
		pmu_slice_deskew_pulse_seq(stage_step);
	}

	if ((u32)(stage_step - 0x11) < 2) {
		if (dmem_read8(PMU_DMEM_CLK_GATE_FLAG) & (1 << 6))
			pmu_slice_metric_scan_and_program();
	}

	if (stage_step == 3 && dmem_read8(PMU_DMEM_STAGE_STEP_MODE) == 0) {
		if (dmem_read8(PMU_DMEM_HOST_MSG_BASE) & (1 << 5))
			pmu_cal_error_code_log();
	}

	dmem_write8(PMU_DMEM_CAL_CTRL_0F, 0);
}

/**
 * pmu_cal_stage_post_process_dcd() - Execute Duty Cycle Distortion (DCD) stage post-processing
 */
void pmu_cal_stage_post_process_dcd(void)
{
	volatile u8 *dmem = (volatile u8 *)PMU_DMEM_BASE;
	__asm__("" : "+r"(dmem));
	u8 mode = dmem[0x8e];
	*(volatile u16 *)(dmem + PMU_DMEM_TRAIN_MODE) = 4;
	if ((mode & 3) != 0) {
		pmu_cal_metric_log(10, 0x02090000);
		pmu_cal_vref_dac_step_adjust(0);
		pmu_post_cmd_conditional_dispatch(2);
	} else {
		pmu_cal_metric_log(10, 0x02080000);
		pmu_cal_vref_dac_step_adjust(0x10);
		pmu_post_cmd_conditional_dispatch(5);
	}
	*(volatile u16 *)(dmem + PMU_DMEM_TRAIN_MODE) = 0x8004;
}

/**
 * pmu_cal_stage_post_process_vref() - Execute VREF calibration post-processing and adjust DAC steps
 * @vref_mode: VREF calibration mode (0 = step 2, non-zero = step 1)
 */
void pmu_cal_stage_post_process_vref(u32 vref_mode)
{
	volatile u8 *dmem = (volatile u8 *)PMU_DMEM_BASE;
	__asm__("" : "+r"(dmem));
	pmu_cal_metric_log(10, (vref_mode != 0) ? 0x020a0000 : 0x020b0000);
	*(volatile u16 *)(dmem + PMU_DMEM_TRAIN_MODE) = 2;
	pmu_cal_vref_dac_step_adjust((vref_mode == 0) ? 2 : 1);
	pmu_post_cmd_conditional_dispatch((vref_mode != 0) ? 1 : 0xfe);
	*(volatile u16 *)(dmem + PMU_DMEM_TRAIN_MODE) = 0x8002;
}

/**
 * pmu_cal_stage_post_process_dqs() - Execute DQS calibration post-processing with frequency threshold check
 * @freq_check_mode: Frequency check mode (0 enables 3200 MT/s check and DQS step adjustment)
 */
void pmu_cal_stage_post_process_dqs(u32 freq_check_mode)
{
	volatile u8 *dmem = (volatile u8 *)PMU_DMEM_BASE;
	__asm__("" : "+r"(dmem));
	u16 freq = *(volatile u16 *)(dmem + PMU_DMEM_DRAM_FREQ_OFF);
	*(volatile u16 *)(dmem + PMU_DMEM_TRAIN_MODE) = 8;

	u8 high_freq_flag = 0;
	if (freq >= PMU_FREQ_THRESHOLD_3200 && freq_check_mode == 0) {
		if ((dmem[0x0d] & (1 << 5)) == 0)
			high_freq_flag = 1;
	}
	dmem[PMU_DMEM_HIGH_FREQ_FLAG] = high_freq_flag;
	dmem[0x402] = (freq_check_mode == 0) ? 1 : 0;

	u8 mode = dmem[0x8e];
	if ((mode & 3) != 0) {
		if (freq_check_mode == 0) {
			pmu_cal_metric_log(10, 0x020d0000);
			pmu_cal_vref_dac_step_adjust(5);
			pmu_cal_vref_dac_step_adjust(4);
		} else {
			pmu_cal_metric_log(10, 0x020c0000);
			if (pmu_cal_ptr_tag_check(0, 0x020c0000) == 0)
				pmu_cal_vref_dac_step_adjust(7);
			pmu_cal_vref_dac_step_adjust(6);
		}
	} else {
		pmu_cal_metric_log(10, 0x020e0000);
	}

	pmu_post_cmd_conditional_dispatch((freq_check_mode == 0) ? 3 : 0xfd);
	dmem[0x402] = 0;
	*(volatile u16 *)(dmem + PMU_DMEM_TRAIN_MODE) = 0x8008;
}

/**
 * pmu_cal_rank_lane_status_check() - Check calibration status and populate lane active masks in DMEM
 *
 * Return: 1 if lane is active and ready, 0 otherwise.
 */
u32 pmu_cal_rank_lane_status_check(void)
{
	volatile u8 *dmem = (volatile u8 *)PMU_DMEM_BASE;
	__asm__("" : "+r"(dmem));

	u32 rank_count = (pmu_cal_status_148_read() >> 6) & 0x3;
	u32 num_channels = *(volatile u32 *)(dmem + PMU_DMEM_CAL_PARAM_410);

	for (u32 rank = 0; rank < rank_count; rank++) {
		for (u32 ch = 0; ch < num_channels; ch++) {
			u8 rank_mask = ch ? dmem[0x40] : dmem[0x25];
			if (!(rank_mask & (1 << rank)))
				continue;

			u32 ch_rank_bit = 1 << (rank + (ch << 1));
			u32 base_off = (ch << 4) + (rank << 3);
			u32 lane_start = ch ? dmem[0xb6c] : dmem[0xb6a];
			u32 lane_end = ch ? dmem[0xb6d] : dmem[0xb6b];

			u32 map_idx = 0;
			volatile u8 *slice_map_pri = dmem + PMU_DMEM_RANK_SLICE_MAP + base_off;
			volatile u8 *slice_map_sec = dmem + PMU_DMEM_RANK_SLICE_COUNTS + base_off;
			u32 bit_query = pmu_dmem_active_lane_query(ch, rank);
			u8 dmem_101 = dmem[0x101];

			for (u32 cur = lane_start; cur <= lane_end; cur += 2) {
				u8 next_val = cur + 1;

				if (bit_query) {
					slice_map_pri[map_idx++] = cur;
					slice_map_pri[map_idx++] = next_val;
				} else if (ch_rank_bit & dmem_101) {
					slice_map_pri[map_idx] = next_val;
					slice_map_sec[map_idx++] = cur;
				} else {
					slice_map_sec[map_idx] = next_val;
					slice_map_pri[map_idx++] = cur;
				}
			}
		}
	}

	return 1;
}

/**
 * pmu_rank_slice_bit_active() - Check whether rank slice bit is active in calibration mapping
 * @dmem: Pointer to base of PMU DMEM parameter space
 * @rank_bit: Rank selection bitmask to evaluate
 *
 * Return: 0 on success, negative error code on failure.
 */
int pmu_rank_slice_bit_active(volatile u8 *dmem, u32 rank_bit)
{
	u32 shift = (dmem[0xb6b] < dmem[0xb68]) ? 2 : 0;
	return (dmem[0x1c] >> shift) & rank_bit;
}

/**
 * pmu_cal_pulse_settle_delay() - Apply calibration descriptor and wait for pulse settling
 * @desc: Pointer to calibration descriptor structure
 */
void pmu_cal_pulse_settle_delay(const void *desc)
{
	pmu_delay_us(0x3a98, 8);
	pmu_cal_descriptor_apply(desc);
	pmu_delay_us(0x30d40, 8);
}

/**
 * pmu_cal_multi_rank_timing_step() - Execute multi-rank calibration sequence and dispatch clock delay pulses
 * @rank_mode: Rank and channel selection mask
 * @steps_table: Pointer to calibrated timing delay steps array in DMEM
 * @eval_error_mask: Non-zero to evaluate lane error mask at sequence conclusion
 * @skip_mask_low: Bitmask of indices 0..31 in sequence map to skip
 * @skip_mask_high: Bitmask of indices 32..63 in sequence map to skip
 * @check_settle_flags: Flag controlling settle delay checks on first iteration
 */
void pmu_cal_multi_rank_timing_step(u32 rank_mode, u32 steps_table, u32 eval_error_mask, u32 skip_mask_low, u32 skip_mask_high, u32 check_settle_flags)
{
	volatile u8 *dmem = (volatile u8 *)PMU_DMEM_BASE;
	__asm__("" : "+r"(dmem));

	pmu_deskew_and_tracker_reset();

	u32 rank_bit = 1 << (1 & ~rank_mode);

	for (u32 i = 0; i < 21; i++) {
		u8 cal_idx = dmem[PMU_DMEM_CAL_SEQ_MAP + i];
		u32 bit = 1U << (cal_idx & 31);
		if ((cal_idx < 32) ? (bit & skip_mask_low) : (bit & skip_mask_high))
			continue;

		u8 step_val = ((const u8 *)(uintptr_t)steps_table)[cal_idx];

		if (i == 4) {
			pmu_cal_sequence_pulse_send(0, 0x10, 4, 0, 0, rank_mode, 0);
		}

		pmu_cal_sequence_pulse_send(0, 6, 0, step_val, cal_idx, rank_mode, 0);
		pmu_cbt_coarse_step_pulse_seq(0, 0x22, 0);

		u16 mult = 0;
		if (i == 0) {
			if (check_settle_flags != 0) {
				u8 flag_b1  = ((const u8 *)(uintptr_t)steps_table)[1];
				u8 flag_b14 = ((const u8 *)(uintptr_t)steps_table)[0x14];
				if ((flag_b1 & 0x08) || (flag_b14 & 0x0f) != 0) {
					pmu_clk_timing_delay_latch(0, 1);
					pmu_deskew_and_tracker_reset();
					pmu_cal_pulse_settle_delay((const void *)(uintptr_t)steps_table);
					continue;
				}
			}
			mult = 200;
		} else {
			if (i == 15 && pmu_rank_slice_bit_active(dmem, rank_bit)) {
				pmu_cal_sequence_pulse_send(0, 6, 0, step_val | 0x80, cal_idx, rank_mode, 0);
				pmu_cbt_coarse_step_pulse_seq(0, 0x22, 0);
			}
			if ((i | 1) == 0x11) {
				mult = 250;
			}
		}
		if (mult) {
			u8 dram_type = dmem[0x08];
			if (!dram_type)
				dram_type = 2;
			u16 freq = *(volatile u16 *)(dmem + PMU_DMEM_DRAM_FREQ_OFF);
			u32 result = ((u32)freq * mult) / ((u32)dram_type * 2000) + 1;
			pmu_cbt_coarse_step_pulse_seq(0, (u16)result, 0);
		}
	}

	pmu_cbt_coarse_step_pulse_seq(0, 0x41, 0);
	pmu_clk_timing_delay_latch(0, 1);

	if (eval_error_mask != 0) {
		pmu_dbyte_lane_error_mask_calc(0, rank_mode);
	}
}

/**
 * pmu_cal_pulse_sync_strobe() - Issue a 6-step CBT coarse pulse synchronization strobe
 *
 * Pulses a 6-step CBT coarse phase strobe using a fixed unit step size.
 */
__attribute__((noinline)) void pmu_cal_pulse_sync_strobe(void)
{
	pmu_cbt_coarse_step_pulse_seq(0, 6, 1);
}

/**
 * pmu_cal_multi_rank_step_commit() - Commit multi-rank calibration steps and dispatch 3-phase CBT pulses
 * @rank_mode: Rank and channel selection mask
 * @steps_table: Pointer to calibrated timing delay steps array in DMEM
 * @eval_error_mask: Non-zero to evaluate lane error mask at sequence conclusion
 * @skip_mask_low: Bitmask of indices 0..31 in sequence map to skip
 * @skip_mask_high: Bitmask of indices 32..63 in sequence map to skip
 * @pulse_coarse_pre: Non-zero to pulse pre-coarse step before each calibration step
 * @check_settle_flags: Flag controlling settle delay checks on first iteration
 */
void pmu_cal_multi_rank_step_commit(u32 rank_mode, u32 steps_table, u32 eval_error_mask, u32 skip_mask_low, u32 skip_mask_high, u32 pulse_coarse_pre, u32 check_settle_flags)
{
	volatile u8 *dmem = (volatile u8 *)PMU_DMEM_BASE;
	__asm__("" : "+r"(dmem));

	pmu_ac_lane_profile_setup();
	u32 rank_sel = rank_mode;
	pmu_deskew_and_tracker_reset();
	pmu_cbt_coarse_step_pulse_seq(0, 5, 1);

	u32 rank_bit = 1 << (1 & ~rank_sel);

	for (u32 i = 0; i < 21; i++) {
		u8 cal_idx = dmem[PMU_DMEM_CAL_SEQ_MAP + i];
		u32 bit = 1U << (cal_idx & 31);
		if ((cal_idx < 32) ? (bit & skip_mask_low) : (bit & skip_mask_high))
			continue;

		u8 step_val = ((const u8 *)(uintptr_t)steps_table)[cal_idx];

		if (pulse_coarse_pre != 0) {
			pmu_cbt_coarse_step_pulse_seq(0, 0x1e, 1);
		}

		if (i == 4) {
			pmu_cal_pulse_sync_strobe();
			pmu_cal_sequence_pulse_send(0, 0x2c, 0, 0, 0, rank_sel, 0);
			pmu_cal_sequence_pulse_send(0, 0x2d, 0, 0, 0, rank_sel, 0);
			pmu_cal_pulse_sync_strobe();
		}

		pmu_dbyte_cal_step_latch(cal_idx, step_val, rank_sel);

		if (i == 0) {
			if (check_settle_flags != 0) {
				u8 flag_b1  = ((const u8 *)(uintptr_t)steps_table)[1];
				u8 flag_b14 = ((const u8 *)(uintptr_t)steps_table)[0x14];
				if ((flag_b1 & 0x08) || (flag_b14 & 0x0f) != 0) {
					pmu_cbt_3phase_pulse_seq();
					pmu_clk_timing_delay_latch(0, 1);
					pmu_deskew_and_tracker_reset();
					pmu_cbt_coarse_step_pulse_seq(0, 5, 1);
					pmu_cal_pulse_settle_delay((const void *)(uintptr_t)steps_table);
				}
			}
			pmu_cbt_coarse_step_pulse_seq(0, 0x18, 1);
		} else if (i == 15 && pmu_rank_slice_bit_active(dmem, rank_bit)) {
			pmu_cal_pulse_sync_strobe();
			pmu_dbyte_cal_step_latch(cal_idx, step_val | 0x80, rank_sel);
		}

		pmu_cal_pulse_sync_strobe();
	}

	pmu_cbt_3phase_pulse_seq();
	pmu_clk_timing_delay_latch(0, 1);

	if (eval_error_mask != 0) {
		pmu_dbyte_lane_error_mask_calc(1, rank_sel);
	}

	pmu_phy_profile_param_program();
}
