// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2026 Alastair D'Silva <alastair@d-silva.org>
 *
 * Allwinner A733 Synopsys DDR PHY PMU Hardware Timer, PLL & Broadcast Helpers
 */

#include "pmu_fw.h"

static inline void pmu_phy_clk_gate_clear(void);
static void pmu_phy_pll_lock_poll(u32 timeout);
static void pmu_profile_param_stride_cfg(u32 mode);
static inline void pmu_phy_clk_gate_set(void);

/**
 * pmu_freq_ratio_mult() - Compute scaled DRAM frequency ratio multiplier
 * @mult: Multiplier factor applied to DRAM frequency
 *
 * Return: (((u32)freq * @mult) / ((u32)type * 2000U)) + 1U.
 */
u32 pmu_freq_ratio_mult(u32 mult)
{
	u32 freq = dmem_read16(PMU_DMEM_DRAM_FREQ_OFF);
	u32 type = dmem_read8(PMU_DMEM_DRAM_FREQ_OFF + 2U);

	if (type == 0U)
		type = 2U;

	return ((freq * mult) / (type * 2000U)) + 1U;
}

/**
 * pmu_phy_pll_lock_init() - Initialize CBT PLL lock and control registers
 *
 * Sets bit 0 of PHY register 0x90040144 into 0x900401f0, executes 4-tick and
 * 31-tick hardware timer delays, and writes 0x0001 to 0x90040144.
 */
void pmu_phy_pll_lock_init(void)
{
	u16 val = *(__volatile__ u16 *)PHY_REG_CBT_CTRL;

	*(__volatile__ u16 *)PHY_REG_CBT_PLL = val | 0x0001U;
	pmu_hw_timer_delay(4U);
	pmu_hw_timer_delay(31U);
	*(__volatile__ u16 *)PHY_REG_CBT_CTRL = 0x0001U;
}

/**
 * pmu_phy_dbyte_broadcast_enable() - Enable DBYTE broadcast mode after PLL lock
 *
 * Executes 31-tick and 4-tick hardware timer delays, clears PHY register
 * 0x900401f0 to 0, and sets bit 1 of PHY register 0x90040144.
 */
void pmu_phy_dbyte_broadcast_enable(void)
{
	u16 val;

	pmu_hw_timer_delay(31U);
	pmu_hw_timer_delay(4U);
	*(__volatile__ u16 *)PHY_REG_CBT_PLL = 0x0000U;
	val = *(__volatile__ u16 *)PHY_REG_CBT_CTRL;
	*(__volatile__ u16 *)PHY_REG_CBT_CTRL = val | 0x0002U;
}

/**
 * pmu_hw_timer_delay() - Execute rollover-safe ARC auxiliary RTC delay
 * @count: Requested raw delay count
 *
 * Scales @count by the shift factor in DCCM 0x80000404, ensures the ARC
 * auxiliary RTC counter (0x104/0x105) is enabled, performs a fast APB dummy
 * read turnaround when ticks < 16, or spins across the 64-bit counter with
 * 32-bit rollover handling when ticks >= 16.
 */
noinline void pmu_hw_timer_delay(u32 count)
{
	u32 s = pmu_sdata()->timer_shift & 31U;
	u32 ticks = (count == 0U) ? 0U : ((count + 1U) >> s);
	u32 start_lo;
	u32 start_hi;
	u32 headroom;

	if ((__builtin_arc_lr(PMU_AUX_RTC_CTRL) & 1U) == 0U) {
		u32 two = 2U;
		u32 one = 1U;

		__builtin_arc_sr(two, PMU_AUX_RTC_CTRL);
		__builtin_arc_sr(one, PMU_AUX_RTC_CTRL);
	}

	if (ticks < 16U) {
		(void)*(__volatile__ u16 *)PHY_REG_TIMER_DUMMY;
		return;
	}

	start_lo = __builtin_arc_lr(PMU_AUX_RTC_LOW);
	start_hi = __builtin_arc_lr(PMU_AUX_RTC_HIGH);
	headroom = ~start_lo;

	if (headroom < ticks) {
		u32 rem = ticks - headroom;

		while (__builtin_arc_lr(PMU_AUX_RTC_LOW) >= start_lo)
			;
		while (__builtin_arc_lr(PMU_AUX_RTC_LOW) < rem)
			;
	} else {
		u32 target = start_lo + ticks;

		while (__builtin_arc_lr(PMU_AUX_RTC_LOW) < target &&
		       __builtin_arc_lr(PMU_AUX_RTC_HIGH) == start_hi)
			;
	}
}

/**
 * pmu_delay_us() - Convert microsecond/unit delay into timer ticks and wait
 * @count: Delay duration count
 * @unit: Delay unit selector passed to pmu_freq_delay_step_calc()
 */
void pmu_delay_us(u32 count, u32 unit)
{
	pmu_hw_timer_delay(pmu_freq_delay_step_calc(count, unit));
}

/**
 * pmu_cal_strobe_pulse_init() - Reset PHY strobe registers and DCCM 0x458 markers
 * Return: Always 0.
 */
u32 pmu_cal_strobe_pulse_init(void)
{
	pmu_phy_deskew_dly0_set(0U);
	pmu_phy_deskew_dly1_set(0U);
	pmu_cal_marker_pair_set(0U);
	pmu_cal_marker_clear();
	return 0U;
}

/**
 * pmu_phy_pll_lock_poll() - Poll PHY clock gate status 0x90040042 with timeout
 * @timeout: Maximum poll iterations before raising assertion 0x00240000
 */
static noinline void pmu_phy_pll_lock_poll(u32 timeout)
{
	u32 locked = 0U;
	u32 i;

	for (i = 0U; i < timeout; i++) {
		if (*(__volatile__ u16 *)PHY_REG_CLK_GATE_STAT != 0U) {
			locked = 1U;
			break;
		}
	}

	if (!locked)
		pmu_assert_or_halt(0U, PMU_ERR_PLL_LOCK_TIMEOUT);
}

/**
 * pmu_cal_slice_step_diff_latch() - Latch slice step difference and timing strobe
 * @timing: 16-bit timing parameter written to PHY 0x9004004e
 * @wait_ack: Non-zero to poll PHY 0x90040042 (timeout 25000) and clear 0x90040040
 */
void pmu_cal_slice_step_diff_latch(u32 timing, u32 wait_ack)
{
	pmu_phy_clk_timing_set((u16)(timing));
	pmu_hw_timer_delay(6U);
	pmu_phy_deskew_dly1_set((u16)(pmu_cal_stride_get()) - 1U);
	pmu_phy_clk_gate_set();
	if (wait_ack != 0U) {
		pmu_phy_pll_lock_poll(25000U);
		pmu_phy_clk_gate_clear();
	}
}

/**
 * pmu_cal_slice_timing_latch() - Latch slice timing value and strobe PHY 0x90040040
 * @timing: 16-bit timing parameter written to PHY 0x9004004e
 * @wait_ack: Non-zero to poll PHY 0x90040042 (timeout 25000) and clear 0x90040040
 */
void pmu_cal_slice_timing_latch(u32 timing, u32 wait_ack)
{
	pmu_phy_clk_timing_set((u16)(timing));
	pmu_hw_timer_delay(6U);
	pmu_phy_clk_gate_set();
	if (wait_ack != 0U) {
		pmu_phy_pll_lock_poll(25000U);
		pmu_phy_clk_gate_clear();
	}
}

/**
 * pmu_cal_delay_line_accum_dispatch() - Compute frequency-scaled delay with floor
 * @count: Raw delay duration count passed to pmu_freq_delay_step_calc()
 * @limit: Minimum step floor returned when computed ticks are smaller
 *
 * Return: max(pmu_freq_delay_step_calc(@count, 0), @limit).
 */
u32 pmu_cal_delay_line_accum_dispatch(u32 count, u32 limit)
{
	u32 ticks = pmu_freq_delay_step_calc(count, 0U);

	return (ticks < limit) ? limit : ticks;
}

/**
 * pmu_freq_delay_step_calc() - Convert delay duration and unit into timer steps
 * @count: Requested delay duration
 * @unit: Calibration delay unit multiplier
 *
 * Scales @count by DRAM frequency (DCCM 0x80000006) and rounding bias
 * (DCCM 0x80000008), using overflow-safe division for @count > 200000, and
 * enforces the minimum calibration delay floor from DCCM 0x80000a7c/0x80000b64.
 *
 * Return: Computed timer step count.
 */
noinline u32 pmu_freq_delay_step_calc(u32 count, u32 unit)
{
	u32 f = dmem_read16(PMU_DMEM_DRAM_FREQ_MHZ);
	u32 t = dmem_read8(PMU_DMEM_PLL_RATIO);
	u32 cycles;
	u32 cycle_delay;
	u32 cal_delay;

	if (t == 0U)
		t = 2U;

	if (count > 200000U)
		cycles = (count / 100000U) * f;
	else
		cycles = (count * f) / 100000U;
	cycles /= 20U;

	cycle_delay = (cycles + t + 1U) / t;
	cal_delay = unit + 1U;
	if (dmem_read8(PMU_DMEM_AC_CAL_ACTIVE) != 0U &&
	    (dmem_read8(PMU_DMEM_CLK_GATE_FLAG) & 8U) == 0U) {
		u32 w_steps = dmem_read8(PMU_DMEM_SEARCH_WINDOW_STEPS);

		if (w_steps != 0U)
			cal_delay = w_steps * unit *
				    dmem_read8(PMU_DMEM_SEARCH_WINDOW_SCALE);
	}

	return (cal_delay < cycle_delay) ? cycle_delay : cal_delay;
}

/**
 * pmu_clk_gate_handoff() - Hand off PHY PLL clock gate and wait 5000 units
 *
 * Clears PHY register 0x90040062, spins until PHY status register 0x90040042 is
 * non-zero, clears PHY clock gate register 0x90040040, and delays 5000 units.
 */
void pmu_clk_gate_handoff(void)
{
	u32 timeout = 100000U;

	phy_write16(PHY_REG_PLL_CLK_CTRL, 0x0000U);
	while (phy_read16(PHY_REG_CLK_GATE_STAT) == 0U) {
		if (--timeout == 0U)
			pmu_assert_or_halt(0U, PMU_ERR_CLK_GATE_TIMEOUT);
		pmu_delay_us(1U, 0U);
	}
	phy_write16(PHY_REG_CLK_GATE, 0x0000U);
	pmu_delay_us(5000U, 0U);
}

/**
 * pmu_phy_analog_cfg_mode_set() - Program channel analog receiver bias registers
 * @mode: Channel mode selector (0: Ch0 only, 1: Ch1 only, >=2: both Ch0 and Ch1)
 */
void pmu_phy_analog_cfg_mode_set(u32 mode)
{
	u8 m = (u8)mode;
	u16 v0 = (m != 1U) ? dmem_read8(PMU_DMEM_CH0_ANALOG_EN) : 0U;
	u16 v1 = (m != 0U) ? dmem_read8(PMU_DMEM_CH1_ACTIVE_FLAG) : 0U;

	phy_write16(PHY_REG_CH0_ANALOG_PWR, v0);
	phy_write16(PHY_REG_CH1_ANALOG_PWR, v1);
}

/**
 * pmu_cal_pll_lock_settle_spin() - Interrogate AC and DBYTE PLL lock status
 * @out_dbyte_pll: Output pointer receiving DBYTE PLL lock mask (0x900201c0 & 0x01ff)
 * @out_ac_pll: Output pointer receiving AC PLL lock mask (0x9006017e & 0x01ff)
 */
void pmu_cal_pll_lock_settle_spin(u16 *out_dbyte_pll, u16 *out_ac_pll)
{
	__volatile__ u16 *p1 = (__volatile__ u16 *)PHY_REG_AC_PLL_CTRL;
	__volatile__ u16 *p2;
	u32 v0;
	u32 v4;
	u32 saved;

	v0 = p1[3];
	v4 = p1[5];
	saved = (v4 << 16) | v0;
	p1[5] = (u16)(v4 & ~0x001fU);
	p1[3] = (u16)(v0 | 0x0800U);
	p1[3] = (u16)(v0 | 0x0c00U);
	pmu_hw_timer_delay(10U);
	p2 = (__volatile__ u16 *)PHY_REG_AC_PLL_CTRL;
	p2[3] = (u16)(saved | 0x0800U);
	*out_ac_pll = *(__volatile__ const u16 *)PHY_REG_AC_PLL_LOCK &
		      0x01ffU;
	*out_dbyte_pll = *(__volatile__ const u16 *)PHY_REG_DBYTE_PLL_LOCK &
			 0x01ffU;
	p2[3] = (u16)saved;
	p2[5] = (u16)(saved >> 16);
}

/**
 * pmu_cal_seq_table_timed_replay() - Replay 6-byte [addr, data] sequence records
 * @seq_table: Pointer to array of 6-byte [u32 addr, u16 data] sequence records
 * @total_bytes: Total byte length of @seq_table (divided by 6 for record count)
 */
void pmu_cal_seq_table_timed_replay(const void *seq_table, u32 total_bytes)
{
	const u8 *ptr = (const u8 *)seq_table;
	u32 count = total_bytes / 6U;
	u32 i;

	for (i = 0U; i < count; i++) {
		u32 addr = (u32)ptr[0] | ((u32)ptr[1] << 8) |
			   ((u32)ptr[2] << 16) | ((u32)ptr[3] << 24);
		u16 data = (u16)((u16)ptr[4] | ((u16)ptr[5] << 8));

		ptr += 6U;
		if (addr == 0xffffffffUL)
			pmu_hw_timer_delay(data);
		else
			*(__volatile__ u16 *)(PHY_REG_PUB_BASE |
					      (addr << 1)) = data;
	}
}

/**
 * pmu_phy_pll_cfg_quad_write_stream() - Write 4 PLL words and append shadow trace
 * @reg_offset: Register offset shifted left by 1 into 0x90140000
 * @p0: First 16-bit parameter word
 * @p1: Second 16-bit parameter word
 * @p2: Third parameter word (low 16 bits written to +4, bits [18:16] to tag)
 * @flags: Control flags shifted left by 3 into the 4th tag halfword
 */
void pmu_phy_pll_cfg_quad_write_stream(u32 reg_offset, u16 p0, u16 p1, u32 p2,
				       u32 flags)
{
	__volatile__ u16 *reg =
		(__volatile__ u16 *)(PHY_REG_SHADOW_TRACK_BASE |
				     (reg_offset << 1));
	u16 *trace = (u16 *)(*(__volatile__ u32 *)PMU_DMEM_SHADOW_TRACE_PTR);
	u16 tag = (u16)(((flags & 0xffU) << 3) | ((p2 >> 16) & 0x07U));

	reg[0] = p0;
	reg[1] = p1;
	reg[2] = (u16)p2;
	reg[3] = tag;

	trace[0] = p0;
	trace[1] = p1;
	trace[2] = (u16)p2;
	trace[3] = tag;
	*(__volatile__ u32 *)PMU_DMEM_SHADOW_TRACE_PTR = (u32)(trace + 4);
}

/**
 * pmu_phy_clk_phase_toggle_pulse() - Toggle PHY clock phase and settle PLL aligners
 *
 * Pulses the master PHY clock phase and clear strobes around 0x90040606, delays
 * for phase stabilization, and restores the original 8-bit configuration.
 */
void pmu_phy_clk_phase_toggle_pulse(void)
{
	__volatile__ u16 *p = (__volatile__ u16 *)PHY_REG_CLK_PHASE_BASE;
	u16 orig;

	orig = p[3];
	p[16] = 1U;
	p[3] = (orig & 0x00efU) | 0x0010U;
	p[16] = 0U;
	p[13] = 1U;
	p[13] = 0U;
	p[17] = 1U;
	pmu_delay_us(0x07bfa480UL, 0U);
	p[17] = 0U;
	p[32] = 0U;
	p[32] = 1U;
	p[32] = 0U;
	p[3] = (u8)orig;
}

/**
 * pmu_cal_pad_timing_delay_set() - Stream pad timing delay halfword frames
 * @bypass_min_clamp: Non-zero to force val = step regardless of @steps
 * @steps: Target delay steps
 * @use_fixed_unit: Non-zero for unit step 1, 0 for (DCCM 0x8000045a << 1)
 */
void pmu_cal_pad_timing_delay_set(u32 bypass_min_clamp, u32 steps,
				  u32 use_fixed_unit)
{
	u16 marker_val = pmu_sdata()->cal_marker_a;
	u16 buf[8];
	u32 step = (use_fixed_unit == 0U) ?
		   (u32)((u8)(pmu_sdata()->cbt_state << 1)) : 1U;
	u32 val, div;

	if (step == 0U)
		step = 1U;
	val = (bypass_min_clamp == 0U && steps > step) ? steps : step;
	div = val / step;
	u32 count;
	u16 v3 = 0U;
	u16 v7 = 0U;

	if (val != div * step)
		div++;
	count = (div - 1U) & 0xffffU;

	memset(buf, 0, sizeof(buf));
	if ((marker_val & 0x7U) == 4U) {
		memset((void *)pmu_tracker()->slot1, 0,
		       sizeof(pmu_tracker()->slot1));
		pmu_cal_marker_status_update();
		pmu_sdata()->cal_marker_a = marker_val + 4U;
	}

	while (count > 255U) {
		u16 s0 = 0x1fU << 10;

		buf[3] = (u16)(s0 - 0x100U);
		buf[7] = s0;
		pmu_cal_tracker_reset_and_apb_zero(buf, 8U);
		count -= 255U;
	}

	if (count != 0U) {
		v3 = (u16)(((count << 12) & 0x7000U) | 0x0b00U);
		v7 = (u16)((count << 7) & 0x7c00U);
	}
	buf[3] = v3;
	buf[7] = v7;

	pmu_cal_tracker_reset_and_apb_zero(buf, 8U);
}

/**
 * pmu_pll_quad_param_tuple_stream() - Stream 2x2 PLL quad parameter tuples
 * @param_array: Input 8-halfword parameter array
 * @out_ctr: In/out trace counter incremented per streamed tuple
 * @base_ptr: In/out base register index incremented per outer iteration
 */
void pmu_pll_quad_param_tuple_stream(const u16 *param_array, u16 *out_ctr,
				     u16 *base_ptr)
{
	u32 iter;

	for (iter = 0U; iter < 4U; iter++) {
		if ((dmem_read32(PMU_DMEM_PLL_GATE_CH0_ADDR) |
		     dmem_read16(PMU_DMEM_PLL_SEQ_EN_ADDR)) != 0U) {
			u32 target = ((u32)(*base_ptr) << 1) +
				     (iter & 1U) + 0x20800UL;

			pmu_phy_pll_cfg_quad_write_stream((u32)(*out_ctr) << 2,
							  param_array[0],
							  param_array[1],
							  target & 0x7ffffUL,
							  3U);
			*out_ctr += 1U;
		}
		param_array += 2;
		if ((iter & 1U) != 0U)
			*base_ptr += 1U;
	}
}

/**
 * pmu_phy_mode_cfg_dispatch() - Configure PHY analog bias and channel bounds for mode
 * @mode: Channel/rank dispatch mode (1 = Ch0, 2 = Ch1, else broadcast)
 */
void pmu_phy_mode_cfg_dispatch(u32 mode)
{
	__volatile__ u16 *apb = (__volatile__ u16 *)PHY_REG_CH0_ANALOG_PWR;
	u8 m = (u8)mode;
	u16 v0 = dmem_read8(PMU_DMEM_CH0_ANALOG_EN);
	u16 v1 = dmem_read8(PMU_DMEM_CH1_ACTIVE_FLAG);

	if (m == 2U)
		v0 = 0U;
	if (m == 1U)
		v1 = 0U;
	apb[0] = v0;
	apb[1] = v1;
	apb[12] = 0U;

	if (m == 2U) {
		dmem_write16(PMU_DMEM_CH_RANGE,
			     dmem_read16(PMU_DMEM_CH_SEC_RANGE));
		apb[8] = 0x000fU;
	} else if (m == 1U) {
		dmem_write16(PMU_DMEM_CH_RANGE,
			     dmem_read16(PMU_DMEM_CH_ACTIVE_RANGE));
		apb[8] = 0x00f0U;
	} else {
		apb[8] &= ~0x0011U;
		dmem_write8(PMU_DMEM_SLICE_START,
			    dmem_read8(PMU_DMEM_RANK0_SLICE_START));
		dmem_write8(PMU_DMEM_SLICE_END,
			    dmem_read8(PMU_DMEM_RANK1_SLICE_END));
		apb[8] = 0U;
	}

	pmu_profile_param_stride_cfg(mode);
}

/**
 * pmu_clk_ratio_pulse_trigger() - Trigger frequency-scaled 2000-step pad timing delay
 */
void pmu_clk_ratio_pulse_trigger(void)
{
	u32 scaled = pmu_freq_ratio_mult(2000U);

	pmu_cal_pad_timing_delay_set(0U, (u16)scaled, 0U);
}

/**
 * pmu_phy_pll_clock_divider_align() - Compute DBYTE slice bounds and active slice mask
 */
void pmu_phy_pll_clock_divider_align(void)
{
	u8 dq0 = dmem_read8(PMU_DMEM_CH0_DQ_WIDTH);
	u8 dq1 = dmem_read8(PMU_DMEM_CH1_DQ_WIDTH);
	u8 rank1_en = dmem_read8(PMU_DMEM_CH1_ACTIVE_FLAG);
	u8 d1c = dmem_read8(PMU_DMEM_DESKEW_LIM);
	u8 num_slices0 = (u8)((dq0 + 7U) >> 3);
	u8 end_slice0 = num_slices0 - 1U;
	u8 start_slice1 = num_slices0;
	u8 end_slice1;
	u8 mask;
	int i;
	u32 rank, ch;

	dmem_write8(PMU_DMEM_RANK0_SLICE_START, 0U);
	dmem_write8(PMU_DMEM_RANK_BOUNDARY, end_slice0);

	if (rank1_en != 0U) {
		start_slice1 = (u8)(pmu_sdata()->active_slice_cnt >> 1);
		num_slices0 = (u8)(((dq1 + 7U) >> 3) + start_slice1);
		end_slice1 = num_slices0 - 1U;
	} else {
		end_slice1 = end_slice0;
	}
	dmem_write8(PMU_DMEM_RANK1_SLICE_START, start_slice1);
	dmem_write8(PMU_DMEM_RANK1_SLICE_END, end_slice1);

	mask = dmem_read8(PMU_DMEM_ACTIVE_SLICE_MASK);
	for (i = 0; i <= (int)end_slice0; i++)
		mask |= (u8)(1U << i);
	dmem_write8(PMU_DMEM_ACTIVE_SLICE_MASK, mask);

	if (end_slice1 >= start_slice1) {
		for (i = (int)start_slice1; i <= (int)end_slice1; i++)
			mask |= (u8)(1U << i);
		dmem_write8(PMU_DMEM_ACTIVE_SLICE_MASK, mask);
	}

	for (rank = 0U; rank < 2U; rank++) {
		for (ch = 0U; ch < 2U; ch++) {
			u8 dq = (ch == 0U) ? dq0 : dq1;
			u8 shift = (u8)(ch * 2U);
			u8 slice_cnt = ((d1c >> shift) & (1U << rank)) ?
				       (u8)((dq + 7U) >> 3) :
				       (u8)((dq + 15U) >> 4);

			dmem_write8(PMU_DMEM_SLICE_CNT_LIMIT +
				    ch * 2U + rank, slice_cnt);
		}
	}
}

/**
 * pmu_profile_param_stride_cfg() - Broadcast profile delay across slices and pulse strobe
 * @mode: Channel/rank mode selector (3 = max across channels, else offset)
 */
static void pmu_profile_param_stride_cfg(u32 mode)
{
	u8 byte_idx;
	u32 val;
	u32 csr_ch_off;
	u32 count;
	u32 i;
	u32 reg;
	__volatile__ u16 *strobe;

	if (dmem_read16(PMU_DMEM_SEQ_CTRL) == 1U)
		return;

	byte_idx = dmem_read8(PMU_DMEM_ACTIVE_RANK);
	if (mode == 3U) {
		const u16 *tbl = (const u16 *)
			dmem_ptr(PMU_DMEM_CAL_TABLE + byte_idx * 4U);

		val = tbl[0];
		if (dmem_read8(PMU_DMEM_CH1_ACTIVE_FLAG) != 0U && tbl[1] > val)
			val = tbl[1];
	} else {
		u32 offset = (mode & ~1U) + byte_idx * 4U;

		val = dmem_read8(PMU_DMEM_CAL_TABLE + offset);
	}

	csr_ch_off = pmu_sdata()->active_csr_offset << 1;
	val = (u16)(s8)val;
	*(__volatile__ u16 *)(PHY_REG_MASTER_DELAY | csr_ch_off) = (u16)val;

	count = pmu_sdata()->active_slice_cnt;
	reg = PHY_REG_DBYTE_BASE | csr_ch_off;
	for (i = 0U; i <= count; i++, reg += PHY_DBYTE_SLICE_STRIDE)
		*(__volatile__ u16 *)reg = (u16)val;

	strobe = (__volatile__ u16 *)PHY_REG_PUB_CAL_STROBE;
	*strobe = 1U;
	pmu_hw_timer_delay(0x20U);
	*strobe = 0U;
}

/**
 * pmu_cal_tracker_csr_commit() - Commit multi-channel calibration step tracker to CSR stream
 * @tracker: Pointer to 22-halfword calibration step tracker structure
 *
 * Return: Updated 16-bit calibration marker from 0x8000045e.
 */
u32 pmu_cal_tracker_csr_commit(__volatile__ void *tracker)
{
	__volatile__ u16 *t = (__volatile__ u16 *)tracker;
	u8 mask = (u8)pmu_sdata()->active_slice_mask;
	u32 popcount = 0U;
	u16 marker_val;
	u8 step_ext_val;
	u8 ch_range_val;
	u16 has_step_ext;
	u16 sample[8];
	u32 base;
	u32 i;

	while (mask != 0U) {
		mask &= (u8)(mask - 1U);
		popcount++;
	}

	marker_val = pmu_sdata()->cal_marker_b;
	step_ext_val = pmu_sdata()->cbt_step_ext;
	ch_range_val = pmu_sdata()->ch_range_lim;
	has_step_ext = (step_ext_val != 0U) ? 1U : 0U;

	t[19] = has_step_ext;

	sample[0] = (t[3] & 0x7fU) | ((t[4] & 0x7fU) << 7) | (t[5] << 14);
	sample[1] = (t[6] & 0x0fU) | (t[7] << 4);
	sample[2] = (t[7] >> 12) & 1U;
	sample[3] = (u16)((u8)(t[2] << 7)) |
		    ((t[0] << 4) & 0x10U) |
		    ((t[1] << 5) & 0x60U) |
		    ((t[18] << 8) & 0x300U) |
		    ((t[20] << 11) & 0x800U) |
		    (has_step_ext << 10) |
		    ((t[17] << 12) & 0x7000U);

	sample[4] = (t[11] & 0x7fU) | ((t[12] & 0x7fU) << 7) | (t[13] << 14);
	sample[5] = (t[14] & 0x0fU) | (t[15] << 4);
	sample[6] = (t[15] >> 12) & 1U;
	sample[7] = (u16)((u8)(t[10] << 7)) |
		    ((t[8] << 4) & 0x10U) |
		    ((t[9] << 5) & 0x60U) |
		    ((t[16] << 8) & 0x300U) |
		    ((t[17] << 7) & 0x7c00U);

	if (ch_range_val != 0U) {
		s32 sum;

		if (ch_range_val != 1U)
			return (u32)marker_val;

		sum = (s8)popcount + pmu_sdata()->tracker_popcnt_acc;
		if (sum < 8) {
			*(__volatile__ u8 *)&pmu_sdata()->active_slice_mask = 0U;
			pmu_sdata()->tracker_popcnt_acc = sum % 8;
			return (u32)marker_val;
		}
	}

	base = PHY_REG_CSR_RESULT_STREAM | ((u32)marker_val << 1);
	for (i = 0U; i < 8U; i++)
		*(__volatile__ u16 *)(base + (i * 2U)) = sample[i];

	marker_val += 8U;
	pmu_sdata()->cal_marker_b = marker_val;

	if ((t[18] & 3U) != 0U)
		pmu_sdata()->step_mode_ovr = (u8)(1U & ~t[18]);

	if (ch_range_val == 1U) {
		s32 sum;

		pmu_tracker()->slot1_mode = 0U;
		sum = (s8)popcount + pmu_sdata()->tracker_popcnt_acc;
		*(__volatile__ u8 *)&pmu_sdata()->active_slice_mask = 0U;
		pmu_sdata()->tracker_popcnt_acc = sum % 8;
	}

	return (u32)marker_val;
}

/**
 * pmu_pll_clk_gate_cfg() - Program PLL clock timing and PUB transceiver gates
 */
void pmu_pll_clk_gate_cfg(void)
{
	u32 csr_sh1 = pmu_sdata()->active_csr_offset << 1;
	u16 val0 = *(__volatile__ const u16 *)(csr_sh1 | PHY_REG_DBYTE_XCVR_GATE);
	__volatile__ u16 *phy_base = (__volatile__ u16 *)PHY_REG_MASTER_BASE;
	u8 cal_mode_flag;

	*(__volatile__ u16 *)(csr_sh1 | PHY_REG_DBYTE_BCAST_XCVR_GATE) =
		(val0 & 0xff78U) | 6U;

	phy_base[0x00c2 >> 1] = 1U;

	cal_mode_flag = *(__volatile__ const u8 *)PMU_DMEM_CAL_MODE_FLAG_ADDR;
	if (cal_mode_flag != 0U) {
		pmu_delay_us(0U, 4U);
		phy_base[0x00c2 >> 1] = 0U;
		phy_base[0x01c2 >> 1] = 1U;
		phy_base[0x01c0 >> 1] = 1U;
		phy_base[0x0620 >> 1] = 0U;
		pmu_delay_us(0x989680U, 0U);
		phy_base[0x0186 >> 1] = 0U;
		phy_base[0x0400 >> 1] = 0x33U;
		phy_base[0x0144 >> 1] = 0U;
		pmu_hw_timer_delay(0x20U);
		phy_base[0x00f0 >> 1] = 1U;
		phy_base[0x0144 >> 1] = 2U;
		pmu_hw_timer_delay(1024U);
	} else {
		pmu_hw_timer_delay(4U);
		phy_base[0x0092 >> 1] = 496U;
		phy_base[0x00c2 >> 1] = 0U;
		phy_base[0x01c2 >> 1] = 0U;
		phy_base[0x0620 >> 1] = 0U;
		phy_base[0x01c0 >> 1] = 0U;
		phy_base[0x0186 >> 1] = 0U;
		pmu_delay_us(0xf4240U, 0U);
		phy_base[0x01c4 >> 1] = 3U;
		pmu_delay_us(0xf4240U >> 1, 0U);
		phy_base[0x01c4 >> 1] = 2U;
		pmu_delay_us(0x989680U, 0U);
		phy_base[0x01c4 >> 1] = 0U;
		phy_base[0x00f0 >> 1] = 1U;
		phy_base[0x0400 >> 1] = 0x33U;
		phy_base[0x0144 >> 1] = 0U;
		pmu_hw_timer_delay(6U);
		phy_base[0x00f0 >> 1] = 2U;
		pmu_delay_us(0U, 0x2710U);
		phy_base[0x0144 >> 1] = 2U;
	}

	phy_base[0x00f0 >> 1] = 0U;
	phy_base[0x0144 >> 1] = 0U;
	pmu_delay_us(0U, 0x20U);
	phy_base[0x0400 >> 1] = 0U;
	phy_base[0x01f0 >> 1] = 0U;
	phy_base[0x0144 >> 1] = 2U;
}

/**
 * pmu_channel_rank_avail_check() - Dispatch mode configuration for active channel
 * @channel: Target channel index (0 = Ch0, 1 = Ch1, >1 = broadcast)
 * @rank: Rank bit index to test in Ch0 (0x80000025) or Ch1 (0x80000040)
 *
 * Return: 1 if dispatched (mode 1, 2, or 3), or 0 if rank bit is inactive.
 */
u32 pmu_channel_rank_avail_check(u32 channel, u32 rank)
{
	u32 mode = 3U;

	if (channel <= 1U) {
		u8 flags = (channel == 0U) ?
			   dmem_read8(PMU_DMEM_CH0_ANALOG_EN) :
			   dmem_read8(PMU_DMEM_CH1_ACTIVE_FLAG);

		if (((flags >> rank) & 1U) == 0U)
			return 0U;
		mode = channel + 1U;
	}

	pmu_phy_mode_cfg_dispatch(mode);
	return 1U;
}

/**
 * pmu_phy_clk_gate_clear() - Clear PHY control register 0x90040040 to 0
 */
static inline void pmu_phy_clk_gate_clear(void)
{
	*(__volatile__ u16 *)PHY_REG_CLK_GATE = 0;
}

/**
 * pmu_phy_cal_stat_read() - Read 16-bit value from PHY register 0x90040148
 *
 * Return: 16-bit register value at 0x90040148.
 */
u16 pmu_phy_cal_stat_read(void)
{
	return *(__volatile__ u16 *)PHY_REG_CAL_STAT;
}

/**
 * pmu_phy_dbyte_cal_trig_pulse() - Execute 5-step strobe pulse on PHY reg 0x176
 * @slice: Target DBYTE slice index (0..3)
 * @bit_idx: Lane bit index (0..8, or 15 for all 9 lanes 0x1ff)
 *
 * Sequences 0 -> mask -> (mask | 0x200) -> mask -> 0 on the slice's PHY
 * register at 0x90020176 + (slice << 13).
 */
void pmu_phy_dbyte_cal_trig_pulse(u32 slice, u32 bit_idx)
{
	__volatile__ u16 *reg =
		(__volatile__ u16 *)(PHY_REG_DBYTE_CAL_TRIG + (slice << 13));
	u32 mask = (bit_idx == 15) ? 511U : (1U << bit_idx);
	u32 strobe = mask | (1U << 9);
	u16 zero = 0;

	*reg = zero;
	*reg = (u16)mask;
	*reg = (u16)strobe;
	*reg = (u16)mask;
	*reg = zero;
}

/**
 * pmu_phy_sec_cal_strobe_pulse() - Pulse PHY register 0x9005e140 high for 32 timer ticks
 */
void pmu_phy_sec_cal_strobe_pulse(void)
{
	*(__volatile__ u16 *)PHY_REG_PUB_CAL_STROBE = 1;
	pmu_hw_timer_delay(32);
	*(__volatile__ u16 *)PHY_REG_PUB_CAL_STROBE = 0;
}

/**
 * pmu_phy_cal_trig_pulse() - Pulse PHY register 0x90040146 high for 32 timer ticks
 */
void pmu_phy_cal_trig_pulse(void)
{
	*(__volatile__ u16 *)PHY_REG_CAL_TRIG = 1;
	pmu_hw_timer_delay(32);
	*(__volatile__ u16 *)PHY_REG_CAL_TRIG = 0;
}

/**
 * pmu_phy_soft_reset_pulse() - Toggle PHY register 0x900400f2 high then low
 */
void pmu_phy_soft_reset_pulse(void)
{
	__volatile__ u16 *p = (__volatile__ u16 *)PHY_REG_RESET_TRIGGER;

	*p = 1;
	*p = 0;
}

/**
 * pmu_phy_clk_gate_set() - Set PHY control register 0x90040040 to 1
 */
static inline void pmu_phy_clk_gate_set(void)
{
	*(__volatile__ u16 *)PHY_REG_CLK_GATE = 1;
}

/**
 * pmu_phy_deskew_dly0_set() - Write 16-bit value to channel-indexed PHY reg 0x90040044
 * @val: Halfword value to write
 */
void pmu_phy_deskew_dly0_set(u16 val)
{
	u32 ch_offset = pmu_sdata()->active_csr_offset;

	*(__volatile__ u16 *)(PHY_REG_DESKEW_DLY0 + (ch_offset << 1)) = val;
}

/**
 * pmu_phy_deskew_dly1_set() - Write 16-bit value to channel-indexed PHY reg 0x90040046
 * @val: Halfword value to write
 */
void pmu_phy_deskew_dly1_set(u16 val)
{
	u32 ch_offset = pmu_sdata()->active_csr_offset;

	*(__volatile__ u16 *)(PHY_REG_DESKEW_DLY1 + (ch_offset << 1)) = val;
}

/**
 * pmu_phy_clk_timing_set() - Write 16-bit value to PHY register 0x9004004e
 * @val: Halfword value to write
 */
void pmu_phy_clk_timing_set(u16 val)
{
	*(__volatile__ u16 *)PHY_REG_CLK_TIMING_CTRL = val;
}

/**
 * pmu_phy_clk_gate_sync_pulse() - Conditionally pulse PHY clock gate reg 0x9005e0a4
 *
 * When bit 0 of DCCM 0x80000001 is set, asserts PHY register 0x9005e0a4 to 1,
 * waits 10 hardware timer ticks, and clears it back to 0.
 */
void pmu_phy_clk_gate_sync_pulse(void)
{
	u8 flag = *(__volatile__ u8 *)PMU_DMEM_CLK_GATE_FLAG;

	if ((flag & 1) != 0) {
		*(__volatile__ u16 *)PHY_REG_SEC_CLK_GATE = 1;
		pmu_hw_timer_delay(10);
		*(__volatile__ u16 *)PHY_REG_SEC_CLK_GATE = 0;
	}
}

/**
 * pmu_phy_pll_cfg_quad_write() - Broadcast halfword mask to PHY 0x900e00c2..0x900e00c8
 * @v0: Halfword mask written to 0x900e00c2, 0x900e00c4, 0x900e00c6, and 0x900e00c8
 */
void pmu_phy_pll_cfg_quad_write(u16 v0)
{
	__volatile__ u16 *p = (__volatile__ u16 *)PHY_REG_PLL_CFG;

	p[0] = v0;
	p[1] = v0;
	p[2] = v0;
	p[3] = v0;
}
