// SPDX-License-Identifier: GPL-2.0+
/*
 * Synopsys DesignWare DDR PHY Training Firmware (LPDDR5)
 * PHY PLL control, clock gating, ratio latching, and frequency math
 * Target Microcontroller: Synopsys ARC EM4 (ARCv2 ISA, Code Density enabled)
 * SoC: Allwinner A733 (Sun60i) / LPDDR5 PHY (Type 9)
 */

#include "lpddr5_pmu_internal.h"

/**
 * pmu_timing_ctrl_set() - Configure PHY clock timing control register
 * @val: 16-bit timing control value (phase and delay enable bits)
 */
void pmu_timing_ctrl_set(u16 val)
{
	phy_write16(PHY_REG_CLK_TIMING_CTRL, val);
}

/**
 * pmu_clk_gate_enable() - Enable master PHY clock gating
 */
void pmu_clk_gate_enable(void)
{
	phy_write16(PHY_REG_CLK_GATE, 1);
}

/**
 * pmu_clk_gate_disable() - Disable master PHY clock gating
 */
void pmu_clk_gate_disable(void)
{
	phy_write16(PHY_REG_CLK_GATE, 0);
}

/**
 * pmu_freq_lt_3200_check() - Check if DRAM clock frequency is below 3200 MT/s threshold
 *
 * Return: 1 if DRAM data rate < 3200 MT/s, 0 otherwise.
 */
u32 pmu_freq_lt_3200_check(void)
{
	return (dmem_read16(PMU_DMEM_DRAM_FREQ_OFF) < PMU_FREQ_THRESHOLD_3200) ? 1 : 0;
}

/**
 * pmu_clk_timing_delay_latch() - Configure clock timing, latch delay line, and optionally wait for PLL
 * @timing: 16-bit clock timing configuration word
 * @wait_ack: Non-zero to poll PLL lock and disable clock gate
 */
void pmu_clk_timing_delay_latch(u16 timing, u32 wait_ack)
{
	pmu_timing_ctrl_set(timing);
	pmu_hw_timer_delay(6);
	u16 stride = pmu_cal_stride_get() - 1;
	pmu_deskew_dly1_set(stride);
	pmu_clk_gate_enable();
	if (wait_ack != 0) {
		pmu_phy_pll_lock_poll(25000);
		pmu_clk_gate_disable();
	}
}

/**
 * pmu_clk_timing_latch() - Configure clock timing, enable master clock gate, and optionally wait for PLL
 * @timing: 16-bit clock timing configuration word
 * @wait_ack: Non-zero to poll PLL lock and disable clock gate
 */
void pmu_clk_timing_latch(u16 timing, u32 wait_ack)
{
	pmu_timing_ctrl_set(timing);
	pmu_hw_timer_delay(6);
	pmu_clk_gate_enable();
	if (wait_ack != 0) {
		pmu_phy_pll_lock_poll(25000);
		pmu_clk_gate_disable();
	}
}

/**
 * pmu_clk_gate_handoff() - Hand off master PHY clock gating and settle
 */
void pmu_clk_gate_handoff(void)
{
	phy_write16(PHY_REG_PLL_CLK_CTRL, 0);
	while (phy_read16(PHY_REG_CLK_GATE_STAT) == 0)
		;
	phy_write16(PHY_REG_CLK_GATE, 0);
	pmu_delay_us(5000, 0);
}

/**
 * pmu_phy_pll_lock_poll() - Poll PHY PLL lock status bit until locked or timed out
 * @timeout: Maximum iteration count to poll before halting
 */
void pmu_phy_pll_lock_poll(int timeout)
{
	u32 ok = 0;
	volatile u16 *stat_reg = (volatile u16 *)(uintptr_t)PHY_REG_CLK_GATE_STAT;

	for (int i = 0; i < timeout; i++) {
		if (*stat_reg != 0) {
			ok = 1;
			break;
		}
	}

	pmu_assert_or_halt(ok, 9 << 18);
}

/**
 * pmu_cal_pll_status_check() - Check PLL lock and status for specified rank
 */
void pmu_cal_pll_status_check(void)
{
	const volatile u8 *p = (const volatile u8 *)(PMU_DMEM_BASE | PMU_DMEM_SEARCH_WIN_COARSE);
	for (u32 i = 0; i < 8; i++)
		pmu_cal_metric_log(4, 0x1a10001, p[i]);
	for (u32 i = 0; i < 8; i++) {
		u8 val = dmem_read8((PMU_DMEM_BASE | PMU_DMEM_WINDOW_STATE_52) + i);
		pmu_cal_metric_log(4, 0x1a20001, val);
	}
}

/**
 * pmu_phy_lock_status_read() - Read AC PLL and DBYTE lock status registers
 * @ac_lock: Output pointer for AC PLL lock status (PHY_REG_AC_PLL_LOCK_STAT)
 * @dbyte_lock: Output pointer for DBYTE lock status (PHY_REG_DBYTE_LOCK_STAT)
 */
void pmu_phy_lock_status_read(u16 *ac_lock, u16 *dbyte_lock)
{
	volatile u16 *p = (volatile u16 *)PHY_REG_PUB_GATE_CTRL;
	__asm__("" : "+r"(p));
	u16 v0 = p[0x06 / 2];
	u16 v4 = p[0x0a / 2];
	u16 lock_cfg = v0 | (1u << 11);

	p[0x0a / 2] = v4 & ~0x1f;
	p[0x06 / 2] = lock_cfg;
	p[0x06 / 2] = v0 | 0xc00;
	pmu_hw_timer_delay(10);
	p[0x06 / 2] = lock_cfg;
	*dbyte_lock = phy_read16(PHY_REG_AC_PLL_LOCK_STAT) & 0x1ff;
	*ac_lock = phy_read16(PHY_REG_DBYTE_LOCK_STAT) & 0x1ff;
	p[0x06 / 2] = v0;
	p[0x0a / 2] = v4;
}

/**
 * pmu_phy_clk_toggle_settle() - Toggle PHY clock sequence and assert PLL settling delay
 */
void pmu_phy_clk_toggle_settle(void)
{
	volatile u16 *base = (volatile u16 *)PHY_REG_CLK_PHASE_TOGGLE;
	u16 orig = base[3];
	base[0x10] = 1;
	base[3] = (orig & 0xef) | (1u << 4);
	base[0x10] = 0;
	base[0xd] = 1;
	base[0xd] = 0;
	base[0x11] = 1;
	pmu_delay_us(0x7bfa480, 0);
	base[0x11] = 0;
	base[0x20] = 0;
	base[0x20] = 1;
	base[0x20] = 0;
	base[3] = (u8)orig;
}

/**
 * pmu_phy_timing_delay_latch() - Configure PHY timing register and latch pulse delay
 * @enable_gate: Non-zero to set PHY_REG_DBYTE_BCAST_PULSE and pulse delay tap 4; 0 for tap 0
 * @delay_code: Timing delay parameter code sent in sequence pulse
 */
void pmu_phy_timing_delay_latch(u32 enable_gate, u32 delay_code)
{
	pmu_deskew_and_tracker_reset();
	u32 offset = (u32)dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET) << 1;
	uintptr_t reg = PHY_REG_DBYTE_BCAST_PULSE | offset;

	if (enable_gate != 0) {
		phy_write16(reg, 1);
		pmu_cal_sequence_pulse_send(0, 6, 0x22, 4, 0x2e, delay_code, 0);
	} else {
		pmu_cal_sequence_pulse_send(0, 6, 0x22, 0, 0x2e, delay_code, 0);
		phy_write16(reg, 0);
	}
	pmu_clk_timing_delay_latch(0, 1);
}

/**
 * pmu_freq_ratio_mult() - Compute frequency-scaled multiplier ratio
 * @mult: Base multiplier factor
 * @unused: Unused alignment parameter
 *
 * Return: Frequency-scaled ratio multiplier.
 */
u32 pmu_freq_ratio_mult(u32 mult, u32 unused)
{
	u32 freq = dmem_read16(PMU_DMEM_DRAM_FREQ_OFF);
	u32 dram_type = dmem_read8(PMU_DMEM_DRAM_TYPE);
	if (!dram_type)
		dram_type = 2;
	return ((freq * mult) / (dram_type * 2000)) + 1;
}

/**
 * pmu_clk_ratio_pulse_trigger() - Trigger clock ratio update pulse sequence
 */
void pmu_clk_ratio_pulse_trigger(void)
{
	u32 res = pmu_freq_ratio_mult(2000, 0);
	pmu_cbt_coarse_step_pulse_seq(0, (u16)res, 0);
}

/**
 * pmu_phy_pll_clock_timing_ctrl() - Execute PHY PLL and clock timing control sequence
 */
void pmu_phy_pll_clock_timing_ctrl(void)
{
	u32 csr_offset = dmem_read32(PMU_DMEM_ACTIVE_CSR_OFFSET);
	u32 csr_sh1 = csr_offset << 1;
	u16 val0 = phy_read16(csr_sh1 | (PHY_REG_DBYTE_BASE | 0x001e));
	phy_write16(csr_sh1 | PHY_REG_DBYTE_BCAST_PARAM, (val0 & 0xff78) | 6);

	volatile u16 *phy_base = (volatile u16 *)PHY_REG_MASTER_BASE;
	__asm__("" : "+r"(phy_base));

	phy_base[0x00c2 >> 1] = 1;

	u8 flag05 = dmem_read8(PMU_DMEM_PLL_BYPASS);
	if (flag05 != 0) {
		pmu_delay_us(0, 4);
		phy_base[0x00c2 >> 1] = 0;
		phy_base[0x01c2 >> 1] = 1;
		phy_base[0x01c0 >> 1] = 1;
		phy_base[0x0620 >> 1] = 0;
		pmu_delay_us(0x989680, 0);
		phy_base[0x0186 >> 1] = 0;

		phy_base[0x0400 >> 1] = 0x33;
		phy_base[0x0144 >> 1] = 0;
		pmu_hw_timer_delay(0x20);

		phy_base[0x00f0 >> 1] = 1;
		phy_base[0x0144 >> 1] = 2;
		pmu_hw_timer_delay(1024);
	} else {
		pmu_hw_timer_delay(4);
		phy_base[0x0092 >> 1] = 496;
		phy_base[0x00c2 >> 1] = 0;
		phy_base[0x01c2 >> 1] = 0;
		phy_base[0x0620 >> 1] = 0;
		phy_base[0x01c0 >> 1] = 0;
		phy_base[0x0186 >> 1] = 0;
		pmu_delay_us(0xf4240, 0);

		phy_base[0x01c4 >> 1] = 3;
		pmu_delay_us(0xf4240 >> 1, 0);

		phy_base[0x01c4 >> 1] = 2;
		pmu_delay_us(0x989680, 0);

		phy_base[0x01c4 >> 1] = 0;
		phy_base[0x00f0 >> 1] = 1;
		phy_base[0x0400 >> 1] = 0x33;
		phy_base[0x0144 >> 1] = 0;
		pmu_hw_timer_delay(6);

		phy_base[0x00f0 >> 1] = 2;
		pmu_delay_us(0, 0x2710);

		phy_base[0x0144 >> 1] = 2;
	}

	phy_base[0x00f0 >> 1] = 0;
	phy_base[0x0144 >> 1] = 0;
	pmu_delay_us(0, 0x20);

	phy_base[0x0400 >> 1] = 0;
	phy_base[0x01f0 >> 1] = 0;
	phy_base[0x0144 >> 1] = 2;
}

/**
 * pmu_cal_pll_lock_retry_poll() - Poll PLL lock status with retry count and log telemetry on failure
 * @tracker_ptr: Pointer offset to calibration tracker structure in DMEM
 *
 * Return: Poll retry lock status code.
 */
u32 pmu_cal_pll_lock_retry_poll(u32 tracker_ptr)
{
	volatile u8 *dmem = (volatile u8 *)PMU_DMEM_BASE;
	volatile u16 *t = (volatile u16 *)(uintptr_t)tracker_ptr;

	u8 v460 = dmem[0x460];
	u32 popcount = 0;
	while (v460 != 0) {
		v460 &= (v460 - 1);
		popcount++;
	}

	u16 marker_val = *(volatile u16 *)(dmem + PMU_DMEM_CAL_MARKER_B);
	u8 v468 = dmem[0x468];
	u8 v45b = dmem[0x45b];

	t[19] = (v45b != 0) ? 1 : 0;

	u16 regs[8];
	regs[0] = (t[3] & 0x7f) | ((t[4] & 0x7f) << 7) | (t[5] << 14);
	regs[1] = (t[6] & 0xf) | (t[7] << 4);
	regs[2] = (t[7] >> 12) & 1;
	s16 term3 = (s8)(u8)(t[2] << 7);
	regs[3] = (u16)(term3 | ((t[0] << 4) & 0x10) | ((t[1] << 5) & 0x60) |
					((t[18] << 8) & 0x300) | ((t[20] << 11) & 0x800) |
					(((v45b != 0) ? 1 : 0) << 10) | ((t[17] << 12) & 0x7000));
	regs[4] = (t[11] & 0x7f) | ((t[12] & 0x7f) << 7) | (t[13] << 14);
	regs[5] = (t[14] & 0xf) | (t[15] << 4);
	regs[6] = (t[15] >> 12) & 1;
	s16 term7 = (s8)(u8)(t[10] << 7);
	regs[7] = (u16)(term7 | ((t[8] << 4) & 0x10) | ((t[9] << 5) & 0x60) |
					((t[16] << 8) & 0x300) | ((t[17] << 7) & 0x7c00));

	if (v468 == 0) {
		/* Direct programming path */
	} else if (v468 == 1) {
		s32 sum = (s8)(u8)popcount + *(volatile s32 *)(dmem + PMU_DMEM_CBT_PARAM_464);
		if (sum < 8) {
			dmem[0x460] = 0;
			*(volatile s32 *)(dmem + PMU_DMEM_CBT_PARAM_464) = sum % 8;
			return (u16)marker_val;
		}
	} else {
		return (u16)marker_val;
	}

	u32 phy_addr = (PHY_REG_APB_BASE | 0x82000) | ((u32)marker_val << 1);
	for (u32 i = 0; i < 8; i++) {
		*(volatile u16 *)(uintptr_t)(phy_addr + (i << 1)) = regs[i];
	}
	marker_val += 8;
	*(volatile u16 *)(dmem + PMU_DMEM_CAL_MARKER_B) = marker_val;

	if ((t[18] & 3) != 0) {
		dmem[0x469] = (t[18] & 1) ? 0 : 1;
	}

	if (v468 == 1) {
		*(volatile u16 *)(dmem + PMU_DMEM_CBT_LOCK_STATUS) = 0;
		s32 sum = (s8)(u8)popcount + *(volatile s32 *)(dmem + PMU_DMEM_CBT_PARAM_464);
		dmem[0x460] = 0;
		*(volatile s32 *)(dmem + PMU_DMEM_CBT_PARAM_464) = sum % 8;
	}

	return (u16)marker_val;
}

/**
 * pmu_freq_delay_step_calc() - Calculate delay step size based on DRAM operating frequency
 * @val: DRAM operating frequency in MT/s or clock ratio parameter
 * @min_delay: Minimum delay step lower clamp bound
 *
 * Return: Clamped frequency delay step.
 */
u32 pmu_freq_delay_step_calc(u32 val, u32 min_delay)
{
	volatile u8 *dmem = (volatile u8 *)PMU_DMEM_BASE;
	__asm__("" : "+r"(dmem));
	u16 freq = *(volatile u16 *)(dmem + PMU_DMEM_DRAM_FREQ_OFF);
	u8 dram_type = dmem[0x08];
	u32 cycles;

	if (val >= 0x30d41) {
		u32 temp = (u32)(((u64)(val >> 5) * 0x0a7c5ac5ULL) >> 39);
		cycles = (temp * freq) / 20;
	} else {
		cycles = (u32)(((u64)(val * freq) * 0x431bde83ULL) >> 51);
	}

	u32 cycle_delay = (cycles + dram_type + 1) / dram_type;
	u32 cal_delay = min_delay + 1;

	if (dmem[0xa7c] != 0 && (dmem[PMU_DMEM_CLK_GATE_FLAG] & (1 << 3)) == 0) {
		u8 cal_stat64 = dmem[PMU_DMEM_SEARCH_WINDOW_STEPS];
		if (cal_stat64 != 0) {
			u8 cal_stat65 = dmem[PMU_DMEM_SEARCH_WINDOW_SCALE];
			cal_delay = cal_stat64 * min_delay * cal_stat65;
		}
	}

	return (cal_delay > cycle_delay) ? cal_delay : cycle_delay;
}

/**
 * pmu_rate_tier_calc() - Calculate DRAM data rate tier code from clock frequency
 * @val: DRAM clock frequency or data rate in MT/s
 *
 * Return: Rate tier code byte ((high << 4) | low).
 */
u32 pmu_rate_tier_calc(u32 val)
{
	u32 low = (val <= 331) ? 0 : (val <= 531) ? 1 : (val <= 799) ? 2 : 3;
	u32 high = (val <= 799) ? (low + 2) : (val <= 1799) ? 5 : (val <= 2199) ? 6 : 7;
	return (high << 4) | low;
}
