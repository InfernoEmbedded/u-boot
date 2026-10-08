// SPDX-License-Identifier: GPL-2.0+
/*
 * Synopsys DesignWare DDR PHY Training Firmware (LPDDR5)
 * Low-level I/O, reset vector, hardware delays, and runtime primitives
 * Target Microcontroller: Synopsys ARC EM4 (ARCv2 ISA, Code Density enabled)
 * SoC: Allwinner A733 (Sun60i) / LPDDR5 PHY (Type 9)
 */

#include "lpddr5_pmu_internal.h"

/**
 * pmu_irq_halt() - Halt ARC EM4 execution on unhandled interrupt
 *
 * Traps unexpected hardware interrupts and halts the ARC EM4 core.
 */
__attribute__((naked)) void pmu_irq_halt(void)
{
	__asm__ volatile (
		"nop\n"
		"nop\n"
		"nop\n"
		"nop\n"
		"flag 1\n"
	);
}

/**
 * pmu_default_exception_handler() - Dump exception diagnostics and halt execution
 *
 * Writes diagnostic signature, exception cause (ECR), exception fault
 * address (EFA), and core registers to PMU DCCM base and halts core.
 */
__attribute__((naked)) void pmu_default_exception_handler(void)
{
	__asm__ volatile (
		"mov     r2, 0x80000000\n"
		"mov     r3, 0x1234dead\n"
		"st      r3, [r2]\n"
		"add_s   r2, r2, 4\n"
		"lr      r3, [1024]\n"
		"st      r3, [r2]\n"
		"add_s   r2, r2, 4\n"
		"lr      r3, [1027]\n"
		"st      r3, [r2]\n"
		"add_s   r2, r2, 4\n"
		"st      r0, [r2]\n"
		"add_s   r2, r2, 4\n"
		"st      r1, [r2]\n"
		"add_s   r2, r2, 4\n"
		"st      sp, [r2]\n"
		"add_s   r2, r2, 4\n"
		"st      fp, [r2]\n"
		"add_s   r2, r2, 4\n"
		"mov     r3, 0xabcddead\n"
		"st      r3, [r2]\n"
		"flag    1\n"
		"nop_s\n"
	);
}

/**
 * pmu_start() - ARC EM4 reset entry point and stack initialization
 *
 * Clears CPU registers, sets up stack pointer (SP) at 0x80010000 and global
 * pointer (GP) at 0x80000400, invokes pmu_train_main(), and halts on return.
 */
__attribute__((naked)) void pmu_start(void)
{
	__asm__ volatile (
		"mov_s   r0, 0\n"
		"mov_s   r1, 0\n"
		"mov_s   r2, 0\n"
		"mov_s   r3, 0\n"
		"mov_s   r10, 0\n"
		"mov_s   r11, 0\n"
		"mov_s   r12, 0\n"
		"mov_s   r13, 0\n"
		"mov_s   r14, 0\n"
		"mov_s   r15, 0\n"
		"mov_s   gp, 0\n"
		"mov_s   fp, 0\n"
		"mov_s   sp, 0\n"
		"mov     ilink, 0\n"
		"mov     r30, 0\n"
		"mov_s   blink, 0\n"
		"mov_s   sp, 0x80010000\n"
		"mov_s   gp, 0x80000400\n"
		"mov_s   fp, 0\n"
		"jl      pmu_train_main\n"
		"1:\n"
		"flag    1\n"
		"nop_s\n"
		"b_s     1b\n"
	);
}

void (* const pmu_reset_vector[20])(void) __attribute__((used, section(".vectors"))) = {
	pmu_start,
	pmu_default_exception_handler,
	pmu_default_exception_handler,
	pmu_default_exception_handler,
	pmu_default_exception_handler,
	pmu_default_exception_handler,
	pmu_default_exception_handler,
	pmu_default_exception_handler,
	pmu_default_exception_handler,
	pmu_default_exception_handler,
	pmu_default_exception_handler,
	pmu_default_exception_handler,
	pmu_default_exception_handler,
	pmu_default_exception_handler,
	pmu_default_exception_handler,
	pmu_default_exception_handler,
	pmu_irq_halt,
	pmu_irq_halt,
	pmu_irq_halt,
	pmu_irq_halt,
};

/*
 * memset - Memory set routine: fills count bytes at dest with val
 */
__attribute__((used)) void *memset(void *dest, int val, size_t count)
{
	u8 *d = (u8 *)dest;
	while (count--)
		*d++ = (u8)val;
	return dest;
}

/*
 * memcpy - Memory copy routine: copies count bytes from src to dest
 */
__attribute__((used)) void *memcpy(void *dest, const void *src, size_t count)
{
	u8 *d = (u8 *)dest;
	const u8 *s = (const u8 *)src;
	while (count--)
		*d++ = *s++;
	return dest;
}

/**
 * pmu_phy_clk_gate_sync_pulse() - Synchronize and pulse PHY clock gating
 *
 * If clock gating flag is set in DMEM, delays and pulses secondary
 * clock gate sync register.
 */
void pmu_phy_clk_gate_sync_pulse(void)
{
	if (dmem_read8(PMU_DMEM_CLK_GATE_FLAG) & 1) {
		pmu_hw_timer_delay(10);
		phy_write16(PHY_REG_PUB_CLK_GATE_SYNC, 1);
		phy_write16(PHY_REG_PUB_CLK_GATE_SYNC, 0);
	}
}

/**
 * pmu_phy_reset_pulse() - Pulse PHY reset trigger register
 *
 * Pulses PHY reset at PHY_REG_RESET_TRIGGER (1 then 0).
 */
void pmu_phy_reset_pulse(void)
{
	phy_write16(PHY_REG_RESET_TRIGGER, 1);
	phy_write16(PHY_REG_RESET_TRIGGER, 0);
}

/**
 * pmu_cal_stride_get() - Read calibration tracker stride parameter from DMEM
 *
 * Return: Calibration tracker stride parameter in delay steps.
 */
u16 pmu_cal_stride_get(void)
{
	return dmem_read16(PMU_DMEM_CAL_MARKER_A) >> 2;
}

/**
 * pmu_dram_scaled_div_round() - Scaled integer rounding division based on DRAM type divisor
 * @val: Scaled dividend value to round and divide
 *
 * Return: Scaled rounded quotient.
 */
s32 pmu_dram_scaled_div_round(s32 val)
{
	u32 denom = (u32)dmem_read8(PMU_DMEM_DRAM_TYPE) << 7;
	if (val >= 1)
		return (val + denom - 1) / (s32)denom;
	return -((-val) / (s32)denom);
}

/**
 * pmu_dmem_train_param_nibble_get() - Extract training parameter nibble from DMEM
 *
 * Return: 4-bit training parameter nibble from DMEM.
 */
u32 pmu_dmem_train_param_nibble_get(void)
{
	return dmem_read8(PMU_DMEM_TRAIN_PARAM_CTRL) & PMU_TRAIN_PARAM_NIBBLE_MASK;
}

/**
 * pmu_hdr_addr_15b_decode() - Decode 15-bit address offset from 2-byte header
 * @ptr: Pointer to 2-byte header buffer containing packed address bits
 *
 * Return: Decoded 15-bit header address offset.
 */
u32 pmu_hdr_addr_15b_decode(const u8 *ptr)
{
	return ((u32)ptr[0] << 7) + ptr[1];
}

/**
 * pmu_hdr_addr_14b_decode() - Decode 14-bit address offset from 2-byte header
 * @ptr: Pointer to 2-byte header buffer containing packed address bits
 *
 * Return: Decoded 14-bit header address offset.
 */
u32 pmu_hdr_addr_14b_decode(const u8 *ptr)
{
	return ((u32)ptr[0] << 6) + ptr[1];
}

/**
 * pmu_2b_identity_lut() - Look up 2-bit lane index from identity permutation table
 * @val: 2-bit selector index (0..3)
 *
 * Return: 2-bit permutation index.
 */
u32 pmu_2b_identity_lut(u32 val)
{
	u32 shift = (val << 1) & 6;
	return (PMU_LUT_2B_IDENTITY_E4 >> shift) & 3;
}

/**
 * pmu_cal_param_unpack() - Unpack packed 32-bit calibration parameter into 4 bytes
 * @val: Packed 32-bit calibration parameter word
 * @out: Destination 4-byte array to receive unpacked parameter bytes
 */
void pmu_cal_param_unpack(u32 val, u8 *out)
{
	out[1] = val & PMU_LCDL_FINE_MASK;
	out[0] = (u8)(((val >> 7) & 7) + ((val >> 6) & 1));
}

/**
 * pmu_dly_line_repack() - Combine packed delay line fields into single register word
 * @val: Packed delay line fields containing coarse and fine delay taps
 *
 * Return: Repacked delay line register value.
 */
u32 pmu_dly_line_repack(u32 val)
{
	return (val & PMU_LCDL_FINE_MASK) + (val & 0x40) + ((val >> 1) & 0x1c0);
}

/**
 * pmu_dly_line_unpack() - Unpack delay line parameter into coarse and fine delay taps
 * @val: Packed delay line parameter word
 * @out: Output 2-byte array (out[0] = coarse delay, out[1] = fine delay)
 */
void pmu_dly_line_unpack(u32 val, u8 *out)
{
	out[1] = val & PMU_LCDL_FINE_MASK;
	out[0] = (u8)(val >> PMU_LCDL_COARSE_SHIFT);
}

/**
 * pmu_delay_tap_step_adjust() - Adjust coarse and fine delay taps based on parity condition
 * @delay: 2-byte delay pair (delay[0] = coarse tap, delay[1] = fine tap)
 * @odd_parity: Parity check mode (0 = even coarse tap, 1 = odd coarse tap)
 */
void pmu_delay_tap_step_adjust(u8 *delay, u32 odd_parity)
{
	u8 val = delay[0];
	if (odd_parity == 0) {
		if (val == 0 || (val & 1))
			return;
	} else {
		if (!(val & 1))
			return;
	}
	delay[0] = val - 1;
	delay[1] += PMU_LCDL_NOMINAL_MIDPOINT;
}

/**
 * pmu_delay_us() - Busy-wait calibrated delay loop
 * @count: Duration count for the delay loop
 * @unit: Timebase scaling factor multiplier
 */
void pmu_delay_us(u32 count, u32 unit)
{
	pmu_hw_timer_delay(pmu_freq_delay_step_calc(count, unit));
}

/**
 * pmu_delay_clamp() - Clamp delay count against an upper bound
 * @count: Requested delay count
 * @max_limit: Maximum permissible delay limit to clamp against
 *
 * Return: Clamped delay value not exceeding max_limit.
 */
u32 pmu_delay_clamp(u32 count, u32 max_limit)
{
	u32 calc = pmu_freq_delay_step_calc(count, 0);
	return (calc < max_limit) ? calc : max_limit;
}

/**
 * pmu_delay_step_calc() - Calculate calibration delay tap step size from DRAM type
 *
 * Return: Calibration delay tap step size based on DRAM type.
 */
u32 pmu_delay_step_calc(void)
{
	u8 rank = dmem_read8(PMU_DMEM_CAL_RANK);
	u8 byte_idx = dmem_read8(PMU_DMEM_CAL_BYTE);
	u32 base_ptr = dmem_read32(PMU_DMEM_CAL_STRUCT_PTR);
	u32 offset = (rank * 108) + (byte_idx * 54) + 3;
	u8 flags = *(volatile u8 *)(uintptr_t)(base_ptr + offset);
	u8 dram_type = dmem_read8(PMU_DMEM_DRAM_TYPE);

	if (dram_type == 2)
		return (flags & 0x10) ? 4 : 8;
	return (flags & 0x10) ? 2 : 4;
}

/**
 * pmu_cal_struct_mask_and() - Apply bitwise AND mask across all ranks and slices in struct
 * @base: Base DMEM offset of the calibration structure array
 * @byte_offset: Offset of the target byte within each rank/slice element
 * @mask: Bitwise AND mask applied to the field
 */
void pmu_cal_struct_mask_and(u32 base, u32 byte_offset, u8 mask)
{
	volatile u8 *p = (volatile u8 *)(uintptr_t)(base + byte_offset);
	for (u32 i = 0; i < 4; i++, p += 54)
		*p &= mask;
}

/**
 * pmu_cal_struct_mask_or() - Apply bitwise OR mask across all ranks and slices in struct
 * @base: Base DMEM offset of the calibration structure array
 * @byte_offset: Offset of the target byte within each rank/slice element
 * @mask: Bitwise OR mask applied to the field
 */
void pmu_cal_struct_mask_or(u32 base, u32 byte_offset, u8 mask)
{
	volatile u8 *p = (volatile u8 *)(uintptr_t)(base + byte_offset);
	for (u32 i = 0; i < 4; i++, p += 54)
		*p |= mask;
}

/**
 * pmu_cal_struct_to_shadow16() - Copy calibration byte field into 16-bit shadow table
 * @base: Base DMEM offset of calibration structure
 * @byte_offset: Byte offset within calibration structure element
 */
void pmu_cal_struct_to_shadow16(u32 base, u32 byte_offset)
{
	const volatile u8 *src = (const volatile u8 *)(uintptr_t)(base + byte_offset);
	u32 dst = PMU_DMEM_CAL_SHADOW_TABLE + (byte_offset * 2);
	for (u32 i = 0; i < 4; i++, src += 54, dst += 108)
		dmem_write16(dst, *src);
}

/**
 * pmu_cal_stride_div_update() - Update calibration stride divisor at marker A
 * @stride: Stride divisor step increment
 * @flags: Calibration control flags
 *
 * Return: 0 on success, negative error code on failure.
 */
int pmu_cal_stride_div_update(int stride, u32 flags)
{
	if (stride < 1)
		return stride;

	u8 div_val = dmem_read8(PMU_DMEM_CAL_MARKER_A + 2);
	u16 marker_a = dmem_read16(PMU_DMEM_CAL_MARKER_A);
	if (div_val != 0)
		stride = stride / (int)div_val;

	u32 force_flag = 0;
	if (!(flags & (1 << 9))) {
		if (dmem_read8(PMU_DMEM_CAL_MARKER_A + 0x11) == 0)
			force_flag = 1;
	}

	u32 res = pmu_cal_tracker_stride_advance(marker_a, (u16)stride, force_flag);
	dmem_write16(PMU_DMEM_CAL_MARKER_A, (u16)res);
	return res;
}

/**
 * pmu_dmem_stride_descriptor_read() - Read calibration stride descriptor and update tracker state
 */
void pmu_dmem_stride_descriptor_read(void)
{
	u32 buf[2] = {0, 0};
	u32 csr_offset = *(u32 *)(uintptr_t)(PMU_DMEM_BASE | PMU_DMEM_ACTIVE_SLICE_IDX);
	pmu_slice_lcdl_delay_collect(csr_offset, buf, 1);
	pmu_cal_metric_log(5, 0x2550000);
	pmu_cal_matrix_trace_dump(0, 1, (const u16 *)buf);
}

/**
 * pmu_cal_tracker_stride_advance() - Advance calibration tracker position by one stride
 * @marker: Target calibration marker value
 * @div: Divisor factor for stride calculation
 * @flag: Mode flag controlling stride direction
 *
 * Return: Advanced marker position in delay steps.
 */
u16 pmu_cal_tracker_stride_advance(u32 marker, u32 div, u32 flag)
{
	volatile u16 *tracker16 = (volatile u16 *)(uintptr_t)(PMU_DMEM_BASE | PMU_DMEM_CAL_TRACKER);
	__asm__("" : "+r"(tracker16));

	while ((u16)div != 0) {
		if (div >= 4 && flag == 1) {
			if ((marker & 7) != 0) {
				div--;
				pmu_clear_tracker_words((void *)tracker16 + 16);
				marker = pmu_cal_tracker_step_update();
				flag = 1;
				continue;
			} else {
				memset((void *)tracker16, 0, 0x20);
				if (div >= 513) {
					div -= 512;
					tracker16[0x22 / 2] = 0xff;
					tracker16[0x24 / 2] = 3;
					marker = pmu_cal_tracker_step_update();
					flag = 1;
					continue;
				} else {
					u32 step_cnt = (div >> 1) - 1;
					if ((div >> 1) < 1) {
						tracker16[0x22 / 2] = (u16)step_cnt;
						tracker16[0x24 / 2] = ((u16)step_cnt == 0) ? 0 : 3;
						marker = pmu_cal_tracker_step_update() + 4;
						pmu_clear_tracker_words((void *)tracker16);
						return (u16)marker;
					} else {
						tracker16[0x24 / 2] = 3;
						tracker16[0x22 / 2] = (u16)step_cnt;
						return (u16)(marker + 8);
					}
				}
			}
		} else {
			flag = 0;
			if ((marker & 7) == 0) {
				if (div == 1) {
					pmu_clear_tracker_words((void *)tracker16);
					return (u16)(marker + 4);
				}
				memset((void *)tracker16, 0, 0x20);
				if (div == 2) {
					return (u16)(marker + 8);
				}
				marker = pmu_cal_tracker_step_update();
				div -= 2;
				continue;
			} else {
				pmu_clear_tracker_words((void *)tracker16 + 16);
				if (div == 1) {
					return (u16)(marker + 4);
				}
				marker = pmu_cal_tracker_step_update();
				if (div == 2) {
					marker += 4;
					pmu_clear_tracker_words((void *)tracker16);
					return (u16)marker;
				}
				div--;
				continue;
			}
		}
	}

	return (u16)*(volatile u16 *)(uintptr_t)(PMU_DMEM_BASE | PMU_DMEM_CAL_MARKER_A);
}

/**
 * pmu_cal_dmem_stride_bytes_fill() - Populate calibration tracking bytes across stride offsets
 * @base: Base pointer to destination buffer in DMEM
 * @curr: Current pointer or limit within destination buffer
 * @val: Byte value to fill
 * @stride: Stride interval in bytes between successive fills
 */
void pmu_cal_dmem_stride_bytes_fill(u8 *base, u8 *curr, u8 val, u32 stride)
{
	do {
		curr[0x1f] = val;
		curr[0x1b] = val;
		curr[0x04] = val;
		curr[0x00] = val;
		curr = base + stride;
	} while (stride++ < 2);
}

/**
 * pmu_cal_struct_mask_remap() - Update calibration structure masks and remap active slices
 */
void pmu_cal_struct_mask_remap(void)
{
	u32 base = (PMU_DMEM_BASE | PMU_DMEM_CAL_STRUCT_BASE);
	pmu_cal_struct_to_shadow16(base, 0x10);
	pmu_cal_struct_mask_or(base, 0x10, 0x41);
	pmu_cal_struct_mask_and(base, 0x10, 0xf3);
	pmu_cal_struct_to_shadow16(base, 0x29);
	pmu_cal_struct_mask_and(base, 0x29, 0x1f);
	pmu_multirank_slice_cal_dispatch((void *)(uintptr_t)base, 0, 0xfffeffff, -513, 0xff, 0);
	pmu_delay_us(20000, 0);
	pmu_tracker_field_extract((u8 *)(uintptr_t)base, 0x29);
}

/**
 * pmu_hw_timer_delay() - Busy-wait delay using ARC EM Real-Time Counter (AUX_RTC)
 * @count: Delay duration in hardware timer clock cycles
 */
void pmu_hw_timer_delay(u32 count)
{
	u32 ticks = 0;
	if (count != 0) {
		u8 shift = dmem_read8(PMU_DMEM_STAGE_STATUS);
		ticks = (count + 1) >> shift;
	}

	u32 ctrl = __builtin_arc_lr(0x103);
	if (!(ctrl & 1)) {
		__builtin_arc_sr(2, 0x103);
		__builtin_arc_sr(1, 0x103);
	}

	if (ticks < 16) {
		/* Dummy read for APB bus delay */
		(void)phy_read16(PHY_REG_MASTER_BASE | 0x0048);
		return;
	}

	u32 start_low = __builtin_arc_lr(0x104);
	u32 rem_ticks = ~start_low;
	u32 start_high;
	u32 target_low;

	if (rem_ticks < ticks) {
		start_high = __builtin_arc_lr(0x105);
		while (__builtin_arc_lr(0x104) >= start_low)
			;
		start_high = __builtin_arc_lr(0x105);
		target_low = ticks - rem_ticks;
	} else {
		start_high = __builtin_arc_lr(0x105);
		target_low = start_low + ticks;
	}

	while (__builtin_arc_lr(0x104) < target_low) {
		if (__builtin_arc_lr(0x105) != start_high)
			break;
	}
}
