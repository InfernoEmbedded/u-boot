// SPDX-License-Identifier: GPL-2.0+
/*
 * Allwinner A733 Cadence DDR PHY PMU Foundation Memory Primitives
 */

#include "pmu_fw.h"

/**
 * memset() - Fill memory region with a constant byte value
 * @s: Pointer to destination memory block
 * @c: Byte value to replicate across @n bytes
 * @n: Number of bytes to set
 *
 * Aligns the destination pointer to a 4-byte boundary using byte stores, fills
 * 32-bit words using the replicated pattern (u8)@c * 0x01010101UL, and writes
 * any trailing bytes.
 *
 * Return: Original destination pointer @s.
 */
void *memset(void *s, int c, size_t n)
{
	u8 *p = (u8 *)s;
	u8 val = (u8)c;
	u32 pat = (u32)val * 0x01010101UL;

	while (n > 0 && ((uintptr_t)p & 3U) != 0U) {
		*p++ = val;
		n--;
	}

	while (n >= 4U) {
		*(u32 *)p = pat;
		p += 4;
		n -= 4U;
	}

	while (n > 0) {
		*p++ = val;
		n--;
	}

	return s;
}

/**
 * memcpy() - Copy memory region from source to destination
 * @dest: Destination buffer pointer
 * @src: Source buffer pointer
 * @n: Number of bytes to copy
 *
 * Copies 32-bit words when both @dest and @src are 4-byte aligned, followed by
 * any trailing bytes; otherwise copies byte-by-byte.
 *
 * Return: Original destination pointer @dest.
 */
void *memcpy(void *dest, const void *src, size_t n)
{
	u8 *d = (u8 *)dest;
	const u8 *s = (const u8 *)src;

	if ((((uintptr_t)d | (uintptr_t)s) & 3U) == 0U) {
		while (n >= 4U) {
			*(u32 *)d = *(const u32 *)s;
			d += 4;
			s += 4;
			n -= 4U;
		}
	}

	while (n > 0) {
		*d++ = *s++;
		n--;
	}

	return dest;
}

union pmu_u64_words {
	u64 full;
	struct {
		u32 lo;
		u32 hi;
	} w;
};

/**
 * pmu_cal_tracker_step_advance() - Advance calibration step tracker across step count
 * @marker: Initial 16-bit calibration marker
 * @count: Number of step iterations to advance
 * @mode: Step mode selector (1 for fast multi-step mode, 0 for single-step)
 *
 * Return: Updated 16-bit calibration marker.
 */
u32 pmu_cal_tracker_step_advance(u32 marker, u32 count, u32 mode)
{
	__volatile__ struct pmu_cal_tracker *trk = pmu_tracker();
	__volatile__ u32 *slot0 = (__volatile__ u32 *)(uintptr_t)trk->slot0;
	__volatile__ u32 *slot1 = (__volatile__ u32 *)(uintptr_t)trk->slot1;

	while ((u16)count != 0U) {
		if (count >= 4U && mode == 1U) {
			if ((marker & 7U) != 0U) {
				count--;
				slot1[0] = 0U;
				slot1[1] = 0U;
				slot1[2] = 0U;
				slot1[3] = 0U;
				marker = pmu_cal_marker_status_update();
				mode = 1U;
			} else {
				memset((void *)trk->slot0, 0,
				       PMU_TRACKER_SLOTS_BYTES);
				if (count >= 513U) {
					count -= 512U;
					trk->repeat_cnt = 0x00ffU;
					trk->slot0_mode = 0x0003U;
					marker = pmu_cal_marker_status_update();
					mode = 1U;
				} else {
					u32 r0 = (count >> 1) - 1U;

					trk->repeat_cnt = (u16)r0;
					if ((count >> 1) == 0U) {
						trk->slot0_mode = 0U;
						marker = pmu_cal_marker_status_update();
						goto clear0_add4;
					}
					trk->slot0_mode = 0x0003U;
					return (u16)(marker + 8U);
				}
			}
		} else {
			mode = 0U;
			if ((marker & 7U) != 0U) {
				slot1[0] = 0U;
				slot1[1] = 0U;
				slot1[2] = 0U;
				slot1[3] = 0U;
				if (count == 1U)
					return (u16)(marker + 4U);
				marker = pmu_cal_marker_status_update();
				if (count == 2U)
					goto clear0_add4;
				count--;
			} else {
				memset((void *)trk->slot0, 0,
				       PMU_TRACKER_SLOTS_BYTES);
				if (count == 1U)
					goto clear0_add4;
				if (count == 2U)
					return (u16)(marker + 8U);
				marker = pmu_cal_marker_status_update();
				count -= 2U;
			}
		}
	}

	return pmu_sdata()->cal_marker_a;

clear0_add4:
	slot0[0] = 0U;
	slot0[1] = 0U;
	slot0[2] = 0U;
	slot0[3] = 0U;
	return (u16)(marker + 4U);
}

/**
 * pmu_default_exception_handler() - Dump ARC exception state to DCCM and halt
 */
__attribute__((naked))
void pmu_default_exception_handler(void)
{
	__asm__ __volatile__("mov     r2, 0x80000000\n\t"
			     "mov     r3, 0x1234dead\n\t"
			     "st      r3, [r2]\n\t"
			     "add_s   r2, r2, 4\n\t"
			     "lr      r3, [0x400]\n\t"
			     "st      r3, [r2]\n\t"
			     "add_s   r2, r2, 4\n\t"
			     "lr      r3, [0x403]\n\t"
			     "st      r3, [r2]\n\t"
			     "add_s   r2, r2, 4\n\t"
			     "st      r0, [r2]\n\t"
			     "add_s   r2, r2, 4\n\t"
			     "st      r1, [r2]\n\t"
			     "add_s   r2, r2, 4\n\t"
			     "st      sp, [r2]\n\t"
			     "add_s   r2, r2, 4\n\t"
			     "st      fp, [r2]\n\t"
			     "add_s   r2, r2, 4\n\t"
			     "mov     r3, 0xabcddead\n\t"
			     "st      r3, [r2]\n\t"
			     "flag    1\n\t"
			     "nop_s\n\t");
}

/**
 * pmu_start() - ARC EM4 CRT0 reset entry point
 *
 * Clears core registers, initializes the stack pointer (0x80010000) and
 * small-data global pointer (0x80000400), invokes pmu_main_wrapper(),
 * and halts the processor upon return.
 */
__attribute__((naked))
void pmu_start(void)
{
	__asm__ __volatile__("mov_s   r0, 0\n\t"
			     "mov_s   r1, 0\n\t"
			     "mov_s   r2, 0\n\t"
			     "mov_s   r3, 0\n\t"
			     "mov_s   r10, 0\n\t"
			     "mov_s   r11, 0\n\t"
			     "mov_s   r12, 0\n\t"
			     "mov_s   r13, 0\n\t"
			     "mov_s   r14, 0\n\t"
			     "mov_s   r15, 0\n\t"
			     "mov_s   gp, 0\n\t"
			     "mov_s   fp, 0\n\t"
			     "mov_s   sp, 0\n\t"
			     "mov     ilink, 0\n\t"
			     "mov     r30, 0\n\t"
			     "mov_s   blink, 0\n\t"
			     "mov_s   sp, 0x80010000\n\t"
			     "mov_s   gp, 0x80000400\n\t"
			     "mov_s   fp, 0\n\t"
			     "jl      pmu_main_wrapper\n\t"
			     "1:\n\t"
			     "flag    1\n\t"
			     "nop_s\n\t"
			     "b_s     1b\n\t");
}

/**
 * pmu_vector_table() - ARC EM4 interrupt and exception vector table (0x0000)
 */
__attribute__((naked))
void pmu_vector_table(void)
{
	__asm__ __volatile__(".word pmu_start\n\t"
			     ".rept 15\n\t"
			     ".word pmu_default_exception_handler\n\t"
			     ".endr\n\t"
			     ".word 0x00000050, 0x00000050, 0x00000050, 0x00000050\n\t"
			     ".short 0x264a, 0x7000, 0x264a, 0x7000\n\t"
			     ".short 0x264a, 0x7000, 0x264a, 0x7000\n\t"
			     ".short 0x2069, 0x0040, 0x0000, 0x0000\n\t"
			     ".short 0x0000, 0x0000, 0x0000, 0x0000\n\t"
			     ".short 0x0000, 0x0000, 0x0000, 0x0000\n\t"
			     ".short 0x0000, 0x0000, 0x0000, 0x0000\n\t");
}

/**
 * pmu_cal_struct_mask_remap() - Remap calibration struct shadow masks (0x7f64)
 */
void pmu_cal_struct_mask_remap(void)
{
	u32 base =
		(PMU_DMEM_BASE | PMU_DMEM_CAL_STRUCT_BASE);

	pmu_cal_eye_table_save((void *)(uintptr_t)(base), PMU_SLICE_DESC_MR16_OFF);
	pmu_cal_lane_mask_or_all(base, PMU_SLICE_DESC_MR16_OFF, 0x41U);
	pmu_cal_lane_mask_and_all(base, PMU_SLICE_DESC_MR16_OFF, 0xf3U);
	pmu_cal_eye_table_save((void *)(uintptr_t)(base), PMU_SLICE_DESC_MR41_OFF);
	pmu_cal_lane_mask_and_all(base, PMU_SLICE_DESC_MR41_OFF, 0x1fU);
	pmu_multirank_slice_cal_dispatch((void *)(uintptr_t)base, 0U,
					 0xfffeffffUL, (u32)-513, 0xffU, 0U);
	pmu_delay_us(20000U, 0U);
	pmu_cal_eye_table_restore((u8 *)(uintptr_t)base, PMU_SLICE_DESC_MR41_OFF);
}

/**
 * pmu_adjust_step_limit() - Adjust coarse/fine step limit byte pair
 * @pair: Two-byte array holding [0] coarse count and [1] fine tap offset
 * @flag: Parity selector (non-zero adjusts odd counts; zero adjusts non-zero even counts)
 *
 * Decrements @pair[0] by 1 and adds 64 fine taps to @pair[1] when @pair[0]
 * matches the parity condition selected by @flag.
 */
void pmu_adjust_step_limit(u8 *pair, u32 flag)
{
	u32 count = pair[0];

	if (flag ? (count & 1U) : (count != 0U && !(count & 1U))) {
		pair[0] = (u8)(count - 1U);
		pair[1] = (u8)(pair[1] + 64U);
	}
}

/**
 * pmu_bit_lookup_e4() - Extract 1-bit truth value from 0xe4 lookup constant
 * @shift: Left shift applied to @mask before indexing
 * @mask: Input mask value
 *
 * Return: 1-bit lookup result ((0xe4 >> ((mask << shift) & 6)) & 1).
 */
u32 pmu_bit_lookup_e4(u32 shift, u32 mask)
{
	u32 lut;

	mask = (mask << shift) & 6U;
	lut = 0xe4U;
	return (lut >> mask) & 1U;
}

struct map_pair {
	u8 match;
	u8 bit;
};

struct map_entry {
	u8 base_mask;
	u8 count;
	struct map_pair pairs[8];
};

/**
 * pmu_map_mask() - Remap bitmask through DCCM lookup table at 0x80000a7e
 * @val: Input bitmask byte
 * @index: Table entry index (0..N)
 *
 * Return: Remapped 8-bit mask combining base_mask and matched bit pairs.
 */
u8 pmu_map_mask(u8 val, u32 index)
{
	const struct map_entry *table =
		(const struct map_entry *)PMU_DMEM_DQ_PIN_MAP_BASE;
	const struct map_entry *entry = &table[index];
	u8 result = entry->base_mask & val;
	u32 i;

	for (i = 0; i < entry->count; i++) {
		if (val & entry->pairs[i].match)
			result |= (1U << entry->pairs[i].bit);
	}
	return result;
}

/**
 * pmu_math_freq_scale_div() - Divide signed value by scaled frequency factor
 * @val: Signed dividend
 *
 * Divides @val by (dmem[0x80000008] << 7), rounding positive values toward
 * +infinity and negative values toward 0.
 *
 * Return: Scaled signed quotient.
 */
s32 pmu_math_freq_scale_div(s32 val)
{
	u32 divisor = (u32)dmem_read8(PMU_DMEM_FREQ_ROUND_BIAS) << 7;
	u32 pos;

	if (val >= 1) {
		u32 num = (u32)(val - 1);

		return (s32)((num + divisor) / divisor);
	}

	pos = (u32)(-val);
	return -(s32)(pos / divisor);
}

/**
 * pmu_unpack_6() - Combine coarse and 6-bit fine delay pair into linear tap value
 * @p: Two-byte array holding [0] coarse step and [1] 6-bit fine offset
 *
 * Return: Combined linear delay value ((p[0] << 6) + p[1]).
 */
u32 pmu_unpack_6(const u8 *p)
{
	return ((u32)p[0] << 6) + p[1];
}

/**
 * pmu_unpack_7() - Combine coarse and 7-bit fine delay pair into linear tap value
 * @p: Two-byte array holding [0] coarse step and [1] 7-bit fine offset
 *
 * Return: Combined linear delay value ((p[0] << 7) + p[1]).
 */
u32 pmu_unpack_7(const u8 *p)
{
	return ((u32)p[0] << 7) + p[1];
}

/**
 * pmu_unpack_byte_pair() - Split packed delay value into coarse/fine byte pair
 * @val: Packed delay value
 * @out: Two-byte output buffer receiving [0] (val >> 6) and [1] (val & 0x3f)
 */
void pmu_unpack_byte_pair(u32 val, u8 *out)
{
	out[1] = (u8)(val & 0x3fU);
	out[0] = (u8)(val >> 6);
}

/**
 * pmu_cal_param_unpack() - Unpack fine and rounded coarse delay fields into byte pair
 * @val: Packed 10-bit delay line parameter
 * @out: Two-byte output array receiving [0] rounded coarse delay and [1] 6-bit fine delay
 */
void pmu_cal_param_unpack(u32 val, u8 *out)
{
	u32 c = val >> 6;
	u32 f = val & 0x3fU;

	out[1] = (u8)f;
	out[0] = (u8)(((c >> 1) & 0x07U) + (c & 0x01U));
}

/**
 * pmu_dly_line_repack() - Repack fine and shifted coarse delay fields
 * @val: Input delay value
 *
 * Return: Recombined delay tap ((val & 0x3f) + (val & 0x40) + ((val >> 1) & 0x1c0)).
 */
u32 pmu_dly_line_repack(u32 val)
{
	return (val & 0x3fU) + (val & 0x40U) + ((val >> 1) & 0x1c0U);
}

/**
 * pmu_midpoint_deviation_calc() - Compute signed LCDL tap deviation from nominal midpoint
 * @val: Input LCDL tap value
 *
 * Return: (s8)(64 - val) when @val < 65, or (s8)((val << 2) + 48) when @val >= 65.
 */
s8 pmu_midpoint_deviation_calc(u32 val)
{
	u32 c = 64U;

	if (val <= c)
		return (s8)(c - val);

	return (s8)((val << 2) + 48U);
}

/**
 * pmu_cal_lane_mask_and_all() - Bitwise-AND byte across all 4 slice descriptors
 * @base: Base DCCM address of the 4-slice (4 x 54-byte) descriptor table
 * @offset: Byte offset within each 54-byte slice descriptor
 * @mask: 8-bit mask applied via bitwise AND to each slice byte
 */
void pmu_cal_lane_mask_and_all(u32 base, u32 offset, u32 mask)
{
	__volatile__ u8 *p = (__volatile__ u8 *)(base + offset);
	u32 i;

	for (i = 0U; i < PMU_SLICE_DESC_TOTAL_BYTES; i += PMU_SLICE_DESC_SIZE)
		p[i] &= (u8)mask;
}

/**
 * pmu_cal_lane_mask_or_all() - Bitwise-OR byte across all 4 slice descriptors
 * @base: Base DCCM address of the 4-slice (4 x 54-byte) descriptor table
 * @offset: Byte offset within each 54-byte slice descriptor
 * @mask: 8-bit mask applied via bitwise OR to each slice byte
 */
void pmu_cal_lane_mask_or_all(u32 base, u32 offset, u32 mask)
{
	__volatile__ u8 *p = (__volatile__ u8 *)(base + offset);
	u32 i;

	for (i = 0U; i < PMU_SLICE_DESC_TOTAL_BYTES; i += PMU_SLICE_DESC_SIZE)
		p[i] |= (u8)mask;
}

/**
 * pmu_cal_lcdl_coarse_fine_pair_pack() - Pack coarse/fine LCDL delay pairs into table
 * @ch_idx: Channel index
 * @threshold: Channel threshold value
 * @limit: Channel limit value
 * @pair_a: Primary coarse/fine pair A
 * @pair_b: Primary coarse/fine pair B
 * @pair_c: Secondary coarse/fine pair C
 * @pair_d: Secondary coarse/fine pair D
 * @table_base: Base pointer of destination LCDL delay table
 */
void pmu_cal_lcdl_coarse_fine_pair_pack(u32 ch_idx, u32 threshold, u32 limit,
					const u8 *pair_a, const u8 *pair_b,
					const u8 *pair_c, const u8 *pair_d,
					void *table_base)
{
	u8 *entry = (u8 *)table_base + (ch_idx << 2);
	bool use_sec = (u8)limit > (u8)threshold;
	const u8 *pa = use_sec ? pair_c : pair_a;
	const u8 *pb = use_sec ? pair_d : pair_b;
	const u8 *src = entry + (use_sec ? 0x02U : 0x12U);
	u16 h_a = *(const u16 *)src;
	u16 h_b = *(const u16 *)(src + 8U);

	*(u16 *)(entry + 0x22U) = h_a;
	*(u16 *)(entry + 0x2aU) = h_b;
	*(u16 *)(entry + 0x20U) = (u16)(((u16)pa[0] << 7) + pa[1]);
	*(u16 *)(entry + 0x28U) = (u16)(((u16)pb[0] << 7) + pb[1]);
}

/**
 * pmu_cal_eye_margin_quadratic_cost_calc() - Compute weighted quadratic eye-centering cost
 * @val: Packed eye sample word (low byte: center offset, high byte: magnitude)
 * @target: Signed 8-bit target center offset
 * @min_bound: Minimum valid magnitude bound
 * @max_bound: Maximum valid magnitude bound
 * @out_overflow: Output pointer receiving 0x3f on violation or abs(delta) in-bounds
 *
 * Return: Weighted quadratic cost ((delta^2 * w1) + (overflow^2 * w2)).
 */
u32 pmu_cal_eye_margin_quadratic_cost_calc(u32 val, u32 target, u32 min_bound,
					   u32 max_bound, u8 *out_overflow)
{
	s32 delta = (s32)((s8)val) - (s32)((s8)target);
	u32 mag = (u16)val >> 8;
	u32 min_b = (u8)min_bound;
	u32 max_b = (u8)max_bound;
	s32 over;
	u8 out = 0x3fU;

	if (delta < 0)
		delta = -delta;
	over = (s32)(min_b - mag);
	if (over <= 0) {
		over = (s32)(mag - max_b);
		if (over <= 0) {
			over = 0;
			out = (u8)delta;
		}
	}
	*out_overflow = out;
	return (u32)(u16)(delta * delta) * dmem_read16(PMU_DMEM_COST_WEIGHT_1_422) +
	       (u32)(u16)(over * over) * dmem_read16(PMU_DMEM_COST_WEIGHT_2_410);
}
