/*
 * Utility functions for FEL mode, when running SPL in AArch64.
 *
 * Copyright (c) 2017 Arm Ltd.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm-offsets.h>
#include <config.h>
#include <asm/system.h>
#include <linux/linkage.h>
#include <asm/arch/cpu.h>

/*
 * We don't overwrite save_boot_params() here, to save the FEL state upon
 * entry, since this would run *after* the RMR reset, which clobbers that
 * state.
 * Instead we store the state _very_ early in the boot0 hook, *before*
 * resetting to AArch64.
 */

/*
 * The FEL routines in BROM run in AArch32.
 * Reset back into 32-bit mode here and restore the saved FEL state
 * afterwards.
 * Resetting back into AArch32/EL3 using the RMR always enters the BROM,
 * but we can use the CPU hotplug mechanism to branch back to our code
 * immediately.
 */
ENTRY(return_to_fel)
	/*
	 * the RMR reset will clear all registers, so save the arguments
	 * (LR and SP) in the fel_stash structure, which we read anyways later
	 */
	adr	x2, fel_stash
	str	w0, [x2]
	str	w1, [x2, #4]

	adr	x1, fel_stash_addr	// to find the fel_stash address in AA32
	str	w2, [x1]

	ldr	w0, =0xfa50392f		// CPU hotplug magic
#if defined(CONFIG_MACH_SUN50I_H616) || defined(CONFIG_MACH_SUN50I_A133) || \
    defined(CONFIG_MACH_SUN55I_A523) || defined(CONFIG_MACH_SUN60I_A733)
	ldr	w2, =(SUNXI_R_CPUCFG_BASE + 0x1c0)
	str	w0, [x2], #0x4
#elif defined(CONFIG_MACH_SUN50I_H6)
	ldr	w2, =(SUNXI_RTC_BASE + 0x1b8)	// BOOT_CPU_HP_FLAG_REG
	str	w0, [x2], #0x4
#else
	ldr	w2, =(SUNXI_CPUCFG_BASE + 0x1a4) // offset for CPU hotplug base
	str	w0, [x2, #0x8]
#endif
#if defined(CONFIG_MACH_SUN60I_A733)
	/*
	 * On A733, BROM cannot directly execute code out of SRAM A1 after
	 * warm reset. Copy the saved FEL state to SRAM A2 (0x00080600) and
	 * 32-bit return trampoline to 0x00080050.
	 */
	ldr	x4, =0x08000200
	mov	w5, #1
	str	w5, [x4]

	adr	x4, fel_stash
	ldr	x5, =0x00080600
	ldp	w6, w7, [x4]
	stp	w6, w7, [x5]
	ldp	w6, w7, [x4, #8]
	stp	w6, w7, [x5, #8]
	ldp	w6, w7, [x4, #16]
	stp	w6, w7, [x5, #16]
	ldp	w6, w7, [x4, #24]
	stp	w6, w7, [x5, #24]

	/* Copy back_in_32 code to 0x00080050 */
	adr	x6, back_in_32
	adr	x7, back_in_32_end
	ldr	x8, =0x00080050
1:	ldr	w9, [x6], #4
	str	w9, [x8], #4
	cmp	x6, x7
	b.lo	1b

	ldr	w0, =0x00080050
	str	w0, [x2]		// 0x070501c4 (CPU 0)
	str	w0, [x2, #4]		// 0x070501c8 (CPU 1)
	str	w0, [x2, #8]		// 0x070501cc (CPU 2)
	str	w0, [x2, #12]		// 0x070501d0 (CPU 3)
	dsb	sy
	isb	sy

	/* Drain UART transmitter */
	ldr	x2, =0x02500000
	ldr	x3, =100000
2:	ldr	w0, [x2, #0x14]
	tst	w0, #0x40
	b.ne	3f
	subs	x3, x3, #1
	b.ne	2b
3:
#else
	adr	x0, back_in_32
	str	w0, [x2]

	dsb	sy
	isb	sy
#endif

	mov	x0, #2			// RMR reset into AArch32
	dsb	sy
	msr	RMR_EL3, x0
	isb	sy
5:	wfi
	b	5b

fel_stash_addr:			// must immediately precede back_in_32:
	.word   0x00000000	// receives fel_stash addr, by AA64 code above

/* AArch32 code to restore the state from fel_stash and return back to FEL. */
.globl back_in_32
.globl back_in_32_end
back_in_32:
#if defined(CONFIG_MACH_SUN60I_A733)
	.word	0xe3a00702	// mov	r0, #0x00080000
	.word	0xe2800c06	// add	r0, r0, #0x00000600
	.word	0xe5901008	// ldr	r1, [r0, #8]
	.word	0xe129f001	// msr	CPSR_fc, r1
	.word	0xf57ff06f	// isb
	.word	0xe590d000	// ldr	sp, [r0]
	.word	0xe590e004	// ldr	lr, [r0, #4]
	.word	0xe5901014	// ldr	r1, [r0, #20]
	.word	0xe121f301	// msr	SP_irq, r1
	.word	0xe5901010	// ldr	r1, [r0, #16]
	.word	0xee0c1f10	// mcr	15, 0, r1, cr12, cr0, {0}
	.word	0xe590100c	// ldr	r1, [r0, #12]
	.word	0xee011f10	// mcr	15, 0, r1, cr1, cr0, {0}
	.word	0xf57ff06f	// isb
	.word	0xe3a01001	// mov	r1, #1
	.word	0xee0c1fbc	// mcr	15, 0, r1, cr12, cr12, {5}
	.word	0xf57ff06f	// isb
	.word	0xee0c1fdc	// mcr	15, 0, r1, cr12, cr12, {6}
	.word	0xe5901018	// ldr	r1, [r0, #24]
	.word	0xee041f16	// mcr	15, 0, r1, cr4, cr6, {0}
	.word	0xe590101c	// ldr	r1, [r0, #28]
	.word	0xee0c1ffc	// mcr	15, 0, r1, cr12, cr12, {7}
	.word	0xf57ff06f	// isb
	.word	0xe12fff1e	// bx	lr
#else
	.word	0xe51f000c	// ldr	r0, [pc, #-12]  ; load fel_stash address
	.word	0xe5901008	// ldr	r1, [r0, #8]
	.word	0xe129f001	// msr	CPSR_fc, r1
	.word	0xf57ff06f	// isb
	.word	0xe590d000	// ldr	sp, [r0]
	.word	0xe590e004	// ldr	lr, [r0, #4]
	.word	0xe5901014	// ldr  r1, [r0, #20]
	.word	0xe121f301	// msr  SP_irq, r1
	.word	0xe5901010	// ldr	r1, [r0, #16]
	.word	0xee0c1f10	// mcr	15, 0, r1, cr12, cr0, {0} ; VBAR
	.word	0xe590100c	// ldr	r1, [r0, #12]
	.word	0xee011f10	// mcr	15, 0, r1, cr1, cr0, {0}  ; SCTLR
	.word	0xf57ff06f	// isb
#ifdef CONFIG_MACH_SUN55I_A523
	.word	0xe5901018	// ldr  r1, [r0, #24]
	.word	0xee041f16	// mcr  15, 0, r1, cr4, cr6, {0}; ICC_PMR
	.word	0xe590101c	// ldr  r1, [r0, #28]
	.word	0xee0c1ffc	// mcr  15, 0, r1, cr12, cr12, {7}; ICC_IGRPEN1
#endif

	.word	0xe12fff1e	// bx	lr		; return to FEL
#endif
back_in_32_end:
ENDPROC(return_to_fel)
