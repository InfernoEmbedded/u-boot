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
#include <asm/arch/serial.h>

#ifdef CONFIG_MACH_SUN60I_A733
#define A733_FEL_EL3_VBAR		SUNXI_FEL_EL3_VBAR
#define A733_FEL_EL3_A32_SYNC_OFF	0x600
#define UART_LSR_OFFSET			0x14
#define UART_LSR_TEMT_BIT		6
#define SCR_EL3_IRQ_EN			(1 << 1)
#define SCR_EL3_FIQ_EN			(1 << 2)
#define SCR_EL3_FEL_A32_CLEAR		(SCR_EL3_RW_AARCH64 | \
					 SCR_EL3_SMD_DIS | \
					 SCR_EL3_EA_EN | \
					 SCR_EL3_FIQ_EN | \
					 SCR_EL3_IRQ_EN | \
					 SCR_EL3_NS_EN)
#define ICC_SRE_EL3_ENABLE_ALL		0xf
#define ICC_SRE_EL1_ENABLE_ALL		0x7
#define ICC_IGRPEN1_EL3_ENABLE_ALL	0x3
#define SPSR_EL3_A32_SVC_MASKED		(SPSR_EL_ASYN_MASK | \
					 SPSR_EL_IRQ_MASK | \
					 SPSR_EL_FIQ_MASK | \
					 SPSR_EL_M_AARCH32 | \
					 SPSR_EL_M_SVC)
#endif

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

#if defined(CONFIG_MACH_SUN60I_A733)
#if CONFIG_IS_ENABLED(SERIAL)
	/* Drain UART0 TX FIFO before dropping to AArch32 */
	ldr	x2, =SUNXI_UART0_BASE
	mov	w1, #0x10000
2:	ldr	w0, [x2, #UART_LSR_OFFSET]
	tbnz	w0, #UART_LSR_TEMT_BIT, 3f
	subs	w1, w1, #1
	b.ne	2b
3:
#endif

	/*
	 * On A733, boot0.h latches CLU0_CPU0_CTRL_REG to AArch64 and FEL uses
	 * SRAM swap buffers across 0x47000. Rather than warm-resetting through
	 * the BROM hotplug path, drop directly to Secure AArch32 SVC via ERET
	 * and install an EL3 SMC vector in SH_SRAM outside the FEL swap buffer
	 * so sunxi-fel can re-enter AArch64 EL3 for BL31.
	 */
	adr	x4, el3_a32_sync
	adr	x5, el3_a32_sync_end
	ldr	x6, =(A733_FEL_EL3_VBAR + A733_FEL_EL3_A32_SYNC_OFF)
	mov	x8, x6
4:	ldr	w7, [x4], #4
	str	w7, [x6], #4
	cmp	x4, x5
	b.lo	4b

5:	dc	cvau, x8
	add	x8, x8, #64
	cmp	x8, x6
	b.lo	5b
	dsb	ish
	ic	iallu
	dsb	ish
	isb

	ldr	x0, =A733_FEL_EL3_VBAR
	msr	vbar_el3, x0

	mrs	x1, ICC_SRE_EL3
	orr	x1, x1, #ICC_SRE_EL3_ENABLE_ALL
	msr	ICC_SRE_EL3, x1
	isb

	mrs	x1, ICC_SRE_EL1
	orr	x1, x1, #ICC_SRE_EL1_ENABLE_ALL
	msr	ICC_SRE_EL1, x1
	mov	x1, #ICC_IGRPEN1_EL3_ENABLE_ALL
	msr	ICC_IGRPEN1_EL3, x1

	mrs	x0, scr_el3
	mov	x1, #SCR_EL3_FEL_A32_CLEAR
	bic	x0, x0, x1
	msr	scr_el3, x0

	mov	x0, #SPSR_EL3_A32_SVC_MASKED
	msr	spsr_el3, x0

	adr	x0, back_in_32
	msr	elr_el3, x0

	dsb	sy
	isb
	eret

	.align	2
el3_a32_sync:
	mrs	x9, scr_el3
	mov	x10, #SCR_EL3_RW_AARCH64
	orr	x9, x9, x10
	msr	scr_el3, x9
	isb
	cbnz	w0, 6f
	movz	w0, #(CONFIG_SUNXI_BL31_BASE & 0xffff)
	movk	w0, #(CONFIG_SUNXI_BL31_BASE >> 16), lsl #16
6:	uxtw	x0, w0
	ic	iallu
	dsb	sy
	isb
	br	x0
el3_a32_sync_end:
#else
	ldr	w0, =0xfa50392f		// CPU hotplug magic
#if defined(CONFIG_MACH_SUN50I_H616) || defined(CONFIG_MACH_SUN50I_A133) || \
    defined(CONFIG_MACH_SUN55I_A523)
	ldr	w2, =(SUNXI_R_CPUCFG_BASE + 0x1c0)
	str	w0, [x2], #0x4
#elif defined(CONFIG_MACH_SUN50I_H6)
	ldr	w2, =(SUNXI_RTC_BASE + 0x1b8)	// BOOT_CPU_HP_FLAG_REG
	str	w0, [x2], #0x4
#else
	ldr	w2, =(SUNXI_CPUCFG_BASE + 0x1a4) // offset for CPU hotplug base
	str	w0, [x2, #0x8]
#endif
	adr	x0, back_in_32
	str	w0, [x2]

	dsb	sy
	isb	sy
	mov	x0, #2			// RMR reset into AArch32
	dsb	sy
	msr	RMR_EL3, x0
	isb	sy
1:	wfi
	b	1b
#endif

fel_stash_addr:			// must immediately precede back_in_32:
	.word   0x00000000	// receives fel_stash addr, by AA64 code above

/* AArch32 code to restore the state from fel_stash and return back to FEL. */
back_in_32:
	.word	0xe51f000c	// ldr	r0, [pc, #-12]  ; load fel_stash address
	.word	0xe5901008	// ldr	r1, [r0, #8]
#ifdef CONFIG_MACH_SUN60I_A733
	.word	0xe38110c0	// orr	r1, r1, #0xc0   ; mask IRQ/FIQ in thunk
#endif
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
#if defined(CONFIG_MACH_SUN55I_A523) || defined(CONFIG_MACH_SUN60I_A733)
	.word	0xe5901018	// ldr  r1, [r0, #24]
	.word	0xee041f16	// mcr  15, 0, r1, cr4, cr6, {0}; ICC_PMR
	.word	0xe590101c	// ldr  r1, [r0, #28]
	.word	0xee0c1ffc	// mcr  15, 0, r1, cr12, cr12, {7}; ICC_IGRPEN1
#endif

	.word	0xe12fff1e	// bx	lr		; return to FEL
ENDPROC(return_to_fel)
