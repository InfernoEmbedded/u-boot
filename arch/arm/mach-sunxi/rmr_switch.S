@
@ ARMv8 RMR reset sequence on Allwinner SoCs.
@
@ All 64-bit capable Allwinner SoCs reset in AArch32 (and continue to
@ exectute the Boot ROM in this state), so we need to switch to AArch64
@ at some point.
@ Section G6.2.133 of the ARMv8 ARM describes the Reset Management Register
@ (RMR), which triggers a warm-reset of a core and can request to switch
@ into a different execution state (AArch32 or AArch64).
@ The address at which execution starts after the reset is held in the
@ RVBAR system register, which is architecturally read-only.
@ Allwinner provides a writable alias of this register in MMIO space, so
@ we can easily set the start address of AArch64 code.
@ This code below switches to AArch64 and starts execution at the specified
@ start address. It needs to be assembled by an ARM(32) assembler and
@ the machine code must be inserted as verbatim .word statements into the
@ beginning of the AArch64 U-Boot code.
@ To get the encoded bytes, use:
@ ${CROSS_COMPILE}gcc -c -Iinclude -Iarch/arm/include	\
@   -D__ASSEMBLY__ -DCONFIG_ARM64			\
@   -o rmr_switch.o arch/arm/mach-sunxi/rmr_switch.S
@ ${CROSS_COMPILE}objdump -d rmr_switch.o
@
@ The resulting words should be inserted into the U-Boot file at
@ arch/arm/include/asm/arch-sunxi/boot0.h.
@
@ This file is not build by the U-Boot build system, but provided only as a
@ reference and to be able to regenerate a (probably fixed) version of this
@ code found in encoded form in boot0.h.

#include <config.h>

.text
	b	start32			// this is "tst x0, x0" in AArch64
	.word	0x14000047		// this is "b   reset"  in AArch64

#if defined(CONFIG_MACH_SUN60I_A733)
	.space  0x7c

start32:
	/* 1. Enter SVC mode, disable IRQ & FIQ */
	mrs	r0, cpsr
	bic	r0, r0, #0x1f
	orr	r0, r0, #0x13
	orr	r0, r0, #0xc0
	msr	cpsr_c, r0

	/* 2. Configure SCTLR: disable MMU and D-cache */
	mrc	p15, 0, r0, c1, c0, 0
	bic	r0, r0, #0x07
	bic	r0, r0, #0x2000
	bic	r0, r0, #0x1000
	mcr	p15, 0, r0, c1, c0, 0

	/* 3. Stack pointer in SRAM A2 */
	ldr	sp, =0x0008d000

	/* 4. Un-gate APB1 clock (CCU + 0x524 = 0) */
	ldr	r1, =0x02002000
	mov	r2, #0
	str	r2, [r1, #0x524]

	/* 5. Un-gate UART0 clock & deassert reset (CCU + 0x0e00 = 0x00010001, CCU + 0x090c = 0x00010001) */
	ldr	r2, =0x00010001
	str	r2, [r1, #0x0e00]
	str	r2, [r1, #0x90c]

	/* 6. Configure Pinmux for UART0 (PB0/PB1 func 3, PB9/PB10 func 2, pull-ups) */
	ldr	r1, =0x02000100
	ldr	r2, [r1, #0x00]
	bic	r2, r2, #0xff
	orr	r2, r2, #0x33
	str	r2, [r1, #0x00]

	ldr	r2, [r1, #0x04]
	ldr	r3, =0x00000ff0
	bic	r2, r2, r3
	ldr	r3, =0x00000220
	orr	r2, r2, r3
	str	r2, [r1, #0x04]

	ldr	r2, [r1, #0x24]
	ldr	r3, =0x000f0000
	bic	r2, r2, r3
	ldr	r3, =0x00050000
	orr	r2, r2, r3
	str	r2, [r1, #0x24]

	/* 7. Initialize UART0 (115200 8N1 @ 24MHz) */
	ldr	r4, =0x02500000
	mov	r2, #0x83
	str	r2, [r4, #0x0c]
	mov	r2, #13
	str	r2, [r4, #0x00]
	mov	r2, #0
	str	r2, [r4, #0x04]
	mov	r2, #0x03
	str	r2, [r4, #0x0c]
	mov	r2, #0x07
	str	r2, [r4, #0x08]
	mov	r2, #0x03
	str	r2, [r4, #0x10]

	/* 8. Send beacon '!' (0x21) with timeout */
	ldr	r3, =200000
1:	ldr	r2, [r4, #0x14]
	tst	r2, #0x20
	bne	2f
	subs	r3, r3, #1
	bne	1b
2:	mov	r2, #0x21
	str	r2, [r4, #0x00]

	/* 9. Drain transmitter with timeout */
	ldr	r3, =200000
3:	ldr	r2, [r4, #0x14]
	tst	r2, #0x40
	bne	4f
	subs	r3, r3, #1
	bne	3b
4:
	/* ~5ms delay for physical wire serialization */
	ldr	r2, =200000
5:	subs	r2, r2, #1
	bne	5b

	/* 10. Configure CPU cluster control registers */
	ldr	r1, =0x08000200
	mov	r2, #1
	str	r2, [r1]			// CPU_DA_DDR_CTRL_REG = 1 (AXI2HIF)
	ldr	r1, =0x08001000
	mov	r2, #1
	str	r2, [r1]			// CLU0_CPU0_CTRL_REG = 1 (AA64 mode)
#ifdef CONFIG_XPL_BUILD
	ldr	r0, =CONFIG_SPL_TEXT_BASE
#else
	ldr	r0, =CONFIG_TEXT_BASE
#endif
	str	r0, [r1, #4]			// CLU0_RVBARADDR0_L
	mov	r2, #0
	str	r2, [r1, #8]			// CLU0_RVBARADDR0_H = 0
#else
	.space  0x78			// gap distance set by the common
					// encoding of the first instruction
fel_stash_addr:
	.word	fel_stash - .		// distance to fel_stash buffer

start32:
	adr	r0, fel_stash_addr	// absolute location of fel_stash_addr
	ldr	r1, fel_stash_addr	// distance to actual fel_stash
	add	r0, r0, r1		// real address of fel_stash

	/* save the current state as needed by the BROM for a later return */
	str	sp, [r0]
	str	lr, [r0, #4]
	mrs	lr, CPSR
	str	lr, [r0, #8]
	mrs	lr, SP_irq
	str	lr, [r0, #20]
	mrc	p15, 0, lr, cr1, cr0, 0		// SCTLR
	str	lr, [r0, #12]
	mrc	p15, 0, lr, cr12, cr0, 0	// VBAR
	str	lr, [r0, #16]
#ifdef CONFIG_MACH_SUN55I_A523
	mrc	p15, 0, lr, cr12, cr12, 5	// ICC_SRE
	tst	lr, #1
	beq	1f
	mrc	p15, 0, lr, c4, c6, 0		// ICC_PMR
	str	lr, [r0, #24]
	mrc	p15, 0, lr, c12, c12, 7		// ICC_IGRPEN1
	str	lr, [r0, #28]
1:
#endif
#else
	ldr	r1, =CONFIG_SUNXI_RVBAR_ADDRESS
	ldr	r0, =SUNXI_SRAMC_BASE
	ldr	r0, [r0, #36]			// SRAM_VER_REG
	ands	r0, r0, #0xff
	ldrne	r1, =CONFIG_SUNXI_RVBAR_ALTERNATIVE
#ifdef CONFIG_XPL_BUILD
	ldr	r0, =CONFIG_SPL_TEXT_BASE
#else
	ldr	r0, =CONFIG_TEXT_BASE
#endif
	str	r0, [r1]			// store start address in RVBAR
#endif
	dsb	sy
	isb	sy
	mrc	15, 0, r0, cr12, cr0, 2	@ read RMR register
	orr	r0, r0, #3		@ request reset in AArch64
	mcr	15, 0, r0, cr12, cr0, 2 @ write RMR register
	isb	sy
1:	wfi
	b	1b
