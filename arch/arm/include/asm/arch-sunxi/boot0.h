/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Configuration settings for the Allwinner A64 (sun50i) CPU
 */

#include <asm/arch/cpu.h>

#if defined(CONFIG_RESERVE_ALLWINNER_BOOT0_HEADER) && !defined(CONFIG_XPL_BUILD)
/* reserve space for BOOT0 header information */
	b	reset
	.space	1532
#elif defined(CONFIG_ARM_BOOT_HOOK_RMR)
/*
 * Switch into AArch64 if needed.
 * Refer to arch/arm/mach-sunxi/rmr_switch.S for the original source.
 */
	tst     x0, x0                  // this is "b #0x84" in ARM
	b       reset
#if defined(CONFIG_MACH_SUN60I_A733)
	.space  0x7c

	.word	0xe10f0000	// mrs     r0, CPSR
	.word	0xe3c0001f	// bic     r0, r0, #31
	.word	0xe3800013	// orr     r0, r0, #19
	.word	0xe38000c0	// orr     r0, r0, #192 ; 0xc0
	.word	0xe121f000	// msr     CPSR_c, r0
	.word	0xe3a0da8d	// mov     sp, #0x8d000
	.word	0xe59f10cc	// ldr     r1, [pc, #204] ; 0x02002000 (CCU_BASE)
	.word	0xe59f20cc	// ldr     r2, [pc, #204] ; 0x00010001
	.word	0xe5812e00	// str     r2, [r1, #0xe00] (UART0_BGR_REG)
	.word	0xe59f10c8	// ldr     r1, [pc, #200] ; 0x02000100 (PIO Bank B)
	.word	0xe5912004	// ldr     r2, [r1, #4]
	.word	0xe3a03eff	// mov     r3, #0xff0
	.word	0xe1c22003	// bic     r2, r2, r3
	.word	0xe3a03e22	// mov     r3, #0x220
	.word	0xe1822003	// orr     r2, r2, r3
	.word	0xe5812004	// str     r2, [r1, #4] (PB9/PB10 = func 2 UART0)
	.word	0xe5912024	// ldr     r2, [r1, #36]
	.word	0xe3a0380f	// mov     r3, #0xf0000
	.word	0xe1c22003	// bic     r2, r2, r3
	.word	0xe3a03805	// mov     r3, #0x50000
	.word	0xe1822003	// orr     r2, r2, r3
	.word	0xe5812024	// str     r2, [r1, #36] (PB9/PB10 pull-up)
	.word	0xe3a04625	// mov     r4, #0x2500000 (UART0 base)
	.word	0xe3a02083	// mov     r2, #0x83
	.word	0xe584200c	// str     r2, [r4, #12] (LCR: enable DLAB)
	.word	0xe3a0200d	// mov     r2, #13
	.word	0xe5842000	// str     r2, [r4] (DLL = 13 for 115200)
	.word	0xe3a02000	// mov     r2, #0
	.word	0xe5842004	// str     r2, [r4, #4] (DLH = 0)
	.word	0xe3a02003	// mov     r2, #3
	.word	0xe584200c	// str     r2, [r4, #12] (LCR: 8N1, disable DLAB)
	.word	0xe3a02007	// mov     r2, #7
	.word	0xe5842008	// str     r2, [r4, #8] (FCR: enable FIFOs)
	.word	0xe3a02021	// mov     r2, #0x21 ('!')
	.word	0xe5842000	// str     r2, [r4] (send '!')
	.word	0xe5942014	// ldr     r2, [r4, #20] (LSR)
	.word	0xe3120040	// tst     r2, #0x40 (TEMT)
	.word	0x0afffffc	// beq     -4 (wait for TEMT)
	.word	0xe59f2058	// ldr     r2, [pc, #88] ; 200000 (delay)
	.word	0xe2522001	// subs    r2, r2, #1
	.word	0x1afffffd	// bne     -1 (delay loop)
	.word	0xe59f1050	// ldr     r1, [pc, #80] ; 0x08000200
	.word	0xe3a02000	// mov     r2, #0
	.word	0xe5812000	// str     r2, [r1] (CPU_DA_DDR_CTRL_REG = 0)
	.word	0xe59f1048	// ldr     r1, [pc, #72] ; 0x08001000
	.word	0xe3a02001	// mov     r2, #1
	.word	0xe5812000	// str     r2, [r1] (CLU0_CPU0_CTRL_REG = 1)
	.word	0xe59f0040	// ldr     r0, [pc, #64] ; CONFIG_*TEXT_BASE
	.word	0xe5810004	// str     r0, [r1, #4] (CLU0_RVBARADDR0_L = TEXT_BASE)
	.word	0xe3a02000	// mov     r2, #0
	.word	0xe5812008	// str     r2, [r1, #8] (CLU0_RVBARADDR0_H = 0)
	.word	0xf57ff04f	// dsb     sy
	.word	0xf57ff06f	// isb     sy
	.word	0xee1c0f50	// mrc     15, 0, r0, cr12, cr0, {2} ; RMR
	.word	0xe3800003	// orr     r0, r0, #3
	.word	0xee0c0f50	// mcr     15, 0, r0, cr12, cr0, {2} ; RMR
	.word	0xf57ff06f	// isb     sy
	.word	0xe320f003	// wfi
	.word	0xeafffffd	// b       @wfi

	.word	0x02002000	// CCU_BASE
	.word	0x00010001	// UART0_GATE_RESET_VAL
	.word	0x02000100	// PIO_BASE_BANK_B
	.word	0x00030d40	// DELAY_COUNT (200000)
	.word	0x08000200	// CPU_DA_DDR_CTRL_REG
	.word	0x08001000	// CLU0_CPU0_CTRL_REG
#ifdef CONFIG_XPL_BUILD
	.word	CONFIG_SPL_TEXT_BASE
#else
	.word	CONFIG_TEXT_BASE
#endif
#else
	.space  0x78
	.word	fel_stash - .

	.word	0xe24f000c	// sub     r0, pc, #12  // @(fel_stash - .)
	.word	0xe51f1010	// ldr     r1, [pc, #-16] // fel_stash - .
	.word	0xe0800001	// add     r0, r0, r1
	.word	0xe580d000	// str     sp, [r0]
	.word	0xe580e004	// str     lr, [r0, #4]
	.word	0xe10fe000	// mrs     lr, CPSR
	.word	0xe580e008	// str     lr, [r0, #8]
	.word	0xe101e300	// mrs     lr, SP_irq
	.word	0xe580e014	// str     lr, [r0, #20]
	.word	0xee11ef10	// mrc     15, 0, lr, cr1, cr0, {0}
	.word	0xe580e00c	// str     lr, [r0, #12]
	.word	0xee1cef10	// mrc     15, 0, lr, cr12, cr0, {0}
	.word	0xe580e010	// str     lr, [r0, #16]
#if defined(CONFIG_MACH_SUN55I_A523)
	.word	0xee1cefbc	// mrc     15, 0, lr, cr12, cr12, {5}
	.word	0xe31e0001	// tst     lr, #1
	.word	0x0a000003	// beq     cc <start32+0x48>
	.word	0xee14ef16	// mrc     15, 0, lr, cr4, cr6, {0}
	.word	0xe580e018	// str     lr, [r0, #24]
	.word	0xee1ceffc	// mrc     15, 0, lr, cr12, cr12, {7}
	.word	0xe580e01c	// str     lr, [r0, #28]
#endif
	.word	0xe59f1034	// ldr     r1, [pc, #52] ; RVBAR_ADDRESS
	.word	0xe59f0034	// ldr     r0, [pc, #52] ; SUNXI_SRAMC_BASE
	.word	0xe5900024	// ldr     r0, [r0, #36] ; SRAM_VER_REG
	.word	0xe21000ff	// ands    r0, r0, #255    ; 0xff
	.word	0x159f102c	// ldrne   r1, [pc, #44] ; RVBAR_ALTERNATIVE
	.word	0xe59f002c	// ldr     r0, [pc, #44] ; CONFIG_*TEXT_BASE
	.word	0xe5810000	// str     r0, [r1]
	.word	0xf57ff04f	// dsb     sy
	.word	0xf57ff06f	// isb     sy
	.word	0xee1c0f50	// mrc     15, 0, r0, cr12, cr0, {2} ; RMR
	.word	0xe3800003	// orr     r0, r0, #3
	.word	0xee0c0f50	// mcr     15, 0, r0, cr12, cr0, {2} ; RMR
	.word	0xf57ff06f	// isb     sy
	.word	0xe320f003	// wfi
	.word	0xeafffffd	// b       @wfi

	.word	CONFIG_SUNXI_RVBAR_ADDRESS	// writable RVBAR mapping addr
	.word	SUNXI_SRAMC_BASE
	.word	CONFIG_SUNXI_RVBAR_ALTERNATIVE	// address for die variant
#ifdef CONFIG_XPL_BUILD
	.word	CONFIG_SPL_TEXT_BASE
#else
	.word   CONFIG_TEXT_BASE
#endif
#endif
#else
/* normal execution */
	b	reset
#endif
