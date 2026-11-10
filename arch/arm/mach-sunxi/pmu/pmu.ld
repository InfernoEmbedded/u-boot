/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Linker script for Allwinner A733 LPDDR5 ARC EM4 PMU Firmware
 */


OUTPUT_FORMAT("elf32-littlearc", "elf32-littlearc", "elf32-littlearc")
OUTPUT_ARCH(arc)
ENTRY(pmu_vector_table)

SECTIONS
{
	.vectors 0x00000000 : {
		KEEP(*(.vectors))
	} = 0xe078

	.text 0x00000080 : {
		KEEP(*(.text.pmu_start))
		KEEP(*(.text.pmu_default_exception_handler))
		*(.text .text.*)
	} = 0xe078

	.rodata : { *(.rodata*) }
	.data   : { *(.data*) *(.sdata*) }
	.bss    : { *(.bss*) *(.sbss*) *(COMMON) }

	/DISCARD/ : {
		*(.comment)
		*(.note*)
		*(.eh_frame*)
		*(.ARC.attributes)
	}
}

ASSERT(SIZEOF(.rodata) == 0 && SIZEOF(.data) == 0 && SIZEOF(.bss) == 0,
       "ARC ICCM is execute-only; .rodata/.data/.bss sections are forbidden")
ASSERT(. <= 0x00010000, "PMU firmware exceeds 64 KiB ICCM")
