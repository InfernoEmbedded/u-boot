/* SPDX-License-Identifier: GPL-2.0+ */
MEMORY
{
    IMEM (rx) : ORIGIN = 0x00000000, LENGTH = 64K
}

SECTIONS
{
    .text 0x00000000 : {
        *(.vectors)
        *(.text*)
        *(.rodata*)
    } > IMEM

    /DISCARD/ : {
        *(.data*)
        *(.bss*)
        *(.comment)
        *(.note*)
    }
}
