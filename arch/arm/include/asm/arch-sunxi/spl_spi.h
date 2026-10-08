/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * (C) Copyright 2026 Alastair D'Silva <alastair@d-silva.org>
 */

#ifndef _ASM_ARCH_SUNXI_SPL_SPI_H_
#define _ASM_ARCH_SUNXI_SPL_SPI_H_

#include <linux/types.h>

void spi0_init(void);
void spi0_deinit(void);
void spi0_read_data(void *buf, u32 addr, u32 len);
int spi0_erase_sector(u32 addr);
int spi0_write_data(u32 addr, const void *buf, u32 len);

#endif /* _ASM_ARCH_SUNXI_SPL_SPI_H_ */
