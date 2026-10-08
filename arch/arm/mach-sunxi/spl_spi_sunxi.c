// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2016 Siarhei Siamashka <siarhei.siamashka@gmail.com>
 */

#include <image.h>
#include <log.h>
#include <spl.h>
#include <asm/arch/spl.h>
#include <asm/arch/spl_spi.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/libfdt.h>
#include <sunxi_gpio.h>

#ifdef CONFIG_SPL_OS_BOOT
#error CONFIG_SPL_OS_BOOT is not supported yet
#endif

/*
 * This is a very simple U-Boot image loading implementation, trying to
 * replicate what the boot ROM is doing when loading the SPL. Because we
 * know the exact pins where the SPI Flash is connected and also know
 * that the Read Data Bytes (03h) command is supported, the hardware
 * configuration is very simple and we don't need the extra flexibility
 * of the SPI framework. Moreover, we rely on the default settings of
 * the SPI controler hardware registers and only adjust what needs to
 * be changed. This is good for the code size and this implementation
 * adds less than 400 bytes to the SPL.
 *
 * There are two variants of the SPI controller in Allwinner SoCs:
 * A10/A13/A20 (sun4i variant) and everything else (sun6i variant).
 * Both of them are supported.
 *
 * The pin mixing part is SoC specific and only A10/A13/A20/H3/A64 are
 * supported at the moment.
 */

/*****************************************************************************/
/* SUN4I variant of the SPI controller                                       */
/*****************************************************************************/

#define SUN4I_SPI0_CCTL             0x1C
#define SUN4I_SPI0_CTL              0x08
#define SUN4I_SPI0_RX               0x00
#define SUN4I_SPI0_TX               0x04
#define SUN4I_SPI0_FIFO_STA         0x28
#define SUN4I_SPI0_BC               0x20
#define SUN4I_SPI0_TC               0x24

#define SUN4I_CTL_ENABLE            BIT(0)
#define SUN4I_CTL_MASTER            BIT(1)
#define SUN4I_CTL_TF_RST            BIT(8)
#define SUN4I_CTL_RF_RST            BIT(9)
#define SUN4I_CTL_XCH               BIT(10)

/*****************************************************************************/
/* SUN6I variant of the SPI controller                                       */
/*****************************************************************************/

#define SUN6I_SPI0_CCTL             0x24
#define SUN6I_SPI0_GCR              0x04
#define SUN6I_SPI0_TCR              0x08
#define SUN6I_SPI0_FIFO_CTL         0x18
#define SUN6I_SPI0_FIFO_STA         0x1C
#define SUN6I_SPI0_MBC              0x30
#define SUN6I_SPI0_MTC              0x34
#define SUN6I_SPI0_BCC              0x38
#define SUN6I_SPI0_TXD              0x200
#define SUN6I_SPI0_RXD              0x300

#define SUN6I_CTL_ENABLE            BIT(0)
#define SUN6I_CTL_MASTER            BIT(1)
#define SUN6I_CTL_SRST              BIT(31)
#define SUN6I_FIFO_CTL_RF_RST       BIT(15)
#define SUN6I_FIFO_CTL_TF_RST       BIT(31)
#define SUN6I_TCR_CS_ACTIVE_LOW     BIT(2)
#define SUN6I_TCR_CS_MANUAL         BIT(6)
#define SUN6I_TCR_CS_LEVEL          BIT(7)
#define SUN6I_TCR_SDM               BIT(13)
#define SUN6I_TCR_XCH               BIT(31)

/*****************************************************************************/

#if IS_ENABLED(CONFIG_MACH_SUN60I_A733)
#define CCM_BASE                    0x02002000
#elif IS_ENABLED(CONFIG_SUN50I_GEN_H6)
#define CCM_BASE                    0x03001000
#elif IS_ENABLED(CONFIG_SUNXI_GEN_NCAT2)
#define CCM_BASE                    0x02001000
#else
#define CCM_BASE                    0x01C20000
#endif

#define CCM_AHB_GATING0             (CCM_BASE + 0x60)
#if IS_ENABLED(CONFIG_MACH_SUN60I_A733)
#define CCM_H6_SPI_BGR_REG          (CCM_BASE + 0x0f04)
#define CCM_SPI0_CLK                (CCM_BASE + 0x0f00)
#elif IS_ENABLED(CONFIG_SUN50I_GEN_H6) || IS_ENABLED(CONFIG_SUNXI_GEN_NCAT2)
#define CCM_H6_SPI_BGR_REG          (CCM_BASE + 0x96c)
#define CCM_SPI0_CLK                (CCM_BASE + 0x940)
#else
#define CCM_H6_SPI_BGR_REG          (CCM_BASE + 0x96c)
#define CCM_SPI0_CLK                (CCM_BASE + 0xA0)
#endif
#define SUN6I_BUS_SOFT_RST_REG0     (CCM_BASE + 0x2C0)

#define AHB_RESET_SPI0_SHIFT        20
#define AHB_GATE_OFFSET_SPI0        20

#define SPI0_CLK_DIV_BY_2           0x1000
#define SPI0_CLK_DIV_BY_4           0x1001
#define SPI0_CLK_DIV_BY_32          0x100f

/*****************************************************************************/

/*
 * Allwinner A10/A20 SoCs were using pins PC0,PC1,PC2,PC23 for booting
 * from SPI Flash, everything else is using pins PC0,PC1,PC2,PC3.
 * The H6 uses PC0, PC2, PC3, PC5, the H616 PC0, PC2, PC3, PC4.
 */
static void spi0_pinmux_setup(unsigned int pin_function)
{
	if (IS_ENABLED(CONFIG_MACH_SUN60I_A733)) {
		if (pin_function == SUNXI_GPIO_DISABLE) {
			sunxi_gpio_set_cfgpin(SUNXI_GPC(2), SUNXI_GPIO_DISABLE);
			sunxi_gpio_set_cfgpin(SUNXI_GPC(3), SUNXI_GPIO_DISABLE);
			sunxi_gpio_set_cfgpin(SUNXI_GPC(4), SUNXI_GPIO_DISABLE);
			sunxi_gpio_set_cfgpin(SUNXI_GPC(12), SUNXI_GPIO_DISABLE);
		} else {
			sunxi_gpio_set_cfgpin(SUNXI_GPC(2), 5);
			sunxi_gpio_set_cfgpin(SUNXI_GPC(3), 5);
			sunxi_gpio_set_cfgpin(SUNXI_GPC(4), 5);
			sunxi_gpio_set_cfgpin(SUNXI_GPC(12), 5);
			sunxi_gpio_set_pull(SUNXI_GPC(3), SUNXI_GPIO_PULL_UP);
		}
		return;
	}

	/* All chips use PC2. And all chips use PC0, except R528/T113 */
	if (!IS_ENABLED(CONFIG_MACH_SUN8I_R528))
		sunxi_gpio_set_cfgpin(SUNXI_GPC(0), pin_function);

	sunxi_gpio_set_cfgpin(SUNXI_GPC(2), pin_function);

	/* All chips except H6/H616/R528/T113 use PC1. */
	if (!IS_ENABLED(CONFIG_SUN50I_GEN_H6) &&
	    !IS_ENABLED(CONFIG_MACH_SUN8I_R528))
		sunxi_gpio_set_cfgpin(SUNXI_GPC(1), pin_function);

	if (IS_ENABLED(CONFIG_MACH_SUN50I_H6) ||
	    IS_ENABLED(CONFIG_MACH_SUN8I_R528))
		sunxi_gpio_set_cfgpin(SUNXI_GPC(5), pin_function);
	if (IS_ENABLED(CONFIG_MACH_SUN50I_H616) ||
	    IS_ENABLED(CONFIG_MACH_SUN8I_R528))
		sunxi_gpio_set_cfgpin(SUNXI_GPC(4), pin_function);

	/* Older generations use PC23 for CS, newer ones use PC3. */
	if (IS_ENABLED(CONFIG_MACH_SUN4I) || IS_ENABLED(CONFIG_MACH_SUN7I) ||
	    IS_ENABLED(CONFIG_MACH_SUN8I_R40))
		sunxi_gpio_set_cfgpin(SUNXI_GPC(23), pin_function);
	else
		sunxi_gpio_set_cfgpin(SUNXI_GPC(3), pin_function);
}

static bool is_sun6i_gen_spi(void)
{
	return IS_ENABLED(CONFIG_SUNXI_GEN_SUN6I) ||
	       IS_ENABLED(CONFIG_SUN50I_GEN_H6) ||
	       IS_ENABLED(CONFIG_SUNXI_GEN_NCAT2) ||
	       IS_ENABLED(CONFIG_MACH_SUN60I_A733) ||
	       IS_ENABLED(CONFIG_MACH_SUN8I_V3S);
}

static uintptr_t spi0_base_address(void)
{
	if (IS_ENABLED(CONFIG_MACH_SUN60I_A733))
		return 0x02540000;

	if (IS_ENABLED(CONFIG_MACH_SUN8I_R40))
		return 0x01C05000;

	if (IS_ENABLED(CONFIG_SUN50I_GEN_H6))
		return 0x05010000;

	if (IS_ENABLED(CONFIG_SUNXI_GEN_NCAT2))
		return 0x04025000;

	if (!is_sun6i_gen_spi() ||
	    IS_ENABLED(CONFIG_MACH_SUNIV))
		return 0x01C05000;

	return 0x01C68000;
}

static void sunxi_spi0_set_cs(uintptr_t base, bool enable)
{
	if (is_sun6i_gen_spi()) {
		if (enable)
			clrsetbits_le32(base + SUN6I_SPI0_TCR, SUN6I_TCR_CS_LEVEL,
					SUN6I_TCR_CS_MANUAL | SUN6I_TCR_CS_ACTIVE_LOW);
		else
			setbits_le32(base + SUN6I_SPI0_TCR,
				     SUN6I_TCR_CS_LEVEL | SUN6I_TCR_CS_MANUAL | SUN6I_TCR_CS_ACTIVE_LOW);
		udelay(1);
	}
}

static void sunxi_spi0_reset_fifo(uintptr_t base)
{
	if (is_sun6i_gen_spi()) {
		setbits_le32(base + SUN6I_SPI0_FIFO_CTL, SUN6I_FIFO_CTL_RF_RST | SUN6I_FIFO_CTL_TF_RST);
		int to = 1000;
		while ((readl(base + SUN6I_SPI0_FIFO_CTL) & (SUN6I_FIFO_CTL_RF_RST | SUN6I_FIFO_CTL_TF_RST)) && --to)
			udelay(1);
	}
}

/*
 * Setup 6 MHz from OSC24M (because the BROM is doing the same).
 */
static void spi0_enable_clock(void)
{
	uintptr_t base = spi0_base_address();

	/* Deassert SPI0 reset on SUN6I */
	if (IS_ENABLED(CONFIG_SUN50I_GEN_H6) ||
	    IS_ENABLED(CONFIG_SUNXI_GEN_NCAT2))
		setbits_le32(CCM_H6_SPI_BGR_REG, (1U << 16) | 0x1);
	else if (is_sun6i_gen_spi())
		setbits_le32(SUN6I_BUS_SOFT_RST_REG0,
			     (1 << AHB_RESET_SPI0_SHIFT));

	/* Open the SPI0 gate */
	if (!IS_ENABLED(CONFIG_SUN50I_GEN_H6) &&
	    !IS_ENABLED(CONFIG_SUNXI_GEN_NCAT2))
		setbits_le32(CCM_AHB_GATING0, (1 << AHB_GATE_OFFSET_SPI0));

	if (IS_ENABLED(CONFIG_MACH_SUNIV)) {
		/* Divide by 32, clock source is AHB clock 200MHz */
		writel(SPI0_CLK_DIV_BY_32, base + SUN6I_SPI0_CCTL);
	} else {
		/* New SoCs do not have a clock divider inside */
		if (!IS_ENABLED(CONFIG_SUNXI_GEN_NCAT2)) {
			/* Divide by 4 */
			writel(SPI0_CLK_DIV_BY_4,
			       base + (is_sun6i_gen_spi() ? SUN6I_SPI0_CCTL :
			       SUN4I_SPI0_CCTL));
		}

		/* 6MHz from OSC24M for A733 (div=3), 24MHz for others */
		if (IS_ENABLED(CONFIG_MACH_SUN60I_A733))
			writel((1 << 31) | 3, CCM_SPI0_CLK);
		else
			writel((1 << 31), CCM_SPI0_CLK);
	}

	if (is_sun6i_gen_spi()) {
		/* Enable SPI in the master mode and do a soft reset */
		setbits_le32(base + SUN6I_SPI0_GCR, SUN6I_CTL_MASTER |
			     SUN6I_CTL_ENABLE | SUN6I_CTL_SRST);
		/* Wait for completion */
		while (readl(base + SUN6I_SPI0_GCR) & SUN6I_CTL_SRST)
			;

		/*
		 * Set manual CS mode, active low, CS_LEVEL high (deselected).
		 * NCAT2 uses SDM; A733 operates reliably at 6MHz without SDM.
		 */
		u32 tcr = SUN6I_TCR_CS_MANUAL | SUN6I_TCR_CS_ACTIVE_LOW | SUN6I_TCR_CS_LEVEL;
		if (IS_ENABLED(CONFIG_SUNXI_GEN_NCAT2) && !IS_ENABLED(CONFIG_MACH_SUN60I_A733))
			tcr |= SUN6I_TCR_SDM;
		clrsetbits_le32(base + SUN6I_SPI0_TCR, BIT(11), tcr);
	} else {
		/* Enable SPI in the master mode and reset FIFO */
		setbits_le32(base + SUN4I_SPI0_CTL, SUN4I_CTL_MASTER |
						    SUN4I_CTL_ENABLE |
						    SUN4I_CTL_TF_RST |
						    SUN4I_CTL_RF_RST);
	}
}

static void spi0_disable_clock(void)
{
	uintptr_t base = spi0_base_address();

	/* Disable the SPI0 controller */
	if (is_sun6i_gen_spi())
		clrbits_le32(base + SUN6I_SPI0_GCR, SUN6I_CTL_MASTER |
					     SUN6I_CTL_ENABLE);
	else
		clrbits_le32(base + SUN4I_SPI0_CTL, SUN4I_CTL_MASTER |
					     SUN4I_CTL_ENABLE);

	/* Disable the SPI0 clock */
	if (!IS_ENABLED(CONFIG_MACH_SUNIV))
		writel(0, CCM_SPI0_CLK);

	/* Close the SPI0 gate */
	if (!IS_ENABLED(CONFIG_SUN50I_GEN_H6) &&
	    !IS_ENABLED(CONFIG_SUNXI_GEN_NCAT2))
		clrbits_le32(CCM_AHB_GATING0, (1 << AHB_GATE_OFFSET_SPI0));

	/* Assert SPI0 reset on SUN6I */
	if (IS_ENABLED(CONFIG_SUN50I_GEN_H6) ||
	    IS_ENABLED(CONFIG_SUNXI_GEN_NCAT2))
		clrbits_le32(CCM_H6_SPI_BGR_REG, (1U << 16) | 0x1);
	else if (is_sun6i_gen_spi())
		clrbits_le32(SUN6I_BUS_SOFT_RST_REG0,
			     (1 << AHB_RESET_SPI0_SHIFT));
}

static u8 sunxi_spi0_read_status(void);
static void sunxi_spi0_check_protection(void);

void spi0_init(void)
{
	unsigned int pin_function = SUNXI_GPC_SPI0;

	if (IS_ENABLED(CONFIG_MACH_SUN60I_A733))
		pin_function = SUN60I_GPC_SPI0;
	else if (IS_ENABLED(CONFIG_MACH_SUN50I) ||
	    IS_ENABLED(CONFIG_SUN50I_GEN_H6))
		pin_function = SUN50I_GPC_SPI0;
	else if (IS_ENABLED(CONFIG_MACH_SUNIV) ||
		 IS_ENABLED(CONFIG_MACH_SUN8I_R528))
		pin_function = SUNIV_GPC_SPI0;

	spi0_pinmux_setup(pin_function);
	spi0_enable_clock();
	if (IS_ENABLED(CONFIG_MACH_SUN60I_A733) &&
	    IS_ENABLED(CONFIG_SUNXI_PMU_TRAINING_CACHE))
		sunxi_spi0_check_protection();
	debug("SPI0: init done, status=0x%02x\n", sunxi_spi0_read_status());
}

void spi0_deinit(void)
{
	/* New SoCs can disable pins, older could only set them as input */
	unsigned int pin_function = SUNXI_GPIO_INPUT;

	if (is_sun6i_gen_spi()) {
		pin_function = SUNXI_GPIO_DISABLE;
		sunxi_spi0_set_cs(spi0_base_address(), false);
	}

	spi0_disable_clock();
	spi0_pinmux_setup(pin_function);
}

/*****************************************************************************/

#define SPI_READ_MAX_SIZE 60 /* FIFO size, minus 4 bytes of the header */

static void sunxi_spi0_read_data(u8 *buf, u32 addr, u32 bufsize,
				 ulong spi_ctl_reg,
				 ulong spi_ctl_xch_bitmask,
				 ulong spi_fifo_reg,
				 ulong spi_tx_reg,
				 ulong spi_rx_reg,
				 ulong spi_bc_reg,
				 ulong spi_tc_reg,
				 ulong spi_bcc_reg)
{
	writel(4 + bufsize, spi_bc_reg); /* Burst counter (total bytes) */
	writel(4, spi_tc_reg);           /* Transfer counter (bytes to send) */
	if (spi_bcc_reg)
		writel(4, spi_bcc_reg);  /* SUN6I also needs this */

	/* Send the Read Data Bytes (03h) command header */
	writeb(0x03, spi_tx_reg);
	writeb((u8)(addr >> 16), spi_tx_reg);
	writeb((u8)(addr >> 8), spi_tx_reg);
	writeb((u8)(addr), spi_tx_reg);

	/* Start the data transfer */
	setbits_le32(spi_ctl_reg, spi_ctl_xch_bitmask);

	/* Wait until everything is received in the RX FIFO */
	while ((readl(spi_fifo_reg) & 0x7F) < 4 + bufsize)
		;

	/* Skip 4 bytes */
	readl(spi_rx_reg);

	/* Read the data */
	while (bufsize-- > 0)
		*buf++ = readb(spi_rx_reg);

	/* tSHSL time is up to 100 ns in various SPI flash datasheets */
	udelay(1);
}

void spi0_read_data(void *buf, u32 addr, u32 len)
{
	u8 *buf8 = buf;
	u32 chunk_len;
	uintptr_t base = spi0_base_address();

	while (len > 0) {
		chunk_len = len;
		if (chunk_len > SPI_READ_MAX_SIZE)
			chunk_len = SPI_READ_MAX_SIZE;

		if (is_sun6i_gen_spi()) {
			sunxi_spi0_reset_fifo(base);
			sunxi_spi0_set_cs(base, true);
			sunxi_spi0_read_data(buf8, addr, chunk_len,
					     base + SUN6I_SPI0_TCR,
					     SUN6I_TCR_XCH,
					     base + SUN6I_SPI0_FIFO_STA,
					     base + SUN6I_SPI0_TXD,
					     base + SUN6I_SPI0_RXD,
					     base + SUN6I_SPI0_MBC,
					     base + SUN6I_SPI0_MTC,
					     base + SUN6I_SPI0_BCC);
			sunxi_spi0_set_cs(base, false);
		} else {
			sunxi_spi0_read_data(buf8, addr, chunk_len,
					     base + SUN4I_SPI0_CTL,
					     SUN4I_CTL_XCH,
					     base + SUN4I_SPI0_FIFO_STA,
					     base + SUN4I_SPI0_TX,
					     base + SUN4I_SPI0_RX,
					     base + SUN4I_SPI0_BC,
					     base + SUN4I_SPI0_TC,
					     0);
		}

		len  -= chunk_len;
		buf8 += chunk_len;
		addr += chunk_len;
	}
}

static void sunxi_spi0_xfer_cmd(u8 cmd)
{
	uintptr_t base = spi0_base_address();

	if (is_sun6i_gen_spi()) {
		int to = 100000;
		sunxi_spi0_reset_fifo(base);
		sunxi_spi0_set_cs(base, true);
		writel(1, base + SUN6I_SPI0_MBC);
		writel(1, base + SUN6I_SPI0_MTC);
		writel(1, base + SUN6I_SPI0_BCC);
		writeb(cmd, base + SUN6I_SPI0_TXD);
		setbits_le32(base + SUN6I_SPI0_TCR, SUN6I_TCR_XCH);
		while ((readl(base + SUN6I_SPI0_TCR) & SUN6I_TCR_XCH) && --to)
			udelay(1);
		to = 100000;
		while (((readl(base + SUN6I_SPI0_FIFO_STA) & 0x7f) < 1) && --to)
			udelay(1);
		sunxi_spi0_set_cs(base, false);
		sunxi_spi0_reset_fifo(base);
	}
	udelay(1);
}

static u8 sunxi_spi0_read_status(void)
{
	uintptr_t base = spi0_base_address();
	u8 status = 0;

	if (is_sun6i_gen_spi()) {
		int to = 100000;
		sunxi_spi0_reset_fifo(base);
		sunxi_spi0_set_cs(base, true);
		writel(2, base + SUN6I_SPI0_MBC);
		writel(1, base + SUN6I_SPI0_MTC);
		writel(1, base + SUN6I_SPI0_BCC);
		writeb(0x05, base + SUN6I_SPI0_TXD);
		setbits_le32(base + SUN6I_SPI0_TCR, SUN6I_TCR_XCH);
		while ((readl(base + SUN6I_SPI0_TCR) & SUN6I_TCR_XCH) && --to)
			udelay(1);
		to = 100000;
		while (((readl(base + SUN6I_SPI0_FIFO_STA) & 0x7f) < 2) && --to)
			udelay(1);
		if (to) {
			readb(base + SUN6I_SPI0_RXD);
			status = readb(base + SUN6I_SPI0_RXD);
		}
		sunxi_spi0_set_cs(base, false);
		sunxi_spi0_reset_fifo(base);
	}
	udelay(1);
	return status;
}

static int sunxi_spi0_wait_ready(void)
{
	int to = 200000;
	while ((sunxi_spi0_read_status() & 1) && --to)
		udelay(10);
	return to ? 0 : -ETIMEDOUT;
}

static void sunxi_spi0_check_protection(void)
{
	uintptr_t base = spi0_base_address();
	int to;
	u8 s1, s2;

	if (!is_sun6i_gen_spi())
		return;

	/* Read Status 1 (0x05) */
	s1 = sunxi_spi0_read_status();

	/* If BP0, BP1, BP2 bits are set (bits 2..5), clear flash write protection */
	if (s1 & 0x3c) {
		/* Read Status 2 (0x35) */
		sunxi_spi0_reset_fifo(base);
		sunxi_spi0_set_cs(base, true);
		writel(2, base + SUN6I_SPI0_MBC);
		writel(1, base + SUN6I_SPI0_MTC);
		writel(1, base + SUN6I_SPI0_BCC);
		writeb(0x35, base + SUN6I_SPI0_TXD);
		setbits_le32(base + SUN6I_SPI0_TCR, SUN6I_TCR_XCH);
		to = 100000;
		while ((readl(base + SUN6I_SPI0_TCR) & SUN6I_TCR_XCH) && --to)
			udelay(1);
		to = 100000;
		while (((readl(base + SUN6I_SPI0_FIFO_STA) & 0x7f) < 2) && --to)
			udelay(1);
		readb(base + SUN6I_SPI0_RXD);
		s2 = readb(base + SUN6I_SPI0_RXD);
		sunxi_spi0_set_cs(base, false);
		sunxi_spi0_reset_fifo(base);

		/* Clear BP bits while preserving Status 2 */
		sunxi_spi0_xfer_cmd(0x06); /* WREN */
		sunxi_spi0_reset_fifo(base);
		sunxi_spi0_set_cs(base, true);
		writel(3, base + SUN6I_SPI0_MBC);
		writel(3, base + SUN6I_SPI0_MTC);
		writel(3, base + SUN6I_SPI0_BCC);
		writeb(0x01, base + SUN6I_SPI0_TXD); /* Write Status */
		writeb(0x00, base + SUN6I_SPI0_TXD); /* Status 1 = 0x00 (clear BP bits) */
		writeb(s2, base + SUN6I_SPI0_TXD);   /* Preserve Status 2 (QE) */
		setbits_le32(base + SUN6I_SPI0_TCR, SUN6I_TCR_XCH);
		to = 100000;
		while ((readl(base + SUN6I_SPI0_TCR) & SUN6I_TCR_XCH) && --to)
			udelay(1);
		sunxi_spi0_set_cs(base, false);
		sunxi_spi0_reset_fifo(base);
		sunxi_spi0_wait_ready();
	}
}

int spi0_erase_sector(u32 addr)
{
	uintptr_t base = spi0_base_address();
	int to = 100000;

	if (!is_sun6i_gen_spi())
		return -ENOSYS;

	sunxi_spi0_wait_ready();
	sunxi_spi0_xfer_cmd(0x06); /* WREN */

	sunxi_spi0_reset_fifo(base);
	sunxi_spi0_set_cs(base, true);
	writel(4, base + SUN6I_SPI0_MBC);
	writel(4, base + SUN6I_SPI0_MTC);
	writel(4, base + SUN6I_SPI0_BCC);
	writeb(0x20, base + SUN6I_SPI0_TXD); /* 4KB Sector Erase */
	writeb((u8)(addr >> 16), base + SUN6I_SPI0_TXD);
	writeb((u8)(addr >> 8), base + SUN6I_SPI0_TXD);
	writeb((u8)(addr), base + SUN6I_SPI0_TXD);
	setbits_le32(base + SUN6I_SPI0_TCR, SUN6I_TCR_XCH);
	while ((readl(base + SUN6I_SPI0_TCR) & SUN6I_TCR_XCH) && --to)
		udelay(1);
	to = 100000;
	while (((readl(base + SUN6I_SPI0_FIFO_STA) & 0x7f) < 4) && --to)
		udelay(1);
	sunxi_spi0_set_cs(base, false);
	sunxi_spi0_reset_fifo(base);
	udelay(1);

	return sunxi_spi0_wait_ready();
}

int spi0_write_data(u32 addr, const void *buf, u32 len)
{
	uintptr_t base = spi0_base_address();
	const u8 *p = buf;

	if (!is_sun6i_gen_spi())
		return -ENOSYS;

	while (len > 0) {
		int to = 100000;
		u32 page_offset = addr & 0xff;
		u32 chunk = 256 - page_offset;
		if (chunk > len)
			chunk = len;
		if (chunk > 60)
			chunk = 60; /* FIFO limit */

		sunxi_spi0_wait_ready();
		sunxi_spi0_xfer_cmd(0x06); /* WREN */

		sunxi_spi0_reset_fifo(base);
		sunxi_spi0_set_cs(base, true);
		writel(4 + chunk, base + SUN6I_SPI0_MBC);
		writel(4 + chunk, base + SUN6I_SPI0_MTC);
		writel(4 + chunk, base + SUN6I_SPI0_BCC);
		writeb(0x02, base + SUN6I_SPI0_TXD); /* Page Program */
		writeb((u8)(addr >> 16), base + SUN6I_SPI0_TXD);
		writeb((u8)(addr >> 8), base + SUN6I_SPI0_TXD);
		writeb((u8)(addr), base + SUN6I_SPI0_TXD);
		for (u32 i = 0; i < chunk; i++)
			writeb(*p++, base + SUN6I_SPI0_TXD);

		setbits_le32(base + SUN6I_SPI0_TCR, SUN6I_TCR_XCH);
		while ((readl(base + SUN6I_SPI0_TCR) & SUN6I_TCR_XCH) && --to)
			udelay(1);
		to = 100000;
		while (((readl(base + SUN6I_SPI0_FIFO_STA) & 0x7f) < (4 + chunk)) && --to)
			udelay(1);
		sunxi_spi0_set_cs(base, false);
		sunxi_spi0_reset_fifo(base);
		udelay(1);

		if (sunxi_spi0_wait_ready())
			return -ETIMEDOUT;

		addr += chunk;
		len -= chunk;
	}

	return 0;
}

static ulong spi_load_read(struct spl_load_info *load, ulong sector,
			   ulong count, void *buf)
{
	spi0_read_data(buf, sector, count);

	return count;
}

/*****************************************************************************/

static int spl_spi_load_image(struct spl_image_info *spl_image,
			      struct spl_boot_device *bootdev)
{
	int ret = 0;
	struct legacy_img_hdr *header;
	uint32_t load_offset = sunxi_get_spl_size();

	header = (struct legacy_img_hdr *)CONFIG_TEXT_BASE;
	load_offset = max_t(uint32_t, load_offset, CONFIG_SYS_SPI_U_BOOT_OFFS);

	spi0_init();

	spi0_read_data((void *)header, load_offset, 0x40);

	if (IS_ENABLED(CONFIG_SPL_LOAD_FIT) &&
	    image_get_magic(header) == FDT_MAGIC) {
		struct spl_load_info load;

		debug("Found FIT image\n");
		spl_load_init(&load, spi_load_read, NULL, 1);
		ret = spl_load_simple_fit(spl_image, &load,
					  load_offset, header);
	} else {
		ret = spl_parse_image_header(spl_image, bootdev, header);
		if (ret)
			return ret;

		spi0_read_data((void *)spl_image->load_addr,
			       load_offset, spl_image->size);
	}

	spi0_deinit();

	return ret;
}

/* Use priorty 0 to override the default if it happens to be linked in */
SPL_LOAD_IMAGE_METHOD("sunxi SPI", 0, BOOT_DEVICE_SPI, spl_spi_load_image);

