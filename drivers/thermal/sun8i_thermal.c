// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2026 Antigravity
 * Allwinner THS Thermal Sensor driver for U-Boot (UCLASS_THERMAL)
 */

#include <errno.h>
#include <log.h>
#include <linux/types.h>
#include <clk.h>
#include <dm.h>
#include <thermal.h>
#include <reset.h>
#include <asm/io.h>
#include <linux/delay.h>

#define SUN50I_H616_THS_CTRL0			0x00
#define SUN50I_H616_THS_ENABLE			0x04
#define SUN50I_H616_THS_PC			0x08
#define SUN50I_H616_THS_DATA_INTS		0x20
#define SUN50I_H616_THS_MFC			0x30
#define SUN50I_H616_THS_TEMP_CALIB		0xa0
#define SUN50I_H616_THS_TEMP_DATA		0xc0

#define SUN50I_THS_CTRL0_T_ACQ(x)		(GENMASK(15, 0) & (x))
#define SUN50I_THS_CTRL0_FS_DIV(x)		((GENMASK(15, 0) & (x)) << 16)
#define SUN50I_THS_FILTER_EN			BIT(2)
#define SUN50I_THS_FILTER_TYPE(x)		(GENMASK(1, 0) & (x))
#define SUN50I_H616_THS_PC_TEMP_PERIOD(x)	((GENMASK(19, 0) & (x)) << 12)

#define SUN60I_A733_DELIMITER		1769
#define SUN60I_A733_OFFSET_ABOVE	2835
#define SUN60I_A733_OFFSET_BELOW	2822
#define SUN60I_A733_SCALE_ABOVE		59
#define SUN60I_A733_SCALE_BELOW		62

struct sun8i_thermal_priv {
	void __iomem *base;
	struct clk bus_clk;
	struct clk gpadc_clk;
	struct reset_ctl reset;
};

static int sun8i_thermal_get_temp(struct udevice *dev, int *temp)
{
	struct sun8i_thermal_priv *priv = dev_get_priv(dev);
	u32 val = 0;
	u32 data_ints = 0;
	int timeout = 500;

	if (!priv->base)
		return -EINVAL;

	/* Sensor 0 is CPU Little: wait for conversion */
	do {
		val = readl(priv->base + SUN50I_H616_THS_TEMP_DATA) & 0xfff;
		data_ints = readl(priv->base + SUN50I_H616_THS_DATA_INTS);
		if ((data_ints & BIT(0)) && val != 0)
			break;
		udelay(1000);
	} while (--timeout);

	if (val == 0) {
		/* Fallback to sensor 1 or sensor 3 (CPU Big) */
		val = readl(priv->base + SUN50I_H616_THS_TEMP_DATA + 4) & 0xfff;
		if (val == 0)
			val = readl(priv->base + SUN50I_H616_THS_TEMP_DATA + 12) & 0xfff;
		if (val == 0) {
			log_err("thermal sensor conversion timeout (ints=0x%x)\n", data_ints);
			return -EAGAIN;
		}
	}

	if (val > SUN60I_A733_DELIMITER)
		*temp = SUN60I_A733_SCALE_BELOW * (SUN60I_A733_OFFSET_BELOW - (int)val);
	else
		*temp = SUN60I_A733_SCALE_ABOVE * (SUN60I_A733_OFFSET_ABOVE - (int)val);

	return 0;
}

static int sun8i_thermal_probe(struct udevice *dev)
{
	struct sun8i_thermal_priv *priv = dev_get_priv(dev);
	void __iomem *syscon;
	int ret;

	priv->base = dev_read_addr_ptr(dev);
	if (!priv->base)
		return -EINVAL;

	/* 1. Enable GPADC analog 24M clock */
	ret = clk_get_by_name(dev, "gpadc", &priv->gpadc_clk);
	if (!ret) {
		clk_enable(&priv->gpadc_clk);
	} else {
		ret = clk_get_by_index(dev, 1, &priv->gpadc_clk);
		if (!ret)
			clk_enable(&priv->gpadc_clk);
	}

	/* 2. Deassert reset */
	ret = reset_get_by_index(dev, 0, &priv->reset);
	if (!ret)
		reset_deassert(&priv->reset);

	/* 3. Enable bus clock */
	ret = clk_get_by_name(dev, "bus", &priv->bus_clk);
	if (!ret) {
		clk_enable(&priv->bus_clk);
	} else {
		ret = clk_get_by_index(dev, 0, &priv->bus_clk);
		if (!ret)
			clk_enable(&priv->bus_clk);
	}

	/* 4. Clear SRAM bit 16 at 0x03000000 */
	syscon = (void __iomem *)0x03000000;
	writel(readl(syscon) & ~BIT(16), syscon);

	/* 5. Initialize THS registers matching BSP sun60iw2_thermal_init */
	writel(SUN50I_THS_CTRL0_T_ACQ(47) | SUN50I_THS_CTRL0_FS_DIV(479),
	       priv->base + SUN50I_H616_THS_CTRL0);
	writel(SUN50I_THS_FILTER_EN | SUN50I_THS_FILTER_TYPE(1),
	       priv->base + SUN50I_H616_THS_MFC);
	writel(SUN50I_H616_THS_PC_TEMP_PERIOD(28),
	       priv->base + SUN50I_H616_THS_PC);
	writel(0x1f, priv->base + SUN50I_H616_THS_DATA_INTS);
	writel(0x1f, priv->base + SUN50I_H616_THS_ENABLE);

	return 0;
}

static const struct dm_thermal_ops sun8i_thermal_ops = {
	.get_temp = sun8i_thermal_get_temp,
};

static const struct udevice_id sun8i_thermal_ids[] = {
	{ .compatible = "allwinner,sun60i-a733-ths" },
	{ .compatible = "allwinner,sun60iw2p1-ths" },
	{ .compatible = "allwinner,sun50i-h616-ths" },
	{ }
};

U_BOOT_DRIVER(sun8i_thermal) = {
	.name      = "sun8i_thermal",
	.id        = UCLASS_THERMAL,
	.of_match  = sun8i_thermal_ids,
	.ops       = &sun8i_thermal_ops,
	.probe     = sun8i_thermal_probe,
	.priv_auto = sizeof(struct sun8i_thermal_priv),
};
