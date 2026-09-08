// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2026 Antigravity
 * Allwinner GPADC driver for U-Boot (UCLASS_ADC)
 */

#include <errno.h>
#include <log.h>
#include <linux/types.h>
#include <adc.h>
#include <clk.h>
#include <dm.h>
#include <reset.h>
#include <asm/io.h>
#include <linux/delay.h>

#define SUN20I_GPADC_SR			0x00
#define SUN20I_GPADC_CTRL		0x04
#define SUN20I_GPADC_CS_EN		0x08
#define SUN20I_GPADC_DATA_INTC		0x28
#define SUN20I_GPADC_DATA_INTS		0x38
#define SUN20I_GPADC_CH_DATA(x)		(0x80 + (x) * 4)

#define SUN20I_GPADC_CTRL_ADC_EN	BIT(16)
#define SUN20I_GPADC_CH_NUM		8

struct sun20i_gpadc_priv {
	void __iomem *base;
	struct clk bus_clk;
	struct reset_ctl reset;
	int active_channel;
};

static int sun20i_gpadc_start_channel(struct udevice *dev, int channel)
{
	struct sun20i_gpadc_priv *priv = dev_get_priv(dev);
	u32 val;

	if (channel < 0 || channel >= SUN20I_GPADC_CH_NUM)
		return -EINVAL;

	priv->active_channel = channel;

	/* Enable channel */
	writel(BIT(channel), priv->base + SUN20I_GPADC_CS_EN);

	/* Enable ADC conversion */
	val = readl(priv->base + SUN20I_GPADC_CTRL);
	val |= SUN20I_GPADC_CTRL_ADC_EN;
	writel(val, priv->base + SUN20I_GPADC_CTRL);

	return 0;
}

static int sun20i_gpadc_channel_data(struct udevice *dev, int channel, unsigned int *data)
{
	struct sun20i_gpadc_priv *priv = dev_get_priv(dev);
	ulong start;

	if (channel < 0 || channel >= SUN20I_GPADC_CH_NUM)
		return -EINVAL;

	/* Poll for data ready interrupt status */
	start = get_timer(0);
	while (!(readl(priv->base + SUN20I_GPADC_DATA_INTS) & BIT(channel))) {
		if (get_timer(start) > 20) { /* 20ms timeout */
			/* Even if status flag not set, read current channel data */
			break;
		}
		udelay(10);
	}

	/* Clear status bit */
	writel(BIT(channel), priv->base + SUN20I_GPADC_DATA_INTS);

	/* Read 12-bit conversion value */
	*data = readl(priv->base + SUN20I_GPADC_CH_DATA(channel)) & 0xfff;

	return 0;
}

static int sun20i_gpadc_stop(struct udevice *dev)
{
	struct sun20i_gpadc_priv *priv = dev_get_priv(dev);

	writel(0, priv->base + SUN20I_GPADC_CS_EN);
	return 0;
}

static int sun20i_gpadc_of_to_plat(struct udevice *dev)
{
	struct adc_uclass_plat *uc_pdata = dev_get_uclass_plat(dev);

	uc_pdata->data_mask = 0xfff; /* 12 bits */
	uc_pdata->data_format = ADC_DATA_FORMAT_BIN;
	uc_pdata->data_timeout_us = 10000;
	uc_pdata->channel_mask = 0xff; /* 8 channels */

	return 0;
}

static int sun20i_gpadc_probe(struct udevice *dev)
{
	struct sun20i_gpadc_priv *priv = dev_get_priv(dev);
	int ret;

	priv->base = dev_read_addr_ptr(dev);
	if (!priv->base)
		return -EINVAL;

	ret = clk_get_by_index(dev, 0, &priv->bus_clk);
	if (!ret)
		clk_enable(&priv->bus_clk);

	ret = reset_get_by_index(dev, 0, &priv->reset);
	if (!ret)
		reset_deassert(&priv->reset);

	return 0;
}

static const struct adc_ops sun20i_gpadc_ops = {
	.start_channel = sun20i_gpadc_start_channel,
	.channel_data  = sun20i_gpadc_channel_data,
	.stop          = sun20i_gpadc_stop,
};

static const struct udevice_id sun20i_gpadc_ids[] = {
	{ .compatible = "allwinner,sun60i-a733-gpadc" },
	{ .compatible = "allwinner,sun55i-a523-gpadc" },
	{ .compatible = "allwinner,sun20i-d1-gpadc" },
	{ }
};

U_BOOT_DRIVER(sun20i_gpadc) = {
	.name       = "sun20i_gpadc",
	.id         = UCLASS_ADC,
	.of_match   = sun20i_gpadc_ids,
	.ops        = &sun20i_gpadc_ops,
	.of_to_plat = sun20i_gpadc_of_to_plat,
	.probe      = sun20i_gpadc_probe,
	.priv_auto  = sizeof(struct sun20i_gpadc_priv),
};
