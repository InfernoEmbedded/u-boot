// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2026 Antigravity
 * Allwinner sun50i/sun60i Enhanced PWM driver for U-Boot (UCLASS_PWM)
 */

#include <errno.h>
#include <log.h>
#include <linux/types.h>
#include <clk.h>
#include <dm.h>
#include <div64.h>
#include <pwm.h>
#include <reset.h>
#include <asm/io.h>

#define PWM_PCCR01	0x0020
#define PWM_PCCR23	0x0024
#define PWM_PCCR45	0x0028
#define PWM_PCCR67	0x002c
#define PWM_PCCR8	0x0300
#define PWM_PER		0x0040

#define PWM_PCR_BASE	0x0060
#define PWM_PPR_BASE	0x0064

#define PWM_CLK_GATING_SHIFT	4
#define PWM_ACT_STA_SHIFT	8

struct sun50i_pwm_priv {
	void __iomem *base;
	struct clk bus_clk;
	struct clk mod_clk;
	struct reset_ctl reset;
	u32 pwm_base;
	u32 pwm_number;
};

static u32 get_pccr_offset(u32 ch)
{
	switch (ch) {
	case 0:
	case 1:
		return PWM_PCCR01;
	case 2:
	case 3:
		return PWM_PCCR23;
	case 4:
	case 5:
		return PWM_PCCR45;
	case 6:
	case 7:
		return PWM_PCCR67;
	case 8:
	case 9:
		return PWM_PCCR8;
	default:
		return PWM_PCCR01;
	}
}

static int sun50i_pwm_set_invert(struct udevice *dev, uint channel, bool polarity)
{
	struct sun50i_pwm_priv *priv = dev_get_priv(dev);
	u32 pcr_offset = PWM_PCR_BASE + 0x20 * channel;
	u32 val;

	val = readl(priv->base + pcr_offset);
	if (polarity)
		val &= ~BIT(PWM_ACT_STA_SHIFT);
	else
		val |= BIT(PWM_ACT_STA_SHIFT);
	writel(val, priv->base + pcr_offset);

	return 0;
}

static int sun50i_pwm_set_config(struct udevice *dev, uint channel,
				 uint period_ns, uint duty_ns)
{
	struct sun50i_pwm_priv *priv = dev_get_priv(dev);
	u32 pccr = get_pccr_offset(channel);
	u64 clk_hz = 24000000ULL;
	u64 entire_cycles, active_cycles;
	u32 prescale = 0;
	u32 val;

	if (period_ns == 0)
		return -EINVAL;

	/* Select 24MHz clock (clk_src = 0) */
	val = readl(priv->base + pccr);
	val &= ~(0x3 << 7); /* clk_src = 0 (24MHz) */
	val &= ~(0xf << 0); /* div_m = 0 (div 1) */
	writel(val, priv->base + pccr);

	/* Calculate total cycles = 24M * period_ns / 1e9 */
	entire_cycles = clk_hz * period_ns;
	do_div(entire_cycles, 1000000000ULL);

	/* Prescaler search if entire_cycles > 65536 */
	while (entire_cycles > 65536 && prescale < 255) {
		prescale++;
		entire_cycles = (clk_hz * period_ns) / (1000000000ULL * (prescale + 1));
	}

	if (entire_cycles == 0)
		entire_cycles = 1;

	active_cycles = entire_cycles * duty_ns;
	do_div(active_cycles, period_ns);

	/* Write prescale to PCR */
	val = readl(priv->base + PWM_PCR_BASE + 0x20 * channel);
	val &= ~0xff;
	val |= (prescale & 0xff);
	/* Default active high */
	val |= BIT(PWM_ACT_STA_SHIFT);
	writel(val, priv->base + PWM_PCR_BASE + 0x20 * channel);

	/* Write active and period cycles to PPR */
	writel(((entire_cycles - 1) << 16) | (active_cycles & 0xffff),
	       priv->base + PWM_PPR_BASE + 0x20 * channel);

	return 0;
}

static int sun50i_pwm_set_enable(struct udevice *dev, uint channel, bool enable)
{
	struct sun50i_pwm_priv *priv = dev_get_priv(dev);
	u32 pccr = get_pccr_offset(channel);
	u32 val;

	/* Enable clock gating in PCCR */
	val = readl(priv->base + pccr);
	if (enable)
		val |= BIT(PWM_CLK_GATING_SHIFT);
	writel(val, priv->base + pccr);

	/* Enable/disable channel in PER */
	val = readl(priv->base + PWM_PER);
	if (enable)
		val |= BIT(channel);
	else
		val &= ~BIT(channel);
	writel(val, priv->base + PWM_PER);

	return 0;
}

static int sun50i_pwm_probe(struct udevice *dev)
{
	struct sun50i_pwm_priv *priv = dev_get_priv(dev);
	int ret;

	priv->base = dev_read_addr_ptr(dev);
	if (!priv->base)
		return -EINVAL;

	dev_read_u32(dev, "pwm-base", &priv->pwm_base);
	dev_read_u32(dev, "pwm-number", &priv->pwm_number);

	ret = clk_get_by_name(dev, "bus", &priv->bus_clk);
	if (!ret)
		clk_enable(&priv->bus_clk);

	ret = clk_get_by_name(dev, "mod", &priv->mod_clk);
	if (!ret)
		clk_enable(&priv->mod_clk);

	ret = reset_get_by_index(dev, 0, &priv->reset);
	if (!ret)
		reset_deassert(&priv->reset);

	return 0;
}

static const struct pwm_ops sun50i_pwm_ops = {
	.set_invert = sun50i_pwm_set_invert,
	.set_config = sun50i_pwm_set_config,
	.set_enable = sun50i_pwm_set_enable,
};

static const struct udevice_id sun50i_pwm_ids[] = {
	{ .compatible = "allwinner,sun60i-a733-pwm" },
	{ .compatible = "allwinner,sun55i-a523-pwm" },
	{ .compatible = "allwinner,sun50i-h616-pwm" },
	{ }
};

U_BOOT_DRIVER(sun50i_pwm) = {
	.name       = "sun50i_pwm",
	.id         = UCLASS_PWM,
	.of_match   = sun50i_pwm_ids,
	.ops        = &sun50i_pwm_ops,
	.probe      = sun50i_pwm_probe,
	.priv_auto  = sizeof(struct sun50i_pwm_priv),
};
