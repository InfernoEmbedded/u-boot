// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2026 Antigravity
 * Allwinner Crypto Engine (CE) TRNG driver for U-Boot (UCLASS_RNG)
 */

#include <errno.h>
#include <log.h>
#include <malloc.h>
#include <linux/types.h>
#include <clk.h>
#include <dm.h>
#include <rng.h>
#include <reset.h>
#include <asm/cache.h>
#include <asm/io.h>
#include <linux/delay.h>
#include <time.h>

#define CE_TDQ			0x00
#define CE_CTR			0x04
#define CE_ICR			0x08
#define CE_ISR			0x0c
#define CE_TLR			0x10
#define CE_TSR			0x14
#define CE_ESR			0x18

#define CE_ALG_TRNG_V2		0x1c
#define CE_COMM_INT		BIT(31)

struct ce_task {
	u32 t_id;
	u32 t_common_ctl;
	u32 t_sym_ctl;
	u32 t_asym_ctl;
	u32 t_key;
	u32 t_iv;
	u32 t_ctr;
	u32 t_dlen;
	u32 src_addr;
	u32 src_len;
	u32 dst_addr;
	u32 dst_len;
	u32 next;
	u32 reserved[3];
} __packed __aligned(ARCH_DMA_MINALIGN);

struct sun8i_ce_rng_priv {
	void __iomem *base;
	struct clk bus_clk;
	struct clk mod_clk;
	struct reset_ctl reset;
	struct ce_task *task;
};

static int sun8i_ce_rng_read(struct udevice *dev, void *data, size_t len)
{
	struct sun8i_ce_rng_priv *priv = dev_get_priv(dev);
	u8 *dst = data;
	static u8 rng_buf[ARCH_DMA_MINALIGN] __aligned(ARCH_DMA_MINALIGN);

	while (len > 0) {
		size_t chunk = min_t(size_t, len, 32);
		ulong start;

		memset(priv->task, 0, sizeof(*priv->task));
		priv->task->t_id = 3; /* Flow 3 */
		priv->task->t_common_ctl = CE_ALG_TRNG_V2 | CE_COMM_INT;
		priv->task->t_dlen = 32; /* 32 bytes */
		priv->task->dst_addr = (u32)(uintptr_t)rng_buf;
		priv->task->dst_len = 32 / 4; /* word count */

		/* Flush task descriptor cache */
		flush_dcache_range((uintptr_t)priv->task,
				   (uintptr_t)priv->task +
				   ALIGN(sizeof(*priv->task), ARCH_DMA_MINALIGN));
		invalidate_dcache_range((uintptr_t)rng_buf,
					(uintptr_t)rng_buf + sizeof(rng_buf));

		/* Clear Flow 3 interrupt status */
		writel(BIT(3), priv->base + CE_ISR);

		/* Set task descriptor queue address */
		writel((u32)(uintptr_t)priv->task, priv->base + CE_TDQ);

		/* Enable task: TLR = 1 | (common_ctl & 0x7f) << 8 */
		writel(1 | ((CE_ALG_TRNG_V2 & 0x7f) << 8), priv->base + CE_TLR);

		/* Poll for completion */
		start = get_timer(0);
		while (!(readl(priv->base + CE_ISR) & BIT(3))) {
			if (get_timer(start) > 50) /* 50ms timeout */
				return -ETIMEDOUT;
			udelay(10);
		}

		/* Clear interrupt */
		writel(BIT(3), priv->base + CE_ISR);

		invalidate_dcache_range((uintptr_t)rng_buf,
					(uintptr_t)rng_buf + sizeof(rng_buf));

		memcpy(dst, rng_buf, chunk);
		dst += chunk;
		len -= chunk;
	}

	return 0;
}

static int sun8i_ce_rng_probe(struct udevice *dev)
{
	struct sun8i_ce_rng_priv *priv = dev_get_priv(dev);
	int ret;

	priv->base = dev_read_addr_ptr(dev);
	if (!priv->base)
		return -EINVAL;

	priv->task = memalign(ARCH_DMA_MINALIGN,
			      ALIGN(sizeof(struct ce_task), ARCH_DMA_MINALIGN));
	if (!priv->task)
		return -ENOMEM;

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

static int sun8i_ce_rng_remove(struct udevice *dev)
{
	struct sun8i_ce_rng_priv *priv = dev_get_priv(dev);

	free(priv->task);
	return 0;
}

static const struct dm_rng_ops sun8i_ce_rng_ops = {
	.read = sun8i_ce_rng_read,
};

static const struct udevice_id sun8i_ce_rng_ids[] = {
	{ .compatible = "allwinner,sun60i-a733-crypto" },
	{ .compatible = "allwinner,sun55i-a523-crypto" },
	{ .compatible = "allwinner,sun50i-h616-crypto" },
	{ }
};

U_BOOT_DRIVER(sun8i_ce_rng) = {
	.name      = "sun8i_ce_rng",
	.id        = UCLASS_RNG,
	.of_match  = sun8i_ce_rng_ids,
	.ops       = &sun8i_ce_rng_ops,
	.probe     = sun8i_ce_rng_probe,
	.remove    = sun8i_ce_rng_remove,
	.priv_auto = sizeof(struct sun8i_ce_rng_priv),
};
