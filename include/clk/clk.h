// SPDX-License-Identifier: GPL-2.0+
/*
 * Compatibility clock header for Allwinner DRM drivers
 */

#ifndef __SUNXI_CLK_CLK_H__
#define __SUNXI_CLK_CLK_H__

#include <clk.h>
#include <reset.h>
#include <malloc.h>
#include <dm/device.h>

static inline struct clk *sunxi_clk_get(struct udevice *dev, const char *name)
{
	struct clk *clk;
	int ret;

	clk = calloc(1, sizeof(*clk));
	if (!clk)
		return NULL;

	ret = clk_get_by_name(dev, name, clk);
	if (ret) {
		free(clk);
		return NULL;
	}
	return clk;
}

static inline int of_periph_clk_config_setup(int node_offset)
{
	return 0;
}

#endif /* __SUNXI_CLK_CLK_H__ */
